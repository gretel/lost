// SPDX-License-Identifier: ISC
//
// lora_scan — LoRa spectral scanner (GR4 graph + orchestrator)
//
// A GR4 flowgraph handles continuous streaming from the SDR with proper
// backpressure, while an orchestrator thread drives the scan logic:
//   Layer 1: wideband FFT energy scan → candidate channels
//   Layer 2: per-channel CAD dwell with shared-buffer-per-BW detection
//
// Graph topology:
//   SoapySource(L1 rate) → Splitter(2)
//       → [0] SpectrumTapBlock → NullSink    (L1 energy via SpectrumState)
//       → [1] CaptureSink                     (L2 CAD on-demand capture)
//
// Run `lora_scan --help` for usage.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

// common.hpp must come AFTER Soapy.hpp (uses soapy types, no SoapySDR include)
#include "common.hpp"

#include <gnuradio-4.0/lora/CaptureSink.hpp>
#include <gnuradio-4.0/lora/SpectrumTapBlock.hpp>
#include <gnuradio-4.0/lora/Splitter.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/log.hpp>

#ifndef GIT_REV
#define GIT_REV "dev"
#endif

using lora_apps::cf32;

struct ScanStats {
    uint32_t    sweeps          = 0;
    uint32_t    overflows       = 0;
    uint32_t    drops           = 0;
    uint32_t    l1_hot          = 0;
    uint32_t    total_detections = 0;
    std::string last_det_freq;
    std::string last_det_time;
};

struct ScanConfig {
    std::string          device       = "uhd";
    std::string          device_param = "type=b200";
    double               freq_start   = 863.0e6;
    double               freq_stop    = 870.0e6;
    std::vector<double>  bws          = {62500.0, 125000.0, 250000.0};
    uint32_t             os_factor    = 4;
    double               gain         = 40.0;
    int                  settle_ms    = 5;
    double               l1_rate      = 16.0e6;
    double               master_clock = 32.0e6;
    float                min_ratio    = 8.0F;
    uint32_t             sweeps       = 0;
    bool                 layer1_only  = false;
    bool                 cbor_out     = false;

    [[nodiscard]] double min_bw() const { return *std::ranges::min_element(bws); }
};

// ─── CLI ──────────────────────────────────────────────────────────────────────

static void print_usage() {
    std::fprintf(stderr,
        "Usage: lora_scan [options]\n\n"
        "LoRa spectral scanner: sweep channels and detect chirp activity.\n\n"
        "Options:\n"
        "  --device <args>       SoapySDR device string (default: uhd)\n"
        "  --device-param <args> Device parameters (default: type=b200)\n"
        "  --freq-start <Hz>     Scan start frequency (default: 863e6)\n"
        "  --freq-stop <Hz>      Scan stop frequency (default: 870e6)\n"
        "  --bw <Hz[,Hz,...]>    Channel bandwidth(s) (default: 62500,125000,250000)\n"
        "  --os <n>              Oversampling factor (default: 4)\n"
        "  --gain <dB>           RX gain (default: 40)\n"
        "  --settle-ms <ms>      PLL settle delay after retune (default: 5)\n"
        "  --l1-rate <Hz>        L1 wideband sample rate (default: 16e6)\n"
        "  --master-clock <Hz>   FPGA master clock rate (default: 32e6)\n"
        "  --min-ratio <f>       Min peak ratio to report (default: 8.0)\n"
        "  --sweeps <N>          Stop after N sweeps (default: 0 = infinite)\n"
        "  --layer1-only         Stop after Layer 1 (energy scan only)\n"
        "  --cbor                Emit CBOR frames on stdout (pipe to lora_spectrum.py)\n"
        "  --log-level <level>   Log level: DEBUG, INFO, WARNING, ERROR\n"
        "  --version             Show version and exit\n"
        "  -h, --help            Show this help\n");
}

static int parseArgs(int argc, char** argv, ScanConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                std::exit(1);
            }
            return argv[i];
        };
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 2;
        }
        if (arg == "--version") {
            std::fprintf(stderr, "lora_scan %s\n", GIT_REV);
            return 2;
        }
        if (arg == "--log-level") {
            std::string lvl = next();
            for (auto& ch : lvl) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
            gr::lora::set_log_level(lvl);
            continue;
        }
        if      (arg == "--device")        cfg.device       = next();
        else if (arg == "--device-param")  cfg.device_param = next();
        else if (arg == "--freq-start")    cfg.freq_start   = std::stod(next());
        else if (arg == "--freq-stop")     cfg.freq_stop    = std::stod(next());
        else if (arg == "--bw") {
            cfg.bws.clear();
            std::istringstream ss(next());
            std::string tok;
            while (std::getline(ss, tok, ',')) {
                cfg.bws.push_back(std::stod(tok));
            }
            std::ranges::sort(cfg.bws);
        }
        else if (arg == "--os")            cfg.os_factor    = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--gain")          cfg.gain         = std::stod(next());
        else if (arg == "--settle-ms")     cfg.settle_ms    = std::stoi(next());
        else if (arg == "--l1-rate")       cfg.l1_rate      = std::stod(next());
        else if (arg == "--master-clock")  cfg.master_clock = std::stod(next());
        else if (arg == "--min-ratio")     cfg.min_ratio    = std::stof(next());
        else if (arg == "--sweeps")        cfg.sweeps       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--layer1-only")   cfg.layer1_only  = true;
        else if (arg == "--cbor")          cfg.cbor_out     = true;
        else {
            std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            print_usage();
            return 1;
        }
    }
    if (cfg.bws.empty()) {
        std::fprintf(stderr, "--bw requires at least one bandwidth\n");
        return 1;
    }
    // Validate that L1 rate / (bw * os) is an integer for every configured BW.
    // Software decimation requires an exact integer factor.
    for (const double bw : cfg.bws) {
        const double targetRate = bw * static_cast<double>(cfg.os_factor);
        const double factor     = cfg.l1_rate / targetRate;
        const double rounded    = std::round(factor);
        if (std::abs(factor - rounded) > 0.01 || rounded < 1.0) {
            std::fprintf(stderr,
                "L1 rate %.0f Hz / (BW %.0f Hz x os %u) = %.3f -- must be an integer\n",
                cfg.l1_rate, bw, cfg.os_factor, factor);
            return 1;
        }
    }
    return 0;
}

// ─── channel list ─────────────────────────────────────────────────────────────

static std::vector<double> makeChannelList(double freqStart, double freqStop, double bw) {
    std::vector<double> channels;
    for (double freq = freqStart; freq <= freqStop + bw * 0.01; freq += bw) {
        channels.push_back(freq);
    }
    return channels;
}

// ─── GR4 graph construction ──────────────────────────────────────────────────

struct ScanGraph {
    std::shared_ptr<gr::lora::SpectrumState> spectrum;
    std::shared_ptr<gr::lora::CaptureState>  capture;
};

static ScanGraph build_scan_graph(gr::Graph& graph, const ScanConfig& cfg,
                                  const std::vector<double>& channels) {
    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };
    using std::string_literals::operator""s;

    ScanGraph sg;

    // SoapySource -- fixed L1 rate, pinned master clock
    const double tileCentre = (channels.front() + channels.back()) / 2.0;
    auto& source = graph.emplaceBlock<
        gr::blocks::soapy::SoapySimpleSource<cf32>>({
        {"device",              cfg.device},
        {"device_parameter",    cfg.device_param},
        {"sample_rate",         cfg.l1_rate},
        {"master_clock_rate",   cfg.master_clock},
        {"rx_center_frequency", gr::Tensor<double>{tileCentre}},
        {"rx_gains",            gr::Tensor<double>{cfg.gain}},
        {"max_chunck_size",     static_cast<uint32_t>(512U << 4U)},
        {"max_overflow_count",  gr::Size_t{1000}},
        {"max_consecutive_errors", gr::Size_t{500}},
        {"max_time_out_us",     static_cast<uint32_t>(10'000U)},
        {"verbose_overflow",    false},
    });

    // Splitter -> 2 outputs
    auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
        {"n_outputs", gr::Size_t{2}},
    });
    if (!ok(graph.connect<"out", "in">(source, splitter))) {
        gr::lora::log_ts("error", "lora_scan", "connect source -> splitter failed");
    }

    // Path 0: SpectrumTap -> NullSink (L1 energy)
    sg.spectrum = std::make_shared<gr::lora::SpectrumState>();
    sg.spectrum->fft_size    = 4096;
    sg.spectrum->sample_rate = static_cast<float>(cfg.l1_rate);
    sg.spectrum->center_freq = tileCentre;
    sg.spectrum->init();

    auto& tap = graph.emplaceBlock<gr::lora::SpectrumTapBlock>({});
    tap._spectrum = sg.spectrum;

    auto& null_sink = graph.emplaceBlock<gr::testing::NullSink<cf32>>({});

    if (!ok(graph.connect(splitter, "out#0"s, tap, "in"s))) {
        gr::lora::log_ts("error", "lora_scan", "connect splitter -> tap failed");
    }
    if (!ok(graph.connect<"out", "in">(tap, null_sink))) {
        gr::lora::log_ts("error", "lora_scan", "connect tap -> null_sink failed");
    }

    // Path 1: CaptureSink (L2 on-demand)
    sg.capture = std::make_shared<gr::lora::CaptureState>();
    // Pre-allocate for worst case: SF12 / min BW at L1 rate
    const double minBw      = cfg.min_bw();
    const double targetRate  = minBw * static_cast<double>(cfg.os_factor);
    const auto   decFactor   = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));
    const uint32_t sf12Win   = (1U << 12U) * cfg.os_factor * 2U;
    sg.capture->buffer.resize(sf12Win * decFactor);

    auto& cap_sink = graph.emplaceBlock<gr::lora::CaptureSink>({});
    cap_sink._state = sg.capture;

    if (!ok(graph.connect(splitter, "out#1"s, cap_sink, "in"s))) {
        gr::lora::log_ts("error", "lora_scan", "connect splitter -> capture failed");
    }

    return sg;
}

// ─── Orchestrator helpers ────────────────────────────────────────────────────

/// Retune the SoapySource frequency via GR4 settings (no stream restart).
/// The graph keeps streaming during the settle delay -- no overflow.
static void retune_source(std::shared_ptr<gr::BlockModel>& source,
                          double freq, int settleMs) {
    gr::property_map props;
    props["rx_center_frequency"] = gr::Tensor<double>{freq};
    std::ignore = source->settings().set(props);
    std::this_thread::sleep_for(std::chrono::milliseconds(settleMs));
}

/// Update the SpectrumState centre frequency to match a retune.
static void update_spectrum_centre(std::shared_ptr<gr::lora::SpectrumState>& spectrum,
                                   double freq) {
    spectrum->center_freq = freq;
}

/// Trigger an on-demand capture via CaptureSink and wait for completion.
/// Returns a span over the captured data, or empty span on stop.
static std::span<const cf32> capture_samples(
        std::shared_ptr<gr::lora::CaptureState>& state,
        uint32_t nSamples,
        const std::atomic<bool>& stopFlag) {
    state->request(nSamples);
    while (!state->done.load(std::memory_order_acquire)) {
        if (stopFlag.load(std::memory_order_relaxed)) {
            state->reset();
            return {};
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    state->reset();
    return {state->buffer.data(), nSamples};
}

/// Decimate a wideband (L1-rate) capture to a narrowband target rate.
/// Uses a simple integrate-and-dump (box-car average) decimation filter.
/// Requires l1Rate / targetRate to be an integer.  Returns decimated buffer.
static std::vector<cf32> decimate(const cf32* src, uint32_t nSamples,
                                  double l1Rate, double targetRate) {
    const auto factor = static_cast<uint32_t>(std::round(l1Rate / targetRate));
    if (factor <= 1) {
        return std::vector<cf32>(src, src + nSamples);
    }
    const uint32_t outLen = nSamples / factor;
    std::vector<cf32> out(outLen);
    const float scale = 1.0F / static_cast<float>(factor);
    for (uint32_t i = 0; i < outLen; ++i) {
        cf32 sum{0.0F, 0.0F};
        for (uint32_t j = 0; j < factor; ++j) {
            sum += src[i * factor + j];
        }
        out[i] = sum * scale;
    }
    return out;
}

// ─── Layer 1: wideband FFT energy scan via SpectrumState ─────────────────────

struct Layer1Result {
    std::vector<double> energy;
    std::vector<int>    hot_idx;
};

/// Extract per-channel energy from a single SpectrumState FFT result.
/// The spectrum must already be tuned to the tile centre and have fresh data.
static void extractTileEnergy(
        std::shared_ptr<gr::lora::SpectrumState>& spectrum,
        const std::vector<double>& channels, double chBw,
        std::vector<double>& energy) {
    // Poll until a fresh FFT result is available
    while (!spectrum->compute()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Copy magnitude_db under lock
    std::vector<float> mag_db;
    {
        std::lock_guard lock(spectrum->result_mutex);
        mag_db = spectrum->magnitude_db;
    }

    const auto fftSize     = spectrum->fft_size;
    const double sampleRate = static_cast<double>(spectrum->sample_rate);
    const double centerFreq = spectrum->center_freq;
    const double hzPerBin   = sampleRate / static_cast<double>(fftSize);
    const double halfTile   = sampleRate / 2.0;

    for (std::size_t chIdx = 0; chIdx < channels.size(); ++chIdx) {
        const double chOffset = channels[chIdx] - centerFreq;
        if (std::abs(chOffset) > halfTile * 0.9) {
            continue;
        }

        // DC-centered FFT: bin 0 = -fs/2, bin fftSize/2 = DC
        const double binCentre = static_cast<double>(fftSize) / 2.0
                               + chOffset / hzPerBin;
        const double halfChBins = (chBw / sampleRate)
                                * static_cast<double>(fftSize) / 2.0;
        const int kLo = static_cast<int>(std::round(binCentre - halfChBins));
        const int kHi = static_cast<int>(std::round(binCentre + halfChBins));

        double sum = 0.0;
        int    cnt = 0;
        for (int k = kLo; k <= kHi; ++k) {
            if (k < 0 || k >= static_cast<int>(fftSize)) {
                continue;
            }
            // Convert dBFS back to linear power for summation
            sum += std::pow(10.0,
                static_cast<double>(mag_db[static_cast<std::size_t>(k)]) / 10.0);
            ++cnt;
        }
        if (cnt > 0) {
            const double snapEnergy = sum / static_cast<double>(cnt);
            energy[chIdx] = std::max(energy[chIdx], snapEnergy);
        }
    }
}

/// L1 scan using SpectrumState.  For single-tile configs, reads the spectrum
/// directly.  For multi-tile, retunes the SoapySource to each tile centre.
static Layer1Result layer1Scan(
        std::shared_ptr<gr::lora::SpectrumState>& spectrum,
        std::shared_ptr<gr::BlockModel>& soapy_source,
        const std::vector<double>& channels,
        const ScanConfig& cfg) {
    const std::size_t nCh = channels.size();
    Layer1Result result;
    result.energy.resize(nCh, 0.0);

    const double chBw      = cfg.min_bw();
    const double scanRange = channels.back() - channels.front() + chBw;
    const double usableBw  = cfg.l1_rate * 0.8;

    // Take multiple FFT snapshots per tile for stability
    constexpr int kL1Snapshots = 4;

    if (scanRange <= usableBw) {
        // Single tile: spectrum is already centred
        for (int snap = 0; snap < kL1Snapshots; ++snap) {
            extractTileEnergy(spectrum, channels, chBw, result.energy);
        }
    } else {
        // Multi-tile: retune to each tile centre
        const double step  = usableBw;
        const double first = channels.front() + usableBw / 2.0;
        const double last  = channels.back()  - usableBw / 2.0;
        for (double tc = first; tc <= last + step * 0.5; tc += step) {
            retune_source(soapy_source, tc, cfg.settle_ms);
            update_spectrum_centre(spectrum, tc);
            for (int snap = 0; snap < kL1Snapshots; ++snap) {
                extractTileEnergy(spectrum, channels, chBw, result.energy);
            }
        }
        // Return to default tile centre
        const double tileCentre = (channels.front() + channels.back()) / 2.0;
        retune_source(soapy_source, tileCentre, cfg.settle_ms);
        update_spectrum_centre(spectrum, tileCentre);
    }

    // Hot channel detection (same threshold logic as before)
    constexpr double kL1Multiplier = 6.0;
    std::vector<double> sorted(result.energy.begin(), result.energy.end());
    std::ranges::sort(sorted);
    const double median = (nCh % 2 == 1)
        ? sorted[nCh / 2]
        : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
    const double thresh = median * kL1Multiplier;

    std::vector<bool> isHot(nCh, false);
    for (std::size_t idx = 0; idx < nCh; ++idx) {
        if (result.energy[idx] > thresh) {
            isHot[idx] = true;
        }
    }

    // Wideband aggregation
    for (const double bw : cfg.bws) {
        const auto groupSize = static_cast<std::size_t>(std::round(bw / chBw));
        if (groupSize <= 1) {
            continue;
        }
        for (std::size_t start = 0; start + groupSize <= nCh; ++start) {
            double groupSum = 0.0;
            for (std::size_t k = 0; k < groupSize; ++k) {
                groupSum += result.energy[start + k];
            }
            const double groupAvg = groupSum / static_cast<double>(groupSize);
            if (groupAvg > thresh) {
                const std::size_t center = start + groupSize / 2;
                if (!isHot[center]) {
                    isHot[center] = true;
                }
            }
        }
    }

    for (std::size_t idx = 0; idx < nCh; ++idx) {
        if (isHot[idx]) {
            result.hot_idx.push_back(static_cast<int>(idx));
        }
    }
    std::ranges::sort(result.hot_idx, [&](int a, int b) {
        return result.energy[static_cast<std::size_t>(a)]
             > result.energy[static_cast<std::size_t>(b)];
    });
    for (int idx : result.hot_idx) {
        gr::lora::log_ts("debug", "lora_scan",
            "L1 HOT %.0f Hz  energy=%.6f (thresh=%.6f)",
            channels[static_cast<std::size_t>(idx)],
            result.energy[static_cast<std::size_t>(idx)], thresh);
    }
    return result;
}

// ─── Layer 2: per-channel CAD dwell — multi-SF on each window ────────────────

/// Build a ChannelActivityDetector pre-initialized for multi-SF detection.
static gr::lora::ChannelActivityDetector buildMultiSfDetector(
        double bw, uint32_t osFactor) {
    gr::lora::ChannelActivityDetector cad;
    cad.sf         = 12U;  // start() needs a valid SF; detectMultiSf uses its own table
    cad.bandwidth  = static_cast<uint32_t>(bw);
    cad.os_factor  = osFactor;
    cad.dual_chirp = true;
    cad.debug      = false;
    cad.start();
    cad.initMultiSf();
    return cad;
}

struct DirectCadResult {
    double   freq          = 0.0;
    double   bw            = 0.0;
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t sf_detected   = 0;
    bool     up_detected   = false;
    bool     dn_detected   = false;
};

// ─── Mode B: interleaved L1 snapshot + immediate L2 ─────────────────────────

struct InterleavedResult {
    std::vector<DirectCadResult> detections;
    std::vector<double>          energy;
    uint32_t                     l1_hot_total = 0;
    uint32_t                     l2_probes    = 0;
};

struct L2BwResult {
    double      bw;
    uint32_t    sf;
    double      ratio;
    const char* chirp;
};

struct SweepCallbacks {
    std::function<void(double freq, const char* reason)>                            onRetune;
    std::function<void(double freq, uint32_t durationMs, std::vector<L2BwResult>)>  onL2Probe;
    std::function<void(int snap, int total, uint32_t hotCount)>                     onProgress;
};

static InterleavedResult interleavedSweep(
        ScanGraph& sg,
        std::shared_ptr<gr::BlockModel>& soapy_source,
        const std::vector<double>& channels,
        const ScanConfig& cfg,
        const std::atomic<bool>& stopFlag,
        const SweepCallbacks& callbacks = {}) {
    InterleavedResult ir;

    const std::size_t nCh    = channels.size();
    const double      chBw   = cfg.min_bw();
    ir.energy.resize(nCh, 0.0);

    struct BwDetector {
        double bw;
        gr::lora::ChannelActivityDetector cad;
    };
    std::vector<BwDetector> bwDets;
    for (const double bw : cfg.bws) {
        bwDets.push_back({bw, buildMultiSfDetector(bw, cfg.os_factor)});
    }

    const double tileCentre = (channels.front() + channels.back()) / 2.0;

    constexpr double kL1Multiplier = 6.0;
    const int kSnapshots = static_cast<int>(2U * nCh * cfg.bws.size());

    std::vector<double> energy(nCh, 0.0);
    std::vector<bool> probed(nCh, false);

    // Ensure we're tuned to tile centre
    retune_source(soapy_source, tileCentre, cfg.settle_ms);
    update_spectrum_centre(sg.spectrum, tileCentre);

    for (int snap = 0; snap < kSnapshots; ++snap) {
        if (stopFlag.load(std::memory_order_relaxed)) {
            break;
        }

        // Single FFT snapshot: extract fresh per-channel energy, then
        // update running-max for CBOR spectrum output.
        std::ranges::fill(energy, 0.0);
        extractTileEnergy(sg.spectrum, channels, chBw, energy);
        for (std::size_t i = 0; i < nCh; ++i) {
            ir.energy[i] = std::max(ir.energy[i], energy[i]);
        }

        std::vector<double> sorted(energy.begin(), energy.end());
        std::ranges::sort(sorted);
        const double median = (nCh % 2 == 1)
            ? sorted[nCh / 2]
            : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
        const double thresh = median * kL1Multiplier;

        int    bestIdx    = -1;
        double bestEnergy = 0.0;

        for (std::size_t idx = 0; idx < nCh; ++idx) {
            if (!probed[idx] && energy[idx] > thresh && energy[idx] > bestEnergy) {
                bestIdx    = static_cast<int>(idx);
                bestEnergy = energy[idx];
            }
        }

        for (const double bw : cfg.bws) {
            const auto groupSize = static_cast<std::size_t>(std::round(bw / chBw));
            if (groupSize <= 1) {
                continue;
            }
            for (std::size_t start = 0; start + groupSize <= nCh; ++start) {
                const std::size_t center = start + groupSize / 2;
                if (probed[center]) {
                    continue;
                }
                double groupSum = 0.0;
                for (std::size_t k = 0; k < groupSize; ++k) {
                    groupSum += energy[start + k];
                }
                const double groupAvg = groupSum / static_cast<double>(groupSize);
                if (groupAvg > thresh && groupAvg > bestEnergy) {
                    bestIdx    = static_cast<int>(center);
                    bestEnergy = groupAvg;
                }
            }
        }

        if (bestIdx < 0) {
            if (callbacks.onProgress && (snap % 100 == 0)) {
                callbacks.onProgress(snap + 1, kSnapshots, ir.l1_hot_total);
            }
            continue;
        }

        ++ir.l1_hot_total;
        const double freq = channels[static_cast<std::size_t>(bestIdx)];

        gr::lora::log_ts("debug", "lora_scan",
            "B: snap %d/%d  HOT %.0f Hz  energy=%.6f (thresh=%.6f)  -> immediate L2",
            snap + 1, kSnapshots, freq, bestEnergy, thresh);

        double bestRatio = 0.0;
        DirectCadResult bestResult;
        std::vector<L2BwResult> bwResults;

        // Retune to hot channel for L2 CAD
        retune_source(soapy_source, freq, cfg.settle_ms);
        if (callbacks.onRetune) callbacks.onRetune(freq, "l2_probe");

        const auto probeStart = std::chrono::steady_clock::now();

        // Single capture: narrowest BW (first in sorted bwDets) needs the
        // most L1 samples.  Wider BWs use a prefix of the same buffer.
        const uint32_t sf12Win = (1U << 12U) * cfg.os_factor * 2U;
        const double   maxTargetRate = bwDets.front().bw * static_cast<double>(cfg.os_factor);
        const auto     maxDecFactor  = static_cast<uint32_t>(std::round(cfg.l1_rate / maxTargetRate));
        const uint32_t maxL1Win      = sf12Win * maxDecFactor;

        const auto l2Raw = capture_samples(sg.capture, maxL1Win, stopFlag);
        if (l2Raw.empty()) {
            retune_source(soapy_source, tileCentre, cfg.settle_ms);
            if (callbacks.onRetune) callbacks.onRetune(tileCentre, "l1_restore");
            continue;
        }

        for (auto& bd : bwDets) {
            if (stopFlag.load(std::memory_order_relaxed)) {
                break;
            }

            const double   targetRate = bd.bw * static_cast<double>(cfg.os_factor);
            const auto     decFactor  = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));
            const uint32_t l1Win      = sf12Win * decFactor;

            // Decimate a prefix of the single capture
            const auto cadBuf = decimate(l2Raw.data(), l1Win, cfg.l1_rate, targetRate);

            const auto r = bd.cad.detectMultiSf(cadBuf.data());
            if (r.detected) {
                const double ratio = static_cast<double>(r.best_ratio);
                if (ratio > bestRatio) {
                    bestRatio         = ratio;
                    bestResult.freq   = freq;
                    bestResult.bw     = bd.bw;
                    bestResult.peak_ratio_up = static_cast<double>(r.peak_ratio_up);
                    bestResult.peak_ratio_dn = static_cast<double>(r.peak_ratio_dn);
                    bestResult.sf_detected   = r.sf;
                    bestResult.up_detected   = r.up_detected;
                    bestResult.dn_detected   = r.dn_detected;
                }
            }

            {
                const double ratio = r.detected
                    ? static_cast<double>(r.best_ratio) : 0.0;
                const char* chirp = !r.detected ? ""
                    : (r.up_detected && r.dn_detected) ? "both"
                    : r.dn_detected ? "dn" : "up";
                bwResults.push_back({bd.bw, r.sf, ratio, chirp});
            }

            if (bestRatio > 10.0) {
                break;
            }
        }

        const auto probeEnd = std::chrono::steady_clock::now();
        const auto probeMs  = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                probeEnd - probeStart).count());

        if (callbacks.onL2Probe) {
            callbacks.onL2Probe(freq, probeMs, std::move(bwResults));
        }

        // Return to tile centre for next L1 snapshot
        retune_source(soapy_source, tileCentre, cfg.settle_ms);
        update_spectrum_centre(sg.spectrum, tileCentre);
        if (callbacks.onRetune) callbacks.onRetune(tileCentre, "l1_restore");

        probed[static_cast<std::size_t>(bestIdx)] = true;
        ++ir.l2_probes;

        if (bestRatio > 0.0) {
            ir.detections.push_back(bestResult);
        }
    }

    return ir;
}

// ─── CBOR output ──────────────────────────────────────────────────────────────

static void emitSpectrumCbor(
    const std::vector<double>& channels,
    const std::vector<double>& energy,
    const std::vector<std::size_t>& hotIndices,
    const std::vector<DirectCadResult>& detections,
    uint32_t sweepCount)
{
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(512);

    cb::encode_map_begin(buf, 9);
    cb::kv_text(buf, "type", "scan_spectrum");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweepCount);
    cb::kv_uint(buf, "freq_min", static_cast<uint64_t>(channels.front()));

    const uint64_t step = channels.size() > 1
        ? static_cast<uint64_t>(channels[1] - channels[0])
        : 62500;
    cb::kv_uint(buf, "freq_step", step);

    const auto nCh = static_cast<uint32_t>(energy.size());
    std::vector<uint8_t> floatBuf(nCh * sizeof(float));
    for (uint32_t i = 0; i < nCh; ++i) {
        auto val = static_cast<float>(energy[i]);
        std::memcpy(floatBuf.data() + i * sizeof(float), &val, sizeof(float));
    }
    cb::kv_bytes(buf, "channels", floatBuf.data(), floatBuf.size());
    cb::kv_uint(buf, "n_channels", nCh);

    cb::encode_text(buf, "hot");
    cb::encode_array_begin(buf, hotIndices.size());
    for (auto idx : hotIndices) {
        cb::encode_uint(buf, static_cast<uint64_t>(idx));
    }

    cb::encode_text(buf, "detections");
    cb::encode_array_begin(buf, detections.size());
    for (const auto& det : detections) {
        cb::encode_map_begin(buf, 7);
        cb::kv_uint(buf, "freq", static_cast<uint64_t>(det.freq));
        cb::kv_uint(buf, "sf", det.sf_detected);
        cb::kv_uint(buf, "bw", static_cast<uint64_t>(det.bw));
        cb::kv_float64(buf, "ratio", std::max(det.peak_ratio_up, det.peak_ratio_dn));
        cb::kv_float64(buf, "ratio_up", det.peak_ratio_up);
        cb::kv_float64(buf, "ratio_dn", det.peak_ratio_dn);
        const char* chirp = (det.up_detected && det.dn_detected) ? "both"
                          : det.dn_detected                     ? "dn"
                          :                                       "up";
        cb::kv_text(buf, "chirp", chirp);
    }

    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

static void emitStatusCbor(const ScanConfig& cfg, const ScanStats& stats,
                           const std::string& mode)
{
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(256);

    cb::encode_map_begin(buf, 8);
    cb::kv_text(buf, "type", "scan_status");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_text(buf, "mode", mode);
    cb::kv_uint(buf, "sweeps", stats.sweeps);
    cb::kv_uint(buf, "detections", stats.total_detections);
    cb::kv_uint(buf, "overflows", stats.overflows);
    cb::kv_uint(buf, "dropouts", stats.drops);

    cb::encode_text(buf, "scan_range");
    cb::encode_array_begin(buf, 2);
    cb::encode_uint(buf, static_cast<uint64_t>(cfg.freq_start));
    cb::encode_uint(buf, static_cast<uint64_t>(cfg.freq_stop));

    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

// ─── CBOR event emitters (real-time diagnostics) ─────────────────────────────

static void emitSweepStartCbor(uint32_t sweep) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(64);
    cb::encode_map_begin(buf, 3);
    cb::kv_text(buf, "type", "scan_sweep_start");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweep);
    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

static void emitSweepEndCbor(uint32_t sweep, uint32_t durationMs,
                              uint32_t l1Snapshots, uint32_t l2Probes,
                              uint32_t hotCount, uint32_t detections,
                              uint32_t overflows) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(128);
    cb::encode_map_begin(buf, 9);
    cb::kv_text(buf, "type", "scan_sweep_end");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweep);
    cb::kv_uint(buf, "duration_ms", durationMs);
    cb::kv_uint(buf, "l1_snapshots", l1Snapshots);
    cb::kv_uint(buf, "l2_probes", l2Probes);
    cb::kv_uint(buf, "hot_count", hotCount);
    cb::kv_uint(buf, "detections", detections);
    cb::kv_uint(buf, "overflows", overflows);
    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

static void emitL2ProbeCbor(uint32_t sweep, double freq, uint32_t durationMs,
                             const std::vector<L2BwResult>& results) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(256);
    cb::encode_map_begin(buf, 6);
    cb::kv_text(buf, "type", "scan_l2_probe");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweep);
    cb::kv_uint(buf, "freq", static_cast<uint64_t>(freq));
    cb::kv_uint(buf, "duration_ms", durationMs);
    cb::encode_text(buf, "results");
    cb::encode_array_begin(buf, results.size());
    for (const auto& r : results) {
        cb::encode_map_begin(buf, 4);
        cb::kv_uint(buf, "bw", static_cast<uint64_t>(r.bw));
        cb::kv_uint(buf, "sf", r.sf);
        cb::kv_float64(buf, "ratio", r.ratio);
        cb::kv_text(buf, "chirp", r.chirp);
    }
    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

static void emitRetuneCbor(uint32_t sweep, double freq, const char* reason) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(96);
    cb::encode_map_begin(buf, 5);
    cb::kv_text(buf, "type", "scan_retune");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweep);
    cb::kv_uint(buf, "freq", static_cast<uint64_t>(freq));
    cb::kv_text(buf, "reason", reason);
    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

static void emitOverflowCbor(uint32_t sweep, uint64_t count) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(64);
    cb::encode_map_begin(buf, 4);
    cb::kv_text(buf, "type", "scan_overflow");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_uint(buf, "sweep", sweep);
    cb::kv_uint(buf, "count", count);
    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

// ─── UTC time helper: "HH:MM:SS.mmm" ─────────────────────────────────────────

static std::string ts_short() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d",
        tm.tm_hour, tm.tm_min, tm.tm_sec,
        static_cast<int>(ms.count()));
    return buf;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    ScanConfig cfg;
    int rc = parseArgs(argc, argv, cfg);
    if (rc != 0) {
        return rc == 2 ? 0 : 1;
    }

    auto& g_stop = lora_apps::install_signal_handler();

    int savedStdout = -1;

    try {
    const auto channels = makeChannelList(cfg.freq_start, cfg.freq_stop, cfg.min_bw());

    std::string bwListStr;
    for (std::size_t i = 0; i < cfg.bws.size(); ++i) {
        if (i > 0) bwListStr += ",";
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%.0f", cfg.bws[i]);
        bwListStr += tmp;
    }

    lora_apps::log_version_banner("lora_scan", GIT_REV);

    gr::lora::log_ts("info ", "lora_scan",
        "%zu channel(s) SF7-12 BW=[%s] Hz os=%u L1=%.1f MS/s",
        channels.size(), bwListStr.c_str(),
        cfg.os_factor, cfg.l1_rate / 1.0e6);

    if (cfg.sweeps == 0) {
        gr::lora::log_ts("info ", "lora_scan", "continuous mode -- Ctrl+C to stop");
    } else {
        gr::lora::log_ts("info ", "lora_scan", "%u sweep(s)", cfg.sweeps);
    }

    lora_apps::apply_fpga_workaround(cfg.device, cfg.device_param);

    // stdout protection: UHD/SoapySDR may print to stdout during device init
    if (cfg.cbor_out) {
        std::fflush(stdout);
        savedStdout = dup(STDOUT_FILENO);
        dup2(STDERR_FILENO, STDOUT_FILENO);
    }

    lora_apps::log_hardware_info("lora_scan", cfg.device, cfg.device_param);

    // Shrink thread pool — singleThreadedBlocking doesn't use it
    gr::thread_pool::Manager::defaultCpuPool()->setThreadBounds(1, 1);

    // Build GR4 graph
    gr::Graph graph;
    auto sg = build_scan_graph(graph, cfg, channels);

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking> sched;
    sched.timeout_inactivity_count = 1U;
    sched.watchdog_min_stall_count = 10U;
    sched.watchdog_max_warnings    = 30U;

    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        gr::lora::log_ts("error", "lora_scan", "scheduler init failed");
        return 1;
    }

    // Find SoapySource for runtime retune (after exchange, original refs are dangling)
    std::shared_ptr<gr::BlockModel> soapy_source;
    for (auto& blk : sched.blocks()) {
        if (blk->typeName().find("Soapy") != std::string_view::npos) {
            soapy_source = blk;
            break;
        }
    }
    if (!soapy_source) {
        gr::lora::log_ts("error", "lora_scan", "SoapySource not found in graph");
        return 1;
    }

    if (savedStdout >= 0) {
        dup2(savedStdout, STDOUT_FILENO);
        close(savedStdout);
        savedStdout = -1;
    }

    // Start scheduler in background thread
    std::atomic<bool> sched_done{false};
    std::thread sched_thread([&sched, &sched_done]() {
        auto ret = sched.runAndWait();
        if (!ret.has_value()) {
            gr::lora::log_ts("warn ", "lora_scan",
                "scheduler stopped: %s", std::format("{}", ret.error()).c_str());
        }
        sched_done.store(true, std::memory_order_relaxed);
    });

    // Give the graph a moment to start streaming before the orchestrator begins
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ScanStats stats;

    // ── Layer 1 only mode ────────────────────────────────────────────────────
    if (cfg.layer1_only) {
        auto* out = cfg.cbor_out ? stderr : stdout;
        std::fprintf(out, "%-15s  %12s  %6s\n", "freq_hz", "energy_lin", "L1");
        std::fprintf(out, "%s\n", std::string(38, '-').c_str());

        while (!g_stop.load(std::memory_order_relaxed)
               && !sched_done.load(std::memory_order_relaxed)) {
            const auto l1 = layer1Scan(sg.spectrum, soapy_source, channels, cfg);
            std::vector<std::size_t> hotIndices;
            for (std::size_t idx = 0; idx < channels.size(); ++idx) {
                const bool hot = std::ranges::find(l1.hot_idx, static_cast<int>(idx)) != l1.hot_idx.end();
                if (hot) hotIndices.push_back(idx);
                std::fprintf(out, "%-15.0f  %12.6f  %6s\n",
                            channels[idx], l1.energy[idx], hot ? "HOT" : "");
            }
            std::fflush(out);
            if (cfg.cbor_out) {
                emitSpectrumCbor(channels, l1.energy, hotIndices, {}, stats.sweeps);
            }
            if (++stats.sweeps >= cfg.sweeps && cfg.sweeps > 0) {
                break;
            }
        }
        sched.requestStop();
        sched_thread.join();
        std::quick_exit(0);
    }

    // ── Full scan: adaptive mode selection ─────────────────────────────────

    {
        auto* out = cfg.cbor_out ? stderr : stdout;
        std::fprintf(out, "%-13s  %10s  %6s  %6s  %6s  %6s  %4s\n",
                    "time", "freq_mhz", "bw_khz", "up", "dn", "wins", "sf");
        std::fprintf(out, "%s\n", std::string(60, '-').c_str());
        std::fflush(out);
    }

    const double scanRange = channels.back() - channels.front() + cfg.min_bw();
    const double usableBw  = cfg.l1_rate * 0.8;

    if (scanRange > usableBw) {
        gr::lora::log_ts("error", "lora_scan",
            "scan range %.1f MHz exceeds usable BW %.1f MHz at L1 rate %.1f MS/s "
            "-- increase --l1-rate or narrow --freq-start/--freq-stop",
            scanRange / 1.0e6, usableBw / 1.0e6, cfg.l1_rate / 1.0e6);
        sched.requestStop();
        sched_thread.join();
        return 1;
    }

    gr::lora::log_ts("info ", "lora_scan",
        "scan mode: single-pass  (%zu ch, range=%.1f MHz, tile=%.1f MHz)",
        channels.size(), scanRange / 1.0e6, usableBw / 1.0e6);

    std::vector<DirectCadResult> sweepDetections;

    auto reportDetection = [&](const DirectCadResult& det) {
        const double ratio = std::max(det.peak_ratio_up, det.peak_ratio_dn);
        if (ratio < static_cast<double>(cfg.min_ratio)) {
            return;
        }

        ++stats.total_detections;
        if (cfg.cbor_out) {
            sweepDetections.push_back(det);
        }

        const auto ts = ts_short();
        const std::string sfStr = det.sf_detected
            ? ("SF" + std::to_string(det.sf_detected)) : "-";

        auto* out = cfg.cbor_out ? stderr : stdout;
        std::fprintf(out, "%-13s  %10.3f  %6.1f  %6.1f  %6.1f  %6s  %4s\n",
                    ts.c_str(), det.freq / 1.0e6, det.bw / 1.0e3,
                    det.peak_ratio_up, det.peak_ratio_dn,
                    "1/1", sfStr.c_str());
        std::fflush(out);

        char freqBuf[32];
        std::snprintf(freqBuf, sizeof(freqBuf), "%.3f MHz", det.freq / 1.0e6);
        stats.last_det_freq = freqBuf;
        stats.last_det_time = ts;
    };

    std::vector<double> sweepEnergy(channels.size(), 0.0);
    auto lastCborStatus = std::chrono::steady_clock::now();
    double avgSweepMs = 0.0;

    // Overflow polling: typed access to SoapySource
    uint64_t lastOverflowCount = 0;
    auto checkOverflows = [&](uint32_t sweep) {
        // Access overflow count via the typed block inside the wrapper.
        // SoapyBlock<cf32, 1>::totalOverflowCount() is atomic and cross-thread safe.
        using SoapyType = gr::blocks::soapy::SoapyBlock<std::complex<float>, 1UL>;
        auto* wrapper = dynamic_cast<gr::BlockWrapper<SoapyType>*>(soapy_source.get());
        if (!wrapper) return;
        const uint64_t current = wrapper->blockRef().totalOverflowCount();
        if (current > lastOverflowCount) {
            stats.overflows = static_cast<uint32_t>(current);
            if (cfg.cbor_out) {
                emitOverflowCbor(sweep, current);
            }
            lastOverflowCount = current;
        }
    };

    while (!g_stop.load(std::memory_order_relaxed)
           && !sched_done.load(std::memory_order_relaxed)) {
        sweepDetections.clear();
        std::ranges::fill(sweepEnergy, 0.0);

        const uint32_t sweepNum = stats.sweeps + 1;
        const auto sweepStart = std::chrono::steady_clock::now();
        uint32_t l2Probes = 0;

        if (cfg.cbor_out) {
            emitSweepStartCbor(sweepNum);
        }

        SweepCallbacks callbacks;
        if (cfg.cbor_out) {
            callbacks.onRetune = [sweepNum](double freq, const char* reason) {
                emitRetuneCbor(sweepNum, freq, reason);
            };
            callbacks.onL2Probe = [&, sweepNum](double freq, uint32_t ms,
                    std::vector<L2BwResult> results) {
                emitL2ProbeCbor(sweepNum, freq, ms, results);
                checkOverflows(sweepNum);
            };
        }

        const auto bResult = interleavedSweep(sg, soapy_source, channels, cfg,
                                               g_stop, callbacks);
        for (const auto& det : bResult.detections) {
            reportDetection(det);
        }
        stats.l1_hot = bResult.l1_hot_total;
        sweepEnergy = bResult.energy;
        l2Probes = bResult.l2_probes;

        ++stats.sweeps;

        const auto sweepMs = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - sweepStart).count());
        avgSweepMs = (stats.sweeps <= 1)
            ? static_cast<double>(sweepMs)
            : avgSweepMs * 0.8 + static_cast<double>(sweepMs) * 0.2;

        if (cfg.cbor_out) {
            std::vector<std::size_t> hotIndices;
            if (!sweepEnergy.empty()) {
                std::vector<double> sorted(sweepEnergy.begin(), sweepEnergy.end());
                std::ranges::sort(sorted);
                const auto nCh = sweepEnergy.size();
                const double median = (nCh % 2 == 1)
                    ? sorted[nCh / 2]
                    : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
                const double thresh = median * 6.0;
                for (std::size_t idx = 0; idx < nCh; ++idx) {
                    if (sweepEnergy[idx] > thresh) {
                        hotIndices.push_back(idx);
                    }
                }
            }

            const auto kTotalSnaps = static_cast<uint32_t>(
                2U * channels.size() * cfg.bws.size());
            emitSweepEndCbor(sweepNum, sweepMs, kTotalSnaps,
                             l2Probes, stats.l1_hot,
                             static_cast<uint32_t>(sweepDetections.size()),
                             stats.overflows);

            emitSpectrumCbor(channels, sweepEnergy, hotIndices,
                             sweepDetections, stats.sweeps);

            const auto now2 = std::chrono::steady_clock::now();
            if (stats.sweeps == 1
                || (now2 - lastCborStatus) >= std::chrono::seconds(5)) {
                lastCborStatus = now2;
                emitStatusCbor(cfg, stats, "single-pass");
            }
        }

        checkOverflows(stats.sweeps);

        if (cfg.sweeps > 0 && stats.sweeps >= cfg.sweeps) {
            break;
        }

        gr::lora::log_ts("info ", "lora_scan",
            "sweep %u  dur %ums  hot %u/%zu  det %zu  OVF %u",
            stats.sweeps, sweepMs, stats.l1_hot, channels.size(),
            sweepDetections.size(), stats.overflows);
    }

    gr::lora::log_ts("info ", "lora_scan",
        "stopped after %u sweep(s), %u overflow(s), %u drop(s)",
        stats.sweeps, stats.overflows, stats.drops);

    sched.requestStop();
    sched_thread.join();
    std::quick_exit(0);
    } catch (const std::exception& e) {
        if (savedStdout >= 0) {
            dup2(savedStdout, STDOUT_FILENO);
            close(savedStdout);
        }
        std::fprintf(stderr, "fatal: %s\n", e.what());
        return 1;
    }
}
