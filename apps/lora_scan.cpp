// SPDX-License-Identifier: ISC
//
// lora_scan — LoRa spectral scanner
//
// Continuously sweeps a set of LoRa channels looking for chirp activity.
//
// Layer 1: wideband FFT energy scan → candidate channels
// Layer 2: per-channel CAD dwell with K consecutive 2-symbol windows
// Layer 3: (optional) SF sweep on CAD-detected channels
//
// Run `lora_scan --help` for usage.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>

#include <gnuradio-4.0/lora/log.hpp>

using cf32 = std::complex<float>;

// Copied from lora_trx: Device + Stream types.
using SoapyDevice = gr::blocks::soapy::Device;
using RxStream    = SoapyDevice::Stream<cf32, SOAPY_SDR_RX>;

// SIGINT handler.
static std::atomic<bool> g_stop{false};

static void sigintHandler(int /*sig*/) {
    g_stop.store(true, std::memory_order_relaxed);
}

struct ScanStats {
    uint32_t    sweeps          = 0;
    uint32_t    overflows       = 0;
    uint32_t    drops           = 0;
    uint32_t    l1_hot          = 0;   // hot channels in current sweep
    std::string last_det_freq;         // last YES detection frequency
    std::string last_det_time;         // last YES detection timestamp
};

struct ScanConfig {
    std::string          device       = "uhd";
    std::string          device_param = "type=b200";
    double               freq_start   = 863.0e6;
    double               freq_stop    = 870.0e6;
    std::vector<double>  bws          = {62500.0, 125000.0, 250000.0};  // sorted narrowest first
    uint32_t             os_factor    = 4;
    double               gain         = 40.0;
    int                  settle_ms    = 5;
    double               l1_rate      = 8.0e6;   // wideband L1 sample rate (even B210 decimation)
    uint32_t             cad_windows  = 8;
    uint32_t             max_l2       = 8;        // max L2 candidates per sweep
    float                min_ratio    = 8.0F;     // minimum peak ratio to report (noise floor ~4)
    uint32_t             sweeps       = 0;        // 0 = infinite
    bool                 layer1_only  = false;
    bool                 all_channels = false;

    /// Narrowest bandwidth (used for channel grid and L1).
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
        "  --l1-rate <Hz>        L1 wideband sample rate (default: 8e6)\n"
        "  --cad-windows <K>     CAD windows per BW per channel (default: 8)\n"
        "  --max-l2 <N>          Max L2 candidates per sweep (default: 8)\n"
        "  --min-ratio <f>       Min peak ratio to report (default: 8.0)\n"
        "  --sweeps <N>          Stop after N sweeps (default: 0 = infinite)\n"
        "  --layer1-only         Stop after Layer 1 (energy scan only)\n"
        "  --all-channels        Run CAD on all channels, not just hot ones\n"
        "  --log-level <level>   Log level: DEBUG, INFO, WARNING, ERROR\n"
        "  --version             Show version and exit\n"
        "  -h, --help            Show this help\n");
}

/// Returns 0 on success, 1 on error, 2 on --help/--version.
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
            std::ranges::sort(cfg.bws);  // narrowest first
        }
        else if (arg == "--os")            cfg.os_factor    = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--gain")          cfg.gain         = std::stod(next());
        else if (arg == "--settle-ms")     cfg.settle_ms    = std::stoi(next());
        else if (arg == "--l1-rate")       cfg.l1_rate      = std::stod(next());
        else if (arg == "--cad-windows")   cfg.cad_windows  = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--max-l2")        cfg.max_l2       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--min-ratio")     cfg.min_ratio    = std::stof(next());
        else if (arg == "--sweeps")        cfg.sweeps       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--layer1-only")   cfg.layer1_only  = true;
        else if (arg == "--all-channels")  cfg.all_channels = true;
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
    const double scanRange = cfg.freq_stop - cfg.freq_start;
    if (scanRange > cfg.l1_rate * 0.9) {
        gr::lora::log_ts("warn ", "lora_scan",
            "scan range %.1f MHz > L1 rate %.1f MS/s — L1 will tile multiple snapshots",
            scanRange / 1.0e6, cfg.l1_rate / 1.0e6);
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

// ─── SDR helpers ──────────────────────────────────────────────────────────────

static SoapyDevice openDevice(const ScanConfig& cfg) {
    auto args = gr::blocks::soapy::detail::buildDeviceArgs(cfg.device, cfg.device_param);
    return SoapyDevice(args);
}

static void retune(RxStream& stream, SoapyDevice& dev, double freq, double sampleRate, int settleMs) {
    stream.deactivate();
    dev.setCenterFrequency(SOAPY_SDR_RX, 0, freq);
    dev.setSampleRate(SOAPY_SDR_RX, 0, sampleRate);
    std::this_thread::sleep_for(std::chrono::milliseconds(settleMs));
    stream.activate();
}

static void capture(RxStream& stream, cf32* buf, uint32_t nSamples, ScanStats& stats) {
    uint32_t collected = 0;
    while (collected < nSamples) {
        int flags = 0;
        long long timeNs = 0;
        std::span<cf32> slice(buf + collected, nSamples - collected);
        int ret = stream.readStream(flags, timeNs, 1'000'000, slice);
        if (ret > 0) {
            collected += static_cast<uint32_t>(ret);
        } else if (ret == SOAPY_SDR_OVERFLOW) {
            ++stats.overflows;
            gr::lora::log_ts("warn ", "lora_scan",
                "USB buffer overflow (#%u)", stats.overflows);
        } else if (ret == SOAPY_SDR_TIMEOUT) {
            ++stats.drops;
            gr::lora::log_ts("warn ", "lora_scan",
                "readStream timeout / dropout (#%u)", stats.drops);
        }
    }
}

// ─── Layer 1: wideband FFT energy scan ───────────────────────────────────────

struct Layer1Result {
    std::vector<double> energy;
    std::vector<int>    hot_idx;
};

// Measure per-channel energy from a single wideband FFT snapshot.
// `tileCentre` and `tileRate` define the SDR tuning; only channels within the
// tile bandwidth are measured (others left at 0).
static void measureTile(RxStream& stream, SoapyDevice& dev,
                        const std::vector<double>& channels,
                        double tileCentre, double tileRate,
                        double chBw, int settleMs,
                        std::vector<double>& energy, ScanStats& stats) {
    retune(stream, dev, tileCentre, tileRate, settleMs);

    // FFT size: at least 256 bins per channel, power of 2.
    const std::size_t nCh = channels.size();
    constexpr std::size_t kMinBinsPerChannel = 256;
    std::size_t fftSize = 1;
    while (fftSize < nCh * kMinBinsPerChannel) {
        fftSize <<= 1U;
    }
    if (fftSize < 4096) {
        fftSize = 4096;  // minimum for wideband to get reasonable resolution
    }

    // Pre-compute Hann window.
    std::vector<float> hann(fftSize);
    for (std::size_t idx = 0; idx < fftSize; ++idx) {
        hann[idx] = static_cast<float>(
            0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(idx)
                                  / static_cast<double>(fftSize - 1)));
    }

    const double hzPerBin   = tileRate / static_cast<double>(fftSize);
    const double halfChBins = (chBw / tileRate) * static_cast<double>(fftSize) / 2.0;
    const double halfTile   = tileRate / 2.0;

    // Take kL1Snapshots consecutive FFT snapshots and max-hold per-channel
    // energy.  A short LoRa burst (SF7 ~330 ms) may only appear in one of
    // the snapshots; max-hold ensures it is detected.  Each snapshot costs
    // fftSize/tileRate seconds (e.g. 16384/8e6 = 2 ms), so 4 snapshots add
    // only ~8 ms per tile.
    constexpr int kL1Snapshots = 4;

    std::vector<cf32> buf(fftSize);
    gr::algorithm::FFT<cf32> fft;

    for (int snap = 0; snap < kL1Snapshots; ++snap) {
        capture(stream, buf.data(), static_cast<uint32_t>(fftSize), stats);

        // Apply Hann window.
        for (std::size_t idx = 0; idx < fftSize; ++idx) {
            buf[idx] *= hann[idx];
        }

        auto spectrum = fft.compute(std::span<const cf32>(buf.data(), fftSize));

        for (std::size_t chIdx = 0; chIdx < nCh; ++chIdx) {
            const double chOffset = channels[chIdx] - tileCentre;
            if (std::abs(chOffset) > halfTile * 0.9) {
                continue;
            }
            const double binCentre = chOffset / hzPerBin;
            const int kLo = static_cast<int>(std::round(binCentre - halfChBins));
            const int kHi = static_cast<int>(std::round(binCentre + halfChBins));

            double sum = 0.0;
            int    cnt = 0;
            for (int k = kLo; k <= kHi; ++k) {
                const auto binIdx = static_cast<std::size_t>(
                    (k % static_cast<int>(fftSize) + static_cast<int>(fftSize))
                    % static_cast<int>(fftSize));
                sum += static_cast<double>(std::norm(spectrum[binIdx]));
                ++cnt;
            }
            if (cnt > 0) {
                const double snapEnergy = sum / static_cast<double>(cnt);
                energy[chIdx] = std::max(energy[chIdx], snapEnergy);
            }
        }
    }
}

static Layer1Result layer1Scan(RxStream& stream, SoapyDevice& dev,
                               const std::vector<double>& channels,
                               const ScanConfig& cfg, ScanStats& stats) {
    const std::size_t nCh = channels.size();
    Layer1Result result;
    result.energy.resize(nCh, 0.0);

    const double chBw      = cfg.min_bw();
    const double scanRange = channels.back() - channels.front() + chBw;
    const double usableBw  = cfg.l1_rate * 0.8;  // 80% usable (roll-off at edges)

    if (scanRange <= usableBw) {
        const double centre = (channels.front() + channels.back()) / 2.0;
        measureTile(stream, dev, channels, centre, cfg.l1_rate,
                    chBw, cfg.settle_ms, result.energy, stats);
        gr::lora::log_ts("debug", "lora_scan",
            "L1 centre=%.0f rate=%.0f fftBw=%.0f (single tile)",
            centre, cfg.l1_rate, cfg.l1_rate);
    } else {
        const double step = usableBw;
        const double first = channels.front() + usableBw / 2.0;
        const double last  = channels.back()  - usableBw / 2.0;
        int nTiles = 0;
        for (double tc = first; tc <= last + step * 0.5; tc += step) {
            measureTile(stream, dev, channels, tc, cfg.l1_rate,
                        chBw, cfg.settle_ms, result.energy, stats);
            ++nTiles;
        }
        gr::lora::log_ts("debug", "lora_scan",
            "L1 %d tile(s) at %.0f MS/s usable=%.0f Hz",
            nTiles, cfg.l1_rate / 1.0e6, usableBw);
    }

    // Median-based threshold: robust to chirp energy spreading across bins.
    // Mean+3*std fails because a single LoRa chirp raises energy in multiple
    // bins, inflating std so the threshold chases the signal upward.
    // Median is unaffected by up to floor(N/2)-1 outlier channels.
    constexpr double kL1Multiplier = 6.0;  // channel is hot if energy > 6× median
    std::vector<double> sorted(result.energy.begin(), result.energy.end());
    std::ranges::sort(sorted);
    const double median = (nCh % 2 == 1)
        ? sorted[nCh / 2]
        : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
    const double thresh = median * kL1Multiplier;

    gr::lora::log_ts("debug", "lora_scan",
        "L1 median=%.6f thresh=%.6f (%zu ch)", median, thresh, nCh);

    // Pass 1: fine-grid detection (narrowest BW).
    std::vector<bool> isHot(nCh, false);
    for (std::size_t idx = 0; idx < nCh; ++idx) {
        if (result.energy[idx] > thresh) {
            isHot[idx] = true;
        }
    }

    // Pass 2: wideband aggregation.  A signal at BW > min_bw spreads its
    // energy across bw/min_bw adjacent fine-grid channels.  Sum groups of
    // that width and compare to threshold × group_size (energy scales
    // linearly with the number of channels if noise is flat).
    for (const double bw : cfg.bws) {
        const auto groupSize = static_cast<std::size_t>(
            std::round(bw / chBw));
        if (groupSize <= 1) {
            continue;  // already handled in pass 1
        }
        // Sliding window of groupSize channels.  Mark the center channel
        // as hot when the group's average energy exceeds the per-channel
        // threshold (equiv. to group sum > thresh × groupSize).
        for (std::size_t start = 0; start + groupSize <= nCh; ++start) {
            double groupSum = 0.0;
            for (std::size_t k = 0; k < groupSize; ++k) {
                groupSum += result.energy[start + k];
            }
            const double groupAvg = groupSum / static_cast<double>(groupSize);
            if (groupAvg > thresh) {
                // Mark center channel of the group as hot.
                const std::size_t center = start + groupSize / 2;
                if (!isHot[center]) {
                    isHot[center] = true;
                    gr::lora::log_ts("debug", "lora_scan",
                        "L1 wideband HOT %.0f Hz  BW=%.0f  group_avg=%.6f (thresh=%.6f, %zu ch group)",
                        channels[center], bw, groupAvg, thresh, groupSize);
                }
            }
        }
    }

    for (std::size_t idx = 0; idx < nCh; ++idx) {
        if (isHot[idx]) {
            result.hot_idx.push_back(static_cast<int>(idx));
        }
    }
    // Sort hot channels by energy (highest first) so the strongest signal
    // gets L2 dwell immediately — critical when ADVERT airtime is ~1.2s
    // and we have many hot channels from ISM wideband noise.
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
//
// Retune to the given BW, capture K windows, and for each window try every
// SF (7–12).  The first SF that fires is the detection.
//
// Two capture strategies:
//   - "per-SF" (default): each SF trial gets its own correctly-sized capture.
//     Accurate for long dwells where the signal persists across all trials.
//   - "shared" (sharedBuf mode): one SF12-sized capture per window, all SFs
//     tested on the same data.  The chirp present in the buffer matches
//     exactly one SF.  Avoids temporal offset between SF trials — critical
//     for short bursts that may end mid-dwell.

struct Layer2Result {
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t sf_detected    = 0;     // which SF triggered (0 = none)
    uint32_t detections     = 0;     // how many windows fired
    uint32_t windows_tried  = 0;
    bool     detected       = false;
};

struct SfDetector {
    gr::lora::ChannelActivityDetector det;
    uint32_t sf;
};

/// Build detectors for SF7–12 at the given BW.
static std::vector<SfDetector> buildDetectors(double bw, uint32_t osFactor) {
    std::vector<SfDetector> detectors;
    for (uint32_t trySf = 7; trySf <= 12; ++trySf) {
        SfDetector sd;
        sd.sf              = trySf;
        sd.det.sf          = trySf;
        sd.det.bandwidth   = static_cast<uint32_t>(bw);
        sd.det.os_factor   = osFactor;
        sd.det.alpha       = gr::lora::ChannelActivityDetector::default_alpha(trySf);
        sd.det.dual_chirp  = true;
        sd.det.debug       = false;
        sd.det.start();
        detectors.push_back(std::move(sd));
    }
    return detectors;
}

/// Run all-SF detection on a pre-captured SF12-sized buffer.
/// Returns the SF with the highest peak ratio, or 0 if none detected.
static Layer2Result detectOnBuffer(const cf32* buf, std::vector<SfDetector>& detectors,
                                   double freq, double bw) {
    Layer2Result result;
    result.windows_tried = 1;

    float    bestUp = 0.0F;
    float    bestDn = 0.0F;
    uint32_t bestSf = 0;

    for (auto& sd : detectors) {
        const auto r = sd.det.detect(buf);
        const float winRatio = std::max(r.peak_ratio_up, r.peak_ratio_dn);
        if (r.detected && winRatio > std::max(bestUp, bestDn)) {
            bestUp = r.peak_ratio_up;
            bestDn = r.peak_ratio_dn;
            bestSf = sd.sf;
        }
    }

    if (bestSf != 0) {
        result.detected      = true;
        result.detections    = 1;
        result.sf_detected   = bestSf;
        result.peak_ratio_up = static_cast<double>(bestUp);
        result.peak_ratio_dn = static_cast<double>(bestDn);

        gr::lora::log_ts("debug", "lora_scan",
            "CAD %.0f Hz BW=%.0f SF%u  ratio_up=%.2f ratio_dn=%.2f",
            freq, bw, bestSf,
            static_cast<double>(bestUp), static_cast<double>(bestDn));
    }
    return result;
}

static Layer2Result layer2Cad(RxStream& stream, SoapyDevice& dev,
                              double freq, double bw,
                              const ScanConfig& cfg, ScanStats& stats) {
    const double sampleRate = bw * static_cast<double>(cfg.os_factor);
    retune(stream, dev, freq, sampleRate, cfg.settle_ms);

    auto detectors = buildDetectors(bw, cfg.os_factor);

    Layer2Result result;
    result.windows_tried = cfg.cad_windows;

    // Use shared SF12-sized buffer: capture once, test all SFs on same data.
    // The chirp in the buffer matches exactly one SF.  This avoids the
    // temporal offset between per-SF captures that causes SF misidentification
    // on short bursts.
    const uint32_t sf12Win = (1U << 12U) * cfg.os_factor * 2U;
    std::vector<cf32> buf(sf12Win);

    for (uint32_t win = 0; win < cfg.cad_windows; ++win) {
        if (g_stop.load(std::memory_order_relaxed)) {
            break;
        }

        capture(stream, buf.data(), sf12Win, stats);
        const auto r = detectOnBuffer(buf.data(), detectors, freq, bw);

        if (r.detected) {
            const double ratio = std::max(r.peak_ratio_up, r.peak_ratio_dn);
            if (ratio > std::max(result.peak_ratio_up, result.peak_ratio_dn)) {
                result.detected      = true;
                result.detections   += 1;
                result.sf_detected   = r.sf_detected;
                result.peak_ratio_up = std::max(result.peak_ratio_up, r.peak_ratio_up);
                result.peak_ratio_dn = std::max(result.peak_ratio_dn, r.peak_ratio_dn);
            }
            break;  // one confirmed window is enough
        }
    }

    gr::lora::log_ts("debug", "lora_scan",
        "L2 %.0f Hz BW=%.0f  %u/%u detected  SF=%u  max_up=%.2f max_dn=%.2f",
        freq, bw, result.detections, result.windows_tried,
        result.sf_detected,
        result.peak_ratio_up, result.peak_ratio_dn);

    return result;
}

// ─── Mode C: direct-CAD sweep (no L1) ───────────────────────────────────────
//
// For narrow scan windows where all channels can be probed directly.
// Iterates every channel × every BW with one SF12-sized CAD window each.
// No L1 FFT, no retune-per-snapshot overhead — just sequential CAD.
//
// Sweep time at BW=62.5k, os=4, 33 channels, 3 BWs:
//   SF12 window = 4096*4*2 = 32768 samples → 32768/(62500*4) = 131 ms
//   33 ch × 3 BW × (131 ms capture + 5 ms settle) ≈ 13.5 s per sweep
//   — still too slow for short bursts, but each individual BW-trial on a
//     channel is a continuous 131 ms capture that catches any chirp present.
//
// Optimisation: for wider BWs, the SF12 capture is shorter (32768/(125k*4)
// = 65 ms at BW125k, 33 ms at BW250k), so wider BWs sweep faster.

struct DirectCadResult {
    double   freq          = 0.0;
    double   bw            = 0.0;
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t sf_detected   = 0;
};

static std::vector<DirectCadResult> directCadSweep(
        RxStream& stream, SoapyDevice& dev,
        const std::vector<double>& channels,
        const ScanConfig& cfg, ScanStats& stats) {
    std::vector<DirectCadResult> results;

    // Pre-build detectors for each BW.
    struct BwDetectors {
        double bw;
        std::vector<SfDetector> dets;
    };
    std::vector<BwDetectors> bwDets;
    for (const double bw : cfg.bws) {
        BwDetectors bd;
        bd.bw   = bw;
        bd.dets = buildDetectors(bw, cfg.os_factor);
        bwDets.push_back(std::move(bd));
    }

    for (const double freq : channels) {
        if (g_stop.load(std::memory_order_relaxed)) {
            break;
        }

        double       bestRatio = 0.0;
        DirectCadResult bestResult;

        for (auto& bd : bwDets) {
            if (g_stop.load(std::memory_order_relaxed)) {
                break;
            }

            const double sampleRate = bd.bw * static_cast<double>(cfg.os_factor);
            retune(stream, dev, freq, sampleRate, cfg.settle_ms);

            // Per-SF capture: each SF trial gets its own correctly-sized
            // buffer.  This avoids SF misidentification from chirp boundary
            // misalignment in a shared buffer, at the cost of slightly more
            // capture time.  Mode C is the "thorough" path, so accuracy
            // matters more than speed here.
            for (auto& sd : bd.dets) {
                if (g_stop.load(std::memory_order_relaxed)) {
                    break;
                }
                const uint32_t sfWin = (1U << sd.sf) * cfg.os_factor * 2U;
                std::vector<cf32> sfBuf(sfWin);
                capture(stream, sfBuf.data(), sfWin, stats);
                const auto r = sd.det.detect(sfBuf.data());
                if (r.detected) {
                    const double ratio = std::max(
                        static_cast<double>(r.peak_ratio_up),
                        static_cast<double>(r.peak_ratio_dn));
                    if (ratio > bestRatio) {
                        bestRatio         = ratio;
                        bestResult.freq   = freq;
                        bestResult.bw     = bd.bw;
                        bestResult.peak_ratio_up = static_cast<double>(r.peak_ratio_up);
                        bestResult.peak_ratio_dn = static_cast<double>(r.peak_ratio_dn);
                        bestResult.sf_detected   = sd.sf;

                        gr::lora::log_ts("debug", "lora_scan",
                            "C: %.0f Hz BW=%.0f SF%u  ratio_up=%.2f ratio_dn=%.2f",
                            freq, bd.bw, sd.sf,
                            static_cast<double>(r.peak_ratio_up),
                            static_cast<double>(r.peak_ratio_dn));
                    }
                }
            }
        }

        if (bestRatio > 0.0) {
            results.push_back(bestResult);
        }
    }

    return results;
}

// ─── Mode B: interleaved L1 snapshot + immediate L2 ─────────────────────────
//
// Take one L1 FFT snapshot, check for HOT channels, immediately run L2 CAD
// on the top-1 HOT channel before taking the next snapshot.  This cuts the
// L1→L2 gap from ~2s (full sweep) to ~8ms (single FFT + retune).
//
// After the interleaved pass, any remaining un-probed HOT channels from the
// last snapshot are processed in a conventional L2 pass.

struct InterleavedResult {
    std::vector<DirectCadResult> detections;
    uint32_t                     l1_hot_total = 0;
};

static InterleavedResult interleavedSweep(
        RxStream& stream, SoapyDevice& dev,
        const std::vector<double>& channels,
        const ScanConfig& cfg, ScanStats& stats) {
    InterleavedResult ir;

    const std::size_t nCh    = channels.size();
    const double      chBw   = cfg.min_bw();

    // Pre-build detectors for each BW (reused across snapshots).
    struct BwDetectors {
        double bw;
        std::vector<SfDetector> dets;
    };
    std::vector<BwDetectors> bwDets;
    for (const double bw : cfg.bws) {
        BwDetectors bd;
        bd.bw      = bw;
        bd.dets    = buildDetectors(bw, cfg.os_factor);
        bwDets.push_back(std::move(bd));
    }

    // L1 FFT infrastructure — same as measureTile but for single snapshots.
    const double tileRate   = cfg.l1_rate;
    const double tileCentre = (channels.front() + channels.back()) / 2.0;

    constexpr std::size_t kMinBinsPerChannel = 256;
    std::size_t fftSize = 1;
    while (fftSize < nCh * kMinBinsPerChannel) {
        fftSize <<= 1U;
    }
    if (fftSize < 4096) {
        fftSize = 4096;
    }

    std::vector<float> hann(fftSize);
    for (std::size_t idx = 0; idx < fftSize; ++idx) {
        hann[idx] = static_cast<float>(
            0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(idx)
                                  / static_cast<double>(fftSize - 1)));
    }

    const double hzPerBin   = tileRate / static_cast<double>(fftSize);
    const double halfChBins = (chBw / tileRate) * static_cast<double>(fftSize) / 2.0;
    const double halfTile   = tileRate / 2.0;

    constexpr double kL1Multiplier = 6.0;
    // Snapshot count scales with channel count × BWs to give each channel
    // multiple chances to be caught.  At ~1 ms per L1 snapshot (+ ~300 ms
    // per L2 retune/capture when HOT), 5 × nCh × nBW snapshots covers the
    // equivalent dwell time that mode C would use sequentially, but with
    // continuous L1 monitoring throughout.
    const int kSnapshots = static_cast<int>(
        5U * nCh * cfg.bws.size());

    std::vector<cf32> fftBuf(fftSize);
    std::vector<double> energy(nCh, 0.0);
    gr::algorithm::FFT<cf32> fft;

    // Track which channels have already been probed this sweep.
    std::vector<bool> probed(nCh, false);

    // Tune to wideband for L1.
    retune(stream, dev, tileCentre, tileRate, cfg.settle_ms);

    for (int snap = 0; snap < kSnapshots; ++snap) {
        if (g_stop.load(std::memory_order_relaxed)) {
            break;
        }

        // If we're not at L1 rate (because we just did an L2 retune), retune back.
        // We track this with a flag to avoid unnecessary retunes.
        // (First snapshot is already tuned from above.)

        // Take one L1 FFT snapshot.
        capture(stream, fftBuf.data(), static_cast<uint32_t>(fftSize), stats);

        for (std::size_t idx = 0; idx < fftSize; ++idx) {
            fftBuf[idx] *= hann[idx];
        }
        auto spectrum = fft.compute(std::span<const cf32>(fftBuf.data(), fftSize));

        // Compute per-channel energy for this snapshot.
        std::ranges::fill(energy, 0.0);
        for (std::size_t chIdx = 0; chIdx < nCh; ++chIdx) {
            const double chOffset = channels[chIdx] - tileCentre;
            if (std::abs(chOffset) > halfTile * 0.9) {
                continue;
            }
            const double binCentre = chOffset / hzPerBin;
            const int kLo = static_cast<int>(std::round(binCentre - halfChBins));
            const int kHi = static_cast<int>(std::round(binCentre + halfChBins));

            double sum = 0.0;
            int    cnt = 0;
            for (int k = kLo; k <= kHi; ++k) {
                const auto binIdx = static_cast<std::size_t>(
                    (k % static_cast<int>(fftSize) + static_cast<int>(fftSize))
                    % static_cast<int>(fftSize));
                sum += static_cast<double>(std::norm(spectrum[binIdx]));
                ++cnt;
            }
            if (cnt > 0) {
                energy[chIdx] = sum / static_cast<double>(cnt);
            }
        }

        // Median threshold.
        std::vector<double> sorted(energy.begin(), energy.end());
        std::ranges::sort(sorted);
        const double median = (nCh % 2 == 1)
            ? sorted[nCh / 2]
            : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
        const double thresh = median * kL1Multiplier;

        // Find the hottest un-probed channel (fine-grid or wideband-aggregated).
        int    bestIdx    = -1;
        double bestEnergy = 0.0;

        // Fine-grid check.
        for (std::size_t idx = 0; idx < nCh; ++idx) {
            if (!probed[idx] && energy[idx] > thresh && energy[idx] > bestEnergy) {
                bestIdx    = static_cast<int>(idx);
                bestEnergy = energy[idx];
            }
        }

        // Wideband aggregation check.
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
            continue;  // nothing hot this snapshot
        }

        ++ir.l1_hot_total;
        const double freq = channels[static_cast<std::size_t>(bestIdx)];

        gr::lora::log_ts("debug", "lora_scan",
            "B: snap %d/%d  HOT %.0f Hz  energy=%.6f (thresh=%.6f)  → immediate L2",
            snap + 1, kSnapshots, freq, bestEnergy, thresh);

        // Run L2 on this channel at each BW (narrowest first).
        // Within each BW: capture one SF12-sized buffer, test all SFs on
        // the same data.  This ensures the correct SF always produces the
        // highest ratio (full 2-symbol correlation vs partial for wrong SFs).
        // Per-BW retune prevents cross-BW false detections.
        double bestRatio = 0.0;
        DirectCadResult bestResult;

        for (auto& bd : bwDets) {
            if (g_stop.load(std::memory_order_relaxed)) {
                break;
            }

            const double sampleRate = bd.bw * static_cast<double>(cfg.os_factor);
            retune(stream, dev, freq, sampleRate, cfg.settle_ms);

            const uint32_t sf12Win = (1U << 12U) * cfg.os_factor * 2U;
            std::vector<cf32> cadBuf(sf12Win);
            capture(stream, cadBuf.data(), sf12Win, stats);

            const auto r = detectOnBuffer(cadBuf.data(), bd.dets, freq, bd.bw);
            if (r.detected) {
                const double ratio = std::max(r.peak_ratio_up, r.peak_ratio_dn);
                if (ratio > bestRatio) {
                    bestRatio         = ratio;
                    bestResult.freq   = freq;
                    bestResult.bw     = bd.bw;
                    bestResult.peak_ratio_up = r.peak_ratio_up;
                    bestResult.peak_ratio_dn = r.peak_ratio_dn;
                    bestResult.sf_detected   = r.sf_detected;
                }
            }

            // Early exit: if we got a strong match at this BW, don't try
            // wider BWs (which could produce weaker cross-BW false hits).
            if (bestRatio > 10.0) {
                break;
            }
        }

        // Retune back to wideband for next L1 snapshot.
        retune(stream, dev, tileCentre, tileRate, cfg.settle_ms);

        // Record result.
        probed[static_cast<std::size_t>(bestIdx)] = true;

        if (bestRatio > 0.0) {
            ir.detections.push_back(bestResult);
        }
    }

    return ir;
}

// ─── UTC time helper for stdout table: "HH:MM:SS.mmm" ────────────────────────

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
        return rc == 2 ? 0 : 1;  // 2 = --help/--version (clean exit)
    }

    std::signal(SIGINT, sigintHandler);

    const auto channels = makeChannelList(cfg.freq_start, cfg.freq_stop, cfg.min_bw());

    // Compute dwell time per channel for display.
    // Each window captures SF12-sized buffers (worst case); dwell at narrowest BW.
    constexpr uint32_t kSF12Len = 4096U;  // 2^12
    const double dwellPerCh = static_cast<double>(kSF12Len * cfg.os_factor * 2U * cfg.cad_windows)
                              / (cfg.min_bw() * static_cast<double>(cfg.os_factor));

    // Build BW list string for info log.
    std::string bwListStr;
    for (std::size_t i = 0; i < cfg.bws.size(); ++i) {
        if (i > 0) bwListStr += ",";
        char tmp[32];
        std::snprintf(tmp, sizeof(tmp), "%.0f", cfg.bws[i]);
        bwListStr += tmp;
    }
    gr::lora::log_ts("info ", "lora_scan",
        "%zu channel(s) SF7-12 BW=[%s] Hz os=%u L1=%.1f MS/s",
        channels.size(), bwListStr.c_str(),
        cfg.os_factor, cfg.l1_rate / 1.0e6);
    // Total dwell accounts for all BWs: first BW gets full windows, rest get quick.
    const uint32_t qw = std::max(4U, cfg.cad_windows / 8U);
    const double totalDwellPerCh = dwellPerCh
        + dwellPerCh * static_cast<double>(qw) / static_cast<double>(cfg.cad_windows)
          * static_cast<double>(cfg.bws.size() > 1 ? cfg.bws.size() - 1 : 0);
    gr::lora::log_ts("info ", "lora_scan",
        "%u CAD windows/ch  ~%.0f ms dwell/ch  ~%.1f s/sweep  max-l2=%u",
        cfg.cad_windows, totalDwellPerCh * 1000.0,
        totalDwellPerCh * static_cast<double>(channels.size()), cfg.max_l2);
    if (cfg.sweeps == 0) {
        gr::lora::log_ts("info ", "lora_scan", "continuous mode — Ctrl+C to stop");
    } else {
        gr::lora::log_ts("info ", "lora_scan", "%u sweep(s)", cfg.sweeps);
    }

    // FPGA workaround: append default FPGA image path for UHD devices.
    if (cfg.device == "uhd" && cfg.device_param.find("fpga=") == std::string::npos) {
        cfg.device_param += ",fpga=usrp_b210_fpga.bin";
    }

    // Open SDR — same pattern as lora_trx.
    auto dev = openDevice(cfg);
    dev.setGain(SOAPY_SDR_RX, 0, cfg.gain);

    const double initialRate = cfg.min_bw() * static_cast<double>(cfg.os_factor);
    dev.setSampleRate(SOAPY_SDR_RX, 0, initialRate);
    dev.setCenterFrequency(SOAPY_SDR_RX, 0, channels.front());

    auto stream = dev.setupStream<cf32, SOAPY_SDR_RX>();
    stream.activate();

    ScanStats stats;

    // ── Layer 1 only mode ────────────────────────────────────────────────────
    if (cfg.layer1_only) {
        std::printf("%-15s  %12s  %6s\n", "freq_hz", "energy_lin", "L1");
        std::printf("%s\n", std::string(38, '-').c_str());

        while (!g_stop.load(std::memory_order_relaxed)) {
            const auto l1 = layer1Scan(stream, dev, channels, cfg, stats);
            for (std::size_t idx = 0; idx < channels.size(); ++idx) {
                const bool hot = std::ranges::find(l1.hot_idx, static_cast<int>(idx)) != l1.hot_idx.end();
                std::printf("%-15.0f  %12.6f  %6s\n",
                            channels[idx], l1.energy[idx], hot ? "HOT" : "");
            }
            std::fflush(stdout);
            if (++stats.sweeps >= cfg.sweeps && cfg.sweeps > 0) {
                break;
            }
        }
        stream.deactivate();
        return 0;
    }

    // ── Full scan: adaptive mode selection ─────────────────────────────────

    // Print header once.
    std::printf("%-13s  %10s  %6s  %6s  %6s  %6s  %4s\n",
                "time", "freq_mhz", "bw_khz", "up", "dn", "wins", "sf");
    std::printf("%s\n", std::string(60, '-').c_str());
    std::fflush(stdout);

    auto lastStatusUpdate = std::chrono::steady_clock::now();

    // Select scan mode based on scan range and channel count.
    //
    // Mode C (direct CAD): skip L1, probe every channel × BW directly.
    //   Best for narrow windows (≤ ~40 channels).  Each channel gets a
    //   131 ms SF12-sized capture — no L1→L2 timing gap.
    //
    // Mode B (interleaved): take one L1 FFT snapshot, immediately run L2
    //   on the top-1 HOT channel, repeat.  Cuts L1→L2 gap to ~8 ms.
    //   Used when scan range fits in one L1 tile but is too wide for C.
    //
    // Mode A (full sweep): L1 tiles + sequential L2.  For wide scans.
    //
    // B+C combination: run C first (catches signals present now), then
    // run B for the remainder of the sweep (catches signals that appear
    // during the B phase).  This maximises detection probability.

    const double scanRange = channels.back() - channels.front() + cfg.min_bw();
    const double usableBw  = cfg.l1_rate * 0.8;
    const bool   singleTile = scanRange <= usableBw;

    constexpr std::size_t kDirectCadMaxChannels = 64;
    const bool useDirectCad = channels.size() <= kDirectCadMaxChannels;
    const bool useInterleaved = singleTile;

    // Mode B subsumes C when both apply (B runs enough snapshots to cover
    // the full dwell).  C is only standalone for non-interleaved narrow scans.
    const char* modeName = useInterleaved
        ? "B (interleaved L1+L2)"
        : (useDirectCad ? "C (direct CAD)" : "A (full L1+L2 sweep)");
    gr::lora::log_ts("info ", "lora_scan",
        "scan mode: %s  (%zu ch, range=%.1f MHz, tile=%.1f MHz)",
        modeName, channels.size(), scanRange / 1.0e6, usableBw / 1.0e6);

    // Helper: report a detection result to stdout and update stats.
    auto reportDetection = [&](const DirectCadResult& det) {
        const double ratio = std::max(det.peak_ratio_up, det.peak_ratio_dn);
        if (ratio < static_cast<double>(cfg.min_ratio)) {
            return;
        }

        const auto ts = ts_short();
        const std::string sfStr = det.sf_detected
            ? ("SF" + std::to_string(det.sf_detected)) : "-";

        std::fprintf(stderr, "\r%s\r", std::string(80, ' ').c_str());
        std::printf("%-13s  %10.3f  %6.1f  %6.1f  %6.1f  %6s  %4s\n",
                    ts.c_str(), det.freq / 1.0e6, det.bw / 1.0e3,
                    det.peak_ratio_up, det.peak_ratio_dn,
                    "1/1", sfStr.c_str());
        std::fflush(stdout);

        char freqBuf[32];
        std::snprintf(freqBuf, sizeof(freqBuf), "%.3f MHz", det.freq / 1.0e6);
        stats.last_det_freq = freqBuf;
        stats.last_det_time = ts;
    };

    // For mode A fallback: secondary BWs get fewer CAD windows.
    const uint32_t quickWindows = std::max(4U, cfg.cad_windows / 8U);

    while (!g_stop.load(std::memory_order_relaxed)) {

        // ── Phase 1: Mode B (interleaved L1+L2)
        // When available, mode B is the primary scan mode.  Its snapshot
        // count scales with the channel/BW count so it covers the full
        // expected dwell time with continuous L1 monitoring.  Mode C is
        // only used as a standalone fallback for very small channel counts
        // where the overhead of L1 FFTs is worse than sequential probing.
        if (useInterleaved) {
            const auto bResult = interleavedSweep(stream, dev, channels, cfg, stats);
            for (const auto& det : bResult.detections) {
                reportDetection(det);
            }
            stats.l1_hot = bResult.l1_hot_total;
        }

        // ── Phase 2: Mode C (direct CAD) — only when B is not available
        if (useDirectCad && !useInterleaved
            && !g_stop.load(std::memory_order_relaxed)) {
            const auto cResults = directCadSweep(stream, dev, channels, cfg, stats);
            for (const auto& det : cResults) {
                reportDetection(det);
            }
            stats.l1_hot = static_cast<uint32_t>(cResults.size());
        }

        // ── Phase 3: Mode A fallback (full L1 + sequential L2) ─────────
        if (!useDirectCad && !useInterleaved
            && !g_stop.load(std::memory_order_relaxed)) {
            const auto l1 = layer1Scan(stream, dev, channels, cfg, stats);
            stats.l1_hot = static_cast<uint32_t>(l1.hot_idx.size());

            std::vector<std::size_t> l2_candidates;
            if (cfg.all_channels) {
                for (std::size_t i = 0; i < channels.size(); ++i) {
                    l2_candidates.push_back(i);
                }
            } else {
                for (int hi : l1.hot_idx) {
                    l2_candidates.push_back(static_cast<std::size_t>(hi));
                }
            }
            if (l2_candidates.size() > cfg.max_l2) {
                l2_candidates.resize(cfg.max_l2);
            }

            for (std::size_t idx : l2_candidates) {
                if (g_stop.load(std::memory_order_relaxed)) {
                    break;
                }
                const double freq = channels[idx];
                double       bestRatio = 0.0;
                Layer2Result bestL2;
                double       bestBw = 0.0;

                for (std::size_t bwIdx = 0; bwIdx < cfg.bws.size(); ++bwIdx) {
                    if (g_stop.load(std::memory_order_relaxed)) {
                        break;
                    }
                    const double bw = cfg.bws[bwIdx];
                    const uint32_t windows = (bwIdx == 0)
                        ? cfg.cad_windows : quickWindows;
                    ScanConfig bwCfg = cfg;
                    bwCfg.cad_windows = windows;
                    const auto l2 = layer2Cad(stream, dev, freq, bw, bwCfg, stats);
                    if (l2.detected) {
                        const double ratio = std::max(l2.peak_ratio_up, l2.peak_ratio_dn);
                        if (ratio > bestRatio) {
                            bestRatio = ratio;
                            bestBw    = bw;
                            bestL2    = l2;
                        }
                    }
                }

                if (bestRatio >= static_cast<double>(cfg.min_ratio)) {
                    DirectCadResult det;
                    det.freq          = freq;
                    det.bw            = bestBw;
                    det.peak_ratio_up = bestL2.peak_ratio_up;
                    det.peak_ratio_dn = bestL2.peak_ratio_dn;
                    det.sf_detected   = bestL2.sf_detected;
                    reportDetection(det);
                }
            }
        }

        ++stats.sweeps;
        if (cfg.sweeps > 0 && stats.sweeps >= cfg.sweeps) {
            break;
        }

        // Status line on stderr — rate-limited to avoid jitter.
        const auto now = std::chrono::steady_clock::now();
        if (!g_stop.load(std::memory_order_relaxed)
            && (now - lastStatusUpdate) >= std::chrono::milliseconds(500)) {
            lastStatusUpdate = now;
            const std::string lastDet = stats.last_det_freq.empty()
                ? "none"
                : (stats.last_det_freq + " " + stats.last_det_time);
            std::fprintf(stderr, "\rsweep %u  hot %u/%zu  OVF %u  DROP %u  last: %s   ",
                         stats.sweeps, stats.l1_hot, channels.size(),
                         stats.overflows, stats.drops, lastDet.c_str());
        }
    }

    // Final newline so shell prompt doesn't overwrite status line.
    std::fputc('\n', stderr);
    gr::lora::log_ts("info ", "lora_scan",
        "stopped after %u sweep(s), %u overflow(s), %u drop(s)",
        stats.sweeps, stats.overflows, stats.drops);
    stream.deactivate();

    // Avoid SoapySDR unloadModules() hang on exit.
    std::quick_exit(0);
}
