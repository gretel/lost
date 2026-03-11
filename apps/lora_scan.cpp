// SPDX-License-Identifier: ISC
//
// lora_scan — LoRa spectral scanner
//
// Layer 1: wideband FFT energy scan → candidate channels
// Layer 2: per-channel CAD dwell with shared-buffer-per-BW detection
//
// Run `lora_scan --help` for usage.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>

// common.hpp must come AFTER Soapy.hpp (uses soapy types, no SoapySDR include)
#include "common.hpp"

#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/log.hpp>

#ifndef GIT_REV
#define GIT_REV "dev"
#endif

using lora_apps::cf32;
using SoapyDevice = gr::blocks::soapy::Device;
using RxStream    = SoapyDevice::Stream<cf32, SOAPY_SDR_RX>;

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
    uint32_t             cad_windows  = 8;
    uint32_t             max_l2       = 8;
    float                min_ratio    = 8.0F;
    uint32_t             sweeps       = 0;
    bool                 layer1_only  = false;
    bool                 all_channels = false;
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
        "  --cad-windows <K>     CAD windows per BW per channel (default: 8)\n"
        "  --max-l2 <N>          Max L2 candidates per sweep (default: 8)\n"
        "  --min-ratio <f>       Min peak ratio to report (default: 8.0)\n"
        "  --sweeps <N>          Stop after N sweeps (default: 0 = infinite)\n"
        "  --layer1-only         Stop after Layer 1 (energy scan only)\n"
        "  --all-channels        Run CAD on all channels, not just hot ones\n"
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
        else if (arg == "--cad-windows")   cfg.cad_windows  = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--max-l2")        cfg.max_l2       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--min-ratio")     cfg.min_ratio    = std::stof(next());
        else if (arg == "--sweeps")        cfg.sweeps       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--layer1-only")   cfg.layer1_only  = true;
        else if (arg == "--all-channels")  cfg.all_channels = true;
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
                "L1 rate %.0f Hz / (BW %.0f Hz × os %u) = %.3f — must be an integer\n",
                cfg.l1_rate, bw, cfg.os_factor, factor);
            return 1;
        }
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

/// Frequency-only retune — sample rate stays at L1 rate throughout the scan.
/// No deactivate/activate cycle, eliminating USB buffer overflows from rate switching.
static void retune(SoapyDevice& dev, double freq, int settleMs) {
    dev.setCenterFrequency(SOAPY_SDR_RX, 0, freq);
    std::this_thread::sleep_for(std::chrono::milliseconds(settleMs));
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

static void measureTile(RxStream& stream, SoapyDevice& dev,
                        const std::vector<double>& channels,
                        double tileCentre, double tileRate,
                        double chBw, int settleMs,
                        std::vector<double>& energy, ScanStats& stats) {
    retune(dev, tileCentre, settleMs);

    const std::size_t nCh = channels.size();
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

    constexpr int kL1Snapshots = 4;

    std::vector<cf32> buf(fftSize);
    gr::algorithm::FFT<cf32> fft;

    for (int snap = 0; snap < kL1Snapshots; ++snap) {
        capture(stream, buf.data(), static_cast<uint32_t>(fftSize), stats);

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
    const double usableBw  = cfg.l1_rate * 0.8;

    if (scanRange <= usableBw) {
        const double centre = (channels.front() + channels.back()) / 2.0;
        measureTile(stream, dev, channels, centre, cfg.l1_rate,
                    chBw, cfg.settle_ms, result.energy, stats);
    } else {
        const double step = usableBw;
        const double first = channels.front() + usableBw / 2.0;
        const double last  = channels.back()  - usableBw / 2.0;
        for (double tc = first; tc <= last + step * 0.5; tc += step) {
            measureTile(stream, dev, channels, tc, cfg.l1_rate,
                        chBw, cfg.settle_ms, result.energy, stats);
        }
    }

    constexpr double kL1Multiplier = 6.0;
    std::vector<double> sorted(result.energy.begin(), result.energy.end());
    std::ranges::sort(sorted);
    const double median = (nCh % 2 == 1)
        ? sorted[nCh / 2]
        : (sorted[nCh / 2 - 1] + sorted[nCh / 2]) / 2.0;
    const double thresh = median * kL1Multiplier;

    // pass 1: fine-grid detection
    std::vector<bool> isHot(nCh, false);
    for (std::size_t idx = 0; idx < nCh; ++idx) {
        if (result.energy[idx] > thresh) {
            isHot[idx] = true;
        }
    }

    // pass 2: wideband aggregation
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

struct Layer2Result {
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t sf_detected    = 0;
    uint32_t detections     = 0;
    uint32_t windows_tried  = 0;
    bool     detected       = false;
};

struct SfDetector {
    gr::lora::ChannelActivityDetector det;
    uint32_t sf;
};

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
                               const ScanConfig& cfg, ScanStats& stats,
                               const std::atomic<bool>& stopFlag) {
    const double targetRate = bw * static_cast<double>(cfg.os_factor);
    retune(dev, freq, cfg.settle_ms);

    auto detectors = buildDetectors(bw, cfg.os_factor);

    Layer2Result result;
    result.windows_tried = cfg.cad_windows;

    // Capture at L1 rate, then decimate to target rate for CAD.
    const uint32_t sf12Win   = (1U << 12U) * cfg.os_factor * 2U;
    const auto     decFactor = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));
    const uint32_t l1Win     = sf12Win * decFactor;
    std::vector<cf32> l1Buf(l1Win);

    for (uint32_t win = 0; win < cfg.cad_windows; ++win) {
        if (stopFlag.load(std::memory_order_relaxed)) {
            break;
        }

        capture(stream, l1Buf.data(), l1Win, stats);
        const auto cadBuf = decimate(l1Buf.data(), l1Win, cfg.l1_rate, targetRate);
        const auto r = detectOnBuffer(cadBuf.data(), detectors, freq, bw);

        if (r.detected) {
            const double ratio = std::max(r.peak_ratio_up, r.peak_ratio_dn);
            if (ratio > std::max(result.peak_ratio_up, result.peak_ratio_dn)) {
                result.detected      = true;
                result.detections   += 1;
                result.sf_detected   = r.sf_detected;
                result.peak_ratio_up = std::max(result.peak_ratio_up, r.peak_ratio_up);
                result.peak_ratio_dn = std::max(result.peak_ratio_dn, r.peak_ratio_dn);
            }
            break;
        }
    }

    return result;
}

// ─── Mode C: direct-CAD sweep (no L1) ───────────────────────────────────────

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
        const ScanConfig& cfg, ScanStats& stats,
        const std::atomic<bool>& stopFlag) {
    std::vector<DirectCadResult> results;

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
        if (stopFlag.load(std::memory_order_relaxed)) {
            break;
        }

        retune(dev, freq, cfg.settle_ms);

        double       bestRatio = 0.0;
        DirectCadResult bestResult;

        for (auto& bd : bwDets) {
            if (stopFlag.load(std::memory_order_relaxed)) {
                break;
            }

            const double targetRate = bd.bw * static_cast<double>(cfg.os_factor);
            const auto   decFactor  = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));

            for (auto& sd : bd.dets) {
                if (stopFlag.load(std::memory_order_relaxed)) {
                    break;
                }
                const uint32_t sfWin = (1U << sd.sf) * cfg.os_factor * 2U;
                const uint32_t l1Win = sfWin * decFactor;
                std::vector<cf32> l1Buf(l1Win);
                capture(stream, l1Buf.data(), l1Win, stats);
                const auto cadBuf = decimate(l1Buf.data(), l1Win, cfg.l1_rate, targetRate);
                const auto r = sd.det.detect(cadBuf.data());
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

struct InterleavedResult {
    std::vector<DirectCadResult> detections;
    std::vector<double>          energy;
    uint32_t                     l1_hot_total = 0;
};

static InterleavedResult interleavedSweep(
        RxStream& stream, SoapyDevice& dev,
        const std::vector<double>& channels,
        const ScanConfig& cfg, ScanStats& stats,
        const std::atomic<bool>& stopFlag) {
    InterleavedResult ir;

    const std::size_t nCh    = channels.size();
    const double      chBw   = cfg.min_bw();
    ir.energy.resize(nCh, 0.0);

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
    const int kSnapshots = static_cast<int>(5U * nCh * cfg.bws.size());

    std::vector<cf32> fftBuf(fftSize);
    std::vector<double> energy(nCh, 0.0);
    gr::algorithm::FFT<cf32> fft;

    std::vector<bool> probed(nCh, false);

    retune(dev, tileCentre, cfg.settle_ms);

    for (int snap = 0; snap < kSnapshots; ++snap) {
        if (stopFlag.load(std::memory_order_relaxed)) {
            break;
        }

        capture(stream, fftBuf.data(), static_cast<uint32_t>(fftSize), stats);

        for (std::size_t idx = 0; idx < fftSize; ++idx) {
            fftBuf[idx] *= hann[idx];
        }
        auto spectrum = fft.compute(std::span<const cf32>(fftBuf.data(), fftSize));

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

        for (std::size_t chIdx = 0; chIdx < nCh; ++chIdx) {
            ir.energy[chIdx] = std::max(ir.energy[chIdx], energy[chIdx]);
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
            continue;
        }

        ++ir.l1_hot_total;
        const double freq = channels[static_cast<std::size_t>(bestIdx)];

        gr::lora::log_ts("debug", "lora_scan",
            "B: snap %d/%d  HOT %.0f Hz  energy=%.6f (thresh=%.6f)  → immediate L2",
            snap + 1, kSnapshots, freq, bestEnergy, thresh);

        double bestRatio = 0.0;
        DirectCadResult bestResult;

        // Retune to hot channel for L2 CAD (frequency-only, stays at L1 rate)
        retune(dev, freq, cfg.settle_ms);

        for (auto& bd : bwDets) {
            if (stopFlag.load(std::memory_order_relaxed)) {
                break;
            }

            const double   targetRate = bd.bw * static_cast<double>(cfg.os_factor);
            const auto     decFactor  = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));
            const uint32_t sf12Win    = (1U << 12U) * cfg.os_factor * 2U;
            const uint32_t l1Win      = sf12Win * decFactor;
            std::vector<cf32> l1Buf(l1Win);
            capture(stream, l1Buf.data(), l1Win, stats);
            const auto cadBuf = decimate(l1Buf.data(), l1Win, cfg.l1_rate, targetRate);

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

            if (bestRatio > 10.0) {
                break;
            }
        }

        // Return to tile centre for next L1 snapshot (frequency-only)
        retune(dev, tileCentre, cfg.settle_ms);

        probed[static_cast<std::size_t>(bestIdx)] = true;

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
        cb::encode_map_begin(buf, 5);
        cb::kv_uint(buf, "freq", static_cast<uint64_t>(det.freq));
        cb::kv_uint(buf, "sf", det.sf_detected);
        cb::kv_uint(buf, "bw", static_cast<uint64_t>(det.bw));
        cb::kv_float64(buf, "ratio", std::max(det.peak_ratio_up, det.peak_ratio_dn));
        const char* chirp = (det.peak_ratio_up > 0 && det.peak_ratio_dn > 0) ? "both"
                          : (det.peak_ratio_dn > det.peak_ratio_up)           ? "dn"
                          :                                                     "up";
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

    constexpr uint32_t kSF12Len = 4096U;
    const double dwellPerCh = static_cast<double>(kSF12Len * cfg.os_factor * 2U * cfg.cad_windows)
                              / (cfg.min_bw() * static_cast<double>(cfg.os_factor));

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

    lora_apps::apply_fpga_workaround(cfg.device, cfg.device_param);

    // stdout protection: UHD/SoapySDR may print to stdout during device init
    if (cfg.cbor_out) {
        std::fflush(stdout);
        savedStdout = dup(STDOUT_FILENO);
        dup2(STDERR_FILENO, STDOUT_FILENO);
    }

    lora_apps::log_hardware_info("lora_scan", cfg.device, cfg.device_param);

    auto dev = openDevice(cfg);

    // Pin FPGA master clock to prevent 32↔16 MHz oscillation during retune.
    // Must be called before setSampleRate() — subsequent rate changes negotiate
    // with a fixed master clock.
    if (cfg.master_clock > 0.0) {
        dev.setMasterClockRate(cfg.master_clock);
    }

    dev.setGain(SOAPY_SDR_RX, 0, cfg.gain);

    // Set L1 rate once and keep it for the entire scan session.
    // L2 CAD uses software decimation instead of hardware rate switching,
    // eliminating USB buffer overflows from AD9361 reconfiguration.
    dev.setSampleRate(SOAPY_SDR_RX, 0, cfg.l1_rate);
    dev.setCenterFrequency(SOAPY_SDR_RX, 0, channels.front());

    auto stream = dev.setupStream<cf32, SOAPY_SDR_RX>();
    stream.activate();

    if (savedStdout >= 0) {
        dup2(savedStdout, STDOUT_FILENO);
        close(savedStdout);
        savedStdout = -1;
    }

    ScanStats stats;

    // ── Layer 1 only mode ────────────────────────────────────────────────────
    if (cfg.layer1_only) {
        auto* out = cfg.cbor_out ? stderr : stdout;
        std::fprintf(out, "%-15s  %12s  %6s\n", "freq_hz", "energy_lin", "L1");
        std::fprintf(out, "%s\n", std::string(38, '-').c_str());

        while (!g_stop.load(std::memory_order_relaxed)) {
            const auto l1 = layer1Scan(stream, dev, channels, cfg, stats);
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
        stream.deactivate();
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

    auto lastStatusUpdate = std::chrono::steady_clock::now();

    const double scanRange = channels.back() - channels.front() + cfg.min_bw();
    const double usableBw  = cfg.l1_rate * 0.8;
    const bool   singleTile = scanRange <= usableBw;

    constexpr std::size_t kDirectCadMaxChannels = 64;
    const bool useDirectCad = channels.size() <= kDirectCadMaxChannels;
    const bool useInterleaved = singleTile;

    const char* modeName = useInterleaved
        ? "B (interleaved L1+L2)"
        : (useDirectCad ? "C (direct CAD)" : "A (full L1+L2 sweep)");
    gr::lora::log_ts("info ", "lora_scan",
        "scan mode: %s  (%zu ch, range=%.1f MHz, tile=%.1f MHz)",
        modeName, channels.size(), scanRange / 1.0e6, usableBw / 1.0e6);

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
        std::fprintf(stderr, "\r%s\r", std::string(80, ' ').c_str());
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

    const uint32_t quickWindows = std::max(4U, cfg.cad_windows / 8U);

    std::vector<double> sweepEnergy(channels.size(), 0.0);
    auto lastCborStatus = std::chrono::steady_clock::now();

    while (!g_stop.load(std::memory_order_relaxed)) {
        sweepDetections.clear();
        std::ranges::fill(sweepEnergy, 0.0);

        if (useInterleaved) {
            const auto bResult = interleavedSweep(stream, dev, channels, cfg, stats, g_stop);
            for (const auto& det : bResult.detections) {
                reportDetection(det);
            }
            stats.l1_hot = bResult.l1_hot_total;
            sweepEnergy = bResult.energy;
        }

        if (useDirectCad && !useInterleaved
            && !g_stop.load(std::memory_order_relaxed)) {
            const auto cResults = directCadSweep(stream, dev, channels, cfg, stats, g_stop);
            for (const auto& det : cResults) {
                reportDetection(det);
            }
            stats.l1_hot = static_cast<uint32_t>(cResults.size());
        }

        if (!useDirectCad && !useInterleaved
            && !g_stop.load(std::memory_order_relaxed)) {
            const auto l1 = layer1Scan(stream, dev, channels, cfg, stats);
            stats.l1_hot = static_cast<uint32_t>(l1.hot_idx.size());
            sweepEnergy = l1.energy;

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
                    const auto l2 = layer2Cad(stream, dev, freq, bw, bwCfg, stats, g_stop);
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
            emitSpectrumCbor(channels, sweepEnergy, hotIndices,
                             sweepDetections, stats.sweeps);

            const auto now2 = std::chrono::steady_clock::now();
            const std::string modeStr = useInterleaved ? "B"
                : (useDirectCad ? "C" : "A");
            if (stats.sweeps == 1
                || (now2 - lastCborStatus) >= std::chrono::seconds(5)) {
                lastCborStatus = now2;
                emitStatusCbor(cfg, stats, modeStr);
            }
        }

        if (cfg.sweeps > 0 && stats.sweeps >= cfg.sweeps) {
            break;
        }

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

    std::fputc('\n', stderr);
    gr::lora::log_ts("info ", "lora_scan",
        "stopped after %u sweep(s), %u overflow(s), %u drop(s)",
        stats.sweeps, stats.overflows, stats.drops);
    stream.deactivate();

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
