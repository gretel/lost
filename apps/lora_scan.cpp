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
// Usage:
//   lora_scan [options]
//
//   --device <args>       SoapySDR device string (default: "uhd")
//   --device-param <args> Device parameters (default: "type=b200")
//   --freq-start <Hz>     Scan start frequency (default: 863e6)
//   --freq-stop  <Hz>     Scan stop  frequency (default: 870e6)
//   --bw <Hz>             Channel bandwidth (default: 62500)
//   --os <n>              Oversampling factor (default: 4)
//   --sf <n>              Spreading factor for CAD (default: 8)
//   --alpha <f>           CAD threshold (default: SF table)
//   --gain <dB>           RX gain (default: 40)
//   --settle-ms <ms>      PLL settle delay after retune (default: 5)
//   --l1-rate <Hz>        L1 wideband sample rate (default: 8e6)
//   --cad-windows <K>     CAD windows per channel per sweep (default: 64)
//   --sweeps <N>          Stop after N sweeps (default: 0 = infinite)
//   --sf-sweep            Enable Layer 3: SF sweep on detected channels
//   --layer1-only         Stop after Layer 1 (energy scan only)
//   --all-channels        Run CAD on all channels, not just hot ones
//   --debug               Verbose stderr output

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

// SF-dependent alpha table (Vangelista & Calvagno 2022, P_fa = 1e-3).
static const std::array<float, 13> kAlphaTable = {
    0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F,
    4.23F, 4.16F, 4.09F, 4.04F, 3.98F, 3.91F,
};

struct ScanConfig {
    std::string device       = "uhd";
    std::string device_param = "type=b200";
    double      freq_start   = 863.0e6;
    double      freq_stop    = 870.0e6;
    double      bw           = 62500.0;
    uint32_t    sf           = 8;
    uint32_t    os_factor    = 4;
    float       alpha        = 0.F;
    double      gain         = 40.0;
    int         settle_ms    = 5;
    double      l1_rate      = 8.0e6;   // wideband L1 sample rate (even B210 decimation)
    uint32_t    cad_windows  = 64;
    uint32_t    sweeps       = 0;       // 0 = infinite
    bool        sf_sweep     = false;
    bool        layer1_only  = false;
    bool        all_channels = false;
    bool        debug        = false;
};

// ─── CLI ──────────────────────────────────────────────────────────────────────

static bool parseArgs(int argc, char** argv, ScanConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                std::exit(1);
            }
            return argv[i];
        };
        if      (arg == "--device")        cfg.device       = next();
        else if (arg == "--device-param")  cfg.device_param = next();
        else if (arg == "--freq-start")    cfg.freq_start   = std::stod(next());
        else if (arg == "--freq-stop")     cfg.freq_stop    = std::stod(next());
        else if (arg == "--bw")            cfg.bw           = std::stod(next());
        else if (arg == "--sf")            cfg.sf           = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--os")            cfg.os_factor    = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--alpha")         cfg.alpha        = std::stof(next());
        else if (arg == "--gain")          cfg.gain         = std::stod(next());
        else if (arg == "--settle-ms")     cfg.settle_ms    = std::stoi(next());
        else if (arg == "--l1-rate")       cfg.l1_rate      = std::stod(next());
        else if (arg == "--cad-windows")   cfg.cad_windows  = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--sweeps")        cfg.sweeps       = static_cast<uint32_t>(std::stoul(next()));
        else if (arg == "--sf-sweep")      cfg.sf_sweep     = true;
        else if (arg == "--layer1-only")   cfg.layer1_only  = true;
        else if (arg == "--all-channels")  cfg.all_channels = true;
        else if (arg == "--debug")         cfg.debug        = true;
        else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            return false;
        }
    }
    if (cfg.sf < 6 || cfg.sf > 12) {
        std::fprintf(stderr, "sf must be 6..12\n");
        return false;
    }
    if (cfg.alpha == 0.F) {
        cfg.alpha = kAlphaTable[cfg.sf];
    }
    const double scanRange = cfg.freq_stop - cfg.freq_start;
    if (scanRange > cfg.l1_rate * 0.9) {
        gr::lora::log_ts("warn ", "lora_scan",
            "scan range %.1f MHz > L1 rate %.1f MS/s — L1 will tile multiple snapshots",
            scanRange / 1.0e6, cfg.l1_rate / 1.0e6);
    }
    return true;
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
        } else if (ret == SOAPY_SDR_TIMEOUT) {
            ++stats.drops;
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

    std::vector<cf32> buf(fftSize);
    capture(stream, buf.data(), static_cast<uint32_t>(fftSize), stats);

    // Hann window.
    for (std::size_t idx = 0; idx < fftSize; ++idx) {
        const auto w = static_cast<float>(
            0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(idx)
                                  / static_cast<double>(fftSize - 1)));
        buf[idx] *= w;
    }

    gr::algorithm::FFT<cf32> fft;
    auto spectrum = fft.compute(std::span<const cf32>(buf.data(), fftSize));

    const double hzPerBin   = tileRate / static_cast<double>(fftSize);
    const double halfChBins = (chBw / tileRate) * static_cast<double>(fftSize) / 2.0;
    const double halfTile   = tileRate / 2.0;

    for (std::size_t chIdx = 0; chIdx < nCh; ++chIdx) {
        const double chOffset = channels[chIdx] - tileCentre;
        // Skip channels outside this tile's bandwidth (with 10% margin).
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
}

static Layer1Result layer1Scan(RxStream& stream, SoapyDevice& dev,
                               const std::vector<double>& channels,
                               const ScanConfig& cfg, ScanStats& stats) {
    const std::size_t nCh = channels.size();
    Layer1Result result;
    result.energy.resize(nCh, 0.0);

    const double scanRange = channels.back() - channels.front() + cfg.bw;
    const double usableBw  = cfg.l1_rate * 0.8;  // 80% usable (roll-off at edges)

    if (scanRange <= usableBw) {
        const double centre = (channels.front() + channels.back()) / 2.0;
        measureTile(stream, dev, channels, centre, cfg.l1_rate,
                    cfg.bw, cfg.settle_ms, result.energy, stats);
        if (cfg.debug) {
            gr::lora::log_ts("debug", "lora_scan",
                "L1 centre=%.0f rate=%.0f fftBw=%.0f (single tile)",
                centre, cfg.l1_rate, cfg.l1_rate);
        }
    } else {
        const double step = usableBw;
        const double first = channels.front() + usableBw / 2.0;
        const double last  = channels.back()  - usableBw / 2.0;
        int nTiles = 0;
        for (double tc = first; tc <= last + step * 0.5; tc += step) {
            measureTile(stream, dev, channels, tc, cfg.l1_rate,
                        cfg.bw, cfg.settle_ms, result.energy, stats);
            ++nTiles;
        }
        if (cfg.debug) {
            gr::lora::log_ts("debug", "lora_scan",
                "L1 %d tile(s) at %.0f MS/s usable=%.0f Hz",
                nTiles, cfg.l1_rate / 1.0e6, usableBw);
        }
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

    if (cfg.debug) {
        gr::lora::log_ts("debug", "lora_scan",
            "L1 median=%.6f thresh=%.6f (%zu ch)", median, thresh, nCh);
    }

    for (int idx = 0; idx < static_cast<int>(nCh); ++idx) {
        if (result.energy[static_cast<std::size_t>(idx)] > thresh) {
            result.hot_idx.push_back(idx);
        }
    }
    // Sort hot channels by energy (highest first) so the strongest signal
    // gets L2 dwell immediately — critical when ADVERT airtime is ~1.2s
    // and we have many hot channels from ISM wideband noise.
    std::ranges::sort(result.hot_idx, [&](int a, int b) {
        return result.energy[static_cast<std::size_t>(a)]
             > result.energy[static_cast<std::size_t>(b)];
    });
    if (cfg.debug) {
        for (int idx : result.hot_idx) {
            gr::lora::log_ts("debug", "lora_scan",
                "L1 HOT %.0f Hz  energy=%.6f (thresh=%.6f)",
                channels[static_cast<std::size_t>(idx)],
                result.energy[static_cast<std::size_t>(idx)], thresh);
        }
    }
    return result;
}

// ─── Layer 2: per-channel CAD dwell with K consecutive windows ───────────────

struct Layer2Result {
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t detections     = 0;     // how many windows fired
    uint32_t windows_tried  = 0;
    bool     detected       = false;
};

static Layer2Result layer2Cad(RxStream& stream, SoapyDevice& dev,
                              double freq, uint32_t sf, const ScanConfig& cfg,
                              ScanStats& stats) {
    const double sampleRate = cfg.bw * static_cast<double>(cfg.os_factor);

    // Retune only once per channel dwell.
    retune(stream, dev, freq, sampleRate, cfg.settle_ms);

    const uint32_t nSymSamples = (1U << sf) * cfg.os_factor;
    const uint32_t winLen      = nSymSamples * 2U;  // 2-symbol window

    // Build detector once, reuse across all K windows.
    gr::lora::ChannelActivityDetector det;
    det.sf         = sf;
    det.bandwidth  = static_cast<uint32_t>(cfg.bw);
    det.os_factor  = cfg.os_factor;
    det.alpha      = cfg.alpha;
    det.dual_chirp = true;
    det.debug      = false;  // too noisy per-window; use outer debug
    det.start();

    using RefType = gr::lora::ChannelActivityDetector::RefType;

    Layer2Result result;
    result.windows_tried = cfg.cad_windows;

    std::vector<cf32> buf(winLen);

    for (uint32_t win = 0; win < cfg.cad_windows; ++win) {
        if (g_stop.load(std::memory_order_relaxed)) {
            break;
        }

        capture(stream, buf.data(), winLen, stats);

        const cf32* w1 = buf.data();
        const cf32* w2 = buf.data() + nSymSamples;

        const float r1Up = det.compute_peak_ratio(w1, RefType::UpchirpRef);
        const float r2Up = det.compute_peak_ratio(w2, RefType::UpchirpRef);
        const float r1Dn = det.compute_peak_ratio(w1, RefType::DownchirpRef);
        const float r2Dn = det.compute_peak_ratio(w2, RefType::DownchirpRef);

        const bool upDet = (r1Up > cfg.alpha) && (r2Up > cfg.alpha);
        const bool dnDet = (r1Dn > cfg.alpha) && (r2Dn > cfg.alpha);

        if (upDet || dnDet) {
            ++result.detections;
            result.detected = true;
        }

        result.peak_ratio_up = std::max(result.peak_ratio_up, static_cast<double>(std::max(r1Up, r2Up)));
        result.peak_ratio_dn = std::max(result.peak_ratio_dn, static_cast<double>(std::max(r1Dn, r2Dn)));
    }

    if (cfg.debug) {
        gr::lora::log_ts("debug", "lora_scan",
            "L2 %.0f Hz SF%u  %u/%u detected  max_up=%.2f max_dn=%.2f  alpha=%.2f",
            freq, sf, result.detections, result.windows_tried,
            result.peak_ratio_up, result.peak_ratio_dn,
            static_cast<double>(cfg.alpha));
    }

    return result;
}

// ─── Layer 3: SF sweep ────────────────────────────────────────────────────────

static uint32_t layer3SfSweep(RxStream& stream, SoapyDevice& dev,
                              double freq, const ScanConfig& cfg, ScanStats& stats) {
    ScanConfig sweepCfg = cfg;
    sweepCfg.cad_windows = std::max(8U, cfg.cad_windows / 4U);
    sweepCfg.debug = false;

    for (uint32_t sf = 7; sf <= 12; ++sf) {
        sweepCfg.sf    = sf;
        sweepCfg.alpha = kAlphaTable[sf];
        const auto l2 = layer2Cad(stream, dev, freq, sf, sweepCfg, stats);
        if (l2.detected) {
            if (cfg.debug) {
                gr::lora::log_ts("debug", "lora_scan",
                    "L3 SF%u detected at %.0f Hz (%u/%u windows)",
                    sf, freq, l2.detections, l2.windows_tried);
            }
            return sf;
        }
    }
    return 0;
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
    if (!parseArgs(argc, argv, cfg)) {
        return 1;
    }

    std::signal(SIGINT, sigintHandler);

    const auto channels = makeChannelList(cfg.freq_start, cfg.freq_stop, cfg.bw);

    // Compute dwell time per channel for display.
    const uint32_t symLen = (1U << cfg.sf) * cfg.os_factor;
    const double dwellPerCh = static_cast<double>(symLen * 2U * cfg.cad_windows)
                              / (cfg.bw * static_cast<double>(cfg.os_factor));

    gr::lora::log_ts("info ", "lora_scan",
        "%zu channel(s) SF%u BW=%.0f Hz alpha=%.2f os=%u L1=%.1f MS/s",
        channels.size(), cfg.sf, cfg.bw,
        static_cast<double>(cfg.alpha), cfg.os_factor,
        cfg.l1_rate / 1.0e6);
    gr::lora::log_ts("info ", "lora_scan",
        "%u CAD windows/ch  ~%.0f ms dwell/ch  ~%.1f s/sweep",
        cfg.cad_windows, dwellPerCh * 1000.0,
        dwellPerCh * static_cast<double>(channels.size()));
    if (cfg.sweeps == 0) {
        gr::lora::log_ts("info ", "lora_scan", "continuous mode — Ctrl+C to stop");
    } else {
        gr::lora::log_ts("info ", "lora_scan", "%u sweep(s)", cfg.sweeps);
    }

    // Open SDR — same pattern as lora_trx.
    auto dev = openDevice(cfg);
    dev.setGain(SOAPY_SDR_RX, 0, cfg.gain);

    const double initialRate = cfg.bw * static_cast<double>(cfg.os_factor);
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

    // ── Full scan: L1 + L2 + (optional) L3, continuous ──────────────────────

    // Print header once.
    std::printf("%-13s  %-15s  %8s  %8s  %5s  %6s  %6s\n",
                "time", "freq_hz", "ratio_up", "ratio_dn", "det", "wins", "sf_est");
    std::printf("%s\n", std::string(73, '-').c_str());
    std::fflush(stdout);

    const double nearMissThresh = static_cast<double>(cfg.alpha) * 0.8;

    while (!g_stop.load(std::memory_order_relaxed)) {
        const auto l1 = layer1Scan(stream, dev, channels, cfg, stats);
        stats.l1_hot = static_cast<uint32_t>(l1.hot_idx.size());

        // Build L2 candidate list: hot channels (energy-sorted) or all channels.
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

        for (std::size_t idx : l2_candidates) {
            if (g_stop.load(std::memory_order_relaxed)) {
                break;
            }

            const double freq = channels[idx];
            const auto l2 = layer2Cad(stream, dev, freq, cfg.sf, cfg, stats);

            uint32_t sfEst = 0;
            if (l2.detected && cfg.sf_sweep) {
                sfEst = layer3SfSweep(stream, dev, freq, cfg, stats);
            }

            const double maxRatio = std::max(l2.peak_ratio_up, l2.peak_ratio_dn);
            const bool nearMiss = !l2.detected && (maxRatio > nearMissThresh);

            // Only print YES detections and near-misses (ratio > 80% of alpha).
            if (l2.detected || nearMiss) {
                const auto ts = ts_short();
                const std::string sfStr = sfEst ? ("SF" + std::to_string(sfEst)) : "-";
                const char* detLabel = l2.detected ? "YES" : "???";
                const std::string winsStr = l2.detected
                    ? std::to_string(l2.detections) + "/" + std::to_string(l2.windows_tried)
                    : "-";

                std::printf("%-13s  %-15.0f  %8.2f  %8.2f  %5s  %6s  %6s\n",
                            ts.c_str(), freq,
                            l2.peak_ratio_up, l2.peak_ratio_dn,
                            detLabel, winsStr.c_str(), sfStr.c_str());
                std::fflush(stdout);

                if (l2.detected) {
                    char freqBuf[32];
                    std::snprintf(freqBuf, sizeof(freqBuf), "%.3f MHz", freq / 1.0e6);
                    stats.last_det_freq = freqBuf;
                    stats.last_det_time = ts;
                }
            }
        }

        ++stats.sweeps;
        if (cfg.sweeps > 0 && stats.sweeps >= cfg.sweeps) {
            break;
        }

        // Status line on stderr (overwritten in place).
        if (!g_stop.load(std::memory_order_relaxed)) {
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
    return 0;
}
