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
// Retune to the given BW, capture K SF12-sized windows, and for each window
// try every SF (7–12).  First SF that fires is the detection.  This merges the
// old L2 (single-SF CAD) and L3 (post-detection SF sweep) into one step.
//
// Each capture is big enough for SF12 (4096×os×2 samples).  For lower SFs
// the detector only reads the first 2^sf × os × 2 samples of the buffer.

struct Layer2Result {
    double   peak_ratio_up = 0.0;
    double   peak_ratio_dn = 0.0;
    uint32_t sf_detected    = 0;     // which SF triggered (0 = none)
    uint32_t detections     = 0;     // how many windows fired
    uint32_t windows_tried  = 0;
    bool     detected       = false;
};

static Layer2Result layer2Cad(RxStream& stream, SoapyDevice& dev,
                              double freq, double bw,
                              const ScanConfig& cfg, ScanStats& stats) {
    const double sampleRate = bw * static_cast<double>(cfg.os_factor);
    retune(stream, dev, freq, sampleRate, cfg.settle_ms);

    // Pre-build detectors for SF7–12, reuse across all windows.
    struct SfDetector {
        gr::lora::ChannelActivityDetector det;
        uint32_t sf;
    };
    std::vector<SfDetector> detectors;
    for (uint32_t trySf = 7; trySf <= 12; ++trySf) {
        SfDetector sd;
        sd.sf              = trySf;
        sd.det.sf          = trySf;
        sd.det.bandwidth   = static_cast<uint32_t>(bw);
        sd.det.os_factor   = cfg.os_factor;
        sd.det.alpha       = gr::lora::ChannelActivityDetector::default_alpha(trySf);
        sd.det.dual_chirp  = true;
        sd.det.debug       = false;
        sd.det.start();
        detectors.push_back(std::move(sd));
    }

    Layer2Result result;
    result.windows_tried = cfg.cad_windows;

    for (uint32_t win = 0; win < cfg.cad_windows; ++win) {
        if (g_stop.load(std::memory_order_relaxed)) {
            break;
        }

        // Try all SFs on this window and pick the one with the highest peak
        // ratio.  Each SF gets its own correctly-sized capture: 2×2^sf×os_factor
        // samples.  A shared SF12 buffer causes lower SFs to examine only the
        // first fraction (e.g. SF8 reads 2048 of 32768 samples), producing weak
        // false SF12 matches (~9 ratio) instead of the true SF at 100+.
        float    bestWinRatioUp = 0.0F;
        float    bestWinRatioDn = 0.0F;
        uint32_t bestWinSf      = 0;

        for (auto& sd : detectors) {
            const uint32_t sfWin = (1U << sd.sf) * cfg.os_factor * 2U;
            std::vector<cf32> sfBuf(sfWin);
            capture(stream, sfBuf.data(), sfWin, stats);
            const auto r = sd.det.detect(sfBuf.data());
            const float winRatio = std::max(r.peak_ratio_up, r.peak_ratio_dn);

            if (r.detected && winRatio > std::max(bestWinRatioUp, bestWinRatioDn)) {
                bestWinRatioUp = r.peak_ratio_up;
                bestWinRatioDn = r.peak_ratio_dn;
                bestWinSf      = sd.sf;

                gr::lora::log_ts("debug", "lora_scan",
                    "L2 %.0f Hz BW=%.0f SF%u  ratio_up=%.2f ratio_dn=%.2f (candidate)",
                    freq, bw, sd.sf,
                    static_cast<double>(r.peak_ratio_up),
                    static_cast<double>(r.peak_ratio_dn));
            }
        }

        if (bestWinSf != 0) {
            ++result.detections;
            result.detected      = true;
            result.sf_detected   = bestWinSf;
            result.peak_ratio_up = std::max(result.peak_ratio_up, static_cast<double>(bestWinRatioUp));
            result.peak_ratio_dn = std::max(result.peak_ratio_dn, static_cast<double>(bestWinRatioDn));
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

    // ── Full scan: L1 + L2 + (optional) L3, continuous ──────────────────────

    // Print header once.
    std::printf("%-13s  %10s  %6s  %6s  %6s  %6s  %4s\n",
                "time", "freq_mhz", "bw_khz", "up", "dn", "wins", "sf");
    std::printf("%s\n", std::string(60, '-').c_str());
    std::fflush(stdout);

    const double nearMissThresh = static_cast<double>(cfg.min_ratio) * 0.9;
    auto lastStatusUpdate = std::chrono::steady_clock::now();

    // For secondary BWs, use fewer CAD windows (quick yes/no, not full dwell).
    const uint32_t quickWindows = std::max(4U, cfg.cad_windows / 8U);

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

        if (l2_candidates.size() > cfg.max_l2) {
            l2_candidates.resize(cfg.max_l2);
        }

        for (std::size_t idx : l2_candidates) {
            if (g_stop.load(std::memory_order_relaxed)) {
                break;
            }

            const double freq = channels[idx];

            // Try all BWs and pick the best match (highest peak ratio).
            // A chirp partially correlates with CAD at any BW, so we can't
            // break on first detection — the matched BW produces the strongest
            // peak ratio and correctly identifies the signal's true bandwidth.
            // Each L2 call now tries all SFs (7–12) internally.
            double       bestRatio = 0.0;
            double       bestBw    = 0.0;
            Layer2Result bestL2;

            for (std::size_t bwIdx = 0; bwIdx < cfg.bws.size(); ++bwIdx) {
                if (g_stop.load(std::memory_order_relaxed)) {
                    break;
                }

                const double bw = cfg.bws[bwIdx];
                // First BW gets full dwell; others get quick probe.
                const uint32_t windows = (bwIdx == 0) ? cfg.cad_windows : quickWindows;

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
                } else {
                    const double maxRatio = std::max(l2.peak_ratio_up, l2.peak_ratio_dn);
                    if (maxRatio > nearMissThresh) {
                        gr::lora::log_ts("debug", "lora_scan",
                            "near-miss %.0f Hz BW=%.0f  ratio=%.2f (min_ratio=%.1f)",
                            freq, bw, maxRatio, static_cast<double>(cfg.min_ratio));
                    }
                }
            }

            // Report the best-matching BW if above min_ratio threshold.
            if (bestRatio >= static_cast<double>(cfg.min_ratio)) {
                const auto ts = ts_short();
                const std::string sfStr = bestL2.sf_detected
                    ? ("SF" + std::to_string(bestL2.sf_detected)) : "-";
                const std::string winsStr =
                    std::to_string(bestL2.detections) + "/" + std::to_string(bestL2.windows_tried);

                // Clear the \r status line before printing a detection row.
                std::fprintf(stderr, "\r%s\r", std::string(80, ' ').c_str());

                std::printf("%-13s  %10.3f  %6.1f  %6.1f  %6.1f  %6s  %4s\n",
                            ts.c_str(), freq / 1.0e6, bestBw / 1.0e3,
                            bestL2.peak_ratio_up, bestL2.peak_ratio_dn,
                            winsStr.c_str(), sfStr.c_str());
                std::fflush(stdout);

                char freqBuf[32];
                std::snprintf(freqBuf, sizeof(freqBuf), "%.3f MHz", freq / 1.0e6);
                stats.last_det_freq = freqBuf;
                stats.last_det_time = ts;
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
