// SPDX-License-Identifier: ISC
//
// lora_rx: Hardware LoRa receiver.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Supports any device the linked backend can enumerate.
// All LoRa PHY parameters and device settings are configurable via CLI.
// Defaults: SF8/BW62.5k/CR4/8/sync=0x12, auto-discover device.
//
// Two modes:
//
//   Single-channel (default):
//     Graph: RadioSource -> BurstDetector -> SymbolDemodulator -> FrameSink
//     Receives on one frequency. Use --freq to set.
//
//   Scanner (--scan):
//     Dual-RX multi-channel scanner using both B210 RX channels.
//     Architecture mirrors GR3 meshcore_scanner.py:
//       - RX0 @ center_freq_0 (default 866 MHz)
//       - RX1 @ center_freq_1 (default 869 MHz)
//       - Sample rate: 5 MS/s per RX, decimated per channel to 4×BW
//       - Each channel: FreqXlatingDecimator -> BurstDetector -> Demod -> Sink
//     Default EU 868 channel plan (5 channels):
//       RX0 (866 MHz): 865.125, 868.100 MHz
//       RX1 (869 MHz): 868.300, 869.525, 869.618 MHz
//
// Robustness:
//   - Probes for device before building graph (with timeout).
//   - Ctrl+C / SIGTERM triggers clean scheduler shutdown via monitor thread.
//   - Clear error messages on device not found / start failure.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <future>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/FreqXlatingDecimator.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>
#include <gnuradio-4.0/lora/algorithm/firdes.hpp>

#include "DualRadioSource.hpp"
#include "FrameSink.hpp"
#include "RadioSource.hpp"
#include "radio_bridge.h"

namespace {
volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

// ---- Channel definition for scanner mode ----

/// A LoRa channel to monitor: absolute frequency + RX assignment.
struct LoraChannel {
    double   freq_hz;       ///< absolute channel frequency (e.g. 869618000)
    uint8_t  rx_index;      ///< 0 = RX0, 1 = RX1
    uint8_t  sf;            ///< spreading factor
    uint32_t bw;            ///< bandwidth in Hz
    uint16_t sync_word;     ///< sync word (0x12, 0x2B, 0x34)
    uint16_t preamble_len;  ///< preamble length in symbols
};

// ---- Configuration ----

struct RxConfig {
    std::string args{"type=b200"};   ///< device args (default: B200/B210)
    std::string clock{};             ///< clock source (empty = device default)
    double   freq{869'618'000.0};
    double   gain{30.0};
    float    rate{250'000.f};
    uint32_t bw{62500};
    uint8_t  sf{8};
    uint16_t sync{0x12};
    uint16_t preamble{8};
    std::string udp{};               ///< "[host:]port" for UDP CBOR server
    bool     cbor{false};            ///< write CBOR to stdout instead of text

    // Scanner mode fields
    bool     scan{false};            ///< enable dual-RX scanner mode
    double   rx0_freq{866'000'000.0};
    double   rx1_freq{869'000'000.0};
    float    wideband_rate{5'000'000.f};  ///< per-RX sample rate in scan mode
    std::vector<LoraChannel> channels;

    // Overflow test mode
    bool     overflow_test{false};   ///< run overflow diagnostics instead of RX
    int      test_duration{30};      ///< overflow test duration in seconds
    int      recv_frames{0};         ///< num_recv_frames override (0 = UHD default)
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Device options:\n"
        "  --args <str>      Device args (default: type=b200)\n"
        "                    Examples: type=b200, addr=192.168.10.2\n"
        "  --clock <src>     Clock source (default: device default)\n"
        "                    Examples: internal, external, gpsdo\n"
        "  --freq <hz>       RX center frequency (default: 869618000)\n"
        "  --gain <db>       RX gain in dB (default: 30)\n"
        "  --rate <sps>      Sample rate in S/s (default: 250000)\n\n"
        "LoRa PHY options:\n"
        "  --bw <hz>         LoRa bandwidth (default: 62500)\n"
        "  --sf <7-12>       Spreading factor (default: 8)\n"
        "  --sync <hex>      Sync word, e.g. 0x12 (default: 0x12)\n"
        "  --preamble <n>    Preamble length in symbols (default: 8)\n\n"
        "Output options:\n"
        "  --udp [host:]port Bind UDP server for CBOR frames (e.g. 5556 or 0.0.0.0:5556)\n"
        "                    Consumers register by sending any datagram to this port.\n"
        "  --cbor            Write CBOR to stdout instead of text\n\n"
        "Scanner mode (--scan):\n"
        "  --scan            Enable dual-RX multi-channel scanner\n"
        "  --rx0-freq <hz>   RX0 center frequency (default: 866000000)\n"
        "  --rx1-freq <hz>   RX1 center frequency (default: 869000000)\n"
        "  --scan-rate <sps> Wideband sample rate per RX (default: 5000000)\n\n"
        "  Default channel plan (EU 868, MeshCore):\n"
        "    RX0 (866 MHz): 865.125, 868.100 MHz\n"
        "    RX1 (869 MHz): 868.300, 869.525, 869.618 MHz\n\n"
        "Buffer tuning:\n"
        "  --recv-frames <n> USB recv buffer count (default: UHD default = 16)\n"
        "                    Increase to 32-64 to reduce overflows.\n"
        "                    At 250 kS/s: 16 frames = ~131 ms buffer.\n"
        "                    At 5 MS/s:   16 frames = ~6.5 ms buffer.\n\n"
        "Diagnostics:\n"
        "  --overflow-test [sec]  Run overflow test for N seconds (default: 30)\n"
        "                    Streams RX without decode, counts overflows,\n"
        "                    and prints buffer tuning recommendations.\n"
        "                    Combine with --scan for dual-RX stress test.\n\n"
        "Other:\n"
        "  -h, --help        Show this help\n\n"
        "Examples:\n"
        "  %s\n"
        "  %s --args type=b200 --clock external\n"
        "  %s --sf 12 --bw 125000 --rate 500000\n"
        "  %s --cbor | python3 scripts/lora_decode_meshcore.py\n"
        "  %s --scan --clock external\n"
        "  %s --scan --udp 5556\n"
        "  %s --overflow-test 60 --recv-frames 32\n"
        "  %s --overflow-test 30 --scan --recv-frames 64\n",
        prog, prog, prog, prog, prog, prog, prog, prog, prog);
}

bool parse_args(int argc, char** argv, RxConfig& cfg) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        } else if (arg == "--args" && i + 1 < argc) {
            cfg.args = argv[++i];
        } else if (arg == "--clock" && i + 1 < argc) {
            cfg.clock = argv[++i];
        } else if (arg == "--freq" && i + 1 < argc) {
            cfg.freq = std::stod(argv[++i]);
        } else if (arg == "--gain" && i + 1 < argc) {
            cfg.gain = std::stod(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            cfg.rate = std::stof(argv[++i]);
        } else if (arg == "--bw" && i + 1 < argc) {
            cfg.bw = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--sf" && i + 1 < argc) {
            cfg.sf = static_cast<uint8_t>(std::stoul(argv[++i]));
        } else if (arg == "--sync" && i + 1 < argc) {
            cfg.sync = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--preamble" && i + 1 < argc) {
            cfg.preamble = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--udp" && i + 1 < argc) {
            cfg.udp = argv[++i];
        } else if (arg == "--cbor") {
            cfg.cbor = true;
        } else if (arg == "--scan") {
            cfg.scan = true;
        } else if (arg == "--rx0-freq" && i + 1 < argc) {
            cfg.rx0_freq = std::stod(argv[++i]);
        } else if (arg == "--rx1-freq" && i + 1 < argc) {
            cfg.rx1_freq = std::stod(argv[++i]);
        } else if (arg == "--scan-rate" && i + 1 < argc) {
            cfg.wideband_rate = std::stof(argv[++i]);
        } else if (arg == "--recv-frames" && i + 1 < argc) {
            cfg.recv_frames = std::stoi(argv[++i]);
        } else if (arg == "--overflow-test") {
            cfg.overflow_test = true;
            // Optional duration argument (next arg if it looks like a number)
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                cfg.test_duration = std::stoi(argv[++i]);
            }
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

/// Build the default EU 868 channel plan for scanner mode.
/// RX0 centered at 866 MHz covers 863.5-868.5 MHz (+/-2.5 MHz).
/// RX1 centered at 869 MHz covers 866.5-871.5 MHz (+/-2.5 MHz).
void build_default_channels(RxConfig& cfg) {
    auto ch = [&](double freq_hz, uint8_t rx) {
        cfg.channels.push_back({freq_hz, rx, cfg.sf, cfg.bw,
                                cfg.sync, cfg.preamble});
    };

    // RX0 @ 866 MHz
    ch(865'125'000.0, 0);   // MeshCore/Reticulum
    ch(868'100'000.0, 0);   // LoRaWAN EU868

    // RX1 @ 869 MHz
    ch(868'300'000.0, 1);   // LoRaWAN EU868
    ch(869'525'000.0, 1);   // MeshCore/Reticulum
    ch(869'618'000.0, 1);   // MeshCore UK/Narrow
}

/// Probe for a device with a timeout. Returns true if found.
/// Device enumeration can hang indefinitely on macOS if the device is in a
/// bad USB state, so we run the probe on a detached thread and give up
/// after timeout_sec seconds.
bool probe_device_with_timeout(const std::string& device_args, int timeout_sec) {
    std::fprintf(stderr, "Probing for device (args=\"%s\", timeout %ds)...\n",
                 device_args.c_str(), timeout_sec);

    auto future = std::async(std::launch::async, [&device_args]() {
        return radio_bridge_probe(device_args.empty() ? nullptr : device_args.c_str());
    });

    auto status = future.wait_for(std::chrono::seconds(timeout_sec));
    if (status == std::future_status::timeout) {
        std::fprintf(stderr,
            "ERROR: Device probe timed out after %ds.\n"
            "       The device may be in a bad USB state. Try:\n"
            "         1. Unplug and replug the device\n"
            "         2. uhd_find_devices\n",
            timeout_sec);
        return false;
    }

    return future.get() != 0;
}

/// Inject num_recv_frames into device args string if --recv-frames was set.
/// UHD B200/B210 default is 16 USB transfer buffers (8176 bytes each).
/// At 250 kS/s this gives ~131 ms buffering. At 5 MS/s only ~6.5 ms.
/// Increasing to 32-64 reduces overflow probability significantly.
std::string apply_recv_frames(const std::string& args, int recv_frames) {
    if (recv_frames <= 0) return args;
    std::string extra = "num_recv_frames=" + std::to_string(recv_frames);
    if (args.empty()) return extra;
    return args + "," + extra;
}

// ---- Overflow test mode ----

/// Stream RX samples in a tight loop for test_duration seconds.
/// No GR4 graph — pure bridge reads. Measures overflow rate and prints
/// buffer tuning recommendations.
int run_overflow_test(RxConfig& cfg) {
    cfg.args = apply_recv_frames(cfg.args, cfg.recv_frames);
    const int duration = cfg.test_duration;

    std::fprintf(stderr, "=== Overflow Test ===\n");
    std::fprintf(stderr, "  Device args: %s\n", cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n", cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  Sample rate: %.0f S/s\n", static_cast<double>(cfg.rate));
    std::fprintf(stderr, "  Duration:    %d s\n", duration);
    if (cfg.recv_frames > 0) {
        std::fprintf(stderr, "  Recv frames: %d\n", cfg.recv_frames);
    }
    std::fprintf(stderr, "\n");

    // Create device
    radio_bridge_config_t config{};
    config.device_args  = cfg.args.c_str();
    config.clock_source = cfg.clock.empty() ? nullptr : cfg.clock.c_str();
    config.sample_rate  = static_cast<double>(cfg.rate);
    config.center_freq  = cfg.freq;
    config.gain_db      = cfg.gain;
    config.bandwidth    = 0.0;
    config.channel      = 0;
    config.antenna      = nullptr;
    config.direction    = RADIO_BRIDGE_RX;

    auto* bridge = radio_bridge_create(&config);
    if (!bridge) {
        std::fprintf(stderr, "ERROR: %s\n", radio_bridge_last_error());
        return 1;
    }

    // Compute buffer metrics
    const double rate = radio_bridge_get_sample_rate(bridge);
    const int default_frames = 16;
    const int frame_bytes = 8176;  // B200/B210 USB transfer size
    const int samples_per_frame = frame_bytes / 8;  // fc32 = 8 bytes/sample
    const int actual_frames = (cfg.recv_frames > 0) ? cfg.recv_frames : default_frames;
    const double buffer_sec = static_cast<double>(actual_frames * samples_per_frame) / rate;

    std::fprintf(stderr, "  Actual rate: %.0f S/s\n", rate);
    std::fprintf(stderr, "  USB buffer:  %d frames x %d samples = %.1f ms\n",
                 actual_frames, samples_per_frame, buffer_sec * 1000.0);
    std::fprintf(stderr, "\nStreaming... (Ctrl+C to abort)\n\n");

    // Stream loop
    constexpr std::size_t chunk_size = 8192;
    std::vector<float> buf(chunk_size * 2);  // interleaved I/Q
    uint64_t total_samples = 0;
    uint64_t total_reads = 0;
    uint64_t overflow_count = 0;
    uint64_t timeout_count = 0;
    uint64_t error_count = 0;

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(duration);
    auto last_report = start_time;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        if (now >= end_time) break;

        int ret = radio_bridge_read(bridge, buf.data(), chunk_size, 0.1);
        total_reads++;

        if (ret > 0) {
            total_samples += static_cast<uint64_t>(ret);
        } else if (ret == RADIO_BRIDGE_ERR_OVERFLOW) {
            overflow_count++;
        } else if (ret == RADIO_BRIDGE_ERR_TIMEOUT) {
            timeout_count++;
        } else {
            error_count++;
        }

        // Periodic status (every 5 seconds)
        if (now - last_report >= std::chrono::seconds(5)) {
            auto elapsed = std::chrono::duration<double>(now - start_time).count();
            std::fprintf(stderr, "  [%.0fs] samples=%" PRIu64 " overflows=%" PRIu64 " rate=%.0f S/s\n",
                         elapsed, total_samples, overflow_count,
                         static_cast<double>(total_samples) / elapsed);
            last_report = now;
        }
    }

    auto actual_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(actual_end - start_time).count();

    radio_bridge_destroy_ex(bridge, 1);

    // Print results
    double expected_samples = rate * elapsed;
    double overflow_pct = (overflow_count > 0)
        ? 100.0 * static_cast<double>(overflow_count) / static_cast<double>(total_reads)
        : 0.0;
    double throughput = static_cast<double>(total_samples) / elapsed;
    double loss_pct = 100.0 * (1.0 - static_cast<double>(total_samples) / expected_samples);

    std::fprintf(stderr,
        "\n=== Results ===\n"
        "  Duration:     %.1f s\n"
        "  Total reads:  %" PRIu64 "\n"
        "  Total samples:%" PRIu64 " (expected ~%.0f)\n"
        "  Throughput:   %.0f S/s (%.1f%% of target)\n"
        "  Overflows:    %" PRIu64 " (%.2f%% of reads)\n"
        "  Timeouts:     %" PRIu64 "\n"
        "  Errors:       %" PRIu64 "\n"
        "  Sample loss:  %.2f%%\n",
        elapsed,
        total_reads,
        total_samples, expected_samples,
        throughput, 100.0 * throughput / rate,
        overflow_count, overflow_pct,
        timeout_count,
        error_count,
        loss_pct > 0.0 ? loss_pct : 0.0);

    // Recommendations
    std::fprintf(stderr, "\n=== Recommendations ===\n");
    if (overflow_count == 0) {
        std::fprintf(stderr, "  PASS: No overflows detected. Buffer configuration is adequate.\n");
    } else {
        std::fprintf(stderr, "  FAIL: %" PRIu64 " overflows detected.\n",
                     overflow_count);
        if (cfg.recv_frames == 0) {
            std::fprintf(stderr,
                "  Try: --recv-frames 32  (doubles USB buffer to ~%.0f ms)\n",
                buffer_sec * 2.0 * 1000.0);
        } else if (cfg.recv_frames < 64) {
            std::fprintf(stderr,
                "  Try: --recv-frames %d  (increase USB buffer)\n",
                cfg.recv_frames * 2);
        } else {
            std::fprintf(stderr,
                "  USB buffer already at %d frames. Consider:\n"
                "    - Lowering sample rate (currently %.0f S/s)\n"
                "    - Closing other USB 3.0 devices\n"
                "    - Using a powered USB 3.0 hub\n",
                cfg.recv_frames, static_cast<double>(cfg.rate));
        }
    }

    std::fprintf(stderr, "\n");
    _exit(overflow_count > 0 ? 1 : 0);
}

/// Dual-channel overflow test: mirrors scanner mode's USB/driver path.
/// Uses radio_bridge_multi (2 RX channels on one USRP) at wideband_rate.
/// This is the configuration where overflows are most likely — two channels
/// at 5 MS/s each sharing one USB 3.0 pipe (10 MS/s aggregate = 40 MB/s).
///
/// B210 RX buffer chain (7 stages, FPGA to host, sc16 wire format):
///
///   Per-radio stages (x2 independent):
///     1. new_rx_framer sample FIFO   2^11=2048 entries  16 KB  0.82 ms/ch
///     2. rx_fifo clock-crossing      2^11=2048 entries  16 KB  0.82 ms/ch
///     3. axi_packet_gate             2^11=2048 entries  16 KB  0.82 ms/ch
///     Per-radio subtotal: ~49 KB, ~2.46 ms
///
///   axi_mux4 merges both radios (round-robin, BUFFER=1)
///
///   Shared stages (both channels interleaved):
///     4. extra_rx_buff               2^12=4096 entries  32 KB  0.82 ms
///     5. GPIF2 path (cross-clk+gate) ~4608 entries      18 KB  0.46 ms
///     6. FX3 USB DMA buffers         2 x 16 KB          32 KB  0.82 ms
///     7. Host libusb buffer pool     16 x 8176 bytes   131 KB  3.27 ms
///     Shared subtotal: ~213 KB, ~5.37 ms
///
///   Grand total at defaults: ~264 KB, ~7.83 ms end-to-end.
///   (Source: ettus-uhd FPGA Verilog + host C++. See uhd skill for details.)
///
/// SOURCE_FLOW_CONTROL=0 — no backpressure from USB to FPGA. When per-radio
/// FIFOs fill (stages 1-3), new_rx_control transitions to IBS_OVERRUN and
/// emits error code 0x8. UHD's handle_overflow() (b200_io_impl.cpp) detects
/// MIMO mode, stops/flushes/restarts the stream with time_spec=now+0.01.
/// This is self-correcting but costs one overflow event per restart.
///
/// Only stage 7 (host USB pool) is tunable via num_recv_frames device arg.
/// At 128 frames: 128 x 8176 = ~1 MB = ~26 ms — enough to absorb macOS
/// scheduling jitter that exceeds the default ~7.83 ms chain depth.
/// The LibreSDR B220 doubles stage 5 (DATA_RX_FIFO_SIZE=14 vs B210's 13).
int run_overflow_test_dual(RxConfig& cfg) {
    cfg.args = apply_recv_frames(cfg.args, cfg.recv_frames);
    const int duration = cfg.test_duration;

    std::fprintf(stderr, "=== Overflow Test (Dual-RX) ===\n");
    std::fprintf(stderr, "  Device args: %s\n", cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n",
                 cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  RX0 center:  %.6f MHz\n", cfg.rx0_freq / 1e6);
    std::fprintf(stderr, "  RX1 center:  %.6f MHz\n", cfg.rx1_freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  Sample rate: %.0f S/s per RX (%.0f S/s aggregate)\n",
                 static_cast<double>(cfg.wideband_rate),
                 static_cast<double>(cfg.wideband_rate) * 2.0);
    std::fprintf(stderr, "  Duration:    %d s\n", duration);
    if (cfg.recv_frames > 0) {
        std::fprintf(stderr, "  Recv frames: %d\n", cfg.recv_frames);
    }
    std::fprintf(stderr, "\n");

    // Create dual-channel device
    radio_bridge_multi_config_t config{};
    config.device_args  = cfg.args.c_str();
    config.clock_source = cfg.clock.empty() ? nullptr : cfg.clock.c_str();
    config.sample_rate  = static_cast<double>(cfg.wideband_rate);
    config.center_freq_0 = cfg.rx0_freq;
    config.gain_db_0     = cfg.gain;
    config.bandwidth_0   = 0.0;
    config.antenna_0     = nullptr;
    config.center_freq_1 = cfg.rx1_freq;
    config.gain_db_1     = cfg.gain;
    config.bandwidth_1   = 0.0;
    config.antenna_1     = nullptr;

    auto* bridge = radio_bridge_multi_create(&config);
    if (!bridge) {
        std::fprintf(stderr, "ERROR: %s\n", radio_bridge_last_error());
        return 1;
    }

    // Compute buffer metrics
    const double rate = radio_bridge_multi_get_sample_rate(bridge);
    const int default_frames = 16;
    // B210 multi-channel: sc16 on wire = 4 bytes/sample, 2 channels interleaved
    // USB frame is 8176 bytes, shared between 2 channels.
    // Each channel gets frame_bytes / (2 * 4) = 1022 samples per USB frame.
    const int frame_bytes = 8176;
    const int samples_per_frame = frame_bytes / (2 * 4);  // 2 channels, sc16
    const int actual_frames = (cfg.recv_frames > 0) ? cfg.recv_frames : default_frames;
    const double buffer_sec = static_cast<double>(actual_frames * samples_per_frame) / rate;

    std::fprintf(stderr, "  Actual rate: %.0f S/s per RX\n", rate);
    std::fprintf(stderr, "  USB buffer:  %d frames x %d samples/ch = %.1f ms\n",
                 actual_frames, samples_per_frame, buffer_sec * 1000.0);
    std::fprintf(stderr, "\nStreaming... (Ctrl+C to abort)\n\n");

    // Stream loop — read both channels simultaneously.
    // Track startup vs steady-state overflows separately.
    // The B210 MIMO startup overflow (per-radio FPGA FIFO chain, ~2.46 ms)
    // typically fires once during the first few seconds and is self-correcting.
    constexpr std::size_t chunk_size = 65536;  // match DualRadioSource
    constexpr double startup_window = 5.0;     // seconds
    std::vector<float> buf0(chunk_size * 2);   // ch0 interleaved I/Q
    std::vector<float> buf1(chunk_size * 2);   // ch1 interleaved I/Q
    uint64_t total_samples = 0;
    uint64_t total_reads = 0;
    uint64_t startup_overflows = 0;
    uint64_t steady_overflows = 0;
    uint64_t timeout_count = 0;
    uint64_t error_count = 0;

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(duration);
    auto last_report = start_time;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        if (now >= end_time) break;

        int ret = radio_bridge_multi_read(bridge, buf0.data(), buf1.data(),
                                          chunk_size, 0.1);
        total_reads++;
        double age = std::chrono::duration<double>(now - start_time).count();

        if (ret > 0) {
            total_samples += static_cast<uint64_t>(ret);
        } else if (ret == RADIO_BRIDGE_ERR_OVERFLOW) {
            if (age < startup_window) {
                startup_overflows++;
            } else {
                steady_overflows++;
            }
        } else if (ret == RADIO_BRIDGE_ERR_TIMEOUT) {
            timeout_count++;
        } else {
            error_count++;
        }

        // Periodic status (every 5 seconds)
        if (now - last_report >= std::chrono::seconds(5)) {
            uint64_t total_ov = startup_overflows + steady_overflows;
            std::fprintf(stderr,
                "  [%.0fs] samples/ch=%" PRIu64 " overflows=%" PRIu64
                " (startup=%" PRIu64 ") rate=%.0f S/s/ch\n",
                age, total_samples, total_ov, startup_overflows,
                static_cast<double>(total_samples) / age);
            last_report = now;
        }
    }

    auto actual_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(actual_end - start_time).count();

    radio_bridge_multi_destroy_ex(bridge, 1);

    // Print results
    uint64_t total_overflows = startup_overflows + steady_overflows;
    double expected_samples = rate * elapsed;
    double overflow_pct = (total_overflows > 0)
        ? 100.0 * static_cast<double>(total_overflows) / static_cast<double>(total_reads)
        : 0.0;
    double throughput = static_cast<double>(total_samples) / elapsed;
    double loss_pct = 100.0 * (1.0 - static_cast<double>(total_samples) / expected_samples);

    std::fprintf(stderr,
        "\n=== Results (Dual-RX) ===\n"
        "  Duration:       %.1f s\n"
        "  Total reads:    %" PRIu64 "\n"
        "  Samples per ch: %" PRIu64 " (expected ~%.0f)\n"
        "  Per-ch rate:    %.0f S/s (%.1f%% of target)\n"
        "  Aggregate:      %.0f S/s (2 channels)\n"
        "  Overflows:      %" PRIu64 " total (%.2f%% of reads)\n"
        "    Startup:      %" PRIu64 " (first %.0fs, B210 MIMO FPGA transient)\n"
        "    Steady-state: %" PRIu64 "\n"
        "  Timeouts:       %" PRIu64 "\n"
        "  Errors:         %" PRIu64 "\n"
        "  Sample loss:    %.2f%%\n",
        elapsed,
        total_reads,
        total_samples, expected_samples,
        throughput, 100.0 * throughput / rate,
        throughput * 2.0,
        total_overflows, overflow_pct,
        startup_overflows, startup_window,
        steady_overflows,
        timeout_count,
        error_count,
        loss_pct > 0.0 ? loss_pct : 0.0);

    // Recommendations — steady-state overflows are the real concern.
    // Startup overflows are a known B210 MIMO artifact (per-radio FPGA FIFO
    // chain overflows before USB pipeline primes, self-correcting via UHD's
    // handle_overflow stop/flush/restart in b200_io_impl.cpp).
    std::fprintf(stderr, "\n=== Assessment ===\n");
    if (steady_overflows == 0 && startup_overflows == 0) {
        std::fprintf(stderr, "  PASS: No overflows at %.0f S/s dual-RX with %d USB frames.\n",
                     rate, actual_frames);
    } else if (steady_overflows == 0) {
        std::fprintf(stderr,
            "  PASS: %" PRIu64 " startup overflow(s) only (B210 MIMO FPGA transient).\n"
            "        No steady-state overflows. Buffer configuration is adequate.\n",
            startup_overflows);
    } else {
        std::fprintf(stderr, "  FAIL: %" PRIu64 " steady-state overflow(s) at %.0f S/s dual-RX.\n",
                     steady_overflows, rate);
        if (cfg.recv_frames == 0) {
            std::fprintf(stderr,
                "  Try: --recv-frames 32  (doubles USB buffer to ~%.1f ms)\n",
                buffer_sec * 2.0 * 1000.0);
        } else if (cfg.recv_frames < 128) {
            std::fprintf(stderr,
                "  Try: --recv-frames %d  (increase USB buffer to ~%.1f ms)\n",
                cfg.recv_frames * 2,
                buffer_sec * 2.0 * 1000.0);
        } else {
            std::fprintf(stderr,
                "  USB buffer at %d frames (%.1f ms). Consider:\n"
                "    - Lowering --scan-rate (currently %.0f S/s)\n"
                "    - Using an external USB 3.0 hub\n"
                "    - Closing other USB 3.0 devices\n",
                cfg.recv_frames, buffer_sec * 1000.0,
                static_cast<double>(cfg.wideband_rate));
        }
    }

    std::fprintf(stderr, "\n");
    _exit(steady_overflows > 0 ? 1 : 0);
}

// ---- Single-channel RX mode ----

int run_single_rx(RxConfig& cfg) {
    cfg.args = apply_recv_frames(cfg.args, cfg.recv_frames);
    const auto os_factor = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    std::fprintf(stderr, "=== LoRa RX ===\n");
    std::fprintf(stderr, "  Device args: %s\n", cfg.args.empty() ? "(auto)" : cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n", cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  Sample rate: %.0f S/s\n", static_cast<double>(cfg.rate));
    std::fprintf(stderr, "  OS factor:   %u\n", os_factor);
    std::fprintf(stderr, "  SF=%u  BW=%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, cfg.sync, cfg.preamble);
    if (!cfg.udp.empty()) {
        std::fprintf(stderr, "  UDP CBOR:    %s\n", cfg.udp.c_str());
    }
    if (cfg.cbor) {
        std::fprintf(stderr, "  Output:      CBOR on stdout\n");
    }
    std::fprintf(stderr, "\n");

    // --- Build the graph ---
    gr::Graph graph;

    auto& source = graph.emplaceBlock<gr::lora::RadioSource>({
        {"device_args", cfg.args},
        {"clock_source", cfg.clock},
        {"sample_rate", cfg.rate},
        {"center_freq", cfg.freq},
        {"gain", cfg.gain},
    });

    auto& detector = graph.emplaceBlock<gr::lora::BurstDetector>({
        {"center_freq", static_cast<uint32_t>(cfg.freq)},
        {"bandwidth", cfg.bw},
        {"sf", cfg.sf},
        {"sync_word", cfg.sync},
        {"os_factor", os_factor},
        {"preamble_len", cfg.preamble},
    });

    auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>({
        {"sf", cfg.sf},
        {"bandwidth", cfg.bw},
    });

    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"udp_dest", cfg.udp},
        {"sync_word", cfg.sync},
        {"phy_sf", cfg.sf},
        {"phy_bw", cfg.bw},
    });
    sink.cbor_stdout = cfg.cbor;

    // Connect: source -> detector -> demod -> sink
    if (graph.connect<"out">(source).to<"in">(detector) != gr::ConnectionResult::SUCCESS) {
        std::fprintf(stderr, "ERROR: failed to connect source -> detector\n");
        return 1;
    }
    if (graph.connect<"out">(detector).to<"in">(demod) != gr::ConnectionResult::SUCCESS) {
        std::fprintf(stderr, "ERROR: failed to connect detector -> demod\n");
        return 1;
    }
    if (graph.connect<"out">(demod).to<"in">(sink) != gr::ConnectionResult::SUCCESS) {
        std::fprintf(stderr, "ERROR: failed to connect demod -> sink\n");
        return 1;
    }

    source._skip_free = true;

    std::fprintf(stderr, "Starting receiver... (Ctrl+C to stop)\n\n");

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "ERROR: Scheduler init failed\n");
        return 1;
    }

    // Monitor thread: poll g_running, request scheduler stop on signal.
    std::atomic<bool> monitor_done{false};
    std::thread monitor([&sched, &monitor_done]() {
        while (g_running && !monitor_done.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running) {
            std::fprintf(stderr, "\nSignal received, stopping...\n");
            sched.requestStop();
        }
    });

    const auto ret = sched.runAndWait();

    monitor_done.store(true, std::memory_order_relaxed);
    monitor.join();

    if (!ret.has_value()) {
        std::fprintf(stderr, "Scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
    }

    source.stop();

    std::fprintf(stderr, "\nReceiver stopped.\n");
    _exit(0);
}

// ---- Scanner (dual-RX multi-channel) mode ----

int run_scanner(RxConfig& cfg) {
    cfg.args = apply_recv_frames(cfg.args, cfg.recv_frames);

    if (cfg.channels.empty()) {
        build_default_channels(cfg);
    }

    // Use scanner gain default (50 dB) if user didn't override
    // (single-channel default is 30 dB, scanner wants more headroom)

    const auto decimation = static_cast<uint32_t>(
        cfg.wideband_rate / static_cast<float>(cfg.bw * 4));
    const float channel_rate = cfg.wideband_rate / static_cast<float>(decimation);
    const auto os_factor = static_cast<uint8_t>(channel_rate / static_cast<float>(cfg.bw));

    std::fprintf(stderr, "=== LoRa Scanner (Dual-RX) ===\n");
    std::fprintf(stderr, "  Device args: %s\n",
                 cfg.args.empty() ? "(auto)" : cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n",
                 cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  RX0 center:  %.6f MHz\n", cfg.rx0_freq / 1e6);
    std::fprintf(stderr, "  RX1 center:  %.6f MHz\n", cfg.rx1_freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  Wideband:    %.0f S/s per RX\n",
                 static_cast<double>(cfg.wideband_rate));
    std::fprintf(stderr, "  Decimation:  %u (channel rate %.0f S/s, os_factor=%u)\n",
                 decimation, static_cast<double>(channel_rate), os_factor);
    std::fprintf(stderr, "  Channels:    %zu\n", cfg.channels.size());
    for (const auto& ch : cfg.channels) {
        std::fprintf(stderr, "    RX%u  %.6f MHz  SF=%u BW=%u sync=0x%02X\n",
                     ch.rx_index, ch.freq_hz / 1e6, ch.sf, ch.bw, ch.sync_word);
    }
    if (!cfg.udp.empty()) {
        std::fprintf(stderr, "  UDP CBOR:    %s\n", cfg.udp.c_str());
    }
    if (cfg.cbor) {
        std::fprintf(stderr, "  Output:      CBOR on stdout\n");
    }
    std::fprintf(stderr, "\n");

    // --- Design channelizer FIR taps ---
    const auto fir_taps = gr::lora::firdes_low_pass(
        101, static_cast<double>(cfg.wideband_rate), 150000.0);

    // --- Build graph ---
    gr::Graph graph;

    auto& source = graph.emplaceBlock<gr::lora::DualRadioSource>({
        {"device_args", cfg.args},
        {"clock_source", cfg.clock},
        {"sample_rate", cfg.wideband_rate},
        {"center_freq_0", cfg.rx0_freq},
        {"gain_0", cfg.gain},
        {"center_freq_1", cfg.rx1_freq},
        {"gain_1", cfg.gain},
        {"max_chunk_size", 65536U},
    });
    source._skip_free = true;

    // Create one decode chain per channel
    for (const auto& ch : cfg.channels) {
        double rx_center = (ch.rx_index == 0) ? cfg.rx0_freq : cfg.rx1_freq;
        double offset = ch.freq_hz - rx_center;

        auto ch_os_factor = static_cast<uint8_t>(
            channel_rate / static_cast<float>(ch.bw));

        auto& xlat = graph.emplaceBlock<gr::lora::FreqXlatingDecimator>();
        xlat.sample_rate = static_cast<double>(cfg.wideband_rate);
        xlat.center_freq_offset = offset;
        xlat.decimation = decimation;
        xlat.setTaps(fir_taps);

        auto& burst = graph.emplaceBlock<gr::lora::BurstDetector>({
            {"center_freq", static_cast<uint32_t>(ch.freq_hz)},
            {"bandwidth", ch.bw},
            {"sf", ch.sf},
            {"sync_word", ch.sync_word},
            {"os_factor", ch_os_factor},
            {"preamble_len", ch.preamble_len},
        });

        auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>({
            {"sf", ch.sf},
            {"bandwidth", ch.bw},
        });

        auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
            {"udp_dest", cfg.udp},
            {"sync_word", ch.sync_word},
            {"phy_sf", ch.sf},
            {"phy_bw", ch.bw},
        });
        sink.cbor_stdout = cfg.cbor;

        // Connect: source(outN) -> xlat -> burst -> demod -> sink
        gr::ConnectionResult cr;
        if (ch.rx_index == 0) {
            cr = graph.connect<"out0">(source).to<"in">(xlat);
        } else {
            cr = graph.connect<"out1">(source).to<"in">(xlat);
        }
        if (cr != gr::ConnectionResult::SUCCESS) {
            std::fprintf(stderr, "ERROR: connect source->xlat for %.6f MHz\n",
                         ch.freq_hz / 1e6);
            return 1;
        }
        if (graph.connect<"out">(xlat).to<"in">(burst) != gr::ConnectionResult::SUCCESS) {
            std::fprintf(stderr, "ERROR: connect xlat->burst for %.6f MHz\n",
                         ch.freq_hz / 1e6);
            return 1;
        }
        if (graph.connect<"out">(burst).to<"in">(demod) != gr::ConnectionResult::SUCCESS) {
            std::fprintf(stderr, "ERROR: connect burst->demod for %.6f MHz\n",
                         ch.freq_hz / 1e6);
            return 1;
        }
        if (graph.connect<"out">(demod).to<"in">(sink) != gr::ConnectionResult::SUCCESS) {
            std::fprintf(stderr, "ERROR: connect demod->sink for %.6f MHz\n",
                         ch.freq_hz / 1e6);
            return 1;
        }

        std::fprintf(stderr, "  Chain: RX%u %.6f MHz (offset %+.0f Hz) -> os=%u\n",
                     ch.rx_index, ch.freq_hz / 1e6, offset, ch_os_factor);
    }

    std::fprintf(stderr, "\nStarting scanner... (Ctrl+C to stop)\n\n");

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "ERROR: Scheduler init failed\n");
        return 1;
    }

    std::atomic<bool> monitor_done{false};
    std::thread monitor([&sched, &monitor_done]() {
        while (g_running && !monitor_done.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running) {
            std::fprintf(stderr, "\nSignal received, stopping...\n");
            sched.requestStop();
        }
    });

    const auto ret = sched.runAndWait();

    monitor_done.store(true, std::memory_order_relaxed);
    monitor.join();

    if (!ret.has_value()) {
        std::fprintf(stderr, "Scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
    }

    source.stop();

    std::fprintf(stderr, "\nScanner stopped.\n");
    _exit(0);
}

}  // namespace

// Signal handler — must only set the atomic flag.
// A monitor thread polls this and calls scheduler requestStop().
void signal_handler(int /*sig*/) {
    g_running = 0;
}

int main(int argc, char** argv) {
    RxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    // --- Install signal handlers ---
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Inject recv-frames into args for probe too ---
    std::string probe_args = apply_recv_frames(cfg.args, cfg.recv_frames);

    // --- Probe device before building graph ---
    if (!probe_device_with_timeout(probe_args, 10)) {
        std::fprintf(stderr,
            "ERROR: No device found. Is the device connected and powered?\n");
        return 1;
    }
    std::fprintf(stderr, "Device found.\n\n");

    if (cfg.overflow_test) {
        return cfg.scan ? run_overflow_test_dual(cfg) : run_overflow_test(cfg);
    }
    if (cfg.scan) {
        return run_scanner(cfg);
    }
    return run_single_rx(cfg);
}
