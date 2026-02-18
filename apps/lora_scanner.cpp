// SPDX-License-Identifier: ISC
//
// lora_scanner: Dual-RX multi-channel LoRa scanner.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Architecture mirrors GR3 meshcore_scanner.py:
//   - RX0 @ center_freq_0 (default 866 MHz): covers channels at offsets from center
//   - RX1 @ center_freq_1 (default 869 MHz): covers channels at offsets from center
//   - Sample rate: 5 MS/s per RX (wideband), decimated 20x to 250 kS/s per channel
//   - Each channel: FreqXlatingDecimator -> BurstDetector -> SymbolDemodulator -> FrameSink
//
// Default EU 868 channel plan (5 channels, same as GR3):
//   RX0 (866 MHz): 865.125, 868.100 MHz
//   RX1 (869 MHz): 868.300, 869.525, 869.618 MHz
//
// Graph (single multi-channel stream, avoids USB bandwidth contention):
//   DualRadioSource -out0- +-- Xlat -> Burst -> Demod -> Sink  (865.125 MHz)
//                          +-- Xlat -> Burst -> Demod -> Sink  (868.100 MHz)
//                   -out1- +-- Xlat -> Burst -> Demod -> Sink  (868.300 MHz)
//                          +-- Xlat -> Burst -> Demod -> Sink  (869.525 MHz)
//                          +-- Xlat -> Burst -> Demod -> Sink  (869.618 MHz)

#include <unistd.h>

#include <atomic>
#include <chrono>
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
#include "radio_bridge.h"

namespace {

volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

/// A LoRa channel to monitor: absolute frequency + RX assignment.
struct LoraChannel {
    double   freq_hz;       ///< absolute channel frequency (e.g. 869618000)
    uint8_t  rx_index;      ///< 0 = RX0, 1 = RX1
    uint8_t  sf;            ///< spreading factor
    uint32_t bw;            ///< bandwidth in Hz
    uint16_t sync_word;     ///< sync word (0x12, 0x2B, 0x34)
    uint16_t preamble_len;  ///< preamble length in symbols
};

struct ScannerConfig {
    // Device
    std::string args{};     ///< device args (empty = auto-discover)
    std::string clock{};    ///< clock source (empty = device default)

    // RX center frequencies
    double   rx0_freq{866'000'000.0};
    double   rx1_freq{869'000'000.0};
    double   gain{50.0};
    float    wideband_rate{5'000'000.f};  ///< per-RX sample rate

    // LoRa defaults
    uint8_t  sf{8};
    uint32_t bw{62500};
    uint16_t sync_word{0x12};
    uint16_t preamble_len{8};

    // Output
    std::string udp{};
    bool        cbor{false};

    // Channel plan: absolute frequencies assigned to RX0 or RX1
    std::vector<LoraChannel> channels;
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Dual-RX multi-channel LoRa scanner. Monitors multiple frequencies\n"
        "simultaneously using 2 RX channels.\n\n"
        "Options:\n"
        "  --args <str>      Device args (default: auto-discover)\n"
        "  --clock <src>     Clock source, e.g. 'external' (default: device default)\n"
        "  --rx0-freq <hz>   RX0 center frequency (default: 866000000)\n"
        "  --rx1-freq <hz>   RX1 center frequency (default: 869000000)\n"
        "  --gain <db>       RX gain in dB (default: 50)\n"
        "  --rate <sps>      Wideband sample rate per RX (default: 5000000)\n"
        "  --sf <7-12>       Spreading factor (default: 8)\n"
        "  --bw <hz>         LoRa bandwidth (default: 62500)\n"
        "  --sync <hex>      Sync word (default: 0x12)\n"
        "  --preamble <n>    Preamble length (default: 8)\n"
        "  --udp <host:port> Send CBOR frames via UDP\n"
        "  --cbor            Write CBOR to stdout instead of text\n"
        "  -h, --help        Show this help\n\n"
        "Default channel plan (EU 868, MeshCore):\n"
        "  RX0 (866 MHz): 865.125, 868.100 MHz\n"
        "  RX1 (869 MHz): 868.300, 869.525, 869.618 MHz\n\n"
        "Example:\n"
        "  %s --gain 40\n"
        "  %s --args type=b200 --clock external\n"
        "  %s --udp 127.0.0.1:5556\n"
        "  %s --cbor | python3 scripts/lora_mon.py --stdin\n",
        prog, prog, prog, prog, prog);
}

bool parse_args(int argc, char** argv, ScannerConfig& cfg) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        } else if (arg == "--args" && i + 1 < argc) {
            cfg.args = argv[++i];
        } else if (arg == "--clock" && i + 1 < argc) {
            cfg.clock = argv[++i];
        } else if (arg == "--rx0-freq" && i + 1 < argc) {
            cfg.rx0_freq = std::stod(argv[++i]);
        } else if (arg == "--rx1-freq" && i + 1 < argc) {
            cfg.rx1_freq = std::stod(argv[++i]);
        } else if (arg == "--gain" && i + 1 < argc) {
            cfg.gain = std::stod(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            cfg.wideband_rate = std::stof(argv[++i]);
        } else if (arg == "--sf" && i + 1 < argc) {
            cfg.sf = static_cast<uint8_t>(std::stoul(argv[++i]));
        } else if (arg == "--bw" && i + 1 < argc) {
            cfg.bw = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--sync" && i + 1 < argc) {
            cfg.sync_word = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--preamble" && i + 1 < argc) {
            cfg.preamble_len = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--udp" && i + 1 < argc) {
            cfg.udp = argv[++i];
        } else if (arg == "--cbor") {
            cfg.cbor = true;
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

/// Build the default EU 868 channel plan.
/// RX0 centered at 866 MHz covers 863.5-868.5 MHz (+/-2.5 MHz).
/// RX1 centered at 869 MHz covers 866.5-871.5 MHz (+/-2.5 MHz).
void build_default_channels(ScannerConfig& cfg) {
    auto ch = [&](double freq_hz, uint8_t rx) {
        cfg.channels.push_back({freq_hz, rx, cfg.sf, cfg.bw,
                                cfg.sync_word, cfg.preamble_len});
    };

    // RX0 @ 866 MHz
    ch(865'125'000.0, 0);   // MeshCore/Reticulum
    ch(868'100'000.0, 0);   // LoRaWAN EU868

    // RX1 @ 869 MHz
    ch(868'300'000.0, 1);   // LoRaWAN EU868
    ch(869'525'000.0, 1);   // MeshCore/Reticulum
    ch(869'618'000.0, 1);   // MeshCore UK/Narrow
}

bool probe_device_with_timeout(const char* device_args, int timeout_sec) {
    std::fprintf(stderr, "Probing for device (timeout %ds)...\n",
                 timeout_sec);
    auto future = std::async(std::launch::async, [device_args]() {
        return radio_bridge_probe(device_args);
    });
    auto status = future.wait_for(std::chrono::seconds(timeout_sec));
    if (status == std::future_status::timeout) {
        std::fprintf(stderr,
            "ERROR: Device probe timed out after %ds.\n"
            "       Try unplugging and replugging the device.\n",
            timeout_sec);
        return false;
    }
    return future.get() != 0;
}

}  // namespace

void signal_handler(int /*sig*/) {
    g_running = 0;
}

int main(int argc, char** argv) {
    ScannerConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    if (cfg.channels.empty()) {
        build_default_channels(cfg);
    }

    // Compute decimation factor and channel sample rate
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

    // --- Install signal handlers ---
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Probe device ---
    const char* probe_args = cfg.args.empty() ? nullptr : cfg.args.c_str();
    if (!probe_device_with_timeout(probe_args, 10)) {
        std::fprintf(stderr, "ERROR: No device found.\n");
        return 1;
    }
    std::fprintf(stderr, "Device found.\n\n");

    // --- Design channelizer FIR taps ---
    // GR3: firdes.low_pass(1, 5000000, 150000, 50000, WIN_HAMMING)
    // Cutoff at 150 kHz, transition 50 kHz -> ~101 taps at 5 MS/s
    const auto fir_taps = gr::lora::firdes_low_pass(
        101, static_cast<double>(cfg.wideband_rate), 150000.0);

    // --- Build graph ---
    gr::Graph graph;

    // DualRadioSource: single multi-channel stream, two output ports.
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

        // Channelizer: shift + filter + decimate
        auto& xlat = graph.emplaceBlock<gr::lora::FreqXlatingDecimator>();
        xlat.sample_rate = static_cast<double>(cfg.wideband_rate);
        xlat.center_freq_offset = offset;
        xlat.decimation = decimation;
        xlat.setTaps(fir_taps);

        // BurstDetector
        auto& burst = graph.emplaceBlock<gr::lora::BurstDetector>({
            {"center_freq", static_cast<uint32_t>(ch.freq_hz)},
            {"bandwidth", ch.bw},
            {"sf", ch.sf},
            {"sync_word", ch.sync_word},
            {"os_factor", ch_os_factor},
            {"preamble_len", ch.preamble_len},
        });

        // SymbolDemodulator
        auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>({
            {"sf", ch.sf},
            {"bandwidth", ch.bw},
        });

        // FrameSink
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

    // --- Run the scheduler ---
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "ERROR: Scheduler init failed\n");
        return 1;
    }

    // Monitor thread for clean Ctrl+C shutdown
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

    // Explicitly stop the stream before _exit() so the USB device
    // is released. Without this, the B210 stays claimed and requires
    // a replug before the next run.
    source.stop();

    std::fprintf(stderr, "\nScanner stopped.\n");
    _exit(0);
}
