// SPDX-License-Identifier: ISC
//
// lora_rx: Hardware LoRa receiver.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Supports any device the linked backend can enumerate.
// All LoRa PHY parameters and device settings are configurable via CLI.
// Defaults: SF8/BW62.5k/CR4/8/sync=0x12, auto-discover device.
//
// Graph: RadioSource -> BurstDetector -> SymbolDemodulator -> FrameSink
//
// Robustness:
//   - Probes for device before building graph (with timeout).
//   - Ctrl+C / SIGTERM triggers clean scheduler shutdown via monitor thread.
//   - Clear error messages on device not found / start failure.

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

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>

#include "FrameSink.hpp"
#include "RadioSource.hpp"
#include "radio_bridge.h"

namespace {
volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

struct RxConfig {
    std::string args{};              ///< device args (empty = auto-discover)
    std::string clock{};             ///< clock source (empty = device default)
    double   freq{869'618'000.0};
    double   gain{30.0};
    float    rate{250'000.f};
    uint32_t bw{62500};
    uint8_t  sf{8};
    uint16_t sync{0x12};
    uint16_t preamble{8};
    std::string udp{};               ///< "host:port" for UDP CBOR output
    bool     cbor{false};            ///< write CBOR to stdout instead of text
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Device options:\n"
        "  --args <str>      Device args (default: auto-discover)\n"
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
        "  --udp <host:port> Send CBOR frames via UDP (e.g. 127.0.0.1:5556)\n"
        "  --cbor            Write CBOR to stdout instead of text\n"
        "  -h, --help        Show this help\n\n"
        "Examples:\n"
        "  %s\n"
        "  %s --args type=b200 --clock external\n"
        "  %s --sf 12 --bw 125000 --rate 500000\n"
        "  %s --cbor | python3 scripts/lora_decode_meshcore.py\n",
        prog, prog, prog, prog, prog);
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
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
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
        // The async thread may still be running inside device enumeration.
        // We can't cancel it, but we exit the process anyway.
        return false;
    }

    return future.get() != 0;
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

    // --- Install signal handlers ---
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Probe device before building graph ---
    if (!probe_device_with_timeout(cfg.args, 10)) {
        std::fprintf(stderr,
            "ERROR: No device found. Is the device connected and powered?\n");
        return 1;
    }
    std::fprintf(stderr, "Device found.\n\n");

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

    // Skip device deallocation on shutdown — it can segfault on macOS due to
    // dual-libc++ teardown ordering. The stream is still properly stopped;
    // only the device handle leaks, which is fine since the process is exiting.
    source._skip_free = true;

    std::fprintf(stderr, "Starting receiver... (Ctrl+C to stop)\n\n");

    // --- Run the scheduler ---
    // Use singleThreaded: runAndWait() blocks in poolWorker() on this thread.
    // A separate monitor thread watches g_running and requests stop on Ctrl+C.
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

    // Stop monitor thread
    monitor_done.store(true, std::memory_order_relaxed);
    monitor.join();

    if (!ret.has_value()) {
        std::fprintf(stderr, "Scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
    }

    // Explicitly stop the stream and release the device before _exit().
    // _exit() skips C++ destructors, so RadioSource::stop() would never run,
    // leaving the USB device claimed and requiring a replug.
    source.stop();

    std::fprintf(stderr, "\nReceiver stopped.\n");

    // Use _exit() to skip C++ static destructors that may crash under
    // dual-libc++ (LLVM 19 vs system) during static teardown. The stream
    // and streamer were already released by source.stop() above.
    _exit(0);
}
