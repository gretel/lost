// SPDX-License-Identifier: ISC
//
// lora_rx: Hardware LoRa receiver using native GR4 SoapySDR blocks.
//
// Single-channel receiver with SoapySimpleSource (any SoapySDR device).
// Default device: uhd (USRP B210/B220 via SoapyUHD).
//
// Graph: SoapySimpleSource -> BurstDetector -> SymbolDemodulator -> FrameSink
//
// All LoRa PHY parameters and device settings are configurable via CLI.
// Defaults: SF8/BW62.5k/CR4/8/sync=0x12 at 869.618 MHz.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>

#include "FrameSink.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void signal_handler(int /*sig*/) { g_running = 0; }

struct RxConfig {
    std::string device{"uhd"};
    std::string device_param{"type=b200"};
    double      freq{869'618'000.0};
    double      gain{30.0};
    float       rate{250'000.f};
    uint32_t    bw{62'500};
    uint8_t     sf{8};
    uint16_t    sync{0x12};
    uint16_t    preamble{8};
    std::string udp{};
    bool        cbor{false};
};

void print_usage(const char* progname) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Options:\n"
        "  --device <name>       SoapySDR driver name (default: uhd)\n"
        "  --device-param <str>  Extra device params, key=val,key=val (default: type=b200)\n"
        "  --freq <Hz>           RX center frequency (default: 869618000)\n"
        "  --gain <dB>           RX gain (default: 30)\n"
        "  --rate <S/s>          Sample rate (default: 250000)\n"
        "  --bw <Hz>             LoRa bandwidth (default: 62500)\n"
        "  --sf <7-12>           Spreading factor (default: 8)\n"
        "  --sync <hex>          Sync word (default: 0x12)\n"
        "  --preamble <n>        Preamble length (default: 8)\n"
        "  --udp [host:]port     Enable CBOR UDP server\n"
        "  --cbor                Write CBOR to stdout\n"
        "  -h, --help            Show this help\n",
        progname);
}

bool parse_args(int argc, char* argv[], RxConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        }
        if (arg == "--device" && i + 1 < argc) { cfg.device = argv[++i]; continue; }
        if (arg == "--device-param" && i + 1 < argc) { cfg.device_param = argv[++i]; continue; }
        if (arg == "--freq" && i + 1 < argc) { cfg.freq = std::stod(argv[++i]); continue; }
        if (arg == "--gain" && i + 1 < argc) { cfg.gain = std::stod(argv[++i]); continue; }
        if (arg == "--rate" && i + 1 < argc) { cfg.rate = std::stof(argv[++i]); continue; }
        if (arg == "--bw" && i + 1 < argc) { cfg.bw = static_cast<uint32_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sf" && i + 1 < argc) { cfg.sf = static_cast<uint8_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sync" && i + 1 < argc) { cfg.sync = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); continue; }
        if (arg == "--preamble" && i + 1 < argc) { cfg.preamble = static_cast<uint16_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--udp" && i + 1 < argc) { cfg.udp = argv[++i]; continue; }
        if (arg == "--cbor") { cfg.cbor = true; continue; }
        std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
        print_usage(argv[0]);
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    RxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const auto os_factor = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    std::fprintf(stderr, "=== LoRa RX (Soapy) ===\n");
    std::fprintf(stderr, "  Device:      %s\n", cfg.device.c_str());
    std::fprintf(stderr, "  Params:      %s\n", cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());
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

    auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapySimpleSource<std::complex<float>>>({
        {"device", cfg.device},
        {"device_parameter", cfg.device_param},
        {"sample_rate", cfg.rate},
        {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
        {"rx_gains", gr::Tensor<double>{cfg.gain}},
        {"max_chunck_size", static_cast<uint32_t>(512U << 4U)},
        {"max_overflow_count", gr::Size_t{0}},  // disable overflow limit
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

    std::fprintf(stderr, "\nReceiver stopped.\n");
    _exit(0);
}
