// SPDX-License-Identifier: ISC
//
// lora_rx_soapy: Hardware LoRa receiver using SoapySDR + B210.
//
// All LoRa PHY parameters (SF, BW, CR, sync word, preamble length) are
// configurable via CLI options. Defaults: SF8/BW62.5k/CR4/8/sync=0x12.
//
// Graph: SoapySource -> BurstDetector -> SymbolDemodulator -> FrameSink

#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>

#include "FrameSink.hpp"
#include "SoapySource.hpp"

namespace {
volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

struct RxConfig {
    double   freq{869'525'000.0};
    double   gain{30.0};
    float    rate{250'000.f};
    uint32_t bw{62500};
    uint8_t  sf{8};
    uint16_t sync{0x12};
    uint16_t preamble{8};
    bool     cbor{false};
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Options:\n"
        "  --freq <hz>       RX center frequency (default: 869525000)\n"
        "  --gain <db>       RX gain in dB (default: 30)\n"
        "  --rate <sps>      Sample rate in S/s (default: 250000)\n"
        "  --bw <hz>         LoRa bandwidth (default: 62500)\n"
        "  --sf <7-12>       Spreading factor (default: 8)\n"
        "  --sync <hex>      Sync word, e.g. 0x12 (default: 0x12)\n"
        "  --preamble <n>    Preamble length in symbols (default: 8)\n"
        "  --cbor            Output decoded frames as CBOR\n"
        "  -h, --help        Show this help\n\n"
        "Example:\n"
        "  %s --sf 12 --bw 125000 --rate 500000\n"
        "  %s --cbor | python3 -c \\\n"
        "    'import cbor2,sys; [print(cbor2.load(sys.stdin.buffer)) for _ in iter(int,1)]'\n",
        prog, prog, prog);
}

bool parse_args(int argc, char** argv, RxConfig& cfg) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
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

}  // namespace

void signal_handler(int /*sig*/) {
    g_running = 0;
}

int main(int argc, char** argv) {
    RxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    const auto os_factor = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    // Print banner to stderr so it doesn't interfere with CBOR output
    auto* banner_fp = cfg.cbor ? stderr : stdout;
    std::fprintf(banner_fp, "=== LoRa RX (SoapySDR via C bridge) ===\n");
    std::fprintf(banner_fp, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(banner_fp, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(banner_fp, "  Sample rate: %.0f S/s\n", static_cast<double>(cfg.rate));
    std::fprintf(banner_fp, "  OS factor:   %u\n", os_factor);
    std::fprintf(banner_fp, "  SF=%u  BW=%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, cfg.sync, cfg.preamble);
    std::fprintf(banner_fp, "  Device:      uhd (B210), clock_source=external\n");
    std::fprintf(banner_fp, "  Output:      %s\n", cfg.cbor ? "CBOR" : "text");
    std::fprintf(banner_fp, "\n");

    // --- Install signal handler for clean shutdown ---
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // --- Build the graph ---
    gr::Graph graph;

    auto& source = graph.emplaceBlock<gr::lora::SoapySource>({
        {"device", std::string("uhd")},
        {"clock_source", std::string("external")},
        {"device_args", std::string("recv_frame_size=16360")},
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
        {"output_mode", std::string(cfg.cbor ? "cbor" : "text")},
        {"sync_word", cfg.sync},
        {"phy_sf", cfg.sf},
        {"phy_bw", cfg.bw},
    });

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

    std::fprintf(banner_fp, "Starting receiver... (Ctrl+C to stop)\n\n");

    // --- Run the scheduler ---
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "Scheduler init error\n");
        return 1;
    }
    const auto ret = sched.runAndWait();
    if (!ret.has_value()) {
        std::fprintf(stderr, "Scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
        return 1;
    }

    std::fprintf(banner_fp, "\nReceiver stopped.\n");
    return 0;
}
