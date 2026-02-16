// SPDX-License-Identifier: ISC
//
// lora_rx_soapy: Hardware LoRa receiver using SoapySDR + B210.
//
// Usage: lora_rx_soapy [--cbor] [freq_hz] [gain_db] [sample_rate]
//
// Defaults: 869.525 MHz, 30 dB gain, 250 kS/s (= 4 * BW for os_factor=4)
//
// Graph: SoapySource -> BurstDetector -> SymbolDemodulator -> ConsoleSink

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

#include "ConsoleSink.hpp"
#include "SoapySource.hpp"

namespace {
volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
}  // namespace

void signal_handler(int /*sig*/) {
    g_running = 0;
}

int main(int argc, char** argv) {
    // --- Parse CLI arguments ---
    constexpr double   kDefaultFreq   = 869'525'000.0;  // MeshCore uk/narrow: 869.525 MHz
    constexpr double   kDefaultGain   = 30.0;
    constexpr float    kDefaultRate   = 250'000.f;       // 4 * 62500 BW
    constexpr uint32_t kDefaultBW     = 62500;
    constexpr uint8_t  kDefaultSF     = 8;
    constexpr uint16_t kDefaultSync   = 0x12;

    // Check for flags
    bool cbor_mode = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--cbor") == 0) {
            cbor_mode = true;
            // Shift remaining args down
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--;
            i--;
        }
    }

    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        std::printf("Usage: %s [--cbor] [freq_hz] [gain_db] [sample_rate]\n", argv[0]);
        std::printf("\n");
        std::printf("  --cbor       Output decoded frames as CBOR (for Python: cbor2)\n");
        std::printf("  freq_hz      RX center frequency (default: %.0f)\n", kDefaultFreq);
        std::printf("  gain_db      RX gain in dB (default: %.0f)\n", kDefaultGain);
        std::printf("  sample_rate  Sample rate in S/s (default: %.0f)\n",
                    static_cast<double>(kDefaultRate));
        std::printf("\n");
        std::printf("LoRa config: SF=%u BW=%u CR=4/8 sync=0x%02X (MeshCore defaults)\n",
                    kDefaultSF, kDefaultBW, kDefaultSync);
        std::printf("\n");
        std::printf("Example:\n");
        std::printf("  %s 869525000 40\n", argv[0]);
        std::printf("  %s --cbor | python3 -c \\\n", argv[0]);
        std::printf("    'import cbor2,sys; [print(cbor2.load(sys.stdin.buffer)) for _ in iter(int,1)]'\n");
        return 0;
    }

    const double freq       = argc >= 2 ? std::stod(argv[1]) : kDefaultFreq;
    const double gain       = argc >= 3 ? std::stod(argv[2]) : kDefaultGain;
    const float  samp_rate  = argc >= 4 ? std::stof(argv[3]) : kDefaultRate;
    const auto   os_factor  = static_cast<uint8_t>(samp_rate / kDefaultBW);

    // Print banner to stderr so it doesn't interfere with CBOR output
    auto* banner_fp = cbor_mode ? stderr : stdout;
    std::fprintf(banner_fp, "=== LoRa RX (SoapySDR via C bridge) ===\n");
    std::fprintf(banner_fp, "  Frequency:   %.6f MHz\n", freq / 1e6);
    std::fprintf(banner_fp, "  Gain:        %.0f dB\n", gain);
    std::fprintf(banner_fp, "  Sample rate: %.0f S/s\n", static_cast<double>(samp_rate));
    std::fprintf(banner_fp, "  OS factor:   %u\n", os_factor);
    std::fprintf(banner_fp, "  SF=%u  BW=%u  CR=4/8  sync=0x%02X\n",
                 kDefaultSF, kDefaultBW, kDefaultSync);
    std::fprintf(banner_fp, "  Device:      uhd (B210), clock_source=external\n");
    std::fprintf(banner_fp, "  Output:      %s\n", cbor_mode ? "CBOR" : "text");
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
        {"sample_rate", samp_rate},
        {"center_freq", freq},
        {"gain", gain},
    });

    auto& detector = graph.emplaceBlock<gr::lora::BurstDetector>({
        {"center_freq", static_cast<uint32_t>(freq)},
        {"bandwidth", kDefaultBW},
        {"sf", static_cast<uint8_t>(kDefaultSF)},
        {"sync_word", kDefaultSync},
        {"os_factor", os_factor},
        {"preamble_len", static_cast<uint16_t>(8)},
    });

    auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>({
        {"sf", static_cast<uint8_t>(kDefaultSF)},
        {"bandwidth", kDefaultBW},
    });

    auto& sink = graph.emplaceBlock<gr::lora::ConsoleSink>({
        {"output_mode", std::string(cbor_mode ? "cbor" : "text")},
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
