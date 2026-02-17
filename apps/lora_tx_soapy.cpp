// SPDX-License-Identifier: ISC
//
// lora_tx_soapy: Hardware LoRa transmitter using SoapySDR + B210.
//
// Generates a complete LoRa frame from a payload string using the shared
// algorithm functions (no GR4 graph needed for batch TX), then writes the
// IQ to the B210 via the pure-C SoapySDR bridge.
//
// Usage: lora_tx_soapy [options] <payload_string>
//
//   --freq <hz>       TX center frequency (default: 869525000)
//   --gain <db>       TX gain in dB (default: 30)
//   --rate <sps>      Sample rate in S/s (default: 250000)
//   --repeat <n>      Number of times to transmit (default: 1)
//   --gap <ms>        Gap between repeats in milliseconds (default: 1000)
//   --dry-run         Generate IQ but do not transmit (dump stats to stderr)
//   -h, --help        Show usage
//
// LoRa config: SF=8, BW=62500, CR=4/8, sync=0x12, explicit header, CRC on
// (defaults match common mesh network configurations)
//
// *** SAFETY: This program transmits on radio frequencies. ***
// *** Ensure you have authorization to transmit on the configured frequency. ***
// *** The --dry-run flag generates IQ without transmitting. ***

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

#include "soapy_c_bridge.h"

namespace {

// ---- CLI argument parsing ----

struct TxConfig {
    double      freq{869'525'000.0};
    double      gain{30.0};
    float       rate{250'000.f};
    uint32_t    bw{62500};
    uint8_t     sf{8};
    uint8_t     cr{4};
    uint16_t    sync_word{0x12};
    uint16_t    preamble_len{8};
    int         repeat{1};
    int         gap_ms{1000};
    bool        dry_run{false};
    std::string payload;
};

void print_usage(const char* prog) {
    std::fprintf(stderr, "Usage: %s [options] <payload_string>\n", prog);
    std::fprintf(stderr, "\nOptions:\n");
    std::fprintf(stderr, "  --freq <hz>       TX frequency (default: 869525000)\n");
    std::fprintf(stderr, "  --gain <db>       TX gain (default: 30)\n");
    std::fprintf(stderr, "  --rate <sps>      Sample rate (default: 250000)\n");
    std::fprintf(stderr, "  --bw <hz>         LoRa bandwidth (default: 62500)\n");
    std::fprintf(stderr, "  --sf <7-12>       Spreading factor (default: 8)\n");
    std::fprintf(stderr, "  --cr <1-4>        Coding rate (default: 4)\n");
    std::fprintf(stderr, "  --sync <hex>      Sync word, e.g. 0x12 (default: 0x12)\n");
    std::fprintf(stderr, "  --preamble <n>    Preamble length (default: 8)\n");
    std::fprintf(stderr, "  --repeat <n>      Transmit count (default: 1)\n");
    std::fprintf(stderr, "  --gap <ms>        Gap between repeats (default: 1000)\n");
    std::fprintf(stderr, "  --dry-run         Generate IQ without transmitting\n");
    std::fprintf(stderr, "  -h, --help        Show this help\n");
    std::fprintf(stderr, "\n*** Ensure you have authorization to transmit! ***\n");
}

bool parse_args(int argc, char** argv, TxConfig& cfg) {
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
        } else if (arg == "--cr" && i + 1 < argc) {
            cfg.cr = static_cast<uint8_t>(std::stoul(argv[++i]));
        } else if (arg == "--sync" && i + 1 < argc) {
            cfg.sync_word = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0));
        } else if (arg == "--preamble" && i + 1 < argc) {
            cfg.preamble_len = static_cast<uint16_t>(std::stoul(argv[++i]));
        } else if (arg == "--repeat" && i + 1 < argc) {
            cfg.repeat = std::stoi(argv[++i]);
        } else if (arg == "--gap" && i + 1 < argc) {
            cfg.gap_ms = std::stoi(argv[++i]);
        } else if (arg == "--dry-run") {
            cfg.dry_run = true;
        } else if (arg[0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            print_usage(argv[0]);
            return false;
        } else {
            // Remaining args are the payload
            cfg.payload = arg;
            for (int j = i + 1; j < argc; j++) {
                cfg.payload += " ";
                cfg.payload += argv[j];
            }
            break;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    TxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    if (cfg.payload.empty()) {
        std::fprintf(stderr, "Error: no payload specified.\n\n");
        print_usage(argv[0]);
        return 1;
    }

    if (cfg.payload.size() > 255) {
        std::fprintf(stderr, "Error: payload too long (%zu bytes, max 255)\n",
                     cfg.payload.size());
        return 1;
    }

    // --- Generate IQ ---
    const auto os_factor = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));
    const uint32_t sps = (1u << cfg.sf) * os_factor;
    std::vector<uint8_t> payload_bytes(cfg.payload.begin(), cfg.payload.end());
    auto iq = gr::lora::generate_frame_iq(payload_bytes, cfg.sf, cfg.cr,
                                          os_factor, cfg.sync_word,
                                          cfg.preamble_len, true, sps * 2,
                                          2, cfg.bw);

    double airtime_sec = static_cast<double>(iq.size()) /
                         static_cast<double>(cfg.rate);

    std::fprintf(stderr, "=== LoRa TX (SoapySDR via C bridge) ===\n");
    std::fprintf(stderr, "  Payload:     \"%s\" (%zu bytes)\n",
                 cfg.payload.c_str(), cfg.payload.size());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  Sample rate: %.0f S/s\n",
                 static_cast<double>(cfg.rate));
    std::fprintf(stderr, "  SF=%u  BW=%u  CR=4/%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, 4 + cfg.cr, cfg.sync_word, cfg.preamble_len);
    std::fprintf(stderr, "  IQ samples:  %zu (%.3f ms airtime)\n",
                 iq.size(), airtime_sec * 1000.0);
    std::fprintf(stderr, "  Repeat:      %d  gap=%d ms\n",
                 cfg.repeat, cfg.gap_ms);
    std::fprintf(stderr, "  Mode:        %s\n",
                 cfg.dry_run ? "DRY RUN (no transmission)" : "TRANSMIT");
    std::fprintf(stderr, "\n");

    if (cfg.dry_run) {
        std::fprintf(stderr, "Dry run complete. IQ frame generated but not sent.\n");
        return 0;
    }

    // --- Probe for hardware ---
    std::fprintf(stderr, "Probing for device...\n");
    if (!soapy_bridge_probe("uhd")) {
        std::fprintf(stderr, "Error: No UHD device found. Is the B210 connected?\n");
        return 1;
    }

    // --- Create TX device ---
    soapy_bridge_config_t bridge_cfg{};
    bridge_cfg.driver       = "uhd";
    bridge_cfg.clock_source = "external";
    bridge_cfg.device_args  = nullptr;
    bridge_cfg.sample_rate  = static_cast<double>(cfg.rate);
    bridge_cfg.center_freq  = cfg.freq;
    bridge_cfg.gain_db      = cfg.gain;
    bridge_cfg.bandwidth    = 0;  // auto
    bridge_cfg.channel      = 0;
    bridge_cfg.antenna      = nullptr;
    bridge_cfg.direction    = SOAPY_BRIDGE_TX;

    soapy_bridge_t* bridge = soapy_bridge_create(&bridge_cfg);
    if (!bridge) {
        std::fprintf(stderr, "Error: %s\n", soapy_bridge_last_error());
        return 1;
    }

    std::fprintf(stderr, "TX device ready. Actual: rate=%.0f freq=%.0f gain=%.1f\n",
                 soapy_bridge_get_sample_rate(bridge),
                 soapy_bridge_get_center_freq(bridge),
                 soapy_bridge_get_gain(bridge));

    // --- Transmit loop ---
    auto* buf = reinterpret_cast<const float*>(iq.data());
    auto total_samples = iq.size();

    for (int tx_num = 0; tx_num < cfg.repeat; tx_num++) {
        if (cfg.repeat > 1) {
            std::fprintf(stderr, "TX %d/%d...\n", tx_num + 1, cfg.repeat);
        }

        std::size_t offset = 0;
        while (offset < total_samples) {
            std::size_t chunk = std::min(total_samples - offset,
                                         std::size_t{8192});
            int is_last = (offset + chunk >= total_samples) ? 1 : 0;
            int ret = soapy_bridge_write(
                bridge,
                buf + offset * 2,  // interleaved float pairs
                chunk,
                100000,    // 100ms timeout
                is_last);  // end-of-burst on last chunk

            if (ret < 0) {
                std::fprintf(stderr, "Write error: %d\n", ret);
                soapy_bridge_destroy(bridge);
                return 1;
            }
            offset += static_cast<std::size_t>(ret);
        }

        std::fprintf(stderr, "  Sent %zu samples (%.3f ms)\n",
                     total_samples, airtime_sec * 1000.0);

        // Gap between repeats
        if (tx_num + 1 < cfg.repeat && cfg.gap_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(cfg.gap_ms));
        }
    }

    // --- Cleanup ---
    soapy_bridge_destroy(bridge);
    std::fprintf(stderr, "\nDone.\n");
    return 0;
}
