// SPDX-License-Identifier: ISC
//
// lora_loopback: Hardware TX->RX challenge-response loopback test.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Opens both TX and RX streams on the same device (the backend
// shares the device via reference counting). Runs a two-round
// challenge-response protocol:
//
//   Round 1 (Challenge): TX a random nonce "CHAL:<hex8>", RX and decode it.
//   Round 2 (Response):  TX "RESP:<decoded_nonce>", RX and decode it.
//                         The response payload is built from what was *decoded*
//                         in round 1, not from the original TX data. This proves
//                         both TX and RX are processing live RF, not stale data.
//
// Both rounds must decode with CRC valid and matching payloads for the test
// to pass. A fixed-payload mode is also available for debugging.
//
// Default: TX on channel 0 (TX/RX port), RX on channel 0 (RX2 port).
//
// Usage: lora_loopback [--args str] [--clock src] [--freq hz]
//                      [--tx-gain db] [--rx-gain db]
//                      [--payload "text"]  (disables challenge-response)

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

#include "radio_bridge.h"

namespace {

// Default LoRa PHY parameters (SF8/BW62.5k/CR4/8)
constexpr uint8_t  kDefaultSF           = 8;
constexpr uint8_t  kDefaultCR           = 4;
constexpr uint32_t kDefaultBW           = 62500;
constexpr uint16_t kDefaultSyncWord     = 0x12;
constexpr uint16_t kDefaultPreambleLen  = 8;
constexpr float    kDefaultSampleRate   = 250000.f;
constexpr double   kDefaultFreq         = 869'618'000.0;
constexpr double   kDefaultTxGain       = 30.0;
constexpr double   kDefaultRxGain       = 40.0;

// ---- RX decode (offline, using GR4 graph) ----

struct LoraParams {
    uint8_t  sf{kDefaultSF};
    uint8_t  cr{kDefaultCR};
    uint32_t bw{kDefaultBW};
    uint16_t sync_word{kDefaultSyncWord};
    uint16_t preamble_len{kDefaultPreambleLen};
    float    sample_rate{kDefaultSampleRate};
    double   freq{kDefaultFreq};
};

struct DecodeResult {
    std::vector<uint8_t> payload;
    bool                 crc_valid{false};
    bool                 decoded{false};
};

DecodeResult decode_iq(const std::vector<std::complex<float>>& iq,
                       const LoraParams& lp) {
    gr::Graph graph;

    auto os_factor = static_cast<uint8_t>(lp.sample_rate / static_cast<float>(lp.bw));

    auto& src = graph.emplaceBlock<gr::testing::TagSource<std::complex<float>,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false}, {"mark_tag", false}
    });
    src.values = iq;

    auto& burst = graph.emplaceBlock<gr::lora::BurstDetector>();
    burst.center_freq  = static_cast<uint32_t>(lp.freq);
    burst.bandwidth    = lp.bw;
    burst.sf           = lp.sf;
    burst.sync_word    = lp.sync_word;
    burst.os_factor    = os_factor;
    burst.preamble_len = lp.preamble_len;

    auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>();
    demod.sf        = lp.sf;
    demod.bandwidth = lp.bw;

    auto& sink = graph.emplaceBlock<gr::testing::TagSink<uint8_t,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true}, {"log_tags", true}
    });

    (void)graph.connect<"out">(src).to<"in">(burst);
    (void)graph.connect<"out">(burst).to<"in">(demod);
    (void)graph.connect<"out">(demod).to<"in">(sink);

    gr::scheduler::Simple sched;
    (void)sched.exchange(std::move(graph));
    sched.runAndWait();

    DecodeResult result;
    result.payload = sink._samples;

    for (const auto& tag : sink._tags) {
        if (auto it = tag.map.find("crc_valid"); it != tag.map.end()) {
            result.crc_valid = pmtv::cast<bool>(it->second);
            result.decoded = true;
        }
    }

    return result;
}

// ---- Hardware TX->RX round: transmit IQ and capture RX simultaneously ----

struct RoundResult {
    DecodeResult         decode;
    std::string          tx_string;
    std::string          rx_string;
    bool                 match{false};
};

/// Build padded TX IQ with leading/trailing silence.
std::vector<std::complex<float>> build_padded_iq(
        const std::string& payload_str, const LoraParams& lp) {
    auto os_factor = static_cast<uint8_t>(lp.sample_rate / static_cast<float>(lp.bw));
    std::vector<uint8_t> payload_bytes(payload_str.begin(),
                                        payload_str.end());
    auto frame_iq = gr::lora::generate_frame_iq(
        payload_bytes, lp.sf, lp.cr, os_factor,
        lp.sync_word, lp.preamble_len, true, 0,
        2, lp.bw);
    uint32_t pad = static_cast<uint32_t>(lp.sample_rate * 0.1f);  // 100ms
    std::vector<std::complex<float>> iq;
    iq.resize(pad, std::complex<float>(0.f, 0.f));
    iq.insert(iq.end(), frame_iq.begin(), frame_iq.end());
    iq.resize(iq.size() + pad, std::complex<float>(0.f, 0.f));
    return iq;
}

/// Execute one TX->RX round: transmit payload, capture IQ, decode offline.
RoundResult do_round(const std::string& label,
                     const std::string& payload_str,
                     radio_bridge_t* tx_bridge,
                     radio_bridge_t* rx_bridge,
                     const LoraParams& lp) {
    RoundResult rr;
    rr.tx_string = payload_str;

    std::fprintf(stderr, "\n--- %s ---\n", label.c_str());
    std::fprintf(stderr, "  TX: \"%s\" (%zu bytes)\n",
                 payload_str.c_str(), payload_str.size());

    auto tx_iq = build_padded_iq(payload_str, lp);

    // RX capture: frame + 500ms margin
    std::size_t rx_total = tx_iq.size() + static_cast<std::size_t>(
        lp.sample_rate * 0.5f);
    std::vector<std::complex<float>> rx_buf(rx_total);
    std::atomic<std::size_t> rx_captured{0};

    // Start RX thread
    std::thread rx_thread([&]() {
        std::size_t offset = 0;
        while (offset < rx_total) {
            std::size_t chunk = std::min(rx_total - offset,
                                         std::size_t{4096});
            auto* buf = reinterpret_cast<float*>(&rx_buf[offset]);
            int ret = radio_bridge_read(rx_bridge, buf, chunk, 0.1);
            if (ret > 0) {
                offset += static_cast<std::size_t>(ret);
                rx_captured.store(offset, std::memory_order_relaxed);
            }
        }
    });

    // Let RX settle before TX
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // TX burst
    std::fprintf(stderr, "  Transmitting %zu samples...\n", tx_iq.size());
    const auto* tx_buf = reinterpret_cast<const float*>(tx_iq.data());
    std::size_t tx_offset = 0;
    while (tx_offset < tx_iq.size()) {
        std::size_t chunk = std::min(tx_iq.size() - tx_offset,
                                     std::size_t{4096});
        int is_last = (tx_offset + chunk >= tx_iq.size()) ? 1 : 0;
        int ret = radio_bridge_write(tx_bridge,
                                      tx_buf + tx_offset * 2,
                                      chunk, 0.1, is_last);
        if (ret < 0) {
            std::fprintf(stderr, "  TX write error: %d\n", ret);
            break;
        }
        tx_offset += static_cast<std::size_t>(ret);
    }
    std::fprintf(stderr, "  TX complete.\n");

    // Wait for RX
    rx_thread.join();
    std::fprintf(stderr, "  RX captured %zu samples.\n",
                 rx_captured.load());

    // Decode
    std::fprintf(stderr, "  Decoding...\n");
    rr.decode = decode_iq(rx_buf, lp);

    if (rr.decode.decoded) {
        // Trim to expected length
        std::size_t expected = payload_str.size();
        if (rr.decode.payload.size() >= expected) {
            rr.rx_string = std::string(
                rr.decode.payload.begin(),
                rr.decode.payload.begin() +
                static_cast<int64_t>(expected));
        } else {
            rr.rx_string = std::string(
                rr.decode.payload.begin(),
                rr.decode.payload.end());
        }
        rr.match = (rr.rx_string == rr.tx_string);
        std::fprintf(stderr, "  RX: \"%s\" CRC=%s match=%s\n",
                     rr.rx_string.c_str(),
                     rr.decode.crc_valid ? "OK" : "FAIL",
                     rr.match ? "YES" : "NO");
    } else {
        std::fprintf(stderr, "  RX: no frame detected!\n");
    }

    return rr;
}

// ---- Random nonce generation ----

std::string random_hex_nonce(int n_bytes) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 255);
    std::string hex;
    hex.reserve(static_cast<std::size_t>(n_bytes) * 2);
    for (int i = 0; i < n_bytes; i++) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x",
                      static_cast<unsigned>(dist(gen)));
        hex += buf;
    }
    return hex;
}

// ---- CLI ----

struct LoopbackConfig {
    std::string args{};              ///< device args (empty = auto-discover)
    std::string clock{};             ///< clock source (empty = device default)
    double      freq{kDefaultFreq};
    double      tx_gain{kDefaultTxGain};
    double      rx_gain{kDefaultRxGain};
    uint32_t    bw{kDefaultBW};
    uint8_t     sf{kDefaultSF};
    uint8_t     cr{kDefaultCR};
    uint16_t    sync_word{kDefaultSyncWord};
    uint16_t    preamble_len{kDefaultPreambleLen};
    float       sample_rate{kDefaultSampleRate};
    std::string payload;  // empty = challenge-response mode
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n\n"
        "Options:\n"
        "  --args <str>      Device args (default: auto-discover)\n"
        "  --clock <src>     Clock source (default: device default)\n"
        "  --freq <hz>       Frequency (default: 869618000)\n"
        "  --tx-gain <db>    TX gain (default: 30)\n"
        "  --rx-gain <db>    RX gain (default: 40)\n"
        "  --bw <hz>         LoRa bandwidth (default: 62500)\n"
        "  --sf <7-12>       Spreading factor (default: 8)\n"
        "  --cr <1-4>        Coding rate (default: 4)\n"
        "  --sync <hex>      Sync word (default: 0x12)\n"
        "  --preamble <n>    Preamble length (default: 8)\n"
        "  --payload \"text\"  Fixed payload (disables challenge-response)\n"
        "\nWithout --payload: runs challenge-response protocol (2 rounds).\n",
        prog);
}

bool parse_args(int argc, char** argv, LoopbackConfig& cfg) {
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
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            cfg.tx_gain = std::stod(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            cfg.rx_gain = std::stod(argv[++i]);
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
        } else if (arg == "--payload" && i + 1 < argc) {
            cfg.payload = argv[++i];
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    LoopbackConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    bool challenge_mode = cfg.payload.empty();

    // Build LoraParams from config
    LoraParams lp;
    lp.sf           = cfg.sf;
    lp.cr           = cfg.cr;
    lp.bw           = cfg.bw;
    lp.sync_word    = cfg.sync_word;
    lp.preamble_len = cfg.preamble_len;
    lp.sample_rate  = cfg.sample_rate;
    lp.freq         = cfg.freq;

    std::fprintf(stderr, "=== LoRa Hardware Loopback Test ===\n");
    std::fprintf(stderr, "  Device args: %s\n",
                 cfg.args.empty() ? "(auto)" : cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n",
                 cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  Mode:        %s\n",
                 challenge_mode ? "challenge-response (2 rounds)"
                                 : "fixed payload (1 round)");
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  TX gain:     %.0f dB  RX gain: %.0f dB\n",
                 cfg.tx_gain, cfg.rx_gain);
    std::fprintf(stderr, "  SF=%u BW=%u CR=4/%u sync=0x%02X preamble=%u\n",
                 cfg.sf, cfg.bw, 4 + cfg.cr, cfg.sync_word, cfg.preamble_len);

    // --- Probe device ---
    std::fprintf(stderr, "\nProbing for device...\n");
    const char* probe_args = cfg.args.empty() ? nullptr : cfg.args.c_str();
    if (!radio_bridge_probe(probe_args)) {
        std::fprintf(stderr, "ERROR: No device found.\n");
        return 1;
    }

    // --- Open RX stream ---
    std::fprintf(stderr, "Opening RX stream...\n");
    radio_bridge_config_t rx_cfg{};
    rx_cfg.device_args  = cfg.args.empty() ? nullptr : cfg.args.c_str();
    rx_cfg.clock_source = cfg.clock.empty() ? nullptr : cfg.clock.c_str();
    rx_cfg.sample_rate  = static_cast<double>(cfg.sample_rate);
    rx_cfg.center_freq  = cfg.freq;
    rx_cfg.gain_db      = cfg.rx_gain;
    rx_cfg.bandwidth    = 0;
    rx_cfg.channel      = 0;
    rx_cfg.antenna      = nullptr;  // device default
    rx_cfg.direction    = RADIO_BRIDGE_RX;

    radio_bridge_t* rx_bridge = radio_bridge_create(&rx_cfg);
    if (!rx_bridge) {
        std::fprintf(stderr, "ERROR: RX create failed: %s\n",
                     radio_bridge_last_error());
        return 1;
    }

    // --- Open TX stream ---
    std::fprintf(stderr, "Opening TX stream...\n");
    radio_bridge_config_t tx_cfg{};
    tx_cfg.device_args  = cfg.args.empty() ? nullptr : cfg.args.c_str();
    tx_cfg.clock_source = cfg.clock.empty() ? nullptr : cfg.clock.c_str();
    tx_cfg.sample_rate  = static_cast<double>(cfg.sample_rate);
    tx_cfg.center_freq  = cfg.freq;
    tx_cfg.gain_db      = cfg.tx_gain;
    tx_cfg.bandwidth    = 0;
    tx_cfg.channel      = 0;
    tx_cfg.antenna      = nullptr;  // device default
    tx_cfg.direction    = RADIO_BRIDGE_TX;

    radio_bridge_t* tx_bridge = radio_bridge_create(&tx_cfg);
    if (!tx_bridge) {
        std::fprintf(stderr, "ERROR: TX create failed: %s\n",
                     radio_bridge_last_error());
        radio_bridge_destroy_ex(rx_bridge, 1);
        _exit(1);
    }

    int exit_code = 0;

    if (challenge_mode) {
        // ---- Challenge-Response Protocol ----
        std::string nonce = random_hex_nonce(4);  // 8 hex chars
        std::string challenge = "CHAL:" + nonce;

        // Round 1: Challenge
        auto r1 = do_round("Round 1: Challenge", challenge,
                           tx_bridge, rx_bridge, lp);
        if (!r1.decode.decoded || !r1.decode.crc_valid || !r1.match) {
            std::fprintf(stderr, "\nRound 1 FAILED.\n");
            exit_code = 1;
        } else {
            // Round 2: Response — build from *decoded* payload
            // Extract the nonce from the decoded challenge string
            std::string decoded_nonce;
            if (r1.rx_string.size() > 5 &&
                r1.rx_string.substr(0, 5) == "CHAL:") {
                decoded_nonce = r1.rx_string.substr(5);
            } else {
                decoded_nonce = r1.rx_string;
            }
            std::string response = "RESP:" + decoded_nonce;

            // Brief pause between rounds
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            auto r2 = do_round("Round 2: Response", response,
                               tx_bridge, rx_bridge, lp);
            if (!r2.decode.decoded || !r2.decode.crc_valid || !r2.match) {
                std::fprintf(stderr, "\nRound 2 FAILED.\n");
                exit_code = 1;
            }

            // Verify the response contains the original nonce
            bool nonce_in_response = (r2.rx_string.find(nonce) !=
                                      std::string::npos);

            std::fprintf(stderr, "\n=============================\n");
            std::fprintf(stderr, "  CHALLENGE-RESPONSE RESULT\n");
            std::fprintf(stderr, "=============================\n");
            std::fprintf(stderr, "  Nonce:       %s\n", nonce.c_str());
            std::fprintf(stderr, "  R1 TX:       \"%s\"\n",
                         r1.tx_string.c_str());
            std::fprintf(stderr, "  R1 RX:       \"%s\"  CRC=%s\n",
                         r1.rx_string.c_str(),
                         r1.decode.crc_valid ? "OK" : "FAIL");
            std::fprintf(stderr, "  R2 TX:       \"%s\"\n",
                         r2.tx_string.c_str());
            std::fprintf(stderr, "  R2 RX:       \"%s\"  CRC=%s\n",
                         r2.rx_string.c_str(),
                         r2.decode.crc_valid ? "OK" : "FAIL");
            std::fprintf(stderr, "  Nonce echo:  %s\n",
                         nonce_in_response ? "YES" : "NO");

            if (r1.match && r1.decode.crc_valid &&
                r2.match && r2.decode.crc_valid &&
                nonce_in_response) {
                std::fprintf(stderr,
                    "\n  *** CHALLENGE-RESPONSE PASSED ***\n\n");
                exit_code = 0;
            } else {
                std::fprintf(stderr,
                    "\n  *** CHALLENGE-RESPONSE FAILED ***\n\n");
                exit_code = 1;
            }
        }
    } else {
        // ---- Single fixed-payload round ----
        auto r = do_round("Single Round", cfg.payload,
                          tx_bridge, rx_bridge, lp);

        std::fprintf(stderr, "\n=== LOOPBACK RESULT ===\n");
        std::fprintf(stderr, "  TX: \"%s\"\n", r.tx_string.c_str());
        std::fprintf(stderr, "  RX: \"%s\"\n", r.rx_string.c_str());
        std::fprintf(stderr, "  CRC:   %s\n",
                     r.decode.crc_valid ? "VALID" : "FAIL");
        std::fprintf(stderr, "  Match: %s\n",
                     r.match ? "YES" : "NO");

        if (r.match && r.decode.crc_valid) {
            std::fprintf(stderr,
                "\n  *** LOOPBACK TEST PASSED ***\n\n");
        } else {
            std::fprintf(stderr,
                "\n  *** LOOPBACK TEST FAILED ***\n\n");
            exit_code = 1;
        }
    }

    // --- Cleanup ---
    // Stop streams and release streamers, but skip device deallocation —
    // static teardown can crash under dual-libc++.
    radio_bridge_destroy_ex(tx_bridge, 1);
    radio_bridge_destroy_ex(rx_bridge, 1);

    _exit(exit_code);
}
