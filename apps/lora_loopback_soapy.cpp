// SPDX-License-Identifier: ISC
//
// lora_loopback_soapy: Hardware TX->RX challenge-response loopback on B210.
//
// Opens both TX and RX streams on the same B210 (SoapySDR shares the device
// via reference counting). Runs a two-round challenge-response protocol:
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
// Usage: lora_loopback_soapy [--freq hz] [--tx-gain db] [--rx-gain db]
//                            [--payload "text"]  (disables challenge-response)

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

#include "soapy_c_bridge.h"

namespace {

// MeshCore LoRa PHY parameters
constexpr uint8_t  SF           = 8;
constexpr uint32_t N            = 1u << SF;
constexpr uint8_t  CR           = 4;
constexpr uint32_t BW           = 62500;
constexpr uint8_t  OS_FACTOR    = 4;
constexpr uint16_t SYNC_WORD    = 0x12;
constexpr uint16_t PREAMBLE_LEN = 8;
constexpr uint32_t SPS          = N * OS_FACTOR;
constexpr float    SAMPLE_RATE  = 250000.f;
constexpr double   DEFAULT_FREQ = 869'525'000.0;
constexpr double   TX_GAIN      = 30.0;
constexpr double   RX_GAIN      = 40.0;

// ---- RX decode (offline, using GR4 graph) ----

struct DecodeResult {
    std::vector<uint8_t> payload;
    bool                 crc_valid{false};
    bool                 decoded{false};
};

DecodeResult decode_iq(const std::vector<std::complex<float>>& iq) {
    gr::Graph graph;

    auto& src = graph.emplaceBlock<gr::testing::TagSource<std::complex<float>,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false}, {"mark_tag", false}
    });
    src.values = iq;

    auto& burst = graph.emplaceBlock<gr::lora::BurstDetector>();
    burst.center_freq  = static_cast<uint32_t>(DEFAULT_FREQ);
    burst.bandwidth    = BW;
    burst.sf           = SF;
    burst.sync_word    = SYNC_WORD;
    burst.os_factor    = OS_FACTOR;
    burst.preamble_len = PREAMBLE_LEN;

    auto& demod = graph.emplaceBlock<gr::lora::SymbolDemodulator>();
    demod.sf        = SF;
    demod.bandwidth = BW;

    auto& sink = graph.emplaceBlock<gr::testing::TagSink<uint8_t,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true}, {"log_tags", true}
    });

    graph.connect<"out">(src).to<"in">(burst);
    graph.connect<"out">(burst).to<"in">(demod);
    graph.connect<"out">(demod).to<"in">(sink);

    gr::scheduler::Simple sched;
    sched.exchange(std::move(graph));
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
        const std::string& payload_str) {
    std::vector<uint8_t> payload_bytes(payload_str.begin(),
                                        payload_str.end());
    auto frame_iq = gr::lora::generate_frame_iq(
        payload_bytes, SF, CR, OS_FACTOR, SYNC_WORD, PREAMBLE_LEN, true, 0);
    uint32_t pad = static_cast<uint32_t>(SAMPLE_RATE * 0.1f);  // 100ms
    std::vector<std::complex<float>> iq;
    iq.resize(pad, std::complex<float>(0.f, 0.f));
    iq.insert(iq.end(), frame_iq.begin(), frame_iq.end());
    iq.resize(iq.size() + pad, std::complex<float>(0.f, 0.f));
    return iq;
}

/// Execute one TX->RX round: transmit payload, capture IQ, decode offline.
RoundResult do_round(const std::string& label,
                     const std::string& payload_str,
                     soapy_bridge_t* tx_bridge,
                     soapy_bridge_t* rx_bridge) {
    RoundResult rr;
    rr.tx_string = payload_str;

    std::fprintf(stderr, "\n--- %s ---\n", label.c_str());
    std::fprintf(stderr, "  TX: \"%s\" (%zu bytes)\n",
                 payload_str.c_str(), payload_str.size());

    auto tx_iq = build_padded_iq(payload_str);

    // RX capture: frame + 500ms margin
    std::size_t rx_total = tx_iq.size() + static_cast<std::size_t>(
        SAMPLE_RATE * 0.5f);
    std::vector<std::complex<float>> rx_buf(rx_total);
    std::atomic<std::size_t> rx_captured{0};

    // Start RX thread
    std::thread rx_thread([&]() {
        std::size_t offset = 0;
        while (offset < rx_total) {
            std::size_t chunk = std::min(rx_total - offset,
                                         std::size_t{4096});
            auto* buf = reinterpret_cast<float*>(&rx_buf[offset]);
            int ret = soapy_bridge_read(rx_bridge, buf, chunk, 100000);
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
        int ret = soapy_bridge_write(tx_bridge,
                                      tx_buf + tx_offset * 2,
                                      chunk, 100000, is_last);
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
    rr.decode = decode_iq(rx_buf);

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
    double      freq{DEFAULT_FREQ};
    double      tx_gain{TX_GAIN};
    double      rx_gain{RX_GAIN};
    std::string payload;  // empty = challenge-response mode
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [--freq hz] [--tx-gain db] [--rx-gain db] "
        "[--payload \"text\"]\n\n"
        "Without --payload: runs challenge-response protocol (2 rounds).\n"
        "With --payload:    single-round loopback with fixed payload.\n",
        prog);
}

bool parse_args(int argc, char** argv, LoopbackConfig& cfg) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        } else if (arg == "--freq" && i + 1 < argc) {
            cfg.freq = std::stod(argv[++i]);
        } else if (arg == "--tx-gain" && i + 1 < argc) {
            cfg.tx_gain = std::stod(argv[++i]);
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            cfg.rx_gain = std::stod(argv[++i]);
        } else if (arg == "--payload" && i + 1 < argc) {
            cfg.payload = argv[++i];
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

    std::fprintf(stderr, "=== LoRa Hardware Loopback Test ===\n");
    std::fprintf(stderr, "  Mode:      %s\n",
                 challenge_mode ? "challenge-response (2 rounds)"
                                : "fixed payload (1 round)");
    std::fprintf(stderr, "  Frequency: %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  TX gain:   %.0f dB  RX gain: %.0f dB\n",
                 cfg.tx_gain, cfg.rx_gain);
    std::fprintf(stderr, "  SF=%u BW=%u CR=4/%u sync=0x%02X\n",
                 SF, BW, 4 + CR, SYNC_WORD);

    // --- Probe device ---
    std::fprintf(stderr, "\nProbing for B210...\n");
    if (!soapy_bridge_probe("uhd")) {
        std::fprintf(stderr, "ERROR: No UHD device found.\n");
        return 1;
    }

    // --- Open RX stream ---
    std::fprintf(stderr, "Opening RX stream...\n");
    soapy_bridge_config_t rx_cfg{};
    rx_cfg.driver       = "uhd";
    rx_cfg.clock_source = "external";
    rx_cfg.device_args  = nullptr;
    rx_cfg.sample_rate  = static_cast<double>(SAMPLE_RATE);
    rx_cfg.center_freq  = cfg.freq;
    rx_cfg.gain_db      = cfg.rx_gain;
    rx_cfg.bandwidth    = 0;
    rx_cfg.channel      = 0;
    rx_cfg.antenna      = "RX2";
    rx_cfg.direction    = SOAPY_BRIDGE_RX;

    soapy_bridge_t* rx_bridge = soapy_bridge_create(&rx_cfg);
    if (!rx_bridge) {
        std::fprintf(stderr, "ERROR: RX create failed: %s\n",
                     soapy_bridge_last_error());
        return 1;
    }

    // --- Open TX stream ---
    std::fprintf(stderr, "Opening TX stream...\n");
    soapy_bridge_config_t tx_cfg{};
    tx_cfg.driver       = "uhd";
    tx_cfg.clock_source = "external";
    tx_cfg.device_args  = nullptr;
    tx_cfg.sample_rate  = static_cast<double>(SAMPLE_RATE);
    tx_cfg.center_freq  = cfg.freq;
    tx_cfg.gain_db      = cfg.tx_gain;
    tx_cfg.bandwidth    = 0;
    tx_cfg.channel      = 0;
    tx_cfg.antenna      = "TX/RX";
    tx_cfg.direction    = SOAPY_BRIDGE_TX;

    soapy_bridge_t* tx_bridge = soapy_bridge_create(&tx_cfg);
    if (!tx_bridge) {
        std::fprintf(stderr, "ERROR: TX create failed: %s\n",
                     soapy_bridge_last_error());
        soapy_bridge_destroy(rx_bridge);
        return 1;
    }

    int exit_code = 0;

    if (challenge_mode) {
        // ---- Challenge-Response Protocol ----
        std::string nonce = random_hex_nonce(4);  // 8 hex chars
        std::string challenge = "CHAL:" + nonce;

        // Round 1: Challenge
        auto r1 = do_round("Round 1: Challenge", challenge,
                           tx_bridge, rx_bridge);
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
                               tx_bridge, rx_bridge);
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
                          tx_bridge, rx_bridge);

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
    soapy_bridge_destroy(tx_bridge);
    soapy_bridge_destroy(rx_bridge);

    return exit_code;
}
