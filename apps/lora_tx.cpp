// SPDX-License-Identifier: ISC
//
// lora_tx: Hardware LoRa transmitter.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Generates a complete LoRa frame from a payload string using
// the shared algorithm functions (no GR4 graph needed for batch TX), then
// writes the IQ to the device via the pure-C bridge.
//
// Three modes:
//
//   CLI mode (default):
//     lora_tx [options] <payload_string>
//     Transmits one or more copies of the payload.
//
//   Stdin mode (--stdin):
//     lora_tx --stdin [--dry-run]
//     Reads concatenated CBOR TX request objects from stdin.
//     See docs/cbor-schemas.md for the schema.
//
//   Loopback mode (--loopback):
//     lora_tx --loopback [--payload "text"]
//     Opens both TX and RX on the same device (shared via the bridge's
//     reference counting). Runs a three-round test protocol:
//       Round 1 (Greeting):  TX "Hello From gnuradio-4.0 (<rev>)!"
//       Round 2 (Challenge): TX a random nonce "CHAL:<hex8>", RX and decode.
//       Round 3 (Response):  TX "RESP:<decoded_nonce>", RX and decode.
//     All rounds must decode with CRC valid and matching payloads.
//     Use --payload for single-round fixed-payload mode.
//
// *** SAFETY: This program transmits on radio frequencies. ***
// *** Ensure you have authorization to transmit on the configured frequency. ***
// *** The --dry-run flag generates IQ without transmitting. ***

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
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

#include "cbor.hpp"
#include "radio_bridge.h"

namespace {

// ---- CLI argument parsing ----

struct TxConfig {
    std::string args{"type=b200"};   ///< device args (default: B200/B210)
    std::string clock{};             ///< clock source (empty = device default)
    double      freq{869'618'000.0};
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
    bool        stdin_mode{false};
    bool        loopback{false};
    bool        loopback_cbor{false};  ///< output CBOR frame on stdout in loopback
    double      rx_gain{40.0};        ///< RX gain for loopback mode
    std::string payload;
    bool        payload_is_hex{false};  ///< payload was set via --payload-hex
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options] <payload_string>\n\n"
        "Device options:\n"
        "  --args <str>      Device args (default: type=b200)\n"
        "                    Examples: type=b200, addr=192.168.10.2\n"
        "  --clock <src>     Clock source (default: device default)\n"
        "                    Examples: internal, external, gpsdo\n"
        "  --freq <hz>       TX frequency (default: 869618000)\n"
        "  --gain <db>       TX gain (default: 30)\n"
        "  --rate <sps>      Sample rate (default: 250000)\n\n"
        "LoRa PHY options:\n"
        "  --bw <hz>         LoRa bandwidth (default: 62500)\n"
        "  --sf <7-12>       Spreading factor (default: 8)\n"
        "  --cr <1-4>        Coding rate (default: 4)\n"
        "  --sync <hex>      Sync word, e.g. 0x12 (default: 0x12)\n"
        "  --preamble <n>    Preamble length (default: 8)\n\n"
        "TX options:\n"
        "  --repeat <n>      Transmit count (default: 1)\n"
        "  --gap <ms>        Gap between repeats (default: 1000)\n"
        "  --dry-run         Generate IQ without transmitting\n"
        "  --stdin           Read CBOR TX requests from stdin\n\n"
        "Loopback mode:\n"
        "  --loopback            TX->RX challenge-response test on same device\n"
        "  --rx-gain <db>        RX gain for loopback (default: 40)\n"
        "  --payload <text>      Fixed text payload (disables challenge-response)\n"
        "  --payload-hex <hex>   Fixed binary payload as hex (e.g. MeshCore wire packet)\n"
        "  --cbor                Output decoded frame as CBOR on stdout (loopback mode)\n\n"
        "Other:\n"
        "  -h, --help            Show this help\n\n"
        "Examples:\n"
        "  %s \"Hello World\"\n"
        "  %s --args type=b200 --clock external \"Hello World\"\n"
        "  %s --sf 12 --bw 125000 --rate 500000 \"Hello World\"\n"
        "  %s --loopback --clock external\n"
        "  %s --loopback --payload \"Test 123\"\n"
        "  %s --loopback --payload-hex 1100abcd... --cbor | python3 scripts/lora_decode_meshcore.py\n\n"
        "*** Ensure you have authorization to transmit! ***\n",
        prog, prog, prog, prog, prog, prog, prog);
}

bool parse_args(int argc, char** argv, TxConfig& cfg) {
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
        } else if (arg == "--stdin") {
            cfg.stdin_mode = true;
        } else if (arg == "--loopback") {
            cfg.loopback = true;
        } else if (arg == "--rx-gain" && i + 1 < argc) {
            cfg.rx_gain = std::stod(argv[++i]);
        } else if (arg == "--payload" && i + 1 < argc) {
            cfg.payload = argv[++i];
        } else if (arg == "--payload-hex" && i + 1 < argc) {
            // Decode hex string into raw bytes stored in payload
            std::string hex = argv[++i];
            cfg.payload.clear();
            cfg.payload.reserve(hex.size() / 2);
            for (std::size_t j = 0; j + 1 < hex.size(); j += 2) {
                auto byte = static_cast<char>(
                    std::stoul(hex.substr(j, 2), nullptr, 16));
                cfg.payload.push_back(byte);
            }
            cfg.payload_is_hex = true;
        } else if (arg == "--cbor") {
            cfg.loopback_cbor = true;  // only used with --loopback
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

// ---- IQ generation ----

std::vector<std::complex<float>> generate_iq(const std::vector<uint8_t>& payload,
                                             const TxConfig& cfg) {
    const auto os_factor = static_cast<uint8_t>(
        cfg.rate / static_cast<float>(cfg.bw));
    const uint32_t sps = (1u << cfg.sf) * os_factor;
    return gr::lora::generate_frame_iq(payload, cfg.sf, cfg.cr,
                                       os_factor, cfg.sync_word,
                                       cfg.preamble_len, true, sps * 2,
                                       2, cfg.bw);
}

void log_tx_info(const std::vector<uint8_t>& payload,
                 const TxConfig& cfg,
                 std::size_t iq_len, double airtime_sec) {
    std::fprintf(stderr, "  Payload:     %zu bytes\n", payload.size());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  SF=%u  BW=%u  CR=4/%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, 4 + cfg.cr, cfg.sync_word, cfg.preamble_len);
    std::fprintf(stderr, "  IQ samples:  %zu (%.3f ms airtime)\n",
                 iq_len, airtime_sec * 1000.0);
}

// ---- Hardware TX ----

/// Send IQ samples through the bridge. Returns 0 on success.
int transmit_iq(radio_bridge_t* bridge,
                const std::vector<std::complex<float>>& iq) {
    const auto* buf = reinterpret_cast<const float*>(iq.data());
    auto total = iq.size();
    std::size_t offset = 0;
    while (offset < total) {
        std::size_t chunk = std::min(total - offset, std::size_t{8192});
        int is_last = (offset + chunk >= total) ? 1 : 0;
        int ret = radio_bridge_write(bridge, buf + offset * 2,
                                     chunk, 0.1, is_last);
        if (ret < 0) {
            std::fprintf(stderr, "Write error: %d\n", ret);
            return -1;
        }
        offset += static_cast<std::size_t>(ret);
    }
    return 0;
}

/// Probe for a device with a timeout. Returns true if found.
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

/// Open the TX hardware device. Returns nullptr on failure.
radio_bridge_t* open_tx_device(const TxConfig& cfg) {
    if (!probe_device_with_timeout(cfg.args, 10)) {
        std::fprintf(stderr, "Error: No device found. Is the device connected and powered?\n");
        return nullptr;
    }

    radio_bridge_config_t bridge_cfg{};
    bridge_cfg.device_args  = cfg.args.empty() ? nullptr : cfg.args.c_str();
    bridge_cfg.clock_source = cfg.clock.empty() ? nullptr : cfg.clock.c_str();
    bridge_cfg.sample_rate  = static_cast<double>(cfg.rate);
    bridge_cfg.center_freq  = cfg.freq;
    bridge_cfg.gain_db      = cfg.gain;
    bridge_cfg.bandwidth    = 0;  // auto
    bridge_cfg.channel      = 0;
    bridge_cfg.antenna      = nullptr;
    bridge_cfg.direction    = RADIO_BRIDGE_TX;

    radio_bridge_t* bridge = radio_bridge_create(&bridge_cfg);
    if (!bridge) {
        std::fprintf(stderr, "Error: %s\n", radio_bridge_last_error());
        return nullptr;
    }

    std::fprintf(stderr, "TX device ready. Actual: rate=%.0f freq=%.0f gain=%.1f\n",
                 radio_bridge_get_sample_rate(bridge),
                 radio_bridge_get_center_freq(bridge),
                 radio_bridge_get_gain(bridge));
    return bridge;
}

// ---- Stdin CBOR mode ----

/// Read exactly `n` bytes from fd. Returns false on EOF/error.
bool read_exact(int fd, uint8_t* buf, std::size_t n) {
    std::size_t total = 0;
    while (total < n) {
        auto r = ::read(fd, buf + total, n - total);
        if (r <= 0) return false;
        total += static_cast<std::size_t>(r);
    }
    return true;
}

/// Read a complete CBOR map object from fd. Returns empty on EOF/error.
std::vector<uint8_t> read_cbor_object(int fd) {
    std::vector<uint8_t> buf;
    buf.resize(1);
    if (!read_exact(fd, buf.data(), 1)) return {};

    uint8_t head = buf[0];
    uint8_t major = static_cast<uint8_t>(head >> 5);
    if (major != 5) {
        std::fprintf(stderr, "CBOR: expected map (major 5), got %u\n", major);
        return {};
    }

    buf.reserve(512);

    uint8_t info = static_cast<uint8_t>(head & 0x1F);
    std::size_t extra = 0;
    if (info >= 24 && info <= 27) {
        extra = std::size_t{1} << (info - 24);
        buf.resize(1 + extra);
        if (!read_exact(fd, buf.data() + 1, extra)) return {};
    }

    constexpr std::size_t kMaxSize = 65536;
    constexpr std::size_t kChunkSize = 256;

    while (buf.size() < kMaxSize) {
        try {
            gr::lora::cbor::decode_map(std::span<const uint8_t>(buf));
            return buf;
        } catch (const gr::lora::cbor::DecodeError&) {
            // Need more data
        }
        std::size_t prev = buf.size();
        buf.resize(prev + kChunkSize);
        auto r = ::read(fd, buf.data() + prev, kChunkSize);
        if (r <= 0) return {};
        buf.resize(prev + static_cast<std::size_t>(r));
    }
    std::fprintf(stderr, "CBOR: message too large (>64 KB)\n");
    return {};
}

/// Apply CBOR TX request fields onto a TxConfig (override defaults).
void apply_cbor_request(TxConfig& cfg,
                        const gr::lora::cbor::Map& msg) {
    using gr::lora::cbor::get_uint_or;
    using gr::lora::cbor::get_bool_or;

    cfg.freq     = static_cast<double>(
        get_uint_or(msg, "freq", static_cast<uint64_t>(cfg.freq)));
    cfg.gain     = static_cast<double>(
        get_uint_or(msg, "gain", static_cast<uint64_t>(cfg.gain)));
    cfg.sf       = static_cast<uint8_t>(get_uint_or(msg, "sf", cfg.sf));
    cfg.bw       = static_cast<uint32_t>(get_uint_or(msg, "bw", cfg.bw));
    cfg.cr       = static_cast<uint8_t>(get_uint_or(msg, "cr", cfg.cr));
    cfg.sync_word = static_cast<uint16_t>(
        get_uint_or(msg, "sync_word", cfg.sync_word));
    cfg.preamble_len = static_cast<uint16_t>(
        get_uint_or(msg, "preamble_len", cfg.preamble_len));
    cfg.repeat   = static_cast<int>(get_uint_or(msg, "repeat", 1));
    cfg.gap_ms   = static_cast<int>(get_uint_or(msg, "gap_ms", 1000));
    cfg.dry_run  = get_bool_or(msg, "dry_run", cfg.dry_run);
}

/// Emit a CBOR response on stdout: {"type":"lora_tx_ack","seq":n,"ok":bool}
void emit_ack(uint64_t seq, bool ok) {
    std::vector<uint8_t> buf;
    buf.reserve(64);
    gr::lora::cbor::encode_map_begin(buf, 3);
    gr::lora::cbor::kv_text(buf, "type", "lora_tx_ack");
    gr::lora::cbor::kv_uint(buf, "seq", seq);
    gr::lora::cbor::kv_bool(buf, "ok", ok);
    ::write(STDOUT_FILENO, buf.data(), buf.size());
}

int run_stdin_mode(TxConfig& cfg) {
    radio_bridge_t* bridge = nullptr;
    if (!cfg.dry_run) {
        bridge = open_tx_device(cfg);
        if (!bridge) return 1;
    }

    std::fprintf(stderr, "=== LoRa TX stdin mode (CBOR) ===\n");
    std::fprintf(stderr, "  Mode: %s\n",
                 cfg.dry_run ? "DRY RUN" : "TRANSMIT");
    std::fprintf(stderr, "  Reading CBOR TX requests from stdin...\n\n");

    uint64_t seq = 0;
    while (true) {
        auto raw = read_cbor_object(STDIN_FILENO);
        if (raw.empty()) break;

        seq++;
        try {
            auto msg = gr::lora::cbor::decode_map(
                std::span<const uint8_t>(raw));

            auto type = gr::lora::cbor::get_text_or(msg, "type", "");
            if (type != "lora_tx") {
                std::fprintf(stderr, "#%" PRIu64 ": unknown type '%s', skipping\n",
                             seq, type.c_str());
                emit_ack(seq, false);
                continue;
            }

            auto payload = gr::lora::cbor::get_bytes(msg, "payload");
            if (payload.empty() || payload.size() > 255) {
                std::fprintf(stderr, "#%" PRIu64 ": invalid payload size %zu\n",
                             seq, payload.size());
                emit_ack(seq, false);
                continue;
            }

            TxConfig req_cfg = cfg;
            apply_cbor_request(req_cfg, msg);

            auto iq = generate_iq(payload, req_cfg);
            double airtime = static_cast<double>(iq.size()) /
                             static_cast<double>(req_cfg.rate);

            std::fprintf(stderr, "#%" PRIu64 ": ", seq);
            log_tx_info(payload, req_cfg, iq.size(), airtime);

            bool ok = true;
            if (!cfg.dry_run) {
                for (int r = 0; r < req_cfg.repeat; r++) {
                    if (transmit_iq(bridge, iq) < 0) {
                        ok = false;
                        break;
                    }
                    if (r + 1 < req_cfg.repeat && req_cfg.gap_ms > 0) {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(req_cfg.gap_ms));
                    }
                }
            }
            emit_ack(seq, ok);
        } catch (const gr::lora::cbor::DecodeError& e) {
            std::fprintf(stderr, "#%" PRIu64 ": CBOR decode error: %s\n",
                         seq, e.what());
            emit_ack(seq, false);
        }
    }

    if (bridge) radio_bridge_destroy(bridge);
    std::fprintf(stderr, "\nStdin closed. %" PRIu64 " requests processed.\n",
                 seq);
    return 0;
}

// ---- Loopback mode ----

struct LoraParams {
    uint8_t  sf;
    uint8_t  cr;
    uint32_t bw;
    uint16_t sync_word;
    uint16_t preamble_len;
    float    sample_rate;
    double   freq;
};

struct DecodeResult {
    std::vector<uint8_t> payload;
    bool                 crc_valid{false};
    bool                 decoded{false};
};

/// Decode IQ samples offline using a GR4 graph.
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
    result.payload.assign(sink._samples.begin(), sink._samples.end());

    for (const auto& tag : sink._tags) {
        if (auto it = tag.map.find("crc_valid"); it != tag.map.end()) {
            result.crc_valid = it->second.value_or<bool>(false);
            result.decoded = true;
        }
    }

    return result;
}

struct RoundResult {
    DecodeResult         decode;
    std::vector<uint8_t> tx_bytes;
    std::vector<uint8_t> rx_bytes;
    bool                 match{false};
};

/// Format bytes as hex string.
std::string to_hex(const std::vector<uint8_t>& data, std::size_t max_bytes = 0) {
    std::string hex;
    std::size_t n = (max_bytes > 0) ? std::min(data.size(), max_bytes) : data.size();
    hex.reserve(n * 3);
    for (std::size_t i = 0; i < n; i++) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X ", data[i]);
        hex += buf;
    }
    if (max_bytes > 0 && data.size() > max_bytes) {
        hex += "...";
    }
    return hex;
}

/// Build padded TX IQ with leading/trailing silence.
std::vector<std::complex<float>> build_padded_iq(
        const std::vector<uint8_t>& payload_bytes, const LoraParams& lp) {
    auto os_factor = static_cast<uint8_t>(lp.sample_rate / static_cast<float>(lp.bw));
    auto frame_iq = gr::lora::generate_frame_iq(
        payload_bytes, lp.sf, lp.cr, os_factor,
        lp.sync_word, lp.preamble_len, true, 0,
        2, lp.bw);
    uint32_t pad = static_cast<uint32_t>(lp.sample_rate * 0.1f);
    std::vector<std::complex<float>> iq;
    iq.resize(pad, std::complex<float>(0.f, 0.f));
    iq.insert(iq.end(), frame_iq.begin(), frame_iq.end());
    iq.resize(iq.size() + pad, std::complex<float>(0.f, 0.f));
    return iq;
}

/// Execute one TX->RX round: transmit payload, capture IQ, decode offline.
RoundResult do_round(const std::string& label,
                     const std::vector<uint8_t>& payload_bytes,
                     radio_bridge_t* tx_bridge,
                     radio_bridge_t* rx_bridge,
                     const LoraParams& lp,
                     bool show_hex = false) {
    RoundResult rr;
    rr.tx_bytes = payload_bytes;

    std::fprintf(stderr, "\n--- %s ---\n", label.c_str());
    if (show_hex) {
        std::fprintf(stderr, "  TX: %zu bytes\n  Hex: %s\n",
                     payload_bytes.size(),
                     to_hex(payload_bytes, 48).c_str());
    } else {
        std::string tx_str(payload_bytes.begin(), payload_bytes.end());
        std::fprintf(stderr, "  TX: \"%s\" (%zu bytes)\n",
                     tx_str.c_str(), payload_bytes.size());
    }

    auto tx_iq = build_padded_iq(payload_bytes, lp);

    std::size_t rx_total = tx_iq.size() + static_cast<std::size_t>(
        lp.sample_rate * 0.5f);
    std::vector<std::complex<float>> rx_buf(rx_total);
    std::atomic<std::size_t> rx_captured{0};

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

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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

    rx_thread.join();
    std::fprintf(stderr, "  RX captured %zu samples.\n",
                 rx_captured.load());

    std::fprintf(stderr, "  Decoding...\n");
    rr.decode = decode_iq(rx_buf, lp);

    if (rr.decode.decoded) {
        std::size_t expected = payload_bytes.size();
        if (rr.decode.payload.size() >= expected) {
            rr.rx_bytes.assign(
                rr.decode.payload.begin(),
                rr.decode.payload.begin() +
                static_cast<int64_t>(expected));
        } else {
            rr.rx_bytes = rr.decode.payload;
        }
        rr.match = (rr.rx_bytes == rr.tx_bytes);
        if (show_hex) {
            std::fprintf(stderr, "  RX: %zu bytes  CRC=%s  match=%s\n"
                                 "  Hex: %s\n",
                         rr.rx_bytes.size(),
                         rr.decode.crc_valid ? "OK" : "FAIL",
                         rr.match ? "YES" : "NO",
                         to_hex(rr.rx_bytes, 48).c_str());
        } else {
            std::string rx_str(rr.rx_bytes.begin(), rr.rx_bytes.end());
            std::fprintf(stderr, "  RX: \"%s\" CRC=%s match=%s\n",
                         rx_str.c_str(),
                         rr.decode.crc_valid ? "OK" : "FAIL",
                         rr.match ? "YES" : "NO");
        }
    } else {
        std::fprintf(stderr, "  RX: no frame detected!\n");
    }

    return rr;
}

/// Generate a random hex nonce for challenge-response.
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

/// Build the default loopback payload: "Hello From gnuradio-4.0 (<rev>)!"
std::string default_loopback_payload() {
#ifdef GR4_GIT_REV
    return std::string("Hello From gnuradio-4.0 (") + GR4_GIT_REV + ")!";
#else
    return "Hello From gnuradio-4.0!";
#endif
}

int run_loopback(const TxConfig& cfg) {
    bool challenge_mode = cfg.payload.empty();

    LoraParams lp;
    lp.sf           = cfg.sf;
    lp.cr           = cfg.cr;
    lp.bw           = cfg.bw;
    lp.sync_word    = cfg.sync_word;
    lp.preamble_len = cfg.preamble_len;
    lp.sample_rate  = cfg.rate;
    lp.freq         = cfg.freq;

    std::fprintf(stderr, "=== LoRa Hardware Loopback Test ===\n");
    std::fprintf(stderr, "  Device args: %s\n",
                 cfg.args.empty() ? "(auto)" : cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n",
                 cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
    std::fprintf(stderr, "  Mode:        %s\n",
                 challenge_mode ? "greeting + challenge-response (3 rounds)"
                                 : "fixed payload (1 round)");
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  TX gain:     %.0f dB  RX gain: %.0f dB\n",
                 cfg.gain, cfg.rx_gain);
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
    rx_cfg.sample_rate  = static_cast<double>(cfg.rate);
    rx_cfg.center_freq  = cfg.freq;
    rx_cfg.gain_db      = cfg.rx_gain;
    rx_cfg.bandwidth    = 0;
    rx_cfg.channel      = 0;
    rx_cfg.antenna      = nullptr;
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
    tx_cfg.sample_rate  = static_cast<double>(cfg.rate);
    tx_cfg.center_freq  = cfg.freq;
    tx_cfg.gain_db      = cfg.gain;
    tx_cfg.bandwidth    = 0;
    tx_cfg.channel      = 0;
    tx_cfg.antenna      = nullptr;
    tx_cfg.direction    = RADIO_BRIDGE_TX;

    radio_bridge_t* tx_bridge = radio_bridge_create(&tx_cfg);
    if (!tx_bridge) {
        std::fprintf(stderr, "ERROR: TX create failed: %s\n",
                     radio_bridge_last_error());
        radio_bridge_destroy_ex(rx_bridge, 1);
        _exit(1);
    }

    int exit_code = 0;
    bool is_hex = cfg.payload_is_hex;

    // Helper to convert string to byte vector.
    auto to_bytes = [](const std::string& s) -> std::vector<uint8_t> {
        return {s.begin(), s.end()};
    };
    // Helper to convert byte vector to string.
    auto to_string = [](const std::vector<uint8_t>& v) -> std::string {
        return {v.begin(), v.end()};
    };

    if (challenge_mode) {
        // Default loopback: version-stamped greeting + challenge-response
        std::string greeting = default_loopback_payload();
        auto r0 = do_round("Round 1: Greeting", to_bytes(greeting),
                           tx_bridge, rx_bridge, lp);

        if (!r0.decode.decoded || !r0.decode.crc_valid || !r0.match) {
            std::fprintf(stderr, "\nRound 1 (greeting) FAILED.\n");
            exit_code = 1;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            std::string nonce = random_hex_nonce(4);
            std::string challenge = "CHAL:" + nonce;

            auto r1 = do_round("Round 2: Challenge", to_bytes(challenge),
                               tx_bridge, rx_bridge, lp);
            if (!r1.decode.decoded || !r1.decode.crc_valid || !r1.match) {
                std::fprintf(stderr, "\nRound 2 (challenge) FAILED.\n");
                exit_code = 1;
            } else {
                std::string rx1_str = to_string(r1.rx_bytes);
                std::string decoded_nonce;
                if (rx1_str.size() > 5 && rx1_str.substr(0, 5) == "CHAL:") {
                    decoded_nonce = rx1_str.substr(5);
                } else {
                    decoded_nonce = rx1_str;
                }
                std::string response = "RESP:" + decoded_nonce;

                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                auto r2 = do_round("Round 3: Response", to_bytes(response),
                                   tx_bridge, rx_bridge, lp);
                if (!r2.decode.decoded || !r2.decode.crc_valid || !r2.match) {
                    std::fprintf(stderr, "\nRound 3 (response) FAILED.\n");
                    exit_code = 1;
                }

                std::string r1_tx_str = to_string(r1.tx_bytes);
                std::string r1_rx_str = to_string(r1.rx_bytes);
                std::string r2_tx_str = to_string(r2.tx_bytes);
                std::string r2_rx_str = to_string(r2.rx_bytes);
                bool nonce_in_response = (r2_rx_str.find(nonce) !=
                                          std::string::npos);

                std::fprintf(stderr, "\n=============================\n");
                std::fprintf(stderr, "  LOOPBACK TEST RESULT\n");
                std::fprintf(stderr, "=============================\n");
                std::fprintf(stderr, "  Greeting:    \"%s\"  CRC=%s\n",
                             greeting.c_str(),
                             r0.decode.crc_valid ? "OK" : "FAIL");
                std::fprintf(stderr, "  Nonce:       %s\n", nonce.c_str());
                std::fprintf(stderr, "  R2 TX:       \"%s\"\n", r1_tx_str.c_str());
                std::fprintf(stderr, "  R2 RX:       \"%s\"  CRC=%s\n",
                             r1_rx_str.c_str(),
                             r1.decode.crc_valid ? "OK" : "FAIL");
                std::fprintf(stderr, "  R3 TX:       \"%s\"\n", r2_tx_str.c_str());
                std::fprintf(stderr, "  R3 RX:       \"%s\"  CRC=%s\n",
                             r2_rx_str.c_str(),
                             r2.decode.crc_valid ? "OK" : "FAIL");
                std::fprintf(stderr, "  Nonce echo:  %s\n",
                             nonce_in_response ? "YES" : "NO");

                if (r0.match && r0.decode.crc_valid &&
                    r1.match && r1.decode.crc_valid &&
                    r2.match && r2.decode.crc_valid &&
                    nonce_in_response) {
                    std::fprintf(stderr,
                        "\n  *** LOOPBACK TEST PASSED ***\n\n");
                    exit_code = 0;
                } else {
                    std::fprintf(stderr,
                        "\n  *** LOOPBACK TEST FAILED ***\n\n");
                    exit_code = 1;
                }
            }
        }
    } else {
        std::vector<uint8_t> payload_bytes(cfg.payload.begin(),
                                            cfg.payload.end());
        auto r = do_round("Single Round", payload_bytes,
                          tx_bridge, rx_bridge, lp, is_hex);

        std::fprintf(stderr, "\n=== LOOPBACK RESULT ===\n");
        std::fprintf(stderr, "  Bytes:  %zu TX, %zu RX\n",
                     r.tx_bytes.size(), r.rx_bytes.size());
        std::fprintf(stderr, "  CRC:    %s\n",
                     r.decode.crc_valid ? "VALID" : "FAIL");
        std::fprintf(stderr, "  Match:  %s\n",
                     r.match ? "YES" : "NO");

        if (r.match && r.decode.crc_valid) {
            std::fprintf(stderr,
                "\n  *** LOOPBACK TEST PASSED ***\n\n");
        } else {
            std::fprintf(stderr,
                "\n  *** LOOPBACK TEST FAILED ***\n\n");
            exit_code = 1;
        }

        // Output CBOR frame on stdout (like lora_rx --cbor) for piping
        // to decoder scripts.
        if (cfg.loopback_cbor && r.decode.decoded) {
            std::vector<uint8_t> cbor_buf;
            // 8 key-value pairs
            gr::lora::cbor::encode_map_begin(cbor_buf, 8);
            gr::lora::cbor::kv_text(cbor_buf, "type", "lora_frame");
            gr::lora::cbor::kv_bytes(cbor_buf, "payload",
                                      r.rx_bytes.data(), r.rx_bytes.size());
            gr::lora::cbor::kv_uint(cbor_buf, "sf", cfg.sf);
            gr::lora::cbor::kv_uint(cbor_buf, "cr", cfg.cr);
            gr::lora::cbor::kv_bool(cbor_buf, "crc_valid",
                                     r.decode.crc_valid);
            gr::lora::cbor::kv_uint(cbor_buf, "sync_word", cfg.sync_word);
            gr::lora::cbor::kv_text(cbor_buf, "protocol",
                                     "meshcore_or_reticulum");
            gr::lora::cbor::kv_uint(cbor_buf, "seq", 1);

            std::fwrite(cbor_buf.data(), 1, cbor_buf.size(), stdout);
            std::fflush(stdout);
        }
    }

    radio_bridge_destroy_ex(tx_bridge, 1);
    radio_bridge_destroy_ex(rx_bridge, 1);

    _exit(exit_code);
}

}  // namespace

int main(int argc, char** argv) {
    TxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    // ---- Stdin CBOR mode ----
    if (cfg.stdin_mode) {
        return run_stdin_mode(cfg);
    }

    // ---- Loopback mode ----
    if (cfg.loopback) {
        return run_loopback(cfg);
    }

    // ---- CLI mode ----
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

    std::vector<uint8_t> payload_bytes(cfg.payload.begin(), cfg.payload.end());
    auto iq = generate_iq(payload_bytes, cfg);
    double airtime_sec = static_cast<double>(iq.size()) /
                         static_cast<double>(cfg.rate);

    std::fprintf(stderr, "=== LoRa TX ===\n");
    std::fprintf(stderr, "  Device args: %s\n", cfg.args.empty() ? "(auto)" : cfg.args.c_str());
    std::fprintf(stderr, "  Clock:       %s\n", cfg.clock.empty() ? "(default)" : cfg.clock.c_str());
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

    radio_bridge_t* bridge = open_tx_device(cfg);
    if (!bridge) return 1;

    for (int tx_num = 0; tx_num < cfg.repeat; tx_num++) {
        if (cfg.repeat > 1) {
            std::fprintf(stderr, "TX %d/%d...\n", tx_num + 1, cfg.repeat);
        }
        if (transmit_iq(bridge, iq) < 0) {
            radio_bridge_destroy(bridge);
            return 1;
        }
        std::fprintf(stderr, "  Sent %zu samples (%.3f ms)\n",
                     iq.size(), airtime_sec * 1000.0);
        if (tx_num + 1 < cfg.repeat && cfg.gap_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(cfg.gap_ms));
        }
    }

    radio_bridge_destroy_ex(bridge, 1);
    std::fprintf(stderr, "\nDone.\n");
    _exit(0);
}
