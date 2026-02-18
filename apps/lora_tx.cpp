// SPDX-License-Identifier: ISC
//
// lora_tx: Hardware LoRa transmitter.
//
// Uses the unified radio_bridge API (UHD or SoapySDR backend, selected at
// link time). Generates a complete LoRa frame from a payload string using
// the shared algorithm functions (no GR4 graph needed for batch TX), then
// writes the IQ to the device via the pure-C bridge.
//
// Two modes:
//   CLI mode:   lora_tx [options] <payload_string>
//   Stdin mode: lora_tx --stdin [--dry-run]
//               Reads concatenated CBOR TX request objects from stdin.
//               See docs/cbor-schemas.md for the schema.
//
//   --args <str>      Device args (default: auto-discover)
//   --clock <src>     Clock source (default: device default)
//   --freq <hz>       TX center frequency (default: 869618000)
//   --gain <db>       TX gain in dB (default: 30)
//   --rate <sps>      Sample rate in S/s (default: 250000)
//   --repeat <n>      Number of times to transmit (default: 1)
//   --gap <ms>        Gap between repeats in milliseconds (default: 1000)
//   --dry-run         Generate IQ but do not transmit (dump stats to stderr)
//   --stdin           Read CBOR TX requests from stdin (streaming)
//   -h, --help        Show usage
//
// LoRa config: SF=8, BW=62500, CR=4/8, sync=0x12, explicit header, CRC on
// (defaults match common mesh network configurations)
//
// *** SAFETY: This program transmits on radio frequencies. ***
// *** Ensure you have authorization to transmit on the configured frequency. ***
// *** The --dry-run flag generates IQ without transmitting. ***

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

#include "cbor.hpp"
#include "radio_bridge.h"

namespace {

// ---- CLI argument parsing ----

struct TxConfig {
    std::string args{};              ///< device args (empty = auto-discover)
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
    std::string payload;
};

void print_usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options] <payload_string>\n\n"
        "Device options:\n"
        "  --args <str>      Device args (default: auto-discover)\n"
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
        "  --stdin           Read CBOR TX requests from stdin\n"
        "  -h, --help        Show this help\n\n"
        "Examples:\n"
        "  %s \"Hello World\"\n"
        "  %s --args type=b200 --clock external \"Hello World\"\n"
        "  %s --sf 12 --bw 125000 --rate 500000 \"Hello World\"\n\n"
        "*** Ensure you have authorization to transmit! ***\n",
        prog, prog, prog, prog);
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

/// Peek at the first byte of a CBOR item to determine total map size.
/// Returns the number of bytes needed (0 on error/EOF).
/// CBOR maps are self-delimiting: we read the head, determine N pairs,
/// then read each item. To simplify, we buffer up to 64 KB and decode.
std::vector<uint8_t> read_cbor_object(int fd) {
    // Read into an accumulation buffer. Start with the head byte.
    std::vector<uint8_t> buf;
    buf.resize(1);
    if (!read_exact(fd, buf.data(), 1)) return {};

    uint8_t head = buf[0];
    uint8_t major = static_cast<uint8_t>(head >> 5);
    if (major != 5) {
        std::fprintf(stderr, "CBOR: expected map (major 5), got %u\n", major);
        return {};
    }

    // We need to read the complete CBOR map. Strategy: accumulate bytes
    // and attempt to decode. Since our messages are small (< 4 KB),
    // read in chunks until decode succeeds.
    buf.reserve(512);

    // Read additional argument bytes for the map count
    uint8_t info = static_cast<uint8_t>(head & 0x1F);
    std::size_t extra = 0;
    if (info >= 24 && info <= 27) {
        extra = std::size_t{1} << (info - 24);  // 1, 2, 4, 8
        buf.resize(1 + extra);
        if (!read_exact(fd, buf.data() + 1, extra)) return {};
    }

    // Now we know the pair count; read items incrementally.
    // Each item starts with a head byte; we can compute its length.
    // For simplicity, read byte-by-byte attempting decode after each chunk.
    // Since max payload is 255 + overhead < 1 KB, this is fast.
    constexpr std::size_t kMaxSize = 65536;
    constexpr std::size_t kChunkSize = 256;

    while (buf.size() < kMaxSize) {
        try {
            gr::lora::cbor::decode_map(std::span<const uint8_t>(buf));
            return buf;  // success
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
        if (raw.empty()) break;  // EOF or error

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

            // Apply per-request overrides
            TxConfig req_cfg = cfg;  // start from CLI defaults
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

    // Stop stream and release streamer, but skip device deallocation —
    // static teardown can crash under dual-libc++.
    radio_bridge_destroy_ex(bridge, 1);
    std::fprintf(stderr, "\nDone.\n");
    _exit(0);
}
