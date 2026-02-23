// SPDX-License-Identifier: ISC
//
// lora_tx: LoRa transmitter using native GR4 SoapySDR blocks.
//
// Graph: TagSource<cf32> -> SoapySinkBlock<cf32>
//
// CLI mode:   ./lora_tx [options] <payload>
// Stdin mode: meshcore_tx.py send ... | ./lora_tx [options]
//
// When no positional argument is given, reads CBOR TX requests from stdin.
// See docs/cbor-schemas.md for the CBOR schema.

#include <unistd.h>

#include <cinttypes>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/soapy/SoapySink.hpp>

#include "cbor.hpp"

namespace {

struct TxConfig {
    std::string device{"uhd"};
    std::string device_param{"type=b200"};
    double      freq{869'618'000.0};
    double      gain{75.0};
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

void print_usage() {
    std::fprintf(stderr,
        "Usage: ./lora_tx [options] [payload]\n\n"
        "Transmit a LoRa frame. If no payload argument is given, reads\n"
        "CBOR TX requests from stdin (see docs/cbor-schemas.md).\n\n"
        "Device:\n"
        "  --device <name>       SoapySDR driver (default: uhd)\n"
        "  --device-param <str>  Device params, key=val,... (default: type=b200)\n"
        "  --freq <hz>           TX frequency (default: 869618000)\n"
        "  --gain <db>           TX gain (default: 75)\n"
        "  --rate <sps>          Sample rate (default: 250000)\n\n"
        "LoRa PHY:\n"
        "  --bw <hz>             Bandwidth (default: 62500)\n"
        "  --sf <7-12>           Spreading factor (default: 8)\n"
        "  --cr <1-4>            Coding rate (default: 4)\n"
        "  --sync <hex>          Sync word (default: 0x12)\n"
        "  --preamble <n>        Preamble length (default: 8)\n\n"
        "TX:\n"
        "  --repeat <n>          Transmit count (default: 1)\n"
        "  --gap <ms>            Gap between repeats (default: 1000)\n"
        "  --dry-run             Generate IQ without transmitting\n\n"
        "  -h, --help            Show this help\n\n"
        "Examples:\n"
        "  ./lora_tx \"Hello World\"\n"
        "  ./lora_tx --repeat 5 --gap 2000 \"Test\"\n"
        "  meshcore_tx.py send ... | ./lora_tx\n"
        "  ./lora_tx --dry-run < requests.cbor\n");
}

bool parse_args(int argc, char** argv, TxConfig& cfg) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return false;
        }
        if (arg == "--device" && i + 1 < argc) { cfg.device = argv[++i]; continue; }
        if (arg == "--device-param" && i + 1 < argc) { cfg.device_param = argv[++i]; continue; }
        if (arg == "--freq" && i + 1 < argc) { cfg.freq = std::stod(argv[++i]); continue; }
        if (arg == "--gain" && i + 1 < argc) { cfg.gain = std::stod(argv[++i]); continue; }
        if (arg == "--rate" && i + 1 < argc) { cfg.rate = std::stof(argv[++i]); continue; }
        if (arg == "--bw" && i + 1 < argc) { cfg.bw = static_cast<uint32_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sf" && i + 1 < argc) { cfg.sf = static_cast<uint8_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--cr" && i + 1 < argc) { cfg.cr = static_cast<uint8_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sync" && i + 1 < argc) { cfg.sync_word = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); continue; }
        if (arg == "--preamble" && i + 1 < argc) { cfg.preamble_len = static_cast<uint16_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--repeat" && i + 1 < argc) { cfg.repeat = std::stoi(argv[++i]); continue; }
        if (arg == "--gap" && i + 1 < argc) { cfg.gap_ms = std::stoi(argv[++i]); continue; }
        if (arg == "--dry-run") { cfg.dry_run = true; continue; }
        if (arg[0] == '-') {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            print_usage();
            return false;
        }
        // Positional: payload (rest of args joined by spaces)
        cfg.payload = arg;
        for (int j = i + 1; j < argc; j++) {
            cfg.payload += " ";
            cfg.payload += argv[j];
        }
        break;
    }
    return true;
}

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

std::vector<std::complex<float>> build_tx_buffer(
        const std::vector<std::complex<float>>& iq,
        int repeat, int gap_ms, float rate) {
    if (repeat <= 1) return iq;

    std::vector<std::complex<float>> buf;
    auto gap_samples = static_cast<uint32_t>(
        rate * static_cast<float>(gap_ms) / 1000.0f);
    for (int r = 0; r < repeat; r++) {
        buf.insert(buf.end(), iq.begin(), iq.end());
        if (r + 1 < repeat && gap_samples > 0) {
            buf.resize(buf.size() + gap_samples,
                       std::complex<float>(0.f, 0.f));
        }
    }
    return buf;
}

int transmit_graph(const std::vector<std::complex<float>>& iq,
                   const TxConfig& cfg) {
    gr::Graph graph;
    using cf32 = std::complex<float>;

    auto& source = graph.emplaceBlock<gr::testing::TagSource<cf32,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    source.values = iq;

    auto& sink = graph.emplaceBlock<gr::blocks::soapy::SoapySinkBlock<cf32, 1UZ>>({
        {"device", cfg.device},
        {"device_parameter", cfg.device_param},
        {"sample_rate", cfg.rate},
        {"tx_center_frequency", gr::Tensor<double>{cfg.freq}},
        {"tx_gains", gr::Tensor<double>{cfg.gain}},
        {"timed_tx", true},
        {"wait_burst_ack", true},
        {"max_underflow_count", gr::Size_t{0}},
    });

    if (graph.connect<"out">(source).to<"in">(sink) != gr::ConnectionResult::SUCCESS) {
        std::fprintf(stderr, "ERROR: failed to connect source -> sink\n");
        return -1;
    }

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "ERROR: Scheduler init failed\n");
        return -1;
    }

    const auto ret = sched.runAndWait();
    if (!ret.has_value()) {
        std::fprintf(stderr, "Scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
        return -1;
    }

    return 0;
}

void log_tx_info(const std::vector<uint8_t>& payload,
                 const TxConfig& cfg,
                 std::size_t iq_len, double airtime_sec) {
    std::fprintf(stderr, "  Payload:     %zu bytes\n", payload.size());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  Gain:        %.0f dB\n", cfg.gain);
    std::fprintf(stderr, "  SF=%u  BW=%u  CR=4/%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, 4 + cfg.cr, cfg.sync_word, cfg.preamble_len);
    std::fprintf(stderr, "  IQ samples:  %zu (%.3f ms airtime)\n",
                 iq_len, airtime_sec * 1000.0);
}

// --- Stdin CBOR mode ---

bool read_exact(int fd, uint8_t* buf, std::size_t n) {
    std::size_t total = 0;
    while (total < n) {
        auto r = ::read(fd, buf + total, n - total);
        if (r <= 0) return false;
        total += static_cast<std::size_t>(r);
    }
    return true;
}

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
            // incomplete — read more
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
    std::fprintf(stderr, "=== LoRa TX stdin mode (CBOR) ===\n");
    std::fprintf(stderr, "  Device:  %s\n", cfg.device.c_str());
    std::fprintf(stderr, "  Params:  %s\n", cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());
    std::fprintf(stderr, "  Mode:    %s\n", cfg.dry_run ? "DRY RUN" : "TRANSMIT");
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
            auto tx_buf = build_tx_buffer(iq, req_cfg.repeat, req_cfg.gap_ms, req_cfg.rate);
            double airtime = static_cast<double>(tx_buf.size()) /
                             static_cast<double>(req_cfg.rate);

            std::fprintf(stderr, "#%" PRIu64 ": ", seq);
            log_tx_info(payload, req_cfg, tx_buf.size(), airtime);

            bool ok = true;
            if (!cfg.dry_run) {
                if (transmit_graph(tx_buf, req_cfg) < 0) {
                    ok = false;
                }
            }
            emit_ack(seq, ok);
        } catch (const gr::lora::cbor::DecodeError& e) {
            std::fprintf(stderr, "#%" PRIu64 ": CBOR decode error: %s\n",
                         seq, e.what());
            emit_ack(seq, false);
        }
    }

    std::fprintf(stderr, "\nStdin closed. %" PRIu64 " requests processed.\n", seq);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    TxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    // Auto-detect mode: positional payload -> CLI mode, otherwise stdin
    if (cfg.payload.empty()) {
        return run_stdin_mode(cfg);
    }

    if (cfg.payload.size() > 255) {
        std::fprintf(stderr, "Error: payload too long (%zu bytes, max 255)\n",
                     cfg.payload.size());
        return 1;
    }

    std::vector<uint8_t> payload_bytes(cfg.payload.begin(), cfg.payload.end());
    auto iq = generate_iq(payload_bytes, cfg);
    auto tx_buf = build_tx_buffer(iq, cfg.repeat, cfg.gap_ms, cfg.rate);
    double airtime_sec = static_cast<double>(tx_buf.size()) /
                         static_cast<double>(cfg.rate);

    std::fprintf(stderr, "=== LoRa TX (Soapy) ===\n");
    std::fprintf(stderr, "  Device:      %s\n", cfg.device.c_str());
    std::fprintf(stderr, "  Params:      %s\n", cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());
    log_tx_info(payload_bytes, cfg, tx_buf.size(), airtime_sec);
    std::fprintf(stderr, "  Sample rate: %.0f S/s\n",
                 static_cast<double>(cfg.rate));
    std::fprintf(stderr, "  Repeat:      %d  gap=%d ms\n",
                 cfg.repeat, cfg.gap_ms);
    std::fprintf(stderr, "  Mode:        %s\n",
                 cfg.dry_run ? "DRY RUN (no transmission)" : "TRANSMIT");
    std::fprintf(stderr, "\n");

    if (cfg.dry_run) {
        std::fprintf(stderr, "Dry run complete. IQ frame generated but not sent.\n");
        return 0;
    }

    if (transmit_graph(tx_buf, cfg) < 0) {
        return 1;
    }

    std::fprintf(stderr, "TX complete. %zu samples (%.1f ms)\n",
                 tx_buf.size(), airtime_sec * 1000.0);
    _exit(0);
}
