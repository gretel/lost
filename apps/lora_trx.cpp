// SPDX-License-Identifier: ISC
//
// lora_trx: full-duplex LoRa transceiver using native GR4 SoapySDR blocks.
//
// RX: SoapySource -> FrameSync -> DemodDecoder -> FrameSink
// TX: TagSource<cf32> -> SoapySinkBlock<cf32> (ephemeral graph per request)
//
// Supports dual-RX (--rx-channels 0,1) for simultaneous decode on two
// antennas via SoapyDualSimpleSource feeding two parallel decode chains.
// TX always uses a single channel (--tx-channel, default 0).
//
// All I/O is CBOR over UDP. No stdout, no stdin. 
// RX frames are fanned out to all connected clients.
// TX requests (type: "lora_tx") are processed and acked.

#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/FrameSync.hpp>
#include <gnuradio-4.0/lora/DemodDecoder.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>
#include <gnuradio-4.0/soapy/SoapySink.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include "FrameSink.hpp"
#include "cbor.hpp"

namespace {

volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void signal_handler(int /*sig*/) { g_running = 0; }

struct TrxConfig {
    std::string          device{"uhd"};
    std::string          device_param{"type=b200"};
    double               freq{869'618'000.0};
    double               gain_rx{30.0};
    double               gain_tx{75.0};
    float                rate{250'000.f};
    uint32_t             bw{62'500};
    uint8_t              sf{8};
    uint8_t              cr{4};
    uint16_t             sync{0x12};
    uint16_t             preamble{8};
    std::string          clock{};
    std::vector<uint32_t> rx_channels{0};
    uint32_t             tx_channel{0};
    std::string          listen{"127.0.0.1"};
    uint16_t             port{5555};
    bool                 debug{false};
};

// --- UDP state (owned by main thread) ---

struct UdpState {
    int                                  fd{-1};
    std::vector<struct sockaddr_storage> clients;
    std::mutex                           mutex;

    void broadcast(const std::vector<uint8_t>& buf) {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& client : clients) {
            auto len = (client.ss_family == AF_INET6)
                ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
            ::sendto(fd, buf.data(), buf.size(), 0,
                     reinterpret_cast<const struct sockaddr*>(&client),
                     static_cast<socklen_t>(len));
        }
    }

    static bool sockaddr_equal(const struct sockaddr_storage& a,
                               const struct sockaddr_storage& b) {
        if (a.ss_family != b.ss_family) return false;
        if (a.ss_family == AF_INET6) {
            const auto& a6 = reinterpret_cast<const struct sockaddr_in6&>(a);
            const auto& b6 = reinterpret_cast<const struct sockaddr_in6&>(b);
            return a6.sin6_port == b6.sin6_port &&
                   std::memcmp(&a6.sin6_addr, &b6.sin6_addr, 16) == 0;
        }
        const auto& a4 = reinterpret_cast<const struct sockaddr_in&>(a);
        const auto& b4 = reinterpret_cast<const struct sockaddr_in&>(b);
        return a4.sin_port == b4.sin_port &&
               a4.sin_addr.s_addr == b4.sin_addr.s_addr;
    }

    void registerSender(const struct sockaddr_storage& sender) {
        std::lock_guard<std::mutex> lock(mutex);
        if (std::ranges::any_of(clients, [&](const auto& c) {
            return sockaddr_equal(c, sender);
        })) return;
        clients.push_back(sender);
        char addr_str[INET6_ADDRSTRLEN]{};
        uint16_t port = 0;
        if (sender.ss_family == AF_INET6) {
            const auto& s6 = reinterpret_cast<const struct sockaddr_in6&>(sender);
            ::inet_ntop(AF_INET6, &s6.sin6_addr, addr_str, sizeof(addr_str));
            port = ntohs(s6.sin6_port);
        } else {
            const auto& s4 = reinterpret_cast<const struct sockaddr_in&>(sender);
            ::inet_ntop(AF_INET, &s4.sin_addr, addr_str, sizeof(addr_str));
            port = ntohs(s4.sin_port);
        }
        std::fprintf(stderr, "  UDP: registered client %s:%u (%zu total)\n",
                     addr_str, port, clients.size());
    }

    void sendTo(const std::vector<uint8_t>& buf, const struct sockaddr_storage& dest) {
        auto len = (dest.ss_family == AF_INET6)
            ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
        ::sendto(fd, buf.data(), buf.size(), 0,
                 reinterpret_cast<const struct sockaddr*>(&dest),
                 static_cast<socklen_t>(len));
    }
};

// --- CLI ---

void print_usage() {
    std::fprintf(stderr,
        "Usage: lora_trx [options]\n\n"
        "Full-duplex LoRa transceiver. All I/O is CBOR over UDP.\n"
        "TX requests use type \"lora_tx\" (see docs/cbor-schemas.md).\n\n"
        "Device:\n"
        "  --device <name>       SoapySDR driver (default: uhd)\n"
        "  --device-param <str>  Device params, key=val,... (default: type=b200)\n"
        "  --freq <hz>           Center frequency (default: 869618000)\n"
        "  --gain-rx <db>        RX gain (default: 30)\n"
        "  --gain-tx <db>        TX gain (default: 75)\n"
        "  --rate <sps>          Sample rate (default: 250000)\n"
        "  --clock <source>      Clock source: internal, external, gpsdo\n\n"
        "LoRa PHY:\n"
        "  --bw <hz>             Bandwidth (default: 62500)\n"
        "  --sf <7-12>           Spreading factor (default: 8)\n"
        "  --cr <1-4>            Coding rate (default: 4)\n"
        "  --sync <hex>          Sync word (default: 0x12)\n"
        "  --preamble <n>        Preamble length (default: 8)\n\n"
        "Channels:\n"
        "  --rx-channels <list>  RX channel(s), comma-separated (default: 0)\n"
        "  --tx-channel <n>      TX channel (default: 0)\n\n"
        "Network:\n"
        "  --listen <addr>       UDP bind address (default: 127.0.0.1)\n"
        "  --port <port>         UDP port (default: 5555)\n\n"
        "Debug:\n"
        "  --debug               Enable verbose state-machine traces on stderr\n\n"
        "  -h, --help            Show this help\n\n"
        "Examples:\n"
        "  lora_trx\n"
        "  lora_trx --listen 0.0.0.0 --port 9000\n"
        "  lora_trx --rx-channels 0,1 --tx-channel 0\n"
        "  lora_trx --rx-channels 1 --tx-channel 0\n");
}

std::vector<uint32_t> parse_channel_list(const char* str) {
    std::vector<uint32_t> result;
    std::string s(str);
    std::size_t pos = 0;
    while (pos < s.size()) {
        auto comma = s.find(',', pos);
        auto token = s.substr(pos, comma == std::string::npos ? comma : comma - pos);
        result.push_back(static_cast<uint32_t>(std::stoul(token)));
        pos = comma == std::string::npos ? s.size() : comma + 1;
    }
    return result;
}

bool parse_args(int argc, char* argv[], TrxConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") { print_usage(); return false; }
        if (arg == "--device" && i + 1 < argc) { cfg.device = argv[++i]; continue; }
        if (arg == "--device-param" && i + 1 < argc) { cfg.device_param = argv[++i]; continue; }
        if (arg == "--freq" && i + 1 < argc) { cfg.freq = std::stod(argv[++i]); continue; }
        if (arg == "--gain-rx" && i + 1 < argc) { cfg.gain_rx = std::stod(argv[++i]); continue; }
        if (arg == "--gain-tx" && i + 1 < argc) { cfg.gain_tx = std::stod(argv[++i]); continue; }
        if (arg == "--rate" && i + 1 < argc) { cfg.rate = std::stof(argv[++i]); continue; }
        if (arg == "--bw" && i + 1 < argc) { cfg.bw = static_cast<uint32_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sf" && i + 1 < argc) { cfg.sf = static_cast<uint8_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--cr" && i + 1 < argc) { cfg.cr = static_cast<uint8_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--sync" && i + 1 < argc) { cfg.sync = static_cast<uint16_t>(std::stoul(argv[++i], nullptr, 0)); continue; }
        if (arg == "--preamble" && i + 1 < argc) { cfg.preamble = static_cast<uint16_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--clock" && i + 1 < argc) { cfg.clock = argv[++i]; continue; }
        if (arg == "--rx-channels" && i + 1 < argc) { cfg.rx_channels = parse_channel_list(argv[++i]); continue; }
        if (arg == "--tx-channel" && i + 1 < argc) { cfg.tx_channel = static_cast<uint32_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--listen" && i + 1 < argc) { cfg.listen = argv[++i]; continue; }
        if (arg == "--port" && i + 1 < argc) { cfg.port = static_cast<uint16_t>(std::stoul(argv[++i])); continue; }
        if (arg == "--debug") { cfg.debug = true; continue; }
        std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
        print_usage();
        return false;
    }
    return true;
}

// --- TX (ephemeral graph, runs in main thread) ---

std::vector<std::complex<float>> generate_iq(
        const std::vector<uint8_t>& payload, const TrxConfig& cfg,
        uint8_t cr_override = 0, uint16_t sync_override = 0,
        uint16_t preamble_override = 0) {
    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));
    uint32_t sps = (1u << cfg.sf) * os;
    uint8_t cr = cr_override ? cr_override : cfg.cr;
    uint16_t sync = sync_override ? sync_override : cfg.sync;
    uint16_t pre = preamble_override ? preamble_override : cfg.preamble;
    return gr::lora::generate_frame_iq(payload, cfg.sf, cr, os, sync, pre,
                                       true, sps * 2, 2, cfg.bw);
}

int transmit_1ch(const std::vector<std::complex<float>>& iq, const TrxConfig& cfg) {
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
        {"tx_gains", gr::Tensor<double>{cfg.gain_tx}},
        {"tx_channels", gr::Tensor<gr::Size_t>(gr::data_from, {static_cast<gr::Size_t>(cfg.tx_channel)})},
        {"clock_source", cfg.clock},
        {"timed_tx", true},
        {"wait_burst_ack", true},
        {"max_underflow_count", gr::Size_t{0}},
    });

    if (graph.connect<"out">(source).to<"in">(sink) != gr::ConnectionResult::SUCCESS) {
        std::fprintf(stderr, "TX: failed to connect source -> sink\n");
        return -1;
    }

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    sched.timeout_inactivity_count = 1'000'000U;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "TX: scheduler init failed\n");
        return -1;
    }

    auto ret = sched.runAndWait();
    if (!ret.has_value()) {
        std::fprintf(stderr, "TX: scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
        return -1;
    }
    return 0;
}

// 2RX+2TX mode: B210 requires balanced channel enables (2+2=4, passes update_enables).
// Real IQ goes to TX channel cfg.tx_channel, zeros go to the other channel.
int transmit_2ch(const std::vector<std::complex<float>>& iq, const TrxConfig& cfg) {
    gr::Graph graph;
    using cf32 = std::complex<float>;

    auto& source = graph.emplaceBlock<gr::testing::TagSource<cf32,
        gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    source.values = iq;

    auto& null_source = graph.emplaceBlock<gr::testing::ConstantSource<cf32>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
    });

    auto& sink = graph.emplaceBlock<gr::blocks::soapy::SoapySinkBlock<cf32, 2UZ>>({
        {"device", cfg.device},
        {"device_parameter", cfg.device_param},
        {"sample_rate", cfg.rate},
        {"tx_center_frequency", gr::Tensor<double>(gr::data_from, {cfg.freq, cfg.freq})},
        {"tx_gains", gr::Tensor<double>(gr::data_from, {cfg.gain_tx, 0.0})},
        {"tx_channels", gr::Tensor<gr::Size_t>(gr::data_from, {gr::Size_t{0}, gr::Size_t{1}})},
        {"clock_source", cfg.clock},
        {"timed_tx", true},
        {"wait_burst_ack", true},
        {"max_underflow_count", gr::Size_t{0}},
    });

    // Connect real IQ to the active TX port, zeros to the dummy port.
    auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
    if (cfg.tx_channel == 0) {
        if (!ok(graph.connect<"out">(source).to<"in", 0>(sink)) ||
            !ok(graph.connect<"out">(null_source).to<"in", 1>(sink))) {
            std::fprintf(stderr, "TX: failed to connect 2-ch graph\n");
            return -1;
        }
    } else {
        if (!ok(graph.connect<"out">(null_source).to<"in", 0>(sink)) ||
            !ok(graph.connect<"out">(source).to<"in", 1>(sink))) {
            std::fprintf(stderr, "TX: failed to connect 2-ch graph\n");
            return -1;
        }
    }

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    sched.timeout_inactivity_count = 1'000'000U;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "TX: scheduler init failed\n");
        return -1;
    }

    auto ret = sched.runAndWait();
    if (!ret.has_value()) {
        std::fprintf(stderr, "TX: scheduler error: %s\n",
                     std::format("{}", ret.error()).c_str());
        return -1;
    }
    return 0;
}

int transmit(const std::vector<std::complex<float>>& iq, const TrxConfig& cfg) {
    if (cfg.rx_channels.size() > 1) {
        return transmit_2ch(iq, cfg);
    }
    return transmit_1ch(iq, cfg);
}

void handle_tx_request(const gr::lora::cbor::Map& msg, const TrxConfig& cfg,
                       UdpState& udp, const struct sockaddr_storage& sender) {
    using gr::lora::cbor::get_uint_or;
    using gr::lora::cbor::get_bool_or;

    auto payload = gr::lora::cbor::get_bytes(msg, "payload");
    if (payload.empty() || payload.size() > 255) {
        std::fprintf(stderr, "TX: invalid payload size %zu\n", payload.size());
        return;
    }

    auto cr = static_cast<uint8_t>(get_uint_or(msg, "cr", cfg.cr));
    auto sync = static_cast<uint16_t>(get_uint_or(msg, "sync_word", cfg.sync));
    auto pre = static_cast<uint16_t>(get_uint_or(msg, "preamble_len", cfg.preamble));
    auto repeat = static_cast<int>(get_uint_or(msg, "repeat", 1));
    auto gap_ms = static_cast<int>(get_uint_or(msg, "gap_ms", 1000));
    bool dry_run = get_bool_or(msg, "dry_run", false);

    auto iq = generate_iq(payload, cfg, cr, sync, pre);

    // repeat + gap
    if (repeat > 1) {
        std::vector<std::complex<float>> buf;
        auto gap_samples = static_cast<uint32_t>(
            cfg.rate * static_cast<float>(gap_ms) / 1000.0f);
        for (int r = 0; r < repeat; r++) {
            buf.insert(buf.end(), iq.begin(), iq.end());
            if (r + 1 < repeat && gap_samples > 0) {
                buf.resize(buf.size() + gap_samples, std::complex<float>(0.f, 0.f));
            }
        }
        iq = std::move(buf);
    }

    double airtime = static_cast<double>(iq.size()) / static_cast<double>(cfg.rate);
    std::fprintf(stderr, "TX: %zu bytes, SF%u CR4/%u sync=0x%02X repeat=%d "
                 "%.1f ms airtime%s\n",
                 payload.size(), cfg.sf, 4u + cr, sync, repeat,
                 airtime * 1000.0, dry_run ? " (dry run)" : "");

    bool ok = true;
    if (!dry_run) {
        ok = (transmit(iq, cfg) == 0);
    }

    // send ack back to the requesting client
    uint64_t seq = get_uint_or(msg, "seq", 0);
    std::vector<uint8_t> ack;
    ack.reserve(64);
    gr::lora::cbor::encode_map_begin(ack, 3);
    gr::lora::cbor::kv_text(ack, "type", "lora_tx_ack");
    gr::lora::cbor::kv_uint(ack, "seq", seq);
    gr::lora::cbor::kv_bool(ack, "ok", ok);
    udp.sendTo(ack, sender);
}

// --- RX graph builders ---

// RX graph: one SoapySimpleSource per channel, each with its own decode chain.
// Each source opens the device independently — SoapyUHD's weak_ptr registry shares
// the underlying B210 hardware. Using separate single-channel streams avoids the
// SoapySDR MIMO stream activation bug (STREAM_ERROR on dual-channel setupStream).
void build_rx_graph_multi(gr::Graph& graph, const TrxConfig& cfg,
                          std::function<void(std::vector<uint8_t>)> callback) {
    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    for (auto ch : cfg.rx_channels) {
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapySimpleSource<std::complex<float>>>({
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
            {"rx_gains", gr::Tensor<double>{cfg.gain_rx}},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, {static_cast<gr::Size_t>(ch)})},
            {"clock_source", cfg.clock},
            {"max_chunck_size", static_cast<uint32_t>(512U << 4U)},
            {"max_overflow_count", gr::Size_t{0}},
        });

        auto& sync = graph.emplaceBlock<gr::lora::FrameSync>({
            {"center_freq", static_cast<uint32_t>(cfg.freq)},
            {"bandwidth", cfg.bw},
            {"sf", cfg.sf},
            {"sync_word", cfg.sync},
            {"os_factor", os},
            {"preamble_len", cfg.preamble},
            {"rx_channel", static_cast<int32_t>(ch)},
            {"debug", cfg.debug},
        });

        auto& demod = graph.emplaceBlock<gr::lora::DemodDecoder>({
            {"sf", cfg.sf},
            {"bandwidth", cfg.bw},
            {"debug", cfg.debug},
        });

        auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
            {"sync_word", cfg.sync},
            {"phy_sf", cfg.sf},
            {"phy_bw", cfg.bw},
        });
        sink._frame_callback = callback;

        auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
        if (!ok(graph.connect<"out">(source).to<"in">(sync)) ||
            !ok(graph.connect<"out">(sync).to<"in">(demod)) ||
            !ok(graph.connect<"out">(demod).to<"in">(sink))) {
            std::fprintf(stderr, "ERROR: failed to connect RX chain for channel %u\n", ch);
        }
    }
}

} // namespace

int main(int argc, char* argv[]) {
    TrxConfig cfg;
    if (!parse_args(argc, argv, cfg)) {
        return 1;
    }

    if (cfg.rx_channels.empty() || cfg.rx_channels.size() > 2) {
        std::fprintf(stderr, "ERROR: --rx-channels must specify 1 or 2 channels\n");
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    std::fprintf(stderr, "=== LoRa TRX (Soapy) ===\n");
    std::fprintf(stderr, "  Device:      %s\n", cfg.device.c_str());
    std::fprintf(stderr, "  Params:      %s\n",
                 cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());
    std::fprintf(stderr, "  Frequency:   %.6f MHz\n", cfg.freq / 1e6);
    std::fprintf(stderr, "  Gain RX:     %.0f dB\n", cfg.gain_rx);
    std::fprintf(stderr, "  Gain TX:     %.0f dB\n", cfg.gain_tx);
    std::fprintf(stderr, "  Sample rate: %.0f S/s  (os_factor=%u)\n",
                 static_cast<double>(cfg.rate), os);
    std::fprintf(stderr, "  SF=%u  BW=%u  CR=4/%u  sync=0x%02X  preamble=%u\n",
                 cfg.sf, cfg.bw, 4u + cfg.cr, cfg.sync, cfg.preamble);
    std::fprintf(stderr, "  RX channel%s:", cfg.rx_channels.size() > 1 ? "s" : "");
    for (auto ch : cfg.rx_channels) std::fprintf(stderr, " %u", ch);
    std::fprintf(stderr, "\n");
    std::fprintf(stderr, "  TX channel:  %u\n", cfg.tx_channel);
    if (!cfg.clock.empty()) {
        std::fprintf(stderr, "  Clock:       %s\n", cfg.clock.c_str());
    }
    if (cfg.debug) {
        std::fprintf(stderr, "  Debug:       enabled\n");
    }
    std::fprintf(stderr, "  Listen:      %s:%u\n", cfg.listen.c_str(), cfg.port);
    std::fprintf(stderr, "\n");

    // --- Bind UDP socket (dual-stack IPv6, accepts IPv4 via mapped addresses) ---
    UdpState udp;

    // Try IPv6 first (dual-stack), fall back to IPv4 if address is explicitly v4
    struct sockaddr_in6 bind6{};
    struct sockaddr_in  bind4{};
    bool use_v6 = false;

    if (::inet_pton(AF_INET6, cfg.listen.c_str(), &bind6.sin6_addr) == 1) {
        use_v6 = true;
    } else if (cfg.listen == "0.0.0.0") {
        // Dual-stack: accept both IPv4 and IPv6 on all interfaces
        use_v6 = true;
        // sin6_addr stays IN6ADDR_ANY (from zero-init)
    } else if (::inet_pton(AF_INET, cfg.listen.c_str(), &bind4.sin_addr) == 1) {
        use_v6 = false;
    } else {
        std::fprintf(stderr, "ERROR: invalid listen address '%s'\n",
                     cfg.listen.c_str());
        return 1;
    }

    udp.fd = ::socket(use_v6 ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
    if (udp.fd < 0) {
        std::fprintf(stderr, "ERROR: socket(): %s\n", std::strerror(errno));
        return 1;
    }

    int reuse = 1;
    ::setsockopt(udp.fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (use_v6) {
        int v6only = 0;
        ::setsockopt(udp.fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));
        bind6.sin6_family = AF_INET6;
        bind6.sin6_port = htons(cfg.port);
        if (::bind(udp.fd, reinterpret_cast<struct sockaddr*>(&bind6),
                   sizeof(bind6)) < 0) {
            std::fprintf(stderr, "ERROR: bind([%s]:%u): %s\n",
                         cfg.listen.c_str(), cfg.port, std::strerror(errno));
            ::close(udp.fd);
            return 1;
        }
    } else {
        bind4.sin_family = AF_INET;
        bind4.sin_port = htons(cfg.port);
        if (::bind(udp.fd, reinterpret_cast<struct sockaddr*>(&bind4),
                   sizeof(bind4)) < 0) {
            std::fprintf(stderr, "ERROR: bind(%s:%u): %s\n",
                         cfg.listen.c_str(), cfg.port, std::strerror(errno));
            ::close(udp.fd);
            return 1;
        }
    }

    // Non-blocking for the recv loop
    int flags = ::fcntl(udp.fd, F_GETFL, 0);
    ::fcntl(udp.fd, F_SETFL, flags | O_NONBLOCK);

    std::fprintf(stderr, "  UDP server bound on %s:%u%s\n",
                 cfg.listen.c_str(), cfg.port, use_v6 ? " (IPv6 dual-stack)" : "");

    // --- Frame callback: fan out RX frames to all clients ---
    auto frame_callback = [&udp](const std::vector<uint8_t>& cbor_frame) {
        udp.broadcast(cbor_frame);
    };

    // --- Build and start RX graph in a background thread ---
    gr::Graph rx_graph;
    build_rx_graph_multi(rx_graph, cfg, frame_callback);

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> rx_sched;
    rx_sched.timeout_inactivity_count = 1'000'000U;
    if (auto ret = rx_sched.exchange(std::move(rx_graph)); !ret) {
        std::fprintf(stderr, "ERROR: RX scheduler init failed\n");
        ::close(udp.fd);
        return 1;
    }

    std::atomic<bool> rx_done{false};
    std::thread rx_thread([&rx_sched, &rx_done]() {
        auto ret = rx_sched.runAndWait();
        if (!ret.has_value()) {
            std::fprintf(stderr, "RX scheduler error: %s\n",
                         std::format("{}", ret.error()).c_str());
        }
        rx_done.store(true, std::memory_order_relaxed);
    });

    std::fprintf(stderr, "  RX running (%zu channel%s)... Ctrl+C to stop.\n\n",
                 cfg.rx_channels.size(),
                 cfg.rx_channels.size() > 1 ? "s" : "");

    // --- Main loop: recv UDP datagrams, dispatch TX requests ---
    std::vector<uint8_t> recv_buf(65536);

    while (g_running && !rx_done.load(std::memory_order_relaxed)) {
        struct sockaddr_storage sender{};
        socklen_t slen = sizeof(sender);
        auto n = ::recvfrom(udp.fd, recv_buf.data(), recv_buf.size(), 0,
                            reinterpret_cast<struct sockaddr*>(&sender), &slen);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            break;
        }

        // Any datagram registers the sender as a client
        udp.registerSender(sender);

        if (n == 0) continue;

        // Try to decode as CBOR TX request
        try {
            auto msg = gr::lora::cbor::decode_map(
                std::span<const uint8_t>(recv_buf.data(),
                                         static_cast<std::size_t>(n)));
            auto type = gr::lora::cbor::get_text_or(msg, "type", "");
            if (type == "lora_tx") {
                handle_tx_request(msg, cfg, udp, sender);
            }
        } catch (const gr::lora::cbor::DecodeError&) {
            // Not a valid CBOR map — ignore (registration-only datagram)
        }
    }

    // --- Shutdown ---
    std::fprintf(stderr, "\nStopping...\n");
    if (!rx_done.load(std::memory_order_relaxed)) {
        rx_sched.requestStop();
    }
    rx_thread.join();

    ::close(udp.fd);
    std::fprintf(stderr, "TRX stopped.\n");
    _exit(0);
}
