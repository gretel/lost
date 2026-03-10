// SPDX-License-Identifier: ISC
//
// lora_trx: full-duplex LoRa transceiver using native GR4 SoapySDR blocks.
//
// 1-ch RX: SoapySimpleSource -> FrameSync -> DemodDecoder -> FrameSink
// 2-ch RX: SoapyDualSimpleSource -+-> FrameSync -> DemodDecoder -> FrameSink (per-chain)
//                                 +-> FrameSync -> DemodDecoder -> FrameSink (per-chain)
// TX: bounded request queue with dedicated worker thread; LBT defers TX until channel clear
//
// Dual-RX uses a single MIMO stream (one device handle, two output ports).
// Full-duplex TX in both modes: RX runs continuously while TX uses an
// ephemeral graph with a separate device handle.
//
// All I/O is CBOR over UDP. RX frames fan out to all connected clients.
// TX requests (type: "lora_tx") are processed and acked.

#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <complex>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <toml++/toml.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/lora/FrameSync.hpp>
#include <gnuradio-4.0/lora/DemodDecoder.hpp>
#include <gnuradio-4.0/lora/Splitter.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>
#include <gnuradio-4.0/soapy/SoapySink.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include "FrameSink.hpp"
#include "TxQueueSource.hpp"
#include "cbor.hpp"
#include <gnuradio-4.0/lora/log.hpp>

namespace {

volatile std::sig_atomic_t g_running = 1;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void signal_handler(int /*sig*/) { g_running = 0; }

// Convert a TOML table to a GR4 property_map.
// TOML has only int64_t, double, bool, string. GR4 blocks need exact types
// (uint8_t, float, etc.) for applyStagedParameters. We store floats as float
// (not double) since most block properties use float, and narrow integers to
// the smallest fitting unsigned type. This covers all FrameSync/DemodDecoder
// properties without requiring type hints in TOML.
gr::property_map toml_to_property_map(const toml::table& tbl) {
    gr::property_map props;
    for (const auto& [key, val] : tbl) {
        std::pmr::string k(key.str());
        if (val.is_floating_point()) {
            // store as float (block properties: energy_thresh, min_snr_db)
            props[k] = static_cast<float>(val.as_floating_point()->get());
        } else if (val.is_integer()) {
            auto v = val.as_integer()->get();
            if (v >= 0 && v <= 255) {
                props[k] = static_cast<uint8_t>(v);
            } else if (v >= 0 && v <= 65535) {
                props[k] = static_cast<uint16_t>(v);
            } else if (v >= 0) {
                props[k] = static_cast<uint32_t>(v);
            } else {
                props[k] = static_cast<int32_t>(v);
            }
        } else if (val.is_boolean()) {
            props[k] = val.as_boolean()->get();
        } else if (val.is_string()) {
            props[k] = std::string(val.as_string()->get());
        }
    }
    return props;
}

// Merge override properties into a base property_map.
void merge_properties(gr::property_map& base, const gr::property_map& overrides) {
    for (const auto& [key, val] : overrides) {
        base.insert_or_assign(std::pmr::string(std::string(key)), val);
    }
}

// Return only the entries whose key appears in `allowed`.
// GR4's initial set() throws on unrecognised keys, so each block must
// receive only the overrides it actually declares as reflectable properties.
template<std::size_t N>
gr::property_map filter_properties(const gr::property_map& src,
                                   const std::array<const char*, N>& allowed) {
    gr::property_map out;
    for (const auto& [key, val] : src) {
        for (const auto* akey : allowed) {
            if (std::string_view(key) == akey) {
                out.insert_or_assign(std::pmr::string(std::string(key)), val);
                break;
            }
        }
    }
    return out;
}

// Override keys each block accepts (beyond the PHY keys already set explicitly).
// FrameSync: energy_thresh, min_snr_db, max_symbols
// DemodDecoder: impl_head, impl_cr, impl_pay_len, impl_has_crc, ldro_mode
static constexpr std::array kFrameSyncOverrides = {"energy_thresh", "min_snr_db", "max_symbols"};
static constexpr std::array kDemodDecoderOverrides = {
    "impl_head", "impl_cr", "impl_pay_len", "impl_has_crc", "ldro_mode"};  // NOLINT(whitespace/indent_namespace)

// Channel mapping: follows physical connector order on UHD case.
//   0 = TRX_A (chain A, TX/RX)   TX + RX
//   1 = RX_A  (chain A, RX2)     RX only
//   2 = RX_B  (chain B, RX2)     RX only
//   3 = TRX_B (chain B, TX/RX)   TX + RX
struct ChannelMap {
    uint32_t    soapy_channel;   // SoapySDR channel index (0 or 1)
    const char* rx_antenna;      // "TX/RX" or "RX2"
    const char* tx_antenna;      // "TX/RX" or nullptr (RX-only port)
    const char* label;           // human-readable label
};

constexpr std::array<ChannelMap, 4> kChannelMap = {{
    {0, "TX/RX", "TX/RX", "TRX_A"},   // config channel 0
    {0, "RX2",   nullptr,  "RX_A"},    // config channel 1
    {1, "RX2",   nullptr,  "RX_B"},    // config channel 2
    {1, "TX/RX", "TX/RX", "TRX_B"},   // config channel 3
}};

// Per-decode-chain configuration.  Each entry produces one FrameSync +
// DemodDecoder pair per radio channel.  Multiple DecodeConfig entries on
// the same radio channel are split via a Splitter block.
// All three identity keys (sf, sync_word, label) are required.
// Block-level overrides (min_snr_db, energy_thresh, ldro_mode, impl_*)
// are collected here from [[set_*.decode]] entries.
struct DecodeConfig {
    uint8_t          sf{8};
    uint16_t         sync_word{0x12};
    std::string      label{};
    gr::property_map block_overrides{};  ///< per-chain FrameSync/DemodDecoder overrides
};

struct TrxConfig {
    std::string          name{};
    std::string          device{"uhd"};
    std::string          device_param{"type=b200"};
    double               freq{869'618'000.0};
    double               gain_rx{30.0};
    double               gain_tx{75.0};
    float                rate{250'000.f};
    uint32_t             bw{62'500};
    uint8_t              sf{8};          ///< default SF (also used for TX)
    uint8_t              cr{4};
    uint16_t             sync{0x12};     ///< default sync word (also used for TX)
    uint16_t             preamble{8};
    std::string          clock{};
    std::vector<uint32_t> rx_channels{0};
    uint32_t             tx_channel{0};
    std::string          listen{"127.0.0.1"};
    uint16_t             port{5556};
    uint32_t             status_interval{10};  ///< seconds between status heartbeats (0 = off)
    bool                 debug{false};
    bool                 lbt{true};             ///< enable LBT (CAD-based)
    uint32_t             lbt_timeout_ms{2000};  ///< max wait for channel clear before rejecting TX
    uint32_t             tx_queue_depth{4};     ///< max queued TX requests (rejects when full)

    std::vector<DecodeConfig> decode_configs{};  ///< per-chain decode configs (at least one required)
};

// --- UDP state (owned by main thread) ---

static constexpr uint32_t kMaxSendFailures = 10;

struct ClientEntry {
    struct sockaddr_storage addr{};
    std::vector<uint16_t>  sync_words{};   // empty = no filter (receive all)
    uint32_t               send_failures{0};

    [[nodiscard]] bool accepts(uint16_t sw) const {
        return sync_words.empty() ||
               std::ranges::find(sync_words, sw) != sync_words.end();
    }
};

struct UdpState {
    int                      fd{-1};
    std::vector<ClientEntry> clients;
    std::mutex               mutex;

    void broadcast(const std::vector<uint8_t>& buf, uint16_t sync_word) {
        sendToAll(buf, &sync_word);
    }

    /// Broadcast to ALL clients, ignoring sync_word filters.
    void broadcast_all(const std::vector<uint8_t>& buf) {
        sendToAll(buf, nullptr);
    }

 private:
    void sendToAll(const std::vector<uint8_t>& buf, const uint16_t* filter_sw) {
        std::lock_guard<std::mutex> lock(mutex);
        std::erase_if(clients, [](const auto& c) {
            return c.send_failures >= kMaxSendFailures;
        });
        for (auto& client : clients) {
            if (filter_sw && !client.accepts(*filter_sw)) continue;
            auto len = (client.addr.ss_family == AF_INET6)
                ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
            auto rc = ::sendto(fd, buf.data(), buf.size(), 0,
                     reinterpret_cast<const struct sockaddr*>(&client.addr),
                     static_cast<socklen_t>(len));
            if (rc < 0) {
                ++client.send_failures;
            } else {
                client.send_failures = 0;
            }
        }
    }

 public:
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

    void subscribe(const struct sockaddr_storage& sender,
                   std::vector<uint16_t> sync_words = {}) {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& c : clients) {
            if (sockaddr_equal(c.addr, sender)) {
                c.sync_words = std::move(sync_words);
                c.send_failures = 0;
                return;
            }
        }
        clients.push_back({sender, std::move(sync_words), 0});
        auto& entry = clients.back();
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
        if (entry.sync_words.empty()) {
            gr::lora::log_ts("info ", "lora_trx",
                "UDP client %s:%u subscribed (all frames, %zu total)",
                addr_str, port, clients.size());
        } else {
            std::string filter;
            for (auto sw : entry.sync_words) {
                if (!filter.empty()) filter += ",";
                char tmp[8];
                std::snprintf(tmp, sizeof(tmp), "0x%02x", sw);
                filter += tmp;
            }
            gr::lora::log_ts("info ", "lora_trx",
                "UDP client %s:%u subscribed (sync_word=%s, %zu total)",
                addr_str, port, filter.c_str(), clients.size());
        }
    }

    void sendTo(const std::vector<uint8_t>& buf, const struct sockaddr_storage& dest) {
        auto len = (dest.ss_family == AF_INET6)
            ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
        ::sendto(fd, buf.data(), buf.size(), 0,
                 reinterpret_cast<const struct sockaddr*>(&dest),
                 static_cast<socklen_t>(len));
    }
};

// --- CLI + TOML config ---


void print_usage() {
    std::fprintf(stderr,
        "Usage: lora_trx [options]\n\n"
        "Full-duplex LoRa transceiver. All I/O is CBOR over UDP.\n"
        "TX requests use type \"lora_tx\" (see docs/cbor-schemas.md).\n\n"
        "Options:\n"
        "  --config <file>   TOML configuration file (default: config.toml)\n"
        "  --debug           Enable verbose state-machine traces on stderr\n"
        "  --no-lbt          Disable listen-before-talk (transmit immediately)\n"
        "  --version         Show version and exit\n"
        "  -h, --help        Show this help\n\n"
        "Configuration is defined entirely in the TOML file.\n"
        "See apps/config.toml for a documented example.\n");
}

// Parse CLI args — only --config, --debug, --version, --help.
// Returns 0 on success (config_path filled), 1 on error, 2 on --help/--version (clean exit).
int parse_args(int argc, char* argv[], TrxConfig& cfg, std::string& config_path) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 2;
        }
        if (arg == "--version") {
            std::fprintf(stderr, "lora_trx %s\n", GIT_REV);
            return 2;
        }
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
            continue;
        }
        if (arg == "--debug") {
            cfg.debug = true;
            continue;
        }
        if (arg == "--no-lbt") {
            cfg.lbt = false;
            continue;
        }
        std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
        print_usage();
        return 1;
    }
    if (config_path.empty()) {
        config_path = "config.toml";
    }
    return 0;
}

// Load TOML config file into a vector of TrxConfig (one per set).
// Returns empty vector on error (messages printed to stderr).
std::vector<TrxConfig> load_config(const std::string& path, bool debug) {
    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error& err) {
        gr::lora::log_ts("error", "lora_trx", "%s: %s",
                         path.c_str(), err.description().data());
        return {};
    }

    // --- [device] section (falls back to flat top-level keys for compatibility) ---
    auto* dev_tbl = tbl["device"].as_table();
    auto dev_or_s = [&](std::string_view key, std::string def) -> std::string {
        if (dev_tbl) {
            if (auto v = dev_tbl->at_path(key).value<std::string>()) return *v;
        }
        if (auto v = tbl[key].value<std::string>()) return *v;
        return def;
    };
    auto dev_or_d = [&](std::string_view key, double def) -> double {
        if (dev_tbl) {
            if (auto v = dev_tbl->at_path(key).value<double>()) return *v;
        }
        if (auto v = tbl[key].value<double>()) return *v;
        return def;
    };
    std::string g_device = dev_or_s("driver", dev_or_s("device", "uhd"));
    std::string g_param  = dev_or_s("param", "type=b200");
    auto        g_rate   = static_cast<float>(dev_or_d("rate", 250'000.0));
    std::string g_clock  = dev_or_s("clock", "");

    // --- [network] section (falls back to flat top-level keys for compatibility) ---
    auto* net_tbl = tbl["network"].as_table();
    auto net_or_s = [&](std::string_view key, std::string def) -> std::string {
        if (net_tbl) {
            if (auto v = net_tbl->at_path(key).value<std::string>()) return *v;
        }
        if (auto v = tbl[key].value<std::string>()) return *v;
        return def;
    };
    auto net_or_i = [&](std::string_view key, int64_t def) -> int64_t {
        if (net_tbl) {
            if (auto v = net_tbl->at_path(key).value<int64_t>()) return *v;
        }
        if (auto v = tbl[key].value<int64_t>()) return *v;
        return def;
    };
    auto net_or_b = [&](std::string_view key, bool def) -> bool {
        if (net_tbl) {
            if (auto v = net_tbl->at_path(key).value<bool>()) return *v;
        }
        if (auto v = tbl[key].value<bool>()) return *v;
        return def;
    };
    std::string g_listen     = net_or_s("udp_listen", "127.0.0.1");
    int64_t     g_port       = net_or_i("udp_port", int64_t{5556});
    int64_t     g_status_int = net_or_i("status_interval", int64_t{10});
    bool        g_lbt        = net_or_b("lbt", true);
    int64_t     g_lbt_tmo    = net_or_i("lbt_timeout_ms", int64_t{2000});
    int64_t     g_tx_qdepth  = net_or_i("tx_queue_depth", int64_t{4});
    bool        g_debug      = debug;

    // --- Collect codec and radio sections ---
    struct CodecCfg {
        uint8_t  sf{8};         ///< TX spreading factor (decode chains specify their own)
        uint32_t bw{62'500};
        uint8_t  cr{4};         ///< stored as offset (1-4) internally
        uint16_t sync{0x12};
        uint16_t preamble{8};
    };
    struct RadioCfg {
        double               freq{869'618'000.0};
        double               rx_gain{30.0};
        double               tx_gain{75.0};
        std::vector<uint32_t> rx_channels{0};
        uint32_t             tx_channel{0};
    };

    std::map<std::string, CodecCfg> codecs;
    std::map<std::string, RadioCfg> radios;

    for (auto&& [key, val] : tbl) {
        if (!val.is_table()) continue;
        auto& section = *val.as_table();
        std::string skey(key.str());

        if (skey.starts_with("codec_")) {
            CodecCfg c;
            // sf is optional in [codec_*] — used for TX only; decode chains set their own
            c.sf       = static_cast<uint8_t>(section["sf"].value_or(int64_t{8}));
            c.bw       = static_cast<uint32_t>(section["bw"].value_or(int64_t{62'500}));
            c.preamble = static_cast<uint16_t>(section["preamble_len"].value_or(int64_t{8}));
            c.sync     = static_cast<uint16_t>(section["sync_word"].value_or(int64_t{0x12}));

            // CR as denominator (5-8) in config, stored as offset (1-4) internally
            auto cr_val = static_cast<uint8_t>(section["cr"].value_or(int64_t{8}));
            if (cr_val >= 5 && cr_val <= 8) {
                c.cr = static_cast<uint8_t>(cr_val - 4);
            } else {
                gr::lora::log_ts("error", "lora_trx",
                    "[%s] cr must be 5-8 (denominator), got %u",
                    skey.c_str(), cr_val);
                return {};
            }

            codecs[skey] = c;
        } else if (skey.starts_with("radio_")) {
            RadioCfg r;
            r.freq    = section["freq"].value_or(869'618'000.0);
            r.rx_gain = section["rx_gain"].value_or(30.0);
            r.tx_gain = section["tx_gain"].value_or(75.0);

            if (auto* arr = section["rx_channel"].as_array()) {
                r.rx_channels.clear();
                for (auto& elem : *arr) {
                    auto ch = static_cast<uint32_t>(elem.value_or(int64_t{0}));
                    if (ch > 3) {
                        gr::lora::log_ts("error", "lora_trx",
                            "[%s] rx_channel %u out of range (0-3)",
                            skey.c_str(), ch);
                        return {};
                    }
                    r.rx_channels.push_back(ch);
                }
            }
            r.tx_channel = static_cast<uint32_t>(
                section["tx_channel"].value_or(int64_t{0}));
            if (r.tx_channel > 3) {
                gr::lora::log_ts("error", "lora_trx",
                    "[%s] tx_channel %u out of range (0-3)",
                    skey.c_str(), r.tx_channel);
                return {};
            }
            if (kChannelMap[r.tx_channel].tx_antenna == nullptr) {
                gr::lora::log_ts("error", "lora_trx",
                    "[%s] tx_channel %u (%s) is RX-only",
                    skey.c_str(), r.tx_channel,
                    kChannelMap[r.tx_channel].label);
                return {};
            }
            radios[skey] = r;
        }
    }

    // --- Parse sets and build TrxConfig vector ---
    // Collect set keys sorted by source line position (preserves file order).
    std::vector<std::pair<std::string, toml::source_position>> set_keys;
    for (auto&& [key, val] : tbl) {
        if (!val.is_table()) continue;
        std::string skey(key.str());
        if (!skey.starts_with("set_")) continue;
        set_keys.emplace_back(skey, val.source().begin);
    }
    std::sort(set_keys.begin(), set_keys.end(),
              [](const auto& a, const auto& b) { return a.second.line < b.second.line; });

    std::vector<TrxConfig> configs;

    // Keys that identify a decode chain entry — not block property overrides.
    static constexpr std::array kDecodeIdentityKeys = {
        std::string_view{"sf"}, std::string_view{"sync_word"}, std::string_view{"label"},
    };

    for (const auto& [skey, _pos] : set_keys) {
        auto& section = *tbl[skey].as_table();

        auto codec_ref = section["codec"].value<std::string>();
        auto radio_ref = section["radio"].value<std::string>();
        if (!codec_ref || !radio_ref) {
            gr::lora::log_ts("error", "lora_trx",
                "[%s] requires 'codec' and 'radio' keys", skey.c_str());
            return {};
        }

        auto cit = codecs.find(*codec_ref);
        if (cit == codecs.end()) {
            gr::lora::log_ts("error", "lora_trx",
                "[%s] references unknown codec '%s'",
                skey.c_str(), codec_ref->c_str());
            return {};
        }
        auto rit = radios.find(*radio_ref);
        if (rit == radios.end()) {
            gr::lora::log_ts("error", "lora_trx",
                "[%s] references unknown radio '%s'",
                skey.c_str(), radio_ref->c_str());
            return {};
        }

        const auto& codec = cit->second;
        const auto& radio = rit->second;

        TrxConfig cfg;
        // Global
        cfg.device       = g_device;
        cfg.device_param = g_param;
        cfg.rate         = static_cast<float>(g_rate);
        cfg.clock        = g_clock;
        cfg.listen       = g_listen;
        cfg.port         = static_cast<uint16_t>(g_port);
        cfg.status_interval = static_cast<uint32_t>(g_status_int);
        cfg.debug        = g_debug;
        cfg.lbt          = g_lbt;
        cfg.lbt_timeout_ms  = static_cast<uint32_t>(g_lbt_tmo);
        cfg.tx_queue_depth  = static_cast<uint32_t>(g_tx_qdepth);
        // Set-level
        cfg.name    = section["name"].value_or(std::string(skey));
        // Radio
        cfg.freq        = radio.freq;
        cfg.gain_rx     = radio.rx_gain;
        cfg.gain_tx     = radio.tx_gain;
        cfg.rx_channels = radio.rx_channels;
        cfg.tx_channel  = radio.tx_channel;
        // Codec
        cfg.sf       = codec.sf;
        cfg.bw       = codec.bw;
        cfg.cr       = codec.cr;
        cfg.sync     = codec.sync;
        cfg.preamble = codec.preamble;

        // Parse [[set_X.decode]] array — required, at least one entry.
        // Each entry specifies a decode chain with sf, sync_word, label (all required)
        // plus optional block property overrides (min_snr_db, energy_thresh, etc.).
        if (auto* decode_arr = section["decode"].as_array()) {
            for (auto& elem : *decode_arr) {
                if (!elem.is_table()) continue;
                auto& dtbl = *elem.as_table();
                DecodeConfig dc;

                auto sf_opt = dtbl["sf"].value<int64_t>();
                if (!sf_opt) {
                    gr::lora::log_ts("error", "lora_trx",
                        "[[%s.decode]] entry missing required key 'sf'",
                        skey.c_str());
                    return {};
                }
                dc.sf = static_cast<uint8_t>(*sf_opt);

                auto sw_opt = dtbl["sync_word"].value<int64_t>();
                if (!sw_opt) {
                    gr::lora::log_ts("error", "lora_trx",
                        "[[%s.decode]] entry missing required key 'sync_word'",
                        skey.c_str());
                    return {};
                }
                dc.sync_word = static_cast<uint16_t>(*sw_opt);

                auto lbl_opt = dtbl["label"].value<std::string>();
                if (!lbl_opt || lbl_opt->empty()) {
                    gr::lora::log_ts("error", "lora_trx",
                        "[[%s.decode]] entry missing required key 'label'",
                        skey.c_str());
                    return {};
                }
                dc.label = *lbl_opt;

                // Collect remaining keys as block property overrides
                for (const auto& [okey, oval] : dtbl) {
                    std::string ok_str(okey.str());
                    if (std::ranges::any_of(kDecodeIdentityKeys,
                                            [&](auto k) { return k == ok_str; })) {
                        continue;
                    }
                    if (oval.is_table()) continue;
                    toml::table tmp;
                    tmp.insert(okey, oval);
                    auto single = toml_to_property_map(tmp);
                    for (auto& [pk, pv] : single)
                        dc.block_overrides.insert_or_assign(pk, pv);
                }

                if (g_debug && !dc.block_overrides.empty()) {
                    gr::lora::log_ts("debug", "lora_trx",
                        "[[%s.decode]] '%s': %zu override(s)",
                        skey.c_str(), dc.label.c_str(),
                        dc.block_overrides.size());
                }

                cfg.decode_configs.push_back(std::move(dc));
            }
        }
        if (cfg.decode_configs.empty()) {
            gr::lora::log_ts("error", "lora_trx",
                "[%s] requires at least one [[%s.decode]] entry "
                "(sf, sync_word, label are required)",
                skey.c_str(), skey.c_str());
            return {};
        }

        if (g_debug) {
            // Build rx channel list string
            std::string rx_list;
            for (std::size_t i = 0; i < cfg.rx_channels.size(); i++) {
                if (i > 0) rx_list += ",";
                rx_list += std::to_string(cfg.rx_channels[i]);
                rx_list += "/";
                rx_list += kChannelMap[cfg.rx_channels[i]].label;
            }
            // Build decode chain string
            std::string dec_list;
            for (std::size_t i = 0; i < cfg.decode_configs.size(); i++) {
                if (i > 0) dec_list += ",";
                const auto& dc = cfg.decode_configs[i];
                char tmp[32];
                std::snprintf(tmp, sizeof(tmp), "SF%u/0x%02X", dc.sf, dc.sync_word);
                dec_list += tmp;
                if (!dc.label.empty()) { dec_list += "("; dec_list += dc.label; dec_list += ")"; }
            }
            gr::lora::log_ts("debug", "lora_trx",
                "config set '%s' (%s): radio='%s' codec='%s' freq=%.3f MHz rx=[%s] tx=%u/%s decode=[%s]",
                skey.c_str(), cfg.name.c_str(),
                radio_ref->c_str(), codec_ref->c_str(),
                cfg.freq / 1e6,
                rx_list.c_str(),
                cfg.tx_channel, kChannelMap[cfg.tx_channel].label,
                dec_list.c_str());
        }

        configs.push_back(std::move(cfg));
    }

    if (configs.empty()) {
        gr::lora::log_ts("error", "lora_trx",
            "no [set_*] sections found in config");
        return {};
    }

    return configs;
}

// --- TX ---

using cf32 = std::complex<float>;

std::vector<cf32> generate_iq(
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

// Persistent TX graph: stays alive for the process lifetime.
//
// Architecture (MIMO / 2-ch RX mode):
//   TxQueueSource(port0=IQ, port1=zeros) -> SoapySinkBlock<cf32,2>
//
// Architecture (single-ch RX mode):
//   TxQueueSource(port0=IQ, port1=zeros) -> SoapySinkBlock<cf32,2>
//
// The same SoapySinkBlock<cf32,2> is used in both modes.  In single-ch mode
// the second TX channel (ch1) is driven with zeros; in MIMO mode ch0 or ch1
// is selected based on cfg.tx_channel and the other receives zeros.
// The key property: the scheduler and device handle are created ONCE at
// startup (before RX starts) so reinitDevice() never races the RX stream.
//
// TX requests push IQ into TxQueueSource::push() and call notifyProgress()
// to wake the singleThreadedBlocking scheduler.
struct TxGraph {
    gr::lora::TxQueueSource*                         source{nullptr};
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking>* sched{nullptr};
    std::thread                                      thread{};
};

std::unique_ptr<gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking>>
build_tx_graph(gr::lora::TxQueueSource*& source_out, const TrxConfig& cfg) {
    const auto& tx_map = kChannelMap[cfg.tx_channel];

    // Both TX SoapySDR channels open (balanced UHD 2RX+2TX mode).
    // ch0 = TRX_A, ch1 = TRX_B.  IQ goes to cfg.tx_channel's soapy_channel;
    // the other channel carries zeros from TxQueueSource::out1.
    // TxQueueSource always puts IQ on out0.  If cfg.tx_channel maps to soapy
    // ch1, we swap port connections: sink.in#0 <- zeros (out1), sink.in#1 <- IQ (out0).
    const bool tx_on_ch1 = (tx_map.soapy_channel == 1);

    gr::Graph graph;

    graph.emplaceBlock<gr::lora::TxQueueSource>();

    auto& sink = graph.emplaceBlock<gr::blocks::soapy::SoapySinkBlock<cf32, 2UZ>>({
        {"device",              cfg.device},
        {"device_parameter",    cfg.device_param},
        {"sample_rate",         cfg.rate},
        {"tx_center_frequency", gr::Tensor<double>(gr::data_from, {cfg.freq, cfg.freq})},
        {"tx_gains",            gr::Tensor<double>(gr::data_from, {cfg.gain_tx, 0.0})},
        {"tx_channels",         gr::Tensor<gr::Size_t>(gr::data_from, {gr::Size_t{0}, gr::Size_t{1}})},
        {"tx_antennae",         std::vector<std::string>{tx_map.tx_antenna, "TX/RX"}},
        {"clock_source",        cfg.clock},
        {"timed_tx",            true},
        {"wait_burst_ack",      true},
        {"max_underflow_count", gr::Size_t{0}},
    });

    // Wire IQ port to the active TX channel; zeros to the dummy channel.
    // We look up the TxQueueSource via the graph's block list since emplaceBlock
    // returns a reference that becomes dangling after exchange(std::move(graph)).
    auto tx_block_list = graph.blocks();
    gr::lora::TxQueueSource* src_ptr = nullptr;
    for (auto& blk : tx_block_list) {
        if (blk->typeName().find("TxQueueSource") != std::string_view::npos) {
            src_ptr = static_cast<gr::lora::TxQueueSource*>(blk->raw());  // BlockWrapper holds block as member
            break;
        }
    }
    if (src_ptr == nullptr) {
        gr::lora::log_ts("error", "lora_trx",
            "TxQueueSource not found in TX graph");
        return nullptr;
    }
    auto& src = *src_ptr;

    if (!tx_on_ch1) {
        // IQ -> sink.in#0 (ch0=TRX_A), zeros -> sink.in#1 (ch1=TRX_B)
        graph.connect(src, "out0"s, sink, "in#0"s);
        graph.connect(src, "out1"s, sink, "in#1"s);
    } else {
        // zeros -> sink.in#0 (ch0=TRX_A), IQ -> sink.in#1 (ch1=TRX_B)
        graph.connect(src, "out1"s, sink, "in#0"s);
        graph.connect(src, "out0"s, sink, "in#1"s);
    }

    auto sched = std::make_unique<
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking>>();
    sched->timeout_inactivity_count  = 1U;
    sched->watchdog_min_stall_count  = 10U;  // safety margin if max_warnings becomes non-zero
    sched->watchdog_max_warnings     = 0U;   // disable all watchdog logging and escalation (TX idle is normal)
    if (auto ret = sched->exchange(std::move(graph)); !ret) {
        gr::lora::log_ts("error", "lora_trx", "TX scheduler init failed");
        return nullptr;
    }

    // src_ptr was captured pre-exchange; the TxQueueSource object is now owned
    // by the scheduler via shared_ptr but the raw pointer remains valid for the
    // process lifetime. No need to re-scan sched->blocks() post-exchange.
    source_out = src_ptr;

    gr::lora::log_ts("info ", "lora_trx",
        "TX graph started: config ch %u -> soapy ch %u / %s",
        cfg.tx_channel, tx_map.soapy_channel, tx_map.label);
    return sched;
}

int transmit(const std::vector<cf32>& iq, gr::lora::TxQueueSource& source) {
    source.push(iq);
    source.notifyProgress();
    return 0;
}

// --- TX request queue for LBT ---

struct TxRequest {
    gr::lora::cbor::Map      msg;
    struct sockaddr_storage   sender;
};

/// Bounded FIFO queue for TX requests.  Thread-safe (mutex + condvar).
/// The UDP loop pushes; a dedicated TX worker thread pops and processes.
class TxRequestQueue {
 public:
    explicit TxRequestQueue(std::size_t capacity) : _capacity(capacity) {}

    /// Push a request.  Returns false if queue is full (caller should reject).
    bool push(TxRequest req) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queue.size() >= _capacity) return false;
        _queue.push(std::move(req));
        _cv.notify_one();
        return true;
    }

    /// Pop a request, blocking until one is available or stop is requested.
    /// Returns std::nullopt on stop.
    std::optional<TxRequest> pop() {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_queue.empty() || _stop; });
        if (_stop && _queue.empty()) return std::nullopt;
        auto req = std::move(_queue.front());
        _queue.pop();
        return req;
    }

    void request_stop() {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
        _cv.notify_all();
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _queue.size();
    }

 private:
    mutable std::mutex          _mutex;
    std::condition_variable     _cv;
    std::queue<TxRequest>       _queue;
    std::size_t                 _capacity;
    bool                        _stop{false};
};

void handle_tx_request(const gr::lora::cbor::Map& msg, const TrxConfig& cfg,
                       UdpState& udp, const struct sockaddr_storage& sender,
                       gr::lora::TxQueueSource& txSource,
                       gr::lora::SpectrumState* tx_spectrum = nullptr) {
    using gr::lora::cbor::get_uint_or;
    using gr::lora::cbor::get_bool_or;

    auto payload = gr::lora::cbor::get_bytes(msg, "payload");
    if (payload.empty() || payload.size() > 255) {
        gr::lora::log_ts("error", "lora_trx",
            "TX invalid payload size %zu", payload.size());
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
        std::vector<cf32> buf;
        auto gap_samples = static_cast<uint32_t>(
            cfg.rate * static_cast<float>(gap_ms) / 1000.0f);
        for (int r = 0; r < repeat; r++) {
            buf.insert(buf.end(), iq.begin(), iq.end());
            if (r + 1 < repeat && gap_samples > 0) {
                buf.resize(buf.size() + gap_samples, cf32(0.f, 0.f));
            }
        }
        iq = std::move(buf);
    }

    double airtime = static_cast<double>(iq.size()) / static_cast<double>(cfg.rate);
    gr::lora::log_ts("info ", "lora_trx",
        "TX %zu bytes SF%u CR4/%u sync=0x%02X repeat=%d %.1f ms airtime%s",
        payload.size(), cfg.sf, 4u + cr, sync, repeat,
        airtime * 1000.0, dry_run ? " (dry run)" : "");

    // Push TX IQ to spectrum tap (if connected) for waterfall display
    if (tx_spectrum != nullptr) {
        tx_spectrum->push(iq.data(), iq.size());
    }

    bool ok = true;
    if (!dry_run) {
        ok = (transmit(iq, txSource) == 0);
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

// --- RX graph builder ---

// Creates FrameSync + DemodDecoder in the graph and returns references to both.
// Does NOT create a FrameSink — caller is responsible for downstream wiring.
//
// `rx_channel`: unique ID for this decode chain (radio_idx * 100 + decode_idx)
// `dc`: decode config (SF, sync_word) — may differ from the codec defaults
struct DecodePair {
    gr::lora::FrameSync&    sync;
    gr::lora::DemodDecoder& demod;
};

DecodePair add_decode_pair(gr::Graph& graph, const TrxConfig& cfg,
                           int32_t rx_channel, const DecodeConfig& dc,
                           std::shared_ptr<gr::lora::SpectrumState> spectrum = nullptr) {
    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    gr::property_map sync_props = {
        {"center_freq", static_cast<uint32_t>(cfg.freq)},
        {"bandwidth", cfg.bw},
        {"sf", dc.sf},
        {"sync_word", dc.sync_word},
        {"os_factor", os},
        {"preamble_len", cfg.preamble},
        {"rx_channel", rx_channel},
        {"debug", cfg.debug},
    };
    merge_properties(sync_props, filter_properties(dc.block_overrides, kFrameSyncOverrides));
    auto& sync = graph.emplaceBlock<gr::lora::FrameSync>(std::move(sync_props));
    sync._spectrum_state = std::move(spectrum);

    gr::property_map demod_props = {
        {"sf", dc.sf},
        {"bandwidth", cfg.bw},
        {"debug", cfg.debug},
    };
    merge_properties(demod_props, filter_properties(dc.block_overrides, kDemodDecoderOverrides));
    auto& demod = graph.emplaceBlock<gr::lora::DemodDecoder>(std::move(demod_props));

    auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
    if (!ok(graph.connect<"out">(sync).to<"in">(demod))) {
        gr::lora::log_ts("error", "lora_trx",
            "failed to connect FrameSync -> DemodDecoder for rx_channel %d",
            rx_channel);
    }
    return {sync, demod};
}

// Adds FrameSync -> DemodDecoder -> FrameSink chain for the simple case
// (1 radio channel, 1 decode config, no Splitter/Combiner needed).
auto& add_decode_chain(gr::Graph& graph, const TrxConfig& cfg,
                       int32_t rx_channel, const DecodeConfig& dc,
                       std::function<void(const std::vector<uint8_t>&, bool)> callback,
                       std::shared_ptr<gr::lora::SpectrumState> spectrum = nullptr) {
    auto [sync, demod] = add_decode_pair(graph, cfg, rx_channel, dc, std::move(spectrum));

    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"sync_word", dc.sync_word},
        {"phy_sf", dc.sf},
        {"phy_bw", cfg.bw},
        {"label", dc.label},
    });
    sink._frame_callback = callback;

    auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
    if (!ok(graph.connect<"out">(demod).to<"in">(sink))) {
        gr::lora::log_ts("error", "lora_trx",
            "failed to connect DemodDecoder -> FrameSink for rx_channel %d",
            rx_channel);
    }
    return sync;
}

// Generalised RX graph builder.
//
// Topology depends on the number of radio channels and decode configs:
//
//   1 radio, 1 decode:
//     Source → Splitter(2) → FrameSync → DemodDecoder → FrameSink
//                          → CAD (channel activity)
//
//   N radio × M decode (total chains > 1):
//     Source ─out#r─→ [Splitter(M+1)] ─out#0..M-1─→ FrameSync → DemodDecoder → FrameSink
//                                     ─out#M─→ CAD (first radio only)
//
// CAD monitors the first radio channel for listen-before-talk (LBT).
// Each decode chain has its own FrameSink (no DiversityCombiner).
//
// Returns pointer to source block's cumulative overflow counter (valid while graph lives).
uint64_t* build_rx_graph(gr::Graph& graph, const TrxConfig& cfg,
                         std::function<void(const std::vector<uint8_t>&, bool)> callback,
                         std::shared_ptr<gr::lora::SpectrumState> spectrum = nullptr,
                         std::atomic<bool>* channel_busy = nullptr) {
    auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
    using std::string_literals::operator""s;

    const auto& decodes = cfg.decode_configs;
    const auto nRadio = cfg.rx_channels.size();
    const auto nDecode = decodes.size();
    const auto nChains = nRadio * nDecode;

    // Translate config channels (0-3) to SoapySDR channels + antennae
    std::vector<gr::Size_t> soapy_channels;
    std::vector<std::string> antennae;
    for (auto ch : cfg.rx_channels) {
        const auto& map = kChannelMap[ch];
        soapy_channels.push_back(static_cast<gr::Size_t>(map.soapy_channel));
        antennae.emplace_back(map.rx_antenna);
    }

    // --- Simple path: 1 radio, 1 decode, no combiner ---
    if (nRadio == 1 && nDecode == 1) {
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapySimpleSource<std::complex<float>>>({
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
            {"rx_gains", gr::Tensor<double>{cfg.gain_rx}},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, soapy_channels)},
            {"rx_antennae", antennae},
            {"clock_source", cfg.clock},
            {"max_chunck_size", static_cast<uint32_t>(512U << 4U)},
            {"max_overflow_count", gr::Size_t{1000}},
            {"max_consecutive_errors", gr::Size_t{500}},
            {"max_time_out_us", static_cast<uint32_t>(10'000U)},
        });

        // Splitter: fanout to decode chain + CAD
        auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
            {"n_outputs", gr::Size_t{2}},
        });
        if (!ok(graph.connect<"out">(source).to<"in">(splitter))) {
            gr::lora::log_ts("error", "lora_trx",
                "failed to connect source -> Splitter");
        }

        auto rx_ch = static_cast<int32_t>(cfg.rx_channels[0]) * 100;
        auto& sync = add_decode_chain(graph, cfg, rx_ch, decodes[0], callback, spectrum);
        if (!ok(graph.connect(splitter, "out#0"s, sync, "in"s))) {
            gr::lora::log_ts("error", "lora_trx",
                "failed to connect Splitter -> decode chain");
        }

        // CAD block for LBT — NullSink drains the async output to prevent
        // the scheduler from stalling on a full unread output buffer.
        auto& cad = graph.emplaceBlock<gr::lora::ChannelActivityDetector>({
            {"sf", cfg.sf},
            {"bandwidth", cfg.bw},
            {"os_factor", static_cast<uint32_t>(cfg.rate / cfg.bw)},
            {"alpha", 4.16f},
            {"dual_chirp", true},
        });
        if (channel_busy != nullptr) {
            cad.set_channel_busy_flag(channel_busy);
        }
        if (!ok(graph.connect(splitter, "out#1"s, cad, "in"s))) {
            gr::lora::log_ts("error", "lora_trx",
                "failed to connect Splitter -> CAD");
        }
        auto& cad_sink = graph.emplaceBlock<gr::testing::NullSink<uint8_t>>();
        if (!ok(graph.connect<"out">(cad).to<"in">(cad_sink))) {
            gr::lora::log_ts("error", "lora_trx",
                "failed to connect CAD -> NullSink");
        }

        return &source._totalOverFlowCount;
    }

    // --- Multi-chain path: per-chain FrameSinks (no DiversityCombiner) ---

    // Wire decode chains for a given source block.
    // connectPort(r, downstream) connects source output `r` to downstream's "in".
    // Both lambdas preserve concrete types for graph.connect() template deduction.
    // Each decode chain gets its own FrameSink wired directly to DemodDecoder.
    auto wireDecodeChains = [&](auto connectPort) {
        std::size_t chain_idx = 0;
        for (std::size_t r = 0; r < nRadio; r++) {
            bool firstChain = (r == 0);
            // CAD gets one extra Splitter output on the first radio
            const bool addCad = firstChain && (channel_busy != nullptr);

            if (nDecode == 1) {
                // Insert Splitter(1 decode + optional CAD)
                auto nOut = static_cast<gr::Size_t>(addCad ? 2 : 1);
                if (nOut > 1) {
                    auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
                        {"n_outputs", nOut},
                    });
                    if (!connectPort(r, splitter)) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect source -> Splitter (radio %zu)", r);
                    }

                    auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100;
                    auto [sync, demod] = add_decode_pair(
                        graph, cfg, rx_ch, decodes[0],
                        firstChain ? spectrum : nullptr);
                    if (!ok(graph.connect(splitter, "out#0"s, sync, "in"s))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect Splitter -> FrameSync (radio %zu)", r);
                    }

                    if (addCad) {
                        auto& cad = graph.emplaceBlock<gr::lora::ChannelActivityDetector>({
                            {"sf", cfg.sf},
                            {"bandwidth", cfg.bw},
                            {"os_factor", static_cast<uint32_t>(cfg.rate / cfg.bw)},
                            {"alpha", 4.16f},
                            {"dual_chirp", true},
                        });
                        cad.set_channel_busy_flag(channel_busy);
                        if (!ok(graph.connect(splitter, "out#1"s, cad, "in"s))) {
                            gr::lora::log_ts("error", "lora_trx",
                                "failed to connect Splitter -> CAD");
                        }
                        auto& cad_sink = graph.emplaceBlock<gr::testing::NullSink<uint8_t>>();
                        if (!ok(graph.connect<"out">(cad).to<"in">(cad_sink))) {
                            gr::lora::log_ts("error", "lora_trx",
                                "failed to connect CAD -> NullSink");
                        }
                    }

                    // Per-chain sink
                    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
                        {"sync_word", decodes[0].sync_word},
                        {"phy_sf", decodes[0].sf},
                        {"phy_bw", cfg.bw},
                        {"label", decodes[0].label},
                    });
                    sink._frame_callback = callback;
                    if (!ok(graph.connect<"out">(demod).to<"in">(sink))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect DemodDecoder -> FrameSink (chain %zu)",
                            chain_idx);
                    }
                } else {
                    // No CAD, no splitter needed — direct connection
                    auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100;
                    auto [sync, demod] = add_decode_pair(
                        graph, cfg, rx_ch, decodes[0],
                        firstChain ? spectrum : nullptr);
                    if (!connectPort(r, sync)) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect source -> FrameSync (radio %zu)", r);
                    }

                    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
                        {"sync_word", decodes[0].sync_word},
                        {"phy_sf", decodes[0].sf},
                        {"phy_bw", cfg.bw},
                        {"label", decodes[0].label},
                    });
                    sink._frame_callback = callback;
                    if (!ok(graph.connect<"out">(demod).to<"in">(sink))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect DemodDecoder -> FrameSink (chain %zu)",
                            chain_idx);
                    }
                }
                chain_idx++;
            } else {
                // nDecode > 1: Splitter with nDecode outputs + optional CAD
                auto nOut = static_cast<gr::Size_t>(nDecode + (addCad ? 1 : 0));
                auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
                    {"n_outputs", nOut},
                });

                if (!connectPort(r, splitter)) {
                    gr::lora::log_ts("error", "lora_trx",
                        "failed to connect source -> Splitter (radio %zu)", r);
                }

                for (std::size_t d = 0; d < nDecode; d++) {
                    auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100
                               + static_cast<int32_t>(d);
                    auto [sync, demod] = add_decode_pair(
                        graph, cfg, rx_ch, decodes[d],
                        (firstChain && d == 0) ? spectrum : nullptr);

                    auto splitterPort = "out#"s + std::to_string(d);
                    if (!ok(graph.connect(splitter, splitterPort, sync, "in"s))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect Splitter -> FrameSync (chain %zu)",
                            chain_idx);
                    }

                    // Per-chain sink
                    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
                        {"sync_word", decodes[d].sync_word},
                        {"phy_sf", decodes[d].sf},
                        {"phy_bw", cfg.bw},
                        {"label", decodes[d].label},
                    });
                    sink._frame_callback = callback;
                    if (!ok(graph.connect<"out">(demod).to<"in">(sink))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect DemodDecoder -> FrameSink (chain %zu)",
                            chain_idx);
                    }
                    chain_idx++;
                }

                // Wire CAD to the extra Splitter output (first radio only)
                if (addCad) {
                    auto cadPort = "out#"s + std::to_string(nDecode);
                    auto& cad = graph.emplaceBlock<gr::lora::ChannelActivityDetector>({
                        {"sf", cfg.sf},
                        {"bandwidth", cfg.bw},
                        {"os_factor", static_cast<uint32_t>(cfg.rate / cfg.bw)},
                        {"alpha", 4.16f},
                        {"dual_chirp", true},
                    });
                    cad.set_channel_busy_flag(channel_busy);
                    if (!ok(graph.connect(splitter, cadPort, cad, "in"s))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect Splitter -> CAD");
                    }
                    auto& cad_sink = graph.emplaceBlock<gr::testing::NullSink<uint8_t>>();
                    if (!ok(graph.connect<"out">(cad).to<"in">(cad_sink))) {
                        gr::lora::log_ts("error", "lora_trx",
                            "failed to connect CAD -> NullSink");
                    }
                }

                for (std::size_t d = 0; d < nDecode; d++) {
                    auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100
                               + static_cast<int32_t>(d);
                    auto [sync, demod] = add_decode_pair(
                        graph, cfg, rx_ch, decodes[d],
                        (firstChain && d == 0) ? spectrum : nullptr);

                    auto splitterPort = "out#"s + std::to_string(d);
                    if (!ok(graph.connect(splitter, splitterPort, sync, "in"s))) {
                        std::fprintf(stderr, "ERROR: failed to connect Splitter -> FrameSync (chain %zu)\n",
                                    chain_idx);
                    }

                    // Per-chain sink
                    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
                        {"sync_word", decodes[d].sync_word},
                        {"phy_sf", decodes[d].sf},
                        {"phy_bw", cfg.bw},
                        {"label", decodes[d].label},
                    });
                    sink._frame_callback = callback;
                    if (!ok(graph.connect<"out">(demod).to<"in">(sink))) {
                        std::fprintf(stderr, "ERROR: failed to connect DemodDecoder -> FrameSink"
                                    " (chain %zu)\n", chain_idx);
                    }
                    chain_idx++;
                }

                // Wire CAD to the extra Splitter output (first radio only)
                if (addCad) {
                    auto& cad = graph.emplaceBlock<gr::lora::ChannelActivityDetector>({
                        {"sf", cfg.sf},
                        {"bandwidth", cfg.bw},
                        {"os_factor", static_cast<uint32_t>(cfg.rate / cfg.bw)},
                        {"alpha", 4.16f},
                        {"dual_chirp", true},
                    });
                    cad.set_channel_busy_flag(channel_busy);
                    auto cadPort = "out#"s + std::to_string(nDecode);
                    if (!ok(graph.connect(splitter, cadPort, cad, "in"s))) {
                        std::fprintf(stderr, "ERROR: failed to connect Splitter -> CAD\n");
                    }
                    auto& cad_sink = graph.emplaceBlock<gr::testing::NullSink<uint8_t>>();
                    if (!ok(graph.connect<"out">(cad).to<"in">(cad_sink))) {
                        std::fprintf(stderr, "ERROR: failed to connect CAD -> NullSink\n");
                    }
                }
            }
        }
    };

    // Create source and wire: 1 radio = SoapySimple, 2 radios = SoapyDual
    uint64_t* overflow_ptr = nullptr;

    if (nRadio == 1) {
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapySimpleSource<std::complex<float>>>({
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
            {"rx_gains", gr::Tensor<double>{cfg.gain_rx}},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, soapy_channels)},
            {"rx_antennae", antennae},
            {"clock_source", cfg.clock},
            {"max_chunck_size", static_cast<uint32_t>(512U << 4U)},
            {"max_overflow_count", gr::Size_t{1000}},
            {"max_consecutive_errors", gr::Size_t{500}},
            {"max_time_out_us", static_cast<uint32_t>(10'000U)},
        });
        overflow_ptr = &source._totalOverFlowCount;
        wireDecodeChains([&graph, &source](std::size_t /*r*/, auto& downstream) {
            return graph.connect<"out">(source).to<"in">(downstream) == gr::ConnectionResult::SUCCESS;
        });
    } else {
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapyDualSimpleSource<std::complex<float>>>({
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>(gr::data_from, {cfg.freq, cfg.freq})},
            {"rx_gains", gr::Tensor<double>(gr::data_from, {cfg.gain_rx, cfg.gain_rx})},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, soapy_channels)},
            {"rx_antennae", antennae},
            {"clock_source", cfg.clock},
            {"max_chunck_size", static_cast<uint32_t>(512U << 4U)},
            {"max_overflow_count", gr::Size_t{1000}},
            {"max_consecutive_errors", gr::Size_t{500}},
            {"max_time_out_us", static_cast<uint32_t>(10'000U)},
        });
        overflow_ptr = &source._totalOverFlowCount;
        wireDecodeChains([&graph, &source](std::size_t r, auto& downstream) {
            auto portName = "out#"s + std::to_string(r);
            return graph.connect(source, portName, downstream, "in"s) == gr::ConnectionResult::SUCCESS;
        });
    }

    if (cfg.debug) {
        gr::lora::log_ts("debug", "lora_trx",
            "RX graph: %zu radio(s) x %zu decode(s) = %zu chains",
            nRadio, nDecode, nChains);
    }

    return overflow_ptr;
}

// --- Shared status (updated by frame callback, read by main loop) ---

struct SharedStatus {
    std::atomic<uint32_t> frame_count{0};
    std::atomic<uint32_t> crc_ok{0};
    std::atomic<uint32_t> crc_fail{0};
    std::atomic<uint64_t> overflow_count{0};
};

/// Build a CBOR "config" message — sent once on subscribe.
std::vector<uint8_t> build_config_cbor(const TrxConfig& cfg) {
    namespace cbor = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(256);

    cbor::encode_map_begin(buf, 4);
    cbor::kv_text(buf, "type", "config");

    cbor::encode_text(buf, "phy");
    cbor::encode_map_begin(buf, 8);
    cbor::kv_float64(buf, "freq", cfg.freq);
    cbor::kv_uint(buf, "sf", cfg.sf);
    cbor::kv_uint(buf, "bw", cfg.bw);
    cbor::kv_uint(buf, "cr", cfg.cr);
    cbor::kv_uint(buf, "sync_word", cfg.sync);
    cbor::kv_uint(buf, "preamble", cfg.preamble);
    cbor::kv_float64(buf, "rx_gain", cfg.gain_rx);
    cbor::kv_float64(buf, "tx_gain", cfg.gain_tx);

    cbor::encode_text(buf, "server");
    cbor::encode_map_begin(buf, 3);
    cbor::kv_text(buf, "device", cfg.device);
    cbor::kv_uint(buf, "status_interval", cfg.status_interval);
    cbor::kv_float64(buf, "sample_rate", static_cast<double>(cfg.rate));

    // Decode chain summary
    cbor::encode_text(buf, "decode_chains");
    cbor::encode_array_begin(buf, static_cast<uint32_t>(cfg.decode_configs.size()));
    for (const auto& dc : cfg.decode_configs) {
        cbor::encode_map_begin(buf, 3);
        cbor::kv_text(buf, "label", dc.label);
        cbor::kv_uint(buf, "sf", dc.sf);
        cbor::kv_uint(buf, "sync_word", dc.sync_word);
    }

    return buf;
}

/// Build a CBOR "status" heartbeat — sent periodically.
std::vector<uint8_t> build_status_cbor(const TrxConfig& cfg, SharedStatus& status) {
    namespace cbor = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(96);

    uint32_t total    = status.frame_count.load();
    uint32_t ok_cnt   = status.crc_ok.load();
    uint32_t fail_cnt = status.crc_fail.load();
    uint64_t ovf_cnt  = status.overflow_count.load();

    cbor::encode_map_begin(buf, 5);
    cbor::kv_text(buf, "type", "status");

    // Timestamp
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    char ts[32]{};
    std::strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&tt));
    cbor::kv_text(buf, "ts", ts);

    cbor::encode_text(buf, "phy");
    cbor::encode_map_begin(buf, 2);
    cbor::kv_float64(buf, "rx_gain", cfg.gain_rx);
    cbor::kv_float64(buf, "tx_gain", cfg.gain_tx);

    cbor::encode_text(buf, "frames");
    cbor::encode_map_begin(buf, 3);
    cbor::kv_uint(buf, "total", total);
    cbor::kv_uint(buf, "crc_ok", ok_cnt);
    cbor::kv_uint(buf, "crc_fail", fail_cnt);

    cbor::kv_uint(buf, "rx_overflows", ovf_cnt);

    return buf;
}

/// Build a CBOR spectrum message from a SpectrumState result buffer.
/// The `type_name` parameter selects the message type ("spectrum" for RX,
/// "spectrum_tx" for TX).
std::vector<uint8_t> build_spectrum_cbor(gr::lora::SpectrumState& spec,
                                         const char* type_name = "spectrum") {
    namespace cbor = gr::lora::cbor;
    std::vector<uint8_t> buf;
    // 5 keys: type, bins (as raw bytes), fft_size, center_freq, sample_rate
    buf.reserve(64 + spec.fft_size * sizeof(float));
    cbor::encode_map_begin(buf, 5);
    cbor::kv_text(buf, "type", type_name);
    {
        std::lock_guard lock(spec.result_mutex);
        cbor::kv_bytes(buf, "bins",
                       reinterpret_cast<const uint8_t*>(spec.magnitude_db.data()),
                       spec.magnitude_db.size() * sizeof(float));
        spec.result_ready.store(false, std::memory_order_relaxed);
    }
    cbor::kv_uint(buf, "fft_size", spec.fft_size);
    cbor::kv_float64(buf, "center_freq", spec.center_freq);
    cbor::kv_float64(buf, "sample_rate", static_cast<double>(spec.sample_rate));
    return buf;
}

// --- Runtime reconfig: block references after scheduler exchange ---

// Holds shared_ptr<BlockModel> references to RX blocks found in the live
// scheduler graph.  After rx_sched.exchange(std::move(graph)), the original
// emplaceBlock references are dangling — we must use rx_sched.blocks() to
// locate them by typeName().
struct RxBlocks {
    std::vector<std::shared_ptr<gr::BlockModel>> frame_sync;
    std::vector<std::shared_ptr<gr::BlockModel>> demod_decoder;
    std::vector<std::shared_ptr<gr::BlockModel>> splitters;
    std::shared_ptr<gr::BlockModel>              soapy_source;
    std::shared_ptr<gr::BlockModel>              cad_block;
};

// Scan the scheduler's block span and collect references by type name.
RxBlocks find_rx_blocks(std::span<std::shared_ptr<gr::BlockModel>> blocks) {
    RxBlocks rx;
    for (auto& blk : blocks) {
        auto tn = blk->typeName();
        if (tn.find("FrameSync") != std::string_view::npos) {
            rx.frame_sync.push_back(blk);
        } else if (tn.find("DemodDecoder") != std::string_view::npos) {
            rx.demod_decoder.push_back(blk);
        } else if (tn.find("Splitter") != std::string_view::npos) {
            rx.splitters.push_back(blk);
        } else if (tn.find("ChannelActivityDetector") != std::string_view::npos) {
            rx.cad_block = blk;
        } else if (tn.find("Soapy") != std::string_view::npos) {
            rx.soapy_source = blk;
        }
    }
    return rx;
}

// Convert CBOR lora_config values to a property_map suitable for
// FrameSync / DemodDecoder settings.  CBOR uint -> narrowed to the exact
// C++ type expected by the block's Annotated<> field.
// Returns empty map if validation fails (e.g. SF out of range).
gr::property_map cbor_to_phy_props(const gr::lora::cbor::Map& msg, bool& valid) {
    namespace cbor = gr::lora::cbor;
    gr::property_map props;
    valid = true;

    auto sf_val  = cbor::get_uint_or(msg, "sf", 0);
    auto bw_val  = cbor::get_uint_or(msg, "bw", 0);
    auto sw_val  = cbor::get_uint_or(msg, "sync_word", 0);
    auto pre_val = cbor::get_uint_or(msg, "preamble", 0);

    if (sf_val > 0) {
        if (sf_val < 7 || sf_val > 12) {
            gr::lora::log_ts("error", "lora_trx",
                "lora_config: sf=%" PRIu64 " out of range [7,12]", sf_val);
            valid = false;
            return {};
        }
        props["sf"] = static_cast<uint8_t>(sf_val);
    }
    if (bw_val  > 0) props["bandwidth"]    = static_cast<uint32_t>(bw_val);
    if (sw_val  > 0) props["sync_word"]    = static_cast<uint16_t>(sw_val);
    if (pre_val > 0) props["preamble_len"] = static_cast<uint16_t>(pre_val);

    return props;
}

// Handle a "lora_config" request: applies PHY parameter changes to the
// live RX blocks, optionally retunes the SoapySource, and updates the
// TrxConfig snapshot so that subsequent TX uses the new parameters.
// Returns true if all settings were accepted.
bool handle_lora_config(const gr::lora::cbor::Map& msg,
                        TrxConfig& cfg,
                        const RxBlocks& rx_blocks,
                        UdpState& udp,
                        const struct sockaddr_storage& sender) {
    namespace cbor = gr::lora::cbor;

    bool valid = true;
    auto phy_props = cbor_to_phy_props(msg, valid);
    bool any_rejected = !valid;

    // Frequency change requires retuning the SoapySource block
    double new_freq = cbor::get_float64_or(msg, "freq", 0.0);
    bool freq_changed = (new_freq > 0.0 && new_freq != cfg.freq);

    // Gain changes
    double new_rx_gain = cbor::get_float64_or(msg, "rx_gain", 0.0);
    double new_tx_gain = cbor::get_float64_or(msg, "tx_gain", 0.0);

    // Apply PHY parameters to all FrameSync blocks
    if (!phy_props.empty()) {
        for (auto& blk : rx_blocks.frame_sync) {
            auto rejected = blk->settings().set(phy_props);
            if (!rejected.empty()) {
                gr::lora::log_ts("warn ", "lora_trx",
                    "lora_config: FrameSync rejected %zu key(s)", rejected.size());
                any_rejected = true;
            }
        }
        // DemodDecoder only cares about sf and bandwidth
        gr::property_map demod_props;
        if (auto it = phy_props.find("sf"); it != phy_props.end())
            demod_props["sf"] = it->second;
        if (auto it = phy_props.find("bandwidth"); it != phy_props.end())
            demod_props["bandwidth"] = it->second;
        if (!demod_props.empty()) {
            for (auto& blk : rx_blocks.demod_decoder) {
                auto rejected = blk->settings().set(demod_props);
                if (!rejected.empty()) {
                    gr::lora::log_ts("warn ", "lora_trx",
                        "lora_config: DemodDecoder rejected %zu key(s)",
                        rejected.size());
                    any_rejected = true;
                }
            }
        }
    }

    // Retune SoapySource center frequency
    if (freq_changed && rx_blocks.soapy_source) {
        gr::property_map freq_props;
        // SoapySource uses Tensor<double> for rx_center_frequency
        freq_props["rx_center_frequency"] = gr::Tensor<double>{new_freq};
        auto rejected = rx_blocks.soapy_source->settings().set(freq_props);
        if (!rejected.empty()) {
            gr::lora::log_ts("warn ", "lora_trx",
                "lora_config: SoapySource rejected freq change");
        }
        // Also update FrameSync center_freq (used for debug/display only)
        for (auto& blk : rx_blocks.frame_sync) {
            std::ignore = blk->settings().set({{"center_freq", static_cast<uint32_t>(new_freq)}});
        }
    }

    // Apply RX gain change to SoapySource
    if (new_rx_gain > 0.0 && rx_blocks.soapy_source) {
        gr::property_map gain_props;
        gain_props["rx_gains"] = gr::Tensor<double>{new_rx_gain};
        std::ignore = rx_blocks.soapy_source->settings().set(gain_props);
    }

    // Update TrxConfig snapshot (affects future TX requests + config broadcast)
    if (auto it = phy_props.find("sf"); it != phy_props.end()) {
        if (auto* ptr = it->second.get_if<uint8_t>()) cfg.sf = *ptr;
    }
    if (auto it = phy_props.find("bandwidth"); it != phy_props.end()) {
        if (auto* ptr = it->second.get_if<uint32_t>()) cfg.bw = *ptr;
    }
    if (auto it = phy_props.find("sync_word"); it != phy_props.end()) {
        if (auto* ptr = it->second.get_if<uint16_t>()) cfg.sync = *ptr;
    }
    if (auto it = phy_props.find("preamble_len"); it != phy_props.end()) {
        if (auto* ptr = it->second.get_if<uint16_t>()) cfg.preamble = *ptr;
    }
    if (freq_changed)
        cfg.freq = new_freq;
    if (new_rx_gain > 0.0)
        cfg.gain_rx = new_rx_gain;
    if (new_tx_gain > 0.0)
        cfg.gain_tx = new_tx_gain;

    gr::lora::log_ts("info ", "lora_trx",
        "lora_config: applied sf=%u bw=%u sync=0x%02X freq=%.0f",
        cfg.sf, cfg.bw, cfg.sync, cfg.freq);

    // Send ack
    bool ok = !any_rejected;
    std::vector<uint8_t> ack;
    ack.reserve(64);
    cbor::encode_map_begin(ack, 2);
    cbor::kv_text(ack, "type", "lora_config_ack");
    cbor::kv_bool(ack, "ok", ok);
    udp.sendTo(ack, sender);

    // Re-broadcast updated config to all clients
    auto config_cbor = build_config_cbor(cfg);
    udp.broadcast_all(config_cbor);

    return ok;
}

}  // namespace

int main(int argc, char* argv[]) {
    TrxConfig cfg;  // holds defaults for parse_args (only debug is set there)
    std::string config_path;
    int rc = parse_args(argc, argv, cfg, config_path);
    if (rc != 0) {
        return rc == 2 ? 0 : 1;  // 2 = --help/--version (clean exit)
    }

    auto configs = load_config(config_path, cfg.debug);
    if (configs.empty()) {
        return 1;
    }

    // Use first set for now (multi-set support is future work)
    cfg = std::move(configs[0]);

    if (cfg.rx_channels.empty()) {
        gr::lora::log_ts("error", "lora_trx",
            "rx_channel must specify at least one channel");
        return 1;
    }
    // Validate no SoapySDR channel conflicts: each config channel (0-3) maps to
    // SoapySDR channel 0 or 1. Two config channels sharing the same SoapySDR
    // channel would require different antenna selections on one stream — not possible.
    {
        std::array<int, 2> soapy_used = {-1, -1};  // tracks which config ch uses each soapy ch
        for (auto ch : cfg.rx_channels) {
            auto soapy_ch = kChannelMap[ch].soapy_channel;
            if (soapy_used[soapy_ch] >= 0) {
                gr::lora::log_ts("error", "lora_trx",
                    "rx_channel %u (%s) and %u (%s) both map to "
                    "SoapySDR channel %u — only one antenna per channel",
                    static_cast<uint32_t>(soapy_used[soapy_ch]),
                    kChannelMap[static_cast<uint32_t>(soapy_used[soapy_ch])].label,
                    ch, kChannelMap[ch].label, soapy_ch);
                return 1;
            }
            soapy_used[soapy_ch] = static_cast<int>(ch);
        }
    }
    if (cfg.rx_channels.size() > 2) {
        gr::lora::log_ts("error", "lora_trx",
            "rx_channel has %zu entries but UHD supports at most "
            "2 simultaneous RX streams (one per SoapySDR channel)",
            cfg.rx_channels.size());
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Homebrew UHD patches B2XX_FPGA_FILE_NAME to use full absolute Cellar paths.
    // These break after Homebrew revision bumps (e.g. 4.9.0.1 → 4.9.0.1_1).
    // Override with bare filenames so find_image_path() resolves via UHD_IMAGES_DIR.
    if (cfg.device == "uhd" && cfg.device_param.find("fpga=") == std::string::npos) {
        cfg.device_param += ",fpga=usrp_b210_fpga.bin";
    }

    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdate-time"
    gr::lora::log_ts("info ", "lora_trx",
        "starting lora_trx %s (built " __DATE__ " " __TIME__ ")", GIT_REV);
#pragma GCC diagnostic pop
    gr::lora::log_ts("info ", "lora_trx",
        "config %s (%zu set%s)  set '%s'",
        config_path.c_str(), configs.size(),
        configs.size() > 1 ? "s" : "",
        cfg.name.c_str());
    gr::lora::log_ts("info ", "lora_trx",
        "device %s  param %s",
        cfg.device.c_str(),
        cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());

    // Query hardware info (FPGA version, serial) via the GR4 RAII wrapper.
    {
        auto args = gr::blocks::soapy::detail::buildDeviceArgs(cfg.device, cfg.device_param);
        gr::blocks::soapy::Device dev(args);
        if (dev.get() != nullptr) {
            auto info = dev.getHardwareInfo();
            std::string fpga_ver, serial;
            if (auto it = info.find("fpga_version"); it != info.end())
                fpga_ver = it->second;
            if (auto it = info.find("serial"); it != info.end())
                serial = it->second;
            if (!fpga_ver.empty() || !serial.empty()) {
                gr::lora::log_ts("info ", "lora_trx",
                    "hardware: FPGA v%s  serial %s",
                    fpga_ver.empty() ? "?" : fpga_ver.c_str(),
                    serial.empty() ? "?" : serial.c_str());
            }
        }
    }

    // Build RX channel string
    {
        std::string rx_ch_str;
        for (auto ch : cfg.rx_channels) {
            if (!rx_ch_str.empty()) rx_ch_str += " ";
            char tmp[32];
            std::snprintf(tmp, sizeof(tmp), "%u(%s)", ch, kChannelMap[ch].label);
            rx_ch_str += tmp;
        }
        std::string lbt_detail;
        if (cfg.lbt) {
            char tmp[64];
            std::snprintf(tmp, sizeof(tmp), " timeout=%ums queue=%u",
                cfg.lbt_timeout_ms, cfg.tx_queue_depth);
            lbt_detail = tmp;
        }
        gr::lora::log_ts("info ", "lora_trx",
            "freq %.6f MHz  gain_rx %.0f dB  gain_tx %.0f dB  "
            "rate %.0f S/s (os=%u)  SF%u BW%u CR4/%u sync=0x%02X pre=%u",
            cfg.freq / 1e6, cfg.gain_rx, cfg.gain_tx,
            static_cast<double>(cfg.rate), os,
            cfg.sf, cfg.bw, 4u + cfg.cr, cfg.sync, cfg.preamble);
        gr::lora::log_ts("info ", "lora_trx",
            "rx ch %s  tx ch %u(%s)  LBT %s%s  listen %s:%u",
            rx_ch_str.c_str(),
            cfg.tx_channel, kChannelMap[cfg.tx_channel].label,
            cfg.lbt ? "on" : "off", lbt_detail.c_str(),
            cfg.listen.c_str(), cfg.port);
    }
    if (cfg.decode_configs.size() > 1) {
        std::string dec_str;
        for (std::size_t i = 0; i < cfg.decode_configs.size(); i++) {
            const auto& dc = cfg.decode_configs[i];
            if (i > 0) dec_str += "  ";
            char tmp[48];
            std::snprintf(tmp, sizeof(tmp), "SF%u/0x%02X", dc.sf, dc.sync_word);
            dec_str += tmp;
            if (!dc.label.empty()) { dec_str += "("; dec_str += dc.label; dec_str += ")"; }
        }
        gr::lora::log_ts("info ", "lora_trx", "decode chains: %s", dec_str.c_str());
    }
    if (!cfg.clock.empty()) {
        gr::lora::log_ts("info ", "lora_trx", "clock source: %s", cfg.clock.c_str());
    }
    if (cfg.debug) {
        gr::lora::log_ts("debug", "lora_trx", "debug output enabled");
    }
    if (cfg.status_interval > 0) {
        gr::lora::log_ts("info ", "lora_trx",
            "status heartbeat every %u s", cfg.status_interval);
    }

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
        gr::lora::log_ts("error", "lora_trx",
            "invalid listen address '%s'", cfg.listen.c_str());
        return 1;
    }

    udp.fd = ::socket(use_v6 ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
    if (udp.fd < 0) {
        gr::lora::log_ts("error", "lora_trx",
            "socket(): %s", std::strerror(errno));
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
            gr::lora::log_ts("error", "lora_trx",
                "bind([%s]:%u): %s",
                cfg.listen.c_str(), cfg.port, std::strerror(errno));
            ::close(udp.fd);
            return 1;
        }
    } else {
        bind4.sin_family = AF_INET;
        bind4.sin_port = htons(cfg.port);
        if (::bind(udp.fd, reinterpret_cast<struct sockaddr*>(&bind4),
                   sizeof(bind4)) < 0) {
            gr::lora::log_ts("error", "lora_trx",
                "bind(%s:%u): %s",
                cfg.listen.c_str(), cfg.port, std::strerror(errno));
            ::close(udp.fd);
            return 1;
        }
    }

    // Non-blocking for the recv loop
    int flags = ::fcntl(udp.fd, F_GETFL, 0);
    ::fcntl(udp.fd, F_SETFL, flags | O_NONBLOCK);

    gr::lora::log_ts("info ", "lora_trx",
        "UDP bound %s:%u%s",
        cfg.listen.c_str(), cfg.port, use_v6 ? " (IPv6 dual-stack)" : "");

    // --- Shared status for periodic heartbeat ---
    SharedStatus shared_status;

    // --- Frame callback: fan out RX frames to all clients + update status ---
    auto frame_callback = [&udp, &shared_status, sync = cfg.sync](
            const std::vector<uint8_t>& cbor_frame, bool crc_valid) {
        udp.broadcast(cbor_frame, sync);
        shared_status.frame_count.fetch_add(1, std::memory_order_relaxed);
        if (crc_valid) {
            shared_status.crc_ok.fetch_add(1, std::memory_order_relaxed);
        } else {
            shared_status.crc_fail.fetch_add(1, std::memory_order_relaxed);
        }
    };

    // --- Build persistent TX graph (created before RX to avoid device handle races) ---
    // SoapySinkBlock<cf32,2> stays open for the process lifetime; TxQueueSource
    // receives bursts from the UDP loop thread via push() + notifyProgress().
    gr::lora::TxQueueSource* tx_source_ptr = nullptr;
    auto tx_sched = build_tx_graph(tx_source_ptr, cfg);
    if (!tx_sched || tx_source_ptr == nullptr) {
        gr::lora::log_ts("error", "lora_trx", "failed to build TX graph");
        ::close(udp.fd);
        return 1;
    }
    gr::lora::TxQueueSource& tx_source = *tx_source_ptr;
    std::thread tx_thread([&tx_sched]() {
        auto ret = tx_sched->runAndWait();
        if (!ret.has_value()) {
            gr::lora::log_ts("warn ", "lora_trx",
                "TX scheduler stopped: %s",
                std::format("{}", ret.error()).c_str());
        }
    });

    // --- Spectrum taps for waterfall display ---
    // RX spectrum: fed by FrameSync in the RX graph
    auto spectrum = std::make_shared<gr::lora::SpectrumState>();
    spectrum->sample_rate = cfg.rate;
    spectrum->center_freq = cfg.freq;
    spectrum->init();

    // TX spectrum: fed by handle_tx_request before transmit
    auto tx_spectrum = std::make_shared<gr::lora::SpectrumState>();
    tx_spectrum->sample_rate = cfg.rate;
    tx_spectrum->center_freq = cfg.freq;
    tx_spectrum->init();

    // --- Shrink the global CPU thread pool ---
    // The default pool spawns hardware_concurrency() threads (8 on M1), all idle.
    // On macOS, each idle thread polls at 10μs (no condvar support) — ~100% wasted CPU.
    // singleThreadedBlocking runs on the calling thread, so the pool is unused.
    gr::thread_pool::Manager::defaultCpuPool()->setThreadBounds(1, 1);

    // --- LBT: channel-busy flag updated by CAD in the RX graph ---
    std::atomic<bool> channel_busy{false};

    // --- Build and start RX graph in a background thread ---
    gr::Graph rx_graph;
    uint64_t* overflow_ptr = build_rx_graph(rx_graph, cfg, frame_callback, spectrum,
                                            &channel_busy);

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking> rx_sched;
    rx_sched.timeout_inactivity_count  = 1U;   // sleep after 1 idle cycle; SoapySource notifies progress on data
    rx_sched.watchdog_min_stall_count  = 10U;  // suppress watchdog for ≤10s gaps (normal inter-frame silence)
    rx_sched.watchdog_max_warnings     = 30U;  // 30s of continuous stall → ERROR
    if (auto ret = rx_sched.exchange(std::move(rx_graph)); !ret) {
        gr::lora::log_ts("error", "lora_trx", "RX scheduler init failed");
        tx_sched->requestStop();
        tx_thread.join();
        ::close(udp.fd);
        return 1;
    }

    // Locate RX blocks in the live scheduler graph for runtime reconfig.
    // After exchange(), the original emplaceBlock references are dangling —
    // we find them by scanning typeName().
    auto rx_blocks = find_rx_blocks(rx_sched.blocks());
    gr::lora::log_ts("info ", "lora_trx",
        "RX blocks: %zu FrameSync  %zu DemodDecoder  %zu Splitter  CAD=%s  soapy=%s",
        rx_blocks.frame_sync.size(), rx_blocks.demod_decoder.size(),
        rx_blocks.splitters.size(),
        rx_blocks.cad_block ? "yes" : "no",
        rx_blocks.soapy_source ? "yes" : "no");

    std::atomic<bool> rx_done{false};
    auto rx_start_time = std::chrono::steady_clock::now();
    std::thread rx_thread([&rx_sched, &rx_done, &shared_status, rx_start_time, overflow_ptr]() {
        auto ret = rx_sched.runAndWait();
        // Sync final overflow count from source block
        if (overflow_ptr) {
            shared_status.overflow_count.store(*overflow_ptr, std::memory_order_relaxed);
        }
        if (!ret.has_value()) {
            auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - rx_start_time).count();
            auto err = std::format("{}", ret.error());
            gr::lora::log_ts("warn ", "lora_trx",
                "RX scheduler stopped: %s", err.c_str());
            gr::lora::log_ts("info ", "lora_trx",
                "uptime %llds  frames %u (crc_ok %u crc_fail %u)  overflows %" PRIu64,
                static_cast<long long>(uptime),
                shared_status.frame_count.load(),
                shared_status.crc_ok.load(),
                shared_status.crc_fail.load(),
                shared_status.overflow_count.load());
            if (err.find("OVERFLOW") != std::string::npos) {
                gr::lora::log_ts("info ", "lora_trx",
                    "hint: sustained USB buffer overflow — "
                    "try increasing num_recv_frames (e.g. device_param="
                    "\"num_recv_frames=128\") or reducing sample rate");
            }
            if (err.find("consecutive errors") != std::string::npos) {
                gr::lora::log_ts("info ", "lora_trx",
                    "hint: SDR device may be unresponsive — "
                    "check USB connection and device health");
            }
            if (err.find("watchdog stall") != std::string::npos) {
                gr::lora::log_ts("info ", "lora_trx",
                    "hint: RX pipeline stalled — "
                    "no data processed for extended period");
            }
        }
        rx_done.store(true, std::memory_order_relaxed);
    });

    gr::lora::log_ts("info ", "lora_trx",
        "RX running (%zu channel%s) — Ctrl+C to stop",
        cfg.rx_channels.size(),
        cfg.rx_channels.size() > 1 ? "s" : "");

    // --- Pre-build config CBOR (sent once on subscribe) ---
    auto config_cbor = build_config_cbor(cfg);

    // --- TX request queue + worker thread (replaces tx_busy + io pool dispatch) ---
    TxRequestQueue tx_queue(cfg.tx_queue_depth);

    std::thread tx_worker([&tx_queue, &cfg, &udp, &tx_source, &channel_busy,
                           tx_spectrum_ref = tx_spectrum]() {
        while (auto req = tx_queue.pop()) {
            // LBT: wait for channel to clear before transmitting
            if (cfg.lbt) {
                auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::milliseconds(cfg.lbt_timeout_ms);
                bool timed_out = false;
                while (channel_busy.load(std::memory_order_acquire)) {
                    if (std::chrono::steady_clock::now() >= deadline) {
                        timed_out = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                if (timed_out) {
                    uint64_t seq = gr::lora::cbor::get_uint_or(req->msg, "seq", 0);
                    gr::lora::log_ts("warn ", "lora_trx",
                        "LBT timeout (channel busy >%ums), rejecting seq=%" PRIu64,
                        cfg.lbt_timeout_ms, seq);
                    std::vector<uint8_t> rej;
                    rej.reserve(80);
                    gr::lora::cbor::encode_map_begin(rej, 4);
                    gr::lora::cbor::kv_text(rej, "type", "lora_tx_ack");
                    gr::lora::cbor::kv_uint(rej, "seq", seq);
                    gr::lora::cbor::kv_bool(rej, "ok", false);
                    gr::lora::cbor::kv_text(rej, "error", "channel_busy");
                    udp.sendTo(rej, req->sender);
                    continue;
                }
            }
            handle_tx_request(req->msg, cfg, udp, req->sender,
                              tx_source, tx_spectrum_ref.get());
        }
    });

    // --- Main loop: recv UDP datagrams, dispatch TX requests ---
    std::vector<uint8_t> recv_buf(65536);
    auto last_status = std::chrono::steady_clock::now();

    while (g_running && !rx_done.load(std::memory_order_relaxed)) {
        struct sockaddr_storage sender{};
        socklen_t slen = sizeof(sender);
        auto n = ::recvfrom(udp.fd, recv_buf.data(), recv_buf.size(), 0,
                            reinterpret_cast<struct sockaddr*>(&sender), &slen);

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Periodic status heartbeat
                if (cfg.status_interval > 0) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - last_status).count();
                    if (elapsed >= static_cast<int64_t>(cfg.status_interval)) {
                        // Sync overflow counter from source block
                        if (overflow_ptr) {
                            shared_status.overflow_count.store(
                                *overflow_ptr, std::memory_order_relaxed);
                        }
                        auto status_msg = build_status_cbor(cfg, shared_status);
                        udp.broadcast_all(status_msg);
                        last_status = now;
                    }
                }
                // Spectrum FFT: compute and broadcast if new data available
                if (spectrum->compute()) {
                    auto spec_msg = build_spectrum_cbor(*spectrum, "spectrum");
                    udp.broadcast_all(spec_msg);
                }
                if (tx_spectrum->compute()) {
                    auto spec_msg = build_spectrum_cbor(*tx_spectrum, "spectrum_tx");
                    udp.broadcast_all(spec_msg);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            break;
        }

        if (n == 0) {
            udp.subscribe(sender);
            udp.sendTo(config_cbor, sender);
            continue;
        }

        // Try to decode as CBOR message
        try {
            auto msg = gr::lora::cbor::decode_map(
                std::span<const uint8_t>(recv_buf.data(),
                                         static_cast<std::size_t>(n)));
            auto type = gr::lora::cbor::get_text_or(msg, "type", "");
            if (type == "lora_tx") {
                udp.subscribe(sender);
                if (!tx_queue.push({msg, sender})) {
                    // Queue full — reject immediately.
                    uint64_t seq = gr::lora::cbor::get_uint_or(msg, "seq", 0);
                    std::vector<uint8_t> rej;
                    rej.reserve(80);
                    gr::lora::cbor::encode_map_begin(rej, 4);
                    gr::lora::cbor::kv_text(rej, "type", "lora_tx_ack");
                    gr::lora::cbor::kv_uint(rej, "seq", seq);
                    gr::lora::cbor::kv_bool(rej, "ok", false);
                    gr::lora::cbor::kv_text(rej, "error", "tx_queue_full");
                    udp.sendTo(rej, sender);
                }
            } else if (type == "lora_config") {
                udp.subscribe(sender);
                handle_lora_config(msg, cfg, rx_blocks, udp, sender);
                // Rebuild config_cbor so new subscribers get the updated config
                config_cbor = build_config_cbor(cfg);
            } else if (type == "subscribe") {
                // Optional sync_word filter — array of uint16 or single uint
                std::vector<uint16_t> filter;
                try {
                    auto arr = gr::lora::cbor::get_uint_array_or(msg, "sync_word");
                    for (auto val : arr) filter.push_back(static_cast<uint16_t>(val));
                } catch (const gr::lora::cbor::DecodeError&) {
                    // Fallback: single uint value
                    auto sw = gr::lora::cbor::get_uint_or(msg, "sync_word", 0);
                    if (sw > 0) filter.push_back(static_cast<uint16_t>(sw));
                }
                udp.subscribe(sender, std::move(filter));
                udp.sendTo(config_cbor, sender);
            } else {
                udp.subscribe(sender);
                udp.sendTo(config_cbor, sender);
            }
        } catch (const gr::lora::cbor::DecodeError&) {
            // Not a valid CBOR map — legacy registration datagram (e.g. "sub")
            udp.subscribe(sender);
            udp.sendTo(config_cbor, sender);
        }
    }

    // --- Shutdown (order matters: stop RX first, then TX worker, then TX graph) ---
    gr::lora::log_ts("info ", "lora_trx", "stopping");
    if (!rx_done.load(std::memory_order_relaxed)) {
        rx_sched.requestStop();
    }
    rx_thread.join();

    // Drain TX request queue: signal stop, then join the worker thread.
    // The worker finishes any in-flight TX before exiting.
    tx_queue.request_stop();
    tx_worker.join();

    // Stop TX graph after worker is done (no more IQ will be pushed).
    tx_sched->requestStop();
    tx_source.notifyProgress();  // wake the scheduler so it sees REQUESTED_STOP
    tx_thread.join();

    ::close(udp.fd);
    gr::lora::log_ts("info ", "lora_trx", "stopped");

    // Skip static destructors and atexit handlers — GR4 scheduler and/or
    // SoapySDR unloadModules() hang during normal exit()/return-from-main.
    // quick_exit() is the C++11 standard mechanism for this; it runs
    // at_quick_exit handlers but not atexit/static dtors.
    std::quick_exit(0);
}
