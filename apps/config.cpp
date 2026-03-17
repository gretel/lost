// SPDX-License-Identifier: ISC
//
// Configuration loading and CBOR serialisation — implementation.

#include "config.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <map>
#include <ranges>
#include <string>

#include <toml++/toml.hpp>

#include <gnuradio-4.0/lora/log.hpp>

namespace lora_config {

// --- Property map utilities (internal) ---

// TOML has only int64_t, double, bool, string. GR4 blocks need exact types
// (uint8_t, float, etc.) for applyStagedParameters. We store floats as float
// (not double) since most block properties use float, and narrow integers to
// the smallest fitting unsigned type.
gr::property_map toml_to_property_map(const toml::table& tbl) {
    gr::property_map props;
    for (const auto& [key, val] : tbl) {
        std::pmr::string k(key.str());
        if (val.is_floating_point()) {
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

void merge_properties(gr::property_map& base, const gr::property_map& overrides) {
    for (const auto& [key, val] : overrides) {
        base.insert_or_assign(std::pmr::string(std::string(key)), val);
    }
}

// --- CLI ---

void print_usage() {
    std::fprintf(stderr,
        "Usage: lora_trx [options]\n\n"
        "Full-duplex LoRa transceiver with CBOR-over-UDP I/O.\n\n"
        "Options:\n"
        "  --config <file>       TOML configuration file (default: config.toml)\n"
        "  --log-level <level>   Log level: DEBUG, INFO, WARNING, ERROR\n"
        "                        Overrides [logging] level in config.toml.\n"
        "                        DEBUG also enables verbose decoder traces.\n"
        "  --version             Show version and exit\n"
        "  -h, --help            Show this help\n");
}

int parse_args(int argc, char* argv[], std::string& config_path,
               std::string& log_level) {
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
        if (arg == "--log-level" && i + 1 < argc) {
            log_level = argv[++i];
            // Uppercase for consistency
            for (auto& ch : log_level) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
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

// --- TOML config loader ---

std::vector<TrxConfig> load_config(const std::string& path,
                                   const std::string& cli_log_level) {
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
    bool        g_debug      = (cli_log_level == "DEBUG");

    // --- [aggregator] section ---
    auto* agg_tbl = tbl["aggregator"].as_table();
    RawConfig::Aggregator g_agg;
    if (agg_tbl) {
        g_agg.upstream  = agg_tbl->at_path("upstream").value_or(std::string{"127.0.0.1:5556"});
        g_agg.listen    = agg_tbl->at_path("listen").value_or(std::string{"127.0.0.1:5555"});
        g_agg.window_ms = static_cast<uint32_t>(agg_tbl->at_path("window_ms").value_or(int64_t{200}));
    }

    // --- [meshcore] section ---
    auto* mc_tbl = tbl["meshcore"].as_table();
    RawConfig::MeshCore g_meshcore;
    if (mc_tbl) {
        g_meshcore.name           = mc_tbl->at_path("name").value_or(std::string{});
        g_meshcore.model          = mc_tbl->at_path("model").value_or(std::string{});
        g_meshcore.identity_file  = mc_tbl->at_path("identity_file").value_or(std::string{});
        g_meshcore.keys_dir       = mc_tbl->at_path("keys_dir").value_or(std::string{});
        g_meshcore.channels_dir   = mc_tbl->at_path("channels_dir").value_or(std::string{});
        g_meshcore.contacts_dir   = mc_tbl->at_path("contacts_dir").value_or(std::string{});
        g_meshcore.lat            = mc_tbl->at_path("lat").value_or(0.0);
        g_meshcore.lon            = mc_tbl->at_path("lon").value_or(0.0);
        g_meshcore.port           = static_cast<uint16_t>(mc_tbl->at_path("port").value_or(int64_t{7834}));
        g_meshcore.region_scope   = mc_tbl->at_path("region_scope").value_or(std::string{});
        g_meshcore.flood_scope    = mc_tbl->at_path("flood_scope").value_or(std::string{});
        g_meshcore.client_repeat  = static_cast<uint32_t>(mc_tbl->at_path("client_repeat").value_or(int64_t{0}));
        g_meshcore.path_hash_mode = static_cast<uint32_t>(mc_tbl->at_path("path_hash_mode").value_or(int64_t{0}));
        g_meshcore.autoadd_config = static_cast<uint32_t>(mc_tbl->at_path("autoadd_config").value_or(int64_t{0}));
        g_meshcore.autoadd_max_hops = static_cast<uint32_t>(mc_tbl->at_path("autoadd_max_hops").value_or(int64_t{0}));
    }

    // --- [logging] section ---
    auto* log_tbl = tbl["logging"].as_table();
    RawConfig::Logging g_logging;
    if (log_tbl) {
        g_logging.level = log_tbl->at_path("level").value_or(std::string{"INFO"});
        g_logging.color = log_tbl->at_path("color").value_or(true);
    }
    // CLI --log-level overrides TOML
    if (!cli_log_level.empty()) {
        g_logging.level = cli_log_level;
    }
    // Apply resolved log level globally (affects all log_ts calls)
    gr::lora::set_log_level(g_logging.level);

    // --- Collect codec and radio sections ---
    struct CodecCfg {
        uint8_t  sf{8};
        uint32_t bw{62'500};
        uint8_t  cr{4};
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
            c.sf       = static_cast<uint8_t>(section["sf"].value_or(int64_t{8}));
            c.bw       = static_cast<uint32_t>(section["bw"].value_or(int64_t{62'500}));
            c.preamble = static_cast<uint16_t>(section["preamble_len"].value_or(int64_t{8}));
            c.sync     = static_cast<uint16_t>(section["sync_word"].value_or(int64_t{0x12}));

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

        // Raw config for Python scripts
        cfg.raw.network.host            = g_listen;
        cfg.raw.network.port            = static_cast<uint16_t>(g_port);
        cfg.raw.network.status_interval = static_cast<uint32_t>(g_status_int);
        cfg.raw.network.lbt             = g_lbt;
        cfg.raw.network.lbt_timeout_ms  = static_cast<uint32_t>(g_lbt_tmo);
        cfg.raw.network.tx_queue_depth  = static_cast<uint32_t>(g_tx_qdepth);
        cfg.raw.aggregator = g_agg;
        cfg.raw.meshcore   = g_meshcore;
        cfg.raw.logging    = g_logging;

        // Parse [[set_X.decode]] array
        if (auto* decode_arr = section["decode"].as_array()) {
            for (auto& elem : *decode_arr) {
                if (!elem.is_table()) continue;
                auto& dtbl = *elem.as_table();
                DecodeConfig dc;

                auto sf_opt = dtbl["sf"].value<int64_t>();
                dc.sf = sf_opt ? static_cast<uint8_t>(*sf_opt) : cfg.sf;

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

                if (!dc.block_overrides.empty()) {
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
                "(sync_word, label are required; sf is optional)",
                skey.c_str(), skey.c_str());
            return {};
        }

        // Parse decode_bws (optional: list of BWs to decode simultaneously)
        if (auto* bw_arr = section["decode_bws"].as_array()) {
            for (auto& elem : *bw_arr) {
                if (auto v = elem.value<int64_t>()) {
                    cfg.decode_bws.push_back(static_cast<uint32_t>(*v));
                }
            }
        }
        if (cfg.decode_bws.empty()) {
            cfg.decode_bws.push_back(cfg.bw);  // default: just the codec BW
        }

        {
            std::string rx_list;
            for (std::size_t i = 0; i < cfg.rx_channels.size(); i++) {
                if (i > 0) rx_list += ",";
                rx_list += std::to_string(cfg.rx_channels[i]);
                rx_list += "/";
                rx_list += kChannelMap[cfg.rx_channels[i]].label;
            }
            std::string dec_list;
            for (std::size_t i = 0; i < cfg.decode_configs.size(); i++) {
                if (i > 0) dec_list += ",";
                const auto& dc = cfg.decode_configs[i];
                char tmp[32];
                std::snprintf(tmp, sizeof(tmp), "SF7-12/0x%02X", dc.sync_word);
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

// --- Scan config loader ---

std::vector<ScanSetConfig> load_scan_config(const std::string& path,
                                            const std::string& cli_log_level) {
    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error& err) {
        gr::lora::log_ts("error", "config", "%s: %s",
                         path.c_str(), err.description().data());
        return {};
    }

    // --- [device] section ---
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
    std::string g_clock  = dev_or_s("clock", "");

    // Scan-specific device keys
    double g_l1_rate      = dev_or_d("l1_rate", 16.0e6);
    double g_master_clock = dev_or_d("master_clock", 32.0e6);

    // --- [logging] section ---
    auto* log_tbl = tbl["logging"].as_table();
    std::string log_level = "INFO";
    if (log_tbl) {
        log_level = log_tbl->at_path("level").value_or(std::string{"INFO"});
    }
    if (!cli_log_level.empty()) {
        log_level = cli_log_level;
    }
    gr::lora::set_log_level(log_level);

    // --- Collect radio sections ---
    struct RadioCfg {
        double freq{866'500'000.0};
        double rx_gain{40.0};
    };
    std::map<std::string, RadioCfg> radios;
    for (auto&& [key, val] : tbl) {
        if (!val.is_table()) continue;
        std::string skey(key.str());
        if (!skey.starts_with("radio_")) continue;
        auto& section = *val.as_table();
        RadioCfg r;
        r.freq    = section["freq"].value_or(866'500'000.0);
        r.rx_gain = section["rx_gain"].value_or(40.0);
        radios[skey] = r;
    }

    // --- Parse [set_*] sections with scan keys ---
    std::vector<std::pair<std::string, toml::source_position>> set_keys;
    for (auto&& [key, val] : tbl) {
        if (!val.is_table()) continue;
        std::string skey(key.str());
        if (!skey.starts_with("set_")) continue;
        // Scan sets have freq_start; TRX sets have codec — skip TRX sets
        if (val.as_table()->contains("codec")) continue;
        set_keys.emplace_back(skey, val.source().begin);
    }
    std::sort(set_keys.begin(), set_keys.end(),
              [](const auto& a, const auto& b) { return a.second.line < b.second.line; });

    std::vector<ScanSetConfig> configs;

    for (const auto& [skey, _pos] : set_keys) {
        auto& section = *tbl[skey].as_table();

        // Radio reference (optional for scan)
        RadioCfg radio;
        if (auto radio_ref = section["radio"].value<std::string>()) {
            auto rit = radios.find(*radio_ref);
            if (rit == radios.end()) {
                gr::lora::log_ts("error", "config",
                    "[%s] references unknown radio '%s'",
                    skey.c_str(), radio_ref->c_str());
                return {};
            }
            radio = rit->second;
        }

        ScanSetConfig cfg;

        // Device
        cfg.device       = g_device;
        cfg.device_param = g_param;
        cfg.clock        = g_clock;
        cfg.l1_rate      = section["l1_rate"].value_or(g_l1_rate);
        cfg.master_clock = section["master_clock"].value_or(g_master_clock);

        // Radio / set overrides
        cfg.gain       = section["rx_gain"].value_or(radio.rx_gain);
        cfg.freq_start = section["freq_start"].value_or(radio.freq);
        cfg.freq_stop  = section["freq_stop"].value_or(cfg.freq_start);

        if (cfg.freq_start >= cfg.freq_stop) {
            gr::lora::log_ts("error", "config",
                "[%s] freq_start (%.0f) must be < freq_stop (%.0f)",
                skey.c_str(), cfg.freq_start, cfg.freq_stop);
            return {};
        }

        // Scan keys
        cfg.os_factor  = static_cast<uint32_t>(section["os_factor"].value_or(int64_t{4}));
        cfg.min_ratio  = static_cast<float>(section["min_ratio"].value_or(8.0));
        cfg.settle_ms  = static_cast<int>(section["settle_ms"].value_or(int64_t{5}));
        cfg.sweeps     = static_cast<uint32_t>(section["sweeps"].value_or(int64_t{0}));
        cfg.layer1_only = section["layer1_only"].value_or(false);

        // BWs
        cfg.bws.clear();
        if (auto* bw_arr = section["bws"].as_array()) {
            for (auto& elem : *bw_arr) {
                if (auto v = elem.value<int64_t>()) {
                    cfg.bws.push_back(static_cast<uint32_t>(*v));
                }
            }
        }
        if (cfg.bws.empty()) {
            cfg.bws = {62500, 125000, 250000};  // default EU868
        }
        std::ranges::sort(cfg.bws);

        // Validate decimation factors
        for (uint32_t bw : cfg.bws) {
            const double targetRate = static_cast<double>(bw) * static_cast<double>(cfg.os_factor);
            const double factor     = cfg.l1_rate / targetRate;
            const double rounded    = std::round(factor);
            if (std::abs(factor - rounded) > 0.01 || rounded < 1.0) {
                gr::lora::log_ts("error", "config",
                    "[%s] l1_rate %.0f / (BW %u x os %u) = %.3f -- must be integer",
                    skey.c_str(), cfg.l1_rate, bw, cfg.os_factor, factor);
                return {};
            }
        }

        {
            std::string bw_list;
            for (std::size_t i = 0; i < cfg.bws.size(); i++) {
                if (i > 0) bw_list += ",";
                if (cfg.bws[i] >= 1000) bw_list += std::to_string(cfg.bws[i] / 1000) + "k";
                else bw_list += std::to_string(cfg.bws[i]);
            }
            gr::lora::log_ts("info ", "config",
                "scan set '%s': %.3f-%.3f MHz  BW=[%s]  os=%u  L1=%.1f MS/s",
                skey.c_str(),
                cfg.freq_start / 1e6, cfg.freq_stop / 1e6,
                bw_list.c_str(), cfg.os_factor, cfg.l1_rate / 1e6);
        }

        configs.push_back(std::move(cfg));
    }

    if (configs.empty()) {
        gr::lora::log_ts("error", "config",
            "no scan [set_*] sections found (must have freq_start, no codec)");
        return {};
    }

    return configs;
}

// --- CBOR serialisation ---

// Helper: count non-empty string fields to determine map size for optional keys.
static uint32_t count_meshcore_fields(const RawConfig::MeshCore& mc) {
    uint32_t n = 0;
    if (!mc.name.empty()) n++;
    if (!mc.model.empty()) n++;
    if (!mc.identity_file.empty()) n++;
    if (!mc.keys_dir.empty()) n++;
    if (!mc.channels_dir.empty()) n++;
    if (!mc.contacts_dir.empty()) n++;
    if (mc.lat != 0.0 || mc.lon != 0.0) n += 2;  // lat + lon
    n++;  // port (always present)
    if (!mc.region_scope.empty()) n++;
    if (!mc.flood_scope.empty()) n++;
    n++;  // client_repeat
    n++;  // path_hash_mode
    n++;  // autoadd_config
    n++;  // autoadd_max_hops
    return n;
}

std::vector<uint8_t> build_config_cbor(const TrxConfig& cfg) {
    namespace cbor = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(512);

    cbor::encode_map_begin(buf, 5);  // type + phy + server + decode_chains + raw
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

    // Raw passthrough sections for Python scripts
    const auto& raw = cfg.raw;
    cbor::encode_text(buf, "raw");
    cbor::encode_map_begin(buf, 4);  // network + aggregator + meshcore + logging

    // raw.network
    cbor::encode_text(buf, "network");
    cbor::encode_map_begin(buf, 6);
    cbor::kv_text(buf, "udp_listen", raw.network.host);
    cbor::kv_uint(buf, "udp_port", raw.network.port);
    cbor::kv_uint(buf, "status_interval", raw.network.status_interval);
    cbor::kv_bool(buf, "lbt", raw.network.lbt);
    cbor::kv_uint(buf, "lbt_timeout_ms", raw.network.lbt_timeout_ms);
    cbor::kv_uint(buf, "tx_queue_depth", raw.network.tx_queue_depth);

    // raw.aggregator
    cbor::encode_text(buf, "aggregator");
    cbor::encode_map_begin(buf, 3);
    cbor::kv_text(buf, "upstream", raw.aggregator.upstream);
    cbor::kv_text(buf, "listen", raw.aggregator.listen);
    cbor::kv_uint(buf, "window_ms", raw.aggregator.window_ms);

    // raw.meshcore
    cbor::encode_text(buf, "meshcore");
    auto mc_fields = count_meshcore_fields(raw.meshcore);
    cbor::encode_map_begin(buf, mc_fields);
    if (!raw.meshcore.name.empty()) cbor::kv_text(buf, "name", raw.meshcore.name);
    if (!raw.meshcore.model.empty()) cbor::kv_text(buf, "model", raw.meshcore.model);
    if (!raw.meshcore.identity_file.empty()) cbor::kv_text(buf, "identity_file", raw.meshcore.identity_file);
    if (!raw.meshcore.keys_dir.empty()) cbor::kv_text(buf, "keys_dir", raw.meshcore.keys_dir);
    if (!raw.meshcore.channels_dir.empty()) cbor::kv_text(buf, "channels_dir", raw.meshcore.channels_dir);
    if (!raw.meshcore.contacts_dir.empty()) cbor::kv_text(buf, "contacts_dir", raw.meshcore.contacts_dir);
    if (raw.meshcore.lat != 0.0 || raw.meshcore.lon != 0.0) {
        cbor::kv_float64(buf, "lat", raw.meshcore.lat);
        cbor::kv_float64(buf, "lon", raw.meshcore.lon);
    }
    cbor::kv_uint(buf, "port", raw.meshcore.port);
    if (!raw.meshcore.region_scope.empty()) cbor::kv_text(buf, "region_scope", raw.meshcore.region_scope);
    if (!raw.meshcore.flood_scope.empty()) cbor::kv_text(buf, "flood_scope", raw.meshcore.flood_scope);
    cbor::kv_uint(buf, "client_repeat", raw.meshcore.client_repeat);
    cbor::kv_uint(buf, "path_hash_mode", raw.meshcore.path_hash_mode);
    cbor::kv_uint(buf, "autoadd_config", raw.meshcore.autoadd_config);
    cbor::kv_uint(buf, "autoadd_max_hops", raw.meshcore.autoadd_max_hops);

    // raw.logging
    cbor::encode_text(buf, "logging");
    cbor::encode_map_begin(buf, 2);
    cbor::kv_text(buf, "level", raw.logging.level);
    cbor::kv_bool(buf, "color", raw.logging.color);

    return buf;
}

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

std::vector<uint8_t> build_spectrum_cbor(gr::lora::SpectrumState& spec,
                                         const char* type_name) {
    namespace cbor = gr::lora::cbor;
    std::vector<uint8_t> buf;
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

}  // namespace lora_config
