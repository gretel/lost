// SPDX-License-Identifier: ISC
//
// Configuration structs, TOML parser, and CBOR serialisation for lora apps.
//
// Extracted from lora_trx.cpp so that config handling can be tested and
// reused independently.  The `RawConfig` struct carries passthrough TOML
// sections (network, aggregator, meshcore, logging) for CBOR distribution
// to Python scripts that no longer read config.toml directly.

#ifndef GR4_LORA_CONFIG_HPP
#define GR4_LORA_CONFIG_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>          // property_map
#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>

namespace lora_config {

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

// Passthrough config sections for Python scripts.
// Mirrors the TOML section structure so Python gets the same nested dict
// it would have gotten from parsing config.toml directly.
struct RawConfig {
    // [network] — UDP addressing (scripts resolve listen addresses from here)
    struct Network {
        std::string  host{"127.0.0.1"};
        uint16_t     port{5556};
        uint32_t     status_interval{10};
        bool         lbt{true};
        uint32_t     lbt_timeout_ms{2000};
        uint32_t     tx_queue_depth{4};
    } network;

    // [aggregator] — lora_agg.py settings
    struct Aggregator {
        std::string  upstream{"127.0.0.1:5556"};
        std::string  listen{"127.0.0.1:5555"};
        uint32_t     window_ms{200};
    } aggregator;

    // [meshcore] — identity, location, flood scope (Python meshcore_bridge.py)
    struct MeshCore {
        std::string  name{};
        std::string  model{};
        std::string  identity_file{};
        std::string  keys_dir{};
        std::string  channels_dir{};
        std::string  contacts_dir{};
        double       lat{0.0};
        double       lon{0.0};
        uint16_t     port{7834};
        std::string  region_scope{};
        std::string  flood_scope{};
        uint32_t     client_repeat{0};
        uint32_t     path_hash_mode{0};
        uint32_t     autoadd_config{0};
        uint32_t     autoadd_max_hops{0};
    } meshcore;

    // [logging] — Python log level and color
    struct Logging {
        std::string  level{"INFO"};
        bool         color{true};
    } logging;
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
    std::vector<uint32_t>     decode_bws{};      ///< BWs to decode (empty = {bw} only)
    RawConfig                 raw{};             ///< passthrough sections for Python scripts
};

/// Scan-specific configuration (parsed from [scan] section).
/// Shares [device] and [radio_*] with TrxConfig; adds scan parameters.
struct ScanSetConfig {
    // From [device]
    std::string  device{"uhd"};
    std::string  device_param{"type=b200"};
    double       l1_rate{16.0e6};           ///< L1 wideband sample rate
    double       master_clock{32.0e6};      ///< FPGA master clock rate
    std::string  clock{};

    // From [radio_*]
    double       freq_start{863.0e6};       ///< scan band start
    double       freq_stop{870.0e6};        ///< scan band stop
    double       gain{40.0};

    // From [scan]
    std::vector<uint32_t> bws{62500, 125000, 250000};  ///< BWs to probe
    uint32_t     os_factor{4};              ///< oversampling factor
    float        min_ratio{8.0F};           ///< CAD detection threshold
    int          settle_ms{5};              ///< PLL settle delay after retune (ms)
    uint32_t     sweeps{0};                 ///< 0 = infinite
    bool         layer1_only{false};
    bool         cbor_out{false};           ///< CBOR on stdout

    // From [scan.network]
    std::string  udp_listen{"127.0.0.1"};
    uint16_t     udp_port{5557};            ///< separate from lora_trx (5556)

    [[nodiscard]] double min_bw() const {
        return bws.empty() ? 62500.0
                           : static_cast<double>(*std::ranges::min_element(bws));
    }
};

// Shared RX status counters (updated by frame callback, read by main loop).
struct SharedStatus {
    std::atomic<uint32_t> frame_count{0};
    std::atomic<uint32_t> crc_ok{0};
    std::atomic<uint32_t> crc_fail{0};
    std::atomic<uint64_t> overflow_count{0};
};

// --- CLI + TOML config ---

void print_usage();

/// Parse CLI args.  Returns 0 on success, 1 on error, 2 on --help/--version.
/// `log_level` receives the --log-level value (empty if not specified).
int parse_args(int argc, char* argv[], std::string& config_path,
               std::string& log_level);

/// Load TOML config file into a vector of TrxConfig (one per set).
/// `cli_log_level` overrides [logging] level from TOML when non-empty.
/// "DEBUG" also enables block-level debug traces (cfg.debug = true).
/// Returns empty vector on error (messages printed to stderr).
std::vector<TrxConfig> load_config(const std::string& path,
                                   const std::string& cli_log_level = "");

/// Load TOML config file for scan mode.
/// Returns empty vector on error (messages printed to stderr).
std::vector<ScanSetConfig> load_scan_config(const std::string& path,
                                            const std::string& cli_log_level = "");

// --- Property map utilities ---

/// Merge override properties into a base property_map.
void merge_properties(gr::property_map& base, const gr::property_map& overrides);

/// Return only the entries whose key appears in `allowed`.
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
static constexpr std::array kFrameSyncOverrides = {"energy_thresh", "min_snr_db", "max_symbols"};
static constexpr std::array kDemodDecoderOverrides = {
    "impl_head", "impl_cr", "impl_pay_len", "impl_has_crc", "ldro_mode"};

// --- CBOR serialisation ---

/// Build a CBOR "config" message — sent once on subscribe.
/// Includes PHY, server, decode_chains, and raw passthrough sections.
std::vector<uint8_t> build_config_cbor(const TrxConfig& cfg);

/// Build a CBOR "status" heartbeat — sent periodically.
std::vector<uint8_t> build_status_cbor(const TrxConfig& cfg, SharedStatus& status);

/// Build a CBOR spectrum message from a SpectrumState result buffer.
std::vector<uint8_t> build_spectrum_cbor(gr::lora::SpectrumState& spec,
                                         const char* type_name = "spectrum");

}  // namespace lora_config

#endif  // GR4_LORA_CONFIG_HPP
