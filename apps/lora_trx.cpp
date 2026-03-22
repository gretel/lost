// SPDX-License-Identifier: ISC
//
// lora_trx: full-duplex LoRa transceiver using native GR4 SoapySDR blocks.
//
// 1-ch RX: SoapySimpleSource -> [Splitter] -> MultiSfDecoder (per BW) -> FrameSink
// 2-ch RX: SoapyDualSimpleSource -+-> [Splitter] -> MultiSfDecoder (per BW) -> FrameSink
//                                 +-> [Splitter] -> MultiSfDecoder (per BW) -> FrameSink
// TX: bounded request queue with dedicated worker thread; LBT defers TX until channel clear
//
// MultiSfDecoder decodes all SFs (7-12) simultaneously on each channel.
// Full-duplex TX in both modes: RX runs continuously while TX uses an
// ephemeral graph with a separate device handle.
//
// All I/O is CBOR over UDP. RX frames fan out to all connected clients.
// TX requests (type: "lora_tx") are processed and acked.

#include "config.hpp"

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
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/soapy/SoapySink.hpp>

#include "graph_builder.hpp"
#include "udp_state.hpp"

#include <gnuradio-4.0/lora/FrameSink.hpp>
#include <gnuradio-4.0/lora/TxQueueSource.hpp>
#include <gnuradio-4.0/lora/algorithm/Telemetry.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/log.hpp>

using namespace lora_config;
using namespace lora_graph;

namespace {


using lora_apps::UdpState;


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
        std::ignore = graph.connect(src, "out0"s, sink, "in#0"s);
        std::ignore = graph.connect(src, "out1"s, sink, "in#1"s);
    } else {
        // zeros -> sink.in#0 (ch0=TRX_A), IQ -> sink.in#1 (ch1=TRX_B)
        std::ignore = graph.connect(src, "out1"s, sink, "in#0"s);
        std::ignore = graph.connect(src, "out0"s, sink, "in#1"s);
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
    constexpr std::size_t kMaxHexBytes = 32;
    auto hex_len = std::min(payload.size(), kMaxHexBytes);
    auto hex = gr::lora::FrameSink::to_hex(
        std::span<const uint8_t>(payload.data(), hex_len));
    char repeat_str[24] = "";
    if (repeat > 1) std::snprintf(repeat_str, sizeof(repeat_str), " repeat=%d", repeat);
    gr::lora::log_ts("info ", "lora_trx",
        "TX bytes=%zu sf=%u cr=4/%u sync=0x%02X airtime=%.1fms%s%s  %s%s",
        payload.size(), cfg.sf, 4u + cr, sync,
        airtime * 1000.0, repeat_str, dry_run ? " dry" : "",
        hex.c_str(), payload.size() > kMaxHexBytes ? "..." : "");

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

// Convert CBOR lora_config values to a property_map suitable for
// MultiSfDecoder settings.  CBOR uint -> narrowed to the exact
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
                        const struct sockaddr_storage& sender,
                        const std::string& device_serial) {
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

    // Apply PHY parameters to all MultiSfDecoder blocks
    if (!phy_props.empty()) {
        for (auto& blk : rx_blocks.multisf_decoders) {
            auto rejected = blk->settings().set(phy_props);
            if (!rejected.empty()) {
                gr::lora::log_ts("warn ", "lora_trx",
                    "lora_config: MultiSfDecoder rejected %zu key(s)", rejected.size());
                any_rejected = true;
            }
        }
    }

    // Retune SoapySource center frequency
    if (freq_changed && rx_blocks.soapy_source) {
        gr::property_map freq_props;
        freq_props["rx_center_frequency"] = gr::Tensor<double>{new_freq};
        auto rejected = rx_blocks.soapy_source->settings().set(freq_props);
        if (!rejected.empty()) {
            gr::lora::log_ts("warn ", "lora_trx",
                "lora_config: SoapySource rejected freq change");
        }
        for (auto& blk : rx_blocks.multisf_decoders) {
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
    auto config_cbor = build_config_cbor(cfg, device_serial);
    udp.broadcast_all(config_cbor);

    return ok;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string config_path;
    std::string log_level;
    int rc = parse_args(argc, argv, config_path, log_level);
    if (rc != 0) {
        return rc == 2 ? 0 : 1;  // 2 = --help/--version (clean exit)
    }

    auto configs = load_config(config_path, log_level);
    if (configs.empty()) {
        return 1;
    }

    // Use first set for now (multi-set support is future work)
    TrxConfig cfg = std::move(configs[0]);

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

    auto& stop_flag = lora_apps::install_signal_handler();

    lora_apps::apply_fpga_workaround(cfg.device, cfg.device_param);

    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    lora_apps::log_version_banner("lora_trx", GIT_REV);
    gr::lora::log_ts("info ", "lora_trx",
        "config %s (%zu set%s)  set '%s'",
        config_path.c_str(), configs.size(),
        configs.size() > 1 ? "s" : "",
        cfg.name.c_str());
    gr::lora::log_ts("info ", "lora_trx",
        "device %s  param %s",
        cfg.device.c_str(),
        cfg.device_param.empty() ? "(none)" : cfg.device_param.c_str());

    auto device_serial = lora_apps::log_hardware_info("lora_trx", cfg.device, cfg.device_param);

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
    gr::lora::log_ts("debug", "lora_trx", "debug output enabled");
    if (cfg.status_interval > 0) {
        gr::lora::log_ts("info ", "lora_trx",
            "status heartbeat every %u s", cfg.status_interval);
    }

    // --- Bind UDP socket ---
    UdpState udp;
    udp.fd = lora_apps::create_udp_socket(cfg.listen, cfg.port);
    if (udp.fd < 0) return 1;

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
    //
    // Wideband mode skips the TX graph: the B210 AD9361 limits master clock to
    // 30.72 MHz with 2 TX channels, but 16 MS/s RX needs 32 MHz (32/16 = 2).
    // TX support in wideband mode requires single-channel TX (future work).
    gr::lora::TxQueueSource* tx_source_ptr = nullptr;
    std::unique_ptr<gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking>> tx_sched;
    std::thread tx_thread;

    if (!cfg.wideband) {
        tx_sched = build_tx_graph(tx_source_ptr, cfg);
        if (!tx_sched || tx_source_ptr == nullptr) {
            gr::lora::log_ts("error", "lora_trx", "failed to build TX graph");
            ::close(udp.fd);
            return 1;
        }
        tx_thread = std::thread([&tx_sched]() {
            auto ret = tx_sched->runAndWait();
            if (!ret.has_value()) {
                gr::lora::log_ts("warn ", "lora_trx",
                    "TX scheduler stopped: %s",
                    std::format("{}", ret.error()).c_str());
            }
        });
    } else {
        gr::lora::log_ts("info ", "lora_trx", "wideband mode: TX disabled (AD9361 2-ch clock limit)");
    }

    // --- Spectrum taps for waterfall display ---
    auto spectrum = std::make_shared<gr::lora::SpectrumState>();
    spectrum->sample_rate = cfg.rate;
    spectrum->center_freq = cfg.freq;
    spectrum->init();

    // TX spectrum: fed by handle_tx_request before transmit
    auto tx_spectrum = std::make_shared<gr::lora::SpectrumState>();
    tx_spectrum->sample_rate = cfg.rate;
    tx_spectrum->center_freq = cfg.freq;
    tx_spectrum->init();

    // singleThreadedBlocking doesn't use the pool; shrink to avoid idle thread CPU waste
    gr::thread_pool::Manager::defaultCpuPool()->setThreadBounds(1, 1);

    // --- LBT: channel-busy flag updated by CAD in the RX graph ---
    std::atomic<bool> channel_busy{false};

    // --- Build and start RX graph in a background thread ---
    gr::Graph rx_graph;
    std::atomic<uint64_t>* overflow_ptr = nullptr;
    if (cfg.wideband) {
        gr::lora::log_ts("info ", "lora_trx", "wideband mode: %.1f MS/s, digital channelization",
                         cfg.wideband_rate / 1e6);
        auto wb_telemetry = [&udp](const gr::property_map& evt) {
            udp.broadcast_all(gr::lora::telemetry::encode(evt));
        };
        overflow_ptr = build_wideband_graph(rx_graph, cfg, frame_callback,
                                            std::move(wb_telemetry));
    } else {
        overflow_ptr = build_rx_graph(rx_graph, cfg, frame_callback, spectrum,
                                      &channel_busy);
    }

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking> rx_sched;
    rx_sched.timeout_inactivity_count  = 1U;   // sleep after 1 idle cycle; SoapySource notifies progress on data
    rx_sched.watchdog_min_stall_count  = 10U;  // suppress watchdog for ≤10s gaps (normal inter-frame silence)
    rx_sched.watchdog_max_warnings     = 30U;  // 30s of continuous stall → ERROR
    if (auto ret = rx_sched.exchange(std::move(rx_graph)); !ret) {
        gr::lora::log_ts("error", "lora_trx", "RX scheduler init failed");
        if (tx_sched) {
            tx_sched->requestStop();
            if (tx_thread.joinable()) tx_thread.join();
        }
        ::close(udp.fd);
        return 1;
    }

    // Locate RX blocks in the live scheduler graph for runtime reconfig.
    // After exchange(), the original emplaceBlock references are dangling —
    // we find them by scanning typeName().
    auto rx_blocks = find_rx_blocks(rx_sched.blocks());
    set_device_serial(rx_sched.blocks(), device_serial);
    gr::lora::log_ts("info ", "lora_trx",
        "RX blocks: %zu MultiSfDecoder (SF7-12)  soapy=%s",
        rx_blocks.multisf_decoders.size(),
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
    auto config_cbor = build_config_cbor(cfg, device_serial);

    // --- TX request queue + worker thread (replaces tx_busy + io pool dispatch) ---
    TxRequestQueue tx_queue(cfg.tx_queue_depth);
    std::thread tx_worker;

    if (tx_source_ptr != nullptr) {
        gr::lora::TxQueueSource& tx_source = *tx_source_ptr;
        tx_worker = std::thread([&tx_queue, &cfg, &udp, &tx_source, &channel_busy,
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
    }

    // --- Main loop: recv UDP datagrams, dispatch TX requests ---
    std::vector<uint8_t> recv_buf(65536);
    auto last_status = std::chrono::steady_clock::now();

    while (!stop_flag.load(std::memory_order_relaxed) && !rx_done.load(std::memory_order_relaxed)) {
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
                handle_lora_config(msg, cfg, rx_blocks, udp, sender, device_serial);
                // Rebuild config_cbor so new subscribers get the updated config
                config_cbor = build_config_cbor(cfg, device_serial);
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
    if (tx_worker.joinable()) tx_worker.join();

    // Stop TX graph after worker is done (no more IQ will be pushed).
    if (tx_sched) {
        tx_sched->requestStop();
        tx_source_ptr->notifyProgress();  // wake the scheduler so it sees REQUESTED_STOP
        tx_thread.join();
    }

    ::close(udp.fd);
    gr::lora::log_ts("info ", "lora_trx", "stopped");

    // Skip static destructors and atexit handlers — GR4 scheduler and/or
    // SoapySDR unloadModules() hang during normal exit()/return-from-main.
    // quick_exit() is the C++11 standard mechanism for this; it runs
    // at_quick_exit handlers but not atexit/static dtors.
    std::quick_exit(0);
}
