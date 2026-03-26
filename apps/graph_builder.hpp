// SPDX-License-Identifier: ISC
//
// Shared GR4 graph construction for lora_trx and lora_scan.
//
// - build_rx_graph():   SoapySource → [Splitter] → MultiSfDecoder → FrameSink
// - build_scan_graph(): SoapySource → Splitter → SpectrumTap + CaptureSink
// - find_rx_blocks():   locate typed blocks after scheduler exchange()

#ifndef GR4_LORA_GRAPH_BUILDER_HPP
#define GR4_LORA_GRAPH_BUILDER_HPP

#include "config.hpp"

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Tensor.hpp>

#include <gnuradio-4.0/lora/MultiSfDecoder.hpp>
#include <gnuradio-4.0/lora/WidebandDecoder.hpp>
#include <gnuradio-4.0/lora/Splitter.hpp>
#include <gnuradio-4.0/lora/FrameSink.hpp>
#include <gnuradio-4.0/lora/CaptureSink.hpp>
#include <gnuradio-4.0/lora/SpectrumTapBlock.hpp>
#include <gnuradio-4.0/lora/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/ScanController.hpp>
#include <gnuradio-4.0/lora/ScanSink.hpp>
#include <gnuradio-4.0/lora/log.hpp>

// Soapy.hpp MUST come before common.hpp (common.hpp uses soapy types)
#include <gnuradio-4.0/soapy/Soapy.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include "common.hpp"

#include <atomic>
#include <complex>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace lora_graph {

using lora_config::TrxConfig;
using lora_config::DecodeConfig;
using lora_config::ScanSetConfig;
using lora_config::kChannelMap;
using lora_apps::cf32;

// ─── RX decode graph (lora_trx) ──────────────────────────────────────────────

/// Creates a MultiSfDecoder → FrameSink chain.
/// Returns reference to the MultiSfDecoder (for connecting upstream).
inline gr::lora::MultiSfDecoder& add_multisf_chain(
        gr::Graph& graph, const TrxConfig& cfg,
        int32_t rx_channel, const DecodeConfig& dc,
        std::function<void(const std::vector<uint8_t>&, bool)> callback,
        std::shared_ptr<gr::lora::SpectrumState> spectrum = nullptr,
        std::function<void(const gr::property_map&)> telemetry_cb = {}) {
    auto os = static_cast<uint8_t>(cfg.rate / static_cast<float>(cfg.bw));

    auto& decoder = graph.emplaceBlock<gr::lora::MultiSfDecoder>();
    decoder.bandwidth    = cfg.bw;
    decoder.sync_word    = dc.sync_word;
    decoder.os_factor    = os;
    decoder.preamble_len = cfg.preamble;
    decoder.center_freq  = static_cast<uint32_t>(cfg.freq);
    decoder.rx_channel   = rx_channel;
    decoder.sf_min       = 7;
    decoder.sf_max       = 12;
    decoder.debug        = cfg.debug;
    decoder.soft_decode  = cfg.soft_decode;
    if (cfg.lo_offset > 0.0) {
        // LO offset shifts DC spur out of passband; reduce HP cutoff to
        // avoid attenuating low-bin sync word symbols (SF8/BW62.5k sw0 = 1953 Hz)
        decoder.dc_blocker_cutoff = cfg.dc_blocker ? 100.f : 0.f;
    } else {
        decoder.dc_blocker_cutoff = cfg.dc_blocker ? cfg.dc_blocker_cutoff : 0.f;
    }
    decoder._spectrum_state = std::move(spectrum);
    if (telemetry_cb) {
        decoder._telemetry = telemetry_cb;
    }

    // Apply block-level overrides from config
    if (auto it = dc.block_overrides.find("energy_thresh"); it != dc.block_overrides.end()) {
        decoder.energy_thresh = it->second.template value_or<float>(1e-6f);
    }
    if (auto it = dc.block_overrides.find("min_snr_db"); it != dc.block_overrides.end()) {
        decoder.min_snr_db = it->second.template value_or<float>(-10.f);
    }
    if (auto it = dc.block_overrides.find("max_symbols"); it != dc.block_overrides.end()) {
        decoder.max_symbols = static_cast<uint32_t>(it->second.template value_or<int64_t>(600));
    }

    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"sync_word", dc.sync_word},
        {"phy_sf", uint8_t{0}},  // MultiSf: SF varies per frame, reported in tag
        {"phy_bw", cfg.bw},
        {"label", dc.label},
    });
    sink._frame_callback = callback;

    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };
    if (!ok(graph.connect<"out", "in">(decoder, sink))) {
        gr::lora::log_ts("error", "graph", "connect MultiSfDecoder -> FrameSink failed (rx_ch %d)", rx_channel);
    }
    return decoder;
}

/// Generalised RX graph builder.
///
/// Topology (per radio channel, single BW):
///   Source → MultiSfDecoder (SF7-12) → FrameSink
///
/// Topology (per radio channel, multi-BW):
///   Source → Splitter(N) → MultiSfDecoder (BW1, SF7-12) → FrameSink
///                        → MultiSfDecoder (BW2, SF7-12) → FrameSink
///
/// Returns pointer to source block's cumulative overflow counter.
inline std::atomic<uint64_t>* build_rx_graph(
        gr::Graph& graph, const TrxConfig& cfg,
        std::function<void(const std::vector<uint8_t>&, bool)> callback,
        std::shared_ptr<gr::lora::SpectrumState> spectrum = nullptr,
        std::atomic<bool>* channel_busy = nullptr,
        std::function<void(const gr::property_map&)> telemetry_cb = {}) {
    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };
    using std::string_literals::operator""s;

    const auto& decodes = cfg.decode_configs;
    const auto nRadio = cfg.rx_channels.size();
    const auto& bws = cfg.decode_bws;
    const auto nBW = bws.size();

    // Translate config channels (0-3) to SoapySDR channels + antennae
    std::vector<gr::Size_t> soapy_channels;
    std::vector<std::string> antennae;
    for (auto ch : cfg.rx_channels) {
        const auto& map = kChannelMap[ch];
        soapy_channels.push_back(static_cast<gr::Size_t>(map.soapy_channel));
        antennae.emplace_back(map.rx_antenna);
    }

    const auto& dc = decodes[0];

    // Wire MultiSfDecoder(s) per radio channel via connectPort lambda.
    auto wireDecodeChains = [&](auto connectPort) {
        for (std::size_t r = 0; r < nRadio; r++) {
            bool firstRadio = (r == 0);

            if (nBW == 1) {
                auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100;
                auto spec = firstRadio ? spectrum : nullptr;

                TrxConfig bw_cfg = cfg;
                bw_cfg.bw = bws[0];
                auto& decoder = add_multisf_chain(graph, bw_cfg, rx_ch, dc, callback, spec, telemetry_cb);

                if (firstRadio && channel_busy != nullptr) {
                    decoder.set_channel_busy_flag(channel_busy);
                }
                if (!connectPort(r, decoder)) {
                    gr::lora::log_ts("error", "graph",
                        "connect source -> MultiSfDecoder failed (radio %zu)", r);
                }
            } else {
                auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
                    {"n_outputs", static_cast<gr::Size_t>(nBW)},
                });
                if (!connectPort(r, splitter)) {
                    gr::lora::log_ts("error", "graph",
                        "connect source -> Splitter failed (radio %zu)", r);
                }

                for (std::size_t b = 0; b < nBW; b++) {
                    auto rx_ch = static_cast<int32_t>(cfg.rx_channels[r]) * 100
                               + static_cast<int32_t>(b);
                    auto spec = (firstRadio && b == 0) ? spectrum : nullptr;

                    TrxConfig bw_cfg = cfg;
                    bw_cfg.bw = bws[b];
                    auto& decoder = add_multisf_chain(graph, bw_cfg, rx_ch, dc, callback, spec, telemetry_cb);

                    if (firstRadio && b == 0 && channel_busy != nullptr) {
                        decoder.set_channel_busy_flag(channel_busy);
                    }

                    auto port = "out#"s + std::to_string(b);
                    if (!ok(graph.connect(splitter, port, decoder, "in"s))) {
                        gr::lora::log_ts("error", "graph",
                            "connect Splitter -> MultiSfDecoder BW%u failed (radio %zu)", bws[b], r);
                    }
                }
            }
        }
    };

    // Create source and wire: 1 radio = SoapySimple, 2 radios = SoapyDual
    std::atomic<uint64_t>* overflow_ptr = nullptr;

    if (nRadio == 1) {
        auto props = lora_apps::soapy_reliability_defaults();
        props.merge(gr::property_map{
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
            {"rx_gains", gr::Tensor<double>{cfg.gain_rx}},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, soapy_channels)},
            {"rx_antennae", antennae},
            {"clock_source", cfg.clock},
            {"lo_offset", cfg.lo_offset},
            {"rx_dc_offset_auto", cfg.dc_offset_auto},
        });
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapySimpleSource<std::complex<float>>>(
            std::move(props));
        overflow_ptr = &source._totalOverFlowCount;
        wireDecodeChains([&graph, &source](std::size_t /*r*/, auto& downstream) {
            return graph.connect<"out", "in">(source, downstream).has_value();
        });
    } else {
        auto props = lora_apps::soapy_reliability_defaults();
        props.merge(gr::property_map{
            {"device", cfg.device},
            {"device_parameter", cfg.device_param},
            {"sample_rate", cfg.rate},
            {"rx_center_frequency", gr::Tensor<double>(gr::data_from, {cfg.freq, cfg.freq})},
            {"rx_gains", gr::Tensor<double>(gr::data_from, {cfg.gain_rx, cfg.gain_rx})},
            {"rx_channels", gr::Tensor<gr::Size_t>(gr::data_from, soapy_channels)},
            {"rx_antennae", antennae},
            {"clock_source", cfg.clock},
            {"lo_offset", cfg.lo_offset},
            {"rx_dc_offset_auto", cfg.dc_offset_auto},
        });
        auto& source = graph.emplaceBlock<gr::blocks::soapy::SoapyDualSimpleSource<std::complex<float>>>(
            std::move(props));
        overflow_ptr = &source._totalOverFlowCount;
        wireDecodeChains([&graph, &source](std::size_t r, auto& downstream) {
            auto portName = "out#"s + std::to_string(r);
            return graph.connect(source, portName, downstream, "in"s).has_value();
        });
    }

    {
        std::string bw_list;
        for (std::size_t i = 0; i < nBW; i++) {
            if (i > 0) bw_list += ",";
            if (bws[i] >= 1000) bw_list += std::to_string(bws[i] / 1000) + "k";
            else bw_list += std::to_string(bws[i]);
        }
        gr::lora::log_ts("debug", "graph",
            "RX graph: %zu radio(s) x %zu BW(s) [%s], MultiSfDecoder SF7-12",
            nRadio, nBW, bw_list.c_str());
    }

    return overflow_ptr;
}

// ─── Post-exchange block discovery ───────────────────────────────────────────

/// Typed block references found after scheduler exchange().
struct RxBlocks {
    std::vector<std::shared_ptr<gr::BlockModel>> multisf_decoders;
    std::shared_ptr<gr::BlockModel>              soapy_source;
};

/// Scan the scheduler's block list and collect references by type name.
inline RxBlocks find_rx_blocks(std::span<std::shared_ptr<gr::BlockModel>> blocks) {
    RxBlocks rx;
    for (auto& blk : blocks) {
        auto tn = blk->typeName();
        if (tn.find("MultiSfDecoder") != std::string_view::npos) {
            rx.multisf_decoders.push_back(blk);
        } else if (tn.find("Soapy") != std::string_view::npos) {
            rx.soapy_source = blk;
        }
    }
    return rx;
}

/// Find the SoapySource block in a scheduler's block list.
inline std::shared_ptr<gr::BlockModel> find_soapy_source(
        std::span<std::shared_ptr<gr::BlockModel>> blocks) {
    for (auto& blk : blocks) {
        if (blk->typeName().find("Soapy") != std::string_view::npos) {
            return blk;
        }
    }
    return nullptr;
}

/// Set `_device_serial` on all FrameSink blocks in the scheduler.
inline void set_device_serial(std::span<std::shared_ptr<gr::BlockModel>> blocks,
                              const std::string& serial) {
    if (serial.empty()) return;
    for (auto& blk : blocks) {
        if (blk->typeName().find("FrameSink") != std::string_view::npos) {
            auto* wrapper = dynamic_cast<gr::BlockWrapper<gr::lora::FrameSink>*>(blk.get());
            if (wrapper) {
                wrapper->blockRef()._device_serial = serial;
            }
        }
    }
}

// ─── Scan graph (lora_scan) ──────────────────────────────────────────────────

/// Shared state handles returned by build_scan_graph().
struct ScanGraph {
    std::shared_ptr<gr::lora::SpectrumState> spectrum;
    std::shared_ptr<gr::lora::CaptureState>  capture;
};

/// Build the scan graph: SoapySource → Splitter → SpectrumTap + CaptureSink
inline ScanGraph build_scan_graph(gr::Graph& graph, const ScanSetConfig& cfg,
                                  const std::vector<double>& channels) {
    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };
    using std::string_literals::operator""s;

    ScanGraph sg;

    // SoapySource -- fixed L1 rate, pinned master clock
    const double tileCentre = (channels.front() + channels.back()) / 2.0;
    auto source_props = lora_apps::soapy_reliability_defaults();
    source_props.merge(gr::property_map{
        {"device",              cfg.device},
        {"device_parameter",    cfg.device_param},
        {"sample_rate",         cfg.l1_rate},
        {"master_clock_rate",   cfg.master_clock},
        {"rx_center_frequency", gr::Tensor<double>{tileCentre}},
        {"rx_gains",            gr::Tensor<double>{cfg.gain}},
        {"verbose_overflow",    false},
    });
    if (!cfg.clock.empty()) {
        source_props["clock_source"] = cfg.clock;
    }
    if (cfg.lo_offset != 0.0) {
        source_props["lo_offset"] = cfg.lo_offset;
    }
    source_props["rx_dc_offset_auto"] = cfg.dc_offset_auto;
    auto& source = graph.emplaceBlock<
        gr::blocks::soapy::SoapySimpleSource<cf32>>(std::move(source_props));

    // Splitter -> 2 outputs
    auto& splitter = graph.emplaceBlock<gr::lora::Splitter>({
        {"n_outputs", gr::Size_t{2}},
    });
    if (!ok(graph.connect<"out", "in">(source, splitter))) {
        gr::lora::log_ts("error", "graph", "connect source -> splitter failed");
    }

    // Path 0: SpectrumTap -> NullSink (L1 energy)
    sg.spectrum = std::make_shared<gr::lora::SpectrumState>();
    sg.spectrum->fft_size    = 4096;
    sg.spectrum->sample_rate = static_cast<float>(cfg.l1_rate);
    sg.spectrum->center_freq = tileCentre;
    sg.spectrum->init();

    auto& tap = graph.emplaceBlock<gr::lora::SpectrumTapBlock>({});
    tap._spectrum = sg.spectrum;

    auto& null_sink = graph.emplaceBlock<gr::testing::NullSink<cf32>>({});

    if (!ok(graph.connect(splitter, "out#0"s, tap, "in"s))) {
        gr::lora::log_ts("error", "graph", "connect splitter -> tap failed");
    }
    if (!ok(graph.connect<"out", "in">(tap, null_sink))) {
        gr::lora::log_ts("error", "graph", "connect tap -> null_sink failed");
    }

    // Path 1: CaptureSink (L2 on-demand)
    sg.capture = std::make_shared<gr::lora::CaptureState>();
    const double minBw      = cfg.min_bw();
    const double targetRate  = minBw * static_cast<double>(cfg.os_factor);
    const auto   decFactor   = static_cast<uint32_t>(std::round(cfg.l1_rate / targetRate));
    const uint32_t sf12Win   = (1U << 12U) * cfg.os_factor * 2U;
    sg.capture->buffer.resize(sf12Win * decFactor);

    auto& cap_sink = graph.emplaceBlock<gr::lora::CaptureSink>({});
    cap_sink._state = sg.capture;

    if (!ok(graph.connect(splitter, "out#1"s, cap_sink, "in"s))) {
        gr::lora::log_ts("error", "graph", "connect splitter -> capture failed");
    }

    return sg;
}

/// Build the streaming scan graph: SoapySource → ScanController → ScanSink
/// No hardware retune — L2 uses digital channelization from the wideband stream.
/// ScanController handles both L1 energy (internal FFT) and L2 CAD.
/// Returns reference to ScanSink for callback setup (must be set before exchange()).
inline gr::lora::ScanSink& build_streaming_scan_graph(gr::Graph& graph, const ScanSetConfig& cfg) {
    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };

    const double centerFreq = cfg.center_freq();

    // SoapySource at L1 wideband rate, fixed center frequency
    auto source_props = lora_apps::soapy_reliability_defaults();
    source_props.merge(gr::property_map{
        {"device",              cfg.device},
        {"device_parameter",    cfg.device_param},
        {"sample_rate",         cfg.l1_rate},
        {"master_clock_rate",   cfg.master_clock},
        {"rx_center_frequency", gr::Tensor<double>{centerFreq}},
        {"rx_gains",            gr::Tensor<double>{cfg.gain}},
        {"verbose_overflow",    false},
    });
    if (!cfg.clock.empty()) {
        source_props["clock_source"] = cfg.clock;
    }
    if (cfg.lo_offset != 0.0) {
        source_props["lo_offset"] = cfg.lo_offset;
    }
    source_props["rx_dc_offset_auto"] = cfg.dc_offset_auto;
    auto& source = graph.emplaceBlock<
        gr::blocks::soapy::SoapySimpleSource<cf32>>(std::move(source_props));

    // ScanController: sole downstream block (IQ sink with ring buffer + L1/L2)
    // Build probe_bws string from config bws vector
    std::string probeBwsStr;
    for (std::size_t i = 0; i < cfg.bws.size(); ++i) {
        if (i > 0) probeBwsStr += ',';
        probeBwsStr += std::to_string(cfg.bws[i]);
    }
    if (probeBwsStr.empty()) probeBwsStr = "62500";

    auto& controller = graph.emplaceBlock<gr::lora::ScanController>({
        {"sample_rate",       static_cast<float>(cfg.l1_rate)},
        {"center_freq",       static_cast<float>(centerFreq)},
        {"min_ratio",         cfg.min_ratio},
        {"buffer_ms",         cfg.buffer_ms},
        {"channel_bw",        cfg.channel_bw},
        {"l1_interval",       uint32_t{16}},    // L1 FFT every 16 calls = ~8ms
        {"l1_snapshots",      uint32_t{4}},     // 4 snapshots = ~33ms sweep
        {"l1_fft_size",       cfg.l1_fft_size},
        {"probe_bws",         probeBwsStr},
        {"dc_blocker_cutoff", cfg.dc_blocker ? cfg.dc_blocker_cutoff : 0.f},
    });

    if (!ok(graph.connect<"out", "in">(source, controller))) {
        gr::lora::log_ts("error", "graph", "connect source -> controller failed");
    }

    // ScanSink: receives spectrum (with embedded detections) from ScanController
    auto& sink = graph.emplaceBlock<gr::lora::ScanSink>({});

    if (!ok(graph.connect<"spectrum_out", "spectrum">(controller, sink))) {
        gr::lora::log_ts("error", "graph", "connect controller.spectrum_out -> sink.spectrum failed");
    }

    return sink;
}

// ─── Wideband decode graph (lora_trx --wideband) ─────────────────────────────

/// Build wideband decode graph:
///   SoapySource (16 MS/s) → WidebandDecoder → FrameSink
///
/// WidebandDecoder handles L1 energy gating + digital channelization + multi-SF
/// decode internally. No Splitter or per-BW chains needed.
inline std::atomic<uint64_t>* build_wideband_graph(
        gr::Graph& graph, const TrxConfig& cfg,
        std::function<void(const std::vector<uint8_t>&, bool)> callback,
        std::function<void(const gr::property_map&)> telemetry_cb = {}) {
    auto ok = [](std::expected<void, gr::Error> r) { return r.has_value(); };

    const auto& dc = cfg.decode_configs[0];

    // SoapySource at wideband rate
    auto source_props = lora_apps::soapy_reliability_defaults();
    source_props.merge(gr::property_map{
        {"device", cfg.device},
        {"device_parameter", cfg.device_param},
        {"sample_rate", static_cast<double>(cfg.wideband_rate)},
        {"master_clock_rate", cfg.wideband_master_clock},
        {"rx_center_frequency", gr::Tensor<double>{cfg.freq}},
        {"rx_gains", gr::Tensor<double>{cfg.gain_rx}},
        {"verbose_overflow", false},
    });
    if (!cfg.clock.empty()) {
        source_props["clock_source"] = cfg.clock;
    }
    if (cfg.lo_offset != 0.0) {
        source_props["lo_offset"] = cfg.lo_offset;
    }
    source_props["rx_dc_offset_auto"] = cfg.dc_offset_auto;
    auto& source = graph.emplaceBlock<
        gr::blocks::soapy::SoapySimpleSource<cf32>>(std::move(source_props));

    // WidebandDecoder
    gr::property_map wb_props{
        {"sample_rate", static_cast<float>(cfg.wideband_rate)},
        {"center_freq", static_cast<float>(cfg.freq)},
        {"channel_bw", 62500.f},
        {"decode_bw", static_cast<float>(cfg.bw)},
        {"max_channels", uint32_t{8}},
        {"sync_word", dc.sync_word},
        {"preamble_len", cfg.preamble},
        {"debug", cfg.debug},
        {"l1_interval", uint32_t{16}},    // L1 FFT every 16 calls = 8ms
        {"l1_snapshots", uint32_t{4}},    // 4 snapshots = 33ms sweep (catches brief ADVERTs)
        {"dc_blocker_cutoff", cfg.dc_blocker ? cfg.dc_blocker_cutoff : 0.f},
        {"soft_decode", cfg.soft_decode},
    };
    if (!cfg.decode_sfs_str.empty()) {
        wb_props["decode_sfs_str"] = cfg.decode_sfs_str;
    }
    auto& decoder = graph.emplaceBlock<gr::lora::WidebandDecoder>(std::move(wb_props));
    if (telemetry_cb) {
        decoder._telemetry = std::move(telemetry_cb);
    }

    // FrameSink
    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"sync_word", dc.sync_word},
        {"phy_sf", uint8_t{0}},  // SF varies per frame, reported in tag
        {"phy_bw", cfg.bw},
        {"label", dc.label},
    });
    sink._frame_callback = callback;


    if (!ok(graph.connect<"out", "in">(source, decoder))) {
        gr::lora::log_ts("error", "graph", "connect source -> WidebandDecoder failed");
    }
    if (!ok(graph.connect<"out", "in">(decoder, sink))) {
        gr::lora::log_ts("error", "graph", "connect WidebandDecoder -> FrameSink failed");
    }

    const auto sf_desc = cfg.decode_sfs_str.empty()
        ? std::string("SF7-12") : ("SF=" + cfg.decode_sfs_str);
    gr::lora::log_ts("info ", "graph",
        "wideband graph: %.1f MS/s, %.3f MHz center, BW%u, %s",
        cfg.wideband_rate / 1e6, cfg.freq / 1e6, cfg.bw, sf_desc.c_str());

    return &source._totalOverFlowCount;
}

}  // namespace lora_graph

#endif  // GR4_LORA_GRAPH_BUILDER_HPP
