// SPDX-License-Identifier: ISC
/// Smoke tests for apps/graph_builder.hpp property_map keys.
///
/// Verifies build_rx_graph / build_scan_graph do not throw
/// GR_MAKE_REFLECTABLE unknown-key exceptions against the
/// apps/config.toml fixture backed by SoapyLoopback. Catches drift
/// after any upstream gnuradio4 rebase that renames or removes
/// SoapySource keys used by our graph builders.
///
/// Scope: apps/graph_builder.hpp only. build_tx_graph currently
/// lives in apps/lora_trx.cpp (TU-local, not exposed in a header),
/// so it is not covered here.

#include <filesystem>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-W#warnings"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <SoapySDR/Modules.hpp>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <gnuradio-4.0/Graph.hpp>

#include "../apps/config.hpp"
#include "../apps/graph_builder.hpp"

using namespace boost::ut;

#ifndef SOAPY_LOOPBACK_MODULE_PATH
#error "SOAPY_LOOPBACK_MODULE_PATH must be defined by CMake"
#endif
#ifndef CONFIG_TOML_PATH
#error "CONFIG_TOML_PATH must be defined by CMake"
#endif

namespace {

/// RAII wrapper: load the SoapyLoopback driver module for the
/// duration of a test so SoapySource can resolve device="loopback".
struct LoopbackModuleGuard {
    LoopbackModuleGuard() {
        const auto err = SoapySDR::loadModule(SOAPY_LOOPBACK_MODULE_PATH);
        expect(err.empty()) << "loadModule failed: " << err;
    }
    ~LoopbackModuleGuard() { SoapySDR::unloadModule(SOAPY_LOOPBACK_MODULE_PATH); }
};

/// Load the first [trx] config and redirect the device stack at the
/// in-process SoapyLoopback driver.  All RF-specific fields that the
/// loopback driver does not exercise (clock source, antenna labels,
/// multi-channel RX) are reset to neutral defaults so the graph
/// builder only exercises property_map key validation.
lora_config::TrxConfig loadTrxForLoopback() {
    auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
    expect(fatal(cfgs.size() >= 1_u)) << "load_config returned empty";
    auto c           = cfgs[0];
    c.device         = "loopback";
    c.device_param   = "";
    c.clock          = "";
    c.rx_channels    = {0U};   // SoapyLoopback exposes 1 RX channel
    c.rx_antenna     = {};     // driver default
    c.lo_offset      = 0.0;    // loopback has no LO
    c.dc_offset_auto = false;  // loopback has no DC offset mode
    c.rx_bandwidths  = {c.bw}; // single BW path (no Splitter fanout)
    return c;
}

/// Load the [scan] config and redirect at SoapyLoopback.
lora_config::ScanSetConfig loadScanForLoopback() {
    auto cfgs = lora_config::load_scan_config(CONFIG_TOML_PATH);
    expect(fatal(cfgs.size() >= 1_u)) << "load_scan_config returned empty";
    auto c           = cfgs[0];
    c.device         = "loopback";
    c.device_param   = "";
    c.clock          = "";
    c.lo_offset      = 0.0;
    c.dc_offset_auto = false;
    c.sweeps         = 0U; // infinite (skips disconnect_on_done)
    return c;
}

} // namespace

const boost::ut::suite graphBuilderSmoke = [] {
    "build_rx_graph does not throw on loopback"_test = [] {
        LoopbackModuleGuard mod;
        auto                cfg = loadTrxForLoopback();
        gr::Graph           graph;
        expect(nothrow([&] {
            auto* overflow_ptr = lora_graph::build_rx_graph(graph, cfg, [](const std::vector<uint8_t>&, bool, uint16_t) {},
                /*spectrum=*/nullptr,
                /*channel_busy=*/nullptr,
                /*telemetry_cb=*/{});
            (void)overflow_ptr;
        }));
    };

    "build_rx_graph multi-chain (nChains > 1) does not throw"_test = [] {
        LoopbackModuleGuard mod;
        auto                cfg = loadTrxForLoopback();
        // Add a second chain (distinct label + sync_word per validation rules).
        lora_config::DecodeConfig extra;
        extra.label     = "secondary";
        extra.sync_word = uint16_t{0x34};
        // sf_set empty = sweep SF7-12
        cfg.rx_chains.push_back(std::move(extra));

        gr::Graph graph;
        expect(nothrow([&] {
            auto* overflow_ptr = lora_graph::build_rx_graph(graph, cfg, [](const std::vector<uint8_t>&, bool, uint16_t) {},
                /*spectrum=*/nullptr,
                /*channel_busy=*/nullptr,
                /*telemetry_cb=*/{});
            (void)overflow_ptr;
        }));
    };

    "build_rx_graph multi-BW x multi-chain does not throw"_test = [] {
        LoopbackModuleGuard mod;
        auto                cfg = loadTrxForLoopback();
        cfg.rx_bandwidths       = {cfg.bw, cfg.bw * 2U}; // 2 distinct BWs
        lora_config::DecodeConfig extra;
        extra.label     = "secondary";
        extra.sync_word = uint16_t{0x34};
        cfg.rx_chains.push_back(std::move(extra));

        gr::Graph graph;
        expect(nothrow([&] {
            auto* overflow_ptr = lora_graph::build_rx_graph(graph, cfg, [](const std::vector<uint8_t>&, bool, uint16_t) {},
                /*spectrum=*/nullptr,
                /*channel_busy=*/nullptr,
                /*telemetry_cb=*/{});
            (void)overflow_ptr;
        }));
    };

    "build_scan_graph does not throw on loopback"_test = [] {
        LoopbackModuleGuard mod;
        auto                cfg = loadScanForLoopback();
        gr::Graph           graph;
        // build_scan_graph needs a non-empty channel list; the
        // configured centre frequency is sufficient for the smoke test.
        std::vector<double> channels{cfg.center_freq()};
        expect(nothrow([&] {
            auto sg = lora_graph::build_scan_graph(graph, cfg, channels);
            (void)sg;
        }));
    };
};

int main() { /* boost::ut drives the suite */ }
