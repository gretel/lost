// SPDX-License-Identifier: ISC
/// Microbenchmarks for the production PHY blocks.
///
/// Covers CssDemod (per-symbol dechirp+FFT+argmax, hard and soft) and
/// DecodeChain (post-demod header+payload pipeline).  These are the blocks
/// actually on the RX hot path; bm_lora_algorithms exercises the underlying
/// algorithm helpers in isolation.
///
/// Run with: ./bm_lora_phy
/// Extra precision: BM_DIGITS=3 ./bm_lora_phy

#include <benchmark.hpp>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/phy/CssDemod.hpp>
#include <gnuradio-4.0/lora/phy/DecodeChain.hpp>
#include <gnuradio-4.0/lora/phy/Types.hpp>

namespace {

using gr::lora::phy::cf32;

constexpr uint8_t  kSf          = 8;
constexpr uint32_t kBandwidthHz = 125'000;
constexpr uint8_t  kCr          = 4;
constexpr uint32_t kN           = uint32_t{1} << kSf;

const std::vector<uint8_t> kPayload = {'H', 'e', 'l', 'l', 'o', ' ', 'M', 'e', 's', 'h', 'C', 'o', 'r', 'e'};

volatile std::size_t g_sink = 0;

/// Build one dechirp-ready SF8 upchirp symbol for a given bin.
[[nodiscard]] std::vector<cf32> makeUpchirpSymbol(uint16_t sym_val) {
    std::vector<cf32> chirp(kN);
    gr::lora::build_upchirp(chirp.data(), sym_val, kSf, /*os_factor=*/1);
    return chirp;
}

/// Generate the full frame bin stream (post-header, post-Gray), ready to feed
/// DecodeChain::push_symbol in lock-step.  Mirrors the pre-modulation stages of
/// gr::lora::generate_frame_iq so the DecodeChain sees the exact symbol
/// sequence the TX side produced for kPayload.
[[nodiscard]] std::vector<uint16_t> makePayloadBins() {
    const bool ldro        = gr::lora::needs_ldro(kSf, kBandwidthHz);
    auto       whitened    = gr::lora::whiten(kPayload);
    auto       withHeader  = gr::lora::insert_header(whitened, static_cast<uint8_t>(kPayload.size()), kCr, /*has_crc=*/true);
    auto       withCrc     = gr::lora::add_crc(withHeader, kPayload, /*has_crc=*/true);
    auto       encoded     = gr::lora::hamming_encode_frame(withCrc, kSf, kCr);
    auto       interleaved = gr::lora::interleave_frame(encoded, kSf, kCr, ldro);
    auto       grayMapped  = gr::lora::gray_demap(interleaved, kSf);

    std::vector<uint16_t> bins(grayMapped.size());
    std::ranges::transform(grayMapped, bins.begin(), [](uint32_t v) { return static_cast<uint16_t>(v); });
    return bins;
}

} // namespace

const boost::ut::suite demod_benchmarks = [] {
    using namespace benchmark;

    gr::lora::phy::CssDemod demod;
    demod.init({.sf = kSf, .bandwidth_hz = kBandwidthHz, .soft_decode = false});
    auto symbol = makeUpchirpSymbol(42);

    constexpr int kReps                                                            = 10'000;
    ::benchmark::benchmark<kReps>(std::string_view("CssDemod::demodHard SF8"), kN) = [&] {
        auto r = demod.demodHard(symbol);
        g_sink = r.bin;
    };

    gr::lora::phy::CssDemod demodSoft;
    demodSoft.init({.sf = kSf, .bandwidth_hz = kBandwidthHz, .soft_decode = true});
    std::vector<double> llrs(kSf);
    ::benchmark::benchmark<kReps>(std::string_view("CssDemod::demodSoft SF8"), kN) = [&] {
        auto r = demodSoft.demodSoft(symbol, llrs);
        g_sink = r.bin;
    };
};

const boost::ut::suite decode_chain_benchmarks = [] {
    using namespace benchmark;

    const auto bins = makePayloadBins();

    constexpr int kReps                                                                            = 1'000;
    ::benchmark::benchmark<kReps>(std::string_view("DecodeChain full frame SF8 CR4"), bins.size()) = [&] {
        gr::lora::phy::DecodeChain chain;
        chain.init({.sf = kSf, .bandwidth_hz = kBandwidthHz, .soft_decode = false});
        for (uint16_t bin : bins) {
            auto r = chain.push_symbol(bin);
            if (r.has_value()) {
                g_sink += r->payload.size();
                break;
            }
        }
    };
};

int main() { return 0; }
