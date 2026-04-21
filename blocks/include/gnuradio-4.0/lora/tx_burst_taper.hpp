// SPDX-License-Identifier: ISC
//
// Raised-cosine burst taper applied to TX IQ bursts to reduce adjacent-channel
// leakage (ACLR) from otherwise-rectangular DAC windows.  Separated from
// apps/lora_trx.cpp so unit tests under test/qa_lora_tx.cpp can exercise it
// directly against `gr::algorithm::BurstTaper<float>`.

#ifndef GR4_LORA_TX_BURST_TAPER_HPP
#define GR4_LORA_TX_BURST_TAPER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <functional>
#include <ranges>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/BurstTaper.hpp>

namespace gr::lora {

/// Ramp length for the head / tail of every TX burst.  The caller is expected
/// to prepend this many zero-samples before calling applyBurstTaper so the
/// rising ramp attenuates silence rather than the live preamble (see
/// prependHeadPadAndTaper).  128 samples is 0.5 ms at 250 kS/s and 32 us at
/// 4 MS/s — short compared to any realistic LoRa preamble.
constexpr std::size_t kTxTaperRampSamples = 128;

namespace detail {

inline const std::array<float, kTxTaperRampSamples>& riseEdgeCoefficients() {
    static const auto edge = [] {
        std::array<float, kTxTaperRampSamples> a{};
        gr::algorithm::BurstTaper<float>::generateEdge(gr::algorithm::TaperType::RaisedCosine, std::span(a), true);
        return a;
    }();
    return edge;
}

inline const std::array<float, kTxTaperRampSamples>& fallEdgeCoefficients() {
    static const auto edge = [] {
        std::array<float, kTxTaperRampSamples> a{};
        gr::algorithm::BurstTaper<float>::generateEdge(gr::algorithm::TaperType::RaisedCosine, std::span(a), false);
        return a;
    }();
    return edge;
}

} // namespace detail

/// Multiply the first kTxTaperRampSamples of `iq` by the rising envelope and
/// the last kTxTaperRampSamples by the falling envelope.  Returns unchanged if
/// the burst is shorter than 2 * kTxTaperRampSamples (head and tail ramps
/// would otherwise overlap).
inline void applyBurstTaper(std::vector<std::complex<float>>& iq) {
    if (iq.size() < 2UZ * kTxTaperRampSamples) {
        return;
    }
    const auto& rise = detail::riseEdgeCoefficients();
    const auto& fall = detail::fallEdgeCoefficients();
    std::transform(rise.begin(), rise.end(), iq.begin(), iq.begin(), std::multiplies{});
    const auto tailStart = iq.size() - kTxTaperRampSamples;
    std::transform(fall.begin(), fall.end(), iq.begin() + static_cast<std::ptrdiff_t>(tailStart), iq.begin() + static_cast<std::ptrdiff_t>(tailStart), std::multiplies{});
}

/// Prepend kTxTaperRampSamples zeros then apply the taper.  The rising ramp
/// attenuates the prepended silence, leaving the first real sample (preamble
/// chirp start) unmodified.  Tail zeros from upstream burst construction
/// already provide a silent fall region.
inline void prependHeadPadAndTaper(std::vector<std::complex<float>>& iq) {
    iq.insert(iq.begin(), kTxTaperRampSamples, std::complex<float>{});
    applyBurstTaper(iq);
}

} // namespace gr::lora

#endif // GR4_LORA_TX_BURST_TAPER_HPP
