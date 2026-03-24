// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_DC_BLOCKER_HPP
#define GNURADIO_LORA_ALGORITHM_DC_BLOCKER_HPP

// 2nd-order Butterworth high-pass DC blocker for complex IQ streams.
//
// Removes the DC spur (LO leakage) that direct-conversion (zero-IF) receivers
// produce at the center frequency. Uses separate I/Q filter instances designed
// via gnuradio4's filter::iir::designFilter (Butterworth, order 2).
//
// Filter coefficients and state are kept in double precision to avoid numerical
// instability at extreme cutoff-to-samplerate ratios (e.g. 10 Hz / 16 MHz).
// Input and output remain cf32; the double-to-float conversion at the I/O
// boundary is lossless for typical SDR dynamic range.
//
// Default cutoff: 2000 Hz — tuned for B210 AD9361 whose DC offset modulates
// at up to ~1 kHz (AGC loop, temperature tracking). At BW62.5k this notches
// ~6% of bins; sync word 0x12 bins fall inside for SF9-12.
//
// Usage:
//   DCBlocker dc;
//   dc.init(16e6f, 10.f);  // sample rate, cutoff Hz
//   dc.processBlock(in_span, out_span);

#include <complex>
#include <cstddef>
#include <span>

#include <gnuradio-4.0/algorithm/filter/FilterTool.hpp>

namespace gr::lora {

using cf32 = std::complex<float>;

struct DCBlocker {
    float _sampleRate{0.f};
    float _cutoffHz{2000.f};
    bool  _initialised{false};

    // double precision avoids float32 coefficient quantisation at low fc/fs ratios
    gr::filter::Filter<double> _filterI;
    gr::filter::Filter<double> _filterQ;

    DCBlocker() = default;

    explicit DCBlocker(float sampleRate, float cutoffHz = 2000.f) { init(sampleRate, cutoffHz); }

    void init(float sampleRate, float cutoffHz = 2000.f) {
        _sampleRate  = sampleRate;
        _cutoffHz    = cutoffHz;
        _initialised = false;

        if (sampleRate <= 0.f || cutoffHz <= 0.f || cutoffHz >= sampleRate / 2.f) {
            return;
        }

        // Order must be ≤ 2. Order 4 is numerically unstable in double precision
        // at extreme fc/fs ratios (e.g. 10 Hz / 250 kHz) — coefficients lose
        // significance and the filter produces garbage or diverges.
        constexpr std::size_t kOrder = 2UZ;

        auto coeffs = gr::filter::iir::designFilter<double>(
            gr::filter::Type::HIGHPASS,
            gr::filter::FilterParameters{
                .order = kOrder,
                .fHigh = static_cast<double>(cutoffHz),
                .fs    = static_cast<double>(sampleRate)},
            gr::filter::iir::Design::BUTTERWORTH);

        _filterI    = gr::filter::Filter<double>(coeffs);
        _filterQ    = gr::filter::Filter<double>(coeffs);
        _initialised = true;
    }

    void reset() {
        _filterI.reset();
        _filterQ.reset();
    }

    [[nodiscard]] cf32 processOne(cf32 sample) noexcept {
        if (!_initialised) {
            return sample;
        }
        return {static_cast<float>(_filterI.processOne(static_cast<double>(sample.real()))),
                static_cast<float>(_filterQ.processOne(static_cast<double>(sample.imag())))};
    }

    void processBlock(std::span<const cf32> in, std::span<cf32> out) noexcept {
        if (!_initialised) {
            if (in.data() != out.data()) {
                std::copy(in.begin(), in.end(), out.begin());
            }
            return;
        }
        for (std::size_t i = 0; i < in.size(); ++i) {
            out[i] = {static_cast<float>(_filterI.processOne(static_cast<double>(in[i].real()))),
                      static_cast<float>(_filterQ.processOne(static_cast<double>(in[i].imag())))};
        }
    }

    [[nodiscard]] bool initialised() const noexcept { return _initialised; }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_ALGORITHM_DC_BLOCKER_HPP
