// SPDX-License-Identifier: ISC
//
// FreqXlatingDecimator.hpp — Frequency-translating decimating FIR filter.
//
// Translates a channel at a given offset frequency to baseband, applies a
// low-pass FIR filter, and decimates by a fixed factor. This is the GR4
// equivalent of GR3's freq_xlating_fir_filter.
//
// Processing per output sample:
//   1. Complex-multiply input by rotating phasor (frequency shift)
//   2. Convolve with real-valued FIR taps
//   3. Decimate: output every `decimation`-th filtered sample
//
// The block manages its own input/output via consume/publish in processBulk,
// similar to FrameSync.

#ifndef GNURADIO_LORA_FREQ_XLATING_DECIMATOR_HPP
#define GNURADIO_LORA_FREQ_XLATING_DECIMATOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

namespace gr::lora {

/// Frequency-translating decimating FIR filter.
///
/// Shifts center_freq_offset Hz to baseband, filters with FIR taps,
/// and decimates by `decimation`.
///
/// The taps are real-valued (symmetric low-pass filter). The frequency
/// translation is done by complex multiplication with a rotating phasor
/// before filtering.
struct FreqXlatingDecimator : gr::Block<FreqXlatingDecimator,
                                        gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<std::complex<float>> out;

    double   sample_rate       = 250000.0;  ///< input sample rate in Hz
    double   center_freq_offset = 0.0;      ///< channel offset from center in Hz
    uint32_t decimation        = 1;         ///< decimation factor (N:1)

    GR_MAKE_REFLECTABLE(FreqXlatingDecimator, in, out,
                        sample_rate, center_freq_offset, decimation);

    // --- Internal state ---
    std::vector<float>               _taps;          ///< FIR filter coefficients
    std::vector<std::complex<float>> _history;       ///< circular buffer for FIR state
    uint32_t                         _hist_idx = 0;  ///< write position in history
    double                           _phase    = 0.0;  ///< phasor phase accumulator
    double                           _phase_inc = 0.0;  ///< phasor phase increment
    bool                             _configured = false;

    void start() {
        recalculate();
    }

    void settingsChanged(const gr::property_map& /*old_settings*/,
                         const gr::property_map& /*new_settings*/) {
        recalculate();
    }

    /// Configure the block with new taps. Must be called after taps are set.
    void setTaps(std::vector<float> new_taps) {
        _taps = std::move(new_taps);
        _history.assign(_taps.size(), {0.f, 0.f});
        _hist_idx = 0;
        _configured = !_taps.empty();
    }

    void recalculate() {
        _phase_inc = -2.0 * std::numbers::pi * center_freq_offset / sample_rate;
        if (decimation < 1) decimation = 1;
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (!_configured || _taps.empty()) {
            // Pass-through mode when no taps configured (decimation only)
            std::size_t n_out = std::min(in_span.size() / decimation, out_span.size());
            for (std::size_t i = 0; i < n_out; i++) {
                auto idx = i * decimation;
                // Frequency shift
                float phase_f = static_cast<float>(_phase + _phase_inc * static_cast<double>(idx));
                auto phasor = std::complex<float>(std::cos(phase_f), std::sin(phase_f));
                out_span[i] = in_span[idx] * phasor;
            }
            _phase += _phase_inc * static_cast<double>(n_out * decimation);
            // Keep phase in [0, 2*pi)
            _phase = std::fmod(_phase, 2.0 * std::numbers::pi);

            std::ignore = input.consume(n_out * decimation);
            output.publish(n_out);
            return gr::work::Status::OK;
        }

        const auto n_taps = static_cast<uint32_t>(_taps.size());

        // Number of output samples we can produce
        std::size_t n_out = std::min(in_span.size() / decimation, out_span.size());
        if (n_out == 0) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        std::size_t in_consumed = 0;
        for (std::size_t o = 0; o < n_out; o++) {
            // Process `decimation` input samples
            for (uint32_t d = 0; d < decimation && in_consumed < in_span.size(); d++) {
                // Frequency shift
                float phase_f = static_cast<float>(_phase);
                auto phasor = std::complex<float>(std::cos(phase_f), std::sin(phase_f));
                _history[_hist_idx] = in_span[in_consumed] * phasor;
                _hist_idx = (_hist_idx + 1) % n_taps;
                _phase += _phase_inc;
                in_consumed++;
            }

            // FIR convolution (dot product of history with taps)
            std::complex<float> acc(0.f, 0.f);
            uint32_t h = _hist_idx;  // oldest sample
            for (uint32_t k = 0; k < n_taps; k++) {
                acc += _history[h] * _taps[k];
                h = (h + 1) % n_taps;
            }
            out_span[o] = acc;
        }

        // Keep phase in [0, 2*pi)
        _phase = std::fmod(_phase, 2.0 * std::numbers::pi);

        std::ignore = input.consume(in_consumed);
        output.publish(n_out);
        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_FREQ_XLATING_DECIMATOR_HPP
