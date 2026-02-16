// SPDX-License-Identifier: ISC
//
// firdes.hpp — FIR filter design: Hamming-windowed sinc low-pass filter.
//
// Textbook implementation. No external dependencies.

#ifndef GNURADIO_LORA_FIRDES_HPP
#define GNURADIO_LORA_FIRDES_HPP

#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

namespace gr::lora {

/// Design a low-pass FIR filter using the windowed-sinc method with
/// a Hamming window.
///
/// @param num_taps       Number of filter taps (should be odd for linear phase)
/// @param sample_rate    Sample rate in Hz
/// @param cutoff         Cutoff frequency in Hz (−6 dB point)
/// @return               Vector of num_taps real-valued filter coefficients
[[nodiscard]] inline std::vector<float> firdes_low_pass(
        uint32_t num_taps, double sample_rate, double cutoff) {
    std::vector<float> taps(num_taps, 0.f);
    if (num_taps == 0 || sample_rate <= 0.0 || cutoff <= 0.0) return taps;

    const double fc = cutoff / sample_rate;        // normalized cutoff [0, 0.5]
    const int M = static_cast<int>(num_taps) - 1;  // filter order

    double sum = 0.0;
    for (int n = 0; n <= M; n++) {
        double nm = static_cast<double>(n) - static_cast<double>(M) / 2.0;

        // Ideal sinc low-pass
        double h;
        if (std::abs(nm) < 1e-12) {
            h = 2.0 * fc;
        } else {
            h = std::sin(2.0 * std::numbers::pi * fc * nm)
              / (std::numbers::pi * nm);
        }

        // Hamming window
        double w = 0.54 - 0.46 * std::cos(2.0 * std::numbers::pi
                                           * static_cast<double>(n)
                                           / static_cast<double>(M));

        taps[static_cast<std::size_t>(n)] = static_cast<float>(h * w);
        sum += h * w;
    }

    // Normalize for unity DC gain
    if (std::abs(sum) > 1e-12) {
        for (auto& t : taps) {
            t = static_cast<float>(static_cast<double>(t) / sum);
        }
    }

    return taps;
}

}  // namespace gr::lora

#endif  // GNURADIO_LORA_FIRDES_HPP
