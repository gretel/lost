// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP
#define GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP

#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

namespace gr::lora {

/// Frequency-shift samples by mixing with a complex NCO.
/// Phase accumulation uses double for precision at high sample rates (16 MS/s).
inline void freqShift(std::span<std::complex<float>> samples,
                      double shift_hz, double sample_rate) {
    const double phaseInc = 2.0 * std::numbers::pi * shift_hz / sample_rate;
    double       phase    = 0.0;
    for (auto& s : samples) {
        auto rot = std::complex<float>(
            static_cast<float>(std::cos(phase)),
            static_cast<float>(std::sin(phase)));
        s *= rot;
        phase += phaseInc;
    }
}

/// Decimate by integer factor using integrate-and-dump (box-car average).
/// factor must divide input evenly.
[[nodiscard]] inline std::vector<std::complex<float>>
decimate(const std::complex<float>* src, uint32_t nSamples, uint32_t factor) {
    if (factor <= 1) {
        return std::vector<std::complex<float>>(src, src + nSamples);
    }
    const uint32_t outLen = nSamples / factor;
    std::vector<std::complex<float>> out(outLen);
    const float scale = 1.0f / static_cast<float>(factor);
    for (uint32_t i = 0; i < outLen; ++i) {
        std::complex<float> sum{0.0f, 0.0f};
        for (uint32_t j = 0; j < factor; ++j) {
            sum += src[i * factor + j];
        }
        out[i] = sum * scale;
    }
    return out;
}

/// Extract a narrowband channel from a wideband buffer.
/// Freq-shifts to baseband, then decimates.
/// f_channel: center frequency of target channel (Hz)
/// f_center:  center frequency of wideband capture (Hz)
/// sample_rate: wideband sample rate (Hz)
/// target_bw: target channel bandwidth (Hz)
[[nodiscard]] inline std::vector<std::complex<float>>
channelize(const std::complex<float>* src, uint32_t nSamples,
           double f_channel, double f_center, double sample_rate,
           double target_bw) {
    // Copy to working buffer for in-place freq shift
    std::vector<std::complex<float>> buf(src, src + nSamples);

    // Negate shift to bring the target channel down to baseband
    const double shift_hz = -(f_channel - f_center);
    freqShift(std::span<std::complex<float>>(buf), shift_hz, sample_rate);

    // Decimate to target bandwidth
    const auto factor = static_cast<uint32_t>(std::round(sample_rate / target_bw));
    return decimate(buf.data(), nSamples, factor);
}

} // namespace gr::lora

#endif // GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP
