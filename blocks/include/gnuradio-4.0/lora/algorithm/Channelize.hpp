// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP
#define GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP

#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>

namespace gr::lora {

/// Frequency-shift samples by mixing with a complex NCO (fast recurrence).
/// Uses complex rotation accumulation (~4 FLOPs/sample) instead of per-sample
/// cos/sin (~100+ cycles/sample).  Renormalizes every 1024 samples to prevent
/// amplitude drift from floating-point error accumulation.
inline void freqShift(std::span<std::complex<float>> samples,
                      double shift_hz, double sample_rate) {
    if (samples.empty()) return;
    const double phaseInc = 2.0 * std::numbers::pi * shift_hz / sample_rate;
    std::complex<float> phasor(
        static_cast<float>(std::cos(phaseInc)),
        static_cast<float>(std::sin(phaseInc)));
    std::complex<float> rot(1.0f, 0.0f);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        samples[i] *= rot;
        rot *= phasor;
        if ((i & 0x3FFU) == 0x3FFU) {
            rot /= std::abs(rot);  // renormalize
        }
    }
}

/// Decimate by integer factor using integrate-and-dump (box-car average).
/// factor must divide input evenly.
/// NOTE: box-car has -13 dB stopband rejection.  Prefer channelizeFir()
/// for applications requiring better spectral purity.
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

/// Extract a narrowband channel from a wideband buffer (box-car decimation).
/// Freq-shifts to baseband, then decimates.  Uses fast NCO recurrence.
/// NOTE: -13 dB stopband.  Prefer channelizeFir() for better quality.
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

/// Extract a narrowband channel using cascaded half-band FIR decimation.
/// Fuses NCO frequency shift + FIR filtering in a single pass (no intermediate
/// buffer).  -45 dB stopband rejection vs -13 dB for box-car channelize().
///
/// @param decimator  Pre-initialized CascadedDecimator (nStages = log2(factor)).
///                   Reset internally per call.  Caller must keep alive.
/// @param out        Output vector; cleared and filled with narrowband samples.
///
/// Total decimation factor = 2^nStages.  The wideband sample rate divided by
/// target_bw must be a power of 2 (e.g., 16 MS/s / 62.5 kHz = 256 = 2^8).
inline void channelizeFir(const std::complex<float>* src, uint32_t nSamples,
                          double f_channel, double f_center, double sample_rate,
                          CascadedDecimator& decimator,
                          std::vector<std::complex<float>>& out) {
    out.clear();
    if (nSamples == 0) return;

    const double shift_hz = -(f_channel - f_center);
    const double phaseInc = 2.0 * std::numbers::pi * shift_hz / sample_rate;
    std::complex<float> step(
        static_cast<float>(std::cos(phaseInc)),
        static_cast<float>(std::sin(phaseInc)));
    std::complex<float> rot(1.0f, 0.0f);

    decimator.reset();
    decimator.processWithNcoBatch(
        std::span<const std::complex<float>>(src, nSamples),
        out, rot, step);
}

} // namespace gr::lora

#endif // GNURADIO_LORA_ALGORITHM_CHANNELIZE_HPP
