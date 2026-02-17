// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_UTILITIES_HPP
#define GNURADIO_LORA_UTILITIES_HPP

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

namespace gr::lora {

inline constexpr int LDRO_MAX_DURATION_MS = 16;

using LLR = double;  ///< Log-Likelihood Ratio type

/// Positive modulus: result is always in [0, b-1].
[[nodiscard]] inline constexpr int64_t mod(int64_t a, int64_t b) noexcept {
    return (a % b + b) % b;
}

/// Convert an integer to a MSB-first vector of bool with n_bits bits.
[[nodiscard]] inline std::vector<bool> int2bool(uint32_t integer, uint8_t n_bits) {
    std::vector<bool> vec(n_bits, false);
    for (std::size_t i = 0; i < n_bits; i++) {
        vec[n_bits - 1 - i] = (integer >> i) & 1;
    }
    return vec;
}

/// Convert a MSB-first vector of bool to an integer.
[[nodiscard]] inline uint32_t bool2int(const std::vector<bool>& b) {
    uint32_t result = 0;
    for (bool bit : b) {
        result = (result << 1) | static_cast<uint32_t>(bit);
    }
    return result;
}

/// Build an upchirp with given symbol ID, SF, and oversampling factor.
/// chirp must point to 2^sf * os_factor elements.
inline void build_upchirp(std::complex<float>* chirp, uint32_t id, uint8_t sf, uint8_t os_factor = 1) {
    const double N      = static_cast<double>(1u << sf);
    const int    n_fold = static_cast<int>(N * os_factor) - static_cast<int>(id * os_factor);
    const int    len    = static_cast<int>(N * os_factor);
    const double os2    = static_cast<double>(os_factor) * static_cast<double>(os_factor);

    for (int n = 0; n < len; n++) {
        double phase;
        if (n < n_fold) {
            phase = 2.0 * std::numbers::pi * (static_cast<double>(n) * n / (2.0 * N) / os2
                                               + (static_cast<double>(id) / N - 0.5) * n / os_factor);
        } else {
            phase = 2.0 * std::numbers::pi * (static_cast<double>(n) * n / (2.0 * N) / os2
                                               + (static_cast<double>(id) / N - 1.5) * n / os_factor);
        }
        chirp[n] = std::complex<float>(static_cast<float>(std::cos(phase)),
                                       static_cast<float>(std::sin(phase)));
    }
}

/// Build reference upchirp (id=0) and its conjugate (downchirp).
/// Each must point to 2^sf * os_factor elements.
inline void build_ref_chirps(std::complex<float>* upchirp, std::complex<float>* downchirp,
                             uint8_t sf, uint8_t os_factor = 1) {
    const int len = (1 << sf) * os_factor;
    build_upchirp(upchirp, 0, sf, os_factor);
    for (int n = 0; n < len; n++) {
        downchirp[n] = std::conj(upchirp[n]);
    }
}

/// Dechirp + FFT + argmax: element-wise multiply samples with ref_chirp,
/// FFT, return bin index with maximum magnitude-squared.
/// scratch must point to N writable elements.
[[nodiscard]] inline uint32_t dechirp_argmax(
        const std::complex<float>* samples,
        const std::complex<float>* ref_chirp,
        std::complex<float>* scratch, uint32_t N,
        gr::algorithm::FFT<std::complex<float>>& fft) {
    for (uint32_t i = 0; i < N; i++) {
        scratch[i] = samples[i] * ref_chirp[i];
    }
    auto fft_out = fft.compute(std::span<const std::complex<float>>(scratch, N));

    float max_val = 0.f;
    uint32_t max_idx = 0;
    for (uint32_t i = 0; i < N; i++) {
        float mag_sq = fft_out[i].real() * fft_out[i].real()
                     + fft_out[i].imag() * fft_out[i].imag();
        if (mag_sq > max_val) {
            max_val = mag_sq;
            max_idx = i;
        }
    }
    return max_idx;
}

}  // namespace gr::lora

#endif  // GNURADIO_LORA_UTILITIES_HPP
