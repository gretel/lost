// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_UTILITIES_HPP
#define GNURADIO_LORA_UTILITIES_HPP

#include <array>
#include <cassert>
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
    assert(b != 0 && "mod() called with divisor 0");
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

/// Result of dechirp + FFT: argmax bin index and peak-to-mean ratio (PMR).
struct DechirpResult {
    uint32_t bin;   ///< FFT bin with maximum magnitude
    float    pmr;   ///< peak / mean magnitude ratio (higher = better quality)
};

/// Dechirp + FFT + argmax + quality: like dechirp_argmax but also returns
/// peak-to-mean ratio (PMR) — the ratio of the peak FFT magnitude to the
/// mean magnitude across all bins.  A real chirp produces PMR >> 1 (e.g., 16
/// for SF8); noise produces PMR ~ 1.5-2.5.
///
/// When remove_dc is true, the mean of the input is subtracted before
/// dechirping.  This suppresses DC offset / LO leakage that would
/// otherwise produce a spurious peak at bin 0.
///
/// scratch must point to N writable elements.
[[nodiscard]] inline DechirpResult dechirp_and_quality(
        const std::complex<float>* samples,
        const std::complex<float>* ref_chirp,
        std::complex<float>* scratch, uint32_t N,
        gr::algorithm::FFT<std::complex<float>>& fft,
        bool remove_dc = false) {
    if (remove_dc) {
        std::complex<float> mean{0.f, 0.f};
        for (uint32_t i = 0; i < N; i++) mean += samples[i];
        mean /= static_cast<float>(N);
        for (uint32_t i = 0; i < N; i++) {
            scratch[i] = (samples[i] - mean) * ref_chirp[i];
        }
    } else {
        for (uint32_t i = 0; i < N; i++) {
            scratch[i] = samples[i] * ref_chirp[i];
        }
    }
    auto fft_out = fft.compute(std::span<const std::complex<float>>(scratch, N));

    float max_mag = 0.f;
    float total_mag = 0.f;
    uint32_t max_idx = 0;
    for (uint32_t i = 0; i < N; i++) {
        float mag = std::sqrt(fft_out[i].real() * fft_out[i].real()
                            + fft_out[i].imag() * fft_out[i].imag());
        total_mag += mag;
        if (mag > max_mag) {
            max_mag = mag;
            max_idx = i;
        }
    }
    float mean_mag = total_mag / static_cast<float>(N);
    float pmr = (mean_mag > 0.f) ? (max_mag / mean_mag) : 0.f;
    return {max_idx, pmr};
}

/// Result of soft dechirp: argmax bin, peak-to-mean ratio, and per-bin |Y[s]|².
struct DechirpSoftResult {
    uint32_t           bin;      ///< argmax (hard symbol)
    float              pmr;      ///< peak-to-mean ratio
    std::vector<float> mag_sq;   ///< |Y[s]|² for all M bins
};

/// Dechirp + FFT returning both the hard symbol AND per-bin magnitude-squared
/// for soft-decision decoding (LLR extraction via GrayPartition).
/// NOTE: This allocating version is kept for backward compatibility (tests).
/// Prefer the buffer-taking overload below for real-time paths.
[[nodiscard]] inline DechirpSoftResult dechirp_soft(
        const std::complex<float>* samples,
        const std::complex<float>* ref_chirp,
        std::complex<float>* scratch, uint32_t N,
        gr::algorithm::FFT<std::complex<float>>& fft) {
    for (uint32_t i = 0; i < N; i++)
        scratch[i] = samples[i] * ref_chirp[i];
    auto fft_out = fft.compute(std::span<const std::complex<float>>(scratch, N));

    DechirpSoftResult result;
    result.mag_sq.resize(N);
    float max_val = 0.f, total = 0.f;
    uint32_t max_idx = 0;
    for (uint32_t i = 0; i < N; i++) {
        float ms = fft_out[i].real() * fft_out[i].real()
                 + fft_out[i].imag() * fft_out[i].imag();
        result.mag_sq[i] = ms;
        float mag = std::sqrt(ms);
        total += mag;
        if (ms > max_val) { max_val = ms; max_idx = i; }
    }
    result.bin = max_idx;
    float mean = total / static_cast<float>(N);
    result.pmr = (mean > 0.f) ? (std::sqrt(max_val) / mean) : 0.f;
    return result;
}

/// Dechirp + FFT into pre-allocated buffer. No heap allocation.
/// mag_sq_out must point to N writable floats (caller-provided).
[[nodiscard]] inline DechirpResult dechirp_soft(
        const std::complex<float>* samples,
        const std::complex<float>* ref_chirp,
        std::complex<float>* scratch, uint32_t N,
        gr::algorithm::FFT<std::complex<float>>& fft,
        float* mag_sq_out) {
    for (uint32_t i = 0; i < N; i++)
        scratch[i] = samples[i] * ref_chirp[i];
    auto fft_out = fft.compute(std::span<const std::complex<float>>(scratch, N));

    float max_val = 0.f, total = 0.f;
    uint32_t max_idx = 0;
    for (uint32_t i = 0; i < N; i++) {
        float ms = fft_out[i].real() * fft_out[i].real()
                 + fft_out[i].imag() * fft_out[i].imag();
        mag_sq_out[i] = ms;
        float mag = std::sqrt(ms);
        total += mag;
        if (ms > max_val) { max_val = ms; max_idx = i; }
    }
    float mean = total / static_cast<float>(N);
    float pmr = (mean > 0.f) ? (std::sqrt(max_val) / mean) : 0.f;
    return {max_idx, pmr};
}

}  // namespace gr::lora

#endif  // GNURADIO_LORA_UTILITIES_HPP
