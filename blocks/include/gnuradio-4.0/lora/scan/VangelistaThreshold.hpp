// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SCAN_VANGELISTA_THRESHOLD_HPP
#define GNURADIO_LORA_SCAN_VANGELISTA_THRESHOLD_HPP

// Rayleigh-derived adaptive CAD threshold (Vangelista §7).
//
// For an L-bin DFT of additive white Gaussian noise, the magnitude of each
// bin is Rayleigh-distributed. Declaring "signal present" iff
//
//     max(|Y|) > α · mean(|Y|)
//
// with
//
//     α = sqrt( -(4/π) · ln(1 − (1 − p₀)^(1/L)) )
//
// gives a uniform false-alarm probability of p₀ across the L bins,
// independent of the noise power and independent of the actual SNR.
//
// For standard LoRa CAD with the 8-position sliding window:
//     L = 8 · M = 8 · 2^SF
//
// For the wideband sliding-window scanner (SlideLoRaScanner):
//     L = N_window
//
// Reference: ~/.config/opencode/skills/lora-sdr-impl/references/vangelista_cad.md

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <span>

namespace gr::lora::scan {

/// Compute the Rayleigh-derived detection multiplier α for a target false-
/// alarm probability `p0` and `L` total tested DFT bins.
[[nodiscard]] inline double compute_alpha(double p0, std::size_t L) noexcept {
    // Clamp degenerate cases to keep the log finite. The caller should
    // never actually hit these, but returning a large but finite α is
    // safer than returning NaN.
    if (p0 <= 0.0 || p0 >= 1.0 || L == 0) {
        return 0.0;
    }
    const double inner = 1.0 - std::pow(1.0 - p0, 1.0 / static_cast<double>(L));
    // inner is in (0, 1). The log is negative, so -(4/π) · log is positive.
    const double radicand = -(4.0 / std::numbers::pi) * std::log(inner);
    return std::sqrt(radicand);
}

/// Standard LoRa CAD: L = 8 · 2^SF (the 8-position slide tests 8 windows
/// of M bins each). `p0` defaults to 1% false-alarm, matching the paper.
[[nodiscard]] inline double alpha_for_sf(uint8_t sf, double p0 = 0.01) noexcept {
    const std::size_t M = std::size_t{1} << sf;
    const std::size_t L = 8 * M;
    return compute_alpha(p0, L);
}

/// Wideband scanner threshold for a single sliding window of N_window bins.
[[nodiscard]] inline double alpha_for_window(std::size_t n_window, double p0 = 0.01) noexcept { return compute_alpha(p0, n_window); }

/// Detection predicate: returns true iff the peak magnitude of `bin_mags`
/// exceeds α times the mean magnitude.
///
/// `bin_mags` contains non-negative per-bin magnitudes (or magnitude
/// squares — the test is scale-invariant as long as it matches how α was
/// derived; the Vangelista paper uses magnitudes).
[[nodiscard]] inline bool detect(std::span<const float> bin_mags, double alpha) noexcept {
    if (bin_mags.empty()) {
        return false;
    }
    const double sum  = std::accumulate(bin_mags.begin(), bin_mags.end(), 0.0, [](double acc, float v) { return acc + static_cast<double>(v); });
    const double mean = sum / static_cast<double>(bin_mags.size());
    const float  peak = *std::ranges::max_element(bin_mags);
    return static_cast<double>(peak) > alpha * mean;
}

} // namespace gr::lora::scan

#endif // GNURADIO_LORA_SCAN_VANGELISTA_THRESHOLD_HPP
