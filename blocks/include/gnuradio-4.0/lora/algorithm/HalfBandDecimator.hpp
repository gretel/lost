// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP
#define GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP

// Cascaded half-band FIR decimator for wideband channelization.
// Replaces the box-car (integrate-and-dump) decimator in ChannelSlot.
//
// Each stage uses a 23-tap half-band FIR with -45.1 dB stopband rejection
// (coefficients from scipy.signal.remez). Half-band symmetry means only
// 6 unique non-zero taps per stage, plus the center tap (0.5).
//
// Decimation by 2^N is achieved by cascading N stages. Each stage
// decimates by 2, so the cascade gives 2^N total decimation.

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace gr::lora {

using cf32 = std::complex<float>;

// 23-tap half-band FIR coefficients.
// Only the 12 unique taps are stored (symmetric: h[k] = h[22-k]).
// Even-indexed taps (except center) are zero for half-band filters.
// Non-zero taps: h[0], h[2], h[4], h[6], h[8], h[10], h[11], h[12], ...
// But h[10]=h[12], h[8]=h[14], etc. due to symmetry.
namespace halfband_detail {

// 6 unique non-zero outer coefficients (symmetric pairs)
inline constexpr std::array<float, 6> kCoeffs = {
    -7.193324475036694e-03f,  // h[0]  = h[22]
     1.361692586579394e-02f,  // h[2]  = h[20]
    -2.628066406628573e-02f,  // h[4]  = h[18]
     4.860759433245809e-02f,  // h[6]  = h[16]
    -9.646258355192097e-02f,  // h[8]  = h[14]
     3.150052960795258e-01f,  // h[10] = h[12]
};

inline constexpr float kCenterTap = 0.5f;  // h[11]
inline constexpr std::size_t kFilterLen = 23;

}  // namespace halfband_detail

// ─── HalfBandStage ──────────────────────────────────────────────────────────
// Single half-band FIR decimation stage (decimate by 2).
// Uses a circular delay line of 23 elements and exploits half-band symmetry
// (even-indexed taps are zero except at the boundaries).

struct HalfBandStage {
    std::array<cf32, halfband_detail::kFilterLen> delay{};
    std::size_t writePos{0};       // circular write pointer
    std::size_t sampleCount{0};    // phase tracking for decimate-by-2

    void reset() noexcept {
        delay.fill(cf32{0.f, 0.f});
        writePos    = 0;
        sampleCount = 0;
    }

    /// Process input samples, appending decimated output to `out`.
    /// For every 2 input samples, produces 1 output sample (decimate-by-2).
    void process(std::span<const cf32> input, std::vector<cf32>& out) {
        using namespace halfband_detail;

        for (const auto& sample : input) {
            // Write new sample into delay line at writePos
            delay[writePos] = sample;

            // Only compute output on every 2nd sample (decimate-by-2)
            if ((sampleCount & 1U) == 1U) {
                // Compute FIR output using half-band symmetry.
                // Tap k maps to delay[(writePos + k + 1) % 23] where
                // writePos points to the just-written newest sample (tap 0
                // of a "newest-first" view). But we want tap k to be the
                // (22-k)-th oldest sample, i.e., delay line index:
                //   idx(k) = (writePos - 22 + k + 23) % 23
                //          = (writePos + k + 1) % 23

                cf32 acc{0.f, 0.f};

                // Symmetric pairs: h[0]*( d[0]+d[22] ), h[2]*( d[2]+d[20] ), ...
                // Tap indices: 0,22  2,20  4,18  6,16  8,14  10,12
                for (std::size_t c = 0; c < kCoeffs.size(); ++c) {
                    const std::size_t k = c * 2;  // tap index: 0, 2, 4, 6, 8, 10
                    const std::size_t kMirror = kFilterLen - 1 - k;  // 22, 20, 18, 16, 14, 12

                    const std::size_t idx1 = (writePos + k + 1) % kFilterLen;
                    const std::size_t idx2 = (writePos + kMirror + 1) % kFilterLen;

                    acc += kCoeffs[c] * (delay[idx1] + delay[idx2]);
                }

                // Center tap: h[11] = 0.5
                const std::size_t centerIdx = (writePos + 11 + 1) % kFilterLen;
                acc += kCenterTap * delay[centerIdx];

                out.push_back(acc);
            }

            // Advance write pointer AFTER computing output
            writePos = (writePos + 1) % kFilterLen;
            ++sampleCount;
        }
    }
};

// ─── CascadedDecimator ──────────────────────────────────────────────────────
// N cascaded half-band stages for decimation by 2^N.
// Uses ping-pong scratch buffers to avoid allocations in the hot path.

struct CascadedDecimator {
    std::vector<HalfBandStage> stages;
    std::vector<cf32>          scratch0;  // ping buffer
    std::vector<cf32>          scratch1;  // pong buffer

    /// Initialize with nStages half-band stages (total decimation = 2^nStages).
    /// maxChunkSize is the maximum expected input chunk size (for pre-allocation).
    void init(std::size_t nStages, std::size_t maxChunkSize = 8192) {
        stages.resize(nStages);
        for (auto& s : stages) {
            s.reset();
        }
        // Pre-allocate scratch buffers for the largest possible intermediate size
        scratch0.reserve(maxChunkSize);
        scratch1.reserve(maxChunkSize);
    }

    /// Reset all stage state (delay lines, phase counters).
    void reset() {
        for (auto& s : stages) {
            s.reset();
        }
    }

    /// Process input through all stages, returning decimated output.
    /// The input is decimated by 2^nStages total.
    [[nodiscard]] std::vector<cf32> process(std::span<const cf32> input) {
        if (stages.empty()) {
            return {input.begin(), input.end()};
        }

        // First stage reads from input
        scratch0.clear();
        stages[0].process(input, scratch0);

        // Subsequent stages ping-pong between scratch buffers
        for (std::size_t i = 1; i < stages.size(); ++i) {
            auto& src = (i & 1U) ? scratch0 : scratch1;
            auto& dst = (i & 1U) ? scratch1 : scratch0;
            dst.clear();
            stages[i].process(std::span<const cf32>(src), dst);
        }

        // Return the final buffer
        auto& result = (stages.size() & 1U) ? scratch0 : scratch1;
        return {result.begin(), result.end()};
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP
