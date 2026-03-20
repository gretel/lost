// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_SFLANE_DETAIL_HPP
#define GNURADIO_LORA_ALGORITHM_SFLANE_DETAIL_HPP

// Shared constants and helper functions for FrameSync / MultiSfDecoder / WidebandDecoder.
// Extracted from MultiSfDecoder.hpp's multisf_detail namespace to enable reuse
// without GR_REGISTER_BLOCK include conflicts.

#include <cmath>
#include <complex>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace gr::lora::sflane_detail {

// Detection thresholds (shared with FrameSync, MultiSfDecoder, WidebandDecoder)
static constexpr int      kPreambleBinTolerance  = 1;
static constexpr int      kSyncWordBinTolerance  = 2;
static constexpr uint8_t  kMaxAdditionalUpchirps = 3;
static constexpr uint8_t  kMaxPreambleRotations  = 4;
static constexpr float    kMinPreamblePMR        = 4.0f;

[[nodiscard]] inline int most_frequent(const std::vector<int>& arr) {
    std::unordered_map<int, int> freq;
    for (int v : arr) freq[v]++;
    int max_count = 0, result = -1;
    for (auto& [val, cnt] : freq) {
        if (cnt > max_count) { max_count = cnt; result = val; }
    }
    return result;
}

[[nodiscard]] inline int my_roundf(float x) noexcept {
    return x > 0 ? static_cast<int>(x + 0.5f)
                 : static_cast<int>(std::ceil(x - 0.5f));
}

inline void complex_multiply(std::complex<float>* out, const std::complex<float>* a,
                             const std::complex<float>* b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) out[i] = a[i] * b[i];
}

}  // namespace gr::lora::sflane_detail

#endif  // GNURADIO_LORA_ALGORITHM_SFLANE_DETAIL_HPP
