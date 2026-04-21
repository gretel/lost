// SPDX-License-Identifier: ISC
#ifndef GNURADIO4_LORA_GRAY_PARTITION_HPP
#define GNURADIO4_LORA_GRAY_PARTITION_HPP

#include <cstdint>
#include <vector>

namespace gr::lora {

/// Pre-computed Gray code partition: for each bit position k, which symbol
/// indices have Gray-coded bit k = 1 vs 0.
/// Built once per SF, shared across all SfLanes of the same SF.
struct GrayPartition {
    uint8_t  sf{0};
    uint32_t M{0};
    // ones[k] = symbol indices where Gray bit k is 1 (k=0 is LSB)
    std::vector<std::vector<uint32_t>> ones;
    // zeros[k] = symbol indices where Gray bit k is 0
    std::vector<std::vector<uint32_t>> zeros;

    void init(uint8_t sf_) {
        sf = sf_;
        M  = 1u << sf;
        ones.resize(sf);
        zeros.resize(sf);
        for (uint8_t k = 0; k < sf; k++) {
            ones[k].clear();
            zeros[k].clear();
            ones[k].reserve(M / 2);
            zeros[k].reserve(M / 2);
            for (uint32_t s = 0; s < M; s++) {
                uint32_t g = s ^ (s >> 1); // Gray encode
                if ((g >> k) & 1u) {
                    ones[k].push_back(s);
                } else {
                    zeros[k].push_back(s);
                }
            }
        }
    }
};

} // namespace gr::lora
#endif
