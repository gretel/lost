// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_DETAIL_CHIRP_REFS_HPP
#define GNURADIO_LORA_DETAIL_CHIRP_REFS_HPP

// Stable import surface for upchirp / downchirp reference tables.
// Forwards to the builders in
//   gnuradio-4.0/lora/algorithm/utilities.hpp
//
// These wrappers return owning vectors for convenience; call sites that need
// to avoid an allocation per call should cache the result.

#include <cstdint>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/phy/Types.hpp>

namespace gr::lora::detail {

/// Build an upchirp with the given symbol id (0 = reference), spreading
/// factor, and oversampling factor. Output length = 2^sf * os_factor.
[[nodiscard]] inline std::vector<phy::cf32> build_upchirp(uint8_t sf, uint8_t os_factor = 1, uint32_t id = 0) {
    const std::size_t      len = (std::size_t{1} << sf) * os_factor;
    std::vector<phy::cf32> out(len);
    gr::lora::build_upchirp(out.data(), id, sf, os_factor);
    return out;
}

/// Build a downchirp (complex-conjugated id=0 upchirp), the dechirp
/// reference for the standard LoRa demodulator. Length = 2^sf * os_factor.
[[nodiscard]] inline std::vector<phy::cf32> build_downchirp(uint8_t sf, uint8_t os_factor = 1) {
    const std::size_t      len = (std::size_t{1} << sf) * os_factor;
    std::vector<phy::cf32> up(len);
    std::vector<phy::cf32> down(len);
    gr::lora::build_ref_chirps(up.data(), down.data(), sf, os_factor);
    return down;
}

/// Build the standard reference pair (upchirp id=0, downchirp). Returns a
/// pair of owning vectors, each of length 2^sf * os_factor.
[[nodiscard]] inline std::pair<std::vector<phy::cf32>, std::vector<phy::cf32>> build_ref_pair(uint8_t sf, uint8_t os_factor = 1) {
    const std::size_t      len = (std::size_t{1} << sf) * os_factor;
    std::vector<phy::cf32> up(len);
    std::vector<phy::cf32> down(len);
    gr::lora::build_ref_chirps(up.data(), down.data(), sf, os_factor);
    return {std::move(up), std::move(down)};
}

} // namespace gr::lora::detail

#endif // GNURADIO_LORA_DETAIL_CHIRP_REFS_HPP
