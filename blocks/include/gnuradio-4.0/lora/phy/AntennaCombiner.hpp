// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_PHY_ANTENNA_COMBINER_HPP
#define GNURADIO_LORA_PHY_ANTENNA_COMBINER_HPP

// AntennaCombiner scaffold for MALoRa §4 coherent multi-antenna combining.
//
// For N=1 (single-antenna, the current hardware), combine() is a
// pass-through memcpy. The interface exists so that a future second-
// antenna hardware addition can slot MALoRa combining in between raw
// input decimation and PreambleSync without touching the sync/demod
// stages.
//
// Reference: ~/.config/opencode/skills/lora-sdr-impl/references/malora_multiantenna.md
//
// MALoRa combining (N > 1) performs:
//   1. Phase estimation from the preamble (conjugate multiply between
//      antenna 0 and each other antenna, averaged over a few upchirps).
//   2. Per-antenna complex weight: conj(phi_n) for antenna n.
//   3. Coherent sum: out[k] = sum_n antenna[n][k] * conj(phi_n).
//
// The combined stream has N_ant× SNR gain (10·log10(N_ant) dB) and the
// same sample rate as each antenna stream. Requires shared hardware clock
// across all antennas (otherwise per-antenna CFO must also be estimated).
//
// Only the N=1 pass-through is implemented. N>1 asserts with a clear
// message so a multi-antenna config cannot ship without real combining.

#include <cassert>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include <gnuradio-4.0/lora/phy/Types.hpp>

namespace gr::lora::phy {

class AntennaCombiner {
public:
    struct Config {
        uint8_t  sf           = 7;
        uint32_t bandwidth_hz = 125000;
        uint8_t  n_antennas   = 1; ///< >= 2 is future hardware (not implemented)
    };

    void init(const Config& cfg) noexcept { _cfg = cfg; }

    /// Combine `antennas.size()` input streams into a single output stream.
    /// For N_ant=1: `out` is a byte-wise copy of `antennas[0]`.
    /// For N_ant>1: would perform MALoRa §4 coherent combining, but is
    /// currently unimplemented and will abort via assertion to prevent
    /// silent data loss.
    ///
    /// All spans must have the same length; `out.size()` must equal
    /// `antennas[0].size()`. Mismatched sizes are truncated to the shortest.
    void combine(std::span<const std::span<const cf32>> antennas, std::span<cf32> out) noexcept {
        if (antennas.empty() || out.empty()) {
            return;
        }
        if (_cfg.n_antennas == 1 || antennas.size() == 1) {
            const auto n = std::min(antennas[0].size(), out.size());
            std::memcpy(out.data(), antennas[0].data(), n * sizeof(cf32));
            return;
        }

        // N_ant > 1: MALoRa combining is a TODO until second antenna
        // hardware exists. Fail loudly (assert in debug, zero the output
        // in release) rather than silently dropping data.
        assert(false && "MALoRa multi-antenna combining not yet implemented");
        std::memset(out.data(), 0, out.size() * sizeof(cf32));
    }

    [[nodiscard]] uint8_t n_antennas() const noexcept { return _cfg.n_antennas; }

private:
    Config _cfg;
};

} // namespace gr::lora::phy

#endif // GNURADIO_LORA_PHY_ANTENNA_COMBINER_HPP
