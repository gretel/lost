// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_PHY_TYPES_HPP
#define GNURADIO_LORA_PHY_TYPES_HPP

// Public interface types for the LoRa PHY library.
//
// Pure C++ structs with no GR4 dependencies, so the PreambleSync / CssDemod /
// DecodeChain classes (and their unit tests) can be exercised without a
// scheduler.

#include <complex>
#include <cstdint>
#include <vector>

namespace gr::lora::phy {

using cf32 = std::complex<float>;

/// Result of the PreambleSync state machine (Xhonneux §6 four-stage iterative
/// sync). Populated incrementally as the machine advances; the caller reads
/// it when `state == State::Locked` to drive downstream CssDemod correction.
struct SyncResult {
    enum class State {
        Detecting, ///< Stage 1 (U1-U3): coarse bin + Vangelista α threshold
        Syncing,   ///< Stage 2+3 in progress (U4-U6, U7+D1)
        Locked,    ///< Stage 4 complete, CFO/STO committed
        Failed,    ///< Timed out, low SNR, or CFO out of range
    };

    State    state            = State::Detecting;
    uint32_t bin_hat          = 0;   ///< coarse preamble bin from Stage 1
    float    cfo_frac         = 0.f; ///< fractional CFO (Stage 2)
    float    sto_frac         = 0.f; ///< fractional STO (Stage 4)
    int32_t  cfo_int          = 0;   ///< integer CFO bins (Stage 3)
    int32_t  sto_int          = 0;   ///< integer STO samples (Stage 3)
    float    snr_db           = 0.f;
    float    peak_db          = 0.f;
    float    noise_db         = 0.f;
    uint32_t samples_consumed = 0; ///< from the current `tick()` call

    /// Sync word observed in NID1/NID2 bins.  Valid only in promiscuous mode
    /// (PreambleSync::Config.promiscuous = true).  Sentinel = kSyncWordUnknown.
    static constexpr uint32_t kSyncWordUnknown   = 0xFFFFFFFFu;
    uint32_t                  sync_word_observed = kSyncWordUnknown;
};

/// Result of a single CssDemod call (hard-decision path). The soft path
/// writes a per-bit LLR vector out-of-band.
struct DemodResult {
    uint16_t bin         = 0;   ///< argmax bin index, [0, 2^sf)
    float    pmr         = 0.f; ///< peak-to-mean magnitude ratio
    float    peak_mag_sq = 0.f; ///< magnitude² at the peak bin
};

/// Terminal result of the DecodeChain pipeline: valid when
/// `header_valid || crc_valid`. `payload` is valid only if `header_valid`.
struct FrameResult {
    bool                 header_valid = false; ///< Hamming decode on header 5 nibbles ok
    bool                 crc_valid    = false; ///< payload CRC-16 matches (if has_crc)
    uint8_t              cr           = 0;     ///< coding rate 1..4
    uint16_t             payload_len  = 0;     ///< bytes
    bool                 has_crc      = false;
    std::vector<uint8_t> payload; ///< dewhitened, CR≤4 bytes

    // Frame context (filled in by the driving Per-SF wrapper, not DecodeChain
    // itself — but keeping them here keeps the downstream path simple).
    uint8_t sf     = 0;
    float   snr_db = 0.f;

    // LDRO detection stats (populated by DecodeChain from raw payload bins).
    // `ldro_cfg` is the rule-based assumption the decoder used
    // (Semtech: symbol_time > 16 ms → LDRO on).
    // `payload_lsb0_hits / payload_syms_seen` is the statistical
    // signature: the LDRO interleaver pads the LSB of each output symbol
    // with a zero (MSB-first `[data|parity|zero]` layout), so under LDRO
    // the post-`adjustBin` LSB is ALWAYS zero → fraction ≈ 1.0.
    // Under no-LDRO the bit is ~uniform → fraction ≈ 0.5.
    // Mismatch between cfg and detected → decoder is likely using the
    // wrong interleaver shape; surfaces the problem in telemetry.
    bool     ldro_cfg          = false;
    uint32_t payload_syms_seen = 0; ///< raw payload symbols tested
    uint32_t payload_lsb0_hits = 0; ///< count where (adjustBin(bin) & 1) == 0
};

} // namespace gr::lora::phy

#endif // GNURADIO_LORA_PHY_TYPES_HPP
