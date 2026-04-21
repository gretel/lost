// SPDX-License-Identifier: ISC

// Phase 5 unit tests: PreambleSync (full iterative Xhonneux §6).
//
// Strategy: synthesize LoRa preamble IQ at Nyquist rate, optionally apply
// a carrier frequency offset (CFO) or sample time offset (STO), drive
// PreambleSync.tick() one symbol at a time, and verify the recovered
// estimates match the inserted offsets within tolerance.
//
// The standard preamble layout this test uses:
//   7 × upchirp(id=0)   [U1..U7, of which sync uses 3+3+1]
//   1 × upchirp(id=sw0) [NID1, skipped by sync]
//   1 × upchirp(id=sw1) [NID2, skipped by sync]
//   1 × downchirp(id=0) [D1]
// = 10 ticks total → PreambleSync transitions to Locked on tick 10.

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/phy/PreambleSync.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <random>
#include <span>
#include <vector>

using namespace boost::ut;
using gr::lora::phy::PreambleSync;
using gr::lora::phy::SyncResult;
using cf32 = std::complex<float>;

namespace {

[[nodiscard]] std::vector<cf32> buildSymbol(uint8_t sf, uint32_t bin_id, bool is_downchirp = false) {
    const uint32_t    N = uint32_t{1} << sf;
    std::vector<cf32> out(N);
    gr::lora::build_upchirp(out.data(), bin_id, sf, /*os_factor=*/1);
    if (is_downchirp) {
        for (auto& s : out) {
            s = std::conj(s);
        }
    }
    return out;
}

/// Construct an 11-symbol standard LoRa preamble (8 upchirps + 2
/// sync-word chirps + 1 downchirp D1 — D2 and quarter are not needed
/// here because PreambleSync locks on D1).
[[nodiscard]] std::vector<std::vector<cf32>> buildPreamble(uint8_t sf, uint16_t sync_word, uint16_t preamble_len = 8) {
    const uint16_t sw0 = static_cast<uint16_t>(((sync_word & 0xF0) >> 4) << 3);
    const uint16_t sw1 = static_cast<uint16_t>((sync_word & 0x0F) << 3);

    std::vector<std::vector<cf32>> symbols;
    for (uint16_t i = 0; i < preamble_len; ++i) {
        symbols.push_back(buildSymbol(sf, 0));
    }
    symbols.push_back(buildSymbol(sf, sw0));
    symbols.push_back(buildSymbol(sf, sw1));
    symbols.push_back(buildSymbol(sf, 0, /*is_downchirp=*/true));
    return symbols;
}

/// Apply a carrier frequency offset (CFO) to a symbol: multiply each
/// sample by exp(j·2π·(cfo_int+cfo_frac)·n/N). `symbol_index` lets the
/// CFO phase continue across symbol boundaries (Xhonneux §4.2 key insight).
void applyCfo(std::vector<cf32>& symbol, uint32_t N, float cfo_int, float cfo_frac, std::size_t symbol_index) {
    const float cfo  = cfo_int + cfo_frac;
    const float incr = 2.f * std::numbers::pi_v<float> * cfo / static_cast<float>(N);
    for (uint32_t n = 0; n < N; ++n) {
        const std::size_t global_n = static_cast<std::size_t>(n) + symbol_index * static_cast<std::size_t>(N);
        const float       phase    = incr * static_cast<float>(global_n);
        symbol[n] *= cf32{std::cos(phase), std::sin(phase)};
    }
}

/// Drive 10 ticks through PreambleSync and return the final SyncResult.
[[nodiscard]] SyncResult drivePreamble(PreambleSync& sync, const std::vector<std::vector<cf32>>& symbols) {
    SyncResult r{};
    for (std::size_t i = 0; i < symbols.size(); ++i) {
        r = sync.tick(std::span<const cf32>(symbols[i].data(), symbols[i].size()));
        if (r.state == SyncResult::State::Locked || r.state == SyncResult::State::Failed) {
            break;
        }
    }
    return r;
}

suite<"PreambleSync clean"> _clean = [] {
    "SF7 clean preamble locks"_test = [] {
        auto                 symbols = buildPreamble(7, 0x12);
        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf = 7;
        // Turn off α threshold for perfectly clean signal — the sinc
        // spreading in the synthetic preamble gives a clean but
        // finite-noise-floor FFT.
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.cfo_int == 0) << "cfo_int=" << r.cfo_int;
        expect(std::abs(r.cfo_frac) < 0.05f) << "cfo_frac=" << r.cfo_frac;
        expect(r.sto_int == 0 || r.sto_int == static_cast<int32_t>(1u << 7)) << "sto_int=" << r.sto_int;
    };

    "SF8 clean preamble locks"_test = [] {
        auto                 symbols = buildPreamble(8, 0x34);
        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = 8;
        cfg.sync_word        = 0x34; // match buildPreamble() sync_word for Stage 3d verify
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.cfo_int == 0);
        expect(std::abs(r.cfo_frac) < 0.05f);
    };

    "SF10 clean preamble locks"_test = [] {
        auto                 symbols = buildPreamble(10, 0x12);
        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = 10;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
    };
};

suite<"PreambleSync integer CFO"> _cfo_int = [] {
    "SF7 integer CFO +3"_test = [] {
        const uint8_t  sf      = 7;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, /*cfo_int=*/3, /*cfo_frac=*/0, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.cfo_int == 3) << "cfo_int=" << r.cfo_int;
    };

    "SF8 integer CFO -5"_test = [] {
        const uint8_t  sf      = 8;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, /*cfo_int=*/-5, /*cfo_frac=*/0, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.cfo_int == -5) << "cfo_int=" << r.cfo_int;
    };
};

suite<"PreambleSync fractional CFO"> _cfo_frac = [] {
    "SF8 fractional CFO +0.3"_test = [] {
        const uint8_t  sf      = 8;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, 0, 0.3f, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        // Tolerance ±0.1 (clean signal, no noise)
        expect(std::abs(r.cfo_frac - 0.3f) < 0.1f) << "cfo_frac=" << r.cfo_frac;
    };

    "SF8 fractional CFO -0.2"_test = [] {
        const uint8_t  sf      = 8;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, 0, -0.2f, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(std::abs(r.cfo_frac - (-0.2f)) < 0.1f) << "cfo_frac=" << r.cfo_frac;
    };
};

suite<"PreambleSync combined CFO"> _cfo_combined = [] {
    "SF8 CFO int=2 frac=0.15"_test = [] {
        const uint8_t  sf      = 8;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, 2, 0.15f, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.cfo_int == 2) << "cfo_int=" << r.cfo_int;
        expect(std::abs(r.cfo_frac - 0.15f) < 0.1f) << "cfo_frac=" << r.cfo_frac;
    };
};

suite<"PreambleSync failure"> _fail = [] {
    "pure noise → Failed or never locks"_test = [] {
        const uint8_t                   sf = 8;
        const uint32_t                  N  = 1u << sf;
        std::mt19937                    rng(42);
        std::normal_distribution<float> noise(0.f, 1.f);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = true;
        cfg.max_detect_ticks = 20;
        sync.init(cfg);

        // Feed 25 ticks of pure noise.
        std::vector<cf32> buf(N);
        SyncResult        r{};
        for (int t = 0; t < 25; ++t) {
            for (auto& s : buf) {
                s = cf32{noise(rng), noise(rng)};
            }
            r = sync.tick(std::span<const cf32>(buf));
            if (r.state == SyncResult::State::Failed) {
                break;
            }
        }
        expect(r.state == SyncResult::State::Failed || r.state == SyncResult::State::Detecting) << "state=" << static_cast<int>(r.state);
        // If the detector fired by chance, it shouldn't have reached Locked.
        expect(r.state != SyncResult::State::Locked);
    };

    "reset allows re-sync"_test = [] {
        const uint8_t sf      = 7;
        auto          symbols = buildPreamble(sf, 0x12);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r1 = drivePreamble(sync, symbols);
        expect(r1.state == SyncResult::State::Locked);

        sync.reset();
        auto r2 = drivePreamble(sync, symbols);
        expect(r2.state == SyncResult::State::Locked);
    };
};

// Regression suite for Xhonneux §6 Stage 2.5 extra-upchirp skip accounting.
//
// Stage 1+2 always consume 6 upchirps (U1..U6) and Stage 3 always consumes
// 1 final upchirp (the s_up measurement at Un). For preamble_len > 7 the
// intermediate upchirps U7..U(n-1) must be consumed by Stage 2.5
// (`_extra_up_remaining = preamble_len - 7`). If that count is wrong,
// Stage 3a/3b/3c/3d land on pure upchirps instead of U_last/NID1/NID2/D1
// and sync word verification rejects the lock.
//
// MeshCore firmware on Heltec V3 transmits with preamble_len = 16 (via
// RadioLib SX126x::begin(...), verified in MeshCore CustomSX1262.h line 45
// and across all MeshCore target.cpp variants 2026-04-09). The default
// config previously used preamble_len = 8, which silently mis-timed
// Stage 3 on every real MeshCore frame and produced the ambient
// SF8/BW62.5k zero-decode symptom tracked in the 2026-04-09 handoff.
suite<"PreambleSync variable preamble length"> _preamble_len = [] {
    "SF8 preamble_len=16 locks"_test = [] {
        const uint8_t sf      = 8;
        auto          symbols = buildPreamble(sf, 0x12, /*preamble_len=*/16);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.preamble_len     = 16;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked) << "state=" << static_cast<int>(r.state);
        expect(r.cfo_int == 0) << "cfo_int=" << r.cfo_int;
        expect(std::abs(r.cfo_frac) < 0.05f) << "cfo_frac=" << r.cfo_frac;
    };

    "SF8 preamble_len=12 locks"_test = [] {
        const uint8_t sf      = 8;
        auto          symbols = buildPreamble(sf, 0x12, /*preamble_len=*/12);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.preamble_len     = 12;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked) << "state=" << static_cast<int>(r.state);
        expect(r.cfo_int == 0) << "cfo_int=" << r.cfo_int;
    };

    "SF7 preamble_len=16 locks"_test = [] {
        const uint8_t sf      = 7;
        auto          symbols = buildPreamble(sf, 0x12, /*preamble_len=*/16);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.preamble_len     = 16;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked) << "state=" << static_cast<int>(r.state);
    };

    "SF8 preamble_len=16 with CFO +4 locks"_test = [] {
        const uint8_t  sf      = 8;
        const uint32_t N       = 1u << sf;
        auto           symbols = buildPreamble(sf, 0x12, /*preamble_len=*/16);
        for (std::size_t i = 0; i < symbols.size(); ++i) {
            applyCfo(symbols[i], N, /*cfo_int=*/4, /*cfo_frac=*/0, i);
        }

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.preamble_len     = 16;
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked) << "state=" << static_cast<int>(r.state);
        expect(r.cfo_int == 4) << "cfo_int=" << r.cfo_int;
    };

    // Hardware failure mode regression guard: if the transmitter uses
    // preamble_len=16 (MeshCore) but the decoder config is stuck at
    // preamble_len=8, Stage 3a/3b/3c should all read pure upchirps from
    // U8/U9/U10, nid1_bin and nid2_bin should match s_up, and sync word
    // verification should reject the lock. This is the exact symptom
    // observed on ambient MeshCore traffic 2026-04-09 (`SYNC_REJECT` lines
    // with `nid1=nid2=s_up`). Without this test a future change to the
    // sync word verifier could silently accept the false lock and
    // reintroduce the zero-decode regression.
    "SF8 cfg.preamble_len=8 rejects real preamble_len=16 signal"_test = [] {
        const uint8_t sf      = 8;
        auto          symbols = buildPreamble(sf, 0x12, /*preamble_len=*/16);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.preamble_len     = 8; // intentional mismatch with transmitter
        cfg.verify_threshold = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Failed) << "state=" << static_cast<int>(r.state);
    };
};

// Promiscuous mode: decoder ignores the configured sync_word and surfaces
// the observed sync_word from NID1/NID2 bins on the SyncResult.
suite<"PreambleSync promiscuous"> _promiscuous = [] {
    "accepts sync 0x12"_test = [] {
        const uint8_t sf      = 7;
        auto          symbols = buildPreamble(sf, 0x12);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf          = sf;
        cfg.sync_word   = 0xDEAD; // would mismatch in strict mode
        cfg.promiscuous = true;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked) << "promiscuous should lock regardless of sync_word";
        expect(r.sync_word_observed == uint32_t{0x12}) << "observed = 0x12, got 0x" << std::hex << r.sync_word_observed;
    };

    "accepts sync 0x34 (LoRaWAN)"_test = [] {
        const uint8_t sf      = 8;
        auto          symbols = buildPreamble(sf, 0x34);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf          = sf;
        cfg.promiscuous = true;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.sync_word_observed == uint32_t{0x34}) << "observed = 0x34, got 0x" << std::hex << r.sync_word_observed;
    };

    "accepts sync 0x00"_test = [] {
        const uint8_t sf      = 7;
        auto          symbols = buildPreamble(sf, 0x00);

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf          = sf;
        cfg.promiscuous = true;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Locked);
        expect(r.sync_word_observed == uint32_t{0x00}) << "observed = 0x00, got 0x" << std::hex << r.sync_word_observed;
    };

    "strict mode still rejects wrong sync"_test = [] {
        // Regression: ensure promiscuous=false preserves the old filter.
        const uint8_t sf      = 7;
        auto          symbols = buildPreamble(sf, 0x34); // TX uses 0x34

        PreambleSync         sync;
        PreambleSync::Config cfg;
        cfg.sf          = sf;
        cfg.sync_word   = 0x12; // RX expects 0x12 — mismatch
        cfg.promiscuous = false;
        sync.init(cfg);

        auto r = drivePreamble(sync, symbols);
        expect(r.state == SyncResult::State::Failed) << "strict 0x12 should reject 0x34 frame";
        expect(r.sync_word_observed == SyncResult::kSyncWordUnknown) << "sync_word_observed stays sentinel in strict mode";
    };
};

} // namespace

int main() { return 0; }
