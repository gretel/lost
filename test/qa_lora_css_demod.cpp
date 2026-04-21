// SPDX-License-Identifier: ISC

// Phase 4 unit tests: CssDemod hard + soft demodulation.
//
// Strategy: build a clean upchirp at a known bin id via the
// build_upchirp helper, dechirp it via CssDemod, and verify the recovered
// argmax bin matches.

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/phy/CssDemod.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <span>
#include <vector>

using namespace boost::ut;
using gr::lora::phy::CssDemod;
using gr::lora::phy::DemodResult;
using cf32 = std::complex<float>;

namespace {

[[nodiscard]] std::vector<cf32> makeUpchirpAtBin(uint8_t sf, uint32_t bin_id) {
    const uint32_t    N = uint32_t{1} << sf;
    std::vector<cf32> chirp(N);
    gr::lora::build_upchirp(chirp.data(), bin_id, sf, /*os_factor=*/1);
    return chirp;
}

suite<"CssDemod hard"> _hard = [] {
    "SF7 bin 0"_test = [] {
        CssDemod d;
        d.init({7, 125000, false});
        auto chirp = makeUpchirpAtBin(7, 0);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 0_u);
        expect(r.pmr > 4.f) << "pmr=" << r.pmr;
    };

    "SF7 bin 42"_test = [] {
        CssDemod d;
        d.init({7, 125000, false});
        auto chirp = makeUpchirpAtBin(7, 42);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 42_u);
        expect(r.pmr > 4.f) << "pmr=" << r.pmr;
    };

    "SF7 bin 127 (last)"_test = [] {
        CssDemod d;
        d.init({7, 125000, false});
        auto chirp = makeUpchirpAtBin(7, 127);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 127_u);
    };

    "SF8 bin 137"_test = [] {
        CssDemod d;
        d.init({8, 125000, false});
        auto chirp = makeUpchirpAtBin(8, 137);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 137_u);
    };

    "SF12 bin 1234"_test = [] {
        CssDemod d;
        d.init({12, 125000, false});
        auto chirp = makeUpchirpAtBin(12, 1234);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 1234_u);
    };

    "SF7 sweep all bins"_test = [] {
        CssDemod d;
        d.init({7, 125000, false});
        for (uint32_t k = 0; k < 128; ++k) {
            auto chirp = makeUpchirpAtBin(7, k);
            auto r     = d.demodHard(std::span<const cf32>(chirp));
            expect(r.bin == static_cast<uint16_t>(k)) << "k=" << k;
        }
    };

    "rejects undersized input"_test = [] {
        CssDemod d;
        d.init({8, 125000, false});
        std::vector<cf32> too_small(64); // expects 256
        auto              r = d.demodHard(std::span<const cf32>(too_small));
        expect(r.bin == 0_u); // default-constructed
        expect(r.pmr == 0.f);
    };
};

suite<"CssDemod soft"> _soft = [] {
    "SF7 LLR vector size and finite"_test = [] {
        CssDemod d;
        d.init({7, 125000, true});
        auto                chirp = makeUpchirpAtBin(7, 31);
        std::vector<double> llrs(7);
        auto                r = d.demodSoft(std::span<const cf32>(chirp), std::span<double>(llrs));
        expect(r.bin == 31_u);
        for (auto v : llrs) {
            expect(std::isfinite(v));
        }
    };

    "SF8 hard agrees with soft on clean signal"_test = [] {
        CssDemod hard, soft;
        hard.init({8, 125000, false});
        soft.init({8, 125000, true});
        std::vector<double> llrs(8);

        for (uint32_t k : {0u, 1u, 100u, 200u, 255u}) {
            auto chirp = makeUpchirpAtBin(8, k);
            auto rh    = hard.demodHard(std::span<const cf32>(chirp));
            auto rs    = soft.demodSoft(std::span<const cf32>(chirp), std::span<double>(llrs));
            expect(rh.bin == rs.bin) << "k=" << k;
        }
    };

    "soft path peak_mag_sq populated"_test = [] {
        CssDemod d;
        d.init({7, 125000, true});
        auto                chirp = makeUpchirpAtBin(7, 50);
        std::vector<double> llrs(7);
        auto                r = d.demodSoft(std::span<const cf32>(chirp), std::span<double>(llrs));
        expect(r.peak_mag_sq > 0.f);
    };
};

suite<"CssDemod CFO correction"> _cfo = [] {
    "set_cfo_correction(0,0) is identity"_test = [] {
        // After set_cfo_correction(0, 0), the downchirp ref should still
        // detect a clean upchirp at the original bin.
        CssDemod d;
        d.init({8, 125000, false});
        d.set_cfo_correction(0.0f, 0);
        auto chirp = makeUpchirpAtBin(8, 100);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 100_u);
    };

    "integer CFO shifts the perceived bin"_test = [] {
        // With cfo_int=5, the dechirp reference uses upchirp(id=5) instead
        // of upchirp(id=0). Dechirping a clean upchirp at id=100 produces
        // an FFT peak at bin (100 - 5) mod N = 95 — verifying the CFO
        // correction reference is being applied.
        CssDemod d;
        d.init({8, 125000, false});
        d.set_cfo_correction(0.0f, /*cfo_int=*/5);
        auto chirp = makeUpchirpAtBin(8, 100);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        // Off-by-one tolerance: depending on the chirp fold position the
        // shift can be ±1 from the analytical answer.
        const int diff = static_cast<int>(r.bin) - 95;
        expect(std::abs(diff) <= 1) << "bin=" << r.bin << " expected~95";
    };

    "reset_cfo_correction restores identity"_test = [] {
        CssDemod d;
        d.init({8, 125000, false});
        d.set_cfo_correction(0.3f, 7);
        d.reset_cfo_correction();
        auto chirp = makeUpchirpAtBin(8, 50);
        auto r     = d.demodHard(std::span<const cf32>(chirp));
        expect(r.bin == 50_u);
    };
};

} // namespace

int main() { return 0; }
