// SPDX-License-Identifier: ISC

// Phase 2 unit tests: Vangelista Rayleigh-derived α threshold.
//
// Reference values come from vangelista_cad.md §7.3:
//
//   p0 = 0.01 (1% false alarm)
//     SF=7,  L=1024   → α ≈ 3.83 (theory)
//     SF=8,  L=2048   → α ≈ 3.94
//     SF=9,  L=4096   → α ≈ 4.05
//     SF=10, L=8192   → α ≈ 4.16
//     SF=11, L=16384  → α ≈ 4.27
//     SF=12, L=32768  → α ≈ 4.38
//
// Tolerance: ±0.02 (matches paper-reported precision)

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/scan/VangelistaThreshold.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

using namespace boost::ut;
using gr::lora::scan::alpha_for_sf;
using gr::lora::scan::alpha_for_window;
using gr::lora::scan::compute_alpha;
using gr::lora::scan::detect;

namespace {

constexpr double kTol = 0.02;

suite<"vangelista α"> _alpha = [] {
    "compute_alpha SF7"_test = [] {
        const double a = compute_alpha(0.01, 8u * 128u);
        expect(std::abs(a - 3.83) < kTol) << "α=" << a;
    };

    "compute_alpha SF8"_test = [] {
        const double a = compute_alpha(0.01, 8u * 256u);
        expect(std::abs(a - 3.94) < kTol) << "α=" << a;
    };

    "compute_alpha SF9"_test = [] {
        const double a = compute_alpha(0.01, 8u * 512u);
        expect(std::abs(a - 4.05) < kTol) << "α=" << a;
    };

    "compute_alpha SF10"_test = [] {
        const double a = compute_alpha(0.01, 8u * 1024u);
        expect(std::abs(a - 4.16) < kTol) << "α=" << a;
    };

    "compute_alpha SF11"_test = [] {
        const double a = compute_alpha(0.01, 8u * 2048u);
        expect(std::abs(a - 4.27) < kTol) << "α=" << a;
    };

    "compute_alpha SF12"_test = [] {
        const double a = compute_alpha(0.01, 8u * 4096u);
        expect(std::abs(a - 4.38) < kTol) << "α=" << a;
    };

    "alpha_for_sf matches compute_alpha"_test = [] {
        for (uint8_t sf = 7; sf <= 12; ++sf) {
            const double by_sf   = alpha_for_sf(sf);
            const double by_comp = compute_alpha(0.01, 8u * (std::size_t{1} << sf));
            expect(std::abs(by_sf - by_comp) < 1e-12) << "sf=" << sf;
        }
    };

    "alpha grows monotonically with L"_test = [] {
        double prev = 0.0;
        for (uint8_t sf = 7; sf <= 12; ++sf) {
            const double a = alpha_for_sf(sf);
            expect(a > prev);
            prev = a;
        }
    };

    "alpha_for_window standalone"_test = [] {
        // For L=1024 (SF7 equivalent bin count), α_window == α_for_sf(7)·0
        // wait no — alpha_for_sf uses L=8·M, alpha_for_window uses L=N.
        // So alpha_for_window(1024) should match compute_alpha(0.01, 1024).
        const double a   = alpha_for_window(1024);
        const double ref = compute_alpha(0.01, 1024);
        expect(std::abs(a - ref) < 1e-12);
    };

    "degenerate inputs return 0"_test = [] {
        expect(compute_alpha(0.0, 1024) == 0.0);
        expect(compute_alpha(1.0, 1024) == 0.0);
        expect(compute_alpha(0.01, 0) == 0.0);
    };
};

suite<"vangelista detect()"> _detect = [] {
    "spike rejects noise floor"_test = [] {
        // 128 zeros with a single spike of 100 at bin 5. Mean = 100/128 ≈ 0.78.
        // α · mean = 3.83 · 0.78 ≈ 3.0. Peak = 100 >> 3.0 → detect = true.
        std::vector<float> mags(128, 0.f);
        mags[5]            = 100.f;
        const double alpha = alpha_for_sf(7);
        expect(detect(std::span<const float>(mags), alpha));
    };

    "uniform rejects"_test = [] {
        // 1024 ones → mean = 1, peak = 1. α ≈ 3.83 > 1 → detect = false.
        std::vector<float> mags(1024, 1.f);
        const double       alpha = alpha_for_sf(7);
        expect(not detect(std::span<const float>(mags), alpha));
    };

    "just-above-threshold detects"_test = [] {
        // 1023 zeros, 1 spike at alpha·mean + 1. Mean = spike/1024. Need:
        //   spike > α · spike/1024
        //   1024 > α   → trivially true for any reasonable α.
        // So any non-zero spike in a sea of zeros should trigger detect.
        std::vector<float> mags(1024, 0.f);
        mags[42]           = 1e-6f;
        const double alpha = alpha_for_sf(7);
        expect(detect(std::span<const float>(mags), alpha));
    };

    "empty span returns false"_test = [] {
        std::vector<float> mags;
        expect(not detect(std::span<const float>(mags), 3.83));
    };

    "noise only (all equal) rejects"_test = [] {
        // Synthetic Rayleigh-like noise where the peak is exactly at the mean.
        std::vector<float> mags;
        for (int i = 0; i < 2048; ++i) {
            mags.push_back(0.5f + 0.01f * static_cast<float>(i % 7));
        }
        // Peak ≈ 0.5 + 0.06 = 0.56, mean ≈ 0.53. Ratio ≈ 1.06 << α.
        const double alpha = alpha_for_sf(8);
        expect(not detect(std::span<const float>(mags), alpha));
    };
};

} // namespace

int main() { return 0; }
