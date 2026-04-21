// SPDX-License-Identifier: ISC

// Phase 1 unit tests: exercise Types, ChirpRefs, and FftPool.
//
// Types: default-construct SyncResult/DemodResult/FrameResult and check
//        default values match the documented contract.
// ChirpRefs: verify build_upchirp(sf=7) returns 128 samples, id=0 starts
//            at (1, 0), downchirp is the conjugate of the upchirp.
// FftPool: acquire FFT for SF=7 twice, verify same instance reused; run
//          FFT of a constant-1 input and verify DC-bin energy dominates.

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/detail/ChirpRefs.hpp>
#include <gnuradio-4.0/lora/detail/FftPool.hpp>
#include <gnuradio-4.0/lora/phy/Types.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <span>
#include <vector>

using namespace boost::ut;
using gr::lora::phy::cf32;

namespace {

suite<"gr::lora Types"> _types = [] {
    "SyncResult defaults"_test = [] {
        gr::lora::phy::SyncResult r{};
        expect(r.state == gr::lora::phy::SyncResult::State::Detecting);
        expect(r.bin_hat == 0u);
        expect(r.cfo_frac == 0.f);
        expect(r.sto_frac == 0.f);
        expect(r.cfo_int == 0);
        expect(r.sto_int == 0);
        expect(r.snr_db == 0.f);
        expect(r.samples_consumed == 0u);
    };

    "DemodResult defaults"_test = [] {
        gr::lora::phy::DemodResult d{};
        expect(d.bin == 0u);
        expect(d.pmr == 0.f);
        expect(d.peak_mag_sq == 0.f);
    };

    "FrameResult defaults"_test = [] {
        gr::lora::phy::FrameResult f{};
        expect(not f.header_valid);
        expect(not f.crc_valid);
        expect(not f.has_crc);
        expect(f.cr == 0);
        expect(f.payload_len == 0);
        expect(f.payload.empty());
        expect(f.sf == 0);
    };
};

suite<"gr::lora ChirpRefs"> _chirps = [] {
    "build_upchirp length SF7 os=1"_test = [] {
        auto up = gr::lora::detail::build_upchirp(7, 1, 0);
        expect(up.size() == 128u);
    };

    "build_upchirp length SF7 os=4"_test = [] {
        auto up = gr::lora::detail::build_upchirp(7, 4, 0);
        expect(up.size() == 512u);
    };

    "build_upchirp length SF12"_test = [] {
        auto up = gr::lora::detail::build_upchirp(12, 1, 0);
        expect(up.size() == 4096u);
    };

    "build_upchirp id=0 starts near (1, 0)"_test = [] {
        // For id=0, the first sample has phase 0 so it should be (1, 0).
        auto up = gr::lora::detail::build_upchirp(8, 1, 0);
        expect(up.size() == 256u);
        expect(std::abs(up[0].real() - 1.f) < 1e-5f);
        expect(std::abs(up[0].imag() - 0.f) < 1e-5f);
    };

    "downchirp is conjugate of upchirp id=0"_test = [] {
        auto [up, down] = gr::lora::detail::build_ref_pair(8, 1);
        expect(up.size() == 256u);
        expect(down.size() == 256u);
        for (std::size_t i = 0; i < up.size(); ++i) {
            expect(std::abs(down[i].real() - up[i].real()) < 1e-6f);
            expect(std::abs(down[i].imag() + up[i].imag()) < 1e-6f);
        }
    };

    "build_downchirp matches build_ref_pair second"_test = [] {
        auto down1      = gr::lora::detail::build_downchirp(7, 1);
        auto [_, down2] = gr::lora::detail::build_ref_pair(7, 1);
        expect(down1.size() == down2.size());
        for (std::size_t i = 0; i < down1.size(); ++i) {
            expect(std::abs(down1[i] - down2[i]) < 1e-6f);
        }
    };
};

suite<"gr::lora FftPool"> _fftpool = [] {
    "acquire same SF returns same instance"_test = [] {
        auto& a = gr::lora::detail::FftPool::acquire(7);
        auto& b = gr::lora::detail::FftPool::acquire(7);
        expect(&a == &b);
    };

    "acquire different SFs returns different instances"_test = [] {
        auto& a = gr::lora::detail::FftPool::acquire(7);
        auto& b = gr::lora::detail::FftPool::acquire(8);
        expect(&a != &b);
    };

    "FFT of constant produces DC peak"_test = [] {
        // Input: all ones. FFT should have energy concentrated in bin 0.
        constexpr std::size_t N = 128;
        std::vector<cf32>     in(N, cf32{1.f, 0.f});
        auto&                 fft = gr::lora::detail::FftPool::acquire(7);
        auto                  out = fft.compute(std::span<const cf32>(in.data(), N));
        expect(out.size() == N);

        // DC bin should contain all the energy.
        const float dc_mag    = std::abs(out[0]);
        float       other_max = 0.f;
        for (std::size_t i = 1; i < N; ++i) {
            const float m = std::abs(out[i]);
            if (m > other_max) {
                other_max = m;
            }
        }
        expect(dc_mag > 100.f * other_max);
    };
};

} // namespace

int main() { return 0; }
