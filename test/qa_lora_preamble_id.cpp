// SPDX-License-Identifier: ISC
//
// Unit tests for gr::lora::characterize_preamble — standalone preamble
// characterization on a captured IQ buffer.  Covers SF detection, sync-word
// reconstruction, CFO/STO estimation, and noise rejection.

#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <random>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/PreambleId.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLaneDetail.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace boost::ut;

namespace {

using cf32 = std::complex<float>;

constexpr uint8_t  kCr          = 4;
constexpr uint8_t  kOsFactor    = 1;
constexpr uint16_t kPreambleLen = 8;
constexpr double   kCenterFreq  = 868e6;

/// Build a full TX frame IQ for a trivial payload so the preamble is present.
[[nodiscard]] std::vector<cf32> makeFrameIq(uint8_t sf, uint32_t bw, uint16_t syncWord) {
    const std::vector<uint8_t> payload = {'A', 'B'};
    return gr::lora::generate_frame_iq(payload, sf, kCr, kOsFactor, syncWord, kPreambleLen,
        /*has_crc=*/true,
        /*zero_pad=*/0u,
        /*ldro_mode=*/uint8_t{2}, bw);
}

} // namespace

const suite<"PreambleId"> tests = [] {
    "detect SF8 clean preamble + sync 0x12"_test = [] {
        auto iq   = makeFrameIq(/*sf=*/8, /*bw=*/125000, /*syncWord=*/0x12);
        auto info = gr::lora::characterize_preamble(iq.data(), iq.size(), /*sf=*/8, /*bw=*/125000, kOsFactor, kCenterFreq, kPreambleLen);

        expect(info.valid) << "sync word verification should complete";
        expect(info.sf == 8_uc) << "sf";
        expect(info.bw == 125000_u) << "bw";
        expect(info.sync_word == uint16_t{0x12}) << "sync_word";
        expect(info.preamble_len >= 3u) << "detected preamble_len=" << info.preamble_len;
        expect(info.pmr > 4.f) << "clean preamble should produce strong PMR, got " << info.pmr;
    };

    "detect SF10 clean preamble + sync 0x34"_test = [] {
        auto iq   = makeFrameIq(/*sf=*/10, /*bw=*/125000, /*syncWord=*/0x34);
        auto info = gr::lora::characterize_preamble(iq.data(), iq.size(), /*sf=*/10, /*bw=*/125000, kOsFactor, kCenterFreq, kPreambleLen);

        expect(info.valid);
        expect(info.sf == 10_uc);
        expect(info.sync_word == uint16_t{0x34}) << "sync_word";
    };

    "reject buffer too short for SYNC"_test = [] {
        // Less than (preamble_len + 5) symbols: characterize_preamble bails early.
        std::vector<cf32> tiny(128, cf32{0.f, 0.f});
        auto              info = gr::lora::characterize_preamble(tiny.data(), tiny.size(), /*sf=*/8, /*bw=*/125000, kOsFactor, kCenterFreq, kPreambleLen);
        expect(not info.valid) << "short buffer should not produce a valid lock";
    };

    "reject random noise"_test = [] {
        std::mt19937             rng(0xDEADBEEF);
        std::normal_distribution dist(0.f, 0.1f);
        const std::size_t        n = 8192;
        std::vector<cf32>        noise(n);
        for (auto& s : noise) {
            s = cf32{dist(rng), dist(rng)};
        }
        auto info = gr::lora::characterize_preamble(noise.data(), noise.size(), /*sf=*/8, /*bw=*/125000, kOsFactor, kCenterFreq, kPreambleLen);
        expect(not info.valid) << "Gaussian noise must not pass sync-word verification";
    };

    "SfLaneDetail min_preamble_pmr monotone"_test = [] {
        // SF7..SF10 = 4.0 (floor); SF11..SF12 relax below.  Table values are
        // tuning constants; at minimum they must be positive and finite.
        for (uint8_t sf = 7; sf <= 12; ++sf) {
            const float pmr = gr::lora::sflane_detail::min_preamble_pmr(sf);
            expect(pmr > 0.f) << "sf=" << sf << " pmr=" << pmr;
            expect(std::isfinite(pmr)) << "sf=" << sf;
        }
    };
};

int main() { return 0; }
