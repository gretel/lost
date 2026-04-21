// SPDX-License-Identifier: ISC
//
// Standalone unit tests for ChannelActivityDetector::detect().  Block-level
// scheduler integration is exercised indirectly through qa_lora_scan; this
// suite drives the detect() entry point directly with synthetic input.

#include <boost/ut.hpp>

#include <complex>
#include <cstdint>
#include <random>
#include <vector>

#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace boost::ut;

namespace {

using cf32 = std::complex<float>;

constexpr uint8_t  kSf          = 8;
constexpr uint32_t kBandwidth   = 62500;
constexpr uint32_t kOsFactor    = 4;
constexpr uint8_t  kCr          = 4;
constexpr uint16_t kSync        = 0x12;
constexpr uint16_t kPreambleLen = 8;

[[nodiscard]] gr::lora::ChannelActivityDetector makeCad() {
    gr::lora::ChannelActivityDetector cad;
    cad.sf         = kSf;
    cad.bandwidth  = kBandwidth;
    cad.os_factor  = kOsFactor;
    cad.alpha      = 4.0F;
    cad.dual_chirp = false;
    cad.start(); // builds reference chirps + workspace
    return cad;
}

} // namespace

const suite<"ChannelActivityDetector"> tests = [] {
    "detect on clean preamble"_test = [] {
        // Generate a real frame with preamble at SF8 / BW62.5k / os=4.
        const std::vector<uint8_t> payload = {'A', 'B'};
        auto                       iq      = gr::lora::generate_frame_iq(payload, kSf, kCr, static_cast<uint8_t>(kOsFactor), kSync, kPreambleLen,
            /*has_crc=*/true,
            /*zero_pad=*/0u,
            /*ldro_mode=*/uint8_t{2}, kBandwidth);

        auto cad = makeCad();
        // detect() consumes 2 oversampled symbols (winLen samples).  The frame
        // starts with kPreambleLen consecutive upchirps so any 2-symbol window
        // inside the first preamble_len-1 symbols should fire.
        const std::size_t winLen = static_cast<std::size_t>(1U << kSf) * kOsFactor * 2U;
        expect(iq.size() >= winLen);
        auto r = cad.detect(iq.data(), /*require_both=*/true);
        expect(r.up_detected) << "preamble upchirp pair must trigger up_detected";
        expect(r.peak_ratio_up > cad.alpha) << "ratio=" << r.peak_ratio_up << " alpha=" << static_cast<float>(cad.alpha);
    };

    "reject Gaussian noise"_test = [] {
        std::mt19937             rng(0xC0FFEE);
        std::normal_distribution dist(0.f, 0.1f);
        const std::size_t        winLen = static_cast<std::size_t>(1U << kSf) * kOsFactor * 2U;
        std::vector<cf32>        noise(winLen);
        for (auto& s : noise) {
            s = cf32{dist(rng), dist(rng)};
        }
        auto cad = makeCad();
        auto r   = cad.detect(noise.data(), /*require_both=*/true);
        expect(not r.detected) << "noise must not trigger CAD; up=" << r.peak_ratio_up << " dn=" << r.peak_ratio_dn;
    };

    "compute_alpha grows with SF"_test = [] {
        float prev = 0.f;
        for (uint32_t sfVal = 7; sfVal <= 12; ++sfVal) {
            const float a = gr::lora::ChannelActivityDetector::compute_alpha(sfVal, /*os_factor=*/1U, /*p_fa=*/0.001F);
            expect(a > prev) << "alpha must grow with SF; sf=" << sfVal << " a=" << a;
            prev = a;
        }
    };

    "default_alpha returns 0 outside [7,12]"_test = [] {
        expect(gr::lora::ChannelActivityDetector::default_alpha(6U) == 0.f);
        expect(gr::lora::ChannelActivityDetector::default_alpha(13U) == 0.f);
        expect(gr::lora::ChannelActivityDetector::default_alpha(7U) > 0.f);
    };
};

int main() { return 0; }
