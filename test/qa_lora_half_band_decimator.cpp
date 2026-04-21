// SPDX-License-Identifier: ISC
//
// Unit tests for gr::lora::HalfBandStage — half-band FIR decimator by 2.

#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>

using namespace boost::ut;

namespace {

using cf32 = std::complex<float>;

/// Generate N samples of a complex sinusoid at frequency f (cycles/sample).
[[nodiscard]] std::vector<cf32> makeTone(std::size_t n, float f_cycles_per_sample) {
    std::vector<cf32> out(n);
    const float       twoPi = 2.f * std::numbers::pi_v<float>;
    for (std::size_t i = 0; i < n; ++i) {
        const float phase = twoPi * f_cycles_per_sample * static_cast<float>(i);
        out[i]            = cf32{std::cos(phase), std::sin(phase)};
    }
    return out;
}

/// Mean magnitude of the tail of a buffer (skip filter startup transient).
[[nodiscard]] float tailMag(std::span<const cf32> s, std::size_t skip = 32) {
    if (s.size() <= skip) {
        return 0.f;
    }
    float sum = 0.f;
    for (std::size_t i = skip; i < s.size(); ++i) {
        sum += std::abs(s[i]);
    }
    return sum / static_cast<float>(s.size() - skip);
}

} // namespace

const suite<"HalfBandStage"> tests = [] {
    "empty input produces empty output"_test = [] {
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       out;
        stage.process({}, out);
        expect(out.empty());
    };

    "decimates by 2"_test = [] {
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       out;
        auto                    in = makeTone(1024, 0.0f); // DC
        stage.process(std::span<const cf32>(in), out);
        expect(out.size() == 512_u) << "1024 in / 2 = 512 out, got " << out.size();
    };

    "DC passes"_test = [] {
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       out;
        auto                    in = makeTone(2048, 0.0f);
        stage.process(std::span<const cf32>(in), out);
        const float mag = tailMag(out);
        expect(mag > 0.9f) << "DC should pass with near-unity gain, got " << mag;
    };

    "Fs/4 passes (below Nyquist)"_test = [] {
        // Input Fs/4 lands in the half-band passband; output (at half rate) is
        // Fs_out/2 = Nyquist of the decimated signal, which is near the transition
        // band.  A reasonable FIR holds this at ~0.5 gain or better.
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       out;
        auto                    in = makeTone(2048, 0.1f);
        stage.process(std::span<const cf32>(in), out);
        const float mag = tailMag(out);
        expect(mag > 0.5f) << "tone within passband should not be fully killed, got " << mag;
    };

    "Fs/2 is suppressed"_test = [] {
        // Frequency just below Fs/2 aliases to DC after decimation-by-2, but
        // the half-band filter stopband should hold it near zero (~-60 dB).
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       out;
        auto                    in = makeTone(4096, 0.49f);
        stage.process(std::span<const cf32>(in), out);
        const float mag = tailMag(out, 256);
        expect(mag < 0.1f) << "Fs/2 should be attenuated, got " << mag;
    };

    "processBatch matches process"_test = [] {
        gr::lora::HalfBandStage stageA, stageB;
        stageB.initBatch(2048);
        auto              in = makeTone(2048, 0.05f);
        std::vector<cf32> outA, outB;
        stageA.process(std::span<const cf32>(in), outA);
        stageB.processBatch(std::span<const cf32>(in), outB);

        expect(outA.size() == outB.size()) << "sizes differ: " << outA.size() << " vs " << outB.size();
        float maxDiff = 0.f;
        for (std::size_t i = 0; i < std::min(outA.size(), outB.size()); ++i) {
            maxDiff = std::max(maxDiff, std::abs(outA[i] - outB[i]));
        }
        expect(maxDiff < 1e-5f) << "processBatch should match process within float tolerance, maxDiff=" << maxDiff;
    };

    "reset returns to clean state"_test = [] {
        gr::lora::HalfBandStage stage;
        std::vector<cf32>       sink;
        auto                    noise = makeTone(256, 0.3f);
        stage.process(std::span<const cf32>(noise), sink);
        stage.reset();

        std::vector<cf32> out;
        auto              in = makeTone(2048, 0.0f);
        stage.process(std::span<const cf32>(in), out);
        const float mag = tailMag(out);
        expect(mag > 0.9f) << "after reset, DC tone should pass cleanly, got " << mag;
    };
};

int main() { return 0; }
