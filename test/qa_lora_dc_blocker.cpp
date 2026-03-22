// SPDX-License-Identifier: ISC
/// Tests for DCBlocker algorithm: 2nd-order Butterworth DC blocker.
///
/// Test progression:
///   1. DC tone at 0 Hz fully rejected (>40 dB attenuation)
///   2. Signal at 100 Hz passes with minimal loss (<1 dB)
///   3. Reset clears filter state
///   4. In-place processing (in == out span)
///   5. Uninitialised blocker passes samples through unchanged
///   6. Cutoff accuracy: -3 dB at specified cutoff frequency

#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <numeric>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/algorithm/DCBlocker.hpp>

using namespace boost::ut;
using namespace boost::ut::literals;

namespace {

using cf32 = std::complex<float>;

constexpr float kSampleRate = 250'000.f;  // narrowband rate
constexpr float kCutoffHz   = 10.f;

// settling time: ~5 time constants for a 2nd-order Butterworth
// τ = 1/(2π·fc), need ~5τ samples at sample rate fs
constexpr std::size_t settlingsamples(float fs, float fc) {
    return static_cast<std::size_t>(5.0 * fs / (2.0 * std::numbers::pi * static_cast<double>(fc)));
}

std::vector<cf32> generateTone(float fHz, float sampleRate, std::size_t nSamples, float amplitude = 1.f) {
    std::vector<cf32> out(nSamples);
    const float omega = 2.f * std::numbers::pi_v<float> * fHz / sampleRate;
    for (std::size_t i = 0; i < nSamples; ++i) {
        const float phase = omega * static_cast<float>(i);
        out[i] = amplitude * cf32{std::cos(phase), std::sin(phase)};
    }
    return out;
}

float rms(std::span<const cf32> signal) {
    double sum = 0.0;
    for (const auto& s : signal) {
        sum += static_cast<double>(std::norm(s));
    }
    return static_cast<float>(std::sqrt(sum / static_cast<double>(signal.size())));
}

}  // namespace

const suite<"DCBlocker algorithm"> dcBlockerTests = [] {
    "DC tone fully rejected"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);
        expect(dc.initialised());

        // need enough samples for the filter to settle
        const std::size_t settle = settlingsamples(kSampleRate, kCutoffHz);
        const std::size_t N      = settle + 50'000;

        std::vector<cf32> input(N, cf32{0.5f, -0.3f});
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        // measure only after full settling
        auto steadyState = std::span(output).subspan(settle);
        float outRms = rms(steadyState);
        float inRms  = rms(std::span(input).subspan(settle));

        float attenDb = 20.f * std::log10(outRms / inRms);
        std::fprintf(stderr, "DC rejection: %.1f dB (settle=%zu, N=%zu)\n", attenDb, settle, N);
        expect(attenDb < -40.f) << "DC should be rejected by >40 dB, got" << attenDb << "dB";
    };

    "signal at 100 Hz passes through"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);

        const std::size_t settle = settlingsamples(kSampleRate, kCutoffHz);
        const std::size_t N      = settle + 25'000;
        auto input = generateTone(100.f, kSampleRate, N);
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        auto inSteady  = std::span(input).subspan(settle);
        auto outSteady = std::span(output).subspan(settle);

        float lossDb = 20.f * std::log10(rms(outSteady) / rms(inSteady));
        std::fprintf(stderr, "100 Hz passthrough loss: %.2f dB\n", lossDb);
        expect(lossDb > -1.f) << "100 Hz should pass with <1 dB loss, got" << lossDb << "dB";
    };

    "signal at 1 kHz unaffected"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);

        const std::size_t settle = settlingsamples(kSampleRate, kCutoffHz);
        const std::size_t N      = settle + 25'000;
        auto input = generateTone(1000.f, kSampleRate, N);
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        float lossDb = 20.f * std::log10(rms(std::span(output).subspan(settle)) / rms(std::span(input).subspan(settle)));
        std::fprintf(stderr, "1 kHz passthrough loss: %.3f dB\n", lossDb);
        expect(lossDb > -0.1f) << "1 kHz should be essentially unaffected, got" << lossDb << "dB";
    };

    "reset clears filter state"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);

        constexpr std::size_t N = 10'000;
        auto input = generateTone(50.f, kSampleRate, N);
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        dc.reset();

        // after reset, first output to a DC step should be a transient (filter has no memory)
        std::vector<cf32> dcInput(1'000, cf32{1.f, 0.f});
        std::vector<cf32> dcOutput(1'000);
        dc.processBlock(dcInput, dcOutput);

        expect(std::abs(dcOutput[0]) > 0.01f) << "first sample should have transient after reset";
    };

    "in-place processing"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);

        constexpr std::size_t N = 10'000;
        auto data = generateTone(500.f, kSampleRate, N);
        auto original = data;

        std::vector<cf32> reference(N);
        dc.processBlock(original, reference);

        dc.reset();
        dc.processBlock(std::span<const cf32>(data), std::span<cf32>(data));

        for (std::size_t i = 0; i < N; ++i) {
            expect(std::abs(data[i].real() - reference[i].real()) < 1e-6f);
            expect(std::abs(data[i].imag() - reference[i].imag()) < 1e-6f);
        }
    };

    "uninitialised blocker is passthrough"_test = [] {
        gr::lora::DCBlocker dc;
        expect(!dc.initialised());

        constexpr std::size_t N = 100;
        std::vector<cf32> input(N);
        for (std::size_t i = 0; i < N; ++i) {
            input[i] = cf32{static_cast<float>(i), -static_cast<float>(i)};
        }
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        for (std::size_t i = 0; i < N; ++i) {
            expect(output[i] == input[i]);
        }
    };

    "processOne matches processBlock"_test = [] {
        gr::lora::DCBlocker dc1(kSampleRate, kCutoffHz);
        gr::lora::DCBlocker dc2(kSampleRate, kCutoffHz);

        constexpr std::size_t N = 5'000;
        auto input = generateTone(200.f, kSampleRate, N);

        std::vector<cf32> blockOut(N);
        dc1.processBlock(input, blockOut);

        std::vector<cf32> oneOut(N);
        for (std::size_t i = 0; i < N; ++i) {
            oneOut[i] = dc2.processOne(input[i]);
        }

        for (std::size_t i = 0; i < N; ++i) {
            expect(std::abs(blockOut[i].real() - oneOut[i].real()) < 1e-6f);
            expect(std::abs(blockOut[i].imag() - oneOut[i].imag()) < 1e-6f);
        }
    };

    "cutoff accuracy at 3 dB point"_test = [] {
        constexpr float cutoff = 50.f;
        gr::lora::DCBlocker dc(kSampleRate, cutoff);

        const std::size_t settle = settlingsamples(kSampleRate, cutoff);
        const std::size_t N      = settle + 100'000;

        auto input = generateTone(cutoff, kSampleRate, N);
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        float attenDb = 20.f * std::log10(rms(std::span(output).subspan(settle)) / rms(std::span(input).subspan(settle)));
        std::fprintf(stderr, "attenuation at cutoff (%.0f Hz): %.2f dB\n", cutoff, attenDb);
        expect(attenDb > -4.5f && attenDb < -1.5f)
            << "expected ~-3 dB at cutoff, got" << attenDb << "dB";
    };

    "DC + signal: DC removed, signal preserved"_test = [] {
        gr::lora::DCBlocker dc(kSampleRate, kCutoffHz);

        const std::size_t settle = settlingsamples(kSampleRate, kCutoffHz);
        const std::size_t N      = settle + 50'000;
        constexpr float dcI = 0.3f, dcQ = -0.2f;

        auto signal = generateTone(500.f, kSampleRate, N, 0.5f);
        for (auto& s : signal) {
            s += cf32{dcI, dcQ};
        }

        std::vector<cf32> output(N);
        dc.processBlock(signal, output);

        auto outSteady = std::span(output).subspan(settle);

        cf32 mean{0.f, 0.f};
        for (const auto& s : outSteady) {
            mean += s;
        }
        mean /= static_cast<float>(outSteady.size());

        std::fprintf(stderr, "residual DC: I=%.6f Q=%.6f\n", mean.real(), mean.imag());
        expect(std::abs(mean.real()) < 0.001f) << "residual DC I should be near zero";
        expect(std::abs(mean.imag()) < 0.001f) << "residual DC Q should be near zero";

        float outRms = rms(outSteady);
        expect(outRms > 0.3f) << "signal should be preserved";
    };

    "wideband rate 16 MS/s"_test = [] {
        constexpr float wbRate = 16'000'000.f;
        gr::lora::DCBlocker dc(wbRate, kCutoffHz);
        expect(dc.initialised());

        // at 16 MS/s with 10 Hz cutoff, settling needs ~2.5M samples (~160ms)
        const std::size_t settle = settlingsamples(wbRate, kCutoffHz);
        const std::size_t N      = settle + 500'000;

        auto input = generateTone(1000.f, wbRate, N);
        std::vector<cf32> output(N);
        dc.processBlock(input, output);

        float lossDb = 20.f * std::log10(rms(std::span(output).subspan(settle)) / rms(std::span(input).subspan(settle)));
        std::fprintf(stderr, "16 MS/s, 1 kHz loss: %.3f dB (settle=%zu, N=%zu)\n", lossDb, settle, N);
        expect(lossDb > -0.1f) << "1 kHz at 16 MS/s should pass clean";
    };
};

int main() { /* boost.ut auto-discovers */ }
