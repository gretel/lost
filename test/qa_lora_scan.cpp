// SPDX-License-Identifier: ISC
// Tests for streaming scan pipeline blocks: SpectrumTap, Channelize, ScanController.

#include "test_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <random>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/ScanController.hpp>
#include <gnuradio-4.0/lora/algorithm/Channelize.hpp>
#include <gnuradio-4.0/lora/algorithm/RingBuffer.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

using namespace std::string_literals;
using cf32 = std::complex<float>;

namespace {

// Generate a pure tone at a given frequency offset from center.
std::vector<cf32> makeTone(std::size_t nSamples, double freq_hz, double sample_rate,
                           float amplitude = 1.0f) {
    std::vector<cf32> out(nSamples);
    const double phaseInc = 2.0 * std::numbers::pi * freq_hz / sample_rate;
    double phase = 0.0;
    for (auto& s : out) {
        s = cf32(amplitude * static_cast<float>(std::cos(phase)),
                 amplitude * static_cast<float>(std::sin(phase)));
        phase += phaseInc;
    }
    return out;
}

// Generate AWGN noise.
std::vector<cf32> makeNoise(std::size_t nSamples, float power = 1.0f, uint32_t seed = 42U) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, std::sqrt(power / 2.f));
    std::vector<cf32> out(nSamples);
    for (auto& s : out) {
        s = cf32(dist(rng), dist(rng));
    }
    return out;
}

} // namespace

// ============================================================================
// Channelize algorithm tests
// ============================================================================

const boost::ut::suite<"Channelize"> channelizeTests = [] {
    using namespace boost::ut;

    "freqShift moves tone to baseband"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double toneFreq  = 500e3;  // 500 kHz tone
        constexpr std::size_t N    = 4096;

        auto buf = makeTone(N, toneFreq, sampleRate);

        // Shift by -500 kHz to bring tone to DC
        gr::lora::freqShift(std::span(buf), -toneFreq, sampleRate);

        // After shift, the signal should be at DC — real part ~constant, imag ~0
        float sumReal = 0.f, sumImag = 0.f;
        for (const auto& s : buf) {
            sumReal += s.real();
            sumImag += s.imag();
        }
        // DC component should be large (N samples of amplitude 1)
        expect(std::abs(sumReal) > static_cast<float>(N) * 0.9f) << "DC real component after shift";
        expect(std::abs(sumImag) < static_cast<float>(N) * 0.1f) << "imaginary near zero after shift";
    };

    "decimate preserves DC signal"_test = [] {
        constexpr uint32_t N      = 1024;
        constexpr uint32_t factor = 8;

        // DC signal (all ones)
        std::vector<cf32> dc(N, cf32(1.0f, 0.0f));
        auto out = gr::lora::decimate(dc.data(), N, factor);

        expect(eq(out.size(), static_cast<std::size_t>(N / factor)));
        for (const auto& s : out) {
            expect(std::abs(s.real() - 1.0f) < 0.01f) << "DC preserved through decimation";
        }
    };

    "decimate factor=1 returns copy"_test = [] {
        constexpr uint32_t N = 128;
        auto noise = makeNoise(N);
        auto out   = gr::lora::decimate(noise.data(), N, 1);
        expect(eq(out.size(), static_cast<std::size_t>(N)));
        for (uint32_t i = 0; i < N; ++i) {
            expect(out[i] == noise[i]);
        }
    };

    "channelize extracts off-center tone"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double toneFreq   = 867.0e6;  // 500 kHz above center
        constexpr double targetBw   = 125e3;
        constexpr uint32_t N        = 128 * 128;  // 16384 wideband samples

        // Generate tone at 500 kHz offset
        auto wb = makeTone(N, toneFreq - centerFreq, sampleRate);

        auto nb = gr::lora::channelize(wb.data(), N, toneFreq, centerFreq, sampleRate, targetBw);

        // Decimation factor = 16e6 / 125e3 = 128
        expect(eq(nb.size(), static_cast<std::size_t>(N / 128)));

        // After channelization, the tone should be at DC in the narrowband output
        float sumReal = 0.f;
        for (const auto& s : nb) {
            sumReal += s.real();
        }
        // DC component should be significant
        expect(std::abs(sumReal) > static_cast<float>(nb.size()) * 0.5f)
            << "tone at DC after channelization";
    };
};

// ============================================================================
// RingBuffer tests
// ============================================================================

const boost::ut::suite<"RingBuffer"> ringBufferTests = [] {
    using namespace boost::ut;

    "push and recent basic"_test = [] {
        gr::lora::RingBuffer ring;
        ring.resize(100);

        std::vector<cf32> data(50, cf32(1.0f, 0.0f));
        ring.push(std::span<const cf32>(data));

        auto out = ring.recent(50);
        expect(eq(out.size(), 50UZ));
        expect(out[0].real() == 1.0f);
    };

    "recent wraps around correctly"_test = [] {
        gr::lora::RingBuffer ring;
        ring.resize(10);

        // Write 15 samples (wraps around a 10-element buffer)
        for (int i = 0; i < 15; ++i) {
            cf32 val(static_cast<float>(i), 0.f);
            ring.push(std::span<const cf32>(&val, 1));
        }

        // Most recent 5 should be [10, 11, 12, 13, 14]
        auto out = ring.recent(5);
        expect(eq(out.size(), 5UZ));
        for (int i = 0; i < 5; ++i) {
            expect(out[static_cast<std::size_t>(i)].real() == static_cast<float>(10 + i))
                << "sample " << i;
        }
    };

    "recent returns empty if not enough data"_test = [] {
        gr::lora::RingBuffer ring;
        ring.resize(100);

        cf32 val(1.0f, 0.0f);
        ring.push(std::span<const cf32>(&val, 1));

        auto out = ring.recent(50);  // only 1 sample written
        expect(out.empty());
    };

    "reset clears buffer"_test = [] {
        gr::lora::RingBuffer ring;
        ring.resize(10);

        std::vector<cf32> data(10, cf32(1.0f, 0.0f));
        ring.push(std::span<const cf32>(data));
        ring.reset();

        auto out = ring.recent(1);
        expect(out.empty()) << "recent returns empty after reset";
    };
};

// ============================================================================
// SpectrumTap tests (standalone, no graph)
// ============================================================================

const boost::ut::suite<"SpectrumTap"> spectrumTapTests = [] {
    using namespace boost::ut;

    "passthrough preserves samples"_test = [] {
        gr::Graph graph;

        constexpr uint32_t fftSize = 64;
        constexpr uint32_t accumulate = 2;
        constexpr float    sampleRate = 16e6f;
        constexpr std::size_t totalSamples = fftSize * accumulate * 2;  // 2 full reports

        auto& source = graph.emplaceBlock<gr::testing::TagSource<cf32>>(
            {{"n_samples_max", static_cast<gr::Size_t>(totalSamples)},
             {"mark_tag", false}});

        auto& tap = graph.emplaceBlock<gr::lora::SpectrumTap>(
            {{"sample_rate", sampleRate},
             {"fft_size", fftSize},
             {"fft_accumulate", accumulate}});

        auto& sink = graph.emplaceBlock<gr::testing::TagSink<cf32,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>(
            {{"log_samples", false}, {"log_tags", false}});

        expect(graph.connect<"out", "in">(source, tap).has_value());
        expect(graph.connect<"out", "in">(tap, sink).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        // Verify passthrough: all samples should appear at the sink
        expect(eq(sink._nSamplesProduced, static_cast<gr::Size_t>(totalSamples)))
            << "all samples passed through";
    };
};

int main() { /* boost.ut auto-runs */ }
