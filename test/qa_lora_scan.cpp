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

// ============================================================================
// ScanController L1 energy detection (algorithm-level, no graph needed)
// ============================================================================

const boost::ut::suite<"ScanController L1 energy detection"> scanControllerL1Tests = [] {
    using namespace boost::ut;

    // Helper: construct a ScanController with test-friendly settings and call start().
    // Uses 1 MHz sample rate so channel_bw=62500 gives 12 usable channels (0.8 MHz / 62.5 kHz).
    auto makeScanController = [](float sampleRate = 1e6f, float centerFreq = 868e6f,
                                 float channelBw = 62500.f, float minRatio = 8.0f,
                                 uint32_t fftSize = 1024, uint32_t interval = 1,
                                 uint32_t snapshots = 4) {
        gr::lora::ScanController sc;
        sc.sample_rate  = sampleRate;
        sc.center_freq  = centerFreq;
        sc.channel_bw   = channelBw;
        sc.min_ratio    = minRatio;
        sc.l1_fft_size  = fftSize;
        sc.l1_interval  = interval;
        sc.l1_snapshots = snapshots;
        sc.buffer_ms    = 200.f;
        sc.probe_bws    = "62500";
        sc.start();
        return sc;
    };

    "channel count matches usable bandwidth"_test = [&] {
        auto sc = makeScanController();
        // usableBw = 1e6 * 0.8 = 800 kHz; nChannels = 800k / 62.5k = 12
        expect(eq(sc._nChannels, 12U)) << "12 channels in 800 kHz usable BW";
    };

    "channelCenterFreq returns correct frequencies"_test = [&] {
        auto sc = makeScanController();
        // Band start = 868e6 - 400e3 = 867.6e6
        // Channel 0 center = 867.6e6 + 0.5 * 62.5e3 = 867631250
        const double bandStart = 868e6 - 0.5 * 1e6 * 0.8;
        for (uint32_t ch = 0; ch < sc._nChannels; ++ch) {
            const double expected = bandStart + (static_cast<double>(ch) + 0.5) * 62500.0;
            expect(std::abs(sc.channelCenterFreq(ch) - expected) < 1.0)
                << "channel " << ch << " center frequency";
        }
    };

    "white noise produces no hot channels"_test = [&] {
        auto sc = makeScanController();
        const auto fftSize = sc.l1_fft_size;

        // Generate white noise and push into ring buffer — enough for several snapshots
        auto noise = makeNoise(fftSize * 8, 1.0f, 123U);
        sc._ring.push(std::span<const cf32>(noise));

        // Take multiple energy snapshots
        for (uint32_t i = 0; i < sc.l1_snapshots; ++i) {
            sc.computeEnergySnapshot();
        }
        expect(eq(sc._snapshotCount, sc.l1_snapshots))
            << "all snapshots accumulated";

        // Find hot channels — white noise should have uniform energy, no outliers
        sc.findHotChannels();
        expect(sc._hotChannels.empty())
            << "white noise should produce no hot channels, got " << sc._hotChannels.size();
    };

    "CW tone produces detection at correct channel"_test = [&] {
        constexpr double sampleRate = 1e6;
        constexpr double channelBw  = 62500.0;
        constexpr uint32_t fftSize = 1024;
        constexpr uint32_t snaps   = 8;

        auto sc = makeScanController(static_cast<float>(sampleRate), 868e6f,
                                     static_cast<float>(channelBw), 8.0f, fftSize, 1, snaps);

        // Tone at +200 kHz from center.
        // Band start = center - usableBw/2 = center - 400 kHz
        // Channel index = (offset + 400 kHz) / 62.5 kHz = 600/62.5 = 9.6 → ch 9
        constexpr double toneOffset = 200e3;
        const uint32_t expectedCh = static_cast<uint32_t>(
            (toneOffset + 0.5 * sampleRate * 0.8) / channelBw);

        // Generate tone + low-level noise (SNR ~30 dB)
        const auto nSamples = fftSize * snaps * 2;  // extra for ring buffer fill
        auto tone  = makeTone(nSamples, toneOffset, sampleRate, 1.0f);
        auto noise = makeNoise(nSamples, 0.001f, 77U);
        for (std::size_t i = 0; i < nSamples; ++i) {
            tone[i] += noise[i];
        }

        sc._ring.push(std::span<const cf32>(tone));

        // Accumulate energy + findHotChannels (kMinHotSweeps = 1: report immediately)
        for (uint32_t i = 0; i < snaps; ++i) {
            sc.computeEnergySnapshot();
        }
        sc.findHotChannels();
        expect(!sc._hotChannels.empty()) << "CW tone should produce at least one hot channel";

        if (!sc._hotChannels.empty()) {
            // The detected channel should be near the expected channel
            // Allow ±1 channel tolerance for FFT bin edge effects
            bool found = false;
            for (uint32_t ch : sc._hotChannels) {
                if (ch >= expectedCh - 1 && ch <= expectedCh + 1) {
                    found = true;
                    break;
                }
            }
            expect(found) << "hot channel near expected channel " << expectedCh
                          << ", got channels: " << [&] {
                              std::string s;
                              for (uint32_t c : sc._hotChannels) {
                                  if (!s.empty()) s += ',';
                                  s += std::to_string(c);
                              }
                              return s;
                          }();
        }

        // Verify that channel energy at the tone channel is much higher than median
        float toneEnergy = sc._channelEnergy[expectedCh];
        std::vector<float> sorted(sc._channelEnergy.begin(),
                                  sc._channelEnergy.begin() + static_cast<std::ptrdiff_t>(sc._nChannels));
        std::ranges::sort(sorted);
        float median = sorted[sorted.size() / 2];
        expect(toneEnergy > median * 6.0f) << "tone channel energy should exceed 6× median";
    };

    "two CW tones at different frequencies"_test = [&] {
        constexpr double sampleRate = 1e6;
        constexpr double channelBw  = 62500.0;
        constexpr uint32_t fftSize = 1024;
        constexpr uint32_t snaps   = 8;

        auto sc = makeScanController(static_cast<float>(sampleRate), 868e6f,
                                     static_cast<float>(channelBw), 8.0f, fftSize, 1, snaps);

        // Tone A at +200 kHz → channel ~9
        constexpr double toneA = 200e3;
        // Tone B at -300 kHz → channel ~(−300e3 + 400e3) / 62.5e3 = 1.6 → channel 1
        constexpr double toneB = -300e3;

        const uint32_t expectedChA = static_cast<uint32_t>(
            (toneA + 0.5 * sampleRate * 0.8) / channelBw);
        const uint32_t expectedChB = static_cast<uint32_t>(
            (toneB + 0.5 * sampleRate * 0.8) / channelBw);

        const auto nSamples = fftSize * snaps * 2;
        auto sig = makeNoise(nSamples, 0.001f, 88U);  // noise floor
        {
            auto tA = makeTone(nSamples, toneA, sampleRate, 1.0f);
            auto tB = makeTone(nSamples, toneB, sampleRate, 1.0f);
            for (std::size_t i = 0; i < nSamples; ++i) {
                sig[i] += tA[i] + tB[i];
            }
        }

        sc._ring.push(std::span<const cf32>(sig));

        // Single sweep sufficient (kMinHotSweeps = 1)
        for (uint32_t i = 0; i < snaps; ++i) {
            sc.computeEnergySnapshot();
        }
        sc.findHotChannels();

        expect(sc._hotChannels.size() >= 2U)
            << "two tones should produce at least 2 hot channels, got " << sc._hotChannels.size();

        // Check both expected channels are found (within ±1 tolerance)
        auto channelFound = [&](uint32_t expected) {
            for (uint32_t ch : sc._hotChannels) {
                if (ch >= expected - 1 && ch <= expected + 1) return true;
            }
            return false;
        };
        expect(channelFound(expectedChA))
            << "tone A channel " << expectedChA << " should be hot";
        expect(channelFound(expectedChB))
            << "tone B channel " << expectedChB << " should be hot";
    };

    "cluster dedup merges adjacent hot bins to single channel"_test = [&] {
        auto sc = makeScanController();

        // Direct-inject channel energy to test findHotChannels cluster dedup in isolation.
        // Uniform noise floor at 1.0, with channels 5-7 elevated (6 is peak).
        sc._channelEnergy.assign(sc._nChannels, 1.0f);
        sc._snapshotCount = 4;  // non-zero so findHotChannels proceeds

        // Make channels 5, 6, 7 hot (6 is peak)
        sc._channelEnergy[5] = 100.f;
        sc._channelEnergy[6] = 200.f;
        sc._channelEnergy[7] = 150.f;

        sc.findHotChannels();
        // Cluster dedup should merge channels 5-7 into a single detection at channel 6 (peak)
        expect(eq(sc._hotChannels.size(), 1UZ))
            << "adjacent hot channels should be merged into 1 cluster";
        if (!sc._hotChannels.empty()) {
            expect(eq(sc._hotChannels[0], 6U))
                << "cluster peak should be channel 6";
        }
    };

    "findHotChannels with no snapshots returns empty"_test = [&] {
        auto sc = makeScanController();
        sc._snapshotCount = 0;
        sc.findHotChannels();
        expect(sc._hotChannels.empty()) << "no snapshots → no hot channels";
    };

    "findHotChannels reports on first sweep and tracks persistence"_test = [&] {
        auto sc = makeScanController();
        auto nCh = sc._nChannels;
        expect(gt(nCh, 0U)) << "should have channels";

        // Sweep 1: one channel hot → reported immediately (kMinHotSweeps = 1)
        sc._channelEnergy.assign(nCh, 1.0f);
        if (nCh > 5) sc._channelEnergy[5] = 100.0f;
        sc._snapshotCount = 1;
        sc.findHotChannels();
        expect(eq(sc._hotChannels.size(), 1UZ))
            << "should report on first sweep";
        if (!sc._hotChannels.empty()) {
            expect(eq(sc._hotChannels[0], 5U)) << "should report channel 5";
        }

        // Sweep 2: channel goes cold → counter resets, not reported
        sc._channelEnergy.assign(nCh, 1.0f);
        sc._snapshotCount = 1;
        sc.findHotChannels();
        expect(sc._hotChannels.empty())
            << "should NOT report after channel goes cold";

        // Sweep 3: channel hot again → reported immediately
        sc._channelEnergy.assign(nCh, 1.0f);
        if (nCh > 5) sc._channelEnergy[5] = 100.0f;
        sc._snapshotCount = 1;
        sc.findHotChannels();
        expect(eq(sc._hotChannels.size(), 1UZ))
            << "should report immediately when hot again";
    };

    "resetAccumulation does not clear persistence counters"_test = [&] {
        auto sc = makeScanController();
        auto nCh = sc._nChannels;

        // Sweep 1: make channel 5 hot (counter = 1, reported immediately)
        sc._channelEnergy.assign(nCh, 1.0f);
        if (nCh > 5) sc._channelEnergy[5] = 100.0f;
        sc._snapshotCount = 1;
        sc.findHotChannels();
        expect(eq(sc._hotChannels.size(), 1UZ)) << "reported on first sweep";

        // Call resetAccumulation (as processBulk does between sweeps)
        sc.resetAccumulation();
        expect(eq(sc._channelHotCount[5], 1U))
            << "persistence counter survives resetAccumulation";

        // Sweep 2: same channel still hot → counter = 2, still reported
        sc._channelEnergy.assign(nCh, 1.0f);
        if (nCh > 5) sc._channelEnergy[5] = 100.0f;
        sc._snapshotCount = 1;
        sc.findHotChannels();
        expect(eq(sc._hotChannels.size(), 1UZ))
            << "channel still reported after persistence survives reset";
    };

    "computeEnergySnapshot requires sufficient ring buffer data"_test = [&] {
        auto sc = makeScanController();
        // Ring buffer has data but less than fft_size
        std::vector<cf32> small(sc.l1_fft_size / 2, cf32(1.0f, 0.0f));
        sc._ring.push(std::span<const cf32>(small));

        sc.computeEnergySnapshot();
        expect(eq(sc._snapshotCount, 0U))
            << "snapshot should not be counted when ring buffer has insufficient data";
    };

    "resetAccumulation clears energy and counters"_test = [&] {
        auto sc = makeScanController();
        // Inject some state
        sc._channelEnergy[0] = 999.f;
        sc._snapshotCount = 10;
        sc._callCount = 42;
        sc._hotChannels.push_back(3);

        sc.resetAccumulation();

        expect(eq(sc._channelEnergy[0], 0.f)) << "energy cleared";
        expect(eq(sc._snapshotCount, 0U)) << "snapshot count cleared";
        expect(eq(sc._callCount, 0U)) << "call count cleared";
        expect(sc._hotChannels.empty()) << "hot channels cleared";
    };

    "DC-adjacent hot channels excluded from probe window"_test = [&] {
        // Use 4 MHz sample rate → 51 channels (3.2 MHz usable / 62.5 kHz).
        // DC exclusion: 625 kHz / 62.5 kHz + 1 = 11 channels radius.
        // centerCh=25.  Channels 14-36 (dist ≤ 11) are DC-excluded.
        auto sc = makeScanController(4e6f, 868e6f, 62500.f, 8.0f, 4096);
        auto nCh = sc._nChannels;
        expect(eq(nCh, 51U)) << "precondition: 51 channels";

        const uint32_t centerCh = nCh / 2;  // = 25
        constexpr uint32_t dcExcludeRadius = 11;  // 625000/62500 + 1

        // Inject hot energy at DC-adjacent channel (25 = center) and
        // an off-center channel (5, well outside DC zone).
        sc._channelEnergy.assign(nCh, 1.0f);
        sc._channelEnergy[centerCh] = 200.f;  // DC spur
        sc._channelEnergy[5]        = 150.f;   // real signal, dist=20 from center
        sc._snapshotCount = 1;

        // findHotChannels still reports both (it doesn't filter DC)
        sc.findHotChannels();
        expect(eq(sc._hotChannels.size(), 2UZ))
            << "findHotChannels reports both DC and off-center";

        // Simulate the probe window assembly from ScanState::Accumulate.
        // Set _probeWindowStart far from both hot channels so they'd be
        // merged via the hot-channel merge path (not the rotating window).
        sc._probeWindowStart = 40;  // rotating window covers ch 40-47

        // Compute usable bounds (mirrors production code)
        constexpr float kUsableFrac = 0.70f;
        const uint32_t usableRadius = static_cast<uint32_t>(
            sc.sample_rate * kUsableFrac / (2.f * sc.channel_bw));
        const uint32_t usableEnd = std::min(centerCh + usableRadius, nCh);

        constexpr uint32_t kProbeWindowSize = 8;
        const uint32_t windowEnd = std::min(sc._probeWindowStart + kProbeWindowSize, usableEnd);

        std::vector<uint32_t> probeChannels;
        for (uint32_t ch = sc._probeWindowStart; ch < windowEnd; ++ch) {
            const auto dist = (ch > centerCh) ? ch - centerCh : centerCh - ch;
            if (dist <= dcExcludeRadius) continue;
            probeChannels.push_back(ch);
        }
        // Merge hot channels with DC exclusion (same logic as production code)
        for (uint32_t ch : sc._hotChannels) {
            if (ch < sc._probeWindowStart || ch >= windowEnd) {
                const auto dist = (ch > centerCh) ? ch - centerCh : centerCh - ch;
                if (dist <= dcExcludeRadius) continue;
                probeChannels.push_back(ch);
            }
        }

        // Channel 25 (DC center, dist=0) should be excluded from merge
        bool hasDcCenter = std::ranges::find(probeChannels, centerCh) != probeChannels.end();
        expect(!hasDcCenter) << "DC center channel should be excluded from probe list";

        // Channel 5 (off-center, dist=20 > dcExcludeRadius=11) should be merged
        bool hasOffCenter = std::ranges::find(probeChannels, 5U) != probeChannels.end();
        expect(hasOffCenter) << "off-center hot channel (5) should be in probe list";

        // Rotating window channels (40-47) should be present (dist > 11 from center)
        for (uint32_t ch = sc._probeWindowStart; ch < windowEnd; ++ch) {
            bool hasIt = std::ranges::find(probeChannels, ch) != probeChannels.end();
            expect(hasIt) << "rotating window channel " << ch << " should be in probe list";
        }
    };

    "DC-adjacent hot channels at edge of exclusion zone"_test = [&] {
        // Use 4 MHz sample rate → 51 channels.
        // DC exclusion: 625 kHz / 62.5 kHz + 1 = 11 channels radius.
        // centerCh=25.  ch 36 (dist=11) = edge of DC zone, ch 38 (dist=13) = outside.
        auto sc = makeScanController(4e6f, 868e6f, 62500.f, 8.0f, 4096);
        auto nCh = sc._nChannels;
        const uint32_t centerCh = nCh / 2;  // = 25
        constexpr uint32_t dcExcludeRadius = 11;  // 625000/62500 + 1

        // ch 36 (dist=11, at DC edge) and ch 38 (dist=13, outside DC).
        // Gap at ch 37 prevents cluster dedup from merging them.
        sc._channelEnergy.assign(nCh, 1.0f);
        sc._channelEnergy[36] = 200.f;  // dist=11, at DC edge
        sc._channelEnergy[38] = 200.f;  // dist=13, outside DC
        sc._snapshotCount = 1;
        sc.findHotChannels();

        // Both reported by findHotChannels (non-adjacent, survive cluster dedup)
        expect(eq(sc._hotChannels.size(), 2UZ)) << "both channels reported by findHotChannels";

        // Simulate probe window NOT covering either channel (window at ch 2-9)
        std::vector<uint32_t> probeChannels;
        for (uint32_t ch = 2; ch < 10; ++ch) {
            probeChannels.push_back(ch);
        }
        for (uint32_t ch : sc._hotChannels) {
            if (ch >= 10) {  // outside window [2,10)
                const auto dist = (ch > centerCh) ? ch - centerCh : centerCh - ch;
                if (dist <= dcExcludeRadius) continue;
                probeChannels.push_back(ch);
            }
        }

        // ch 36 (dist=11 == dcExcludeRadius) should be excluded from hot merge
        bool hasCh36 = std::ranges::find(probeChannels, 36U) != probeChannels.end();
        expect(!hasCh36) << "channel 36 (dist=11, at DC edge) excluded from hot merge";

        // ch 38 (dist=13 > dcExcludeRadius) should be included in hot merge
        bool hasCh38 = std::ranges::find(probeChannels, 38U) != probeChannels.end();
        expect(hasCh38) << "channel 38 (dist=13, outside DC) included in hot merge";
    };

    "state machine transitions ACCUMULATE → findHotChannels on snapshot threshold"_test = [&] {
        constexpr uint32_t fftSize = 1024;
        constexpr uint32_t snaps   = 2;  // very low threshold for quick test

        auto sc = makeScanController(1e6f, 868e6f, 62500.f, 8.0f, fftSize, 1, snaps);

        // Feed noise — should complete a sweep and reset (no hot channels)
        auto noise = makeNoise(fftSize * snaps * 4, 1.0f, 99U);
        sc._ring.push(std::span<const cf32>(noise));

        // Simulate the processBulk L1 path manually:
        // interval=1, so every call triggers a snapshot
        for (uint32_t i = 0; i < snaps; ++i) {
            sc.computeEnergySnapshot();
        }

        expect(eq(sc._snapshotCount, snaps)) << "snapshot count reached threshold";
        expect(sc._state == gr::lora::ScanController::ScanState::Accumulate)
            << "still in Accumulate before findHotChannels";

        sc.findHotChannels();
        expect(sc._hotChannels.empty()) << "noise → no hot channels";
        // In the real processBulk, this would trigger resetAccumulation and sweepCount++
    };
};

int main() { /* boost.ut auto-runs */ }
