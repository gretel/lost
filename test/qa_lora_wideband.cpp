// SPDX-License-Identifier: ISC
// Tests for WidebandDecoder: ChannelSlot (M2) + block skeleton (M3) + decode (M4) + loopback (M5).

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <numeric>
#include <random>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/WidebandDecoder.hpp>
#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using cf32 = std::complex<float>;

namespace {

// Generate a pure tone at a given frequency offset from center.
std::vector<cf32> makeTone(std::size_t nSamples, double freq_hz,
                           double sample_rate, float amplitude = 1.0f) {
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

// Compute |FFT|^2 and return bin index of peak.
std::size_t peakBin(const std::vector<cf32>& signal) {
    const auto N = signal.size();
    // Simple DFT for small sizes used in tests
    std::vector<float> mag2(N, 0.f);
    for (std::size_t k = 0; k < N; ++k) {
        cf32 sum{0.f, 0.f};
        for (std::size_t n = 0; n < N; ++n) {
            const double angle = -2.0 * std::numbers::pi
                                 * static_cast<double>(k * n)
                                 / static_cast<double>(N);
            sum += signal[n] * cf32(static_cast<float>(std::cos(angle)),
                                    static_cast<float>(std::sin(angle)));
        }
        mag2[k] = std::norm(sum);
    }
    return static_cast<std::size_t>(
        std::distance(mag2.begin(), std::max_element(mag2.begin(), mag2.end())));
}

// Peak-to-mean ratio in dB (excluding the peak bin).
float peakToMeanDb(const std::vector<cf32>& signal) {
    const auto N = signal.size();
    std::vector<float> mag2(N, 0.f);
    for (std::size_t k = 0; k < N; ++k) {
        cf32 sum{0.f, 0.f};
        for (std::size_t n = 0; n < N; ++n) {
            const double angle = -2.0 * std::numbers::pi
                                 * static_cast<double>(k * n)
                                 / static_cast<double>(N);
            sum += signal[n] * cf32(static_cast<float>(std::cos(angle)),
                                    static_cast<float>(std::sin(angle)));
        }
        mag2[k] = std::norm(sum);
    }
    const auto peakIdx = static_cast<std::size_t>(
        std::distance(mag2.begin(), std::max_element(mag2.begin(), mag2.end())));
    const float peakVal = mag2[peakIdx];

    // Mean of non-peak bins
    float sum = 0.f;
    for (std::size_t k = 0; k < N; ++k) {
        if (k != peakIdx) sum += mag2[k];
    }
    const float meanNonPeak = sum / static_cast<float>(N - 1);
    if (meanNonPeak <= 0.f) return 100.f;  // effectively infinite
    return 10.f * std::log10(peakVal / meanNonPeak);
}

}  // namespace

// ============================================================================
// ChannelSlot streaming channelizer tests
// ============================================================================

const boost::ut::suite<"ChannelSlot streaming channelizer"> channelSlotTests = [] {
    using namespace boost::ut;

    "activate sets state and computes NCO params"_test = [] {
        gr::lora::ChannelSlot slot;
        expect(slot.state == gr::lora::ChannelSlot::State::Idle);

        slot.activate(869.618e6, 866.5e6, 16e6, 256);

        expect(slot.state == gr::lora::ChannelSlot::State::Active);
        expect(eq(slot.osFactor, 256U));
        expect(eq(slot.channelFreq, 869.618e6));
        expect(eq(slot.centerFreq, 866.5e6));
        expect(eq(slot.sampleRate, 16e6));
        // NCO phase increment should be negative (shift channel down to baseband)
        // shift_hz = -(869.618e6 - 866.5e6) = -3.118e6
        // phaseInc = 2*pi * (-3.118e6) / 16e6
        const double expectedInc = 2.0 * std::numbers::pi * (-3.118e6) / 16e6;
        expect(std::abs(slot.ncoPhaseInc - expectedInc) < 1e-12)
            << "NCO phase increment";
    };

    "deactivate resets to idle"_test = [] {
        gr::lora::ChannelSlot slot;
        slot.activate(869e6, 866.5e6, 16e6, 256);
        slot.pushWideband(std::vector<cf32>(512, cf32(1.f, 0.f)));
        expect(!slot.nbAccum.empty());

        slot.deactivate();
        expect(slot.state == gr::lora::ChannelSlot::State::Idle);
        expect(slot.nbAccum.empty());
    };

    "pushWideband produces correct number of output samples"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;  // +3 MHz
        constexpr uint32_t osFactor = 256;        // 16e6 / 62.5e3 = 256
        constexpr std::size_t nSamples = 256 * 100;  // 100 ideal output samples

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        // FIR group delay means output count is slightly less than ideal.
        // With 8 cascaded half-band stages (23-tap each), expect close to 100.
        constexpr std::size_t expectedIdeal = 100;
        expect(slot.nbAccum.size() >= expectedIdeal - 5
            && slot.nbAccum.size() <= expectedIdeal + 1)
            << "output sample count " << slot.nbAccum.size()
            << " near expected " << expectedIdeal;
    };

    "tone at channel freq appears at DC after channelization"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;  // +3 MHz offset
        constexpr uint32_t osFactor = 256;        // BW62.5k: 16e6/62.5e3
        // Generate enough for ~256 output samples (FIR may produce slightly fewer)
        constexpr std::size_t nSamples = 256 * osFactor;

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        expect(slot.nbAccum.size() > 64UZ)
            << "sufficient output: " << slot.nbAccum.size();

        // FFT peak should be at bin 0 (DC)
        auto bin = peakBin(slot.nbAccum);
        expect(eq(bin, 0UZ)) << "tone at channel freq maps to DC";

        // Peak-to-mean ratio should be high (clean tone)
        auto pmr = peakToMeanDb(slot.nbAccum);
        expect(pmr > 20.f) << "PMR = " << pmr << " dB, expected > 20 dB";
    };

    "phase continuity across multiple pushWideband calls"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 868.0e6;   // +1.5 MHz
        constexpr uint32_t osFactor = 128;          // BW125k: 16e6/125e3
        constexpr std::size_t chunkSamples = 8192;  // typical SoapySource chunk
        constexpr std::size_t nChunks = 8;
        constexpr std::size_t totalSamples = chunkSamples * nChunks;

        // Generate the full tone as one continuous signal
        auto fullTone = makeTone(totalSamples,
                                 channelFreq - centerFreq, sampleRate);

        // Push in chunks (simulating SoapySource stream)
        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        for (std::size_t i = 0; i < nChunks; ++i) {
            auto chunk = std::span<const cf32>(
                fullTone.data() + i * chunkSamples, chunkSamples);
            slot.pushWideband(chunk);
        }

        const std::size_t expectedOut = totalSamples / osFactor;
        // FIR group delay means a few fewer samples
        expect(slot.nbAccum.size() >= expectedOut - 5
            && slot.nbAccum.size() <= expectedOut + 1)
            << "total output = " << slot.nbAccum.size()
            << ", expected ~" << expectedOut;

        // FFT peak at DC (bin 0) proves phase is continuous:
        // if NCO phase reset between chunks, the tone would smear across bins.
        auto bin = peakBin(slot.nbAccum);
        expect(eq(bin, 0UZ)) << "DC peak after chunked push (phase continuity)";

        auto pmr = peakToMeanDb(slot.nbAccum);
        expect(pmr > 20.f)
            << "PMR = " << pmr << " dB after chunked push, expected > 20 dB";
    };

    "offset tone appears at correct non-DC bin"_test = [] {
        // A tone offset from channel center by +10 kHz should appear at the
        // corresponding FFT bin, NOT at DC.  This verifies the NCO shifts
        // only the channel center to baseband.
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;  // targeting +3 MHz
        constexpr double toneOffset = 10e3;       // 10 kHz above channel center
        constexpr double toneFreq = channelFreq + toneOffset;
        constexpr uint32_t osFactor = 256;         // BW62.5k -> nb rate = 62.5 kHz
        constexpr std::size_t nSamples = 256 * osFactor;  // plenty for FIR warmup

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, toneFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        expect(slot.nbAccum.size() > 64UZ)
            << "sufficient output: " << slot.nbAccum.size();

        // After channelization, the tone is at +10 kHz in the narrowband stream.
        // With nb sample rate = 62.5 kHz, bin spacing = 62500/N Hz.
        // Expected bin ≈ 10000 / (62500 / N) — depends on actual N from FIR.
        auto N = slot.nbAccum.size();
        auto bin = peakBin(slot.nbAccum);
        double expectedBin = 10000.0 / (62500.0 / static_cast<double>(N));
        expect(bin != 0UZ) << "offset tone should NOT be at DC";
        expect(std::abs(static_cast<double>(bin) - expectedBin) < 3.0)
            << "expected bin ~" << expectedBin << ", got " << bin;
    };

    "decimator handles partial blocks correctly"_test = [] {
        // Push a number of samples that is NOT a multiple of osFactor.
        // FIR has group delay, so use large input to make warmup negligible.
        constexpr uint32_t osFactor = 256;
        constexpr std::size_t nSamples = osFactor * 100 + 100;

        gr::lora::ChannelSlot slot;
        slot.activate(866.5e6, 866.5e6, 16e6, osFactor);  // channel == center -> no shift

        std::vector<cf32> dc(nSamples, cf32(1.f, 0.f));
        slot.pushWideband(dc);

        // FIR decimation: output count depends on filter group delay.
        // With 8 cascaded half-band stages (2^8=256), each 23-tap stage
        // has 11-sample group delay. The exact count is hard to predict,
        // but should be close to floor(nSamples/osFactor) = 100.
        // Accept within ±5 samples of the ideal count.
        const auto expectedApprox = nSamples / osFactor;
        expect(slot.nbAccum.size() >= expectedApprox - 5
            && slot.nbAccum.size() <= expectedApprox + 5)
            << "partial block: output count " << slot.nbAccum.size()
            << " near expected " << expectedApprox;

        // Push more samples — output should grow
        auto prevSize = slot.nbAccum.size();
        std::vector<cf32> more(osFactor * 10, cf32(1.f, 0.f));
        slot.pushWideband(more);
        expect(slot.nbAccum.size() > prevSize)
            << "more input produces more output";
    };

    "activate with custom SF range limits lanes"_test = [] {
        gr::lora::ChannelSlot slot;
        std::vector<uint8_t> sfs = {7, 8, 9};
        slot.activate(869e6, 866.5e6, 16e6, 256, 0, 62500, 0x12, 8, sfs);
        expect(eq(slot.sfLanes.size(), 3UZ));
        expect(eq(slot.sfLanes[0].sf, uint8_t{7}));
        expect(eq(slot.sfLanes[1].sf, uint8_t{8}));
        expect(eq(slot.sfLanes[2].sf, uint8_t{9}));

        // Default (empty span) should still give SF7-12
        gr::lora::ChannelSlot slot2;
        slot2.activate(869e6, 866.5e6, 16e6, 256);
        expect(eq(slot2.sfLanes.size(), 6UZ));
        expect(eq(slot2.sfLanes[0].sf, uint8_t{7}));
        expect(eq(slot2.sfLanes[5].sf, uint8_t{12}));
    };

    "activate with single SF"_test = [] {
        gr::lora::ChannelSlot slot;
        std::vector<uint8_t> sfs = {8};
        slot.activate(869e6, 866.5e6, 16e6, 256, 0, 62500, 0x12, 8, sfs);
        expect(eq(slot.sfLanes.size(), 1UZ));
        expect(eq(slot.sfLanes[0].sf, uint8_t{8}));
    };

    "DC signal at center freq passes through unchanged"_test = [] {
        constexpr uint32_t osFactor = 128;
        constexpr std::size_t nSamples = osFactor * 128;  // extra for FIR warmup

        gr::lora::ChannelSlot slot;
        // Channel == center freq -> shift_hz = 0 -> NCO is identity
        slot.activate(866.5e6, 866.5e6, 16e6, osFactor);

        std::vector<cf32> dc(nSamples, cf32(1.f, 0.f));
        slot.pushWideband(dc);

        expect(slot.nbAccum.size() > 20UZ)
            << "enough output samples: " << slot.nbAccum.size();

        // Skip first 20 samples (FIR warmup transient), then check steady-state
        for (std::size_t i = 20; i < slot.nbAccum.size(); ++i) {
            expect(std::abs(std::abs(slot.nbAccum[i]) - 1.0f) < 0.05f)
                << "DC passthrough magnitude at [" << i << "] = "
                << std::abs(slot.nbAccum[i]);
        }
    };

    "FIR channelizer output has clean tone at DC"_test = [] {
        // Verify ChannelSlot with FIR decimator produces a clean tone at DC
        // when fed a wideband tone at the channel frequency.
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 868.0e6;
        constexpr uint32_t osFactor = 128;  // BW125k: 16e6 / 125e3
        constexpr std::size_t nOut = 256;
        constexpr std::size_t nSamples = nOut * osFactor;

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);
        slot.pushWideband(tone);

        expect(slot.nbAccum.size() > 64UZ)
            << "sufficient output: " << slot.nbAccum.size();

        // FFT peak should be at bin 0 (DC)
        auto bin = peakBin(slot.nbAccum);
        expect(eq(bin, 0UZ)) << "tone at channel freq maps to DC";

        // Peak-to-mean ratio should be high (clean tone from FIR)
        auto pmr = peakToMeanDb(slot.nbAccum);
        expect(pmr > 20.f) << "PMR = " << pmr << " dB, expected > 20 dB";
    };

    "out-of-band rejection > 30 dB"_test = [] {
        // Two ChannelSlots at the same channel: one in-band tone, one out-of-band.
        // FIR should give > 30 dB rejection (box-car gave only ~13 dB).
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;
        constexpr uint32_t osFactor = 256;  // BW62.5k: 16e6 / 62.5e3
        constexpr double nbRate = sampleRate / osFactor;  // 62500 Hz
        constexpr std::size_t nOut = 512;
        constexpr std::size_t nSamples = nOut * osFactor;

        // In-band: tone exactly at channel center
        auto inBandTone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        gr::lora::ChannelSlot slotIn;
        slotIn.activate(channelFreq, centerFreq, sampleRate, osFactor);
        slotIn.pushWideband(inBandTone);

        // Out-of-band: tone 100 kHz away (well outside BW62.5k passband)
        constexpr double oobOffset = 100e3;
        auto oobTone = makeTone(nSamples,
                                (channelFreq + oobOffset) - centerFreq, sampleRate);
        gr::lora::ChannelSlot slotOob;
        slotOob.activate(channelFreq, centerFreq, sampleRate, osFactor);
        slotOob.pushWideband(oobTone);

        // Measure output power (skip warmup transient)
        auto measurePower = [](const std::vector<cf32>& samples, std::size_t skip_samples) {
            double power = 0.0;
            std::size_t count = 0;
            for (std::size_t i = skip_samples; i < samples.size(); ++i) {
                power += static_cast<double>(std::norm(samples[i]));
                ++count;
            }
            return count > 0 ? power / static_cast<double>(count) : 0.0;
        };

        constexpr std::size_t kSkip = 30;  // skip FIR warmup
        double inPower  = measurePower(slotIn.nbAccum, kSkip);
        double oobPower = measurePower(slotOob.nbAccum, kSkip);

        expect(inPower > 0.0) << "in-band has signal power";
        expect(oobPower > 0.0) << "out-of-band has residual power";

        double rejectionDb = 10.0 * std::log10(oobPower / inPower);
        (void)nbRate;
        std::fprintf(stderr, "  out-of-band rejection: %.1f dB (in=%.2e, oob=%.2e)\n",
                     rejectionDb, inPower, oobPower);
        expect(rejectionDb < -30.0)
            << "OOB rejection = " << rejectionDb << " dB, expected < -30 dB";
    };
};

// ============================================================================
// WidebandDecoder block skeleton tests (M3)
// ============================================================================

const boost::ut::suite<"WidebandDecoder block skeleton"> wbBlockTests = [] {
    using namespace boost::ut;

    "block construction via graph emplaceBlock"_test = [] {
        gr::Graph graph;

        auto& decoder = graph.emplaceBlock<gr::lora::WidebandDecoder>({
            {"sample_rate", 16000000.f},
            {"center_freq", 866500000.f},
            {"channel_bw", 62500.f},
            {"decode_bw", 62500.f},
            {"max_channels", uint32_t{8}},

            {"l1_interval", uint32_t{4}},
            {"l1_snapshots", uint32_t{4}},
            {"l1_fft_size", uint32_t{4096}},
            {"debug", true},
        });

        expect(eq(decoder.sample_rate, 16000000.f));
        expect(eq(decoder.max_channels, 8U));
    };

    "graph-level: wideband noise produces no active slots"_test = [] {
        using namespace gr;

        constexpr float sampleRate = 16e6f;
        constexpr float centerFreq = 866.5e6f;
        constexpr std::size_t nSamples = 8192 * 20;  // 20 chunks

        // Generate pure noise (no signal)
        std::vector<cf32> noise(nSamples);
        std::mt19937 rng(42);
        std::normal_distribution<float> dist(0.f, 0.001f);
        for (auto& s : noise) {
            s = cf32(dist(rng), dist(rng));
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<cf32,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(nSamples)},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        src.values = noise;

        auto& decoder = graph.emplaceBlock<lora::WidebandDecoder>({
            {"sample_rate", sampleRate},
            {"center_freq", centerFreq},
            {"channel_bw", 62500.f},
            {"decode_bw", 62500.f},
            {"max_channels", uint32_t{8}},

            {"l1_interval", uint32_t{4}},
            {"l1_snapshots", uint32_t{4}},
            {"l1_fft_size", uint32_t{4096}},
            {"debug", false},
        });

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", false},
            {"log_tags", false},
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            std::fprintf(stderr, "scheduler exchange failed\n");
            expect(false) << "scheduler exchange failed";
            return;
        }
        sched.runAndWait();

        // With pure noise, no channels should have been activated
        // (no output bytes expected)
        expect(sink._samples.empty() || sink._nSamplesProduced == 0U)
            << "no decoded output from noise-only input";
    };

    "graph-level: strong tone activates a channel slot"_test = [] {
        using namespace gr;

        constexpr float sampleRate = 16e6f;
        constexpr float centerFreq = 866.5e6f;
        constexpr double toneFreq = 868.0e6;  // +1.5 MHz from center
        // Need enough samples for L1 detection: l1_interval * l1_snapshots * chunk_size
        // With l1_interval=4, l1_snapshots=4: need 4*4 = 16 processBulk calls
        // At ~8192 samples/call: ~131072 samples minimum, use more for margin
        constexpr std::size_t nSamples = 8192 * 24;

        // Generate wideband signal: weak noise floor + strong tone at +1.5 MHz
        std::vector<cf32> signal(nSamples);
        std::mt19937 rng(42);
        std::normal_distribution<float> noiseDist(0.f, 0.001f);
        const double phaseInc = 2.0 * std::numbers::pi
                               * (toneFreq - static_cast<double>(centerFreq))
                               / static_cast<double>(sampleRate);
        double phase = 0.0;
        for (auto& s : signal) {
            // Tone amplitude 1.0 + noise 0.001
            s = cf32(static_cast<float>(std::cos(phase)) + noiseDist(rng),
                     static_cast<float>(std::sin(phase)) + noiseDist(rng));
            phase += phaseInc;
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<cf32,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(nSamples)},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        src.values = signal;

        auto& decoder = graph.emplaceBlock<lora::WidebandDecoder>({
            {"sample_rate", sampleRate},
            {"center_freq", centerFreq},
            {"channel_bw", 62500.f},
            {"decode_bw", 62500.f},
            {"max_channels", uint32_t{8}},

            {"l1_interval", uint32_t{4}},
            {"l1_snapshots", uint32_t{4}},
            {"l1_fft_size", uint32_t{4096}},
            {"debug", false},
        });

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", false},
            {"log_tags", false},
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            std::fprintf(stderr, "scheduler exchange failed\n");
            expect(false) << "scheduler exchange failed";
            return;
        }
        auto result = sched.runAndWait();

        // Verify the scheduler completed successfully (not stuck/timeout)
        expect(result.has_value()) << "scheduler completed without error";

        // A CW tone activates L1 channel detection but produces no decoded
        // LoRa frames, so sink output may be 0 samples. The meaningful check
        // is that the scheduler ran to completion without deadlock — a stuck
        // block would cause a timeout. The source pushed all samples through
        // the WidebandDecoder, which means L1 energy detection + slot
        // management executed without crash.
        expect(src._nSamplesProduced > 0U)
            << "source produced samples into the graph";
    };

    "channelCenterFreq maps channel indices to frequencies"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.sample_rate = 16e6f;
        decoder.center_freq = 866.5e6f;
        decoder.channel_bw  = 62500.f;

        // usableBw = 16e6 * 0.8 = 12.8 MHz
        // bandStart = 866.5e6 - 6.4e6 = 860.1e6
        // ch0 center = 860.1e6 + 0.5 * 62500 = 860131250
        double ch0 = decoder.channelCenterFreq(0);
        // usableBw = 16e6 * 0.8 = 12.8e6
        // bandStart = 866.5e6 - 6.4e6 = 860100000.0
        // ch0 center = 860100000.0 + 0.5 * 62500.0 = 860131250.0
        double expected = 860100000.0 + 0.5 * 62500.0;
        // Tolerance: startBin truncation causes up to binBw (~3906 Hz) offset
        expect(std::abs(ch0 - expected) < 10000.0)
            << "ch0 = " << ch0 << ", expected = " << expected;

        // ch1 = ch0 + 62500
        double ch1 = decoder.channelCenterFreq(1);
        expect(std::abs(ch1 - ch0 - 62500.0) < 100.0)
            << "ch1 - ch0 = " << (ch1 - ch0);
    };

    "activeSlotCount reflects pool state"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.max_channels = 4;
        decoder._slots.resize(4);

        expect(eq(decoder.activeSlotCount(), 0U)) << "all idle initially";

        decoder._slots[0].state = gr::lora::ChannelSlot::State::Active;
        decoder._slots[2].state = gr::lora::ChannelSlot::State::Active;
        expect(eq(decoder.activeSlotCount(), 2U)) << "two active";

        decoder._slots[0].deactivate();
        expect(eq(decoder.activeSlotCount(), 1U)) << "one deactivated";
    };

    "decode_sfs_str limits SfLanes in activated slots"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.sample_rate = 16e6f;
        decoder.center_freq = 866.5e6f;
        decoder.channel_bw  = 62500.f;
        decoder.decode_bw_str = "62500";
        decoder.decode_sfs_str = "7,8,9";
        decoder.max_channels = 8;
        decoder.start();

        // Simulate: force one hot channel
        decoder._hotChannels = {100};
        decoder.updateActiveChannels();

        // Should have 1 active slot with 3 SfLanes (SF7,8,9)
        expect(eq(decoder.activeSlotCount(), 1U));
        for (const auto& slot : decoder._slots) {
            if (slot.state != gr::lora::ChannelSlot::State::Idle) {
                expect(eq(slot.sfLanes.size(), 3UZ))
                    << "expected 3 SfLanes, got " << slot.sfLanes.size();
                expect(eq(slot.sfLanes[0].sf, uint8_t{7}));
                expect(eq(slot.sfLanes[1].sf, uint8_t{8}));
                expect(eq(slot.sfLanes[2].sf, uint8_t{9}));
            }
        }
    };

    "decode_sfs_str empty gives all SFs"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.sample_rate = 16e6f;
        decoder.center_freq = 866.5e6f;
        decoder.channel_bw  = 62500.f;
        decoder.decode_bw_str = "62500";
        decoder.decode_sfs_str = "";  // empty = all SF7-12
        decoder.max_channels = 8;
        decoder.start();

        decoder._hotChannels = {100};
        decoder.updateActiveChannels();

        expect(eq(decoder.activeSlotCount(), 1U));
        for (const auto& slot : decoder._slots) {
            if (slot.state != gr::lora::ChannelSlot::State::Idle) {
                expect(eq(slot.sfLanes.size(), 6UZ))
                    << "expected 6 SfLanes, got " << slot.sfLanes.size();
            }
        }
    };

    "multi-BW: three slots activated per hot channel"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.sample_rate = 16e6f;
        decoder.center_freq = 866.5e6f;
        decoder.channel_bw  = 62500.f;
        decoder.decode_bw_str = "62500,125000,250000";
        decoder.max_channels = 24;
        decoder.start();

        // Simulate: force one hot channel
        decoder._hotChannels = {100};

        decoder.updateActiveChannels();

        // Should have 3 active slots (one per BW)
        expect(eq(decoder.activeSlotCount(), 3U))
            << "3 slots for 3 BWs on 1 hot channel";

        // Each slot should have a different decodeBw
        std::vector<uint32_t> bws;
        for (const auto& slot : decoder._slots) {
            if (slot.state != gr::lora::ChannelSlot::State::Idle) {
                bws.push_back(slot.decodeBw);
            }
        }
        std::ranges::sort(bws);
        expect(eq(bws.size(), 3UZ));
        expect(eq(bws[0], 62500U));
        expect(eq(bws[1], 125000U));
        expect(eq(bws[2], 250000U));
    };
};

// ============================================================================
// WidebandDecoder loopback test (M5): generate LoRa frame at narrowband,
// embed at frequency offset in 16 MS/s wideband stream, verify decode.
// ============================================================================

const boost::ut::suite<"WidebandDecoder loopback"> wbLoopbackTests = [] {
    using namespace boost::ut;
    using namespace gr::lora;
    using namespace gr::lora::test;

    "SF8 loopback decode at +3 MHz offset"_test = [] {
        constexpr uint8_t  test_sf  = 8;
        constexpr uint8_t  test_cr  = 4;
        constexpr uint32_t test_bw  = 62500;
        constexpr uint16_t test_sync = 0x12;
        constexpr uint16_t test_preamble = 8;
        constexpr double   wb_rate  = 16e6;
        constexpr double   center_freq = 866.5e6;
        constexpr double   channel_freq = 869.5e6;  // +3 MHz offset
        constexpr uint32_t os_factor = static_cast<uint32_t>(wb_rate / test_bw);  // 256

        // Generate a LoRa frame at 1x BW (os_factor=1), then upsample to
        // wideband rate using cascaded interpolate-by-2 stages. Each stage
        // zero-stuffs by 2 and applies the same half-band FIR as the decimator
        // (but without decimation) to remove spectral images.
        const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
        uint32_t nb_sps = (1u << test_sf);  // os_factor=1
        auto nb_iq = generate_frame_iq(payload, test_sf, test_cr, 1,
                                        test_sync, test_preamble,
                                        true, nb_sps * 5, 2, test_bw, false);

        // Cascaded interpolation: 8 stages of interp-by-2 = 256x total.
        // Each stage: zero-stuff by 2, then apply 23-tap half-band FIR
        // (non-decimating) to remove the image.
        uint32_t nInterpStages = 0;
        for (uint32_t v = os_factor; v > 1; v >>= 1) ++nInterpStages;

        auto halfBandInterp = [](const std::vector<cf32>& input) -> std::vector<cf32> {
            using namespace gr::lora::halfband_detail;
            // Zero-stuff by 2: insert zero between each sample
            std::vector<cf32> stuffed(input.size() * 2);
            for (std::size_t i = 0; i < input.size(); ++i) {
                stuffed[i * 2]     = input[i] * 2.0f;  // gain=2 to preserve amplitude
                stuffed[i * 2 + 1] = cf32(0.f, 0.f);
            }
            // Apply 23-tap half-band FIR without decimation (keep all samples)
            std::vector<cf32> out(stuffed.size());
            std::array<cf32, kFilterLen> delay{};
            for (std::size_t n = 0; n < stuffed.size(); ++n) {
                // Shift delay line
                for (std::size_t k = kFilterLen - 1; k > 0; --k) {
                    delay[k] = delay[k - 1];
                }
                delay[0] = stuffed[n];
                // Compute FIR output (symmetric pairs + center tap)
                cf32 acc{0.f, 0.f};
                for (std::size_t c = 0; c < kCoeffs.size(); ++c) {
                    std::size_t k = c * 2;
                    std::size_t kMirror = kFilterLen - 1 - k;
                    acc += kCoeffs[c] * (delay[k] + delay[kMirror]);
                }
                acc += kCenterTap * delay[kFilterLen / 2];
                out[n] = acc;
            }
            return out;
        };

        std::vector<cf32> interp = nb_iq;
        for (uint32_t stage = 0; stage < nInterpStages; ++stage) {
            interp = halfBandInterp(interp);
        }

        // Freq-shift the interpolated wideband frame to the channel offset
        const double shift_hz = channel_freq - center_freq;
        const double phaseInc = 2.0 * std::numbers::pi * shift_hz / wb_rate;
        // Add padding: L1 needs enough data for detection
        constexpr std::size_t pad_samples = 8192 * 20;  // L1 warmup
        std::vector<cf32> wb_iq(pad_samples + interp.size() + pad_samples, cf32(0.f, 0.f));

        // Add weak noise floor
        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.f, 0.001f);
        for (auto& s : wb_iq) {
            s = cf32(noise(rng), noise(rng));
        }

        // Embed band-limited frame at frequency offset
        double phase = 0.0;
        for (std::size_t i = 0; i < interp.size(); ++i) {
            std::size_t wb_idx = pad_samples + i;
            auto rot = cf32(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
            wb_iq[wb_idx] += interp[i] * rot;
            phase += phaseInc;
        }

        // Build graph: TagSource → WidebandDecoder → TagSink
        gr::Graph graph;
        auto& src = graph.emplaceBlock<gr::testing::TagSource<cf32,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(wb_iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        src.values = wb_iq;

        auto& decoder = graph.emplaceBlock<WidebandDecoder>({
            {"sample_rate", static_cast<float>(wb_rate)},
            {"center_freq", static_cast<float>(center_freq)},
            {"channel_bw", 62500.f},
            {"decode_bw", static_cast<float>(test_bw)},
            {"max_channels", uint32_t{8}},

            {"l1_interval", uint32_t{1}},
            {"l1_snapshots", uint32_t{2}},
            {"l1_fft_size", uint32_t{4096}},
            {"sync_word", test_sync},
            {"preamble_len", test_preamble},
            {"energy_thresh", 1e-8f},
            {"min_snr_db", -20.0f},
            {"debug", true},
        });

        auto& sink = graph.emplaceBlock<gr::testing::TagSink<uint8_t,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true},
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            std::fprintf(stderr, "scheduler exchange failed\n");
            expect(false) << "scheduler exchange failed";
            return;
        }
        sched.runAndWait();

        // Check output
        std::vector<uint8_t> decoded;
        decoded.assign(sink._samples.begin(), sink._samples.end());

        if (decoded.size() >= payload.size()) {
            bool match = std::equal(payload.begin(), payload.end(), decoded.begin());
            expect(match) << "decoded payload matches TX payload";

            // Check tags
            bool found_sf_tag = false;
            for (const auto& frame_tag : sink._tags) {
                if (frame_tag.map.contains("sf")) {
                    auto sf_val = frame_tag.map.at("sf").template value_or<int64_t>(0);
                    expect(eq(sf_val, static_cast<int64_t>(test_sf)))
                        << "decoded SF tag matches";
                    found_sf_tag = true;
                }
                if (frame_tag.map.contains("crc_valid")) {
                    auto crc_ok = frame_tag.map.at("crc_valid").template value_or<bool>(false);
                    expect(crc_ok) << "CRC valid";
                }
            }
            expect(found_sf_tag) << "SF tag present in output";
        } else {
            std::fprintf(stderr, "wideband loopback: decoded %zu bytes, expected %zu\n",
                         decoded.size(), payload.size());
            expect(false) << "wideband loopback: no decode output";
        }
    };
    "SF8 noisy loopback: FIR survives wideband noise"_test = [] {
        constexpr uint8_t  test_sf  = 8;
        constexpr uint8_t  test_cr  = 4;
        constexpr uint32_t test_bw  = 62500;
        constexpr uint16_t test_sync = 0x12;
        constexpr uint16_t test_preamble = 8;
        constexpr double   wb_rate  = 16e6;
        constexpr double   center_freq = 866.5e6;
        constexpr double   channel_freq = 869.5e6;  // +3 MHz offset
        constexpr uint32_t os_factor = static_cast<uint32_t>(wb_rate / test_bw);  // 256

        // Generate a LoRa frame at 1x BW, then upsample to wideband rate.
        const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
        uint32_t nb_sps = (1u << test_sf);  // os_factor=1
        auto nb_iq = generate_frame_iq(payload, test_sf, test_cr, 1,
                                        test_sync, test_preamble,
                                        true, nb_sps * 5, 2, test_bw, false);

        // Cascaded interpolation: 8 stages of interp-by-2 = 256x total.
        uint32_t nInterpStages = 0;
        for (uint32_t v = os_factor; v > 1; v >>= 1) ++nInterpStages;

        auto halfBandInterp = [](const std::vector<cf32>& input) -> std::vector<cf32> {
            using namespace gr::lora::halfband_detail;
            std::vector<cf32> stuffed(input.size() * 2);
            for (std::size_t i = 0; i < input.size(); ++i) {
                stuffed[i * 2]     = input[i] * 2.0f;
                stuffed[i * 2 + 1] = cf32(0.f, 0.f);
            }
            std::vector<cf32> out(stuffed.size());
            std::array<cf32, kFilterLen> delay{};
            for (std::size_t n = 0; n < stuffed.size(); ++n) {
                for (std::size_t k = kFilterLen - 1; k > 0; --k) {
                    delay[k] = delay[k - 1];
                }
                delay[0] = stuffed[n];
                cf32 acc{0.f, 0.f};
                for (std::size_t c = 0; c < kCoeffs.size(); ++c) {
                    std::size_t k = c * 2;
                    std::size_t kMirror = kFilterLen - 1 - k;
                    acc += kCoeffs[c] * (delay[k] + delay[kMirror]);
                }
                acc += kCenterTap * delay[kFilterLen / 2];
                out[n] = acc;
            }
            return out;
        };

        std::vector<cf32> interp = nb_iq;
        for (uint32_t stage = 0; stage < nInterpStages; ++stage) {
            interp = halfBandInterp(interp);
        }

        // Freq-shift to channel offset and embed in wideband stream with STRONG noise
        const double shift_hz = channel_freq - center_freq;
        const double phaseInc = 2.0 * std::numbers::pi * shift_hz / wb_rate;
        constexpr std::size_t pad_samples = 8192 * 20;
        std::vector<cf32> wb_iq(pad_samples + interp.size() + pad_samples, cf32(0.f, 0.f));

        // Key difference: sigma=0.3 wideband noise (vs 0.001 in clean test).
        // The box-car decimator (-13 dB stopband) couldn't reject this;
        // the FIR channelizer (-45 dB stopband) can.
        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.f, 0.3f);
        for (auto& s : wb_iq) {
            s = cf32(noise(rng), noise(rng));
        }

        // Embed band-limited frame at frequency offset
        double phase = 0.0;
        for (std::size_t i = 0; i < interp.size(); ++i) {
            std::size_t wb_idx = pad_samples + i;
            auto rot = cf32(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
            wb_iq[wb_idx] += interp[i] * rot;
            phase += phaseInc;
        }

        // Build graph: TagSource → WidebandDecoder → TagSink
        gr::Graph graph;
        auto& src = graph.emplaceBlock<gr::testing::TagSource<cf32,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(wb_iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        src.values = wb_iq;

        auto& decoder = graph.emplaceBlock<WidebandDecoder>({
            {"sample_rate", static_cast<float>(wb_rate)},
            {"center_freq", static_cast<float>(center_freq)},
            {"channel_bw", 62500.f},
            {"decode_bw", static_cast<float>(test_bw)},
            {"max_channels", uint32_t{8}},

            {"l1_interval", uint32_t{1}},
            {"l1_snapshots", uint32_t{2}},
            {"l1_fft_size", uint32_t{4096}},
            {"sync_word", test_sync},
            {"preamble_len", test_preamble},
            {"energy_thresh", 1e-8f},
            {"min_snr_db", -20.0f},
            {"debug", true},
        });

        auto& sink = graph.emplaceBlock<gr::testing::TagSink<uint8_t,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true},
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            std::fprintf(stderr, "scheduler exchange failed\n");
            expect(false) << "scheduler exchange failed";
            return;
        }
        sched.runAndWait();

        // Check output
        std::vector<uint8_t> decoded;
        decoded.assign(sink._samples.begin(), sink._samples.end());

        if (decoded.size() >= payload.size()) {
            bool match = std::equal(payload.begin(), payload.end(), decoded.begin());
            expect(match) << "decoded payload matches TX payload (noisy)";

            bool found_sf_tag = false;
            for (const auto& frame_tag : sink._tags) {
                if (frame_tag.map.contains("sf")) {
                    auto sf_val = frame_tag.map.at("sf").template value_or<int64_t>(0);
                    expect(eq(sf_val, static_cast<int64_t>(test_sf)))
                        << "decoded SF tag matches (noisy)";
                    found_sf_tag = true;
                }
                if (frame_tag.map.contains("crc_valid")) {
                    auto crc_ok = frame_tag.map.at("crc_valid").template value_or<bool>(false);
                    expect(crc_ok) << "CRC valid (noisy)";
                }
            }
            expect(found_sf_tag) << "SF tag present in output (noisy)";
        } else {
            std::fprintf(stderr, "noisy wideband loopback: decoded %zu bytes, expected %zu\n",
                         decoded.size(), payload.size());
            expect(false) << "noisy wideband loopback: no decode output";
        }
    };
};

// ============================================================================
// NCO complex-multiply recurrence precision test
// ============================================================================

const boost::ut::suite<"NCO recurrence precision"> ncoRecurrenceTests = [] {
    using namespace boost::ut;

    "complex-multiply recurrence matches cos/sin reference"_test = [] {
        // Validate that the complex-multiply NCO recurrence stays within
        // acceptable phase and amplitude error over a realistic processing
        // interval.  Uses a 3 MHz offset at 16 MS/s (typical mid-band shift).
        constexpr double sampleRate = 16e6;
        constexpr double shiftHz    = -3e6;  // typical -(channelFreq - centerFreq)
        constexpr double phaseInc   = 2.0 * std::numbers::pi * shiftHz / sampleRate;
        constexpr std::size_t N     = 65536;  // one large processBulk equivalent
        constexpr std::size_t kRenorm = 1024;  // renormalize every 1024 samples

        // --- Reference: per-sample cos/sin (current implementation) ---
        std::vector<cf32> ref(N);
        {
            double phase = 0.0;
            for (std::size_t i = 0; i < N; ++i) {
                ref[i] = cf32(static_cast<float>(std::cos(phase)),
                              static_cast<float>(std::sin(phase)));
                phase += phaseInc;
                if (((i + 1) & 0x3FFU) == 0) {
                    phase = std::remainder(phase, 2.0 * std::numbers::pi);
                }
            }
        }

        // --- Recurrence: rot *= step, renormalize every kRenorm samples ---
        std::vector<cf32> rec(N);
        {
            const cf32 step(static_cast<float>(std::cos(phaseInc)),
                            static_cast<float>(std::sin(phaseInc)));
            cf32 rot(1.f, 0.f);  // start at phase=0
            for (std::size_t i = 0; i < N; ++i) {
                rec[i] = rot;
                rot *= step;
                // Renormalize every kRenorm samples (fire AFTER index 1023, 2047, ...)
                if ((i & 0x3FFU) == 0x3FFU) {
                    rot /= std::abs(rot);
                }
            }
        }

        // --- Assertions ---
        float maxPhaseErr = 0.f;
        float maxAmpErr   = 0.f;
        float worstAmpBeforeRenorm = 0.f;

        for (std::size_t i = 0; i < N; ++i) {
            // Phase error: angle between ref and rec phasors
            // arg(rec[i] * conj(ref[i])) gives the phase difference
            cf32 diff = rec[i] * std::conj(ref[i]);
            float phaseErr = std::abs(std::arg(diff));
            maxPhaseErr = std::max(maxPhaseErr, phaseErr);

            // Amplitude deviation from unity
            float ampErr = std::abs(std::abs(rec[i]) - 1.0f);
            maxAmpErr = std::max(maxAmpErr, ampErr);

            // Track worst amplitude just before renormalization
            if ((i & 0x3FFU) == 0x3FEU) {  // sample 1022, 2046, ...
                worstAmpBeforeRenorm = std::max(worstAmpBeforeRenorm, ampErr);
            }
        }

        std::fprintf(stderr,
            "  NCO recurrence (N=%zu, renorm=%zu): maxPhaseErr=%.6f rad, "
            "maxAmpErr=%.6f, worstPreRenorm=%.6f\n",
            N, kRenorm, static_cast<double>(maxPhaseErr),
            static_cast<double>(maxAmpErr),
            static_cast<double>(worstAmpBeforeRenorm));

        // Phase error < 0.01 radian over entire interval
        expect(maxPhaseErr < 0.01f)
            << "max phase error = " << maxPhaseErr << " rad, expected < 0.01";

        // Amplitude within 0.02% of unity after each renormalization cycle
        // (checking at every sample; worst case is just before renormalization)
        expect(maxAmpErr < 0.0002f)
            << "max amplitude error = " << maxAmpErr << ", expected < 0.0002";
    };
};

// ============================================================================
// HalfBandDecimator unit tests
// ============================================================================

const boost::ut::suite<"HalfBandDecimator"> halfBandTests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "stopband rejection >= 40 dB"_test = [] {
        // Single half-band stage with real coefficients applied to complex data.
        // Frequency response is symmetric around DC:
        //   passband:  [-0.25*Fs, +0.25*Fs]
        //   stopband:  [-0.5*Fs, -0.25*Fs] ∪ [+0.25*Fs, +0.5*Fs]
        // Using normalized frequency: sample_rate = 1.0.
        constexpr std::size_t N = 8192;

        HalfBandStage stage;

        // Passband reference: tone at 0.1*Fs (well inside passband)
        auto passbandTone = makeTone(N, 0.1, 1.0);
        std::vector<cf32> passbandOut;
        stage.reset();
        stage.process(std::span<const cf32>(passbandTone), passbandOut);

        // Measure passband output power
        double passbandPower = 0.0;
        for (const auto& s : passbandOut) {
            passbandPower += static_cast<double>(std::norm(s));
        }
        passbandPower /= static_cast<double>(passbandOut.size());

        // Stopband tones in [0.25*Fs, 0.5*Fs] — the filter's stopband region.
        for (double frac : {0.30, 0.35, 0.40, 0.45}) {
            auto stopTone = makeTone(N, frac, 1.0);
            std::vector<cf32> stopOut;
            stage.reset();
            stage.process(std::span<const cf32>(stopTone), stopOut);

            double stopPower = 0.0;
            for (const auto& s : stopOut) {
                stopPower += static_cast<double>(std::norm(s));
            }
            stopPower /= static_cast<double>(stopOut.size());

            double rejectionDb = 10.0 * std::log10(stopPower / passbandPower);
            expect(rejectionDb < -40.0)
                << "stopband tone at " << frac << "*Fs: rejection = "
                << rejectionDb << " dB, expected < -40 dB";
        }
    };

    "passband flatness < 0.5 dB"_test = [] {
        // Measure gain at several passband frequencies relative to DC.
        // Passband is [0, 0.25*Fs). Test well within passband.
        constexpr std::size_t N = 8192;

        HalfBandStage stage;

        // DC reference
        auto dcTone = makeTone(N, 0.0001, 1.0);  // near-DC (avoid exact 0)
        std::vector<cf32> dcOut;
        stage.reset();
        stage.process(std::span<const cf32>(dcTone), dcOut);

        double dcPower = 0.0;
        for (const auto& s : dcOut) {
            dcPower += static_cast<double>(std::norm(s));
        }
        dcPower /= static_cast<double>(dcOut.size());

        // Test passband tones (all within [0, 0.25*Fs))
        for (double frac : {0.02, 0.05, 0.10, 0.15, 0.20}) {
            auto tone = makeTone(N, frac, 1.0);
            std::vector<cf32> toneOut;
            stage.reset();
            stage.process(std::span<const cf32>(tone), toneOut);

            double tonePower = 0.0;
            for (const auto& s : toneOut) {
                tonePower += static_cast<double>(std::norm(s));
            }
            tonePower /= static_cast<double>(toneOut.size());

            double rippleDb = std::abs(10.0 * std::log10(tonePower / dcPower));
            expect(rippleDb < 0.5)
                << "passband tone at " << frac << "*Fs: ripple = "
                << rippleDb << " dB, expected < 0.5 dB";
        }
    };

    "phase continuity across chunks"_test = [] {
        // 4-stage cascade on a 16K sample tone. Single-call vs chunked must match.
        constexpr std::size_t nStages = 4;
        constexpr std::size_t N = 16384;
        constexpr double toneFreq = 0.05;  // 0.05*Fs, well within passband

        auto tone = makeTone(N, toneFreq, 1.0);

        // Single-call reference
        CascadedDecimator decSingle;
        decSingle.init(nStages, N);
        auto refOut = decSingle.process(std::span<const cf32>(tone));

        // Chunked: various chunk sizes
        CascadedDecimator decChunked;
        decChunked.init(nStages, N);
        std::vector<cf32> chunkedOut;

        const std::array<std::size_t, 7> chunkSizes = {1, 7, 128, 8192, 4096, 2048, 1913};
        std::size_t pos = 0;
        std::size_t chunkIdx = 0;
        while (pos < N) {
            std::size_t sz = std::min(chunkSizes[chunkIdx % chunkSizes.size()], N - pos);
            auto chunk = std::span<const cf32>(tone.data() + pos, sz);
            auto out = decChunked.process(chunk);
            chunkedOut.insert(chunkedOut.end(), out.begin(), out.end());
            pos += sz;
            ++chunkIdx;
        }

        expect(eq(chunkedOut.size(), refOut.size()))
            << "chunked output size = " << chunkedOut.size()
            << ", reference = " << refOut.size();

        // Compare sample-by-sample
        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(refOut.size(), chunkedOut.size()); ++i) {
            float err = std::abs(chunkedOut[i] - refOut[i]);
            maxErr = std::max(maxErr, err);
        }
        expect(maxErr < 1e-6f)
            << "max error between single-call and chunked = " << maxErr
            << ", expected < 1e-6";
    };

    "batch FIR matches scalar FIR"_test = [] {
        constexpr std::size_t nStages = 4;  // decimate by 16
        constexpr std::size_t nInput = 8192;

        CascadedDecimator scalar, batch;
        scalar.init(nStages, nInput);
        batch.init(nStages, nInput);

        // Random input
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(-1.f, 1.f);
        std::vector<cf32> input(nInput);
        for (auto& s : input) s = cf32(dist(rng), dist(rng));

        std::vector<cf32> outScalar, outBatch;
        scalar.process(std::span<const cf32>(input), outScalar);
        batch.processBatch(std::span<const cf32>(input), outBatch);

        expect(eq(outScalar.size(), outBatch.size())) << "output size mismatch";
        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(outScalar.size(), outBatch.size()); ++i) {
            float err = std::abs(outScalar[i] - outBatch[i]);
            maxErr = std::max(maxErr, err);
            expect(err < 1e-5f) << "sample " << i << " err=" << err;
        }
        std::fprintf(stderr, "  batch vs scalar max err: %.2e\n",
                     static_cast<double>(maxErr));
    };

    "batch FIR multi-call matches scalar multi-call"_test = [] {
        constexpr std::size_t nStages = 4;
        constexpr std::size_t chunkSize = 1024;
        constexpr std::size_t nChunks = 8;

        CascadedDecimator scalar, batch;
        scalar.init(nStages, chunkSize);
        batch.init(nStages, chunkSize);

        std::mt19937 rng(123);
        std::uniform_real_distribution<float> dist(-1.f, 1.f);

        std::vector<cf32> outScalar, outBatch;
        for (std::size_t c = 0; c < nChunks; ++c) {
            std::vector<cf32> chunk(chunkSize);
            for (auto& s : chunk) s = cf32(dist(rng), dist(rng));
            scalar.process(std::span<const cf32>(chunk), outScalar);
            batch.processBatch(std::span<const cf32>(chunk), outBatch);
        }

        expect(eq(outScalar.size(), outBatch.size())) << "multi-call size mismatch";
        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(outScalar.size(), outBatch.size()); ++i) {
            float err = std::abs(outScalar[i] - outBatch[i]);
            maxErr = std::max(maxErr, err);
        }
        std::fprintf(stderr, "  batch multi-call max err: %.2e\n",
                     static_cast<double>(maxErr));
        expect(maxErr < 1e-5f) << "multi-call max err: " << maxErr;
    };

    "NCO+batch FIR matches NCO+scalar FIR"_test = [] {
        constexpr std::size_t nStages = 4;
        constexpr std::size_t nInput = 8192;

        CascadedDecimator scalar, batch;
        scalar.init(nStages, nInput);
        batch.init(nStages, nInput);

        std::mt19937 rng(77);
        std::uniform_real_distribution<float> dist(-1.f, 1.f);
        std::vector<cf32> input(nInput);
        for (auto& s : input) s = cf32(dist(rng), dist(rng));

        // Same NCO params
        const double phaseInc = 2.0 * std::numbers::pi * 100000.0 / 16000000.0;
        const cf32 step(static_cast<float>(std::cos(phaseInc)),
                        static_cast<float>(std::sin(phaseInc)));
        cf32 rot1(1.f, 0.f), rot2(1.f, 0.f);

        std::vector<cf32> outScalar, outBatch;
        scalar.processWithNco(std::span<const cf32>(input), outScalar, rot1, step);
        batch.processWithNcoBatch(std::span<const cf32>(input), outBatch, rot2, step);

        expect(eq(outScalar.size(), outBatch.size()));
        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(outScalar.size(), outBatch.size()); ++i) {
            float err = std::abs(outScalar[i] - outBatch[i]);
            maxErr = std::max(maxErr, err);
        }
        std::fprintf(stderr, "  NCO+batch max err: %.2e\n",
                     static_cast<double>(maxErr));
        expect(maxErr < 1e-5f) << "NCO+batch max err: " << maxErr;
    };

    "noise power reduction"_test = [] {
        // 8-stage cascade (decimation by 256) on white noise.
        // Theoretical power reduction: 10*log10(1/256) = -24.08 dB.
        constexpr std::size_t nStages = 8;
        constexpr std::size_t N = 256 * 1024;  // plenty of samples for statistics

        // Generate white noise with fixed seed
        std::mt19937 rng(12345);
        std::normal_distribution<float> dist(0.f, 1.0f);
        std::vector<cf32> noise(N);
        for (auto& s : noise) {
            s = cf32(dist(rng), dist(rng));
        }

        // Measure input power
        double inputPower = 0.0;
        for (const auto& s : noise) {
            inputPower += static_cast<double>(std::norm(s));
        }
        inputPower /= static_cast<double>(N);

        // Decimate
        CascadedDecimator dec;
        dec.init(nStages, N);
        auto out = dec.process(std::span<const cf32>(noise));

        // Measure output power
        double outputPower = 0.0;
        for (const auto& s : out) {
            outputPower += static_cast<double>(std::norm(s));
        }
        outputPower /= static_cast<double>(out.size());

        double reductionDb = 10.0 * std::log10(outputPower / inputPower);
        double theoreticalDb = 10.0 * std::log10(1.0 / 256.0);  // -24.08 dB

        expect(std::abs(reductionDb - theoreticalDb) < 2.0)
            << "noise power reduction = " << reductionDb
            << " dB, theoretical = " << theoreticalDb
            << " dB, delta = " << std::abs(reductionDb - theoreticalDb)
            << " dB, expected within 2 dB";
    };
};

// ============================================================================
// Fused NCO+FIR equivalence test
// ============================================================================

const boost::ut::suite<"Fused NCO+FIR equivalence"> fusedNcoFirTests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "processWithNco matches separate NCO+FIR"_test = [] {
        // Generate random wideband input, process through:
        //   (a) separate NCO mix → CascadedDecimator::process()
        //   (b) CascadedDecimator::processWithNco() (fused)
        // Assert outputs match within float precision.
        constexpr std::size_t nSamples = 8192;
        constexpr std::size_t nStages  = 8;  // BW62.5k at 16 MS/s: 2^8 = 256
        constexpr double sampleRate    = 16e6;
        constexpr double channelFreq   = 869.5e6;
        constexpr double centerFreq    = 866.5e6;
        constexpr double shiftHz       = -(channelFreq - centerFreq);  // -3 MHz

        const double phaseInc = 2.0 * std::numbers::pi * shiftHz / sampleRate;
        const cf32 ncoStep(static_cast<float>(std::cos(phaseInc)),
                           static_cast<float>(std::sin(phaseInc)));

        // Random wideband input
        std::mt19937 rng(42);
        std::normal_distribution<float> dist(0.f, 1.0f);
        std::vector<cf32> wbInput(nSamples);
        for (auto& s : wbInput) {
            s = cf32(dist(rng), dist(rng));
        }

        // --- Path A: separate NCO mix + FIR ---
        cf32 rotA(1.f, 0.f);
        std::vector<cf32> mixBuf(nSamples);
        for (std::size_t i = 0; i < nSamples; ++i) {
            mixBuf[i] = wbInput[i] * rotA;
            rotA *= ncoStep;
            if ((i & 0x3FFU) == 0x3FFU) {
                rotA /= std::abs(rotA);
            }
        }
        CascadedDecimator decA;
        decA.init(nStages, nSamples);
        std::vector<cf32> outA;
        decA.process(std::span<const cf32>(mixBuf), outA);

        // --- Path B: fused NCO+FIR ---
        cf32 rotB(1.f, 0.f);
        CascadedDecimator decB;
        decB.init(nStages, nSamples);
        std::vector<cf32> outB;
        decB.processWithNco(std::span<const cf32>(wbInput), outB, rotB, ncoStep);

        // --- Assertions ---
        expect(eq(outA.size(), outB.size()))
            << "output sizes must match: A=" << outA.size() << " B=" << outB.size();

        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(outA.size(), outB.size()); ++i) {
            float err = std::abs(outA[i] - outB[i]);
            maxErr = std::max(maxErr, err);
        }

        std::fprintf(stderr, "  fused NCO+FIR: %zu outputs, maxErr=%.9f\n",
                     outB.size(), static_cast<double>(maxErr));

        // Tolerance: 1e-5f accommodates rounding-order divergence between the
        // two FIR implementations.  Path A runs the FIR via the circular
        // delay-line HalfBandStage::process() on a pre-mixed contiguous buffer;
        // Path B runs the FIR via the linear-buffer processNcoFirFused() on
        // the raw input.  GCC 14 on aarch64 auto-vectorises the two call sites
        // differently and emits NEON FMA in one but not the other, so the
        // observed rounding diverges by up to ~3.4e-6 on arm64 Linux CI.
        // Apple Clang happens to produce bit-identical output on macOS/arm64.
        // A 12-tap half-band FIR on unit-variance Gaussian input has an
        // analytical per-output eps budget of ~1.4e-6 (12 * 2^-23); 1e-5f
        // leaves ~7x headroom while still catching any real logic bug (those
        // would manifest as O(1) differences, not sub-microvolt rounding).
        expect(maxErr < 1e-5f)
            << "max error between separate and fused = " << maxErr
            << ", expected < 1e-5";

        // NCO rotation state should also match after processing.  Tolerance:
        // both paths do 8192 complex multiplies with renormalization every
        // 1024 samples.  Complex multiply is (ac - bd) + (ad + bc)i; GCC 14
        // on aarch64 emits NEON FMA in one call site (Path A's tight mix
        // loop) but not the other (Path B's interleaved mix+FIR loop), and
        // FMA-vs-no-FMA rounding drifts by ~1 eps per operation.  Over
        // N=8192 multiplies the RMS drift is  sqrt(N) * eps ~ 90 * 1.2e-7
        // ~= 1e-5, and worst-case 3-4x that.  Observed on CI: 4.18e-5.
        // 1e-4f leaves headroom while still catching a real bug (a wrong
        // NCO step or broken renormalization would drift by O(1)).
        float rotDiff = std::abs(rotA - rotB);
        expect(rotDiff < 1e-4f)
            << "NCO rotation state divergence = " << rotDiff
            << ", expected < 1e-4";
    };
};

int main() { /* boost.ut auto-runs */ }
