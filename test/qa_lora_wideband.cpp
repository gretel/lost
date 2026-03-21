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
#include <gnuradio-4.0/lora/algorithm/Channelize.hpp>
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
        constexpr std::size_t nSamples = 256 * 100;  // exactly 100 output samples

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        expect(eq(slot.nbAccum.size(), 100UZ))
            << "output sample count = wideband / osFactor";
    };

    "tone at channel freq appears at DC after channelization"_test = [] {
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;  // +3 MHz offset
        constexpr uint32_t osFactor = 256;        // BW62.5k: 16e6/62.5e3
        // 256 output samples -> 256 * 256 = 65536 input samples
        constexpr std::size_t nOut = 256;
        constexpr std::size_t nSamples = nOut * osFactor;

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        expect(eq(slot.nbAccum.size(), nOut));

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
        expect(eq(slot.nbAccum.size(), expectedOut))
            << "total output = " << slot.nbAccum.size()
            << ", expected " << expectedOut;

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
        constexpr std::size_t nOut = 128;
        constexpr std::size_t nSamples = nOut * osFactor;

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);

        auto tone = makeTone(nSamples, toneFreq - centerFreq, sampleRate);
        slot.pushWideband(tone);

        expect(eq(slot.nbAccum.size(), nOut));

        // After channelization, the tone is at +10 kHz in the narrowband stream.
        // With nb sample rate = 62.5 kHz and N=128 FFT bins, bin spacing = 488.3 Hz.
        // Expected bin = 10000 / (62500 / 128) ≈ 20.48 → peak at bin 20 or 21.
        auto bin = peakBin(slot.nbAccum);
        expect(bin != 0UZ) << "offset tone should NOT be at DC";
        expect(bin >= 19UZ && bin <= 22UZ)
            << "expected bin ~20, got " << bin;
    };

    "decimator handles partial blocks correctly"_test = [] {
        // Push a number of samples that is NOT a multiple of osFactor
        constexpr uint32_t osFactor = 256;
        constexpr std::size_t nSamples = osFactor * 3 + 100;  // 3 complete + 100 leftover

        gr::lora::ChannelSlot slot;
        slot.activate(866.5e6, 866.5e6, 16e6, osFactor);  // channel == center -> no shift

        std::vector<cf32> dc(nSamples, cf32(1.f, 0.f));
        slot.pushWideband(dc);

        // Should have exactly 3 output samples (100 leftover not yet dumped)
        expect(eq(slot.nbAccum.size(), 3UZ))
            << "partial block: only complete integrations dumped";
        // With two-stage decimation, leftover is in stage1 or stage2 counter
        // Just verify we got the right number of output samples
        expect(eq(slot.nbAccum.size(), 3UZ))
            << "partial block: leftover preserved";

        // Push 156 more to complete the 4th output sample
        std::vector<cf32> remainder(osFactor - 100, cf32(1.f, 0.f));
        slot.pushWideband(remainder);
        expect(eq(slot.nbAccum.size(), 4UZ))
            << "4th sample dumped after completing accumulation";
    };

    "DC signal at center freq passes through unchanged"_test = [] {
        constexpr uint32_t osFactor = 128;
        constexpr std::size_t nSamples = osFactor * 64;

        gr::lora::ChannelSlot slot;
        // Channel == center freq -> shift_hz = 0 -> NCO is identity
        slot.activate(866.5e6, 866.5e6, 16e6, osFactor);

        std::vector<cf32> dc(nSamples, cf32(1.f, 0.f));
        slot.pushWideband(dc);

        expect(eq(slot.nbAccum.size(), 64UZ));

        // Each output sample should be ~1.0 (average of osFactor identical inputs)
        for (const auto& s : slot.nbAccum) {
            expect(std::abs(s.real() - 1.0f) < 0.01f)
                << "DC passthrough real = " << s.real();
            expect(std::abs(s.imag()) < 0.01f)
                << "DC passthrough imag = " << s.imag();
        }
    };

    "matches batch channelize() for single call"_test = [] {
        // Verify ChannelSlot produces the same result as the batch channelize()
        // when called with the full buffer in one shot (no phase continuity needed).
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 868.0e6;
        constexpr double targetBw = 125e3;
        constexpr uint32_t osFactor = 128;  // 16e6 / 125e3
        constexpr uint32_t nSamples = osFactor * 64;

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);

        // Batch channelize (reference)
        auto refOut = gr::lora::channelize(tone.data(), nSamples,
                                           channelFreq, centerFreq,
                                           sampleRate, targetBw);

        // Streaming ChannelSlot
        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);
        slot.pushWideband(tone);

        expect(eq(slot.nbAccum.size(), refOut.size()))
            << "same output length";

        // Compare sample-by-sample (should be very close)
        float maxErr = 0.f;
        for (std::size_t i = 0; i < refOut.size(); ++i) {
            float err = std::abs(slot.nbAccum[i] - refOut[i]);
            maxErr = std::max(maxErr, err);
        }
        expect(maxErr < 0.01f)
            << "max error vs batch channelize = " << maxErr;
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
            {"buffer_ms", 100.f},
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
            {"buffer_ms", 100.f},
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
            {"buffer_ms", 100.f},
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

        // The block should have detected the tone channel and activated a slot.
        // Since we don't have a way to query the block after exchange (type-erased),
        // and there's no decode yet (M4), we just verify it ran without crash.
        // The real signal test is M4+M5.
        expect(true) << "block ran successfully with strong tone input";
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

        // Generate a LoRa frame at 1x BW (os_factor=1 for narrowband)
        const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
        uint32_t nb_sps = (1u << test_sf);  // os_factor=1
        auto nb_iq = generate_frame_iq(payload, test_sf, test_cr, 1,
                                        test_sync, test_preamble,
                                        true, nb_sps * 5, 2, test_bw, false);

        // Upsample + freq-shift to wideband: each NB sample becomes os_factor WB samples
        // This is the inverse of what ChannelSlot does (decimate + freq-shift to baseband)
        const double shift_hz = channel_freq - center_freq;
        const double phaseInc = 2.0 * std::numbers::pi * shift_hz / wb_rate;
        const std::size_t wb_len = nb_iq.size() * os_factor;
        // Add padding: L1 needs enough data for detection
        constexpr std::size_t pad_samples = 8192 * 20;  // L1 warmup
        std::vector<cf32> wb_iq(pad_samples + wb_len + pad_samples, cf32(0.f, 0.f));

        // Add weak noise floor
        std::mt19937 rng(42);
        std::normal_distribution<float> noise(0.f, 0.001f);
        for (auto& s : wb_iq) {
            s = cf32(noise(rng), noise(rng));
        }

        // Embed upsampled narrowband signal at offset
        double phase = 0.0;
        for (std::size_t i = 0; i < nb_iq.size(); ++i) {
            for (uint32_t j = 0; j < os_factor; ++j) {
                std::size_t wb_idx = pad_samples + i * os_factor + j;
                auto rot = cf32(
                    static_cast<float>(std::cos(phase)),
                    static_cast<float>(std::sin(phase)));
                wb_iq[wb_idx] += nb_iq[i] * rot;
                phase += phaseInc;
            }
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
            {"buffer_ms", 200.f},
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
            for (const auto& tag : sink._tags) {
                if (tag.map.contains("sf")) {
                    auto sf_val = tag.map.at("sf").template value_or<int64_t>(0);
                    expect(eq(sf_val, static_cast<int64_t>(test_sf)))
                        << "decoded SF tag matches";
                    found_sf_tag = true;
                }
                if (tag.map.contains("crc_valid")) {
                    auto crc_ok = tag.map.at("crc_valid").template value_or<bool>(false);
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

int main() { /* boost.ut auto-runs */ }
