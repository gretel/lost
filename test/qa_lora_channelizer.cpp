// SPDX-License-Identifier: ISC
/// Tests for FreqXlatingDecimator and firdes_low_pass.
///
/// Test progression:
///   1. firdes_low_pass: tap count, DC gain, symmetry, stopband attenuation
///   2. FreqXlatingDecimator: frequency shift, decimation, FIR filtering
///   3. Integration: wideband LoRa frame → channelizer → BurstDetector → SymbolDemodulator

#include "test_helpers.hpp"

#include <cmath>
#include <numbers>
#include <numeric>
#include <random>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/FreqXlatingDecimator.hpp>
#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>
#include <gnuradio-4.0/lora/algorithm/firdes.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace gr::lora::test;

namespace {

/// Compute the frequency response magnitude of a real FIR filter at a given
/// normalized frequency (0 = DC, 0.5 = Nyquist).
double fir_response_db(const std::vector<float>& taps, double freq_normalized) {
    double re = 0.0, im = 0.0;
    for (std::size_t n = 0; n < taps.size(); n++) {
        double phase = -2.0 * std::numbers::pi * freq_normalized * static_cast<double>(n);
        re += static_cast<double>(taps[n]) * std::cos(phase);
        im += static_cast<double>(taps[n]) * std::sin(phase);
    }
    double mag = std::sqrt(re * re + im * im);
    return 20.0 * std::log10(std::max(mag, 1e-15));
}

}  // namespace

// ============================================================================
// Test 1: firdes_low_pass — FIR filter design
// ============================================================================

const boost::ut::suite<"firdes_low_pass"> firdes_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "Correct number of taps"_test = [] {
        auto taps = firdes_low_pass(51, 250000.0, 31250.0);
        expect(eq(taps.size(), 51UZ));

        taps = firdes_low_pass(101, 1000000.0, 62500.0);
        expect(eq(taps.size(), 101UZ));

        taps = firdes_low_pass(1, 250000.0, 31250.0);
        expect(eq(taps.size(), 1UZ));
    };

    "Edge cases: zero taps, zero rate, zero cutoff"_test = [] {
        auto t0 = firdes_low_pass(0, 250000.0, 31250.0);
        expect(eq(t0.size(), 0UZ));

        auto t1 = firdes_low_pass(51, 0.0, 31250.0);
        // All zeros — no valid filter
        float sum = 0.f;
        for (auto v : t1) sum += std::abs(v);
        expect(lt(sum, 1e-6f));

        auto t2 = firdes_low_pass(51, 250000.0, 0.0);
        sum = 0.f;
        for (auto v : t2) sum += std::abs(v);
        expect(lt(sum, 1e-6f));
    };

    "Unity DC gain"_test = [] {
        auto taps = firdes_low_pass(51, 250000.0, 31250.0);
        double sum = 0.0;
        for (auto t : taps) sum += static_cast<double>(t);
        // DC gain should be 1.0 (unity)
        expect(lt(std::abs(sum - 1.0), 1e-6))
            << "DC gain: " << sum << " (expected 1.0)";
    };

    "Symmetry (linear phase)"_test = [] {
        auto taps = firdes_low_pass(51, 250000.0, 31250.0);
        float max_asym = 0.f;
        for (std::size_t i = 0; i < taps.size() / 2; i++) {
            float diff = std::abs(taps[i] - taps[taps.size() - 1 - i]);
            if (diff > max_asym) max_asym = diff;
        }
        expect(lt(max_asym, 1e-6f))
            << "Max asymmetry: " << max_asym;
    };

    "Passband flat, stopband attenuated"_test = [] {
        // Design a filter: 250 kHz sample rate, 31.25 kHz cutoff (BW/2 for
        // decimation 4 from 250 kHz to 62.5 kHz).
        auto taps = firdes_low_pass(65, 250000.0, 31250.0);

        // Passband: at DC and 20 kHz should be near 0 dB
        double dc_db = fir_response_db(taps, 0.0);
        double pass_db = fir_response_db(taps, 20000.0 / 250000.0);
        expect(lt(std::abs(dc_db), 0.1))
            << "DC response: " << dc_db << " dB";
        expect(gt(pass_db, -3.0))
            << "Passband (20 kHz) response: " << pass_db << " dB";

        // Stopband: at 100 kHz (0.4 Nyquist) should be well attenuated
        double stop_db = fir_response_db(taps, 100000.0 / 250000.0);
        expect(lt(stop_db, -30.0))
            << "Stopband (100 kHz) response: " << stop_db << " dB";

        std::printf("  FIR response: DC=%.1f dB, 20kHz=%.1f dB, 100kHz=%.1f dB\n",
                    dc_db, pass_db, stop_db);
    };

    "Longer filter gives better stopband"_test = [] {
        auto taps_short = firdes_low_pass(31, 250000.0, 31250.0);
        auto taps_long  = firdes_low_pass(101, 250000.0, 31250.0);

        double stop_short = fir_response_db(taps_short, 100000.0 / 250000.0);
        double stop_long  = fir_response_db(taps_long,  100000.0 / 250000.0);

        expect(lt(stop_long, stop_short))
            << "Longer filter should have better stopband: "
            << stop_long << " vs " << stop_short << " dB";
    };
};

// ============================================================================
// Test 2: FreqXlatingDecimator — unit tests
// ============================================================================

const boost::ut::suite<"FreqXlatingDecimator unit"> xlating_unit_tests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::lora;

    "Frequency shift accuracy (no decimation, no FIR)"_test = [] {
        // Generate a pure tone at 50 kHz, shift it to baseband (0 Hz)
        // using center_freq_offset = +50 kHz. No filtering, decimation = 1.
        constexpr double fs = 250000.0;
        constexpr double tone_freq = 50000.0;
        constexpr std::size_t n_samples = 1024;

        std::vector<std::complex<float>> input(n_samples);
        for (std::size_t n = 0; n < n_samples; n++) {
            double phase = 2.0 * std::numbers::pi * tone_freq
                         * static_cast<double>(n) / fs;
            input[n] = std::complex<float>(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = input;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = fs;
        xlat.center_freq_offset = tone_freq;
        xlat.decimation = 1;
        // No taps → pass-through mode (freq shift only)

        auto& sink = graph.emplaceBlock<testing::TagSink<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", false}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        // After shifting +50 kHz tone by -50 kHz, result should be ~DC.
        // Check: imaginary part should be near zero, real part ~constant.
        expect(ge(sink._samples.size(), n_samples / 2))
            << "Output should have samples: " << sink._samples.size();

        if (sink._samples.size() > 100) {
            // Skip first few samples (transient), check later samples
            float max_imag = 0.f;
            for (std::size_t i = 50; i < sink._samples.size(); i++) {
                float im = std::abs(sink._samples[i].imag());
                if (im > max_imag) max_imag = im;
            }
            // At DC, imaginary part should be very small
            expect(lt(max_imag, 0.05f))
                << "After shift to DC, max imag component: " << max_imag;

            // Real part should be approximately constant magnitude (~1)
            float min_real = 2.f, max_real = -2.f;
            for (std::size_t i = 50; i < sink._samples.size(); i++) {
                float r = sink._samples[i].real();
                if (r < min_real) min_real = r;
                if (r > max_real) max_real = r;
            }
            float variation = max_real - min_real;
            expect(lt(variation, 0.1f))
                << "Real part should be ~constant: range " << variation;
        }
    };

    "Decimation ratio correct"_test = [] {
        constexpr double fs = 250000.0;
        constexpr std::size_t n_samples = 4000;
        constexpr uint32_t dec = 4;

        std::vector<std::complex<float>> input(n_samples, {1.f, 0.f});

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = input;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = fs;
        xlat.center_freq_offset = 0.0;
        xlat.decimation = dec;
        // No taps: pass-through with decimation

        auto& sink = graph.emplaceBlock<testing::TagSink<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", false}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::size_t expected = n_samples / dec;
        expect(eq(sink._samples.size(), expected))
            << "Decimation " << dec << ": expected " << expected
            << " got " << sink._samples.size();
    };

    "FIR filtering attenuates out-of-band"_test = [] {
        // Input: DC (in-band) + 100 kHz tone (out-of-band for BW/2=31.25 kHz)
        constexpr double fs = 250000.0;
        constexpr double oob_freq = 100000.0;
        constexpr std::size_t n_samples = 4000;
        constexpr uint32_t dec = 4;

        auto taps = firdes_low_pass(65, fs, 31250.0);

        std::vector<std::complex<float>> input(n_samples);
        for (std::size_t n = 0; n < n_samples; n++) {
            double phase = 2.0 * std::numbers::pi * oob_freq
                         * static_cast<double>(n) / fs;
            // DC component (1.0) + out-of-band tone (1.0)
            input[n] = std::complex<float>(
                1.f + static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = input;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = fs;
        xlat.center_freq_offset = 0.0;
        xlat.decimation = dec;
        xlat.setTaps(taps);

        auto& sink = graph.emplaceBlock<testing::TagSink<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", false}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::size_t expected_out = n_samples / dec;
        expect(eq(sink._samples.size(), expected_out))
            << "Output size: " << sink._samples.size();

        // After FIR filter settles (skip first taps.size()/dec transient samples),
        // the output should be ~DC only. The 100 kHz tone should be attenuated.
        if (sink._samples.size() > 50) {
            float max_imag = 0.f;
            float mean_real = 0.f;
            std::size_t start = 30;  // skip filter transient
            std::size_t count = sink._samples.size() - start;
            for (std::size_t i = start; i < sink._samples.size(); i++) {
                mean_real += sink._samples[i].real();
                float im = std::abs(sink._samples[i].imag());
                if (im > max_imag) max_imag = im;
            }
            mean_real /= static_cast<float>(count);

            // DC component should survive (~1.0)
            expect(gt(mean_real, 0.5f))
                << "DC should survive filtering: mean_real=" << mean_real;

            // Out-of-band should be suppressed (imaginary part from 100 kHz tone)
            expect(lt(max_imag, 0.1f))
                << "100 kHz tone should be attenuated: max_imag=" << max_imag;

            std::printf("  FIR filtering: mean_real=%.3f, max_imag=%.4f\n",
                        static_cast<double>(mean_real),
                        static_cast<double>(max_imag));
        }
    };

    "Shift + decimate combined"_test = [] {
        // A 50 kHz tone at 250 kHz sample rate. Shift by -50 kHz and decimate
        // by 4 to get a DC signal at 62.5 kHz sample rate.
        constexpr double fs = 250000.0;
        constexpr double tone_freq = 50000.0;
        constexpr std::size_t n_samples = 4000;
        constexpr uint32_t dec = 4;

        auto taps = firdes_low_pass(65, fs, 31250.0);

        std::vector<std::complex<float>> input(n_samples);
        for (std::size_t n = 0; n < n_samples; n++) {
            double phase = 2.0 * std::numbers::pi * tone_freq
                         * static_cast<double>(n) / fs;
            input[n] = std::complex<float>(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = input;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = fs;
        xlat.center_freq_offset = tone_freq;
        xlat.decimation = dec;
        xlat.setTaps(taps);

        auto& sink = graph.emplaceBlock<testing::TagSink<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", false}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::size_t expected_out = n_samples / dec;
        expect(eq(sink._samples.size(), expected_out))
            << "Output: " << sink._samples.size() << " expected " << expected_out;

        // After shifting +50 kHz to DC and filtering, output should be ~DC.
        if (sink._samples.size() > 50) {
            float max_imag = 0.f;
            std::size_t start = 30;
            for (std::size_t i = start; i < sink._samples.size(); i++) {
                float im = std::abs(sink._samples[i].imag());
                if (im > max_imag) max_imag = im;
            }
            expect(lt(max_imag, 0.05f))
                << "After shift+decimate, max imag: " << max_imag;
        }
    };
};

// ============================================================================
// Test 3: Integration — wideband LoRa frame through channelizer to decoded payload
// ============================================================================

const boost::ut::suite<"Channelizer integration"> integration_tests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::lora;

    "LoRa frame at +50 kHz offset decoded through channelizer"_test = [] {
        // 1. Generate a LoRa frame at baseband (BW = 62.5 kHz, sample rate = BW = 62.5 kHz)
        //    using os_factor=1 since we'll embed it in a wideband stream ourselves.
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};

        // Generate at os_factor=1 (1 sample per chip)
        auto frame_iq = generate_frame_iq(payload, SF, CR, 1 /*os_factor*/,
                                          SYNC_WORD, PREAMBLE_LEN,
                                          true, N * 5,   // zero_pad
                                          2, BW, false);

        // 2. Upsample to wideband (250 kHz) and place at +50 kHz offset
        constexpr double wb_rate = 250000.0;
        constexpr double channel_offset = 50000.0;
        constexpr uint32_t upsample = 4;  // 62.5 kHz * 4 = 250 kHz

        std::size_t wb_len = frame_iq.size() * upsample + 1024;  // extra padding
        std::vector<std::complex<float>> wideband(wb_len, {0.f, 0.f});

        // Simple upsample-by-4: insert original samples every 4th position,
        // then shift to +50 kHz. For a proper test, we don't need perfect
        // interpolation — the BurstDetector at os_factor=4 will handle the
        // equivalent of seeing 4 samples per chip.
        for (std::size_t i = 0; i < frame_iq.size(); i++) {
            // Zero-order hold: replicate each sample 4 times (crude but
            // preserves the chirp structure for os_factor=4 detection)
            for (uint32_t u = 0; u < upsample; u++) {
                std::size_t idx = i * upsample + u;
                if (idx < wideband.size()) {
                    wideband[idx] = frame_iq[i];
                }
            }
        }

        // Shift to +50 kHz
        for (std::size_t n = 0; n < wideband.size(); n++) {
            double phase = 2.0 * std::numbers::pi * channel_offset
                         * static_cast<double>(n) / wb_rate;
            auto phasor = std::complex<float>(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
            wideband[n] *= phasor;
        }

        // 3. Channelize: shift -50 kHz, filter, decimate 4x → 62.5 kHz baseband
        //    Then feed to BurstDetector (os_factor=1) + SymbolDemodulator
        auto taps = firdes_low_pass(65, wb_rate, 31250.0);

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(wideband.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = wideband;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = wb_rate;
        xlat.center_freq_offset = channel_offset;
        xlat.decimation = upsample;
        xlat.setTaps(taps);

        auto& burst = graph.emplaceBlock<BurstDetector>();
        burst.center_freq  = CENTER_FREQ;
        burst.bandwidth    = BW;
        burst.sf           = SF;
        burst.sync_word    = SYNC_WORD;
        burst.os_factor    = 1;  // output from channelizer is at BW rate
        burst.preamble_len = PREAMBLE_LEN;

        auto& demod = graph.emplaceBlock<SymbolDemodulator>();
        demod.sf        = SF;
        demod.bandwidth = BW;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(burst);
        (void)graph.connect<"out">(burst).template to<"in">(demod);
        (void)graph.connect<"out">(demod).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::printf("Channelizer integration: %zu bytes output (expected 5)\n",
                    sink._samples.size());

        expect(ge(sink._samples.size(), 5UZ))
            << "Expected >= 5 bytes, got " << sink._samples.size();
        if (sink._samples.size() >= 5) {
            std::string decoded(sink._samples.begin(), sink._samples.begin() + 5);
            expect(eq(decoded, std::string("Hello")))
                << "Decoded: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : sink._tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(pmtv::cast<bool>(it->second)) << "CRC should be valid";
                found_crc = true;
            }
        }
        expect(found_crc) << "Should have crc_valid tag";
    };

    "LoRa frame at native os_factor=4 through channelizer (decimate=1 passthrough)"_test = [] {
        // Generate frame at os_factor=4, run through channelizer with
        // decimation=1, offset=0 (pure passthrough). Should still decode.
        std::vector<uint8_t> payload = {'T', 'e', 's', 't'};
        auto iq = generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                    SYNC_WORD, PREAMBLE_LEN,
                                    true, SPS * 5, 2, BW, false);

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = iq;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = static_cast<double>(BW * OS_FACTOR);
        xlat.center_freq_offset = 0.0;
        xlat.decimation = 1;
        // No taps: pure passthrough

        auto& burst = graph.emplaceBlock<BurstDetector>();
        burst.center_freq  = CENTER_FREQ;
        burst.bandwidth    = BW;
        burst.sf           = SF;
        burst.sync_word    = SYNC_WORD;
        burst.os_factor    = OS_FACTOR;
        burst.preamble_len = PREAMBLE_LEN;

        auto& demod = graph.emplaceBlock<SymbolDemodulator>();
        demod.sf        = SF;
        demod.bandwidth = BW;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(burst);
        (void)graph.connect<"out">(burst).template to<"in">(demod);
        (void)graph.connect<"out">(demod).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::printf("Passthrough integration: %zu bytes output (expected 4)\n",
                    sink._samples.size());

        expect(ge(sink._samples.size(), 4UZ))
            << "Expected >= 4 bytes, got " << sink._samples.size();
        if (sink._samples.size() >= 4) {
            std::string decoded(sink._samples.begin(), sink._samples.begin() + 4);
            expect(eq(decoded, std::string("Test")))
                << "Decoded: \"" << decoded << "\"";
        }
    };

    "LoRa frame at -75 kHz offset decoded through channelizer"_test = [] {
        // Test with a negative offset to verify sign handling
        std::vector<uint8_t> payload = {'O', 'K'};

        auto frame_iq = generate_frame_iq(payload, SF, CR, 1,
                                          SYNC_WORD, PREAMBLE_LEN,
                                          true, N * 5, 2, BW, false);

        constexpr double wb_rate = 250000.0;
        constexpr double channel_offset = -75000.0;
        constexpr uint32_t upsample = 4;

        std::size_t wb_len = frame_iq.size() * upsample + 1024;
        std::vector<std::complex<float>> wideband(wb_len, {0.f, 0.f});

        for (std::size_t i = 0; i < frame_iq.size(); i++) {
            for (uint32_t u = 0; u < upsample; u++) {
                std::size_t idx = i * upsample + u;
                if (idx < wideband.size()) {
                    wideband[idx] = frame_iq[i];
                }
            }
        }

        // Shift to -75 kHz
        for (std::size_t n = 0; n < wideband.size(); n++) {
            double phase = 2.0 * std::numbers::pi * channel_offset
                         * static_cast<double>(n) / wb_rate;
            auto phasor = std::complex<float>(
                static_cast<float>(std::cos(phase)),
                static_cast<float>(std::sin(phase)));
            wideband[n] *= phasor;
        }

        auto taps = firdes_low_pass(65, wb_rate, 31250.0);

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(wideband.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = wideband;

        auto& xlat = graph.emplaceBlock<FreqXlatingDecimator>();
        xlat.sample_rate = wb_rate;
        xlat.center_freq_offset = channel_offset;
        xlat.decimation = upsample;
        xlat.setTaps(taps);

        auto& burst = graph.emplaceBlock<BurstDetector>();
        burst.center_freq  = CENTER_FREQ;
        burst.bandwidth    = BW;
        burst.sf           = SF;
        burst.sync_word    = SYNC_WORD;
        burst.os_factor    = 1;
        burst.preamble_len = PREAMBLE_LEN;

        auto& demod = graph.emplaceBlock<SymbolDemodulator>();
        demod.sf        = SF;
        demod.bandwidth = BW;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out">(src).template to<"in">(xlat);
        (void)graph.connect<"out">(xlat).template to<"in">(burst);
        (void)graph.connect<"out">(burst).template to<"in">(demod);
        (void)graph.connect<"out">(demod).template to<"in">(sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::printf("Negative offset integration: %zu bytes output (expected 2)\n",
                    sink._samples.size());

        expect(ge(sink._samples.size(), 2UZ))
            << "Expected >= 2 bytes, got " << sink._samples.size();
        if (sink._samples.size() >= 2) {
            std::string decoded(sink._samples.begin(), sink._samples.begin() + 2);
            expect(eq(decoded, std::string("OK")))
                << "Decoded: \"" << decoded << "\"";
        }
    };
};

int main() { /* boost::ut auto-runs all suites */ }
