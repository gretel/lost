// SPDX-License-Identifier: ISC
/// Tests for the 2-block LoRa RX architecture: BurstDetector + SymbolDemodulator.
///
/// Test progression:
///   1. SymbolDemodulator algorithm-level (bypass BurstDetector, feed aligned symbols)
///   2. BurstDetector + SymbolDemodulator graph: GR3 IQ → decoded payload
///   3. Multi-frame: two back-to-back frames
///   4. Robustness: AWGN + CFO

#include "test_helpers.hpp"

#include <cmath>
#include <numbers>
#include <random>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/BurstDetector.hpp>
#include <gnuradio-4.0/lora/SymbolDemodulator.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace gr::lora::test;

namespace {

// --- Test frame generation using shared TX chain ---

std::vector<std::complex<float>> make_frame_iq(
        const std::string& payload_str,
        uint32_t zero_pad = SPS * 5) {
    std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());
    return gr::lora::generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                       SYNC_WORD, PREAMBLE_LEN,
                                       true, zero_pad);
}

/// Add AWGN noise to IQ samples.
void add_awgn(std::vector<std::complex<float>>& iq, float snr_db) {
    double sig_power = 0;
    for (auto& s : iq) {
        sig_power += static_cast<double>(s.real()) * static_cast<double>(s.real())
                   + static_cast<double>(s.imag()) * static_cast<double>(s.imag());
    }
    sig_power /= static_cast<double>(iq.size());

    double noise_power = sig_power / std::pow(10.0, static_cast<double>(snr_db) / 10.0);
    double noise_std = std::sqrt(noise_power / 2.0);

    std::mt19937 gen(42);
    std::normal_distribution<float> dist(0.f, static_cast<float>(noise_std));

    for (auto& s : iq) {
        s += std::complex<float>(dist(gen), dist(gen));
    }
}

/// Apply CFO (frequency offset) to IQ samples.
void apply_cfo(std::vector<std::complex<float>>& iq, float cfo_hz, float sample_rate) {
    for (std::size_t n = 0; n < iq.size(); n++) {
        float phase = 2.f * static_cast<float>(std::numbers::pi) * cfo_hz
                    / sample_rate * static_cast<float>(n);
        iq[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
    }
}

/// Helper: run a full BurstDetector + SymbolDemodulator graph on given IQ data.
/// Returns {decoded_bytes, tags}.
struct DecodeResult {
    std::vector<uint8_t> samples;
    std::vector<gr::Tag> tags;
};

DecodeResult run_decode(const std::vector<std::complex<float>>& iq) {
    using namespace gr;
    using namespace gr::lora;

    Graph graph;

    auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false},
        {"mark_tag", false}
    });
    src.values = iq;

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

    (void)graph.connect<"out">(src).template to<"in">(burst);
    (void)graph.connect<"out">(burst).template to<"in">(demod);
    (void)graph.connect<"out">(demod).template to<"in">(sink);

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
    }
    sched.runAndWait();

    return {sink._samples, sink._tags};
}

} // namespace

// ============================================================================
// Test 1: SymbolDemodulator algorithm-level
//   Feed aligned, downsampled symbols directly (bypassing BurstDetector).
//   Uses the GR3 test vector IQ but only the payload symbols (after preamble).
// ============================================================================

const boost::ut::suite<"SymbolDemodulator algorithm-level"> symdemod_algo_tests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::lora;

    "SymbolDemodulator decodes Hello MeshCore from aligned symbols"_test = [] {
        auto iq = load_cf32("tx_07_iq_frame.cf32");
        auto expected_payload = load_text("payload.txt");

        // Extract aligned, downsampled payload symbols from the test IQ.
        // Preamble: 8 upchirps + 2 sync + 2.25 downchirps = 12.25 symbols
        std::size_t payload_start = static_cast<std::size_t>(12.25 * SPS);

        // Total payload symbols (excluding zero padding at end)
        std::size_t n_payload_syms = (iq.size() - payload_start - SPS * 5) / SPS;

        // Build aligned symbol blocks (N samples each, downsampled from os_factor)
        // NOTE: use offset 0, NOT OS_FACTOR/2. The chirp phase formula means
        // that samples at indices 0, OS, 2*OS, ... match the os_factor=1 chirp
        // used by SymbolDemodulator's dechirp reference. Offset OS/2 causes
        // the FFT peak to fall between bins (scalloping), giving ±1 errors.
        std::vector<std::complex<float>> aligned_symbols;
        for (std::size_t s = 0; s < n_payload_syms; s++) {
            std::size_t sym_start = payload_start + s * SPS;
            for (uint32_t i = 0; i < N; i++) {
                std::size_t idx = sym_start + i * OS_FACTOR;
                if (idx < iq.size()) {
                    aligned_symbols.push_back(iq[idx]);
                } else {
                    aligned_symbols.push_back({0.f, 0.f});
                }
            }
        }

        Graph graph;
        auto& source = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(aligned_symbols.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        source.values = aligned_symbols;
        source._tags = {
            Tag{0UZ, {{"burst_start", pmtv::pmt(true)},
                      {"sf", pmtv::pmt(static_cast<int64_t>(SF))},
                      {"cfo_int", pmtv::pmt(static_cast<int64_t>(0))},
                      {"cfo_frac", pmtv::pmt(0.0)}}}
        };

        auto& demod = graph.emplaceBlock<SymbolDemodulator>();
        demod.sf = SF;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        expect(eq(graph.connect<"out">(source).template to<"in">(demod), ConnectionResult::SUCCESS));
        expect(eq(graph.connect<"out">(demod).template to<"in">(sink), ConnectionResult::SUCCESS));

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        auto& output = sink._samples;
        std::printf("SymbolDemodulator algo: %zu bytes output (expected %zu)\n",
                    output.size(), expected_payload.size());

        expect(ge(output.size(), expected_payload.size()))
            << "Output too small: " << output.size()
            << " expected >= " << expected_payload.size();

        if (output.size() >= expected_payload.size()) {
            std::string decoded(output.begin(),
                                output.begin() + static_cast<std::ptrdiff_t>(expected_payload.size()));
            expect(eq(decoded, expected_payload))
                << "Decoded: \"" << decoded << "\" expected: \"" << expected_payload << "\"";
        }

        // Verify CRC valid tag
        bool found_crc_tag = false;
        for (const auto& t : sink._tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(pmtv::cast<bool>(it->second)) << "CRC should be valid";
                found_crc_tag = true;
            }
        }
        expect(found_crc_tag) << "Should have crc_valid tag";
    };
};

// ============================================================================
// Test 2: Full 2-block graph: BurstDetector → SymbolDemodulator
//   GR3 test vector IQ → decoded "Hello MeshCore"
// ============================================================================

const boost::ut::suite<"Full 2-block RX graph"> full_graph_tests = [] {
    using namespace boost::ut;

    "BurstDetector + SymbolDemodulator decodes Hello MeshCore"_test = [] {
        auto iq = load_cf32("tx_07_iq_frame.cf32");
        auto expected_payload = load_text("payload.txt");

        auto result = run_decode(iq);

        std::printf("Full 2-block: %zu bytes output (expected %zu)\n",
                    result.samples.size(), expected_payload.size());

        expect(ge(result.samples.size(), expected_payload.size()))
            << "Output size: " << result.samples.size()
            << " expected >= " << expected_payload.size();

        if (result.samples.size() >= expected_payload.size()) {
            std::string decoded(result.samples.begin(),
                                result.samples.begin()
                                + static_cast<std::ptrdiff_t>(expected_payload.size()));
            expect(eq(decoded, expected_payload))
                << "Decoded: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(pmtv::cast<bool>(it->second)) << "CRC should be valid";
                found_crc = true;
            }
        }
        expect(found_crc) << "Should have crc_valid tag";
    };
};

// ============================================================================
// Test 3: Multi-frame — two back-to-back frames
// ============================================================================

const boost::ut::suite<"Multi-frame decoding"> multi_frame_tests = [] {
    using namespace boost::ut;

    "Two back-to-back frames decoded correctly"_test = [] {
        auto iq1 = make_frame_iq("LoRa PHY test!", SPS * 3);
        auto iq2 = make_frame_iq("Second Frame!!", SPS * 5);

        std::vector<std::complex<float>> iq;
        iq.insert(iq.end(), iq1.begin(), iq1.end());
        iq.insert(iq.end(), iq2.begin(), iq2.end());

        auto result = run_decode(iq);

        std::printf("Multi-frame: %zu bytes output (expected 28 = 14 + 14)\n",
                    result.samples.size());

        // Count decoded frames from tags
        int frame_count = 0;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("pay_len"); it != t.map.end()) {
                frame_count++;
            }
        }

        std::printf("  Decoded frames: %d (expected 2)\n", frame_count);
        expect(ge(frame_count, 2)) << "Expected 2 decoded frames";

        expect(ge(result.samples.size(), 28UZ))
            << "Expected 28 bytes total (14 + 14)";

        if (result.samples.size() >= 14) {
            std::string frame1(result.samples.begin(), result.samples.begin() + 14);
            std::printf("  Frame 1: \"%s\"\n", frame1.c_str());
            expect(eq(frame1, std::string("LoRa PHY test!")))
                << "Frame 1";
        }
        if (result.samples.size() >= 28) {
            std::string frame2(result.samples.begin() + 14, result.samples.begin() + 28);
            std::printf("  Frame 2: \"%s\"\n", frame2.c_str());
            expect(eq(frame2, std::string("Second Frame!!")))
                << "Frame 2";
        }
    };
};

// ============================================================================
// Test 4: Robustness — AWGN + CFO
// ============================================================================

const boost::ut::suite<"Robustness tests"> robustness_tests = [] {
    using namespace boost::ut;

    "Decodes at +10 dB SNR"_test = [] {
        auto iq = make_frame_iq("LoRa PHY test!");
        add_awgn(iq, 10.0f);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size at 10dB: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "10dB decode: \"" << decoded << "\"";
        }
    };

    "Decodes at 0 dB SNR"_test = [] {
        auto iq = make_frame_iq("LoRa PHY test!");
        add_awgn(iq, 0.0f);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size at 0dB: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "0dB decode: \"" << decoded << "\"";
        }
    };

    "Decodes with +500 Hz CFO"_test = [] {
        float sample_rate = static_cast<float>(BW * OS_FACTOR);  // 250 kHz
        auto iq = make_frame_iq("LoRa PHY test!");
        apply_cfo(iq, 500.f, sample_rate);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size with +500Hz CFO: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "+500Hz decode: \"" << decoded << "\"";
        }
    };

    "Decodes with -500 Hz CFO"_test = [] {
        float sample_rate = static_cast<float>(BW * OS_FACTOR);
        auto iq = make_frame_iq("LoRa PHY test!");
        apply_cfo(iq, -500.f, sample_rate);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size with -500Hz CFO: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "-500Hz decode: \"" << decoded << "\"";
        }
    };

    "Decodes with +1000 Hz CFO"_test = [] {
        float sample_rate = static_cast<float>(BW * OS_FACTOR);
        auto iq = make_frame_iq("LoRa PHY test!");
        apply_cfo(iq, 1000.f, sample_rate);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size with +1000Hz CFO: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "+1000Hz decode: \"" << decoded << "\"";
        }
    };

    "Decodes with -1000 Hz CFO"_test = [] {
        float sample_rate = static_cast<float>(BW * OS_FACTOR);
        auto iq = make_frame_iq("LoRa PHY test!");
        apply_cfo(iq, -1000.f, sample_rate);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 14UZ))
            << "Output size with -1000Hz CFO: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "-1000Hz decode: \"" << decoded << "\"";
        }
    };
};

// ============================================================================
// Test 5: Downchirp (inverted IQ) frame handling
//
// Design: downchirp detection uses input conjugation (conj(downchirp) = upchirp).
// The RX conjugates the IQ stream, making downchirp frames appear as normal
// upchirp frames to BurstDetector + SymbolDemodulator. This is the standard
// SDR approach (two parallel chains: normal + conjugated).
//
// These tests verify: (a) inverted_iq TX produces conjugated output,
// (b) conjugating the input before the RX chain decodes downchirp frames.
// ============================================================================

const boost::ut::suite<"Downchirp detection"> downchirp_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "Downchirp frame decoded via input conjugation"_test = [] {
        std::vector<uint8_t> payload = {'L', 'o', 'R', 'a', ' ', 'P',
                                        'H', 'Y', ' ', 't', 'e', 's', 't', '!'};
        auto iq = generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                    SYNC_WORD, PREAMBLE_LEN,
                                    true, SPS * 5,
                                    2,       // ldro_mode auto
                                    BW,
                                    true);   // inverted_iq

        // Conjugate the IQ to convert downchirp frame → upchirp frame
        for (auto& s : iq) s = std::conj(s);

        auto result = run_decode(iq);

        std::printf("Downchirp (conj): %zu bytes output (expected 14)\n",
                    result.samples.size());

        expect(ge(result.samples.size(), 14UZ))
            << "Downchirp output size: " << result.samples.size();
        if (result.samples.size() >= 14) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 14);
            expect(eq(decoded, std::string("LoRa PHY test!")))
                << "Downchirp decode: \"" << decoded << "\"";
        }

        // Verify CRC valid
        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(pmtv::cast<bool>(it->second)) << "CRC should be valid";
                found_crc = true;
            }
        }
        expect(found_crc) << "Should have crc_valid tag";
    };

    "Downchirp frame with CFO decoded via input conjugation"_test = [] {
        float sample_rate = static_cast<float>(BW * OS_FACTOR);
        std::vector<uint8_t> payload = {'T', 'e', 's', 't', '!'};
        auto iq = generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                    SYNC_WORD, PREAMBLE_LEN,
                                    true, SPS * 5,
                                    2, BW, true);  // inverted_iq
        apply_cfo(iq, 500.f, sample_rate);

        // Conjugate to convert downchirp → upchirp
        for (auto& s : iq) s = std::conj(s);

        auto result = run_decode(iq);

        expect(ge(result.samples.size(), 5UZ))
            << "Downchirp+CFO output size: " << result.samples.size();
        if (result.samples.size() >= 5) {
            std::string decoded(result.samples.begin(), result.samples.begin() + 5);
            expect(eq(decoded, std::string("Test!")))
                << "Downchirp+CFO decode: \"" << decoded << "\"";
        }
    };

    "Inverted IQ is exact conjugate of normal IQ"_test = [] {
        // Verify the mathematical identity: inverted_iq output = conj(normal output)
        std::vector<uint8_t> payload = {'H', 'i'};
        auto iq_normal   = generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                             SYNC_WORD, PREAMBLE_LEN,
                                             true, 0, 2, BW, false);
        auto iq_inverted = generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                             SYNC_WORD, PREAMBLE_LEN,
                                             true, 0, 2, BW, true);

        expect(eq(iq_normal.size(), iq_inverted.size()));

        float max_err = 0.f;
        for (std::size_t i = 0; i < iq_normal.size(); i++) {
            auto conj_normal = std::conj(iq_normal[i]);
            float err = std::max(std::abs(conj_normal.real() - iq_inverted[i].real()),
                                 std::abs(conj_normal.imag() - iq_inverted[i].imag()));
            if (err > max_err) max_err = err;
        }
        expect(lt(max_err, 1e-5f))
            << "conj(normal) should equal inverted: max err " << max_err;
    };
};

// ============================================================================
// Test 6: Error paths — wrong sync word, CRC failure, low SNR
// ============================================================================

const boost::ut::suite<"Error path tests"> error_path_tests = [] {
    using namespace boost::ut;

    "Wrong sync word is rejected"_test = [] {
        // Generate frame with sync_word=0x34, but decoder expects 0x12
        std::vector<uint8_t> payload = {'L', 'o', 'R', 'a', ' ', 'P',
                                        'H', 'Y', ' ', 't', 'e', 's', 't', '!'};
        auto iq = gr::lora::generate_frame_iq(payload, SF, CR, OS_FACTOR,
                                              0x34,  // wrong sync word
                                              PREAMBLE_LEN, true, SPS * 5);

        auto result = run_decode(iq);  // decoder uses SYNC_WORD=0x12

        // BurstDetector should reject: no decoded output
        expect(eq(result.samples.size(), 0UZ))
            << "Wrong sync word should produce 0 bytes, got " << result.samples.size();

        // No crc_valid tag should be present
        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (t.map.contains("crc_valid")) found_crc = true;
        }
        expect(!found_crc) << "Should have no crc_valid tag after sync word rejection";
    };

    "CRC failure is flagged (corrupted payload IQ)"_test = [] {
        auto iq = make_frame_iq("LoRa PHY test!");

        // Corrupt a payload symbol by zeroing out one symbol's worth of IQ
        // in the middle of the payload. The preamble is ~12.25 symbols.
        std::size_t payload_start = static_cast<std::size_t>(12.25 * SPS);
        std::size_t corrupt_sym = 5;  // corrupt the 6th payload symbol
        std::size_t corrupt_start = payload_start + corrupt_sym * SPS;
        for (std::size_t i = corrupt_start; i < corrupt_start + SPS && i < iq.size(); i++) {
            iq[i] = std::complex<float>(0.f, 0.f);
        }

        auto result = run_decode(iq);

        // SymbolDemodulator should still output bytes, but crc_valid=false
        // (it always outputs payload on header success, regardless of CRC)
        if (!result.samples.empty()) {
            bool found_crc = false;
            bool crc_valid = true;
            for (const auto& t : result.tags) {
                if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                    crc_valid = pmtv::cast<bool>(it->second);
                    found_crc = true;
                }
            }
            // If we got output, CRC should be flagged invalid
            if (found_crc) {
                expect(!crc_valid) << "CRC should be invalid for corrupted frame";
            }
        }
        // Either no output (detection failed) or output with crc_valid=false.
        // Both are acceptable — the key is no false positive.
    };

    "Low SNR (-15 dB) does not produce false positive"_test = [] {
        auto iq = make_frame_iq("LoRa PHY test!");
        add_awgn(iq, -15.0f);

        auto result = run_decode(iq);

        // At -15 dB, preamble detection may fail entirely (0 bytes)
        // or decoder may produce garbage with CRC invalid.
        // The key assertion: no valid CRC output.
        bool false_positive = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                if (pmtv::cast<bool>(it->second)) {
                    false_positive = true;
                }
            }
        }
        expect(!false_positive)
            << "Should not produce crc_valid=true at -15 dB SNR";
    };
};

int main() { /* boost::ut auto-runs all suites */ }
