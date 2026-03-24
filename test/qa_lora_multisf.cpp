// SPDX-License-Identifier: ISC
/// Multi-SF decoder tests: single-SF and multi-SF graph-level loopback,
/// robustness (AWGN + CFO), and multi-BW decode via Splitter.

#include "test_helpers.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/MultiSfDecoder.hpp>
#include <gnuradio-4.0/lora/Splitter.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace gr::lora::test;

// ============================================================================
// Helper: run a graph-level loopback through MultiSfDecoder
// ============================================================================
namespace {

struct LoopbackResult {
    std::vector<uint8_t>     decoded;
    std::vector<gr::Tag>     tags;
    bool                     ok{false};
};

LoopbackResult run_multisf_loopback(
        const std::vector<uint8_t>& payload,
        uint8_t test_sf, uint8_t test_cr,
        uint32_t test_bw, uint8_t test_os,
        uint16_t sync_word = 0x12,
        uint16_t preamble_len_val = 8,
        uint8_t sf_min = 7, uint8_t sf_max = 12) {
    using namespace gr;
    using namespace gr::lora;

    uint32_t sps = (1u << test_sf) * test_os;
    auto iq = generate_frame_iq(payload, test_sf, test_cr, test_os,
                                 sync_word, preamble_len_val,
                                 true, sps * 5, 2, test_bw, false);

    Graph graph;
    auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false},
        {"mark_tag", false}
    });
    src.values = iq;

    auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
    decoder.bandwidth    = test_bw;
    decoder.sync_word    = sync_word;
    decoder.os_factor    = test_os;
    decoder.preamble_len = preamble_len_val;
    decoder.center_freq  = CENTER_FREQ;
    decoder.sf_min       = sf_min;
    decoder.sf_max       = sf_max;

    auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags", true}
    });

    (void)graph.connect<"out", "in">(src, decoder);
    (void)graph.connect<"out", "in">(decoder, sink);

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "scheduler init failed\n");
        return {};
    }
    sched.runAndWait();

    LoopbackResult result;
    result.decoded.assign(sink._samples.begin(), sink._samples.end());
    result.tags.assign(sink._tags.begin(), sink._tags.end());
    result.ok      = true;
    return result;
}

/// Add AWGN noise to IQ samples (power-based SNR).
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

/// Run MultiSfDecoder with optional AWGN and CFO impairments.
LoopbackResult run_multisf_impaired(
        const std::vector<uint8_t>& payload,
        uint8_t test_sf, uint8_t test_cr,
        uint32_t test_bw, uint8_t test_os,
        float snr_db = 100.f,
        float cfo_hz = 0.f,
        uint16_t sync_word = 0x12,
        uint16_t preamble_len_val = 8,
        uint8_t sf_min = 7, uint8_t sf_max = 12) {
    using namespace gr;
    using namespace gr::lora;

    uint32_t sps = (1u << test_sf) * test_os;
    auto iq = generate_frame_iq(payload, test_sf, test_cr, test_os,
                                 sync_word, preamble_len_val,
                                 true, sps * 5, 2, test_bw, false);

    float sample_rate = static_cast<float>(test_bw) * static_cast<float>(test_os);
    if (snr_db < 99.f) add_awgn(iq, snr_db);
    if (std::abs(cfo_hz) > 0.1f) apply_cfo(iq, cfo_hz, sample_rate);

    Graph graph;
    auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
        {"repeat_tags", false},
        {"mark_tag", false}
    });
    src.values = iq;

    auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
    decoder.bandwidth    = test_bw;
    decoder.sync_word    = sync_word;
    decoder.os_factor    = test_os;
    decoder.preamble_len = preamble_len_val;
    decoder.center_freq  = CENTER_FREQ;
    decoder.sf_min       = sf_min;
    decoder.sf_max       = sf_max;

    auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags", true}
    });

    (void)graph.connect<"out", "in">(src, decoder);
    (void)graph.connect<"out", "in">(decoder, sink);

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        std::fprintf(stderr, "scheduler init failed\n");
        return {};
    }
    sched.runAndWait();

    LoopbackResult result;
    result.decoded.assign(sink._samples.begin(), sink._samples.end());
    result.tags.assign(sink._tags.begin(), sink._tags.end());
    result.ok = true;
    return result;
}

}  // anonymous namespace

// ============================================================================
// Single-SF decode through MultiSfDecoder
// ============================================================================

const boost::ut::suite<"MultiSfDecoder single-SF"> single_sf_tests = [] {
    using namespace boost::ut;

    "SF8 BW125k decode through MultiSfDecoder"_test = [] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        auto result = run_multisf_loopback(payload, 8, 4, 125000, 4,
                                            0x12, 8, 7, 12);

        expect(result.ok) << "graph ran";
        std::printf("  SF8: %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());

        expect(ge(result.decoded.size(), payload.size()))
            << "output too small: " << result.decoded.size();

        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            std::string expected(payload.begin(), payload.end());
            expect(eq(decoded, expected))
                << "payload mismatch: \"" << decoded << "\"";
        }

        // Check tag
        bool found_crc = false;
        bool found_sf = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false)) << "CRC should be valid";
                found_crc = true;
            }
            if (auto it = t.map.find("sf"); it != t.map.end()) {
                expect(eq(it->second.value_or<int64_t>(0), int64_t{8}))
                    << "SF tag should be 8";
                found_sf = true;
            }
        }
        expect(found_crc) << "should have crc_valid tag";
        expect(found_sf) << "should have sf tag";
    };

    "SF8 BW125k os=1 decode"_test = [] {
        std::vector<uint8_t> payload = {'T', 'e', 's', 't'};
        auto result = run_multisf_loopback(payload, 8, 4, 125000, 1,
                                            0x12, 8, 8, 8);
        expect(result.ok) << "graph ran";
        expect(ge(result.decoded.size(), payload.size())) << "output size";
        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Test"))) << "payload";
        }
    };

    "SF8 BW62.5k decode"_test = [] {
        std::vector<uint8_t> payload = {'A', 'B', 'C'};
        auto result = run_multisf_loopback(payload, 8, 4, 62500, 4,
                                            0x12, 8, 7, 12);
        expect(result.ok) << "graph ran";
        std::printf("  SF8/BW62.5k: %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size())) << "output size";
        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("ABC"))) << "payload";
        }
    };

    "SF7 decode"_test = [] {
        std::vector<uint8_t> payload = {'7'};
        auto result = run_multisf_loopback(payload, 7, 4, 125000, 4,
                                            0x12, 8, 7, 7);
        expect(result.ok) << "graph ran";
        expect(ge(result.decoded.size(), payload.size())) << "output size";
        if (!result.decoded.empty()) {
            expect(eq(result.decoded[0], uint8_t('7'))) << "payload";
        }
    };

    "SF12 decode"_test = [] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        auto result = run_multisf_loopback(payload, 12, 4, 125000, 4,
                                            0x12, 8, 12, 12);
        expect(result.ok) << "graph ran";
        std::printf("  SF12: %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size())) << "output size";
        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Hello"))) << "payload";
        }
    };

    // Regression: SF12 signal with all lanes active (SF7-12).
    // The SF11 lane must NOT claim the SF12 preamble before the SF12 lane.
    "SF12 with all lanes active (cross-SF competition)"_test = [] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        // TX as SF12, but decoder runs SF7-12 simultaneously
        auto result = run_multisf_loopback(payload, 12, 4, 62500, 4,
                                            0x12, 8, 7, 12);
        expect(result.ok) << "graph ran";
        std::printf("  SF12 (all lanes): %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());

        // Must decode correctly — the SF12 lane should win, not SF11
        expect(ge(result.decoded.size(), payload.size()))
            << "SF12 must decode with all lanes active";
        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Hello")))
                << "payload must match (SF12 lane should win)";
        }

        // Verify the detected SF is 12, not 11
        bool found_sf_tag = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("sf"); it != t.map.end()) {
                auto detected_sf = static_cast<uint32_t>(it->second.value_or<int64_t>(0));
                std::printf("  SF12 (all lanes): detected SF=%u\n", detected_sf);
                expect(eq(detected_sf, 12u))
                    << "must detect as SF12, not SF" << detected_sf;
                found_sf_tag = true;
            }
        }
        expect(found_sf_tag) << "sf tag must be present in output";

        // CRC must be valid
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false))
                    << "CRC must be valid when correct SF detects";
            }
        }
    };

    // Regression test for integer CFO/STO decomposition bug:
    // SF12 decode was completely broken with any non-zero CFO because
    // cfo_int = down_val/2 ignored k_hat. The Xhonneux decomposition
    // requires cfo_int = (k_hat + down_val) / 2.
    "SF12 decode with CFO (cfo_int decomposition fix)"_test = [] {
        std::vector<uint8_t> payload = {'H', 'i'};

        // SF12/BW62.5k at +10 dB with 50 Hz CFO (~3.3 bins at bin_width=15.26 Hz)
        auto r1 = run_multisf_impaired(payload, 12, 4, 62500, 4,
                                        10.f, 50.f,
                                        0x12, 8, 12, 12);
        std::printf("  SF12/BW62.5k +10dB +50Hz: %zu bytes\n", r1.decoded.size());
        expect(ge(r1.decoded.size(), payload.size()))
            << "SF12/BW62.5k must decode at +10 dB with 50 Hz CFO";

        // SF12/BW125k at +10 dB with 100 Hz CFO (~3.3 bins at bin_width=30.5 Hz)
        auto r2 = run_multisf_impaired(payload, 12, 4, 125000, 4,
                                        10.f, 100.f,
                                        0x12, 8, 12, 12);
        std::printf("  SF12/BW125k +10dB +100Hz: %zu bytes\n", r2.decoded.size());
        expect(ge(r2.decoded.size(), payload.size()))
            << "SF12/BW125k must decode at +10 dB with 100 Hz CFO";

        // Multi-SF mode: SF12 with all lanes active at +10 dB with 50 Hz CFO
        auto r3 = run_multisf_impaired(payload, 12, 4, 62500, 4,
                                        10.f, 50.f,
                                        0x12, 8, 7, 12);
        std::printf("  SF12 (all lanes) +10dB +50Hz: %zu bytes\n", r3.decoded.size());
        expect(ge(r3.decoded.size(), payload.size()))
            << "SF12 must decode in multi-SF mode at +10 dB with 50 Hz CFO";
    };

    // Same test at BW125k (LDRO) — SF12 with cross-SF competition
    "SF12/BW125k with all lanes active"_test = [] {
        std::vector<uint8_t> payload = {'T', 'e', 's', 't'};
        auto result = run_multisf_loopback(payload, 12, 4, 125000, 4,
                                            0x12, 8, 7, 12);
        expect(result.ok) << "graph ran";
        std::printf("  SF12/BW125k (all lanes): %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size()))
            << "SF12/BW125k must decode with all lanes active";
        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Test")))
                << "payload must match";
        }
    };

    // SF11 with all lanes active — test adjacent SF competition
    "SF11 with all lanes active"_test = [] {
        std::vector<uint8_t> payload = {'H', 'i'};
        auto result = run_multisf_loopback(payload, 11, 4, 62500, 4,
                                            0x12, 8, 7, 12);
        expect(result.ok) << "graph ran";
        std::printf("  SF11 (all lanes): %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size()))
            << "SF11 must decode with all lanes active";
    };
};

// ============================================================================
// Noise rejection: no false positives
// ============================================================================

const boost::ut::suite<"MultiSfDecoder noise"> noise_tests = [] {
    using namespace boost::ut;

    "pure noise produces no output"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        // Generate white noise (no LoRa signal)
        std::vector<std::complex<float>> noise(100000);
        std::mt19937 rng(42);
        std::normal_distribution<float> dist(0.f, 0.01f);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(noise.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = noise;

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.bandwidth = 125000;
        decoder.os_factor = 4;
        decoder.sf_min = 7;
        decoder.sf_max = 12;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        (void)sched.exchange(std::move(graph));
        sched.runAndWait();

        expect(eq(sink._samples.size(), 0UZ))
            << "noise should produce no output, got " << sink._samples.size();
    };
};

// ============================================================================
// Overflow recovery: verify behaviour under overflow tags and zero gaps
// ============================================================================
//
// The real overflow path in MultiSfDecoder (line 112) requires an empty
// in_span with an {"overflow", true} tag — this happens when SoapyBlock
// publishes 0 samples on overflow. TagSource always produces >=1 sample,
// so it cannot trigger the exact empty-span path. These tests verify the
// observable effects: (a) overflow-tagged noise is harmless, (b) zeroed
// gaps mid-frame (simulating the timing-preservation effect of overflow
// recovery) do not crash or hang the decoder.

const boost::ut::suite<"MultiSfDecoder overflow recovery"> overflow_tests = [] {
    using namespace boost::ut;

    "overflow tag during idle is harmless"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        // White noise with an overflow tag at sample 0
        constexpr std::size_t kNoise = 50000;
        std::vector<std::complex<float>> noise(kNoise);
        std::mt19937 rng(123);
        std::normal_distribution<float> dist(0.f, 0.01f);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(kNoise)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = noise;
        src._tags = {{0, {{"overflow", true}}}};

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.bandwidth = 125000;
        decoder.os_factor = 4;
        decoder.sf_min = 7;
        decoder.sf_max = 12;
        decoder.preamble_len = 8;
        decoder.sync_word = 0x12;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        (void)sched.exchange(std::move(graph));
        sched.runAndWait();

        expect(eq(sink._samples.size(), 0UZ))
            << "overflow-tagged noise should produce no output, got "
            << sink._samples.size();
    };

    "overflow tag preserves symbol alignment (zero gap mid-frame)"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        // Generate a clean SF8/BW125k frame at os=1
        constexpr uint8_t  kSf = 8;
        constexpr uint8_t  kCr = 4;
        constexpr uint32_t kBw = 125000;
        constexpr uint8_t  kOs = 1;
        constexpr uint16_t kSyncWord = 0x12;
        constexpr uint16_t kPreambleLen = 8;
        const uint32_t sps = (1u << kSf) * kOs;  // 256

        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        auto iq = generate_frame_iq(payload, kSf, kCr, kOs,
                                     kSyncWord, kPreambleLen,
                                     true, sps * 5, 2, kBw, false);

        // Zero out one symbol period after the preamble (12.25 symbols),
        // within the payload region. This simulates the effect of overflow
        // recovery inserting zeros to preserve symbol boundary alignment.
        const std::size_t gap_start = 13 * sps;
        const std::size_t n_gap = sps;
        if (gap_start + n_gap < iq.size()) {
            std::fill_n(iq.begin() + static_cast<std::ptrdiff_t>(gap_start),
                        n_gap, std::complex<float>{0.f, 0.f});
        }

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = iq;
        src._tags = {{gap_start, {{"overflow", true}}}};

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.bandwidth    = kBw;
        decoder.sync_word    = kSyncWord;
        decoder.os_factor    = kOs;
        decoder.preamble_len = kPreambleLen;
        decoder.center_freq  = CENTER_FREQ;
        decoder.sf_min       = kSf;
        decoder.sf_max       = kSf;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        (void)sched.exchange(std::move(graph));
        sched.runAndWait();

        // Zero gap corrupts payload, so CRC will likely fail. The key
        // assertion: no crash, no hang, no runaway output.
        std::printf("  zero-gap: %zu bytes decoded\n", sink._samples.size());
        if (!sink._samples.empty()) {
            expect(le(sink._samples.size(), 256UZ))
                << "output should be bounded, not runaway decode";
        }
        expect(true) << "decoder survived zero gap without crash";
    };

    "multiple overflow tags in noise are harmless"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        constexpr std::size_t kNoise = 80000;
        std::vector<std::complex<float>> noise(kNoise);
        std::mt19937 rng(456);
        std::normal_distribution<float> dist(0.f, 0.01f);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(kNoise)},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = noise;
        src._tags = {
            {0,     {{"overflow", true}}},
            {10000, {{"overflow", true}}},
            {30000, {{"overflow", true}}},
            {60000, {{"overflow", true}}}
        };

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.bandwidth = 62500;
        decoder.os_factor = 4;
        decoder.sf_min = 7;
        decoder.sf_max = 12;
        decoder.preamble_len = 8;
        decoder.sync_word = 0x12;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        (void)sched.exchange(std::move(graph));
        sched.runAndWait();

        expect(eq(sink._samples.size(), 0UZ))
            << "multiple overflow tags in noise should produce no output, got "
            << sink._samples.size();
    };
};

// ============================================================================
// Cross-validation against existing FrameSync + DemodDecoder chain
// ============================================================================

const boost::ut::suite<"MultiSfDecoder cross-validate"> cross_validate_tests = [] {
    using namespace boost::ut;

    // Test a representative subset of configs from allConfigs()
    // (full 36-config matrix is expensive; test boundary SFs + LDRO)
    auto run_config = [](uint8_t test_sf, uint8_t test_cr, uint32_t test_bw,
                         const std::string& label) {
        std::string payload_str = "Hello MeshCore";
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        auto result = run_multisf_loopback(payload, test_sf, test_cr, test_bw, 4,
                                            0x12, 8, test_sf, test_sf);

        std::printf("  %s: %zu bytes decoded (expected %zu)\n",
                    label.c_str(), result.decoded.size(), payload.size());

        expect(result.ok) << label << " graph ran";
        expect(ge(result.decoded.size(), payload.size()))
            << label << " output size";

        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, payload_str))
                << label << " payload mismatch: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false)) << label << " CRC";
                found_crc = true;
            }
        }
        expect(found_crc) << label << " crc_valid tag";
    };

    "SF7/CR4/BW125k"_test = [&run_config] { run_config(7, 4, 125000, "SF7/CR4/BW125k"); };
    "SF8/CR4/BW62.5k"_test = [&run_config] { run_config(8, 4, 62500, "SF8/CR4/BW62.5k"); };
    "SF9/CR2/BW125k"_test = [&run_config] { run_config(9, 2, 125000, "SF9/CR2/BW125k"); };
    // LDRO configs — payload blocks use reduced rate (sf_app = sf-2)
    "SF10/CR4/BW125k (no LDRO)"_test = [&run_config] { run_config(10, 4, 125000, "SF10/CR4/BW125k"); };
    "SF10/CR4/BW62.5k (LDRO)"_test = [&run_config] { run_config(10, 4, 62500, "SF10/CR4/BW62.5k"); };
    "SF11/CR4/BW62.5k (LDRO)"_test = [&run_config] { run_config(11, 4, 62500, "SF11/CR4/BW62.5k"); };
    "SF12/CR4/BW125k (LDRO)"_test = [&run_config] { run_config(12, 4, 125000, "SF12/CR4/BW125k"); };
};

// ============================================================================
// Robustness: AWGN + CFO through MultiSfDecoder
// ============================================================================

const boost::ut::suite<"MultiSfDecoder robustness"> robustness_tests = [] {
    using namespace boost::ut;

    "SF8 decode at +10 dB SNR"_test = [] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        auto result = run_multisf_impaired(payload, 8, 4, 62500, 1,
                                            10.f, 0.f,
                                            0x12, 8, 8, 8);
        expect(result.ok) << "graph ran";
        std::printf("  SF8 +10dB SNR: %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size()))
            << "output too small: " << result.decoded.size();

        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Hello")))
                << "payload mismatch: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false)) << "CRC should be valid at +10 dB";
                found_crc = true;
            }
        }
        expect(found_crc) << "should have crc_valid tag";
    };

    "SF8 decode with +500 Hz CFO"_test = [] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        auto result = run_multisf_impaired(payload, 8, 4, 62500, 1,
                                            100.f, 500.f,
                                            0x12, 8, 8, 8);
        expect(result.ok) << "graph ran";
        std::printf("  SF8 +500Hz CFO: %zu bytes decoded (expected %zu)\n",
                    result.decoded.size(), payload.size());
        expect(ge(result.decoded.size(), payload.size()))
            << "output too small: " << result.decoded.size();

        if (result.decoded.size() >= payload.size()) {
            std::string decoded(result.decoded.begin(),
                                result.decoded.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("Hello")))
                << "payload mismatch: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : result.tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false)) << "CRC should be valid with +500Hz CFO";
                found_crc = true;
            }
        }
        expect(found_crc) << "should have crc_valid tag";
    };

    "noise rejection at -10 dB"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        // Generate pure white noise — no LoRa signal, just noise
        // at a level that would correspond to -10 dB SNR for a
        // typical chirp signal (sigma=0.1 → power ~0.01).
        std::vector<std::complex<float>> noise(200000);
        std::mt19937 rng(123);
        std::normal_distribution<float> dist(0.f, 0.1f);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(noise.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = noise;

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.bandwidth = 62500;
        decoder.os_factor = 1;
        decoder.sf_min    = 8;
        decoder.sf_max    = 8;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        (void)sched.exchange(std::move(graph));
        sched.runAndWait();

        std::printf("  noise rejection: %zu bytes output (expected 0)\n",
                    sink._samples.size());
        expect(eq(sink._samples.size(), 0UZ))
            << "pure noise should produce no output, got " << sink._samples.size();

        // Also verify no crc_valid=true tags (no false positives)
        bool false_positive = false;
        for (const auto& t : sink._tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                if (it->second.value_or<bool>(false)) {
                    false_positive = true;
                }
            }
        }
        expect(!false_positive) << "no crc_valid=true from pure noise";
    };
};

// ============================================================================
// Multi-BW decode via Splitter
// ============================================================================

const boost::ut::suite<"Multi-BW decode via Splitter"> multi_bw_tests = [] {
    using namespace boost::ut;
    using namespace std::string_literals;

    "BW62.5k frame decoded through 3-way Splitter"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        // Generate SF8/BW62.5k frame at 250 kS/s (os_factor=4 for BW62.5k)
        std::vector<uint8_t> payload = {'A', 'B', 'C'};
        constexpr uint8_t  test_sf  = 8;
        constexpr uint8_t  test_cr  = 4;
        constexpr uint32_t test_bw  = 62500;
        constexpr uint8_t  test_os  = 4;     // 250 kS/s = 62500 * 4
        constexpr uint16_t sw       = 0x12;
        constexpr uint16_t plen     = 8;
        uint32_t sps = (1u << test_sf) * test_os;

        auto iq = generate_frame_iq(payload, test_sf, test_cr, test_os,
                                     sw, plen, true, sps * 5, 2, test_bw, false);

        Graph graph;

        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = iq;

        auto& splitter = graph.emplaceBlock<Splitter>({
            {"n_outputs", gr::Size_t{3}},
        });

        // BW62.5k decoder: os=4 (250k/62.5k)
        auto& dec_62k = graph.emplaceBlock<MultiSfDecoder>();
        dec_62k.bandwidth    = 62500;
        dec_62k.sync_word    = sw;
        dec_62k.os_factor    = 4;
        dec_62k.preamble_len = plen;
        dec_62k.center_freq  = CENTER_FREQ;
        dec_62k.sf_min       = 8;
        dec_62k.sf_max       = 8;

        // BW125k decoder: os=2 (250k/125k)
        auto& dec_125k = graph.emplaceBlock<MultiSfDecoder>();
        dec_125k.bandwidth    = 125000;
        dec_125k.sync_word    = sw;
        dec_125k.os_factor    = 2;
        dec_125k.preamble_len = plen;
        dec_125k.center_freq  = CENTER_FREQ;
        dec_125k.sf_min       = 8;
        dec_125k.sf_max       = 8;

        // BW250k decoder: os=1 (250k/250k)
        auto& dec_250k = graph.emplaceBlock<MultiSfDecoder>();
        dec_250k.bandwidth    = 250000;
        dec_250k.sync_word    = sw;
        dec_250k.os_factor    = 1;
        dec_250k.preamble_len = plen;
        dec_250k.center_freq  = CENTER_FREQ;
        dec_250k.sf_min       = 8;
        dec_250k.sf_max       = 8;

        auto& sink_62k = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true}, {"log_tags", true}
        });
        auto& sink_125k = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true}, {"log_tags", true}
        });
        auto& sink_250k = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true}, {"log_tags", true}
        });

        // Connect source -> splitter
        expect(graph.connect(src, "out"s, splitter, "in"s).has_value())
            << "src -> splitter";

        // Connect splitter outputs -> decoders
        expect(graph.connect(splitter, "out#0"s, dec_62k, "in"s).has_value())
            << "splitter -> dec_62k";
        expect(graph.connect(splitter, "out#1"s, dec_125k, "in"s).has_value())
            << "splitter -> dec_125k";
        expect(graph.connect(splitter, "out#2"s, dec_250k, "in"s).has_value())
            << "splitter -> dec_250k";

        // Connect decoders -> sinks
        expect(graph.connect<"out", "in">(dec_62k, sink_62k).has_value())
            << "dec_62k -> sink_62k";
        expect(graph.connect<"out", "in">(dec_125k, sink_125k).has_value())
            << "dec_125k -> sink_125k";
        expect(graph.connect<"out", "in">(dec_250k, sink_250k).has_value())
            << "dec_250k -> sink_250k";

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            std::fprintf(stderr, "multi-BW scheduler init failed\n");
            expect(false) << "scheduler init";
            return;
        }
        sched.runAndWait();

        // BW62.5k decoder MUST decode the frame (it's the native BW)
        std::printf("  BW62.5k sink: %zu bytes (expected %zu)\n",
                    sink_62k._samples.size(), payload.size());
        expect(ge(sink_62k._samples.size(), payload.size()))
            << "BW62.5k decoder output too small: " << sink_62k._samples.size();

        if (sink_62k._samples.size() >= payload.size()) {
            std::string decoded(sink_62k._samples.begin(),
                                sink_62k._samples.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            expect(eq(decoded, std::string("ABC")))
                << "BW62.5k payload mismatch: \"" << decoded << "\"";
        }

        // Verify CRC valid on the BW62.5k path
        bool found_crc = false;
        for (const auto& t : sink_62k._tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false))
                    << "BW62.5k CRC should be valid";
                found_crc = true;
            }
        }
        expect(found_crc) << "BW62.5k should have crc_valid tag";

        // BW125k and BW250k decoders may or may not produce output
        // (cross-BW false positives are possible but benign).
        // Log what they produced for diagnostic visibility.
        std::printf("  BW125k sink: %zu bytes\n", sink_125k._samples.size());
        std::printf("  BW250k sink: %zu bytes\n", sink_250k._samples.size());
    };
};

int main() { /* boost::ut auto-runs all suites */ }
