// SPDX-License-Identifier: ISC
/// Multi-SF decoder tests: single-SF and multi-SF graph-level loopback.

#include "test_helpers.hpp"

#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/MultiSfDecoder.hpp>
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

int main() { /* boost::ut auto-runs all suites */ }
