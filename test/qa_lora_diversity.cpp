// SPDX-License-Identifier: ISC
/// Tests for DiversityCombiner: N-input diversity combining for LoRa RX.
///
/// Test progression:
///   1. Single-input passthrough (N=1): frame passes through immediately
///   2. Dual-input same frame: selects CRC_OK over CRC_FAIL
///   3. Dual-input same frame: selects higher SNR when both CRC_OK
///   4. Timeout: emits best candidate when deadline expires
///   5. Diversity metadata tags: verify diversity_* tag fields
///   6. Two distinct frames: no cross-matching
///   7. Vector tag (Tensor) round-trip verification
///   8. pmt::Value vector-to-Tensor conversion proof
///   9. Both-CRC_FAIL SNR tiebreak
///  10. Empty payload (pay_len=0) edge case
///  11. Interrupted frame accumulation (new tag mid-collection)
///  12. Three-input combining (N=3)

#include "test_helpers.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/DiversityCombiner.hpp>

using namespace gr::lora::test;
using namespace std::string_literals;

namespace {

struct FakeFrame {
    std::vector<uint8_t> payload;
    gr::Tag              tag;
};

FakeFrame makeFrame(const std::string& text, bool crc_valid, double snr_db,
                    int64_t rx_channel) {
    std::vector<uint8_t> payload(text.begin(), text.end());
    gr::property_map map = {
        {"pay_len", gr::pmt::Value(static_cast<int64_t>(payload.size()))},
        {"crc_valid", gr::pmt::Value(crc_valid)},
        {"snr_db", gr::pmt::Value(snr_db)},
        {"rx_channel", gr::pmt::Value(rx_channel)},
    };
    return {.payload = std::move(payload),
            .tag = gr::Tag{.index = 0UZ, .map = std::move(map)}};
}

struct CombineResult {
    std::vector<uint8_t> samples;
    std::vector<gr::Tag> tags;
};

CombineResult runCombiner(
        uint32_t nInputs,
        const std::vector<std::vector<FakeFrame>>& channelFrames,
        uint32_t timeout_symbols = 30,
        uint32_t bandwidth = BW,
        uint8_t testSf = SF) {
    using namespace gr;
    using namespace gr::lora;

    Graph graph;

    auto& combiner = graph.emplaceBlock<DiversityCombiner>({
        {"n_inputs", nInputs},
        {"timeout_symbols", timeout_symbols},
        {"bandwidth", bandwidth},
        {"sf", testSf},
        {"debug", true},
        {"disconnect_on_done", false},
    });

    auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags", true},
        {"disconnect_on_done", false},
    });

    if (graph.connect(combiner, "out"s, sink, "in"s) != ConnectionResult::SUCCESS) {
        throw std::runtime_error("failed to connect combiner.out -> sink.in");
    }

    for (uint32_t ch = 0; ch < nInputs; ch++) {
        std::vector<uint8_t> bytes;
        std::vector<gr::Tag> tags;
        if (ch < channelFrames.size()) {
            for (const auto& frame : channelFrames[ch]) {
                gr::Tag frameTag = frame.tag;
                frameTag.index = bytes.size();
                tags.push_back(std::move(frameTag));
                bytes.insert(bytes.end(), frame.payload.begin(),
                             frame.payload.end());
            }
        }

        auto& src = graph.emplaceBlock<testing::TagSource<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(
                bytes.empty() ? 1U : static_cast<uint32_t>(bytes.size()))},
            {"repeat_tags", false},
            {"mark_tag", false},
            {"disconnect_on_done", false},
        });
        if (!bytes.empty()) {
            src.values = std::move(bytes);
        } else {
            src.values = {0};
        }
        src._tags = std::move(tags);

        auto portName = "in#"s + std::to_string(ch);
        if (graph.connect(src, "out"s, combiner, portName) != ConnectionResult::SUCCESS) {
            throw std::runtime_error(
                std::format("failed to connect source[{}] -> combiner.in[{}]", ch, ch));
        }
    }

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(
            std::format("failed to init scheduler: {}", ret.error()));
    }
    if (!sched.runAndWait().has_value()) {
        throw std::runtime_error("scheduler runAndWait failed");
    }

    return {.samples = {sink._samples.begin(), sink._samples.end()},
            .tags = sink._tags};
}

}  // namespace

const boost::ut::suite<"DiversityCombiner N=1 passthrough"> n1_tests = [] {
    using namespace boost::ut;

    "Single frame passes through with N=1"_test = [] {
        auto frame = makeFrame("Hello", true, 5.0, 0);
        auto result = runCombiner(1, {{frame}});

        expect(ge(result.samples.size(), 5UZ))
            << "Expected 5 bytes, got " << result.samples.size();
        if (result.samples.size() >= 5) {
            std::string decoded(result.samples.begin(),
                                result.samples.begin() + 5);
            expect(eq(decoded, std::string("Hello")));
        }
    };

    "Two sequential frames pass through with N=1"_test = [] {
        auto frame1 = makeFrame("First!", true, 3.0, 0);
        auto frame2 = makeFrame("Second", true, 4.0, 0);
        auto result = runCombiner(1, {{frame1, frame2}});

        expect(ge(result.samples.size(), 12UZ))
            << "Expected 12 bytes, got " << result.samples.size();
        if (result.samples.size() >= 12) {
            std::string dec1(result.samples.begin(), result.samples.begin() + 6);
            std::string dec2(result.samples.begin() + 6,
                             result.samples.begin() + 12);
            expect(eq(dec1, std::string("First!")));
            expect(eq(dec2, std::string("Second")));
        }
    };
};

const boost::ut::suite<"DiversityCombiner CRC selection"> crc_tests = [] {
    using namespace boost::ut;

    "CRC_OK selected over CRC_FAIL (ch0=FAIL, ch1=OK)"_test = [] {
        auto frameCh0 = makeFrame("Hello", false, 8.0, 0);
        auto frameCh1 = makeFrame("Hello", true,  3.0, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        expect(ge(result.samples.size(), 5UZ))
            << "Expected 5 bytes, got " << result.samples.size();
        if (result.samples.size() >= 5) {
            std::string decoded(result.samples.begin(),
                                result.samples.begin() + 5);
            expect(eq(decoded, std::string("Hello")));
        }

        bool found = false;
        for (const auto& frameTag : result.tags) {
            if (auto iter = frameTag.map.find("crc_valid"); iter != frameTag.map.end()) {
                expect(iter->second.value_or<bool>(false))
                    << "Winner should have CRC_OK";
                found = true;
            }
        }
        expect(found) << "Should have crc_valid tag";
    };
};

const boost::ut::suite<"DiversityCombiner SNR selection"> snr_tests = [] {
    using namespace boost::ut;

    "Higher SNR selected when both CRC_OK"_test = [] {
        auto frameCh0 = makeFrame("Hello", true, 2.0, 0);
        auto frameCh1 = makeFrame("Hello", true, 7.5, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        expect(ge(result.samples.size(), 5UZ));

        bool found = false;
        for (const auto& frameTag : result.tags) {
            if (auto iter = frameTag.map.find("diversity_decoded_channel");
                iter != frameTag.map.end()) {
                auto channel = iter->second.value_or<int64_t>(-1);
                expect(eq(channel, int64_t{1}))
                    << "Should select ch=1 (SNR 7.5 > 2.0)";
                found = true;
            }
        }
        expect(found) << "Should have diversity_decoded_channel tag";
    };
};

const boost::ut::suite<"DiversityCombiner timeout"> timeout_tests = [] {
    using namespace boost::ut;

    "Emits when only one channel decodes"_test = [] {
        // ch0 decodes "Solo!"; ch1 sends noise bytes with no pay_len tag.
        // The combiner should emit ch0's frame (unmatched by ch1).
        auto frameCh0 = makeFrame("Solo!", true, 4.0, 0);
        // ch1: noise bytes with no pay_len tag — combiner ignores them
        FakeFrame noiseCh1 = {
            .payload = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE},
            .tag = gr::Tag{.index = 0UZ, .map = {
                {"noise", gr::pmt::Value(true)},  // arbitrary, no pay_len
            }},
        };
        auto result = runCombiner(2, {{frameCh0}, {noiseCh1}});

        expect(ge(result.samples.size(), 5UZ))
            << "Expected 5 bytes on timeout, got " << result.samples.size();
        if (result.samples.size() >= 5) {
            std::string decoded(result.samples.begin(),
                                result.samples.begin() + 5);
            expect(eq(decoded, std::string("Solo!")));
        }
    };
};

const boost::ut::suite<"DiversityCombiner metadata"> metadata_tests = [] {
    using namespace boost::ut;

    "Diversity tags present with N=2"_test = [] {
        auto frameCh0 = makeFrame("Test!", false, 2.0, 0);
        auto frameCh1 = makeFrame("Test!", true,  5.0, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        expect(ge(result.samples.size(), 5UZ));

        bool foundN = false, foundCh = false, foundMask = false, foundGap = false;
        for (const auto& frameTag : result.tags) {
            if (auto iter = frameTag.map.find("diversity_n_candidates");
                iter != frameTag.map.end()) {
                expect(eq(iter->second.value_or<int64_t>(0), int64_t{2}));
                foundN = true;
            }
            if (auto iter = frameTag.map.find("diversity_decoded_channel");
                iter != frameTag.map.end()) {
                expect(eq(iter->second.value_or<int64_t>(-1), int64_t{1}));
                foundCh = true;
            }
            if (auto iter = frameTag.map.find("diversity_crc_mask");
                iter != frameTag.map.end()) {
                auto mask = iter->second.value_or<int64_t>(0);
                // ch0=FAIL(0), ch1=OK(1) -> mask bit1=1 -> mask=0b10=2
                expect(eq(mask, int64_t{2}))
                    << "CRC mask should be 2 (only ch1 OK)";
                foundMask = true;
            }
            if (auto iter = frameTag.map.find("diversity_gap_us");
                iter != frameTag.map.end()) {
                auto gap = iter->second.value_or<int64_t>(-1);
                expect(ge(gap, int64_t{0}))
                    << "diversity_gap_us should be non-negative";
                foundGap = true;
            }
        }
        expect(foundN) << "Should have diversity_n_candidates";
        expect(foundCh) << "Should have diversity_decoded_channel";
        expect(foundMask) << "Should have diversity_crc_mask";
        expect(foundGap) << "Should have diversity_gap_us";
    };
};

const boost::ut::suite<"DiversityCombiner distinct frames"> distinct_tests = [] {
    using namespace boost::ut;

    "Two distinct payloads emit as separate frames"_test = [] {
        auto frameCh0 = makeFrame("AAAA", true, 3.0, 0);
        auto frameCh1 = makeFrame("BBBB", true, 4.0, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        // Both frames should be emitted separately (total 8 bytes)
        expect(ge(result.samples.size(), 8UZ))
            << "Expected 8 bytes (two 4-byte frames), got "
            << result.samples.size();
    };
};

const boost::ut::suite<"DiversityCombiner vector tags"> vector_tag_tests = [] {
    using namespace boost::ut;

    "diversity_rx_channels and diversity_snr_db Tensor tags verified"_test = [] {
        auto frameCh0 = makeFrame("Hello", true, 3.0, 0);
        auto frameCh1 = makeFrame("Hello", true, 8.0, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        expect(ge(result.samples.size(), 5UZ));

        bool foundChannels = false;
        bool foundSnr = false;
        for (const auto& frameTag : result.tags) {
            // diversity_rx_channels: stored as Tensor<int64_t> (vector->Tensor conversion)
            if (auto iter = frameTag.map.find("diversity_rx_channels");
                iter != frameTag.map.end()) {
                auto* tensor = iter->second.get_if<gr::Tensor<int64_t>>();
                expect(neq(tensor, static_cast<const gr::Tensor<int64_t>*>(nullptr)))
                    << "diversity_rx_channels should be Tensor<int64_t>";
                if (tensor) {
                    expect(eq(tensor->size(), 2UZ)) << "should have 2 entries";
                    if (tensor->size() >= 2) {
                        expect(eq((*tensor)[0], int64_t{0})) << "first channel = 0";
                        expect(eq((*tensor)[1], int64_t{1})) << "second channel = 1";
                    }
                }
                foundChannels = true;
            }
            // diversity_snr_db: stored as Tensor<double>
            if (auto iter = frameTag.map.find("diversity_snr_db");
                iter != frameTag.map.end()) {
                auto* tensor = iter->second.get_if<gr::Tensor<double>>();
                expect(neq(tensor, static_cast<const gr::Tensor<double>*>(nullptr)))
                    << "diversity_snr_db should be Tensor<double>";
                if (tensor) {
                    expect(eq(tensor->size(), 2UZ)) << "should have 2 entries";
                    if (tensor->size() >= 2) {
                        expect(eq((*tensor)[0], 3.0)) << "ch0 SNR = 3.0";
                        expect(eq((*tensor)[1], 8.0)) << "ch1 SNR = 8.0";
                    }
                }
                foundSnr = true;
            }
        }
        expect(foundChannels) << "Should have diversity_rx_channels tag";
        expect(foundSnr) << "Should have diversity_snr_db tag";
    };
};

const boost::ut::suite<"DiversityCombiner pmt round-trip"> pmt_tests = [] {
    using namespace boost::ut;

    "std::vector<int64_t> stored in pmt::Value round-trips via Tensor"_test = [] {
        std::vector<int64_t> original = {10, 20, 30};
        gr::pmt::Value val(original);

        // get_if<std::vector<T>> is excluded by requires constraint;
        // must use get_if<Tensor<T>> instead
        auto* tensor = val.get_if<gr::Tensor<int64_t>>();
        expect(neq(tensor, static_cast<gr::Tensor<int64_t>*>(nullptr)))
            << "vector<int64_t> should be retrievable as Tensor<int64_t>";
        if (tensor) {
            expect(eq(tensor->size(), 3UZ));
            if (tensor->size() >= 3) {
                expect(eq((*tensor)[0], int64_t{10}));
                expect(eq((*tensor)[1], int64_t{20}));
                expect(eq((*tensor)[2], int64_t{30}));
            }
        }
    };

    "std::vector<double> stored in pmt::Value round-trips via Tensor"_test = [] {
        std::vector<double> original = {1.5, 2.5, -3.5};
        gr::pmt::Value val(original);

        auto* tensor = val.get_if<gr::Tensor<double>>();
        expect(neq(tensor, static_cast<gr::Tensor<double>*>(nullptr)))
            << "vector<double> should be retrievable as Tensor<double>";
        if (tensor) {
            expect(eq(tensor->size(), 3UZ));
            if (tensor->size() >= 3) {
                expect(eq((*tensor)[0], 1.5));
                expect(eq((*tensor)[1], 2.5));
                expect(eq((*tensor)[2], -3.5));
            }
        }
    };
};

const boost::ut::suite<"DiversityCombiner both-FAIL tiebreak"> fail_tests = [] {
    using namespace boost::ut;

    "Higher SNR selected when both CRC_FAIL"_test = [] {
        auto frameCh0 = makeFrame("Hello", false, 2.0, 0);
        auto frameCh1 = makeFrame("Hello", false, 7.5, 1);
        auto result = runCombiner(2, {{frameCh0}, {frameCh1}});

        expect(ge(result.samples.size(), 5UZ));

        bool found = false;
        for (const auto& frameTag : result.tags) {
            if (auto iter = frameTag.map.find("diversity_decoded_channel");
                iter != frameTag.map.end()) {
                auto channel = iter->second.value_or<int64_t>(-1);
                expect(eq(channel, int64_t{1}))
                    << "Should select ch=1 (SNR 7.5 > 2.0, both FAIL)";
                found = true;
            }
            // Verify CRC mask: both FAIL -> mask = 0
            if (auto iter = frameTag.map.find("diversity_crc_mask");
                iter != frameTag.map.end()) {
                auto mask = iter->second.value_or<int64_t>(-1);
                expect(eq(mask, int64_t{0}))
                    << "CRC mask should be 0 (both FAIL)";
            }
        }
        expect(found) << "Should have diversity_decoded_channel tag";
    };
};

const boost::ut::suite<"DiversityCombiner empty payload"> empty_tests = [] {
    using namespace boost::ut;

    "Empty payload (pay_len=0) passes through with N=1"_test = [] {
        // pay_len=0: FNV hash is the offset basis value
        FakeFrame emptyFrame = {
            .payload = {},
            .tag = gr::Tag{.index = 0UZ, .map = {
                {"pay_len", gr::pmt::Value(int64_t{0})},
                {"crc_valid", gr::pmt::Value(true)},
                {"snr_db", gr::pmt::Value(5.0)},
                {"rx_channel", gr::pmt::Value(int64_t{0})},
            }},
        };

        // ch0: empty frame followed by a normal frame to verify the empty
        // one didn't corrupt state
        auto normalFrame = makeFrame("OK", true, 4.0, 0);
        auto result = runCombiner(1, {{emptyFrame, normalFrame}});

        // Should get the normal frame's 2 bytes at minimum
        expect(ge(result.samples.size(), 2UZ))
            << "Expected at least 2 bytes from normal frame";
        // Verify we got a tag for the empty frame (it still emits a tag)
        expect(ge(result.tags.size(), 1UZ))
            << "Should have at least 1 tag";
    };

    "Two empty payloads on different channels group by hash"_test = [] {
        // Both empty payloads hash to the same FNV offset basis.
        // With 0 output bytes, the GR4 framework discards tags
        // (tags are not published when processedOut == 0, Block.hpp ~1829).
        // Verify the combiner does not crash and produces no output bytes.
        FakeFrame emptyA = {
            .payload = {},
            .tag = gr::Tag{.index = 0UZ, .map = {
                {"pay_len", gr::pmt::Value(int64_t{0})},
                {"crc_valid", gr::pmt::Value(true)},
                {"snr_db", gr::pmt::Value(3.0)},
                {"rx_channel", gr::pmt::Value(int64_t{0})},
            }},
        };
        FakeFrame emptyB = {
            .payload = {},
            .tag = gr::Tag{.index = 0UZ, .map = {
                {"pay_len", gr::pmt::Value(int64_t{0})},
                {"crc_valid", gr::pmt::Value(true)},
                {"snr_db", gr::pmt::Value(6.0)},
                {"rx_channel", gr::pmt::Value(int64_t{1})},
            }},
        };
        auto result = runCombiner(2, {{emptyA}, {emptyB}});

        // No output bytes expected (both payloads are empty).
        // Tags are dropped by the framework since processedOut == 0.
        expect(eq(result.samples.size(), 0UZ))
            << "Empty payloads should produce 0 output bytes";
    };
};

const boost::ut::suite<"DiversityCombiner interrupted accumulation"> interrupted_tests = [] {
    using namespace boost::ut;

    "New pay_len tag mid-collection submits partial frame"_test = [] {
        // Craft a byte stream where a second pay_len tag arrives before
        // the first frame's bytes are fully accumulated.
        // Byte stream: [A, B, C, D, E, F] with:
        //   tag at offset 0: pay_len=4 (expects 4 bytes: A,B,C,D)
        //   tag at offset 2: pay_len=2 (interrupts after A,B; starts new frame C,D)
        // The combiner should submit partial frame {A,B} from the first tag,
        // then complete frame {C,D} from the second tag.
        using namespace gr;
        using namespace gr::lora;

        Graph graph;

        auto& combiner = graph.emplaceBlock<DiversityCombiner>({
            {"n_inputs", gr::Size_t{1}},
            {"timeout_symbols", uint32_t{30}},
            {"bandwidth", BW},
            {"sf", SF},
            {"debug", true},
            {"disconnect_on_done", false},
        });

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true},
            {"disconnect_on_done", false},
        });

        expect(eq(graph.connect(combiner, "out"s, sink, "in"s),
                  ConnectionResult::SUCCESS));

        // Build source with two tags in the same byte stream
        std::vector<uint8_t> bytes = {'A', 'B', 'C', 'D', 'E', 'F'};
        std::vector<gr::Tag> tags = {
            gr::Tag{.index = 0UZ, .map = {
                {"pay_len", pmt::Value(int64_t{4})},
                {"crc_valid", pmt::Value(true)},
                {"snr_db", pmt::Value(3.0)},
                {"rx_channel", pmt::Value(int64_t{0})},
            }},
            gr::Tag{.index = 2UZ, .map = {
                {"pay_len", pmt::Value(int64_t{2})},
                {"crc_valid", pmt::Value(true)},
                {"snr_db", pmt::Value(5.0)},
                {"rx_channel", pmt::Value(int64_t{0})},
            }},
        };

        auto& src = graph.emplaceBlock<testing::TagSource<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(bytes.size())},
            {"repeat_tags", false},
            {"mark_tag", false},
            {"disconnect_on_done", false},
        });
        src.values = std::move(bytes);
        src._tags = std::move(tags);

        expect(eq(graph.connect(src, "out"s, combiner, "in#0"s),
                  ConnectionResult::SUCCESS));

        scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        // Should have at least 2 tags (one per submitted frame)
        expect(ge(sink._tags.size(), 2UZ))
            << "Expected 2 tags (partial + complete), got " << sink._tags.size();

        // The partial frame {A,B} and complete frame {C,D} should both be emitted.
        // Total output bytes should be at least 4 (2 + 2).
        expect(ge(sink._samples.size(), 4UZ))
            << "Expected at least 4 bytes, got " << sink._samples.size();
    };
};

const boost::ut::suite<"DiversityCombiner N=3"> n3_tests = [] {
    using namespace boost::ut;

    "Three-input combining selects best among 3 channels"_test = [] {
        auto frameCh0 = makeFrame("Hello", false, 5.0, 0);
        auto frameCh1 = makeFrame("Hello", true,  2.0, 1);
        auto frameCh2 = makeFrame("Hello", true,  9.0, 2);
        auto result = runCombiner(3, {{frameCh0}, {frameCh1}, {frameCh2}});

        expect(ge(result.samples.size(), 5UZ));

        bool foundCh = false;
        bool foundN = false;
        for (const auto& frameTag : result.tags) {
            if (auto iter = frameTag.map.find("diversity_decoded_channel");
                iter != frameTag.map.end()) {
                auto channel = iter->second.value_or<int64_t>(-1);
                expect(eq(channel, int64_t{2}))
                    << "Should select ch=2 (CRC_OK + highest SNR 9.0)";
                foundCh = true;
            }
            if (auto iter = frameTag.map.find("diversity_n_candidates");
                iter != frameTag.map.end()) {
                expect(eq(iter->second.value_or<int64_t>(0), int64_t{3}))
                    << "Should have 3 candidates";
                foundN = true;
            }
            if (auto iter = frameTag.map.find("diversity_crc_mask");
                iter != frameTag.map.end()) {
                auto mask = iter->second.value_or<int64_t>(0);
                // ch0=FAIL, ch1=OK, ch2=OK -> bits 1,2 set -> 0b110 = 6
                expect(eq(mask, int64_t{6}))
                    << "CRC mask should be 6 (ch1+ch2 OK)";
            }
        }
        expect(foundCh) << "Should have diversity_decoded_channel tag";
        expect(foundN) << "Should have diversity_n_candidates tag";
    };
};

int main() { /* boost::ut auto-runs all suites */ }
