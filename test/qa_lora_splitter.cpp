// SPDX-License-Identifier: ISC
/// Tests for Splitter: 1-to-N signal fan-out block.
///
/// Test progression:
///   1. N=1 passthrough: samples and tags pass through unchanged
///   2. N=2 split: both outputs receive identical samples
///   3. N=2 tags: tags are forwarded to all outputs
///   4. N=3 split: verify three-way copy

#include "test_helpers.hpp"

#include <complex>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/Splitter.hpp>

using namespace gr::lora::test;
using namespace std::string_literals;

namespace {

using cf32 = std::complex<float>;

struct SplitResult {
    std::vector<std::vector<cf32>> outputs;  // one per output port
    std::vector<std::vector<gr::Tag>> tags;  // one per output port
};

SplitResult runSplitter(
        uint32_t nOutputs,
        const std::vector<cf32>& inputSamples,
        const std::vector<gr::Tag>& inputTags = {}) {
    using namespace gr;
    using namespace gr::lora;

    Graph graph;

    auto& splitter = graph.emplaceBlock<Splitter>({
        {"n_outputs", nOutputs},
    });

    auto& src = graph.emplaceBlock<testing::TagSource<cf32,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(inputSamples.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    src.values = inputSamples;
    src._tags = inputTags;

    if (!graph.connect(src, "out"s, splitter, "in"s)) {
        throw std::runtime_error("failed to connect source -> splitter");
    }

    SplitResult result;
    result.outputs.resize(nOutputs);
    result.tags.resize(nOutputs);

    // Use raw pointers to reference sinks — they're owned by the graph
    std::vector<testing::TagSink<cf32, testing::ProcessFunction::USE_PROCESS_BULK>*> sinks;
    for (uint32_t i = 0; i < nOutputs; i++) {
        auto& sink = graph.emplaceBlock<testing::TagSink<cf32,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true},
        });
        sinks.push_back(&sink);

        auto portName = "out#"s + std::to_string(i);
        if (!graph.connect(splitter, portName, sink, "in"s)) {
            throw std::runtime_error(
                std::format("failed to connect splitter.out[{}] -> sink[{}]", i, i));
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

    for (uint32_t i = 0; i < nOutputs; i++) {
        result.outputs[i] = {sinks[i]->_samples.begin(), sinks[i]->_samples.end()};
        result.tags[i] = sinks[i]->_tags;
    }

    return result;
}

}  // namespace

const boost::ut::suite<"Splitter N=1 passthrough"> n1_tests = [] {
    using namespace boost::ut;

    "Samples pass through unchanged with N=1"_test = [] {
        std::vector<cf32> input = {{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}};
        auto result = runSplitter(1, input);

        expect(eq(result.outputs.size(), 1UZ));
        expect(eq(result.outputs[0].size(), 3UZ))
            << "Expected 3 samples, got " << result.outputs[0].size();
        if (result.outputs[0].size() >= 3) {
            expect(eq(result.outputs[0][0], cf32{1.0f, 2.0f}));
            expect(eq(result.outputs[0][1], cf32{3.0f, 4.0f}));
            expect(eq(result.outputs[0][2], cf32{5.0f, 6.0f}));
        }
    };
};

const boost::ut::suite<"Splitter N=2 copy"> n2_tests = [] {
    using namespace boost::ut;

    "Both outputs receive identical samples"_test = [] {
        std::vector<cf32> input = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}};
        auto result = runSplitter(2, input);

        expect(eq(result.outputs.size(), 2UZ));
        for (std::size_t port = 0; port < 2; port++) {
            expect(eq(result.outputs[port].size(), 3UZ))
                << "Port " << port << " expected 3 samples";
            if (result.outputs[port].size() >= 3) {
                expect(eq(result.outputs[port][0], cf32{1.0f, 0.0f}));
                expect(eq(result.outputs[port][1], cf32{0.0f, 1.0f}));
                expect(eq(result.outputs[port][2], cf32{-1.0f, 0.0f}));
            }
        }
    };

    "Tags forwarded to both outputs"_test = [] {
        std::vector<cf32> input = {{1.0f, 0.0f}, {2.0f, 0.0f}};
        std::vector<gr::Tag> tags = {
            gr::Tag{.index = 0UZ, .map = {
                {"burst_start", gr::pmt::Value(true)},
                {"sf", gr::pmt::Value(int64_t{8})},
            }},
        };
        auto result = runSplitter(2, input, tags);

        expect(eq(result.outputs.size(), 2UZ));
        for (std::size_t port = 0; port < 2; port++) {
            expect(ge(result.tags[port].size(), 1UZ))
                << "Port " << port << " should have at least 1 tag";
            if (!result.tags[port].empty()) {
                bool hasBurstStart = result.tags[port][0].map.contains("burst_start");
                expect(hasBurstStart)
                    << "Port " << port << " should have burst_start tag";
            }
        }
    };
};

const boost::ut::suite<"Splitter N=3 copy"> n3_tests = [] {
    using namespace boost::ut;

    "Three-way split produces identical outputs"_test = [] {
        std::vector<cf32> input = {{1.0f, 2.0f}, {3.0f, 4.0f}};
        auto result = runSplitter(3, input);

        expect(eq(result.outputs.size(), 3UZ));
        for (std::size_t port = 0; port < 3; port++) {
            expect(eq(result.outputs[port].size(), 2UZ))
                << "Port " << port << " expected 2 samples";
            if (result.outputs[port].size() >= 2) {
                expect(eq(result.outputs[port][0], cf32{1.0f, 2.0f}));
                expect(eq(result.outputs[port][1], cf32{3.0f, 4.0f}));
            }
        }
    };
};

int main() { /* boost::ut auto-runs all suites */ }
