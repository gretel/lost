// SPDX-License-Identifier: ISC
/// End-to-end: upstream sample_rate / frequency / ppm_error tags placed on
/// a TagSource flow through MultiSfDecoder's per-chunk tag cache. Verified
/// via the public lastTimingTagsForTest() accessor. No frame decode is
/// required — only that the cache is populated from mergedInputTag().

#include <cmath>
#include <complex>
#include <cstdint>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/MultiSfDecoder.hpp>

using cf32 = std::complex<float>;

const boost::ut::suite<"MultiSfDecoder tag forwarding"> tag_forwarding_tests = [] {
    using namespace boost::ut;
    using namespace gr;

    "caches sample_rate, frequency and ppm_error from upstream tag"_test = [] {
        Graph graph;

        // SoapySource emits these as float / double / float (SoapySource.hpp:503-507).
        // MultiSfDecoder reads each via property_map::value_or<T>, so the types
        // flowing through the test must match.
        auto& source = graph.emplaceBlock<testing::TagSource<cf32, testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<Size_t>(8192)},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        source._tags = {Tag{0, property_map{
                                   {"sample_rate", 250000.f},
                                   {"frequency", 869.618e6},
                                   {"ppm_error", 1.2f},
                               }}};

        // sf_set = {7} → single lane, _min_sps = 2^7 * os_factor = 512,
        // well below the 8192-sample source so processBulk progresses and the
        // cache-populating branch runs on the first invocation.
        auto& decoder = graph.emplaceBlock<lora::MultiSfDecoder>({
            {"bandwidth", uint32_t{125000}},
            {"sf_set", std::vector<uint8_t>{7}},
        });

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t, testing::ProcessFunction::USE_PROCESS_BULK>>();

        expect(graph.connect<"out", "in">(source, decoder).has_value());
        expect(graph.connect<"out", "in">(decoder, sink).has_value());

        scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        const auto cache = decoder.lastTimingTagsForTest();
        expect(std::abs(cache.sample_rate - 250000.f) < 1e-3f) << "sample_rate was " << cache.sample_rate;
        expect(std::abs(cache.frequency - 869.618e6) < 1.0) << "frequency was " << cache.frequency;
        expect(std::abs(cache.ppm_error - 1.2f) < 1e-6f) << "ppm_error was " << cache.ppm_error;
    };
};

int main() { /* boost::ut auto-runs all suites */ }
