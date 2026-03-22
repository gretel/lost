// SPDX-License-Identifier: ISC
/// Tests for TxQueueSource: thread-safe burst IQ source for persistent TX graph.
///
/// Test progression:
///   1. Empty queue: push nothing, verify zero samples output
///   2. Single burst: push 100 samples, verify out0 matches input, out1 is zeros
///   3. Multiple bursts: push two bursts (50 + 30), verify 80 total in order
///   4. Thread safety: push from std::thread while scheduler drains, no data loss

#include <algorithm>
#include <atomic>
#include <complex>
#include <cstdint>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/TxQueueSource.hpp>

using namespace std::string_literals;
using namespace std::chrono_literals;

namespace {

using cf32 = std::complex<float>;

/// Watchdog: requests scheduler stop after `timeout` if not already stopped.
/// Returns (thread, timedOut flag). Caller must join the thread.
auto createWatchdog(auto& sched, std::chrono::milliseconds timeout = 2000ms) {
    auto timedOut = std::make_shared<std::atomic_bool>(false);
    std::thread t([&sched, timedOut, timeout]() {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (sched.state() == gr::lifecycle::State::STOPPED) {
                return;
            }
            std::this_thread::sleep_for(20ms);
        }
        timedOut->store(true, std::memory_order_relaxed);
        sched.requestStop();
    });
    return std::make_pair(std::move(t), timedOut);
}

/// Generate a test burst of `n` cf32 samples with sequential values.
/// sample[i] = cf32(offset + i, -(offset + i))
std::vector<cf32> makeBurst(std::size_t n, float offset = 0.0f) {
    std::vector<cf32> burst(n);
    for (std::size_t i = 0; i < n; i++) {
        burst[i] = cf32{offset + static_cast<float>(i),
                        -(offset + static_cast<float>(i))};
    }
    return burst;
}

/// Run TxQueueSource in a graph with two TagSinks, using a watchdog for timeout.
/// `setupFn` is called with (TxQueueSource&, scheduler&) before the scheduler
/// starts, allowing the test to push bursts or spawn producer threads.
/// `postStartFn` is called after sched.start() with (TxQueueSource&, scheduler&)
/// for tests that need to push after the scheduler is running.
struct DrainResult {
    std::vector<cf32> out0_samples;
    std::vector<cf32> out1_samples;
    bool timed_out{false};
};

template <typename SetupFn, typename PostStartFn>
DrainResult runTxQueue(SetupFn setupFn, PostStartFn postStartFn,
                       std::chrono::milliseconds timeout = 2000ms) {
    using namespace gr;

    Graph graph;

    auto& src = graph.emplaceBlock<gr::lora::TxQueueSource>();

    auto& sink0 = graph.emplaceBlock<testing::TagSink<cf32,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags", false},
    });
    auto& sink1 = graph.emplaceBlock<testing::TagSink<cf32,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags", false},
    });

    if (!graph.connect(src, "out0"s, sink0, "in"s)) {
        throw std::runtime_error("failed to connect out0 -> sink0");
    }
    if (!graph.connect(src, "out1"s, sink1, "in"s)) {
        throw std::runtime_error("failed to connect out1 -> sink1");
    }

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(
            std::format("scheduler exchange failed: {}", ret.error()));
    }

    // Let test set up pre-start state (push bursts before scheduler runs).
    setupFn(src, sched);

    // Watchdog stops the scheduler after timeout (TxQueueSource never returns DONE).
    auto [watchdog, timedOut] = createWatchdog(sched, timeout);

    // runAndWait() blocks until all blocks reach DONE or STOPPED.
    // TxQueueSource only returns DONE on REQUESTED_STOP, so the watchdog
    // or the test's own requestStop() call is needed to unblock.
    std::thread schedThread([&sched]() {
        std::ignore = sched.runAndWait();
    });

    // Let test do post-start work (push from threads, etc.).
    postStartFn(src, sched);

    schedThread.join();
    watchdog.join();

    DrainResult result;
    result.out0_samples.assign(sink0._samples.begin(), sink0._samples.end());
    result.out1_samples.assign(sink1._samples.begin(), sink1._samples.end());
    result.timed_out = timedOut->load();
    return result;
}

/// Convenience overload: setup only, no post-start action.
template <typename SetupFn>
DrainResult runTxQueue(SetupFn setupFn,
                       std::chrono::milliseconds timeout = 2000ms) {
    return runTxQueue(
        setupFn,
        [](gr::lora::TxQueueSource&, auto&) {},  // no post-start action
        timeout);
}

}  // namespace

// ---------------------------------------------------------------------------
const boost::ut::suite<"TxQueueSource empty queue"> empty_tests = [] {
    using namespace boost::ut;

    "empty queue produces zero samples"_test = [] {
        // Push nothing, let the watchdog stop the scheduler after a short timeout.
        auto result = runTxQueue(
            [](gr::lora::TxQueueSource&, auto& sched) {
                // No push — queue stays empty.
                // Request stop immediately so we don't wait for the full watchdog.
                std::thread([&sched]() {
                    std::this_thread::sleep_for(200ms);
                    sched.requestStop();
                }).detach();
            },
            500ms);

        expect(eq(result.out0_samples.size(), 0UZ))
            << "out0 should have 0 samples, got " << result.out0_samples.size();
        expect(eq(result.out1_samples.size(), 0UZ))
            << "out1 should have 0 samples, got " << result.out1_samples.size();
    };
};

// ---------------------------------------------------------------------------
const boost::ut::suite<"TxQueueSource single burst"> single_burst_tests = [] {
    using namespace boost::ut;

    "push and drain single burst"_test = [] {
        constexpr std::size_t kBurstLen = 100;

        auto burst = makeBurst(kBurstLen);

        auto result = runTxQueue(
            [&burst](gr::lora::TxQueueSource& src, auto& sched) {
                src.push(burst);
                src.notifyProgress();
                // Give the scheduler time to drain, then stop.
                std::thread([&sched]() {
                    std::this_thread::sleep_for(500ms);
                    sched.requestStop();
                }).detach();
            },
            2000ms);

        // Verify out0 has exactly the burst samples.
        expect(eq(result.out0_samples.size(), kBurstLen))
            << "out0 expected " << kBurstLen << " samples, got "
            << result.out0_samples.size();

        if (result.out0_samples.size() == kBurstLen) {
            for (std::size_t i = 0; i < kBurstLen; i++) {
                expect(eq(result.out0_samples[i], burst[i]))
                    << "out0 mismatch at sample " << i;
            }
        }

        // Verify out1 has matching zeros.
        expect(eq(result.out1_samples.size(), kBurstLen))
            << "out1 expected " << kBurstLen << " samples, got "
            << result.out1_samples.size();

        if (result.out1_samples.size() == kBurstLen) {
            bool allZero = std::all_of(result.out1_samples.begin(),
                                       result.out1_samples.end(),
                                       [](cf32 v) { return v == cf32{}; });
            expect(allZero) << "out1 should be all zeros";
        }
    };
};

// ---------------------------------------------------------------------------
const boost::ut::suite<"TxQueueSource multiple bursts"> multi_burst_tests = [] {
    using namespace boost::ut;

    "multiple bursts drain sequentially"_test = [] {
        constexpr std::size_t kBurst1Len = 50;
        constexpr std::size_t kBurst2Len = 30;

        auto burst1 = makeBurst(kBurst1Len, 0.0f);
        auto burst2 = makeBurst(kBurst2Len, 1000.0f);  // offset to distinguish

        auto result = runTxQueue(
            [&](gr::lora::TxQueueSource& src, auto& sched) {
                src.push(burst1);
                src.push(burst2);
                src.notifyProgress();
                std::thread([&sched]() {
                    std::this_thread::sleep_for(500ms);
                    sched.requestStop();
                }).detach();
            },
            2000ms);

        const std::size_t totalExpected = kBurst1Len + kBurst2Len;
        expect(eq(result.out0_samples.size(), totalExpected))
            << "out0 expected " << totalExpected << " samples, got "
            << result.out0_samples.size();

        if (result.out0_samples.size() == totalExpected) {
            // First burst: samples 0..49 should match burst1.
            for (std::size_t i = 0; i < kBurst1Len; i++) {
                expect(eq(result.out0_samples[i], burst1[i]))
                    << "burst1 mismatch at sample " << i;
            }
            // Second burst: samples 50..79 should match burst2.
            for (std::size_t i = 0; i < kBurst2Len; i++) {
                expect(eq(result.out0_samples[kBurst1Len + i], burst2[i]))
                    << "burst2 mismatch at sample " << i;
            }
        }

        // out1 should have matching zeros, same total count.
        expect(eq(result.out1_samples.size(), totalExpected))
            << "out1 expected " << totalExpected << " zeros, got "
            << result.out1_samples.size();

        if (result.out1_samples.size() == totalExpected) {
            bool allZero = std::all_of(result.out1_samples.begin(),
                                       result.out1_samples.end(),
                                       [](cf32 v) { return v == cf32{}; });
            expect(allZero) << "out1 should be all zeros";
        }
    };
};

// ---------------------------------------------------------------------------
const boost::ut::suite<"TxQueueSource thread safety"> thread_safety_tests = [] {
    using namespace boost::ut;

    "push from thread while draining"_test = [] {
        constexpr std::size_t kNBursts   = 20;
        constexpr std::size_t kBurstLen  = 50;
        const std::size_t     totalExpected = kNBursts * kBurstLen;

        // Build the expected output: 20 sequential bursts.
        std::vector<cf32> expectedAll;
        expectedAll.reserve(totalExpected);
        for (std::size_t b = 0; b < kNBursts; b++) {
            auto burst = makeBurst(kBurstLen, static_cast<float>(b * kBurstLen));
            expectedAll.insert(expectedAll.end(), burst.begin(), burst.end());
        }

        auto result = runTxQueue(
            [](gr::lora::TxQueueSource&, auto&) {
                // No pre-start setup — all pushes happen post-start.
            },
            [&](gr::lora::TxQueueSource& src, auto& sched) {
                // Producer thread: push bursts with slight delay.
                std::thread producer([&src]() {
                    for (std::size_t b = 0; b < kNBursts; b++) {
                        auto burst = makeBurst(kBurstLen,
                                               static_cast<float>(b * kBurstLen));
                        src.push(std::move(burst));
                        src.notifyProgress();
                        std::this_thread::sleep_for(10ms);
                    }
                });
                producer.join();

                // Give scheduler time to drain all bursts, then stop.
                std::this_thread::sleep_for(500ms);
                sched.requestStop();
            },
            5000ms);

        expect(!result.timed_out) << "test should complete before watchdog";

        // Verify total sample count — no data loss.
        expect(eq(result.out0_samples.size(), totalExpected))
            << "out0 expected " << totalExpected << " samples, got "
            << result.out0_samples.size();

        // Verify data integrity: every sample matches expected order.
        if (result.out0_samples.size() == totalExpected) {
            bool dataOk = true;
            for (std::size_t i = 0; i < totalExpected; i++) {
                if (result.out0_samples[i] != expectedAll[i]) {
                    dataOk = false;
                    boost::ut::log << "first mismatch at sample " << i
                                   << ": expected " << expectedAll[i].real()
                                   << " got " << result.out0_samples[i].real();
                    break;
                }
            }
            expect(dataOk) << "all samples should match expected order";
        }

        // out1 should be all zeros with same count.
        expect(eq(result.out1_samples.size(), totalExpected))
            << "out1 should have " << totalExpected << " zeros";

        if (result.out1_samples.size() == totalExpected) {
            bool allZero = std::all_of(result.out1_samples.begin(),
                                       result.out1_samples.end(),
                                       [](cf32 v) { return v == cf32{}; });
            expect(allZero) << "out1 should be all zeros";
        }
    };
};

int main() { /* boost::ut auto-runs all suites */ }
