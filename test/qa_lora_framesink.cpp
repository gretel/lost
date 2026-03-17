// SPDX-License-Identifier: ISC
/// Tests for FrameSink: CBOR output and text logging verification.
///
/// Test progression:
///   CBOR suite:
///     1. basic frame produces CBOR via callback
///     2. CRC fail propagates to callback
///     3. optional fields increase CBOR size
///     4. downchirp flag accepted
///   Text suite:
///     5. single-line format contains expected fields
///     6. no ASCII dump in output
///     7. no Hex: prefix in output
///     8. long payload hex is truncated with "..."

#include "test_helpers.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/FrameSink.hpp>

using namespace std::string_literals;

namespace {

struct FrameResult {
    std::vector<uint8_t> cbor_buf;
    bool                 crc_valid{false};
    bool                 callback_called{false};
};

FrameResult runFrameSink(
        const std::vector<uint8_t>& payload,
        const gr::property_map& tag_map,
        uint16_t sync_word = 0x12,
        uint32_t phy_bw = 125000,
        const std::string& label = "") {
    using namespace gr;

    FrameResult result;

    Graph graph;

    auto& src = graph.emplaceBlock<testing::TagSource<uint8_t,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(payload.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    src._tags = {Tag{0, tag_map}};
    src.values = payload;

    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"sync_word", sync_word},
        {"phy_sf", uint8_t{0}},
        {"phy_bw", phy_bw},
        {"label", label},
    });

    sink._frame_callback = [&result](const std::vector<uint8_t>& buf, bool crc_ok) {
        result.cbor_buf = buf;
        result.crc_valid = crc_ok;
        result.callback_called = true;
    };

    (void)graph.connect<"out", "in">(src, sink);

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error("scheduler init failed");
    }
    sched.runAndWait();

    return result;
}

// Capture stderr around a function call by redirecting fd 2 to a pipe.
std::string captureStderr(std::function<void()> fn) {
    int pipefd[2];
    if (::pipe(pipefd) != 0) {
        throw std::runtime_error("pipe() failed");
    }

    int saved_stderr = ::dup(STDERR_FILENO);
    ::dup2(pipefd[1], STDERR_FILENO);

    fn();

    // Flush stderr so buffered output reaches the pipe
    std::fflush(stderr);

    ::dup2(saved_stderr, STDERR_FILENO);
    ::close(pipefd[1]);

    std::string output;
    char buf[4096];
    for (;;) {
        auto n = ::read(pipefd[0], buf, sizeof(buf));
        if (n <= 0) break;
        output.append(buf, static_cast<std::size_t>(n));
    }
    ::close(pipefd[0]);
    ::close(saved_stderr);

    return output;
}

std::vector<uint8_t> makePayload(std::size_t n) {
    std::vector<uint8_t> p(n);
    for (std::size_t i = 0; i < n; i++) {
        p[i] = static_cast<uint8_t>(i & 0xFF);
    }
    return p;
}

}  // namespace

// ============================================================================
// CBOR output suite
// ============================================================================

const boost::ut::suite<"FrameSink CBOR output"> cbor_tests = [] {
    using namespace boost::ut;

    "basic frame CBOR"_test = [] {
        auto payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"sf", int64_t{8}},
            {"cr", int64_t{4}},
            {"crc_valid", true},
            {"snr_db", 5.0},
        };

        auto result = runFrameSink(payload, frame_tag);

        expect(result.callback_called) << "callback should have been called";
        expect(result.crc_valid) << "crc_valid should be true";
        expect(gt(result.cbor_buf.size(), 0UZ)) << "CBOR buffer should be non-empty";

        // CBOR map marker: major type 5 (0xA0-0xBF)
        expect(ge(result.cbor_buf[0], uint8_t{0xA0})) << "should start with CBOR map";

        // The CBOR buffer should contain "lora_frame" as a text string
        auto haystack = std::string(result.cbor_buf.begin(), result.cbor_buf.end());
        expect(haystack.find("lora_frame") != std::string::npos)
            << "CBOR should contain 'lora_frame' type string";
    };

    "CRC fail"_test = [] {
        auto payload = makePayload(10);
        gr::property_map frame_tag{
            {"pay_len", int64_t{10}},
            {"crc_valid", false},
        };

        auto result = runFrameSink(payload, frame_tag);

        expect(result.callback_called) << "callback should have been called";
        expect(!result.crc_valid) << "crc_valid should be false";
    };

    "optional fields present"_test = [] {
        auto payload = makePayload(14);
        gr::property_map tag_minimal{
            {"pay_len", int64_t{14}},
            {"sf", int64_t{8}},
            {"cr", int64_t{4}},
            {"crc_valid", true},
            {"snr_db", 5.0},
        };
        auto result_minimal = runFrameSink(payload, tag_minimal);

        gr::property_map tag_full{
            {"pay_len", int64_t{14}},
            {"sf", int64_t{8}},
            {"cr", int64_t{4}},
            {"crc_valid", true},
            {"snr_db", 5.0},
            {"noise_floor_db", -45.0},
            {"peak_db", -12.0},
            {"snr_db_td", 3.5},
            {"rx_channel", int64_t{100}},
        };
        auto result_full = runFrameSink(payload, tag_full);

        expect(result_full.callback_called) << "callback should have been called";
        expect(gt(result_full.cbor_buf.size(), result_minimal.cbor_buf.size()))
            << "CBOR with optional fields should be larger: "
            << result_full.cbor_buf.size() << " vs " << result_minimal.cbor_buf.size();
    };

    "downchirp flag"_test = [] {
        auto payload = makePayload(8);
        gr::property_map frame_tag{
            {"pay_len", int64_t{8}},
            {"is_downchirp", true},
        };

        auto result = runFrameSink(payload, frame_tag);
        expect(result.callback_called) << "callback should have been called for downchirp frame";
    };
};

// ============================================================================
// Text output suite
// ============================================================================

const boost::ut::suite<"FrameSink text output"> text_tests = [] {
    using namespace boost::ut;

    "single line format"_test = [] {
        auto payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"sf", int64_t{8}},
            {"cr", int64_t{4}},
            {"crc_valid", true},
            {"snr_db", 5.0},
        };

        std::string output = captureStderr([&] {
            runFrameSink(payload, frame_tag);
        });

        // Should contain key=value format elements
        expect(output.find("sf=") != std::string::npos)
            << "output should contain 'sf=': " << output;
        expect(output.find("bw=") != std::string::npos)
            << "output should contain 'bw=': " << output;
        expect(output.find("bytes=") != std::string::npos)
            << "output should contain 'bytes=': " << output;
        expect(output.find("cr=4/") != std::string::npos)
            << "output should contain 'cr=4/': " << output;
        expect(output.find("crc=OK") != std::string::npos)
            << "output should contain 'crc=OK': " << output;
        expect(output.find("sync=0x") != std::string::npos)
            << "output should contain 'sync=0x': " << output;
        expect(output.find("snr=") != std::string::npos)
            << "output should contain 'snr=': " << output;
        expect(output.find("seq=#") != std::string::npos)
            << "output should contain 'seq=#': " << output;

        // Exactly one frame = exactly one newline in the frame output line.
        // Filter to lines containing "sf=" to avoid scheduler noise.
        std::size_t frame_lines = 0;
        std::size_t pos = 0;
        while (pos < output.size()) {
            auto nl = output.find('\n', pos);
            if (nl == std::string::npos) nl = output.size();
            auto line = output.substr(pos, nl - pos);
            if (line.find("sf=") != std::string::npos
                && line.find("bytes=") != std::string::npos) {
                frame_lines++;
            }
            pos = nl + 1;
        }
        expect(eq(frame_lines, 1UZ)) << "should have exactly 1 frame line";
    };

    "no ASCII dump"_test = [] {
        auto payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] {
            runFrameSink(payload, frame_tag);
        });

        expect(output.find("ASCII:") == std::string::npos)
            << "output should NOT contain 'ASCII:': " << output;
    };

    "no Hex prefix"_test = [] {
        auto payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] {
            runFrameSink(payload, frame_tag);
        });

        expect(output.find("Hex:") == std::string::npos)
            << "output should NOT contain 'Hex:': " << output;
    };

    "hex truncation for long payload"_test = [] {
        auto payload = makePayload(100);
        gr::property_map frame_tag{
            {"pay_len", int64_t{100}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] {
            runFrameSink(payload, frame_tag);
        });

        // Find the frame line (contains "bytes=")
        std::string frame_line;
        std::size_t pos = 0;
        while (pos < output.size()) {
            auto nl = output.find('\n', pos);
            if (nl == std::string::npos) nl = output.size();
            auto line = output.substr(pos, nl - pos);
            if (line.find("bytes=") != std::string::npos) {
                frame_line = line;
                break;
            }
            pos = nl + 1;
        }

        expect(!frame_line.empty()) << "should have a frame line";
        // >32 bytes payload → hex is truncated with "..."
        expect(frame_line.find("...") != std::string::npos)
            << "long payload should be truncated with '...': " << frame_line;
    };
};

int main() { /* boost::ut auto-runs all suites */ }
