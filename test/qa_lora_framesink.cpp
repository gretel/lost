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
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/FrameSink.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>

using namespace std::string_literals;

namespace {

// ============================================================================
// Test-local recursive CBOR decoder — extends cb::decode_map() to handle
// nested maps (the "phy" sub-map in FrameSink CBOR output).
// The production decoder (cbor.hpp) doesn't support nested maps in its
// Value variant. This test-local version adds a boxed Map value type.
// ============================================================================
namespace cb = gr::lora::cbor;

/// Forward declaration for the boxed sub-map type.
struct BoxedMap;

/// Extended value type that can hold nested maps.
using ExtValue = std::variant<uint64_t, std::string, std::vector<uint8_t>, bool, std::vector<uint64_t>, double, std::shared_ptr<BoxedMap>>;
using ExtMap   = std::unordered_map<std::string, ExtValue>;

struct BoxedMap {
    ExtMap map;
};

namespace test_cbor {

/// Decode a single CBOR data item (extended: supports nested maps).
inline ExtValue decode_item_ext(std::span<const uint8_t> data, std::size_t& pos) {
    if (pos >= data.size()) {
        throw cb::DecodeError("unexpected end of input");
    }
    uint8_t head  = data[pos++];
    uint8_t major = static_cast<uint8_t>(head >> 5);
    uint8_t info  = static_cast<uint8_t>(head & 0x1F);

    switch (major) {
    case 0: { // unsigned integer
        return cb::detail::read_argument(data, pos, info);
    }
    case 2: { // byte string
        uint64_t len = cb::detail::read_argument(data, pos, info);
        if (pos + len > data.size()) {
            throw cb::DecodeError("byte string overflow");
        }
        std::vector<uint8_t> bytes(data.begin() + static_cast<std::ptrdiff_t>(pos), data.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
        return bytes;
    }
    case 3: { // text string
        uint64_t len = cb::detail::read_argument(data, pos, info);
        if (pos + len > data.size()) {
            throw cb::DecodeError("text string overflow");
        }
        std::string s(reinterpret_cast<const char*>(data.data() + pos), len);
        pos += len;
        return s;
    }
    case 4: { // array (uint elements only)
        uint64_t              count = cb::detail::read_argument(data, pos, info);
        std::vector<uint64_t> arr;
        arr.reserve(count);
        for (uint64_t i = 0; i < count; i++) {
            auto  item = decode_item_ext(data, pos);
            auto* v    = std::get_if<uint64_t>(&item);
            if (!v) {
                throw cb::DecodeError("array element is not uint");
            }
            arr.push_back(*v);
        }
        return arr;
    }
    case 5: { // map (nested)
        uint64_t n_pairs = cb::detail::read_argument(data, pos, info);
        auto     sub     = std::make_shared<BoxedMap>();
        sub->map.reserve(n_pairs);
        for (uint64_t i = 0; i < n_pairs; i++) {
            auto  key_val = decode_item_ext(data, pos);
            auto* key_str = std::get_if<std::string>(&key_val);
            if (!key_str) {
                throw cb::DecodeError("map key is not text");
            }
            auto value = decode_item_ext(data, pos);
            sub->map.emplace(std::move(*key_str), std::move(value));
        }
        return sub;
    }
    case 7: { // simple values (bool, float)
        if (info == 20) {
            return false;
        }
        if (info == 21) {
            return true;
        }
        if (info == 25) {
            if (pos + 2 > data.size()) {
                throw cb::DecodeError("unexpected end");
            }
            uint16_t half = static_cast<uint16_t>(data[pos] << 8 | data[pos + 1]);
            pos += 2;
            int    exp  = (half >> 10) & 0x1F;
            int    mant = half & 0x3FF;
            double val;
            if (exp == 0) {
                val = std::ldexp(static_cast<double>(mant), -24);
            } else if (exp == 31) {
                val = (mant == 0) ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
            } else {
                val = std::ldexp(static_cast<double>(mant + 1024), exp - 25);
            }
            return (half & 0x8000) ? -val : val;
        }
        if (info == 26) {
            if (pos + 4 > data.size()) {
                throw cb::DecodeError("unexpected end");
            }
            uint32_t bits = 0;
            for (int i = 0; i < 4; i++) {
                bits = (bits << 8) | data[pos++];
            }
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return static_cast<double>(f);
        }
        if (info == 27) {
            if (pos + 8 > data.size()) {
                throw cb::DecodeError("unexpected end");
            }
            uint64_t bits = 0;
            for (int i = 0; i < 8; i++) {
                bits = (bits << 8) | data[pos++];
            }
            double d;
            std::memcpy(&d, &bits, sizeof(d));
            return d;
        }
        throw cb::DecodeError("unsupported simple value");
    }
    default: throw cb::DecodeError("unsupported major type");
    }
}

/// Decode a CBOR map with text keys (supports nested maps).
inline ExtMap decode_map_ext(std::span<const uint8_t> data) {
    if (data.empty()) {
        throw cb::DecodeError("empty input");
    }
    std::size_t pos   = 0;
    uint8_t     head  = data[pos++];
    uint8_t     major = static_cast<uint8_t>(head >> 5);
    uint8_t     info  = static_cast<uint8_t>(head & 0x1F);
    if (major != 5) {
        throw cb::DecodeError("expected CBOR map");
    }
    uint64_t n_pairs = cb::detail::read_argument(data, pos, info);
    ExtMap   result;
    result.reserve(n_pairs);
    for (uint64_t i = 0; i < n_pairs; i++) {
        auto  key_val = decode_item_ext(data, pos);
        auto* key_str = std::get_if<std::string>(&key_val);
        if (!key_str) {
            throw cb::DecodeError("map key is not text");
        }
        auto value = decode_item_ext(data, pos);
        result.emplace(std::move(*key_str), std::move(value));
    }
    return result;
}

// Typed accessors for ExtMap (mirror cb:: accessors).

inline std::string get_text(const ExtMap& m, const std::string& key, const std::string& def = "") {
    auto it = m.find(key);
    if (it == m.end()) {
        return def;
    }
    auto* v = std::get_if<std::string>(&it->second);
    return v ? *v : def;
}

inline uint64_t get_uint(const ExtMap& m, const std::string& key, uint64_t def = 0) {
    auto it = m.find(key);
    if (it == m.end()) {
        return def;
    }
    auto* v = std::get_if<uint64_t>(&it->second);
    return v ? *v : def;
}

inline bool get_bool(const ExtMap& m, const std::string& key, bool def = false) {
    auto it = m.find(key);
    if (it == m.end()) {
        return def;
    }
    auto* v = std::get_if<bool>(&it->second);
    return v ? *v : def;
}

inline double get_float64(const ExtMap& m, const std::string& key, double def = 0.0) {
    auto it = m.find(key);
    if (it == m.end()) {
        return def;
    }
    if (auto* d = std::get_if<double>(&it->second)) {
        return *d;
    }
    if (auto* u = std::get_if<uint64_t>(&it->second)) {
        return static_cast<double>(*u);
    }
    return def;
}

inline const ExtMap* get_sub_map(const ExtMap& m, const std::string& key) {
    auto it = m.find(key);
    if (it == m.end()) {
        return nullptr;
    }
    auto* box = std::get_if<std::shared_ptr<BoxedMap>>(&it->second);
    if (!box || !*box) {
        return nullptr;
    }
    return &(*box)->map;
}

inline std::vector<uint8_t> get_bytes(const ExtMap& m, const std::string& key) {
    auto it = m.find(key);
    if (it == m.end()) {
        return {};
    }
    auto* v = std::get_if<std::vector<uint8_t>>(&it->second);
    return v ? *v : std::vector<uint8_t>{};
}

} // namespace test_cbor

struct FrameResult {
    std::vector<uint8_t> cbor_buf;
    bool                 crc_valid{false};
    bool                 callback_called{false};
};

FrameResult runFrameSink(const std::vector<uint8_t>& payload, const gr::property_map& tag_map, uint16_t sync_word = 0x12, uint32_t phy_bw = 125000, const std::string& label = "") {
    using namespace gr;

    FrameResult result;

    Graph graph;

    auto& src  = graph.emplaceBlock<testing::TagSource<uint8_t, testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(payload.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    src._tags  = {Tag{0, tag_map}};
    src.values = payload;

    auto& sink = graph.emplaceBlock<gr::lora::FrameSink>({
        {"sync_word", sync_word},
        {"phy_sf", uint8_t{0}},
        {"phy_bw", phy_bw},
        {"label", label},
    });

    sink._frame_callback = [&result](const std::vector<uint8_t>& buf, bool crc_ok, uint16_t /*sync_word*/) {
        result.cbor_buf        = buf;
        result.crc_valid       = crc_ok;
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
    char        buf[4096];
    for (;;) {
        auto n = ::read(pipefd[0], buf, sizeof(buf));
        if (n <= 0) {
            break;
        }
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

} // namespace

// ============================================================================
// CBOR output suite
// ============================================================================

const boost::ut::suite<"FrameSink CBOR output"> cbor_tests = [] {
    using namespace boost::ut;

    "basic frame CBOR"_test = [] {
        auto             payload = makePayload(14);
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
        expect(haystack.find("lora_frame") != std::string::npos) << "CBOR should contain 'lora_frame' type string";

        // Decode CBOR and verify field values
        auto m = test_cbor::decode_map_ext(std::span<const uint8_t>(result.cbor_buf));
        expect(test_cbor::get_text(m, "type") == "lora_frame"s) << "type should be lora_frame";
        expect(eq(test_cbor::get_uint(m, "seq"), uint64_t{1})) << "first frame should have seq=1";
        // Top-level crc_valid (canonical location after carrier/phy split)
        expect(test_cbor::get_bool(m, "crc_valid") == true) << "top-level crc_valid should be true";
        expect(eq(test_cbor::get_uint(m, "payload_len"), uint64_t{14})) << "payload_len should match";
        expect(test_cbor::get_bool(m, "is_downchirp") == false) << "is_downchirp should be false";
        // Verify payload bytes are present and match
        auto decoded_payload = test_cbor::get_bytes(m, "payload");
        expect(eq(decoded_payload.size(), 14UZ)) << "payload should be 14 bytes";
        expect(decoded_payload == payload) << "payload bytes should match input";
        // Verify id (UUID) is present and non-empty
        auto id = test_cbor::get_text(m, "id");
        expect(id.size() > 0UZ) << "id (UUID) should be present";
        // Verify direction is always "rx" on the RX side
        expect(test_cbor::get_text(m, "direction") == "rx"s) << "direction should be 'rx'";
        // Verify carrier sub-map (frame identity: sync/sf/bw/cr)
        auto* carrier = test_cbor::get_sub_map(m, "carrier");
        expect(carrier != nullptr) << "carrier sub-map should be present";
        if (carrier) {
            expect(eq(test_cbor::get_uint(*carrier, "sf"), uint64_t{8})) << "carrier.sf should be 8";
            expect(eq(test_cbor::get_uint(*carrier, "bw"), uint64_t{125000})) << "carrier.bw should be 125000";
            expect(eq(test_cbor::get_uint(*carrier, "cr"), uint64_t{4})) << "carrier.cr should be 4";
            expect(eq(test_cbor::get_uint(*carrier, "sync_word"), uint64_t{0x12})) << "carrier.sync_word should be 0x12";
        }
        // Verify phy sub-map (DSP state only — no sync_word/sf/bw/cr here)
        auto* phy = test_cbor::get_sub_map(m, "phy");
        expect(phy != nullptr) << "phy sub-map should be present";
        if (phy) {
            expect(std::abs(test_cbor::get_float64(*phy, "snr_db") - 5.0) < 0.01) << "phy.snr_db should be 5.0";
        }
        // crc_valid is top-level (moved out of phy in the carrier/phy split).
        expect(test_cbor::get_bool(m, "crc_valid") == true) << "top-level crc_valid should be true";
    };

    "CRC fail"_test = [] {
        auto             payload = makePayload(10);
        gr::property_map frame_tag{
            {"pay_len", int64_t{10}},
            {"crc_valid", false},
        };

        auto result = runFrameSink(payload, frame_tag);

        expect(result.callback_called) << "callback should have been called";
        expect(!result.crc_valid) << "crc_valid should be false";

        // Decode CBOR and verify crc_valid is false
        expect(gt(result.cbor_buf.size(), 0UZ)) << "CBOR buffer should be non-empty";
        auto m = test_cbor::decode_map_ext(std::span<const uint8_t>(result.cbor_buf));
        expect(test_cbor::get_text(m, "type") == "lora_frame"s);
        expect(test_cbor::get_bool(m, "crc_valid") == false) << "CBOR crc_valid should be false";
        // crc_valid is top-level now (carrier/phy split).
        expect(test_cbor::get_bool(m, "crc_valid") == false) << "top-level crc_valid should be false";
        // Verify payload round-trips correctly even on CRC fail
        auto decoded_payload = test_cbor::get_bytes(m, "payload");
        expect(eq(decoded_payload.size(), 10UZ)) << "payload should be 10 bytes";
    };

    "optional fields present"_test = [] {
        auto             payload = makePayload(14);
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
        expect(gt(result_full.cbor_buf.size(), result_minimal.cbor_buf.size())) << "CBOR with optional fields should be larger: " << result_full.cbor_buf.size() << " vs " << result_minimal.cbor_buf.size();

        // Decode the full CBOR and verify optional fields have correct values
        auto m = test_cbor::decode_map_ext(std::span<const uint8_t>(result_full.cbor_buf));
        // rx_channel is a top-level optional field
        expect(eq(test_cbor::get_uint(m, "rx_channel"), uint64_t{100})) << "rx_channel should be 100";
        // Optional fields in the phy sub-map
        auto* phy = test_cbor::get_sub_map(m, "phy");
        expect(phy != nullptr) << "phy sub-map should be present";
        if (phy) {
            expect(std::abs(test_cbor::get_float64(*phy, "snr_db") - 5.0) < 0.01) << "phy.snr_db should be 5.0";
            expect(std::abs(test_cbor::get_float64(*phy, "noise_floor_db") - (-45.0)) < 0.01) << "phy.noise_floor_db should be -45.0";
            expect(std::abs(test_cbor::get_float64(*phy, "peak_db") - (-12.0)) < 0.01) << "phy.peak_db should be -12.0";
            expect(std::abs(test_cbor::get_float64(*phy, "snr_db_td") - 3.5) < 0.01) << "phy.snr_db_td should be 3.5";
        }

        // Verify the minimal CBOR does NOT contain the optional phy fields
        auto  m_min   = test_cbor::decode_map_ext(std::span<const uint8_t>(result_minimal.cbor_buf));
        auto* phy_min = test_cbor::get_sub_map(m_min, "phy");
        expect(phy_min != nullptr) << "minimal phy sub-map should be present";
        if (phy_min) {
            // noise_floor_db should be absent (sentinel -999.0 means not emitted)
            expect(phy_min->find("noise_floor_db") == phy_min->end()) << "minimal phy should not contain noise_floor_db";
            expect(phy_min->find("peak_db") == phy_min->end()) << "minimal phy should not contain peak_db";
            expect(phy_min->find("snr_db_td") == phy_min->end()) << "minimal phy should not contain snr_db_td";
        }
        // rx_channel should be absent in minimal
        expect(m_min.find("rx_channel") == m_min.end()) << "minimal should not contain rx_channel";
    };

    "downchirp flag"_test = [] {
        auto             payload = makePayload(8);
        gr::property_map frame_tag{
            {"pay_len", int64_t{8}},
            {"is_downchirp", true},
        };

        auto result = runFrameSink(payload, frame_tag);
        expect(result.callback_called) << "callback should have been called for downchirp frame";

        // Decode CBOR and verify is_downchirp flag
        expect(gt(result.cbor_buf.size(), 0UZ)) << "CBOR buffer should be non-empty";
        auto m = test_cbor::decode_map_ext(std::span<const uint8_t>(result.cbor_buf));
        expect(test_cbor::get_text(m, "type") == "lora_frame"s);
        expect(test_cbor::get_bool(m, "is_downchirp") == true) << "CBOR is_downchirp should be true";
        // Verify payload
        auto decoded_payload = test_cbor::get_bytes(m, "payload");
        expect(eq(decoded_payload.size(), 8UZ)) << "payload should be 8 bytes";
    };

    "phy map emits sample_rate, frequency_corrected and ppm_error timing fields"_test = [] {
        // Task 7 (FrameSink) surfaces the upstream MultiSfDecoder timing
        // cache into the phy CBOR sub-map. Here we bypass MultiSfDecoder
        // and drive the tags directly on FrameSink's input.
        //
        // Tag types match SigMF defaults (Tag.hpp:196): sample_rate is
        // float32 (SigMF tag::SAMPLE_RATE), frequency is float64 (SigMF
        // tag::FREQUENCY), ppm_error is a custom float64 tag. FrameSink
        // caches sample_rate as float and widens to double only at the
        // CBOR emission boundary.
        auto             payload = makePayload(8);
        gr::property_map frame_tag{
            {"pay_len", int64_t{8}},
            {"sf", int64_t{8}},
            {"cr", int64_t{1}},
            {"crc_valid", true},
            {"snr_db", 5.0},
            {"sample_rate", 250000.f},
            {"frequency", 869.618e6},
            {"ppm_error", 1.2},
        };

        auto result = runFrameSink(payload, frame_tag);

        expect(result.callback_called) << "callback should have been called";
        expect(gt(result.cbor_buf.size(), 0UZ)) << "CBOR buffer should be non-empty";

        auto  m   = test_cbor::decode_map_ext(std::span<const uint8_t>(result.cbor_buf));
        auto* phy = test_cbor::get_sub_map(m, "phy");
        expect(phy != nullptr) << "phy sub-map should be present";
        if (phy) {
            expect(std::abs(test_cbor::get_float64(*phy, "sample_rate") - 250000.0) < 1e-9) << "phy.sample_rate should be 250000.0";
            expect(std::abs(test_cbor::get_float64(*phy, "frequency_corrected") - 869.618e6) < 1.0) << "phy.frequency_corrected should be 869.618e6";
            expect(std::abs(test_cbor::get_float64(*phy, "ppm_error") - 1.2) < 1e-6) << "phy.ppm_error should be 1.2";
        }
    };
};

// ============================================================================
// Text output suite
// ============================================================================

const boost::ut::suite<"FrameSink text output"> text_tests = [] {
    using namespace boost::ut;

    "single line format"_test = [] {
        auto             payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"sf", int64_t{8}},
            {"cr", int64_t{4}},
            {"crc_valid", true},
            {"snr_db", 5.0},
        };

        std::string output = captureStderr([&] { runFrameSink(payload, frame_tag); });

        // Should contain key=value format elements
        expect(output.find("sf=") != std::string::npos) << "output should contain 'sf=': " << output;
        expect(output.find("bw=") != std::string::npos) << "output should contain 'bw=': " << output;
        expect(output.find("bytes=") != std::string::npos) << "output should contain 'bytes=': " << output;
        expect(output.find("cr=4/") != std::string::npos) << "output should contain 'cr=4/': " << output;
        expect(output.find("crc=OK") != std::string::npos) << "output should contain 'crc=OK': " << output;
        expect(output.find("sync=0x") != std::string::npos) << "output should contain 'sync=0x': " << output;
        expect(output.find("snr=") != std::string::npos) << "output should contain 'snr=': " << output;
        expect(output.find("seq=#") != std::string::npos) << "output should contain 'seq=#': " << output;

        // Exactly one frame = exactly one newline in the frame output line.
        // Filter to lines containing "sf=" to avoid scheduler noise.
        std::size_t frame_lines = 0;
        std::size_t pos         = 0;
        while (pos < output.size()) {
            auto nl = output.find('\n', pos);
            if (nl == std::string::npos) {
                nl = output.size();
            }
            auto line = output.substr(pos, nl - pos);
            if (line.find("sf=") != std::string::npos && line.find("bytes=") != std::string::npos) {
                frame_lines++;
            }
            pos = nl + 1;
        }
        expect(eq(frame_lines, 1UZ)) << "should have exactly 1 frame line";
    };

    "no ASCII dump"_test = [] {
        auto             payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] { runFrameSink(payload, frame_tag); });

        expect(output.find("ASCII:") == std::string::npos) << "output should NOT contain 'ASCII:': " << output;
    };

    "no Hex prefix"_test = [] {
        auto             payload = makePayload(14);
        gr::property_map frame_tag{
            {"pay_len", int64_t{14}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] { runFrameSink(payload, frame_tag); });

        expect(output.find("Hex:") == std::string::npos) << "output should NOT contain 'Hex:': " << output;
    };

    "hex truncation for long payload"_test = [] {
        auto             payload = makePayload(100);
        gr::property_map frame_tag{
            {"pay_len", int64_t{100}},
            {"crc_valid", true},
        };

        std::string output = captureStderr([&] { runFrameSink(payload, frame_tag); });

        // Find the frame line (contains "bytes=")
        std::string frame_line;
        std::size_t pos = 0;
        while (pos < output.size()) {
            auto nl = output.find('\n', pos);
            if (nl == std::string::npos) {
                nl = output.size();
            }
            auto line = output.substr(pos, nl - pos);
            if (line.find("bytes=") != std::string::npos) {
                frame_line = line;
                break;
            }
            pos = nl + 1;
        }

        expect(!frame_line.empty()) << "should have a frame line";
        // >32 bytes payload → hex is truncated with "..."
        expect(frame_line.find("...") != std::string::npos) << "long payload should be truncated with '...': " << frame_line;
    };
};

int main() { /* boost::ut auto-runs all suites */ }
