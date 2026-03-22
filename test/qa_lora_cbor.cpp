// SPDX-License-Identifier: ISC
// Unit tests for the minimal CBOR encoder + decoder (cbor.hpp) and
// the telemetry property_map encoder (Telemetry.hpp).

#include "test_helpers.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/algorithm/Telemetry.hpp>

using namespace std::string_literals;
namespace cb = gr::lora::cbor;

// ═══════════════════════════════════════════════════════════════════════════════
// Suite 1: CBOR encoding primitives
// ═══════════════════════════════════════════════════════════════════════════════

static const boost::ut::suite cbor_encoding = [] {
    using namespace boost::ut;

    "encode_uint tiny (0)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 0);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x00});
    };

    "encode_uint 1-byte (23)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 23);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x17});
    };

    "encode_uint 2-byte (24)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 24);
        expect(eq(buf.size(), 2uz));
        expect(buf[0] == uint8_t{0x18});
        expect(buf[1] == uint8_t{24});
    };

    "encode_uint 3-byte (256)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 256);
        expect(eq(buf.size(), 3uz));
        expect(buf[0] == uint8_t{0x19});  // 2-byte follows
        expect(buf[1] == uint8_t{0x01});
        expect(buf[2] == uint8_t{0x00});
    };

    "encode_uint 5-byte (65536)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 65536);
        expect(eq(buf.size(), 5uz));
        expect(buf[0] == uint8_t{0x1A});  // 4-byte follows
    };

    "encode_uint 9-byte (large 64-bit)"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 0x100000000ULL);
        expect(eq(buf.size(), 9uz));
        expect(buf[0] == uint8_t{0x1B});  // 8-byte follows
    };

    "encode_sint positive"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_sint(buf, 42);
        std::vector<uint8_t> ref;
        cb::encode_uint(ref, 42);
        expect(buf == ref);
    };

    "encode_sint -1"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_sint(buf, -1);
        // CBOR: major 1, value 0 → 0x20
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x20});
    };

    "encode_sint -24"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_sint(buf, -24);
        // CBOR: major 1, value 23 → 0x37
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x37});
    };

    "encode_sint -25"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_sint(buf, -25);
        // CBOR: major 1, additional 24, value 24 → 0x38 0x18
        expect(eq(buf.size(), 2uz));
        expect(buf[0] == uint8_t{0x38});
        expect(buf[1] == uint8_t{0x18});
    };

    "encode_sint INT64_MIN no UB"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_sint(buf, std::numeric_limits<int64_t>::min());
        expect(eq(buf.size(), 9uz));
        expect(buf[0] == uint8_t{0x3B});  // major 1, 8-byte
    };

    "encode_text empty"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_text(buf, "");
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x60});
    };

    "encode_text hello"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_text(buf, "hello");
        expect(eq(buf.size(), 6uz));
        expect(buf[0] == uint8_t{0x65});
        expect(buf[1] == uint8_t{'h'});
    };

    "encode_bytes empty"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_bytes(buf, nullptr, 0);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x40});
    };

    "encode_bytes 5 bytes"_test = [] {
        std::vector<uint8_t> buf;
        uint8_t data[] = {1, 2, 3, 4, 5};
        cb::encode_bytes(buf, data, 5);
        expect(eq(buf.size(), 6uz));
        expect(buf[0] == uint8_t{0x45});
        expect(buf[5] == uint8_t{5});
    };

    "encode_bool true"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_bool(buf, true);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0xF5});
    };

    "encode_bool false"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_bool(buf, false);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0xF4});
    };

    "encode_float64"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_float64(buf, 1.5);
        expect(eq(buf.size(), 9uz));
        expect(buf[0] == uint8_t{0xFB});
        // Decode raw bytes back to verify round-trip
        uint64_t bits = 0;
        for (int i = 0; i < 8; i++) bits = (bits << 8) | buf[static_cast<std::size_t>(1 + i)];
        double decoded{};
        std::memcpy(&decoded, &bits, sizeof(decoded));
        expect(eq(decoded, 1.5));
    };

    "encode_map_begin"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 3);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0xA3});
    };

    "encode_array_begin"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_array_begin(buf, 2);
        expect(eq(buf.size(), 1uz));
        expect(buf[0] == uint8_t{0x82});
    };
};

// ═══════════════════════════════════════════════════════════════════════════════
// Suite 2: CBOR encode→decode round-trip
// ═══════════════════════════════════════════════════════════════════════════════

static const boost::ut::suite cbor_roundtrip = [] {
    using namespace boost::ut;

    "round-trip map with mixed types"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 5);
        cb::kv_uint(buf, "count", 42);
        cb::kv_text(buf, "name", "test");
        cb::kv_bool(buf, "active", true);
        cb::kv_float64(buf, "ratio", 3.14);
        uint8_t payload[] = {0xDE, 0xAD};
        cb::kv_bytes(buf, "data", payload, 2);

        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(m.size(), 5uz));
        expect(eq(cb::get_uint_or(m, "count", 0), uint64_t{42}));
        expect(cb::get_text_or(m, "name", "") == "test"s);
        expect(cb::get_bool_or(m, "active", false) == true);
        expect(std::abs(cb::get_float64_or(m, "ratio", 0.0) - 3.14) < 1e-10);
        auto bytes = cb::get_bytes(m, "data");
        expect(eq(bytes.size(), 2uz));
        expect(bytes[0] == uint8_t{0xDE});
    };

    "round-trip empty map"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 0);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(m.empty());
    };

    "decode_map rejects non-map"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_uint(buf, 42);
        expect(throws([&] { cb::decode_map(std::span<const uint8_t>(buf)); }));
    };

    "decode_map rejects empty input"_test = [] {
        std::vector<uint8_t> buf;
        expect(throws([&] { cb::decode_map(std::span<const uint8_t>(buf)); }));
    };

    "decode_map rejects truncated input"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 2);
        cb::kv_uint(buf, "a", 1);
        // Only 1 of 2 pairs — truncated
        expect(throws([&] { cb::decode_map(std::span<const uint8_t>(buf)); }));
    };

    "typed accessor defaults"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 0);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(cb::get_uint_or(m, "missing", 99), uint64_t{99}));
        expect(cb::get_text_or(m, "missing", "def") == "def"s);
        expect(cb::get_bool_or(m, "missing", true) == true);
        expect(eq(cb::get_float64_or(m, "missing", 1.0), 1.0));
    };

    "kv_sint positive round-trip"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 1);
        cb::kv_sint(buf, "val", 100);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(cb::get_uint_or(m, "val", 0), uint64_t{100}));
    };

    "float64_or accepts uint"_test = [] {
        std::vector<uint8_t> buf;
        cb::encode_map_begin(buf, 1);
        cb::kv_uint(buf, "val", 42);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(cb::get_float64_or(m, "val", 0.0), 42.0));
    };
};

// ═══════════════════════════════════════════════════════════════════════════════
// Suite 3: Telemetry property_map → CBOR encoding
// ═══════════════════════════════════════════════════════════════════════════════

static const boost::ut::suite telemetry_encoding = [] {
    using namespace boost::ut;

    "telemetry encode basic types"_test = [] {
        gr::property_map evt;
        evt["type"]  = std::pmr::string("test_event");
        evt["count"] = static_cast<uint32_t>(42);
        evt["ratio"] = 3.14;
        evt["flag"]  = true;

        auto buf = gr::lora::telemetry::encode(evt);
        expect(buf.size() > 10uz) << "CBOR output should be non-trivial";

        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(cb::get_text_or(m, "type", "") == "test_event"s);
        expect(eq(cb::get_uint_or(m, "count", 0), uint64_t{42}));
        expect(std::abs(cb::get_float64_or(m, "ratio", 0.0) - 3.14) < 1e-10);
        expect(cb::get_bool_or(m, "flag", false) == true);
    };

    "telemetry auto-adds timestamp"_test = [] {
        gr::property_map evt;
        evt["type"] = std::pmr::string("ts_test");

        auto buf = gr::lora::telemetry::encode(evt);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));

        expect(m.contains("ts")) << "ts should be auto-added";
        auto ts = cb::get_text_or(m, "ts", "");
        expect(ts.size() >= 20uz) << "timestamp should be ISO 8601";
        expect(ts.back() == 'Z') << "timestamp should end with Z (UTC)";
        expect(ts[4] == '-') << "timestamp should have date separator";
    };

    "telemetry preserves existing ts"_test = [] {
        gr::property_map evt;
        evt["type"] = std::pmr::string("ts_test");
        evt["ts"]   = std::pmr::string("custom_timestamp");

        auto buf = gr::lora::telemetry::encode(evt);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(cb::get_text_or(m, "ts", "") == "custom_timestamp"s);
    };

    "telemetry signed integer types"_test = [] {
        gr::property_map evt;
        evt["type"]  = std::pmr::string("sint_test");
        evt["i32"]   = static_cast<int32_t>(42);

        auto buf = gr::lora::telemetry::encode(evt);
        expect(buf.size() > 5uz);

        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(cb::get_uint_or(m, "i32", 0), uint64_t{42}));
    };

    "telemetry unsigned integer types"_test = [] {
        gr::property_map evt;
        evt["type"]  = std::pmr::string("uint_test");
        evt["u8"]    = static_cast<uint8_t>(255);
        evt["u16"]   = static_cast<uint16_t>(1000);
        evt["u32"]   = static_cast<uint32_t>(100000);
        evt["u64"]   = static_cast<uint64_t>(1ULL << 40);

        auto buf = gr::lora::telemetry::encode(evt);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(eq(cb::get_uint_or(m, "u8", 0), uint64_t{255}));
        expect(eq(cb::get_uint_or(m, "u16", 0), uint64_t{1000}));
        expect(eq(cb::get_uint_or(m, "u32", 0), uint64_t{100000}));
        expect(eq(cb::get_uint_or(m, "u64", 0), uint64_t{1ULL << 40}));
    };

    "telemetry float and double"_test = [] {
        gr::property_map evt;
        evt["type"]  = std::pmr::string("float_test");
        evt["f32"]   = 1.5f;
        evt["f64"]   = 2.718;

        auto buf = gr::lora::telemetry::encode(evt);
        auto m = cb::decode_map(std::span<const uint8_t>(buf));
        expect(std::abs(cb::get_float64_or(m, "f32", 0.0) - 1.5) < 1e-6);
        expect(std::abs(cb::get_float64_or(m, "f64", 0.0) - 2.718) < 1e-10);
    };
};

int main() {}
