// SPDX-License-Identifier: ISC
//
// CBOR event encoder for the lora apps.
// Two layers of API:
//   (1) encode(property_map)      — auto-adds "ts"; scalar + nested-map + Tensor
//   (2) beginEvent(type, nFields) — opens a map, writes "type"+"ts", returns the
//                                   buffer so callers can append remaining fields
//                                   with direct cbor.hpp calls (for byte-string
//                                   blobs and other shapes that don't fit
//                                   property_map cleanly).

#ifndef GR4_LORA_TELEMETRY_HPP
#define GR4_LORA_TELEMETRY_HPP

#include <chrono>
#include <cstdio>
#include <ctime>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>

namespace gr::lora::telemetry {

/// ISO 8601 UTC timestamp (2026-03-22T03:05:19.627Z)
inline std::string isoTimestamp() {
    auto    now = std::chrono::system_clock::now();
    auto    tt  = std::chrono::system_clock::to_time_t(now);
    auto    ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
    ::gmtime_r(&tt, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms.count()));
    return {buf};
}

namespace detail {

inline void encodeValue(std::vector<uint8_t>& buf, const pmt::Value& val) {
    namespace cb = gr::lora::cbor;

    // signed integers
    if (auto* p = val.get_if<int8_t>()) {
        cb::encode_sint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<int16_t>()) {
        cb::encode_sint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<int32_t>()) {
        cb::encode_sint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<int64_t>()) {
        cb::encode_sint(buf, *p);
        return;
    }

    // unsigned integers
    if (auto* p = val.get_if<uint8_t>()) {
        cb::encode_uint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<uint16_t>()) {
        cb::encode_uint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<uint32_t>()) {
        cb::encode_uint(buf, *p);
        return;
    }
    if (auto* p = val.get_if<uint64_t>()) {
        cb::encode_uint(buf, *p);
        return;
    }

    // floats
    if (auto* p = val.get_if<float>()) {
        cb::encode_float64(buf, static_cast<double>(*p));
        return;
    }
    if (auto* p = val.get_if<double>()) {
        cb::encode_float64(buf, *p);
        return;
    }

    // bool
    if (auto* p = val.get_if<bool>()) {
        cb::encode_bool(buf, *p);
        return;
    }

    // string (pmr::string is the native storage type in pmt::Value;
    // std::string is explicitly excluded by pmt::Value::get_if constraints)
    if (auto* p = val.get_if<std::pmr::string>()) {
        cb::encode_text(buf, std::string(*p));
        return;
    }

    // nested map
    if (auto* p = val.get_if<pmt::Value::Map>()) {
        cb::encode_map_begin(buf, p->size());
        for (const auto& [k, v] : *p) {
            cb::encode_text(buf, std::string(k));
            encodeValue(buf, v);
        }
        return;
    }

    // Tensor<uint8_t> → CBOR byte string (major type 2)
    if (auto* p = val.get_if<Tensor<std::uint8_t>>()) {
        cb::encode_bytes(buf, p->data(), p->size());
        return;
    }

    // Tensor<T> for numeric T → CBOR array of T
    auto encodeArray = [&](auto* ptr, auto writer) {
        cb::encode_array_begin(buf, ptr->size());
        for (const auto& elem : *ptr) {
            writer(buf, elem);
        }
    };
    if (auto* p = val.get_if<Tensor<std::uint16_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_uint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<std::uint32_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_uint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<std::uint64_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_uint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<std::int16_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_sint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<std::int32_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_sint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<std::int64_t>>()) {
        encodeArray(p, [](auto& b, auto v) { cb::encode_sint(b, v); });
        return;
    }
    if (auto* p = val.get_if<Tensor<float>>()) {
        encodeArray(p, [](auto& b, float v) { cb::encode_float64(b, static_cast<double>(v)); });
        return;
    }
    if (auto* p = val.get_if<Tensor<double>>()) {
        encodeArray(p, [](auto& b, double v) { cb::encode_float64(b, v); });
        return;
    }

    // Tensor<Value> → CBOR array of values (recursed through encodeValue)
    if (auto* p = val.get_if<Tensor<pmt::Value>>()) {
        cb::encode_array_begin(buf, p->size());
        for (const auto& elem : *p) {
            encodeValue(buf, elem);
        }
        return;
    }

    std::fprintf(stderr, "[telemetry] unhandled pmt::Value type, skipping\n");
    cb::encode_bool(buf, false); // placeholder to keep CBOR map count valid
}

} // namespace detail

/// Encode a property_map as a CBOR map. Adds "ts" if not present.
inline std::vector<uint8_t> encode(const gr::property_map& evt) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(512);

    bool hasTs = evt.contains("ts");
    cb::encode_map_begin(buf, evt.size() + (hasTs ? 0 : 1));

    if (!hasTs) {
        cb::kv_text(buf, "ts", isoTimestamp());
    }

    for (const auto& [key, val] : evt) {
        cb::encode_text(buf, std::string(key));
        detail::encodeValue(buf, val);
    }
    return buf;
}

/// Open a CBOR event map with the standard "type" + "ts" header fields.
/// Caller then appends nBodyFields more kv_* pairs with direct cbor.hpp calls
/// (useful when the body contains byte strings or arrays that don't fit
/// a property_map cleanly).
inline std::vector<uint8_t> beginEvent(std::string_view type, std::size_t nBodyFields) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(256);
    cb::encode_map_begin(buf, nBodyFields + 2); // +type +ts
    cb::kv_text(buf, "type", std::string(type));
    cb::kv_text(buf, "ts", isoTimestamp());
    return buf;
}

} // namespace gr::lora::telemetry

#endif // GR4_LORA_TELEMETRY_HPP
