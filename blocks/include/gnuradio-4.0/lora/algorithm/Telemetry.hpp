// SPDX-License-Identifier: ISC
//
// Generic property_map → CBOR encoder for block telemetry events.
// Blocks emit property_map with a "type" key; the app encodes to CBOR and
// broadcasts via UDP.  Blocks stay CBOR-agnostic.

#ifndef GR4_LORA_TELEMETRY_HPP
#define GR4_LORA_TELEMETRY_HPP

#include <chrono>
#include <cstdio>
#include <ctime>
#include <functional>
#include <string>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>

namespace gr::lora::telemetry {

/// ISO 8601 UTC timestamp (2026-03-22T03:05:19.627Z)
inline std::string isoTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto tt  = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;
    std::tm tm{};
    ::gmtime_r(&tt, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms.count()));
    return {buf};
}

namespace detail {

inline void encodeValue(std::vector<uint8_t>& buf, const pmt::Value& val) {
    namespace cb = gr::lora::cbor;

    // signed integers
    if (auto* p = val.get_if<int8_t>())   { cb::encode_sint(buf, *p); return; }
    if (auto* p = val.get_if<int16_t>())  { cb::encode_sint(buf, *p); return; }
    if (auto* p = val.get_if<int32_t>())  { cb::encode_sint(buf, *p); return; }
    if (auto* p = val.get_if<int64_t>())  { cb::encode_sint(buf, *p); return; }

    // unsigned integers
    if (auto* p = val.get_if<uint8_t>())  { cb::encode_uint(buf, *p); return; }
    if (auto* p = val.get_if<uint16_t>()) { cb::encode_uint(buf, *p); return; }
    if (auto* p = val.get_if<uint32_t>()) { cb::encode_uint(buf, *p); return; }
    if (auto* p = val.get_if<uint64_t>()) { cb::encode_uint(buf, *p); return; }

    // floats
    if (auto* p = val.get_if<float>())    { cb::encode_float64(buf, static_cast<double>(*p)); return; }
    if (auto* p = val.get_if<double>())   { cb::encode_float64(buf, *p); return; }

    // bool
    if (auto* p = val.get_if<bool>())     { cb::encode_bool(buf, *p); return; }

    // string (pmr::string is the native storage type)
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

    // byte vector (raw binary data for energy arrays etc.)
    // pmt::Value doesn't directly hold vector<uint8_t>, but Tensor<uint8_t>
    // can be used.  For now, skip unrecognised types with a warning.

    std::fprintf(stderr, "[telemetry] unhandled pmt::Value type, skipping\n");
    cb::encode_bool(buf, false);  // placeholder to keep CBOR map count valid
}

}  // namespace detail

/// Encode a property_map as a CBOR map.
/// Adds "ts" field automatically if not present in the map.
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



}  // namespace gr::lora::telemetry

#endif  // GR4_LORA_TELEMETRY_HPP
