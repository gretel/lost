// SPDX-License-Identifier: ISC
//
// Minimal CBOR encoder + decoder (RFC 8949, subset).
// Supports: unsigned int, text string, byte string, bool, map.
// Encoder writes to std::vector<uint8_t>.
// Decoder reads from std::span<const uint8_t> into a variant-based map.

#ifndef GR4_LORA_CBOR_HPP
#define GR4_LORA_CBOR_HPP

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace gr::lora::cbor {

inline void encode_head(std::vector<uint8_t>& buf,
                        uint8_t major, uint64_t val) {
    uint8_t mt = static_cast<uint8_t>(major << 5);
    if (val < 24) {
        buf.push_back(static_cast<uint8_t>(mt | val));
    } else if (val <= 0xFF) {
        buf.push_back(mt | 24);
        buf.push_back(static_cast<uint8_t>(val));
    } else if (val <= 0xFFFF) {
        buf.push_back(mt | 25);
        buf.push_back(static_cast<uint8_t>(val >> 8));
        buf.push_back(static_cast<uint8_t>(val));
    } else if (val <= 0xFFFFFFFF) {
        buf.push_back(mt | 26);
        buf.push_back(static_cast<uint8_t>(val >> 24));
        buf.push_back(static_cast<uint8_t>(val >> 16));
        buf.push_back(static_cast<uint8_t>(val >> 8));
        buf.push_back(static_cast<uint8_t>(val));
    } else {
        buf.push_back(mt | 27);
        for (int i = 7; i >= 0; i--) {
            buf.push_back(static_cast<uint8_t>(val >> (8 * i)));
        }
    }
}

inline void encode_uint(std::vector<uint8_t>& buf, uint64_t val) {
    encode_head(buf, 0, val);
}

inline void encode_text(std::vector<uint8_t>& buf, const std::string& s) {
    encode_head(buf, 3, s.size());
    buf.insert(buf.end(), s.begin(), s.end());
}

inline void encode_bytes(std::vector<uint8_t>& buf,
                         const uint8_t* data, std::size_t len) {
    encode_head(buf, 2, len);
    buf.insert(buf.end(), data, data + len);
}

inline void encode_bool(std::vector<uint8_t>& buf, bool val) {
    buf.push_back(val ? 0xF5 : 0xF4);  // true=0xF5, false=0xF4
}

inline void encode_float64(std::vector<uint8_t>& buf, double val) {
    buf.push_back(0xFB);  // major 7, additional 27 = IEEE 754 double
    uint64_t bits{};
    std::memcpy(&bits, &val, 8);
    for (int i = 7; i >= 0; i--) {
        buf.push_back(static_cast<uint8_t>(bits >> (8 * i)));
    }
}

inline void encode_map_begin(std::vector<uint8_t>& buf, uint64_t n_pairs) {
    encode_head(buf, 5, n_pairs);
}

inline void encode_array_begin(std::vector<uint8_t>& buf, uint64_t n_items) {
    encode_head(buf, 4, n_items);
}

// Convenience: encode a key-value pair where key is text
inline void kv_uint(std::vector<uint8_t>& buf,
                    const std::string& key, uint64_t val) {
    encode_text(buf, key);
    encode_uint(buf, val);
}

inline void kv_text(std::vector<uint8_t>& buf,
                    const std::string& key, const std::string& val) {
    encode_text(buf, key);
    encode_text(buf, val);
}

inline void kv_bool(std::vector<uint8_t>& buf,
                    const std::string& key, bool val) {
    encode_text(buf, key);
    encode_bool(buf, val);
}

inline void kv_bytes(std::vector<uint8_t>& buf,
                     const std::string& key,
                     const uint8_t* data, std::size_t len) {
    encode_text(buf, key);
    encode_bytes(buf, data, len);
}

inline void kv_float64(std::vector<uint8_t>& buf,
                       const std::string& key, double val) {
    encode_text(buf, key);
    encode_float64(buf, val);
}

// ============================================================================
// Decoder — reads a CBOR map with text keys from a byte buffer.
// ============================================================================

/// Value types the decoder can extract from CBOR.
using Value = std::variant<uint64_t, std::string, std::vector<uint8_t>, bool,
                           std::vector<uint64_t>, double>;

/// Decoded CBOR map: text key → value.
using Map = std::unordered_map<std::string, Value>;

/// Thrown when the input is malformed or uses unsupported CBOR features.
struct DecodeError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

namespace detail {

/// Read the CBOR "additional information" value (argument) from a head byte.
/// Advances `pos` past the argument bytes consumed.
inline uint64_t read_argument(std::span<const uint8_t> data,
                              std::size_t& pos, uint8_t info) {
    if (info < 24) {
        return info;
    }
    if (info == 24) {
        if (pos >= data.size()) throw DecodeError("unexpected end of input");
        return data[pos++];
    }
    if (info == 25) {
        if (pos + 2 > data.size()) throw DecodeError("unexpected end of input");
        uint64_t v = static_cast<uint64_t>(data[pos]) << 8
                   | static_cast<uint64_t>(data[pos + 1]);
        pos += 2;
        return v;
    }
    if (info == 26) {
        if (pos + 4 > data.size()) throw DecodeError("unexpected end of input");
        uint64_t v = static_cast<uint64_t>(data[pos])     << 24
                   | static_cast<uint64_t>(data[pos + 1]) << 16
                   | static_cast<uint64_t>(data[pos + 2]) << 8
                   | static_cast<uint64_t>(data[pos + 3]);
        pos += 4;
        return v;
    }
    if (info == 27) {
        if (pos + 8 > data.size()) throw DecodeError("unexpected end of input");
        uint64_t v = 0;
        for (int i = 0; i < 8; i++) {
            v = (v << 8) | data[pos++];
        }
        return v;
    }
    throw DecodeError("unsupported additional info: " + std::to_string(info));
}

/// Decode a single CBOR data item into a Value. Advances `pos`.
inline Value decode_item(std::span<const uint8_t> data, std::size_t& pos) {
    if (pos >= data.size()) throw DecodeError("unexpected end of input");
    uint8_t head = data[pos++];
    uint8_t major = static_cast<uint8_t>(head >> 5);
    uint8_t info  = static_cast<uint8_t>(head & 0x1F);

    switch (major) {
    case 0: {  // unsigned integer
        return read_argument(data, pos, info);
    }
    case 2: {  // byte string
        uint64_t len = read_argument(data, pos, info);
        if (pos + len > data.size()) throw DecodeError("byte string overflows input");
        std::vector<uint8_t> bytes(data.begin() + static_cast<std::ptrdiff_t>(pos),
                                   data.begin() + static_cast<std::ptrdiff_t>(pos + len));
        pos += len;
        return bytes;
    }
    case 3: {  // text string
        uint64_t len = read_argument(data, pos, info);
        if (pos + len > data.size()) throw DecodeError("text string overflows input");
        std::string s(reinterpret_cast<const char*>(data.data() + pos), len);
        pos += len;
        return s;
    }
    case 4: {  // array — decode as vector<uint64_t> (only uint elements supported)
        uint64_t count = read_argument(data, pos, info);
        std::vector<uint64_t> arr;
        arr.reserve(count);
        for (uint64_t i = 0; i < count; i++) {
            auto item = decode_item(data, pos);
            auto* v = std::get_if<uint64_t>(&item);
            if (!v) throw DecodeError("array element is not uint");
            arr.push_back(*v);
        }
        return arr;
    }
    case 7: {  // simple values (bool, null, float)
        if (info == 20) return false;
        if (info == 21) return true;
        if (info == 25) {  // half-precision float (IEEE 754)
            if (pos + 2 > data.size()) throw DecodeError("unexpected end of input");
            uint16_t half = static_cast<uint16_t>(data[pos] << 8 | data[pos + 1]);
            pos += 2;
            int exp = (half >> 10) & 0x1F;
            int mant = half & 0x3FF;
            double val;
            if (exp == 0) val = std::ldexp(static_cast<double>(mant), -24);
            else if (exp == 31) val = (mant == 0) ? std::numeric_limits<double>::infinity() : std::numeric_limits<double>::quiet_NaN();
            else val = std::ldexp(static_cast<double>(mant + 1024), exp - 25);
            return (half & 0x8000) ? -val : val;
        }
        if (info == 26) {  // single-precision float
            if (pos + 4 > data.size()) throw DecodeError("unexpected end of input");
            uint32_t bits = 0;
            for (int i = 0; i < 4; i++) bits = (bits << 8) | data[pos++];
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return static_cast<double>(f);
        }
        if (info == 27) {  // double-precision float
            if (pos + 8 > data.size()) throw DecodeError("unexpected end of input");
            uint64_t bits = 0;
            for (int i = 0; i < 8; i++) bits = (bits << 8) | data[pos++];
            double d;
            std::memcpy(&d, &bits, sizeof(d));
            return d;
        }
        throw DecodeError("unsupported simple value: " + std::to_string(info));
    }
    default:
        throw DecodeError("unsupported major type: " + std::to_string(major));
    }
}

}  // namespace detail

/// Decode a CBOR map with text keys from `data`.
/// Returns the decoded key-value pairs. Throws DecodeError on malformed input.
inline Map decode_map(std::span<const uint8_t> data) {
    if (data.empty()) throw DecodeError("empty input");
    std::size_t pos = 0;

    uint8_t head = data[pos++];
    uint8_t major = static_cast<uint8_t>(head >> 5);
    uint8_t info  = static_cast<uint8_t>(head & 0x1F);

    if (major != 5) throw DecodeError("expected CBOR map (major type 5)");

    uint64_t n_pairs = detail::read_argument(data, pos, info);
    Map result;
    result.reserve(n_pairs);

    for (uint64_t i = 0; i < n_pairs; i++) {
        // Key must be a text string
        auto key_val = detail::decode_item(data, pos);
        auto* key_str = std::get_if<std::string>(&key_val);
        if (!key_str) throw DecodeError("map key is not a text string");

        auto value = detail::decode_item(data, pos);
        result.emplace(std::move(*key_str), std::move(value));
    }
    return result;
}

// Typed accessors — return the value or throw if missing/wrong type.

inline std::vector<uint8_t> get_bytes(const Map& m, const std::string& key) {
    auto it = m.find(key);
    if (it == m.end()) throw DecodeError("missing key: " + key);
    auto* v = std::get_if<std::vector<uint8_t>>(&it->second);
    if (!v) throw DecodeError("key '" + key + "' is not bytes");
    return *v;
}

/// Get a uint value, returning `default_val` if the key is absent.
inline uint64_t get_uint_or(const Map& m, const std::string& key,
                            uint64_t default_val) {
    auto it = m.find(key);
    if (it == m.end()) return default_val;
    auto* v = std::get_if<uint64_t>(&it->second);
    if (!v) throw DecodeError("key '" + key + "' is not uint");
    return *v;
}

/// Get a text value, returning `default_val` if the key is absent.
inline std::string get_text_or(const Map& m, const std::string& key,
                               const std::string& default_val) {
    auto it = m.find(key);
    if (it == m.end()) return default_val;
    auto* v = std::get_if<std::string>(&it->second);
    if (!v) throw DecodeError("key '" + key + "' is not text");
    return *v;
}

/// Get a bool value, returning `default_val` if the key is absent.
inline bool get_bool_or(const Map& m, const std::string& key,
                        bool default_val) {
    auto it = m.find(key);
    if (it == m.end()) return default_val;
    auto* v = std::get_if<bool>(&it->second);
    if (!v) throw DecodeError("key '" + key + "' is not bool");
    return *v;
}

/// Get a float64 value, returning `default_val` if the key is absent.
/// Also accepts uint values (converts to double) for convenience.
inline double get_float64_or(const Map& m, const std::string& key,
                             double default_val) {
    auto it = m.find(key);
    if (it == m.end()) return default_val;
    if (auto* d = std::get_if<double>(&it->second)) return *d;
    if (auto* u = std::get_if<uint64_t>(&it->second)) return static_cast<double>(*u);
    throw DecodeError("key '" + key + "' is not float64 or uint");
}

/// Get a uint array value, returning empty vector if absent.
inline std::vector<uint64_t> get_uint_array_or(const Map& m,
                                                const std::string& key) {
    auto it = m.find(key);
    if (it == m.end()) return {};
    auto* v = std::get_if<std::vector<uint64_t>>(&it->second);
    if (!v) throw DecodeError("key '" + key + "' is not a uint array");
    return *v;
}

}  // namespace gr::lora::cbor

#endif  // GR4_LORA_CBOR_HPP
