// SPDX-License-Identifier: ISC
//
// Minimal CBOR encoder (RFC 8949, subset).
// Supports: unsigned int, text string, byte string, bool, map.
// Encodes to a std::vector<uint8_t> buffer.

#ifndef GR4_LORA_CBOR_HPP
#define GR4_LORA_CBOR_HPP

#include <cstdint>
#include <string>
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

inline void encode_map_begin(std::vector<uint8_t>& buf, uint64_t n_pairs) {
    encode_head(buf, 5, n_pairs);
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

}  // namespace gr::lora::cbor

#endif  // GR4_LORA_CBOR_HPP
