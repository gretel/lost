// SPDX-License-Identifier: ISC
//
// ConsoleSink: prints decoded LoRa frames to stdout.
//
// Consumes uint8_t payload bytes from SymbolDemodulator and watches for
// the {pay_len, cr, crc_valid} tag that marks the start of each frame.
// Accumulates pay_len bytes, then prints the frame.
//
// Two output modes:
//   "text"  — human-readable console output (default)
//   "cbor"  — concatenated CBOR objects on stdout (for Python: cbor2.CBORDecoder)
//
// MeshCore v1 packet structure (from protocol spec):
//   [header(1)][transport_codes(4, optional)][path_length(1)][path(N)][payload]
//
//   Header byte: 0bVVPPPPRR
//     RR   (bits 0-1): Route Type
//     PPPP (bits 2-5): Payload Type
//     VV   (bits 6-7): Payload Version

#ifndef GR4_LORA_CONSOLE_SINK_HPP
#define GR4_LORA_CONSOLE_SINK_HPP

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

namespace gr::lora {

// ---- Minimal CBOR encoder (RFC 8949, subset) ----
// Supports: unsigned int, text string, byte string, bool, map.
// Encodes to a std::vector<uint8_t> buffer, then write(STDOUT) atomically.
namespace cbor {

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

}  // namespace cbor

// ---- MeshCore protocol constants ----
namespace meshcore {

static constexpr std::array<const char*, 4> kRouteNames = {
    "T_FLOOD", "FLOOD", "DIRECT", "T_DIRECT"
};

static constexpr std::array<const char*, 16> kPayloadNames = {
    "REQ", "RESP", "TXT", "ACK", "ADVERT", "GRP_TXT",
    "GRP_DATA", "ANON_REQ", "PATH", "TRACE", "MULTI", "CTRL",
    "rsv12", "rsv13", "rsv14", "RAW_CUSTOM"
};

/// Parsed MeshCore packet header + framing.
struct PacketInfo {
    uint8_t     route_type{0};
    uint8_t     payload_type{0};
    uint8_t     version{0};
    bool        has_transport{false};  // T_FLOOD or T_DIRECT
    uint16_t    transport_code1{0};
    uint16_t    transport_code2{0};
    uint8_t     path_length{0};
    std::size_t path_offset{0};       // offset of path[] in frame
    std::size_t payload_offset{0};    // offset of MeshCore payload in frame
    std::size_t payload_size{0};
};

/// Read a little-endian uint16.
inline uint16_t read_le16(const uint8_t* p) {
    return static_cast<uint16_t>(
        static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8));
}

/// Read a little-endian uint32.
inline uint32_t read_le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

/// Parse the MeshCore packet framing from a raw LoRa payload.
[[nodiscard]] inline PacketInfo parse_packet(
        std::span<const uint8_t> frame) {
    PacketInfo info{};
    if (frame.empty()) return info;

    uint8_t hdr = frame[0];
    info.route_type   = hdr & 0x03;
    info.payload_type = (hdr >> 2) & 0x0F;
    info.version      = (hdr >> 6) & 0x03;
    info.has_transport = (info.route_type == 0 || info.route_type == 3);

    std::size_t offset = 1;

    // Transport codes (4 bytes) for T_FLOOD and T_DIRECT
    if (info.has_transport) {
        if (offset + 4 > frame.size()) return info;
        info.transport_code1 = read_le16(&frame[offset]);
        info.transport_code2 = read_le16(&frame[offset + 2]);
        offset += 4;
    }

    // Path length + path
    if (offset >= frame.size()) return info;
    info.path_length = frame[offset];
    offset += 1;
    info.path_offset = offset;
    if (info.path_length > 64 || offset + info.path_length > frame.size()) {
        // Invalid path length — treat rest as payload
        info.payload_offset = offset;
        info.payload_size = frame.size() - offset;
        return info;
    }
    offset += info.path_length;

    info.payload_offset = offset;
    info.payload_size = frame.size() > offset ? frame.size() - offset : 0;
    return info;
}

/// Parse ADVERT appdata to extract the node name.
/// ADVERT payload: [pubkey(32)][timestamp(4)][signature(64)][appdata...]
/// Appdata: [flags(1)][lat(4)?][lon(4)?][feat1(2)?][feat2(2)?][name...]
[[nodiscard]] inline std::string parse_advert_name(
        std::span<const uint8_t> payload) {
    constexpr std::size_t kMinAdvert = 32 + 4 + 64;  // pubkey+ts+sig
    if (payload.size() <= kMinAdvert) return "";

    std::span<const uint8_t> appdata = payload.subspan(kMinAdvert);
    if (appdata.empty()) return "";

    uint8_t flags = appdata[0];
    std::size_t off = 1;

    if (flags & 0x10) off += 8;   // lat(4) + lon(4)
    if (flags & 0x20) off += 2;   // feature 1
    if (flags & 0x40) off += 2;   // feature 2

    if ((flags & 0x80) && off < appdata.size()) {
        // Name is the rest of appdata
        return std::string(
            reinterpret_cast<const char*>(&appdata[off]),
            appdata.size() - off);
    }
    return "";
}

/// Describe ADVERT appdata flags.
[[nodiscard]] inline std::string describe_advert_flags(uint8_t flags) {
    std::string result;
    // Node type (lower nibble values)
    uint8_t node_type = flags & 0x0F;
    if (node_type == 0x01) result = "chat";
    else if (node_type == 0x02) result = "repeater";
    else if (node_type == 0x03) result = "room";
    else if (node_type == 0x04) result = "sensor";
    // Optional fields
    if (flags & 0x10) {
        if (!result.empty()) result += ",";
        result += "loc";
    }
    if (flags & 0x80) {
        if (!result.empty()) result += ",";
        result += "name";
    }
    return result;
}

}  // namespace meshcore

struct ConsoleSink : gr::Block<ConsoleSink, gr::NoDefaultTagForwarding> {
    gr::PortIn<uint8_t> in;

    // Output mode: "text" for human-readable, "cbor" for machine-parseable
    std::string output_mode{"text"};

    GR_MAKE_REFLECTABLE(ConsoleSink, in, output_mode);

    // Frame accumulation state
    uint32_t             _pay_len{0};
    uint8_t              _cr{4};
    bool                 _crc_valid{false};
    bool                 _collecting{false};
    std::vector<uint8_t> _frame;
    uint32_t             _frame_count{0};

    [[nodiscard]] static std::string timestamp_now() {
        auto now   = std::chrono::system_clock::now();
        auto t     = std::chrono::system_clock::to_time_t(now);
        auto ms    = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now.time_since_epoch()) % 1000;
        std::tm tm{};
        localtime_r(&t, &tm);
        return std::format("{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}.{:03d}",
                           tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                           tm.tm_hour, tm.tm_min, tm.tm_sec,
                           static_cast<int>(ms.count()));
    }

    [[nodiscard]] static std::string to_hex(std::span<const uint8_t> data) {
        std::string result;
        result.reserve(data.size() * 3);
        for (std::size_t i = 0; i < data.size(); i++) {
            if (i > 0) result += ' ';
            result += std::format("{:02X}", data[i]);
        }
        return result;
    }

    [[nodiscard]] static std::string to_ascii(std::span<const uint8_t> data) {
        std::string result;
        result.reserve(data.size());
        for (auto b : data) {
            result += (b >= 0x20 && b < 0x7F) ? static_cast<char>(b) : '.';
        }
        return result;
    }

    void printFrameText() {
        _frame_count++;
        auto ts = timestamp_now();
        auto frame_span = std::span<const uint8_t>(_frame.data(), _pay_len);

        const char* crc_str = _crc_valid ? "CRC_OK" : "CRC_FAIL";

        // Parse MeshCore packet structure
        auto pkt = meshcore::parse_packet(frame_span);
        const char* route = meshcore::kRouteNames[pkt.route_type];
        const char* ptype = meshcore::kPayloadNames[pkt.payload_type];

        // Header line
        std::printf("[%s] #%u  %u bytes  CR=4/%u  %s\n",
                    ts.c_str(), _frame_count,
                    _pay_len, 4 + _cr, crc_str);
        std::printf("  %s %s v%u (hdr=0x%02X)", route, ptype, pkt.version + 1,
                    _frame[0]);

        // Transport codes
        if (pkt.has_transport) {
            std::printf("  tc=[%04X,%04X]",
                        pkt.transport_code1, pkt.transport_code2);
        }

        // Path
        if (pkt.path_length > 0) {
            std::printf("  path(%u)=[", pkt.path_length);
            for (uint8_t i = 0; i < pkt.path_length; i++) {
                if (i > 0) std::printf(",");
                std::printf("%02X",
                            _frame[pkt.path_offset + i]);
            }
            std::printf("]");
        }
        std::printf("\n");

        // Payload-type-specific details (only for CRC_OK frames)
        if (_crc_valid && pkt.payload_size > 0) {
            auto mc_payload = frame_span.subspan(
                pkt.payload_offset, pkt.payload_size);

            if (pkt.payload_type == 4 && mc_payload.size() > 100) {
                // ADVERT: pubkey(32) + timestamp(4) + signature(64) + appdata
                auto name = meshcore::parse_advert_name(mc_payload);
                if (mc_payload.size() > 100) {
                    uint32_t adv_ts = meshcore::read_le32(
                        &mc_payload[32]);
                    std::printf("  ADVERT: ts=%u", adv_ts);
                    if (mc_payload.size() > 100) {
                        uint8_t flags = mc_payload[100];
                        auto flag_desc = meshcore::describe_advert_flags(flags);
                        if (!flag_desc.empty()) {
                            std::printf("  flags=[%s]", flag_desc.c_str());
                        }
                    }
                    if (!name.empty()) {
                        std::printf("  name=\"%s\"", name.c_str());
                    }
                    std::printf("\n");
                }
            } else if ((pkt.payload_type == 0 || pkt.payload_type == 1
                        || pkt.payload_type == 2 || pkt.payload_type == 8)
                       && mc_payload.size() >= 4) {
                // REQ/RESP/TXT/PATH: dst_hash(1) + src_hash(1) + MAC(2) + cipher
                std::printf("  dst=%02X src=%02X mac=%04X cipher=%zu bytes\n",
                            mc_payload[0], mc_payload[1],
                            meshcore::read_le16(&mc_payload[2]),
                            mc_payload.size() - 4);
            } else if (pkt.payload_type == 3 && mc_payload.size() >= 4) {
                // ACK: checksum(4)
                uint32_t cksum = meshcore::read_le32(mc_payload.data());
                std::printf("  ACK checksum=%08X\n", cksum);
            } else if (pkt.payload_type == 11 && !mc_payload.empty()) {
                // CTRL: flags(1) + data
                uint8_t ctrl_flags = mc_payload[0];
                uint8_t sub_type = (ctrl_flags >> 4) & 0x0F;
                std::printf("  CTRL sub_type=%u data=%zu bytes\n",
                            sub_type, mc_payload.size() - 1);
            }
        }

        // Hex dump
        std::printf("  Hex: %s\n", to_hex(frame_span).c_str());

        // ASCII
        std::printf("  ASCII: %s\n", to_ascii(frame_span).c_str());

        std::printf("\n");
        std::fflush(stdout);
    }

    void emitFrameCbor() {
        _frame_count++;
        auto ts = timestamp_now();
        auto frame_span = std::span<const uint8_t>(_frame.data(), _pay_len);
        auto pkt = meshcore::parse_packet(frame_span);

        // Count map entries
        uint64_t n_pairs = 8;  // timestamp,frame,pay_len,cr,crc_valid,payload,route,type
        std::string mc_node;
        if (pkt.has_transport) n_pairs += 2;  // tc1, tc2
        if (pkt.path_length > 0) n_pairs++;   // path
        n_pairs++;  // version

        if (_crc_valid && pkt.payload_type == 4 && pkt.payload_size > 100) {
            auto mc_payload = frame_span.subspan(
                pkt.payload_offset, pkt.payload_size);
            mc_node = meshcore::parse_advert_name(mc_payload);
            if (!mc_node.empty()) n_pairs++;
        }

        std::vector<uint8_t> buf;
        buf.reserve(128 + _pay_len);

        cbor::encode_map_begin(buf, n_pairs);
        cbor::kv_text(buf, "timestamp", ts);
        cbor::kv_uint(buf, "frame", _frame_count);
        cbor::kv_uint(buf, "pay_len", _pay_len);
        cbor::kv_uint(buf, "cr", _cr);
        cbor::kv_bool(buf, "crc_valid", _crc_valid);
        cbor::kv_bytes(buf, "payload", _frame.data(),
                       std::min(static_cast<std::size_t>(_pay_len),
                                _frame.size()));
        cbor::kv_text(buf, "route",
                      meshcore::kRouteNames[pkt.route_type]);
        cbor::kv_text(buf, "type",
                      meshcore::kPayloadNames[pkt.payload_type]);
        cbor::kv_uint(buf, "version", pkt.version + 1);
        if (pkt.has_transport) {
            cbor::kv_uint(buf, "tc1", pkt.transport_code1);
            cbor::kv_uint(buf, "tc2", pkt.transport_code2);
        }
        if (pkt.path_length > 0) {
            cbor::kv_bytes(buf, "path",
                           &_frame[pkt.path_offset], pkt.path_length);
        }
        if (!mc_node.empty()) {
            cbor::kv_text(buf, "advert_name", mc_node);
        }

        // Atomic write
        ::write(STDOUT_FILENO, buf.data(), buf.size());
    }

    void emitFrame() {
        if (output_mode == "cbor") {
            emitFrameCbor();
        } else {
            printFrameText();
        }
    }

    void processOne(uint8_t byte) noexcept {
        // Check for new frame tag
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (auto it = tag.map.find("pay_len"); it != tag.map.end()) {
                // New frame starting — flush any incomplete previous frame
                if (_collecting && !_frame.empty()) {
                    _pay_len = static_cast<uint32_t>(_frame.size());
                    emitFrame();
                }

                _pay_len = static_cast<uint32_t>(
                    pmtv::cast<int64_t>(it->second));
                if (auto it2 = tag.map.find("cr"); it2 != tag.map.end()) {
                    _cr = static_cast<uint8_t>(
                        pmtv::cast<int64_t>(it2->second));
                }
                if (auto it2 = tag.map.find("crc_valid");
                    it2 != tag.map.end()) {
                    _crc_valid = pmtv::cast<bool>(it2->second);
                }

                _frame.clear();
                _frame.reserve(_pay_len);
                _collecting = true;
            }
        }

        // Accumulate payload bytes
        if (_collecting) {
            if (_frame.size() < _pay_len) {
                _frame.push_back(byte);
            }
            if (_frame.size() >= _pay_len) {
                emitFrame();
                _collecting = false;
                _frame.clear();
            }
        }
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_CONSOLE_SINK_HPP
