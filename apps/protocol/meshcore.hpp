// SPDX-License-Identifier: ISC
//
// MeshCore v1 protocol parser.
//
// Parses MeshCore packet framing from raw LoRa payloads. Extracted from
// ConsoleSink.hpp to decouple protocol-specific logic from PHY-layer output.
//
// MeshCore v1 packet structure:
//   [header(1)][transport_codes(4, optional)][path_length(1)][path(N)][payload]
//
//   Header byte: 0bVVPPPPRR
//     RR   (bits 0-1): Route Type
//     PPPP (bits 2-5): Payload Type
//     VV   (bits 6-7): Payload Version

#ifndef GR4_LORA_PROTOCOL_MESHCORE_HPP
#define GR4_LORA_PROTOCOL_MESHCORE_HPP

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace gr::lora::meshcore {

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
    std::size_t payload_offset{0};    // offset of application payload in frame
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
        // Invalid path length -- treat rest as payload
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
        return std::string(
            reinterpret_cast<const char*>(&appdata[off]),
            appdata.size() - off);
    }
    return "";
}

/// Describe ADVERT appdata flags.
[[nodiscard]] inline std::string describe_advert_flags(uint8_t flags) {
    std::string result;
    uint8_t node_type = flags & 0x0F;
    if (node_type == 0x01) result = "chat";
    else if (node_type == 0x02) result = "repeater";
    else if (node_type == 0x03) result = "room";
    else if (node_type == 0x04) result = "sensor";
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

}  // namespace gr::lora::meshcore

#endif  // GR4_LORA_PROTOCOL_MESHCORE_HPP
