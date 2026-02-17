// SPDX-License-Identifier: ISC
//
// ConsoleSink: LoRa frame output with MeshCore protocol parsing.
//
// Thin wrapper around FrameSink that adds MeshCore v1 protocol decoding
// for the text output mode. CBOR mode outputs protocol-agnostic PHY data
// plus a MeshCore-specific sub-map when sync_word matches.
//
// For protocol-agnostic output without MeshCore parsing, use FrameSink
// directly.

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

#include "cbor.hpp"
#include "protocol/meshcore.hpp"

namespace gr::lora {

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
                // ADVERT
                auto name = meshcore::parse_advert_name(mc_payload);
                uint32_t adv_ts = meshcore::read_le32(&mc_payload[32]);
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
            } else if ((pkt.payload_type == 0 || pkt.payload_type == 1
                        || pkt.payload_type == 2 || pkt.payload_type == 8)
                       && mc_payload.size() >= 4) {
                // REQ/RESP/TXT/PATH
                std::printf("  dst=%02X src=%02X mac=%04X cipher=%zu bytes\n",
                            mc_payload[0], mc_payload[1],
                            meshcore::read_le16(&mc_payload[2]),
                            mc_payload.size() - 4);
            } else if (pkt.payload_type == 3 && mc_payload.size() >= 4) {
                // ACK
                uint32_t cksum = meshcore::read_le32(mc_payload.data());
                std::printf("  ACK checksum=%08X\n", cksum);
            } else if (pkt.payload_type == 11 && !mc_payload.empty()) {
                // CTRL
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
                // New frame starting -- flush any incomplete previous frame
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
