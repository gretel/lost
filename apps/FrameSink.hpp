// SPDX-License-Identifier: ISC
//
// FrameSink: protocol-agnostic LoRa frame output sink.
//
// Consumes uint8_t payload bytes from SymbolDemodulator and watches for
// the {pay_len, cr, crc_valid} tag that marks the start of each frame.
// Accumulates pay_len bytes, then outputs the frame.
//
// Output:
//   - Text (hex dump + ASCII) always goes to stdout.
//   - When udp_dest is set ("host:port"), each frame is also sent as a
//     CBOR-encoded UDP datagram for machine consumers (lora_mon.py, etc).
//
// Protocol-agnostic: the CBOR schema contains only PHY-layer metadata.
// A "protocol" hint is derived from the sync word:
//   0x12 -> "meshcore_or_reticulum"
//   0x2B -> "meshtastic"
//   0x34 -> "lorawan"
//   else -> "unknown"

#ifndef GR4_LORA_FRAME_SINK_HPP
#define GR4_LORA_FRAME_SINK_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

#include "cbor.hpp"

namespace gr::lora {

/// Guess the likely protocol from sync word.
[[nodiscard]] inline std::string guess_protocol(uint16_t sync_word) {
    switch (sync_word) {
    case 0x12: return "meshcore_or_reticulum";
    case 0x2B: return "meshtastic";
    case 0x34: return "lorawan";
    default:   return "unknown";
    }
}

struct FrameSink : gr::Block<FrameSink, gr::NoDefaultTagForwarding> {
    gr::PortIn<uint8_t> in;

    // Configuration
    std::string udp_dest{};            ///< "host:port" for UDP CBOR output (empty=off)
    uint16_t    sync_word{0x12};       ///< for protocol hint in output
    uint8_t     phy_sf{8};             ///< SF for metadata
    uint32_t    phy_bw{62500};         ///< BW for metadata

    GR_MAKE_REFLECTABLE(FrameSink, in, udp_dest, sync_word, phy_sf, phy_bw);

    // Frame accumulation state
    uint32_t             _pay_len{0};
    uint8_t              _cr{4};
    bool                 _crc_valid{false};
    bool                 _is_downchirp{false};
    bool                 _collecting{false};
    std::vector<uint8_t> _frame;
    uint32_t             _frame_count{0};

    // UDP socket state
    int                  _udp_fd{-1};
    struct sockaddr_in   _udp_addr{};

    void start() {
        if (udp_dest.empty()) return;

        auto colon = udp_dest.rfind(':');
        if (colon == std::string::npos || colon == 0) {
            std::fprintf(stderr, "ERROR: invalid udp_dest '%s' (expected host:port)\n",
                         udp_dest.c_str());
            return;
        }
        auto host = udp_dest.substr(0, colon);
        auto port = static_cast<uint16_t>(
            std::stoul(udp_dest.substr(colon + 1)));

        _udp_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (_udp_fd < 0) {
            std::fprintf(stderr, "ERROR: socket(): %s\n", std::strerror(errno));
            return;
        }
        std::memset(&_udp_addr, 0, sizeof(_udp_addr));
        _udp_addr.sin_family = AF_INET;
        _udp_addr.sin_port   = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &_udp_addr.sin_addr) != 1) {
            std::fprintf(stderr, "ERROR: invalid UDP host '%s'\n", host.c_str());
            ::close(_udp_fd);
            _udp_fd = -1;
            return;
        }
        std::fprintf(stderr, "  UDP output: %s:%u\n", host.c_str(), port);
    }

    void stop() {
        if (_udp_fd >= 0) {
            ::close(_udp_fd);
            _udp_fd = -1;
        }
    }

    [[nodiscard]] static std::string timestamp_now() {
        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        std::tm tm{};
        localtime_r(&t, &tm);
        return std::format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:03d}Z",
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
        auto ts = timestamp_now();
        auto frame_span = std::span<const uint8_t>(_frame.data(), _pay_len);

        const char* crc_str = _crc_valid ? "CRC_OK" : "CRC_FAIL";
        auto protocol = guess_protocol(sync_word);

        std::printf("[%s] #%u  %u bytes  CR=4/%u  %s  %s%s\n",
                    ts.c_str(), _frame_count,
                    _pay_len, 4 + _cr, crc_str,
                    protocol.c_str(),
                    _is_downchirp ? "  (downchirp)" : "");

        std::printf("  Hex: %s\n", to_hex(frame_span).c_str());
        std::printf("  ASCII: %s\n", to_ascii(frame_span).c_str());
        std::printf("\n");
        std::fflush(stdout);
    }

    void sendFrameUdp() {
        if (_udp_fd < 0) return;

        auto ts = timestamp_now();
        auto protocol = guess_protocol(sync_word);

        std::vector<uint8_t> buf;
        buf.reserve(128 + _pay_len);

        cbor::encode_map_begin(buf, 10);
        cbor::kv_text(buf, "type", "lora_frame");
        cbor::kv_text(buf, "ts", ts);
        cbor::kv_uint(buf, "seq", _frame_count);

        cbor::encode_text(buf, "phy");
        cbor::encode_map_begin(buf, 5);
        cbor::kv_uint(buf, "sf", phy_sf);
        cbor::kv_uint(buf, "bw", phy_bw);
        cbor::kv_uint(buf, "cr", _cr);
        cbor::kv_bool(buf, "crc_valid", _crc_valid);
        cbor::kv_uint(buf, "sync_word", sync_word);

        cbor::kv_bytes(buf, "payload", _frame.data(),
                       std::min(static_cast<std::size_t>(_pay_len),
                                _frame.size()));
        cbor::kv_uint(buf, "payload_len", _pay_len);
        cbor::kv_bool(buf, "crc_valid", _crc_valid);
        cbor::kv_uint(buf, "cr", _cr);
        cbor::kv_text(buf, "protocol", protocol);
        cbor::kv_bool(buf, "is_downchirp", _is_downchirp);

        ::sendto(_udp_fd, buf.data(), buf.size(), 0,
                 reinterpret_cast<const struct sockaddr*>(&_udp_addr),
                 sizeof(_udp_addr));
    }

    void emitFrame() {
        _frame_count++;
        printFrameText();
        sendFrameUdp();
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
                if (auto it2 = tag.map.find("is_downchirp");
                    it2 != tag.map.end()) {
                    _is_downchirp = pmtv::cast<bool>(it2->second);
                } else {
                    _is_downchirp = false;
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

#endif  // GR4_LORA_FRAME_SINK_HPP
