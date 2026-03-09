// SPDX-License-Identifier: ISC
//
// FrameSink: LoRa frame output sink.
//
// Consumes uint8_t payload bytes from DemodDecoder, accumulates
// frames delimited by {pay_len, cr, crc_valid} tags, then outputs them
// as text (default), CBOR stdout (--cbor), or CBOR UDP (--udp).

#ifndef GR4_LORA_FRAME_SINK_HPP
#define GR4_LORA_FRAME_SINK_HPP

#include <arpa/inet.h>
#include <fcntl.h>
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
#include <functional>
#include <mutex>
#include <random>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

#include "cbor.hpp"

namespace gr::lora {

struct FrameSink : gr::Block<FrameSink, gr::NoDefaultTagForwarding> {
    using Description = Doc<"LoRa frame output sink — text display, CBOR UDP broadcast with UUID and payload hash">;

    gr::PortIn<uint8_t> in;

    std::string udp_dest{};
    bool        cbor_stdout{false};
    uint16_t    sync_word{0x12};
    uint8_t     phy_sf{8};
    uint32_t    phy_bw{62500};
    std::string label{};    ///< decode chain label — emitted as "decode_label" in CBOR output

    GR_MAKE_REFLECTABLE(FrameSink, in, udp_dest, cbor_stdout, sync_word, phy_sf, phy_bw, label);

    uint32_t             _pay_len{0};
    uint8_t              _cr{4};
    bool                 _crc_valid{false};
    bool                 _is_downchirp{false};
    double               _snr_db{0.0};
    double               _snr_db_td{-999.0};
    double               _noise_floor_db{-999.0};
    double               _peak_db{-999.0};
    int64_t              _rx_channel{-1};
    bool                 _collecting{false};
    std::vector<uint8_t> _frame;
    uint32_t             _frame_count{0};

    // Per-frame unique ID generation (UUID v4, seeded in start())
    std::mt19937_64      _rng{};

    std::function<void(const std::vector<uint8_t>&, bool crc_valid)> _frame_callback{};

    int                              _udp_fd{-1};
    std::vector<struct sockaddr_in>  _udp_clients;
    std::mutex                       _udp_mutex;
    std::thread                      _udp_listener;
    bool                             _udp_running{false};

    /// "[host:]port" -> bind address + port number.
    static bool parse_udp_dest(const std::string& spec,
                               std::string& host, uint16_t& port) {
        auto colon = spec.rfind(':');
        if (colon != std::string::npos && colon > 0) {
            host = spec.substr(0, colon);
            port = static_cast<uint16_t>(std::stoul(spec.substr(colon + 1)));
        } else {
            // Port only — bind all interfaces
            host = "0.0.0.0";
            port = static_cast<uint16_t>(std::stoul(spec));
        }
        return port > 0;
    }

    /// FNV-1a 64-bit hash of payload bytes — used by lora_agg for deduplication.
    [[nodiscard]] static uint64_t fnv1a64(const uint8_t* data, std::size_t len) noexcept {
        uint64_t hash = 14695981039346656037ULL;
        for (std::size_t i = 0; i < len; i++) {
            hash ^= data[i];
            hash *= 1099511628211ULL;
        }
        return hash;
    }

    /// Generate a UUID v4 string (RFC 4122) using the instance RNG.
    [[nodiscard]] std::string generateUUID() {
        uint64_t hi = _rng();
        uint64_t lo = _rng();
        // Set version 4 bits in hi word (bits 12-15 of the third group)
        hi = (hi & 0xFFFFFFFFFFFF0FFFULL) | 0x0000000000004000ULL;
        // Set variant bits in lo word (top 2 bits = 10)
        lo = (lo & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;
        char out[37];
        std::snprintf(out, sizeof(out),
            "%08x-%04x-%04x-%04x-%012llx",
            static_cast<unsigned>(hi >> 32),
            static_cast<unsigned>((hi >> 16) & 0xFFFFU),
            static_cast<unsigned>(hi & 0xFFFFU),
            static_cast<unsigned>(lo >> 48),
            static_cast<uint64_t>(lo & 0x0000FFFFFFFFFFFFULL));
        return out;
    }

    void udpListenerLoop() {
        while (_udp_running) {
            struct sockaddr_in sender{};
            socklen_t slen = sizeof(sender);
            char buf[64];
            auto n = ::recvfrom(_udp_fd, buf, sizeof(buf), 0,
                                reinterpret_cast<struct sockaddr*>(&sender),
                                &slen);
            if (n < 0) {
                // EAGAIN/EWOULDBLOCK from non-blocking socket — just loop
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                break;  // real error or fd closed
            }
            std::lock_guard<std::mutex> lock(_udp_mutex);
            auto match = [&](const auto& c) {
                return c.sin_addr.s_addr == sender.sin_addr.s_addr &&
                       c.sin_port == sender.sin_port;
            };
            if (!std::ranges::any_of(_udp_clients, match)) {
                _udp_clients.push_back(sender);
                char addr_str[INET_ADDRSTRLEN];
                ::inet_ntop(AF_INET, &sender.sin_addr,
                            addr_str, sizeof(addr_str));
                std::fprintf(stderr,
                    "  UDP: registered consumer %s:%u (%zu total)\n",
                    addr_str, ntohs(sender.sin_port),
                    _udp_clients.size());
            }
        }
    }

    void start() {
        _rng.seed(std::random_device {}());

        if (udp_dest.empty()) return;

        std::string host;
        uint16_t port{0};
        if (!parse_udp_dest(udp_dest, host, port)) {
            std::fprintf(stderr, "ERROR: invalid udp_dest '%s'\n",
                         udp_dest.c_str());
            return;
        }

        _udp_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (_udp_fd < 0) {
            std::fprintf(stderr, "ERROR: socket(): %s\n", std::strerror(errno));
            return;
        }

        int reuse = 1;
        ::setsockopt(_udp_fd, SOL_SOCKET, SO_REUSEADDR,
                     &reuse, sizeof(reuse));

        struct sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port   = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &bind_addr.sin_addr) != 1) {
            std::fprintf(stderr, "ERROR: invalid UDP bind address '%s'\n",
                         host.c_str());
            ::close(_udp_fd);
            _udp_fd = -1;
            return;
        }

        if (::bind(_udp_fd, reinterpret_cast<struct sockaddr*>(&bind_addr),
                   sizeof(bind_addr)) < 0) {
            std::fprintf(stderr, "ERROR: bind(%s:%u): %s\n",
                         host.c_str(), port, std::strerror(errno));
            ::close(_udp_fd);
            _udp_fd = -1;
            return;
        }

        int flags = ::fcntl(_udp_fd, F_GETFL, 0);
        ::fcntl(_udp_fd, F_SETFL, flags | O_NONBLOCK);

        std::fprintf(stderr, "  UDP server: bound %s:%u "
                     "(consumers register by sending any datagram)\n",
                     host.c_str(), port);

        _udp_running = true;
        _udp_listener = std::thread([this] { udpListenerLoop(); });
    }

    void stop() {
        _udp_running = false;
        if (_udp_fd >= 0) {
            ::close(_udp_fd);
            _udp_fd = -1;
        }
        if (_udp_listener.joinable()) {
            _udp_listener.join();
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

    void printFrameText(const std::string& ts) {
        auto frame_span = std::span<const uint8_t>(_frame.data(), _pay_len);

        const char* crc_str = _crc_valid ? "CRC_OK" : "CRC_FAIL";

        std::printf("[%s] #%u  %u bytes  CR=4/%u  %s  sync=0x%02X  SNR=%.1f dB",
                    ts.c_str(), _frame_count,
                    _pay_len, 4u + _cr, crc_str,
                    sync_word, _snr_db);
        if (_noise_floor_db > -999.0) {
            std::printf("  NF=%.1f dBFS", _noise_floor_db);
        }
        if (_peak_db > -999.0) {
            std::printf("  peak=%.1f dBFS", _peak_db);
        }
        if (_snr_db_td > -999.0) {
            std::printf("  SNR_td=%.1f dB", _snr_db_td);
        }
        if (_rx_channel >= 0) {
            std::printf("  ch=%d", static_cast<int>(_rx_channel));
        }
        if (!label.empty()) {
            std::printf("  [%s]", label.c_str());
        }
        if (_is_downchirp) {
            std::printf("  (downchirp)");
        }
        std::printf("\n");

        std::printf("  Hex: %s\n", to_hex(frame_span).c_str());
        std::printf("  ASCII: %s\n", to_ascii(frame_span).c_str());
        std::printf("\n");
        std::fflush(stdout);
    }

    [[nodiscard]] std::vector<uint8_t> buildFrameCbor(const std::string& ts) {
        std::vector<uint8_t> buf;
        buf.reserve(128 + _pay_len);

        // Unique decode-event ID — used by lora_agg to track candidates
        auto uuid = generateUUID();

        // FNV-1a payload hash — used by lora_agg to group identical payloads
        auto pay_len_actual = std::min(static_cast<std::size_t>(_pay_len), _frame.size());
        uint64_t phash = fnv1a64(_frame.data(), pay_len_actual);

        uint32_t top_fields = 11;  // type, ts, seq, phy, payload, payload_len,
                                   // crc_valid, cr, is_downchirp, id, payload_hash
        if (_rx_channel >= 0) top_fields++;
        if (!label.empty()) top_fields++;

        cbor::encode_map_begin(buf, top_fields);
        cbor::kv_text(buf, "type", "lora_frame");
        cbor::kv_text(buf, "ts", ts);
        cbor::kv_uint(buf, "seq", _frame_count);

        cbor::encode_text(buf, "phy");
        uint32_t phy_fields = 6;
        if (_noise_floor_db > -999.0) phy_fields++;
        if (_peak_db > -999.0) phy_fields++;
        if (_snr_db_td > -999.0) phy_fields++;
        cbor::encode_map_begin(buf, phy_fields);
        cbor::kv_uint(buf, "sf", phy_sf);
        cbor::kv_uint(buf, "bw", phy_bw);
        cbor::kv_uint(buf, "cr", _cr);
        cbor::kv_bool(buf, "crc_valid", _crc_valid);
        cbor::kv_uint(buf, "sync_word", sync_word);
        cbor::kv_float64(buf, "snr_db", _snr_db);
        if (_noise_floor_db > -999.0) {
            cbor::kv_float64(buf, "noise_floor_db", _noise_floor_db);
        }
        if (_peak_db > -999.0) {
            cbor::kv_float64(buf, "peak_db", _peak_db);
        }
        if (_snr_db_td > -999.0) {
            cbor::kv_float64(buf, "snr_db_td", _snr_db_td);
        }

        cbor::kv_bytes(buf, "payload", _frame.data(), pay_len_actual);
        cbor::kv_uint(buf, "payload_len", _pay_len);
        cbor::kv_bool(buf, "crc_valid", _crc_valid);
        cbor::kv_uint(buf, "cr", _cr);
        cbor::kv_bool(buf, "is_downchirp", _is_downchirp);
        if (_rx_channel >= 0) {
            cbor::kv_uint(buf, "rx_channel", static_cast<uint64_t>(_rx_channel));
        }
        if (!label.empty()) {
            cbor::kv_text(buf, "decode_label", label);
        }

        // Unique decode event identifier (UUID v4, RFC 4122)
        cbor::kv_text(buf, "id", uuid);

        // Payload hash for aggregation grouping (FNV-1a 64-bit)
        cbor::kv_uint(buf, "payload_hash", phash);

        return buf;
    }

    void sendFrameUdp(const std::vector<uint8_t>& buf) {
        if (_udp_fd < 0) return;
        std::lock_guard<std::mutex> lock(_udp_mutex);
        for (const auto& client : _udp_clients) {
            ::sendto(_udp_fd, buf.data(), buf.size(), 0,
                     reinterpret_cast<const struct sockaddr*>(&client),
                     sizeof(client));
        }
    }

    static void sendFrameStdout(const std::vector<uint8_t>& buf) {
        std::fwrite(buf.data(), 1, buf.size(), stdout);
        std::fflush(stdout);
    }

    void emitFrame() {
        _frame_count++;
        std::fprintf(stderr, "[FrameSink] frame #%u: %u bytes, CR=4/%u, %s%s\n",
                     _frame_count, _pay_len, 4u + _cr,
                     _crc_valid ? "CRC_OK" : "CRC_FAIL",
                     _is_downchirp ? " (downchirp)" : "");
        auto ts = timestamp_now();

        if (_frame_callback || cbor_stdout || _udp_fd >= 0) {
            auto buf = buildFrameCbor(ts);
            if (_frame_callback) {
                _frame_callback(buf, _crc_valid);
            }
            if (cbor_stdout) {
                sendFrameStdout(buf);
            }
            sendFrameUdp(buf);
        }

        if (!cbor_stdout) {
            printFrameText(ts);
        }
    }

    void processOne(uint8_t byte) {
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (auto it = tag.map.find("pay_len"); it != tag.map.end()) {
                if (_collecting && !_frame.empty()) {
                    _pay_len = static_cast<uint32_t>(_frame.size());
                    emitFrame();
                }

                _pay_len = static_cast<uint32_t>(
                    it->second.value_or<int64_t>(0));
                if (auto it2 = tag.map.find("cr"); it2 != tag.map.end()) {
                    _cr = static_cast<uint8_t>(
                        it2->second.value_or<int64_t>(0));
                }
                if (auto it2 = tag.map.find("crc_valid");
                    it2 != tag.map.end()) {
                    _crc_valid = it2->second.value_or<bool>(false);
                }
                if (auto it2 = tag.map.find("is_downchirp");
                    it2 != tag.map.end()) {
                    _is_downchirp = it2->second.value_or<bool>(false);
                } else {
                    _is_downchirp = false;
                }
                if (auto it2 = tag.map.find("snr_db");
                    it2 != tag.map.end()) {
                    _snr_db = it2->second.value_or<double>(0.0);
                } else {
                    _snr_db = 0.0;
                }
                if (auto it2 = tag.map.find("noise_floor_db");
                    it2 != tag.map.end()) {
                    _noise_floor_db = it2->second.value_or<double>(-999.0);
                } else {
                    _noise_floor_db = -999.0;
                }
                if (auto it2 = tag.map.find("peak_db");
                    it2 != tag.map.end()) {
                    _peak_db = it2->second.value_or<double>(-999.0);
                } else {
                    _peak_db = -999.0;
                }
                if (auto it2 = tag.map.find("snr_db_td");
                    it2 != tag.map.end()) {
                    _snr_db_td = it2->second.value_or<double>(-999.0);
                } else {
                    _snr_db_td = -999.0;
                }
                if (auto it2 = tag.map.find("rx_channel");
                    it2 != tag.map.end()) {
                    _rx_channel = it2->second.value_or<int64_t>(-1);
                } else {
                    _rx_channel = -1;
                }

                _frame.clear();
                _frame.reserve(_pay_len);
                _collecting = true;
            }
        }

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
