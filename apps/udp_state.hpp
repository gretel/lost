// SPDX-License-Identifier: ISC
//
// Shared UDP state for lora_trx and lora_scan.
// Client management, broadcast, subscribe, and socket setup.

#ifndef GR4_LORA_UDP_STATE_HPP
#define GR4_LORA_UDP_STATE_HPP

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <gnuradio-4.0/lora/log.hpp>

namespace lora_apps {

static constexpr uint32_t kMaxSendFailures = 10;

struct ClientEntry {
    struct sockaddr_storage addr{};
    std::vector<uint16_t>  sync_words{};   // empty = no filter (receive all)
    uint32_t               send_failures{0};

    [[nodiscard]] bool accepts(uint16_t sw) const {
        return sync_words.empty() ||
               std::ranges::find(sync_words, sw) != sync_words.end();
    }
};

struct UdpState {
    int                      fd{-1};
    std::vector<ClientEntry> clients;
    std::mutex               mutex;

    void broadcast(const std::vector<uint8_t>& buf, uint16_t sync_word) {
        sendToAll(buf, &sync_word);
    }

    /// Broadcast to ALL clients, ignoring sync_word filters.
    void broadcast_all(const std::vector<uint8_t>& buf) {
        sendToAll(buf, nullptr);
    }

 private:
    void sendToAll(const std::vector<uint8_t>& buf, const uint16_t* filter_sw) {
        std::lock_guard<std::mutex> lock(mutex);
        std::erase_if(clients, [](const auto& c) {
            return c.send_failures >= kMaxSendFailures;
        });
        for (auto& client : clients) {
            if (filter_sw && !client.accepts(*filter_sw)) continue;
            auto len = (client.addr.ss_family == AF_INET6)
                ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
            auto rc = ::sendto(fd, buf.data(), buf.size(), 0,
                     reinterpret_cast<const struct sockaddr*>(&client.addr),
                     static_cast<socklen_t>(len));
            if (rc < 0) {
                ++client.send_failures;
            } else {
                client.send_failures = 0;
            }
        }
    }

 public:
    static bool sockaddr_equal(const struct sockaddr_storage& a,
                               const struct sockaddr_storage& b) {
        if (a.ss_family != b.ss_family) return false;
        if (a.ss_family == AF_INET6) {
            const auto& a6 = reinterpret_cast<const struct sockaddr_in6&>(a);
            const auto& b6 = reinterpret_cast<const struct sockaddr_in6&>(b);
            return a6.sin6_port == b6.sin6_port &&
                   std::memcmp(&a6.sin6_addr, &b6.sin6_addr, 16) == 0;
        }
        const auto& a4 = reinterpret_cast<const struct sockaddr_in&>(a);
        const auto& b4 = reinterpret_cast<const struct sockaddr_in&>(b);
        return a4.sin_port == b4.sin_port &&
               a4.sin_addr.s_addr == b4.sin_addr.s_addr;
    }

    void subscribe(const struct sockaddr_storage& sender,
                   std::vector<uint16_t> sync_words = {}) {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& c : clients) {
            if (sockaddr_equal(c.addr, sender)) {
                c.sync_words = std::move(sync_words);
                c.send_failures = 0;
                return;
            }
        }
        clients.push_back({sender, std::move(sync_words), 0});
        auto& entry = clients.back();
        char addr_str[INET6_ADDRSTRLEN]{};
        uint16_t port = 0;
        if (sender.ss_family == AF_INET6) {
            const auto& s6 = reinterpret_cast<const struct sockaddr_in6&>(sender);
            ::inet_ntop(AF_INET6, &s6.sin6_addr, addr_str, sizeof(addr_str));
            port = ntohs(s6.sin6_port);
        } else {
            const auto& s4 = reinterpret_cast<const struct sockaddr_in&>(sender);
            ::inet_ntop(AF_INET, &s4.sin_addr, addr_str, sizeof(addr_str));
            port = ntohs(s4.sin_port);
        }
        if (entry.sync_words.empty()) {
            gr::lora::log_ts("info ", "udp",
                "client %s:%u subscribed (all, %zu total)",
                addr_str, port, clients.size());
        } else {
            std::string filter;
            for (auto sw : entry.sync_words) {
                if (!filter.empty()) filter += ",";
                char tmp[8];
                std::snprintf(tmp, sizeof(tmp), "0x%02x", sw);
                filter += tmp;
            }
            gr::lora::log_ts("info ", "udp",
                "client %s:%u subscribed (sync=%s, %zu total)",
                addr_str, port, filter.c_str(), clients.size());
        }
    }

    void sendTo(const std::vector<uint8_t>& buf, const struct sockaddr_storage& dest) {
        auto len = (dest.ss_family == AF_INET6)
            ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
        ::sendto(fd, buf.data(), buf.size(), 0,
                 reinterpret_cast<const struct sockaddr*>(&dest),
                 static_cast<socklen_t>(len));
    }
};

/// Create, bind, and set non-blocking on a UDP socket.
/// Returns fd >= 0 on success, -1 on error (messages logged to stderr).
inline int create_udp_socket(const std::string& listen_addr, uint16_t port) {
    struct sockaddr_in6 bind6{};
    struct sockaddr_in  bind4{};
    bool use_v6 = false;

    if (::inet_pton(AF_INET6, listen_addr.c_str(), &bind6.sin6_addr) == 1) {
        use_v6 = true;
    } else if (listen_addr == "0.0.0.0") {
        use_v6 = true;  // dual-stack
    } else if (::inet_pton(AF_INET, listen_addr.c_str(), &bind4.sin_addr) == 1) {
        use_v6 = false;
    } else {
        gr::lora::log_ts("error", "udp",
            "invalid listen address '%s'", listen_addr.c_str());
        return -1;
    }

    int fd = ::socket(use_v6 ? AF_INET6 : AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        gr::lora::log_ts("error", "udp", "socket(): %s", std::strerror(errno));
        return -1;
    }

    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (use_v6) {
        int v6only = 0;
        ::setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));
        bind6.sin6_family = AF_INET6;
        bind6.sin6_port = htons(port);
        if (::bind(fd, reinterpret_cast<struct sockaddr*>(&bind6), sizeof(bind6)) < 0) {
            gr::lora::log_ts("error", "udp", "bind([%s]:%u): %s",
                listen_addr.c_str(), port, std::strerror(errno));
            ::close(fd);
            return -1;
        }
    } else {
        bind4.sin_family = AF_INET;
        bind4.sin_port = htons(port);
        if (::bind(fd, reinterpret_cast<struct sockaddr*>(&bind4), sizeof(bind4)) < 0) {
            gr::lora::log_ts("error", "udp", "bind(%s:%u): %s",
                listen_addr.c_str(), port, std::strerror(errno));
            ::close(fd);
            return -1;
        }
    }

    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    gr::lora::log_ts("info ", "udp", "bound %s:%u%s",
        listen_addr.c_str(), port, use_v6 ? " (IPv6 dual-stack)" : "");
    return fd;
}

}  // namespace lora_apps

#endif  // GR4_LORA_UDP_STATE_HPP
