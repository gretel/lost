// SPDX-License-Identifier: ISC
/// Tests for UdpState: client management, sync_word filtering, and UDP broadcast.
///
/// Test progression:
///   1-3. ClientEntry::accepts() — empty filter, single match, multiple match
///   4-6. sockaddr_equal() — same address, different port, different IP
///   7.   subscribe + broadcast_all over loopback
///   8.   sync_word filter: broadcast() skips non-matching clients

#include <boost/ut.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "../apps/udp_state.hpp"

using namespace boost::ut;
using namespace lora_apps;

namespace {

/// Build a sockaddr_storage from an IPv4 address string and port.
sockaddr_storage make_v4(const char* ip, uint16_t port) {
    sockaddr_storage ss{};
    auto&            s4 = reinterpret_cast<sockaddr_in&>(ss);
    s4.sin_family       = AF_INET;
    s4.sin_port         = htons(port);
    inet_pton(AF_INET, ip, &s4.sin_addr);
    return ss;
}

/// RAII wrapper so sockets are always closed, even on test failure.
struct ScopedFd {
    int fd{-1};
    explicit ScopedFd(int f) : fd(f) {}
    ~ScopedFd() {
        if (fd >= 0) {
            ::close(fd);
        }
    }
    ScopedFd(const ScopedFd&)            = delete;
    ScopedFd& operator=(const ScopedFd&) = delete;
              operator int() const { return fd; } // NOLINT implicit for readability
};

/// Create a UDP socket bound to 127.0.0.1 on a random port.
/// Returns the fd and fills `out` with the bound address.
int make_loopback_socket(sockaddr_in& out) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port   = 0; // kernel picks a free port
    inet_pton(AF_INET, "127.0.0.1", &out.sin_addr);
    ::bind(fd, reinterpret_cast<sockaddr*>(&out), sizeof(out));
    socklen_t len = sizeof(out);
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&out), &len);
    return fd;
}

} // namespace

// ---------------------------------------------------------------------------
const boost::ut::suite client_entry_tests = [] {
    "accepts all when empty"_test = [] {
        ClientEntry entry{}; // empty sync_words
        expect(entry.accepts(0x12)) << "should accept 0x12";
        expect(entry.accepts(0x34)) << "should accept 0x34";
        expect(entry.accepts(0xFF)) << "should accept 0xFF";
    };

    "accepts matching"_test = [] {
        ClientEntry entry{{}, {0x12}};
        expect(entry.accepts(0x12)) << "should accept 0x12";
        expect(!entry.accepts(0x34)) << "should reject 0x34";
    };

    "accepts multiple"_test = [] {
        ClientEntry entry{{}, {0x12, 0x34}};
        expect(entry.accepts(0x12)) << "should accept 0x12";
        expect(entry.accepts(0x34)) << "should accept 0x34";
        expect(!entry.accepts(0xFF)) << "should reject 0xFF";
    };
};

// ---------------------------------------------------------------------------
const boost::ut::suite sockaddr_equal_tests = [] {
    "same address"_test = [] {
        auto a = make_v4("127.0.0.1", 5000);
        auto b = make_v4("127.0.0.1", 5000);
        expect(UdpState::sockaddr_equal(a, b)) << "identical addresses should be equal";
    };

    "different port"_test = [] {
        auto a = make_v4("127.0.0.1", 5000);
        auto b = make_v4("127.0.0.1", 5001);
        expect(!UdpState::sockaddr_equal(a, b)) << "different ports should not be equal";
    };

    "different addr"_test = [] {
        auto a = make_v4("127.0.0.1", 5000);
        auto b = make_v4("127.0.0.2", 5000);
        expect(!UdpState::sockaddr_equal(a, b)) << "different IPs should not be equal";
    };
};

// ---------------------------------------------------------------------------
const boost::ut::suite broadcast_tests = [] {
    "subscribe + broadcast_all"_test = [] {
        sockaddr_in srv_addr{};
        sockaddr_in cli_addr{};
        ScopedFd    server_fd{make_loopback_socket(srv_addr)};
        ScopedFd    client_fd{make_loopback_socket(cli_addr)};

        // Set receive timeout so test doesn't hang on failure.
        struct timeval tv{0, 100'000}; // 100 ms
        ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Build client address as sockaddr_storage for subscribe().
        sockaddr_storage cli_storage{};
        std::memcpy(&cli_storage, &cli_addr, sizeof(cli_addr));

        UdpState udp;
        udp.fd = server_fd;
        udp.subscribe(cli_storage);
        udp.broadcast_all({0x01, 0x02, 0x03});

        uint8_t buf[64]{};
        auto    n = ::recvfrom(client_fd, buf, sizeof(buf), 0, nullptr, nullptr);
        expect(n == 3_i) << "should receive 3 bytes";
        expect(eq(buf[0], uint8_t{0x01})) << "byte 0";
        expect(eq(buf[1], uint8_t{0x02})) << "byte 1";
        expect(eq(buf[2], uint8_t{0x03})) << "byte 2";
    };

    "sync_word filter"_test = [] {
        sockaddr_in srv_addr{};
        sockaddr_in cli_addr{};
        ScopedFd    server_fd{make_loopback_socket(srv_addr)};
        ScopedFd    client_fd{make_loopback_socket(cli_addr)};

        struct timeval tv{0, 100'000};
        ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        sockaddr_storage cli_storage{};
        std::memcpy(&cli_storage, &cli_addr, sizeof(cli_addr));

        UdpState udp;
        udp.fd = server_fd;
        udp.subscribe(cli_storage, {0x12});

        // Send with non-matching sync_word — client should NOT receive.
        udp.broadcast({0xAA}, 0x34);
        uint8_t buf[64]{};
        auto    n = ::recvfrom(client_fd, buf, sizeof(buf), 0, nullptr, nullptr);
        expect(n < 0_i) << "non-matching sync_word should not be delivered";

        // Send with matching sync_word — client SHOULD receive.
        udp.broadcast({0xBB}, 0x12);
        n = ::recvfrom(client_fd, buf, sizeof(buf), 0, nullptr, nullptr);
        expect(n == 1_i) << "matching sync_word should deliver 1 byte";
        expect(eq(buf[0], uint8_t{0xBB})) << "payload should be 0xBB";
    };
};

int main() {}
