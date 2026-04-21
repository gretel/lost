// SPDX-License-Identifier: ISC
/// Exception-safety tests for apps/tx_worker.cpp::process_tx_request.
///
/// Verifies that malformed CBOR payloads do not escape the worker
/// lambda and that a NACK with error="internal" is emitted to the
/// sender in each case.
///
/// UdpState::sendTo is non-virtual, so instead of a fake override the
/// tests run process_tx_request with a real loopback UDP socket pair:
/// `udp.fd` transmits on one end, the test reads back the NACK on the
/// other (`_rxfd`).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/ut.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>

#include "../apps/tx_worker.hpp"

using namespace boost::ut;
using namespace std::chrono_literals;

namespace {

/// Pair of loopback UDP sockets used as the test fixture.
///
///   udp.fd  --(sendto sender)-->  _rxfd (bound to an ephemeral port)
///
/// The test can then `::recv(_rxfd, ...)` to observe the NACK bytes.
struct LoopbackUdp {
    UdpState                udp{};
    int                     _rxfd{-1};
    struct sockaddr_storage _rx_addr{};

    LoopbackUdp() {
        udp.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        _rxfd  = ::socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in bind_any{};
        bind_any.sin_family      = AF_INET;
        bind_any.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bind_any.sin_port        = 0; // ephemeral
        if (::bind(_rxfd, reinterpret_cast<sockaddr*>(&bind_any), sizeof(bind_any)) < 0) {
            ::close(_rxfd);
            _rxfd = -1;
            return;
        }

        // Capture the bound address so process_tx_request's sendTo
        // lands the NACK in our receiver socket.
        auto*     rx  = reinterpret_cast<sockaddr_in*>(&_rx_addr);
        socklen_t len = sizeof(*rx);
        ::getsockname(_rxfd, reinterpret_cast<sockaddr*>(rx), &len);
        rx->sin_family = AF_INET;
    }

    ~LoopbackUdp() {
        if (udp.fd >= 0) {
            ::close(udp.fd);
        }
        if (_rxfd >= 0) {
            ::close(_rxfd);
        }
    }

    /// Address process_tx_request should NACK to.
    [[nodiscard]] const struct sockaddr_storage& sender() const { return _rx_addr; }

    /// Drain one datagram with a short timeout. Returns empty on timeout.
    std::vector<uint8_t> recv(std::chrono::milliseconds timeout = 500ms) {
        auto                      deadline = std::chrono::steady_clock::now() + timeout;
        std::array<uint8_t, 4096> buf{};
        while (std::chrono::steady_clock::now() < deadline) {
            auto n = ::recv(_rxfd, buf.data(), buf.size(), MSG_DONTWAIT);
            if (n > 0) {
                return std::vector<uint8_t>(buf.begin(), buf.begin() + n);
            }
            std::this_thread::sleep_for(2ms);
        }
        return {};
    }
};

lora_config::TrxConfig makeCfg() {
    lora_config::TrxConfig cfg{};
    cfg.lbt      = false; // skip LBT wait loop
    cfg.sf       = 8;
    cfg.bw       = 125'000u;
    cfg.rate     = 250'000.f;
    cfg.cr       = 1;
    cfg.sync     = 0x12;
    cfg.preamble = 8;
    return cfg;
}

std::string nackError(const std::vector<uint8_t>& buf) {
    auto msg = gr::lora::cbor::decode_map(std::span<const uint8_t>(buf.data(), buf.size()));
    return gr::lora::cbor::get_text_or(msg, "error", "");
}

uint64_t nackSeq(const std::vector<uint8_t>& buf) {
    auto msg = gr::lora::cbor::decode_map(std::span<const uint8_t>(buf.data(), buf.size()));
    return gr::lora::cbor::get_uint_or(msg, "seq", 0);
}

} // namespace

// ---------------------------------------------------------------------------
const boost::ut::suite<"tx_worker exception safety"> txWorkerExceptionSafety = [] {
    "payload typed as uint emits internal NACK"_test = [] {
        LoopbackUdp             udp;
        auto                    cfg = makeCfg();
        std::atomic<bool>       busy{false};
        gr::lora::TxQueueSource tx_source;

        gr::lora::cbor::Map msg;
        msg["type"]    = std::string("lora_tx");
        msg["seq"]     = uint64_t{42};
        msg["payload"] = uint64_t{0x1234}; // WRONG TYPE -> DecodeError
        TxRequest req{msg, udp.sender()};

        expect(nothrow([&] { process_tx_request(req, cfg, udp.udp, tx_source, busy, nullptr); }));

        auto nack = udp.recv();
        expect(!nack.empty()) << "expected NACK datagram";
        expect(eq(nackError(nack), std::string("internal")));
        expect(eq(nackSeq(nack), 42_u));
    };

    "missing payload key emits internal NACK"_test = [] {
        LoopbackUdp             udp;
        auto                    cfg = makeCfg();
        std::atomic<bool>       busy{false};
        gr::lora::TxQueueSource tx_source;

        gr::lora::cbor::Map msg;
        msg["type"] = std::string("lora_tx");
        msg["seq"]  = uint64_t{43};
        TxRequest req{msg, udp.sender()};

        expect(nothrow([&] { process_tx_request(req, cfg, udp.udp, tx_source, busy, nullptr); }));

        auto nack = udp.recv();
        expect(!nack.empty()) << "expected NACK datagram";
        expect(eq(nackError(nack), std::string("internal")));
        expect(eq(nackSeq(nack), 43_u));
    };

    "seq typed as text yields seq=0 fallback"_test = [] {
        LoopbackUdp             udp;
        auto                    cfg = makeCfg();
        std::atomic<bool>       busy{false};
        gr::lora::TxQueueSource tx_source;

        gr::lora::cbor::Map msg;
        msg["type"] = std::string("lora_tx");
        msg["seq"]  = std::string("forty-four"); // WRONG TYPE
        TxRequest req{msg, udp.sender()};

        expect(nothrow([&] { process_tx_request(req, cfg, udp.udp, tx_source, busy, nullptr); }));

        auto nack = udp.recv();
        expect(!nack.empty()) << "expected NACK datagram";
        expect(eq(nackError(nack), std::string("internal")));
        expect(eq(nackSeq(nack), 0_u)) << "defensive seq extract should fall back to 0";
    };
};

int main() { /* boost::ut auto-runs all suites */ }
