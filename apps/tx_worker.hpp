// SPDX-License-Identifier: ISC
//
// tx_worker — TX request processing for lora_trx.
//
// Extracted from lora_trx.cpp so the worker helpers can be unit-tested
// without a real SDR.  The TX worker thread pops TxRequest values from
// TxRequestQueue and calls process_tx_request on each, which handles LBT
// waits, CBOR payload parsing, IQ generation, and ACK/NACK replies.
//
// process_tx_request is the exception boundary for the worker thread:
// it is noexcept and converts any thrown std::exception into a
// lora_tx_ack{ok=false, error="internal"} delivered to the requesting
// sender, so a malformed CBOR payload cannot kill the worker.

#ifndef GR4_LORA_APPS_TX_WORKER_HPP
#define GR4_LORA_APPS_TX_WORKER_HPP

#include <atomic>
#include <cstdint>
#include <string_view>
#include <sys/socket.h>

#include "config.hpp"
#include "udp_state.hpp"

#include <gnuradio-4.0/lora/TxQueueSource.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>

using lora_apps::UdpState;

/// One TX request drained by the worker thread.
struct TxRequest {
    gr::lora::cbor::Map     msg;
    struct sockaddr_storage sender;
};

/// Send a `lora_tx_ack` NACK to the requesting client.
/// Consolidates the error sites: tx_disabled / tx_queue_full /
/// channel_busy / internal.  Never throws.
void send_tx_nack(UdpState& udp, const struct sockaddr_storage& sender, uint64_t seq, std::string_view error) noexcept;

/// Generate IQ for the request, push it to txSource, then ACK the sender.
/// Throws on malformed CBOR (missing / wrong-typed keys); callers that
/// need exception safety must wrap this (see process_tx_request).
void handle_tx_request(const gr::lora::cbor::Map& msg, const lora_config::TrxConfig& cfg, UdpState& udp, const struct sockaddr_storage& sender, gr::lora::TxQueueSource& txSource, gr::lora::SpectrumState* tx_spectrum);

/// Exception boundary for the TX worker thread.  Performs LBT waits,
/// dispatches to handle_tx_request, and converts any std::exception
/// into a lora_tx_ack NACK with error="internal" carrying the request
/// sequence number (0 if the defensive `seq` extract also fails).
/// Never throws.
void process_tx_request(const TxRequest& req, const lora_config::TrxConfig& cfg, UdpState& udp, gr::lora::TxQueueSource& tx_source, std::atomic<bool>& channel_busy, gr::lora::SpectrumState* tx_spectrum) noexcept;

#endif // GR4_LORA_APPS_TX_WORKER_HPP
