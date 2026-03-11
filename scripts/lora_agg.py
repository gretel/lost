#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""lora_agg.py — LoRa frame aggregation service.

Subscribes to one or more lora_trx raw ports (default :5556), deduplicates
frames by payload_hash within a time window, and forwards the best candidate
(CRC_OK preferred, then highest SNR) with diversity metadata to consumers
on :5555.
Also proxies TX requests from consumers to the first upstream transceiver.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import selectors
import signal
import socket
import time
from typing import Any

import cbor2

from lora_common import (
    config_agg_listen,
    config_agg_upstream,
    config_agg_window_ms,
    load_config,
    parse_host_port,
    setup_logging,
)

KEEPALIVE_INTERVAL = 5.0  # seconds
RECV_TIMEOUT = 0.05  # 50ms poll interval for draining groups
CLIENT_EXPIRY = 60.0  # evict consumers with no keepalive after this
UPSTREAM_SILENCE_WARN = 15.0  # warn if no upstream data for this long
UDP_RCVBUF = 512 * 1024  # 512 KB receive buffer for burst resilience
DEDUP_COOLDOWN = 5.0  # suppress duplicate hashes for this long after emit

log = logging.getLogger("gr4.agg")


# ---------------------------------------------------------------------------
# Aggregation data structures
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class Candidate:
    msg: dict[str, Any]  # raw decoded CBOR message
    rx_channel: int
    snr_db: float
    crc_valid: bool
    decode_label: str  # from "decode_label" field
    frame_id: str  # UUID from "id" field
    arrived_at: float  # time.monotonic()


@dataclasses.dataclass
class PendingGroup:
    payload_hash: int
    created_at: float
    candidates: list[Candidate] = dataclasses.field(default_factory=list)
    best_idx: int = 0


def is_better(a: Candidate, b: Candidate) -> bool:
    """Return True if candidate a is better than b (CRC_OK > SNR)."""
    if a.crc_valid != b.crc_valid:
        return a.crc_valid
    return a.snr_db > b.snr_db


def _snr_db(msg: dict[str, Any]) -> float:
    return msg.get("phy", {}).get("snr_db") or -999.0


def extract_candidate(msg: dict[str, Any], arrived_at: float) -> Candidate | None:
    """Extract a Candidate from a raw lora_frame, or None if missing fields."""
    payload_hash = msg.get("payload_hash")
    frame_id = msg.get("id", "")
    if payload_hash is None:
        return None
    return Candidate(
        msg=msg,
        rx_channel=msg.get("rx_channel") or 0,
        snr_db=_snr_db(msg),
        crc_valid=msg.get("crc_valid", False),
        decode_label=msg.get("decode_label", ""),
        frame_id=frame_id,
        arrived_at=arrived_at,
    )


def build_aggregated(group: PendingGroup) -> dict[str, Any]:
    """Build the aggregated CBOR dict from the best candidate + diversity metadata."""
    winner = group.candidates[group.best_idx]
    msg = dict(winner.msg)  # shallow copy

    rx_channels = [c.rx_channel for c in group.candidates]
    snr_values = [c.snr_db for c in group.candidates]
    crc_mask = sum(1 << i for i, c in enumerate(group.candidates) if c.crc_valid)
    source_ids = [c.frame_id for c in group.candidates]
    gap_us = int((group.candidates[-1].arrived_at - group.created_at) * 1e6)

    msg["diversity"] = {
        "n_candidates": len(group.candidates),
        "decoded_channel": winner.rx_channel,
        "rx_channels": rx_channels,
        "snr_db": snr_values,
        "crc_mask": crc_mask,
        "gap_us": max(0, gap_us),
        "source_ids": source_ids,
    }
    return msg


def format_ruling(group: PendingGroup) -> str:
    """One-line ruling summary for log output."""
    winner = group.candidates[group.best_idx]
    msg = winner.msg
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    crc_str = "CRC_OK" if winner.crc_valid else "CRC_FAIL"
    seq = msg.get("seq", 0)
    snr_db = phy.get("snr_db")
    n = len(group.candidates)
    n_ok = sum(1 for c in group.candidates if c.crc_valid)

    parts = [f"FWD #{seq} {len(payload)}B {crc_str}"]
    if snr_db is not None:
        parts.append(f"SNR={snr_db:.1f}dB")
    parts.append(f"ch={winner.rx_channel}")
    if winner.decode_label:
        parts.append(f"[{winner.decode_label}]")
    parts.append(f"hash={group.payload_hash:08x}")

    if n > 1:
        parts.append(f"[best of {n}, {n_ok} CRC_OK]")
    return " ".join(parts)


# ---------------------------------------------------------------------------
# Upstream connection (subscriber to lora_trx raw port)
# ---------------------------------------------------------------------------


def _try_set_rcvbuf(sock: socket.socket, size: int) -> None:
    """Best-effort SO_RCVBUF increase (kernel may cap it)."""
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, size)
    except OSError:
        pass


def _make_upstream_socket(
    host: str, port: int
) -> tuple[socket.socket, tuple[str, int]]:
    """Create a UDP client socket subscribed to lora_trx at host:port."""
    af = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _try_set_rcvbuf(sock, UDP_RCVBUF)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    addr = (host, port)
    sub = cbor2.dumps({"type": "subscribe"})
    try:
        sock.sendto(sub, addr)
    except OSError as e:
        log.warning("initial subscribe to %s:%d failed: %s", host, port, e)
    return sock, addr


# ---------------------------------------------------------------------------
# Consumer-facing server (same model as lora_trx UdpState)
# ---------------------------------------------------------------------------


class ConsumerServer:
    """UDP server that accepts subscriptions and broadcasts frames."""

    def __init__(self, host: str, port: int) -> None:
        af = socket.AF_INET6 if ":" in host else socket.AF_INET
        self.sock = socket.socket(af, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self._clients: dict[tuple[str, int], dict[str, Any]] = {}
        self._max_failures = 10
        self._fail_counts: dict[tuple[str, int], int] = {}
        self._last_seen: dict[tuple[str, int], float] = {}

    def handle_datagram(
        self, data: bytes, addr: tuple[str, int]
    ) -> dict[str, Any] | None:
        """Process incoming datagram. Registers sender. Returns TX msg if applicable."""
        now = time.monotonic()
        self._last_seen[addr] = now

        try:
            msg = cbor2.loads(data)
        except Exception:
            self._clients[addr] = {"sync_words": set()}
            return None

        if not isinstance(msg, dict):
            self._clients[addr] = {"sync_words": set()}
            return None

        msg_type = msg.get("type", "")
        if msg_type == "subscribe":
            sync_words = set(msg.get("sync_word", []))
            is_new = addr not in self._clients
            self._clients[addr] = {"sync_words": sync_words}
            if is_new:
                sw_str = (
                    ", ".join(f"0x{sw:02x}" for sw in sorted(sync_words))
                    if sync_words
                    else "all"
                )
                log.info("consumer connected %s:%d sw=%s", addr[0], addr[1], sw_str)
            return None
        if msg_type == "lora_tx":
            self._clients.setdefault(addr, {"sync_words": set()})
            return msg

        self._clients.setdefault(addr, {"sync_words": set()})
        return None

    def broadcast(self, data: bytes, sync_word: int | None = None) -> None:
        """Broadcast data to all registered consumers (filtered by sync_word)."""
        dead: list[tuple[str, int]] = []
        for addr, info in self._clients.items():
            sw_filter = info.get("sync_words", set())
            if sw_filter and sync_word is not None and sync_word not in sw_filter:
                continue
            try:
                self.sock.sendto(data, addr)
                self._fail_counts[addr] = 0
            except OSError:
                self._fail_counts[addr] = self._fail_counts.get(addr, 0) + 1
                if self._fail_counts[addr] >= self._max_failures:
                    dead.append(addr)
        for addr in dead:
            log.info(
                "evicting consumer %s:%d after %d send failures",
                addr[0],
                addr[1],
                self._max_failures,
            )
            self._clients.pop(addr, None)
            self._fail_counts.pop(addr, None)
            self._last_seen.pop(addr, None)

    def evict_stale(self, max_age: float) -> None:
        """Remove consumers that haven't sent any datagram recently."""
        now = time.monotonic()
        stale = [addr for addr, ts in self._last_seen.items() if (now - ts) >= max_age]
        for addr in stale:
            log.info(
                "evicting stale consumer %s:%d (no activity for %.0fs)",
                addr[0],
                addr[1],
                now - self._last_seen[addr],
            )
            self._clients.pop(addr, None)
            self._fail_counts.pop(addr, None)
            self._last_seen.pop(addr, None)

    @property
    def client_count(self) -> int:
        return len(self._clients)


# ---------------------------------------------------------------------------
# Main aggregation loop
# ---------------------------------------------------------------------------


def run(args: argparse.Namespace, cfg: dict[str, Any]) -> None:
    window_s = config_agg_window_ms(cfg) / 1000.0

    # Parse upstream addresses
    upstreams: list[tuple[str, int]] = []
    for up in args.upstream:
        try:
            upstreams.append(parse_host_port(up, default_port=5556))
        except ValueError as exc:
            log.error("invalid upstream address %r: %s", up, exc)
            raise SystemExit(1) from exc

    # Parse listen address
    try:
        listen_host, listen_port = parse_host_port(args.listen, default_port=5555)
    except ValueError as exc:
        log.error("invalid listen address %r: %s", args.listen, exc)
        raise SystemExit(1) from exc

    # Create upstream sockets
    up_socks: list[socket.socket] = []
    up_addrs: list[tuple[str, int]] = []
    for host, port in upstreams:
        sock, addr = _make_upstream_socket(host, port)
        up_socks.append(sock)
        up_addrs.append(addr)
        log.info("upstream: %s:%d", host, port)

    # Consumer-facing server
    server = ConsumerServer(listen_host, listen_port)
    log.info(
        "listening: %s:%d (window=%dms)",
        listen_host,
        listen_port,
        config_agg_window_ms(cfg),
    )

    # Aggregation state
    pending: dict[int, PendingGroup] = {}
    recently_emitted: dict[int, float] = {}  # hash -> emit timestamp (cooldown)
    last_keepalive = time.monotonic()
    last_evict = last_keepalive
    frames_total = 0
    frames_since_log = 0
    last_status_log = last_keepalive

    # Per-upstream health tracking
    last_upstream_rx: dict[int, float] = {
        i: time.monotonic() for i in range(len(up_socks))
    }
    upstream_warned: dict[int, bool] = {i: False for i in range(len(up_socks))}
    upstream_connected: dict[int, bool] = {i: False for i in range(len(up_socks))}

    # Map socket fd -> upstream index for health tracking
    sock_to_idx: dict[int, int] = {sock.fileno(): i for i, sock in enumerate(up_socks)}

    # Graceful shutdown via SIGTERM
    shutdown = False

    def _handle_signal(signum: int, frame: Any) -> None:
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGTERM, _handle_signal)

    def send_keepalive() -> None:
        sub = cbor2.dumps({"type": "subscribe"})
        for i, (sock, addr) in enumerate(zip(up_socks, up_addrs)):
            try:
                sock.sendto(sub, addr)
            except OSError as e:
                log.warning("keepalive to %s:%d failed: %s", addr[0], addr[1], e)

    def emit_group(group: PendingGroup) -> None:
        nonlocal frames_total, frames_since_log
        log.info(format_ruling(group))
        recently_emitted[group.payload_hash] = time.monotonic()
        msg = build_aggregated(group)
        data = cbor2.dumps(msg)
        sync_word = msg.get("phy", {}).get("sync_word")
        server.broadcast(data, sync_word)
        frames_total += 1
        frames_since_log += 1

    def drain_expired() -> None:
        now = time.monotonic()
        expired = [k for k, g in pending.items() if (now - g.created_at) >= window_s]
        for k in expired:
            emit_group(pending.pop(k))
        # Purge stale cooldown entries
        stale = [h for h, t in recently_emitted.items() if (now - t) >= DEDUP_COOLDOWN]
        for h in stale:
            del recently_emitted[h]

    def handle_upstream_frame(msg: dict[str, Any]) -> None:
        if msg.get("type") != "lora_frame":
            # Pass through status/config messages
            data = cbor2.dumps(msg)
            server.broadcast(data)
            return
        now = time.monotonic()
        cand = extract_candidate(msg, now)
        if cand is None:
            # Old lora_trx without payload_hash — pass through
            data = cbor2.dumps(msg)
            server.broadcast(data)
            return
        h = cand.msg["payload_hash"]
        if h in recently_emitted:
            log.debug(
                "dedup hash=%016x (%.0fms late)", h, (now - recently_emitted[h]) * 1000
            )
            return
        if h not in pending:
            pending[h] = PendingGroup(payload_hash=h, created_at=now)
        group = pending[h]
        group.candidates.append(cand)
        if is_better(cand, group.candidates[group.best_idx]):
            group.best_idx = len(group.candidates) - 1

    def check_upstream_health() -> None:
        now = time.monotonic()
        for i, addr in enumerate(up_addrs):
            silence = now - last_upstream_rx[i]
            if silence >= UPSTREAM_SILENCE_WARN and not upstream_warned[i]:
                log.warning(
                    "no data from upstream %s:%d for %.0fs", addr[0], addr[1], silence
                )
                upstream_warned[i] = True

    # Main loop
    sel = selectors.DefaultSelector()
    for sock in up_socks:
        sel.register(sock, selectors.EVENT_READ)
    sel.register(server.sock, selectors.EVENT_READ)

    try:
        while not shutdown:
            now = time.monotonic()

            # Periodic keepalive (re-subscribe to upstreams)
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                send_keepalive()
                last_keepalive = now
                check_upstream_health()

            # Periodic stale consumer eviction
            if now - last_evict >= CLIENT_EXPIRY:
                server.evict_stale(CLIENT_EXPIRY)
                last_evict = now

            # Periodic status log (every 5 min)
            if now - last_status_log >= 300.0:
                log.info(
                    "status: %d frames total, %d in last 5min, %d consumers, %d pending",
                    frames_total,
                    frames_since_log,
                    server.client_count,
                    len(pending),
                )
                frames_since_log = 0
                last_status_log = now

            drain_expired()

            try:
                events = sel.select(timeout=RECV_TIMEOUT)
            except (InterruptedError, OSError):
                continue

            for key, _ in events:
                assert isinstance(key.fileobj, socket.socket)
                sock = key.fileobj

                if sock is server.sock:
                    try:
                        data, addr = server.sock.recvfrom(65536)
                    except OSError:
                        continue
                    tx_msg = server.handle_datagram(data, addr)
                    if tx_msg is not None and up_addrs:
                        fwd = cbor2.dumps(tx_msg)
                        try:
                            up_socks[0].sendto(fwd, up_addrs[0])
                        except OSError as e:
                            log.warning(
                                "TX proxy to %s:%d failed: %s",
                                up_addrs[0][0],
                                up_addrs[0][1],
                                e,
                            )
                else:
                    try:
                        data, _ = sock.recvfrom(65536)
                    except OSError:
                        continue

                    # Track upstream health
                    idx = sock_to_idx.get(sock.fileno())
                    if idx is not None:
                        last_upstream_rx[idx] = time.monotonic()
                        if not upstream_connected[idx]:
                            log.info(
                                "upstream connected: %s:%d",
                                up_addrs[idx][0],
                                up_addrs[idx][1],
                            )
                            upstream_connected[idx] = True
                        elif upstream_warned[idx]:
                            log.info(
                                "upstream reconnected: %s:%d",
                                up_addrs[idx][0],
                                up_addrs[idx][1],
                            )
                            upstream_warned[idx] = False

                    try:
                        msg = cbor2.loads(data)
                    except Exception:
                        continue
                    if isinstance(msg, dict):
                        msg_type = msg.get("type", "")
                        if msg_type == "lora_tx_ack":
                            server.broadcast(data)
                        else:
                            handle_upstream_frame(msg)

    except KeyboardInterrupt:
        pass
    finally:
        log.info("shutting down (%d frames total)", frames_total)
        # Drain any remaining pending groups
        for group in pending.values():
            emit_group(group)
        pending.clear()
        sel.close()
        for sock in up_socks:
            sock.close()
        server.sock.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    cfg = load_config()

    parser = argparse.ArgumentParser(
        description="LoRa frame aggregation — dedup multi-decoder output",
    )
    default_upstream = config_agg_upstream(cfg)
    default_listen = config_agg_listen(cfg)

    parser.add_argument(
        "--upstream",
        nargs="+",
        default=[default_upstream],
        metavar="HOST:PORT",
        help=f"lora_trx raw port(s) (default: {default_upstream})",
    )
    parser.add_argument(
        "--listen",
        default=default_listen,
        metavar="HOST:PORT",
        help=f"Consumer-facing bind address (default: {default_listen})",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable colored log output",
    )
    args = parser.parse_args()

    setup_logging("gr4.agg", cfg, debug=args.debug, no_color=args.no_color)
    run(args, cfg)


if __name__ == "__main__":
    main()
