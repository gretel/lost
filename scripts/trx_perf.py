#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
trx_perf.py -- lora_trx real-time frame monitor.

Subscribes to lora_trx CBOR events via UDP and prints one line per decoded
frame plus periodic summaries. Useful for interactive debugging.

Usage:
    python3 scripts/trx_perf.py [--host HOST] [--port PORT]

Output lines (grep-friendly):
    FRAME seq=1 SF8 BW62.5k CRC=OK SNR=12.3 freq=869.618 cfo_int=-2
    SUMMARY frames=10 crc_ok=8 crc_fail=2 avg_snr=10.5
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
from dataclasses import dataclass, field

import cbor2

from lora_common import create_udp_subscriber


@dataclass
class FrameStats:
    """Accumulates cross-frame statistics."""

    frame_count: int = 0
    crc_ok: int = 0
    crc_fail: int = 0
    snr_values: list[float] = field(default_factory=list)
    sf_counts: dict[int, int] = field(default_factory=dict)


def format_bw(bw: float) -> str:
    """Format BW as human-readable (62500 -> '62.5k')."""
    if bw >= 1000:
        val = bw / 1000
        if val == int(val):
            return f"{int(val)}k"
        return f"{val:g}k"
    return str(int(bw))


def emit_frame(msg: dict) -> str:
    """Format one FRAME line from a lora_frame event."""
    phy = msg.get("phy", {})
    seq = msg.get("seq", 0)
    sf = phy.get("sf", "?")
    bw = phy.get("bw", 0)
    crc_ok = msg.get("crc_valid", False)
    snr = phy.get("snr_db", 0.0)
    channel_freq = phy.get("channel_freq", 0.0)
    cfo_int = phy.get("cfo_int", None)
    cfo_frac = phy.get("cfo_frac", None)
    sfo_hat = phy.get("sfo_hat", None)
    decode_bw = phy.get("decode_bw", 0)

    bw_str = f"BW{format_bw(bw)}" if bw else ""
    crc_str = "OK" if crc_ok else "FAIL"
    parts = [f"FRAME seq={seq} SF{sf} {bw_str} CRC={crc_str} SNR={snr:.1f}"]

    if channel_freq > 0:
        parts.append(f"freq={channel_freq / 1e6:.3f}")
    if decode_bw > 0:
        parts.append(f"dec_bw={format_bw(decode_bw)}")
    if cfo_int is not None:
        parts.append(f"cfo_int={int(cfo_int)}")
    if cfo_frac is not None:
        parts.append(f"cfo_frac={cfo_frac:.3f}")
    if sfo_hat is not None:
        parts.append(f"sfo={sfo_hat:.3f}")

    payload = msg.get("payload", b"")
    if payload:
        parts.append(f"len={len(payload)}")

    return " ".join(parts)


def emit_summary(stats: FrameStats) -> str:
    """Format a SUMMARY line from accumulated statistics."""
    n = stats.frame_count
    if n == 0:
        return "SUMMARY frames=0"

    avg_snr = sum(stats.snr_values) / len(stats.snr_values) if stats.snr_values else 0.0
    sf_str = ",".join(f"SF{sf}:{c}" for sf, c in sorted(stats.sf_counts.items()))

    return (
        f"SUMMARY frames={n} crc_ok={stats.crc_ok} crc_fail={stats.crc_fail} "
        f"avg_snr={avg_snr:.1f} sfs={sf_str or '-'}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="lora_trx real-time frame monitor. Subscribes via UDP.",
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="lora_trx UDP host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=5556, help="lora_trx UDP port (default: 5556)"
    )
    args = parser.parse_args()

    sock, _sub_msg, _addr = create_udp_subscriber(args.host, args.port)

    stats = FrameStats()
    summary_interval = 10

    def print_final_summary() -> None:
        if stats.frame_count > 0:
            print(emit_summary(stats), flush=True)

    atexit.register(print_final_summary)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

    while True:
        data, _ = sock.recvfrom(65536)
        try:
            msg = cbor2.loads(data)
        except Exception:
            continue
        if not isinstance(msg, dict):
            continue

        msg_type = msg.get("type", "")
        if msg_type != "lora_frame":
            continue

        phy = msg.get("phy", {})
        crc_ok = msg.get("crc_valid", False)
        snr = phy.get("snr_db", 0.0)
        sf = phy.get("sf", 0)

        stats.frame_count += 1
        if crc_ok:
            stats.crc_ok += 1
        else:
            stats.crc_fail += 1
        stats.snr_values.append(snr)
        if sf > 0:
            stats.sf_counts[sf] = stats.sf_counts.get(sf, 0) + 1

        print(emit_frame(msg), flush=True)

        if stats.frame_count % summary_interval == 0:
            print(emit_summary(stats), flush=True)


if __name__ == "__main__":
    main()
