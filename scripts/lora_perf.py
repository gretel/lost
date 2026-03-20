#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_perf.py -- lora_scan sweep performance analyzer.

Subscribes to lora_scan CBOR events via UDP and prints per-sweep metrics
and rolling summaries to stdout.  Works with both the streaming pipeline
(scan_spectrum + scan_sweep_end) and legacy pipeline (additional events).

Usage:
    python3 scripts/lora_perf.py [--host HOST] [--port PORT]

Output lines (grep-friendly):
    SWEEP sweep=1 dur=524ms hot=3 det=1 ovf=0
    DET sweep=1 n=1 869.618/SF8/BW62k/r=50.2
    SUMMARY sweeps=10 avg_dur=530ms p50_dur=524ms ...
"""

from __future__ import annotations

import argparse
import atexit
import signal
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field

import cbor2

from lora_common import create_udp_subscriber  # noqa: reportImplicitRelativeImport


@dataclass
class RunningStats:
    """Accumulates cross-sweep statistics."""

    sweep_durations: list[int] = field(default_factory=list)
    hot_counts: list[int] = field(default_factory=list)
    detection_counts: list[int] = field(default_factory=list)
    total_overflows: int = 0
    det_ratios: list[float] = field(default_factory=list)
    det_sfs: dict[int, int] = field(default_factory=dict)  # SF -> count

    def record_detections(self, dets: list[dict]) -> None:
        for d in dets:
            r = d.get("ratio", 0.0)
            if r > 0:
                self.det_ratios.append(r)
            sf = d.get("sf", 0)
            if sf > 0:
                self.det_sfs[sf] = self.det_sfs.get(sf, 0) + 1

    def record_sweep(self, dur_ms: int, hot: int, det: int, ovf: int) -> None:
        self.sweep_durations.append(dur_ms)
        self.hot_counts.append(hot)
        self.detection_counts.append(det)
        self.total_overflows = ovf


def _avg(vals: Sequence[int] | Sequence[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def _p50(vals: Sequence[int] | Sequence[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    mid = len(s) // 2
    return float(s[mid]) if len(s) % 2 else (s[mid - 1] + s[mid]) / 2.0


def emit_sweep(msg: dict) -> str:
    """Format one SWEEP line from a scan_sweep_end event."""
    dur = msg.get("duration_ms", 0)
    hot = msg.get("hot_count", 0)
    det = msg.get("detections", 0)
    ovf = msg.get("overflows", 0)
    sweep_num = msg.get("sweep", 0)
    l1 = msg.get("l1_snapshots", 0)

    return f"SWEEP sweep={sweep_num} dur={dur}ms l1={l1} hot={hot} det={det} ovf={ovf}"


def emit_summary(stats: RunningStats) -> str:
    """Format a SUMMARY line from accumulated statistics."""
    n = len(stats.sweep_durations)
    if n == 0:
        return "SUMMARY sweeps=0"

    avg_dur = int(_avg(stats.sweep_durations))
    min_dur = min(stats.sweep_durations)
    max_dur = max(stats.sweep_durations)
    p50_dur = int(_p50(stats.sweep_durations))
    avg_hot = _avg(stats.hot_counts)
    total_det = sum(stats.detection_counts)

    avg_ratio = _avg(stats.det_ratios) if stats.det_ratios else 0.0
    min_ratio = min(stats.det_ratios) if stats.det_ratios else 0.0
    max_ratio = max(stats.det_ratios) if stats.det_ratios else 0.0
    sf_str = ",".join(f"SF{sf}:{c}" for sf, c in sorted(stats.det_sfs.items()))

    return (
        f"SUMMARY sweeps={n} "
        f"avg_dur={avg_dur}ms min_dur={min_dur}ms max_dur={max_dur}ms p50_dur={p50_dur}ms "
        f"avg_hot={avg_hot:.1f} "
        f"total_det={total_det} total_ovf={stats.total_overflows}"
        f" avg_ratio={avg_ratio:.1f} min_ratio={min_ratio:.1f} max_ratio={max_ratio:.1f}"
        f" sfs={sf_str or '-'}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="lora_scan sweep performance analyzer. Subscribes via UDP.",
    )
    _ = parser.add_argument(
        "--host", default="127.0.0.1", help="lora_scan UDP host (default: 127.0.0.1)"
    )
    _ = parser.add_argument(
        "--port", type=int, default=5557, help="lora_scan UDP port (default: 5557)"
    )
    args = parser.parse_args()

    sock, _sub_msg, _addr = create_udp_subscriber(args.host, args.port)

    stats = RunningStats()
    summary_interval = 10

    def print_final_summary() -> None:
        if stats.sweep_durations:
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

        if msg_type == "scan_spectrum":
            dets = [d for d in msg.get("detections", []) if isinstance(d, dict)]
            if dets:
                stats.record_detections(dets)
                sweep_num = msg.get("sweep", 0)
                det_parts = []
                for d in dets:
                    freq_mhz = d.get("freq", 0) / 1e6
                    sf = d.get("sf", 0)
                    bw_k = d.get("bw", 0) / 1e3
                    ratio = d.get("ratio", 0.0)
                    det_parts.append(
                        f"{freq_mhz:.3f}/SF{sf}/BW{bw_k:.0f}k/r={ratio:.1f}"
                    )
                print(
                    f"DET sweep={sweep_num} n={len(dets)} {' '.join(det_parts)}",
                    flush=True,
                )

        elif msg_type == "scan_sweep_end":
            line = emit_sweep(msg)
            print(line, flush=True)

            stats.record_sweep(
                dur_ms=msg.get("duration_ms", 0),
                hot=msg.get("hot_count", 0),
                det=msg.get("detections", 0),
                ovf=msg.get("overflows", 0),
            )

            if len(stats.sweep_durations) % summary_interval == 0:
                print(emit_summary(stats), flush=True)

        # Legacy event compat (ignored in streaming mode, kept for backward compat)
        elif msg_type == "scan_overflow":
            stats.total_overflows = msg.get("count", stats.total_overflows)


if __name__ == "__main__":
    main()
