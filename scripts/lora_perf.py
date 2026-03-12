#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_perf.py -- lora_scan sweep performance analyzer.

Reads CBOR Sequence events from stdin (piped from lora_scan --cbor) and prints
per-sweep metrics and rolling summaries to stdout.  Designed for observation
via pty_read(pattern=...) during long-running scans.

Usage:
    lora_scan --cbor 2>scan.log | python3 scripts/lora_perf.py

Output lines (grep-friendly):
    SWEEP sweep=1 dur=5832ms ...        # per-sweep metrics
    SUMMARY sweeps=10 avg_dur=5715ms .. # every 10 sweeps + on exit

CBOR event types consumed:
    scan_sweep_start, scan_sweep_end, scan_l2_probe, scan_retune, scan_overflow
"""

from __future__ import annotations

import atexit
import signal
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field

from cbor_stream import read_cbor_seq


@dataclass
class SweepState:
    """Accumulates per-sweep data from CBOR events between start and end."""

    sweep: int = 0
    probe_durations: list[int] = field(default_factory=list)
    probe_freqs: list[float] = field(default_factory=list)
    retune_count: int = 0
    overflow_count: int = 0


@dataclass
class RunningStats:
    """Accumulates cross-sweep statistics."""

    sweep_durations: list[int] = field(default_factory=list)
    probe_counts: list[int] = field(default_factory=list)
    probe_durations: list[int] = field(default_factory=list)
    hot_counts: list[int] = field(default_factory=list)
    detection_counts: list[int] = field(default_factory=list)
    retune_counts: list[int] = field(default_factory=list)
    l2_time_fracs: list[float] = field(default_factory=list)
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

    def record_sweep(
        self,
        dur_ms: int,
        n_probes: int,
        probe_durs: list[int],
        hot: int,
        det: int,
        retunes: int,
    ) -> None:
        self.sweep_durations.append(dur_ms)
        self.probe_counts.append(n_probes)
        self.probe_durations.extend(probe_durs)
        self.hot_counts.append(hot)
        self.detection_counts.append(det)
        self.retune_counts.append(retunes)
        l2_total = sum(probe_durs) if probe_durs else 0
        self.l2_time_fracs.append(l2_total / dur_ms * 100.0 if dur_ms > 0 else 0.0)


def _avg(vals: Sequence[int] | Sequence[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def _p50(vals: Sequence[int] | Sequence[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    mid = len(s) // 2
    return float(s[mid]) if len(s) % 2 else (s[mid - 1] + s[mid]) / 2.0


def emit_sweep(sweep_end: dict, current: SweepState) -> str:
    """Format one SWEEP line from a scan_sweep_end event + accumulated state."""
    dur = sweep_end.get("duration_ms", 0)
    l1 = sweep_end.get("l1_snapshots", 0)
    l2 = sweep_end.get("l2_probes", 0)
    hot = sweep_end.get("hot_count", 0)
    det = sweep_end.get("detections", 0)
    ovf = sweep_end.get("overflows", 0)
    sweep_num = sweep_end.get("sweep", 0)

    probes_str = ",".join(str(d) for d in current.probe_durations)
    freqs_str = (
        ",".join(f"{f / 1e6:.3f}" for f in current.probe_freqs)
        if current.probe_freqs
        else "-"
    )
    avg_p = int(_avg(current.probe_durations)) if current.probe_durations else 0
    max_p = max(current.probe_durations) if current.probe_durations else 0
    l2_total = sum(current.probe_durations)
    l2_pct = l2_total / dur * 100.0 if dur > 0 else 0.0
    l1_est_ms = dur - l2_total - current.retune_count * 5  # retune settle ~5ms each
    l1_pct = l1_est_ms / dur * 100.0 if dur > 0 else 0.0
    retune_pct = current.retune_count * 5 / dur * 100.0 if dur > 0 else 0.0

    return (
        f"SWEEP sweep={sweep_num} dur={dur}ms l1={l1} l2={l2} "
        f"hot={hot} det={det} ovf={ovf} "
        f"probes={probes_str or '-'}ms "
        f"avg_probe={avg_p}ms max_probe={max_p}ms "
        f"retunes={current.retune_count} "
        f"l2_pct={l2_pct:.1f} l1_pct={l1_pct:.1f} retune_pct={retune_pct:.1f} "
        f"hot_freqs={freqs_str}"
    )


def emit_summary(stats: RunningStats) -> str:
    """Format a SUMMARY line from accumulated statistics."""
    n = len(stats.sweep_durations)
    if n == 0:
        return "SUMMARY sweeps=0"

    avg_dur = int(_avg(stats.sweep_durations))
    min_dur = min(stats.sweep_durations)
    max_dur = max(stats.sweep_durations)
    p50_dur = int(_p50(stats.sweep_durations))
    avg_probes = _avg(stats.probe_counts)
    avg_probe_ms = int(_avg(stats.probe_durations)) if stats.probe_durations else 0
    p50_probe_ms = int(_p50(stats.probe_durations)) if stats.probe_durations else 0
    max_probe_ms = max(stats.probe_durations) if stats.probe_durations else 0
    avg_hot = _avg(stats.hot_counts)
    avg_det = _avg(stats.detection_counts)
    avg_l2_pct = _avg(stats.l2_time_fracs)
    avg_retunes = _avg(stats.retune_counts)
    total_det = sum(stats.detection_counts)

    # Detection quality stats
    avg_ratio = _avg(stats.det_ratios) if stats.det_ratios else 0.0
    min_ratio = min(stats.det_ratios) if stats.det_ratios else 0.0
    max_ratio = max(stats.det_ratios) if stats.det_ratios else 0.0
    sf_str = ",".join(f"SF{sf}:{n}" for sf, n in sorted(stats.det_sfs.items()))

    return (
        f"SUMMARY sweeps={n} "
        f"avg_dur={avg_dur}ms min_dur={min_dur}ms max_dur={max_dur}ms p50_dur={p50_dur}ms "
        f"avg_probes={avg_probes:.1f} avg_hot={avg_hot:.1f} "
        f"avg_probe_ms={avg_probe_ms} p50_probe_ms={p50_probe_ms} max_probe_ms={max_probe_ms} "
        f"avg_retunes={avg_retunes:.1f} "
        f"avg_l2_pct={avg_l2_pct:.1f} "
        f"total_det={total_det} total_ovf={stats.total_overflows}"
        f" avg_ratio={avg_ratio:.1f} min_ratio={min_ratio:.1f} max_ratio={max_ratio:.1f}"
        f" sfs={sf_str or '-'}"
    )


def main() -> None:
    stats = RunningStats()
    current = SweepState()
    summary_interval = 10

    def print_final_summary() -> None:
        if stats.sweep_durations:
            print(emit_summary(stats), flush=True)

    atexit.register(print_final_summary)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

    for msg in read_cbor_seq(sys.stdin.buffer):
        if not isinstance(msg, dict):
            continue

        msg_type = msg.get("type", "")

        if msg_type == "scan_sweep_start":
            current = SweepState(sweep=msg.get("sweep", 0))

        elif msg_type == "scan_l2_probe":
            dur = msg.get("duration_ms", 0)
            current.probe_durations.append(dur)
            current.probe_freqs.append(msg.get("freq", 0))

        elif msg_type == "scan_retune":
            current.retune_count += 1

        elif msg_type == "scan_overflow":
            current.overflow_count = msg.get("count", 0)
            stats.total_overflows = current.overflow_count

        elif msg_type == "scan_spectrum":
            dets = [d for d in msg.get("detections", []) if isinstance(d, dict)]
            if dets:
                # scan_spectrum arrives AFTER scan_sweep_end in the CBOR
                # stream, so record detections directly into stats and emit
                # a supplementary DET line (the SWEEP line was already
                # printed without detection detail).
                stats.record_detections(dets)
                sweep_num = msg.get("sweep", current.sweep)
                det_parts = []
                for d in dets:
                    freq_mhz = d.get("freq", 0) / 1e6
                    sf = d.get("sf", 0)
                    bw_k = d.get("bw", 0) / 1e3
                    ratio = d.get("ratio", 0.0)
                    chirp = d.get("chirp", "")
                    det_parts.append(
                        f"{freq_mhz:.3f}/SF{sf}/BW{bw_k:.0f}k/r={ratio:.1f}/{chirp}"
                    )
                print(
                    f"DET sweep={sweep_num} n={len(dets)} {' '.join(det_parts)}",
                    flush=True,
                )

        elif msg_type == "scan_sweep_end":
            line = emit_sweep(msg, current)
            print(line, flush=True)

            stats.record_sweep(
                dur_ms=msg.get("duration_ms", 0),
                n_probes=msg.get("l2_probes", 0),
                probe_durs=current.probe_durations,
                hot=msg.get("hot_count", 0),
                det=msg.get("detections", 0),
                retunes=current.retune_count,
            )

            if len(stats.sweep_durations) % summary_interval == 0:
                print(emit_summary(stats), flush=True)


if __name__ == "__main__":
    main()
