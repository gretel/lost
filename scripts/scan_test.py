#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
scan_test.py -- Unified scan detection test harness.

Tests lora_scan detection quality by sending ADVERTs from a companion
device at various SF/BW/frequency configs and collecting detection
metrics via CBOR/UDP.

Usage:
    python3 scripts/scan_test.py [--configs basic|full] [--adverts N] [--gap SECS]

Requires: serial_bridge.py running (TCP 7835), lora_scan running (UDP 5557).

Output: JSON results to tmp/scan_test_<label>_<timestamp>.json
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone

import cbor2

from lora_common import CompanionDriver, create_udp_subscriber


# ---- Test matrix configs ----


@dataclass
class TestConfig:
    label: str
    sf: int
    bw_hz: int
    freq_mhz: float
    adverts: int = 3
    gap_s: float = 5.0
    flush_s: float = 10.0


BASIC_CONFIGS = [
    TestConfig("SF8/BW62k baseline", sf=8, bw_hz=62500, freq_mhz=869.618),
    TestConfig("SF12/BW62k low-edge", sf=12, bw_hz=62500, freq_mhz=863.500),
    TestConfig("SF7/BW62k high-edge", sf=7, bw_hz=62500, freq_mhz=870.000),
]

FULL_CONFIGS = [
    TestConfig("SF7/BW62k low-edge", sf=7, bw_hz=62500, freq_mhz=864.000),
    TestConfig("SF8/BW62k baseline", sf=8, bw_hz=62500, freq_mhz=869.618),
    TestConfig("SF9/BW62k center", sf=9, bw_hz=62500, freq_mhz=866.500),
    TestConfig("SF10/BW62k high-edge", sf=10, bw_hz=62500, freq_mhz=870.000),
    TestConfig("SF12/BW62k low-edge", sf=12, bw_hz=62500, freq_mhz=863.500),
    TestConfig("SF7/BW125k LoRaWAN", sf=7, bw_hz=125000, freq_mhz=868.100),
]


# ---- Result data model ----


@dataclass
class ConfigResult:
    config: TestConfig
    detected: bool = False
    det_count: int = 0
    best_ratio: float = 0.0
    best_sf: int = 0
    sweeps: int = 0
    avg_sweep_ms: int = 0
    overflows: int = 0
    detections: list = field(default_factory=list)


# ---- CBOR event collector (background thread) ----


class EventCollector:
    """Collects scan CBOR events from UDP in a background thread."""

    def __init__(self, sock: socket.socket):
        self._sock = sock
        self._events: list[dict] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._stop.clear()
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def drain(self) -> list[dict]:
        with self._lock:
            events = self._events
            self._events = []
            return events

    def _run(self) -> None:
        self._sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)
                msg = cbor2.loads(data)
                if isinstance(msg, dict):
                    with self._lock:
                        self._events.append(msg)
            except TimeoutError:
                continue
            except Exception:
                continue


# ---- Per-config test execution ----


def run_config(
    cfg: TestConfig,
    companion: CompanionDriver,
    collector: EventCollector,
    min_ratio: float,
) -> ConfigResult:
    """Run one test config: reconfigure radio, send ADVERTs, collect results."""
    result = ConfigResult(config=cfg)

    # Reconfigure companion radio
    bw_khz = cfg.bw_hz / 1000.0
    if not companion.set_radio(cfg.freq_mhz, bw_khz, cfg.sf, cr=8):
        print(f"  FAIL: set_radio({cfg.freq_mhz},{bw_khz},{cfg.sf})", flush=True)
        return result

    # Let ring buffer energy settle after radio change
    time.sleep(2.0)

    # Drain stale events
    collector.drain()

    # Send ADVERTs
    for i in range(cfg.adverts):
        ok = companion.send_advert()
        status = "ok" if ok else "FAIL"
        print(f"  advert {i + 1}/{cfg.adverts}: {status}", flush=True)
        if i < cfg.adverts - 1:
            time.sleep(cfg.gap_s)

    # Wait for flush
    time.sleep(cfg.flush_s)

    # Collect events
    events = collector.drain()

    # Parse results
    sweep_durs: list[int] = []
    all_dets: list[dict] = []

    for ev in events:
        ev_type = ev.get("type", "")
        if ev_type == "scan_sweep_end":
            dur = ev.get("duration_ms", 0)
            if dur > 0:
                sweep_durs.append(dur)
            result.overflows += ev.get("overflows", 0)
        elif ev_type == "scan_spectrum":
            dets = ev.get("detections", [])
            for d in dets:
                if isinstance(d, dict) and d.get("ratio", 0) > 0:
                    all_dets.append(d)

    result.sweeps = len(sweep_durs)
    result.avg_sweep_ms = int(sum(sweep_durs) / len(sweep_durs)) if sweep_durs else 0
    result.detections = all_dets
    result.det_count = len(all_dets)

    if all_dets:
        best = max(all_dets, key=lambda d: d.get("ratio", 0))
        result.best_ratio = best.get("ratio", 0.0)
        result.best_sf = best.get("sf", 0)
        result.detected = result.best_ratio >= min_ratio

    return result


# ---- Output formatting ----


def print_summary(label: str, results: list[ConfigResult]) -> None:
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    print(f"\n=== scan_test: {label} ({ts}) ===\n", flush=True)
    print(
        f"{'Config':<25} {'Det?':>4} {'Count':>5} {'Best SF':>7} "
        f"{'Ratio':>8} {'Sweeps':>6} {'Dur':>5}",
        flush=True,
    )
    print("-" * 70, flush=True)
    for r in results:
        det_str = "YES" if r.detected else "NO"
        sf_str = f"SF{r.best_sf}" if r.best_sf > 0 else "-"
        ratio_str = f"{r.best_ratio:.1f}" if r.best_ratio > 0 else "-"
        dur_str = str(r.avg_sweep_ms) if r.avg_sweep_ms > 0 else "-"
        print(
            f"{r.config.label:<25} {det_str:>4} {r.det_count:>5} {sf_str:>7} "
            f"{ratio_str:>8} {r.sweeps:>6} {dur_str:>5}",
            flush=True,
        )
    print(flush=True)


def write_results(path: str, label: str, results: list[ConfigResult]) -> None:
    doc = {
        "test": "scan_detection",
        "label": label,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "configs": [
            {
                "label": r.config.label,
                "tx_sf": r.config.sf,
                "tx_bw": r.config.bw_hz,
                "tx_freq_mhz": r.config.freq_mhz,
                "detected": r.detected,
                "det_count": r.det_count,
                "best_ratio": round(r.best_ratio, 1),
                "best_sf": r.best_sf,
                "sweeps": r.sweeps,
                "avg_sweep_ms": r.avg_sweep_ms,
                "overflows": r.overflows,
                "detections": r.detections,
            }
            for r in results
        ],
    }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
    print(f"results: {path}", flush=True)


# ---- Main ----


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified scan detection test harness")
    parser.add_argument("--bridge-port", type=int, default=7835)
    parser.add_argument("--scan-port", type=int, default=5557)
    parser.add_argument("--adverts", type=int, default=3, help="ADVERTs per config")
    parser.add_argument(
        "--gap", type=float, default=5.0, help="seconds between ADVERTs"
    )
    parser.add_argument("--label", default="scan")
    parser.add_argument("--output", default="", help="JSON output path")
    parser.add_argument("--tx-power", type=int, default=2)
    parser.add_argument("--configs", choices=["basic", "full"], default="basic")
    parser.add_argument(
        "--serial", default=None, help="direct serial port (skip bridge)"
    )
    parser.add_argument("--min-ratio", type=float, default=8.0)
    args = parser.parse_args()

    # Select config matrix
    configs = BASIC_CONFIGS if args.configs == "basic" else FULL_CONFIGS

    # Apply CLI overrides to all configs
    for cfg in configs:
        cfg.adverts = args.adverts
        cfg.gap_s = args.gap

    # Output path
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output = args.output or f"tmp/scan_test_{args.label}_{ts}.json"

    # Companion driver
    companion = CompanionDriver(
        bridge_host="127.0.0.1",
        bridge_port=args.bridge_port,
        serial=args.serial,
    )

    # Verify companion is reachable
    radio = companion.get_radio()
    if not radio:
        print(
            "ERROR: cannot reach companion (is serial_bridge.py running?)",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"companion: {radio}", flush=True)

    # Set TX power
    if not companion.set_tx_power(args.tx_power):
        print(f"WARNING: set_tx_power({args.tx_power}) failed", file=sys.stderr)

    # Subscribe to lora_scan UDP
    sock, _, _ = create_udp_subscriber("127.0.0.1", args.scan_port)
    collector = EventCollector(sock)
    collector.start()

    # Wait for subscription to register
    time.sleep(2.0)

    # Run each config
    print(
        f"\n--- {args.configs} matrix: {len(configs)} configs, "
        f"{args.adverts} adverts/config, {args.gap}s gap ---\n",
        flush=True,
    )

    results: list[ConfigResult] = []
    for i, cfg in enumerate(configs):
        print(
            f"[{i + 1}/{len(configs)}] {cfg.label} "
            f"(SF{cfg.sf}/BW{cfg.bw_hz / 1000:.0f}k @ {cfg.freq_mhz:.3f} MHz)",
            flush=True,
        )
        result = run_config(cfg, companion, collector, args.min_ratio)
        results.append(result)

    collector.stop()
    sock.close()

    # Restore companion to baseline
    companion.set_radio(869.618, 62.5, 8, cr=8)

    # Output
    print_summary(args.label, results)
    write_results(output, args.label, results)


if __name__ == "__main__":
    main()
