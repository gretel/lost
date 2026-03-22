#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
trx_test.py -- Wideband decoder A/B test harness.

Tests lora_trx decode quality by sending ADVERTs from a companion device
at various SF/BW/frequency configs and collecting decoded frames via
CBOR/UDP.

Usage:
    python3 scripts/trx_test.py [--configs basic|full] [--adverts N] [--gap SECS]

Requires: serial_bridge.py running (TCP 7835), lora_trx running (UDP 5556).

Output: JSON results to tmp/trx_test_<label>_<timestamp>.json
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
    flush_s: float = 15.0


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
    TestConfig("SF8/BW125k mid-band", sf=8, bw_hz=125000, freq_mhz=867.500),
    TestConfig("SF8/BW250k center", sf=8, bw_hz=250000, freq_mhz=866.500),
]


# ---- Result data model ----


@dataclass
class TrxResult:
    config: TestConfig
    frames: int = 0
    crc_ok: int = 0
    crc_fail: int = 0
    best_snr: float = -999.0
    detected_sfs: dict[int, int] = field(default_factory=dict)
    cfo_ints: list[int] = field(default_factory=list)
    frame_details: list[dict] = field(default_factory=list)


# ---- CBOR frame collector (background thread) ----


class FrameCollector:
    """Collects lora_frame CBOR events from UDP in a background thread."""

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
                if isinstance(msg, dict) and msg.get("type") == "lora_frame":
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
    collector: FrameCollector,
) -> TrxResult:
    """Run one test config: reconfigure radio, send ADVERTs, collect frames."""
    result = TrxResult(config=cfg)

    # Reconfigure companion radio
    bw_khz = cfg.bw_hz / 1000.0
    if not companion.set_radio(cfg.freq_mhz, bw_khz, cfg.sf, cr=8):
        print(f"  FAIL: set_radio({cfg.freq_mhz},{bw_khz},{cfg.sf})", flush=True)
        return result

    # Let pipeline settle after radio change
    time.sleep(2.0)

    # Drain stale frames
    collector.drain()

    # Send ADVERTs
    for i in range(cfg.adverts):
        ok = companion.send_advert()
        status = "ok" if ok else "FAIL"
        print(f"  advert {i + 1}/{cfg.adverts}: {status}", flush=True)
        if i < cfg.adverts - 1:
            time.sleep(cfg.gap_s)

    # Wait for decode pipeline to finish
    time.sleep(cfg.flush_s)

    # Collect frames
    events = collector.drain()

    # Filter frames by channel_freq ±200 kHz of TX frequency
    tx_freq_hz = cfg.freq_mhz * 1e6
    freq_tol = 200e3

    for ev in events:
        phy = ev.get("phy", {})
        channel_freq = phy.get("channel_freq", 0.0)

        # Accept frame if channel_freq matches or if no channel_freq (non-wideband)
        if channel_freq > 0 and abs(channel_freq - tx_freq_hz) > freq_tol:
            continue

        crc_ok = ev.get("crc_valid", False)
        snr = phy.get("snr_db", -999.0)
        sf = phy.get("sf", 0)
        cfo_int = phy.get("cfo_int", None)

        result.frames += 1
        if crc_ok:
            result.crc_ok += 1
        else:
            result.crc_fail += 1
        if snr > result.best_snr:
            result.best_snr = snr
        if sf > 0:
            result.detected_sfs[sf] = result.detected_sfs.get(sf, 0) + 1
        if cfo_int is not None:
            result.cfo_ints.append(int(cfo_int))

        result.frame_details.append({
            "seq": ev.get("seq", 0),
            "sf": sf,
            "bw": phy.get("bw", 0),
            "crc_ok": crc_ok,
            "snr_db": snr,
            "channel_freq": channel_freq,
            "decode_bw": phy.get("decode_bw", 0),
            "cfo_int": cfo_int,
            "cfo_frac": phy.get("cfo_frac", None),
            "sfo_hat": phy.get("sfo_hat", None),
        })

    return result


# ---- Output formatting ----


def print_summary(label: str, results: list[TrxResult]) -> None:
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    print(f"\n=== trx_test: {label} ({ts}) ===\n", flush=True)
    print(
        f"{'Config':<25} {'Frames':>6} {'CRC OK':>6} {'CRC Fail':>8} "
        f"{'SF':>7} {'SNR':>6} {'CFO_int':>7}",
        flush=True,
    )
    print("-" * 75, flush=True)
    for r in results:
        sf_str = (
            ",".join(f"SF{sf}:{c}" for sf, c in sorted(r.detected_sfs.items()))
            if r.detected_sfs
            else "-"
        )
        snr_str = f"{r.best_snr:.1f}" if r.best_snr > -999.0 else "-"
        cfo_str = (
            f"{min(r.cfo_ints)}..{max(r.cfo_ints)}"
            if r.cfo_ints
            else "-"
        )
        print(
            f"{r.config.label:<25} {r.frames:>6} {r.crc_ok:>6} {r.crc_fail:>8} "
            f"{sf_str:>7} {snr_str:>6} {cfo_str:>7}",
            flush=True,
        )

    # Overall totals
    total_frames = sum(r.frames for r in results)
    total_ok = sum(r.crc_ok for r in results)
    total_fail = sum(r.crc_fail for r in results)
    print(f"\nTotal: {total_frames} frames, {total_ok} CRC OK, {total_fail} CRC FAIL", flush=True)


def write_results(path: str, label: str, results: list[TrxResult]) -> None:
    doc = {
        "test": "trx_decode",
        "label": label,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "configs": [
            {
                "label": r.config.label,
                "tx_sf": r.config.sf,
                "tx_bw": r.config.bw_hz,
                "tx_freq_mhz": r.config.freq_mhz,
                "frames": r.frames,
                "crc_ok": r.crc_ok,
                "crc_fail": r.crc_fail,
                "best_snr": round(r.best_snr, 1) if r.best_snr > -999.0 else None,
                "detected_sfs": {str(k): v for k, v in sorted(r.detected_sfs.items())},
                "cfo_ints": r.cfo_ints,
                "frame_details": r.frame_details,
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
    parser = argparse.ArgumentParser(description="Wideband decoder A/B test harness")
    parser.add_argument("--bridge-port", type=int, default=7835)
    parser.add_argument("--trx-port", type=int, default=5556)
    parser.add_argument("--adverts", type=int, default=3, help="ADVERTs per config")
    parser.add_argument(
        "--gap", type=float, default=5.0, help="seconds between ADVERTs"
    )
    parser.add_argument("--label", default="trx")
    parser.add_argument("--output", default="", help="JSON output path")
    parser.add_argument("--tx-power", type=int, default=2)
    parser.add_argument("--configs", choices=["basic", "full"], default="basic")
    parser.add_argument(
        "--serial", default=None, help="direct serial port (skip bridge)"
    )
    parser.add_argument(
        "--flush", type=float, default=15.0, help="seconds to wait after last ADVERT"
    )
    args = parser.parse_args()

    # Select config matrix
    configs = BASIC_CONFIGS if args.configs == "basic" else FULL_CONFIGS

    # Apply CLI overrides to all configs
    for cfg in configs:
        cfg.adverts = args.adverts
        cfg.gap_s = args.gap
        cfg.flush_s = args.flush

    # Output path
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output = args.output or f"tmp/trx_test_{args.label}_{ts}.json"

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

    # Subscribe to lora_trx UDP
    sock, _, _ = create_udp_subscriber("127.0.0.1", args.trx_port)
    collector = FrameCollector(sock)
    collector.start()

    # Wait for subscription to register
    time.sleep(2.0)

    # Run each config
    print(
        f"\n--- {args.configs} matrix: {len(configs)} configs, "
        f"{args.adverts} adverts/config, {args.gap}s gap ---\n",
        flush=True,
    )

    results: list[TrxResult] = []
    for i, cfg in enumerate(configs):
        print(
            f"[{i + 1}/{len(configs)}] {cfg.label} "
            f"(SF{cfg.sf}/BW{cfg.bw_hz / 1000:.0f}k @ {cfg.freq_mhz:.3f} MHz)",
            flush=True,
        )
        result = run_config(cfg, companion, collector)
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
