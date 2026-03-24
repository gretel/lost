#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_test.py -- Hardware A/B test harness for gr4-lora.

Runs structured experiments against a LoRa SDR receiver (lora_trx or
lora_scan) using a Heltec V3 companion as the transmitter.  Manages the
full lifecycle: serial_bridge, SDR binary, companion configuration,
packet transmission, event collection, and structured JSON output.

Design principles:
  - One packet per config point (no repeats -- each ADVERT has a unique timestamp)
  - Machine-parseable JSON output with provenance (git SHA, hypothesis, config)
  - Harness manages all processes; do not pre-start serial_bridge or lora_trx

Usage:
    python3 scripts/lora_test.py decode --matrix full --serial /dev/cu.usbserial-0001
    python3 scripts/lora_test.py scan --matrix basic --serial /dev/cu.usbserial-0001
    python3 scripts/lora_test.py decode --matrix dc_edge --label dc-on \\
        --hypothesis "2kHz DC blocker does not degrade SF12" --serial ...
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path

import cbor2

from lora_common import CompanionDriver, create_udp_subscriber

SCRIPT_DIR = Path(__file__).parent

# -- Timing defaults (seconds) -----------------------------------------------

SETTLE_S = 2.0  # wait after radio change for PLL lock
FLUSH_DECODE_S = 5.0  # wait after TX for decode pipeline
FLUSH_SCAN_S = 3.0  # wait after TX for scan sweeps (2-3 sweep cycles)
FLUSH_TX_S = 3.0  # wait after SDR TX for Heltec to receive
BRIDGE_PORT = 7835
TRX_PORT = 5556  # lora_trx UDP port (accepts lora_tx messages)
MIN_RATIO = 8.0  # scan detection threshold

# -- Config point -------------------------------------------------------------


@dataclass
class ConfigPoint:
    """One point in the experiment matrix.  Each gets exactly one TX."""

    sf: int
    bw: int  # Hz
    freq_mhz: float
    tx_power: int = 2  # dBm


# -- Predefined matrices ------------------------------------------------------

MATRICES: dict[str, list[ConfigPoint]] = {
    "basic": [
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618),
        ConfigPoint(sf=12, bw=62500, freq_mhz=863.500),
        ConfigPoint(sf=7, bw=62500, freq_mhz=870.000),
    ],
    "full": [
        ConfigPoint(sf=7, bw=62500, freq_mhz=864.000),
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618),
        ConfigPoint(sf=9, bw=62500, freq_mhz=866.500),
        ConfigPoint(sf=10, bw=62500, freq_mhz=870.000),
        ConfigPoint(sf=12, bw=62500, freq_mhz=863.500),
        ConfigPoint(sf=7, bw=125000, freq_mhz=868.100),
        ConfigPoint(sf=8, bw=125000, freq_mhz=867.500),
        ConfigPoint(sf=8, bw=250000, freq_mhz=866.500),
    ],
    "dc_edge": [
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=7, bw=125000, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=12, bw=62500, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=10, bw=125000, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=9, bw=62500, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=7, bw=250000, freq_mhz=869.618, tx_power=2),
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618, tx_power=-4),
        ConfigPoint(sf=12, bw=125000, freq_mhz=869.618, tx_power=-4),
        ConfigPoint(sf=8, bw=125000, freq_mhz=869.618, tx_power=-9),
        ConfigPoint(sf=10, bw=62500, freq_mhz=869.618, tx_power=-4),
    ],
    "sf_sweep": [
        ConfigPoint(sf=sf, bw=62500, freq_mhz=869.618) for sf in [7, 8, 9, 10, 11, 12]
    ],
    "bw_sweep": [
        ConfigPoint(sf=8, bw=bw, freq_mhz=869.618) for bw in [62500, 125000, 250000]
    ],
    "power_sweep": [
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618, tx_power=p)
        for p in [2, 0, -4, -9, -14]
    ],
}


# -- Result model --------------------------------------------------------------


@dataclass
class PointResult:
    """Result for one config point."""

    config: dict
    tx_ok: bool = False
    # decode mode
    frames: list[dict] = field(default_factory=list)
    crc_ok: int = 0
    crc_fail: int = 0
    best_snr: float | None = None
    detected_sfs: dict[int, int] = field(default_factory=dict)
    # scan mode
    detections: list[dict] = field(default_factory=list)
    det_count: int = 0
    best_ratio: float | None = None
    best_sf: int | None = None
    sweeps: int = 0
    avg_sweep_ms: int = 0
    overflows: int = 0


# -- Process lifecycle ---------------------------------------------------------


def _start_process(cmd: list[str], log_path: str) -> subprocess.Popen:
    os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
    log_fd = open(log_path, "w")
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=log_fd,
        preexec_fn=os.setpgrp,
    )


def _stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass


def _wait_tcp(port: int, timeout: float = 15.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            s.close()
            return True
        except Exception:
            time.sleep(0.5)
    return False


def _wait_udp(port: int, timeout: float = 30.0) -> bool:
    sub_msg = cbor2.dumps({"type": "subscribe"})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            sock.bind(("", 0))
            sock.sendto(sub_msg, ("127.0.0.1", port))
            sock.recvfrom(65536)
            sock.close()
            return True
        except Exception:
            if sock:
                sock.close()
            time.sleep(1.0)
    return False


# -- CBOR event collector (background thread) ----------------------------------


class EventCollector:
    """Collects CBOR events from UDP in a background thread."""

    def __init__(self, sock: socket.socket, event_types: set[str] | None = None):
        self._sock = sock
        self._types = event_types
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
            out = self._events
            self._events = []
            return out

    def _run(self) -> None:
        self._sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)
                msg = cbor2.loads(data)
                if not isinstance(msg, dict):
                    continue
                if self._types and msg.get("type") not in self._types:
                    continue
                with self._lock:
                    self._events.append(msg)
            except TimeoutError:
                continue
            except Exception:
                continue


# -- Helpers -------------------------------------------------------------------

FREQ_TOL = 200e3  # Hz — accept events within ±200 kHz of TX freq


def point_label(p: ConfigPoint) -> str:
    bw_k = p.bw / 1000
    pwr = f" {p.tx_power}dBm" if p.tx_power != 2 else ""
    return f"SF{p.sf}/BW{bw_k:.0f}k@{p.freq_mhz:.3f}{pwr}"


def _info(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def _err(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr, flush=True)


def _git_sha() -> str:
    try:
        r = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return r.stdout.strip() if r.returncode == 0 else "unknown"
    except Exception:
        return "unknown"


# -- Per-point execution -------------------------------------------------------


def run_point(
    mode: str,
    point: ConfigPoint,
    companion: CompanionDriver,
    collector: EventCollector,
) -> PointResult:
    """Run one config point: set radio, TX one ADVERT, collect results."""
    flush_s = FLUSH_DECODE_S if mode == "decode" else FLUSH_SCAN_S
    result = PointResult(config=asdict(point))

    # Configure companion
    bw_khz = point.bw / 1000.0
    if not companion.set_radio(point.freq_mhz, bw_khz, point.sf, cr=8):
        _err(f"set_radio failed for {point_label(point)}")
        return result
    if not companion.set_tx_power(point.tx_power):
        _err(f"set_tx_power({point.tx_power}) failed")

    time.sleep(SETTLE_S)
    collector.drain()  # discard stale events

    # TX one unique ADVERT
    result.tx_ok = companion.send_advert()
    _info(f"  TX {'ok' if result.tx_ok else 'FAIL'}")

    time.sleep(flush_s)
    events = collector.drain()
    tx_freq_hz = point.freq_mhz * 1e6

    if mode == "decode":
        _collect_decode(result, events, tx_freq_hz)
    else:
        _collect_scan(result, events, tx_freq_hz)

    return result


def _collect_decode(result: PointResult, events: list[dict], tx_freq_hz: float) -> None:
    """Extract decode results from lora_frame events."""
    for ev in events:
        if ev.get("type") != "lora_frame":
            continue
        phy = ev.get("phy", {})
        ch_freq = phy.get("channel_freq", 0.0)
        if ch_freq > 0 and abs(ch_freq - tx_freq_hz) > FREQ_TOL:
            continue

        crc_ok = ev.get("crc_valid", False)
        snr = phy.get("snr_db")
        sf = phy.get("sf", 0)

        if crc_ok:
            result.crc_ok += 1
        else:
            result.crc_fail += 1
        if snr is not None and (result.best_snr is None or snr > result.best_snr):
            result.best_snr = round(snr, 1)
        if sf > 0:
            result.detected_sfs[sf] = result.detected_sfs.get(sf, 0) + 1

        result.frames.append(
            {
                "sf": sf,
                "bw": phy.get("bw", 0),
                "crc_ok": crc_ok,
                "snr_db": round(snr, 1) if snr is not None else None,
                "channel_freq": ch_freq,
                "cfo_int": phy.get("cfo_int"),
            }
        )


def _collect_scan(result: PointResult, events: list[dict], tx_freq_hz: float) -> None:
    """Extract scan results from scan_spectrum / scan_sweep_end events."""
    sweep_durs: list[int] = []
    for ev in events:
        t = ev.get("type", "")
        if t == "scan_sweep_end":
            dur = ev.get("duration_ms", 0)
            if dur > 0:
                sweep_durs.append(dur)
            result.overflows += ev.get("overflows", 0)
        elif t == "scan_spectrum":
            for d in ev.get("detections", []):
                if isinstance(d, dict) and d.get("ratio", 0) > 0:
                    if abs(d.get("freq", 0) - tx_freq_hz) < FREQ_TOL:
                        result.detections.append(d)

    result.sweeps = len(sweep_durs)
    result.avg_sweep_ms = int(sum(sweep_durs) / len(sweep_durs)) if sweep_durs else 0
    result.det_count = len(result.detections)

    if result.detections:
        best = max(result.detections, key=lambda d: d.get("ratio", 0))
        result.best_ratio = round(best.get("ratio", 0.0), 1)
        sf_counts: dict[int, int] = {}
        for d in result.detections:
            sf = d.get("sf", 0)
            if sf > 0:
                sf_counts[sf] = sf_counts.get(sf, 0) + 1
        if sf_counts:
            result.best_sf = max(sf_counts, key=lambda s: sf_counts[s])


# -- SDR TX helpers ------------------------------------------------------------


def _get_sdr_pubkey() -> str:
    """Get the SDR identity's public key (64-char hex)."""
    try:
        r = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / "meshcore_tx.py"), "--show-key"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def _sdr_tx_advert(port: int, freq_mhz: float, sf: int, bw: int) -> bool:
    """Send an ADVERT from the SDR via meshcore_tx.py → lora_trx → air."""
    cmd = [
        sys.executable,
        str(SCRIPT_DIR / "meshcore_tx.py"),
        "advert",
        "--name",
        "lora_test",
        "--connect",
        f"127.0.0.1:{port}",
        "--freq",
        str(int(freq_mhz * 1e6)),
        "--sf",
        str(sf),
        "--bw",
        str(bw),
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


def _sdr_tx_msg(
    port: int,
    dest_pubkey: str,
    text: str,
    freq_mhz: float,
    sf: int,
    bw: int,
) -> bool:
    """Send a TXT_MSG from the SDR via meshcore_tx.py → lora_trx → air."""
    cmd = [
        sys.executable,
        str(SCRIPT_DIR / "meshcore_tx.py"),
        "send",
        "--dest",
        dest_pubkey,
        "--route",
        "direct",
        "--connect",
        f"127.0.0.1:{port}",
        "--freq",
        str(int(freq_mhz * 1e6)),
        "--sf",
        str(sf),
        "--bw",
        str(bw),
        text,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


# -- MeshCore companion (direct serial via meshcore_py) ------------------------


class MeshCoreCompanion:
    """Direct connection to Heltec V3 via meshcore_py for TX verification.

    Uses async event subscriptions to monitor incoming RF packets (ADVERTs)
    and messages (TXT_MSG) in real time. Thread-safe drain methods for
    collecting events from the sync test loop.
    """

    def __init__(self):
        self._mc = None
        self._serial_port = ""
        self._adverts: list[dict] = []
        self._messages: list[dict] = []
        self._lock = threading.Lock()
        self._pubkey = ""

    async def connect(self, serial_port: str) -> bool:
        from meshcore import MeshCore
        from meshcore.events import EventType

        self._serial_port = serial_port
        self._mc = await MeshCore.create_serial(serial_port, 115200)
        if self._mc is None:
            return False

        self._pubkey = self._mc.self_info.get("public_key", "")

        # Subscribe to RF packet log (rich data: SNR, RSSI, parsed adverts)
        self._mc.subscribe(EventType.RX_LOG_DATA, self._on_rf_packet)
        # Subscribe to ADVERT push notifications (backup, simpler)
        self._mc.subscribe(EventType.ADVERTISEMENT, self._on_advert_push)
        # Subscribe to incoming private messages
        self._mc.subscribe(EventType.CONTACT_MSG_RECV, self._on_message)
        # Auto-fetch queued messages from device
        await self._mc.start_auto_message_fetching()

        return True

    def _on_rf_packet(self, event) -> None:
        d = event.payload
        if not isinstance(d, dict):
            return
        if d.get("payload_typename") == "ADVERT" or d.get("payload_type") == 4:
            with self._lock:
                self._adverts.append(
                    {
                        "adv_key": d.get("adv_key", ""),
                        "adv_name": d.get("adv_name", ""),
                        "snr": d.get("snr"),
                        "rssi": d.get("rssi"),
                        "source": "rx_log",
                    }
                )

    def _on_advert_push(self, event) -> None:
        d = event.payload
        if not isinstance(d, dict):
            return
        with self._lock:
            self._adverts.append(
                {
                    "adv_key": d.get("public_key", ""),
                    "adv_name": "",
                    "snr": None,
                    "rssi": None,
                    "source": "push",
                }
            )

    def _on_message(self, event) -> None:
        d = event.payload
        if not isinstance(d, dict):
            return
        with self._lock:
            self._messages.append(
                {
                    "text": d.get("text", ""),
                    "pubkey_prefix": d.get("pubkey_prefix", ""),
                    "txt_type": d.get("txt_type", 0),
                }
            )

    @property
    def pubkey(self) -> str:
        return self._pubkey

    async def set_radio(
        self, freq_mhz: float, bw_khz: float, sf: int, cr: int = 8
    ) -> bool:
        from meshcore.events import EventType

        try:
            result = await self._mc.commands.set_radio(
                freq=freq_mhz,  # meshcore_py takes MHz (multiplies by 1000 internally)
                bw=bw_khz,
                sf=sf,
                cr=cr,
            )
            if result.type == EventType.OK:
                return True
            # Heltec reboots after radio param change — reconnect
            _info(f"  set_radio: reconnecting ({result.payload})")
            return await self._reconnect()
        except Exception as e:
            _info(f"  set_radio: reconnecting after exception ({e})")
            return await self._reconnect()

    async def _reconnect(self) -> bool:
        from meshcore import MeshCore
        from meshcore.events import EventType

        try:
            await self._mc.disconnect()
        except Exception:
            pass
        await asyncio.sleep(3.0)  # wait for Heltec reboot
        self._mc = await MeshCore.create_serial(self._serial_port, 115200)
        if self._mc is None:
            _err("reconnect failed")
            return False
        self._mc.subscribe(EventType.RX_LOG_DATA, self._on_rf_packet)
        self._mc.subscribe(EventType.ADVERTISEMENT, self._on_advert_push)
        self._mc.subscribe(EventType.CONTACT_MSG_RECV, self._on_message)
        await self._mc.start_auto_message_fetching()
        _info("  reconnected")
        return True

    def drain_adverts(self) -> list[dict]:
        with self._lock:
            out = self._adverts
            self._adverts = []
            return out

    def drain_messages(self) -> list[dict]:
        with self._lock:
            out = self._messages
            self._messages = []
            return out

    async def close(self) -> None:
        if self._mc:
            await self._mc.disconnect()


async def run_tx_point(
    point: ConfigPoint,
    companion: MeshCoreCompanion,
    sdr_pubkey: str,
) -> PointResult:
    """Run one TX test: SDR sends ADVERT + TXT_MSG, verify Heltec received."""
    result = PointResult(config=asdict(point))

    # Configure Heltec radio to receive at this config
    bw_khz = point.bw / 1000.0
    if not await companion.set_radio(point.freq_mhz, bw_khz, point.sf, cr=8):
        _err(f"set_radio failed for {point_label(point)}")
        return result

    await asyncio.sleep(SETTLE_S)
    companion.drain_adverts()  # discard stale

    # --- ADVERT test ---
    ok = _sdr_tx_advert(TRX_PORT, point.freq_mhz, point.sf, point.bw)
    result.tx_ok = ok
    _info(f"  ADVERT TX {'ok' if ok else 'FAIL'}")
    if not ok:
        return result

    await asyncio.sleep(FLUSH_TX_S)

    adverts = companion.drain_adverts()
    advert_ok = any(a.get("adv_key", "").lower() == sdr_pubkey.lower() for a in adverts)
    result.frames.append(
        {
            "test": "advert_rx",
            "passed": advert_ok,
            "adverts_seen": len(adverts),
            "snr": next(
                (
                    a["snr"]
                    for a in adverts
                    if a.get("adv_key", "").lower() == sdr_pubkey.lower()
                ),
                None,
            ),
        }
    )
    if advert_ok:
        result.crc_ok += 1
        snr = result.frames[-1]["snr"]
        _info(f"  ADVERT RX: OK (SNR={snr})")
    else:
        result.crc_fail += 1
        keys = [a.get("adv_key", "")[:12] for a in adverts]
        _info(f"  ADVERT RX: NOT received (saw {len(adverts)} adverts: {keys})")

    # --- TXT_MSG test ---
    heltec_pubkey = companion.pubkey
    if heltec_pubkey:
        companion.drain_messages()  # discard stale
        ts = int(time.time())
        msg_text = f"test_{ts}"
        ok = _sdr_tx_msg(
            TRX_PORT,
            heltec_pubkey,
            msg_text,
            point.freq_mhz,
            point.sf,
            point.bw,
        )
        _info(f"  TXT_MSG TX {'ok' if ok else 'FAIL'}")
        if ok:
            await asyncio.sleep(FLUSH_TX_S)
            msgs = companion.drain_messages()
            msg_ok = any(msg_text in m.get("text", "") for m in msgs)
            result.frames.append(
                {
                    "test": "txt_msg_rx",
                    "passed": msg_ok,
                    "sent": msg_text,
                    "msgs_seen": len(msgs),
                }
            )
            if msg_ok:
                result.crc_ok += 1
                _info(f"  TXT_MSG RX: OK '{msg_text}'")
            else:
                result.crc_fail += 1
                texts = [m.get("text", "")[:40] for m in msgs]
                _info(f"  TXT_MSG RX: NOT received (saw {len(msgs)} msgs: {texts})")
        else:
            result.crc_fail += 1
    else:
        _info("  TXT_MSG: skipped (no Heltec pubkey)")

    return result


# -- Summary builder -----------------------------------------------------------


def build_summary(mode: str, results: list[PointResult]) -> dict:
    n = len(results)
    tx_ok = sum(1 for r in results if r.tx_ok)

    if mode == "decode":
        total_frames = sum(len(r.frames) for r in results)
        total_crc_ok = sum(r.crc_ok for r in results)
        total_crc_fail = sum(r.crc_fail for r in results)
        pts_decode = sum(1 for r in results if r.frames)
        snrs = [r.best_snr for r in results if r.best_snr is not None]
        return {
            "points": n,
            "tx_ok": tx_ok,
            "points_with_decode": pts_decode,
            "total_frames": total_frames,
            "total_crc_ok": total_crc_ok,
            "total_crc_fail": total_crc_fail,
            "crc_ok_rate": round(total_crc_ok / total_frames, 3) if total_frames else 0,
            "snr_min": min(snrs) if snrs else None,
            "snr_max": max(snrs) if snrs else None,
        }
    elif mode == "scan":
        total_det = sum(r.det_count for r in results)
        pts_det = sum(1 for r in results if r.det_count > 0)
        ratios = [r.best_ratio for r in results if r.best_ratio is not None]
        total_ovf = sum(r.overflows for r in results)
        return {
            "points": n,
            "tx_ok": tx_ok,
            "points_detected": pts_det,
            "total_detections": total_det,
            "total_overflows": total_ovf,
            "ratio_min": min(ratios) if ratios else None,
            "ratio_max": max(ratios) if ratios else None,
        }
    else:  # tx
        total_tests = sum(r.crc_ok + r.crc_fail for r in results)
        total_pass = sum(r.crc_ok for r in results)
        total_fail = sum(r.crc_fail for r in results)
        return {
            "points": n,
            "tx_ok": tx_ok,
            "total_tests": total_tests,
            "total_pass": total_pass,
            "total_fail": total_fail,
            "pass_rate": round(total_pass / total_tests, 3) if total_tests else 0,
        }


# -- Main ----------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hardware A/B test harness for gr4-lora",
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    for name in ("decode", "scan", "tx"):
        p = sub.add_parser(name)
        p.add_argument("--serial", required=True, help="Companion serial port")
        p.add_argument(
            "--matrix",
            choices=list(MATRICES.keys()),
            default="basic",
            help="Config matrix (default: basic)",
        )
        p.add_argument(
            "--config",
            default="apps/config.toml",
            help="TOML config for lora_trx/lora_scan",
        )
        p.add_argument("--hypothesis", default="")
        p.add_argument("--label", default="")

    args = parser.parse_args()

    matrix = list(MATRICES[args.matrix])
    mode = args.mode

    # Resolve binary and UDP port from mode
    if mode == "decode" or mode == "tx":
        binary = "./build/apps/lora_trx"
        udp_port = TRX_PORT
        event_types = {"lora_frame"}
    else:
        binary = "./build/apps/lora_scan"
        udp_port = 5557
        event_types = {"scan_spectrum", "scan_sweep_end"}

    label = args.label or f"{mode}_{args.matrix}"
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join("data", "testing")
    output_path = os.path.join(output_dir, f"lora_test_{label}_{ts}.json")
    os.makedirs(output_dir, exist_ok=True)

    # -- TX mode: separate path (meshcore_py, no serial_bridge) ---
    if mode == "tx":
        asyncio.run(_run_tx_experiment(args, matrix, binary, label, output_path))
        return

    # -- decode/scan: serial_bridge + CompanionDriver ---
    _info(f"Starting serial_bridge on {args.serial}")
    bridge_proc = _start_process(
        [
            sys.executable,
            str(SCRIPT_DIR / "serial_bridge.py"),
            "--serial",
            args.serial,
            "--tcp-port",
            str(BRIDGE_PORT),
        ],
        "tmp/bridge.log",
    )
    if not _wait_tcp(BRIDGE_PORT):
        _err("serial_bridge failed (see tmp/bridge.log)")
        _stop_process(bridge_proc)
        sys.exit(1)
    _info("serial_bridge ready")

    companion = CompanionDriver(bridge_host="127.0.0.1", bridge_port=BRIDGE_PORT)
    radio_info = companion.get_radio()
    if not radio_info:
        _err("companion not responding")
        _stop_process(bridge_proc)
        sys.exit(1)
    _info(f"companion: {radio_info}")

    _info(f"Starting {binary}")
    binary_proc = _start_process(
        [binary, "--config", args.config],
        f"tmp/{mode}.log",
    )
    if not _wait_udp(udp_port):
        _err(f"{binary} failed to start (see tmp/{mode}.log)")
        _stop_process(binary_proc)
        _stop_process(bridge_proc)
        sys.exit(1)
    _info(f"{binary} ready")

    sock, _, _ = create_udp_subscriber("127.0.0.1", udp_port)
    collector = EventCollector(sock, event_types)
    collector.start()
    time.sleep(1.0)

    _info(f"\n--- {mode}: {len(matrix)} points, matrix={args.matrix} ---")
    if args.hypothesis:
        _info(f"H: {args.hypothesis}")
    _info("")

    results: list[PointResult] = []
    try:
        for i, point in enumerate(matrix):
            _info(f"[{i + 1}/{len(matrix)}] {point_label(point)}")
            result = run_point(mode, point, companion, collector)
            results.append(result)
            if mode == "decode":
                snr = f" SNR={result.best_snr}" if result.best_snr is not None else ""
                _info(f"  => {len(result.frames)} frames, {result.crc_ok} CRC OK{snr}")
            else:
                _info(
                    f"  => {result.det_count} det, ratio={result.best_ratio or 0:.1f}"
                )
    except KeyboardInterrupt:
        _info("\nInterrupted")
    finally:
        collector.stop()
        sock.close()
        companion.set_radio(869.618, 62.5, 8, cr=8)
        _stop_process(binary_proc)
        _stop_process(bridge_proc)

    _write_results(output_path, label, args, mode, binary, results)


async def _run_tx_experiment(
    args,
    matrix: list[ConfigPoint],
    binary: str,
    label: str,
    output_path: str,
) -> None:
    """TX experiment: meshcore_py connects directly to serial, no bridge."""
    # Start lora_trx (needed for SDR TX via UDP)
    _info(f"Starting {binary}")
    binary_proc = _start_process(
        [binary, "--config", args.config],
        "tmp/tx.log",
    )
    if not _wait_udp(TRX_PORT):
        _err(f"{binary} failed to start (see tmp/tx.log)")
        _stop_process(binary_proc)
        sys.exit(1)
    _info(f"{binary} ready")

    # Connect to Heltec via meshcore_py
    companion = MeshCoreCompanion()
    _info(f"Connecting to companion on {args.serial}")
    if not await companion.connect(args.serial):
        _err("meshcore_py: cannot connect to companion")
        _stop_process(binary_proc)
        sys.exit(1)
    _info(f"companion: {companion.pubkey[:16]}...")

    sdr_pubkey = _get_sdr_pubkey()
    if not sdr_pubkey:
        _err("cannot get SDR pubkey")
        await companion.close()
        _stop_process(binary_proc)
        sys.exit(1)
    _info(f"SDR pubkey: {sdr_pubkey[:16]}...")

    _info(f"\n--- tx: {len(matrix)} points, matrix={args.matrix} ---")
    if args.hypothesis:
        _info(f"H: {args.hypothesis}")
    _info("")

    results: list[PointResult] = []
    try:
        for i, point in enumerate(matrix):
            _info(f"[{i + 1}/{len(matrix)}] {point_label(point)}")
            result = await run_tx_point(point, companion, sdr_pubkey)
            results.append(result)
            _info(f"  => {result.crc_ok} pass, {result.crc_fail} fail")
    except KeyboardInterrupt:
        _info("\nInterrupted")
    finally:
        await companion.set_radio(869.618, 62.5, 8, cr=8)
        await companion.close()
        _stop_process(binary_proc)

    _write_results(output_path, label, args, "tx", binary, results)


def _write_results(
    output_path: str,
    label: str,
    args,
    mode: str,
    binary: str,
    results: list[PointResult],
) -> None:
    """Write JSON results to file and summary to stdout."""
    doc = {
        "experiment": label,
        "hypothesis": args.hypothesis,
        "mode": mode,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "git_sha": _git_sha(),
        "binary": binary,
        "config_file": args.config,
        "matrix": args.matrix,
        "results": [
            {
                "config": r.config,
                "tx_ok": r.tx_ok,
                "frames": r.frames,
                "crc_ok": r.crc_ok,
                "crc_fail": r.crc_fail,
                "best_snr": r.best_snr,
                "detected_sfs": r.detected_sfs,
                "detections": r.detections,
                "det_count": r.det_count,
                "best_ratio": r.best_ratio,
                "best_sf": r.best_sf,
                "sweeps": r.sweeps,
                "avg_sweep_ms": r.avg_sweep_ms,
                "overflows": r.overflows,
            }
            for r in results
        ],
        "summary": build_summary(mode, results),
    }
    with open(output_path, "w") as f:
        json.dump(doc, f, indent=2)
    _info(f"\nResults: {output_path}")
    json.dump(doc["summary"], sys.stdout, indent=2)
    print(file=sys.stdout)


if __name__ == "__main__":
    main()
