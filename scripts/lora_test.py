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
    # USB serial (local companion):
    python3 scripts/lora_test.py decode --matrix full --serial /dev/cu.usbserial-0001
    python3 scripts/lora_test.py scan --matrix basic --serial /dev/cu.usbserial-0001

    # TCP (remote companion via WiFi serial bridge):
    python3 scripts/lora_test.py decode --matrix full --tcp 192.168.1.42:4000
    python3 scripts/lora_test.py scan --matrix basic --tcp 10.0.0.5:7835

    python3 scripts/lora_test.py decode --matrix dc_edge --label dc-on \\
        --hypothesis "2kHz DC blocker does not degrade SF12" --serial ...
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import random
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

from lora_common import (
    CompanionDriver,
    create_udp_subscriber,
    parse_host_port,
    send_lora_config,
)

SCRIPT_DIR = Path(__file__).parent

# -- Timing defaults (seconds) -----------------------------------------------

SETTLE_S = 2.0  # wait after radio change for PLL lock
FLUSH_DECODE_S = 5.0  # wait after TX for decode pipeline
FLUSH_SCAN_S = 8.0  # wait after TX for scan sweeps (~14 cycles at 555ms)
FLUSH_TX_S = 3.0  # wait after SDR TX for Heltec to receive
FLUSH_BRIDGE_S = 5.0  # wait for bridge RX path (decode + companion protocol)
BRIDGE_PORT = 7835  # serial_bridge TCP port
MESHCORE_BRIDGE_PORT = 7834  # meshcore_bridge TCP port
TRX_PORT = 5556  # lora_trx UDP port (accepts lora_tx messages)
AGG_PORT = 5555  # lora_agg consumer-facing UDP port
MIN_RATIO = 8.0  # scan detection threshold


def lora_airtime_s(
    sf: int,
    bw: int,
    n_bytes: int = 126,
    cr: int = 4,
    preamble: int = 8,
    explicit_header: bool = True,
) -> float:
    """Estimate LoRa packet airtime in seconds (Semtech AN1200.13).

    Default *n_bytes* = 126 (typical ADVERT with name + location).
    """
    t_sym = (1 << sf) / bw
    n_preamble = (preamble + 4.25) * t_sym
    de = 1 if (sf >= 11 and bw == 125000) or (sf == 12 and bw <= 125000) else 0
    ih = 0 if explicit_header else 1
    payload_bits = 8 * n_bytes - 4 * sf + 28 + 16 - 20 * ih
    n_payload = 8 + max(
        0, ((payload_bits + 4 * (sf - 2 * de) - 1) // (4 * (sf - 2 * de)))
    ) * (cr + 4)
    return n_preamble + n_payload * t_sym


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
    # Bridge-optimised: mixed BW to keep airtime under B210 TX limit (~1.5s)
    "bridge_full": [
        ConfigPoint(sf=7, bw=62500, freq_mhz=869.618),  # 652ms
        ConfigPoint(sf=8, bw=62500, freq_mhz=869.618),  # 1140ms
        ConfigPoint(sf=9, bw=125000, freq_mhz=869.618),  # 1042ms
        ConfigPoint(sf=10, bw=250000, freq_mhz=869.618),  # 1042ms
    ],
    "bw125k_sweep": [
        ConfigPoint(sf=sf, bw=125000, freq_mhz=869.618) for sf in [7, 8, 9]
    ],
    "bw250k_sweep": [
        ConfigPoint(sf=sf, bw=250000, freq_mhz=869.618) for sf in [7, 8, 9, 10]
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
    if mode == "decode":
        # SF12/BW62.5k airtime is ~4.5s; the fixed 5.0s left only 0.5s for
        # decode pipeline processing.  Use airtime + 3s margin.
        flush_s = max(FLUSH_DECODE_S, lora_airtime_s(point.sf, point.bw) + 3.0)
    else:
        flush_s = FLUSH_SCAN_S
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

    # TX ADVERT(s) — scan mode sends 3 with gaps to increase detection
    # probability (streaming scan has ~60% per-ADVERT detection rate due
    # to L1 sweep timing alignment)
    if mode == "scan":
        ok_count = 0
        for i in range(3):
            if companion.send_advert():
                ok_count += 1
            if i < 2:
                time.sleep(2.0)
        result.tx_ok = ok_count > 0
        _info(f"  TX {ok_count}/3 ok")
    else:
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
    n_spectrum = sum(1 for e in events if e.get("type") == "scan_spectrum")
    n_with_det = sum(
        1 for e in events if e.get("type") == "scan_spectrum" and e.get("detections")
    )
    if n_spectrum > 0:
        _info(f"  cbor: {n_spectrum} spectrum, {n_with_det} with det")
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
        self._tcp_host = ""
        self._tcp_port = 0
        self._adverts: list[dict] = []
        self._messages: list[dict] = []
        self._lock = threading.Lock()
        self._pubkey = ""

    async def connect(
        self,
        serial_port: str | None = None,
        *,
        tcp_host: str | None = None,
        tcp_port: int | None = None,
    ) -> bool:
        from meshcore import MeshCore
        from meshcore.events import EventType

        if tcp_host is not None and tcp_port is not None:
            self._serial_port = ""
            self._tcp_host = tcp_host
            self._tcp_port = tcp_port
            self._mc = await MeshCore.create_tcp(tcp_host, tcp_port)
        else:
            self._serial_port = serial_port or ""
            self._tcp_host = ""
            self._tcp_port = 0
            self._mc = await MeshCore.create_serial(self._serial_port, 115200)
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
        if self._tcp_host:
            self._mc = await MeshCore.create_tcp(self._tcp_host, self._tcp_port)
        else:
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

    async def send_advert(self) -> bool:
        """Send an ADVERT from the Heltec via meshcore_py."""
        try:
            result = await self._mc.commands.send_advert()
            from meshcore.events import EventType

            return result.type == EventType.OK
        except Exception as e:
            _info(f"  send_advert failed: {e}")
            return False

    async def send_message(self, dest_pubkey: bytes, text: str) -> bool:
        """Send a TXT_MSG to a destination by pubkey bytes."""
        try:
            result = await self._mc.commands.send_msg(dest_pubkey, text)
            from meshcore.events import EventType

            return result.type in (EventType.OK, EventType.MSG_SENT)
        except Exception as e:
            _info(f"  send_message failed: {e}")
            return False

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


# -- Bridge mode ---------------------------------------------------------------


def bridge_payload(point: ConfigPoint) -> str:
    """Build a unique payload string for bridge mode.

    Format: sf{sf}bw{bw_khz}f{freq_khz}_{nonce:02x}
    Example: sf8bw62f869618_a7
    """
    bw_khz = int(point.bw / 1000)
    freq_khz = int(point.freq_mhz * 1000)
    nonce = random.randint(0, 255)
    return f"sf{point.sf}bw{bw_khz}f{freq_khz}_{nonce:02x}"


def _extract_pubkey_from_card(card_output: str) -> str:
    """Extract the 64-char hex pubkey from meshcore-cli card output.

    The card output is a URI like:
        meshcore://contact/add?name=...&public_key=<64hex>&type=1
    or a raw hex bizcard:
        meshcore://<hex>
    """
    m = re.search(r"public_key=([0-9a-fA-F]{64})", card_output)
    if m:
        return m.group(1).lower()
    # Try extracting from raw bizcard: meshcore://<hex wire packet>
    # ADVERT wire format: header(1) + path_len(1) + pubkey(32) + ...
    # So pubkey starts at byte 2 = hex offset 4.
    m = re.search(r"meshcore://([0-9a-fA-F]+)", card_output)
    if m:
        hexdata = m.group(1)
        if len(hexdata) >= 68:  # 4 (header+path) + 64 (pubkey)
            return hexdata[4:68].lower()
    return ""


async def run_bridge_point(
    point: ConfigPoint,
    companion: MeshCoreCompanion,
    driver: CompanionDriver,
    config_sock: socket.socket,
    config_addr: tuple[str, int],
    sdr_pubkey: str,
    heltec_name: str,
    contact_exchanged: bool,
) -> dict:
    """Run one bridge test point with phases A-D."""
    result: dict = {
        "config": asdict(point),
        "advert_rx": None,  # Phase A (None = skipped)
        "advert_tx": False,  # Phase B
        "advert_tx_snr": None,
        "msg_tx": False,  # Phase C
        "msg_tx_payload": "",
        "msg_tx_acked": None,  # Phase E (None = skipped/unknown)
        "msg_rx": False,  # Phase D
        "msg_rx_payload": "",
        "phases_passed": 0,
        "phases_total": 4,
    }

    # Step 0: Reconfigure PHY
    _info("  Step 0: reconfigure PHY")
    # Update lora_trx decode + TX params via CBOR
    # Drain stale spectrum/status messages before sending config —
    # lora_trx floods subscribers at ~50 spectrum msgs/s and the ack
    # gets buried if we don't flush first.
    config_sock.setblocking(False)
    try:
        while True:
            config_sock.recvfrom(65536)
    except BlockingIOError:
        pass
    config_sock.setblocking(True)
    send_lora_config(
        config_sock,
        config_addr,
        sf=point.sf,
        bw=point.bw,
        freq=int(point.freq_mhz * 1e6),
    )
    # Update bridge TX params via companion protocol (CMD_SET_RADIO_PARAMS).
    # The bridge may not receive the lora_trx config broadcast reliably,
    # so we set its state directly.
    bw_khz = point.bw / 1000.0
    driver.set_radio(point.freq_mhz, bw_khz, point.sf, cr=8)
    # Update Heltec radio (triggers reboot + auto-reconnect)
    if not await companion.set_radio(point.freq_mhz, bw_khz, point.sf, cr=8):
        _err(f"  set_radio failed for {point_label(point)}")
        return result
    await asyncio.sleep(SETTLE_S)

    # Phase A: Heltec ADVERT → Bridge RX (contact exchange)
    if not contact_exchanged:
        _info("  Phase A: Heltec ADVERT → Bridge RX")
        await companion.send_advert()
        tx_air_a = lora_airtime_s(point.sf, point.bw)
        await asyncio.sleep(max(FLUSH_BRIDGE_S, tx_air_a + 2.0))
        contacts_out = driver.get_contacts()
        heltec_pk = companion.pubkey.lower()
        advert_rx = heltec_pk[:12] in contacts_out.lower() if heltec_pk else False
        result["advert_rx"] = advert_rx
        if advert_rx:
            result["phases_passed"] += 1
            _info("  Phase A: PASS (contact seen)")
        else:
            _info(f"  Phase A: FAIL (pubkey {heltec_pk[:12]} not in contacts)")
    else:
        result["advert_rx"] = True  # already exchanged
        result["phases_passed"] += 1
        _info("  Phase A: SKIP (contact already exchanged)")

    # Inter-phase gap: let the radio settle between TX bursts to avoid
    # overlapping transmissions that crash the B210 USB transport.
    await asyncio.sleep(1.0)

    # Phase B: Bridge ADVERT → Heltec RX
    _info("  Phase B: Bridge ADVERT → Heltec RX")
    companion.drain_adverts()  # discard stale
    driver.send_advert()
    # Wait for TX airtime + propagation.  At high SF the ADVERT airtime is
    # significant (SF9/BW62.5k ≈ 2s, SF12/BW62.5k ≈ 17s).  The B210 USB
    # transport crashes if another TX is queued while the radio is busy.
    tx_air = lora_airtime_s(point.sf, point.bw)
    await asyncio.sleep(max(FLUSH_TX_S, tx_air + 1.0))
    adverts = companion.drain_adverts()
    advert_tx = any(
        a.get("adv_key", "").lower().startswith(sdr_pubkey[:12].lower())
        for a in adverts
    )
    result["advert_tx"] = advert_tx
    if advert_tx:
        result["phases_passed"] += 1
        snr = next(
            (
                a["snr"]
                for a in adverts
                if a.get("adv_key", "").lower().startswith(sdr_pubkey[:12].lower())
            ),
            None,
        )
        result["advert_tx_snr"] = snr
        _info(f"  Phase B: PASS (SNR={snr})")
    else:
        keys = [a.get("adv_key", "")[:12] for a in adverts]
        _info(f"  Phase B: FAIL (saw {len(adverts)} adverts: {keys})")

    await asyncio.sleep(1.0)

    # Phase C: Bridge TXT_MSG → Heltec RX
    _info("  Phase C: Bridge TXT_MSG → Heltec RX")
    companion.drain_messages()  # discard stale
    payload_c = bridge_payload(point)
    result["msg_tx_payload"] = payload_c
    msg_ok, msg_output = driver.send_msg(heltec_name, payload_c)
    # Phase E: parse ACK status from meshcore-cli output
    # meshcore-cli prints "acked" when the recipient ACKs the message.
    msg_acked = "acked" in msg_output.lower() if msg_ok else None
    result["msg_tx_acked"] = msg_acked
    # TXT_MSG is shorter than ADVERT but still needs airtime margin
    tx_air_c = lora_airtime_s(point.sf, point.bw, n_bytes=40)
    await asyncio.sleep(max(FLUSH_TX_S, tx_air_c + 1.0))
    msgs = companion.drain_messages()
    msg_tx = any(payload_c in m.get("text", "") for m in msgs)
    result["msg_tx"] = msg_tx
    if msg_tx:
        result["phases_passed"] += 1
        ack_str = f" acked={msg_acked}" if msg_acked is not None else ""
        _info(f"  Phase C: PASS ('{payload_c}'{ack_str})")
    else:
        texts = [m.get("text", "")[:40] for m in msgs]
        _info(f"  Phase C: FAIL (sent '{payload_c}', saw {len(msgs)} msgs: {texts})")

    await asyncio.sleep(1.0)

    # Phase D: Heltec TXT_MSG → Bridge RX
    _info("  Phase D: Heltec TXT_MSG → Bridge RX")
    payload_d = bridge_payload(point)
    result["msg_rx_payload"] = payload_d
    sdr_pubkey_bytes = bytes.fromhex(sdr_pubkey)
    await companion.send_message(sdr_pubkey_bytes, payload_d)
    # Poll for the expected payload with retries.  The Heltec→RF→decode→bridge
    # chain has variable latency (decode time depends on SF), so we poll the
    # message queue repeatedly instead of a single fixed sleep.
    tx_air_d = lora_airtime_s(point.sf, point.bw, n_bytes=40)
    deadline = asyncio.get_event_loop().time() + max(FLUSH_BRIDGE_S, tx_air_d + 3.0)
    all_recv: list[str] = []
    msg_rx = False
    while asyncio.get_event_loop().time() < deadline:
        await asyncio.sleep(1.0)
        for _ in range(10):
            recv_out = driver.recv_msg(timeout=1.0)
            if not recv_out:
                break
            all_recv.append(recv_out)
            if payload_d in recv_out:
                msg_rx = True
        if msg_rx:
            break
    result["msg_rx"] = msg_rx
    if msg_rx:
        result["phases_passed"] += 1
        _info(f"  Phase D: PASS ('{payload_d}')")
    else:
        shown = [r[-40:] for r in all_recv[:3]]
        _info(
            f"  Phase D: FAIL (sent '{payload_d}', got {len(all_recv)} msgs: {shown})"
        )

    await asyncio.sleep(1.0)

    # Phase F: Bridge GRP_TXT → Heltec RX (channel message)
    # Uses the public channel PSK (8b3387e9c5cdea6ac9e5edbaa115cd72).
    # Bridge→Heltec direction only for v1.
    _info("  Phase F: Bridge GRP_TXT → Heltec RX (public channel)")
    companion.drain_messages()
    payload_f = bridge_payload(point)
    chan_ok = driver.chan_msg("public", payload_f)
    result["chan_tx"] = False
    result["chan_tx_payload"] = payload_f
    if chan_ok:
        tx_air_f = lora_airtime_s(point.sf, point.bw, n_bytes=40)
        await asyncio.sleep(max(FLUSH_TX_S, tx_air_f + 1.0))
        chan_msgs = companion.drain_messages()
        chan_rx = any(payload_f in m.get("text", "") for m in chan_msgs)
        result["chan_tx"] = chan_rx
        if chan_rx:
            _info(f"  Phase F: PASS ('{payload_f}')")
        else:
            texts = [m.get("text", "")[:40] for m in chan_msgs]
            _info(
                f"  Phase F: FAIL (sent '{payload_f}', saw {len(chan_msgs)} msgs: {texts})"
            )
    else:
        _info("  Phase F: FAIL (chan_msg command failed)")

    return result


def _collect_bridge(results: list[dict]) -> dict:
    """Build bridge mode summary."""
    n = len(results)
    advert_rx = sum(1 for r in results if r.get("advert_rx"))
    advert_tx = sum(1 for r in results if r.get("advert_tx"))
    msg_tx = sum(1 for r in results if r.get("msg_tx"))
    msg_tx_acked = sum(1 for r in results if r.get("msg_tx_acked"))
    msg_rx = sum(1 for r in results if r.get("msg_rx"))
    chan_tx = sum(1 for r in results if r.get("chan_tx"))
    return {
        "mode": "bridge",
        "points": n,
        "advert_rx": advert_rx,
        "advert_tx": advert_tx,
        "msg_tx": msg_tx,
        "msg_tx_acked": msg_tx_acked,
        "msg_rx": msg_rx,
        "chan_tx": chan_tx,
        "pass_rate": round(
            (advert_rx + advert_tx + msg_tx + msg_rx) / max(1, 4 * n), 3
        ),
    }


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

    for name in ("decode", "scan", "tx", "bridge"):
        p = sub.add_parser(name)
        conn = p.add_mutually_exclusive_group(required=True)
        conn.add_argument(
            "--serial",
            help="Companion serial port (USB, e.g. /dev/cu.usbserial-0001)",
        )
        conn.add_argument(
            "--tcp",
            metavar="HOST:PORT",
            help="Companion TCP address (WiFi bridge, e.g. 192.168.1.42:4000)",
        )
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

    # Parse --tcp early so we get a clear error for bad HOST:PORT
    tcp_host: str | None = None
    tcp_port: int | None = None
    if args.tcp:
        try:
            tcp_host, tcp_port = parse_host_port(args.tcp)
        except ValueError as e:
            parser.error(f"--tcp: {e}")

    matrix = list(MATRICES[args.matrix])
    mode = args.mode

    # Resolve binary and UDP port from mode
    if mode in ("decode", "tx", "bridge"):
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
        asyncio.run(
            _run_tx_experiment(
                args,
                matrix,
                binary,
                label,
                output_path,
                tcp_host=tcp_host,
                tcp_port=tcp_port,
            )
        )
        return

    # -- Bridge mode: lora_trx + meshcore_bridge + meshcore_py ---
    if mode == "bridge":
        asyncio.run(
            _run_bridge_experiment(
                args,
                matrix,
                binary,
                label,
                output_path,
                tcp_host=tcp_host,
                tcp_port=tcp_port,
            )
        )
        return

    # -- decode/scan: companion via serial_bridge (USB) or direct TCP ---
    bridge_proc = None
    if tcp_host is not None:
        # Remote companion: connect directly via TCP (no local serial_bridge)
        _info(f"Connecting to companion via TCP {tcp_host}:{tcp_port}")
        companion = CompanionDriver(bridge_host=tcp_host, bridge_port=tcp_port)
    else:
        # Local companion: spawn serial_bridge for DTR-safe USB passthrough
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
    # Scan mode needs longer startup — B210 at 16 MS/s has ~8s transient
    # (master clock switch, overflow burst, L1 energy settling)
    startup_delay = 10.0 if mode == "scan" else 1.0
    time.sleep(startup_delay)

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
    *,
    tcp_host: str | None = None,
    tcp_port: int | None = None,
) -> None:
    """TX experiment: meshcore_py connects via serial or TCP, no bridge."""
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

    # Connect to Heltec via meshcore_py (TCP or serial)
    companion = MeshCoreCompanion()
    if tcp_host is not None:
        conn_label = f"{tcp_host}:{tcp_port}"
        _info(f"Connecting to companion via TCP {conn_label}")
        connected = await companion.connect(tcp_host=tcp_host, tcp_port=tcp_port)
    else:
        conn_label = args.serial
        _info(f"Connecting to companion on {conn_label}")
        connected = await companion.connect(args.serial)
    if not connected:
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


async def _run_bridge_experiment(
    args,
    matrix: list[ConfigPoint],
    binary: str,
    label: str,
    output_path: str,
    *,
    tcp_host: str | None = None,
    tcp_port: int | None = None,
) -> None:
    """Bridge experiment: full bidirectional message exchange via meshcore_bridge.

    Process stack (inner to outer):
      lora_trx:5556 ← lora_agg:5555 ← meshcore_bridge:7834
    lora_agg deduplicates TX echoes (fnv1a64 hash matching, TX_ECHO_WINDOW=10s)
    so Phase D doesn't see the bridge's own Phase C packets.
    Config socket goes direct to lora_trx:5556 (lora_agg doesn't forward lora_config).
    """
    # 1. Start lora_trx
    _info(f"Starting {binary}")
    binary_proc = _start_process(
        [binary, "--config", args.config],
        "tmp/bridge_trx.log",
    )
    if not _wait_udp(TRX_PORT):
        _err(f"{binary} failed to start (see tmp/bridge_trx.log)")
        _stop_process(binary_proc)
        sys.exit(1)
    _info(f"{binary} ready")

    # 2. Start lora_agg.py (dedup + TX echo suppression)
    _info("Starting lora_agg")
    agg_cmd = [
        sys.executable,
        str(SCRIPT_DIR / "lora_agg.py"),
        "--upstream",
        f"127.0.0.1:{TRX_PORT}",
        "--listen",
        f"127.0.0.1:{AGG_PORT}",
    ]
    agg_proc = _start_process(agg_cmd, "tmp/lora_agg.log")
    if not _wait_udp(AGG_PORT):
        _err("lora_agg failed (see tmp/lora_agg.log)")
        _stop_process(agg_proc)
        _stop_process(binary_proc)
        sys.exit(1)
    _info("lora_agg ready")

    # 3. Start meshcore_bridge.py (connects to lora_agg, not lora_trx directly)
    _info("Starting meshcore_bridge")
    bridge_cmd = [
        sys.executable,
        str(SCRIPT_DIR / "meshcore_bridge.py"),
        "--connect",
        f"127.0.0.1:{AGG_PORT}",
        "--port",
        str(MESHCORE_BRIDGE_PORT),
    ]
    bridge_proc = _start_process(bridge_cmd, "tmp/meshcore_bridge.log")
    if not _wait_tcp(MESHCORE_BRIDGE_PORT):
        _err("meshcore_bridge failed (see tmp/meshcore_bridge.log)")
        _stop_process(bridge_proc)
        _stop_process(agg_proc)
        _stop_process(binary_proc)
        sys.exit(1)
    _info("meshcore_bridge ready")

    # 3. Connect to Heltec via meshcore_py (same as tx mode)
    companion = MeshCoreCompanion()
    if tcp_host is not None:
        conn_label = f"{tcp_host}:{tcp_port}"
        _info(f"Connecting to companion via TCP {conn_label}")
        connected = await companion.connect(tcp_host=tcp_host, tcp_port=tcp_port)
    else:
        conn_label = args.serial
        _info(f"Connecting to companion on {conn_label}")
        connected = await companion.connect(args.serial)
    if not connected:
        _err("meshcore_py: cannot connect to companion")
        _stop_process(bridge_proc)
        _stop_process(agg_proc)
        _stop_process(binary_proc)
        sys.exit(1)
    heltec_pubkey = companion.pubkey
    _info(f"Heltec pubkey: {heltec_pubkey[:16]}...")

    # 5. Create CompanionDriver for the bridge
    driver = CompanionDriver(bridge_host="127.0.0.1", bridge_port=MESHCORE_BRIDGE_PORT)

    # 6. Get SDR pubkey from bridge's card
    card_out = driver.get_card()
    sdr_pubkey = _extract_pubkey_from_card(card_out)
    if not sdr_pubkey:
        _err(f"cannot extract SDR pubkey from card output: {card_out[:80]}")
        await companion.close()
        _stop_process(bridge_proc)
        _stop_process(agg_proc)
        _stop_process(binary_proc)
        sys.exit(1)
    _info(f"SDR pubkey: {sdr_pubkey[:16]}...")

    # 7. Heltec contact name (for meshcore-cli msg <name> <text>)
    # meshcore-cli uses the node name from the ADVERT; we'll use the first
    # portion of the pubkey as fallback
    heltec_name = heltec_pubkey[:8]  # short prefix as contact name

    # 8. Create UDP socket for lora_config messages.
    # Config goes direct to lora_trx:5556 (lora_agg doesn't forward lora_config).
    # Pre-subscribe and drain: lora_trx floods subscribers with ~50 spectrum
    # messages/s. Without draining first, send_lora_config's ack gets buried
    # and times out.
    config_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    config_sock.bind(("127.0.0.1", 0))
    config_addr = ("127.0.0.1", TRX_PORT)
    config_sock.sendto(cbor2.dumps({"type": "subscribe"}), config_addr)
    await asyncio.sleep(1.0)
    config_sock.settimeout(0.1)
    try:
        while True:
            config_sock.recvfrom(65536)
    except socket.timeout:
        pass
    config_sock.settimeout(None)

    _info(f"\n--- bridge: {len(matrix)} points, matrix={args.matrix} ---")
    if args.hypothesis:
        _info(f"H: {args.hypothesis}")
    _info("")

    # 9. Run each config point
    bridge_results: list[dict] = []
    contact_exchanged = False
    try:
        for i, point in enumerate(matrix):
            _info(f"[{i + 1}/{len(matrix)}] {point_label(point)}")
            result = await run_bridge_point(
                point,
                companion,
                driver,
                config_sock,
                config_addr,
                sdr_pubkey,
                heltec_name,
                contact_exchanged,
            )
            bridge_results.append(result)
            passed = result["phases_passed"]
            total = result["phases_total"]
            _info(f"  => {passed}/{total} phases passed")
            # After first successful Phase A, skip it for subsequent points
            if result.get("advert_rx"):
                contact_exchanged = True
    except KeyboardInterrupt:
        _info("\nInterrupted")
    finally:
        config_sock.close()
        await companion.set_radio(869.618, 62.5, 8, cr=8)
        await companion.close()
        _stop_process(bridge_proc)
        _stop_process(agg_proc)
        _stop_process(binary_proc)

    # 10. Write results
    summary = _collect_bridge(bridge_results)
    doc = {
        "experiment": label,
        "hypothesis": args.hypothesis,
        "mode": "bridge",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "git_sha": _git_sha(),
        "binary": binary,
        "config_file": args.config,
        "matrix": args.matrix,
        "results": bridge_results,
        "summary": summary,
    }
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(doc, f, indent=2)
    _info(f"\nResults: {output_path}")

    n = summary["points"]
    _info(
        f"BRIDGE {n}/{n} points: "
        f"advert_rx={summary['advert_rx']}/{n} "
        f"advert_tx={summary['advert_tx']}/{n} "
        f"msg_tx={summary['msg_tx']}/{n} "
        f"msg_tx_acked={summary['msg_tx_acked']}/{n} "
        f"msg_rx={summary['msg_rx']}/{n}"
    )
    json.dump(summary, sys.stdout, indent=2)
    print(file=sys.stdout)


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
