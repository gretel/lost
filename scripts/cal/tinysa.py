#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Shared TinySA Ultra serial client + helpers.

Used by aclr_sa.py (ACLR + harmonic measurement) and sdr_pwr_cal.py
(TX power characterisation). Self-contained: pyserial + numpy required;
Pillow optional (PNG screen capture falls through if missing).

TinySA serial protocol reference:
    https://tinysa.org/wiki/pmwiki.php?n=Main.USBInterface

RGB565 screen decode + '-:.0' firmware sanitiser adapted from
LC-Linkous/tinySA_python (MIT) --
    https://github.com/LC-Linkous/tinySA_python
"""
from __future__ import annotations

import dataclasses
import math
import pathlib
import struct
import time
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# TinySA Ultra / Ultra+ / ZS-406 constants.
TSA_VID_PID = (0x0483, 0x5740)
TSA_ULTRA_SCREEN = (480, 320)         # RGB565, 2 bytes/pixel
TSA_ULTRA_DBM_OFFSET = 174            # dBm = raw/32 - 174 on Ultra/Ultra+
TSA_MIN_RBW_KHZ = 3                   # firmware floor per wiki
TSA_NOISE_FLOOR_DBM = -105            # typical, RBW 3 kHz, high mode
TSA_DAMAGE_DBM = 10                   # absolute max input


@dataclasses.dataclass
class Trace:
    freq_hz: np.ndarray      # shape (N,)
    dbm: np.ndarray          # shape (N,)


class TinySA:
    """Minimal TinySA Ultra serial client. Thread-unsafe; single-shot use."""

    PROMPT = b"ch> "

    def __init__(self, port: str, timeout: float = 3.0):
        self.ser = serial.Serial(port, baudrate=115200, timeout=timeout)
        # CDC-ACM ignores baud; timeout still applies to reads.
        self._drain()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _drain(self) -> None:
        """Flush stale output until no bytes arrive for one tick."""
        self.ser.reset_input_buffer()
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def cmd(self, line: str, read_until_prompt: bool = True,
            extra_timeout: float = 0.0) -> bytes:
        """Send a command, return response bytes (prompt-stripped)."""
        self.ser.write(line.encode("ascii") + b"\r")
        self.ser.flush()
        if not read_until_prompt:
            return b""
        old = self.ser.timeout
        if extra_timeout:
            self.ser.timeout = old + extra_timeout
        try:
            buf = bytearray()
            while True:
                chunk = self.ser.read(4096)
                if not chunk:
                    break
                buf += chunk
                if buf.endswith(self.PROMPT):
                    break
            txt = bytes(buf)
            if txt.endswith(self.PROMPT):
                txt = txt[:-len(self.PROMPT)]
            nl = txt.find(b"\r\n")
            if nl >= 0:
                txt = txt[nl + 2:]
            return txt.rstrip(b"\r\n ")
        finally:
            self.ser.timeout = old

    def info(self) -> str:
        return self.cmd("info").decode("ascii", errors="replace")

    def pause(self) -> None:
        self.cmd("pause")

    def resume(self) -> None:
        self.cmd("resume")

    def set_rbw_khz(self, rbw_khz: float) -> None:
        if rbw_khz < TSA_MIN_RBW_KHZ:
            raise ValueError(
                f"RBW {rbw_khz} kHz below TinySA Ultra minimum "
                f"{TSA_MIN_RBW_KHZ} kHz")
        self.cmd(f"rbw {int(round(rbw_khz))}")

    def set_sweep(self, start_hz: float, stop_hz: float) -> None:
        self.cmd(f"sweep {int(start_hz)} {int(stop_hz)}")

    def waitscan(self, n: int = 1) -> None:
        self.cmd(f"waitscan {n}", extra_timeout=max(10.0, 3.0 * n))

    def set_repeat(self, n: int) -> None:
        """Set the per-frequency measurement count (`repeat 1..1000`).

        Applies to subsequent `hop` and `scan` commands — increasing N
        averages out noise at the cost of wall time.  See TinySA USB
        interface reference.
        """
        self.cmd(f"repeat {max(1, min(1000, int(n)))}")

    def hop(self, freq_hz: float) -> float:
        """Spot power measurement at `freq_hz` (dBm).

        Uses TinySA Ultra's `hop <start> <stop> <points> <outmask>`
        command with start=stop=freq_hz, points=1, outmask=2 (level
        only) — returns a single dBm value.  Averaging is configured
        separately via set_repeat().  Caller must set the desired RBW
        beforehand (e.g. equal to the integration bandwidth for a
        direct ICP read).  Returns NaN if the firmware reply is
        malformed.
        """
        f = int(round(freq_hz))
        raw = self.cmd(f"hop {f} {f} 1 2", extra_timeout=2.0)
        text = raw.decode("ascii", errors="replace").strip()
        for line in reversed(text.splitlines()):
            line = line.strip()
            if not line:
                continue
            try:
                return float(line)
            except ValueError:
                continue
        return float("nan")

    def scanraw(self, start_hz: float, stop_hz: float,
                points: int = 290) -> Trace:
        """Binary scan: returns Trace with N points between start and stop.

        Response framing:  '{' ('x' MSB LSB)*points '}'
        Ultra dBm:         raw/32 - 174
        """
        expected = 3 * points + 2
        self.ser.write(
            f"scanraw {int(start_hz)} {int(stop_hz)} {points}\r"
            .encode("ascii"))
        self.ser.flush()
        buf = bytearray()
        deadline = time.time() + max(8.0, points / 100.0)
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
            brace = buf.find(b"{")
            if brace >= 0 and len(buf) - brace >= expected:
                break
        brace = buf.find(b"{")
        if brace < 0 or len(buf) - brace < expected:
            raise RuntimeError(
                f"scanraw short read: want {expected}, "
                f"have {len(buf) - max(brace, 0)}")
        body = bytes(buf[brace + 1:brace + 1 + 3 * points])
        self._drain()
        raw = struct.unpack("<" + "xH" * points, body)
        dbm = np.array(raw, dtype=np.int32) / 32.0 - TSA_ULTRA_DBM_OFFSET
        freq = np.linspace(start_hz, stop_hz, points)
        return Trace(freq_hz=freq, dbm=dbm.astype(np.float64))

    def capture_screen(self) -> Optional[bytes]:
        """Returns raw 480*320*2 RGB565 bytes or None on short read."""
        w, h = TSA_ULTRA_SCREEN
        expected = w * h * 2
        self.ser.write(b"capture\r")
        self.ser.flush()
        time.sleep(0.05)
        buf = bytearray()
        deadline = time.time() + 15.0
        while time.time() < deadline and len(buf) < expected:
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            buf += chunk
        self._drain()
        if len(buf) < expected:
            return None
        return bytes(buf[:expected])


def autodetect_port() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        if p.vid is not None and (p.vid, p.pid) == TSA_VID_PID:
            return p.device
    return None


def render_rgb565_png(raw: bytes, out_path: pathlib.Path) -> bool:
    """Decode TinySA 480x320 RGB565 screen dump to PNG."""
    if not HAS_PIL:
        return False
    w, h = TSA_ULTRA_SCREEN
    n = w * h
    pixels = struct.unpack(f">{n}H", raw)
    arr = np.array(pixels, dtype=np.uint32)
    arr = (0xFF000000
           | ((arr & 0xF800) >> 8)
           | ((arr & 0x07E0) << 5)
           | ((arr & 0x001F) << 19))
    img = Image.frombuffer("RGBA", (w, h), arr.astype(np.uint32).tobytes(),
                           "raw", "RGBA", 0, 1)
    img.save(str(out_path))
    return True


def compute_aclr(trace: Trace, center_hz: float, main_bw_hz: float,
                 adj_offsets_hz: list) -> dict:
    """Integrate power (linear) over main and adjacent windows.

    Band definitions:
        main   = [center - bw/2,          center + bw/2]
        adj+k  = [center + off_k - bw/2,  center + off_k + bw/2]
        adj-k  = [center - off_k - bw/2,  center - off_k + bw/2]
    Two-sided ACLR = 10*log10(mean(adj+, adj-) / main).
    """
    freq = trace.freq_hz
    dbm = trace.dbm
    bin_hz = float(freq[1] - freq[0]) if len(freq) > 1 else 1.0
    mw = np.power(10.0, dbm / 10.0)

    def band_mw(lo: float, hi: float) -> float:
        mask = (freq >= lo) & (freq <= hi)
        return float(mw[mask].sum()) if mask.any() else math.nan

    main_lo = center_hz - main_bw_hz / 2.0
    main_hi = center_hz + main_bw_hz / 2.0
    main_p = band_mw(main_lo, main_hi)

    result = {
        "bin_hz": bin_hz,
        "main_bw_hz": main_bw_hz,
        "main_dbm": 10.0 * math.log10(main_p) if main_p > 0 else math.nan,
    }
    for off in adj_offsets_hz:
        plus = band_mw(center_hz + off - main_bw_hz / 2.0,
                       center_hz + off + main_bw_hz / 2.0)
        minus = band_mw(center_hz - off - main_bw_hz / 2.0,
                        center_hz - off + main_bw_hz / 2.0)
        adj_mean = (plus + minus) / 2.0
        key_k = f"adj_{int(round(off / 1e3))}khz"
        result[f"{key_k}_plus_dbm"] = (
            10.0 * math.log10(plus) if plus > 0 else math.nan)
        result[f"{key_k}_minus_dbm"] = (
            10.0 * math.log10(minus) if minus > 0 else math.nan)
        result[f"{key_k}_dbc"] = (
            10.0 * math.log10(adj_mean / main_p)
            if (main_p > 0 and adj_mean > 0) else math.nan)
    return result


def measure_harmonic(tsa: TinySA, fundamental_hz: float, order: int,
                     span_hz: float, rbw_khz: float, points: int,
                     averages: int, attenuation_db: float) -> dict:
    """Sweep around N*fundamental, return peak + ETSI EN 300 220-1 margin.

    Mask (simplified): below 1 GHz -> -36 dBm/100 kHz,
                       1-4 GHz    -> -30 dBm/100 kHz,
                       above 4 GHz -> -30 dBm/MHz (informative).
    """
    center = fundamental_hz * order
    start = center - span_hz / 2.0
    stop = center + span_hz / 2.0
    tsa.pause()
    tsa.set_rbw_khz(rbw_khz)
    tsa.set_sweep(start, stop)
    tsa.resume()
    tsa.waitscan(averages)
    tsa.pause()
    trace = tsa.scanraw(start, stop, points)
    peak_sa = float(np.max(trace.dbm))
    peak_src = peak_sa + attenuation_db
    peak_per_100khz_src = peak_src + 10.0 * math.log10(100.0 / rbw_khz)
    if center < 1e9:
        mask = -36.0
    elif center < 4e9:
        mask = -30.0
    else:
        mask = -30.0  # per MHz above 4 GHz; informative only
    return {
        "order": order,
        "freq_hz": center,
        "peak_dbm_at_sa": peak_sa,
        "peak_dbm_at_source": peak_src,
        "peak_dbm_per_100khz_at_source": peak_per_100khz_src,
        "etsi_mask_dbm_per_100khz": mask,
        "margin_db": mask - peak_per_100khz_src,
        "rbw_khz": rbw_khz,
        "trace": trace,
    }
