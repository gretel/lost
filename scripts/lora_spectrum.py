#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# pyright: reportImplicitRelativeImport=false, reportExplicitAny=false
# pyright: reportAny=false, reportUnknownMemberType=false
# pyright: reportUnknownVariableType=false, reportUnknownArgumentType=false
"""lora_spectrum.py — real-time LoRa spectrum bar graph.

Reads CBOR frames from lora_scan on stdin and renders a live spectrum
display with detection history to the terminal.

Usage:
    lora_scan --cbor 2>scan.log | python3 scripts/lora_spectrum.py
"""

from __future__ import annotations

import argparse
import atexit
import io
import math
import os
import struct
import sys
import time
from typing import Any

from cbor_stream import read_cbor_seq  # noqa: reportImplicitRelativeImport

# ─── Constants ─────────────────────────────────────────────────────────────────

DET_HISTORY = 4  # number of recent detections to show in footer
EMA_ALPHA = 0.3  # spectrum smoothing factor
DB_MARGIN = 5.0  # padding above signal range
HEADER_LINES = 2 + DET_HISTORY  # header + freq axis + footer lines

# ANSI colours (basic — no xterm-256)
C_GREEN = "\033[32m"
C_HOT = "\033[1;33m"  # bright yellow
C_DIM = "\033[2m"
C_BOLD = "\033[1m"
C_WARN = "\033[33m"
C_RST = "\033[0m"

# Use /dev/tty for display so stdout remains available as a pipe passthrough.
# Falls back to stderr when /dev/tty is unavailable (e.g. non-interactive).
_tty = None


def _out() -> Any:
    global _tty
    if _tty is None:
        try:
            _tty = io.FileIO(os.open("/dev/tty", os.O_WRONLY), mode="wb")
        except OSError:
            _tty = sys.stderr.buffer
    return _tty


def _write(s: str) -> None:
    _out().write(s.encode())
    _out().flush()


def _cleanup() -> None:
    try:
        _write("\033[?25h\033[0m\n")  # show cursor, reset colours
    except OSError:
        pass


_ = atexit.register(_cleanup)


# ─── State ─────────────────────────────────────────────────────────────────────


class State:
    """Spectrum and sweep state accumulated from CBOR messages."""

    def __init__(self, ema_alpha: float = EMA_ALPHA) -> None:
        self.energy: list[float] | None = None
        self.freq_min: int = 0
        self.freq_step: int = 62500
        self.n_channels: int = 0
        self.hot: set[int] = set()
        self.det_history: list[tuple[str, int, dict[str, Any]]] = []  # (ts, sweep, det)
        self.ema_alpha: float = ema_alpha

        # from scan_status
        self.mode: str = "?"
        self.sweeps: int = 0
        self.total_det: int = 0
        self.overflows: int = 0
        self.dropouts: int = 0

        # from scan_sweep_end
        self.last_sweep_ms: int = 0

        # sweep rate (computed from scan_spectrum arrival times)
        self.sweep_rate: float = 0.0
        self._prev_spectrum_time: float = 0.0
        self._prev_sweep_count: int = 0

    # ── handlers ──

    def on_spectrum(self, msg: dict[str, Any]) -> None:
        n = msg["n_channels"]
        raw = list(struct.unpack(f"<{n}f", msg["channels"]))

        if self.energy is None or len(self.energy) != n:
            self.energy = raw
        else:
            a = self.ema_alpha
            self.energy = [a * r + (1 - a) * e for r, e in zip(raw, self.energy)]

        self.freq_min = msg["freq_min"]
        self.freq_step = msg["freq_step"]
        self.n_channels = n
        self.hot = set(msg.get("hot", []))

        ts = time.strftime("%H:%M:%S")
        sweep = msg.get("sweep", self.sweeps)
        for det in msg.get("detections", []):
            self.det_history.append((ts, sweep, det))
        self.det_history = self.det_history[-DET_HISTORY:]

        self.sweeps = msg.get("sweep", self.sweeps)
        mono = time.monotonic()
        if self._prev_spectrum_time > 0:
            dt = mono - self._prev_spectrum_time
            ds = self.sweeps - self._prev_sweep_count
            if dt > 0.1 and ds > 0:
                self.sweep_rate = ds / dt
        self._prev_spectrum_time = mono
        self._prev_sweep_count = self.sweeps

    def on_status(self, msg: dict[str, Any]) -> None:
        self.mode = msg.get("mode", self.mode)
        self.sweeps = msg.get("sweeps", self.sweeps)
        self.total_det = msg.get("detections", self.total_det)
        self.overflows = msg.get("overflows", self.overflows)
        self.dropouts = msg.get("dropouts", self.dropouts)

    def on_sweep_start(self, _msg: dict[str, Any]) -> None:
        pass

    def on_sweep_end(self, msg: dict[str, Any]) -> None:
        self.last_sweep_ms = msg.get("duration_ms", 0)

    def on_l2_probe(self, _msg: dict[str, Any]) -> None:
        pass

    def on_overflow(self, msg: dict[str, Any]) -> None:
        self.overflows = msg.get("count", self.overflows)


# ─── Helpers ───────────────────────────────────────────────────────────────────


def _db(e: float) -> float:
    return 10.0 * math.log10(max(e, 1e-12))


def _resample(energy: list[float], cols: int) -> list[float]:
    """Map n_channels energy values to cols columns (shrink only)."""
    n = len(energy)
    if n == 0:
        return [0.0] * cols
    if n <= cols:
        # Pad by repeating — usually n≈112 and cols≈120
        out: list[float] = []
        for i, e in enumerate(energy):
            lo = i * cols // n
            hi = (i + 1) * cols // n
            out.extend([e] * (hi - lo))
        return out[:cols]
    # Shrink: average groups
    out = []
    for c in range(cols):
        lo = c * n // cols
        hi = (c + 1) * n // cols
        out.append(sum(energy[lo:hi]) / max(1, hi - lo))
    return out


# ─── Rendering ─────────────────────────────────────────────────────────────────

# Cached fd for terminal size queries (avoid opening /dev/tty each frame)
_tty_rd_fd: int = -1


def _term_size() -> tuple[int, int]:
    global _tty_rd_fd
    try:
        if _tty_rd_fd < 0:
            _tty_rd_fd = os.open("/dev/tty", os.O_RDONLY)
        return os.get_terminal_size(_tty_rd_fd)
    except OSError:
        return 80, 24


def _render_header(s: State) -> str:
    """Build single status line."""
    ovf = f" {C_WARN}OVF {s.overflows}{C_DIM}" if s.overflows else ""
    return (
        f"{C_DIM}sw {s.sweeps:>5}  {s.last_sweep_ms:>5}ms"
        + f"  det {s.total_det:>4}{ovf}"
        + f"  {s.sweep_rate:>4.1f} sw/s  {time.strftime('%H:%M:%S')}{C_RST}\033[K\n"
    )


def _render_footer(s: State) -> str:
    """Build footer: last N detections, one per line, newest first."""
    lines: list[str] = []
    for ts, sweep, det in reversed(s.det_history):
        freq_mhz = det["freq"] / 1e6
        bw_k = det["bw"] / 1000
        ratio_up = det.get("ratio_up", 0.0)
        ratio_dn = det.get("ratio_dn", 0.0)
        if ratio_up == 0 and ratio_dn == 0:
            ratio_up = ratio_dn = det.get("ratio", 0.0)
        chirp = det.get("chirp", "")
        lines.append(
            f"{C_DIM}{ts}{C_RST}"
            + f" #{sweep:<4}"
            + f" {freq_mhz:>7.3f} SF{det['sf']}/{bw_k:.0f}k"
            + f"  up={ratio_up:>5.1f} dn={ratio_dn:>5.1f}"
            + (f"  {chirp}" if chirp else "")
            + "\033[K"
        )
    # Pad to DET_HISTORY lines so the layout is stable
    while len(lines) < DET_HISTORY:
        lines.append("\033[K")
    return "\n".join(lines)


def render_header(s: State) -> None:
    """Update status line in-place."""
    _write(f"\033[H{_render_header(s)}")


def render(s: State) -> None:
    """Full redraw: status + frequency axis + bar chart + detection log."""
    if s.energy is None:
        return

    term_w, term_h = _term_size()

    lbl_w = 8  # left margin for dB labels
    spec_w = max(10, term_w - lbl_w)
    body_h = max(2, term_h - HEADER_LINES)

    col_e = _resample(s.energy, spec_w)
    n_cols = len(col_e)

    db = [_db(e) for e in col_e]
    nonzero = sorted(d for d, e in zip(db, col_e) if e > 1e-15)
    if nonzero:
        db_min = nonzero[len(nonzero) // 4] - 10
    else:
        db_min = -100.0
    db_max = max(db) + DB_MARGIN
    db_range = max(db_max - db_min, 20.0)

    # Bar heights (integer rows, 0..body_h)
    bar_h = [
        max(0, int((db[c] - db_min) / db_range * body_h + 0.5)) for c in range(n_cols)
    ]

    # HOT columns
    hot_cols: set[int] = set()
    for idx in s.hot:
        col = int(idx * n_cols / s.n_channels) if s.n_channels > 0 else 0
        hot_cols.add(min(col, n_cols - 1))

    out: list[str] = ["\033[H"]  # cursor home
    out.append(_render_header(s))

    # ── Line 3: frequency axis ──
    f_lo = s.freq_min / 1e6
    f_hi = (s.freq_min + s.n_channels * s.freq_step) / 1e6
    f_mid = (f_lo + f_hi) / 2
    left = f"{f_lo:.1f}"
    mid = f"{f_mid:.1f}"
    right = f"{f_hi:.1f} MHz"
    pad1 = max(1, spec_w // 2 - len(left) - len(mid) // 2)
    pad2 = max(1, spec_w - len(left) - pad1 - len(mid) - len(right))
    out.append(
        f"{' ' * lbl_w}{C_BOLD}{left}{' ' * pad1}{mid}"
        + f"{' ' * pad2}{right}{C_RST}\033[K\n"
    )

    # ── Body: bar chart ──
    for row in range(body_h):
        # dB label
        if row == 0:
            label = f"{db_max:6.0f}dB"
        elif row == body_h - 1:
            label = f"{db_min:6.0f}dB"
        elif body_h > 6 and row % max(1, body_h // 4) == 0:
            label = f"{db_max - (row / body_h) * db_range:6.0f}dB"
        else:
            label = ""
        out.append(f"{C_DIM}{label:>{lbl_w}}{C_RST}")

        threshold = body_h - row  # bar must reach this height to fill cell
        run_color: str = ""
        run_chars = 0
        for col in range(n_cols):
            if bar_h[col] >= threshold and col_e[col] > 1e-15:
                color = C_HOT if col in hot_cols else C_GREEN
                if color != run_color:
                    if run_chars:
                        out.append(f"{run_color}{'#' * run_chars}")
                    run_color = color
                    run_chars = 1
                else:
                    run_chars += 1
            else:
                if run_chars:
                    out.append(f"{run_color}{'#' * run_chars}{C_RST}")
                    run_color = ""
                    run_chars = 0
                out.append(" ")
        if run_chars:
            out.append(f"{run_color}{'#' * run_chars}{C_RST}")
        out.append("\033[K\n")

    # ── Footer: recent detection history ──
    out.append(_render_footer(s) + "\n")

    out.append("\033[J")  # clear below
    _write("".join(out))


# ─── Main loop ─────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa spectrum bar graph. Reads CBOR from lora_scan on stdin.",
        usage="lora_scan --cbor 2>scan.log | %(prog)s [options]",
    )
    _ = parser.add_argument(
        "--ema",
        type=float,
        default=EMA_ALPHA,
        help=f"EMA smoothing alpha (default: {EMA_ALPHA})",
    )
    args = parser.parse_args()

    _write("\033[?25l")  # hide cursor

    state = State(ema_alpha=args.ema)
    first = True

    dispatch = {
        "scan_spectrum": state.on_spectrum,
        "scan_status": state.on_status,
        "scan_sweep_start": state.on_sweep_start,
        "scan_sweep_end": state.on_sweep_end,
        "scan_l2_probe": state.on_l2_probe,
        "scan_overflow": state.on_overflow,
    }

    # Message types that trigger a header-only refresh (cheap, in-place)
    header_types = {
        "scan_status",
        "scan_sweep_start",
        "scan_sweep_end",
        "scan_l2_probe",
        "scan_overflow",
    }

    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type", "")
            handler = dispatch.get(msg_type)
            if handler:
                handler(msg)

            if msg_type == "scan_spectrum":
                if first:
                    _write("\033[2J")  # clear screen after UHD logs
                    first = False
                render(state)
            elif msg_type in header_types and not first:
                render_header(state)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
