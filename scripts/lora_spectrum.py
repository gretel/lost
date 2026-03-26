#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# pyright: reportImplicitRelativeImport=false, reportExplicitAny=false
# pyright: reportAny=false, reportUnknownMemberType=false
# pyright: reportUnknownVariableType=false, reportUnknownArgumentType=false
"""lora_spectrum.py — real-time LoRa spectrum bar graph.

Subscribes to lora_scan CBOR events via UDP and renders a live spectrum
display with detection anchors and history to the terminal.

Usage:
    python3 scripts/lora_spectrum.py [--host HOST] [--port PORT]
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

import cbor2

from lora_common import create_udp_subscriber  # noqa: reportImplicitRelativeImport

# ─── Constants ─────────────────────────────────────────────────────────────────

DET_HISTORY = 6  # number of recent detections to show in footer
EMA_ALPHA = 0.3  # spectrum smoothing factor
DB_HEADROOM = 1.0  # dB padding above peak
DB_FLOOR_PAD = 3.0  # dB padding below noise floor (25th percentile)
ANCHOR_TTL = 3.0  # seconds to keep detection anchors visible
HEADER_LINES = 3 + DET_HISTORY  # header + marker row + freq axis + footer lines

# ANSI helpers — xterm-256 for reliable rendering on any background
C_DIM = "\033[2m"
C_BOLD = "\033[1m"
C_RST = "\033[0m"

# Named xterm-256 colors for header/footer
C_CYAN = "\033[38;5;87m"  # bright cyan — frequencies
C_GREEN = "\033[38;5;46m"  # bright green — good values
C_YELLOW = "\033[38;5;226m"  # yellow — warnings, medium values
C_ORANGE = "\033[38;5;214m"  # orange — hot values
C_RED = "\033[38;5;196m"  # red — bad values
C_BLUE = "\033[38;5;39m"  # blue — info labels
C_WHITE = "\033[38;5;255m"  # bright white — emphasis
C_GRAY = "\033[38;5;242m"  # dim gray — secondary info
C_MAGENTA = "\033[38;5;213m"  # magenta — detection highlights

# Energy gradient (xterm-256): index 0 = weakest, index N-1 = strongest.
# Foreground-only codes — render identically on any terminal background.
_GRAD = [
    "\033[38;5;17m",  # dark navy     — deep noise
    "\033[38;5;24m",  # dark blue     — noise floor
    "\033[38;5;30m",  # teal          — above noise
    "\033[38;5;35m",  # dark green    — weak signal
    "\033[38;5;40m",  # bright green  — signal
    "\033[38;5;154m",  # lime          — good signal
    "\033[38;5;226m",  # yellow        — strong signal
    "\033[38;5;214m",  # orange        — very strong
    "\033[38;5;202m",  # dark orange   — hot
    "\033[38;5;196m",  # red           — peak
]
_N_GRAD = len(_GRAD)

_BAR = "\u2588"  # █ — full block for solid fill

# Use /dev/tty for display so stdout remains available as a pipe passthrough.
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
        self.det_history: list[tuple[float, int, dict[str, Any]]] = []
        self.ema_alpha: float = ema_alpha

        # from scan_status
        self.mode: str = "?"
        self.sweeps: int = 0
        self.total_det: int = 0
        self.overflows: int = 0
        self.dropouts: int = 0

        # from scan_sweep_end
        self.last_sweep_ms: int = 0

        # sweep rate
        self.sweep_rate: float = 0.0
        self._prev_spectrum_time: float = 0.0
        self._prev_sweep_count: int = 0

        # active detections for anchor rendering — (monotonic_time, det) pairs
        # kept for ANCHOR_TTL seconds so labels persist across sweeps
        self.active_dets: list[tuple[float, dict[str, Any]]] = []

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

        wall = time.time()
        mono = time.monotonic()
        sweep = msg.get("sweep", self.sweeps)
        dets = msg.get("detections", [])
        for det in dets:
            self.active_dets.append((mono, det))
            self.det_history.append((wall, sweep, det))
        # Prune anchors older than ANCHOR_TTL
        self.active_dets = [
            (t, d) for t, d in self.active_dets if mono - t < ANCHOR_TTL
        ]
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
        self.sweeps = msg.get("sweep", self.sweeps)
        self.total_det += msg.get("detections", 0)
        ovf = msg.get("overflows", 0)
        if ovf > 0:
            self.overflows = ovf

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
        out: list[float] = []
        for i, e in enumerate(energy):
            lo = i * cols // n
            hi = (i + 1) * cols // n
            out.extend([e] * (hi - lo))
        return out[:cols]
    out = []
    for c in range(cols):
        lo = c * n // cols
        hi = (c + 1) * n // cols
        out.append(sum(energy[lo:hi]) / max(1, hi - lo))
    return out


def _grad_color(frac: float) -> str:
    """Return gradient ANSI escape for a fraction in [0, 1]."""
    idx = min(int(frac * _N_GRAD), _N_GRAD - 1)
    return _GRAD[idx]


def _bw_short(bw_hz: float) -> str:
    """Format BW as short string: 62.5k, 125k, 250k, 500k."""
    if bw_hz <= 0:
        return "?"
    k = bw_hz / 1000
    if k == int(k):
        return f"{int(k)}k"
    return f"{k:g}k"


def _det_col(
    det: dict[str, Any], freq_min: int, freq_step: int, n_ch: int, spec_w: int
) -> int:
    """Map a detection frequency to a column index in the spectrum."""
    freq = det["freq"]
    if freq_step <= 0 or n_ch <= 0:
        return spec_w // 2
    ch_idx = (freq - freq_min) / freq_step
    return max(0, min(spec_w - 1, int(ch_idx * spec_w / n_ch)))


def _ratio_color(ratio: float) -> str:
    """Color-code a detection ratio: green=strong, yellow=medium, red=weak."""
    if ratio >= 50:
        return C_GREEN
    if ratio >= 20:
        return C_YELLOW
    return C_ORANGE


def _sweep_rate_color(rate: float) -> str:
    """Color-code sweep rate: green=fast, yellow=ok, red=slow."""
    if rate >= 0.8:
        return C_GREEN
    if rate >= 0.4:
        return C_YELLOW
    return C_RED


def _ts_hires(wall: float) -> str:
    """Format wall time as HH:MM:SS.mmm."""
    lt = time.localtime(wall)
    ms = int((wall % 1) * 1000)
    return f"{lt.tm_hour:02d}:{lt.tm_min:02d}:{lt.tm_sec:02d}.{ms:03d}"


# ─── Rendering ─────────────────────────────────────────────────────────────────

_tty_rd_fd: int = -1


def _term_size() -> tuple[int, int]:
    global _tty_rd_fd
    try:
        if _tty_rd_fd < 0:
            _tty_rd_fd = os.open("/dev/tty", os.O_RDONLY)
        return os.get_terminal_size(_tty_rd_fd)
    except OSError:
        return 80, 24


def _render_header(s: State, term_w: int) -> str:
    """Colorful status header line."""
    parts: list[str] = []

    # Script name
    parts.append(f"{C_WHITE}lora_spectrum{C_RST}")

    # Sweep counter
    parts.append(f"  {C_BLUE}sw{C_RST} {C_WHITE}{s.sweeps:>5}{C_RST}")

    # Sweep duration
    if s.last_sweep_ms > 0:
        dur_c = (
            C_GREEN
            if s.last_sweep_ms < 1500
            else C_YELLOW
            if s.last_sweep_ms < 3000
            else C_RED
        )
        parts.append(f"  {dur_c}{s.last_sweep_ms:>5}ms{C_RST}")

    # Detections
    det_c = C_MAGENTA if s.total_det > 0 else C_GRAY
    parts.append(f"  {C_BLUE}det{C_RST} {det_c}{s.total_det:>4}{C_RST}")

    # Overflows
    if s.overflows > 0:
        parts.append(f"  {C_RED}OVF {s.overflows}{C_RST}")

    # Frequency range
    if s.n_channels > 0:
        f_lo = s.freq_min / 1e6
        f_hi = (s.freq_min + s.freq_step * s.n_channels) / 1e6
        parts.append(f"  {C_CYAN}{f_lo:.3f}\u2013{f_hi:.3f} MHz{C_RST}")

    # Sweep rate
    if s.sweep_rate > 0:
        rc = _sweep_rate_color(s.sweep_rate)
        parts.append(f"  {rc}{s.sweep_rate:.1f} sw/s{C_RST}")

    # Clock
    parts.append(f"  {C_GRAY}{time.strftime('%H:%M:%S')}{C_RST}")

    return "".join(parts) + "\033[K\n"


def _render_footer(s: State, term_w: int) -> str:
    """Detection history: high-precision timestamps, readable labels, full width."""
    lines: list[str] = []
    mono_now = time.monotonic()
    for i, (wall, sweep, det) in enumerate(reversed(s.det_history)):
        freq_mhz = det["freq"] / 1e6
        ratio = det.get("ratio", 0.0)
        rc = _ratio_color(ratio)

        # Format chirp slope as human-readable
        k = det.get("chirp_slope", 0)
        k_str = f"{k / 1000:.0f}k" if k >= 1000 else f"{k:.0f}"
        probe_bw = det.get("probe_bw", 0)
        probe_str = _bw_short(probe_bw) if probe_bw else "?"

        ts = _ts_hires(wall)
        line = (
            f"{C_GRAY}{ts}{C_RST}"
            f"  {C_BLUE}#{sweep:<5}{C_RST}"
            f"  {C_CYAN}{freq_mhz:>7.3f} MHz{C_RST}"
            f"  {C_WHITE}k={k_str}{C_RST} {C_CYAN}@{probe_str}{C_RST}"
            f"  {rc}ratio {ratio:>5.1f}{C_RST}"
        )
        line += "\033[K"

        # Bold the most recent detection if it's within ANCHOR_TTL
        is_recent = len(s.det_history) > 0 and (
            mono_now - s.active_dets[0][0] < ANCHOR_TTL if s.active_dets else False
        )
        if i == 0 and is_recent:
            line = f"{C_BOLD}{line}{C_RST}"

        lines.append(line)
    while len(lines) < DET_HISTORY:
        lines.append("\033[K")
    return "\n".join(lines)


def render_header(s: State) -> None:
    tw, _ = _term_size()
    _write(f"\033[H{_render_header(s, tw)}")


def render(s: State) -> None:
    """Full redraw: status + marker row + frequency axis + bar chart + detection log."""
    if s.energy is None:
        return

    term_w, term_h = _term_size()

    lbl_w = 8
    spec_w = max(10, term_w - lbl_w)
    body_h = max(2, term_h - HEADER_LINES)

    col_e = _resample(s.energy, spec_w)
    n_cols = len(col_e)

    db = [_db(e) for e in col_e]
    nonzero = sorted(d for d, e in zip(db, col_e) if e > 1e-15)
    if nonzero:
        db_min = nonzero[len(nonzero) // 4] - DB_FLOOR_PAD
    else:
        db_min = -100.0
    db_max = max(db) + DB_HEADROOM
    db_range = max(db_max - db_min, 10.0)

    # Bar heights (integer rows, 0..body_h); clamp to body_h
    bar_h = [
        min(body_h, max(0, int((db[c] - db_min) / db_range * body_h + 0.5)))
        for c in range(n_cols)
    ]

    # Per-column gradient color from bar height
    col_color = [_grad_color(bar_h[c] / body_h) for c in range(n_cols)]

    # Detection columns for magenta highlight + vertical labels
    det_cols: set[int] = set()
    # det_labels: col → (freq_str, ratio_color) for vertical overlay
    det_labels: dict[int, tuple[str, str]] = {}
    for _t, det in s.active_dets:
        col = _det_col(det, s.freq_min, s.freq_step, s.n_channels, spec_w)
        det_cols.add(col)
        if col not in det_labels:
            freq_mhz = det.get("freq", 0) / 1e6
            ratio = det.get("ratio", 0.0)
            # Compact label: "869.6" written vertically down the bar
            label = f"{freq_mhz:.1f}"
            det_labels[col] = (label, _ratio_color(ratio))

    # Build vertical label overlay map: (row, col) → (char, color)
    # Labels start from row 0 (top) and go down, sticky at top of bar area
    vlabel_map: dict[tuple[int, int], tuple[str, str]] = {}
    for col, (label, lbl_color) in det_labels.items():
        for i, ch in enumerate(label):
            if i < body_h:
                vlabel_map[(i, col)] = (ch, lbl_color)

    out: list[str] = ["\033[H"]
    out.append(_render_header(s, term_w))

    # ── Marker row: ▼ carets at detection frequencies ──
    marker_line = [" "] * spec_w
    marker_colors: dict[int, str] = {}
    for _t, det in s.active_dets:
        col = _det_col(det, s.freq_min, s.freq_step, s.n_channels, spec_w)
        marker_colors[col] = _ratio_color(det.get("ratio", 0.0))
    # Build the marker row with proper column alignment
    parts: list[str] = []
    parts.append(" " * lbl_w)
    for c in range(spec_w):
        if c in marker_colors:
            parts.append(f"{marker_colors[c]}\u25bc{C_RST}")
        else:
            parts.append(" ")
    out.append("".join(parts) + "\033[K\n")

    # ── Frequency axis ──
    f_lo = s.freq_min / 1e6
    f_hi = (s.freq_min + s.n_channels * s.freq_step) / 1e6
    f_mid = (f_lo + f_hi) / 2
    left = f"{f_lo:.1f}"
    mid = f"{f_mid:.1f}"
    right = f"{f_hi:.1f} MHz"
    pad1 = max(1, spec_w // 2 - len(left) - len(mid) // 2)
    pad2 = max(1, spec_w - len(left) - pad1 - len(mid) - len(right))
    out.append(
        f"{' ' * lbl_w}{C_CYAN}{left}{' ' * pad1}{mid}"
        + f"{' ' * pad2}{right}{C_RST}\033[K\n"
    )

    # ── Body: bar chart with detection highlights + vertical freq labels ──
    for row in range(body_h):
        # dB label on left margin
        if row == 0:
            label = f"{db_max:6.0f}dB"
        elif row == body_h - 1:
            label = f"{db_min:6.0f}dB"
        elif body_h > 6 and row % max(1, body_h // 4) == 0:
            label = f"{db_max - (row / body_h) * db_range:6.0f}dB"
        else:
            label = ""
        out.append(f"{C_GRAY}{label:>{lbl_w}}{C_RST}")

        # RLE fast path with magenta highlight + vertical label overlay
        threshold = body_h - row
        prev_color: str = ""
        run_len = 0
        for col in range(n_cols):
            # Check for vertical label overlay at this position
            vlabel = vlabel_map.get((row, col))
            if vlabel:
                # Flush pending RLE run
                if run_len:
                    out.append(f"{prev_color}{_BAR * run_len}")
                    prev_color = ""
                    run_len = 0
                ch, lc = vlabel
                # White bold char on the bar (or on empty space)
                out.append(f"{C_WHITE}{C_BOLD}{ch}{C_RST}")
            elif bar_h[col] >= threshold and col_e[col] > 1e-15:
                cc = C_MAGENTA if col in det_cols else col_color[col]
                if cc == prev_color:
                    run_len += 1
                else:
                    if run_len:
                        out.append(f"{prev_color}{_BAR * run_len}")
                    prev_color = cc
                    run_len = 1
            else:
                if run_len:
                    out.append(f"{prev_color}{_BAR * run_len}{C_RST}")
                    prev_color = ""
                    run_len = 0
                out.append(" ")
        if run_len:
            out.append(f"{prev_color}{_BAR * run_len}{C_RST}")
        out.append("\033[K\n")

    # ── Footer: detection history ──
    out.append(_render_footer(s, term_w) + "\n")

    out.append("\033[J")
    _write("".join(out))


# ─── Main loop ─────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa spectrum bar graph. Subscribes to lora_scan via UDP.",
    )
    _ = parser.add_argument(
        "--host", default="127.0.0.1", help="lora_scan UDP host (default: 127.0.0.1)"
    )
    _ = parser.add_argument(
        "--port", type=int, default=5557, help="lora_scan UDP port (default: 5557)"
    )
    _ = parser.add_argument(
        "--ema",
        type=float,
        default=EMA_ALPHA,
        help=f"EMA smoothing alpha (default: {EMA_ALPHA})",
    )
    args = parser.parse_args()

    _write("\033[?25l")  # hide cursor

    sock, _sub_msg, _addr = create_udp_subscriber(args.host, args.port)

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

    header_types = {
        "scan_status",
        "scan_sweep_start",
        "scan_sweep_end",
        "scan_l2_probe",
        "scan_overflow",
    }

    try:
        while True:
            data, _ = sock.recvfrom(65536)
            try:
                msg = cbor2.loads(data)
            except Exception:
                continue  # skip malformed CBOR
            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type", "")
            handler = dispatch.get(msg_type)
            if handler:
                handler(msg)

            if msg_type == "scan_spectrum":
                if first:
                    _write("\033[2J")
                    first = False
                render(state)
            elif msg_type in header_types and not first:
                render_header(state)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
