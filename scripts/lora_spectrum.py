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

EMA_ALPHA = 0.3  # spectrum smoothing factor
DB_HEADROOM = 1.0  # dB padding above peak
DB_FLOOR_PAD = 3.0  # dB padding below noise floor (25th percentile)
ANCHOR_TTL = 3.0  # seconds to keep detection anchors visible
PEAK_HOLD_S = 5.0  # seconds to hold peak detection bar height
HEADER_LINES = 2 + 1  # status line + bar chart freq axis + freq legend

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
        self.energy: list[float] | None = None  # EMA-smoothed (for color gradient)
        self.raw_energy: list[float] | None = None  # unsmoothed (for bar heights)
        self.freq_min: int = 0
        self.freq_step: int = 62500
        self.n_channels: int = 0
        self.hot: set[int] = set()
        self.det_history: list[
            tuple[float, int, dict[str, Any]]
        ] = []  # unused, kept for future
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

        # Peak hold: per-channel peak dB values that decay slowly.
        # When a detection occurs, the peak hold is set to the current dB
        # at the detection column and drawn as a bright horizontal line
        # that slowly drops over PEAK_HOLD_S seconds.
        # List of (column_index_in_channels, peak_db, monotonic_time)
        self.peak_holds: list[tuple[int, float, float]] = []

    def on_spectrum(self, msg: dict[str, Any]) -> None:
        n = msg["n_channels"]
        raw = list(struct.unpack(f"<{n}f", msg["channels"]))

        # raw_energy: unsmoothed — used for bar heights (actual power level)
        self.raw_energy = raw

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
            # Register peak hold at the detection's channel index
            freq = det.get("freq", 0)
            if self.freq_step > 0 and freq > 0:
                ch = int((freq - self.freq_min) / self.freq_step + 0.5)
                ch = max(0, min(ch, n - 1))
                if self.raw_energy:
                    db_val = _db(self.raw_energy[ch])
                    self.peak_holds.append((ch, db_val, mono))
        # Prune anchors older than ANCHOR_TTL
        self.active_dets = [
            (t, d) for t, d in self.active_dets if mono - t < ANCHOR_TTL
        ]
        # Prune peak holds older than PEAK_HOLD_S
        self.peak_holds = [
            (ch, db, t) for ch, db, t in self.peak_holds if mono - t < PEAK_HOLD_S
        ]
        self.det_history = self.det_history[-20:]

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


def _render_freq_legend(s: State, lbl_w: int, spec_w: int) -> str:
    """One-line frequency legend spanning the full spectrum width."""
    if s.n_channels == 0 or s.freq_step == 0:
        return " " * (lbl_w + spec_w) + "\033[K"

    f_lo = s.freq_min / 1e6
    f_hi = (s.freq_min + s.freq_step * s.n_channels) / 1e6
    f_range = f_hi - f_lo

    # Place tick labels at regular intervals; aim for one every ~12 chars
    n_ticks = max(2, spec_w // 12)
    buf = [" "] * spec_w
    for i in range(n_ticks + 1):
        frac = i / n_ticks
        freq = f_lo + frac * f_range
        label = f"{freq:.1f}"
        # Centre-align label at the column position, clamped to buffer edges
        col = int(frac * (spec_w - 1))
        start = max(0, min(col - len(label) // 2, spec_w - len(label)))
        for j, ch in enumerate(label):
            if start + j < spec_w:
                buf[start + j] = ch

    return f"{' ' * lbl_w}{C_CYAN}{''.join(buf)}{C_RST}\033[K"


def render_header(s: State) -> None:
    tw, _ = _term_size()
    _write(f"\033[H{_render_header(s, tw)}")


def render(s: State) -> None:
    """Full redraw: status + marker row + frequency axis + bar chart + detection log."""
    if s.energy is None or s.raw_energy is None:
        return

    term_w, term_h = _term_size()

    lbl_w = 8
    spec_w = max(10, term_w - lbl_w)
    # -1 safety margin so bars never write into header or footer on resize races
    body_h = max(2, term_h - HEADER_LINES - 1)

    # EMA-smoothed energy for color gradient; raw energy for bar heights
    col_e_smooth = _resample(s.energy, spec_w)
    col_e_raw = _resample(s.raw_energy, spec_w)
    n_cols = len(col_e_smooth)

    # Scale derived from raw energy so the dB axis reflects actual power
    db_raw = [_db(e) for e in col_e_raw]
    nonzero = sorted(d for d, e in zip(db_raw, col_e_raw) if e > 1e-15)
    if nonzero:
        db_min = nonzero[len(nonzero) // 4] - DB_FLOOR_PAD
    else:
        db_min = -100.0
    db_max = max(db_raw) + DB_HEADROOM
    db_range = max(db_max - db_min, 10.0)

    # Bar heights from raw (actual) power level
    bar_h = [
        min(body_h, max(0, int((db_raw[c] - db_min) / db_range * body_h + 0.5)))
        for c in range(n_cols)
    ]

    # Per-column gradient color from smoothed energy (nicer visuals)
    db_smooth = [_db(e) for e in col_e_smooth]
    db_min_s = min(db_smooth)
    db_max_s = max(db_smooth)
    db_range_s = max(db_max_s - db_min_s, 10.0)
    col_color = [
        _grad_color((db_smooth[c] - db_min_s) / db_range_s) for c in range(n_cols)
    ]

    # Detection columns for magenta highlight
    det_cols: set[int] = set()
    # Collect all detection columns with freq/ratio for clustering
    det_label_candidates: list[tuple[int, float, float]] = []
    for _t, det in s.active_dets:
        col = _det_col(det, s.freq_min, s.freq_step, s.n_channels, spec_w)
        det_cols.add(col)
        freq_mhz = det.get("freq", 0) / 1e6
        ratio = det.get("ratio", 0.0)
        det_label_candidates.append((col, freq_mhz, ratio))

    # Cluster adjacent detection columns (within ±2 cols) → one label per cluster
    # at the median column, using average freq and best ratio.
    det_label_candidates.sort()
    clusters: list[list[tuple[int, float, float]]] = []
    for item in det_label_candidates:
        if clusters and item[0] - clusters[-1][-1][0] <= 2:
            clusters[-1].append(item)
        else:
            clusters.append([item])

    det_labels: dict[int, tuple[str, str]] = {}
    for cluster in clusters:
        cols = [c[0] for c in cluster]
        median_col = cols[len(cols) // 2]
        avg_freq = sum(c[1] for c in cluster) / len(cluster)
        best_ratio = max(c[2] for c in cluster)
        label = f"{avg_freq:.1f}"
        det_labels[median_col] = (label, _ratio_color(best_ratio))

    # Peak hold: collect per-column peak rows, then cluster adjacent columns
    _peak_raw: dict[int, int] = {}  # col → peak_row (highest = smallest row)
    mono_now = time.monotonic()
    for ch_idx, peak_db, t in s.peak_holds:
        age = mono_now - t
        if age >= PEAK_HOLD_S:
            continue
        col = int(ch_idx * spec_w / s.n_channels) if s.n_channels > 0 else 0
        col = max(0, min(col, spec_w - 1))
        peak_row = body_h - int((peak_db - db_min) / db_range * body_h + 0.5)
        peak_row = max(0, min(peak_row, body_h - 1))
        if col not in _peak_raw or peak_row < _peak_raw[col]:
            _peak_raw[col] = peak_row

    # Cluster adjacent peak hold columns (±2) → single marker at median
    # peak_hold_rows: col → (row, freq_label, label_color)
    peak_hold_rows: dict[int, tuple[int, str, str]] = {}
    if _peak_raw:
        sorted_peaks = sorted(_peak_raw.items())
        pk_clusters: list[list[tuple[int, int]]] = []
        for item in sorted_peaks:
            if pk_clusters and item[0] - pk_clusters[-1][-1][0] <= 2:
                pk_clusters[-1].append(item)
            else:
                pk_clusters.append([item])
        for cluster in pk_clusters:
            median_col = cluster[len(cluster) // 2][0]
            best_row = min(c[1] for c in cluster)  # highest peak
            # Attach freq label from det_labels if available for this cluster
            label, lbl_color = det_labels.get(median_col, ("", C_MAGENTA))
            if not label:
                # fall back: search nearby cols within ±3
                for dc in range(1, 4):
                    for mc in (median_col - dc, median_col + dc):
                        if mc in det_labels:
                            label, lbl_color = det_labels[mc]
                            break
                    if label:
                        break
            peak_hold_rows[median_col] = (best_row, label, lbl_color)

    out: list[str] = ["\033[H"]
    out.append(_render_header(s, term_w))

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

        # Build per-column char map for this row from peak hold markers + labels.
        # peak_hold_rows: col → (marker_row, freq_label, color)
        row_overlays: dict[int, tuple[str, str]] = {}  # col → (char, color)
        for mc, (marker_row, freq_label, lbl_color) in peak_hold_rows.items():
            if marker_row == row:
                row_overlays[mc] = ("\u25bc", C_MAGENTA)  # ▼ at marker row
            if freq_label and marker_row - 1 == row:
                # Freq label one row above the ▼, left-anchored at marker col
                for i, ch in enumerate(freq_label):
                    c = mc + i
                    if 0 <= c < n_cols and c not in row_overlays:
                        row_overlays[c] = (ch, lbl_color)

        # RLE fast path with magenta highlight and peak hold markers
        threshold = body_h - row
        prev_color: str = ""
        run_len = 0
        for col in range(n_cols):
            has_bar = bar_h[col] >= threshold and col_e_raw[col] > 1e-15
            overlay = row_overlays.get(col)

            if overlay and not has_bar:
                # Overlay (marker or label) in empty space above the bar
                if run_len:
                    out.append(f"{prev_color}{_BAR * run_len}{C_RST}")
                    prev_color = ""
                    run_len = 0
                ch, oc = overlay
                out.append(f"{oc}{ch}{C_RST}")
            elif has_bar:
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

    # ── Footer: frequency legend ──
    out.append(_render_freq_legend(s, lbl_w, spec_w) + "\n")

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
