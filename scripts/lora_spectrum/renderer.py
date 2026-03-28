#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Terminal rendering for lora_spectrum."""

from __future__ import annotations

import io
import logging
import math
import os
import sys
from typing import TYPE_CHECKING, BinaryIO

if TYPE_CHECKING:
    from .config import SpectrumConfig
    from .state import Detection, PeakHold, SpectrumState

logger = logging.getLogger(__name__)

# ANSI escape codes
C_RESET = "\033[0m"
C_DIM = "\033[2m"
C_BOLD = "\033[1m"

# Named colors (xterm-256)
C_CYAN = "\033[38;5;87m"
C_GREEN = "\033[38;5;46m"
C_YELLOW = "\033[38;5;226m"
C_ORANGE = "\033[38;5;214m"
C_RED = "\033[38;5;196m"
C_BLUE = "\033[38;5;39m"
C_WHITE = "\033[38;5;255m"
C_GRAY = "\033[38;5;242m"
C_MAGENTA = "\033[38;5;213m"

# Energy gradient (index 0 = weakest, index N-1 = strongest)
ENERGY_GRADIENT = [
    "\033[38;5;17m",  # dark navy
    "\033[38;5;24m",  # dark blue
    "\033[38;5;30m",  # teal
    "\033[38;5;35m",  # dark green
    "\033[38;5;40m",  # bright green
    "\033[38;5;154m",  # lime
    "\033[38;5;226m",  # yellow
    "\033[38;5;214m",  # orange
    "\033[38;5;202m",  # dark orange
    "\033[38;5;196m",  # red
]

BAR_CHAR = "\u2588"  # █


def to_db(value: float) -> float:
    """Convert linear value to dB."""
    return 10.0 * math.log10(max(value, 1e-12))


def resample(energy: list[float], n_cols: int) -> list[float]:
    """Map n_channels energy values to n_cols columns."""
    n = len(energy)
    if n == 0:
        return [0.0] * n_cols

    if n <= n_cols:
        # Expand: replicate values
        out: list[float] = []
        for i, e in enumerate(energy):
            lo = i * n_cols // n
            hi = (i + 1) * n_cols // n
            out.extend([e] * (hi - lo))
        return out[:n_cols]
    else:
        # Shrink: average values
        out = []
        for c in range(n_cols):
            lo = c * n // n_cols
            hi = (c + 1) * n // n_cols
            out.append(sum(energy[lo:hi]) / max(1, hi - lo))
        return out


def gradient_color(frac: float) -> str:
    """Get gradient color for a fraction in [0, 1]."""
    idx = min(int(frac * len(ENERGY_GRADIENT)), len(ENERGY_GRADIENT) - 1)
    return ENERGY_GRADIENT[idx]


def ratio_color(ratio: float) -> str:
    """Get color for detection ratio."""
    if ratio >= 50:
        return C_GREEN
    if ratio >= 20:
        return C_YELLOW
    return C_ORANGE


def sweep_rate_color(rate: float) -> str:
    """Get color for sweep rate."""
    if rate >= 0.8:
        return C_GREEN
    if rate >= 0.4:
        return C_YELLOW
    return C_RED


def sweep_duration_color(ms: int) -> str:
    """Get color for sweep duration."""
    if ms < 1500:
        return C_GREEN
    if ms < 3000:
        return C_YELLOW
    return C_RED


class TerminalRenderer:
    """Renders spectrum state to terminal."""

    def __init__(self, config: "SpectrumConfig") -> None:
        """Initialize renderer."""
        self._config = config
        self._tty: BinaryIO | None = None
        self._first_render = True

        # Terminal dimensions cache
        self._term_width: int = 80
        self._term_height: int = 24

        logger.debug("TerminalRenderer initialized")

    def _get_tty(self) -> BinaryIO:
        """Get or create TTY file handle."""
        if self._tty is None:
            try:
                fd = os.open("/dev/tty", os.O_WRONLY)
                self._tty = io.FileIO(fd, mode="wb")
            except OSError:
                self._tty = sys.stderr.buffer
        # Type assertion: self._tty is now definitely set
        assert self._tty is not None
        return self._tty

    def _write(self, s: str) -> None:
        """Write string to TTY."""
        self._get_tty().write(s.encode())
        self._get_tty().flush()

    def _get_terminal_size(self) -> tuple[int, int]:
        """Get current terminal dimensions."""
        try:
            fd = os.open("/dev/tty", os.O_RDONLY)
            size = os.get_terminal_size(fd)
            os.close(fd)
            return size.columns, size.lines
        except OSError:
            return 80, 24

    def clear_screen(self) -> None:
        """Clear the terminal screen."""
        self._write("\033[2J")
        self._first_render = False

    def cleanup(self) -> None:
        """Cleanup terminal state."""
        try:
            self._write("\033[?25h\033[0m\n")  # Show cursor, reset colors
        except OSError:
            pass

    def render(self, state: "SpectrumState") -> None:
        """Render full spectrum display."""
        if state.energy is None or state.raw_energy is None:
            return

        # Get terminal size
        self._term_width, self._term_height = self._get_terminal_size()

        # Calculate layout
        label_width = 8
        spec_width = max(10, self._term_width - label_width)
        header_lines = 2 + 1  # Status + freq axis + legend
        body_height = max(2, self._term_height - header_lines - 1)

        # Resample energy to terminal columns
        col_smooth = resample(state.energy, spec_width)
        col_raw = resample(state.raw_energy, spec_width)
        n_cols = len(col_smooth)

        # Calculate dB scale
        db_raw = [to_db(e) for e in col_raw]
        nonzero = sorted(d for d, e in zip(db_raw, col_raw) if e > 1e-15)
        if nonzero:
            db_min = nonzero[len(nonzero) // 4] - self._config.db_floor_pad
        else:
            db_min = -100.0
        db_max = max(db_raw) + self._config.db_headroom
        db_range = max(db_max - db_min, 10.0)

        # Calculate bar heights
        bar_heights = [
            min(
                body_height,
                max(0, int((db_raw[c] - db_min) / db_range * body_height + 0.5)),
            )
            for c in range(n_cols)
        ]

        # Calculate gradient colors
        db_smooth = [to_db(e) for e in col_smooth]
        db_min_s = min(db_smooth)
        db_max_s = max(db_smooth)
        db_range_s = max(db_max_s - db_min_s, 10.0)
        col_colors = [
            gradient_color((db_smooth[c] - db_min_s) / db_range_s)
            for c in range(n_cols)
        ]

        # Get detection columns
        det_cols = self._get_detection_columns(state, n_cols)

        # Get peak hold markers
        peak_holds = self._get_peak_hold_rows(
            state, db_min, db_range, body_height, n_cols
        )

        # Build output
        lines = ["\033[H"]  # Move cursor to top-left

        # Header
        lines.append(self._render_header(state, self._term_width))

        # Frequency axis
        lines.append(self._render_freq_axis(state, label_width, spec_width))

        # Bar chart
        lines.extend(
            self._render_bars(
                body_height,
                label_width,
                n_cols,
                bar_heights,
                col_colors,
                det_cols,
                peak_holds,
                db_min,
                db_max,
                db_range,
            )
        )

        # Footer
        lines.append(self._render_footer(state, label_width, spec_width))

        # Clear to end of screen
        lines.append("\033[J")

        # Output
        self._write("".join(lines))

    def _render_header(self, state: "SpectrumState", width: int) -> str:
        """Render status header line."""
        parts: list[str] = []

        # Script name
        parts.append(f"{C_WHITE}lora_spectrum{C_RESET}")

        # Sweep info
        sw_str = f"{state.sweep_count:>5}"
        if state.sweep_rate > 0:
            rate_color = sweep_rate_color(state.sweep_rate)
            rate_str = f" {rate_color}{state.sweep_rate:.1f}{C_RESET}"
        else:
            rate_str = ""
        parts.append(
            f"  {C_BLUE}sw{C_RESET} {C_WHITE}{sw_str}{C_RESET}  {C_BLUE}sw/s{C_RESET}{rate_str}"
        )

        # Duration
        if state.last_sweep_duration_ms > 0:
            dur_color = sweep_duration_color(state.last_sweep_duration_ms)
            parts.append(f"  {dur_color}{state.last_sweep_duration_ms:>5}ms{C_RESET}")

        # Detections
        det_color = C_MAGENTA if state.total_detections > 0 else C_GRAY
        parts.append(
            f"  {C_BLUE}det{C_RESET} {det_color}{state.total_detections:>4}{C_RESET}"
        )

        # Overflows
        if state.overflows > 0:
            parts.append(f"  {C_RED}OVF {state.overflows}{C_RESET}")

        # Frequency range
        if state.n_channels > 0:
            f_lo = state.freq_min / 1e6
            f_hi = state.freq_max / 1e6
            parts.append(f"  {C_CYAN}{f_lo:.3f}\u2013{f_hi:.3f} MHz{C_RESET}")

        # Clock
        import time

        parts.append(f"  {C_GRAY}{time.strftime('%H:%M:%S')}{C_RESET}")

        return "".join(parts) + "\033[K\n"

    def _render_freq_axis(
        self, state: "SpectrumState", label_w: int, spec_w: int
    ) -> str:
        """Render frequency axis line."""
        if state.n_channels == 0 or state.freq_step == 0:
            return " " * (label_w + spec_w) + "\033[K\n"

        f_lo = state.freq_min / 1e6
        f_hi = state.freq_max / 1e6
        f_mid = (f_lo + f_hi) / 2

        left = f"{f_lo:.1f}"
        mid = f"{f_mid:.1f}"
        right = f"{f_hi:.1f} MHz"

        pad1 = max(1, spec_w // 2 - len(left) - len(mid) // 2)
        pad2 = max(1, spec_w - len(left) - pad1 - len(mid) - len(right))

        return f"{' ' * label_w}{C_CYAN}{left}{' ' * pad1}{mid}{' ' * pad2}{right}{C_RESET}\033[K\n"

    def _get_detection_columns(self, state: "SpectrumState", n_cols: int) -> set[int]:
        """Calculate which columns have detections."""
        det_cols: set[int] = set()
        for _, det in state.active_detections:
            if state.freq_step > 0 and det.freq > 0:
                ch_idx = (det.freq - state.freq_min) / state.freq_step
                col = int(ch_idx * n_cols / state.n_channels)
                col = max(0, min(n_cols - 1, col))
                det_cols.add(col)
        return det_cols

    def _get_peak_hold_rows(
        self,
        state: "SpectrumState",
        db_min: float,
        db_range: float,
        body_h: int,
        spec_w: int,
    ) -> dict[int, tuple[int, str, str]]:
        """Get peak hold markers grouped by clusters."""
        import time

        now = time.monotonic()
        raw_peaks: dict[int, tuple[int, str, str]] = {}

        for ph in state.peak_holds:
            age = now - ph.timestamp
            if age >= self._config.peak_hold_s:
                continue

            col = (
                int(ph.channel * spec_w / state.n_channels)
                if state.n_channels > 0
                else 0
            )
            col = max(0, min(col, spec_w - 1))

            peak_row = body_h - int((ph.db_value - db_min) / db_range * body_h + 0.5)
            peak_row = max(0, min(peak_row, body_h - 1))

            if col not in raw_peaks or peak_row < raw_peaks[col][0]:
                raw_peaks[col] = (peak_row, ph.label, ph.label_color)

        # Cluster adjacent markers
        if not raw_peaks:
            return {}

        result: dict[int, tuple[int, str, str]] = {}
        sorted_peaks = sorted(raw_peaks.items())
        clusters: list[list[tuple[int, tuple[int, str, str]]]] = []

        for item in sorted_peaks:
            if clusters and item[0] - clusters[-1][-1][0] <= 2:
                clusters[-1].append(item)
            else:
                clusters.append([item])

        for cluster in clusters:
            median_col = cluster[len(cluster) // 2][0]
            best_row = min(c[1][0] for c in cluster)
            best_entry = min(cluster, key=lambda c: c[1][0])
            label, lbl_color = best_entry[1][1], best_entry[1][2]
            result[median_col] = (best_row, label, lbl_color)

        return result

    def _render_bars(
        self,
        body_h: int,
        label_w: int,
        n_cols: int,
        bar_heights: list[int],
        col_colors: list[str],
        det_cols: set[int],
        peak_holds: dict[int, tuple[int, str, str]],
        db_min: float,
        db_max: float,
        db_range: float,
    ) -> list[str]:
        """Render bar chart body."""
        lines: list[str] = []

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

            lines.append(f"{C_GRAY}{label:>{label_w}}{C_RESET}")

            # Build overlays for this row
            row_overlays: dict[int, tuple[str, str]] = {}
            for col, (marker_row, lbl, color) in peak_holds.items():
                if marker_row == row:
                    row_overlays[col] = ("\u25bc", C_MAGENTA)  # ▼
                if lbl and marker_row - 1 == row:
                    for i, ch in enumerate(lbl):
                        c = col + i
                        if 0 <= c < n_cols and c not in row_overlays:
                            row_overlays[c] = (ch, color)

            # RLE render bars
            threshold = body_h - row
            prev_color = ""
            run_len = 0

            for col in range(n_cols):
                has_bar = bar_heights[col] >= threshold
                overlay = row_overlays.get(col)

                if overlay and not has_bar:
                    # Draw overlay character
                    if run_len:
                        lines.append(f"{prev_color}{BAR_CHAR * run_len}{C_RESET}")
                        prev_color = ""
                        run_len = 0
                    ch, oc = overlay
                    lines.append(f"{oc}{ch}{C_RESET}")
                elif has_bar:
                    # Draw bar
                    cc = C_MAGENTA if col in det_cols else col_colors[col]
                    if cc == prev_color:
                        run_len += 1
                    else:
                        if run_len:
                            lines.append(f"{prev_color}{BAR_CHAR * run_len}")
                        prev_color = cc
                        run_len = 1
                else:
                    # Empty space
                    if run_len:
                        lines.append(f"{prev_color}{BAR_CHAR * run_len}{C_RESET}")
                        prev_color = ""
                        run_len = 0
                    lines.append(" ")

            if run_len:
                lines.append(f"{prev_color}{BAR_CHAR * run_len}{C_RESET}")

            lines.append("\033[K\n")

        return lines

    def _render_footer(self, state: "SpectrumState", label_w: int, spec_w: int) -> str:
        """Render footer line with frequency legend."""
        if state.n_channels == 0 or state.freq_step == 0:
            return " " * (label_w + spec_w) + "\033[K\n"

        f_lo = state.freq_min / 1e6
        f_hi = state.freq_max / 1e6
        f_range = f_hi - f_lo

        # Place tick labels
        n_ticks = max(2, spec_w // 12)
        buf = [" "] * spec_w
        for i in range(n_ticks + 1):
            frac = i / n_ticks
            freq = f_lo + frac * f_range
            label = f"{freq:.1f}"
            col = int(frac * (spec_w - 1))
            start = max(0, min(col - len(label) // 2, spec_w - len(label)))
            for j, ch in enumerate(label):
                if start + j < spec_w:
                    buf[start + j] = ch

        return f"{' ' * label_w}{C_CYAN}{''.join(buf)}{C_RESET}\033[K\n"


def create_renderer(config: "SpectrumConfig") -> TerminalRenderer:
    """Factory function to create renderer."""
    return TerminalRenderer(config)
