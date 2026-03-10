#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# Requires Python >= 3.14
"""
lora_waterfall.py -- Console waterfall display for lora_trx.

Connects to lora_trx via UDP, receives spectrum CBOR messages, and renders
a scrolling waterfall in the terminal using xterm-256color escape codes.
Rows are tinted green (CRC_OK) or red (CRC_FAIL) during frame decodes.
The dB color scale auto-adjusts based on RX gain from status messages and
tracks the actual noise floor from spectrum data.

Mouse scroll support: scroll up to pause and browse history, scroll down
(or past the end) to resume live mode.  Requires a terminal that supports
SGR mouse mode (wezterm, iTerm2, kitty, xterm, etc.).

Usage:
    lora_waterfall.py                          # default: 127.0.0.1:5555
    lora_waterfall.py --connect 192.168.1.10:5555
    lora_waterfall.py --bw-factor 1.5          # show 1.5x bandwidth
"""

from __future__ import annotations

import argparse
import logging
import os
import re
import select
import sys
import termios
import time
import tty
from collections import deque
from typing import Any

import cbor2

from lora_common import (
    KEEPALIVE_INTERVAL,
    config_udp_host,
    config_udp_port,
    create_udp_subscriber,
    format_ascii,
    load_config,
    parse_host_port,
    setup_logging,
    sync_word_name,
)

log = logging.getLogger("gr4.waterfall")


# ---- xterm-256color palette for waterfall ----
# Map dB range to color ramp: dark navy -> teal -> green -> yellow -> warm orange
# Using xterm palette indices 16-231 (6x6x6 color cube)
# Avoids purple/violet tones for a more eye-soothing display.


def _cube(r: int, g: int, b: int) -> int:
    """xterm-256 6x6x6 cube index from r,g,b (each 0-5)."""
    return 16 + 36 * r + 6 * g + b


def _build_ramp(stops: list[tuple[int, int, int]]) -> list[int]:
    """Convert a list of (r, g, b) tuples (0-5 each) to xterm-256 indices."""
    return [_cube(r, g, b) for r, g, b in stops]


# --- Normal palette: navy → teal → green → yellow → warm orange ---
RAMP_NORMAL: list[int] = _build_ramp(
    [
        # Dark navy (0-7)
        (0, 0, 1),
        (0, 0, 1),
        (0, 0, 2),
        (0, 0, 2),
        (0, 1, 2),
        (0, 1, 2),
        (0, 1, 3),
        (0, 1, 3),
        # Teal (8-19)
        (0, 2, 3),
        (0, 2, 3),
        (0, 2, 2),
        (0, 2, 2),
        (0, 3, 2),
        (0, 3, 2),
        (0, 3, 1),
        (0, 3, 1),
        (0, 3, 0),
        (0, 3, 0),
        (0, 4, 0),
        (0, 4, 0),
        # Green (20-31)
        (0, 4, 0),
        (0, 5, 0),
        (0, 5, 0),
        (1, 5, 0),
        (1, 5, 0),
        (1, 5, 0),
        (2, 5, 0),
        (2, 5, 0),
        (2, 5, 0),
        (3, 5, 0),
        (3, 5, 0),
        (3, 5, 0),
        # Yellow (32-47)
        (3, 5, 0),
        (4, 5, 0),
        (4, 5, 0),
        (4, 5, 0),
        (4, 5, 0),
        (5, 5, 0),
        (5, 5, 0),
        (5, 5, 0),
        (5, 5, 0),
        (5, 5, 0),
        (5, 4, 0),
        (5, 4, 0),
        (5, 4, 0),
        (5, 4, 0),
        (5, 3, 0),
        (5, 3, 0),
        # Warm orange (48-63)
        (5, 3, 0),
        (5, 3, 0),
        (5, 2, 0),
        (5, 2, 0),
        (5, 2, 0),
        (5, 2, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 0, 0),
        (5, 0, 0),
        (5, 0, 0),
    ]
)

# --- CRC_OK palette: dark teal → cyan → bright aqua → white-cyan ---
RAMP_DECODE_OK: list[int] = _build_ramp(
    [
        # Dark teal (0-7)
        (0, 1, 1),
        (0, 1, 1),
        (0, 1, 2),
        (0, 1, 2),
        (0, 2, 2),
        (0, 2, 2),
        (0, 2, 3),
        (0, 2, 3),
        # Teal-cyan (8-19)
        (0, 2, 3),
        (0, 3, 3),
        (0, 3, 3),
        (0, 3, 4),
        (0, 3, 4),
        (0, 3, 4),
        (0, 4, 4),
        (0, 4, 4),
        (0, 4, 5),
        (0, 4, 5),
        (0, 5, 5),
        (0, 5, 5),
        # Bright cyan (20-31)
        (0, 5, 5),
        (0, 5, 5),
        (1, 5, 5),
        (1, 5, 5),
        (1, 5, 5),
        (2, 5, 5),
        (2, 5, 5),
        (2, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        # Light cyan (32-47)
        (3, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        # White-cyan (48-63)
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
    ]
)

# --- CRC_FAIL palette: dark red → red → orange-red ---
RAMP_DECODE_FAIL: list[int] = _build_ramp(
    [
        # Dark red (0-7)
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        # Medium red (8-19)
        (2, 0, 0),
        (2, 0, 0),
        (3, 0, 0),
        (3, 0, 0),
        (3, 0, 0),
        (3, 0, 0),
        (3, 1, 0),
        (3, 1, 0),
        (4, 0, 0),
        (4, 0, 0),
        (4, 1, 0),
        (4, 1, 0),
        # Bright red (20-31)
        (4, 1, 0),
        (4, 1, 0),
        (5, 0, 0),
        (5, 0, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 1, 0),
        (5, 2, 0),
        (5, 2, 0),
        (5, 2, 0),
        (5, 2, 0),
        # Orange-red (32-47)
        (5, 2, 0),
        (5, 2, 1),
        (5, 3, 0),
        (5, 3, 0),
        (5, 3, 1),
        (5, 3, 1),
        (5, 3, 1),
        (5, 4, 1),
        (5, 4, 1),
        (5, 4, 2),
        (5, 4, 2),
        (5, 4, 2),
        (5, 4, 3),
        (5, 5, 3),
        (5, 5, 3),
        (5, 5, 4),
        # Light orange-red (48-63)
        (5, 5, 4),
        (5, 5, 4),
        (5, 5, 4),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
    ]
)

# --- TX ack palette: dark purple → blue → light blue ---
RAMP_TX: list[int] = _build_ramp(
    [
        # Dark purple (0-7)
        (1, 0, 1),
        (1, 0, 1),
        (1, 0, 2),
        (1, 0, 2),
        (1, 0, 2),
        (1, 0, 3),
        (1, 0, 3),
        (1, 0, 3),
        # Blue-purple (8-19)
        (1, 0, 3),
        (1, 0, 4),
        (1, 0, 4),
        (1, 1, 4),
        (1, 1, 4),
        (1, 1, 5),
        (1, 1, 5),
        (1, 1, 5),
        (0, 1, 5),
        (0, 2, 5),
        (0, 2, 5),
        (0, 2, 5),
        # Blue (20-31)
        (0, 2, 5),
        (0, 3, 5),
        (0, 3, 5),
        (0, 3, 5),
        (1, 3, 5),
        (1, 3, 5),
        (1, 4, 5),
        (1, 4, 5),
        (1, 4, 5),
        (2, 4, 5),
        (2, 4, 5),
        (2, 4, 5),
        # Light blue (32-47)
        (2, 5, 5),
        (2, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        (3, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (4, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        # White-blue (48-63)
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
        (5, 5, 5),
    ]
)

N_COLORS: int = len(RAMP_NORMAL)

# Palette constants for render_row
PALETTE_NORMAL: int = 0
PALETTE_DECODE_OK: int = 1
PALETTE_DECODE_FAIL: int = 2
PALETTE_TX: int = 3
PALETTES: list[list[int]] = [RAMP_NORMAL, RAMP_DECODE_OK, RAMP_DECODE_FAIL, RAMP_TX]

# Header occupies 3 fixed lines at the top
HEADER_LINES: int = 3

# EMA alpha for noise floor tracking from spectrum data
_NOISE_FLOOR_ALPHA: float = 0.05

# Maximum number of rendered rows to keep for scroll-back history
HISTORY_SIZE: int = 2000

# Lines scrolled per mouse wheel notch
SCROLL_LINES: int = 3


def db_to_color_index(db: float, db_min: float, db_max: float) -> int:
    """Map a dB value to a color ramp index [0, N_COLORS-1]."""
    if db <= db_min:
        return 0
    if db >= db_max:
        return N_COLORS - 1
    frac = (db - db_min) / (db_max - db_min)
    return min(int(frac * N_COLORS), N_COLORS - 1)


def render_row(
    bins: list[float],
    width: int,
    db_min: float,
    db_max: float,
    center_idx: int,
    visible_bins: int,
    *,
    palette: int = PALETTE_NORMAL,
) -> str:
    """Render one waterfall row as xterm-256color background-colored spaces.

    palette selects the color ramp: PALETTE_NORMAL, PALETTE_DECODE_OK,
    PALETTE_DECODE_FAIL, or PALETTE_TX.
    """
    half = visible_bins // 2
    start = max(0, center_idx - half)
    end = min(len(bins), start + visible_bins)
    start = max(0, end - visible_bins)
    visible = bins[start:end]

    n = len(visible)
    if n == 0:
        return " " * width

    ramp = PALETTES[palette]
    parts: list[str] = []
    for col in range(width):
        src_idx = min(col * n // width, n - 1)
        ci = db_to_color_index(visible[src_idx], db_min, db_max)
        parts.append(f"\033[48;5;{ramp[ci]}m ")
    parts.append("\033[0m")
    return "".join(parts)


def get_terminal_size() -> tuple[int, int]:
    """Get terminal (columns, rows), default (80, 24)."""
    try:
        sz = os.get_terminal_size()
        return sz.columns, sz.lines
    except (AttributeError, ValueError, OSError):
        return 80, 24


def format_freq_axis(
    center_freq_mhz: float,
    sample_rate_hz: float,
    bw_factor: float,
    width: int,
) -> str:
    """Format a frequency axis string spanning the visible bandwidth."""
    visible_bw = sample_rate_hz * bw_factor
    f_lo = center_freq_mhz - visible_bw / 2e6
    f_hi = center_freq_mhz + visible_bw / 2e6

    lo_str = f"{f_lo:.3f}"
    mid_str = f"{center_freq_mhz:.3f}"
    hi_str = f"{f_hi:.3f}"

    if width < 30:
        return mid_str.center(width)

    axis = [" "] * width
    for i, ch in enumerate(lo_str):
        if i < width:
            axis[i] = ch
    r_start = width - len(hi_str)
    for i, ch in enumerate(hi_str):
        if r_start + i < width:
            axis[r_start + i] = ch
    c_start = (width - len(mid_str)) // 2
    for i, ch in enumerate(mid_str):
        if 0 <= c_start + i < width:
            axis[c_start + i] = ch
    return "".join(axis)


def format_db_legend(db_min: float, db_max: float) -> str:
    """Format a short dB scale legend with color samples."""
    n_samples = min(16, N_COLORS)
    parts = [f" {db_min:.0f}dB "]
    for i in range(n_samples):
        ci = i * (N_COLORS - 1) // (n_samples - 1)
        color = RAMP_NORMAL[ci]
        parts.append(f"\033[48;5;{color}m \033[0m")
    parts.append(f" {db_max:.0f}dB")
    return "".join(parts)


def _ansi_visible_len(s: str) -> int:
    """Return the visible length of a string containing ANSI escape codes."""
    return len(re.sub(r"\033\[[^m]*m", "", s))


def format_header(
    center_freq_mhz: float,
    sample_rate: float,
    bw_factor: float,
    db_min: float,
    db_max: float,
    fft_size: int,
    total_width: int,
    rx_gain: float | None,
    last_decode: dict[str, Any] | None,
    last_status: dict[str, Any] | None,
    *,
    paused: bool = False,
    scroll_offset: int = 0,
    history_len: int = 0,
) -> str:
    """Build the fixed 3-line header string (no I/O, no flush)."""
    buf: list[str] = []

    # Line 1: frequency axis
    buf.append("\033[1;1H\033[2K")
    axis = format_freq_axis(center_freq_mhz, sample_rate, bw_factor, total_width)
    buf.append(f"\033[1m{axis}\033[0m")

    # Line 2: dB legend + FFT/SR + gain + status counters + pause indicator
    buf.append("\033[2;1H\033[2K")
    legend = format_db_legend(db_min, db_max)
    info = f"  FFT={fft_size}  SR={sample_rate / 1000:.0f}kHz"
    if rx_gain is not None:
        info += f"  G={rx_gain:.0f}dB"
    if last_status is not None:
        frames = last_status.get("frames", {})
        total = frames.get("total", 0)
        ok = frames.get("crc_ok", 0)
        fail = frames.get("crc_fail", 0)
        info += f"  \033[2m{ok}\033[0m/\033[2m{total}\033[0m"
        if fail:
            info += f" \033[31m{fail}!\033[0m"
        ovf = last_status.get("rx_overflows", 0)
        if ovf:
            info += f" \033[33mOVF={ovf}\033[0m"
    if paused:
        info += (
            f"  \033[1;33mPAUSED\033[0m \033[2m-{scroll_offset}/{history_len}\033[0m"
        )
    buf.append(f"{legend}{info}")

    # Line 3: last decode metadata + payload preview (or blank)
    buf.append("\033[3;1H\033[2K")
    if last_decode is not None:
        decode_str = _format_decode_line(last_decode, total_width)
        buf.append(decode_str)

    return "".join(buf)


def _format_decode_line(decode: dict[str, Any], width: int) -> str:
    """Format last-decode metadata + ASCII payload preview for header line 3."""
    crc_ok = decode.get("crc_valid", False)
    if crc_ok:
        crc_tag = "\033[1;32mCRC_OK\033[0m"
    else:
        crc_tag = "\033[1;31mCRC_FAIL\033[0m"

    sf = decode.get("sf", "?")
    cr = decode.get("cr", 0)
    cr_str = f"CR4/{4 + cr}" if isinstance(cr, int) else f"CR?/{cr}"
    snr = decode.get("snr_db")
    snr_str = f"SNR={snr:+.1f}dB" if snr is not None else ""
    payload_len = decode.get("payload_len", 0)
    rx_ch = decode.get("rx_channel")
    ch_str = f"ch{rx_ch}" if rx_ch is not None else ""
    sw = decode.get("sync_word")
    sw_str = sync_word_name(sw) if sw is not None else ""

    # Local timezone timestamp from wall-clock time captured at decode
    ts_wall = decode.get("_wall_time")
    if ts_wall is not None:
        ts_str = time.strftime("%H:%M:%S", time.localtime(ts_wall))
    else:
        ts_str = ""

    parts = [f"{crc_tag} {payload_len}B SF{sf} {cr_str}"]
    if sw_str:
        parts.append(sw_str)
    if snr_str:
        parts.append(snr_str)
    if ch_str:
        parts.append(ch_str)
    if ts_str:
        parts.append(ts_str)

    meta = " ".join(parts)

    # Append ASCII payload preview to fill remaining width
    payload = decode.get("_payload", b"")
    if payload:
        meta_vis = _ansi_visible_len(meta)
        remaining = width - meta_vis - 2  # 2 for " |" separator
        if remaining > 4:
            ascii_preview = format_ascii(payload, max_bytes=remaining)
            meta += f" \033[2m{ascii_preview}\033[0m"

    return meta


def decode_bins(bins_raw: bytes) -> list[float]:
    """Decode float32 LE binary blob to list of floats via zero-copy cast."""
    # memoryview.cast avoids an intermediate copy vs struct.unpack
    n_bytes = len(bins_raw) - (len(bins_raw) % 4)
    return list(memoryview(bins_raw[:n_bytes]).cast("f"))


def _estimate_noise_floor(bins: list[float]) -> float:
    """Estimate noise floor as the 25th percentile of spectrum bins."""
    if not bins:
        return -100.0
    # Partial sort: only need the lower quarter
    n = len(bins)
    q1_idx = n // 4
    # Use sorted slice of a sample for speed on large FFTs
    if n > 256:
        step = n // 128
        sampled = sorted(bins[::step])
        return sampled[len(sampled) // 4]
    return sorted(bins)[q1_idx]


# ---- Mouse input parsing ----

# SGR mouse mode reports: CSI < button ; col ; row M  (press)
#                         CSI < button ; col ; row m  (release)
# Scroll up = button 64, scroll down = button 65.
_SGR_MOUSE_RE = re.compile(rb"\x1b\[<(\d+);\d+;\d+[Mm]")


def parse_mouse_events(buf: bytes) -> tuple[int, bool, bytes]:
    """Parse SGR mouse events from raw stdin bytes.

    Returns (scroll_delta, click_to_live, remaining_bytes) where:
      - scroll_delta is positive for scroll-up, negative for scroll-down
      - click_to_live is True if any mouse button was pressed (not scroll)
    """
    delta = 0
    click = False
    remaining = buf
    while True:
        m = _SGR_MOUSE_RE.search(remaining)
        if m is None:
            break
        btn = int(m.group(1))
        terminator = remaining[m.end() - 1 : m.end()]
        if btn == 64:  # scroll up
            delta += SCROLL_LINES
        elif btn == 65:  # scroll down
            delta -= SCROLL_LINES
        elif btn <= 2 and terminator == b"M":  # button 0/1/2 press
            click = True
        remaining = remaining[m.end() :]
    return delta, click, remaining


def drain_stdin() -> bytes:
    """Read all available bytes from stdin without blocking."""
    chunks: list[bytes] = []
    fd = sys.stdin.fileno()
    while True:
        rlist, _, _ = select.select([fd], [], [], 0)
        if not rlist:
            break
        chunk = os.read(fd, 4096)
        if not chunk:
            break
        chunks.append(chunk)
    return b"".join(chunks)


def redraw_from_history(
    history: deque[str],
    scroll_offset: int,
    th: int,
    tw: int,
) -> str:
    """Redraw the waterfall area from history at the given scroll offset.

    scroll_offset is the number of rows scrolled back from the newest row.
    Returns the full escape sequence string to repaint the waterfall area.
    """
    waterfall_rows = th - HEADER_LINES
    if waterfall_rows <= 0:
        return ""

    n = len(history)
    # The newest row is history[-1].  At scroll_offset=0 we show the most
    # recent `waterfall_rows` rows.  At scroll_offset=k we show rows ending
    # k positions earlier.
    end = n - scroll_offset  # exclusive
    start = max(0, end - waterfall_rows)
    visible = list(history)[start:end]

    buf: list[str] = []
    for i, row in enumerate(visible):
        line_num = HEADER_LINES + 1 + i
        buf.append(f"\033[{line_num};1H{row}")
    # Clear any remaining lines below the visible history
    for i in range(len(visible), waterfall_rows):
        line_num = HEADER_LINES + 1 + i
        buf.append(f"\033[{line_num};1H\033[2K")
    return "".join(buf)


def _compute_gain_scale(rx_gain: float, db_range: float) -> tuple[float, float]:
    """Compute dB scale from RX gain.

    Higher gain raises the noise floor in dBFS. At 0 dB gain the thermal
    noise floor sits around -100 dBFS; each dB of gain lifts it ~1 dB.
    """
    db_min = -100.0 + rx_gain
    db_max = db_min + db_range
    return db_min, db_max


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Console waterfall display for lora_trx"
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=None,
        help="lora_trx UDP address (default from config.toml or 127.0.0.1:5556)",
    )
    parser.add_argument(
        "--bw-factor",
        type=float,
        default=1.0,
        help="Show this fraction of the sample rate bandwidth (default: 1.0)",
    )
    parser.add_argument(
        "--db-min",
        type=float,
        default=None,
        help="Minimum dB for color scale (auto-set from gain if omitted)",
    )
    parser.add_argument(
        "--db-max",
        type=float,
        default=None,
        help="Maximum dB for color scale (auto-set from gain if omitted)",
    )
    parser.add_argument(
        "--db-range",
        type=float,
        default=60.0,
        help="dB range for auto-scale (default: 60)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
        help="Target rendering framerate (default: 10)",
    )
    parser.add_argument(
        "--smoothing",
        type=float,
        default=0.3,
        help="EMA smoothing alpha, 0.0-1.0 (default: 0.3, lower=smoother)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output (also: NO_COLOR env var)",
    )
    args = parser.parse_args()

    # Resolve address
    cfg = load_config()
    setup_logging("gr4.waterfall", cfg, no_color=args.no_color)

    # Connect directly to lora_trx (not lora_agg) — needs raw spectrum frames
    if args.connect:
        host, port = parse_host_port(args.connect)
    else:
        host, port = config_udp_host(cfg), config_udp_port(cfg)

    # Subscribe without sync_word filter — need spectrum + all frame types
    sock, sub_msg, addr = create_udp_subscriber(host, port, timeout=0.0, blocking=False)
    log.info("connecting to %s:%d", host, port)

    # Terminal raw mode for mouse input
    stdin_fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(stdin_fd)
    tty.setraw(stdin_fd)

    # State
    center_freq_mhz: float = 0.0
    sample_rate: float = 0.0
    fft_size: int = 0
    header_printed: bool = False
    last_keepalive: float = time.monotonic()
    frame_count: int = 0
    skip_count: int = 0

    # dB scale -- auto-adjusted from rx_gain, overridable by CLI
    db_min: float = args.db_min if args.db_min is not None else -80.0
    db_max: float = args.db_max if args.db_max is not None else -20.0
    db_min_manual: bool = args.db_min is not None
    db_max_manual: bool = args.db_max is not None
    rx_gain: float | None = None  # from config/status messages
    noise_floor_ema: float | None = None  # tracked from spectrum data

    # FPS throttling
    min_frame_interval: float = 1.0 / max(args.fps, 1.0)
    last_render_time: float = 0.0

    # EMA smoothing
    ema_alpha: float = max(0.01, min(1.0, args.smoothing))
    rx_ema: list[float] | None = None

    # Frame activity: track most recent decode for header + row tinting
    frame_marker_duration: float = 0.8  # seconds to tint rows after decode
    last_decode: dict | None = None  # most recent decode metadata
    last_tx_time: float = 0.0
    last_status: dict | None = None  # most recent status heartbeat

    # Scrolling state (terminal scroll region)
    scroll_region_set: bool = False
    last_term_width: int = 0
    last_term_height: int = 0

    # Mouse scroll history
    history: deque[str] = deque(maxlen=HISTORY_SIZE)
    scroll_offset: int = 0  # 0 = live, >0 = paused (rows back from newest)
    mouse_buf: bytes = b""
    was_paused: bool = False

    # Periodic header repaint (for status counters etc.)
    header_repaint_interval: float = 2.0  # seconds
    last_header_repaint: float = 0.0

    # Clear screen, hide cursor, enable SGR mouse mode
    sys.stdout.write(
        "\033[2J\033[H\033[?25l"  # clear, home, hide cursor
        "\033[?1003h"  # any-event mouse tracking (includes scroll wheel)
        "\033[?1006h"  # SGR mouse mode (extended coordinates)
    )
    sys.stdout.flush()

    try:
        while True:
            # Periodic keepalive
            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(sub_msg, addr)
                last_keepalive = now

            # Poll both stdin (mouse) and UDP socket
            rlist, _, _ = select.select([stdin_fd, sock], [], [], 0.05)

            # ---- Handle mouse / keyboard input ----
            if stdin_fd in rlist:
                raw = drain_stdin()
                if raw:
                    # Check for Ctrl-C (0x03) in raw mode
                    if b"\x03" in raw:
                        raise KeyboardInterrupt
                    # Check for 'q' or 'Q' to quit
                    if b"q" in raw or b"Q" in raw:
                        raise KeyboardInterrupt

                    mouse_buf += raw
                    delta, click_live, mouse_buf = parse_mouse_events(mouse_buf)
                    # Prevent mouse_buf from growing unboundedly with
                    # non-mouse input
                    if len(mouse_buf) > 256:
                        mouse_buf = mouse_buf[-64:]

                    # Mouse click (any button press) jumps to live view
                    if click_live and scroll_offset > 0:
                        delta = -scroll_offset  # force to live

                    if delta != 0:
                        tw, th = get_terminal_size()
                        waterfall_rows = th - HEADER_LINES
                        max_offset = max(0, len(history) - waterfall_rows)
                        scroll_offset = max(0, min(scroll_offset + delta, max_offset))

                        if scroll_offset > 0:
                            # Paused -- redraw from history
                            if not was_paused:
                                # First pause: reset scroll region so we can
                                # paint arbitrary rows without terminal scroll
                                sys.stdout.write("\033[r")
                                was_paused = True
                            frame_buf_parts: list[str] = []
                            # Repaint header with PAUSED indicator
                            frame_buf_parts.append(
                                format_header(
                                    center_freq_mhz,
                                    sample_rate,
                                    args.bw_factor,
                                    db_min,
                                    db_max,
                                    fft_size,
                                    tw,
                                    rx_gain,
                                    last_decode,
                                    last_status,
                                    paused=True,
                                    scroll_offset=scroll_offset,
                                    history_len=len(history),
                                )
                            )
                            frame_buf_parts.append(
                                redraw_from_history(history, scroll_offset, th, tw)
                            )
                            header_printed = True
                            sys.stdout.write("".join(frame_buf_parts))
                            sys.stdout.flush()
                        else:
                            # Resumed live mode -- redraw latest rows
                            # then re-establish scroll region on next
                            # spectrum frame
                            redraw = redraw_from_history(history, 0, th, tw)
                            sys.stdout.write(redraw)
                            sys.stdout.flush()
                            was_paused = False
                            header_printed = False
                            scroll_region_set = False

            # ---- Handle UDP data ----
            if sock not in rlist:
                # If paused, repaint header periodically for PAUSED indicator
                if scroll_offset > 0 and not header_printed:
                    tw, th = get_terminal_size()
                    hdr = format_header(
                        center_freq_mhz,
                        sample_rate,
                        args.bw_factor,
                        db_min,
                        db_max,
                        fft_size,
                        tw,
                        rx_gain,
                        last_decode,
                        last_status,
                        paused=True,
                        scroll_offset=scroll_offset,
                        history_len=len(history),
                    )
                    sys.stdout.write(hdr)
                    sys.stdout.flush()
                    header_printed = True
                continue

            try:
                data, _addr = sock.recvfrom(65536)
            except (BlockingIOError, OSError):
                continue

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type")

            # Handle config message (get PHY params + initial gain)
            if msg_type == "config":
                phy = msg.get("phy", {})
                server = msg.get("server", {})
                f = phy.get("freq", 0)
                if f > 0:
                    center_freq_mhz = f / 1e6
                sr = server.get("sample_rate", 0)
                if sr > 0:
                    sample_rate = sr
                g = phy.get("rx_gain")
                if g is not None and float(g) != rx_gain:
                    rx_gain = float(g)
                    # Only reset dB scale on actual gain changes (config is
                    # re-sent on every keepalive -- resetting unconditionally
                    # would clobber the dynamic noise floor tracking).
                    new_min, new_max = _compute_gain_scale(rx_gain, args.db_range)
                    if not db_min_manual:
                        db_min = new_min
                    if not db_max_manual:
                        db_max = new_max
                    header_printed = False
                continue

            # Handle status heartbeat (periodic gain updates + counters)
            if msg_type == "status":
                last_status = msg
                phy = msg.get("phy", {})
                g = phy.get("rx_gain")
                if g is not None and float(g) != rx_gain:
                    rx_gain = float(g)
                    new_min, new_max = _compute_gain_scale(rx_gain, args.db_range)
                    if not db_min_manual:
                        db_min = new_min
                    if not db_max_manual:
                        db_max = new_max
                # Don't set header_printed=False here -- the header will be
                # repainted on the periodic timer below.  Unconditional
                # repaints on every status heartbeat caused visible flicker.
                continue

            # Track frame decodes for tinting + header line 3
            if msg_type == "lora_frame":
                phy = msg.get("phy", {})
                last_decode = {
                    "crc_valid": msg.get("crc_valid", False),
                    "sf": phy.get("sf", "?"),
                    "cr": phy.get("cr", 0),
                    "snr_db": phy.get("snr_db"),
                    "sync_word": phy.get("sync_word"),
                    "payload_len": msg.get("payload_len", 0),
                    "rx_channel": msg.get("rx_channel"),
                    "_payload": msg.get("payload", b""),
                    "_time": time.monotonic(),
                    "_wall_time": time.time(),
                }
                header_printed = False  # update line 3
                continue

            if msg_type == "lora_tx_ack":
                last_tx_time = time.monotonic()
                continue

            # Only process RX spectrum from here
            if msg_type != "spectrum":
                continue

            # Parse spectrum message
            bins_raw = msg.get("bins", b"")
            new_fft_size = msg.get("fft_size", 0)
            new_center_freq = msg.get("center_freq", 0.0)
            new_sample_rate = msg.get("sample_rate", 0.0)

            if not bins_raw or new_fft_size == 0:
                continue

            if new_center_freq > 0:
                center_freq_mhz = new_center_freq / 1e6
            if new_sample_rate > 0:
                sample_rate = new_sample_rate
            fft_size = new_fft_size

            bins = decode_bins(bins_raw)
            n_bins = len(bins)

            # EMA update
            if rx_ema is None or len(rx_ema) != n_bins:
                rx_ema = bins[:]
            else:
                for i in range(n_bins):
                    rx_ema[i] += ema_alpha * (bins[i] - rx_ema[i])

            # Track noise floor from spectrum data (for dynamic adjustment)
            nf = _estimate_noise_floor(rx_ema)
            if noise_floor_ema is None:
                noise_floor_ema = nf
            else:
                noise_floor_ema += _NOISE_FLOOR_ALPHA * (nf - noise_floor_ema)

            # Dynamic dB scale: blend gain-based baseline with actual noise floor
            if not db_min_manual and noise_floor_ema is not None:
                # Set floor 10 dB below tracked noise floor
                target_min = noise_floor_ema - 10.0
                # Smooth transition
                db_min += 0.02 * (target_min - db_min)
            if not db_max_manual:
                db_max = db_min + args.db_range

            # Throttle rendering to target FPS
            now = time.monotonic()
            if now - last_render_time < min_frame_interval:
                skip_count += 1
                continue
            last_render_time = now

            tw, th = get_terminal_size()

            # Set up scroll region on first use or terminal resize
            need_scroll_region: bool = False
            if center_freq_mhz > 0 and sample_rate > 0:
                if (
                    not scroll_region_set
                    or tw != last_term_width
                    or th != last_term_height
                ):
                    need_scroll_region = True
                    scroll_region_set = True
                    last_term_width = tw
                    last_term_height = th
                    header_printed = False  # force header repaint after region change

            if not scroll_region_set:
                continue

            # Compute visible bins
            rx_n = len(rx_ema)
            rx_visible = int(rx_n * min(args.bw_factor, 1.0))
            if rx_visible < 1:
                rx_visible = rx_n
            rx_center = rx_n // 2

            # Select palette based on recent frame activity
            active_palette: int = PALETTE_NORMAL
            now_mono = time.monotonic()

            if last_decode is not None:
                decode_age = now_mono - last_decode["_time"]
                if decode_age < frame_marker_duration:
                    if last_decode["crc_valid"]:
                        active_palette = PALETTE_DECODE_OK
                    else:
                        active_palette = PALETTE_DECODE_FAIL

            if active_palette == PALETTE_NORMAL:
                if now_mono - last_tx_time < frame_marker_duration:
                    active_palette = PALETTE_TX

            row = render_row(
                rx_ema,
                tw,
                db_min,
                db_max,
                rx_center,
                rx_visible,
                palette=active_palette,
            )

            # Store row in history for scroll-back
            history.append(row)

            # If paused (scrolled back), don't update the display --
            # just keep accumulating history.  On resize, repaint from
            # history so the layout stays clean (rows may be old width).
            if scroll_offset > 0:
                if need_scroll_region:
                    waterfall_rows = th - HEADER_LINES
                    max_offset = max(0, len(history) - waterfall_rows)
                    scroll_offset = min(scroll_offset, max_offset)
                    parts: list[str] = ["\033[r"]  # reset scroll region
                    parts.append(
                        format_header(
                            center_freq_mhz,
                            sample_rate,
                            args.bw_factor,
                            db_min,
                            db_max,
                            fft_size,
                            tw,
                            rx_gain,
                            last_decode,
                            last_status,
                            paused=True,
                            scroll_offset=scroll_offset,
                            history_len=len(history),
                        )
                    )
                    parts.append(redraw_from_history(history, scroll_offset, th, tw))
                    sys.stdout.write("".join(parts))
                    sys.stdout.flush()
                    header_printed = True
                frame_count += 1
                continue

            # Periodic header repaint for status counters / dB scale changes
            if (
                header_printed
                and now_mono - last_header_repaint >= header_repaint_interval
            ):
                header_printed = False

            # Build single atomic output: scroll region + header + row
            # Batching into one write avoids intermediate terminal states
            # that cause flicker (wezterm redraws on DECSTBM / cursor moves).
            frame_buf: list[str] = []
            if need_scroll_region:
                frame_buf.append(f"\033[{HEADER_LINES + 1};{th}r")
            if not header_printed:
                frame_buf.append(
                    format_header(
                        center_freq_mhz,
                        sample_rate,
                        args.bw_factor,
                        db_min,
                        db_max,
                        fft_size,
                        tw,
                        rx_gain,
                        last_decode,
                        last_status,
                    )
                )
                header_printed = True
                last_header_repaint = now_mono
            frame_buf.append(f"\033[{th};1H{row}\n")
            sys.stdout.write("".join(frame_buf))
            sys.stdout.flush()
            frame_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal: disable mouse, reset scroll region, show cursor,
        # restore terminal attributes
        sys.stdout.write(
            "\033[?1006l"  # disable SGR mouse mode
            "\033[?1003l"  # disable any-event mouse tracking
            "\033[r"  # reset scroll region
            "\033[?25h"  # show cursor
        )
        sys.stdout.flush()
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_termios)
        sock.close()
        log.info("stopped (%d rendered, %d skipped)", frame_count, skip_count)


if __name__ == "__main__":
    main()
