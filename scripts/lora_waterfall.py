#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# Requires Python >= 3.14
"""
lora_waterfall.py -- Console waterfall display for lora_trx.

Connects to lora_trx via UDP, receives spectrum CBOR messages, and renders
a scrolling waterfall in the terminal using xterm-256color escape codes.
Rows are tinted green (CRC_OK) or red (CRC_FAIL) during frame decodes.

Keyboard Controls:
    Space  - Pause/resume live view
    +/-    - Adjust EMA smoothing alpha
    0-9    - Set EMA alpha to 0.0-0.9
    q      - Quit

Mouse:
    Scroll up    - Pause and browse history
    Scroll down  - Resume live mode
    Click        - Resume live mode

Usage:
    lora_waterfall.py                          # default: 127.0.0.1:5555
    lora_waterfall.py --connect 192.168.1.10:5555
    lora_waterfall.py --bw-hz 150000           # zoom to 150 kHz
    lora_waterfall.py --delay 2.0              # longer delay for SF12
"""

from __future__ import annotations

import argparse
import logging
import math
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
    add_logging_args,
    create_udp_subscriber,
    format_ascii,
    format_frame,
    parse_host_port,
    setup_logging,
)

log = logging.getLogger("gr4.waterfall")


# ---- xterm-256color palette + block-density characters for waterfall ----
#
# Each waterfall cell uses two visual channels simultaneously:
#   1. Background color (64-step ramp: navy → teal → green → yellow → orange)
#   2. Unicode block character whose density encodes the same dB level
#
# This doubles the dynamic range visible to the eye and works even in
# terminals with poor 256-color rendering.


def _cube(r: int, g: int, b: int) -> int:
    """xterm-256 6x6x6 cube index from r,g,b (each 0-5)."""
    return 16 + 36 * r + 6 * g + b


# Base ramp stops: navy → teal → green → yellow → orange → red → hot white (64 entries)
_BASE_STOPS: list[tuple[int, int, int]] = [
    # Dark navy (0-7)
    (0, 0, 1),
    (0, 0, 1),
    (0, 0, 2),
    (0, 0, 2),
    (0, 1, 2),
    (0, 1, 2),
    (0, 1, 3),
    (0, 1, 3),
    # Teal (8-15)
    (0, 2, 3),
    (0, 2, 3),
    (0, 2, 2),
    (0, 3, 2),
    (0, 3, 1),
    (0, 3, 0),
    (0, 4, 0),
    (0, 4, 0),
    # Green (16-27)
    (0, 5, 0),
    (0, 5, 0),
    (1, 5, 0),
    (1, 5, 0),
    (2, 5, 0),
    (2, 5, 0),
    (3, 5, 0),
    (3, 5, 0),
    (4, 5, 0),
    (4, 5, 0),
    (5, 5, 0),
    (5, 5, 0),
    # Yellow → orange (28-39)
    (5, 5, 0),
    (5, 4, 0),
    (5, 4, 0),
    (5, 3, 0),
    (5, 3, 0),
    (5, 2, 0),
    (5, 2, 0),
    (5, 2, 0),
    (5, 1, 0),
    (5, 1, 0),
    (5, 1, 0),
    (5, 1, 0),
    # Orange → red (40-51)
    (5, 0, 0),
    (5, 0, 0),
    (5, 0, 0),
    (5, 0, 0),
    (5, 0, 0),
    (5, 0, 0),
    (5, 0, 1),
    (5, 0, 1),
    (5, 0, 2),
    (5, 0, 2),
    (5, 0, 3),
    (5, 0, 3),
    # Red → hot white (52-63)
    (5, 1, 3),
    (5, 1, 4),
    (5, 2, 4),
    (5, 2, 5),
    (5, 3, 5),
    (5, 3, 5),
    (5, 4, 5),
    (5, 4, 5),
    (5, 5, 5),
    (5, 5, 5),
    (5, 5, 5),
    (5, 5, 5),
]

N_COLORS: int = len(_BASE_STOPS)

# xterm-256 color index for each ramp position
RAMP_BASE: list[int] = [_cube(r, g, b) for r, g, b in _BASE_STOPS]

# Block-density characters mapped to color index ranges.
# Five levels: space (noise) → ░ → ▒ → ▓ → █ (peak).
# Breakpoints at 20 % intervals across N_COLORS=64.
_LEVEL_CHARS: list[str] = []
for _ci in range(N_COLORS):
    _frac = _ci / (N_COLORS - 1)
    if _frac < 0.20:
        _LEVEL_CHARS.append(" ")  # below noise: invisible
    elif _frac < 0.40:
        _LEVEL_CHARS.append("░")  # U+2591 light shade
    elif _frac < 0.60:
        _LEVEL_CHARS.append("▒")  # U+2592 medium shade
    elif _frac < 0.80:
        _LEVEL_CHARS.append("▓")  # U+2593 dark shade
    else:
        _LEVEL_CHARS.append("█")  # U+2588 full block


def _estimate_noise_floor(bins: list[float]) -> float:
    """Estimate noise floor as the 25th percentile of spectrum bins."""
    if not bins:
        return -100.0
    n = len(bins)
    if n > 256:
        step = n // 128
        sampled = sorted(bins[::step])
        return sampled[len(sampled) // 4]
    return sorted(bins)[n // 4]


# Header occupies 3 fixed lines at the top
HEADER_LINES: int = 3

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
) -> str:
    """Render one waterfall row using background color + block-density chars.

    Each cell combines:
      - Background color from RAMP_BASE (64-step navy→orange ramp)
      - A Unicode block character (space/░/▒/▓/█) encoding the same level
    """
    half = visible_bins // 2
    start = max(0, center_idx - half)
    end = min(len(bins), start + visible_bins)
    start = max(0, end - visible_bins)
    visible = bins[start:end]

    n = len(visible)
    if n == 0:
        return " " * width

    parts: list[str] = []
    prev_ci = -1
    for col in range(width):
        src_idx = min(col * n // width, n - 1)
        ci = db_to_color_index(visible[src_idx], db_min, db_max)
        if ci != prev_ci:
            c = RAMP_BASE[ci]
            parts.append(f"\033[38;5;{c}m\u2588")
            prev_ci = ci
        else:
            parts.append("\u2588")
    parts.append("\033[0m")
    return "".join(parts)


def get_terminal_size() -> tuple[int, int]:
    """Get terminal (columns, rows), default (80, 24)."""
    try:
        sz = os.get_terminal_size()
        return sz.columns, sz.lines
    except AttributeError, ValueError, OSError:
        return 80, 24


def format_freq_axis(
    center_freq_mhz: float,
    visible_bw_hz: float,
    width: int,
) -> str:
    """Format a frequency axis string spanning the visible bandwidth."""
    f_lo = center_freq_mhz - visible_bw_hz / 2e6
    f_hi = center_freq_mhz + visible_bw_hz / 2e6

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
    # DC spur marker: ▾ just left of center frequency label
    dc_pos = c_start - 1
    if 0 <= dc_pos < width and axis[dc_pos] == " ":
        axis[dc_pos] = "\u25be"  # ▾
    return "".join(axis)


def format_db_legend(db_min: float, db_max: float) -> str:
    """Format a short dB scale legend with color samples."""
    n_samples = min(16, N_COLORS)
    parts = [f" {db_min:.0f}dB "]
    for i in range(n_samples):
        ci = i * (N_COLORS - 1) // (n_samples - 1)
        color = RAMP_BASE[ci]
        parts.append(f"\033[38;5;{color}m\u2588\033[0m")
    parts.append(f" {db_max:.0f}dB")
    return "".join(parts)


def _ansi_visible_len(s: str) -> int:
    """Return the visible length of a string containing ANSI escape codes."""
    return len(re.sub(r"\033\[[^m]*m", "", s))


def format_header(
    center_freq_mhz: float,
    sample_rate: float,
    visible_bw_hz: float,
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
    axis = format_freq_axis(center_freq_mhz, visible_bw_hz, total_width)
    buf.append(f"\033[1m{axis}\033[0m")

    # Line 2: dB legend + FFT/SR + gain + status counters + pause indicator
    buf.append("\033[2;1H\033[2K")
    legend = format_db_legend(db_min, db_max)
    info = f"  FFT={fft_size}  SR={sample_rate / 1000:.0f}kHz  VBW={visible_bw_hz / 1000:.0f}kHz"
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
        info += f"  \033[2m-{scroll_offset}/{history_len}\033[0m"
    buf.append(f"{legend}{info}")

    # Line 3: last decode metadata + payload preview (or blank)
    buf.append("\033[3;1H\033[2K")
    if last_decode is not None:
        decode_str = _format_decode_line(last_decode, total_width)
        buf.append(decode_str)

    return "".join(buf)


def _format_decode_line(decode: dict[str, Any], width: int) -> str:
    """Format last-decode metadata + ASCII payload preview for header line 3.

    Uses format_frame() from lora_common for consistent display with lora_mon.
    Shows only the first line of format_frame output, with ASCII payload
    preview appended to fill the remaining terminal width.
    """
    # Local timezone timestamp from wall-clock time captured at decode
    ts_wall = decode.get("_wall_time")
    ts_str = (
        time.strftime("%H:%M:%S", time.localtime(ts_wall))
        if ts_wall is not None
        else ""
    )

    # Get first line of format_frame output (the header line)
    full = format_frame(decode)
    first_line = full.split("\n")[0]

    meta = first_line
    if ts_str:
        meta += f" {ts_str}"

    # Append ASCII payload preview to fill remaining width
    payload = decode.get("payload", b"")
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


def _frame_duration(decode: dict[str, Any]) -> float:
    """Estimate LoRa frame air-time in seconds from a lora_frame CBOR message.

    Uses SF, BW, CR, and payload_len.  The estimate is intentionally generous
    (adds preamble + header symbols) so the tinting window fully covers the
    frame even under timing jitter.
    """
    phy = decode.get("phy", {})
    sf = phy.get("sf", 8)
    bw = phy.get("bw", 125000)
    cr = phy.get("cr", 4)  # coding rate 1-4 → denominator is 4+cr
    payload_len = decode.get("payload_len", 0)
    if bw <= 0:
        bw = 125000
    symbol_duration = (2**sf) / bw  # seconds per symbol
    # Data symbols: ceil((8*payload + 28 - 4*sf + 20) / (4*(sf-2))) * (4+cr)
    # Simplified conservative estimate: payload symbols + 20 overhead symbols
    data_syms = max(8, int(payload_len * 8 / max(sf - 2, 1)) + 20)
    preamble_syms = 12  # 8 preamble + 4.25 sync, rounded up
    return symbol_duration * (preamble_syms + data_syms)


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
        "--bw-hz",
        type=float,
        default=None,
        help=(
            "Visible bandwidth in Hz (default: full sample rate). "
            "Use e.g. 150000 to zoom in on a 62.5 kHz LoRa signal."
        ),
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
        default=30.0,
        help="dB range for auto-scale (default: 30)",
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
        "--delay",
        type=float,
        default=1.0,
        help=(
            "Display delay in seconds (default: 1.0). "
            "Rows are held in a pending queue and flushed after this delay, "
            "allowing decode events to arrive before their rows are rendered "
            "so frame tinting aligns with the actual frame in the waterfall. "
            "Increase for SF12 frames (~4s air-time)."
        ),
    )
    add_logging_args(parser)
    args = parser.parse_args()

    setup_logging("gr4.waterfall", log_level=args.log_level, no_color=args.no_color)

    # Connect directly to lora_trx (needs raw spectrum frames)
    if args.connect:
        host, port = parse_host_port(args.connect)
    else:
        host, port = "127.0.0.1", 5556  # lora_trx direct (not agg)

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

    # dB scale — calibrated once from the first spectrum's noise floor, then
    # frozen.  --db-min / --db-max override for manual control.
    db_min: float = args.db_min if args.db_min is not None else -80.0
    db_max: float = args.db_max if args.db_max is not None else -20.0
    db_min_manual: bool = args.db_min is not None
    db_max_manual: bool = args.db_max is not None
    _scale_calibrated: bool = db_min_manual  # skip auto-cal if user set --db-min
    rx_gain: float | None = None  # from config/status messages
    # Visible bandwidth in Hz — set from --bw-hz, or defaults to full sample rate.
    # Resolved once sample_rate is known; capped at sample_rate.
    visible_bw_hz: float = 0.0  # 0 = unset; resolved when sample_rate arrives

    # FPS throttling
    min_frame_interval: float = 1.0 / max(args.fps, 1.0)
    last_render_time: float = 0.0

    # EMA smoothing
    ema_alpha: float = max(0.01, min(1.0, args.smoothing))
    rx_ema: list[float] | None = None

    # Upper bound extension for tinting window: time for EMA to decay to 10%
    # of a step response. The frame signal stays visible in EMA for this long
    # after the frame ends, so the tinting window must extend by this amount.
    # Formula: ceil(log(0.1) / log(1 - alpha)) / fps
    _ema_decay_tail: float = math.ceil(
        math.log(0.1) / math.log(max(1e-9, 1.0 - ema_alpha))
    ) / max(args.fps, 1.0)

    # Frame activity: track the most recent decoded frame for header and tinting.
    # active_frame = (dec_time, frame_duration, tint_mode) of the last decode.
    # Rows are held in pending_rows for display_delay seconds before rendering;
    # at flush time each row's tint is chosen by checking whether active_frame's
    # window [dec_time - duration, dec_time + _ema_decay_tail] covers capture_time.
    # Using only the most recent decode keeps tints clean — LoRa is half-duplex,
    # so only one frame is on-air at a time.
    display_delay: float = max(0.0, args.delay)
    last_decode: dict[str, Any] | None = None  # most recent decode (for header line 3)
    # active_frame: (dec_time, duration_s, crc_ok) | None
    active_frame: tuple[float, float, bool] | None = None
    last_tx_time: float = 0.0
    last_status: dict[str, Any] | None = None  # most recent status heartbeat
    # pending_rows: (capture_time, ema_snapshot, db_min, db_max) awaiting flush
    pending_rows: deque[tuple[float, list[float], float, float]] = deque()

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
                    # Space = pause/resume
                    if b" " in raw:
                        if scroll_offset > 0:
                            # Resume
                            scroll_offset = 0
                            was_paused = False
                            header_printed = False
                            scroll_region_set = False
                        else:
                            # Pause
                            tw, th = get_terminal_size()
                            waterfall_rows = th - HEADER_LINES
                            max_offset = max(0, len(history) - waterfall_rows)
                            scroll_offset = max_offset
                            if not was_paused:
                                sys.stdout.write("\033[r")
                                was_paused = True
                            # Trigger redraw
                            delta = 0
                    # +/- adjust EMA
                    if b"+" in raw or b"=" in raw:
                        ema_alpha = min(1.0, ema_alpha + 0.1)
                        header_printed = False
                    if b"-" in raw:
                        ema_alpha = max(0.01, ema_alpha - 0.1)
                        header_printed = False
                    # 0-9 set EMA directly
                    for digit in b"0123456789":
                        if digit in raw:
                            ema_alpha = (digit - ord("0")) / 10.0
                            if ema_alpha < 0.01:
                                ema_alpha = 0.01
                            header_printed = False
                            break

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
                                    visible_bw_hz,
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
                        visible_bw_hz,
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
            except BlockingIOError, OSError:
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
                    visible_bw_hz = (
                        min(args.bw_hz, sample_rate) if args.bw_hz else sample_rate
                    )
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

            # Track frame decodes for header line 3 and delayed row tinting.
            # active_frame = (dec_time, duration, crc_ok) for the most recent
            # decode only — LoRa is half-duplex so one frame at a time.
            if msg_type == "lora_frame":
                last_decode = dict(msg)
                last_decode["_time"] = time.monotonic()
                last_decode["_wall_time"] = time.time()
                active_frame = (
                    last_decode["_time"],
                    _frame_duration(last_decode),
                    bool(last_decode.get("crc_valid", False)),
                )
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
                visible_bw_hz = (
                    min(args.bw_hz, sample_rate) if args.bw_hz else sample_rate
                )
            fft_size = new_fft_size

            bins = decode_bins(bins_raw)
            n_bins = len(bins)

            # EMA update
            if rx_ema is None or len(rx_ema) != n_bins:
                rx_ema = bins[:]
            else:
                for i in range(n_bins):
                    rx_ema[i] += ema_alpha * (bins[i] - rx_ema[i])

            # Adaptive scale calibration: on the first spectrum frame, set db_min
            # so the observed noise floor maps to ~20% up the ramp (just above
            # the invisible "space" zone).  After that, db_max tracks the actual
            # peak with slow decay so strong local signals use the full gradient
            # (red at the top) instead of clipping to white.
            nf = _estimate_noise_floor(rx_ema)
            peak = max(rx_ema)
            if not _scale_calibrated:
                if not db_min_manual:
                    db_min = nf - 0.20 * args.db_range
                if not db_max_manual:
                    db_max = max(db_min + args.db_range, peak + 3.0)
                _scale_calibrated = True
            else:
                # Slowly track noise floor (prevents drift on short spikes)
                if not db_min_manual:
                    new_floor = nf - 0.20 * args.db_range
                    db_min = db_min + 0.02 * (new_floor - db_min)
                # Expand db_max instantly to accommodate peaks, decay slowly
                if not db_max_manual:
                    target_max = max(db_min + args.db_range, peak + 3.0)
                    if target_max > db_max:
                        db_max = target_max  # instant expand
                    else:
                        db_max = db_max + 0.01 * (target_max - db_max)  # slow decay

            # Throttle spectrum ingestion to target FPS
            now = time.monotonic()
            if now - last_render_time < min_frame_interval:
                skip_count += 1
                # Still flush pending rows even when skipping new spectrum
            else:
                last_render_time = now
                # Enqueue EMA snapshot + scale at capture time for delayed
                # rendering.  Snapshotting db_min/db_max here prevents the
                # noise-floor AGC from retroactively darkening pre-chirp rows
                # when chirp energy lifts the scale during the display delay.
                pending_rows.append((now, list(rx_ema), db_min, db_max))

            # ---- Scroll region + header setup ----
            # Must happen immediately on first spectrum / terminal resize,
            # BEFORE any row is flushed.  If we defer to inside the flush loop
            # the first \033[S fires without DECSTBM set and scrolls the header.
            now_flush = time.monotonic()
            tw, th = get_terminal_size()

            if center_freq_mhz > 0 and sample_rate > 0:
                if (
                    not scroll_region_set
                    or tw != last_term_width
                    or th != last_term_height
                ):
                    scroll_region_set = True
                    last_term_width = tw
                    last_term_height = th
                    if scroll_offset == 0:
                        # Establish DECSTBM and repaint header immediately.
                        # Clears waterfall area to avoid stale rows at wrong size.
                        setup: list[str] = [
                            f"\033[{HEADER_LINES + 1};{th}r",  # scroll region
                            f"\033[{HEADER_LINES + 1};{th}r",  # set twice: wezterm quirk
                        ]
                        setup.append(
                            format_header(
                                center_freq_mhz,
                                sample_rate,
                                visible_bw_hz,
                                db_min,
                                db_max,
                                fft_size,
                                tw,
                                rx_gain,
                                last_decode,
                                last_status,
                            )
                        )
                        sys.stdout.write("".join(setup))
                        sys.stdout.flush()
                        header_printed = True
                        last_header_repaint = now_flush
                    else:
                        # Paused: reset scroll region + repaint header + history
                        waterfall_rows = th - HEADER_LINES
                        max_offset = max(0, len(history) - waterfall_rows)
                        scroll_offset = min(scroll_offset, max_offset)
                        parts: list[str] = ["\033[r"]
                        parts.append(
                            format_header(
                                center_freq_mhz,
                                sample_rate,
                                visible_bw_hz,
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
                        parts.append(
                            redraw_from_history(history, scroll_offset, th, tw)
                        )
                        sys.stdout.write("".join(parts))
                        sys.stdout.flush()
                        header_printed = True

            if not scroll_region_set:
                continue

            # Header repaint: fire immediately when header_printed=False
            # (new decode / gain change), or on the periodic timer.
            now_mono = time.monotonic()
            needs_header = not header_printed or (
                now_mono - last_header_repaint >= header_repaint_interval
            )
            if scroll_offset == 0 and needs_header:
                sys.stdout.write(
                    format_header(
                        center_freq_mhz,
                        sample_rate,
                        visible_bw_hz,
                        db_min,
                        db_max,
                        fft_size,
                        tw,
                        rx_gain,
                        last_decode,
                        last_status,
                    )
                )
                sys.stdout.flush()
                header_printed = True
                last_header_repaint = now_mono

            # Compute visible bin parameters (same for all flushed rows this cycle)
            rx_n = len(rx_ema)
            rx_visible = int(
                rx_n * min(visible_bw_hz, sample_rate) / max(sample_rate, 1.0)
            )
            if rx_visible < 1:
                rx_visible = rx_n
            rx_center = rx_n // 2

            # ---- Flush rows whose display_delay has elapsed ----
            while pending_rows and (now_flush - pending_rows[0][0]) >= display_delay:
                capture_time, snap, snap_db_min, snap_db_max = pending_rows.popleft()

                # Render with scale snapshotted at capture time.
                row = render_row(
                    snap,
                    tw,
                    snap_db_min,
                    snap_db_max,
                    rx_center,
                    rx_visible,
                )

                # Store row in history for scroll-back
                history.append(row)

                if scroll_offset > 0:
                    # Paused: accumulate history only, don't touch the display.
                    frame_count += 1
                    continue

                # Live mode: scroll region up 1, write new row at the bottom.
                # \033[S = scroll up within DECSTBM (set above, before flush).
                # No \n — avoids a second implicit scroll if cursor is at th.
                sys.stdout.write(f"\033[{th};1H\033[S\033[{th};1H{row}")
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
