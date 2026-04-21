#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Render the TX-power calibration curve in GR4 ImChart style.

Output is a braille-based line chart with ANSI colours; run and
redirect stdout to data/testing/power_level.txt.  No arguments — the
data is baked in from the 2026-04-19 measurement (git 764b58ea).
Regenerate by editing the DATA list below.
"""
from __future__ import annotations

import sys

# gain, source dBm (measured 2026-04-19, git 764b58ea, 60 dB fixture)
DATA = [
    (30.0, -23.63),
    (40.0, -13.89),
    (50.0,  -3.82),
    (55.0,  +1.31),
    (60.0,  +6.21),
    (65.0, +11.45),
    (70.0, +16.47),
    (75.0, +20.50),
]

# matches gr::graphs::Color
ANSI = {
    "reset":  "\x1B[39m",
    "blue":   "\x1B[34m",
    "red":    "\x1B[31m",
    "yellow": "\x1B[33m",
    "cyan":   "\x1B[36m",
    "lgreen": "\x1B[92m",
    "dgray":  "\x1B[90m",
}

# braille offsets for a 2x4 cell.  bit -> (dx, dy) from cell origin.
#    dot 1 (0,0)=1  dot 4 (1,0)=8
#    dot 2 (0,1)=2  dot 5 (1,1)=16
#    dot 3 (0,2)=4  dot 6 (1,2)=32
#    dot 7 (0,3)=64 dot 8 (1,3)=128
BRAILLE_BIT = [
    [  1,   8],
    [  2,  16],
    [  4,  32],
    [ 64, 128],
]

CELL_W = 2
CELL_H = 4


def main() -> int:
    # chart geometry matches qa_BurstTaper.cpp: 120 cols x 25 rows
    W = 120
    H = 25
    x_min, x_max = 28.0, 77.0
    y_min, y_max = -28.0, 28.0

    # plot area leaves margin for axis labels + legend line
    plot_w = W - 10                        # 10 cols for y-axis label
    plot_h = H - 3                         # 3 rows for x-axis + legend
    br_w = plot_w * CELL_W                 # braille grid width
    br_h = plot_h * CELL_H                 # braille grid height

    def to_col(x: float) -> int:
        return int(round((x - x_min) / (x_max - x_min) * (br_w - 1)))

    def to_row(y: float) -> int:
        # row 0 = top (y_max); increasing row = decreasing y
        return int(round((y_max - y) / (y_max - y_min) * (br_h - 1)))

    # series -> set of (col, row) in braille grid, plus colour
    series = []

    # 1) linear fit line
    fit_pts = set()
    slope, intercept = 1.004, -53.93
    for col in range(br_w):
        gain = x_min + (col / (br_w - 1)) * (x_max - x_min)
        dbm = slope * gain + intercept
        if y_min <= dbm <= y_max:
            fit_pts.add((col, to_row(dbm)))
    series.append(("linear fit 1.004/gain", fit_pts, ANSI["blue"]))

    # 2) horizontal reference at +23 dBm (pad rating)
    pad_pts = set()
    r_pad = to_row(23.0)
    for col in range(br_w):
        pad_pts.add((col, r_pad))
    series.append(("pad rating +23 dBm", pad_pts, ANSI["red"]))

    # 3) horizontal reference at +20 dBm (EU 100 mW)
    eu_pts = set()
    r_eu = to_row(20.0)
    for col in range(br_w):
        eu_pts.add((col, r_eu))
    series.append(("EU 100 mW +20 dBm", eu_pts, ANSI["yellow"]))

    # 4) horizontal at 0 dBm (1 mW reference)
    ref_pts = set()
    r_0 = to_row(0.0)
    for col in range(br_w):
        ref_pts.add((col, r_0))
    series.append(("1 mW 0 dBm", ref_pts, ANSI["dgray"]))

    # 5) measured points (draw as 2x2 block so they're visible over the fit)
    meas_pts = set()
    for gx, gy in DATA:
        c = to_col(gx); r = to_row(gy)
        for dc in (-1, 0, 1, 2):
            for dr in (-1, 0, 1, 2):
                cc, rr = c + dc, r + dr
                if 0 <= cc < br_w and 0 <= rr < br_h:
                    meas_pts.add((cc, rr))
    series.append(("measured", meas_pts, ANSI["lgreen"]))

    # render: for each plot cell, OR together the braille dots from
    # every series that touches it; if multiple series in one cell we
    # just take the highest-priority (last) colour.
    cells = [[None for _ in range(plot_w)] for _ in range(plot_h)]  # (bits, colour)

    for name, pts, colour in series:
        for col, row in pts:
            pcol, bx = divmod(col, CELL_W)
            prow, by = divmod(row, CELL_H)
            if 0 <= pcol < plot_w and 0 <= prow < plot_h:
                bit = BRAILLE_BIT[by][bx]
                cur = cells[prow][pcol]
                if cur is None:
                    cells[prow][pcol] = (bit, colour)
                else:
                    cells[prow][pcol] = (cur[0] | bit, colour)

    # header
    print(f"{ANSI['cyan']}B220 Mini TX power vs tx_gain @ 869.618 MHz{ANSI['reset']}"
          f"  (measured 2026-04-19, 60 dB fixture)")
    print()

    # body + y-axis labels
    y_label_rows = {  # row -> label text
        to_row(25.0) // CELL_H: " +25 dBm",
        to_row(20.0) // CELL_H: " +20 dBm",
        to_row(15.0) // CELL_H: " +15 dBm",
        to_row(10.0) // CELL_H: " +10 dBm",
        to_row( 5.0) // CELL_H: "  +5 dBm",
        to_row( 0.0) // CELL_H: "   0 dBm",
        to_row(-5.0) // CELL_H: "  -5 dBm",
        to_row(-10.0) // CELL_H: " -10 dBm",
        to_row(-15.0) // CELL_H: " -15 dBm",
        to_row(-20.0) // CELL_H: " -20 dBm",
        to_row(-25.0) // CELL_H: " -25 dBm",
    }

    for prow in range(plot_h):
        label = y_label_rows.get(prow, "        ")
        line = label + " │"
        for pcol in range(plot_w):
            cell = cells[prow][pcol]
            if cell is None:
                line += " "
            else:
                bits, colour = cell
                ch = chr(0x2800 + bits)
                line += colour + ch + ANSI["reset"]
        print(line)

    # x-axis
    axis = "          " + "└" + "─" * plot_w
    print(axis)

    # x tick labels: every 5 gain values
    tick_line = "           "
    cursor = len(tick_line)
    for gain in range(30, 80, 5):
        col = to_col(gain) // CELL_W
        target = 11 + col
        if target > cursor:
            tick_line += " " * (target - cursor)
        label = f"{gain}"
        tick_line += label
        cursor = target + len(label)
    print(tick_line)
    print(" " * 52 + f"{ANSI['dgray']}tx_gain{ANSI['reset']}")
    print()

    # legend
    print(f"  {ANSI['lgreen']}⠿{ANSI['reset']} measured"
          f"   {ANSI['blue']}⠿{ANSI['reset']} linear fit (P_out_dBm = 1.004·tx_gain − 53.93)"
          f"   {ANSI['yellow']}⠿{ANSI['reset']} EU 100 mW"
          f"   {ANSI['red']}⠿{ANSI['reset']} pad rating"
          f"   {ANSI['dgray']}⠿{ANSI['reset']} 1 mW")
    print()

    # footer: operating points
    print(f"{ANSI['cyan']}operating points:{ANSI['reset']}   "
          f"{ANSI['lgreen']}100 mW{ANSI['reset']} → tx_gain ≈ 73.5   "
          f"{ANSI['lgreen']} 25 mW{ANSI['reset']} → tx_gain ≈ 68   "
          f"{ANSI['lgreen']}  1 mW{ANSI['reset']} → tx_gain ≈ 54")
    return 0


if __name__ == "__main__":
    sys.exit(main())
