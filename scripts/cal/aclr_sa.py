#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""ACLR capture driver for TinySA Ultra (incl. ZS-406 clone).

Captures PRE/POST spectrum traces around 869.618 MHz via the TinySA's USB
CDC-ACM serial interface, computes adjacent-channel leakage ratio (dBc)
at two offsets, and writes a CSV trace + JSON summary + RGB565 PNG screen
dump per capture.

Fixture expectation (mandatory):
    B220 TX SMA -> [30 dB pad >= +20 dBm CW] -> [10 dB pad >= +20 dBm CW]
                -> TinySA Ultra RF IN
Default 40 dB attenuation is safe even if the SDR transiently outputs
+10 dBm (SA sees <= -30 dBm).

Measurement caveat: apps/config.toml uses lo_offset = 62500, so the LO
leakage spur sits at +62.5 kHz from the carrier -- exactly inside the
adj1 window of a BW62.5k measurement. The resulting ACLR+adj1 therefore
includes LO leakage; pre/post delta is still meaningful because
burst-taper changes skirt shape, not LO leakage.

TinySA serial protocol: https://tinysa.org/wiki/pmwiki.php?n=Main.USBInterface
RGB565 -> RGBA decode and '-:.0' firmware sanitiser adapted from
LC-Linkous/tinySA_python (MIT) -- https://github.com/LC-Linkous/tinySA_python
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import time

import numpy as np

# Shared TinySA client + measurement helpers.
from tinysa import (  # noqa: E402  (sibling module on sys.path via __file__)
    HAS_PIL,
    Trace,
    TinySA,
    autodetect_port,
    compute_aclr,
    measure_harmonic,
    render_rgb565_png,
)


def safety_interlock(tsa: TinySA, center_hz: float, args) -> None:
    """Abort if the TinySA sees more than --safety-threshold-dbm.

    Run BEFORE aclr_tx.py is dwelling. If peak > threshold while we are
    not yet transmitting, the fixture is catching stray RF far above
    the noise floor and something is wrong. Run AGAIN during dwell
    (caller responsibility); both checks apply.
    """
    tsa.pause()
    tsa.set_rbw_khz(args.rbw)
    start = center_hz - args.span / 2.0
    stop = center_hz + args.span / 2.0
    tsa.set_sweep(start, stop)
    tsa.resume()
    tsa.waitscan(2)
    tsa.pause()
    trace = tsa.scanraw(start, stop, args.points)
    peak = float(np.max(trace.dbm))
    if peak > args.safety_threshold_dbm:
        raise SystemExit(
            f"SAFETY ABORT: TinySA peak {peak:+.1f} dBm exceeds threshold "
            f"{args.safety_threshold_dbm:+.1f} dBm. Check attenuator chain "
            f"({args.attenuation_db} dB expected). Fixture may be damaged.")
    print(f"Safety check OK: peak {peak:+.1f} dBm "
          f"(threshold {args.safety_threshold_dbm:+.1f} dBm).")


def git_rev() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=pathlib.Path(__file__).resolve().parent.parent.parent)
        return out.decode("ascii").strip()
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser(
        description="ACLR capture via TinySA Ultra.")
    ap.add_argument("--port", default=None,
                    help="Serial port (default: autodetect VID:PID 0483:5740)")
    ap.add_argument("--center", type=float, default=869.618e6)
    ap.add_argument("--span", type=float, default=1.0e6)
    ap.add_argument("--rbw", type=float, default=3,
                    help="RBW in kHz (min 3 on Ultra)")
    ap.add_argument("--main-bw", type=float, default=62.5e3,
                    help="Main channel bandwidth in Hz (LoRa BW)")
    ap.add_argument("--adj-offset", type=float, nargs="+",
                    default=[62.5e3, 125e3],
                    help="Adjacent channel offsets in Hz")
    ap.add_argument("--points", type=int, default=290,
                    help="scanraw points (max 450 on Ultra)")
    ap.add_argument("--averages", type=int, default=4,
                    help="Number of waitscan cycles before capture")
    ap.add_argument("--attenuation-db", type=float, default=40.0,
                    help="External pad chain dB, for P_source derivation")
    ap.add_argument("--safety-threshold-dbm", type=float, default=-10.0,
                    help="Abort if peak exceeds this. Default -10 dBm "
                         "(20 dB margin below TinySA +10 dBm damage).")
    ap.add_argument("--no-safety", action="store_true",
                    help="Skip safety interlock (DEBUG ONLY).")
    ap.add_argument("--harmonics", action="store_true",
                    help="After ACLR, sweep harmonic orders listed in "
                         "--harmonic-orders and record peak + ETSI margin.")
    ap.add_argument("--harmonic-orders", type=int, nargs="+",
                    default=[2, 3, 4, 5],
                    help="Harmonic integer orders to sweep when --harmonics is set.")
    ap.add_argument("--harmonic-span-hz", type=float, default=10e6,
                    help="Span around each harmonic centre (Hz).")
    ap.add_argument("--harmonic-rbw-khz", type=float, default=30.0,
                    help="RBW for harmonic sweeps (kHz).")
    ap.add_argument("--label", required=True,
                    help="Filename tag, e.g. 'pre' or 'post-20db'")
    ap.add_argument("--outdir", type=pathlib.Path,
                    default=pathlib.Path("data/testing"))
    args = ap.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)

    port = args.port or autodetect_port()
    if not port:
        print("ERROR: no TinySA Ultra autodetected; pass --port /dev/...",
              file=sys.stderr)
        return 1

    print(f"Connecting to {port} ...")
    tsa = TinySA(port)
    try:
        print("Device info:")
        print(tsa.info().replace("\r\n", "\n  "))

        if not args.no_safety:
            print("Running safety interlock...")
            safety_interlock(tsa, args.center, args)

        start = args.center - args.span / 2.0
        stop = args.center + args.span / 2.0
        print(f"Sweep {start/1e6:.3f}-{stop/1e6:.3f} MHz, "
              f"RBW {args.rbw} kHz, {args.points} points, "
              f"averages {args.averages}.")
        tsa.set_rbw_khz(args.rbw)
        tsa.set_sweep(start, stop)
        tsa.resume()
        tsa.waitscan(args.averages)
        tsa.pause()
        trace = tsa.scanraw(start, stop, args.points)

        # Derive P_source from SA peak + pad chain.
        peak_sa_dbm = float(np.max(trace.dbm))
        p_source_dbm = peak_sa_dbm + args.attenuation_db
        bw_tag = f"sf8bw{int(args.main_bw/1e3)}" if args.main_bw != 62.5e3 \
            else "sf8bw62_5"
        center_tag = f"{int(args.center/1e3)}"
        stem = f"aclr_{args.label}_{center_tag}_{bw_tag}"

        # Write CSV.
        csv_path = args.outdir / f"{stem}.csv"
        with csv_path.open("w") as f:
            f.write("freq_hz,dbm_at_sa\n")
            for fhz, d in zip(trace.freq_hz, trace.dbm):
                f.write(f"{fhz:.1f},{d:.3f}\n")
        print(f"wrote {csv_path}")

        # Compute ACLR.
        aclr = compute_aclr(trace, args.center, args.main_bw, args.adj_offset)

        # Integrated channel power referred to source + mW conversion.
        icp_dbm_source = (aclr["main_dbm"] + args.attenuation_db
                          if not math.isnan(aclr["main_dbm"]) else math.nan)
        icp_mw_source = (10.0 ** (icp_dbm_source / 10.0)
                         if not math.isnan(icp_dbm_source) else math.nan)

        # Optional harmonic sweep.
        harmonic_results: list[dict] = []
        if args.harmonics:
            print("\nHarmonic sweep...")
            for order in args.harmonic_orders:
                h_freq_mhz = order * args.center / 1e6
                print(f"  {order}f = {h_freq_mhz:.3f} MHz ...")
                h = measure_harmonic(
                    tsa, args.center, order,
                    args.harmonic_span_hz, args.harmonic_rbw_khz,
                    args.points, args.averages, args.attenuation_db)
                h_csv = args.outdir / f"{stem}_h{order}.csv"
                with h_csv.open("w") as f:
                    f.write("freq_hz,dbm_at_sa\n")
                    for fhz, d in zip(h["trace"].freq_hz, h["trace"].dbm):
                        f.write(f"{fhz:.1f},{d:.3f}\n")
                print(f"    wrote {h_csv}")
                h.pop("trace")
                print(f"    peak at source {h['peak_dbm_at_source']:+.1f} dBm, "
                      f"ETSI margin {h['margin_db']:+.1f} dB")
                harmonic_results.append(h)

        # Screen dump.
        png_path = args.outdir / f"{stem}.png"
        png_ok = False
        raw = tsa.capture_screen()
        if raw and HAS_PIL:
            png_ok = render_rgb565_png(raw, png_path)
            if png_ok:
                print(f"wrote {png_path}")
        if not png_ok:
            reason = "PIL missing" if not HAS_PIL else "capture short read"
            print(f"skipped PNG ({reason})")

        # JSON summary.
        summary = {
            "label": args.label,
            "git_rev": git_rev(),
            "timestamp_unix": time.time(),
            "sa_model": "TinySA Ultra (ZS-406)",
            "sa_peak_dbm": peak_sa_dbm,
            "attenuation_db": args.attenuation_db,
            "p_source_dbm_estimated": p_source_dbm,
            "icp_dbm_at_source": icp_dbm_source,
            "icp_mw_at_source": icp_mw_source,
            "center_hz": args.center,
            "span_hz": args.span,
            "rbw_khz": args.rbw,
            "main_bw_hz": args.main_bw,
            "adj_offsets_hz": args.adj_offset,
            "points": args.points,
            "averages": args.averages,
            "aclr": aclr,
            "harmonics": harmonic_results,
            "notes": [
                "lo_offset=62500 in apps/config.toml; LO leakage lands at "
                "+62.5 kHz from carrier (within adj1 window)",
                "TinySA Ultra RBW floor is 3 kHz per firmware",
                "Ultra dBm = raw/32 - 174 scale",
            ],
        }
        json_path = args.outdir / f"{stem}.json"
        with json_path.open("w") as f:
            json.dump(summary, f, indent=2)
        print(f"wrote {json_path}")

        # Terse console summary.
        print(f"\nP_source peak: {p_source_dbm:+.1f} dBm "
              f"(SA peak {peak_sa_dbm:+.1f} dBm + {args.attenuation_db:.0f} dB)")
        if not math.isnan(icp_mw_source):
            print(f"ICP ({args.main_bw/1e3:.1f} kHz main): "
                  f"{icp_dbm_source:+.2f} dBm = {icp_mw_source:.3f} mW")
        for off in args.adj_offset:
            key = f"adj_{int(round(off/1e3))}khz_dbc"
            print(f"  ACLR +/- {off/1e3:.1f} kHz: {aclr[key]:+.2f} dBc")
        if harmonic_results:
            print("Harmonics (source-referred, 100-kHz-normalised):")
            for h in harmonic_results:
                print(f"  {h['order']}f={h['freq_hz']/1e6:.1f} MHz "
                      f"peak {h['peak_dbm_at_source']:+.1f} dBm "
                      f"({h['peak_dbm_per_100khz_at_source']:+.1f} dBm/100 kHz) "
                      f"ETSI margin {h['margin_db']:+.1f} dB")
    finally:
        try:
            tsa.resume()
        except Exception:
            pass
        tsa.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
