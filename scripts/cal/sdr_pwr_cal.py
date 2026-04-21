#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""SDR TX power calibration via TinySA Ultra.

Sweeps tx_gain x frequency by spawning a fresh lora_trx per
(freq, tx_gain) point with those values baked into a derived
config.toml.  Runtime settings changes are avoided entirely:
each measurement point is an isolated trx process with its gain
and frequency fixed at graph build.

Lifecycle mirrors scripts/apps/lora_test.py:

- Each (freq, tx_gain) point spawns lora_trx with a tmp config.toml,
  waits for UDP readiness, runs the TinySA measurement, and sends
  SIGINT to let the trx exit cleanly before the next spawn.
- atexit + signal handlers on SIGINT/SIGTERM force a final teardown
  regardless of how the script exits.

Fixture (conducted test, no radiation):
    SDR TX port -> pad chain -> TinySA RF IN.  Default 60 dB
    (30 + 20 + 10 dB SMA) with all pads rated +23 dBm CW.
    ETSI duty-cycle does not apply.  Pad rating is the binding
    safety constraint, checked via --safety-threshold-source-dbm.

Supervision (mandatory):
    Operator watches the SDR's TX indicator.  If it stays on after
    a run ends, disconnect USB.

Per-frequency sweep ascends tx_gain; the first gain that clears
--detection-threshold-sa-dbm is recorded as the sensitivity floor.

Output per frequency:
    data/testing/tx_cal/{label}_f{freq_khz}_{ts}.json   -- sweep points
    data/testing/tx_cal/{label}_summary_{ts}.json       -- combined
"""
from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import pathlib
import re
import secrets
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from typing import Optional

import cbor2
import numpy as np

from tinysa import TinySA, autodetect_port, compute_aclr


# --- lora_trx process lifecycle ---------------------------------------------
#
# Pattern copied from scripts/apps/lora_test.py::_start_process/_stop_process.
# preexec_fn=os.setpgrp puts the child in a new process group so a single
# killpg takes out any grandchildren.  SIGINT lets lora_trx run its own
# shutdown path (quick_exit with a clean log line); a 5 s timeout then
# SIGKILL is the hard backstop.

def _start_lora_trx(binary: str, config_path: str, log_path: str
                    ) -> subprocess.Popen:
    os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
    log_fd = open(log_path, "w", encoding="utf-8")
    return subprocess.Popen(
        [binary, "--config", config_path],
        stdout=subprocess.DEVNULL,
        stderr=log_fd,
        preexec_fn=os.setpgrp,
    )


def _stop_lora_trx(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2)
            except Exception:
                pass


def _wait_udp_ready(port: int, timeout: float = 30.0) -> bool:
    """Send {type:subscribe} CBOR until lora_trx replies.

    lora_trx registers the sender as a subscriber and echoes its config
    CBOR back — a received reply confirms the UDP loop + RX + TX
    schedulers are up.
    """
    sub_msg = cbor2.dumps({"type": "subscribe"})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    sock.bind(("", 0))
    deadline = time.monotonic() + timeout
    next_sub = 0.0
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_sub:
                sock.sendto(sub_msg, ("127.0.0.1", port))
                next_sub = now + 5.0
            try:
                sock.recvfrom(65536)
                return True
            except socket.timeout:
                continue
            except Exception:
                time.sleep(1.0)
    finally:
        sock.close()
    return False


# --- Per-point config derivation --------------------------------------------

def write_point_config(base_path: str, radio_section: str,
                        freq_hz: float, tx_gain: float,
                        out_path: str) -> None:
    """Write a derived config.toml with `freq` and `tx_gain` overridden
    in `[<radio_section>]`.

    Minimal TOML mutation: parses section headers only.  `tx_gain` is
    inserted right after the section header if absent; `freq` is
    replaced in-place if present.  Base config must have a line
    `freq = <N>` inside the target section (apps/config.toml does).
    """
    src = pathlib.Path(base_path).read_text(encoding="utf-8")
    target_header = f"[{radio_section}]"
    lines = src.splitlines()
    out: list = []
    in_target = False
    saw_tx_gain = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            # New section boundary; insert tx_gain before leaving target.
            if in_target and not saw_tx_gain:
                out.append(f"tx_gain    = {tx_gain:.3f}"
                           f"        # injected by sdr_pwr_cal.py")
                saw_tx_gain = True
            in_target = (stripped == target_header)
            out.append(line)
            continue
        if in_target:
            m_freq = re.match(r"^(\s*freq\s*=\s*)\S+(.*)", line)
            if m_freq:
                out.append(f"{m_freq.group(1)}{freq_hz:.0f}{m_freq.group(2)}")
                continue
            m_gain = re.match(r"^(\s*tx_gain\s*=\s*)\S+(.*)", line)
            if m_gain:
                out.append(f"{m_gain.group(1)}{tx_gain:.3f}{m_gain.group(2)}")
                saw_tx_gain = True
                continue
        out.append(line)
    if in_target and not saw_tx_gain:
        out.append(f"tx_gain    = {tx_gain:.3f}"
                   f"        # injected by sdr_pwr_cal.py")
    pathlib.Path(out_path).write_text("\n".join(out) + "\n", encoding="utf-8")


# --- CBOR helpers ---

def send_tx_cbor(sock: socket.socket, host: str, port: int,
                 seq: int, payload: bytes,
                 gap_ms: int = 50) -> None:
    """Send a lora_tx CBOR datagram.  Fire-and-forget."""
    sock.sendto(cbor2.dumps({
        "type": "lora_tx",
        "seq": seq,
        "payload": payload,
        "gap_ms": gap_ms,
    }), (host, port))


def wait_tx_ack(sock: socket.socket, want_seq: int,
                 timeout_s: float = 3.0) -> bool:
    """Block until lora_trx emits lora_tx_ack for `want_seq` (or timeout).

    Returns True when the matching ack arrives.  ACK's arrival means
    lora_trx's TX worker has finished the burst — BURST_ACK received
    (UHD acks the TX burst, so the SDR is idle
    before moving on.  Ignores unrelated CBOR traffic (status, spectrum)
    so we don't wait on heartbeat cadence.
    """
    deadline = time.monotonic() + timeout_s
    saved = sock.gettimeout()
    try:
        while time.monotonic() < deadline:
            sock.settimeout(max(0.05, deadline - time.monotonic()))
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except Exception:
                continue
            try:
                msg = cbor2.loads(data)
            except Exception:
                continue
            if (isinstance(msg, dict)
                    and msg.get("type") == "lora_tx_ack"
                    and msg.get("seq") == want_seq):
                return True
    finally:
        sock.settimeout(saved)
    return False


# 16-byte payload is enough to show a recognisable LoRa symbol stream
# without being wasteful.  Fixture is conducted (SDR -> pads -> SA)
# so there is no regulatory airtime budget — the cost we're watching
# is wall-clock time per sweep + thermal creep.
BURST_PAYLOAD_BYTES = 16


# No background-burster class: the measurement now fires exactly ONE
# lora_tx burst synchronously and reads the SA with hop() while that
# burst is in flight, then blocks on lora_tx_ack so the SDR is
# provably idle before the next measurement setup.  Any persistent
# background burster leaks TX time past the hop window — empirically
# causing "SA paused while TX is active" — no upside now that
# hop is the measurement primitive.


# --- Per-point spawned lora_trx context manager -----------------------------

class _PointContext:
    """One (freq, tx_gain) point: derive config, spawn trx, wait ready.

    Guarantees on __exit__: SIGINT lora_trx, wait for clean exit.  Crashes
    in the measurement code still get the script-level safety probe via
    atexit.
    """

    # Monotonic spawn counter — used to make each spawn's log file unique
    # so re-running the same (freq, gain) point (probe then sweep) doesn't
    # overwrite the earlier log.
    _n: int = 0

    def __init__(self, *, binary: str, base_config: str, radio_section: str,
                 freq_hz: float, tx_gain: float, udp_port: int,
                 log_dir: str, tmp_dir: str,
                 ready_timeout: float = 45.0):
        self.binary = binary
        self.base_config = base_config
        self.radio_section = radio_section
        self.freq_hz = freq_hz
        self.tx_gain = tx_gain
        self.udp_port = udp_port
        self.log_dir = log_dir
        self.tmp_dir = tmp_dir
        self.ready_timeout = ready_timeout
        self.proc: Optional[subprocess.Popen] = None
        self.cfg_path: Optional[str] = None
        self.log_path: Optional[str] = None

    def __enter__(self) -> "_PointContext":
        # Include a monotonic counter in the log filename so every spawn
        # gets its own log — probe / sweep re-use the same (freq, gain)
        # point and a shared filename clobbers the earlier spawn's log,
        # hiding diagnostic info when one of them fails.
        _PointContext._n += 1
        seq = _PointContext._n
        tag = (f"f{int(self.freq_hz/1e3)}_g{self.tx_gain:.2f}_{seq:03d}"
               .replace(".", "p"))
        self.cfg_path = os.path.join(self.tmp_dir, f"sdr_pwr_cal_{tag}.toml")
        self.log_path = os.path.join(self.log_dir,
                                      f"sdr_pwr_cal_trx_{tag}.log")
        write_point_config(self.base_config, self.radio_section,
                           self.freq_hz, self.tx_gain, self.cfg_path)
        self.proc = _start_lora_trx(self.binary, self.cfg_path, self.log_path)
        if not _wait_udp_ready(self.udp_port, timeout=self.ready_timeout):
            # Fail fast: tear down before raising so the safety probe fires.
            self.__exit__(None, None, None)
            raise RuntimeError(
                f"lora_trx not ready on udp:{self.udp_port} "
                f"(see {self.log_path})")
        return self

    def __exit__(self, *_exc) -> None:
        # SIGINT -> lora_trx shutdown path -> quick_exit(0).  Any
        # residual USB state is resolved by the next spawn's own
        # device init (SoapySDR enumerate + UHD reinit).
        _stop_lora_trx(self.proc)
        self.proc = None


# --- SA scan + ICP ---

# Delay from send_tx_cbor returning to TX actually radiating.  Covers:
# UDP datagram queueing (~ms), lora_trx TxRequestQueue pop + IQ gen
# (~ms), SoapySink buffering, UHD tx_streamer send + timed-TX pipeline,
# TX chain activation + PLL re-lock (~10 ms).  Empirically ~80 ms
# on B210; 100 ms gives margin so the SA sweep lands inside the
# in-flight burst window.
_TX_LIFTOFF_S = 0.10

# Maximum LoRa payload (firmware limit 255 bytes; use 254 to stay safe).
# At SF8/BW62.5k/CR4/8 the airtime of one 254-byte packet is ~2.15 s.
# scanraw(averages=2) at ~1 s per sweep = ~2 s, so a single big burst
# covers the entire SA integration window with margin.
BIG_PAYLOAD_BYTES = 254


def _single_burst_measure_icp(tsa: TinySA, sock: socket.socket,
                                host: str, port: int, seq: int,
                                *, center_hz: float, span_hz: float,
                                rbw_khz: float, main_bw_hz: float,
                                points: int, averages: int,
                                attenuation_db: float) -> dict:
    """Fire ONE long TX burst, waitscan+scanraw during its airtime.

    `hop` gave ~16 µs sample windows — far too short to time-integrate
    a CSS chirp (symbol period 4.096 ms at SF8/BW62.5k).  Back to
    scanraw + compute_aclr which does proper IF-filter time-integration
    across the waitscan window.

    Sequence:
      1. RBW + sweep range config; SA resume (screen stays animated).
      2. Fire ONE 254-byte lora_tx burst (~2.15 s airtime at SF8/BW62.5k).
      3. Wait _TX_LIFTOFF_S so the burst is actually radiating.
      4. waitscan(averages) — SA integrates while the burst is in flight.
      5. scanraw to read the averaged trace.
      6. wait_tx_ack — TX chain powered down, SDR idle.

    The big payload guarantees the burst outlasts the SA integration
    window; no ContinuousBurster thread, no TX leak past ack.
    """
    start = center_hz - span_hz / 2.0
    stop = center_hz + span_hz / 2.0
    tsa.set_rbw_khz(rbw_khz)
    tsa.set_sweep(start, stop)
    tsa.resume()
    send_tx_cbor(sock, host, port, seq,
                 secrets.token_bytes(BIG_PAYLOAD_BYTES))
    time.sleep(_TX_LIFTOFF_S)
    tsa.waitscan(averages)
    trace = tsa.scanraw(start, stop, points)
    # Ack arrival signals TX chain has powered down.  Long timeout to
    # cover the 2+ s airtime of a 254-byte SF8/BW62.5k burst.
    wait_tx_ack(sock, seq, timeout_s=6.0)
    peak_sa = float(np.max(trace.dbm))
    aclr = compute_aclr(trace, center_hz, main_bw_hz, [])
    icp_sa = aclr["main_dbm"]
    if math.isnan(icp_sa):
        icp_source = math.nan
        icp_mw = math.nan
    else:
        icp_source = icp_sa + attenuation_db
        icp_mw = 10.0 ** (icp_source / 10.0)
    return {
        "sa_peak_dbm": peak_sa,
        "p_source_peak_dbm": peak_sa + attenuation_db,
        "sa_icp_dbm": icp_sa,
        "p_source_icp_dbm": icp_source,
        "p_source_icp_mw": icp_mw,
        "bin_hz": aclr["bin_hz"],
    }


# --- One-point measurement (spawn + dwell + measure) ------------------------

def _measure_one_point(tsa: TinySA, *, host: str, port: int,
                        binary: str, base_config: str, radio_section: str,
                        freq_hz: float, tx_gain: float,
                        span_hz: float, rbw_khz: float, main_bw_hz: float,
                        points: int, averages: int, attenuation_db: float,
                        log_dir: str, tmp_dir: str) -> dict:
    """Spawn a fresh lora_trx at (freq, tx_gain), run one ICP measurement,
    tear down.  Returns the measure_icp dict.
    """
    with _PointContext(binary=binary, base_config=base_config,
                        radio_section=radio_section,
                        freq_hz=freq_hz, tx_gain=tx_gain,
                        udp_port=port, log_dir=log_dir,
                        tmp_dir=tmp_dir):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Initial burst to warm the TX chain; thermal settle.
            warmup_seq = 1
            send_tx_cbor(sock, host, port, warmup_seq,
                         secrets.token_bytes(BURST_PAYLOAD_BYTES))
            wait_tx_ack(sock, warmup_seq, timeout_s=3.0)
            time.sleep(0.6)  # thermal settle post-warmup
            print(f"    scan@{freq_hz/1e6:.3f}MHz (avg={averages})...",
                  flush=True)
            # Single-burst measurement primitive: fire one TX, read SA
            # during its airtime, block on ack so SDR is provably idle
            # before teardown.  No background burster, no linger.
            meas_seq = 2
            meas = _single_burst_measure_icp(
                tsa, sock, host, port, meas_seq,
                center_hz=freq_hz, span_hz=span_hz,
                rbw_khz=rbw_khz, main_bw_hz=main_bw_hz,
                points=points, averages=averages,
                attenuation_db=attenuation_db)
        finally:
            sock.close()
    return meas


# --- Main ---

def git_rev() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=pathlib.Path(__file__).resolve().parent.parent.parent)
        return out.decode("ascii").strip()
    except Exception:
        return "unknown"


def linear_fit(gains: list, p_dbm: list) -> dict:
    """Fit P_dBm = slope * gain + intercept.  Skip NaN points."""
    xs = np.array([g for g, p in zip(gains, p_dbm)
                   if not (p is None or math.isnan(p))])
    ys = np.array([p for p in p_dbm if not (p is None or math.isnan(p))])
    if len(xs) < 2:
        return {"slope_db_per_gain_unit": math.nan,
                "intercept_dbm": math.nan, "r_squared": math.nan}
    slope, intercept = np.polyfit(xs, ys, 1)
    y_pred = slope * xs + intercept
    ss_res = float(np.sum((ys - y_pred) ** 2))
    ss_tot = float(np.sum((ys - np.mean(ys)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else math.nan
    return {"slope_db_per_gain_unit": float(slope),
            "intercept_dbm": float(intercept), "r_squared": r2}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="SDR TX power calibration via TinySA Ultra (spawn-per-point).")
    ap.add_argument("--port", default=None,
                    help="TinySA serial port (autodetect VID:PID 0483:5740)")
    ap.add_argument("--lora-trx-host", default="127.0.0.1")
    ap.add_argument("--lora-trx-port", type=int, default=5556)
    ap.add_argument("--freqs", type=float, nargs="+",
                    default=[863.5e6, 865.5e6, 867.5e6, 869.618e6, 869.8e6])
    ap.add_argument("--gains", type=float, nargs="+",
                    default=[30, 40, 50, 55, 60, 65, 70, 75, 80, 85, 89.75])
    ap.add_argument("--detection-threshold-sa-dbm", type=float, default=-90.0,
                    help="Points whose SA peak falls below this are flagged "
                         "detected=false; the first gain that clears it is "
                         "recorded as the per-freq sensitivity floor.")
    ap.add_argument("--attenuation-db", type=float, default=60.0,
                    help="External pad chain dB.  Default 60 dB (30+20+10) "
                         "per bootstrap-paradox discipline for B220 Mini at "
                         "max gain.")
    ap.add_argument("--main-bw", type=float, default=62.5e3,
                    help="Integration bandwidth for ICP (Hz)")
    ap.add_argument("--span", type=float, default=500e3,
                    help="SA sweep span (Hz)")
    ap.add_argument("--rbw", type=float, default=3.0, help="RBW kHz")
    ap.add_argument("--points", type=int, default=290)
    ap.add_argument("--averages", type=int, default=3)
    ap.add_argument("--safety-threshold-sa-dbm", type=float, default=-15.0,
                    help="Abort if SA peak exceeds this (SA damage gate). "
                         "Usually superseded by the pad-rating gate below "
                         "since CSS peak << integrated channel power.")
    ap.add_argument("--safety-threshold-source-dbm", type=float, default=20.0,
                    help="Abort if the computed source ICP exceeds this "
                         "(pad-rating gate).  Default +20 dBm = 3 dB under "
                         "a typical +23 dBm CW attenuator rating.  Set to "
                         "match the RATING OF YOUR FIRST PAD minus 3 dB.")
    ap.add_argument("--label", default="b220_868ism")
    ap.add_argument("--outdir", type=pathlib.Path,
                    default=pathlib.Path("data/testing/tx_cal"))
    ap.add_argument("--lora-trx-binary", default="./build/apps/lora_trx",
                    help="Path to lora_trx binary (script spawns + kills)")
    ap.add_argument("--base-config", default="apps/config.toml",
                    help="Base TOML config; tx_gain+freq are templated per point")
    ap.add_argument("--radio-section", default="radio_868",
                    help="TOML section to override (default: radio_868)")
    ap.add_argument("--log-dir", default="tmp",
                    help="Per-point lora_trx logs directory")
    args = ap.parse_args()

    # Fail fast if base config isn't present — spawning would die inside
    # _PointContext with a harder-to-interpret error.
    if not os.path.isfile(args.base_config):
        print(f"ERROR: --base-config {args.base_config!r} not found",
              file=sys.stderr)
        return 1
    if not os.access(args.lora_trx_binary, os.X_OK):
        print(f"ERROR: --lora-trx-binary {args.lora_trx_binary!r} not executable",
              file=sys.stderr)
        return 1

    args.outdir.mkdir(parents=True, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d%H%M%S")

    port = args.port or autodetect_port()
    if not port:
        print("ERROR: no TinySA Ultra autodetected", file=sys.stderr)
        return 1
    print(f"TinySA on {port}", flush=True)
    tsa = TinySA(port)
    # Ultra mode is a product/firmware variant, not a USB-toggleable
    # state — see https://tinysa.org/wiki/pmwiki.php?n=Main.USBInterface
    # which omits any `ultra on/off` command.  Nothing to configure.

    host = args.lora_trx_host
    port_trx = args.lora_trx_port
    tmp_dir = tempfile.mkdtemp(prefix="sdr_pwr_cal_")

    summary = {
        "label": args.label,
        "git_rev": git_rev(),
        "timestamp_unix": time.time(),
        "fixture": {
            "attenuation_db": args.attenuation_db,
            "pad_chain": "30+20+10 dB SMA, DC-4 GHz, 23 dBm CW (60 dB "
                         "bootstrap default)",
            "sa_model": "TinySA Ultra / ZS-406",
        },
        "sweep_params": {
            "freqs_hz": args.freqs,
            "gains": args.gains,
            "main_bw_hz": args.main_bw,
            "span_hz": args.span,
            "rbw_khz": args.rbw,
            "points": args.points,
            "averages": args.averages,
        },
        "per_freq": {},
        "freq_flatness": [],
        "uncertainty_db": 2.2,
    }
    aborted = False

    # Top-level teardown: each _PointContext already tears its own
    # lora_trx down on exit; this atexit/signal path only needs to
    # release the TinySA serial handle (and leave the SA in resume so
    # its screen trace keeps running if the operator plugs it into
    # another tool).
    _teardown_done = threading.Event()

    def _teardown():
        if _teardown_done.is_set():
            return
        _teardown_done.set()
        try:
            tsa.resume()
        except Exception:
            pass
        try:
            tsa.close()
        except Exception:
            pass

    def _signal_handler(signum, _frame):
        print(f"\nsignal {signum} received — tearing down", file=sys.stderr,
              flush=True)
        _teardown()
        sys.exit(128 + signum)

    atexit.register(_teardown)
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        # --- Full sweep: outer=freq, inner=gain.  First gain that clears
        # the detection threshold is recorded as the sensitivity floor;
        # no separate probe pass (the sweep itself characterises the
        # detected range).
        for freq in args.freqs:
            print(f"\n=== freq {freq/1e6:.3f} MHz ===", flush=True)
            freq_res: dict = {
                "freq_hz": freq,
                "detection_floor_tx_gain": None,
                "points": [],
            }

            for gain in args.gains:
                print(f"  [gain={gain:5.1f}] spawn-measure-teardown",
                      flush=True)
                try:
                    meas = _measure_one_point(
                        tsa, host=host, port=port_trx,
                        binary=args.lora_trx_binary,
                        base_config=args.base_config,
                        radio_section=args.radio_section,
                        freq_hz=freq, tx_gain=gain,
                        span_hz=args.span, rbw_khz=args.rbw,
                        main_bw_hz=args.main_bw, points=args.points,
                        averages=args.averages,
                        attenuation_db=args.attenuation_db,
                        log_dir=args.log_dir, tmp_dir=tmp_dir)
                except RuntimeError as e:
                    print(f"ABORT: tx_gain={gain} freq={freq/1e6:.3f} MHz: {e}",
                          flush=True)
                    aborted = True
                    break
                # Pad-rating abort (REACTIVE): if this measurement already
                # drove the first pad over its CW rating, stop the sweep
                # before transmitting the next (higher-gain) point.  The
                # current measurement has already happened; this prevents
                # the NEXT one from extending pad exposure.
                src_icp = meas["p_source_icp_dbm"]
                if (not math.isnan(src_icp)
                        and src_icp > args.safety_threshold_source_dbm):
                    print(f"ABORT: source ICP {src_icp:+.2f} dBm > "
                          f"pad-rating gate {args.safety_threshold_source_dbm} dBm "
                          f"(freq={freq/1e6:.3f} MHz gain={gain}) — "
                          f"stop before next point to protect pad chain",
                          flush=True)
                    aborted = True
                    break
                # SA-peak abort (diagnostic): catches freak peaks from
                # LO leakage / spurs that spike above the SA's damage
                # threshold even when integrated source ICP is still
                # safe.  Usually covered by the pad-rating gate above
                # since SA peak << source ICP for CSS chirp.
                if meas["sa_peak_dbm"] > args.safety_threshold_sa_dbm:
                    print(f"ABORT: SA peak {meas['sa_peak_dbm']:+.1f} dBm > "
                          f"SA gate {args.safety_threshold_sa_dbm} dBm "
                          f"(freq={freq/1e6:.3f} MHz gain={gain})",
                          flush=True)
                    aborted = True
                    break
                noise_limited = (meas["sa_icp_dbm"] < -95.0
                                 if not math.isnan(meas["sa_icp_dbm"])
                                 else True)
                detected = (not math.isnan(meas["sa_peak_dbm"])
                            and meas["sa_peak_dbm"]
                            > args.detection_threshold_sa_dbm)
                if (detected
                        and freq_res["detection_floor_tx_gain"] is None):
                    freq_res["detection_floor_tx_gain"] = gain
                point = {
                    "tx_gain": gain,
                    "sa_peak_dbm": meas["sa_peak_dbm"],
                    "p_source_peak_dbm": meas["p_source_peak_dbm"],
                    "sa_icp_dbm": meas["sa_icp_dbm"],
                    "p_source_icp_dbm": meas["p_source_icp_dbm"],
                    "p_source_icp_mw": meas["p_source_icp_mw"],
                    "noise_limited": noise_limited,
                    "detected": detected,
                }
                freq_res["points"].append(point)
                flags = []
                if not detected: flags.append("not-detected")
                if noise_limited: flags.append("noise-limited")
                flag_str = f" ({', '.join(flags)})" if flags else ""
                print(f"    tx_gain={gain:5.1f}: "
                      f"icp_src={meas['p_source_icp_dbm']:+.2f} dBm "
                      f"= {meas['p_source_icp_mw']:.4f} mW{flag_str}",
                      flush=True)
                # Bootstrap-paradox cross-check: >=2 prior points, fit,
                # halt ONLY if measured power exceeds the linear-fit
                # prediction by >10 dB — that means the source is
                # stronger than the pad chain was sized for (fixture
                # weaker than modelled, SA at risk).  Measurements
                # BELOW the prediction are diagnostic only (TX chain
                # compression, noise-limited floor, TX chain not warmed
                # up yet) and do not threaten hardware safety.
                if (len(freq_res["points"]) >= 2
                        and not math.isnan(meas["p_source_icp_dbm"])):
                    prior_gains = [p["tx_gain"] for p in freq_res["points"][:-1]
                                   if not p["noise_limited"]]
                    prior_dbm = [p["p_source_icp_dbm"] for p in freq_res["points"][:-1]
                                 if not p["noise_limited"]]
                    if len(prior_gains) >= 2:
                        fit = linear_fit(prior_gains, prior_dbm)
                        predicted = (fit["slope_db_per_gain_unit"] * gain
                                     + fit["intercept_dbm"])
                        excess = meas["p_source_icp_dbm"] - predicted
                        if excess > 10.0:
                            print(f"ABORT: tx_gain={gain} measured "
                                  f"{meas['p_source_icp_dbm']:+.2f} dBm vs "
                                  f"predicted {predicted:+.2f} dBm "
                                  f"(excess +{excess:.1f} dB > 10 dB — "
                                  f"source stronger than fixture sized for).",
                                  flush=True)
                            aborted = True
                            break

            gains_fit = [p["tx_gain"] for p in freq_res["points"]
                         if not p["noise_limited"]]
            dbm_fit = [p["p_source_icp_dbm"] for p in freq_res["points"]
                       if not p["noise_limited"]]
            freq_res["linear_fit"] = linear_fit(gains_fit, dbm_fit)
            print(f"  linear_fit: "
                  f"slope={freq_res['linear_fit']['slope_db_per_gain_unit']:.3f} "
                  f"dB/unit intercept={freq_res['linear_fit']['intercept_dbm']:.2f} dBm "
                  f"R^2={freq_res['linear_fit']['r_squared']:.4f}",
                  flush=True)

            per_file = args.outdir / f"{args.label}_f{int(freq/1e3)}_{ts}.json"
            with per_file.open("w") as f:
                json.dump({**summary, **freq_res,
                           "per_freq": None, "freq_flatness": None},
                          f, indent=2)
            print(f"  wrote {per_file}", flush=True)
            summary["per_freq"][f"{int(freq/1e3)}"] = {
                "file": str(per_file),
                "linear_fit": freq_res["linear_fit"],
                "n_points": len(freq_res["points"]),
            }

            at_70 = next((p for p in freq_res["points"] if p["tx_gain"] == 70.0),
                         None)
            if at_70:
                summary["freq_flatness"].append({
                    "freq_hz": freq,
                    "icp_at_gain_70_dbm": at_70["p_source_icp_dbm"],
                    "icp_at_gain_70_mw": at_70["p_source_icp_mw"],
                })

            if aborted:
                break

        summary["aborted"] = aborted
        summary_path = args.outdir / f"{args.label}_summary_{ts}.json"
        with summary_path.open("w") as f:
            json.dump(summary, f, indent=2)
        print(f"\nwrote summary {summary_path}", flush=True)

    finally:
        _teardown()

    return 2 if aborted else 0


if __name__ == "__main__":
    sys.exit(main())
