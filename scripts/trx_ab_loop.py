#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
trx_ab_loop.py -- Automated A/B test loop for wideband decoder.

Manages the full lifecycle: serial_bridge + lora_trx + ADVERTs + collection.
Each round: build → start lora_trx → send ADVERTs → collect → stop → report.

Usage:
    python3 scripts/trx_ab_loop.py --serial /dev/cu.usbserial-0001 [--rounds 5]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import cbor2

SCRIPT_DIR = Path(__file__).parent


# ---- Process management ----


def start_process(cmd: list[str], log_path: str) -> subprocess.Popen:
    log_fd = open(log_path, "w")
    return subprocess.Popen(
        cmd, stdout=subprocess.DEVNULL, stderr=log_fd,
        preexec_fn=os.setpgrp,
    )


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass


# ---- serial_bridge ----


def start_bridge(serial: str, port: int, log_path: str) -> subprocess.Popen:
    return start_process(
        [sys.executable, str(SCRIPT_DIR / "serial_bridge.py"),
         "--serial", serial, "--tcp-port", str(port)],
        log_path,
    )


def wait_for_bridge(port: int, timeout: float = 15.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2.0)
            s.close()
            return True
        except Exception:
            time.sleep(0.5)
    return False


# ---- lora_trx ----


def start_trx(config_path: str, log_path: str) -> subprocess.Popen:
    return start_process(
        ["./build/apps/lora_trx", "--config", config_path],
        log_path,
    )


def wait_for_udp(port: int, timeout: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout
    sub_msg = cbor2.dumps({"type": "subscribe"})
    while time.monotonic() < deadline:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(2.0)
            sock.bind(("", 0))
            sock.sendto(sub_msg, ("127.0.0.1", port))
            sock.recvfrom(65536)
            sock.close()
            return True
        except Exception:
            if sock:
                sock.close()
            time.sleep(1.0)
    return False


# ---- Companion ----


def companion_cmd(bridge_port: int, *args: str, timeout: float = 20.0) -> tuple[bool, str]:
    try:
        r = subprocess.run(
            ["uvx", "meshcore-cli", "-q", "-t", "127.0.0.1", "-p", str(bridge_port), *args],
            capture_output=True, text=True, timeout=timeout,
        )
        return r.returncode == 0, r.stdout.strip()
    except Exception as e:
        return False, str(e)


# ---- Frame collector ----


def collect_frames(port: int, duration: float) -> list[dict]:
    sub_msg = cbor2.dumps({"type": "subscribe"})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.bind(("", 0))
    sock.sendto(sub_msg, ("127.0.0.1", port))

    frames: list[dict] = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(65536)
            msg = cbor2.loads(data)
            if isinstance(msg, dict) and msg.get("type") == "lora_frame":
                frames.append(msg)
        except TimeoutError:
            continue
        except Exception:
            continue
    sock.close()
    return frames


# ---- Build ----


def build_trx() -> bool:
    r = subprocess.run(
        ["cmake", "--build", "build", "--", "-j4"],
        capture_output=True, text=True, timeout=300,
    )
    if r.returncode != 0:
        print(f"BUILD FAILED:\n{r.stderr[-500:]}", flush=True)
    return r.returncode == 0


def run_unit_tests() -> tuple[bool, str]:
    r = subprocess.run(
        ["ctest", "--test-dir", "build", "-R", "qa_lora",
         "--output-on-failure", "--timeout", "60"],
        capture_output=True, text=True, timeout=300,
    )
    lines = r.stdout.strip().split("\n")
    summary = next((l for l in reversed(lines) if "tests passed" in l or "tests failed" in l), "")
    return r.returncode == 0, summary


# ---- Result ----


@dataclass
class RoundResult:
    round_num: int
    build_ok: bool = False
    tests_ok: bool = False
    test_summary: str = ""
    trx_started: bool = False
    frames_total: int = 0
    crc_ok: int = 0
    crc_fail: int = 0
    sfs: dict[int, int] = field(default_factory=dict)
    best_snr: float = -999.0
    cfo_ints: list[int] = field(default_factory=list)
    freq_mhz: list[float] = field(default_factory=list)
    duration_s: float = 0.0
    notes: str = ""


def print_round(r: RoundResult) -> None:
    sf_str = ",".join(f"SF{sf}:{c}" for sf, c in sorted(r.sfs.items())) if r.sfs else "-"
    snr_str = f"{r.best_snr:.1f}" if r.best_snr > -999.0 else "-"
    cfo_str = f"{min(r.cfo_ints)}..{max(r.cfo_ints)}" if r.cfo_ints else "-"
    status = "PASS" if r.crc_ok > 0 else ("BUILD_FAIL" if not r.build_ok else "NO_DECODE")
    print(
        f"  R{r.round_num}: {status}  frames={r.frames_total} "
        f"crc_ok={r.crc_ok} crc_fail={r.crc_fail} "
        f"snr={snr_str} sfs={sf_str} cfo={cfo_str} "
        f"({r.duration_s:.0f}s) {r.notes}",
        flush=True,
    )


# ---- Round ----


def run_round(
    round_num: int,
    config_path: str,
    trx_port: int,
    bridge_port: int,
    adverts: int,
    gap_s: float,
    flush_s: float,
    skip_build: bool,
) -> RoundResult:
    result = RoundResult(round_num=round_num)
    t0 = time.monotonic()

    # 1. Build
    if not skip_build:
        print(f"\n{'='*60}\nRound {round_num}: build", flush=True)
        if not build_trx():
            result.notes = "build failed"
            result.duration_s = time.monotonic() - t0
            return result
    result.build_ok = True

    # 2. Unit tests
    print(f"Round {round_num}: unit tests", flush=True)
    ok, summary = run_unit_tests()
    result.tests_ok = ok
    result.test_summary = summary
    print(f"  {summary}", flush=True)
    if not ok:
        result.notes = f"tests: {summary}"
        # Continue despite test failures during wideband development

    # 3. Start lora_trx
    log_path = f"tmp/trx_r{round_num}.log"
    os.makedirs("tmp", exist_ok=True)
    print(f"Round {round_num}: starting lora_trx", flush=True)
    trx_proc = start_trx(config_path, log_path)

    try:
        if not wait_for_udp(trx_port, timeout=30):
            print(f"  ERROR: lora_trx didn't start (see {log_path})", flush=True)
            result.notes = "trx startup failed"
            result.duration_s = time.monotonic() - t0
            return result
        result.trx_started = True
        print(f"  lora_trx ready", flush=True)

        # 4. Subscribe early so we don't miss frames
        sub_msg = cbor2.dumps({"type": "subscribe"})
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        sock.bind(("", 0))
        sock.sendto(sub_msg, ("127.0.0.1", trx_port))

        # 5. Wait for radio to settle
        time.sleep(3.0)

        # 6. Send ADVERTs
        print(f"Round {round_num}: sending {adverts} ADVERTs", flush=True)
        for i in range(adverts):
            ok, _ = companion_cmd(bridge_port, "advert")
            print(f"  advert {i+1}/{adverts}: {'ok' if ok else 'FAIL'}", flush=True)
            if i < adverts - 1:
                time.sleep(gap_s)

        # 7. Collect frames
        print(f"Round {round_num}: collecting ({flush_s}s)...", flush=True)
        frames: list[dict] = []
        deadline = time.monotonic() + flush_s
        while time.monotonic() < deadline:
            try:
                data, _ = sock.recvfrom(65536)
                msg = cbor2.loads(data)
                if isinstance(msg, dict) and msg.get("type") == "lora_frame":
                    frames.append(msg)
                    phy = msg.get("phy", {})
                    sf = phy.get("sf", "?")
                    crc = "OK" if msg.get("crc_valid") else "FAIL"
                    snr = phy.get("snr_db", 0)
                    freq = phy.get("channel_freq", 0)
                    freq_str = f" freq={freq/1e6:.3f}" if freq else ""
                    cfo = phy.get("cfo_int")
                    cfo_str = f" cfo={int(cfo)}" if cfo is not None else ""
                    print(f"    FRAME SF{sf} CRC={crc} SNR={snr:.1f}{freq_str}{cfo_str}", flush=True)
            except TimeoutError:
                continue
            except Exception:
                continue
        sock.close()

        # 8. Analyze
        for ev in frames:
            phy = ev.get("phy", {})
            crc_ok = ev.get("crc_valid", False)
            snr = phy.get("snr_db", -999.0)
            sf = phy.get("sf", 0)
            cfo_int = phy.get("cfo_int")
            ch_freq = phy.get("channel_freq", 0)

            result.frames_total += 1
            if crc_ok:
                result.crc_ok += 1
            else:
                result.crc_fail += 1
            if snr > result.best_snr:
                result.best_snr = snr
            if sf > 0:
                result.sfs[sf] = result.sfs.get(sf, 0) + 1
            if cfo_int is not None:
                result.cfo_ints.append(int(cfo_int))
            if ch_freq > 0:
                result.freq_mhz.append(ch_freq / 1e6)

    finally:
        stop_process(trx_proc)

    result.duration_s = time.monotonic() - t0
    return result


# ---- Main ----


def main() -> None:
    parser = argparse.ArgumentParser(description="Automated wideband A/B test loop")
    parser.add_argument("--serial", required=True, help="companion serial port")
    parser.add_argument("--rounds", type=int, default=5)
    parser.add_argument("--config", default="apps/config.toml")
    parser.add_argument("--trx-port", type=int, default=5556)
    parser.add_argument("--bridge-port", type=int, default=7835)
    parser.add_argument("--adverts", type=int, default=3)
    parser.add_argument("--gap", type=float, default=5.0)
    parser.add_argument("--flush", type=float, default=15.0)
    parser.add_argument("--no-build", action="store_true")
    args = parser.parse_args()

    os.makedirs("tmp", exist_ok=True)

    # Start serial_bridge
    print(f"Starting serial_bridge on {args.serial} port {args.bridge_port}", flush=True)
    bridge_proc = start_bridge(args.serial, args.bridge_port, "tmp/bridge.log")
    if not wait_for_bridge(args.bridge_port):
        print("ERROR: serial_bridge didn't start (see tmp/bridge.log)", flush=True)
        stop_process(bridge_proc)
        sys.exit(1)
    print("serial_bridge ready", flush=True)

    # Verify companion
    ok, radio = companion_cmd(args.bridge_port, "get", "radio")
    if not ok:
        print("ERROR: companion not responding", flush=True)
        stop_process(bridge_proc)
        sys.exit(1)
    print(f"companion: {radio}", flush=True)

    results: list[RoundResult] = []

    try:
        for i in range(1, args.rounds + 1):
            result = run_round(
                round_num=i,
                config_path=args.config,
                trx_port=args.trx_port,
                bridge_port=args.bridge_port,
                adverts=args.adverts,
                gap_s=args.gap,
                flush_s=args.flush,
                skip_build=args.no_build or i > 1,  # only build on round 1
            )
            results.append(result)
            print_round(result)
    except KeyboardInterrupt:
        print("\nInterrupted", flush=True)
    finally:
        stop_process(bridge_proc)

    # Summary
    print(f"\n{'='*60}\nA/B TEST SUMMARY\n{'='*60}", flush=True)
    for r in results:
        print_round(r)

    total_ok = sum(r.crc_ok for r in results)
    total_fail = sum(r.crc_fail for r in results)
    total_frames = sum(r.frames_total for r in results)
    print(f"\nTotal: {total_frames} frames, {total_ok} CRC OK, {total_fail} CRC FAIL", flush=True)

    # JSON
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_path = f"tmp/trx_ab_{ts}.json"
    doc = {
        "test": "trx_ab_loop",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "rounds": [
            {
                "round": r.round_num,
                "build_ok": r.build_ok,
                "tests_ok": r.tests_ok,
                "test_summary": r.test_summary,
                "frames": r.frames_total,
                "crc_ok": r.crc_ok,
                "crc_fail": r.crc_fail,
                "best_snr": round(r.best_snr, 1) if r.best_snr > -999.0 else None,
                "sfs": {str(k): v for k, v in sorted(r.sfs.items())},
                "cfo_ints": r.cfo_ints,
                "freq_mhz": [round(f, 3) for f in r.freq_mhz],
                "duration_s": round(r.duration_s, 1),
                "notes": r.notes,
            }
            for r in results
        ],
    }
    with open(out_path, "w") as f:
        json.dump(doc, f, indent=2)
    print(f"Results: {out_path}", flush=True)


if __name__ == "__main__":
    main()
