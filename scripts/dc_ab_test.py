#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
dc_ab_test.py -- DC blocker A/B test with varied SF/BW/power configs.

Sends ADVERTs from Heltec V3 companion via direct serial. Each config:
  1. set radio (persists to flash, reboots Heltec)
  2. sleep (wait for reboot)
  3. advert (sends 1 ADVERT over RF, reboots)
  4. collect frames from lora_trx UDP for flush_s seconds

Requires: lora_trx already running on UDP port 5556.

Usage:
    python3 scripts/dc_ab_test.py --serial /dev/cu.usbserial-0001
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone

import cbor2


@dataclass
class TestConfig:
    label: str
    sf: int
    bw_khz: float
    freq_mhz: float
    cr: int = 8
    tx_power: int | None = None


@dataclass
class Result:
    config: TestConfig
    advert_ok: bool = False
    frames: int = 0
    crc_ok: int = 0
    crc_fail: int = 0
    best_snr: float = -999.0
    detected_sfs: dict[int, int] = field(default_factory=dict)


# varied configs: different SF, BW, power — no duplicates
CONFIGS = [
    TestConfig("SF8/BW62k 2dBm", sf=8, bw_khz=62.5, freq_mhz=869.618, tx_power=2),
    TestConfig("SF7/BW125k 2dBm", sf=7, bw_khz=125.0, freq_mhz=869.618, tx_power=2),
    TestConfig("SF12/BW62k 2dBm", sf=12, bw_khz=62.5, freq_mhz=869.618, tx_power=2),
    TestConfig("SF10/BW125k 2dBm", sf=10, bw_khz=125.0, freq_mhz=869.618, tx_power=2),
    TestConfig("SF9/BW62k 2dBm", sf=9, bw_khz=62.5, freq_mhz=869.618, tx_power=2),
    TestConfig("SF7/BW250k 2dBm", sf=7, bw_khz=250.0, freq_mhz=869.618, tx_power=2),
    TestConfig("SF8/BW62k -4dBm", sf=8, bw_khz=62.5, freq_mhz=869.618, tx_power=-4),
    TestConfig("SF12/BW125k -4dBm", sf=12, bw_khz=125.0, freq_mhz=869.618, tx_power=-4),
    TestConfig("SF8/BW125k -9dBm", sf=8, bw_khz=125.0, freq_mhz=869.618, tx_power=-9),
    TestConfig("SF10/BW62k -4dBm", sf=10, bw_khz=62.5, freq_mhz=869.618, tx_power=-4),
]


def cli(serial: str, *args: str, timeout: float = 20.0) -> tuple[bool, str]:
    """Run meshcore-cli command via direct serial."""
    try:
        r = subprocess.run(
            ["uvx", "meshcore-cli", "-q", "-s", serial, *args],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return r.returncode == 0, r.stdout.strip()
    except Exception as e:
        return False, str(e)


def collect_frames(port: int, duration: float) -> list[dict]:
    """Subscribe to lora_trx UDP and collect lora_frame events."""
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


def run_config(
    cfg: TestConfig, serial: str, trx_port: int, reboot_wait: float, flush_s: float
) -> Result:
    result = Result(config=cfg)

    # 1. set radio (persists to flash, reboots Heltec)
    radio_str = f"{cfg.freq_mhz},{cfg.bw_khz},{cfg.sf},{cfg.cr}"
    ok, out = cli(serial, "set", "radio", radio_str)
    if not ok:
        print(f"  set radio FAIL: {out}", flush=True)
    else:
        print(f"  set radio {radio_str}: ok", flush=True)
    time.sleep(reboot_wait)

    # 2. set tx power (if specified — reboots again)
    if cfg.tx_power is not None:
        ok, out = cli(serial, "set", "tx", str(cfg.tx_power))
        if not ok:
            print(f"  set tx {cfg.tx_power} FAIL: {out}", flush=True)
        else:
            print(f"  set tx {cfg.tx_power}dBm: ok", flush=True)
        time.sleep(reboot_wait)

    # 3. subscribe to lora_trx UDP before sending advert
    sub_msg = cbor2.dumps({"type": "subscribe"})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    sock.bind(("", 0))
    sock.sendto(sub_msg, ("127.0.0.1", trx_port))

    # 4. send advert (reboots Heltec)
    ok, out = cli(serial, "advert")
    result.advert_ok = ok
    print(f"  advert: {'ok' if ok else 'FAIL'}", flush=True)

    # 5. collect frames
    frames: list[dict] = []
    deadline = time.monotonic() + flush_s
    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(65536)
            msg = cbor2.loads(data)
            if isinstance(msg, dict) and msg.get("type") == "lora_frame":
                frames.append(msg)
        except TimeoutError:
            # re-subscribe periodically
            sock.sendto(sub_msg, ("127.0.0.1", trx_port))
            continue
        except Exception:
            continue
    sock.close()

    # 6. analyze
    for ev in frames:
        phy = ev.get("phy", {})
        crc_ok = ev.get("crc_valid", False)
        snr = phy.get("snr_db", -999.0)
        sf = phy.get("sf", 0)

        result.frames += 1
        if crc_ok:
            result.crc_ok += 1
        else:
            result.crc_fail += 1
        if snr > result.best_snr:
            result.best_snr = snr
        if sf > 0:
            result.detected_sfs[sf] = result.detected_sfs.get(sf, 0) + 1

    return result


def main():
    parser = argparse.ArgumentParser(description="DC blocker A/B test")
    parser.add_argument("--serial", required=True, help="Heltec V3 serial port")
    parser.add_argument("--trx-port", type=int, default=5556)
    parser.add_argument(
        "--reboot-wait",
        type=float,
        default=12.0,
        help="seconds to wait after each serial command (Heltec reboot)",
    )
    parser.add_argument(
        "--flush",
        type=float,
        default=15.0,
        help="seconds to collect frames after advert",
    )
    parser.add_argument("--label", default="dc_ab")
    args = parser.parse_args()

    os.makedirs("tmp", exist_ok=True)

    print(f"DC A/B test: {len(CONFIGS)} configs, serial={args.serial}", flush=True)
    print(f"Requires: lora_trx running on UDP {args.trx_port}\n", flush=True)

    results: list[Result] = []
    for i, cfg in enumerate(CONFIGS):
        pwr = f" {cfg.tx_power}dBm" if cfg.tx_power is not None else ""
        print(
            f"[{i + 1}/{len(CONFIGS)}] {cfg.label} (SF{cfg.sf}/BW{cfg.bw_khz}k{pwr})",
            flush=True,
        )
        result = run_config(
            cfg, args.serial, args.trx_port, args.reboot_wait, args.flush
        )
        results.append(result)

    # summary table
    print(
        f"\n=== DC A/B: {args.label} ({datetime.now(timezone.utc).isoformat()}) ===\n"
    )
    print(
        f"{'Config':<28s} {'Adv':>3s} {'Frm':>3s} {'OK':>3s} {'Fail':>4s} "
        f"{'SF':<12s} {'SNR':>5s}"
    )
    print("-" * 65)
    total_frames = 0
    total_ok = 0
    total_fail = 0
    for r in results:
        sf_str = (
            ",".join(f"SF{sf}:{c}" for sf, c in sorted(r.detected_sfs.items())) or "-"
        )
        snr_str = f"{r.best_snr:.1f}" if r.best_snr > -999 else "-"
        adv = "ok" if r.advert_ok else "FAIL"
        print(
            f"{r.config.label:<28s} {adv:>3s} {r.frames:>3d} {r.crc_ok:>3d} "
            f"{r.crc_fail:>4d} {sf_str:<12s} {snr_str:>5s}"
        )
        total_frames += r.frames
        total_ok += r.crc_ok
        total_fail += r.crc_fail

    print(f"\nTotal: {total_frames} frames, {total_ok} CRC OK, {total_fail} CRC FAIL")

    # JSON output
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_path = f"tmp/dc_ab_{args.label}_{ts}.json"
    data = {
        "label": args.label,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "configs": len(CONFIGS),
        "total_frames": total_frames,
        "total_crc_ok": total_ok,
        "total_crc_fail": total_fail,
        "results": [
            {
                "label": r.config.label,
                "sf": r.config.sf,
                "bw_khz": r.config.bw_khz,
                "tx_power": r.config.tx_power,
                "advert_ok": r.advert_ok,
                "frames": r.frames,
                "crc_ok": r.crc_ok,
                "crc_fail": r.crc_fail,
                "best_snr": r.best_snr if r.best_snr > -999 else None,
                "detected_sfs": r.detected_sfs,
            }
            for r in results
        ],
    }
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"results: {out_path}")


if __name__ == "__main__":
    main()
