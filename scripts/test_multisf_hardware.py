#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
test_multisf_hardware.py -- Hardware A/B test for MultiSfDecoder.

Cycles through SF/packet-type combinations using a MeshCore companion device
(Heltec V3 on serial) as TX, and lora_trx (with MultiSfDecoder) as RX.
Records decode success/failure per config. Retries each test up to ATTEMPTS
times to account for companion reboot timing.

Requirements:
  - lora_trx running on UDP 5556 (with MultiSfDecoder)
  - Companion device on /dev/cu.usbserial-0001 (or --serial flag)
  - Both on same frequency/BW

Usage:
  .venv/bin/python3 scripts/test_multisf_hardware.py [--serial /dev/cu.usbserial-0001]
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone

import cbor2

# ---- Config ----
FREQ_MHZ = 869.618
BW_KHZ = 62.5
CR = 8
LORA_TRX_HOST = "127.0.0.1"
LORA_TRX_PORT = 5556
MESHCORE_CLI = ["uv", "run", "--with", "meshcore-cli", "meshcore-cli", "-q"]
REBOOT_WAIT = 14  # seconds after set radio (companion serial reboot)
INTER_CMD_WAIT = 13  # seconds between serial commands (companion DTR reboot)
DECODE_WAIT = {7: 5, 8: 6, 9: 8, 10: 12, 11: 16, 12: 22}
ATTEMPTS = 3


def subscribe_udp():
    """Open a UDP socket and subscribe to lora_trx."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LORA_TRX_HOST, 0))
    sock.settimeout(1.0)
    sub = cbor2.dumps({"subtype": {}})
    sock.sendto(sub, (LORA_TRX_HOST, LORA_TRX_PORT))
    for _ in range(10):
        try:
            sock.recv(4096)
        except socket.timeout:
            break
    return sock


def drain_frames(sock):
    """Drain any pending frames."""
    for _ in range(50):
        try:
            sock.recv(8192)
        except socket.timeout:
            break


def wait_for_frame(sock, timeout_s, expect_bytes_min=0):
    """Wait for a lora_frame CBOR message."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            data = sock.recv(8192)
        except socket.timeout:
            continue
        try:
            msg = cbor2.loads(data)
        except Exception:
            continue
        if not isinstance(msg, dict):
            continue
        if msg.get("type") == "lora_frame":
            payload = msg.get("payload", b"")
            if len(payload) >= expect_bytes_min:
                return msg
    return None


def cli_cmd(serial, *args):
    """Run a meshcore-cli command. Returns stdout."""
    cmd = MESHCORE_CLI + ["-s", serial] + list(args)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    return r.stdout.strip()


def set_radio(serial, sf):
    """Set companion radio to given SF, wait for reboot."""
    param = f"{FREQ_MHZ},{BW_KHZ},{sf},{CR}"
    result = cli_cmd(serial, "set", "radio", param)
    ok = "ok" in result.lower()
    if ok:
        time.sleep(REBOOT_WAIT)
    return ok


def send_advert(serial):
    """Trigger ADVERT from companion."""
    result = cli_cmd(serial, "advert")
    return "sent" in result.lower()


def send_message(serial, dest_name, text):
    """Send a message via companion."""
    cli_cmd(serial, "msg", dest_name, text)


def try_send_and_decode(sock, serial, label, sf, send_fn, min_bytes):
    """Try sending and decoding up to ATTEMPTS times. Returns result dict."""
    wait_s = DECODE_WAIT.get(sf, 10)

    for attempt in range(1, ATTEMPTS + 1):
        tag = f"  [{label}] attempt {attempt}/{ATTEMPTS}"
        print(f"{tag} (wait {wait_s}s)...", end="", flush=True)

        drain_frames(sock)
        send_fn()
        frame = wait_for_frame(sock, wait_s, min_bytes)

        if frame is not None:
            phy = frame.get("phy", {})
            rx_sf = phy.get("sf", "?")
            pay_len = len(frame.get("payload", b""))
            crc_ok = phy.get("crc_valid", False)
            snr = phy.get("snr_db", 0)
            status = "CRC_OK" if crc_ok else "CRC_FAIL"
            print(f"  -> {status} SF{rx_sf} {pay_len}B SNR={snr:.1f}dB")
            return {
                "label": label,
                "sf": sf,
                "rx_sf": rx_sf,
                "pay_len": pay_len,
                "crc_valid": crc_ok,
                "snr_db": round(snr, 1),
                "result": status,
                "attempt": attempt,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        print(f"  -> NO DECODE")
        if attempt < ATTEMPTS:
            print(
                f"      waiting {INTER_CMD_WAIT}s for companion reboot...", flush=True
            )
            time.sleep(INTER_CMD_WAIT)

    return {
        "label": label,
        "sf": sf,
        "result": "NO_DECODE",
        "attempts": ATTEMPTS,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }


def run_test_matrix(serial, results_file):
    """Run the full SF x packet-type test matrix."""
    sock = subscribe_udp()
    dest = "DO2THX \U0001f421"

    test_configs = [
        # Start with SF8 (known working) as sanity check
        (8, "SF8 msg (warmup)", lambda: send_message(serial, dest, "warmup"), 10),
        (8, "SF8 advert", lambda: send_advert(serial), 20),
        (8, "SF8 msg", lambda: send_message(serial, dest, "test SF8"), 10),
        # SF7
        (7, "SF7 advert", lambda: send_advert(serial), 20),
        (7, "SF7 msg", lambda: send_message(serial, dest, "test SF7"), 10),
        # SF10 (LDRO at BW62.5k)
        (10, "SF10 advert", lambda: send_advert(serial), 20),
        (10, "SF10 msg", lambda: send_message(serial, dest, "test SF10"), 10),
        # SF12 (LDRO, very long air time)
        (12, "SF12 advert", lambda: send_advert(serial), 20),
        (12, "SF12 msg", lambda: send_message(serial, dest, "test SF12"), 10),
    ]

    results = []
    current_sf = None

    print(f"\n{'=' * 70}")
    print(f"MultiSfDecoder Hardware A/B Test  ({ATTEMPTS} attempts per config)")
    print(f"Companion: {serial}  Freq: {FREQ_MHZ} MHz  BW: {BW_KHZ} kHz  CR: {CR}")
    print(f"{'=' * 70}\n")

    for sf, label, send_fn, min_bytes in test_configs:
        if sf != current_sf:
            print(f"\n--- Setting companion to SF{sf} ---")
            if not set_radio(serial, sf):
                print(f"  FAIL: could not set radio to SF{sf}")
                results.append(
                    {
                        "label": label,
                        "sf": sf,
                        "result": "RADIO_FAIL",
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                )
                continue
            current_sf = sf
            drain_frames(sock)

        result = try_send_and_decode(sock, serial, label, sf, send_fn, min_bytes)
        results.append(result)

    # Restore companion to SF8
    print(f"\n--- Restoring companion to SF8 ---")
    set_radio(serial, 8)
    sock.close()

    # Summary
    print(f"\n{'=' * 70}")
    print(f"RESULTS SUMMARY")
    print(f"{'=' * 70}")
    print(
        f"{'Label':<25} {'SF':>3} {'Result':<12} {'RX SF':>5} {'Bytes':>5} {'SNR':>7} {'Att':>3}"
    )
    print(f"{'-' * 70}")
    for r in results:
        rx_sf = r.get("rx_sf", "")
        pay = r.get("pay_len", "")
        snr = f"{r['snr_db']:.1f}" if "snr_db" in r else ""
        att = r.get("attempt", r.get("attempts", ""))
        print(
            f"{r['label']:<25} {r['sf']:>3} {r['result']:<12} {rx_sf!s:>5} {pay!s:>5} {snr:>7} {att!s:>3}"
        )

    passed = sum(1 for r in results if r["result"] == "CRC_OK")
    total = len(results)
    print(f"\n{passed}/{total} passed")

    # Save
    os.makedirs(os.path.dirname(results_file) or ".", exist_ok=True)
    with open(results_file, "w") as f:
        json.dump(
            {
                "test": "multisf_hardware",
                "binary": "ac0ffef",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "companion": serial,
                "freq_mhz": FREQ_MHZ,
                "bw_khz": BW_KHZ,
                "cr": CR,
                "results": results,
            },
            f,
            indent=2,
        )
    print(f"\nResults saved to {results_file}")
    return passed, total


def main():
    parser = argparse.ArgumentParser(description="MultiSfDecoder hardware A/B test")
    parser.add_argument("--serial", default="/dev/cu.usbserial-0001")
    parser.add_argument("--output", default="tmp/multisf_hw_test.json")
    parser.add_argument("--rounds", type=int, default=1)
    args = parser.parse_args()

    passed = total = 0
    for round_num in range(1, args.rounds + 1):
        if args.rounds > 1:
            print(f"\n{'#' * 70}")
            print(f"  ROUND {round_num}/{args.rounds}")
            print(f"{'#' * 70}")
            out = args.output.replace(".json", f"_r{round_num}.json")
        else:
            out = args.output
        passed, total = run_test_matrix(args.serial, out)

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
