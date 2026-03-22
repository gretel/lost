#!/usr/bin/env python3
"""DC spur diagnostic: measures center-bin power from lora_trx spectrum UDP stream.

Usage:
    python3 scripts/dc_diag.py [--host 127.0.0.1] [--port 5556] [--frames 20]

Subscribes to lora_trx UDP, reads spectrum CBOR messages, reports:
  - DC bin power (center bin, dBFS)
  - Noise floor (25th percentile, dBFS)
  - DC spur height above noise (dB)
  - Adjacent bin powers for context
"""

import argparse
import socket
import struct
import sys
import time

try:
    import cbor2
except ImportError:
    print("pip install cbor2", file=sys.stderr)
    sys.exit(1)


def decode_bins(raw: bytes) -> list[float]:
    n = len(raw) - (len(raw) % 4)
    return list(struct.unpack(f"<{n // 4}f", raw[:n]))


def noise_floor(bins: list[float]) -> float:
    s = sorted(bins)
    return s[len(s) // 4]  # 25th percentile


def main():
    ap = argparse.ArgumentParser(description="DC spur diagnostic")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--frames", type=int, default=20, help="spectrum frames to collect")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", 0))
    sock.settimeout(5.0)

    # subscribe
    sub = cbor2.dumps({"type": "subscribe"})
    sock.sendto(sub, (args.host, args.port))

    print(
        f"Subscribed to {args.host}:{args.port}, waiting for {args.frames} spectrum frames...",
        file=sys.stderr,
    )

    collected = 0
    dc_powers = []
    nf_powers = []
    spur_heights = []

    t0 = time.monotonic()
    while collected < args.frames:
        if time.monotonic() - t0 > 30:
            print("TIMEOUT: no spectrum frames in 30s", file=sys.stderr)
            break

        try:
            data, _ = sock.recvfrom(65536)
        except socket.timeout:
            # re-subscribe
            sock.sendto(sub, (args.host, args.port))
            continue

        try:
            msg = cbor2.loads(data)
        except Exception:
            continue

        if not isinstance(msg, dict):
            continue
        if msg.get("type") != "spectrum":
            continue

        bins_raw = msg.get("bins", b"")
        fft_size = msg.get("fft_size", 0)
        center_freq = msg.get("center_freq", 0.0)
        sample_rate = msg.get("sample_rate", 0.0)

        if not bins_raw or fft_size == 0:
            continue

        bins = decode_bins(bins_raw)
        n = len(bins)
        if n == 0:
            continue

        # DC bin is at center (fftshifted)
        dc_idx = n // 2
        dc_power = bins[dc_idx]
        nf = noise_floor(bins)
        spur_h = dc_power - nf

        # adjacent bins for context
        adj_lo = bins[dc_idx - 1] if dc_idx > 0 else float("nan")
        adj_hi = bins[dc_idx + 1] if dc_idx < n - 1 else float("nan")

        dc_powers.append(dc_power)
        nf_powers.append(nf)
        spur_heights.append(spur_h)
        collected += 1

        print(
            f"  [{collected:3d}] DC={dc_power:+6.1f} dB  NF={nf:+6.1f} dB  "
            f"spur={spur_h:+5.1f} dB  adj=[{adj_lo:+6.1f}, {adj_hi:+6.1f}]  "
            f"fft={fft_size} fc={center_freq / 1e6:.3f} MHz sr={sample_rate / 1e3:.0f} kHz"
        )

    if collected == 0:
        print("No spectrum frames received.", file=sys.stderr)
        sock.close()
        return

    avg_dc = sum(dc_powers) / len(dc_powers)
    avg_nf = sum(nf_powers) / len(nf_powers)
    avg_spur = sum(spur_heights) / len(spur_heights)

    print(f"\n--- SUMMARY ({collected} frames) ---")
    print(f"  avg DC bin:     {avg_dc:+6.1f} dBFS")
    print(f"  avg noise floor:{avg_nf:+6.1f} dBFS")
    print(f"  avg spur height:{avg_spur:+5.1f} dB above NF")
    print(f"  verdict: {'DC SPUR PRESENT' if avg_spur > 6.0 else 'DC spur mitigated'}")

    sock.close()


if __name__ == "__main__":
    main()
