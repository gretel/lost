#!/usr/bin/env python3
"""One-shot ACLR dwell: repeatedly TX ADVERTs via lora_trx for SA capture."""
import argparse
import secrets
import socket
import time

import cbor2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--duration", type=float, default=60.0,
                    help="Seconds of repeated TX")
    ap.add_argument("--gap-ms", type=int, default=100,
                    help="Idle gap between bursts (ms)")
    ap.add_argument("--payload-bytes", type=int, default=16)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)
    deadline = time.time() + args.duration
    seq = 0
    while time.time() < deadline:
        seq += 1
        payload = secrets.token_bytes(args.payload_bytes)
        msg = cbor2.dumps({
            "type": "lora_tx",
            "seq": seq,
            "payload": payload,
            "gap_ms": args.gap_ms,
        })
        sock.sendto(msg, target)
        time.sleep(args.gap_ms / 1000.0)
    print(f"sent {seq} bursts over {args.duration:.0f}s")


if __name__ == "__main__":
    main()
