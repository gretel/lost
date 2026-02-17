#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_decode_meshtastic.py -- Meshtastic protocol decoder (stub).

Reads concatenated CBOR frames on stdin. Filters for frames with
sync_word=0x2B (Meshtastic) and prints raw info.

Meshtastic uses Protobuf-encoded payloads (meshtastic.mesh_pb2). Full
decoding requires the Meshtastic protobuf definitions, which are not
included here. This stub prints the raw payload hex for frames matching
the Meshtastic sync word.

To add full decoding:
    pip install meshtastic
    from meshtastic.mesh_pb2 import MeshPacket
    pkt = MeshPacket()
    pkt.ParseFromString(payload)

Note: for live monitoring, use lora_mon.py (reads UDP from lora_rx_soapy).
This script is for offline analysis of saved CBOR streams.

Usage:
    python3 scripts/lora_decode_meshtastic.py < captured.cbor
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

import cbor2

from cbor_stream import read_cbor_seq

MESHTASTIC_SYNC_WORD = 0x2B


def format_hex(data: bytes, sep: str = " ") -> str:
    return sep.join(f"{b:02X}" for b in data)


def format_ascii(data: bytes) -> str:
    return "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in data)


def print_text(msg: dict[str, Any]) -> None:
    """Print Meshtastic frame in human-readable text format."""
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    crc = "CRC_OK" if msg.get("crc_valid", False) else "CRC_FAIL"
    dc = "  (downchirp)" if msg.get("is_downchirp", False) else ""

    print(
        f"[{msg.get('ts', '')}] #{msg.get('seq', 0)}  "
        f"{len(payload)} bytes  "
        f"SF{phy.get('sf', '?')} BW{phy.get('bw', '?')} CR4/{4 + phy.get('cr', 0)}  "
        f"{crc}  Meshtastic{dc}"
    )
    print(f"  Hex:   {format_hex(payload[:80])}")
    if len(payload) > 80:
        print(f"         ... ({len(payload)} bytes total)")
    print(f"  ASCII: {format_ascii(payload[:80])}")
    print()
    print("  (Full Meshtastic decoding requires: pip install meshtastic)")
    print()


def print_json(msg: dict[str, Any]) -> None:
    """Print Meshtastic frame as a JSON line."""
    payload = msg.get("payload", b"")
    out = {
        "seq": msg.get("seq", 0),
        "ts": msg.get("ts", ""),
        "crc_valid": msg.get("crc_valid", False),
        "is_downchirp": msg.get("is_downchirp", False),
        "protocol": "meshtastic",
        "payload_hex": format_hex(payload, sep=""),
        "payload_len": len(payload),
    }
    print(json.dumps(out))


def main() -> None:
    parser = argparse.ArgumentParser(description="Meshtastic protocol decoder (stub)")
    parser.add_argument(
        "--json", action="store_true", help="output JSON lines instead of text"
    )
    args = parser.parse_args()

    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            phy = msg.get("phy", {})
            sync_word = phy.get("sync_word", 0)
            if sync_word != MESHTASTIC_SYNC_WORD:
                continue

            if args.json:
                print_json(msg)
            else:
                print_text(msg)

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass


if __name__ == "__main__":
    main()
