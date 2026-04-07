#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_decode_raw.py -- Raw LoRa frame decoder.

Reads concatenated CBOR frames on stdin and prints a human-readable
summary of each frame.

Note: for live monitoring, use lora_mon.py (reads UDP from lora_trx).
This script is for offline analysis of saved CBOR streams.

Usage:
    python3 scripts/lora_decode_raw.py < captured.cbor
    python3 scripts/lora_decode_raw.py --json < captured.cbor
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))

from cbor_stream import read_cbor_seq
from lora_common import format_ascii, format_hex, sync_word_name


def decode_frame(msg: dict[str, Any]) -> dict[str, Any]:
    """Extract and normalize fields from a CBOR lora_frame message."""
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    return {
        "seq": msg.get("seq", 0),
        "ts": msg.get("ts", ""),
        "payload": payload,
        "payload_len": msg.get("payload_len", len(payload)),
        "crc_valid": msg.get("crc_valid", False),
        "cr": phy.get("cr", 0),
        "sf": phy.get("sf", 0),
        "bw": phy.get("bw", 0),
        "sync_word": phy.get("sync_word", 0),
        "is_downchirp": msg.get("is_downchirp", False),
    }


def print_text(frame: dict[str, Any]) -> None:
    """Print a frame in human-readable text format."""
    payload = frame["payload"]
    crc = "CRC_OK" if frame["crc_valid"] else "CRC_FAIL"
    sw_name = sync_word_name(frame["sync_word"])
    dc = "  (downchirp)" if frame["is_downchirp"] else ""

    print(
        f"[{frame['ts']}] #{frame['seq']}  "
        f"{frame['payload_len']} bytes  "
        f"SF{frame['sf']} BW{frame['bw']} CR4/{4 + frame['cr']}  "
        f"{crc}  {sw_name}{dc}"
    )
    print(f"  Hex:   {format_hex(payload)}")
    print(f"  ASCII: {format_ascii(payload)}")
    print()


def print_json(frame: dict[str, Any]) -> None:
    """Print a frame as a JSON line (payload as hex)."""
    out = dict(frame)
    out["payload"] = format_hex(frame["payload"], sep="")
    print(json.dumps(out))


def main() -> None:
    parser = argparse.ArgumentParser(description="Raw LoRa frame decoder")
    parser.add_argument(
        "--json", action="store_true", help="output JSON lines instead of text"
    )
    args = parser.parse_args()

    printer = print_json if args.json else print_text

    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict):
                print(
                    f"Warning: skipping non-map CBOR item: {type(msg)}", file=sys.stderr
                )
                continue

            msg_type = msg.get("type", "")
            if msg_type != "lora_frame":
                print(f"Warning: skipping message type '{msg_type}'", file=sys.stderr)
                continue

            frame = decode_frame(msg)
            printer(frame)
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass


if __name__ == "__main__":
    main()
