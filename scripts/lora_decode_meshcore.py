#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_decode_meshcore.py -- MeshCore v1 protocol decoder.

Reads concatenated CBOR frames on stdin, parses the MeshCore v1 packet
framing, and prints decoded information.

Only processes frames where protocol == "meshcore_or_reticulum".
Frames with other protocols are passed through with minimal annotation.

Note: for live monitoring, use lora_mon.py (reads UDP from lora_rx_soapy).
This script is for offline analysis of saved CBOR streams.

Usage:
    python3 scripts/lora_decode_meshcore.py < captured.cbor
    python3 scripts/lora_decode_meshcore.py --json < captured.cbor
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from dataclasses import dataclass
from typing import Any

from cbor_stream import read_cbor_seq
from lora_common import PAYLOAD_NAMES, ROUTE_NAMES, format_hex

NODE_TYPE_NAMES = {
    0x01: "chat",
    0x02: "repeater",
    0x03: "room",
    0x04: "sensor",
}


# ---- MeshCore packet parsing ----


@dataclass
class MeshCorePacket:
    """Parsed MeshCore v1 packet."""

    route_type: int = 0
    payload_type: int = 0
    version: int = 0
    has_transport: bool = False
    transport_code1: int = 0
    transport_code2: int = 0
    path_length: int = 0
    path: bytes = b""
    app_payload: bytes = b""
    # ADVERT-specific
    advert_name: str = ""
    advert_flags: int = 0
    advert_pubkey: bytes = b""
    advert_timestamp: int = 0

    @property
    def route_name(self) -> str:
        return (
            ROUTE_NAMES[self.route_type]
            if self.route_type < len(ROUTE_NAMES)
            else f"?{self.route_type}"
        )

    @property
    def payload_name(self) -> str:
        return (
            PAYLOAD_NAMES[self.payload_type]
            if self.payload_type < len(PAYLOAD_NAMES)
            else f"?{self.payload_type}"
        )


def parse_meshcore(data: bytes) -> MeshCorePacket | None:
    """Parse MeshCore v1 packet framing from raw LoRa payload bytes."""
    if len(data) < 1:
        return None

    pkt = MeshCorePacket()
    hdr = data[0]
    pkt.route_type = hdr & 0x03
    pkt.payload_type = (hdr >> 2) & 0x0F
    pkt.version = (hdr >> 6) & 0x03
    pkt.has_transport = pkt.route_type in (0, 3)  # T_FLOOD or T_DIRECT

    off = 1

    # Transport codes (4 bytes, little-endian uint16 pairs)
    if pkt.has_transport:
        if off + 4 > len(data):
            return pkt
        pkt.transport_code1 = struct.unpack_from("<H", data, off)[0]
        pkt.transport_code2 = struct.unpack_from("<H", data, off + 2)[0]
        off += 4

    # Path
    if off >= len(data):
        return pkt
    pkt.path_length = data[off]
    off += 1
    if pkt.path_length > 64 or off + pkt.path_length > len(data):
        pkt.app_payload = data[off:]
        return pkt
    pkt.path = data[off : off + pkt.path_length]
    off += pkt.path_length

    pkt.app_payload = data[off:]

    # ADVERT parsing
    if pkt.payload_type == 4:  # ADVERT
        _parse_advert(pkt)

    return pkt


def _parse_advert(pkt: MeshCorePacket) -> None:
    """Parse ADVERT payload: [pubkey(32)][timestamp(4)][signature(64)][appdata]."""
    payload = pkt.app_payload
    min_advert = 32 + 4 + 64  # pubkey + timestamp + signature
    if len(payload) <= min_advert:
        return

    pkt.advert_pubkey = payload[:32]
    pkt.advert_timestamp = struct.unpack_from("<I", payload, 32)[0]
    appdata = payload[min_advert:]
    if not appdata:
        return

    pkt.advert_flags = appdata[0]
    off = 1
    if pkt.advert_flags & 0x10:
        off += 8  # lat(4) + lon(4)
    if pkt.advert_flags & 0x20:
        off += 2  # feature 1
    if pkt.advert_flags & 0x40:
        off += 2  # feature 2

    if (pkt.advert_flags & 0x80) and off < len(appdata):
        pkt.advert_name = appdata[off:].decode("utf-8", errors="replace")


def format_path(path: bytes) -> str:
    """Format path as a chain of 1-byte node hashes."""
    if not path:
        return "(direct)"
    return " > ".join(f"{b:02X}" for b in path)


# ---- Output formatters ----


def print_text(msg: dict[str, Any], pkt: MeshCorePacket) -> None:
    """Print MeshCore packet in human-readable text format."""
    phy = msg.get("phy", {})
    crc = "CRC_OK" if msg.get("crc_valid", False) else "CRC_FAIL"
    ts = msg.get("ts", "")
    seq = msg.get("seq", 0)
    dc = "  (downchirp)" if msg.get("is_downchirp", False) else ""

    print(
        f"[{ts}] #{seq}  "
        f"{msg.get('payload_len', 0)} bytes  "
        f"SF{phy.get('sf', '?')} CR4/{4 + phy.get('cr', 0)}  "
        f"{crc}{dc}"
    )

    transport = ""
    if pkt.has_transport:
        transport = f"  tc1={pkt.transport_code1:04X} tc2={pkt.transport_code2:04X}"

    print(
        f"  MeshCore v{pkt.version}: {pkt.route_name} / {pkt.payload_name}{transport}"
    )
    print(f"  Path: [{pkt.path_length}] {format_path(pkt.path)}")

    if pkt.payload_type == 4 and pkt.advert_name:  # ADVERT
        node_type = NODE_TYPE_NAMES.get(pkt.advert_flags & 0x0F, "unknown")
        print(f'  ADVERT: "{pkt.advert_name}" ({node_type})')
        print(f"  Pubkey: {format_hex(pkt.advert_pubkey[:8])}...")
        if pkt.advert_timestamp:
            print(f"  Timestamp: {pkt.advert_timestamp}")
    elif pkt.app_payload:
        print(f"  Payload: {format_hex(pkt.app_payload[:64])}")
        if len(pkt.app_payload) > 64:
            print(f"           ... ({len(pkt.app_payload)} bytes total)")

    print()


def print_json(msg: dict[str, Any], pkt: MeshCorePacket) -> None:
    """Print MeshCore packet as a JSON line."""
    out: dict[str, Any] = {
        "seq": msg.get("seq", 0),
        "ts": msg.get("ts", ""),
        "crc_valid": msg.get("crc_valid", False),
        "is_downchirp": msg.get("is_downchirp", False),
        "meshcore": {
            "version": pkt.version,
            "route": pkt.route_name,
            "payload_type": pkt.payload_name,
            "path_length": pkt.path_length,
            "path": format_hex(pkt.path, sep=""),
            "app_payload_hex": format_hex(pkt.app_payload, sep=""),
            "app_payload_len": len(pkt.app_payload),
        },
    }
    if pkt.has_transport:
        out["meshcore"]["transport_code1"] = pkt.transport_code1
        out["meshcore"]["transport_code2"] = pkt.transport_code2
    if pkt.payload_type == 4 and pkt.advert_name:
        out["meshcore"]["advert_name"] = pkt.advert_name
        out["meshcore"]["advert_flags"] = pkt.advert_flags
        out["meshcore"]["advert_pubkey"] = format_hex(pkt.advert_pubkey, sep="")
        out["meshcore"]["advert_timestamp"] = pkt.advert_timestamp

    print(json.dumps(out))


def print_passthrough_text(msg: dict[str, Any]) -> None:
    """Print non-MeshCore frame with minimal annotation."""
    payload = msg.get("payload", b"")
    crc = "CRC_OK" if msg.get("crc_valid", False) else "CRC_FAIL"
    protocol = msg.get("protocol", "unknown")
    print(
        f"[{msg.get('ts', '')}] #{msg.get('seq', 0)}  "
        f"{len(payload)} bytes  {crc}  [{protocol}]"
    )
    print(f"  Hex: {format_hex(payload[:64])}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="MeshCore v1 protocol decoder")
    parser.add_argument(
        "--json", action="store_true", help="output JSON lines instead of text"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="also show non-MeshCore frames (passthrough)",
    )
    args = parser.parse_args()

    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            protocol = msg.get("protocol", "")
            if protocol != "meshcore_or_reticulum":
                if args.all:
                    print_passthrough_text(msg)
                continue

            payload = msg.get("payload", b"")
            pkt = parse_meshcore(payload)
            if pkt is None:
                continue

            if args.json:
                print_json(msg, pkt)
            else:
                print_text(msg, pkt)

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass


if __name__ == "__main__":
    main()
