#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_mon.py -- LoRa streaming monitor.

Listens for CBOR-encoded LoRa frames on a UDP socket or reads concatenated
CBOR from stdin, and prints formatted text with running statistics.

Usage:
    # UDP mode (default):
    lora_rx_soapy --udp 127.0.0.1:5556 &
    python3 scripts/lora_mon.py
    python3 scripts/lora_mon.py --port 5556 --json
    python3 scripts/lora_mon.py --compact --stats

    # Pipe mode:
    lora_rx_soapy --cbor | python3 scripts/lora_mon.py --stdin

Dependencies: cbor2
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from dataclasses import dataclass, field
from typing import Any, TextIO

import cbor2

from cbor_stream import read_cbor_seq

# ---- Protocol constants ----

ROUTE_NAMES = ["T_FLOOD", "FLOOD", "DIRECT", "T_DIRECT"]
PAYLOAD_NAMES = [
    "REQ",
    "RESP",
    "TXT",
    "ACK",
    "ADVERT",
    "GRP_TXT",
    "GRP_DATA",
    "ANON_REQ",
    "PATH",
    "TRACE",
    "MULTI",
    "CTRL",
    "rsv12",
    "rsv13",
    "rsv14",
    "RAW_CUSTOM",
]
SYNC_NAMES = {0x12: "MeshCore", 0x2B: "Meshtastic", 0x34: "LoRaWAN"}

DEFAULT_PORT = 5556


# ---- MeshCore v1 one-line summary ----


def parse_meshcore_summary(data: bytes) -> str:
    """One-line MeshCore summary or empty string on failure."""
    if len(data) < 2:
        return ""
    hdr = data[0]
    route = hdr & 0x03
    ptype = (hdr >> 2) & 0x0F
    version = (hdr >> 6) & 0x03

    route_name = ROUTE_NAMES[route] if route < len(ROUTE_NAMES) else f"?{route}"
    ptype_name = PAYLOAD_NAMES[ptype] if ptype < len(PAYLOAD_NAMES) else f"?{ptype}"
    has_transport = route in (0, 3)

    off = 1
    transport_str = ""
    if has_transport:
        if off + 4 <= len(data):
            tc1 = struct.unpack_from("<H", data, off)[0]
            tc2 = struct.unpack_from("<H", data, off + 2)[0]
            transport_str = f" tc={tc1:04X}/{tc2:04X}"
            off += 4

    path_len = data[off] if off < len(data) else 0
    off += 1

    advert_info = ""
    if ptype == 4:  # ADVERT
        path_end = off + path_len
        if path_end + 100 < len(data):
            appdata = data[path_end + 100 :]
            if appdata:
                flags = appdata[0]
                name_off = 1
                if flags & 0x10:
                    name_off += 8
                if flags & 0x20:
                    name_off += 2
                if flags & 0x40:
                    name_off += 2
                if (flags & 0x80) and name_off < len(appdata):
                    name = appdata[name_off:].decode("utf-8", errors="replace")
                    advert_info = f' "{name}"'

    return f"v{version} {route_name}/{ptype_name}{transport_str} path={path_len}{advert_info}"


# ---- Formatting helpers ----


def format_hex(data: bytes, max_bytes: int = 24) -> str:
    preview = " ".join(f"{b:02X}" for b in data[:max_bytes])
    if len(data) > max_bytes:
        preview += f" ...({len(data)}B)"
    return preview


def format_ascii(data: bytes, max_bytes: int = 40) -> str:
    s = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in data[:max_bytes])
    if len(data) > max_bytes:
        s += "..."
    return s


# ---- Stats ----


@dataclass
class Stats:
    total: int = 0
    crc_ok: int = 0
    crc_fail: int = 0
    first_ts: str = ""
    last_ts: str = ""
    protocols: dict[str, int] = field(default_factory=dict)

    def update(self, msg: dict[str, Any]) -> None:
        self.total += 1
        if msg.get("crc_valid", False):
            self.crc_ok += 1
        else:
            self.crc_fail += 1
        ts = msg.get("ts", "")
        if not self.first_ts:
            self.first_ts = ts
        self.last_ts = ts
        sw = msg.get("phy", {}).get("sync_word", 0)
        proto = SYNC_NAMES.get(sw, f"0x{sw:02X}")
        self.protocols[proto] = self.protocols.get(proto, 0) + 1

    def summary(self) -> str:
        if self.total == 0:
            return "No frames received"
        pct = self.crc_ok / self.total * 100
        protos = ", ".join(f"{k}:{v}" for k, v in sorted(self.protocols.items()))
        return (
            f"frames={self.total} crc_ok={self.crc_ok} ({pct:.0f}%) "
            f"crc_fail={self.crc_fail} [{protos}]"
        )


# ---- Frame formatting ----


def format_frame(msg: dict[str, Any], *, compact: bool = False) -> str:
    """Format a single lora_frame message as human-readable text."""
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    crc_ok = msg.get("crc_valid", False)
    crc_str = "CRC_OK" if crc_ok else "CRC_FAIL"
    seq = msg.get("seq", 0)
    ts = msg.get("ts", "")
    ts_short = ts[11:23] if len(ts) > 11 else ts  # HH:MM:SS.mmm
    cr = phy.get("cr", 0)
    sync_word = phy.get("sync_word", 0)
    proto = SYNC_NAMES.get(sync_word, f"0x{sync_word:02X}")
    dc = " (downchirp)" if msg.get("is_downchirp") else ""

    header = (
        f"#{seq:<4d} [{ts_short}] {len(payload):>3d}B "
        f"SF{phy.get('sf', '?')} CR4/{4 + cr} {crc_str} {proto}{dc}"
    )

    lines = [header]

    if sync_word == 0x12 and payload:
        mc = parse_meshcore_summary(payload)
        if mc:
            lines.append(f"     {mc}")

    hex_line = format_hex(payload)
    if hex_line:
        lines.append(f"     {hex_line}")

    if any(0x20 <= b < 0x7F for b in payload[:40]):
        lines.append(f"     {format_ascii(payload)}")

    if compact:
        return lines[0]
    return "\n".join(lines)


def format_frame_json(msg: dict[str, Any]) -> str:
    """Format a single lora_frame message as a JSON line."""
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    out = {
        "seq": msg.get("seq", 0),
        "ts": msg.get("ts", ""),
        "payload_len": len(payload),
        "payload_hex": "".join(f"{b:02X}" for b in payload),
        "crc_valid": msg.get("crc_valid", False),
        "sf": phy.get("sf", 0),
        "bw": phy.get("bw", 0),
        "cr": phy.get("cr", 0),
        "sync_word": phy.get("sync_word", 0),
    }
    if msg.get("is_downchirp"):
        out["is_downchirp"] = True
    sw = phy.get("sync_word", 0)
    if sw == 0x12 and payload:
        mc = parse_meshcore_summary(payload)
        if mc:
            out["meshcore"] = mc
    return json.dumps(out)


# ---- UDP receiver ----


def receive_udp(
    port: int,
    out: TextIO,
    *,
    use_json: bool = False,
    compact: bool = False,
    stats_every: int = 0,
    bind_addr: str = "0.0.0.0",
) -> Stats:
    """Listen on UDP port, decode CBOR datagrams, write text to out."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_addr, port))
    print(f"Listening on UDP {bind_addr}:{port} ...", file=sys.stderr)
    sys.stderr.flush()

    stats = Stats()
    try:
        while True:
            data, _addr = sock.recvfrom(65536)
            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            stats.update(msg)

            if use_json:
                out.write(format_frame_json(msg))
            else:
                out.write(format_frame(msg, compact=compact))
            out.write("\n")
            out.flush()

            if stats_every > 0 and stats.total % stats_every == 0:
                print(f"--- {stats.summary()}", file=sys.stderr)
                sys.stderr.flush()

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    return stats


def receive_stdin(
    out: TextIO,
    *,
    use_json: bool = False,
    compact: bool = False,
    stats_every: int = 0,
) -> Stats:
    """Read CBOR Sequence (RFC 8742) from stdin, decode, write text to out."""
    stats = Stats()
    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            stats.update(msg)

            if use_json:
                out.write(format_frame_json(msg))
            else:
                out.write(format_frame(msg, compact=compact))
            out.write("\n")
            out.flush()

            if stats_every > 0 and stats.total % stats_every == 0:
                print(f"--- {stats.summary()}", file=sys.stderr)
                sys.stderr.flush()

    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass

    return stats


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa streaming monitor -- listens for CBOR frames on UDP or stdin"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"UDP port to listen on (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="bind address (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="output JSON lines instead of text",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="one line per frame (no hex/ascii/detail)",
    )
    parser.add_argument(
        "--stats-every",
        type=int,
        default=0,
        metavar="N",
        help="print stats summary to stderr every N frames (0=off)",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="print final stats summary to stderr on exit",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read CBOR Sequence (RFC 8742) from stdin instead of UDP",
    )
    args = parser.parse_args()

    if args.stdin:
        stats = receive_stdin(
            sys.stdout,
            use_json=args.json,
            compact=args.compact,
            stats_every=args.stats_every,
        )
    else:
        stats = receive_udp(
            args.port,
            sys.stdout,
            use_json=args.json,
            compact=args.compact,
            stats_every=args.stats_every,
            bind_addr=args.bind,
        )

    if args.stats and stats.total > 0:
        print(f"--- {stats.summary()}", file=sys.stderr)


if __name__ == "__main__":
    main()
