#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_mon.py -- LoRa streaming monitor.

Connects to a lora_trx UDP server and displays decoded frames.

Usage:
    lora_mon.py                           # connects to 127.0.0.1:5555
    lora_mon.py --connect 127.0.0.1:5556
    lora_mon.py --connect [::1]:5555      # IPv6

Dependencies: cbor2
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
from typing import Any

import cbor2

from lora_common import (
    PAYLOAD_NAMES,
    ROUTE_NAMES,
    format_ascii,
    format_hex,
    sync_word_name,
)

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555


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


# ---- Frame formatting ----


def format_frame(msg: dict[str, Any]) -> str:
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
    sw_label = sync_word_name(sync_word)
    snr_db = phy.get("snr_db")
    rx_ch = msg.get("rx_channel")
    dc = " (downchirp)" if msg.get("is_downchirp") else ""

    header = (
        f"#{seq:<4d} [{ts_short}] {len(payload):>3d}B "
        f"SF{phy.get('sf', '?')} CR4/{4 + cr} {crc_str} {sw_label}"
    )
    if snr_db is not None:
        header += f" SNR={snr_db:.1f}dB"
    if rx_ch is not None:
        header += f" ch={rx_ch}"
    header += dc

    lines = [header]

    if sync_word == 0x12 and payload:
        mc = parse_meshcore_summary(payload)
        if mc:
            lines.append(f"     {mc}")

    hex_line = format_hex(payload, max_bytes=24)
    if hex_line:
        lines.append(f"     {hex_line}")

    if any(0x20 <= b < 0x7F for b in payload[:40]):
        lines.append(f"     {format_ascii(payload, max_bytes=40)}")

    return "\n".join(lines)


# ---- UDP receiver ----


def connect_udp(host: str, port: int) -> None:
    """Connect to a lora_trx UDP server and display frames."""
    # Resolve address family
    af = socket.AF_INET
    if ":" in host:
        af = socket.AF_INET6

    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    local_port = sock.getsockname()[1]

    sock.sendto(b"sub", (host, port))
    print(
        f"Connected to {host}:{port} (listening on :{local_port})",
        file=sys.stderr,
    )
    sys.stderr.flush()

    total = 0
    crc_ok = 0
    try:
        while True:
            data, _addr = sock.recvfrom(65536)
            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            total += 1
            if msg.get("crc_valid", False):
                crc_ok += 1

            print(format_frame(msg))
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if total > 0:
        pct = crc_ok / total * 100
        print(
            f"\n--- {total} frames, {crc_ok} CRC_OK ({pct:.0f}%)",
            file=sys.stderr,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa streaming monitor -- connects to lora_trx UDP server"
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=f"{DEFAULT_HOST}:{DEFAULT_PORT}",
        help=f"UDP server address (default: {DEFAULT_HOST}:{DEFAULT_PORT})",
    )
    args = parser.parse_args()

    # Parse host:port (handle IPv6 bracket notation)
    spec = args.connect
    if spec.startswith("["):
        # [host]:port
        bracket = spec.index("]")
        host = spec[1:bracket]
        port = int(spec[bracket + 2 :])
    else:
        colon = spec.rfind(":")
        if colon <= 0:
            parser.error("--connect requires HOST:PORT (e.g. 127.0.0.1:5555)")
        host = spec[:colon]
        port = int(spec[colon + 1 :])

    connect_udp(host, port)


if __name__ == "__main__":
    main()
