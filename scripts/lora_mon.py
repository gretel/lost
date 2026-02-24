#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_mon.py -- LoRa streaming monitor.

Connects to a lora_trx UDP server and displays decoded frames.
Optionally decrypts MeshCore TXT_MSG and ANON_REQ using a local
identity and auto-learned public keys from ADVERT beacons.

Usage:
    lora_mon.py                           # connects to 127.0.0.1:5555
    lora_mon.py --connect 127.0.0.1:5556
    lora_mon.py --connect [::1]:5555      # IPv6
    lora_mon.py --identity ~/.config/gr4-lora/identity.bin

Dependencies: cbor2, pynacl, pycryptodome
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any

import cbor2

from lora_common import (
    PAYLOAD_NAMES,
    ROUTE_NAMES,
    format_ascii,
    format_hex,
    sync_word_name,
)
from meshcore_crypto import (
    DEFAULT_IDENTITY_FILE,
    DEFAULT_KEYS_DIR,
    extract_advert_name,
    extract_advert_pubkey,
    load_identity,
    load_known_keys,
    save_pubkey,
    try_decrypt_anon_req,
    try_decrypt_txt_msg,
)

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555
KEEPALIVE_INTERVAL = 5.0
RECV_TIMEOUT = 10.0


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


def format_frame(
    msg: dict[str, Any],
    *,
    our_prv: bytes | None = None,
    our_pub: bytes | None = None,
    known_keys: dict[str, bytes] | None = None,
) -> str:
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
    noise_floor_db = phy.get("noise_floor_db")
    rx_ch = msg.get("rx_channel")
    dc = " (downchirp)" if msg.get("is_downchirp") else ""

    header = (
        f"#{seq:<4d} [{ts_short}] {len(payload):>3d}B "
        f"SF{phy.get('sf', '?')} CR4/{4 + cr} {crc_str} {sw_label}"
    )
    if snr_db is not None:
        header += f" SNR={snr_db:.1f}dB"
    if noise_floor_db is not None:
        header += f" NF={noise_floor_db:.1f}dBFS"
    if rx_ch is not None:
        header += f" ch={rx_ch}"
    header += dc

    lines = [header]

    if sync_word == 0x12 and payload:
        mc = parse_meshcore_summary(payload)
        if mc:
            lines.append(f"     {mc}")

    # Attempt decryption for CRC_OK MeshCore frames
    decrypt_line = None
    if crc_ok and sync_word == 0x12 and payload and our_prv and our_pub:
        decrypt_line = _try_decrypt(payload, our_prv, our_pub, known_keys or {})

    if decrypt_line:
        lines.append(f"     >> {decrypt_line}")

    hex_line = format_hex(payload, max_bytes=24)
    if hex_line:
        lines.append(f"     {hex_line}")

    if any(0x20 <= b < 0x7F for b in payload[:40]):
        lines.append(f"     {format_ascii(payload, max_bytes=40)}")

    return "\n".join(lines)


def _try_decrypt(
    payload: bytes,
    our_prv: bytes,
    our_pub: bytes,
    known_keys: dict[str, bytes],
) -> str | None:
    """Try decrypting a MeshCore payload. Returns display string or None."""
    result = try_decrypt_txt_msg(payload, our_prv, our_pub, known_keys)
    if result:
        text, sender_pub = result
        return f"TXT_MSG from {sender_pub[:4].hex()}: {text}"

    result = try_decrypt_anon_req(payload, our_prv, our_pub)
    if result:
        text, sender_pub = result
        return f"ANON_REQ from {sender_pub[:4].hex()}: {text}"

    return None


# ---- UDP receiver ----


def connect_udp(
    host: str,
    port: int,
    identity_path: Path,
    keys_dir: Path,
) -> None:
    """Connect to a lora_trx UDP server and display frames."""
    # Load identity for decryption (optional)
    our_prv: bytes | None = None
    our_pub: bytes | None = None
    identity = load_identity(identity_path)
    if identity:
        our_prv, our_pub, _seed = identity
        sys.stderr.write(f"Identity: {our_pub.hex()}\n")
    else:
        sys.stderr.write(f"No identity at {identity_path} (decryption disabled)\n")

    # Load known keys
    known_keys = load_known_keys(keys_dir)
    if known_keys:
        sys.stderr.write(f"Loaded {len(known_keys)} known key(s) from {keys_dir}\n")

    # Resolve address family
    af = socket.AF_INET
    if ":" in host:
        af = socket.AF_INET6

    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    sock.settimeout(RECV_TIMEOUT)
    local_port = sock.getsockname()[1]

    sock.sendto(b"sub", (host, port))
    sys.stderr.write(f"Connected to {host}:{port} (local port :{local_port})\n")
    sys.stderr.flush()

    total = 0
    crc_ok_count = 0
    last_keepalive = time.monotonic()

    try:
        while True:
            try:
                data, _addr = sock.recvfrom(65536)
            except socket.timeout:
                # Re-subscribe on timeout (server may have restarted)
                sock.sendto(b"sub", (host, port))
                last_keepalive = time.monotonic()
                continue

            # Periodic keepalive to maintain registration
            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(b"sub", (host, port))
                last_keepalive = now

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            total += 1
            payload = msg.get("payload", b"")
            crc_ok = msg.get("crc_valid", False)
            if crc_ok:
                crc_ok_count += 1

            # Auto-learn keys from ADVERTs
            if crc_ok and payload:
                pubkey = extract_advert_pubkey(payload)
                if pubkey and save_pubkey(keys_dir, pubkey):
                    name = extract_advert_name(payload) or ""
                    label = f' "{name}"' if name else ""
                    sys.stderr.write(f"  Key learned: {pubkey[:4].hex()}..{label}\n")
                    known_keys[pubkey.hex()] = pubkey

            print(
                format_frame(
                    msg,
                    our_prv=our_prv,
                    our_pub=our_pub,
                    known_keys=known_keys,
                )
            )
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if total > 0:
        pct = crc_ok_count / total * 100
        sys.stderr.write(f"\n--- {total} frames, {crc_ok_count} CRC_OK ({pct:.0f}%)\n")


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
    parser.add_argument(
        "--identity",
        type=Path,
        default=DEFAULT_IDENTITY_FILE,
        help=f"Identity file for decryption (default: {DEFAULT_IDENTITY_FILE})",
    )
    parser.add_argument(
        "--keys-dir",
        type=Path,
        default=DEFAULT_KEYS_DIR,
        help=f"Key store directory (default: {DEFAULT_KEYS_DIR})",
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

    connect_udp(host, port, args.identity, args.keys_dir)


if __name__ == "__main__":
    main()
