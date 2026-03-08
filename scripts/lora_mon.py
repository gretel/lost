#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_mon.py -- LoRa streaming monitor.

Connects to a lora_trx UDP server and displays decoded frames.
Decrypts MeshCore TXT_MSG, ANON_REQ, and GRP_TXT (group messages)
using a local identity, auto-learned public keys, and channel PSKs.

Usage:
    lora_mon.py                           # connects to 127.0.0.1:5555
    lora_mon.py --connect 127.0.0.1:5556
    lora_mon.py --connect [::1]:5555      # IPv6
    lora_mon.py --identity scripts/data/meshcore/identity.bin
    lora_mon.py --channel secret:BASE64   # add a group channel

Dependencies: cbor2, pynacl, pycryptodome
"""

from __future__ import annotations

import argparse
import logging
import socket
import struct
import time
from pathlib import Path
from typing import Any

import cbor2

from lora_common import (
    PAYLOAD_NAMES,
    ROUTE_NAMES,
    config_region_scope,
    format_hexdump,
    load_config,
    resolve_udp_address,
    sanitize_text,
    setup_logging,
    sync_word_name,
)
from meshcore_crypto import (
    DEFAULT_CHANNELS_DIR,
    DEFAULT_IDENTITY_FILE,
    DEFAULT_KEYS_DIR,
    GroupChannel,
    extract_advert_name,
    extract_advert_pubkey,
    grp_txt_channel_hash,
    load_channels,
    load_identity,
    load_known_keys,
    save_channel,
    save_pubkey,
    try_decrypt_anon_req,
    try_decrypt_grp_txt,
    try_decrypt_txt_msg,
)

log = logging.getLogger("gr4.mon")

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555
KEEPALIVE_INTERVAL = 5.0
RECV_TIMEOUT = 10.0

# Gain advisor thresholds (dBFS / dB)
GAIN_PEAK_CLIP = -3.0  # peak above this → clipping risk
GAIN_NOISE_LOW = -65.0  # noise floor below this → gain too low
GAIN_NOISE_HIGH = -25.0  # noise floor above this → gain too high
GAIN_SWEET_LOW = -55.0  # ideal noise floor lower bound
GAIN_SWEET_HIGH = -40.0  # ideal noise floor upper bound


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
                    raw = appdata[name_off : name_off + 32]
                    name = sanitize_text(
                        raw.decode("utf-8", errors="replace").rstrip("\x00")
                    )
                    advert_info = f' "{name}"'

    return f"v{version} {route_name}/{ptype_name}{transport_str} path={path_len}{advert_info}"


# ---- Frame formatting ----


def format_frame(
    msg: dict[str, Any],
    *,
    our_prv: bytes | None = None,
    our_pub: bytes | None = None,
    known_keys: dict[str, bytes] | None = None,
    channels: list[GroupChannel] | None = None,
) -> str:
    """Format a single lora_frame message as human-readable text.

    Layout (each on its own line, continuation lines indented by formatter):

    1. Header: ``#seq sizeB CRC SF/CR sync [ch=N] [SNR=XdB]``
    2. Protocol + decrypted content (MeshCore only, if available)
    3. Hex dump (compact, omitted when decrypted content is shown)

    All string fields from the CBOR message are sanitized — the message
    arrives from lora_trx over UDP which is not fully trusted.
    """
    payload = msg.get("payload", b"")
    phy = msg.get("phy", {})
    crc_ok = msg.get("crc_valid", False)
    crc_str = "CRC_OK" if crc_ok else "CRC_FAIL"
    seq = msg.get("seq", 0)
    cr = phy.get("cr", 0)
    sync_word = phy.get("sync_word", 0)
    sw_label = sync_word_name(sync_word)
    snr_db = phy.get("snr_db")
    snr_db_td = phy.get("snr_db_td")
    noise_floor_db = phy.get("noise_floor_db")
    peak_db = phy.get("peak_db")
    rx_ch = msg.get("rx_channel")
    dc = " (downchirp)" if msg.get("is_downchirp") else ""

    # Line 1: identity + PHY essentials + signal quality
    header = f"#{seq} {len(payload)}B {crc_str} SF{phy.get('sf', '?')}/CR4:{4 + cr} {sw_label}"
    if rx_ch is not None:
        header += f" ch={rx_ch}"
    if snr_db is not None:
        header += f" SNR={snr_db:.1f}dB"
    if snr_db_td is not None:
        header += f" SNR_td={snr_db_td:.1f}dB"
    if noise_floor_db is not None:
        header += f" NF={noise_floor_db:.1f}dBFS"
    if peak_db is not None:
        header += f" peak={peak_db:.1f}dBFS"
    header += dc

    lines = [header]

    # Line 2: protocol summary + decrypted content
    mc_summary = ""
    if sync_word == 0x12 and payload:
        mc_summary = parse_meshcore_summary(payload)

    decrypt_line = None
    if crc_ok and sync_word == 0x12 and payload:
        decrypt_line = _try_decrypt(
            payload, our_prv, our_pub, known_keys or {}, channels or []
        )

    if mc_summary and decrypt_line:
        # Protocol routing + decrypted content on one line
        lines.append(f"  {mc_summary} | {decrypt_line}")
    elif mc_summary:
        lines.append(f"  {mc_summary}")
    elif decrypt_line:
        lines.append(f"  {decrypt_line}")

    # Hex dump: show when there's no decrypted content (raw/unknown frames)
    if payload and not decrypt_line:
        lines.append(format_hexdump(payload, max_bytes=48, indent="  "))

    return "\n".join(lines)


def _try_decrypt(
    payload: bytes,
    our_prv: bytes | None,
    our_pub: bytes | None,
    known_keys: dict[str, bytes],
    channels: list[GroupChannel],
) -> str | None:
    """Try decrypting a MeshCore payload. Returns display string or None."""
    # Per-peer decryption requires an identity
    if our_prv and our_pub:
        result = try_decrypt_txt_msg(payload, our_prv, our_pub, known_keys)
        if result:
            text, sender_pub = result
            return f"TXT_MSG from {sender_pub[:4].hex()}: {sanitize_text(text)}"

        result = try_decrypt_anon_req(payload, our_prv, our_pub)
        if result:
            text, sender_pub = result
            return f"ANON_REQ from {sender_pub[:4].hex()}: {sanitize_text(text)}"

    # Group decryption only needs channel PSKs (no identity required)
    grp = try_decrypt_grp_txt(payload, channels) if channels else None
    if grp:
        text, channel_name = grp
        return f"GRP_TXT #{sanitize_text(channel_name)}: {sanitize_text(text)}"

    # Show unknown channel hash for undecryptable GRP_TXT
    ch_hash = grp_txt_channel_hash(payload)
    if ch_hash is not None:
        return f"GRP_TXT [ch={ch_hash.hex()}] (unknown channel)"

    return None


# ---- Gain advisor ----


def gain_advice(phy: dict[str, Any]) -> str | None:
    """Return a one-line gain recommendation based on PHY metrics, or None."""
    peak = phy.get("peak_db")
    nf = phy.get("noise_floor_db")
    if peak is None and nf is None:
        return None

    # Clipping takes priority
    if peak is not None and peak > GAIN_PEAK_CLIP:
        return (
            f"GAIN HIGH: peak {peak:.1f} dBFS (>{GAIN_PEAK_CLIP:.0f}) — reduce RX gain"
        )

    # Noise floor too high (compression without clipping)
    if nf is not None and nf > GAIN_NOISE_HIGH:
        return f"GAIN HIGH: noise floor {nf:.1f} dBFS (>{GAIN_NOISE_HIGH:.0f}) — reduce RX gain"

    # Noise floor too low
    if nf is not None and nf < GAIN_NOISE_LOW:
        return f"GAIN LOW: noise floor {nf:.1f} dBFS (<{GAIN_NOISE_LOW:.0f}) — increase RX gain"

    return None


def format_config(msg: dict[str, Any]) -> str:
    """Format a config message as a two-line human-readable summary.

    All string fields from the CBOR message are sanitized (the message
    originates from lora_trx over UDP and is not fully trusted).
    """
    phy = msg.get("phy", {})
    server = msg.get("server", {})
    freq = phy.get("freq", 0)
    freq_mhz = freq / 1e6 if freq else 0
    sf = phy.get("sf", "?")
    bw = phy.get("bw", 0)
    bw_khz = bw / 1e3 if bw else 0
    cr = phy.get("cr", "?")
    sw = phy.get("sync_word", "?")
    sw_name = sync_word_name(sw) if isinstance(sw, int) else str(sw)
    rx_g = phy.get("rx_gain", "?")
    tx_g = phy.get("tx_gain", "?")
    device = sanitize_text(str(server.get("device", "?")))
    rate = server.get("sample_rate", 0)
    rate_ks = rate / 1e3 if rate else 0
    si = server.get("status_interval", 0)

    cr_label = f"CR4/{4 + cr}" if isinstance(cr, int) else f"CR4/{cr}"
    lines = [
        f"--- config: {freq_mhz:.3f} MHz SF{sf} BW {bw_khz:.1f}k {cr_label} {sw_name}",
        f"    gain: RX={rx_g} dB  TX={tx_g} dB  |  {device} @ {rate_ks:.0f} kS/s  status every {si}s",
    ]
    return "\n".join(lines)


def format_status(msg: dict[str, Any]) -> str:
    """Format a status heartbeat as a single-line human-readable summary."""
    ts = sanitize_text(str(msg.get("ts", "")))
    ts_short = ts[11:19] if len(ts) > 11 else ts
    phy = msg.get("phy", {})
    frames = msg.get("frames", {})
    total = frames.get("total", 0)
    ok = frames.get("crc_ok", 0)
    fail = frames.get("crc_fail", 0)
    pct = ok / total * 100 if total > 0 else 0
    rx_g = phy.get("rx_gain", "?")
    return f"[{ts_short}] {total} frames ({ok} OK, {fail} fail, {pct:.0f}%) RX gain={rx_g} dB"


# ---- UDP receiver ----


def connect_udp(
    host: str,
    port: int,
    identity_path: Path,
    keys_dir: Path,
    channels: list[GroupChannel],
) -> None:
    """Connect to a lora_trx UDP server and display frames."""
    # Load identity for decryption (optional)
    our_prv: bytes | None = None
    our_pub: bytes | None = None
    identity = load_identity(identity_path)
    if identity:
        our_prv, our_pub, _seed = identity
        log.info("identity: %s", our_pub.hex())
    else:
        log.info("no identity at %s (decryption disabled)", identity_path)

    # Load known keys
    known_keys = load_known_keys(keys_dir)
    if known_keys:
        log.info("loaded %d known key(s) from %s", len(known_keys), keys_dir)

    # Log channels
    if channels:
        names = ", ".join(f"#{sanitize_text(ch.name)}" for ch in channels)
        log.info("channels: %s", names)

    # Resolve address family
    af = socket.AF_INET
    if ":" in host:
        af = socket.AF_INET6

    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    sock.settimeout(RECV_TIMEOUT)
    local_port = sock.getsockname()[1]

    # Subscribe with sync_word filter (0x12 = MeshCore/Reticulum)
    sub_msg = cbor2.dumps({"type": "subscribe", "sync_word": [0x12]})
    sock.sendto(sub_msg, (host, port))
    log.info("connected to %s:%d (local port :%d)", host, port, local_port)

    total = 0
    crc_ok_count = 0
    last_keepalive = time.monotonic()
    waiting = False  # True after first timeout with no frames received

    try:
        while True:
            try:
                data, _addr = sock.recvfrom(65536)
                waiting = False
            except socket.timeout:
                # Re-subscribe on timeout (server may have restarted)
                sock.sendto(sub_msg, (host, port))
                last_keepalive = time.monotonic()
                if not waiting:
                    log.info("waiting for frames from %s:%d", host, port)
                    waiting = True
                continue

            # Periodic keepalive to maintain registration
            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(sub_msg, (host, port))
                last_keepalive = now

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict):
                continue

            msg_type = msg.get("type")

            # Handle config (sent once on subscribe)
            if msg_type == "config":
                log.debug("%s", format_config(msg))
                continue

            # Handle periodic status heartbeat
            if msg_type == "status":
                log.debug("%s", format_status(msg))
                continue

            if msg_type != "lora_frame":
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
                    label = f' "{sanitize_text(name)}"' if name else ""
                    log.info("key learned: %s..%s", pubkey[:4].hex(), label)
                    known_keys[pubkey.hex()] = pubkey

            log.info(
                "%s",
                format_frame(
                    msg,
                    our_prv=our_prv,
                    our_pub=our_pub,
                    known_keys=known_keys,
                    channels=channels,
                ),
            )

            # Gain advisor (per-frame)
            advice = gain_advice(msg.get("phy", {}))
            if advice:
                log.warning("%s", advice)

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if total > 0:
        pct = crc_ok_count / total * 100
        log.info("%d frames, %d CRC_OK (%.0f%%)", total, crc_ok_count, pct)


def _parse_channel_arg(spec: str) -> GroupChannel:
    """Parse a --channel NAME:BASE64 argument into a GroupChannel."""
    import base64 as b64

    colon = spec.find(":")
    if colon <= 0:
        raise ValueError(f"invalid channel spec '{spec}' (expected NAME:BASE64_PSK)")
    name = spec[:colon]
    psk_b64 = spec[colon + 1 :]
    try:
        psk_raw = b64.b64decode(psk_b64)
    except Exception as exc:
        raise ValueError(f"invalid base64 in channel '{name}': {exc}") from exc
    if len(psk_raw) not in (16, 32):
        raise ValueError(
            f"channel '{name}' PSK must be 16 or 32 bytes, got {len(psk_raw)}"
        )
    return GroupChannel(name, psk_raw)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa streaming monitor -- connects to lora_trx UDP server"
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=None,
        help="UDP server address (default from config.toml or 127.0.0.1:5555)",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to config.toml (auto-detected if omitted)",
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
    parser.add_argument(
        "--channels-dir",
        type=Path,
        default=DEFAULT_CHANNELS_DIR,
        help=f"Channel store directory (default: {DEFAULT_CHANNELS_DIR})",
    )
    parser.add_argument(
        "--channel",
        action="append",
        metavar="NAME:PSK_B64",
        help="Add a group channel (e.g. 'secret:BASE64'). May be repeated.",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output (also: NO_COLOR env var)",
    )
    args = parser.parse_args()

    # Load shared config (explicit path or auto-detect)
    cfg = load_config(args.config)
    setup_logging("gr4.mon", cfg, no_color=args.no_color)

    # Log region scope if configured
    region = config_region_scope(cfg)
    if region:
        log.info("region: %s", sanitize_text(str(region)))

    # Resolve --connect default from config
    try:
        host, port = resolve_udp_address(args.connect, cfg)
    except ValueError as exc:
        parser.error(str(exc))

    # Load channels from directory (seeds from repo on first run)
    channels = load_channels(args.channels_dir)

    # Add CLI channels (also persisted to channels dir)
    if args.channel:
        known_names = {ch.name for ch in channels}
        for ch_spec in args.channel:
            try:
                ch = _parse_channel_arg(ch_spec)
            except ValueError as exc:
                parser.error(str(exc))
            if ch.name not in known_names:
                channels.append(ch)
                known_names.add(ch.name)
                # Recover raw PSK and persist
                raw = ch.secret.rstrip(b"\x00")
                save_channel(args.channels_dir, ch.name, raw)

    connect_udp(host, port, args.identity, args.keys_dir, channels)


if __name__ == "__main__":
    main()
