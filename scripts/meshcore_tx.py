#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
meshcore_tx.py -- MeshCore protocol TX message builder.

Constructs MeshCore packets (ADVERT, TXT_MSG, ANON_REQ) and sends them
to lora_trx via CBOR over UDP. Manages persistent Ed25519 identity keypairs.

Usage:
  # Send ADVERT beacon via UDP to lora_trx:
  meshcore_tx.py advert --name "MyNode" --udp 127.0.0.1:5555

  # Send encrypted message to a known contact (DIRECT routing):
  meshcore_tx.py send --dest <64hex_pubkey> --udp localhost:5555 "Hello"

  # Send anonymous encrypted request (includes sender pubkey):
  meshcore_tx.py anon-req --dest <64hex_pubkey> --udp localhost:5555 "Hello"

  # Print contact QR code (scannable by MeshCore companion app):
  meshcore_tx.py advert --name "MyNode" --qr

  # Dry run (print packet hex to stderr, CBOR to stdout):
  meshcore_tx.py advert --name "MyNode" --dry-run

Key management:
  Identity is stored in scripts/data/meshcore/identity.bin (64 bytes:
  32-byte seed + 32-byte public key). Generated on first run if
  missing. Use --identity <path> to override.

Protocol reference:
  MeshCore v1 packet format. See docs/lora-protocols.md and
  https://github.com/meshcore-dev/MeshCore

Crypto:
  - Ed25519 signing (pynacl / libsodium)
  - X25519 ECDH key exchange (Ed25519 keys converted to Curve25519)
  - AES-128-ECB + 2-byte HMAC-SHA256 MAC (pycryptodome)
"""

from __future__ import annotations

import argparse
import logging
import struct
import sys
import time
from pathlib import Path

import socket

import cbor2
import segno
from nacl.signing import SigningKey

from lora_common import (
    config_udp_host,
    config_udp_port,
    load_config,
    parse_host_port,
    setup_logging,
)
from meshcore_crypto import (
    # Crypto primitives
    PUB_KEY_SIZE,
    PRV_KEY_SIZE,
    SIGNATURE_SIZE,
    CIPHER_KEY_SIZE,
    CIPHER_BLOCK_SIZE,
    CIPHER_MAC_SIZE,
    PATH_HASH_SIZE,
    DEFAULT_IDENTITY_FILE,
    load_or_create_identity,
    meshcore_expanded_key,
    meshcore_shared_secret,
    meshcore_encrypt_then_mac,
    meshcore_mac_then_decrypt,
    # Protocol constants (canonical source)
    ROUTE_FLOOD,
    ROUTE_T_FLOOD,
    ROUTE_DIRECT,
    ROUTE_T_DIRECT,
    PAYLOAD_REQ,
    PAYLOAD_RESP,
    PAYLOAD_TXT,
    PAYLOAD_ACK,
    PAYLOAD_ADVERT,
    PAYLOAD_GRP_TXT,
    PAYLOAD_GRP_DATA,
    PAYLOAD_ANON_REQ,
    PAYLOAD_PATH,
    PAYLOAD_TRACE,
    PAYLOAD_MULTI,
    PAYLOAD_CTRL,
    PAYLOAD_RAW_CUSTOM,
    ADVERT_NODE_CHAT,
    ADVERT_NODE_REPEATER,
    ADVERT_NODE_ROOM,
    ADVERT_NODE_SENSOR,
    ADVERT_HAS_LOCATION,
    ADVERT_HAS_FEATURE1,
    ADVERT_HAS_FEATURE2,
    ADVERT_HAS_NAME,
)

log = logging.getLogger("gr4.tx")

# ---- MeshCore contact QR code ----

# URI builders live in lora_common.py (shared module). Re-exported here
# for backward compatibility.
from lora_common import build_bizcard_uri, build_contact_uri  # noqa: F401


def print_qr_code(uri: str) -> None:
    """Print a QR code to the terminal via segno."""
    qr = segno.make(uri)
    # segno terminal output uses Unicode block chars, compact=True for half-height
    qr.terminal(compact=True)


# ---- MeshCore packet construction ----


def make_header(route_type: int, payload_type: int, version: int = 0) -> int:
    """Build MeshCore header byte."""
    return (version << 6) | (payload_type << 2) | route_type


def build_wire_packet(
    header: int,
    payload: bytes,
    *,
    path: bytes = b"",
    transport_codes: tuple[int, int] | None = None,
) -> bytes:
    """Build a complete MeshCore wire-format packet.

    Wire format: [header][opt transport(4)][path_len(1)][path(N)][payload(M)]
    """
    parts = [bytes([header])]
    if transport_codes is not None:
        parts.append(struct.pack("<HH", transport_codes[0], transport_codes[1]))
    parts.append(bytes([len(path)]))
    parts.append(path)
    parts.append(payload)
    return b"".join(parts)


def build_advert(
    seed: bytes,
    pub_key: bytes,
    *,
    node_type: int = ADVERT_NODE_CHAT,
    name: str = "",
    timestamp: int | None = None,
    lat: float | None = None,
    lon: float | None = None,
    route_type: int = ROUTE_FLOOD,
    transport_codes: tuple[int, int] | None = None,
) -> bytes:
    """Build a MeshCore ADVERT packet payload.

    Payload: [pubkey(32)][timestamp(4)][signature(64)][app_data(N)]
    App data: [flags(1)][opt lat(4)+lon(4)][opt name(UTF-8)]

    seed: 32-byte Ed25519 seed (used for signing via libsodium).
    """
    if timestamp is None:
        timestamp = int(time.time())

    # Build app_data
    flags = node_type & 0x0F
    app_parts = []

    if lat is not None and lon is not None:
        flags |= ADVERT_HAS_LOCATION
    if name:
        flags |= ADVERT_HAS_NAME

    app_parts.append(bytes([flags]))

    if lat is not None and lon is not None:
        app_parts.append(struct.pack("<ii", int(lat * 1e6), int(lon * 1e6)))

    if name:
        app_parts.append(name.encode("utf-8"))

    app_data = b"".join(app_parts)

    # Build message to sign: pubkey + timestamp + app_data
    ts_bytes = struct.pack("<I", timestamp)
    message = pub_key + ts_bytes + app_data

    # Sign with Ed25519 using the seed
    sk = SigningKey(seed)
    signed = sk.sign(message)
    signature = signed.signature  # 64 bytes

    # Build payload
    payload = pub_key + ts_bytes + signature + app_data

    # Build wire packet (ADVERT defaults to FLOOD — it's a broadcast)
    header = make_header(route_type, PAYLOAD_ADVERT)
    return build_wire_packet(header, payload, transport_codes=transport_codes)


def build_anon_req(
    expanded_prv: bytes,
    pub_key: bytes,
    dest_pub: bytes,
    data: bytes,
    *,
    timestamp: int | None = None,
    route_type: int = ROUTE_DIRECT,
) -> bytes:
    """Build a MeshCore ANON_REQ packet.

    ANON_REQ includes the sender's public key in the packet, allowing the
    recipient to derive the shared secret without prior contact.

    Payload: [dest_hash(1)][sender_pub(32)][MAC(2)][encrypted_data(N*16)]
    Encrypted data: [timestamp(4)][message_bytes]

    expanded_prv: 64-byte MeshCore expanded private key (for ECDH).
    """
    if timestamp is None:
        timestamp = int(time.time())

    # Compute shared secret
    secret = meshcore_shared_secret(expanded_prv, dest_pub)
    assert secret is not None, "invalid key material"

    # Build plaintext: timestamp + data
    ts_bytes = struct.pack("<I", timestamp)
    plaintext = ts_bytes + data

    # Encrypt then MAC
    encrypted = meshcore_encrypt_then_mac(secret, plaintext)

    # Build payload: dest_hash + sender_pub + encrypted
    dest_hash = dest_pub[:PATH_HASH_SIZE]
    payload = dest_hash + pub_key + encrypted

    # Build wire packet (ANON_REQ defaults to DIRECT)
    header = make_header(route_type, PAYLOAD_ANON_REQ)
    return build_wire_packet(header, payload)


def build_txt_msg(
    expanded_prv: bytes,
    pub_key: bytes,
    dest_pub: bytes,
    text: str | bytes,
    *,
    timestamp: int | None = None,
    attempt: int = 0,
    txt_type: int = 0,
    route_type: int = ROUTE_DIRECT,
    transport_codes: tuple[int, int] | None = None,
) -> bytes:
    """Build a MeshCore TXT_MSG packet (encrypted text to a known contact).

    Uses the same wire format as REQ/RESPONSE (createDatagram):
    Payload: [dest_hash(1)][src_hash(1)][MAC(2)][encrypted_data(N*16)]

    Encrypted plaintext: [timestamp(4)][txt_type_attempt(1)][message_text(N)]
    txt_type_attempt: upper 6 bits = txt_type, lower 2 bits = attempt (0-3)

    Both parties must know each other's public keys (shared secret via ECDH).

    expanded_prv: 64-byte MeshCore expanded private key (for ECDH).
    pub_key: 32-byte sender's Ed25519 public key.
    dest_pub: 32-byte recipient's Ed25519 public key.
    text: Message text (UTF-8 str) or raw payload bytes (e.g. for binary CLI req).
    attempt: Retry attempt number (0 for first send).
    txt_type: Message type (0=plain, 1=CLI, 2=signed). Default 0.
    """
    if timestamp is None:
        timestamp = int(time.time())

    # Compute shared secret
    secret = meshcore_shared_secret(expanded_prv, dest_pub)
    assert secret is not None, "invalid key material"

    # Build plaintext: timestamp(4) + txt_type_attempt(1) + text
    # txt_type_attempt: upper 6 bits = txt_type, lower 2 bits = attempt
    ts_bytes = struct.pack("<I", timestamp)
    text_bytes = text if isinstance(text, bytes) else text.encode("utf-8")
    plaintext = ts_bytes + bytes([(txt_type << 2) | (attempt & 0x03)]) + text_bytes

    # Encrypt then MAC
    encrypted = meshcore_encrypt_then_mac(secret, plaintext)

    # Build payload: dest_hash(1) + src_hash(1) + encrypted
    dest_hash = dest_pub[:PATH_HASH_SIZE]
    src_hash = pub_key[:PATH_HASH_SIZE]
    payload = dest_hash + src_hash + encrypted

    # Build wire packet (TXT_MSG defaults to DIRECT)
    header = make_header(route_type, PAYLOAD_TXT)
    return build_wire_packet(header, payload, transport_codes=transport_codes)


def make_cbor_tx_request(packet: bytes, **phy_overrides) -> bytes:
    """Wrap a MeshCore wire packet in a CBOR TX request for lora_trx."""
    msg = {
        "type": "lora_tx",
        "payload": packet,
    }
    msg.update(phy_overrides)
    return cbor2.dumps(msg)


# ---- Route type helpers ----

ROUTE_NAMES = {
    "flood": ROUTE_FLOOD,
    "direct": ROUTE_DIRECT,
    "tc-flood": ROUTE_T_FLOOD,
    "tc-direct": ROUTE_T_DIRECT,
}


def _parse_dest_pubkey(hex_str: str) -> bytes:
    """Parse and validate a destination public key from hex string."""
    try:
        dest_pub = bytes.fromhex(hex_str)
    except ValueError:
        log.error("invalid hex in destination key: %r", hex_str)
        sys.exit(1)
    if len(dest_pub) != PUB_KEY_SIZE:
        log.error(
            "destination key must be %d hex chars (%d bytes), got %d chars",
            PUB_KEY_SIZE * 2,
            PUB_KEY_SIZE,
            len(hex_str),
        )
        sys.exit(1)
    return dest_pub


# ---- CLI ----


def cmd_advert(args, expanded_prv: bytes, pub_key: bytes, seed: bytes) -> bytes:
    """Build and emit an ADVERT packet."""
    return build_advert(
        seed,
        pub_key,
        node_type=args.node_type,
        name=args.name or "",
        lat=args.lat,
        lon=args.lon,
    )


def cmd_send(args, expanded_prv: bytes, pub_key: bytes, seed: bytes) -> bytes:
    """Build and emit a TXT_MSG packet."""
    dest_pub = _parse_dest_pubkey(args.dest)

    message = " ".join(args.message)
    if not message:
        log.error("message cannot be empty")
        sys.exit(1)

    route = ROUTE_NAMES[args.route]
    return build_txt_msg(expanded_prv, pub_key, dest_pub, message, route_type=route)


def cmd_anon_req(args, expanded_prv: bytes, pub_key: bytes, seed: bytes) -> bytes:
    """Build and emit an ANON_REQ packet."""
    dest_pub = _parse_dest_pubkey(args.dest)

    message = " ".join(args.message).encode("utf-8")
    if not message:
        log.error("message cannot be empty")
        sys.exit(1)

    route = ROUTE_NAMES[args.route]
    return build_anon_req(expanded_prv, pub_key, dest_pub, message, route_type=route)


_EPILOG = """\
examples:
  # Broadcast ADVERT beacon via UDP to lora_trx:
  %(prog)s advert --name "MyNode" --udp 127.0.0.1:5555

  # Print contact QR code (scannable by MeshCore companion):
  %(prog)s advert --name "MyNode" --qr

  # Send encrypted message to a known contact (direct):
  %(prog)s send --dest <64hex> --udp localhost:5555 "Hello from gr4-lora"

  # Anonymous encrypted request (includes sender pubkey):
  %(prog)s anon-req --dest <64hex> --udp localhost:5555 "Hello"

  # Dry run (CBOR to stdout, no transmission):
  %(prog)s advert --name "MyNode" --dry-run

  # Show identity public key:
  %(prog)s --show-key
"""


def main():
    # Shared arguments available to all subcommands
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument(
        "--identity",
        type=Path,
        default=DEFAULT_IDENTITY_FILE,
        help=f"Identity file path (default: {DEFAULT_IDENTITY_FILE})",
    )
    shared.add_argument(
        "--dry-run",
        action="store_true",
        help="Print packet hex to stderr, still output CBOR to stdout",
    )
    shared.add_argument(
        "--freq", type=int, default=None, help="Override TX frequency in Hz"
    )
    shared.add_argument(
        "--sf", type=int, default=None, help="Override spreading factor (7-12)"
    )
    shared.add_argument("--bw", type=int, default=None, help="Override bandwidth in Hz")
    shared.add_argument(
        "--udp",
        type=str,
        default=None,
        metavar="HOST:PORT",
        help="Send CBOR TX request via UDP instead of stdout (e.g. 127.0.0.1:5555)",
    )
    shared.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to config.toml (auto-detected if omitted)",
    )

    parser = argparse.ArgumentParser(
        description="MeshCore TX message builder. Outputs CBOR TX requests for lora_trx.",
        epilog=_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[shared],
    )
    parser.add_argument(
        "--show-key", action="store_true", help="Print public key hex and exit"
    )

    sub = parser.add_subparsers(dest="command")

    # advert subcommand
    p_advert = sub.add_parser(
        "advert",
        parents=[shared],
        help="Send ADVERT beacon (broadcast identity)",
        description="Broadcast an ADVERT beacon advertising this node's identity. "
        "Uses FLOOD routing by default (ADVERTs are broadcasts).",
    )
    p_advert.add_argument("--name", type=str, default="", help="Node name (UTF-8)")
    p_advert.add_argument(
        "--node-type",
        type=int,
        default=ADVERT_NODE_CHAT,
        choices=[1, 2, 3, 4],
        help="Node type: 1=chat, 2=repeater, 3=room, 4=sensor (default: 1)",
    )
    p_advert.add_argument("--lat", type=float, default=None, help="Latitude (float)")
    p_advert.add_argument("--lon", type=float, default=None, help="Longitude (float)")
    p_advert.add_argument(
        "--qr",
        action="store_true",
        help="Print contact QR code to terminal and exit (no CBOR output)",
    )

    # send subcommand (TXT_MSG)
    p_send = sub.add_parser(
        "send",
        parents=[shared],
        help="Send encrypted message to a known contact",
        description="Send an encrypted TXT_MSG to a contact whose public key you know. "
        "Both parties must have exchanged keys (ECDH shared secret). "
        "Uses DIRECT routing by default.",
    )
    p_send.add_argument(
        "--dest",
        required=True,
        help="Destination public key (64 hex chars = 32 bytes)",
    )
    p_send.add_argument(
        "--route",
        choices=list(ROUTE_NAMES.keys()),
        default="direct",
        help="Routing type (default: direct)",
    )
    p_send.add_argument("message", nargs="+", help="Message text")

    # anon-req subcommand
    p_anon = sub.add_parser(
        "anon-req",
        parents=[shared],
        help="Send anonymous encrypted request (includes sender pubkey)",
        description="Send an encrypted ANON_REQ to a contact. Includes the sender's "
        "public key in cleartext so the recipient can derive the shared secret "
        "without prior contact. Uses DIRECT routing by default.",
    )
    p_anon.add_argument(
        "--dest",
        required=True,
        help="Destination public key (64 hex chars = 32 bytes)",
    )
    p_anon.add_argument(
        "--route",
        choices=list(ROUTE_NAMES.keys()),
        default="direct",
        help="Routing type (default: direct)",
    )
    p_anon.add_argument("message", nargs="+", help="Message text")

    parser.add_argument(
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output (also: NO_COLOR env var)",
    )

    args = parser.parse_args()

    # Configure logging (library-level loggers inherit root config)
    cfg = load_config(args.config)
    setup_logging(
        "gr4.tx", cfg, debug=getattr(args, "debug", False), no_color=args.no_color
    )

    # Load identity
    expanded_prv, pub_key, seed = load_or_create_identity(args.identity)

    if args.show_key:
        print(pub_key.hex())
        return

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Handle --qr for advert subcommand
    if args.command == "advert" and args.qr:
        name = args.name or ""
        uri = build_contact_uri(pub_key, name, args.node_type)
        log.info("contact URI: %s", uri)
        log.info("public key: %s", pub_key.hex())
        if name:
            log.info("name: %s", name)
        # Also show the CLI biz card URI for reference
        packet = cmd_advert(args, expanded_prv, pub_key, seed)
        biz_uri = build_bizcard_uri(packet)
        log.info("biz card: %s", biz_uri)
        print_qr_code(uri)
        return

    # Build packet
    if args.command == "advert":
        packet = cmd_advert(args, expanded_prv, pub_key, seed)
    elif args.command == "send":
        packet = cmd_send(args, expanded_prv, pub_key, seed)
    elif args.command == "anon-req":
        packet = cmd_anon_req(args, expanded_prv, pub_key, seed)
    else:
        parser.print_help()
        sys.exit(1)

    # Log
    log.info("packet: %dB header=0x%02X hex=%s", len(packet), packet[0], packet.hex())
    if args.dry_run:
        log.info("dry run -- CBOR written to stdout, not transmitted")

    # Build CBOR TX request
    phy = {}
    if args.freq is not None:
        phy["freq"] = args.freq
    if args.sf is not None:
        phy["sf"] = args.sf
    if args.bw is not None:
        phy["bw"] = args.bw
    if args.dry_run:
        phy["dry_run"] = True

    cbor_msg = make_cbor_tx_request(packet, **phy)

    if args.udp:
        # Send via UDP to lora_trx
        try:
            host, port = parse_host_port(args.udp)
        except ValueError as exc:
            log.error("%s", exc)
            sys.exit(1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cbor_msg, (host, port))
        sock.close()
        log.info("sent %dB to %s:%d", len(cbor_msg), host, port)
    else:
        sys.stdout.buffer.write(cbor_msg)
        sys.stdout.buffer.flush()


if __name__ == "__main__":
    main()
