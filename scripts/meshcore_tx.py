#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
meshcore_tx.py -- MeshCore protocol TX message builder.

Constructs MeshCore packets (ADVERT, ANON_REQ) and sends them to lora_tx
via CBOR on stdout. Manages persistent Ed25519 identity keypairs.

Usage:
  # Generate or load identity, send ADVERT beacon:
  meshcore_tx.py advert --name "MyNode"

  # Send anonymous encrypted message to a known peer:
  meshcore_tx.py anon-req --dest <pubkey_hex> "Hello from gr4-lora"

  # Pipe to lora_tx:
  meshcore_tx.py advert --name "MyNode" | lora_tx --stdin

  # Dry run (print packet hex to stderr, CBOR to stdout):
  meshcore_tx.py advert --name "MyNode" --dry-run

Key management:
  Identity is stored in ~/.config/gr4-lora/identity.bin (96 bytes:
  64-byte Ed25519 private key + 32-byte public key). Generated on first
  run if missing. Use --identity <path> to override.

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
import hashlib
import hmac
import os
import struct
import sys
import time
from pathlib import Path

import cbor2
import segno
from Crypto.Cipher import AES
from nacl.signing import SigningKey, VerifyKey
from nacl.bindings import (
    crypto_sign_ed25519_pk_to_curve25519,
    crypto_scalarmult,
)

# ---- MeshCore protocol constants ----

# Route types (2 LSBs of header byte)
ROUTE_FLOOD = 0x01
ROUTE_T_FLOOD = 0x00
ROUTE_DIRECT = 0x02
ROUTE_T_DIRECT = 0x03

# Payload types (bits [5:2] of header byte)
PAYLOAD_REQ = 0x00
PAYLOAD_RESP = 0x01
PAYLOAD_TXT = 0x02
PAYLOAD_ACK = 0x03
PAYLOAD_ADVERT = 0x04
PAYLOAD_GRP_TXT = 0x05
PAYLOAD_GRP_DATA = 0x06
PAYLOAD_ANON_REQ = 0x07
PAYLOAD_PATH = 0x08
PAYLOAD_TRACE = 0x09
PAYLOAD_MULTI = 0x0A
PAYLOAD_CTRL = 0x0B
PAYLOAD_RAW_CUSTOM = 0x0F

# Key/crypto sizes (from MeshCore.h)
PUB_KEY_SIZE = 32
PRV_KEY_SIZE = 64
SIGNATURE_SIZE = 64
CIPHER_KEY_SIZE = 16
CIPHER_BLOCK_SIZE = 16
CIPHER_MAC_SIZE = 2
PATH_HASH_SIZE = 1

# ADVERT flags
ADVERT_NODE_CHAT = 0x01
ADVERT_NODE_REPEATER = 0x02
ADVERT_NODE_ROOM = 0x03
ADVERT_NODE_SENSOR = 0x04
ADVERT_HAS_LOCATION = 0x10
ADVERT_HAS_FEATURE1 = 0x20
ADVERT_HAS_FEATURE2 = 0x40
ADVERT_HAS_NAME = 0x80

# Default identity path
DEFAULT_IDENTITY_DIR = Path.home() / ".config" / "gr4-lora"
DEFAULT_IDENTITY_FILE = DEFAULT_IDENTITY_DIR / "identity.bin"


# ---- Identity management ----


def load_or_create_identity(path: Path) -> tuple[bytes, bytes, bytes]:
    """Load or generate an Ed25519 keypair in MeshCore format.

    Returns (expanded_prv_64, pub_key_32, seed_32):
      expanded_prv = SHA-512(seed) with clamping (MeshCore's ed25519 private key)
      pub_key = 32-byte Ed25519 public key
      seed = 32-byte random seed (needed for libsodium signing)

    The file stores [seed(32) | pub_key(32)] = 64 bytes.
    The expanded private key is derived on load via SHA-512(seed).
    """
    if path.exists():
        data = path.read_bytes()
        if len(data) == 64:
            seed = data[:32]
            pub = data[32:64]
            # Validate: derive pubkey from seed and check
            sk = SigningKey(seed)
            if bytes(sk.verify_key) == pub:
                expanded = meshcore_expanded_key(seed)
                sys.stderr.write(f"Loaded identity from {path}\n")
                sys.stderr.write(f"  Public key: {pub.hex()}\n")
                return expanded, pub, seed
            else:
                sys.stderr.write(
                    f"WARNING: identity file {path} has mismatched keys, regenerating\n"
                )
        else:
            sys.stderr.write(
                f"WARNING: identity file {path} has wrong size ({len(data)}), regenerating\n"
            )

    # Generate new keypair
    seed = os.urandom(32)
    sk = SigningKey(seed)
    pub = bytes(sk.verify_key)
    expanded = meshcore_expanded_key(seed)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(seed + pub)
    path.chmod(0o600)
    sys.stderr.write(f"Generated new identity at {path}\n")
    sys.stderr.write(f"  Public key: {pub.hex()}\n")
    return expanded, pub, seed


# ---- MeshCore crypto ----


def _clamp_scalar(scalar: bytes) -> bytes:
    """Apply Curve25519 clamping to a 32-byte scalar.

    This matches MeshCore's ed25519_key_exchange() clamping:
      e[0] &= 248; e[31] &= 63; e[31] |= 64;
    """
    ba = bytearray(scalar)
    ba[0] &= 248
    ba[31] &= 63
    ba[31] |= 64
    return bytes(ba)


def meshcore_expanded_key(seed: bytes) -> bytes:
    """Expand a 32-byte seed into MeshCore's 64-byte private key format.

    MeshCore's ed25519_create_keypair: private_key = SHA-512(seed), then
    clamp bytes [0] and [31]. The first 32 bytes become the Ed25519 scalar,
    the second 32 bytes are used for signing nonces.
    """
    expanded = hashlib.sha512(seed).digest()
    return _clamp_scalar(expanded[:32]) + expanded[32:]


def meshcore_shared_secret(my_prv_64: bytes, other_pub_32: bytes) -> bytes:
    """Compute MeshCore ECDH shared secret.

    Replicates MeshCore's ed25519_key_exchange():
      1. Take private_key[0:32] and clamp (idempotent if already clamped)
      2. Convert Ed25519 public key from Edwards to Montgomery (X25519)
      3. X25519 scalar multiply

    my_prv_64: MeshCore expanded private key (SHA-512(seed) with clamping).
               First 32 bytes are the scalar.
    other_pub_32: Ed25519 public key (32 bytes).
    """
    # Clamp first 32 bytes (idempotent if already clamped by ed25519_create_keypair)
    curve_sk = _clamp_scalar(my_prv_64[:32])
    # Convert Ed25519 public key to Curve25519 (Montgomery X coordinate)
    curve_pk = crypto_sign_ed25519_pk_to_curve25519(other_pub_32)
    # X25519 scalar multiply
    return crypto_scalarmult(curve_sk, curve_pk)


def meshcore_encrypt_then_mac(shared_secret: bytes, plaintext: bytes) -> bytes:
    """Encrypt data using MeshCore's encryptThenMAC.

    AES-128-ECB encrypt (zero-padded to 16-byte blocks), then
    HMAC-SHA256(secret, ciphertext) truncated to 2 bytes.

    Returns: MAC(2) + ciphertext(N*16).
    """
    key = shared_secret[:CIPHER_KEY_SIZE]  # AES-128 uses first 16 bytes

    # Pad plaintext to 16-byte boundary
    padded = plaintext + b"\x00" * (
        (CIPHER_BLOCK_SIZE - len(plaintext) % CIPHER_BLOCK_SIZE) % CIPHER_BLOCK_SIZE
    )

    # AES-128-ECB encrypt
    cipher = AES.new(key, AES.MODE_ECB)
    ciphertext = cipher.encrypt(padded)

    # HMAC-SHA256 truncated to 2 bytes
    mac = hmac.new(shared_secret[:PUB_KEY_SIZE], ciphertext, hashlib.sha256).digest()[
        :CIPHER_MAC_SIZE
    ]

    return mac + ciphertext


def meshcore_mac_then_decrypt(shared_secret: bytes, data: bytes) -> bytes | None:
    """Decrypt data using MeshCore's MACThenDecrypt.

    Verifies 2-byte HMAC-SHA256 MAC, then AES-128-ECB decrypts.

    Returns plaintext on success, None on MAC failure.
    """
    if len(data) <= CIPHER_MAC_SIZE:
        return None

    mac_received = data[:CIPHER_MAC_SIZE]
    ciphertext = data[CIPHER_MAC_SIZE:]

    key = shared_secret[:CIPHER_KEY_SIZE]
    mac_computed = hmac.new(
        shared_secret[:PUB_KEY_SIZE], ciphertext, hashlib.sha256
    ).digest()[:CIPHER_MAC_SIZE]

    if mac_received != mac_computed:
        return None

    cipher = AES.new(key, AES.MODE_ECB)
    return cipher.decrypt(ciphertext)


# ---- MeshCore contact URI + QR code ----

# Node type names for the contact URI
NODE_TYPE_NAMES = {
    ADVERT_NODE_CHAT: "chat",
    ADVERT_NODE_REPEATER: "repeater",
    ADVERT_NODE_ROOM: "room",
    ADVERT_NODE_SENSOR: "sensor",
}


def build_contact_uri(
    pub_key: bytes, name: str, node_type: int = ADVERT_NODE_CHAT
) -> str:
    """Build a MeshCore contact URI for QR code sharing.

    Format: meshcore://contact/add?name=<name>&public_key=<64hex>&type=<int>
    """
    from urllib.parse import quote

    params = []
    if name:
        params.append(f"name={quote(name, safe='')}")
    params.append(f"public_key={pub_key.hex()}")
    params.append(f"type={node_type}")
    return "meshcore://contact/add?" + "&".join(params)


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
        app_parts.append(struct.pack("<ff", lat, lon))

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

    # Build wire packet (ADVERT via FLOOD)
    header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
    return build_wire_packet(header, payload)


def build_anon_req(
    expanded_prv: bytes,
    pub_key: bytes,
    dest_pub: bytes,
    data: bytes,
    *,
    timestamp: int | None = None,
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

    # Build plaintext: timestamp + data
    ts_bytes = struct.pack("<I", timestamp)
    plaintext = ts_bytes + data

    # Encrypt then MAC
    encrypted = meshcore_encrypt_then_mac(secret, plaintext)

    # Build payload: dest_hash + sender_pub + encrypted
    dest_hash = dest_pub[:PATH_HASH_SIZE]
    payload = dest_hash + pub_key + encrypted

    # Build wire packet (ANON_REQ via FLOOD)
    header = make_header(ROUTE_FLOOD, PAYLOAD_ANON_REQ)
    return build_wire_packet(header, payload)


def make_cbor_tx_request(packet: bytes, **phy_overrides) -> bytes:
    """Wrap a MeshCore wire packet in a CBOR TX request for lora_tx --stdin."""
    msg = {
        "type": "lora_tx",
        "payload": packet,
    }
    msg.update(phy_overrides)
    return cbor2.dumps(msg)


# ---- CLI ----


def cmd_advert(args, expanded_prv: bytes, pub_key: bytes, seed: bytes) -> bytes:
    """Build and emit an ADVERT packet."""
    packet = build_advert(
        seed,
        pub_key,
        node_type=args.node_type,
        name=args.name or "",
        lat=args.lat,
        lon=args.lon,
    )
    return packet


def cmd_anon_req(args, expanded_prv: bytes, pub_key: bytes, seed: bytes) -> bytes:
    """Build and emit an ANON_REQ packet."""
    dest_pub = bytes.fromhex(args.dest)
    if len(dest_pub) != PUB_KEY_SIZE:
        sys.stderr.write(
            f"ERROR: destination public key must be {PUB_KEY_SIZE} bytes ({PUB_KEY_SIZE * 2} hex chars)\n"
        )
        sys.exit(1)

    message = " ".join(args.message).encode("utf-8")
    if not message:
        sys.stderr.write("ERROR: message cannot be empty\n")
        sys.exit(1)

    packet = build_anon_req(expanded_prv, pub_key, dest_pub, message)
    return packet


def main():
    parser = argparse.ArgumentParser(
        description="MeshCore TX message builder. Outputs CBOR TX requests for lora_tx --stdin.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--identity",
        type=Path,
        default=DEFAULT_IDENTITY_FILE,
        help=f"Identity file path (default: {DEFAULT_IDENTITY_FILE})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print packet hex to stderr, still output CBOR to stdout",
    )
    parser.add_argument(
        "--freq", type=int, default=None, help="Override TX frequency (Hz)"
    )
    parser.add_argument(
        "--sf", type=int, default=None, help="Override spreading factor"
    )
    parser.add_argument("--bw", type=int, default=None, help="Override bandwidth (Hz)")
    parser.add_argument(
        "--show-key", action="store_true", help="Print public key and exit"
    )
    parser.add_argument(
        "--qr",
        action="store_true",
        help="Print MeshCore contact QR code to terminal and exit",
    )
    parser.add_argument(
        "--qr-name",
        type=str,
        default="",
        help="Node name for QR contact URI (default: empty)",
    )

    sub = parser.add_subparsers(dest="command")

    # advert subcommand
    p_advert = sub.add_parser("advert", help="Send ADVERT beacon")
    p_advert.add_argument("--name", type=str, default="", help="Node name (UTF-8)")
    p_advert.add_argument(
        "--node-type",
        type=int,
        default=ADVERT_NODE_CHAT,
        choices=[1, 2, 3, 4],
        help="Node type: 1=chat, 2=repeater, 3=room, 4=sensor (default: 1)",
    )
    p_advert.add_argument("--lat", type=float, default=None, help="Latitude")
    p_advert.add_argument("--lon", type=float, default=None, help="Longitude")

    # anon-req subcommand
    p_anon = sub.add_parser("anon-req", help="Send anonymous encrypted request")
    p_anon.add_argument(
        "--dest", required=True, help="Destination public key (64 hex chars)"
    )
    p_anon.add_argument("message", nargs="+", help="Message text")

    args = parser.parse_args()

    # Load identity
    expanded_prv, pub_key, seed = load_or_create_identity(args.identity)

    if args.show_key:
        print(pub_key.hex())
        return

    if args.qr:
        name = args.qr_name
        uri = build_contact_uri(pub_key, name)
        sys.stderr.write(f"Contact URI: {uri}\n")
        sys.stderr.write(f"Public key:  {pub_key.hex()}\n")
        if name:
            sys.stderr.write(f"Name:        {name}\n")
        sys.stderr.write("\n")
        print_qr_code(uri)
        return

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Build packet
    if args.command == "advert":
        packet = cmd_advert(args, expanded_prv, pub_key, seed)
    elif args.command == "anon-req":
        packet = cmd_anon_req(args, expanded_prv, pub_key, seed)
    else:
        parser.print_help()
        sys.exit(1)

    # Log
    sys.stderr.write(f"Packet: {len(packet)} bytes\n")
    sys.stderr.write(f"  Header: 0x{packet[0]:02X}\n")
    sys.stderr.write(f"  Hex: {packet.hex()}\n")
    if args.dry_run:
        sys.stderr.write(
            "  (dry run — CBOR written to stdout but not piped to lora_tx)\n"
        )

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
    sys.stdout.buffer.write(cbor_msg)
    sys.stdout.buffer.flush()


if __name__ == "__main__":
    main()
