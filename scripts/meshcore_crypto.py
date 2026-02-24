#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
meshcore_crypto.py -- Shared MeshCore cryptographic primitives and key management.

Provides ECDH key exchange, AES-128-ECB encryption, HMAC-SHA256 MAC, identity
management, and a contact key store for MeshCore protocol operations.

Used by meshcore_tx.py (TX) and lora_mon.py (RX decryption).

Crypto stack:
  - Ed25519 signing (pynacl / libsodium)
  - X25519 ECDH key exchange (Ed25519 keys converted to Curve25519)
  - AES-128-ECB + 2-byte HMAC-SHA256 MAC (pycryptodome)
"""

from __future__ import annotations

import hashlib
import hmac as hmac_mod
import os
import struct
import sys
from pathlib import Path

from Crypto.Cipher import AES
from nacl.signing import SigningKey
from nacl.bindings import (
    crypto_sign_ed25519_pk_to_curve25519,
    crypto_scalarmult,
)

# ---- Constants ----

PUB_KEY_SIZE = 32
PRV_KEY_SIZE = 64
SIGNATURE_SIZE = 64
CIPHER_KEY_SIZE = 16
CIPHER_BLOCK_SIZE = 16
CIPHER_MAC_SIZE = 2
PATH_HASH_SIZE = 1

# MeshCore payload types (bits [5:2] of header byte)
PAYLOAD_REQ = 0x00
PAYLOAD_TXT = 0x02
PAYLOAD_ADVERT = 0x04
PAYLOAD_ANON_REQ = 0x07

# Default paths
DEFAULT_CONFIG_DIR = Path.home() / ".config" / "gr4-lora"
DEFAULT_IDENTITY_FILE = DEFAULT_CONFIG_DIR / "identity.bin"
DEFAULT_KEYS_DIR = DEFAULT_CONFIG_DIR / "keys"


# ---- Scalar clamping ----


def _clamp_scalar(scalar: bytes) -> bytes:
    """Apply Curve25519 clamping to a 32-byte scalar."""
    ba = bytearray(scalar)
    ba[0] &= 248
    ba[31] &= 63
    ba[31] |= 64
    return bytes(ba)


# ---- Key expansion ----


def meshcore_expanded_key(seed: bytes) -> bytes:
    """Expand a 32-byte seed into MeshCore's 64-byte private key format.

    SHA-512(seed), then clamp bytes [0] and [31]. First 32 bytes are the
    Ed25519 scalar, second 32 bytes are the signing nonce half.
    """
    expanded = hashlib.sha512(seed).digest()
    return _clamp_scalar(expanded[:32]) + expanded[32:]


# ---- ECDH shared secret ----


def meshcore_shared_secret(my_prv_64: bytes, other_pub_32: bytes) -> bytes | None:
    """Compute MeshCore ECDH shared secret via X25519.

    my_prv_64: 64-byte expanded private key (first 32 bytes = scalar).
    other_pub_32: 32-byte Ed25519 public key.

    Returns 32-byte shared secret, or None if other_pub_32 is not a valid
    Ed25519 public key (e.g. truncated, corrupted, or wrong key type).
    """
    try:
        curve_sk = _clamp_scalar(my_prv_64[:32])
        curve_pk = crypto_sign_ed25519_pk_to_curve25519(other_pub_32)
        return crypto_scalarmult(curve_sk, curve_pk)
    except Exception:
        return None


# ---- Encrypt / Decrypt ----


def meshcore_encrypt_then_mac(shared_secret: bytes, plaintext: bytes) -> bytes:
    """AES-128-ECB encrypt then HMAC-SHA256 MAC (truncated to 2 bytes).

    Returns: MAC(2) + ciphertext(N*16).
    """
    key = shared_secret[:CIPHER_KEY_SIZE]
    padded = plaintext + b"\x00" * (
        (CIPHER_BLOCK_SIZE - len(plaintext) % CIPHER_BLOCK_SIZE) % CIPHER_BLOCK_SIZE
    )
    cipher = AES.new(key, AES.MODE_ECB)
    ciphertext = cipher.encrypt(padded)
    mac = hmac_mod.new(
        shared_secret[:PUB_KEY_SIZE], ciphertext, hashlib.sha256
    ).digest()[:CIPHER_MAC_SIZE]
    return mac + ciphertext


def meshcore_mac_then_decrypt(shared_secret: bytes, data: bytes) -> bytes | None:
    """Verify 2-byte HMAC-SHA256 MAC then AES-128-ECB decrypt.

    Returns plaintext on success, None on MAC failure.
    """
    if len(data) <= CIPHER_MAC_SIZE:
        return None
    mac_received = data[:CIPHER_MAC_SIZE]
    ciphertext = data[CIPHER_MAC_SIZE:]
    mac_computed = hmac_mod.new(
        shared_secret[:PUB_KEY_SIZE], ciphertext, hashlib.sha256
    ).digest()[:CIPHER_MAC_SIZE]
    if mac_received != mac_computed:
        return None
    cipher = AES.new(shared_secret[:CIPHER_KEY_SIZE], AES.MODE_ECB)
    return cipher.decrypt(ciphertext)


# ---- Identity management ----


def load_or_create_identity(path: Path) -> tuple[bytes, bytes, bytes]:
    """Load or generate an Ed25519 keypair in MeshCore format.

    Returns (expanded_prv_64, pub_key_32, seed_32).
    File format: [seed(32) | pub_key(32)] = 64 bytes.
    """
    if path.exists():
        data = path.read_bytes()
        if len(data) == 64:
            seed = data[:32]
            pub = data[32:64]
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


def load_identity(path: Path) -> tuple[bytes, bytes, bytes] | None:
    """Load an existing identity (no generation). Returns None if missing."""
    if not path.exists():
        return None
    data = path.read_bytes()
    if len(data) != 64:
        return None
    seed = data[:32]
    pub = data[32:64]
    sk = SigningKey(seed)
    if bytes(sk.verify_key) != pub:
        return None
    return meshcore_expanded_key(seed), pub, seed


# ---- Contact key store ----


def load_known_keys(keys_dir: Path) -> dict[str, bytes]:
    """Load all known public keys from the keys directory.

    Returns {hex_pubkey: raw_32_bytes} for each .key file.
    File naming: <64-hex-chars>.key containing 32 raw bytes.
    """
    keys: dict[str, bytes] = {}
    if not keys_dir.is_dir():
        return keys
    for f in keys_dir.glob("*.key"):
        data = f.read_bytes()
        if len(data) == PUB_KEY_SIZE:
            keys[data.hex()] = data
    return keys


def save_pubkey(keys_dir: Path, pubkey: bytes, name: str | None = None) -> bool:
    """Save a public key to the key store. Returns True if newly saved."""
    keys_dir.mkdir(parents=True, exist_ok=True)
    path = keys_dir / f"{pubkey.hex()}.key"
    if path.exists():
        return False
    path.write_bytes(pubkey)
    return True


# ---- RX decryption helpers ----


def parse_meshcore_header(data: bytes) -> dict | None:
    """Parse the MeshCore header byte and transport/path prefix.

    Returns dict with route, ptype, version, off (offset to app_payload), path_len
    or None if data is too short.
    """
    if len(data) < 2:
        return None
    hdr = data[0]
    route = hdr & 0x03
    ptype = (hdr >> 2) & 0x0F
    version = (hdr >> 6) & 0x03
    has_transport = route in (0, 3)

    off = 1
    if has_transport:
        if off + 4 > len(data):
            return None
        off += 4

    if off >= len(data):
        return None
    path_len = data[off]
    off += 1 + path_len

    return {
        "route": route,
        "ptype": ptype,
        "version": version,
        "off": off,
        "path_len": data[off - path_len - 1] if path_len >= 0 else 0,
        "has_transport": has_transport,
    }


def extract_advert_pubkey(data: bytes) -> bytes | None:
    """Extract the 32-byte public key from an ADVERT packet payload."""
    hdr = parse_meshcore_header(data)
    if hdr is None or hdr["ptype"] != PAYLOAD_ADVERT:
        return None
    off = hdr["off"]
    if off + PUB_KEY_SIZE > len(data):
        return None
    return data[off : off + PUB_KEY_SIZE]


def extract_advert_name(data: bytes) -> str | None:
    """Extract the node name from an ADVERT packet (if present)."""
    hdr = parse_meshcore_header(data)
    if hdr is None or hdr["ptype"] != PAYLOAD_ADVERT:
        return None
    off = hdr["off"]
    # ADVERT payload: pubkey(32) + timestamp(4) + signature(64) + app_data
    app_off = off + PUB_KEY_SIZE + 4 + SIGNATURE_SIZE
    if app_off >= len(data):
        return None
    flags = data[app_off]
    name_off = app_off + 1
    if flags & 0x10:  # HAS_LOCATION
        name_off += 8
    if flags & 0x20:  # HAS_FEATURE1
        name_off += 2
    if flags & 0x40:  # HAS_FEATURE2
        name_off += 2
    if (flags & 0x80) and name_off < len(data):  # HAS_NAME
        return data[name_off:].decode("utf-8", errors="replace")
    return None


def try_decrypt_txt_msg(
    data: bytes, our_prv: bytes, our_pub: bytes, known_keys: dict[str, bytes]
) -> tuple[str, bytes] | None:
    """Try to decrypt a TXT_MSG using all known keys.

    TXT_MSG payload: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
    Plaintext: timestamp(4) + attempt(1) + text

    Returns (decrypted_text, matched_pubkey) or None.
    """
    hdr = parse_meshcore_header(data)
    if hdr is None or hdr["ptype"] != PAYLOAD_TXT:
        return None
    off = hdr["off"]
    payload = data[off:]
    if len(payload) < 2 + CIPHER_MAC_SIZE + CIPHER_BLOCK_SIZE:
        return None

    dest_hash = payload[0:1]
    src_hash = payload[1:2]
    encrypted = payload[2:]

    # Try our own pubkey hash as dest
    our_hash = our_pub[:PATH_HASH_SIZE]

    for hex_key, pubkey_bytes in known_keys.items():
        secret = meshcore_shared_secret(our_prv, pubkey_bytes)
        if secret is None:
            continue
        plaintext = meshcore_mac_then_decrypt(secret, encrypted)
        if plaintext is not None:
            # Verify hashes match (dest should be us, src should be them)
            their_hash = pubkey_bytes[:PATH_HASH_SIZE]
            if dest_hash == our_hash and src_hash == their_hash:
                # Extract text: skip timestamp(4) + attempt(1)
                if len(plaintext) > 5:
                    text = (
                        plaintext[5:].rstrip(b"\x00").decode("utf-8", errors="replace")
                    )
                    return text, pubkey_bytes
            # MAC matched but hash didn't — still return (could be for someone else)
            if len(plaintext) > 5:
                text = plaintext[5:].rstrip(b"\x00").decode("utf-8", errors="replace")
                return text, pubkey_bytes

    return None


def try_decrypt_anon_req(
    data: bytes, our_prv: bytes, our_pub: bytes
) -> tuple[str, bytes] | None:
    """Try to decrypt an ANON_REQ addressed to us.

    ANON_REQ payload: dest_hash(1) + sender_pub(32) + MAC(2) + ciphertext
    Plaintext: timestamp(4) + message_data

    Returns (decrypted_text, sender_pubkey) or None.
    """
    hdr = parse_meshcore_header(data)
    if hdr is None or hdr["ptype"] != PAYLOAD_ANON_REQ:
        return None
    off = hdr["off"]
    payload = data[off:]
    if len(payload) < 1 + PUB_KEY_SIZE + CIPHER_MAC_SIZE + CIPHER_BLOCK_SIZE:
        return None

    dest_hash = payload[0:1]
    sender_pub = payload[1 : 1 + PUB_KEY_SIZE]
    encrypted = payload[1 + PUB_KEY_SIZE :]

    our_hash = our_pub[:PATH_HASH_SIZE]
    if dest_hash != our_hash:
        return None

    secret = meshcore_shared_secret(our_prv, sender_pub)
    if secret is None:
        return None
    plaintext = meshcore_mac_then_decrypt(secret, encrypted)
    if plaintext is None:
        return None

    # Extract text: skip timestamp(4)
    if len(plaintext) > 4:
        text = plaintext[4:].rstrip(b"\x00").decode("utf-8", errors="replace")
        return text, sender_pub

    return None
