#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
test_meshcore_crypto.py -- Unit tests for meshcore_crypto.py.

Tests key store load/save, header parsing, ADVERT extraction,
and RX decryption helpers (try_decrypt_txt_msg, try_decrypt_anon_req).

Run:  python3 -m unittest scripts/test_meshcore_crypto.py -v
"""

import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

from nacl.signing import SigningKey

from meshcore_crypto import (
    CIPHER_BLOCK_SIZE,
    CIPHER_MAC_SIZE,
    DEFAULT_KEYS_DIR,
    PAYLOAD_ADVERT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_TXT,
    PUB_KEY_SIZE,
    SIGNATURE_SIZE,
    _clamp_scalar,
    extract_advert_name,
    extract_advert_pubkey,
    load_identity,
    load_known_keys,
    load_or_create_identity,
    meshcore_encrypt_then_mac,
    meshcore_expanded_key,
    meshcore_mac_then_decrypt,
    meshcore_shared_secret,
    parse_meshcore_header,
    save_pubkey,
    try_decrypt_anon_req,
    try_decrypt_txt_msg,
)

# Deterministic test identity
TEST_SEED = bytes.fromhex(
    "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
)
TEST_SEED_PUB = bytes.fromhex(
    "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
)
TEST_SEED_EXPANDED = meshcore_expanded_key(TEST_SEED)


def _build_advert_packet(seed: bytes, pub: bytes, name: str = "") -> bytes:
    """Build a minimal ADVERT packet for testing extraction functions."""
    from meshcore_tx import build_advert

    return build_advert(seed, pub, name=name)


def _build_txt_msg_packet(
    sender_expanded: bytes,
    sender_pub: bytes,
    dest_pub: bytes,
    text: str,
    timestamp: int = 1700000000,
) -> bytes:
    from meshcore_tx import build_txt_msg

    return build_txt_msg(
        sender_expanded, sender_pub, dest_pub, text, timestamp=timestamp
    )


def _build_anon_req_packet(
    sender_expanded: bytes,
    sender_pub: bytes,
    dest_pub: bytes,
    message: bytes,
    timestamp: int = 1700000000,
) -> bytes:
    from meshcore_tx import build_anon_req

    return build_anon_req(
        sender_expanded, sender_pub, dest_pub, message, timestamp=timestamp
    )


# ---- Scalar clamping ----


class TestClampScalar(unittest.TestCase):
    def test_clamp_modifies_bytes(self):
        raw = bytes(32)
        clamped = _clamp_scalar(raw)
        self.assertEqual(clamped[0] & 7, 0)  # low 3 bits cleared
        self.assertEqual(clamped[31] & 0xC0, 0x40)  # bit 6 set, bit 7 cleared

    def test_clamp_length(self):
        self.assertEqual(len(_clamp_scalar(os.urandom(32))), 32)

    def test_clamp_idempotent(self):
        raw = os.urandom(32)
        once = _clamp_scalar(raw)
        twice = _clamp_scalar(once)
        self.assertEqual(once, twice)


# ---- Key expansion ----


class TestExpandedKey(unittest.TestCase):
    def test_length(self):
        expanded = meshcore_expanded_key(os.urandom(32))
        self.assertEqual(len(expanded), 64)

    def test_deterministic(self):
        seed = os.urandom(32)
        self.assertEqual(meshcore_expanded_key(seed), meshcore_expanded_key(seed))

    def test_clamped(self):
        expanded = meshcore_expanded_key(os.urandom(32))
        self.assertEqual(expanded[0] & 7, 0)
        self.assertEqual(expanded[31] & 0xC0, 0x40)


# ---- Key store ----


class TestKeyStore(unittest.TestCase):
    def test_save_and_load(self):
        with tempfile.TemporaryDirectory() as td:
            keys_dir = Path(td) / "keys"
            pub = os.urandom(32)
            self.assertTrue(save_pubkey(keys_dir, pub))
            keys = load_known_keys(keys_dir)
            self.assertIn(pub.hex(), keys)
            self.assertEqual(keys[pub.hex()], pub)

    def test_save_duplicate_returns_false(self):
        with tempfile.TemporaryDirectory() as td:
            keys_dir = Path(td) / "keys"
            pub = os.urandom(32)
            self.assertTrue(save_pubkey(keys_dir, pub))
            self.assertFalse(save_pubkey(keys_dir, pub))

    def test_load_empty_dir(self):
        with tempfile.TemporaryDirectory() as td:
            keys = load_known_keys(Path(td))
            self.assertEqual(keys, {})

    def test_load_nonexistent_dir(self):
        keys = load_known_keys(Path("/nonexistent/path/keys"))
        self.assertEqual(keys, {})

    def test_load_ignores_wrong_size(self):
        with tempfile.TemporaryDirectory() as td:
            keys_dir = Path(td)
            bad = keys_dir / "bad.key"
            bad.write_bytes(b"short")
            keys = load_known_keys(keys_dir)
            self.assertEqual(keys, {})

    def test_multiple_keys(self):
        with tempfile.TemporaryDirectory() as td:
            keys_dir = Path(td) / "keys"
            pubs = [os.urandom(32) for _ in range(5)]
            for pub in pubs:
                save_pubkey(keys_dir, pub)
            keys = load_known_keys(keys_dir)
            self.assertEqual(len(keys), 5)
            for pub in pubs:
                self.assertIn(pub.hex(), keys)


# ---- Identity management ----


class TestLoadIdentity(unittest.TestCase):
    def test_load_nonexistent(self):
        self.assertIsNone(load_identity(Path("/nonexistent/identity.bin")))

    def test_load_wrong_size(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "identity.bin"
            path.write_bytes(b"short")
            self.assertIsNone(load_identity(path))

    def test_load_mismatched_keys(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "identity.bin"
            path.write_bytes(os.urandom(64))  # random seed + random pub = mismatch
            self.assertIsNone(load_identity(path))

    def test_load_valid_identity(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "identity.bin"
            _, _, _ = load_or_create_identity(path)
            result = load_identity(path)
            self.assertIsNotNone(result)
            exp, pub, seed = result
            self.assertEqual(len(exp), 64)
            self.assertEqual(len(pub), 32)
            self.assertEqual(len(seed), 32)


# ---- Header parsing ----


class TestParseMeshcoreHeader(unittest.TestCase):
    def test_too_short(self):
        self.assertIsNone(parse_meshcore_header(b""))
        self.assertIsNone(parse_meshcore_header(b"\x00"))

    def test_flood_advert(self):
        # FLOOD (route=1) + ADVERT (ptype=4) + version 0
        # header = 0x01 | (0x04 << 2) = 0x01 | 0x10 = 0x11
        data = b"\x11\x00" + b"\xaa" * 10
        hdr = parse_meshcore_header(data)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr["route"], 1)  # FLOOD
        self.assertEqual(hdr["ptype"], 4)  # ADVERT
        self.assertEqual(hdr["version"], 0)
        self.assertFalse(hdr["has_transport"])

    def test_t_flood_with_transport(self):
        # T_FLOOD (route=0) + REQ (ptype=0) + version 0
        # header = 0x00
        data = b"\x00" + b"\x01\x02\x03\x04" + b"\x00" + b"\xff" * 5
        hdr = parse_meshcore_header(data)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr["route"], 0)  # T_FLOOD
        self.assertTrue(hdr["has_transport"])

    def test_direct_txt(self):
        # DIRECT (route=2) + TXT (ptype=2) + version 0
        # header = 0x02 | (0x02 << 2) = 0x02 | 0x08 = 0x0A
        data = b"\x0a\x00" + b"\xbb" * 20
        hdr = parse_meshcore_header(data)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr["route"], 2)  # DIRECT
        self.assertEqual(hdr["ptype"], 2)  # TXT
        self.assertFalse(hdr["has_transport"])

    def test_path_length_parsed(self):
        # FLOOD + ADVERT, path_len=3
        data = b"\x11\x03\x01\x02\x03" + b"\xcc" * 10
        hdr = parse_meshcore_header(data)
        self.assertIsNotNone(hdr)
        # off should be past header(1) + path_len(1) + path(3) = 5
        self.assertEqual(hdr["off"], 5)

    def test_version_bits(self):
        # version=2, route=1 (FLOOD), ptype=0 (REQ)
        # header = (2 << 6) | (0 << 2) | 1 = 0x81
        data = b"\x81\x00" + b"\xdd" * 5
        hdr = parse_meshcore_header(data)
        self.assertIsNotNone(hdr)
        self.assertEqual(hdr["version"], 2)


# ---- ADVERT extraction ----


class TestExtractAdvert(unittest.TestCase):
    def test_extract_pubkey(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB, name="Test")
        pubkey = extract_advert_pubkey(pkt)
        self.assertIsNotNone(pubkey)
        self.assertEqual(pubkey, TEST_SEED_PUB)

    def test_extract_pubkey_no_name(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB)
        pubkey = extract_advert_pubkey(pkt)
        self.assertEqual(pubkey, TEST_SEED_PUB)

    def test_extract_name(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB, name="MyNode")
        name = extract_advert_name(pkt)
        self.assertEqual(name, "MyNode")

    def test_extract_name_no_name(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB)
        name = extract_advert_name(pkt)
        self.assertIsNone(name)

    def test_non_advert_returns_none(self):
        # TXT packet, not ADVERT
        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_pub = bytes(sk2.verify_key)
        pkt = _build_txt_msg_packet(TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, "Hi")
        self.assertIsNone(extract_advert_pubkey(pkt))
        self.assertIsNone(extract_advert_name(pkt))

    def test_truncated_packet(self):
        self.assertIsNone(extract_advert_pubkey(b"\x11\x00\xaa"))
        self.assertIsNone(extract_advert_name(b"\x11\x00\xaa"))

    def test_empty_packet(self):
        self.assertIsNone(extract_advert_pubkey(b""))
        self.assertIsNone(extract_advert_name(b""))


# ---- try_decrypt_txt_msg ----


class TestTryDecryptTxtMsg(unittest.TestCase):
    def setUp(self):
        self.dest_seed = os.urandom(32)
        self.dest_sk = SigningKey(self.dest_seed)
        self.dest_pub = bytes(self.dest_sk.verify_key)
        self.dest_expanded = meshcore_expanded_key(self.dest_seed)

    def test_decrypt_success(self):
        pkt = _build_txt_msg_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "Hello"
        )
        known_keys = {TEST_SEED_PUB.hex(): TEST_SEED_PUB}
        result = try_decrypt_txt_msg(pkt, self.dest_expanded, self.dest_pub, known_keys)
        self.assertIsNotNone(result)
        text, sender = result
        self.assertEqual(text, "Hello")
        self.assertEqual(sender, TEST_SEED_PUB)

    def test_decrypt_no_matching_key(self):
        pkt = _build_txt_msg_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "Secret"
        )
        # Known keys don't include the sender — use a valid Ed25519 pubkey
        other_sk = SigningKey(os.urandom(32))
        other_pub = bytes(other_sk.verify_key)
        known_keys = {other_pub.hex(): other_pub}
        result = try_decrypt_txt_msg(pkt, self.dest_expanded, self.dest_pub, known_keys)
        self.assertIsNone(result)

    def test_decrypt_empty_known_keys(self):
        pkt = _build_txt_msg_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "Nope"
        )
        result = try_decrypt_txt_msg(pkt, self.dest_expanded, self.dest_pub, {})
        self.assertIsNone(result)

    def test_non_txt_returns_none(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB, name="Adv")
        known_keys = {TEST_SEED_PUB.hex(): TEST_SEED_PUB}
        result = try_decrypt_txt_msg(pkt, self.dest_expanded, self.dest_pub, known_keys)
        self.assertIsNone(result)

    def test_decrypt_multiple_keys_finds_correct(self):
        pkt = _build_txt_msg_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "FindMe"
        )
        # Several valid Ed25519 keys plus the correct one
        known_keys = {}
        for _ in range(5):
            rk_sk = SigningKey(os.urandom(32))
            rk = bytes(rk_sk.verify_key)
            known_keys[rk.hex()] = rk
        known_keys[TEST_SEED_PUB.hex()] = TEST_SEED_PUB

        result = try_decrypt_txt_msg(pkt, self.dest_expanded, self.dest_pub, known_keys)
        self.assertIsNotNone(result)
        text, sender = result
        self.assertEqual(text, "FindMe")
        self.assertEqual(sender, TEST_SEED_PUB)


# ---- try_decrypt_anon_req ----


class TestTryDecryptAnonReq(unittest.TestCase):
    def setUp(self):
        self.dest_seed = os.urandom(32)
        self.dest_sk = SigningKey(self.dest_seed)
        self.dest_pub = bytes(self.dest_sk.verify_key)
        self.dest_expanded = meshcore_expanded_key(self.dest_seed)

    def test_decrypt_success(self):
        pkt = _build_anon_req_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, b"AnonHello"
        )
        result = try_decrypt_anon_req(pkt, self.dest_expanded, self.dest_pub)
        self.assertIsNotNone(result)
        text, sender = result
        self.assertEqual(text, "AnonHello")
        self.assertEqual(sender, TEST_SEED_PUB)

    def test_decrypt_wrong_recipient(self):
        # Build packet addressed to dest, but try to decrypt as different identity
        other_seed = os.urandom(32)
        other_sk = SigningKey(other_seed)
        other_pub = bytes(other_sk.verify_key)
        other_expanded = meshcore_expanded_key(other_seed)

        pkt = _build_anon_req_packet(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, b"NotForYou"
        )
        # dest_hash won't match other's pubkey first byte (usually)
        result = try_decrypt_anon_req(pkt, other_expanded, other_pub)
        # Should be None because dest_hash doesn't match
        # (There's a 1/256 chance the first bytes match, so we
        # accept either None or a valid result in this edge case)
        if other_pub[0] != self.dest_pub[0]:
            self.assertIsNone(result)

    def test_non_anon_req_returns_none(self):
        pkt = _build_advert_packet(TEST_SEED, TEST_SEED_PUB, name="Adv")
        result = try_decrypt_anon_req(pkt, self.dest_expanded, self.dest_pub)
        self.assertIsNone(result)

    def test_truncated_packet(self):
        result = try_decrypt_anon_req(
            b"\x1d\x00\xaa", self.dest_expanded, self.dest_pub
        )
        self.assertIsNone(result)


# ---- Encrypt/decrypt round-trip (additional coverage beyond test_meshcore_tx) ----


class TestEncryptDecryptEdgeCases(unittest.TestCase):
    def test_empty_plaintext(self):
        secret = b"\x42" * 32
        encrypted = meshcore_encrypt_then_mac(secret, b"")
        # Empty plaintext: (0 % 16) % 16 = 0, no padding → MAC(2) only
        self.assertEqual(len(encrypted), CIPHER_MAC_SIZE)
        # Decrypt returns None because len(data) <= MAC_SIZE — no ciphertext
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertIsNone(decrypted)

    def test_decrypt_too_short(self):
        secret = b"\x42" * 32
        self.assertIsNone(meshcore_mac_then_decrypt(secret, b""))
        self.assertIsNone(meshcore_mac_then_decrypt(secret, b"\x00"))
        self.assertIsNone(meshcore_mac_then_decrypt(secret, b"\x00\x00"))

    def test_exact_block_size(self):
        secret = b"\x42" * 32
        plaintext = b"A" * 16
        encrypted = meshcore_encrypt_then_mac(secret, plaintext)
        self.assertEqual(len(encrypted), CIPHER_MAC_SIZE + 16)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertEqual(decrypted, plaintext)


if __name__ == "__main__":
    unittest.main()
