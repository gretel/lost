#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
test_meshcore_tx.py -- Unit tests for meshcore_tx.py.

Cross-validates crypto against MeshCore C++ implementation using known
test vectors from Identity.cpp (validatePrivateKey test client keypair).
"""

import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from meshcore_crypto import (
    CIPHER_MAC_SIZE,
    PATH_HASH_SIZE,
    PUB_KEY_SIZE,
    SIGNATURE_SIZE,
    load_or_create_identity,
    meshcore_encrypt_then_mac,
    meshcore_expanded_key,
    meshcore_mac_then_decrypt,
    meshcore_shared_secret,
)
from meshcore_tx import (
    ADVERT_HAS_NAME,
    ADVERT_NODE_CHAT,
    ADVERT_NODE_REPEATER,
    PAYLOAD_ADVERT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_TXT,
    ROUTE_DIRECT,
    ROUTE_FLOOD,
    build_advert,
    build_anon_req,
    build_bizcard_uri,
    build_contact_uri,
    build_txt_msg,
    build_wire_packet,
    make_cbor_tx_request,
    make_header,
)


# ---- Known test vectors from MeshCore Identity.cpp ----

# Fixed seed for deterministic signing tests (NOT from MeshCore — we generate this).
# Used for ADVERT signing tests where we need the seed for libsodium's Ed25519.
TEST_SEED = bytes.fromhex(
    "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
)
TEST_SEED_PUB = bytes.fromhex(
    "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
)
TEST_SEED_EXPANDED = meshcore_expanded_key(TEST_SEED)

# Test client keypair from LocalIdentity::validatePrivateKey()
# NOTE: This is in MeshCore EXPANDED format (SHA-512(seed) clamped), NOT a seed.
# Used only for shared secret / key exchange tests.
TEST_CLIENT_PRV = bytes(
    [
        0x70,
        0x65,
        0xE1,
        0x8F,
        0xD9,
        0xFA,
        0xBB,
        0x70,
        0xC1,
        0xED,
        0x90,
        0xDC,
        0xA1,
        0x99,
        0x07,
        0xDE,
        0x69,
        0x8C,
        0x88,
        0xB7,
        0x09,
        0xEA,
        0x14,
        0x6E,
        0xAF,
        0xD9,
        0x3D,
        0x9B,
        0x83,
        0x0C,
        0x7B,
        0x60,
        0xC4,
        0x68,
        0x11,
        0x93,
        0xC7,
        0x9B,
        0xBC,
        0x39,
        0x94,
        0x5B,
        0xA8,
        0x06,
        0x41,
        0x04,
        0xBB,
        0x61,
        0x8F,
        0x8F,
        0xD7,
        0xA8,
        0x4A,
        0x0A,
        0xF6,
        0xF5,
        0x70,
        0x33,
        0xD6,
        0xE8,
        0xDD,
        0xCD,
        0x64,
        0x71,
    ]
)
TEST_CLIENT_PUB = bytes(
    [
        0x1E,
        0xC7,
        0x71,
        0x75,
        0xB0,
        0x91,
        0x8E,
        0xD2,
        0x06,
        0xF9,
        0xAE,
        0x04,
        0xEC,
        0x13,
        0x6D,
        0x6D,
        0x5D,
        0x43,
        0x15,
        0xBB,
        0x26,
        0x30,
        0x54,
        0x27,
        0xF6,
        0x45,
        0xB4,
        0x92,
        0xE9,
        0x35,
        0x0C,
        0x10,
    ]
)


class TestHeader(unittest.TestCase):
    def test_advert_flood(self):
        h = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        self.assertEqual(h & 0x03, ROUTE_FLOOD)
        self.assertEqual((h >> 2) & 0x0F, PAYLOAD_ADVERT)
        self.assertEqual((h >> 6) & 0x03, 0)  # version 0

    def test_anon_req_flood(self):
        h = make_header(ROUTE_FLOOD, PAYLOAD_ANON_REQ)
        self.assertEqual(h & 0x03, ROUTE_FLOOD)
        self.assertEqual((h >> 2) & 0x0F, PAYLOAD_ANON_REQ)

    def test_roundtrip_all_types(self):
        for pt in range(16):
            for rt in range(4):
                h = make_header(rt, pt)
                self.assertEqual(h & 0x03, rt)
                self.assertEqual((h >> 2) & 0x0F, pt)


class TestWirePacket(unittest.TestCase):
    def test_basic_structure(self):
        header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        payload = b"\x01\x02\x03"
        pkt = build_wire_packet(header, payload)

        self.assertEqual(pkt[0], header)
        self.assertEqual(pkt[1], 0)  # path_len = 0
        self.assertEqual(pkt[2:], payload)

    def test_with_path(self):
        header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        payload = b"\xaa\xbb"
        path = b"\x01\x02\x03"
        pkt = build_wire_packet(header, payload, path=path)

        self.assertEqual(pkt[0], header)
        self.assertEqual(pkt[1], 3)  # path_len
        self.assertEqual(pkt[2:5], path)
        self.assertEqual(pkt[5:], payload)

    def test_with_transport_codes(self):
        header = 0x00  # T_FLOOD
        payload = b"\xff"
        pkt = build_wire_packet(header, payload, transport_codes=(0x1234, 0x5678))

        self.assertEqual(pkt[0], header)
        self.assertEqual(struct.unpack_from("<HH", pkt, 1), (0x1234, 0x5678))
        self.assertEqual(pkt[5], 0)  # path_len
        self.assertEqual(pkt[6:], payload)


class TestSharedSecret(unittest.TestCase):
    """Cross-validate ECDH key exchange with MeshCore test vectors.

    MeshCore's validatePrivateKey() computes shared secrets from both sides
    and checks they match. We replicate this test.
    """

    def test_shared_secret_symmetry(self):
        """Both parties must compute the same shared secret."""
        from nacl.signing import SigningKey

        # Generate a second keypair in MeshCore expanded format
        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        expanded2 = meshcore_expanded_key(seed2)
        pub2 = bytes(sk2.verify_key)

        ss1 = meshcore_shared_secret(TEST_CLIENT_PRV, pub2)
        ss2 = meshcore_shared_secret(expanded2, TEST_CLIENT_PUB)

        self.assertEqual(ss1, ss2)
        self.assertEqual(len(ss1), 32)

    def test_shared_secret_not_zero(self):
        """Shared secret must not be all zeros (MeshCore rejects these)."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        pub2 = bytes(sk2.verify_key)

        ss = meshcore_shared_secret(TEST_CLIENT_PRV, pub2)
        self.assertNotEqual(ss, b"\x00" * 32)

    def test_known_keypair_exchange(self):
        """Replicate MeshCore's validatePrivateKey() test:
        ss1 = key_exchange(test_client_pub, prv)
        ss2 = key_exchange(derived_pub, test_client_prv)
        assert ss1 == ss2
        """
        from nacl.signing import SigningKey

        seed_new = os.urandom(32)
        sk_new = SigningKey(seed_new)
        expanded_new = meshcore_expanded_key(seed_new)
        pub_new = bytes(sk_new.verify_key)

        ss1 = meshcore_shared_secret(expanded_new, TEST_CLIENT_PUB)
        ss2 = meshcore_shared_secret(TEST_CLIENT_PRV, pub_new)

        self.assertEqual(ss1, ss2)


class TestEncryption(unittest.TestCase):
    def test_encrypt_decrypt_roundtrip(self):
        """encryptThenMAC -> MACThenDecrypt must recover original data."""
        secret = b"\x42" * 32
        plaintext = b"Hello MeshCore"

        encrypted = meshcore_encrypt_then_mac(secret, plaintext)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)

        self.assertIsNotNone(decrypted)
        # Decrypted may have trailing zero padding
        self.assertTrue(decrypted.startswith(plaintext))

    def test_mac_detects_corruption(self):
        """Corrupted ciphertext must fail MAC check."""
        secret = b"\x42" * 32
        plaintext = b"Hello MeshCore"

        encrypted = meshcore_encrypt_then_mac(secret, plaintext)
        # Corrupt one byte of ciphertext (after MAC)
        corrupted = encrypted[:3] + bytes([encrypted[3] ^ 0xFF]) + encrypted[4:]
        result = meshcore_mac_then_decrypt(secret, corrupted)
        self.assertIsNone(result)

    def test_wrong_key_fails(self):
        """Wrong shared secret must fail MAC check."""
        secret1 = b"\x42" * 32
        secret2 = b"\x43" * 32
        plaintext = b"Secret message"

        encrypted = meshcore_encrypt_then_mac(secret1, plaintext)
        result = meshcore_mac_then_decrypt(secret2, encrypted)
        self.assertIsNone(result)

    def test_encrypt_format(self):
        """Encrypted output must be MAC(2) + ciphertext(N*16)."""
        secret = b"\x00" * 32
        plaintext = b"X" * 15  # 15 bytes -> padded to 16

        encrypted = meshcore_encrypt_then_mac(secret, plaintext)
        self.assertEqual(len(encrypted), CIPHER_MAC_SIZE + 16)

    def test_encrypt_block_alignment(self):
        """Test various plaintext sizes for correct padding."""
        secret = b"\xab" * 32
        for size in [1, 15, 16, 17, 31, 32, 48, 100]:
            plaintext = b"A" * size
            encrypted = meshcore_encrypt_then_mac(secret, plaintext)
            ciphertext_len = len(encrypted) - CIPHER_MAC_SIZE
            self.assertEqual(ciphertext_len % 16, 0)
            # Roundtrip
            decrypted = meshcore_mac_then_decrypt(secret, encrypted)
            self.assertIsNotNone(decrypted)
            self.assertTrue(decrypted.startswith(plaintext))


class TestAdvert(unittest.TestCase):
    def test_advert_structure(self):
        """ADVERT packet must have correct wire format."""
        pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name="TestNode")

        # Wire: header(1) + path_len(1) + payload
        self.assertEqual(pkt[0] & 0x03, ROUTE_FLOOD)
        self.assertEqual((pkt[0] >> 2) & 0x0F, PAYLOAD_ADVERT)
        self.assertEqual(pkt[1], 0)  # path_len = 0

        payload = pkt[2:]
        # payload: pubkey(32) + timestamp(4) + signature(64) + app_data
        self.assertEqual(payload[:PUB_KEY_SIZE], TEST_SEED_PUB)

        ts_offset = PUB_KEY_SIZE
        sig_offset = ts_offset + 4
        app_offset = sig_offset + SIGNATURE_SIZE

        # Verify signature is present (64 bytes, non-zero)
        signature = payload[sig_offset:app_offset]
        self.assertEqual(len(signature), SIGNATURE_SIZE)
        self.assertNotEqual(signature, b"\x00" * SIGNATURE_SIZE)

        # Verify app_data starts with flags byte
        app_data = payload[app_offset:]
        flags = app_data[0]
        self.assertEqual(flags & 0x0F, ADVERT_NODE_CHAT)
        self.assertTrue(flags & ADVERT_HAS_NAME)

        # Node name at the end
        name_bytes = app_data[1:]  # no location, no features
        self.assertEqual(name_bytes, b"TestNode")

    def test_advert_min_size(self):
        """Minimum ADVERT (no name, no location): header + path_len + 32 + 4 + 64 + 1."""
        pkt = build_advert(TEST_SEED, TEST_SEED_PUB)
        # wire: header(1) + path_len(1) + pubkey(32) + ts(4) + sig(64) + flags(1) = 103
        self.assertEqual(len(pkt), 103)

    def test_advert_signature_verifies(self):
        """Signature in ADVERT must verify with the public key."""
        from nacl.signing import VerifyKey

        pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name="Verify")
        payload = pkt[2:]

        pub = payload[:PUB_KEY_SIZE]
        ts = payload[PUB_KEY_SIZE : PUB_KEY_SIZE + 4]
        sig = payload[PUB_KEY_SIZE + 4 : PUB_KEY_SIZE + 4 + SIGNATURE_SIZE]
        app_data = payload[PUB_KEY_SIZE + 4 + SIGNATURE_SIZE :]

        # Message that was signed
        message = pub + ts + app_data

        vk = VerifyKey(pub)
        # This will raise if signature is invalid
        vk.verify(message, sig)

    def test_advert_with_location(self):
        """ADVERT with GPS coordinates includes lat/lon."""
        pkt = build_advert(
            TEST_SEED, TEST_SEED_PUB, name="GPS", lat=51.5074, lon=-0.1278
        )
        payload = pkt[2:]
        app_data = payload[PUB_KEY_SIZE + 4 + SIGNATURE_SIZE :]

        flags = app_data[0]
        self.assertTrue(flags & ADVERT_HAS_NAME)
        self.assertTrue(flags & 0x10)  # HAS_LOCATION

        lat, lon = struct.unpack_from("<ff", app_data, 1)
        self.assertAlmostEqual(lat, 51.5074, places=3)
        self.assertAlmostEqual(lon, -0.1278, places=3)


class TestAnonReq(unittest.TestCase):
    def test_anon_req_structure(self):
        """ANON_REQ packet must have correct wire format."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_anon_req(TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, b"Hello")

        # Wire: header(1) + path_len(1) + payload
        self.assertEqual(pkt[0] & 0x03, ROUTE_DIRECT)  # default is DIRECT
        self.assertEqual((pkt[0] >> 2) & 0x0F, PAYLOAD_ANON_REQ)
        self.assertEqual(pkt[1], 0)  # path_len

        payload = pkt[2:]
        # payload: dest_hash(1) + sender_pub(32) + MAC(2) + ciphertext
        self.assertEqual(payload[0], dest_pub[0])  # dest_hash = first byte of pubkey
        self.assertEqual(payload[1 : 1 + PUB_KEY_SIZE], TEST_SEED_PUB)

    def test_anon_req_flood_route(self):
        """ANON_REQ can use FLOOD routing explicitly."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_anon_req(
            TEST_SEED_EXPANDED,
            TEST_SEED_PUB,
            dest_pub,
            b"Hi",
            route_type=ROUTE_FLOOD,
        )
        self.assertEqual(pkt[0] & 0x03, ROUTE_FLOOD)

    def test_anon_req_decrypt(self):
        """Recipient must be able to decrypt ANON_REQ."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_expanded = meshcore_expanded_key(seed2)
        dest_pub = bytes(sk2.verify_key)

        message = b"Hello from gr4-lora"
        pkt = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, message, timestamp=1700000000
        )

        payload = pkt[2:]
        # Skip dest_hash(1) + sender_pub(32) to get MAC+ciphertext
        sender_pub = payload[1 : 1 + PUB_KEY_SIZE]
        encrypted = payload[1 + PUB_KEY_SIZE :]

        # Recipient computes shared secret from sender's pubkey
        secret = meshcore_shared_secret(dest_expanded, sender_pub)

        # Decrypt
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertIsNotNone(decrypted)

        # First 4 bytes are timestamp
        ts = struct.unpack_from("<I", decrypted, 0)[0]
        self.assertEqual(ts, 1700000000)

        # Remaining bytes are the message (may have trailing zeros from AES padding)
        msg = decrypted[4 : 4 + len(message)]
        self.assertEqual(msg, message)


class TestTxtMsg(unittest.TestCase):
    """Tests for TXT_MSG (encrypted text to a known contact)."""

    def test_txt_msg_structure(self):
        """TXT_MSG has correct wire format: dest_hash + src_hash + encrypted."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_txt_msg(TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, "Hello")

        # Wire: header(1) + path_len(1) + payload
        self.assertEqual(pkt[0] & 0x03, ROUTE_DIRECT)  # default
        self.assertEqual((pkt[0] >> 2) & 0x0F, PAYLOAD_TXT)
        self.assertEqual(pkt[1], 0)  # path_len

        payload = pkt[2:]
        # payload: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
        self.assertEqual(payload[0], dest_pub[0])  # dest_hash
        self.assertEqual(payload[1], TEST_SEED_PUB[0])  # src_hash

    def test_txt_msg_decrypt(self):
        """Recipient must be able to decrypt TXT_MSG."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_expanded = meshcore_expanded_key(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_txt_msg(
            TEST_SEED_EXPANDED,
            TEST_SEED_PUB,
            dest_pub,
            "Hello gr4-lora",
            timestamp=1700000000,
        )

        payload = pkt[2:]
        # Skip dest_hash(1) + src_hash(1) to get MAC+ciphertext
        encrypted = payload[2:]

        # Recipient computes shared secret from sender's pubkey
        secret = meshcore_shared_secret(dest_expanded, TEST_SEED_PUB)

        # Decrypt
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertIsNotNone(decrypted)
        assert decrypted is not None  # for type checker

        # First 4 bytes are timestamp
        ts = struct.unpack_from("<I", decrypted, 0)[0]
        self.assertEqual(ts, 1700000000)

        # Byte 4 is attempt number
        self.assertEqual(decrypted[4], 0)

        # Remaining bytes are the message text
        msg = decrypted[5 : 5 + len("Hello gr4-lora")]
        self.assertEqual(msg, b"Hello gr4-lora")

    def test_txt_msg_attempt_number(self):
        """Attempt number is encoded in plaintext byte 4."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_expanded = meshcore_expanded_key(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_txt_msg(
            TEST_SEED_EXPANDED,
            TEST_SEED_PUB,
            dest_pub,
            "retry",
            attempt=3,
        )

        payload = pkt[2:]
        encrypted = payload[2:]
        secret = meshcore_shared_secret(dest_expanded, TEST_SEED_PUB)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertIsNotNone(decrypted)
        assert decrypted is not None
        self.assertEqual(decrypted[4], 3)

    def test_txt_msg_flood_route(self):
        """TXT_MSG can use FLOOD routing explicitly."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_pub = bytes(sk2.verify_key)

        pkt = build_txt_msg(
            TEST_SEED_EXPANDED,
            TEST_SEED_PUB,
            dest_pub,
            "flood",
            route_type=ROUTE_FLOOD,
        )
        self.assertEqual(pkt[0] & 0x03, ROUTE_FLOOD)

    def test_txt_msg_symmetry(self):
        """Both parties compute the same shared secret for TXT_MSG."""
        from nacl.signing import SigningKey

        seed2 = os.urandom(32)
        sk2 = SigningKey(seed2)
        dest_expanded = meshcore_expanded_key(seed2)
        dest_pub = bytes(sk2.verify_key)

        # Alice sends to Bob
        pkt = build_txt_msg(TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, "from Alice")
        encrypted = pkt[2 + 2 :]  # skip header + path_len + dest_hash + src_hash

        # Bob decrypts using his key + Alice's pubkey
        secret = meshcore_shared_secret(dest_expanded, TEST_SEED_PUB)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)
        self.assertIsNotNone(decrypted)
        assert decrypted is not None
        self.assertIn(b"from Alice", decrypted)


class TestIdentity(unittest.TestCase):
    def test_create_and_reload(self):
        """Identity must persist across load/save cycles."""
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "identity.bin"
            exp1, pub1, seed1 = load_or_create_identity(path)
            self.assertTrue(path.exists())
            self.assertEqual(len(exp1), 64)
            self.assertEqual(len(pub1), 32)
            self.assertEqual(len(seed1), 32)

            exp2, pub2, seed2 = load_or_create_identity(path)
            self.assertEqual(exp1, exp2)
            self.assertEqual(pub1, pub2)
            self.assertEqual(seed1, seed2)

    def test_corrupted_file_regenerates(self):
        """Corrupted identity file must trigger regeneration."""
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "identity.bin"
            path.write_bytes(b"garbage")

            exp, pub, seed = load_or_create_identity(path)
            self.assertEqual(len(exp), 64)
            self.assertEqual(len(pub), 32)
            self.assertEqual(len(seed), 32)
            # File should be overwritten with valid identity: seed(32) + pub(32) = 64
            self.assertEqual(len(path.read_bytes()), 64)


class TestCborTxRequest(unittest.TestCase):
    def test_cbor_structure(self):
        """CBOR TX request must have correct structure."""
        import cbor2

        packet = b"\x11\x00" + b"\xaa" * 100
        cbor_msg = make_cbor_tx_request(packet, freq=869618000)
        decoded = cbor2.loads(cbor_msg)

        self.assertEqual(decoded["type"], "lora_tx")
        self.assertEqual(decoded["payload"], packet)
        self.assertEqual(decoded["freq"], 869618000)

    def test_cbor_dry_run(self):
        """Dry run flag must propagate to CBOR."""
        import cbor2

        packet = b"\x11\x00\xff"
        cbor_msg = make_cbor_tx_request(packet, dry_run=True)
        decoded = cbor2.loads(cbor_msg)

        self.assertTrue(decoded["dry_run"])


class TestContactUri(unittest.TestCase):
    """Tests for MeshCore contact URI generation (mobile app QR format)."""

    def test_contact_uri_format(self):
        """Contact URI uses meshcore://contact/add? format."""
        pub = bytes(range(32))
        uri = build_contact_uri(pub, "TestNode")
        self.assertTrue(uri.startswith("meshcore://contact/add?"))
        self.assertIn("name=TestNode", uri)
        self.assertIn(f"public_key={pub.hex()}", uri)
        self.assertIn("type=1", uri)

    def test_contact_uri_no_name(self):
        """Contact URI without a name."""
        pub = bytes(range(32))
        uri = build_contact_uri(pub, "")
        self.assertNotIn("name=", uri)
        self.assertIn(f"public_key={pub.hex()}", uri)

    def test_contact_uri_special_chars(self):
        """Contact URI with special characters in name."""
        pub = bytes(range(32))
        uri = build_contact_uri(pub, "My Node+1")
        self.assertNotIn(" ", uri)
        self.assertIn("My%20Node%2B1", uri)

    def test_contact_uri_custom_type(self):
        """Contact URI with repeater node type."""
        pub = bytes(range(32))
        uri = build_contact_uri(pub, "Relay", node_type=ADVERT_NODE_REPEATER)
        self.assertIn("type=2", uri)

    def test_bizcard_uri_is_hex_packet(self):
        """Biz card URI is meshcore:// + hex-encoded ADVERT packet."""
        pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name="TestNode")
        uri = build_bizcard_uri(pkt)
        self.assertTrue(uri.startswith("meshcore://"))
        recovered = bytes.fromhex(uri[11:])
        self.assertEqual(recovered, pkt)

    def test_bizcard_uri_roundtrip(self):
        """Biz card URI hex decodes back to original ADVERT packet."""
        pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name="Roundtrip")
        uri = build_bizcard_uri(pkt)
        recovered = bytes.fromhex(uri[11:])
        self.assertEqual(recovered[0] & 0x03, ROUTE_FLOOD)
        self.assertEqual((recovered[0] >> 2) & 0x0F, PAYLOAD_ADVERT)
        payload = recovered[2:]
        self.assertEqual(payload[:PUB_KEY_SIZE], TEST_SEED_PUB)

    def test_qr_code_generation(self):
        """QR code generates without error (output goes to terminal)."""
        import io
        import segno

        pub = bytes(range(32))
        uri = build_contact_uri(pub, "Test")
        qr = segno.make(uri)
        buf = io.BytesIO()
        qr.save(buf, kind="svg")
        self.assertGreater(buf.tell(), 0)


if __name__ == "__main__":
    unittest.main()
