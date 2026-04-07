#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
test_meshcore_integration.py -- Integration tests for MeshCore encode→decode.

Validates that meshcore_tx.py (encoder) and lora_decode_meshcore.py (decoder)
agree on wire format. Also tests the full CBOR pipeline: wrap packet in a
lora_frame CBOR message, pipe through the decoder, verify JSON output.

These tests run without hardware — they exercise the protocol layer only.
The PHY loopback is tested separately on B210 hardware via lora_trx.
"""

import io
import json
import os
import struct
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))
sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent)
)  # transitional — remove in Task 4

import cbor2

from lora_decode_meshcore import parse_meshcore, print_json
from meshcore_crypto import (
    CIPHER_MAC_SIZE,
    PUB_KEY_SIZE,
    SIGNATURE_SIZE,
    meshcore_encrypt_then_mac,
    meshcore_expanded_key,
    meshcore_mac_then_decrypt,
    meshcore_shared_secret,
)
from meshcore_tx import (
    ADVERT_HAS_LOCATION,
    ADVERT_HAS_NAME,
    ADVERT_NODE_CHAT,
    PAYLOAD_ADVERT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_TXT,
    ROUTE_DIRECT,
    ROUTE_FLOOD,
    build_advert,
    build_anon_req,
    build_txt_msg,
    build_wire_packet,
    make_header,
)
from nacl.signing import SigningKey, VerifyKey

# Deterministic test identity (same as test_meshcore_tx.py)
TEST_SEED = bytes.fromhex(
    "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
)
TEST_SEED_PUB = bytes.fromhex(
    "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
)
TEST_SEED_EXPANDED = meshcore_expanded_key(TEST_SEED)


def _make_lora_frame(payload: bytes, sync_word: int = 0x12) -> dict:
    """Wrap raw MeshCore bytes in a lora_frame dict (mimics FrameSink output)."""
    return {
        "type": "lora_frame",
        "payload": payload,
        "crc_valid": True,
        "phy": {
            "sf": 8,
            "bw": 62500,
            "cr": 4,
            "crc_valid": True,
            "sync_word": sync_word,
        },
        "payload_len": len(payload),
        "cr": 4,
        "is_downchirp": False,
        "seq": 1,
    }


# ---- ADVERT encode→decode ----


class TestAdvertRoundtrip(unittest.TestCase):
    """Verify build_advert() output is correctly parsed by parse_meshcore()."""

    def test_basic_advert(self):
        """ADVERT with name round-trips through encoder→decoder."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="TestNode")
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.version, 0)
        self.assertEqual(pkt.route_type, ROUTE_FLOOD)
        self.assertEqual(pkt.payload_type, PAYLOAD_ADVERT)
        self.assertFalse(pkt.has_transport)
        self.assertEqual(pkt.path_length, 0)
        self.assertEqual(pkt.path, b"")
        self.assertEqual(pkt.advert_name, "TestNode")
        self.assertEqual(pkt.advert_pubkey, TEST_SEED_PUB)
        self.assertEqual(pkt.advert_flags & 0x0F, ADVERT_NODE_CHAT)
        self.assertTrue(pkt.advert_flags & ADVERT_HAS_NAME)

    def test_advert_no_name(self):
        """Minimal ADVERT (no name) parses without error."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB)
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.payload_type, PAYLOAD_ADVERT)
        self.assertEqual(pkt.advert_pubkey, TEST_SEED_PUB)
        self.assertEqual(pkt.advert_name, "")

    def test_advert_with_location(self):
        """ADVERT with GPS coordinates round-trips correctly."""
        pkt_bytes = build_advert(
            TEST_SEED, TEST_SEED_PUB, name="GPS", lat=51.5074, lon=-0.1278
        )
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.advert_name, "GPS")
        self.assertTrue(pkt.advert_flags & ADVERT_HAS_LOCATION)
        self.assertTrue(pkt.advert_flags & ADVERT_HAS_NAME)

    def test_advert_timestamp_preserved(self):
        """Encoder timestamp is accessible in decoder's app_payload."""
        ts = 1700000000
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="TS", timestamp=ts)
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.advert_timestamp, ts)

    def test_advert_signature_verifies_after_decode(self):
        """Signature extracted by decoder verifies against the pubkey."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="SigCheck")

        # Raw parse: wire = header(1) + path_len(1) + app_payload
        # app_payload = pubkey(32) + ts(4) + sig(64) + appdata(N)
        payload = pkt_bytes[2:]  # skip header + path_len
        pub = payload[:PUB_KEY_SIZE]
        ts = payload[PUB_KEY_SIZE : PUB_KEY_SIZE + 4]
        sig = payload[PUB_KEY_SIZE + 4 : PUB_KEY_SIZE + 4 + SIGNATURE_SIZE]
        appdata = payload[PUB_KEY_SIZE + 4 + SIGNATURE_SIZE :]

        # Decoder should have extracted the same pubkey
        pkt = parse_meshcore(pkt_bytes)
        self.assertEqual(pkt.advert_pubkey, pub)

        # Verify signature
        vk = VerifyKey(pub)
        vk.verify(pub + ts + appdata, sig)  # raises on failure

    def test_advert_route_and_payload_names(self):
        """Decoder's human-readable names match expected values."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="Names")
        pkt = parse_meshcore(pkt_bytes)

        self.assertEqual(pkt.route_name, "FLOOD")
        self.assertEqual(pkt.payload_name, "ADVERT")


# ---- ANON_REQ encode→decode ----


class TestAnonReqRoundtrip(unittest.TestCase):
    """Verify build_anon_req() output is correctly parsed by parse_meshcore()."""

    def setUp(self):
        """Create a destination keypair for encryption tests."""
        self.dest_seed = os.urandom(32)
        self.dest_sk = SigningKey(self.dest_seed)
        self.dest_pub = bytes(self.dest_sk.verify_key)
        self.dest_expanded = meshcore_expanded_key(self.dest_seed)

    def test_basic_anon_req(self):
        """ANON_REQ round-trips: header, route, payload type."""
        pkt_bytes = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, b"Hello"
        )
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.version, 0)
        self.assertEqual(pkt.route_type, ROUTE_DIRECT)  # default is DIRECT
        self.assertEqual(pkt.payload_type, PAYLOAD_ANON_REQ)
        self.assertFalse(pkt.has_transport)
        self.assertEqual(pkt.path_length, 0)

    def test_anon_req_sender_pubkey(self):
        """Decoder can extract sender pubkey from ANON_REQ app_payload."""
        pkt_bytes = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, b"Sender"
        )
        pkt = parse_meshcore(pkt_bytes)

        # app_payload: dest_hash(1) + sender_pub(32) + MAC(2) + ciphertext
        self.assertGreater(len(pkt.app_payload), 1 + PUB_KEY_SIZE + CIPHER_MAC_SIZE)
        sender_pub = pkt.app_payload[1 : 1 + PUB_KEY_SIZE]
        self.assertEqual(sender_pub, TEST_SEED_PUB)

    def test_anon_req_decrypt_from_decoder_output(self):
        """Recipient decrypts message using fields extracted by decoder."""
        message = b"Secret message from gr4-lora"
        ts = 1700000000
        pkt_bytes = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, message, timestamp=ts
        )
        pkt = parse_meshcore(pkt_bytes)

        # Extract sender pubkey and encrypted payload from decoded packet
        sender_pub = pkt.app_payload[1 : 1 + PUB_KEY_SIZE]
        encrypted = pkt.app_payload[1 + PUB_KEY_SIZE :]

        # Recipient computes shared secret and decrypts
        secret = meshcore_shared_secret(self.dest_expanded, sender_pub)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)

        self.assertIsNotNone(decrypted)
        # First 4 bytes are timestamp
        decoded_ts = struct.unpack_from("<I", decrypted, 0)[0]
        self.assertEqual(decoded_ts, ts)
        # Rest is message (may have trailing zero padding)
        decoded_msg = decrypted[4 : 4 + len(message)]
        self.assertEqual(decoded_msg, message)

    def test_anon_req_route_names(self):
        """Decoder produces correct human-readable names."""
        pkt_bytes = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, b"X"
        )
        pkt = parse_meshcore(pkt_bytes)

        self.assertEqual(pkt.route_name, "DIRECT")  # default is DIRECT
        self.assertEqual(pkt.payload_name, "ANON_REQ")


# ---- TXT_MSG encode→decode ----


class TestTxtMsgRoundtrip(unittest.TestCase):
    """Verify build_txt_msg() output is correctly parsed by parse_meshcore()."""

    def setUp(self):
        """Create a destination keypair for encryption tests."""
        self.dest_seed = os.urandom(32)
        self.dest_sk = SigningKey(self.dest_seed)
        self.dest_pub = bytes(self.dest_sk.verify_key)
        self.dest_expanded = meshcore_expanded_key(self.dest_seed)

    def test_basic_txt_msg(self):
        """TXT_MSG round-trips: header, route, payload type."""
        pkt_bytes = build_txt_msg(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "Hello"
        )
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.version, 0)
        self.assertEqual(pkt.route_type, ROUTE_DIRECT)  # default is DIRECT
        self.assertEqual(pkt.payload_type, PAYLOAD_TXT)
        self.assertFalse(pkt.has_transport)
        self.assertEqual(pkt.path_length, 0)

    def test_txt_msg_hashes(self):
        """TXT_MSG app_payload starts with dest_hash + src_hash."""
        pkt_bytes = build_txt_msg(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "Hashes"
        )
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        # app_payload: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
        self.assertGreater(len(pkt.app_payload), 4)
        self.assertEqual(pkt.app_payload[0], self.dest_pub[0])  # dest_hash
        self.assertEqual(pkt.app_payload[1], TEST_SEED_PUB[0])  # src_hash

    def test_txt_msg_decrypt_from_decoder_output(self):
        """Recipient decrypts TXT_MSG using fields extracted by decoder."""
        text = "Secret message from gr4-lora"
        ts = 1700000000
        pkt_bytes = build_txt_msg(
            TEST_SEED_EXPANDED,
            TEST_SEED_PUB,
            self.dest_pub,
            text,
            timestamp=ts,
        )
        pkt = parse_meshcore(pkt_bytes)

        # Extract encrypted payload (skip dest_hash + src_hash)
        encrypted = pkt.app_payload[2:]

        # Recipient computes shared secret and decrypts
        secret = meshcore_shared_secret(self.dest_expanded, TEST_SEED_PUB)
        decrypted = meshcore_mac_then_decrypt(secret, encrypted)

        self.assertIsNotNone(decrypted)
        # First 4 bytes are timestamp
        decoded_ts = struct.unpack_from("<I", decrypted, 0)[0]
        self.assertEqual(decoded_ts, ts)
        # Byte 4 is attempt
        self.assertEqual(decrypted[4], 0)
        # Rest is message text
        decoded_msg = decrypted[5 : 5 + len(text)]
        self.assertEqual(decoded_msg, text.encode("utf-8"))

    def test_txt_msg_route_names(self):
        """Decoder produces correct human-readable names for TXT_MSG."""
        pkt_bytes = build_txt_msg(TEST_SEED_EXPANDED, TEST_SEED_PUB, self.dest_pub, "X")
        pkt = parse_meshcore(pkt_bytes)

        self.assertEqual(pkt.route_name, "DIRECT")
        self.assertEqual(pkt.payload_name, "TXT")


# ---- Transport codes ----


class TestTransportCodeRoundtrip(unittest.TestCase):
    """Verify packets with transport codes parse correctly."""

    def test_t_flood_transport_codes(self):
        """T_FLOOD (route 0) packet includes transport codes."""
        header = make_header(0x00, PAYLOAD_ADVERT)  # T_FLOOD
        payload = b"\xaa" * 10
        pkt_bytes = build_wire_packet(header, payload, transport_codes=(0xABCD, 0x1234))
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertTrue(pkt.has_transport)
        self.assertEqual(pkt.transport_code1, 0xABCD)
        self.assertEqual(pkt.transport_code2, 0x1234)
        self.assertEqual(pkt.app_payload, payload)

    def test_t_direct_transport_codes(self):
        """T_DIRECT (route 3) packet includes transport codes."""
        header = make_header(0x03, PAYLOAD_ADVERT)  # T_DIRECT
        payload = b"\xbb" * 5
        pkt_bytes = build_wire_packet(header, payload, transport_codes=(0x0001, 0x0002))
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertTrue(pkt.has_transport)
        self.assertEqual(pkt.transport_code1, 0x0001)
        self.assertEqual(pkt.transport_code2, 0x0002)

    def test_flood_no_transport(self):
        """FLOOD (route 1) packet has no transport codes."""
        header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        payload = b"\xcc" * 5
        pkt_bytes = build_wire_packet(header, payload)
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertFalse(pkt.has_transport)
        self.assertEqual(pkt.transport_code1, 0)
        self.assertEqual(pkt.transport_code2, 0)


# ---- Path ----


class TestPathRoundtrip(unittest.TestCase):
    """Verify packets with path hops parse correctly."""

    def test_with_path(self):
        """Packet with 3-hop path round-trips."""
        header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        payload = b"\xdd" * 8
        path = b"\x01\x02\x03"
        pkt_bytes = build_wire_packet(header, payload, path=path)
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.path_length, 3)
        self.assertEqual(pkt.path, path)
        self.assertEqual(pkt.app_payload, payload)


# ---- Full CBOR pipeline ----


class TestCborPipeline(unittest.TestCase):
    """Test the full CBOR encode→decode pipeline via subprocess."""

    def _run_decoder(
        self, cbor_data: bytes, extra_args: list[str] | None = None
    ) -> str:
        """Run lora_decode_meshcore.py with CBOR data on stdin."""
        scripts_dir = os.path.dirname(os.path.dirname(__file__))  # tests/.. -> scripts/
        cmd = [
            sys.executable,
            os.path.join(scripts_dir, "lora_decode_meshcore.py"),
            "--json",
        ]
        if extra_args:
            cmd.extend(extra_args)

        result = subprocess.run(
            cmd,
            input=cbor_data,
            capture_output=True,
            timeout=10,
            cwd=scripts_dir,
        )
        self.assertEqual(
            result.returncode, 0, f"Decoder failed: {result.stderr.decode()}"
        )
        return result.stdout.decode().strip()

    def test_advert_cbor_json_output(self):
        """ADVERT wrapped in CBOR lora_frame produces valid JSON output."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="CborTest")
        frame = _make_lora_frame(pkt_bytes)
        cbor_data = cbor2.dumps(frame)

        output = self._run_decoder(cbor_data)
        self.assertTrue(output, "Decoder produced no output")

        parsed = json.loads(output)
        self.assertEqual(parsed["crc_valid"], True)
        self.assertEqual(parsed["meshcore"]["version"], 0)
        self.assertEqual(parsed["meshcore"]["route"], "FLOOD")
        self.assertEqual(parsed["meshcore"]["payload_type"], "ADVERT")
        self.assertEqual(parsed["meshcore"]["advert_name"], "CborTest")
        self.assertEqual(parsed["meshcore"]["path_length"], 0)
        self.assertEqual(
            parsed["meshcore"]["advert_pubkey"],
            TEST_SEED_PUB.hex().upper(),
        )

    def test_anon_req_cbor_json_output(self):
        """ANON_REQ wrapped in CBOR lora_frame produces valid JSON output."""
        dest_seed = os.urandom(32)
        dest_sk = SigningKey(dest_seed)
        dest_pub = bytes(dest_sk.verify_key)

        pkt_bytes = build_anon_req(
            TEST_SEED_EXPANDED, TEST_SEED_PUB, dest_pub, b"CborMsg"
        )
        frame = _make_lora_frame(pkt_bytes)
        cbor_data = cbor2.dumps(frame)

        output = self._run_decoder(cbor_data)
        self.assertTrue(output, "Decoder produced no output")

        parsed = json.loads(output)
        self.assertEqual(parsed["meshcore"]["route"], "DIRECT")  # default is DIRECT
        self.assertEqual(parsed["meshcore"]["payload_type"], "ANON_REQ")
        self.assertGreater(parsed["meshcore"]["app_payload_len"], 0)

    def test_multiple_frames(self):
        """Multiple CBOR frames in sequence are all decoded."""
        frames = []
        for i in range(3):
            pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name=f"Multi{i}")
            frame = _make_lora_frame(pkt)
            frame["seq"] = i + 1
            frames.append(frame)

        # Concatenate CBOR Sequence (RFC 8742)
        cbor_data = b"".join(cbor2.dumps(f) for f in frames)

        output = self._run_decoder(cbor_data)
        lines = [line for line in output.splitlines() if line.strip()]
        self.assertEqual(len(lines), 3)

        for i, line in enumerate(lines):
            parsed = json.loads(line)
            self.assertEqual(parsed["meshcore"]["advert_name"], f"Multi{i}")
            self.assertEqual(parsed["seq"], i + 1)

    def test_non_meshcore_filtered(self):
        """Frames with wrong sync word are filtered out by default."""
        frame = _make_lora_frame(b"\x00" * 10, sync_word=0x34)
        cbor_data = cbor2.dumps(frame)

        output = self._run_decoder(cbor_data)
        self.assertEqual(output, "", "Non-MeshCore frame should be filtered")

    def test_crc_fail_still_decoded(self):
        """Frame with crc_valid=False is still decoded (decoder doesn't filter on CRC)."""
        pkt_bytes = build_advert(TEST_SEED, TEST_SEED_PUB, name="CrcFail")
        frame = _make_lora_frame(pkt_bytes)
        frame["crc_valid"] = False
        cbor_data = cbor2.dumps(frame)

        output = self._run_decoder(cbor_data)
        self.assertTrue(output, "CRC_FAIL frame should still be decoded")
        parsed = json.loads(output)
        self.assertEqual(parsed["crc_valid"], False)
        self.assertEqual(parsed["meshcore"]["advert_name"], "CrcFail")


# ---- Edge cases ----


class TestEdgeCases(unittest.TestCase):
    """Edge cases for the decode path."""

    def test_empty_payload(self):
        """Empty payload returns None."""
        pkt = parse_meshcore(b"")
        self.assertIsNone(pkt)

    def test_header_only(self):
        """Single-byte packet (header only) parses without crash."""
        pkt = parse_meshcore(b"\x11")
        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.route_type, ROUTE_FLOOD)
        self.assertEqual(pkt.payload_type, PAYLOAD_ADVERT)

    def test_truncated_advert(self):
        """Truncated ADVERT (header + path_len + partial pubkey) doesn't crash."""
        pkt_bytes = b"\x11\x00" + b"\xaa" * 10  # too short for full ADVERT
        pkt = parse_meshcore(pkt_bytes)
        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.payload_type, PAYLOAD_ADVERT)
        # Name should be empty since ADVERT parsing fails gracefully
        self.assertEqual(pkt.advert_name, "")

    def test_max_path_length(self):
        """Packet with path_length=64 (max) parses correctly."""
        header = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        path = bytes(range(64))
        payload = b"\xee" * 4
        pkt_bytes = build_wire_packet(header, payload, path=path)
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.path_length, 64)
        self.assertEqual(pkt.path, path)

    def test_oversized_path_rejected(self):
        """Packet claiming path_length > 64 treats rest as app_payload."""
        # Manually construct: header + path_len(65) + data
        pkt_bytes = b"\x11" + b"\x41" + b"\xff" * 20  # path_len=65
        pkt = parse_meshcore(pkt_bytes)

        self.assertIsNotNone(pkt)
        self.assertEqual(pkt.path_length, 65)
        self.assertEqual(pkt.path, b"")  # rejected, path not populated


if __name__ == "__main__":
    unittest.main()
