#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
test_lora_mqtt.py -- Unit tests for lora_mqtt.py.

Tests the frame_to_meshcore_json conversion function.

Run:  python3 -m unittest scripts/test_lora_mqtt.py -v
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from lora_mqtt import frame_to_meshcore_json

# Deterministic test identity
TEST_SEED = bytes.fromhex(
    "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
)
TEST_SEED_PUB = bytes.fromhex(
    "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
)


def _make_frame(
    payload: bytes,
    crc_valid: bool = True,
    snr_db: float = 5.0,
    ts: str = "2026-02-24T12:00:00.000Z",
) -> dict:
    return {
        "type": "lora_frame",
        "seq": 1,
        "ts": ts,
        "payload": payload,
        "crc_valid": crc_valid,
        "phy": {"sf": 8, "bw": 62500, "cr": 4, "sync_word": 0x12, "snr_db": snr_db},
    }


def _build_advert_payload() -> bytes:
    """Build a minimal ADVERT payload (wire format, without LoRa PHY)."""
    from meshcore_tx import build_advert

    pkt = build_advert(TEST_SEED, TEST_SEED_PUB, name="Test")
    return pkt


class TestFrameToMeshcoreJson(unittest.TestCase):
    def test_advert_conversion(self):
        payload = _build_advert_payload()
        msg = _make_frame(payload)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["raw"], payload.hex().upper())
        self.assertEqual(result["len"], len(payload))
        self.assertEqual(result["type"], 4)  # ADVERT
        self.assertEqual(result["route"], 1)  # FLOOD
        self.assertGreater(result["payload_len"], 0)
        self.assertEqual(result["snr"], 5.0)
        self.assertIn("timestamp", result)

    def test_crc_fail_returns_none(self):
        payload = _build_advert_payload()
        msg = _make_frame(payload, crc_valid=False)
        result = frame_to_meshcore_json(msg)
        self.assertIsNone(result)

    def test_empty_payload_returns_none(self):
        msg = _make_frame(b"")
        result = frame_to_meshcore_json(msg)
        self.assertIsNone(result)

    def test_no_payload_key_returns_none(self):
        msg = {"type": "lora_frame", "crc_valid": True, "phy": {}}
        result = frame_to_meshcore_json(msg)
        self.assertIsNone(result)

    def test_too_short_payload_returns_none(self):
        msg = _make_frame(b"\x11")
        result = frame_to_meshcore_json(msg)
        self.assertIsNone(result)

    def test_direct_txt_route(self):
        # DIRECT (route=2) + TXT (ptype=2)
        # header = 0x02 | (0x02 << 2) = 0x0A
        payload = b"\x0a\x00" + b"\xaa" * 20
        msg = _make_frame(payload)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["type"], 2)  # TXT
        self.assertEqual(result["route"], 2)  # DIRECT

    def test_t_flood_with_transport(self):
        # T_FLOOD (route=0) + REQ (ptype=0)
        # header = 0x00, transport=4 bytes, path_len=0
        payload = b"\x00" + b"\x01\x02\x03\x04" + b"\x00" + b"\xff" * 10
        msg = _make_frame(payload)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["route"], 0)  # T_FLOOD
        self.assertEqual(result["type"], 0)  # REQ
        self.assertEqual(result["payload_len"], 10)

    def test_snr_and_rssi(self):
        payload = _build_advert_payload()
        msg = _make_frame(payload, snr_db=10.5)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["snr"], 10.5)
        self.assertAlmostEqual(result["rssi"], -120.0 + 10.5)

    def test_timestamp_preserved(self):
        payload = _build_advert_payload()
        ts = "2026-02-24T15:30:00.123Z"
        msg = _make_frame(payload, ts=ts)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["timestamp"], ts)

    def test_missing_snr_defaults_zero(self):
        payload = _build_advert_payload()
        msg = _make_frame(payload)
        msg["phy"] = {"sf": 8}  # no snr_db
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["snr"], 0.0)

    def test_anon_req_type(self):
        # DIRECT (route=2) + ANON_REQ (ptype=7)
        # header = 0x02 | (0x07 << 2) = 0x02 | 0x1C = 0x1E
        payload = b"\x1e\x00" + b"\xbb" * 50
        msg = _make_frame(payload)
        result = frame_to_meshcore_json(msg)

        self.assertIsNotNone(result)
        self.assertEqual(result["type"], 7)  # ANON_REQ
        self.assertEqual(result["route"], 2)  # DIRECT


if __name__ == "__main__":
    unittest.main()
