#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Tests for lora_mon.py.

Run:  python3 -m unittest scripts/test_lora_mon.py -v
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
import unittest
from typing import Any

import cbor2

sys.path.insert(0, os.path.dirname(__file__))
import lora_mon


# ---- Helpers ----


def make_frame(
    payload: bytes = b"Hello",
    sf: int = 8,
    bw: int = 62500,
    cr: int = 4,
    crc_valid: bool = True,
    sync_word: int = 0x12,
    seq: int = 1,
    ts: str = "2026-02-17T12:00:00.123Z",
    **extra: Any,
) -> dict:
    msg = {
        "type": "lora_frame",
        "seq": seq,
        "ts": ts,
        "payload": payload,
        "crc_valid": crc_valid,
        "phy": {"sf": sf, "bw": bw, "cr": cr, "sync_word": sync_word},
    }
    msg.update(extra)
    return msg


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ---- format_frame / format_frame_json ----


class TestFormatFrame(unittest.TestCase):
    def test_text_contains_essentials(self):
        text = lora_mon.format_frame(make_frame())
        self.assertIn("#1", text)
        self.assertIn("CRC_OK", text)
        self.assertIn("SF8", text)

    def test_crc_fail(self):
        text = lora_mon.format_frame(make_frame(crc_valid=False))
        self.assertIn("CRC_FAIL", text)

    def test_compact_is_single_line(self):
        text = lora_mon.format_frame(make_frame(), compact=True)
        self.assertEqual(text.count("\n"), 0)

    def test_sync_word_label(self):
        self.assertIn("MeshCore", lora_mon.format_frame(make_frame(sync_word=0x12)))
        self.assertIn("Meshtastic", lora_mon.format_frame(make_frame(sync_word=0x2B)))
        self.assertIn("LoRaWAN", lora_mon.format_frame(make_frame(sync_word=0x34)))
        self.assertIn("0xFF", lora_mon.format_frame(make_frame(sync_word=0xFF)))

    def test_json_roundtrip(self):
        text = lora_mon.format_frame_json(make_frame(seq=42))
        obj = json.loads(text)
        self.assertEqual(obj["seq"], 42)
        self.assertIn("payload_hex", obj)


# ---- Stats ----


class TestStats(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(lora_mon.Stats().summary(), "No frames received")

    def test_counts(self):
        s = lora_mon.Stats()
        s.update(make_frame(crc_valid=True))
        s.update(make_frame(crc_valid=False))
        s.update(make_frame(crc_valid=True))
        self.assertEqual(s.total, 3)
        self.assertEqual(s.crc_ok, 2)
        self.assertEqual(s.crc_fail, 1)


# ---- UDP integration (single send/receive round) ----


class TestUdpReceive(unittest.TestCase):
    def test_receive_and_format(self):
        port = find_free_port()
        results = []

        def receiver():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(2.0)
            sock.bind(("127.0.0.1", port))
            try:
                data, _ = sock.recvfrom(65536)
                msg = cbor2.loads(data)
                results.append(lora_mon.format_frame(msg))
            except socket.timeout:
                pass
            finally:
                sock.close()

        t = threading.Thread(target=receiver, daemon=True)
        t.start()
        time.sleep(0.05)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cbor2.dumps(make_frame(seq=7)), ("127.0.0.1", port))
        sock.close()

        t.join(timeout=3)
        self.assertEqual(len(results), 1)
        self.assertIn("#7", results[0])


if __name__ == "__main__":
    unittest.main()
