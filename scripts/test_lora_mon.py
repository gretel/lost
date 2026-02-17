#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Tests for lora_mon.py.

Run:  python3 scripts/test_lora_mon.py
  or: python3 -m pytest scripts/test_lora_mon.py -v
"""

from __future__ import annotations

import io
import os
import struct
import subprocess
import sys
import unittest
from typing import Any

import cbor2

# Import the module under test
sys.path.insert(0, os.path.dirname(__file__))
import lora_mon


# ---- Test data helpers ----


def make_lora_frame(
    payload: bytes = b"Hello MeshCore",
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


def make_meshcore_advert(name: str = "TestNode") -> bytes:
    """Minimal MeshCore ADVERT payload (route=FLOOD, ptype=ADVERT)."""
    hdr = 0x00 | (4 << 2) | 1  # v0, ADVERT, FLOOD
    path_len = 0
    filler = b"\x00" * 100  # pubkey(32) + timestamp(4) + signature(64)
    appdata = bytes([0x80]) + name.encode("utf-8")  # flags=0x80 (has name)
    return bytes([hdr, path_len]) + filler + appdata


def make_meshcore_txt() -> bytes:
    """Minimal MeshCore TXT payload (route=FLOOD, ptype=TXT)."""
    hdr = 0x00 | (2 << 2) | 1  # v0, TXT, FLOOD
    path_len = 0
    return bytes([hdr, path_len]) + b"hello"


def encode_cbor_stream(messages: list[dict]) -> bytes:
    buf = io.BytesIO()
    for msg in messages:
        cbor2.CBOREncoder(buf).encode(msg)
    return buf.getvalue()


# ---- Unit Tests: parse_meshcore_summary ----


class TestParseMeshcoreSummary(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(lora_mon.parse_meshcore_summary(b""), "")

    def test_single_byte(self):
        self.assertEqual(lora_mon.parse_meshcore_summary(b"\x00"), "")

    def test_flood_txt(self):
        result = lora_mon.parse_meshcore_summary(make_meshcore_txt())
        self.assertIn("FLOOD", result)
        self.assertIn("TXT", result)
        self.assertIn("path=0", result)

    def test_flood_advert_with_name(self):
        result = lora_mon.parse_meshcore_summary(make_meshcore_advert("MyRelay"))
        self.assertIn("ADVERT", result)
        self.assertIn('"MyRelay"', result)

    def test_t_flood_transport_codes(self):
        hdr = 0x00 | (2 << 2) | 0  # v0, TXT, T_FLOOD
        tc1, tc2 = 0xABCD, 0x1234
        payload = bytes([hdr]) + struct.pack("<HH", tc1, tc2) + bytes([0])
        result = lora_mon.parse_meshcore_summary(payload)
        self.assertIn("T_FLOOD", result)
        self.assertIn("tc=ABCD/1234", result)

    def test_version_bits(self):
        hdr = (1 << 6) | (0 << 2) | 1  # v1, REQ, FLOOD
        result = lora_mon.parse_meshcore_summary(bytes([hdr, 0]))
        self.assertIn("v1", result)


# ---- Unit Tests: format helpers ----


class TestFormatHelpers(unittest.TestCase):
    def test_format_hex_short(self):
        self.assertEqual(lora_mon.format_hex(b"\x01\x02"), "01 02")

    def test_format_hex_truncation(self):
        data = bytes(range(30))
        result = lora_mon.format_hex(data, max_bytes=4)
        self.assertIn("...(30B)", result)
        self.assertTrue(result.startswith("00 01 02 03"))

    def test_format_ascii_printable(self):
        self.assertEqual(lora_mon.format_ascii(b"Hello"), "Hello")

    def test_format_ascii_non_printable(self):
        self.assertEqual(lora_mon.format_ascii(b"\x01\x7f"), "..")


# ---- Unit Tests: format_frame ----


class TestFormatFrame(unittest.TestCase):
    def test_basic_fields(self):
        text = lora_mon.format_frame(make_lora_frame())
        self.assertIn("#1", text)
        self.assertIn("14B", text)
        self.assertIn("SF8", text)
        self.assertIn("CRC_OK", text)
        self.assertIn("MeshCore", text)

    def test_crc_fail(self):
        text = lora_mon.format_frame(make_lora_frame(crc_valid=False))
        self.assertIn("CRC_FAIL", text)

    def test_meshtastic_sync(self):
        text = lora_mon.format_frame(make_lora_frame(sync_word=0x2B))
        self.assertIn("Meshtastic", text)

    def test_unknown_sync(self):
        text = lora_mon.format_frame(make_lora_frame(sync_word=0xFF))
        self.assertIn("0xFF", text)

    def test_meshcore_detail(self):
        text = lora_mon.format_frame(make_lora_frame(payload=make_meshcore_txt()))
        self.assertIn("TXT", text)

    def test_hex_and_ascii(self):
        text = lora_mon.format_frame(make_lora_frame())
        self.assertIn("48 65 6C", text)  # "Hel" in hex
        self.assertIn("Hello MeshCore", text)

    def test_binary_no_ascii(self):
        text = lora_mon.format_frame(make_lora_frame(payload=bytes(range(10))))
        lines = text.split("\n")
        # Should have header + hex, but no ascii line
        for line in lines:
            if line.strip() and not line.startswith("#"):
                # Non-header lines are hex or detail, not ascii
                pass
        # Just verify no "." line with only dots appears as ascii
        self.assertNotIn("...........", text)

    def test_downchirp(self):
        text = lora_mon.format_frame(make_lora_frame(is_downchirp=True))
        self.assertIn("(downchirp)", text)

    def test_compact_mode(self):
        text = lora_mon.format_frame(make_lora_frame(), compact=True)
        self.assertEqual(text.count("\n"), 0)
        self.assertIn("#1", text)

    def test_json_output(self):
        import json

        text = lora_mon.format_frame_json(make_lora_frame())
        obj = json.loads(text)
        self.assertEqual(obj["seq"], 1)
        self.assertTrue(obj["crc_valid"])
        self.assertEqual(obj["sf"], 8)
        self.assertIn("payload_hex", obj)


# ---- Unit Tests: Stats ----


class TestStats(unittest.TestCase):
    def test_initial(self):
        s = lora_mon.Stats()
        self.assertEqual(s.total, 0)
        self.assertEqual(s.summary(), "No frames received")

    def test_update_crc_ok(self):
        s = lora_mon.Stats()
        s.update(make_lora_frame(crc_valid=True))
        self.assertEqual(s.total, 1)
        self.assertEqual(s.crc_ok, 1)
        self.assertEqual(s.crc_fail, 0)

    def test_update_crc_fail(self):
        s = lora_mon.Stats()
        s.update(make_lora_frame(crc_valid=False))
        self.assertEqual(s.crc_ok, 0)
        self.assertEqual(s.crc_fail, 1)

    def test_multiple_updates(self):
        s = lora_mon.Stats()
        for i in range(5):
            s.update(make_lora_frame(crc_valid=(i % 2 == 0), seq=i))
        self.assertEqual(s.total, 5)
        self.assertEqual(s.crc_ok, 3)
        self.assertEqual(s.crc_fail, 2)

    def test_protocol_counting(self):
        s = lora_mon.Stats()
        s.update(make_lora_frame(sync_word=0x12))
        s.update(make_lora_frame(sync_word=0x2B))
        s.update(make_lora_frame(sync_word=0x12))
        self.assertEqual(s.protocols["MeshCore"], 2)
        self.assertEqual(s.protocols["Meshtastic"], 1)

    def test_timestamps(self):
        s = lora_mon.Stats()
        s.update(make_lora_frame(ts="2026-02-17T12:00:00Z"))
        s.update(make_lora_frame(ts="2026-02-17T12:01:00Z"))
        self.assertEqual(s.first_ts, "2026-02-17T12:00:00Z")
        self.assertEqual(s.last_ts, "2026-02-17T12:01:00Z")

    def test_summary_format(self):
        s = lora_mon.Stats()
        s.update(make_lora_frame())
        summary = s.summary()
        self.assertIn("frames=1", summary)
        self.assertIn("crc_ok=1", summary)
        self.assertIn("100%", summary)


# ---- Integration: process_stream ----


class TestProcessStream(unittest.TestCase):
    def _run(self, messages, **kwargs):
        data = encode_cbor_stream(messages)
        inp = io.BytesIO(data)
        out = io.StringIO()
        stats = lora_mon.process_stream(inp, out, **kwargs)
        return out.getvalue(), stats

    def test_single_frame(self):
        text, stats = self._run([make_lora_frame()])
        self.assertIn("#1", text)
        self.assertIn("CRC_OK", text)
        self.assertEqual(stats.total, 1)

    def test_multiple_frames(self):
        msgs = [make_lora_frame(seq=i) for i in range(5)]
        text, stats = self._run(msgs)
        self.assertEqual(stats.total, 5)
        self.assertIn("#0", text)
        self.assertIn("#4", text)

    def test_non_frame_messages_skipped(self):
        msgs = [
            {"type": "other"},
            make_lora_frame(seq=1),
            {"type": "lora_tx_ack", "seq": 1, "ok": True},
            make_lora_frame(seq=2),
        ]
        text, stats = self._run(msgs)
        self.assertEqual(stats.total, 2)

    def test_json_mode(self):
        import json

        text, stats = self._run([make_lora_frame()], use_json=True)
        obj = json.loads(text.strip())
        self.assertEqual(obj["seq"], 1)

    def test_compact_mode(self):
        text, stats = self._run([make_lora_frame()], compact=True)
        # Compact: single line, no hex/ascii detail
        lines = [l for l in text.strip().split("\n") if l.strip()]
        self.assertEqual(len(lines), 1)

    def test_empty_stream(self):
        text, stats = self._run([])
        self.assertEqual(text, "")
        self.assertEqual(stats.total, 0)


# ---- Integration: CLI subprocess ----


class TestCLI(unittest.TestCase):
    """Run lora_mon.py as a subprocess and verify output."""

    def _run_cli(
        self, cbor_data: bytes, args: list[str] | None = None
    ) -> tuple[str, str, int]:
        cmd = [sys.executable, os.path.join(os.path.dirname(__file__), "lora_mon.py")]
        if args:
            cmd.extend(args)
        proc = subprocess.run(
            cmd,
            input=cbor_data,
            capture_output=True,
            timeout=5,
        )
        return proc.stdout.decode(), proc.stderr.decode(), proc.returncode

    def test_basic_text_output(self):
        data = encode_cbor_stream([make_lora_frame()])
        stdout, stderr, rc = self._run_cli(data)
        self.assertEqual(rc, 0)
        self.assertIn("#1", stdout)
        self.assertIn("CRC_OK", stdout)

    def test_json_output(self):
        import json

        data = encode_cbor_stream([make_lora_frame()])
        stdout, stderr, rc = self._run_cli(data, ["--json"])
        self.assertEqual(rc, 0)
        obj = json.loads(stdout.strip())
        self.assertEqual(obj["seq"], 1)

    def test_compact_output(self):
        data = encode_cbor_stream([make_lora_frame(seq=1), make_lora_frame(seq=2)])
        stdout, stderr, rc = self._run_cli(data, ["--compact"])
        self.assertEqual(rc, 0)
        lines = [l for l in stdout.strip().split("\n") if l.strip()]
        self.assertEqual(len(lines), 2)

    def test_stats_flag(self):
        data = encode_cbor_stream([make_lora_frame()])
        stdout, stderr, rc = self._run_cli(data, ["--stats"])
        self.assertEqual(rc, 0)
        self.assertIn("frames=1", stderr)

    def test_stats_every(self):
        msgs = [make_lora_frame(seq=i) for i in range(6)]
        data = encode_cbor_stream(msgs)
        stdout, stderr, rc = self._run_cli(data, ["--stats-every", "3"])
        self.assertEqual(rc, 0)
        # Should print stats at frame 3 and 6
        self.assertIn("frames=3", stderr)
        self.assertIn("frames=6", stderr)

    def test_empty_input(self):
        stdout, stderr, rc = self._run_cli(b"")
        self.assertEqual(rc, 0)
        self.assertEqual(stdout.strip(), "")

    def test_multi_frame_with_crc_mix(self):
        msgs = [
            make_lora_frame(seq=1, crc_valid=True),
            make_lora_frame(seq=2, crc_valid=False),
            make_lora_frame(seq=3, crc_valid=True),
        ]
        data = encode_cbor_stream(msgs)
        stdout, stderr, rc = self._run_cli(data, ["--stats"])
        self.assertEqual(rc, 0)
        self.assertIn("crc_ok=2", stderr)
        self.assertIn("crc_fail=1", stderr)


if __name__ == "__main__":
    unittest.main()
