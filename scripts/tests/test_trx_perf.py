#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Tests for trx_perf.py pure functions.

Run:  .venv/bin/python3 -m unittest scripts/test_trx_perf.py -v
"""

from __future__ import annotations

import os
import sys
import time
import unittest

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))
sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent)
)  # transitional — remove in Task 4
from trx_perf import FrameStats, emit_frame, emit_summary, format_bw


# ---- format_bw ----


class TestFormatBw(unittest.TestCase):
    def test_62500(self):
        self.assertEqual(format_bw(62500), "62.5k")

    def test_125000(self):
        self.assertEqual(format_bw(125000), "125k")

    def test_250000(self):
        self.assertEqual(format_bw(250000), "250k")

    def test_500000(self):
        self.assertEqual(format_bw(500000), "500k")

    def test_sub_1000(self):
        """Values below 1000 are returned as integer strings."""
        self.assertEqual(format_bw(500), "500")

    def test_float_input(self):
        self.assertEqual(format_bw(62500.0), "62.5k")

    def test_1000_exact(self):
        self.assertEqual(format_bw(1000), "1k")


# ---- emit_frame ----


class TestEmitFrame(unittest.TestCase):
    def _make_msg(self, **overrides):
        msg = {
            "type": "lora_frame",
            "seq": 1,
            "crc_valid": True,
            "phy": {"sf": 8, "bw": 62500, "snr_db": 5.0},
            "payload": b"\x12\x00\xbb",
        }
        msg.update(overrides)
        return msg

    def test_basic_frame(self):
        line = emit_frame(self._make_msg())
        self.assertIn("FRAME seq=1", line)
        self.assertIn("SF8", line)
        self.assertIn("BW62.5k", line)
        self.assertIn("CRC=OK", line)
        self.assertIn("SNR=5.0", line)
        self.assertIn("len=3", line)

    def test_crc_fail(self):
        line = emit_frame(self._make_msg(crc_valid=False))
        self.assertIn("CRC=FAIL", line)

    def test_channel_freq(self):
        msg = self._make_msg()
        msg["phy"]["channel_freq"] = 869618000.0
        line = emit_frame(msg)
        self.assertIn("freq=869.618", line)

    def test_decode_bw(self):
        msg = self._make_msg()
        msg["phy"]["decode_bw"] = 125000
        line = emit_frame(msg)
        self.assertIn("dec_bw=125k", line)

    def test_cfo_fields(self):
        msg = self._make_msg()
        msg["phy"]["cfo_int"] = -3
        msg["phy"]["cfo_frac"] = 0.214
        line = emit_frame(msg)
        self.assertIn("cfo_int=-3", line)
        self.assertIn("cfo_frac=0.214", line)

    def test_sfo_field(self):
        msg = self._make_msg()
        msg["phy"]["sfo_hat"] = -0.001
        line = emit_frame(msg)
        self.assertIn("sfo=-0.001", line)

    def test_no_payload(self):
        """Empty payload omits len= field."""
        line = emit_frame(self._make_msg(payload=b""))
        self.assertNotIn("len=", line)

    def test_no_phy(self):
        """Missing phy dict doesn't crash."""
        msg = {"type": "lora_frame", "seq": 5, "crc_valid": False}
        line = emit_frame(msg)
        self.assertIn("FRAME seq=5", line)
        self.assertIn("CRC=FAIL", line)

    def test_bw_zero_omits_bw(self):
        """BW=0 should produce empty BW string."""
        msg = self._make_msg()
        msg["phy"]["bw"] = 0
        line = emit_frame(msg)
        self.assertNotIn("BW", line)


# ---- emit_summary ----


class TestEmitSummary(unittest.TestCase):
    def test_empty_stats(self):
        stats = FrameStats()
        self.assertEqual(emit_summary(stats), "SUMMARY frames=0")

    def test_with_data(self):
        stats = FrameStats(
            frame_count=10,
            crc_ok=8,
            crc_fail=2,
            snr_values=[10.0, 20.0],
            sf_counts={8: 7, 12: 3},
        )
        line = emit_summary(stats)
        self.assertIn("frames=10", line)
        self.assertIn("crc_ok=8", line)
        self.assertIn("crc_fail=2", line)
        self.assertIn("avg_snr=15.0", line)
        self.assertIn("SF8:7", line)
        self.assertIn("SF12:3", line)


# ---- is_duplicate ----


class TestIsDuplicate(unittest.TestCase):
    """Test the is_duplicate() closure via the module-level recent_hashes dict.

    is_duplicate() is a closure inside main(), so we cannot import it directly.
    Instead, we reconstruct its logic here using the same algorithm to verify
    the dedup semantics. The actual closure accesses a local `recent_hashes`
    dict; we simulate this faithfully.
    """

    def setUp(self):
        """Create a fresh is_duplicate function mirroring the production logic."""
        self.recent_hashes: dict[int, float] = {}

        def _is_duplicate(phash: int) -> bool:
            now = time.monotonic()
            stale = [k for k, t in self.recent_hashes.items() if now - t > 1.0]
            for k in stale:
                del self.recent_hashes[k]
            if phash in self.recent_hashes:
                return True
            self.recent_hashes[phash] = now
            return False

        self.is_duplicate = _is_duplicate

    def test_first_seen_is_not_duplicate(self):
        self.assertFalse(self.is_duplicate(12345))

    def test_second_seen_is_duplicate(self):
        self.assertFalse(self.is_duplicate(12345))
        self.assertTrue(self.is_duplicate(12345))

    def test_different_hashes_are_independent(self):
        self.assertFalse(self.is_duplicate(100))
        self.assertFalse(self.is_duplicate(200))
        self.assertTrue(self.is_duplicate(100))
        self.assertTrue(self.is_duplicate(200))

    def test_stale_entries_evicted(self):
        """Entries older than 1s are evicted."""
        self.assertFalse(self.is_duplicate(999))
        # Manually backdate the entry
        self.recent_hashes[999] = time.monotonic() - 2.0
        # Should be evicted and treated as new
        self.assertFalse(self.is_duplicate(999))


# ---- FrameStats dedup_count ----


class TestFrameStatsDedup(unittest.TestCase):
    """Verify dedup_count doesn't inflate frame_count."""

    def test_dedup_count_independent_of_frame_count(self):
        stats = FrameStats()
        # Simulate 3 frames with 2 dedup'd
        stats.frame_count += 1
        stats.frame_count += 1
        stats.frame_count += 1
        stats.dedup_count += 2
        self.assertEqual(stats.frame_count, 3)
        self.assertEqual(stats.dedup_count, 2)

    def test_fresh_stats(self):
        stats = FrameStats()
        self.assertEqual(stats.frame_count, 0)
        self.assertEqual(stats.dedup_count, 0)
        self.assertEqual(stats.crc_ok, 0)
        self.assertEqual(stats.crc_fail, 0)
        self.assertEqual(stats.snr_values, [])
        self.assertEqual(stats.sf_counts, {})

    def test_dedup_does_not_change_frame_count(self):
        """Simulates the main loop logic: deduped frames bump dedup_count, not frame_count."""
        stats = FrameStats()
        recent: dict[int, float] = {}

        def process_frame(phash: int):
            now = time.monotonic()
            if phash in recent and now - recent[phash] < 1.0:
                stats.dedup_count += 1
                return  # skip
            recent[phash] = now
            stats.frame_count += 1

        process_frame(1001)  # new
        process_frame(1002)  # new
        process_frame(1001)  # dup
        process_frame(1003)  # new
        process_frame(1002)  # dup

        self.assertEqual(stats.frame_count, 3)
        self.assertEqual(stats.dedup_count, 2)


if __name__ == "__main__":
    unittest.main()
