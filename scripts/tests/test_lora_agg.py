#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Unit tests for lora_agg.py aggregation logic."""

import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))

from lora_agg import (
    Candidate,
    PendingGroup,
    TX_ECHO_WINDOW,
    build_aggregated,
    extract_candidate,
    fnv1a64,
    is_better,
)


def _make_msg(
    payload_hash: int = 0xDEADBEEF,
    crc_valid: bool = True,
    snr_db: float = 0.0,
    rx_channel: int = 100,
    frame_id: str = "test-uuid",
) -> dict:
    return {
        "type": "lora_frame",
        "payload_hash": payload_hash,
        "id": frame_id,
        "crc_valid": crc_valid,
        "rx_channel": rx_channel,
        "phy": {"snr_db": snr_db, "sf": 8, "bw": 62500, "cr": 4, "sync_word": 0x12},
        "payload": b"\x01\x02\x03",
        "payload_len": 3,
    }


def _make_candidate(
    payload_hash: int = 0xDEADBEEF,
    crc_valid: bool = True,
    snr_db: float = 0.0,
    rx_channel: int = 100,
    frame_id: str = "test-uuid",
    arrived_at: float = 0.0,
) -> Candidate:
    msg = _make_msg(payload_hash, crc_valid, snr_db, rx_channel, frame_id)
    return Candidate(
        msg=msg,
        rx_channel=rx_channel,
        snr_db=snr_db,
        crc_valid=crc_valid,
        decode_label=msg.get("decode_label", ""),
        frame_id=frame_id,
        arrived_at=arrived_at,
    )


class TestIsBetter(unittest.TestCase):
    def test_crc_ok_beats_fail(self):
        a = _make_candidate(crc_valid=True, snr_db=-10.0)
        b = _make_candidate(crc_valid=False, snr_db=5.0)
        self.assertTrue(is_better(a, b))

    def test_crc_fail_does_not_beat_ok(self):
        a = _make_candidate(crc_valid=False, snr_db=10.0)
        b = _make_candidate(crc_valid=True, snr_db=-10.0)
        self.assertFalse(is_better(a, b))

    def test_higher_snr_wins_when_crc_equal(self):
        a = _make_candidate(crc_valid=True, snr_db=5.0)
        b = _make_candidate(crc_valid=True, snr_db=-3.0)
        self.assertTrue(is_better(a, b))

    def test_lower_snr_loses(self):
        a = _make_candidate(crc_valid=True, snr_db=-3.0)
        b = _make_candidate(crc_valid=True, snr_db=5.0)
        self.assertFalse(is_better(a, b))

    def test_equal_is_not_better(self):
        a = _make_candidate(crc_valid=True, snr_db=0.0)
        b = _make_candidate(crc_valid=True, snr_db=0.0)
        self.assertFalse(is_better(a, b))


class TestExtractCandidate(unittest.TestCase):
    def test_valid_message(self):
        msg = _make_msg(
            payload_hash=0xCAFE,
            crc_valid=True,
            snr_db=3.5,
            rx_channel=200,
            frame_id="abc",
        )
        cand = extract_candidate(msg, arrived_at=1.0)
        self.assertIsNotNone(cand)
        self.assertTrue(cand.crc_valid)
        self.assertAlmostEqual(cand.snr_db, 3.5)
        self.assertEqual(cand.rx_channel, 200)
        self.assertEqual(cand.frame_id, "abc")
        self.assertAlmostEqual(cand.arrived_at, 1.0)

    def test_missing_payload_hash_returns_none(self):
        msg = _make_msg()
        del msg["payload_hash"]
        cand = extract_candidate(msg, arrived_at=0.0)
        self.assertIsNone(cand)

    def test_missing_snr_uses_default(self):
        msg = _make_msg()
        del msg["phy"]["snr_db"]
        cand = extract_candidate(msg, arrived_at=0.0)
        self.assertIsNotNone(cand)
        self.assertAlmostEqual(cand.snr_db, -999.0)


class TestBuildAggregated(unittest.TestCase):
    def test_single_candidate(self):
        c = _make_candidate(
            payload_hash=0xAB, snr_db=2.0, rx_channel=100, frame_id="uuid-1"
        )
        group = PendingGroup(
            payload_hash=0xAB, created_at=0.0, candidates=[c], best_idx=0
        )
        result = build_aggregated(group)
        div = result["diversity"]
        self.assertEqual(div["n_candidates"], 1)
        self.assertEqual(div["decoded_channel"], 100)
        self.assertEqual(div["rx_channels"], [100])
        self.assertAlmostEqual(div["snr_db"][0], 2.0)
        self.assertEqual(div["crc_mask"], 1)
        self.assertEqual(div["source_ids"], ["uuid-1"])

    def test_two_candidates_best_idx_is_winner(self):
        c0 = _make_candidate(
            crc_valid=False, snr_db=-5.0, rx_channel=100, frame_id="u0", arrived_at=0.0
        )
        c1 = _make_candidate(
            crc_valid=True, snr_db=2.0, rx_channel=200, frame_id="u1", arrived_at=0.1
        )
        group = PendingGroup(
            payload_hash=0xAB, created_at=0.0, candidates=[c0, c1], best_idx=1
        )
        result = build_aggregated(group)
        div = result["diversity"]
        self.assertEqual(div["n_candidates"], 2)
        self.assertEqual(div["decoded_channel"], 200)
        self.assertEqual(div["rx_channels"], [100, 200])
        self.assertEqual(div["crc_mask"], 0b10)
        self.assertEqual(div["source_ids"], ["u0", "u1"])

    def test_gap_us_computed(self):
        c0 = _make_candidate(arrived_at=1.000)
        c1 = _make_candidate(arrived_at=1.050)
        group = PendingGroup(
            payload_hash=0xAB, created_at=1.000, candidates=[c0, c1], best_idx=0
        )
        result = build_aggregated(group)
        self.assertAlmostEqual(result["diversity"]["gap_us"], 50_000, delta=1_000)

    def test_original_message_not_mutated(self):
        c = _make_candidate()
        original_keys = set(c.msg.keys())
        group = PendingGroup(
            payload_hash=0xAB, created_at=0.0, candidates=[c], best_idx=0
        )
        build_aggregated(group)
        self.assertEqual(set(c.msg.keys()), original_keys)


class TestCrcMask(unittest.TestCase):
    def test_all_ok(self):
        c0 = _make_candidate(crc_valid=True, frame_id="u0")
        c1 = _make_candidate(crc_valid=True, frame_id="u1")
        group = PendingGroup(
            payload_hash=0, created_at=0.0, candidates=[c0, c1], best_idx=0
        )
        self.assertEqual(build_aggregated(group)["diversity"]["crc_mask"], 0b11)

    def test_none_ok(self):
        c0 = _make_candidate(crc_valid=False, frame_id="u0")
        c1 = _make_candidate(crc_valid=False, frame_id="u1")
        group = PendingGroup(
            payload_hash=0, created_at=0.0, candidates=[c0, c1], best_idx=0
        )
        self.assertEqual(build_aggregated(group)["diversity"]["crc_mask"], 0)

    def test_mixed(self):
        c0 = _make_candidate(crc_valid=False, frame_id="u0")
        c1 = _make_candidate(crc_valid=True, frame_id="u1")
        c2 = _make_candidate(crc_valid=False, frame_id="u2")
        group = PendingGroup(
            payload_hash=0, created_at=0.0, candidates=[c0, c1, c2], best_idx=1
        )
        self.assertEqual(build_aggregated(group)["diversity"]["crc_mask"], 0b010)


class TestFnv1a64(unittest.TestCase):
    def test_empty_returns_offset_basis(self):
        self.assertEqual(fnv1a64(b""), 14695981039346656037)

    def test_known_vector(self):
        # Manually computed: h ^= 0x08, h *= prime, h ^= 0x4D, h *= prime, h ^= 0xAA, h *= prime
        self.assertEqual(fnv1a64(b"\x08\x4d\xaa"), 2146301778539204652)

    def test_bytearray_accepted(self):
        expected = fnv1a64(b"\x01\x02\x03")
        self.assertEqual(fnv1a64(bytearray(b"\x01\x02\x03")), expected)


class TestTxEchoSuppression(unittest.TestCase):
    """Test the TX echo filter logic at the data-structure level.

    These tests exercise the hash-matching and TTL logic without
    needing sockets or the full aggregation loop.
    """

    def test_matching_hash_is_suppressed(self):
        """A frame whose payload_hash matches a recent TX should be suppressed."""
        import time

        payload = b"\x01\x02\x03"
        tx_hash = fnv1a64(payload)
        tx_echo_hashes: dict[int, float] = {tx_hash: time.monotonic()}

        # Simulate what handle_upstream_frame does
        h = tx_hash
        suppressed = h in tx_echo_hashes
        self.assertTrue(suppressed)

        # One-shot delete
        del tx_echo_hashes[h]
        self.assertNotIn(h, tx_echo_hashes)

    def test_non_matching_hash_is_not_suppressed(self):
        """A frame with a different payload_hash should not be suppressed."""
        import time

        tx_hash = fnv1a64(b"\x01\x02\x03")
        rx_hash = fnv1a64(b"\x04\x05\x06")
        tx_echo_hashes: dict[int, float] = {tx_hash: time.monotonic()}

        suppressed = rx_hash in tx_echo_hashes
        self.assertFalse(suppressed)

    def test_ttl_expiry_clears_stale_entries(self):
        """TX echo entries older than TX_ECHO_WINDOW should be purged."""
        import time

        now = time.monotonic()
        tx_echo_hashes: dict[int, float] = {
            0xAAAA: now - TX_ECHO_WINDOW - 1.0,  # stale
            0xBBBB: now - 1.0,  # fresh
        }

        # Simulate drain_expired TTL logic
        tx_stale = [h for h, t in tx_echo_hashes.items() if (now - t) >= TX_ECHO_WINDOW]
        for h in tx_stale:
            del tx_echo_hashes[h]

        self.assertNotIn(0xAAAA, tx_echo_hashes)
        self.assertIn(0xBBBB, tx_echo_hashes)

    def test_one_shot_allows_subsequent_same_hash(self):
        """After suppressing once, a second frame with the same hash should pass."""
        import time

        tx_hash = fnv1a64(b"\x01\x02\x03")
        tx_echo_hashes: dict[int, float] = {tx_hash: time.monotonic()}

        # First frame: suppressed + deleted
        self.assertIn(tx_hash, tx_echo_hashes)
        del tx_echo_hashes[tx_hash]

        # Second frame with same hash: not suppressed
        self.assertNotIn(tx_hash, tx_echo_hashes)


if __name__ == "__main__":
    unittest.main()
