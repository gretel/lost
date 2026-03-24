#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Unit tests for lora_test.py pure functions."""

import unittest
from lora_test import (
    ConfigPoint,
    PointResult,
    MATRICES,
    point_label,
    build_summary,
    _collect_decode,
    _collect_scan,
    FREQ_TOL,
)
from dataclasses import asdict


class TestConfigPoint(unittest.TestCase):
    def test_defaults(self):
        p = ConfigPoint(sf=8, bw=62500, freq_mhz=869.618)
        self.assertEqual(p.tx_power, 2)

    def test_asdict_roundtrip(self):
        p = ConfigPoint(sf=12, bw=125000, freq_mhz=863.5, tx_power=-4)
        d = asdict(p)
        p2 = ConfigPoint(**d)
        self.assertEqual(p, p2)


class TestPointLabel(unittest.TestCase):
    def test_default_power(self):
        p = ConfigPoint(sf=8, bw=62500, freq_mhz=869.618)
        self.assertEqual(point_label(p), "SF8/BW62k@869.618")

    def test_custom_power(self):
        p = ConfigPoint(sf=12, bw=125000, freq_mhz=863.5, tx_power=-4)
        self.assertEqual(point_label(p), "SF12/BW125k@863.500 -4dBm")


class TestMatrices(unittest.TestCase):
    def test_all_matrices_exist(self):
        for name in ("basic", "full", "dc_edge", "sf_sweep", "bw_sweep", "power_sweep"):
            self.assertIn(name, MATRICES)

    def test_no_duplicate_configs(self):
        """Each matrix should have unique config points."""
        for name, points in MATRICES.items():
            seen = set()
            for p in points:
                key = (p.sf, p.bw, p.freq_mhz, p.tx_power)
                self.assertNotIn(key, seen, f"duplicate in {name}: {key}")
                seen.add(key)

    def test_basic_has_3_points(self):
        self.assertEqual(len(MATRICES["basic"]), 3)

    def test_full_has_8_points(self):
        self.assertEqual(len(MATRICES["full"]), 8)


class TestCollectDecode(unittest.TestCase):
    def test_crc_ok_frame(self):
        result = PointResult(config={})
        events = [
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "bw": 62500, "snr_db": 5.1, "channel_freq": 0.0},
            },
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(result.crc_ok, 1)
        self.assertEqual(result.crc_fail, 0)
        self.assertEqual(result.best_snr, 5.1)
        self.assertEqual(result.detected_sfs, {8: 1})
        self.assertEqual(len(result.frames), 1)

    def test_crc_fail_frame(self):
        result = PointResult(config={})
        events = [
            {
                "type": "lora_frame",
                "crc_valid": False,
                "phy": {"sf": 12, "snr_db": -2.0},
            },
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(result.crc_ok, 0)
        self.assertEqual(result.crc_fail, 1)

    def test_freq_filter(self):
        """Frames outside ±200kHz should be rejected."""
        result = PointResult(config={})
        events = [
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "channel_freq": 866.0e6},
            },  # 3.6 MHz away
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(len(result.frames), 0)

    def test_freq_filter_within_tolerance(self):
        """Frames within ±200kHz should be accepted."""
        result = PointResult(config={})
        tx = 869.618e6
        events = [
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "channel_freq": tx + 100e3, "snr_db": 3.0},
            },
        ]
        _collect_decode(result, events, tx)
        self.assertEqual(len(result.frames), 1)

    def test_no_channel_freq_accepted(self):
        """Narrowband mode: channel_freq=0 is always accepted."""
        result = PointResult(config={})
        events = [
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "channel_freq": 0.0, "snr_db": 10.0},
            },
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(len(result.frames), 1)

    def test_ignores_non_frame_events(self):
        result = PointResult(config={})
        events = [
            {"type": "config", "phy": {}},
            {"type": "scan_spectrum"},
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(len(result.frames), 0)

    def test_multiple_frames_best_snr(self):
        result = PointResult(config={})
        events = [
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "snr_db": 3.0, "channel_freq": 0.0},
            },
            {
                "type": "lora_frame",
                "crc_valid": True,
                "phy": {"sf": 8, "snr_db": 11.0, "channel_freq": 0.0},
            },
        ]
        _collect_decode(result, events, 869.618e6)
        self.assertEqual(result.best_snr, 11.0)
        self.assertEqual(result.crc_ok, 2)


class TestCollectScan(unittest.TestCase):
    def test_detection_and_sweep(self):
        result = PointResult(config={})
        tx = 869.618e6
        events = [
            {"type": "scan_sweep_end", "duration_ms": 530, "overflows": 0},
            {"type": "scan_sweep_end", "duration_ms": 510, "overflows": 1},
            {
                "type": "scan_spectrum",
                "detections": [
                    {"freq": tx, "ratio": 42.5, "sf": 8},
                ],
            },
        ]
        _collect_scan(result, events, tx)
        self.assertEqual(result.sweeps, 2)
        self.assertEqual(result.avg_sweep_ms, 520)
        self.assertEqual(result.overflows, 1)
        self.assertEqual(result.det_count, 1)
        self.assertEqual(result.best_ratio, 42.5)
        self.assertEqual(result.best_sf, 8)

    def test_freq_filter_scan(self):
        result = PointResult(config={})
        events = [
            {
                "type": "scan_spectrum",
                "detections": [
                    {"freq": 866.0e6, "ratio": 50.0, "sf": 10},
                ],
            },
        ]
        _collect_scan(result, events, 869.618e6)
        self.assertEqual(result.det_count, 0)

    def test_mode_sf(self):
        """best_sf should be the most common SF, not the highest ratio."""
        result = PointResult(config={})
        tx = 869.618e6
        events = [
            {
                "type": "scan_spectrum",
                "detections": [
                    {"freq": tx, "ratio": 100.0, "sf": 12},
                    {"freq": tx, "ratio": 30.0, "sf": 8},
                    {"freq": tx, "ratio": 35.0, "sf": 8},
                ],
            },
        ]
        _collect_scan(result, events, tx)
        self.assertEqual(result.best_sf, 8)  # mode, not max-ratio


class TestBuildSummary(unittest.TestCase):
    def test_decode_summary(self):
        results = [
            PointResult(
                config={},
                crc_ok=1,
                crc_fail=0,
                best_snr=5.0,
                frames=[{"sf": 8}],
                tx_ok=True,
            ),
            PointResult(
                config={},
                crc_ok=0,
                crc_fail=1,
                best_snr=-2.0,
                frames=[{"sf": 12}],
                tx_ok=True,
            ),
            PointResult(config={}, tx_ok=True),  # no frames
        ]
        s = build_summary("decode", results)
        self.assertEqual(s["points"], 3)
        self.assertEqual(s["tx_ok"], 3)
        self.assertEqual(s["points_with_decode"], 2)
        self.assertEqual(s["total_crc_ok"], 1)
        self.assertEqual(s["total_crc_fail"], 1)
        self.assertAlmostEqual(s["crc_ok_rate"], 0.5)
        self.assertEqual(s["snr_min"], -2.0)
        self.assertEqual(s["snr_max"], 5.0)

    def test_decode_no_frames(self):
        s = build_summary("decode", [PointResult(config={}, tx_ok=True)])
        self.assertEqual(s["crc_ok_rate"], 0)
        self.assertIsNone(s["snr_min"])

    def test_scan_summary(self):
        results = [
            PointResult(
                config={}, det_count=3, best_ratio=42.0, overflows=2, tx_ok=True
            ),
            PointResult(config={}, det_count=0, tx_ok=True),
        ]
        s = build_summary("scan", results)
        self.assertEqual(s["points"], 2)
        self.assertEqual(s["points_detected"], 1)
        self.assertEqual(s["total_detections"], 3)
        self.assertEqual(s["total_overflows"], 2)
        self.assertEqual(s["ratio_min"], 42.0)

    def test_tx_summary(self):
        results = [
            PointResult(config={}, crc_ok=2, crc_fail=0, tx_ok=True),  # advert+msg pass
            PointResult(
                config={}, crc_ok=1, crc_fail=1, tx_ok=True
            ),  # advert pass, msg fail
            PointResult(config={}, crc_ok=0, crc_fail=0, tx_ok=False),  # tx failed
        ]
        s = build_summary("tx", results)
        self.assertEqual(s["points"], 3)
        self.assertEqual(s["tx_ok"], 2)
        self.assertEqual(s["total_tests"], 4)
        self.assertEqual(s["total_pass"], 3)
        self.assertEqual(s["total_fail"], 1)
        self.assertAlmostEqual(s["pass_rate"], 0.75)

    def test_tx_summary_no_tests(self):
        s = build_summary("tx", [PointResult(config={}, tx_ok=False)])
        self.assertEqual(s["pass_rate"], 0)


if __name__ == "__main__":
    unittest.main()
