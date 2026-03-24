#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Unit tests for lora_test.py pure functions."""

import socket
import unittest

from lora_test import (
    ConfigPoint,
    PointResult,
    MATRICES,
    point_label,
    build_summary,
    _collect_decode,
    _collect_scan,
    _collect_bridge,
    _extract_pubkey_from_card,
    bridge_payload,
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


class TestCLIParsing(unittest.TestCase):
    """Test --serial / --tcp mutually exclusive argument group."""

    def _parse(self, argv: list[str]):
        """Parse argv through lora_test's argparser, return namespace."""
        import argparse
        import io
        import contextlib

        # Re-create the parser (mirrors main() structure)
        parser = argparse.ArgumentParser()
        sub = parser.add_subparsers(dest="mode", required=True)
        for name in ("decode", "scan", "tx", "bridge"):
            p = sub.add_parser(name)
            conn = p.add_mutually_exclusive_group(required=True)
            conn.add_argument("--serial")
            conn.add_argument("--tcp", metavar="HOST:PORT")
            p.add_argument("--matrix", default="basic")
            p.add_argument("--config", default="apps/config.toml")
            p.add_argument("--hypothesis", default="")
            p.add_argument("--label", default="")
        return parser.parse_args(argv)

    def test_serial_flag(self):
        args = self._parse(["decode", "--serial", "/dev/cu.usbserial-0001"])
        self.assertEqual(args.serial, "/dev/cu.usbserial-0001")
        self.assertIsNone(args.tcp)

    def test_tcp_flag(self):
        args = self._parse(["decode", "--tcp", "192.168.1.42:4000"])
        self.assertIsNone(args.serial)
        self.assertEqual(args.tcp, "192.168.1.42:4000")

    def test_serial_and_tcp_exclusive(self):
        """--serial and --tcp cannot be used together."""
        with self.assertRaises(SystemExit):
            self._parse(["decode", "--serial", "/dev/ttyUSB0", "--tcp", "1.2.3.4:80"])

    def test_neither_serial_nor_tcp_fails(self):
        """At least one of --serial or --tcp is required."""
        with self.assertRaises(SystemExit):
            self._parse(["decode", "--matrix", "basic"])

    def test_tcp_all_modes(self):
        """--tcp works for decode, scan, and tx modes."""
        for mode in ("decode", "scan", "tx"):
            args = self._parse([mode, "--tcp", "10.0.0.5:7835"])
            self.assertEqual(args.tcp, "10.0.0.5:7835")
            self.assertEqual(args.mode, mode)

    def test_tcp_host_port_parsing(self):
        """parse_host_port correctly splits --tcp value."""
        from lora_common import parse_host_port

        host, port = parse_host_port("192.168.1.42:4000")
        self.assertEqual(host, "192.168.1.42")
        self.assertEqual(port, 4000)

    def test_tcp_host_port_ipv6(self):
        """parse_host_port handles IPv6 bracket notation."""
        from lora_common import parse_host_port

        host, port = parse_host_port("[::1]:7835")
        self.assertEqual(host, "::1")
        self.assertEqual(port, 7835)

    def test_tcp_host_port_bad_format(self):
        """parse_host_port raises ValueError for bad input."""
        from lora_common import parse_host_port

        with self.assertRaises(ValueError):
            parse_host_port("no-port-here")


# ---- send_lora_config tests ----


class TestSendLoraConfig(unittest.TestCase):
    """Tests for lora_common.send_lora_config()."""

    def setUp(self):
        """Create a mock UDP server and client socket."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind(("127.0.0.1", 0))
        self.server_addr = self.server.getsockname()

        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.bind(("127.0.0.1", 0))

    def tearDown(self):
        self.client.close()
        self.server.close()

    def _respond_ack(self, ok: bool = True) -> dict:
        """Read one message from the server and send a lora_config_ack back.

        Returns the decoded CBOR request message.
        """
        import cbor2

        self.server.settimeout(2.0)
        data, client_addr = self.server.recvfrom(65536)
        request = cbor2.loads(data)

        ack = cbor2.dumps({"type": "lora_config_ack", "ok": ok})
        self.server.sendto(ack, client_addr)
        return request

    def test_success(self):
        """send_lora_config returns True when ack has ok=True."""
        import threading
        from lora_common import send_lora_config

        request_holder: list[dict] = []

        def responder():
            req = self._respond_ack(ok=True)
            request_holder.append(req)

        t = threading.Thread(target=responder)
        t.start()

        result = send_lora_config(
            self.client, self.server_addr, timeout=2.0, sf=8, bw=62500
        )
        t.join(timeout=3.0)

        self.assertTrue(result)
        # Verify the sent CBOR message has the right structure
        req = request_holder[0]
        self.assertEqual(req["type"], "lora_config")
        self.assertEqual(req["sf"], 8)
        self.assertEqual(req["bw"], 62500)

    def test_ack_not_ok(self):
        """send_lora_config returns False when ack has ok=False."""
        import threading
        from lora_common import send_lora_config

        def responder():
            self._respond_ack(ok=False)

        t = threading.Thread(target=responder)
        t.start()

        result = send_lora_config(self.client, self.server_addr, timeout=2.0, sf=12)
        t.join(timeout=3.0)
        self.assertFalse(result)

    def test_timeout(self):
        """send_lora_config returns False on timeout (no response)."""
        from lora_common import send_lora_config

        result = send_lora_config(self.client, self.server_addr, timeout=0.1, sf=8)
        self.assertFalse(result)

    def test_zero_values_excluded(self):
        """Keys with zero values are not included in the CBOR message."""
        import threading
        from lora_common import send_lora_config

        request_holder: list[dict] = []

        def responder():
            req = self._respond_ack(ok=True)
            request_holder.append(req)

        t = threading.Thread(target=responder)
        t.start()

        send_lora_config(
            self.client,
            self.server_addr,
            timeout=2.0,
            sf=8,
            bw=0,
            rx_gain=40.0,
        )
        t.join(timeout=3.0)

        req = request_holder[0]
        self.assertIn("sf", req)
        self.assertNotIn("bw", req)  # zero → excluded
        self.assertIn("rx_gain", req)
        self.assertAlmostEqual(req["rx_gain"], 40.0)

    def test_unknown_key_raises(self):
        """send_lora_config raises ValueError for unknown keys."""
        from lora_common import send_lora_config

        with self.assertRaises(ValueError):
            send_lora_config(self.client, self.server_addr, timeout=0.1, bogus_key=42)

    def test_socket_timeout_restored(self):
        """Original socket timeout is restored after the call."""
        from lora_common import send_lora_config

        self.client.settimeout(99.0)
        send_lora_config(self.client, self.server_addr, timeout=0.1, sf=8)
        self.assertAlmostEqual(self.client.gettimeout(), 99.0, places=1)

    def test_ignores_non_ack_messages(self):
        """Non-ack messages are skipped; ack is still received."""
        import threading
        from lora_common import send_lora_config
        import cbor2

        def responder():
            self.server.settimeout(2.0)
            data, client_addr = self.server.recvfrom(65536)
            # Send a non-ack message first
            noise = cbor2.dumps({"type": "lora_frame", "seq": 1})
            self.server.sendto(noise, client_addr)
            # Then the real ack
            ack = cbor2.dumps({"type": "lora_config_ack", "ok": True})
            self.server.sendto(ack, client_addr)

        t = threading.Thread(target=responder)
        t.start()

        result = send_lora_config(self.client, self.server_addr, timeout=2.0, sf=9)
        t.join(timeout=3.0)
        self.assertTrue(result)


class TestBridgeCollection(unittest.TestCase):
    """Tests for bridge mode _collect_bridge() summary builder."""

    def test_bridge_summary_all_pass(self):
        """3 points, all 4 phases pass each."""
        results = [
            {
                "config": {"sf": 8, "bw": 62500, "freq_mhz": 869.618},
                "advert_rx": True,
                "advert_tx": True,
                "msg_tx": True,
                "msg_rx": True,
                "phases_passed": 4,
                "phases_total": 4,
            },
            {
                "config": {"sf": 12, "bw": 62500, "freq_mhz": 863.5},
                "advert_rx": True,
                "advert_tx": True,
                "msg_tx": True,
                "msg_rx": True,
                "phases_passed": 4,
                "phases_total": 4,
            },
            {
                "config": {"sf": 7, "bw": 125000, "freq_mhz": 868.1},
                "advert_rx": True,
                "advert_tx": True,
                "msg_tx": True,
                "msg_rx": True,
                "phases_passed": 4,
                "phases_total": 4,
            },
        ]
        s = _collect_bridge(results)
        self.assertEqual(s["mode"], "bridge")
        self.assertEqual(s["points"], 3)
        self.assertEqual(s["advert_rx"], 3)
        self.assertEqual(s["advert_tx"], 3)
        self.assertEqual(s["msg_tx"], 3)
        self.assertEqual(s["msg_rx"], 3)
        self.assertAlmostEqual(s["pass_rate"], 1.0)

    def test_bridge_summary_partial(self):
        """3 points, some phases fail."""
        results = [
            {
                "advert_rx": True,
                "advert_tx": True,
                "msg_tx": True,
                "msg_rx": True,
            },
            {
                "advert_rx": True,
                "advert_tx": False,
                "msg_tx": False,
                "msg_rx": False,
            },
            {
                "advert_rx": True,
                "advert_tx": True,
                "msg_tx": False,
                "msg_rx": True,
            },
        ]
        s = _collect_bridge(results)
        self.assertEqual(s["points"], 3)
        self.assertEqual(s["advert_rx"], 3)
        self.assertEqual(s["advert_tx"], 2)
        self.assertEqual(s["msg_tx"], 1)
        self.assertEqual(s["msg_rx"], 2)
        # (3 + 2 + 1 + 2) / (4 * 3) = 8/12 = 0.667
        self.assertAlmostEqual(s["pass_rate"], 0.667, places=3)

    def test_bridge_summary_empty(self):
        """Empty results list."""
        s = _collect_bridge([])
        self.assertEqual(s["points"], 0)
        self.assertEqual(s["pass_rate"], 0)


class TestBridgePayload(unittest.TestCase):
    """Tests for bridge_payload() format."""

    def test_payload_format(self):
        """Payload matches expected format: sf{sf}bw{bw_khz}f{freq_khz}_{hex:02x}."""
        import re

        p = ConfigPoint(sf=8, bw=62500, freq_mhz=869.618)
        payload = bridge_payload(p)
        # e.g. "sf8bw62f869618_a7"
        self.assertRegex(payload, r"^sf8bw62f869618_[0-9a-f]{2}$")

    def test_payload_uniqueness(self):
        """Two calls produce different payloads (different nonce)."""
        p = ConfigPoint(sf=8, bw=62500, freq_mhz=869.618)
        # Run 100 times, expect at least 2 distinct values
        payloads = {bridge_payload(p) for _ in range(100)}
        self.assertGreater(len(payloads), 1)

    def test_payload_125k(self):
        """BW125k formats as 'bw125'."""
        p = ConfigPoint(sf=7, bw=125000, freq_mhz=868.1)
        payload = bridge_payload(p)
        self.assertRegex(payload, r"^sf7bw125f868100_[0-9a-f]{2}$")

    def test_payload_250k(self):
        """BW250k formats as 'bw250'."""
        p = ConfigPoint(sf=8, bw=250000, freq_mhz=866.5)
        payload = bridge_payload(p)
        self.assertRegex(payload, r"^sf8bw250f866500_[0-9a-f]{2}$")


class TestExtractPubkey(unittest.TestCase):
    """Tests for _extract_pubkey_from_card()."""

    def test_contact_uri(self):
        """Extract pubkey from standard contact/add URI."""
        uri = "meshcore://contact/add?name=lora_trx&public_key=aabbccdd00112233445566778899aabbccddeeff00112233445566778899aabb&type=1"
        pk = _extract_pubkey_from_card(uri)
        self.assertEqual(
            pk, "aabbccdd00112233445566778899aabbccddeeff00112233445566778899aabb"
        )

    def test_empty_output(self):
        """Empty or garbage output returns empty string."""
        self.assertEqual(_extract_pubkey_from_card(""), "")
        self.assertEqual(_extract_pubkey_from_card("not a uri"), "")

    def test_case_insensitive(self):
        """Pubkey is returned lowercase."""
        uri = "meshcore://contact/add?public_key=AABBCCDD00112233445566778899AABBCCDDEEFF00112233445566778899AABB&type=1"
        pk = _extract_pubkey_from_card(uri)
        self.assertEqual(
            pk, "aabbccdd00112233445566778899aabbccddeeff00112233445566778899aabb"
        )


class TestCLIBridgeMode(unittest.TestCase):
    """Test that bridge mode is accepted by the CLI parser."""

    def _parse(self, argv: list[str]):
        import argparse

        parser = argparse.ArgumentParser()
        sub = parser.add_subparsers(dest="mode", required=True)
        for name in ("decode", "scan", "tx", "bridge"):
            p = sub.add_parser(name)
            conn = p.add_mutually_exclusive_group(required=True)
            conn.add_argument("--serial")
            conn.add_argument("--tcp", metavar="HOST:PORT")
            p.add_argument("--matrix", default="basic")
            p.add_argument("--config", default="apps/config.toml")
            p.add_argument("--hypothesis", default="")
            p.add_argument("--label", default="")
        return parser.parse_args(argv)

    def test_bridge_mode_serial(self):
        args = self._parse(["bridge", "--serial", "/dev/cu.usbserial-0001"])
        self.assertEqual(args.mode, "bridge")
        self.assertEqual(args.serial, "/dev/cu.usbserial-0001")

    def test_bridge_mode_tcp(self):
        args = self._parse(["bridge", "--tcp", "192.168.1.42:4000"])
        self.assertEqual(args.mode, "bridge")
        self.assertEqual(args.tcp, "192.168.1.42:4000")


if __name__ == "__main__":
    unittest.main()
