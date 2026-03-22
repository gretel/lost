#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Tests for lora_mon.py and lora_decode_lorawan.py.

Run:  python3 -m unittest scripts/test_lora_mon.py -v
"""

from __future__ import annotations

import io
import os
import socket
import sys
import threading
import time
import unittest
from typing import Any

import cbor2

sys.path.insert(0, os.path.dirname(__file__))
import cbor_stream
import lora_common
import lora_mon
import lora_decode_lorawan


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


# ---- format_frame ----


class TestFormatFrame(unittest.TestCase):
    def test_text_contains_essentials(self):
        text = lora_common.format_frame(make_frame())
        self.assertIn("#1", text)
        self.assertIn("CRC_OK", text)
        self.assertIn("SF8", text)

    def test_crc_fail(self):
        text = lora_common.format_frame(make_frame(crc_valid=False))
        self.assertIn("CRC_FAIL", text)

    def test_sync_word_label(self):
        self.assertIn("0x12", lora_common.format_frame(make_frame(sync_word=0x12)))
        self.assertIn("0x2B", lora_common.format_frame(make_frame(sync_word=0x2B)))
        self.assertIn("0x34", lora_common.format_frame(make_frame(sync_word=0x34)))
        self.assertIn("0xFF", lora_common.format_frame(make_frame(sync_word=0xFF)))

    def test_snr_shown(self):
        msg = make_frame()
        msg["phy"]["snr_db"] = 12.5
        text = lora_common.format_frame(msg)
        self.assertIn("SNR=12.5dB", text)

    def test_rx_channel_shown(self):
        msg = make_frame()
        msg["rx_channel"] = 1
        text = lora_common.format_frame(msg)
        self.assertIn("ch=1", text)

    def test_no_snr_when_absent(self):
        text = lora_common.format_frame(make_frame())
        self.assertNotIn("SNR=", text)

    def test_no_channel_when_absent(self):
        text = lora_common.format_frame(make_frame())
        self.assertNotIn("ch=", text)

    def test_noise_floor_shown(self):
        msg = make_frame()
        msg["phy"]["noise_floor_db"] = -45.3
        text = lora_common.format_frame(msg)
        self.assertIn("NF=-45.3dBFS", text)

    def test_no_noise_floor_when_absent(self):
        text = lora_common.format_frame(make_frame())
        self.assertNotIn("NF=", text)

    def test_bw_shown(self):
        text = lora_common.format_frame(make_frame(bw=125000))
        self.assertIn("BW125k", text)

    def test_bw_62500_shown(self):
        text = lora_common.format_frame(make_frame(bw=62500))
        self.assertIn("BW62.5k", text)

    def test_decode_label_shown(self):
        msg = make_frame()
        msg["decode_label"] = "SF8-sync12"
        text = lora_common.format_frame(msg)
        self.assertIn("[SF8-sync12]", text)

    def test_no_decode_label_when_absent(self):
        msg = make_frame()
        # sync_word=0xFF → no MeshCore summary, so no "[" from any source
        msg["phy"]["sync_word"] = 0xFF
        text = lora_common.format_frame(msg)
        self.assertNotIn("[SF", text)

    def test_diversity_shown(self):
        msg = make_frame()
        msg["diversity"] = {
            "n_candidates": 2,
            "decoded_channel": 1,
            "rx_channels": [0, 1],
            "snr_db": [3.2, 5.1],
            "crc_mask": 3,
            "gap_us": 20000,
            "source_ids": ["a", "b"],
        }
        text = lora_common.format_frame(msg)
        self.assertIn("div:", text)
        self.assertIn("2 chains", text)
        self.assertIn("gap=20ms", text)
        self.assertIn("ch=1 won", text)
        self.assertIn("SNR:", text)

    def test_no_diversity_when_absent(self):
        text = lora_common.format_frame(make_frame())
        self.assertNotIn("div:", text)

    def test_diversity_single_chain(self):
        msg = make_frame()
        msg["diversity"] = {
            "n_candidates": 1,
            "decoded_channel": 0,
            "rx_channels": [0],
            "snr_db": [5.0],
            "crc_mask": 1,
            "gap_us": 0,
            "source_ids": ["x"],
        }
        text = lora_common.format_frame(msg)
        self.assertIn("1 chain", text)
        self.assertNotIn("chains", text)
        self.assertNotIn("ch=", text)  # no "won" line for single chain
        self.assertNotIn("gap=", text)  # gap_us == 0, omitted


# ---- Decryption display ----


class TestFormatFrameDecryption(unittest.TestCase):
    """Test format_frame() with decryption parameters."""

    def setUp(self):
        from nacl.signing import SigningKey

        from meshcore_crypto import meshcore_expanded_key

        self.dest_seed = os.urandom(32)
        self.dest_sk = SigningKey(self.dest_seed)
        self.dest_pub = bytes(self.dest_sk.verify_key)
        self.dest_expanded = meshcore_expanded_key(self.dest_seed)

        self.sender_seed = os.urandom(32)
        self.sender_sk = SigningKey(self.sender_seed)
        self.sender_pub = bytes(self.sender_sk.verify_key)
        self.sender_expanded = meshcore_expanded_key(self.sender_seed)

    def test_txt_msg_decrypted(self):
        """TXT_MSG shows decrypted text when keys are available."""
        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            self.sender_expanded, self.sender_pub, self.dest_pub, "Hello World"
        )
        msg = make_frame(payload=pkt, crc_valid=True, sync_word=0x12)
        known_keys = {self.sender_pub.hex(): self.sender_pub}
        text = lora_common.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
            _parse_summary=lora_mon.parse_meshcore_summary,
            _decrypt=lora_mon._try_decrypt,
        )
        self.assertIn("TXT_MSG from", text)
        self.assertIn("Hello World", text)

    def test_anon_req_decrypted(self):
        """ANON_REQ shows decrypted text when addressed to us."""
        from meshcore_tx import build_anon_req

        pkt = build_anon_req(
            self.sender_expanded, self.sender_pub, self.dest_pub, b"AnonMsg"
        )
        msg = make_frame(payload=pkt, crc_valid=True, sync_word=0x12)
        text = lora_common.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys={},
            _parse_summary=lora_mon.parse_meshcore_summary,
            _decrypt=lora_mon._try_decrypt,
        )
        self.assertIn("ANON_REQ from", text)
        self.assertIn("AnonMsg", text)

    def test_no_decryption_without_keys(self):
        """Without our_prv/our_pub, no decryption line appears."""
        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            self.sender_expanded, self.sender_pub, self.dest_pub, "Secret"
        )
        msg = make_frame(payload=pkt, crc_valid=True, sync_word=0x12)
        text = lora_common.format_frame(msg)
        self.assertNotIn("TXT_MSG from", text)
        self.assertNotIn("ANON_REQ from", text)

    def test_no_decryption_for_crc_fail(self):
        """CRC_FAIL frames are not decrypted even with keys."""
        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            self.sender_expanded, self.sender_pub, self.dest_pub, "NoCRC"
        )
        msg = make_frame(payload=pkt, crc_valid=False, sync_word=0x12)
        known_keys = {self.sender_pub.hex(): self.sender_pub}
        text = lora_common.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
            _parse_summary=lora_mon.parse_meshcore_summary,
            _decrypt=lora_mon._try_decrypt,
        )
        self.assertNotIn("TXT_MSG from", text)

    def test_no_decryption_for_non_meshcore(self):
        """Non-MeshCore sync words are not decrypted."""
        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            self.sender_expanded, self.sender_pub, self.dest_pub, "Meshtastic"
        )
        msg = make_frame(payload=pkt, crc_valid=True, sync_word=0x2B)
        known_keys = {self.sender_pub.hex(): self.sender_pub}
        text = lora_common.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
            _parse_summary=lora_mon.parse_meshcore_summary,
            _decrypt=lora_mon._try_decrypt,
        )
        self.assertNotIn("TXT_MSG from", text)


# ---- MeshCore summary ----


class TestParseMeshcoreSummary(unittest.TestCase):
    def test_advert_with_name(self):
        from meshcore_tx import build_advert

        seed = bytes.fromhex(
            "deadbeefcafebabe0123456789abcdeffedcba9876543210baadf00ddeadbeef"
        )
        pub = bytes.fromhex(
            "682500378e875577ff16b044bb6f2823bc4a7fcb3f34cab962f13553fb2369b5"
        )
        pkt = build_advert(seed, pub, name="NodeX")
        summary = lora_mon.parse_meshcore_summary(pkt)
        self.assertIn("FLOOD/ADVERT", summary)
        self.assertIn('"NodeX"', summary)

    def test_empty_payload(self):
        self.assertEqual(lora_mon.parse_meshcore_summary(b""), "")

    def test_single_byte(self):
        self.assertEqual(lora_mon.parse_meshcore_summary(b"\x00"), "")


# ---- UDP integration ----


class TestConnectUdp(unittest.TestCase):
    """Test the connect_udp flow (server-side registration model)."""

    def test_connect_and_receive(self):
        """Simulate a lora_trx server: bind, accept registration, send frame."""
        port = find_free_port()
        results = []

        def fake_server():
            srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.settimeout(2.0)
            srv.bind(("127.0.0.1", port))
            try:
                _data, client_addr = srv.recvfrom(64)
                frame = cbor2.dumps(make_frame(seq=42))
                srv.sendto(frame, client_addr)
            except socket.timeout:
                pass
            finally:
                srv.close()

        def client():
            client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            client_sock.bind(("0.0.0.0", 0))
            client_sock.settimeout(2.0)
            client_sock.sendto(b"sub", ("127.0.0.1", port))
            try:
                data, _ = client_sock.recvfrom(65536)
                msg = cbor2.loads(data)
                results.append(lora_common.format_frame(msg))
            except socket.timeout:
                pass
            finally:
                client_sock.close()

        srv_thread = threading.Thread(target=fake_server, daemon=True)
        srv_thread.start()
        time.sleep(0.05)

        cli_thread = threading.Thread(target=client, daemon=True)
        cli_thread.start()

        cli_thread.join(timeout=3)
        srv_thread.join(timeout=3)
        self.assertEqual(len(results), 1)
        self.assertIn("#42", results[0])


# ---- LoRaWAN decoder ----


class TestLoRaWANParser(unittest.TestCase):
    """Unit tests for lora_decode_lorawan.parse_lorawan()."""

    def _make_data_frame(
        self,
        mtype: int = 2,
        dev_addr: int = 0x26011234,
        fctrl: int = 0x80,
        fcnt: int = 42,
        fport: int | None = 1,
        frm_payload: bytes = b"\xab\xcd\xef",
        mic: int = 0xDEADBEEF,
    ) -> bytes:
        import struct

        mhdr = (mtype << 5) | 0x00
        buf = bytearray([mhdr])
        buf += struct.pack("<I", dev_addr)
        buf += bytes([fctrl])
        buf += struct.pack("<H", fcnt)
        fopts_len = fctrl & 0x0F
        buf += b"\x00" * fopts_len
        if fport is not None:
            buf += bytes([fport])
            buf += frm_payload
        buf += struct.pack("<I", mic)
        return bytes(buf)

    def test_unconf_data_up(self):
        raw = self._make_data_frame(mtype=2, dev_addr=0x26011234, fcnt=42, fport=1)
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mtype, 2)
        self.assertEqual(frame.mtype_name, "UnconfDataUp")
        self.assertTrue(frame.is_uplink)
        self.assertTrue(frame.is_data)
        self.assertEqual(frame.dev_addr, 0x26011234)
        self.assertEqual(frame.fcnt, 42)
        self.assertEqual(frame.fport, 1)
        self.assertEqual(frame.frm_payload, b"\xab\xcd\xef")
        self.assertEqual(frame.mic, 0xDEADBEEF)
        self.assertTrue(frame.adr)

    def test_conf_data_down(self):
        raw = self._make_data_frame(mtype=5, fctrl=0x30)
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mtype_name, "ConfDataDown")
        self.assertFalse(frame.is_uplink)
        self.assertTrue(frame.ack)
        self.assertTrue(frame.fpending)
        self.assertFalse(frame.adr)

    def test_fctrl_fopts(self):
        raw = self._make_data_frame(fctrl=0x03, fport=None, frm_payload=b"")
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.fopts_len, 3)
        self.assertEqual(len(frame.fopts), 3)
        self.assertIsNone(frame.fport)

    def test_join_request(self):
        import struct

        mhdr = 0 << 5
        join_eui = 0x0102030405060708
        dev_eui = 0x1112131415161718
        dev_nonce = 0xABCD
        mic = 0x12345678
        raw = bytes([mhdr])
        raw += struct.pack("<Q", join_eui)
        raw += struct.pack("<Q", dev_eui)
        raw += struct.pack("<H", dev_nonce)
        raw += struct.pack("<I", mic)

        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mtype, 0)
        self.assertEqual(frame.mtype_name, "JoinRequest")
        self.assertTrue(frame.is_join)
        self.assertTrue(frame.is_uplink)
        self.assertEqual(frame.join_eui, join_eui)
        self.assertEqual(frame.dev_eui, dev_eui)
        self.assertEqual(frame.dev_nonce, dev_nonce)
        self.assertEqual(frame.mic, mic)

    def test_join_accept(self):
        import struct

        mhdr = 1 << 5
        raw = bytes([mhdr]) + b"\x00" * 12 + struct.pack("<I", 0xCAFEBABE)
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mtype, 1)
        self.assertTrue(frame.is_join)
        self.assertEqual(frame.mic, 0xCAFEBABE)
        self.assertEqual(len(frame.frm_payload), 12)

    def test_too_short(self):
        self.assertIsNone(lora_decode_lorawan.parse_lorawan(b""))
        self.assertIsNone(lora_decode_lorawan.parse_lorawan(b"\x40\x00\x00"))

    def test_major_version(self):
        raw = self._make_data_frame()
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertEqual(frame.major, 0)
        self.assertEqual(frame.major_name, "LoRaWAN R1")

    def test_fport_zero_is_mac(self):
        raw = self._make_data_frame(fport=0, frm_payload=b"\x02\x01")
        frame = lora_decode_lorawan.parse_lorawan(raw)
        self.assertEqual(frame.fport, 0)

    def test_format_dev_addr(self):
        self.assertEqual(lora_decode_lorawan.format_dev_addr(0x26011234), "26011234")

    def test_format_eui(self):
        eui = 0x0807060504030201
        result = lora_decode_lorawan.format_eui(eui)
        self.assertEqual(result, "08:07:06:05:04:03:02:01")


# ---- lora_common shared module ----


class TestLoraCommon(unittest.TestCase):
    def test_format_hex_basic(self):
        self.assertEqual(lora_common.format_hex(b"\xab\xcd"), "AB CD")

    def test_format_hex_truncation(self):
        result = lora_common.format_hex(b"\x01\x02\x03", max_bytes=2)
        self.assertIn("...(3B)", result)

    def test_format_hex_sep(self):
        self.assertEqual(lora_common.format_hex(b"\xab\xcd", sep=""), "ABCD")

    def test_format_ascii_basic(self):
        self.assertEqual(lora_common.format_ascii(b"Hi\x00"), "Hi\\x00")

    def test_sync_word_name(self):
        self.assertEqual(lora_common.sync_word_name(0x12), "0x12")
        self.assertEqual(lora_common.sync_word_name(0x2B), "0x2B")
        self.assertEqual(lora_common.sync_word_name(0xFF), "0xFF")

    def test_constants(self):
        self.assertEqual(len(lora_common.ROUTE_NAMES), 4)
        self.assertEqual(len(lora_common.PAYLOAD_NAMES), 16)


# ---- CBOR Sequence stream reader ----


class TestCborStream(unittest.TestCase):
    """Tests for cbor_stream.read_cbor_seq() -- RFC 8742 CBOR Sequence reader."""

    def test_single_item(self):
        data = cbor2.dumps({"hello": "world"})
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data)))
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0], {"hello": "world"})

    def test_multiple_items(self):
        frames = [{"seq": i, "data": f"frame_{i}"} for i in range(5)]
        data = b"".join(cbor2.dumps(f) for f in frames)
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data)))
        self.assertEqual(len(items), 5)
        for i, item in enumerate(items):
            self.assertEqual(item["seq"], i)

    def test_empty_stream(self):
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(b"")))
        self.assertEqual(items, [])

    def test_mixed_types(self):
        values = [{"a": 1}, [1, 2, 3], 42, "hello"]
        data = b"".join(cbor2.dumps(v) for v in values)
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data)))
        self.assertEqual(items, values)

    def test_large_item(self):
        big = {"payload": b"\xab" * 20000}
        data = cbor2.dumps(big)
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data), chunk_size=256))
        self.assertEqual(len(items), 1)
        self.assertEqual(len(items[0]["payload"]), 20000)

    def test_truncated_item_discarded(self):
        good = cbor2.dumps({"ok": True})
        truncated = cbor2.dumps({"will": "be truncated"})[:3]
        data = good + truncated
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data)))
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0], {"ok": True})

    def test_pipe_no_hang(self):
        r_fd, w_fd = os.pipe()
        r_file = os.fdopen(r_fd, "rb")
        w_file = os.fdopen(w_fd, "wb")

        frame = {"type": "lora_frame", "seq": 1, "payload": b"\x01\x02\x03"}
        result = []
        error = []

        def reader():
            try:
                for item in cbor_stream.read_cbor_seq(r_file):
                    result.append(item)
                    break
            except Exception as e:
                error.append(e)

        t = threading.Thread(target=reader, daemon=True)
        t.start()

        w_file.write(cbor2.dumps(frame))
        w_file.flush()

        t.join(timeout=2.0)
        w_file.close()
        r_file.close()

        self.assertFalse(t.is_alive(), "read_cbor_seq blocked on pipe (hung)")
        self.assertEqual(len(error), 0, f"reader raised: {error}")
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["seq"], 1)

    def test_pipe_multiple_writes(self):
        r_fd, w_fd = os.pipe()
        r_file = os.fdopen(r_fd, "rb")
        w_file = os.fdopen(w_fd, "wb")

        result = []
        n_frames = 3

        def reader():
            for item in cbor_stream.read_cbor_seq(r_file):
                result.append(item)
                if len(result) >= n_frames:
                    break

        t = threading.Thread(target=reader, daemon=True)
        t.start()

        for i in range(n_frames):
            w_file.write(cbor2.dumps({"seq": i}))
            w_file.flush()
            time.sleep(0.01)

        t.join(timeout=5.0)
        w_file.close()
        r_file.close()

        self.assertFalse(t.is_alive(), "read_cbor_seq blocked on pipe")
        self.assertEqual(len(result), n_frames)
        for i in range(n_frames):
            self.assertEqual(result[i]["seq"], i)

    def test_lora_frame_roundtrip(self):
        frame = make_frame(seq=99, payload=b"test roundtrip")
        data = cbor2.dumps(frame)
        items = list(cbor_stream.read_cbor_seq(io.BytesIO(data)))
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["seq"], 99)
        self.assertEqual(items[0]["type"], "lora_frame")


# ---- peak_db in format_frame ----


class TestFormatFramePeak(unittest.TestCase):
    def test_peak_shown(self):
        msg = make_frame()
        msg["phy"]["peak_db"] = -6.2
        text = lora_common.format_frame(msg)
        self.assertIn("peak=-6.2dBFS", text)

    def test_no_peak_when_absent(self):
        text = lora_common.format_frame(make_frame())
        self.assertNotIn("peak=", text)


# ---- gain_advice ----


class TestGainAdvice(unittest.TestCase):
    def test_clipping(self):
        """Peak above -3 dBFS should warn about clipping."""
        state, msg = lora_mon.gain_advice({"peak_db": -1.5})
        self.assertEqual(state, "HIGH")
        self.assertIn("GAIN HIGH", msg)
        self.assertIn("reduce", msg.lower())

    def test_noise_too_high(self):
        """Noise floor above -25 dBFS should warn."""
        state, msg = lora_mon.gain_advice({"noise_floor_db": -20.0})
        self.assertEqual(state, "HIGH")
        self.assertIn("GAIN HIGH", msg)

    def test_normal_noise_no_advice(self):
        """Healthy noise floor (-70 dBFS) must NOT fire a false alarm."""
        state, msg = lora_mon.gain_advice({"noise_floor_db": -70.0})
        self.assertIsNone(state)
        self.assertIsNone(msg)

    def test_noise_dead_hardware(self):
        """Noise floor below -85 dBFS indicates hardware problem."""
        state, msg = lora_mon.gain_advice({"noise_floor_db": -90.0})
        self.assertEqual(state, "LOW")
        self.assertIn("GAIN LOW", msg)
        self.assertIn("hardware", msg.lower())

    def test_sweet_spot_no_advice(self):
        """Normal levels should return (None, None)."""
        state, msg = lora_mon.gain_advice({"peak_db": -12.0, "noise_floor_db": -45.0})
        self.assertIsNone(state)
        self.assertIsNone(msg)

    def test_empty_phy_no_advice(self):
        """No metrics at all should return (None, None)."""
        state, msg = lora_mon.gain_advice({})
        self.assertIsNone(state)
        self.assertIsNone(msg)

    def test_clipping_overrides_noise(self):
        """Clipping warning takes priority over low noise floor."""
        state, msg = lora_mon.gain_advice({"peak_db": -1.0, "noise_floor_db": -70.0})
        self.assertEqual(state, "HIGH")
        self.assertIn("peak", msg)

    def test_snr_td_low_crc_ok(self):
        """Low time-domain SNR on a CRC_OK frame → GAIN LOW."""
        state, msg = lora_mon.gain_advice({"snr_db_td": -8.0}, crc_valid=True)
        self.assertEqual(state, "LOW")
        self.assertIn("GAIN LOW", msg)
        self.assertIn("SNR", msg)

    def test_snr_td_low_crc_fail(self):
        """Low SNR on a CRC_FAIL frame must not advise (cause unknown)."""
        state, msg = lora_mon.gain_advice({"snr_db_td": -8.0}, crc_valid=False)
        self.assertIsNone(state)
        self.assertIsNone(msg)

    def test_snr_td_ok(self):
        """Healthy time-domain SNR should return (None, None)."""
        state, msg = lora_mon.gain_advice({"snr_db_td": 5.0}, crc_valid=True)
        self.assertIsNone(state)
        self.assertIsNone(msg)

    def test_state_is_none_when_no_data(self):
        """Only peak_db present but within limits → (None, None)."""
        state, msg = lora_mon.gain_advice({"peak_db": -30.0})
        self.assertIsNone(state)
        self.assertIsNone(msg)


# ---- format_config ----


class TestFormatConfig(unittest.TestCase):
    def test_config_contains_freq(self):
        msg = {
            "type": "config",
            "phy": {
                "freq": 869618000.0,
                "sf": 8,
                "bw": 62500,
                "cr": 4,
                "sync_word": 18,
                "preamble": 8,
                "rx_gain": 40.0,
                "tx_gain": 74.0,
            },
            "server": {
                "device": "uhd",
                "status_interval": 10,
                "sample_rate": 250000.0,
            },
        }
        text = lora_mon.format_config(msg)
        self.assertIn("869.618", text)
        self.assertIn("SF8", text)
        self.assertIn("RX=40", text)
        self.assertIn("uhd", text)
        self.assertIn("250 kS/s", text)

    def test_config_empty(self):
        """Empty config should not crash."""
        text = lora_mon.format_config({"type": "config"})
        self.assertIn("config", text)


# ---- format_status ----


class TestFormatStatus(unittest.TestCase):
    def test_status_shows_counts(self):
        msg = {
            "type": "status",
            "ts": "2026-02-25T12:00:00Z",
            "phy": {"rx_gain": 40.0, "tx_gain": 74.0},
            "frames": {"total": 42, "crc_ok": 38, "crc_fail": 4},
        }
        text = lora_mon.format_status(msg)
        self.assertIn("42 frames", text)
        self.assertIn("38 OK", text)
        self.assertIn("4 fail", text)
        self.assertIn("90%", text)
        self.assertIn("12:00:00", text)

    def test_status_zero_frames(self):
        msg = {
            "type": "status",
            "ts": "2026-02-25T12:00:00Z",
            "phy": {"rx_gain": 40.0},
            "frames": {"total": 0, "crc_ok": 0, "crc_fail": 0},
        }
        text = lora_mon.format_status(msg)
        self.assertIn("0 frames", text)


# ---- load_config ----


class TestLoadConfig(unittest.TestCase):
    def test_load_missing_returns_empty(self):
        """Non-existent config path returns empty dict."""
        result = lora_common.load_config("/nonexistent/config.toml")
        self.assertEqual(result, {})

    def test_load_valid_toml(self):
        """Valid TOML file is parsed correctly."""
        import tempfile

        with tempfile.NamedTemporaryFile(mode="w", suffix=".toml", delete=False) as f:
            f.write('udp_listen = "10.0.0.1"\nudp_port = 7777\n')
            f.flush()
            cfg = lora_common.load_config(f.name)
        self.assertEqual(cfg["udp_listen"], "10.0.0.1")
        self.assertEqual(cfg["udp_port"], 7777)
        os.unlink(f.name)

    def test_config_udp_defaults(self):
        """Empty config returns default host and port."""
        self.assertEqual(lora_common.config_udp_host({}), "127.0.0.1")
        self.assertEqual(lora_common.config_udp_port({}), 5555)

    def test_config_udp_from_dict(self):
        """Config values are extracted correctly."""
        cfg = {"udp_listen": "0.0.0.0", "udp_port": 9999}
        self.assertEqual(lora_common.config_udp_host(cfg), "0.0.0.0")
        self.assertEqual(lora_common.config_udp_port(cfg), 9999)


# ---- sanitize_text ----


class TestSanitizeText(unittest.TestCase):
    """Tests for lora_common.sanitize_text()."""

    def test_ascii_passthrough(self):
        """Normal printable ASCII passes through unchanged."""
        self.assertEqual(lora_common.sanitize_text("Hello, World!"), "Hello, World!")

    def test_c0_nul(self):
        """NUL (U+0000) is escaped."""
        self.assertEqual(lora_common.sanitize_text("\x00"), "\\x00")

    def test_c0_bell(self):
        """BEL (U+0007) is escaped."""
        self.assertEqual(lora_common.sanitize_text("\x07"), "\\x07")

    def test_c0_tab(self):
        """TAB (U+0009) is escaped (C0 control)."""
        self.assertEqual(lora_common.sanitize_text("\t"), "\\x09")

    def test_c0_newline(self):
        """LF (U+000A) is escaped (C0 control)."""
        self.assertEqual(lora_common.sanitize_text("\n"), "\\x0a")

    def test_c0_carriage_return(self):
        """CR (U+000D) is escaped (C0 control)."""
        self.assertEqual(lora_common.sanitize_text("\r"), "\\x0d")

    def test_del(self):
        """DEL (U+007F) is escaped."""
        self.assertEqual(lora_common.sanitize_text("\x7f"), "\\x7f")

    def test_empty_string(self):
        """Empty string returns empty string."""
        self.assertEqual(lora_common.sanitize_text(""), "")

    def test_mixed_content(self):
        """Mixed printable + control chars."""
        result = lora_common.sanitize_text("OK\x00end")
        self.assertEqual(result, "OK\\x00end")

    def test_c1_control(self):
        """C1 control (U+009B = CSI) is escaped."""
        result = lora_common.sanitize_text("\u009b")
        self.assertEqual(result, "\\u009b")

    def test_bidi_override(self):
        """Bidi override (U+202E = RLO) is escaped."""
        result = lora_common.sanitize_text("\u202e")
        self.assertEqual(result, "\\u202e")

    def test_unicode_passthrough(self):
        """Normal Unicode (emoji, CJK) passes through."""
        self.assertEqual(lora_common.sanitize_text("café ☕ 你好"), "café ☕ 你好")


# ---- parse_host_port ----


class TestParseHostPort(unittest.TestCase):
    """Tests for lora_common.parse_host_port()."""

    def test_ipv4(self):
        host, port = lora_common.parse_host_port("127.0.0.1:5555")
        self.assertEqual(host, "127.0.0.1")
        self.assertEqual(port, 5555)

    def test_ipv6_bracket_with_port(self):
        host, port = lora_common.parse_host_port("[::1]:5555")
        self.assertEqual(host, "::1")
        self.assertEqual(port, 5555)

    def test_ipv6_bracket_default_port(self):
        host, port = lora_common.parse_host_port("[::1]", default_port=5556)
        self.assertEqual(host, "::1")
        self.assertEqual(port, 5556)

    def test_hostname_port(self):
        host, port = lora_common.parse_host_port("localhost:8080")
        self.assertEqual(host, "localhost")
        self.assertEqual(port, 8080)

    def test_no_colon_raises(self):
        with self.assertRaises(ValueError):
            lora_common.parse_host_port("no-colon-here")

    def test_port_only(self):
        """':5555' yields (empty host, 5555) via rfind(':') at index 0."""
        host, port = lora_common.parse_host_port(":5555")
        self.assertEqual(host, "")
        self.assertEqual(port, 5555)

    def test_default_port_ipv4(self):
        """Default port used when not specified in ipv4 format — but colon is required."""
        # parse_host_port requires a colon for non-bracket notation
        with self.assertRaises(ValueError):
            lora_common.parse_host_port("127.0.0.1")

    def test_ipv6_full_address(self):
        host, port = lora_common.parse_host_port("[2001:db8::1]:9999")
        self.assertEqual(host, "2001:db8::1")
        self.assertEqual(port, 9999)


if __name__ == "__main__":
    unittest.main()
