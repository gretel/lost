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
        text = lora_mon.format_frame(make_frame())
        self.assertIn("#1", text)
        self.assertIn("CRC_OK", text)
        self.assertIn("SF8", text)

    def test_crc_fail(self):
        text = lora_mon.format_frame(make_frame(crc_valid=False))
        self.assertIn("CRC_FAIL", text)

    def test_sync_word_label(self):
        self.assertIn("0x12", lora_mon.format_frame(make_frame(sync_word=0x12)))
        self.assertIn("0x2B", lora_mon.format_frame(make_frame(sync_word=0x2B)))
        self.assertIn("0x34", lora_mon.format_frame(make_frame(sync_word=0x34)))
        self.assertIn("0xFF", lora_mon.format_frame(make_frame(sync_word=0xFF)))

    def test_snr_shown(self):
        msg = make_frame()
        msg["phy"]["snr_db"] = 12.5
        text = lora_mon.format_frame(msg)
        self.assertIn("SNR=12.5dB", text)

    def test_rx_channel_shown(self):
        msg = make_frame()
        msg["rx_channel"] = 1
        text = lora_mon.format_frame(msg)
        self.assertIn("ch=1", text)

    def test_no_snr_when_absent(self):
        text = lora_mon.format_frame(make_frame())
        self.assertNotIn("SNR=", text)

    def test_no_channel_when_absent(self):
        text = lora_mon.format_frame(make_frame())
        self.assertNotIn("ch=", text)

    def test_noise_floor_shown(self):
        msg = make_frame()
        msg["phy"]["noise_floor_db"] = -45.3
        text = lora_mon.format_frame(msg)
        self.assertIn("NF=-45.3dBFS", text)

    def test_no_noise_floor_when_absent(self):
        text = lora_mon.format_frame(make_frame())
        self.assertNotIn("NF=", text)


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
        text = lora_mon.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
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
        text = lora_mon.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys={},
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
        text = lora_mon.format_frame(msg)
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
        text = lora_mon.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
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
        text = lora_mon.format_frame(
            msg,
            our_prv=self.dest_expanded,
            our_pub=self.dest_pub,
            known_keys=known_keys,
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
                results.append(lora_mon.format_frame(msg))
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


if __name__ == "__main__":
    unittest.main()
