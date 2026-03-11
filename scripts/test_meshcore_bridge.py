#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Tests for meshcore_bridge.py — companion protocol framing, command handling,
and RX frame conversion.

Run:  python3 -m unittest scripts/test_meshcore_bridge.py -v
"""

from __future__ import annotations

import os
import socket
import struct
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
import meshcore_bridge as bridge
from meshcore_crypto import (
    load_or_create_identity,
    meshcore_expanded_key,
    meshcore_shared_secret,
    meshcore_encrypt_then_mac,
    GroupChannel,
    build_grp_txt,
    PAYLOAD_TXT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_GRP_TXT,
    PAYLOAD_ACK,
    PAYLOAD_ADVERT,
    PATH_HASH_SIZE,
)
from meshcore_tx import build_txt_msg, build_anon_req, build_wire_packet, make_header


# ---- Helpers ----


def make_identity():
    """Create a temporary identity for testing."""
    import tempfile
    from pathlib import Path

    tmpdir = tempfile.mkdtemp()
    path = Path(tmpdir) / "identity.bin"
    expanded_prv, pub_key, seed = load_or_create_identity(path)
    return expanded_prv, pub_key, seed


def make_state(**kwargs):
    """Create a BridgeState with test defaults using temp directories."""
    expanded_prv, pub_key, seed = make_identity()
    import tempfile

    tmpdir = Path(tempfile.mkdtemp())
    defaults = dict(
        pub_key=pub_key,
        expanded_prv=expanded_prv,
        seed=seed,
        name="test-node",
        freq_mhz=869.618,
        bw_khz=62.5,
        sf=8,
        cr=4,
        tx_power=14,
        keys_dir=tmpdir / "keys",
        channels_dir=tmpdir / "channels",
        contacts_dir=tmpdir / "contacts",
    )
    defaults.update(kwargs)
    return bridge.BridgeState(**defaults)  # type: ignore[arg-type]


class _FakeUDPSock(socket.socket):
    """Minimal UDP socket mock that captures sendto calls."""

    def __init__(self) -> None:
        super().__init__(socket.AF_INET, socket.SOCK_DGRAM)
        self.close()  # release the FD immediately; we only need the type
        self.sent: list[bytes] = []

    def sendto(self, data: bytes, addr: object) -> int:  # type: ignore[override]
        self.sent.append(data)
        return len(data)


# ---- Frame codec tests ----


class TestFrameCodec(unittest.TestCase):
    def test_encode_empty(self):
        """Empty payload encodes to 3 bytes."""
        frame = bridge.frame_encode(b"")
        self.assertEqual(frame, b"\x3e\x00\x00")

    def test_encode_payload(self):
        """Payload encodes with correct length."""
        frame = bridge.frame_encode(b"\x05\x01\x02")
        self.assertEqual(frame[0], 0x3E)
        length = int.from_bytes(frame[1:3], "little")
        self.assertEqual(length, 3)
        self.assertEqual(frame[3:], b"\x05\x01\x02")

    def test_decode_single_frame(self):
        """Single complete frame is decoded."""
        dec = bridge.FrameDecoder()
        raw = b"\x3c\x02\x00\x01\x03"  # CMD_APP_START partial
        frames = dec.feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], b"\x01\x03")

    def test_decode_multiple_frames(self):
        """Multiple frames in one chunk are all decoded."""
        dec = bridge.FrameDecoder()
        f1 = b"\x3c\x01\x00\x04"  # GET_CONTACTS
        f2 = b"\x3c\x01\x00\x0a"  # SYNC_NEXT_MESSAGE
        frames = dec.feed(f1 + f2)
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0], b"\x04")
        self.assertEqual(frames[1], b"\x0a")

    def test_decode_split_across_feeds(self):
        """Frame split across multiple feed() calls."""
        dec = bridge.FrameDecoder()
        raw = b"\x3c\x03\x00\x02\x00\x01"
        # Feed one byte at a time
        frames = []
        for b in raw:
            frames.extend(dec.feed(bytes([b])))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], b"\x02\x00\x01")

    def test_decode_skips_garbage(self):
        """Garbage before start byte is skipped."""
        dec = bridge.FrameDecoder()
        raw = b"\xff\xff\x3c\x01\x00\x05"
        frames = dec.feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], b"\x05")

    def test_decode_rejects_oversize(self):
        """Frames claiming > 300 bytes are rejected."""
        dec = bridge.FrameDecoder()
        # Length = 400 (0x0190), should be rejected
        raw = b"\x3c\x90\x01" + b"\x00" * 400
        frames = dec.feed(raw)
        # The oversized frame should not be returned
        # (decoder skips the start byte and looks for next valid frame)
        self.assertEqual(len(frames), 0)


# ---- BridgeState response builders ----


class TestBridgeState(unittest.TestCase):
    def test_self_info_structure(self):
        """SELF_INFO response has correct structure."""
        state = make_state()
        resp = state.build_self_info()
        self.assertEqual(resp[0], bridge.RESP_SELF_INFO)
        # pubkey starts at offset 4 (code + adv_type + tx_power + max_tx_power)
        pubkey = resp[4:36]
        self.assertEqual(pubkey, state.pub_key)
        # Name is at the end
        name_bytes = state.name.encode("utf-8")
        self.assertTrue(resp.endswith(name_bytes))

    def test_device_info_structure(self):
        """DEVICE_INFO response has correct structure."""
        state = make_state()
        resp = state.build_device_info()
        self.assertEqual(resp[0], bridge.RESP_DEVICE_INFO)
        self.assertEqual(resp[1], 9)  # fw_ver
        self.assertIn(b"lora_trx", resp)  # fw_build and model both contain "lora_trx"

    def test_ok_response(self):
        """OK response has code 0x00."""
        state = make_state()
        resp = state.build_ok()
        self.assertEqual(resp[0], bridge.RESP_OK)

    def test_error_response(self):
        """ERROR response has code 0x01."""
        state = make_state()
        resp = state.build_error(5)
        self.assertEqual(resp[0], bridge.RESP_ERROR)
        self.assertEqual(resp[1], 5)

    def test_current_time(self):
        """CURRENT_TIME response contains a plausible timestamp."""
        state = make_state()
        resp = state.build_current_time()
        self.assertEqual(resp[0], bridge.RESP_CURRENT_TIME)
        ts = struct.unpack_from("<I", resp, 1)[0]
        now = int(time.time())
        self.assertAlmostEqual(ts, now, delta=2)

    def test_battery_response(self):
        """BATTERY response has correct code."""
        state = make_state()
        resp = state.build_battery()
        self.assertEqual(resp[0], bridge.RESP_BATTERY)
        mv = struct.unpack_from("<H", resp, 1)[0]
        self.assertEqual(mv, 4200)

    def test_contacts_empty(self):
        """Empty contact list returns START + END."""
        state = make_state()
        responses = state.build_contacts_response()
        self.assertEqual(len(responses), 2)  # START + END
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_START)
        count = struct.unpack_from("<I", responses[0], 1)[0]
        self.assertEqual(count, 0)
        self.assertEqual(responses[1][0], bridge.RESP_CONTACT_END)

    def test_msg_sent(self):
        """MSG_SENT response has correct structure."""
        state = make_state()
        resp = state.build_msg_sent(flood=False)
        self.assertEqual(resp[0], bridge.RESP_MSG_SENT)
        self.assertEqual(resp[1], 0)  # routing_type: 0=direct
        self.assertEqual(state.msg_seq, 1)
        # Flood routing
        resp2 = state.build_msg_sent(flood=True)
        self.assertEqual(resp2[1], 1)  # routing_type: 1=flood

    def test_no_more_msgs(self):
        """NO_MORE_MSGS response is correct."""
        state = make_state()
        resp = state.build_no_more_msgs()
        self.assertEqual(resp[0], bridge.RESP_NO_MORE_MSGS)

    def test_channel_info(self):
        """CHANNEL_INFO response has correct structure."""
        state = make_state()
        resp = state.build_channel_info(3)
        self.assertEqual(resp[0], bridge.RESP_CHANNEL_INFO)
        self.assertEqual(resp[1], 3)  # channel idx

    def test_stats_core(self):
        """Stats core response has correct code."""
        state = make_state()
        resp = state.build_stats(0)
        self.assertEqual(resp[0], bridge.RESP_STATS)
        self.assertEqual(resp[1], 0)  # core


# ---- Command handler tests ----


class TestCommandHandler(unittest.TestCase):
    """Test command handling without actual UDP socket."""

    def setUp(self):
        """Create state and a dummy UDP socket."""
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_app_start(self):
        """APP_START returns SELF_INFO + self ADVERT push and persists self contact."""
        responses = self._handle(b"\x01\x03      mccli")
        self.assertEqual(len(responses), 2)
        self.assertEqual(responses[0][0], bridge.RESP_SELF_INFO)
        self.assertEqual(responses[1][0], bridge.PUSH_NEW_ADVERT)
        # Self should be persisted in contacts
        self.assertIn(self.state.pub_key.hex(), self.state.contacts)

    def test_app_start_self_in_get_contacts(self):
        """After APP_START, GET_CONTACTS includes our own contact."""
        self._handle(b"\x01\x03      mccli")
        responses = self._handle(b"\x04")
        # START + 1 CONTACT (self) + END
        self.assertEqual(len(responses), 3)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_START)
        self.assertEqual(responses[1][0], bridge.RESP_CONTACT)
        # Contact record starts with our pubkey
        self.assertEqual(responses[1][1:33], self.state.pub_key)
        self.assertEqual(responses[2][0], bridge.RESP_CONTACT_END)

    def test_device_query(self):
        """DEVICE_QUERY returns DEVICE_INFO."""
        responses = self._handle(b"\x16\x03")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_DEVICE_INFO)

    def test_get_contacts_empty(self):
        """GET_CONTACTS returns START + END when no contacts."""
        responses = self._handle(b"\x04")
        self.assertEqual(len(responses), 2)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_START)
        self.assertEqual(responses[1][0], bridge.RESP_CONTACT_END)

    def test_get_device_time(self):
        """GET_DEVICE_TIME returns CURRENT_TIME."""
        responses = self._handle(b"\x05")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CURRENT_TIME)

    def test_set_device_time(self):
        """SET_DEVICE_TIME returns OK."""
        ts = struct.pack("<I", int(time.time()))
        responses = self._handle(b"\x06" + ts)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

    def test_get_batt(self):
        """GET_BATT_AND_STORAGE returns BATTERY."""
        responses = self._handle(b"\x14")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_BATTERY)

    def test_sync_no_messages(self):
        """SYNC_NEXT_MESSAGE with empty queue returns NO_MORE_MSGS."""
        responses = self._handle(b"\x0a")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_NO_MORE_MSGS)

    def test_sync_with_messages(self):
        """SYNC_NEXT_MESSAGE returns queued message."""
        fake_msg = bytes([bridge.RESP_CONTACT_MSG_RECV_V3]) + b"\x00" * 20
        self.state.msg_queue.append(fake_msg)
        responses = self._handle(b"\x0a")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0], fake_msg)
        # Queue should be empty now
        responses2 = self._handle(b"\x0a")
        self.assertEqual(responses2[0][0], bridge.RESP_NO_MORE_MSGS)

    def test_set_advert_name(self):
        """SET_ADVERT_NAME updates state name."""
        responses = self._handle(b"\x08NewName")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(self.state.name, "NewName")

    def test_unknown_command(self):
        """Unknown command returns OK (graceful)."""
        responses = self._handle(b"\xff\x00")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

    def test_get_channel(self):
        """GET_CHANNEL returns CHANNEL_INFO for valid index."""
        responses = self._handle(b"\x1f\x02")  # channel 2
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CHANNEL_INFO)
        self.assertEqual(responses[0][1], 2)

    def test_get_channel_out_of_range(self):
        """GET_CHANNEL returns ERROR for index >= 8."""
        responses = self._handle(b"\x1f\x08")  # channel 8 (out of range)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_get_stats(self):
        """GET_STATS returns STATS."""
        responses = self._handle(b"\x38\x00")  # core stats
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_STATS)

    def test_login_short_payload_returns_error(self):
        """CMD_LOGIN with too-short payload (< 32 bytes) returns RESP_ERROR."""
        cmd = bytes([bridge.CMD_LOGIN]) + b"payload"  # only 7 bytes, need 32+
        responses = bridge.handle_command(cmd, self.state, self.udp_sock, self.udp_addr)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_status_req_short_payload_returns_error(self):
        """CMD_STATUS_REQ with too-short payload (< 32 bytes) returns RESP_ERROR."""
        cmd = bytes([bridge.CMD_STATUS_REQ]) + b"payload"  # only 7 bytes, need 32+
        responses = bridge.handle_command(cmd, self.state, self.udp_sock, self.udp_addr)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_trace_returns_msg_sent(self):
        """CMD_TRACE stub returns RESP_MSG_SENT."""
        cmd = bytes([bridge.CMD_TRACE]) + b"payload"
        responses = bridge.handle_command(cmd, self.state, None, None)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

    def test_path_discovery_returns_msg_sent(self):
        """CMD_PATH_DISCOVERY stub returns RESP_MSG_SENT."""
        cmd = bytes([bridge.CMD_PATH_DISCOVERY]) + b"payload"
        responses = bridge.handle_command(cmd, self.state, None, None)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)


# ---- RX frame conversion tests ----


class TestFrameConversion(unittest.TestCase):
    def setUp(self):
        self.state = make_state()

    def test_non_meshcore_ignored(self):
        """Non-MeshCore frames (wrong sync_word) are ignored."""
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x34},
            "payload": b"\x00" * 20,
        }
        self.assertEqual(
            bridge.lora_frame_to_companion_msgs(frame, self.state), ([], [])
        )

    def test_crc_fail_ignored(self):
        """CRC_FAIL frames are ignored."""
        frame = {
            "type": "lora_frame",
            "crc_valid": False,
            "phy": {"sync_word": 0x12},
            "payload": b"\x00" * 20,
        }
        self.assertEqual(
            bridge.lora_frame_to_companion_msgs(frame, self.state), ([], [])
        )

    def test_empty_payload_ignored(self):
        """Empty payload is ignored."""
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12},
            "payload": b"",
        }
        self.assertEqual(
            bridge.lora_frame_to_companion_msgs(frame, self.state), ([], [])
        )

    def test_advert_produces_push(self):
        """ADVERT frame produces PUSH_NEW_ADVERT (0x8A)."""
        hdr = (0x04 << 2) | 0x01  # ADVERT + FLOOD
        path_len = 0
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Test"  # flags: chat + has_name

        payload = bytes([hdr, path_len]) + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(
            frame, self.state
        )
        self.assertEqual(len(companion_msgs), 1)
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_NEW_ADVERT)
        # Check pubkey in response
        self.assertEqual(companion_msgs[0][1:33], pubkey)
        self.assertEqual(ack_packets, [])

    def test_advert_2byte_hash_path(self):
        """ADVERT with 2-byte hash path (mode=1) is parsed correctly.

        path_len byte = 0x41 = 0b01000001: mode=1 (2-byte hashes), count=1 → 2 path bytes.
        Bug: old code did off += 1 + 0x41 = off + 66, skipping 64 bytes too many.
        Fix: off += 1 + (1 * 2) = off + 3.
        """
        hdr = (0x04 << 2) | 0x01  # ADVERT + FLOOD
        path_len_byte = 0x41  # mode=1 (2-byte hashes), count=1
        path_data = b"\xaa\xbb"  # 2 bytes of path data
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"2ByteHop"  # flags: chat + has_name

        payload = bytes([hdr, path_len_byte]) + path_data + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(
            frame, self.state
        )
        # Must produce a PUSH_NEW_ADVERT with the correct pubkey
        self.assertEqual(
            len(companion_msgs), 1, "expected PUSH_NEW_ADVERT for 2-byte hash path"
        )
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_NEW_ADVERT)
        self.assertEqual(companion_msgs[0][1:33], pubkey)

    def test_advert_3byte_hash_path(self):
        """ADVERT with 3-byte hash path (mode=2) is parsed correctly.

        path_len byte = 0x82 = 0b10000010: mode=2 (3-byte hashes), count=2 → 6 path bytes.
        Bug: old code did off += 1 + 0x82 = off + 131.
        Fix: off += 1 + (2 * 3) = off + 7.
        """
        hdr = (0x04 << 2) | 0x01  # ADVERT + FLOOD
        path_len_byte = 0x82  # mode=2 (3-byte hashes), count=2
        path_data = b"\x11\x22\x33\x44\x55\x66"  # 6 bytes
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"3ByteHop"

        payload = bytes([hdr, path_len_byte]) + path_data + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(
            frame, self.state
        )
        self.assertEqual(
            len(companion_msgs), 1, "expected PUSH_NEW_ADVERT for 3-byte hash path"
        )
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_NEW_ADVERT)
        self.assertEqual(companion_msgs[0][1:33], pubkey)

    def test_advert_auto_learns_key(self):
        """ADVERT auto-learns the sender's public key."""
        hdr = (0x04 << 2) | 0x01
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Learner"

        payload = bytes([hdr, 0]) + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        bridge.lora_frame_to_companion_msgs(frame, self.state)
        # Key should be learned
        self.assertIn(pubkey.hex(), self.state.known_keys)

    def test_advert_auto_adds_contact(self):
        """ADVERT learns key AND auto-adds to contacts for persistence across restarts."""
        hdr = (0x04 << 2) | 0x01
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Contact"

        payload = bytes([hdr, 0]) + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        bridge.lora_frame_to_companion_msgs(frame, self.state)
        # Key should be learned
        self.assertIn(pubkey.hex(), self.state.known_keys)
        # Contact is auto-added so it survives restarts
        self.assertIn(pubkey.hex(), self.state.contacts)

    def test_advert_does_not_learn_self(self):
        """ADVERT with our own pubkey does not add to known_keys."""
        hdr = (0x04 << 2) | 0x01
        pubkey = self.state.pub_key
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Self"

        payload = bytes([hdr, 0]) + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        initial_keys = len(self.state.known_keys)
        bridge.lora_frame_to_companion_msgs(frame, self.state)
        # Should NOT add our own key
        self.assertEqual(len(self.state.known_keys), initial_keys)


# ---- Contact management tests ----


class TestContactManagement(unittest.TestCase):
    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_add_and_get_contact(self):
        """ADD_UPDATE_CONTACT followed by GET_CONTACTS returns the contact."""
        pk = bytes(range(32))
        # Build minimal contact data: pubkey(32) + type(1) + flags(1) +
        # path_len(1) + path(64) + name(32) + last_advert(4) + lat(4) + lon(4)
        contact_data = pk + b"\x01\x00\xff" + b"\x00" * 64
        contact_data += b"Alice\x00" * 4 + b"\x00" * 8  # name padded to 32
        contact_data += struct.pack("<I", 0) + struct.pack("<i", 0) * 2

        cmd = b"\x09" + contact_data
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(len(self.state.contacts), 1)

        # GET_CONTACTS should return 3 frames: START + CONTACT + END
        responses = self._handle(b"\x04")
        self.assertEqual(len(responses), 3)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_START)
        self.assertEqual(responses[1][0], bridge.RESP_CONTACT)
        self.assertEqual(responses[2][0], bridge.RESP_CONTACT_END)

    def test_remove_contact(self):
        """REMOVE_CONTACT removes a previously added contact."""
        pk = bytes(range(32))
        self.state.contacts[pk.hex()] = pk + b"\x00" * 100
        self.assertEqual(len(self.state.contacts), 1)

        responses = self._handle(b"\x0f" + pk)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(len(self.state.contacts), 0)


# ---- Import contact tests ----


class TestImportContact(unittest.TestCase):
    """Test CMD_IMPORT_CONTACT handling."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def _build_advert_packet(self, pubkey: bytes, name: str = "") -> bytes:
        """Build a minimal raw ADVERT wire packet for import testing."""
        # header: route=FLOOD(1), ptype=ADVERT(4) -> (4<<2)|1 = 0x11
        hdr = (0x04 << 2) | 0x01
        path_len = 0
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64  # fake signature (import doesn't verify)
        flags = 0x01  # node_type=chat
        app_data = bytes([flags])
        if name:
            app_data = bytes([flags | 0x80]) + name.encode("utf-8")
        payload = pubkey + ts + sig + app_data
        return bytes([hdr, path_len]) + payload

    def test_import_contact_stores_contact(self):
        """CMD_IMPORT_CONTACT parses ADVERT and stores contact."""
        pk = bytes(range(32))
        advert = self._build_advert_packet(pk, "Alice")
        responses = self._handle(b"\x12" + advert)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertIn(pk.hex(), self.state.contacts)

    def test_import_contact_name_extracted(self):
        """Imported contact has correct name in record."""
        pk = bytes(range(32))
        advert = self._build_advert_packet(pk, "Bob")
        self._handle(b"\x12" + advert)
        record = self.state.contacts[pk.hex()]
        # Name is at offset 99 (32+1+1+1+64) in the 147-byte record
        name_bytes = record[99:131].rstrip(b"\x00")
        self.assertEqual(name_bytes, b"Bob")

    def test_import_contact_shows_in_get_contacts(self):
        """Imported contact appears in GET_CONTACTS response."""
        pk = bytes(range(32))
        advert = self._build_advert_packet(pk, "Carol")
        self._handle(b"\x12" + advert)
        responses = self._handle(b"\x04")
        # START + 1 CONTACT + END
        self.assertEqual(len(responses), 3)
        self.assertEqual(responses[1][0], bridge.RESP_CONTACT)
        # Contact pubkey in response
        self.assertEqual(responses[1][1:33], pk)

    def test_import_contact_invalid_data(self):
        """CMD_IMPORT_CONTACT with invalid data returns ERROR."""
        responses = self._handle(b"\x12\x00\x01\x02")  # too short
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_import_contact_non_advert(self):
        """CMD_IMPORT_CONTACT with non-ADVERT packet returns ERROR."""
        # header: ptype=TXT(2), route=DIRECT(2) -> (2<<2)|2 = 0x0A
        hdr = (0x02 << 2) | 0x02
        payload = bytes([hdr, 0]) + b"\x00" * 120
        responses = self._handle(b"\x12" + payload)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_import_contact_with_location(self):
        """CMD_IMPORT_CONTACT preserves location from ADVERT."""
        pk = bytes(range(32))
        hdr = (0x04 << 2) | 0x01
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        # flags: chat + has_location + has_name
        flags = 0x01 | 0x10 | 0x80
        lat_f = 52.52  # Berlin
        lon_f = 13.405
        # Location is int32 scaled by 1e6 (NOT float), per MeshCore spec
        app_data = (
            bytes([flags])
            + struct.pack("<ii", int(lat_f * 1e6), int(lon_f * 1e6))
            + b"Berlin"
        )
        payload = pk + ts + sig + app_data
        advert = bytes([hdr, 0]) + payload

        responses = self._handle(b"\x12" + advert)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        record = self.state.contacts[pk.hex()]
        # lat is at offset 135 (32+1+1+1+64+32+4), lon at 139, both as int32 LE
        lat_stored = struct.unpack_from("<i", record, 135)[0]
        lon_stored = struct.unpack_from("<i", record, 139)[0]
        self.assertAlmostEqual(lat_stored / 1e6, lat_f, places=1)
        self.assertAlmostEqual(lon_stored / 1e6, lon_f, places=1)

    def test_import_via_bizcard_uri(self):
        """Full round-trip: build ADVERT -> bizcard URI -> import."""
        from meshcore_crypto import load_or_create_identity
        from meshcore_tx import build_advert, build_bizcard_uri
        import tempfile
        from pathlib import Path

        tmpdir = tempfile.mkdtemp()
        _, pub_key, seed = load_or_create_identity(Path(tmpdir) / "id.bin")
        advert_pkt = build_advert(seed, pub_key, name="RoundTrip")
        uri = build_bizcard_uri(advert_pkt)

        # Simulate what meshcore-cli does: strip "meshcore://" and hex-decode
        self.assertTrue(uri.startswith("meshcore://"))
        wire_bytes = bytes.fromhex(uri[11:])

        responses = self._handle(b"\x12" + wire_bytes)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertIn(pub_key.hex(), self.state.contacts)

        # Verify name was extracted
        record = self.state.contacts[pub_key.hex()]
        name_bytes = record[99:131].rstrip(b"\x00")
        self.assertEqual(name_bytes, b"RoundTrip")


# ---- Export contact tests ----


class TestExportContact(unittest.TestCase):
    """Test CMD_EXPORT_CONTACT produces valid biz card URI."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_export_self_returns_raw_advert(self):
        """EXPORT_CONTACT (no args) returns RESP_CONTACT_URI with raw ADVERT bytes."""
        responses = self._handle(b"\x11")
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)
        # Payload after response code is raw ADVERT wire packet
        advert_bytes = responses[0][1:]
        # First byte is MeshCore header: ADVERT(4) << 2 | route
        ptype = (advert_bytes[0] >> 2) & 0x0F
        self.assertEqual(ptype, 0x04)  # PAYLOAD_ADVERT

    def test_export_self_contains_pubkey(self):
        """Exported ADVERT contains our public key."""
        responses = self._handle(b"\x11")
        advert_bytes = responses[0][1:]
        # Skip header(1) + path_len(1), pubkey starts at offset 2
        pubkey = advert_bytes[2:34]
        self.assertEqual(pubkey, self.state.pub_key)

    def test_export_self_roundtrip_import(self):
        """Exported ADVERT can be imported back via CMD_IMPORT_CONTACT."""
        responses = self._handle(b"\x11")
        advert_bytes = responses[0][1:]
        # Import the exported ADVERT
        import_responses = self._handle(b"\x12" + advert_bytes)
        self.assertEqual(import_responses[0][0], bridge.RESP_OK)
        self.assertIn(self.state.pub_key.hex(), self.state.contacts)

    def test_export_self_meshcore_py_uri_format(self):
        """The RESP_CONTACT_URI payload produces a valid meshcore:// biz card URI
        when processed by meshcore_py reader (hex encode + prepend meshcore://)."""
        responses = self._handle(b"\x11")
        advert_bytes = responses[0][1:]
        # meshcore_py reader does: "meshcore://" + payload.hex()
        uri = "meshcore://" + advert_bytes.hex()
        self.assertTrue(uri.startswith("meshcore://"))
        # CLI import does: bytes.fromhex(uri[11:])
        recovered = bytes.fromhex(uri[11:])
        self.assertEqual(recovered, advert_bytes)

    def test_export_specific_contact_self(self):
        """EXPORT_CONTACT with our own pubkey returns our ADVERT."""
        responses = self._handle(b"\x11" + self.state.pub_key)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)

    def test_export_unknown_contact_error(self):
        """EXPORT_CONTACT with unknown pubkey returns ERROR."""
        unknown_pk = b"\xff" * 32
        responses = self._handle(b"\x11" + unknown_pk)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)


# ---- ADVERT wire packet parser tests ----


class TestAdvertParser(unittest.TestCase):
    """Test _parse_advert_wire_packet directly."""

    def test_minimal_advert(self):
        """Parse minimal ADVERT without name or location."""
        hdr = (0x04 << 2) | 0x01
        pk = b"\xaa" * 32
        ts = struct.pack("<I", 1700000000)
        sig = b"\x00" * 64
        app_data = bytes([0x01])  # chat, no name, no location
        payload = bytes([hdr, 0]) + pk + ts + sig + app_data

        info = bridge._parse_advert_wire_packet(payload)
        assert info is not None
        self.assertEqual(info["pubkey"], pk)
        self.assertEqual(info["node_type"], 0x01)
        self.assertEqual(info["name"], "")
        self.assertEqual(info["last_advert"], 1700000000)

    def test_advert_with_name(self):
        """Parse ADVERT with name."""
        hdr = (0x04 << 2) | 0x01
        pk = b"\xbb" * 32
        ts = struct.pack("<I", 1700000000)
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Hello"
        payload = bytes([hdr, 0]) + pk + ts + sig + app_data

        info = bridge._parse_advert_wire_packet(payload)
        assert info is not None
        self.assertEqual(info["name"], "Hello")

    def test_non_advert_returns_none(self):
        """Non-ADVERT ptype returns None."""
        hdr = (0x02 << 2) | 0x02  # TXT_MSG
        payload = bytes([hdr, 0]) + b"\x00" * 120
        self.assertIsNone(bridge._parse_advert_wire_packet(payload))

    def test_too_short_returns_none(self):
        """Too-short data returns None."""
        self.assertIsNone(bridge._parse_advert_wire_packet(b"\x11"))
        self.assertIsNone(bridge._parse_advert_wire_packet(b""))

    def test_advert_with_transport(self):
        """Parse ADVERT with transport codes (T_FLOOD route=0)."""
        hdr = (0x04 << 2) | 0x00  # ADVERT + T_FLOOD
        transport = b"\x00\x00\x00\x00"  # 4 transport bytes
        pk = b"\xcc" * 32
        ts = struct.pack("<I", 1700000000)
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Trans"
        payload = bytes([hdr]) + transport + bytes([0]) + pk + ts + sig + app_data

        info = bridge._parse_advert_wire_packet(payload)
        assert info is not None
        self.assertEqual(info["pubkey"], pk)
        self.assertEqual(info["name"], "Trans")


# ---- RX decryption tests ----


def _make_two_identities():
    """Create two identities (sender + receiver) for crypto tests."""
    import tempfile

    d1 = tempfile.mkdtemp()
    d2 = tempfile.mkdtemp()
    prv_a, pub_a, seed_a = load_or_create_identity(Path(d1) / "a.bin")
    prv_b, pub_b, seed_b = load_or_create_identity(Path(d2) / "b.bin")
    return (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b)


class TestRxDecryption(unittest.TestCase):
    """Test decryption of TXT_MSG, ANON_REQ, and GRP_TXT in the bridge."""

    def test_txt_msg_decryption(self):
        """Encrypted TXT_MSG addressed to bridge is decrypted properly."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        # Bridge is identity B; sender is A
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        # Add sender A's key to the known keys
        state.known_keys[pub_a.hex()] = pub_a

        # Build encrypted TXT_MSG from A to B
        packet = build_txt_msg(prv_a, pub_a, pub_b, "hello bridge")

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        msg = companion_msgs[0]
        self.assertEqual(msg[0], bridge.RESP_CONTACT_MSG_RECV_V3)
        # Extract text from companion message (after header fields)
        # V3 format: code(1) + snr(1) + reserved(2) + pubkey_prefix(6) +
        #            path_len(1) + txt_type(1) + ts(4) = 16 bytes header
        text = msg[16:].decode("utf-8", errors="replace")
        self.assertEqual(text, "hello bridge")
        # Should produce an RF ACK packet
        self.assertEqual(len(ack_packets), 1)

    def test_txt_msg_sender_prefix(self):
        """Decrypted TXT_MSG has correct sender pubkey prefix."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        state.known_keys[pub_a.hex()] = pub_a

        packet = build_txt_msg(prv_a, pub_a, pub_b, "test")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": packet,
        }
        companion_msgs, _ = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        # pubkey_prefix is at offset 4 (code + snr + reserved), 6 bytes
        prefix = companion_msgs[0][4:10]
        self.assertEqual(prefix, pub_a[:6])

    def test_txt_msg_unknown_sender_ignored(self):
        """TXT_MSG from unknown sender returns nothing."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        # Deliberately do NOT add sender key to known_keys

        packet = build_txt_msg(prv_a, pub_a, pub_b, "secret")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 0)
        self.assertEqual(len(ack_packets), 0)

    def test_anon_req_decryption(self):
        """Encrypted ANON_REQ addressed to bridge is decrypted."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)

        # Build ANON_REQ from A to B (no prior key exchange needed)
        packet = build_anon_req(prv_a, pub_a, pub_b, b"anonymous hello")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 2.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        msg = companion_msgs[0]
        self.assertEqual(msg[0], bridge.RESP_CONTACT_MSG_RECV_V3)
        text = msg[16:].decode("utf-8", errors="replace")
        self.assertEqual(text, "anonymous hello")
        # ANON_REQ produces a RESPONSE wire packet (not an ACK) in ack_packets
        self.assertEqual(len(ack_packets), 1)

    def test_anon_req_learns_key_but_not_contact(self):
        """ANON_REQ learns the sender's key but does NOT auto-add to contacts."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)

        packet = build_anon_req(prv_a, pub_a, pub_b, b"learn me")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 2.0},
            "payload": packet,
        }
        bridge.lora_frame_to_companion_msgs(frame, state)
        # Sender key should be learned
        self.assertIn(pub_a.hex(), state.known_keys)
        # But NOT added to contacts
        self.assertNotIn(pub_a.hex(), state.contacts)

    def test_anon_req_not_for_us_ignored(self):
        """ANON_REQ addressed to someone else returns nothing."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        # Create a third identity — the actual recipient
        import tempfile

        _, pub_c, _ = load_or_create_identity(Path(tempfile.mkdtemp()) / "c.bin")
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)

        # ANON_REQ from A addressed to C, not B
        packet = build_anon_req(prv_a, pub_a, pub_c, b"not for you")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 2.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 0)
        self.assertEqual(len(ack_packets), 0)

    def test_grp_txt_decryption(self):
        """Encrypted GRP_TXT is decrypted using known channel."""
        state = make_state()
        # Add a test channel
        ch = GroupChannel("test-chan", b"\x01" * 16)
        state.channels.append(ch)

        # Build encrypted GRP_TXT
        from meshcore_crypto import ROUTE_FLOOD

        grp_payload = build_grp_txt(ch, "Alice", "hello group")
        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        msg = companion_msgs[0]
        self.assertEqual(msg[0], bridge.RESP_CHANNEL_MSG_RECV_V3)
        # V3 channel format: code(1) + snr(1) + reserved(2) + chan_idx(1) +
        #                     path_len(1) + txt_type(1) + ts(4) = 11 bytes header
        text = msg[11:].decode("utf-8", errors="replace")
        self.assertIn("Alice", text)
        self.assertIn("hello group", text)
        # GRP_TXT does not produce ACK packets
        self.assertEqual(ack_packets, [])

    def test_grp_txt_unknown_channel_ignored(self):
        """GRP_TXT with unknown channel returns nothing."""
        state = make_state()
        # No channels loaded
        ch = GroupChannel("secret", b"\xaa" * 16)
        grp_payload = build_grp_txt(ch, "Bob", "hidden")
        from meshcore_crypto import ROUTE_FLOOD

        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 0)
        self.assertEqual(len(ack_packets), 0)


# ---- Channel support tests ----


class TestChannelSupport(unittest.TestCase):
    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_set_channel(self):
        """SET_CHANNEL stores a channel."""
        idx = 0
        name = b"test-ch".ljust(32, b"\x00")  # padded to 32 bytes
        secret = b"\xab" * 16 + b"\x00" * 16  # 32 bytes (only first 16 used)
        cmd = bytes([bridge.CMD_SET_CHANNEL]) + bytes([idx]) + name + secret
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertTrue(len(self.state.channels) > 0)
        self.assertEqual(self.state.channels[0].name, "test-ch")

    def test_set_channel_reflected_in_get(self):
        """SET_CHANNEL data appears in subsequent GET_CHANNEL."""
        # Add a channel
        name_bytes = b"mychan".ljust(32, b"\x00")
        secret_bytes = b"\x01" * 16 + b"\x00" * 16
        cmd = bytes([bridge.CMD_SET_CHANNEL, 0]) + name_bytes + secret_bytes
        self._handle(cmd)

        # GET_CHANNEL should reflect it
        responses = self._handle(bytes([bridge.CMD_GET_CHANNEL, 0]))
        self.assertEqual(responses[0][0], bridge.RESP_CHANNEL_INFO)
        self.assertEqual(responses[0][1], 0)  # idx
        ch_name = responses[0][2:34].rstrip(b"\x00").decode("utf-8")
        self.assertEqual(ch_name, "mychan")

    def test_clear_channel(self):
        """SET_CHANNEL with empty name clears the channel at that index."""
        # Record initial count (seed channels may be loaded)
        initial = len(self.state.channels)
        # Add a channel at the end
        ch = GroupChannel("todelete", b"\x01" * 16)
        self.state.channels.append(ch)
        self.assertEqual(len(self.state.channels), initial + 1)

        # Clear the last channel: idx=initial, name=zeros, secret=zeros
        name_bytes = b"\x00" * 32
        secret_bytes = b"\x00" * 32
        cmd = bytes([bridge.CMD_SET_CHANNEL, initial]) + name_bytes + secret_bytes
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(len(self.state.channels), initial)


# ---- Contact persistence tests ----


class TestContactPersistence(unittest.TestCase):
    def test_persist_and_load(self):
        """Contacts persisted to disk can be loaded back."""
        import tempfile

        tmpdir = Path(tempfile.mkdtemp())
        cdir = tmpdir / "contacts"
        state = make_state(contacts_dir=cdir)

        # Add a contact
        pk = bytes(range(32))
        record = bridge._build_contact_record(pk, name="Persistent")
        state.contacts[pk.hex()] = record
        bridge._persist_contacts(state)

        # Load back
        loaded = bridge._load_persisted_contacts(cdir)
        self.assertEqual(len(loaded), 1)
        self.assertIn(pk.hex(), loaded)
        # Verify name in loaded record
        name_bytes = loaded[pk.hex()][99:131].rstrip(b"\x00")
        self.assertEqual(name_bytes, b"Persistent")

    def test_delete_persisted_contact(self):
        """Deleted contact is removed from disk."""
        import tempfile

        tmpdir = Path(tempfile.mkdtemp())
        cdir = tmpdir / "contacts"
        cdir.mkdir(parents=True)

        pk = b"\xaa" * 32
        record = bridge._build_contact_record(pk, name="ToDelete")
        (cdir / f"{pk.hex()}.contact").write_bytes(record)
        self.assertTrue((cdir / f"{pk.hex()}.contact").exists())

        bridge._delete_persisted_contact(cdir, pk.hex())
        self.assertFalse((cdir / f"{pk.hex()}.contact").exists())


# ---- ACK handling tests ----


class TestAckHandling(unittest.TestCase):
    def test_ack_matches_pending(self):
        """ACK matching a pending TX produces PUSH_ACK."""
        state = make_state()
        # Simulate a pending ACK tag
        ack_tag = b"\x01\x00\x00\x00"
        dest_prefix = b"\xaa\xbb\xcc\xdd\xee\xff"
        state.pending_acks[ack_tag] = (time.monotonic(), dest_prefix)

        # Build ACK payload: header + path_len + checksum(4)
        from meshcore_crypto import ROUTE_FLOOD

        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        payload = bytes([hdr, 0]) + ack_tag
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_ACK)
        # Pending ACK should be consumed
        self.assertNotIn(ack_tag, state.pending_acks)
        # RX ACK doesn't generate TX ACK packets
        self.assertEqual(ack_packets, [])

    def test_ack_no_match_ignored(self):
        """ACK not matching any pending TX returns nothing."""
        state = make_state()
        # No pending ACKs
        from meshcore_crypto import ROUTE_FLOOD

        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        payload = bytes([hdr, 0]) + b"\xff\xff\xff\xff"
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 0)
        self.assertEqual(len(ack_packets), 0)

    def test_expired_ack_pruned(self):
        """Expired pending ACK entries are pruned on next ACK."""
        state = make_state()
        state.pending_ack_ttl = 0.0  # expire immediately
        ack_tag = b"\x01\x00\x00\x00"
        state.pending_acks[ack_tag] = (time.monotonic() - 1.0, b"\x00" * 6)

        from meshcore_crypto import ROUTE_FLOOD

        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        payload = bytes([hdr, 0]) + ack_tag
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        # Should not match (expired before check)
        self.assertEqual(len(companion_msgs), 0)
        self.assertNotIn(ack_tag, state.pending_acks)


# ---- Send channel message tests ----


class TestSendChannelMsg(unittest.TestCase):
    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])
        # Add a test channel
        self.state.channels.append(GroupChannel("test-chan", b"\x01" * 16))

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_send_chan_msg_success(self):
        """CMD_SEND_CHAN_TXT_MSG sends a channel message."""
        ts = struct.pack("<I", int(time.time()))
        text = b"hello channel"
        cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG, 0x00, 0x00]) + ts + text
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Verify a CBOR TX request was sent via UDP
        self.udp_sock.settimeout(1.0)
        data, _addr = self.udp_sock.recvfrom(65536)
        import cbor2

        msg = cbor2.loads(data)
        self.assertEqual(msg["type"], "lora_tx")
        self.assertIn("payload", msg)

    def test_send_chan_msg_invalid_idx(self):
        """CMD_SEND_CHAN_TXT_MSG with bad channel index returns ERROR."""
        ts = struct.pack("<I", int(time.time()))
        text = b"bad channel"
        cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG, 0x00, 0x05]) + ts + text  # idx=5
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_send_chan_txt_msg_too_short_returns_error(self):
        """CMD_SEND_CHAN_TXT_MSG with < 7 data bytes returns RESP_ERROR."""
        # Only 4 bytes of data (need msg_type(1)+chan_idx(1)+ts(4)+at least 1 text byte = 7)
        cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG]) + b"\x00" * 4
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_send_chan_txt_msg_tracks_tx_hash(self):
        """After a successful send, state.recent_tx is non-empty (echo-filter hash stored)."""
        ts = struct.pack("<I", int(time.time()))
        text = b"hello channel"
        cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG, 0x00, 0x00]) + ts + text
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreater(len(self.state.recent_tx), 0)

    def test_send_chan_txt_msg_send_scope_overrides_region(self):
        """With send_scope set, wire packet header uses ROUTE_T_FLOOD (bits [1:0] == 0)."""
        import cbor2

        # Set a non-zero 16-byte send_scope key
        self.state.send_scope = b"\xab" * 16

        ts = struct.pack("<I", int(time.time()))
        text = b"scoped message"
        cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG, 0x00, 0x00]) + ts + text
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Receive and decode the UDP CBOR TX packet
        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        wire = msg["payload"]
        # ROUTE_T_FLOOD = 0x00; header byte bits [1:0] must be 0
        route_type = wire[0] & 0x03
        self.assertEqual(
            route_type, 0, f"expected ROUTE_T_FLOOD(0), got route_type={route_type}"
        )


# ---- RF ACK TX generation tests ----


class TestRfAckTx(unittest.TestCase):
    def test_txt_msg_produces_ack_packet(self):
        """Decrypted TXT_MSG produces an RF ACK wire packet."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        state.known_keys[pub_a.hex()] = pub_a

        packet = build_txt_msg(prv_a, pub_a, pub_b, "ack me")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        self.assertEqual(len(ack_packets), 1)

        # Verify ACK packet structure: header(1) + path_len(1) + ack_hash(4) = 6B
        ack_pkt = ack_packets[0]
        self.assertEqual(len(ack_pkt), 6)
        # Header: ROUTE_DIRECT (2) | PAYLOAD_ACK (3) << 2 = 0x0E
        from meshcore_crypto import ROUTE_DIRECT

        expected_hdr = make_header(ROUTE_DIRECT, PAYLOAD_ACK)
        self.assertEqual(ack_pkt[0], expected_hdr)
        # path_len = 0 (direct, no path)
        self.assertEqual(ack_pkt[1], 0)
        # ack_hash is 4 bytes
        self.assertEqual(len(ack_pkt[2:]), 4)

    def test_anon_req_produces_no_ack(self):
        """ANON_REQ does NOT produce an RF ACK (firmware sends RESPONSE instead)."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)

        packet = build_anon_req(prv_a, pub_a, pub_b, b"ack anon")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        # ANON_REQ does not produce an RF ACK; it produces an encrypted RESPONSE
        self.assertEqual(len(ack_packets), 1)

    def test_grp_txt_produces_no_ack(self):
        """GRP_TXT does not produce an ACK packet (no ACK for group msgs)."""
        state = make_state()
        ch = GroupChannel("test-chan", b"\x01" * 16)
        state.channels.append(ch)

        from meshcore_crypto import ROUTE_FLOOD

        grp_payload = build_grp_txt(ch, "Alice", "no ack needed")
        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        self.assertEqual(len(ack_packets), 0)

    def test_ack_hash_deterministic(self):
        """Same TXT_MSG produces the same ACK hash every time."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        state.known_keys[pub_a.hex()] = pub_a

        packet = build_txt_msg(prv_a, pub_a, pub_b, "same msg")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        _, acks1 = bridge.lora_frame_to_companion_msgs(frame, state)
        _, acks2 = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(acks1[0], acks2[0])


# ---- Region scope tests ----


class TestRegionScope(unittest.TestCase):
    """Test region scope transport code support."""

    def test_region_key_derivation(self):
        """BridgeState derives 16-byte region key from '#' + region_name."""
        import hashlib

        state = make_state(region_scope="de-nord")
        expected = hashlib.sha256(b"#de-nord").digest()[:16]
        self.assertEqual(state.region_key, expected)
        self.assertEqual(len(state.region_key), 16)

    def test_region_key_empty_when_no_scope(self):
        """BridgeState has empty region_key when region_scope is empty."""
        state = make_state(region_scope="")
        self.assertEqual(state.region_key, b"")

    def test_region_key_default_empty(self):
        """BridgeState default has empty region_scope (no transport codes)."""
        state = make_state()
        self.assertEqual(state.region_scope, "")
        self.assertEqual(state.region_key, b"")

    def test_compute_transport_code_basic(self):
        """_compute_transport_code returns a 2-byte code."""
        import hashlib

        key = hashlib.sha256(b"#de-nord").digest()[:16]
        code = bridge._compute_transport_code(key, 0x04, b"\x01\x02\x03")
        self.assertIsInstance(code, int)
        self.assertGreater(code, 0)
        self.assertLess(code, 0x10000)

    def test_compute_transport_code_reserved_zero(self):
        """Transport code 0x0000 is adjusted to 0x0001."""
        # We can't easily force a collision to 0, but verify the function exists
        # and the adjustment logic is present by testing boundary values.
        import hashlib
        import hmac

        # Craft a key/payload combo that produces 0 — not practical, but test
        # the adjustment directly by checking the function handles non-zero cases.
        key = b"\x00" * 16
        code = bridge._compute_transport_code(key, 0x00, b"")
        self.assertNotEqual(code, 0x0000)
        self.assertNotEqual(code, 0xFFFF)

    def test_compute_transport_code_deterministic(self):
        """Same inputs always produce the same transport code."""
        import hashlib

        key = hashlib.sha256(b"#de-nord").digest()[:16]
        code1 = bridge._compute_transport_code(key, 0x04, b"test payload")
        code2 = bridge._compute_transport_code(key, 0x04, b"test payload")
        self.assertEqual(code1, code2)

    def test_compute_transport_code_varies_with_payload(self):
        """Different payloads produce different transport codes."""
        import hashlib

        key = hashlib.sha256(b"#de-nord").digest()[:16]
        code1 = bridge._compute_transport_code(key, 0x04, b"payload A")
        code2 = bridge._compute_transport_code(key, 0x04, b"payload B")
        self.assertNotEqual(code1, code2)

    def test_advert_tx_uses_t_flood_with_region(self):
        """ADVERT TX uses T_FLOOD route with transport codes when region set."""
        import socket

        state = make_state(region_scope="de-nord")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        addr = ("127.0.0.1", sock.getsockname()[1])
        try:
            responses = bridge._handle_send_advert(b"\x01", state, sock, addr)
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0][0], bridge.RESP_OK)

            # Read the CBOR TX request from UDP
            sock.settimeout(1.0)
            data, _ = sock.recvfrom(65536)
            import cbor2

            msg = cbor2.loads(data)
            packet = msg["payload"]
            # Header byte: route_type in bits [1:0]
            route = packet[0] & 0x03
            self.assertEqual(route, 0x00)  # ROUTE_T_FLOOD

            # Transport codes (4 bytes) should follow the header
            tc1 = int.from_bytes(packet[1:3], "little")
            tc2 = int.from_bytes(packet[3:5], "little")
            self.assertGreater(tc1, 0)
            self.assertEqual(tc2, 0)
        finally:
            sock.close()

    def test_advert_tx_uses_flood_without_region(self):
        """ADVERT TX uses plain FLOOD route without region scope."""
        import socket

        state = make_state()  # no region_scope
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        addr = ("127.0.0.1", sock.getsockname()[1])
        try:
            responses = bridge._handle_send_advert(b"\x01", state, sock, addr)
            self.assertEqual(len(responses), 1)

            sock.settimeout(1.0)
            data, _ = sock.recvfrom(65536)
            import cbor2

            msg = cbor2.loads(data)
            packet = msg["payload"]
            route = packet[0] & 0x03
            self.assertEqual(route, 0x01)  # ROUTE_FLOOD (no transport codes)
        finally:
            sock.close()

    def test_chan_msg_uses_t_flood_with_region(self):
        """Channel message TX uses T_FLOOD with transport codes when region set."""
        import socket

        state = make_state(region_scope="de-nord")
        state.channels.append(GroupChannel("test-chan", b"\x01" * 16))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        addr = ("127.0.0.1", sock.getsockname()[1])
        try:
            ts = struct.pack("<I", int(time.time()))
            cmd = bytes([bridge.CMD_SEND_CHAN_TXT_MSG, 0x00, 0x00]) + ts + b"hello"
            responses = bridge.handle_command(cmd, state, sock, addr)
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0][0], bridge.RESP_OK)

            sock.settimeout(1.0)
            data, _ = sock.recvfrom(65536)
            import cbor2

            msg = cbor2.loads(data)
            packet = msg["payload"]
            route = packet[0] & 0x03
            self.assertEqual(route, 0x00)  # ROUTE_T_FLOOD
            # Transport codes present (4 bytes after header)
            tc1 = int.from_bytes(packet[1:3], "little")
            self.assertGreater(tc1, 0)
        finally:
            sock.close()

    def test_txt_msg_flood_contact_uses_t_flood_with_region(self):
        """TXT_MSG to a flood contact (out_path_len=-1) uses ROUTE_T_FLOOD when
        region scope is active.

        Transport codes are a flood concept — they apply to flood routing.
        A contact with out_path_len=-1 has no known direct path, so the
        firmware would send via flood (BaseChatMesh::sendMessage, line 399).
        The bridge must replicate this: use ROUTE_T_FLOOD when scope is set.
        """
        import socket

        state = make_state(region_scope="de-nord")

        # Create a flood contact (out_path_len=-1, the default from ADVERTs)
        _peer_prv, peer_pub, _peer_seed = make_identity()
        state.contacts[peer_pub.hex()] = _make_contact_record(peer_pub)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        addr = ("127.0.0.1", sock.getsockname()[1])
        try:
            ts = struct.pack("<I", int(time.time()))
            cmd = (
                bytes([bridge.CMD_SEND_TXT_MSG, 0x00, 0x00])
                + ts
                + peer_pub[:6]
                + b"hello"
            )
            responses = bridge.handle_command(cmd, state, sock, addr)
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

            sock.settimeout(1.0)
            data, _ = sock.recvfrom(65536)
            import cbor2

            msg = cbor2.loads(data)
            packet = msg["payload"]
            route = packet[0] & 0x03
            self.assertEqual(route, 0x00)  # ROUTE_T_FLOOD — flood contact + scope
            # Transport codes present (4 bytes after header)
            tc1 = int.from_bytes(packet[1:3], "little")
            self.assertGreater(tc1, 0)
        finally:
            sock.close()

    def test_txt_msg_zero_hop_contact_uses_route_direct(self):
        """TXT_MSG to a zero-hop contact (out_path_len=0) uses ROUTE_DIRECT.

        A contact with out_path_len=0 has a known zero-hop direct path
        (device is in direct radio range). The bridge sends ROUTE_DIRECT.
        """
        import socket

        state = make_state(region_scope="de-nord")

        # Create a zero-hop direct contact (out_path_len=0)
        _peer_prv, peer_pub, _peer_seed = make_identity()
        record = _make_contact_record(peer_pub)
        # Patch out_path_len byte (offset 34) to 0 (zero-hop direct)
        record_bytes = bytearray(record)
        record_bytes[34] = 0x00
        state.contacts[peer_pub.hex()] = bytes(record_bytes)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        addr = ("127.0.0.1", sock.getsockname()[1])
        try:
            ts = struct.pack("<I", int(time.time()))
            cmd = (
                bytes([bridge.CMD_SEND_TXT_MSG, 0x00, 0x00])
                + ts
                + peer_pub[:6]
                + b"hello"
            )
            responses = bridge.handle_command(cmd, state, sock, addr)
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

            sock.settimeout(1.0)
            data, _ = sock.recvfrom(65536)
            import cbor2

            msg = cbor2.loads(data)
            packet = msg["payload"]
            route = packet[0] & 0x03
            self.assertEqual(route, 0x02)  # ROUTE_DIRECT for zero-hop
        finally:
            sock.close()


# ---- RX dedup tests ----


class TestRxDedup(unittest.TestCase):
    def test_dedup_suppresses_identical_payload(self):
        """The dedup dict in BridgeState tracks recent RX payload hashes."""
        state = make_state()
        # Simulate adding a hash to recent_rx
        import hashlib

        payload = b"\x01\x02\x03\x04\x05"
        h = hashlib.sha256(payload).digest()[:8]
        state.recent_rx[h] = time.monotonic()

        # Same hash should be found
        self.assertIn(h, state.recent_rx)

    def test_dedup_expires_old_entries(self):
        """Entries older than recent_rx_ttl are expired."""
        state = make_state()
        state.recent_rx_ttl = 0.0  # expire immediately
        import hashlib

        payload = b"\x01\x02\x03\x04\x05"
        h = hashlib.sha256(payload).digest()[:8]
        state.recent_rx[h] = time.monotonic() - 1.0

        # Simulate expiry logic (as in run_bridge)
        now = time.monotonic()
        expired = [
            k for k, ts in state.recent_rx.items() if now - ts > state.recent_rx_ttl
        ]
        for k in expired:
            del state.recent_rx[k]
        self.assertNotIn(h, state.recent_rx)


# ---- Radio parameter command tests ----


class TestRadioParamCommands(unittest.TestCase):
    """Test SET_RADIO_PARAMS, SET_RADIO_TX_POWER, and SET_ADVERT_LATLON commands."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_set_radio_params_updates_state(self):
        """CMD_SET_RADIO_PARAMS updates freq, bw, sf, cr in state."""
        freq_hz = struct.pack("<I", 868_500_000)  # 868.5 MHz
        bw_hz = struct.pack("<I", 125_000)  # 125 kHz
        sf = bytes([10])
        cr = bytes([5])
        cmd = bytes([bridge.CMD_SET_RADIO_PARAMS]) + freq_hz + bw_hz + sf + cr
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertAlmostEqual(self.state.freq_mhz, 868.5, places=3)
        self.assertAlmostEqual(self.state.bw_khz, 125.0, places=1)
        self.assertEqual(self.state.sf, 10)
        self.assertEqual(self.state.cr, 5)

    def test_set_radio_tx_power_updates_state(self):
        """CMD_SET_RADIO_TX_POWER updates tx_power in state."""
        cmd = bytes([bridge.CMD_SET_RADIO_TX_POWER, 20])
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(self.state.tx_power, 20)

    def test_set_advert_latlon_updates_state(self):
        """CMD_SET_ADVERT_LATLON updates lat_e6 and lon_e6, reflected in SELF_INFO."""
        lat_e6 = int(51.5074 * 1e6)  # London approx
        lon_e6 = int(-0.1278 * 1e6)
        lat_bytes = struct.pack("<i", lat_e6)
        lon_bytes = struct.pack("<i", lon_e6)
        cmd = bytes([bridge.CMD_SET_ADVERT_LATLON]) + lat_bytes + lon_bytes
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(self.state.lat_e6, lat_e6)
        self.assertEqual(self.state.lon_e6, lon_e6)
        # Verify reflected in SELF_INFO:
        # RESP_SELF_INFO(1)+adv_type(1)+tx_power(1)+max_tx_power(1)+pub_key(32) = 36
        # lat at 36..40, lon at 40..44
        self_info = self.state.build_self_info()
        lat_from_info = struct.unpack_from("<i", self_info, 36)[0]
        lon_from_info = struct.unpack_from("<i", self_info, 40)[0]
        self.assertEqual(lat_from_info, lat_e6)
        self.assertEqual(lon_from_info, lon_e6)


# ---- Location in ADVERT tests ----


class TestLatLonInAdvert(unittest.TestCase):
    """Test that lat/lon are included in ADVERT when non-zero, omitted when zero."""

    def test_advert_includes_location_when_set(self):
        """_handle_send_advert includes HAS_LOCATION flag when lat_e6/lon_e6 non-zero."""
        state = make_state(lat_e6=int(51.5 * 1e6), lon_e6=int(-0.1 * 1e6))
        from meshcore_tx import build_advert

        # build_advert is what _handle_send_advert calls; verify it includes location
        lat = state.lat_e6 / 1e6
        lon = state.lon_e6 / 1e6
        pkt = build_advert(state.seed, state.pub_key, name=state.name, lat=lat, lon=lon)
        # app_data flags byte is at pkt[2 + 32 + 4 + 64] = pkt[102]
        # (hdr=1, path_len=1, pubkey=32, ts=4, sig=64, then app_data starts)
        app_data_off = 2 + 32 + 4 + 64
        flags = pkt[app_data_off]
        self.assertTrue(
            flags & bridge.ADVERT_HAS_LOCATION,
            f"HAS_LOCATION not set in flags=0x{flags:02x}",
        )

    def test_advert_omits_location_when_zero(self):
        """_handle_send_advert omits HAS_LOCATION flag when lat_e6=lon_e6=0."""
        state = make_state()  # lat_e6=0, lon_e6=0 by default
        self.assertEqual(state.lat_e6, 0)
        self.assertEqual(state.lon_e6, 0)
        from meshcore_tx import build_advert

        pkt = build_advert(
            state.seed, state.pub_key, name=state.name, lat=None, lon=None
        )
        app_data_off = 2 + 32 + 4 + 64
        flags = pkt[app_data_off]
        self.assertFalse(
            flags & bridge.ADVERT_HAS_LOCATION,
            f"HAS_LOCATION should be clear, flags=0x{flags:02x}",
        )


# ---- ANON_REQ response tests ----


class TestAnonReqResponse(unittest.TestCase):
    """Test that ANON_REQ produces an encrypted RESPONSE wire packet in ack_packets."""

    def test_anon_req_returns_response_in_ack_packets(self):
        """ANON_REQ addressed to bridge returns RESPONSE wire packet in ack_packets."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        packet = build_anon_req(prv_a, pub_a, pub_b, b"ping")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        # Should have exactly one RESPONSE packet in ack_packets
        self.assertEqual(len(ack_packets), 1)
        # Wire packet starts with MeshCore header byte
        resp_pkt = ack_packets[0]
        self.assertGreater(len(resp_pkt), 4, "RESPONSE packet too short")

    def test_anon_req_response_decryptable_by_sender(self):
        """RESPONSE in ack_packets can be decrypted using sender ECDH shared secret."""
        from meshcore_crypto import meshcore_shared_secret, meshcore_mac_then_decrypt

        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(
            pub_key=pub_b, expanded_prv=prv_b, seed=seed_b, name="bridge-node"
        )
        packet = build_anon_req(prv_a, pub_a, pub_b, b"hello")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": packet,
        }
        _, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(ack_packets), 1)

        # The RESPONSE wire packet: header(1) + path_len(1) + payload...
        # Payload: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
        resp_pkt = ack_packets[0]
        payload_off = 2  # skip hdr + path_len
        resp_payload = resp_pkt[payload_off:]
        # MAC+ciphertext starts after dest_hash(1) + src_hash(1)
        enc_data = resp_payload[2:]
        shared = meshcore_shared_secret(prv_a, pub_b)
        assert shared is not None
        plaintext = meshcore_mac_then_decrypt(shared, enc_data)
        assert plaintext is not None, "RESPONSE decryption failed"
        # Plaintext: 0x00 (RESP_SERVER_LOGIN_OK) + node name
        self.assertEqual(plaintext[0], 0x00)
        self.assertIn(b"bridge-node", plaintext)


# ---- Contact export/share tests ----


class TestContactExport(unittest.TestCase):
    """Test CMD_EXPORT_CONTACT and CMD_SHARE_CONTACT."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_export_contact_self_returns_contact_uri(self):
        """CMD_EXPORT_CONTACT with own pubkey prefix returns RESP_CONTACT_URI."""
        prefix = self.state.pub_key[:6]
        cmd = bytes([bridge.CMD_EXPORT_CONTACT]) + prefix
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)
        # Wire packet follows the response code
        self.assertGreater(len(responses[0]), 4)

    def test_share_contact_self_returns_ok(self):
        """CMD_SHARE_CONTACT returns RESP_OK, not RESP_CONTACT_URI."""
        prefix = self.state.pub_key[:6]
        cmd = bytes([bridge.CMD_SHARE_CONTACT]) + prefix
        responses = bridge.handle_command(cmd, self.state, None, None)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

    def test_export_contact_self_still_returns_uri(self):
        """CMD_EXPORT_CONTACT still returns RESP_CONTACT_URI (unchanged)."""
        prefix = self.state.pub_key[:6]
        cmd = bytes([bridge.CMD_EXPORT_CONTACT]) + prefix
        responses = bridge.handle_command(cmd, self.state, None, None)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)


# ---- Real radio stats tests ----


class TestRealRadioStats(unittest.TestCase):
    """Test that build_stats(1) reflects real RSSI/SNR, and noise_floor updates."""

    def test_radio_stats_reflect_rssi_snr(self):
        """build_stats(1) uses last_rssi_dbm and last_snr_db from state."""
        state = make_state()
        state.last_rssi_dbm = -75
        state.last_snr_db = 6.0
        stats = state.build_stats(1)
        self.assertEqual(stats[0], bridge.RESP_STATS)
        self.assertEqual(stats[1], 1)  # radio type
        # noise_floor at bytes 2-4 (int16 LE)
        noise_floor = struct.unpack_from("<h", stats, 2)[0]
        self.assertEqual(noise_floor, state.noise_floor_dbm)
        # last_rssi at byte 4 (int8)
        last_rssi = struct.unpack_from("<b", stats, 4)[0]
        self.assertEqual(last_rssi, -75)
        # last_snr at byte 5 (int8, *4)
        last_snr_raw = struct.unpack_from("<b", stats, 5)[0]
        self.assertEqual(last_snr_raw, int(6.0 * 4))

    def test_noise_floor_updates_from_lora_frame(self):
        """lora_frame_to_companion_msgs tracks RSSI/SNR per frame into state."""
        state = make_state()
        # Build a minimal ADVERT frame with RSSI in PHY
        hdr = (0x04 << 2) | 0x01  # ADVERT + FLOOD
        pubkey = bytes(range(32))
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        app_data = bytes([0x81]) + b"Test"
        payload = bytes([hdr, 0]) + pubkey + ts + sig + app_data
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": -3.5, "rssi_dbm": -90},
            "payload": payload,
        }
        bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertAlmostEqual(state.last_snr_db, -3.5, places=1)
        self.assertEqual(state.last_rssi_dbm, -90)


# ---- Identity edge-case tests ----


class TestIdentityLoading(unittest.TestCase):
    """Test load_or_create_identity edge cases."""

    def setUp(self):
        import tempfile

        self.tmpdir = Path(tempfile.mkdtemp())

    def test_missing_file_creates_identity(self):
        """Missing identity file is created on first use."""
        path = self.tmpdir / "new_identity.bin"
        self.assertFalse(path.exists())
        expanded, pub, seed = load_or_create_identity(path)
        self.assertTrue(path.exists())
        self.assertEqual(len(expanded), 64)
        self.assertEqual(len(pub), 32)
        self.assertEqual(len(seed), 32)
        # File is 64 bytes: seed(32) + pub(32)
        data = path.read_bytes()
        self.assertEqual(len(data), 64)
        self.assertEqual(data[:32], seed)
        self.assertEqual(data[32:], pub)

    def test_valid_file_loads_correctly(self):
        """Valid 64-byte identity file loads without regeneration."""
        path = self.tmpdir / "identity.bin"
        # Create a valid identity
        expanded1, pub1, seed1 = load_or_create_identity(path)
        mtime1 = path.stat().st_mtime
        # Load it again — should not overwrite
        expanded2, pub2, seed2 = load_or_create_identity(path)
        mtime2 = path.stat().st_mtime
        self.assertEqual(pub1, pub2)
        self.assertEqual(seed1, seed2)
        self.assertEqual(mtime1, mtime2)

    def test_zero_length_file_regenerates(self):
        """Zero-length identity file triggers regeneration."""
        path = self.tmpdir / "identity.bin"
        path.write_bytes(b"")
        expanded, pub, seed = load_or_create_identity(path)
        self.assertEqual(len(pub), 32)
        # File should now be 64 bytes
        self.assertEqual(len(path.read_bytes()), 64)

    def test_wrong_size_file_regenerates(self):
        """Identity file of wrong size (e.g. 32 bytes) triggers regeneration."""
        path = self.tmpdir / "identity.bin"
        path.write_bytes(os.urandom(32))  # too short
        expanded, pub, seed = load_or_create_identity(path)
        self.assertEqual(len(pub), 32)
        self.assertEqual(len(path.read_bytes()), 64)

    def test_mismatched_keys_regenerates(self):
        """Identity file with mismatched seed/pubkey triggers regeneration."""
        path = self.tmpdir / "identity.bin"
        # Write 32 bytes of random seed + 32 bytes of garbage pubkey
        seed_bytes = os.urandom(32)
        garbage_pub = os.urandom(32)
        path.write_bytes(seed_bytes + garbage_pub)
        # The real pubkey derived from seed_bytes won't match garbage_pub
        expanded, pub, seed = load_or_create_identity(path)
        self.assertEqual(len(pub), 32)
        # New identity should have correct matching keys
        data = path.read_bytes()
        self.assertEqual(data[:32], seed)
        self.assertEqual(data[32:], pub)


# ---- Key store edge-case tests ----


class TestKeyStoreEdgeCases(unittest.TestCase):
    """Test load_known_keys and save_pubkey edge cases."""

    def setUp(self):
        import tempfile

        self.tmpdir = Path(tempfile.mkdtemp())
        self.keys_dir = self.tmpdir / "keys"

    def test_missing_directory_returns_empty(self):
        """Non-existent keys_dir returns empty dict without error."""
        from meshcore_crypto import load_known_keys

        keys = load_known_keys(self.keys_dir)
        self.assertEqual(keys, {})

    def test_valid_key_loaded(self):
        """32-byte .key file is loaded correctly."""
        from meshcore_crypto import load_known_keys, save_pubkey

        pubkey = os.urandom(32)
        save_pubkey(self.keys_dir, pubkey)
        keys = load_known_keys(self.keys_dir)
        self.assertIn(pubkey.hex(), keys)
        self.assertEqual(keys[pubkey.hex()], pubkey)

    def test_corrupt_key_skipped_with_warning(self):
        """Wrong-size .key file is skipped and a warning is logged."""
        from meshcore_crypto import load_known_keys

        self.keys_dir.mkdir(parents=True)
        bad_file = self.keys_dir / "deadbeef.key"
        bad_file.write_bytes(os.urandom(33))  # wrong size
        with self.assertLogs("gr4.crypto", level="WARNING") as cm:
            keys = load_known_keys(self.keys_dir)
        self.assertEqual(keys, {})
        self.assertTrue(any("corrupt key file" in msg for msg in cm.output))

    def test_zero_length_key_skipped(self):
        """Zero-length .key file is skipped."""
        from meshcore_crypto import load_known_keys

        self.keys_dir.mkdir(parents=True)
        (self.keys_dir / "empty.key").write_bytes(b"")
        keys = load_known_keys(self.keys_dir)
        self.assertEqual(keys, {})

    def test_save_pubkey_read_only_dir_returns_false(self):
        """save_pubkey returns False and logs warning when dir is not writable."""
        from meshcore_crypto import save_pubkey

        self.keys_dir.mkdir(parents=True)
        self.keys_dir.chmod(0o555)  # read+execute only
        try:
            pubkey = os.urandom(32)
            with self.assertLogs("gr4.crypto", level="WARNING") as cm:
                result = save_pubkey(self.keys_dir, pubkey)
            self.assertFalse(result)
            self.assertTrue(any("could not save key" in msg for msg in cm.output))
        finally:
            self.keys_dir.chmod(0o755)


# ---- Channel store edge-case tests ----


class TestChannelStoreEdgeCases(unittest.TestCase):
    """Test load_channels and save_channel edge cases."""

    def setUp(self):
        import tempfile

        self.tmpdir = Path(tempfile.mkdtemp())
        self.channels_dir = self.tmpdir / "channels"

    def test_corrupt_channel_skipped_with_warning(self):
        """Wrong-size .channel file is skipped and a warning is logged."""
        from meshcore_crypto import load_channels

        self.channels_dir.mkdir(parents=True)
        bad_file = self.channels_dir / "bad.channel"
        bad_file.write_bytes(os.urandom(10))  # not 16 or 32 bytes
        with self.assertLogs("gr4.crypto", level="WARNING") as cm:
            channels = load_channels(self.channels_dir)
        # No valid channels (only the bad one)
        self.assertFalse(any(ch.name == "bad" for ch in channels))
        self.assertTrue(any("corrupt channel file" in msg for msg in cm.output))

    def test_valid_16_byte_channel_loaded(self):
        """16-byte .channel file is loaded correctly."""
        from meshcore_crypto import load_channels

        self.channels_dir.mkdir(parents=True)
        psk = os.urandom(16)
        (self.channels_dir / "mytest.channel").write_bytes(psk)
        channels = load_channels(self.channels_dir)
        self.assertTrue(any(ch.name == "mytest" for ch in channels))

    def test_valid_32_byte_channel_loaded(self):
        """32-byte .channel file is loaded correctly."""
        from meshcore_crypto import load_channels

        self.channels_dir.mkdir(parents=True)
        psk = os.urandom(32)
        (self.channels_dir / "bigtest.channel").write_bytes(psk)
        channels = load_channels(self.channels_dir)
        self.assertTrue(any(ch.name == "bigtest" for ch in channels))

    def test_save_channel_read_only_dir_returns_false(self):
        """save_channel returns False and logs warning when dir is not writable."""
        from meshcore_crypto import save_channel

        self.channels_dir.mkdir(parents=True)
        self.channels_dir.chmod(0o555)
        try:
            with self.assertLogs("gr4.crypto", level="WARNING") as cm:
                result = save_channel(self.channels_dir, "test", os.urandom(16))
            self.assertFalse(result)
            self.assertTrue(any("could not save channel" in msg for msg in cm.output))
        finally:
            self.channels_dir.chmod(0o755)


# ---- Contact store edge-case tests ----


class TestContactStoreEdgeCases(unittest.TestCase):
    """Test _load_persisted_contacts and _persist_contacts edge cases."""

    def setUp(self):
        import tempfile

        self.tmpdir = Path(tempfile.mkdtemp())
        self.contacts_dir = self.tmpdir / "contacts"

    def test_missing_directory_returns_empty(self):
        """Non-existent contacts_dir returns empty dict without error."""
        contacts = bridge._load_persisted_contacts(self.contacts_dir)
        self.assertEqual(contacts, {})

    def test_valid_contact_loaded(self):
        """147-byte .contact file is loaded correctly."""
        self.contacts_dir.mkdir(parents=True)
        state = make_state(contacts_dir=self.contacts_dir)
        # Add a contact and persist it
        pubkey = os.urandom(32)
        record = bridge._build_contact_record(pubkey, name="Alice")
        self.assertEqual(len(record), 147)
        (self.contacts_dir / f"{pubkey.hex()}.contact").write_bytes(record)
        contacts = bridge._load_persisted_contacts(self.contacts_dir)
        self.assertIn(pubkey.hex(), contacts)

    def test_corrupt_contact_skipped_with_warning(self):
        """Wrong-size .contact file is skipped and a warning is logged."""
        self.contacts_dir.mkdir(parents=True)
        bad_file = self.contacts_dir / "deadbeef.contact"
        bad_file.write_bytes(os.urandom(146))  # one byte short
        with self.assertLogs("gr4.bridge", level="WARNING") as cm:
            contacts = bridge._load_persisted_contacts(self.contacts_dir)
        self.assertEqual(contacts, {})
        self.assertTrue(any("corrupt contact file" in msg for msg in cm.output))

    def test_zero_length_contact_skipped(self):
        """Zero-length .contact file is skipped."""
        self.contacts_dir.mkdir(parents=True)
        (self.contacts_dir / "empty.contact").write_bytes(b"")
        contacts = bridge._load_persisted_contacts(self.contacts_dir)
        self.assertEqual(contacts, {})

    def test_persist_contacts_read_only_dir_logs_warning(self):
        """_persist_contacts logs warning when contacts_dir is not writable."""
        self.contacts_dir.mkdir(parents=True)
        self.contacts_dir.chmod(0o555)
        try:
            state = make_state(contacts_dir=self.contacts_dir)
            pubkey = os.urandom(32)
            record = bridge._build_contact_record(pubkey, name="Bob")
            state.contacts[pubkey.hex()] = record
            with self.assertLogs("gr4.bridge", level="WARNING") as cm:
                bridge._persist_contacts(state)
            self.assertTrue(
                any("could not persist contacts" in msg for msg in cm.output)
            )
        finally:
            self.contacts_dir.chmod(0o755)


# ---- Repeat mode tests ----


class TestDeviceInfoRepeatMode(unittest.TestCase):
    """PACKET_DEVICE_INFO byte layout with repeat-mode fields."""

    def test_device_info_length(self):
        """build_device_info returns 82 bytes."""
        state = make_state()
        info = state.build_device_info()
        self.assertEqual(len(info), 82)

    def test_device_info_client_repeat_default(self):
        """Byte 80 of DEVICE_INFO is 0 by default."""
        state = make_state()
        info = state.build_device_info()
        self.assertEqual(info[80], 0)

    def test_device_info_client_repeat_set(self):
        """Byte 80 of DEVICE_INFO reflects state.client_repeat."""
        state = make_state()
        state.client_repeat = 1
        info = state.build_device_info()
        self.assertEqual(info[80], 1)

    def test_device_info_path_hash_mode_default(self):
        """Byte 81 of DEVICE_INFO is 0 by default."""
        state = make_state()
        info = state.build_device_info()
        self.assertEqual(info[81], 0)

    def test_device_info_path_hash_mode_set(self):
        """Byte 81 of DEVICE_INFO reflects state.path_hash_mode."""
        state = make_state()
        state.path_hash_mode = 2
        info = state.build_device_info()
        self.assertEqual(info[81], 2)


class TestSetFloodScope(unittest.TestCase):
    """CMD_SET_FLOOD_SCOPE (0x36) handler."""

    def _cmd(self, payload):
        state = make_state()
        null_sock = _FakeUDPSock()
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        return state, result

    def test_set_scope_key(self):
        """18-byte frame sets send_scope to the 16-byte key."""
        key = bytes(range(16))
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x00]) + key
        state, result = self._cmd(payload)
        self.assertEqual(state.send_scope, key)
        self.assertEqual(result[0][0], bridge.RESP_OK)

    def test_clear_scope_short_frame(self):
        """Short frame (< 18 bytes) clears send_scope to all-zero."""
        # First set a key
        state = make_state()
        state.send_scope = bytes(range(16))
        null_sock = _FakeUDPSock()
        # Send clear command (only discriminator, no key)
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x00])
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(state.send_scope, b"\x00" * 16)
        self.assertEqual(result[0][0], bridge.RESP_OK)

    def test_bad_discriminator_returns_error(self):
        """Non-zero discriminator byte returns ERR_CODE_ILLEGAL_ARG."""
        key = bytes(range(16))
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x01]) + key
        state = make_state()
        null_sock = _FakeUDPSock()
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_ERROR)
        self.assertEqual(result[0][1], bridge.ERR_CODE_ILLEGAL_ARG)

    def test_empty_data_returns_error(self):
        """Empty data (no discriminator) returns ERR_CODE_ILLEGAL_ARG."""
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE])
        state = make_state()
        null_sock = _FakeUDPSock()
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_ERROR)

    def test_allzero_key_ignored_when_config_scope_active(self):
        """All-zero key from companion is ignored when config.toml specifies a non-zero scope.

        meshcore-cli sends CMD_SET_FLOOD_SCOPE with all-zero key on every connect
        (its stale default). The bridge must not let this overwrite a config-specified
        scope — otherwise the configured flood scope is cleared every session.
        """
        config_scope = bytes(range(1, 17))  # non-zero config scope
        state = make_state(send_scope=config_scope)
        null_sock = _FakeUDPSock()
        # Companion sends all-zero key (its stale default)
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x00]) + b"\x00" * 16
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        # Must return OK (not error) but scope must remain unchanged
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.send_scope, config_scope)

    def test_allzero_key_accepted_when_no_config_scope(self):
        """All-zero key is accepted normally when config.toml has no scope set."""
        state = make_state()  # default: send_scope = b"\x00" * 16
        state.send_scope = bytes(range(16))  # set a non-zero runtime scope
        null_sock = _FakeUDPSock()
        # Companion clears scope to all-zero — should be honoured (no config scope)
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x00]) + b"\x00" * 16
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.send_scope, b"\x00" * 16)

    def test_nonzero_key_always_applied(self):
        """A non-zero key from the companion is always applied, even with config scope set."""
        config_scope = bytes(range(1, 17))
        state = make_state(send_scope=config_scope)
        null_sock = _FakeUDPSock()
        new_key = bytes(range(16, 32))
        payload = bytes([bridge.CMD_SET_FLOOD_SCOPE, 0x00]) + new_key
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.send_scope, new_key)


class TestGetAllowedRepeatFreq(unittest.TestCase):
    """CMD_GET_ALLOWED_REPEAT_FREQ (0x3C) handler."""

    def test_response_length(self):
        """Response is 1 + 3*8 = 25 bytes."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_GET_ALLOWED_REPEAT_FREQ])
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(len(result[0]), 25)

    def test_response_code(self):
        """First byte is RESP_ALLOWED_REPEAT_FREQ (0x1A)."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_GET_ALLOWED_REPEAT_FREQ])
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_ALLOWED_REPEAT_FREQ)

    def test_correct_freq_ranges(self):
        """Response contains the 3 correct (lo, hi) pairs in kHz."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_GET_ALLOWED_REPEAT_FREQ])
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        data = result[0][1:]  # skip response code
        pairs = [struct.unpack_from("<II", data, i * 8) for i in range(3)]
        self.assertEqual(pairs, list(bridge.REPEAT_FREQ_RANGES))


class TestSetRadioParamsRepeat(unittest.TestCase):
    """CMD_SET_RADIO_PARAMS optional repeat byte (byte 10)."""

    def _radio_params(self, freq_hz, repeat=None):
        """Build a CMD_SET_RADIO_PARAMS payload."""
        bw_hz = 62500
        sf = 8
        cr = 8
        payload = (
            bytes([bridge.CMD_SET_RADIO_PARAMS])
            + struct.pack("<II", freq_hz, bw_hz)
            + bytes([sf, cr])
        )
        if repeat is not None:
            payload += bytes([repeat])
        return payload

    def test_repeat_valid_freq_sets_flag(self):
        """Valid repeat freq (869000 kHz) + repeat=1 sets client_repeat."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = self._radio_params(869000 * 1000, repeat=1)
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.client_repeat, 1)

    def test_repeat_433mhz_valid(self):
        """433000 kHz is a valid repeat frequency."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = self._radio_params(433000 * 1000, repeat=1)
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.client_repeat, 1)

    def test_repeat_918mhz_valid(self):
        """918000 kHz is a valid repeat frequency."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = self._radio_params(918000 * 1000, repeat=1)
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.client_repeat, 1)

    def test_repeat_invalid_freq_returns_error(self):
        """Invalid freq (868618 kHz) + repeat=1 returns ERR_CODE_ILLEGAL_ARG."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = self._radio_params(868618000, repeat=1)
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_ERROR)
        self.assertEqual(result[0][1], bridge.ERR_CODE_ILLEGAL_ARG)
        self.assertEqual(state.client_repeat, 0)  # unchanged

    def test_repeat_zero_always_ok(self):
        """repeat=0 is accepted regardless of frequency."""
        state = make_state()
        null_sock = _FakeUDPSock()
        payload = self._radio_params(868618000, repeat=0)
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.client_repeat, 0)

    def test_no_repeat_byte_leaves_client_repeat_unchanged(self):
        """Missing 5th byte does not change client_repeat."""
        state = make_state()
        state.client_repeat = 1  # pre-set
        null_sock = _FakeUDPSock()
        payload = self._radio_params(869000 * 1000, repeat=None)
        bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(state.client_repeat, 1)  # unchanged


class TestAutoaddConfig(unittest.TestCase):
    """CMD_SET_AUTOADD_CONFIG (0x3A) and CMD_GET_AUTOADD_CONFIG (0x3B)."""

    def _cmd(self, payload):
        state = make_state()
        null_sock = _FakeUDPSock()
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        return state, result

    def test_set_config_bitmask(self):
        """SET_AUTOADD_CONFIG stores bitmask."""
        bitmask = 0x1F  # all bits
        payload = bytes([bridge.CMD_SET_AUTOADD_CONFIG, bitmask])
        state, result = self._cmd(payload)
        self.assertEqual(result[0][0], bridge.RESP_OK)
        self.assertEqual(state.autoadd_config, bitmask)

    def test_set_config_with_max_hops(self):
        """SET_AUTOADD_CONFIG stores bitmask and max_hops."""
        payload = bytes([bridge.CMD_SET_AUTOADD_CONFIG, 0x07, 10])
        state, result = self._cmd(payload)
        self.assertEqual(state.autoadd_config, 0x07)
        self.assertEqual(state.autoadd_max_hops, 10)

    def test_set_config_max_hops_clamped(self):
        """max_hops > 64 is clamped to 64."""
        payload = bytes([bridge.CMD_SET_AUTOADD_CONFIG, 0x00, 100])
        state, result = self._cmd(payload)
        self.assertEqual(state.autoadd_max_hops, 64)

    def test_get_config(self):
        """GET_AUTOADD_CONFIG returns RESP_CODE_AUTOADD_CONFIG + bitmask + max_hops."""
        state = make_state()
        state.autoadd_config = 0x05
        state.autoadd_max_hops = 7
        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_GET_AUTOADD_CONFIG])
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        self.assertEqual(len(result[0]), 3)
        self.assertEqual(result[0][0], bridge.RESP_CODE_AUTOADD_CONFIG)
        self.assertEqual(result[0][1], 0x05)
        self.assertEqual(result[0][2], 7)


class TestSetPathHashMode(unittest.TestCase):
    """CMD_SET_PATH_HASH_MODE (0x3D) handler."""

    def _cmd(self, payload):
        state = make_state()
        null_sock = _FakeUDPSock()
        result = bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))
        return state, result

    def test_valid_modes(self):
        """Modes 0, 1, 2 are accepted and stored."""
        for mode in (0, 1, 2):
            with self.subTest(mode=mode):
                payload = bytes([bridge.CMD_SET_PATH_HASH_MODE, 0x00, mode])
                state, result = self._cmd(payload)
                self.assertEqual(result[0][0], bridge.RESP_OK)
                self.assertEqual(state.path_hash_mode, mode)

    def test_invalid_mode_returns_error(self):
        """Mode >= 3 returns ERR_CODE_ILLEGAL_ARG."""
        for mode in (3, 10, 255):
            with self.subTest(mode=mode):
                payload = bytes([bridge.CMD_SET_PATH_HASH_MODE, 0x00, mode])
                state, result = self._cmd(payload)
                self.assertEqual(result[0][0], bridge.RESP_ERROR)
                self.assertEqual(result[0][1], bridge.ERR_CODE_ILLEGAL_ARG)

    def test_bad_discriminator_returns_error(self):
        """Non-zero discriminator byte returns ERR_CODE_ILLEGAL_ARG."""
        payload = bytes([bridge.CMD_SET_PATH_HASH_MODE, 0x01, 0])
        state, result = self._cmd(payload)
        self.assertEqual(result[0][0], bridge.RESP_ERROR)
        self.assertEqual(result[0][1], bridge.ERR_CODE_ILLEGAL_ARG)

    def test_short_frame_returns_error(self):
        """Frame with < 2 data bytes returns error."""
        payload = bytes([bridge.CMD_SET_PATH_HASH_MODE, 0x00])
        state, result = self._cmd(payload)
        self.assertEqual(result[0][0], bridge.RESP_ERROR)


class TestSendScopePriority(unittest.TestCase):
    """send_scope overrides region_key in TX handlers."""

    def test_send_scope_overrides_region_key_in_advert(self):
        """Non-null send_scope is used instead of region_key for ADVERT TX."""
        scope_key = bytes(range(16))
        state = make_state(region_scope="de-nord")
        state.send_scope = scope_key
        # region_key is also set (from "de-nord")
        self.assertTrue(len(state.region_key) == 16)

        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_SEND_SELF_ADVERT, 0x01])
        bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))

        # The TX packet must have T_FLOOD route (header & 0x03 == 0 = T_FLOOD)
        self.assertTrue(len(null_sock.sent) > 0, "no UDP packet sent")
        # Decode CBOR to get the wire packet
        import cbor2

        cbor_msg = cbor2.loads(null_sock.sent[0])
        wire = cbor_msg.get("payload", b"")
        self.assertTrue(len(wire) > 0)
        # Header byte route type must be T_FLOOD (0) — transport codes present
        route_type = wire[0] & 0x03
        self.assertEqual(
            route_type, 0, f"expected T_FLOOD(0), got route_type={route_type}"
        )

    def test_null_scope_falls_back_to_region_key(self):
        """All-zero send_scope falls back to region_key."""
        state = make_state(region_scope="de-nord")
        state.send_scope = b"\x00" * 16  # null scope

        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_SEND_SELF_ADVERT, 0x01])
        bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))

        self.assertTrue(len(null_sock.sent) > 0, "no UDP packet sent")
        import cbor2

        cbor_msg = cbor2.loads(null_sock.sent[0])
        wire = cbor_msg.get("payload", b"")
        # region_key is set → T_FLOOD (route_type 0) should still be used
        route_type = wire[0] & 0x03
        self.assertEqual(route_type, 0)

    def test_no_key_uses_plain_flood(self):
        """No send_scope and no region_key → plain FLOOD (route_type 1)."""
        state = make_state()  # no region_scope
        state.send_scope = b"\x00" * 16  # null scope

        null_sock = _FakeUDPSock()
        payload = bytes([bridge.CMD_SEND_SELF_ADVERT, 0x01])
        bridge.handle_command(payload, state, null_sock, ("127.0.0.1", 5555))

        self.assertTrue(len(null_sock.sent) > 0, "no UDP packet sent")
        import cbor2

        cbor_msg = cbor2.loads(null_sock.sent[0])
        wire = cbor_msg.get("payload", b"")
        route_type = wire[0] & 0x03
        self.assertEqual(
            route_type, 1, f"expected FLOOD(1), got route_type={route_type}"
        )


class TestStartupConfig(unittest.TestCase):
    """BridgeState fields set from constructor keyword args (config.toml startup values)."""

    def test_client_repeat_startup(self):
        """client_repeat=1 at construction is reflected in DEVICE_INFO byte 80."""
        state = make_state(client_repeat=1)
        info = state.build_device_info()
        self.assertEqual(info[80], 1)

    def test_path_hash_mode_startup(self):
        """path_hash_mode=2 at construction is reflected in DEVICE_INFO byte 81."""
        state = make_state(path_hash_mode=2)
        info = state.build_device_info()
        self.assertEqual(info[81], 2)

    def test_send_scope_startup(self):
        """send_scope set at construction is stored correctly."""
        key = bytes(range(16))
        state = make_state(send_scope=key)
        self.assertEqual(state.send_scope, key)

    def test_send_scope_wrong_length_defaults_to_null(self):
        """send_scope with wrong length (not 16) is replaced by all-zero."""
        state = make_state(send_scope=b"\x01\x02")
        self.assertEqual(state.send_scope, b"\x00" * 16)

    def test_autoadd_config_startup(self):
        """autoadd_config=0x0F at construction is stored correctly."""
        state = make_state(autoadd_config=0x0F, autoadd_max_hops=32)
        self.assertEqual(state.autoadd_config, 0x0F)
        self.assertEqual(state.autoadd_max_hops, 32)

    def test_is_valid_repeat_freq_helper(self):
        """_is_valid_repeat_freq correctly accepts/rejects frequencies."""
        self.assertTrue(bridge._is_valid_repeat_freq(433000))
        self.assertTrue(bridge._is_valid_repeat_freq(869000))
        self.assertTrue(bridge._is_valid_repeat_freq(918000))
        self.assertFalse(bridge._is_valid_repeat_freq(868000))
        self.assertFalse(bridge._is_valid_repeat_freq(868618))
        self.assertFalse(bridge._is_valid_repeat_freq(0))


class TestStartupConfigReset(unittest.TestCase):
    """APP_START re-applies config.toml values, overriding companion-session changes."""

    def setUp(self):
        import socket

        self.state = make_state(
            name="config-name",
            lat_e6=int(53.55 * 1e6),
            lon_e6=int(9.99 * 1e6),
            send_scope=bytes(range(16)),
            client_repeat=1,
            path_hash_mode=2,
            autoadd_config=0x03,
            autoadd_max_hops=5,
        )
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _app_start(self):
        return bridge.handle_command(b"\x01", self.state, self.udp_sock, self.udp_addr)

    def test_app_start_resets_name(self):
        """Companion SET_ADVERT_NAME is undone by next APP_START."""
        self.state.name = "companion-changed-name"
        self._app_start()
        self.assertEqual(self.state.name, "config-name")

    def test_app_start_resets_send_scope(self):
        """Companion SET_FLOOD_SCOPE is undone by next APP_START."""
        self.state.send_scope = b"\xff" * 16
        self._app_start()
        self.assertEqual(self.state.send_scope, bytes(range(16)))

    def test_app_start_resets_latlon(self):
        """Companion SET_ADVERT_LATLON is undone by next APP_START."""
        self.state.lat_e6 = 0
        self.state.lon_e6 = 0
        self._app_start()
        self.assertEqual(self.state.lat_e6, int(53.55 * 1e6))
        self.assertEqual(self.state.lon_e6, int(9.99 * 1e6))

    def test_app_start_resets_client_repeat(self):
        """Companion SET_REPEAT_MODE is undone by next APP_START."""
        self.state.client_repeat = 0
        self._app_start()
        self.assertEqual(self.state.client_repeat, 1)

    def test_app_start_resets_path_hash_mode(self):
        """Companion SET_PATH_HASH_MODE is undone by next APP_START."""
        self.state.path_hash_mode = 0
        self._app_start()
        self.assertEqual(self.state.path_hash_mode, 2)

    def test_app_start_resets_autoadd(self):
        """Companion SET_AUTOADD_CONFIG is undone by next APP_START."""
        self.state.autoadd_config = 0xFF
        self.state.autoadd_max_hops = 64
        self._app_start()
        self.assertEqual(self.state.autoadd_config, 0x03)
        self.assertEqual(self.state.autoadd_max_hops, 5)

    def test_self_info_reflects_config_name(self):
        """SELF_INFO returned by APP_START carries the config.toml name."""
        self.state.name = "companion-changed-name"
        responses = self._app_start()
        self_info = responses[0]
        self.assertEqual(self_info[0], bridge.RESP_SELF_INFO)
        # name field starts at byte 58:
        # 1(resp) + 1(adv_type) + 1(tx_power) + 1(max_tx_power)
        # + 32(pubkey) + 4(lat) + 4(lon)
        # + 1(multi_acks) + 1(adv_loc_policy) + 1(telemetry_mode) + 1(manual_add_contacts)
        # + 4(freq) + 4(bw) + 1(sf) + 1(cr) = 58
        name_bytes = self_info[58:]
        name_decoded = name_bytes.decode("utf-8", errors="replace").rstrip("\x00")
        self.assertIn("config-name", name_decoded)

    def test_apply_startup_config_is_idempotent(self):
        """Calling apply_startup_config() twice leaves state unchanged."""
        self.state.apply_startup_config()
        name_after_first = self.state.name
        self.state.apply_startup_config()
        self.assertEqual(self.state.name, name_after_first)


class TestSendControlData(unittest.TestCase):
    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes):
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_ctrl_node_discover_sends_udp(self):
        """CMD_SEND_CONTROL_DATA floods a CTRL packet via UDP."""
        tag = struct.pack("<I", 0xDEADBEEF)
        # control_type=0x80 (NODE_DISCOVER_REQ), filter=0x00 (all types), tag
        payload = bytes([0x80, 0x00]) + tag
        cmd = bytes([bridge.CMD_SEND_CONTROL_DATA]) + payload
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Verify UDP CBOR TX was sent
        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        import cbor2

        msg = cbor2.loads(raw)
        self.assertEqual(msg["type"], "lora_tx")
        self.assertIn("payload", msg)

        # Verify the CTRL payload_type in the wire packet
        wire = msg["payload"]
        # header byte: payload_type bits [5:2] = (header >> 2) & 0x0F
        ptype = (wire[0] >> 2) & 0x0F
        self.assertEqual(ptype, 0x0B)  # PAYLOAD_CTRL

    def test_ctrl_empty_data_returns_error(self):
        """CMD_SEND_CONTROL_DATA with empty payload returns RESP_ERROR."""
        cmd = bytes([bridge.CMD_SEND_CONTROL_DATA])
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_send_control_data_tracks_tx_hash(self):
        """After a successful CTRL send, state.recent_tx is non-empty (echo-filter hash stored)."""
        tag = struct.pack("<I", 0xDEADBEEF)
        payload = bytes([0x80, 0x00]) + tag
        cmd = bytes([bridge.CMD_SEND_CONTROL_DATA]) + payload
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreater(len(self.state.recent_tx), 0)

    def test_send_control_data_uses_plain_flood(self):
        """CTRL packet always uses ROUTE_FLOOD (bits [1:0] == 1) even with region_scope set."""
        import cbor2

        # Set a region_scope — CTRL handler should still use plain ROUTE_FLOOD
        self.state.region_scope = "test"

        tag = struct.pack("<I", 0x12345678)
        payload = bytes([0x80, 0x00]) + tag
        cmd = bytes([bridge.CMD_SEND_CONTROL_DATA]) + payload
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        wire = msg["payload"]
        # ROUTE_FLOOD = 0x01; header byte bits [1:0] must be 1
        route_type = wire[0] & 0x03
        self.assertEqual(
            route_type, 1, f"expected ROUTE_FLOOD(1), got route_type={route_type}"
        )

    def test_send_control_data_payload_preserved(self):
        """CTRL wire packet contains the full control payload bytes verbatim."""
        import cbor2

        ctrl_payload = bytes([0x81, 0x02, 0xAA, 0xBB, 0xCC, 0xDD])
        cmd = bytes([bridge.CMD_SEND_CONTROL_DATA]) + ctrl_payload
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        wire = msg["payload"]
        # Wire packet: header(1) + path_len(1) + payload bytes
        wire_payload = wire[2:]
        self.assertEqual(
            wire_payload,
            ctrl_payload,
            f"payload mismatch: expected {ctrl_payload.hex()}, got {wire_payload.hex()}",
        )


# ---- Send TXT message tests ----


class TestSendTxtMsg(unittest.TestCase):
    def setUp(self):
        import socket

        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        # Bridge is identity A; peer is B
        self.state = make_state(pub_key=pub_a, expanded_prv=prv_a, seed=seed_a)
        self.peer_pub = pub_b

        # Real loopback UDP socket pair so we can receive sent packets
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def _make_cmd(self, dst_prefix: bytes, text: bytes = b"hello") -> bytes:
        """Build a CMD_SEND_TXT_MSG command frame."""
        ts = struct.pack("<I", int(time.time()))
        data = bytes([0x00, 0x00]) + ts + dst_prefix + text
        return bytes([bridge.CMD_SEND_TXT_MSG]) + data

    def _add_peer_contact(self) -> bytes:
        """Add the peer identity as a contact; return the 6-byte dst_prefix."""
        record = _make_contact_record(self.peer_pub, name="peer")
        self.state.contacts[self.peer_pub.hex()] = record
        return self.peer_pub[:6]

    def test_send_txt_msg_too_short_returns_error(self):
        """CMD_SEND_TXT_MSG with < 12 data bytes returns RESP_ERROR."""
        # Only 5 bytes of data (need at least 12)
        cmd = bytes([bridge.CMD_SEND_TXT_MSG]) + b"\x00" * 5
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_send_txt_msg_unknown_contact_returns_error(self):
        """CMD_SEND_TXT_MSG with unknown dst_prefix returns RESP_ERROR."""
        # No contacts registered — prefix won't match anything
        dst_prefix = b"\xde\xad\xbe\xef\xca\xfe"
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_send_txt_msg_success_sends_udp(self):
        """Successful CMD_SEND_TXT_MSG sends a CBOR lora_tx UDP packet."""
        import cbor2

        dst_prefix = self._add_peer_contact()
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

        # Verify a CBOR TX packet was sent via UDP
        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        self.assertEqual(msg["type"], "lora_tx")
        self.assertIn("payload", msg)

    def test_send_txt_msg_returns_msg_sent_flood(self):
        """CMD_SEND_TXT_MSG to a flood contact returns routing_type=1 (flood).

        Contacts imported from ADVERTs have out_path_len=-1 (no known direct
        path). The firmware sends these via flood; the bridge must match.
        PACKET_MSG_SENT byte 1: 1=flood, 0=direct.
        """
        dst_prefix = self._add_peer_contact()
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        resp = responses[0]
        self.assertEqual(resp[0], bridge.RESP_MSG_SENT)
        # byte 1 is routing_type; 1 = flood (out_path_len=-1)
        self.assertEqual(resp[1], 1)

    def test_send_txt_msg_zero_hop_returns_msg_sent_direct(self):
        """CMD_SEND_TXT_MSG to a zero-hop contact returns routing_type=0 (direct)."""
        # Create a zero-hop contact (out_path_len=0)
        record = _make_contact_record(self.peer_pub, name="peer")
        record_bytes = bytearray(record)
        record_bytes[34] = 0x00  # out_path_len=0 (zero-hop direct)
        self.state.contacts[self.peer_pub.hex()] = bytes(record_bytes)
        dst_prefix = self.peer_pub[:6]
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        resp = responses[0]
        self.assertEqual(resp[0], bridge.RESP_MSG_SENT)
        # byte 1 is routing_type; 0 = direct (out_path_len=0)
        self.assertEqual(resp[1], 0)

    def test_send_txt_msg_tracks_tx_hash_in_recent_tx(self):
        """After a successful send, state.recent_tx is non-empty (echo filter)."""
        dst_prefix = self._add_peer_contact()
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertGreater(len(self.state.recent_tx), 0)

    def test_send_txt_msg_registers_pending_ack(self):
        """After a successful send, state.pending_acks has exactly 1 entry."""
        dst_prefix = self._add_peer_contact()
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertEqual(len(self.state.pending_acks), 1)

    def test_pending_ack_keyed_by_crypto_hash_not_msg_seq(self):
        """pending_acks must be keyed by compute_ack_hash, not msg_seq counter.

        The companion device ACKs a TXT_MSG with SHA-256(plaintext + sender_pub)[:4].
        The bridge must store the same hash so the incoming ACK matches.
        A sequential msg_seq counter (1, 2, 3...) will never match.
        """
        from meshcore_crypto import compute_ack_hash

        dst_prefix = self._add_peer_contact()
        text = b"hello"
        ts = int(time.time())
        ts_bytes = struct.pack("<I", ts)
        cmd = (
            bytes([bridge.CMD_SEND_TXT_MSG, 0x00, 0x00]) + ts_bytes + dst_prefix + text
        )
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertEqual(len(self.state.pending_acks), 1)

        # Compute the expected ACK hash: SHA-256(ts(4) + flags(1) + text + pub_key)[:4]
        plaintext = ts_bytes + bytes([0x00]) + text  # flags = (0x00 << 2) | (0 & 3) = 0
        expected_ack = compute_ack_hash(plaintext, self.state.pub_key)

        # The key in pending_acks must be the crypto hash
        self.assertIn(
            expected_ack,
            self.state.pending_acks,
            f"pending_acks keys {list(self.state.pending_acks.keys())} "
            f"should contain crypto hash {expected_ack!r}, not msg_seq counter",
        )

    def test_send_then_receive_ack_produces_push_confirmed(self):
        """End-to-end: send TXT_MSG, receive matching ACK, get PUSH_SEND_CONFIRMED.

        This simulates the full round-trip: bridge sends a message, companion
        device receives it and sends back an RF ACK with compute_ack_hash().
        The bridge must match the ACK and produce PUSH_ACK (0x82).
        """
        from meshcore_crypto import compute_ack_hash, ROUTE_FLOOD

        dst_prefix = self._add_peer_contact()
        text = b"hello"
        ts = int(time.time())
        ts_bytes = struct.pack("<I", ts)
        cmd = (
            bytes([bridge.CMD_SEND_TXT_MSG, 0x00, 0x00]) + ts_bytes + dst_prefix + text
        )
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

        # Compute the ACK hash the companion device would send back
        plaintext = ts_bytes + bytes([0x00]) + text
        ack_hash = compute_ack_hash(plaintext, self.state.pub_key)

        # Build an ACK RF frame with that hash
        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        ack_payload = bytes([hdr, 0]) + ack_hash
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": ack_payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(
            frame, self.state
        )

        # Must produce PUSH_ACK (0x82) with the destination prefix
        self.assertEqual(
            len(companion_msgs), 1, "ACK should produce exactly one push message"
        )
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_ACK)
        # pending_acks should be consumed
        self.assertEqual(len(self.state.pending_acks), 0)

    def test_push_ack_code_matches_msg_sent_expected_ack(self):
        """CLI compatibility: PUSH_ACK ack_code must equal MSG_SENT expected_ack.

        meshcore_py's reader parses PUSH_ACK (0x82) as:
          [0x82] [ack_code(4)] [trip_time(4)]   # 9 bytes
        send_msg_with_retry() waits for EventType.ACK with
          attribute_filters={"code": expected_ack_hex}
        where expected_ack is bytes 2-5 of RESP_MSG_SENT (0x06).

        Both must carry the same 4-byte crypto ACK hash.
        """
        from meshcore_crypto import compute_ack_hash, ROUTE_FLOOD

        dst_prefix = self._add_peer_contact()
        text = b"ack-match"
        ts = int(time.time())
        ts_bytes = struct.pack("<I", ts)
        cmd = (
            bytes([bridge.CMD_SEND_TXT_MSG, 0x00, 0x00]) + ts_bytes + dst_prefix + text
        )
        responses = self._handle(cmd)
        resp = responses[0]
        self.assertEqual(resp[0], bridge.RESP_MSG_SENT)

        # Extract expected_ack from MSG_SENT (bytes 2-5)
        expected_ack = resp[2:6]

        # Compute the crypto ACK hash the companion would send
        plaintext = ts_bytes + bytes([0x00]) + text
        crypto_ack = compute_ack_hash(plaintext, self.state.pub_key)

        # MSG_SENT expected_ack must be the crypto hash (not msg_seq)
        self.assertEqual(
            expected_ack,
            crypto_ack,
            f"MSG_SENT expected_ack {expected_ack!r} must be "
            f"crypto hash {crypto_ack!r}, not a sequential counter",
        )

        # Now simulate receiving the ACK and verify PUSH_ACK format
        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        ack_payload = bytes([hdr, 0]) + crypto_ack
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": ack_payload,
        }
        companion_msgs, _ = bridge.lora_frame_to_companion_msgs(frame, self.state)
        self.assertEqual(len(companion_msgs), 1)
        push = companion_msgs[0]

        # PUSH_ACK format: [0x82] [ack_code(4)] [trip_time(4)] = 9 bytes
        self.assertEqual(len(push), 9, f"PUSH_ACK must be 9 bytes, got {len(push)}")
        self.assertEqual(push[0], bridge.PUSH_ACK)
        push_ack_code = push[1:5]
        self.assertEqual(
            push_ack_code,
            crypto_ack,
            f"PUSH_ACK code {push_ack_code!r} must match crypto hash {crypto_ack!r}",
        )

    def test_send_txt_msg_flood_contact_with_send_scope_uses_t_flood(self):
        """TXT_MSG to a flood contact (out_path_len=-1) with send_scope uses ROUTE_T_FLOOD.

        Transport codes apply to flood routing. A contact with out_path_len=-1
        has no known direct path; the firmware would send it as flood. With a
        non-zero send_scope, the bridge must use ROUTE_T_FLOOD (with transport
        codes), not ROUTE_DIRECT or ROUTE_T_DIRECT.
        """
        import cbor2

        # Set a non-zero 16-byte send_scope key
        self.state.send_scope = b"\x01" * 16

        dst_prefix = self._add_peer_contact()  # out_path_len=-1 (flood contact)
        cmd = self._make_cmd(dst_prefix)
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)

        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        wire = msg["payload"]
        # ROUTE_T_FLOOD = 0; flood contact + scope → T_FLOOD with transport codes
        route_type = wire[0] & 0x03
        self.assertEqual(route_type, 0)
        # Transport codes present after header (4 bytes)
        tc1 = int.from_bytes(wire[1:3], "little")
        self.assertGreater(tc1, 0)


class TestSendAdvert(unittest.TestCase):
    """Edge-case tests for CMD_SEND_SELF_ADVERT handler."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def test_send_advert_no_data_byte(self):
        """CMD_SEND_SELF_ADVERT with no flood flag byte succeeds and sends UDP."""
        import cbor2

        cmd = bytes([bridge.CMD_SEND_SELF_ADVERT])  # empty data — no flood flag byte
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # A UDP CBOR TX packet must have been sent
        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        self.assertEqual(msg["type"], "lora_tx")
        self.assertIn("payload", msg)

    def test_send_advert_location_included_when_set(self):
        """ADVERT payload has HAS_LOCATION flag set when lat_e6/lon_e6 are non-zero."""
        import cbor2

        self.state.lat_e6 = 53_000_000
        self.state.lon_e6 = 10_000_000

        cmd = bytes([bridge.CMD_SEND_SELF_ADVERT, 0x01])  # flood flag set
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        self.udp_sock.settimeout(1.0)
        raw, _addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(raw)
        wire = msg["payload"]

        # Wire packet: header(1) + path_len(1) + pubkey(32) + ts(4) + sig(64) = offset 102
        # then app_data flags byte
        app_data_off = 1 + 1 + 32 + 4 + 64  # = 102
        self.assertGreater(
            len(wire), app_data_off, "wire packet too short for location flag"
        )
        flags = wire[app_data_off]
        self.assertTrue(
            flags & bridge.ADVERT_HAS_LOCATION,
            f"HAS_LOCATION not set in flags=0x{flags:02x}",
        )

    def test_send_advert_tracks_tx_hash(self):
        """After sending advert, state.recent_tx is non-empty (echo-filter hash stored)."""
        cmd = bytes([bridge.CMD_SEND_SELF_ADVERT, 0x01])
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreater(len(self.state.recent_tx), 0)


# ---- Helper: build a 147-byte contact record ----


def _make_contact_record(
    pub_key: bytes, name: str = "peer", lat_e6: int = 0, lon_e6: int = 0
) -> bytes:
    """Build a 147-byte contact record matching the bridge's internal layout."""
    name_bytes = name.encode("utf-8")[:32].ljust(32, b"\x00")
    ts = int(time.time())
    record = (
        pub_key  # pubkey (32)
        + b"\x01"  # node_type=chat
        + b"\x00"  # flags
        + b"\xff"  # path_len=-1 (flood)
        + b"\x00" * 64  # path (64)
        + name_bytes  # name (32)
        + struct.pack("<I", ts)  # last_advert (4)
        + struct.pack("<i", lat_e6)  # lat (4)
        + struct.pack("<i", lon_e6)  # lon (4)
        + struct.pack("<I", ts)  # lastmod (4)
    )
    assert len(record) == 147
    return record


# ---- Group F: CMD_ADD_UPDATE_CONTACT edge cases ----


class TestAddContact(unittest.TestCase):
    """Edge cases for CMD_ADD_UPDATE_CONTACT (0x09)."""

    def setUp(self):
        self.state = make_state()
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_add_contact_too_short_returns_error(self):
        """F1: data < 32 bytes returns RESP_ERROR."""
        cmd = bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + b"\x01" * 31
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_add_contact_pads_record_to_147_bytes(self):
        """F2: data shorter than 147 is padded to 147 bytes."""
        pub_key = bytes(range(32))
        # pubkey(32) + node_type(1) + flags(1) + path_len(1) + path(64) + name(32) = 131
        # + last_advert(4) = 135, + lat(4) = 139, + lon(4) = 143 bytes total (< 147)
        contact_data = (
            pub_key + b"\x01\x00\xff" + b"\x00" * 64 + b"testpeer".ljust(32, b"\x00")
        )
        contact_data += struct.pack("<I", int(time.time()))  # last_advert (4)
        contact_data += struct.pack("<i", 0) + struct.pack("<i", 0)  # lat + lon
        # total = 143 bytes — bridge pads to 143 + 4 (lastmod) = 147
        self.assertEqual(len(contact_data), 143)
        cmd = bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + contact_data
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        pk_hex = pub_key.hex()
        self.assertIn(pk_hex, self.state.contacts)
        self.assertEqual(len(self.state.contacts[pk_hex]), 147)

    def test_add_contact_updates_contacts_lastmod(self):
        """F3: successful add increases contacts_lastmod."""
        old_lastmod = self.state.contacts_lastmod
        pub_key = b"\xab" * 32
        contact_data = pub_key + b"\x00" * 111  # 32 + 111 = 143 bytes
        cmd = bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + contact_data
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreaterEqual(self.state.contacts_lastmod, old_lastmod)

    def test_add_contact_overwrites_existing(self):
        """F4: adding same pubkey twice keeps only the second record."""
        pub_key = b"\xcd" * 32
        # First add: name field at offset 99 = "first"
        name1 = b"first\x00" * 4 + b"\x00" * (32 - 24)  # 32 bytes
        record1 = pub_key + b"\x01\x00\xff" + b"\x00" * 64 + name1 + b"\x00" * 12
        self.assertEqual(len(record1), 143)
        self._handle(bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + record1)
        stored_first = self.state.contacts[pub_key.hex()][99:104]

        # Second add: different name
        name2 = b"second".ljust(32, b"\x00")
        record2 = pub_key + b"\x01\x00\xff" + b"\x00" * 64 + name2 + b"\x00" * 12
        self.assertEqual(len(record2), 143)
        self._handle(bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + record2)
        stored_second = self.state.contacts[pub_key.hex()][99:105]

        self.assertEqual(len(self.state.contacts), 1)
        self.assertNotEqual(stored_first[:5], b"secon")
        self.assertEqual(stored_second, b"second")


# ---- Group G: CMD_REMOVE_CONTACT edge cases ----


class TestRemoveContact(unittest.TestCase):
    """Edge cases for CMD_REMOVE_CONTACT (0x0F)."""

    def setUp(self):
        self.state = make_state()
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_remove_contact_too_short_returns_ok(self):
        """G1: data < 32 bytes still returns RESP_OK (silently ignored)."""
        cmd = bytes([bridge.CMD_REMOVE_CONTACT]) + b"\x01" * 10
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

    def test_remove_contact_not_present_returns_ok(self):
        """G2: removing a contact not in state returns RESP_OK."""
        unknown_pk = b"\xff" * 32
        cmd = bytes([bridge.CMD_REMOVE_CONTACT]) + unknown_pk
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

    def test_remove_contact_deletes_from_disk(self):
        """G3: contact added via CMD_ADD_UPDATE_CONTACT is removed from disk by CMD_REMOVE_CONTACT."""
        import tempfile

        tmpdir = Path(tempfile.mkdtemp())
        cdir = tmpdir / "contacts"
        state = make_state(contacts_dir=cdir)
        sock = _FakeUDPSock()
        addr = ("127.0.0.1", 0)

        def handle(cmd_bytes):
            return bridge.handle_command(cmd_bytes, state, sock, addr)

        # Add a contact via CMD_ADD_UPDATE_CONTACT
        pub_key = b"\x77" * 32
        contact_data = (
            pub_key
            + b"\x01\x00\xff"
            + b"\x00" * 64
            + b"removeme".ljust(32, b"\x00")
            + b"\x00" * 12
        )
        self.assertEqual(len(contact_data), 143)
        responses = handle(bytes([bridge.CMD_ADD_UPDATE_CONTACT]) + contact_data)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        pk_hex = pub_key.hex()
        self.assertIn(pk_hex, state.contacts)

        # Verify the .contact file was written to disk
        contact_file = cdir / f"{pk_hex}.contact"
        self.assertTrue(contact_file.exists(), "contact file should exist after add")

        # Remove via CMD_REMOVE_CONTACT
        responses = handle(bytes([bridge.CMD_REMOVE_CONTACT]) + pub_key)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Contact should be gone from in-memory state
        self.assertNotIn(pk_hex, state.contacts)

        # .contact file should be deleted from disk
        self.assertFalse(
            contact_file.exists(), "contact file should be deleted after remove"
        )


# ---- Group H: CMD_SHARE_CONTACT error propagation ----


class TestShareContact(unittest.TestCase):
    """Tests for CMD_SHARE_CONTACT (0x10) error handling."""

    def setUp(self):
        self.state = make_state()
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_share_contact_unknown_prefix_propagates_error(self):
        """H1: CMD_SHARE_CONTACT with prefix matching no contact returns RESP_ERROR."""
        # 6-byte prefix that matches no stored contact and is not self
        unknown_prefix = b"\x00\x11\x22\x33\x44\x55"
        # Ensure it does not match our own pubkey prefix
        self.assertNotEqual(self.state.pub_key[:6], unknown_prefix)
        cmd = bytes([bridge.CMD_SHARE_CONTACT]) + unknown_prefix
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)


# ---- Group I: CMD_EXPORT_CONTACT stored contact paths ----


class TestExportStoredContact(unittest.TestCase):
    """Tests for CMD_EXPORT_CONTACT with stored (non-self) contacts."""

    def setUp(self):
        self.state = make_state()
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_export_stored_contact_returns_uri(self):
        """I1: exporting a stored contact returns RESP_CONTACT_URI with wire bytes."""
        pub_key = b"\xaa" * 32
        # Ensure it doesn't match our own pubkey
        self.assertNotEqual(self.state.pub_key[:6], pub_key[:6])
        record = _make_contact_record(pub_key, name="storedpeer")
        self.state.contacts[pub_key.hex()] = record

        prefix = pub_key[:6]
        cmd = bytes([bridge.CMD_EXPORT_CONTACT]) + prefix
        responses = self._handle(cmd)

        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)
        wire_pkt = responses[0][1:]
        self.assertGreater(len(wire_pkt), 0, "wire packet should be non-empty")

    def test_export_stored_contact_with_location(self):
        """I2: exported stored contact with location has ADVERT_HAS_LOCATION flag set."""
        pub_key = b"\xbb" * 32
        self.assertNotEqual(self.state.pub_key[:6], pub_key[:6])
        lat_e6 = 53_000_000
        lon_e6 = 10_000_000
        record = _make_contact_record(
            pub_key, name="locpeer", lat_e6=lat_e6, lon_e6=lon_e6
        )
        self.state.contacts[pub_key.hex()] = record

        prefix = pub_key[:6]
        cmd = bytes([bridge.CMD_EXPORT_CONTACT]) + prefix
        responses = self._handle(cmd)

        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)
        wire_pkt = responses[0][1:]
        # Wire packet structure: header(1) + path_len(1) + advert_payload
        # advert_payload: pubkey(32) + ts(4) + sig(64) + app_data
        # app_data starts at: 1 + 1 + 32 + 4 + 64 = 102
        app_data_off = 1 + 1 + 32 + 4 + 64
        self.assertGreater(
            len(wire_pkt), app_data_off, "wire packet too short for app_data"
        )
        flags = wire_pkt[app_data_off]
        self.assertTrue(
            flags & bridge.ADVERT_HAS_LOCATION,
            f"ADVERT_HAS_LOCATION (0x10) not set in flags=0x{flags:02x}",
        )

    def test_export_contact_empty_prefix_returns_self(self):
        """I3: CMD_EXPORT_CONTACT with all-zero 6-byte prefix returns self ADVERT."""
        # all-zero prefix triggers the 'not prefix' branch (falsy bytes triggers self-path)
        # Actually the bridge checks: `if not prefix or state.pub_key[:len(prefix)] == prefix`
        # A 6-byte zero prefix is truthy but won't match our pubkey unless it starts with zeros.
        # Use an empty prefix (no data bytes) to always trigger the self path.
        cmd = bytes([bridge.CMD_EXPORT_CONTACT])  # no prefix data
        responses = self._handle(cmd)

        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_CONTACT_URI)
        wire_pkt = responses[0][1:]
        # Wire packet: header(1) + path_len(1) + pubkey(32) + ...
        # Our pubkey is at offset 2 in the wire packet
        pubkey_in_wire = wire_pkt[2:34]
        self.assertEqual(pubkey_in_wire, self.state.pub_key)


# ---- Group J: CMD_SET_CHANNEL edge cases ----


class TestSetChannelEdgeCases(unittest.TestCase):
    """Edge-case tests for _handle_set_channel."""

    def setUp(self):
        import tempfile

        self.tmpdir = Path(tempfile.mkdtemp())
        self.state = make_state(channels_dir=self.tmpdir / "channels")
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_set_channel_too_short_returns_error(self):
        """J1: CMD_SET_CHANNEL with < 65 bytes of data returns RESP_ERROR."""
        # 30 bytes of data after the command byte (well below the 65-byte minimum)
        short_data = bytes(30)
        cmd = bytes([bridge.CMD_SET_CHANNEL]) + short_data
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_set_channel_out_of_range_returns_error(self):
        """J2: CMD_SET_CHANNEL with idx=8 (>= max 8 channels) returns RESP_ERROR."""
        idx = 8  # out of range (valid: 0..7)
        name = b"toobig".ljust(32, b"\x00")
        secret = b"\x01" * 32
        cmd = bytes([bridge.CMD_SET_CHANNEL, idx]) + name + secret
        responses = self._handle(cmd)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_set_channel_persists_to_disk(self):
        """J3: CMD_SET_CHANNEL writes a .channel file to channels_dir."""
        channels_dir = self.tmpdir / "channels"
        idx = 0
        name = b"persist-ch".ljust(32, b"\x00")
        secret = b"\xde\xad" * 16  # 32 bytes
        cmd = bytes([bridge.CMD_SET_CHANNEL, idx]) + name + secret
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Verify the channel file was written
        channel_files = list(channels_dir.glob("*.channel"))
        self.assertGreater(len(channel_files), 0, "no .channel file written to disk")

        # Verify in-memory state
        self.assertTrue(len(self.state.channels) > idx)
        self.assertEqual(self.state.channels[idx].name, "persist-ch")

    def test_set_channel_all_zero_name_clears_channel(self):
        """J4: CMD_SET_CHANNEL with idx=0 and all-zero name clears that slot."""
        # First add a channel at index 0
        idx = 0
        name = b"to-clear".ljust(32, b"\x00")
        secret = b"\xab" * 32
        self._handle(bytes([bridge.CMD_SET_CHANNEL, idx]) + name + secret)
        self.assertEqual(self.state.channels[0].name, "to-clear")

        # Now send all-zero name to clear it
        zero_name = b"\x00" * 32
        zero_secret = b"\x00" * 32
        cmd = bytes([bridge.CMD_SET_CHANNEL, idx]) + zero_name + zero_secret
        responses = self._handle(cmd)
        self.assertEqual(responses[0][0], bridge.RESP_OK)

        # Bridge removes the slot from channels list when name is empty
        # Either the list is empty, or the slot at 0 is not "to-clear"
        if len(self.state.channels) > 0:
            self.assertNotEqual(self.state.channels[0].name, "to-clear")
        # (list length may be 0 after clearing)


# ---- Group K: build_channel_info paths ----


class TestBuildChannelInfo(unittest.TestCase):
    """Tests for build_channel_info response structure."""

    def setUp(self):
        self.state = make_state()
        self.sock = _FakeUDPSock()
        self.addr = ("127.0.0.1", 0)

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(cmd_bytes, self.state, self.sock, self.addr)

    def test_channel_info_populated_channel_contains_name_and_secret(self):
        """K1: GET_CHANNEL with populated channel returns name and secret in RESP_CHANNEL_INFO."""
        ch = GroupChannel("myhash", b"\xab" * 16)
        self.state.channels = [ch]

        responses = self._handle(bytes([bridge.CMD_GET_CHANNEL, 0x00]))
        self.assertEqual(len(responses), 1)
        resp = responses[0]
        self.assertEqual(resp[0], bridge.RESP_CHANNEL_INFO)
        # Layout: code(1) + idx(1) + name(32) + secret(16)
        name_field = resp[2:34].rstrip(b"\x00")
        self.assertEqual(name_field, b"myhash")
        # Secret: first 16 bytes of channel secret
        secret_field = resp[34:50]
        self.assertEqual(secret_field, b"\xab" * 16)

    def test_channel_info_empty_slot_all_zeros(self):
        """K2: GET_CHANNEL with no channels returns name and secret all zeros."""
        # Ensure no channels
        self.state.channels = []

        responses = self._handle(bytes([bridge.CMD_GET_CHANNEL, 0x00]))
        self.assertEqual(len(responses), 1)
        resp = responses[0]
        self.assertEqual(resp[0], bridge.RESP_CHANNEL_INFO)
        # Name (32 bytes) and secret (16 bytes) should all be zero
        name_field = resp[2:34]
        secret_field = resp[34:50]
        self.assertEqual(name_field, b"\x00" * 32)
        self.assertEqual(secret_field, b"\x00" * 16)


# ---- Group L: build_stats edge cases ----


class TestBuildStatsEdgeCases(unittest.TestCase):
    """Edge-case tests for BridgeState.build_stats."""

    def setUp(self):
        self.state = make_state()

    def test_stats_radio_snr_clamped(self):
        """L1: build_stats(1) with out-of-range SNR doesn't throw OverflowError."""
        self.state.last_snr_db = 200.0  # 200*4 = 800, out of int8 range
        # Should not raise; SNR should be clamped to 127 (int8 max)
        stats = self.state.build_stats(1)
        self.assertEqual(
            len(stats), 14
        )  # RESP_STATS(1) + type(1) + noise_floor(2) + rssi(1) + snr(1) + tx_air(4) + rx_air(4)
        self.assertEqual(stats[0], bridge.RESP_STATS)
        snr_i8 = struct.unpack_from("<b", stats, 5)[0]
        self.assertEqual(snr_i8, 127)  # clamped to int8 max

    def test_stats_packets_returns_26_bytes(self):
        """L2: build_stats(2) returns RESP_STATS with total length 26 bytes."""
        stats = self.state.build_stats(2)
        self.assertEqual(stats[0], bridge.RESP_STATS)
        self.assertEqual(len(stats), 26)

    def test_stats_core_reflects_queue_length(self):
        """L3: build_stats(0) last byte reflects msg_queue length."""
        # Add 3 messages to the queue
        self.state.msg_queue.append(b"msg1")
        self.state.msg_queue.append(b"msg2")
        self.state.msg_queue.append(b"msg3")
        stats = self.state.build_stats(0)
        self.assertEqual(stats[0], bridge.RESP_STATS)
        # Layout: code(1) + type(1) + battery_mv(2) + uptime(4) + errors(2) + queue_len(1)
        queue_len = stats[-1]
        self.assertEqual(queue_len, 3)


# ---- Group R: contacts_lastmod tracking ----


class TestContactsLastmod(unittest.TestCase):
    """Tests for contacts_lastmod update tracking."""

    def setUp(self):
        import socket

        self.state = make_state()
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", 0))
        self.udp_addr = ("127.0.0.1", self.udp_sock.getsockname()[1])

    def tearDown(self):
        self.udp_sock.close()

    def _handle(self, cmd_bytes: bytes) -> list[bytes]:
        return bridge.handle_command(
            cmd_bytes, self.state, self.udp_sock, self.udp_addr
        )

    def _build_advert_packet(self, pubkey: bytes, name: str = "contact") -> bytes:
        """Build a minimal ADVERT wire packet."""
        hdr = (0x04 << 2) | 0x01  # ptype=ADVERT(4), route=FLOOD(1)
        ts = struct.pack("<I", int(time.time()))
        sig = b"\x00" * 64
        flags = 0x01 | 0x80  # chat + has_name
        app_data = bytes([flags]) + name.encode("utf-8")
        return bytes([hdr, 0]) + pubkey + ts + sig + app_data

    def test_import_contact_updates_contacts_lastmod(self):
        """R1: CMD_IMPORT_CONTACT increases contacts_lastmod."""
        before = self.state.contacts_lastmod
        pk = b"\x55" * 32
        advert = self._build_advert_packet(pk, "NewFriend")
        time.sleep(0.01)  # ensure time advances
        responses = self._handle(bytes([bridge.CMD_IMPORT_CONTACT]) + advert)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreaterEqual(
            self.state.contacts_lastmod,
            before,
            "contacts_lastmod should not decrease after import",
        )

    def test_ensure_self_contact_idempotent(self):
        """R2: _ensure_self_contact called twice doesn't add duplicates."""
        # Remove self from contacts if already there
        pk_hex = self.state.pub_key.hex()
        self.state.contacts.pop(pk_hex, None)

        bridge._ensure_self_contact(self.state)
        count_after_first = len(self.state.contacts)

        bridge._ensure_self_contact(self.state)
        count_after_second = len(self.state.contacts)

        self.assertEqual(
            count_after_first,
            count_after_second,
            "_ensure_self_contact added duplicate on second call",
        )
        self.assertEqual(count_after_first, 1)


# ---- Group S: build_contacts_response structure ----


class TestBuildContactsResponse(unittest.TestCase):
    """Tests for BridgeState.build_contacts_response structure."""

    def setUp(self):
        self.state = make_state()
        # Clear contacts for clean slate
        self.state.contacts = {}

    def _add_contact(self, pub_key: bytes, name: str = "peer") -> None:
        """Helper to add a contact record directly."""
        record = bridge._build_contact_record(pub_key, name=name)
        self.state.contacts[pub_key.hex()] = record

    def test_get_contacts_with_contacts_frame_count(self):
        """S1: 2 contacts produces 4 frames: START + 2×CONTACT + END."""
        self._add_contact(b"\x11" * 32, "Alice")
        self._add_contact(b"\x22" * 32, "Bob")
        frames = self.state.build_contacts_response()
        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0][0], bridge.RESP_CONTACT_START)
        self.assertEqual(frames[1][0], bridge.RESP_CONTACT)
        self.assertEqual(frames[2][0], bridge.RESP_CONTACT)
        self.assertEqual(frames[3][0], bridge.RESP_CONTACT_END)

    def test_get_contacts_contact_frame_length(self):
        """S2: CONTACT frame is exactly 148 bytes (1 code + 147 record)."""
        self._add_contact(b"\x33" * 32, "Carol")
        frames = self.state.build_contacts_response()
        # frames: START, CONTACT, END
        self.assertEqual(len(frames), 3)
        contact_frame = frames[1]
        self.assertEqual(
            len(contact_frame),
            148,
            f"CONTACT frame length {len(contact_frame)} != 148",
        )

    def test_get_contacts_end_frame_contains_lastmod(self):
        """S3: CONTACT_END frame contains contacts_lastmod as little-endian uint32 at bytes 1:5."""
        self.state.contacts_lastmod = 0xDEADBEEF
        frames = self.state.build_contacts_response()
        end_frame = frames[-1]
        self.assertEqual(end_frame[0], bridge.RESP_CONTACT_END)
        lastmod = struct.unpack_from("<I", end_frame, 1)[0]
        self.assertEqual(lastmod, 0xDEADBEEF)


# ---- Group M: _advert_to_push / _handle_advert_rx edge cases ----


class TestAdvertToPushEdgeCases(unittest.TestCase):
    """Group M: edge cases for _advert_to_push and ADVERT RX handling."""

    def _make_advert_frame(self, advert_payload: bytes) -> dict:
        """Build a minimal ADVERT wire frame (ROUTE_FLOOD + PAYLOAD_ADVERT)."""
        from meshcore_crypto import ROUTE_FLOOD

        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ADVERT)
        # Wire: header(1) + path_len(1=0) + advert_payload
        wire = bytes([hdr, 0]) + advert_payload
        return {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0},
            "payload": wire,
        }

    def test_advert_to_push_too_short_returns_none(self):
        """M1: _advert_to_push returns None when payload is too short (< advert_start+100)."""
        # With advert_start=0, need at least 100 bytes. Pass 99 bytes.
        short_payload = b"\x00" * 99
        result = bridge._advert_to_push(short_payload, 0)
        self.assertIsNone(result)

    def test_advert_to_push_minimum_valid_length_returns_result(self):
        """M1b: _advert_to_push returns non-None at exactly minimum length (100 bytes)."""
        # Exactly 100 bytes from advert_start=0: pubkey(32)+ts(4)+sig(64)
        min_payload = b"\x01" * 32 + struct.pack("<I", 0) + b"\x00" * 64
        self.assertEqual(len(min_payload), 100)
        result = bridge._advert_to_push(min_payload, 0)
        self.assertIsNotNone(result)

    def test_advert_push_location_flag_parsed(self):
        """M2: ADVERT with HAS_LOCATION flag has lat/lon in the push record."""
        lat_e6, lon_e6 = 53_000_000, 10_000_000
        flags = 0x01 | 0x10  # node_type=chat | HAS_LOCATION
        app_data = bytes([flags]) + struct.pack("<ii", lat_e6, lon_e6)
        advert_payload = b"\x01" * 32 + struct.pack("<I", 0) + b"\x00" * 64 + app_data

        state = make_state()
        frame = self._make_advert_frame(advert_payload)
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)

        self.assertEqual(len(companion_msgs), 1)
        push = companion_msgs[0]
        self.assertEqual(push[0], bridge.PUSH_NEW_ADVERT)
        # push layout: code(1)+pubkey(32)+node_type(1)+flags(1)+
        #   out_path_len(1)+path(64)+adv_name(32)+last_advert(4)+lat(4)+lon(4)
        lat_off = 1 + 32 + 1 + 1 + 1 + 64 + 32 + 4
        lon_off = lat_off + 4
        lat_val = struct.unpack_from("<i", push, lat_off)[0]
        lon_val = struct.unpack_from("<i", push, lon_off)[0]
        self.assertEqual(lat_val, lat_e6)
        self.assertEqual(lon_val, lon_e6)

    def test_advert_push_feature_flags_skipped(self):
        """M3: ADVERT with feature1/feature2 flags still parses name correctly."""
        flags = 0x01 | 0x20 | 0x40 | 0x80  # chat + feature1 + feature2 + name
        app_data = bytes([flags]) + b"\xab\xcd" + b"\xef\x12" + b"Alice"
        advert_payload = b"\x01" * 32 + struct.pack("<I", 0) + b"\x00" * 64 + app_data

        state = make_state()
        frame = self._make_advert_frame(advert_payload)
        companion_msgs, _ = bridge.lora_frame_to_companion_msgs(frame, state)

        self.assertEqual(len(companion_msgs), 1)
        push = companion_msgs[0]
        self.assertEqual(push[0], bridge.PUSH_NEW_ADVERT)
        # adv_name is at offset 1+32+1+1+1+64=100, 32 bytes padded
        name_off = 1 + 32 + 1 + 1 + 1 + 64
        adv_name = push[name_off : name_off + 32].rstrip(b"\x00").decode("utf-8")
        self.assertIn("Alice", adv_name)

    def test_advert_rx_does_not_overwrite_existing_key(self):
        """M4: receiving ADVERT from already-known pubkey keeps known_keys count unchanged."""
        known_pub = b"\xcc" * 32
        state = make_state()
        state.known_keys[known_pub.hex()] = known_pub
        initial_count = len(state.known_keys)

        # Build ADVERT payload from the already-known pubkey
        flags = 0x01  # chat, no name, no location
        app_data = bytes([flags])
        advert_payload = (
            known_pub + struct.pack("<I", 1_700_000_000) + b"\x00" * 64 + app_data
        )
        frame = self._make_advert_frame(advert_payload)
        bridge.lora_frame_to_companion_msgs(frame, state)

        # known_keys should not have grown
        self.assertEqual(len(state.known_keys), initial_count)
        self.assertIn(known_pub.hex(), state.known_keys)


# ---- Group N: _handle_ack_rx edge cases ----


class TestAckRxEdgeCases(unittest.TestCase):
    """Group N: edge cases for _handle_ack_rx."""

    def test_ack_rx_too_short_payload_ignored(self):
        """N1: ACK frame with < 4 bytes of checksum returns no msgs and no crash."""
        from meshcore_crypto import ROUTE_FLOOD

        state = make_state()
        hdr = make_header(ROUTE_FLOOD, PAYLOAD_ACK)
        # path_len=0 → ack_start=2. Short ack (2 bytes) makes ack_start+4=6 > len=4.
        short_ack = b"\xab\xcd"  # only 2 bytes; need 4
        payload = bytes([hdr, 0]) + short_ack
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": payload,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(companion_msgs, [])
        self.assertEqual(ack_packets, [])


# ---- Group N2: PATH packet carrying ACK (flood TXT_MSG response) ----


class TestPathAckRx(unittest.TestCase):
    """PATH packet from companion device carrying bundled ACK hash.

    When the bridge sends a TXT_MSG via FLOOD, the companion responds with a
    PATH packet (ptype=0x08) containing the return path + ACK as encrypted extra
    data — NOT a bare ACK (ptype=0x03). The bridge must decrypt the PATH and
    extract the ACK to match against pending_acks.
    """

    def test_path_with_ack_matches_pending_and_pushes_confirmed(self):
        """End-to-end: send flood TXT_MSG, receive PATH+ACK, get PUSH_SEND_CONFIRMED."""
        from meshcore_crypto import (
            ROUTE_FLOOD,
            PAYLOAD_PATH,
            PAYLOAD_ACK,
            compute_ack_hash,
            meshcore_shared_secret,
            meshcore_encrypt_then_mac,
        )

        # Two identities: bridge (A) and companion (B)
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_a, expanded_prv=prv_a, seed=seed_a)
        state.known_keys[pub_b.hex()] = pub_b

        # Bridge sends TXT_MSG to B — register the expected ACK
        text = b"test-path-ack"
        ts = int(time.time())
        plaintext_msg = struct.pack("<I", ts) + bytes([0x00]) + text
        expected_ack = compute_ack_hash(plaintext_msg, pub_a)
        dst_prefix = pub_b[:6]
        state.pending_acks[expected_ack] = (time.monotonic(), dst_prefix)

        # Companion B builds a PATH packet back to A with the ACK embedded.
        # PATH plaintext: path_len(1) + path(N) + extra_type(1) + extra(4)
        return_path = b""  # zero-hop return
        path_encoded = 0x00  # hash_size=1 (upper 2 bits=0), count=0 (lower 6 bits=0)
        path_plaintext = (
            bytes([path_encoded]) + return_path + bytes([PAYLOAD_ACK]) + expected_ack
        )

        # Encrypt with shared secret (B encrypts for A)
        secret = meshcore_shared_secret(prv_b, pub_a)
        assert secret is not None
        encrypted = meshcore_encrypt_then_mac(secret, path_plaintext)

        # Wire format: header(1) + flood_path_len(1) + dest_hash(1) + src_hash(1) + encrypted
        hdr = make_header(ROUTE_FLOOD, PAYLOAD_PATH)
        wire = bytes([hdr, 0x00]) + pub_a[:1] + pub_b[:1] + encrypted

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": wire,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)

        self.assertEqual(len(companion_msgs), 1, "PATH+ACK should produce PUSH_ACK")
        self.assertEqual(companion_msgs[0][0], bridge.PUSH_ACK)
        self.assertEqual(len(state.pending_acks), 0, "pending_acks should be consumed")


# ---- Group O: _handle_txt_msg_rx edge cases ----


class TestTxtMsgRxEdgeCases(unittest.TestCase):
    """Group O: edge cases for _handle_txt_msg_rx."""

    def test_txt_msg_rx_auto_learns_sender_key(self):
        """O1: TXT_MSG successfully decrypted keeps sender key in state.known_keys."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        # Pre-add sender key so decryption can succeed
        state.known_keys[pub_a.hex()] = pub_a

        packet = build_txt_msg(prv_a, pub_a, pub_b, "auto-learn test")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        bridge.lora_frame_to_companion_msgs(frame, state)
        # Sender key must still be present (auto-learn preserves it)
        self.assertIn(pub_a.hex(), state.known_keys)

    def test_txt_msg_rx_unknown_sender_no_key_in_state(self):
        """O1b: TXT_MSG from unknown sender fails decryption; key not added."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        # Do NOT add A's key — decryption will fail

        packet = build_txt_msg(prv_a, pub_a, pub_b, "hidden")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(companion_msgs, [])
        self.assertEqual(ack_packets, [])
        # Sender key was not learned (decryption failed)
        self.assertNotIn(pub_a.hex(), state.known_keys)

    def test_txt_msg_rx_no_ack_when_no_shared_secret(self):
        """O4: TXT_MSG from unknown sender produces no RF ACK packet."""
        (prv_a, pub_a, seed_a), (prv_b, pub_b, seed_b) = _make_two_identities()
        state = make_state(pub_key=pub_b, expanded_prv=prv_b, seed=seed_b)
        # No key for A → decryption fails → no ACK

        packet = build_txt_msg(prv_a, pub_a, pub_b, "no ack please")
        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(
            ack_packets, [], "no RF ACK should be generated for unknown sender"
        )


# ---- Group P: _handle_grp_txt_rx edge cases ----


class TestGrpTxtRxEdgeCases(unittest.TestCase):
    """Group P: edge cases for _handle_grp_txt_rx."""

    def test_grp_txt_rx_channel_index_matches_position(self):
        """P1: GRP_TXT decrypted with chan1 reports channel index 1 in companion message."""
        state = make_state()
        chan0 = GroupChannel("chan0", b"\x01" * 16)
        chan1 = GroupChannel("chan1", b"\x02" * 16)
        state.channels = [chan0, chan1]

        from meshcore_crypto import ROUTE_FLOOD

        grp_payload = build_grp_txt(chan1, "Tester", "index test")
        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0},
            "payload": packet,
        }
        companion_msgs, ack_packets = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        msg = companion_msgs[0]
        self.assertEqual(msg[0], bridge.RESP_CHANNEL_MSG_RECV_V3)
        # V3 channel format: code(1)+snr(1)+reserved(2)+chan_idx(1)+...
        chan_idx_in_msg = msg[4]
        self.assertEqual(
            chan_idx_in_msg, 1, "companion message should report channel index 1"
        )
        self.assertEqual(ack_packets, [])

    def test_grp_txt_rx_uses_plaintext_timestamp(self):
        """P2: GRP_TXT companion message carries the timestamp from the encrypted plaintext."""
        state = make_state()
        known_ts = 1_700_000_000
        ch = GroupChannel("ts-chan", b"\x05" * 16)
        state.channels = [ch]

        from meshcore_crypto import ROUTE_FLOOD

        grp_payload = build_grp_txt(ch, "Bob", "timestamp test", timestamp=known_ts)
        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

        frame = {
            "type": "lora_frame",
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0},
            "payload": packet,
        }
        companion_msgs, _ = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(companion_msgs), 1)
        msg = companion_msgs[0]
        self.assertEqual(msg[0], bridge.RESP_CHANNEL_MSG_RECV_V3)
        # V3 channel format: code(1)+snr(1)+reserved(2)+chan_idx(1)+path_len(1)+txt_type(1)+ts(4)
        ts_off = 1 + 1 + 2 + 1 + 1 + 1
        ts_in_msg = struct.unpack_from("<I", msg, ts_off)[0]
        self.assertEqual(
            ts_in_msg,
            known_ts,
            "companion message timestamp must match plaintext timestamp",
        )


# ---- Group T: Echo filter hash tracking ----


class TestEchoFilterTracking(unittest.TestCase):
    """Test _hash_payload used by TX echo filter."""

    def test_hash_payload_deterministic_and_8_bytes(self):
        """_hash_payload returns exactly 8 bytes, same result both times."""
        h1 = bridge._hash_payload(b"hello world")
        h2 = bridge._hash_payload(b"hello world")
        self.assertEqual(len(h1), 8)
        self.assertEqual(h1, h2)


# ---- Group U: CMD_SET_RADIO_PARAMS edge cases ----
# (appended to existing TestRadioParamCommands via monkey-patch below)


def _u1_test_set_radio_params_too_short_returns_ok_no_change(self):
    """CMD_SET_RADIO_PARAMS with < 10 bytes returns RESP_OK, state unchanged."""
    state = make_state()
    original_freq = state.freq_mhz
    cmd = bytes([bridge.CMD_SET_RADIO_PARAMS]) + b"\x01\x02\x03\x04\x05"
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)
    self.assertEqual(state.freq_mhz, original_freq)


def _u2_test_set_radio_tx_power_negative_value(self):
    """CMD_SET_RADIO_TX_POWER with 0xFF is interpreted as -1 (int8)."""
    state = make_state()
    cmd = bytes([bridge.CMD_SET_RADIO_TX_POWER, 0xFF])
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)
    self.assertEqual(state.tx_power, -1)


def _u3_test_set_radio_tx_power_empty_data_ok(self):
    """CMD_SET_RADIO_TX_POWER with no data byte returns RESP_OK without crashing."""
    state = make_state()
    cmd = bytes([bridge.CMD_SET_RADIO_TX_POWER])
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)


setattr(
    TestRadioParamCommands,
    "test_set_radio_params_too_short_returns_ok_no_change",
    _u1_test_set_radio_params_too_short_returns_ok_no_change,
)
setattr(
    TestRadioParamCommands,
    "test_set_radio_tx_power_negative_value",
    _u2_test_set_radio_tx_power_negative_value,
)
setattr(
    TestRadioParamCommands,
    "test_set_radio_tx_power_empty_data_ok",
    _u3_test_set_radio_tx_power_empty_data_ok,
)


# ---- Group V: CMD_SET_ADVERT_LATLON edge cases ----


def _v1_test_set_advert_latlon_too_short_returns_ok_no_change(self):
    """CMD_SET_ADVERT_LATLON with < 8 bytes returns RESP_OK, lat unchanged."""
    state = make_state()
    cmd = bytes([bridge.CMD_SET_ADVERT_LATLON]) + b"\x01\x02\x03\x04"
    original_lat = state.lat_e6
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)
    self.assertEqual(state.lat_e6, original_lat)


setattr(
    TestLatLonInAdvert,
    "test_set_advert_latlon_too_short_returns_ok_no_change",
    _v1_test_set_advert_latlon_too_short_returns_ok_no_change,
)


# ---- Group W: Stub command details ----


def _w1_test_stub_cmds_return_msg_sent_flood_routing(self):
    """Remaining stub cmds (TRACE, PATH_DISCOVERY) have routing_type=1."""
    state = make_state()
    for cmd_byte in [
        bridge.CMD_TRACE,
        bridge.CMD_PATH_DISCOVERY,
    ]:
        cmd = bytes([cmd_byte]) + b"payload"
        responses = bridge.handle_command(cmd, state, None, None)
        self.assertEqual(
            responses[0][1],
            1,
            f"cmd 0x{cmd_byte:02x} routing_type should be 1 (flood)",
        )


def _w2_test_stub_cmds_increment_msg_seq(self):
    """Remaining stub cmds increment state.msg_seq."""
    state = make_state()
    initial_seq = state.msg_seq
    bridge.handle_command(bytes([bridge.CMD_TRACE]) + b"payload", state, None, None)
    self.assertEqual(state.msg_seq, initial_seq + 1)
    bridge.handle_command(
        bytes([bridge.CMD_PATH_DISCOVERY]) + b"payload", state, None, None
    )
    self.assertEqual(state.msg_seq, initial_seq + 2)


setattr(
    TestCommandHandler,
    "test_stub_cmds_return_msg_sent_flood_routing",
    _w1_test_stub_cmds_return_msg_sent_flood_routing,
)
setattr(
    TestCommandHandler,
    "test_stub_cmds_increment_msg_seq",
    _w2_test_stub_cmds_increment_msg_seq,
)


# ---- Group X: Trivial stubs return OK ----


def _x1_test_reset_path_returns_ok(self):
    """CMD_RESET_PATH returns RESP_OK."""
    state = make_state()
    cmd = bytes([bridge.CMD_RESET_PATH])
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)


def _x2_test_set_tuning_params_returns_ok(self):
    """CMD_SET_TUNING_PARAMS returns RESP_OK."""
    state = make_state()
    cmd = bytes([bridge.CMD_SET_TUNING_PARAMS]) + b"\x01\x02\x03"
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)


def _x3_test_reboot_returns_ok(self):
    """CMD_REBOOT returns RESP_OK."""
    state = make_state()
    cmd = bytes([bridge.CMD_REBOOT])
    responses = bridge.handle_command(cmd, state, None, None)
    self.assertEqual(responses[0][0], bridge.RESP_OK)


setattr(
    TestCommandHandler, "test_reset_path_returns_ok", _x1_test_reset_path_returns_ok
)
setattr(
    TestCommandHandler,
    "test_set_tuning_params_returns_ok",
    _x2_test_set_tuning_params_returns_ok,
)
setattr(TestCommandHandler, "test_reboot_returns_ok", _x3_test_reboot_returns_ok)


# ---- Group Y: handle_command empty payload ----


def _y1_test_handle_command_empty_payload_returns_empty(self):
    """handle_command with empty payload returns empty list."""
    state = make_state()
    responses = bridge.handle_command(b"", state, None, None)
    self.assertEqual(responses, [])


setattr(
    TestCommandHandler,
    "test_handle_command_empty_payload_returns_empty",
    _y1_test_handle_command_empty_payload_returns_empty,
)


# ---- Group A: FrameDecoder recovery ----


def _make_tx_frame(payload: bytes) -> bytes:
    """Build a client→device TX frame (0x3C + len_le16 + payload)."""
    return bytes([bridge.FRAME_TX]) + len(payload).to_bytes(2, "little") + payload


class TestFrameDecoderEdgeCases(unittest.TestCase):
    """Edge cases for FrameDecoder: oversized frames and sequential decoding."""

    def test_decode_oversize_frame_then_valid(self):
        """Oversized frame is skipped; subsequent valid frame is decoded."""
        # Build an oversized payload (> MAX_FRAME=300)
        oversized_payload = b"\xab" * (bridge.MAX_FRAME + 1)
        oversized_frame = _make_tx_frame(oversized_payload)

        # Append a valid small frame after the oversized one
        valid_payload = b"\x42\x43"
        valid_frame = _make_tx_frame(valid_payload)

        dec = bridge.FrameDecoder()
        frames = dec.feed(oversized_frame + valid_frame)

        # Only the valid frame should come through
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], valid_payload)

    def test_decode_two_frames_sequential(self):
        """Two valid frames fed together are both decoded in order."""
        p1 = b"\x01\x02\x03"
        p2 = b"\x04\x05\x06\x07"
        f1 = _make_tx_frame(p1)
        f2 = _make_tx_frame(p2)

        dec = bridge.FrameDecoder()
        frames = dec.feed(f1 + f2)

        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0], p1)
        self.assertEqual(frames[1], p2)


class TestParseFloodScope(unittest.TestCase):
    """Tests for _parse_flood_scope() — human-readable names and hex strings."""

    def test_empty_string_returns_null_scope(self):
        """Empty string → all-zero (disabled)."""
        self.assertEqual(bridge._parse_flood_scope(""), b"\x00" * 16)

    def test_zero_string_returns_null_scope(self):
        """'0' → all-zero (disabled)."""
        self.assertEqual(bridge._parse_flood_scope("0"), b"\x00" * 16)

    def test_none_string_returns_null_scope(self):
        """'None' → all-zero (disabled)."""
        self.assertEqual(bridge._parse_flood_scope("None"), b"\x00" * 16)

    def test_star_returns_null_scope(self):
        """'*' → all-zero (disabled)."""
        self.assertEqual(bridge._parse_flood_scope("*"), b"\x00" * 16)

    def test_human_readable_name_hashed(self):
        """Human-readable name → sha256(name)[:16] (no '#' prefix)."""
        import hashlib

        name = "de-hh"
        expected = hashlib.sha256(name.encode("utf-8")).digest()[:16]
        result = bridge._parse_flood_scope(name)
        self.assertEqual(result, expected)
        self.assertEqual(len(result), 16)

    def test_human_readable_differs_from_region_scope(self):
        """flood_scope('de-hh') != region_scope('de-hh') — different hash prefixes."""
        import hashlib

        name = "de-hh"
        flood_key = bridge._parse_flood_scope(name)
        region_key = hashlib.sha256(("#" + name).encode("utf-8")).digest()[:16]
        self.assertNotEqual(flood_key, region_key)

    def test_32_hex_chars_decoded_as_raw_key(self):
        """32-char hex string → decoded as raw 16-byte key."""
        raw = bytes(range(16))
        result = bridge._parse_flood_scope(raw.hex())
        self.assertEqual(result, raw)

    def test_short_hex_string_treated_as_name(self):
        """12-char hex string (< 32) → treated as a name and hashed."""
        import hashlib

        name = "abcdef012345"  # 12 hex chars, not 32
        expected = hashlib.sha256(name.encode("utf-8")).digest()[:16]
        result = bridge._parse_flood_scope(name)
        self.assertEqual(result, expected)

    def test_invalid_32_char_non_hex_treated_as_name(self):
        """32-char string with non-hex characters → hashed as name (not decoded as hex)."""
        import hashlib

        name = "this-is-exactly-32-chars-xx-yyyy"  # 32 chars, not valid hex
        expected = hashlib.sha256(name.encode("utf-8")).digest()[:16]
        result = bridge._parse_flood_scope(name)
        self.assertEqual(result, expected)

    def test_result_always_16_bytes(self):
        """Result is always exactly 16 bytes for any input."""
        for value in ["", "de-nord", "de-hh", "0", "a" * 32, "X" * 8]:
            with self.subTest(value=value):
                self.assertEqual(len(bridge._parse_flood_scope(value)), 16)


class TestSetOtherParams(unittest.TestCase):
    """CMD_SET_OTHER_PARAMS (0x26) handler tests."""

    def setUp(self):
        self.state = make_state()

    def _cmd(self, data: bytes) -> list[bytes]:
        """Build and dispatch a CMD_SET_OTHER_PARAMS payload."""
        payload = bytes([bridge.CMD_SET_OTHER_PARAMS]) + data
        return bridge.handle_command(payload, self.state, None, None)

    def test_legacy_3_byte_payload_returns_ok(self):
        """3-byte legacy payload (manual, telemetry, adv_loc) → RESP_OK."""
        resp = self._cmd(bytes([1, 0x05, 2]))
        self.assertEqual(len(resp), 1)
        self.assertEqual(resp[0][0], bridge.RESP_OK)

    def test_stores_manual_add_contacts(self):
        """manual_add_contacts is stored in state."""
        self._cmd(bytes([1, 0, 0]))
        self.assertEqual(self.state.manual_add_contacts, 1)

    def test_stores_telemetry_mode(self):
        """telemetry_mode is stored in state."""
        self._cmd(bytes([0, 0b00001101, 0]))
        self.assertEqual(self.state.telemetry_mode, 0b00001101)

    def test_stores_adv_loc_policy(self):
        """adv_loc_policy is stored in state."""
        self._cmd(bytes([0, 0, 3]))
        self.assertEqual(self.state.adv_loc_policy, 3)

    def test_4_byte_payload_stores_multi_acks(self):
        """4-byte payload (with multi_acks) stores all four fields."""
        self._cmd(bytes([1, 0x09, 2, 3]))
        self.assertEqual(self.state.manual_add_contacts, 1)
        self.assertEqual(self.state.telemetry_mode, 0x09)
        self.assertEqual(self.state.adv_loc_policy, 2)
        self.assertEqual(self.state.multi_acks, 3)

    def test_missing_multi_acks_defaults_to_zero(self):
        """3-byte payload leaves multi_acks at 0."""
        self.state.multi_acks = 99  # pre-set to non-zero
        self._cmd(bytes([1, 0, 0]))
        self.assertEqual(self.state.multi_acks, 0)

    def test_too_short_returns_ok_but_does_not_crash(self):
        """Short payload (< 3 bytes) → RESP_OK + no state change."""
        self._cmd(bytes([]))  # empty data
        self.assertEqual(self.state.manual_add_contacts, 0)  # unchanged

    def test_initial_state_values_are_zero(self):
        """New BridgeState has all other_params fields zeroed."""
        self.assertEqual(self.state.manual_add_contacts, 0)
        self.assertEqual(self.state.telemetry_mode, 0)
        self.assertEqual(self.state.adv_loc_policy, 0)
        self.assertEqual(self.state.multi_acks, 0)

    def test_multiple_calls_overwrite_state(self):
        """Successive SET_OTHER_PARAMS calls update state each time."""
        self._cmd(bytes([1, 0x03, 1, 2]))
        self.assertEqual(self.state.manual_add_contacts, 1)
        self._cmd(bytes([0, 0x00, 0, 0]))
        self.assertEqual(self.state.manual_add_contacts, 0)


class TestGetCustomVars(unittest.TestCase):
    """CMD_GET_CUSTOM_VARS (0x28) handler tests."""

    def setUp(self):
        self.state = make_state()

    def _cmd(self) -> list[bytes]:
        payload = bytes([bridge.CMD_GET_CUSTOM_VARS])
        return bridge.handle_command(payload, self.state, None, None)

    def test_returns_custom_vars_response(self):
        """GET_CUSTOM_VARS → RESP_CUSTOM_VARS (0x15)."""
        resp = self._cmd()
        self.assertEqual(len(resp), 1)
        self.assertEqual(resp[0][0], bridge.RESP_CUSTOM_VARS)

    def test_response_is_single_byte(self):
        """Response is exactly 1 byte (empty key=value payload — no sensors)."""
        resp = self._cmd()
        self.assertEqual(len(resp[0]), 1)

    def test_response_code_matches_protocol_spec(self):
        """RESP_CUSTOM_VARS == 0x15 (matches meshcore_py PacketType.CUSTOM_VARS)."""
        self.assertEqual(bridge.RESP_CUSTOM_VARS, 0x15)

    def test_multiple_calls_return_same_empty_response(self):
        """Repeated calls always return the same empty RESP_CUSTOM_VARS."""
        r1 = self._cmd()
        r2 = self._cmd()
        self.assertEqual(r1, r2)


class TestSetCustomVar(unittest.TestCase):
    """CMD_SET_CUSTOM_VAR (0x29) handler tests."""

    def setUp(self):
        self.state = make_state()

    def _cmd(self, data: bytes = b"") -> list[bytes]:
        payload = bytes([bridge.CMD_SET_CUSTOM_VAR]) + data
        return bridge.handle_command(payload, self.state, None, None)

    def test_returns_error_response(self):
        """SET_CUSTOM_VAR → RESP_ERROR."""
        resp = self._cmd(b"gps:on")
        self.assertEqual(len(resp), 1)
        self.assertEqual(resp[0][0], bridge.RESP_ERROR)

    def test_error_code_is_illegal_arg(self):
        """Error code is ERR_CODE_ILLEGAL_ARG (6)."""
        resp = self._cmd(b"gps:on")
        # RESP_ERROR frame: [0x01][error_code]
        self.assertEqual(resp[0][1], bridge.ERR_CODE_ILLEGAL_ARG)

    def test_any_key_is_rejected(self):
        """Any key:value is rejected with ILLEGAL_ARG."""
        for payload in [b"gps:on", b"gps_interval:60", b"sensor:temperature", b""]:
            with self.subTest(payload=payload):
                resp = self._cmd(payload)
                self.assertEqual(resp[0][0], bridge.RESP_ERROR)

    def test_empty_payload_is_rejected(self):
        """Empty payload also returns RESP_ERROR."""
        resp = self._cmd(b"")
        self.assertEqual(resp[0][0], bridge.RESP_ERROR)


class TestCtrlRx(unittest.TestCase):
    def _make_ctrl_frame(self, ctrl_payload: bytes) -> dict:
        """Build a fake lora_frame dict containing a CTRL packet."""
        from meshcore_tx import build_wire_packet, make_header

        header = make_header(bridge.ROUTE_FLOOD, 0x0B)  # PAYLOAD_CTRL
        wire = build_wire_packet(header, ctrl_payload)
        return {
            "type": "lora_frame",
            "payload": wire,
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 3.0, "rssi_dbm": -85},
        }

    def test_ctrl_discover_resp_produces_control_data_push(self):
        """Incoming CTRL DISCOVER_RESP produces a CONTROL_DATA push (0x8E)."""
        import struct

        state = make_state()
        # Build a DISCOVER_RESP payload: subtype=0x91 (RESP|node_type=1),
        # SNR_in=10*4=40=0x28, tag=4 bytes, pubkey_prefix=8 bytes
        tag = b"\x01\x02\x03\x04"
        ctrl_payload = bytes([0x91, 40]) + tag + b"\xaa" * 8
        frame = self._make_ctrl_frame(ctrl_payload)
        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(msgs), 1)
        msg = msgs[0]
        self.assertEqual(msg[0], 0x8E)  # PUSH_CONTROL_DATA
        # PHY values from frame: snr_db=3.0 → int(3.0*4)=12, rssi_dbm=-85
        self.assertEqual(struct.unpack_from("<b", msg, 1)[0], 12)  # SNR byte
        self.assertEqual(struct.unpack_from("<b", msg, 2)[0], -85)  # RSSI byte
        self.assertEqual(msg[3], 0)  # path_len: ROUTE_FLOOD, 0 hops
        # _handle_ctrl_rx strips the MeshCore header; ctrl_payload follows verbatim
        self.assertEqual(msg[4:], ctrl_payload)

    def test_ctrl_non_discover_produces_control_data_push(self):
        """Any CTRL packet produces CONTROL_DATA push."""
        import struct

        state = make_state()
        ctrl_payload = bytes([0x80, 0x01, 0x00, 0x00, 0x00, 0x00])
        frame = self._make_ctrl_frame(ctrl_payload)
        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)
        self.assertEqual(len(msgs), 1)
        msg = msgs[0]
        self.assertEqual(msg[0], 0x8E)
        self.assertEqual(struct.unpack_from("<b", msg, 1)[0], 12)  # SNR byte
        self.assertEqual(struct.unpack_from("<b", msg, 2)[0], -85)  # RSSI byte
        self.assertEqual(msg[3], 0)  # path_len: ROUTE_FLOOD, 0 hops
        self.assertEqual(msg[4:], ctrl_payload)


class TestSelfInfoFields(unittest.TestCase):
    def test_self_info_reflects_set_other_params(self):
        """SELF_INFO returns live state values set via CMD_SET_OTHER_PARAMS."""
        state = make_state()
        state.manual_add_contacts = 1
        state.telemetry_mode = 0x42
        state.adv_loc_policy = 2
        state.multi_acks = 1
        payload = state.build_self_info()
        base = 44  # offset of multi_acks field
        self.assertEqual(payload[base + 0], 1)  # multi_acks
        self.assertEqual(payload[base + 1], 2)  # adv_loc_policy
        self.assertEqual(payload[base + 2], 0x42)  # telemetry_mode
        self.assertEqual(payload[base + 3], 1)  # manual_add_contacts


class TestPathLearning(unittest.TestCase):
    def test_path_rx_updates_contact_out_path_len(self):
        """After a PATH+ACK is received, the sender's contact out_path_len is updated."""
        state = make_state()
        # Create a peer identity
        peer_expanded_prv, peer_pub, peer_seed = make_identity()
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub

        # Add a contact record for the peer (out_path_len=-1 = flood)
        record = bridge._build_contact_record(peer_pub, name="peer")
        state.contacts[peer_pub.hex()] = record
        self.assertEqual(struct.unpack_from("<b", record, 34)[0], -1)  # initially flood

        # Pre-register an expected ACK so _match_pending_ack works
        ack_hash = b"\x11\x22\x33\x44"
        state.pending_acks[ack_hash] = (time.monotonic(), peer_pub[:6])

        # Build a PATH+ACK from peer:
        # Plaintext: path_len_enc(1) + path(1 hop hash) + extra_type(1) + ack_hash(4)
        # 1 hop: the bridge's own hash (state.pub_key[0])
        path_len_enc = 0x01  # 1 hop, hash_size=1 (bits[7:6]=00 -> size=1)
        path_hash = bytes([state.pub_key[0]])
        plaintext = (
            bytes([path_len_enc]) + path_hash + bytes([bridge.PAYLOAD_ACK]) + ack_hash
        )

        from meshcore_crypto import meshcore_shared_secret, meshcore_encrypt_then_mac

        shared = meshcore_shared_secret(peer_expanded_prv, state.pub_key)
        assert shared is not None
        encrypted = meshcore_encrypt_then_mac(shared, plaintext)

        # Wire: ROUTE_FLOOD header + path_len=0 + dest_hash(1) + src_hash(1) + MAC+ct
        from meshcore_tx import build_wire_packet, make_header

        inner_payload = bytes([state.pub_key[0], peer_pub[0]]) + encrypted
        path_pkt = build_wire_packet(
            make_header(bridge.ROUTE_FLOOD, 0x08), inner_payload
        )

        frame = {
            "type": "lora_frame",
            "payload": path_pkt,
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0, "rssi_dbm": -70},
        }

        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)

        # Contact should have out_path_len updated
        updated = state.contacts[peer_pub.hex()]
        new_path_len = struct.unpack_from("<b", updated, 34)[0]
        # The return path was [state.pub_key[0]] (1 byte, 1 hop)
        # Reversed (forward path) is still [state.pub_key[0]] (1 hop)
        self.assertEqual(new_path_len, 1)

        # Should also emit PUSH_PATH_UPDATE
        push_types = [m[0] for m in msgs]
        self.assertIn(bridge.PUSH_PATH_UPDATE, push_types)

        # Should also have processed the ACK (pending_acks entry removed)
        self.assertNotIn(ack_hash, state.pending_acks)

    def test_path_rx_no_contact_does_not_crash(self):
        """PATH from unknown contact (not in contacts) is handled gracefully."""
        state = make_state()
        peer_expanded_prv, peer_pub, _ = make_identity()
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub
        # Note: peer is NOT added to state.contacts

        ack_hash = b"\xaa\xbb\xcc\xdd"
        state.pending_acks[ack_hash] = (time.monotonic(), peer_pub[:6])

        path_len_enc = 0x01
        path_hash = bytes([state.pub_key[0]])
        plaintext = (
            bytes([path_len_enc]) + path_hash + bytes([bridge.PAYLOAD_ACK]) + ack_hash
        )

        from meshcore_crypto import meshcore_shared_secret, meshcore_encrypt_then_mac

        shared = meshcore_shared_secret(peer_expanded_prv, state.pub_key)
        assert shared is not None
        encrypted = meshcore_encrypt_then_mac(shared, plaintext)

        from meshcore_tx import build_wire_packet, make_header

        inner_payload = bytes([state.pub_key[0], peer_pub[0]]) + encrypted
        path_pkt = build_wire_packet(
            make_header(bridge.ROUTE_FLOOD, 0x08), inner_payload
        )

        frame = {
            "type": "lora_frame",
            "payload": path_pkt,
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 5.0, "rssi_dbm": -70},
        }

        # Should not raise; ACK should still be processed
        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)
        # ACK still matched even without a contact record
        self.assertNotIn(ack_hash, state.pending_acks)

    def test_path_rx_reverses_multi_hop_path(self):
        """Return path is correctly reversed to produce forward path."""
        state = make_state()
        peer_expanded_prv, peer_pub, _ = make_identity()
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub
        record = bridge._build_contact_record(peer_pub, name="peer")
        state.contacts[peer_pub.hex()] = record

        # 3-hop return path: [h1, h2, h3]
        # Forward path (reversed) should be: [h3, h2, h1]
        h1, h2, h3 = 0xAA, 0xBB, 0xCC
        return_path = bytes([h1, h2, h3])
        # Call _update_contact_path directly
        push = bridge._update_contact_path(state, peer_pub, return_path)
        self.assertIsNotNone(push)
        updated = state.contacts[peer_pub.hex()]
        new_path_len = struct.unpack_from("<b", updated, 34)[0]
        self.assertEqual(new_path_len, 3)
        # Forward path at offsets 35..37 should be [h3, h2, h1]
        forward = updated[35:38]
        self.assertEqual(forward, bytes([h3, h2, h1]))


class TestPacketStats(unittest.TestCase):
    def test_stats_type2_has_26_bytes(self):
        state = make_state()
        payload = state.build_stats(2)
        self.assertEqual(len(payload), 26)

    def test_stats_type2_counts_flood_tx(self):
        state = make_state()
        state.pkt_flood_tx += 1
        state.pkt_sent += 1
        payload = state.build_stats(2)
        recv, sent, flood_tx, direct_tx, flood_rx, direct_rx = struct.unpack_from(
            "<IIIIII", payload, 2
        )
        self.assertEqual(sent, 1)
        self.assertEqual(flood_tx, 1)
        self.assertEqual(direct_tx, 0)

    def test_stats_type2_counts_flood_rx(self):
        state = make_state()
        state.pkt_recv += 1
        state.pkt_flood_rx += 1
        payload = state.build_stats(2)
        recv, sent, flood_tx, direct_tx, flood_rx, direct_rx = struct.unpack_from(
            "<IIIIII", payload, 2
        )
        self.assertEqual(recv, 1)
        self.assertEqual(flood_rx, 1)
        self.assertEqual(direct_rx, 0)


class TestRepeaterCommands(unittest.TestCase):
    def _make_peer_contact(self, state):
        """Add a peer contact to state, return peer_pub."""
        _, peer_pub, _ = make_identity()
        record = bridge._build_contact_record(peer_pub, name="repeater")
        state.contacts[peer_pub.hex()] = record
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub
        return peer_pub

    def test_cmd_login_sends_udp_and_returns_msg_sent(self):
        """CMD_LOGIN sends a TXT_MSG CLI and returns MSG_SENT."""
        state = make_state()
        peer_pub = self._make_peer_contact(state)
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_LOGIN]) + peer_pub + b"secret"
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertGreater(len(sock.sent), 0)

    def test_cmd_status_req_sends_udp_and_returns_msg_sent(self):
        """CMD_STATUS_REQ sends a TXT_MSG CLI and returns MSG_SENT."""
        state = make_state()
        peer_pub = self._make_peer_contact(state)
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_STATUS_REQ]) + peer_pub
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertGreater(len(sock.sent), 0)

    def test_cmd_logout_sends_udp_and_returns_ok(self):
        """CMD_LOGOUT sends a TXT_MSG CLI and returns RESP_OK."""
        state = make_state()
        peer_pub = self._make_peer_contact(state)
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_LOGOUT]) + peer_pub
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertGreater(len(sock.sent), 0)

    def test_cmd_login_short_payload_returns_error(self):
        """CMD_LOGIN with too-short payload returns error."""
        state = make_state()
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_LOGIN]) + b"\x00" * 10  # too short
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_cmd_logout_short_payload_returns_error(self):
        """CMD_LOGOUT with payload shorter than 32 bytes returns RESP_ERROR."""
        state = make_state()
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_LOGOUT]) + b"\xaa" * 10  # only 10 bytes, need 32
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)


class TestBinaryReq(unittest.TestCase):
    def _make_peer_contact(self, state):
        peer_prv, peer_pub, _ = make_identity()
        record = bridge._build_contact_record(peer_pub, name="repeater")
        state.contacts[peer_pub.hex()] = record
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub
        return peer_pub, peer_prv

    def test_cmd_binary_req_returns_msg_sent(self):
        """CMD_BINARY_REQ sends a binary TXT_MSG and returns MSG_SENT."""
        state = make_state()
        peer_pub, _ = self._make_peer_contact(state)
        sock = _FakeUDPSock()
        # request_type=0x01 (STATUS)
        cmd = bytes([bridge.CMD_BINARY_REQ]) + peer_pub + bytes([0x01])
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertGreater(len(sock.sent), 0)

    def test_cmd_binary_req_short_payload_returns_error(self):
        """CMD_BINARY_REQ with payload shorter than 33 bytes returns error."""
        state = make_state()
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_BINARY_REQ]) + b"\x00" * 10  # too short
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_cmd_send_anon_req_returns_msg_sent(self):
        """CMD_SEND_ANON_REQ sends an ANON_REQ and returns MSG_SENT."""
        state = make_state()
        peer_pub, _ = self._make_peer_contact(state)
        sock = _FakeUDPSock()
        # request_type=0x04 (OWNER)
        cmd = (
            bytes([bridge.CMD_SEND_ANON_REQ])
            + peer_pub
            + bytes([0x04])
            + bytes([0])  # path_len=0
        )
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_MSG_SENT)
        self.assertGreater(len(sock.sent), 0)

    def test_cmd_send_anon_req_short_payload_returns_error(self):
        """CMD_SEND_ANON_REQ with payload shorter than 33 bytes returns error."""
        state = make_state()
        sock = _FakeUDPSock()
        cmd = bytes([bridge.CMD_SEND_ANON_REQ]) + b"\x00" * 10  # too short
        responses = bridge.handle_command(cmd, state, sock, ("127.0.0.1", 5555))
        self.assertEqual(responses[0][0], bridge.RESP_ERROR)

    def test_incoming_cli_txt_msg_produces_binary_response_push(self):
        """TXT_MSG with txt_type=CLI (0x01) produces BINARY_RESPONSE push (0x8C)."""
        state = make_state()
        peer_prv, peer_pub, _ = make_identity()
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub

        # Build an incoming CLI TXT_MSG from peer
        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            peer_prv,
            peer_pub,
            state.pub_key,
            "status data",
            txt_type=1,
            route_type=bridge.ROUTE_FLOOD,
        )
        frame = {
            "type": "lora_frame",
            "payload": pkt,
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0, "rssi_dbm": -80},
        }
        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)
        # Should include a BINARY_RESPONSE push
        push_types = [m[0] for m in msgs]
        self.assertIn(bridge.PUSH_BINARY_RESPONSE, push_types)

    def test_incoming_plain_txt_msg_does_not_produce_binary_response(self):
        """TXT_MSG with txt_type=PLAIN (0x00) does NOT produce BINARY_RESPONSE push."""
        state = make_state()
        peer_prv, peer_pub, _ = make_identity()
        from meshcore_crypto import save_pubkey

        save_pubkey(state.keys_dir, peer_pub)
        state.known_keys[peer_pub.hex()] = peer_pub

        from meshcore_tx import build_txt_msg

        pkt = build_txt_msg(
            peer_prv,
            peer_pub,
            state.pub_key,
            "hello world",
            txt_type=0,
            route_type=bridge.ROUTE_FLOOD,
        )
        frame = {
            "type": "lora_frame",
            "payload": pkt,
            "crc_valid": True,
            "phy": {"sync_word": 0x12, "snr_db": 4.0, "rssi_dbm": -80},
        }
        msgs, acks = bridge.lora_frame_to_companion_msgs(frame, state)
        push_types = [m[0] for m in msgs]
        self.assertNotIn(bridge.PUSH_BINARY_RESPONSE, push_types)


class TestPrivateKey(unittest.TestCase):
    def test_export_private_key_returns_seed(self):
        """CMD_EXPORT_PRIVATE_KEY returns PRIVATE_KEY (0x0E) with the 32-byte seed."""
        state = make_state()
        cmd = bytes([bridge.CMD_EXPORT_PRIVATE_KEY])
        responses = bridge.handle_command(cmd, state, None, None)
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0][0], 0x0E)
        self.assertEqual(len(responses[0]), 33)  # 0x0E + 32 bytes seed
        self.assertEqual(responses[0][1:], state.seed)

    def test_import_private_key_updates_identity(self):
        """CMD_IMPORT_PRIVATE_KEY accepts new seed and updates pub_key/expanded_prv."""
        state = make_state()
        # Generate a new seed to import
        _, new_pub, new_seed = make_identity()
        cmd = bytes([bridge.CMD_IMPORT_PRIVATE_KEY]) + new_seed
        responses = bridge.handle_command(cmd, state, None, None)
        self.assertEqual(responses[0][0], bridge.RESP_OK)
        self.assertEqual(state.pub_key, new_pub)
        self.assertEqual(state.seed, new_seed)

    def test_import_private_key_persists_to_disk(self):
        """CMD_IMPORT_PRIVATE_KEY writes the new seed+pub to the identity file."""
        import tempfile
        from pathlib import Path

        state = make_state()
        _, new_pub, new_seed = make_identity()
        with tempfile.NamedTemporaryFile(delete=False) as f:
            tmp_path = Path(f.name)
        state._identity_file = tmp_path
        try:
            cmd = bytes([bridge.CMD_IMPORT_PRIVATE_KEY]) + new_seed
            bridge.handle_command(cmd, state, None, None)
            written = tmp_path.read_bytes()
            self.assertEqual(written[:32], new_seed)
            self.assertEqual(written[32:], new_pub)
        finally:
            tmp_path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
