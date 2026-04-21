#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
Integration tests for meshcore_bridge.py -- subprocess + raw TCP.

Spawns the bridge as a subprocess, connects via raw TCP using the companion
protocol, and verifies end-to-end behavior. A mock UDP server stands in for
lora_trx.

Run:  python3 -m unittest scripts/tests/test_meshcore_bridge_integration.py -v
"""

from __future__ import annotations

import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import cbor2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))

from conftest import (
    FRAME_TX,
    FRAME_RX,
    CMD_APP_START,
    CMD_GET_CONTACTS,
    CMD_DEVICE_QUERY,
    CMD_GET_CHANNEL,
    CMD_EXPORT_CONTACT,
    CMD_SYNC_NEXT_MESSAGE,
    CMD_SET_ADVERT_NAME,
    RESP_OK,
    RESP_ERROR,
    RESP_CONTACT_START,
    RESP_CONTACT,
    RESP_CONTACT_END,
    RESP_SELF_INFO,
    RESP_NO_MORE_MSGS,
    RESP_CONTACT_URI,
    RESP_DEVICE_INFO,
    RESP_CHANNEL_INFO,
    PUSH_MESSAGES_WAITING,
    PUSH_NEW_ADVERT,
    RESP_CONTACT_MSG_RECV_V3,
    frame_encode_tx as _frame_encode_tx,
    recv_frames as _recv_frames,
    recv_until as _recv_until,
    free_port as _free_port,
)


class BridgeFixture:
    """Manages a bridge subprocess + mock UDP server."""

    def __init__(self) -> None:
        self.tcp_port = _free_port()
        self.udp_port = _free_port()
        self.tmpdir = tempfile.mkdtemp()
        self.identity_file = Path(self.tmpdir) / "identity.bin"
        self.proc: subprocess.Popen | None = None
        self.udp_sock: socket.socket | None = None

    def start(
        self,
        name: str = "test-bridge",
        extra_args: list[str] | None = None,
    ) -> None:
        """Start the bridge subprocess and mock UDP server."""
        # Start mock UDP server first (so bridge can subscribe)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", self.udp_port))
        self.udp_sock.settimeout(3.0)

        # Start bridge subprocess
        bridge_script = (
            Path(__file__).resolve().parent.parent / "apps" / "meshcore_bridge.py"
        )
        cmd = [
            sys.executable,
            str(bridge_script),
            "--port",
            str(self.tcp_port),
            "--connect",
            f"127.0.0.1:{self.udp_port}",
            "--identity",
            str(self.identity_file),
            "--name",
            name,
        ]
        if extra_args:
            cmd.extend(extra_args)
        self.proc = subprocess.Popen(
            cmd,
            cwd=str(Path(__file__).resolve().parent.parent.parent),
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )

        # Wait for bridge to start listening
        self._wait_for_tcp_ready()

    def _wait_for_tcp_ready(self, timeout: float = 5.0) -> None:
        """Poll until TCP port is accepting connections."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection(("127.0.0.1", self.tcp_port), timeout=0.1)
                s.close()
                return
            except ConnectionRefusedError, OSError:
                time.sleep(0.05)
        raise RuntimeError(
            f"Bridge did not start within {timeout}s on port {self.tcp_port}"
        )

    def connect(self) -> socket.socket:
        """Create a TCP connection to the bridge."""
        sock = socket.create_connection(("127.0.0.1", self.tcp_port), timeout=3.0)
        sock.settimeout(3.0)
        return sock

    def recv_udp(self, timeout: float = 2.0) -> bytes:
        """Receive a UDP datagram from the bridge."""
        assert self.udp_sock is not None
        self.udp_sock.settimeout(timeout)
        data, _addr = self.udp_sock.recvfrom(65536)
        return data

    def send_udp_to_bridge(self, data: bytes) -> None:
        """Send a UDP datagram to the bridge (as if from lora_trx).

        We need the bridge's UDP source address, which we get from the
        subscribe message it sends on startup.
        """
        assert self.udp_sock is not None
        # The bridge sent subscribe from its ephemeral port; we recorded
        # the address when receiving the subscribe message.
        self.udp_sock.sendto(data, self._bridge_udp_addr)

    def drain_subscribe(self) -> dict:
        """Read and parse the initial subscribe message from the bridge."""
        data = self.recv_udp(timeout=3.0)
        msg = cbor2.loads(data)
        # Record bridge's UDP source address for sending RX frames back
        # We need to receive again to get the address
        return msg

    def drain_subscribe_with_addr(self) -> tuple[dict, tuple[str, int]]:
        """Read subscribe message and capture bridge's UDP source address."""
        assert self.udp_sock is not None
        self.udp_sock.settimeout(3.0)
        data, addr = self.udp_sock.recvfrom(65536)
        msg = cbor2.loads(data)
        self._bridge_udp_addr = addr
        return msg, addr

    def stop(self) -> None:
        """Stop the bridge subprocess and clean up."""
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            if self.proc.stdout:
                self.proc.stdout.close()
            if self.proc.stderr:
                self.proc.stderr.close()
            self.proc = None
        if self.udp_sock is not None:
            self.udp_sock.close()
            self.udp_sock = None


# ---- Integration tests ----


class TestBridgeIntegration(unittest.TestCase):
    """End-to-end tests: spawn bridge subprocess, connect via TCP."""

    fixture: BridgeFixture

    @classmethod
    def setUpClass(cls) -> None:
        cls.fixture = BridgeFixture()
        cls.fixture.start(name="int-test")
        # Drain the initial subscribe message
        cls.subscribe_msg, _ = cls.fixture.drain_subscribe_with_addr()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.fixture.stop()

    def _connect_and_app_start(self) -> tuple[socket.socket, list[bytes]]:
        """Connect, send APP_START, return (socket, response frames).

        APP_START returns SELF_INFO + PUSH_NEW_ADVERT (2 frames).
        """
        sock = self.fixture.connect()
        sock.sendall(_frame_encode_tx(bytes([CMD_APP_START])))
        # APP_START returns SELF_INFO + PUSH_NEW_ADVERT
        frames = _recv_until(sock, 2, timeout=3.0)
        return sock, frames

    def test_00_subscribe_message(self):
        """Bridge sends a CBOR subscribe message on startup."""
        msg = self.subscribe_msg
        self.assertEqual(msg.get("type"), "subscribe")
        self.assertIn(0x12, msg.get("sync_word", []))

    def test_01_app_start_returns_self_info(self):
        """APP_START returns SELF_INFO + PUSH_NEW_ADVERT."""
        sock, frames = self._connect_and_app_start()
        try:
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[0][0], RESP_SELF_INFO)
            # SELF_INFO contains pubkey (32 bytes) after response code + firmware info
            self.assertGreater(len(frames[0]), 32)
            # Second frame is PUSH_NEW_ADVERT (our own ADVERT)
            self.assertEqual(frames[1][0], PUSH_NEW_ADVERT)
        finally:
            sock.close()

    def test_02_device_query(self):
        """DEVICE_QUERY returns DEVICE_INFO."""
        sock, _ = self._connect_and_app_start()
        try:
            sock.sendall(_frame_encode_tx(bytes([CMD_DEVICE_QUERY])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_DEVICE_INFO)
        finally:
            sock.close()

    def test_03_get_contacts_returns_self(self):
        """GET_CONTACTS returns at least self contact (CONTACT_START + CONTACT... + CONTACT_END)."""
        sock, _ = self._connect_and_app_start()
        try:
            sock.sendall(_frame_encode_tx(bytes([CMD_GET_CONTACTS])))
            # Receive frames until we see CONTACT_END (bridge may have persisted contacts)
            frames = _recv_until(sock, 20, timeout=3.0)
            self.assertGreaterEqual(len(frames), 3)
            self.assertEqual(frames[0][0], RESP_CONTACT_START)
            # At least one CONTACT (self)
            contact_frames = [f for f in frames if f[0] == RESP_CONTACT]
            self.assertGreaterEqual(len(contact_frames), 1)
            # Last frame should be CONTACT_END
            end_frames = [f for f in frames if f[0] == RESP_CONTACT_END]
            self.assertEqual(len(end_frames), 1)
        finally:
            sock.close()

    def test_04_get_channel_loop(self):
        """GET_CHANNEL returns CHANNEL_INFO for idx 0-7, ERROR for idx 8."""
        sock, _ = self._connect_and_app_start()
        try:
            # Request channels 0 through 8
            for idx in range(9):
                sock.sendall(_frame_encode_tx(bytes([CMD_GET_CHANNEL, idx])))

            frames = _recv_until(sock, 9, timeout=3.0)
            self.assertEqual(len(frames), 9)
            # Channels 0-7 return CHANNEL_INFO
            for i in range(8):
                self.assertEqual(
                    frames[i][0],
                    RESP_CHANNEL_INFO,
                    f"channel {i} should be CHANNEL_INFO",
                )
            # Channel 8 returns ERROR
            self.assertEqual(frames[8][0], RESP_ERROR)
        finally:
            sock.close()

    def test_05_export_self_contact(self):
        """EXPORT_CONTACT for self returns CONTACT_URI with raw ADVERT bytes."""
        sock, self_info = self._connect_and_app_start()
        try:
            # Export our own contact (index 0)
            sock.sendall(_frame_encode_tx(bytes([CMD_EXPORT_CONTACT, 0x00])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_CONTACT_URI)
            # Payload after response code should be raw ADVERT bytes
            raw_advert = frames[0][1:]
            self.assertGreater(len(raw_advert), 32)
        finally:
            sock.close()

    def test_06_sync_next_message_empty(self):
        """SYNC_NEXT_MESSAGE with no queued messages returns NO_MORE_MSGS."""
        sock, _ = self._connect_and_app_start()
        try:
            sock.sendall(_frame_encode_tx(bytes([CMD_SYNC_NEXT_MESSAGE])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_NO_MORE_MSGS)
        finally:
            sock.close()

    def test_07_set_advert_name(self):
        """SET_ADVERT_NAME returns OK."""
        sock, _ = self._connect_and_app_start()
        try:
            name = b"NewTestName"
            sock.sendall(_frame_encode_tx(bytes([CMD_SET_ADVERT_NAME]) + name))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_OK)
        finally:
            sock.close()

    def test_08_full_session_flow(self):
        """Simulate a full meshcore-cli session: APP_START -> DEVICE_QUERY -> GET_CONTACTS -> GET_CHANNEL loop."""
        sock, self_info = self._connect_and_app_start()
        try:
            # Verify SELF_INFO
            self.assertEqual(self_info[0][0], RESP_SELF_INFO)

            # DEVICE_QUERY
            sock.sendall(_frame_encode_tx(bytes([CMD_DEVICE_QUERY])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(frames[0][0], RESP_DEVICE_INFO)

            # GET_CONTACTS (may have persisted contacts beyond just self)
            sock.sendall(_frame_encode_tx(bytes([CMD_GET_CONTACTS])))
            frames = _recv_until(sock, 20, timeout=3.0)
            self.assertEqual(frames[0][0], RESP_CONTACT_START)
            end_frames = [f for f in frames if f[0] == RESP_CONTACT_END]
            self.assertEqual(len(end_frames), 1)

            # GET_CHANNEL loop (0..8)
            for idx in range(9):
                sock.sendall(_frame_encode_tx(bytes([CMD_GET_CHANNEL, idx])))
            frames = _recv_until(sock, 9, timeout=3.0)
            self.assertEqual(len(frames), 9)
            self.assertEqual(frames[8][0], RESP_ERROR)
        finally:
            sock.close()

    def test_09_rx_frame_injection(self):
        """Inject a CBOR RX frame via UDP, verify MESSAGES_WAITING push."""
        sock, _ = self._connect_and_app_start()
        try:
            # Give the bridge time to process TCP client registration
            time.sleep(0.3)

            # Build a fake lora_frame CBOR message with MeshCore ADVERT payload.
            # ADVERT: route=1 (T_SINGLE), ptype=4
            hdr = (0x04 << 2) | 0x01  # ADVERT + T_SINGLE
            path_len = 0
            pubkey = b"\xaa" * 32
            timestamp = b"\x00\x00\x00\x00"
            signature = b"\x00" * 64
            app_data = bytes([0x81]) + b"TestNode"  # has_name + name
            fake_payload = (
                bytes([hdr, path_len]) + pubkey + timestamp + signature + app_data
            )
            cbor_frame = cbor2.dumps(
                {
                    "type": "lora_frame",
                    "seq": 1,
                    "phy": {
                        "sync_word": 0x12,
                        "snr_db": 5.0,
                        "crc_valid": True,
                    },
                    "payload": fake_payload,
                    "crc_valid": True,
                }
            )
            self.fixture.send_udp_to_bridge(cbor_frame)

            # Should receive PUSH_MESSAGES_WAITING
            frames = _recv_until(sock, 1, timeout=3.0)
            self.assertGreaterEqual(len(frames), 1)
            self.assertEqual(frames[0][0], PUSH_MESSAGES_WAITING)

            # SYNC_NEXT_MESSAGE should return the ADVERT as PUSH_NEW_ADVERT
            sock.sendall(_frame_encode_tx(bytes([CMD_SYNC_NEXT_MESSAGE])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            # ADVERT frames are queued as PUSH_NEW_ADVERT (0x8A)
            self.assertEqual(frames[0][0], PUSH_NEW_ADVERT)

            # Next sync should be empty
            sock.sendall(_frame_encode_tx(bytes([CMD_SYNC_NEXT_MESSAGE])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_NO_MORE_MSGS)
        finally:
            sock.close()

    def test_10_reconnect_resets_queue(self):
        """Disconnecting and reconnecting resets the message queue."""
        # First connection
        sock1, _ = self._connect_and_app_start()
        sock1.close()
        time.sleep(0.3)

        # Second connection — APP_START returns SELF_INFO + PUSH_NEW_ADVERT
        sock2, app_frames = self._connect_and_app_start()
        try:
            self.assertEqual(app_frames[0][0], RESP_SELF_INFO)

            # Drain any additional push messages (e.g. from UDP keepalive)
            time.sleep(0.2)
            _ = _recv_frames(sock2, timeout=0.3)

            # Queue should be empty — SYNC_NEXT_MESSAGE returns NO_MORE_MSGS
            sock2.sendall(_frame_encode_tx(bytes([CMD_SYNC_NEXT_MESSAGE])))
            frames = _recv_until(sock2, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0][0], RESP_NO_MORE_MSGS)
        finally:
            sock2.close()


class TestBridgeRxDecryptionIntegration(unittest.TestCase):
    """End-to-end test: inject an encrypted TXT_MSG, verify decrypted delivery."""

    fixture: BridgeFixture
    sender_prv: bytes
    sender_pub: bytes
    bridge_pub: bytes

    @classmethod
    def setUpClass(cls) -> None:
        from meshcore_crypto import load_or_create_identity, save_pubkey

        cls.fixture = BridgeFixture()

        # Pre-create sender identity and save key BEFORE starting bridge
        sender_id_path = Path(cls.fixture.tmpdir) / "sender.bin"
        cls.sender_prv, cls.sender_pub, _ = load_or_create_identity(sender_id_path)

        # Save sender key to a keys dir that the bridge will use
        keys_dir = Path(cls.fixture.tmpdir) / "keys"
        keys_dir.mkdir(exist_ok=True)
        save_pubkey(keys_dir, cls.sender_pub)
        cls._keys_dir = keys_dir

        # Start bridge with custom --keys-dir
        cls.fixture.start(
            name="decrypt-test",
            extra_args=["--keys-dir", str(keys_dir)],
        )
        _, _ = cls.fixture.drain_subscribe_with_addr()

        # Load bridge identity to encrypt TO it
        bridge_prv, cls.bridge_pub, _ = load_or_create_identity(
            cls.fixture.identity_file
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.fixture.stop()

    def test_encrypted_txt_msg_delivered_decrypted(self):
        """Inject encrypted TXT_MSG via UDP, verify decrypted text at companion."""
        from meshcore_tx import build_txt_msg

        # Connect companion client
        sock, _ = self._connect_and_app_start()
        try:
            time.sleep(0.3)

            # Build encrypted TXT_MSG from sender to bridge
            packet = build_txt_msg(
                self.sender_prv,
                self.sender_pub,
                self.bridge_pub,
                "hello from test",
            )

            # Inject as CBOR lora_frame
            cbor_frame = cbor2.dumps(
                {
                    "type": "lora_frame",
                    "seq": 42,
                    "phy": {
                        "sync_word": 0x12,
                        "snr_db": 7.0,
                        "crc_valid": True,
                    },
                    "payload": packet,
                    "crc_valid": True,
                }
            )
            self.fixture.send_udp_to_bridge(cbor_frame)

            # Should receive PUSH_MESSAGES_WAITING
            frames = _recv_until(sock, 1, timeout=3.0)
            self.assertGreaterEqual(len(frames), 1)
            self.assertEqual(frames[0][0], PUSH_MESSAGES_WAITING)

            # SYNC_NEXT_MESSAGE should return decrypted CONTACT_MSG_RECV_V3
            sock.sendall(_frame_encode_tx(bytes([CMD_SYNC_NEXT_MESSAGE])))
            frames = _recv_until(sock, 1, timeout=2.0)
            self.assertEqual(len(frames), 1)
            msg = frames[0]
            self.assertEqual(msg[0], RESP_CONTACT_MSG_RECV_V3)

            # Extract text: V3 header is 16 bytes
            text = msg[16:].decode("utf-8", errors="replace")
            self.assertEqual(text, "hello from test")
        finally:
            sock.close()

    def _connect_and_app_start(self) -> tuple[socket.socket, list[bytes]]:
        sock = self.fixture.connect()
        sock.sendall(_frame_encode_tx(bytes([CMD_APP_START])))
        frames = _recv_until(sock, 2, timeout=3.0)
        return sock, frames


if __name__ == "__main__":
    unittest.main()
