#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
meshcore_bridge.py -- MeshCore companion protocol bridge.

TCP server speaking the MeshCore companion binary protocol, bridging to
lora_trx via CBOR/UDP. Compatible with meshcore-cli and other MeshCore
companion apps.

Usage:
    meshcore_bridge.py                    # default: TCP 7834, UDP 127.0.0.1:5555
    meshcore_bridge.py --port 5000 --udp 127.0.0.1:5555
    meshcore_bridge.py --config apps/config.toml

Architecture:
    meshcore-cli ──TCP:7834──► meshcore_bridge ──UDP:5555──► lora_trx
                  companion                      CBOR          (SDR)
                  protocol

Note: port 5000 is occupied by macOS AirPlay/Control Center. Use 7834 (default)
or configure a different port in [meshcore] port = ... in config.toml.

The bridge:
  - Accepts one TCP client at a time (matches real MeshCore firmware)
  - Responds to companion protocol commands (APP_START, GET_CONTACTS, etc.)
  - Translates CMD_SEND_TXT_MSG to CBOR lora_tx requests via UDP
  - Subscribes to lora_trx for RX frames, queues them for the client
  - Pushes MESSAGES_WAITING when new frames arrive

Dependencies: cbor2, pynacl, pycryptodome
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import logging
import selectors
import socket
import struct
import subprocess
import time
from pathlib import Path
from typing import Any

import cbor2

from lora_common import (
    config_agg_listen,
    config_region_scope,
    config_udp_host,
    config_udp_port,
    load_config,
    parse_host_port,
    sanitize_text,
    setup_logging,
    utf8_truncate,
)
from meshcore_crypto import (
    DEFAULT_IDENTITY_FILE,
    DEFAULT_KEYS_DIR,
    DEFAULT_CHANNELS_DIR,
    load_or_create_identity,
    load_known_keys,
    save_pubkey,
    load_channels,
    save_channel,
    GroupChannel,
    meshcore_expanded_key,
    meshcore_shared_secret,
    meshcore_encrypt_then_mac,
    meshcore_mac_then_decrypt,
    try_decrypt_txt_msg,
    try_decrypt_anon_req,
    try_decrypt_grp_txt,
    extract_advert_pubkey,
    extract_advert_name,
    parse_meshcore_header,
    decode_path_len,
    build_grp_txt,
    compute_ack_hash,
    # Protocol constants (canonical source)
    PUB_KEY_SIZE,
    CIPHER_MAC_SIZE,
    CIPHER_BLOCK_SIZE,
    PATH_HASH_SIZE,
    ROUTE_MASK,
    PTYPE_SHIFT,
    PTYPE_MASK,
    PAYLOAD_TXT,
    PAYLOAD_ACK,
    PAYLOAD_ADVERT,
    PAYLOAD_RESP,
    PAYLOAD_GRP_TXT,
    PAYLOAD_ANON_REQ,
    PAYLOAD_PATH,
    PAYLOAD_CTRL,
    ROUTE_DIRECT,
    ROUTE_FLOOD,
    ROUTE_T_DIRECT,
    ROUTE_T_FLOOD,
    ADVERT_HAS_LOCATION,
    ADVERT_HAS_NAME,
)

log = logging.getLogger("gr4.bridge")


def _get_git_rev() -> str:
    """Return the short git SHA of the gr4-lora repo, or 'dev' if unavailable."""
    try:
        here = Path(__file__).resolve().parent
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=here,
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except OSError, subprocess.TimeoutExpired:
        pass
    return "dev"


# ---- Companion protocol constants ----

BRIDGE_PORT = (
    7834  # MeshCore companion WiFi port (avoids macOS AirPlay/Control Center on 5000)
)

# Framing bytes
FRAME_TX = 0x3C  # client -> device ('<')
FRAME_RX = 0x3E  # device -> client ('>')
MAX_FRAME = 300  # max payload size

# Command codes (client -> bridge)
CMD_APP_START = 0x01
CMD_SEND_TXT_MSG = 0x02
CMD_SEND_CHAN_TXT_MSG = 0x03
CMD_GET_CONTACTS = 0x04
CMD_GET_DEVICE_TIME = 0x05
CMD_SET_DEVICE_TIME = 0x06
CMD_SEND_SELF_ADVERT = 0x07
CMD_SET_ADVERT_NAME = 0x08
CMD_ADD_UPDATE_CONTACT = 0x09
CMD_SYNC_NEXT_MESSAGE = 0x0A
CMD_SET_RADIO_PARAMS = 0x0B
CMD_SET_RADIO_TX_POWER = 0x0C
CMD_RESET_PATH = 0x0D
CMD_SET_ADVERT_LATLON = 0x0E
CMD_REMOVE_CONTACT = 0x0F
CMD_SHARE_CONTACT = 0x10
CMD_EXPORT_CONTACT = 0x11
CMD_IMPORT_CONTACT = 0x12
CMD_REBOOT = 0x13
CMD_GET_BATT_AND_STORAGE = 0x14
CMD_SET_TUNING_PARAMS = 0x15
CMD_DEVICE_QUERY = 0x16
CMD_LOGIN = 0x1A
CMD_STATUS_REQ = 0x1B
CMD_GET_CHANNEL = 0x1F
CMD_SET_CHANNEL = 0x20
CMD_TRACE = 0x24
CMD_SET_OTHER_PARAMS = 0x26
CMD_GET_CUSTOM_VARS = 0x28
CMD_SET_CUSTOM_VAR = 0x29
CMD_PATH_DISCOVERY = 0x34
CMD_GET_STATS = 0x38
CMD_SEND_CONTROL_DATA = 0x37
CMD_SET_FLOOD_SCOPE = 0x36
CMD_SET_AUTOADD_CONFIG = 0x3A
CMD_GET_AUTOADD_CONFIG = 0x3B
CMD_GET_ALLOWED_REPEAT_FREQ = 0x3C
CMD_SET_PATH_HASH_MODE = 0x3D

# Response codes (bridge -> client)
RESP_OK = 0x00
RESP_ERROR = 0x01
RESP_CONTACT_START = 0x02
RESP_CONTACT = 0x03
RESP_CONTACT_END = 0x04
RESP_SELF_INFO = 0x05
RESP_MSG_SENT = 0x06
RESP_CONTACT_MSG_RECV = 0x07
RESP_CHANNEL_MSG_RECV = 0x08
RESP_CURRENT_TIME = 0x09
RESP_NO_MORE_MSGS = 0x0A
RESP_CONTACT_URI = 0x0B
RESP_BATTERY = 0x0C
RESP_DEVICE_INFO = 0x0D
RESP_CONTACT_MSG_RECV_V3 = 0x10
RESP_CHANNEL_MSG_RECV_V3 = 0x11
RESP_CHANNEL_INFO = 0x12
RESP_CUSTOM_VARS = (
    0x15  # GET_CUSTOM_VARS response (empty payload = no hardware sensors)
)
RESP_STATS = 0x18
RESP_CODE_AUTOADD_CONFIG = 0x19
RESP_ALLOWED_REPEAT_FREQ = 0x1A

# Error codes
ERR_CODE_ILLEGAL_ARG = 6

# Repeat mode: valid frequencies (kHz) for client_repeat = 1
REPEAT_FREQ_RANGES = [(433000, 433000), (869000, 869000), (918000, 918000)]

# Push codes (bridge -> client, unsolicited)
PUSH_ADVERTISEMENT = 0x80
PUSH_PATH_UPDATE = 0x81
PUSH_ACK = 0x82
PUSH_MESSAGES_WAITING = 0x83
PUSH_NEW_ADVERT = 0x8A
PUSH_CONTROL_DATA = 0x8E

# ---- Helpers ----


def _is_valid_repeat_freq(freq_khz: int) -> bool:
    """Return True if freq_khz is in one of the allowed repeat frequency ranges."""
    return any(lo <= freq_khz <= hi for lo, hi in REPEAT_FREQ_RANGES)


def _parse_flood_scope(value: str) -> bytes:
    """Parse a flood_scope config value into a 16-byte transport key.

    Accepts two formats (matching the companion library's set_flood_scope()):
      - 32 hex characters (64 nibbles) → decoded as raw 16-byte key
      - Any other non-empty string → sha256(name)[:16] (human-readable scope name)
      - Empty string or "*" or "0" or "None" → all-zero key (disabled)

    The hash formula for human-readable names is sha256(name) without a "#" prefix,
    matching meshcore_py's set_flood_scope() implementation.  This differs from
    region_scope which uses sha256("#" + name).
    """
    if not value or value in ("0", "None", "*"):
        return b"\x00" * 16
    # Try 32-char hex first (raw key)
    if len(value) == 32:
        try:
            key = bytes.fromhex(value)
            return key  # exactly 16 bytes
        except ValueError:
            pass  # fall through to name hashing
    # Human-readable scope name: sha256(name)[:16]
    return hashlib.sha256(value.encode("utf-8")).digest()[:16]


# ---- Companion frame codec ----


def frame_encode(payload: bytes) -> bytes:
    """Encode a response frame: 0x3E + len_le16 + payload."""
    length = len(payload)
    return bytes([FRAME_RX]) + struct.pack("<H", length) + payload


class FrameDecoder:
    """Streaming decoder for companion protocol frames (0x3C framing)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._expected = 0
        self._header_done = False

    def feed(self, data: bytes) -> list[bytes]:
        """Feed raw TCP bytes, return list of complete payloads."""
        self._buf.extend(data)
        frames: list[bytes] = []
        while True:
            if not self._header_done:
                # Look for start byte
                idx = self._buf.find(bytes([FRAME_TX]))
                if idx < 0:
                    self._buf.clear()
                    break
                if idx > 0:
                    del self._buf[:idx]  # discard garbage before start
                if len(self._buf) < 3:
                    break  # need more header bytes
                self._expected = int.from_bytes(self._buf[1:3], "little")
                if self._expected > MAX_FRAME:
                    # Invalid size, skip this start byte
                    del self._buf[:1]
                    continue
                self._header_done = True
            # Have header, wait for payload
            total = 3 + self._expected
            if len(self._buf) < total:
                break
            payload = bytes(self._buf[3:total])
            del self._buf[:total]
            self._header_done = False
            frames.append(payload)
        return frames


# ---- Bridge state ----


class BridgeState:
    """Holds the bridge's identity and config for building responses."""

    def __init__(
        self,
        pub_key: bytes,
        expanded_prv: bytes,
        seed: bytes,
        name: str,
        freq_mhz: float,
        bw_khz: float,
        sf: int,
        cr: int,
        tx_power: int,
        *,
        lat_e6: int = 0,
        lon_e6: int = 0,
        keys_dir: Path | None = None,
        channels_dir: Path | None = None,
        contacts_dir: Path | None = None,
        region_scope: str = "",
        client_repeat: int = 0,
        path_hash_mode: int = 0,
        send_scope: bytes = b"\x00" * 16,
        autoadd_config: int = 0,
        autoadd_max_hops: int = 0,
        model: str = "lora_trx",
    ) -> None:
        self.pub_key = pub_key
        self.expanded_prv = expanded_prv
        self.seed = seed
        self.name = name
        self.freq_mhz = freq_mhz
        self.bw_khz = bw_khz
        self.sf = sf
        self.cr = cr
        self.tx_power = tx_power
        self.model = model

        # Location (int32 * 1e6 = decimal degrees).  Read from config at startup;
        # updated at runtime by CMD_SET_ADVERT_LATLON (session-only, no persistence).
        self.lat_e6: int = lat_e6
        self.lon_e6: int = lon_e6

        # Real radio stats — updated from received lora_frame and config CBOR messages
        self.noise_floor_dbm: int = -120
        self.last_rssi_dbm: int = -128
        self.last_snr_db: float = 0.0

        # Message queue for inbound RX frames (companion protocol messages)
        self.msg_queue: collections.deque[bytes] = collections.deque(maxlen=256)
        self.msg_seq = 0  # TX sequence counter
        # Packet counters for STATS_TYPE_PACKETS (type 2)
        self.pkt_recv: int = 0
        self.pkt_sent: int = 0
        self.pkt_flood_tx: int = 0
        self.pkt_direct_tx: int = 0
        self.pkt_flood_rx: int = 0
        self.pkt_direct_rx: int = 0

        # Contact store (pubkey_hex -> contact_bytes for companion protocol)
        self.contacts: dict[str, bytes] = {}
        self.contacts_lastmod = 0

        # TX echo filter: hash -> timestamp of recently sent payloads
        self.recent_tx: dict[bytes, float] = {}
        self.recent_tx_ttl = 30.0  # seconds before entries expire

        # Pending ACK tracking: ack_tag(4 bytes) -> (timestamp, dest_prefix)
        self.pending_acks: dict[bytes, tuple[float, bytes]] = {}
        self.pending_ack_ttl = 60.0  # seconds before entries expire

        # RX dedup: payload_hash -> monotonic timestamp (suppress duplicates
        # from dual-channel RX or mesh retransmits within a short window)
        self.recent_rx: dict[bytes, float] = {}
        self.recent_rx_ttl = 5.0  # seconds

        # Key store and channel store directories
        self.keys_dir = keys_dir or DEFAULT_KEYS_DIR
        self.channels_dir = channels_dir or DEFAULT_CHANNELS_DIR
        self.contacts_dir = contacts_dir

        # Load known public keys from disk
        self.known_keys: dict[str, bytes] = load_known_keys(self.keys_dir)

        # Region scope for transport codes (hashtag region, e.g. "de-nord")
        self.region_scope = region_scope
        self.region_key: bytes = b""  # 16-byte transport key
        if region_scope:
            # Key = first 16 bytes of SHA-256("#" + region_name)
            self.region_key = hashlib.sha256(
                ("#" + region_scope).encode("utf-8")
            ).digest()[:16]

        # Load group channels from disk
        self.channels: list[GroupChannel] = load_channels(self.channels_dir)

        # Repeat mode state (in-memory; set by companion commands or config at startup)
        self.client_repeat: int = client_repeat
        self.path_hash_mode: int = path_hash_mode
        # send_scope: 16-byte raw TransportKey; all-zero = null (disabled)
        self.send_scope: bytes = send_scope if len(send_scope) == 16 else b"\x00" * 16
        self.autoadd_config: int = autoadd_config
        self.autoadd_max_hops: int = autoadd_max_hops

        # Startup config mirrors (config.toml values) — re-applied on each APP_START
        # so the bridge's identity is restored after a companion session overwrites it.
        self._startup_name: str = name
        self._startup_lat_e6: int = lat_e6
        self._startup_lon_e6: int = lon_e6
        self._startup_send_scope: bytes = self.send_scope  # already validated above
        self._startup_client_repeat: int = client_repeat
        self._startup_path_hash_mode: int = path_hash_mode
        self._startup_autoadd_config: int = autoadd_config
        self._startup_autoadd_max_hops: int = autoadd_max_hops

        # Other params (CMD_SET_OTHER_PARAMS 0x26); stored in-memory only
        self.manual_add_contacts: int = 0
        self.telemetry_mode: int = 0
        self.adv_loc_policy: int = 0
        self.multi_acks: int = 0

    def apply_startup_config(self) -> None:
        """Re-apply config.toml settings to live state.

        Called on each APP_START so the bridge's identity (name, location,
        flood scope, repeat mode, etc.) is restored to config.toml values
        after a companion session may have overwritten them.
        """
        self.name = self._startup_name
        self.lat_e6 = self._startup_lat_e6
        self.lon_e6 = self._startup_lon_e6
        self.send_scope = self._startup_send_scope
        self.client_repeat = self._startup_client_repeat
        self.path_hash_mode = self._startup_path_hash_mode
        self.autoadd_config = self._startup_autoadd_config
        self.autoadd_max_hops = self._startup_autoadd_max_hops

    def build_self_info(self) -> bytes:
        """Build SELF_INFO (0x05) response payload."""
        # Convert freq/bw to milli-Hz (uint32 LE)
        freq_mhz_int = int(self.freq_mhz * 1e3)  # MHz -> milli-Hz
        bw_mhz_int = int(self.bw_khz)  # kHz -> milli-Hz (field is mHz of MHz)
        # Actually: radio_freq and radio_bw in the companion protocol are in
        # milli-Hz units where the frequency is in MHz*1000 and BW in kHz*1000
        # meshcore_py divides by 1000 to get MHz/kHz:
        #   freq = int.from_bytes(payload[42:46], "little") / 1000  -> MHz
        #   bw = int.from_bytes(payload[46:50], "little") / 1000    -> kHz
        freq_field = int(self.freq_mhz * 1000)  # MHz * 1000
        bw_field = int(self.bw_khz * 1000)  # kHz * 1000

        buf = bytearray()
        buf.append(RESP_SELF_INFO)
        buf.append(0x01)  # adv_type: chat
        buf.append(self.tx_power)  # tx_power
        buf.append(22)  # max_tx_power (dBm, typical for SX1262)
        buf.extend(self.pub_key)  # pubkey (32 bytes)
        buf.extend(struct.pack("<i", self.lat_e6))  # lat (int32 * 1e6)
        buf.extend(struct.pack("<i", self.lon_e6))  # lon (int32 * 1e6)
        buf.append(self.multi_acks)
        buf.append(self.adv_loc_policy)
        buf.append(self.telemetry_mode)
        buf.append(self.manual_add_contacts)
        buf.extend(struct.pack("<I", freq_field))  # radio_freq
        buf.extend(struct.pack("<I", bw_field))  # radio_bw
        buf.append(self.sf)  # radio_sf
        buf.append(self.cr)  # radio_cr
        buf.extend(utf8_truncate(self.name, 32))  # name (bounded)
        return bytes(buf)

    def build_device_info(self) -> bytes:
        """Build DEVICE_INFO (0x0D) response payload (82 bytes total)."""
        git_rev = _get_git_rev()
        # fw_build (12 bytes): binary name, zero-padded
        fw_build = utf8_truncate("lora_trx", 12).ljust(12, b"\x00")
        # model (40 bytes): configurable via [meshcore] model in config.toml
        model_b = utf8_truncate(self.model, 40).ljust(40, b"\x00")
        # version (20 bytes): git short SHA
        version_b = utf8_truncate(git_rev, 20).ljust(20, b"\x00")
        buf = bytearray()
        buf.append(RESP_DEVICE_INFO)
        buf.append(9)  # fw_ver (protocol version 9, matches CMD_DEVICE_QUERY ver=3)
        buf.append(64)  # max_contacts / 2  (128 contacts)
        buf.append(8)  # max_channels
        buf.extend(struct.pack("<I", 0))  # ble_pin (0 = no PIN)
        buf.extend(fw_build)  # fw_build (12 bytes)
        buf.extend(model_b)  # model (40 bytes)
        buf.extend(version_b)  # version (20 bytes): git rev SHA
        buf.append(self.client_repeat)  # byte 80: client_repeat (v9+)
        buf.append(self.path_hash_mode)  # byte 81: path_hash_mode (v10+)
        return bytes(buf)

    def build_battery(self) -> bytes:
        """Build BATTERY (0x0C) response."""
        buf = bytearray()
        buf.append(RESP_BATTERY)
        buf.extend(struct.pack("<H", 4200))  # battery mV (fake: full)
        buf.extend(struct.pack("<I", 0))  # used_kb
        buf.extend(struct.pack("<I", 1024))  # total_kb
        return bytes(buf)

    def build_ok(self, value: int = 0) -> bytes:
        """Build OK (0x00) response."""
        return bytes([RESP_OK]) + struct.pack("<I", value)

    def build_error(self, code: int = 1) -> bytes:
        """Build ERROR (0x01) response."""
        return bytes([RESP_ERROR, code])

    def build_current_time(self) -> bytes:
        """Build CURRENT_TIME (0x09) response."""
        return bytes([RESP_CURRENT_TIME]) + struct.pack("<I", int(time.time()))

    def build_contacts_response(self) -> list[bytes]:
        """Build the full GET_CONTACTS response sequence."""
        frames = []
        count = len(self.contacts)
        frames.append(bytes([RESP_CONTACT_START]) + struct.pack("<I", count))
        for record in self.contacts.values():
            # Each record is 147 bytes; prepend RESP_CONTACT code
            frames.append(bytes([RESP_CONTACT]) + record)
        frames.append(
            bytes([RESP_CONTACT_END]) + struct.pack("<I", self.contacts_lastmod)
        )
        return frames

    def build_no_more_msgs(self) -> bytes:
        """Build NO_MORE_MSGS (0x0A) response."""
        return bytes([RESP_NO_MORE_MSGS])

    def build_msg_sent(
        self, flood: bool = False, ack_hash: bytes | None = None
    ) -> bytes:
        """Build MSG_SENT (0x06) response.

        Byte 1 is routing_type: 1 = sent via flood, 0 = sent via direct route.
        Bytes 2-5 are expected_ack: the 4-byte ACK hash the companion device
        will send back (SHA-256(plaintext + sender_pub)[:4]).  meshcore-cli
        uses this to match against PUSH_SEND_CONFIRMED.
        """
        if ack_hash is not None:
            ack_tag = ack_hash[:4]
        else:
            self.msg_seq += 1
            ack_tag = struct.pack("<I", self.msg_seq)
        timeout = struct.pack("<I", 30000)  # 30s suggested timeout
        routing_type = 1 if flood else 0
        return bytes([RESP_MSG_SENT, routing_type]) + ack_tag + timeout

    def build_channel_info(self, idx: int) -> bytes:
        """Build CHANNEL_INFO (0x12) response.

        Returns real channel data if a channel exists at this index,
        otherwise an empty channel record.
        """
        buf = bytearray()
        buf.append(RESP_CHANNEL_INFO)
        buf.append(idx)
        if idx < len(self.channels):
            ch = self.channels[idx]
            buf.extend(utf8_truncate(ch.name, 32).ljust(32, b"\x00"))
            # Secret: first 16 bytes of the zero-padded secret
            buf.extend(ch.secret[:16])
        else:
            buf.extend(b"\x00" * 32)  # channel name (empty)
            buf.extend(b"\x00" * 16)  # channel secret (empty)
        return bytes(buf)

    def build_stats(self, stats_type: int) -> bytes:
        """Build STATS (0x18) response."""
        buf = bytearray()
        buf.append(RESP_STATS)
        buf.append(stats_type)
        if stats_type == 0:  # core
            buf.extend(struct.pack("<H", 5000))  # battery_mv
            buf.extend(struct.pack("<I", 0))  # uptime
            buf.extend(struct.pack("<H", 0))  # errors
            buf.append(len(self.msg_queue))  # queue_len
        elif stats_type == 1:  # radio
            buf.extend(struct.pack("<h", self.noise_floor_dbm))  # noise_floor (int16)
            buf.extend(struct.pack("<b", self.last_rssi_dbm))  # last_rssi (int8)
            snr_i8 = max(-128, min(127, int(self.last_snr_db * 4)))
            buf.extend(struct.pack("<b", snr_i8))  # last_snr (int8, *4)
            buf.extend(struct.pack("<I", 0))  # tx_air_secs
            buf.extend(struct.pack("<I", 0))  # rx_air_secs
        elif stats_type == 2:  # packets
            buf.extend(
                struct.pack(
                    "<IIIIII",
                    self.pkt_recv,
                    self.pkt_sent,
                    self.pkt_flood_tx,
                    self.pkt_direct_tx,
                    self.pkt_flood_rx,
                    self.pkt_direct_rx,
                )
            )
        return bytes(buf)


# ---- RX frame -> companion message conversion ----


def _handle_ctrl_rx(
    payload: bytes,
    path_len: int,
    state: BridgeState,
) -> list[bytes]:
    """Handle received CTRL packet: push as CONTROL_DATA (0x8E) to companion.

    Wire format after header + [transport_codes] + path_len + path:
      [ctrl_payload...]

    Companion push format (matches meshcore_py reader.py PacketType.CONTROL_DATA):
      [0x8E][snr_byte(1)][rssi_byte(1)][path_len(1)][ctrl_payload...]
    """
    hdr_info = parse_meshcore_header(payload)
    if hdr_info is None:
        return []
    off = hdr_info["off"]
    ctrl_payload = payload[off:]

    snr_byte = max(-128, min(127, int(state.last_snr_db * 4)))
    rssi_byte = max(-128, min(127, state.last_rssi_dbm))

    push = (
        bytes([PUSH_CONTROL_DATA])
        + struct.pack("<b", snr_byte)
        + struct.pack("<b", rssi_byte)
        + bytes([path_len])
        + ctrl_payload
    )
    return [push]


def lora_frame_to_companion_msgs(
    frame: dict[str, Any],
    state: BridgeState,
) -> tuple[list[bytes], list[bytes]]:
    """Convert a CBOR lora_frame to companion protocol messages.

    Returns (companion_msgs, ack_packets):
      - companion_msgs: list of raw response payloads for the TCP client
      - ack_packets: list of MeshCore wire packets to TX as RF ACKs

    Decrypts TXT_MSG, ANON_REQ, and GRP_TXT using the bridge's identity,
    known keys, and group channels — mirroring what real MeshCore firmware
    does before passing messages to the companion app.
    """
    empty: tuple[list[bytes], list[bytes]] = ([], [])

    payload = frame.get("payload", b"")
    if not payload or len(payload) < 2:
        return empty
    if not frame.get("crc_valid", False):
        return empty

    # Only handle MeshCore frames (sync_word 0x12)
    phy = frame.get("phy", {})
    if phy.get("sync_word") != 0x12:
        return empty

    # Parse MeshCore header
    hdr = payload[0]
    ptype = (hdr >> PTYPE_SHIFT) & PTYPE_MASK
    route = hdr & ROUTE_MASK

    # Determine path_len from wire
    has_transport = route in (0, 3)
    off = 1
    if has_transport:
        off += 4  # skip transport codes
    if off >= len(payload):
        return empty
    path_len = payload[off]
    hop_count, _, path_byte_len = decode_path_len(path_len)
    advert_start = off + 1 + path_byte_len

    # SNR/RSSI from PHY — track for real radio stats
    snr_db = phy.get("snr_db", 0.0)
    snr_byte = max(-128, min(127, int(snr_db * 4)))  # signed, *4
    state.last_snr_db = snr_db
    rssi = phy.get("rssi_dbm")
    if rssi is not None:
        state.last_rssi_dbm = max(-128, min(127, int(rssi)))

    # Count received frames by route type
    # T_DIRECT (route=3) is classified as direct (correct: it has a known path)
    state.pkt_recv += 1
    if route in (ROUTE_FLOOD, ROUTE_T_FLOOD):
        state.pkt_flood_rx += 1
    else:  # ROUTE_DIRECT or ROUTE_T_DIRECT
        state.pkt_direct_rx += 1

    # For ADVERT: push as NEW_ADVERT + auto-learn key + auto-add contact
    if ptype == PAYLOAD_ADVERT:
        return _handle_advert_rx(payload, advert_start, state), []

    # For ACK: check against pending ACKs and push confirmation
    if ptype == PAYLOAD_ACK:
        return _handle_ack_rx(payload, advert_start, state), []

    # For TXT_MSG: decrypt and produce CONTACT_MSG_RECV_V3 + ACK TX
    if ptype == PAYLOAD_TXT:
        return _handle_txt_msg_rx(payload, snr_byte, path_len, state)

    # For ANON_REQ: decrypt and produce CONTACT_MSG_RECV_V3 + ACK TX
    if ptype == PAYLOAD_ANON_REQ:
        return _handle_anon_req_rx(payload, snr_byte, path_len, state)

    # For GRP_TXT: decrypt and produce CHANNEL_MSG_RECV_V3 (no ACK for group)
    if ptype == PAYLOAD_GRP_TXT:
        return _handle_grp_txt_rx(payload, snr_byte, path_len, state), []

    # For PATH: decrypt and extract bundled ACK (response to flood TXT_MSG)
    if ptype == PAYLOAD_PATH:
        return _handle_path_rx(payload, path_len, state), []

    # For CTRL: push as CONTROL_DATA (for node_discover responses etc.)
    if ptype == PAYLOAD_CTRL:
        return _handle_ctrl_rx(payload, path_len, state), []

    return empty


def _advert_to_push(payload: bytes, advert_start: int) -> bytes | None:
    """Convert an ADVERT payload to a PUSH_NEW_ADVERT (0x8A) companion message.

    The push uses the same format as CONTACT (0x03): pubkey(32) + type(1) +
    flags(1) + out_path_len(1) + path(64) + name(32) + last_advert(4) +
    lat(4) + lon(4) + lastmod(4) = 147 bytes after the response code.
    """
    if advert_start + 100 > len(payload):
        return None  # too short for pubkey + timestamp + signature

    pubkey = payload[advert_start : advert_start + 32]
    ts_bytes = payload[advert_start + 32 : advert_start + 36]
    # Skip signature (64 bytes)
    app_start = advert_start + 100
    app_data = payload[app_start:] if app_start < len(payload) else b""

    # Parse app_data for name and location
    name = b""
    lat = 0
    lon = 0
    node_type = 0x01
    if app_data:
        flags = app_data[0]
        node_type = flags & 0x0F
        off = 1
        if flags & 0x10:  # has location
            if off + 8 <= len(app_data):
                lat, lon = struct.unpack_from("<ii", app_data, off)
            off += 8
        if flags & 0x20:
            off += 2  # feature1
        if flags & 0x40:
            off += 2  # feature2
        if flags & 0x80 and off < len(app_data):
            name = app_data[off:]

    last_advert = int.from_bytes(ts_bytes, "little") if ts_bytes else 0

    # Build contact record (147 bytes)
    buf = bytearray()
    buf.append(PUSH_NEW_ADVERT)
    buf.extend(pubkey)  # 32
    buf.append(node_type)  # 1
    buf.append(0)  # flags: 0
    buf.extend(struct.pack("<b", -1))  # out_path_len: -1 (flood)
    buf.extend(b"\x00" * 64)  # path (unused)
    buf.extend(name[:32].ljust(32, b"\x00"))  # adv_name (32, padded)
    buf.extend(struct.pack("<I", last_advert))
    buf.extend(struct.pack("<i", lat))
    buf.extend(struct.pack("<i", lon))
    buf.extend(struct.pack("<I", last_advert))  # lastmod
    return bytes(buf)


def _handle_advert_rx(
    payload: bytes,
    advert_start: int,
    state: BridgeState,
) -> list[bytes]:
    """Handle received ADVERT: push to companion + auto-learn key + auto-add contact.

    Contacts are automatically persisted to disk so they survive bridge restarts.
    This mirrors what real MeshCore firmware does: every received ADVERT is stored
    as a contact that the companion app can discover via GET_CONTACTS.
    """
    push = _advert_to_push(payload, advert_start)
    if push is None:
        return []

    # Auto-learn public key into key store
    pubkey = extract_advert_pubkey(payload)
    if pubkey is not None and pubkey != state.pub_key:
        is_new = save_pubkey(state.keys_dir, pubkey)
        if is_new:
            state.known_keys[pubkey.hex()] = pubkey

        # Auto-add to contacts so GET_CONTACTS returns it and it survives restarts.
        # The contact record is the 147 bytes starting after the push code byte.
        pk_hex = pubkey.hex()
        if pk_hex not in state.contacts:
            record = push[1:]  # strip PUSH_NEW_ADVERT code byte
            if len(record) == 147:
                state.contacts[pk_hex] = record
                state.contacts_lastmod = int(time.time())
                _persist_contacts(state)
                name = extract_advert_name(payload) or ""
                log.info("contact learned: %s.. '%s'", pk_hex[:8], sanitize_text(name))

    return [push]


def _match_pending_ack(ack_bytes: bytes, state: BridgeState) -> list[bytes]:
    """Match a 4-byte ACK hash against pending_acks and produce PUSH_ACK.

    Shared by _handle_ack_rx (bare ACK packets) and _handle_path_rx (PATH
    packets carrying a bundled ACK as encrypted extra data).
    """
    # Expire old entries
    now = time.monotonic()
    expired = [
        k
        for k, (ts, _) in state.pending_acks.items()
        if now - ts > state.pending_ack_ttl
    ]
    for k in expired:
        del state.pending_acks[k]

    if ack_bytes in state.pending_acks:
        _, dest_prefix = state.pending_acks.pop(ack_bytes)
        # PUSH_SEND_CONFIRMED: [0x82, ack_code(4), trip_time(4)] = 9 bytes
        # ack_code is the 4-byte ACK hash (same as expected_ack in MSG_SENT).
        # meshcore_py reader.py reads bytes 1-4 as "code" and matches against
        # the expected_ack from MSG_SENT. trip_time is zero (we don't track it).
        trip_time = struct.pack("<I", 0)
        push = bytes([PUSH_ACK]) + ack_bytes[:4] + trip_time
        log.info("ACK received for %s", ack_bytes.hex())
        return [push]

    return []


def _handle_ack_rx(
    payload: bytes,
    ack_start: int,
    state: BridgeState,
) -> list[bytes]:
    """Handle received bare ACK (ptype 0x03): extract 4-byte hash and match."""
    if ack_start + 4 > len(payload):
        return []
    ack_bytes = payload[ack_start : ack_start + 4]
    return _match_pending_ack(ack_bytes, state)


def _update_contact_path(
    state: BridgeState,
    sender_pub: bytes,
    return_path: bytes,
) -> bytes | None:
    """Update a contact's out_path_len/path from a received PATH return path.

    The return_path is the route from sender back to us — reversed, it becomes
    the forward path from us to the sender.  Returns a PUSH_PATH_UPDATE (0x81)
    payload if a contact was updated, else None.
    """
    pk_hex = sender_pub.hex()
    record = state.contacts.get(pk_hex)
    if record is None:
        return None

    # Reverse the return path to get the outgoing forward path
    forward_path = bytes(reversed(return_path))
    n = len(forward_path)
    if n > 64:
        forward_path = forward_path[:64]
        n = 64

    # Unpack and repack the 147-byte contact record
    buf = bytearray(record)
    struct.pack_into("<b", buf, 34, n)  # out_path_len (signed)
    buf[35 : 35 + n] = forward_path  # path bytes
    if n < 64:
        buf[35 + n : 99] = b"\x00" * (64 - n)  # zero-pad remainder

    state.contacts[pk_hex] = bytes(buf)
    _persist_contacts(state)

    log.info(
        "path learned for %s..: %d hops → %s",
        pk_hex[:8],
        n,
        forward_path.hex(),
    )

    # Build PUSH_PATH_UPDATE: [0x81][pubkey_prefix(6)][path_len(1)][path(N)]
    push = bytes([PUSH_PATH_UPDATE]) + sender_pub[:6] + bytes([n]) + forward_path
    return push


def _handle_path_rx(
    payload: bytes,
    path_len: int,
    state: BridgeState,
) -> list[bytes]:
    """Handle received PATH packet: decrypt, extract bundled ACK, match pending.

    When the bridge sends a TXT_MSG via FLOOD, the companion responds with a
    PAYLOAD_PATH (0x08) packet — NOT a bare ACK. The PATH packet is encrypted
    with the ECDH shared secret and contains:
      plaintext: [path_len_enc(1)][return_path(N)][extra_type(1)][extra(M)]
    If extra_type == PAYLOAD_ACK (0x03), the extra is a 4-byte ACK hash.

    Wire layout after header+flood_path:
      [dest_hash(1)][src_hash(1)][MAC(2)][ciphertext]
    """
    hdr_info = parse_meshcore_header(payload)
    if hdr_info is None:
        return []
    off = hdr_info["off"]
    inner = payload[off:]
    if len(inner) < 4:  # need at least dest_hash + src_hash + MAC(2)
        return []

    dest_hash = inner[0]
    src_hash = inner[1]
    encrypted = inner[2:]  # MAC(2) + ciphertext

    # Check if we're the destination
    if dest_hash != state.pub_key[0]:
        return []

    # Trial decryption over known keys (match src_hash)
    candidates = [pub for pub in state.known_keys.values() if pub[0] == src_hash]
    plaintext: bytes | None = None
    sender_pub: bytes | None = None
    for peer_pub in candidates:
        secret = meshcore_shared_secret(state.expanded_prv, peer_pub)
        if secret is None:
            continue
        pt = meshcore_mac_then_decrypt(secret, encrypted)
        if pt is not None:
            plaintext = pt
            sender_pub = peer_pub
            break

    if plaintext is None or len(plaintext) < 2:
        log.debug("PATH: decryption failed (not for us or unknown sender)")
        return []

    # Parse plaintext: path_len_enc(1) + path(N) + extra_type(1) + extra(M)
    path_len_enc = plaintext[0]
    hash_count = path_len_enc & 0x3F
    hash_size = ((path_len_enc >> 6) & 0x03) + 1
    path_bytes = hash_count * hash_size
    idx = 1 + path_bytes
    return_path = plaintext[1 : 1 + path_bytes]

    results: list[bytes] = []

    # Learn the path even if there's no extra data (PATH-only is valid)
    if path_bytes > 0 and sender_pub is not None:
        path_push = _update_contact_path(state, sender_pub, return_path)
        if path_push is not None:
            results.append(path_push)

    if idx >= len(plaintext):
        log.debug("PATH: no extra data after return path")
        return results  # Return with any path update, no ACK

    extra_type = plaintext[idx] & 0x0F
    extra = plaintext[idx + 1 :]

    if extra_type == PAYLOAD_ACK and len(extra) >= 4:
        ack_bytes = extra[:4]
        log.info("PATH+ACK: extracted ACK hash %s", ack_bytes.hex())
        results.extend(_match_pending_ack(ack_bytes, state))

    return results


def _build_contact_msg_recv_v3(
    snr_byte: int,
    src_prefix: bytes,
    path_len: int,
    txt_type: int,
    ts: int,
    text: bytes,
) -> bytes:
    """Build a CONTACT_MSG_RECV_V3 (0x10) response payload."""
    buf = bytearray()
    buf.append(RESP_CONTACT_MSG_RECV_V3)
    buf.extend(struct.pack("<b", snr_byte))  # SNR * 4
    buf.extend(b"\x00\x00")  # reserved
    buf.extend(src_prefix[:6].ljust(6, b"\x00"))  # pubkey prefix (6 bytes)
    buf.append(path_len)  # raw packed path_len byte (mode bits [7:6] + hop count [5:0])
    buf.append(txt_type)
    buf.extend(struct.pack("<I", ts))
    buf.extend(text)
    return bytes(buf)


def _build_ack_wire_packet(ack_hash: bytes) -> bytes:
    """Build a MeshCore ACK wire packet with direct routing (no path).

    Wire format: header(1) + path_len(1) + ack_hash(4) = 6 bytes.
    Uses ROUTE_DIRECT with empty path (path_len=0) for point-to-point testing.
    """
    from meshcore_tx import build_wire_packet, make_header

    header = make_header(ROUTE_DIRECT, PAYLOAD_ACK)
    return build_wire_packet(header, ack_hash)


def _handle_txt_msg_rx(
    payload: bytes,
    snr_byte: int,
    path_len: int,
    state: BridgeState,
) -> tuple[list[bytes], list[bytes]]:
    """Decrypt TXT_MSG and produce CONTACT_MSG_RECV_V3 + RF ACK packet."""
    empty: tuple[list[bytes], list[bytes]] = ([], [])
    result = try_decrypt_txt_msg(
        payload, state.expanded_prv, state.pub_key, state.known_keys
    )
    if result is not None:
        text, sender_pub = result
        log.info(">> TXT_MSG from %s..: %s", sender_pub.hex()[:8], sanitize_text(text))

        # Auto-learn sender key
        if sender_pub.hex() not in state.known_keys:
            save_pubkey(state.keys_dir, sender_pub)
            state.known_keys[sender_pub.hex()] = sender_pub

        # Extract timestamp from the plaintext (already decrypted, but we
        # need the original wire timestamp for the companion message)
        hdr_info = parse_meshcore_header(payload)
        if hdr_info is None:
            return empty
        off = hdr_info["off"]
        inner = payload[off:]
        # TXT_MSG inner: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
        # Decrypt to get timestamp from plaintext[0:4]
        secret = meshcore_shared_secret(state.expanded_prv, sender_pub)
        plaintext: bytes | None = None
        if secret is not None:
            plaintext = meshcore_mac_then_decrypt(secret, inner[2:])
            if plaintext is not None and len(plaintext) >= 5:
                ts = struct.unpack_from("<I", plaintext, 0)[0]
                txt_type = plaintext[4] >> 2
            else:
                ts = int(time.time())
                txt_type = 0
        else:
            ts = int(time.time())
            txt_type = 0

        msg = _build_contact_msg_recv_v3(
            snr_byte,
            sender_pub[:6],
            path_len,
            txt_type,
            ts,
            text.encode("utf-8"),
        )

        # Build RF ACK packet (direct routing, no path)
        ack_packets: list[bytes] = []
        if plaintext is not None and len(plaintext) >= 5:
            ack_hash = compute_ack_hash(plaintext, sender_pub)
            ack_packets.append(_build_ack_wire_packet(ack_hash))
            log.info("ACK hash: %s for %s..", ack_hash.hex(), sender_pub.hex()[:8])

        return ([msg], ack_packets)

    # Decryption failed — pass as opaque (not addressed to us, or unknown sender)
    log.debug("TXT_MSG: decryption failed (not for us or unknown sender)")
    return empty


def _handle_anon_req_rx(
    payload: bytes,
    snr_byte: int,
    path_len: int,
    state: BridgeState,
) -> tuple[list[bytes], list[bytes]]:
    """Decrypt ANON_REQ, deliver to companion, and send encrypted RESPONSE to sender.

    MeshCore firmware does NOT ACK ANON_REQ — it sends a RESPONSE (ptype 0x01)
    back to the sender, encrypted with the ECDH shared secret derived from the
    sender's full public key (embedded in the ANON_REQ payload).  We match
    this behaviour: the RESPONSE is returned in ack_packets so run_bridge()
    transmits it as an RF packet.
    """
    from meshcore_tx import build_wire_packet, make_header

    empty: tuple[list[bytes], list[bytes]] = ([], [])
    result = try_decrypt_anon_req(payload, state.expanded_prv, state.pub_key)
    if result is not None:
        text, sender_pub = result
        log.info(">> ANON_REQ from %s..: %s", sender_pub.hex()[:8], sanitize_text(text))

        # Auto-learn sender key (key store only — NOT contacts)
        pk_hex = sender_pub.hex()
        if pk_hex not in state.known_keys:
            save_pubkey(state.keys_dir, sender_pub)
            state.known_keys[pk_hex] = sender_pub
            log.info("key learned: %s.. (ANON_REQ)", pk_hex[:8])

        ts = int(time.time())
        msg = _build_contact_msg_recv_v3(
            snr_byte,
            sender_pub[:6],
            path_len,
            0,  # txt_type: plain text
            ts,
            text.encode("utf-8"),
        )

        # Build encrypted RESPONSE (payload_type 0x01) back to sender.
        # Plaintext: RESP_SERVER_LOGIN_OK (0x00) + our node name.
        # This mirrors what MeshCore firmware sends in handleAnonMessage().
        shared = meshcore_shared_secret(state.expanded_prv, sender_pub)
        if shared is None:
            log.warning("ANON_REQ: ECDH failed, skipping RESPONSE")
            return ([msg], [])
        resp_plaintext = bytes([0x00]) + state.name.encode("utf-8")[:32]
        resp_enc = meshcore_encrypt_then_mac(shared, resp_plaintext)
        # RESP payload: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
        resp_payload = bytes([sender_pub[0], state.pub_key[0]]) + resp_enc
        resp_pkt = build_wire_packet(
            make_header(ROUTE_DIRECT, PAYLOAD_RESP), resp_payload, path=b""
        )
        log.debug("ANON_REQ: sending RESPONSE to %s..", pk_hex[:8])
        return ([msg], [resp_pkt])

    log.debug("ANON_REQ: decryption failed (not addressed to us)")
    return empty


def _handle_grp_txt_rx(
    payload: bytes,
    snr_byte: int,
    path_len: int,
    state: BridgeState,
) -> list[bytes]:
    """Decrypt GRP_TXT and produce CHANNEL_MSG_RECV_V3."""
    result = try_decrypt_grp_txt(payload, state.channels)
    if result is not None:
        text, channel_name = result
        log.info(">> GRP_TXT #%s: %s", sanitize_text(channel_name), sanitize_text(text))

        # Find channel index for companion protocol
        chan_idx = 0
        for i, ch in enumerate(state.channels):
            if ch.name == channel_name:
                chan_idx = i
                break

        # Extract timestamp from decrypted plaintext
        hdr_info = parse_meshcore_header(payload)
        ts = int(time.time())
        if hdr_info is not None:
            off = hdr_info["off"]
            inner = payload[off:]
            # GRP_TXT inner: channel_hash(1) + MAC(2) + ciphertext
            ch_obj = (
                state.channels[chan_idx] if chan_idx < len(state.channels) else None
            )
            if ch_obj is not None:
                plaintext = meshcore_mac_then_decrypt(ch_obj.secret, inner[1:])
                if plaintext is not None and len(plaintext) >= 4:
                    ts = struct.unpack_from("<I", plaintext, 0)[0]

        buf = bytearray()
        buf.append(RESP_CHANNEL_MSG_RECV_V3)
        buf.extend(struct.pack("<b", snr_byte))
        buf.extend(b"\x00\x00")  # reserved
        buf.append(chan_idx)
        buf.append(
            path_len
        )  # raw packed path_len byte (mode bits [7:6] + hop count [5:0])
        buf.append(0)  # txt_type: text
        buf.extend(struct.pack("<I", ts))
        buf.extend(text.encode("utf-8"))
        return [bytes(buf)]

    log.debug("GRP_TXT: decryption failed (unknown channel)")
    return []


# ---- Contact persistence ----

DEFAULT_CONTACTS_DIR = (
    Path(__file__).resolve().parent / "data" / "meshcore" / "contacts"
)


def _persist_contacts(state: BridgeState) -> None:
    """Save all contacts to disk for persistence across restarts."""
    cdir = state.contacts_dir
    if cdir is None:
        cdir = DEFAULT_CONTACTS_DIR
    try:
        cdir.mkdir(parents=True, exist_ok=True)
        for pk_hex, record in state.contacts.items():
            path = cdir / f"{pk_hex}.contact"
            path.write_bytes(record)
    except OSError as exc:
        log.warning("could not persist contacts to %s: %s", cdir, exc)


def _load_persisted_contacts(contacts_dir: Path | None) -> dict[str, bytes]:
    """Load persisted contacts from disk."""
    cdir = contacts_dir if contacts_dir is not None else DEFAULT_CONTACTS_DIR
    contacts: dict[str, bytes] = {}
    if not cdir.is_dir():
        return contacts
    for f in cdir.glob("*.contact"):
        data = f.read_bytes()
        if len(data) == 147:
            # Pubkey is first 32 bytes of the record
            pk_hex = data[:32].hex()
            contacts[pk_hex] = data
        else:
            log.warning(
                "skipping corrupt contact file %s (%d bytes, expected 147)",
                f,
                len(data),
            )
    return contacts


def _delete_persisted_contact(contacts_dir: Path | None, pk_hex: str) -> None:
    """Delete a persisted contact file."""
    cdir = contacts_dir if contacts_dir is not None else DEFAULT_CONTACTS_DIR
    path = cdir / f"{pk_hex}.contact"
    try:
        if path.exists():
            path.unlink()
    except OSError as exc:
        log.warning("could not delete contact file %s: %s", path, exc)


# ---- Command handlers ----


def _build_contact_record(
    pubkey: bytes,
    name: str = "",
    node_type: int = 0x01,
    lat: int = 0,
    lon: int = 0,
    last_advert: int = 0,
) -> bytes:
    """Build a 147-byte contact record (matches CONTACT response format).

    Layout: pubkey(32) + type(1) + flags(1) + path_len(1) + path(64) +
            name(32) + last_advert(4) + lat(4) + lon(4) + lastmod(4) = 147 bytes.
    """
    if last_advert == 0:
        last_advert = int(time.time())
    buf = bytearray()
    buf.extend(pubkey[:32].ljust(32, b"\x00"))
    buf.append(node_type)
    buf.append(0)  # flags
    buf.extend(struct.pack("<b", -1))  # path_len: -1 (flood)
    buf.extend(b"\x00" * 64)  # path
    buf.extend(utf8_truncate(name, 32).ljust(32, b"\x00"))
    buf.extend(struct.pack("<I", last_advert))
    buf.extend(struct.pack("<i", lat))
    buf.extend(struct.pack("<i", lon))
    buf.extend(struct.pack("<I", last_advert))  # lastmod
    return bytes(buf)


def _build_self_advert_push(state: BridgeState) -> bytes | None:
    """Build a PUSH_NEW_ADVERT (0x8A) for our own identity.

    This lets the companion app auto-learn us as a contact on connect.
    """
    record = _build_contact_record(state.pub_key, name=state.name)
    return bytes([PUSH_NEW_ADVERT]) + record


def handle_command(
    cmd_payload: bytes,
    state: BridgeState,
    udp_sock: socket.socket | None,
    udp_addr: tuple[str, int] | None,
) -> list[bytes]:
    """Handle a companion protocol command, return list of response payloads."""
    if not cmd_payload:
        return []

    cmd = cmd_payload[0]
    data = cmd_payload[1:]

    if cmd == CMD_APP_START:
        log.info("companion: APP_START")
        # Re-apply config.toml values so companion session changes don't persist
        state.apply_startup_config()
        # Send SELF_INFO + push our own ADVERT so companion learns our contact
        responses = [state.build_self_info()]
        self_advert = _build_self_advert_push(state)
        if self_advert is not None:
            responses.append(self_advert)
        # Also persist self in contacts so GET_CONTACTS returns us
        _ensure_self_contact(state)
        return responses

    if cmd == CMD_DEVICE_QUERY:
        return [state.build_device_info()]

    if cmd == CMD_GET_CONTACTS:
        return state.build_contacts_response()

    if cmd == CMD_GET_DEVICE_TIME:
        return [state.build_current_time()]

    if cmd == CMD_SET_DEVICE_TIME:
        return [state.build_ok()]

    if cmd == CMD_GET_BATT_AND_STORAGE:
        return [state.build_battery()]

    if cmd == CMD_GET_CHANNEL:
        idx = data[0] if data else 0
        if idx >= 8:  # max_channels (must match build_device_info)
            return [state.build_error()]
        return [state.build_channel_info(idx)]

    if cmd == CMD_GET_STATS:
        stats_type = data[0] if data else 0
        return [state.build_stats(stats_type)]

    if cmd == CMD_SYNC_NEXT_MESSAGE:
        if state.msg_queue:
            msg = state.msg_queue.popleft()
            return [msg]
        return [state.build_no_more_msgs()]

    if cmd == CMD_SEND_TXT_MSG:
        assert udp_sock is not None and udp_addr is not None
        return _handle_send_txt_msg(data, state, udp_sock, udp_addr)

    if cmd == CMD_SEND_CHAN_TXT_MSG:
        assert udp_sock is not None and udp_addr is not None
        return _handle_send_chan_txt_msg(data, state, udp_sock, udp_addr)

    if cmd == CMD_SEND_CONTROL_DATA:
        assert udp_sock is not None and udp_addr is not None
        return _handle_send_control_data(data, state, udp_sock, udp_addr)

    if cmd == CMD_SEND_SELF_ADVERT:
        assert udp_sock is not None and udp_addr is not None
        return _handle_send_advert(data, state, udp_sock, udp_addr)

    if cmd == CMD_SET_ADVERT_NAME:
        name = data.decode("utf-8", errors="replace").rstrip("\x00")
        state.name = name
        log.info("companion: SET_ADVERT_NAME '%s'", sanitize_text(name))
        return [state.build_ok()]

    if cmd == CMD_SET_RADIO_PARAMS:
        # Data: freq_hz(4 LE u32) + bw_hz(4 LE u32) + sf(1) + cr(1) [+ repeat(1)]
        if len(data) >= 10:
            freq_hz = struct.unpack_from("<I", data, 0)[0]
            bw_hz = struct.unpack_from("<I", data, 4)[0]
            sf_new = data[8]
            cr_new = data[9]
            # Optional 5th byte: repeat flag (firmware v9+)
            if len(data) >= 11:
                repeat = data[10]
                if repeat and not _is_valid_repeat_freq(freq_hz // 1000):
                    log.warning(
                        "companion: SET_RADIO_PARAMS repeat=1 rejected: "
                        "freq %d kHz not in allowed repeat ranges",
                        freq_hz // 1000,
                    )
                    return [state.build_error(ERR_CODE_ILLEGAL_ARG)]
                state.client_repeat = repeat
            state.freq_mhz = freq_hz / 1_000_000.0
            state.bw_khz = bw_hz / 1_000.0
            state.sf = sf_new
            state.cr = cr_new
            log.warning(
                "companion: SET_RADIO_PARAMS freq=%.3f MHz bw=%.1f kHz sf=%d cr=%d "
                "repeat=%d (RX retune requires lora_trx restart)",
                state.freq_mhz,
                state.bw_khz,
                state.sf,
                state.cr,
                state.client_repeat,
            )
        return [state.build_ok()]

    if cmd == CMD_SET_RADIO_TX_POWER:
        if data:
            # Byte is unsigned; values ≥ 128 are treated as negative (int8 range)
            dbm = data[0] if data[0] < 128 else data[0] - 256
            state.tx_power = dbm
            log.info("companion: SET_RADIO_TX_POWER %d dBm", dbm)
        return [state.build_ok()]

    if cmd == CMD_IMPORT_CONTACT:
        return _handle_import_contact(data, state)

    if cmd == CMD_ADD_UPDATE_CONTACT:
        return _handle_add_contact(data, state)

    if cmd == CMD_REMOVE_CONTACT:
        if len(data) >= 32:
            pk_hex = data[:32].hex()
            state.contacts.pop(pk_hex, None)
            _delete_persisted_contact(state.contacts_dir, pk_hex)
            log.info("companion: REMOVE_CONTACT %s..", pk_hex[:8])
        return [state.build_ok()]

    if cmd == CMD_EXPORT_CONTACT:
        # Return raw ADVERT wire packet for the requested contact (6-byte prefix).
        # Response: RESP_CONTACT_URI (0x0B) + wire packet bytes.
        return _handle_export_contact(data, state)

    if cmd == CMD_SHARE_CONTACT:
        # share_contact: build the wire packet (same as export), log URI, return OK.
        # mccli's share_contact awaits EventType.OK, not CONTACT_URI.
        frames = _handle_export_contact(data, state)
        if frames and frames[0][0] == RESP_CONTACT_URI:
            wire_pkt = frames[0][1:]
            log.info("share_contact URI: meshcore://%s", wire_pkt.hex())
        else:
            return frames  # propagate errors unchanged
        return [state.build_ok()]

    if cmd == CMD_SET_CHANNEL:
        return _handle_set_channel(data, state)

    if cmd == CMD_SET_ADVERT_LATLON:
        # Data: lat(4 LE i32) + lon(4 LE i32), values are degrees * 1e6
        if len(data) >= 8:
            state.lat_e6 = struct.unpack_from("<i", data, 0)[0]
            state.lon_e6 = struct.unpack_from("<i", data, 4)[0]
            log.info(
                "companion: SET_ADVERT_LATLON lat=%.6f lon=%.6f",
                state.lat_e6 / 1e6,
                state.lon_e6 / 1e6,
            )
        return [state.build_ok()]

    if cmd == CMD_SET_FLOOD_SCOPE:
        # Frame: [0x36][0x00][key?:16]  (second byte is discriminator, must be 0)
        if not data or data[0] != 0:
            return [state.build_error(ERR_CODE_ILLEGAL_ARG)]
        if len(data) >= 17:  # discriminator(1) + key(16)
            incoming_key = bytes(data[1:17])
            _null_key = b"\x00" * 16
            # If config.toml specifies a scope and the companion sends all-zeros,
            # treat it as the client's stale default (not an intentional clear) and
            # ignore it so the config value survives the companion init sequence.
            if incoming_key == _null_key and any(state._startup_send_scope):
                log.debug(
                    "companion: SET_FLOOD_SCOPE all-zero ignored (config scope active)"
                )
            else:
                state.send_scope = incoming_key
                log.info("companion: SET_FLOOD_SCOPE key=%s", state.send_scope.hex())
        else:
            state.send_scope = b"\x00" * 16  # clear scope
            log.info("companion: SET_FLOOD_SCOPE cleared")
        return [state.build_ok()]

    if cmd == CMD_GET_ALLOWED_REPEAT_FREQ:
        buf = bytearray([RESP_ALLOWED_REPEAT_FREQ])
        for lo, hi in REPEAT_FREQ_RANGES:
            buf += struct.pack("<II", lo, hi)
        return [bytes(buf)]

    if cmd == CMD_SET_AUTOADD_CONFIG:
        state.autoadd_config = data[0] if data else 0
        if len(data) >= 2:
            state.autoadd_max_hops = min(data[1], 64)
        log.info(
            "companion: SET_AUTOADD_CONFIG config=0x%02x max_hops=%d",
            state.autoadd_config,
            state.autoadd_max_hops,
        )
        return [state.build_ok()]

    if cmd == CMD_GET_AUTOADD_CONFIG:
        return [
            bytes(
                [RESP_CODE_AUTOADD_CONFIG, state.autoadd_config, state.autoadd_max_hops]
            )
        ]

    if cmd == CMD_SET_PATH_HASH_MODE:
        # Frame: [0x3D][0x00][mode(1)]  (second byte is discriminator, must be 0)
        if len(data) < 2 or data[0] != 0:
            return [state.build_error(ERR_CODE_ILLEGAL_ARG)]
        if data[1] >= 3:
            return [state.build_error(ERR_CODE_ILLEGAL_ARG)]
        state.path_hash_mode = data[1]
        log.info("companion: SET_PATH_HASH_MODE mode=%d", state.path_hash_mode)
        return [state.build_ok()]

    if cmd == CMD_SET_OTHER_PARAMS:
        # Frame: [0x26][manual_add_contacts(1)][telemetry_mode(1)][adv_loc_policy(1)][multi_acks(1)?]
        # Legacy variant is 3 bytes; new variant adds a 4th byte for multi_acks.
        if len(data) >= 3:
            state.manual_add_contacts = data[0]
            state.telemetry_mode = data[1]
            state.adv_loc_policy = data[2]
            state.multi_acks = data[3] if len(data) >= 4 else 0
            log.info(
                "companion: SET_OTHER_PARAMS manual_add=%d telemetry=0x%02x adv_loc=%d multi_acks=%d",
                state.manual_add_contacts,
                state.telemetry_mode,
                state.adv_loc_policy,
                state.multi_acks,
            )
        else:
            log.warning("companion: SET_OTHER_PARAMS too short (%d bytes)", len(data))
        return [state.build_ok()]

    if cmd == CMD_GET_CUSTOM_VARS:
        # Bridge has no hardware sensors, so custom vars are always empty.
        # Response: RESP_CUSTOM_VARS (0x15) with no key=value pairs.
        log.debug("companion: GET_CUSTOM_VARS → empty (no hardware sensors)")
        return [bytes([RESP_CUSTOM_VARS])]

    if cmd == CMD_SET_CUSTOM_VAR:
        # Bridge has no writable custom vars — all keys are illegal.
        log.debug("companion: SET_CUSTOM_VAR → ILLEGAL_ARG (no hardware sensors)")
        return [state.build_error(ERR_CODE_ILLEGAL_ARG)]

    if cmd in (CMD_RESET_PATH, CMD_SET_TUNING_PARAMS, CMD_REBOOT):
        return [state.build_ok()]

    if cmd in (CMD_LOGIN, CMD_STATUS_REQ, CMD_TRACE, CMD_PATH_DISCOVERY):
        log.debug("companion: unimplemented repeater cmd 0x%02x (stub MSG_SENT)", cmd)
        return [state.build_msg_sent(flood=True)]

    # Unknown command — return OK silently
    return [state.build_ok()]


def _hash_payload(payload: bytes) -> bytes:
    """Hash a payload for TX echo filtering."""
    return hashlib.sha256(payload).digest()[:8]


def _compute_transport_code(
    region_key: bytes, payload_type: int, payload: bytes
) -> int:
    """Compute a 2-byte transport code using HMAC-SHA256.

    Matches MeshCore TransportKey::calcTransportCode():
      HMAC-SHA256(key, payload_type(1) + payload) truncated to 2 bytes (LE).
      Values 0x0000 and 0xFFFF are reserved and adjusted.
    """
    import hmac

    msg = bytes([payload_type]) + payload
    mac = hmac.new(region_key, msg, hashlib.sha256).digest()[:2]
    code = int.from_bytes(mac, "little")
    if code == 0x0000:
        code = 0x0001
    elif code == 0xFFFF:
        code = 0xFFFE
    return code


def _handle_send_txt_msg(
    data: bytes,
    state: BridgeState,
    udp_sock: socket.socket,
    udp_addr: tuple[str, int],
) -> list[bytes]:
    """Handle CMD_SEND_TXT_MSG: build MeshCore wire packet and TX via UDP."""
    if len(data) < 12:  # type(1) + attempt(1) + ts(4) + dst(6)
        return [state.build_error()]

    msg_type = data[0]  # 0=text, 1=command
    attempt = data[1]
    ts = struct.unpack_from("<I", data, 2)[0]
    dst_prefix = data[6:12]
    text = data[12:]

    log.info(
        "companion: SEND_TXT_MSG type=%d dst=%s '%s'",
        msg_type,
        dst_prefix.hex(),
        sanitize_text(text.decode("utf-8", errors="replace")),
    )

    # Look up full destination pubkey and contact record (6-byte prefix match)
    resolved = _resolve_contact(state, dst_prefix)
    if resolved is None:
        log.error("no contact matching prefix %s", dst_prefix.hex())
        return [state.build_error(2)]
    dest_pub, contact_record = resolved

    # Determine routing: replicate firmware BaseChatMesh::sendMessage() logic.
    # out_path_len == -1 (0xFF) → no known direct path → send as FLOOD.
    # out_path_len == 0         → zero-hop direct → ROUTE_DIRECT, path_len=0.
    # out_path_len  > 0         → multi-hop direct → ROUTE_DIRECT + stored path.
    out_path_len = struct.unpack_from("<b", contact_record, 34)[0]  # signed byte
    use_flood = out_path_len < 0

    # Priority: send_scope (CMD_SET_FLOOD_SCOPE raw key) > region_key (hashtag-derived)
    active_key = state.send_scope if any(state.send_scope) else state.region_key

    from meshcore_tx import (
        build_txt_msg,
        build_wire_packet,
        make_cbor_tx_request,
        make_header,
    )

    text_str = text.decode("utf-8", errors="replace")

    if use_flood:
        # Build encrypted payload first (ROUTE_FLOOD, no transport codes).
        # Then optionally repackage with ROUTE_T_FLOOD + transport codes.
        tmp = build_txt_msg(
            state.expanded_prv,
            state.pub_key,
            dest_pub,
            text_str,
            timestamp=ts,
            attempt=attempt,
            route_type=ROUTE_FLOOD,
        )
        if active_key:
            # Extract payload: skip header(1) + path_len(1) — no transport codes
            # in the plain ROUTE_FLOOD packet built above.
            txt_payload = tmp[2:]
            tc1 = _compute_transport_code(active_key, PAYLOAD_TXT, txt_payload)
            header = make_header(ROUTE_T_FLOOD, PAYLOAD_TXT)
            packet = build_wire_packet(header, txt_payload, transport_codes=(tc1, 0))
        else:
            packet = tmp
        log.info("TX: %dB TXT_MSG (flood) to %s", len(packet), dst_prefix.hex())
    else:
        # Zero-hop direct (out_path_len == 0) or known multi-hop path (out_path_len > 0).
        # Multi-hop path routing (ROUTE_T_DIRECT with stored_path) is not yet implemented;
        # for now fall through to zero-hop ROUTE_DIRECT which works for both cases.
        packet = build_txt_msg(
            state.expanded_prv,
            state.pub_key,
            dest_pub,
            text_str,
            timestamp=ts,
            attempt=attempt,
            route_type=ROUTE_DIRECT,
        )
        log.info(
            "TX: %dB TXT_MSG (direct, path_len=%d) to %s",
            len(packet),
            out_path_len,
            dst_prefix.hex(),
        )

    # Track TX hash to filter echo
    h = _hash_payload(packet)
    state.recent_tx[h] = time.monotonic()
    log.debug("echo-filter: track TX hash %s", h.hex())

    # Send via UDP CBOR
    cbor_msg = make_cbor_tx_request(packet)
    udp_sock.sendto(cbor_msg, udp_addr)
    state.pkt_sent += 1
    if use_flood:
        state.pkt_flood_tx += 1
    else:
        state.pkt_direct_tx += 1

    # Build response and register expected ACK hash for tracking.
    # The companion device ACKs with SHA-256(plaintext + sender_pub)[:4].
    # Both RESP_MSG_SENT (expected_ack field) and pending_acks must use the
    # same crypto hash — meshcore-cli matches PUSH_ACK.code against
    # MSG_SENT.expected_ack to confirm delivery.
    plaintext = struct.pack("<I", ts) + bytes([(0x00 << 2) | (attempt & 0x03)]) + text
    expected_ack = compute_ack_hash(plaintext, state.pub_key)
    resp = state.build_msg_sent(flood=use_flood, ack_hash=expected_ack)
    state.pending_acks[expected_ack] = (time.monotonic(), dst_prefix)

    return [resp]


def _handle_send_advert(
    data: bytes,
    state: BridgeState,
    udp_sock: socket.socket,
    udp_addr: tuple[str, int],
) -> list[bytes]:
    """Handle CMD_SEND_SELF_ADVERT: broadcast ADVERT via UDP."""
    from meshcore_tx import build_advert, build_wire_packet, make_cbor_tx_request

    flood = len(data) > 0 and data[0] == 0x01

    # Include location in ADVERT when set (CMD_SET_ADVERT_LATLON or config.toml)
    lat = state.lat_e6 / 1e6 if state.lat_e6 != 0 else None
    lon = state.lon_e6 / 1e6 if state.lon_e6 != 0 else None

    # Priority: send_scope (CMD_SET_FLOOD_SCOPE raw key) > region_key (hashtag-derived)
    active_key = state.send_scope if any(state.send_scope) else state.region_key
    if active_key:
        # Build with T_FLOOD route + transport codes.
        tmp = build_advert(state.seed, state.pub_key, name=state.name, lat=lat, lon=lon)
        # Extract payload: skip header(1) + path_len(1) — no transport codes
        # in the default ROUTE_FLOOD packet.
        advert_payload = tmp[2:]
        tc1 = _compute_transport_code(active_key, PAYLOAD_ADVERT, advert_payload)
        from meshcore_tx import make_header

        header = make_header(ROUTE_T_FLOOD, PAYLOAD_ADVERT)
        packet = build_wire_packet(header, advert_payload, transport_codes=(tc1, 0))
    else:
        packet = build_advert(
            state.seed, state.pub_key, name=state.name, lat=lat, lon=lon
        )

    # Track TX hash to filter echo
    h = _hash_payload(packet)
    state.recent_tx[h] = time.monotonic()
    log.debug("echo-filter: track TX hash %s", h.hex())

    cbor_msg = make_cbor_tx_request(packet)
    udp_sock.sendto(cbor_msg, udp_addr)
    state.pkt_sent += 1
    state.pkt_flood_tx += 1
    log.info("TX: ADVERT '%s' %dB", state.name, len(packet))

    return [state.build_ok()]


def _handle_send_chan_txt_msg(
    data: bytes,
    state: BridgeState,
    udp_sock: socket.socket,
    udp_addr: tuple[str, int],
) -> list[bytes]:
    """Handle CMD_SEND_CHAN_TXT_MSG: build encrypted GRP_TXT and TX via UDP.

    Companion format: [0x03, msg_type(1), chan_idx(1), ts(4 LE), text...]
    The firmware disambiguates CMD_SEND_TXT_MSG (0x02) vs CMD_SEND_CHAN_TXT_MSG
    (0x03) by command code. Our bridge separates them at the command dispatch level.
    """
    if len(data) < 7:  # msg_type(1) + chan_idx(1) + ts(4) + at least 1 byte text
        return [state.build_error()]

    _msg_type = data[0]
    chan_idx = data[1]
    ts = struct.unpack_from("<I", data, 2)[0]
    text = data[6:]

    if chan_idx >= len(state.channels):
        log.error(
            "channel index %d out of range (%d channels loaded)",
            chan_idx,
            len(state.channels),
        )
        return [state.build_error(3)]  # channel not found

    channel = state.channels[chan_idx]
    text_str = text.decode("utf-8", errors="replace")

    log.info(
        "companion: SEND_CHAN_TXT_MSG ch=%d '%s': '%s'",
        chan_idx,
        sanitize_text(channel.name),
        sanitize_text(text_str),
    )

    # Build encrypted GRP_TXT payload
    from meshcore_tx import build_wire_packet, make_header

    grp_payload = build_grp_txt(channel, state.name, text_str, timestamp=ts)

    # Priority: send_scope (CMD_SET_FLOOD_SCOPE raw key) > region_key (hashtag-derived)
    active_key = state.send_scope if any(state.send_scope) else state.region_key
    if active_key:
        # Use T_FLOOD with transport codes
        tc1 = _compute_transport_code(active_key, PAYLOAD_GRP_TXT, grp_payload)
        header = make_header(ROUTE_T_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload, transport_codes=(tc1, 0))
    else:
        header = make_header(ROUTE_FLOOD, PAYLOAD_GRP_TXT)
        packet = build_wire_packet(header, grp_payload)

    # Track TX hash to filter echo
    h = _hash_payload(packet)
    state.recent_tx[h] = time.monotonic()
    log.debug("echo-filter: track TX hash %s", h.hex())

    # Send via UDP CBOR
    from meshcore_tx import make_cbor_tx_request

    cbor_msg = make_cbor_tx_request(packet)
    udp_sock.sendto(cbor_msg, udp_addr)
    state.pkt_sent += 1
    state.pkt_flood_tx += 1
    log.info("TX: %dB GRP_TXT to #%s", len(packet), channel.name)

    return [state.build_ok()]


def _handle_send_control_data(
    data: bytes,
    state: BridgeState,
    udp_sock: socket.socket,
    udp_addr: tuple[str, int],
) -> list[bytes]:
    """Handle CMD_SEND_CONTROL_DATA: build CTRL wire packet and TX via UDP.

    Companion format: [0x37, control_type(1), payload...]
    The data parameter here is everything after the 0x37 command byte.
    Used by mccli node_discover (control_type=0x80|flags).
    """
    if not data:
        return [state.build_error()]

    from meshcore_tx import build_wire_packet, make_header

    header = make_header(ROUTE_FLOOD, PAYLOAD_CTRL)
    packet = build_wire_packet(header, data)

    # Track TX hash to filter echo
    h = _hash_payload(packet)
    state.recent_tx[h] = time.monotonic()
    log.debug("echo-filter: track TX hash %s", h.hex())

    from meshcore_tx import make_cbor_tx_request

    cbor_msg = make_cbor_tx_request(packet)
    udp_sock.sendto(cbor_msg, udp_addr)
    state.pkt_sent += 1
    state.pkt_flood_tx += 1
    log.info("TX: %dB CTRL type=0x%02x", len(packet), data[0])

    return [state.build_ok()]


def _handle_set_channel(data: bytes, state: BridgeState) -> list[bytes]:
    """Handle CMD_SET_CHANNEL: store or clear a channel.

    Format: [idx(1), name(32), secret(32)] = 65 bytes of data.
    If name is all zeros, the channel is cleared.
    """
    if len(data) < 65:
        return [state.build_error(2)]

    idx = data[0]
    name_raw = data[1:33].rstrip(b"\x00")
    secret_raw = data[33:65]

    if idx >= 8:
        return [state.build_error(5)]  # index out of range

    name = name_raw.decode("utf-8", errors="replace").rstrip("\x00")

    if not name:
        # Clear channel at this index
        if idx < len(state.channels):
            old_name = state.channels[idx].name
            state.channels[idx : idx + 1] = []
            log.info("companion: cleared channel %d '%s'", idx, sanitize_text(old_name))
        return [state.build_ok()]

    # Only first 16 bytes of secret are used (MeshCore 128-bit PSK)
    psk_16 = secret_raw[:16]
    ch = GroupChannel(name, psk_16)

    # Store at index (extend list if needed)
    while len(state.channels) <= idx:
        state.channels.append(GroupChannel("", b"\x00" * 16))
    state.channels[idx] = ch

    # Persist to disk
    save_channel(state.channels_dir, name, psk_16)
    log.info("companion: SET_CHANNEL %d '%s'", idx, sanitize_text(name))

    return [state.build_ok()]


def _handle_add_contact(data: bytes, state: BridgeState) -> list[bytes]:
    """Handle CMD_ADD_UPDATE_CONTACT."""
    if len(data) < 32:
        return [state.build_error()]

    pk = data[:32]
    pk_hex = pk.hex()

    # Extract name for logging
    name_offset = 32 + 1 + 1 + 1 + 64  # pubkey + type + flags + path_len + path
    if name_offset + 32 <= len(data):
        name = (
            data[name_offset : name_offset + 32]
            .rstrip(b"\x00")
            .decode("utf-8", errors="replace")
        )
    else:
        name = ""

    # CMD_ADD_UPDATE_CONTACT data is 143 bytes (no lastmod), pad to 147
    now = int(time.time())
    record = data[:143].ljust(143, b"\x00") + struct.pack("<I", now)
    state.contacts[pk_hex] = record
    state.contacts_lastmod = now
    _persist_contacts(state)

    log.info("contact: %s.. '%s'", pk_hex[:8], sanitize_text(name))
    return [state.build_ok()]


def _parse_advert_wire_packet(data: bytes) -> dict[str, Any] | None:
    """Parse a raw MeshCore ADVERT wire packet and extract contact info.

    The wire packet format is: [header(1)][opt transport(4)][path_len(1)][path(N)]
    followed by ADVERT payload: [pubkey(32)][timestamp(4)][signature(64)][app_data(N)].

    Returns dict with pubkey, name, node_type, lat, lon, last_advert or None on failure.
    """
    if len(data) < 2:
        return None

    hdr = data[0]
    ptype = (hdr >> PTYPE_SHIFT) & PTYPE_MASK
    route = hdr & ROUTE_MASK

    if ptype != PAYLOAD_ADVERT:
        return None

    # Skip transport codes if present
    has_transport = route in (0, 3)
    off = 1
    if has_transport:
        off += 4

    if off >= len(data):
        return None
    path_len = data[off]
    _, _, path_byte_len = decode_path_len(path_len)
    off += 1 + path_byte_len  # skip path_len byte + path

    # ADVERT payload starts here: pubkey(32) + timestamp(4) + signature(64) + app_data
    advert_start = off
    if advert_start + 100 > len(data):
        return None  # too short for pubkey + timestamp + signature

    pubkey = data[advert_start : advert_start + 32]
    ts_bytes = data[advert_start + 32 : advert_start + 36]
    # Skip signature (64 bytes)
    app_start = advert_start + 100
    app_data = data[app_start:] if app_start < len(data) else b""

    # Parse app_data
    name = b""
    lat = 0
    lon = 0
    node_type = 0x01
    if app_data:
        flags = app_data[0]
        node_type = flags & 0x0F
        aoff = 1
        if flags & 0x10:  # has location
            if aoff + 8 <= len(app_data):
                lat, lon = struct.unpack_from("<ii", app_data, aoff)
            aoff += 8
        if flags & 0x20:
            aoff += 2  # feature1
        if flags & 0x40:
            aoff += 2  # feature2
        if flags & 0x80 and aoff < len(app_data):
            name = app_data[aoff:]

    last_advert = int.from_bytes(ts_bytes, "little") if ts_bytes else 0

    return {
        "pubkey": pubkey,
        "name": name.decode("utf-8", errors="replace").rstrip("\x00"),
        "node_type": node_type,
        "lat": lat,
        "lon": lon,
        "last_advert": last_advert,
    }


def _handle_import_contact(data: bytes, state: BridgeState) -> list[bytes]:
    """Handle CMD_IMPORT_CONTACT: parse raw ADVERT wire packet and store contact.

    meshcore-cli sends the raw hex-decoded wire packet from a meshcore:// URI.
    """
    info = _parse_advert_wire_packet(data)
    if info is None:
        log.warning("companion: IMPORT_CONTACT -- failed to parse ADVERT")
        return [state.build_error()]

    pk_hex = info["pubkey"].hex()
    record = _build_contact_record(
        info["pubkey"],
        name=info["name"],
        node_type=info["node_type"],
        lat=info["lat"],
        lon=info["lon"],
        last_advert=info["last_advert"],
    )
    state.contacts[pk_hex] = record
    state.contacts_lastmod = int(time.time())
    _persist_contacts(state)

    log.info(
        "companion: IMPORT_CONTACT %s.. '%s'", pk_hex[:8], sanitize_text(info["name"])
    )
    return [state.build_ok()]


def _handle_export_contact(data: bytes, state: BridgeState) -> list[bytes]:
    """Handle CMD_EXPORT_CONTACT and CMD_SHARE_CONTACT.

    Data is a 6-byte pubkey prefix.  Returns RESP_CONTACT_URI (0x0B) + raw
    ADVERT wire packet bytes.  The companion app hex-encodes these and prepends
    "meshcore://" to form the biz card URI (compatible with meshcore-cli
    import_contact).

    For our own identity: returns a fully signed ADVERT from build_advert().
    For stored contacts: reconstructs an unsigned wire packet from the
    147-byte contact record (zeroed 64-byte signature — original not stored).
    """
    from meshcore_tx import build_advert, build_wire_packet, make_header

    prefix = data[:6] if len(data) >= 6 else data

    # Check if this is ourselves
    if not prefix or state.pub_key[: len(prefix)] == prefix:
        lat = state.lat_e6 / 1e6 if state.lat_e6 != 0 else None
        lon = state.lon_e6 / 1e6 if state.lon_e6 != 0 else None
        advert_pkt = build_advert(
            state.seed, state.pub_key, name=state.name, lat=lat, lon=lon
        )
        return [bytes([RESP_CONTACT_URI]) + advert_pkt]

    # Search stored contacts by prefix
    resolved = _resolve_contact(state, prefix)
    if resolved is None:
        log.warning("companion: EXPORT_CONTACT prefix not found: %s", prefix.hex())
        return [state.build_error(2)]

    full_pub, record = resolved  # 147-byte contact record

    # Unpack fields from record layout:
    # pubkey(32)+node_type(1)+flags(1)+path_len(1)+path(64)+name(32)+
    # last_advert(4)+lat(4)+lon(4)+lastmod(4)
    pubkey = record[0:32]
    node_type = record[32]
    name_raw = record[99:131].rstrip(b"\x00")
    last_adv = struct.unpack_from("<I", record, 131)[0]
    lat_e6 = struct.unpack_from("<i", record, 135)[0]
    lon_e6 = struct.unpack_from("<i", record, 139)[0]

    # Reconstruct app_data for the ADVERT payload
    flags = node_type & 0x0F
    app_data_body = bytearray()
    if lat_e6 != 0 or lon_e6 != 0:
        flags |= ADVERT_HAS_LOCATION
        app_data_body.extend(struct.pack("<ii", lat_e6, lon_e6))
    if name_raw:
        flags |= ADVERT_HAS_NAME
        app_data_body.extend(name_raw)
    app_data = bytes([flags]) + bytes(app_data_body)

    # Build unsigned ADVERT payload: pubkey(32) + ts(4) + sig(64 zeros) + app_data
    advert_payload = pubkey + struct.pack("<I", last_adv) + b"\x00" * 64 + app_data
    wire_pkt = build_wire_packet(
        make_header(ROUTE_FLOOD, PAYLOAD_ADVERT), advert_payload
    )
    log.info(
        "companion: EXPORT_CONTACT %s.. '%s' (unsigned)",
        full_pub.hex()[:8],
        name_raw.decode("utf-8", errors="replace"),
    )
    return [bytes([RESP_CONTACT_URI]) + wire_pkt]


def _ensure_self_contact(state: BridgeState) -> None:
    """Add or update our own identity in the contacts store.

    Called on APP_START so that GET_CONTACTS returns our own contact.
    """
    pk_hex = state.pub_key.hex()
    if pk_hex not in state.contacts:
        record = _build_contact_record(state.pub_key, name=state.name)
        state.contacts[pk_hex] = record
        state.contacts_lastmod = int(time.time())


def _resolve_contact(state: BridgeState, prefix: bytes) -> tuple[bytes, bytes] | None:
    """Resolve a pubkey prefix to (pubkey, record) from contacts.

    Returns (32-byte pubkey, 147-byte contact record) or None if not found.
    """
    for pk_hex, record in state.contacts.items():
        pk_bytes = bytes.fromhex(pk_hex)
        if pk_bytes[: len(prefix)] == prefix:
            return pk_bytes, record
    return None


# ---- Main server loop ----


def run_bridge(
    tcp_port: int,
    udp_host: str,
    udp_port: int,
    state: BridgeState,
) -> None:
    """Run the bridge: TCP server + UDP subscriber."""
    # UDP socket for lora_trx communication
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.bind(("", 0))
    udp_sock.setblocking(False)
    udp_addr = (udp_host, udp_port)

    # Subscribe to lora_trx (sync_word=0x12 for MeshCore)
    sub_msg = cbor2.dumps({"type": "subscribe", "sync_word": [0x12]})
    udp_sock.sendto(sub_msg, udp_addr)
    log.info("subscribed to lora_trx at %s:%d", udp_host, udp_port)

    # TCP server
    tcp_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_srv.bind(("0.0.0.0", tcp_port))
    tcp_srv.listen(1)
    tcp_srv.setblocking(False)
    log.info("listening on TCP port %d", tcp_port)

    sel = selectors.DefaultSelector()
    sel.register(tcp_srv, selectors.EVENT_READ, data="tcp_accept")
    sel.register(udp_sock, selectors.EVENT_READ, data="udp_rx")

    client_sock: socket.socket | None = None
    client_decoder: FrameDecoder | None = None
    last_keepalive = time.monotonic()
    config_shown = False
    last_status_total = -1

    try:
        while True:
            events = sel.select(timeout=2.0)

            # Periodic UDP keepalive
            now = time.monotonic()
            if now - last_keepalive >= 5.0:
                udp_sock.sendto(sub_msg, udp_addr)
                last_keepalive = now

            for key, mask in events:
                # --- TCP accept ---
                if key.data == "tcp_accept":
                    conn, addr = tcp_srv.accept()
                    conn.setblocking(False)
                    log.info("companion connected: %s:%d", addr[0], addr[1])
                    # Single client: close previous connection
                    if client_sock is not None:
                        sel.unregister(client_sock)
                        client_sock.close()
                        log.info("previous client disconnected")
                    client_sock = conn
                    client_decoder = FrameDecoder()
                    sel.register(client_sock, selectors.EVENT_READ, data="tcp_rx")
                    # Reset message queue for new client
                    state.msg_queue.clear()
                    state.msg_seq = 0

                # --- TCP receive ---
                elif key.data == "tcp_rx":
                    try:
                        raw = client_sock.recv(4096)  # type: ignore[union-attr]
                    except ConnectionError, OSError:
                        raw = b""
                    if not raw:
                        log.info("companion disconnected")
                        sel.unregister(client_sock)  # type: ignore[arg-type]
                        client_sock.close()  # type: ignore[union-attr]
                        client_sock = None
                        client_decoder = None
                        continue

                    frames = client_decoder.feed(raw)  # type: ignore[union-attr]
                    for cmd_payload in frames:
                        responses = handle_command(
                            cmd_payload, state, udp_sock, udp_addr
                        )
                        for resp in responses:
                            _tcp_send(client_sock, resp)  # type: ignore[arg-type]

                # --- UDP receive (RX frames from lora_trx) ---
                elif key.data == "udp_rx":
                    try:
                        dgram, _from = udp_sock.recvfrom(65536)
                    except BlockingIOError, OSError:
                        continue
                    try:
                        msg = cbor2.loads(dgram)
                    except Exception:
                        continue

                    if not isinstance(msg, dict):
                        continue

                    msg_type = msg.get("type")

                    # Handle config (log once)
                    if msg_type == "config":
                        if not config_shown:
                            phy = msg.get("phy", {})
                            freq = phy.get("freq", 0)
                            log.info(
                                "config: %.3f MHz SF%s BW %.1fk",
                                freq / 1e6,
                                phy.get("sf", "?"),
                                phy.get("bw", 0) / 1e3,
                            )
                            config_shown = True
                        # Update noise floor from config if provided
                        noise_floor = msg.get("noise_floor_dbm")
                        if noise_floor is not None:
                            state.noise_floor_dbm = int(noise_floor)
                        continue

                    # Handle status (log only when frame count changes)
                    if msg_type == "status":
                        frames_info = msg.get("frames", {})
                        total = frames_info.get("total", 0)
                        if total != last_status_total:
                            ok = frames_info.get("crc_ok", 0)
                            log.info("status: %d frames (%d CRC_OK)", total, ok)
                            last_status_total = total
                        continue

                    if msg_type != "lora_frame":
                        continue

                    # Expire old TX echo hashes
                    now = time.monotonic()
                    cutoff = now - state.recent_tx_ttl
                    expired = [k for k, ts in state.recent_tx.items() if ts < cutoff]
                    for k in expired:
                        log.debug("echo-filter: expired hash %s", k.hex())
                        del state.recent_tx[k]

                    # Filter our own TX echoes
                    rx_payload = msg.get("payload", b"")
                    if rx_payload:
                        h = _hash_payload(rx_payload)
                        if h in state.recent_tx:
                            log.debug(
                                "echo-filter: matched TX hash %s, dropping echo",
                                h.hex(),
                            )
                            del state.recent_tx[h]
                            continue
                        log.debug(
                            "echo-filter: RX hash %s (no match, %d tracked)",
                            h.hex(),
                            len(state.recent_tx),
                        )

                    # RX dedup: suppress duplicate frames from dual-channel RX
                    # or mesh retransmits (same payload within recent_rx_ttl)
                    if rx_payload:
                        rx_h = _hash_payload(rx_payload)
                        now_rx = time.monotonic()
                        # Expire old entries
                        rx_expired = [
                            k
                            for k, ts in state.recent_rx.items()
                            if now_rx - ts > state.recent_rx_ttl
                        ]
                        for k in rx_expired:
                            del state.recent_rx[k]
                        if rx_h in state.recent_rx:
                            log.debug("dedup: suppressed duplicate RX %s", rx_h.hex())
                            continue
                        state.recent_rx[rx_h] = now_rx

                    # Convert to companion messages and queue
                    companion_msgs, ack_packets = lora_frame_to_companion_msgs(
                        msg, state
                    )

                    # TX RF ACK packets (direct routing, no path)
                    if ack_packets:
                        from meshcore_tx import make_cbor_tx_request

                        for ack_pkt in ack_packets:
                            cbor_msg = make_cbor_tx_request(ack_pkt)
                            udp_sock.sendto(cbor_msg, udp_addr)
                            log.info("TX ACK: %s (%dB)", ack_pkt.hex(), len(ack_pkt))

                    # Packet forwarding: when client_repeat is enabled, re-transmit
                    # the raw received wire packet back to the RF medium (repeater mode).
                    # Excluded: packets we sent ourselves (already filtered by recent_tx
                    # echo check above) and zero-length payloads.
                    if state.client_repeat and rx_payload:
                        from meshcore_tx import make_cbor_tx_request

                        fwd_cbor = make_cbor_tx_request(rx_payload)
                        udp_sock.sendto(fwd_cbor, udp_addr)
                        log.info("repeat: forwarded %dB", len(rx_payload))

                    for companion_msg in companion_msgs:
                        # ACK pushes (PUSH_ACK) are sent immediately, not queued
                        if companion_msg[0] == PUSH_ACK:
                            if client_sock is not None:
                                _tcp_send(client_sock, companion_msg)
                        else:
                            state.msg_queue.append(companion_msg)
                    if companion_msgs:
                        # Push MESSAGES_WAITING if we queued any non-push msgs
                        queued = [m for m in companion_msgs if m[0] != PUSH_ACK]
                        if queued and client_sock is not None:
                            _tcp_send(
                                client_sock,
                                bytes([PUSH_MESSAGES_WAITING]),
                            )
                        seq = msg.get("seq", "?")
                        plen = len(rx_payload)
                        log.info(
                            "RX #%s: %dB (queue %d)",
                            seq,
                            plen,
                            len(state.msg_queue),
                        )

    except KeyboardInterrupt:
        pass
    finally:
        if client_sock is not None:
            client_sock.close()
        tcp_srv.close()
        udp_sock.close()
        sel.close()
        log.info("bridge stopped")


def _tcp_send(sock: socket.socket, payload: bytes) -> None:
    """Send a framed companion protocol response."""
    try:
        sock.sendall(frame_encode(payload))
    except ConnectionError, OSError:
        pass  # client disconnected, will be cleaned up on next recv


# ---- CLI ----


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MeshCore companion protocol bridge to lora_trx"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help=f"TCP listen port (overrides config.toml; default: {BRIDGE_PORT})",
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=None,
        help="lora_trx UDP address (default from config.toml or 127.0.0.1:5555)",
    )
    parser.add_argument(
        "--identity",
        type=Path,
        default=None,
        help=f"Identity file (overrides config.toml; default: {DEFAULT_IDENTITY_FILE})",
    )
    parser.add_argument(
        "--name",
        default=None,
        help="Node name for SELF_INFO (overrides config.toml; default: gr4-lora)",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to config.toml (auto-detected if omitted)",
    )
    parser.add_argument(
        "--keys-dir",
        type=Path,
        default=None,
        help=f"Key store directory (overrides config.toml; default: {DEFAULT_KEYS_DIR})",
    )
    parser.add_argument(
        "--channels-dir",
        type=Path,
        default=None,
        help=f"Channel store directory (overrides config.toml; default: {DEFAULT_CHANNELS_DIR})",
    )
    parser.add_argument(
        "--contacts-dir",
        type=Path,
        default=None,
        help=f"Contact store directory (overrides config.toml; default: {DEFAULT_CONTACTS_DIR})",
    )
    parser.add_argument(
        "--region-scope",
        metavar="NAME",
        default=None,
        help="Region scope for transport codes (e.g. 'de-nord'). "
        "Default from config.toml or empty (disabled).",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging (echo filter diagnostics, etc.)",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output (also: NO_COLOR env var)",
    )
    args = parser.parse_args()

    # Load config and configure logging
    cfg = load_config(args.config)
    setup_logging("gr4.bridge", cfg, debug=args.debug, no_color=args.no_color)

    # Resolve UDP address — prefer lora_agg consumer port when configured
    if args.connect:
        try:
            udp_host, udp_port = parse_host_port(args.connect)
        except ValueError as exc:
            parser.error(str(exc))
    elif cfg.get("aggregator"):
        udp_host, udp_port = parse_host_port(config_agg_listen(cfg))
    else:
        udp_host = config_udp_host(cfg)
        udp_port = config_udp_port(cfg)

    # Extract PHY params from config for SELF_INFO (identity loaded after config resolution)
    # Find first set and resolve codec + radio
    freq_mhz = 869.618
    bw_khz = 62.5
    sf = 8
    cr = 4
    tx_power = 14

    for key, val in cfg.items():
        if key.startswith("set_") and isinstance(val, dict):
            codec_name = val.get("codec", "")
            radio_name = val.get("radio", "")
            codec = cfg.get(codec_name, {})
            radio = cfg.get(radio_name, {})
            if codec:
                # sf is now in [[set_*.decode]] entries; fall back to codec for compat
                decode_entries = val.get("decode", [])
                sf = (
                    decode_entries[0].get("sf", codec.get("sf", sf))
                    if decode_entries
                    else codec.get("sf", sf)
                )
                bw_khz = codec.get("bw", bw_khz * 1000) / 1000
                cr = codec.get("cr", cr + 4) - 4  # denominator -> offset
            if radio:
                freq_mhz = radio.get("freq", freq_mhz * 1e6) / 1e6
                tx_power = int(radio.get("tx_gain", tx_power))
            break

    # Resolve region scope: CLI flag overrides config.toml
    if args.region_scope is not None:
        region_scope = args.region_scope
    else:
        region_scope = config_region_scope(cfg)
    if region_scope:
        log.info("region scope: #%s", region_scope)

    # Read optional settings from [meshcore] config section.
    # Resolution order: CLI flag > config.toml [meshcore] > built-in default.
    meshcore_cfg = cfg.get("meshcore", {})
    lat_e6 = int(float(meshcore_cfg.get("lat", 0.0)) * 1e6)
    lon_e6 = int(float(meshcore_cfg.get("lon", 0.0)) * 1e6)

    # TCP port
    if args.port is not None:
        tcp_port = args.port
    else:
        tcp_port = int(meshcore_cfg.get("port", BRIDGE_PORT))

    # Node name (SET_ADVERT_NAME equivalent)
    if args.name is not None:
        node_name = args.name
    else:
        node_name = meshcore_cfg.get("name", "lora_trx")

    # Device model string reported in DEVICE_INFO (CMD_DEVICE_QUERY response)
    node_model = meshcore_cfg.get("model", "lora_trx")

    # Identity file
    if args.identity is not None:
        identity_file = args.identity
    else:
        identity_file = Path(meshcore_cfg.get("identity_file", DEFAULT_IDENTITY_FILE))

    # Store directories
    if args.keys_dir is not None:
        keys_dir = args.keys_dir
    else:
        keys_dir = Path(meshcore_cfg.get("keys_dir", DEFAULT_KEYS_DIR))

    if args.channels_dir is not None:
        channels_dir = args.channels_dir
    else:
        channels_dir = Path(meshcore_cfg.get("channels_dir", DEFAULT_CHANNELS_DIR))

    if args.contacts_dir is not None:
        contacts_dir = args.contacts_dir
    else:
        contacts_dir = Path(meshcore_cfg.get("contacts_dir", DEFAULT_CONTACTS_DIR))

    # Repeat mode settings (config.toml [meshcore] only, no CLI flags)
    startup_client_repeat = int(meshcore_cfg.get("client_repeat", 0))
    startup_path_hash_mode = int(meshcore_cfg.get("path_hash_mode", 0))
    startup_autoadd_config = int(meshcore_cfg.get("autoadd_config", 0))
    startup_autoadd_max_hops = min(int(meshcore_cfg.get("autoadd_max_hops", 0)), 64)
    flood_scope_raw = meshcore_cfg.get("flood_scope", "")
    startup_send_scope = _parse_flood_scope(flood_scope_raw)

    # Load identity (now that identity_file is resolved)
    expanded_prv, pub_key, seed = load_or_create_identity(identity_file)
    log.info("identity: %s", pub_key.hex())
    log.info("name: %s", node_name)

    # Print contact URIs (two incompatible formats — see CLAUDE.md)
    from lora_common import build_bizcard_uri, build_contact_uri
    from meshcore_tx import build_advert

    advert_pkt = build_advert(seed, pub_key, name=node_name)
    # Phone companion app format (QR / paste into app)
    contact_uri = build_contact_uri(pub_key, node_name)
    log.info("contact URI: %s", contact_uri)
    # CLI biz card format (meshcore-cli import_contact)
    biz_uri = build_bizcard_uri(advert_pkt)
    log.info("biz card: %s", biz_uri)

    state = BridgeState(
        pub_key=pub_key,
        expanded_prv=expanded_prv,
        seed=seed,
        name=node_name,
        freq_mhz=freq_mhz,
        bw_khz=bw_khz,
        sf=sf,
        cr=cr,
        tx_power=tx_power,
        keys_dir=keys_dir,
        channels_dir=channels_dir,
        contacts_dir=contacts_dir,
        region_scope=region_scope,
        lat_e6=lat_e6,
        lon_e6=lon_e6,
        client_repeat=startup_client_repeat,
        path_hash_mode=startup_path_hash_mode,
        send_scope=startup_send_scope,
        autoadd_config=startup_autoadd_config,
        autoadd_max_hops=startup_autoadd_max_hops,
        model=node_model,
    )

    # Load persisted contacts from disk
    persisted = _load_persisted_contacts(contacts_dir)
    if persisted:
        state.contacts.update(persisted)
        state.contacts_lastmod = int(time.time())
        log.info("loaded %d persisted contacts", len(persisted))

    # Log loaded state
    log.info(
        "keys: %d, channels: %d, contacts: %d",
        len(state.known_keys),
        len(state.channels),
        len(state.contacts),
    )

    run_bridge(tcp_port, udp_host, udp_port, state)


if __name__ == "__main__":
    main()
