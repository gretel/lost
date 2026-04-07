#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_decode_lorawan.py -- LoRaWAN MAC header decoder.

Reads concatenated CBOR frames on stdin. Filters for frames with
sync_word=0x34 (LoRaWAN) and parses the cleartext MAC header fields:
MType, DevAddr, FCnt, FCtrl, FPort, and MIC.

LoRaWAN payloads are AES-128 encrypted (FRMPayload) with CMAC integrity
(MIC). This decoder shows the unencrypted header fields only.

LoRaWAN MAC frame format (LoRaWAN L2 1.0.4 spec, section 4):

    MHDR(1) | MACPayload | MIC(4)

    MHDR byte: MType[7:5] | RFU[4:2] | Major[1:0]

    MACPayload (for data frames):
        FHDR          | FPort(1)? | FRMPayload(N)?
        DevAddr(4,LE) | FCtrl(1) | FCnt(2,LE) | FOpts(0..15)

    FCtrl (uplink):   ADR[7] | ADRACKReq[6] | ACK[5] | ClassB[4] | FOptsLen[3:0]
    FCtrl (downlink): ADR[7] | RFU[6]       | ACK[5] | FPending[4] | FOptsLen[3:0]

Note: for live monitoring, use lora_mon.py (reads UDP from lora_trx).
This script is for offline analysis of saved CBOR streams.

Usage:
    python3 scripts/lora_decode_lorawan.py < captured.cbor
    python3 scripts/lora_decode_lorawan.py --json < captured.cbor
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))

from cbor_stream import read_cbor_seq
from lora_common import format_hex

LORAWAN_SYNC_WORD = 0x34

MTYPE_NAMES = [
    "JoinRequest",  # 0
    "JoinAccept",  # 1
    "UnconfDataUp",  # 2
    "UnconfDataDown",  # 3
    "ConfDataUp",  # 4
    "ConfDataDown",  # 5
    "RFU",  # 6
    "Proprietary",  # 7
]

MAJOR_NAMES = {0: "LoRaWAN R1"}


# ---- LoRaWAN MAC header parsing ----


@dataclass
class LoRaWANFrame:
    """Parsed LoRaWAN MAC frame (cleartext header fields only)."""

    # MHDR
    mtype: int = 0
    rfu: int = 0
    major: int = 0

    # FHDR (data frames only)
    dev_addr: int = 0
    fctrl: int = 0
    fcnt: int = 0
    fopts: bytes = b""

    # FPort (present when FRMPayload exists)
    fport: int | None = None

    # FRMPayload (encrypted, shown as hex)
    frm_payload: bytes = b""

    # MIC (last 4 bytes)
    mic: int = 0

    # Join frames
    join_eui: int = 0  # JoinRequest: AppEUI/JoinEUI (8 bytes LE)
    dev_eui: int = 0  # JoinRequest: DevEUI (8 bytes LE)
    dev_nonce: int = 0  # JoinRequest: DevNonce (2 bytes LE)

    @property
    def mtype_name(self) -> str:
        return (
            MTYPE_NAMES[self.mtype]
            if self.mtype < len(MTYPE_NAMES)
            else f"?{self.mtype}"
        )

    @property
    def major_name(self) -> str:
        return MAJOR_NAMES.get(self.major, f"?{self.major}")

    @property
    def is_uplink(self) -> bool:
        return self.mtype in (0, 2, 4)  # JoinReq, UnconfUp, ConfUp

    @property
    def is_data(self) -> bool:
        return self.mtype in (2, 3, 4, 5)

    @property
    def is_join(self) -> bool:
        return self.mtype in (0, 1)

    # FCtrl decoded fields
    @property
    def adr(self) -> bool:
        return bool(self.fctrl & 0x80)

    @property
    def ack(self) -> bool:
        return bool(self.fctrl & 0x20)

    @property
    def fopts_len(self) -> int:
        return self.fctrl & 0x0F

    @property
    def adr_ack_req(self) -> bool:
        """Uplink only."""
        return bool(self.fctrl & 0x40)

    @property
    def class_b(self) -> bool:
        """Uplink only."""
        return bool(self.fctrl & 0x10)

    @property
    def fpending(self) -> bool:
        """Downlink only."""
        return bool(self.fctrl & 0x10)


def parse_lorawan(data: bytes) -> LoRaWANFrame | None:
    """Parse LoRaWAN MAC frame from raw LoRa payload bytes."""
    if len(data) < 5:  # MHDR(1) + MIC(4) minimum
        return None

    frame = LoRaWANFrame()

    # MHDR
    mhdr = data[0]
    frame.mtype = (mhdr >> 5) & 0x07
    frame.rfu = (mhdr >> 2) & 0x07
    frame.major = mhdr & 0x03

    # MIC (last 4 bytes, little-endian)
    frame.mic = struct.unpack_from("<I", data, len(data) - 4)[0]

    mac_payload = data[1:-4]

    if frame.mtype == 0:  # JoinRequest
        # JoinEUI(8) + DevEUI(8) + DevNonce(2) = 18 bytes
        if len(mac_payload) >= 18:
            frame.join_eui = struct.unpack_from("<Q", mac_payload, 0)[0]
            frame.dev_eui = struct.unpack_from("<Q", mac_payload, 8)[0]
            frame.dev_nonce = struct.unpack_from("<H", mac_payload, 16)[0]
        return frame

    if frame.mtype == 1:  # JoinAccept
        # JoinAccept payload is encrypted (with NwkKey). We can't decode further.
        frame.frm_payload = mac_payload
        return frame

    if not frame.is_data:
        # RFU or Proprietary
        frame.frm_payload = mac_payload
        return frame

    # Data frame: FHDR | FPort? | FRMPayload?
    # FHDR = DevAddr(4) + FCtrl(1) + FCnt(2) + FOpts(0..15)
    if len(mac_payload) < 7:  # minimum FHDR
        return frame

    frame.dev_addr = struct.unpack_from("<I", mac_payload, 0)[0]
    frame.fctrl = mac_payload[4]
    frame.fcnt = struct.unpack_from("<H", mac_payload, 5)[0]

    fopts_len = frame.fopts_len
    fhdr_len = 7 + fopts_len

    if fhdr_len > len(mac_payload):
        return frame

    frame.fopts = mac_payload[7:fhdr_len]

    remaining = mac_payload[fhdr_len:]
    if remaining:
        frame.fport = remaining[0]
        frame.frm_payload = remaining[1:]

    return frame


# ---- Output formatters ----


def format_dev_addr(addr: int) -> str:
    """Format DevAddr as 4-byte hex (network byte order display)."""
    return f"{addr:08X}"


def format_eui(eui: int) -> str:
    """Format EUI-64 as colon-separated hex."""
    b = eui.to_bytes(8, "little")
    return ":".join(f"{x:02X}" for x in reversed(b))


def print_text(msg: dict[str, Any], frame: LoRaWANFrame) -> None:
    """Print LoRaWAN frame in human-readable text format."""
    phy = msg.get("phy", {})
    crc = "CRC_OK" if msg.get("crc_valid", False) else "CRC_FAIL"
    dc = "  (downchirp)" if msg.get("is_downchirp", False) else ""

    print(
        f"[{msg.get('ts', '')}] #{msg.get('seq', 0)}  "
        f"{msg.get('payload_len', 0)} bytes  "
        f"SF{phy.get('sf', '?')} BW{phy.get('bw', '?')} CR4/{4 + phy.get('cr', 0)}  "
        f"{crc}{dc}"
    )

    direction = "UP" if frame.is_uplink else "DOWN"
    print(f"  LoRaWAN {frame.major_name}: {frame.mtype_name} ({direction})")

    if frame.mtype == 0:  # JoinRequest
        print(f"  JoinEUI:  {format_eui(frame.join_eui)}")
        print(f"  DevEUI:   {format_eui(frame.dev_eui)}")
        print(f"  DevNonce: {frame.dev_nonce}")
    elif frame.mtype == 1:  # JoinAccept
        print("  (JoinAccept payload is encrypted)")
    elif frame.is_data:
        flags = []
        if frame.adr:
            flags.append("ADR")
        if frame.ack:
            flags.append("ACK")
        if frame.is_uplink and frame.adr_ack_req:
            flags.append("ADRACKReq")
        if frame.is_uplink and frame.class_b:
            flags.append("ClassB")
        if not frame.is_uplink and frame.fpending:
            flags.append("FPending")
        flags_str = " [" + ",".join(flags) + "]" if flags else ""

        print(f"  DevAddr:  {format_dev_addr(frame.dev_addr)}{flags_str}")
        print(f"  FCnt:     {frame.fcnt}")
        if frame.fport is not None:
            fport_label = ""
            if frame.fport == 0:
                fport_label = " (MAC commands)"
            elif frame.fport == 224:
                fport_label = " (LoRaWAN test)"
            print(f"  FPort:    {frame.fport}{fport_label}")
        if frame.fopts:
            print(f"  FOpts:    {format_hex(frame.fopts)}")
        if frame.frm_payload:
            print(f"  Payload:  {format_hex(frame.frm_payload[:64])} (encrypted)")
            if len(frame.frm_payload) > 64:
                print(f"            ... ({len(frame.frm_payload)} bytes total)")

    print(f"  MIC:      {frame.mic:08X}")
    print()


def print_json(msg: dict[str, Any], frame: LoRaWANFrame) -> None:
    """Print LoRaWAN frame as a JSON line."""
    out: dict[str, Any] = {
        "seq": msg.get("seq", 0),
        "ts": msg.get("ts", ""),
        "crc_valid": msg.get("crc_valid", False),
        "is_downchirp": msg.get("is_downchirp", False),
        "lorawan": {
            "mtype": frame.mtype,
            "mtype_name": frame.mtype_name,
            "major": frame.major,
            "direction": "up" if frame.is_uplink else "down",
            "mic": f"{frame.mic:08X}",
        },
    }

    lw = out["lorawan"]
    if frame.mtype == 0:  # JoinRequest
        lw["join_eui"] = format_eui(frame.join_eui)
        lw["dev_eui"] = format_eui(frame.dev_eui)
        lw["dev_nonce"] = frame.dev_nonce
    elif frame.is_data:
        lw["dev_addr"] = format_dev_addr(frame.dev_addr)
        lw["fcnt"] = frame.fcnt
        lw["fctrl"] = {
            "adr": frame.adr,
            "ack": frame.ack,
            "fopts_len": frame.fopts_len,
        }
        if frame.is_uplink:
            lw["fctrl"]["adr_ack_req"] = frame.adr_ack_req
            lw["fctrl"]["class_b"] = frame.class_b
        else:
            lw["fctrl"]["fpending"] = frame.fpending
        if frame.fport is not None:
            lw["fport"] = frame.fport
        if frame.fopts:
            lw["fopts_hex"] = format_hex(frame.fopts, sep="")
        if frame.frm_payload:
            lw["frm_payload_hex"] = format_hex(frame.frm_payload, sep="")
            lw["frm_payload_len"] = len(frame.frm_payload)

    print(json.dumps(out))


def main() -> None:
    parser = argparse.ArgumentParser(description="LoRaWAN MAC header decoder")
    parser.add_argument(
        "--json", action="store_true", help="output JSON lines instead of text"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="also show non-LoRaWAN frames (passthrough)",
    )
    args = parser.parse_args()

    try:
        for msg in read_cbor_seq(sys.stdin.buffer):
            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            phy = msg.get("phy", {})
            sync_word = phy.get("sync_word", 0)
            if sync_word != LORAWAN_SYNC_WORD:
                if args.all:
                    payload = msg.get("payload", b"")
                    crc = "CRC_OK" if msg.get("crc_valid", False) else "CRC_FAIL"
                    sw = phy.get("sync_word", 0)
                    print(
                        f"[{msg.get('ts', '')}] #{msg.get('seq', 0)}  "
                        f"{len(payload)} bytes  {crc}  [sync=0x{sw:02X}]"
                    )
                    print(f"  Hex: {format_hex(payload[:64])}")
                    print()
                continue

            payload = msg.get("payload", b"")
            frame = parse_lorawan(payload)
            if frame is None:
                continue

            if args.json:
                print_json(msg, frame)
            else:
                print_text(msg, frame)

            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        pass


if __name__ == "__main__":
    main()
