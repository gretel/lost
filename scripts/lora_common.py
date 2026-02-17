#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_common.py -- Shared constants and helpers for LoRa decoder scripts.
"""

from __future__ import annotations

# ---- Protocol constants ----

SYNC_NAMES: dict[int, str] = {
    0x12: "MeshCore/Reticulum",
    0x2B: "Meshtastic",
    0x34: "LoRaWAN",
}

ROUTE_NAMES: list[str] = ["T_FLOOD", "FLOOD", "DIRECT", "T_DIRECT"]

PAYLOAD_NAMES: list[str] = [
    "REQ",
    "RESP",
    "TXT",
    "ACK",
    "ADVERT",
    "GRP_TXT",
    "GRP_DATA",
    "ANON_REQ",
    "PATH",
    "TRACE",
    "MULTI",
    "CTRL",
    "rsv12",
    "rsv13",
    "rsv14",
    "RAW_CUSTOM",
]


# ---- Formatting helpers ----


def sync_word_name(sw: int) -> str:
    """Map sync word to a human-readable protocol name."""
    return SYNC_NAMES.get(sw, f"0x{sw:02X}")


def format_hex(data: bytes, *, sep: str = " ", max_bytes: int | None = None) -> str:
    """Format bytes as uppercase hex with optional truncation."""
    if max_bytes is not None:
        preview = sep.join(f"{b:02X}" for b in data[:max_bytes])
        if len(data) > max_bytes:
            preview += f" ...({len(data)}B)"
        return preview
    return sep.join(f"{b:02X}" for b in data)


def format_ascii(data: bytes, *, max_bytes: int | None = None) -> str:
    """Format bytes as printable ASCII (replace non-printable with '.')."""
    if max_bytes is not None:
        s = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in data[:max_bytes])
        if len(data) > max_bytes:
            s += "..."
        return s
    return "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in data)
