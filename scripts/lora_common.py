#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_common.py -- Shared constants and helpers for LoRa decoder scripts.
"""

from __future__ import annotations

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


def sync_word_name(sw: int) -> str:
    """Format sync word as hex string."""
    return f"0x{sw:02X}"


def format_hex(data: bytes, *, sep: str = " ", max_bytes: int | None = None) -> str:
    """Format bytes as uppercase hex with optional truncation."""
    if max_bytes is not None:
        preview = sep.join(f"{b:02X}" for b in data[:max_bytes])
        if len(data) > max_bytes:
            preview += f" ...({len(data)}B)"
        return preview
    return sep.join(f"{b:02X}" for b in data)


def format_ascii(data: bytes, *, max_bytes: int | None = None) -> str:
    """Decode as UTF-8 where valid, replace invalid bytes with hex escapes."""
    raw = data[:max_bytes] if max_bytes is not None else data
    parts: list[str] = []
    i = 0
    while i < len(raw):
        b = raw[i]
        if b < 0x20 and b not in (0x09, 0x0A):
            parts.append(f"\\x{b:02x}")
            i += 1
        elif b < 0x80:
            parts.append(chr(b))
            i += 1
        else:
            # Try to decode a multi-byte UTF-8 sequence
            for length in (4, 3, 2):
                if i + length <= len(raw):
                    try:
                        parts.append(raw[i : i + length].decode("utf-8"))
                        i += length
                        break
                    except UnicodeDecodeError:
                        continue
            else:
                parts.append(f"\\x{b:02x}")
                i += 1
    s = "".join(parts)
    if max_bytes is not None and len(data) > max_bytes:
        s += "..."
    return s
