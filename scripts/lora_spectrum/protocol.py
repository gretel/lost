#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""CBOR protocol handlers for lora_spectrum."""

from __future__ import annotations

import logging
import struct
import time
from typing import Any, Callable

import cbor2

logger = logging.getLogger(__name__)


class ProtocolHandler:
    """Handles CBOR message parsing and dispatch.

    Maps CBOR message types to handler callbacks.
    """

    def __init__(self) -> None:
        """Initialize handler with empty dispatch table."""
        self._handlers: dict[str, Callable[[dict[str, Any]], None]] = {}

    def register(
        self, msg_type: str, handler: Callable[[dict[str, Any]], None]
    ) -> None:
        """Register a handler for a specific message type."""
        self._handlers[msg_type] = handler
        logger.debug(f"Registered handler for {msg_type}")

    def unregister(self, msg_type: str) -> None:
        """Unregister a handler."""
        if msg_type in self._handlers:
            del self._handlers[msg_type]

    def handle(self, data: bytes) -> str | None:
        """Parse and dispatch a CBOR message.

        Args:
            data: Raw CBOR-encoded bytes

        Returns:
            Message type if handled, None if parsing failed
        """
        try:
            msg = cbor2.loads(data)
        except Exception as e:
            logger.debug(f"Failed to parse CBOR: {e}")
            return None

        if not isinstance(msg, dict):
            logger.debug(f"Expected dict, got {type(msg).__name__}")
            return None

        msg_type = msg.get("type", "")
        if not msg_type:
            logger.debug("Message missing 'type' field")
            return None

        handler = self._handlers.get(msg_type)
        if handler:
            try:
                handler(msg)
            except Exception as e:
                logger.warning(f"Handler for {msg_type} failed: {e}")

        return msg_type


class SpectrumData:
    """Parsed spectrum data from scan_spectrum message."""

    def __init__(self, msg: dict[str, Any]) -> None:
        """Parse scan_spectrum message."""
        self.n_channels: int = msg.get("n_channels", 0)
        self.freq_min: int = msg.get("freq_min", 0)
        self.freq_step: int = msg.get("freq_step", 62500)
        self.sweep: int = msg.get("sweep", 0)

        # Parse channel energy data
        channels = msg.get("channels", b"")
        if isinstance(channels, bytes) and len(channels) >= self.n_channels * 4:
            self.raw_energy = list(struct.unpack(f"<{self.n_channels}f", channels))
        else:
            self.raw_energy = [0.0] * self.n_channels

        # Parse hot channels
        self.hot: set[int] = set(msg.get("hot", []))

        # Parse detections
        self.detections: list[dict[str, Any]] = msg.get("detections", [])

        # Timestamp for rate calculations
        self.timestamp = time.monotonic()

    @property
    def freq_max(self) -> int:
        """Calculate maximum frequency."""
        return self.freq_min + self.freq_step * self.n_channels


class SweepEndData:
    """Parsed sweep end data from scan_sweep_end message."""

    def __init__(self, msg: dict[str, Any]) -> None:
        """Parse scan_sweep_end message."""
        self.sweep: int = msg.get("sweep", 0)
        self.duration_ms: int = msg.get("duration_ms", 0)
        self.l1_snapshots: int = msg.get("l1_snapshots", 0)
        self.hot_count: int = msg.get("hot_count", 0)
        self.detections: int = msg.get("detections", 0)
        self.overflows: int = msg.get("overflows", 0)


class StatusData:
    """Parsed status data from scan_status message."""

    def __init__(self, msg: dict[str, Any]) -> None:
        """Parse scan_status message."""
        self.mode: str = msg.get("mode", "unknown")
        self.sweeps: int = msg.get("sweeps", 0)
        self.total_det: int = msg.get("detections", 0)
        self.overflows: int = msg.get("overflows", 0)
        self.dropouts: int = msg.get("dropouts", 0)


def create_default_handler(
    on_spectrum: Callable[[SpectrumData], None] | None = None,
    on_sweep_end: Callable[[SweepEndData], None] | None = None,
    on_status: Callable[[StatusData], None] | None = None,
) -> ProtocolHandler:
    """Create a protocol handler with common callbacks."""
    handler = ProtocolHandler()

    def spectrum_wrapper(msg: dict[str, Any]) -> None:
        if on_spectrum:
            on_spectrum(SpectrumData(msg))

    def sweep_end_wrapper(msg: dict[str, Any]) -> None:
        if on_sweep_end:
            on_sweep_end(SweepEndData(msg))

    def status_wrapper(msg: dict[str, Any]) -> None:
        if on_status:
            on_status(StatusData(msg))

    handler.register("scan_spectrum", spectrum_wrapper)
    handler.register("scan_sweep_end", sweep_end_wrapper)
    handler.register("scan_status", status_wrapper)
    handler.register("scan_sweep_start", lambda msg: None)
    handler.register("scan_l2_probe", lambda msg: None)
    handler.register("scan_overflow", lambda msg: None)

    return handler
