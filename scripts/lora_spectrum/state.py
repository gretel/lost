#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""State management for lora_spectrum."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .config import SpectrumConfig
    from .protocol import SpectrumData, SweepEndData, StatusData

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """A single detection record."""

    freq: int  # Hz
    sf: int
    bw: int  # Hz
    ratio: float
    chirp: str
    timestamp: float  # monotonic
    sync_word: int | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any], timestamp: float) -> "Detection":
        """Create from CBOR detection sub-map."""
        return cls(
            freq=int(data.get("freq", 0)),
            sf=int(data.get("sf", 0)),
            bw=int(data.get("bw", 0)),
            ratio=float(data.get("ratio", 0.0)),
            chirp=str(data.get("chirp", "up")),
            sync_word=data.get("sync_word"),  # Optional field
            timestamp=timestamp,
        )


@dataclass
class PeakHold:
    """A peak hold marker."""

    channel: int
    db_value: float
    timestamp: float
    label: str
    label_color: str


class SpectrumState:
    """Maintains spectrum display state.

    Handles:
    - EMA smoothing of spectrum energy
    - Detection history and active detections
    - Peak hold markers
    - Sweep timing statistics
    """

    def __init__(self, config: "SpectrumConfig") -> None:
        """Initialize state with configuration."""
        self._config = config

        # Spectrum data
        self.energy: list[float] | None = None  # EMA-smoothed
        self.raw_energy: list[float] | None = None  # Unsmoothed (for bar heights)
        self.freq_min: int = 0
        self.freq_step: int = 62500
        self.n_channels: int = 0
        self.hot_channels: set[int] = set()

        # Sweep tracking
        self.sweep_count: int = 0
        self.last_sweep_duration_ms: int = 0
        self.sweep_rate: float = 0.0
        self._prev_spectrum_time: float = 0.0
        self._prev_sweep_count: int = 0

        # Detection tracking
        self.active_detections: list[tuple[float, Detection]] = []
        self.detection_history: list[tuple[float, int, Detection]] = []
        self.total_detections: int = 0

        # Peak holds
        self.peak_holds: list[PeakHold] = []

        # Status
        self.mode: str = "unknown"
        self.overflows: int = 0
        self.dropouts: int = 0

        logger.debug("SpectrumState initialized")

    def on_spectrum(self, data: "SpectrumData") -> None:
        """Process new spectrum data."""
        # Update frequency parameters
        self.freq_min = data.freq_min
        self.freq_step = data.freq_step
        self.n_channels = data.n_channels
        self.hot_channels = data.hot
        self.sweep_count = data.sweep

        # Store raw energy (for bar heights)
        self.raw_energy = list(data.raw_energy)

        # Apply EMA smoothing if enabled
        if self._config.enable_ema:
            if self.energy is None or len(self.energy) != self.n_channels:
                self.energy = list(data.raw_energy)
            else:
                alpha = self._config.ema_alpha
                self.energy = [
                    alpha * r + (1 - alpha) * e
                    for r, e in zip(data.raw_energy, self.energy)
                ]
        else:
            self.energy = list(data.raw_energy)

        # Process detections
        mono_now = time.monotonic()
        for det_data in data.detections:
            det = Detection.from_dict(det_data, mono_now)
            self.active_detections.append((mono_now, det))
            self.detection_history.append((time.time(), data.sweep, det))
            self.total_detections += 1

            # Add peak hold at detection location
            self._add_peak_hold(det, data, mono_now)

        # Prune old detections
        self._prune_detections(mono_now)

        # Prune old peak holds
        self._prune_peak_holds(mono_now)

        # Limit history size
        if len(self.detection_history) > self._config.max_history:
            self.detection_history = self.detection_history[-self._config.max_history :]

        # Update sweep rate
        self._update_sweep_rate(data.timestamp)

    def on_sweep_end(self, data: "SweepEndData") -> None:
        """Process sweep end data."""
        self.last_sweep_duration_ms = data.duration_ms
        self.sweep_count = data.sweep
        self.overflows = data.overflows
        logger.debug(f"Sweep {data.sweep} completed in {data.duration_ms}ms")

    def on_status(self, data: "StatusData") -> None:
        """Process status data."""
        self.mode = data.mode
        self.sweep_count = data.sweeps
        self.total_detections = data.total_det
        self.overflows = data.overflows
        self.dropouts = data.dropouts

    def _add_peak_hold(
        self, det: Detection, data: "SpectrumData", timestamp: float
    ) -> None:
        """Add a peak hold marker for a detection."""
        if not self.raw_energy or self.n_channels == 0:
            return

        # Calculate channel index from frequency
        if self.freq_step > 0 and det.freq > 0:
            ch = int((det.freq - self.freq_min) / self.freq_step + 0.5)
            ch = max(0, min(ch, self.n_channels - 1))
        else:
            ch = 0

        # Get dB value at that channel
        db_value = 10.0 * __import__("math").log10(max(self.raw_energy[ch], 1e-12))

        # Build label
        parts = [f"r{det.ratio:.0f}"]
        # Check for sync_word in detection data
        if hasattr(det, "sync_word") and det.sync_word is not None:
            parts.append(f"0x{det.sync_word:02X}")
        label = " ".join(parts)

        # Color based on ratio
        if det.ratio >= 50:
            color = "\033[38;5;46m"  # green
        elif det.ratio >= 20:
            color = "\033[38;5;226m"  # yellow
        else:
            color = "\033[38;5;214m"  # orange

        self.peak_holds.append(
            PeakHold(
                channel=ch,
                db_value=db_value,
                timestamp=timestamp,
                label=label,
                label_color=color,
            )
        )

    def _prune_detections(self, now: float) -> None:
        """Remove old detections beyond TTL."""
        ttl = self._config.anchor_ttl
        self.active_detections = [
            (t, d) for t, d in self.active_detections if now - t < ttl
        ]

    def _prune_peak_holds(self, now: float) -> None:
        """Remove old peak holds."""
        hold_time = self._config.peak_hold_s
        self.peak_holds = [
            ph for ph in self.peak_holds if now - ph.timestamp < hold_time
        ]

    def _update_sweep_rate(self, timestamp: float) -> None:
        """Calculate sweep rate from timestamps."""
        if self._prev_spectrum_time > 0:
            dt = timestamp - self._prev_spectrum_time
            ds = self.sweep_count - self._prev_sweep_count
            if dt > 0.1 and ds > 0:
                self.sweep_rate = ds / dt

        self._prev_spectrum_time = timestamp
        self._prev_sweep_count = self.sweep_count

    @property
    def freq_max(self) -> int:
        """Calculate maximum frequency."""
        return self.freq_min + self.freq_step * self.n_channels

    def reset(self) -> None:
        """Reset all state to initial values."""
        self.energy = None
        self.raw_energy = None
        self.sweep_count = 0
        self.active_detections = []
        self.detection_history = []
        self.peak_holds = []
        logger.debug("SpectrumState reset")
