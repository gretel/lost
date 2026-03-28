#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Configuration management for lora_spectrum."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import tomllib
except ImportError:
    import tomli as tomllib  # type: ignore[import-not-found]

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SpectrumConfig:
    """Configuration for lora_spectrum visualization.

    Attributes:
        ema_alpha: EMA smoothing factor (0.0-1.0, higher = more smoothing)
        db_headroom: dB padding above peak
        db_floor_pad: dB padding below noise floor (25th percentile)
        anchor_ttl: seconds to keep detection anchors visible
        peak_hold_s: seconds to hold peak detection bar height
        hot_multiplier: threshold multiplier for hot channels
        enable_ema: whether EMA smoothing is enabled
        bar_style: rendering style ('block', 'gradient', 'simple')
        show_grid: whether to show frequency grid lines
        max_history: maximum detection history entries
        update_hz: target update rate (affects CPU usage)
    """

    # Display parameters
    ema_alpha: float = 0.3
    db_headroom: float = 1.0
    db_floor_pad: float = 3.0
    anchor_ttl: float = 3.0
    peak_hold_s: float = 5.0
    hot_multiplier: float = 6.0

    # Feature toggles
    enable_ema: bool = True
    bar_style: str = "gradient"
    show_grid: bool = True
    max_history: int = 20
    update_hz: float = 30.0

    # Network
    host: str = "127.0.0.1"
    port: int = 5557

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SpectrumConfig":
        """Create config from dictionary (e.g., from TOML)."""
        return cls(
            ema_alpha=float(data.get("ema_alpha", 0.3)),
            db_headroom=float(data.get("db_headroom", 1.0)),
            db_floor_pad=float(data.get("db_floor_pad", 3.0)),
            anchor_ttl=float(data.get("anchor_ttl", 3.0)),
            peak_hold_s=float(data.get("peak_hold_s", 5.0)),
            hot_multiplier=float(data.get("hot_multiplier", 6.0)),
            enable_ema=bool(data.get("enable_ema", True)),
            bar_style=str(data.get("bar_style", "gradient")),
            show_grid=bool(data.get("show_grid", True)),
            max_history=int(data.get("max_history", 20)),
            update_hz=float(data.get("update_hz", 30.0)),
            host=str(data.get("host", "127.0.0.1")),
            port=int(data.get("port", 5557)),
        )

    @classmethod
    def from_toml(cls, path: Path | str | None = None) -> "SpectrumConfig":
        """Load config from TOML file.

        Searches for config.toml in standard locations if path not provided.
        """
        if path is None:
            path = cls._find_config()

        if path is None or not Path(path).exists():
            logger.debug("No config file found, using defaults")
            return cls()

        try:
            with open(path, "rb") as f:
                data = tomllib.load(f)

            spectrum_data = data.get("spectrum", {})
            config = cls.from_dict(spectrum_data)

            # Override network settings from [network] section if present
            network = data.get("network", {})
            if "udp_listen" in network:
                config = config.replace(host=network["udp_listen"])
            if "udp_port" in network:
                config = config.replace(port=int(network["udp_port"]))

            logger.debug(f"Loaded config from {path}")
            return config

        except Exception as e:
            logger.warning(f"Failed to load config from {path}: {e}")
            return cls()

    @staticmethod
    def _find_config() -> Path | None:
        """Find config.toml in standard locations."""
        search_paths = [
            Path("config.toml"),
            Path("apps/config.toml"),
            Path(__file__).parent.parent.parent / "apps" / "config.toml",
            Path.home() / ".config" / "gr4-lora" / "config.toml",
        ]

        for p in search_paths:
            if p.exists():
                return p
        return None

    def replace(self, **kwargs: Any) -> "SpectrumConfig":
        """Return new config with modified values."""
        return SpectrumConfig(
            ema_alpha=float(kwargs.get("ema_alpha", self.ema_alpha)),
            db_headroom=float(kwargs.get("db_headroom", self.db_headroom)),
            db_floor_pad=float(kwargs.get("db_floor_pad", self.db_floor_pad)),
            anchor_ttl=float(kwargs.get("anchor_ttl", self.anchor_ttl)),
            peak_hold_s=float(kwargs.get("peak_hold_s", self.peak_hold_s)),
            hot_multiplier=float(kwargs.get("hot_multiplier", self.hot_multiplier)),
            enable_ema=bool(kwargs.get("enable_ema", self.enable_ema)),
            bar_style=str(kwargs.get("bar_style", self.bar_style)),
            show_grid=bool(kwargs.get("show_grid", self.show_grid)),
            max_history=int(kwargs.get("max_history", self.max_history)),
            update_hz=float(kwargs.get("update_hz", self.update_hz)),
            host=str(kwargs.get("host", self.host)),
            port=int(kwargs.get("port", self.port)),
        )


def load_config(path: Path | str | None = None) -> SpectrumConfig:
    """Convenience function to load spectrum config."""
    return SpectrumConfig.from_toml(path)
