#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# pyright: reportImplicitRelativeImport=false
"""lora_spectrum.py — real-time LoRa spectrum bar graph (legacy entry point).

This is a compatibility wrapper around the new modular lora_spectrum package.
All functionality has been moved to:
  - lora_spectrum/config.py      - Configuration management
  - lora_spectrum/state.py       - State management
  - lora_spectrum/protocol.py    - CBOR protocol handlers
  - lora_spectrum/renderer.py    - Terminal rendering
  - lora_spectrum/input_handler.py - Keyboard controls
  - lora_spectrum/main.py        - Main application

Usage:
    python3 scripts/lora_spectrum.py [--host HOST] [--port PORT] [--ema ALPHA]

For new code, use: python3 -m lora_spectrum (see __main__.py)
"""

from __future__ import annotations

import sys
from pathlib import Path

# Add scripts directory to sys.path for absolute imports
_scripts_dir = Path(__file__).parent
if str(_scripts_dir) not in sys.path:
    sys.path.insert(0, str(_scripts_dir))

from lora_spectrum.main import main

if __name__ == "__main__":
    main()
