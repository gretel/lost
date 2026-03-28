#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Entry point for python3 -m lora_spectrum"""

from __future__ import annotations

import sys
from pathlib import Path

# Add scripts directory to sys.path for absolute imports
_scripts_dir = Path(__file__).parent.parent
if str(_scripts_dir) not in sys.path:
    sys.path.insert(0, str(_scripts_dir))

from lora_spectrum.main import main

if __name__ == "__main__":
    main()
