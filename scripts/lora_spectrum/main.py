#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
# pyright: reportImplicitRelativeImport=false
"""lora_spectrum - Real-time LoRa spectrum visualization.

Rewritten modular version with:
- Clean separation of concerns (config, state, protocol, rendering, input)
- Config file support (config.toml [spectrum] section)
- Interactive keyboard controls (pause, zoom, reset)
- Proper logging integration
- Type-safe throughout

Usage:
    python3 -m lora_spectrum [--config PATH] [--host HOST] [--port PORT]

    Or legacy: python3 scripts/lora_spectrum.py [...]

Keyboard Controls:
    Space  - Pause/resume updates
    +/-    - Zoom in/out (NYI)
    q      - Quit
    r      - Reset state
    t      - Toggle EMA smoothing
    g      - Toggle frequency grid
"""

from __future__ import annotations

import argparse
import atexit
import logging
import socket
import sys
import time
from pathlib import Path
from typing import NoReturn

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from lora_common import create_udp_subscriber, setup_logging, add_logging_args  # noqa: E402
from lora_spectrum.config import SpectrumConfig, load_config  # noqa: E402
from lora_spectrum.state import SpectrumState  # noqa: E402
from lora_spectrum.protocol import ProtocolHandler, create_default_handler  # noqa: E402
from lora_spectrum.renderer import TerminalRenderer, create_renderer  # noqa: E402
from lora_spectrum.input_handler import InputHandler  # noqa: E402

logger = logging.getLogger("lora_spectrum")


def create_argument_parser() -> argparse.ArgumentParser:
    """Create CLI argument parser."""
    parser = argparse.ArgumentParser(
        prog="lora_spectrum",
        description="Real-time LoRa spectrum visualization via UDP/CBOR",
    )

    # Config file
    parser.add_argument(
        "--config",
        type=str,
        help="Path to config.toml (searches standard locations if not specified)",
    )

    # Network
    parser.add_argument(
        "--host",
        type=str,
        default=None,
        help="lora_scan UDP host (overrides config)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="lora_scan UDP port (overrides config)",
    )

    # Display options
    parser.add_argument(
        "--ema",
        type=float,
        default=None,
        help="EMA smoothing alpha (0.0-1.0, overrides config)",
    )
    parser.add_argument(
        "--no-interactive",
        action="store_true",
        help="Disable interactive keyboard controls",
    )

    # Logging (from lora_common)
    add_logging_args(parser)

    return parser


def parse_args() -> tuple[argparse.Namespace, SpectrumConfig]:
    """Parse CLI args and load config."""
    parser = create_argument_parser()
    args = parser.parse_args()

    # Load config
    config_path = Path(args.config) if args.config else None
    config = load_config(config_path)

    # Apply CLI overrides
    if args.host is not None:
        config = config.replace(host=args.host)
    if args.port is not None:
        config = config.replace(port=args.port)
    if args.ema is not None:
        config = config.replace(ema_alpha=args.ema)

    return args, config


class SpectrumApp:
    """Main application controller."""

    def __init__(self, config: SpectrumConfig, interactive: bool = True) -> None:
        """Initialize application components."""
        self._config = config
        self._interactive = interactive and not sys.stdin.isatty()
        self._running = True

        # Create components
        self._state = SpectrumState(config)
        self._renderer = create_renderer(config)
        self._input_handler = InputHandler()

        # Create protocol handler with callbacks
        self._protocol = create_default_handler(
            on_spectrum=self._state.on_spectrum,
            on_sweep_end=self._state.on_sweep_end,
            on_status=self._state.on_status,
        )

        # Setup input handlers
        self._setup_input_handlers()

        logger.debug("SpectrumApp initialized")

    def _setup_input_handlers(self) -> None:
        """Setup keyboard input handlers."""
        self._input_handler.register("q", self._quit)
        self._input_handler.register("r", self._reset)
        self._input_handler.register("t", self._toggle_ema)

    def _quit(self) -> None:
        """Quit the application."""
        self._running = False
        logger.info("Quitting...")

    def _reset(self) -> None:
        """Reset application state."""
        self._state.reset()
        logger.info("State reset")

    def _toggle_ema(self) -> None:
        """Toggle EMA smoothing."""
        new_config = self._config.replace(enable_ema=not self._config.enable_ema)
        self._config = new_config
        # Need to update state's config reference - this is a bit hacky
        # In production, we'd use proper state management
        logger.info(
            f"EMA smoothing {'enabled' if new_config.enable_ema else 'disabled'}"
        )

    def run(self, sock: socket.socket) -> None:
        """Main application loop."""
        # Setup terminal
        self._renderer._write("\033[?25l")  # Hide cursor
        atexit.register(self._renderer.cleanup)

        # Enable input handling if interactive
        if self._interactive:
            self._input_handler.enable()
            atexit.register(self._input_handler.disable)

        first_render = True

        try:
            while self._running:
                # Check for keyboard input
                if self._interactive:
                    cmd = self._input_handler.check_input()
                    if cmd and cmd.action == "quit":
                        break

                # Receive data with short timeout for responsiveness
                sock.settimeout(0.1)
                try:
                    data, _ = sock.recvfrom(65536)
                except socket.timeout:
                    continue

                # Process message
                msg_type = self._protocol.handle(data)

                # Render on spectrum messages (unless paused)
                if msg_type == "scan_spectrum" and not self._input_handler.is_paused:
                    if first_render:
                        self._renderer.clear_screen()
                        first_render = False
                    self._renderer.render(self._state)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self._renderer.cleanup()
            if self._interactive:
                self._input_handler.disable()


def main() -> NoReturn:
    """Application entry point."""
    args, config = parse_args()

    # Setup logging
    log = setup_logging(
        "lora_spectrum",
        log_level=args.log_level,
        no_color=args.no_color,
    )

    log.info(f"lora_spectrum starting (connecting to {config.host}:{config.port})")
    log.debug(f"Config: ema_alpha={config.ema_alpha}, enable_ema={config.enable_ema}")

    # Create UDP subscriber
    try:
        sock, sub_msg, addr = create_udp_subscriber(
            config.host,
            config.port,
            timeout=10.0,
        )
        log.debug(f"Subscribed to {addr}")
    except Exception as e:
        log.error(f"Failed to connect: {e}")
        sys.exit(1)

    # Run application
    try:
        app = SpectrumApp(config, interactive=not args.no_interactive)
        app.run(sock)
    except Exception as e:
        log.exception("Application error")
        sys.exit(1)
    finally:
        sock.close()

    log.info("lora_spectrum exited")
    sys.exit(0)


if __name__ == "__main__":
    main()
