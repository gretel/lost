#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""Keyboard input handling for lora_spectrum."""

from __future__ import annotations

import logging
import os
import select
import sys
import termios
import tty
from dataclasses import dataclass, field
from typing import Any, Callable

logger = logging.getLogger(__name__)


@dataclass
class InputCommand:
    """A parsed input command."""

    action: str
    data: dict[str, object] = field(default_factory=dict)


class InputHandler:
    """Handles keyboard input for interactive controls.

    Supports:
    - Space: pause/resume updates
    - +/-: zoom in/out
    - q: quit
    - r: reset state
    - t: toggle EMA smoothing
    - g: toggle grid
    - s: toggle bar style
    """

    def __init__(self) -> None:
        """Initialize input handler."""
        self._handlers: dict[str, Callable[[], None]] = {}
        self._old_tty: Any | None = None
        self._enabled = False
        self._paused = False

        logger.debug("InputHandler initialized")

    def register(self, key: str, handler: Callable[[], None]) -> None:
        """Register a handler for a specific key."""
        self._handlers[key.lower()] = handler

    def enable(self) -> None:
        """Enable raw terminal input mode."""
        if self._enabled:
            return

        try:
            # Save current terminal settings
            self._old_tty = termios.tcgetattr(sys.stdin.fileno())

            # Set raw mode
            tty.setcbreak(sys.stdin.fileno())

            self._enabled = True
            logger.debug("Input handler enabled")
        except Exception as e:
            logger.warning(f"Failed to enable input handler: {e}")

    def disable(self) -> None:
        """Restore original terminal settings."""
        if not self._enabled or self._old_tty is None:
            return

        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSAFLUSH, self._old_tty)
            self._enabled = False
            logger.debug("Input handler disabled")
        except Exception as e:
            logger.warning(f"Failed to disable input handler: {e}")

    def check_input(self) -> InputCommand | None:
        """Check for and process pending input.

        Returns:
            InputCommand if a key was pressed, None otherwise.
        """
        if not self._enabled:
            return None

        try:
            # Check if input is available (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                return self._handle_key(char)
        except Exception as e:
            logger.debug(f"Input check failed: {e}")

        return None

    def _handle_key(self, char: str) -> InputCommand | None:
        """Handle a single keypress."""
        # Check for registered handlers first
        handler = self._handlers.get(char.lower())
        if handler:
            handler()
            return InputCommand("handled", {"key": char})

        # Default handlers
        if char == " ":
            self._paused = not self._paused
            logger.info(f"Display {'paused' if self._paused else 'resumed'}")
            return InputCommand("pause" if self._paused else "resume")

        if char == "q":
            return InputCommand("quit")

        if char == "r":
            return InputCommand("reset")

        if char == "t":
            return InputCommand("toggle_ema")

        if char == "g":
            return InputCommand("toggle_grid")

        if char == "s":
            return InputCommand("cycle_style")

        if char in "+-":
            return InputCommand("zoom", {"direction": 1 if char == "+" else -1})

        return None

    @property
    def is_paused(self) -> bool:
        """Check if display is paused."""
        return self._paused

    def __enter__(self) -> "InputHandler":
        """Context manager entry."""
        self.enable()
        return self

    def __exit__(self, *args: object) -> None:
        """Context manager exit."""
        self.disable()


def create_default_handler(
    on_pause: Callable[[], None] | None = None,
    on_resume: Callable[[], None] | None = None,
    on_quit: Callable[[], None] | None = None,
    on_reset: Callable[[], None] | None = None,
) -> InputHandler:
    """Create input handler with common callbacks."""
    handler = InputHandler()

    if on_pause:
        handler.register(" ", on_pause)
    if on_resume:
        handler.register(" ", on_resume)
    if on_quit:
        handler.register("q", on_quit)
    if on_reset:
        handler.register("r", on_reset)

    return handler
