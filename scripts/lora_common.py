#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_common.py -- Shared constants and helpers for LoRa scripts.

Provides:
  - MeshCore protocol display names (ROUTE_NAMES, PAYLOAD_NAMES)
  - Hex/ASCII formatting helpers
  - Shared TOML configuration loader (config.toml search path)
  - UDP subscriber setup for connecting to lora_trx
  - Host:port parsing (IPv4 + IPv6)
  - MeshCore URI builders
"""

from __future__ import annotations

import logging
import os
import socket
import sys
import tomllib
from pathlib import Path
from typing import IO, Any
from urllib.parse import quote

import cbor2

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


def format_hexdump(
    data: bytes, *, max_bytes: int = 48, indent: str = "  ", cols: int = 16
) -> str:
    """Format bytes as an xxd-style hex dump with ASCII sidebar.

    Each line shows up to *cols* bytes as hex pairs, followed by printable
    ASCII (dots for non-printable).  Truncates at *max_bytes* with an
    indicator.  Returns a multi-line string (no trailing newline).
    """
    show = data[:max_bytes]
    lines: list[str] = []
    for off in range(0, len(show), cols):
        chunk = show[off : off + cols]
        hex_part = " ".join(f"{b:02X}" for b in chunk)
        # Pad hex part to fixed width so the ASCII sidebar aligns
        hex_width = cols * 3 - 1  # "XX XX XX..." with spaces
        ascii_part = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
        lines.append(f"{indent}{hex_part:<{hex_width}}  {ascii_part}")
    if len(data) > max_bytes:
        lines.append(f"{indent}... ({len(data)} bytes total)")
    return "\n".join(lines)


# Codepoints that are dangerous or invisible when rendered on a terminal.
# Escaped by sanitize_text() regardless of context.
_DANGEROUS_CODEPOINTS: frozenset[int] = frozenset(
    # C1 controls (U+0080..U+009F) — includes CSI U+009B which acts as
    # ESC[ on 8-bit terminals.
    set(range(0x80, 0xA0))
    # Lone surrogates (U+D800..U+DFFF) — invalid Unicode, crash on re-encode.
    | set(range(0xD800, 0xE000))
    # Zero-width / bidi / formatting characters.
    | {
        0x200B,  # zero-width space
        0x200C,
        0x200D,  # ZWNJ, ZWJ
        0x200E,
        0x200F,  # LRM, RLM
        0x202A,
        0x202B,
        0x202C,
        0x202D,
        0x202E,  # bidi embeddings
        0x2066,
        0x2067,
        0x2068,
        0x2069,  # bidi isolates
        0xFEFF,  # BOM / ZWNBSP
        0xFFF9,
        0xFFFA,
        0xFFFB,  # interlinear annotations
    }
    # Unicode TAG characters (U+E0001..U+E007F) — invisible tagging.
    | set(range(0xE0001, 0xE0080))
)


def sanitize_text(text: str) -> str:
    """Remove terminal-hostile characters from a decoded string.

    Escapes C0 controls, DEL, C1 controls (U+0080-U+009F), lone surrogates,
    Unicode bidi overrides, zero-width characters, and TAG characters.
    Safe for printing to any terminal and for embedding in log messages.
    """
    parts: list[str] = []
    for ch in text:
        cp = ord(ch)
        if cp < 0x20:
            # C0 controls (U+0000..U+001F) — escape all including NUL, TAB, LF
            parts.append(f"\\x{cp:02x}")
        elif cp == 0x7F:
            parts.append("\\x7f")
        elif cp in _DANGEROUS_CODEPOINTS:
            parts.append(f"\\u{cp:04x}" if cp < 0x10000 else f"\\U{cp:08x}")
        else:
            parts.append(ch)
    return "".join(parts)


def format_ascii(data: bytes, *, max_bytes: int | None = None) -> str:
    """Decode as UTF-8 where valid, replace invalid bytes with hex escapes.

    All output is console-safe: control characters, C1 controls, bidi
    overrides, and DEL are escaped.
    """
    raw = data[:max_bytes] if max_bytes is not None else data
    parts: list[str] = []
    i = 0
    while i < len(raw):
        b = raw[i]
        if b < 0x20:
            # C0 controls — escape all (no TAB/LF exceptions)
            parts.append(f"\\x{b:02x}")
            i += 1
        elif b == 0x7F:
            parts.append("\\x7f")
            i += 1
        elif b < 0x80:
            parts.append(chr(b))
            i += 1
        else:
            # Try to decode a multi-byte UTF-8 sequence
            decoded = False
            for length in (4, 3, 2):
                if i + length <= len(raw):
                    try:
                        ch = raw[i : i + length].decode("utf-8")
                        # Sanitize the decoded character(s)
                        parts.append(sanitize_text(ch))
                        i += length
                        decoded = True
                        break
                    except UnicodeDecodeError:
                        continue
            if not decoded:
                parts.append(f"\\x{b:02x}")
                i += 1
    s = "".join(parts)
    if max_bytes is not None and len(data) > max_bytes:
        s += "..."
    return s


def utf8_truncate(s: str, max_bytes: int) -> bytes:
    """Encode *s* as UTF-8 and truncate to at most *max_bytes* without
    splitting a multi-byte sequence.

    Returns the truncated ``bytes`` (which may be shorter than *max_bytes*
    if a codepoint was split at the boundary).
    """
    encoded = s.encode("utf-8")
    if len(encoded) <= max_bytes:
        return encoded
    # Walk back from the cut point to a valid UTF-8 character boundary.
    # Continuation bytes have the pattern 10xxxxxx (0x80..0xBF).
    trunc = encoded[:max_bytes]
    while trunc and (trunc[-1] & 0xC0) == 0x80:
        trunc = trunc[:-1]
    # If the last byte is a leading byte (11xxxxxx) whose sequence was
    # cut, remove it too.
    if trunc and trunc[-1] >= 0xC0:
        trunc = trunc[:-1]
    return trunc


# ---- Logging ----

#: Default log format (used by plain-text path; color path overrides this).
DEFAULT_LOG_FORMAT = "%(asctime)s  %(name)s  %(levelname)s  %(message)s"

#: Timestamp is handled by _ColorFormatter.formatTime() (UTC ms + Z).
#: None means the formatter produces its own timestamp string.
DEFAULT_LOG_DATEFMT: str | None = None


def _want_color(stream: IO[Any]) -> bool:
    """Determine whether to use ANSI color on *stream*.

    Priority: ``NO_COLOR`` (off) > ``FORCE_COLOR`` (on) > ``isatty()``.
    Follows the Python 3.14 convention and https://no-color.org/.
    """
    if os.environ.get("NO_COLOR", "") != "":
        return False
    if os.environ.get("FORCE_COLOR") is not None:
        return True
    return hasattr(stream, "isatty") and stream.isatty()


class _ColorFormatter(logging.Formatter):
    """Log formatter with ANSI colors keyed on log level.

    Timestamps are local time HH:MM:SS.mmmm (4 fractional digits, 100µs precision).
    Level labels are padded full words (e.g. "info   ", "warning").
    The label is bold+colored; timestamp, logger name, and message are plain.

    Falls back to plain text when *use_color* is ``False``.
    """

    # ANSI SGR codes
    _RESET = "\033[0m"
    _DIM = "\033[2m"
    _BOLD = "\033[1m"

    # Per-level: (color code, padded full-word label)
    _LEVEL_STYLE: dict[int, tuple[str, str]] = {
        logging.DEBUG: ("\033[36m", "debug  "),  # cyan,     7 chars
        logging.INFO: ("\033[32m", "info   "),  # green,    7 chars
        logging.WARNING: ("\033[33m", "warning"),  # yellow,   7 chars
        logging.ERROR: ("\033[31m", "error  "),  # red,      7 chars
        logging.CRITICAL: ("\033[1;31m", "critical"),  # bold red, 8 chars
    }
    _DEFAULT_STYLE = ("\033[37m", "???????")  # white fallback, 7 chars

    def __init__(
        self,
        fmt: str | None = None,
        datefmt: str | None = None,
        *,
        use_color: bool = True,
    ) -> None:
        # We handle formatting ourselves; the parent format string is unused
        # when color is active, but kept as fallback for plain mode.
        super().__init__(fmt=fmt, datefmt=datefmt)
        self._use_color = use_color

    def formatTime(self, record: logging.LogRecord, datefmt: str | None = None) -> str:
        """Return local time HH:MM:SS with 4 fractional digits (100µs precision)."""
        import datetime as _dt

        now = _dt.datetime.fromtimestamp(record.created)
        return now.strftime("%H:%M:%S") + f".{now.microsecond // 100:04d}"

    def format(self, record: logging.LogRecord) -> str:
        if not self._use_color:
            return super().format(record)

        ts = self.formatTime(record)
        color, label = self._LEVEL_STYLE.get(record.levelno, self._DEFAULT_STYLE)
        name = record.name
        msg = record.getMessage()

        # Exception info
        extra = ""
        if record.exc_info and not record.exc_text:
            record.exc_text = self.formatException(record.exc_info)
        if record.exc_text:
            extra = "\n" + record.exc_text

        # Build the prefix: dim timestamp  dim name  bold+colored label  plain message
        prefix = (
            f"{self._DIM}{ts}{self._RESET}  "
            f"{self._DIM}{name}{self._RESET}  "
            f"{color}{self._BOLD}{label}{self._RESET}"
        )

        # For multiline messages, indent continuation lines to align under
        # the message text (two spaces after each separator).
        lines = msg.split("\n")
        first = f"{prefix}  {lines[0]}"
        if len(lines) == 1:
            return first + extra

        pad = " " * (len(ts) + 2 + len(name) + 2 + len(label) + 2)
        rest = "\n".join(f"{pad}{line}" for line in lines[1:])
        return first + "\n" + rest + extra


def setup_logging(
    name: str,
    cfg: dict[str, Any] | None = None,
    *,
    debug: bool = False,
    no_color: bool = False,
) -> logging.Logger:
    """Configure syslog-style logging and return a named logger.

    Reads optional ``[logging]`` section from *cfg* (a parsed config.toml dict):

    .. code-block:: toml

        [logging]
        level  = "INFO"        # DEBUG, INFO, WARNING, ERROR
        color  = true          # false to force-disable ANSI colors

    Colors are strictly a local terminal concern — ANSI codes are only emitted
    when stderr is a TTY.  Piped output, files, and syslog always get plain text.

    The *debug* flag overrides the configured level to DEBUG.
    The *no_color* flag forces plain output (also respected: ``NO_COLOR`` env var).
    Output goes to stderr so that stdout remains available for data.
    """
    log_cfg = (cfg or {}).get("logging", {})
    level_str = "DEBUG" if debug else log_cfg.get("level", "INFO")
    level = getattr(logging, level_str.upper(), logging.INFO)
    fmt = log_cfg.get("format", DEFAULT_LOG_FORMAT)
    datefmt = log_cfg.get("datefmt", DEFAULT_LOG_DATEFMT)

    # Color: --no-color flag > config > auto-detect
    if no_color:
        use_color = False
    else:
        cfg_color = log_cfg.get("color")
        if cfg_color is not None:
            use_color = bool(cfg_color) and _want_color(sys.stderr)
        else:
            use_color = _want_color(sys.stderr)

    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(_ColorFormatter(fmt=fmt, datefmt=datefmt, use_color=use_color))

    root = logging.getLogger()
    root.setLevel(level)
    # Replace any existing handlers (safe for single-entry-point scripts)
    root.handlers = [handler]

    return logging.getLogger(name)


# ---- Shared TOML configuration ----

# Search path for config.toml (first match wins):
#   1. ./config.toml                         (CWD, e.g. when running from apps/)
#   2. ./apps/config.toml                    (CWD = project root)
#   3. <script-dir>/../apps/config.toml      (script-relative, works from scripts/)
#   4. ~/.config/gr4-lora/config.toml        (user config)
_CONFIG_SEARCH = [
    Path("config.toml"),
    Path("apps/config.toml"),
    Path(__file__).parent.parent / "apps" / "config.toml",
    Path.home() / ".config" / "gr4-lora" / "config.toml",
]

_log = logging.getLogger(__name__)


def find_config(explicit: str | Path | None = None) -> Path | None:
    """Find config.toml by explicit path or search path. Returns None if not found."""
    if explicit is not None:
        p = Path(explicit)
        return p if p.is_file() else None
    for p in _CONFIG_SEARCH:
        if p.is_file():
            return p
    return None


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """Load config.toml and return the raw dict. Returns {} if not found.

    Logs a warning when no config file is found so the failure is visible.
    An explicit path that doesn't exist is also warned about.
    """
    if path is not None:
        p = Path(path)
        if not p.is_file():
            _log.warning("config file not found: %s — using defaults", p)
            return {}
    p = find_config(path)
    if p is None:
        _log.warning(
            "config.toml not found (searched: %s) — using defaults",
            ", ".join(str(s) for s in _CONFIG_SEARCH),
        )
        return {}
    _log.debug("loaded config: %s", p)
    with open(p, "rb") as f:
        return tomllib.load(f)


def config_udp_host(cfg: dict[str, Any]) -> str:
    """Extract UDP host from config (checks [network] section first, then flat keys)."""
    net = cfg.get("network", {})
    return str(net.get("udp_listen", cfg.get("udp_listen", "127.0.0.1")))


def config_udp_port(cfg: dict[str, Any]) -> int:
    """Extract UDP port from config (checks [network] section first, then flat keys).

    Returns the raw lora_trx port (default 5555 for empty config / legacy configs,
    5556 for new sectioned configs that run lora_agg).  Scripts that want the
    aggregated consumer stream should use config_agg_listen() instead.
    """
    net = cfg.get("network", {})
    return int(net.get("udp_port", cfg.get("udp_port", 5555)))


def config_agg_upstream(cfg: dict[str, Any]) -> str:
    """Extract lora_agg upstream address (lora_trx raw port) from [aggregator] section."""
    agg = cfg.get("aggregator", {})
    return str(agg.get("upstream", cfg.get("agg_upstream", "127.0.0.1:5556")))


def config_agg_listen(cfg: dict[str, Any]) -> str:
    """Extract lora_agg consumer listen address from [aggregator] section."""
    agg = cfg.get("aggregator", {})
    return str(agg.get("listen", cfg.get("agg_listen", "127.0.0.1:5555")))


def config_agg_window_ms(cfg: dict[str, Any]) -> int:
    """Extract lora_agg diversity combining window (ms) from [aggregator] section."""
    agg = cfg.get("aggregator", {})
    return int(agg.get("window_ms", cfg.get("agg_window_ms", 200)))


def config_region_scope(cfg: dict[str, Any]) -> str:
    """Extract region_scope from [meshcore] section, defaulting to empty."""
    return str(cfg.get("meshcore", {}).get("region_scope", ""))


# ---- UDP subscriber helpers ----

#: Keepalive interval for UDP subscriber registration (seconds).
KEEPALIVE_INTERVAL = 5.0

#: Default receive timeout for UDP sockets (seconds).
RECV_TIMEOUT = 10.0

#: Default lora_trx host.
DEFAULT_HOST = "127.0.0.1"

#: Default lora_trx port.
DEFAULT_PORT = 5555


def parse_host_port(
    spec: str, default_host: str = DEFAULT_HOST, default_port: int = DEFAULT_PORT
) -> tuple[str, int]:
    """Parse a HOST:PORT string, supporting IPv6 bracket notation.

    Formats:
        "host:port"       -> (host, port)
        "[::1]:5555"      -> ("::1", 5555)
        ":5555"           -> (default_host, 5555)

    Raises ValueError if the format is invalid.
    """
    if spec.startswith("["):
        # IPv6: [host]:port
        bracket = spec.index("]")
        host = spec[1:bracket]
        rest = spec[bracket + 1 :]
        if rest.startswith(":"):
            port = int(rest[1:])
        else:
            port = default_port
        return host, port

    colon = spec.rfind(":")
    if colon <= 0:
        raise ValueError(
            f"invalid HOST:PORT spec '{spec}' (expected host:port, e.g. 127.0.0.1:5555)"
        )
    return spec[:colon], int(spec[colon + 1 :])


def resolve_udp_address(
    connect_arg: str | None,
    cfg: dict[str, Any],
) -> tuple[str, int]:
    """Resolve consumer UDP address from --connect arg or config.toml.

    Priority: explicit --connect > [aggregator].listen (if present) >
    [network].udp_port > defaults (127.0.0.1:5555).

    When lora_agg is configured, clients should connect to the aggregator
    (5555) rather than directly to lora_trx (5556).
    """
    if connect_arg:
        return parse_host_port(connect_arg)
    if cfg.get("aggregator"):
        return parse_host_port(config_agg_listen(cfg))
    return config_udp_host(cfg), config_udp_port(cfg)


def create_udp_subscriber(
    host: str,
    port: int,
    *,
    sync_words: list[int] | None = None,
    timeout: float = RECV_TIMEOUT,
    blocking: bool = True,
) -> tuple[socket.socket, bytes, tuple[str, int]]:
    """Create a UDP socket, subscribe to lora_trx/lora_agg, return (sock, sub_msg, addr).

    Args:
        host: Server host address.
        port: Server UDP port.
        sync_words: Optional sync_word filter (e.g. [0x12] for MeshCore).
        timeout: Socket receive timeout (seconds). Ignored if blocking=False.
        blocking: If False, set socket to non-blocking mode.

    Returns:
        (sock, sub_msg, addr) where sub_msg can be re-sent for keepalives.
    """
    af = socket.AF_INET6 if ":" in host else socket.AF_INET
    sock = socket.socket(af, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("" if af == socket.AF_INET else "::", 0))
    if blocking:
        sock.settimeout(timeout)
    else:
        sock.setblocking(False)

    sub: dict[str, Any] = {"type": "subscribe"}
    if sync_words:
        sub["sync_word"] = sync_words
    sub_msg = cbor2.dumps(sub)
    addr = (host, port)
    sock.sendto(sub_msg, addr)
    return sock, sub_msg, addr


# ---- MeshCore URI helpers ----

# ADVERT node types (used as defaults in URI builder)
ADVERT_NODE_CHAT = 0x01


def build_contact_uri(
    pub_key: bytes, name: str, node_type: int = ADVERT_NODE_CHAT
) -> str:
    """Build a MeshCore contact URI for QR code scanning by the companion app.

    Format: meshcore://contact/add?name=<name>&public_key=<64hex>&type=<int>

    This is the mobile app QR format documented in MeshCore/docs/qr_codes.md.
    """
    params = []
    if name:
        params.append(f"name={quote(name, safe='')}")
    params.append(f"public_key={pub_key.hex()}")
    params.append(f"type={node_type}")
    return "meshcore://contact/add?" + "&".join(params)


def build_bizcard_uri(advert_packet: bytes) -> str:
    """Build a MeshCore CLI "biz card" URI from a raw ADVERT wire packet.

    Format: meshcore://<hex-encoded raw ADVERT packet>

    Used by meshcore-cli import_contact and firmware serial/BLE companion
    protocol. The CLI calls bytes.fromhex(uri[11:]) and feeds the result
    into importContact() which deserializes it as a Packet.
    """
    return "meshcore://" + advert_packet.hex()
