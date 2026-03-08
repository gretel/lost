#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_wav.py -- LoRa chirp audio exporter.

Subscribes to a lora_trx UDP server. For each decoded frame, synthesises
a LoRa chirp audio clip (re-mapped to audible frequencies), saves it as
data/audio/<sha256_8hex>.wav, and optionally plays it via sounddevice.

The WAV filename is derived from SHA-256(payload)[:8] so identical payloads
map to the same file and are not re-synthesised.  PHY metadata is embedded
in the WAV RIFF INFO chunk (ICMT / INAM) for DuckDB / later aggregation.

Usage:
    lora_wav.py                           # live UDP, default 127.0.0.1:5555
    lora_wav.py --no-play                 # save WAV only, no speaker output
    lora_wav.py --connect 127.0.0.1:5556
    lora_wav.py --audio-dir /tmp/lora

Dependencies: cbor2, sounddevice (optional -- disabled by --no-play)
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import math
import socket
import struct
import time
import wave
from pathlib import Path
from typing import Any

import cbor2

from lora_common import (
    KEEPALIVE_INTERVAL,
    RECV_TIMEOUT,
    create_udp_subscriber,
    load_config,
    resolve_udp_address,
    setup_logging,
)

log = logging.getLogger("gr4.wav")

DEFAULT_AUDIO_DIR = "data/audio"
AUDIO_SAMPLE_RATE = 22050  # Hz — quarter CD rate, smooth phase at low frequency
CHIRP_AMPLITUDE = 0.4  # 0..1 — keep well below full scale
SYMBOL_DURATION_S = 0.15  # seconds per chirp — slow enough to sound intentional
CHIRP_F_LO = 100.0  # Hz — low, gentle register
CHIRP_F_HI = 900.0  # Hz — stays below 1 kHz for pleasant tone


# ---------------------------------------------------------------------------
# Chirp synthesis
# ---------------------------------------------------------------------------


def _synthesise_chirp(
    *,
    upchirp: bool = True,
    n_symbols: int = 1,
) -> list[float]:
    """Return PCM samples for *n_symbols* LoRa-style chirps.

    Symbol duration is fixed at SYMBOL_DURATION_S (not tied to SF) so the
    sweep is always slow enough to hear clearly.  Frequency range is
    CHIRP_F_LO..CHIRP_F_HI — a low, gentle register.  Amplitude is
    CHIRP_AMPLITUDE to avoid clipping.
    """
    n = int(SYMBOL_DURATION_S * AUDIO_SAMPLE_RATE)  # samples per symbol
    f_span = CHIRP_F_HI - CHIRP_F_LO

    samples: list[float] = []
    for _ in range(n_symbols):
        for k in range(n):
            frac = k / n
            t = k / AUDIO_SAMPLE_RATE
            if upchirp:
                phi = 2 * math.pi * (CHIRP_F_LO * t + 0.5 * f_span * t * frac)
            else:
                phi = 2 * math.pi * (CHIRP_F_HI * t - 0.5 * f_span * t * frac)
            samples.append(CHIRP_AMPLITUDE * math.sin(phi))
    return samples


def synthesise_frame(*, crc_ok: bool, preamble: int = 8) -> list[float]:
    """Synthesise audio for a full LoRa frame (preamble + sync word).

    CRC_OK  → preamble upchirps + 2 downchirps (sync word signature)
    CRC_FAIL → downchirp preamble only (distinctive descending tone)
    """
    if not crc_ok:
        return _synthesise_chirp(upchirp=False, n_symbols=preamble)

    audio = _synthesise_chirp(upchirp=True, n_symbols=preamble)
    audio += _synthesise_chirp(upchirp=False, n_symbols=2)
    return audio


# ---------------------------------------------------------------------------
# WAV I/O with RIFF INFO metadata
# ---------------------------------------------------------------------------


def _pack_riff_info(comment: str, name: str) -> bytes:
    """Pack a RIFF LIST INFO chunk with ICMT and INAM sub-chunks."""

    def subchunk(fcc: bytes, text: str) -> bytes:
        data = text.encode("utf-8", errors="replace")
        # RIFF chunks must be WORD-aligned; pad with a NUL byte if needed
        if len(data) % 2:
            data += b"\x00"
        return fcc + struct.pack("<I", len(data)) + data

    payload = subchunk(b"ICMT", comment) + subchunk(b"INAM", name)
    return b"LIST" + struct.pack("<I", 4 + len(payload)) + b"INFO" + payload


def save_wav(path: Path, samples: list[float], comment: str, name: str) -> None:
    """Write float32 samples to *path* as 16-bit PCM WAV with INFO metadata."""
    # Convert to 16-bit signed PCM
    pcm = struct.pack(f"<{len(samples)}h", *(int(s * 32767) for s in samples))

    # Build the WAV via stdlib wave, then append the INFO chunk manually
    import io

    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(AUDIO_SAMPLE_RATE)
        wf.writeframes(pcm)

    wav_bytes = buf.getvalue()

    # Append RIFF INFO chunk after the data chunk and fix RIFF size
    info_chunk = _pack_riff_info(comment, name)
    new_body = wav_bytes[8:] + info_chunk  # strip "RIFF????WAVE" header
    riff_size = 4 + len(new_body)
    final = b"RIFF" + struct.pack("<I", riff_size) + b"WAVE" + new_body[4:]

    path.write_bytes(final)


# ---------------------------------------------------------------------------
# Frame key
# ---------------------------------------------------------------------------


def frame_key(payload: bytes) -> str:
    """Return 16-hex-char SHA-256 digest of *payload* — used as filename stem."""
    return hashlib.sha256(payload).hexdigest()[:16]


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run(
    host: str,
    port: int,
    audio_dir: Path,
    *,
    play: bool,
) -> None:
    audio_dir.mkdir(parents=True, exist_ok=True)

    sock, sub_msg, addr = create_udp_subscriber(host, port)
    local_port = sock.getsockname()[1]
    log.info("subscribed to lora_trx at %s:%d (local :%d)", host, port, local_port)

    sd = None
    if play:
        try:
            import sounddevice as sd  # type: ignore[import]
        except ImportError:
            log.warning("sounddevice not available — disabling playback")
            sd = None

    last_keepalive = time.monotonic()
    waiting = False

    try:
        while True:
            try:
                data, _addr = sock.recvfrom(65536)
                waiting = False
            except socket.timeout:
                sock.sendto(sub_msg, addr)
                last_keepalive = time.monotonic()
                if not waiting:
                    log.info("waiting for frames from %s:%d", host, port)
                    waiting = True
                continue

            now = time.monotonic()
            if now - last_keepalive >= KEEPALIVE_INTERVAL:
                sock.sendto(sub_msg, addr)
                last_keepalive = now

            try:
                msg = cbor2.loads(data)
            except Exception:
                continue

            if not isinstance(msg, dict) or msg.get("type") != "lora_frame":
                continue

            payload: bytes = msg.get("payload", b"")
            phy: dict[str, Any] = msg.get("phy", {})
            crc_ok: bool = msg.get("crc_valid", False)
            sf: int = phy.get("sf", 8)
            bw: int = phy.get("bw", 62500)
            snr: float | None = phy.get("snr_db")
            sync_word: int = phy.get("sync_word", 0)
            seq: int = msg.get("seq", 0)

            key = frame_key(payload)
            wav_path = audio_dir / f"{key}.wav"

            if not wav_path.exists():
                samples = synthesise_frame(crc_ok=crc_ok)
                snr_str = f"{snr:.1f}dB" if snr is not None else "n/a"
                crc_str = "OK" if crc_ok else "FAIL"
                comment = (
                    f"SF={sf} BW={bw} CRC={crc_str} "
                    f"SNR={snr_str} seq={seq} sync=0x{sync_word:02X}"
                )
                save_wav(wav_path, samples, comment=comment, name=key)
                log.info(
                    "saved %s (%dB payload, CRC_%s)",
                    wav_path.name,
                    len(payload),
                    crc_str,
                )
            else:
                log.debug("skip existing %s", wav_path.name)

            if sd is not None:
                import numpy as np  # type: ignore[import]  # transitive via sounddevice

                with wave.open(str(wav_path)) as wf:
                    raw = wf.readframes(wf.getnframes())
                arr = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                sd.play(arr, samplerate=AUDIO_SAMPLE_RATE)  # type: ignore[union-attr]

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LoRa chirp audio exporter — saves WAV per decoded frame"
    )
    parser.add_argument(
        "--connect",
        metavar="HOST:PORT",
        default=None,
        help="lora_trx UDP server (default from config.toml or 127.0.0.1:5555)",
    )
    parser.add_argument(
        "--config",
        metavar="PATH",
        default=None,
        help="Path to config.toml (auto-detected if omitted)",
    )
    parser.add_argument(
        "--audio-dir",
        metavar="PATH",
        default=DEFAULT_AUDIO_DIR,
        help=f"Directory for WAV output (default: {DEFAULT_AUDIO_DIR})",
    )
    parser.add_argument(
        "--no-play",
        action="store_true",
        default=False,
        help="Save WAV files without playing audio",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        default=False,
        help="Disable ANSI color output",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    setup_logging("gr4.wav", cfg, no_color=args.no_color)

    try:
        host, port = resolve_udp_address(args.connect, cfg)
    except ValueError as exc:
        parser.error(str(exc))

    run(host, port, Path(args.audio_dir), play=not args.no_play)


if __name__ == "__main__":
    main()
