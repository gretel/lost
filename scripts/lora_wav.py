#!/usr/bin/env python3
# SPDX-License-Identifier: ISC
"""
lora_wav.py -- LoRa chirp audio exporter.

Subscribes to a lora_trx UDP server. For each decoded frame, synthesises
a LoRa chirp audio clip (re-mapped to audible frequencies), saves it as
data/audio/<sha256_16hex>.wav, and optionally plays it via sounddevice.

Each payload produces a deterministic, recognisable audio signature:
  - 1 generic upchirp cue (or downchirp for CRC_FAIL) as an "attention" tone
  - 3–6 payload-derived chirps with varied frequency range, sweep span,
    duration, and direction — derived from the SHA-256 of the payload

Identical payloads always produce the same sound. Playback is queued so that
concurrent frames play sequentially without interruption. Short cosine fades
and silence padding at each end prevent clicks and clipping.

The WAV filename is derived from SHA-256(payload)[:16] so identical payloads
map to the same file and are not re-synthesised. PHY metadata is embedded in
the WAV RIFF INFO chunk (ICMT / INAM) for DuckDB / later aggregation.

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
import queue
import socket
import struct
import threading
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
CHIRP_AMPLITUDE = 0.38  # 0..1 — leave headroom for cosine fade

# Cue chirp (fixed, payload-independent — "attention, frame arrived")
CUE_F_LO = 200.0  # Hz
CUE_F_HI = 800.0  # Hz
CUE_DURATION_S = 0.07  # seconds

# Payload signature chirps (range boundaries — hash bytes select within these)
SIG_F_LO_MIN = 80.0  # Hz — lowest base frequency
SIG_F_LO_MAX = 350.0  # Hz — highest base frequency
SIG_SPAN_MIN = 200.0  # Hz — minimum sweep span
SIG_SPAN_MAX = 750.0  # Hz — maximum sweep span
SIG_DUR_MIN = 0.055  # seconds — shortest chirp
SIG_DUR_MAX = 0.120  # seconds — longest chirp
SIG_N_MIN = 3  # fewest payload chirps
SIG_N_MAX = 6  # most payload chirps

# Padding / envelope
SILENCE_PAD_S = 0.010  # 10 ms silence at start and end of each WAV
FADE_S = 0.005  # 5 ms cosine fade-in and fade-out on each chirp
INTER_CHIRP_S = 0.012  # 12 ms gap between cue and signature chirps


# ---------------------------------------------------------------------------
# Chirp synthesis helpers
# ---------------------------------------------------------------------------


def _silence(n_samples: int) -> list[float]:
    return [0.0] * n_samples


def _cosine_fade(samples: list[float], fade_n: int) -> list[float]:
    """Apply cosine fade-in and fade-out to *samples* (in-place clone)."""
    s = list(samples)
    n = len(s)
    fade_n = min(fade_n, n // 2)
    for i in range(fade_n):
        t = i / fade_n
        w = 0.5 - 0.5 * math.cos(math.pi * t)  # 0 → 1 over fade_n samples
        s[i] *= w
        s[n - 1 - i] *= w
    return s


def _chirp(
    f_lo: float, f_hi: float, duration_s: float, *, upchirp: bool = True
) -> list[float]:
    """Synthesise one chirp.  Returns samples with cosine fade applied."""
    n = int(duration_s * AUDIO_SAMPLE_RATE)
    f_span = f_hi - f_lo
    fade_n = int(FADE_S * AUDIO_SAMPLE_RATE)
    samples: list[float] = []
    for k in range(n):
        frac = k / n
        t = k / AUDIO_SAMPLE_RATE
        if upchirp:
            phi = 2 * math.pi * (f_lo * t + 0.5 * f_span * t * frac)
        else:
            phi = 2 * math.pi * (f_hi * t - 0.5 * f_span * t * frac)
        samples.append(CHIRP_AMPLITUDE * math.sin(phi))
    return _cosine_fade(samples, fade_n)


def _lerp(lo: float, hi: float, t: float) -> float:
    """Linear interpolate: t ∈ [0, 1] → [lo, hi]."""
    return lo + (hi - lo) * t


# ---------------------------------------------------------------------------
# Signature synthesis
# ---------------------------------------------------------------------------


def _signature_params(payload: bytes) -> dict[str, Any]:
    """Derive chirp signature parameters from SHA-256(payload).

    Consumes 10 bytes of the hash deterministically:
      [0,1]   → base frequency (f_lo)
      [2,3]   → sweep span
      [4]     → symbol duration
      [5]     → number of payload chirps (3–6)
      [6]     → direction bitmask (bit i = upchirp for chirp i)
      [7]     → reserved / future use
    """
    h = hashlib.sha256(payload).digest()

    f_lo = _lerp(SIG_F_LO_MIN, SIG_F_LO_MAX, int.from_bytes(h[0:2], "big") / 65535)
    span = _lerp(SIG_SPAN_MIN, SIG_SPAN_MAX, int.from_bytes(h[2:4], "big") / 65535)
    duration_s = _lerp(SIG_DUR_MIN, SIG_DUR_MAX, h[4] / 255)
    n_chirps = SIG_N_MIN + (h[5] % (SIG_N_MAX - SIG_N_MIN + 1))
    dir_mask = h[6]  # bit i: 1 = upchirp, 0 = downchirp

    return {
        "f_lo": f_lo,
        "f_hi": f_lo + span,
        "duration_s": duration_s,
        "n_chirps": n_chirps,
        "dir_mask": dir_mask,
    }


def synthesise_frame(*, payload: bytes, crc_ok: bool) -> list[float]:
    """Synthesise audio for a full LoRa frame.

    Structure:
      [silence pad]
      [cue chirp: 1 upchirp (CRC OK) or 1 downchirp (CRC FAIL)]
      [inter-chirp gap]
      [payload signature: 3–6 chirps derived from payload hash]
      [silence pad]
    """
    pad_n = int(SILENCE_PAD_S * AUDIO_SAMPLE_RATE)
    gap_n = int(INTER_CHIRP_S * AUDIO_SAMPLE_RATE)

    # Cue
    cue = _chirp(CUE_F_LO, CUE_F_HI, CUE_DURATION_S, upchirp=crc_ok)

    # Payload signature
    p = _signature_params(payload)
    sig: list[float] = []
    for i in range(p["n_chirps"]):
        upchirp = bool((p["dir_mask"] >> i) & 1)
        sig += _chirp(p["f_lo"], p["f_hi"], p["duration_s"], upchirp=upchirp)

    return _silence(pad_n) + cue + _silence(gap_n) + sig + _silence(pad_n)


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
# Playback thread
# ---------------------------------------------------------------------------


def _playback_worker(play_queue: "queue.Queue[list[float] | None]") -> None:
    """Background thread: dequeues audio arrays and plays them sequentially.

    Sending *None* terminates the thread.  Each item plays to completion before
    the next is dequeued, so concurrent frames never interrupt each other.
    """
    try:
        import sounddevice as sd  # type: ignore[import]
        import numpy as np  # type: ignore[import]
    except ImportError:
        log.warning("sounddevice/numpy unavailable — playback thread exiting")
        return

    while True:
        item = play_queue.get()
        if item is None:
            break
        arr = np.array(item, dtype=np.float32)
        try:
            sd.play(arr, samplerate=AUDIO_SAMPLE_RATE)
            sd.wait()
        except Exception as exc:  # noqa: BLE001
            log.debug("playback error: %s", exc)


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

    play_queue: queue.Queue[list[float] | None] = queue.Queue()
    player_thread: threading.Thread | None = None

    if play:
        try:
            import sounddevice  # type: ignore[import]  # noqa: F401
            import numpy  # type: ignore[import]  # noqa: F401

            player_thread = threading.Thread(
                target=_playback_worker,
                args=(play_queue,),
                daemon=True,
                name="lora-wav-player",
            )
            player_thread.start()
        except ImportError:
            log.warning("sounddevice/numpy not available — disabling playback")
            play = False

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
                samples = synthesise_frame(payload=payload, crc_ok=crc_ok)
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

            if play:
                # Load from WAV (so cached files are played identically)
                with wave.open(str(wav_path)) as wf:
                    raw = wf.readframes(wf.getnframes())
                # Convert 16-bit PCM → float list for the queue
                n_frames = len(raw) // 2
                arr = [
                    struct.unpack_from("<h", raw, i * 2)[0] / 32768.0
                    for i in range(n_frames)
                ]
                play_queue.put(arr)

    except KeyboardInterrupt:
        pass
    finally:
        if player_thread is not None:
            play_queue.put(None)  # signal shutdown
            player_thread.join(timeout=2.0)
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
