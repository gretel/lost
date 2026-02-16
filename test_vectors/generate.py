#!/usr/bin/env python3
"""Generate LoRa PHY test vectors using GNU Radio 3.10 gr-lora_sdr blocks.

Produces binary reference files for each stage of the TX chain, plus a full
TX→RX loopback verification. These files are used by GR4 tests to validate
the LoRa port against the known-good GR3 implementation.

Configuration: MeshCore (SF=8, BW=62.5kHz, CR=4/8, sync=0x12, explicit, CRC)
"""

import json
import os
import struct
import sys
import numpy as np
from pathlib import Path

from gnuradio import gr, blocks, lora_sdr
import pmt

# ---------------------------------------------------------------------------
# Configuration — MeshCore parameters
# ---------------------------------------------------------------------------
CONFIG = {
    "sf": 8,
    "bw": 62500,
    "cr": 4,
    "sync_word": 0x12,
    "impl_head": False,
    "has_crc": True,
    "preamble_len": 8,
    "soft_decoding": True,
    "payload": "Hello MeshCore",  # 14 bytes — exercises CRC and several interleaver blocks
}

OUTDIR = Path(__file__).parent
SAMP_RATE = CONFIG["bw"] * 4  # 250 kS/s, os_factor = 4


# ---------------------------------------------------------------------------
# TX chain — capture every intermediate stage
# ---------------------------------------------------------------------------
class TxVectorGenerator(gr.top_block):
    """Wire the TX chain manually (not hier block) so we can tap each stage."""

    def __init__(self, cfg):
        gr.top_block.__init__(self, "TX Vector Generator")

        sf = cfg["sf"]
        bw = cfg["bw"]
        cr = cfg["cr"]
        impl_head = cfg["impl_head"]
        has_crc = cfg["has_crc"]
        payload = cfg["payload"]
        sync_word = cfg["sync_word"]
        preamble_len = cfg["preamble_len"]

        samp_rate = SAMP_RATE

        # --- TX blocks ---
        # Use file_source with comma-separated payload to get exactly 1 frame
        import tempfile

        self._tmpfile = tempfile.NamedTemporaryFile(
            mode="w", suffix=".txt", delete=False
        )
        self._tmpfile.write(payload + ",")
        self._tmpfile.flush()
        self._tmpfile.close()

        self.file_src = blocks.file_source(gr.sizeof_char, self._tmpfile.name, False)
        self.whitening = lora_sdr.whitening(False, False, ",", "")
        self.header = lora_sdr.header(impl_head, has_crc, cr)
        self.add_crc = lora_sdr.add_crc(has_crc)
        self.hamming = lora_sdr.hamming_enc(cr, sf)
        self.interleav = lora_sdr.interleaver(cr, sf, 0, bw)
        self.gray_dmap = lora_sdr.gray_demap(sf)

        # Zero-pad: enough silence for RX frame_sync to finish processing
        zero_pad = 2**sf * int(samp_rate / bw) * 5
        self.modulate = lora_sdr.modulate(
            sf, samp_rate, bw, [sync_word], zero_pad, preamble_len
        )

        # --- IQ output sink ---
        self.sink_iq = blocks.vector_sink_c()

        # --- Stream connections ---
        self.connect(self.file_src, self.whitening)
        self.connect(self.whitening, self.header)
        self.connect(self.header, self.add_crc)
        self.connect(self.add_crc, self.hamming)
        self.connect(self.hamming, self.interleav)
        self.connect(self.interleav, self.gray_dmap)
        self.connect(self.gray_dmap, self.modulate)
        self.connect(self.modulate, self.sink_iq)


def generate_tx_iq(cfg):
    """Generate the full TX IQ frame and return as numpy array."""
    print("Generating TX IQ frame...")
    tb = TxVectorGenerator(cfg)
    tb.run()  # run() blocks until all sources exhaust → clean single-frame output

    # Clean up temp file
    os.unlink(tb._tmpfile.name)

    iq = np.array(tb.sink_iq.data(), dtype=np.complex64)
    if len(iq) == 0:
        print("ERROR: No IQ data produced. Check gr-lora_sdr installation.")
        sys.exit(1)
    print(f"  TX IQ: {len(iq)} samples ({len(iq) / SAMP_RATE * 1000:.1f} ms)")
    return iq


# ---------------------------------------------------------------------------
# Compute TX intermediates using pure-Python reimplementation of trivial ops
# (These are intentionally simple — the GR3 blocks are the ground truth for
# the IQ output. These intermediates let us unit-test individual GR4 blocks.)
# ---------------------------------------------------------------------------

# Whitening sequence from gr-lora_sdr/lib/tables.h
WHITENING_SEQ = [
    0xFF,
    0xFE,
    0xFC,
    0xF8,
    0xF0,
    0xE1,
    0xC2,
    0x85,
    0x0B,
    0x17,
    0x2F,
    0x5E,
    0xBC,
    0x78,
    0xF1,
    0xE3,
    0xC6,
    0x8D,
    0x1A,
    0x34,
    0x68,
    0xD0,
    0xA0,
    0x40,
    0x80,
    0x01,
    0x02,
    0x04,
    0x08,
    0x11,
    0x23,
    0x47,
    0x8E,
    0x1C,
    0x38,
    0x71,
    0xE2,
    0xC4,
    0x89,
    0x12,
    0x25,
    0x4B,
    0x97,
    0x2E,
    0x5C,
    0xB8,
    0x70,
    0xE0,
    0xC0,
    0x81,
    0x03,
    0x06,
    0x0C,
    0x19,
    0x32,
    0x65,
    0xCB,
    0x96,
    0x2C,
    0x58,
    0xB1,
    0x62,
    0xC5,
    0x8B,
    0x16,
    0x2D,
    0x5A,
    0xB4,
    0x69,
    0xD2,
    0xA4,
    0x48,
    0x91,
    0x22,
    0x44,
    0x88,
    0x10,
    0x21,
    0x43,
    0x86,
    0x0D,
    0x1B,
    0x36,
    0x6D,
    0xDB,
    0xB6,
    0x6C,
    0xD8,
    0xB0,
    0x60,
    0xC1,
    0x83,
    0x07,
    0x0E,
    0x1D,
    0x3A,
    0x75,
    0xEA,
    0xD4,
    0xA8,
    0x51,
    0xA2,
    0x44,
    0x89,
    0x13,
    0x26,
    0x4C,
    0x99,
    0x33,
    0x66,
    0xCC,
    0x98,
    0x31,
    0x63,
    0xC7,
    0x8F,
    0x1E,
    0x3C,
    0x79,
    0xF2,
    0xE4,
    0xC8,
    0x90,
    0x20,
    0x41,
    0x82,
    0x05,
    0x0A,
    0x15,
    0x2A,
    0x54,
    0xA9,
    0x52,
    0xA5,
    0x4A,
    0x95,
    0x2B,
    0x56,
    0xAD,
    0x5B,
    0xB7,
    0x6E,
    0xDC,
    0xB9,
    0x72,
    0xE5,
    0xCA,
    0x94,
    0x29,
    0x53,
    0xA6,
    0x4D,
    0x9B,
    0x37,
    0x6F,
    0xDE,
    0xBD,
    0x7A,
    0xF5,
    0xEB,
    0xD6,
    0xAC,
    0x59,
    0xB2,
    0x64,
    0xC9,
    0x92,
    0x24,
    0x49,
    0x93,
    0x27,
    0x4E,
    0x9D,
    0x3B,
    0x77,
    0xEE,
    0xDD,
    0xBA,
    0x74,
    0xE8,
    0xD1,
    0xA3,
    0x46,
    0x8C,
    0x18,
    0x30,
    0x61,
    0xC3,
    0x87,
    0x0F,
    0x1F,
    0x3E,
    0x7D,
    0xFB,
    0xF6,
    0xEC,
    0xD9,
    0xB3,
    0x67,
    0xCE,
    0x9C,
    0x39,
    0x73,
    0xE6,
    0xCD,
    0x9A,
    0x35,
    0x6B,
    0xD7,
    0xAE,
    0x5D,
    0xBB,
    0x76,
    0xED,
    0xDA,
    0xB5,
    0x6A,
    0xD5,
    0xAA,
    0x55,
    0xAB,
    0x57,
    0xAF,
    0x5F,
    0xBE,
    0x7C,
    0xF9,
    0xF3,
    0xE7,
    0xCE,
    0x9C,
    0x39,
    0x72,
    0xE4,
    0xC8,
    0x91,
    0x23,
    0x46,
    0x8D,
    0x1B,
    0x36,
    0x6C,
    0xD9,
    0xB2,
    0x65,
    0xCA,
    0x95,
    0x2A,
    0x55,
    0xAA,
    0x54,
    0xA8,
    0x50,
    0xA1,
    0x43,
]


def compute_whitening(payload_bytes):
    """Whitening + split into nibbles (low nibble first, high nibble second)."""
    nibbles = []
    for i, b in enumerate(payload_bytes):
        whitened = b ^ WHITENING_SEQ[i]
        nibbles.append(whitened & 0x0F)  # low nibble
        nibbles.append((whitened >> 4) & 0x0F)  # high nibble
    return nibbles


def compute_header_insert(nibbles, payload_len, cr, has_crc):
    """Prepend 5-nibble explicit header."""
    # Header nibbles (high nibble of payload_len first, per LoRa spec)
    n0 = (payload_len >> 4) & 0x0F
    n1 = payload_len & 0x0F
    n2 = (cr << 1) | (1 if has_crc else 0)

    # Header checksum (5 bits: c4, c3, c2, c1, c0)
    # Matches gr-lora_sdr/lib/header_impl.cc lines 125-129 exactly.
    a0 = (n0 >> 3) & 1
    a1 = (n0 >> 2) & 1
    a2 = (n0 >> 1) & 1
    a3 = n0 & 1
    a4 = (n1 >> 3) & 1
    a5 = (n1 >> 2) & 1
    a6 = (n1 >> 1) & 1
    a7 = n1 & 1
    a8 = (n2 >> 3) & 1
    a9 = (n2 >> 2) & 1
    a10 = (n2 >> 1) & 1
    a11 = n2 & 1

    c4 = a0 ^ a1 ^ a2 ^ a3
    c3 = a0 ^ a4 ^ a5 ^ a6 ^ a11
    c2 = a1 ^ a4 ^ a7 ^ a8 ^ a10
    c1 = a2 ^ a5 ^ a7 ^ a9 ^ a10 ^ a11
    c0 = a3 ^ a6 ^ a8 ^ a9 ^ a10 ^ a11

    n3 = c4
    n4 = (c3 << 3) | (c2 << 2) | (c1 << 1) | c0

    header = [n0, n1, n2, n3, n4]
    return header + nibbles


def compute_crc16(data):
    """CRC-16/CCITT (poly 0x1021, init 0x0000)."""
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def compute_add_crc(nibbles_with_header, payload_bytes):
    """Append 4 CRC nibbles to the frame."""
    pay_len = len(payload_bytes)
    if pay_len < 2:
        crc = compute_crc16(b"")
    else:
        crc = compute_crc16(payload_bytes[:-2])
    # XOR with last 2 bytes
    if pay_len >= 1:
        crc ^= payload_bytes[-1]
    if pay_len >= 2:
        crc ^= payload_bytes[-2] << 8

    # 4 CRC nibbles (low byte first, low nibble first within each byte)
    crc_lo = crc & 0xFF
    crc_hi = (crc >> 8) & 0xFF
    crc_nibbles = [
        crc_lo & 0x0F,
        (crc_lo >> 4) & 0x0F,
        crc_hi & 0x0F,
        (crc_hi >> 4) & 0x0F,
    ]
    return nibbles_with_header + crc_nibbles


def int2bool(val, n_bits):
    """Integer to MSB-first bit list."""
    return [(val >> (n_bits - 1 - i)) & 1 for i in range(n_bits)]


def bool2int(bits):
    """MSB-first bit list to integer."""
    val = 0
    for b in bits:
        val = (val << 1) | b
    return val


def compute_hamming_enc(nibbles, sf, cr):
    """Hamming encode each nibble. First sf-2 always use cr=4.

    Matches GR3 hamming_enc_impl.cc exactly:
      data_bin = int2bool(nibble, 4)  # MSB-first: data_bin[0]=MSB, data_bin[3]=LSB
      p0 = data_bin[3] ^ data_bin[2] ^ data_bin[1]
      output = data_bin[3]<<7 | data_bin[2]<<6 | ... | p0<<3 | ...
    The key: data_bin[3] (LSB of nibble) goes to MSB of output codeword.
    """
    encoded = []
    for idx, nib in enumerate(nibbles):
        cr_app = 4 if idx < (sf - 2) else cr
        d = int2bool(nib, 4)  # d[0]=MSB, d[1], d[2], d[3]=LSB

        if cr_app == 1:
            # Parity only (4/5): GR3 output = d[3]<<4 | d[2]<<3 | d[1]<<2 | d[0]<<1 | p4
            p4 = d[0] ^ d[1] ^ d[2] ^ d[3]
            codeword = [d[3], d[2], d[1], d[0], p4]
        else:
            # Hamming (4, 4+cr): GR3 uses data_bin[] indices directly
            p0 = d[3] ^ d[2] ^ d[1]
            p1 = d[2] ^ d[1] ^ d[0]
            p2 = d[3] ^ d[2] ^ d[0]
            p3 = d[3] ^ d[1] ^ d[0]
            # Output: data_bin[3]<<7 | data_bin[2]<<6 | ... >> (4-cr_app)
            full = [d[3], d[2], d[1], d[0], p0, p1, p2, p3]
            codeword = full[: 4 + cr_app]

        encoded.append(bool2int(codeword))
    return encoded


def compute_interleave(encoded, sf, cr):
    """Diagonal block interleaver with LDRO for first block."""
    symbols = []
    idx = 0
    is_first = True

    while idx < len(encoded):
        if is_first:
            sf_app = sf - 2
            cw_len = 8  # always 4+4 for first block
        else:
            sf_app = sf
            cw_len = 4 + cr

        block_size = sf_app
        block = encoded[idx : idx + block_size]
        if len(block) < block_size:
            # Pad with zeros if needed
            block = block + [0] * (block_size - len(block))
        idx += block_size

        # Build binary matrix: sf_app rows × cw_len columns
        matrix = []
        for cw in block:
            matrix.append(int2bool(cw, cw_len))

        # Diagonal interleaving: inter[i][j] = matrix[(i-j-1) mod sf_app][i]
        inter = [[0] * sf_app for _ in range(cw_len)]
        for i in range(cw_len):
            for j in range(sf_app):
                inter[i][j] = matrix[(i - j - 1) % sf_app][i]

        # For first block / LDRO: append parity bit + zero
        for i in range(cw_len):
            row = inter[i]
            if is_first:
                parity = 0
                for b in row:
                    parity ^= b
                row.append(parity)
                row.append(0)

            symbols.append(bool2int(row))

        is_first = False

    return symbols


def compute_gray_demap(symbols, sf):
    """Binary to Gray + offset by 1."""
    N = 1 << sf
    result = []
    for s in symbols:
        gray = s
        shift = s
        for _ in range(1, sf):
            shift >>= 1
            gray ^= shift
        result.append((gray + 1) % N)
    return result


def compute_tx_intermediates(cfg):
    """Compute all TX intermediate representations using Python reimplementation."""
    payload = cfg["payload"]
    sf = cfg["sf"]
    cr = cfg["cr"]
    has_crc = cfg["has_crc"]

    payload_bytes = list(payload.encode("utf-8"))

    print("Computing TX intermediates...")

    # Stage 1: Whitening
    whitened = compute_whitening(payload_bytes)
    print(f"  01_whitened: {len(whitened)} nibbles")

    # Stage 2: Header Insert
    with_header = compute_header_insert(whitened, len(payload_bytes), cr, has_crc)
    print(f"  02_with_header: {len(with_header)} nibbles")

    # Stage 3: Add CRC
    with_crc = compute_add_crc(with_header, payload_bytes)
    print(f"  03_with_crc: {len(with_crc)} nibbles")

    # Stage 4: Hamming Encode
    encoded = compute_hamming_enc(with_crc, sf, cr)
    print(f"  04_encoded: {len(encoded)} codewords")

    # Stage 5: Interleaver
    interleaved = compute_interleave(encoded, sf, cr)
    print(f"  05_interleaved: {len(interleaved)} symbols")

    # Stage 6: Gray Demap
    gray_mapped = compute_gray_demap(interleaved, sf)
    print(f"  06_gray_mapped: {len(gray_mapped)} symbols")

    return {
        "payload_bytes": payload_bytes,
        "whitened": whitened,
        "with_header": with_header,
        "with_crc": with_crc,
        "encoded": encoded,
        "interleaved": interleaved,
        "gray_mapped": gray_mapped,
    }


# ---------------------------------------------------------------------------
# RX loopback test — feed TX IQ through the full RX chain
# ---------------------------------------------------------------------------
class RxLoopbackTest(gr.top_block):
    """Feed TX IQ through the RX chain and capture decoded payload."""

    def __init__(self, iq_data, cfg):
        gr.top_block.__init__(self, "RX Loopback Test")

        sf = cfg["sf"]
        bw = cfg["bw"]
        cr = cfg["cr"]
        impl_head = cfg["impl_head"]
        has_crc = cfg["has_crc"]
        sync_word = cfg["sync_word"]
        soft_decoding = cfg["soft_decoding"]
        preamble_len = cfg["preamble_len"]
        samp_rate = SAMP_RATE
        os_factor = int(samp_rate / bw)
        center_freq = 868100000  # doesn't matter for loopback, just for SFO calc

        # Source: feed the TX IQ data
        self.src = blocks.vector_source_c(iq_data.tolist(), False)

        # RX chain
        self.frame_sync = lora_sdr.frame_sync(
            center_freq, bw, sf, impl_head, [sync_word], os_factor, preamble_len
        )
        self.fft_demod = lora_sdr.fft_demod(soft_decoding, True)
        self.gray_map = lora_sdr.gray_mapping(soft_decoding)
        self.deinterleav = lora_sdr.deinterleaver(soft_decoding)
        self.hamming_dec = lora_sdr.hamming_dec(soft_decoding)
        self.header_dec = lora_sdr.header_decoder(impl_head, cr, 255, has_crc, 0, False)
        self.dewhiten = lora_sdr.dewhitening()
        self.crc_verif = lora_sdr.crc_verif(True, True)

        # Output sinks
        self.payload_sink = blocks.vector_sink_b()
        self.crc_sink = blocks.vector_sink_b()

        # Stream connections
        self.connect(self.src, self.frame_sync)
        self.connect(self.frame_sync, self.fft_demod)
        self.connect(self.fft_demod, self.gray_map)
        self.connect(self.gray_map, self.deinterleav)
        self.connect(self.deinterleav, self.hamming_dec)
        self.connect(self.hamming_dec, self.header_dec)
        self.connect(self.header_dec, self.dewhiten)
        self.connect(self.dewhiten, self.crc_verif)
        self.connect((self.crc_verif, 0), self.payload_sink)
        self.connect((self.crc_verif, 1), self.crc_sink)

        # Message feedback: header_decoder → frame_sync
        self.msg_connect(
            (self.header_dec, "frame_info"), (self.frame_sync, "frame_info")
        )

        # Also capture payload via message port for verification
        self.msg_sink = blocks.message_debug()
        self.msg_connect((self.crc_verif, "msg"), (self.msg_sink, "store"))


def run_rx_loopback(iq_data, cfg):
    """Feed TX IQ through RX chain and return decoded payload."""
    print("Running RX loopback...")
    tb = RxLoopbackTest(iq_data, cfg)
    tb.run()  # blocks until vector_source_c exhausts

    payload_bytes = list(tb.payload_sink.data())
    crc_flags = list(tb.crc_sink.data())

    print(f"  RX decoded: {len(payload_bytes)} bytes, CRC flags: {crc_flags}")
    return payload_bytes, crc_flags


# ---------------------------------------------------------------------------
# File I/O helpers
# ---------------------------------------------------------------------------
def save_u8(path, data):
    with open(path, "wb") as f:
        f.write(bytes(data))


def save_u32(path, data):
    with open(path, "wb") as f:
        for v in data:
            f.write(struct.pack("<I", v))


def save_cf32(path, data):
    data.astype(np.complex64).tofile(path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print(f"LoRa Test Vector Generator")
    print(f"  SF={CONFIG['sf']}, BW={CONFIG['bw']}, CR=4/{4 + CONFIG['cr']}")
    print(f'  Payload: "{CONFIG["payload"]}" ({len(CONFIG["payload"])} bytes)')
    print(f"  Sample rate: {SAMP_RATE} S/s (os_factor={SAMP_RATE // CONFIG['bw']})")
    print()

    # Save configuration
    with open(OUTDIR / "config.json", "w") as f:
        json.dump(CONFIG, f, indent=2)

    # Save raw payload
    with open(OUTDIR / "payload.txt", "w") as f:
        f.write(CONFIG["payload"])

    # Compute TX intermediates (Python reimplementation)
    intermediates = compute_tx_intermediates(CONFIG)

    save_u8(OUTDIR / "tx_01_whitened_nibbles.u8", intermediates["whitened"])
    save_u8(OUTDIR / "tx_02_with_header.u8", intermediates["with_header"])
    save_u8(OUTDIR / "tx_03_with_crc.u8", intermediates["with_crc"])
    save_u8(OUTDIR / "tx_04_encoded.u8", intermediates["encoded"])
    save_u32(OUTDIR / "tx_05_interleaved.u32", intermediates["interleaved"])
    save_u32(OUTDIR / "tx_06_gray_mapped.u32", intermediates["gray_mapped"])

    # Generate TX IQ using actual GR3 blocks (the ground truth)
    iq = generate_tx_iq(CONFIG)
    save_cf32(OUTDIR / "tx_07_iq_frame.cf32", iq)

    # Run RX loopback to verify round-trip
    payload_bytes, crc_flags = run_rx_loopback(iq, CONFIG)

    # Verify round-trip (take first frame only)
    expected = list(CONFIG["payload"].encode("utf-8"))
    pay_len = len(expected)
    first_frame = payload_bytes[:pay_len]
    first_crc = crc_flags[:1] if crc_flags else []

    # Save only first-frame results
    save_u8(OUTDIR / "rx_decoded_payload.bin", first_frame)
    save_u8(OUTDIR / "rx_crc_flags.u8", first_crc)

    if first_frame == expected and first_crc == [1]:
        print()
        print("SUCCESS: RX loopback matches TX payload, CRC valid.")
    else:
        print()
        print("WARNING: RX loopback mismatch!")
        print(f"  Expected: {expected}")
        print(f"  Got:      {first_frame}")
        print(f"  CRC:      {first_crc}")

    # Summary
    print()
    print("Generated files:")
    for p in (
        sorted(OUTDIR.glob("*.u8"))
        + sorted(OUTDIR.glob("*.u32"))
        + sorted(OUTDIR.glob("*.cf32"))
        + sorted(OUTDIR.glob("*.bin"))
        + sorted(OUTDIR.glob("*.json"))
        + sorted(OUTDIR.glob("*.txt"))
    ):
        if p.name == "generate.py":
            continue
        sz = p.stat().st_size
        print(f"  {p.name:35s} {sz:>8d} bytes")


if __name__ == "__main__":
    main()
