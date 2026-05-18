---
date: 2026-05-17T20:34:58+0200
author: Tom Hensel
commit: 0c0d784
branch: iio-on-device
repository: gr4-lora
topic: "LoRa payload CRC decode fully fixed — 3 bugs squashed"
tags: [crc, dewhitening, implicit-header, sdr, decode, lora-phy, handoff]
status: complete
last_updated: 2026-05-17T20:34:58+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: CRC decode fully fixed — all 3 bugs squashed

## Task(s)

### COMPLETED — Fix payload CRC-16 decode on IIO local backend

Goal: get `crc=OK` for both SF7 explicit (promisc chain) and SF8 implicit (sf8-implicit chain) frames from the MeshCore companion (Heltec V3 + SX1262) on the tezuka PlutoSDR via local IIO (`uri=local:`).

Outcome: All three independent bugs that masked each other identified and fixed. Verified on-device with real companion ADVERTs — both SF7 and SF8 show `crc=OK`.

#### Bug 1: Wrong CRC algorithm (XOR-2 variant)

`lora_payload_crc` used `CRC16(payload[0..N-3]) ^ payload[N-2] ^ (payload[N-1] << 8)` instead of plain `CRC16(payload[0..N-1])`. SX1262 computes plain CRC-16/XMODEM (poly 0x1021, init 0x0000) over all payload bytes.

**Fix** (`58b9765`): `lora_payload_crc` and `lora_verify_crc` now use `crc16(payload)` directly.

#### Bug 2: Implicit mode frame length included spurious +8 header symbols

`DecodeChain::reset()` used `_frame_symb_numb = 8u + computePayloadSymbols()` unconditionally. Implicit mode has no header symbols — frame length should be just `computePayloadSymbols()`. The +8 caused the decoder to read 8 symbols into the next frame before finishing, corrupting CRC.

**Fix** (`0c0d784`): Implicit mode path now uses `_frame_symb_numb = computePayloadSymbols()`.

#### Bug 3: CRC bytes not dewhitened

SX1262 whitens the entire data stream including CRC nibbles. `dewhiten()` only dewhitens `pay_len` bytes. CRC bytes were left whitened, so CRC comparison was against whitened CRC vs unwhitened computed CRC. Since nibble-level dewhitening is equivalent to byte-level XOR (nibbles in disjoint bit positions), `bytes[pay_len] ^= whitening_seq[pay_len]` recovers the dewhitened CRC byte.

**Fix** (`0c0d784`): After `dewhiten()` in `finishFrame()`, XOR the 2 CRC bytes with `whitening_seq[pay_len]` and `whitening_seq[pay_len+1]`.

### PENDING — TX CRC whitening

`add_crc` in `tx_chain.hpp` appends raw (unwhitened) CRC nibbles. For TX to SX1262 receivers, CRC nibbles should be whitened. Low priority — only affects local loopback tests which are self-consistent.

## Critical References

- `blocks/include/gnuradio-4.0/lora/algorithm/crc.hpp` — CRC functions: `lora_payload_crc`, `lora_verify_crc`, `crc16`
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp` — `finishFrame()` (CRC dewhitening at ~line 338), `reset()` (frame_symb_numb at ~line 70)
- `blocks/include/gnuradio-4.0/lora/algorithm/tx_chain.hpp` — `dewhiten()`, `add_crc()`
- `apps/config-iio-local.toml` — tezuka config with sf8-implicit chain

## Recent changes

- `58b9765` — CRC algorithm: XOR-2 → plain CRC-16/XMODEM (`crc.hpp`)
- `fe650d2` — Fix -Werror in dead debug code (`HalfBandDecimator.hpp:411`, `MultiSfDecoder.hpp:665`)
- `0c0d784` — CRC dewhitening + implicit mode frame length fix (`DecodeChain.hpp:68-78, 338-349`)

Also from earlier session:

- `9945f03` — TX enable (tx_queue_depth, RF bandwidth fix)
- `e3376c9` — IIOSource ring buffer for local IIO buffer-boundary fix
- `5d58c52` / `809be34` / `6c7b0e6` — Implicit header mode plumbing

## Learnings

1. **SX1262 LoRa CRC is plain CRC-16/XMODEM** (poly 0x1021, init 0x0000) over ALL payload bytes. No XOR-2 variant.

2. **SX1262 whitens CRC bytes** as part of the data stream. All data nibbles after the header are XOR'd with the whitening sequence — payload AND CRC.

3. **Nibble-level dewhitening ≅ byte-level XOR**: Since `dewhiten()` assembles `byte = (hi << 4) | lo` and nibbles occupy disjoint bit positions, `bytes[k] ^= whitening_seq[k]` accomplishes the same as nibble-by-nibble XOR. This makes post-hoc CRC dewhitening trivial.

4. **Implicit mode has no header symbols**. `_frame_symb_numb` must not include the +8 header offset. Without this fix, the decoder reads into the next frame.

5. **Three independent bugs masked each other**: wrong CRC algorithm, missing CRC dewhitening, wrong frame length for implicit mode. Fixing any single one was insufficient.

6. **Local IIO buffer-boundary discontinuities** (previous session): Each `_buf.refill()` returns one kernel buffer. Without ring buffer, stride decode picks samples across boundaries. Fix: ring buffer in IIOSource. Remote IIO (`uri=ip:`) is continuous (iiod stitches kernel buffers into TCP stream).

## Artifacts

- `thoughts/shared/handoffs/2026-05-17_2100-crc-fix.md` — intermediate handoff (updated with all fixes)
- `apps/config-iio-local.toml` — tezuka config: 2 RX chains (promisc + sf8-implicit)
- CI runs: 25998349777 (final, successful), 25998101425 (CRC algorithm only)

## Action Items & Next Steps

1. **[LOW]** TX CRC whitening: modify `add_crc` in `tx_chain.hpp` to whiten CRC nibbles before appending to nibble stream. Required for TX-to-SX1262 compatibility.
2. **[FUTURE]** RF loopback test: connect PlutoSDR TX port to RX port with SMA cable + 30dB attenuator to verify end-to-end decode quality.
3. **[FUTURE]** Multi-chain deployment: add explicit SF8 chain alongside sf8-implicit for comparison.
4. **[ARCHIVE]** Clean up old handoffs if they're superseded.

## Other Notes

- **Tezuka Pluto**: FISH Ball board, 1T1R, AD9361, IP 10.0.23.149, analog password
- **Companion**: Heltec V3 (ESP32+SX1262), SF8 implicit (CR=1, pay_len=28), SF7 explicit, ~30s ADVERT interval
- **Sample rate**: 2.5 MS/s (OS=40, stride decode), no AA filter
- **Binary**: `~/lora/bin/lora_trx` on tezuka (6.1 MB armv7), commit 0c0d784
- **Config**: `~/lora/etc/config-tezuka-tx-test.toml`
- **Deploy**: download CI artifact, `tar cf - bin/lora_trx | sshpass -p "analog" ssh root@10.0.23.149 "cd ~/lora && tar xf -"`
- **Quick test**: `python3 -c "import socket,cbor2; sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.sendto(cbor2.dumps({'type':'lora_tx','seq':1,'payload':bytes(range(23))}), ('10.0.23.149',5561))"`
