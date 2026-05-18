---
date: 2026-05-17T19:26:54+0200
author: Tom Hensel
commit: 6c7b0e6
branch: iio-on-device
repository: gr4-lora
topic: "Local IIO stride decode fixed — ring buffer + implicit header"
tags:
  [
    iio,
    local-iiobackend,
    stride-decode,
    ring-buffer,
    buffer-boundary,
    implicit-header,
    sf8,
    bug-fix,
    handoff,
  ]
status: in_progress
last_updated: 2026-05-17T19:26:54+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: Local IIO stride decode working — SF7 explicit + SF8 implicit header

## Task(s)

### [COMPLETED] Task 1: Fix TX loopback (IIOSink TX path)

TX was dead — `enable_tx=true` but `tx_queue_depth=0` made `TxRequestQueue(0)` always full.
**Root cause**: `tx_queue_depth = 0` in config-iio-local.toml. Every TX request rejected "tx_queue_full".
**Fix**: `tx_queue_depth = 8`. Also fixed TX RF bandwidth from `cfg.rate` (sample rate) to `cfg.bw` (62.5 kHz).
**Verified**: CBOR request → IIO buffer push → DAC. No loopback without RF cable.

### [COMPLETED] Task 2 (evolved): Fix local IIO stride decode — ring buffer

Local IIO (`uri=local:`) stride decode failed every frame (100% `csum=0`). Previously attributed to "stride aliasing" at OS=40. **Real root cause**: each `_buf.refill()` returns one kernel buffer via `read()` on the IIO char device. Without ring buffer, consecutive `read()` calls produce non-contiguous sample blocks. Stride picks every 40th sample — a boundary crossing corrupts the chirp.
**Fix**: Ring buffer in IIOSource accumulates all refills into a continuous circular buffer. processBulk drains from ring head — downstream sees seamless stream.
**Verified**: SF7 decode `crc=OK` on local IIO. First working stride decode on this path ever.

### [COMPLETED] Task 3: Implicit header mode for SF8

MeshCore companion transmits SF8 with **implicit header mode** (no header symbols). DecodeChain always expected explicit 5-nibble header → read random payload bits → `csum=0`.
**Fix**: Added `implicit_header` flag to DecodeChain.Config. Skip CollectingHeader, go straight to CollectingPayload with configured CR + payload_len.
**Three bugs fixed in pipeline**:

1. `graph_builder.hpp` only plumbed 2/5 override keys from `block_overrides`. Added `implicit_header`, `implicit_cr`, `implicit_pay_len` lookups.
2. `init()` set `_phase = CollectingPayload` but `reset()` overwrote to `CollectingHeader` after each frame.
3. `reset()` fixed to check `_cfg.implicit_header` and set phase accordingly.
   **Verified**: SF8 frames emitted with 28-byte payload data. No HDR_NIB from implicit chain.

### [PENDING] Task 4: Payload CRC failure

Both SF7 (explicit) and SF8 (implicit) decodes show `crc=FAIL` on payload CRC-16. Decoded payload bytes look like valid MeshCore ADVERT data (28-byte payload). Likely causes: wrong CRC polynomial, frame format includes extra fields, or CRC computed over different byte range.

## Critical References

- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:68-100` — implicit header init + reset logic
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — ring buffer implementation
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:100-106` — PerSfDecoder::init() wiring

## Recent changes

- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — added ring buffer `_ring[]`, `_ringHead/Tail`, `appendRefill()`, `compactRing()`, `ensureRingCapacity()`; modified `processBulk` to refill → ring → drain → compact
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp` — added `implicit_header`, `default_cr`, `default_pay_len` to Config; `computePayloadSymbols()` helper; init/reset respect implicit_header flag
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp` — added `implicit_header`, `implicit_cr`, `implicit_pay_len` properties, GR_MAKE_REFLECTABLE entries, plumbing through PerSfDecoder::init()
- `apps/graph_builder.hpp` — added `implicit_header/CR/pay_len` lookups from `block_overrides`
- `apps/lora_trx.cpp` — TX RF bandwidth fix: `cfg.rate` → `cfg.bw`
- `apps/config-iio-local.toml` — `enable_tx=true`, `tx_queue_depth=8`, `tx_lo_powerdown=false`, added `sf8-implicit` chain with `implicit_header=true`

## Learnings

1. **Local IIO `uri=local:` has buffer-boundary discontinuities.** Each `_buf.refill()` returns one kernel buffer via `read()`. Consecutive reads produce non-contiguous blocks. Remote IIO (`uri=ip:`) is continuous because iiod concatenates kernel buffers into TCP stream. The ring buffer fix applies to both modes but is only required for local backend.
   - IIOSource bugs discovered earlier (seify) don't apply — local IIO channels ARE enabled before create_buffer.

2. **"Stride aliasing at OS=40" was a red herring.** The actual root cause was buffer-boundary data corruption. The handoff's data (3 different headers from same ADVERT) was explained by stride hitting different discontinuities each time. Properly continuous stride decode works at OS=40.

3. **MeshCore companion uses implicit header for SF8, explicit for SF7.** The DecodeChain must be configured per-chain with `implicit_header=true` for SF8. The TOML `[[trx.receive.chain]]` config supports unknown keys → `block_overrides` → GR4 reflectable properties, but `graph_builder.hpp` only applied `min_snr_db` and `max_symbols`. All override keys must be explicitly plumbed.

4. **`DecodeChain.reset()` is called after every frame.** Any per-frame state (like implicit_header) must be preserved across resets. The `_cfg` member persists.

5. **`gh workflow run` may race with git push.** The workflow may pick up a stale commit if triggered before the push fully propagates. Always verify `origin/<branch>` SHA matches expected before triggering CI.

6. **SSH to tezuka has connection-pool exhaustion.** After ~5 rapid connections in <30s, password auth gets locked out. Wait 5-15s between attempts or use `PreferredAuthentications=password`.

## Artifacts

- `thoughts/shared/handoffs/2026-05-17_15-50-00_iio-tx-fix.md` — previous handoff (TX fix, AA filter analysis)
- `apps/config-iio-local.toml` — local IIO config with TX + sf8-implicit chain
- Tezuka deployed files:
  - `~/lora/bin/lora_trx` — commit 6c7b0e6 (armv7)
  - `~/lora/etc/config-tezuka-tx-test.toml` — two RX chains (promisc + sf8-implicit)

## Action Items & Next Steps

1. **Fix payload CRC-16** — SF7 and SF8 both show `crc=FAIL`. Investigate CRC polynomial (likely CRC-16-MCRF4XX or CRC-16-IBM, poly 0x8005 or 0x1021). Check `crc.hpp` in lora-sdr-impl for the current implementation. Also verify whether the ADVERT frame format includes the CRC in the payload or appends it separately.

2. **Test with RF loopback cable** — Connect PlutoSDR TX port to RX port with SMA cable + 30dB attenuator. Verify AA filter decode quality with strong local TX. This was blocked by physical isolation on FISH Ball.

3. **Deploy across all 3 chains** — The current config has two chains (promisc + sf8-implicit). Consider adding explicit SF8 chain for comparison, or configuring per-SF CRC/implicit settings.

4. **(Future) Single-stage polyphase decimator** — If AA filter performance is needed for weak signals, the half-band cascade's band-edge droop (-6 dB) is fundamental. A single-stage polyphase decimator with N=512+ taps would give flat passband at the cost of ~8 MACs/output at 62.5 kS/s (negligible).

## Other Notes

- **Tezuka Pluto details**: FISH Ball board, 1T1R mode, AD9361, analog password, IP 10.0.23.149
- **Companion**: Heltec V3 (ESP32+SX1262), default 2 dBm TX power, max +22 dBm
- **Deployed config**: Two RX chains — `promisc` (SF7-12 explicit) and `sf8-implicit` (SF8 implicit, CR=1, pay_len=28)
- **Sample rate**: 2.5 MS/s (OS=40, stride decode). No AA filter (use_aa_filter=false)
- **CI**: ARMv7 Cross CI on `gretel/chirpmunk-gr4`, branch `iio-on-device`. Artifacts contain `lora_trx` binary at path `tmp/artifact-*/gr4-lora-armv7-*/bin/lora_trx`
- **Key command for quick test**: `python3 -c "import socket,cbor2; sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.sendto(cbor2.dumps({'type':'lora_tx','seq':1,'payload':bytes(range(23))}), ('10.0.23.149',5561)); sock.close()"`
