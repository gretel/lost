---
date: 2026-05-16T19:18:19+0200
author: Tom Hensel
commit: 4bc9a73
branch: iio-on-device
repository: gr4-lora
topic: "IIO Header Decode — Full Diagnostic Trace"
tags: [debug, iio, phy, stride-decimation, header-decode, gr4-incubator]
status: in_progress
last_updated: 2026-05-16T19:18:19+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: IIO Header Decode — Full Diagnostic Trace

## Task(s)

**Completed:**

1. ✅ **TX path fix** — `set_mode_at_init=true` removed from `apps/lora_trx.cpp:163`. Not supported by current gr4-incubator IIOSink (submodule at main 42fdf80). Commit `fb0dbc8`. Deployed to tezuka.
2. ✅ **chirpmunk-trx IIO conflict** — Rust daemon holds `cf-ad9361-lpc` buffer, starving lora_trx. Kill chirpmunk-trx first. Saved to memory lessons.
3. ✅ **IIO buffer continuity verified** — Per-refill debug logging added to gr4-incubator IIOSource (submodule `fd00c2f`, commit `4bc9a73`). Every refill returns full 32768 samples (131072 bytes, step=4). No gaps.
4. ✅ **AD9361 BW aliasing ruled out** — Tested both 200 kHz (min clamp) and 2.5 MHz. Same corruption at both settings — aliasing not the cause.
5. ✅ **HDR_GRAY trace logging** — `DecodeChain.hpp:170-180` logs gray-decoded shifted symbols before deinterleave. Commit `35280ba`.
6. ✅ **IIOSource `debug` flag** — Wired `cfg.debug` to IIOSource `debug` property via `graph_builder.hpp:238`. `--log-level DEBUG` enables per-refill stderr output.

**In Progress / Remaining:**

7. 🔴 **Root cause not found** — Header decode fails (`csum_ok=0`) on ALL frames despite PMR=14-41 on stride=40 header symbols in real-time lora_trx. Same PMR at stride=4 (offline), stride=10, stride=40 → stride magnitude NOT the root cause.
8. 🔴 **`decim_phase` stuck at 0** — Comment at `MultiSfDecoder.hpp:585-586` says decim_phase should be set from `sto_frac * os`, but the fix was reverted (commit `2d04b20`, revert of `d1a1c78`). Revert message: "tail adjustment already handles fractional STO → double-compensation". However, this does NOT affect PMR (FFT shift theorem preserves magnitude spectra).

## Critical References

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:585-600` — stride decimation loop, `decim_phase` always 0
- `blocks/include/gnuradio-4.0/lora/algorithm/utilities.hpp:55-75` — `build_upchirp` phase formula (corrected from my wrong Python)
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:170-180` — HDR_GRAY trace
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — IIOSource per-refill debug (submodule fd00c2f)

## Recent changes

- `apps/lora_trx.cpp:163` — removed `set_mode_at_init` from IIOSink property map
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — added `debug` flag, per-refill `fprintf` logging
- `apps/graph_builder.hpp:238` — wired `{"debug", cfg.debug}` to IIOSource
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:170-180` — HDR_GRAY trace
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:90-91` — `output_symbol_count`, `decim_phase` fields

## Learnings

1. **Stride PMR is NOT the problem.** lora_trx real-time shows PMR 14-41 on stride=40 header symbols. Offline analysis (same captured IQ) showed PMR 3.9 at stride=40 because the capture missed the actual ADVERT signal (timing mismatch between iio_readdev and companion TX).

2. **iio_readdev capture methodology is flawed.** Companion TX at 2 dBm takes ~20s to transmit 9 ADVERTs. iio_readdev with `--samples N` captures only N/2.5e6 seconds. The window is too narrow to guarantee signal capture. Need to either: (a) start capture AFTER companion TX starts, or (b) capture at full sample rate for 10+ seconds (160 MB).

3. **`decim_phase` always 0** — the fix `d1a1c78` was reverted in `2d04b20`. The revert claims double-compensation with tail adjustment. However, due to FFT shift theorem, decim_phase does NOT affect FFT magnitude or PMR. It can shift bin indices (phase rotation), which would corrupt gray decode values. **This may be the root cause** — need to verify by computing bin offset.

4. **Correct dechirp formula** from `build_upchirp` in `utilities.hpp:55-70`:

   ```
   phi_up(n) = 2*pi * [n^2 / (2*N*os^2) - 0.5*n/os]     for n < L/2
   phi_up(n) = 2*pi * [n^2 / (2*N*os^2) - 1.5*n/os]     for n >= L/2
   downchirp[n] = conj(upchirp[n]) = exp(-j * phi_up(n))
   ```

5. **IIOSink submodule at 42fdf80 (main) does NOT have `set_mode_at_init`** — only the `feat/iio-sink-unification` worktree branch has it. Our IIO TX path uses the main branch version which powers TX LO in `reinitDevice()` via `applyAd9361CenterFrequency()`. The `set_mode_at_init` property was silently ignored before removal.

## Artifacts

- `thoughts/shared/handoffs/2026-05-16_14-42-14_iio-header-decode-diagnostics-and-history-squash.md` — first handoff (stride hypothesis falsified)
- `thoughts/shared/handoffs/2026-05-16_19-30-00_iio-stride-decode-corruption.md` — second handoff (this document, stride verified, heading deeper)
- Commits on `iio-on-device`: `fb0dbc8` (TX fix) → `35280ba` (HDR_GRAY) → `ce1079c` (submodule bump) → `4bc9a73` (IIOSource debug)
- gr4-incubator commit: `fd00c2f` (IIOSource debug logging)
- Tezuka binary: `~/lora/bin/lora_trx` (MD5 at last deploy: `5637f2d9`, commit `4bc9a73`)
- Logs on tezuka: `/tmp/lora_live.log` (latest), `/tmp/lora_refill.log` (debug), `/tmp/lora_gray.log` (HDR_GRAY), `/tmp/lora_trx_tx_test.log` (initial)
- Raw captures on tezuka: `/tmp/long_cap.bin` (160 MB, 20M samples), `/tmp/iio_capture.bin` (16 MB)
- Local offline analysis: `tmp/compare_strides.py`, `tmp/analyze_long.py`, `tmp/check_capture.py`, `tmp/check_capture2.py`
- Config: `tmp/config-tezuka-tx-test.toml` (TX-enabled IIO config)
- Companion: `10.0.23.152:5000`

## Action Items & Next Steps

### Critical Path: Fix Header Decode

With stride PMR confirmed good (14-41), the focus must shift to the decode chain itself. The most likely remaining issue:

**1. Verify `decim_phase` effect on bin indices**

Even though PMR is unaffected by decim_phase (FFT shift theorem), the BIN VALUE may shift. Compute: with `sto_frac=0.282` and `decim_phase=0`, the stride picks `sample[0], sample[40], sample[80], ...` instead of `sample[11], sample[51], sample[91], ...`. This 11-sample offset at the full rate (2.5 MS/s) translates to `11/40` of a Nyquist sample period. After the FFT, the peak bin may be shifted by `round(sto_frac * N) mod N` bins.

**Action:** Compare the HDR_GRAY values against known ADVERT header from the companion meshcore protocol. Or re-enable the decim_phase fix and test.

**2. Add per-decode-stage logging**

Add trace logging for:

- Deinterleaver codeword output (before Hamming decode)
- Hamming-corrected nibbles vs raw codewords (1-bit error rate)
- Gray-decoded symbol values (HDR_GRAY already exists)
- Expected ADVERT header from companion for comparison

**3. Test with OS=4 on tezuka**

Create config with BW=625 kHz at 2.5 MS/s → OS=4. Companion at 62.5 kHz won't match, but use local TX loopback or synthetic vector to verify decode chain works end-to-end through IIO at low stride.

**4. Verify companion ADVERT format**

Check MeshCore companion protocol: what is the exact ADVERT frame structure? Is sync_word=0x12 correct? Is preamble_len=16 correct? Is the header format (CR, payload_len) what the decoder expects?

### Quick Diagnostic Commands

```bash
# Restart lora_trx on tezuka with DEBUG
sshpass -p 'analog' ssh root@10.0.23.149 \
  "killall -9 lora_trx chirpmunk-trx 2>/dev/null; sleep 1; \
   nohup ~/lora/bin/lora_trx --config ~/lora/etc/config-tezuka-tx-test.toml --log-level DEBUG > /tmp/lora_live.log 2>&1 &"

# Send companion ADVERTs
lora hwtest transmit --matrix basic --tcp 10.0.23.152:5000

# Check decode results
sshpass -p 'analog' ssh root@10.0.23.149 \
  "grep -E 'HDR_CHK|HDR_NIB|HDR_GRAY|HDR_RAW|LOCK|DROP' /tmp/lora_live.log | tail -30"

# Check PMR distribution
sshpass -p 'analog' ssh root@10.0.23.149 \
  "grep -oP 'pmr=[\d.]+' /tmp/lora_live.log | sort -n | tail -10"
```

## Other Notes

- **SSH instability**: Dropbear on tezuka drops connections during long `lora hwtest` commands (which use their own `--tcp` connection). Best to: (1) start lora_trx, (2) run companion TX in separate command, (3) check tezuka logs in third command.
- **AD9361 minimum sample rate**: 2.083 MS/s or 2.5 MS/s (from `sampling_frequency_available`: `[2083333 1 61440000]`). The "1" entry is a kernel artifact (1 Hz, not useful).
- **AD9361 minimum BW**: 200 kHz (silently clamps any value below).
- **Companion**: Heltec V3 running meshcore at 869.618 MHz, 62.5 kHz BW, SF7/SF8/SF12, CR 4/8, sync 0x12, preamble 16.
- **Tezuka IP**: 10.0.23.149, root/analog. Uses dropbear (no sftp-server). Use `scp -O`.
- **160 MB capture** at `/tmp/long_cap.bin` on tezuka (20M samples, 8s at 2.5 MS/s) — can be sliced with dd for offline analysis.
- **chirpmunk-trx restart**: Might restart via supervisor on tezuka. Check `ps | grep chirp` and re-kill if needed.
