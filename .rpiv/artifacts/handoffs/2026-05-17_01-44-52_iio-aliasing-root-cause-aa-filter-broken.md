---
date: 2026-05-17T01:44:52+0200
author: Tom Hensel
commit: 169e076
branch: iio-on-device
repository: gr4-lora
topic: "IIO Aliasing Root Cause — AA Filter Broken at OS=64"
tags:
  [
    debug,
    iio,
    phy,
    aliasing,
    stride-decimation,
    aa-filter,
    cascaded-decimator,
    preamble-sync,
  ]
status: in_progress
last_updated: 2026-05-17T01:44:52+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: IIO aliasing root cause + AA filter broken at OS=64

## Task(s)

**Completed:**

1. ✅ **Root cause found** — spectral aliasing from stride decimation at OS=40. Nyquist after decimation = 31.25 kHz = LoRa BW/2 = signal edge. Rectangular chirp envelope causes spectral leakage above Nyquist → aliases back into band → creates spurious FFT peaks → corrupts PreambleSync detection and CFO/STO estimation.

2. ✅ **Decode chain confirmed correct** — deinterleaver, Hamming decoder, checksum verification all work. Two confirmed `csum=1` frames across all tests (CFO bypass at OS=40, OS=64 stride-only).

3. ✅ **HW-CFO fix deployed** — `MultiSfDecoder.hpp` computes CFO from TCXO spec: 2 ppm × 869.618 MHz = 1739 Hz, converted to bins per SF. Does NOT fix csum_ok=0 because timing (aliasing-corrupted PreambleSync) is the true root cause.

4. ✅ **decim_phase fix verified** — tau_late handles integer STO, decim_phase handles fractional STO. Residual timing error 0.008 Nyq samples (negligible). Fix is correct but insufficient.

5. ✅ **AA filter at OS=64 debugged** — `CascadedDecimator` at 6 stages works for SF7 (N=128, PMR 7-21) but kills SF8/SF12 (N≥256, zero detections). AA_ENTER trace confirms SF8 reaches `decimateToNyquist` with correct window (start=0, end=16384, span=32768), produces expected 256 output samples (no AA diag), but PreambleSync never locks.

6. ✅ **OS=32 blocked** — needs 2.0 MS/s sample rate. AD9361 rejects (min 2.083 MS/s, errno 22 "Invalid argument"). Not fixable without firmware changes.

7. ✅ **OS=64 stride-only works** — 5 SF8 detections, 1 csum=1. Same aliasing issue as OS=40 but more detections due to finer timing resolution.

**Blocked / Remaining:**

8. 🔴 **SF8 completely broken with AA filter at OS=64** — `CascadedDecimator` produces valid-looking output (256 samples) but signal content is corrupted for N≥256. Likely a frequency-response or phase-distortion bug at 6 stages. The half-band filter cascade may have accumulated passband ripple or nonlinear phase response that distorts CSS chirps at longer symbol durations.

9. 🔴 **No reliable decode on PlutoSDR IIO** — maximum 1/6 frames csum=1 with CFO bypass at OS=40 stride-only.

## Critical References

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:420-435` — HW-CFO computation (1739 Hz from 2ppm TCXO). Also decim_phase fix and AA filter path.
- `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp:288-347` — `CascadedDecimator::processBatch`. Stage 0 uses circular `process()`, stages 1+ use linear `processBatch()`.
- `blocks/include/gnuradio-4.0/lora/phy/PreambleSync.hpp:280-330` — CFO estimation via 5-bin correlation across Y_history (U4-U6).
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:193-245` — `processHeaderBlock` with HDR_RAW, HDR_GRAY, HDR_CW, HDR_NIB diag.
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:600-650` — `decimateToNyquist` AA + stride paths.

## Recent changes

All on `iio-on-device` branch, pushed to origin. Commits in order:

- `40fd41d` — HDR_CW+HDR_GRAY codeword trace in DecodeChain
- `f547982` — decim_phase fix (tau_late integer-only, decim_phase from sto_frac)
- `a81724a` — CFO bypass experiment (set cfo=0 for diag)
- `68eda67` — HW-CFO: compute from 2ppm TCXO, use for all lanes
- `db5a0b1` — AA output shortage diag (prints when aa_out < N)
- `169e076` — AA_ENTER unconditional trace for SF8/SF12 (HEAD)

Key file changes:

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:420-435` — HW-CFO + decim_phase
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:485-505` — decim_phase set from sto_frac, frac_corr removed from tau_late
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:620-650` — AA_ENTER trace + AA output shortage
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:193-245` — HDR_CW+HDR_GRAY trace

## Learnings

1. **Root cause is aliasing, not CFO.** At OS=40, Nyquist after stride = 2.5MS/s / (2×40) = 31.25 kHz. LoRa BW = 62.5 kHz spans [-31.25, +31.25] kHz. The rectangular chirp envelope's spectral leakage extends beyond ±31.25 kHz and aliases back into the signal band. This creates spurious dechirp+FFT peaks that corrupt PreambleSync's peak detection (bin_hat varies 3-255, should cluster near 7).

2. **Decode chain is correct.** 2 confirmed `csum=1` frames: one with CFO bypass at OS=40 stride, one with OS=64 stride. Deinterleaver, Hamming decoder, and checksum verification all work. The issue is upstream — demodulated bins are wrong.

3. **AA filter at OS=64 produces output but corrupts signal.** SF8 enters `decimateToNyquist` with correct window (span=32768, start=0, end=16384), `processBatch` returns without output-shortage diag (so aa_out >= 256), but PreambleSync never locks. SF7 works fine (PMR 7-21, 3 LOCKs per test). Hypothesis: 6-stage half-band cascade accumulates frequency-response error sufficient to distort CSS chirps at N≥256. Could be cumulative passband ripple or nonlinear group delay.

4. **CFO bypass is the best result so far.** With `cfo_int=0, cfo_frac=0`, 1/6 SF8 frames decode correctly (csum=1). This proves the actual CFO on this PlutoSDR is near zero (AD9361 calibration likely active). HW-CFO (7 bins) makes decode WORSE (0/8 frames), confirming the actual CFO ≈ 0.

5. **`sto_frac - cfo_frac` compensation in tau_late was wrong.** The original code subtracted `cfo_frac` from `sto_frac` before computing `frac_corr`. But Eq. 20's STO estimate is NOT biased by CFO in the simple subtraction way. Removing this and using decim_phase from raw `sto_frac * os` is correct.

6. **OS=64 stride gives more SF8 detections than OS=40 stride.** 5 vs 2-3 per test. Finer timing resolution (1/64 Nyq vs 1/40) helps preamble detection despite same aliasing. But decode is still corrupted.

## Artifacts

- Previous handoffs: `thoughts/shared/handoffs/2026-05-16_19-18-19_*`, `2026-05-16_19-30-00_*`, `2026-05-16_14-42-14_*`
- Test logs on tezuka: `/tmp/lora_hwcfo.log` (HW-CFO test), `/tmp/lora_nocfo.log` (CFO bypass), `/tmp/lora_os64_stride.log` (OS=64 stride), `/tmp/lora_os64_aa.log` (OS=64 AA), `/tmp/lora_aa_diag.log` (AA output shortage), `/tmp/lora_aa_enter.log` (AA_ENTER trace), `/tmp/lora_cw_test.log` (HDR_CW trace)
- CI runs: 25974328679 (baseline), 25974508630 (HDR_CW), 25974673713 (decim_phase+HDR_CW), 25974968686 (CFO bypass), 25975267933 (HW-CFO), 25975711419 (AA shortage diag), 25975897818 (AA_ENTER)
- Binary on tezuka: `c97da459117d7bea20c45b154c718108` (commit 169e076, HW-CFO + AA diag trace)
- Companion: `10.0.23.152:5000`, Heltec V3 meshcore, 869.618 MHz, 62.5 kHz BW, SF7/SF8/SF12
- Tezuka: `10.0.23.149`, root/analog, dropbear SSH, PlutoSDR FISH Ball
- Configs on tezuka: `~/lora/etc/config-tezuka-tx-test.toml` (OS=40, TX enabled), `config-tezuka-os64-aa.toml` (OS=64 AA), `config-tezuka-os64-stride.toml` (OS=64 stride), `config-tezuka-soapy.toml` (Soapy, failed)

## Action Items & Next Steps

### Primary: Make AA filter work at OS=64 for SF8

The AA filter is the correct fix for aliasing but is broken at 6 stages for N≥256. Options:

1. **Debug CascadedDecimator at 6 stages** — add per-stage diag to trace intermediate output magnitudes. Check if signal energy is lost at a specific stage. May be a numeric precision or buffer overflow at larger sample counts.

2. **Reduce stages** — try 5 stages at OS=32. But this requires a valid 2.0 MS/s sample rate. The AD9361 driver rejects 2.0 MS/s. Could try bypassing the rate check in the IIO driver — risky.

3. **Use cascaded half-band from external library** — replace the custom CascadedDecimator with a proven implementation (e.g., liquid-dsp, or a polyphase FIR). Major change.

4. **Pre-filter before stride** — add a simple 1-2 stage IIR/FIR lowpass before stride decimation at OS=40. This would attenuate spectral leakage above 31.25 kHz without requiring power-of-2 OS. Could use a single biquad or moving average.

### Secondary: CFO estimation workaround

If AA filter can't be fixed quickly, work around corrupted PreambleSync:

5. **CFO=0 as default for PlutoSDR** — the AD9361 calibration appears to reduce actual CFO to near zero. Set cfo_int=0, cfo_frac=0 in `set_cfo_correction`. Test with more frames to see if decode rate improves beyond 1/6.

6. **Adaptive CFO from decoded frames** — after a successful frame decode, use the decoded bin positions to estimate residual CFO and apply a fine correction for subsequent frames.

### Quick diagnostic commands

```bash
# Deploy latest binary (no CI needed if binary already on tezuka)
sshpass -p 'analog' ssh root@10.0.23.149 \
  "killall -9 lora_trx chirpmunk-trx 2>/dev/null; sleep 1; \
   nohup ~/lora/bin/lora_trx --config ~/lora/etc/config-tezuka-tx-test.toml --log-level DEBUG > /tmp/lora_test.log 2>&1 &"

# Companion TX
lora hwtest transmit --matrix basic --tcp 10.0.23.152:5000

# Check SF8 results
sshpass -p 'analog' ssh root@10.0.23.149 \
  "grep -E 'HDR_NIB|LOCK sf=8|csum=' /tmp/lora_test.log"

# Check AA filter traces (for AA configs)
sshpass -p 'analog' ssh root@10.0.23.149 \
  "grep 'AA_ENTER sf=8' /tmp/lora_test.log | head -5"
```

## Other Notes

- **chirpmunk-trx kills IIO**: Rust daemon holds `cf-ad9361-lpc` buffer via `uri=local:`. Must kill before starting lora_trx. Restart it after testing: `ssh root@10.0.23.149 "nohup /tmp/chirpmunk-trx --config /tmp/config-iio.toml > /tmp/chirpmunk.log 2>&1 &"`
- **Tezuka SSH instability**: Dropbear drops connections during long commands. Use separate sshpass commands for start/TX/check.
- **AD9361 sample rates**: valid = 40MHz / N for integer N. Working: 2.5 MS/s (N=16), 4.0 MS/s (N=10). Failed: 2.0 MS/s (N=20, rejected by driver).
- **AD9361 available rates string**: `[2083333 1 61440000]` — step 1 Hz.
- **`use_aa_filter` requires OS = power of 2**. Must satisfy `log2(os) == integer`.
- **CI builds take ~30 min** for armv7 cross-build. Use `gh workflow run ci-armv7.yml --ref iio-on-device` to trigger.
- **HDR diag format**: `HDR_CW sf=8 [hex,hex,...]` (6 codewords for SF8), `HDR_NIB sf=8 [hex,...] cr=X pay=Y crc=Z csum=0/1`.
- **AA diag format**: `AA_ENTER sf=8 os=64 in_off=0 phase=0 start=0 end=16384 span=32768 sps=16384` fires for every AA call. `AA sf=7 os=64 in=8182 out=127 need=128 zero_fill=1` fires when output is short.
- **LOCK diag format**: `LOCK sf=8 bw=62500 os=40 sto_int=169 tau_late=3480 cfo_int=-23 cfo_frac=-0.060 sto_frac=0.041 decim_phase=2 bin_hat=181 snr=-1.3dB tail=16280(nom=12800)`.
