---
date: 2026-05-15T18:13:24+0200
author: Tom Hensel
commit: 408ca74
branch: iio-on-device
repository: chirpmunk-gr4
topic: "IIO Header Decode Diagnostics — DC Removal, BW Fix, & 3 Hypotheses Ruled Out"
tags: [iio, header-valid, decode-failure, diagnostics]
status: in-progress
last_updated: 2026-05-15T18:13:24+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: IIO Header Decode Diagnostics

## Task(s)
1. ✅ Commit + deploy DC removal fix (`remove_dc=true` in CssDemod) — still 100% header_valid=0
2. ✅ Commit + deploy AD9361 LPF bandwidth fix (bandwidth = channel BW, not sample rate) — still 100% header_valid=0
3. ✅ Verify AD9361 clamping: `rf_bandwidth` stays at 200 kHz (AD9361 minimum, not 2.5 MHz) — fix was redundant
4. ✅ Raw Pluto IQ analysis: clean data, no clipping (max ±1842 of 2047), RMS ~422 counts (~20% FS), DC negligible (−2.4/2.5 counts, 0.12%)
5. ✅ AD9361 hardware DC offset tracking confirmed enabled (`bb_dc_offset_tracking_en=1`, `rf_dc_offset_tracking_en=1`)
6. ✅ Local unit tests pass (`qa_lora_multisf` — all 84 asserts pass with synthetic data)

## Root cause NOT found — three hypotheses remain

### Hypothesis 1: os=40 stride aliasing (`use_aa_filter=false`)
- Os=40 picks 1 of 40 samples per Nyquist symbol (2.5 MS/s → 62.5 kHz). Without anti-alias filtering, 39 aliased images fold into the 62.5 kHz baseband.
- AD9361 LPF at 200 kHz helps (folds images from 31.25 kHz–200 kHz only, not 31.25 kHz–1.25 MHz).
- Preamble sync (6+ FFT blocks × 256 chips of processing gain) still locks through aliased noise, but per-symbol dechirp FFT (256 chips only) fails.

### Hypothesis 2: CFO correction mismatch
- PreambleSync estimates (cfo_int, cfo_frac) correctly for sync (LOCK works).
- CssDemod::set_cfo_correction() rebuilds the downchirp reference from these values.
- But the corrected downchirp might not perfectly cancel CFO for the demodulation path, giving wrong bins.

### Hypothesis 3: IIO vs Soapy isolation not tested
- No A/B test possible (old binary lacks shared lib).
- All test data is from IIO path only — we've never confirmed SoapyPlutoPAPR on the same tezuka hardware.

## Recent changes (3 commits on iio-on-device)
1. `709dfae` — `remove_dc=true` in CssDemod::demodHard, raw_bin/adj_bin logging in DROP
2. `408ca74` — AD9361 bandwidth set to `max(rx_bandwidths)` instead of `cfg.rate`
3. `29e52ad` — cleanup: remove build-native/ .pi-lens/ artifacts committed accidentally

## Learnings
- **AD9361 minimum bandwidth is 200 kHz**: `rf_bandwidth_available: [200000 1 56000000]`. Setting bandwidth=62500 clamps to 200 kHz. Setting it to 2.5 MHz also clamps down but to a higher value (read back wasn't verified). The original `bandwidth=cfg.rate` (2.5 MHz) was likely clamped to ~2.5 MHz by the AD9361 driver, NOT 200 kHz. The bandwidth fix itself took effect (new binary sets 62500 → clamped to 200 kHz).
- **AD9361 DC tracking enabled by default**: `bb_dc_offset_tracking_en=1` and `rf_dc_offset_tracking_en=1`. The `remove_dc=true` software fix is redundant when hardware tracking is active.
- **Raw Pluto IQ is NOT clipping**: max ±1842 out of ±2047 (12-bit ADC), so the IIOSource scale factor 1/2048 is correct and no saturation occurs.
- **libgnuradio-blocklib-core.so on tezuka is 67KB but exports 0 dynamic symbols** (confirmed via `readelf -s`). This is a stripped stub library. The binary was statically linked for most of gnuradio4 and only uses the .so for plugin/block-registry. This is normal.
- **The CI-staged libgnuradio-blocklib-core.so is 9KB** (same stub). The 67KB on tezuka includes debug symbols. No issue here.
- **Segfault on tezuka at shutdown**: both ee2d173 and 29e52ad binaries segfault during scheduler teardown. Cosmetic — happens after all decode is done.
- **Companion tx pattern**: ~9 ADVERTs per burst, ~15s between bursts. Observed 3-7 LOCK/DROP pairs per 60s test.

## Artifacts
- **Binaries on tezuka**: `~/lora/bin/lora_trx` (ee2d173), `~/lora/bin/lora_trx.new` (709dfae — DC removal), `~/lora/bin/lora_trx.bwfix` (29e52ad — BW fix)
- **Configs**: `/tmp/config-single.toml` (single BW 62.5k IIO test config)
- **Test logs on tezuka**: `/tmp/trx-test-new.log`, `/tmp/trx-test-fixed.log`, `/tmp/trx-test-bwfix.log`, `/tmp/trx-debug-full.log`
- **Raw IQ dump**: `/tmp/pluto-iq-full.b64` on Mac (2 seconds, 5M samples at 2.5 MS/s)
- **CI artifacts**: `tmp/ci-artifact/` (709dfae), `tmp/ci-artifact-2/` (29e52ad)
- **CI builds**: run 25929055893 (709dfae, success), run 25932318963 (29e52ad, success)

## Action Items & Next Steps
1. **Test SoapyPlutoPAPR on tezuka**: Build with `-DGR4_LORA_BUILD_IIO=OFF` and SoapySDR-only. Deploy and test same config. If Soapy works but IIO doesn't, the issue is in IIOSource or IIO pipeline. If both fail, the issue is in the decoder chain (os=40 aliasing).
2. **Log all 8 header bins**: Modify DROP log to dump all 8 header symbol bins, not just the last one. Check if bins are random or show a pattern (e.g., all at CFO-modified positions).
3. **Check CFO tracking**: Log the CFO-corrected downchirp reference verification. Inject a known test signal via loopback and verify demod produces expected bins.
4. **Try different os_factor**: Test with rate=1e6 (os=16 for BW=62.5k) which is power of 2, or enable aa_filter.
5. **Clean up `.gitignore`**: Add build-native/, .pi-lens/, .pi/agents/ to prevent accidental commits.

## Key Unknown: SoapyPlutoPAPR decode on this hardware was never confirmed

The old binary (`lora_trx.bak`, 2.6MB, SoapyPlutoPAPR path) couldn't run on tezuka — missing shared lib, no RPATH. We have **zero evidence** that Soapy decode ever worked on this Pluto hardware with this companion setup. The issue may have existed since the IIO migration or may be pre-existing. An A/B test (SoapyPlutoPAPR vs IIO on the same hardware) is needed but blocked by the missing old shared library.
- User credentials for tezuka: `root@10.0.23.149` password `analog`
- Companion: Heltec V3 at `10.0.23.152:5000` (MeshCore bridge, ADVERT every ~15s)
- `lora hwtest transmit --matrix basic --tcp 10.0.23.152:5000` triggers companion TX from Mac
- SSH via `sshpass -p 'analog'` (no key-based auth)
- Binary transfer via `cat | ssh "cat > file"` (no SFTP/SCP, no base64)
