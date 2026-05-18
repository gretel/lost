---
date: 2026-05-16T14:42:14+0200
author: Tom Hensel
commit: 9f6a520
branch: iio-on-device
repository: gr4-lora
topic: "IIO Header Decode — Diagnostics & Root Cause Isolation"
tags: [debug, iio, phy, diagnostics, stride-decimation, header-decode]
status: complete
last_updated: 2026-05-16T14:42:14+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: Header decode diagnostics confirm stride path correct; root cause isolated to IIO data delivery

## Task(s)

**Completed:**

1. ✅ Added `HDR_SYM` per-symbol header PMR logging (`MultiSfDecoder.hpp` — `output_symbol_count` field)
2. ✅ Deployed to tezuka, captured live data: PMR = 16–35 for ALL header symbols despite `header_valid=0`
3. ✅ **Falsified handoff hypothesis** — stride decimation does NOT lose 97.5% signal energy. Per-symbol PMR proves dechirp+FFT peaks are pristine.
4. ✅ Added OS=40 stride + STO prepend regression tests (`qa_lora_multisf.cpp`) — ALL PASS on macOS
5. ✅ Added `DecodeChain` header-decode trace: `HDR_RAW`, `HDR_NIB`, `HDR_CHK` log lines
6. ✅ Squashed noisy 79-commit history → 1 milestone commit (`9f6a520`), force-pushed
7. ✅ 27/27 LoRa tests pass on macOS

**Verified / ruled out:**

- Stride decimation energy loss: **FALSIFIED** (PMR=16-35)
- Decode logic correctness: **CONFIRMED** (all synthetic tests pass at OS=1, OS=4, OS=40, OS=40+STO offset)
- ARMv7 precision: **CONFIRMED** (all PHY suites pass on tezuka)
- AA filter: not relevant (disabled, `use_aa_filter=false`)
- CFO correction: tested at ±8 kHz synthetic offset
- STO correction at OS=40: **CONFIRMED** (new prepend tests: 10-sample=0.25 Nyquist STO, 20-sample=0.50 Nyquist STO)

**Remaining hypothesis:** IIO data delivery — sample rate accuracy, buffer gaps between refill calls, or AD9361 frontend behavior.

## Critical References

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:90-91` — `output_symbol_count` + `decim_phase` fields
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:553-565` — HDR_SYM per-symbol logging
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:48-50` — `debug` flag in Config
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:188-230` — HDR_RAW/NIB/CHK logging in `processHeaderBlock`
- `test/qa_lora_multisf.cpp:663-703` — OS=40 STO prepend regression tests
- Tezuka: `10.0.23.149`, root/analog, binary at `~/lora/bin/lora_trx` (MD5 `47e449b2`, build `e61f69b`)
- Companion: `10.0.23.152:5000`

## Recent changes

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:91` — added `output_symbol_count` field
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:161` — reset in `resetLane()`
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:553-565` — HDR_SYM log in Output state
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:42` — added `#include <gnuradio-4.0/lora/log.hpp>`
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:50` — `bool debug = false` in Config
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:57` — `_debug = cfg.debug` in init
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:148` — `_debug` member
- `blocks/include/gnuradio-4.0/lora/phy/DecodeChain.hpp:188-230` — HDR_RAW/NIB/CHK trace logging
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:99` — `decode_debug` parameter on `init()`
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:126` — `ec.debug = decode_debug`
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:267` — passes `debug` to `init()`
- `test/qa_lora_multisf.cpp:663-703` — `run_sto_prepend_os40` lambda + 2 OS=40 prepend tests

## Learnings

1. **PMR disproved stride hypothesis**: Per-symbol PMR of 16–35 means every header symbol has an unambiguous FFT peak. Stride decimation at OS=40 produces clean symbols. The `header_valid=0` failure is NOT due to signal degradation.

2. **Synthetic OS=40 tests pass**: New regression tests confirm stride decimation + STO correction works correctly with synthetic data at OS=40, even with 0.25–0.50 Nyquist sample STO offsets. Decode chain is correct.

3. **Root cause is in IIO data path**: With decode logic verified correct, the failure must be in how the IIO source delivers samples. Likely candidates: sample rate mismatch (AD9361 actual ≠ 2.5 MS/s nominal), buffer gaps between `iio_buffer_refill()` calls, or AD9361 frontend transients at symbol boundaries.

4. **TX path broken on current build**: `set_mode_at_init=true` not supported by current gr4-incubator IIOSink. TX cannot be used for loopback testing until this is fixed or removed.

5. **History squashed**: 79 noisy commits → 1 milestone. Backup at `iio-on-device-backup`. Force-pushed.

6. **Tezuka SSH**: uses `sshpass -p 'analog'` with dropbear (no sftp-server). Use `scp -O`.

## Artifacts

- `thoughts/shared/handoffs/2026-05-16_01-51-08_aa-filter-test-confirms-decode-logic-correct.md` — previous handoff (hypothesis now falsified)
- `thoughts/shared/handoffs/2026-05-16_14-42-14_iio-header-decode-diagnostics-and-history-squash.md` — this handoff
- Squashed commit: `9f6a520` on `iio-on-device` (force-pushed)
- Backup branch: `iio-on-device-backup` (79 original commits)
- Diagnostic binary on tezuka: `~/lora/bin/lora_trx` (MD5 `47e449b2`)
- Test additions: `test/qa_lora_multisf.cpp:663-703`

## Action Items & Next Steps

### Primary: Capture raw IIO IQ for offline comparison

When Pluto RF path is confirmed working (antenna, gain, frequency match with companion):

1. **Add raw IQ capture** to lora_trx or a helper: dump first N samples per detected preamble to file
2. **Generate synthetic test vector** matching the transmitted frame (SF/BW/CR/payload)
3. **Compare sample-by-sample**: check for sample rate drift, buffer gaps, amplitude discontinuities
4. **Check IIO buffer continuity**: add logging between `refill()` calls to detect gaps

### Quick diagnostic: use companion TX with tezuka RX

```bash
# Start lora_trx on tezuka
sshpass -p 'analog' ssh root@10.0.23.149 \
  "nohup ~/lora/bin/lora_trx --config ~/lora/etc/config-tezuka.toml --log-level DEBUG > /tmp/lora_diag.log 2>&1 &"

# Transmit from companion
lora hwtest transmit --matrix basic --tcp 10.0.23.152:5000

# Check for HDR_RAW/HDR_NIB/HDR_CHK
sshpass -p 'analog' ssh root@10.0.23.149 \
  "grep -E 'HDR_RAW|HDR_NIB|HDR_CHK|LOCK|DROP' /tmp/lora_diag.log"
```

### Deploy updated binary

```bash
# After CI builds new commit
gh run watch <id> --compact --exit-status --interval 180
ARTIFACT=$(gh api repos/gretel/chirpmunk-gr4/actions/runs/<id>/artifacts --jq '.artifacts[0].name')
rm -rf ./tmp/artifact && mkdir -p ./tmp/artifact
gh run download <id> -n "$ARTIFACT" -D ./tmp/artifact
sshpass -panalog scp -O ./tmp/artifact/bin/lora_trx root@10.0.23.149:~/lora/bin/
```

### Fix TX path (for loopback testing)

Remove or fix `set_mode_at_init=true` at `apps/lora_trx.cpp:163` — not supported by current gr4-incubator IIOSink.

## Other Notes

- PMR (Peak-to-Mean Ratio) from `CssDemod::demodHard`: max_mag / mean_mag across 256 FFT bins. PMR > 10 = clean symbol.
- HDR_SYM diagnostic filter: `grep HDR_SYM` — shows 8 header symbols per frame with raw_bin + pmr
- HDR_RAW diagnostic: shows the 8 raw demod bins fed to DecodeChain
- HDR_NIB: shows the 5 Hamming-decoded nibbles (hex) from header
- HDR_CHK: shows parsed CR, payload_len, has_crc, checksum_valid
- All three HDR\_\* lines appear only when `MultiSfDecoder::debug` is true (`--log-level DEBUG`)
- Uncommitted: `apps/tx_worker.cpp` has SF/BW override changes for TX CBOR requests (pre-existing)
- `use_aa_filter = false` is correct for tezuka — AA filter is broken at OS=40 (assert fires, log2(40) ≠ integer, decimates by 32 not 40)
