---
date: 2026-05-16T01:51:08+0200
author: Tom Hensel
commit: 2d04b20
branch: iio-on-device
repository: gr4-lora
topic: "IIO Header Decode — AA Filter Test & Narrowed Root Cause"
tags: [debug, iio, phy, aa-filter, stride-decimation]
status: complete
last_updated: 2026-05-16T01:51:08+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: AA filter test confirms — decode logic correct, issue in stride decimation

## Task(s)

**Completed:**
1. ✅ Fixed stale IIO config format (`uri = "local:"` → `param = "uri=local:"`) — resolved IIOSource ERROR state
2. ✅ Optimized `non_blocking=true` → `false` — +10dB preamble SNR, faster lock
3. ✅ Reverted `decim_phase` fix — tail adjustment already handles fractional STO
4. ✅ **Ran all PHY test suites on ARMv7 tezuka — ALL PASS** (237 asserts). Rules out: FFT precision, sin/cos, 32-bit types, endianness, CFO correction math, DecodeChain logic.
5. ✅ **Tested `use_aa_filter=true`** — SNR collapsed from +10dB to -13dB. `decim_phase` shows garbage values (7, 30, 22) — suggests AA filter buffer overflow corrupting `PerSfDecoder` struct.

## Critical References

- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:90` — `decim_phase` field (never set, but reads garbage with AA on)
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:568-597` — `decimateToNyquist` AA filter path
- `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp` — `CascadedDecimator` (likely buggy at OS=40)
- `apps/graph_builder.hpp:228` — `non_blocking=false` rationale
- Tezuka: `10.0.23.149`, root/analog, `~/lora/bin/lora_trx`, `~/lora/etc/config-tezuka.toml`
- Config currently has `use_aa_filter = true` (set during this test session)
- Test binaries deployed to tezuka at `/tmp/qa_lora_*` + `/tmp/libgnuradio-blocklib-core.so`

## Recent changes

- `apps/graph_builder.hpp:225-237` — `non_blocking: true → false` with rationale comment
- `apps/lora_trx.cpp:583-594` — updated scheduler comment
- `apps/config-iio-local.toml:5` — already correct (`param = "uri=local:"`)
- Tezuka config: `use_aa_filter = true` (for AA filter test — should revert to false after)

## Learnings

1. **Config field migration**: `[device].uri` → `[device].param = "uri=local:"`. Old key silently ignored.

2. **non_blocking=false confirmed optimal**: Full 32768-sample buffers vs ETIMEDOUT partial reads.

3. **PHY algorithms correct on ARMv7**: All 5 test suites pass (237 asserts). No 32-bit, float, or compiler issues.

4. **AA filter is BROKEN at OS=40**: SNR collapses from +10dB to -13dB. `decim_phase` shows garbage values (7, 30, 22) despite never being set — suggests `CascadedDecimator::processBatch` writes beyond its buffer, corrupting the `PerSfDecoder` struct.

5. **header_valid=0 is in the stride decimation path (no AA)**: With clean non-AA stride path, preamble locks at +10dB SNR but every header fails. Preamble uses non-coherent multi-symbol integration (robust). Header demod is per-symbol (fragile). Stride decimation at OS=40 picks 1 of 40 samples — 97.5% energy discarded per symbol.

## Artifacts

- `thoughts/shared/handoffs/2026-05-16_01-35-04_iio-header-decode-debug.md` — previous handoff (superseded by this one)
- `thoughts/shared/handoffs/2026-05-16_01-51-08_aa-filter-test-confirms-decode-logic-correct.md` — this handoff
- Commits on `iio-on-device`: `d610991` (non_blocking=false), `7b79a7a` (docs), `2d04b20` (revert decim_phase)
- Working binary on tezuka: `~/lora/bin/lora_trx` (md5 `4796fdd88be5db030c22ce571099bbd6`)
- Test binaries on tezuka: `/tmp/qa_lora_*`

## Action Items & Next Steps

### Primary: Fix stride decimation at OS=40 without AA filter

**Hypothesis:** Stride decimation loses 97.5% of signal energy per symbol. The preamble (6+ symbols, non-coherent integration) still locks. The header (8 symbols, per-symbol demod) gets wrong bins due to aliased noise → `header_valid=0`.

**Options to test:**

1. **Reduce OS factor** (requires lowering sample rate): At 2.5 MS/s and BW=62.5k, OS=40. If sample rate dropped to 1.25 MS/s, OS=20. If 500 kS/s, OS=8. Check AD9361 minimum sample rate constraints (2.083 MS/s is the IIO default minimum for Pluto).

2. **Fix AA filter for OS=40** — investigate `CascadedDecimator` buffer overflow. Check `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp`. Keys: delay line sizing, output buffer allocation. The filter shows SNR collapse and struct corruption → likely allocating wrong size for OS=40.

3. **Implement proper polyphase decimator** — GR4's built-in `gr::basic::Decimator` block placed BEFORE the splitter would decimate with proper anti-alias before the per-lane stride path. This is the architecturally correct solution (decimate once, not per-lane).

4. **Add per-symbol diagnostic logging** — log actual FFT bin vs expected for header symbols. Add to `CssDemod::demodHard` or to the `processLaneSymbol` Output path. Compare against known-good test vector expectations.

### Quick diagnostic: check AA filter buffer overflow

```bash
# On tezuka, check if process crashes or corrupts memory
# The "decim_phase garbage" is the clue — read HalfBandDecimator
```

### Useful commands
```bash
# Deploy CI artifact
sshpass -panalog scp -O -o StrictHostKeyChecking=no ./tmp/artifact/bin/lora_trx root@10.0.23.149:~/lora/bin/

# Run with debug
sshpass -panalog ssh root@10.0.23.149 "timeout 60 ~/lora/bin/lora_trx --config ~/lora/etc/config-tezuka.toml --log-level DEBUG 2>&1"

# Run test binaries on tezuka
sshpass -panalog ssh root@10.0.23.149 "LD_LIBRARY_PATH=/tmp /tmp/qa_lora_css_demod"

# Toggle AA filter on tezuka config
sshpass -panalog ssh root@10.0.23.149 "sed -i 's/use_aa_filter = .*/use_aa_filter = false/' ~/lora/etc/config-tezuka.toml"

# Trigger CI
gh workflow run "ARMv7 Cross CI" --ref iio-on-device
```

## Other Notes

- Tezuka uses dropbear SSH (no sftp-server). Use `scp -O`.
- `decim_phase` field in `PerSfDecoder:90` reads garbage values (7, 30, 22) with AA filter ON — strong evidence of buffer overflow in AA filter. With AA filter OFF, `decim_phase` shows 0 correctly.
- Test binaries pass on ARMv7 → LoRa PHY algorithms are bit-accurate across platforms.
- The `sfd_tail_remaining` tail adjustment already handles integer+frac STO in time domain. `decim_phase=0` is correct for stride path.
- SNR with AA filter OFF: up to +10.7dB (good). SNR with AA filter ON: -13dB (broken). Something is very wrong with the AA filter at OS=40.
