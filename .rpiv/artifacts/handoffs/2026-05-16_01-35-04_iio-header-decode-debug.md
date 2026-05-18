---
date: 2026-05-16T01:35:04+0200
author: Tom Hensel
commit: 2d04b20
branch: iio-on-device
repository: gr4-lora
topic: "IIO Header Decode Debug — Header Validation Failure on ARMv7"
tags: [implementation, debug, iio, armv7, decode, phy]
status: complete
last_updated: 2026-05-16T01:35:04+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: IIO header_valid=0 on tezuka (ARMv7) — works on macOS

## Task(s)

**Completed:**
1. Fixed stale IIO config format (`uri = "local:"` → `param = "uri=local:"`) — resolved IIOSource ERROR state blocking all decode
2. Optimized `non_blocking=true` → `false` — confirmed +10dB preamble SNR, faster lock on local backend
3. Reverted `decim_phase` fix — tail adjustment (`tau_late -= frac_corr`) already handles fractional STO; adding `decim_phase = frac_corr` causes double-compensation → zero decodes

**Open:** `header_valid=0` — preamble locks solid (SNR up to 10.7dB, 8 locks/40s) but **every** header decode fails. Does NOT occur on macOS. Likely an ARMv7-specific issue (32-bit, VFPv3 float, alignment, or compiler).

## Critical References

- `blocks/include/gnuradio-4.0/lora/phy/PreambleSync.hpp` — preamble detection + CFO/STO estimation
- `blocks/include/gnuradio-4.0/lora/phy/CssDemod.hpp` — `set_cfo_correction()` + `demodHard()` dechirp FFT
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp` — pipeline orchestration, LOCK handler at ~line 400-510
- `apps/graph_builder.hpp:228` — `non_blocking=false` with rationale comment
- `apps/config-iio-local.toml` / `apps/config-iio-remote.toml` — reference configs (use `param` not `uri`)
- Tezuka: `10.0.23.149`, root/analog, `~/lora/bin/lora_trx`, `~/lora/etc/config-tezuka.toml`

## Recent changes

- `apps/graph_builder.hpp:225-237` — `non_blocking: true → false` with comment explaining rationale
- `apps/lora_trx.cpp:583-594` — updated scheduler comment to reflect `non_blocking=false`
- `apps/config-iio-local.toml:5` — already correct (`param = "uri=local:"`)
- Tezuka config fixed: `param = "uri=local:"` replacing stale `uri = "local:"`
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp:478-490` — reverted to original (no `decim_phase` change)

## Learnings

1. **Config field migration**: After `feat/iio-sink-unification` merge, `[device].uri` replaced by `[device].param`. Old `uri` key silently ignored → empty `device_param` → IIOSource with `uri=""` → ERROR state. Lesson: grep for stale config keys when debugging IIO failures.

2. **non_blocking=false confirmed better on local backend**: Blocking `refill()` gives full 32768-sample buffers vs ETIMEDOUT/partial reads. Remote backend unaffected (ENOSYS from setBlockingMode → buffer stays blocking).

3. **decim_phase is NOT the fix**: The tail adjustment (`tau_late -= frac_corr`) already compensates fractional STO in the time domain (extending/shortening SFD tail). Setting `decim_phase = frac_corr` shifts the decimation window AGAIN → double-compensation → misaligned output symbols.

4. **header_valid=0 is platform-specific**: Works on macOS (Apple Silicon ARM64), fails on tezuka (ARMv7 Cortex-A9). Same code, same config. Candidates: 32-bit type sizes, VFPv3 vs NEON float, alignment, GCC vs Clang.

## Artifacts

- `thoughts/shared/handoffs/2026-05-16_01-35-04_iio-header-decode-debug.md` — this handoff
- Commits on `iio-on-device`: `d610991` (non_blocking=false), `7b79a7a` (docs), `2d04b20` (revert decim_phase)
- Working binary on tezuka: `~/lora/bin/lora_trx` (commit `d610991` equivalent, md5 `4796fdd...`)

## Action Items & Next Steps

### Primary: Find ARMv7-specific root cause of header_valid=0

**Hypothesis 1 — FFT floating-point differences (VFPv3 vs NEON)**
- ARMv7 Cortex-A9 uses VFPv3 (scalar float). ARM64 uses NEON (SIMD float).
- Subtle differences in `sin/cos` or complex multiply-accumulate could shift FFT bins by 1-2 positions.
- Preamble sync uses 6+ symbols (robust to 1-bin errors). Header uses 8 symbols (less margin).
- **Test**: Run unit tests ON the ARMv7 binary. The CI builds `qa_lora_*` test binaries alongside `lora_trx`. Copy them to tezuka and run `qa_lora_css_demod`, `qa_lora_lib_pipeline`, etc. Compare results with macOS.
- Files: `test/qa_lora_css_demod.cpp`, `test/generate_vectors/`

**Hypothesis 2 — `std::complex<float>` alignment**
- On ARMv7 GCC, `std::complex<float>` may be 4-byte aligned (2× float). On ARM64, 8-byte aligned.
- If SIMD or hand-rolled NEON code expects 8-byte alignment, unaligned access could corrupt data.
- Check: `blocks/include/gnuradio-4.0/lora/detail/FftPool.hpp` and the FFT implementation.
- The `_cfo_downchirp` vector and `dechirp_and_quality` operate on `cf32` (= `std::complex<float>`).

**Hypothesis 3 — 32-bit type size in chirp generation**
- `size_t` = uint32_t on ARMv7, uint64_t on ARM64. If used in calculations that wrap or overflow differently.
- Check: `blocks/include/gnuradio-4.0/lora/detail/ChirpRefs.hpp` — `build_upchirp`, `build_downchirp`
- Check: `blocks/include/gnuradio-4.0/lora/algorithm/utilities.hpp` — `dechirp_and_quality`

**Hypothesis 4 — Compiler optimization difference**
- GCC 15 (Bootlin cross) on ARMv7 vs Clang 20 on macOS.
- `-ffast-math` or similar flags could reorder float operations, changing `sin/cos` results.
- Check: CMakeLists.txt for float-related compiler flags.
- File: `CMakeLists.txt`, `cmake/CompilerWarnings.cmake` or similar.

**Hypothesis 5 — Integer modulo with negative values**
- C++ `%` with negative operands: `-29 % 256 = -29` (truncation toward zero in C++11+).
- `set_cfo_correction` does: `((cfo_int % N) + N) % N` — should be correct on all platforms.
- But check: `cfo_int` is `int32_t` everywhere. What if it's sign-extended differently?
- File: `blocks/include/gnuradio-4.0/lora/phy/CssDemod.hpp:71-73`

### Quick diagnostic tests
1. **ssh to tezuka, run test binaries**: `~/lora/bin/qa_lora_css_demod` etc. (need to scp from CI artifact)
2. **Add per-symbol debug to CssDemod**: Log `demodHard` bin for header symbols vs expected (CR 4/8 header = 8 symbols, fixed whitening sequence). Compare macOS vs tezuka.
3. **Transmit known frame**: Use `lora_trx` TX or companion device, verify the exact bytes transmitted, compare received raw bins.

### Useful commands
```bash
# Deploy CI artifact to tezuka
sshpass -panalog scp -O -o StrictHostKeyChecking=no ./tmp/artifact/bin/lora_trx root@10.0.23.149:~/lora/bin/

# Run with debug
sshpass -panalog ssh root@10.0.23.149 "timeout 60 ~/lora/bin/lora_trx --config ~/lora/etc/config-tezuka.toml --log-level DEBUG 2>&1"

# Trigger CI
gh workflow run "ARMv7 Cross CI" --ref iio-on-device
```

## Other Notes

- Tezuka uses dropbear SSH (no sftp-server). Use `scp -O` for legacy protocol.
- The CI builds test binaries (`qa_lora_*`) alongside `lora_trx` — these are in the artifact's `bin/` directory.
- Current working binary on tezuka is functionally equivalent to commit `d610991` (`non_blocking=false`, no other changes). md5sum: `4796fdd88be5db030c22ce571099bbd6`.
- PreambleSync CFO estimation uses 3-bin parabolic interpolation — the integer part is the FFT peak bin, the fractional part is the interpolated offset. Both should be platform-independent with correct float math, but VFPv3 might produce slightly different `sin/cos` values.
