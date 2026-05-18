---
date: 2026-05-15T14:13:36+0200
author: Tom Hensel
commit: ee2d173
branch: iio-on-device
repository: chirpmunk-gr4
topic: "IIO CI Build + Deploy + Decode Test"
tags: [iio, ci, cross-build, deploy, decode-test, companion, header-valid]
status: in-progress
last_updated: 2026-05-15T14:13:36+0200
last_updated_by: Tom Hensel
type: bug_fix
---

# Handoff: IIO CI Build + Deploy + Decode Test

## Task(s)
1. ✅ Resume from handoff `2026-05-15_05-54-12_iio-merge-and-multi-bw-debug.md`
2. ✅ Commit stash cleanup (post-merge dead code removal)
3. ✅ Fix CI armv7 cross-build (libiio v0.26 issues)
4. ✅ Deploy new binary to tezuka (CI artifact download + scp)
5. 🔴 Decode failure — root cause identified: preamble sync works (LOCK), but **header decode fails (header_valid=0) on EVERY detected signal**.

## Critical References
- `.github/workflows/_cross-build.yml` — CI cross-build pipeline (libiio CMake flags, PKG_CONFIG_PATH)
- `apps/CMakeLists.txt:88-105` — IIO smoke target (CLI11 optional for cross-build)
- `apps/graph_builder.hpp:205-233` — IIO source block creation in build_rx_graph
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp` — multi-BW decode pipeline, `processLaneSymbol()` header validation logic

## Recent changes
- `apps/graph_builder.hpp` — removed IIOSourceMimo include + nRadio==2 dead code, use_aa_filter=false
- `apps/lora_trx.cpp` — IIOSink<cf32> single-port, find_iio_source() for logging
- `apps/config-iio-remote.toml` — use_aa_filter=false, formatting
- `apps/CMakeLists.txt:88-105` — CLI11 optional (find_package QUIET, skip iio_smoke if not found)
- `.github/workflows/_cross-build.yml` — libiio v0.26 CMake flags + PKG_CONFIG_PATH + sed patch for NEED_LIBXML2
- `test/qa_lora_graph_builder_iio.cpp` — updated for integrated build_rx_graph API

## CI fixes (5 commits on iio-on-device after merge)
1. `cleanup(iio)` — remove IIOSourceMimo, fix IIOSink single-port, update tests
2. `fix(ci)` — disable IIOD/AIO/ZSTD/SERIAL in libiio cross-build
3. `fix(ci)` — sed out NEED_LIBXML2 in libiio CMakeLists.txt (bootlin sysroot lacks libxml2)
4. `fix(ci)` — set PKG_CONFIG_PATH for libiio pkg-config discovery
5. `fix(ci)` — make iio_smoke optional (CLI11 not in sysroot), fix compile_defs inside conditional

## Learnings
- **libiio v0.26 cross-build**: `WITH_NETWORK_BACKEND=ON` unconditionally sets `NEED_LIBXML2=1` (CMakeLists.txt:396). Requires `-DWITH_IIOD=OFF -DWITH_AIO=OFF -DWITH_ZSTD=OFF` to break dependency chain (IIOD→local→AIO/libaio→XML). Bootlin glibc sysroot has no libxml2 headers.
- **PKG_CONFIG_PATH required**: `pkg_check_modules` in CMake doesn't search sysroot automatically. Set `PKG_CONFIG_PATH` to `${SYSROOT}/usr/lib/pkgconfig` in CI env for cross-build.
- **Comments in bash \\-continued cmake lines break**: `#` in the middle of a `\`-continued command eats all subsequent lines.
- **CLI11 not in sysroot**: Header-only library. `find_package(CLI11 CONFIG REQUIRED)` fails in cross-build. Made optional with `find_package(CLI11 CONFIG QUIET)`.
- **Companion at 10.0.23.152:5000** — Heltec V3 running MeshCore. `lora hwtest transmit --matrix basic --tcp 10.0.23.152:5000` triggers 9 ADVERTs (~15s).
- **`lora hwtest decode`** is the proper harness but runs lora_trx **locally** on the Mac. Mac build lacks IIO support, so it can't connect to remote Pluto via IIO-TCP.
- **Root cause: header decode fails (header_valid=0) on every detected signal**: With `--log-level DEBUG`, every companion packet shows:
  ```
  LOCK sf=8 bw=62500 os=40 sto_int=136 tau_late=4791 cfo_int=-53 cfo_frac=-0.143 sto_frac=0.080 decim_phase=0 bin_hat=219 snr=9.1dB tail=17591(nom=12800)
  DROP sf=8 bw=62500 header_valid=0 pay_len=0 cr=0 crc=0 snr=9.1dB
  ```
  LOCKs on sf=8 are CORRECT (companion transmitting SF8). Preamble sync works fine. Header decode (sync word, CR, payload length, header CRC) fails 100% of the time on this binary. ~20 LOCK/DROP pairs in 60s, SNR range -2.4 to 9.2dB.

## Artifacts
- `.github/workflows/_cross-build.yml` — CI cross-build with IIO/libiio fixes
- `apps/CMakeLists.txt:88-105` — optional iio_smoke
- `tmp/bw-test/config-ondevice-multibw.toml` — multi-BW test config for tezuka
- `thoughts/shared/handoffs/2026-05-15_05-54-12_iio-merge-and-multi-bw-debug.md` — previous handoff
- CI artifact: `gr4-lora-armv7-eabihf-ee2d173...` (deployed to tezuka `~/lora/bin/lora_trx`)

## Action Items & Next Steps
1. **Investigate header_valid=0 failure**: Root cause is clear — preamble sync works, header decode fails. Possible avenues:
   - Sync word filtering in promiscuous mode — config has `sync_word=0x12` in `[trx.transmit]`, companion uses 0x2B (MeshCore). Verify promiscuous mode accepts any sync word in `PreambleSync`.
   - Header CRC / CR decode failure — check `header_valid` logic and `decodeHeader()` in `processLaneSymbol()` in `MultiSfDecoder.hpp`.
   - Regression check: compare against `lora_trx.bak` (2.6MB, same test config).
2. **Run A/B with old binary**: Swap `lora_trx.bak` back in, run same `--log-level DEBUG` test with companion. If old binary decodes, regression is in this build's code.
3. **Multi-BW decode**: Separate concern — investigate once single BW decodes reliably.
4. **Uncommitted stash changes**: Project-rename WIP (~20 files) still unstaged from stash@{0} pop.

## Other Notes
- New binary on tezuka: `~/lora/bin/lora_trx` (6.4MB, CI-built commit ee2d173, IIO-enabled)
- Backup: `~/lora/bin/lora_trx.bak` (old binary, 2.6MB)
- Configs on tezuka: `/tmp/config-single.toml` (BW=62500 only), `/tmp/config-ondevice-multibw.toml` (3 BWs)
- Companion at `10.0.23.152:5000` — responds to `lora hwtest transmit` commands
- Remote Pluto (tezuka) at `10.0.23.149` — ping ~11ms RTT
- Old config files in `~/lora/etc/` use legacy format (`uri` field, `use_aa_filter=true`, `rx_antenna`)
- `lora hwtest decode --test-config apps/test-configs/b210-vs-pluto.toml --tcp 10.0.23.152:5000` exists but runs lora_trx locally
