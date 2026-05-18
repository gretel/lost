---
date: 2026-05-15T05:54:12+0200
author: Tom Hensel
commit: 2d2b237
branch: iio-on-device
repository: gr4-lora
topic: "IIO Merge + Multi-BW Decode Investigation"
tags: [merge, iio, config, multi-bw, decode]
status: in-progress
last_updated: 2026-05-15T05:54:12+0200
last_updated_by: Tom Hensel
type: feature_development
---

# Handoff: IIO Merge + Multi-BW Decode Investigation

## Task(s)
1. ✅ Rename config files: `config.toml` → `config-uhd.toml`, `config-tezuka.toml` → `config-iio-local.toml`, new `config-iio-remote.toml`
2. ✅ Merge `feat/iio-sink-unification` into `iio-on-device` — resolved conflicts, removed dead code, updated config struct
3. ✅ Build all targets, 27/27 tests pass
4. 🔴 Multi-BW decode issue on IIO — `bandwidths=[62500]` decodes, `bandwidths=[62500,125000,250000]` does not

## Critical References
- `apps/graph_builder.hpp` — IIO path in `build_rx_graph()` at line 205-233
- `apps/config-iio-remote.toml` — remote Pluto config (base for BW tests)
- `blocks/include/gnuradio-4.0/lora/MultiSfDecoder.hpp` — stride decimation in `decimateToNyquist()`

## Recent changes
- `apps/config.cpp` — removed `uri` field from parsed config struct, moved to `device_param` as `"uri=..."`
- `apps/config.hpp` — removed `uri` from `TrxConfig` and `ScanSetConfig`
- `apps/graph_builder.hpp` — removed `build_rx_graph_iio()`, `build_scan_graph_iio()`, `build_streaming_scan_graph_iio()`; integrated IIO detection via `cfg.device=="iio"`; removed `IIOSourceMimo.hpp` include and MIMO path; changed IIO gain_mode to `slow_attack`, added `non_blocking=true`, `max_overflow_count=10`
- `apps/lora_trx.cpp` — merged IIO dispatch (now direct `build_rx_graph()` call), added IIOSink TX path (single-port `IIOSink<cf32>`), fixed `rx_blocks.iio_source` → `find_iio_source()`
- `apps/lora_scan.cpp` — removed `cfg.uri.empty()` checks, now uses common `build_scan_graph()`/`build_streaming_scan_graph()` with IIO detection inside
- `test/qa_lora_graph_builder_iio.cpp` — rewritten for new API (`build_rx_graph` instead of `build_rx_graph_iio`, `find_iio_source` instead of `rx_blocks.iio_source`)
- `test/qa_lora_config.cpp` — updated expectations for merged `config-uhd.toml`
- `test/CMakeLists.txt` — `CONFIG_TOML_PATH` → `config-uhd.toml`
- `CMakeLists.txt` — added `GR4_LORA_HAS_IIO=1` to test target

## Learnings
- **IIO config format**: `param = "uri=..."` instead of separate `uri` field (feat/iio-sink-unification merged this change)
- **IIOSourceMimo deleted**: `IIOSourceMimo.hpp` removed from gr4-incubator submodule. IIO path only supports nRadio==1.
- **IIOSink is single-port**: gr4-incubator has `IIOSink<T>`, not `IIOSink<T,nPorts>`. Wire `out0 → in` only.
- **rx_antenna removed from IIO path**: `rf_port` hardcoded per channel index (`rx_channels[0]==1` → `B_BALANCED`, else `A_BALANCED`)
- **IIO detection**: uses `cfg.device == "iio"`, URI from `cfg.device_param` (strips `"uri="` prefix)
- **IIO source settings**: `gain_mode=slow_attack` (AGC), `non_blocking=true`, `max_overflow_count=10`, `buffer_size=32768`
- **Config renames**: `config-uhd.toml` (UHD B210), `config-iio-local.toml` (IIO local Pluto), `config-iio-remote.toml` (IIO remote Pluto)
- `use_aa_filter` must be `false` for OS values that aren't powers of 2 (the assert at MultiSfDecoder.hpp:143 fires otherwise)
- gr4-incubator submodule pinned to `fc70844` (ahead of `feat/iio-block`, includes non_blocking + progress fix + overflowCount)
- Stash pop after merge restored pre-existing WIP changes (project rename `gr4-lora` → `chirpmunk-gr4` across many files) which are still unstaged
- **Multi-BW on IIO does not decode**: single BW (`bandwidths=[62500]`) works on remote Pluto, but `[62500,125000,250000]` yields 0 frames. UHD path handles multi-BW fine. Root cause not identified.

## Artifacts
- `apps/config-uhd.toml` — UHD B210 config (was `config.toml`)
- `apps/config-iio-local.toml` — IIO local Pluto config (was `config-tezuka.toml`)
- `apps/config-iio-remote.toml` — IIO remote Pluto config (new)
- `tmp/bw-test/config-bw-*.toml` — BW test configs (need `bandwidths` duplicate fix — both commented and uncommented lines present)
- `thoughts/shared/handoffs/2026-05-15_05-54-12_iio-merge-and-multi-bw-debug.md`

## Action Items & Next Steps
1. **Diagnose multi-BW decode failure on IIO**: determine why `bandwidths=[62500,125000,250000]` fails. Test each BW individually, then pairs:
   - `bandwidths=[125000]` only → does it decode?
   - `bandwidths=[250000]` only → does it decode?
   - `bandwidths=[62500,125000]` → does either decode?
   - `bandwidths=[125000,250000]` → does either decode?
2. **Fix Pluto reachability**: `lora_trx --config apps/config-iio-remote.toml` got 0 frames in latest test (single BW). Check if Pluto IIO context connects, if AD9361 is streaming, if there's a companion transmitting.
3. **Commit or clean stash changes**: the stash pop restored project-rename WIP (unstaged). Either commit the rename or discard.
4. **Update skill files**: `~/.agent/skills/tezuka-dev/SKILL.md` and `~/.agent/skills/gr4-dev/references/gr4_lora_app_config.md` reference old config filenames (`config-pluto.toml`, `config.toml`). Home dir write was blocked — ask user for access or provide manual edit instructions.

## Other Notes
- Test configs in `tmp/bw-test/` have duplicate `bandwidths` lines (commented + uncommented). Use `sed -i '' '/^#bandwidths/d' tmp/bw-test/*.toml` to fix.
- Uncommitted stash changes touch many files: `.github/workflows/*.yml`, `CMakeLists.txt` (project rename), `pyproject.toml`, `scripts/`, etc.
- The uncommitted changes to `apps/config-iio-local.toml` and `apps/config-iio-remote.toml` changed `use_aa_filter` from `true` (merge) to `false` (stash). This is correct for OS=40/20/10.
- `config-iio-remote.toml` has `rate=2500000` giving OS=40/20/10 for BWs 62.5k/125k/250k — none are powers of 2, so `use_aa_filter` must stay `false`.
- For `use_aa_filter=true` to work, need power-of-2 OS. 1 MS/s gives OS=16/8/4 for BWs 62.5k/125k/250k — all powers of 2. But 1 MS/s needs Pluto rate validation.
