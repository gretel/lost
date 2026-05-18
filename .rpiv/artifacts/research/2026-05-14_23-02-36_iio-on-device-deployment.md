---
date: 2026-05-14T23:02:36+0200
author: Tom Hensel
commit: ca811b5
branch: iio-on-device
repository: gr4-lora
topic: "On-device LoRa decode via IIO on tezuka — replace SoapySDR-over-TCP with direct libiio"
tags: [research, iio, plutosdr, tezuka, deployment]
status: complete
last_updated: 2026-05-14T23:02:36+0200
last_updated_by: Tom Hensel
---

# Research: On-device LoRa decode via IIO on tezuka — replace SoapySDR-over-TCP with direct libiio

## Research Question

What is the current state of the IIO integration for on-device LoRa decode on tezuka (FISH Ball PlutoSDR), and what is needed to deploy, verify, and harden it?

## Summary

The IIO integration is already implemented on the `iio-on-device` branch (HEAD `ca811b5`). The source-block dispatch (`build_rx_graph_iio()` when `cfg.uri` non-empty), the CMake libiio detection, the cross-build CI for ARM64, and the tezuka-specific config (`config-tezuka.toml`) are all in place. The precedent commit chain shows 13+ fix commits within ~16 hours after the initial integration, concentrated on 4 Pluto behavioral quirks (non-blocking buffer, LO calibration stall, MIMO→single-port revert, CMake detection). The priority goal is: cross-build, deploy to tezuka, run hardware verification.

## Detailed Findings

### 1. IIO Source Dispatch — Already Wired

The selection between SoapySDR and IIO is a runtime decision via `cfg.uri` (`apps/config.hpp:68`):

- `uri = ""` → SoapySDR path (`build_rx_graph()` / `build_scan_graph()`)
- `uri = "local:"` → IIO path (`build_rx_graph_iio()` / `build_scan_graph_iio()`)

**Consumers of `cfg.uri`** (`apps/lora_trx.cpp:531-540`):

```cpp
if (!cfg.uri.empty()) {
    overflow_ptr = build_rx_graph_iio(rx_graph, cfg, frame_callback, ...);
} else {
    overflow_ptr = build_rx_graph(rx_graph, cfg, frame_callback, ...);
}
```

The same pattern exists in `apps/lora_scan.cpp:917-923` (streaming scan) and `lora_scan.cpp:1111-1117` (tuning scan). All dispatch on `!cfg.uri.empty()` with no URI-scheme awareness — any non-empty URI (including `"ip:10.0.23.149"`) routes to the same IIO path.

### 2. Build System & IIO Detection

**Native builds** (`apps/CMakeLists.txt:38-45`): Searches `/opt/homebrew` then `/usr` for `iio.h`. On macOS, Homebrew installs libiio under `/opt/homebrew`.

**Cross-builds for ARM64** (`apps/CMakeLists.txt:35-36`): Uses `${CMAKE_SYSROOT}/usr` as the prefix. The CI at `.github/workflows/_cross-build.yml:165-241` cross-compiles libiio v0.26 into the Bootlin toolchain sysroot with `WITH_LOCAL_BACKEND=ON`, `WITH_NETWORK_BACKEND=OFF`.

When found, an INTERFACE library `gr4l::blocks_iio_headers` is created, both `lora_trx` and `lora_scan` link it, and both receive `GR4_LORA_HAVE_IIO` as a private compile definition.

**Lesson from precedents**: CMake detection has been fragile across environments — the fix that finally worked brute-checks `EXISTS` on `iio.h`/`libiio.so` at `${CMAKE_SYSROOT}/usr/` rather than using `find_path`/`find_library` or `pkg-config`.

### 3. IIOSource Block — AD9361 Attribute Path

The `IIOSource<cf32>` block at `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` wraps libiio v0.26 for AD9361 access. Its `reinitDevice()` (`IIOSource.hpp:147-198`) performs:

1. `detail::Context(uri)` → `iio_create_context_from_uri("local:")` (creates local kernel IIO context)
2. `findDevice("ad9361-phy")` → PHY device for LO, filter, gain control
3. `findDevice("cf-ad9361-lpc")` → streaming DMA device for I/Q samples
4. Channel discovery: `voltage0` (I), `voltage1` (Q)
5. AD9361 attribute writes (only when `phy_device == "ad9361-phy"`):
   - `altvoltage0/frequency` ← `center_frequency` (LO tuning)
   - `voltage0/sampling_frequency` ← `sample_rate`
   - `voltage0/rf_bandwidth` ← `bandwidth` (analog LPF)
   - `voltage0/gain_control_mode` ← `gain_mode`
   - `voltage0/hardwaregain` ← `gain` (only when `gain_mode == "manual"`)
   - `voltage0/rf_port_select` ← `rf_port` (best-effort, non-fatal)
6. `iio_device_create_buffer(_streamDev, 32768, false)` → DMA buffer
7. `setBlockingMode(!non_blocking)` → default blocking

### 4. Known Pluto Behavioral Quirks (from precedents)

| Quirk                                         | Fix commit           | Description                                                                                                                                                           |
| --------------------------------------------- | -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Blocking buffer starves scheduler             | `0aa8461`            | Default `non_blocking=false` caused `refill()` to block for full `timeout_ms` (1s). Non-blocking poll mode required for local IIO backends.                           |
| LO frequency write triggers calibration stall | `4d6c731`, `abb01fa` | Setting `center_frequency` on Pluto local backend triggers AD9361 calibration that stalls streaming. LO writes disabled in IIOSource property map for graph builders. |
| MIMO abstraction wrong for Pluto              | `0f82cba`, `1df4923` | Pluto presents channels across separate device entries, not a MIMO block. Reverted to single-port `IIOSource<cf32>` with 2 channels.                                  |
| `iio_device_create_buffer` rejects size=0     | `a8c2d51`            | Fixed to use `buffer_size=32768` explicitly.                                                                                                                          |
| `rf_port_select` fails on some firmware       | `9997d62`, `4467b12` | Wrapped in try-catch / best-effort on tezuka_fw.                                                                                                                      |

### 5. TX Disabled on IIO Path

At `apps/lora_trx.cpp:492-515`, TX is explicitly disabled when `cfg.uri` is non-empty:

```cpp
if (!cfg.enable_tx || !cfg.uri.empty()) {
    // ... TX disabled, "not yet supported" log
}
```

When a client sends a `"lora_tx"` CBOR message, `tx_source_ptr == nullptr` triggers a `"tx_disabled"` NACK at `apps/lora_trx.cpp:710-714`. The `TxRequestQueue` worker thread is never started.

### 6. Scan Mode IIO Path

`lora_scan` has two IIO graph builders:

- `build_streaming_scan_graph_iio()` (`apps/graph_builder.hpp:764-824`) — IIOSource → ScanController → ScanSink
- `build_scan_graph_iio()` (`apps/graph_builder.hpp:702-762`) — IIOSource → Splitter → SpectrumTapBlock + CaptureSink

**Inconsistency**: Both scan paths use `gain_mode = "slow_attack"` (hardcoded AGC), overriding the manual gain from `config-tezuka.toml:12` (`rx_gain = 60`). This is intentional (scan across 3 MHz needs variable gain) but could mask sensitivity issues.

### 7. Test Coverage Gap

No test exercises `build_rx_graph_iio()`. The incubator's `qa_IIOSource.cpp` only tests property reflection and never calls `reinitDevice()` with a real libiio context. A regression in the AD9361 attribute path (e.g., wrong IIO attribute name, broken `writeAttrLL`) would only surface on actual Pluto hardware.

## Code References

- `apps/config.hpp:65` — `TrxConfig::uri` field, the dispatch discriminator
- `apps/config.hpp:68` — `TrxConfig::uri` default empty string
- `apps/config.cpp:130` — `parse_device()` reads `[device].uri` into `ParsedDevice::uri`
- `apps/config.cpp:290` — `cfg.uri = dev.uri` copy into TrxConfig
- `apps/lora_trx.cpp:531-540` — IIO vs Soapy dispatch gate
- `apps/lora_trx.cpp:492-515` — TX disabled for IIO path
- `apps/lora_trx.cpp:710-714` — `"tx_disabled"` NACK handler
- `apps/graph_builder.hpp:605-701` — `build_rx_graph_iio()` full implementation
- `apps/graph_builder.hpp:636-647` — IIOSource property map (uri, device, phy_device, etc.)
- `apps/graph_builder.hpp:702-762` — `build_scan_graph_iio()` (tuning)
- `apps/graph_builder.hpp:764-824` — `build_streaming_scan_graph_iio()` (streaming)
- `apps/graph_builder.hpp:43-73` — `add_multisf_chain()` shared by both paths
- `apps/CMakeLists.txt:30-68` — IIO detection, INTERFACE library, `GR4_LORA_HAVE_IIO`
- `.github/workflows/_cross-build.yml:165-241` — libiio v0.26 cross-build into ARM64 sysroot
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — IIOSource block definition
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIORaiiWrapper.hpp` — RAII wrappers (Context, Buffer)
- `apps/config-tezuka.toml` — Tezuka on-device Pluto config
- `test/qa_lora_graph_builder.cpp:52-72` — Soapy-only smoke test (no IIO coverage)
- `gr4-incubator/blocks/iio/test/qa_IIOSource.cpp` — property-reflection only (no hardware)

## Integration Points

### Inbound References

- `apps/lora_trx.cpp:531-540` — `build_rx_graph_iio()` called when `!cfg.uri.empty()`
- `apps/lora_trx.cpp:564` — `find_rx_blocks()` discovers IIOSource post-exchange
- `apps/lora_scan.cpp:912, 1114` — IIO scan graphs called when `!cfg.uri.empty()`
- `apps/lora_scan.cpp:933` — `find_source_block()` finds IIOSource post-exchange

### Outbound Dependencies

- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — requires libiio v0.26 (local backend)
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIORaiiWrapper.hpp` — wraps `iio.h`
- `apps/CMakeLists.txt:40-68` — depends on `iio.h` in sysroot/prefix

### Infrastructure Wiring

- `.github/workflows/_cross-build.yml:165-241` — CI cross-compiles libiio into sysroot
- `cmake/toolchain-aarch64-bootlin.cmake:16` — sets `CMAKE_SYSROOT` pointing to Bootlin sysroot

## Architecture Insights

1. **Dual-source design**: SoapySDR and IIO coexist. Selection is runtime via `cfg.uri`, not compile-time. However, `GR4_LORA_HAVE_IIO` must be defined at compile time for IIO path to exist.

2. **UDP/CBOR protocol is unchanged**: The entire frame callback chain (`FrameSink::_frame_callback` → `UdpState::broadcast`) is source-agnostic. Only the source block in the RX graph differs.

3. **IIO path is single-channel only**: The IIOSource has one output port `"out"`. Multi-channel `rx_channel = [0, 1]` would fail because the source can't provide `"out#1"`. `config-tezuka.toml` correctly sets `rx_channel = [0]`.

4. **AD9361 analog LPF = sample rate**: The `bandwidth` property in `build_rx_graph_iio()` is set to `cfg.rate` (2.5 MHz), not the LoRa channel bandwidth (62.5 kHz). This is the AD9361's analog anti-aliasing filter, not the decode bandwidth.

5. **No software DC blocking in IIO path**: The IIOSource does not have `dc_blocker_enabled` or `dc_offset_mode` properties. AD9361 hardware DC tracking is always active and not configurable via this API.

## Precedents & Lessons

10+ similar past changes analyzed (all on this branch in the last 5 days).

### Precedent: On-device decode commit (full integration)

**Commit(s)**: `4b54410` — "feat(iio): on-device decode via IIOSource (tezuka/PlutoSDR)" (2026-05-14)
**Blast radius**: 10 files across 5 layers
apps/graph_builder.hpp — 234 insertions (IIO-specific graph builders)
apps/config-tezuka.toml — 62 lines (new on-device config)
apps/CMakeLists.txt — 41 insertions (IIO header target + `GR4_LORA_HAVE_IIO`)
apps/config.cpp, config.hpp — `uri` field parsing
apps/lora_trx.cpp, lora_scan.cpp — IIO path dispatch
.github/workflows/\_cross-build.yml — cross-build libiio v0.26
gr4-incubator — submodule bump

**Follow-up fixes**:

- `37c5ed4` — config-tezuka.toml lost during submodule migration
- `ec52fa2` — uri field parsing lost during branch migration
- `ff495b4` → `0699941` → `e710c89` — 3 CMake detection iterations
- `0aa8461` — non_blocking flag added, blocking mode caused scheduler stalls
- `a8c2d51` — buffer_size=32768 for Pluto local backend
- `abb01fa` — skip AD9361 aliases (LO calibration stalls streaming)
- `4d6c731` — non-blocking buffer mode, skip LO freq write
- `ca811b5` — Homebrew prefix CMake fix (current HEAD)
- `a036b42` — gr4-incubator submodule for non_blocking support
- `2b82ad0`/`90c20ef` — CI cache invalidation
- `35433ae` — WITH\_\* flags for libiio cross-build
- `3d2d94e` — iio-config.cmake generation
- `d8dcd55` — YAML heredoc issue in CI

**Takeaway**: Highest fix density of any feature in this repo — 13 fix commits within ~16 hours. Concentrated in: build detection (4), AD9361 behavioral differences (4), submodule/CI (5), config file management (2).

### Precedent: Initial libiio direct backend wiring

**Commit(s)**: `85c014a` — "feat(apps): wire libiio direct backend into lora_trx, lora_scan, and add iio_smoke" (2026-05-09)
**Blast radius**: 3 files (graph_builder.hpp, iio_smoke.cpp, lora_trx.cpp)

**Follow-up fixes**:

- `0f82cba` — revert IIOSource to clean single-port, drop MIMO
- `1df4923` — use IIOSink instead of IIOSinkMimo for Pluto TX
- `3307f1d` — IIOSink<cf32> unified template fix

### Composite Lessons

1. **AD9361 local backend vs networked Pluto behavioral differences** are the #1 failure pattern: `sampling_frequency`/`center_frequency` triggers calibration that stalls streaming on local IIO. The fix: skip LO frequency writes entirely. Any new Pluto-local path must test with these writes disabled.

2. **CMake/libiio detection in cross-build requires brute-force**: `find_path`/`find_library`/`pkg-config` all failed in different environments. The working solution is direct `EXISTS` checks on `iio.h`/`libiio.so` at `CMAKE_SYSROOT/usr/`.

3. **Submodule management risks losing new files**: `config-tezuka.toml` was lost during submodule migration. Verify with `git diff --stat HEAD~1 -- apps/config-tezuka.toml` after any submodule bump.

4. **Blocking IIO buffer refill starves the GR4 scheduler**: Always use `non_blocking=true` with `buffer_size=32768` for local IIO backends.

5. **MIMO abstraction is wrong for Pluto**: Pluto presents channels across separate device entries. Use `IIOSource<cf32>` single-port with 2 channels.

## Historical Context (from thoughts/)

No thoughts/ documents found in `thoughts/shared/` — all subdirectories are empty.

## Developer Context

**Q (`apps/lora_trx.cpp:531`): The IIO path is already wired — `cfg.uri` non-empty selects `build_rx_graph_iio()`. What's the primary next step?**
A: Deploy & verify on tezuka (priority #1), then add IIO test coverage (#2), map out TX on IIO (#3), fix remaining Pluto quirks (#4).

## Related Research

(None yet — this is the first research artifact for IIO on-device.)

## Open Questions

- What is the current Pluto firmware on the FISH Ball (tezuka_fw version)?
- Does the Pluto present as `cf-ad9361-lpc` and `ad9361-phy` via `uri="local:"` on tezuka?
- Has `config-tezuka.toml` been validated against the actual Pluto hardware's AD9361 capabilities?
- What sample rates does the tezuka_fw AD9361 PLL support (the 40 MHz / 16 = 2.5 MS/s chain is assumed)?
