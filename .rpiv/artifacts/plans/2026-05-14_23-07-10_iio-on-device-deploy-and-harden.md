---
date: 2026-05-14T23:07:10+0200
author: Tom Hensel
commit: ca811b5
branch: iio-on-device
repository: gr4-lora
topic: "On-device LoRa decode via IIO on tezuka — deploy, verify, and harden"
tags: [plan, iio, plutosdr, tezuka, deployment]
status: ready
parent: "thoughts/shared/research/2026-05-14_23-02-36_iio-on-device-deployment.md"
last_updated: 2026-05-14T23:07:10+0200
last_updated_by: Tom Hensel
---

# IIO On-Device Deploy and Harden — Implementation Plan

## Overview

The IIO integration (`build_rx_graph_iio()`, `config-tezuka.toml`, CMake libiio detection, cross-build CI) is already implemented on the `iio-on-device` branch at commit `ca811b5`. This plan covers deploying the ARM64 binary to tezuka, running hardware verification against the FISH Ball PlutoSDR, adding CI test coverage for the IIO path, scoping what's needed for TX on IIO, and addressing any remaining Pluto behavioral quirks.

Priority order (confirmed by developer):

1. Cross-build & deploy to tezuka — run `lora_trx` with `config-tezuka.toml` on actual hardware
2. Hardware verification — validate decode, overflow, and scan paths at 2.5 MS/s
3. IIO test coverage — add `qa_lora_graph_builder_iio.cpp`
4. IIO TX path exploration — document what's needed
5. Fix remaining Pluto quirks — audit and resolve edge cases

## Desired End State

- `lora_trx` runs continuously on tezuka via local IIO (`uri="local:"`), decoding LoRa frames from the PlutoSDR at 2.5 MS/s with 3 bandwidth chains (62.5k, 125k, 250k), streaming CBOR frames over UDP to subscribing clients
- Overflow counters are visible in status heartbeats and remain low under normal load
- `lora_scan` works in IIO mode for spectral scanning
- IIO graph builder path has CI test coverage (property-map verification without hardware)
- Documented understanding of what's needed for IIO TX
- Known Pluto quirks documented with workarounds verified

## What We're NOT Doing

- Not re-architecting the source-block dispatch (it works)
- Not building an IIO TX graph (analysis only in this phase)
- Not changing the UDP/CBOR protocol layer
- Not porting to other IIO devices (only Pluto/AD9361)
- Not writing hardware-in-the-loop CI tests (no Pluto in CI)

## Phase 1: Cross-Build & Deploy to tezuka

### Overview

Cross-compile an ARM64 binary of `lora_trx` (and `lora_scan`) with `GR4_LORA_HAVE_IIO` enabled, deploy to tezuka via SCP, and verify it starts and connects to the PlutoSDR over local IIO.

### Changes Required:

#### 1. Trigger Cross-Build

The cross-build CI at `.github/workflows/_cross-build.yml:165-241` already cross-compiles libiio v0.26 with `WITH_LOCAL_BACKEND=ON` into the Bootlin sysroot and generates the synthetic `iio-config.cmake`.

**Action**: Push the current `iio-on-device` branch to trigger CI, or cross-build manually using the Bootlin toolchain.

Manual cross-build workflow (for reference):

```bash
# Set up toolchain (already in .github/workflows/_cross-build.yml)
cmake -S . -B build-tezuka -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64-bootlin.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_LORA_APPS=ON \
  -DENABLE_LORA_BENCHMARKS=OFF

cmake --build build-tezuka -j$(nproc)
```

Expected artifacts:

- `build-tezuka/apps/lora_trx` — main RX decode binary with IIO support
- `build-tezuka/apps/lora_scan` — scanner binary with IIO support

#### 2. Deploy Binary to tezuka

```bash
# SCP the binary and config to tezuka
scp build-tezuka/apps/lora_trx tom@10.0.23.149:~/lora_trx_iio
# The config-tezuka.toml is already part of the source; deploy alongside binary
# Verify it's present: scp apps/config-tezuka.toml tom@10.0.23.149:~/
```

#### 3. Verify Pluto Accessibility via libiio

```bash
# On tezuka, verify the Pluto is visible via local IIO
iio_info -s
# Expected output should show local context with:
#   - ad9361-phy
#   - cf-ad9361-lpc (or similar streaming device)

# If iio_info is not installed, verify from /sys:
ls /sys/bus/iio/devices/
# Expected: iio:device0 (ad9361-phy), iio:device1 (cf-ad9361-lpc)
```

#### 4. Run lora_trx Smoke Test

```bash
# Start lora_trx with tezuka config
cd ~
./lora_trx_iio --config config-tezuka.toml --log-level DEBUG
```

Expected startup banner (relevant lines):

```
device     iio ? (clock: internal)
radio      869.618 MHz  ...
rx chains  ch0/A_BALANCED  @ 60 dB
TX disabled: RX-only mode
ready      3 decoders (SF7-12) on 1 RX chain — Ctrl+C to stop
```

If IIO source fails, the error will be one of:

- `"IIO source requested but GR4_LORA_HAVE_IIO not enabled"` — binary not compiled with IIO support
- `iio_create_context_from_uri('local:')` exception — no Pluto kernel driver
- `iio_context_find_device('ad9361-phy') returned null` — wrong firmware/device names

#### 5. Verify Pluto Device Names

If `ad9361-phy` or `cf-ad9361-lpc` are not found:

```bash
# List all IIO devices and their names on tezuka
for d in /sys/bus/iio/devices/iio:device*/name; do echo "$d: $(cat $d)"; done
# Or via iio_info -d
iio_info -d
```

If names differ, update the hardcoded device names in `apps/graph_builder.hpp:637-638` (`"cf-ad9361-lpc"` / `"ad9361-phy"`) or use the `attributes` property_map for custom devices.

### Success Criteria:

#### Automated Verification:

- [ ] `lora_trx` binary exists and is an ARM64 ELF: `file lora_trx_iio | grep -q "ELF 64-bit LSB.*aarch64"`
- [ ] `lora_trx_iio --help` prints usage with no error
- [ ] The binary links against libiio: `ldd lora_trx_iio | grep -q libiio`

#### Manual Verification:

- [ ] `iio_info -s` shows a local context with AD9361 devices on tezuka
- [ ] `lora_trx --config config-tezuka.toml --log-level INFO` starts without crashing
- [ ] Startup log shows `"TX disabled: RX-only mode"` (confirms IIO path)
- [ ] Binary responds to Ctrl+C cleanly (logs `"stopping"` then `"stopped"`)
- [ ] No segfaults or uncaught exceptions during 30s idle run

---

## Phase 2: Hardware Verification on tezuka

### Overview

Validate that the IIO path produces valid LoRa frame decodes, monitor overflow counts, and test scan mode. This phase requires actual RF stimulus or a test signal.

### Changes Required:

No code changes — this is a testing and verification phase.

#### 1. Verify Frame Decode

With a known LoRa transmitter nearby (or a test signal via a second SDR):

```bash
# Start lora_trx with DEBUG logging
./lora_trx_iio --config config-tezuka.toml --log-level DEBUG

# Subscribe via UDP (from a second terminal on tezuka or remote)
# The binary sends CBOR frames to all subscribed clients on port 5556
```

Expected output:

```
[2026-05-14T12:00:00] [info ] lora_trx: ready 3 decoders (SF7-12) on 1 RX chain
[2026-05-14T12:00:05] [info ] lora_trx: frame ch=0 label=promisc sf=8 bw=62500 snr=8.2 ...
```

If no frames are decoded after 30s with a confirmed transmitter:

- Check Pluto gain: `rx_gain = 60` in config — AD9361 has 0-71 dB range
- Verify frequency matches transmitter: `freq = 869618000` Hz
- Check for overflows in status heartbeats (`rx_overflows` in CBOR status)

#### 2. Monitor Overflow Counters

The status heartbeat (every 10s by default) includes `rx_overflows`. Monitor with a UDP listener:

```bash
# Simple hexdump of CBOR status messages to check for non-zero overflows
nc -u -l 5556 | xxd | head -100
```

During PHY-layer smoking test (ADS-B standard IIO smoke test):

```bash
# Poll lora_trx stdout for overflow log lines
./lora_trx_iio --config config-tezuka.toml 2>&1 | grep -i overflow
```

Expected: zero or very low overflow count. At 2.5 MS/s with 32768-sample buffer = 13.1ms per refill, the ARM CPU should keep up.

If overflows are high:

- Try increasing `buffer_size` in `apps/graph_builder.hpp:634` (e.g., 65536)
- Check USB bandwidth contention on tezuka
- Consider reducing sample rate (e.g., 1 MS/s) as fallback

#### 3. Validate Scan Mode

```bash
# Run lora_scan with tezuka config (streaming mode)
./lora_scan_iio --config config-tezuka.toml --log-level DEBUG
```

Expected: `lora_scan` starts, creates IIOSource → ScanController → ScanSink graph, produces L1 energy data and CAD captures. Scan uses `gain_mode="slow_attack"` (AGC) unlike decode path.

Note: If scan shows poor sensitivity, investigate the `"slow_attack"` AGC vs `"manual"` gain divergence documented in the research artifact.

#### 4. Performance Baseline

Record baseline metrics:

- CPU usage: `top -b -n 60 -p $(pgrep lora_trx)` or equivalent
- Overflow rate: per status heartbeat
- Frame decode rate: from `crc_ok` counter in status heartbeats
- Memory usage: `ps -o rss,pmem -p $(pgrep lora_trx)`

### Success Criteria:

#### Automated Verification:

- [ ] Status heartbeats are received on UDP port 5556 every ~10s
- [ ] Heartbeat CBOR contains `rx_overflows` field with expected structure

#### Manual Verification:

- [ ] At least one LoRa frame decoded (crc_ok > 0) within 60s with a transmitter
- [ ] `rx_overflows` stays at 0 or very low (< 10 per heartbeat) at 2.5 MS/s
- [ ] `lora_scan` starts and produces spectrum output in IIO mode
- [ ] Pluto DMA buffers do not overflow under continuous streaming for 5+ minutes
- [ ] CPU usage on ARM stays reasonable (no sustained 100% of a single core)

---

## Phase 3: Add IIO Test Coverage

### Overview

Write a unit test that exercises `build_rx_graph_iio()` properties (property-map key verification) without requiring actual Pluto hardware. This catches regressions in the AD9361 attribute path at CI time.

### Changes Required:

#### 1. New Test File: `test/qa_lora_graph_builder_iio.cpp`

Create a smoke test that:

- Constructs a `TrxConfig` with `uri = "local:"` (enabling IIO path)
- Calls `build_rx_graph_iio()` and verifies the property_map keys on the IIOSource block are correct
- Calls `build_scan_graph_iio()` for scan path
- Verifies `find_rx_blocks()` correctly identifies IIOSource by type name
- Verifies `read_source_overflow()` dynamic_cast works for IIOSource

Note: This test does NOT start the scheduler (which would require real libiio hardware). It verifies graph construction and property reflection only — the same pattern as `qa_IIOSource.cpp` but at the graph-builder level.

**File**: `test/qa_lora_graph_builder_iio.cpp`
**Pattern**: Follow `test/qa_lora_graph_builder.cpp:52-72` (`loadTrxForLoopback()` pattern) but set `c.uri = "local:"` and call `build_rx_graph_iio()` instead of `build_rx_graph()`.

#### 2. Update `test/CMakeLists.txt`

Add a new test target `qa_lora_graph_builder_iio`:

- Link against the same libraries as `qa_lora_graph_builder`
- Add `CONFIG_TOML_PATH = apps/config-tezuka.toml`
- Gate on `GR4_LORA_HAVE_IIO` being available (for native builds where libiio is installed)

```cmake
# In test/CMakeLists.txt:
if(TARGET gr4l::blocks_iio_headers)
    add_gr4_test(qa_lora_graph_builder_iio
        SOURCES qa_lora_graph_builder_iio.cpp
        LIBRARIES gnuradio-core-group gr-lora gr-sdr gr-testing tomlplusplus::tomlplusplus
                  gr4l::blocks_iio_headers
        DEFINES CONFIG_TOML_PATH="${PROJECT_SOURCE_DIR}/apps/config-tezuka.toml"
                GIT_REV="test"
                GR4_LORA_HAVE_IIO)
endif()
```

#### 3. Verify Test Passes Natively

```bash
# Build and run the new test (requires libiio from Homebrew on macOS)
cmake -B build -G Ninja -DENABLE_LORA_APPS=ON
cmake --build build -j$(nproc)
ctest --test-dir build -R qa_lora_graph_builder_iio -V
```

### Success Criteria:

#### Automated Verification:

- [ ] `qa_lora_graph_builder_iio.cpp` compiles without errors
- [ ] Test passes on macOS (Homebrew libiio): `ctest -R qa_lora_graph_builder_iio`
- [ ] Test is gated by `GR4_LORA_HAVE_IIO` in CMake — skipped when libiio unavailable
- [ ] Test verifies IIOSource property keys match expected values from `build_rx_graph_iio()`
- [ ] Test verifies `find_rx_blocks()` can identify IIOSource by type name
- [ ] CI passes with the new test target (cross-build only compiles, doesn't run tests)

#### Manual Verification:

- [ ] Review the test covers the key property_map entries (uri, device, phy_device, sample_rate, center_frequency, bandwidth, gain, gain_mode, rf_port)
- [ ] Verify the test catches a typo in property_map keys (e.g., change `"device"` to `"dev"` in source and confirm test fails)

---

## Phase 4: Map Out IIO TX Path

### Overview

Analysis phase: explore what would be needed to enable TX on the IIO path. Currently disabled at `apps/lora_trx.cpp:492-515` with "not yet supported". Produce an explore artifact documenting the gaps.

### Changes Required:

No source code changes — produce an explore document.

#### 1. Research IIO TX Block Status

- Read `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSink.hpp` if it exists
- Trace the TX-chain commits from precedent analysis: `85c014a` (initial IIOSink wiring), `1df4923` (IIOSink instead of IIOSinkMimo), `89ee9f7` (TX_LO powerdown), `780da70` (attributes walker + TX_LO powerdown)
- Check if an IIOSink equivalent of `build_rx_graph_iio()` exists or was attempted

#### 2. Identify Gaps

Based on the research findings, known gaps include:

- No `build_tx_graph_iio()` function (equivalent of `build_tx_graph()` at `apps/lora_trx.cpp:664-748` using SoapySink)
- No property_map for IIOSink (TX equivalents of the AD9361 attributes in IIOSource)
- TX LO powerup/powerdown sequencing (commit `89ee9f7`, `780da70`)
- DMA buffer format differences between RX and TX on Pluto local backend (commit `0fd7434`)

#### 3. Write Explore Artifact

Write to `thoughts/shared/solutions/2026-05-14_23-07-10_iio-tx-path.md`:

Options considered:

- **A**: `build_tx_graph_iio()` mirroring `build_tx_graph()` but with IIOSink instead of SoapySink
- **B**: Reuse existing SoapySink for TX while using IIOSource for RX (hybrid)
- **C**: Wait for gr4-incubator to mature its IIO TX blocks

Each with pros/cons and a recommended approach.

### Success Criteria:

#### Automated Verification:

- [ ] Explore document exists with clear options, gaps, and recommendation

#### Manual Verification:

- [ ] Document covers what's needed to remove the `!cfg.uri.empty()` TX guard
- [ ] Document identifies the IIOSink property_map requirements (frequency, gain, LO config)
- [ ] Document notes any submodule bumps needed in gr4-incubator

---

## Phase 5: Fix Remaining Pluto Quirks

### Overview

Audit the 4 known Pluto behavioral quirks discovered during the initial integration and verify they're properly handled. Address any edge cases discovered during Phase 2 hardware verification.

### Changes Required:

#### 1. Audit Known Quirks

| #   | Quirk                                         | Fix commit           | Currently in codebase?                                                                                 | Needs attention?                                                                     |
| --- | --------------------------------------------- | -------------------- | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ |
| 1   | Blocking buffer starves scheduler             | `0aa8461`            | ✅ `non_blocking=false` at `graph_builder.hpp:637` — blocking mode used intentionally for lower CPU    | Verify on tezuka — if CPU is fine, keep. If scheduler stalls, switch to `true`       |
| 2   | LO frequency write triggers calibration stall | `abb01fa`, `4d6c731` | ✅ `build_rx_graph_iio()` sets `center_frequency` and LO is skipped implicitly via `settingsChanged()` | Manual check: does setting `center_frequency` in property_map trigger a calibration? |
| 3   | MIMO→single-port revert                       | `0f82cba`, `1df4923` | ✅ `IIOSource<cf32>` single-port with 2 channels                                                       | Confirmed correct for Pluto                                                          |
| 4   | `buffer_size=0` rejected                      | `a8c2d51`            | ✅ `buffer_size=32768`                                                                                 | Needs re-verification only if error surfaces on different Pluto firmware             |

#### 2. Apply Fixes from Hardware Verification

Any issues found during Phase 2 go here. For each issue:

- Root cause (is it a new Pluto quirk, a code bug, or a config issue?)
- Proposed fix with file:line references
- Success criteria for the fix

#### 3. Document Pluto Hardware Verification Checklist

Create a checklist in the research doc or as a standalone verification document covering:

- AD9361 device enumeration via `iio_info`
- Valid sample rate divisors (40 MHz / N)
- Gain range (0-71 dB manual PGA)
- Antenna port selection (A_BALANCED, A_NEG, B_BALANCED, B_NEG, C_BALANCED, C_NEG)
- LO frequency limits (325 MHz to 3.8 GHz)
- RF bandwidth limits (200 kHz to 20 MHz)

### Success Criteria:

#### Automated Verification:

- [ ] No compile warnings related to IIO blocks after applying any fixes

#### Manual Verification:

- [ ] Known quirk #2 verified: LO calibration does not stall streaming on `uri="local:"`
- [ ] `buffer_size=32768` confirmed compatible with Pluto's local kernel buffer allocation
- [ ] Pluto hardware verification checklist completed
- [ ] Any issues from Phase 2 fixed and documented

---

## Testing Strategy

### Automated:

- **Phase 3**: `qa_lora_graph_builder_iio.cpp` — property-map key verification for IIO graph builders (runs on native builds with libiio)
- **Phase 1/2**: No automated tests (require Pluto hardware)
- **Pre-commit**: `clang-format-18 -i **/*.{hpp,cpp}` on all changed files
- **Lint**: `prek run --all-files` on Python files only (no C++ linter configured)

### Manual Testing Steps (tezuka hardware):

1. Cross-build ARM64 binary with `GR4_LORA_HAVE_IIO` (CI or manual)
2. Deploy via SCP to tezuka: `scp build/apps/lora_trx tom@10.0.23.149:~/`
3. Verify Pluto enumerated: `iio_info -s` shows local context
4. Start `lora_trx --config config-tezuka.toml --log-level DEBUG`
5. Wait 10s for first status heartbeat, check `rx_overflows`
6. Transmit LoRa frame from a second device, verify `crc_ok` increments
7. Run `lora_scan --config config-tezuka.toml` and verify spectrum output
8. Run for extended duration (30+ minutes), monitor CPU and overflows
9. Ctrl+C to stop, verify clean shutdown

## Performance Considerations

- **Sample rate 2.5 MS/s**: 32,768-sample DMA buffer = 13.1ms per refill. ARM CPU on tezuka (Allwinner H3 or similar) must drain each buffer before the next fills. At 2.5 MS/s × 4 bytes/sample = 10 MB/s USB throughput, the kernel DMA engine handles the transfer; the application processes ~76 buffers/s.
- **Triple-bandwidth decode**: 3 × MultiSfDecoder chains (62.5k, 125k, 250k) each processing all SF7-12 simultaneously. OS factors of 40, 20, 10. The `use_aa_filter = true` adds half-band FIR taps to the decimation chain for the higher OS factors.
- **Non-blocking vs blocking**: Current code uses `non_blocking=false` (blocking refill). This is optimal for CPU (scheduler thread sleeps between DMA completions) but risks scheduler stalls if DMA latency spikes. If stalling is observed, switch to `non_blocking=true` (poll mode, higher CPU).

## References

- Research: `thoughts/shared/research/2026-05-14_23-02-36_iio-on-device-deployment.md`
- Source: branch `iio-on-device`, commit `ca811b5`
- Key files:
  - `apps/graph_builder.hpp:605-701` — `build_rx_graph_iio()`
  - `apps/config-tezuka.toml` — tezuka Pluto config
  - `apps/lora_trx.cpp:531-540` — IIO dispatch
  - `apps/CMakeLists.txt:30-68` — IIO CMake detection
  - `.github/workflows/_cross-build.yml:165-241` — libiio cross-build
  - `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — IIO source block
