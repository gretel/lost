# Implementation Plan: Bridge Hardware Test

**Spec:** `docs/superpowers/specs/2026-03-24-bridge-hardware-test-design.md`
**Date:** 2026-03-24

## Phase 0: Fix lora_config + add tests

### W0: Fix cbor_to_phy_props SF mapping (apps/lora_trx.cpp)

- [ ] Change line 337: `props["sf"]` → `props["sf_min"]` + `props["sf_max"]`
  - Both set to `static_cast<uint8_t>(sf_val)`
  - This makes lora_config set single-SF decode (sf_min == sf_max)
- [ ] Verify: `settingsChanged()` in MultiSfDecoder watches for `"sf_min"` ✓
- [ ] Build: `cmake --build build --target lora_trx -- -j4`

### W1: Add C++ reconfig test (test/qa_lora_multisf.cpp)

- [ ] New test section: "MultiSfDecoder runtime reconfigure"
- [ ] Test 1: "SF change via settings triggers reconfigure"
  - Build graph: TagSource(SF8 frame) → MultiSfDecoder(sf_min=8, sf_max=8) → TagSink
  - Run → verify decode succeeds (CRC OK)
  - Apply settings: `{sf_min: 12, sf_max: 12, bandwidth: 62500}`
  - Inject SF12 frame → verify decode succeeds
- [ ] Test 2: "BW change via settings triggers reconfigure"
  - Build graph at BW125k, inject BW125k frame → OK
  - Apply settings: `{bandwidth: 62500}`
  - Inject BW62.5k frame → OK
- [ ] Build: `cmake --build build --target qa_lora_multisf -- -j4`
- [ ] Run: `ctest --test-dir build -R qa_lora_multisf --output-on-failure --timeout 60`

### W2: Add Python send_lora_config helper (scripts/lora_common.py)

- [ ] Add `send_lora_config(sock, addr, **kwargs) -> bool`
  - Builds CBOR `{"type": "lora_config", ...kwargs}`
  - Sends via UDP, waits for `lora_config_ack` with timeout (2s)
  - Returns `ack["ok"]`
- [ ] Add unit test in test_lora_test.py or test_lora_common.py

### Checkpoint: rebuild + verify all tests pass

- [ ] Rebuild lora_trx + qa_lora_multisf
- [ ] `ctest --test-dir build -R "qa_lora" --output-on-failure --timeout 60`
- [ ] Python tests: meshcore_bridge, meshcore_crypto, lora_test

## Phase 1: bridge mode in lora_test.py

### W3: Add bridge mode skeleton

- [ ] Add `"bridge"` to CLI subcommand choices
- [ ] Add `_run_bridge_experiment()` function
- [ ] Process lifecycle: start lora_trx, start meshcore_bridge, create
      MeshCoreCompanion + CompanionDriver
- [ ] Bridge startup: `_start_process("scripts/meshcore_bridge.py", ...)`
      with `_wait_tcp(7834)` readiness check
- [ ] Get SDR pubkey from CompanionDriver (parse `infos` output)

### W4: Per-point PHY reconfig

- [ ] Add `_reconfig_sdr(sock, addr, sf, bw, freq)` — sends lora_config CBOR
- [ ] Integrate with MeshCoreCompanion.set_radio() for Heltec side
- [ ] Sleep SETTLE_S after both sides configured

### W5: Implement 4 test phases

- [ ] Phase A: `_bridge_phase_advert_rx()` — Heltec advert, verify contact
- [ ] Phase B: `_bridge_phase_advert_tx()` — Bridge advert, verify Heltec RX
- [ ] Phase C: `_bridge_phase_msg_tx()` — Bridge TXT_MSG, verify Heltec RX
- [ ] Phase D: `_bridge_phase_msg_rx()` — Heltec TXT_MSG, verify Bridge RX
- [ ] Payload format: `sf{sf}bw{bw_k}f{freq_khz}_{nonce:02x}`

### W6: Result collection + output

- [ ] `_collect_bridge()` — assemble per-point results dict
- [ ] JSON output matching existing format
- [ ] Summary line: `BRIDGE N/N points: advert_rx=... msg_tx=... msg_rx=...`

### W7: Unit tests

- [ ] Add bridge mode tests to test_lora_test.py
- [ ] Test result collection, summary formatting
- [ ] Test payload generation

### Checkpoint: full verification

- [ ] All C++ tests pass
- [ ] All Python tests pass
- [ ] Manual dry-run of bridge mode with `--help`
