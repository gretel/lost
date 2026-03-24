# Bridge Hardware Test Design

**Date:** 2026-03-24
**Status:** Approved
**Branch:** feature/algorithm-improvements

## Problem

The full MeshCore companion protocol path has never been tested end-to-end
on hardware:

```
meshcore-cli ──TCP──► meshcore_bridge ──UDP/CBOR──► lora_trx ──RF──► Heltec V3
                                                                        │
meshcore-cli ◄──TCP── meshcore_bridge ◄──UDP/CBOR── lora_trx ◄──RF──── ┘
```

Existing test modes cover partial paths:

| Mode | Path | Gap |
|------|------|-----|
| `decode` | Heltec TX → lora_trx RX | No bridge involvement |
| `scan` | Heltec TX → lora_scan | No bridge involvement |
| `tx` | meshcore_tx → lora_trx TX → Heltec RX | Bypasses bridge (raw CBOR to lora_trx) |

The bridge builds all wire packets (ADVERT, TXT_MSG, GRP_TXT, ACK), handles
all encryption (Ed25519 signing, X25519 ECDH, AES-128-ECB), and manages the
companion protocol state machine. None of this is exercised on RF.

## Prerequisite: lora_config SF mapping bug

`lora_config` CBOR runtime reconfig (documented in `docs/cbor-schemas.md`)
has never been tested on hardware. Investigation revealed a bug:

**`cbor_to_phy_props()`** in `lora_trx.cpp:337` maps the CBOR `"sf"` key to
`props["sf"]`, but `MultiSfDecoder` has reflected fields `sf_min` and `sf_max`
(not `sf`). The `settings().set({"sf": 8})` call is silently rejected because
`"sf"` is not a reflected field name. The `TrxConfig` snapshot updates (so TX
uses the new SF), but the decode blocks don't reconfigure (RX stays at old SF).

**Fix:** Map CBOR `"sf"` to both `props["sf_min"]` and `props["sf_max"]`
(single-SF decode). This also requires `settingsChanged()` to trigger
`reconfigure()` when either key changes (already the case — it watches for
`"sf_min"` and `"sf_max"`).

## Design

### New `bridge` mode in lora_test.py

A fourth mode alongside `decode`, `scan`, `tx`. Tests bidirectional message
exchange through the full bridge stack.

### Process stack

| Process | Port | Started by | Lifetime |
|---------|------|-----------|----------|
| `lora_trx` | UDP 5556 | harness | Entire experiment |
| `meshcore_bridge.py` | TCP 7834, UDP→5556 | harness | Entire experiment |
| meshcore_py | Serial | harness (in-process) | Entire experiment |

The harness drives the bridge side via `CompanionDriver` (meshcore-cli
subprocess calls to TCP 7834). The Heltec side is driven via `MeshCoreCompanion`
(meshcore_py async, direct serial — same as `tx` mode).

### Per-point PHY reconfiguration

Instead of restarting lora_trx per config point, the harness sends a
`lora_config` CBOR message to lora_trx via UDP:

```python
cbor2.dumps({"type": "lora_config", "sf": 12, "bw": 125000, "freq": 868100000.0})
```

This triggers:
1. `MultiSfDecoder.settingsChanged()` → `reconfigure()` (rebuilds chirp tables)
2. `SoapySource` retune (if freq changed)
3. `TrxConfig` snapshot update (affects future TX)
4. `config` broadcast to all clients (bridge auto-updates `state.sf/bw/freq`)

The Heltec is reconfigured via `meshcore_py.set_radio()` per point (triggers
ESP32 reboot, auto-reconnect ~3s).

### Per-point test sequence (4 phases)

```
For each ConfigPoint(sf, bw, freq, tx_power):

  Step 0: Reconfigure PHY
    - Send lora_config{sf, bw, freq} → lora_trx UDP, wait for ack
    - meshcore_py.set_radio(freq, bw, sf) → Heltec (reboot + reconnect)
    - Sleep SETTLE_S (2s) for PLL lock

  Phase A: Heltec ADVERT → Bridge RX (contact exchange)
    - meshcore_py sends advert from Heltec
    - Wait FLUSH_BRIDGE_S (5s)
    - CompanionDriver.get_contacts() → verify Heltec pubkey present
    - Result: advert_rx: pass/fail
    - NOTE: only needed on first point (contact persists)

  Phase B: Bridge ADVERT → Heltec RX (identity broadcast)
    - CompanionDriver.send_advert() via bridge
    - Wait FLUSH_TX_S (3s)
    - companion.drain_adverts() → check SDR pubkey match
    - Result: advert_tx: pass/fail, SNR

  Phase C: Bridge TXT_MSG → Heltec RX (encrypted TX)
    - CompanionDriver.send_msg(heltec_name, payload)
    - Wait FLUSH_TX_S (3s)
    - companion.drain_messages() → check text content match
    - Result: msg_tx: pass/fail, payload match

  Phase D: Heltec TXT_MSG → Bridge RX (encrypted RX)
    - meshcore_py sends TXT_MSG to bridge's pubkey
    - Wait FLUSH_BRIDGE_S (5s)
    - CompanionDriver.recv_message() → check text content match
    - Result: msg_rx: pass/fail, payload match
```

### Payload format

```
sf8bw62f869618_a7
```

Components: `sf{sf}bw{bw_khz_int}f{freq_khz_int}_{random_hex:02x}`

Human-readable, unique per point, compact (saves airtime), 2-digit hex nonce
prevents dedup collisions across runs.

### Contact exchange strategy

Phase A (Heltec ADVERT → Bridge) serves as the contact exchange step. The
bridge auto-learns the Heltec's pubkey from the received ADVERT. This is the
natural MeshCore workflow — no pre-import needed.

Phase A only runs on the first config point. Subsequent points skip it if the
contact is already known (verified via GET_CONTACTS).

### ACK verification

Deferred to v2. v1 verifies message delivery only (Heltec receives the
decrypted text). The ACK chain (PATH+ACK → PUSH_SEND_CONFIRMED) is validated
by existing unit tests (TestPathAckRx).

### Output JSON (per point)

```json
{
  "config": {"sf": 8, "bw": 62500, "freq_mhz": 869.618, "tx_power": 2},
  "advert_rx": true,
  "advert_tx": true,
  "advert_tx_snr": 12.3,
  "msg_tx": true,
  "msg_tx_payload": "sf8bw62f869618_a7",
  "msg_rx": true,
  "msg_rx_payload": "sf8bw62f869618_a7",
  "phases_passed": 4,
  "phases_total": 4
}
```

### Summary output

```
BRIDGE 3/3 points: advert_rx=1/1 advert_tx=3/3 msg_tx=2/3 msg_rx=2/3
```

### Matrices

All existing matrices (`basic`, `full`, `sf_sweep`, etc.) work with bridge
mode. The `basic` matrix (3 points) is the recommended starting point.

## Implementation phases

### Phase 0: Fix lora_config + add tests

1. Fix `cbor_to_phy_props()` SF key mapping (`"sf"` → `"sf_min"` + `"sf_max"`)
2. Add C++ loopback test: `MultiSfDecoder` reconfigure via `settings().set()`
   - Build graph at SF8, inject SF12 frame → 0 output
   - Apply `{sf_min: 12, sf_max: 12}` settings change
   - Inject SF12 frame → decode succeeds
3. Add Python `send_lora_config()` helper in `lora_common.py`
4. Rebuild lora_trx binary

### Phase 1: bridge mode in lora_test.py

1. Add `_run_bridge_experiment()` function (parallel to `_run_tx_experiment()`)
2. Add `_send_lora_config()` helper for per-point PHY reconfig
3. Add `_collect_bridge()` result collector
4. Add `run_bridge_point()` with phases A-D
5. Wire into CLI argument parser
6. Add unit tests in `test_lora_test.py`

### Phase 2 (future)

- Full ACK chain verification via meshcore-cli `wait_ack`
- Payload length variation (10, 50, 133 bytes)
- GRP_TXT channel message round-trip
- ANON_REQ round-trip
