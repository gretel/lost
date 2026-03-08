# scripts/

Python tools for `lora_trx`. All scripts use the project venv (`.venv`).
Activate with `direnv allow` or prefix with `.venv/bin/python3`.

## Scripts

| Script | Role |
|--------|------|
| `lora_mon.py` | Streaming monitor — connects to `lora_trx` UDP, decrypts MeshCore frames |
| `meshcore_bridge.py` | MeshCore companion protocol bridge (TCP ↔ CBOR/UDP) |
| `meshcore_tx.py` | One-shot MeshCore TX: ADVERT, TXT_MSG, ANON_REQ |
| `meshcore_crypto.py` | Shared crypto: ECDH, AES-128-ECB, identity, key store |
| `lora_mqtt.py` | LoRa-to-MQTT bridge for letsmesh.net (TLS, dedup) |
| `cbor_stream.py` | CBOR Sequence (RFC 8742) stream reader for pipes |
| `lora_common.py` | Shared constants, formatting, config loading |
| `lora_decode_meshcore.py` | Offline MeshCore protocol decoder |
| `lora_decode_meshtastic.py` | Offline Meshtastic protocol decoder |
| `lora_decode_lorawan.py` | LoRaWAN MAC header decoder |
| `lora_decode_raw.py` | Offline raw hex/ASCII dump of saved CBOR streams |
| `lora_duckdb.py` | DuckDB frame storage and query |
| `lora_waterfall.py` | Terminal waterfall display |
| `lora_trx.lua` | Wireshark dissector for lora_trx CBOR UDP frames |

---

## meshcore_bridge.py — implementation status

TCP server on port 7834 (default) implementing the MeshCore companion binary
protocol. Bridges to `lora_trx` via CBOR/UDP.

### Companion commands (app → bridge)

Legend: ✅ full · ⚠ stub (returns OK, no effect) · — not implemented

| Code | Command | Status | Notes |
|------|---------|:------:|-------|
| 0x01 | `CMD_APP_START` | ✅ | Returns `PACKET_SELF_INFO`: radio params, pubkey, node name |
| 0x02 | `CMD_SEND_TXT_MSG` | ✅ | Encrypted TXT_MSG via lora_trx; flood scope / region key priority |
| 0x03 | `CMD_SEND_CHAN_TXT_MSG` | ✅ | Encrypted GRP_TXT via lora_trx; flood scope / region key priority |
| 0x04 | `CMD_GET_CONTACTS` | ✅ | Streams 147-byte contact records from `contacts_dir` |
| 0x05 | `CMD_GET_DEVICE_TIME` | ✅ | Returns current Unix timestamp |
| 0x06 | `CMD_SET_DEVICE_TIME` | ✅ | Updates in-memory clock offset (session-only) |
| 0x07 | `CMD_SEND_SELF_ADVERT` | ✅ | Builds and transmits signed ADVERT via lora_trx |
| 0x08 | `CMD_SET_ADVERT_NAME` | ✅ | Updates in-memory node name (session-only) |
| 0x09 | `CMD_ADD_UPDATE_CONTACT` | ✅ | Parses 147-byte record; persists to `contacts_dir` |
| 0x0A | `CMD_SYNC_NEXT_MESSAGE` | ✅ | Dequeues next RX message from internal queue |
| 0x0B | `CMD_SET_RADIO_PARAMS` | ✅ | In-memory only (RX retune needs `lora_trx` restart); optional 5th byte sets `client_repeat`; freq validated against `REPEAT_FREQ_RANGES` |
| 0x0C | `CMD_SET_RADIO_TX_POWER` | ✅ | In-memory only; forwarded in next TX CBOR request |
| 0x0D | `CMD_RESET_PATH` | ⚠ | Returns OK; no path table is maintained |
| 0x0E | `CMD_SET_ADVERT_LATLON` | ✅ | Updates `lat_e6`/`lon_e6` in-memory; reflected in next ADVERT TX and `PACKET_SELF_INFO` |
| 0x0F | `CMD_REMOVE_CONTACT` | ✅ | Removes by 6-byte pubkey prefix from memory and `contacts_dir` |
| 0x10 | `CMD_SHARE_CONTACT` | ✅ | Returns `PACKET_EXPORT_CONTACT` with biz-card URI (`meshcore://<hex>`) |
| 0x11 | `CMD_EXPORT_CONTACT` | ✅ | Same as SHARE; reconstructs unsigned ADVERT for non-self contacts |
| 0x12 | `CMD_IMPORT_CONTACT` | ✅ | Decodes ADVERT wire packet; stores contact; learns pubkey |
| 0x13 | `CMD_REBOOT` | ⚠ | Returns OK; bridge process does not restart |
| 0x14 | `CMD_GET_BATT_AND_STORAGE` | ✅ | Returns `PACKET_BATTERY`: fake 100% (4200 mV); storage from `contacts_dir` |
| 0x15 | `CMD_SET_TUNING_PARAMS` | ⚠ | Returns OK; tuning params are not applied |
| 0x16 | `CMD_DEVICE_QUERY` | ✅ | Returns `PACKET_DEVICE_INFO` (82 bytes): `client_repeat` at byte 80, `path_hash_mode` at byte 81 |
| 0x1F | `CMD_GET_CHANNEL` | ✅ | Returns `PACKET_CHANNEL_INFO` from channels loaded from `channels_dir` |
| 0x20 | `CMD_SET_CHANNEL` | ✅ | Stores 32-byte channel secret; persists to `channels_dir` |
| 0x36 | `CMD_SET_FLOOD_SCOPE` | ✅ | Sets 16-byte `send_scope` key (discriminator byte validated); overrides `region_key` in all TX |
| 0x38 | `CMD_GET_STATS` | ✅ | Sub-type 0 (core uptime/errors), 1 (radio: live RSSI/SNR/noise), 2 (packet counters) |
| 0x3A | `CMD_SET_AUTOADD_CONFIG` | ✅ | Sets `autoadd_config` bitmask + optional `autoadd_max_hops` (clamped to 64) |
| 0x3B | `CMD_GET_AUTOADD_CONFIG` | ✅ | Returns `[0x19, autoadd_config, autoadd_max_hops]` (3 bytes) |
| 0x3C | `CMD_GET_ALLOWED_REPEAT_FREQ` | ✅ | Returns 25-byte response: 3 lo/hi freq pairs (433 / 869 / 918 MHz) |
| 0x3D | `CMD_SET_PATH_HASH_MODE` | ✅ | Mode 0–2; discriminator byte validated; `ERR_CODE_ILLEGAL_ARG` on bad input |

Unknown commands return `PACKET_OK` silently.

### Push / unsolicited responses (bridge → app)

| Code | Name | Status | Notes |
|------|------|:------:|-------|
| 0x82 | `PUSH_SEND_CONFIRMED` | ✅ | Emitted when an RF ACK matches a pending outbound TX tag |
| 0x83 | `PUSH_MSG_WAITING` | ✅ | Emitted when an RX frame is queued while no companion is connected |
| 0x8A | `PUSH_NEW_ADVERT` | ✅ | Emitted on ADVERT RX with the full 147-byte contact record |
| 0x80 | `PUSH_ADVERTISEMENT` | — | Legacy; bridge always uses 0x8A instead |
| 0x81 | `PUSH_PATH_UPDATE` | — | Not produced (no path table) |
| 0x88 | `PUSH_LOG_DATA` | — | Not produced |

### RX frame handling (lora_trx → bridge → app)

| Payload type | Status | Notes |
|--------------|:------:|-------|
| ADVERT (0x04) | ✅ | Pubkey learned, contact stored, `PUSH_NEW_ADVERT` sent to companion |
| TXT_MSG (0x02) | ✅ | ECDH trial-decryption against all known keys; RF ACK TX on success; `CONTACT_MSG_RECV_V3` pushed |
| ANON_REQ (0x07) | ✅ | Decrypts using embedded 32-byte sender pubkey; sends encrypted RESP; contact learned |
| GRP_TXT (0x05) | ✅ | PSK decryption using channels from `channels_dir`; `CHANNEL_MSG_RECV_V3` pushed |
| ACK (0x03) | ✅ | Matched against `pending_acks`; `PUSH_SEND_CONFIRMED` emitted on match |
| Packet forwarding | ✅ | When `client_repeat != 0`: raw RX payload re-transmitted via lora_trx (echo-filtered) |
| PATH (0x08) | — | Received but not decoded |
| TRACE (0x09) | — | Received but not decoded |
| CTRL (0x0B) | — | Received but not decoded |

### TX transport code priority

When building TX packets the bridge selects a transport key in this order:

1. **`send_scope`** — set by `CMD_SET_FLOOD_SCOPE` or `flood_scope` in config (non-zero 16-byte raw key)
2. **`region_key`** — derived from `region_scope` hashtag: `SHA-256("#" + name)[:16]`
3. **No transport codes** — plain flood / direct routing

### config.toml `[meshcore]` keys

| Key | Type | Default | Status | Notes |
|-----|------|---------|:------:|-------|
| `port` | int | 7834 | ✅ | TCP listen port; avoid 5000 on macOS (AirPlay) |
| `name` | str | `"gr4-lora"` | ✅ | Node display name shown to companion apps |
| `identity_file` | path | `~/.config/gr4-lora/identity.bin` | ✅ | Ed25519 seed + expanded key (64 bytes) |
| `keys_dir` | path | `~/.config/gr4-lora/keys/` | ✅ | Learned pubkeys (`<hex>.key`, 32 bytes each) |
| `channels_dir` | path | `~/.config/gr4-lora/channels/` | ✅ | Channel PSKs (`<name>.channel`, 16 or 32 bytes) |
| `contacts_dir` | path | `~/.config/gr4-lora/contacts/` | ✅ | Persisted contacts (`<hex>.contact`, 147 bytes each) |
| `region_scope` | str | `""` | ✅ | Regional transport-code hashtag (e.g. `"de-nord"`); empty = disabled |
| `lat` | float | 0 | ✅ | GPS latitude (decimal degrees); included in ADVERT TX when non-zero |
| `lon` | float | 0 | ✅ | GPS longitude (decimal degrees); included in ADVERT TX when non-zero |
| `client_repeat` | int | 0 | ✅ | 0 = off; non-zero = forward all RX frames back to the mesh |
| `path_hash_mode` | int | 0 | ✅ | 0–2; reflected in `PACKET_DEVICE_INFO` byte 81 |
| `autoadd_config` | int | 0 | ✅ | Bitmask for auto-add behaviour |
| `autoadd_max_hops` | int | 0 | ✅ | Max hops for auto-add (clamped to 64 at startup) |
| `flood_scope` | str | `""` | ✅ | 32 hex chars (16 bytes) for `send_scope`; overrides `region_scope` in TX |

### CLI flags

| Flag | Maps to | Notes |
|------|---------|-------|
| `--port N` | `[meshcore] port` | Overrides config |
| `--name STR` | `[meshcore] name` | Overrides config |
| `--identity PATH` | `[meshcore] identity_file` | Overrides config |
| `--keys-dir PATH` | `[meshcore] keys_dir` | Overrides config |
| `--channels-dir PATH` | `[meshcore] channels_dir` | Overrides config |
| `--contacts-dir PATH` | `[meshcore] contacts_dir` | Overrides config |
| `--udp HOST:PORT` | UDP connection to lora_trx | Default `127.0.0.1:5555`; CLI-only |
| `--config PATH` | Config file path | CLI-only |
| `--debug` | Log level DEBUG | CLI-only |

### State persistence

In-memory only — lost on bridge restart (startup values from `config.toml`):

- `client_repeat`, `path_hash_mode`, `send_scope`, `autoadd_config`, `autoadd_max_hops`
- Radio params (`freq_mhz`, `bw_khz`, `sf`, `cr`, `tx_power`)
- Node name, GPS location, clock offset
- Message queue, TX echo filter, RX dedup table, pending ACK table

Persisted to disk:

- Identity (`identity_file`)
- Learned pubkeys (`keys_dir/*.key`)
- Channel PSKs (`channels_dir/*.channel`)
- Contacts (`contacts_dir/*.contact`)

---

## Tests

Run the full suite from the repo root:

```sh
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

| Suite | Tests | Covers |
|-------|------:|--------|
| `test_lora_mon.py` | 59 | Monitor frame parsing, decryption, key learning |
| `test_meshcore_tx.py` | 37 | ADVERT / TXT_MSG / ANON_REQ packet construction |
| `test_meshcore_integration.py` | 28 | MeshCore encode-decode round-trips |
| `test_meshcore_crypto.py` | 67 | ECDH, AES-128-ECB, identity management, key store |
| `test_meshcore_bridge.py` | 159 | Companion protocol commands, RX conversion, repeat mode, flood scope, autoadd, path hash mode |
| `test_meshcore_bridge_integration.py` | 12 | Bridge integration: APP_START → TX round-trip |
| `test_lora_mqtt.py` | 11 | MQTT bridge frame conversion and dedup |
| **Total** | **373** | |

`test_bridge_live.py` requires a running `lora_trx` instance and is excluded
from the automated suite.
