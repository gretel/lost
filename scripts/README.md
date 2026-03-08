# scripts/

Python tools for gr4-lora. All use the project venv (`.venv`).
Activate with `direnv allow`, or prefix with `.venv/bin/python3`.

---

## lora_mon.py

Connects to `lora_trx` UDP and prints decoded frames. With an identity file,
decrypts TXT_MSG, ANON_REQ, and GRP_TXT in real time. Learned public keys are
saved to `keys_dir` automatically.

```sh
lora_mon.py
lora_mon.py --connect 127.0.0.1:5556
lora_mon.py --identity ~/.config/gr4-lora/identity.bin
lora_mon.py --channel "name:BASE64SECRET"
```

---

## meshcore_bridge.py

MeshCore companion protocol bridge. Listens on TCP (default port 7834) and
translates between the companion binary protocol and `lora_trx` CBOR/UDP.
Compatible with `meshcore-cli` and the Android companion app (via port
forwarding).

```sh
meshcore_bridge.py
meshcore_bridge.py --port 5000    # some apps hardcode this
meshcore-cli -p 7834
```

Identity, contacts, channels, and public keys are persisted to disk across
restarts (see `[meshcore]` config keys). Radio parameter changes via companion
commands are in-memory only — RX retune requires restarting `lora_trx`.

### Command support

| Code | Command | Notes |
|------|---------|-------|
| 0x01 | APP_START | Returns SELF_INFO |
| 0x02 | SEND_TXT_MSG | Encrypted TXT_MSG → lora_trx |
| 0x03 | SEND_CHAN_TXT_MSG | Encrypted GRP_TXT → lora_trx |
| 0x04 | GET_CONTACTS | Streams persisted contacts |
| 0x05 | GET_DEVICE_TIME | |
| 0x06 | SET_DEVICE_TIME | In-memory |
| 0x07 | SEND_SELF_ADVERT | Signed ADVERT → lora_trx |
| 0x08 | SET_ADVERT_NAME | In-memory |
| 0x09 | ADD_UPDATE_CONTACT | Persisted |
| 0x0A | SYNC_NEXT_MESSAGE | Dequeues RX |
| 0x0B | SET_RADIO_PARAMS | In-memory; 5th byte = repeat flag |
| 0x0C | SET_RADIO_TX_POWER | In-memory |
| 0x0D | RESET_PATH | Stub (OK) |
| 0x0E | SET_ADVERT_LATLON | In-memory |
| 0x0F | REMOVE_CONTACT | Persisted |
| 0x10 | SHARE_CONTACT | Returns biz-card URI |
| 0x11 | EXPORT_CONTACT | Returns biz-card URI |
| 0x12 | IMPORT_CONTACT | Persisted |
| 0x13 | REBOOT | Stub (OK) |
| 0x14 | GET_BATT_AND_STORAGE | Fixed 100% battery |
| 0x15 | SET_TUNING_PARAMS | Stub (OK) |
| 0x16 | DEVICE_QUERY | Returns DEVICE_INFO (82 bytes) |
| 0x1F | GET_CHANNEL | From channels_dir |
| 0x20 | SET_CHANNEL | Persisted |
| 0x26 | SET_OTHER_PARAMS | Stores manual_add_contacts/telemetry_mode/adv_loc_policy/multi_acks |
| 0x28 | GET_CUSTOM_VARS | Returns empty RESP_CUSTOM_VARS (no hardware sensors on bridge) |
| 0x29 | SET_CUSTOM_VAR | Returns RESP_ERROR (no writable vars on bridge) |
| 0x36 | SET_FLOOD_SCOPE | Sets 16-byte send_scope (name or raw hex; from companion) |
| 0x38 | GET_STATS | Sub-types 0/1/2 (core/radio/packets) |
| 0x3A | SET_AUTOADD_CONFIG | |
| 0x3B | GET_AUTOADD_CONFIG | |
| 0x3C | GET_ALLOWED_REPEAT_FREQ | Returns 433/869/918 MHz pairs |
| 0x3D | SET_PATH_HASH_MODE | Mode 0–2 |

Unknown commands return OK silently.

### RX handling

| Frame | Action |
|-------|--------|
| ADVERT | Key learned; PUSH_NEW_ADVERT sent |
| TXT_MSG | ECDH trial-decrypted; RF ACK sent on success |
| ANON_REQ | Decrypted; RESP sent |
| GRP_TXT | PSK-decrypted using channels_dir |
| ACK | Matched to pending TX; PUSH_SEND_CONFIRMED if matched |
| Any (client_repeat≠0) | Raw payload re-forwarded to mesh (echo-filtered) |

### TX transport key priority

1. `send_scope` (SET_FLOOD_SCOPE or `flood_scope` in config — accepts scope name or 32-char hex)
2. `region_key` (derived from `region_scope` hashtag via `sha256("#" + name)[:16]`)
3. No transport codes

Note: `flood_scope` and `region_scope` use different hash formulas and produce different keys,
even with the same name. `flood_scope` uses `sha256(name)[:16]` (matching the companion
library); `region_scope` uses `sha256("#" + name)[:16]`.

---

## meshcore_tx.py

Builds and sends MeshCore wire packets directly to `lora_trx`.

```sh
meshcore_tx.py advert --name "MyNode"
meshcore_tx.py advert --name "MyNode" --qr
meshcore_tx.py send --dest <64hex> "Hello"
meshcore_tx.py anon-req --dest <64hex> "Hello"
meshcore_tx.py send --dest <64hex> "Hello" --dry-run
```

All subcommands: `--udp HOST:PORT`, `--identity`, `--freq`, `--sf`, `--bw`,
`--route` (flood | direct | tc-flood | tc-direct).

---

## meshcore_crypto.py

Shared crypto used by `lora_mon.py`, `meshcore_tx.py`, and
`meshcore_bridge.py`. Not a CLI tool.

Provides: Ed25519 identity management, X25519 ECDH (MeshCore-compatible —
**not** interchangeable with libsodium's `sk_to_curve25519`), AES-128-ECB
encrypt/decrypt, 2-byte HMAC-SHA256 MAC, ACK hash, key store load/save,
trial decryption for TXT_MSG / ANON_REQ / GRP_TXT.

---

## lora_mqtt.py

Publishes decoded frames to letsmesh.net MQTT (TLS, port 443). Suppresses
duplicates from dual-channel RX.

```sh
lora_mqtt.py --iata HAM
lora_mqtt.py --iata HAM --connect 127.0.0.1:5555 --mqtt-host mqtt-eu-v1.letsmesh.net
```

Topic: `meshcore/<IATA>/<pubkey_hex>/packets`

---

## Other scripts

| Script | Purpose |
|--------|---------|
| `cbor_stream.py` | CBOR Sequence (RFC 8742) reader for pipes and files |
| `lora_common.py` | Config loading, constants, format helpers — shared library |
| `lora_decode_meshcore.py` | Offline MeshCore frame decoder |
| `lora_decode_meshtastic.py` | Offline Meshtastic frame decoder |
| `lora_decode_lorawan.py` | Offline LoRaWAN MAC decoder |
| `lora_decode_raw.py` | Offline raw hex/ASCII dump of CBOR streams |
| `lora_duckdb.py` | Log decoded frames to DuckDB |
| `lora_waterfall.py` | Terminal waterfall display |
| `lora_trx.lua` | Wireshark dissector for lora_trx CBOR UDP frames |

---

## Tests

```sh
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

| Suite | Count |
|-------|------:|
| test_lora_mon | 59 |
| test_meshcore_tx | 37 |
| test_meshcore_integration | 28 |
| test_meshcore_crypto | 67 |
| test_meshcore_bridge | 159 |
| test_meshcore_bridge_integration | 12 |
| test_lora_mqtt | 11 |
| **total** | **373** |

`test_bridge_live.py` is excluded — it requires a live `lora_trx` instance.
