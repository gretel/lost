# gr4-lora

LoRa PHY for [GNU Radio 4](https://github.com/fair-acc/gnuradio4).
Receives and transmits MeshCore protocol via USRP B210/B220 or any SoapySDR-supported SDR.

## Quick start

```sh
brew bundle                     # install dependencies (see Brewfile)
direnv allow                    # activate environment (.envrc)
cmake --preset default          # configure
cmake --build build -- -j4      # build
lora_trx                        # run (decodes ambient frames on 869.618 MHz)
```

In a second terminal:

```sh
lora_mon.py                     # stream decoded frames from lora_trx
```

## Architecture

```
                  lora_trx (C++)
                  ┌─────────────────────────────┐
RF ──► SoapySDR ──► FrameSync ──► DemodDecoder ──► FrameSink ──► UDP :5555
                  └─────────────────────────────┘        │
                         ▲                               ▼ CBOR
                         │                        ┌──────────────┐
         meshcore_tx.py ─┘ TX request (CBOR UDP)  │  lora_mon.py │
                                                  └──────────────┘
                                                         │
         meshcore-cli ──TCP:7834──► meshcore_bridge.py ──┘
                        companion       ▲ CBOR/UDP
                        protocol        │
                                   config.toml
```

`lora_trx` binds UDP port 5555. Clients send any datagram to register; frames are
broadcast as CBOR. TX requests use the same port.

`meshcore_bridge.py` speaks the MeshCore companion binary protocol on TCP 7834,
bridging to `lora_trx` via CBOR/UDP. Compatible with `meshcore-cli` and other
companion apps (e.g. the Android app via port forwarding).

## Configuration

All tools read from a shared `config.toml` (search order: `./config.toml`,
`./apps/config.toml`, `~/.config/gr4-lora/config.toml`). Override with
`--config <path>`.

### config.toml reference

```toml
# ── SDR device ───────────────────────────────────────────────────────────────
device = "uhd"                # "uhd" or "soapy" driver name
param  = "type=b200"          # device selector (UHD device args)
rate   = 250000               # sample rate (Hz); must be 4 × BW or higher
clock  = "external"           # clock source: "internal", "external", "gpsdo"

# ── UDP transport (lora_trx ↔ Python scripts) ────────────────────────────────
udp_listen       = "127.0.0.1"
udp_port         = 5555
status_interval  = 10         # seconds between status log lines

# ── MeshCore companion bridge ─────────────────────────────────────────────────
[meshcore]
region_scope  = "de-nord"     # regional transport-code hashtag (or "" to disable)
# port         = 7834         # TCP listen port  (default 7834; avoid 5000 on macOS)
# name         = "gr4-lora"   # node display name shown to companion apps
# identity_file = "~/.config/gr4-lora/identity.bin"
# keys_dir     = "~/.config/gr4-lora/keys"
# channels_dir = "~/.config/gr4-lora/channels"
# contacts_dir = "~/.config/gr4-lora/contacts"
# lat          = 53.55        # GPS latitude  (decimal degrees, optional)
# lon          = 9.99         # GPS longitude (decimal degrees, optional)

# ── Logging ──────────────────────────────────────────────────────────────────
[logging]
level = "INFO"                # DEBUG | INFO | WARNING | ERROR

# ── LoRa codec ───────────────────────────────────────────────────────────────
[codec_meshcore_868]
sf           = 8              # spreading factor 7–12
bw           = 62500          # bandwidth (Hz)
cr           = 8              # coding rate denominator 4/N  (5–8)
sync_word    = 0x12           # 0x12 = MeshCore/Reticulum, 0x2B = Meshtastic
preamble_len = 8
# min_snr_db     = -12.0      # minimum SNR threshold for frame sync
# energy_thresh  = 1e-6       # preamble energy threshold
# timeout_symbols = 30        # inter-chain combining timeout

# ── RF radio ─────────────────────────────────────────────────────────────────
[radio_868]
freq       = 869618000        # centre frequency (Hz)
rx_channel = [1, 2]           # receive channels (B210: 0=TRX_A, 1=RX_A, 2=RX_B, 3=TRX_B)
tx_channel = 0
rx_gain    = 43               # dB
tx_gain    = 50               # dB  (B220 minimum usable: 70 dB)

# ── Active set (binds codec + radio) ─────────────────────────────────────────
[set_meshcore_868]
name  = "MeshCore 868 MHz"
codec = "codec_meshcore_868"
radio = "radio_868"
# Optional: multiple decode chains (one FrameSync+DemodDecoder pair each).
# If omitted, one chain is created from the codec's sf/sync_word.
# [[set_meshcore_868.decode]]
# sf        = 8
# sync_word = 0x12
# label     = "meshcore"
```

All `--port`, `--name`, `--identity`, `--keys-dir`, `--channels-dir`,
`--contacts-dir` CLI flags override the corresponding `[meshcore]` key.
CLI flags that have no config equivalent (`--udp`, `--config`, `--debug`)
are CLI-only.

**macOS note:** port 5000 is occupied by AirPlay Receiver / Control Center.
Use the default 7834 or disable AirPlay in System Settings → General →
AirDrop & Handoff.

## Python scripts

All scripts live in `scripts/` and use the project venv (`.venv`, managed by `uv`).
Activate with `direnv allow` or prefix commands with `.venv/bin/python3`.

### lora_mon.py — streaming monitor

Connects to `lora_trx` UDP and displays decoded frames with gain advice.

```sh
lora_mon.py                             # default: 127.0.0.1:5555
lora_mon.py --connect 127.0.0.1:5556   # custom port
```

With an identity, decrypts incoming messages automatically:

| Message type | Decryption | Notes |
|---|---|---|
| ADVERT | n/a | Pubkey auto-saved to key store |
| TXT_MSG | Yes | ECDH shared secret tried for each known key |
| ANON_REQ | Yes | Only if addressed to us (dest_hash match) |
| GRP_TXT | Yes | Uses pre-shared channel key from channels_dir |

Key learning: when an ADVERT arrives, its 32-byte Ed25519 public key is saved to
`~/.config/gr4-lora/keys/<hex>.key`. The key store grows passively over time.

### meshcore_tx.py — MeshCore transmitter

Builds and sends MeshCore packets via `lora_trx`.

```sh
meshcore_tx.py advert --name "MyNode"                  # broadcast ADVERT
meshcore_tx.py advert --name "MyNode" --qr             # + print QR code
meshcore_tx.py send --dest <pubkey_hex> "Hello"        # encrypted TXT_MSG
meshcore_tx.py anon  --dest <pubkey_hex> "Hello"       # anonymous request
meshcore_tx.py send --dest <pubkey_hex> "Hi" --dry-run # show packet, don't send
```

All subcommands accept `--udp HOST:PORT` (default `127.0.0.1:5555`),
`--identity`, `--freq`, `--sf`, `--bw`.

### meshcore_bridge.py — MeshCore companion bridge

TCP server speaking the MeshCore companion binary protocol, bridging to
`lora_trx` via CBOR/UDP.

```sh
meshcore_bridge.py                   # TCP 7834, UDP 127.0.0.1:5555
meshcore_bridge.py --port 5000       # for apps that require port 5000
meshcore-cli -p 7834                 # connect with meshcore-cli
```

Supported companion commands: `APP_START`, `DEVICE_QUERY`, `GET_CONTACTS`,
`ADD_UPDATE_CONTACT`, `REMOVE_CONTACT`, `IMPORT_CONTACT`, `EXPORT_CONTACT`,
`SHARE_CONTACT`, `SEND_TXT_MSG`, `SEND_CHAN_TXT_MSG`, `GET_CHANNEL`,
`SET_CHANNEL`, `GET_MESSAGE`, `GET_BATTERY`, `GET_DEVICE_TIME`,
`SET_DEVICE_TIME`, `SET_ADVERT_NAME`, `SEND_ADVERT`, `GET_STATS`,
`SET_RADIO_PARAMS`, `SET_RADIO_TX_POWER`, `SET_ADVERT_LATLON`.

RX frames from `lora_trx` are decrypted (TXT_MSG / ANON_REQ / GRP_TXT) and
pushed to the connected companion as `CONTACT_MSG_RECV_V3` / `CHANNEL_MSG_RECV_V3`.
Contacts and channels are persisted across restarts.

**Identity and data directories** (all configurable in `[meshcore]` or via CLI):

| Setting | Default |
|---|---|
| `identity_file` | `~/.config/gr4-lora/identity.bin` |
| `keys_dir` | `~/.config/gr4-lora/keys/` |
| `channels_dir` | `~/.config/gr4-lora/channels/` |
| `contacts_dir` | `~/.config/gr4-lora/contacts/` |

**Startup robustness:** corrupt or wrong-size identity files are detected and
regenerated with a warning. Corrupt `.key` / `.channel` / `.contact` files are
skipped with a log warning rather than silently accepted or crashing. Permission
errors on data directories are logged and do not crash the bridge.

### meshcore_crypto.py — shared crypto module

ECDH key exchange (X25519 via Ed25519 scalar), AES-128-ECB encryption,
HMAC-SHA256 MAC, Ed25519 identity management, contact key store, and ACK hash
computation. Used by `meshcore_tx.py`, `lora_mon.py`, and `meshcore_bridge.py`.

### lora_mqtt.py — LoRa-to-MQTT bridge

Publishes decoded LoRa frames to the letsmesh.net MQTT packet analyzer
(`meshcore/<IATA>/<pubkey>/packets`, TLS port 443, payload-hex dedup).

## Tests

```sh
# C++ (7 suites, ~17s)
ctest --test-dir build -R qa_lora --output-on-failure --timeout 60

# Python (310 tests)
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

| Suite | Count | What it covers |
|---|---|---|
| `test_lora_mon.py` | 41 | Monitor frame parsing and decryption |
| `test_meshcore_tx.py` | 37 | MeshCore TX packet construction |
| `test_meshcore_integration.py` | 28 | MeshCore encode-decode round-trips |
| `test_meshcore_crypto.py` | 41 | ECDH, AES, identity, key store |
| `test_meshcore_bridge.py` | 124 | Companion protocol, RX conversion, edge cases |
| `test_meshcore_bridge_integration.py` | 12 | Bridge integration (APP_START→TX round-trip) |
| `test_lora_mqtt.py` | 27 | MQTT bridge frame conversion |

## License

ISC
