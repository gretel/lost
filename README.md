# gr4-lora

LoRa PHY for [GNU Radio 4](https://github.com/fair-acc/gnuradio4). Receives and transmits MeshCore, Meshtastic, LoRaWAN, and other LoRa-based protocols via USRP B210/B220 or any SoapySDR-supported SDR.

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
                  └─────────────────────────────┘       │
                        ▲                               ▼ CBOR
                        │                        ┌──────────────┐
        meshcore_tx.py ─┘ TX request (CBOR UDP)  │  lora_mon.py │
                                                 │  lora_mqtt.py│
                                                 └──────────────┘
```

`lora_trx` binds UDP port 5555. Clients send any datagram to register; frames are broadcast as CBOR. TX requests use the same port.

## Python scripts

All scripts live in `scripts/` and use the project venv (`.venv`, managed by `uv`).

### lora_mon.py -- streaming monitor

Connects to `lora_trx` UDP and displays decoded frames.

```sh
lora_mon.py                                    # default: 127.0.0.1:5555
lora_mon.py --connect 127.0.0.1:5556           # custom port
```

**Decryption** is optional. Without an identity file, it works as a passive monitor showing hex dumps, ASCII, MeshCore header summaries, and SNR.

With an identity (created by `meshcore_tx.py`), it can decrypt:

| Message type | Decryption | Notes |
|---|---|---|
| ADVERT | n/a (unencrypted) | Pubkey auto-saved to key store |
| TXT_MSG | Yes | Tries ECDH shared secret with each known key |
| ANON_REQ | Yes | Only if addressed to us (dest_hash match) |
| GRP_TXT | No | Uses pre-shared channel key -- not yet implemented |
| GRP_DATA | No | Same as GRP_TXT |

**How key learning works:** When an ADVERT arrives, its 32-byte Ed25519 public key is saved to `~/.config/gr4-lora/keys/<hex>.key`. When a TXT_MSG arrives, all known keys are tried via ECDH + 2-byte HMAC check. The key store grows passively over time.

**Identity and keys:**
- Identity: `~/.config/gr4-lora/identity.bin` (64 bytes: seed + pubkey)
- Key store: `~/.config/gr4-lora/keys/*.key` (32 bytes each)
- Override with `--identity <path>` and `--keys-dir <path>`

### meshcore_tx.py -- MeshCore transmitter

Builds and sends MeshCore packets via `lora_trx`.

```sh
meshcore_tx.py advert --name "MyNode"                    # broadcast ADVERT
meshcore_tx.py advert --name "MyNode" --qr               # + print QR code
meshcore_tx.py send --dest <pubkey_hex> "Hello"           # encrypted TXT_MSG
meshcore_tx.py anon --dest <pubkey_hex> "Hello"           # anonymous request
meshcore_tx.py send --dest <pubkey_hex> "Hi" --dry-run    # show packet, don't send
```

All subcommands accept `--udp HOST:PORT` (default `127.0.0.1:5555`), `--identity`, `--freq`, `--sf`, `--bw`.

### lora_mqtt.py -- MQTT bridge for letsmesh.net

Forwards frames from `lora_trx` to an MQTT broker in [meshcoretomqtt](https://letsmesh.net) JSON format. TLS always enabled. Duplicate frames from dual-channel RX are suppressed.

```sh
lora_mqtt.py --iata BER                                            # default broker
lora_mqtt.py --iata BER --mqtt-host mqtt-eu-v1.letsmesh.net        # explicit host
```

**Testing locally with mosquitto:**

```sh
# Terminal 1: local TLS broker (self-signed cert)
mosquitto -c /opt/homebrew/etc/mosquitto/mosquitto.conf

# Terminal 2: subscribe
mosquitto_sub -t 'meshcore/#' -v --cafile ...

# Terminal 3: bridge against local broker
lora_mqtt.py --iata BER --mqtt-host localhost

# Terminal 4: lora_trx (or it's already running)
lora_trx
```

Topic format: `meshcore/<IATA>/<pubkey>/packets`

**Limitations:** No MQTT username/password auth. RSSI is approximated from SNR (SDR doesn't provide true RSSI). No MQTT reconnect on broker disconnect.

### meshcore_crypto.py -- shared crypto module

Provides ECDH key exchange (X25519), AES-128-ECB encryption, HMAC-SHA256 MAC, identity management, and a contact key store. Used by `meshcore_tx.py` and `lora_mon.py`.

## Tests

```sh
# C++ (7 suites)
ctest --test-dir build -R qa_lora --output-on-failure --timeout 60

# Python (158 tests)
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

## LoRa PHY parameters (MeshCore EU/Narrow)

| Parameter | Value |
|---|---|
| Frequency | 869.618 MHz |
| Spreading factor | 8 |
| Bandwidth | 62,500 Hz |
| Coding rate | 4/8 |
| Sync word | 0x12 |
| Preamble | 8 symbols |
| Sample rate | 250,000 S/s |

## License

ISC
