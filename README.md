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
                                                         │
         meshcore-cli ──TCP:7834──► meshcore_bridge.py ──┘
                        companion       ▲ CBOR/UDP
                        protocol        │
                                   config.toml
```

`lora_trx` binds UDP port 5555. Clients send any datagram to register; frames are broadcast as CBOR. TX requests use the same port.

`meshcore_bridge.py` speaks the MeshCore companion binary protocol on TCP 7834, bridging to `lora_trx` via CBOR/UDP. Compatible with `meshcore-cli`.

## Configuration

All tools read from a shared `config.toml` (search order: `./config.toml`, `./apps/config.toml`, `~/.config/gr4-lora/config.toml`). Override with `--config <path>`.

## Python scripts

All scripts live in `scripts/` and use the project venv (`.venv`, managed by `uv`).

### lora_mon.py -- streaming monitor

Connects to `lora_trx` UDP and displays decoded frames with gain advice.

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

**How key learning works:** When an ADVERT arrives, its 32-byte Ed25519 public key is saved to `~/.config/gr4-lora/keys/<hex>.key`. When a TXT_MSG arrives, all known keys are tried via ECDH + 2-byte HMAC check. The key store grows passively over time.

**Identity and keys:**
- Identity: `~/.config/gr4-lora/identity.bin` (64 bytes: seed + pubkey)
- Key store: `~/.config/gr4-lora/keys/*.key` (32 bytes each)
- Override with `--identity <path>` and `--keys-dir <path>`

### meshcore_tx.py -- MeshCore transmitter

Builds and sends MeshCore packets via `lora_trx`.

```sh
meshcore_tx.py advert --name "MyNode"                     # broadcast ADVERT
meshcore_tx.py advert --name "MyNode" --qr                # + print QR code
meshcore_tx.py send --dest <pubkey_hex> "Hello"           # encrypted TXT_MSG
meshcore_tx.py anon --dest <pubkey_hex> "Hello"           # anonymous request
meshcore_tx.py send --dest <pubkey_hex> "Hi" --dry-run    # show packet, don't send
```

All subcommands accept `--udp HOST:PORT` (default `127.0.0.1:5555`), `--identity`, `--freq`, `--sf`, `--bw`.

### meshcore_bridge.py -- MeshCore companion bridge

TCP server speaking the MeshCore companion binary protocol, bridging to `lora_trx` via CBOR/UDP. Compatible with `meshcore-cli`.

```sh
meshcore_bridge.py                                        # TCP 7834, UDP 127.0.0.1:5555
meshcore-cli -p 7834                                      # connect from meshcore-cli
```

Supports: `APP_START`, `GET_CONTACTS`, `SEND_TXT_MSG`, `IMPORT_CONTACT`, `EXPORT_CONTACT`, `GET_CHANNEL`, `GET_STATS`, and all standard companion protocol commands. RX frames from `lora_trx` are queued and pushed to the companion client.

### test_bridge_live.py -- live TX integration test

Connects to a running `meshcore_bridge.py` via TCP, imports a target contact, sends an encrypted TXT_MSG, and verifies MSG_SENT acknowledgment. This is the quickest way to test the full TX path end-to-end.

**Prerequisites:** `lora_trx` running with SDR hardware, `meshcore_bridge.py` running.

```sh
# Terminal 1: start the transceiver
lora_trx

# Terminal 2: start the bridge
meshcore_bridge.py

# Terminal 3: run the live test
test_bridge_live.py                                       # send to DO2THX (default target)
test_bridge_live.py --message "Hello from gr4-lora"       # custom message
test_bridge_live.py --bridge 192.168.1.10:7834            # remote bridge
```

The test performs these steps:
1. Connect to bridge TCP port
2. Send `APP_START`, verify `SELF_INFO` response
3. Import target contact via `IMPORT_CONTACT` (synthetic ADVERT)
4. Verify contact was added via `GET_CONTACTS`
5. Send encrypted `TXT_MSG`, verify `MSG_SENT` response

The default target is `DO2THX` (`6be972..`). To change it, edit `TARGET_PUBKEY` and `TARGET_NAME` in the script.

**Troubleshooting:**
- `ERROR code 2` = contact not found (prefix mismatch, import may have failed)
- No `MSG_SENT` = bridge couldn't build the encrypted packet (identity or crypto issue)
- Message sent but not received on phone = check radio range, identity match (see below)

### Sending messages to a companion device

For the companion device to decrypt your message, it must know your gr4-lora node's public key. The workflow:

1. **Advertise your identity** so the companion learns your pubkey:
   ```sh
   meshcore_tx.py advert --name "MyNode"         # broadcast ADVERT via lora_trx
   ```
   The companion device should show your node in its contact list after receiving the ADVERT.

2. **Import the companion's pubkey** into the bridge. Either:
   - The companion sends you an ADVERT (received by `lora_trx`, forwarded to bridge)
   - Or use `test_bridge_live.py` which imports a synthetic ADVERT for the target

3. **Send the message** via the bridge:
   ```sh
   test_bridge_live.py --message "Hello"          # uses bridge companion protocol
   ```

**Identity file:** `~/.config/gr4-lora/identity.bin`. Both `meshcore_tx.py` and `meshcore_bridge.py` use the same identity. If you regenerate it, the companion device will no longer recognize your node -- you'll need to re-advertise.

### meshcore_crypto.py -- shared crypto module

Provides ECDH key exchange (X25519), AES-128-ECB encryption, HMAC-SHA256 MAC, identity management, and a contact key store. Used by `meshcore_tx.py`, `lora_mon.py`, and `meshcore_bridge.py`.

## Tests

```sh
# C++ (7 suites)
ctest --test-dir build -R qa_lora --output-on-failure --timeout 60

# Python (271 tests)
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

## License

ISC
