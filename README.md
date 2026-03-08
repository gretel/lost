# gr4-lora

LoRa PHY for [GNU Radio 4](https://github.com/fair-acc/gnuradio4).
Full-duplex transceiver targeting USRP B210/B220 (UHD) or any SoapySDR device.
Receives and transmits MeshCore protocol; includes a full companion protocol bridge.

## Build

```sh
brew bundle
direnv allow
cmake --preset default
cmake --build build -- -j4
```

Requires LLVM 21 (see `toolchain.cmake`). Use `-j4` max — template-heavy headers
use ~4 GB RAM per compilation unit.

## Run

```sh
lora_trx                        # RX on 869.618 MHz, CBOR frames → UDP :5555
lora_mon.py                     # display frames from lora_trx
meshcore_bridge.py              # companion protocol bridge on TCP :7834
```

## Architecture

```
RF ──► SoapySDR ──► FrameSync ──► DemodDecoder ──► FrameSink ──► UDP :5555
                         ▲                                              │
            TX (CBOR/UDP)│                                         CBOR │
                         │                                              ▼
                  meshcore_tx.py                                  lora_mon.py
                                                                        │
                  meshcore-cli ──TCP:7834──► meshcore_bridge.py ────────┘
```

`lora_trx` owns the UDP port (default 5555). Clients register by sending any
datagram; frames are broadcast as CBOR to all registered clients. TX requests
use the same port.

## Configuration

All tools read `config.toml` (search order: `./config.toml`,
`./apps/config.toml`, `~/.config/gr4-lora/config.toml`).

```toml
# SDR device
device = "uhd"
param  = "type=b200"
rate   = 250000               # Hz; must be ≥ 4 × BW
clock  = "internal"           # "internal" | "external" | "gpsdo"

# UDP (lora_trx ↔ scripts)
udp_listen      = "127.0.0.1"
udp_port        = 5555
status_interval = 10          # seconds

[logging]
level = "INFO"                # DEBUG | INFO | WARNING | ERROR

# LoRa codec (MeshCore EU narrow)
[codec_meshcore_868]
sf           = 8
bw           = 62500          # Hz
cr           = 8              # coding rate denominator (4/N, N=5–8)
sync_word    = 0x12           # 0x12 = MeshCore/Reticulum, 0x2B = Meshtastic
preamble_len = 8

# Radio
[radio_868]
freq       = 869618000        # Hz
rx_channel = [1, 2]           # B210: 0=TRX_A 1=RX_A 2=RX_B 3=TRX_B
tx_channel = 0
rx_gain    = 43               # dB
tx_gain    = 50               # dB (B220 minimum usable: 70 dB)

# Active set — binds codec + radio
[set_meshcore_868]
name  = "MeshCore 868 MHz"
codec = "codec_meshcore_868"
radio = "radio_868"

# Companion bridge
[meshcore]
name         = "MyNode"
region_scope = "de-nord"      # transport-code hashtag, or "" to disable
# port         = 7834         # avoid 5000 on macOS (AirPlay)
# identity_file = "~/.config/gr4-lora/identity.bin"
# keys_dir      = "~/.config/gr4-lora/keys"
# channels_dir  = "~/.config/gr4-lora/channels"
# contacts_dir  = "~/.config/gr4-lora/contacts"
# lat = 53.55
# lon = 9.99
# client_repeat   = 0         # non-zero = forward RX frames back to mesh
# path_hash_mode  = 0         # 0–2
# autoadd_config  = 0
# autoadd_max_hops = 0
# flood_scope = ""            # 32 hex chars (16 bytes); overrides region_scope
```

CLI flags (`--port`, `--name`, `--identity`, `--keys-dir`, `--channels-dir`,
`--contacts-dir`) override the corresponding `[meshcore]` key.

## Tests

```sh
# C++ (8 suites)
ctest --test-dir build -R qa_lora --output-on-failure --timeout 60

# Python (373 tests)
.venv/bin/python3 -m unittest discover -s scripts -p 'test_*.py' -v
```

See `scripts/README.md` for the per-suite breakdown.

## License

ISC
