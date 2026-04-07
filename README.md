# gr4-lora

LoRa PHY for [GNU Radio 4](https://github.com/fair-acc/gnuradio4).
Full-duplex transceiver targeting USRP B210/B220 (UHD) or any SoapySDR device.
Decodes all LoRa spreading factors (SF7-12), coding rates (CR 4/5-4/8), and
bandwidths simultaneously. Includes MeshCore companion protocol bridge.

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
./build/apps/lora_trx --config apps/config.toml    # RX + TX transceiver
./build/apps/lora_scan --config apps/config.toml    # wideband scanner

scripts/apps/lora_mon.py                             # display decoded frames
scripts/apps/meshcore_bridge.py                      # companion protocol bridge (TCP :7834)
```

## Architecture

See [DIAGRAMS.md](DIAGRAMS.md) for detailed Mermaid flow graphs of all GR4 block
topologies (RX single/multi-BW, TX, lora_scan, MultiSfDecoder internals).

**lora_trx** runs two GR4 graphs: an RX graph with one `MultiSfDecoder` per BW
per radio channel, and a TX graph with `TxQueueSource` feeding `SoapySinkBlock`.
Multi-BW decode uses a `Splitter` to fan out the IQ stream. With 2 radios and
3 BWs, there are 6 decoders with 36 simultaneous SF lanes.

**lora_scan** uses `SpectrumTapBlock` and `CaptureSink` for L1 energy detection
and L2 CAD-based SF identification across the ISM band.

**MultiSfDecoder** is a single GR4 block that simultaneously detects and decodes
SF7-12 on one BW. CR is auto-detected from the explicit header. LDRO is
auto-applied from symbol duration. LBT channel-busy and spectrum waterfall are
built in.

## Configuration

All tools read `apps/config.toml` (or `~/.config/gr4-lora/config.toml`).

```toml
[device]
rate = 250000                    # sample rate (Hz)

[codec_meshcore_868]
sf   = 8                         # TX spreading factor (RX decodes SF7-12 auto)
bw   = 62500                     # TX bandwidth (RX BWs via decode_bws)
cr   = 8                         # TX coding rate 4/N (RX auto-detects from header)
sync_word    = 0x12              # 0x12 = MeshCore, 0x2B = Meshtastic
preamble_len = 8

[radio_868]
freq       = 869618000           # center frequency (Hz)
rx_channel = [1, 2]              # B210: 0=TRX_A 1=RX_A 2=RX_B 3=TRX_B
tx_channel = 0
rx_gain    = 40
tx_gain    = 75

[set_meshcore_868]
codec = "codec_meshcore_868"
radio = "radio_868"
decode_bws = [62500, 125000, 250000]  # simultaneous multi-BW decode

[[set_meshcore_868.decode]]
label     = "meshcore"
sync_word = 0x12
```

See `apps/config.toml` for full reference with all options.

## Tests

```sh
# C++ (11 suites including MultiSfDecoder, LDRO, benchmarks)
ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 120

# Hardware A/B test (requires companion device + lora_trx running)
.venv/bin/python3 scripts/apps/lora_test.py decode --matrix basic

# Python
.venv/bin/python3 -m unittest discover -s scripts/tests -p 'test_*.py' -v
```

## License

ISC
