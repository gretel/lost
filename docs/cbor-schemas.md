# CBOR Schemas

All inter-process communication uses [CBOR (RFC 8949)](https://cbor.io/) —
a compact binary encoding of JSON-like data. Every message is a CBOR map with
a text key `"type"` that identifies the frame type. Recipients dispatch on
this string value.

**C++ encoder/decoder:** `blocks/include/gnuradio-4.0/lora/cbor.hpp` — a
minimal, hand-rolled single-header implementation. Encodes to / decodes from
`std::vector<uint8_t>`. The decoder only handles the top-level map; nested
maps are produced but never consumed on the C++ side.

**Python:** `cbor2` (`pip install cbor2`).

**Wireshark:** `scripts/lora_trx.lua` — dissects all frame types on UDP 5555.

## Frame types

| Type | Direction | Source | Port |
|------|-----------|--------|------|
| [`lora_frame`](#lora_frame) | server → clients | `FrameSink.hpp` | 5555 |
| [`lora_tx`](#lora_tx) | client → server | any CBOR client | 5555 |
| [`lora_tx_ack`](#lora_tx_ack) | server → requesting client | `lora_trx.cpp` | 5555 |
| [`subscribe`](#subscribe) | client → server | any CBOR client | 5555 |
| [`config`](#config) | server → client (on subscribe) | `lora_trx.cpp` | 5555 |
| [`status`](#status) | server → all clients (periodic) | `lora_trx.cpp` | 5555 |
| [`spectrum`](#spectrum) | server → all clients | `lora_trx.cpp` | 5555 |
| [`spectrum_tx`](#spectrum_tx) | server → all clients | `lora_trx.cpp` | 5555 |
| [`lora_config`](#lora_config) | client → server | any CBOR client | 5555 |
| [`lora_config_ack`](#lora_config_ack) | server → requesting client | `lora_trx.cpp` | 5555 |
| [`scan_result`](#scan_result) | lora_scan → consumers | `lora_scan.cpp` | TBD |

---

## `lora_frame`

Decoded RX frame. Produced by `FrameSink::buildFrameCbor()` once per decode
event, delivered to clients via `lora_trx` UDP broadcast.

```
{
  "type":         "lora_frame",       // text
  "ts":           "2026-02-17T...",   // text, ISO 8601 UTC
  "seq":          1,                  // uint, 1-based frame counter (per FrameSink)
  "phy": {                            // map
    "sf":             8,              // uint, spreading factor
    "bw":             62500,          // uint, bandwidth (Hz)
    "cr":             4,              // uint, coding rate (1–4)
    "crc_valid":      true,           // bool
    "sync_word":      18,             // uint (0x12 = 18)
    "snr_db":         12.3,           // float64, FFT-domain SNR estimate (dB)
    "noise_floor_db": -42.1,          // float64, optional, EMA noise floor (dBFS)
    "peak_db":        -6.2,           // float64, optional, EMA peak amplitude (dBFS)
    "snr_db_td":      -7.2            // float64, optional, time-domain SNR (dB)
  },
  "payload":      h'48656C6C6F',      // bytes, raw payload
  "payload_len":  5,                  // uint
  "crc_valid":    true,               // bool (top-level duplicate)
  "cr":           4,                  // uint (top-level duplicate)
  "is_downchirp": false,              // bool
  "rx_channel":   0,                  // uint, optional, RX channel index
  "decode_label": "SF8-sync12",       // text, optional, decode chain label
  "id":           "550e8400-...",      // text, UUIDv4, unique per decode event
  "payload_hash": 12345678901234      // uint, FNV-1a 64-bit hash
}
```

**Optional fields:** `noise_floor_db`, `peak_db`, `snr_db_td` are only
present when their EMA values have been initialised (value > -999). `rx_channel`
is only present in multi-channel configurations. `decode_label` is only present
when the decode chain has a non-empty label (configured via TOML).

**SNR fields:** `snr_db` is an FFT-domain estimate (peak-vs-rest energy ratio
averaged over preamble symbols) and includes ~SF×3 dB of FFT processing gain.
`snr_db_td` is a time-domain estimate (`signal_db - noise_floor_db`) that
normalises the FFT peak back to per-sample power, making it comparable to
RSSI-based SNR values reported by companion devices.

**Consumers:** `lora_mon.py`, `lora_agg.py`, `lora_waterfall.py`,
`lora_duckdb.py`, `meshcore_bridge.py`, `lora_trx.lua`.

### Aggregated frame (from lora_agg on :5555)

Same as `lora_frame`, plus a `diversity` sub-map added by `lora_agg.py`:

```
{
  // ... all lora_frame fields ...
  "diversity": {                      // map, added by lora_agg
    "n_candidates":    2,             // uint, decode chains that decoded this frame
    "decoded_channel": 1,             // uint, rx_channel of the winning decode
    "rx_channels":     [0, 1],        // uint array, per-candidate rx_channel
    "snr_db":          [3.2, 5.1],    // float64 array, per-candidate SNR
    "crc_mask":        3,             // uint, bitmask of CRC-OK candidates
    "gap_us":          20000,         // uint, microseconds from first to last candidate
    "source_ids":      ["uuid1", ...] // text array, UUID of each raw candidate
  }
}
```

---

## `lora_tx`

TX request sent from a client to `lora_trx`. Triggers transmission of the
payload using the server's current PHY parameters, with optional overrides
for coding rate, sync word, and preamble length.

```
{
  "type":         "lora_tx",          // text, required
  "payload":      h'48656C6C6F',      // bytes, required, 1–255 bytes
  "seq":          1,                  // uint, optional, echoed in lora_tx_ack
  "cr":           4,                  // uint, optional, coding rate (1–4)
  "sync_word":    18,                 // uint, optional (0x12 = 18)
  "preamble_len": 8,                  // uint, optional, preamble symbols
  "repeat":       1,                  // uint, optional, transmit count
  "gap_ms":       1000,               // uint, optional, gap between repeats (ms)
  "dry_run":      false               // bool, optional, skip actual transmission
}
```

**Note:** `lora_trx` only reads the fields listed above. Any additional keys
(e.g., `freq`, `sf`, `bw`) are silently ignored. To change PHY parameters at
runtime, use [`lora_config`](#lora_config) instead.

**Producers:** `meshcore_tx.py`, `meshcore_bridge.py`, `lora_agg.py` (proxy).

---

## `lora_tx_ack`

Sent back to the requesting client after a `lora_tx` request is processed
(or rejected).

```
{
  "type":   "lora_tx_ack",            // text
  "seq":    1,                        // uint, echoed from lora_tx request
  "ok":     true,                     // bool, false on error
  "error":  "channel_busy"            // text, optional, reason for rejection
}
```

**Error values:** `"channel_busy"` (LBT timeout), `"tx_queue_full"` (queue at
capacity). The `error` field is only present when `ok` is `false`.

**Consumers:** `lora_agg.py` (pass-through), `lora_waterfall.py` (records TX
time), `lora_trx.lua`.

---

## `subscribe`

Sent to `lora_trx` (or `lora_agg`) to register as a client and optionally
filter frames by sync word. Re-sending acts as a keepalive and updates the
filter. Any non-CBOR datagram also registers the sender (no filter — receives
all frames). Dead clients (10 consecutive `sendto` failures) are pruned.

```
{
  "type":       "subscribe",          // text, required
  "sync_word":  [18]                  // uint array, optional, filter by sync word
}
```

`sync_word` values: `0x12` (18) = MeshCore/Reticulum, `0x2B` (43) =
Meshtastic, `0x34` (52) = LoRaWAN. Omit the field to receive all frames.

**Producers:** `lora_common.py` (used by `lora_mon.py`, `lora_waterfall.py`,
`lora_duckdb.py`), `lora_agg.py` (upstream keepalive), `meshcore_bridge.py`.

---

## `config`

Sent once to a client immediately after it subscribes. Also re-broadcast to
all clients after a `lora_config` change. Contains the current PHY parameters,
server metadata, and passthrough config for Python scripts.

```
{
  "type":  "config",                  // text
  "phy": {                            // map, 8 keys
    "freq":       869618000.0,        // float64, center frequency (Hz)
    "sf":         8,                  // uint, spreading factor
    "bw":         62500,              // uint, bandwidth (Hz)
    "cr":         4,                  // uint, coding rate (1–4)
    "sync_word":  18,                 // uint (0x12 = 18)
    "preamble":   8,                  // uint, preamble symbols
    "rx_gain":    40.0,               // float64, RX gain (dB)
    "tx_gain":    74.0                // float64, TX gain (dB)
  },
  "server": {                         // map, 3 keys
    "device":          "uhd",         // text, SDR backend
    "status_interval": 10,            // uint, seconds between status heartbeats (0 = off)
    "sample_rate":     250000.0       // float64, sample rate (S/s)
  },
  "decode_chains": [                  // array of maps, one per decode chain
    {
      "label":     "SF8-sync12",      // text, decode chain label
      "sf":        8,                 // uint, spreading factor
      "sync_word": 18                 // uint, sync word
    }
  ],
  "raw": {                            // map, passthrough config for Python scripts
    "network": {                      // map, [network] TOML section
      "udp_listen":      "::",        // text, UDP listen address
      "udp_port":        5555,        // uint, UDP port
      "status_interval": 10,         // uint, seconds between status heartbeats
      "lbt":             true,        // bool, listen-before-talk enabled
      "lbt_timeout_ms":  2000,        // uint, max wait for channel clear (ms)
      "tx_queue_depth":  16           // uint, max queued TX requests
    },
    "aggregator": {                   // map, [aggregator] TOML section
      "upstream":   "127.0.0.1:5556", // text, lora_trx raw port (host:port)
      "listen":     ":::5555",        // text, consumer-facing bind address (host:port)
      "window_ms":  100               // uint, diversity combining window (ms)
    },
    "meshcore": {                     // map, [meshcore] TOML section
      "name":             "node1",    // text, optional, node display name
      "model":            "B210",     // text, optional, device model string
      "identity_file":    "id.bin",   // text, optional, path to identity file
      "keys_dir":         "keys/",    // text, optional, path to keys directory
      "channels_dir":     "chan/",    // text, optional, path to channels directory
      "contacts_dir":     "cont/",   // text, optional, path to contacts directory
      "lat":              48.123,     // float64, optional, latitude
      "lon":              11.456,     // float64, optional, longitude
      "port":             4200,       // uint, TCP listen port
      "region_scope":     "de-nord",  // text, optional, region scope hashtag
      "flood_scope":      "local",    // text, optional, flood scope string
      "client_repeat":    0,          // uint, 0=off, 1=on
      "path_hash_mode":   0,          // uint, 0-2
      "autoadd_config":   0,          // uint, bitmask
      "autoadd_max_hops": 4           // uint, 0-64
    },
    "logging": {                      // map, [logging] TOML section
      "level":  "INFO",              // text, log level (DEBUG, INFO, WARNING, ERROR)
      "color":  true                  // bool, ANSI color enabled
    }
  }
}
```

**Optional fields in `raw.meshcore`:** `name`, `model`, `identity_file`,
`keys_dir`, `channels_dir`, `contacts_dir`, `region_scope`, `flood_scope`
are omitted when the string is empty. `lat` and `lon` are omitted when zero.

**Consumers:** `lora_mon.py`, `lora_waterfall.py`, `meshcore_bridge.py`,
`lora_trx.lua`.

---

## `status`

Broadcast periodically to **all** connected clients (ignores sync_word
filters). Interval is configured via `status_interval` in `config.toml`
(default 10 seconds, 0 = disabled).

```
{
  "type":         "status",           // text
  "ts":           "2026-02-25T...",   // text, ISO 8601 UTC
  "phy": {                            // map, 2 keys
    "rx_gain":    40.0,               // float64, RX gain (dB)
    "tx_gain":    74.0                // float64, TX gain (dB)
  },
  "frames": {                         // map, 3 keys
    "total":      42,                 // uint, total frames since startup
    "crc_ok":     38,                 // uint, frames with valid CRC
    "crc_fail":   4                   // uint, frames with CRC failure
  },
  "rx_overflows": 0                   // uint, cumulative SoapySDR overflow count
}
```

**Note:** `rx_overflows` is a top-level key, not inside `frames`.

**Consumers:** `lora_mon.py`, `lora_waterfall.py`, `meshcore_bridge.py`,
`lora_trx.lua`.

---

## `spectrum`

RX waterfall data. Broadcast to all connected clients when spectrum data is
available. Generated by `SpectrumTap` in `FrameSync` during DETECT state.
Bins are float32 LE magnitude values in dBFS, DC-centred (fftshift applied).

```
{
  "type":         "spectrum",         // text
  "bins":         h'...',             // bytes, fft_size × float32 LE, dBFS
  "fft_size":     1024,               // uint, number of FFT bins
  "center_freq":  869618000.0,        // float64, center frequency (Hz)
  "sample_rate":  250000.0            // float64, sample rate (S/s)
}
```

**Consumers:** `lora_waterfall.py`, `lora_trx.lua`.

---

## `spectrum_tx`

TX waterfall data. Same schema as `spectrum` except `"type": "spectrum_tx"`.
Generated from TX IQ data during transmission, allowing the waterfall to show
TX bursts alongside the RX spectrum.

```
{
  "type":         "spectrum_tx",      // text
  "bins":         h'...',             // bytes, fft_size × float32 LE, dBFS
  "fft_size":     1024,               // uint
  "center_freq":  869618000.0,        // float64 (Hz)
  "sample_rate":  250000.0            // float64 (S/s)
}
```

**Consumers:** `lora_trx.lua`. Python clients receive this type but
`lora_waterfall.py` currently only renders `spectrum` (RX).

---

## `lora_config`

Runtime reconfig request. Sent to `lora_trx` to change PHY parameters without
restarting. All fields except `type` are optional — only the parameters
present in the request are changed. After applying the change, `lora_trx`
broadcasts an updated `config` message to all connected clients.

```
{
  "type":         "lora_config",      // text, required
  "sf":           8,                  // uint, optional, spreading factor (7–12)
  "bw":           62500,              // uint, optional, bandwidth (Hz)
  "sync_word":    18,                 // uint, optional (0x12 = 18)
  "preamble":     8,                  // uint, optional, preamble symbols
  "freq":         869618000.0,        // float64, optional, center frequency (Hz)
  "rx_gain":      40.0,               // float64, optional, RX gain (dB)
  "tx_gain":      74.0                // float64, optional, TX gain (dB)
}
```

Changing `sf` or `bw` triggers `recalculate()` in FrameSync and DemodDecoder
(rebuilds chirp tables, resets state machine). Changing `freq` retunes the
SoapySource. Gain changes are applied directly to the SDR device.

---

## `lora_config_ack`

Sent back to the requesting client after a `lora_config` request is processed.

```
{
  "type":  "lora_config_ack",         // text
  "ok":    true                       // bool
}
```

---

## `scan_result`

Channel activity report from `lora_scan`. Emitted after each L2/L3 detection
(not for noise-only channels). Intended for consumption by a second `lora_trx`
instance that can hop to active frequencies.

```
{
  "type":         "scan_result",      // text
  "ts":           "2026-03-11T...",   // text, ISO 8601 UTC
  "freq":         868100000.0,        // float64, channel center frequency (Hz)
  "bw":           62500,              // uint, channel bandwidth (Hz)
  "detected":     true,               // bool, L2 CAD detection result
  "peak_ratio_up":  5.23,             // float64, best upchirp peak ratio
  "peak_ratio_dn":  1.12,             // float64, best downchirp peak ratio
  "sf_est":       8,                  // uint, optional, estimated SF from L3 sweep (0 = unknown)
  "sweep":        1                   // uint, sweep counter (1-based)
}
```

**Status:** not yet implemented. `lora_scan` currently writes text to stdout.

---

## UDP protocol

`lora_trx` exposes a single UDP port (default 5555, IPv6 dual-stack) for both
RX frame output and TX/config input.

### Client subscription

Clients send a CBOR `subscribe` message to register and set a sync word
filter. Any non-CBOR datagram also registers the sender (no filter — receives
all frames). Re-subscribing from the same address updates the filter and
resets the failure counter.

### RX → client

Decoded frames are broadcast as `lora_frame` maps to subscribed clients.
Clients with a `sync_word` filter only receive frames matching their filter.

### Client → TX

Clients send `lora_tx` maps as UDP datagrams. `lora_trx` transmits the frame
and sends a `lora_tx_ack` back to the requesting client. TX requests also
register the sender if not already subscribed.

### Client → runtime reconfig

Clients send `lora_config` maps to change PHY parameters. `lora_trx` applies
the changes, sends a `lora_config_ack` back, and broadcasts an updated
`config` message to all clients.

### Aggregator

`lora_agg.py` subscribes to `lora_trx` on port 5556 (raw frames),
deduplicates across decode chains, and re-broadcasts aggregated frames
(with a `diversity` sub-map) on port 5555.

```bash
# Start aggregator (subscribes to lora_trx:5556, serves consumers on :5555)
python3 scripts/lora_agg.py

# Monitor aggregated frames (default port 5555)
python3 scripts/lora_mon.py

# Monitor raw frames directly from lora_trx
python3 scripts/lora_mon.py --connect 127.0.0.1:5556
```

## Python examples

```python
# Subscribe to MeshCore/Reticulum frames only
import socket, cbor2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sub = cbor2.dumps({"type": "subscribe", "sync_word": [0x12]})
sock.sendto(sub, ("127.0.0.1", 5555))
while True:
    data, addr = sock.recvfrom(65536)
    frame = cbor2.loads(data)
    if frame.get("type") == "lora_frame":
        phy = frame.get("phy", {})
        print(f"#{frame['seq']} {frame['payload_len']}B "
              f"CRC={'OK' if frame['crc_valid'] else 'FAIL'} "
              f"SNR={phy.get('snr_db', 0):.1f}dB")
```

```python
# Send TX request
import socket, cbor2

msg = cbor2.dumps({"type": "lora_tx", "payload": b"Hello"})
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("127.0.0.1", 5555))
```

```python
# Change PHY parameters at runtime
import socket, cbor2

msg = cbor2.dumps({"type": "lora_config", "sf": 12, "bw": 125000})
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("127.0.0.1", 5555))
data, _ = sock.recvfrom(65536)
ack = cbor2.loads(data)
print(f"config applied: {ack['ok']}")
```
