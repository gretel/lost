# CBOR Schemas

All inter-process communication uses [CBOR (RFC 8949)](https://cbor.io/) —
a compact binary encoding of JSON-like data.

Python: `pip install cbor2`, then `cbor2.loads(datagram)`.

## RX Output

`lora_trx` outputs decoded frames as CBOR `lora_frame` maps via UDP (default
port 5555). Clients send a 1-byte registration datagram to
join the broadcast.

```bash
# Monitor lora_trx via UDP (default port 5555)
python3 scripts/lora_mon.py

# Monitor with custom address
python3 scripts/lora_mon.py --connect 192.168.1.10:5555

# Pipe to offline decoder
python3 scripts/lora_mon.py | python3 scripts/lora_decode_meshcore.py
```

```
{
  "type":         "lora_frame",       // text, always "lora_frame"
  "ts":           "2026-02-17T...",   // text, ISO 8601 timestamp
  "seq":          1,                  // uint, 1-based frame counter
  "phy": {                            // map, PHY-layer metadata
    "sf":         8,                  // uint, spreading factor
    "bw":         62500,              // uint, bandwidth in Hz
    "cr":         4,                  // uint, coding rate (1–4)
    "crc_valid":  true,               // bool
    "sync_word":  18,                 // uint (0x12 = 18)
    "snr_db":     12.3,               // float64, preamble-based SNR estimate (dB)
    "noise_floor_db": -42.1,           // float64, optional, EMA noise floor (dBFS)
    "peak_db":    -6.2                 // float64, optional, EMA peak amplitude (dBFS)
  },
  "rx_channel":   0,                  // uint, optional, RX channel index
  "payload":      h'48656C6C6F',      // bytes, raw payload
  "payload_len":  5,                  // uint
  "crc_valid":    true,               // bool (top-level duplicate for convenience)
  "cr":           4,                  // uint (top-level duplicate)
  "is_downchirp": false,              // bool
  "diversity": {                      // map, optional, only present in dual-RX mode
    "rx_channels":    [0, 1],         // uint array, channels that decoded the frame
    "decoded_channel": 1,             // uint, channel selected as best decode
    "snr_db":         [3.2, 5.1],     // float64 array, per-channel SNR
    "crc_mask":       3,              // uint, bitmask of CRC-OK channels (bit 0 = ch 0)
    "n_candidates":   2               // uint, number of decode candidates
  }
}
```

## TX Request

Sent to `lora_trx` via UDP on `udp_port` (default 5555). One CBOR map per
request. All fields except `type` and `payload` are optional — config file
defaults apply for any omitted field.

Send with `meshcore_tx.py --udp host:port` or any CBOR-capable UDP client.

```
{
  "type":         "lora_tx",          // text, required, must be "lora_tx"
  "payload":      h'48656C6C6F',      // bytes, required, 1–255 bytes
  "cr":           4,                  // uint, optional, coding rate (1–4)
  "sync_word":    18,                 // uint, optional (0x12 = 18)
  "preamble_len": 8,                  // uint, optional
  "repeat":       1,                  // uint, optional, transmit count
  "gap_ms":       1000,               // uint, optional, gap between repeats
  "dry_run":      false               // bool, optional, skip actual transmission
}
```

## TX Acknowledgement

Sent back to the requesting client's UDP address after TX completes.

```
{
  "type":  "lora_tx_ack",            // text
  "seq":   1,                        // uint, matches request order (1-based)
  "ok":    true                      // bool, false on error
}
```

## Subscribe

Sent to `lora_trx` to register as a client and optionally filter frames by
sync word. Re-sending the same message acts as a keepalive and updates the
filter. Clients that don't subscribe (or send a non-CBOR datagram) receive
all frames — backward compatible with the previous `b"sub"` registration.

Dead clients (10 consecutive `sendto` failures) are automatically pruned.

```
{
  "type":       "subscribe",          // text, required
  "sync_word":  [18]                  // uint array, optional, filter by sync word
}
```

`sync_word` values: `0x12` (18) = MeshCore/Reticulum, `0x2B` (43) = Meshtastic,
`0x34` (52) = LoRaWAN. Omit the field to receive all frames.

## Config (sent on subscribe)

Sent once to a client immediately after it subscribes. Contains the current
PHY parameters, gain settings, and server metadata. Allows clients to
display configuration without needing local access to `config.toml`.

```
{
  "type":  "config",                   // text, always "config"
  "phy": {                             // map, PHY parameters
    "freq":       869618000.0,         // float64, center frequency (Hz)
    "sf":         8,                   // uint, spreading factor
    "bw":         62500,               // uint, bandwidth (Hz)
    "cr":         4,                   // uint, coding rate (1–4)
    "sync_word":  18,                  // uint (0x12 = 18)
    "preamble":   8,                   // uint, preamble symbols
    "rx_gain":    40.0,                // float64, RX gain (dB)
    "tx_gain":    74.0                 // float64, TX gain (dB)
  },
  "server": {                          // map, server metadata
    "device":          "uhd",          // text, SDR backend
    "status_interval": 10,             // uint, seconds between status heartbeats (0 = off)
    "sample_rate":     250000.0        // float64, sample rate (S/s)
  }
}
```

## Status Heartbeat

Broadcast periodically to **all** connected clients (ignores sync_word
filters). Interval is configured via `status_interval` in `config.toml`
(default 10 seconds, 0 = disabled).

```
{
  "type":  "status",                   // text, always "status"
  "ts":    "2026-02-25T12:00:00Z",     // text, ISO 8601 UTC timestamp
  "phy": {                             // map, current gain settings
    "rx_gain":  40.0,                  // float64, RX gain (dB)
    "tx_gain":  74.0                   // float64, TX gain (dB)
  },
  "frames": {                          // map, frame counters (since startup)
    "total":    42,                    // uint, total frames detected
    "crc_ok":   38,                    // uint, frames with valid CRC
    "crc_fail": 4                      // uint, frames with CRC failure
  }
}
```

## Spectrum (waterfall data)

Broadcast periodically to **all** connected clients when spectrum data is
available. Generated by SpectrumTap in FrameSync during DETECT state.
Bins are float32 LE magnitude values in dBFS, DC-centered (fftshift applied).

```
{
  "type":         "spectrum",            // text, always "spectrum"
  "bins":         h'...',                // bytes, fft_size x float32 LE, dBFS
  "fft_size":     1024,                  // uint, number of FFT bins
  "center_freq":  869618000.0,           // float64, center frequency (Hz)
  "sample_rate":  250000.0               // float64, sample rate (S/s)
}
```

Display with: `lora_waterfall.py --connect 127.0.0.1:5555`

## Spectrum TX

Same schema as Spectrum above, but with `type: "spectrum_tx"`. Generated from
TX IQ data pushed into a separate SpectrumTap during `handle_tx_request`,
allowing the waterfall to show TX bursts alongside the RX spectrum.

```
{
  "type":         "spectrum_tx",         // text, always "spectrum_tx"
  "bins":         h'...',                // bytes, fft_size x float32 LE, dBFS
  "fft_size":     1024,                  // uint, number of FFT bins
  "center_freq":  869618000.0,           // float64, center frequency (Hz)
  "sample_rate":  250000.0               // float64, sample rate (S/s)
}
```

## Runtime Reconfig Request

Sent to `lora_trx` via UDP to change PHY parameters at runtime without
restarting. All fields except `type` are optional — only the parameters
present in the request are changed. The change takes effect on the next
scheduler work cycle (typically within milliseconds). Any in-progress frame
decode is abandoned; the receiver restarts with the new parameters.

After applying the changes, `lora_trx` broadcasts an updated `config` message
to all connected clients.

```
{
  "type":         "lora_config",        // text, required
  "sf":           8,                    // uint, optional, spreading factor (7–12)
  "bw":           62500,                // uint, optional, bandwidth (Hz)
  "sync_word":    18,                   // uint, optional (0x12 = 18)
  "preamble":     8,                    // uint, optional, preamble symbols
  "freq":         869618000.0,          // float64, optional, center frequency (Hz)
  "rx_gain":      40.0,                 // float64, optional, RX gain (dB)
  "tx_gain":      74.0                  // float64, optional, TX gain (dB)
}
```

Changing `sf` or `bw` triggers `recalculate()` in both FrameSync and
DemodDecoder (rebuilds chirp tables, resets state machine). Changing `freq`
retunes the SoapySource. Gain changes are applied directly to the SDR device.

## Runtime Reconfig Acknowledgement

Sent back to the requesting client after a `lora_config` request is processed.

```
{
  "type":  "lora_config_ack",          // text
  "ok":    true                        // bool
}
```

## `lora_trx` UDP Protocol

`lora_trx` exposes a single UDP port (default 5555, IPv6 dual-stack) for both
RX frame output and TX request input.

### Client Subscription

Clients send a CBOR `subscribe` message to register and set a sync word
filter. Any non-CBOR datagram also registers the sender (no filter — receives
all frames). Re-subscribing from the same address updates the filter and
resets the failure counter.

### RX → Client

Decoded frames are broadcast as CBOR `lora_frame` maps (see RX Output above)
to subscribed clients. Clients with a `sync_word` filter only receive frames
matching their filter. Clients without a filter receive all frames.

### Client → TX

Clients send CBOR `lora_tx` maps (see TX Request above) as UDP datagrams.
`lora_trx` transmits the frame and sends a `lora_tx_ack` back to the
requesting client. TX requests also register the sender if not already
subscribed.

### Client → Runtime Reconfig

Clients send CBOR `lora_config` maps (see Runtime Reconfig Request above) to
change PHY parameters at runtime. `lora_trx` applies the changes to the live
RX pipeline, sends a `lora_config_ack` back, and broadcasts an updated
`config` message to all clients.

### `lora_mon.py`

Default: `--connect 127.0.0.1:5555`. Subscribes with `sync_word=[0x12]`
(MeshCore/Reticulum), then decodes and displays received CBOR frames.
Supports IPv6 bracket notation (`[::1]:5555`).

### `meshcore_tx.py --udp`

Sends a CBOR TX request via UDP: `meshcore_tx.py --udp 127.0.0.1:5555 send
--dest <key> "message"`.

## Python Examples

```python
# Subscribe to MeshCore/Reticulum frames only
import socket
import cbor2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sub = cbor2.dumps({"type": "subscribe", "sync_word": [0x12]})
sock.sendto(sub, ("127.0.0.1", 5555))
while True:
    data, addr = sock.recvfrom(65536)
    frame = cbor2.loads(data)
    if frame.get("type") == "lora_frame":
        snr = frame.get("phy", {}).get("snr_db")
        ch = frame.get("rx_channel")
        info = f"#{frame['seq']} {frame['payload_len']}B CRC={'OK' if frame['crc_valid'] else 'FAIL'}"
        if snr is not None:
            info += f" SNR={snr:.1f}dB"
        if ch is not None:
            info += f" ch={ch}"
        print(info)
```

```python
# Subscribe to all frames (no filter)
import socket
import cbor2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sub = cbor2.dumps({"type": "subscribe"})
sock.sendto(sub, ("127.0.0.1", 5555))
```

```python
# Send TX request to lora_trx via UDP
import socket
import cbor2

packet = bytes.fromhex("48656C6C6F")  # "Hello"
msg = cbor2.dumps({"type": "lora_tx", "payload": packet})
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("127.0.0.1", 5555))
```

```python
# Change PHY parameters at runtime (e.g. switch to SF12, 125 kHz)
import socket
import cbor2

msg = cbor2.dumps({"type": "lora_config", "sf": 12, "bw": 125000})
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("127.0.0.1", 5555))
data, _ = sock.recvfrom(65536)
ack = cbor2.loads(data)
print(f"config applied: {ack['ok']}")
```

