# CBOR Schemas

All inter-process communication uses [CBOR (RFC 8949)](https://cbor.io/) —
a compact binary encoding of JSON-like data.

Python: `pip install cbor2`, then `cbor2.loads(datagram)`.

## RX Output (`lora_rx_soapy --udp host:port`)

Each decoded frame is sent as a single CBOR-encoded UDP datagram by
`FrameSink`. Text output goes to stdout simultaneously.

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
    "sync_word":  18                  // uint (0x12 = 18)
  },
  "payload":      h'48656C6C6F',      // bytes, raw payload
  "payload_len":  5,                  // uint
  "crc_valid":    true,               // bool (top-level duplicate for convenience)
  "cr":           4,                  // uint (top-level duplicate)
  "protocol":     "meshcore_or_reticulum",  // text, guess from sync_word
  "is_downchirp": false               // bool
}
```

## TX Request (`lora_tx_soapy --stdin`)

Read from stdin by `lora_tx_soapy` in streaming mode. One CBOR map per
request. All fields except `type` and `payload` are optional — CLI defaults
apply for any omitted field.

```
{
  "type":         "lora_tx",          // text, required, must be "lora_tx"
  "payload":      h'48656C6C6F',      // bytes, required, 1–255 bytes
  "freq":         869525000,          // uint, optional, TX frequency in Hz
  "gain":         30,                 // uint, optional, TX gain in dB
  "sf":           8,                  // uint, optional, spreading factor (7–12)
  "bw":           62500,              // uint, optional, bandwidth in Hz
  "cr":           4,                  // uint, optional, coding rate (1–4)
  "sync_word":    18,                 // uint, optional (0x12 = 18)
  "preamble_len": 8,                  // uint, optional
  "repeat":       1,                  // uint, optional, transmit count
  "gap_ms":       1000,               // uint, optional, gap between repeats
  "dry_run":      false               // bool, optional, override --dry-run
}
```

## TX Acknowledgement (stdout)

Emitted by `lora_tx_soapy --stdin` after processing each request.

```
{
  "type":  "lora_tx_ack",            // text
  "seq":   1,                        // uint, matches request order (1-based)
  "ok":    true                      // bool, false on error
}
```

## Python Example

```python
import socket
import cbor2

# Receive frames via UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5556))
while True:
    data, addr = sock.recvfrom(65536)
    frame = cbor2.loads(data)
    print(f"#{frame['seq']} {frame['payload_len']}B CRC={'OK' if frame['crc_valid'] else 'FAIL'}")
```
