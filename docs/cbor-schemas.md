# CBOR Schemas

All inter-process communication uses [CBOR (RFC 8949)](https://cbor.io/) —
a compact binary encoding of JSON-like data.

Python: `pip install cbor2`, then `cbor2.loads(datagram)`.

## RX Output

Two CBOR output modes, combinable:

- **`--udp host:port`** — each frame sent as a CBOR-encoded UDP datagram.
- **`--cbor`** — concatenated CBOR maps written to stdout (replaces text).

When neither flag is set, stdout gets human-readable text (hex dump + ASCII).
When `--cbor` is set, stdout gets raw CBOR instead. Each CBOR map is
self-delimiting, so consumers can decode a concatenated stream directly:

```bash
# Pipe to offline decoder (direnv adds build/apps to PATH)
lora_rx --cbor | python3 scripts/lora_decode_meshcore.py

# Pipe to streaming monitor
lora_rx --cbor | python3 scripts/lora_mon.py --stdin

# Save for later
lora_rx --cbor > captured.cbor

# Both UDP and stdout CBOR simultaneously
lora_rx --cbor --udp 127.0.0.1:5556
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
    "sync_word":  18                  // uint (0x12 = 18)
  },
  "payload":      h'48656C6C6F',      // bytes, raw payload
  "payload_len":  5,                  // uint
  "crc_valid":    true,               // bool (top-level duplicate for convenience)
  "cr":           4,                  // uint (top-level duplicate)
  "is_downchirp": false               // bool
}
```

## TX Request (`lora_tx --stdin`)

Read from stdin by `lora_tx` in streaming mode. One CBOR map per
request. All fields except `type` and `payload` are optional — CLI defaults
apply for any omitted field.

```
{
  "type":         "lora_tx",          // text, required, must be "lora_tx"
  "payload":      h'48656C6C6F',      // bytes, required, 1–255 bytes
  "freq":         869618000,          // uint, optional, TX frequency in Hz
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

Emitted by `lora_tx --stdin` after processing each request.

```
{
  "type":  "lora_tx_ack",            // text
  "seq":   1,                        // uint, matches request order (1-based)
  "ok":    true                      // bool, false on error
}
```

## Python Examples

```python
# Read concatenated CBOR from stdin (pipe mode)
# Uses cbor_stream.read_cbor_seq() which handles pipe streaming correctly.
# Do NOT use cbor2.CBORDecoder(stream).decode() — it blocks on pipes.
import sys
from cbor_stream import read_cbor_seq

for frame in read_cbor_seq(sys.stdin.buffer):
    if not isinstance(frame, dict) or frame.get("type") != "lora_frame":
        continue
    print(f"#{frame['seq']} {frame['payload_len']}B CRC={'OK' if frame['crc_valid'] else 'FAIL'}")
```

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
