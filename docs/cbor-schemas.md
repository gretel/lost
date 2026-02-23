# CBOR Schemas

All inter-process communication uses [CBOR (RFC 8949)](https://cbor.io/) —
a compact binary encoding of JSON-like data.

Python: `pip install cbor2`, then `cbor2.loads(datagram)`.

## RX Output

Three CBOR output modes, combinable:

- **`--udp [host:]port`** — UDP server: binds port, broadcasts CBOR frames to
  registered clients. Clients send a 1-byte registration datagram to join.
- **`--cbor`** — concatenated CBOR maps written to stdout (replaces text).
- **Text** (default) — human-readable hex dump + ASCII to stdout.

`lora_trx` always enables UDP on `--port` (default 5555). `lora_rx` supports
`--udp` and `--cbor` flags. All three modes can operate simultaneously.

```bash
# Pipe to offline decoder (direnv adds build/apps to PATH)
lora_rx --cbor | python3 scripts/lora_decode_meshcore.py

# Save for later
lora_rx --cbor > captured.cbor

# Both UDP and stdout CBOR simultaneously
lora_rx --cbor --udp 5556

# Monitor lora_trx via UDP (default port 5555)
python3 scripts/lora_mon.py

# Monitor with custom address
python3 scripts/lora_mon.py --connect 192.168.1.10:5555
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
    "snr_db":     12.3                // float64, preamble-based SNR estimate (dB)
  },
  "rx_channel":   0,                  // uint, optional, RX channel index (lora_trx only)
  "payload":      h'48656C6C6F',      // bytes, raw payload
  "payload_len":  5,                  // uint
  "crc_valid":    true,               // bool (top-level duplicate for convenience)
  "cr":           4,                  // uint (top-level duplicate)
  "is_downchirp": false               // bool
}
```

## TX Request

Accepted by `lora_tx` on stdin and by `lora_trx` via UDP. One CBOR map per
request. All fields except `type` and `payload` are optional — CLI defaults
apply for any omitted field.

- **`lora_tx`**: reads CBOR from stdin (pipe mode).
- **`lora_trx`**: receives CBOR via UDP on `--port` (default 5555). Send with
  `meshcore_tx.py --udp host:port` or any CBOR-capable UDP client.

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

## TX Acknowledgement

Emitted by `lora_tx` on stdout (pipe mode) and by `lora_trx` via UDP
(sent back to the requesting client's address).

```
{
  "type":  "lora_tx_ack",            // text
  "seq":   1,                        // uint, matches request order (1-based)
  "ok":    true                      // bool, false on error
}
```

## `lora_trx` UDP Protocol

`lora_trx` exposes a single UDP port (default 5555, IPv6 dual-stack) for both
RX frame output and TX request input.

### Client Registration

Clients send any datagram (typically 1 byte, e.g. `\x00`) to register.
`lora_trx` records the sender address and broadcasts all decoded frames to
registered clients. Registration is implicit — any incoming datagram from an
unknown address registers it.

### RX → Client

Decoded frames are broadcast as CBOR `lora_frame` maps (see RX Output above)
to all registered clients via UDP datagrams.

### Client → TX

Clients send CBOR `lora_tx` maps (see TX Request above) as UDP datagrams.
`lora_trx` transmits the frame and sends a `lora_tx_ack` back to the
requesting client.

### `lora_mon.py`

Default: `--connect 127.0.0.1:5555`. Sends registration datagram, then
decodes and displays received CBOR frames. Supports IPv6 bracket notation
(`[::1]:5555`).

### `meshcore_tx.py --udp`

Sends a CBOR TX request via UDP: `meshcore_tx.py --udp 127.0.0.1:5555 send
--dest <key> "message"`.

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
import socket
import cbor2

# Connect to lora_trx UDP server (register + receive frames)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"\x00", ("127.0.0.1", 5555))  # register
while True:
    data, addr = sock.recvfrom(65536)
    frame = cbor2.loads(data)
    if frame.get("type") == "lora_frame":
        print(f"#{frame['seq']} {frame['payload_len']}B CRC={'OK' if frame['crc_valid'] else 'FAIL'}")
```

```python
import socket
import cbor2

# Send TX request to lora_trx via UDP
packet = bytes.fromhex("48656C6C6F")  # "Hello"
msg = cbor2.dumps({"type": "lora_tx", "payload": packet})
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("127.0.0.1", 5555))
```
