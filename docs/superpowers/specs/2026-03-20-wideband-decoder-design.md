# Wideband Multimode Decoder — Design Spec

**Date:** 2026-03-20
**Status:** Approved
**Branch:** `feature/streaming-scan` (extends existing work)

## Problem

`lora_trx` currently operates at 250 kHz sample rate, decoding signals within
a single ~250 kHz channel. The streaming scan pipeline (`lora_scan`) already
captures the entire EU868 band (863-870 MHz) at 16 MS/s but only detects
signals (CAD), it cannot decode them.

## Goal

Create a `WidebandDecoder` block that fuses the scan pipeline's wideband
capture + L1 energy detection with MultiSfDecoder's full decode pipeline.
One block replaces the 8-block `lora_trx` graph with a 2-block wideband
graph: `SoapySource → WidebandDecoder → FrameSink`.

## Architecture

### Scan-triggered decode

L1 energy gating identifies hot channels (0-3 at any moment in EU868).
Only hot channels get channelization + decode chains. A small fixed pool
of 8 ChannelLane slots handles any realistic traffic density.

### Graph

```
SoapySource (16 MS/s, 866.5 MHz) → WidebandDecoder → FrameSink
```

Two blocks. Replaces `SoapySource → Splitter → 3×MultiSfDecoder → 3×FrameSink`.

### WidebandDecoder block

```cpp
struct WidebandDecoder : gr::Block<WidebandDecoder, gr::NoDefaultTagForwarding> {
    gr::PortIn<cf32>      in;
    gr::PortOut<uint8_t>  out;       // decoded bytes + tags
    gr::PortOut<DataSet<float>, Async> status_out;

    float    sample_rate{16000000.f};
    float    center_freq{866500000.f};
    float    channel_bw{62500.f};
    float    buffer_ms{512.f};
    float    min_ratio{8.0f};
    uint32_t max_channels{8};
    std::string probe_bws{"62500,125000,250000"};
};
```

### ChannelLane (per active channel)

Phase-continuous streaming channelizer + 6 SfLane state machines:
- Persistent NCO phase across processBulk calls
- Integrate-and-dump decimation with running accumulator
- Feeds narrowband samples to SfLane state machines
- Idle timeout: deactivate after 10 sweeps of no energy

### State machine

ACCUMULATE → PROBE_AND_ACTIVATE → DECODE → REPORT → ACCUMULATE

- ACCUMULATE: L1 energy snapshots (same as ScanController)
- PROBE_AND_ACTIVATE: CAD on hot channels, activate/deactivate ChannelLanes
- DECODE: every processBulk, push wideband IQ through active ChannelLanes
- REPORT: emit status on async port

### Shared code extraction

- `algorithm/SfLane.hpp` — extracted from MultiSfDecoder's multisf_detail
- `algorithm/RingBuffer.hpp` — extracted from ScanController

## Files

| File | Action |
|------|--------|
| `blocks/.../algorithm/SfLane.hpp` | NEW — extract from MultiSfDecoder |
| `blocks/.../algorithm/RingBuffer.hpp` | NEW — extract from ScanController |
| `blocks/.../WidebandDecoder.hpp` | NEW — fused scan+decode block |
| `blocks/.../MultiSfDecoder.hpp` | MODIFY — include shared SfLane |
| `blocks/.../ScanController.hpp` | MODIFY — include shared RingBuffer |
| `apps/graph_builder.hpp` | MODIFY — add build_wideband_graph() |
| `apps/config.toml` | MODIFY — add [trx] wideband option |
| `apps/lora_trx.cpp` | MODIFY — wideband mode dispatch |
| `test/qa_lora_wideband.cpp` | NEW — unit tests |

## Milestones

1. M1: Extract SfLane + RingBuffer to shared headers
2. M2: ChannelLane with phase-continuous NCO
3. M3: WidebandDecoder block (L1 + channel pool)
4. M4: Fuse SfLane decode into ChannelLane
5. M5: Graph integration + lora_trx wideband mode
6. M6: Hardware A/B test validation
