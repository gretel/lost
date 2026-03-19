# Streaming Scan Pipeline — Design Spec

**Date:** 2026-03-19
**Status:** Approved
**Branch:** `feature/streaming-scan`

## Problem

`lora_scan` is an imperative orchestrator that retunes SoapySource via
`settings().set()` + hard sleep, using shared-state atomics
(CaptureState/SpectrumState) to exchange data with the GR4 graph. This
works (sub-second sweeps at opt-4) but:

1. The shared-state bridges (spin-waits, mutexes) bypass GR4's data flow model
2. The 5ms settle sleep per L2 retune dominates probe latency
3. The imperative loop cannot be composed with other GR4 blocks

## Goals

1. **Make lora_scan a proper GR4 graph** — eliminate shared-state atomics,
   use async ports for all cross-block communication
2. **Reduce retune latency** — eliminate hardware retune entirely via digital
   channelization from the wideband stream

## Non-Goals

- Unifying lora_scan and lora_trx into one binary (deferred)
- UHD direct source block (phase 2, after SoapySDR version works)
- Full polyphase filter bank channelizer (simple freq-shift + decimate suffices)

## Key Insight

The B210 at 16 MS/s captures the entire EU868 ISM band (863–870 MHz = 7 MHz)
in a single tile. The L1 energy scan already operates without retuning. L2
CAD probing currently retunes to get a narrowband capture, but at 16 MS/s the
oversampling ratio is 256x relative to BW62.5k — the narrowband signal is
already in the wideband stream. Digital extraction (freq-shift + decimate)
eliminates the retune entirely.

With no retune, there is no upstream feedback. The graph is acyclic.

## Upstream Context

Ralph Steinhagen (fair-acc) recommended `gr::PortIn<T, gr::Async>` /
`gr::PortOut<T, gr::Async>` for low-latency control loops (~1–2µs). He
explicitly advised against `msgIn`/`msgOut` for tight feedback (slow/best-effort).
With the no-retune design, the tight feedback loop is eliminated — async ports
are used only for downstream data flow (spectrum energy, detection results).

## Architecture

### Graph Topology

```
SoapySource (16 MS/s, cf32, center=866.5 MHz)
    │
    └─► Splitter(2)
            │
            ├─► SpectrumTap ──[sync out]──► NullSink (or future decode chain)
            │       │
            │       └──[async out: DataSet<float>]──┐
            │                                        │
            └─► ScanController ◄────────────────────┘
                    │
                    ├──[async out: DataSet<float>]──► ScanSink (detections → CBOR/UDP)
                    └──[async out: DataSet<float>]──► (spectrum display)
```

### SpectrumTap Block (L1)

Passthrough block that computes FFT energy per channel on the wideband stream.

```cpp
struct SpectrumTap : gr::Block<SpectrumTap, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>              in;
    gr::PortOut<std::complex<float>>             out;       // sync passthrough
    gr::PortOut<DataSet<float>, gr::Async>       spectrum;  // async energy

    float    sample_rate;
    float    channel_bw{62500.f};
    uint32_t fft_size{1024};
    uint32_t fft_accumulate{64};
};
```

- Passes all input to sync output (1:1)
- Internally: accumulates FFT-sized windows, computes |FFT|², averages over
  `fft_accumulate` windows (~4ms at 64 × 1024-pt FFTs at 16 MS/s)
- Emits one `DataSet<float>` per accumulation cycle on async port
- DataSet contains per-channel energy in dBFS indexed by frequency

### ScanController Block (L2 + decisions)

Stateful sink block implementing the scan state machine. It consumes wideband
IQ into an internal ring buffer and emits detection results and spectrum data
on async output ports. There is no sync output — this is a terminal block for
the IQ data path.

```cpp
struct ScanController : gr::Block<ScanController, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>              in;           // sync: wideband IQ (sink)
    gr::PortIn<DataSet<float>, gr::Async>        energy_in;   // async: from SpectrumTap
    gr::PortOut<DataSet<float>, gr::Async>       detect_out;  // async: detection results
    gr::PortOut<DataSet<float>, gr::Async>       spectrum_out; // async: for display

    float    sample_rate;
    float    center_freq;
    float    min_ratio{8.0f};
    float    buffer_ms{512.f};
    // BW list, SF range, etc. from config
};
```

**Note on detect_out type:** Detection results use `DataSet<float>` (not
`property_map`) because `DataSet` is a proven async port payload type in GR4
(used by `StreamToDataSet`). Detection metadata (freq, sf, bw, ratio) is
carried in the DataSet's `signal_names`, `signal_quantities`, and `meta_information`
fields. FrameSink extracts these fields and serializes to CBOR.

**Internal state:**
- **IQ ring buffer**: ~512ms = 8M samples at 16 MS/s (~64 MB), configurable
  via `buffer_ms`. Uses overwrite-oldest policy — if L2 probing falls behind,
  the oldest IQ is silently overwritten. This is intentional: L2 probes should
  run on recent data, and a signal present during L1 detection may have ended
  by the time L2 runs. The probe detects whatever is currently in the buffer.
- **Channel energy accumulator**: updated from async `energy_in` reads
- **Per-channel probe state**: which channels probed this sweep, current probe
  index for multi-call PROBE traversal
- **Multi-SF CAD instances**: one per BW, pre-initialized with `initMultiSf()`

**Overflow tag handling:** When an `{overflow: true}` tag arrives from
SoapySource (propagated through Splitter), ScanController invalidates the
ring buffer contents (reset write index). The current accumulation cycle
restarts. This prevents CAD from running on discontinuous IQ data.

**processBulk behavior:**
1. Check for overflow tags — if present, reset ring buffer and restart cycle
2. Append incoming IQ to ring buffer (always consume all)
3. Check async `energy_in` — update channel energy map if new data
4. Run scan state machine (see below)

### Sweep State Machine

```
IDLE ──► ACCUMULATE ──► PROBE ──► REPORT ──► IDLE
                         │   │
                         └───┘  (one hot channel per processBulk call)
```

- **IDLE → ACCUMULATE**: Reset channel energy, clear probe flags, reset
  probe index to 0.
- **ACCUMULATE**: Count energy reports from SpectrumTap. After
  `l1_accumulate_count` reports (configurable, default 64 — representing
  ~256ms of accumulated energy at 4ms per report), compute per-channel
  average energy. Channels exceeding `median_energy * 6.0` are marked hot.
  Transition to PROBE with the hot channel list.
- **PROBE**: Process one hot channel per processBulk call. For the current
  channel at `_probe_index`:
  1. Freq-shift channel to baseband: `x[n] * exp(-j*2π*Δf*n/fs)`
  2. Decimate by os_factor (256x/128x/64x per BW)
  3. Run `detectMultiSf()` on narrowband IQ
  4. Store result, increment `_probe_index`
  5. If `_probe_index < hot_channels.size()`, remain in PROBE (next call
     processes next channel)
  6. If all hot channels probed, transition to REPORT
- **REPORT**: Emit detection results on `detect_out` (one DataSet per
  detection above `min_ratio`), emit spectrum summary on `spectrum_out`,
  return to IDLE.

### ScanSink Block (output)

Lightweight sink that receives detection results and spectrum data from
ScanController and serializes them to CBOR over UDP. This is a new block,
separate from FrameSink (which handles decoded frame bytes from the RX
pipeline). Keeping them separate avoids modifying FrameSink's existing
`PortIn<uint8_t>` interface.

```cpp
struct ScanSink : gr::Block<ScanSink> {
    gr::PortIn<DataSet<float>, gr::Async>  detections;  // from ScanController
    gr::PortIn<DataSet<float>, gr::Async>  spectrum;    // from ScanController

    std::string udp_host{"127.0.0.1"};
    uint16_t    udp_port{5557};
};
```

- Reads async inputs, extracts metadata from DataSet fields
- Serializes to CBOR (same wire format as current `lora_scan` events)
- Sends over UDP to subscribed clients
- Also logs detections to stderr (same format as current console output)

The main thread's role is reduced to: start graph, poll overflow counter
(via `BlockWrapper` dynamic_cast on SoapySource, same pattern as current
`lora_scan`), render status bar to stderr, handle signals.

### Digital Channelization (L2 extraction)

```
Ring buffer (16 MS/s, centered at f_center)
    │
    ├── freq_shift(f_ch - f_center)   // complex oscillator multiply
    └── decimate(os_factor)           // 256x/128x/64x per BW
         │
         └── narrowband IQ → detectMultiSf()
```

The existing `decimate()` function (integrate-and-dump box-car) is reused.
It has no explicit anti-alias filter, but the box-car itself acts as a crude
low-pass (sinc response with first null at `fs/os_factor`). This is the same
decimation path used by the current retune-based scanner and is
hardware-validated for all three BWs. If sensitivity testing (M5) reveals
aliasing issues at high decimation factors (256x for BW62.5k), a proper FIR
anti-alias filter can be added as an optimization — but the box-car is the
proven baseline.

Freq-shift is new: `x[n] * exp(-j*2π*Δf*n/fs)`. Implemented as a complex
NCO (numerically controlled oscillator) — precompute phase increment
`dφ = 2π*Δf/fs`, accumulate phase per sample. The `decimate()` function and
freq-shift helper should be extracted from `lora_scan.cpp` into a shared
header (`blocks/include/gnuradio-4.0/lora/algorithm/Channelize.hpp`) for
reuse by ScanController.

### Output Events

Same CBOR event types as current lora_scan. The CBOR wire format over UDP
is unchanged — `lora_spectrum.py` and `lora_perf.py` require no modifications.
The only change is the emission path: events originate from ScanController
(via async port → ScanSink → UDP) instead of directly from the main thread.
FrameSink handles CBOR serialization from the DataSet metadata fields.

| Event | Fields |
|-------|--------|
| `scan_sweep_start` | sweep_id, timestamp |
| `scan_detection` | freq, sf, bw, ratio, sweep_id |
| `scan_spectrum` | channel_energies[], timestamp |
| `scan_sweep_end` | sweep_id, duration_ms, n_detections |
| `scan_status` | sweep_count, overflow_count, avg_sweep_ms |

The `overflow_count` in `scan_status` is read from SoapySource's atomic
overflow counter via the existing `BlockWrapper` dynamic_cast pattern (see
gr4-dev skill). The main thread polls this periodically for the status bar
and includes it in status events sent to ScanSink.

### Configuration

```toml
[scan]
radio        = "radio_868"
center_freq  = 866500000
sample_rate  = 16000000
master_clock = 32000000
bws          = [62500, 125000, 250000]    # L2 probe bandwidths
min_ratio    = 8.0
buffer_ms    = 512
l1_fft_size  = 1024                       # FFT bins for L1 energy
l1_accumulate = 64                        # FFTs to average per energy report
l1_reports   = 64                         # reports before probing (= l1_accumulate_count)
channel_bw   = 62500                      # L1 channel grid spacing (independent of L2 bws)
```

## What Changes vs Current

| Component | Current | New |
|-----------|---------|-----|
| L1 energy | SpectrumTapBlock + shared SpectrumState + mutex + spin-wait | SpectrumTap with async output port |
| L2 probe | retune SoapySource + sleep + CaptureSink + shared CaptureState + spin-wait | Digital channelization from ring buffer |
| Control flow | Imperative loop on main thread | ScanController state machine in processBulk |
| Retune | `settings().set()` + 5ms sleep per probe | None — radio stays fixed |
| Output | Direct CBOR/UDP from main thread | Async port → FrameSink |
| Dead time | ~5ms per L2 probe + L1 paused during L2 | Zero — continuous stream |

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| 64 MB ring buffer memory | Configurable `buffer_ms`. 512ms default, reduce to 256ms (32 MB) if SF12 not needed |
| Digital channelization SNR loss | ~3–6 dB at band edges vs hardware retune. Acceptable for detection. Adaptive retune can be added later (phase 2) |
| processBulk latency during L2 CAD | One channel per call. CAD on 32k narrowband samples ~50µs. Well within 0.5ms call interval |
| GR4 async port maturity | Production-proven in Selector, StreamToDataSet, SyncBlock |
| Ring buffer overflow (slow L2) | Overwrite oldest data. L2 operates on most recent buffer content anyway |

## Phased Approach

**Phase 1 (this spec):** SoapySDR + streaming pipeline. No retune. Digital
channelization. All GR4 async ports.

**Phase 2 (future):** UHD direct source block for lower-level control
(timed commands, sub-ms retune if needed for sensitivity-critical cases).

**Phase 3 (future):** Evaluate PFB channelizer for simultaneous multi-channel
decode (gateway mode).

## Files to Create/Modify

### New files
- `blocks/include/gnuradio-4.0/lora/ScanController.hpp` — L2 + state machine
- `blocks/include/gnuradio-4.0/lora/SpectrumTap.hpp` — L1 FFT energy (replaces SpectrumTapBlock)
- `blocks/include/gnuradio-4.0/lora/ScanSink.hpp` — detection/spectrum CBOR serialization + UDP
- `blocks/include/gnuradio-4.0/lora/algorithm/Channelize.hpp` — freq-shift + decimate helpers (extracted from lora_scan.cpp)
- `test/qa_lora_scan.cpp` — unit tests for all three blocks

### Modified files
- `apps/lora_scan.cpp` — rewrite to build streaming graph, remove imperative loop
- `apps/graph_builder.hpp` — update `build_scan_graph()` for new topology
- `apps/config.hpp` / `apps/config.cpp` — new scan config fields
- `blocks/CMakeLists.txt` — register new blocks

### Removed/deprecated
- `blocks/include/gnuradio-4.0/lora/CaptureSink.hpp` — replaced by ring buffer
- Shared-state classes: `CaptureState`, `SpectrumState` — replaced by async ports
