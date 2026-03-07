# RX enhancements design

> **For implementer:** Use superpowers:executing-plans to implement this plan
> task-by-task.

**Goal:** Improve the multi-antenna RX pipeline with correct diversity
combining, generalised N-decoder support, non-blocking TX, and channel
activity detection.

**Architecture:** Five tasks in dependency order. Each builds on the
previous. The existing 2-block RX pipeline (FrameSync → DemodDecoder) is
unchanged — all work is in how decode chains are composed, combined, and
wired in `lora_trx`.

**Tech stack:** C++23, GR4 framework, SoapySDR, boost::ut.

---

## Current RX topology

### Single-channel

```
SoapySimpleSource → FrameSync → DemodDecoder → FrameSink
```

### Dual-channel (MIMO)

```
SoapyDualSource ─out#0─→ FrameSync₀ → DemodDecoder₀ ─→ DiversityCombiner → FrameSink
                ─out#1─→ FrameSync₁ → DemodDecoder₁ ─→         ↑
```

Both channels share SF, bandwidth, and sync word. DiversityCombiner groups
decoded frames by FNV-1a payload hash, selects the best candidate
(CRC_OK preferred, then highest SNR), and emits one frame.

---

## Task A: DiversityCombiner timing fix

### Problem

Both RX chains decode the same frame but arrive ~40ms apart in different
scheduler cycles. The combiner's `age >= 2` drain emits the first
candidate before the second arrives. Result: two frames with
`diversity_n_candidates=1` instead of one frame with
`diversity_n_candidates=2`.

The real-time deadline (`timeout_symbols=30` = ~123ms at SF8/BW62.5k)
correctly covers all observed inter-chain gaps (max ~97ms on old FPGA,
typically same-cycle on updated FPGA). The problem is that the age-based
drain fires first.

### Design

Replace the age-based drain threshold from `age >= 2` to `age >= 4`.
This gives the real-time deadline primacy as the drain mechanism while
keeping age as a safety fallback for cases where wall-clock timing is
unreliable (e.g. tests, offline processing).

No other changes to the five emission conditions (complete, timeout,
single-input, aged, activity-based).

### Files

- Modify: `blocks/include/gnuradio-4.0/lora/DiversityCombiner.hpp`
  - Line with `age >= 2` → `age >= 4`
- Modify: `test/qa_lora_diversity.cpp`
  - Add test: staggered delivery across 3 `processBulk` cycles, verify
    both candidates are grouped (not emitted separately)

### Verification

- `ctest --test-dir build -R qa_lora_diversity --output-on-failure --timeout 60`
- Hardware: `lora_trx --rx-channels 0,1 --debug` — verify
  `diversity_n_candidates=2` on ambient frames

### Risk

Low. Single constant change with clear test coverage.

---

## Task B: N-decoder RX pipeline

### Problem

`lora_trx` supports exactly 1 or 2 RX channels with one SF and one sync
word. To monitor multiple SFs (e.g. SF7 + SF8), multiple sync words
(e.g. MeshCore 0x12 + Meshtastic 0x2B), or more than 2 antennas, the
graph construction needs to be generalised.

### Design

#### Splitter block

GR4 does not support fan-out from one port to multiple blocks natively.
Introduce a `Splitter<T>` block: 1 input, N outputs (dynamic port
count). `processBulk` copies the input span to all output spans.

```cpp
struct Splitter : gr::Block<Splitter, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>   in;
    std::vector<gr::PortOut<std::complex<float>>> out;

    gr::Size_t n_outputs = 2;
};
```

Tags are forwarded to all outputs. This is a pure infrastructure block
with no LoRa-specific logic.

#### Generalised graph construction

Replace the current `build_rx_graph()` with a generalised builder that
accepts a list of decode chain configurations:

```toml
# apps/config.toml
[[decode]]
sf = 8
sync_word = 0x12
label = "meshcore-sf8"

[[decode]]
sf = 7
sync_word = 0x2B
label = "meshtastic-sf7"
```

Each `[[decode]]` entry produces one `FrameSync → DemodDecoder` chain per
radio channel. All chains feed into a single `DiversityCombiner(n_inputs=N)`
where `N = n_radio_channels × n_decode_configs`.

#### Topology example (2 channels × 2 decode configs = 4 chains)

```
SoapyDualSource ─out#0─→ Splitter₀ ─out#0─→ FS(sf=8,sw=0x12) → DD₀ ─→ Combiner.in#0
                                    ─out#1─→ FS(sf=7,sw=0x2B) → DD₁ ─→ Combiner.in#1
                ─out#1─→ Splitter₁ ─out#0─→ FS(sf=8,sw=0x12) → DD₂ ─→ Combiner.in#2
                                    ─out#1─→ FS(sf=7,sw=0x2B) → DD₃ ─→ Combiner.in#3
                                                                        Combiner.out → FrameSink
```

When only one `[[decode]]` entry exists (the default), the Splitter is
omitted and the topology is identical to today's pipeline.

#### DiversityCombiner grouping semantics

With N-decoder, the combiner receives frames from chains decoding
different SFs or sync words. Frames with different payloads hash
differently and form separate pending groups — this is already correct.
Two chains decoding the same over-the-air frame at the same SF produce
identical payloads and hash-match into the same group.

The `n_inputs` property is set to the total chain count. A group is
"complete" when all chains that decoded the same frame have reported.
Since different-SF chains won't decode the same frame, groups will
typically have at most `n_radio_channels` candidates, not `n_inputs`.
The real-time timeout and age-based drain handle this correctly — groups
are emitted when the deadline expires, not when `n_inputs` candidates
arrive.

#### rx_channel assignment

Each decode chain gets a unique `rx_channel` value encoding both the
radio channel and the decode config:

```
rx_channel = radio_channel_index * 100 + decode_config_index
```

For example: radio 0, decode 0 → `rx_channel=0`; radio 0, decode 1 →
`rx_channel=1`; radio 1, decode 0 → `rx_channel=100`; radio 1, decode 1
→ `rx_channel=101`.

#### Dynamic PHY reconfiguration

`handle_lora_config()` currently pushes SF/bandwidth changes to the live
blocks via `settings().set()`. With N-decoder, it needs to iterate all
decode chains or target a specific one by `rx_channel`. For simplicity,
the first implementation applies changes to all chains sharing the same
`[[decode]]` label. Per-chain reconfiguration is deferred.

### Files

- Create: `blocks/include/gnuradio-4.0/lora/Splitter.hpp`
- Create: `test/qa_lora_splitter.cpp`
- Modify: `blocks/CMakeLists.txt` — add Splitter header to interface target
- Modify: `test/CMakeLists.txt` — add qa_lora_splitter
- Modify: `apps/lora_trx.cpp` — generalise `build_rx_graph()`,
  `add_decode_chain()`, `find_rx_blocks()`, `handle_lora_config()`
- Modify: `apps/config.toml` — add `[[decode]]` table array (with backward
  compatibility: if absent, use current `sf`/`sync_word` from `[codec_*]`)

### Verification

- `ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 60`
- Hardware: `lora_trx --debug` with 2 decode configs, verify both SFs
  produce decoded frames

### Risk

Medium. The Splitter block is trivial but the `lora_trx.cpp` graph
construction refactoring is the most complex change in this plan. GR4's
dynamic port connection API with string-based port names needs careful
testing.

---

## Task C: TX threading with GR4 io thread pool

### Problem

TX runs synchronously on the main thread. During transmission (~400ms for
a short frame, ~1.2s for ADVERT), the UDP I/O loop blocks. No messages
are processed, no status heartbeats are sent, and spectrum updates stop.

### Design

Dispatch TX work to the GR4 io thread pool
(`gr::thread_pool::Manager::defaultIoPool()`). The io pool is growable
(unlike the fixed CPU pool) and designed for blocking work.

```
Main thread (UDP loop)          IO pool thread
─────────────────────           ──────────────
receive "lora_tx" request
  → set tx_busy = true
  → dispatch to io pool ──────→ generate_iq()
  → continue UDP loop           transmit()
  → process other requests       push TX spectrum tap
                                 send lora_tx_ack
                                 set tx_busy = false
```

A `std::atomic<bool> tx_busy` flag prevents concurrent TX requests. If a
TX request arrives while `tx_busy` is true, an immediate `lora_tx_ack`
with `success=false` and `error="tx_busy"` is sent back.

#### Thread safety

- `sendto()` is thread-safe on POSIX — the io pool thread can send
  `lora_tx_ack` directly.
- `transmit_direct()` uses a persistent `TxState` (separate stream from
  RX) — no contention with the RX scheduler thread.
- `transmit_1ch()` creates an ephemeral graph on the io pool thread —
  fully isolated.
- The TX spectrum tap (`_tx_spectrum`) is already `std::shared_ptr` with
  atomic ref counting. The main thread reads it for FFT; the io pool
  thread writes it. Add a `std::mutex` around the spectrum state push/read
  to prevent torn reads.

### Files

- Modify: `apps/lora_trx.cpp`
  - Add `std::atomic<bool> tx_busy{false}`
  - Wrap `handle_tx_request()` body in `ioPool.execute([...]{...})`
  - Add `tx_busy` check and early `lora_tx_ack` rejection
  - Add `std::mutex` for TX spectrum tap access

### Verification

- `ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 60`
  (no test changes, but ensure nothing regresses)
- Hardware: send 2 rapid TX requests via `meshcore_tx.py --udp`, verify
  first transmits, second gets `tx_busy` rejection, UDP loop stays
  responsive during TX

### Risk

Low. The io pool is designed for this use case. The only subtlety is the
spectrum tap mutex, which is a straightforward addition.

---

## Task D: channel activity detection (CAD) block

### Problem

No channel activity detection. TX uses no listen-before-talk, and there
is no way to detect chirp presence without full decode.

### Design

Implement `ChannelActivityDetector` using the Vangelista & Calvagno
dual-window algorithm:

1. Collect `2N` IQ samples (`N = 2^SF`, 2 symbol periods)
2. Per window: dechirp with reference chirp, FFT, compute
   `peak_ratio = max|Y[k]| / mean|Y[k]|`
3. Detect if `peak_ratio > α` in **both** windows
4. Optionally run in parallel with downchirp reference (dual-chirp mode)

#### Block interface

```cpp
struct ChannelActivityDetector
    : gr::Block<ChannelActivityDetector, gr::NoDefaultTagForwarding> {
    using Description = Doc<"Detects LoRa chirp activity in a 2-symbol window">;

    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<uint8_t, gr::Async>  out;

    uint32_t sf         = 8;
    uint32_t bandwidth  = 62500;
    uint32_t os_factor  = 4;
    float    alpha      = 4.16f;   // SF-dependent, see table below
    bool     dual_chirp = true;
    bool     debug      = false;

    GR_MAKE_REFLECTABLE(ChannelActivityDetector, in, out,
                        sf, bandwidth, os_factor, alpha, dual_chirp, debug);
};
```

#### Output

One byte per detection window pair:
- Bit 0: upchirp detected
- Bit 1: downchirp detected

Output tag on detection:

| Key | Type | Description |
|-----|------|-------------|
| `cad_detected` | bool | true if any chirp detected |
| `cad_upchirp` | bool | upchirp detected |
| `cad_downchirp` | bool | downchirp detected |
| `cad_peak_ratio_up` | double | peak/mean ratio for upchirp |
| `cad_peak_ratio_down` | double | peak/mean ratio for downchirp |

#### Threshold table (P_fa = 10⁻³)

| SF | α | Min working SNR | Window (BW=62.5 kHz) |
|----|------|-----------------|----------------------|
| 7 | 4.23 | -7 dB | 4.1 ms |
| 8 | 4.16 | -10 dB | 8.2 ms |
| 9 | 4.09 | -13 dB | 16.4 ms |
| 10 | 4.04 | -16 dB | 32.8 ms |
| 11 | 3.98 | -19 dB | 65.5 ms |
| 12 | 3.91 | -22 dB | 131.1 ms |

#### Integration in lora_trx

Wire via Splitter (from Task B) in parallel with decode chains. Optional
— enabled by a `[cad]` config section. For listen-before-talk, the main
loop checks a `cad_active` atomic before dispatching TX.

### Files

- Create: `blocks/include/gnuradio-4.0/lora/ChannelActivityDetector.hpp`
- Create: `test/qa_lora_cad.cpp`
- Modify: `blocks/CMakeLists.txt` — add header
- Modify: `test/CMakeLists.txt` — add test target
- Modify: `apps/lora_trx.cpp` — optional CAD wiring (behind `[cad]` config)

### Test plan

1. **Known signal**: 2 preamble upchirps at SNR=+10 dB → expect detection
2. **Noise only**: Gaussian noise at P_fa threshold → expect no detection
3. **SNR sweep**: preamble + AWGN at min working SNR per SF → verify detection
4. **Downchirp**: conjugated preamble → expect `cad_downchirp=true`
5. **False alarm rate**: large noise corpus → measure empirical P_fa < 10⁻³

### Verification

- `ctest --test-dir build -R qa_lora_cad --output-on-failure --timeout 60`
- Hardware: `lora_trx --debug` with CAD enabled, verify detection traces
  on ambient MeshCore traffic

### Risk

Low. Well-defined algorithm with known thresholds from the paper.
Independent of the rest of the pipeline.

---

## Task E: SNR sensitivity test (deferred)

### Problem

No systematic measurement of decode sensitivity across SNR levels, SFs,
and diversity configurations.

### Design

Extend `qa_lora_burst.cpp` with a parameterised AWGN sweep that measures
packet reception rate (PRR) at each SNR point. Compare:

- Single decoder (baseline)
- Dual decoder with DiversityCombiner (selection diversity)
- Multiple SFs via N-decoder pipeline

This validates that Tasks A-D do not regress sensitivity.

### Deferred

Depends on Tasks A and B. Primarily a validation exercise.

---

## Dependency graph

```
Task A (combiner timing)
  └─→ Task B (N-decoder + Splitter)
        ├─→ Task C (TX threading)     [independent, but after B for clean base]
        └─→ Task D (CAD block)        [uses Splitter from B]
              └─→ Task E (SNR sweep)  [validates A-D]
```

## References

- Vangelista & Calvagno (2022) — CAD algorithm, threshold α tables
- Hou, Xia & Zheng (2022) — MALoRa: multi-antenna coherent combining
- Tapparel & Burg (GRCon 2024) — LoRa PHY, gr-lora_sdr reference
- Semtech AN1200.85 — hardware CAD registers and timing
