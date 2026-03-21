# FIR Channelizer for WidebandDecoder — Design Spec

**Date:** 2026-03-21
**Status:** Approved
**Builds on:** `2026-03-20-wideband-decoder-design.md` (M1-M4 complete)

---

## 1. Problem

The WidebandDecoder's `ChannelSlot::pushWideband()` uses a two-stage
integrate-and-dump (box-car) decimator. At os_factor=256 (16 MS/s →
62.5 kHz), the box-car's sinc frequency response has first sidelobe at
-13 dB. Broadband noise from the full 16 MHz bandwidth aliases into the
62.5 kHz decode channel, causing ~13 dB SNR loss vs hardware-tuned
narrowband reception.

**Consequence:** Loopback tests pass (noise-free synthetic signals) but
hardware signals fail CRC. The WidebandDecoder cannot decode real
over-the-air frames.

**Root cause:** `sinc(x)` frequency response of the box-car filter. At
the Nyquist edge of the decimated signal, rejection is only -13 dB. A
proper anti-aliasing low-pass filter needs ≥-40 dB stopband.

---

## 2. Solution: Cascaded Half-Band FIR Decimator

Replace the integrate-and-dump with a cascade of half-band FIR filters,
each decimating by 2 with ≥-40 dB stopband rejection.

### 2.1 Why half-band

A half-band filter has the property that every other coefficient is zero
(except the center tap). For a 15-tap half-band filter, only 4 unique
non-zero coefficients exist (exploiting both the zero-tap and symmetric
properties). This gives 4 multiply-adds per output sample — comparable
to the box-car's 1 multiply-add, but with -40 dB stopband instead of
-13 dB.

The transition band of a half-band filter is symmetric around 0.25× the
input sample rate (i.e., 0.5× Nyquist). After decimation by 2, this
places the transition at the output Nyquist edge — exactly where the
anti-aliasing cutoff needs to be.

### 2.2 Cascade depth per target bandwidth

Each stage decimates by 2. The number of stages is `log2(os_factor)`.

| Target BW | os_factor | Stages | Intermediate rates (from 16 MS/s) |
|-----------|-----------|--------|-----------------------------------|
| 62.5 kHz  | 256       | 8      | 8M → 4M → 2M → 1M → 500k → 250k → 125k → 62.5k |
| 125 kHz   | 128       | 7      | 8M → 4M → 2M → 1M → 500k → 250k → 125k |
| 250 kHz   | 64        | 6      | 8M → 4M → 2M → 1M → 500k → 250k |

### 2.3 Filter coefficients

Single set of half-band coefficients, shared by all stages and channels.
Target: ≥-40 dB stopband with <0.1 dB passband ripple.

A half-band filter of length N has `(N-1)/4` unique non-zero coefficients
(exploiting symmetry + zero-tap property), plus the fixed center tap of
0.5. The number of taps determines achievable stopband rejection:

| Taps | Non-zero unique | MAC/output | Measured stopband |
|------|-----------------|------------|-------------------|
| 15   | 4               | 4          | ~-33 dB           |
| 19   | 5               | 5          | -38.9 dB          |
| 23   | 6               | 6          | **-45.1 dB**      |

**Decision:** Use **23 taps** (6 unique non-zero coefficients). 19 taps
only achieved -38.9 dB (short of -40 dB target). 23 taps gives -45.1 dB
with 0.048 dB passband ripple. The 50% compute increase over 15 taps
(6 vs 4 MAC/output) is still negligible relative to the total budget.

Coefficients computed via `scipy.signal.remez` with transition band
[0.4, 0.6] × Fs, half-band constraints enforced (odd taps zero except
center h[11] = 0.5):

```
h[0]  = h[22] = -7.193324475036694e-03
h[2]  = h[20] =  1.361692586579394e-02
h[4]  = h[18] = -2.628066406628573e-02
h[6]  = h[16] =  4.860759433245809e-02
h[8]  = h[14] = -9.646258355192097e-02
h[10] = h[12] =  3.150052960795258e-01
h[11]         =  0.5 (center tap)
```

### 2.4 Compute cost

Per active channel (BW62.5k, 8 stages, 6 MAC/output with 23 taps):
- Stage 1: 16M input/s → 8M output/s, 6 MAC per output = 48M MAC/s
- Stage 2: 8M → 4M, 6 MAC per output = 24M MAC/s
- Stage 3-8: geometric sum continues...
- Total: ~95M MAC/s per channel (geometric series converges to 2× first)

For 8 active channels across 3 BWs (24 slots max): ~2.3G MAC/s — well
within Apple M1's budget (~100 GFLOP/s single-core).

**Per-processBulk latency** at 8192-sample chunks (1953 calls/s):
24 slots × 8192 samples × 6 MAC ≈ 1.2M MACs per call ≈ 0.30 ms.
Budget per chunk: 8192/16M = 0.51 ms → ~59% CPU at full occupancy.
Tight but feasible in Release builds. In practice, most sweeps have
0-3 hot channels (6-9 active slots), using ~25% CPU.

**Memory per slot:** ~2.5 MB dominated by SfLane buffers (SF12 preamble
storage). FIR delay lines add 22 samples × 8 stages × 8 bytes = 1.4 KB
per slot (negligible). 24 slots total: ~60 MB for SfLane buffers.
Acceptable for desktop/M1.

---

## 3. Implementation Scope

### 3.1 New file: `algorithm/HalfBandDecimator.hpp`

**`HalfBandStage`** struct:
- `std::array<cf32, 23> delay` — delay line (23-tap filter, circular buffer)
- `uint32_t phase` — polyphase phase counter (0 or 1, for decimate-by-2)
- `void reset()`
- `cf32 processSample(cf32 input)` — returns decimated output every 2nd input, or sentinel

**`CascadedDecimator`** struct:
- `std::vector<HalfBandStage> stages`
- `std::vector<cf32> intermediate` — scratch buffer between stages
- `void init(uint32_t nStages)` — allocates stages and scratch
- `void reset()` — clears all delay lines
- `void process(std::span<const cf32> in, std::vector<cf32>& out)` — full cascade

The cascade processes stage-by-stage: stage 0 processes all input samples
producing N/2 outputs into a scratch buffer. Stage 1 processes those N/2
samples producing N/4 outputs. And so on. This is more cache-friendly
than per-sample cascading (which would chase pointers through 8 stages
for every input sample).

### 3.2 Modified: `ChannelSlot` in `WidebandDecoder.hpp`

**Remove:**
- `decim1Accum`, `decim1Count`, `stage1Factor`
- `decim2Accum`, `decim2Count`, `stage2Factor`
- The two-stage factor computation in `activate()`
- The integrate-and-dump loop in `pushWideband()`

**Add:**
- `CascadedDecimator decimator`
- `uint32_t decodeBw` field

**`activate()` changes:**
- Assert `osFactor` is a power of 2: `assert((osFactor & (osFactor - 1)) == 0)`
- Compute `nStages = __builtin_ctz(osFactor)` (count trailing zeros = log2)
- Call `decimator.init(nStages)`
- Store `decodeBw`

**Power-of-2 constraint:** The cascaded half-band design requires
os_factor to be a power of 2. Valid configurations at 16 MS/s:
BW62.5k (os=256), BW125k (os=128), BW250k (os=64), BW500k (os=32).
The `start()` method validates that `sample_rate / bw` is a power of 2
for each BW in `decode_bws`, and logs an error + skips invalid BWs.
Non-power-of-2 os_factors (e.g., 10 MS/s / 62.5 kHz = 160) require
a different sample rate or a rational resampler stage — out of scope.

**`pushWideband()` changes:**
- NCO mix loop stays the same (phase-continuous, double precision)
- After mixing, call `decimator.process(mixed, nbAccum)` instead of
  the integrate-and-dump loop

### 3.3 Modified: `WidebandDecoder` block settings and channel pool

**New/changed settings:**
- `decode_bw` (existing `float`) repurposed: ignored when `decode_bw_str`
  is set. Kept for backward compatibility with single-BW configs.
- `decode_bw_str`: `std::string` defaulting to `"62500,125000,250000"`.
  Parsed in `start()` into `_decodeBws: std::vector<uint32_t>` (internal).
  String type avoids GR4 `GR_MAKE_REFLECTABLE` limitations with vector
  types.
- `max_channels` default increased from 8 to 24.

**Data structure changes:**
- `_activeChannelMap` changes from `vector<uint32_t>` (channelIdx → slotIdx)
  to `vector<std::vector<uint32_t>>` (channelIdx → list of slotIdx, one
  per active BW). Initialized in `start()` with `_nChannels` empty vectors.
- Each `ChannelSlot` stores `decodeBw` to identify which BW it serves.

**`updateActiveChannels()` changes:**
For each hot channel, iterate `_decodeBws` (narrowest first). For each
BW, check if a slot already exists for this (channel, bw) pair via
`_activeChannelMap[ch]`. If not, find a free slot and activate it.

```
for ch in hotChannels:
    for bw in _decodeBws:  // sorted narrowest-first
        already_active = any slot in _activeChannelMap[ch] has decodeBw == bw
        if already_active: continue
        freeSlot = find first Idle slot
        if none: break
        slots[freeSlot].activate(freq, center, rate, rate/bw, ch, bw, ...)
        _activeChannelMap[ch].push_back(freeSlot)
```

Deactivation mirrors: when a channel leaves `hotChannels`, ALL slots in
`_activeChannelMap[ch]` are deactivated and the vector is cleared.

**Narrowest-BW-first dedup:**
When `finishFrame()` succeeds with CRC OK, the slot's `decodeBw` is
checked. If it's the narrowest BW for this channel, mark all wider-BW
slots in `_activeChannelMap[slot.channelIdx]` where `decodeBw > slot.decodeBw`
as `Draining`. Their SfLanes reset on the next processBulk cycle. This
prevents duplicate frame output from chirp-slope equivalences.

The existing `bw` parameter in `activate()` (used for SfLane init) is
set to the slot's `decodeBw` — they are the same value. SfLanes run at
os_factor=1 after the channelizer decimates to 1× the decode BW.

### 3.4 Not changed

These components are proven and require zero modification:

- L1 energy detection (`computeEnergySnapshot`, `findHotChannels`)
- Ring buffer (`RingBuffer.hpp`)
- SfLane state machines (`SfLane.hpp`)
- DC removal IIR filter
- Overflow tag handling
- NCO phase accumulator (double precision, wraps every 1024 samples)
- `processDetect` / `processSync` / `processOutput` / `finishFrame`
  (entire decode pipeline after channelizer)
- Output tags and FrameSink integration
- `channelCenterFreq()` helper

---

## 4. Multi-BW Decode Architecture

### 4.1 Slot allocation

Each hot channel consumes up to `decode_bws.size()` slots from the pool.
With 3 BWs and `max_channels=24`, the system supports up to 8 simultaneous
hot channels each decoded at all three bandwidths.

### 4.2 BW-specific cascade depths

| decode_bw | os_factor (at 16 MS/s) | Cascade stages |
|-----------|------------------------|----------------|
| 62500     | 256                    | 8              |
| 125000    | 128                    | 7              |
| 250000    | 64                     | 6              |

### 4.3 Chirp slope equivalence handling

Same-slope equivalences (`k = BW²/2^SF`):
- SF8/BW62.5k = SF10/BW125k = SF12/BW250k (k = 15,258,789)
- SF7/BW62.5k = SF9/BW125k = SF11/BW250k

The BW62.5k slot's SfLanes will always identify the correct SF for its BW.
The wider-BW slots may also detect the signal at a higher SF. The
narrowest-BW-first dedup (§3.3) ensures only the most precise SF
identification is emitted.

### 4.4 Signals only visible at wider BWs

SF7/BW125k (LoRaWAN standard) has `k = 122,070,312` — no BW62.5k
equivalent. It can only be decoded by a BW125k slot. Similarly,
SF7/BW250k and SF8/BW250k have no narrower-BW equivalents. The multi-BW
slot activation ensures these signals are captured.

---

## 5. Verification Plan

### 5.1 Unit tests (new)

1. **HalfBandStage stopband rejection:** Inject complex exponential tones
   at 0.6, 0.7, 0.8, 0.9, 1.0 × Nyquist. Verify output power ≤ -40 dB
   relative to passband.

2. **HalfBandStage passband flatness:** Inject tones at 0.0, 0.1, 0.2,
   0.3, 0.4 × Nyquist. Verify output power within ±0.1 dB of unity.

3. **CascadedDecimator phase continuity:** Process a long signal in
   varying chunk sizes (1, 7, 128, 8192 samples). Compare concatenated
   output to single-call processing of the full signal. Must be bit-exact.

4. **CascadedDecimator noise rejection:** Feed white Gaussian noise
   (fixed seed for reproducibility) at 16 MS/s through 8-stage cascade
   (os=256). Measure output noise power. Verify reduction is within 1 dB
   of theoretical (10*log10(256) = 24 dB processing gain, with ≤1 dB
   excess from filter transition band).

### 5.2 Integration tests (modified existing)

5. **Loopback decode:** Existing `qa_lora_wideband.cpp` tests must pass
   unchanged — clean signal decode at all SFs.

6. **Noisy loopback decode:** New test: inject LoRa signal at -5 dB SNR
   (wideband) into 16 MS/s stream. Verify FIR-based WidebandDecoder
   decodes successfully where box-car version fails.

### 5.3 Hardware validation

7. **A/B test:** `trx_ab_loop.py` with Heltec V3 companion sending
   ADVERTs at SF8/BW62.5k, SF7/BW125k, SF8/BW250k. Compare CRC-valid
   frame count between box-car and FIR versions. Success criterion:
   FIR version achieves ≥80% CRC-valid rate across all configs (box-car
   baseline: 0% at BW62.5k).

---

## 6. Milestones

| ID | Description | Depends on | Deliverable |
|----|-------------|------------|-------------|
| M1 | HalfBandDecimator implementation + unit tests | — | `algorithm/HalfBandDecimator.hpp`, test in `qa_lora_wideband.cpp` |
| M2 | Replace box-car in ChannelSlot | M1 | Modified `pushWideband()`, loopback tests pass |
| M3 | Multi-BW slot activation + dedup | M2 | `decode_bws` setting, `updateActiveChannels()` changes |
| M4 | Noisy loopback test | M2 | New test case validating SNR improvement |
| M5 | Hardware A/B validation | M3 | `trx_ab_loop.py` results for 3 BW configs |

---

## 7. Alternatives Considered

### 7.1 Polyphase Filter Bank (PFB)

Single PFB channelizes ALL 204 channels simultaneously with -60 dB+
rejection and O(K log K) total cost. Rejected because:
- Processes all 204 channels continuously, wasting CPU on ~200 idle
  channels in sparse EU868 traffic
- Complex data routing (K demuxed streams to SfLane selectors)
- New block type with significantly more implementation complexity
- The scan-triggered architecture (only channelize hot channels) is the
  right strategy for sparse traffic

### 7.2 Hybrid coarse-PFB + per-tile FIR

16-point PFB splits 16 MS/s into 1 MHz tiles, then per-hot-tile FIR
cascade (4 stages instead of 8). Rejected because:
- Two new components vs one
- Tile boundary effects require dual processing for edge signals
- Marginal benefit over pure cascaded half-band (which already has
  geometric cost reduction per stage)

### 7.3 Keep box-car, improve with window function

Apply a raised-cosine or Blackman window to the box-car averaging
kernel. Would improve stopband from -13 dB to ~-30 dB but requires
wider kernel (more taps than a simple average), losing the box-car's
simplicity without reaching the -40 dB target. Not competitive with
proper half-band FIR design.

---

## 8. Deferred Optimizations

- **SIMD vectorization:** The half-band FIR inner loop (5 multiply-adds
  per output) is a candidate for NEON/SSE auto-vectorization. The initial
  implementation uses `std::complex<float>` for clarity. If profiling
  shows the FIR cascade is a bottleneck, split into separate real/imaginary
  arrays for better auto-vectorization. Deferred to post-hardware-validation.

- **PFB upgrade path:** If channel occupancy increases beyond 8
  simultaneous channels (dense urban deployment), the per-channel FIR
  cascade becomes less efficient than a PFB. The architecture is designed
  so the PFB can replace the per-channel cascaded decimator without
  changing the SfLane decode pipeline.

- **Scratch buffer sizing:** `CascadedDecimator::intermediate` is sized
  to `max_input_chunk / 2` (4096 samples for 8192-sample SoapySource
  chunks = 32 KB per slot). Allocated once in `init()`, reused per call.
