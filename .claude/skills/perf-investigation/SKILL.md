---
name: perf-investigation
description: >
  Runtime latency profiling for GR4 (gnuradio4) signal processing pipelines
  on macOS Apple Silicon. Use this skill whenever the user reports overflows,
  scheduler stalls, processBulk timing violations, dropped samples, or any
  "it works in loopback but fails on hardware" latency problem. Also trigger
  when the user mentions: Instruments, flamegraph, Time Profiler, perf budget,
  cycle budget, CPU hotspot, timing regression, -ftime-trace, overflow cascade,
  scheduler death, "where is the time going", "why is it slow", or asks to
  profile/benchmark any GR4 block. Trigger even for casual questions like
  "the wideband path overflows but narrowband doesn't" or "processBulk takes
  too long". This skill covers the full investigation workflow from symptom
  to root cause, including tooling, instrumentation, and known GR4 anti-patterns.
---

# GR4 Runtime Latency Profiling

## When to use this skill

You're looking at a **real-time signal processing pipeline** where blocks
must complete their `processBulk` within a strict timing budget dictated by
the sample rate and chunk size:

```
budget_ms = chunk_size / sample_rate * 1000
```

At 16 MS/s with 8192-sample chunks, that's **0.512 ms per cycle**. At 250
kS/s it's 32.8 ms. Exceeding the budget causes USB FIFO overflows
(SoapyBlock), sample discontinuities, and eventually silent scheduler death.

The symptom is almost never "it's slow" — it's **overflows, CRC failures on
hardware but not loopback, or the scheduler silently stopping**. The root
cause is always a code path in `processBulk` that takes too long.

## Investigation workflow

Follow these phases in order. Each phase has a specific question it answers.

### Phase 0: Characterize the symptom + collect telemetry

Before touching any profiler, establish the baseline facts AND start
collecting overflow telemetry in parallel:

1. **Sample rate and chunk size** → compute the cycle budget
2. **Which block is suspected** (or "I don't know")
3. **Is it reproducible** in loopback or only on hardware?
4. **Overflow count pattern**: startup-only vs steady-state vs bursty

**Start telemetry immediately** (runs in parallel with build):
```bash
# For lora_scan (streaming mode):
./build/apps/lora_scan --config apps/config.toml --cbor 2>tmp/scan_stderr.log \
  | timeout 30 .venv/bin/python3 scripts/lora_perf.py --diag 2>&1 \
  | tee tmp/perf_output.log

# For lora_trx:
./build/apps/lora_trx --config apps/config.toml 2>tmp/trx_stderr.log &
timeout 30 .venv/bin/python3 scripts/trx_perf.py 2>&1 | tee tmp/perf_output.log
```

The stderr log captures UHD init, overflow `O` markers, and block startup
messages. The perf script captures sweep timing, overflow counts, hot
channels, and detection rates.

**The passthrough test** isolates CPU vs transport if telemetry is ambiguous:

```cpp
// Add to the suspected block's processBulk:
if (min_ratio >= 9999.f) {  // or a dedicated bypass flag
    std::ignore = input.consume(in_span.size());
    output.publish(0UZ);
    return gr::work::Status::OK;
}
```

If overflows vanish → the block's per-sample work exceeds the budget.
If overflows persist → the problem is upstream (USB transport, scheduler
overhead, or a different block).

### Phase 1: Identify the hotspot (Instruments Time Profiler)

**Goal:** Find which function(s) inside `processBulk` consume the most time.

**Step 1: Build profiling binary** (separate build dir, uses all gr4-dev flags):

```bash
/opt/homebrew/bin/bash -c 'cmake -B build-prof \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer" \
  -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake \
  -DGIT_REV=$(git rev-parse --short HEAD) \
  -DWARNINGS_AS_ERRORS=OFF \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_OpenSSL=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_ZLIB=ON \
  -DCMAKE_DISABLE_FIND_PACKAGE_Brotli=ON \
  -G Ninja'
cmake --build build-prof --target lora_scan -j$(sysctl -n hw.ncpu)
```

**Step 2: Record trace** (15s is enough for steady-state profiling):

```bash
xcrun xctrace record --template "Time Profiler" \
  --time-limit 15s \
  --output "profile-lora_scan-$(date +%Y%m%d-%H%M%S).trace" \
  --launch -- ./build-prof/apps/lora_scan --config apps/config.toml --cbor
```

**Step 3: Export and analyze** — the XML export is 28K+ lines, too large
for direct reading. Export to file, then use a subagent to extract hotspots:

```bash
xcrun xctrace export --input profile.trace \
  --xpath '//trace-toc/run/data/table[@schema="time-profile"]' > tmp/profile.xml
```

Dispatch an explore/general subagent to grep the XML for:
- The `binary` element with `name="lora_scan"` to get its ref ID
- All `frame` elements referencing that binary → count by function name
- Thread distribution (main thread vs worker threads)
- The result is a ranked table of self-time and inclusive-time per function

**Step 4: Open in Instruments.app** for the visual flamegraph:

```bash
open profile-lora_scan-*.trace
```

The `scripts/profile_block.sh` helper automates steps 1-3 but uses minimal
cmake flags. Prefer the manual approach above for gr4-lora (needs GIT_REV,
WARNINGS_AS_ERRORS, etc.).

**What to look for:**

- Any function taking >10% of a `processBulk` call tree
- `operator%` or `operator/` in inner loops (modulo on non-power-of-2 is
  expensive — the half-band FIR `% 23` bug was found this way)
- `malloc` / `operator new` inside processBulk (heap allocation in the hot
  path — the soft decode bug: 29,400 heap ops/sec)
- `std::vector::resize` or `push_back` (implicit reallocation)
- `fprintf`, `std::cout`, any I/O in processBulk (the debug logging
  overflow cascade: each `fprintf` blocks long enough to cause the next overflow)
- `std::cos` / `std::sin` per-sample (NCO should use recurrence, not trig)
- `computeFilter<double>` or any IIR at 16 MS/s in double precision
  (11.6% of scheduler thread — use float or skip in non-critical states)
- `_platform_memmove` from FIR cascade ping-pong buffers (5.4% — avoidable)

### Phase 2: Measure per-call timing (instrumentation) — optional

**Goal:** Get cycle-accurate timing of each `processBulk` call to find
outliers and jitter.

**Skip this phase** when the Time Profiler gives clear results (>70% of
scheduler thread attributable to specific functions). Use ProcessBulkTimer
only when: (a) the hotspot is unclear, (b) you need to distinguish
steady-state from bursty violations, or (c) you want automated
budget-checking in CI.

Add the `ProcessBulkTimer` RAII guard from `scripts/process_bulk_timer.hpp`
to the suspected block:

```cpp
#include "process_bulk_timer.hpp"

gr::work::Status processBulk(/* ... */) {
    perf::ProcessBulkTimer timer("WidebandDecoder", in_span.size());
    // ... existing code ...
}
```

This measures wall-clock duration per call and prints a summary every 1000
calls to stderr:

```
[PERF] WidebandDecoder: n=1000 avg=0.31ms p50=0.28ms p99=0.52ms max=1.23ms budget=0.51ms VIOLATIONS=3
```

The `VIOLATIONS` count shows how many calls exceeded the budget. The p99 and
max values identify jitter sources (GC pauses, page faults, USB stalls).

**Important:** This adds ~1 microsecond overhead per call from `clock_gettime`.
Remove before production/hardware testing — the measurement itself can push
marginal blocks over budget at 16 MS/s.

### Phase 3: Trace compilation overhead (-ftime-trace)

**Goal:** Find template instantiation bloat that inflates binary size and
icache pressure.

GR4 is heavily templated (`Block<Derived>`, `PortIn<T, Attr>`, etc.). Deep
template instantiation trees increase binary size and reduce icache locality,
which shows up as mysterious slowdowns that don't appear in function-level
profiling.

```bash
# Add -ftime-trace to the build
cmake -B build-trace -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-ftime-trace" \
  -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -G Ninja
cmake --build build-trace --target lora_trx 2>&1

# Find the trace files (one per TU)
find build-trace -name "*.json" -newer build-trace/CMakeCache.txt
```

Open in `chrome://tracing`, Perfetto, or Speedscope. Look for:
- Template instantiations taking >100ms (especially `Block::workInternal`,
  `invokeUserProvidedFunction`, `processBulk` dispatch)
- `GR_MAKE_REFLECTABLE` expansions (each reflected field generates
  significant template machinery)
- Duplicate instantiations across TUs (symptom: same template shows up in
  multiple .json files with similar cost)

### Phase 4: System-level tracing (Instruments System Trace)

**Goal:** Correlate block execution with OS scheduling, USB interrupts, and
thread contention.

Use when Time Profiler shows no single hotspot but overflows still occur —
the problem is likely thread scheduling or I/O contention.

```bash
xcrun xctrace record --template "System Trace" \
  --time-limit 10s --output systrace.trace \
  --launch -- ./build-prof/apps/lora_trx --config apps/config.toml
```

In Instruments, look at:
- **Thread States**: is the processBulk thread being preempted?
- **Virtual Memory**: page faults during processBulk (from `std::vector`
  growth or first-touch on pre-allocated buffers)
- **System Calls**: unexpected `write()` (logging), `mmap()` (allocation),
  `select()`/`poll()` (socket I/O in the wrong thread)

### Phase 5: Validate the fix (A/B testing)

After identifying and fixing the hotspot, validate with hardware A/B testing.
Use the existing `lora_test.py` harness:

```bash
# Before fix: record baseline
python3 scripts/lora_test.py decode --matrix dc_edge --serial /dev/cu.usbserial-0001

# After fix: same matrix, same conditions
python3 scripts/lora_test.py decode --matrix dc_edge --serial /dev/cu.usbserial-0001
```

Compare: overflow count, CRC rate, SNR, sweep duration. The fix is validated
when the metric that was failing improves without regressing other metrics.

For automated budget regression testing, use `scripts/budget_check.py`:

```bash
python3 .claude/skills/perf-investigation/scripts/budget_check.py \
  --binary ./build/apps/lora_trx \
  --config apps/config.toml \
  --duration 10 \
  --budget-ms 0.512
```

## Known GR4 latency anti-patterns

These are patterns discovered during gr4-lora development that cause latency
violations. When investigating a new issue, check for these first — they
account for the majority of cases.

| Anti-pattern | Symptom | Example from codebase |
|---|---|---|
| Modulo on non-power-of-2 in inner loop | **39% of scheduler thread** | `HalfBandStage::processWithMix`: `% 23` × 3/sample × 16 MS/s = 48M div/s. **Fixed:** `processWithMixBatch()` uses linear-buffer indexing. |
| Double-precision IIR at wideband rate | **12% of scheduler thread** | `computeFilter<double>` DC blocker at 16 MS/s. **Fixed:** removed from scan path; L1 FFT NF interpolation + DC channel exclusion sufficient. |
| Heap allocation in processBulk | Bursty overflows, p99 spikes | Soft decode: 5 `vector::resize` per symbol = 29,400 heap ops/s |
| fprintf/cout in hot path | Self-sustaining overflow cascade | Debug overflow logging: 14,000 lines in 26s |
| FIR cascade buffer copies | 5% of scheduler thread | `_platform_memmove` from ping-pong scratch + `out.insert()` in CascadedDecimator |
| Ring buffer push per-sample | Steady overhead | `modulo` loop at 16M samples/s vs `std::copy_n` bulk |
| `consume(0)` in processBulk | Scheduler sleeps (1 call/s instead of 30k/s) | GR4 `singleThreadedBlocking` inactivity timeout |
| `gr::exception` from processBulk | Silent scheduler death | SoapyBlock fragment/overflow limits |
| Large FFT in SfLane SYNC/OUTPUT | CPU spike on detection | SF12: 4096-point FFT per symbol in decode state |
| channelize() blocking | Multi-channel probe stall | 8 channels × 3 BWs × 5ms = 120ms block |

## Telemetry integration

The codebase has a telemetry system (`algorithm/Telemetry.hpp`) that emits
`property_map` events as CBOR over UDP. Use it to add non-invasive timing
to blocks without recompiling:

```cpp
// In your block's processBulk, after the hot section:
if (_telemetry) {
    gr::property_map evt;
    evt["type"] = std::string("block_timing");
    evt["block"] = std::string("WidebandDecoder");
    evt["duration_us"] = static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start).count());
    evt["samples"] = static_cast<int64_t>(in_span.size());
    _telemetry(evt);
}
```

Monitor with `trx_perf.py` or a custom subscriber.

## Quick reference: cycle budgets

| Sample rate | Chunk size | Budget | Use case |
|---|---|---|---|
| 250 kS/s | 8192 | 32.8 ms | Narrowband decode (comfortable) |
| 1 MS/s | 8192 | 8.2 ms | Multi-BW decode |
| 8 MS/s | 8192 | 1.024 ms | Reduced wideband |
| 16 MS/s | 8192 | 0.512 ms | Full wideband (tight) |
| 16 MS/s | 65536 | 4.096 ms | Large chunks (makes USB worse) |

## Tool reference

| Tool | When to use | Overhead |
|---|---|---|
| Instruments Time Profiler | First look — where is CPU time spent | ~5% (sampling) |
| Instruments System Trace | Thread scheduling, I/O, page faults | ~10-15% |
| `-ftime-trace` | Template bloat, compile-time cost | Zero runtime |
| `ProcessBulkTimer` (this skill) | Per-call budget violation counting | ~1 us/call |
| `budget_check.py` (this skill) | Automated regression gate | Uses ProcessBulkTimer |
| `sample` (macOS) | Quick stack snapshot (no Instruments) | Minimal |
| `lora_perf.py --diag` | Scan overflow + L1 energy diagnostics | Zero (UDP subscriber) |
| `trx_perf.py` | Decode frame monitoring | Zero (UDP subscriber) |
| Existing benchmarks (`bm_*`) | Algorithm-level microbenchmarks | N/A (standalone) |

## Files in this skill

- `scripts/profile_block.sh` — Instruments automation wrapper
- `scripts/process_bulk_timer.hpp` — RAII per-call timer for processBulk
- `scripts/budget_check.py` — Automated budget violation detector
- `references/antipatterns.md` — Full catalog of GR4 latency anti-patterns
