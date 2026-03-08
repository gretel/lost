# Fix Watchdog Logging Implementation Plan

> **Status: COMPLETED** — Hardware-validated 2026-03-08 (commit `a7fc199`).

**Goal:** Eliminate spurious `watchdog: no progress` log spam during normal inter-frame silence in `lora_trx` MIMO mode.

**Architecture:** Two root causes. First: `watchdog_timeout`, `watchdog_max_warnings`, `watchdog_min_stall_count`, and `poolName` are declared as `Annotated<>` fields on `SchedulerBase` but are absent from `GR_MAKE_REFLECTABLE` — making them invisible to the settings reflection system. Second: `timeout_inactivity_count = 1U` (set for scheduler sleep latency) is reused as the watchdog log threshold, causing a log line every 1-second watchdog tick between LoRa frames. The fix adds the missing fields to `GR_MAKE_REFLECTABLE` and sets `watchdog_min_stall_count = 10U` in `lora_trx.cpp` to decouple the log threshold from the sleep threshold.

**Tech Stack:** C++23, GR4 `Annotated<>` / `GR_MAKE_REFLECTABLE`, CMake, ctest, `lora_trx` TOML config.

---

### Context: Key facts

- Working directory: `/Users/tom/src/uhd/gr4-lora/`
- Submodule: `gnuradio4/` — changes committed separately first, then parent repo updated
- Default shell is **elvish** — use `mcp_bash` with `/opt/homebrew/bin/bash -c '...'` for `$(...)` subshell syntax
- Build: `cmake --build build --target lora_trx -- -j4`
- Test: `ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 60`
- Hardware test: `./build/apps/lora_trx --config apps/config.toml` (no `--rx-channels` — config is TOML-only)
- Binary SHA check: `strings build/apps/lora_trx | grep -E "^[0-9a-f]{7}$" | tail -1`

### Context: Root cause

`Scheduler.hpp` line 189 (the `GR_MAKE_REFLECTABLE` line in `SchedulerBase`) is missing four fields:

```cpp
// CURRENT (broken):
GR_MAKE_REFLECTABLE(SchedulerBase, timeout_ms, timeout_inactivity_count,
    process_stream_to_message_ratio, max_work_items, sched_settings);

// MISSING fields declared at lines 180–185:
//   watchdog_timeout        (line 180)
//   watchdog_max_warnings   (line 182)
//   watchdog_min_stall_count (line 183)
//   poolName                (line 184)
```

The watchdog logging condition in `runWatchDog()` (line 739):
```cpp
if (nWarnings >= timeOut_count) { // timeOut_count = watchdogLogThreshold from start()
```

In `start()` (line 587):
```cpp
const std::size_t watchdogLogThreshold = watchdog_min_stall_count.value > 0U
    ? watchdog_min_stall_count.value
    : timeout_inactivity_count.value;
```

With `timeout_inactivity_count = 1U` and `watchdog_min_stall_count` not reliably
propagated (possibly reset to 0 by settings system internals), `watchdogLogThreshold`
resolves to `1` → logs every stall second.

---

### Task 1: Remove the temporary diagnostic `std::println` from `start()`

Added during debugging — must be removed before committing.

**Files:**
- Modify: `gnuradio4/core/include/gnuradio-4.0/Scheduler.hpp` (around line 588)

**Step 1: Remove the diagnostic line**

Find and remove this line in `Scheduler.hpp`:
```cpp
std::println(stderr, "[Scheduler] watchdog: timeout={}ms threshold={} (min_stall={} inactivity={})", watchdog_timeout.value, watchdogLogThreshold, watchdog_min_stall_count.value, timeout_inactivity_count.value);
```

**Step 2: Verify it's gone**
```bash
grep -n "Scheduler.*watchdog.*timeout=.*threshold=" gnuradio4/core/include/gnuradio-4.0/Scheduler.hpp
```
Expected: no output.

---

### Task 2: Add missing fields to `GR_MAKE_REFLECTABLE` in `SchedulerBase`

**Files:**
- Modify: `gnuradio4/core/include/gnuradio-4.0/Scheduler.hpp` line 189

**Step 1: Update the macro**

Replace:
```cpp
    GR_MAKE_REFLECTABLE(SchedulerBase, timeout_ms, timeout_inactivity_count, process_stream_to_message_ratio, max_work_items, sched_settings);
```

With:
```cpp
    GR_MAKE_REFLECTABLE(SchedulerBase, timeout_ms, watchdog_timeout, timeout_inactivity_count, watchdog_max_warnings, watchdog_min_stall_count, poolName, process_stream_to_message_ratio, max_work_items, sched_settings);
```

**Step 2: Build `lora_trx` to verify it compiles**
```bash
cmake --build build --target lora_trx -- -j4
```
Expected: exit 0, no errors.

**Step 3: Run full test suite**
```bash
ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 60
```
Expected: 12/12 pass.

**Step 4: Run pre-commit clang-format in submodule**
```bash
cd gnuradio4 && pre-commit run --files core/include/gnuradio-4.0/Scheduler.hpp
```
Expected: Passed (or auto-fixed — re-stage if modified).

**Step 5: Commit submodule**
```bash
cd gnuradio4
git add core/include/gnuradio-4.0/Scheduler.hpp
git commit -m "Add missing fields to GR_MAKE_REFLECTABLE in SchedulerBase

watchdog_timeout, watchdog_max_warnings, watchdog_min_stall_count, and
poolName were declared as Annotated<> settings fields but absent from
GR_MAKE_REFLECTABLE, making them invisible to the settings reflection
system. Direct member assignment still worked, but the fields could not
be set via property_map or constructor init-map, and their values were
not included in settings serialisation/deserialisation."
```

---

### Task 3: Verify `watchdog_min_stall_count = 10U` is set correctly in `lora_trx.cpp`

**Files:**
- Read: `apps/lora_trx.cpp` lines 1573–1577

**Step 1: Confirm the assignment is present**
```bash
grep -n "watchdog_min_stall_count" apps/lora_trx.cpp
```
Expected output:
```
1575:    rx_sched.watchdog_min_stall_count  = 10U; // suppress watchdog log for ≤10s gaps
```

If absent, add it after `timeout_inactivity_count`:
```cpp
rx_sched.timeout_inactivity_count  = 1U;   // sleep after 1 idle cycle
rx_sched.watchdog_min_stall_count  = 10U;  // suppress watchdog log for ≤10s gaps (normal inter-frame silence at SF8)
rx_sched.watchdog_max_warnings     = 30U;  // 30s of continuous stall → ERROR
```

**Step 2: Confirm the value is correct**

At SF8/BW62.5k, inter-frame gaps are ~4 s. Watchdog interval is 1000 ms.
`watchdog_min_stall_count = 10` → 10 s threshold → well above any normal gap.
`watchdog_max_warnings = 30` → ERROR after 30 s of continuous stall.

---

### Task 4: Reconfigure, rebuild, verify SHA, run tests

**Step 1: Reconfigure with current HEAD SHA**
```bash
/opt/homebrew/bin/bash -c 'cmake -S . -B build \
    -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake \
    -DGIT_REV=$(git rev-parse --short HEAD) \
    -DCMAKE_BUILD_TYPE=RelWithAssert -DWARNINGS_AS_ERRORS=OFF \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_OpenSSL=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_ZLIB=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_Brotli=ON'
```

**Step 2: Build `lora_trx`**
```bash
cmake --build build --target lora_trx -- -j4
```

**Step 3: Verify embedded SHA**
```bash
git log --oneline -1
strings build/apps/lora_trx | grep -E "^[0-9a-f]{7}$" | tail -1
```
Both must match.

**Step 4: Run full test suite**
```bash
ctest --test-dir build -R "qa_lora|bm_lora" --output-on-failure --timeout 60
```
Expected: 12/12 pass.

---

### Task 5: Commit parent repo (submodule pointer + lora_trx.cpp)

**Step 1: Check what's changed**
```bash
git status
git diff HEAD -- apps/lora_trx.cpp gnuradio4
```

**Step 2: Stage and commit**
```bash
git add gnuradio4 apps/lora_trx.cpp
git commit -m "Fix watchdog log spam: add missing fields to GR_MAKE_REFLECTABLE

watchdog_min_stall_count was set to 10 but fell back to
timeout_inactivity_count (= 1) because GR_MAKE_REFLECTABLE omitted
the watchdog fields from SchedulerBase, leaving them opaque to the
settings system. Adding them to the macro makes the direct member
assignment reliable.

Result: watchdog silent during normal inter-frame gaps (≤10 s at SF8),
logs after 10 s of genuine stall, ERROR after 30 s."
```

---

### Task 6: Hardware validation

**Step 1: Run lora_trx**
```bash
./build/apps/lora_trx --config apps/config.toml 2>&1 | tee tmp/watchdog_test.log
```

**Step 2: Monitor for watchdog messages**

Expected: NO `watchdog: no progress` lines during normal operation (between frames).
Frames should decode normally. Watchdog silence until 10+ seconds of genuine pipeline stall.

**Step 3: Check log for false positives**
```bash
grep "watchdog" tmp/watchdog_test.log
```
Expected: empty (or only after a genuine 10 s stall).

---

### Task 7: Update the `gr4-dev` skill

**File:** `/Users/tom/src/uhd/.opencode/skills/gr4-dev/SKILL.md`

**Changes needed:**

1. **Remove all `--rx-channels` references** — this flag no longer exists. Multi-channel RX is configured via TOML `rx_channel = [1, 2]` in `[radio_*]`.

2. **Update hardware test command** — replace any `lora_trx --rx-channels 0,1` with:
   ```bash
   ./build/apps/lora_trx --config apps/config.toml
   ```

3. **Add GR4 pitfall: missing fields in `GR_MAKE_REFLECTABLE`** — add to the "GR4 block development: critical rules" section:

   ```
   ### SchedulerBase: GR_MAKE_REFLECTABLE must include all Annotated<> fields

   Annotated<> fields on SchedulerBase (watchdog_timeout, watchdog_max_warnings,
   watchdog_min_stall_count, poolName) must appear in GR_MAKE_REFLECTABLE or they
   are invisible to the settings system. Direct member assignment still works (it
   sets .value directly via Annotated::operator=), but the values may not survive
   settings flush cycles, cannot be set via property_map, and are excluded from
   serialisation. Symptom: assigned value appears to be silently ignored at runtime.
   Fix: add the field name to GR_MAKE_REFLECTABLE.
   ```

4. **Add lora_trx scheduler settings reference** — document the key settings and their purpose:
    ```
    rx_sched.timeout_inactivity_count = 1U;   // sleep after 1 idle cycle (low latency)
    rx_sched.watchdog_min_stall_count = 10U;  // log threshold: 10 s (above any inter-frame gap)
    rx_sched.watchdog_max_warnings    = 30U;  // ERROR after 30 s of continuous stall
    ```

---

### Post-mortem: deviations from original plan

The original plan identified only the **RX scheduler** as the watchdog spam source and
prescribed adding `watchdog_min_stall_count=10U` to `rx_sched`. In practice:

1. **TX scheduler was the actual source.** `build_tx_graph()` had `timeout_inactivity_count=1U`
   with no watchdog settings. Its `TxQueueSource` blocks idle waiting for UDP TX requests,
   so the watchdog saw perpetual "no progress." Fixed by adding `watchdog_min_stall_count=10U`
   and `watchdog_max_warnings=0U` to the TX scheduler.

2. **`watchdog_max_warnings=0` did not suppress logging.** The `runWatchDog()` method logged
   every cycle once `nWarnings >= timeOut_count`, regardless of `max_warnings`. Even with
   `min_stall_count=10`, it started logging at cycle 10 and continued forever. Required a
   framework-level patch (`9f1dfa0`) to gate logging on `max_warnings > 0`.

3. **Three hardware iterations required.** First test showed TX scheduler as source (not RX).
   Second test showed threshold worked but logging continued indefinitely. Third test (final)
   confirmed zero watchdog output.

Commits: `92c5def` (GR_MAKE_REFLECTABLE + RX settings), `a7fc199` (TX scheduler + framework patch).
Submodule commits: `98ca779` (GR_MAKE_REFLECTABLE), `9f1dfa0` (log suppression patch).
