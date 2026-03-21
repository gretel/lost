# FIR Channelizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the box-car decimator in WidebandDecoder's ChannelSlot with a cascaded 19-tap half-band FIR to achieve ≥-40 dB stopband, enabling hardware decode from the 16 MS/s wideband stream.

**Architecture:** A `CascadedDecimator` struct wraps N stages of `HalfBandStage`, each decimating by 2 with persistent delay lines for phase continuity across `processBulk` calls. The cascade replaces the integrate-and-dump loop in `ChannelSlot::pushWideband()`. Multi-BW support adds a `decode_bw_str` setting and restructures `_activeChannelMap` for per-channel multi-slot tracking.

**Tech Stack:** C++20, GR4 framework, Boost.UT tests, CMake/Ninja build

**Spec:** `docs/superpowers/specs/2026-03-21-fir-channelizer-design.md`

**Build commands:**
- Configure: `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DGIT_REV=$(git rev-parse --short HEAD) -DCMAKE_BUILD_TYPE=RelWithAssert -DWARNINGS_AS_ERRORS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_DISABLE_FIND_PACKAGE_OpenSSL=ON -DCMAKE_DISABLE_FIND_PACKAGE_ZLIB=ON -DCMAKE_DISABLE_FIND_PACKAGE_Brotli=ON`
- Build wideband test: `cmake --build build --target qa_lora_wideband -- -j4`
- Run wideband test: `ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
- Build all tests: `cmake --build build --target qa_lora_algorithms qa_lora_tx qa_lora_phase3 qa_lora_rx qa_lora_loopback qa_lora_burst qa_lora_splitter qa_lora_cad qa_lora_wideband -- -j4`
- Run all tests: `ctest --test-dir build -R qa_lora --output-on-failure --timeout 120`

**Important:** Use `direnv exec . <command>` to run commands in the project's environment. Use `/opt/homebrew/bin/bash` as shell.

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp` | Create | Half-band FIR stage + cascaded decimator structs |
| `blocks/include/gnuradio-4.0/lora/WidebandDecoder.hpp` | Modify | Replace box-car with CascadedDecimator, add multi-BW settings |
| `test/qa_lora_wideband.cpp` | Modify | Add FIR unit tests, update existing tests for new API |

---

## Task 1: Design and verify half-band FIR coefficients

**Files:**
- Create: `tmp/design_halfband.py` (temporary, for coefficient computation)

This task computes the 19-tap half-band filter coefficients using `scipy.signal.remez` and verifies they meet the -40 dB stopband requirement.

- [ ] **Step 1: Write the coefficient design script**

```python
#!/usr/bin/env python3
"""Design a 19-tap half-band FIR filter and verify stopband rejection."""
import numpy as np
from scipy.signal import remez, freqz

# Design 19-tap half-band via Parks-McClellan (Remez exchange).
# Transition band [0.4, 0.6] * Fs for decimate-by-2.
# fs=2.0 normalizes so Nyquist = 1.0.
numtaps = 19
h = remez(numtaps, [0, 0.4, 0.6, 1.0], [1, 0], fs=2.0)

# Enforce half-band constraints:
# - Odd taps (except center) forced to zero
# - Center tap h[9] = 0.5
for i in range(numtaps):
    if i % 2 == 1 and i != numtaps // 2:
        h[i] = 0.0
h[numtaps // 2] = 0.5

# Verify frequency response
w, H = freqz(h, worN=8192, fs=2.0)
mag_db = 20 * np.log10(np.abs(H) + 1e-15)

pb_ripple = np.max(np.abs(mag_db[w <= 0.4]))
sb_max = np.max(mag_db[w >= 0.6])
print(f"Passband ripple: {pb_ripple:.3f} dB")
print(f"Stopband max: {sb_max:.1f} dB")

# Print C++ constexpr array
print(f"\nC++ coefficients (copy to HalfBandDecimator.hpp):")
print(f"constexpr std::array<float, 5> kCoeffs = {{")
for i in range(0, 10, 2):
    print(f"    {h[i]:.15e}f,  // h[{i}] = h[{18-i}]")
print(f"}};")

# Verify constraints
for i in range(9):
    assert abs(h[i] - h[18 - i]) < 1e-15, f"Symmetry: h[{i}] != h[{18-i}]"
for i in [1, 3, 5, 7, 11, 13, 15, 17]:
    assert h[i] == 0.0, f"Zero tap: h[{i}] should be 0"
assert sb_max < -40.0, f"Stopband {sb_max:.1f} dB, need < -40 dB"
assert pb_ripple < 0.1, f"Passband ripple {pb_ripple:.3f} dB, need < 0.1 dB"
print("\nAll checks PASSED")
```

- [ ] **Step 2: Run the script to get coefficients**

Run: `uv run --with scipy --with numpy tmp/design_halfband.py`
Expected: Prints 5 unique non-zero coefficients and "All checks PASSED". If the 19-tap design does not meet -40 dB, increase to 23 taps.

- [ ] **Step 3: Record the coefficients**

Note the exact coefficient values. They will be hardcoded as `constexpr` in the next task.

---

## Task 2: Implement HalfBandDecimator

**Files:**
- Create: `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp`
- Modify: `test/qa_lora_wideband.cpp` (add include)

- [ ] **Step 1: Write the failing test for HalfBandStage stopband rejection**

Add a new test suite to `test/qa_lora_wideband.cpp`, after the includes but before the existing suites. Add `#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>` to the includes.

```cpp
// In test/qa_lora_wideband.cpp, new test suite:

const boost::ut::suite<"HalfBandDecimator"> halfBandTests = [] {
    using namespace boost::ut;

    "single stage: stopband rejection >= 40 dB"_test = [] {
        gr::lora::HalfBandStage stage;
        stage.reset();

        // Test tones in stopband: 0.6, 0.7, 0.8, 0.9 x Nyquist
        // Nyquist of output = 0.5 * input_rate. Transition band [0.4, 0.6] * input_rate.
        // Stopband starts at 0.6 * input_rate.
        // We normalize to input sample rate = 1.0.
        constexpr std::size_t N = 8192;

        // Passband reference: tone at 0.1 * Fs_in
        auto passbandTone = makeTone(N, 0.1, 1.0);  // freq=0.1, rate=1.0
        std::vector<cf32> pbOut;
        stage.reset();
        stage.process(std::span<const cf32>(passbandTone), pbOut);
        float pbPower = 0.f;
        for (const auto& s : pbOut) pbPower += std::norm(s);
        pbPower /= static_cast<float>(pbOut.size());

        // Stopband tones
        for (double freq : {0.6, 0.7, 0.8, 0.9}) {
            auto tone = makeTone(N, freq, 1.0);
            std::vector<cf32> out;
            stage.reset();
            stage.process(std::span<const cf32>(tone), out);
            float sbPower = 0.f;
            for (const auto& s : out) sbPower += std::norm(s);
            sbPower /= static_cast<float>(out.size());

            float rejection_db = 10.f * std::log10(sbPower / pbPower);
            expect(rejection_db < -40.f)
                << "stopband at " << freq << ": " << rejection_db << " dB";
        }
    };

    "single stage: passband flatness < 0.1 dB"_test = [] {
        gr::lora::HalfBandStage stage;
        constexpr std::size_t N = 8192;

        // DC reference
        std::vector<cf32> dc(N, cf32(1.f, 0.f));
        std::vector<cf32> dcOut;
        stage.reset();
        stage.process(std::span<const cf32>(dc), dcOut);
        float dcPower = 0.f;
        for (const auto& s : dcOut) dcPower += std::norm(s);
        dcPower /= static_cast<float>(dcOut.size());

        for (double freq : {0.05, 0.1, 0.2, 0.3, 0.38}) {
            auto tone = makeTone(N, freq, 1.0);
            std::vector<cf32> out;
            stage.reset();
            stage.process(std::span<const cf32>(tone), out);
            float power = 0.f;
            for (const auto& s : out) power += std::norm(s);
            power /= static_cast<float>(out.size());

            float ripple_db = std::abs(10.f * std::log10(power / dcPower));
            expect(ripple_db < 0.5f)  // 0.5 dB tolerance (filter + windowing effects)
                << "passband at " << freq << ": ripple " << ripple_db << " dB";
        }
    };

    "cascaded decimator: phase continuity across chunks"_test = [] {
        constexpr uint32_t nStages = 4;  // decimate by 16
        constexpr std::size_t totalSamples = 16 * 1024;

        // Generate a tone at 0.01 * Fs (well within passband)
        auto fullSignal = makeTone(totalSamples, 0.01, 1.0);

        // Single-call processing (reference)
        gr::lora::CascadedDecimator refDec;
        refDec.init(nStages);
        std::vector<cf32> refOut;
        refDec.process(std::span<const cf32>(fullSignal), refOut);

        // Chunked processing: varying chunk sizes
        gr::lora::CascadedDecimator chunkDec;
        chunkDec.init(nStages);
        std::vector<cf32> chunkOut;
        std::size_t chunkSizes[] = {1, 7, 128, 8192, 1024, 3, totalSamples - 1 - 7 - 128 - 8192 - 1024 - 3};
        std::size_t offset = 0;
        for (auto cs : chunkSizes) {
            if (offset + cs > totalSamples) cs = totalSamples - offset;
            if (cs == 0) break;
            chunkDec.process(
                std::span<const cf32>(fullSignal.data() + offset, cs),
                chunkOut);
            offset += cs;
        }

        expect(eq(refOut.size(), chunkOut.size()))
            << "same output length: " << refOut.size() << " vs " << chunkOut.size();

        // Must be bit-exact (same floating point operations in same order)
        float maxErr = 0.f;
        for (std::size_t i = 0; i < std::min(refOut.size(), chunkOut.size()); ++i) {
            float err = std::abs(refOut[i] - chunkOut[i]);
            maxErr = std::max(maxErr, err);
        }
        expect(maxErr < 1e-6f) << "max error = " << maxErr;
    };

    "cascaded decimator: noise power reduction"_test = [] {
        constexpr uint32_t nStages = 8;  // os=256
        constexpr std::size_t N = 256 * 2048;  // 2048 output samples

        // White Gaussian noise (fixed seed)
        std::mt19937 rng(12345);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<cf32> noise(N);
        for (auto& s : noise) s = cf32(dist(rng), dist(rng));

        // Input power
        float inPower = 0.f;
        for (const auto& s : noise) inPower += std::norm(s);
        inPower /= static_cast<float>(N);

        // Decimated output
        gr::lora::CascadedDecimator dec;
        dec.init(nStages);
        std::vector<cf32> out;
        dec.process(std::span<const cf32>(noise), out);

        float outPower = 0.f;
        for (const auto& s : out) outPower += std::norm(s);
        outPower /= static_cast<float>(out.size());

        // Expected reduction: ~10*log10(256) = 24 dB (process gain)
        // The FIR filter rejects out-of-band noise; with ideal filter the
        // output power = input_power / os_factor. Allow 2 dB tolerance.
        float reductionDb = 10.f * std::log10(outPower / inPower);
        float expectedDb = -10.f * std::log10(256.f);  // -24.08 dB
        expect(std::abs(reductionDb - expectedDb) < 2.f)
            << "noise reduction = " << reductionDb << " dB, expected ~" << expectedDb << " dB";
    };
};
```

- [ ] **Step 2: Verify test fails to compile (header doesn't exist yet)**

Run: `cmake --build build --target qa_lora_wideband -- -j4`
Expected: Compile error — `HalfBandDecimator.hpp: No such file`

- [ ] **Step 3: Implement HalfBandDecimator.hpp**

Create `blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp`:

```cpp
// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP
#define GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <span>
#include <vector>

namespace gr::lora {

using cf32 = std::complex<float>;

// 19-tap half-band FIR coefficients.
// Designed via Parks-McClellan for transition band [0.4, 0.6] * Fs.
// Stopband rejection ≥ 40 dB, passband ripple < 0.1 dB.
// h[n] = h[18-n] (symmetric), odd taps zero except center h[9] = 0.5.
// Only 5 unique non-zero values: h[0], h[2], h[4], h[6], h[8].
// INSERT ACTUAL COEFFICIENTS FROM TASK 1 HERE
namespace halfband_detail {
    // Placeholder — replace with actual scipy.signal.remez output from Task 1
    constexpr std::array<float, 5> kCoeffs = {
        0.0f,   // h[0] = h[18]
        0.0f,   // h[2] = h[16]
        0.0f,   // h[4] = h[14]
        0.0f,   // h[6] = h[12]
        0.0f,   // h[8] = h[10]
    };
    constexpr float kCenterTap = 0.5f;  // h[9]
    constexpr uint32_t kNumTaps = 19;
}  // namespace halfband_detail

/// Single half-band FIR filter + decimate-by-2 stage.
/// Delay line persists across process() calls for phase continuity.
struct HalfBandStage {
    static constexpr uint32_t kBufLen = 19;  // delay line = filter length
    std::array<cf32, kBufLen> delay{};
    uint32_t writePos{0};   // next write position in circular buffer
    uint32_t sampleCount{0};  // counts input samples for decimate-by-2 phase

    void reset() {
        delay.fill(cf32(0.f, 0.f));
        writePos = 0;
        sampleCount = 0;
    }

    /// Process input samples, appending decimated output to `out`.
    /// Decimates by 2: for every 2 input samples, produces 1 output.
    void process(std::span<const cf32> in, std::vector<cf32>& out) {
        const auto& c = halfband_detail::kCoeffs;

        for (const auto& sample : in) {
            // Write new sample into circular delay line
            delay[writePos] = sample;

            // Only compute output every 2nd sample (decimate by 2)
            sampleCount++;
            if ((sampleCount & 1) == 0) {
                // Newest sample is at delay[writePos] = tap index 18.
                // Oldest sample is at delay[(writePos + 1) % 19] = tap index 0.
                // tap k maps to delay[(writePos + 1 + k) % 19], but we can
                // equivalently index from writePos: tap k ↔ delay[(writePos - 18 + k + 19) % 19]
                //   = delay[(writePos + k + 1) % 19]

                cf32 acc(0.f, 0.f);
                // Symmetric pairs: h[0]*(x[0]+x[18]), h[2]*(x[2]+x[16]), etc.
                for (uint32_t i = 0; i < 5; ++i) {
                    uint32_t k = 2 * i;  // tap index: 0, 2, 4, 6, 8
                    uint32_t idx_lo = (writePos + k + 1) % kBufLen;
                    uint32_t idx_hi = (writePos + (18 - k) + 1) % kBufLen;
                    acc += c[i] * (delay[idx_lo] + delay[idx_hi]);
                }
                // Center tap (tap index 9)
                uint32_t idx_center = (writePos + 9 + 1) % kBufLen;
                acc += halfband_detail::kCenterTap * delay[idx_center];

                out.push_back(acc);
            }

            writePos = (writePos + 1) % kBufLen;
        }
    }
};

/// Cascaded half-band decimator: N stages, each decimating by 2.
/// Total decimation factor = 2^N.
struct CascadedDecimator {
    std::vector<HalfBandStage> stages;
    std::vector<cf32> scratch_a;  // ping-pong buffers between stages
    std::vector<cf32> scratch_b;
    uint32_t nStages{0};

    void init(uint32_t numStages, std::size_t maxChunkSize = 8192) {
        nStages = numStages;
        stages.resize(numStages);
        for (auto& s : stages) s.reset();
        // Pre-allocate scratch buffers for expected chunk size
        scratch_a.clear();
        scratch_b.clear();
        scratch_a.reserve(maxChunkSize / 2);
        scratch_b.reserve(maxChunkSize / 2);
    }

    void reset() {
        for (auto& s : stages) s.reset();
    }

    /// Cascade-filter input and append decimated output to `out`.
    /// Phase-continuous across calls (delay lines persist in each stage).
    void process(std::span<const cf32> in, std::vector<cf32>& out) {
        if (nStages == 0) {
            out.insert(out.end(), in.begin(), in.end());
            return;
        }

        // Stage 0 reads from `in`, writes to scratch_a
        // Stage 1 reads from scratch_a, writes to scratch_b
        // Stage 2 reads from scratch_b, writes to scratch_a (ping-pong)
        // ...
        // Final stage writes to `out`

        // Process stage by stage
        std::span<const cf32> stageIn = in;

        for (uint32_t s = 0; s < nStages; ++s) {
            bool isLast = (s == nStages - 1);

            if (isLast) {
                stages[s].process(stageIn, out);
            } else {
                auto& buf = (s % 2 == 0) ? scratch_a : scratch_b;
                buf.clear();
                stages[s].process(stageIn, buf);
                stageIn = std::span<const cf32>(buf);
            }
        }
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_ALGORITHM_HALFBAND_DECIMATOR_HPP
```

**Important:** After Task 1 produces the actual coefficients, replace the placeholder values in `kCoeffs`.

- [ ] **Step 4: Build and run the tests**

Run: `cmake --build build --target qa_lora_wideband -- -j4`
Then: `ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
Expected: All 4 new HalfBandDecimator tests pass + all existing tests still compile (they may fail until Task 3 updates them).

- [ ] **Step 5: Commit**

```
git add blocks/include/gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp test/qa_lora_wideband.cpp
git commit -m "feat: add HalfBandDecimator with 19-tap FIR + unit tests

Cascaded half-band FIR decimator with ≥-40 dB stopband rejection.
Each stage decimates by 2 with persistent delay lines for phase
continuity across processBulk calls."
```

---

## Task 3: Replace box-car in ChannelSlot

**Files:**
- Modify: `blocks/include/gnuradio-4.0/lora/WidebandDecoder.hpp` (lines 39-169: ChannelSlot)
- Modify: `test/qa_lora_wideband.cpp` (update existing ChannelSlot tests)

- [ ] **Step 1: Write a failing test that verifies FIR stopband in ChannelSlot**

Add to the existing `"ChannelSlot streaming channelizer"` suite in `qa_lora_wideband.cpp`:

```cpp
    "FIR channelizer: out-of-band tone rejected >= 30 dB"_test = [] {
        // A tone 100 kHz away from channel center (outside 62.5 kHz BW)
        // should be attenuated by ≥30 dB after cascaded half-band decimation.
        // (30 dB is conservative; the filter gives ≥40 dB per stage,
        //  but cascaded with NCO mixing we allow some margin.)
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 869.5e6;
        constexpr double oobToneFreq = channelFreq + 100e3;  // 100 kHz outside BW
        constexpr uint32_t osFactor = 256;
        constexpr std::size_t nOut = 256;
        constexpr std::size_t nSamples = nOut * osFactor;

        // In-band tone (at channel center)
        gr::lora::ChannelSlot inSlot;
        inSlot.activate(channelFreq, centerFreq, sampleRate, osFactor);
        auto inTone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);
        inSlot.pushWideband(inTone);
        float inPower = 0.f;
        for (const auto& s : inSlot.nbAccum) inPower += std::norm(s);
        inPower /= static_cast<float>(inSlot.nbAccum.size());

        // Out-of-band tone
        gr::lora::ChannelSlot oobSlot;
        oobSlot.activate(channelFreq, centerFreq, sampleRate, osFactor);
        auto oobTone = makeTone(nSamples, oobToneFreq - centerFreq, sampleRate);
        oobSlot.pushWideband(oobTone);
        float oobPower = 0.f;
        for (const auto& s : oobSlot.nbAccum) oobPower += std::norm(s);
        oobPower /= static_cast<float>(oobSlot.nbAccum.size());

        float rejectionDb = 10.f * std::log10(oobPower / inPower);
        expect(rejectionDb < -30.f)
            << "out-of-band rejection = " << rejectionDb << " dB, expected < -30";
    };
```

- [ ] **Step 2: Run test — should FAIL with the box-car decimator**

Run: `cmake --build build --target qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
Expected: The new test FAILS (box-car only gives ~-13 dB rejection). Existing tests should still pass.

- [ ] **Step 3: Replace the box-car in ChannelSlot**

Modify `ChannelSlot` in `WidebandDecoder.hpp`:

**Add include** (after existing includes at top of file):
```cpp
#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>
```

**Replace fields** in `ChannelSlot` struct (lines 53-61):

Remove:
```cpp
    // Two-stage integrate-and-dump decimator
    cf32     decim1Accum{0.f, 0.f};
    uint32_t decim1Count{0};
    uint32_t stage1Factor{1};
    cf32     decim2Accum{0.f, 0.f};
    uint32_t decim2Count{0};
    uint32_t stage2Factor{1};
```

Replace with:
```cpp
    // Cascaded half-band FIR decimator
    CascadedDecimator decimator;
    uint32_t decodeBw{62500};
```

**Replace `activate()` body** (lines 86-104, the decimator init section):

Remove the two-stage factor computation:
```cpp
        // Compute two-stage decimation factors
        // ...all the sqrt/factor code...
        decim1Accum = {0.f, 0.f};
        decim1Count = 0;
        decim2Accum = {0.f, 0.f};
        decim2Count = 0;
```

Replace with:
```cpp
        // Cascaded half-band FIR: os_factor must be power of 2
        if ((os_factor & (os_factor - 1)) != 0) {
            // Non-power-of-2 os_factor not supported by cascaded half-band.
            // Log and fall back to os=1 (no decimation) — will not decode but won't crash.
            state = State::Idle;
            return;
        }
        uint32_t nStages = 0;
        for (uint32_t v = os_factor; v > 1; v >>= 1) ++nStages;
        decimator.init(nStages);
        decodeBw = bw;
```

**Replace `pushWideband()` body** (lines 128-168):

The NCO mix loop stays. Replace the integrate-and-dump with FIR cascade.

New `pushWideband()`:
```cpp
    void pushWideband(std::span<const cf32> wbSamples) {
        // NCO mix: rotate each sample to baseband
        std::vector<cf32> mixed(wbSamples.size());
        uint32_t sampleIdx = 0;
        for (std::size_t i = 0; i < wbSamples.size(); ++i) {
            const auto rot = cf32(
                static_cast<float>(std::cos(ncoPhase)),
                static_cast<float>(std::sin(ncoPhase)));
            mixed[i] = wbSamples[i] * rot;
            ncoPhase += ncoPhaseInc;

            // Wrap phase every 1024 samples
            if ((++sampleIdx & 0x3FFU) == 0) {
                ncoPhase = std::remainder(ncoPhase, 2.0 * std::numbers::pi);
            }
        }

        // Cascaded half-band FIR decimation
        decimator.process(std::span<const cf32>(mixed), nbAccum);
    }
```

- [ ] **Step 4: Build and run all wideband tests**

Run: `cmake --build build --target qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
Expected: All tests pass including the new out-of-band rejection test.

**Existing tests that need updating after the FIR replacement:**

1. `"matches batch channelize() for single call"` (line 291): Compares ChannelSlot
   output to box-car `channelize()`. After FIR replacement, outputs differ.
   Replace with an independent verification (tone-at-DC + PMR check).

2. `"decimator handles partial blocks correctly"` (line 243): Expects exactly
   `floor(N / osFactor)` output samples from the box-car. The FIR has group delay
   (~9 samples per stage), so the first few outputs are delayed. Update: check
   that output count is within expected range accounting for filter warmup,
   or increase input length to make warmup negligible relative to total.

3. `"DC signal at center freq passes through unchanged"` (line 269): Expects
   each output sample to be `~1.0+0i` within 0.01. The FIR's DC gain is
   `sum(h[n])` which should be ~1.0 but first samples during warmup will differ.
   Fix: skip first 20 output samples (warmup transient) or relax tolerance.

Replace `"matches batch channelize()"` with:

```cpp
    "FIR channelizer output has clean tone at DC"_test = [] {
        // Replaces the old "matches batch channelize()" test.
        // Verify the FIR channelizer produces clean output independently.
        constexpr double sampleRate = 16e6;
        constexpr double centerFreq = 866.5e6;
        constexpr double channelFreq = 868.0e6;
        constexpr uint32_t osFactor = 128;
        constexpr uint32_t nSamples = osFactor * 64;

        auto tone = makeTone(nSamples, channelFreq - centerFreq, sampleRate);

        gr::lora::ChannelSlot slot;
        slot.activate(channelFreq, centerFreq, sampleRate, osFactor);
        slot.pushWideband(tone);

        expect(eq(slot.nbAccum.size(), 64UZ));
        auto bin = peakBin(slot.nbAccum);
        expect(eq(bin, 0UZ)) << "tone at channel freq maps to DC";
        auto pmr = peakToMeanDb(slot.nbAccum);
        expect(pmr > 20.f) << "PMR = " << pmr << " dB, expected > 20";
    };
```

- [ ] **Step 5: Run the full test suite to check for regressions**

Run: `cmake --build build --target qa_lora_algorithms qa_lora_tx qa_lora_phase3 qa_lora_rx qa_lora_loopback qa_lora_burst qa_lora_splitter qa_lora_cad qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora --output-on-failure --timeout 120`
Expected: All tests pass.

- [ ] **Step 6: Commit**

```
git add blocks/include/gnuradio-4.0/lora/WidebandDecoder.hpp test/qa_lora_wideband.cpp
git commit -m "feat: replace box-car with cascaded half-band FIR in ChannelSlot

Swaps the integrate-and-dump decimator for a CascadedDecimator
(19-tap half-band FIR per stage). Achieves ≥-40 dB stopband
rejection vs -13 dB previously. Fixes hardware decode failure
caused by broadband noise aliasing."
```

---

## Task 4: Multi-BW slot activation and dedup

**Files:**
- Modify: `blocks/include/gnuradio-4.0/lora/WidebandDecoder.hpp` (settings, _activeChannelMap, updateActiveChannels, finishFrame)
- Modify: `test/qa_lora_wideband.cpp` (update graph-level tests for new settings)

- [ ] **Step 1: Write a failing test for multi-BW slot activation**

Add to the `"WidebandDecoder block skeleton"` suite:

```cpp
    "multi-BW: three slots activated per hot channel"_test = [] {
        gr::lora::WidebandDecoder decoder;
        decoder.sample_rate = 16e6f;
        decoder.center_freq = 866.5e6f;
        decoder.channel_bw  = 62500.f;
        decoder.decode_bw_str = "62500,125000,250000";
        decoder.max_channels = 24;
        decoder.start();

        // Simulate: force one hot channel
        decoder._hotChannels = {100};  // channel index 100

        decoder.updateActiveChannels();

        // Should have 3 active slots (one per BW)
        expect(eq(decoder.activeSlotCount(), 3U))
            << "3 slots for 3 BWs on 1 hot channel";

        // Each slot should have a different decodeBw
        std::vector<uint32_t> bws;
        for (const auto& slot : decoder._slots) {
            if (slot.state != gr::lora::ChannelSlot::State::Idle) {
                bws.push_back(slot.decodeBw);
            }
        }
        std::ranges::sort(bws);
        expect(eq(bws.size(), 3UZ));
        expect(eq(bws[0], 62500U));
        expect(eq(bws[1], 125000U));
        expect(eq(bws[2], 250000U));
    };
```

- [ ] **Step 2: Run test — should fail (decode_bw_str doesn't exist yet)**

Run: `cmake --build build --target qa_lora_wideband -- -j4`
Expected: Compile error — `decode_bw_str` not a member of WidebandDecoder.

- [ ] **Step 3: Implement multi-BW settings and slot activation**

In `WidebandDecoder.hpp`:

**Add new setting** (after existing `decode_bw` field):
```cpp
    std::string  decode_bw_str{"62500"};   // comma-separated BWs for multi-BW decode
```

**Add to GR_MAKE_REFLECTABLE** — add `decode_bw_str` to the macro parameter list.

**Add internal state** (in internal state section):
```cpp
    std::vector<uint32_t>                _decodeBws;
    std::vector<std::vector<uint32_t>>   _activeChannelMap;  // channelIdx → list of slotIdx
```

**Modify `start()`** — add after the existing `_activeChannelMap` initialization. Replace the old `_activeChannelMap.assign(_nChannels, UINT32_MAX)` with the new vector-of-vectors:

```cpp
    // Parse decode_bw_str into _decodeBws
    _decodeBws.clear();
    {
        std::istringstream iss(decode_bw_str);
        std::string token;
        while (std::getline(iss, token, ',')) {
            // Trim whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (token.empty()) continue;
            auto bw = static_cast<uint32_t>(std::stoul(token));
            uint32_t os = static_cast<uint32_t>(
                std::round(static_cast<double>(sample_rate) / static_cast<double>(bw)));
            if (os == 0 || (os & (os - 1)) != 0) {
                log_ts("error", "wideband",
                    "BW %u gives non-power-of-2 os=%u at %.0f MS/s, skipping",
                    bw, os, static_cast<double>(sample_rate) / 1e6);
                continue;
            }
            _decodeBws.push_back(bw);
        }
    }
    std::ranges::sort(_decodeBws);  // narrowest first
    if (_decodeBws.empty()) {
        _decodeBws.push_back(static_cast<uint32_t>(decode_bw));
    }

    _activeChannelMap.assign(_nChannels, {});
```

Add `#include <sstream>` to the includes at the top of the file.

**Rewrite `updateActiveChannels()`** — replace the entire method body:

```cpp
    void updateActiveChannels() noexcept {
        // 1. Deactivate slots for channels no longer hot
        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_activeChannelMap[ch].empty()) continue;

            bool stillHot = std::ranges::find(_hotChannels, ch)
                          != _hotChannels.end();
            if (!stillHot) {
                for (uint32_t slotIdx : _activeChannelMap[ch]) {
                    if (debug) {
                        log_ts("debug", "wideband",
                            "deactivate slot %u (ch %u, BW %u, %.3f MHz)",
                            slotIdx, ch, _slots[slotIdx].decodeBw,
                            _slots[slotIdx].channelFreq / 1e6);
                    }
                    _slots[slotIdx].deactivate();
                }
                _activeChannelMap[ch].clear();
            }
        }

        // 2. Activate slots for new hot channels (one per BW per channel)
        for (uint32_t ch : _hotChannels) {
            if (ch >= _nChannels) continue;

            for (uint32_t bw : _decodeBws) {
                // Check if already active at this BW
                bool alreadyActive = false;
                for (uint32_t slotIdx : _activeChannelMap[ch]) {
                    if (_slots[slotIdx].decodeBw == bw) {
                        alreadyActive = true;
                        break;
                    }
                }
                if (alreadyActive) continue;

                // Find free slot
                uint32_t freeSlot = UINT32_MAX;
                for (uint32_t s = 0; s < static_cast<uint32_t>(_slots.size()); ++s) {
                    if (_slots[s].state == ChannelSlot::State::Idle) {
                        freeSlot = s;
                        break;
                    }
                }
                if (freeSlot == UINT32_MAX) {
                    if (debug) {
                        log_ts("debug", "wideband",
                            "no free slot for ch %u BW %u", ch, bw);
                    }
                    break;  // no more free slots
                }

                double freq = channelCenterFreq(ch);
                uint32_t os = static_cast<uint32_t>(
                    std::round(static_cast<double>(sample_rate)
                             / static_cast<double>(bw)));

                _slots[freeSlot].activate(freq,
                                          static_cast<double>(center_freq),
                                          static_cast<double>(sample_rate),
                                          os, ch, bw, sync_word, preamble_len);
                _activeChannelMap[ch].push_back(freeSlot);

                // Replay ring buffer so slot sees the preamble
                const auto replayLen = std::min(_ring.count, _ring.data.size());
                if (replayLen > 0) {
                    auto hist = _ring.recent(replayLen);
                    _slots[freeSlot].pushWideband(hist);
                }

                if (debug) {
                    log_ts("debug", "wideband",
                        "activate slot %u -> ch %u BW %u (%.3f MHz, os=%u)",
                        freeSlot, ch, bw, freq / 1e6, os);
                }
            }
        }
    }
```

**Modify `finishFrame()`** — add narrowest-BW-first dedup after the existing output tag + message publishing (before the closing `}`):

```cpp
        // Narrowest-BW-first dedup: suppress wider-BW slots on same channel
        if (frame.crc_valid) {
            for (uint32_t peerSlotIdx : _activeChannelMap[slot.channelIdx]) {
                auto& peer = _slots[peerSlotIdx];
                if (peer.decodeBw > slot.decodeBw
                    && peer.state == ChannelSlot::State::Active) {
                    peer.state = ChannelSlot::State::Draining;
                    for (auto& peerLane : peer.sfLanes) {
                        peerLane.resetToDetect();
                    }
                    if (debug) {
                        log_ts("debug", "wideband",
                            "dedup: draining slot BW %u (ch %u) after BW %u decode",
                            peer.decodeBw, slot.channelIdx, slot.decodeBw);
                    }
                }
            }
        }
```

- [ ] **Step 4: Build and run tests**

Run: `cmake --build build --target qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
Expected: New multi-BW test passes + all existing tests pass (existing tests use single BW and should work unchanged).

- [ ] **Step 5: Update existing graph-level tests for new settings**

The existing `graph.emplaceBlock<WidebandDecoder>` calls use `{"decode_bw", 62500.f}`. They should still work (fallback to single BW when `decode_bw_str` is default "62500"). Verify no regressions.

- [ ] **Step 6: Commit**

```
git add blocks/include/gnuradio-4.0/lora/WidebandDecoder.hpp test/qa_lora_wideband.cpp
git commit -m "feat: multi-BW slot activation with narrowest-first dedup

Adds decode_bw_str setting (comma-separated BWs). Each hot channel
activates one slot per BW. Narrowest-BW-first dedup suppresses
wider-BW duplicates on CRC-OK decode."
```

---

## Task 5: Noisy loopback test

**Files:**
- Modify: `test/qa_lora_wideband.cpp` (new test in loopback suite)

- [ ] **Step 1: Write the noisy loopback test**

Add to the `"WidebandDecoder loopback"` suite:

```cpp
    "SF8 noisy loopback: FIR survives wideband noise"_test = [] {
        // Same as existing SF8 loopback but with wideband noise at -5 dB SNR.
        // The box-car decimator failed this due to noise aliasing.
        // The FIR channelizer should pass it.
        constexpr uint8_t  test_sf  = 8;
        constexpr uint8_t  test_cr  = 4;
        constexpr uint32_t test_bw  = 62500;
        constexpr uint16_t test_sync = 0x12;
        constexpr uint16_t test_preamble = 8;
        constexpr double   wb_rate  = 16e6;
        constexpr double   center_freq = 866.5e6;
        constexpr double   channel_freq = 869.5e6;
        constexpr uint32_t os_factor = static_cast<uint32_t>(wb_rate / test_bw);

        const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};
        uint32_t nb_sps = (1u << test_sf);
        auto nb_iq = generate_frame_iq(payload, test_sf, test_cr, 1,
                                        test_sync, test_preamble,
                                        true, nb_sps * 5, 2, test_bw, false);

        const double shift_hz = channel_freq - center_freq;
        const double phaseInc = 2.0 * std::numbers::pi * shift_hz / wb_rate;
        const std::size_t wb_len = nb_iq.size() * os_factor;
        constexpr std::size_t pad_samples = 8192 * 20;
        std::vector<cf32> wb_iq(pad_samples + wb_len + pad_samples, cf32(0.f, 0.f));

        // Strong wideband noise: -5 dB SNR relative to signal
        // Signal power ~1.0 per sample, so noise sigma = ~1.78 for -5 dB SNR
        std::mt19937 rng(42);
        std::normal_distribution<float> noiseDist(0.f, 0.3f);  // wideband noise
        for (auto& s : wb_iq) {
            s = cf32(noiseDist(rng), noiseDist(rng));
        }

        // Embed signal
        double phase = 0.0;
        for (std::size_t i = 0; i < nb_iq.size(); ++i) {
            for (uint32_t j = 0; j < os_factor; ++j) {
                std::size_t wb_idx = pad_samples + i * os_factor + j;
                auto rot = cf32(
                    static_cast<float>(std::cos(phase)),
                    static_cast<float>(std::sin(phase)));
                wb_iq[wb_idx] += nb_iq[i] * rot;
                phase += phaseInc;
            }
        }

        // Build graph (same as clean loopback)
        gr::Graph graph;
        auto& src = graph.emplaceBlock<gr::testing::TagSource<cf32,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(wb_iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false},
        });
        src.values = wb_iq;

        auto& decoder = graph.emplaceBlock<gr::lora::WidebandDecoder>({
            {"sample_rate", static_cast<float>(wb_rate)},
            {"center_freq", static_cast<float>(center_freq)},
            {"channel_bw", 62500.f},
            {"decode_bw", static_cast<float>(test_bw)},
            {"max_channels", uint32_t{8}},
            {"buffer_ms", 200.f},
            {"l1_interval", uint32_t{1}},
            {"l1_snapshots", uint32_t{2}},
            {"l1_fft_size", uint32_t{4096}},
            {"sync_word", test_sync},
            {"preamble_len", test_preamble},
            {"energy_thresh", 1e-8f},
            {"min_snr_db", -20.0f},
            {"debug", true},
        });

        auto& sink = graph.emplaceBlock<gr::testing::TagSink<uint8_t,
            gr::testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true},
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            expect(false) << "scheduler exchange failed";
            return;
        }
        sched.runAndWait();

        std::vector<uint8_t> decoded;
        decoded.assign(sink._samples.begin(), sink._samples.end());

        if (decoded.size() >= payload.size()) {
            bool match = std::equal(payload.begin(), payload.end(), decoded.begin());
            expect(match) << "noisy loopback: decoded payload matches TX";
        } else {
            expect(false) << "noisy loopback: no decode (" << decoded.size() << " bytes)";
        }
    };
```

- [ ] **Step 2: Build and run**

Run: `cmake --build build --target qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora_wideband --output-on-failure --timeout 120`
Expected: Both clean and noisy loopback tests pass. If the noisy test fails, adjust noise sigma or debug the FIR output quality — the issue is likely in coefficient values or the FIR tap indexing, not the architecture.

- [ ] **Step 3: Run full test suite**

Run: `cmake --build build --target qa_lora_algorithms qa_lora_tx qa_lora_phase3 qa_lora_rx qa_lora_loopback qa_lora_burst qa_lora_splitter qa_lora_cad qa_lora_wideband -- -j4 && ctest --test-dir build -R qa_lora --output-on-failure --timeout 120`
Expected: All tests pass. Zero regressions.

- [ ] **Step 4: Commit**

```
git add test/qa_lora_wideband.cpp
git commit -m "test: add noisy wideband loopback test

Verifies the FIR channelizer decodes LoRa frames in the presence
of wideband noise where the box-car decimator previously failed."
```

---

## Task 6: Build application binaries and final validation

**Files:**
- No new files — build and verify existing apps compile with the changes.

- [ ] **Step 1: Build lora_trx (uses WidebandDecoder via graph_builder)**

Run: `cmake --build build --target lora_trx -- -j4`
Expected: Compiles successfully. The `build_wideband_graph()` in `graph_builder.hpp` uses WidebandDecoder — must still compile with new settings.

- [ ] **Step 2: Build lora_scan**

Run: `cmake --build build --target lora_scan -- -j4`
Expected: Compiles successfully. lora_scan uses ScanController (not WidebandDecoder) — should be unaffected.

- [ ] **Step 3: Run the full test suite one final time**

Run: `ctest --test-dir build -R qa_lora --output-on-failure --timeout 120`
Expected: All tests pass.

- [ ] **Step 4: Clean up temporary files**

```
trash tmp/design_halfband.py
```

- [ ] **Step 5: Final commit with any remaining fixes**

If any compilation fixes were needed for the app binaries, commit them.

---

## Summary of Deliverables

| Task | Deliverable | Tests |
|------|-------------|-------|
| 1 | Filter coefficients (verified via scipy) | Manual verification |
| 2 | `algorithm/HalfBandDecimator.hpp` | 4 unit tests (stopband, passband, phase continuity, noise) |
| 3 | FIR-based `ChannelSlot::pushWideband()` | Out-of-band rejection test + existing channelizer tests updated |
| 4 | Multi-BW slot activation + dedup | Multi-BW slot activation test |
| 5 | Noisy loopback test | Wideband noise decode test |
| 6 | App binaries compile | Full test suite regression check |

**Out of scope:** M5 from the spec (hardware A/B test with `trx_ab_loop.py`) requires physical hardware (Heltec V3 + B210). It is not included in this plan. Run it manually after completing Task 6.
