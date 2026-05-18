---
date: 2026-05-16T19:30:00+0200
author: Tom Hensel (via agent)
commit: 4bc9a73
branch: iio-on-device
repository: gr4-lora
topic: "IIO Header Decode — Root Cause Isolated to OS=40 Stride Decimation"
tags: [debug, iio, phy, stride-decimation, header-decode]
status: in_progress
last_updated: 2026-05-16T19:30:00+0200
last_updated_by: Tom Hensel (via agent)
type: bug_fix
---

# Handoff: IIO header decode — stride decimation PMR confirmed, dechirp C++ formula documented

## Progress Summary

### Verified / Done

1. **TX path fixed** — `set_mode_at_init=true` removed from `apps/lora_trx.cpp`. New binary builds, deploys, runs. (commit `fb0dbc8`)
2. **IIO buffer conflict found & documented** — chirpmunk-trx (Rust daemon) holds `cf-ad9361-lpc` buffer, starving lora_trx via `uri=local:`. Kill chirpmunk-trx first. Symptom: lora_trx alive but zero RX log output. Memory lesson saved.
3. **IIO buffer is continuous** — per-refill debug logging shows every `iio_buffer_refill()` returns full `32768` samples (131072 bytes, step=4). No gaps or partial reads. (commit `4bc9a73`, incubator `fd00c2f`)
4. **Aliasing ruled out** — AD9361 BW set to both 200 kHz (min clamp) and 2.5 MHz. Same decode corruption at both settings. (tezuka sysfs test)
5. **Real IIO data flows** — lora_trx detects frames, decodes headers, PMR=14-75 in real-time logs. But `csum_ok=0` on all frames.
6. **Header corruption confirmed** — Same ADVERT (SF7, transmitted 3× by companion) produces 3 different headers. Rules out "noise lock" hypothesis.
7. **HDR_GRAY trace logging** added to DecodeChain to show gray-decoded shifted symbols (commit `35280ba`). Deployed on tezuka.
8. **Offline IQ capture** — 16 MB capture made with `iio_readdev`. Stride dechirp+FFT formula verified from C++ source `utilities.hpp`.

### Root Cause Hypothesis

The dechirp formula from `build_upchirp` in `blocks/include/gnuradio-4.0/lora/algorithm/utilities.hpp`:

```
phi_up(n) = 2*pi * [n^2 / (2*N*os^2) - 0.5*n/os]
downchirp[n] = conj(upchirp[n]) = exp(-j*pi*n^2/(N*os^2) + j*pi*n/os)
```

Offline analysis of captured IQ:

- **OS=40 stride**: Best PMR = 4.31 (poor)
- **OS=4 full-rate**: Best PMR = 11.95 (good)
- 2.8× PMR loss from stride decimation

BUT: lora_trx real-time logs show PMR=14-75 on the same signal. This discrepancy needs investigation:

- Does the real-time decoder use preamble chirp accumulation (averaging) that boosts PMR?
- Did the iio_readdev capture miss the companion TX window?
- Is the stride dechirp phase index correct in offline script?

### Key References

- `blocks/include/gnuradio-4.0/lora/algorithm/utilities.hpp:55-75` — `build_upchirp` phase formula
- `blocks/include/gnuradio-4.0/lora/detail/ChirpRefs.hpp:29-35` — `build_downchirp` wrapper
- `gr4-incubator/blocks/iio/include/gnuradio-4.0/iio/IIOSource.hpp` — debug logging added at `fd00c2f`
- `apps/graph_builder.hpp:195-240` — IIOSource `debug` flag wired
- Tezuka: `~/lora/bin/lora_trx` (MD5 at deploy time: `5637f2d9` for commit `4bc9a73`)
- IIO diagnostic logs: `/tmp/lora_refill.log` (IIOSource per-refill debug)
- Raw IQ capture: `tmp/iio_capture.bin` (16 MB, 4194304 complex samples at 2.5 MS/s)
- Offline analysis scripts: `tmp/check_capture.py`, `tmp/check_capture2.py`

### next steps

1. **Verify capture has actual signal** — cross-correlate the 16 MB capture against a preamble to find exact symbol start. Or re-trigger companion TX while lora_trx is running and compare PMR at OS=4 vs OS=40.
2. **Test OS=4 in lora_trx** — modify config to use a BW where stride is ~4 (e.g. BW=625 kHz at 2.5 MS/s → OS=4). Verify decode works.
3. **Preamble coherence** — check if lora_trx PreambleSync accumulates U4-U6 (3 chirp averaging) giving 3× PMR boost vs single-chirp FFT in offline script.
4. **Final plan**: If stride at OS=40 degrades PMR below decode threshold, add anti-alias filter or change to power-of-2 OS (32) to enable DSP AA filter (`use_aa_filter=true`).

### Config (current on tezuka)

`~/lora/etc/config-tezuka-tx-test.toml` — IIO TX-test config with `enable_tx=true`.
Companion at `10.0.23.152:5000`.
