# lora_scan performance baseline — 2026-03-26

Branch: `fix/dc-hot-channel-exclusion`
Binary: `lora_scan fbac13a` (built Mar 26 2026)
Hardware: B210 (LibreSDR_B220mini), internal clock, USB 3.0
UHD buffer: `num_recv_frames=512, recv_frame_size=16384` (65ms FIFO)
No TX source (ambient RF only).

## Streaming scanner (`streaming=true`)

Config: `apps/config.toml` — EU868 863-870 MHz, 16 MS/s, os=4, BW=[62.5k,125k,250k]

```
SUMMARY sweeps=620 avg_dur=55ms min_dur=37ms max_dur=100ms p50_dur=55ms
        avg_hot=7.8 total_det=0 total_ovf=104
        avg_ratio=0.0 min_ratio=0.0 max_ratio=0.0
        sfs=- slopes=-
```

| Metric | Value |
|--------|-------|
| Sweeps | 620 in ~35s |
| p50 sweep | 55ms |
| min/max sweep | 37ms / 100ms |
| avg_hot | 7.8 channels |
| Overflows | 104 total (~1 per 6 sweeps, ~3/s) |
| Detections | 0 (no TX) |

## Tuning scanner (`streaming=false`)

Config: `tmp/config_tuning_nomcr.toml` — EU868, 16 MS/s, internal clock,
`master_clock=16000000` (avoids 16→32 MHz MCR switch)

```
sweep 1  dur 41217ms  hot 13/113  det 0  OVF 14
sweep 2  dur 103363ms hot 35/113  det 0  OVF 23
sweep 3  dur 67882ms  hot 17/113  det 0  OVF 26
sweep 4  dur 63988ms  hot 15/113  det 0  OVF 28
sweep 5  dur 18780ms  hot  3/113  det 0  OVF 29
stopped after 5 sweep(s), 29 overflow(s), 0 drop(s)
```

| Metric | Value |
|--------|-------|
| Sweeps | 5 in ~5 min |
| avg sweep | 59ms (streaming) vs 59046ms (tuning) — 1000x slower |
| p50 sweep | ~64s (sweeps 2-4) |
| min/max sweep | 19s / 103s |
| avg_hot | 17 channels (settling: 13→35→17→15→3) |
| Overflows | 29 total (~6/sweep) |
| Detections | 0 (no TX) |

### Tuning scanner notes

- Hot channel count settles from 35 (B210 thermal transient) to 3 after 4 sweeps
- Sweep duration is proportional to hot count: 103s@35hot → 19s@3hot
- Steady-state: ~19-20s/sweep with 3 hot channels (noise spurs)
- `capture_samples()` timeout (2000ms) fires on ~50% of L2 probes (partial fills 258k-1.5M of 2M requested)
- `extractTileEnergy()` timeout (50ms) fires on ~30% of L1 snapshots during overflow bursts

## Comparison with prior baselines

| Metric | Streaming (this) | Streaming (2026-03-20) | Change |
|--------|------------------|------------------------|--------|
| p50 sweep | 55ms | 524ms | **9.5x faster** (rotating probe window) |
| Overflows | 104/620 | 0/332 | Higher (clock unlocked this session) |

| Metric | Tuning (this) | Tuning (pre-timeout) | Change |
|--------|---------------|---------------------|--------|
| Sweep completes | Yes | No (infinite hang) | **Fixed** |
| Steady-state sweep | ~19s | N/A | New capability |
