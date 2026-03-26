# lora_test.py --attach Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `--attach` flag to `lora_test.py` that skips binary (lora_trx/lora_scan) startup/teardown, enabling tests against already-running daemons (e.g. for live demo with `lora_scan` + `lora_spectrum.py` already running).

**Architecture:** Single flag `--attach` on all subcommands. When set: skip `_start_process(binary)`, skip `_wait_udp(udp_port)`, skip `_stop_process(binary_proc)` at teardown; skip startup delay. Companion (`serial_bridge`) lifecycle unchanged. `tx` and `bridge` modes also get the flag for consistency but it skips their binary start too.

**Tech Stack:** Python 3.12, argparse, existing `lora_test.py` structure.

---

## Affected files

- **Modify:** `scripts/lora_test.py` — add `--attach` to argparse, thread it through all four mode paths

No new files needed. No test file changes (existing tests don't cover binary lifecycle).

---

### Task 1: Add `--attach` to argparse

**Files:**
- Modify: `scripts/lora_test.py:1173-1198` (the `for name in ("decode", "scan", "tx", "bridge"):` loop)

- [ ] **Step 1: Add the flag inside the subcommand loop**

  In the `for name in ("decode", "scan", "tx", "bridge"):` loop (around line 1196), add after the existing `p.add_argument("--label", ...)`:

  ```python
  p.add_argument(
      "--attach",
      action="store_true",
      default=False,
      help="Attach to already-running SDR binary; skip start/stop of lora_trx/lora_scan",
  )
  ```

- [ ] **Step 2: Verify the flag parses cleanly**

  ```bash
  python3 scripts/lora_test.py scan --help
  ```
  Expected: `--attach` appears in the help output.

  ```bash
  python3 scripts/lora_test.py decode --help
  ```
  Expected: same.

---

### Task 2: Thread `--attach` through `main()` — decode/scan path

**Files:**
- Modify: `scripts/lora_test.py:1292-1313` (the binary start block in `main()`)

The relevant block currently reads:
```python
_info(f"Starting {binary}")
binary_proc = _start_process(
    [binary, "--config", args.config],
    f"tmp/{mode}.log",
)
# Tuning scanner (streaming=false) needs longer startup ...
udp_timeout = 90.0 if mode == "scan" else 30.0
if not _wait_udp(udp_port, timeout=udp_timeout):
    _err(f"{binary} failed to start (see tmp/{mode}.log)")
    _stop_process(binary_proc)
    _stop_process(bridge_proc)
    sys.exit(1)
_info(f"{binary} ready")
```

And the startup delay:
```python
startup_delay = 10.0 if mode == "scan" else 1.0
time.sleep(startup_delay)
```

- [ ] **Step 1: Wrap binary start in `if not args.attach`**

  Replace the binary start block with:

  ```python
  binary_proc = None
  if not args.attach:
      _info(f"Starting {binary}")
      binary_proc = _start_process(
          [binary, "--config", args.config],
          f"tmp/{mode}.log",
      )
      udp_timeout = 90.0 if mode == "scan" else 30.0
      if not _wait_udp(udp_port, timeout=udp_timeout):
          _err(f"{binary} failed to start (see tmp/{mode}.log)")
          _stop_process(binary_proc)
          _stop_process(bridge_proc)
          sys.exit(1)
      _info(f"{binary} ready")
  else:
      _info(f"Attaching to already-running {binary} on port {udp_port}")
  ```

- [ ] **Step 2: Skip startup delay in attach mode**

  Replace:
  ```python
  startup_delay = 10.0 if mode == "scan" else 1.0
  time.sleep(startup_delay)
  ```
  With:
  ```python
  if not args.attach:
      startup_delay = 10.0 if mode == "scan" else 1.0
      time.sleep(startup_delay)
  ```

- [ ] **Step 3: `_stop_process(binary_proc)` in finally already handles None**

  Check `_stop_process`:
  ```python
  def _stop_process(proc: subprocess.Popen | None) -> None:
      if proc is None:
          return
  ```
  It already guards `None` — no change needed in the `finally` block.

---

### Task 3: Thread `--attach` through `_run_tx_experiment()`

**Files:**
- Modify: `scripts/lora_test.py:1361-1427` (`_run_tx_experiment`)

The function currently starts `binary_proc` unconditionally. `args` is passed in, so `args.attach` is available.

- [ ] **Step 1: Wrap binary start**

  Replace:
  ```python
  # Start lora_trx (needed for SDR TX via UDP)
  _info(f"Starting {binary}")
  binary_proc = _start_process(
      [binary, "--config", args.config],
      "tmp/tx.log",
  )
  if not _wait_udp(TRX_PORT):
      _err(f"{binary} failed to start (see tmp/tx.log)")
      _stop_process(binary_proc)
      sys.exit(1)
  _info(f"{binary} ready")
  ```
  With:
  ```python
  binary_proc = None
  if not args.attach:
      _info(f"Starting {binary}")
      binary_proc = _start_process(
          [binary, "--config", args.config],
          "tmp/tx.log",
      )
      if not _wait_udp(TRX_PORT):
          _err(f"{binary} failed to start (see tmp/tx.log)")
          _stop_process(binary_proc)
          sys.exit(1)
      _info(f"{binary} ready")
  else:
      _info(f"Attaching to already-running {binary} on port {TRX_PORT}")
  ```

- [ ] **Step 2: `_stop_process(binary_proc)` in finally already handles None — no change needed**

---

### Task 4: Thread `--attach` through `_run_bridge_experiment()`

**Files:**
- Modify: `scripts/lora_test.py:1430-1619` (`_run_bridge_experiment`)

Three binaries are started: `binary_proc` (lora_trx), `agg_proc` (lora_agg), `bridge_proc` (meshcore_bridge). In attach mode, skip all three. The function receives `args`.

- [ ] **Step 1: Wrap all three binary starts**

  Replace the three start blocks (lines ~1449-1495):

  ```python
  binary_proc = None
  agg_proc = None
  bridge_proc = None

  if not args.attach:
      # 1. Start lora_trx
      _info(f"Starting {binary}")
      binary_proc = _start_process(
          [binary, "--config", args.config],
          "tmp/bridge_trx.log",
      )
      if not _wait_udp(TRX_PORT):
          _err(f"{binary} failed to start (see tmp/bridge_trx.log)")
          _stop_process(binary_proc)
          sys.exit(1)
      _info(f"{binary} ready")

      # 2. Start lora_agg.py
      _info("Starting lora_agg")
      agg_cmd = [
          sys.executable,
          str(SCRIPT_DIR / "lora_agg.py"),
          "--upstream",
          f"127.0.0.1:{TRX_PORT}",
          "--listen",
          f"127.0.0.1:{AGG_PORT}",
      ]
      agg_proc = _start_process(agg_cmd, "tmp/lora_agg.log")
      if not _wait_udp(AGG_PORT):
          _err("lora_agg failed (see tmp/lora_agg.log)")
          _stop_process(agg_proc)
          _stop_process(binary_proc)
          sys.exit(1)
      _info("lora_agg ready")

      # 3. Start meshcore_bridge.py
      _info("Starting meshcore_bridge")
      bridge_cmd = [
          sys.executable,
          str(SCRIPT_DIR / "meshcore_bridge.py"),
          "--connect",
          f"127.0.0.1:{AGG_PORT}",
          "--port",
          str(MESHCORE_BRIDGE_PORT),
      ]
      bridge_proc = _start_process(bridge_cmd, "tmp/meshcore_bridge.log")
      if not _wait_tcp(MESHCORE_BRIDGE_PORT):
          _err("meshcore_bridge failed (see tmp/meshcore_bridge.log)")
          _stop_process(bridge_proc)
          _stop_process(agg_proc)
          _stop_process(binary_proc)
          sys.exit(1)
      _info("meshcore_bridge ready")
  else:
      _info(f"Attaching to already-running stack (trx:{TRX_PORT} agg:{AGG_PORT} bridge:{MESHCORE_BRIDGE_PORT})")
  ```

- [ ] **Step 2: Verify teardown `finally` block calls `_stop_process` on all three — already handles None**

---

### Task 5: Update docstring to document `--attach`

**Files:**
- Modify: `scripts/lora_test.py:3-27` (module docstring)

- [ ] **Step 1: Add attach usage examples**

  In the `Usage:` section, after the existing examples, add:

  ```
      # Attach to already-running daemons (e.g. for live demo):
      python3 scripts/lora_scan --config apps/config.toml &
      python3 scripts/lora_spectrum.py &
      python3 scripts/lora_test.py scan --matrix basic --serial /dev/cu.usbserial-0001 --attach
  ```

  Also update the third design principle bullet to read:
  ```
    - With --attach: harness connects to already-running SDR binary; only manages serial_bridge
  ```
  (replacing `Harness manages all processes; do not pre-start serial_bridge or lora_trx`)

---

### Task 6: Manual smoke test

- [ ] **Step 1: Verify non-attach path unchanged**

  ```bash
  python3 scripts/lora_test.py scan --help
  python3 scripts/lora_test.py decode --help
  ```
  Confirm `--attach` shows in both.

- [ ] **Step 2: Dry-run attach mode without hardware**

  With `lora_scan` NOT running:
  ```bash
  python3 scripts/lora_test.py scan --matrix basic --tcp 127.0.0.1:7835 --attach
  ```
  Expected: script starts, prints `Attaching to already-running ...`, companion connect
  fails (TCP not up), exits cleanly with error — does NOT try to start `lora_scan`.

- [ ] **Step 3: Commit**

  ```bash
  git add scripts/lora_test.py
  git commit -m "feat(lora_test): add --attach flag to skip SDR binary lifecycle"
  ```
