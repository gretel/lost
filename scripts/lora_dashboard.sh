#!/usr/bin/env bash
# lora_dashboard.sh — open two WezTerm tabs, each a 2×2 grid
#
#  Tab "rx":                      Tab "app":
#  ┌─────────────┬─────────────┐  ┌─────────────┬─────────────┐
#  │  lora_trx   │ meshcore-cli│  │ lora_duckdb │  lora_wav   │
#  ├─────────────┼─────────────┤  ├─────────────┼─────────────┤
#  │  lora_mon   │  lora_wfall │  │ meshcore_br │  lora_agg   │
#  └─────────────┴─────────────┘  └─────────────┴─────────────┘
#
# Each tab uses a single 50/50 root split — the only WezTerm layout
# that resizes both columns equally when the window is resized.
#
# Usage: lora_dashboard.sh
set -euo pipefail

CWD="$(cd "$(dirname "$0")/.." && pwd)"
MCLI_DIR="$CWD/../meshcore-cli"

send() {
    local pane="$1" cmd="$2"
    printf "%s\n" "$cmd" | wezterm cli send-text --pane-id "$pane" --no-paste
}

D="direnv exec $CWD"

# ── Tab 1: rx ────────────────────────────────────────────────────────────────
T1_L=$(wezterm cli spawn --pane-id "$WEZTERM_PANE" --cwd "$CWD/apps" -- elvish)
wezterm cli set-tab-title --pane-id "$T1_L" rx

T1_R=$(wezterm cli split-pane --pane-id "$T1_L" --right --percent 50 --cwd "$MCLI_DIR" -- elvish)
T1_BL=$(wezterm cli split-pane --pane-id "$T1_L" --bottom --percent 50 --cwd "$CWD" -- elvish)
T1_BR=$(wezterm cli split-pane --pane-id "$T1_R" --bottom --percent 50 --cwd "$CWD" -- elvish)

send "$T1_L"  "$D lora_trx"
send "$T1_R"  "sleep 5; uv run meshcore-cli -t 127.0.0.1 -p 7834"
send "$T1_BL" "$D lora_mon.py"
send "$T1_BR" "$D lora_waterfall.py"

# ── Tab 2: app ───────────────────────────────────────────────────────────────
T2_L=$(wezterm cli spawn --pane-id "$T1_L" --cwd "$CWD" -- elvish)
wezterm cli set-tab-title --pane-id "$T2_L" app

T2_R=$(wezterm cli split-pane --pane-id "$T2_L" --right --percent 50 --cwd "$CWD"      -- elvish)
T2_BL=$(wezterm cli split-pane --pane-id "$T2_L" --bottom --percent 50 --cwd "$CWD"    -- elvish)
T2_BR=$(wezterm cli split-pane --pane-id "$T2_R" --bottom --percent 50 --cwd "$CWD"      -- elvish)

send "$T2_L"  "$D lora_duckdb.py"
send "$T2_R"  "$D lora_wav.py"
send "$T2_BL" "$D meshcore_bridge.py"
send "$T2_BR" "$D lora_agg.py"

# ── Focus tab 1, top-left ────────────────────────────────────────────────────
wezterm cli activate-pane --pane-id "$T1_L"
