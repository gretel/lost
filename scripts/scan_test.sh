#!/usr/bin/env bash
# SPDX-License-Identifier: ISC
# scan_test.sh -- Automated A/B test harness for streaming scan pipeline.
#
# Requires:
#   - serial_bridge.py running on TCP 7835 (or --bridge-port)
#   - lora_scan running (UDP 5557 or --scan-port)
#
# Usage:
#   scripts/scan_test.sh [--adverts N] [--gap SECS] [--label LABEL]

set -euo pipefail

ADVERTS=10
GAP=3
SCAN_PORT=5557
BRIDGE_PORT=7835
BRIDGE_HOST="127.0.0.1"
LABEL="streaming"
PERF_LOG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --adverts)  ADVERTS="$2"; shift 2 ;;
        --gap)      GAP="$2"; shift 2 ;;
        --scan-port) SCAN_PORT="$2"; shift 2 ;;
        --bridge-port) BRIDGE_PORT="$2"; shift 2 ;;
        --label)    LABEL="$2"; shift 2 ;;
        *) echo "unknown: $1"; exit 1 ;;
    esac
done

TIMESTAMP=$(date -u +%Y%m%d_%H%M%S)
PERF_LOG="tmp/perf_${LABEL}_${TIMESTAMP}.log"
mkdir -p tmp

echo "=== scan_test.sh: $LABEL ==="
echo "    adverts=$ADVERTS gap=${GAP}s scan_port=$SCAN_PORT bridge=$BRIDGE_HOST:$BRIDGE_PORT"
echo "    log=$PERF_LOG"
echo ""

# Start perf monitor in background
cd scripts
python3 lora_perf.py --port "$SCAN_PORT" > "../$PERF_LOG" 2>&1 &
PERF_PID=$!
cd ..
trap 'kill $PERF_PID 2>/dev/null; wait $PERF_PID 2>/dev/null' EXIT

# Wait for perf to subscribe and scanner to stabilize
sleep 5

echo "sending $ADVERTS ADVERTs (${GAP}s gap)..."
for i in $(seq 1 "$ADVERTS"); do
    printf "  advert %d/%d..." "$i" "$ADVERTS"
    if uvx meshcore-cli -q -t "$BRIDGE_HOST" -p "$BRIDGE_PORT" advert 2>/dev/null; then
        echo " ok"
    else
        echo " FAIL"
    fi
    if [[ $i -lt $ADVERTS ]]; then
        sleep "$GAP"
    fi
done

# Wait for ring buffer flush + final sweep
echo ""
echo "waiting for flush..."
sleep 10

# Stop perf monitor
kill $PERF_PID 2>/dev/null
wait $PERF_PID 2>/dev/null || true
trap - EXIT

# Extract final SUMMARY line
SUMMARY=$(grep "^SUMMARY" "$PERF_LOG" | tail -1)
DET_COUNT=$(grep "^DET" "$PERF_LOG" | wc -l | tr -d ' ')

echo ""
echo "=== RESULTS ($LABEL) ==="
echo "$SUMMARY"
echo "DET lines: $DET_COUNT"
echo ""
echo "Full log: $PERF_LOG"

# Parse key metrics for quick comparison
if [[ -n "$SUMMARY" ]]; then
    avg_dur=$(echo "$SUMMARY" | grep -o 'avg_dur=[0-9]*' | cut -d= -f2)
    p50_dur=$(echo "$SUMMARY" | grep -o 'p50_dur=[0-9]*' | cut -d= -f2)
    total_det=$(echo "$SUMMARY" | grep -o 'total_det=[0-9]*' | cut -d= -f2)
    total_ovf=$(echo "$SUMMARY" | grep -o 'total_ovf=[0-9]*' | cut -d= -f2)
    avg_ratio=$(echo "$SUMMARY" | grep -o 'avg_ratio=[0-9.]*' | cut -d= -f2)
    sfs=$(echo "$SUMMARY" | grep -o 'sfs=.*' | cut -d= -f2)
    sweeps=$(echo "$SUMMARY" | grep -o 'sweeps=[0-9]*' | cut -d= -f2)

    echo ""
    echo "| Metric     | Value |"
    echo "|------------|-------|"
    echo "| sweeps     | $sweeps |"
    echo "| avg_dur    | ${avg_dur}ms |"
    echo "| p50_dur    | ${p50_dur}ms |"
    echo "| total_det  | $total_det |"
    echo "| total_ovf  | $total_ovf |"
    echo "| avg_ratio  | $avg_ratio |"
    echo "| SFs        | $sfs |"
fi
