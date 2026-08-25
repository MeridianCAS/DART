#!/usr/bin/env bash
# Minimal cohort runner for paper reproduction (sequential, resumable via DONE markers).
set -uo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
PY="${PYTHON:-python}"
cd "$REPO"
OUT="${OUT:-data/queue_runs}"
LOG="$OUT/logs"
mkdir -p "$LOG" "$OUT"

export DART_JUMP_DIR="${DART_JUMP_DIR:-sim/scenarios}"
export PYTHONUNBUFFERED=1

ANCHOR="${ANCHOR:-dart_a20r7_valley12}"
STRATS="${STRATS:-dart,dart_latched,rwpd}"

log() { echo "=== [$(date -Iseconds)] $*"; }

run_cohort() {
 local tag="$1"
 shift
 local done_m="$OUT/${tag}.DONE"
 local logf="$LOG/${tag}.log"
 if [[ -f "$done_m" ]]; then
 log "SKIP $tag (already done)"
 return 0
 fi
 log "START $tag"
 if ! "$PY" scripts/dart_bench.py "$@" --tag "$tag" >"$logf" 2>&1; then
 log "FAIL $tag (see $logf)"
 return 1
 fi
 touch "$done_m"
 log "OK $tag"
}
