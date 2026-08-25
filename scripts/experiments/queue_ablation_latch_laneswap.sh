#!/usr/bin/env bash
# Lane-swap confound control for latch ablation.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: ablation_latch_laneswap"

APPR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90
  --launch-mode approach --approach-ground-paused 1
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0)

run_cohort "abl_latch_laneswap" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies "$STRATS" \
  "${APPR_COMMON[@]}" || true

log "QUEUE DONE"
