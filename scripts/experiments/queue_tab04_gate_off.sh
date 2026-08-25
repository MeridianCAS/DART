#!/usr/bin/env bash
# Fig. 8 end-to-end gate (off arm); Tab IV steep-lip approach head-to-head.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: tab04_gate_off"

E2=(--angles 20 --rise 7 --run-up 50 --base-x 50 --v-entry 22
    --launch-mode approach --fresh-spawn-dart 1 --dart-pitch-control differential
    --dart-roll-adaptive 1
    --gate-v-crit 8.0 --gate-a-brake 4.0
    --rolls 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400)

run_cohort "fig08_e2e_gate_off" "${E2[@]}" --reachability-gate 0 || true

run_cohort "tab04_appr_h2h_steeplip" \
  --angles 20 --rise 7 --run-up 50 --base-x 50 --v-entry 15 \
  --launch-mode approach --approach-ground-paused 1 \
  --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  --dart-roll-adaptive 1 \
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0 \
  --dart-pitch-control differential \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 || true

log "QUEUE DONE"
