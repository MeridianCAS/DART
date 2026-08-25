#!/usr/bin/env bash
# Fig. 8 end-to-end gate (on arm; final gate parameters).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: fig08_gate_on"

E2=(--angles 20 --rise 7 --run-up 50 --base-x 50 --v-entry 22
    --launch-mode approach --fresh-spawn-dart 1 --dart-pitch-control differential
    --dart-roll-adaptive 1
    --rolls 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400)

run_cohort "fig08_e2e_gate_on" "${E2[@]}" \
  --reachability-gate 1 --gate-v-crit 11.0 --gate-a-brake 4.0 \
  --gate-adaptive-abrake 1 --gate-coast-m 1.5 \
  --gate-lip-power-recover 1 --gate-lip-launch-target 11.0 || true

log "QUEUE DONE"
