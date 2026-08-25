#!/usr/bin/env bash
# Per-axis ablation at alpha=28; approach mass ladder.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: ablation_mass_peraxis"

AIR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90)
COND1=(--launch-mode air-impulse --air-impulse-pitch-deg 12 --air-impulse-roll-deg 28)
APPR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90
  --launch-mode approach --approach-ground-paused 1
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0)

run_cohort "abl_peraxis_air_a28" \
  --jump-scenario dart_a28r7_valley12 \
  --simul-3way 1 --simul-strategies dart_dual,dart_pitch_only,dart_roll_only \
  "${COND1[@]}" "${AIR_COMMON[@]}" || true

run_cohort "rob_mass_m080_appr" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  --pc vehicles/sbr/dart_4motor_m080.pc "${APPR_COMMON[@]}" || true
run_cohort "rob_mass_m120_appr" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  --pc vehicles/sbr/dart_4motor_m120.pc "${APPR_COMMON[@]}" || true

log "QUEUE DONE"
