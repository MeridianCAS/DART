#!/usr/bin/env bash
# Per-flight roll-latch ablation matrix.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: ablation_latch_matrix"

AIR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90)
COND1=(--launch-mode air-impulse --air-impulse-pitch-deg 12 --air-impulse-roll-deg 28)
MICRO=(--takeoff-state-jitter-sigma 0.25 --takeoff-state-jitter-seed 20260703)
APPR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90
  --launch-mode approach --approach-ground-paused 1
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0)

run_cohort "abl_latch_a28_air" \
  --jump-scenario dart_a28r7_valley12 --simul-3way 1 --simul-strategies "$STRATS" \
  "${COND1[@]}" "${AIR_COMMON[@]}" || true

run_cohort "abl_latch_s0b_lat20" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies "$STRATS" \
  "${COND1[@]}" "${AIR_COMMON[@]}" "${MICRO[@]}" \
  --actuator-latency-ms 20 || true

run_cohort "abl_latch_flat_appr" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies "$STRATS" \
  "${APPR_COMMON[@]}" || true

run_cohort "abl_latch_m120_appr" \
  --jump-scenario "$ANCHOR" --simul-3way 1 --simul-strategies "$STRATS" \
  --pc vehicles/sbr/dart_4motor_m120.pc "${APPR_COMMON[@]}" || true

log "QUEUE DONE"
