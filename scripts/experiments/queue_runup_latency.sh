#!/usr/bin/env bash
# Latency at σ→0⁺, run-up surfaces, flat approach H2H, per-axis extensions.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: runup_latency"

AIR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90)
COND1=(--launch-mode air-impulse --air-impulse-pitch-deg 12 --air-impulse-roll-deg 28)
MICRO=(--takeoff-state-jitter-sigma 0.25 --takeoff-state-jitter-seed 20260703)
APPR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90
  --launch-mode approach --approach-ground-paused 1
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0)

run_cohort "abl_peraxis_jitter_lat20" \
  --jump-scenario "$ANCHOR" \
  --simul-3way 1 --simul-strategies dart_dual,dart_pitch_only,dart_roll_only \
  "${COND1[@]}" "${AIR_COMMON[@]}" "${MICRO[@]}" \
  --actuator-latency-ms 20 || true

run_cohort "rob_jitter_s0b_lat0" \
  --jump-scenario "$ANCHOR" \
  --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  "${COND1[@]}" "${AIR_COMMON[@]}" "${MICRO[@]}" \
  --actuator-latency-ms 0 || true

for L in 10 40; do
  run_cohort "rob_jitter_s0b_lat${L}" \
    --jump-scenario "$ANCHOR" \
    --simul-3way 1 --simul-strategies dart,rwpd,tobb \
    "${COND1[@]}" "${AIR_COMMON[@]}" "${MICRO[@]}" \
    --actuator-latency-ms "$L" || true
done

for GT in ASPHALT GRAVEL DIRT; do
  gt_lc=$(echo "$GT" | tr '[:upper:]' '[:lower:]')
  run_cohort "rob_surface_${gt_lc}" \
    --jump-scenario "$ANCHOR" \
    --simul-3way 1 --simul-strategies dart,rwpd,tobb \
    --runup-ground-type "$GT" \
    "${APPR_COMMON[@]}" || true
done

run_cohort "appr_flat_h2h" \
  --jump-scenario "$ANCHOR" \
  --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  "${APPR_COMMON[@]}" || true
run_cohort "abl_peraxis_appr" \
  --jump-scenario "$ANCHOR" \
  --simul-3way 1 --simul-strategies dart_dual,dart_pitch_only,dart_roll_only \
  "${APPR_COMMON[@]}" || true

log "QUEUE DONE"
