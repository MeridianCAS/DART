#!/usr/bin/env bash
# Takeoff-state jitter sweep (Sec. V text); banked per-axis ablation.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: jitter_banked_probes"

CAMBER_ARGS=(--v-entry 13 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90
  --launch-mode approach --approach-ground-paused 1
  --simul-3way 1
  --simul-layout y_copy --simul-copy-spacing 45
  --simul-takeoff-spread-gate 0
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0
  --control-trace 0 --hud 0 --max-steps 1200 --cond-jitter 0
  --dart-pitch-control differential
  --reachability-gate 1 --gate-v-crit 11.0 --gate-a-brake 4.0
  --gate-adaptive-abrake 1 --gate-coast-m 1.5 --gate-lip-power-recover 1
  --gate-lip-launch-target 11.0
  --camber-air-early-landmatch 0 --landmatch 1
  --camber-air-touch-roll-boost 1.0 --camber-air-touch-roll-gain 1.0
  --camber-air-pred-horizon-sec 0)
AIR_COMMON=(--rolls 30 --paired-valid-target 30 --control-trace 0 --hud 0
  --max-steps 1200 --paired-max-attempts 90)
COND1=(--launch-mode air-impulse --air-impulse-pitch-deg 12 --air-impulse-roll-deg 28)

run_cohort "abl_peraxis_banked_camber12" \
  --jump-scenario dart_a20r7_camber12 \
  --simul-strategies dart_dual,dart_pitch_only,dart_roll_only \
  "${CAMBER_ARGS[@]}" || true

for S in 0 1 2 3 5; do
  run_cohort "rob_jitter_s${S}" \
    --jump-scenario "$ANCHOR" \
    --simul-3way 1 --simul-strategies dart,rwpd,tobb \
    "${COND1[@]}" "${AIR_COMMON[@]}" \
    --takeoff-state-jitter-sigma "$S" --actuator-latency-ms 20 || true
done

log "QUEUE DONE"
