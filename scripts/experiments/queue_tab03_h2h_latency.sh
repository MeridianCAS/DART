#!/usr/bin/env bash
# Tab III H2H, latency sweep, per-axis ablation, pitch-authority and nose-up probes.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: tab03_h2h_latency"

INJ=(--air-impulse-pitch-deg 12 --air-impulse-roll-deg 28)
MRC=(--simul-3way 1 --simul-strategies dart --launch-mode air-impulse
     --control-trace 0 --hud 0 --max-steps 1200)

run_cohort "smoke_h2h" \
  --jump-scenario dart_a20r7_valley12 --multiroll-strategies dart,rwpd,tobb,dart_replicate \
  "${MRC[@]}" "${INJ[@]}" --rolls 8 --paired-valid-target 8 --paired-max-attempts 24 || true
if [[ ! -f "$OUT/smoke_h2h.DONE" ]]; then
  log "ABORT: multiroll smoke failed"; exit 9
fi

for A in a20 a24 a28; do
  run_cohort "tab03_h2h_${A}" \
    --jump-scenario "dart_${A}r7_valley12" --multiroll-strategies dart,rwpd,tobb,dart_replicate \
    "${MRC[@]}" "${INJ[@]}" --dart-roll-adaptive 1 \
    --rolls 120 --paired-valid-target 120 --paired-max-attempts 220 || true
done

run_cohort "abl_peraxis_stressed" \
  --jump-scenario dart_a24r7_valley12 --multiroll-strategies dart_dual,dart_pitch_only,dart_roll_only,dart_replicate \
  "${MRC[@]}" "${INJ[@]}" --rolls 120 --paired-valid-target 120 --paired-max-attempts 220 || true

for P in 10 15 20 25 30 38 45; do
  run_cohort "probe_pitch_envelope_p${P}" \
    --jump-scenario dart_a20r7_valley12 --simul-3way 1 --simul-strategies dart \
    --launch-mode air-impulse --air-impulse-pitch-deg "$P" --air-impulse-roll-deg 0 \
    --control-trace 0 --hud 0 --max-steps 1200 \
    --rolls 5 --paired-valid-target 5 --paired-max-attempts 15 || true
done

for V in 16 20; do
  run_cohort "probe_noseup_v${V}" \
    --jump-scenario dart_a20r7_valley12 --simul-3way 1 --simul-strategies dart \
    --launch-mode air-impulse --air-impulse-pitch-deg -25 --air-impulse-roll-deg 0 \
    --v-entry "$V" --control-trace 0 --hud 0 --max-steps 1400 \
    --rolls 12 --paired-valid-target 12 --paired-max-attempts 36 || true
done

run_cohort "appr_a24_companion" \
  --jump-scenario dart_a24r7_valley12 --simul-3way 1 --simul-strategies dart,rwpd,tobb \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --launch-mode approach --approach-ground-paused 1 \
  --approach-simul3-session-reuse 1 --approach-simul3-refresh-every 0 \
  --control-trace 0 --hud 0 --max-steps 1200 || true

for L in 0 10 20 40; do
  run_cohort "rob_latency_lat${L}" \
    --jump-scenario dart_a20r7_valley12 --multiroll-strategies dart,rwpd,tobb,dart_replicate \
    "${MRC[@]}" --air-impulse-pitch-deg 11.565 --air-impulse-roll-deg 28.124 \
    --takeoff-state-jitter-sigma 0.25 --takeoff-state-jitter-seed 20260703 \
    --actuator-latency-ms "$L" --rolls 120 --paired-valid-target 120 --paired-max-attempts 220 || true
done

log "QUEUE DONE"
