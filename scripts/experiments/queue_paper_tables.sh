#!/usr/bin/env bash
# Tabs V–VIII plus budget-interior, Condition-2, and matched-geometry cells.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/_queue_common.sh"

log "QUEUE START: paper_tables"

run_cohort "tab07_gain_nom" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kp0p5" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --diff-pitch-rate-kp 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kp2x" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --diff-pitch-rate-kp 4.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kphi0p5" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 0.5 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kphi2x" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 2.0 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kdrive0p5" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --diff-k-drive 0.2 --land-match-z 1.5 \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps \
  1400 --dart-roll-adaptive 1 || true

run_cohort "tab07_gain_kdrive2x" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --landing-slope-mode gap \
  --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --diff-k-drive 0.8 --land-match-z 1.5 \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps \
  1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_tq1p0" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_tq1p0.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_tq0p6" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_tq0p6.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_tq1p4" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_tq1p4.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_mulo" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_mulo.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_muhi" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_muhi.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_cogfront" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc \
  vehicles/sbr/dart_4motor_cogfront.pc --landing-slope-mode gap --valley-floor-run 35.0 \
  --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 --dart-pitch-control \
  differential --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 --paired-valid-target 30 \
  --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_ihi" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_ihi.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab08_plat_ilo" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 --cond-jitter-seed \
  20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 8.462 --air-impulse-roll-deg 31.31 --pc vehicles/sbr/dart_4motor_ilo.pc \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 \
  --landing-slope-len 150.0 --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z \
  1.5 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 \
  --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_appr_a10r7" \
  --angles 10 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --cond-jitter 21 --cond-jitter-seed 20260621 \
  --cond-jitter-v-hi 18.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-hi 12.0 --run-up 50.0 \
  --landing-slope-mode gap --ballistic-gamma-deg 7.0 --landing-slope-len 130.0 --dart-pitch-control \
  differential --diff-roll-steer-gain 1.0 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_appr_a14r7" \
  --angles 14 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --cond-jitter 21 --cond-jitter-seed 20260621 \
  --cond-jitter-v-hi 18.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-hi 12.0 --run-up 50.0 \
  --landing-slope-mode gap --ballistic-gamma-deg 7.0 --landing-slope-len 130.0 --dart-pitch-control \
  differential --diff-roll-steer-gain 1.0 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_appr_a22r7" \
  --angles 22 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --cond-jitter 21 --cond-jitter-seed 20260621 \
  --cond-jitter-v-hi 18.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-hi 12.0 --run-up 50.0 \
  --landing-slope-mode gap --ballistic-gamma-deg 7.0 --landing-slope-len 130.0 --dart-pitch-control \
  differential --diff-roll-steer-gain 1.0 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_appr_a14r13" \
  --angles 14 --rise 13.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --cond-jitter 21 --cond-jitter-seed 20260621 \
  --cond-jitter-v-hi 18.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-hi 12.0 --run-up 50.0 \
  --landing-slope-mode gap --ballistic-gamma-deg 7.0 --landing-slope-len 130.0 --dart-pitch-control \
  differential --diff-roll-steer-gain 1.0 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_air_a14r4" \
  --angles 14 --rise 4.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 21 \
  --cond-jitter-seed 20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 \
  --cond-jitter-roll-lo 12.0 --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 \
  --lip-launch-z-offset 2.5 --air-impulse-pitch-deg 7.974 --air-impulse-roll-deg 21.242 \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_air_a14r7" \
  --angles 14 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 21 \
  --cond-jitter-seed 20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 \
  --cond-jitter-roll-lo 12.0 --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 \
  --lip-launch-z-offset 2.5 --air-impulse-pitch-deg 7.974 --air-impulse-roll-deg 21.242 \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab06_geomgen_air_a14r13" \
  --angles 14 --rise 13.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 21 \
  --cond-jitter-seed 20260621 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 \
  --cond-jitter-roll-lo 12.0 --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 \
  --lip-launch-z-offset 2.5 --air-impulse-pitch-deg 7.974 --air-impulse-roll-deg 21.242 \
  --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab05_banked_camber8" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power-m 5.0 --simul-3way 1 \
  --simul-layout y_copy --simul-copy-spacing 45.0 --simul-strategies dart,rwpd,tobb \
  --reachability-gate 1 --gate-v-crit 11.0 --gate-coast-m 1.5 --gate-adaptive-abrake 1 \
  --gate-lip-power-recover 1 --width 12.0 --run-up 50.0 --v-entry 13.0 --spawn-v 0.0 \
  --landing-slope-deg -12.0 --landing-slope-mode valley --valley-auto-rise 0 --ballistic-v0 13.0 \
  --landing-slope-len 0.0 --runup-camber-deg 8.0 --camber-air-early-landmatch 0 \
  --camber-air-touch-roll-boost 1.0 --camber-air-touch-roll-gain 1.0 --camber-air-pred-horizon-sec 0.0 \
  --dart-pitch-control differential --gate-lip-launch-target 11.0 --approach-simul3-refresh-every 10 \
  --approach-simul3-session-reuse 1 --jump-scenario dart_a20r7_camber8 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab05_banked_camber8_gate0" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-layout y_copy --simul-copy-spacing 45.0 --simul-strategies dart,rwpd,tobb \
  --width 12.0 --run-up 50.0 --v-entry 13.0 --spawn-v 0.0 --landing-slope-deg -12.0 \
  --landing-slope-mode valley --valley-auto-rise 0 --ballistic-v0 13.0 --landing-slope-len 0.0 \
  --runup-camber-deg 8.0 --camber-air-early-landmatch 0 --camber-air-touch-roll-boost 1.0 \
  --camber-air-touch-roll-gain 1.0 --camber-air-pred-horizon-sec 0.0 --dart-pitch-control differential \
  --approach-simul3-session-reuse 1 --jump-scenario dart_a20r7_camber8 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab05_banked_camber4" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-layout y_copy --simul-copy-spacing 45.0 --simul-strategies dart,rwpd,tobb \
  --reachability-gate 1 --gate-v-crit 11.0 --gate-coast-m 1.5 --gate-adaptive-abrake 1 \
  --gate-lip-power-recover 1 --width 12.0 --run-up 50.0 --v-entry 13.0 --spawn-v 0.0 \
  --landing-slope-deg -12.0 --landing-slope-mode valley --valley-auto-rise 0 --ballistic-v0 13.0 \
  --landing-slope-len 0.0 --runup-camber-deg 4.0 --camber-air-early-landmatch 0 \
  --camber-air-touch-roll-boost 1.0 --camber-air-touch-roll-gain 1.0 --camber-air-pred-horizon-sec 0.0 \
  --dart-pitch-control differential --gate-lip-launch-target 11.0 --approach-simul3-refresh-every 5 \
  --approach-simul3-session-reuse 1 --jump-scenario dart_a20r7_camber4 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "tab05_banked_camber12" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-layout y_copy --simul-copy-spacing 45.0 --simul-strategies dart,rwpd,tobb \
  --reachability-gate 1 --gate-v-crit 11.0 --gate-coast-m 1.5 --gate-adaptive-abrake 1 \
  --gate-lip-power-recover 1 --width 12.0 --run-up 50.0 --v-entry 13.0 --spawn-v 0.0 \
  --landing-slope-deg -12.0 --landing-slope-mode valley --valley-auto-rise 0 --ballistic-v0 13.0 \
  --landing-slope-len 0.0 --runup-camber-deg 12.0 --camber-air-early-landmatch 0 \
  --camber-air-touch-roll-boost 1.0 --camber-air-touch-roll-gain 1.0 --camber-air-pred-horizon-sec 0.0 \
  --dart-pitch-control differential --gate-lip-launch-target 11.0 --approach-simul3-refresh-every 5 \
  --approach-simul3-session-reuse 1 --jump-scenario dart_a20r7_camber12 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "cond2_overshoot_air" \
  --angles 20 --rise 14.25 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,rwpd,tobb --run-up 50.0 --v-entry 13.0 --spawn-v 0.0 \
  --launch-mode air-impulse --air-impulse-pitch-deg 12.0 --air-impulse-roll-deg 28.0 \
  --landing-slope-deg -24.0 --landing-slope-mode valley --ballistic-v0 13.0 --landing-slope-len 0.0 \
  --approach-simul3-refresh-every 10 --jump-scenario dart_a20r7_valley24 --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 || true

run_cohort "abl_ratetrack_n30" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 30 --cond-jitter-seed \
  20260623 --cond-jitter-v-hi 19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 \
  --cond-jitter-roll-hi 36.0 --launch-mode air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 \
  --air-impulse-pitch-deg 10.077 --air-impulse-roll-deg 32.986 --postland-hold-sec 0.5 \
  --postfail-hold-sec 0.3 --landing-slope-mode gap --valley-floor-run 35.0 --ballistic-v0 13.0 \
  --ballistic-clearance 0.4 --landing-slope-len 150.0 --dart-pitch-control differential \
  --diff-roll-steer-gain 1.0 --land-match-z 1.5 --rolls 30 --paired-valid-target 30 \
  --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 || true

run_cohort "abl_naivepd_n30" \
  --angles 24 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 --simul-3way 1 \
  --simul-strategies dart,tobb,rwpd --cond-jitter 30 --cond-jitter-seed 20260623 --cond-jitter-v-hi \
  19.0 --cond-jitter-pitch-hi 12.0 --cond-jitter-roll-lo 28.0 --cond-jitter-roll-hi 36.0 --launch-mode \
  air-impulse --lip-launch-m 2.0 --lip-launch-z-offset 2.5 --air-impulse-pitch-deg 10.077 \
  --air-impulse-roll-deg 32.986 --postland-hold-sec 0.5 --postfail-hold-sec 0.3 --landing-slope-mode \
  gap --valley-floor-run 35.0 --ballistic-v0 13.0 --ballistic-clearance 0.4 --landing-slope-len 150.0 \
  --dart-pitch-control differential --diff-roll-steer-gain 1.0 --diff-pitch-naive 1 --land-match-z 1.5 \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps \
  1400 || true

run_cohort "slg_airimp" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --dart-airtime-guardrail 1 --cond-jitter 12 \
  --cond-jitter-seed 20260621 --cond-jitter-v-lo 14.0 --cond-jitter-v-hi 18.0 --cond-jitter-pitch-lo \
  12.0 --cond-jitter-pitch-hi 16.0 --launch-mode air-impulse --air-impulse-pitch-deg 13.642 \
  --air-impulse-roll-deg 10.965 --landing-slope-mode gap --ballistic-gamma-deg 7.0 \
  --ballistic-clearance 0.4 --landing-slope-len 130.0 --dart-pitch-control differential \
  --diff-roll-steer-gain 1.0 --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "slg_appr" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,rwpd,tobb --cond-jitter 21 --cond-jitter-v-hi 18.0 --run-up \
  50.0 --v-entry 15.0 --landing-slope-mode gap --ballistic-gamma-deg 7.0 --ballistic-clearance 0.4 \
  --landing-slope-len 130.0 --kp-pitch 1.5 --kd-pitch 4.0 --dart-pitch-control differential --rolls 30 \
  --paired-valid-target 30 --paired-max-attempts 90 --control-trace 0 --hud 0 --max-steps 1400 --dart-roll-adaptive 1 || true

run_cohort "budgetin_a20" \
  --angles 20 --rise 7.0 --ramp-mode kicker --lip-radius 35.0 --lip-power 1 --lip-power-m 5.0 \
  --simul-3way 1 --simul-strategies dart,tobb,rwpd --run-up 50.0 --v-entry 13.0 \
  --launch-mode air-impulse --lip-launch-m 4.0 --lip-launch-z-offset 0.65 \
  --air-impulse-pitch-deg 10.0 --air-impulse-roll-deg 0 --air-impulse-pitch-rate-dps 23.0 \
  --landing-slope-mode valley --landing-slope-deg -12.0 --valley-floor-run 45.0 \
  --ballistic-v0 13.0 --ballistic-clearance 0.5 --land-match-z 2.5 \
  --rolls 30 --paired-valid-target 30 --paired-max-attempts 90 \
  --control-trace 0 --hud 0 --max-steps 1200 || true

log "QUEUE DONE"
