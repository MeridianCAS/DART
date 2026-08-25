# DART — Experiment Reproduction Package

Everything needed to **re-run the experiments** behind
[DART: Dual-Axis Airborne Reachability-Gated Torque-Reaction for Off-Road
Vehicle Jumps](https://arxiv.org/abs/2607.29011)
([arXiv:2607.29011](https://arxiv.org/abs/2607.29011) [cs.RO]).
The package ships the BeamNG experiment runner, cohort queue scripts,
jump scenarios, and vehicle configs. 

**Authors.** Yu Hu, Fangzhou Zhao, Mingyuan Sang, Chen Min, Liang Chen,
Wei Li, Wenyu Kuang, Shican Chen, Jinwei Li, and Baolei Chen.

**Affiliations.**
<sup>1</sup>Research Center for Intelligent Computing Systems, Institute
of Computing Technology, CAS;
<sup>2</sup>School of Computer Science and Technology, University of
Chinese Academy of Sciences;
<sup>3</sup>Dong Feng Off-Road Vehicle Co., Ltd.
Yu Hu<sup>1,2,*</sup>, Fangzhou Zhao<sup>1</sup>, Mingyuan
Sang<sup>1,2</sup>, Chen Min<sup>1</sup>, Liang Chen<sup>1</sup>, Wei
Li<sup>1,2</sup>, Wenyu Kuang<sup>3</sup>, Shican Chen<sup>3</sup>,
Jinwei Li<sup>3</sup>, Baolei Chen<sup>3</sup>.
Corresponding author: Yu Hu (`huyu@ict.ac.cn`).

**No pre-computed cohort data is included.** You run the experiments locally; each
output file is named to map directly onto the paper's tables and figures
(e.g. `data/cohorts/dart_bench_tab08_plat_ihi.json` → Table VIII,
high-inertia platform variant). The goal is **design-faithful reproduction** — same
geometry, controllers, protocol, and statistics — not byte-identical medians:
BeamNG soft-body physics carries session-level variation, so expect the paper's
orderings and significance patterns to reproduce rather than exact numbers.

All experiments use deterministic-stepping full-scale simulation
(BeamNG.tech 0.38.3, custom 4WIDS electric vehicle, curb mass ≈ 1383 kg).

## Layout

```
scripts/
 dart_bench.py Main runner: jump cohorts → data/cohorts/dart_bench_<TAG>.json
 reachcert_numeric_xval.py LP cross-validation of the Theorem 3 certificate (no BeamNG)
 experiments/*.sh Cohort queue scripts (see table below)
control/dart/ DART controller, baselines, reachability gate, phase logic
data_pipeline/beamng_session.py BeamNG attach / ready / freerun / teardown helpers
tests/ Unit tests for the reachability certificate, go/no-go gate, and LipMap
sim/scenarios/ Jump scenarios referenced by the queue scripts (JSON)
vehicles/sbr/ 4WIDS vehicle part configs (nominal + robustness variants)
data/cohorts/ Generated cohort JSONs (empty until you run)
data/derived/ Runner diagnostics
data/queue_runs/ Queue logs and DONE markers (resumable reruns)
```

## Requirements

- Windows with **BeamNG.tech 0.38.3** (research license; set `DART_BNG_HOME`)
- Python 3.11+, `pip install -r requirements.txt`
- Vehicle configs from `vehicles/sbr/` copied into the BeamNG user folder
  (`.../BeamNG.tech/current/vehicles/sbr/`)

Sanity check without BeamNG: `python -m unittest discover -s tests` exercises
the closed-form reachability certificate used by the go/no-go gate.

Numeric cross-validation without BeamNG:
`python scripts/reachcert_numeric_xval.py` reproduces the paper's LP
cross-validation of the Theorem 3 closed-form certificate (18 parameter
configurations, 400-step linear programs via `scipy` HiGHS). Expected result:
all configurations sound (no certified state outside the exact reachable set)
and coverage 89–96% (median 94%). Output:
`data/derived/reachcert_numeric_xval.json`.

## BeamNG session helper

`data_pipeline/beamng_session.py` is **not** the experiment runner. It is the
session-lifecycle layer that `scripts/dart_bench.py` imports so one BeamNG.tech
process can stay attached across a long cohort: connect, wait until the
scenario is actually running, resume a paused sim (`ensure_freerun`), and kill
stale processes that still hold the research port.

This file is what remained after the authors burned a large amount of wall-clock
time and debugging tokens on silent BeamNG failures: the scenario never leaving
`wait_scenario_running`, a vehicle that looks live but never moves, a freeze
at the instant of impact, an orphan process still bound to port 25252, and
overnight GPU throttle after the display sleeps. Do not replace it with a bare
`BeamNGpy` connect — those failure modes will return.

## Controller and cohort naming (paper-aligned)

| CLI / JSON name                                    | Paper role                                          |
| -------------------------------------------------- | --------------------------------------------------- |
| `dart`                                             | DART airborne law, always-on roll                   |
| `dart_latched`                                     | DART with the per-flight roll latch (full DART law) |
| `dart_replicate`                                   | Bit-identical DART replicate (same-law noise floor) |
| `rwpd`                                             | RW-PD baseline (reaction-wheel-style PD)            |
| `tobb`                                             | TOBB baseline (time-optimal bang-bang)              |
| `dart_dual` / `dart_pitch_only` / `dart_roll_only` | Per-axis ablation variants                          |

Historical bench keys (`c7` / `pd` / `mpc`) are accepted as CLI aliases via
`LEGACY_STRATEGY_ALIASES` in `scripts/dart_bench.py`.

**Paper DART = per-flight roll latch enabled.** Decisive and generalization
cells (Tab III–VIII, Fig. 8) therefore pass `--dart-roll-adaptive 1`, which
enables the latch on the `dart` leg. Cells the paper explicitly labels
always-on-roll (run-up surface, mass ladder, jitter/latency text cells, and
the authority/budget probes) keep the flag off. The latch-ablation cohorts
(`abl_latch_*`) compare `dart` vs. `dart_latched` directly.

Cohort tags use paper anchors: `tabNN_*`, `figNN_*`, `abl_*`, `rob_*`,
`probe_*`, `cond2_*`, `slg_*`, `appr_*`, `budgetin_*`. See the mapping table
below.

## Queue scripts (recommended run order)

Each script is resumable: cohorts run sequentially and skip tags that
already have a `.DONE` marker under `data/queue_runs/`. Run one queue script
at a time (single BeamNG instance).

| Script                             | Paper coverage (summary)                                                                                                                                                                            |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `queue_tab03_h2h_latency.sh`       | **Tab III** air-impulse head-to-head; actuator-latency sweep (Sec. V text); per-axis stressed ablation; **Fig. 5** pitch-authority ladder; nose-up budget probes; gentler α=24° approach companion  |
| `queue_tab04_gate_off.sh`          | **Tab IV** steep-lip approach head-to-head; **Fig. 8** end-to-end gate **off** arm                                                                                                                  |
| `queue_fig08_gate_on.sh`           | **Fig. 8** end-to-end gate **on** arm (final gate parameters)                                                                                                                                       |
| `queue_paper_tables.sh`            | **Tab V / Fig. 7** banked run-up sweep; **Tab VI / Fig. 9** geometric generalization; **Tab VII** gain sensitivity; **Tab VIII** platform variants; budget-interior cell; Condition-2; rate-tracking ablation; matched-geometry control |
| `queue_jitter_banked_probes.sh`    | Takeoff-state jitter (σ) sweep (Sec. V text); banked per-axis ablation                                                                                                                              |
| `queue_runup_latency.sh`           | Latency gradient at σ→0⁺ (Sec. V text); run-up surface robustness; flat-approach head-to-head; per-axis extensions                                                                                  |
| `queue_ablation_mass_peraxis.sh`   | Per-axis ablation at α=28°; approach mass ladder (±20%)                                                                                                                                             |
| `queue_ablation_latch_matrix.sh`   | Per-flight roll-latch ablation matrix (flat, α=28°, resonance, 1.2× mass)                                                                                                                           |
| `queue_ablation_latch_laneswap.sh` | Lane-swap confound control for latch ablation                                                                                                                                                       |

```bash
bash scripts/experiments/queue_tab03_h2h_latency.sh
bash scripts/experiments/queue_tab04_gate_off.sh
bash scripts/experiments/queue_fig08_gate_on.sh
bash scripts/experiments/queue_paper_tables.sh
bash scripts/experiments/queue_jitter_banked_probes.sh
bash scripts/experiments/queue_runup_latency.sh
bash scripts/experiments/queue_ablation_mass_peraxis.sh
bash scripts/experiments/queue_ablation_latch_matrix.sh
bash scripts/experiments/queue_ablation_latch_laneswap.sh
```

Single-cohort example:

```bash
DART_JUMP_DIR=sim/scenarios \
python scripts/dart_bench.py --jump-scenario dart_a20r7_valley12 \
 --simul-3way 1 --simul-strategies dart,dart_latched,rwpd \
 --launch-mode approach --rolls 30 --paired-valid-target 30 --tag MY_TAG
```

Output: `data/cohorts/dart_bench_MY_TAG.json` (plus `.raw.json` and optional
`.checkpoint.json` for resumable approach runs).

## Cohort file ↔ paper mapping

Every cohort file is `data/cohorts/dart_bench_<TAG>.json`.

| Paper item                                     | Primary queue script                              | Example tags                                                                                         |
| ---------------------------------------------- | ------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Tab III — air-impulse head-to-head             | `queue_tab03_h2h_latency`                         | `tab03_h2h_a20`, `tab03_h2h_a24`, `tab03_h2h_a28`                                                    |
| Tab IV / Fig. 6 — steep-lip approach           | `queue_tab04_gate_off`                            | `tab04_appr_h2h_steeplip`                                                                            |
| Tab V / Fig. 7 — cross-slope sweep             | `queue_paper_tables`                              | `tab05_banked_camber4`, `tab05_banked_camber8`, `tab05_banked_camber12`, `tab05_banked_camber8_gate0` |
| Tab VI / Fig. 9 — geometric generalization     | `queue_paper_tables`                              | `tab06_geomgen_appr_a10r7`, `tab06_geomgen_air_a14r7`, …                                             |
| Tab VII — controller gain sensitivity          | `queue_paper_tables`                              | `tab07_gain_nom`, `tab07_gain_kp0p5`, …                                                              |
| Tab VIII — platform robustness                 | `queue_paper_tables`                              | `tab08_plat_tq1p0`, `tab08_plat_tq0p6`, `tab08_plat_ihi`, …                                          |
| Fig. 5 — pitch-authority envelope              | `queue_tab03_h2h_latency`                         | `probe_pitch_envelope_p10` … `probe_pitch_envelope_p45`                                              |
| Fig. 8 — end-to-end gate                       | `queue_tab04_gate_off` + `queue_fig08_gate_on`    | `fig08_e2e_gate_off`, `fig08_e2e_gate_on`                                                            |
| Budget-interior takeoff cell (Sec. V)          | `queue_paper_tables`                              | `budgetin_a20`                                                                                       |
| Takeoff-state jitter, σ sweep (Sec. V text)    | `queue_jitter_banked_probes`                      | `rob_jitter_s1`, `rob_jitter_s5`, …                                                                  |
| Latency at σ→0⁺ (Sec. V text)                  | `queue_runup_latency`                             | `rob_jitter_s0b_lat0`, `rob_jitter_s0b_lat10`, …                                                     |
| Actuator-latency interleaved sweep (Sec. V)    | `queue_tab03_h2h_latency`                         | `rob_latency_lat0`, `rob_latency_lat10`, …                                                           |
| Per-axis ablation (stressed)                   | `queue_tab03_h2h_latency`                         | `abl_peraxis_stressed`                                                                               |
| Per-axis ablation (extensions)                 | several queues                                    | `abl_peraxis_air_a28`, `abl_peraxis_appr`, `abl_peraxis_jitter_lat20`, `abl_peraxis_banked_camber12` |
| Rate-tracking vs naive angle-PD                | `queue_paper_tables`                              | `abl_ratetrack_n30`, `abl_naivepd_n30`                                                               |
| Per-flight latch ablation                      | `queue_ablation_latch_matrix`                     | `abl_latch_flat_appr`, `abl_latch_a28_air`, …                                                        |
| Lane-swap confound control                     | `queue_ablation_latch_laneswap`                   | `abl_latch_laneswap`                                                                                 |
| Condition-2 overshoot                          | `queue_paper_tables`                              | `cond2_overshoot_air`                                                                                |
| Matched-geometry attribution control           | `queue_paper_tables`                              | `slg_airimp`, `slg_appr`                                                                             |
| Flat approach head-to-head                     | `queue_runup_latency`                             | `appr_flat_h2h`                                                                                      |
| Run-up surface robustness                      | `queue_runup_latency`                             | `rob_surface_asphalt`, `rob_surface_gravel`, `rob_surface_dirt`                                      |
| Mass ladder (approach)                         | `queue_ablation_mass_peraxis`                     | `rob_mass_m080_appr`, `rob_mass_m120_appr`                                                           |
| Nose-up budget probes                          | `queue_tab03_h2h_latency`                         | `probe_noseup_v16`, `probe_noseup_v20`                                                               |
| Gentler α=24° approach companion               | `queue_tab03_h2h_latency`                         | `appr_a24_companion`                                                                                 |

## Reproducibility

Cohorts use seed-controlled, deterministic physics stepping (fixed 1/sps step),
and same-tick pairing where the protocol allows. Decision-grade cells use N=30
per configuration; interleaved cells include a 30-jump same-law replicate whose
replicate gap defines the noise floor. Apply the paper's paired statistical
conventions (Section IV) within cohorts. Reference numbers in the published
paper come from the companion data package; this repository re-runs the same
experimental designs.

## License

This project is licensed under the [Apache License, Version 2.0](LICENSE).

## Citation

If you use this reproduction package or build on DART, please cite:

```bibtex
@misc{hu2026dart,
  title         = {DART: Dual-Axis Airborne Reachability-Gated Torque-Reaction
                   for Off-Road Vehicle Jumps},
  author        = {Yu Hu and Fangzhou Zhao and Mingyuan Sang and Chen Min and
                   Liang Chen and Wei Li and Wenyu Kuang and Shican Chen and
                   Jinwei Li and Baolei Chen},
  year          = {2026},
  eprint        = {2607.29011},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2607.29011},
}
```

Plain text:

> Yu Hu, Fangzhou Zhao, Mingyuan Sang, Chen Min, Liang Chen, Wei Li,
> Wenyu Kuang, Shican Chen, Jinwei Li, and Baolei Chen, "DART: Dual-Axis
> Airborne Reachability-Gated Torque-Reaction for Off-Road Vehicle Jumps,"
> arXiv:2607.29011, 2026.
