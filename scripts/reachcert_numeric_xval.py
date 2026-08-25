#!/usr/bin/env python3
"""LP cross-check of the Theorem 3 closed-form certificate (no BeamNG).

Ground truth: HiGHS LPs on the same budget-constrained double integrator.
If the closed form stays inside the LP interval it is sound. Paper anchor:
18 configs, coverage 89–96% (median 94%). Writes
``data/derived/reachcert_numeric_xval.json``.
"""
from __future__ import annotations

import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "data" / "derived" / "reachcert_numeric_xval.json"

# vehicle constants (= paper Section III / Fig. 3 parameter box)
I_YY = 2043.0          # kg m^2
I_W = 1.2              # kg m^2 per wheel
N_W = 4
R_WHEEL = 0.36         # m
OMEGA_MAX = 1200.0 * 2.0 * math.pi / 60.0   # rad/s (= 125.66)
OMEGA_MIN = -OMEGA_MAX                      # reverse-inclusive lower bound
A_PITCH = 135.0        # deg/s^2 symmetric pitch angular-acceleration bound

N_STEPS = 400          # LP discretization
E0_GRID = 61           # rate-error grid points per config
TOL_DEG = 0.30         # discretization tolerance for soundness check

def budgets_dps(v0: float) -> tuple[float, float]:
    """Directional body pitch-rate budgets (deg/s) at takeoff speed v0 (A5)."""
    w0 = v0 / R_WHEEL
    b_up = N_W * I_W * (OMEGA_MAX - w0) / I_YY
    b_dn = N_W * I_W * (w0 - OMEGA_MIN) / I_YY
    return math.degrees(b_up), math.degrees(b_dn)

def cert_interval(e0, T, a, b_up, b_dn, w_tgt):
    """Closed-form F_theta: admissible dtheta_req interval at rate error e0."""
    if not (-b_up <= e0 <= b_dn):
        return None
    t_r = abs(e0) / a
    if t_r > T:
        return None
    tau = T - t_r
    drift = w_tgt * T + e0 * abs(e0) / (2.0 * a)

    def cap(b_r):
        if b_r <= 0.0:
            return 0.0
        if tau < 2.0 * b_r / a:
            return 0.25 * a * tau * tau
        return b_r * tau - b_r * b_r / a

    d_pos = cap(b_up + e0)   # r_theta >= 0 branch
    d_neg = cap(b_dn - e0)   # r_theta < 0 branch
    return drift - d_neg, drift + d_pos

def lp_interval(e0, T, a, b_up, b_dn, w_tgt, n=N_STEPS):
    """Exact (discretized) reachable dtheta_req interval via 2 LPs.

    Work in q = omega - omega_target: q(0)=e0, q(T)=0,
    running constraint q(t) - e0 in [-b_dn, +b_up].
    displacement = w_tgt*T + integral(q dt)
    integral(q dt) = T*e0 + dt^2 * sum_k (n-1-k+0.5) * u_k
    """
    dt = T / n
    coef = dt * dt * (np.arange(n)[::-1] + 0.5)
    lower = np.tril(np.ones((n, n))) * dt          # cumulative rate change rows
    a_ub = np.vstack([lower, -lower])
    b_ub = np.concatenate([np.full(n, b_up), np.full(n, b_dn)])
    a_eq = np.full((1, n), dt)
    b_eq = np.array([-e0])                          # q(T) = 0
    bounds = [(-a, a)] * n
    ends = []
    for c in (coef, -coef):                         # minimize, then maximize
        res = linprog(c, A_ub=a_ub, b_ub=b_ub, A_eq=a_eq, b_eq=b_eq,
                      bounds=bounds, method="highs")
        if not res.success:
            return None
        ends.append(float(T * e0 + coef @ res.x))
    lo, hi = min(ends), max(ends)
    return w_tgt * T + lo, w_tgt * T + hi

def run_config(v0, T, w_tgt):
    a = A_PITCH
    b_up, b_dn = budgets_dps(v0)
    e_lo = max(-b_up, -a * T) + 1e-9
    e_hi = min(b_dn, a * T) - 1e-9
    e0s = np.linspace(e_lo, e_hi, E0_GRID)
    max_viol = 0.0
    len_cert = len_lp = 0.0
    n_pts = 0
    for e0 in e0s:
        ci = cert_interval(float(e0), T, a, b_up, b_dn, w_tgt)
        li = lp_interval(float(e0), T, a, b_up, b_dn, w_tgt)
        if ci is None and li is None:
            continue
        if ci is not None and li is None:
            max_viol = max(max_viol, 999.0)   # cert claims feasible, LP says no
            continue
        n_pts += 1
        len_lp += li[1] - li[0]
        if ci is None:
            continue
        len_cert += ci[1] - ci[0]
        max_viol = max(max_viol, li[0] - ci[0], ci[1] - li[1])
    coverage = 100.0 * len_cert / len_lp if len_lp > 0 else float("nan")
    return {
        "v0_mps": v0, "T_s": T, "omega_target_dps": w_tgt,
        "B_up_dps": round(b_up, 2), "B_down_dps": round(b_dn, 2),
        "n_rate_points": n_pts,
        "max_violation_deg": round(max_viol, 4),
        "sound": bool(max_viol <= TOL_DEG),
        "coverage_pct": round(coverage, 2),
    }

def main() -> int:
    configs = []
    for v0 in (10.5, 16.0, 20.0):
        for T in (0.95, 1.5, 2.5):
            for w_tgt in (0.0, -5.0):
                print(f"config v0={v0} T={T} w_tgt={w_tgt} ...", flush=True)
                configs.append(run_config(v0, T, w_tgt))
    covs = [c["coverage_pct"] for c in configs]
    agg = {
        "all_sound": bool(all(c["sound"] for c in configs)),
        "max_violation_deg": max(c["max_violation_deg"] for c in configs),
        "n_configs": len(configs),
        "coverage_min_pct": min(covs),
        "coverage_max_pct": max(covs),
        "coverage_median_pct": round(float(np.median(covs)), 2),
        "tolerance_deg": TOL_DEG,
    }
    out = {
        "_provenance": {
            "kind": "pure_numeric_lp_xval",
            "time_integrity_ok": True,
            "exempt_reason": "zero BeamNG; deterministic pure numerics",
            "generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "script": "scripts/reachcert_numeric_xval.py",
            "lp_steps": N_STEPS, "e0_grid": E0_GRID,
        },
        "model": {
            "I_yy": I_YY, "I_w": I_W, "n_w": N_W, "r_wheel": R_WHEEL,
            "omega_max_rpm": 1200, "omega_min": "reverse-inclusive (-omega_max)",
            "a_pitch_dps2": A_PITCH,
        },
        "aggregate": agg,
        "configs": configs,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(agg, indent=1))
    print(f"wrote {OUT}")
    return 0 if agg["all_sound"] else 1

if __name__ == "__main__":
    sys.exit(main())
