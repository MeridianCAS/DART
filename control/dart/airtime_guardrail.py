"""Airtime-based favorability guardrail for the DART advantage.

Calibration: a within-angle regression over N=202 jumps found that longer
airtime favors DART (beta=+0.42) while larger takeoff pitch theta0 works
against it (beta=-0.41); |omega_y0| was not significant. Intuition: airtime
is the correction window, and a large theta0 consumes it.

Favorability model (standardized within-angle coefficients):
    favorability = B_AIR*z(airtime) + B_TH*z(theta0),  z(x) = (x-mean)/std
    favorability > 0  <=>  the (airtime, theta0) pair favors DART
Zero-crossing forms:
    theta0_cap(airtime) = TH_MEAN + SLOPE*(airtime - AIR_MEAN)
    min_airtime(theta0) = AIR_MEAN + (theta0 - TH_MEAN)/SLOPE
"""
from __future__ import annotations

AIR_MEAN, AIR_STD = 1.1871, 0.2979      # airtime (s)
TH_MEAN, TH_STD = 4.1906, 5.1475        # theta0 (deg)
B_AIR, B_TH = 0.422, -0.413             # within-angle  (C2 )
CALIB_N = 202
CALIB_SRC = "lsgrid_N202_within_angle_2026-06-20"

# Slope of theta0_cap vs airtime (deg per s), from the favorability=0 locus:
#   B_AIR*(airtime-ā)/σ_a = -B_TH*(theta0-θ̄)/σ_θ
#   => theta0_cap = θ̄ + [B_AIR*σ_θ / (-B_TH*σ_a)] * (airtime - ā)
_SLOPE = (B_AIR * TH_STD) / (-B_TH * AIR_STD)   # ≈ 17.66 deg per s

def favorability_index(airtime_s: float, theta0_deg: float) -> float:
    """Standardized favorability index; >0 means the takeoff state favors DART."""
    za = (float(airtime_s) - AIR_MEAN) / AIR_STD
    zt = (float(theta0_deg) - TH_MEAN) / TH_STD
    return B_AIR * za + B_TH * zt

def theta0_cap_deg(airtime_s: float) -> float:
    """Largest theta0 (deg) still favorable at the given airtime."""
    return TH_MEAN + _SLOPE * (float(airtime_s) - AIR_MEAN)

def min_airtime_for_theta0(theta0_deg: float) -> float:
    """Shortest airtime (s) still favorable at the given theta0."""
    return AIR_MEAN + (float(theta0_deg) - TH_MEAN) / _SLOPE

def evaluate(airtime_s: float, theta0_deg: float, *, warn_margin: float = 0.0) -> dict:
    """Score a measured (airtime, theta0) pair against the guardrail.

    warn_margin: require favorability >= this value for an ok verdict
    (>0 tightens the gate). Returns a dict suitable for the cohort report.
    """
    airtime_s = float(airtime_s)
    theta0_deg = float(theta0_deg)
    idx = favorability_index(airtime_s, theta0_deg)
    cap = theta0_cap_deg(airtime_s)
    need_air = min_airtime_for_theta0(theta0_deg)
    ok = idx >= warn_margin
    if ok:
        verdict = "FAVORABLE"
        rec = (f"theta0={theta0_deg:.1f} deg <= cap {cap:.1f} deg at airtime {airtime_s:.2f} s; "
               f"correction window sufficient, state favors DART")
    else:
        verdict = "ADVERSE"
        rec = (f"theta0={theta0_deg:.1f} deg > cap {cap:.1f} deg at airtime {airtime_s:.2f} s: "
               f"raise airtime to >= {need_air:.2f} s (lip geometry, launch offset, or v0) "
               f"or reduce theta0 to <= {cap:.1f} deg")
    return {
        "verdict": verdict,
        "ok": bool(ok),
        "favorability_index": round(idx, 3),
        "airtime_s": round(airtime_s, 3),
        "theta0_deg": round(theta0_deg, 2),
        "theta0_cap_deg": round(cap, 2),
        "min_airtime_for_theta0_s": round(need_air, 3),
        "warn_margin": warn_margin,
        "recommendation": rec,
        "calib": {
            "B_air": B_AIR, "B_theta0": B_TH,
            "air_mean": AIR_MEAN, "air_std": AIR_STD,
            "theta0_mean": TH_MEAN, "theta0_std": TH_STD,
            "slope_deg_per_s": round(_SLOPE, 3),
            "n": CALIB_N, "src": CALIB_SRC,
        },
    }
