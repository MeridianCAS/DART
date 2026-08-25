"""Baseline airborne controllers (paper Section IV): RW-PD and TOBB.

Both act through the same actuator channel as DART (wheel throttle/brake
reaction torque plus counter-steer) so the head-to-head comparison isolates
the control law.

Shared conventions (as wired by dart_bench):
- Pitch error err = target_pitch_deg - pitch_d; u>0 -> throttle (nose-up),
  u<0 -> brake (nose-down).
- Roll counter-steer: steer = clamp(-k_roll*roll_d/30, +-smax).
- omega_cap: cut throttle when mean wheel speed exceeds cap*max(omega_tgt, 1)
  (wheel-speed saturation guard).
"""
from __future__ import annotations

def _counter_steer(roll_d, k_roll, smax):
    return max(-smax, min(smax, -k_roll * roll_d / 30.0))

def rwpd_airborne_ctrl(pitch_d, pdot, roll_d, target_pitch_deg, *, kp, kd, k_roll, smax,
                     omega_w=0.0, omega_tgt=0.0, omega_cap=0.0):
    """Baseline #1: reaction-wheel-style attitude PD on throttle/brake.

    u = kp*err/20 - kd*pdot/100; returns (thr, brk, steer)."""
    err = target_pitch_deg - pitch_d
    u = kp * err / 20.0 - kd * pdot / 100.0
    if u > 0 and omega_cap > 0 and omega_w > omega_cap * max(omega_tgt, 1.0):
        u = 0.0
    thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
    return thr, brk, _counter_steer(roll_d, k_roll, smax)

def tobb_airborne_ctrl(pitch_d, pdot, roll_d, target_pitch_deg, *, a_max_dps2, k_roll, smax,
                      pitch_db_deg=0.5, rate_db_dps=2.0, omega_w=0.0, omega_tgt=0.0, omega_cap=0.0):
    """Baseline #2: time-optimal bang-bang (TOBB) on the double integrator.

    Switching function s = e + edot*|edot|/(2*a_max) with e = pitch - target;
    u = -a_max*sign(s). u>0 -> throttle, u<0 -> brake; inside the deadband the
    controller coasts. Returns (thr, brk, steer)."""
    e = pitch_d - target_pitch_deg
    edot = pdot
    a = max(1e-6, a_max_dps2)
    if abs(e) <= pitch_db_deg and abs(edot) <= rate_db_dps:
        thr = brk = 0.0
    else:
        s = e + edot * abs(edot) / (2.0 * a)
        u_pos = (s <= 0)                      # s>0 -> brake (nose-down); s<=0 -> throttle
        if u_pos and omega_cap > 0 and omega_w > omega_cap * max(omega_tgt, 1.0):
            thr = brk = 0.0
        else:
            thr, brk = (1.0, 0.0) if u_pos else (0.0, 1.0)
    return thr, brk, _counter_steer(roll_d, k_roll, smax)
