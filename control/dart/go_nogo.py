"""Pre-takeoff go/no-go gate. Default OFF; enable with ``--reachability-gate 1``."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from control.dart.reachability import TakeoffReachableSet, pitch_reachable, vmax_cross_section

class GoNoGo(Enum):
    INACTIVE = 0
    GO = 1
    NOGO = 2

class SafetyAction(Enum):
    NONE = 0
    DECELERATE = 1
    ABORT_JUMP = 2

@dataclass(frozen=True)
class GoNoGoDecision:
    decision: GoNoGo
    action: SafetyAction
    reason: str
    v_max: Optional[float] = None
    recommended_target_speed: Optional[float] = None

@dataclass
class GoNoGoGate:
    """Hard pre-takeoff go/no-go gate.

    Parameters
    ----------
    reachable_set : Fixed pitch-axis takeoff certificate.
    v_crit : Critical takeoff speed (m/s).
    a_brake : Available run-up deceleration (m/s^2).
    enabled : Opt-in switch. Disabled gates always return ``INACTIVE``.
    speed_margin : Additional margin subtracted from ``v_max`` (m/s).
    """

    reachable_set: TakeoffReachableSet
    v_crit: float
    a_brake: float
    enabled: bool = False
    speed_margin: float = 0.0

    def evaluate(
        self,
        current_speed: float,
        distance_to_lip: float,
        theta0_pred: float,
        omega_y0_pred: float,
    ) -> GoNoGoDecision:
        """Evaluate the gate for one control tick.

        Parameters
        ----------
        current_speed : Current speed (m/s).
        distance_to_lip : Remaining braking distance (m).
        theta0_pred, omega_y0_pred : Predicted takeoff pitch and rate.
        """
        if not self.enabled:
            return GoNoGoDecision(
                GoNoGo.INACTIVE, SafetyAction.NONE,
                reason="gate_disabled(opt-in default OFF)",
            )

        if self.reachable_set.is_empty():
            return GoNoGoDecision(
                GoNoGo.NOGO, SafetyAction.ABORT_JUMP,
                reason="R_takeoff_empty",
            )

        v_max = vmax_cross_section(self.v_crit, self.a_brake, max(0.0, distance_to_lip))
        v_max_eff = max(0.0, v_max - self.speed_margin)

        attitude_reachable = self.reachable_set.contains(theta0_pred, omega_y0_pred)
        speed_ok = current_speed <= v_max_eff + 1e-9

        if attitude_reachable and speed_ok:
            return GoNoGoDecision(
                GoNoGo.GO, SafetyAction.NONE,
                reason="reachable", v_max=v_max_eff,
            )

        return GoNoGoDecision(
            GoNoGo.NOGO,
            SafetyAction.DECELERATE,
            reason=(
                "speed_over_vmax" if not speed_ok else "attitude_unreachable"
            ),
            v_max=v_max_eff,
            recommended_target_speed=v_max_eff,
        )

@dataclass(frozen=True)
class CertifiedSpeedWindow:
    """One-dimensional scan of certified takeoff speeds."""

    intervals: tuple
    v_target: Optional[float]
    binding_constraint: str
    per_v: tuple

    @property
    def empty(self) -> bool:
        return self.v_target is None

    def contains(self, v: float, tol: float = 0.26) -> bool:
        return any(lo - tol <= v <= hi + tol for lo, hi in self.intervals)

def certified_speed_window(
    predict,
    *,
    theta_L: float,
    a_pitch: float,
    v_lo: float,
    v_hi: float,
    step: float = 0.5,
    v_cap: Optional[float] = None,
    budgets_fn=None,
    omega_target: float = 0.0,
    omega_bar: float = 0.0,
) -> CertifiedSpeedWindow:
    """Scan ``V_cert`` over a speed grid (paper certified-speed set).

    ``predict(v)`` must return ``(theta0_rad, omega_rad_s, T_s)``.
    """
    per_v = []
    reasons = {"speed": 0, "rate": 0, "displacement": 0}
    if v_hi < v_lo:
        return CertifiedSpeedWindow((), None, "empty_domain", ())
    n = max(1, int(round((v_hi - v_lo) / step)) + 1)
    for i in range(n):
        v = min(v_hi, v_lo + i * step)
        if v_cap is not None and v > v_cap + 1e-9:
            per_v.append((round(v, 2), False, "speed"))
            reasons["speed"] += 1
            continue
        th0, om, T = predict(v)
        kw = {}
        if budgets_fn is not None:
            b_up, b_dn = budgets_fn(v)
            kw = {"B_up": b_up, "B_down": b_dn}
        result = pitch_reachable(
            th0, om, theta_L, max(0.05, float(T)), a_pitch,
            omega_target=omega_target, omega_bar=omega_bar, **kw,
        )
        if result.reachable:
            per_v.append((round(v, 2), True, "ok"))
        else:
            binding = (
                "rate"
                if (not result.velocity_nullable or not result.budget_ok)
                else "displacement"
            )
            per_v.append((round(v, 2), False, binding))
            reasons[binding] += 1

    intervals = []
    cur_lo = None
    prev_v = None
    for v, ok, _binding in per_v:
        if ok and cur_lo is None:
            cur_lo = v
        elif not ok and cur_lo is not None:
            intervals.append((cur_lo, prev_v))
            cur_lo = None
        prev_v = v
    if cur_lo is not None:
        intervals.append((cur_lo, prev_v))

    if intervals:
        v_target = intervals[-1][1]
        binding = "ok"
    else:
        v_target = None
        binding = max(reasons, key=reasons.get) if any(reasons.values()) else "empty_domain"
    return CertifiedSpeedWindow(tuple(intervals), v_target, binding, tuple(per_v))

__all__ = [
    "GoNoGo",
    "SafetyAction",
    "GoNoGoDecision",
    "GoNoGoGate",
    "CertifiedSpeedWindow",
    "certified_speed_window",
]
