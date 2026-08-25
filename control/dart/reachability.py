"""Closed-form pitch reachability (Theorems 1 and 3) and ``v_max`` shaping."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

@dataclass(frozen=True)
class PitchReachability:
    """Pitch-axis reachability result and diagnostic margins."""

    reachable: bool
    velocity_nullable: bool
    displacement_ok: bool
    budget_ok: bool
    e: float
    s: float
    T_r: float
    r_theta: float
    T_c: float
    B_r: float
    margin: float

def pitch_accel_limit(tau_max: float, J_y: float, n_wheels: int = 4) -> float:
    """Return the pitch-acceleration limit ``n * tau_max / J_y``."""
    if J_y <= 0:
        raise ValueError("J_y must be > 0")
    return n_wheels * tau_max / J_y

def angular_momentum_budget(
    I_w: float,
    omega_max: float,
    omega_now: float | list[float] | tuple[float, ...],
    n_wheels: int = 4,
) -> float:
    """Return the legacy drive-direction momentum budget in kg m^2/s."""
    if isinstance(omega_now, (list, tuple)):
        return sum(max(0.0, I_w * (omega_max - float(w))) for w in omega_now)
    return max(0.0, n_wheels * I_w * (omega_max - float(omega_now)))

def directional_rate_budgets(
    I_w: float,
    omega_max: float,
    omega_min: float,
    omega_now: float | list[float] | tuple[float, ...],
    J_y: float,
    n_wheels: int = 4,
) -> tuple[float, float]:
    """Return ``(B_up, B_down)`` in body-rate units (rad/s).

    ``B_up`` uses remaining drive-direction wheel-speed capacity.
    ``B_down`` uses braking capacity, including reversal when
    ``omega_min < 0``.
    """
    if J_y <= 0:
        raise ValueError("J_y must be > 0")
    if isinstance(omega_now, (list, tuple)):
        h_up = sum(max(0.0, I_w * (omega_max - float(w))) for w in omega_now)
        h_down = sum(max(0.0, I_w * (float(w) - omega_min)) for w in omega_now)
    else:
        w = float(omega_now)
        h_up = max(0.0, n_wheels * I_w * (omega_max - w))
        h_down = max(0.0, n_wheels * I_w * (w - omega_min))
    return h_up / J_y, h_down / J_y

def displacement_capacity(T_c: float, B_r: float, a: float) -> float:
    """Return the terminal-rate-zero displacement capacity ``D(T_c, B_r)``."""
    if a <= 0:
        raise ValueError("a must be > 0")
    if T_c <= 0.0 or B_r <= 0.0:
        return 0.0
    if math.isinf(B_r) or T_c < 2.0 * B_r / a:
        return 0.25 * a * T_c * T_c
    return B_r * T_c - B_r * B_r / a

def pitch_reachable(
    theta0: float,
    omega_y0: float,
    theta_L: float,
    T: float,
    a: float,
    *,
    H_max: Optional[float] = None,
    J_y: Optional[float] = None,
    B_up: Optional[float] = None,
    B_down: Optional[float] = None,
    omega_target: float = 0.0,
    omega_bar: float = 0.0,
) -> PitchReachability:
    """Evaluate the constructive pitch-axis reachability certificate.

    Explicit ``B_up`` and ``B_down`` take precedence over the legacy
    symmetric budget ``H_max / J_y``. Omitting both leaves only the time
    and displacement constraints. ``omega_bar > 0`` switches the rate
    condition to the Theorem 1 window
    ``-(B_up + omega_bar) <= e0 <= B_down + omega_bar`` and uses the
    null-to-window plus residual-drift construction.
    """
    if T <= 0:
        raise ValueError("T (flight time) must be > 0")
    if a <= 0:
        raise ValueError("a (pitch accel limit) must be > 0")
    if omega_bar < 0:
        raise ValueError("omega_bar must be >= 0")

    e = theta0 - theta_L + omega_target * T
    s = omega_y0 - omega_target

    if B_up is not None and B_down is not None:
        b_up = max(0.0, float(B_up))
        b_dn = max(0.0, float(B_down))
    elif H_max is not None and J_y is not None and J_y > 0:
        b_sym = max(0.0, float(H_max) / float(J_y))
        b_up = b_dn = b_sym
    else:
        b_up = b_dn = float("inf")

    budget_ok = (-(b_up + omega_bar) - 1e-12) <= s <= (b_dn + omega_bar + 1e-12)

    b_dir = b_dn if s > 0.0 else b_up
    if abs(s) <= omega_bar + 1e-15:
        reduce = 0.0
    elif math.isinf(b_dir) or abs(s) <= b_dir:
        reduce = abs(s)
    else:
        reduce = min(abs(s), max(b_dir, abs(s) - omega_bar))
    r_w = math.copysign(abs(s) - reduce, s) if s != 0.0 else 0.0
    T_r = reduce / a
    velocity_nullable = T_r <= T + 1e-12
    T_c = max(0.0, T - T_r)

    null_disp = (
        math.copysign((s * s - r_w * r_w) / (2.0 * a), s) if s != 0.0 else 0.0
    )
    r_theta = e + null_disp + r_w * T_c

    if math.isinf(b_up):
        B_r = float("inf")
    else:
        if s > 0.0:
            b_up_after, b_dn_after = b_up + reduce, b_dn - reduce
        elif s < 0.0:
            b_up_after, b_dn_after = b_up - reduce, b_dn + reduce
        else:
            b_up_after, b_dn_after = b_up, b_dn
        B_r = max(0.0, b_dn_after if r_theta >= 0.0 else b_up_after)

    if not velocity_nullable:
        displacement_ok = False
        cap = 0.0
    else:
        cap = displacement_capacity(T_c, B_r, a)
        displacement_ok = abs(r_theta) <= cap + 1e-12
    margin = cap - abs(r_theta)
    reachable = velocity_nullable and displacement_ok and budget_ok

    return PitchReachability(
        reachable=reachable,
        velocity_nullable=velocity_nullable,
        displacement_ok=displacement_ok,
        budget_ok=budget_ok,
        e=e,
        s=s,
        T_r=T_r,
        r_theta=r_theta,
        T_c=T_c,
        B_r=B_r,
        margin=margin,
    )

def vmax_cross_section(v_crit: float, a_brake: float, d: float) -> float:
    """Return ``sqrt(v_crit^2 + 2 * a_brake * d)``."""
    if a_brake < 0 or d < 0 or v_crit < 0:
        raise ValueError("v_crit, a_brake, d must be >= 0")
    return math.sqrt(v_crit * v_crit + 2.0 * a_brake * d)

@dataclass(frozen=True)
class TakeoffReachableSet:
    """Query wrapper for a fixed pitch-axis takeoff certificate."""

    theta_L: float
    T: float
    a: float
    H_max: Optional[float] = None
    J_y: Optional[float] = None
    B_up: Optional[float] = None
    B_down: Optional[float] = None
    omega_bar: float = 0.0

    def contains(self, theta0: float, omega_y0: float) -> bool:
        return self.query(theta0, omega_y0).reachable

    def query(self, theta0: float, omega_y0: float) -> PitchReachability:
        return pitch_reachable(
            theta0,
            omega_y0,
            self.theta_L,
            self.T,
            self.a,
            H_max=self.H_max,
            J_y=self.J_y,
            B_up=self.B_up,
            B_down=self.B_down,
            omega_bar=self.omega_bar,
        )

    def omega_bound(self) -> float:
        """Return a conservative scalar takeoff-rate bound."""
        bound = self.a * self.T
        if self.B_up is not None and self.B_down is not None:
            bound = min(bound, max(0.0, min(self.B_up, self.B_down)))
        elif self.H_max is not None and self.J_y is not None and self.J_y > 0:
            bound = min(bound, self.H_max / self.J_y)
        return bound

    def is_empty(self) -> bool:
        """Return whether even the aligned, zero-rate takeoff state fails."""
        return not self.contains(self.theta_L, 0.0)

__all__ = [
    "PitchReachability",
    "TakeoffReachableSet",
    "pitch_accel_limit",
    "angular_momentum_budget",
    "directional_rate_budgets",
    "displacement_capacity",
    "pitch_reachable",
    "vmax_cross_section",
]
