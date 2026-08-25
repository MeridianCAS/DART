"""Regression tests for the closed-form DART reachability certificate."""
from __future__ import annotations

import math
import unittest

from control.dart.reachability import (
    directional_rate_budgets,
    displacement_capacity,
    pitch_reachable,
)

class ReachabilityTests(unittest.TestCase):
    def test_unbounded_displacement_capacity_has_quarter_factor(self) -> None:
        self.assertAlmostEqual(displacement_capacity(2.0, math.inf, 3.0), 3.0)

    def test_trapezoidal_displacement_capacity(self) -> None:
        self.assertAlmostEqual(displacement_capacity(3.0, 1.0, 2.0), 2.5)

    def test_negative_rate_nulling_uses_signed_displacement(self) -> None:
        result = pitch_reachable(
            theta0=0.5,
            omega_y0=-1.0,
            theta_L=0.0,
            T=2.0,
            a=2.0,
        )
        self.assertAlmostEqual(result.r_theta, 0.25)
        self.assertTrue(result.reachable)

    def test_directional_budget_rejects_only_exhausted_direction(self) -> None:
        nose_down = pitch_reachable(
            0.0, -1.5, 0.0, 2.0, 2.0, B_up=1.0, B_down=3.0
        )
        nose_up = pitch_reachable(
            0.0, 1.5, 0.0, 2.0, 2.0, B_up=1.0, B_down=3.0
        )
        self.assertFalse(nose_down.budget_ok)
        self.assertTrue(nose_up.budget_ok)

    def test_directional_budget_uses_per_wheel_speed(self) -> None:
        b_up, b_down = directional_rate_budgets(
            I_w=1.0,
            omega_max=10.0,
            omega_min=-10.0,
            omega_now=[2.0, 3.0, 4.0, 5.0],
            J_y=2.0,
        )
        self.assertAlmostEqual(b_up, 13.0)
        self.assertAlmostEqual(b_down, 27.0)

    def test_omega_target_constant_rate_approach_reaches_target(self) -> None:
        # Regression lock: the displacement error must carry the
        # omega_target*T term of the paper's r_theta. A state on the
        # constant-rate approach trajectory (theta0 = theta_L - w_t*T,
        # omega0 = w_t) reaches the terminal pair with zero demand.
        w_t, flight = 0.2, 1.5
        result = pitch_reachable(
            -w_t * flight, w_t, 0.0, flight, 2.0,
            B_up=0.4, B_down=0.4, omega_target=w_t,
        )
        self.assertAlmostEqual(result.r_theta, 0.0)
        self.assertTrue(result.reachable)

    def test_omega_target_shifts_displacement_demand(self) -> None:
        # With theta0 = theta_L and omega0 = omega_target, the demand is
        # omega_target*T (an implementation missing that term reports 0).
        w_t, flight = 0.2, 1.5
        result = pitch_reachable(
            0.0, w_t, 0.0, flight, 2.0,
            B_up=0.4, B_down=0.4, omega_target=w_t,
        )
        self.assertAlmostEqual(result.r_theta, w_t * flight)
        self.assertTrue(result.reachable)

    def test_omega_bar_zero_matches_unspecified(self) -> None:
        a = pitch_reachable(0.1, -0.2, 0.0, 1.2, 2.0, B_up=0.3, B_down=0.5)
        b = pitch_reachable(
            0.1, -0.2, 0.0, 1.2, 2.0, B_up=0.3, B_down=0.5, omega_bar=0.0
        )
        self.assertEqual(a.reachable, b.reachable)
        self.assertAlmostEqual(a.r_theta, b.r_theta)
        self.assertAlmostEqual(a.T_r, b.T_r)

    def test_omega_bar_widens_rate_condition(self) -> None:
        kw = dict(theta_L=0.0, T=1.5, a=math.radians(120.0))
        b_up, b_dn = math.radians(10.1), math.radians(24.1)
        s = math.radians(-13.5)
        closed = pitch_reachable(
            math.radians(8.1), s, kw["theta_L"], kw["T"], kw["a"],
            B_up=b_up, B_down=b_dn,
        )
        windowed = pitch_reachable(
            math.radians(8.1), s, kw["theta_L"], kw["T"], kw["a"],
            B_up=b_up, B_down=b_dn, omega_bar=math.radians(5.0),
        )
        self.assertFalse(closed.budget_ok)
        self.assertTrue(windowed.budget_ok)
        self.assertTrue(windowed.reachable)

    def test_omega_bar_within_window_is_pure_drift(self) -> None:
        result = pitch_reachable(
            0.0, 0.04, 0.0, T=1.0, a=2.0,
            B_up=0.3, B_down=0.3, omega_bar=0.05,
        )
        self.assertEqual(result.T_r, 0.0)
        self.assertAlmostEqual(result.r_theta, 0.04)
        self.assertTrue(result.reachable)

if __name__ == "__main__":
    unittest.main()
