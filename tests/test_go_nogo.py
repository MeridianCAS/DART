"""Tests for the go/no-go gate and certified-speed scan."""
from __future__ import annotations

import unittest

from control.dart.go_nogo import (
    GoNoGo,
    GoNoGoGate,
    certified_speed_window,
)
from control.dart.reachability import TakeoffReachableSet

class GoNoGoTests(unittest.TestCase):
    def test_disabled_gate_is_inactive(self) -> None:
        gate = GoNoGoGate(
            TakeoffReachableSet(0.0, 1.0, 2.0),
            v_crit=8.0,
            a_brake=4.0,
            enabled=False,
        )
        dec = gate.evaluate(10.0, 30.0, 0.0, 0.0)
        self.assertEqual(dec.decision, GoNoGo.INACTIVE)

    def test_certified_window_finds_interior_speeds(self) -> None:
        def predict(v):
            # Slow takeoffs stay near the origin; fast ones add pitch rate.
            return 0.0, 0.02 * (v - 10.0), 1.0

        win = certified_speed_window(
            predict,
            theta_L=0.0,
            a_pitch=2.0,
            v_lo=8.0,
            v_hi=14.0,
            step=1.0,
        )
        self.assertFalse(win.empty)
        self.assertTrue(win.contains(10.0))

if __name__ == "__main__":
    unittest.main()
