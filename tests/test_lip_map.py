"""Tests for the saved LipMap predictor."""
from __future__ import annotations

import unittest

from control.dart.lip_map import LipMap, LipMapFit, predict_omega

class LipMapTests(unittest.TestCase):
    def test_impulse_family(self) -> None:
        self.assertAlmostEqual(predict_omega("impulse", (100.0,), 10.0), 10.0)

    def test_map_predict(self) -> None:
        lm = LipMap(
            omega_fit=LipMapFit("impulse", (200.0,)),
            theta0_coef=(2.0, 0.5),
            T_coef=(0.8, 0.01),
            v_range=(8.0, 16.0),
        )
        th, om, tt = lm.predict(10.0)
        self.assertAlmostEqual(th, 7.0)
        self.assertAlmostEqual(om, 20.0)
        self.assertAlmostEqual(tt, 0.9)

if __name__ == "__main__":
    unittest.main()
