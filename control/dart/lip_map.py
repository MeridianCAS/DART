"""Takeoff-state predictor ``v -> (theta0, omega_y0, T)``. Load a saved fit; do not refit here."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

@dataclass(frozen=True)
class LipMapFit:
    family: str
    params: tuple[float, ...]
    holdout_median_err: float = float("nan")
    holdout_median_abs_err: float = float("nan")

    def predict(self, v: float) -> float:
        return predict_omega(self.family, self.params, v)

@dataclass(frozen=True)
class LipMap:
    omega_fit: LipMapFit
    theta0_coef: tuple[float, float]
    T_coef: tuple[float, float]
    v_range: tuple[float, float]
    residual_sigma: dict[str, float] = field(default_factory=dict)

    def predict(self, v: float) -> tuple[float, float, float]:
        """Return ``(theta0_deg, omega_y0_dps, T_s)``."""
        th = self.theta0_coef[0] + self.theta0_coef[1] * v
        om = self.omega_fit.predict(v)
        tt = self.T_coef[0] + self.T_coef[1] * v
        return th, om, tt

def predict_omega(family: str, params: Sequence[float], v: float) -> float:
    v = max(0.5, float(v))
    if family == "impulse":
        (c,) = params
        return c / v
    if family == "rational":
        a, vc = params
        return a * v / (1.0 + (v / vc) ** 2)
    if family == "inv_affine":
        c, b = params
        return c / v + b
    if family == "pwl":
        knots = list(params)
        vs = knots[0::2]
        ws = knots[1::2]
        if not vs:
            raise ValueError("pwl params empty")
        if v <= vs[0]:
            return ws[0]
        if v >= vs[-1]:
            return ws[-1]
        for i in range(1, len(vs)):
            if v <= vs[i]:
                t = (v - vs[i - 1]) / max(1e-9, vs[i] - vs[i - 1])
                return ws[i - 1] + t * (ws[i] - ws[i - 1])
        return ws[-1]
    raise ValueError(f"unknown family {family!r}")

def load_lip_map_fit(path) -> LipMap:
    """Rebuild a LipMap from a G1-passed fit JSON. Fail-closed if G1 failed."""
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    g1 = doc.get("G1") or {}
    if not g1.get("pass"):
        raise ValueError(
            f"lip map fit {path}: G1 did not pass "
            f"(holdout_median_err={g1.get('holdout_median_err_dps')})"
        )
    per_level = doc.get("per_level") or {}
    v_meds = [
        lv.get("v0_med")
        for lv in per_level.values()
        if lv.get("v0_med") is not None
    ]
    v_rng = (min(v_meds), max(v_meds)) if v_meds else (0.0, 0.0)
    fit = LipMapFit(
        family=str(doc["selected_family"]),
        params=tuple(float(p) for p in doc["selected_params"]),
        holdout_median_err=float(g1.get("holdout_median_err_dps", float("nan"))),
        holdout_median_abs_err=float(
            g1.get("holdout_median_abs_err_dps", float("nan"))
        ),
    )
    return LipMap(
        omega_fit=fit,
        theta0_coef=tuple(float(c) for c in doc["theta0_coef"]),
        T_coef=tuple(float(c) for c in doc["T_coef"]),
        v_range=v_rng,
        residual_sigma=dict(doc.get("residual_sigma") or {}),
    )

__all__ = ["LipMapFit", "LipMap", "predict_omega", "load_lip_map_fit"]
