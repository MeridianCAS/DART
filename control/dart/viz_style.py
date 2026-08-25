"""Locked simul-leg colors: dart=red, tobb=yellow, rwpd=cyan."""
from __future__ import annotations

STRAT_CAR_RGBA = {
    "dart": (1.0, 0.0, 0.0, 1.0),
    "tobb": (1.0, 0.9, 0.0, 1.0),
    "rwpd": (0.0, 0.9, 1.0, 1.0),
}

CANONICAL_HUE = {"dart": "red", "tobb": "yellow", "rwpd": "cyan"}

def car_color(strategy: str):
    """Return the locked BeamNG RGBA for a strategy; unknown keys raise."""
    if strategy not in STRAT_CAR_RGBA:
        raise KeyError(
            f"[viz_style] unknown strategy={strategy!r}; expected one of {list(STRAT_CAR_RGBA)}"
        )
    return STRAT_CAR_RGBA[strategy]

def assert_canonical_car_colors(spec: dict | None = None) -> None:
    """Assert the canonical color lock (dart=red, tobb=yellow, rwpd=cyan).

    With ``spec`` given (strategy -> (dart_on, baseline, rgba)), also assert
    that the runner's strategy table carries the same colors. Raises
    ``AssertionError`` on any drift (fail-closed).
    """
    expect = {
        "dart": (1.0, 0.0, 0.0, 1.0),
        "tobb": (1.0, 0.9, 0.0, 1.0),
        "rwpd": (0.0, 0.9, 1.0, 1.0),
    }
    for s, rgba in expect.items():
        assert STRAT_CAR_RGBA[s] == rgba, (
            f"[viz_style] {s} color drifted: {STRAT_CAR_RGBA[s]} != {rgba} ({CANONICAL_HUE[s]})"
        )
    if spec is not None:
        for s, rgba in expect.items():
            assert s in spec, f"[viz_style] strategy {s!r} missing from spec"
            got = spec[s][2] if isinstance(spec[s], (list, tuple)) and len(spec[s]) >= 3 else spec[s]
            assert tuple(got) == rgba, (
                f"[viz_style] {s} spec color {got} != {rgba} ({CANONICAL_HUE[s]})"
            )

__all__ = ["STRAT_CAR_RGBA", "CANONICAL_HUE", "car_color", "assert_canonical_car_colors"]
