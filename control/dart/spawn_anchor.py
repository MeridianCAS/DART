"""Reuse proven simul-3 spawn poses. Never reuse an anchor across camber levels."""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent.parent
DEFAULT_REGISTRY = REPO / "artifacts" / "dart_viz" / "spawn_anchors.json"

def registry_path() -> Path:
    import os
    p = os.environ.get("DART_SPAWN_ANCHOR_REGISTRY")
    if p:
        pp = Path(p)
        return pp if pp.is_absolute() else (REPO / pp)
    return DEFAULT_REGISTRY

def fingerprint_from_run(*, args, simul_legs: list[dict]) -> dict[str, Any]:
    """Stable experiment key; the cohort tag is deliberately excluded so anchors transfer across runs."""
    labels = sorted(str(l.get("label", "")) for l in simul_legs)
    fp: dict[str, Any] = {
        "schema": "dart_spawn_anchor_v1",
        "jump_scenario": str(getattr(args, "jump_scenario", "") or ""),
        "launch_mode": str(getattr(args, "launch_mode", "") or ""),
        "simul_layout": str(getattr(args, "simul_layout", "") or ""),
        "simul_strategies": ",".join(
            s.strip()
            for s in str(getattr(args, "simul_strategies", "") or "").split(",")
            if s.strip()
        ),
        "simul_copy_spacing": round(float(getattr(args, "simul_copy_spacing", 0.0) or 0.0), 3),
        "simul_lane_gap": round(float(getattr(args, "simul_lane_gap", 0.0) or 0.0), 3),
        "run_up": round(float(getattr(args, "run_up", 0.0) or 0.0), 3),
        "base_x": round(float(getattr(args, "base_x", 0.0) or 0.0), 3),
        "runup_camber_deg": round(float(getattr(args, "runup_camber_deg", 0.0) or 0.0), 3),
        "approach_spawn_roll_deg": round(
            float(getattr(args, "approach_spawn_roll_deg", 0.0) or 0.0), 3),
        "leg_labels": labels,
    }
    return fp

def fingerprint_key(fp: dict[str, Any]) -> str:
    blob = json.dumps(fp, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]

def _load_registry() -> dict[str, Any]:
    p = registry_path()
    if not p.is_file():
        return {"schema": "dart_spawn_anchor_registry_v1", "anchors": {}}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return {"schema": "dart_spawn_anchor_registry_v1", "anchors": {}}
    data.setdefault("anchors", {})
    return data

def _write_registry(data: dict[str, Any]) -> Path:
    p = registry_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return p

def _fingerprint_consistent(stored: dict[str, Any], current: dict[str, Any]) -> bool:
    """Reject stored anchors whose scenario or camber differs from the current run."""
    for k in ("jump_scenario", "runup_camber_deg"):
        if stored.get(k) != current.get(k):
            return False
    return True

def load_anchors(*, args, simul_legs: list[dict]) -> dict[str, tuple[float, float, float]] | None:
    fp = fingerprint_from_run(args=args, simul_legs=simul_legs)
    key = fingerprint_key(fp)
    reg = _load_registry()
    entry = reg.get("anchors", {}).get(key)
    if not entry:
        return None
    stored_fp = entry.get("fingerprint") or {}
    if not _fingerprint_consistent(stored_fp, fp):
        return None
    legs = entry.get("legs") or {}
    out: dict[str, tuple[float, float, float]] = {}
    for leg in simul_legs:
        lab = str(leg.get("label", ""))
        rec = legs.get(lab)
        if not rec or "pos" not in rec:
            return None
        pos = rec["pos"]
        if not isinstance(pos, (list, tuple)) or len(pos) != 3:
            return None
        out[lab] = (float(pos[0]), float(pos[1]), float(pos[2]))
    if len(out) != len(simul_legs):
        return None
    return out

def save_anchors(
    *,
    args,
    simul_legs: list[dict],
    leg_positions: dict[str, tuple[float, float, float]],
    tag: str,
    jump_id: int,
) -> Path:
    fp = fingerprint_from_run(args=args, simul_legs=simul_legs)
    key = fingerprint_key(fp)
    reg = _load_registry()
    legs_out = {}
    for lab, pos in sorted(leg_positions.items()):
        legs_out[lab] = {
            "pos": [round(float(pos[0]), 4), round(float(pos[1]), 4), round(float(pos[2]), 4)],
        }
    reg.setdefault("anchors", {})[key] = {
        "fingerprint": fp,
        "fingerprint_key": key,
        "legs": legs_out,
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source_tag": tag,
        "source_jump_id": int(jump_id),
    }
    return _write_registry(reg)

def clear_anchors(*, args, simul_legs: list[dict]) -> bool:
    fp = fingerprint_from_run(args=args, simul_legs=simul_legs)
    key = fingerprint_key(fp)
    reg = _load_registry()
    if key not in reg.get("anchors", {}):
        return False
    del reg["anchors"][key]
    _write_registry(reg)
    return True
