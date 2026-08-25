#!/usr/bin/env python
"""DART jump experiment bench for the arXiv paper reproduction package.

Runs parameterized jump cohorts in BeamNG.tech (air-impulse and full approach
modes), compares DART / RW-PD / TOBB and ablation variants, and writes one JSON
per cohort under ``data/cohorts/``. Controller names match the paper
(``dart`` / ``rwpd`` / ``tobb`` and the latch / ablation variants).
"""
from __future__ import annotations
import argparse, contextlib, json, math, os, random, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
import scripts._ramp_feather as rf
import scripts._natural_jump as nj
import scripts._ramp_bench as bench
import scripts._tp_perramp as tp
from control.dart.provenance import make_provenance, make_provenance_from_data
from control.dart.runup_ground import (
    apply_ramp_material,
    build_runup_pad_segments,
    place_ramp_with_ground,
    runup_ground_audit,
)
from control.dart.airtime_guardrail import evaluate as airtime_guardrail_eval
from control.dart.baselines import rwpd_airborne_ctrl, tobb_airborne_ctrl
from control.dart.reachability import TakeoffReachableSet
from control.dart.go_nogo import GoNoGoGate, GoNoGo, SafetyAction
from control.dart.viz_style import car_color, assert_canonical_car_colors
ART = REPO / "data" / "cohorts"
DT = 0.01

LEGACY_STRATEGY_ALIASES = {
    "c7": "dart",
    "pd": "rwpd",
    "mpc": "tobb",
    "c7_adp": "dart_latched",
    "c7_b": "dart_replicate",
    "c7_dual": "dart_dual",
    "c7_pitch": "dart_pitch_only",
    "c7_roll": "dart_roll_only",
}

def _normalize_strategy_args(args) -> None:
    """Map legacy strategy codenames on the CLI to the paper names."""
    for attr in ("simul_strategies", "multiroll_strategies"):
        v = getattr(args, attr, None)
        if v:
            toks = [t.strip() for t in str(v).split(",") if t.strip()]
            setattr(args, attr, ",".join(LEGACY_STRATEGY_ALIASES.get(t, t) for t in toks))

def _session_checkpoint_path(tag: str) -> Path:
    return ART / f"dart_bench_{tag}.checkpoint.json"

def _save_session_checkpoint(*, tag: str, data: dict, n_valid: int, jump_id: int,
                             target_valid: int, angle: float,
                             last_refresh_at_valid: int = 0) -> None:
    """Write a checkpoint on each ACCEPT; resume with --resume-checkpoint after long-run interrupt."""
    ART.mkdir(parents=True, exist_ok=True)
    payload = {
        "tag": tag,
        "angle": angle,
        "n_valid": n_valid,
        "target_valid": target_valid,
        "last_jump_id": jump_id,
        "last_refresh_at_valid": int(last_refresh_at_valid),
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "data": data,
    }
    _session_checkpoint_path(tag).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )
    print(f"[CS-ckpt] saved valid={n_valid}/{target_valid} jump_id={jump_id} "
          f"-> {_session_checkpoint_path(tag).name}", flush=True)

def _load_session_checkpoint(tag: str) -> dict | None:
    p = _session_checkpoint_path(tag)
    if not p.is_file():
        return None
    try:
        ckpt = json.loads(p.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"[CS-ckpt] WARN load {p.name} failed: {e}", flush=True)
        return None
    if ckpt.get("tag") and ckpt.get("tag") != tag:
        print(f"[CS-ckpt] WARN tag mismatch ckpt={ckpt.get('tag')} != {tag}", flush=True)
        return None
    return ckpt
ANGLES = [10, 15, 20, 25, 30, 35, 40, 45]
LUA_WMEAN = ("local s=0 local n=0 if wheels~=nil and wheels.wheels~=nil then "
             "for _,wd in pairs(wheels.wheels) do s=s+math.abs(wd.angularVelocity) n=n+1 end end "
             "if n>0 then return tostring(s/n) else return '0' end")
LUA_WHEELS_ALL = ("local out={} if wheels~=nil and wheels.wheels~=nil then "
                  "for _,wd in pairs(wheels.wheels) do "
                  "if wd.name~=nil then out[tostring(wd.name)]=wd.angularVelocity end end end "
                  "return jsonEncode(out)")
LUA_DAMAGE = (
    "local out={} "
    "if electrics and electrics.values then out.damage=electrics.values.damage end "
    "if damageTracker and damageTracker.getDamage then "
    "  local ok,d=pcall(function() return damageTracker.getDamage() end) if ok then out.tracker=d end "
    "end "
    "return jsonEncode(out)"
)

def wheel_w(veh):
    """Mean wheel angular velocity at touchdown (rad/s): DART pitch drive can spin wheels to ~200; hard slide at contact risks blowout; land-prep should reduce this sharply."""
    try: return round(float(nj._vlua(veh, LUA_WMEAN)), 0)
    except Exception: return None

def wheel_w_all(veh):
    """Per-wheel angular velocity dict {wheel_name: angularVelocity} (4WIDS proxy). Returns {} if unreadable."""
    try:
        raw = nj._vlua(veh, LUA_WHEELS_ALL)
        if not raw:
            return {}
        obj = json.loads(str(raw))
        return {k: round(float(v), 2) for k, v in obj.items()} if isinstance(obj, dict) else {}
    except Exception:
        return {}

def damage_readback(veh):
    """Best-effort BeamNG damage/part readback. API varies by version/vehicle; returns None if unavailable."""
    try:
        raw = nj._vlua(veh, LUA_DAMAGE)
        if not raw:
            return None
        obj = json.loads(str(raw))
        return obj if obj else None
    except Exception:
        return None

def _pulse_cmd_from_pred(args, *, pred_err, pdot, cap=None, pulse_map=None,
                         full_error_deg=None, kd=None):
    """Map predicted pitch error to a bounded pulse command u (+throttle / -brake)."""
    cap = float(args.dart_pulse_max_cmd if cap is None else cap)
    pulse_map = str(args.dart_pulse_map if pulse_map is None else pulse_map)
    kd = float(args.dart_pulse_kd if kd is None else kd)
    if pulse_map == "linear":
        full = max(1e-6, float(args.dart_pulse_full_error_deg if full_error_deg is None else full_error_deg))
        u0 = cap * float(pred_err) / full - kd * float(pdot) / 100.0
    elif pulse_map == "segmented":
        eabs = abs(float(pred_err))
        if eabs <= float(args.dart_pulse_seg1_err_deg):
            mag = float(args.dart_pulse_seg1_cmd)
        elif eabs <= float(args.dart_pulse_seg2_err_deg):
            mag = float(args.dart_pulse_seg2_cmd)
        else:
            mag = float(args.dart_pulse_seg3_cmd)
        u0 = math.copysign(min(cap, mag), float(pred_err)) - kd * float(pdot) / 100.0
    else:
        u0 = float(args.dart_pulse_gain) * float(pred_err) / 20.0 - kd * float(pdot) / 100.0
    u = max(-cap, min(cap, u0))
    if abs(float(pred_err)) <= float(args.dart_pitch_deadband_deg):
        u = 0.0
    return u

def _angle_diff_deg(a, b):
    """Return (a-b) wrapped to [-180,180] degrees."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0

def _interp_profile_z(profile, x):
    """Linearly interpolate landing profile z in world coords; returns None if x is out of range."""
    if not profile:
        return None
    pts = [(float(px), float(pz)) for px, pz in profile]
    if x < pts[0][0] - 1e-6 or x > pts[-1][0] + 1e-6:
        return None
    for (x0, z0), (x1, z1) in zip(pts[:-1], pts[1:]):
        if min(x0, x1) - 1e-6 <= x <= max(x0, x1) + 1e-6:
            t = 0.0 if abs(x1 - x0) < 1e-9 else (x - x0) / (x1 - x0)
            return z0 + t * (z1 - z0)
    return pts[-1][1]

def slope_z_at_x(x, *, peak_x, peak_z, beta_deg):
    """Downslope plane z(x). For x<peak_x return peak height approx to avoid extrapolating onto takeoff ramp."""
    return peak_z + max(0.0, x - peak_x) * math.tan(math.radians(beta_deg))

def front_clearance_proxy(pos, direction, pitch_rad, *, peak_x, peak_z, beta_deg,
                          front_x, front_z_offset):
    """Front bumper/nose geometric clearance proxy vs downslope (m).

    Uses body pose + simplified geometry only; no BeamNG node names:
    - front point = vehicle ref pos + forward_unit * front_x + vertical offset front_z_offset
    - front_z_offset<0 approximates bumper below vehicle ref/CoM
    - clearance = front_point_z - slope_z(front_point_x)

    Conservative trend metric for comparing whether flare keeps the nose farther from slope; absolute values need later node/vehicle calibration.
    """
    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
    dx, dy, dz = float(direction[0]), float(direction[1]), float(direction[2])
    n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    dx, dy, dz = dx / n, dy / n, dz / n
    nose_x = px + dx * front_x
    nose_z = pz + dz * front_x + front_z_offset
    ground_z = slope_z_at_x(nose_x, peak_x=peak_x, peak_z=peak_z, beta_deg=beta_deg)
    return {
        "front_x": round(nose_x, 2),
        "front_z": round(nose_z, 2),
        "slope_z": round(ground_z, 2),
        "clearance_m": round(nose_z - ground_z, 3),
        "pitch_deg": round(math.degrees(pitch_rad), 1),
        "beta_deg": round(beta_deg, 2),
    }

def _slope_marker(name, x, z, theta, *, y=0.0, width=1.0, along=1.0, thick=0.18,
                  clearance=0.05, material="track_editor_C_border"):
    """Place a visual bump marker above the slope surface. size=(worldY width, along-slope length, thickness)."""
    normal_x = -math.sin(theta)
    normal_z = math.cos(theta)
    cx = x + (clearance + thick / 2.0) * normal_x
    cz = z + (clearance + thick / 2.0) * normal_z
    return {"name": name, "pos": (cx, y, cz), "size": (width, along, thick),
            "rot": (0.0, math.sin(theta / 2.0), 0.0, math.cos(theta / 2.0)),
            "material": material}

def _vertical_marker(name, x, z, *, y, size=(1.0, 1.0, 3.0), material="track_editor_C_border",
                     root_ground=False):
    """Exaggerated vertical post/wall marker: no slope rotation so distant camera still sees landing bounds.

    root_ground=True: post rooted at SmallGrid z=0, top above slope (z+size[2]). **Flat/no-bank only**;
    after bank_landing_segments camber/cross-slope, root_ground base no longer sits on slope -> use root_ground=False.
    root_ground=False: base on slope z, height size[2], then rigid bank rotation -> flush with slope.
    """
    if root_ground:
        h = max(0.5, z + size[2])
        return {"name": name, "pos": (x, y, h / 2.0), "size": (size[0], size[1], h),
                "rot": (0.0, 0.0, 0.0, 1.0), "material": material}
    return {"name": name, "pos": (x, y, z + size[2] / 2.0), "size": size,
            "rot": (0.0, 0.0, 0.0, 1.0), "material": material}

def _bank_xyz(x, y, z, phi_deg, *, pivot_y=0.0, pivot_z=0.0):
    """Coordinates after banking about world X through line y=pivot_y, z=pivot_z."""
    phi = math.radians(float(phi_deg))
    if abs(phi) < 1e-9:
        return float(x), float(y), float(z)
    c, s = math.cos(phi), math.sin(phi)
    yr, zr = float(y) - pivot_y, float(z) - pivot_z
    return float(x), pivot_y + yr * c - zr * s, pivot_z + yr * s + zr * c

def _quat_bank_x(phi_deg):
    phi = math.radians(float(phi_deg))
    return (math.sin(phi / 2.0), 0.0, 0.0, math.cos(phi / 2.0))

def _quat_slope_y(theta):
    return (0.0, math.sin(theta / 2.0), 0.0, math.cos(theta / 2.0))

def _banked_vertical_post(name, x, z_prof, edge_y, theta, *, phi_deg, pivot_y,
                          size=(0.7, 0.7, 1.4), surface_clear=0.35):
    """Post base on already-banked slope; orientation = bank × slope (no second bank_landing_segments)."""
    z_base = float(z_prof) + surface_clear
    bx, by, bz = _bank_xyz(x, edge_y, z_base, phi_deg, pivot_y=pivot_y)
    q = _quat_mul(_quat_bank_x(phi_deg), _quat_slope_y(theta))
    phi = math.radians(float(phi_deg))
    uy, uz = -math.sin(phi), math.cos(phi)
    h = float(size[2])
    return {"name": name, "pos": (bx, by + uy * h / 2.0, bz + uz * h / 2.0),
            "size": size, "rot": q, "material": "track_editor_C_border"}

def _banked_slope_rail(name, mx, mz, theta, edge_y, along, *, phi_deg, pivot_y,
                       width=0.45, thick=0.35, clearance=0.22, ramp_thick=0.8):
    nx, nz = -math.sin(theta), math.cos(theta)
    off = clearance + thick / 2.0
    cx_u = mx + off * nx
    cz_u = mz + off * nz + ramp_thick * 0.5
    bx, by, bz = _bank_xyz(cx_u, edge_y, cz_u, phi_deg, pivot_y=pivot_y)
    q = _quat_mul(_quat_bank_x(phi_deg), _quat_slope_y(theta))
    return {"name": name, "pos": (bx, by, bz), "size": (width, along, thick),
            "rot": q, "material": "track_editor_C_border"}

def _post_root_ground(*, runup_camber_deg: float = 0.0, cross_slope_deg: float = 0.0) -> bool:
    """With camber / landing cross-slope, posts must sit on slope; forbid root_ground at z=0."""
    return abs(float(runup_camber_deg)) < 1e-6 and abs(float(cross_slope_deg)) < 1e-6

def _append_ramp_rail_visuals(segs, pts, *, prefix, ri, width, post_root_ground=True):
    """Add outer slope-hugging rails + gate posts along polyline slope (landing_slope family; takeoff/landing shared)."""
    if len(pts) < 2:
        return segs
    rail_w = 0.45
    edge_y = width / 2.0 - 1.0
    for i in range(len(pts) - 1):
        x0, z0 = pts[i]; x1, z1 = pts[i + 1]
        theta = math.atan2(z1 - z0, x1 - x0)
        mx, mz = 0.5 * (x0 + x1), 0.5 * (z0 + z1)
        seg_len = math.hypot(x1 - x0, z1 - z0)
        for side, y in (("L", edge_y), ("R", -edge_y)):
            segs.append(_slope_marker(
                f"{prefix}{ri}_rail_{side}_{i}", mx, mz, theta, y=y,
                width=rail_w, along=seg_len, thick=0.35, clearance=0.22))
    step = max(1, len(pts) // 8)
    for j in range(0, len(pts), step):
        x, z = pts[j]
        for side, y in (("L", edge_y), ("R", -edge_y)):
            segs.append(_vertical_marker(
                f"{prefix}{ri}_post_{side}_{j}", x, z, y=y,
                size=(0.7, 0.7, 1.4), root_ground=post_root_ground))
    return segs

def _quat_mul(a, b):
    """Hamilton product a⊗b, quaternion (x,y,z,w). Apply b then a (left-multiply = further world-frame rotation)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )

def bank_landing_segments(lsegs, phi_deg, *, pivot_y=0.0, pivot_z=0.0):
    """Rigid bank phi_deg about ground line {y=pivot_y, z=pivot_z} along forward (landing / run-up+takeoff shared).

    A plane rotated about a line stays a plane, so the whole rig (rails included) stays coherent, only laterally tilted.
    - pivot_y=0 (landing cross-slope): bank about centerline; centerline fixed.
    - pivot_y=∓W/2 (run-up camber): bank about low edge -> low edge at z=0 (natural feather), rest lifted,
      both wheels on incline -> true lateral roll (=γ). Center pivot lets low half sink into flat ground and lose contact, no roll.
    phi_deg>0: viewed along +X, dips down-right (right side low); <0 opposite. Returns new segs list (does not mutate input).
    """
    if abs(float(phi_deg)) < 1e-6:
        return lsegs
    phi = math.radians(float(phi_deg))
    c, s = math.cos(phi), math.sin(phi)
    q_bank = (math.sin(phi / 2.0), 0.0, 0.0, math.cos(phi / 2.0))
    out = []
    for seg in lsegs:
        x, y, z = seg["pos"]
        yr = y - pivot_y
        zr = z - pivot_z
        new_y = pivot_y + yr * c - zr * s
        new_z = pivot_z + yr * s + zr * c
        ns = dict(seg)
        ns["pos"] = (x, new_y, new_z)
        ns["rot"] = _quat_mul(q_bank, tuple(seg.get("rot", (0.0, 0.0, 0.0, 1.0))))
        out.append(ns)
    return out

def _camber_pivot_y(args) -> float:
    """Run-up camber pivot y = ∓W/2 about low edge (γ>0 low side at y<0 -> pivot=-W/2).

    Under camber use apron width (default 40 m) not narrow ramp width (12 m) so low edge truly grounds and center lifts enough."""
    gamma = _runup_camber_deg(args)
    half_w = _camber_apron_width(args) / 2.0
    return -math.copysign(half_w, gamma)

def _prepend_runup_flat(pts, x_back, *, z_ground=0.0):
    """Prepend flat z=0 run-up before kicker polyline to avoid banked ramp toe becoming a vertical wall."""
    if not pts:
        return pts
    x0, z0 = float(pts[0][0]), float(pts[0][1])
    if x0 <= float(x_back) + 0.15:
        return list(pts)
    span = x0 - float(x_back)
    n = max(4, int(span / 2.5))
    pre = [(float(x_back) + span * k / n, z_ground) for k in range(n)]
    return pre + list(pts)

def _runup_camber_deg(args) -> float:
    return float(getattr(args, "runup_camber_deg", 0.0) or 0.0)

def _simul_layout(args) -> str:
    return str(getattr(args, "simul_layout", "y_lane") or "y_lane")

def _simul_x_copy(args) -> bool:
    return _simul_layout(args) == "x_copy"

def _simul_y_copy(args) -> bool:
    return _simul_layout(args) == "y_copy"

def _simul_multi_ramp_copy(args) -> bool:
    """True when each simul leg gets its own translated ramp mesh (x_copy or y_copy)."""
    return _simul_x_copy(args) or _simul_y_copy(args)

def _leg_base_x(leg, args) -> float:
    return float(leg.get("base_x", args.base_x))

def _leg_peak_x(leg, default_peak_x: float) -> float:
    v = leg.get("peak_x")
    return float(v) if v is not None else float(default_peak_x)

def _translate_mesh_segments(segs, dx: float = 0.0, dy: float = 0.0) -> list:
    """Rigid translate ramp/apron segments (x_copy=+X, y_copy=+Y)."""
    dx, dy = float(dx), float(dy)
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return [dict(s) for s in segs]
    out = []
    for seg in segs:
        ns = dict(seg)
        x, y, z = seg["pos"]
        ns["pos"] = (float(x) + dx, float(y) + dy, float(z))
        out.append(ns)
    return out

def _shift_profile_xy(lpts, dx: float) -> list:
    dx = float(dx)
    return [(float(x) + dx, float(z)) for x, z in lpts]

def _assign_simul_leg_ramp_geom(simul_legs, *, peak_x: float, lpts, args) -> None:
    bx0 = float(args.base_x)
    prof_tpl = [(round(float(x), 3), round(float(z), 3)) for x, z in lpts]
    for leg in simul_legs:
        dx = _leg_base_x(leg, args) - bx0
        leg["peak_x"] = float(peak_x) + dx
        leg["profile"] = _shift_profile_xy(prof_tpl, dx)

def _place_simul3_ramp_meshes(bng, qlua, simul_legs, segs, lsegs, lpts, peak_x, args):
    """Place ramp mesh(es). y_copy/y_lane/x_copy per --simul-layout."""
    gt = getattr(args, "runup_ground_type", "ASPHALT")
    if not _simul_multi_ramp_copy(args) or len(simul_legs) <= 1:
        placed = place_ramp_with_ground(bng, segs + lsegs, qlua, gt)
        _assign_simul_leg_ramp_geom(simul_legs, peak_x=peak_x, lpts=lpts, args=args)
        return placed
    all_segs: list = []
    if _simul_y_copy(args):
        for leg in simul_legs:
            all_segs.extend(_translate_mesh_segments(segs + lsegs, dy=float(leg["y"])))
        placed = place_ramp_with_ground(bng, all_segs, qlua, gt)
        _assign_simul_leg_ramp_geom(simul_legs, peak_x=peak_x, lpts=lpts, args=args)
        _y_sp = float(getattr(args, "simul_copy_spacing", 45.0))
        print(f"[CS-simul3-ycopy] placed {len(simul_legs)} ramp copies "
              f"y={[round(float(l['y']), 1) for l in simul_legs]} spacing={_y_sp}m "
              f"base_x={float(args.base_x):.1f}", flush=True)
        return placed
    bx0 = float(args.base_x)
    for leg in simul_legs:
        dx = _leg_base_x(leg, args) - bx0
        all_segs.extend(_translate_mesh_segments(segs + lsegs, dx=dx))
    placed = place_ramp_with_ground(bng, all_segs, qlua, gt)
    _assign_simul_leg_ramp_geom(simul_legs, peak_x=peak_x, lpts=lpts, args=args)
    print(f"[CS-simul3-xcopy] placed {len(simul_legs)} ramp copies spacing="
          f"{float(getattr(args, 'simul_copy_spacing', 180.0))}m "
          f"base_x={[round(_leg_base_x(l, args), 1) for l in simul_legs]}", flush=True)
    return placed

def _simul_takeoff_spread_passes(v0_sp, th0_sp, r0_sp, args) -> tuple[bool, list[str]]:
    if not bool(int(getattr(args, "simul_takeoff_spread_gate", 0) or 0)):
        return True, []
    reasons: list[str] = []
    mv = float(getattr(args, "simul_max_v0_spread", 0.15))
    mt = float(getattr(args, "simul_max_theta0_spread", 0.5))
    mr = float(getattr(args, "simul_max_roll0_spread", 1.0))
    if v0_sp is not None and v0_sp > mv:
        reasons.append("v0_spread")
    if th0_sp is not None and th0_sp > mt:
        reasons.append("theta0_spread")
    if r0_sp is not None and r0_sp > mr:
        reasons.append("roll0_spread")
    return (len(reasons) == 0), reasons

def _camber_apron_width(args) -> float:
    """Camber wide apron width (decoupled from narrow ramp width); pivot_y uses this."""
    if abs(_runup_camber_deg(args)) < 1e-6:
        return float(getattr(args, "width", 36.0))
    return float(getattr(args, "runup_apron_width", 40.0) or 40.0)

def _camber_mesh_width(args) -> float:
    """Under camber, takeoff+landing mesh matches apron width to avoid 12 m narrow ramp vs 40 m apron transition falling off collision."""
    if abs(_runup_camber_deg(args)) < 1e-6:
        return float(getattr(args, "width", 36.0))
    return _camber_apron_width(args)

def _build_runup_apron(args) -> list:
    """Wide thick banked run-up local ground: single slab spawn->ramp toe.

    Joint filler: between apron (horizontal slab) and kicker uphill first segment
    (tilted slab) at x=base_x junction, side view shows wedge cavity and apron front only penetrates ramp body
    margin_front (~3 m), visible gap. Filler flush with apron top, extends from apron front
    8 m further into ramp — slope rises above filler top in that span; drive surface unchanged, cavity sealed only."""
    from control.dart.runup_ground import resolve_runup_material

    x0 = (float(args.base_x) - float(args.run_up)
          - float(getattr(args, "runup_apron_margin_back", 13.0)))
    x1 = float(args.base_x) + float(getattr(args, "runup_apron_margin_front", 0.0))
    length = max(2.0, x1 - x0)
    cx = 0.5 * (x0 + x1)
    w = _camber_apron_width(args)
    thick = max(0.55, float(args.thick))
    top_z = 0.0
    cz = top_z - thick * 0.5 + 0.03
    gt = str(getattr(args, "runup_ground_type", "ASPHALT") or "ASPHALT")
    mat = resolve_runup_material(gt)
    segs = [{
        "name": "dart_runup_apron",
        "pos": (cx, 0.0, cz),
        "size": (w, length, thick),
        "rot": (0.0, 0.0, 0.0, 1.0),
        "material": mat,
    }]
    segs += _build_toe_joint_filler(args, width=w, from_x=x1)
    return segs

def _build_toe_joint_filler(args, *, width: float, from_x: float | None = None) -> list:
    """Ramp-toe joint filler: seals wedge cavity between apron/ground and kicker uphill first segment (tilted box).

    Top flush with ground (z=0), dropped 15 mm against z-fighting; extends into ramp from 2 m before toe
    joint_fill_len; slope rises above filler in that span without changing drive surface. Non-camber cases
    (no apron, run-up = native SmallGrid ground) also apply."""
    from control.dart.runup_ground import resolve_runup_material

    joint_len = float(getattr(args, "runup_joint_fill_len", 8.0) or 0.0)
    if joint_len <= 0.0:
        return []
    x_start = float(from_x) if from_x is not None else float(args.base_x) - 2.0
    thick = max(0.55, float(args.thick))
    cz = 0.0 - thick * 0.5 + 0.03 - 0.015
    gt = str(getattr(args, "runup_ground_type", "ASPHALT") or "ASPHALT")
    return [{
        "name": "dart_runup_joint",
        "pos": (x_start + joint_len * 0.5, 0.0, cz),
        "size": (width, joint_len, thick),
        "rot": (0.0, 0.0, 0.0, 1.0),
        "material": resolve_runup_material(gt),
    }]

def _apron_surface_z(y: float, args, *, base_z: float = 0.0) -> float:
    """Banked apron top surface world z at lane y (after rotation about low-edge pivot_y)."""
    camber = _runup_camber_deg(args)
    if abs(camber) < 1e-6:
        return base_z
    pivot_y = _camber_pivot_y(args)
    return base_z + (float(y) - pivot_y) * math.sin(math.radians(camber))

def _approach_steer_vehicle_sign(args) -> float:
    """dart 4motor EV: vehicle.control(steering) sign opposite natural_jump/SBR convention (steer>0=left).

    Warmup tests showed py≈+0.5 m left bias with steer_cmd<0 -> drifts further left (positive feedback).
    Default -1 negates -K·err law output before send; use --approach-steer-sign 1 to restore SBR convention."""
    return float(getattr(args, "approach_steer_sign", -1.0) or -1.0)

def _approach_lane_keep_gains(args):
    """Approach lane-keep gains. Flat default steer=0; camber weak lateral + strong heading, keep roll0≈γ."""
    kp_y = float(getattr(args, "approach_kp_y", 0.0) or 0.0)
    kp_yaw = float(getattr(args, "approach_kp_yaw", 0.0) or 0.0)
    ki_y = float(getattr(args, "approach_ki_y", 0.0) or 0.0)
    scl = float(getattr(args, "approach_steer_clamp", 0.12) or 0.12)
    gamma = abs(_runup_camber_deg(args))
    if gamma > 1e-6:
        if not bool(getattr(args, "_approach_kp_y_explicit", False)):
            kp_y = 0.38 + 0.028 * gamma
        if not bool(getattr(args, "_approach_kp_yaw_explicit", False)):
            kp_yaw = 0.14 + 0.038 * gamma
        if not bool(getattr(args, "_approach_ki_y_explicit", False)):
            ki_y = 0.0
        scl = max(scl, min(0.65, 0.34 + 0.022 * gamma))
        _hw = _camber_mesh_width(args) / 2.0
        if not args.__dict__.get("_camber_lane_keep_logged"):
            _sgn = _approach_steer_vehicle_sign(args)
            print(f"[CS-runup-camber] auto lane-keep v6.1-roll-gamma kp_y={kp_y:.3f} ki_y={ki_y:.3f} "
                  f"kp_yaw={kp_yaw:.3f} steer_clamp={scl:.2f} approach_steer_sign={_sgn:+.0f} "
                  f"yaw_lock≤{float(getattr(args, 'prelip_yaw_lock_m', 10.0)):g}m×{float(getattr(args, 'prelip_yaw_lock_boost', 6.0)):g} "
                  f"prelip≤{float(getattr(args, 'prelip_yaw_priority_m', 30.0)):g}m "
                  f"lip_v_cap={float(getattr(args, 'approach_lip_v_cap_mps', 0.0) or 11.0):g}m/s "
                  f"brake≤{float(getattr(args, 'approach_lip_stability_coast_m', 3.0)):g}m "
                  f"crest≤{float(getattr(args, 'approach_lip_stability_crest_m', 2.5)):g}m×{float(getattr(args, 'approach_lip_stability_crest_throttle', 0.50)):g} "
                  f"(γ={gamma:.1f}°, mesh half-width={_hw:.1f}m)", flush=True)
            _ac = _c7_air_cfg(args)
            if _ac.get("active") and not args.__dict__.get("_camber_air_logged"):
                print(f"[CS-camber-air] DART airborne v6.1 roll_tgt=γ={_ac['roll_target_deg']:.1f}° "
                      f"roll_gain={_ac['roll_gain']:.2f} touch_boost×{_ac['touch_roll_boost']:.2f} "
                      f"pred_h={_ac['pred_horizon_sec']:.2f}s p1b_pitch_kp={float(getattr(args, 'camber_air_p1b_pitch_kp', 1.35)):.2f} "
                      f"early_landmatch nc≥{_ac['early_landmatch_nc']} pz≤{_ac['early_landmatch_z']:.1f}m "
                      f"(γ={_ac['gamma']:.1f}°)", flush=True)
                args.__dict__["_camber_air_logged"] = True
            args.__dict__["_camber_lane_keep_logged"] = True
    return kp_y, kp_yaw, ki_y, scl

def _camber_gravity_steer_ff(gamma_deg, gspd_mps, args, *, err_y: float = 0.0) -> float:
    """Enabled only with explicit --camber-steer-ff; auto-camber default 0 (pre-bank roll cancels most gravity; auto ff causes left bias)."""
    _ff = float(getattr(args, "camber_steer_ff", 0.0) or 0.0)
    if abs(_ff) < 1e-9:
        return 0.0
    v = max(4.0, float(gspd_mps))
    scale = min(1.0, (v - 4.0) / 9.0)
    ff = math.copysign(_ff * scale, float(gamma_deg))
    ey = float(err_y)
    if float(gamma_deg) > 0 and ey > 0.06:
        ff *= max(0.0, 1.0 - ey / max(0.5, _camber_mesh_width(args) * 0.35))
    elif float(gamma_deg) < 0 and ey < -0.06:
        ff *= max(0.0, 1.0 - abs(ey) / max(0.5, _camber_mesh_width(args) * 0.35))
    return ff

def _camber_lane_bias_steer(gamma_deg, gspd_mps, err_y, args) -> float:
    """Optional constant bias; auto default 0 (after steer_sign fix, no negative bias cranking)."""
    if abs(float(gamma_deg)) < 1e-6:
        return 0.0
    _explicit = getattr(args, "camber_steer_bias", None)
    if _explicit is not None and abs(float(_explicit)) > 1e-9:
        return float(_explicit) * min(1.0, max(0.0, (float(gspd_mps) - 3.5) / 7.0))
    return 0.0

def _approach_lane_keep_steer(py, yaw_err, lane_y, args, *, gspd_mps=0.0, lane_i_err=0.0,
                              d_to_lip_m=None):
    """Approach lip-yaw0 steering: far segment py+yaw; ≤30 m yaw-first; ≤10 m yaw-lock (heading-only + full gain) for lip yaw0≈±17°."""
    _kpy, _kpyaw, _kiy, _scl = _approach_lane_keep_gains(args)
    err_y = float(py) - float(lane_y)
    gamma = _runup_camber_deg(args)
    v = float(gspd_mps)
    if abs(gamma) > 1e-6 and v > 6.0:
        _kpy *= 1.0 + 0.22 * min(2.0, (v - 6.0) / 4.5)
        _kpyaw *= 1.0 + 0.18 * min(2.0, (v - 6.0) / 5.0)
        if _kiy > 1e-9:
            _kiy *= 1.0 + 0.10 * min(1.0, (v - 8.0) / 6.0)
    _yaw_deg = math.degrees(float(yaw_err))
    _lock_m = float(getattr(args, "prelip_yaw_lock_m", 10.0) or 10.0)
    _lock_boost = float(getattr(args, "prelip_yaw_lock_boost", 6.0) or 6.0)
    _lock_min = float(getattr(args, "prelip_yaw_lock_min_deg", 2.0) or 2.0)
    _prelip_m = float(getattr(args, "prelip_yaw_priority_m", 30.0) or 30.0)
    _yaw_min = float(getattr(args, "prelip_yaw_min_deg", 3.0) or 3.0)
    _yaw_boost = float(getattr(args, "prelip_yaw_k_boost", 4.0) or 4.0)
    steer_py = -_kpy * err_y
    steer_yaw = -_kpyaw * float(yaw_err)
    _in_lock = (d_to_lip_m is not None and float(d_to_lip_m) <= _lock_m
                and abs(_yaw_deg) >= _lock_min)
    _prelip_yaw = (d_to_lip_m is not None and float(d_to_lip_m) <= _prelip_m
                   and abs(_yaw_deg) >= _yaw_min)
    if _in_lock:
        steer_yaw = -(_kpyaw * _lock_boost) * float(yaw_err)
        steer = steer_yaw
    elif _prelip_yaw:
        _kpyaw *= _yaw_boost
        steer_yaw = -_kpyaw * float(yaw_err)
        if abs(_yaw_deg) >= 5.0:
            _py_w = max(0.08, 1.0 - abs(_yaw_deg) / 12.0)
            steer = steer_yaw + _py_w * steer_py
        else:
            steer = steer_py + steer_yaw
    else:
        steer = steer_py + steer_yaw
    if _kiy > 1e-9:
        steer -= _kiy * float(lane_i_err)
    if abs(gamma) > 1e-6:
        steer += _camber_gravity_steer_ff(gamma, gspd_mps, args, err_y=err_y)
        steer += _camber_lane_bias_steer(gamma, gspd_mps, err_y, args)
    steer *= _approach_steer_vehicle_sign(args)
    return max(-_scl, min(_scl, steer))

def _approach_lip_v_cap_mps(args, v_entry):
    """Camber landing stability: lip target max speed (lower v_entry/tire load); 0=no cap."""
    _cap = float(getattr(args, "approach_lip_v_cap_mps", 0.0) or 0.0)
    if _cap > 0.0:
        return _cap
    if abs(_runup_camber_deg(args)) >= 6.0:
        return min(float(v_entry), 11.0)
    return float(v_entry)

def _approach_p0_gate_on(args, *, impulse_launch: bool = False) -> bool:
    return bool(int(getattr(args, "reachability_gate", 0))) and not impulse_launch

def _approach_p0_gate_update(args, *, px, gspd, d_to_lip, gs: dict):
    """P0 reachability gate one step: update gs, return (gate_brake, gate_in_coast, vmax_d)."""
    if not _approach_p0_gate_on(args):
        return False, False, None
    _coast_m = float(getattr(args, "gate_coast_m", 0.0))
    _d_brake = max(0.0, float(d_to_lip) - _coast_m)
    if bool(int(getattr(args, "gate_adaptive_abrake", 0))) and gs.get("gate_brake_prev") \
            and gs.get("gate_prev_px") is not None and gs.get("gate_prev_v") is not None:
        _dx = float(px) - float(gs["gate_prev_px"])
        if _dx > 0.05:
            _a_inst = (float(gs["gate_prev_v"]) ** 2 - float(gspd) ** 2) / (2.0 * _dx)
            if 0.5 < _a_inst < 25.0:
                _est = gs.get("gate_a_brake_est")
                gs["gate_a_brake_est"] = _a_inst if _est is None else 0.7 * _est + 0.3 * _a_inst
    _a_for_vmax = (gs.get("gate_a_brake_est") if (bool(int(getattr(args, "gate_adaptive_abrake", 0)))
                                                  and gs.get("gate_a_brake_est") is not None)
                   else float(args.gate_a_brake))
    _vmax_d = math.sqrt(max(0.0, float(args.gate_v_crit) ** 2 + 2.0 * _a_for_vmax * _d_brake))
    if gs.get("gate_spawn_v") is None:
        gs["gate_spawn_v"] = round(float(gspd), 2)
    _gate_in_coast = float(d_to_lip) <= _coast_m
    _gate_brake = (not _gate_in_coast) and (float(gspd) > _vmax_d + 0.2)
    if _gate_brake and gs.get("gate_brk_v0") is None:
        gs["gate_brk_v0"] = round(float(gspd), 2)
        gs["gate_brk_x0"] = round(float(px), 2)
    gs["gate_prev_px"] = float(px)
    gs["gate_prev_v"] = float(gspd)
    gs["gate_brake_prev"] = _gate_brake
    gs["gate_vmax_lip"] = round(float(args.gate_v_crit), 2)
    if _gate_brake:
        gs["gate_intervened_steps"] = int(gs.get("gate_intervened_steps", 0)) + 1
    return _gate_brake, _gate_in_coast, _vmax_d

def _approach_p0_gate_thr_brk(args, thr, brk, *, gspd, d_to_lip, px, peak_x, v_entry,
                              gate_brake, gate_in_coast, gate_on, vmax_d):
    """P0 gate final thr/brk ruling (same as one_jump approach segment)."""
    if not gate_on:
        return float(thr), float(brk)
    if gate_brake:
        return 0.0, 1.0
    if gate_in_coast:
        if bool(int(getattr(args, "gate_lip_power_recover", 0))):
            _lt = float(getattr(args, "gate_lip_launch_target", 0.0) or 0.0)
            return (1.0 if (_lt <= 0.0 or float(gspd) < _lt) else 0.0), float(brk)
        return 0.0, float(brk)
    if args.lip_power and float(px) >= (float(peak_x) - float(args.lip_power_m)):
        return 0.0, float(brk)
    _vt = min(float(v_entry), float(vmax_d)) if vmax_d is not None else float(v_entry)
    return (1.0 if float(gspd) < _vt else 0.0), float(brk)

def _apply_approach_lip_stability(thr, brk, *, gspd, d_to_lip_m, args, v_entry):
    """Lip cap takeoff speed: light brake above cap; partial throttle at crest (not full), avoid ~46 km/h rollover class."""
    if _approach_p0_gate_on(args):
        return float(thr), float(brk)
    _brake_m = float(getattr(args, "approach_lip_stability_coast_m", 3.0) or 3.0)
    _crest_m = float(getattr(args, "approach_lip_stability_crest_m", 2.5) or 2.5)
    _crest_thr = float(getattr(args, "approach_lip_stability_crest_throttle", 0.50) or 0.50)
    if d_to_lip_m is None:
        return float(thr), float(brk)
    d = float(d_to_lip_m)
    _cap = _approach_lip_v_cap_mps(args, v_entry)
    _floor = float(getattr(args, "approach_lip_stability_v_floor_mps", 10.0) or 10.0)
    g = float(gspd)
    if d <= _brake_m and g > _cap + 0.2:
        thr = 0.0
        _kg = float(getattr(args, "approach_lip_stability_brk_gain", 0.28) or 0.28)
        _bmax = float(getattr(args, "approach_lip_stability_brk_max", 0.4) or 0.4)
        brk = max(float(brk), min(_bmax, _kg * (g - _cap)))
    elif d <= _crest_m and g < _cap - 0.15:
        thr = max(float(thr), min(1.0, _crest_thr))
    elif d <= _brake_m + 2.0 and g < _floor:
        thr = max(float(thr), 1.0 if g < min(float(v_entry), _cap) else 0.0)
    return thr, brk

def _prelip_yaw_lock_active(d_to_lip_m, yaw_err, args) -> bool:
    """Lip yaw-lock zone: cut throttle + heading-only steer to buy steer response time."""
    if d_to_lip_m is None:
        return False
    _ydeg = abs(math.degrees(float(yaw_err)))
    return (float(d_to_lip_m) <= float(getattr(args, "prelip_yaw_lock_m", 10.0) or 10.0)
            and _ydeg >= float(getattr(args, "prelip_yaw_lock_throttle_deg", 5.0) or 5.0))

def _c7_air_cfg(args):
    """γ≥6 camber-specific DART airborne params; explicit --camber-air-* CLI overrides."""
    gamma = abs(_runup_camber_deg(args))
    active = gamma >= float(getattr(args, "camber_air_min_deg", 6.0) or 6.0)
    if not active:
        return {"active": False}
    return {
        "active": True,
        "gamma": gamma,
        "yaw_gain": float(getattr(args, "camber_air_yaw_gain", 0.8) or 0.8),
        "yaw_steer_max": float(getattr(args, "camber_air_yaw_steer_max", 0.6) or 0.6),
        "yaw_roll_atten_floor": float(getattr(args, "camber_air_yaw_roll_atten_floor", 0.55) or 0.55),
        "yaw_roll_atten_start_deg": float(getattr(args, "camber_air_yaw_roll_atten_start_deg", 12.0) or 12.0),
        "pitch_brk_touch_cap": float(getattr(args, "camber_air_pitch_brk_touch_cap", 0.35) or 0.35),
        "early_landmatch_z": float(getattr(args, "camber_air_early_landmatch_z", 5.5) or 5.5),
        "early_landmatch_nc": int(getattr(args, "camber_air_early_landmatch_nc", 1) or 1),
        "roll_gain": float(getattr(args, "camber_air_roll_gain", 2.4) or 2.4),
        "roll_steer_max": float(getattr(args, "camber_air_roll_steer_max", 0.65) or 0.65),
        "roll_target_deg": float(getattr(args, "runup_camber_deg", 0.0) or 0.0),
        "yaw_atten_on_touch": float(getattr(args, "camber_air_yaw_atten_on_touch", 0.35) or 0.35),
        "roll_priority_err_deg": float(getattr(args, "camber_air_roll_priority_err_deg", 5.0) or 5.0),
        "touch_roll_boost": float(getattr(args, "camber_air_touch_roll_boost", 1.35) or 1.35),
        "touch_roll_gain": float(getattr(args, "camber_air_touch_roll_gain", 1.15) or 1.15),
        "touch_roll_deadband_deg": float(getattr(args, "camber_air_touch_roll_deadband_deg", 1.0) or 1.0),
        "pred_horizon_sec": float(getattr(args, "camber_air_pred_horizon_sec", 0.28) or 0.28),
        "p1b_pitch_window_deg": float(getattr(args, "camber_air_p1b_pitch_window_deg", 20.0) or 20.0),
    }

def _camber_air_pred_horizon_sec(args, air_cfg) -> float:
    _hz = float(getattr(args, "dart_air_pred_horizon_sec", 0.0) or 0.0)
    if _hz > 0.0:
        return _hz
    if air_cfg.get("active"):
        return float(air_cfg.get("pred_horizon_sec", 0.28) or 0.28)
    return 0.0

def _camber_air_pitch_u(args, air_cfg, *, pitch_d, pdot, target_pitch_deg, p1a: bool) -> float:
    """Camber pitch: P1a prediction + overshoot cap; P1b error vs target (fixes β=-12° where abs(pitch)<8 never fired)."""
    err = float(target_pitch_deg) - float(pitch_d)
    kp = float(args.kp_pitch)
    kd = float(args.kd_pitch)
    cap = 1.0
    if p1a:
        hz = _camber_air_pred_horizon_sec(args, air_cfg)
        err_eff = (float(target_pitch_deg) - (float(pitch_d) + float(pdot) * hz)) if hz > 0.0 else err
        overshoot = float(getattr(args, "camber_air_pitch_overshoot_deg", 5.0) or 5.0)
        if float(pitch_d) < float(target_pitch_deg) - overshoot:
            err_eff = min(err_eff, float(getattr(args, "camber_air_pitch_overshoot_err_cap", 2.5) or 2.5))
    else:
        win = float(air_cfg.get("p1b_pitch_window_deg", 20.0) or 20.0)
        if abs(err) > win:
            return 0.0
        kp = float(getattr(args, "camber_air_p1b_pitch_kp", 1.35) or 1.35)
        kd = float(getattr(args, "camber_air_p1b_pitch_kd", 1.3) or 1.3)
        cap = float(getattr(args, "camber_air_p1b_pitch_cap", 0.48) or 0.48)
        err_eff = err
        rate_tol = float(getattr(args, "camber_air_p1b_rate_tol_dps", 30.0) or 30.0)
        if err < 0.0 and float(pdot) > rate_tol:
            err_eff -= 0.35 * (float(pdot) - rate_tol) / 100.0 * 20.0
    u = kp * err_eff / 20.0 - kd * float(pdot) / 100.0
    return max(-cap, min(cap, u))

def _camber_pitch_near_target(pitch_d, target_pitch_deg, air_cfg) -> bool:
    win = float(air_cfg.get("p1b_pitch_window_deg", 20.0) or 20.0)
    return abs(float(pitch_d) - float(target_pitch_deg)) < win

def _blend_camber_pitch_act(thr, brk, u, *, cap=1.0):
    """Blend camber pitch command: u>0=nose-up (thr), u<0=nose-down (brk)."""
    if u > 0.0:
        thr = max(float(thr), min(float(cap), u))
        if thr > 0.05:
            brk = 0.0
    elif u < 0.0:
        brk = max(float(brk), min(float(cap), -u))
        if brk > 0.05:
            thr = 0.0
    return thr, brk

def _camber_air_roll_err(roll_d, args, air_cfg):
    """Accept roll≈γ: correct only deviation from cross-slope target."""
    if not air_cfg.get("active"):
        return float(roll_d)
    return float(roll_d) - float(air_cfg.get("roll_target_deg", 0.0) or 0.0)

def _camber_air_roll_steer(roll_d, args, air_cfg, *, nc=0):
    """Camber airborne roll steer: differential positive-sign law + target roll≈γ; boost on slope contact (nc≥1); keeps land_roll≤8°."""
    if not air_cfg.get("active"):
        k = float(getattr(args, "k_roll", 1.0) or 1.0)
        cap = float(getattr(args, "smax", 0.3) or 0.3)
        return max(-cap, min(cap, -k * float(roll_d) / 30.0))
    err = _camber_air_roll_err(roll_d, args, air_cfg)
    db = float(getattr(args, "dart_roll_deadband_deg", 2.0) or 2.0)
    k = float(air_cfg["roll_gain"])
    cap = float(air_cfg["roll_steer_max"])
    if int(nc) >= int(air_cfg.get("early_landmatch_nc", 1)):
        _tg = float(air_cfg.get("touch_roll_gain", 1.15) or 1.15)
        _tb = float(air_cfg.get("touch_roll_boost", 1.35) or 1.35)
        if _tg > 1.01 or _tb > 1.01:
            k *= _tg
            cap = min(1.0, cap * _tb)
            db = min(db, float(air_cfg.get("touch_roll_deadband_deg", 1.0) or 1.0))
    if abs(err) <= db:
        return 0.0
    return max(-cap, min(cap, k * err / 30.0))

def _camber_p2_touch_roll_steer(roll_d, args, air_cfg, *, nc: int) -> float:
    """DART P2 on slope contact (nc≥1): keep φ_target=γ roll steer after pitch differential stops; optional boost vs P1a."""
    if not air_cfg.get("active"):
        return 0.0
    s = _camber_air_roll_steer(roll_d, args, air_cfg, nc=nc)
    if int(nc) < 1:
        return s
    boost = float(getattr(args, "camber_air_p2_roll_steer_gain", 2.0) or 2.0)
    cap = float(air_cfg["roll_steer_max"])
    return max(-cap, min(cap, s * boost))

def _c7_adaptive_roll_weight(args, roll_err_deg, yaw_err_air, force=False,
                             strat=None) -> float:
    """Adaptive roll channel weight (converged via interventional iterations).

    variant 2 (default, hysteresis latch): |roll_err| ≥ on_thr(=authority 8°) -> full authority (w=1),
      stay engaged until error enters deadband; never crossed on_thr -> w=0 ≈ pitch-only.
      Rationale: 1) proportional fade (variant 1) slows large-disturbance convergence -> medium tilt exposure tail lengthens ->
      sin(tilt) leakage increases (chaotic cell test: valid 18/30 vs 26/30, yaw 92° vs 26°);
      2) yaw budget gate cuts roll -> tilt persists -> more leakage (ruled out empirically).
      => roll channel either unused (flat, off = zero leakage) or full-speed correction (shortest exposure window).
    variant 1 (proportional, archive replay only): w = min(1, |roll_err|/authority) [× yaw budget, default off].
    Default off (returns 1.0) = baseline reproducible; --dart-roll-adaptive 1 enables globally,
    or simul strategy 'dart_latched' per-leg force (force=True). Hysteresis state keyed by strat in args._adp_hyst."""
    if not force and not int(getattr(args, "dart_roll_adaptive", 0) or 0):
        return 1.0
    auth = max(1e-6, float(getattr(args, "dart_roll_authority_deg", 8.0) or 8.0))
    variant = int(getattr(args, "dart_adp_variant", 3) or 3)
    if variant == 3:
        hyst = args.__dict__.setdefault("_adp_hyst", {})
        key = strat or "dart_latched"
        engaged = bool(hyst.get(key))
        if abs(float(roll_err_deg)) >= auth:
            engaged = True
        hyst[key] = engaged
        return 1.0 if engaged else 0.0
    if variant == 2:
        hyst = args.__dict__.setdefault("_adp_hyst", {})
        key = strat or "dart_latched"
        engaged = bool(hyst.get(key))
        err = abs(float(roll_err_deg))
        if err >= auth:
            engaged = True
        elif err <= float(getattr(args, "dart_roll_deadband_deg", 2.0) or 2.0):
            engaged = False
        hyst[key] = engaged
        return 1.0 if engaged else 0.0
    w = min(1.0, abs(float(roll_err_deg)) / auth)
    budget = float(getattr(args, "dart_yaw_budget_deg", 0.0) or 0.0)
    if yaw_err_air is not None and budget > 0:
        ydeg = abs(math.degrees(float(yaw_err_air)))
        w *= max(0.0, 1.0 - ydeg / budget)
    return w

def _c7_adp_reset_latch(args, strat=None) -> None:
    """Flight-latch reset. Prep-level per jump only; **do not** hook on nc≥1 touchdown branch
    (air-impulse launcher nc flicker false-resets -> half-correct bursts observed in tests)."""
    hyst = args.__dict__.get("_adp_hyst")
    if hyst:
        hyst[strat or "dart_latched"] = False

def _diff_nc_touch_roll_steer(roll_d, args, air_cfg, *, nc: int, roll_gain: float,
                              yaw_err_air=None, adp_force=False, strat=None) -> float:
    """Differential on slope contact nc≥1: roll steer only, no pitch differential."""
    if air_cfg.get("active"):
        return _camber_p2_touch_roll_steer(roll_d, args, air_cfg, nc=nc)
    if abs(roll_gain) <= 1e-9:
        return 0.0
    _rcap = float(getattr(args, "diff_roll_steer_max", 1.0) or 1.0)
    w = _c7_adaptive_roll_weight(args, roll_d, yaw_err_air, force=adp_force, strat=strat)
    if abs(float(roll_d)) <= float(getattr(args, "dart_roll_deadband_deg", 2.0)):
        return 0.0
    return max(-_rcap, min(_rcap, w * roll_gain * float(roll_d) / 30.0))

def _camber_early_landmatch(args, *, nc, pz, air_cfg) -> bool:
    """Camber early P1b landmatch on slope contact (default nc≥1 and pz≤7m)."""
    if not air_cfg.get("active"):
        return False
    if int(getattr(args, "camber_air_early_landmatch", 1) or 0) == 0:
        return False
    if int(args.dart_disable_landmatch) or not args.landmatch:
        return False
    return (int(nc) >= int(air_cfg["early_landmatch_nc"])
            and float(pz) <= float(air_cfg["early_landmatch_z"]))

def _c7_air_yaw_hold_steer(steer_roll, yaw_err_air, args, *, steer_cap, air_cfg=None,
                          roll_d=None, nc=0):
    """Airborne heading hold: camber touch with large |roll−γ| yields yaw to roll."""
    if yaw_err_air is None:
        return float(steer_roll)
    cfg = air_cfg if air_cfg is not None else _c7_air_cfg(args)
    if cfg.get("active"):
        _ky = float(cfg["yaw_gain"])
        _ycap = float(cfg["yaw_steer_max"])
        _atten_floor = float(cfg["yaw_roll_atten_floor"])
        _atten_start = float(cfg["yaw_roll_atten_start_deg"])
    else:
        _ky = float(getattr(args, "dart_air_yaw_hold_gain", 0.55) or 0.0)
        _ycap = float(getattr(args, "dart_air_yaw_steer_max", 0.45) or 0.45)
        _atten_floor = 0.2
        _atten_start = 8.0
    if _ky < 1e-9:
        return float(steer_roll)
    _ydeg = abs(math.degrees(float(yaw_err_air)))
    _db = float(getattr(args, "dart_air_yaw_deadband_deg", 2.0) or 2.0)
    sr = float(steer_roll)
    if _ydeg > _atten_start:
        sr *= max(_atten_floor, 1.0 - _ydeg / 45.0)
    if _ydeg < _db:
        return max(-steer_cap, min(steer_cap, sr))
    sy = (-_ky * float(yaw_err_air)) * _approach_steer_vehicle_sign(args)
    if cfg.get("active") and roll_d is not None and int(nc) >= int(cfg.get("early_landmatch_nc", 1)):
        _re = abs(_camber_air_roll_err(roll_d, args, cfg))
        if _re >= float(cfg.get("roll_priority_err_deg", 5.0) or 5.0):
            sy *= float(cfg.get("yaw_atten_on_touch", 0.35) or 0.35)
    sy = max(-_ycap, min(_ycap, sy))
    return max(-steer_cap, min(steer_cap, sr + sy))

def _landed_vehicle_safety_reset(veh, args, *, first_step=False):
    """After landing: clear DART diff per-wheel factors + progressive brake hold.

    Endo fix: full brake+parking on steep fast landing loads front axle -> nose-up/flip
    (visual observation; gspd zero crosses exit gate while flip occurs outside measurement window).
    Progressive: first 1 s brake≤0.4, full brake+parking only when gspd<3 m/s. Metrics sampled at first touch; this affects
    post-landing rollout visuals/next-jump stability only, not published numbers."""
    _set_wheel_factors(veh, 1.0, 1.0, 1.0, 1.0)
    if first_step:
        args.__dict__["_land_safety_t0"] = time.perf_counter()
    _el = time.perf_counter() - float(args.__dict__.get("_land_safety_t0") or time.perf_counter())
    _gspd = float(args.__dict__.get("_land_safety_gspd") or 99.0)
    if _gspd < 3.0:
        _brk, _pb = 1.0, 1.0
    elif _el < 1.0:
        _brk, _pb = 0.4, 0.0
    else:
        _brk, _pb = 0.7, 0.0
    try:
        veh.control(throttle=0.0, brake=_brk, steering=0.0, parkingbrake=_pb)
    except Exception:
        pass
    if first_step:
        print("[CS-land-safety] landed: wheelFactor→1 + progressive brake (endo-safe)", flush=True)

def _camber_stabilize_before_mesh(simul_legs, args, spawn_rot, bng, *, tag=""):
    """Before place_ramp hot-swap: brake + return to spawn to avoid mesh delete after clip/fall-off -> BNG crash."""
    if not simul_legs:
        return
    for leg in simul_legs:
        try:
            leg["veh"].control(throttle=0.0, brake=1.0, steering=0.0, parkingbrake=1.0)
        except Exception:
            pass
    rf._step(bng, 6)
    for leg in simul_legs:
        sx = _leg_base_x(leg, args) - float(args.run_up)
        try:
            leg["veh"].teleport(_leg_spawn_xyz(leg, args), spawn_rot, reset=False)
        except Exception:
            pass
    rf._step(bng, 4)
    print(f"[CS-runup-camber] stabilize→spawn before mesh ({tag})", flush=True)

def _camber_on_runup_apron(px: float, args, *, base_x: float | None = None) -> bool:
    """True = still on flat banked apron; kicker uphill forbids lateral snap (apron z would bury into mesh)."""
    bx = float(base_x if base_x is not None else args.base_x)
    return float(px) < bx - 0.25

def _camber_snap_py_now(veh, lane_y, args, spawn_rot, bng, *, tag="", base_x: float | None = None,
                          leg: dict | None = None):
    """Correct lateral py->lane_y on flat apron; uphill segment lane-keep only, no teleport."""
    if abs(_runup_camber_deg(args)) < 1e-6:
        return
    st = nj._poll(veh)
    pos = st.get("pos") or (0, 0, 0)
    px, py = float(pos[0]), float(pos[1])
    if not _camber_on_runup_apron(px, args, base_x=base_x):
        return
    ly = float(lane_y)
    tol = float(getattr(args, "camber_snap_py_tol", 0.15) or 0.15)
    if leg and leg.get("spawn_anchor_pos") and _spawn_anchor_enabled(args):
        _, ty, tz = _leg_spawn_xyz(leg, args)
    else:
        ty, tz = ly, _spawn_z_for_leg({"y": ly}, args, lane_y=ly)
    if abs(py - ty) <= tol:
        return
    try:
        veh.teleport((px, ty, tz), spawn_rot, reset=False)
    except Exception:
        return
    rf._step(bng, 3)
    print(f"[CS-runup-camber] snap py {py:.2f}→{ty:.1f}m @ px={px:.1f} apron ({tag})", flush=True)

def _camber_approach_apron_snap_if_needed(veh, lane_y, px, gspd, step_i, args, spawn_rot, bng, *,
                                          base_x: float | None = None):
    """Low-speed apron periodic py correction; disabled on kicker uphill (avoid BNG crash)."""
    if abs(_runup_camber_deg(args)) < 1e-6:
        return
    if not _camber_on_runup_apron(px, args, base_x=base_x):
        return
    if float(gspd) > float(getattr(args, "camber_apron_snap_max_v", 11.0) or 11.0):
        return
    stride = int(getattr(args, "camber_apron_snap_stride", 22) or 22)
    if stride <= 0 or step_i <= 0 or (int(step_i) % stride) != 0:
        return
    st = nj._poll(veh)
    py = float((st.get("pos") or (0, 0, 0))[1])
    if abs(py - float(lane_y)) <= float(getattr(args, "camber_snap_py_tol", 0.15) or 0.15):
        return
    _camber_snap_py_now(veh, lane_y, args, spawn_rot, bng, tag=f"apron-i{step_i}", base_x=base_x)

def _camber_snap_lane_center(simul_legs, args, spawn_rot, bng, *, tag=""):
    """Spawn |py-lane_y| over threshold -> teleport back to centerline."""
    if not simul_legs or abs(_runup_camber_deg(args)) < 1e-6:
        return
    tol = float(getattr(args, "camber_snap_py_tol", 0.15) or 0.15)
    for leg in simul_legs:
        st = nj._poll(leg["veh"])
        pos = st.get("pos") or (0, 0, 0)
        py = float(pos[1])
        ly = float(leg["y"])
        if abs(py - ly) <= tol:
            continue
        try:
            leg["veh"].teleport(_leg_spawn_xyz(leg, args), spawn_rot, reset=False)
        except Exception:
            pass
        print(f"[CS-runup-camber] snap py {py:.2f}→{_leg_spawn_xyz(leg, args)[1]:.1f}m @ spawn ({tag})", flush=True)
    rf._step(bng, 6)

def _spawn_z_for_lane(y: float, args, *, base_z: float = 0.3) -> float:
    """Banked run-up (rotate about low-edge pivot_y): road height at lane y ≈ (y - pivot_y)·sin(γ) + clearance.
    Low edge (y=pivot_y) z=0 grounded, center raised (W/2)·sin|γ| -> vehicle on lifted strip, both wheels contact."""
    camber = _runup_camber_deg(args)
    if abs(camber) < 1e-6:
        return base_z
    pivot_y = _camber_pivot_y(args)
    return base_z + (float(y) - pivot_y) * math.sin(math.radians(camber))

def _spawn_z_for_leg(leg, args, *, lane_y: float | None = None) -> float:
    """Simul leg spawn/teleport Z on apron surface.
    y_copy: each ramp rigidly translated, vehicle on copy centerline (local y=0) -> use global y=0 camber height.
    y_lane/x_copy: world lane_y is lateral coordinate on apron."""
    if _simul_y_copy(args):
        return _spawn_z_for_lane(0.0, args)
    y = float(lane_y if lane_y is not None else leg.get("y", 0.0))
    return _spawn_z_for_lane(y, args)

def _spawn_anchor_enabled(args) -> bool:
    return bool(int(getattr(args, "spawn_anchor_use", 1) or 0))

def _spawn_anchor_save_enabled(args) -> bool:
    return bool(int(getattr(args, "spawn_anchor_save", 1) or 0))

def _bind_spawn_anchors(simul_legs, args) -> bool:
    """Load registry anchor for current experiment fingerprint -> leg['spawn_anchor_pos']."""
    if not _spawn_anchor_enabled(args) or not simul_legs:
        return False
    if str(getattr(args, "launch_mode", "")) != "approach":
        return False
    try:
        from control.dart.spawn_anchor import fingerprint_key, fingerprint_from_run, load_anchors
        anchors = load_anchors(args=args, simul_legs=simul_legs)
        if not anchors:
            _bind_spawn_anchors_log_miss(args, simul_legs)
            return False
        fp = fingerprint_from_run(args=args, simul_legs=simul_legs)
        key = fingerprint_key(fp)
        _gamma = _runup_camber_deg(args)
        for leg in simul_legs:
            lab = str(leg.get("label", ""))
            leg["spawn_anchor_pos"] = anchors[lab]
        print(f"[CS-spawn-anchor] loaded proven spawn key={key} γ={_gamma:.1f}° "
              f"legs={ {lab: [round(v, 3) for v in anchors[lab]] for lab in anchors} }",
              flush=True)
        return True
    except Exception as e:
        print(f"[CS-spawn-anchor] WARN load failed: {e!r}", flush=True)
        return False

def _bind_spawn_anchors_log_miss(args, simul_legs) -> None:
    if str(getattr(args, "launch_mode", "")) != "approach" or not _spawn_anchor_enabled(args):
        return
    if abs(_runup_camber_deg(args)) < 1e-6:
        return
    print(f"[CS-spawn-anchor] no proven anchor (γ={_runup_camber_deg(args):.1f}°, "
          f"scenario={getattr(args, 'jump_scenario', '')}) → analytic spawn + settle/snap; "
          f"do not reuse other γ coordinates", flush=True)

def _leg_spawn_xyz(leg, args) -> tuple[float, float, float]:
    """World spawn/teleport XYZ: proven anchor preferred, else analytic formula."""
    ap = leg.get("spawn_anchor_pos")
    formula_z = _spawn_z_for_leg(leg, args)
    if ap and _spawn_anchor_enabled(args):
        ax, ay, az = float(ap[0]), float(ap[1]), float(ap[2])
        if abs(_runup_camber_deg(args)) < 1e-6 and az < formula_z - 0.05:
            az = formula_z
        return ax, ay, az
    sx = _leg_base_x(leg, args) - float(args.run_up)
    return sx, float(leg["y"]), formula_z

def _save_spawn_anchors_from_jump(*, simul_legs, args, res: dict, tag: str, jump_id: int) -> None:
    if not _spawn_anchor_save_enabled(args):
        return
    if all(leg.get("spawn_anchor_pos") for leg in simul_legs):
        return
    leg_positions: dict[str, tuple[float, float, float]] = {}
    for leg in simul_legs:
        lab = str(leg.get("label", ""))
        rec = res.get(lab) or {}
        pos = rec.get("approach_spawn_pos")
        if not pos or len(pos) != 3:
            return
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if abs(_runup_camber_deg(args)) < 1e-6:
            z = _spawn_z_for_leg(leg, args)
        leg_positions[lab] = (x, y, z)
    if len(leg_positions) != len(simul_legs):
        return
    try:
        from control.dart.spawn_anchor import save_anchors, fingerprint_key, fingerprint_from_run
        fp = fingerprint_from_run(args=args, simul_legs=simul_legs)
        key = fingerprint_key(fp)
        path = save_anchors(
            args=args, simul_legs=simul_legs, leg_positions=leg_positions,
            tag=tag, jump_id=jump_id,
        )
        print(f"[CS-spawn-anchor] saved proven spawn key={key} jump={jump_id} -> {path.name}",
              flush=True)
    except Exception as e:
        print(f"[CS-spawn-anchor] WARN save failed: {e!r}", flush=True)

def _build_approach_disturbance_audit(args, *, kick_step: int | None = None) -> dict | None:
    camber = _runup_camber_deg(args)
    spawn_roll = float(getattr(args, "approach_spawn_roll_deg", 0.0) or 0.0)
    kick = float(getattr(args, "lip_roll_rate_kick_dps", 0.0) or 0.0)
    if abs(camber) < 1e-6 and abs(spawn_roll) < 1e-6 and abs(kick) < 1e-6:
        return None
    audit = {
        "runup_camber_deg": round(camber, 2),
        "spawn_roll_deg": round(spawn_roll, 2),
        "lip_roll_rate_kick_dps": round(kick, 2),
    }
    if kick_step is not None:
        audit["kick_step"] = int(kick_step)
    if abs(camber) > 1e-6 and abs(spawn_roll) < 1e-6 and abs(kick) < 1e-6:
        audit["mode"] = "camber"
    elif abs(spawn_roll) > 1e-6 and abs(camber) < 1e-6 and abs(kick) < 1e-6:
        audit["mode"] = "spawn_roll"
    elif abs(kick) > 1e-6 and abs(camber) < 1e-6 and abs(spawn_roll) < 1e-6:
        audit["mode"] = "lip_kick"
    else:
        audit["mode"] = "combo"
    return audit

def landing_slope_segments(peak_x, peak_z, beta_deg, *, length, width, thick, ri, visual=True,
                           post_root_ground=True):
    """Matched downslope landing (A): **lip-continuous** downslope back-face, target landing pitch=beta_deg.

    beta_deg<0 means downslope along +X. Real jumps do not place a downslope patch at a flat landing point,
    but continue a downslope back-face from lip; vehicle lands downstream on that slope ballistically.
    This function starts at (peak_x, peak_z), extends length m by beta_deg, entire slope above grid
    visible and continuous with lip. Score |pitch-beta_deg| not |pitch|.
    """
    if abs(beta_deg) < 1e-6 or length <= 0:
        return []
    theta = math.radians(beta_deg)
    if beta_deg < 0:
        max_visible = max(0.5, (peak_z - 0.05) / abs(math.tan(theta)))
        length = min(float(length), max_visible)
    start_x = peak_x
    end_x = peak_x + float(length)
    z0 = peak_z
    z1 = z0 + (end_x - start_x) * math.tan(theta)
    pts = [(start_x, z0), (end_x, z1)]
    segs = bench.ramp_segments(pts, 100 + ri, width, thick, 1.0)
    for s in segs:
        s["name"] = s["name"].replace(f"r{100 + ri}", f"landing{ri}")
        s["material"] = "track_editor_C_border"
    if not visual:
        return segs

    mid_x = 0.5 * (start_x + end_x)
    mid_z = 0.5 * (z0 + z1)
    slope_len = math.hypot(end_x - start_x, z1 - z0)
    rail_w = 0.45
    for side, y in (("L", width / 2.0 - 1.0), ("R", -width / 2.0 + 1.0)):
        segs.append(_slope_marker(
            f"landing{ri}_rail_{side}", mid_x, mid_z, theta, y=y,
            width=rail_w, along=slope_len, thick=0.35, clearance=0.22))
        for end_name, x, z in (("start", start_x, z0), ("end", end_x, z1)):
            segs.append(_vertical_marker(
                f"landing{ri}_post_{side}_{end_name}", x, z, y=y,
                size=(0.8, 0.8, 1.6), root_ground=post_root_ground))
    return segs

def ballistic_curve(peak_x, peak_z, v0, gamma_deg, clearance, *, g=9.81):
    """Vehicle flight parabola (shifted down clearance) as landing z(dx). Returns (k, slope0, dx_ground, fn_z, fn_tan).

    Parabola (lip origin, dx=x-peak_x): z = peak_z + tanγ·dx - k·dx², k = g/(2·(v0·cosγ)²).
    Landing surface = curve shifted down clearance (~CG-to-wheel vertical), tangential wheel contact.
    dx_ground = where landing surface hits z=0 (SmallGrid); beyond that slope goes underground.
    """
    gx = math.radians(gamma_deg)
    vx = v0 * math.cos(gx)
    vz = v0 * math.sin(gx)
    if vx < 0.1:
        return None
    k = g / (2.0 * vx * vx)
    slope0 = vz / vx
    a, b, c = -k, slope0, peak_z - clearance
    disc = b * b - 4 * a * c
    dx_ground = (-b - math.sqrt(disc)) / (2 * a) if disc > 0 else 0.0

    def fn_z(dx):
        return peak_z - clearance + slope0 * dx - k * dx * dx

    def fn_slope(dx):
        return slope0 - 2.0 * k * dx

    def fn_tan(dx):
        return math.atan(fn_slope(dx))

    return {"k": k, "slope0": slope0, "dx_ground": dx_ground, "vx": vx, "vz": vz,
            "fn_z": fn_z, "fn_slope": fn_slope, "fn_tan": fn_tan}

def landing_hill_profile(peak_z, v0, gamma_deg, clearance, *, face_max_deg=33.0,
                         length=60.0, g=9.81, ds=0.6):
    """Natural dune landing profile z(dx), dx from lip. Three C1-smooth segments:

      segment 1 ballistic face: follows flight parabola (shifted clearance), convex brink + steepening slide,
                  until tangent reaches -face_max_deg (dune angle of repose cap, default 33°);
      segment 2 concave toe runout: arc smooths tangent from -face_max to 0°, tangent to flat at z=0,
                  removes hard "knife into ground" kink.

    Landing surface shifted clearance (z0=peak_z-clearance), start below kicker top by clearance;
    Seam removed by lowering kicker top clearance on main side; this function keeps natural convex downhill shape.
    Returns (pts[(dx,z)], meta). pts monotonic in dx; last point (dx_toe, 0) horizontal tangent.
    """
    bc = ballistic_curve(0.0, peak_z, v0, gamma_deg, clearance, g=g)
    if bc is None:
        return [], {}
    k, slope0 = bc["k"], bc["slope0"]
    face_max = math.radians(abs(face_max_deg))
    s_face = -math.tan(face_max)
    dx_a = (slope0 - s_face) / (2.0 * k)
    z_a = bc["fn_z"](dx_a)
    if z_a <= clearance * 0.5 or dx_a <= 0.0:
        dx_g = bc["dx_ground"]
        n = max(8, int(dx_g / ds))
        pts = [(dx_g * i / n, max(0.0, bc["fn_z"](dx_g * i / n))) for i in range(n + 1)]
        meta = {"mode": "parabola_only", "dx_a": round(dx_a, 2), "z_a": round(z_a, 2),
                "dx_toe": round(dx_g, 2), "face_max_deg": round(face_max_deg, 1)}
        return pts, meta
    R = z_a / (1.0 - math.cos(face_max))
    dx_toe = dx_a + R * math.sin(face_max)
    pts = []
    n1 = max(4, int(dx_a / ds))
    for i in range(n1 + 1):
        dx = dx_a * i / n1
        pts.append((dx, bc["fn_z"](dx)))
    n2 = max(4, int((dx_toe - dx_a) / ds))
    for i in range(1, n2 + 1):
        alpha = face_max * (1.0 - i / n2)
        x = dx_toe - R * math.sin(alpha)
        z = R * (1.0 - math.cos(alpha))
        pts.append((x, max(0.0, z)))
    if dx_toe > length:
        pts = [(dx, z) for (dx, z) in pts if dx <= length]
    meta = {"mode": "parabola+toe", "dx_a": round(dx_a, 2), "z_a": round(z_a, 2),
            "R_toe": round(R, 2), "dx_toe": round(dx_toe, 2),
            "face_max_deg": round(face_max_deg, 1),
            "z_start": round(bc["fn_z"](0.0), 3)}
    return pts, meta

def ballistic_landing_segments(peak_x, peak_z, v0, gamma_deg, *, clearance, length,
                               width, thick, ri, visual=True, g=9.81, face_max_deg=33.0,
                               post_root_ground=True):
    """Natural dune ballistic landing: upper ballistic face (tangential soft landing + convex brink), lower concave runout to flat.

    Replaces unnatural straight downslope (cannot match parabola) and "parabola stab into ground" (hard kink).
    Returns (segs, pts_xz, meta).
    """
    prof, meta = landing_hill_profile(peak_z, v0, gamma_deg, clearance,
                                      face_max_deg=face_max_deg, length=length, g=g)
    if not prof or len(prof) < 2:
        return [], [], {}
    pts = [(peak_x + dx, z) for (dx, z) in prof]
    segs = bench.ramp_segments(pts, 200 + ri, width, thick, 1.0)
    for s in segs:
        s["name"] = s["name"].replace(f"r{200 + ri}", f"balland{ri}")
        s["material"] = "track_editor_C_border"
    meta = dict(meta)
    meta.update({"vx": round(bc_vx(v0, gamma_deg), 2), "n_seg": len(segs)})
    if not visual:
        return segs, pts, meta
    for side, y in (("L", width / 2.0 - 1.0), ("R", -width / 2.0 + 1.0)):
        for j in range(0, len(pts), max(1, len(pts) // 6)):
            x, z = pts[j]
            segs.append(_vertical_marker(f"balland{ri}_post_{side}_{j}", x, z, y=y,
                                         size=(0.7, 0.7, 1.4), root_ground=post_root_ground))
    return segs, pts, meta

def gap_landing_profile(peak_z, v0, gamma_deg, *, clearance_catch=0.4, gap_max=2.5,
                        shape_p=1.5, face_max_deg=33.0, feather_len=6.5,
                        length=80.0, g=9.81, ds=0.5):
    """True jump landing profile (gap mode): from lip, smooth **unimodal-slope** landing hill (natural hill / ski-jump style),
    Vehicle flies freely ~1.5 s above then lands at toe. Profile z(dx), dx from lip.

    Slope σ(t) = σ_max·W(t) (t=dx/L, unimodal: ends 0, mid peak) -> z(dx)=peak_z-∫tan(σ)ds:
      - unimodal slope, no reversal -> monotonic descent, no slope inflection -> **no ridge**;
      - W->0 at ends -> smooth lip join, natural feather to ground, no hard kink or separate runout;
      - L = ballistic landing (T=0), toe at vehicle landing; σ_max solves drop = peak_z;
      - shape_p shifts slope peak: =1 symmetric (smoothest), >1 peak right (steeper landing end), <1 peak left (steeper apex).
    Note: no tangential soft landing (gap vs tangential conflict on single surface; ω_y0 overspin dominates landing),
        traded for ridge-free natural hill shape + true gap. Returns (pts[(dx,z)], meta).
    """
    gx = math.radians(gamma_deg)
    vx = v0 * math.cos(gx); vz = v0 * math.sin(gx)
    if vx < 0.1:
        return [], {}
    k = g / (2.0 * vx * vx); slope0 = vz / vx

    def T(dx):
        return peak_z + slope0 * dx - k * dx * dx

    L = (slope0 + math.sqrt(slope0 * slope0 + 4.0 * k * peak_z)) / (2.0 * k)
    if L <= 0:
        return [], {}

    def W(t):
        tt = min(1.0, max(0.0, t)) ** shape_p
        return math.sin(math.pi * tt)

    nn = max(60, int(L / ds))
    def drop(sm_deg):
        sm = math.radians(sm_deg); s = 0.0
        for i in range(nn):
            s += math.tan(sm * W((i + 0.5) / nn)) * (L / nn)
        return s
    lo, hi = 1.0, 80.0
    for _ in range(44):
        mid = 0.5 * (lo + hi)
        if drop(mid) < peak_z:
            lo = mid
        else:
            hi = mid
    sigma_max = 0.5 * (lo + hi)
    sm_rad = math.radians(sigma_max)
    pts = [(0.0, peak_z)]; z = peak_z; max_face = 0.0
    for i in range(1, nn + 1):
        sloc = sm_rad * W((i - 0.5) / nn)
        max_face = max(max_face, sloc)
        z -= math.tan(sloc) * (L / nn)
        pts.append((L * i / nn, max(0.0, z)))
    if pts[-1][1] > 1e-3:
        pts.append((pts[-1][0] + 1.0, 0.0))
    if length > pts[-1][0] + 1e-6:
        pts.append((float(length), 0.0))
    pts = [(dx, zz) for (dx, zz) in pts if dx <= length]
    gap_samples = [T(dx) - zz for (dx, zz) in pts if dx < L - 0.5]
    land_slope_deg = -round(math.degrees(sm_rad * W(0.98)), 1)
    feather_actual = 0.0
    meta = {"mode": "gap_valley", "dx_land": round(L, 2),
            "z_land": 0.0, "land_slope_deg": land_slope_deg,
            "gap_max_actual": round(max(gap_samples), 2) if gap_samples else 0.0,
            "gap_min_flight": round(min(gap_samples), 2) if gap_samples else 0.0,
            "sigma_max_deg": round(sigma_max, 1), "max_face_deg": round(math.degrees(max_face), 1),
            "shape_p": shape_p, "clearance_catch": clearance_catch,
            "dx_toe": round(pts[-1][0], 2), "z_start": round(peak_z, 2)}
    return pts, meta

def gap_landing_segments(peak_x, peak_z, v0, gamma_deg, *, clearance_catch, gap_max,
                         shape_p, feather_len, length, width, thick, ri, visual=True, g=9.81,
                         face_max_deg=33.0, post_root_ground=True):
    """Gap-mode landing (steep valley + tangential catch + lower feather). Returns (segs, pts_xz, meta)."""
    prof, meta = gap_landing_profile(peak_z, v0, gamma_deg, clearance_catch=clearance_catch,
                                     gap_max=gap_max, shape_p=shape_p, feather_len=feather_len,
                                     face_max_deg=face_max_deg, length=length, g=g)
    if not prof or len(prof) < 2:
        return [], [], {}
    pts = [(peak_x + dx, z) for (dx, z) in prof]
    segs = bench.ramp_segments(pts, 300 + ri, width, thick, 1.0)
    for s in segs:
        s["name"] = s["name"].replace(f"r{300 + ri}", f"gapland{ri}")
        s["material"] = "track_editor_C_border"
    meta = dict(meta); meta.update({"vx": round(bc_vx(v0, gamma_deg), 2), "n_seg": len(segs)})
    if not visual:
        return segs, pts, meta
    for side, y in (("L", width / 2.0 - 1.0), ("R", -width / 2.0 + 1.0)):
        for j in range(0, len(pts), max(1, len(pts) // 6)):
            x, z = pts[j]
            segs.append(_vertical_marker(f"gapland{ri}_post_{side}_{j}", x, z, y=y,
                                         size=(0.7, 0.7, 1.4), root_ground=post_root_ground))
    return segs, pts, meta

def gap_ramp_landing_segments(peak_x, peak_z, v0, gamma_deg, beta_deg, *, clearance,
                              gap_run, length, width, thick, ri, visual=True, g=9.81,
                              post_root_ground=True):
    """Gap + tilted catch slope (downslope generalization): vehicle crosses **air gap (true free flight gap_run m)** from lip,
    lands on **straight downslope catch top** at angle β (nose-down); DART must match pitch to β = tests airborne pitch authority on steeper landing.

    vs existing modes (confirmed): gap lands on ~0° feathered valley floor (target_pitch≈0, no slope-steepness test);
    ballistic follows trajectory without gap. This mode fills gap: true air gap + tilted landing surface.

    Geometry (gap_run decoupled; tangential placement covers trajectory, discarded): catch **top** at ballistic dx=gap_run
    (z_top=trajectory height-clearance, wheels touch at top), slope extends down at β. **β independent of landing point ->
    sweep β at fixed airtime (=gap_run/vx), decouple steepness from airtime**; gap_run is separate length/airtime axis.
    lip->ramp top is air gap (no mesh, true free flight; short fall into gap = fail = real envelope).
    Feasibility: gap_run past apex (descent leg) and z_top>0, else infeasible. Returns (segs, pts, meta)."""
    cur = ballistic_curve(peak_x, peak_z, v0, gamma_deg, 0.0, g=g)
    if cur is None:
        return [], [], {"infeasible": True, "reason": "vx~0"}
    slope0 = cur["slope0"]; k = cur["k"]; fn_z = cur["fn_z"]; fn_slope = cur["fn_slope"]
    apex_dx = slope0 / (2.0 * k)
    if gap_run <= apex_dx + 0.5:
        return [], [], {"infeasible": True, "reason": "gap_run_before_apex",
                        "gap_run": round(gap_run, 2), "apex_dx": round(apex_dx, 2)}
    z_top = fn_z(gap_run) - clearance
    if z_top <= 0.05:
        return [], [], {"infeasible": True, "reason": "ramp_top_below_ground",
                        "gap_run": round(gap_run, 2), "z_top": round(z_top, 2)}
    x_top = peak_x + gap_run
    segs = landing_slope_segments(x_top, z_top, -abs(beta_deg), length=length,
                                  width=width, thick=thick, ri=ri, visual=visual,
                                  post_root_ground=post_root_ground)
    pts = [(peak_x, peak_z), (x_top, z_top)]
    meta = {"mode": "gap_ramp", "beta_deg": -abs(beta_deg), "land_slope_deg": -abs(beta_deg),
            "dx_land": round(gap_run, 2), "x_land": round(x_top, 2), "z_land": round(z_top, 2),
            "airtime_s": round(gap_run / cur["vx"], 3), "gap_run": round(gap_run, 2),
            "traj_slope_at_land_deg": round(math.degrees(math.atan(fn_slope(gap_run))), 1),
            "v0_design": round(v0, 2), "gamma_deg": gamma_deg, "vx": round(cur["vx"], 2),
            "n_seg": len(segs)}
    return segs, pts, meta

def _valley_crest_pts(peak_z, lip_exit_slope_deg, beta_deg, dx_c, n=None):
    """Valley lip rounding bulge (gap-style smooth join): quadratic Bezier on [0,dx_c] blends lip exit slope s0
    smooth transition to -β, **bulges above -β straight line**, end (dx_c, peak_z-tanβ·dx_c) tangent-merge to -β.

    Key: -β line still through lip -> landing/feather/floor/auto-rise use original through-lip closed form; crest is lip rounding only,
    landing geometry unchanged (vehicle lands on constant -β line beyond dx_c, flies over crest). Returns crest points [(dx,z)] (t>0, not dx=0)."""
    s0 = math.radians(max(0.0, float(lip_exit_slope_deg)))
    br = math.radians(abs(float(beta_deg))); tb = math.tan(br)
    if dx_c <= 0.1:
        return []
    if n is None:
        n = max(24, int(dx_c / 0.15))
    P0 = (0.0, peak_z)
    P3 = (dx_c, peak_z - tb * dx_c)
    h = dx_c
    B1 = (P0[0] + (h / 3.0) * math.cos(s0), P0[1] + (h / 3.0) * math.sin(s0))
    B2 = (P3[0] - (h / 3.0) * math.cos(br), P3[1] - (h / 3.0) * (-math.sin(br)))
    pts = []
    for i in range(1, n + 1):
        t = i / n; u = 1.0 - t
        x = u**3 * P0[0] + 3 * u * u * t * B1[0] + 3 * u * t * t * B2[0] + t**3 * P3[0]
        z = u**3 * P0[1] + 3 * u * u * t * B1[1] + 3 * u * t * t * B2[1] + t**3 * P3[1]
        pts.append((x, z))
    return pts

def valley_landing_segments(peak_x, peak_z, v0, gamma_deg, beta_deg, *, clearance,
                            floor_depth, length_req, width, thick, ri, visual=True, g=9.81,
                            feather_len=6.0, floor_run=14.0, lip_exit_slope_deg=0.0,
                            crest_blend_len=6.0, post_root_ground=True):
    """Continuous downslope landing (valley: baseline slope + feathered floor + convex lip join): from lip through
    **convex crest** (lip exit->-β, gap-style end slope fade, removes lip ridge kink) -> **constant -β straight slope**
    (vehicle lands ballistically; DART matches pitch to β = tests landing-slope airborne pitch authority) -> **concave toe feather** (tangent -β->0°)
    -> **flat valley floor** (anchored at smallgrid z=0, no underground drop-off).

    Four C1 segments: 1) convex crest (s0->-β) 2) straight -β (covers landing + margin) 3) concave toe (R=feather_len/sinβ, -β->0°)
    4) flat floor (floor_run). Crest shape independent of peak_z -> landing still closed form. floor_depth kept for compat (no dig currently).
    Returns (segs, pts, meta)."""
    beta = abs(float(beta_deg))
    br = math.radians(beta)
    tb = math.tan(br)
    if tb < 1e-4:
        return [], [], {"infeasible": True, "reason": "beta~0"}
    cur = ballistic_curve(peak_x, peak_z, v0, gamma_deg, clearance, g=g)
    if cur is None:
        return [], [], {"infeasible": True, "reason": "vx~0"}
    k = cur["k"]; slope0 = cur["slope0"]
    disc = (slope0 + tb) ** 2 - 4.0 * k * clearance
    if disc <= 0:
        return [], [], {"infeasible": True, "reason": "no_landing_crossing"}
    dx_land = ((slope0 + tb) + math.sqrt(disc)) / (2.0 * k)
    R = max(2.0, float(feather_len)) / max(math.sin(br), 1e-3)
    dz_fillet = R * (1.0 - math.cos(br))
    dx_straight = max(float(length_req or 0.0), dx_land + 6.0)
    z_straight = peak_z - dx_straight * tb
    floor_z = z_straight - dz_fillet
    if floor_z < 0.0:
        floor_z = 0.0
        z_straight = dz_fillet
        dx_straight = max(0.5, (peak_z - z_straight) / tb)
    dx_toe = dx_straight + R * math.sin(br)
    dx_c = min(float(crest_blend_len), max(0.0, dx_straight - 1.0))
    crest_pts = _valley_crest_pts(peak_z, lip_exit_slope_deg, beta, dx_c)
    ds = 0.8
    prof = [(0.0, peak_z)] + list(crest_pts)
    n1 = max(2, int((dx_straight - dx_c) / ds))
    for i in range(1, n1 + 1):
        dx = dx_c + (dx_straight - dx_c) * i / n1
        prof.append((dx, peak_z - dx * tb))
    n2 = max(3, int((dx_toe - dx_straight) / ds))
    for i in range(1, n2 + 1):
        alpha = br * (1.0 - i / n2)
        prof.append((dx_toe - R * math.sin(alpha), floor_z + R * (1.0 - math.cos(alpha))))
    prof.append((dx_toe + float(floor_run), floor_z))
    pts = [(peak_x + dx, z) for (dx, z) in prof]
    segs = bench.ramp_segments(pts, 100 + ri, width, thick, 1.0)
    for s in segs:
        s["name"] = s["name"].replace(f"r{100 + ri}", f"valley{ri}")
        s["material"] = "track_editor_C_border"
    total_len = dx_toe + float(floor_run)
    meta = {"mode": "valley", "beta_deg": -beta, "land_slope_deg": -beta,
            "dx_land": round(dx_land, 2), "x_land": round(peak_x + dx_land, 2),
            "z_land": round(peak_z - dx_land * tb, 2), "airtime_s": round(dx_land / cur["vx"], 3),
            "floor_z": round(floor_z, 2), "R_toe": round(R, 2), "dx_toe": round(dx_toe, 2),
            "dx_crest": round(dx_c, 2), "lip_exit_slope_deg": round(float(lip_exit_slope_deg), 1),
            "feather_len": round(R * math.sin(br), 2), "floor_run": round(float(floor_run), 2),
            "length": round(total_len, 2), "v0_design": round(v0, 2),
            "gamma_deg": gamma_deg, "vx": round(cur["vx"], 2), "n_seg": len(segs)}
    if dx_straight < dx_land + 0.05:
        meta["warn"] = "straight_short_land_on_fillet"
    if visual and total_len > 1.0:
        segs = _append_ramp_rail_visuals(segs, pts, prefix="valley", ri=ri, width=width,
                                         post_root_ground=post_root_ground)
    return segs, pts, meta

def valley_min_peak_z(v0, gamma_deg, beta_deg, *, clearance, feather_len, margin=1.0, g=9.81):
    """Valley mode: closed-form minimum lip height peak_z so ballistic landing hits constant -β line (not fillet arc).

    -β line through lip (crest rounding only). Floor at z=0, landing+6 m floor_z>=margin:
      peak_z >= (dx_land+6)·tanβ + R(1-cosβ) + margin. Returns required peak_z (m), 0 if infeasible."""
    beta = abs(float(beta_deg)); br = math.radians(beta); tb = math.tan(br)
    if tb < 1e-4:
        return 0.0
    cur = ballistic_curve(0.0, 10.0, v0, gamma_deg, clearance, g=g)
    if cur is None:
        return 0.0
    k, slope0 = cur["k"], cur["slope0"]
    disc = (slope0 + tb) ** 2 - 4.0 * k * clearance
    if disc <= 0:
        return 0.0
    dx_land = ((slope0 + tb) + math.sqrt(disc)) / (2.0 * k)
    R = max(2.0, float(feather_len)) / max(math.sin(br), 1e-3)
    dz_fillet = R * (1.0 - math.cos(br))
    return (dx_land + 6.0) * tb + dz_fillet + float(margin)

def valley_airtime_for_v0(v0, gamma_deg, beta_deg, clearance, g=9.81):
    """Valley flight time on constant -β line T=dx_land/vx (closed form). Independent of peak_z (only γ/β/clearance/v0).
    Returns None if no landing intersection (disc<=0) or vx~0."""
    br = math.radians(abs(float(beta_deg))); tb = math.tan(br)
    cur = ballistic_curve(0.0, 10.0, v0, gamma_deg, clearance, g=g)
    if cur is None:
        return None
    k, slope0, vx = cur["k"], cur["slope0"], cur["vx"]
    if vx < 1e-6:
        return None
    disc = (slope0 + tb) ** 2 - 4.0 * k * clearance
    if disc <= 0:
        return None
    dx_land = ((slope0 + tb) + math.sqrt(disc)) / (2.0 * k)
    return dx_land / vx

def solve_v0_for_airtime(target_T, gamma_deg, beta_deg, clearance, *, lo=4.0, hi=45.0, iters=60):
    """Invert v0 so valley flight time ≈ target_T (bisection; airtime monotone in v0).
    Returns (v0, achieved_T, clamped). Clamps to bounds if target outside [lo,hi] reachable range."""
    t_lo = valley_airtime_for_v0(lo, gamma_deg, beta_deg, clearance)
    t_hi = valley_airtime_for_v0(hi, gamma_deg, beta_deg, clearance)
    if t_lo is not None and target_T <= t_lo:
        return lo, t_lo, True
    if t_hi is not None and target_T >= t_hi:
        return hi, t_hi, True
    a, b = lo, hi
    for _ in range(iters):
        mid = 0.5 * (a + b)
        at = valley_airtime_for_v0(mid, gamma_deg, beta_deg, clearance)
        if at is None:
            a = mid; continue
        if at < target_T:
            a = mid
        else:
            b = mid
    v0 = 0.5 * (a + b)
    return v0, valley_airtime_for_v0(v0, gamma_deg, beta_deg, clearance), False

def bc_vx(v0, gamma_deg):
    return v0 * math.cos(math.radians(gamma_deg))

def set_track_cam(bng, sm, cam_z):
    """Landing follow camera (de-jitter): EMA-smooth vehicle x/y + **fixed cam_z** (no vertical bob on takeoff/landing
    -> removes vertical jitter); lookAt follows smoothed vehicle (centered). Side 18 m (large vehicle in frame). Returns cam-vehicle distance."""
    px, py, pz = sm
    cam = (px - 3.0, py - 18.0, cam_z)
    dx, dy, dz = px - cam[0], py - cam[1], pz - cam[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    try:
        bng.camera.set_free(cam, (dx / n, dy / n, dz / n)); return round(n, 1)
    except Exception:
        return None

def _aim(cam, tgt):
    dx, dy, dz = tgt[0] - cam[0], tgt[1] - cam[1], tgt[2] - cam[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    return (dx / n, dy / n, dz / n)

def _frust_ang(cam, look, tx, tz):
    """Angle (deg) between target (tx,0,tz) and camera axis; <~28° treated in frustum (default FOV~60°)."""
    vx, vy, vz = tx - cam[0], 0.0 - cam[1], tz - cam[2]
    vn = math.sqrt(vx * vx + vy * vy + vz * vz) or 1.0
    c = (vx * look[0] + vy * look[1] + vz * look[2]) / vn
    return math.degrees(math.acos(max(-1, min(1, c))))

def _simul_lateral_y_span(simul_legs) -> float:
    if not simul_legs:
        return 0.0
    ys = [float(l.get("y", 0.0)) for l in simul_legs]
    return max(ys) - min(ys) if len(ys) > 1 else 0.0

def _simul_cam_look_y(simul_legs) -> float:
    if not simul_legs:
        return 0.0
    ys = [float(l.get("y", 0.0)) for l in simul_legs]
    return 0.5 * (min(ys) + max(ys)) if ys else 0.0

def _simul_cam_look_y_ab(simul_legs, args) -> float:
    """AB approach camera aim: y_copy tracks dart red lane, other layouts track lateral center."""
    if _simul_y_copy(args) and simul_legs:
        for leg in simul_legs:
            if str(leg.get("label", "")) == "dart" or str(leg.get("strategy", "")) == "dart":
                return float(leg["y"])
        return float(simul_legs[0]["y"])
    return _simul_cam_look_y(simul_legs)

def _framed_side_cam(framed_lo, framed_hi, rise, *, look_z=None, look_y: float = 0.0,
                     lateral_y_span: float = 0.0, side_extra_pullback: float = 0.0):
    """Side framed camera: horizontal frame [framed_lo, framed_hi], side distance/height scale with span (same family as C).
    lateral_y_span>0 (y_copy/y_lane): extra side pullback to frame lateral copies/lanes."""
    framed_span = max(1.0, float(framed_hi) - float(framed_lo))
    look_x = 0.5 * (float(framed_lo) + float(framed_hi))
    if look_z is None:
        look_z = max(1.0, float(rise) * 0.4)
    cx = look_x
    _lat = max(0.0, float(lateral_y_span))
    cy = -max(22.0, 0.72 * framed_span) - 0.45 * _lat - max(0.0, float(side_extra_pullback))
    cz = float(rise) + max(9.0, 0.36 * framed_span) + 0.08 * _lat
    cam = (cx, cy, cz)
    look = _aim(cam, (look_x, float(look_y), look_z))
    return cam, look, look_x, framed_span

def _cam_c_z_drop_from_args(args) -> float:
    """air-impulse uses C camera throughout; approach switches to C after takeoff without extra drop."""
    if str(getattr(args, "launch_mode", "")) != "air-impulse":
        return 0.0
    return max(0.0, float(getattr(args, "cam_air_impulse_z_drop_m", 10.0)))

_DATA_POSTLAND_HOLD_SEC = 0.3
_DATA_POSTFAIL_HOLD_SEC = 0.2

def _hud_on(args) -> bool:
    return bool(int(getattr(args, "hud", 0)))

def _cam_update_due(step_i: int, args, *, phase_key: str | None = None) -> bool:
    """Decimate bng.camera.set_free calls (--cam-update-every N, default 1=every step).

    Phase AB->C switch always triggers an immediate update so takeoff is not missed visually.
    """
    every = max(1, int(getattr(args, "cam_update_every", 1) or 1))
    if every <= 1:
        return True
    if phase_key is not None:
        last = args.__dict__.get("_cam_last_phase")
        if last != phase_key:
            args.__dict__["_cam_last_phase"] = phase_key
            return True
    return int(step_i) % every == 0

def _postland_hold_sec(args, hud_on=None) -> float:
    """one_jump post-landing hold (wall clock): hud=0 data mode fast collect 0.3 s."""
    if hud_on is None:
        hud_on = _hud_on(args)
    if hud_on:
        return float(getattr(args, "postland_hold_sec", 3.5))
    return _DATA_POSTLAND_HOLD_SEC

def _postfail_hold_sec(args, hud_on=None) -> float:
    """Early-fail hold: hud=0 data mode fast collect 0.2 s (one_jump=wall clock, simul3=sim steps)."""
    if hud_on is None:
        hud_on = _hud_on(args)
    if hud_on:
        return float(getattr(args, "postfail_hold_sec", 3.5))
    return _DATA_POSTFAIL_HOLD_SEC

def _post_land_sec_simul3(args) -> float:
    """simul3 post-landing min sim seconds: hud=0 data mode 0.3 s; trace mode ≥5 s (with gspd gate)."""
    if int(getattr(args, "control_trace", 0)):
        return max(5.0, float(getattr(args, "post_land_sec", 3.5)))
    if _hud_on(args):
        return float(getattr(args, "post_land_sec", 3.5))
    return _DATA_POSTLAND_HOLD_SEC

def _post_land_gspd_gate_mps(args) -> float:
    """Post-landing early-exit gate: exit only when all landed vehicles gspd≤this (full-brake park target)."""
    return max(0.0, float(getattr(args, "post_land_gspd_gate_mps", 2.0) or 2.0))

def _post_land_max_sec_simul3(args) -> float:
    """Post-landing hold hard cap (prevent gspd gate never satisfied deadlock)."""
    _base = _post_land_sec_simul3(args)
    _cap = float(getattr(args, "post_land_max_sec", 0.0) or 0.0)
    if _cap > _base:
        return _cap
    return _base + (15.0 if int(getattr(args, "control_trace", 0)) else 5.0)

def build_phase_cams(base_x, run_up, peak_x, R, rise, land_far_x=None, c_span_cap=32.0,
                     approach_post_lip_m=2.0, approach_pre_start_m=2.0,
                     approach_cam_z_drop_m=8.0, cam_c_z_drop_m=0.0,
                     *, look_y: float = 0.0, look_y_ab: float | None = None,
                     lateral_y_span: float = 0.0, ab_side_extra_pullback_m: float = 0.0):
    """Fixed cameras (static within phase = zero jitter):
    AB (approach run-up+climb, merged): frame [start_x-pre_start, peak_x+post_lip] shows start and post-lip meters;
    C (post-takeoff / full air-impulse): unified [peak_x, land_far_x] frame (valley/gap/fixed family, no fallback close-up).
    c_span_cap: C frame right bound = peak_x + min(land_far_x-peak_x, cap) + 6 m margin."""
    start_x = base_x - run_up
    post_lip = max(2.0, float(approach_post_lip_m))
    ab_lo = float(start_x) - max(1.0, float(approach_pre_start_m))
    ab_hi = float(peak_x) + post_lip
    ab_look_z = max(1.0, float(rise) * 0.45)
    _ly_ab = float(look_y if look_y_ab is None else look_y_ab)
    cam_ab, look_ab, ab_look_x, ab_span = _framed_side_cam(
        ab_lo, ab_hi, rise, look_z=ab_look_z, look_y=_ly_ab, lateral_y_span=lateral_y_span,
        side_extra_pullback=ab_side_extra_pullback_m)
    _z_drop = max(0.0, float(approach_cam_z_drop_m))
    if _z_drop > 0.0:
        cam_ab = (cam_ab[0], cam_ab[1], cam_ab[2] - _z_drop)
        look_ab = _aim(cam_ab, (ab_look_x, _ly_ab, ab_look_z))
    cam_a, look_a = cam_ab, look_ab
    cam_b, look_b = cam_ab, look_ab

    if land_far_x is None or float(land_far_x) <= float(peak_x):
        land_far_x = float(peak_x) + max(float(R), 8.0)
    toe_span = float(land_far_x) - float(peak_x)
    view_span = min(toe_span, float(c_span_cap)) + 6.0
    left_margin = max(10.0, 0.28 * view_span)
    framed_lo = float(peak_x) - left_margin
    framed_hi = float(peak_x) + view_span
    c_look_z = 1.0
    cam_c, look_c, land_x, _ = _framed_side_cam(
        framed_lo, framed_hi, rise, look_z=c_look_z, look_y=look_y, lateral_y_span=lateral_y_span)
    _c_z_drop = max(0.0, float(cam_c_z_drop_m))
    if _c_z_drop > 0.0:
        cam_c = (cam_c[0], cam_c[1], cam_c[2] - _c_z_drop)
        look_c = _aim(cam_c, (land_x, float(look_y), c_look_z))

    cams = {"AB": (cam_ab, look_ab), "A": (cam_a, look_a), "B": (cam_b, look_b), "C": (cam_c, look_c)}
    metas = {
        "AB": {"z": round(cam_ab[2], 1), "framed_lo": round(ab_lo, 1), "framed_hi": round(ab_hi, 1),
               "post_lip_m": round(post_lip, 1), "pre_start_m": round(float(approach_pre_start_m), 1),
               "z_drop_m": round(_z_drop, 1), "look_y": round(_ly_ab, 1),
               "side_extra_pullback_m": round(float(ab_side_extra_pullback_m), 1),
               "ang_start": round(_frust_ang(cam_ab, look_ab, start_x, 1.0), 1),
               "ang_peak": round(_frust_ang(cam_ab, look_ab, peak_x, rise), 1),
               "ang_post_lip": round(_frust_ang(cam_ab, look_ab, peak_x + post_lip, rise * 0.5), 1)},
        "A": {"z": round(cam_a[2], 1), "ang_target": round(_frust_ang(cam_a, look_a, ab_look_x, 1.0), 1),
              "merged_with": "AB"},
        "B": {"z": round(cam_b[2], 1), "ang_takeoff": round(_frust_ang(cam_b, look_b, peak_x, rise), 1),
              "merged_with": "AB"},
        "C": {"z": round(cam_c[2], 1), "ang_land": round(_frust_ang(cam_c, look_c, land_x, 0.5), 1),
              "ang_peak": round(_frust_ang(cam_c, look_c, peak_x, rise), 1),
              "land_x": round(land_x, 1), "land_far_x": round(float(land_far_x), 1),
              "view_span": round(view_span, 1), "z_drop_m": round(_c_z_drop, 1)},
    }
    return cams, metas, land_x

def set_hud(bng, lines):
    """Screen-space fixed HUD: guihooks 'ScenarioRealtimeDisplay' — corner-fixed scenario panel,
    in-place update (resend replaces, no stack), not world/vehicle tied, no flicker (persistent UI). Multi-line via <br>. Low rate OK."""
    txt = "<br>".join(str(x).replace("'", "").replace("\\", "") for x in lines)
    html = f'<span style="font-size:62%; line-height:1.05">{txt}</span>'
    try: bng.queue_lua_command(f"guihooks.trigger('ScenarioRealtimeDisplay', {{msg = '{html}'}})")
    except Exception: pass

def clear_hud(bng):
    try: bng.queue_lua_command("guihooks.trigger('ScenarioRealtimeDisplay', {msg = ''})")
    except Exception: pass

def hud(bng, msg):
    """Screen caption (beamngpy ui.display_message); periodic resend for persistent live update."""
    try: bng.ui.display_message(msg)
    except Exception:
        try: bng.display_gui_message(msg)
        except Exception: pass

STEER_MAX_DEG = 41.0

def _prepare_takeoff_segments(segs, args):
    """Apply --runup-ground-type to takeoff ramp + optional run-up pad (landing mesh unchanged)."""
    gt = str(getattr(args, "runup_ground_type", "ASPHALT") or "ASPHALT").upper()
    apply_ramp_material(segs, gt)
    if abs(_runup_camber_deg(args)) > 1e-6:
        pad = []
    else:
        pad = build_runup_pad_segments(
            base_x=float(args.base_x),
            run_up=float(args.run_up),
            width=float(args.width),
            thick=float(args.thick),
            ground_type=gt,
            margin_back=float(getattr(args, "runup_pad_margin_back", 11.0) or 11.0),
            margin_front=float(getattr(args, "runup_pad_margin_front", 1.5) or 1.5),
        )
    audit = runup_ground_audit(gt)
    if pad:
        _p = pad[0]["pos"][0]
        _half = pad[0]["size"][1] * 0.5
        print(
            f"[CS-runup-ground] type={gt} material={audit['material']} "
            f"μs≈{audit['nominal_static_friction']} unified_pad={len(pad)} "
            f"x=[{_p - _half:.1f},{_p + _half:.1f}]m (flat prepend=OFF, visual+contact=same ramp material)",
            flush=True,
        )
    args.__dict__["_runup_ground_audit"] = audit
    return pad + segs

def build_ramp_pts(args, base_x, angle):
    """Dispatch ramp polyline by --ramp-mode. tabletop=original convex crest with abrupt lip; kicker=straight face + controlled lip arc (B1).

    kicker lip radius: use --lip-radius>0 directly; if ==0 auto-compute from --lip-omega-target-dps
    R_lip=v_peak/|omega_target| (v_peak from ramp_speeds takeoff speed for this angle)."""
    mode = getattr(args, "ramp_mode", "tabletop")
    if mode == "kicker":
        R_lip = float(args.lip_radius)
        if R_lip <= 0.0:
            vp, _vb, _R = bench.ramp_speeds(args.rise)
            v_peak = vp[ANGLES.index(angle)] if angle in ANGLES else vp[0]
            omega = max(1e-3, abs(math.radians(args.lip_omega_target_dps)))
            R_lip = v_peak / omega
        efl = float(args.entry_fillet_len)
        r_min = float(getattr(args, "entry_fillet_radius", 0.0) or 0.0)
        n_entry = int(args.n_entry)
        if r_min > 0.0:
            efl_adapt = r_min * math.sin(math.radians(angle))
            if efl_adapt > efl:
                n_entry = max(n_entry, int(math.ceil(n_entry * efl_adapt / max(efl, 1e-6))))
                efl = efl_adapt
        return bench.kicker_polyline(
            base_x, angle, args.rise, efl, R_lip,
            args.lip_sweep_deg, n_entry, args.n_straight, args.n_lip, args.sink), R_lip
    fl = tp.fillet_len_for(angle, args.fillet_rise_budget, args.fillet_len_max)
    return bench.ramp_polyline(base_x, angle, args.rise, fl, args.n_fillet,
                               args.n_main, args.sink), None

def respawn_ego(bng, old_veh, pos, rot, Vehicle, Electrics):
    """Fresh ego each roll: despawn old + spawn new (full EV quad-motor/wheel-speed/gear reset,
    fixes "post-landing drivetrain stuck -> next roll loss-of-drive on ramp"). Returns new veh handle.
    On failure (API glitch) fall back to old veh (caller still has teleport reset fallback)."""
    try:
        try: bng.vehicles.despawn(old_veh)
        except Exception: pass
        rf._step(bng, 2)
        nv = Vehicle("ego", model="sbr", part_config="vehicles/sbr/dart_4motor.pc")
        try: nv.attach_sensor("electrics", Electrics())
        except Exception: pass
        bng.vehicles.spawn(nv, tuple(pos), rot_quat=tuple(rot), cling=True, connect=True)
        rf._step(bng, 3)
        try: bng.vehicles.switch(nv)
        except Exception: pass
        return nv
    except Exception as e:
        print(f"[CS] respawn_ego failed, falling back to old veh teleport: {e}", flush=True)
        return old_veh

def one_jump(bng, veh, qlua, *, angle, base_x, run_up, v_entry, R_flight, dart_on, args,
             ramp_idx=0, n_ramps=8, hud_on=False):
    """Single-ramp jump (ramp already built). dart_on: airborne pitch torque + roll steer correction. Returns landing data."""
    _init_actuator_shim(args)
    args.__dict__.pop("_cam_last_phase", None)
    syaw = math.radians(270.0); rot = (0.0, 0.0, math.sin(syaw / 2), math.cos(syaw / 2))
    pts, _R_lip = build_ramp_pts(args, base_x, angle)
    peak_x = pts[-1][0]
    peak_z = pts[-1][1]
    def _ramp_z_at(x):
        for (x0, z0), (x1, z1) in zip(pts[:-1], pts[1:]):
            if min(x0, x1) - 1e-6 <= x <= max(x0, x1) + 1e-6:
                t = 0.0 if abs(x1 - x0) < 1e-9 else (x - x0) / (x1 - x0)
                return z0 + t * (z1 - z0)
        return pts[0][1] if x < pts[0][0] else pts[-1][1]
    launch_mode = getattr(args, "launch_mode", "approach")
    if launch_mode in ("lip-impulse", "air-impulse"):
        sx = max(pts[0][0] + 0.5, peak_x - float(args.lip_launch_m))
        sz = (_ramp_z_at(sx) if launch_mode == "lip-impulse" else peak_z) + float(args.lip_launch_z_offset)
        _air_roll_deg = float(getattr(args, "air_impulse_roll_deg", 0.0) or 0.0)
        if launch_mode == "air-impulse" and (abs(float(args.air_impulse_pitch_deg)) > 1e-6 or abs(_air_roll_deg) > 1e-6):
            yaw = syaw; pitch = math.radians(float(args.air_impulse_pitch_deg)); roll = math.radians(_air_roll_deg)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            rot = (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )
    else:
        sx = base_x - run_up
        sz = _spawn_z_for_lane(0.0, args)
    pair_id = args.__dict__.get("_pair_id")
    _eff_baseline = args.__dict__.get("_leg_baseline_override") or args.baseline_strategy
    _leg_label = args.__dict__.get("_leg_label") or ("dart" if dart_on else _eff_baseline)
    launch_request = {
        "mode": launch_mode,
        "pair_id": pair_id,
        "sx": round(sx, 3),
        "sy": 0.0,
        "sz": round(sz, 3),
        "pitch_deg": round(float(args.air_impulse_pitch_deg), 3) if launch_mode == "air-impulse" else 0.0,
        "roll_deg": round(float(getattr(args, "air_impulse_roll_deg", 0.0) or 0.0), 3)
                   if launch_mode == "air-impulse" else 0.0,
        "v_entry": round(float(v_entry), 3),
        "takeoff_state_jitter": args.__dict__.get("_takeoff_jitter_audit"),
        "actuator_latency_ms": round(float(getattr(args, "actuator_latency_ms", 0.0) or 0.0), 3),
    }
    tp_reset = bool(getattr(args, "teleport_reset", 1))
    if not bool(getattr(args, "no_launch_teleport", 0)):
        try: veh.teleport((sx, 0.0, sz), rot, reset=tp_reset)
        except Exception: pass
    try: veh.ai.set_mode("disabled")
    except Exception: pass
    rf._step(bng, 3)
    tp.set_side_cam(bng, base_x, peak_x, R_flight, run_up); bench.force_gameplay(bng)
    _gear_dbg = bool(getattr(args, "gear_debug", 0))
    if _gear_dbg:
        print(f"[CS-geardbg] θ={angle}° dart={'ON' if dart_on else 'OFF'} after settle gear={nj._gear(veh)}", flush=True)
    if launch_mode != "air-impulse":
        for _ in range(12):
            try: veh.control(throttle=0.0, brake=1.0, steering=0.0, parkingbrake=0.0); rf._step(bng, 1)
            except Exception: pass
    else:
        for _ in range(2):
            try: veh.control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=0.0); rf._step(bng, 1)
            except Exception: pass
    if _gear_dbg:
        print(f"[CS-geardbg] θ={angle}° after settle(brake12) gear={nj._gear(veh)}", flush=True)
    def set_fac(): nj._vlua(veh, "electrics.values.throttleFactorFL=1 electrics.values.throttleFactorFR=1 "
                                 "electrics.values.throttleFactorRL=1 electrics.values.throttleFactorRR=1")
    set_fac()
    # forward-creep: release brake + full-throttle forward pulse from rest so auto-box self-selects D (never shiftToGear),
    def _vfwd():
        s = nj._poll(veh); vv = s.get("vel") or (0, 0, 0); dd = s.get("dir") or (1, 0, 0)
        return float(vv[0]) * float(dd[0]) + float(vv[1]) * float(dd[1])
    def _px():
        s = nj._poll(veh); pp = s.get("pos") or (0, 0, 0); return float(pp[0])
    _nudge_v = float(getattr(args, "launch_nudge_v", 0.0))
    if _nudge_v > 0.0:
        for _kn in range(max(1, int(getattr(args, "launch_nudge_steps", 12)))):
            try: veh.set_velocity(_nudge_v, 1.0)
            except Exception: pass
            rf._step(bng, 2)
            if _vfwd() >= 0.5 * _nudge_v:
                break
        if _gear_dbg:
            print(f"[CS-geardbg] θ={angle}° after nudge(v={_nudge_v} k={_kn+1}) gear={nj._gear(veh)} vfwd={_vfwd():+.2f}", flush=True)
    creep_ok = False; n_creep = 0; _x_creep0 = _px()
    _min_disp = float(getattr(args, "launch_creep_min_disp", 0.0))
    _creep_thr = float(getattr(args, "launch_creep_throttle", 1.0))
    for _ in range(max(0, args.launch_creep_steps)):
        n_creep += 1
        if n_creep % 5 == 1: set_fac()             # Keep throttleFactor=1 (diag: front motors need it for traction)
        try: veh.control(throttle=_creep_thr, brake=0.0, steering=0.0, parkingbrake=0.0)
        except Exception: pass
        rf._step(bng, 1)
        _disp = _px() - _x_creep0
        if _gear_dbg and (n_creep % 25 == 1):
            print(f"[CS-geardbg] θ={angle}° creep n={n_creep} gear={nj._gear(veh)} "
                  f"disp={_disp:+.2f} vfwd={_vfwd():+.2f}", flush=True)
        if _min_disp > 0.0:
            if _disp >= _min_disp:
                creep_ok = True; break
        elif _vfwd() >= args.launch_creep_vmin:
            creep_ok = True; break
    impulse_launch = launch_mode in ("lip-impulse", "air-impulse")
    v_launch = float(v_entry if impulse_launch else min(v_entry, args.spawn_v))
    _inj_dt = float(args.lip_impulse_dt) if impulse_launch else 0.3
    _inj_settle = max(3, int(float(args.lip_impulse_hold_steps) if impulse_launch else 3))
    launch_ok = False; n_launch_try = 0
    for _try in range(max(1, args.launch_retries)):
        n_launch_try = _try + 1
        try: veh.set_velocity(v_launch, _inj_dt)
        except Exception: pass
        rf._step(bng, _inj_settle)
        st0 = nj._poll(veh); v0p = st0.get("vel") or (0, 0, 0)
        gsp0 = math.hypot(float(v0p[0]), float(v0p[1]))
        if gsp0 >= args.launch_min_frac * v_launch:
            launch_ok = True; break
    if _gear_dbg:
        print(f"[CS-geardbg] θ={angle}° after inject(v_launch={v_launch:.0f} dt={_inj_dt:.2f} k={n_launch_try}) "
              f"gear={nj._gear(veh)} gsp={gsp0:.1f}", flush=True)
    if not launch_ok:
        print(f"[CS] WARN θ={angle}° dart={'ON' if dart_on else 'OFF'} launch not ready"
              f"(creep_ok={creep_ok} n_creep={n_creep} inject {n_launch_try} tries failed to hold target {v_launch:.0f}m/s)", flush=True)
    took_off = False; t_takeoff = t_land = None; land = None; air_streak = 0
    max_z = -9; max_roll_air = 0.0; max_yaw_delta_air = 0.0; max_pitchrate = 0.0; prev_pitch = None
    prev_x = sx; tumbled = False; post_land_flip = False; past_apex = False; apex_z = -9.0; apex_v_kmh = None
    lane_i_err = 0.0; land_safety_done = False
    phase_cams = None; cam_phase = "C"
    gear_logged = False
    rwpd_prev = None; wheel_w_land = None; omega_tgt_land = None
    t_wall_prev = None; dt_step = DT; wall_takeoff = None; wall_land = None; dt_samples = []
    takeoff_state = None
    front_clear_land = None; damage_land = None
    land_x_world = None; yaw0_air = None; land_yaw_delta = None
    max_yaw_delta_air_dv = 0.0; land_yaw_delta_dv = None
    land_ground_z = None; land_on_mesh = None; apex_clearance_land = None
    land_vz_mps = None; land_impact_speed_mps = None; land_height_above_ground = None
    dart_pulse_until = None; dart_pulse_u = 0.0; dart_pulse_meta = None
    dart_pulse_phase = None; dart_pulse_events = []; dart_phase_fired = set()
    cmp_off = args.__dict__.get("_cmp_off")
    trace_path = args.__dict__.get("_control_trace_path")
    trace_rows = [] if trace_path else None; _roll_t0 = None
    _gate_brk_v0 = None; _gate_brk_x0 = None; _gate_vmax_lip = None; _gate_spawn_v = None
    _a_brake_est = None; _gate_prev_px = None; _gate_prev_v = None; _gate_brake_prev = False
    fail_t0 = None
    target_pitch_deg = (
        float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg))
        + float(args.landing_flare_deg)
    )
    for i in range(args.max_steps):
        _t_now = time.time()
        dt_step = DT if t_wall_prev is None else min(0.5, max(0.005, _t_now - t_wall_prev))
        t_wall_prev = _t_now
        if took_off and t_land is None:
            dt_samples.append(dt_step)
        st = nj._poll(veh)
        pos = st.get("pos") or (0, 0, 0); vel = st.get("vel") or (0, 0, 0); d = st.get("dir") or (1, 0, 0)
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        gspd = math.hypot(float(vel[0]), float(vel[1]))
        vx_fwd = float(vel[0]) * float(d[0]) + float(vel[1]) * float(d[1])
        roll, pitch, yaw = nj._rpy(veh, st); nc = nj._contact(veh) or 0
        roll_d, pitch_d = math.degrees(roll), math.degrees(pitch)
        pdot = (pitch_d - rwpd_prev) / dt_step if rwpd_prev is not None else 0.0
        rwpd_prev = pitch_d
        air_streak = air_streak + 1 if (nc == 0) else 0
        if os.environ.get("ANYRALLY_DT_DEBUG") == "1":
            print(f"[DTDBG] i={i} pz={pz:.4f} vz={float(vel[2]):.4f} gspd={gspd:.3f} nc={nc} took_off={int(took_off)}", flush=True)
        if took_off and (abs(roll_d) > 80 or abs(pitch_d) > 85):
            tumbled = True
            if t_land is not None:
                post_land_flip = True
        if not took_off and air_streak >= 3 and px >= peak_x - 3.0 and pz >= (args.rise - args.sink - 0.7):
            took_off = True; t_takeoff = i; apex_z = pz; prev_pitch = pitch_d; apex_v_kmh = round(gspd * 3.6)
            wall_takeoff = _t_now
            yaw0_deg = math.degrees(math.atan2(float(d[1]), float(d[0])))
            yaw0_air = yaw0_deg
            takeoff_state = {"theta0_deg": round(pitch_d, 1), "omega_y0_dps": round(pdot, 0),
                             "v0_mps": round(gspd, 2), "roll0_deg": round(roll_d, 1),
                             "yaw0_deg": round(yaw0_deg, 1), "y0_m": round(py, 2),
                             "vz0_mps": round(float(vel[2]), 2)}
            _dist = _build_approach_disturbance_audit(args)
            if _dist:
                takeoff_state["approach_disturbance"] = dict(_dist)
        if took_off and t_land is None:
            max_z = max(max_z, pz); max_roll_air = max(max_roll_air, abs(roll_d))
            yaw_d = math.degrees(yaw)
            if yaw0_air is not None:
                max_yaw_delta_air = max(max_yaw_delta_air, abs(_angle_diff_deg(yaw_d, yaw0_air)))
                _yaw_dv = math.degrees(math.atan2(float(d[1]), float(d[0])))
                max_yaw_delta_air_dv = max(max_yaw_delta_air_dv, abs(_angle_diff_deg(_yaw_dv, yaw0_air)))
            if prev_pitch is not None:
                pr = abs((pitch_d - prev_pitch) / dt_step)
                if pr < 800: max_pitchrate = max(max_pitchrate, pr)
            prev_pitch = pitch_d
            if pz > apex_z: apex_z = pz
            if not past_apex and pz < apex_z - 0.5: past_apex = True
            if past_apex and (pz < 0.9 or nc >= 3):
                t_land = i; land = (round(pitch_d, 1), round(roll_d, 1)); wheel_w_land = wheel_w(veh)
                wall_land = _t_now
                land_x_world = round(px, 1)
                profile = args.__dict__.get("_current_landing_profile")
                gz = _interp_profile_z(profile, px)
                land_ground_z = round(gz, 2) if gz is not None else None
                land_on_mesh = bool(gz is not None and abs(py) <= float(args.width) * 0.5 + 1.0)
                land_height_above_ground = round(pz - gz, 2) if gz is not None else None
                apex_clearance_land = round(max_z - gz, 2) if gz is not None else None
                land_vz_mps = round(float(vel[2]), 2)
                land_impact_speed_mps = round(math.sqrt(
                    float(vel[0]) * float(vel[0]) + float(vel[1]) * float(vel[1]) + float(vel[2]) * float(vel[2])), 2)
                if yaw0_air is not None:
                    land_yaw_delta = round(_angle_diff_deg(math.degrees(yaw), yaw0_air), 1)
                    _land_yaw_dv = math.degrees(math.atan2(float(d[1]), float(d[0])))
                    land_yaw_delta_dv = round(_angle_diff_deg(_land_yaw_dv, yaw0_air), 1)
                omega_tgt_land = round(gspd / args.wheel_r, 0)
                front_clear_land = front_clearance_proxy(
                    pos, d, pitch, peak_x=peak_x, peak_z=args.rise - args.sink,
                    beta_deg=float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg)),
                    front_x=args.front_probe_x, front_z_offset=args.front_probe_z_offset)
                damage_land = damage_readback(veh)
        steer = 0.0
        if not took_off:
            brk = 0.0; pb = 0.0
            if impulse_launch and px <= peak_x + 1.0:
                try: veh.set_velocity(v_entry, float(args.lip_impulse_dt))
                except Exception: pass
            if not impulse_launch and abs(_runup_camber_deg(args)) > 1e-6:
                _camber_approach_apron_snap_if_needed(
                    veh, 0.0, px, gspd, i, args, _spawn_rot_from_args(args), bng)
            yaw_err = math.atan2(float(d[1]), float(d[0]))
            d_to_lip = peak_x - px
            lane_i_err = max(-1.5, min(1.5, lane_i_err + (float(py) - 0.0) * dt_step))
            steer = _approach_lane_keep_steer(
                py, yaw_err, 0.0, args, gspd_mps=gspd, lane_i_err=lane_i_err, d_to_lip_m=d_to_lip)
            if _prelip_yaw_lock_active(d_to_lip, yaw_err, args):
                thr = 0.0
            elif abs(float(args.prelip_steer_amp)) > 1e-6:
                if float(args.prelip_steer_end_m) <= d_to_lip <= float(args.prelip_steer_start_m):
                    steer = max(-_scl, min(_scl, float(args.prelip_steer_amp)))
            _gate_on_approach = bool(int(getattr(args, "reachability_gate", 0))) and not impulse_launch
            _gate_brake = False; _vmax_d = None; _gate_in_coast = False
            if _gate_on_approach:
                _coast_m = float(getattr(args, "gate_coast_m", 0.0))
                _d_brake = max(0.0, d_to_lip - _coast_m)
                if bool(int(getattr(args, "gate_adaptive_abrake", 0))) and _gate_brake_prev \
                        and _gate_prev_px is not None and _gate_prev_v is not None:
                    _dx = px - _gate_prev_px
                    if _dx > 0.05:
                        _a_inst = (_gate_prev_v ** 2 - gspd ** 2) / (2.0 * _dx)
                        if 0.5 < _a_inst < 25.0:
                            _a_brake_est = _a_inst if _a_brake_est is None else 0.7 * _a_brake_est + 0.3 * _a_inst
                _a_for_vmax = (_a_brake_est if (bool(int(getattr(args, "gate_adaptive_abrake", 0)))
                                               and _a_brake_est is not None) else float(args.gate_a_brake))
                _vmax_d = math.sqrt(max(0.0, float(args.gate_v_crit) ** 2 + 2.0 * _a_for_vmax * _d_brake))
                if _gate_spawn_v is None:
                    _gate_spawn_v = round(gspd, 2)
                _gate_in_coast = d_to_lip <= _coast_m
                _gate_brake = (not _gate_in_coast) and (gspd > _vmax_d + 0.2)
                if _gate_brake and _gate_brk_v0 is None:
                    _gate_brk_v0 = round(gspd, 2); _gate_brk_x0 = round(px, 2)
                _gate_prev_px = px; _gate_prev_v = gspd; _gate_brake_prev = _gate_brake
                _gate_vmax_lip = round(float(args.gate_v_crit), 2)
            if _gate_brake:
                thr = 0.0; brk = 1.0
            elif _gate_on_approach and _gate_in_coast:
                if bool(int(getattr(args, "gate_lip_power_recover", 0))):
                    _lt = float(getattr(args, "gate_lip_launch_target", 0.0) or 0.0)
                    thr = 1.0 if (_lt <= 0.0 or gspd < _lt) else 0.0
                else:
                    thr = 0.0
            elif _gate_on_approach and args.lip_power and px >= (peak_x - args.lip_power_m):
                thr = 0.0
            elif args.lip_power and px >= (peak_x - args.lip_power_m):
                thr = 1.0
            elif args.lip_throttle_cut_m > 0.0 and px >= (peak_x - args.lip_throttle_cut_m):
                thr = 0.0
            else:
                _vt = min(float(v_entry), _vmax_d) if _gate_on_approach else float(v_entry)
                thr = 1.0 if math.hypot(float(vel[0]), float(vel[1])) < _vt else 0.0
            thr, brk = _apply_approach_lip_stability(
                thr, brk, gspd=gspd, d_to_lip_m=d_to_lip, args=args, v_entry=v_entry)
            if float(args.prelip_traction_throttle) >= 0.0:
                if 0.0 <= d_to_lip <= float(args.prelip_traction_m):
                    thr = max(0.0, min(1.0, float(args.prelip_traction_throttle)))
        elif t_land is None:
            if dart_on:
                omega_w = wheel_w(veh) or 0.0
                omega_tgt = max(gspd / args.wheel_r, 0.0)
                if pz > args.land_match_z:
                    err = target_pitch_deg - pitch_d
                    if args.dart_pitch_control == "steer-probe":
                        thr, brk = 0.0, 0.0
                        t_air = (i - t_takeoff) * DT if t_takeoff is not None else 0.0
                        start = float(args.dart_steer_probe_start_sec)
                        pulse_sec = max(1e-6, float(args.dart_steer_probe_pulse_sec))
                        gap_sec = max(0.0, float(args.dart_steer_probe_gap_sec))
                        period = pulse_sec + gap_sec
                        rel = t_air - start
                        active = False
                        pulse_idx = -1
                        if rel >= 0 and period > 0:
                            pulse_idx = int(rel / period)
                            active = (
                                pulse_idx < int(args.dart_steer_probe_cycles)
                                and (rel - pulse_idx * period) < pulse_sec
                            )
                        if active:
                            sgn = -1.0 if (args.dart_steer_probe_alternate and pulse_idx % 2) else 1.0
                            steer = max(-args.smax, min(args.smax, sgn * float(args.dart_steer_probe_amp)))
                            if len(dart_pulse_events) <= pulse_idx:
                                ev = {
                                    "mode": "steer-probe",
                                    "pulse_idx": pulse_idx,
                                    "start_i": i,
                                    "t_air_sec": round(t_air, 3),
                                    "steer": round(steer, 3),
                                    "roll_deg": round(roll_d, 2),
                                    "yaw0_deg": (takeoff_state or {}).get("yaw0_deg"),
                                    "pitch_deg": round(pitch_d, 2),
                                    "pdot_dps": round(pdot, 1),
                                }
                                dart_pulse_events.append(ev)
                                dart_pulse_meta = {"mode": "steer-probe", "events": dart_pulse_events}
                        else:
                            steer = 0.0
                    elif args.dart_pitch_control == "mech-probe":
                        t_air = (i - t_takeoff) * DT if t_takeoff is not None else 0.0
                        _ms = float(args.mech_probe_start_sec)
                        _mh = max(1e-6, float(args.mech_probe_hold_sec))
                        _amp = float(args.mech_probe_amp)
                        if _ms <= t_air < _ms + _mh:
                            if args.mech_probe_axis == "torque":
                                _act_set_wheel_factors(veh, args, _amp, -_amp, _amp, -_amp)
                                thr, brk, steer = 1.0, 0.0, 0.0
                            else:
                                _act_set_wheel_factors(veh, args, 1.0, 1.0, 1.0, 1.0)
                                thr, brk = 1.0, 0.0
                                steer = max(-1.0, min(1.0, _amp))
                            if not dart_pulse_events:
                                dart_pulse_meta = {
                                    "mode": "mech-probe", "axis": str(args.mech_probe_axis),
                                    "amp": _amp,
                                    "steer_deg_cmd": (round(_amp * STEER_MAX_DEG, 1)
                                                      if args.mech_probe_axis != "torque" else 0.0),
                                    "start_sec": _ms, "hold_sec": _mh, "roll0_deg": round(roll_d, 2),
                                }
                                dart_pulse_events.append(dart_pulse_meta)
                        else:
                            _act_set_wheel_factors(veh, args, 0.0, 0.0, 0.0, 0.0)
                            thr, brk, steer = 0.0, 0.0, 0.0
                    elif args.dart_pitch_control in ("pulse", "phased-pulse"):
                        if args.dart_pitch_control == "phased-pulse":
                            t_air = (i - t_takeoff) * DT if t_takeoff is not None else 0.0
                            if dart_pulse_until is not None and i >= dart_pulse_until:
                                dart_pulse_until = None; dart_pulse_phase = None
                            if dart_pulse_until is None:
                                phase = None
                                sec = float(args.dart_pulse_sec)
                                cap = float(args.dart_pulse_max_cmd)
                                horizon = float(args.dart_pulse_horizon_sec)
                                full = None
                                kd = float(args.dart_pulse_kd)
                                if ("takeoff" not in dart_phase_fired
                                        and t_air <= float(args.dart_phase_takeoff_window_sec)):
                                    phase = "takeoff"
                                    sec = float(args.dart_phase_takeoff_sec)
                                    cap = float(args.dart_phase_takeoff_cap)
                                    horizon = float(args.dart_phase_takeoff_horizon_sec)
                                    full = float(args.dart_phase_takeoff_full_error_deg)
                                elif ("landing" not in dart_phase_fired
                                      and pz <= float(args.dart_phase_landing_z)):
                                    phase = "landing"
                                    sec = float(args.dart_phase_landing_sec)
                                    cap = float(args.dart_phase_landing_cap)
                                    horizon = float(args.dart_phase_landing_horizon_sec)
                                    full = float(args.dart_phase_landing_full_error_deg)
                                    kd = float(args.dart_phase_landing_kd)
                                elif ("mid" not in dart_phase_fired
                                      and t_air >= float(args.dart_phase_mid_after_sec)):
                                    phase = "mid"
                                    sec = float(args.dart_phase_mid_sec)
                                    cap = float(args.dart_phase_mid_cap)
                                    horizon = float(args.dart_phase_mid_horizon_sec)
                                    full = float(args.dart_phase_mid_full_error_deg)
                                if phase is not None:
                                    pred_pitch = pitch_d + pdot * horizon
                                    pred_err = target_pitch_deg - pred_pitch
                                    cmd_err = float(args.dart_phase_pitch_sign) * pred_err
                                    dart_pulse_u = _pulse_cmd_from_pred(
                                        args, pred_err=cmd_err, pdot=pdot, cap=cap,
                                        pulse_map=str(args.dart_phase_pulse_map),
                                        full_error_deg=full, kd=kd)
                                    dart_pulse_until = i + max(1, int(sec / DT))
                                    dart_pulse_phase = phase
                                    dart_phase_fired.add(phase)
                                    ev = {
                                        "phase": phase,
                                        "start_i": i,
                                        "until_i": dart_pulse_until,
                                        "t_air_sec": round(t_air, 3),
                                        "map": str(args.dart_phase_pulse_map),
                                        "pitch_sign": float(args.dart_phase_pitch_sign),
                                        "pred_pitch_deg": round(pred_pitch, 2),
                                        "pred_err_deg": round(pred_err, 2),
                                        "cmd_err_deg": round(cmd_err, 2),
                                        "pdot_dps": round(pdot, 1),
                                        "u": round(dart_pulse_u, 3),
                                    }
                                    dart_pulse_events.append(ev)
                                    dart_pulse_meta = {
                                        "mode": "phased-pulse",
                                        "events": dart_pulse_events,
                                        "last_phase": phase,
                                        "u": round(dart_pulse_u, 3),
                                    }
                            if dart_pulse_until is not None and i < dart_pulse_until:
                                thr, brk = (max(0.0, dart_pulse_u), 0.0) if dart_pulse_u > 0 else (0.0, max(0.0, -dart_pulse_u))
                            else:
                                thr, brk = 0.0, 0.0
                        elif dart_pulse_until is None:
                            pred_pitch = pitch_d + pdot * float(args.dart_pulse_horizon_sec)
                            pred_err = target_pitch_deg - pred_pitch
                            cap = float(args.dart_pulse_max_cmd)
                            pulse_map = str(args.dart_pulse_map)
                            dart_pulse_u = _pulse_cmd_from_pred(args, pred_err=pred_err, pdot=pdot,
                                                               cap=cap, pulse_map=pulse_map)
                            dart_pulse_until = i + max(1, int(float(args.dart_pulse_sec) / DT))
                            dart_pulse_meta = {
                                "mode": "pulse",
                                "start_i": i,
                                "until_i": dart_pulse_until,
                                "map": pulse_map,
                                "pred_pitch_deg": round(pred_pitch, 2),
                                "pred_err_deg": round(pred_err, 2),
                                "u": round(dart_pulse_u, 3),
                            }
                        if args.dart_pitch_control == "pulse" and i < dart_pulse_until:
                            thr, brk = (max(0.0, dart_pulse_u), 0.0) if dart_pulse_u > 0 else (0.0, max(0.0, -dart_pulse_u))
                        elif args.dart_pitch_control == "pulse":
                            thr, brk = 0.0, 0.0
                    else:
                        in_action_window = pz <= float(args.dart_action_z_max)
                        outside_deadband = (
                            abs(err) > float(args.dart_pitch_deadband_deg)
                            or abs(pdot) > float(args.dart_rate_deadband_dps)
                        )
                        if in_action_window and outside_deadband:
                            _hz = float(getattr(args, "dart_air_pred_horizon_sec", 0.0) or 0.0)
                            err_eff = (target_pitch_deg - (pitch_d + pdot * _hz)) if _hz > 0.0 else err
                            u = args.kp_pitch * err_eff / 20.0 - args.kd_pitch * pdot / 100.0
                            if u > 0 and omega_w > args.omega_cap * max(omega_tgt, 1.0):
                                u = 0.0
                            thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
                        else:
                            thr, brk = 0.0, 0.0
                    if args.dart_pitch_control not in ("steer-probe", "mech-probe"):
                        steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0))
                    pb = 0.0
                elif int(args.dart_disable_landmatch):
                    thr, brk, pb = 0.0, 0.0, 0.0
                    steer = 0.0
                elif args.landmatch:
                    d_omega = omega_w - omega_tgt
                    if d_omega > args.omega_tol:
                        thr = 0.0; brk = min(args.land_brake_cap, args.kp_omega * d_omega / 100.0)
                    elif d_omega < -args.omega_tol and abs(pitch_d) < 8.0:
                        thr = min(args.land_brake_cap, args.kp_omega * (-d_omega) / 100.0); brk = 0.0
                    else:
                        thr, brk = 0.0, 0.0
                    if abs(pdot) > args.rate_tol and abs(pitch_d) < 8.0:
                        if pdot > 0 and brk == 0.0:
                            brk = min(args.land_brake_cap, args.kd_land * pdot / 100.0); thr = 0.0
                        elif pdot < 0 and thr == 0.0:
                            thr = min(args.land_brake_cap, args.kd_land * (-pdot) / 100.0); brk = 0.0
                    steer = 0.0; pb = 0.0
                elif args.landprep:
                    thr, brk, pb = 0.0, 0.0, 0.0
                    steer = 0.0
                else:
                    err = target_pitch_deg - pitch_d
                    u = args.kp_pitch * err / 20.0 - args.kd_pitch * pdot / 100.0
                    thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
                    steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0)); pb = 0.0
            elif _eff_baseline == "human":
                err = target_pitch_deg - pitch_d
                u = args.air_trim_kp * err / 20.0 - args.air_trim_kd * pdot / 100.0
                omega_w = wheel_w(veh) or 0.0
                omega_tgt = max(gspd / args.wheel_r, 0.0)
                if u > 0 and omega_w > args.omega_cap * max(omega_tgt, 1.0):
                    u = 0.0
                thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
                steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0))
                pb = 0.0
            else:
                thr, brk, pb = 0.0, 0.0, 0.0
        else:
            thr, brk, pb = 0.0, 1.0, 1.0
        if os.environ.get("C7_AIR_DBG") and took_off and t_land is None:
            print(f"[AIRDBG] i={i} dart={dart_on} ctrl={args.dart_pitch_control} pz={pz:.2f} "
                  f"pitch={pitch_d:.1f} pdot={pdot:.1f} air_streak={air_streak} "
                  f"thr={thr:.2f} brk={brk:.2f} omw={(wheel_w(veh) or 0.0):.0f} "
                  f"land_match_z={args.land_match_z}", flush=True)
        # **Never shiftToGear** (forcing N or D freezes EV auto-box -> approach sequence stuck).
        thr = max(0.0, min(1.0, float(thr))); brk = max(0.0, min(1.0, float(brk)))
        rev_back = bool(took_off and vx_fwd < -0.3)
        if t_land is not None:
            thr, brk, steer, pb = 0.0, 1.0, 0.0, 1.0
            args.__dict__["_land_safety_gspd"] = gspd
            if not land_safety_done:
                _landed_vehicle_safety_reset(veh, args, first_step=True)
                land_safety_done = True
            else:
                _landed_vehicle_safety_reset(veh, args, first_step=False)
        elif rev_back:
            thr, brk, pb = 0.0, 1.0, 1.0
        if i % 5 == 0: set_fac()
        if i % 25 == 0: bench.force_gameplay(bng)
        _act_vehicle_control(veh, args, thr, brk, steer, pb)
        if trace_rows is not None:
            if _roll_t0 is None:
                _roll_t0 = _t_now
            _phase = "approach" if not took_off else ("air" if t_land is None else "landed")
            trace_rows.append({
                "i": i, "t": round(_t_now - _roll_t0, 3), "dt": round(dt_step, 4), "phase": _phase,
                "thr": round(float(thr), 3), "brk": round(float(brk), 3),
                "steer": round(float(steer), 4), "steer_deg": round(float(steer) * STEER_MAX_DEG, 1),
                "pb": round(float(pb), 3),
                "pitch_deg": round(pitch_d, 2), "roll_deg": round(roll_d, 2),
                "yaw_deg": round(math.degrees(yaw), 2), "pdot_dps": round(pdot, 1),
                "gspd": round(gspd, 2), "vx_fwd": round(vx_fwd, 2), "vz": round(float(vel[2]), 2),
                "pz": round(pz, 3), "nc": int(nc), "wheels": wheel_w_all(veh),
            })
        if phase_cams is None:
            _far_x = args.__dict__.get("_current_land_far_x") or args.__dict__.get("_current_land_toe_x")
            _span_cap = float(getattr(args, "cam_c_span_m", 32.0))
            if _far_x is not None and float(_far_x) > peak_x:
                _span_cap = max(_span_cap, float(_far_x) - peak_x)
            phase_cams, cmetas, land_x_pred = build_phase_cams(
                base_x, run_up, peak_x, R_flight, args.rise,
                land_far_x=_far_x, c_span_cap=_span_cap,
                approach_post_lip_m=float(getattr(args, "cam_approach_post_lip_m", 2.0)),
                approach_pre_start_m=float(getattr(args, "cam_approach_pre_start_m", 2.0)),
                approach_cam_z_drop_m=float(getattr(args, "cam_approach_z_drop_m", 8.0)),
                cam_c_z_drop_m=_cam_c_z_drop_from_args(args))
            print(f"[CS-cam] θ={angle}° AB:z={cmetas['AB']['z']} frame=[{cmetas['AB']['framed_lo']},"
                  f"{cmetas['AB']['framed_hi']}] ang_start={cmetas['AB']['ang_start']}° "
                  f"ang_peak={cmetas['AB']['ang_peak']}° | "
                  f"C:z={cmetas['C']['z']} land_x={cmetas['C']['land_x']} "
                  f"land_far={cmetas['C'].get('land_far_x')} ang_land={cmetas['C']['ang_land']}° "
                  f"ang_peak={cmetas['C'].get('ang_peak')}° (C ang_peak<28=takeoff car in frame); "
                  f"all z>0={all(cmetas[k]['z']>0 for k in ('AB','C'))}", flush=True)
        if not took_off:
            cam_phase = "AB"
        else:
            cam_phase = "C"
        if _cam_update_due(i, args, phase_key=cam_phase):
            try: bng.camera.set_free(phase_cams[cam_phase][0], phase_cams[cam_phase][1])
            except Exception: pass
        if hud_on and i % 12 == 0:
            rollsteer_active = bool(dart_on and took_off and (t_land is None) and pz > args.desteer_z
                                    and abs(steer) > 0.01)
            d_deg = int(round(steer * STEER_MAX_DEG))
            phase = "approach" if not took_off else ("AIRBORNE" if t_land is None else "LANDED")
            ps = {"approach": "appr", "AIRBORNE": "AIR", "LANDED": "LAND"}.get(phase, phase[:4])
            lp = f"{land[0]:.0f}" if land else f"{pitch_d:.0f}(live)"
            lr = f"{land[1]:.0f}" if land else f"{roll_d:.0f}(live)"
            pe = f"{(land[0] - target_pitch_deg):.0f}" if land else f"{(pitch_d - target_pitch_deg):.0f}(live)"
            if rollsteer_active:
                rs = f"d{d_deg}°"
            elif dart_on and took_off:
                rs = "ctr"
            elif dart_on:
                rs = "ON"
            else:
                rs = "OFF"
            spd_kmh = int(round(gspd * 3.6))
            line_land = f"Land P={lp}° E={pe}° R={lr}°"
            if dart_on and cmp_off is not None:
                line_land += f" | OFF P={cmp_off[0]:.0f}° R={cmp_off[1]:.0f}°"
            lines = [
                f"R{ramp_idx+1}/{n_ramps} {angle}° {ps} {spd_kmh}km/h",
                f"DART pitch={'ON' if dart_on else 'OFF'} roll={rs}",
                line_land,
            ]
            set_hud(bng, lines)
        rf._step(bng, 1)
        if t_land is not None and not gear_logged and i > t_land + 18:
            try:
                g = nj._vlua(veh, "return tostring(electrics.values.gear)")
                print(f"[CS-gear] θ={angle}° dart={'ON' if dart_on else 'OFF'} post-landing gear={g} "
                      f"(not R means guard active)", flush=True)
            except Exception: pass
            gear_logged = True
        prev_x = px
        _hold_sec = _postland_hold_sec(args, hud_on)
        if t_land is not None and wall_land is not None and (_t_now - wall_land) >= _hold_sec:
            break
        if not took_off:
            if fail_t0 is None and (px > peak_x + 3.0 or (i > 30 and gspd < 0.5)):
                fail_t0 = _t_now
            _fail_hold = _postfail_hold_sec(args, hud_on)
            if fail_t0 is not None and (_t_now - fail_t0) >= _fail_hold:
                break
    airtime_steps = round((t_land - t_takeoff) * DT, 2) if (t_takeoff and t_land) else None
    airtime = (round(wall_land - wall_takeoff, 3) if (wall_takeoff and wall_land) else airtime_steps)
    dt_eff = round(sum(dt_samples) / len(dt_samples), 4) if dt_samples else None
    land_pitch_error = round(land[0] - target_pitch_deg, 1) if land else None
    if trace_rows is not None and trace_path:
        try:
            rec = {
                "tag": getattr(args, "tag", None), "pair_id": pair_id, "dart_on": bool(dart_on),
                "baseline_strategy": getattr(args, "baseline_strategy", None),
                "dart_pitch_control": getattr(args, "dart_pitch_control", None),
                "angle_deg": angle, "v_entry": v_entry,
                "cross_slope_deg": args.__dict__.get("_current_cross_slope_deg", 0.0),
                "air_impulse_pitch_deg": getattr(args, "air_impulse_pitch_deg", None),
                "air_impulse_roll_deg": getattr(args, "air_impulse_roll_deg", None),
                "took_off": took_off, "land_pitch": land[0] if land else None,
                "land_roll": land[1] if land else None, "target_pitch_deg": target_pitch_deg,
                "airtime": airtime, "n_steps": len(trace_rows), "trace": trace_rows,
            }
            with open(trace_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            print(f"[CS-trace] roll(dart={'ON' if dart_on else 'OFF'} pair={pair_id}) "
                  f"wrote {len(trace_rows)} control trace steps", flush=True)
        except Exception as e:
            print(f"[CS-trace] WARN failed to write control trace: {e}", flush=True)
    _gate_appr = None
    if bool(int(getattr(args, "reachability_gate", 0))) and launch_mode == "approach":
        _v0_launch = (takeoff_state or {}).get("v0_mps")
        _a_brake_meas = None
        if _gate_brk_v0 is not None and _gate_brk_x0 is not None and _v0_launch is not None:
            _brk_dist = max(0.1, float(peak_x) - float(_gate_brk_x0))
            _a_brake_meas = round((float(_gate_brk_v0) ** 2 - float(_v0_launch) ** 2) / (2.0 * _brk_dist), 2)
        _gate_appr = {
            "enabled": True, "mode": "approach_continuous_vmax_brake",
            "v_crit": round(float(args.gate_v_crit), 2), "a_brake_assumed": round(float(args.gate_a_brake), 2),
            "spawn_v": _gate_spawn_v, "brake_engage_v": _gate_brk_v0, "brake_engage_x": _gate_brk_x0,
            "v0_launch": _v0_launch, "vmax_at_lip": _gate_vmax_lip,
            "a_brake_measured": _a_brake_meas, "run_up": round(float(run_up), 2),
            "intervened": bool(_gate_brk_v0 is not None),
            "adaptive_abrake": bool(int(getattr(args, "gate_adaptive_abrake", 0))),
            "a_brake_online_est": round(_a_brake_est, 2) if _a_brake_est is not None else None,
        }
    _c7_guard = None
    if int(getattr(args, "dart_airtime_guardrail", 0)) >= 1 and airtime is not None \
            and takeoff_state and takeoff_state.get("theta0_deg") is not None:
        try:
            _c7_guard = airtime_guardrail_eval(
                float(airtime), float(takeoff_state["theta0_deg"]),
                warn_margin=float(getattr(args, "dart_airtime_guardrail_margin", 0.0)),
            )
            print(f"[DART-airtime-guard] {'ok' if _c7_guard['ok'] else 'FAIL'} {_c7_guard['verdict']} "
                  f"idx={_c7_guard['favorability_index']:+.2f} airtime={_c7_guard['airtime_s']:.2f}s "
                  f"θ0={_c7_guard['theta0_deg']:.1f}° (cap {_c7_guard['theta0_cap_deg']:.1f}°)"
                  + ("" if _c7_guard["ok"] else f" | {_c7_guard['recommendation']}"), flush=True)
        except Exception as _e:
            print(f"[DART-airtime-guard] WARN guardrail evaluation failed: {_e}", flush=True)
    return {"took_off": took_off, "land_pitch": land[0] if land else None,
            "reachability_gate": _gate_appr, "dart_airtime_guardrail": _c7_guard,
            "land_pitch_error": land_pitch_error, "target_pitch_deg": target_pitch_deg,
            "land_roll": land[1] if land else None, "airtime": airtime, "max_z": round(max_z, 2),
            "max_roll_air": round(max_roll_air, 1), "max_yaw_delta_air": round(max_yaw_delta_air, 1),
            "land_yaw_delta": land_yaw_delta, "max_yaw_delta_air_dv": round(max_yaw_delta_air_dv, 1),
            "land_yaw_delta_dv": land_yaw_delta_dv, "max_pitchrate": round(max_pitchrate, 0),
            "tumbled": tumbled, "post_land_flip": bool(post_land_flip),
            "wheel_w_land": wheel_w_land, "omega_tgt_land": omega_tgt_land,
            "front_clearance": front_clear_land, "damage_land": damage_land,
            "land_x_world": land_x_world, "land_ground_z": land_ground_z, "land_on_mesh": land_on_mesh,
            "land_height_above_ground": land_height_above_ground, "apex_clearance_land": apex_clearance_land,
            "land_vz_mps": land_vz_mps, "land_impact_speed_mps": land_impact_speed_mps,
            "launch_tries": n_launch_try, "launch_ok": launch_ok,
            "creep_ok": creep_ok, "n_creep": n_creep,
            "takeoff_state": takeoff_state, "T_flight": airtime,
            "airtime_steps": airtime_steps, "dt_eff": dt_eff,
            "cross_slope_deg": args.__dict__.get("_current_cross_slope_deg", 0.0),
            "pair_id": pair_id, "launch_request": launch_request,
            "takeoff_state_jitter": args.__dict__.get("_takeoff_jitter_audit"),
            "actuator_latency_ms": round(float(getattr(args, "actuator_latency_ms", 0.0) or 0.0), 3),
            "dart_pulse": dart_pulse_meta,
            "_provenance": make_provenance("freerun_wallclock",
                                           sps=int(getattr(args, "sim_steps_per_second", 100) or 100),
                                           dt_eff=dt_eff)}

def _run_approach_fresh_spawn(bng, veh, qlua, sc, segs, lsegs, angle, R_flight, v_entry,
                              ramp_idx, args, data, *, wait_scenario_ready, ensure_freerun):
    """Real approach run-up: each jump scenario.load+start reload + respawn (EV powertrain reset to D,
    avoid teleport gear-lock false positive) + place_ramp rebuild + one_jump no teleport (keep spawn state).
    Gate OFF/ON from args.reachability_gate; dart leg from args.fresh_spawn_c7; N from args.rolls.
    Evidence: reload spawn equivalent to first fresh spawn; throttle 0->19 m/s all gear=D."""
    a = angle
    mode = "on" if bool(int(getattr(args, "fresh_spawn_c7", 1))) else "off"
    dart_on = (mode == "on")
    if a not in data:
        data[a] = {"off": [], "on": [], "invalid_pairs": []}
    n = int(args.rolls)
    gate_on = bool(int(getattr(args, "reachability_gate", 0)))
    max_attempts = max(n, int(getattr(args, "paired_max_attempts", 0)) or (n * 3))
    print(f"[CS-fresh] θ={a}° real approach run-up: N={n}(max_attempts={max_attempts}) dart={mode.upper()} "
          f"gate={'ON' if gate_on else 'OFF'} v_entry={v_entry} run_up={args.run_up} "
          f"(reload scenario+ramp each jump, no teleport, EV box reset to D)", flush=True)
    n_valid = 0
    for k in range(max_attempts):
        if n_valid >= n:
            break
        ok = False
        for _retry in range(3):
            try:
                bng.scenario.load(sc); bng.scenario.start()
                wait_scenario_ready(bng, expected_vid="ego", timeout_sec=20.0)
                ensure_freerun(bng)
                for _ in range(4):
                    bench.force_gameplay(bng); rf._step(bng, 4)
                ok = bench.ensure_gameplay_live(bng, veh)
                if ok:
                    break
            except Exception as e:
                print(f"[CS-fresh] θ={a}° attempt{k} reload retry{_retry + 1} err={e}", flush=True)
            rf._step(bng, 10)
        if not ok:
            print(f"[CS-fresh] θ={a}° attempt{k} reload failed, skip", flush=True)
            continue
        place_ramp_with_ground(bng, segs + lsegs, qlua, getattr(args, "runup_ground_type", "ASPHALT"))
        rf._step(bng, 3); bench.force_gameplay(bng)
        args.__dict__["_pair_id"] = n_valid
        _v_jump, _ = prep_jump_e6_injection(args, v_entry, jump_seed=ramp_idx * 1000 + k)
        r = one_jump(bng, veh, qlua, angle=a, base_x=args.base_x, run_up=args.run_up,
                     v_entry=_v_jump, R_flight=R_flight, dart_on=dart_on, args=args,
                     ramp_idx=ramp_idx, n_ramps=len(args.angles), hud_on=bool(args.hud))
        _ts = r.get("takeoff_state") or {}
        _g = r.get("reachability_gate") or {}
        if not r.get("took_off"):
            print(f"[CS-fresh] θ={a}° attempt{k} took_off=False (approach flake), retry "
                  f"(valid={n_valid}/{n})", flush=True)
            continue
        data[a][mode].append(r)
        n_valid += 1
        print(f"[CS-fresh] θ={a}° valid{n_valid}/{n}(attempt{k}) took_off=True v0_launch={_ts.get('v0_mps')} "
              f"land_pitch={r['land_pitch']} land_err={r.get('land_pitch_error')} tumbled={r['tumbled']} "
              f"impact={r.get('land_impact_speed_mps')} | gate brake_v0={_g.get('brake_engage_v')} "
              f"a_brake_meas={_g.get('a_brake_measured')}", flush=True)
    if n_valid < n:
        print(f"[CS-fresh] WARN θ={a}° only collected valid={n_valid}/{n} (attempts exhausted {max_attempts})", flush=True)
    args.__dict__["_pair_id"] = None
    recs = data[a][mode]
    took = sum(1 for x in recs if x.get("took_off"))
    v0s = sorted(round(float((x.get("takeoff_state") or {}).get("v0_mps")), 2)
                 for x in recs if (x.get("takeoff_state") or {}).get("v0_mps") is not None)
    errs = [x.get("land_pitch_error") for x in recs if x.get("land_pitch_error") is not None]
    print(f"[CS-fresh] === θ={a}° summary: took_off={took}/{len(recs)} "
          f"v0_launch={('['+str(v0s[0])+'~'+str(v0s[-1])+']') if v0s else 'NA'} "
          f"land_err median={med(errs) if errs else 'NA'}° ===", flush=True)

def _simul3_appr_reapply_sim_settings(bng, args) -> None:
    """Reapply deterministic stepping after hard_refresh/reload (same as session cold start)."""
    _sps = int(getattr(args, "sim_steps_per_second", 100))
    for _attempt in (
        lambda: bng.settings.set_deterministic(_sps),
        lambda: (bng.settings.set_steps_per_second(_sps), bng.settings.set_deterministic()),
        lambda: (bng.set_deterministic(), bng.set_steps_per_second(_sps)),
    ):
        try:
            _attempt()
            return
        except Exception:
            continue

def _simul3_appr_reload_scenario(bng, sc, expected_vid, *, wait_scenario_ready, ensure_freerun,
                                 simul_legs=None, rebind=False, args=None):
    """approach simul3: scenario.load+start + freerun warmup (EV spawn in D). Returns (ok, bng)."""
    session = getattr(bng, "_anyrally_session", None)
    _reload_timeout = float(getattr(args, "approach_simul3_reload_timeout_sec", 90.0) or 90.0)
    _ready_timeout = min(30.0, max(15.0, _reload_timeout * 0.35))
    for _retry in range(3):
        try:
            print(f"[CS-simul3-appr] reload try={_retry + 1} load+start vid={expected_vid} "
                  f"(reload_timeout={_reload_timeout:.0f}s ready_timeout={_ready_timeout:.0f}s)",
                  flush=True)
            bng.scenario.load(sc)
            bng.scenario.start()
            ready = wait_scenario_ready(
                bng, expected_vid=expected_vid, timeout_sec=_ready_timeout,
                log_prefix="[CS-simul3-appr]",
            )
            if not ready:
                raise TimeoutError(
                    f"wait_scenario_ready timeout after {_ready_timeout:.0f}s")
            ensure_freerun(bng)
            if rebind and simul_legs is not None:
                _simul3_appr_rebind_vehicles(bng, sc, simul_legs)
                if args is not None:
                    _simul3_appr_reapply_sim_settings(bng, args)
            for _ in range(6):
                bench.force_gameplay(bng)
                rf._step(bng, 4)
            if rebind and simul_legs is not None and args is not None:
                if not _simul3_appr_wait_spawn_live(bng, simul_legs, args, tag=f"reload{_retry + 1}"):
                    raise RuntimeError("spawn poll dead after reload+rebind")
            print(f"[CS-simul3-appr] reload try={_retry + 1} OK", flush=True)
            return True, bng
        except Exception as e:
            err_s = f"{type(e).__name__}: {e}"
            print(f"[CS-simul3-appr] reload retry{_retry + 1} err={err_s}", flush=True)
            if session is not None and (
                "not running" in err_s or "Disconnected" in err_s or "timeout" in err_s.lower()
            ):
                print("[CS-simul3-appr] BNG disconnected/hung → hard_refresh reconnect", flush=True)
                try:
                    bng = session.hard_refresh(sleep_sec=5.0, kill=True)
                    rebind = True
                except Exception as e2:
                    print(f"[CS-simul3-appr] hard_refresh reconnect FAILED: {e2}", flush=True)
        with contextlib.suppress(Exception):
            rf._step(bng, 10)
    print("[CS-simul3-appr] reload FAILED after 3 tries", flush=True)
    return False, bng

def _simul3_appr_rebind_vehicles(bng, sc, simul_legs):
    """After hard_refresh reconnect BeamNGpy; detach old sensors then attach new State/Electrics."""
    from beamngpy.sensors import Electrics, State  # type: ignore
    sc.connect(bng, connect_player=True, connect_existing=True)
    for leg in simul_legs:
        v = leg["veh"]
        try:
            v.disconnect()
        except Exception:
            pass
        for _sname in ("state", "electrics"):
            try:
                if _sname in getattr(v, "sensors", {}):
                    v.detach_sensor(_sname)
            except Exception:
                with contextlib.suppress(Exception):
                    del v.sensors[_sname]
        for _sname, _cls in (("state", State), ("electrics", Electrics)):
            try:
                v.attach_sensor(_sname, _cls())
            except Exception:
                pass
        try:
            v.connect(bng)
        except Exception:
            pass

def _simul3_appr_wait_spawn_live(bng, simul_legs, args, *, max_steps=120, tag="") -> bool:
    """Step after reload/teleport until poll non-zero and pz reasonable."""
    for _i in range(max_steps):
        bench.force_gameplay(bng)
        rf._step(bng, 4)
        if not _simul3_appr_spawn_health_ok(simul_legs, args):
            continue
        st = nj._poll(simul_legs[0]["veh"])
        pos = st.get("pos") or (0, 0, 0)
        gspd = float(st.get("gspd") or 0.0)
        print(f"[CS-simul3-appr] spawn live @step={(_i + 1) * 4} ({tag}) "
              f"px={float(pos[0]):.1f} py={float(pos[1]):.3f} pz={float(pos[2]):.2f} gspd={gspd:.2f}",
              flush=True)
        return True
    return False

def _simul3_appr_spawn_health_ok(simul_legs, args) -> bool:
    """After reload/refresh poll non-zero = vehicle handle still valid."""
    if not simul_legs:
        return False
    try:
        st = nj._poll(simul_legs[0]["veh"])
        pos = st.get("pos") or (0, 0, 0)
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        gspd = float(st.get("gspd") or 0.0)
        if abs(px) < 0.01 and abs(py) < 0.01 and abs(pz) < 0.01 and gspd < 0.01:
            return False
        exp_z = _leg_spawn_xyz(simul_legs[0], args)[2]
        if abs(pz) < 0.05 and abs(exp_z) > 0.5:
            return False
        return True
    except Exception:
        return False

def _simul3_appr_maybe_hard_refresh(bng, n_valid, refresh_every, last_refresh_at_valid):
    """Soft refresh every N valid ACCEPTs (full reload+rebind, do not kill BeamNG).
    hard_refresh(kill) breaks the Vehicle handle -> poll zeros -> no_takeoff."""
    if refresh_every <= 0 or n_valid <= 0:
        return bng, False, last_refresh_at_valid
    if (n_valid % refresh_every) != 0 or n_valid == last_refresh_at_valid:
        return bng, False, last_refresh_at_valid
    print(f"[CS-simul3-appr] valid={n_valid}/{refresh_every} soft_refresh "
          f"(reload+rebind, no kill; every {refresh_every} ACCEPTs)", flush=True)
    return bng, True, n_valid

def _quat_from_yaw_pitch_roll(yaw_rad, pitch_deg=0.0, roll_deg=0.0):
    """World yaw + optional pitch/roll (deg) -> BeamNG rot_quat (x,y,z,w)."""
    pitch = math.radians(float(pitch_deg))
    roll = math.radians(float(roll_deg))
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)

def _spawn_rot_from_args(args):
    """approach spawn quat: yaw=270 deg + --approach-spawn-roll-deg + camber gamma body roll.
    On camber, body must pre-roll gamma to match banked apron; else flat body on 8 deg slope = 2-wheel contact -> slip."""
    syaw = math.radians(270.0)
    roll_deg = (float(getattr(args, "approach_spawn_roll_deg", 0.0) or 0.0)
                + _runup_camber_deg(args))
    return _quat_from_yaw_pitch_roll(syaw, 0.0, roll_deg)

RATE_KICK_DT_S = 0.05

def _apply_lip_roll_rate_kick(vehicles, kick_dps, dt=RATE_KICK_DT_S):
    """At liftoff add body-roll angular velocity to all vehicles on the same tick (deg/s, world x axis).

    Implemented through vehicle Lua `thrusters.applyAccel(accel, dt, nodeId, angularAccel)`.
    Note: `obj:setAngularVelocity` does not exist in this BeamNG build (pcall swallows the
    error silently, so the kick becomes a no-op); applyAccel is the working path.
    The caller must step the simulation >= dt for the kick to inject fully."""
    wx = math.radians(float(kick_dps))
    lua = (
        f"local dt={float(dt):.4f} "
        f"thrusters.applyAccel(vec3(0,0,0), dt, nil, vec3({wx:.6f}/dt, 0, 0)) "
        "return 'ok'"
    )
    for veh in vehicles:
        fn = getattr(veh, "queue_lua_command", None)
        if not callable(fn):
            continue
        try:
            fn(lua, response=True)
        except Exception:
            pass

def _apply_pitch_rate_kick(vehicles, kick_dps, dt=RATE_KICK_DT_S):
    """After air-impulse launch settle, add body-pitch angular velocity (deg/s).

    Same `thrusters.applyAccel` path as the roll kick (see note there about
    `obj:setAngularVelocity` being absent from this build). At spawn yaw=270 deg the
    vehicle travels along world +x, so the pitch axis is world y; calibrate the
    sign-to-nose-up mapping from the smoke readback (takeoff_state.omega_y0_dps).
    Used by the budget-interior entry-rate calibration cell."""
    wy = math.radians(float(kick_dps))
    lua = (
        f"local dt={float(dt):.4f} "
        f"thrusters.applyAccel(vec3(0,0,0), dt, nil, vec3(0, {wy:.6f}/dt, 0)) "
        "return 'ok'"
    )
    for veh in vehicles:
        fn = getattr(veh, "queue_lua_command", None)
        if not callable(fn):
            continue
        try:
            fn(lua, response=True)
        except Exception:
            pass

def _read_pitch_rate_dps(veh):
    """Read back body pitch angular velocity (deg/s); None if unreadable."""
    try:
        r = veh.queue_lua_command("return tostring(obj:getPitchAngularVelocity())", response=True)
        val = r.get("result") if isinstance(r, dict) else r
        return round(math.degrees(float(str(val).strip())), 1)
    except Exception:
        return None

def _simul3_appr_teleport_spawn(simul_legs, args, spawn_rot, bng):
    """Session reuse batch: teleport three vehicles to run-up spawn (no scenario.load)."""
    tp_reset = bool(int(getattr(args, "teleport_reset", 1)))
    for leg in simul_legs:
        try:
            leg["veh"].teleport(_leg_spawn_xyz(leg, args), spawn_rot, reset=tp_reset)
        except Exception:
            pass
        try:
            leg["veh"].ai.set_mode("disabled")
        except Exception:
            pass
    for _ in range(4):
        bench.force_gameplay(bng)
        rf._step(bng, 4)
    if abs(_runup_camber_deg(args)) < 1e-6:
        for leg in simul_legs:
            try:
                leg["veh"].control(throttle=0.0, brake=1.0, steering=0.0, parkingbrake=1.0)
            except Exception:
                pass
        rf._step(bng, int(getattr(args, "flat_runup_settle_steps", 15) or 15))
        for leg in simul_legs:
            try:
                leg["veh"].control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=0.0)
            except Exception:
                pass
        rf._step(bng, 2)
    elif abs(_runup_camber_deg(args)) > 1e-6:
        for leg in simul_legs:
            try: leg["veh"].control(throttle=0.0, brake=1.0, steering=0.0, parkingbrake=1.0)
            except Exception: pass
        rf._step(bng, int(getattr(args, "camber_settle_steps", 40) or 40))
        try:
            _st0 = nj._poll(simul_legs[0]["veh"])
            _py0 = float((_st0.get("pos") or (0, 0, 0))[1])
            print(f"[CS-runup-camber] spawn readback py={_py0:.3f}m (target lane_y={float(simul_legs[0]['y']):.1f}m, "
                  f"~0=centerline spawn aligned)", flush=True)
        except Exception:
            pass
        _camber_snap_lane_center(simul_legs, args, spawn_rot, bng, tag="post-settle")
        _camber_appr_warmup(bng, simul_legs, args, spawn_rot)
        _camber_snap_lane_center(simul_legs, args, spawn_rot, bng, tag="post-warmup")

def _camber_appr_warmup(bng, simul_legs, args, spawn_rot):
    """Camber run-up: release settle brake + nudge/creep (lane-keep), real displacement on banked apron before approach."""
    if not simul_legs or abs(_runup_camber_deg(args)) < 1e-6:
        return
    if not _simul3_appr_spawn_health_ok(simul_legs, args):
        print("[CS-runup-camber] WARN warmup skip: spawn poll dead", flush=True)
        return
    for leg in simul_legs:
        try:
            nj._vlua(leg["veh"],
                     "electrics.values.throttleFactorFL=1 electrics.values.throttleFactorFR=1 "
                     "electrics.values.throttleFactorRL=1 electrics.values.throttleFactorRR=1")
        except Exception:
            pass
    for _ in range(4):
        for leg in simul_legs:
            try: leg["veh"].control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=0.0)
            except Exception: pass
        rf._step(bng, 1)
    nudge = float(getattr(args, "launch_nudge_v", 0.0) or 0.0)
    if nudge <= 0:
        nudge = float(getattr(args, "camber_nudge_v", 2.5) or 2.5)
    _use_nudge = bool(int(getattr(args, "camber_nudge_enable", 0) or 0))
    if _use_nudge:
        for leg in simul_legs:
            try: leg["veh"].set_velocity(nudge, 0.5)
            except Exception: pass
        for _ in range(max(6, int(getattr(args, "camber_nudge_steps", 10) or 10))):
            for leg in simul_legs:
                st = nj._poll(leg["veh"])
                pos = st.get("pos") or (0, 0, 0)
                d = st.get("dir") or (1, 0, 0)
                steer = _approach_lane_keep_steer(float(pos[1]), math.atan2(float(d[1]), float(d[0])),
                                                  float(leg["y"]), args, gspd_mps=nudge)
                try: leg["veh"].control(throttle=0.0, brake=0.0, steering=steer, parkingbrake=0.0)
                except Exception: pass
            rf._step(bng, 1)
    creep_steps = int(getattr(args, "camber_creep_steps", 0) or 0)
    if creep_steps <= 0:
        creep_steps = max(int(getattr(args, "launch_creep_steps", 0) or 0), 60)
    creep_thr = float(getattr(args, "camber_creep_throttle", 0.65) or 0.65)
    min_disp = float(getattr(args, "camber_creep_min_disp", 1.5) or 1.5)
    veh0 = simul_legs[0]["veh"]
    x0 = float((nj._poll(veh0).get("pos") or (0, 0, 0))[0])
    disp = 0.0
    for _ in range(creep_steps):
        for leg in simul_legs:
            st = nj._poll(leg["veh"])
            pos = st.get("pos") or (0, 0, 0)
            d = st.get("dir") or (1, 0, 0)
            py = float(pos[1])
            yaw_err = math.atan2(float(d[1]), float(d[0]))
            vv = st.get("vel") or (0, 0, 0)
            steer = _approach_lane_keep_steer(
                py, yaw_err, float(leg["y"]), args,
                gspd_mps=math.hypot(float(vv[0]), float(vv[1])))
            try: leg["veh"].control(throttle=creep_thr, brake=0.0, steering=steer, parkingbrake=0.0)
            except Exception: pass
        rf._step(bng, 1)
        px = float((nj._poll(veh0).get("pos") or (0, 0, 0))[0])
        disp = px - x0
        if disp >= min_disp:
            break
    st = nj._poll(veh0)
    pos = st.get("pos") or (0, 0, 0)
    py_end = float(pos[1])
    vv = st.get("vel") or (0, 0, 0)
    gspd = math.hypot(float(vv[0]), float(vv[1]))
    _half_w = _camber_mesh_width(args) / 2.0
    print(f"[CS-runup-camber] warmup creep-only(nudge=OFF) disp={disp:.2f}m gspd={gspd:.2f} "
          f"py={py_end:.2f}m (mesh half-width={_half_w:.1f}m, |py|>{0.35*_half_w:.1f}-> off ramp surface)", flush=True)
    if abs(py_end) > 0.35 * _half_w:
        print(f"[CS-runup-camber] WARN after warmup |py|={abs(py_end):.2f}m exceeds 60% ramp half-width -> likely run off ramp",
              flush=True)

def _simul3_appr_prep_jump(bng, sc, qlua, simul_legs, segs, lsegs, lpts, peak_x, args, *,
                            k, n_valid, refresh_every, last_refresh_at_valid,
                            _pvid, _session_reuse, ramp_i, a, vi, _cj, _gate_v_eff, _gate_audit,
                            wait_scenario_ready, ensure_freerun):
    """Single-jump prep: optional refresh -> reload -> mesh -> spawn health gate (retry prep on fail)."""
    args.__dict__["_adp_hyst"] = {}
    if _cj > 0:
        _rng = random.Random(int(args.cond_jitter_seed) + ramp_i * 1000 + k)
        _v_req = round(_rng.uniform(float(args.cond_jitter_v_lo), float(args.cond_jitter_v_hi)), 2)
    else:
        _v_req = float(vi)
    _spawn_rot = _spawn_rot_from_args(args)
    _did_refresh = False
    for _prep_try in range(3):
        bng, _did_refresh, last_refresh_at_valid = _simul3_appr_maybe_hard_refresh(
            bng, n_valid, refresh_every, last_refresh_at_valid)
        need_reload = _did_refresh or (not _session_reuse) or (_prep_try > 0)
        print(f"[CS-simul3-appr] jump{k} prep try={_prep_try + 1}/3 reload={need_reload} "
              f"(refresh={_did_refresh} session_reuse={int(_session_reuse)})", flush=True)
        if need_reload:
            ok, bng = _simul3_appr_reload_scenario(
                bng, sc, _pvid,
                wait_scenario_ready=wait_scenario_ready,
                ensure_freerun=ensure_freerun,
                simul_legs=simul_legs,
                rebind=_did_refresh or _prep_try > 0,
                args=args,
            )
            if not ok:
                print(f"[CS-simul3-appr] jump{k} prep try={_prep_try + 1} reload FAILED", flush=True)
                continue
            _simul3_appr_teleport_spawn(simul_legs, args, _spawn_rot, bng)
            if _did_refresh or _prep_try > 0:
                if not _simul3_appr_wait_spawn_live(bng, simul_legs, args, tag=f"jump{k}-post-tp"):
                    print(f"[CS-simul3-appr] jump{k} prep try={_prep_try + 1} post-teleport spawn dead",
                          flush=True)
                    continue
                rf._step(bng, 8)
                bench.force_gameplay(bng)
        else:
            _simul3_appr_teleport_spawn(simul_legs, args, _spawn_rot, bng)
        if k > 0:
            _camber_stabilize_before_mesh(simul_legs, args, _spawn_rot, bng, tag=f"jump{k}")
        _place_simul3_ramp_meshes(bng, qlua, simul_legs, segs, lsegs, lpts, peak_x, args)
        rf._step(bng, 3)
        bench.force_gameplay(bng)
        _simul3_appr_teleport_spawn(simul_legs, args, _spawn_rot, bng)
        if abs(_runup_camber_deg(args)) > 1e-6:
            try:
                _exp_z = _leg_spawn_xyz(simul_legs[0], args)[2]
                _st_chk = nj._poll(simul_legs[0]["veh"])
                _pz_chk = float((_st_chk.get("pos") or (0, 0, 0))[2])
                if abs(_pz_chk - _exp_z) > 0.35:
                    print(f"[CS-runup-camber] WARN θ={a}° jump{k} spawn pz={_pz_chk:.2f} "
                          f"off apron surface {_exp_z:.2f}(>0.35m)", flush=True)
            except Exception:
                pass
        if _simul3_appr_spawn_health_ok(simul_legs, args):
            return bng, _v_req, last_refresh_at_valid, True
        print(f"[CS-simul3-appr] jump{k} prep try={_prep_try + 1} spawn_health FAIL "
              f"(poll~0, will retry reload)", flush=True)
    return bng, _v_req, last_refresh_at_valid, False

def _set_wheel_factors(veh, fl, fr, rl, rr):
    """Per-motor throttleFactor (differential actuator; probe confirmed independent+invertible).
    factor>0=forward drive (wheels forward -> nose-up reaction); factor<0=reverse (wheels back -> nose-down).
    Global throttle supplies torque; sign from factor (_dart_4motor_diff_probe)."""
    try:
        nj._vlua(veh, f"electrics.values.throttleFactorFL={float(fl):.3f} electrics.values.throttleFactorFR={float(fr):.3f} "
                      f"electrics.values.throttleFactorRL={float(rl):.3f} electrics.values.throttleFactorRR={float(rr):.3f}")
    except Exception:
        pass

class ActuatorLatencyShim:
    """E6 actuator latency: FIFO buffer for controls (incl diff per-wheel factors), step = 1/sps."""

    def __init__(self, latency_ms, *, sps=100.0):
        self.latency_ms = float(latency_ms)
        self.n_delay = max(0, int(round(self.latency_ms * float(sps) / 1000.0)))
        self.n_delay_steps = self.n_delay
        self._q = []
        self._pending_wf = None

    def set_wheel_factors(self, fl, fr, rl, rr):
        self._pending_wf = (float(fl), float(fr), float(rl), float(rr))

    def apply_control(self, veh, thr, brk, steer, pb):
        wf = self._pending_wf
        self._pending_wf = None
        self._q.append((float(thr), float(brk), float(steer), float(pb), wf))
        if len(self._q) <= self.n_delay:
            d_thr, d_brk, d_steer, d_pb, d_wf = 0.0, 0.0, 0.0, 0.0, None
        else:
            d_thr, d_brk, d_steer, d_pb, d_wf = self._q[-self.n_delay - 1]
        if d_wf is not None:
            _set_wheel_factors(veh, *d_wf)
        try:
            veh.control(throttle=d_thr, brake=d_brk, steering=float(d_steer), parkingbrake=d_pb)
        except Exception:
            pass

def apply_takeoff_state_jitter(args, v_entry, *, jump_seed):
    """E6 takeoff-state Gaussian jitter: pitch/roll (deg) + v_entry (m/s) add N(0,sigma). sigma=0 bit-exact."""
    sigma = float(getattr(args, "takeoff_state_jitter_sigma", 0.0) or 0.0)
    if sigma <= 0.0:
        args.__dict__.pop("_takeoff_jitter_audit", None)
        return float(v_entry), None
    base_seed = int(getattr(args, "takeoff_state_jitter_seed", 20260628))
    rng = random.Random(base_seed + int(jump_seed))
    d_pitch = rng.gauss(0.0, sigma)
    d_roll = rng.gauss(0.0, sigma)
    d_v = rng.gauss(0.0, sigma)
    base_pitch = float(getattr(args, "air_impulse_pitch_deg", 0.0) or 0.0)
    base_roll = float(getattr(args, "air_impulse_roll_deg", 0.0) or 0.0)
    if not int(getattr(args, "cond_jitter", 0) or 0):
        base_pitch = float(args.__dict__.get("_base_air_impulse_pitch_deg", base_pitch) or 0.0)
        base_roll = float(args.__dict__.get("_base_air_impulse_roll_deg", base_roll) or 0.0)
    pitch = base_pitch + d_pitch
    roll = base_roll + d_roll
    v_eff = max(0.0, float(v_entry) + d_v)
    args.__dict__["air_impulse_pitch_deg"] = round(pitch, 3)
    args.__dict__["air_impulse_roll_deg"] = round(roll, 3)
    audit = {
        "sigma": round(sigma, 4),
        "seed": base_seed + int(jump_seed),
        "delta_pitch_deg": round(d_pitch, 3),
        "delta_roll_deg": round(d_roll, 3),
        "delta_v_mps": round(d_v, 3),
        "pitch_deg": round(pitch, 3),
        "roll_deg": round(roll, 3),
        "v_entry_mps": round(v_eff, 3),
    }
    args.__dict__["_takeoff_jitter_audit"] = audit
    return v_eff, audit

def prep_jump_e6_injection(args, v_entry, *, jump_seed):
    """Each jump entry: takeoff jitter + return effective v_entry (for one_jump/simul3)."""
    return apply_takeoff_state_jitter(args, v_entry, jump_seed=jump_seed)

def _init_actuator_shim(args):
    lat = float(getattr(args, "actuator_latency_ms", 0.0) or 0.0)
    if lat > 0.0:
        sps = float(getattr(args, "sim_steps_per_second", 100) or 100)
        args.__dict__["_actuator_shim"] = ActuatorLatencyShim(lat, sps=sps)
    else:
        args.__dict__.pop("_actuator_shim", None)

def _act_set_wheel_factors(veh, args, fl, fr, rl, rr):
    shim = args.__dict__.get("_actuator_shim")
    if shim is not None:
        shim.set_wheel_factors(fl, fr, rl, rr)
    else:
        _set_wheel_factors(veh, fl, fr, rl, rr)

def _act_vehicle_control(veh, args, thr, brk, steer, pb):
    shim = args.__dict__.get("_actuator_shim")
    if shim is not None:
        shim.apply_control(veh, thr, brk, steer, pb)
    else:
        try:
            veh.control(throttle=thr, brake=brk, steering=float(steer), parkingbrake=pb)
        except Exception:
            pass

def _gate_lipmap_cached(args):
    """Load a saved LipMap fit (cached on args). Fail-closed if G1 did not pass."""
    p = str(getattr(args, "gate_lipmap", "") or "")
    if not p:
        return None
    cached = args.__dict__.get("_gate_lipmap_obj")
    if cached is not None and args.__dict__.get("_gate_lipmap_path") == p:
        return cached
    from control.dart.lip_map import load_lip_map_fit
    lm = load_lip_map_fit(p)
    args.__dict__["_gate_lipmap_obj"] = lm
    args.__dict__["_gate_lipmap_path"] = p
    print(
        f"[CS-gate-lipmap] loaded {p}: family={lm.omega_fit.family} "
        f"params={lm.omega_fit.params} v_range={lm.v_range}",
        flush=True,
    )
    return lm

def reachability_gate_decision(args, angle, v_req, target_pitch_deg):
    """Pre-takeoff go/no-go gate from the paper source (arXiv 2607.29011).

    Returns ``(v_eff, audit)``. Optional LipMap / directional-budget /
    certified-speed scan match the published gate; defaults stay legacy
    (ramp-angle takeoff, zero rate, infinite budget).
    """
    from control.dart.reachability import directional_rate_budgets
    from control.dart.go_nogo import certified_speed_window

    theta_L = math.radians(target_pitch_deg)
    lm = _gate_lipmap_cached(args)
    _lm_pred = None
    _gate_on = bool(int(getattr(args, "reachability_gate", 0)))
    _climb_loss = float(getattr(args, "gate_climb_loss_mps", 0.0) or 0.0)
    if _gate_on:
        _v_shape = float(getattr(args, "gate_v_crit", 8.0))
        if bool(int(getattr(args, "gate_lip_power_recover", 0))):
            _lt = float(getattr(args, "gate_lip_launch_target", 0.0) or 0.0)
            if _lt > 0.0:
                _v_shape = _lt
        v0_exp = max(0.5, min(float(v_req), _v_shape) - _climb_loss)
    else:
        v0_exp = float(v_req)
    if lm is not None:
        _v_lo, _v_hi = lm.v_range
        _v_q = min(max(v0_exp, _v_lo), _v_hi)
        th0_d, om0_dps, T_pred = lm.predict(_v_q)
        theta0_pred = math.radians(th0_d)
        omega_y0_pred = math.radians(om0_dps)
        T = max(0.05, float(T_pred))
        _lm_pred = {
            "v_query": round(_v_q, 2),
            "v0_expected": round(v0_exp, 2),
            "v_clamped": bool(abs(_v_q - v0_exp) > 1e-9),
            "theta0_deg": round(th0_d, 2),
            "omega_y0_dps": round(om0_dps, 1),
            "T_s": round(T, 3),
            "family": lm.omega_fit.family,
        }
    else:
        theta0_pred = math.radians(float(angle))
        omega_y0_pred = 0.0
        T = float(getattr(args, "gate_flight_time_s", 0.95))
    a_pitch = math.radians(float(getattr(args, "gate_a_pitch_dps2", 120.0)))
    _b_up = _b_dn = None
    if bool(int(getattr(args, "gate_wheel_budget", 0) or 0)):
        _w0 = max(0.0, float(v_req)) / max(1e-6, float(args.wheel_r))
        _b_up, _b_dn = directional_rate_budgets(
            float(getattr(args, "gate_wheel_iw", 1.2)),
            float(getattr(args, "gate_wheel_omega_max_radps", 125.7)),
            float(getattr(args, "gate_wheel_omega_min_radps", -125.7)),
            _w0,
            float(getattr(args, "gate_j_y", 2043.0)),
        )
    _omega_bar = math.radians(float(getattr(args, "gate_omega_bar_dps", 0.0) or 0.0))
    rset = TakeoffReachableSet(
        theta_L=theta_L, T=max(1e-3, T), a=max(1e-6, a_pitch),
        B_up=_b_up, B_down=_b_dn, omega_bar=_omega_bar,
    )
    gate = GoNoGoGate(
        reachable_set=rset,
        v_crit=float(getattr(args, "gate_v_crit", 8.0)),
        a_brake=float(getattr(args, "gate_a_brake", 4.0)),
        enabled=_gate_on,
        speed_margin=float(getattr(args, "gate_speed_margin", 0.0)),
    )
    d_lip = float(args.run_up)
    dec = gate.evaluate(
        current_speed=float(v_req), distance_to_lip=d_lip,
        theta0_pred=theta0_pred, omega_y0_pred=omega_y0_pred,
    )
    v_eff = float(v_req)
    aborted = False
    _abort_reason = None
    if dec.decision == GoNoGo.NOGO:
        if dec.action == SafetyAction.DECELERATE and dec.recommended_target_speed is not None:
            v_eff = min(float(v_req), float(dec.recommended_target_speed))
        elif dec.action == SafetyAction.ABORT_JUMP:
            aborted = True
            _abort_reason = dec.reason
            if dec.v_max is not None:
                v_eff = min(float(v_req), float(dec.v_max))
    _v_min = float(getattr(args, "gate_v_min_clearance", 0.0) or 0.0)
    if _gate_on and _v_min > 0.0:
        _v_lip_cap = float(getattr(args, "gate_v_crit", 8.0))
        if _v_lip_cap < _v_min - 1e-9:
            aborted = True
            _abort_reason = (
                f"speed_window_empty(v_crit={round(_v_lip_cap, 2)}"
                f"<v_min_clearance={_v_min})"
            )
    _vcert_audit = None
    if lm is not None and bool(int(getattr(args, "gate_vcert", 0) or 0)) and _gate_on:
        def _lm_predict_rad(v):
            th_d, om_d, T_s = lm.predict(v)
            return math.radians(th_d), math.radians(om_d), T_s

        _budgets_fn = None
        if bool(int(getattr(args, "gate_wheel_budget", 0) or 0)):
            def _budgets_fn(v):
                return directional_rate_budgets(
                    float(getattr(args, "gate_wheel_iw", 1.2)),
                    float(getattr(args, "gate_wheel_omega_max_radps", 125.7)),
                    float(getattr(args, "gate_wheel_omega_min_radps", -125.7)),
                    max(0.0, v) / max(1e-6, float(args.wheel_r)),
                    float(getattr(args, "gate_j_y", 2043.0)),
                )

        _sw_lo, _sw_hi = lm.v_range
        if _v_min > 0.0:
            _sw_lo = max(_sw_lo, _v_min)
        _v_shape_cap = max(0.5, min(float(v_req), float(getattr(args, "gate_v_crit", 8.0))))
        win = certified_speed_window(
            _lm_predict_rad, theta_L=theta_L,
            a_pitch=math.radians(float(getattr(args, "gate_a_pitch_dps2", 120.0))),
            v_lo=_sw_lo, v_hi=_sw_hi, step=0.5, v_cap=None,
            budgets_fn=_budgets_fn, omega_bar=_omega_bar,
        )
        _member = win.contains(v0_exp)
        if win.empty:
            aborted = True
            _abort_reason = f"vcert_window_empty(binding={win.binding_constraint})"
        _vcert_audit = {
            "omega_bar_dps": round(math.degrees(_omega_bar), 1),
            "intervals": [[round(a2, 2), round(b2, 2)] for a2, b2 in win.intervals],
            "v_target": (round(win.v_target, 2) if win.v_target is not None else None),
            "binding_constraint": win.binding_constraint,
            "v0_expected": round(v0_exp, 2),
            "v0_in_window": bool(_member),
            "v_shape_cap": round(_v_shape_cap, 2),
            "v_target_shapeable": (
                win.v_target is not None and win.v_target <= _v_shape_cap + 1e-9
            ),
            "recommended_v_crit": (
                round(win.v_target + _climb_loss, 2)
                if win.v_target is not None else None
            ),
        }
    args.__dict__["_gate_aborted"] = aborted
    audit = {
        "enabled": _gate_on,
        "decision": ("NOGO" if aborted else dec.decision.name),
        "action": ("ABORT_JUMP" if aborted else dec.action.name),
        "reason": (_abort_reason if aborted and _abort_reason else dec.reason),
        "v_max": (round(dec.v_max, 3) if dec.v_max is not None else None),
        "v_requested": round(float(v_req), 3), "v_effective": round(v_eff, 3),
        "intervened": bool(abs(v_eff - float(v_req)) > 1e-6 or aborted),
        "aborted": aborted,
        "theta0_pred_deg": round(math.degrees(theta0_pred), 1),
        "omega_y0_pred_dps": round(math.degrees(omega_y0_pred), 1),
        "theta_L_deg": round(float(target_pitch_deg), 1),
        "T_flight_s": round(T, 3),
        "v_crit": float(getattr(args, "gate_v_crit", 8.0)),
        "a_brake": float(getattr(args, "gate_a_brake", 4.0)),
        "a_pitch_dps2": float(getattr(args, "gate_a_pitch_dps2", 120.0)),
        "v_min_clearance": _v_min,
        "distance_to_lip": d_lip,
        "speed_margin": float(getattr(args, "gate_speed_margin", 0.0)),
        "lipmap_pred": _lm_pred,
        "budgets_dps": (
            {"B_up": round(math.degrees(_b_up), 2), "B_down": round(math.degrees(_b_dn), 2)}
            if _b_up is not None else None
        ),
        "vcert": _vcert_audit,
    }
    return v_eff, audit

def _airborne_ctrl(args, dart_on, eff_baseline, veh, *, pitch_d, pdot, roll_d, pz, gspd, target_pitch_deg, nc=0, air_streak=0, strategy=None, yaw_err_air=None):
    """Airborne control (per strategy): dart / rwpd / tobb / human / passive.
    Returns (thr, brk, steer, pb). dart/human/passive mirror one_jump branches (continuous only)."""
    strat = strategy or ("dart" if dart_on else eff_baseline)
    if strat == "rwpd":
        omega_w = wheel_w(veh) or 0.0
        omega_tgt = max(gspd / args.wheel_r, 0.0)
        thr, brk, steer = rwpd_airborne_ctrl(
            pitch_d, pdot, roll_d, target_pitch_deg,
            kp=float(getattr(args, "rwpd_kp_pitch", args.air_trim_kp)),
            kd=float(getattr(args, "rwpd_kd_pitch", args.air_trim_kd)),
            k_roll=float(getattr(args, "rwpd_k_roll", args.k_roll)),
            smax=float(getattr(args, "rwpd_smax", args.smax)),
            omega_w=omega_w, omega_tgt=omega_tgt, omega_cap=float(getattr(args, "omega_cap", 0.0)))
        return thr, brk, steer, 0.0
    if strat == "tobb":
        omega_w = wheel_w(veh) or 0.0
        omega_tgt = max(gspd / args.wheel_r, 0.0)
        thr, brk, steer = tobb_airborne_ctrl(
            pitch_d, pdot, roll_d, target_pitch_deg,
            a_max_dps2=float(getattr(args, "tobb_a_max_dps2", 120.0)),
            k_roll=float(getattr(args, "tobb_k_roll", args.k_roll)),
            smax=float(getattr(args, "tobb_smax", args.smax)),
            omega_w=omega_w, omega_tgt=omega_tgt, omega_cap=float(getattr(args, "omega_cap", 0.0)))
        return thr, brk, steer, 0.0
    steer = 0.0
    _diff = str(getattr(args, "dart_pitch_control", "continuous")) == "differential"
    _air_cfg = _c7_air_cfg(args)
    _in_p1a = False
    _camber_early = False
    _camber_touch = False
    _ab_pitch_on = True
    _ab_roll_gain = float(getattr(args, "diff_roll_steer_gain", 0.0) or 0.0)
    _adp_force = (strat == "dart_latched")
    if strat == "dart_pitch_only":
        _ab_roll_gain = 0.0
    elif strat == "dart_roll_only":
        _ab_pitch_on = False
    if dart_on:
        omega_w = wheel_w(veh) or 0.0
        omega_tgt = max(gspd / args.wheel_r, 0.0)
        _camber_early = _camber_early_landmatch(args, nc=nc, pz=pz, air_cfg=_air_cfg)
        _in_p1a = float(pz) > float(args.land_match_z) and not _camber_early
        _camber_touch = (_air_cfg.get("active") and int(nc) > 0
                         and float(pz) <= float(_air_cfg.get("early_landmatch_z", 999.0)))
        if _in_p1a:
            err = target_pitch_deg - pitch_d
            in_win = pz <= float(args.dart_action_z_max)
            outside_db = (abs(err) > float(args.dart_pitch_deadband_deg)
                          or abs(pdot) > float(args.dart_rate_deadband_dps))
            if _diff:
                cap = float(getattr(args, "diff_omega_cap", 0) or 0)
                omega_ok = (cap <= 0) or (float(omega_w) <= cap)
                clear = air_streak >= int(getattr(args, "diff_engage_air_steps", 6))
                if in_win and clear and nc == 0 and omega_ok:
                    if not _ab_pitch_on:
                        f_pitch = 0.0
                    elif int(getattr(args, "diff_pitch_naive", 0)):
                        f_pitch = max(-1.0, min(1.0,
                                      args.diff_naive_kp * err / 20.0 - args.diff_naive_kd * pdot / 100.0))
                    else:
                        des_rate = max(-args.diff_rate_max, min(args.diff_rate_max, args.diff_pitch_rate_kp * err))
                        f_pitch = max(-1.0, min(1.0, args.diff_k_drive * (des_rate - pdot) / 100.0))
                    _rsg = _ab_roll_gain
                    if _air_cfg.get("active"):
                        _roll_tgt = float(_air_cfg.get("roll_target_deg", 0.0) or 0.0)
                        _roll_err = float(roll_d) - _roll_tgt
                        _rdb = float(_air_cfg.get("touch_roll_deadband_deg", 1.0) or 1.0)
                        roll_active = abs(_roll_err) > _rdb
                    else:
                        _roll_err = float(roll_d)
                        roll_active = (abs(_rsg) > 1e-9) and (abs(roll_d) > float(getattr(args, "dart_roll_deadband_deg", 2.0)))
                    _w_roll = (_c7_adaptive_roll_weight(args, _roll_err, yaw_err_air,
                                                        force=_adp_force, strat=strat)
                               if roll_active else 1.0)
                    if roll_active and _w_roll <= 1e-3:
                        roll_active = False
                    f_spin = max(0.0, min(1.0, float(getattr(args, "diff_k_roll", 0.0) or 0.0))) if roll_active else 0.0
                    base = f_pitch if abs(f_pitch) >= f_spin else (f_spin if f_pitch >= 0 else -f_spin)
                    fl = fr = rl = rr = max(-1.0, min(1.0, base))
                    _act_set_wheel_factors(veh, args, fl, fr, rl, rr)
                    thr, brk = 1.0, 0.0
                    if _air_cfg.get("active"):
                        steer = _camber_air_roll_steer(roll_d, args, _air_cfg, nc=nc) if roll_active else 0.0
                    else:
                        _rcap = float(getattr(args, "diff_roll_steer_max", 1.0) or 1.0)
                        steer = max(-_rcap, min(_rcap, _w_roll * _rsg * _roll_err / 30.0)) if roll_active else 0.0
                else:
                    if _diff:
                        _act_set_wheel_factors(veh, args, 0.0, 0.0, 0.0, 0.0)
                    thr, brk = 0.0, 0.0
                    steer = _diff_nc_touch_roll_steer(
                        roll_d, args, _air_cfg, nc=int(nc), roll_gain=_ab_roll_gain,
                        yaw_err_air=yaw_err_air, adp_force=_adp_force, strat=strat)
                pb = 0.0
            else:
                if in_win and outside_db:
                    if _air_cfg.get("active"):
                        u = _camber_air_pitch_u(
                            args, _air_cfg, pitch_d=pitch_d, pdot=pdot,
                            target_pitch_deg=target_pitch_deg, p1a=True)
                    else:
                        u = args.kp_pitch * err / 20.0 - args.kd_pitch * pdot / 100.0
                    if u > 0 and omega_w > args.omega_cap * max(omega_tgt, 1.0):
                        u = 0.0
                    thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
                    if _camber_touch and brk > 0.0:
                        brk = min(brk, float(_air_cfg["pitch_brk_touch_cap"]))
                else:
                    thr, brk = 0.0, 0.0
                if _air_cfg.get("active"):
                    steer = _camber_air_roll_steer(roll_d, args, _air_cfg, nc=nc)
                else:
                    steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0))
                pb = 0.0
        elif int(args.dart_disable_landmatch):
            if _diff: _act_set_wheel_factors(veh, args, 1.0, 1.0, 1.0, 1.0)
            thr, brk, pb = 0.0, 0.0, 0.0; steer = 0.0
        elif args.landmatch or _camber_early:
            if _diff: _act_set_wheel_factors(veh, args, 1.0, 1.0, 1.0, 1.0)
            d_omega = omega_w - omega_tgt
            _pitch_ok = (_camber_pitch_near_target(pitch_d, target_pitch_deg, _air_cfg)
                         if _air_cfg.get("active") else abs(pitch_d) < 8.0)
            if d_omega > args.omega_tol:
                thr = 0.0; brk = min(args.land_brake_cap, args.kp_omega * d_omega / 100.0)
            elif d_omega < -args.omega_tol and _pitch_ok:
                thr = min(args.land_brake_cap, args.kp_omega * (-d_omega) / 100.0); brk = 0.0
            else:
                thr, brk = 0.0, 0.0
            _rate_tol = (float(getattr(args, "camber_air_p1b_rate_tol_dps", 30.0) or 30.0)
                         if _air_cfg.get("active") else float(args.rate_tol))
            if abs(pdot) > _rate_tol and _pitch_ok:
                if pdot > 0 and brk == 0.0:
                    brk = min(args.land_brake_cap, args.kd_land * pdot / 100.0); thr = 0.0
                elif pdot < 0 and thr == 0.0:
                    thr = min(args.land_brake_cap, args.kd_land * (-pdot) / 100.0); brk = 0.0
            if _camber_early and _air_cfg.get("active"):
                u = _camber_air_pitch_u(
                    args, _air_cfg, pitch_d=pitch_d, pdot=pdot,
                    target_pitch_deg=target_pitch_deg, p1a=False)
                _pcap = float(getattr(args, "camber_air_p1b_pitch_cap", 0.48) or 0.48)
                thr, brk = _blend_camber_pitch_act(thr, brk, u, cap=_pcap)
            steer = 0.0
            if _camber_early or _air_cfg.get("active"):
                steer = (_camber_p2_touch_roll_steer(roll_d, args, _air_cfg, nc=int(nc))
                         if int(nc) >= 1 else _camber_air_roll_steer(roll_d, args, _air_cfg, nc=nc))
            pb = 0.0
        else:
            if _diff: _act_set_wheel_factors(veh, args, 1.0, 1.0, 1.0, 1.0)
            err = target_pitch_deg - pitch_d
            u = args.kp_pitch * err / 20.0 - args.kd_pitch * pdot / 100.0
            thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
            steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0)); pb = 0.0
    elif eff_baseline == "human":
        err = target_pitch_deg - pitch_d
        u = args.air_trim_kp * err / 20.0 - args.air_trim_kd * pdot / 100.0
        omega_w = wheel_w(veh) or 0.0
        omega_tgt = max(gspd / args.wheel_r, 0.0)
        if u > 0 and omega_w > args.omega_cap * max(omega_tgt, 1.0):
            u = 0.0
        thr, brk = (min(1.0, u), 0.0) if u > 0 else (0.0, min(1.0, -u))
        steer = max(-args.smax, min(args.smax, -args.k_roll * roll_d / 30.0)); pb = 0.0
    else:
        thr, brk, pb = 0.0, 0.0, 0.0; steer = 0.0
    if dart_on and yaw_err_air is not None and (
            _in_p1a or _camber_early
            or (_air_cfg.get("active") and int(nc) >= 1 and (args.landmatch or _camber_early))):
        _sc = float(getattr(args, "diff_roll_steer_max", 1.0) or 1.0) if _diff else float(args.smax)
        if _air_cfg.get("active"):
            _sc = max(_sc, float(_air_cfg["yaw_steer_max"]), float(_air_cfg["roll_steer_max"]))
        steer = _c7_air_yaw_hold_steer(
            steer, yaw_err_air, args, steer_cap=_sc, air_cfg=_air_cfg,
            roll_d=roll_d, nc=nc)
    return thr, brk, steer, pb

def one_jump_simul3(bng, qlua, legs, *, angle, base_x, run_up, v_entry, R_flight,
                    peak_x, peak_z, pts, ramp_idx, n_ramps, args, ground_launch=False):
    """Synchronized three-vehicle jump. legs = [{label, veh, dart_on, baseline, y, color}, ...].
      - air-impulse (ground_launch=False, default): same tick teleport->settle->set_velocity launch.
      - approach (ground_launch=True): no teleport, ground spawn (EV box in D), same tick full throttle
        run-up climb, natural takeoff at lip. Run-up control identical; deterministic stepping -> nearly identical takeoff,
        divergence only airborne -> same-tick pairing. Diversity from caller sweeping v_entry/angle per jump.
    Then per-vehicle poll/detect/control/act/trace each step. Returns {label: result_dict} (fields match one_jump)."""
    if str(getattr(args, "dart_pitch_control", "continuous")) not in ("continuous", "differential"):
        raise ValueError("simul-3way only supports --dart-pitch-control continuous | differential "
                         "(pulse/phased/steer-probe are exploratory, not in three-way validation)")
    _init_actuator_shim(args)
    args.__dict__["_adp_hyst"] = {}
    args.__dict__.pop("_cam_last_phase", None)
    # control.resume() returns to real-time free-run -> control.step(1) ~0.2s (3 vehicles), whole air segment ~4 updates
    # unrepresentative pairing. control.pause() back to stepped: control.step(1)=fixed 1/sps~0.01s,
    # 0.8s air -> ~80 control updates. **No resume/force_gameplay here** (stepped mode pauses between steps).
    # approach(ground_launch): default run-up also paused stepping (approach_ground_paused=1) -> matched takeoff + fast;
    # If EV won't drive from rest while paused, set 0 for gameplay-live run-up (pause after first takeoff). _paused_air in main loop.
    _defer_pause = ground_launch and not bool(int(getattr(args, "approach_ground_paused", 1)))
    _paused_air = not _defer_pause
    if not _defer_pause:
        try: bng.control.pause()
        except Exception: pass
    syaw = math.radians(270.0)
    rot = (0.0, 0.0, math.sin(syaw / 2), math.cos(syaw / 2))
    _air_roll = float(getattr(args, "air_impulse_roll_deg", 0.0) or 0.0)
    if abs(float(args.air_impulse_pitch_deg)) > 1e-6 or abs(_air_roll) > 1e-6:
        yaw = syaw; pitch = math.radians(float(args.air_impulse_pitch_deg)); roll = math.radians(_air_roll)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        rot = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
               cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    sx = max(pts[0][0] + 0.5, peak_x - float(args.lip_launch_m))
    sz = peak_z + float(args.lip_launch_z_offset)
    tp_reset = bool(getattr(args, "teleport_reset", 1))
    target_pitch_deg = (float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg))
                        + float(args.landing_flare_deg))
    profile = args.__dict__.get("_current_landing_profile")
    trace_path = args.__dict__.get("_control_trace_path")
    impulse_dt = float(args.lip_impulse_dt)
    if not ground_launch:
        for leg in legs:
            veh = leg["veh"]
            _dx = _leg_base_x(leg, args) - float(base_x)
            _lp = _leg_peak_x(leg, peak_x)
            _lsx = max(pts[0][0] + 0.5 + _dx, _lp - float(args.lip_launch_m))
            try: veh.teleport((_lsx, float(leg["y"]), sz), rot, reset=tp_reset)
            except Exception: pass
            try: veh.ai.set_mode("disabled")
            except Exception: pass
        rf._step(bng, 3)
        for _ in range(2):
            for leg in legs:
                try: leg["veh"].control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=0.0)
                except Exception: pass
            rf._step(bng, 1)
        for leg in legs:
            try: nj._vlua(leg["veh"], "electrics.values.throttleFactorFL=1 electrics.values.throttleFactorFR=1 "
                                      "electrics.values.throttleFactorRL=1 electrics.values.throttleFactorRR=1")
            except Exception: pass
        v_launch = float(v_entry)
        _inj_settle = max(3, int(float(args.lip_impulse_hold_steps)))
        for leg in legs:
            try: leg["veh"].set_velocity(v_launch, impulse_dt)
            except Exception: pass
        rf._step(bng, _inj_settle)
        _v0_tgt = float(getattr(args, "air_launch_v0_target", 0.0) or 0.0)
        if _v0_tgt > 0:
            _v0_tol = float(getattr(args, "air_launch_v0_tol", 0.3) or 0.3)
            _v0_tries = max(1, int(getattr(args, "air_launch_v0_max_tries", 8) or 8))
            _req = {leg["label"]: _v0_tgt for leg in legs}
            _worst = 0.0
            for _try in range(_v0_tries):
                _worst = 0.0
                for leg in legs:
                    st_ = nj._poll(leg["veh"])
                    vv = st_.get("vel") or (0, 0, 0)
                    gs = math.hypot(float(vv[0]), float(vv[1]))
                    _worst = max(_worst, abs(gs - _v0_tgt))
                    if abs(gs - _v0_tgt) > _v0_tol:
                        lab = leg["label"]
                        ratio = _v0_tgt / max(gs, 0.5)
                        _req[lab] = max(0.15 * _v0_tgt, min(6.0 * _v0_tgt,
                                        _req[lab] * max(0.25, min(3.0, ratio))))
                        try: leg["veh"].set_velocity(_req[lab], impulse_dt)
                        except Exception: pass
                if _worst <= _v0_tol:
                    print(f"[CS-v0-gate] converged try={_try} worst_dv={_worst:.2f} target={_v0_tgt} "
                          f"req={ {k: round(v, 2) for k, v in _req.items()} }", flush=True)
                    break
                rf._step(bng, 3)
            else:
                print(f"[CS-v0-gate] WARN not converged after {_v0_tries} tries worst_dv={_worst:.2f} "
                      f"target={_v0_tgt} req={ {k: round(v, 2) for k, v in _req.items()} } (treat data as discounted)", flush=True)
            args.__dict__["_air_launch_v0_gate_audit"] = {
                "target": _v0_tgt, "tol": _v0_tol, "worst_dv_final": round(_worst, 3),
                "final_requests": {k: round(v, 2) for k, v in _req.items()}}
        _prk = float(getattr(args, "air_impulse_pitch_rate_dps", 0.0) or 0.0)
        if abs(_prk) > 1e-6:
            _pre = {l["label"]: _read_pitch_rate_dps(l["veh"]) for l in legs}
            _apply_pitch_rate_kick([l["veh"] for l in legs], _prk)
            _sps = int(getattr(args, "sim_steps_per_second", 100) or 100)
            rf._step(bng, int(math.ceil(RATE_KICK_DT_S * _sps)) + 1)
            _post = {l["label"]: _read_pitch_rate_dps(l["veh"]) for l in legs}
            args.__dict__["_air_pitch_rate_kick_audit"] = {
                "kick_dps": round(_prk, 2), "axis": "world_y",
                "impl": "thrusters.applyAccel", "dt_s": RATE_KICK_DT_S,
                "readback_pre_dps": _pre, "readback_post_dps": _post}
            print(f"[CS-rate-kick] pitch-rate kick {_prk:+.1f} deg/s via thrusters.applyAccel: "
                  f"readback pre={_pre} post={_post}", flush=True)
    else:
        # No velocity inject, no teleport (avoid gear-lock); throttleFactor=1 four-motor drive; ai off, script control.
        for leg in legs:
            try: leg["veh"].ai.set_mode("disabled")
            except Exception: pass
            try: nj._vlua(leg["veh"], "electrics.values.throttleFactorFL=1 electrics.values.throttleFactorFR=1 "
                                      "electrics.values.throttleFactorRL=1 electrics.values.throttleFactorRR=1")
            except Exception: pass
        for _ in range(2):
            for leg in legs:
                try: leg["veh"].control(throttle=0.0, brake=0.0, steering=0.0, parkingbrake=0.0)
                except Exception: pass
            rf._step(bng, 1)

    def _new_state():
        return {"took_off": False, "t_takeoff": None, "t_land": None, "land": None, "air_streak": 0,
                "max_z": -9.0, "max_roll_air": 0.0, "max_yaw_delta_air": 0.0, "max_pitchrate": 0.0,
                "prev_pitch": None, "rwpd_prev": None, "tumbled": False, "past_apex": False, "apex_z": -9.0,
                "wall_takeoff": None, "wall_land": None, "dt_samples": [], "takeoff_state": None,
                "yaw0_air": None, "land_yaw_delta": None, "max_yaw_delta_air_dv": 0.0, "land_yaw_delta_dv": None,
                "vz_prev": None, "max_land_accel": 0.0, "ctrl_t_samples": [],
                "wheel_w_land": None, "omega_tgt_land": None,
                "front_clear_land": None, "damage_land": None, "land_x_world": None, "land_ground_z": None,
                "land_on_mesh": None, "apex_clearance_land": None, "land_vz_mps": None,
                "land_impact_speed_mps": None, "land_height_above_ground": None,
                "trace_rows": ([] if trace_path else None), "roll_t0": None, "prev_x": sx,
                "lane_i_err": 0.0, "land_safety_done": False, "post_land_flip": False,
                "gate_a_brake_est": None, "gate_prev_px": None, "gate_prev_v": None,
                "gate_brake_prev": False, "gate_brk_v0": None, "gate_brk_x0": None,
                "gate_spawn_v": None, "gate_vmax_lip": None, "gate_intervened_steps": 0}
    states = {leg["label"]: _new_state() for leg in legs}
    cam_pose = None
    cams = None
    try:
        _far_x = args.__dict__.get("_current_land_far_x") or args.__dict__.get("_current_land_toe_x")
        _span_cap = float(getattr(args, "cam_c_span_m", 32.0))
        if _far_x is not None and float(_far_x) > peak_x:
            _span_cap = max(_span_cap, float(_far_x) - peak_x)
        _cam_look_y = _simul_cam_look_y(legs)
        _cam_look_y_ab = _simul_cam_look_y_ab(legs, args)
        _cam_lat = _simul_lateral_y_span(legs) if _simul_y_copy(args) or _simul_layout(args) == "y_lane" else 0.0
        _ab_pre = float(getattr(args, "cam_approach_pre_start_m", 2.0))
        _ab_zdrop = float(getattr(args, "cam_approach_z_drop_m", 8.0))
        _ab_pull = 0.0
        if _simul_y_copy(args):
            _ab_pre = max(_ab_pre, 6.0)
            _ab_zdrop = min(_ab_zdrop, 3.0)
            _ab_pull = 22.0
        cams, _cm, _lx = build_phase_cams(base_x, run_up, peak_x, R_flight, args.rise,
                                          land_far_x=_far_x, c_span_cap=_span_cap,
                                          approach_post_lip_m=float(getattr(args, "cam_approach_post_lip_m", 2.0)),
                                          approach_pre_start_m=_ab_pre,
                approach_cam_z_drop_m=_ab_zdrop,
                cam_c_z_drop_m=_cam_c_z_drop_from_args(args),
                look_y=_cam_look_y, look_y_ab=_cam_look_y_ab, lateral_y_span=_cam_lat,
                ab_side_extra_pullback_m=_ab_pull)
        cam_pose = cams["C"]
        args.__dict__["_cam_c_metas"] = _cm
        _abm = _cm.get("AB") or {}
        print(f"[CS-cam-simul3] θ={angle}° layout={_simul_layout(args)} "
              f"AB:look_y={_abm.get('look_y')} pull={_abm.get('side_extra_pullback_m')}m "
              f"ang_start={_abm.get('ang_start')}° | C:look_y={_cam_look_y:.1f} lat={_cam_lat:.1f}m "
              f"z={_cm['C']['z']} ang_peak={_cm['C'].get('ang_peak')}°", flush=True)
    except Exception:
        cam_pose = None; cams = None
    # Airtime provenance: simul3 runs deterministic stepped mode (control.pause), each rf._step(bng,1)=fixed 1/sps sec.
    _dt_sim = 1.0 / float(getattr(args, "sim_steps_per_second", 100) or 100)
    _sps = float(getattr(args, "sim_steps_per_second", 100) or 100)
    _post_land_steps = int(round(_post_land_sec_simul3(args) * _sps))
    _post_land_max_steps = int(round(_post_land_max_sec_simul3(args) * _sps))
    _postfail_steps = max(5, int(round(_postfail_hold_sec(args) * _sps)))
    _gspd_gate = _post_land_gspd_gate_mps(args)
    print(f"[CS-simul3-hold] hud={int(_hud_on(args))} post_land≥{_post_land_sec_simul3(args):g}s "
          f"max={_post_land_max_sec_simul3(args):g}s gspd_gate≤{_gspd_gate:g}m/s "
          f"postfail={_postfail_hold_sec(args):g}s → {_post_land_steps}/{_post_land_max_steps}/{_postfail_steps} steps @{_sps:g}Hz",
          flush=True)
    _cam_every = max(1, int(getattr(args, "cam_update_every", 1) or 1))
    if _cam_every > 1:
        print(f"[CS-cam] set_free decimated every {_cam_every} steps (phase switch still immediate)", flush=True)
    _simul_fail_i = None
    t_wall_prev = None
    args.__dict__.pop("_lip_kick_applied", None)
    for i in range(args.max_steps):
        _t_now = time.time()
        dt_step = _dt_sim
        t_wall_prev = _t_now
        n_active = 0
        if i > 0 and i % 100 == 0:
            print(f"[CS-hb] i={i}/{args.max_steps}", flush=True)
        for leg in legs:
            label = leg["label"]; veh = leg["veh"]; dart_on = bool(leg["dart_on"]); eff_baseline = leg["baseline"]
            S = states[label]
            _leg_bx = _leg_base_x(leg, args)
            _leg_peak = _leg_peak_x(leg, peak_x)
            _leg_prof = leg.get("profile") or profile
            st = nj._poll(veh)
            pos = st.get("pos") or (0, 0, 0); vel = st.get("vel") or (0, 0, 0); d = st.get("dir") or (1, 0, 0)
            px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            if ground_launch and i < 2:
                print(f"[CS-spawnpos] i={i} leg={label} px={px:.3f} py={py:.3f} pz={pz:.3f} "
                      f"dist_to_ramp_base={_leg_bx - px:.3f}m", flush=True)
            if ground_launch and i == 0:
                S["approach_spawn_pos"] = [round(px, 4), round(py, 4), round(pz, 4)]
            gspd = math.hypot(float(vel[0]), float(vel[1]))
            vx_fwd = float(vel[0]) * float(d[0]) + float(vel[1]) * float(d[1])
            roll, pitch, yaw = nj._rpy(veh, st); nc = nj._contact(veh) or 0
            roll_d, pitch_d = math.degrees(roll), math.degrees(pitch)
            pdot = (pitch_d - S["rwpd_prev"]) / dt_step if S["rwpd_prev"] is not None else 0.0
            S["rwpd_prev"] = pitch_d
            S["cur_px"], S["cur_pz"], S["cur_pitch"] = px, pz, pitch_d
            S["gspd_mps"] = gspd
            S["air_streak"] = S["air_streak"] + 1 if nc == 0 else 0
            if S["took_off"] and S["t_land"] is None:
                S["dt_samples"].append(dt_step)
            if S["took_off"] and (abs(roll_d) > 80 or abs(pitch_d) > 85):
                S["tumbled"] = True
                if S["t_land"] is not None:
                    S["post_land_flip"] = True
            if (not S["took_off"]) and S["air_streak"] >= 3 and px >= _leg_peak - 3.0 and pz >= (args.rise - args.sink - 0.7):
                S["took_off"] = True; S["t_takeoff"] = i; S["apex_z"] = pz; S["prev_pitch"] = pitch_d
                S["wall_takeoff"] = _t_now
                yaw0_deg = math.degrees(math.atan2(float(d[1]), float(d[0])))
                S["yaw0_air"] = yaw0_deg
                S["takeoff_state"] = {"theta0_deg": round(pitch_d, 1), "omega_y0_dps": round(pdot, 0),
                                      "v0_mps": round(gspd, 2), "roll0_deg": round(roll_d, 1),
                                      "yaw0_deg": round(yaw0_deg, 1), "y0_m": round(py, 2),
                                      "vz0_mps": round(float(vel[2]), 2)}
                _y0tol = float(getattr(args, "lip_yaw0_warn_deg", 8.0) or 8.0)
                if abs(yaw0_deg) > _y0tol:
                    print(f"[CS-lip-yaw0] WARN leg={label} takeoff |yaw0|={abs(yaw0_deg):.1f}° > {_y0tol}° "
                          f"(py={py:.2f}m roll0={roll_d:.1f}°)", flush=True)
                _dist = _build_approach_disturbance_audit(args)
                if _dist:
                    S["takeoff_state"]["approach_disturbance"] = dict(_dist)
                    args.__dict__["_approach_disturbance_audit"] = _dist
            if S["took_off"] and S["t_land"] is None:
                S["max_z"] = max(S["max_z"], pz); S["max_roll_air"] = max(S["max_roll_air"], abs(roll_d))
                if S["yaw0_air"] is not None:
                    S["max_yaw_delta_air"] = max(S["max_yaw_delta_air"],
                                                 abs(_angle_diff_deg(math.degrees(yaw), S["yaw0_air"])))
                    _yaw_dv = math.degrees(math.atan2(float(d[1]), float(d[0])))
                    S["max_yaw_delta_air_dv"] = max(S["max_yaw_delta_air_dv"],
                                                    abs(_angle_diff_deg(_yaw_dv, S["yaw0_air"])))
                if S["prev_pitch"] is not None:
                    pr = abs((pitch_d - S["prev_pitch"]) / dt_step)
                    if pr < 800: S["max_pitchrate"] = max(S["max_pitchrate"], pr)
                S["prev_pitch"] = pitch_d
                if pz > S["apex_z"]: S["apex_z"] = pz
                if (not S["past_apex"]) and pz < S["apex_z"] - 0.5: S["past_apex"] = True
                if S["past_apex"] and (pz < 0.9 or nc >= 3):
                    S["t_land"] = i; S["land"] = (round(pitch_d, 1), round(roll_d, 1)); S["wheel_w_land"] = wheel_w(veh)
                    S["wall_land"] = _t_now; S["land_x_world"] = round(px, 1)
                    gz = _interp_profile_z(_leg_prof, px)
                    S["land_ground_z"] = round(gz, 2) if gz is not None else None
                    S["land_on_mesh"] = bool(gz is not None and abs(py) <= float(args.width) * 0.5 + 1.0)
                    S["land_height_above_ground"] = round(pz - gz, 2) if gz is not None else None
                    S["apex_clearance_land"] = round(S["max_z"] - gz, 2) if gz is not None else None
                    S["land_vz_mps"] = round(float(vel[2]), 2)
                    S["land_impact_speed_mps"] = round(math.sqrt(
                        float(vel[0]) ** 2 + float(vel[1]) ** 2 + float(vel[2]) ** 2), 2)
                    if S["yaw0_air"] is not None:
                        S["land_yaw_delta"] = round(_angle_diff_deg(math.degrees(yaw), S["yaw0_air"]), 1)
                        _land_yaw_dv = math.degrees(math.atan2(float(d[1]), float(d[0])))
                        S["land_yaw_delta_dv"] = round(_angle_diff_deg(_land_yaw_dv, S["yaw0_air"]), 1)
                    S["omega_tgt_land"] = round(gspd / args.wheel_r, 0)
                    S["front_clear_land"] = front_clearance_proxy(
                        pos, d, pitch, peak_x=_leg_peak, peak_z=args.rise - args.sink,
                        beta_deg=float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg)),
                        front_x=args.front_probe_x, front_z_offset=args.front_probe_z_offset)
                    S["damage_land"] = damage_readback(veh)
            if S["took_off"] and S["past_apex"] and (S["t_land"] is None or i <= S["t_land"] + 20):
                _vz = float(vel[2])
                if S["vz_prev"] is not None:
                    _a = abs((_vz - S["vz_prev"]) / dt_step)
                    if _a < 5000.0:
                        S["max_land_accel"] = max(S["max_land_accel"], _a)
                S["vz_prev"] = _vz
            if S["t_land"] is not None:
                thr, brk, steer, pb = 0.0, 1.0, 0.0, 1.0
                args.__dict__["_land_safety_gspd"] = gspd
                if not S.get("land_safety_done"):
                    _landed_vehicle_safety_reset(veh, args, first_step=True)
                    S["land_safety_done"] = True
                else:
                    _landed_vehicle_safety_reset(veh, args, first_step=False)
            elif not S["took_off"]:
                brk = 0.0; pb = 0.0
                if ground_launch:
                    if abs(_runup_camber_deg(args)) > 1e-6:
                        _camber_approach_apron_snap_if_needed(
                            veh, float(leg["y"]), px, gspd, i, args, _spawn_rot_from_args(args), bng,
                            base_x=_leg_bx)
                    yaw_err = math.atan2(float(d[1]), float(d[0]))
                    _d_to_lip = _leg_peak - px
                    if float(_approach_lane_keep_gains(args)[2]) > 1e-9:
                        _ey = float(py) - float(leg["y"])
                        S["lane_i_err"] = max(-1.5, min(1.5, float(S.get("lane_i_err", 0.0)) + _ey * dt_step))
                    steer = _approach_lane_keep_steer(
                        py, yaw_err, float(leg["y"]), args, gspd_mps=gspd,
                        lane_i_err=float(S.get("lane_i_err", 0.0)), d_to_lip_m=_d_to_lip)
                    thr = 0.0
                    _gate_on = _approach_p0_gate_on(args) and dart_on
                    _gate_brake, _gate_in_coast, _vmax_d = _approach_p0_gate_update(
                        args, px=px, gspd=gspd, d_to_lip=_d_to_lip, gs=S)
                    if _prelip_yaw_lock_active(_d_to_lip, yaw_err, args):
                        thr = 0.0
                    elif args.lip_power and px >= (_leg_peak - args.lip_power_m) and not _gate_on:
                        thr = 1.0
                    elif args.lip_throttle_cut_m > 0.0 and px >= (_leg_peak - args.lip_throttle_cut_m):
                        thr = 0.0
                    elif not _gate_on:
                        thr = 1.0 if gspd < v_entry else 0.0
                    thr, brk = _approach_p0_gate_thr_brk(
                        args, thr, brk, gspd=gspd, d_to_lip=_d_to_lip, px=px, peak_x=_leg_peak,
                        v_entry=v_entry, gate_brake=_gate_brake, gate_in_coast=_gate_in_coast,
                        gate_on=_gate_on, vmax_d=_vmax_d)
                    if not _gate_on:
                        thr, brk = _apply_approach_lip_stability(
                            thr, brk, gspd=gspd, d_to_lip_m=_d_to_lip, args=args, v_entry=v_entry)
                else:
                    steer = 0.0
                    if px <= _leg_peak + 1.0:
                        try: veh.set_velocity(v_entry, impulse_dt)
                        except Exception: pass
                    thr = 1.0 if gspd < v_entry else 0.0
            else:
                _ct0 = time.perf_counter()
                thr, brk, steer, pb = _airborne_ctrl(
                    args, dart_on, eff_baseline, veh, pitch_d=pitch_d, pdot=pdot, roll_d=roll_d,
                    pz=pz, gspd=gspd, target_pitch_deg=target_pitch_deg, nc=nc, air_streak=S["air_streak"],
                    strategy=leg.get("strategy"),
                    yaw_err_air=math.atan2(float(d[1]), float(d[0])))
                S["ctrl_t_samples"].append((time.perf_counter() - _ct0) * 1000.0)
            thr = max(0.0, min(1.0, float(thr))); brk = max(0.0, min(1.0, float(brk)))
            if (S["took_off"] and vx_fwd < -0.3) or S["t_land"] is not None:
                thr, brk, pb = 0.0, 1.0, 1.0
            _act_vehicle_control(veh, args, thr, brk, steer, pb)
            if S["trace_rows"] is not None:
                if S["roll_t0"] is None: S["roll_t0"] = _t_now
                _phase = "approach" if not S["took_off"] else ("air" if S["t_land"] is None else "landed")
                S["trace_rows"].append({
                    "i": i, "t": round(_t_now - S["roll_t0"], 3), "dt": round(dt_step, 4), "phase": _phase,
                    "thr": round(float(thr), 3), "brk": round(float(brk), 3),
                    "steer": round(float(steer), 4), "steer_deg": round(float(steer) * STEER_MAX_DEG, 1),
                    "pb": round(float(pb), 3), "pitch_deg": round(pitch_d, 2), "roll_deg": round(roll_d, 2),
                    "yaw_deg": round(math.degrees(yaw), 2), "pdot_dps": round(pdot, 1),
                    "gspd": round(gspd, 2), "vx_fwd": round(vx_fwd, 2), "vz": round(float(vel[2]), 2),
                    "px": round(px, 2), "py": round(py, 2),
                    "yaw_err_deg": round(math.degrees(math.atan2(float(d[1]), float(d[0]))), 2)
                    if (not S["took_off"]) or (S["t_land"] is None and S["took_off"]) else None,
                    "d_to_lip_m": round(_leg_peak - px, 2) if not S["took_off"] else None,
                    "pz": round(pz, 3), "nc": int(nc), "wheels": wheel_w_all(veh)})
            S["prev_x"] = px
            if S["t_land"] is None:
                n_active += 1
        if ground_launch and not args.__dict__.get("_lip_kick_applied"):
            _kick_dps = float(getattr(args, "lip_roll_rate_kick_dps", 0.0) or 0.0)
            if abs(_kick_dps) > 1e-6 and any(states[l["label"]]["t_takeoff"] == i for l in legs):
                _apply_lip_roll_rate_kick([l["veh"] for l in legs], _kick_dps)
                args.__dict__["_lip_kick_applied"] = True
                _dist_audit = _build_approach_disturbance_audit(args, kick_step=int(i))
                if _dist_audit:
                    args.__dict__["_approach_disturbance_audit"] = _dist_audit
                    for _leg in legs:
                        _ts = states[_leg["label"]].get("takeoff_state")
                        if _ts is not None:
                            _ts["approach_disturbance"] = dict(_dist_audit)
                    print(f"[CS-appr-dist] lip roll-rate kick {_kick_dps}°/s @ step {i} "
                          f"(mode={_dist_audit.get('mode')}, camber={_dist_audit.get('runup_camber_deg')}°, "
                          f"spawn_roll={_dist_audit.get('spawn_roll_deg')}°, all {len(legs)} legs)", flush=True)
        # approach: pause to deterministic stepping as soon as first vehicle takes off; airborne segment fixed 1/sps cadence (approach already ran gameplay-live)
        if (not _paused_air) and any(states[l["label"]]["took_off"] for l in legs):
            try: bng.control.pause()
            except Exception: pass
            _paused_air = True
        _cam = cam_pose
        _cam_phase = "C"
        if ground_launch and cams is not None:
            if any(states[l["label"]]["took_off"] for l in legs):
                _cam = cams["C"]
                _cam_phase = "C"
            else:
                _cam = cams["AB"]
                _cam_phase = "AB"
        if _cam is not None and _cam_update_due(i, args, phase_key=_cam_phase):
            try: bng.camera.set_free(_cam[0], _cam[1])
            except Exception: pass
        rf._step(bng, 1)
        _landed = [S["t_land"] for S in states.values() if S["t_land"] is not None]
        if _landed:
            _land_t0 = min(_landed)
            _landed_labels = [l["label"] for l in legs if states[l["label"]]["t_land"] is not None]
            _all_slow = all(states[lb].get("gspd_mps", 999.0) <= _gspd_gate for lb in _landed_labels)
            _gate_on = _gspd_gate > 0.0
            if _gate_on:
                if i > _land_t0 + _post_land_steps and _all_slow:
                    if not args.__dict__.get("_land_gspd_gate_logged"):
                        _gs = {lb: round(states[lb].get("gspd_mps", 0.0), 2) for lb in _landed_labels}
                        print(f"[CS-land-gspd-gate] exit i={i} all gspd≤{_gspd_gate:g}m/s {_gs}", flush=True)
                        args.__dict__["_land_gspd_gate_logged"] = True
                    break
                if i > _land_t0 + _post_land_max_steps:
                    _gs = {lb: round(states[lb].get("gspd_mps", 0.0), 2) for lb in _landed_labels}
                    print(f"[CS-land-gspd-gate] WARN max post_land cap i={i} gspd={_gs} → force exit",
                          flush=True)
                    break
            else:
                if n_active == 0 and i > max(_landed) + 35:
                    break
                if i > _land_t0 + _post_land_steps:
                    break
        if ground_launch and _simul_fail_i is None:
            _any_to = any(states[l["label"]]["took_off"] for l in legs)
            _lip_miss = 8.0
            _all_miss = 10.0
            if not _any_to:
                if all(states[l["label"]]["prev_x"] > _leg_peak_x(l, peak_x) + _all_miss for l in legs):
                    _simul_fail_i = i
                    print(f"[CS-simul3-appr] all 3 vehicles past crest+{_all_miss:.0f}m no takeoff → fail-fast latch i={i}",
                          flush=True)
            else:
                if any((not states[l["label"]]["took_off"])
                       and states[l["label"]]["prev_x"] > _leg_peak_x(l, peak_x) + _lip_miss
                       for l in legs):
                    _simul_fail_i = i
                    print(f"[CS-simul3-appr] some vehicles past crest+{_lip_miss:.0f}m no takeoff (pair must be invalid) "
                          f"→ fail-fast latch i={i}", flush=True)
        if _simul_fail_i is not None and i >= _simul_fail_i + _postfail_steps:
            print(f"[CS-simul3-appr] fail-fast exit i={i} (+{_postfail_steps} steps)", flush=True)
            break
    results = {}
    for leg in legs:
        label = leg["label"]; dart_on = bool(leg["dart_on"]); S = states[label]
        airtime_steps = round((S["t_land"] - S["t_takeoff"]) * DT, 2) if (S["t_takeoff"] and S["t_land"]) else None
        # SR-airtime: simul3 uses deterministic stepping; physical airtime = airtime_steps (steps×DT).
        # wall clock (wall_land-wall_takeoff) is "real compute seconds to finish this jump" (3-vehicle load ~0.2s/step); renamed wall_runtime, do not use as physical airtime.
        wall_runtime = (round(S["wall_land"] - S["wall_takeoff"], 3)
                        if (S["wall_takeoff"] and S["wall_land"]) else None)
        airtime = airtime_steps
        dt_eff = round(sum(S["dt_samples"]) / len(S["dt_samples"]), 4) if S["dt_samples"] else None
        land_pitch = S["land"][0] if S["land"] else None
        land_pitch_error = round(land_pitch - target_pitch_deg, 1) if land_pitch is not None else None
        if S["trace_rows"] is not None and trace_path:
            try:
                rec = {"tag": getattr(args, "tag", None), "leg": label, "dart_on": dart_on,
                       "baseline_strategy": leg["baseline"], "lane_y": leg["y"], "color": leg.get("color"),
                       "dart_pitch_control": getattr(args, "dart_pitch_control", None), "angle_deg": angle,
                       "v_entry": v_entry, "cross_slope_deg": args.__dict__.get("_current_cross_slope_deg", 0.0),
                       "took_off": S["took_off"], "land_pitch": land_pitch,
                       "land_roll": S["land"][1] if S["land"] else None, "target_pitch_deg": target_pitch_deg,
                       "airtime": airtime, "n_steps": len(S["trace_rows"]), "trace": S["trace_rows"]}
                with open(trace_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                print(f"[CS-trace] simul3 leg={label}(dart={dart_on}) wrote {len(S['trace_rows'])} control-trace steps", flush=True)
            except Exception as e:
                print(f"[CS-trace] WARN simul3 leg={label} trace write failed: {e}", flush=True)
        results[label] = {
            "took_off": S["took_off"], "land_pitch": land_pitch, "land_pitch_error": land_pitch_error,
            "target_pitch_deg": target_pitch_deg, "land_roll": S["land"][1] if S["land"] else None,
            "airtime": airtime, "max_z": round(S["max_z"], 2), "max_roll_air": round(S["max_roll_air"], 1),
            "max_yaw_delta_air": round(S["max_yaw_delta_air"], 1), "land_yaw_delta": S["land_yaw_delta"],
            "max_yaw_delta_air_dv": round(S["max_yaw_delta_air_dv"], 1), "land_yaw_delta_dv": S["land_yaw_delta_dv"],
            "max_land_accel_g": round(S["max_land_accel"] / 9.81, 2) if S["max_land_accel"] else None,
            "ctrl_latency_ms": (
                {"p50": round(sorted(S["ctrl_t_samples"])[len(S["ctrl_t_samples"]) // 2], 4),
                 "p99": round(sorted(S["ctrl_t_samples"])[min(len(S["ctrl_t_samples"]) - 1,
                                                              int(len(S["ctrl_t_samples"]) * 0.99))], 4),
                 "max": round(max(S["ctrl_t_samples"]), 4), "n": len(S["ctrl_t_samples"])}
                if S["ctrl_t_samples"] else None),
            "max_pitchrate": round(S["max_pitchrate"], 0), "tumbled": S["tumbled"],
            "post_land_flip": bool(S.get("post_land_flip")),
            "wheel_w_land": S["wheel_w_land"], "omega_tgt_land": S["omega_tgt_land"],
            "front_clearance": S["front_clear_land"], "damage_land": S["damage_land"],
            "land_x_world": S["land_x_world"], "land_ground_z": S["land_ground_z"],
            "land_on_mesh": S["land_on_mesh"], "land_height_above_ground": S["land_height_above_ground"],
            "apex_clearance_land": S["apex_clearance_land"], "land_vz_mps": S["land_vz_mps"],
            "land_impact_speed_mps": S["land_impact_speed_mps"], "takeoff_state": S["takeoff_state"],
            "T_flight": airtime, "airtime_steps": airtime_steps, "wall_runtime": wall_runtime, "dt_eff": dt_eff,
            "cross_slope_deg": args.__dict__.get("_current_cross_slope_deg", 0.0),
            "runup_camber_deg": args.__dict__.get("_current_runup_camber_deg", _runup_camber_deg(args)),
            "leg": label, "lane_y": leg["y"], "color": leg.get("color"),
            "dart_on": dart_on, "baseline_strategy": leg["baseline"],
            "takeoff_state_jitter": args.__dict__.get("_takeoff_jitter_audit"),
            "approach_disturbance": args.__dict__.get("_approach_disturbance_audit"),
            "actuator_latency_ms": round(float(getattr(args, "actuator_latency_ms", 0.0) or 0.0), 3),
            "gate_intervened_steps": int(S.get("gate_intervened_steps", 0)),
            "approach_spawn_pos": S.get("approach_spawn_pos"),
            "_provenance": make_provenance("deterministic_stepped",
                                           sps=int(getattr(args, "sim_steps_per_second", 100) or 100),
                                           dt_eff=dt_eff)}
    return results

def med(vals):
    v = sorted(x for x in vals if x is not None)
    return v[len(v) // 2] if v else None

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--angles", type=int, nargs="*", default=ANGLES)
    p.add_argument("--rolls", type=int, default=3)
    p.add_argument("--paired-rolls", type=int, default=0,
                   help="1=run OFF then ON per pair, writing pair_id/launch_request; reduces false positives from OFF/ON execution-order drift")
    p.add_argument("--paired-valid-target", type=int, default=0,
                   help=">0: under paired-rolls, only pairs with pair_match=true count toward OFF/ON stats; invalid pairs go to invalid_pairs and collection continues until N_valid")
    p.add_argument("--paired-max-attempts", type=int, default=0,
                   help="Max pair attempts when paired-valid-target>0; 0=use --rolls")
    p.add_argument("--pair-v0-tol", type=float, default=0.5,
                   help="paired-rolls post-audit: OFF/ON takeoff v0 tolerance (m/s); exceed -> pair_match=false")
    p.add_argument("--pair-wy0-tol", type=float, default=15.0,
                   help="paired-rolls post-audit: OFF/ON takeoff omega_y0 tolerance (deg/s)")
    p.add_argument("--pair-theta0-tol", type=float, default=1.0,
                   help="paired-rolls post-audit: OFF/ON takeoff theta0 tolerance (deg)")
    p.add_argument("--pair-vz0-tol", type=float, default=0.5,
                   help="paired-rolls post-audit: OFF/ON takeoff vz0 tolerance (m/s)")
    p.add_argument("--pair-roll0-tol", type=float, default=2.0,
                   help="paired-rolls post-audit: OFF/ON takeoff roll0 tolerance (deg)")
    p.add_argument("--pair-yaw0-tol", type=float, default=5.0,
                   help="paired-rolls post-audit: OFF/ON takeoff yaw0 tolerance (deg)")
    p.add_argument("--pair-y0-tol", type=float, default=0.5,
                   help="paired-rolls post-audit: OFF/ON takeoff lateral position y0 tolerance (m)")
    p.add_argument("--pair-min-t-flight", type=float, default=0.0,
                   help="paired-rolls post-audit: >0 requires both OFF/ON T_flight >= this value for pair_match")
    p.add_argument("--pair-min-apex-clearance", type=float, default=0.0,
                   help="paired-rolls post-audit: >0 requires both OFF/ON apex_clearance_land >= this value")
    p.add_argument("--pair-max-apex-clearance", type=float, default=0.0,
                   help="paired-rolls post-audit: >0 requires both OFF/ON apex_clearance_land <= this value")
    p.add_argument("--pair-require-landing-mesh", type=int, default=0,
                   help="paired-rolls post-audit: 1=both OFF/ON must have land_on_mesh=True")
    p.add_argument("--pair-randomize-order", type=int, default=1,
                   help="1=randomize OFF/ON order per pair (eliminates 'second run lands worse' systematic bias); 0=legacy fixed OFF->ON")
    p.add_argument("--pair-order-seed", type=int, default=20260617,
                   help="Pair order randomization seed (reproducible); per ramp derived as seed+ramp_idx*1009")
    p.add_argument("--pair-bias-floor", type=int, default=0,
                   help="1=bias-floor calibration probe: ON leg also runs dart_off replicate (twin), quantifying protocol-intrinsic noise floor (should be ~0); calibrates non-DART effects")
    p.add_argument("--rise", type=float, default=3.0, help="Ramp lip height H (m)")
    p.add_argument("--ramp-mode", choices=["tabletop", "kicker"], default="tabletop",
                   help="tabletop=original testbed (convex crest abrupt lip termination, uncontrolled omega_y0); kicker=qualified geometry (straight ramp + controlled lip radius)")
    p.add_argument("--lip-radius", type=float, default=0.0,
                   help="kicker lip convex arc radius R_lip (m); >0 use directly, ==0 back-compute from --lip-omega-target-dps")
    p.add_argument("--lip-omega-target-dps", type=float, default=50.0,
                   help="kicker target takeoff nose-down rate omega_y0 (deg/s, positive takes absolute value); R_lip=v_peak/|omega_target|")
    p.add_argument("--lip-sweep-deg", type=float, default=8.0,
                   help="kicker lip arc sweep angle (deg); positive=convex (reduces exit angle, nose-down theta0~alpha-sweep); negative=concave nose-up (increases exit angle, nose-up theta0~alpha+|sweep|)")
    p.add_argument("--lip-throttle-cut-m", type=float, default=0.0,
                   help="Takeoff control: cut throttle N meters before lip for coast settle (reduces omega_y0 nose-whip); 0=no cut (full throttle to lip)")
    p.add_argument("--lip-power", type=int, default=0,
                   help="1=powered over lip (full throttle only --lip-power-m meters before crest, otherwise maintain v_entry); simulates driver powered takeoff (drive+squat->nose-up, countering ramp nose-down). Applied equally to OFF/DART legs")
    p.add_argument("--lip-power-m", type=float, default=8.0,
                   help="--lip-power powered zone: full throttle N meters before crest (only this segment, prevents whole run-up overspeed)")
    p.add_argument("--baseline-strategy", choices=["off", "human"], default="off",
                   help="Baseline (OFF control) air strategy: off=throttle0 passive (worst driver); human=in-air pitch trim (nose-low add throttle/nose-high brake, simulates real driver). Combine with --lip-power for full driver strategy")
    p.add_argument("--air-trim-kp", type=float, default=0.6, help="human baseline in-air pitch trim P gain")
    p.add_argument("--air-trim-kd", type=float, default=0.3, help="human baseline in-air pitch trim D gain")
    p.add_argument("--simul-3way", type=int, default=0,
                   help=">0: launch three vehicles side-by-side on the same tick on the testbed (DART / human / passive), identical launch conditions except lane Y offset. One jump yields all three conditions simultaneously, eliminating post-takeoff state matching. air-impulse + dart-pitch-control continuous only. Use --paired-valid-target for N, up to --paired-max-attempts attempts")
    p.add_argument("--simul-lane-gap", type=float, default=9.0,
                   help="simul-3way y_lane lateral spacing (m): first strategy at -((N-1)/2)*gap, others spaced sequentially")
    p.add_argument("--simul-layout", choices=["y_lane", "y_copy", "x_copy"], default="y_lane",
                   help="simul-3way layout: y_copy=three full ramp copies along Y (each vehicle gets run-up/ramp/landing, Ct/Ctp); y_lane=multiple lanes on same ramp along Y; x_copy=three copies chained along X (each vehicle y=0, base_x offset)")
    p.add_argument("--simul-copy-spacing", type=float, default=180.0,
                   help="Copy spacing (m): y_copy=adjacent ramp center Y spacing (recommend >=runup_apron_width); x_copy=adjacent ramp base_x spacing (must exceed single-jump landing footprint)")
    p.add_argument("--simul-takeoff-spread-gate", type=int, default=0,
                   help=">0: approach simul3 only ACCEPT jumps with cross-leg takeoff spread <= threshold (Ct smoke)")
    p.add_argument("--simul-max-v0-spread", type=float, default=0.15,
                   help="simul spread gate: v0_mps max-min upper bound (m/s)")
    p.add_argument("--simul-max-theta0-spread", type=float, default=0.5,
                   help="simul spread gate: theta0_deg max-min upper bound (deg)")
    p.add_argument("--simul-max-roll0-spread", type=float, default=1.0,
                   help="simul spread gate: roll0_deg max-min upper bound (deg)")
    p.add_argument("--simul-strategies", type=str, default="dart,human,passive",
                   help="simul-3way lane control strategies (comma-separated); paper head-to-head baseline uses 'dart,rwpd,tobb'. Options: dart (DART rate-tracking differential) / rwpd (RW-PD reaction-wheel PD) / tobb (TOBB time-optimal bang-bang, analytical minimum-time switching, not online optimization) / dart_latched (per-flight roll latch) / dart_replicate (same-law replicate) / dart_dual/dart_pitch_only/dart_roll_only (per-axis ablations) / human (legacy air-trim) / passive (coast). Legacy aliases dart_off/pd/mpc etc. also accepted (see LEGACY_STRATEGY_ALIASES)")
    p.add_argument("--rwpd-kp-pitch", type=float, default=0.6, help="RW-PD baseline pitch P gain")
    p.add_argument("--rwpd-kd-pitch", type=float, default=0.3, help="RW-PD baseline pitch D gain")
    p.add_argument("--rwpd-k-roll", type=float, default=1.0, help="RW-PD baseline roll counter-steer gain")
    p.add_argument("--rwpd-smax", type=float, default=0.3, help="RW-PD baseline steering clamp")
    p.add_argument("--tobb-a-max-dps2", type=float, default=120.0,
                   help="TOBB baseline pitch angular acceleration limit (deg/s^2, bang-bang switching curve; reflects 4-motor reaction authority)")
    p.add_argument("--tobb-k-roll", type=float, default=1.0, help="TOBB baseline roll counter-steer gain")
    p.add_argument("--tobb-smax", type=float, default=0.3, help="TOBB baseline steering clamp")
    p.add_argument("--reachability-gate", type=int, default=0,
                   help=">0: enable pre-takeoff go/no-go reachability gate (infeasible entry speed -> brake to v_safe / empty feasible set -> abort). ON/OFF each run one round to compare success/rollover (pillar B closed-loop). Default 0 (honest baseline)")
    p.add_argument("--dart-airtime-guardrail", type=int, default=0,
                   help=">=1: per jump evaluate DART airtime leverage guardrail from measured (airtime, theta0) (N=202 within-angle calibration): favorability=B_air*z(airtime)+B_theta*z(theta0), <0=trading steep angle for airtime where theta0 penalty cancels benefit. Writes report.dart_airtime_guardrail + verdict line. Default 0 (off, bit-exact backward compatible)")
    p.add_argument("--dart-airtime-guardrail-margin", type=float, default=0.0,
                   help="Guardrail favorability safety margin: must be >= this value for FAVORABLE (>0=more conservative). Default 0")
    p.add_argument("--gate-v-crit", type=float, default=8.0, help="Gate: takeoff-section critical speed v_crit (m/s)")
    p.add_argument("--gate-v-min-clearance", type=float, default=0.0,
                   help="Gate: minimum entry speed v_min (m/s) to clear gap/land on landing slope. >0 and v_max<v_min -> empty speed window -> ABORT_JUMP. Default 0 (off, upper-bound gate only). Demo for P1-G ABORT branch (real go/no-go lower bound)")
    p.add_argument("--gate-a-brake", type=float, default=4.0, help="Gate: approach available deceleration a_brake (m/s^2)")
    p.add_argument("--gate-coast-m", type=float, default=0.0,
                   help="Gate: terminal coast zone length (m). >0: gate brakes to v_crit within coast_m before lip, then zero throttle/brake coast for suspension/pitch recovery, fixing 'hard brake at lip -> weight transfer -> nose-dive' coupling. 0=continuous brake to lip (legacy)")
    p.add_argument("--gate-adaptive-abrake", type=int, default=0,
                   help="Closed-loop v0_launch: 1=online EMA of measured deceleration during braking adapts a_brake in v_max(d) envelope -> consistent brake to v_crit per jump, offsetting assumed vs real a_brake mismatch propagating coast entry speed -> v0_launch -> lip omega_y0 variance. 0=fixed assumed (legacy)")
    p.add_argument("--gate-lip-power-recover", type=int, default=0,
                   help="Gate lever 1: 1=terminal coast_m zone uses throttle hold over lip instead of pure coasting -- climb segment holds v_crit (back off if over) -> drive squat restores nose-up torque correcting omega_y0 pitch-over, without overspeed. Requires --gate-coast-m>0")
    p.add_argument("--gate-a-pitch-dps2", type=float, default=120.0,
                   help="Gate: pitch angular acceleration limit (deg/s^2, Eq. 6.4 reachable set; same order as TOBB a_max)")
    p.add_argument("--gate-flight-time-s", type=float, default=0.95,
                   help="Gate: flight duration T (s, post-fix measured ~0.95)")
    p.add_argument("--gate-speed-margin", type=float, default=0.0,
                   help="Gate: extra safety margin on v_max (m/s, increase for conservative mode)")
    p.add_argument("--gate-lipmap", type=str, default="",
                   help="Gate: path to a G1-passed LipMap fit JSON. When set, takeoff "
                        "(theta0, omega_y0, T) comes from LipMap(v) instead of ramp-angle/zero-rate.")
    p.add_argument("--gate-wheel-budget", type=int, default=0,
                   help="Gate: 1=use Theorem 1 directional budgets from wheel speed v/r. 0=unlimited.")
    p.add_argument("--gate-wheel-iw", type=float, default=1.2,
                   help="Gate directional budget: per-wheel spin inertia I_w (kg m^2)")
    p.add_argument("--gate-wheel-omega-max-radps", type=float, default=125.7,
                   help="Gate directional budget: wheel-speed upper envelope (rad/s)")
    p.add_argument("--gate-wheel-omega-min-radps", type=float, default=-125.7,
                   help="Gate directional budget: wheel-speed lower envelope (rad/s)")
    p.add_argument("--gate-j-y", type=float, default=2043.0,
                   help="Gate directional budget: body pitch inertia I_yy (kg m^2)")
    p.add_argument("--gate-vcert", type=int, default=0,
                   help="Gate: 1=scan the certified speed set (requires --gate-lipmap). Empty window -> abort.")
    p.add_argument("--gate-climb-loss-mps", type=float, default=0.0,
                   help="Gate: climb speed loss (m/s) subtracted from the shaped takeoff query.")
    p.add_argument("--gate-omega-bar-dps", type=float, default=0.0,
                   help="Gate: terminal rate-window half-width (deg/s). 0=exact-pair Theorem 3.")
    p.add_argument("--cond-jitter", type=int, default=0,
                   help=">0: per ramp angle run N jumps, each sampling operational envelope (v_entry/takeoff pitch/roll perturbation), building real variance population (not repeated clones). Same seed OFF/ON -> pair by (theta, jump_idx). Default 0 (off)")
    p.add_argument("--cond-jitter-seed", type=int, default=20260618,
                   help="Condition sampling seed (reproducible; OFF/ON same seed -> same condition sequence -> pairable)")
    p.add_argument("--cond-jitter-v-lo", type=float, default=13.0, help="Condition jitter: v_entry lower bound (m/s)")
    p.add_argument("--cond-jitter-v-hi", type=float, default=26.0, help="Condition jitter: v_entry upper bound (m/s)")
    p.add_argument("--cond-jitter-pitch-lo", type=float, default=6.0, help="Condition jitter: takeoff pitch perturbation lower bound (deg)")
    p.add_argument("--cond-jitter-pitch-hi", type=float, default=18.0, help="Condition jitter: takeoff pitch perturbation upper bound (deg)")
    p.add_argument("--cond-jitter-roll-lo", type=float, default=6.0, help="Condition jitter: takeoff roll perturbation lower bound (deg)")
    p.add_argument("--cond-jitter-roll-hi", type=float, default=18.0, help="Condition jitter: takeoff roll perturbation upper bound (deg)")
    p.add_argument("--entry-fillet-len", type=float, default=5.0,
                   help="kicker entry concave fillet minimum horizontal length (m, smooth ground contact); floor for span in adaptive mode")
    p.add_argument("--entry-fillet-radius", type=float, default=22.0,
                   help="kicker entry fillet minimum curvature radius (m): adaptive entry span = max(entry_fillet_len, R*sin(alpha)), ensuring R_in>=R (steeper ramps get longer feathering, limiting entry centripetal accel ~v^2/R, preventing nose/chassis ground strike); 0=disable adaptive (fixed len)")
    p.add_argument("--n-entry", type=int, default=12, help="kicker entry fillet segment count")
    p.add_argument("--n-straight", type=int, default=6, help="kicker straight-ramp segment count")
    p.add_argument("--n-lip", type=int, default=8, help="kicker lip convex arc segment count")
    p.add_argument("--fillet-len-max", type=float, default=15.0)
    p.add_argument("--fillet-rise-budget", type=float, default=3.0)
    p.add_argument("--width", type=float, default=36.0)
    p.add_argument("--thick", type=float, default=0.8)
    p.add_argument("--n-fillet", type=int, default=14)
    p.add_argument("--n-main", type=int, default=3)
    p.add_argument("--overlap", type=float, default=1.10)
    p.add_argument("--sink", type=float, default=0.05)
    p.add_argument("--base-x", type=float, default=50.0)
    p.add_argument("--run-up", type=float, default=30.0)
    p.add_argument("--runup-ground-type", choices=["ASPHALT", "GRAVEL", "DIRT"], default="ASPHALT",
                   help="Run-up flat + takeoff ramp ground model: ASPHALT=track_editor_C_border (default); GRAVEL/DIRT=register dart_runup_* materials and pave run-up pad (full approach chain)")
    p.add_argument("--runup-pad-margin-back", type=float, default=11.0,
                   help="Unified run-up pad extension behind spawn (m); spawn_x=base_x-run_up, default 11 (=original 3+8)")
    p.add_argument("--runup-pad-margin-front", type=float, default=1.5,
                   help="Run-up pad forward overlap past ramp base_x (m), fills gap")
    p.add_argument("--v-entry", type=float, default=0.0,
                   help="Designed takeoff speed (m/s, ~actual liftoff speed); >0 overrides v_base model, decouples speed from rise (prevents downhill divergence)")
    p.add_argument("--spawn-v", type=float, default=8.0,
                   help="Per-roll startup initial speed cap (m/s, set_velocity); improves reliable momentum over lip (prevents pure coast when drivetrain drops out)")
    p.add_argument("--launch-mode", choices=["approach", "lip-impulse", "air-impulse"], default="approach",
                   help="approach=real run-up; lip-impulse=short-distance launch before lip; air-impulse=launch above lip in air, decouples EV auto-box to validate air segment")
    p.add_argument("--lip-launch-m", type=float, default=4.0,
                   help="launch-mode=lip-impulse: launch from N meters before lip")
    p.add_argument("--lip-launch-z-offset", type=float, default=0.65,
                   help="launch-mode=lip-impulse: vehicle z clearance above ramp surface (m), avoids initial penetration")
    p.add_argument("--lip-impulse-dt", type=float, default=0.2,
                   help="launch-mode=lip-impulse: set_velocity dt; small dt for short-distance speed hold")
    p.add_argument("--lip-impulse-hold-steps", type=int, default=3,
                   help="launch-mode=lip-impulse: simulation steps after each set_velocity")
    p.add_argument("--air-impulse-pitch-deg", type=float, default=0.0,
                   help="launch-mode=air-impulse: initial pitch angle (deg), controllable toss attitude for air testbed")
    p.add_argument("--air-impulse-roll-deg", type=float, default=0.0,
                   help="launch-mode=air-impulse: initial roll angle (deg), cross-slope/bank disturbance testbed, tests DART roll correction authority (bench a)")
    p.add_argument("--air-impulse-pitch-rate-dps", type=float, default=0.0,
                   help="air-impulse (simul-3way): after launch settle, add a body-pitch angular-velocity kick on the same tick (deg/s, world y axis; calibrate sign-to-nose-up mapping from smoke takeoff_state.omega_y0_dps). Used by the budget-interior entry-rate cell; 0=off")
    p.add_argument("--air-launch-v0-target", type=float, default=0.0,
                   help="v0 alignment gate: >0 air-impulse launch iteratively set_velocity to converge measured gspd (set_velocity force varies with mass, mass ladder required; recommend 8.5=m100 measured baseline); 0=OFF (legacy protocol compatible)")
    p.add_argument("--air-launch-v0-tol", type=float, default=0.3,
                   help="v0 alignment gate convergence tolerance (m/s)")
    p.add_argument("--air-launch-v0-max-tries", type=int, default=8,
                   help="v0 alignment gate max iterations (each 2 steps ~0.02s air altitude loss)")
    p.add_argument("--takeoff-state-jitter-sigma", type=float, default=0.0,
                   help="Takeoff state Gaussian perturbation sigma (deg for pitch/roll, same sigma for v_entry m/s); per jump adds N(0,sigma) to CLI baseline; sigma=0 bit-exact compatible")
    p.add_argument("--takeoff-state-jitter-seed", type=int, default=20260628,
                   help="Takeoff state jitter RNG base seed; actual seed=base+ramp_idx*1000+jump_idx")
    p.add_argument("--actuator-latency-ms", type=float, default=0.0,
                   help="Control command latency (ms); converted to FIFO step delay via sim_steps_per_second (includes diff per-round factor)")
    p.add_argument("--control-trace", type=int, default=0,
                   help="1=step-wise log steering/throttle/brake/4WIDS wheel speeds/attitude control trace to JSONL (one line per roll)")
    p.add_argument("--control-trace-path", default=None,
                   help="Control trace JSONL output path (default data/cohorts/ctrl_trace_<tag>.jsonl)")
    p.add_argument("--respawn-per-roll", type=int, default=0,
                   help="0=teleport reset only (default, verified reliable); 1=per-roll despawn+spawn fresh ego -- **Win32 tested: despawn/spawn crashes BeamNG.tech (first roll 'not running any more') -> all subsequent failures, do not use**")
    p.add_argument("--launch-retries", type=int, default=4,
                   help="Startup initial-speed injection (set_velocity) verification retries; fixes fresh-spawn occasional failure to hold speed -> vehicle starts ~0, cannot climb, took_off=False")
    p.add_argument("--launch-min-frac", type=float, default=0.6,
                   help="Startup injection verification threshold: measured gspd>=frac*target speed counts as held, else creep re-activate D and re-inject")
    p.add_argument("--launch-creep-steps", type=int, default=0,
                   help="Startup readiness gate (default 0=disabled, simple launch). >0: full throttle creep from stationary up to N steps to leave R/N -- warning: full throttle from stationary can trigger box jump to R and reverse yaw drift, use cautiously")
    p.add_argument("--launch-creep-vmin", type=float, default=3.0,
                   help="Creep readiness threshold: forward speed reaches this (m/s) -> box in D, stop creep and inject initial speed")
    p.add_argument("--launch-creep-min-disp", type=float, default=0.0,
                   help=">0: box warmup gate uses true displacement (m) instead of vfwd, prevents wheelspin false ready (EV cold-start wheel slip); 0=vfwd threshold")
    p.add_argument("--teleport-reset", type=int, default=1,
                   help="1=teleport reset=True (default); 0=reset=False keeps drivable box state (fixes EV teleport cold-start first roll not entering D)")
    p.add_argument("--no-launch-teleport", type=int, default=0,
                   help="1=one_jump without teleport, vehicle stays at scenario (loaded) spawn state (EV auto-box enters D, throttle drivable). approach real run-up only; requires --approach-fresh-spawn to reload scenario and respawn each jump")
    p.add_argument("--approach-fresh-spawn", type=int, default=0,
                   help="1=approach real run-up mode: each jump scenario.load+start reloads scenario (powertrain reset to D)+place_ramp rebuilds ramp +one_jump no teleport. N set by --rolls, DART leg by --fresh-spawn-dart. Avoids teleport gear-lock root cause")
    p.add_argument("--fresh-spawn-dart", type=int, default=1,
                   help="Under --approach-fresh-spawn which leg to run: 1=dart ON (default), 0=dart OFF (baseline). Gate OFF/ON set by --reachability-gate")
    p.add_argument("--postland-hold-sec", type=float, default=3.5,
                   help="HUD mode (one_jump) post-landing hold seconds (wall clock); hud=0 uses constant 0.3s fast teardown")
    p.add_argument("--postfail-hold-sec", type=float, default=3.5,
                   help="HUD mode early-fail hold seconds; hud=0 uses constant 0.2s (one_jump wall clock / simul3 sim steps) fast teardown")
    p.add_argument("--cam-c-span-m", type=float, default=32.0,
                   help="C landing camera framing span cap (m): larger pulls camera farther/higher, shows longer post-landing slide-out (vehicle smaller). approach real run-up for ~100m slide-out use ~110; air-impulse testbed close-up uses default 32")
    p.add_argument("--cam-approach-post-lip-m", type=float, default=2.0,
                   help="approach combined AB camera: horizontal frame right bound = peak_x + this value (m), must show run-up and meters past lip")
    p.add_argument("--cam-approach-pre-start-m", type=float, default=2.0,
                   help="approach combined AB camera: horizontal frame left bound = start_x - this value (m); smaller zooms in (default 2, old ~5)")
    p.add_argument("--cam-approach-z-drop-m", type=float, default=8.0,
                   help="approach combined AB camera: extra height drop relative to C-family formula (m), slightly lower side view (default 8)")
    p.add_argument("--cam-air-impulse-z-drop-m", type=float, default=10.0,
                   help="launch-mode=air-impulse C camera extra height drop (m); approach post-takeoff C unaffected (default 10)")
    p.add_argument("--pc", type=str, default="vehicles/sbr/dart_4motor.pc",
                   help="Vehicle part_config path (platform parameter robustness sweep: switch to dart_4motor_tq*/m*/i* variants). Default = nominal 4-motor EV. Variants must use unique part names (avoid jbeam same-name cache)")
    p.add_argument("--approach-kp-y", type=float, default=0.0,
                   help="Run-up heading lock lateral offset gain (internal -kp_y*py, multiplied by approach-steer-sign before dispatch); default 0")
    p.add_argument("--approach-kp-yaw", type=float, default=0.0,
                   help="Run-up heading lock yaw error gain (internal -kp_yaw*yaw_err, multiplied by approach-steer-sign); default 0")
    p.add_argument("--approach-ki-y", type=float, default=0.0,
                   help="Run-up lane-keep lateral integral gain (internal -ki_y*integral(err), camber auto default 0.010+0.0012*gamma); default 0=auto")
    p.add_argument("--approach-steer-sign", type=float, default=-1.0,
                   help="Run-up steer dispatch sign: -1=dart EV (opposite natural_jump steer>0=left convention, default); +1=SBR convention")
    p.add_argument("--approach-steer-clamp", type=float, default=0.12,
                   help="Run-up heading lock steer limit (rad-ish, independent of smax): small value prevents over-steer oscillation; fixes EV uphill drift")
    p.add_argument("--prelip-steer-amp", type=float, default=0.0,
                   help="[DEPRECATED] Ground-contact steer preset in a window before the lip; 0=disabled. The ground-contact takeoff route is infeasible on the current vehicle model; flag kept for reuse on other platforms")
    p.add_argument("--prelip-steer-start-m", type=float, default=5.0,
                   help="prelip-steer: start attitude preset when distance to lip < this")
    p.add_argument("--prelip-steer-end-m", type=float, default=3.0,
                   help="prelip-steer: end attitude preset when distance to lip < this, leave room for traction/takeoff")
    p.add_argument("--prelip-traction-throttle", type=float, default=-1.0,
                   help="Ground-contact phase scaffold: pre-lip traction throttle; <0=disabled, 0~1=override throttle")
    p.add_argument("--prelip-traction-m", type=float, default=3.0,
                   help="prelip traction: apply traction throttle when distance to lip < this")
    p.add_argument("--prelip-yaw-priority-m", type=float, default=30.0,
                   help="Within this distance before lip enable yaw-first (reduce py weight); v5.4 default 30m")
    p.add_argument("--prelip-yaw-k-boost", type=float, default=4.0,
                   help="prelip yaw priority zone kp_yaw multiplier")
    p.add_argument("--prelip-yaw-min-deg", type=float, default=3.0,
                   help="prelip yaw priority: activate when |yaw_err| >= this angle (deg)")
    p.add_argument("--prelip-yaw-lock-m", type=float, default=10.0,
                   help="lip yaw0 specialty: distance to lip <= this and |yaw|>=lock-min -> pure yaw steer (*lock-boost)")
    p.add_argument("--prelip-yaw-lock-boost", type=float, default=6.0,
                   help="yaw-lock zone kp_yaw multiplier")
    p.add_argument("--prelip-yaw-lock-min-deg", type=float, default=2.0,
                   help="yaw-lock zone |yaw_err| activation threshold (deg)")
    p.add_argument("--prelip-yaw-lock-throttle-deg", type=float, default=5.0,
                   help="yaw-lock zone |yaw|>=this -> cut throttle for steer grip (v5.4)")
    p.add_argument("--approach-lip-v-cap-mps", type=float, default=0.0,
                   help="camber lip target max speed (m/s); 0=auto (gamma>=6 -> min(v_entry,11))")
    p.add_argument("--approach-lip-stability-coast-m", type=float, default=3.0,
                   help="Pre-lip light brake zone (m); brake only when gspd>cap")
    p.add_argument("--approach-lip-stability-crest-m", type=float, default=2.5,
                   help="Pre-lip crest partial throttle zone (m); maintain takeoff speed when gspd<cap")
    p.add_argument("--approach-lip-stability-crest-throttle", type=float, default=0.50,
                   help="Crest zone throttle cap (0~1); prevents full throttle overspeed causing landing rollover")
    p.add_argument("--approach-lip-stability-v-floor-mps", type=float, default=10.0,
                   help="In brake zone gspd below this -> add throttle to climb kicker")
    p.add_argument("--approach-lip-stability-brk-gain", type=float, default=0.28,
                   help="Pre-lip stability zone light brake gain brk=min(max, gain*(gspd-cap)) when over cap")
    p.add_argument("--approach-lip-stability-brk-max", type=float, default=0.45,
                   help="Pre-lip stability zone light brake cap")
    p.add_argument("--lip-yaw0-warn-deg", type=float, default=8.0,
                   help="Log [CS-lip-yaw0] WARN when takeoff |yaw0| exceeds this")
    p.add_argument("--gear-debug", type=int, default=0,
                   help="1=print gear at each roll teleport/settle/creep stage (diagnose EV auto-box R engagement timing)")
    p.add_argument("--launch-creep-throttle", type=float, default=1.0,
                   help="Warmup segment throttle (default 1.0=floor). After settle box in D, full throttle from stationary instantly jumps to R reverse; reduce (e.g. 0.3) to keep box in D and avoid R (root-cause fix)")
    p.add_argument("--launch-nudge-v", type=float, default=0.0,
                   help=">0: before warmup use set_velocity forward nudge to this speed (m/s, e.g. 4), keep box in D avoiding 'full throttle from stationary jumps R reverse yaw drift' (root-cause fix)")
    p.add_argument("--launch-nudge-steps", type=int, default=4,
                   help="Forward nudge steps (each step set_velocity+step1)")
    p.add_argument("--landing-slope-deg", type=float, default=0.0,
                   help="Matched downhill landing slope angle / target landing pitch (deg), 0=flat; negative=downhill along +X")
    p.add_argument("--landing-slope-mode", choices=["fixed", "auto-to-flight", "ballistic", "gap", "gap-ramp", "valley"], default="fixed",
                   help="fixed=use landing_slope_deg; auto-to-flight=back-solve back-slope angle; ballistic=ballistic-matched surface (hugs trajectory, no air gap); gap=steep valley + tangential catch (true free flight, lands near flat valley floor); gap-ramp=air-gap free flight + inclined beta catch ramp (deprecated, hits ramp crest); valley=continuous smooth descent from lip to z<0 valley floor (beta=|landing_slope_deg|, vehicle free-flies onto slope, tests DART pitch authority)")
    p.add_argument("--valley-floor-depth", type=float, default=15.0,
                   help="valley mode: maximum valley floor depth below ground plane (m), limits ramp length")
    p.add_argument("--valley-feather-len", type=float, default=6.0,
                   help="valley mode: horizontal feather at downhill-to-floor junction (m), tangent smooths from -beta to 0 deg, removes hard kink/prevents drop")
    p.add_argument("--valley-floor-run", type=float, default=45.0,
                   help="valley mode: horizontal valley floor segment after feather (m), smooth post-landing slide-out (default 45m)")
    p.add_argument("--valley-crest-len", type=float, default=6.0,
                   help="valley mode: crest convex arc horizontal span (m), tangent smooths from lip exit angle to -beta, removes hard crest ridge")
    p.add_argument("--valley-auto-rise", type=int, default=1,
                   help="valley mode: 1=steep beta auto-raises kicker lip (peak_z) for vertical room on -beta straight slope, landing on full -beta slope not feather arc; 0=no raise")
    p.add_argument("--valley-rise-margin", type=float, default=1.0,
                   help="valley auto-rise margin (m): required lip height + this margin, prevents landing exactly on feather start")
    p.add_argument("--valley-adaptive-ventry", type=int, default=0,
                   help="1=valley mode back-solves v_entry for flight time ~ --target-airtime (equal-airtime, decouples steepness from airtime); 0=fixed --v-entry")
    p.add_argument("--target-airtime", type=float, default=0.9,
                   help="--valley-adaptive-ventry=1 target flight time (s); back-solve v0 per angle so T_flight ~ this value")
    p.add_argument("--ventry-burst-offset", type=float, default=2.0,
                   help="adaptive: lip crest full-throttle burst makes actual liftoff v0 ~ v_entry+this offset; v_entry=v0_design-offset so measured airtime hits target")
    p.add_argument("--ventry-min", type=float, default=8.0, help="adaptive v_entry lower bound (m/s)")
    p.add_argument("--ventry-max", type=float, default=30.0, help="adaptive v_entry upper bound (m/s)")
    p.add_argument("--landing-gap-max", type=float, default=2.5,
                   help="gap mode mid-flight maximum air gap (m, vehicle height above surface); larger flies higher")
    p.add_argument("--landing-gap-shape-p", type=float, default=1.5,
                   help="gap mode clearance shape exponent; small->steep apex harder landing, large->gentle apex softer tangential landing (1.5 balanced in practice)")
    p.add_argument("--landing-feather-len", type=float, default=6.5,
                   help="gap mode landing slope lower edge ground feather horizontal span (m), removes hard kink")
    p.add_argument("--ballistic-v0", type=float, default=0.0,
                   help="ballistic mode takeoff speed (m/s); 0=estimate from v_entry/v_base")
    p.add_argument("--ballistic-gamma-deg", type=float, default=0.0,
                   help="ballistic mode takeoff trajectory angle (deg, velocity direction); 0=use lip exit angle (kicker=alpha-sweep)")
    p.add_argument("--ballistic-clearance", type=float, default=0.5,
                   help="ballistic landing surface offset below CG parabola (m, ~CG to wheel contact vertical distance)")
    p.add_argument("--dune-face-deg", type=float, default=33.0,
                   help="Dune slip-face angle of repose limit (deg); ballistic surface steepens to this angle then concave runout smooths to flat")
    p.add_argument("--landing-gap-run", type=float, default=10.0,
                   help="gap-ramp mode: horizontal air-gap length lip->catch ramp crest (m)=free-flight distance, sets airtime (=gap_run/vx). beta (slope) decoupled -> fixed gap_run sweep beta tests pure slope generalization, sweep gap_run tests length/airtime generalization")
    p.add_argument("--landing-slope-len", type=float, default=45.0,
                   help="Downhill landing zone length (m), only when landing_slope_deg!=0")
    p.add_argument("--landing-mesh-end-x", type=float, default=0.0,
                   help="Landing mesh truncated at world x (m, >0 active): per-angle landing slope length truncated by (this value - peak_x), mesh ends here; also sets far_x=this value -> C camera frame [peak, this value] tighter and shows mesh end. 0=no truncation (use --landing-slope-len)")
    p.add_argument("--landing-cross-slope-deg", type=float, default=0.0,
                   help="Landing zone cross-slope/bank angle (deg): entire landing testbed banks about forward ground line, vehicle gains lateral roll moment on contact. 0=flat; positive=right side low, negative=left side low. For cross-slope landing three-way comparison (DART/driver/OFF)")
    p.add_argument("--runup-camber-deg", type=float, default=0.0,
                   help="Run-up + takeoff ramp cross-slope (deg): entire run-up apron + takeoff mesh banks about low side; positive=right side low; 0=flat. Scenario library runup_camber_deg can be overridden by CLI")
    p.add_argument("--camber-settle-steps", type=int, default=40,
                   help="camber: extra settle steps after teleport onto banked apron (suspension compresses, stable 4-wheel contact before drive)")
    p.add_argument("--runup-apron-width", type=float, default=40.0,
                   help="camber: wide apron local ground width (m), decoupled from narrow ramp width; pivot_y=+/-W/2")
    p.add_argument("--runup-apron-margin-back", type=float, default=13.0,
                   help="camber: apron extension behind spawn (m); default 13 (original 5 + 8m rear extension)")
    p.add_argument("--runup-apron-margin-front", type=float, default=3.0,
                   help="camber: apron front overlap past base_x with kicker (m)")
    p.add_argument("--runup-joint-fill-len", type=float, default=8.0,
                   help="Apron front seam fill plate depth into ramp body (m), seals apron/ramp side-view cavity; 0=off")
    p.add_argument("--camber-nudge-v", type=float, default=2.5,
                   help="camber warmup: forward set_velocity nudge (m/s), 0=use launch-nudge-v")
    p.add_argument("--camber-nudge-steps", type=int, default=10,
                   help="camber warmup: settle steps after nudge")
    p.add_argument("--camber-creep-steps", type=int, default=60,
                   help="camber warmup: max creep steps before run-up (true displacement confirmation)")
    p.add_argument("--camber-creep-throttle", type=float, default=0.65,
                   help="camber warmup: creep throttle (gentle, prevents box jump to R)")
    p.add_argument("--camber-creep-min-disp", type=float, default=1.5,
                   help="camber warmup: creep target displacement (m), must reach before approach main loop")
    p.add_argument("--camber-snap-py-tol", type=float, default=0.15,
                   help="camber: |py-lane_y| exceeds this (m) -> snap back to centerline (settle/warmup/run-up every 20 steps)")
    p.add_argument("--camber-air-min-deg", type=float, default=6.0,
                   help="|runup_camber|>=this enables camber-specific DART air parameters")
    p.add_argument("--camber-air-yaw-gain", type=float, default=0.8,
                   help="camber air yaw hold gain (lever 1, default 0.8)")
    p.add_argument("--camber-air-yaw-steer-max", type=float, default=0.6,
                   help="camber air yaw steer cap (default 0.6)")
    p.add_argument("--camber-air-yaw-roll-atten-floor", type=float, default=0.55,
                   help="camber large |yaw| roll-steer minimum retention ratio (default 0.55, flat 0.2)")
    p.add_argument("--camber-air-yaw-roll-atten-start-deg", type=float, default=12.0,
                   help="camber |yaw_err| at which roll-steer attenuation starts (deg)")
    p.add_argument("--camber-air-pitch-brk-touch-cap", type=float, default=0.35,
                   help="camber post-touchdown P1a pitch full brake cap (lever 2, default 0.35)")
    p.add_argument("--camber-air-early-landmatch", type=int, default=1,
                   help="1=camber early switch to P1b landmatch on touch (lever 3)")
    p.add_argument("--camber-air-early-landmatch-z", type=float, default=5.5,
                   help="camber early landmatch height threshold (m); v6.0 default 5.5 earlier wheel-speed match/braking")
    p.add_argument("--camber-air-early-landmatch-nc", type=int, default=1,
                   help="camber early landmatch minimum wheels touching")
    p.add_argument("--camber-air-roll-gain", type=float, default=2.4,
                   help="camber roll steer gain (relative to roll-gamma, diff positive-sign law)")
    p.add_argument("--camber-air-p2-roll-steer-gain", type=float, default=2.0,
                   help="DART P2 post-touch (nc>=1) roll steer multiplier relative to P1a; v2.3 default 2.0")
    p.add_argument("--camber-air-roll-steer-max", type=float, default=0.65,
                   help="camber roll steer cap (can amplify slightly on touch)")
    p.add_argument("--camber-air-touch-roll-boost", type=float, default=1.35,
                   help="camber post-touch (nc>=early_nc) roll steer cap multiplier (v6.0 suppresses land_roll)")
    p.add_argument("--camber-air-touch-roll-gain", type=float, default=1.15,
                   help="camber post-touch roll steer gain multiplier")
    p.add_argument("--camber-air-touch-roll-deadband-deg", type=float, default=1.0,
                   help="camber post-touch roll deadband (deg); smaller than default air 2 deg")
    p.add_argument("--camber-air-pred-horizon-sec", type=float, default=0.28,
                   help="camber P1a predictive pitch horizon (s); effective when dart_air_pred_horizon=0 (v6.1 jump1 pitch)")
    p.add_argument("--camber-air-pitch-overshoot-deg", type=float, default=5.0,
                   help="camber P1a: limit deepening when pitch below target exceeds this (more nose-down)")
    p.add_argument("--camber-air-pitch-overshoot-err-cap", type=float, default=2.5,
                   help="camber P1a overshoot zone err_eff cap (deg)")
    p.add_argument("--camber-air-p1b-pitch-kp", type=float, default=1.35,
                   help="camber P1b relative target pitch P gain (v6.1 touch rebound nose-down)")
    p.add_argument("--camber-air-p1b-pitch-kd", type=float, default=1.3,
                   help="camber P1b relative target pitch D gain")
    p.add_argument("--camber-air-p1b-pitch-cap", type=float, default=0.48,
                   help="camber P1b pitch brake/drive amplitude cap")
    p.add_argument("--camber-air-p1b-pitch-window-deg", type=float, default=20.0,
                   help="camber P1b pitch control error window relative to target (deg)")
    p.add_argument("--camber-air-p1b-rate-tol-dps", type=float, default=30.0,
                   help="camber P1b pitch-rate damping threshold (deg/s); replaces abs(pitch)<8 flat-road assumption")
    p.add_argument("--camber-air-yaw-atten-on-touch", type=float, default=0.35,
                   help="On touch and |roll-gamma|>=priority, yaw steer attenuation multiplier")
    p.add_argument("--camber-air-roll-priority-err-deg", type=float, default=5.0,
                   help="Post-touch roll priority over yaw when |roll-gamma| exceeds this (deg)")
    p.add_argument("--camber-nudge-enable", type=int, default=0,
                   help="camber warmup: 1=enable body-frame set_velocity nudge (default 0; roll!=0 nudge can induce lateral drift)")
    p.add_argument("--camber-steer-ff", type=float, default=0.0,
                   help="camber gravity sideslip feedforward (0=off; only explicit >0 enables, auto-camber default off)")
    p.add_argument("--camber-steer-bias", type=float, default=None,
                   help="camber run-up constant steer bias (0=auto right-turn anti-left-drift; explicit value overrides auto)")
    p.add_argument("--camber-apron-snap-stride", type=int, default=22,
                   help="camber apron low-speed periodic py snap stride (0=off)")
    p.add_argument("--camber-apron-snap-max-v", type=float, default=11.0,
                   help="camber apron snap allowed max speed (m/s)")
    p.add_argument("--landing-slope-pre-len", type=float, default=12.0,
                   help="deprecated: downhill landing now connects from ramp crest, parameter unused")
    p.add_argument("--landing-visual", type=int, default=1,
                   help="1=add raised side rails/stripes on downhill landing zone for visual identification")
    p.add_argument("--landing-flare-deg", type=float, default=0.0,
                   help="Landing flare: target pitch = back-slope angle + flare_deg; positive=slight nose-up protecting front bumper")
    p.add_argument("--front-probe-x", type=float, default=2.35,
                   help="Front bumper clearance proxy: forward distance from vehicle reference to front bumper (m, approximate)")
    p.add_argument("--front-probe-z-offset", type=float, default=-0.65,
                   help="Front bumper clearance proxy: vertical offset of front bumper relative to vehicle reference (m, negative=lower)")
    p.add_argument("--kp-pitch", type=float, default=1.0, help="Pitch vector P gain")
    p.add_argument("--kd-pitch", type=float, default=0.8, help="Pitch D damping gain (anti-overshoot)")
    p.add_argument("--dart-pitch-control", choices=["continuous", "pulse", "phased-pulse", "steer-probe", "differential", "mech-probe"],
                   default="continuous",
                   help="continuous=legacy continuous PD (global thr/brk, nose-up only); pulse=single predictive pulse; phased-pulse=three-phase short pulses; steer-probe=short in-air steer probe; differential=four-motor diff (per-wheel throttleFactor, bidirectional pitch + L/R roll); mech-probe=mechanism probe (M_y proportional sin delta): open-loop fixed steer/L-R torque differential in air to measure induced roll")
    p.add_argument("--diff-k-roll", type=float, default=0.8,
                   help="differential: roll spin base (maintain four-wheel spin when roll error exits deadband, supplies angular momentum for drive torque reaction; 0~1)")
    p.add_argument("--diff-roll-steer-gain", type=float, default=2.5,
                   help="differential roll correction gain (steer=gain*roll_d/30, positive=correct): steering reorients spin axis, drive torque reaction M_y~sin(delta). Default 2.5; 0=disable roll correction")
    p.add_argument("--dart-air-yaw-hold-gain", type=float, default=0.55,
                   help="In-air heading hold: steer += -gain*yaw_err (same sign as approach); 0=off. Corrects residual lip yaw0 + roll-steer induced yaw drift")
    p.add_argument("--dart-air-yaw-steer-max", type=float, default=0.45,
                   help="In-air yaw-hold steer component cap (rad-ish)")
    p.add_argument("--dart-air-yaw-deadband-deg", type=float, default=2.0,
                   help="In-air yaw-hold deadband (deg)")
    p.add_argument("--diff-roll-steer-max", type=float, default=1.0,
                   help="differential roll correction independent steer cap (not clamped by global smax): 1.0->delta~36 deg (strong roll), 0.3->delta~11 deg (weak). Default 1.0")
    p.add_argument("--dart-roll-adaptive", type=int, default=0,
                   help="Adaptive roll channel weight: w=min(1,|roll_err|/authority)*max(0,1-|yaw_air|/budget). Flat/small disturbance auto-tends pitch-only, yaw over budget hard-zeroed (fixes differential yaw coupling). Default 0=off (baseline reproducible)")
    p.add_argument("--dart-roll-authority-deg", type=float, default=8.0,
                   help="adaptive roll: engagement threshold (deg); v2=hysteresis switch-on threshold, v1=proportional full-weight threshold")
    p.add_argument("--multiroll-strategies", type=str, default="",
                   help="Single-vehicle interleaved mode: comma-separated law list (e.g. dart,rwpd,tobb,dart_replicate). Requires --simul-strategies single leg; rotates law by n_valid (interleaved to prevent session drift mixing), dart_replicate=same-law noise-floor bucket; statistics=distribution comparison (not same-tick pairing)")
    p.add_argument("--dart-adp-variant", type=int, default=3,
                   help="adaptive roll variant: 3=flight-latch (default: engage above threshold, hold until landing / never crossed = off throughout) 2=hysteresis band 1=proportional decay; 1/2 archive reproduction only")
    p.add_argument("--dart-yaw-budget-deg", type=float, default=0.0,
                   help="adaptive roll yaw budget (deg); high-bank interventional tests showed cutting roll -> sustained tilt -> more leakage, default 0=off; >0 low-bank experiments only")
    p.add_argument("--diff-omega-cap", type=float, default=200.0,
                   help="differential: per-wheel angular velocity safety cap (rad/s, prevents runaway only; too low blocks braking reverse torque). 0=unlimited")
    p.add_argument("--diff-pitch-rate-kp", type=float, default=2.0,
                   help="differential rate tracking: desired pitch rate = kp*(target-pitch) (deg/s), naturally decays to 0 near target preventing overshoot. Default 2.0; old default 6.0 overshot")
    p.add_argument("--diff-rate-max", type=float, default=25.0,
                   help="differential: desired pitch rate cap (deg/s). Default 25 (gentle); old default 120 overshot nose-whip")
    p.add_argument("--diff-k-drive", type=float, default=0.4,
                   help="differential: rate error to motor torque gain f=k_drive*(des_rate-pdot)/100. Default 0.4 (gentle); old default 1.5 full-throttle reversal overshot")
    p.add_argument("--diff-pitch-naive", type=int, default=0,
                   help="G8 ablation: 1=naive angle PD for pitch inner loop (f=naive_kp*err/20 - naive_kd*pdot/100, no rate-target saturation -> momentum overshoot), 0=rate tracking (default). Differential-arm Section III-B design counterexample only")
    p.add_argument("--diff-naive-kp", type=float, default=3.0,
                   help="G8 naive angle PD proportional gain (on angle error, /20 normalized). Default 3.0")
    p.add_argument("--diff-naive-kd", type=float, default=1.0,
                   help="G8 naive angle PD derivative gain (on pitch rate, /100 normalized). Default 1.0")
    p.add_argument("--diff-engage-air-steps", type=int, default=6,
                   help="differential: engage after this many consecutive no-touch steps post-takeoff (ensures clear of ramp, prevents reverse wheel scraping lip)")
    p.add_argument("--dart-disable-landmatch", type=int, default=0,
                   help="1=disable near-ground landmatch wheel-speed/brake sub-control (pure pitch authority calibration, removes landmatch landing attitude confound)")
    p.add_argument("--sim-steps-per-second", type=int, default=100,
                   help="BeamNG deterministic step rate (each step=1/K seconds); 100=>0.01s/step matches DT, fixes non-deterministic stepping T_flight/pdot/pulse duration ~6.7x distortion")
    p.add_argument("--dart-pulse-horizon-sec", type=float, default=0.35,
                   help="pulse mode prediction window: pred_pitch = pitch + pitch_rate*horizon")
    p.add_argument("--dart-pulse-sec", type=float, default=0.08,
                   help="pulse mode pulse duration (s)")
    p.add_argument("--dart-pulse-gain", type=float, default=1.0,
                   help="pulse mode predictive error gain")
    p.add_argument("--dart-pulse-kd", type=float, default=0.0,
                   help="pulse mode extra pitch-rate damping")
    p.add_argument("--dart-pulse-max-cmd", type=float, default=0.55,
                   help="pulse mode throttle/brake command cap")
    p.add_argument("--dart-pulse-map", choices=["legacy", "linear", "segmented"], default="legacy",
                   help="pulse command mapping: legacy=old pred_err/20 then cap; linear=pred_err/full_error linear to cap; segmented=error tiers")
    p.add_argument("--dart-pulse-full-error-deg", type=float, default=40.0,
                   help="linear mapping predictive error magnitude (deg) that reaches max_cmd")
    p.add_argument("--dart-pulse-seg1-err-deg", type=float, default=12.0,
                   help="segmented mapping: |pred_err|<=this threshold uses seg1_cmd")
    p.add_argument("--dart-pulse-seg2-err-deg", type=float, default=28.0,
                   help="segmented mapping: |pred_err|<=this threshold uses seg2_cmd, larger uses seg3_cmd")
    p.add_argument("--dart-pulse-seg1-cmd", type=float, default=0.10,
                   help="segmented mapping small-error command amplitude")
    p.add_argument("--dart-pulse-seg2-cmd", type=float, default=0.20,
                   help="segmented mapping medium-error command amplitude")
    p.add_argument("--dart-pulse-seg3-cmd", type=float, default=0.30,
                   help="segmented mapping large-error command amplitude")
    p.add_argument("--dart-phase-pulse-map", choices=["linear", "segmented", "legacy"], default="linear",
                   help="phased-pulse three phases shared error-to-command mapping")
    p.add_argument("--dart-phase-pitch-sign", type=float, default=1.0,
                   help="phased-pulse: predictive error to wheel torque command sign; -1 validates testbed reaction sign inversion")
    p.add_argument("--dart-phase-takeoff-window-sec", type=float, default=0.20,
                   help="phased-pulse: allow takeoff micro-pulse within this window after liftoff")
    p.add_argument("--dart-phase-takeoff-sec", type=float, default=0.04,
                   help="phased-pulse: takeoff pulse duration")
    p.add_argument("--dart-phase-takeoff-cap", type=float, default=0.16,
                   help="phased-pulse: takeoff micro-pulse command cap")
    p.add_argument("--dart-phase-takeoff-horizon-sec", type=float, default=0.18,
                   help="phased-pulse: takeoff phase prediction window")
    p.add_argument("--dart-phase-takeoff-full-error-deg", type=float, default=45.0,
                   help="phased-pulse linear mapping: takeoff phase full-amplitude error")
    p.add_argument("--dart-phase-mid-after-sec", type=float, default=0.18,
                   help="phased-pulse: wait at least this long after liftoff before mid compensation pulse")
    p.add_argument("--dart-phase-mid-sec", type=float, default=0.05,
                   help="phased-pulse: mid pulse duration")
    p.add_argument("--dart-phase-mid-cap", type=float, default=0.24,
                   help="phased-pulse: mid pulse command cap")
    p.add_argument("--dart-phase-mid-horizon-sec", type=float, default=0.22,
                   help="phased-pulse: mid phase prediction window")
    p.add_argument("--dart-phase-mid-full-error-deg", type=float, default=50.0,
                   help="phased-pulse linear mapping: mid phase full-amplitude error")
    p.add_argument("--dart-phase-landing-z", type=float, default=4.0,
                   help="phased-pulse: allow landing impact-cancel pulse when pz below this height (must be above land_match_z)")
    p.add_argument("--dart-phase-landing-sec", type=float, default=0.04,
                   help="phased-pulse: landing impact-cancel pulse duration")
    p.add_argument("--dart-phase-landing-cap", type=float, default=0.18,
                   help="phased-pulse: landing impact-cancel command cap")
    p.add_argument("--dart-phase-landing-horizon-sec", type=float, default=0.12,
                   help="phased-pulse: landing phase prediction window")
    p.add_argument("--dart-phase-landing-full-error-deg", type=float, default=35.0,
                   help="phased-pulse linear mapping: landing phase full-amplitude error")
    p.add_argument("--dart-phase-landing-kd", type=float, default=0.35,
                   help="phased-pulse: landing phase extra pitch-rate damping")
    p.add_argument("--dart-steer-probe-start-sec", type=float, default=0.05,
                   help="steer-probe: seconds after liftoff to start steer pulses")
    p.add_argument("--dart-steer-probe-pulse-sec", type=float, default=0.06,
                   help="steer-probe: single steer pulse duration")
    p.add_argument("--dart-steer-probe-gap-sec", type=float, default=0.03,
                   help="steer-probe: gap between multiple pulses")
    p.add_argument("--dart-steer-probe-cycles", type=int, default=2,
                   help="steer-probe: number of steer pulses")
    p.add_argument("--dart-steer-probe-amp", type=float, default=0.16,
                   help="steer-probe: steer pulse amplitude, clamped by --smax")
    p.add_argument("--dart-steer-probe-alternate", type=int, default=1,
                   help="steer-probe: 1=alternate left/right, 0=same-direction pulses")
    p.add_argument("--mech-probe-axis", choices=["steer", "torque"], default="steer",
                   help="mech-probe: steer=four-wheel drive + open-loop fixed steer delta (measure steer-induced roll); torque=pure L/R wheel torque differential steer=0 (control, should be ~0 roll)")
    p.add_argument("--mech-probe-amp", type=float, default=0.0,
                   help="mech-probe: steer arm=normalized steer command (delta~amp*41 deg); torque arm=L/R per-wheel factor amplitude (+/-amp)")
    p.add_argument("--mech-probe-start-sec", type=float, default=0.15,
                   help="mech-probe: seconds after takeoff to apply open-loop command (settle margin)")
    p.add_argument("--mech-probe-hold-sec", type=float, default=0.6,
                   help="mech-probe: open-loop command duration (measure roll rate response in this window)")
    p.add_argument("--gate-lip-launch-target", type=float, default=0.0,
                   help="Gate terminal speed-capped burst target launch speed (m/s): full throttle in coast zone until this speed then cut, pins v0 immune to a_brake variability propagating v0 jitter. 0=no cap (legacy full-throttle burst). Recommend ~14")
    p.add_argument("--dart-air-pred-horizon-sec", type=float, default=0.0,
                   help="DART in-air predictive pitch terminal horizon (s): use pitch+pdot*horizon to predict landing attitude error, full nose-up authority at takeoff for short-airtime under-actuation. 0=off (instantaneous PD). Recommend 0.3-0.5")
    p.add_argument("--dart-action-z-max", type=float, default=999.0,
                   help="DART pitch PD maximum action altitude; above this only coast/roll correction, for long air windows preventing early over-control")
    p.add_argument("--dart-pitch-deadband-deg", type=float, default=0.0,
                   help="DART pitch PD error deadband (deg); exit attitude correction inside deadband, prevents overshoot near target")
    p.add_argument("--dart-rate-deadband-dps", type=float, default=0.0,
                   help="DART pitch PD pitch-rate deadband (deg/s); with pitch deadband decides exit from correction")
    p.add_argument("--dart-roll-deadband-deg", type=float, default=2.0,
                   help="DART differential roll control error deadband (deg); maintain spin+steer only outside deadband (gyroscopic precession), prevents small perturbation wheel churn")
    p.add_argument("--k-roll", type=float, default=1.0, help="Roll steering correction gain")
    p.add_argument("--smax", type=float, default=0.3, help="Steering clamp")
    p.add_argument("--desteer-z", type=float, default=1.5, help="Below this altitude center steering (prevents lateral slide-out)")
    p.add_argument("--land-prep-z", type=float, default=1.5, help="(legacy) below this altitude stop pitch drive and coast")
    p.add_argument("--landprep", type=int, default=1, help="1=landing prep coast (legacy tuned); 0=drive until touch (old control)")
    p.add_argument("--landmatch", type=int, default=1, help="1=P1b landing wheel-speed rematch (Stage1, default); 0=revert to landprep")
    p.add_argument("--land-match-z", type=float, default=2.5, help="Below this altitude switch to P1b wheel-speed rematch (>land_prep_z leaves enough window)")
    p.add_argument("--wheel-r", type=float, default=0.30, help="Wheel effective radius m (Omega*=v_x/r for landing match)")
    p.add_argument("--omega-cap", type=float, default=1.5, help="P1a wheel speed cap=cap*Omega*, stop drive above to prevent blowout")
    p.add_argument("--omega-tol", type=float, default=5.0, help="P1b wheel speed tolerance rad/s (|Omega-Omega*|<tol no action)")
    p.add_argument("--kp-omega", type=float, default=1.0, help="P1b wheel speed regulation P gain")
    p.add_argument("--kd-land", type=float, default=1.0, help="P1b terminal omega_y->0 damping gain")
    p.add_argument("--rate-tol", type=float, default=60.0, help="P1b pitch-rate damping trigger threshold deg/s")
    p.add_argument("--land-brake-cap", type=float, default=0.35, help="P1b brake/drive amplitude cap (limits large nose-down)")
    p.add_argument("--max-steps", type=int, default=1200)
    p.add_argument("--post-land-sec", type=float, default=3.5,
                   help="Minimum post-landing hold (sim seconds); simul3 also constrained by post-land-gspd-gate")
    p.add_argument("--post-land-gspd-gate-mps", type=float, default=2.0,
                   help="Post-landing early teardown: all landed vehicles gspd<=this and post-land-sec elapsed before exit (default 2 m/s)")
    p.add_argument("--post-land-max-sec", type=float, default=0.0,
                   help="Post-landing hold hard cap (seconds); 0=auto (trace+15s / data+5s)")
    p.add_argument("--approach-ground-paused", type=int, default=1,
                   help="simul3 approach: 1=run-up also uses deterministic paused stepping (three vehicles nearly identical takeoff state, precise pairing + fast, requires EV drivable from stationary in paused); 0=run-up gameplay-live real-time stepping (EV must drive but non-deterministic, scattered takeoff state), switch to paused after first takeoff")
    p.add_argument("--approach-spawn-roll-deg", type=float, default=0.0,
                   help="approach simul3: run-up spawn initial roll bank (deg), combined with yaw=270 deg quaternion; same for all three vehicles (Phase 0 perturbation)")
    p.add_argument("--lip-roll-rate-kick-dps", type=float, default=0.0,
                   help="approach simul3: at liftoff same tick add roll-axis angular velocity impulse to all three (deg/s); 0=off (Phase 0 perturbation)")
    p.add_argument("--approach-simul3-refresh-every", type=int, default=0,
                   help="simul3 approach: soft_refresh (reload+rebind, no kill) every N ACCEPT; 0=disabled (default, long cohort hang prevention)")
    p.add_argument("--approach-simul3-reload-timeout-sec", type=float, default=90.0,
                   help="simul3 approach: single reload wait_scenario_ready timeout (s); timeout -> hard_refresh retry")
    p.add_argument("--approach-simul3-session-reuse", type=int, default=0,
                   help="simul3 approach: 1=reuse session within batch (teleport reset between jumps); 0=reload each jump (default, EV gear more stable)")
    p.add_argument("--resume-checkpoint", type=int, default=0,
                   help="1=resume from data/cohorts/dart_bench_{tag}.checkpoint.json (simul3 approach)")
    p.add_argument("--spawn-anchor-use", type=int, default=1,
                   help="1=reuse proven spawn coordinates from spawn_anchors.json for same recipe (default on)")
    p.add_argument("--spawn-anchor-save", type=int, default=1,
                   help="1=after ACCEPT write approach step0 coordinates to spawn_anchors.json (default on)")
    p.add_argument("--spawn-anchor-clear", type=int, default=0,
                   help="1=on startup clear spawn anchor for current fingerprint (force fallback analytic formula)")
    p.add_argument("--hud", type=int, default=1,
                   help="1=on-screen subtitle HUD (default, manual acceptance); 0=data cohort mode: no HUD + landing 0.3s/fail 0.2s fast teardown (N=30 formal runs)")
    p.add_argument("--cam-alpha", type=float, default=0.95, help="Follow camera EMA smoothing coefficient (debounce; larger=smoother/more lag)")
    p.add_argument("--cam-update-every", type=int, default=1,
                   help="bng.camera.set_free every N sim steps (default 1=every step; local approach visual recommend 5-10 to reduce wall-clock overhead; AB->C phase switch still updates immediately)")
    p.add_argument("--tag", default="cs1")
    p.add_argument("--jump-scenario", default=None,
                   help="Jump scenario library: load geometry from sim/scenarios/<name>.json (overrides defaults; controller/eval params still via CLI)")
    args = p.parse_args(argv)
    _normalize_strategy_args(args)

    _argv = list(argv if argv is not None else __import__("sys").argv[1:])
    args.__dict__["_approach_kp_y_explicit"] = any(
        x.startswith("--approach-kp-y") and not x.startswith("--approach-kp-yaw") for x in _argv)
    args.__dict__["_approach_kp_yaw_explicit"] = any(x.startswith("--approach-kp-yaw") for x in _argv)
    args.__dict__["_approach_ki_y_explicit"] = any(x.startswith("--approach-ki-y") for x in _argv)

    if getattr(args, "jump_scenario", None):
        from control.dart.jump_library import apply_to_args as _apply_jump
        _argv = list(argv if argv is not None else __import__("sys").argv[1:])
        _explicit = set()
        for _act in p._actions:
            if _act.dest and any(_os in _argv for _os in _act.option_strings):
                _explicit.add(_act.dest)
        _applied = _apply_jump(args, args.jump_scenario, skip=_explicit)
        print(f"[CS-jumplib] loaded jump scenario '{args.jump_scenario}': applied {len(_applied)} geometry keys (CLI-explicit params kept)", flush=True)

    args.__dict__["_base_air_impulse_pitch_deg"] = float(getattr(args, "air_impulse_pitch_deg", 0.0) or 0.0)
    args.__dict__["_base_air_impulse_roll_deg"] = float(getattr(args, "air_impulse_roll_deg", 0.0) or 0.0)

    if int(getattr(args, "control_trace", 0)):
        ctp = args.control_trace_path or str(ART / f"ctrl_trace_{args.tag}.jsonl")
        Path(ctp).parent.mkdir(parents=True, exist_ok=True)
        try: Path(ctp).write_text("", encoding="utf-8")
        except Exception: pass
        args.__dict__["_control_trace_path"] = ctp
        print(f"[CS-trace] control trace -> {ctp}", flush=True)

    vp, vb, R = bench.ramp_speeds(args.rise)
    aidx = {a: (ANGLES.index(a) if a in ANGLES
                else min(range(len(ANGLES)), key=lambda j: abs(ANGLES[j] - a)))
            for a in args.angles}
    from beamngpy import Scenario, Vehicle  # type: ignore
    from beamngpy.sensors import Electrics, State  # type: ignore
    from data_pipeline.beamng_session import (
        BeamNGSessionConfig, BeamNGSession, wait_scenario_ready, ensure_freerun)
    cfg = BeamNGSessionConfig.from_env(
        home=(BeamNGSessionConfig.from_env().home or r"C:\BeamNG.tech"),
        linux_binary=None,
    )
    spawn_rot = _spawn_rot_from_args(args)
    data = {}
    _simul3 = bool(int(getattr(args, "simul_3way", 0)))
    simul_legs = []
    with BeamNGSession(cfg) as bng:
        sc = Scenario("smallgrid", "dart_c7_cross")
        if _simul3:
            _gap = float(getattr(args, "simul_lane_gap", 9.0))
            _layout = _simul_layout(args)
            _copy_sp = float(getattr(args, "simul_copy_spacing", 180.0))
            _STRAT_SPEC = {
                "dart":      (True,  "off",   car_color("dart")),
                "rwpd":      (False, "rwpd",    car_color("rwpd")),
                "tobb":     (False, "tobb",   car_color("tobb")),
                "human":   (False, "human", (0.05, 0.05, 0.05, 1.0)),
                "passive": (False, "off",   (1.0, 1.0, 1.0, 1.0)),
                "dart_dual":  (True, "off",   car_color("dart")),
                "dart_pitch_only": (True, "off",   (0.6, 0.0, 0.8, 1.0)),
                "dart_roll_only":  (True, "off",   (0.0, 0.7, 0.3, 1.0)),
                "dart_latched":   (True, "off",   (1.0, 0.5, 0.0, 1.0)),
                "dart_replicate":     (True, "off",   (0.55, 0.27, 0.07, 1.0)),
            }
            assert_canonical_car_colors(_STRAT_SPEC)   # fail-closed: three-car colors changed -> ABORT
            _strats = [s.strip() for s in str(getattr(args, "simul_strategies", "dart,human,passive")).split(",") if s.strip()]
            for s in _strats:
                if s not in _STRAT_SPEC:
                    raise SystemExit(f"[CS-simul3] unknown strategy={s!r}; options {list(_STRAT_SPEC)}")
            _mr_names = [s.strip() for s in str(getattr(args, "multiroll_strategies", "") or "").split(",") if s.strip()]
            if _mr_names:
                if len(_strats) != 1:
                    raise SystemExit("[CS-multiroll] requires --simul-strategies single leg (lane commonization)")
                for s in _mr_names:
                    if s not in _STRAT_SPEC:
                        raise SystemExit(f"[CS-multiroll] unknown strategy={s!r}; options {list(_STRAT_SPEC)}")
                args.__dict__["_mr_spec"] = [(s, _STRAT_SPEC[s][0], _STRAT_SPEC[s][1]) for s in _mr_names]
                print(f"[CS-multiroll] lane-artifact-proof single-vehicle interleave: lane=shared strategies={_mr_names} "
                      f"(rotate by n_valid, dart_replicate=same-law noise floor bucket)", flush=True)
            _n = len(_strats)
            _lane_spec = []
            for _idx, s in enumerate(_strats):
                dart_on, baseline, color = _STRAT_SPEC[s]
                if _layout == "x_copy":
                    y = 0.0
                    bx = float(args.base_x) + _idx * _copy_sp
                elif _layout == "y_copy":
                    y = (_idx - (_n - 1) / 2.0) * _copy_sp
                    bx = float(args.base_x)
                else:
                    y = (_idx - (_n - 1) / 2.0) * _gap
                    bx = float(args.base_x)
                _lane_spec.append((s, dart_on, baseline, y, color, bx))
            simul_legs = []
            for label, dart_on, baseline, y, color, bx in _lane_spec:
                simul_legs.append({"label": label, "dart_on": dart_on, "strategy": label,
                                   "baseline": baseline, "y": y, "color": color, "base_x": bx})
            if str(getattr(args, "launch_mode", "")) == "approach":
                if bool(int(getattr(args, "spawn_anchor_clear", 0) or 0)):
                    try:
                        from control.dart.spawn_anchor import clear_anchors
                        if clear_anchors(args=args, simul_legs=simul_legs):
                            print("[CS-spawn-anchor] cleared anchor for current fingerprint", flush=True)
                    except Exception as e:
                        print(f"[CS-spawn-anchor] WARN clear failed: {e!r}", flush=True)
                _bind_spawn_anchors(simul_legs, args)
            for leg in simul_legs:
                label = leg["label"]
                dart_on, baseline, y, color, bx = (
                    leg["dart_on"], leg["baseline"], leg["y"], leg["color"], leg["base_x"])
                v = Vehicle(f"ego_{label}", model="sbr",
                            part_config=getattr(args, "pc", "vehicles/sbr/dart_4motor.pc"), color=color)
                try:
                    v.attach_sensor("state", State())
                    v.attach_sensor("electrics", Electrics())
                except Exception:
                    pass
                _sx, _sy, _sz = _leg_spawn_xyz(leg, args)
                sc.add_vehicle(v, pos=(_sx, _sy, _sz), rot_quat=spawn_rot)
                leg["veh"] = v
            veh = simul_legs[0]["veh"]
            if _layout == "x_copy":
                print(f"[CS-simul3] {_n} vehicles x_copy strategies={_strats} "
                      f"base_x={[round(l['base_x'], 1) for l in simul_legs]} spacing={_copy_sp}m y=0", flush=True)
            elif _layout == "y_copy":
                print(f"[CS-simul3] {_n} vehicles y_copy strategies={_strats} "
                      f"y={[round(float(l['y']), 1) for l in simul_legs]} spacing={_copy_sp}m "
                      f"base_x={float(args.base_x):.1f}", flush=True)
                for leg in simul_legs:
                    _sx, _sy, _sz = _leg_spawn_xyz(leg, args)
                    _src = "anchor" if leg.get("spawn_anchor_pos") else "formula"
                    print(f"[CS-spawn-ycopy] {leg['label']} spawn="
                          f"({_sx:.1f}, {_sy:.1f}, {_sz:.2f}) ({_src})", flush=True)
            else:
                print(f"[CS-simul3] {_n} vehicles y_lane strategies={_strats} "
                      f"y={[round(l[3],1) for l in _lane_spec]} gap={_gap}m", flush=True)
        else:
            veh = Vehicle("ego", model="sbr", part_config=getattr(args, "pc", "vehicles/sbr/dart_4motor.pc"))
            try:
                veh.attach_sensor("state", State())
                veh.attach_sensor("electrics", Electrics())
            except Exception:
                pass
            sc.add_vehicle(veh, pos=(args.base_x - args.run_up, 0.0, _spawn_z_for_lane(0.0, args)),
                           rot_quat=spawn_rot)
        sc.make(bng)
        def qlua(c): return bng.queue_lua_command(c, response=True)
        # deterministic stepping fix: in non-deterministic mode control.step(1) advances load-dependent ~0.067s (≠DT 0.01),
        _sps = int(getattr(args, "sim_steps_per_second", 100))
        _det_ok = False; _det_how = None
        for _how, _attempt in (
            ("settings.set_deterministic(sps)", lambda: bng.settings.set_deterministic(_sps)),
            ("settings.set_steps_per_second+set_deterministic", lambda: (bng.settings.set_steps_per_second(_sps), bng.settings.set_deterministic())),
            ("set_deterministic+set_steps_per_second", lambda: (bng.set_deterministic(), bng.set_steps_per_second(_sps))),
        ):
            try:
                _attempt(); _det_ok = True; _det_how = _how; break
            except Exception:
                continue
        print(f"[CS-det] deterministic={_det_ok} via={_det_how} steps_per_second={_sps} (per_step={1.0/_sps:.4f}s, DT={DT})", flush=True)
        bng.scenario.load(sc); bng.scenario.start()
        wait_scenario_ready(bng, expected_vid=(f"ego_{simul_legs[0]['label']}" if _simul3 else "ego"), timeout_sec=20.0); ensure_freerun(bng)
        for _ in range(4): bench.force_gameplay(bng); rf._step(bng, 4)
        # liveness hard gate: continue only if truly in gameplay (not menu pause), else fast abort (no empty run)
        if not bench.ensure_gameplay_live(bng, veh):
            print("[CS] ABORT(exit 6): BeamNG stuck in menu, not in gameplay. Restart BeamNG (kill all) and rerun.", flush=True)
            return 6

        _rise_base = float(args.rise)
        _cohort_shortfall = []
        for ramp_i, a in enumerate(args.angles):
            idx = aidx[a]
            vi = float(args.v_entry) if args.v_entry > 0 else vb[idx]
            ri_R = R[idx]
            if (args.landing_slope_mode == "valley"
                    and bool(int(getattr(args, "valley_adaptive_ventry", 0)))):
                _bg = (float(args.ballistic_gamma_deg) if args.ballistic_gamma_deg != 0
                       else ((a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)))
                _v0t, _atT, _clamped = solve_v0_for_airtime(
                    float(args.target_airtime), _bg, abs(float(args.landing_slope_deg)),
                    float(args.ballistic_clearance))
                args.ballistic_v0 = round(_v0t, 2)
                _off = float(getattr(args, "ventry_burst_offset", 2.0))
                vi = max(float(args.ventry_min), min(float(args.ventry_max), _v0t - _off))
                print(f"[CS-valley-adaptive] θ={a}° β={abs(float(args.landing_slope_deg))}° γ={_bg:.1f}° "
                      f"target_airtime={float(args.target_airtime):.2f}s -> v0_design={_v0t:.2f}m/s"
                      f"{'(CLAMPED, T_achievable=%.2fs)' % _atT if _clamped else ''} "
                      f"v_entry={vi:.2f}m/s(burst_off={_off}) [equal-airtime, remove steepness↔airtime confound]", flush=True)
            args.rise = _rise_base
            if args.landing_slope_mode == "valley" and int(getattr(args, "valley_auto_rise", 1)):
                _bv0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                _bg = (float(args.ballistic_gamma_deg) if args.ballistic_gamma_deg != 0
                       else ((a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)))
                _need = valley_min_peak_z(_bv0, _bg, abs(float(args.landing_slope_deg)),
                                          clearance=args.ballistic_clearance,
                                          feather_len=float(args.valley_feather_len),
                                          margin=float(args.valley_rise_margin))
                if _need > _rise_base + 0.05:
                    args.rise = round(_need, 2)
                    print(f"[CS-valley-autorise] θ={a}° β={abs(float(args.landing_slope_deg))}° "
                          f"raised kicker rise {_rise_base:.1f}m -> {args.rise:.1f}m "
                          f"(vertical room for full -β straight slope; v0={_bv0:.1f} γ={_bg:.1f}°)", flush=True)
            pts, R_lip_used = build_ramp_pts(args, args.base_x, a)
            _runup_camber = _runup_camber_deg(args)
            if abs(_runup_camber) <= 1e-6 and not bool(int(getattr(args, "runup_unified_pad", 1) or 1)):
                x_run_back = float(args.base_x) - float(args.run_up)
                pts = _prepend_runup_flat(pts, x_run_back)
            peak_x = pts[-1][0]
            peak_z = pts[-1][1]
            peak_z_land = peak_z
            if args.landing_slope_mode == "ballistic" and peak_z > 1e-6:
                _scale = (peak_z - args.ballistic_clearance) / peak_z
                pts = [(px, pz * _scale) for (px, pz) in pts]
                peak_z = pts[-1][1]
            _overlap = float(args.overlap)
            if abs(_runup_camber) > 1e-6:
                _overlap = max(_overlap, 1.22)
            _mesh_w = _camber_mesh_width(args)
            if abs(_runup_camber) > 1e-6:
                print(f"[CS-runup-camber] mesh width {_mesh_w:.0f}m (=apron, replaces narrow ramp {float(args.width):.0f}m)", flush=True)
            _cross_phi = float(getattr(args, "landing_cross_slope_deg", 0.0))
            _post_rg = _post_root_ground(runup_camber_deg=_runup_camber, cross_slope_deg=_cross_phi)
            _mesh_vis_flat = bool(args.landing_visual) and abs(_runup_camber) <= 1e-6
            if bool(args.landing_visual) and abs(_runup_camber) > 1e-6:
                print("[CS-runup-camber] camber monolithic full-width mesh (no wings/pillars; BeamNG split boxes always show seams)",
                      flush=True)
            segs = bench.ramp_segments(pts, 0, _mesh_w, args.thick, _overlap)
            segs = _prepare_takeoff_segments(segs, args)
            if abs(_runup_camber) > 1e-6:
                segs = _build_runup_apron(args) + segs
            else:
                segs = _build_toe_joint_filler(args, width=_mesh_w) + segs
            if _mesh_vis_flat:
                segs = _append_ramp_rail_visuals(segs, pts, prefix="takeoff", ri=ramp_i, width=_mesh_w,
                                                 post_root_ground=True)
            if args.ramp_mode == "kicker":
                _slopes_k = [round(math.degrees(math.atan2(pts[i + 1][1] - pts[i][1],
                                                           pts[i + 1][0] - pts[i][0])), 1)
                             for i in range(len(pts) - 1)]
                vp_k, _vb_k, _R_k = bench.ramp_speeds(args.rise)
                v_peak_k = vp_k[idx]
                omega_y0_lip = -math.degrees(v_peak_k / R_lip_used) if R_lip_used else None
                print(f"[CS-kicker] θ={a}° R_lip={R_lip_used:.1f}m lip_exit≈{_slopes_k[-1]}° "
                      f"v_peak≈{v_peak_k:.1f}m/s ω_y0_lip≈{omega_y0_lip:.0f}°/s "
                      f"peak=({peak_x:.1f},{peak_z:.1f}) n_seg={len(segs)}", flush=True)
            land_x = peak_x + ri_R
            _orig_land_len = float(args.__dict__.setdefault("_orig_landing_slope_len", float(args.landing_slope_len)))
            _mesh_end_x = float(getattr(args, "landing_mesh_end_x", 0.0) or 0.0)
            if _mesh_end_x > 0.0:
                _capped = max(2.0, _mesh_end_x - peak_x)
                args.__dict__["landing_slope_len"] = _capped
                print(f"[CS-meshcap] θ={a}° landing mesh capped at world x={_mesh_end_x:.1f} → landing slope len={_capped:.1f}m (peak_x={peak_x:.1f})", flush=True)
            else:
                args.__dict__["landing_slope_len"] = _orig_land_len
            beta_land = float(args.landing_slope_deg)
            ball_meta = None
            lpts = []
            if args.landing_slope_mode == "ballistic":
                b_v0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                if args.ballistic_gamma_deg != 0:
                    b_gamma = float(args.ballistic_gamma_deg)
                else:
                    b_gamma = (a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)
                lsegs, lpts, ball_meta = ballistic_landing_segments(
                    peak_x, peak_z_land, b_v0, b_gamma, clearance=args.ballistic_clearance,
                    length=args.landing_slope_len, width=_mesh_w, thick=args.thick,
                    ri=ramp_i, visual=_mesh_vis_flat, face_max_deg=args.dune_face_deg,
                    post_root_ground=_post_rg)
                beta_land = -float(args.dune_face_deg)
            elif args.landing_slope_mode == "gap":
                b_v0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                if args.ballistic_gamma_deg != 0:
                    b_gamma = float(args.ballistic_gamma_deg)
                else:
                    b_gamma = (a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)
                lsegs, lpts, ball_meta = gap_landing_segments(
                    peak_x, peak_z_land, b_v0, b_gamma, clearance_catch=args.ballistic_clearance,
                    gap_max=args.landing_gap_max, shape_p=args.landing_gap_shape_p,
                    feather_len=args.landing_feather_len,
                    length=args.landing_slope_len, width=_mesh_w, thick=args.thick,
                    ri=ramp_i, visual=_mesh_vis_flat, face_max_deg=args.dune_face_deg,
                    post_root_ground=_post_rg)
                beta_land = ball_meta.get("land_slope_deg", -float(args.dune_face_deg)) if ball_meta else -float(args.dune_face_deg)
            elif args.landing_slope_mode == "gap-ramp":
                b_v0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                if args.ballistic_gamma_deg != 0:
                    b_gamma = float(args.ballistic_gamma_deg)
                else:
                    b_gamma = (a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)
                _beta = abs(float(args.landing_slope_deg))
                lsegs, lpts, ball_meta = gap_ramp_landing_segments(
                    peak_x, peak_z_land, b_v0, b_gamma, _beta, clearance=args.ballistic_clearance,
                    gap_run=float(args.landing_gap_run), length=args.landing_slope_len,
                    width=_mesh_w, thick=args.thick, ri=ramp_i, visual=_mesh_vis_flat,
                    post_root_ground=_post_rg)
                if ball_meta.get("infeasible"):
                    print(f"[CS-land] SKIP θ={a}° β={_beta}° gap-ramp infeasible ({ball_meta.get('reason')}, "
                          f"meta={ball_meta}): tune --landing-gap-run / raise rise for more energy", flush=True)
                    continue
                beta_land = ball_meta.get("beta_deg", -_beta)
            elif args.landing_slope_mode == "valley":
                b_v0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                if args.ballistic_gamma_deg != 0:
                    b_gamma = float(args.ballistic_gamma_deg)
                else:
                    b_gamma = (a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)
                _beta = abs(float(args.landing_slope_deg))
                _lip_exit = (math.degrees(math.atan2(pts[-1][1] - pts[-2][1],
                                                     pts[-1][0] - pts[-2][0])) if len(pts) >= 2 else 0.0)
                lsegs, lpts, ball_meta = valley_landing_segments(
                    peak_x, peak_z_land, b_v0, b_gamma, _beta, clearance=args.ballistic_clearance,
                    floor_depth=float(args.valley_floor_depth), length_req=float(args.landing_slope_len),
                    width=_mesh_w, thick=args.thick, ri=ramp_i, visual=_mesh_vis_flat,
                    feather_len=float(args.valley_feather_len), floor_run=float(args.valley_floor_run),
                    lip_exit_slope_deg=_lip_exit, crest_blend_len=float(args.valley_crest_len),
                    post_root_ground=_post_rg)
                if ball_meta.get("infeasible"):
                    print(f"[CS-land] SKIP θ={a}° β={_beta}° valley infeasible ({ball_meta.get('reason')}, meta={ball_meta})", flush=True)
                    continue
                beta_land = ball_meta.get("beta_deg", -_beta)
            else:
                b_v0 = float(args.ballistic_v0) if args.ballistic_v0 > 0 else vi
                if args.ballistic_gamma_deg != 0:
                    b_gamma = float(args.ballistic_gamma_deg)
                else:
                    b_gamma = (a - args.lip_sweep_deg) if args.ramp_mode == "kicker" else float(a)
                if args.landing_slope_mode == "auto-to-flight":
                    target_len = max(1.0, float(args.landing_slope_len))
                    beta_land = -math.degrees(math.atan2(max(0.1, peak_z), target_len))
                lsegs = landing_slope_segments(
                    peak_x, peak_z, beta_land, length=args.landing_slope_len,
                    width=_mesh_w, thick=args.thick, ri=ramp_i, visual=_mesh_vis_flat,
                    post_root_ground=_post_rg)
                end_x = peak_x + float(args.landing_slope_len)
                end_z = peak_z + float(args.landing_slope_len) * math.tan(math.radians(beta_land))
                lpts = [(peak_x, peak_z), (end_x, max(0.0, end_z))]
            if abs(_cross_phi) > 1e-6 and lsegs:
                lsegs = bank_landing_segments(lsegs, _cross_phi, pivot_z=0.0)
                print(f"[CS-crossslope] θ={a}° landing cross-slope φ={_cross_phi:+.1f}° "
                      f"(bank about forward ground line, {'right low' if _cross_phi > 0 else 'left low'}) n_land_seg={len(lsegs)}",
                      flush=True)
            args.__dict__["_current_cross_slope_deg"] = _cross_phi
            if abs(_runup_camber) > 1e-6:
                _pivot_y = _camber_pivot_y(args)
                segs = bank_landing_segments(segs, _runup_camber, pivot_y=_pivot_y, pivot_z=0.0)
                if lsegs:
                    lsegs = bank_landing_segments(lsegs, _runup_camber, pivot_y=_pivot_y, pivot_z=0.0)
                args.__dict__["_current_runup_camber_deg"] = _runup_camber
                _cl_raise = (0.0 - _pivot_y) * math.sin(math.radians(_runup_camber))
                print(f"[CS-runup-camber] θ={a}° full-surface cross-slope γ={_runup_camber:+.1f}° "
                      f"(bank about low side pivot_y={_pivot_y:+.1f}, {'right low' if _runup_camber > 0 else 'left low'}; "
                      f"low edge grounded + center raised≈{_cl_raise:.2f}m; apron W={_camber_apron_width(args):.0f}m) "
                      f"n_takeoff_seg={len(segs)} n_land_seg={len(lsegs)}", flush=True)
            args.__dict__["_current_landing_slope_deg"] = beta_land
            _toe_dx = (ball_meta.get("dx_toe") if ball_meta else None) or float(args.landing_slope_len)
            args.__dict__["_current_land_toe_x"] = peak_x + float(_toe_dx)
            _far_dx = (ball_meta.get("length") if ball_meta else None) or _toe_dx
            if float(_far_dx) < 1.0 and args.landing_slope_mode == "fixed":
                _bc = ballistic_curve(peak_x, peak_z, b_v0, b_gamma, float(args.ballistic_clearance))
                if _bc and float(_bc.get("dx_ground") or 0.0) > 0.0:
                    _far_dx = max(float(_far_dx),
                                  float(_bc["dx_ground"]) + float(getattr(args, "valley_floor_run", 14.0) or 14.0))
                _far_dx = max(float(_far_dx), float(ri_R), 8.0)
            args.__dict__["_current_land_far_x"] = peak_x + float(_far_dx)
            args.__dict__["_current_peak_x"] = peak_x
            args.__dict__["_current_landing_profile"] = [(round(float(x), 3), round(float(z), 3)) for x, z in lpts]
            if ball_meta is not None and ball_meta.get("mode") == "gap_valley":
                print(f"[CS-gap] θ={a}° v0={b_v0:.1f} γ={b_gamma:.1f}° single-peak landing slope "
                      f"touchdown dx_land={ball_meta.get('dx_land')}m "
                      f"flight gap max={ball_meta.get('gap_max_actual')}m/min={ball_meta.get('gap_min_flight')}m "
                      f"steepest={ball_meta.get('max_face_deg')}°(σ_max={ball_meta.get('sigma_max_deg')}°) "
                      f"toe slope={ball_meta.get('land_slope_deg')}° shape_p={ball_meta.get('shape_p')} "
                      f"toe dx_toe={ball_meta.get('dx_toe')}m", flush=True)
            elif ball_meta is not None and ball_meta.get("mode") == "valley":
                print(f"[CS-valley] θ={a}° v0={b_v0:.1f} γ={b_gamma:.1f}° β={_beta}° crest fillet + straight slope + concave valley floor "
                      f"crest dx_crest={ball_meta.get('dx_crest')}m (lip exit {ball_meta.get('lip_exit_slope_deg')}° smooth→-β) "
                      f"touchdown dx_land={ball_meta.get('dx_land')}m/z_land={ball_meta.get('z_land')}m "
                      f"concave R_toe={ball_meta.get('R_toe')}m (span={ball_meta.get('feather_len')}m) "
                      f"toe dx_toe={ball_meta.get('dx_toe')}m floor_z={ball_meta.get('floor_z')}m "
                      f"flat_run={ball_meta.get('floor_run')}m airtime={ball_meta.get('airtime_s')}s"
                      + (f" WARN={ball_meta.get('warn')}" if ball_meta.get('warn') else ""), flush=True)
            elif ball_meta is not None:
                print(f"[CS-ballistic] θ={a}° v0={b_v0:.1f} γ={b_gamma:.1f}° mode={ball_meta.get('mode')} "
                      f"ballistic surface to dx_a={ball_meta.get('dx_a')}m/z_a={ball_meta.get('z_a')}m "
                      f"concave R_toe={ball_meta.get('R_toe')}m toe dx_toe={ball_meta.get('dx_toe')}m "
                      f"slip face={ball_meta.get('face_max_deg')}° (target_pitch={beta_land:.1f}°) "
                      f"clearance={args.ballistic_clearance}m", flush=True)
            _deleted, _placed, _enum = _place_simul3_ramp_meshes(
                bng, qlua, simul_legs, segs, lsegs, lpts, peak_x, args)
            rf._step(bng, 3); bench.force_gameplay(bng)
            if lsegs:
                actual_len = args.landing_slope_len
                if beta_land < 0:
                    actual_len = min(
                        actual_len,
                        max(0.5, (peak_z - 0.05) / abs(math.tan(math.radians(beta_land)))))
                target_land = beta_land + args.landing_flare_deg
                print(f"[CS-land] θ={a}° landing_slope={beta_land:.1f}° flare={args.landing_flare_deg:.1f}° "
                      f"mode={args.landing_slope_mode} target_pitch={target_land:.1f}° "
                      f"peak=({peak_x:.1f},{peak_z:.1f}) "
                      f"R_flight={ri_R:.1f} visible_len={actual_len:.1f}m requested_len={args.landing_slope_len}m "
                      f"visual_segs={len(lsegs)}",
                      flush=True)
            if bool(int(getattr(args, "approach_fresh_spawn", 0))) and not _simul3:
                _run_approach_fresh_spawn(bng, veh, qlua, sc, segs, lsegs, a, ri_R, vi, ramp_i, args, data,
                                          wait_scenario_ready=wait_scenario_ready, ensure_freerun=ensure_freerun)
                continue
            if _simul3 and str(getattr(args, "launch_mode", "")) == "approach":
                _labels = [l["label"] for l in simul_legs]
                _pvid = simul_legs[0]["veh"].vid
                _cj = int(getattr(args, "cond_jitter", 0) or 0)
                target_valid = _cj if _cj > 0 else int(args.rolls)
                max_attempts = max(target_valid, int(getattr(args, "paired_max_attempts", 0)) or (target_valid * 3))
                _resume = bool(int(getattr(args, "resume_checkpoint", 0) or 0))
                _ckpt = _load_session_checkpoint(args.tag) if _resume else None
                n_valid = 0
                start_k = 0
                if _ckpt and float(_ckpt.get("angle", -999)) == float(a) and int(_ckpt.get("n_valid", 0)) > 0:
                    _ckpt_data = _ckpt.get("data") or {}
                    _da = _ckpt_data.get(a) or _ckpt_data.get(str(a))
                    if _da and all(lab in _da for lab in _labels):
                        data[a] = _da
                        n_valid = int(_ckpt["n_valid"])
                        start_k = int(_ckpt.get("last_jump_id", -1)) + 1
                        print(f"[CS-ckpt] resume angle={a} valid={n_valid}/{target_valid} "
                              f"from jump={start_k} (ckpt saved {_ckpt.get('saved_at')})", flush=True)
                    else:
                        print(f"[CS-ckpt] WARN resume data missing/incomplete for angle={a}, cold start", flush=True)
                        data[a] = {lab: [] for lab in _labels}; data[a]["invalid_jumps"] = []
                else:
                    data[a] = {lab: [] for lab in _labels}; data[a]["invalid_jumps"] = []
                _last_refresh_at_valid = (
                    int(_ckpt.get("last_refresh_at_valid", 0))
                    if (_ckpt and n_valid > 0) else 0
                )
                _gate_tgt_pitch = float(beta_land) + float(args.landing_flare_deg)
                _gate_v_eff, _gate_audit = reachability_gate_decision(args, a, float(vi), _gate_tgt_pitch)
                data[a]["reachability_gate"] = _gate_audit
                print(f"[CS-gate-appr] θ={a}° gate={'ON' if _gate_audit['enabled'] else 'OFF'} "
                      f"{_gate_audit['decision']}/{_gate_audit['action']}({_gate_audit['reason']}) "
                      f"v_req={_gate_audit['v_requested']}→v_eff={_gate_audit['v_effective']} "
                      f"v_max={_gate_audit['v_max']} intervened={_gate_audit['intervened']}", flush=True)
                _refresh_every = int(getattr(args, "approach_simul3_refresh_every", 10) or 0)
                _session_reuse = bool(int(getattr(args, "approach_simul3_session_reuse", 0) or 0))
                print(f"[CS-simul3-appr] θ={a}° synced {len(_labels)}-vehicle approach strategies={_labels} "
                      f"target={target_valid} max_attempts={max_attempts} v_entry={vi} run_up={args.run_up} "
                      f"refresh_every={_refresh_every} session_reuse={int(_session_reuse)} "
                      f"resume={int(_resume)} start_k={start_k} "
                      f"(reload={'batch' if _session_reuse else 'per-jump'}, EV box reset to D)", flush=True)
                for k in range(start_k, max_attempts):
                    if n_valid >= target_valid:
                        break
                    bng, _v_req, _last_refresh_at_valid, _prep_ok = _simul3_appr_prep_jump(
                        bng, sc, qlua, simul_legs, segs, lsegs, lpts, peak_x, args,
                        k=k, n_valid=n_valid, refresh_every=_refresh_every,
                        last_refresh_at_valid=_last_refresh_at_valid,
                        _pvid=_pvid, _session_reuse=_session_reuse, ramp_i=ramp_i, a=a, vi=vi,
                        _cj=_cj, _gate_v_eff=_gate_v_eff, _gate_audit=_gate_audit,
                        wait_scenario_ready=wait_scenario_ready, ensure_freerun=ensure_freerun,
                    )
                    if not _prep_ok:
                        print(f"[CS-simul3-appr] θ={a}° jump{k} ABORT prep(spawn dead after 3 tries), skip",
                              flush=True)
                        continue
                    if _cj > 0:
                        _gate_v_eff, _gate_audit = reachability_gate_decision(args, a, _v_req, _gate_tgt_pitch)
                    _v_jump, _ = prep_jump_e6_injection(args, _gate_v_eff if _cj <= 0 else _v_req, jump_seed=ramp_i * 1000 + k)
                    res = one_jump_simul3(bng, qlua, simul_legs, angle=a, base_x=args.base_x,
                                          run_up=args.run_up, v_entry=_v_jump, R_flight=ri_R,
                                          peak_x=peak_x, peak_z=peak_z, pts=pts,
                                          ramp_idx=ramp_i, n_ramps=len(args.angles), args=args,
                                          ground_launch=True)
                    _gate_audit = dict(_gate_audit, mode="approach_simul3", v_requested=_v_req,
                                       gate_intervened_steps=res.get(_labels[0], {}).get("gate_intervened_steps"))
                    ts = {lab: (res[lab].get("takeoff_state") or {}) for lab in _labels}
                    v0s = [ts[l].get("v0_mps") for l in _labels if ts[l].get("v0_mps") is not None]
                    th0s = [ts[l].get("theta0_deg") for l in _labels if ts[l].get("theta0_deg") is not None]
                    r0s = [ts[l].get("roll0_deg") for l in _labels if ts[l].get("roll0_deg") is not None]
                    v0_spread = round(max(v0s) - min(v0s), 2) if len(v0s) == len(_labels) else None
                    th0_spread = round(max(th0s) - min(th0s), 2) if len(th0s) == len(_labels) else None
                    r0_spread = round(max(r0s) - min(r0s), 2) if len(r0s) == len(_labels) else None
                    reasons = [f"{lab}:no_takeoff" for lab in _labels if not res[lab].get("took_off")]
                    valid = not reasons
                    _spread_ok, _spread_reasons = _simul_takeoff_spread_passes(
                        v0_spread, th0_spread, r0_spread, args)
                    if valid and not _spread_ok:
                        valid = False
                        reasons.extend(_spread_reasons)
                    for lab in _labels:
                        res[lab]["reachability_gate"] = _gate_audit
                        res[lab]["jump_id"] = k
                        res[lab]["takeoff_v0_spread"] = v0_spread
                        res[lab]["takeoff_theta0_spread"] = th0_spread
                        res[lab]["takeoff_roll0_spread"] = r0_spread
                        res[lab]["simul_layout"] = _simul_layout(args)
                    _pitch_str = " ".join(f"{lab}={res[lab].get('land_pitch')}" for lab in _labels)
                    print(f"[CS-simul3-appr] θ={a}° jump{k} v_req={_v_req} pitch {_pitch_str} | "
                          f"v0_spread={v0_spread} th0_spread={th0_spread} roll0_spread={r0_spread} "
                          f"valid={valid} reasons={reasons}", flush=True)
                    if valid:
                        for lab in _labels:
                            data[a][lab].append(res[lab])
                        n_valid += 1
                        print(f"[CS-simul3-appr] θ={a}° jump{k} ACCEPT valid={n_valid}/{target_valid}", flush=True)
                        _save_spawn_anchors_from_jump(
                            simul_legs=simul_legs, args=args, res=res, tag=args.tag, jump_id=k)
                        _save_session_checkpoint(tag=args.tag, data=data, n_valid=n_valid, jump_id=k,
                                                 target_valid=target_valid, angle=a,
                                                 last_refresh_at_valid=_last_refresh_at_valid)
                    else:
                        data[a]["invalid_jumps"].append({"jump_id": k, **{lab: res[lab] for lab in _labels}})
                if n_valid < target_valid:
                    print(f"[CS-simul3-appr] WARN θ={a}° only valid={n_valid}/{target_valid} after {max_attempts} jumps", flush=True)
                    _cohort_shortfall.append((a, n_valid, target_valid))
                continue
            if _simul3:
                target_valid = int(getattr(args, "paired_valid_target", 0)) or args.rolls
                max_attempts = int(getattr(args, "paired_max_attempts", 0)) or (target_valid * 4)
                min_apex = float(args.pair_min_apex_clearance); max_apex = float(args.pair_max_apex_clearance)
                req_mesh = bool(int(args.pair_require_landing_mesh))
                _labels = [l["label"] for l in simul_legs]
                _nlane = len(_labels)
                _mr_spec = args.__dict__.get("_mr_spec") or []
                _mr = bool(_mr_spec)
                if _mr:
                    assert _nlane == 1, "multiroll requires single leg (--simul-strategies must give exactly 1)"
                    data[a] = {name: [] for name, _, _ in _mr_spec}
                    data[a]["invalid_jumps"] = []
                else:
                    data[a] = {lab: [] for lab in _labels}
                    data[a]["invalid_jumps"] = []
                n_valid = 0
                _gate_tgt_pitch = float(beta_land) + float(args.landing_flare_deg)
                _cj = int(getattr(args, "cond_jitter", 0) or 0)
                if _cj > 0:
                    target_valid = _cj; max_attempts = _cj
                if _cj <= 0:
                    _gate_v_eff, _gate_audit = reachability_gate_decision(args, a, vi, _gate_tgt_pitch)
                    data[a]["reachability_gate"] = _gate_audit
                    print(f"[CS-gate] θ={a}° gate={'ON' if _gate_audit['enabled'] else 'OFF'} "
                          f"{_gate_audit['decision']}/{_gate_audit['action']}({_gate_audit['reason']}) "
                          f"v_req={_gate_audit['v_requested']}→v_eff={_gate_audit['v_effective']} "
                          f"v_max={_gate_audit['v_max']} intervened={_gate_audit['intervened']}", flush=True)
                else:
                    data[a]["reachability_gate"] = {"enabled": bool(int(getattr(args, "reachability_gate", 0))),
                                                    "mode": "cond_jitter", "n": _cj}
                print(f"[CS-simul3] θ={a}° synced {_nlane} vehicles strategies={_labels}: target_valid={target_valid} "
                      f"max_attempts={max_attempts} apex∈[{min_apex},{max_apex}] require_mesh={req_mesh} "
                      f"cond_jitter={_cj}", flush=True)
                for k in range(max_attempts):
                    if _cj > 0:
                        _rng = random.Random(int(args.cond_jitter_seed) + ramp_i * 1000 + k)
                        _v_req = _rng.uniform(float(args.cond_jitter_v_lo), float(args.cond_jitter_v_hi))
                        _pi = _rng.uniform(float(args.cond_jitter_pitch_lo), float(args.cond_jitter_pitch_hi))
                        _ri = _rng.uniform(float(args.cond_jitter_roll_lo), float(args.cond_jitter_roll_hi))
                        args.__dict__["air_impulse_pitch_deg"] = round(_pi, 3)
                        args.__dict__["air_impulse_roll_deg"] = round(_ri, 3)
                        _gate_v_eff, _gate_audit = reachability_gate_decision(args, a, _v_req, _gate_tgt_pitch)
                        _gate_audit = dict(_gate_audit, pitch_imp_deg=round(_pi, 2), roll_imp_deg=round(_ri, 2))
                    if _mr:
                        _mr_name, _mr_c7on, _mr_base = _mr_spec[n_valid % len(_mr_spec)]
                        _leg0 = simul_legs[0]
                        _leg0["strategy"] = _mr_name if _mr_name != "dart_replicate" else "dart"
                        _leg0["dart_on"] = _mr_c7on
                        _leg0["baseline"] = _mr_base
                    _v_jump, _jit = prep_jump_e6_injection(args, _gate_v_eff, jump_seed=ramp_i * 1000 + k)
                    if _jit:
                        _gate_audit = dict(_gate_audit or {}, takeoff_state_jitter=_jit)
                    res = one_jump_simul3(bng, qlua, simul_legs, angle=a, base_x=args.base_x,
                                          run_up=args.run_up, v_entry=_v_jump, R_flight=ri_R,
                                          peak_x=peak_x, peak_z=peak_z, pts=pts,
                                          ramp_idx=ramp_i, n_ramps=len(args.angles), args=args)
                    for lab in _labels:
                        res[lab]["reachability_gate"] = _gate_audit
                    reasons = []
                    for lab in _labels:
                        r = res[lab]
                        if not r.get("took_off"): reasons.append(f"{lab}:no_takeoff")
                        ap = r.get("apex_clearance_land")
                        if min_apex > 0 or max_apex > 0:
                            if ap is None: reasons.append(f"{lab}:no_apex")
                            else:
                                if min_apex > 0 and ap < min_apex: reasons.append(f"{lab}:apex<{min_apex}")
                                if max_apex > 0 and ap > max_apex: reasons.append(f"{lab}:apex>{max_apex}")
                        if req_mesh and not r.get("land_on_mesh"): reasons.append(f"{lab}:mesh_miss")
                    ts = {lab: (res[lab].get("takeoff_state") or {}) for lab in _labels}
                    v0s = [ts[l].get("v0_mps") for l in ts if ts[l].get("v0_mps") is not None]
                    th0s = [ts[l].get("theta0_deg") for l in ts if ts[l].get("theta0_deg") is not None]
                    v0_spread = round(max(v0s) - min(v0s), 2) if len(v0s) == _nlane else None
                    th0_spread = round(max(th0s) - min(th0s), 2) if len(th0s) == _nlane else None
                    valid = not reasons
                    for lab in _labels:
                        res[lab]["jump_id"] = k
                        res[lab]["jump_valid"] = valid
                        res[lab]["takeoff_v0_spread"] = v0_spread
                        res[lab]["takeoff_theta0_spread"] = th0_spread
                    _pitch_str = " ".join(f"{lab}={res[lab].get('land_pitch')}" for lab in _labels)
                    if _cj > 0:
                        print(f"[CS-jit] θ={a}° jump{k} cond(v={_gate_audit['v_requested']},"
                              f"pi={_gate_audit.get('pitch_imp_deg')},ri={_gate_audit.get('roll_imp_deg')}) "
                              f"gate={'ON' if _gate_audit['enabled'] else 'OFF'}→v_eff={_gate_audit['v_effective']} "
                              f"| pitch {_pitch_str}", flush=True)
                    else:
                        print(f"[CS-simul3] θ={a}° jump{k} pitch {_pitch_str} | "
                              f"takeoff v0_spread={v0_spread} θ0_spread={th0_spread} valid={valid} "
                              f"reasons={reasons}", flush=True)
                    if valid:
                        if _mr:
                            res[_labels[0]]["mr_strategy"] = _mr_name
                            data[a][_mr_name].append(res[_labels[0]])
                            n_valid += 1
                            print(f"[CS-simul3] θ={a}° jump{k} ACCEPT[{_mr_name}] "
                                  f"valid={n_valid}/{target_valid} "
                                  f"(bucket={len(data[a][_mr_name])})", flush=True)
                        else:
                            for lab in _labels:
                                data[a][lab].append(res[lab])
                            n_valid += 1
                            print(f"[CS-simul3] θ={a}° jump{k} ACCEPT valid={n_valid}/{target_valid}", flush=True)
                        if n_valid >= target_valid:
                            break
                    else:
                        data[a]["invalid_jumps"].append({"jump_id": k, **{lab: res[lab] for lab in _labels}})
                if n_valid < target_valid:
                    print(f"[CS-simul3] WARN θ={a}° only valid={n_valid}/{target_valid} after {max_attempts} jumps", flush=True)
                    _cohort_shortfall.append((a, n_valid, target_valid))
                continue
            data[a] = {"off": [], "on": [], "invalid_pairs": []}
            args.__dict__["_cmp_off"] = None
            if args.paired_rolls:
                target_valid = int(getattr(args, "paired_valid_target", 0))
                max_attempts = int(getattr(args, "paired_max_attempts", 0)) or args.rolls
                n_valid = 0
                print(f"[CS-pair] θ={a}° paired_rolls=ON: each pair runs OFF→ON back-to-back, pair_id written to JSON "
                      f"target_valid={target_valid} max_attempts={max_attempts}", flush=True)
                _pair_rng = random.Random(int(args.pair_order_seed) + ramp_i * 1009)
                _bias_floor = bool(int(args.pair_bias_floor))
                for k in range(max_attempts):
                    args.__dict__["_pair_id"] = k
                    order = ["off", "on"]
                    if int(args.pair_randomize_order):
                        _pair_rng.shuffle(order)
                    _v_jump, _jit = prep_jump_e6_injection(args, vi, jump_seed=ramp_i * 1000 + k)
                    _jit_pitch = args.air_impulse_pitch_deg
                    _jit_roll = args.air_impulse_roll_deg
                    for run_idx, mode in enumerate(order):
                        if _jit:
                            args.__dict__["air_impulse_pitch_deg"] = _jit_pitch
                            args.__dict__["air_impulse_roll_deg"] = _jit_roll
                            args.__dict__["_takeoff_jitter_audit"] = _jit
                        if args.respawn_per_roll:
                            veh = respawn_ego(bng, veh, (args.base_x - args.run_up, 0.0,
                                                         _spawn_z_for_lane(0.0, args)),
                                              spawn_rot, Vehicle, Electrics)
                        _c7_on = (mode == "on") and (not _bias_floor)
                        r = one_jump(bng, veh, qlua, angle=a, base_x=args.base_x, run_up=args.run_up,
                                     v_entry=_v_jump, R_flight=ri_R, dart_on=_c7_on, args=args,
                                     ramp_idx=ramp_i, n_ramps=len(args.angles), hud_on=bool(args.hud))
                        r["roll_order_index"] = run_idx
                        r["bias_floor"] = _bias_floor
                        data[a][mode].append(r)
                        _ts = r.get("takeoff_state") or {}
                        print(f"[CS] θ={a}° {mode}(seq{run_idx}) pair{k} took_off={r['took_off']} land_pitch={r['land_pitch']} "
                              f"err={r.get('land_pitch_error')} v0={_ts.get('v0_mps')} "
                              f"wy0={_ts.get('omega_y0_dps')} airtime={r['airtime']}", flush=True)
                    off_r, on_r = data[a]["off"][-1], data[a]["on"][-1]
                    pair_match = False
                    pair_delta = None
                    pair_reject_reasons = []
                    if off_r.get("land_pitch_error") is not None and on_r.get("land_pitch_error") is not None:
                        pair_delta = abs(off_r["land_pitch_error"]) - abs(on_r["land_pitch_error"])
                        off_ts, on_ts = off_r.get("takeoff_state") or {}, on_r.get("takeoff_state") or {}
                        if off_ts and on_ts:
                            checks = [
                                ("v0", abs(float(off_ts.get("v0_mps", 999)) - float(on_ts.get("v0_mps", -999))), args.pair_v0_tol),
                                ("wy0", abs(float(off_ts.get("omega_y0_dps", 999)) - float(on_ts.get("omega_y0_dps", -999))), args.pair_wy0_tol),
                                ("theta0", abs(float(off_ts.get("theta0_deg", 999)) - float(on_ts.get("theta0_deg", -999))), args.pair_theta0_tol),
                                ("vz0", abs(float(off_ts.get("vz0_mps", 999)) - float(on_ts.get("vz0_mps", -999))), args.pair_vz0_tol),
                                ("roll0", abs(float(off_ts.get("roll0_deg", 999)) - float(on_ts.get("roll0_deg", -999))), args.pair_roll0_tol),
                                ("yaw0", abs(float(off_ts.get("yaw0_deg", 999)) - float(on_ts.get("yaw0_deg", -999))), args.pair_yaw0_tol),
                                ("y0", abs(float(off_ts.get("y0_m", 999)) - float(on_ts.get("y0_m", -999))), args.pair_y0_tol),
                            ]
                            for name, diff, tol in checks:
                                if diff > tol:
                                    pair_reject_reasons.append(f"{name}={diff:.2f}>{tol}")
                            min_tf = float(args.pair_min_t_flight)
                            if min_tf > 0:
                                off_tf = off_r.get("T_flight")
                                on_tf = on_r.get("T_flight")
                                if off_tf is None or on_tf is None or float(off_tf) < min_tf or float(on_tf) < min_tf:
                                    pair_reject_reasons.append(f"T_flight<{min_tf}")
                            min_apex = float(args.pair_min_apex_clearance)
                            max_apex = float(args.pair_max_apex_clearance)
                            if min_apex > 0 or max_apex > 0:
                                off_ap = off_r.get("apex_clearance_land")
                                on_ap = on_r.get("apex_clearance_land")
                                if off_ap is None or on_ap is None:
                                    pair_reject_reasons.append("missing_apex_clearance")
                                else:
                                    if min_apex > 0 and (float(off_ap) < min_apex or float(on_ap) < min_apex):
                                        pair_reject_reasons.append(f"apex_clearance<{min_apex}")
                                    if max_apex > 0 and (float(off_ap) > max_apex or float(on_ap) > max_apex):
                                        pair_reject_reasons.append(f"apex_clearance>{max_apex}")
                            if int(args.pair_require_landing_mesh):
                                if not off_r.get("land_on_mesh") or not on_r.get("land_on_mesh"):
                                    pair_reject_reasons.append("landing_mesh_miss")
                            pair_match = not pair_reject_reasons
                        else:
                            pair_reject_reasons.append("missing_takeoff_state")
                    else:
                        pair_reject_reasons.append("missing_landing_error")
                    off_r["pair_match"] = pair_match
                    on_r["pair_match"] = pair_match
                    off_r["pair_reject_reasons"] = pair_reject_reasons
                    on_r["pair_reject_reasons"] = pair_reject_reasons
                    off_r["pair_delta_abs_error"] = round(pair_delta, 3) if pair_delta is not None else None
                    on_r["pair_delta_abs_error"] = round(pair_delta, 3) if pair_delta is not None else None
                    def _pd(key, lower_better=True, use_abs=False):
                        ov, nv = off_r.get(key), on_r.get(key)
                        if not isinstance(ov, (int, float)) or not isinstance(nv, (int, float)):
                            return None
                        if use_abs:
                            ov, nv = abs(ov), abs(nv)
                        d = (ov - nv) if lower_better else (nv - ov)
                        return round(d, 3)
                    multi = {
                        "impact": _pd("land_impact_speed_mps"),
                        "vz": _pd("land_vz_mps", use_abs=True),
                        "roll": _pd("land_roll", use_abs=True),
                        "pitchrate": _pd("max_pitchrate"),
                        "front_clear": _pd("front_clearance", lower_better=False),
                        "tumble": ((1 if off_r.get("tumbled") else 0) - (1 if on_r.get("tumbled") else 0)),
                    }
                    off_r["pair_delta_multi"] = multi
                    on_r["pair_delta_multi"] = multi
                    if pair_delta is not None:
                        print(f"[CS-pair] θ={a}° pair{k} abs_err OFF→DART "
                              f"{abs(off_r['land_pitch_error']):.1f}→{abs(on_r['land_pitch_error']):.1f} "
                              f"Δ={pair_delta:+.1f} matched={pair_match} reasons={pair_reject_reasons}", flush=True)
                    else:
                        print(f"[CS-pair] θ={a}° pair{k} missing landing error matched=False", flush=True)
                    if target_valid > 0:
                        off_keep, on_keep = data[a]["off"].pop(), data[a]["on"].pop()
                        if pair_match:
                            data[a]["off"].append(off_keep)
                            data[a]["on"].append(on_keep)
                            n_valid += 1
                            print(f"[CS-pair] θ={a}° pair{k} ACCEPT valid={n_valid}/{target_valid}", flush=True)
                            if n_valid >= target_valid:
                                break
                        else:
                            data[a]["invalid_pairs"].append({"pair_id": k, "off": off_keep, "on": on_keep})
                            print(f"[CS-pair] θ={a}° pair{k} REJECT invalid={len(data[a]['invalid_pairs'])}", flush=True)
                if target_valid > 0 and n_valid < target_valid:
                    print(f"[CS-pair] WARN θ={a}° only collected valid={n_valid}/{target_valid} "
                          f"after attempts={max_attempts}", flush=True)
                args.__dict__["_pair_id"] = None
            else:
                for mode in ("off", "on"):
                    for k in range(args.rolls):
                        if args.respawn_per_roll:
                            veh = respawn_ego(bng, veh, (args.base_x - args.run_up, 0.0,
                                                         _spawn_z_for_lane(0.0, args)),
                                              spawn_rot, Vehicle, Electrics)
                        _v_jump, _ = prep_jump_e6_injection(args, vi, jump_seed=ramp_i * 1000 + k)
                        r = one_jump(bng, veh, qlua, angle=a, base_x=args.base_x, run_up=args.run_up,
                                     v_entry=_v_jump, R_flight=ri_R, dart_on=(mode == "on"), args=args,
                                     ramp_idx=ramp_i, n_ramps=len(args.angles), hud_on=bool(args.hud))
                        data[a][mode].append(r)
                    if mode == "off":
                        op = med([x["land_pitch"] for x in data[a]["off"]])
                        orr = med([x["land_roll"] for x in data[a]["off"] if x["land_roll"] is not None])
                        args.__dict__["_cmp_off"] = (op if op is not None else 0.0,
                                                     orr if orr is not None else 0.0)
                        _ts = r.get("takeoff_state") or {}
                        print(f"[CS] θ={a}° {mode} roll{k} took_off={r['took_off']} land_pitch={r['land_pitch']} "
                              f"land_roll={r['land_roll']} tumbled={r['tumbled']} max_z={r['max_z']} "
                              f"v0={_ts.get('v0_mps')} wy0={_ts.get('omega_y0_dps')} "
                              f"max_pr={r['max_pitchrate']}°/s airtime={r['airtime']}", flush=True)
            offp = med([x["land_pitch"] for x in data[a]["off"]])
            onp = med([x["land_pitch"] for x in data[a]["on"]])
            offe = med([x.get("land_pitch_error") for x in data[a]["off"]])
            one = med([x.get("land_pitch_error") for x in data[a]["on"]])
            offr = med([abs(x["land_roll"]) for x in data[a]["off"] if x["land_roll"] is not None])
            onr = med([abs(x["land_roll"]) for x in data[a]["on"] if x["land_roll"] is not None])
            off_tum = sum(x["tumbled"] for x in data[a]["off"]); on_tum = sum(x["tumbled"] for x in data[a]["on"])
            dpitch = round(abs(offp) - abs(onp), 1) if (offp is not None and onp is not None) else None
            derr = round(abs(offe) - abs(one), 1) if (offe is not None and one is not None) else None
            droll = round(offr - onr, 1) if (offr is not None and onr is not None) else None
            off_ww = med([x.get("wheel_w_land") for x in data[a]["off"]])
            on_ww = med([x.get("wheel_w_land") for x in data[a]["on"]])
            tgt_ww = med([x.get("omega_tgt_land") for x in data[a]["on"] if x.get("omega_tgt_land")])
            on_ratio = round(on_ww / tgt_ww, 1) if (on_ww and tgt_ww) else None
            off_clear = med([((x.get("front_clearance") or {}).get("clearance_m"))
                             for x in data[a]["off"]])
            on_clear = med([((x.get("front_clearance") or {}).get("clearance_m"))
                            for x in data[a]["on"]])
            n_off, n_on = len(data[a]["off"]), len(data[a]["on"])
            print(f"[DART-CS] θ={a}° | OFF pitch={offp} roll={offr} tumble={off_tum}/{n_off} "
                  f"| DART pitch={onp} roll={onr} tumble={on_tum}/{n_on} "
                  f"| Δ|pitch|={dpitch}(+=improved) Δ|roll|={droll} max_pr={med([x['max_pitchrate'] for x in data[a]['off']])}°/s",
                  flush=True)
            target_pitch = (
                float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg))
                + float(args.landing_flare_deg)
            )
            print(f"[DART-LAND] θ={a}° target={target_pitch:.1f}° | "
                  f"OFF err={offe}° DART err={one}° Δ|err|={derr}(+=closer to landing zone)", flush=True)
            print(f"[DART-CLEAR] θ={a}° front_clearance_proxy(m): OFF={off_clear} DART={on_clear} "
                  f"(>0=front bumper above back-slope; larger safer; absolute value needs vehicle calibration)", flush=True)
            print(f"[DART-WW] θ={a}° landing wheel speed Ω*={tgt_ww} | OFF Ω={off_ww} | DART Ω={on_ww} "
                  f"overspin ratio={on_ratio}× (target≈1, >2 harsh slip/blowout)", flush=True)
            try:
                off_ts = [x["takeoff_state"] for x in data[a]["off"] if x.get("takeoff_state")]
                if off_ts:
                    def _srt(key):
                        return sorted(t[key] for t in off_ts if t.get(key) is not None)
                    t0 = med([t.get("theta0_deg") for t in off_ts]); w0 = med([t.get("omega_y0_dps") for t in off_ts])
                    v0 = med([t.get("v0_mps") for t in off_ts]); tf = med([x["T_flight"] for x in data[a]["off"]
                                                                          if x.get("T_flight")])
                    v0s = _srt("v0_mps"); w0s = _srt("omega_y0_dps"); t0s = _srt("theta0_deg")
                    if v0s and w0s and t0s:
                        print(f"[DART-TO] θ={a}° takeoff state (OFF n={len(off_ts)}): "
                              f"θ₀={t0}°[{t0s[0]}~{t0s[-1]}] ω_y0={w0}°/s[{w0s[0]}~{w0s[-1]}] "
                              f"v₀={v0}m/s[{v0s[0]}~{v0s[-1]}] T_flight={tf}s (median [min~max])", flush=True)
                lxs = sorted(x["land_x_world"] for x in (data[a]["off"] + data[a]["on"])
                             if x.get("land_x_world") is not None)
                if lxs:
                    _peakx = float(args.__dict__.get("_current_peak_x", args.base_x))
                    _farx = float(args.__dict__.get("_current_land_far_x",
                                                     args.__dict__.get("_current_land_toe_x", _peakx)))
                    _span_cap = max(float(getattr(args, "cam_c_span_m", 32.0)), _farx - _peakx)
                    cam_lo, cam_hi = _peakx, _peakx + min(_farx - _peakx, _span_cap) + 6.0
                    in_frame = all(cam_lo <= lx <= cam_hi for lx in lxs)
                    print(f"[DART-CAM] θ={a}° touchdown x=[{lxs[0]}~{lxs[-1]}] camera frame≈[{cam_lo:.0f}~{cam_hi:.0f}] "
                          f"touchdown in frame={in_frame}", flush=True)
                rows_imp = [x for x in (data[a]["off"] + data[a]["on"]) if x.get("land_x_world") is not None]
                if rows_imp:
                    on_mesh = sum(1 for x in rows_imp if x.get("land_on_mesh"))
                    apex_med = med([x.get("apex_clearance_land") for x in rows_imp])
                    vz_med = med([abs(x.get("land_vz_mps")) for x in rows_imp if x.get("land_vz_mps") is not None])
                    imp_med = med([x.get("land_impact_speed_mps") for x in rows_imp])
                    print(f"[DART-IMPACT] θ={a}° landing_mesh_hit={on_mesh}/{len(rows_imp)} "
                          f"apex_clearance_med={apex_med}m |vz_land|_med={vz_med}m/s "
                          f"impact_speed_med={imp_med}m/s", flush=True)
            except Exception as _rep_e:
                print(f"[DART-WARN] θ={a}° per-ramp extra report exception (skipped, data save unaffected): "
                      f"{type(_rep_e).__name__}: {_rep_e}", flush=True)

        ART.mkdir(parents=True, exist_ok=True)
        try:
            (ART / f"dart_bench_{args.tag}.raw.json").write_text(
                json.dumps(
                    {"params": vars(args), "data": data,
                     "_provenance": make_provenance_from_data(
                         data, "deterministic_stepped",
                         sps=int(getattr(args, "sim_steps_per_second", 100) or 100))},
                    indent=2, ensure_ascii=False, default=str),
                encoding="utf-8")
            print(f"[CS] saved RAW dart_bench_{args.tag}.raw.json (pre-aggregation safety copy)", flush=True)
        except Exception as _raw_e:
            print(f"[CS] WARN raw safety copy save failed: {type(_raw_e).__name__}: {_raw_e}", flush=True)

        if _simul3:
            summary = []
            _labels = [l["label"] for l in simul_legs]
            if args.__dict__.get("_mr_spec"):
                _labels = [name for name, _, _ in args.__dict__["_mr_spec"]]
            _ref = _labels[0]
            print(f"[DART-3W] === synced {len(_labels)}-vehicle comparison strategies={_labels} "
                  f"(Δ|err|>0 = reference {_ref} landing attitude better) ===", flush=True)
            for a in args.angles:
                da = data.get(a, {})
                rows_by_lab = {lab: da.get(lab, []) for lab in _labels}
                ref_rows = rows_by_lab[_ref]
                n = len(ref_rows)
                def _m(rows, key, use_abs=False):
                    vals = [(abs(x.get(key)) if (use_abs and isinstance(x.get(key), (int, float))) else x.get(key))
                            for x in rows]
                    return med(vals)
                row = {"angle": a, "n_valid": n, "ref": _ref, "strategies": _labels,
                       "target_pitch": (float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg))
                                        + float(args.landing_flare_deg))}
                for lab in _labels:
                    rows = rows_by_lab[lab]
                    row[f"{lab}_abs_err"] = _m(rows, "land_pitch_error", use_abs=True)
                    row[f"{lab}_roll"] = _m(rows, "land_roll", use_abs=True)
                    row[f"{lab}_pitchrate"] = _m(rows, "max_pitchrate")
                    row[f"{lab}_impact"] = _m(rows, "land_impact_speed_mps")
                    row[f"{lab}_vz"] = _m(rows, "land_vz_mps", use_abs=True)
                    row[f"{lab}_tumble"] = sum(1 for x in rows if x.get("tumbled"))
                    row[f"{lab}_airtime"] = _m(rows, "T_flight")
                def _d(base_key, ref_key):
                    b, c = row.get(base_key), row.get(ref_key)
                    return round(b - c, 2) if (isinstance(b, (int, float)) and isinstance(c, (int, float))) else None
                for lab in _labels:
                    if lab == _ref: continue
                    row[f"d_abs_err_{lab}_vs_{_ref}"] = _d(f"{lab}_abs_err", f"{_ref}_abs_err")
                    row[f"d_roll_{lab}_vs_{_ref}"] = _d(f"{lab}_roll", f"{_ref}_roll")
                row["takeoff_v0_spread_med"] = med([x.get("takeoff_v0_spread") for x in ref_rows])
                row["takeoff_theta0_spread_med"] = med([x.get("takeoff_theta0_spread") for x in ref_rows])
                _g = da.get("reachability_gate")
                if _g is not None:
                    row["gate_enabled"] = _g.get("enabled")
                    row["gate_decision"] = _g.get("decision")
                    row["gate_action"] = _g.get("action")
                    row["gate_v_requested"] = _g.get("v_requested")
                    row["gate_v_effective"] = _g.get("v_effective")
                    row["gate_v_max"] = _g.get("v_max")
                    row["gate_intervened"] = _g.get("intervened")
                summary.append(row)
                _err_str = " ".join(f"{lab}={row[f'{lab}_abs_err']}" for lab in _labels)
                _roll_str = " ".join(f"{lab}={row[f'{lab}_roll']}" for lab in _labels)
                _tum_str = " ".join(f"{lab}={row[f'{lab}_tumble']}/{n}" for lab in _labels)
                _d_str = " ".join(f"Δ{lab}={row.get(f'd_abs_err_{lab}_vs_{_ref}')}" for lab in _labels if lab != _ref)
                _gate_str = (f" | gate={'ON' if row.get('gate_enabled') else 'OFF'} "
                             f"{row.get('gate_decision')}/{row.get('gate_action')} "
                             f"v={row.get('gate_v_requested')}→{row.get('gate_v_effective')}"
                             if da.get("reachability_gate") is not None else "")
                print(f"  θ={a}° N={n} target={row['target_pitch']:.1f}° | |err| {_err_str} ({_d_str}) | "
                      f"|roll| {_roll_str} | tumble {_tum_str} | "
                      f"takeoff spread v0={row['takeoff_v0_spread_med']} θ0={row['takeoff_theta0_spread_med']}"
                      f"{_gate_str}", flush=True)
            ART.mkdir(parents=True, exist_ok=True)
            (ART / f"dart_bench_{args.tag}.json").write_text(
                json.dumps(
                    {"params": vars(args), "mode": "simul_3way", "data": data, "summary": summary,
                     "_provenance": make_provenance_from_data(
                         data, "deterministic_stepped",
                         sps=int(getattr(args, "sim_steps_per_second", 100) or 100))},
                    indent=2, ensure_ascii=False, default=str),
                encoding="utf-8")
            print(f"[CS] saved dart_bench_{args.tag}.json (simul_3way)", flush=True)
            if _cohort_shortfall:
                print(f"[CS] ABORT(exit 7): cohort under target {_cohort_shortfall} — "
                      f"no DONE written, launcher should rerun or manual review", flush=True)
                return 7
            return 0

        print("[DART-CS] === cross-ramp effectiveness curve (Δ>0=C7 improves landing attitude) ===", flush=True)
        summary = []
        for a in args.angles:
            offp = med([x["land_pitch"] for x in data[a]["off"]]); onp = med([x["land_pitch"] for x in data[a]["on"]])
            offe = med([x.get("land_pitch_error") for x in data[a]["off"]])
            one = med([x.get("land_pitch_error") for x in data[a]["on"]])
            offr = med([abs(x["land_roll"]) for x in data[a]["off"] if x["land_roll"] is not None])
            onr = med([abs(x["land_roll"]) for x in data[a]["on"] if x["land_roll"] is not None])
            offt = sum(x["tumbled"] for x in data[a]["off"]); ont = sum(x["tumbled"] for x in data[a]["on"])
            mpr = med([x["max_pitchrate"] for x in data[a]["off"]])
            dpitch = round(abs(offp) - abs(onp), 1) if (offp is not None and onp is not None) else None
            derr = round(abs(offe) - abs(one), 1) if (offe is not None and one is not None) else None
            droll = round(offr - onr, 1) if (offr is not None and onr is not None) else None
            target_pitch = (
                float(args.__dict__.get("_current_landing_slope_deg", args.landing_slope_deg))
                + float(args.landing_flare_deg)
            )
            pitch_metric = derr if abs(target_pitch) > 1e-6 else dpitch
            eff = (pitch_metric is not None and pitch_metric > 2) or (droll is not None and droll > 3) or (ont < offt)
            summary.append({"angle": a, "off_pitch": offp, "on_pitch": onp, "off_roll": offr, "on_roll": onr,
                            "target_pitch": target_pitch, "off_pitch_error": offe,
                            "on_pitch_error": one, "d_pitch_error": derr,
                            "off_front_clearance_m": med([((x.get("front_clearance") or {}).get("clearance_m"))
                                                          for x in data[a]["off"]]),
                            "on_front_clearance_m": med([((x.get("front_clearance") or {}).get("clearance_m"))
                                                         for x in data[a]["on"]]),
                            "off_tumble": offt, "on_tumble": ont, "d_pitch": dpitch, "d_roll": droll,
                            "max_pitchrate": mpr, "dart_effective": eff})
            print(f"  θ={a:>2}° OFF(p={offp},r={offr},tum={offt}) DART(p={onp},r={onr},tum={ont}) "
                  f"target={target_pitch} err({offe}->{one},Δ={derr}) "
                  f"Δ|p|={dpitch} Δ|r|={droll} effective={eff} max_pr={mpr}°/s", flush=True)
        eff_angles = [s["angle"] for s in summary if s["dart_effective"]]
        boundary = next((s["angle"] for s in summary if not s["dart_effective"]), None)
        print(f"[DART-CS] === conclusion === C7 effective ramps={eff_angles}; failure starts at≈{boundary}°", flush=True)
        ART.mkdir(parents=True, exist_ok=True)
        (ART / f"dart_bench_{args.tag}.json").write_text(
            json.dumps(
                {"params": vars(args), "data": data, "summary": summary,
                 "eff_angles": eff_angles, "boundary": boundary,
                 "_provenance": make_provenance_from_data(
                     data, "freerun_wallclock",
                     sps=int(getattr(args, "sim_steps_per_second", 100) or 100))},
                indent=2, ensure_ascii=False, default=str),
            encoding="utf-8")
        print(f"[CS] saved dart_bench_{args.tag}.json", flush=True)
    if _cohort_shortfall:
        print(f"[CS] ABORT(exit 7): cohort under target {_cohort_shortfall} — "
              f"no DONE written, launcher should rerun or manual review", flush=True)
        return 7
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
