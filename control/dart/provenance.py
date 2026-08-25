"""Write ``_provenance`` on cohort JSON so later readers can see the time base."""
from __future__ import annotations

import subprocess
import time
from pathlib import Path

SCHEMA_VERSION = "dart-jump-report/2"
AIRTIME_FIX_COMMIT = "5809bb3"
REPO = Path(__file__).resolve().parents[2]


def git_commit(short: bool = True):
    try:
        args = ["git", "rev-parse", "--short", "HEAD"] if short else ["git", "rev-parse", "HEAD"]
        return subprocess.check_output(args, cwd=str(REPO), stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def make_provenance(dt_mode: str, sps=None, dt_eff=None, note: str | None = None) -> dict:
    ok = False
    if dt_mode == "deterministic_stepped":
        if sps and dt_eff is not None:
            tol = 0.2 * (1.0 / sps)
            if abs(float(dt_eff) - 1.0 / sps) <= tol:
                ok, reason = True, f"deterministic, dt_eff={dt_eff}≈1/{sps}"
            else:
                reason = (
                    f"deterministic dt_eff={dt_eff} deviates from 1/{sps}={1.0/sps:.4f} "
                    f"(tol={tol:.4f})"
                )
        else:
            reason = "deterministic mode but sps/dt_eff missing"
    elif dt_mode == "freerun_wallclock":
        reason = "freerun wall-clock stepping; airtime/rate metrics are not comparable"
    else:
        reason = f"unknown dt_mode={dt_mode!r}"
    prov = {
        "schema_version": SCHEMA_VERSION,
        "code_commit": git_commit(),
        "airtime_fix_commit": AIRTIME_FIX_COMMIT,
        "dt_mode": dt_mode,
        "sps": sps,
        "dt_eff": dt_eff,
        "time_integrity_ok": ok,
        "time_integrity_reason": reason,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    if note:
        prov["note"] = note
    return prov


def collect_field(obj, key):
    out = []

    def walk(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == key and isinstance(v, (int, float)):
                    out.append(v)
                else:
                    walk(v)
        elif isinstance(o, list):
            for x in o:
                walk(x)

    walk(obj)
    return out


def make_provenance_from_data(data, dt_mode: str, sps=None, note: str | None = None) -> dict:
    dts = sorted(collect_field(data, "dt_eff"))
    agg = round(dts[len(dts) // 2], 4) if dts else None
    return make_provenance(dt_mode, sps=sps, dt_eff=agg, note=note)
