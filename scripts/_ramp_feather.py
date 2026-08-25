"""BeamNG ProceduralMesh helpers used by the DART experiment runner.

Places rotated cube segments, steps the sim, and keeps the BeamNG window
in the foreground. Geometry polylines themselves live in
``scripts/_ramp_bench.py`` (``ramp_polyline`` / ``kicker_polyline``).
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

def foreground():
    ps = ('Add-Type @"\nusing System;using System.Runtime.InteropServices;\n'
          'public class W{[DllImport("user32.dll")]public static extern bool SetForegroundWindow(IntPtr h);'
          '[DllImport("user32.dll")]public static extern bool ShowWindow(IntPtr h,int n);}\n"@\n'
          'Get-Process|?{$_.MainWindowTitle -like "*BeamNG*"}|'
          '%{[W]::ShowWindow($_.MainWindowHandle,9);[W]::SetForegroundWindow($_.MainWindowHandle)}')
    try:
        subprocess.run(["powershell", "-NoProfile", "-Command", ps], timeout=15,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

LUA_ENUM = (
    "local out={} "
    "local ok,names=pcall(function() return scenetree.findClassObjects('ProceduralMesh') end) "
    "if ok and names then for _,n in ipairs(names) do "
    "local o=scenetree.findObject(n) "
    "if o then local bb=o:getWorldBox() "
    "out[n]={bb.minExtents.x,bb.minExtents.y,bb.minExtents.z,"
    "bb.maxExtents.x,bb.maxExtents.y,bb.maxExtents.z} end end "
    "else out['_enum_error']=tostring(names) end "
    "return jsonEncode(out)"
)
LUA_DELETE_ALL = (
    "local deleted={} "
    "local ok,names=pcall(function() return scenetree.findClassObjects('ProceduralMesh') end) "
    "if ok and names then for _,n in ipairs(names) do "
    "local o=scenetree.findObject(n) "
    "if o then pcall(function() o:delete() end) deleted[#deleted+1]=n end end end "
    "return jsonEncode(deleted)"
)

def _step(bng, n):
    try:
        bng.control.step(n)
        return True
    except Exception:
        try:
            bng.step(n)
            return True
        except Exception:
            time.sleep(n / 100.0)
            return False

def place_ramp(bng, segs, queue_lua):
    """Delete any existing ProceduralMesh objects, place the segments, return (deleted, placed, enum)."""
    from beamngpy import ProceduralCube  # type: ignore
    try:
        deleted = json.loads(queue_lua(LUA_DELETE_ALL))
    except Exception as exc:  # noqa: BLE001
        deleted = {"error": repr(exc)}
    _step(bng, 3)
    placed = 0
    for s in segs:
        try:
            ProceduralCube(pos=s["pos"], size=s["size"], name=s["name"],
                           rot_quat=s["rot"],
                           material=s.get("material", "track_editor_C_border")).place(bng)
            placed += 1
        except Exception as exc:  # noqa: BLE001
            print(f"place {s['name']} FAILED: {exc!r}", flush=True)
    _step(bng, 3)
    try:
        enum = json.loads(queue_lua(LUA_ENUM))
    except Exception as exc:  # noqa: BLE001
        enum = {"error": repr(exc)}
    return deleted, placed, enum
