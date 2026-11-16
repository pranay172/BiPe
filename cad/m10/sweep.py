"""M10 phase B3 pose sweep of the detailed assembly.

/snap/bin/freecad.cmd -c "p='cad/m10/sweep.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Rebuilds cad/m10/assembly.py at every pose of a grid (hip height x axle
fore-aft offset from the balance point) and searches the sit pose: the
lowest balanced hip height whose folded legs still fit, down to the body
resting on the floor. Checks per pose: no cross-link overlap above 1 mm3,
nothing but the tire below the ground, knee behind the hip-axle line.
Writes artifacts/m10/cad/sweep.json with the joint ranges covered.
"""

import importlib.util
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from bipe.m10.concept import pose_at

_spec = importlib.util.spec_from_file_location(
    "m10_assembly", Path(__file__).with_name("assembly.py")
)
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

C, E = A.C, A.E
OUT = ROOT / "artifacts/m10/cad"
HEIGHTS = [round(0.072 + i * (0.135 - 0.072) / 7, 4) for i in range(8)]
OFFSETS = [-0.010, -0.005, 0.0, 0.005, 0.010]
SIT_STEP = 0.003
GROUND_PARTS = ("Tire", "Tread")
# The skid pad (phase D4) is meant to rest on the floor: the sit pose is the lowest pose
# with the skid at or above the floor; the fold search below it ignores the floor for
# the skid alone, so the joint ranges cover the leg fold down to the body floor.
SKID_PARTS = ("SkidPad", "SkidSkin")


def knee_behind(state):
    """Signed distance (m) of the knee behind the hip-axle line; positive is behind."""
    kx, kz = state["knee_m"]
    ax, az = state["axle_m"]
    d = math.hypot(ax, az)
    ux, uz = ax / d, az / d
    along = kx * ux + kz * uz
    return -(kx - along * ux)


def check(state):
    parts = E.to_world(state, A.build(state))
    lowest = min(
        (shape.BoundBox.ZMin, name)
        for _, name, shape, _ in parts
        if not name.endswith(GROUND_PARTS + SKID_PARTS)
    )
    skid = min(
        (shape.BoundBox.ZMin for _, name, shape, _ in parts if name in SKID_PARTS),
        default=None,
    )
    return {
        "hip_height_m": state["hip_height_m"],
        "axle_offset_m": state["axle_offset_m"],
        "a1_deg": round(state["a1_deg"], 2),
        "a2_deg": round(state["a2_deg"], 2),
        "knee_angle_deg": round(state["knee_angle_deg"], 2),
        "knee_behind_m": round(knee_behind(state), 4),
        "lowest_non_tire_mm": [round(lowest[0], 2), lowest[1]],
        "skid_lowest_mm": None if skid is None else round(skid, 2),
        "cross_link_overlaps_mm3": E.overlaps(parts),
    }


def clean(row):
    return (
        not row["cross_link_overlaps_mm3"]
        and row["lowest_non_tire_mm"][0] >= -0.01
        and row["knee_behind_m"] > 0
    )


def skid_clear(row):
    return row["skid_lowest_mm"] is None or row["skid_lowest_mm"] >= -0.01


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    grid = []
    for h in HEIGHTS:
        for dx in OFFSETS:
            row = check(pose_at(h, dx, C))
            row["clean"] = clean(row) and skid_clear(row)
            grid.append(row)
            print("M10 SWEEP", h, dx, row["clean"], row["cross_link_overlaps_mm3"], flush=True)
    sit = []
    floor = C["body"]["hip_z_above_bottom"]
    h = HEIGHTS[0] - SIT_STEP
    while h >= floor - 1e-9:
        try:
            state = pose_at(round(h, 4), 0.0, C)
        except ValueError as err:
            sit.append({"hip_height_m": round(h, 4), "error": str(err), "clean": False})
            break
        row = check(state)
        row["clean"] = clean(row)
        sit.append(row)
        print("M10 SIT", round(h, 4), row["clean"], row["cross_link_overlaps_mm3"], flush=True)
        h -= SIT_STEP
    sit_clean = [r for r in sit if r["clean"] and skid_clear(r)]
    covered = [r for r in grid + sit if r["clean"]]
    ranges = {
        key: [min(r[key] for r in covered), max(r[key] for r in covered)]
        for key in ("a1_deg", "a2_deg", "knee_angle_deg")
    }
    record = {
        "basis": "Rigid CAD from cad/m10/assembly.py rebuilt per pose; overlaps > 1 mm3 between different links. Sit pose: lowest clean pose with the skid pad at or above the floor; the fold search below it ignores the floor for the skid only, so the joint ranges cover the leg fold down to the body floor",
        "heights_m": HEIGHTS,
        "axle_offsets_m": OFFSETS,
        "grid_clean": all(r["clean"] for r in grid),
        "sit_pose_hip_height_m": min((r["hip_height_m"] for r in sit_clean), default=None),
        "joint_ranges_covered_deg": ranges,
        "grid": grid,
        "sit_search": sit,
        "seconds": round(time.time() - t0, 1),
    }
    (OUT / "sweep.json").write_text(json.dumps(record, indent=2) + "\n")
    print(
        "M10 SWEEP DONE",
        json.dumps(
            {
                k: record[k]
                for k in (
                    "grid_clean",
                    "sit_pose_hip_height_m",
                    "joint_ranges_covered_deg",
                    "seconds",
                )
            }
        ),
        flush=True,
    )
    return record


if __name__ in ("__main__", "bridge_main"):
    run()
