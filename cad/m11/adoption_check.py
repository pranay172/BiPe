"""M11 phase D step 5: clearance of the adopted electronics and service loops in the M10 CAD.

/snap/bin/freecad.cmd -c "p='cad/m11/adoption_check.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds cad/m10/assembly.py at the probe hip heights and, for every M11 part (boards,
Teensy, BEC, Pi, switch and the harness service loops), reports overlaps above 1 mm^3
with any other part and the nearest other part with its distance. Loops may touch the
harness bundles they join; those pairs are listed but not counted. Writes
artifacts/m11/packaging/adoption.json.
"""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location("m10_assembly", ROOT / "cad/m10/assembly.py")
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
C = A.C
OUT = ROOT / "artifacts/m11/packaging/adoption.json"
HIP_HEIGHTS_M = [0.06, 0.072, 0.103, 0.135]
M11_PARTS = (
    "CarrierPCB",
    "CarrierParts",
    "Teensy",
    "PowerPCB",
    "PowerParts",
    "BEC",
    "PiZero",
    "PowerSwitch",
    "HarnessLoopBody",
    "HarnessLoopHip",
    "HarnessLoopKnee",
)
NEAR_MM = 5.0


def harness_pair(a, b):
    return "Harness" in a and "Harness" in b


def main():
    poses = {}
    for height in HIP_HEIGHTS_M:
        parts = A.build(pose(height, C))
        rows = {}
        for _, name, shape, _ in parts:
            if not name.endswith(M11_PARTS):
                continue
            box = shape.BoundBox
            box.enlarge(NEAR_MM)
            overlaps, joins, nearest = [], [], (None, NEAR_MM)
            for _, other, oshape, _ in parts:
                if other == name or not box.intersect(oshape.BoundBox):
                    continue
                volume = shape.common(oshape).Volume
                if volume > 1.0:
                    (joins if harness_pair(name, other) else overlaps).append(
                        [other, round(volume, 1)]
                    )
                    continue
                if harness_pair(name, other):
                    continue
                d = shape.distToShape(oshape)[0]
                if d < nearest[1]:
                    nearest = (other, d)
            rows[name] = {
                "overlaps_mm3": overlaps,
                "harness_joins_mm3": joins,
                "nearest": [nearest[0], round(nearest[1], 2)],
            }
        poses[f"{height:.3f}"] = rows
        print("M11 ADOPTION pose", height, flush=True)
    clear = all(not r["overlaps_mm3"] for p in poses.values() for r in p.values())
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        json.dumps(
            {
                "hip_heights_m": HIP_HEIGHTS_M,
                "near_limit_mm": NEAR_MM,
                "note": "nearest is [part, mm] within near_limit_mm, or [null, near_limit_mm]",
                "clear": clear,
                "poses": poses,
            },
            indent=1,
        )
        + "\n"
    )
    print("M11 ADOPTION clear", clear, flush=True)


if __name__ == "__main__":
    main()
