"""M11 phase D: harness and shaft geometry from the M10 assembly at one pose.

M11_POSE=0.103 /snap/bin/freecad.cmd -c "p='cad/m11/harness_geometry.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Writes artifacts/m11/harness/geometry_<pose>.json: the bounding box (link frame, mm),
volume and, for harness bundles, the length along the bundle (volume over the
bundle cross-section) of each harness, shaft and port-related part.
"""

import importlib.util
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location("m10_assembly", ROOT / "cad/m10/assembly.py")
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
C = A.C
KEYS = ("Harness", "Shaft", "Axle", "Gallery", "Port")


def main():
    height = float(os.environ.get("M11_POSE", "0.103"))
    bundle_d = C["internals"]["harness"]["bundle_diameter"] * 1000
    area = math.pi * bundle_d**2 / 4
    rows = []
    for link, part, shape, _ in A.build(pose(height, C)):
        if not any(k in part for k in KEYS):
            continue
        bb = shape.BoundBox
        row = {
            "link": link,
            "part": part,
            "bbox_mm": [
                round(v, 2) for v in (bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax)
            ],
            "volume_mm3": round(shape.Volume, 1),
        }
        if "Harness" in part:
            row["length_mm"] = round(shape.Volume / area, 1)
        rows.append(row)
        print("M11 HARNESS", link, part, row["bbox_mm"], row.get("length_mm", ""), flush=True)
    out = ROOT / f"artifacts/m11/harness/geometry_{height:.3f}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {"hip_height_m": height, "bundle_diameter_mm": bundle_d, "parts": rows}, indent=1
        )
    )


if __name__ == "__main__":
    main()
