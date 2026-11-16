"""List the parts inside the M10 body interior at a pose, with body-frame bounding boxes (mm).

/snap/bin/freecad.cmd -c "p='cad/m11/interior_map.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"
"""

import importlib.util
import json
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

if __name__ == "__main__":
    name = os.environ.get("M11_POSE", "nominal")
    state = pose(C["poses"][f"{name}_hip_height"], C)
    parts = A.build(state)
    box = A.interior_box().BoundBox
    rows = []
    for link, part, shape, _ in parts:
        b = shape.BoundBox
        if b.intersect(box) and part not in ("BodyCover", "BodyTub"):
            rows.append(
                [link, part]
                + [round(v, 1) for v in (b.XMin, b.XMax, b.YMin, b.YMax, b.ZMin, b.ZMax)]
            )
    out = ROOT / "artifacts/m11/packaging"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"interior_{name}.json").write_text(
        json.dumps(
            {
                "pose": name,
                "interior_mm": [
                    round(v, 1)
                    for v in (box.XMin, box.XMax, box.YMin, box.YMax, box.ZMin, box.ZMax)
                ],
                "parts": rows,
            },
            indent=1,
        )
    )
    print("M11 INTERIOR", name, len(rows), flush=True)
