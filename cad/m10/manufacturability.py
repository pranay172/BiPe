"""M10 phase B3 manufacturability screen: measured wall thickness of printed parts.

/snap/bin/freecad.cmd -c "p='cad/m10/manufacturability.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds cad/m10/assembly.py at the nominal pose and, for every printed part
(PA12 or TPU in m10.mass_properties.part_material), casts rays inward along
the surface normal from points sampled on each face. The length of the
solid run that starts at the sample point is the local wall thickness.
Reports the minimum, 5th percentile and the count below the screen
threshold. Writes artifacts/m10/cad/manufacturability.json.
"""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import Part

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location(
    "m10_assembly", Path(__file__).with_name("assembly.py")
)
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

C, V = A.C, A.V
MP = C["mass_properties"]
OUT = ROOT / "artifacts/m10/cad"
PRINTED = {"pa12", "tpu"}
# SLS PA12/TPU minimum wall guidance is about 0.8-1.0 mm; screen at 1.0 mm (assumption).
THRESHOLD_MM = 1.0
GRID = 5  # samples per face direction
MAX_RAY_MM = 40.0


def base_name(name):
    for side in ("left_", "right_"):
        if name.startswith(side):
            return name[len(side) :]
    return name


def thickness_samples(shape):
    out = []
    for face in shape.Faces:
        u0, u1, v0, v1 = face.ParameterRange
        for i in range(GRID):
            for j in range(GRID):
                u = u0 + (u1 - u0) * (i + 0.5) / GRID
                v = v0 + (v1 - v0) * (j + 0.5) / GRID
                p = face.valueAt(u, v)
                if not face.isInside(p, 1e-6, True):
                    continue
                n = face.normalAt(u, v)  # outward for a solid's face
                start = p - n * 1e-3
                ray = Part.makeLine(start, p - n * MAX_RAY_MM)
                inside = shape.common(ray)
                runs = [
                    e
                    for e in inside.Edges
                    if min(
                        e.Vertexes[0].Point.distanceToPoint(start),
                        e.Vertexes[-1].Point.distanceToPoint(start),
                    )
                    < 0.05
                ]
                if runs:
                    out.append(runs[0].Length)
    return out


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    state = pose(C["poses"]["nominal_hip_height"], C)
    seen = {}
    for _, name, shape, _ in A.build(state):
        key = base_name(name)
        material = MP["part_material"].get(key)
        if material not in PRINTED or key in seen or name.startswith("right_"):
            continue
        t = np.array(thickness_samples(shape))
        if t.size == 0:
            continue
        seen[key] = {
            "material": material,
            "samples": int(t.size),
            "min_mm": round(float(t.min()), 2),
            "p05_mm": round(float(np.percentile(t, 5)), 2),
            "median_mm": round(float(np.median(t)), 2),
            "below_threshold": int((t < THRESHOLD_MM).sum()),
        }
        print("M10 WALL", key, json.dumps(seen[key]), flush=True)
    record = {
        "basis": "Inward normal rays from a 5x5 parameter grid per face on the nominal assembly; thin features such as fillet lips and split-line edges register as short runs",
        "threshold_mm": THRESHOLD_MM,
        "threshold_basis": "SLS PA12/TPU minimum wall guidance about 0.8-1.0 mm (assumption)",
        "parts": seen,
    }
    (OUT / "manufacturability.json").write_text(json.dumps(record, indent=2) + "\n")
    return record


if __name__ in ("__main__", "bridge_main"):
    run()
