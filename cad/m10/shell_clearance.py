"""M10 phase D4 clearance behind the printed walls at the sideways strike spots.

/snap/bin/freecad.cmd -c "p='cad/m10/shell_clearance.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

For each spot in config m10.physics.contact.shell_backing (link-frame point and
inward contact normal, from the sideways falls run without the shell law's
backing), builds cad/m10/assembly.py at the nominal pose, casts a 5 x 5 grid of
rays at +/-6 mm around the spot from the struck part's outer surface along the
inward normal, and reports the gap from the inner face of the struck wall to
the first other part (or the part's own far wall). Rays that start inside the
wall report the wall from the start point, so their wall thickness reads low;
their gaps are still valid. A ray whose struck section is thicker than
SOLID_WALL_MM crosses a solid boss (the knee-stop boss behind the thigh's groove),
which the shell strip law does not describe and which does not deflect by the
travel; it is reported but left out of the minimum. Results in
artifacts/m10/cad/shell_clearance.json (mm); the config's travel_m is the
smallest thin-wall gap per part, rounded down to 0.1 mm.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import Part  # noqa: E402
from FreeCAD import Vector as V  # noqa: E402

from bipe.m10.concept import pose  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "m10_assembly", Path(__file__).with_name("assembly.py")
)
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
C = A.C
OUT = ROOT / "artifacts/m10/cad/shell_clearance.json"
GRID_MM = (-6, -3, 0, 3, 6)
REACH_MM = 40.0
SOLID_WALL_MM = 5.0


def to_assembly(link, v, a1):
    """Link-frame vector (mm) to the assembly frame: thigh links rotate by a1 about y
    at the hip; the body frame is the assembly frame."""
    if not link.endswith("thigh"):
        return V(*v)
    x, y, z = v
    ca, sa = math.cos(a1), math.sin(a1)
    return V(x * ca - z * sa, y, x * sa + z * ca)


def probe(shape, others, point, inward):
    t = inward.cross(V(0, 1, 0))
    if t.Length < 1e-6:
        t = inward.cross(V(1, 0, 0))
    t.normalize()
    u = inward.cross(t)
    u.normalize()
    rays = []
    for du in GRID_MM:
        for dv in GRID_MM:
            s = shape.distToShape(Part.Vertex(point + t * du + u * dv))[1][0][0]
            a, b = s - inward * 1.0, s + inward * REACH_MM
            seg = Part.LineSegment(a, b).toShape()
            hits = []
            for name, sh in [("self", shape)] + list(others.items()):
                for e in seg.common(sh).Edges:
                    p, q = e.Vertexes[0].Point, e.Vertexes[-1].Point
                    hits.append((*sorted(((p - a).Length - 1.0, (q - a).Length - 1.0)), name))
            hits.sort()
            wall = next((h for h in hits if h[2] == "self"), None)
            if wall is None:
                continue
            behind = [h for h in hits if h[0] > wall[1] + 1e-6]
            first = behind[0] if behind else None
            rays.append(
                {
                    "offset_mm": [du, dv],
                    "wall_mm": round(wall[1] - wall[0], 2),
                    "gap_mm": round(first[0] - wall[1], 2) if first else None,
                    "first_obstacle": first[2] if first else None,
                    "solid": wall[1] - wall[0] > SOLID_WALL_MM,
                }
            )
    return rays


def main():
    state = pose(C["poses"]["nominal_hip_height"], C)
    a1 = math.radians(state["a1_deg"])
    parts = A.build(state)
    shapes = {n: s for _, n, s, _ in parts}
    links = {n: lk for lk, n, _, _ in parts}
    backing = C["physics"]["contact"]["shell_backing"]
    out = {
        "status": "Phase D4 clearance behind the sideways strike spots, nominal pose, mm",
        "pose": {k: state[k] for k in ("a1_deg", "a2_deg")},
        "parts": {},
    }
    for name, spec in backing["parts"].items():
        shape, link = shapes[name], links[name]
        others = {n: s for n, s in shapes.items() if n != name}
        spots = []
        for spot, normal in zip(spec["spots_m"], spec["inward_normals"]):
            point = to_assembly(link, [1000 * c for c in spot], a1)
            inward = to_assembly(link, normal, a1)
            inward.normalize()
            rays = probe(shape, others, point, inward)
            gaps = [r["gap_mm"] for r in rays if r["gap_mm"] is not None and not r["solid"]]
            spots.append(
                {
                    "spot_mm": [round(1000 * c, 2) for c in spot],
                    "surface_distance_mm": round(shape.distToShape(Part.Vertex(point))[0], 2),
                    "min_gap_mm": min(gaps) if gaps else None,
                    "obstacles": sorted(
                        {r["first_obstacle"] for r in rays if r["first_obstacle"] and not r["solid"]}
                    ),
                    "solid_rays": sum(r["solid"] for r in rays),
                    "rays": rays,
                }
            )
        gaps = [s["min_gap_mm"] for s in spots if s["min_gap_mm"] is not None]
        out["parts"][name] = {
            "link": link,
            "min_gap_mm": min(gaps) if gaps else None,
            "config_travel_mm": 1000 * spec["travel_m"],
            "spots": spots,
        }
    OUT.write_text(json.dumps(out, indent=2) + "\n")
    print(
        "M10 SHELL CLEARANCE",
        json.dumps({k: [v["min_gap_mm"], v["config_travel_mm"]] for k, v in out["parts"].items()}),
    )


if __name__ == "__main__":
    main()
