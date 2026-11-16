"""M10 phase D section properties of the printed structural parts.

/snap/bin/freecad.cmd -c "p='cad/m10/sections.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds cad/m10/assembly.py at the nominal pose and slices the printed parts
that carry fall loads, normal to their load axis:
- Thigh (left; the right is mirrored) from the hip to the knee, along the link
  x axis (hip -> knee);
- Shin from the knee to the axle;
- HipBracket on planes of constant x between its hub and its bulkhead arms;
- Neck on planes of constant z between the body and the head.
Each slice gives the section area, centroid and the second moments about the
section's centroidal axes in the link frame: I_y about the joint axis
(sagittal bending), I_n about the in-plane normal (lateral bending), the polar
moment, and the extreme-fibre distances. Results are SI, in
artifacts/m10/cad/sections.json. Thin closed rings are flagged; open sections
must not be credited with the polar moment for torsion.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

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
OUT = ROOT / "artifacts/m10/cad/sections.json"


def part(parts, link, name):
    shapes = [s for lk, n, s, _ in parts if lk == link and n in (name, f"left_{name}")]
    if not shapes:
        raise KeyError(f"{link}/{name} not in the assembly")
    shape = shapes[0]
    for s in shapes[1:]:
        shape = shape.fuse(s)
    return shape


def section(shape, origin, axis, e_a, e_b):
    """Section of shape on the plane through origin (mm) normal to axis.

    e_a, e_b: in-plane unit vectors (link frame). Returns SI properties with
    second moments about the centroidal e_a and e_b axes.
    """
    wires = shape.slice(V(*axis), float(np.dot(axis, origin)))
    if not wires:
        return None
    faces = Part.makeFace(wires, "Part::FaceMakerBullseye").Faces
    area = sum(f.Area for f in faces)
    if area <= 0:
        return None
    pts = np.array([[v.x, v.y, v.z] for w in wires for v in w.discretize(Deflection=0.02)])
    cents = [np.array([f.CenterOfMass.x, f.CenterOfMass.y, f.CenterOfMass.z]) for f in faces]
    c = sum(f.Area * k for f, k in zip(faces, cents)) / area
    inertia = np.zeros((3, 3))
    for f, k in zip(faces, cents):
        m = f.MatrixOfInertia
        d = k - c
        inertia += np.array([[m.A[r * 4 + j] for j in range(3)] for r in range(3)]) + f.Area * (
            np.dot(d, d) * np.eye(3) - np.outer(d, d)
        )
    # MatrixOfInertia is the unit-density inertia tensor about the centroid:
    # the moment about an in-plane axis u is u.I.u = integral of (distance to u)^2.
    i_about_a = float(e_a @ inertia @ e_a)
    i_about_b = float(e_b @ inertia @ e_b)
    rel = pts - c
    # Single-cell closed sections (Bredt): area enclosed by the wall mid-line, over the
    # faces that enclose a hole (two loops of separate faces are not a closed cell).
    cells = [f for f in faces if len(f.Wires) >= 2]
    enclosed_mid = sum(Part.Face(f.OuterWire).Area - f.Area / 2 for f in cells)
    return {
        "enclosed_mid_area_m2": enclosed_mid * 1e-6,
        "area_m2": area * 1e-6,
        "faces": len(faces),
        "centroid_hip_m": (c * 1e-3).tolist(),
        "I_about_a_m4": i_about_a * 1e-12,
        "I_about_b_m4": i_about_b * 1e-12,
        "polar_m4": (i_about_a + i_about_b) * 1e-12,
        "extreme_along_a_m": float(np.max(np.abs(rel @ e_a))) * 1e-3,
        "extreme_along_b_m": float(np.max(np.abs(rel @ e_b))) * 1e-3,
        "radius_max_m": float(np.max(np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1)))
        * 1e-3,
        "loops": len(wires),
        "closed_ring": bool(cells),
    }


def link_stations(shape, origin_mm, angle, length_mm, fractions):
    e1 = np.array([math.cos(angle), 0.0, math.sin(angle)])
    ey = np.array([0.0, 1.0, 0.0])
    en = np.array([-math.sin(angle), 0.0, math.cos(angle)])
    rows = []
    for f in fractions:
        point = np.array([origin_mm[0], 0.0, origin_mm[1]]) + f * length_mm * e1
        # e_a = joint axis y (sagittal bending), e_b = in-plane normal (lateral bending).
        props = section(shape, point, e1, ey, en)
        if props:
            # About y: distance measured along en; about en: distance along y.
            rows.append(
                {
                    "station_m": f * length_mm * 1e-3,
                    "fraction": f,
                    **props,
                    "Z_sagittal_m3": props["I_about_a_m4"] / props["extreme_along_b_m"],
                    "Z_lateral_m3": props["I_about_b_m4"] / props["extreme_along_a_m"],
                }
            )
    return rows


def main():
    state = pose(C["poses"]["nominal_hip_height"], C)
    parts = A.build(state)
    a1, a2 = math.radians(state["a1_deg"]), math.radians(state["a2_deg"])
    knee = np.array(state["knee_m"]) * 1000
    leg = C["leg"]
    # The last station sits 1 mm outside the knee bearing bore (B3: 0.88 with the 15 mm
    # 6700 bore; phase D4: 0.85 with the 19 mm 6800 bore).
    knee_od = A.INT["bearings"][A.BRG["KneeInnerBearing"]][1]
    last = round(1 - (knee_od / 2 + 0.001) / leg["thigh_length"], 2)
    fractions = [0.12, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, last]
    out = {
        "status": "Phase D section properties of the D4 CAD (B3 assembly with the protection set) at "
        "the nominal pose; uncut by holes "
        "not modelled in B3 (screw holes are modelled; panel pockets are)",
        "frames": "Thigh and shin: plane normal to the link x axis; a = joint axis y "
        "(sagittal bending, Z_sagittal), b = in-plane normal (lateral bending, Z_lateral). "
        "HipBracket: planes of constant x, a = y, b = z. Neck: planes of constant z, a = x, b = y.",
        "pose": {k: state[k] for k in ("a1_deg", "a2_deg", "knee_m", "axle_m")},
        "thigh": link_stations(
            part(parts, "left_thigh", "Thigh"), [0.0, 0.0], a1, leg["thigh_length"] * 1000,
            fractions,
        ),
        # Thigh with its lid as a closed tube: valid only if the lid is fastened along the
        # rim to carry the shear flow (a Phase D remedy candidate).
        "thigh_closed": link_stations(
            part(parts, "left_thigh", "Thigh").fuse(part(parts, "left_thigh", "ThighLid")),
            [0.0, 0.0], a1, leg["thigh_length"] * 1000, fractions,
        ),
        # Torsion cells with the 7075 hip flange (phase D4) screwed to the inner wall: it
        # bridges the outer-shaft bore that opens the hip-end slices. Only closed_ring and
        # enclosed_mid_area_m2 are used (Bredt); bending stays the PA12 thigh_closed rows.
        "thigh_torsion": link_stations(
            part(parts, "left_thigh", "Thigh")
            .fuse(part(parts, "left_thigh", "ThighLid"))
            .fuse(part(parts, "left_thigh", "HipFlange")),
            [0.0, 0.0], a1, leg["thigh_length"] * 1000, fractions,
        ),
        "shin": link_stations(
            part(parts, "left_shin", "Shin"), knee, a2, leg["shin_length"] * 1000, fractions
        ),
    }
    bracket = part(parts, "body", "HipBracket")
    bb = bracket.BoundBox
    out["hip_bracket"] = []
    for x in np.linspace(bb.XMin + 1.0, bb.XMax - 1.0, 9):
        props = section(
            bracket, np.array([x, 0, 0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
            np.array([0, 0, 1.0]),
        )
        if props:
            out["hip_bracket"].append({"x_m": x * 1e-3, **props})
    neck = part(parts, "body", "Neck")
    nb = neck.BoundBox
    out["neck"] = []
    for z in np.linspace(nb.ZMin + 0.5, nb.ZMax - 0.5, 7):
        props = section(
            neck, np.array([0, 0, z]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0]),
            np.array([0, 1.0, 0]),
        )
        if props:
            out["neck"].append({"z_m": z * 1e-3, **props})
    # Stations of the load-path parts (left side), hip frame at the nominal pose.
    keys = [
        "BulkheadBearing", "InnerShaftEndBearing", "HipInnerBearing", "HipPodBearing",
        "KneeInnerBearing", "KneeOuterBearing", "InnerShaft", "InnerShaftEnds", "OuterShaft",
        "KneeShaft", "AxleTube", "HipFlange", "ThighHipPulley", "KneeHipPulley", "KneeBeltHipPulley",
        "KneePulley", "KneeBelt", "HipBracket", "Bulkhead", "Shin", "Thigh", "Tire",
        "HubMotorStator", "HubMotorRotor", "HubDisc",
    ]
    out["stations"] = {}
    for link, name, shape, _ in parts:
        key = name[5:] if name.startswith("left_") else name
        if key not in keys or name.startswith("right_") or (link.startswith("right_")):
            continue
        b = shape.BoundBox
        out["stations"].setdefault(key, []).append(
            {
                "link": link,
                "x_m": [b.XMin * 1e-3, b.XMax * 1e-3],
                "y_m": [b.YMin * 1e-3, b.YMax * 1e-3],
                "z_m": [b.ZMin * 1e-3, b.ZMax * 1e-3],
                "volume_m3": shape.Volume * 1e-9,
            }
        )
    out["all_part_names"] = sorted({n for _, n, _, _ in parts if not n.startswith("right_")})
    OUT.write_text(json.dumps(out, indent=2) + "\n")
    print("M10 SECTIONS", {k: len(v) for k, v in out.items() if isinstance(v, list)}, flush=True)


if __name__ in ("__main__", "bridge_main"):
    main()
