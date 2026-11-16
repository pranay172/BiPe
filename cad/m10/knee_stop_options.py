"""M10 phase D4 knee-angle stop options: look study for the user (13 October 2026).

Historical: the user chose option B, which now lives in cad/m10/exterior.py
(m10.internals.knee_stop). This script was run against the pre-adoption exterior
(19 mm knee end, no stop); rerunning it on the adopted exterior would double-cut the
groove. Its renders are kept in artifacts/m10/knee_stop/ as the decision record.

/snap/bin/freecad.cmd -c "p='cad/m10/knee_stop_options.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

The falls use a knee-angle stop pad of TPU 88A, 12 mm long, 24 mm2, at 16 mm
radius (m10.physics.end_stop_pads.joints.knee_angle). Neither range end fits
inside the B3 knee (19 mm knee-end radius with a 5 mm edge fillet). Two
geometric options are built on the D4 exterior for review:

A  knuckles: a PA12 knuckle with a TPU pad at each range end, struck by the shin
   flank 16 mm from the flank's tangent point (16 mm moment arm). The folding
   stop sits on the thigh face; the extension stop hangs off the knee end.
B  arc slot: a pin on the shin's inner face runs in a groove in the thigh's outer
   face with a TPU pad at each end (6 x 4 mm face, 12 mm long, at 14.5 mm
   radius). The thigh's knee end grows from 19 to 23 mm radius.

Each option is built at the nominal pose and at both stops (knee angle 75 and
176 deg, thigh at its nominal angle, wheel on the floor) and written with its
review camera views to artifacts/m10/knee_stop/<option>/render-mesh.json.gz
for cad/m10/render_blender.py (--arg stage=knee_stop/<option>).
"""

import copy
import gzip
import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import Part

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location(
    "m10_exterior", Path(__file__).with_name("exterior.py")
)
E = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(E)

C, mm, V = E.C, E.mm, E.V
OUT = ROOT / "artifacts/m10/knee_stop"
E.COLORS.update({"stop_pad": (0.80, 0.30, 0.16), "metal": (0.70, 0.71, 0.73)})
PAD = C["physics"]["end_stop_pads"]["joints"]["knee_angle"]
STOPS = C["physics"]["end_stops_deg"]["knee_angle"]  # [extended, folded]
CLEAR = 0.3  # mm, pad and knuckle clearance to the shin faces


def leg_state(knee_angle):
    """Nominal thigh and hip height, shin at the given knee angle (the wheel leaves the floor)."""
    base = pose(C["poses"]["nominal_hip_height"], C)
    a1 = base["a1_deg"]
    a2 = a1 + knee_angle
    kx, kz = base["knee_m"]
    ls = C["leg"]["shin_length"]
    axle = [kx + ls * math.cos(math.radians(a2)), kz + ls * math.sin(math.radians(a2))]
    return {
        **base,
        "a2_deg": a2,
        "knee_angle_deg": knee_angle,
        "axle_m": axle,
        "axle_m_nominal": base["axle_m"],
    }


def prism(points, y0, y1):
    """Polygon in the x-z plane (mm), extruded from y0 to y1."""
    lo, hi = min(y0, y1), max(y0, y1)
    poly = Part.makePolygon([V(x, lo, z) for x, z in points] + [V(points[0][0], lo, points[0][1])])
    return Part.Face(poly).extrude(V(0, hi - lo, 0))


def hull(points):
    """2D convex hull (monotone chain), counter-clockwise."""
    pts = sorted(set(points))

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (ax, az), (bx, bz) = out[-2], out[-1]
                if (bx - ax) * (p[1] - az) - (bz - az) * (p[0] - ax) > 0:
                    break
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(reversed(pts))
    return lower[:-1] + upper[:-1]


def mirror(shape):
    return shape.mirror(V(0, 0, 0), V(0, 1, 0))


def option_a(state):
    """Knuckle stops: a TPU pad ahead of the leading shin flank at each range end."""
    a1 = math.radians(state["a1_deg"])
    kx, kz = (mm(v) for v in state["knee_m"])
    rk, ra = mm(C["exterior"]["shin"]["knee_radius"]), mm(C["exterior"]["shin"]["axle_radius"])
    alpha = math.acos((rk - ra) / mm(C["leg"]["shin_length"]))
    t1 = mm(C["leg"]["thigh_y"][1])
    s0, s1 = (mm(v) for v in C["leg"]["shin_y"])
    arm, length = mm(PAD["radius_m"]), mm(PAD["thickness_m"])
    width = PAD["area_m2"] * 1e6 / (s1 - s0 - 2 * CLEAR)  # mm along the flank
    backing = mm(C["physics"]["end_stop_pads"]["backing_thickness_m"]) + 1.0
    parts = []
    for knee_angle, sense, tag in ((STOPS[0], -1, "Extend"), (STOPS[1], 1, "Fold")):
        phi = a1 + math.radians(knee_angle)
        # Leading flank: the one moving towards the stop (sense = direction of travel).
        a = phi + sense * alpha
        n = (math.cos(a), math.sin(a))
        # Along the (tapered) flank from its tangent point, away from the knee.
        d = (-n[1], n[0]) if -n[1] * math.cos(phi) + n[0] * math.sin(phi) > 0 else (n[1], -n[0])
        p = (kx + rk * n[0] + arm * d[0], kz + rk * n[1] + arm * d[1])

        def at(u, w, p=p, d=d, n=n):
            return (p[0] + u * d[0] + w * n[0], p[1] + u * d[1] + w * n[1])

        pad = [at(-width / 2, 0), at(width / 2, 0), at(width / 2, length), at(-width / 2, length)]
        back = [
            at(-width / 2 - 1.5, length),
            at(width / 2 + 1.5, length),
            at(width / 2 + 1.5, length + backing),
            at(-width / 2 - 1.5, length + backing),
        ]
        # Root: the knuckle's footprint joined to a pad of thigh face 13 mm from the knee.
        r_root = 13.0
        c = math.atan2(p[1] + n[1] * length / 2 - kz, p[0] + n[0] * length / 2 - kx)
        anchor = [
            (kx + r_root * math.cos(c + k * 0.15), kz + r_root * math.sin(c + k * 0.15))
            for k in range(-2, 3)
        ]
        root = prism(
            hull(
                back
                + anchor
                + [at(-width / 2 - 1.5, length * 0.4), at(width / 2 + 1.5, length * 0.4)]
            ),
            t1 - 3.0,
            t1,
        )
        knuckle = root.fuse(prism(back, t1 - 3.0, s1 - CLEAR)).removeSplitter()
        knuckle = E.soft(knuckle, 0.6)
        parts += [
            ("left_thigh", f"left_StopKnuckle{tag}", knuckle, "shell"),
            ("left_thigh", f"left_StopPad{tag}", prism(pad, s0 + CLEAR, s1 - CLEAR), "stop_pad"),
        ]
    parts += [
        (lk.replace("left", "right"), n.replace("left", "right"), mirror(sh), col)
        for lk, n, sh, col in parts
    ]
    return parts, {}


def option_b(state):
    """Arc-slot stop: shin pin in a padded groove in the thigh's outer face."""
    a1 = math.radians(state["a1_deg"])
    kx, kz = (mm(v) for v in state["knee_m"])
    t1 = mm(C["leg"]["thigh_y"][1])
    s0 = mm(C["leg"]["shin_y"][0])
    r_pin, pin_d, depth, radial = 14.5, 5.0, 4.0, 6.0
    length = mm(PAD["thickness_m"])
    r0, r1 = r_pin - radial / 2, r_pin + radial / 2
    half_pin = math.asin(pin_d / 2 / r_pin)
    pad_angle = length / r_pin
    lo = a1 + math.radians(STOPS[0]) - half_pin - pad_angle
    hi = a1 + math.radians(STOPS[1]) + half_pin + pad_angle

    def sector(b0, b1, y0, y1, steps=48):
        n = max(4, int(steps * (b1 - b0) / math.pi))
        arc = [b0 + (b1 - b0) * k / n for k in range(n + 1)]
        pts = [(kx + r1 * math.cos(b), kz + r1 * math.sin(b)) for b in arc]
        pts += [(kx + r0 * math.cos(b), kz + r0 * math.sin(b)) for b in reversed(arc)]
        return prism(pts, y0, y1)

    groove = sector(lo, hi, t1 - depth, t1 + 1)
    pads = [
        sector(lo, lo + pad_angle, t1 - depth, t1 - 0.2),
        sector(hi - pad_angle, hi, t1 - depth, t1 - 0.2),
    ]
    phi = a1 + math.radians(state["knee_angle_deg"])
    pin = Part.makeCylinder(
        pin_d / 2,
        depth - 0.4 + 1.0,
        V(kx + r_pin * math.cos(phi), s0 + 1.0, kz + r_pin * math.sin(phi)),
        V(0, -1, 0),
    )
    parts = [
        ("left_thigh", "left_StopPad", pads[0].fuse(pads[1]), "stop_pad"),
        ("left_shin", "left_StopPin", pin, "metal"),
    ]
    parts += [
        (lk.replace("left", "right"), n.replace("left", "right"), mirror(sh), col)
        for lk, n, sh, col in parts
    ]
    return parts, {"left_Thigh": groove, "right_Thigh": mirror(groove)}


OPTIONS = {
    "current": (None, {}),
    "A_knuckles": (option_a, {}),
    "B_arc_slot": (option_b, {"knee_radius": 0.023}),
}
POSES = {"nominal": None, "extended_stop": STOPS[0], "folded_stop": STOPS[1]}


def build(option, knee_angle):
    builder, thigh_patch = OPTIONS[option]
    saved = copy.deepcopy(E.X["thigh"])
    E.X["thigh"].update(thigh_patch)
    try:
        state = pose(C["poses"]["nominal_hip_height"], C)
        if knee_angle is not None:
            state = leg_state(knee_angle)
        parts = E.build(state)
        if builder is not None:
            extra, cuts = builder(state)
            parts = [(lk, n, sh.cut(cuts[n]) if n in cuts else sh, col) for lk, n, sh, col in parts]
            parts += extra
    finally:
        E.X["thigh"].clear()
        E.X["thigh"].update(saved)
    return state, E.to_world(state, parts)


def checks(option):
    """Stop contact: pad-to-striker gap (mm) and overlap volume (mm3) at each pose."""
    if option == "current":
        return {}
    striker = {"A_knuckles": "left_Shin", "B_arc_slot": "left_StopPin"}[option]
    out = {}
    for name, knee_angle in POSES.items():
        _, parts = build(option, knee_angle)
        shapes = {n: sh for _, n, sh, _ in parts}
        hit = shapes[striker]
        row = {}
        for part, shape in shapes.items():
            if part.startswith("left_Stop") and part != striker:
                row[part] = {
                    "gap_mm": round(shape.distToShape(hit)[0], 2),
                    "overlap_mm3": round(shape.common(hit).Volume, 2),
                }
        out[name] = row
    print("M10 KNEE STOP CHECK", option, json.dumps(out), flush=True)
    return out


WHEEL = ["left_Tire", "left_Tread", "left_HubDisc", "left_HubBlade", "left_HubCap"]


def views(states):
    """Close-ups of the left knee from outboard at each pose, and a rear three-quarter.

    The stop poses hold the nominal hip height, so they hide the floor and the near
    wheel (which then sits over the thigh) to show the knee.
    """
    out = []
    for name, state in states.items():
        kx, kz = state["knee_m"]
        target = [kx - state["axle_m"][0], C["leg"]["shin_y"][1], kz + state["hip_height_m"]]
        stop = name != "nominal"
        out.append(
            {
                "name": f"{name}_side",
                "pose": name,
                "direction": [0.05, 1.0, 0.12],
                "target": target,
                "lens": 70,
                "size": 0.12,
                "floor": not stop,
                "hide": WHEEL if stop else [],
            }
        )
        if name == "nominal":
            out.append(
                {
                    "name": "nominal_rear_three_quarter",
                    "pose": name,
                    "direction": [-0.8, 1.0, 0.35],
                    "target": target,
                    "lens": 70,
                    "size": 0.14,
                }
            )
    return out


def main():
    for option in OPTIONS:
        record = {"poses": {}, "units": "m", "colors": {k: list(v) for k, v in E.COLORS.items()}}
        states = {}
        for name, knee_angle in POSES.items():
            state, parts = build(option, knee_angle)
            states[name] = state
            comps = []
            for link, part, shape, color in parts:
                pts, tris = shape.tessellate(0.1)
                comps.append(
                    {
                        "name": part,
                        "link": link,
                        "color": list(E.COLORS[color]),
                        "color_key": color,
                        "vertices": [
                            [round(p.x / E.MM, 6), round(p.y / E.MM, 6), round(p.z / E.MM, 6)]
                            for p in pts
                        ],
                        "triangles": [list(t) for t in tris],
                    }
                )
            record["poses"][name] = comps
        record["views"] = views(states)
        record["checks"] = checks(option)
        record["status"] = "Phase D4 knee-angle stop look study (not adopted)"
        folder = OUT / option
        folder.mkdir(parents=True, exist_ok=True)
        with gzip.open(folder / "render-mesh.json.gz", "wt") as f:
            json.dump(record, f)
        print("M10 KNEE STOP", option, list(record["poses"]), flush=True)


if __name__ in ("__main__", "bridge_main"):
    main()
