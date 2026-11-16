"""M10 phase D4 hip (thigh_a1) and knee-drive (knee_drive_a2) end stop options: look study.

Historical: the user chose option A (18 October 2026), which now lives in
cad/m10/assembly.py (m10.internals.end_stops) with one change: the hip dowel moved
from the PA12 tub wall to the 7075 hip bracket arm (groove in the thigh pulley's
inboard face). This script was run against the pre-adoption assembly; rerunning it
on the adopted one would double-cut the grooves. Its renders and checks are kept in
artifacts/m10/end_stop/ as the decision record.

/snap/bin/freecad.cmd -c "p='cad/m10/end_stop_options.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

The falls use TPU 88A stop pads on both joints (m10.physics.end_stop_pads.joints),
modelled in the plant only. Two options are built on the D4 assembly for review:

A  hidden: a steel ISO 8734 dowel fixed to the body runs in an arc groove in the
   face of the joint's 80T hip pulley, with a TPU pad at each groove end (6 mm
   radial, 8 mm long, at 20 mm radius).
   Hip: Ø5 x 8 dowel pressed through the body side wall into the hip housing
   (1 mm skin), groove in the outboard face of the thigh 80T pulley.
   Knee drive: one Ø5 x 16 dowel through the bulkhead; its ends run in grooves
   in the inboard faces of both knee 80T pulleys.
B  visible hip: a PA12 lug with a TPU face on the hip housing's outer face at
   each range end, struck by the thigh's flank about 17 mm from the flank's
   tangent point. The knee drive is as in A.

Each option is built at the nominal pose and at both stop poses (both joints at
their low or high stop together, nominal hip height, wheel off the floor) and
written with review camera views to artifacts/m10/end_stop/<option>/render-mesh.json.gz
for cad/m10/render_blender.py (--arg stage=end_stop/<option>).
"""

import gzip
import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import Part

from bipe.m10.concept import leg_points, pose

_spec = importlib.util.spec_from_file_location(
    "m10_assembly", Path(__file__).with_name("assembly.py")
)
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
E, C, mm, V = A.E, A.C, A.mm, A.V

OUT = ROOT / "artifacts/m10/end_stop"
E.COLORS.update({"stop_pad": (0.80, 0.30, 0.16)})
STOPS = C["physics"]["end_stops_deg"]
LG, ST = C["leg"], C["internals"]["y_stack"]
R_PIN, PIN_D, DEPTH, RADIAL, PAD_LEN = 20.0, 5.0, 4.0, 6.0, 8.0  # mm
PSI = {"hip": 200.0, "knee": 180.0}  # dowel polar angle in the body frame, deg (hip: below the z=0 shell split)
CLEAR = 0.4  # mm, dowel end to groove floor
LUG = {"radius": 30.0, "pad": 4.0, "width": 8.0, "backing": 3.0, "top": 12.5}  # mm


def leg_state(a1, a2):
    """Nominal hip height with the thigh and shin at the given angles (deg)."""
    base = pose(C["poses"]["nominal_hip_height"], C)
    knee, axle = leg_points(math.radians(a1), math.radians(a2), C)
    return {
        **base,
        "a1_deg": a1,
        "a2_deg": a2,
        "knee_angle_deg": a2 - a1,
        "knee_m": knee.tolist(),
        "axle_m": axle.tolist(),
    }


POSES = {
    "nominal": None,
    "low_stops": (STOPS["thigh_a1"][0], STOPS["knee_drive_a2"][0]),
    "high_stops": (STOPS["thigh_a1"][1], STOPS["knee_drive_a2"][1]),
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


def sector(b0, b1, y0, y1, r0=R_PIN - RADIAL / 2, r1=R_PIN + RADIAL / 2, steps=48):
    """Annular sector about the hip axis (angles in rad from +x towards +z)."""
    n = max(4, int(steps * (b1 - b0) / math.pi))
    arc = [b0 + (b1 - b0) * k / n for k in range(n + 1)]
    pts = [(r1 * math.cos(b), r1 * math.sin(b)) for b in arc]
    pts += [(r0 * math.cos(b), r0 * math.sin(b)) for b in reversed(arc)]
    return prism(pts, y0, y1)


def arc_slot(psi, stops, angle, face_y, into):
    """Groove and end pads in a pulley face at the joint angle (deg).

    A pulley point at local angle b sits at b + angle in the body frame, so the fixed
    dowel at psi sweeps local angles psi - hi .. psi - lo over the range; the groove
    adds the dowel's half angle and a pad at each end. Returns (groove, pads) in mm.
    """
    lo, hi = (math.radians(v) for v in stops)
    psi, a = math.radians(psi), math.radians(angle)
    half_pin, pad = math.asin(PIN_D / 2 / R_PIN), PAD_LEN / R_PIN
    b0, b1 = psi - hi + a - half_pin - pad, psi - lo + a + half_pin + pad
    floor = face_y + into * DEPTH
    groove = sector(b0, b1, floor, face_y - into)
    pads = sector(b0, b0 + pad, floor, face_y + into * 0.2).fuse(
        sector(b1 - pad, b1, floor, face_y + into * 0.2)
    )
    return groove, pads


def dowel(psi, y0, y1):
    p = math.radians(psi)
    return A.ycyl(PIN_D / 2, y0, y1, R_PIN * math.cos(p), R_PIN * math.sin(p))


def hip_slot(state):
    """Option A hip: dowel in the side wall and hip housing, groove in the thigh pulley."""
    y_pulley_out = mm(ST["bulkhead_half"]) + 4 * mm(ST["clearance"]) + 2 * mm(
        ST["pulley_width"]
    ) + 2 * mm(ST["motor_module_width"])
    pod1 = mm(LG["hip_housing_y"][1])
    groove, pads = arc_slot(PSI["hip"], STOPS["thigh_a1"], state["a1_deg"], y_pulley_out, -1)
    pin = dowel(PSI["hip"], y_pulley_out - DEPTH + CLEAR, pod1 - 1.0)
    adds = [
        ("left_thigh", "left_HipStopPad", pads, "stop_pad"),
        ("body", "left_HipStopPin", pin, "metal"),
    ]
    return adds, {"left_ThighHipPulley": groove}, ["left_HipStopPin"]


def knee_slot(state):
    """Knee drive (both options): one dowel through the bulkhead, grooves in both knee pulleys."""
    y_face = mm(ST["bulkhead_half"]) + mm(ST["clearance"])
    groove, pads = arc_slot(PSI["knee"], STOPS["knee_drive_a2"], state["a2_deg"], y_face, 1)
    half = y_face + DEPTH - CLEAR
    adds = [
        ("left_kneedrive", "left_KneeDriveStopPad", pads, "stop_pad"),
        ("body", "KneeDriveStopPin", dowel(PSI["knee"], -half, half), "metal"),
    ]
    return adds, {"left_KneeHipPulley": groove}, ["KneeDriveStopPin"]


def hip_lugs(state):
    """Option B hip: TPU-faced lugs on the hip housing's outer face at each range end."""
    xt = C["exterior"]["thigh"]
    rh, rk, lt = mm(xt["hip_radius"]), mm(xt["knee_radius"]), mm(LG["thigh_length"])
    alpha = math.acos((rh - rk) / lt)
    arm = math.sqrt(LUG["radius"] ** 2 - rh**2)
    y0 = mm(LG["hip_housing_y"][1])
    width, t, back = LUG["width"], LUG["pad"], LUG["backing"]
    adds = []
    for a1, sense, tag in ((STOPS["thigh_a1"][0], -1, "Low"), (STOPS["thigh_a1"][1], 1, "High")):
        phi = math.radians(a1)
        a = phi + sense * alpha  # normal of the leading flank at the stop
        n = (math.cos(a), math.sin(a))
        d = (-n[1], n[0]) if -n[1] * math.cos(phi) + n[0] * math.sin(phi) > 0 else (n[1], -n[0])
        p = (rh * n[0] + arm * d[0], rh * n[1] + arm * d[1])

        def at(u, w, p=p, d=d, n=n):
            return (p[0] + u * d[0] + w * n[0], p[1] + u * d[1] + w * n[1])

        pad = [at(-width / 2, 0), at(width / 2, 0), at(width / 2, t), at(-width / 2, t)]
        body = hull(
            [at(-width / 2 - 1.5, t), at(width / 2 + 1.5, t)]
            + [at(-width / 2 - 1.5, t + back), at(width / 2 + 1.5, t + back)]
        )
        lug = E.soft(prism(body, y0, y0 + LUG["top"]), 0.8)
        adds += [
            ("body", f"left_HipStopLug{tag}", lug, "shell_dark"),
            ("body", f"left_HipStopPad{tag}", prism(pad, y0 + 0.3, y0 + LUG["top"] - 1.0), "stop_pad"),
        ]
    return adds, {}, []


OPTIONS = {"A_hidden_slots": (hip_slot, knee_slot), "B_hip_lugs": (hip_lugs, knee_slot)}
STRIKERS = {
    "left_HipStopPad": "left_HipStopPin",
    "left_KneeDriveStopPad": "KneeDriveStopPin",
    "left_HipStopPadLow": "left_Thigh",
    "left_HipStopPadHigh": "left_Thigh",
}
BASE = {}
SEATS = {}


def base(name):
    if name not in BASE:
        angles = POSES[name]
        state = pose(C["poses"]["nominal_hip_height"], C) if angles is None else leg_state(*angles)
        BASE[name] = (state, A.build(state))
    return BASE[name]


def build(option, name):
    """Hip-frame parts with the option's stops; dowels are seated in every body part they cross."""
    state, parts = base(name)
    adds, cuts, seated = [], {}, []
    for builder in OPTIONS[option]:
        a, c, s = builder(state)
        adds += a
        cuts.update(c)
        seated += s
    adds += [
        (lk.replace("left", "right"), n.replace("left", "right"), mirror(sh), col)
        for lk, n, sh, col in adds
        if n.startswith("left_")
    ]
    cuts.update({k.replace("left", "right"): mirror(v) for k, v in list(cuts.items())})
    seated += [n.replace("left", "right") for n in seated if n.startswith("left_")]
    pins = {n: sh for _, n, sh, _ in adds if n in seated}
    out, seats = [], {n: [] for n in pins}
    for lk, n, sh, col in parts:
        if n in cuts:
            sh = sh.cut(cuts[n])
        if lk == "body":
            for pin_name, pin in pins.items():
                if sh.BoundBox.intersect(pin.BoundBox) and sh.common(pin).Volume > 1e-3:
                    sh = sh.cut(pin)
                    seats[pin_name].append(n)
        out.append((lk, n, sh, col))
    SEATS[option] = seats
    return state, out + adds, [n for _, n, _, _ in adds]


def checks(option):
    """Pad-to-striker gap (mm) and overlap (mm3) at each pose, and new-part overlaps."""
    out = {}
    for name in POSES:
        _, parts, new = build(option, name)
        shapes = {n: sh for _, n, sh, _ in parts}
        row = {"stops": {}, "new_part_overlaps_mm3": []}
        for pad, striker in STRIKERS.items():
            if pad in shapes:
                row["stops"][pad] = {
                    "striker": striker,
                    "gap_mm": round(shapes[pad].distToShape(shapes[striker])[0], 2),
                    "overlap_mm3": round(shapes[pad].common(shapes[striker]).Volume, 2),
                }
        for n in new:
            for other, sh in shapes.items():
                if other == n or not sh.BoundBox.intersect(shapes[n].BoundBox):
                    continue
                v = shapes[n].common(sh).Volume
                if v > 0.05:
                    row["new_part_overlaps_mm3"].append([n, other, round(v, 2)])
        row["dowel_seats"] = SEATS[option]
        out[name] = row
    print("M10 END STOP CHECK", option, json.dumps(out), flush=True)
    return out


def views(option, states, world):
    """Close-ups of the left hip. Option A: cutaways to the pulley faces; option B: outboard."""
    out = []
    for name, state in states.items():
        hip = [-state["axle_m"][0], 0.0, state["hip_height_m"]]
        stop = name != "nominal"
        boxes = {n: sh.BoundBox for _, n, sh, _ in world[name]}
        if option.startswith("A"):
            y_hip = 0.051
            keep = {"left_ThighHipPulley", "left_HipStopPin", "left_HipStopPad"}
            hide = [n for n, b in boxes.items() if b.YMax > 51.2 and n not in keep]
            out.append(
                {
                    "name": f"{name}_hip_slot",
                    "pose": name,
                    "direction": [0.15, 1.0, 0.25],
                    "target": [hip[0], y_hip, hip[2]],
                    "lens": 70,
                    "size": 0.075,
                    "floor": False,
                    "hide": hide,
                }
            )
            keep = {"left_KneeHipPulley", "KneeDriveStopPin", "left_KneeDriveStopPad"}
            hide = [n for n, b in boxes.items() if b.YMin < 4.4 and n not in keep]
            out.append(
                {
                    "name": f"{name}_knee_drive_slot",
                    "pose": name,
                    "direction": [0.15, -1.0, 0.25],
                    "target": [hip[0], 0.0045, hip[2]],
                    "lens": 70,
                    "size": 0.075,
                    "floor": False,
                    "hide": hide,
                }
            )
        else:
            wheel_shin = [n for lk, n, _, _ in world[name] if lk in ("left_shin", "left_wheel")]
            out.append(
                {
                    "name": f"{name}_hip_lugs",
                    "pose": name,
                    "direction": [0.15, 1.0, 0.25],
                    "target": [hip[0], 0.065, hip[2]],
                    "lens": 70,
                    "size": 0.11,
                    "floor": not stop,
                    "hide": wheel_shin if stop else [],
                }
            )
        if name == "nominal":
            out.append(
                {
                    "name": "nominal_rear_three_quarter",
                    "pose": name,
                    "direction": [-0.8, 1.0, 0.35],
                    "target": [hip[0], 0.06, hip[2] - 0.03],
                    "lens": 70,
                    "size": 0.16,
                }
            )
    return out


def main():
    for option in OPTIONS:
        record = {"poses": {}, "units": "m", "colors": {k: list(v) for k, v in E.COLORS.items()}}
        states, world = {}, {}
        for name in POSES:
            state, parts, _ = build(option, name)
            parts = E.to_world(state, parts)
            states[name], world[name] = state, parts
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
        record["views"] = views(option, states, world)
        record["checks"] = checks(option)
        record["status"] = "Phase D4 hip and knee-drive end stop look study (not adopted)"
        folder = OUT / option
        folder.mkdir(parents=True, exist_ok=True)
        with gzip.open(folder / "render-mesh.json.gz", "wt") as f:
            json.dump(record, f)
        print("M10 END STOP", option, list(record["poses"]), flush=True)


if __name__ in ("__main__", "bridge_main"):
    main()
