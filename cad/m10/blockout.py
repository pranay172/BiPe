"""M10 phase A blockout: reference-style body, head, serial legs and hub wheels.

freecad.cmd cad/m10/blockout.py

Reads config/robot.json "m10" (SI) and converts to millimetres here only.
Leg angles come from bipe.m10.concept so CAD and sizing share one model.
Writes artifacts/m10/concept/{blockout.FCStd, render-mesh.json.gz, check.json}.
Blockout fidelity: forms and envelopes, not detailed parts.
"""

import gzip
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import FreeCAD as App
import Part

from bipe.m10.concept import envelope, load, pose

MM = 1000.0
OUT = ROOT / "artifacts/m10/concept"
C = load()
V = App.Vector

COLORS = {
    "shell": (0.86, 0.85, 0.82),
    "shell_dark": (0.70, 0.70, 0.68),
    "tire": (0.42, 0.42, 0.42),
    "hub": (0.80, 0.80, 0.78),
    "eye": (1.0, 0.93, 0.78),
    "lens": (0.05, 0.06, 0.07),
    "ear": (0.85, 0.22, 0.12),
}


def mm(value):
    return value * MM


def soft(shape, radius):
    """Fillet every edge when the kernel allows it; blockouts keep the raw form otherwise."""
    try:
        return shape.makeFillet(radius, shape.Edges)
    except Part.OCCError:
        return shape


def body_shell():
    """Cuboid body with generously rounded edges, as on the reference."""
    b = C["body"]
    length, width, height = mm(b["length"]), mm(b["width"]), mm(b["height"])
    corner = V(mm(-b["hip_x"]) - length / 2, -width / 2, -mm(b["hip_z_above_bottom"]))
    box = Part.makeBox(length, width, height, corner)
    return box.makeFillet(mm(b["edge_radius"]), box.Edges)


def front_cameras():
    b = C["body"]
    cam = b["front_cameras"]
    x = mm(-b["hip_x"] + b["length"] / 2)
    z = -mm(b["hip_z_above_bottom"]) + mm(cam["z_above_bottom"])
    parts = []
    for s in (1, -1):
        y = s * mm(cam["spacing_y"]) / 2
        bezel = Part.makeCylinder(mm(cam["diameter"]) * 0.75, 3, V(x - 1.5, y, z), V(1, 0, 0))
        lens = Part.makeCylinder(mm(cam["diameter"]) / 2, 3.6, V(x - 1.5, y, z), V(1, 0, 0))
        parts += [("FrontCameraBezel", bezel, "shell"), ("FrontCameraLens", lens, "lens")]
    return parts


def head():
    b, h = C["body"], C["head"]
    cx = mm(-b["hip_x"] + h["x"])
    top = -mm(b["hip_z_above_bottom"]) + mm(b["height"])
    radius = mm(h["pod_height"]) / 2
    zc = top + mm(h["neck_height"]) + radius
    depth = mm(h["pod_depth_x"])
    spacing = mm(h["eye_spacing_y"])
    pod = None
    for s in (1, -1):
        eye_can = Part.makeCylinder(
            radius, depth, V(cx - depth / 2, s * spacing / 2, zc), V(1, 0, 0)
        )
        pod = eye_can if pod is None else pod.fuse(eye_can)
    bridge = Part.makeBox(
        depth * 0.9, spacing, radius * 1.5, V(cx - depth * 0.45, -spacing / 2, zc - radius * 0.75)
    )
    # Rear housing behind the eyes, as on the reference pod.
    rear = Part.makeBox(
        depth * 0.5,
        spacing + 2 * radius * 0.8,
        radius * 1.7,
        V(cx - depth / 2, -(spacing / 2 + radius * 0.8), zc - radius * 0.85),
    )
    pod = soft(pod.fuse(bridge).fuse(rear).removeSplitter(), 3.0)
    neck = Part.makeCylinder(
        mm(h["neck_diameter"]) / 2, mm(h["neck_height"]) + 2, V(cx, 0, top - 1)
    )
    parts = [("HeadPod", pod, "shell"), ("Neck", neck, "shell_dark")]
    eye_r = mm(h["eye_diameter"]) / 2
    front = cx + depth / 2
    for s in (1, -1):
        lens = Part.makeCylinder(
            eye_r, mm(h["eye_protrusion"]) + 1, V(front - 1, s * spacing / 2, zc), V(1, 0, 0)
        )
        parts.append(("EyeLens", soft(lens, 1.2), "eye"))
    w, hh = (mm(v) for v in h["camera_window"])
    camera = Part.makeBox(2.5, w, hh, V(front - 1, -w / 2, zc - hh / 2))
    parts.append(("CameraWindow", camera, "lens"))
    ear = h["ear"]
    for s in (1, -1):
        length, width, thick = mm(ear["length"]), mm(ear["width"]), mm(ear["thickness"])
        prof = [
            V(0, 0, 0),
            V(width, 0, 0),
            V(width * 0.75, 0, length),
            V(width * 0.1, 0, length * 0.92),
        ]
        flap = Part.Face(Part.makePolygon(prof + [prof[0]])).extrude(V(0, thick, 0))
        flap = soft(flap, 1.2)
        # Broad face to the side, turned slightly forward, swept back and splayed outward.
        flap.translate(V(-width / 2, -thick / 2, 0))
        flap.rotate(V(0, 0, 0), V(0, 0, 1), -s * ear["face_yaw_deg"])
        flap.rotate(V(0, 0, 0), V(0, 1, 0), -ear["tilt_deg"])
        flap.rotate(V(0, 0, 0), V(1, 0, 0), -s * ear["splay_deg"])
        flap.translate(V(0, thick / 2, 0))
        flap.translate(V(cx - depth * 0.45, s * mm(ear["y"]) - thick / 2, zc + radius * 0.7))
        parts.append(("Ear", flap, "ear"))
    return parts


def capsule(p0, p1, width, y0, y1):
    """Stadium link in the x-z plane between two joint centres (mm)."""
    r = width / 2
    a = Part.makeCylinder(r, y1 - y0, V(p0[0], y0, p0[1]), V(0, 1, 0))
    b = Part.makeCylinder(r, y1 - y0, V(p1[0], y0, p1[1]), V(0, 1, 0))
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    box = Part.makeBox(length, y1 - y0, width, V(0, y0, -r))
    box.rotate(V(0, 0, 0), V(0, 1, 0), -math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])))
    box.translate(V(p0[0], 0, p0[1]))
    return soft(a.fuse(b).fuse(box).removeSplitter(), min(3.0, (y1 - y0) / 2 - 0.2))


def leg(state, side):
    lg, w = C["leg"], C["wheel"]
    s = 1 if side == "left" else -1

    def ys(pair):
        lo, hi = (mm(v) for v in pair)
        return (lo, hi) if s > 0 else (-hi, -lo)

    hip = (0.0, 0.0)
    knee = tuple(mm(v) for v in state["knee_m"])
    axle = tuple(mm(v) for v in state["axle_m"])
    parts = []
    y0, y1 = ys(lg["hip_housing_y"])
    disc = Part.makeCylinder(mm(lg["hip_housing_diameter"]) / 2, y1 - y0, V(0, y0, 0), V(0, 1, 0))
    parts.append((f"{side}_HipHousing", soft(disc, 1.5), "shell_dark"))
    y0, y1 = ys(lg["thigh_y"])
    parts.append((f"{side}_Thigh", capsule(hip, knee, mm(lg["thigh_width"]), y0, y1), "shell"))
    y0, y1 = ys(lg["shin_y"])
    parts.append((f"{side}_Shin", capsule(knee, axle, mm(lg["shin_width"]), y0, y1), "shell_dark"))
    r = mm(w["diameter"]) / 2
    width = mm(w["width"])
    yc = s * mm(w["y_center"])
    tire = Part.makeCylinder(r, width, V(axle[0], yc - width / 2, axle[1]), V(0, 1, 0))
    tire = tire.cut(
        Part.makeCylinder(r - 9, width + 2, V(axle[0], yc - width / 2 - 1, axle[1]), V(0, 1, 0))
    )
    parts.append((f"{side}_Tire", soft(tire, 6.0), "tire"))
    rim = Part.makeCylinder(r - 8.5, width - 2, V(axle[0], yc - width / 2 + 1, axle[1]), V(0, 1, 0))
    parts.append((f"{side}_HubMotorRim", rim, "hub"))
    # Three-spoke hub cover on the outer face.
    face_y = yc + s * (width / 2 - 1)
    for k in range(3):
        spoke = Part.makeBox(r - 10, 2.2, 5, V(0, -1.1, -2.5))
        spoke.rotate(V(0, 0, 0), V(0, 1, 0), 90 + k * 120)
        spoke.translate(V(axle[0], face_y, axle[1]))
        parts.append((f"{side}_HubSpoke", spoke, "shell"))
    cap = Part.makeCylinder(6, 2.4, V(axle[0], face_y, axle[1]), V(0, s, 0))
    parts.append((f"{side}_HubCap", cap, "shell"))
    return parts


def assemble(name):
    state = pose(C["poses"][f"{name}_hip_height"], C)
    parts = [("BodyShell", body_shell(), "shell")] + front_cameras() + head()
    for side in ("left", "right"):
        parts += leg(state, side)
    # World: ground at z=0, origin midway between the wheel contacts.
    shift = V(-mm(state["axle_m"][0]), 0, mm(state["hip_height_m"]))
    placed = []
    for part, shape, color in parts:
        shape = shape.copy()
        shape.translate(shift)
        placed.append((part, shape, color))
    return state, placed


def views(doc):
    """Shaded GUI views for quick review when run through the FreeCAD GUI bridge."""
    import FreeCADGui as Gui

    Gui.setActiveDocument(doc.Name)
    view = Gui.activeDocument().activeView()
    folder = OUT / "views"
    folder.mkdir(exist_ok=True)

    def look(direction):
        # Camera axes: right, up, and backward (-direction); FreeCAD cameras look along -Z.
        d = V(*direction).normalize()
        right = d.cross(V(0, 0, 1)).normalize()
        up = right.cross(d)
        back = d * -1
        m = App.Matrix(right.x, up.x, back.x, 0, right.y, up.y, back.y, 0, right.z, up.z, back.z, 0)
        # Set the Coin camera directly: setCameraOrientation animates when navigation
        # animations are on, and saveImage would capture a mid-animation frame.
        return lambda: view.getCameraNode().orientation.setValue(*App.Rotation(m).Q)

    # Robot faces +x. Front-left three-quarter from slightly above, like the reference photos.
    for name, setter in [
        ("three_quarter", look((-1.0, -0.75, -0.42))),
        ("front", look((-1, 0, 0))),
        ("side", look((0, 1, 0))),
        ("rear_three_quarter", look((0.8, -0.9, -0.35))),
    ]:
        Gui.updateGui()
        setter()
        view.fitAll()
        Gui.updateGui()
        view.saveImage(str(folder / f"{name}.png"), 1200, 900, "White")


def export():
    OUT.mkdir(parents=True, exist_ok=True)
    record = {"poses": {}, "units": "m", "colors": COLORS}
    checks = {}
    for name in ["crouched", "nominal", "extended"]:
        state, parts = assemble(name)
        comps = []
        bound = None
        for part, shape, color in parts:
            pts, tris = shape.tessellate(0.15)
            comps.append(
                {
                    "name": part,
                    "color": COLORS[color],
                    "vertices": [
                        [round(p.x / MM, 6), round(p.y / MM, 6), round(p.z / MM, 6)] for p in pts
                    ],
                    "triangles": [list(t) for t in tris],
                }
            )
            bb = shape.BoundBox
            bound = bb if bound is None else bound.united(bb)
        record["poses"][name] = comps
        # Moving leg parts must clear the body and each other; the hip disc,
        # neck and cameras are intentionally set into the shell.
        moving = ("Thigh", "Shin", "Tire", "HubMotorRim")
        overlaps = []
        for i, (na, sa, _) in enumerate(parts):
            for nb, sb, _ in parts[i + 1 :]:
                pair = (na, nb)
                leg_vs_body = (
                    any(m in na for m in moving)
                    and nb in ("BodyShell", "HeadPod")
                    or (any(m in nb for m in moving) and na in ("BodyShell", "HeadPod"))
                )
                adjacent = {na.split("_")[-1], nb.split("_")[-1]} in (
                    {"Thigh", "Shin"},
                    {"Shin", "Tire"},
                    {"Shin", "HubMotorRim"},
                    {"Tire", "HubMotorRim"},
                    {"HipHousing", "Thigh"},
                )
                same_side = na.split("_")[0] == nb.split("_")[0]
                leg_vs_leg = (
                    same_side and any(m in na for m in moving) and any(m in nb for m in moving)
                )
                if not (leg_vs_body or (leg_vs_leg and not adjacent)):
                    continue
                if sa.BoundBox.intersect(sb.BoundBox):
                    volume = sa.common(sb).Volume
                    if volume > 1.0:
                        overlaps.append([*pair, round(volume, 1)])
        checks[name] = {
            "bounds_mm": [
                round(bound.XLength, 1),
                round(bound.YLength, 1),
                round(bound.ZLength, 1),
            ],
            "lowest_z_mm": round(bound.ZMin, 2),
            "concept_envelope_m": envelope(state, C),
            "leg_state": {
                k: state[k] for k in ["a1_deg", "a2_deg", "knee_angle_deg", "com_height_m"]
            },
            "unintended_overlaps_mm3": overlaps,
        }
        if name == "nominal":
            doc = App.newDocument("M10Blockout")
            for part, shape, color in parts:
                obj = doc.addObject("Part::Feature", part)
                obj.Shape = shape
                if App.GuiUp:
                    obj.ViewObject.ShapeColor = COLORS[color]
                    obj.ViewObject.LineColor = (0.55, 0.55, 0.55)
            doc.recompute()
            doc.saveAs(str(OUT / "blockout.FCStd"))
            if App.GuiUp:
                views(doc)
    with gzip.open(OUT / "render-mesh.json.gz", "wt") as f:
        json.dump(record, f)
    (OUT / "check.json").write_text(json.dumps(checks, indent=2) + "\n")
    print("M10 BLOCKOUT", json.dumps({k: v["bounds_mm"] for k, v in checks.items()}), flush=True)


export()
