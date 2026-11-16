"""M10 phase B1 exterior: styled head, legs and wheels on the approved body.

python3 tools/freecad_bridge.py cad/m10/exterior.py

Reads config/robot.json "m10" (SI) and converts to millimetres here only.
Leg angles come from bipe.m10.concept. Appearance fidelity: outer forms and
details, solid parts; internals are fitted in phase B2. Writes
artifacts/m10/exterior/{exterior.FCStd, render-mesh.json.gz, check.json, views/}.
"""

import gzip
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import FreeCAD as App
import Part

from bipe.m10.concept import envelope, load, pose

MM = 1000.0
OUT = ROOT / "artifacts/m10/exterior"
C = load()
X = C["exterior"]
COLORS = {k: tuple(v) for k, v in X["colors_srgb"].items()}
V = App.Vector


def mm(value):
    return value * MM


def soft(shape, radius, edges=None):
    """Fillet edges when the kernel allows it; keep the raw form otherwise."""
    try:
        return shape.makeFillet(radius, edges or shape.Edges)
    except Part.OCCError:
        print(f"M10 EXTERIOR fillet {radius} mm skipped", flush=True)
        return shape


def sharp_edges(shape, min_deg=5.0):
    """Edges whose two faces meet at more than min_deg; tangent (G1) seams cannot be filleted."""
    found = []
    for edge in shape.Edges:
        faces = shape.ancestorsOfType(edge, Part.Face)
        if len(faces) != 2:
            found.append(edge)
            continue
        p = edge.valueAt(0.5 * (edge.FirstParameter + edge.LastParameter))
        normals = [f.normalAt(*f.Surface.parameter(p)) for f in faces]
        cos = max(-1.0, min(1.0, normals[0].dot(normals[1])))
        if math.degrees(math.acos(cos)) > min_deg:
            found.append(edge)
    return found


def tapered_link(p0, r0, p1, r1, y0, y1):
    """Convex hull of two circles in the x-z plane, extruded from y0 to y1 (mm)."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(dx, dz)
    phi = math.atan2(dz, dx)
    alpha = math.acos((r0 - r1) / d)

    def at(c, r, a):
        return V(c[0] + r * math.cos(a), y0, c[1] + r * math.sin(a))

    a_up, a_dn = phi + alpha, phi - alpha
    edges = [
        Part.LineSegment(at(p0, r0, a_up), at(p1, r1, a_up)).toShape(),
        Part.Arc(at(p1, r1, a_up), at(p1, r1, phi), at(p1, r1, a_dn)).toShape(),
        Part.LineSegment(at(p1, r1, a_dn), at(p0, r0, a_dn)).toShape(),
        Part.Arc(at(p0, r0, a_dn), at(p0, r0, phi + math.pi), at(p0, r0, a_up)).toShape(),
    ]
    return Part.Face(Part.Wire(edges)).extrude(V(0, y1 - y0, 0))


def rounded_plate(length, width, thick, radius):
    """Rounded rectangle plate centred on the origin in the x-z plane, thickness along +y."""
    box = Part.makeBox(length, thick, width, V(-length / 2, 0, -width / 2))
    vertical = [e for e in box.Edges if abs(e.BoundBox.YLength - thick) < 1e-6]
    return soft(box, radius, vertical)


def body_shell():
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
        parts += [("body", "FrontCameraBezel", soft(bezel, 0.8), "shell_dark")]
        parts += [("body", "FrontCameraLens", lens, "lens")]
    return parts


def camera_bezel(front, zc, grow=0.0):
    """Camera bezel plate pitched up like the reference about a y axis at the recessed web face,
    optionally grown by `grow` mm on every side (seat and boss in assembly.head_internals).
    Returns (camera centre z, pitch rotation arguments, bezel shape)."""
    xh = X["head"]
    bw, bh = (mm(v) + 2 * grow for v in xh["camera_bezel"])
    recess = mm(xh["front_recess"])
    cam_z = zc - mm(xh["camera_drop_z"])
    bezel = rounded_plate(bw, bh, 3.0 + 2 * grow, 1.5 + grow)
    bezel.rotate(V(0, 0, 0), V(0, 0, 1), -90)
    bezel.translate(V(front - recess - 1.5 - grow, 0, cam_z))
    pitch_axis = (V(front - recess, 0, cam_z), V(0, 1, 0), -xh["camera_pitch_deg"])
    bezel.rotate(*pitch_axis)
    return cam_z, pitch_axis, bezel


def head():
    """Binocular pod: two fused eye barrels on a flat base, camera between the eyes."""
    b, h, xh = C["body"], C["head"], X["head"]
    cx = mm(-b["hip_x"] + h["x"])
    top = -mm(b["hip_z_above_bottom"]) + mm(b["height"])
    r = mm(h["pod_height"]) / 2
    length = mm(h["pod_depth_x"])
    half = mm(h["eye_spacing_y"]) / 2
    zc = top + mm(h["neck_height"]) + r - mm(xh["base_flat_from_bottom"])
    back, front = cx - length / 2, cx + length / 2
    rear_round = mm(xh["rear_round"])
    pod = None
    for s in (1, -1):
        barrel = Part.makeCylinder(r, length, V(back, s * half, zc), V(1, 0, 0))
        rear_rim = [e for e in barrel.Edges if abs(e.BoundBox.XMax - back) < 1e-6]
        barrel = soft(barrel, rear_round, rear_rim)
        pod = barrel if pod is None else pod.fuse(barrel)
    # Web between the barrels; it starts clear of the rear rounds (beyond the fillet
    # radius from their tangent seams) and runs to the front face so the fillet
    # sees no step.
    recess = mm(xh["front_recess"])
    web_x0 = back + rear_round + 3.0
    web = Part.makeBox(
        front - web_x0, 2 * half, xh["web_height_ratio"] * r, V(web_x0, -half, zc - r)
    )
    barrels = pod
    pod = pod.fuse(web).removeSplitter()
    # Flat base on the neck.
    base_z = zc - r + mm(xh["base_flat_from_bottom"])
    pod = pod.cut(
        Part.makeBox(length + 2, 4 * (half + r), r, V(back - 1, -2 * (half + r), base_z - r))
    )
    pod = pod.removeSplitter()
    pod = soft(pod, mm(xh["fillet"]), sharp_edges(pod))
    # Camera recess: the web's front set back between the eye barrels, cut after the fillet.
    pocket = Part.makeBox(
        recess + 1, 2 * half, xh["web_height_ratio"] * r + 1, V(front - recess, -half, base_z - 1)
    )
    pod = pod.cut(pocket.cut(barrels))
    eye_r = mm(h["eye_diameter"]) / 2
    rim_recess = mm(xh["eye_rim_recess"])
    parts = []
    for s in (1, -1):
        # The recess runs back to the lens's rear face: the lens seats on a ledge
        # left by the internal eye window (assembly.head_internals).
        lens_back = front - rim_recess - 0.5
        pod = pod.cut(
            Part.makeCylinder(eye_r, front - lens_back + 1, V(lens_back, s * half, zc), V(1, 0, 0))
        )
        lens = Part.makeCylinder(eye_r - 0.2, 1.0, V(lens_back, s * half, zc), V(1, 0, 0))
        parts.append(("body", "EyeLens", soft(lens, 0.4), "eye"))
        screw = Part.makeCylinder(
            1.4, 0.6, V(back + 0.25 * length, s * (half + r - 0.3), zc), V(0, s, 0)
        )
        parts.append(("body", "HeadScrew", screw, "bolt"))
    parts.insert(0, ("body", "HeadPod", pod, "shell"))
    cam_z, pitch_axis, bezel = camera_bezel(front, zc)
    parts.append(("body", "CameraBezel", bezel, "camera"))
    lens_r = mm(xh["camera_lens_diameter"]) / 2
    ring = Part.makeCylinder(lens_r + 1.0, 0.4, V(front - recess + 1.5, 0, cam_z), V(1, 0, 0))
    ring.rotate(*pitch_axis)
    parts.append(("body", "CameraRing", ring, "bolt"))
    lens = Part.makeCylinder(lens_r, 0.7, V(front - recess + 1.5, 0, cam_z), V(1, 0, 0))
    lens = soft(lens, 0.3)
    lens.rotate(*pitch_axis)
    parts.append(("body", "CameraLens", lens, "lens"))
    neck = Part.makeCylinder(mm(h["neck_diameter"]) / 2, base_z - top + 2, V(cx, 0, top - 1))
    parts.append(("body", "Neck", soft(neck, 1.0), "shell_dark"))
    ear = h["ear"]
    for s in (1, -1):
        e_len, e_w, thick = mm(ear["length"]), mm(ear["width"]), mm(ear["thickness"])
        prof = [V(0, 0, 0), V(e_w, 0, 0), V(e_w * 0.8, 0, e_len), V(e_w * 0.15, 0, e_len * 0.9)]
        flap = Part.Face(Part.makePolygon([*prof, prof[0]])).extrude(V(0, thick, 0))
        flap = soft(flap, min(2.5, thick / 2 - 0.2))
        flap.translate(V(-e_w / 2, -thick / 2, 0))
        parts.append(("body", "Ear", place_ear(flap, s), "ear"))
    return parts


def place_ear(shape, s):
    """Ear frame -> hip frame: root edge centred at the origin, broad face to the side,
    turned slightly forward, swept back and splayed outward."""
    b, h = C["body"], C["head"]
    ear = h["ear"]
    r = mm(h["pod_height"]) / 2
    top = -mm(b["hip_z_above_bottom"]) + mm(b["height"])
    zc = top + mm(h["neck_height"]) + r - mm(X["head"]["base_flat_from_bottom"])
    back = mm(-b["hip_x"] + h["x"]) - mm(h["pod_depth_x"]) / 2
    shape = shape.copy()
    shape.rotate(V(0, 0, 0), V(0, 0, 1), -s * ear["face_yaw_deg"])
    shape.rotate(V(0, 0, 0), V(0, 1, 0), -ear["tilt_deg"])
    shape.rotate(V(0, 0, 0), V(1, 0, 0), -s * ear["splay_deg"])
    shape.translate(V(back + 0.22 * mm(h["pod_depth_x"]), s * mm(ear["y"]), zc + 0.2 * r))
    return shape


def knee_sector(knee, r0, r1, b0, b1, y0, y1):
    """Annular sector about the knee axis (x-z, mm; angles in degrees from +x towards +z)
    between y0 and y1: a radial rectangle revolved about the axis."""
    a = math.radians(b0)
    lo, hi = min(y0, y1), max(y0, y1)
    pts = [
        V(knee[0] + r * math.cos(a), y, knee[1] + r * math.sin(a))
        for r, y in ((r0, lo), (r1, lo), (r1, hi), (r0, hi))
    ]
    face = Part.Face(Part.makePolygon([*pts, pts[0]]))
    return Part.Solid(face.revolve(V(knee[0], 0, knee[1]), V(0, -1, 0), b1 - b0))


def leg(state, side):
    lg, w, xt, xs, xw = C["leg"], C["wheel"], X["thigh"], X["shin"], X["wheel"]
    s = 1 if side == "left" else -1

    def ys(pair):
        lo, hi = (mm(v) for v in pair)
        return (lo, hi) if s > 0 else (-hi, -lo)

    def outer(pair):
        lo, hi = ys(pair)
        return hi if s > 0 else lo

    hip = (0.0, 0.0)
    knee = tuple(mm(v) for v in state["knee_m"])
    axle = tuple(mm(v) for v in state["axle_m"])
    thigh_link, shin_link, wheel_link = (f"{side}_{n}" for n in ("thigh", "shin", "wheel"))
    parts = []

    # Hip housing on the body flank with a thin flange ring.
    y0, y1 = ys(lg["hip_housing_y"])
    disc = Part.makeCylinder(mm(lg["hip_housing_diameter"]) / 2, y1 - y0, V(0, y0, 0), V(0, 1, 0))
    parts.append(("body", f"{side}_HipHousing", soft(disc, 1.5), "shell_dark"))

    # Thigh: tapered, softly rounded link with hip cap and a flush inset cover panel.
    y0, y1 = ys(lg["thigh_y"])
    rh, rk = mm(xt["hip_radius"]), mm(xt["knee_radius"])
    thigh = soft(tapered_link(hip, rh, knee, rk, y0, y1), mm(xt["edge_fillet"]))
    face = outer(lg["thigh_y"])
    angle = math.degrees(math.atan2(knee[1], knee[0]))
    at = xt["panel_at"]
    depth = mm(xt["panel_depth"])

    def along_thigh(length, width, thick, top):
        """Plate on the thigh axis whose outer face sits at y = top."""
        plate = rounded_plate(length, width, thick, 2.5)
        plate.rotate(V(0, 0, 0), V(0, 1, 0), -angle)
        plate.translate(V(knee[0] * at, top - thick if s > 0 else top, knee[1] * at))
        return plate

    pocket = along_thigh(mm(xt["panel_length"]), mm(xt["panel_width"]), depth + 1, face + s)
    thigh = thigh.cut(pocket)
    # Hip cap: a flush inset disc (like the panel) so the folding shin can pass over it.
    cap_r, cap_depth = mm(xt["hip_cap_radius"]), mm(xt["hip_cap_depth"])
    gap = mm(xt["panel_gap"])
    thigh = thigh.cut(
        Part.makeCylinder(cap_r, cap_depth + 1, V(0, face - s * cap_depth, 0), V(0, s, 0))
    )
    # Knee-angle stop (phase D4 arc slot, m10.internals.knee_stop): a groove in the thigh's
    # outer face on the pin circle with a TPU pad at each end; the shin's steel pin meets a
    # pad exactly at each plant stop (m10.physics.end_stops_deg.knee_angle).
    ks = C["internals"]["knee_stop"]
    rp, pin_r = mm(ks["pin_circle_radius"]), mm(ks["pin_diameter"]) / 2
    half_w, depth = mm(ks["groove_radial_width"]) / 2, mm(ks["groove_depth"])
    pad_deg = math.degrees(mm(ks["pad_length"]) / rp)
    pin_deg = math.degrees(math.asin(pin_r / rp))
    stops = C["physics"]["end_stops_deg"]["knee_angle"]
    base = state["a1_deg"] + ks["pin_offset_deg"]
    lo, hi = base + stops[0] - pin_deg - pad_deg, base + stops[1] + pin_deg + pad_deg
    thigh = thigh.cut(
        knee_sector(knee, rp - half_w, rp + half_w, lo, hi, face - s * depth, face + s)
    )
    recess = mm(ks["pad_top_recess"])
    pads = knee_sector(
        knee, rp - half_w, rp + half_w, lo, lo + pad_deg, face - s * depth, face - s * recess
    )
    pads = pads.fuse(
        knee_sector(
            knee, rp - half_w, rp + half_w, hi - pad_deg, hi, face - s * depth, face - s * recess
        )
    )
    parts.append((thigh_link, f"{side}_Thigh", thigh, "shell"))
    parts.append((thigh_link, f"{side}_KneeStopPad", pads, "stop_pad"))
    panel = along_thigh(
        mm(xt["panel_length"]) - 2 * gap,
        mm(xt["panel_width"]) - 2 * gap,
        depth - 0.2,
        face - s * 0.2,
    )
    parts.append((thigh_link, f"{side}_ThighPanel", panel, "panel"))
    cap = Part.makeCylinder(cap_r - gap, cap_depth - 0.2, V(0, face - s * cap_depth, 0), V(0, s, 0))
    parts.append((thigh_link, f"{side}_HipCap", soft(cap, 0.2), "shell"))

    # Shin: short slim link from the knee to the wheel's inner face; the knee
    # pivot cap shows on its outer face.
    y0, y1 = ys(lg["shin_y"])
    shin = tapered_link(knee, mm(xs["knee_radius"]), axle, mm(xs["axle_radius"]), y0, y1)
    # Knee-stop pin: pressed into a blind hole in the shin's inner face, reaching into the
    # thigh groove with floor_clearance to spare.
    b = math.radians(state["a1_deg"] + state["knee_angle_deg"] + ks["pin_offset_deg"])
    reach = depth - mm(ks["floor_clearance"])
    pin = Part.makeCylinder(
        pin_r,
        reach + mm(ks["pin_press_depth"]),
        V(knee[0] + rp * math.cos(b), face - s * reach, knee[1] + rp * math.sin(b)),
        V(0, s, 0),
    )
    shin = soft(shin, mm(xs["edge_fillet"])).cut(pin)
    parts.append((shin_link, f"{side}_Shin", shin, "shell_dark"))
    parts.append((shin_link, f"{side}_KneeStopPin", pin, "bolt"))
    shin_face = outer(lg["shin_y"])
    bolt = Part.makeCylinder(
        mm(xt["knee_bolt_diameter"]) / 2,
        mm(xt["knee_bolt_proud"]),
        V(knee[0], shin_face, knee[1]),
        V(0, s, 0),
    )
    parts.append((shin_link, f"{side}_KneeBolt", soft(bolt, 0.4), "bolt"))

    # Wheel: rounded tire with knurled tread and a recessed bladed hub.
    r = mm(w["diameter"]) / 2
    width = mm(w["width"])
    yc = s * mm(w["y_center"])
    y_in = yc - width / 2
    section = mm(xw["tire_section"])
    kx, ky, kz = (mm(v) for v in xw["tread_knob"])
    # Knob tips sit on the nominal radius; the tire carcass is that much smaller.
    r_tire = r - kz + 0.2
    tire = Part.makeCylinder(r_tire, width, V(axle[0], y_in, axle[1]), V(0, 1, 0))
    tire = tire.cut(
        Part.makeCylinder(r - section, width + 2, V(axle[0], y_in - 1, axle[1]), V(0, 1, 0))
    )
    rim_edges = [e for e in tire.Edges if abs(getattr(e.Curve, "Radius", 0) - r_tire) < 1e-6]
    tire = soft(tire, mm(xw["shoulder_fillet"]), rim_edges)
    parts.append((wheel_link, f"{side}_Tire", tire, "tire"))
    recess = mm(xw["hub_recess"])
    hub = Part.makeCylinder(
        r - section,
        width - recess,
        V(axle[0], y_in if s > 0 else y_in + recess, axle[1]),
        V(0, 1, 0),
    )
    parts.append((wheel_link, f"{side}_HubDisc", hub, "hub"))
    face_y = yc + s * (width / 2 - recess)
    blade_len = r - section - 1.5
    for k in range(xw["blades"]):
        blade = Part.makeBox(
            blade_len,
            mm(xw["blade_height"]),
            mm(xw["blade_width"]),
            V(0, 0, -mm(xw["blade_width"]) / 2),
        )
        if s < 0:
            blade.translate(V(0, -mm(xw["blade_height"]), 0))
        blade.rotate(V(0, 0, 0), V(0, 1, 0), 90 + k * 360 / xw["blades"])
        blade.translate(V(axle[0], face_y, axle[1]))
        parts.append((wheel_link, f"{side}_HubBlade", blade, "hub"))
    cap = Part.makeCylinder(
        mm(xw["hub_cap_diameter"]) / 2,
        mm(xw["blade_height"]) + 0.6,
        V(axle[0], face_y, axle[1]),
        V(0, s, 0),
    )
    parts.append((wheel_link, f"{side}_HubCap", soft(cap, 0.6), "hub"))
    knobs = []
    rows = xw["tread_rows"]
    for row in range(rows):
        y_row = yc + (row - (rows - 1) / 2) * mm(xw["tread_row_pitch"])
        for k in range(xw["tread_per_row"]):
            a = 2 * math.pi * (k + 0.5 * (row % 2)) / xw["tread_per_row"]
            knob = Part.makeBox(kx, ky, kz, V(-kx / 2, -ky / 2, -0.2))
            knob.rotate(V(0, 0, 0), V(0, 0, 1), 45)
            knob.translate(V(0, 0, r_tire))
            knob.rotate(V(0, 0, 0), V(0, 1, 0), math.degrees(a))
            knob.translate(V(axle[0], y_row, axle[1]))
            knobs.append(knob)
    parts.append((wheel_link, f"{side}_Tread", Part.makeCompound(knobs), "tire"))
    return parts


def build(state):
    """Exterior parts in the hip frame (mm) at a leg state."""
    parts = [("body", "BodyShell", body_shell(), "shell"), *front_cameras(), *head()]
    for side in ("left", "right"):
        parts += leg(state, side)
    return parts


def to_world(state, parts):
    """World frame: ground at z=0, origin over the wheel contacts."""
    shift = V(-mm(state["axle_m"][0]), 0, mm(state["hip_height_m"]))
    placed = []
    for link, part, shape, color in parts:
        shape = shape.copy()
        shape.translate(shift)
        placed.append((link, part, shape, color))
    return placed


def assemble(name):
    state = pose(C["poses"][f"{name}_hip_height"], C)
    return state, to_world(state, build(state))


def overlaps(parts, same_link=False):
    """Overlaps between parts on different links (or on the same link); joint faces only touch."""
    found = []
    solid = [p for p in parts if not p[1].endswith("Tread")]
    for i, (la, na, sa, _) in enumerate(solid):
        for lb, nb, sb, _ in solid[i + 1 :]:
            if (la == lb) != same_link or not sa.BoundBox.intersect(sb.BoundBox):
                continue
            volume = sa.common(sb).Volume
            if volume > 1.0:
                found.append([na, nb, round(volume, 1)])
    return found


def views(doc, out=OUT, prefix=""):
    """Shaded GUI views for quick review when run through the FreeCAD GUI bridge."""
    import FreeCADGui as Gui
    from pivy import coin  # noqa: F401  (loads the SWIG wrappers getCameraNode needs)

    Gui.setActiveDocument(doc.Name)
    view = Gui.activeDocument().activeView()
    folder = out / "views"
    folder.mkdir(exist_ok=True)

    def look(direction):
        d = V(*direction).normalize()
        right = d.cross(V(0, 0, 1)).normalize()
        up = right.cross(d)
        back = d * -1
        m = App.Matrix(right.x, up.x, back.x, 0, right.y, up.y, back.y, 0, right.z, up.z, back.z, 0)
        # Set the Coin camera directly: setCameraOrientation animates.
        return lambda: view.getCameraNode().orientation.setValue(*App.Rotation(m).Q)

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
        view.saveImage(str(folder / f"{prefix}{name}.png"), 1200, 900, "White")


def export(assemble_pose=assemble, out=OUT, doc_name="M10Exterior", extra_checks=None):
    out.mkdir(parents=True, exist_ok=True)
    record = {"poses": {}, "units": "m", "colors": {k: list(v) for k, v in COLORS.items()}}
    checks = {}
    for name in ["crouched", "nominal", "extended"]:
        state, parts = assemble_pose(name)
        comps = []
        bound = None
        for link, part, shape, color in parts:
            pts, tris = shape.tessellate(0.1)
            comps.append(
                {
                    "name": part,
                    "link": link,
                    "color": list(COLORS[color]),
                    "color_key": color,
                    "vertices": [
                        [round(p.x / MM, 6), round(p.y / MM, 6), round(p.z / MM, 6)] for p in pts
                    ],
                    "triangles": [list(t) for t in tris],
                }
            )
            bound = shape.BoundBox if bound is None else bound.united(shape.BoundBox)
        record["poses"][name] = comps
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
            "cross_link_overlaps_mm3": overlaps(parts),
        }
        if extra_checks:
            checks[name].update(extra_checks(parts))
        if name == "nominal":
            doc = App.newDocument(doc_name)
            for _, part, shape, color in parts:
                obj = doc.addObject("Part::Feature", part)
                obj.Shape = shape
                if App.GuiUp:
                    obj.ViewObject.ShapeColor = COLORS[color]
                    obj.ViewObject.LineColor = (0.6, 0.6, 0.6)
            doc.recompute()
            # M10_FCSTD_DIR redirects the FreeCAD document (e.g. to keep a working copy).
            fcstd = Path(os.environ.get("M10_FCSTD_DIR") or out)
            fcstd.mkdir(parents=True, exist_ok=True)
            doc.saveAs(str(fcstd / f"{out.name}.FCStd"))
            if App.GuiUp:
                views(doc, out)
    with gzip.open(out / "render-mesh.json.gz", "wt") as f:
        json.dump(record, f)
    (out / "check.json").write_text(json.dumps(checks, indent=2) + "\n")
    print(doc_name, json.dumps({k: v["bounds_mm"] for k, v in checks.items()}), flush=True)
    return checks


if __name__ in ("__main__", "bridge_main"):
    export()
