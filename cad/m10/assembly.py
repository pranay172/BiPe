"""M10 phase B2 assembly: approved exterior with the internal drive, power and electronics parts.

python3 tools/freecad_bridge.py cad/m10/assembly.py

Reads config/robot.json "m10" (SI) and converts to millimetres here only.
The exterior comes from cad/m10/exterior.py and is hollowed to its wall
thickness; internal parts carry the rigid link they move with. Designed
seats (bearings in housings, shafts through walls) are cut from the part
that holds them. Writes artifacts/m10/assembly/{assembly.FCStd,
render-mesh.json.gz, check.json, views/}.

Layout (B2.1 option A): four RO40 modules on one transverse axis behind the
hip drive concentric hip shafts through 4:1 belts; each knee is driven by
a 1:1 belt inside the thigh; each wheel is a GL40 hub motor with an AS5047P
on the fixed axle end.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import FreeCAD as App
import Part

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location(
    "m10_exterior", Path(__file__).with_name("exterior.py")
)
E = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(E)

C, mm, soft, V = E.C, E.mm, E.soft, E.V
INT = C["internals"]
BLD = INT["build"]
# Bearing designations per position (m10.structure.bearings, the screen's source).
BRG = C["structure"]["bearings"]
OUT = ROOT / "artifacts/m10/assembly"
Y_AXIS = V(0, 1, 0)
INTERNAL = set()
# Display colours (sRGB) for internal parts; appearance only, not physical data.
E.COLORS.update(
    {
        "motor": (0.25, 0.26, 0.28),
        "metal": (0.70, 0.71, 0.73),
        "belt": (0.10, 0.10, 0.10),
        "bearing": (0.55, 0.56, 0.58),
        "frame": (0.35, 0.37, 0.40),
        "battery": (0.20, 0.35, 0.65),
        "pcb": (0.10, 0.45, 0.25),
        "harness": (0.80, 0.32, 0.12),
        "foam": (0.16, 0.17, 0.19),
        "foam_skin": (0.12, 0.12, 0.13),
        # Skid (user decision 13 October 2026): a grey close to the tire's (0.42).
        "skid_foam": (0.36, 0.36, 0.37),
        "skid_skin": (0.44, 0.44, 0.45),
    }
)


def ycyl(r, y0, y1, x=0.0, z=0.0):
    """Cylinder about an axis parallel to y between y0 and y1 (either order)."""
    lo, hi = min(y0, y1), max(y0, y1)
    return Part.makeCylinder(r, hi - lo, V(x, lo, z), Y_AXIS)


def ring(r_out, r_in, y0, y1, x=0.0, z=0.0):
    lo, hi = min(y0, y1), max(y0, y1)
    return ycyl(r_out, lo, hi, x, z).cut(ycyl(r_in, lo - 1, hi + 1, x, z))


def ybox(x0, x1, y0, y1, z0, z1):
    lo, hi = min(y0, y1), max(y0, y1)
    return Part.makeBox(x1 - x0, hi - lo, z1 - z0, V(x0, lo, z0))


def pitch_r(teeth, pitch):
    return teeth * pitch / (2 * math.pi)


def pulley(teeth, pitch, width, bore, flange, y0, s, x=0.0, z=0.0):
    """Toothed pulley envelope at the pitch radius with two 0.5 mm flanges (mm)."""
    r = mm(pitch_r(teeth, pitch))
    y1 = y0 + s * width
    body = ycyl(r, y0, y1, x, z)
    for face in (y0, y1 - s * 0.5):
        body = body.fuse(ycyl(r + flange, face, face + s * 0.5, x, z))
    lo, hi = min(y0, y1), max(y0, y1)
    return body.removeSplitter().cut(ycyl(bore / 2, lo - 1, hi + 1, x, z))


def belt(p0, r0, p1, r1, thick, y0, y1):
    """Closed belt band on the pitch circles of two pulleys."""
    lo, hi = min(y0, y1), max(y0, y1)
    outer = E.tapered_link(p0, r0 + thick, p1, r1 + thick, lo, hi)
    return outer.cut(E.tapered_link(p0, r0, p1, r1, lo - 1, hi + 1))


def bearing(spec, y0, s, x=0.0, z=0.0):
    bore, od, width = (mm(v) for v in INT["bearings"][spec])
    return ring(od / 2, bore / 2, y0, y0 + s * width, x, z), od


def stop_slot(joint, psi, angle, face, into):
    """End-stop arc slot (m10.internals.end_stops) in a hip pulley face at the joint angle.

    A pulley point at local angle b sits at b + angle in the body frame, so the body's dowel
    at psi sweeps local angles psi - hi .. psi - lo over the stop range; the groove adds the
    dowel's half angle and a pad at each end. face: the pulley face (mm, signed y); into:
    +1 or -1 towards the groove floor. Returns (groove, pads)."""
    es = INT["end_stops"]
    lo, hi = C["physics"]["end_stops_deg"][joint]
    rp, pin_r = mm(es["pin_circle_radius"]), mm(es["pin_diameter"]) / 2
    half_w, depth = mm(es["groove_radial_width"]) / 2, mm(es["groove_depth"])
    pad_deg = math.degrees(mm(es["pad_length"]) / rp)
    pin_deg = math.degrees(math.asin(pin_r / rp))
    b0 = psi - hi + angle - pin_deg - pad_deg
    b1 = psi - lo + angle + pin_deg + pad_deg
    floor, top = face + into * depth, face + into * mm(es["pad_top_recess"])
    r0, r1 = rp - half_w, rp + half_w
    groove = E.knee_sector((0.0, 0.0), r0, r1, b0, b1, floor, face - into)
    pads = E.knee_sector((0.0, 0.0), r0, r1, b0, b0 + pad_deg, floor, top).fuse(
        E.knee_sector((0.0, 0.0), r0, r1, b1 - pad_deg, b1, floor, top)
    )
    return groove, pads


def stop_dowel(psi, y0, y1):
    es = INT["end_stops"]
    rp, a = mm(es["pin_circle_radius"]), math.radians(psi)
    return ycyl(mm(es["pin_diameter"]) / 2, y0, y1, rp * math.cos(a), rp * math.sin(a))


def bulkhead_bearing_y():
    """Inboard face of each bulkhead bearing: clear of the knee hip pulley by half the
    stack clearance, so the left and right seats leave a web at the centre plane."""
    st = INT["y_stack"]
    width = mm(INT["bearings"][BRG["BulkheadBearing"]][2])
    return mm(st["bulkhead_half"]) + mm(st["clearance"]) / 2 - width


def interior_box():
    b = C["body"]
    w, r = mm(INT["wall"]), mm(INT["inner_edge_radius"])
    length, width, height = mm(b["length"]) - 2 * w, mm(b["width"]) - 2 * w, mm(b["height"]) - 2 * w
    corner = V(mm(-b["hip_x"]) - length / 2, -width / 2, -mm(b["hip_z_above_bottom"]) + w)
    box = Part.makeBox(length, width, height, corner)
    return box.makeFillet(r, box.Edges)


def side_internals(state, side):
    """Hip drive, knee drive and hub internals for one side; returns (parts, cuts)."""
    s = 1 if side == "left" else -1
    st, hb, kb, hub = INT["y_stack"], INT["hip_belt"], INT["knee_belt"], INT["hub"]
    lg, w = C["leg"], C["wheel"]
    knee = tuple(mm(v) for v in state["knee_m"])
    axle = tuple(mm(v) for v in state["axle_m"])
    body, thigh, shin, wheel = "body", f"{side}_thigh", f"{side}_shin", f"{side}_wheel"
    kdrive = f"{side}_kneedrive"
    parts, cuts = [], []

    def add(link, name, shape, color, seats=()):
        parts.append((link, f"{side}_{name}", shape, color))
        for target in seats:
            cuts.append((target, shape))

    c = mm(st["clearance"])
    pw, mw = mm(st["pulley_width"]), mm(st["motor_module_width"])
    y_knee_pulley = mm(st["bulkhead_half"]) + c
    y_knee_motor = y_knee_pulley + pw + c
    y_thigh_motor = y_knee_motor + mw + c
    y_thigh_pulley = y_thigh_motor + mw + c
    mx = -mm(hb["centre_distance"])
    motor_r = mm(INT["motor_module"]["diameter"]) / 2
    shaft_d = mm(INT["motor_module"]["shaft_diameter"])
    flange = mm(hb["flange_proud"])
    pitch = hb["pitch"]
    r_large, r_small = mm(pitch_r(hb["large_teeth"], pitch)), mm(pitch_r(hb["small_teeth"], pitch))
    belt_t = mm(hb["belt_thickness"])
    shaft_in = mm(INT["inner_shaft"]["diameter"])
    shaft_end = mm(INT["inner_shaft"]["end_diameter"])
    so, si = (
        mm(v) for v in (INT["outer_shaft"]["outer_diameter"], INT["outer_shaft"]["inner_diameter"])
    )

    # Rear motor row: knee motor inboard, thigh motor outboard.
    add(body, "KneeMotor", ycyl(motor_r, s * y_knee_motor, s * (y_knee_motor + mw), mx), "motor")
    add(body, "ThighMotor", ycyl(motor_r, s * y_thigh_motor, s * (y_thigh_motor + mw), mx), "motor")
    add(body, "KneeMotorShaft", ycyl(shaft_d / 2, s * y_knee_pulley, s * y_knee_motor, mx), "metal")
    add(
        body,
        "ThighMotorShaft",
        ycyl(shaft_d / 2, s * (y_thigh_motor + mw), s * (y_thigh_pulley + pw), mx),
        "metal",
    )
    for name, y0 in (("Knee", y_knee_pulley), ("Thigh", y_thigh_pulley)):
        small = pulley(hb["small_teeth"], pitch, pw, shaft_d, flange, s * y0, s, mx)
        add(body, f"{name}MotorPulley", small, "metal")
        band = belt((0, 0), r_large, (mx, 0), r_small, belt_t, s * (y0 + 0.5), s * (y0 + pw - 0.5))
        add(body, f"{name}HipBelt", band, "belt")

    # Concentric hip outputs. Inner (knee drive) shaft from the bulkhead into the thigh.
    t0, t1 = (mm(v) for v in lg["thigh_y"])
    wall = mm(INT["wall"])
    end_spec = BRG["InnerShaftEndBearing"]
    end_w = mm(INT["bearings"][end_spec][2])
    y_bulk_bearing = bulkhead_bearing_y()
    brg, _ = bearing(BRG["BulkheadBearing"], s * y_bulk_bearing, s)
    add(body, "BulkheadBearing", brg, "bearing")
    end_in = mm(BLD["inner_shaft_end_inset"])
    end_y0 = t1 - end_in - end_w
    ends = ycyl(shaft_end / 2, s * y_bulk_bearing, s * y_knee_pulley).fuse(
        ycyl(shaft_end / 2, s * end_y0, s * (t1 - end_in))
    )
    # Harness: Ø5 bore from the body port to the port in the thigh gallery, which lies
    # between the knee-belt hip pulley and the end bearing (m10.internals.harness).
    hn = INT["harness"]
    bore_r, port_r = mm(hn["shaft_bore_diameter"]) / 2, mm(hn["port_diameter"]) / 2
    kp_w = mm(kb["pulley_width"])
    hf = INT["hip_flange"]
    flange_t = mm(hf["thickness"])
    # The knee-belt stack starts outboard of the hip flange on the inner wall.
    kp_y0 = t0 + wall + flange_t + c
    y_bp = mm(hn["body_port_y"])
    y_g = (kp_y0 + kp_w + end_y0) / 2
    to_knee = V(knee[0], 0, knee[1]).normalize()

    def port(y, direction, r_shaft, x=0.0, z=0.0):
        return Part.makeCylinder(port_r, r_shaft + 1, V(x, s * y, z), direction)

    harness_cut = ycyl(bore_r, s * (y_bp - port_r), s * (y_g + port_r))
    harness_cut = harness_cut.fuse(port(y_bp, V(0, 0, -1), shaft_in / 2))
    harness_cut = harness_cut.fuse(port(y_g, to_knee, shaft_in / 2))
    ends = ends.cut(harness_cut)
    add(kdrive, "InnerShaftEnds", ends, "metal")
    inner = ycyl(shaft_in / 2, s * y_knee_pulley, s * end_y0).cut(harness_cut)
    add(kdrive, "InnerShaft", inner, "metal")
    add(
        kdrive,
        "KneeHipPulley",
        pulley(hb["large_teeth"], pitch, pw, shaft_in, flange, s * y_knee_pulley, s),
        "metal",
    )
    # Knee-drive end stop (D4 option A, m10.internals.end_stops): padded groove in the knee
    # 80T pulley's inboard face; one dowel through the bulkhead serves both sides.
    es = INT["end_stops"]
    psi_k = es["knee_drive_pin_angle_deg"]
    groove, pads = stop_slot("knee_drive_a2", psi_k, state["a2_deg"], s * y_knee_pulley, s)
    cuts.append((f"{side}_KneeHipPulley", groove))
    add(kdrive, "KneeDriveStopPad", pads, "stop_pad")
    if s > 0:
        half = mm(es["knee_drive_pin_length"]) / 2
        parts.append((body, "KneeDriveStopPin", stop_dowel(psi_k, -half, half), "bolt"))
        cuts.append(("Bulkhead", stop_dowel(psi_k, -half, half)))
    knee_pitch = kb["pitch"]
    add(
        kdrive,
        "KneeBeltHipPulley",
        pulley(kb["teeth"], knee_pitch, kp_w, shaft_in, flange, s * kp_y0, s),
        "metal",
    )
    brg, od = bearing(end_spec, s * end_y0, s)
    add(thigh, "InnerShaftEndBearing", brg, "bearing", seats=[f"{side}_Thigh"])
    cap_floor = t1 - mm(C["exterior"]["thigh"]["hip_cap_depth"])
    cuts.append((f"{side}_Thigh", ycyl(od / 2, s * end_y0, s * cap_floor)))

    # Outer (thigh drive) shaft with its pulley, two 6704 bearings and a sleeve for the inner shaft.
    y_out0 = mm(INT["outer_shaft"]["inboard_y"])
    outer = ring(so / 2, si / 2, s * y_out0, s * (t0 + wall))
    add(thigh, "OuterShaft", outer, "metal")
    cuts.append((f"{side}_Thigh", ycyl(so / 2, s * y_out0, s * (t0 + wall))))
    add(
        thigh,
        "ThighHipPulley",
        pulley(hb["large_teeth"], pitch, pw, so, flange, s * y_thigh_pulley, s),
        "metal",
    )
    # Hip end stop (D4 option A): padded groove in the thigh 80T pulley's inboard face; the
    # dowel is pressed through the hip bracket arm (below, after the bracket).
    groove, pads = stop_slot(
        "thigh_a1", es["hip_pin_angle_deg"], state["a1_deg"], s * y_thigh_pulley, s
    )
    cuts.append((f"{side}_ThighHipPulley", groove))
    add(thigh, "HipStopPad", pads, "stop_pad")
    add(thigh, "Sleeve", ring(si / 2, shaft_in / 2, s * y_out0, s * (t0 + wall)), "bearing")
    # Hip flange (phase D4): machined integral with the outer shaft, on the cavity side of
    # the inner wall, screwed to the wall with six countersunk M2 from the wall's outer face.
    # Kept as its own solid so the thigh section can include it in torsion.
    f0, f1 = t0 + wall, t0 + wall + flange_t
    hip_flange = ring(mm(hf["diameter"]) / 2, si / 2, s * f0, s * f1)
    pcd = mm(hf["screw_circle_diameter"]) / 2
    hole_r, sink_r = mm(hf["screw_clearance_diameter"]) / 2, mm(hf["countersink_diameter"]) / 2
    # Screw angles are fixed to the thigh (from its link axis), clear of the section stations.
    for k in range(hf["screws"]):
        a = math.radians(state["a1_deg"] + hf["screw_phase_deg"]) + 2 * math.pi * k / hf["screws"]
        hx, hz = pcd * math.cos(a), pcd * math.sin(a)
        hole = ycyl(hole_r, s * (t0 - 1), s * (f1 + 1), hx, hz)
        sink = Part.makeCone(sink_r, hole_r, sink_r - hole_r, V(hx, s * t0, hz), V(0, s, 0))
        cuts += [(f"{side}_Thigh", hole.fuse(sink))]
        hip_flange = hip_flange.cut(hole)
    add(thigh, "HipFlange", hip_flange, "metal", seats=[f"{side}_Thigh"])
    in_bearing_y = y_out0 + 1.5
    brg, od67 = bearing("6704", s * in_bearing_y, s)
    add(body, "HipInnerBearing", brg, "bearing")
    w67 = mm(INT["bearings"]["6704"][2])
    pod0, pod1 = (mm(v) for v in lg["hip_housing_y"])
    out_bearing_y = pod1 - 0.5 - w67
    brg, _ = bearing("6704", s * out_bearing_y, s)
    add(body, "HipPodBearing", brg, "bearing")
    bore = ycyl(od67 / 2, s * (pod0 - 1), s * pod1)
    spigot_r = od67 / 2 + mm(BLD["pod_spigot_radial"])
    wall_out = mm(C["body"]["width"]) / 2
    pod_r = mm(lg["hip_housing_diameter"]) / 2
    cuts += [
        ("BodyShell", ycyl(spigot_r, s * (pod0 - 1), s * wall_out)),
        (f"{side}_HipHousing", bore),
        (f"{side}_HipHousing", ring(pod_r + 1, spigot_r, s * (pod0 - 1), s * wall_out)),
    ]

    # Hip bracket: bearing hub inboard of the thigh pulley, arm forward, web to the side wall.
    hbk = INT["hip_bracket"]
    hub_r, thick = mm(hbk["hub_diameter"]) / 2, mm(hbk["thickness"])
    yb0, yb1 = in_bearing_y - 0.5, in_bearing_y - 0.5 + thick
    ax0, ax1 = (mm(v) for v in hbk["arm_x"])
    wall_in = mm(C["body"]["width"]) / 2 - wall
    bracket = ycyl(hub_r, s * yb0, s * yb1).fuse(ybox(8, ax1, s * yb0, s * yb1, -6, 6))
    bracket = bracket.fuse(ybox(ax0, ax1, s * yb0, s * wall_in, -6, 6)).removeSplitter()
    sad = BLD["motor_saddle"]
    a = sad["half_angle_deg"]
    arc = ring(motor_r + mm(sad["thickness"]), motor_r, s * yb0, s * yb1, mx)
    reach = 3 * motor_r
    ylo = min(s * yb0, s * yb1) - 1
    corners = [V(mx, ylo, 0)] + [
        V(mx + reach * math.cos(math.radians(d)), ylo, reach * math.sin(math.radians(d)))
        for d in (-a, 0, a)
    ]
    wedge = Part.Face(Part.makePolygon([*corners, corners[0]])).extrude(V(0, abs(yb1 - yb0) + 2, 0))
    bracket = bracket.fuse(arc.common(wedge)).removeSplitter()
    bracket = bracket.cut(ycyl(od67 / 2, s * (yb0 - 1), s * (yb1 + 1)))
    add(body, "HipBracket", bracket, "frame")
    pin_y1 = y_thigh_pulley + mm(es["groove_depth"]) - mm(es["floor_clearance"])
    dowel = stop_dowel(es["hip_pin_angle_deg"], s * (pin_y1 - mm(es["hip_pin_length"])), s * pin_y1)
    add(body, "HipStopPin", dowel, "bolt", seats=[f"{side}_HipBracket"])
    so_ = BLD["motor_standoffs"]
    y_face = mm(st["bulkhead_half"])
    for ang in so_["angles_deg"]:
        x = mx + mm(so_["radius"]) * math.cos(math.radians(ang))
        z = mm(so_["radius"]) * math.sin(math.radians(ang))
        standoff = ycyl(mm(so_["diameter"]) / 2, s * y_face, s * y_knee_motor, x, z)
        add(body, "MotorStandoff", standoff, "metal")
        if s > 0:
            # Belt tension slot along x through the bulkhead; one M2.5 stud clamps the
            # left and right standoffs.
            tr, cr = mm(so_["tension_half_travel"]), mm(BLD["screw_clearance_diameter"]) / 2
            yb = y_face + 1
            slot = ycyl(cr, -yb, yb, x - tr, z).fuse(ycyl(cr, -yb, yb, x + tr, z))
            cuts.append(("Bulkhead", slot.fuse(ybox(x - tr, x + tr, -yb, yb, z - cr, z + cr))))

    # Knee: 1:1 belt inside the thigh, knee pulley and shaft fixed to the shin, BRG bearings.
    kn = INT["knee"]
    kshaft = mm(kn["shaft_diameter"])
    s0, s1 = (mm(v) for v in lg["shin_y"])
    boss_y0 = t0 - mm(kn["boss_proud"])
    knee_in, knee_out = BRG["KneeInnerBearing"], BRG["KneeOuterBearing"]
    w67b = mm(INT["bearings"][knee_in][2])
    add(thigh, "KneeBoss", ycyl(mm(kn["boss_diameter"]) / 2, s * boss_y0, s * t0, *knee), "shell")
    brg, od70 = bearing(knee_in, s * boss_y0, s, *knee)
    add(thigh, "KneeInnerBearing", brg, "bearing")
    cuts += [(f"{side}_Thigh", ycyl(od70 / 2, s * boss_y0, s * (boss_y0 + w67b), *knee))]
    cuts += [(f"{side}_KneeBoss", ycyl(od70 / 2, s * boss_y0, s * (boss_y0 + w67b), *knee))]
    w_out = mm(INT["bearings"][knee_out][2])
    brg, od_out = bearing(knee_out, s * (t1 - w_out), s, *knee)
    add(thigh, "KneeOuterBearing", brg, "bearing")
    cuts.append((f"{side}_Thigh", ycyl(od_out / 2, s * (t1 - w_out), s * t1 + s, *knee)))
    shaft = ycyl(kshaft / 2, s * boss_y0, s * s1, *knee)
    cuts.append((f"{side}_Shin", shaft))
    # Harness: knee-shaft bore from its gallery port to the channel inside the shin. The axle-tube
    # seat floor leaves a 1.2 mm skin on the shin's inner face; the channel runs 0.3 mm above it so
    # it opens into the seat without grooving the floor.
    ch_r = mm(hn["shin_channel_diameter"]) / 2
    seat_floor = s0 + 1.2
    y_shin = seat_floor + 0.3 + ch_r
    kx, kz = knee
    ax, az = axle
    to_axle = V(ax - kx, 0, az - kz)
    channel = Part.makeCylinder(ch_r, to_axle.Length, V(kx, s * y_shin, kz), to_axle)
    shaft = shaft.cut(ycyl(bore_r, s * (y_g - port_r), s * (y_shin + ch_r), *knee))
    shaft = shaft.cut(port(y_g, -to_knee, kshaft / 2, kx, kz)).cut(channel)
    add(shin, "KneeShaft", shaft, "metal")
    cuts.append((f"{side}_Shin", channel))
    bundle_r = mm(hn["bundle_diameter"]) / 2
    run = V(kx, 0, kz).Length - shaft_in / 2 - kshaft / 2 - 1.0
    add(
        thigh,
        "HarnessThigh",
        Part.makeCylinder(
            bundle_r, run, to_knee * (shaft_in / 2 + 0.5) + V(0, s * y_g, 0), to_knee
        ),
        "harness",
    )
    # Service loops (M11 phase D, m10.electronics.harness): r >= the loop radius plus half the
    # bundle, one bundle thick. Body port: half annulus below the inner shaft (the port turns with
    # a2, -30 to +37 deg from straight down between the end stops). Gallery: full annuli round
    # the inner shaft and the knee shaft in the gallery plane.
    loop_out = mm(C["electronics"]["harness"]["service_loop_radius_m"]) + bundle_r
    half_t = bundle_r
    body_loop = ring(loop_out, shaft_in / 2 + 0.5, s * (y_bp - half_t), s * (y_bp + half_t))
    body_loop = body_loop.cut(ybox(-loop_out - 1, loop_out + 1, -200, 200, 0, loop_out + 1))
    add(body, "HarnessLoopBody", body_loop, "harness")
    hip_loop = ring(loop_out, shaft_in / 2 + 0.5, s * (y_g - half_t), s * (y_g + half_t))
    add(thigh, "HarnessLoopHip", hip_loop, "harness")
    knee_loop = ring(loop_out, kshaft / 2 + 0.5, s * (y_g - half_t), s * (y_g + half_t), *knee)
    add(thigh, "HarnessLoopKnee", knee_loop, "harness")
    add(
        shin,
        "KneePulley",
        pulley(kb["teeth"], knee_pitch, kp_w, kshaft, flange, s * kp_y0, s, *knee),
        "metal",
    )
    rk = mm(pitch_r(kb["teeth"], knee_pitch))
    by0 = kp_y0 + (kp_w - mm(kb["belt_width"])) / 2
    add(
        thigh,
        "KneeBelt",
        belt((0, 0), rk, knee, rk, belt_t, s * by0, s * (by0 + mm(kb["belt_width"]))),
        "belt",
    )

    # Hub: GL40 stator base on the shin face, rotor can in the hub, encoder on the axle end.
    gd, gh = (mm(v) for v in hub["motor_envelope"])
    tube_od, tube_id = (mm(v) for v in hub["axle_tube"])
    base_h = 1.0
    add(
        shin,
        "HubStatorBase",
        ring(gd / 2 - 0.5, tube_od / 2, s * s1, s * (s1 + base_h), *axle),
        "motor",
    )
    add(
        wheel,
        "HubRotor",
        ring(gd / 2, tube_od / 2, s * (s1 + base_h), s * (s1 + gh), *axle),
        "motor",
    )
    pcb_d, pcb_t = (mm(v) for v in hub["encoder_pcb"])
    mag_d, mag_t = (mm(v) for v in hub["magnet"])
    hub_wall = mm(hub["hub_wall"])
    face = mm(w["y_center"] + w["width"] / 2) - mm(C["exterior"]["wheel"]["hub_recess"])
    mag_y0 = face - hub_wall
    chip_top = mag_y0 - mm(hub["air_gap"])
    pcb_y0 = chip_top - mm(hub["chip_height"]) - pcb_t
    # The tube stands on the seat floor, so its bore is closed on the shin's inner face; the
    # harness channel enters it through the tube wall.
    tube_y0 = seat_floor
    tube = ring(tube_od / 2, tube_id / 2, s * tube_y0, s * pcb_y0, *axle).cut(channel)
    add(shin, "AxleTube", tube, "metal")
    cuts.append((f"{side}_Shin", ycyl(tube_od / 2, s * tube_y0, s * pcb_y0, *axle)))
    tube_r = tube_od / 2
    span = to_axle.Length - kshaft / 2 - tube_r - 1.0
    start = V(kx, s * y_shin, kz) + V(to_axle).normalize() * (kshaft / 2 + 0.5)
    add(shin, "HarnessShin", Part.makeCylinder(bundle_r, span, start, to_axle), "harness")
    add(shin, "EncoderPCB", ycyl(pcb_d / 2, s * pcb_y0, s * (pcb_y0 + pcb_t), *axle), "pcb")
    chip = ybox(-2, 2, s * (pcb_y0 + pcb_t), s * chip_top, -2, 2)
    chip.translate(V(axle[0], 0, axle[1]))
    add(shin, "EncoderChip", chip, "lens")
    magnet = ycyl(mag_d / 2, s * mag_y0, s * (mag_y0 + mag_t), *axle)
    add(wheel, "EncoderMagnet", magnet, "metal", seats=[f"{side}_HubDisc"])
    clr = mm(hub["cavity_clearance"])
    cuts += [
        (f"{side}_HubDisc", ycyl(gd / 2 + clr, s * s1, s * (s1 + gh), *axle)),
        (f"{side}_HubDisc", ycyl(pcb_d / 2 + 2.5, s * (s1 + gh - 0.1), s * mag_y0, *axle)),
    ]
    return parts, cuts


def body_internals():
    parts = []
    bh = INT["bulkhead"]
    half = mm(INT["y_stack"]["bulkhead_half"])
    x0, x1 = (mm(v) for v in bh["x"])
    z0, z1 = (mm(v) for v in bh["z"])
    inner = interior_box()
    bb = inner.BoundBox
    bulk = ybox(x0, x1, -half, half, z0, z1)
    # Feet to the tub floor and the cover, where the bulkhead is screwed.
    for fx0, fx1 in BLD["bulkhead_feet_x"]:
        bulk = bulk.fuse(ybox(mm(fx0), mm(fx1), -half, half, bb.ZMin, bb.ZMax))
    bulk = bulk.removeSplitter()
    for hx_, hz_, hd_ in BLD["bulkhead_holes_xzd"]:
        bulk = bulk.cut(ycyl(mm(hd_) / 2, -half - 1, half + 1, mm(hx_), mm(hz_)))
    od = mm(INT["bearings"][BRG["BulkheadBearing"]][1])
    y0 = bulkhead_bearing_y()
    for s in (1, -1):
        bulk = bulk.cut(ycyl(od / 2, s * y0, s * (half + 1)))
    parts.append(("body", "Bulkhead", bulk, "frame"))
    bat = INT["battery"]
    bx, by, bz = (mm(v) for v in bat["envelope_xyz"])
    front = bb.XMax - mm(bat["x_front_from_interior"])
    parts.append(
        ("body", "Battery", ybox(front - bx, front, -by / 2, by / 2, -bz / 2, bz / 2), "battery")
    )
    mx_, my_, mz_ = (mm(v) for v in INT["moteus"]["envelope"])
    row_x = -mm(INT["hip_belt"]["centre_distance"]) - 5
    y_row = half + 0.5 + my_ / 2
    for s in (1, -1):
        for zsign, zface in ((1, bb.ZMax), (-1, bb.ZMin)):
            zc = zface - zsign * (mz_ / 2 + 0.5)
            parts.append(
                (
                    "body",
                    "Moteus",
                    ybox(
                        row_x - mx_ / 2,
                        row_x + mx_ / 2,
                        s * (y_row - my_ / 2),
                        s * (y_row + my_ / 2),
                        zc - mz_ / 2,
                        zc + mz_ / 2,
                    ),
                    "pcb",
                )
            )
        y_side = by / 2 + 0.5
        parts.append(
            (
                "body",
                "Moteus",
                ybox(front - mx_, front, s * y_side, s * (y_side + mz_), -my_ / 2, my_ / 2),
                "pcb",
            )
        )
    return parts + electronics_parts()


def board_shapes(key):
    """PCB, module and parts solids of an M11 board in the body frame (mm).

    Footprint body boxes come from the KiCad layout (artifacts/m11/boards/<key>/envelopes.json);
    heights and the board placement come from m10.electronics.boards."""
    boards = C["electronics"]["boards"]
    b, heights = boards[key], boards["component_heights_m"]
    env = json.loads((ROOT / f"artifacts/m11/boards/{key}/envelopes.json").read_text())
    origin = V(*(mm(v) for v in b["origin_body_m"]))
    ax, ay, n = (V(*b[k]) for k in ("board_x_axis_body", "board_y_axis_body", "front_normal_body"))
    t = mm(b["thickness_m"])

    def box(x0, x1, y0, y1, w0, w1):
        pts = [
            origin + ax * u + ay * v + n * w for u in (x0, x1) for v in (y0, y1) for w in (w0, w1)
        ]
        lo = [min(getattr(p, k) for p in pts) for k in "xyz"]
        hi = [max(getattr(p, k) for p in pts) for k in "xyz"]
        return ybox(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])

    width, height = env["size_mm"]
    module, rest = [], []
    for part in env["parts"]:
        h = heights[part["footprint"]]
        w0, w1 = (t + mm(h[0]), t + mm(h[1])) if isinstance(h, list) else (t, t + mm(h))
        teensy = part["footprint"].startswith(("Teensy40", "PinHeader"))
        (module if teensy else rest).append(box(*part["box_mm"], w0, w1))

    def fused(shapes):
        return shapes[0].fuse(shapes[1:]).removeSplitter() if shapes else None

    return box(0, width, 0, height, 0, t), fused(module), fused(rest)


def switch_shapes():
    """Rear-wall slide switch (body and actuator) and the wall slot it needs (mm)."""
    sw = INT["electronics"]["switch"]
    x_in = interior_box().BoundBox.XMin
    wall = mm(INT["wall"])
    yc, zc = (mm(v) for v in sw["centre_yz_m"])
    bx, by, bz = (mm(v) for v in sw["body_xyz_m"])
    ay, az = (mm(v) for v in sw["actuator_yz_m"])
    c = mm(sw["slot_clearance_m"])
    body = ybox(x_in, x_in + bx, yc - by / 2, yc + by / 2, zc - bz / 2, zc + bz / 2)
    actuator = ybox(
        x_in - mm(sw["actuator_length_m"]), x_in, yc - ay / 2, yc + ay / 2, zc - az / 2, zc + az / 2
    )
    sy = mm(sw["travel_m"]) + ay + 2 * c
    slot = ybox(
        x_in - wall - 1, x_in + 0.1, yc - sy / 2, yc + sy / 2, zc - az / 2 - c, zc + az / 2 + c
    )
    return body.fuse(actuator).removeSplitter(), slot


def electronics_parts():
    """M11 electronics on the body link: the two boards, the BEC, the Pi and the switch."""
    el = INT["electronics"]
    parts = []
    for key, name in (("carrier", "Carrier"), ("power", "Power")):
        pcb, module, rest = board_shapes(key)
        parts.append(("body", f"{name}PCB", pcb, "pcb"))
        if module is not None:
            parts.append(("body", "Teensy", module, "frame"))
        parts.append(("body", f"{name}Parts", rest, "frame"))
    parts.append(("body", "BEC", ybox(*(mm(v) for v in el["bec_box_m"])), "pcb"))
    parts.append(("body", "PiZero", ybox(*(mm(v) for v in el["pi_zero_box_m"])), "pcb"))
    parts.append(("body", "PowerSwitch", switch_shapes()[0], "frame"))
    return parts


def head_internals():
    """Hollow the head pod; LED boards behind the eye lenses; camera module behind the bezel."""
    b, h, xh, hi = C["body"], C["head"], C["exterior"]["head"], INT["head"]
    cx = mm(-b["hip_x"] + h["x"])
    top = -mm(b["hip_z_above_bottom"]) + mm(b["height"])
    r = mm(h["pod_height"]) / 2
    length = mm(h["pod_depth_x"])
    half = mm(h["eye_spacing_y"]) / 2
    zc = top + mm(h["neck_height"]) + r - mm(xh["base_flat_from_bottom"])
    back, front = cx - length / 2, cx + length / 2
    t = mm(hi["shell_wall"])
    rim = mm(xh["eye_rim_recess"])
    # Barrels end one wall behind the eye lens seat (exterior.head); a window through
    # that wall leaves a ledge the lens sits on.
    lens_back = front - rim - 0.5
    x0, x1 = back + mm(xh["rear_round"]), lens_back - t
    eye_r = mm(h["eye_diameter"]) / 2
    window_r = eye_r - 0.2 - mm(hi["eye_lens_ledge"])
    cavity, windows = None, []
    for s in (1, -1):
        barrel = Part.makeCylinder(r - t, x1 - x0, V(x0, s * half, zc), V(1, 0, 0))
        cavity = barrel if cavity is None else cavity.fuse(barrel)
        # Separate cut: fused into the cavity, the window's axis on the web's side plane
        # gives a degenerate boolean.
        windows.append(
            ("HeadPod", Part.makeCylinder(window_r, t + 1, V(x1 - 0.5, s * half, zc), V(1, 0, 0)))
        )
    base_z = zc - r + mm(xh["base_flat_from_bottom"])
    recess = mm(xh["front_recess"])
    # The outer web starts 3 mm past the rear rounds (exterior.head); keep its rear wall.
    web_x0 = x0 + 3.0 + t
    web = Part.makeBox(
        front - recess - t - web_x0,
        2 * half,
        zc + (xh["web_height_ratio"] - 1) * r - t - (base_z + t),
        V(web_x0, -half, base_z + t),
    )
    cavity = cavity.fuse(web).removeSplitter()
    cavity = cavity.cut(
        Part.makeBox(length, 4 * (half + r), r, V(back, -2 * (half + r), base_z + t - r))
    )
    parts = []
    led_d, led_t = (mm(v) for v in hi["led_board"])
    for s in (1, -1):
        x_led = x1 - mm(hi["led_board_set_back"]) - led_t
        parts.append(
            (
                "body",
                "EyeLEDBoard",
                Part.makeCylinder(led_d / 2, led_t, V(x_led, s * half, zc), V(1, 0, 0)),
                "pcb",
            )
        )
    cw, ch, cd = (mm(v) for v in hi["camera_module"])
    lens_w = mm(hi["camera_lens_block"][0])
    stack = mm(hi["camera_board_stack"])
    below = mm(hi["camera_lens_centre_from_bottom"])
    cam_z, pitch, _ = E.camera_bezel(front, zc)
    cam_x1 = front - recess - 1.5 - 0.3
    board = Part.makeBox(stack, cw, ch, V(cam_x1 - cd, -cw / 2, cam_z - below))
    lens = Part.makeBox(
        cd - stack, lens_w, lens_w, V(cam_x1 - cd + stack, -lens_w / 2, cam_z - lens_w / 2)
    )
    aperture = Part.makeBox(
        7, lens_w + 1, lens_w + 1, V(cam_x1 - 5, -(lens_w + 1) / 2, cam_z - (lens_w + 1) / 2)
    )
    for shape in (board, lens, aperture):
        shape.rotate(*pitch)
    # The bezel is a separate insert: a seat with a fit gap, backed by a boss one wall
    # thick so the pitched seat leaves no skin against the vertical web wall.
    gap = mm(hi["camera_bezel_gap"])
    seat = E.camera_bezel(front, zc, gap)[2]
    # Open the seat forward along x: the pitched bezel's top sits behind the front plane,
    # and a closed seat would leave a knife-edge lip of barrel material over it.
    nose = max(
        (f for f in seat.Faces if f.Surface.__class__.__name__ == "Plane"),
        key=lambda f: f.normalAt(0, 0).x,
    )
    seat_open = nose.extrude(V(5, 0, 0))
    cavity = cavity.cut(E.camera_bezel(front, zc, gap + t)[2])
    # TPU ear roots (phase D4): a local boss inside the barrel wall under each ear root for
    # its three M2 heat-set inserts (m10.physics.bumpers.ear.retention).
    ret = C["physics"]["bumpers"]["ear"]["retention"]
    boss_t = mm(ret["member_thickness_m"])
    pitch = mm(ret["bolt_spacing_m"]) / (ret["per_interface"] - 1)
    margin = 2 * mm(ret["edge_center_distance_m"])
    for s in (1, -1):
        shell = Part.makeCylinder(r - t, length, V(back, s * half, zc), V(1, 0, 0)).cut(
            Part.makeCylinder(r - t - boss_t, length, V(back, s * half, zc), V(1, 0, 0))
        )
        # Insert positions along the ear root line; the boss is the shell sector (radial
        # side faces, so square edges) that covers them with the edge margin.
        roots = [
            E.place_ear(Part.Vertex(V((k - (ret["per_interface"] - 1) / 2) * pitch, 0, 0)), s).Point
            for k in range(ret["per_interface"])
        ]
        angles = [math.atan2(p.z - zc, s * (p.y - s * half)) for p in roots]
        da = margin / (r - t)
        a0, a1 = min(angles) - da, max(angles) + da
        x0, x1 = min(p.x for p in roots) - margin, max(p.x for p in roots) + margin
        reach = 2 * r
        sector = [V(x0, s * half, zc)] + [
            V(x0, s * half + s * reach * math.cos(a), zc + reach * math.sin(a))
            for a in (a0, (a0 + a1) / 2, a1)
        ]
        prism = Part.Face(Part.makePolygon([*sector, sector[0]])).extrude(V(x1 - x0, 0, 0))
        cavity = cavity.cut(shell.common(prism))
    parts.append(("body", "CameraBoard", board, "pcb"))
    parts.append(("body", "CameraLensBlock", lens, "lens"))
    return parts, [
        ("HeadPod", cavity),
        *windows,
        ("HeadPod", seat),
        ("HeadPod", seat_open),
        ("HeadPod", aperture),
    ]


def thigh_cavity(state, side):
    """Thigh interior: inner wall INT.wall, outer wall INT.thigh_outer_wall, edges filleted at
    the outer edge fillet minus the wall so the wall stays uniform around the rims."""
    s = 1 if side == "left" else -1
    xt, lg = C["exterior"]["thigh"], C["leg"]
    wall, outer = mm(INT["wall"]), mm(INT["thigh_outer_wall"])
    t0, t1 = (mm(v) for v in lg["thigh_y"])
    knee = tuple(mm(v) for v in state["knee_m"])
    y_in, y_out = s * (t0 + wall), s * (t1 - outer)
    cavity = E.tapered_link(
        (0, 0),
        mm(xt["hip_radius"]) - wall,
        knee,
        mm(xt["knee_radius"]) - wall,
        min(y_in, y_out),
        max(y_in, y_out),
    )
    rim = mm(xt["edge_fillet"])
    inner_edges = [
        e
        for e in cavity.Edges
        if abs(e.BoundBox.YMax - e.BoundBox.YMin) < 1e-6 and abs(e.BoundBox.YMin - y_in) < 1e-6
    ]
    outer_edges = [
        e
        for e in cavity.Edges
        if abs(e.BoundBox.YMax - e.BoundBox.YMin) < 1e-6 and abs(e.BoundBox.YMin - y_out) < 1e-6
    ]
    cavity = cavity.makeFillet(rim - wall, inner_edges)
    outer_edges = [
        e
        for e in cavity.Edges
        if abs(e.BoundBox.YMax - e.BoundBox.YMin) < 1e-6 and abs(e.BoundBox.YMin - y_out) < 1e-6
    ]
    cavity = cavity.makeFillet(rim - outer, outer_edges)
    # Knee bearing seats deeper than the walls keep a boss of knee-boss diameter inside the
    # cavity (phase D4 6800s), so each seat holds its bearing over the full width.
    boss_r = mm(INT["knee"]["boss_diameter"]) / 2
    in_end = t0 - mm(INT["knee"]["boss_proud"]) + mm(INT["bearings"][BRG["KneeInnerBearing"]][2])
    out_start = t1 - mm(INT["bearings"][BRG["KneeOuterBearing"]][2])
    if in_end > t0 + wall:
        cavity = cavity.cut(ycyl(boss_r, s * (t0 + wall - 0.1), s * in_end, *knee))
    if out_start < t1 - outer:
        cavity = cavity.cut(ycyl(boss_r, s * out_start, s * (t1 - outer + 0.1), *knee))
    # Knee-stop groove (phase D4 arc slot): the lid is solid behind it, from the lid split
    # (split_and_fasten) to the outer wall, out to the groove radius plus boss_margin, and
    # bored to the outer knee bearing's diameter for the bearing and the knee shaft.
    ks = INT["knee_stop"]
    groove_r = mm(ks["pin_circle_radius"] + ks["groove_radial_width"] / 2 + ks["boss_margin"])
    bore_r = mm(INT["bearings"][BRG["KneeOuterBearing"]][1]) / 2
    y_split = t1 - mm(xt["edge_fillet"]) - 0.2
    cavity = cavity.cut(ring(groove_r, bore_r, s * y_split, s * (t1 - outer + 0.1), *knee))
    return (f"{side}_Thigh", cavity)


def protection_parts(state):
    """Phase D4 fall protection in the hip frame: the frame-mounted skid under the tub and
    a foam ring on each wheel's outer face, the plant's shapes (m10.physics.bumpers). Each
    pad is LD45 foam behind a 1 mm TPU wear skin; each ring sits on a PA12 carrier that
    fills the hub recess, so it bears on the hub disc and the tire face."""
    bp = C["physics"]["bumpers"]
    skin = mm(bp["skin_thickness_m"])
    parts = []

    def cup(pad, backing):
        """Uniform TPU skin over every face but the backing face; the foam is the rest,
        carried out to the backing face through `backing` (the skin-deep slab under it,
        inset by the skin at the sides)."""
        core = pad.makeOffsetShape(-skin, 0.01, join=2).fuse(pad.common(backing))
        core = core.removeSplitter()
        return core, pad.cut(core)

    sk = bp["skid"]
    lx, ly, lz = (mm(v) for v in sk["size_m"])
    cx, top = mm(sk["center_x_m"]), -mm(C["body"]["hip_z_above_bottom"])
    flat = mm(sk.get("flat_length_m", sk["size_m"][0]))
    rise = (lx - flat) / 2 * math.tan(math.radians(sk.get("end_bevel_deg", 0.0)))
    prof = [
        (cx - lx / 2, top),
        (cx + lx / 2, top),
        (cx + lx / 2, top - lz + rise),
        (cx + flat / 2, top - lz),
        (cx - flat / 2, top - lz),
        (cx - lx / 2, top - lz + rise),
    ]
    poly = Part.makePolygon(
        [V(x, -ly / 2, z) for x, z in prof] + [V(prof[0][0], -ly / 2, prof[0][1])]
    )
    pad = Part.Face(poly).extrude(V(0, ly, 0))
    slab = Part.makeBox(
        lx - 2 * skin,
        ly - 2 * skin,
        skin + 0.1,
        V(cx - lx / 2 + skin, -ly / 2 + skin, top - skin - 0.1),
    )
    foam, wear = cup(pad, slab)
    parts += [("body", "SkidPad", foam, "skid_foam"), ("body", "SkidSkin", wear, "skid_skin")]
    wr = bp["wheel_ring"]
    r_in, r_out, t = (mm(wr[k]) for k in ("inner_radius_m", "outer_radius_m", "thickness_m"))
    rim = t - (r_out - r_in) * math.tan(math.radians(wr.get("dome_deg", 0.0)))
    w, xw = C["wheel"], C["exterior"]["wheel"]
    ax, az = (mm(v) for v in state["axle_m"])
    hub_r = mm(w["diameter"] / 2 - xw["tire_section"])
    for side in ("left", "right"):
        s = 1 if side == "left" else -1
        face = mm(w["y_center"] + w["width"] / 2)
        prof = [(r_in, 0.0), (r_out, 0.0), (r_out, rim), (r_in, t)]
        poly = Part.makePolygon([V(ax + r, s * (face + d), az) for r, d in prof + prof[:1]])
        ring_ = Part.Face(poly).revolve(V(ax, 0, az), V(0, 1, 0), 360)
        slab = ring(r_out - skin, r_in + skin, s * face, s * (face + skin + 0.1), ax, az)
        foam, wear = cup(ring_, slab)
        carrier = ring(hub_r, r_in, s * (face - mm(xw["hub_recess"])), s * face, ax, az)
        parts += [
            (f"{side}_wheel", f"{side}_WheelRing", foam, "foam"),
            (f"{side}_wheel", f"{side}_WheelRingSkin", wear, "foam_skin"),
            (f"{side}_wheel", f"{side}_WheelRingCarrier", carrier, "hub"),
        ]
    return parts


def zcyl(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def split_and_fasten(parts):
    """Body shell -> cover, tub and battery hatch with screw columns; head -> front and rear;
    neck cable bore. Parts keep their link; only exterior shells change."""
    b, h, xh = C["body"], C["head"], C["exterior"]["head"]
    bb = interior_box().BoundBox
    zs = mm(BLD["shell_split_z"])
    bottom = -mm(b["hip_z_above_bottom"])
    top = bottom + mm(b["height"])
    big = 1000.0
    upper = Part.makeBox(2 * big, 2 * big, big, V(-big, -big, zs))
    lower = Part.makeBox(2 * big, 2 * big, big, V(-big, -big, zs - big))
    col_r = mm(BLD["screw_column_diameter"]) / 2
    clr_r = mm(BLD["screw_clearance_diameter"]) / 2
    cb_d, cb_h = (mm(v) for v in BLD["screw_counterbore"])
    ins_d, ins_h = (mm(v) for v in BLD["insert_hole"])
    cols_low, cols_up, holes_low, holes_up = [], [], [], []
    for cx_, cy_ in BLD["screw_columns_xy"]:
        for sy in (1, -1):
            x, y = mm(cx_), sy * mm(cy_)
            cols_low.append(zcyl(col_r, bb.ZMin, zs, x, y))
            cols_up.append(zcyl(col_r, zs, bb.ZMax, x, y))
            holes_low.append(zcyl(clr_r, bottom - 1, zs + 0.1, x, y))
            holes_low.append(zcyl(cb_d / 2, bottom - 1, bottom + cb_h, x, y))
            holes_up.append(zcyl(ins_d / 2, zs - 0.1, zs + ins_h, x, y))
    # Battery hatch in the tub floor. Its front edge lies on a radial plane through the
    # shared centre of the inner and outer front-bottom fillets, so both parts keep
    # square edges where the opening crosses the curved shell.
    bat = INT["battery"]
    bx, by, _ = (mm(v) for v in bat["envelope_xyz"])
    front = bb.XMax - mm(bat["x_front_from_interior"])
    m_back, m_side = (mm(v) for v in BLD["battery_hatch_margin"])
    hz1 = mm(BLD["battery_hatch_top_z"])
    fillet_cx = mm(-b["hip_x"] + b["length"] / 2 - b["edge_radius"])
    fillet_cz = bottom + mm(b["edge_radius"])
    phi = math.radians(BLD["battery_hatch_front_cut_deg"])
    g = mm(BLD["hatch_gap"])

    def hatch_prism(shrink):
        x0 = front - bx - m_back + shrink
        cx_, cz_ = fillet_cx - shrink * math.cos(phi), fillet_cz - shrink * math.sin(phi)
        zb = bottom - 4
        t_ = (cz_ - zb) / math.cos(phi)
        x_top = cx_ - (hz1 - cz_) * math.tan(phi)  # radial line at the prism top
        pts = [(x0, zb), (x0, hz1), (x_top, hz1), (cx_ + t_ * math.sin(phi), zb)]
        y0 = by / 2 + m_side - shrink
        poly = Part.makePolygon([V(x, -y0, z) for x, z in pts] + [V(pts[0][0], -y0, pts[0][1])])
        return Part.Face(poly).extrude(V(0, 2 * y0, 0))

    hatch_box = hatch_prism(0.0)
    hatch_inner = hatch_prism(g)
    # Head split and neck bore.
    cx = mm(-b["hip_x"] + h["x"])
    r = mm(h["pod_height"]) / 2
    zc = top + mm(h["neck_height"]) + r - mm(xh["base_flat_from_bottom"])
    base_z = zc - r + mm(xh["base_flat_from_bottom"])
    xs = cx + mm(BLD["head_split_x_from_centre"])
    rear = Part.makeBox(big, 2 * big, 2 * big, V(xs - big, -big, -big))
    fore = Part.makeBox(big, 2 * big, 2 * big, V(xs, -big, -big))
    bore = zcyl(
        mm(BLD["neck_bore_diameter"]) / 2,
        bb.ZMax - 1,
        base_z + mm(INT["head"]["shell_wall"]) + 1,
        cx,
    )

    out = []
    for link, name, shape, color in parts:
        if name == "BodyShell":
            tub = shape.common(lower).fuse(cols_low).removeSplitter().cut(holes_low)
            hatch = tub.common(hatch_inner)
            tub = tub.cut(hatch_box)
            cover = shape.common(upper).fuse(cols_up).removeSplitter().cut(holes_up).cut(bore)
            out += [
                (link, "BodyCover", cover, color),
                (link, "BodyTub", tub, color),
                (link, "BatteryHatch", hatch, "shell_dark"),
            ]
        elif name == "HeadPod":
            out += [
                (link, "HeadRear", shape.common(rear).cut(bore), color),
                (link, "HeadFront", shape.common(fore).cut(bore), color),
            ]
        elif name == "Neck":
            out.append((link, name, shape.cut(bore), color))
        elif name.endswith("_Thigh"):
            # Thigh body and an outer lid at the outer wall, so the knee belt, pulleys and
            # outer bearings can be fitted; the lid carries the hip cap and panel inlays.
            # The split sits 0.2 mm below the cavity's outer edge fillet (thigh_cavity), so
            # the body ends in a square rim and the lid carries the rounded edge.
            s = 1 if name.startswith("left") else -1
            outer = mm(INT["thigh_outer_wall"])
            fillet = mm(C["exterior"]["thigh"]["edge_fillet"]) - outer
            y_split = s * (mm(C["leg"]["thigh_y"][1]) - outer - fillet - 0.2)
            outboard = Part.makeBox(
                2 * big, big, 2 * big, V(-big, y_split if s > 0 else y_split - big, -big)
            )
            side = name[: -len("_Thigh")]
            out += [
                (link, name, shape.cut(outboard), color),
                (link, f"{side}_ThighLid", shape.common(outboard), color),
            ]
        else:
            out.append((link, name, shape, color))
    return out


def ear_root(flap, s, pod, proud=1.5):
    """TPU ear ending on the head pod: the flap's root end is trimmed square (in the ear
    frame) where its outer face stands at least `proud` mm off the pod, then clipped to
    the pod surface. A plain clip would leave a feather edge where the flap dives into
    the barrel at a shallow angle."""
    ear = C["head"]["ear"]
    e_w, thick = mm(ear["width"]), mm(ear["thickness"])
    xs, ys = (-e_w / 2 + 3, 0.0, e_w / 2 - 3), (-thick / 2 + 0.5, thick / 2 - 0.5)

    def clearance(x, y, z):
        v = E.place_ear(Part.Vertex(V(x, y, z)), s)
        d = pod.distToShape(v)[0]
        return -d if pod.isInside(v.Point, 1e-6, True) else d

    z_cut = 0.0
    while z_cut < mm(ear["length"]) / 2:
        if all(max(clearance(x, y, z_cut) for y in ys) >= proud for x in xs):
            break
        z_cut += 0.5
    stub = Part.makeBox(2 * e_w, 2 * thick, 20 + z_cut, V(-e_w, -thick, -20))
    return flap.cut(E.place_ear(stub, s)).cut(pod)


def build(state):
    ext = E.build(state)
    head_parts, cuts = head_internals()
    internal = body_internals() + head_parts + protection_parts(state)
    # The TPU ears end on the head pod's outer surface (their root tabs are screwed to the
    # bosses inside the wall); the hub blades are notched into the ring carriers.
    pod = next(shape for _, name, shape, _ in ext if name == "HeadPod")
    ears = iter((1, -1))
    ext = [(lk, n, ear_root(sh, next(ears), pod) if n == "Ear" else sh, c) for lk, n, sh, c in ext]
    for _, name, shape, _ in ext:
        if name.endswith("_HubBlade"):
            cuts.append((name.replace("HubBlade", "WheelRingCarrier"), shape))
    cuts.append(("BodyShell", interior_box()))
    cuts.append(("BodyShell", switch_shapes()[1]))
    for side in ("left", "right"):
        p, k = side_internals(state, side)
        internal += p
        cuts += k
        cuts.append(thigh_cavity(state, side))
    by_name = {}
    for target, shape in cuts:
        by_name.setdefault(target, []).append(shape)
    hollowed = []
    for link, name, shape, color in ext:
        for tool in by_name.pop(name, []):
            shape = shape.cut(tool)
        hollowed.append((link, name, shape, color))
    # Internal parts that are themselves seat targets (knee bosses).
    cut_internal = []
    for link, name, shape, color in internal:
        for tool in by_name.pop(name, []):
            shape = shape.cut(tool)
        cut_internal.append((link, name, shape, color))
    internal = cut_internal
    INTERNAL.update(n for _, n, _, _ in internal)
    unused = sorted(by_name)
    if unused:
        print("M10 ASSEMBLY unused seat targets", unused, flush=True)
    return split_and_fasten(hollowed) + internal


def assemble(name):
    state = pose(C["poses"][f"{name}_hip_height"], C)
    return state, E.to_world(state, build(state))


def internal_checks(parts):
    """Same-link overlaps that involve an internal part; exterior-only pairs are checked in B1."""
    found = E.overlaps(parts, same_link=True)
    shapes = {n: sh for _, n, sh, _ in parts}
    hn = C["internals"]["harness"]
    # Knee-stop pin hole to the shin harness channel: bundle gap less the channel clearance.
    web = (
        shapes["left_KneeStopPin"].distToShape(shapes["left_HarnessShin"])[0]
        - mm(hn["shin_channel_diameter"] - hn["bundle_diameter"]) / 2
    )
    return {
        "same_link_overlaps_mm3": [
            f
            for f in found
            if (f[0] in INTERNAL or f[1] in INTERNAL)
            # A service loop joins the bundle it serves.
            and not ("Harness" in f[0] and "Harness" in f[1])
        ],
        "knee_stop_pin_channel_web_mm": round(web, 2),
    }


def cutaway_views(doc_name="M10Assembly"):
    """GUI views with the outer shells see-through so the internals show."""
    doc = App.getDocument(doc_name)
    shells = (
        "BodyCover",
        "BodyTub",
        "BatteryHatch",
        "HeadFront",
        "HeadRear",
        "Neck",
        "Thigh",
        "ThighLid",
        "ThighPanel",
        "HipCap",
        "HipHousing",
        "Shin",
        "Tire",
        "Tread",
        "HubDisc",
    )
    for obj in doc.Objects:
        if obj.Name.split("_")[-1].rstrip("0123456789") in shells or obj.Name in shells:
            obj.ViewObject.Transparency = 82
    E.views(doc, OUT, prefix="cutaway_")


if __name__ in ("__main__", "bridge_main"):
    E.export(assemble, OUT, "M10Assembly", internal_checks)
    if App.GuiUp:
        cutaway_views()
