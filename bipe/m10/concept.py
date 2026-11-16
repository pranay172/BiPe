"""M10 phase A concept: serial-leg kinematics, coarse mass layout and static sizing.

Body frame: origin at the hip axis on the midplane, x forward, z up. Each leg
is hip -> thigh -> knee -> shin -> axle with the knee behind the hip-axle
line, as on the reference robot (config leg.knee = "back"). The coaxial hip module drives the thigh's absolute angle a1
and, through a 1:1 belt in the thigh, the shin's absolute angle a2; both
motors are body-fixed. Masses are concept allocations (config m10), not CAD.
"""

import json
import math

import numpy as np

from bipe.parameters import ROOT

G = 9.81


def load(path=None):
    return json.loads((path or ROOT / "config/robot.json").read_text())["m10"]


def leg_ik(axle, c):
    """Absolute thigh and shin angles placing the axle at body-frame (x, z)."""
    l1, l2 = c["leg"]["thigh_length"], c["leg"]["shin_length"]
    x, z = axle
    d = math.hypot(x, z)
    if not abs(l1 - l2) < d < l1 + l2:
        raise ValueError(f"Axle at {d:.4f} m is outside the leg workspace")
    along = (l1**2 - l2**2 + d**2) / (2 * d)
    across = math.sqrt(l1**2 - along**2)
    ux, uz = x / d, z / d
    # Two intersections, one each side of the hip-axle line; pick by the x of the
    # knee's offset from that line (behind = more negative x for leg.knee "back").
    candidates = [(along * ux - s * across * uz, along * uz + s * across * ux) for s in (1, -1)]
    side = {"forward": 1, "back": -1}[c["leg"]["knee"]]
    knee = max(candidates, key=lambda k: side * (k[0] - (k[0] * ux + k[1] * uz) * ux))
    a1 = math.atan2(knee[1], knee[0])
    a2 = math.atan2(z - knee[1], x - knee[0])
    return a1, a2, np.array(knee)


def leg_points(a1, a2, c):
    l1, l2 = c["leg"]["thigh_length"], c["leg"]["shin_length"]
    knee = l1 * np.array([math.cos(a1), math.sin(a1)])
    axle = knee + l2 * np.array([math.cos(a2), math.sin(a2)])
    return knee, axle


def masses(c):
    """Point masses for the balance: CAD rigid bodies when config m10.cad exists, else the concept.

    Returns (body, legs). body: [(name, kg, x, z)] in the hip frame. legs: per-side
    {segment: (kg, (x, z) in the segment frame)}; the thigh frame has its origin at
    the hip and x toward the knee, the shin frame origin at the knee and x toward
    the axle, the wheel frame origin at the axle with hip-frame axes.
    """
    if "cad" in c:
        return cad_masses(c)
    b, h, m, lay = c["body"], c["head"], c["mass_budget_kg"], c["layout"]
    centre_x = -b["hip_x"]
    bottom = -b["hip_z_above_bottom"]
    top = bottom + b["height"]
    head_z = top + h["neck_height"] + h["pod_height"] / 2
    body = [
        ("battery", m["battery_6S_850"], lay["battery_x"], lay["battery_z"]),
        (
            "electronics",
            m["electronics_six_moteus_c1_carrier_power"],
            lay["electronics_x"],
            lay["electronics_z"],
        ),
        # Rear motor row; output pulleys, shafts and bearings stay on the hip axis.
        (
            "motor_row",
            m["hip_modules_four_RO40_with_reductions_bearings"] - lay["hip_module_mass_at_hip_kg"],
            lay["motor_row_x"],
            lay["motor_row_z"],
        ),
        ("hip_outputs", lay["hip_module_mass_at_hip_kg"], 0.0, 0.0),
        ("shell_frame", m["body_shell_and_frame"], centre_x, bottom + b["height"] / 2),
        ("harness_fasteners", m["harness"] + m["fasteners"], centre_x, bottom + 0.035),
        ("head", m["head_pod_eyes_ears_neck"] + m["head_pan_reserve"], centre_x + h["x"], head_z),
    ]
    l1, l2 = c["leg"]["thigh_length"], c["leg"]["shin_length"]
    legs = {
        "thigh": (
            (m["thigh_shin_structure_and_knee_belts"] * 0.6 + m["leg_covers"] * 0.7) / 2,
            (l1 / 2, 0.0),
        ),
        "shin": (
            (m["thigh_shin_structure_and_knee_belts"] * 0.4 + m["leg_covers"] * 0.3) / 2,
            (l2 / 2, 0.0),
        ),
        "wheel": ((m["hub_motors_two_GL40"] + m["wheel_rims_tires_encoders"]) / 2, (0.0, 0.0)),
    }
    return body, legs


def cad_masses(c):
    """CAD rigid bodies (config m10.cad, from cad/m10/mass_properties.py) plus allowances."""
    b, h, cad = c["body"], c["head"], c["cad"]
    rb = cad["rigid_bodies"]
    allow = c["mass_properties"]["allowances_kg"]
    com = rb["body"]["com_link_m"]
    body = [("body_cad", rb["body"]["mass_kg"], com[0], com[2])]
    # Knee drive trains and RO40 rotors spin on body-fixed axes with their
    # centres of mass on those axes, so statically they are body masses.
    for side in ("left", "right"):
        for role in ("kneedrive", "thighrotor", "kneerotor"):
            r = rb[f"{side}_{role}"]
            origin, angle = r["frame"]["origin_hip_m"], math.radians(r["frame"]["angle_deg"])
            x, z = np.asarray(origin) + rotate(angle, (r["com_link_m"][0], r["com_link_m"][2]))
            body.append((f"{side}_{role}", r["mass_kg"], float(x), float(z)))
    # Harness and fasteners sit at the centre of mass of the body-mounted CAD.
    mounted = sum(m for _, m, _, _ in body)
    x0 = sum(m * x for _, m, x, _ in body) / mounted
    z0 = sum(m * z for _, m, _, z in body) / mounted
    top = -b["hip_z_above_bottom"] + b["height"]
    head_z = top + h["neck_height"] + h["pod_height"] / 2
    body += [
        ("harness_fasteners", allow["harness"] + allow["fasteners_and_inserts"], x0, z0),
        ("head_pan_reserve", allow["head_pan_reserve"], -b["hip_x"] + h["x"], head_z),
    ]
    legs = {}
    for seg in ("thigh", "shin", "wheel"):
        r = rb[f"left_{seg}"]
        legs[seg] = (r["mass_kg"], (r["com_link_m"][0], r["com_link_m"][2]))
    return body, legs


def rotate(angle, v):
    """Rotate an (x, z) vector by angle (rad) in the sagittal plane."""
    ca, sa = math.cos(angle), math.sin(angle)
    return np.array([v[0] * ca - v[1] * sa, v[0] * sa + v[1] * ca])


def pose(hip_height, c, iterations=30):
    """Level-body equilibrium: axle under the whole-robot centre of mass."""
    body, legs = masses(c)
    r = c["wheel"]["diameter"] / 2
    z_axle = -(hip_height - r)
    x_axle = 0.0
    for _ in range(iterations):
        a1, a2, knee = leg_ik((x_axle, z_axle), c)
        knee, axle = leg_points(a1, a2, c)
        parts = [(name, mass, x, z) for name, mass, x, z in body]
        for side in (1, -1):
            parts += [
                ("thigh", legs["thigh"][0], *rotate(a1, legs["thigh"][1])),
                ("shin", legs["shin"][0], *(knee + rotate(a2, legs["shin"][1]))),
                ("wheel", legs["wheel"][0], *(axle + np.array(legs["wheel"][1]))),
            ]
        total = sum(p[1] for p in parts)
        com = np.array([sum(p[1] * p[2] for p in parts), sum(p[1] * p[3] for p in parts)]) / total
        x_axle = com[0]
    return {
        "hip_height_m": hip_height,
        "a1_deg": math.degrees(a1),
        "a2_deg": math.degrees(a2),
        "knee_angle_deg": math.degrees(a2 - a1),
        "knee_m": knee.tolist(),
        "axle_m": axle.tolist(),
        "com_m": com.tolist(),
        "com_height_m": float(com[1] + hip_height),
        "mass_kg": total,
        "legs": legs,
    }


def pose_at(hip_height, dx, c):
    """Leg state with the axle dx (m) fore-aft of the level-body balance point.

    Same IK as ``pose``; no balance search, so the result is a kinematic pose
    (accelerating, on a slope or pushed), not an equilibrium.
    """
    balanced = pose(hip_height, c)
    z_axle = -(hip_height - c["wheel"]["diameter"] / 2)
    a1, a2, _ = leg_ik((balanced["axle_m"][0] + dx, z_axle), c)
    knee, axle = leg_points(a1, a2, c)
    # Continuous ranges through the low crouch: thigh angle in (-360, 0], knee in [0, 360).
    a1_deg = (math.degrees(a1) % 360) - 360
    return {
        "hip_height_m": hip_height,
        "axle_offset_m": dx,
        "a1_deg": a1_deg,
        "a2_deg": math.degrees(a2),
        "knee_angle_deg": (math.degrees(a2) - a1_deg) % 360,
        "knee_m": knee.tolist(),
        "axle_m": axle.tolist(),
    }


def static_torques(state, c):
    """Per-leg torques on the thigh (a1) and shin (a2) drives, N m.

    Virtual work with the ground reaction at the contact point under the axle
    and gravity on the leg segments; the body is level and in equilibrium.
    """
    a1, a2 = math.radians(state["a1_deg"]), math.radians(state["a2_deg"])
    l1, l2 = c["leg"]["thigh_length"], c["leg"]["shin_length"]
    (m_t, c_t), (m_s, c_s), (m_w, _) = (state["legs"][k] for k in ("thigh", "shin", "wheel"))
    normal = state["mass_kg"] * G / 2
    # Vertical rates of the knee/axle and of the segment centres of mass per joint angle.
    d1 = l1 * math.cos(a1)
    d2 = l2 * math.cos(a2)
    t1 = rotate(a1 + math.pi / 2, c_t)[1]
    s2 = rotate(a2 + math.pi / 2, c_s)[1]
    # Generalized forces that the drives must resist (positive = lifting).
    tau1 = normal * d1 - G * (m_t * t1 + (m_s + m_w) * d1)
    tau2 = normal * d2 - G * (m_s * s2 + m_w * d2)
    return {"thigh_drive_Nm": float(tau1), "shin_drive_Nm": float(tau2)}


def envelope(state, c):
    """Overall length, width and height at a pose (blockout boxes)."""
    b, h, w, leg = c["body"], c["head"], c["wheel"], c["leg"]
    hip = state["hip_height_m"]
    centre_x = -b["hip_x"]
    r = w["diameter"] / 2
    xs = [
        centre_x - b["length"] / 2,
        centre_x + b["length"] / 2,
        centre_x + h["x"] + h["pod_depth_x"] / 2 + h["eye_protrusion"],
        state["axle_m"][0] - r,
        state["axle_m"][0] + r,
        state["knee_m"][0] + leg["thigh_width"] / 2,
    ]
    top = hip - b["hip_z_above_bottom"] + b["height"] + h["neck_height"] + h["pod_height"]
    width = 2 * (w["y_center"] + w["width"] / 2)
    return {
        "length_m": max(xs) - min(xs),
        "width_m": width,
        "height_m": top,
        "body_ground_clearance_m": hip - b["hip_z_above_bottom"],
    }


def report(c=None):
    c = c or load()
    out = {}
    for name in ["crouched", "nominal", "extended"]:
        state = pose(c["poses"][f"{name}_hip_height"], c)
        state.pop("legs")
        state.update(static_torques(pose(c["poses"][f"{name}_hip_height"], c), c))
        state.update(envelope(state, c))
        out[name] = state
    out["allocated_mass_kg"] = sum(c["mass_budget_kg"].values())
    if "cad" in c:
        out["mass_source"] = "cad"
        out["cad_allocated_mass_kg"] = c["cad"]["allocated_kg"]
    return out


if __name__ == "__main__":
    print(json.dumps(report(), indent=2))
