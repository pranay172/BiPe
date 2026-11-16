"""SI five-bar kinematics in the adopted CAD crank-angle convention.

Positive crank angle rotates around negative Y. Output wheels retain positive
Y spin. Kinematic placement assumes nominal belt/clutch alignment; independent
input rotor coordinates and elastic deflections live in ``m9_plant``.
"""

import math

import numpy as np


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def rotation2(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def derivative2(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[-s, -c], [c, -s]])


def height_command(angle_deg, p):
    """Inherited 30/40/60/65 degree commands -> 118/100/70/45 mm.

    Cubic smoothstep within each interval gives a continuous zero-slope join.
    Values outside the approved inherited command range are rejected.
    """
    if not math.isfinite(angle_deg) or not 30 <= angle_deg <= 65:
        raise ValueError("M9 posture command must be within 30..65 degrees")
    g = p["geometry"]
    angles = [30.0, 40.0, 60.0, 65.0]
    heights = [
        g["height_range"][1],
        g["nominal_height"],
        g["crouched_height"],
        g["height_range"][0],
    ]
    i = min(2, int(np.searchsorted(angles, angle_deg, side="right")) - 1)
    u = (angle_deg - angles[i]) / (angles[i + 1] - angles[i])
    return heights[i] + (heights[i + 1] - heights[i]) * u * u * (3 - 2 * u)


def leg_ik(x, z, p):
    g = p["geometry"]
    q = []
    for i, bx in enumerate([-g["base"] / 2, g["base"] / 2]):
        r = math.hypot(x - bx, z)
        cosine = (r * r + g["crank"] ** 2 - g["rod"] ** 2) / (2 * r * g["crank"])
        if abs(cosine) > 1:
            raise ValueError("Requested axle position is outside the five-bar workspace")
        q.append(math.atan2(z, x - bx) + (-1 if i == 0 else 1) * math.acos(cosine))
    return np.array(q)


def posture(angle_deg, p):
    pair = leg_ik(0, -height_command(angle_deg, p), p)
    return np.tile(pair, 2)


def elbows(q, p):
    g = p["geometry"]
    return np.column_stack(
        [np.array([-g["base"] / 2, g["base"] / 2]) + g["crank"] * np.cos(q), g["crank"] * np.sin(q)]
    )


def leg_fk(q, p):
    e = elbows(np.asarray(q), p)
    v = e[1] - e[0]
    distance = float(np.linalg.norm(v))
    if distance < 1e-10 or distance > 2 * p["geometry"]["rod"]:
        raise ValueError("Five-bar circles do not have a physical lower intersection")
    perpendicular = np.array([-v[1], v[0]]) / distance
    length = math.sqrt(max(0.0, p["geometry"]["rod"] ** 2 - distance * distance / 4))
    a, b = e.mean(axis=0) + length * perpendicular, e.mean(axis=0) - length * perpendicular
    return a if a[1] < b[1] else b


def leg_jacobian(q, p):
    e = elbows(np.asarray(q), p)
    vectors = leg_fk(q, p) - e
    crank_derivatives = p["geometry"]["crank"] * np.column_stack([-np.sin(q), np.cos(q)])
    return np.linalg.solve(vectors, np.diag(np.sum(vectors * crank_derivatives, axis=1)))


def rod_angles_and_jacobians(q, p):
    foot = leg_fk(q, p)
    jf = leg_jacobian(q, p)
    e = elbows(np.asarray(q), p)
    out = []
    for i in range(2):
        je = np.zeros((2, 2))
        je[:, i] = p["geometry"]["crank"] * np.array([-math.sin(q[i]), math.cos(q[i])])
        v = foot - e[i]
        theta = math.atan2(v[1], v[0])
        dt = np.array([-v[1], v[0]]) @ (jf - je) / p["geometry"]["rod"] ** 2
        out.append((theta, dt, je))
    return out


def wheel_carrier_angle_rate(q, qd, p):
    theta, jac, _ = rod_angles_and_jacobians(q, p)[0]
    return theta, float(jac @ qd)


def body_placements(p, q):
    """21 native CAD body origins/rotations relative to the chassis origin."""
    g, drive, hip = p["geometry"], p["drive"], p["joint_drive"]
    out = {"chassis": (np.zeros(3), np.eye(3))}
    for side, pair in zip(["left", "right"], [q[:2], q[2:]], strict=True):
        e = elbows(np.asarray(pair), p)
        foot = leg_fk(pair, p)
        for i, bx in enumerate([-g["base"] / 2, g["base"] / 2]):
            origin = np.array([bx, 0, 0])
            out[f"{side}_crank{i}"] = (origin, rotation(pair[i]))
            out[f"{side}_hipgear{i}"] = (origin, rotation(pair[i]))
            out[f"{side}_hiprotor{i}"] = (
                origin + [0, 0, hip["motor_z"]],
                rotation(pair[i] * hip["ratio"]),
            )
            theta = math.atan2(foot[1] - e[i, 1], foot[0] - e[i, 0])
            if i == 0:
                rod0_angle = theta
            out[f"{side}_rod{i}"] = (np.array([e[i, 0], 0, e[i, 1]]), rotation(theta))
        out[f"{side}_wheel"] = (np.array([foot[0], 0, foot[1]]), np.eye(3))
        origin, rot = out[f"{side}_rod0"]
        local = np.array(
            [g["rod"] + drive["motor_along_rod_from_foot"], 0, drive["motor_perpendicular"]]
        )
        theta = rod0_angle
        out[f"{side}_rotor"] = (
            origin + rot @ local,
            rotation((1 - drive["large_teeth"] / drive["small_teeth"]) * theta),
        )
    return out


def com_and_axle(p, q):
    place = body_placements(p, q)
    mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())
    com = (
        sum(
            r["mass_kg"] * (place[n][0] + place[n][1] @ r["com_m"])
            for n, r in p["rigid_bodies"].items()
        )
        / mass
    )
    axle = np.mean([leg_fk(pair, p) for pair in [q[:2], q[2:]]], axis=0)
    return com[[0, 2]], axle


def equilibrium(p, q):
    com, axle = com_and_axle(p, q)
    v = com - axle
    return -math.atan2(v[0], v[1])


def support_feedforward(p, q, pitch, roll=0.0):
    """Analytic gravity virtual work minus nominal equal wheel support.

    Includes all 21 CAD COMs; independently spinning centered rotor COMs use
    nominal belt alignment. It does not read contacts/inverse dynamics.
    """
    gravity = p["sizing"]["gravity"]
    up = np.array([-math.sin(pitch), math.cos(pitch) * math.cos(roll)])
    mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())
    g, drive, hip = p["geometry"], p["drive"], p["joint_drive"]
    torques = []
    for side, pair in zip(["left", "right"], [q[:2], q[2:]], strict=True):
        weighted = np.zeros((2, 2))
        rod_data = rod_angles_and_jacobians(pair, p)
        jf = leg_jacobian(pair, p)
        for role in [
            "crank0",
            "crank1",
            "hipgear0",
            "hipgear1",
            "hiprotor0",
            "hiprotor1",
            "rod0",
            "rod1",
            "wheel",
            "rotor",
        ]:
            row = p["rigid_bodies"][f"{side}_{role}"]
            local = np.array(row["com_m"])[[0, 2]]
            jc = np.zeros((2, 2))
            if role.startswith(("crank", "hipgear", "hiprotor")):
                i = int(role[-1])
                ratio = hip["ratio"] if role.startswith("hiprotor") else 1
                jc[:, i] = ratio * derivative2(ratio * pair[i]) @ local
            elif role.startswith("rod"):
                theta, dt, je = rod_data[int(role[-1])]
                jc = je + np.outer(derivative2(theta) @ local, dt)
            elif role == "wheel":
                jc = jf
            else:
                theta, dt, je = rod_data[0]
                motor_local = np.array(
                    [g["rod"] + drive["motor_along_rod_from_foot"], drive["motor_perpendicular"]]
                )
                ratio = 1 - drive["large_teeth"] / drive["small_teeth"]
                jc = je + np.outer(
                    derivative2(theta) @ motor_local + ratio * derivative2(ratio * theta) @ local,
                    dt,
                )
            weighted += row["mass_kg"] * jc
        torques.extend(gravity * (weighted - mass / 2 * jf).T @ up)
    return np.asarray(torques)


def crouch_reference(t, p):
    from .kinematics import smooth_segment

    if t < 2:
        a, speed = smooth_segment(t, 2, 40, 30)
    elif t < 32:
        segment = int((t - 2) / 3)
        a, speed = smooth_segment(
            (t - 2) - 3 * segment, 3, 30 if segment % 2 == 0 else 60, 60 if segment % 2 == 0 else 30
        )
    else:
        a, speed = smooth_segment(t - 32, 2, 30, 40)
    q = posture(a, p)
    eps = 1e-4
    lo, hi = max(30, a - eps), min(65, a + eps)
    qd = (posture(hi, p) - posture(lo, p)) / (hi - lo) * speed
    return q, qd
