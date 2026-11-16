"""SI serial-leg kinematics for the M10 plant, in the CAD angle convention.

Joint coordinates per side are the two drive outputs, q = [a1, a2]:
a1 is the thigh's angle and a2 the knee-drive pulley's angle, both measured
in the body x-z plane from +x towards +z, which is positive rotation about
body -Y. The 1:1 knee belt makes a2 the shin's body-frame angle, so the knee
angle is a2 - a1. Placements assume rigid belts; belt deflection and the
independent rotor coordinates live in ``bipe.m10.plant``.
"""

import math

import numpy as np

SIDES = ["left", "right"]
# Rigid-body roles per side, as exported by cad/m10/mass_properties.py.
ROLES = ["thigh", "kneedrive", "shin", "wheel", "thighrotor", "kneerotor"]


def rotation(angle):
    """Rotation by a CAD angle (about body -Y)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def derivative2(angle):
    """d/dangle of the planar rotation acting on (x, z)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[-s, -c], [c, -s]])


def rotation2(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def _smoothstep(knots, values, label):
    i = min(len(knots) - 2, int(np.searchsorted(knots, label, side="right")) - 1)
    u = (label - knots[i]) / (knots[i + 1] - knots[i])
    return values[i] + (values[i + 1] - values[i]) * u * u * (3 - 2 * u)


def posture_target(angle_deg, p):
    """Posture label (30..65) -> axle depth and x below the hip, by smoothstep knots.

    The depth is the hip-to-axle height (hip height above the ground minus the
    wheel radius). The axle x knots are the level-body balance points at each
    knot depth (``balance_x``), so a held posture keeps the body near level.
    """
    if not math.isfinite(angle_deg) or not 30 <= angle_deg <= 65:
        raise ValueError("M10 posture command must be within 30..65 degrees")
    m = p["posture_mapping"]
    knots = m["command_deg"]
    return (
        _smoothstep(knots, m["axle_depth_m"], angle_deg),
        _smoothstep(knots, m["axle_x_m"], angle_deg),
    )


def height_command(angle_deg, p):
    return posture_target(angle_deg, p)[0]


def leg_ik(x, z, p):
    """Drive angles [a1, a2] placing the axle at (x, z) in the hip frame, knee behind."""
    g = p["geometry"]
    l1, l2 = g["thigh_length"], g["shin_length"]
    d = math.hypot(x, z)
    if not abs(l1 - l2) < d < l1 + l2:
        raise ValueError("Requested axle position is outside the serial-leg workspace")
    along = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
    across = math.sqrt(l1 * l1 - along * along)
    ux, uz = x / d, z / d
    # Knee behind the hip-axle line (leg.knee "back"): the intersection whose
    # offset from that line points towards -x.
    candidates = [(along * ux - s * across * uz, along * uz + s * across * ux) for s in (1, -1)]
    knee = min(candidates, key=lambda k: k[0] - (k[0] * ux + k[1] * uz) * ux)
    a1 = math.atan2(knee[1], knee[0])
    # Continuous through the low crouch, where the thigh passes straight back.
    if a1 > 0:
        a1 -= 2 * math.pi
    a2 = math.atan2(z - knee[1], x - knee[0])
    return np.array([a1, a2])


def leg_fk(q, p):
    g = p["geometry"]
    a1, a2 = q
    return g["thigh_length"] * np.array([math.cos(a1), math.sin(a1)]) + g["shin_length"] * np.array(
        [math.cos(a2), math.sin(a2)]
    )


def leg_jacobian(q, p):
    g = p["geometry"]
    a1, a2 = q
    return np.column_stack(
        [
            g["thigh_length"] * np.array([-math.sin(a1), math.cos(a1)]),
            g["shin_length"] * np.array([-math.sin(a2), math.cos(a2)]),
        ]
    )


def knee_point(q, p):
    a1 = q[0]
    return p["geometry"]["thigh_length"] * np.array([math.cos(a1), math.sin(a1)])


def posture(angle_deg, p):
    height, x = posture_target(angle_deg, p)
    return np.tile(leg_ik(x, -height, p), 2)


def wheel_carrier_angle_rate(q, qd, p):
    """Shin body-frame angle and rate (CAD convention): the wheel encoder's carrier."""
    return float(q[1]), float(qd[1])


def body_placements(p, q):
    """Rigid-frame origins and rotations relative to the chassis frame (hip axis, midplane)."""
    g = p["geometry"]
    ratio = p["joint_drive"]["ratio"]
    rotor = np.array([g["motor_row_x"], 0.0, g["motor_row_z"]])
    out = {"chassis": (np.zeros(3), np.eye(3))}
    for side, pair in zip(SIDES, [q[:2], q[2:]], strict=True):
        a1, a2 = pair
        knee, axle = knee_point(pair, p), leg_fk(pair, p)
        out[f"{side}_thigh"] = (np.zeros(3), rotation(a1))
        out[f"{side}_kneedrive"] = (np.zeros(3), rotation(a2))
        out[f"{side}_shin"] = (np.array([knee[0], 0, knee[1]]), rotation(a2))
        out[f"{side}_wheel"] = (np.array([axle[0], 0, axle[1]]), np.eye(3))
        out[f"{side}_thighrotor"] = (rotor, rotation(ratio * a1))
        out[f"{side}_kneerotor"] = (rotor, rotation(ratio * a2))
    return out


def com_and_axle(p, q):
    place = body_placements(p, q)
    rb = p["rigid_bodies"]
    mass = sum(r["mass_kg"] for r in rb.values())
    com = sum(r["mass_kg"] * (place[n][0] + place[n][1] @ r["com_m"]) for n, r in rb.items()) / mass
    axle = np.mean([leg_fk(pair, p) for pair in [q[:2], q[2:]]], axis=0)
    return com[[0, 2]], axle


def equilibrium(p, q):
    com, axle = com_and_axle(p, q)
    v = com - axle
    return -math.atan2(v[0], v[1])


def com_jacobians(p, pair, side):
    """Per-body planar COM Jacobians d(x, z)/d[a1, a2] for one side."""
    ratio = p["joint_drive"]["ratio"]
    a1, a2 = pair
    jk = np.column_stack(
        [p["geometry"]["thigh_length"] * np.array([-math.sin(a1), math.cos(a1)]), np.zeros(2)]
    )
    out = {}
    for role in ROLES:
        local = np.asarray(p["rigid_bodies"][f"{side}_{role}"]["com_m"])[[0, 2]]
        jc = np.zeros((2, 2))
        if role == "thigh":
            jc[:, 0] = derivative2(a1) @ local
        elif role == "kneedrive":
            jc[:, 1] = derivative2(a2) @ local
        elif role == "shin":
            jc = jk.copy()
            jc[:, 1] += derivative2(a2) @ local
        elif role == "wheel":
            jc = leg_jacobian(pair, p)
        elif role == "thighrotor":
            jc[:, 0] = ratio * derivative2(ratio * a1) @ local
        else:
            jc[:, 1] = ratio * derivative2(ratio * a2) @ local
        out[role] = jc
    return out


def support_feedforward(p, q, pitch, roll=0.0):
    """Analytic gravity virtual work minus nominal equal wheel support, on [a1, a2] per side.

    Uses every CAD rigid body with rigid belts; it does not read contacts or
    inverse dynamics.
    """
    gravity = p["sizing"]["gravity"]
    up = np.array([-math.sin(pitch), math.cos(pitch) * math.cos(roll)])
    mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())
    torques = []
    for side, pair in zip(SIDES, [q[:2], q[2:]], strict=True):
        weighted = sum(
            p["rigid_bodies"][f"{side}_{role}"]["mass_kg"] * jc
            for role, jc in com_jacobians(p, pair, side).items()
        )
        torques.extend(gravity * (weighted - mass / 2 * leg_jacobian(pair, p)).T @ up)
    return np.asarray(torques)


def balance_x(depth, p, iterations=40):
    """Axle x at a hip-to-axle depth that puts the whole-robot COM over the axle, body level."""
    x = 0.0
    for _ in range(iterations):
        q = np.tile(leg_ik(x, -depth, p), 2)
        x = float(com_and_axle(p, q)[0][0])
    return x


def crouch_reference(t, p):
    from ..kinematics import smooth_segment

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
