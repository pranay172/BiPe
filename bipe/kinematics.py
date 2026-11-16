"""Planar two-link kinematics and nominal mass-based support feedforward."""

import math

import numpy as np

from .parameters import mass_at, total_mass
from .profiles import kinematics


def leg_fk(q, length):
    hip, knee = q
    return -length * np.array(
        [math.sin(hip) + math.sin(hip + knee), math.cos(hip) + math.cos(hip + knee)]
    )


def leg_jacobian(q, length):
    hip, knee = q
    return length * np.array(
        [
            [-math.cos(hip) - math.cos(hip + knee), -math.cos(hip + knee)],
            [math.sin(hip) + math.sin(hip + knee), math.sin(hip + knee)],
        ]
    )


def leg_ik(x, z, length):
    cosine = (x * x + z * z - 2 * length * length) / (2 * length * length)
    if not -1 <= cosine <= 1:
        raise ValueError("Requested axle position is outside the two-link workspace")
    knee = -math.acos(cosine)
    hip = math.atan2(-x, -z) - math.atan2(math.sin(knee), 1 + math.cos(knee))
    return np.array([hip, knee])


def posture(angle_deg, p=None):
    native = kinematics(p)
    if native is not None:
        return native.posture(angle_deg, p)
    a = math.radians(angle_deg)
    return np.array([a, -2 * a, a, -2 * a])


def com_and_axle(p, q):
    native = kinematics(p)
    if native is not None:
        return native.com_and_axle(p, q)
    if "rigid_bodies" in p:
        from .mass_properties import assembled_rows

        rows = assembled_rows(p, q)
        center = sum(row["mass_kg"] * np.asarray(row["com_m"]) for row in rows) / total_mass(p)
        feet = [leg_fk(pair, p["geometry"]["link_length"]) for pair in [q[:2], q[2:]]]
        return center[[0, 2]], np.mean(feet, axis=0)
    length = p["geometry"]["link_length"]
    weighted = np.array([0.0, mass_at(p, "body") * p["geometry"]["body_center_above_hip"]])
    feet = []
    for pair in [q[:2], q[2:]]:
        knee = -length * np.array([math.sin(pair[0]), math.cos(pair[0])])
        foot = leg_fk(pair, length)
        weighted += (mass_at(p, "upper_link") / 4 + mass_at(p, "knee") / 2) * knee
        weighted += mass_at(p, "lower_link") / 4 * (knee + foot)
        weighted += (mass_at(p, "axle") + mass_at(p, "wheel")) / 2 * foot
        feet.append(foot)
    return weighted / total_mass(p), np.mean(feet, axis=0)


def equilibrium(p, q):
    com, axle = com_and_axle(p, q)
    vector = com - axle
    return -math.atan2(vector[0], vector[1])


def support_feedforward(p, q, pitch, roll=0.0):
    """G - J.T F using nominal weight, not simulator contacts or inverse dynamics."""
    native = kinematics(p)
    if native is not None:
        return native.support_feedforward(p, q, pitch, roll)
    length = p["geometry"]["link_length"]
    gravity = p["sizing"]["gravity"]
    world_up_body = np.array([-math.sin(pitch), math.cos(pitch) * math.cos(roll)])
    support = total_mass(p) * gravity / 2 * world_up_body
    torques = []
    for side, pair in zip(["left", "right"], [q[:2], q[2:]], strict=True):
        jac = leg_jacobian(pair, length)
        jk = np.zeros((2, 2))
        jk[:, 0] = length * np.array([-math.cos(pair[0]), math.sin(pair[0])])
        proximal = (
            mass_at(p, "upper_link") / 4 + mass_at(p, "knee") / 2 + mass_at(p, "lower_link") / 4
        )
        distal = mass_at(p, "lower_link") / 4 + (mass_at(p, "axle") + mass_at(p, "wheel")) / 2
        if "rigid_bodies" in p:
            a, b = pair

            def derivative(angle):
                c, s = math.cos(angle), math.sin(angle)
                return np.array([[-s, c], [-c, -s]])

            u, low, wheel = [
                p["rigid_bodies"][side + "_" + role] for role in ["upper", "lower", "wheel"]
            ]
            upper_com = np.asarray(u["com_m"])[[0, 2]]
            lower_com = np.asarray(low["com_m"])[[0, 2]]
            upper_j = np.column_stack([derivative(a) @ upper_com, np.zeros(2)])
            lower_j = np.column_stack(
                [jk[:, 0] + derivative(a + b) @ lower_com, derivative(a + b) @ lower_com]
            )
            weight = (
                gravity
                * (u["mass_kg"] * upper_j + low["mass_kg"] * lower_j + wheel["mass_kg"] * jac).T
                @ world_up_body
            )
        else:
            weight = gravity * (proximal * jk + distal * jac).T @ world_up_body
        torques.extend(weight - jac.T @ support)
    return np.asarray(torques)


def smooth_segment(t, duration, start, end):
    u = float(np.clip(t / duration, 0, 1))
    scale = 10 * u**3 - 15 * u**4 + 6 * u**5
    speed = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
    return start + (end - start) * scale, (end - start) * speed


def crouch_reference(t):
    if t < 2:
        a, speed = smooth_segment(t, 2, 40, 30)
    elif t < 32:
        segment = int((t - 2) / 3)
        local = (t - 2) - 3 * segment
        a, speed = smooth_segment(
            local, 3, 30 if segment % 2 == 0 else 60, 60 if segment % 2 == 0 else 30
        )
    else:
        a, speed = smooth_segment(t - 32, 2, 30, 40)
    return posture(a), posture(speed)
