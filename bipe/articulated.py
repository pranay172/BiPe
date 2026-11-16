"""Free-root six-hinge robot with backward-folded torque-controlled legs."""

import math
import xml.etree.ElementTree as ET
from functools import lru_cache

import mujoco
import numpy as np

from .kinematics import equilibrium, leg_fk, posture
from .mass_properties import add_inertial
from .model import generate_xml as fixed_xml
from .model import values
from .parameters import mass_at
from .profiles import plant

JOINTS = ["left_hip", "left_knee", "right_hip", "right_knee"]
WHEELS = ["left_wheel_joint", "right_wheel_joint"]


def generate_xml(p):
    native = plant(p)
    if native is not None:
        return native.generate_xml(p)
    # Reuse the verified renderer/contact defaults, replacing only the robot tree.
    root = ET.fromstring(fixed_xml(p))
    root.set("model", "bipe_articulated_v1")
    root.find("option").set("gravity", values([0, 0, -p["sizing"]["gravity"]]))
    world = root.find("worldbody")
    world.remove(world.find("body"))
    # The fixed model already contains CAD visuals; rebuild them in moving frames.
    for mesh in list(root.find("asset").findall("mesh")):
        root.find("asset").remove(mesh)
    root.remove(root.find("actuator"))
    root.remove(root.find("sensor"))
    g, joint = p["geometry"], p["articulation"]
    if joint["knee_fold_direction"] != -1:
        raise ValueError("This joint convention requires backward-folded knees")
    length = g["link_length"]
    base = ET.SubElement(world, "body", name="chassis", pos="0 0 .2")
    ET.SubElement(base, "freejoint", name="base")
    ET.SubElement(
        base,
        "geom",
        name="body_envelope",
        type="box",
        pos=values([0, 0, g["body_center_above_hip"]]),
        size=values([g["body_length"] / 2, g["body_width"] / 2, g["body_height"] / 2]),
        mass=str(mass_at(p, "body")),
        rgba=".82 .87 .92 1",
    )
    ET.SubElement(
        base, "site", name="imu", pos=values([0, 0, g["body_center_above_hip"]]), size=".003"
    )
    for sign, side in [(1, "left"), (-1, "right")]:
        ET.SubElement(
            base,
            "geom",
            name=f"{side}_eye",
            type="sphere",
            pos=values([g["body_length"] / 2 + 0.001, sign * 0.022, 0.045]),
            size=".009",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba=".08 .17 .23 1",
        )
        y = sign * g["hip_half_spacing"]
        cylinder(
            base,
            f"{side}_hip_motor",
            [0, y, 0],
            g["joint_motor_radius"],
            g["joint_motor_width"],
            mass_at(p, "hip") / 2,
        )
        upper = ET.SubElement(base, "body", name=f"{side}_upper", pos=values([0, y, 0]))
        hinge(upper, f"{side}_hip", joint["hip_limits_deg"], joint)
        link(upper, f"{side}_upper_link", length, g["link_radius"], mass_at(p, "upper_link") / 2)
        cylinder(
            upper,
            f"{side}_knee_motor",
            [0, 0, -length],
            g["joint_motor_radius"],
            g["joint_motor_width"],
            mass_at(p, "knee") / 2,
        )
        lower = ET.SubElement(upper, "body", name=f"{side}_lower", pos=values([0, 0, -length]))
        hinge(lower, f"{side}_knee", joint["knee_limits_deg"], joint)
        link(lower, f"{side}_lower_link", length, g["link_radius"], mass_at(p, "lower_link") / 2)
        cylinder(
            lower,
            f"{side}_wheel_motor",
            [0, 0, -length],
            g["wheel_motor_radius"],
            g["wheel_motor_width"],
            mass_at(p, "axle") / 2,
        )
        wheel = ET.SubElement(
            lower,
            "body",
            name=f"{side}_wheel",
            pos=values([0, sign * (g["wheel_track"] / 2 - g["hip_half_spacing"]), -length]),
        )
        ET.SubElement(
            wheel,
            "joint",
            name=f"{side}_wheel_joint",
            type="hinge",
            axis="0 1 0",
            limited="false",
            damping=str(p["motor"]["viscous_friction"]),
            armature=str(p["motor"]["rotor_inertia_at_output"]),
        )
        cylinder(
            wheel,
            f"{side}_tire",
            [0, 0, 0],
            g["wheel_radius"],
            g["wheel_width"],
            mass_at(p, "wheel") / 2,
            ".08 .11 .15 1",
        )
        ET.SubElement(wheel, "site", name=f"{side}_axle", size=".001")
        cylinder(
            wheel,
            f"{side}_hub",
            [0, sign * (g["wheel_width"] / 2 + 0.001), 0],
            g["wheel_radius"] * 0.53,
            0.002,
            0,
            ".3 .55 .72 1",
            cosmetic=True,
        )
        ET.SubElement(
            wheel,
            "geom",
            name=f"{side}_rotation_mark",
            type="sphere",
            pos=values([g["wheel_radius"] * 0.38, sign * (g["wheel_width"] / 2 + 0.003), 0]),
            size=".003",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba="1 .55 .15 1",
        )
    actuator = ET.SubElement(root, "actuator")
    for name in JOINTS + WHEELS:
        peak = (
            joint["peak_torque_per_joint"]
            if name in JOINTS
            else p["motor"]["peak_torque_per_wheel"]
        )
        ET.SubElement(
            actuator,
            "motor",
            name=name + "_drive",
            joint=name,
            gear="1",
            ctrllimited="true",
            ctrlrange=values([-peak, peak]),
        )
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    for name in JOINTS + WHEELS:
        ET.SubElement(sensors, "jointpos", name=name + "_encoder", joint=name)
    if "rigid_bodies" in p:
        root.find("compiler").set("inertiafromgeom", "false")
        for body in base.iter("body"):
            add_inertial(body, p["rigid_bodies"][body.get("name")])
        # Drawing-based housing offsets: output faces lie on the joint axis
        # plane; the stationary motor mass extends inboard of that face.
        c = p["mechanics"]
        width = g["joint_motor_width"]
        for sign, side in [(1, "left"), (-1, "right")]:
            for role, output_y, parent_y, z in [
                ("hip", 0, sign * g["hip_half_spacing"], 0),
                ("knee", c["knee_output_y"], 0, -length),
                ("wheel", c["wheel_output_y"], 0, -length),
            ]:
                geom = base.find(f".//geom[@name='{side}_{role}_motor']")
                geom.set("pos", values([0, parent_y + sign * (output_y - width / 2), z]))
            # Contact surrogates retain real housing/wheel outer envelopes.
            # The links use smaller capsules; full assembly interference is
            # checked independently in FreeCAD, not inferred from these.
        for hinge_node in base.iter("joint"):
            spec = joint if hinge_node.get("name") in JOINTS else p["motor"]
            hinge_node.set("frictionloss", str(spec.get("coulomb_friction", 0)))
        from .cad_geometry import add_cad_geometry

        add_cad_geometry(root, p)
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def cylinder(parent, name, pos, radius, width, mass, color=".26 .34 .42 1", cosmetic=False):
    attrs = {
        "name": name,
        "type": "cylinder",
        "pos": values(pos),
        "euler": values([math.pi / 2, 0, 0]),
        "size": values([radius, width / 2]),
        "mass": str(mass),
        "rgba": color,
    }
    if cosmetic:
        attrs.update(contype="0", conaffinity="0")
    ET.SubElement(parent, "geom", **attrs)


def link(parent, name, length, radius, mass):
    ET.SubElement(
        parent,
        "geom",
        name=name,
        type="capsule",
        fromto=values([0, 0, 0, 0, 0, -length]),
        size=str(radius),
        mass=str(mass),
        rgba=".16 .38 .56 1",
    )


def hinge(parent, name, limits, joint):
    ET.SubElement(
        parent,
        "joint",
        name=name,
        type="hinge",
        axis="0 1 0",
        limited="true",
        range=values(np.radians(limits)),
        damping=str(joint["viscous_friction"]),
        armature=str(joint["rotor_inertia_at_output"]),
    )


def build(p):
    model = mujoco.MjModel.from_xml_string(generate_xml(p))
    return model, mujoco.MjData(model)


def reset(model, data, p, angle_deg=40, offset_deg=0):
    native = plant(p)
    if native is not None:
        return native.reset(model, data, p, angle_deg, offset_deg)
    mujoco.mj_resetData(model, data)
    q = posture(angle_deg)
    for name, value in zip(JOINTS, q, strict=True):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
    pitch = equilibrium(p, q) + math.radians(offset_deg)
    data.qpos[3:7] = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    foot = leg_fk(q[:2], p["geometry"]["link_length"])
    rotated = np.array(
        [
            math.cos(pitch) * foot[0] + math.sin(pitch) * foot[1],
            -math.sin(pitch) * foot[0] + math.cos(pitch) * foot[1],
        ]
    )
    data.qpos[0] = -rotated[0]
    data.qpos[2] = p["geometry"]["wheel_radius"] - 0.0001 - rotated[1]
    mujoco.mj_forward(model, data)


@lru_cache(maxsize=8)
def topology(model):
    return (
        np.array([model.jnt_qposadr[model.joint(n).id] for n in JOINTS]),
        np.array([model.jnt_dofadr[model.joint(n).id] for n in JOINTS]),
        np.array([model.jnt_dofadr[model.joint(n).id] for n in WHEELS]),
        [model.site(side + "_axle").id for side in ["left", "right"]],
        model.body("chassis").id,
        model.sensor("gyro").adr[0],
    )


def truth(model, data, p):
    native = plant(p)
    if native is not None:
        return native.truth(model, data, p)
    qi, vi, wi, sites, chassis, gyro_adr = topology(model)
    R = data.xmat[chassis].reshape(3, 3)
    pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
    roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    q, qd, wheel_speed = data.qpos[qi].copy(), data.qvel[vi].copy(), data.qvel[wi].copy()
    positions, velocities = [], []
    for ix in sites:
        positions.append(data.site_xpos[ix].copy())
        v = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, ix, v, 0)
        velocities.append(v[3:])
    axle = np.mean(positions, axis=0)
    velocity = np.mean(velocities, axis=0)
    gyro = data.sensordata[gyro_adr : gyro_adr + 3]
    pitch_rate = gyro[1] * math.cos(roll) - gyro[2] * math.sin(roll)
    yaw_rate = (gyro[1] * math.sin(roll) + gyro[2] * math.cos(roll)) / math.cos(pitch)
    return {
        "position": axle[:2],
        "velocity": float(velocity[:2] @ [math.cos(yaw), math.sin(yaw)]),
        "pitch": pitch,
        "roll": roll,
        "yaw": yaw,
        "pitch_rate": float(pitch_rate),
        "yaw_rate": float(yaw_rate),
        "q": q,
        "qd": qd,
        "wheel_speed": wheel_speed,
        "height": float(data.qpos[2] - axle[2]),
        "equilibrium": equilibrium(p, q),
    }
