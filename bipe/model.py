"""Free-base, fixed-leg-posture MuJoCo model generated from shared parameters."""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .parameters import leg_geometry, mass_at


def values(items):
    return " ".join(f"{x:.12g}" for x in items)


def generate_xml(p):
    g = p["geometry"]
    pose = leg_geometry(p)
    motor = p["motor"]
    contact = p["contact"]
    root = ET.Element("mujoco", model=p["name"])
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="true")
    ET.SubElement(
        root,
        "option",
        timestep=str(p["simulation"]["physics_dt"]),
        gravity="0 0 -9.81",
        integrator="implicitfast",
        cone="elliptic",
        iterations="80",
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(
        visual, "headlight", diffuse=".65 .65 .65", ambient=".3 .3 .3", specular=".1 .1 .1"
    )
    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="floor",
        type="2d",
        builtin="checker",
        rgb1=".24 .29 .35",
        rgb2=".29 .34 .4",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset, "material", name="floor", texture="floor", texrepeat="8 8", reflectance=".1"
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        friction=values(
            [contact[k] for k in ["sliding_friction", "torsional_friction", "rolling_friction"]]
        ),
        condim="6",
        solref=".005 1",
        solimp=".95 .99 .001",
    )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="-1 -1 3", dir=".3 .3 -1", diffuse=".8 .8 .8")
    ET.SubElement(world, "geom", name="ground", type="plane", size="0 0 .1", material="floor")
    base = ET.SubElement(
        world, "body", name="chassis", pos=values([0, 0, g["wheel_radius"] + 0.001])
    )
    ET.SubElement(base, "freejoint", name="base")
    ET.SubElement(
        base,
        "geom",
        name="body_envelope",
        type="box",
        pos=values([0, 0, pose["hip_z"] + g["body_center_above_hip"]]),
        size=values([g["body_length"] / 2, g["body_width"] / 2, g["body_height"] / 2]),
        mass=str(mass_at(p, "body")),
        rgba=".82 .87 .92 1",
    )
    ET.SubElement(
        base,
        "site",
        name="imu",
        pos=values([0, 0, pose["hip_z"] + g["body_center_above_hip"]]),
        size=".003",
        rgba=".1 .8 .5 1",
    )
    # Simple face details are massless and non-colliding. Reserved camera mass is
    # already included in the central body budget, not counted a second time.
    for sign in [-1, 1]:
        ET.SubElement(
            base,
            "geom",
            type="sphere",
            pos=values([g["body_length"] / 2 + 0.001, sign * 0.022, pose["hip_z"] + 0.045]),
            size=".009",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba=".08 .17 .23 1",
        )
    for sign, label in [(1, "left"), (-1, "right")]:
        y = sign * g["hip_half_spacing"]
        hip = [0, y, pose["hip_z"]]
        knee = [pose["knee_x"], y, pose["knee_z"]]
        axle = [0, y, 0]
        for name, start, end, loc in [
            ("upper", hip, knee, "upper_link"),
            ("lower", knee, axle, "lower_link"),
        ]:
            ET.SubElement(
                base,
                "geom",
                name=f"{label}_{name}",
                type="capsule",
                fromto=values(start + end),
                size=str(g["link_radius"]),
                mass=str(mass_at(p, loc) / 2),
                rgba=".16 .38 .56 1",
            )
        for name, pos, loc in [("hip", hip, "hip"), ("knee", knee, "knee")]:
            ET.SubElement(
                base,
                "geom",
                name=f"{label}_{name}_motor",
                type="cylinder",
                pos=values(pos),
                euler=values([math.pi / 2, 0, 0]),
                size=values([g["joint_motor_radius"], g["joint_motor_width"] / 2]),
                mass=str(mass_at(p, loc) / 2),
                rgba=".26 .34 .42 1",
            )
        ET.SubElement(
            base,
            "geom",
            name=f"{label}_wheel_motor",
            type="cylinder",
            pos=values(axle),
            euler=values([math.pi / 2, 0, 0]),
            size=values([g["wheel_motor_radius"], g["wheel_motor_width"] / 2]),
            mass=str(mass_at(p, "axle") / 2),
            rgba=".26 .34 .42 1",
        )
        wheel = ET.SubElement(
            base, "body", name=f"{label}_wheel", pos=values([0, sign * g["wheel_track"] / 2, 0])
        )
        ET.SubElement(
            wheel,
            "joint",
            name=f"{label}_wheel_joint",
            type="hinge",
            axis="0 1 0",
            damping=str(motor["viscous_friction"]),
            armature=str(motor["rotor_inertia_at_output"]),
        )
        ET.SubElement(
            wheel,
            "geom",
            name=f"{label}_tire",
            type="cylinder",
            euler=values([math.pi / 2, 0, 0]),
            size=values([g["wheel_radius"], g["wheel_width"] / 2]),
            mass=str(mass_at(p, "wheel") / 2),
            rgba=".08 .11 .15 1",
        )
        ET.SubElement(
            wheel,
            "geom",
            name=f"{label}_hub",
            type="cylinder",
            pos=values([0, sign * (g["wheel_width"] / 2 + 0.001), 0]),
            euler=values([math.pi / 2, 0, 0]),
            size=values([g["wheel_radius"] * 0.53, 0.001]),
            mass="0",
            contype="0",
            conaffinity="0",
            rgba=".3 .55 .72 1",
        )
        ET.SubElement(
            wheel,
            "geom",
            name=f"{label}_rotation_mark",
            type="sphere",
            pos=values([g["wheel_radius"] * 0.38, sign * (g["wheel_width"] / 2 + 0.003), 0]),
            size=".003",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba="1 .55 .15 1",
        )
    actuators = ET.SubElement(root, "actuator")
    for side in ["left", "right"]:
        ET.SubElement(
            actuators,
            "motor",
            name=f"{side}_drive",
            joint=f"{side}_wheel_joint",
            gear="1",
            ctrllimited="true",
            ctrlrange=values([-motor["peak_torque_per_wheel"], motor["peak_torque_per_wheel"]]),
        )
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    if "rigid_bodies" in p:
        from .kinematics import posture
        from .mass_properties import add_inertial, assembled_rows, combine

        root.find("compiler").set("inertiafromgeom", "false")
        row = combine(assembled_rows(p, posture(g["fixed_leg_angle_deg"]), include_wheels=False))
        row["com_m"][2] += pose["hip_z"]
        add_inertial(base, row)
        for sign, side in [(1, "left"), (-1, "right")]:
            add_inertial(
                base.find(f"body[@name='{side}_wheel']"), p["rigid_bodies"][side + "_wheel"]
            )
            for role, output_y, z in [
                ("hip", 0, pose["hip_z"]),
                ("knee", p["mechanics"]["knee_output_y"], pose["knee_z"]),
                ("wheel", p["mechanics"]["wheel_output_y"], 0),
            ]:
                geom = base.find(f"geom[@name='{side}_{role}_motor']")
                geom.set(
                    "pos",
                    values(
                        [
                            pose["knee_x"] if role == "knee" else 0,
                            sign * (g["hip_half_spacing"] + output_y - g["joint_motor_width"] / 2),
                            z,
                        ]
                    ),
                )
            hinge = base.find(f"body[@name='{side}_wheel']/joint")
            hinge.set("frictionloss", str(motor.get("coulomb_friction", 0)))
        from .cad_geometry import add_cad_geometry

        add_cad_geometry(root, p, fixed=True)
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def build(p):
    model = mujoco.MjModel.from_xml_string(generate_xml(p))
    data = mujoco.MjData(model)
    chassis = model.body("chassis").id
    com = model.body_ipos[chassis]
    equilibrium = -math.atan2(com[0], com[2])
    return model, data, equilibrium


def reset(model, data, equilibrium, offset_deg=0):
    mujoco.mj_resetData(model, data)
    pitch = equilibrium + math.radians(offset_deg)
    data.qpos[3:7] = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    mujoco.mj_forward(model, data)


def state(data):
    q = data.qpos[3:7]
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, q)
    R = matrix.reshape(3, 3)
    return {
        "x": float(data.qpos[0]),
        "velocity": float(data.qvel[0]),
        "pitch": math.atan2(R[0, 2], R[2, 2]),
        "pitch_rate": float(data.qvel[4]),
        "roll": math.atan2(-R[1, 2], R[2, 2]),
        "wheel_speed": data.qvel[6:8].copy(),
    }
