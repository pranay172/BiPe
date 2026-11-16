"""Free-base M9 five-bar plant with finite elastic belts and slip clutches.

All inertial bodies use adopted SI CAD frames/tensors. No robot pose or velocity
is prescribed after reset. Five-bar connect equalities model the real foot
hinges. FOC lag, belt dead zones and finite clutch friction are physical drive
effects, not pose stabilizers. Contact laws are stated digital assumptions.
"""

import copy
import gzip
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from functools import lru_cache

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from .m9_kinematics import (
    body_placements,
    equilibrium,
    leg_fk,
    posture,
    rod_angles_and_jacobians,
    support_feedforward,
)
from .mass_properties import add_inertial, combine
from .model import values
from .parameters import ROOT

JOINTS = ["left_hip", "left_knee", "right_hip", "right_knee"]
WHEELS = ["left_wheel_joint", "right_wheel_joint"]
ROTORS = [f"{s}_hip{i}_rotor_joint" for s in ["left", "right"] for i in range(2)] + [
    "left_rotor_joint",
    "right_rotor_joint",
]
GEARS = [f"{s}_hip{i}_gear_joint" for s in ["left", "right"] for i in range(2)]


def parameters(shared):
    """Flatten the separate M9 CAD/physics profiles without changing M4."""
    cad, phys = shared["m9_cad"], shared["m9_physics"]
    p = copy.deepcopy(cad["parameters"])
    p.update(profile="m9", name="bipe_m9_five_bar", units="SI")
    p["rigid_bodies"] = copy.deepcopy(cad["rigid_bodies"])
    p["nominal_world"] = copy.deepcopy(cad["nominal_world"])
    p["physics"] = copy.deepcopy(phys)
    p["m9_physics"] = p["physics"]
    material_path = ROOT / phys["contact"]["material_source"]
    p["contact_materials"] = json.loads(material_path.read_text())
    p["geometry_source_sha256"] = hashlib.sha256(
        json.dumps(cad, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    p["collision_mesh_sha256"] = hashlib.sha256(
        (ROOT / "artifacts/milestone-9/cad/render-mesh.json.gz").read_bytes()
    ).hexdigest()
    p["simulation"] = copy.deepcopy(phys["simulation"])
    p["sizing"] = copy.deepcopy(shared["sizing"])
    p["sizing"]["mass_ceiling"] = cad["overall_mass_ceiling_kg"]
    p["contact"] = copy.deepcopy(shared["contact"])
    motor, friction = phys["motor"], phys["friction"]
    speed = motor["no_load_speed_motor_rad_s"] / motor["ratio"]
    p["motor"] = {
        "peak_torque_per_wheel": motor["normal_wheel_command_cap_Nm"],
        "continuous_torque_per_wheel": motor["continuous_torque_motor_Nm"]
        * motor["ratio"]
        * motor["efficiency_screen"],
        "no_load_speed": speed,
        "torque_time_constant": motor["FOC_time_constant_s"],
        "rotor_inertia_at_output": motor["rotor_core_inertia_kg_m2"] * motor["ratio"] ** 2,
        "viscous_friction": friction["wheel_viscous_Nm_s_rad"],
        "coulomb_friction": friction["wheel_coulomb_Nm"],
        "peak_basis": "RO40 motor0.6Nm/4:1/eta=.9 finite command capability; independently represented actual rotor inertia, no extra armature",
    }
    p["articulation"] = {
        "peak_torque_per_joint": motor["normal_hip_command_cap_Nm"],
        "continuous_torque_reference": motor["continuous_torque_motor_Nm"]
        * motor["ratio"]
        * motor["efficiency_screen"],
        "no_load_speed": speed,
        "torque_time_constant": motor["FOC_time_constant_s"],
        "rotor_inertia_at_output": motor["rotor_core_inertia_kg_m2"] * motor["ratio"] ** 2,
        "viscous_friction": friction["hip_viscous_Nm_s_rad"],
        "coulomb_friction": _journal_pretension_friction(p),
        "hip_limits_deg": [-220.0, -65.0],
        "knee_limits_deg": [-115.0, 40.0],
        "knee_fold_direction": -1,
    }
    p["mass_budget"] = [
        {
            "name": name,
            "location": "body" if name == "chassis" else name,
            "count": 1,
            "mass_each": row["mass_kg"],
        }
        for name, row in p["rigid_bodies"].items()
    ]
    # Compatibility scalars describe the M9 CAD envelope; native five-bar
    # dimensions above remain the geometry source used for all placements.
    g = p["geometry"]
    g.update(
        body_length=g["shell_size"][0],
        body_width=g["shell_size"][1],
        body_height=g["shell_size"][2],
        body_center_above_hip=g["shell_center"][2],
        hip_half_spacing=g["hip_motor_center_y"],
        link_length=g["rod"],
        joint_motor_radius=g["hip_motor_radius"],
        joint_motor_width=g["hip_motor_width"],
        wheel_motor_radius=g["hip_motor_radius"],
        wheel_motor_width=g["hip_motor_width"],
        fixed_leg_angle_deg=40.0,
        link_radius=p["structure"]["rod_width"] / 2,
    )
    return p


def _journal_pretension_friction(p):
    h = p["joint_drive"]
    radius = h["large_teeth"] * h["pitch"] / (2 * math.pi)
    pull = (
        2
        * h["belt_pretension_per_span_N"]
        * math.sqrt(1 - ((radius - radius / h["ratio"]) / h["center_distance"]) ** 2)
    )
    overhang = h["front_journal_center_y_m"] - h["belt_pull_y_m"]
    return (
        p["physics"]["friction"]["journal_mu"]
        * p["physics"]["friction"]["journal_radius_m"]
        * pull
        * (1 + 2 * overhang / h["bearing_span_m"])
    )


def _quat_cad(angle):
    return values([math.cos(angle / 2), 0, -math.sin(angle / 2), 0])


def _hinge(body, name, angle, axis="0 -1 0", **attrs):
    return ET.SubElement(
        body,
        "joint",
        name=name,
        type="hinge",
        axis=axis,
        ref=str(angle),
        **{k: str(v) for k, v in attrs.items()},
    )


@lru_cache(maxsize=1)
def _exterior_meshes():
    """Existing SI CAD exterior solids; convex hulls retain outer surfaces.

    This deliberately does not regenerate or modify historical CAD artifacts.
    Concave pockets and holes are filled by MuJoCo convex collision geometry.
    """
    source = ROOT / "artifacts/milestone-9/cad/render-mesh.json.gz"
    data = json.loads(gzip.decompress(source.read_bytes()))
    rows = []
    for c in data["components"]:
        name = c["name"]
        exterior = name in ["ShellRear", "ShellFaceCap", "Smile"] or any(
            token in name
            for token in [
                "EyeSocket",
                "EyeLens",
                "CrankCover",
                "RodCover",
                "_Crank",
                "_Rod",
                "WheelFork",
                "Readhead",
                "LegImpactPad",
                "RO40Housing",
                "_ROHousing",
                "_PassiveFork",
            ]
        )
        if not exterior or "Encoder" in name or "Magnet" in name:
            continue
        vertices = np.asarray(c["vertices"])
        hull = ConvexHull(vertices)
        ids = hull.vertices
        lookup = {v: i for i, v in enumerate(ids)}
        faces = np.array([[lookup[i] for i in face] for face in hull.simplices])
        rows.append((name, c["body"], vertices[ids], faces, c["color"]))
    return rows


def generate_xml(p):
    phys, g, h = p["physics"], p["geometry"], p["joint_drive"]
    q = posture(40, p)
    root = ET.Element("mujoco", model="bipe_m9_free_five_bar")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false")
    ET.SubElement(
        root,
        "option",
        timestep=str(p["simulation"]["physics_dt"]),
        gravity=values([0, 0, -p["sizing"]["gravity"]]),
        integrator="implicitfast",
        cone="elliptic",
        iterations="80",
        noslip_iterations=str(p["simulation"].get("noslip_iterations", 0)),
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    asset = ET.SubElement(root, "asset")
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        mass="0",
        friction=values(
            [
                p["contact"][k]
                for k in ["sliding_friction", "torsional_friction", "rolling_friction"]
            ]
        ),
        condim="6",
        solref=values(phys["contact"]["solref_s"]),
        solimp=values(phys["contact"]["solimp"]),
        contype="1",
        conaffinity="3",
    )
    ET.SubElement(default, "joint", armature="0")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="-1 -1 3", dir=".3 .3 -1")
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        size="0 0 .1",
        rgba=".26 .31 .37 1",
        contype="2",
        conaffinity="5",
    )
    base = ET.SubElement(
        world, "body", name="chassis", pos=values([0, 0, g["nominal_height"] + g["wheel_radius"]])
    )
    ET.SubElement(base, "freejoint", name="base")
    ET.SubElement(base, "site", name="imu", pos=values(g["shell_center"]), size=".003")
    nodes = {"chassis": base}
    equality, contact = ET.SubElement(root, "equality"), ET.SubElement(root, "contact")
    tendons = ET.SubElement(root, "tendon")
    friction = phys["friction"]
    for side, pair in zip(["left", "right"], [q[:2], q[2:]], strict=True):
        rod_angles = rod_angles_and_jacobians(pair, p)
        for i, bx in enumerate([-g["base"] / 2, g["base"] / 2]):
            crank = ET.SubElement(
                base,
                "body",
                name=f"{side}_crank{i}",
                pos=values([bx, 0, 0]),
                quat=_quat_cad(pair[i]),
            )
            output_name = f"{side}_{'hip' if i == 0 else 'knee'}"
            _hinge(
                crank,
                output_name,
                pair[i],
                damping=friction["hip_viscous_Nm_s_rad"],
                frictionloss=_journal_pretension_friction(p),
                limited="true",
                range=values(
                    np.radians(p["articulation"]["hip_limits_deg" if i == 0 else "knee_limits_deg"])
                ),
            )
            nodes[f"{side}_crank{i}"] = crank
            rod = ET.SubElement(
                crank,
                "body",
                name=f"{side}_rod{i}",
                pos=values([g["crank"], 0, 0]),
                quat=_quat_cad(rod_angles[i][0] - pair[i]),
            )
            _hinge(
                rod,
                f"{side}_rod{i}_joint",
                rod_angles[i][0] - pair[i],
                damping=friction["passive_rod_viscous_Nm_s_rad"],
                frictionloss=friction["passive_rod_coulomb_Nm"],
            )
            nodes[f"{side}_rod{i}"] = rod
            gear = ET.SubElement(
                base,
                "body",
                name=f"{side}_hipgear{i}",
                pos=values([bx, 0, 0]),
                quat=_quat_cad(pair[i]),
            )
            _hinge(gear, f"{side}_hip{i}_gear_joint", pair[i], frictionloss="1e-8")
            nodes[f"{side}_hipgear{i}"] = gear
            clutch = ET.SubElement(
                tendons,
                "fixed",
                name=f"{side}_hip{i}_clutch",
                frictionloss=str(phys["clutch"]["release_torque_Nm"]),
                solreffriction=".0005 1",
                solimpfriction=".99 .999 .0001",
            )
            ET.SubElement(clutch, "joint", joint=f"{side}_hip{i}_gear_joint", coef="1")
            ET.SubElement(clutch, "joint", joint=output_name, coef="-1")
            rotor = ET.SubElement(
                base,
                "body",
                name=f"{side}_hiprotor{i}",
                pos=values([bx, 0, h["motor_z"]]),
                quat=_quat_cad(pair[i] * h["ratio"]),
            )
            _hinge(
                rotor,
                f"{side}_hip{i}_rotor_joint",
                pair[i] * h["ratio"],
                damping=phys["motor"]["viscous_motor_Nm_s_rad"],
                frictionloss=phys["motor"]["coulomb_motor_Nm"],
            )
            nodes[f"{side}_hiprotor{i}"] = rotor
            ET.SubElement(contact, "exclude", body1=f"{side}_crank{i}", body2=f"{side}_hipgear{i}")
        theta = rod_angles[0][0]
        wheel = ET.SubElement(
            nodes[f"{side}_rod0"],
            "body",
            name=f"{side}_wheel",
            pos=values([g["rod"], 0, 0]),
            quat=_quat_cad(-theta),
        )
        _hinge(
            wheel,
            f"{side}_wheel_joint",
            0,
            axis="0 1 0",
            damping=friction["wheel_viscous_Nm_s_rad"],
            frictionloss=friction["wheel_coulomb_Nm"],
        )
        nodes[f"{side}_wheel"] = wheel
        sign = 1 if side == "left" else -1
        ET.SubElement(
            wheel,
            "geom",
            name=f"{side}_tire",
            type="cylinder",
            pos=values([0, sign * g["wheel_track"] / 2, 0]),
            quat=values([math.sqrt(0.5), math.sqrt(0.5), 0, 0]),
            size=values([g["wheel_radius"], g["wheel_width"] / 2]),
            rgba=".08 .11 .15 1",
        )
        ET.SubElement(
            wheel,
            "site",
            name=f"{side}_axle",
            pos=values([0, sign * g["wheel_track"] / 2, 0]),
            size=".001",
        )
        d = p["drive"]
        rotor = ET.SubElement(
            nodes[f"{side}_rod0"],
            "body",
            name=f"{side}_rotor",
            pos=values([g["rod"] + d["motor_along_rod_from_foot"], 0, d["motor_perpendicular"]]),
            quat=_quat_cad(-h["ratio"] * theta),
        )
        _hinge(
            rotor,
            f"{side}_rotor_joint",
            0,
            axis="0 1 0",
            damping=phys["motor"]["viscous_motor_Nm_s_rad"],
            frictionloss=phys["motor"]["coulomb_motor_Nm"],
        )
        nodes[f"{side}_rotor"] = rotor
        ET.SubElement(
            equality,
            "connect",
            name=f"{side}_foot_hinge",
            body1=f"{side}_rod0",
            body2=f"{side}_rod1",
            anchor=values([g["rod"], 0, 0]),
            solref=".001 1",
            solimp=".99 .999 .0001",
        )
        ET.SubElement(contact, "exclude", body1=f"{side}_rod0", body2=f"{side}_rod1")
        ET.SubElement(contact, "exclude", body1=f"{side}_rod1", body2=f"{side}_wheel")
    for name, node in nodes.items():
        add_inertial(node, p["rigid_bodies"][name])
    for name, body, vertices, faces, color in _exterior_meshes():
        ET.SubElement(
            asset,
            "mesh",
            name=name + "_mesh",
            vertex=values(vertices.ravel()),
            face=" ".join(str(v) for v in faces.ravel()),
        )
        extra = (
            {"contype": "4", "conaffinity": "2"}
            if name.endswith(("CrankCover", "RodCover"))
            else {}
        )
        ET.SubElement(
            nodes[body],
            "geom",
            name=name,
            type="mesh",
            mesh=name + "_mesh",
            rgba=values(color + [1]),
            **extra,
        )
    # Explicitly unadopted bumper experiments carry real frame mass/tensors
    # and finite metal contact behind the foam. Normal CAD profiles omit this.
    for member in p.get("candidate_frame_geoms", []):
        ET.SubElement(
            base,
            "geom",
            name=member["name"],
            type="box",
            mass="0",
            pos=values(member["center_m"]),
            size=values(np.array(member["size_m"]) / 2),
            quat=values(member["quat_wxyz"]),
            rgba=".45 .5 .55 1",
        )
    actuators = ET.SubElement(root, "actuator")
    peak = phys["motor"]["peak_torque_motor_Nm"]
    for name in ROTORS:
        ET.SubElement(
            actuators,
            "motor",
            name=name + "_drive",
            joint=name,
            gear="1",
            ctrllimited="true",
            ctrlrange=values([-peak, peak]),
            forcelimited="true",
            forcerange=values([-peak, peak]),
        )
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    for name in JOINTS + WHEELS:
        ET.SubElement(sensors, "jointpos", name=name + "_encoder", joint=name)
    return ET.tostring(root, encoding="unicode")


def build(p):
    model = mujoco.MjModel.from_xml_string(generate_xml(p))
    from .m9_impact import MaterialContacts

    MaterialContacts(p).configure(model)
    return model, mujoco.MjData(model)


def _refit_world_bvh(m, ground):
    """Refit the world body's bounding volume to the ground plane's current orientation.

    The compiler bounds the world body (the ground plane alone) by the flat half-space
    z <= 0. A plane tilted after compilation rises above z = 0 uphill, so the stale
    bound culls valid contacts in the midphase for any body with several geoms (the
    chassis, rods and forks). A single-geom wheel is not culled. Same repair as
    bipe.m10.plant._refit_world_bvh.
    """
    if m.body_bvhnum[0] != 1 or m.body_geomnum[0] != 1:
        raise ValueError("The world body must hold the ground plane alone to refit its bound")
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, m.geom_quat[ground])
    rot = rot.reshape(3, 3)
    center, half = m.geom_aabb[ground, :3], m.geom_aabb[ground, 3:]
    node = m.body_bvhadr[0]
    m.bvh_aabb[node, :3] = m.geom_pos[ground] + rot @ center
    m.bvh_aabb[node, 3:] = abs(rot) @ half


def reset(m, d, p, angle_deg=40, offset_deg=0, slope_deg=0):
    mujoco.mj_resetData(m, d)
    q = posture(angle_deg, p)
    qnom = posture(40, p)
    for name, value in zip(JOINTS, q, strict=True):
        d.qpos[m.joint(name).qposadr[0]] = value
    for side, pair, pn in zip(["left", "right"], [q[:2], q[2:]], [qnom[:2], qnom[2:]], strict=True):
        angles = rod_angles_and_jacobians(pair, p)
        nomtheta = rod_angles_and_jacobians(pn, p)[0][0]
        for i in range(2):
            d.qpos[m.joint(f"{side}_rod{i}_joint").qposadr[0]] = angles[i][0] - pair[i]
            d.qpos[m.joint(f"{side}_hip{i}_gear_joint").qposadr[0]] = pair[i]
            d.qpos[m.joint(f"{side}_hip{i}_rotor_joint").qposadr[0]] = (
                pair[i] * p["joint_drive"]["ratio"]
            )
        d.qpos[m.joint(f"{side}_wheel_joint").qposadr[0]] = angles[0][0] - nomtheta
        d.qpos[m.joint(f"{side}_rotor_joint").qposadr[0]] = p["joint_drive"]["ratio"] * (
            angles[0][0] - nomtheta
        )
    pitch = equilibrium(p, q) + math.radians(offset_deg)
    d.qpos[3:7] = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    foot = np.mean([leg_fk(q[:2], p), leg_fk(q[2:], p)], axis=0)
    rotated = (
        np.array([[math.cos(pitch), math.sin(pitch)], [-math.sin(pitch), math.cos(pitch)]]) @ foot
    )
    d.qpos[0] = -rotated[0]
    d.qpos[2] = p["geometry"]["wheel_radius"] - 0.0001 - rotated[1]
    slope = math.radians(slope_deg)
    ground = m.geom("ground").id
    # The compiler marked the original identity plane as body-aligned. Clear
    # that optimization before changing its transform, otherwise kinematics
    # silently keeps the physical plane horizontal despite the new quaternion.
    m.geom_sameframe[ground] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
    m.geom_quat[ground] = [math.cos(slope / 2), 0, -math.sin(slope / 2), 0]
    _refit_world_bvh(m, ground)
    mujoco.mj_forward(m, d)
    if slope_deg:
        axle = np.mean([d.site(s + "_axle").xpos for s in ["left", "right"]], axis=0)
        n = np.array([-math.sin(slope), 0, math.cos(slope)])
        d.qpos[2] += (p["geometry"]["wheel_radius"] - 0.0001 - n @ axle) / n[2]
        mujoco.mj_forward(m, d)
    # Static pretension is an initial condition, never reapplied as a pose clamp.
    preload = support_feedforward(p, q, pitch)
    k = p["physics"]["belts"]["hip_stiffness_Nm_rad"]
    half = math.radians(p["physics"]["belts"]["hip_lash_deg"]) / 2
    for i, name in enumerate(GEARS):
        gear_q = q[i]
        d.qpos[m.joint(name).qposadr[0]] = gear_q
        d.qpos[m.joint(ROTORS[i]).qposadr[0]] = p["joint_drive"]["ratio"] * (
            gear_q + np.sign(preload[i]) * half + preload[i] / k
        )
    from .m9_impact import MaterialContacts

    contacts = MaterialContacts(p)
    contacts.configure(m)
    _material_forward(m, d, contacts)


def _material_forward(m, d, contacts):
    mujoco.mj_forward(m, d)
    contacts(m, d)
    mujoco.mj_fwdConstraint(m, d)
    # mj_forward already marked cacc/cfrc as computed. Rebuilding material
    # constraints changes qacc, but sensorAcc otherwise reuses that stale
    # body-acceleration cache. Refresh the physical IMU at the new solution.
    mujoco.mj_rnePostConstraint(m, d)
    mujoco.mj_sensorAcc(m, d)


@lru_cache(maxsize=16)
def topology(m):
    qi = np.array([m.joint(n).qposadr[0] for n in JOINTS])
    vi = np.array([m.joint(n).dofadr[0] for n in JOINTS])
    wv = np.array([m.joint(n).dofadr[0] for n in WHEELS])
    return (
        qi,
        vi,
        wv,
        [m.site(s + "_axle").id for s in ["left", "right"]],
        m.body("chassis").id,
        m.sensor("gyro").adr[0],
    )


def truth(m, d, p):
    qi, vi, wv, sites, chassis, ga = topology(m)
    rot = d.xmat[chassis].reshape(3, 3)
    pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
    roll, yaw = math.atan2(rot[2, 1], rot[2, 2]), math.atan2(rot[1, 0], rot[0, 0])
    q, qd = d.qpos[qi].copy(), d.qvel[vi].copy()
    positions, velocities = [], []
    for ix in sites:
        positions.append(d.site_xpos[ix].copy())
        v = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, ix, v, 0)
        velocities.append(v[3:])
    axle, velocity = np.mean(positions, axis=0), np.mean(velocities, axis=0)
    gyro = d.sensordata[ga : ga + 3]
    pitch_rate = gyro[1] * math.cos(roll) - gyro[2] * math.sin(roll)
    yaw_rate = (gyro[1] * math.sin(roll) + gyro[2] * math.cos(roll)) / max(0.01, math.cos(pitch))
    # Wheel joint coordinates are relative to rod0, whose negative-Y angle
    # changes during crouch. Encoder-aware estimation uses this same correction.
    world_wheel_speed = np.array(
        [
            d.qvel[wv[i]] + gyro[1] - qd[2 * i] - d.qvel[m.joint(f"{side}_rod0_joint").dofadr[0]]
            for i, side in enumerate(["left", "right"])
        ]
    )
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
        "wheel_speed": d.qvel[wv].copy(),
        "world_wheel_speed": world_wheel_speed,
        "height": float(d.qpos[2] - axle[2]),
        "equilibrium": equilibrium(p, q),
    }


def geometry_check(p):
    rows = []
    place = body_placements(p, posture(40, p))
    for name, row in p["rigid_bodies"].items():
        origin, rot = place[name]
        rows.append(
            {
                "mass_kg": row["mass_kg"],
                "com_m": origin
                + rot @ row["com_m"]
                + [0, 0, p["geometry"]["nominal_height"] + p["geometry"]["wheel_radius"]],
                "inertia_kg_m2": rot @ row["inertia_kg_m2"] @ rot.T,
            }
        )
    combined = combine(rows)
    closure = 0.0
    for angle in np.linspace(30, 65, 71):
        pair = posture(float(angle), p)[:2]
        foot = leg_fk(pair, p)
        from .m9_kinematics import elbows

        closure = max(
            closure,
            float(
                np.max(abs(np.linalg.norm(foot - elbows(pair, p), axis=1) - p["geometry"]["rod"]))
            ),
        )
    com_error = float(np.linalg.norm(np.array(combined["com_m"]) - p["nominal_world"]["com_m"]))
    inertia_error = float(
        np.max(abs(np.asarray(combined["inertia_kg_m2"]) - p["nominal_world"]["inertia_kg_m2"]))
    )
    return {
        "profile": "m9",
        "rigid_body_count": len(rows),
        "mass_kg": combined["mass_kg"],
        "CAD_COM_error_m": com_error,
        "CAD_inertia_error_kg_m2": inertia_error,
        "closure_error_m": closure,
        "passed": com_error < 1e-9 and inertia_error < 1e-9 and closure < 1e-10,
    }


class MechanicalDrives:
    """Six actual-angle motors, finite belt lash and four dry-friction clutches.

    A backward-Euler belt-force predictor uses the full mass matrix. Hip
    clutches are finite fixed-tendon Coulomb constraints solved jointly with
    physical contacts/loop joints. MuJoCo integrates every robot coordinate.
    """

    def __init__(self, m, p, disturbance=None):
        self.p, self.disturbance = p, disturbance or {}
        self.rq = np.array([m.joint(n).qposadr[0] for n in ROTORS])
        self.rv = np.array([m.joint(n).dofadr[0] for n in ROTORS])
        self.gq = np.array([m.joint(n).qposadr[0] for n in GEARS])
        self.gv = np.array([m.joint(n).dofadr[0] for n in GEARS])
        self.oq = np.array([m.joint(n).qposadr[0] for n in JOINTS + WHEELS])
        self.ov = np.array([m.joint(n).dofadr[0] for n in JOINTS + WHEELS])
        self.J = np.zeros((10, m.nv))
        self.ratio = p["physics"]["motor"]["ratio"]
        self.J[np.arange(6), self.rv] = 1 / self.ratio
        self.J[np.arange(4), self.gv] = -1
        self.J[np.arange(4, 6), self.ov[4:]] = -1
        self.J[6 + np.arange(4), self.gv] = 1
        self.J[6 + np.arange(4), self.ov[:4]] = -1
        self.inverse_rows = np.zeros_like(self.J)
        b, c = p["physics"]["belts"], p["physics"]["clutch"]
        self.stiffness = np.r_[
            np.full(
                4, self.disturbance.get("hip_belt_stiffness_Nm_rad", b["hip_stiffness_Nm_rad"])
            ),
            np.full(
                2, self.disturbance.get("wheel_belt_stiffness_Nm_rad", b["wheel_stiffness_Nm_rad"])
            ),
            np.zeros(4),
        ]
        self.damping = np.r_[
            np.full(4, b["hip_damping_Nm_s_rad"]),
            np.full(2, b["wheel_damping_Nm_s_rad"]),
            np.zeros(4),
        ]
        self.half = np.r_[
            np.full(
                4, math.radians(self.disturbance.get("hip_backlash_deg", b["hip_lash_deg"])) / 2
            ),
            np.full(2, math.radians(self.disturbance.get("backlash_deg", b["wheel_lash_deg"])) / 2),
            np.zeros(4),
        ]
        self.motor_torque = np.zeros(6)
        self.phase_current = np.zeros(6)
        self.clutch_rest = np.zeros(4)
        self.transmitted = np.zeros(6)
        self.last_forces = np.zeros(10)
        self.plastic_energy = 0.0
        self.peak_belt_torque = np.zeros(6)
        self.peak_clutch_torque = np.zeros(4)
        self.peak_motor_current = np.zeros(6)
        self.step_count = 0
        self.slip_angle = np.zeros(4)
        self.clutch_release = c["release_torque_Nm"]
        self.clutch_ids = np.array(
            [m.tendon(f"{s}_hip{i}_clutch").id for s in ["left", "right"] for i in range(2)]
        )
        self._last_clutch_coordinates = None
        from .m9_impact import MaterialContacts

        self.contacts = MaterialContacts(p)
        self.contacts.configure(m)

    def forward(self, m, d, contact_callback=None):
        _material_forward(m, d, contact_callback or self.contacts)

    def motor_speed(self, d):
        return d.qvel[self.rv].copy() / self.ratio

    def _coordinates(self, d):
        return np.r_[
            d.qpos[self.rq] / self.ratio - np.r_[d.qpos[self.gq], d.qpos[self.oq[4:]]],
            d.qpos[self.gq] - d.qpos[self.oq[:4]] - self.clutch_rest,
        ]

    def apply(self, m, d, requested, dt=None, contact_callback=None):
        dt = float(m.opt.timestep if dt is None else dt)
        motor = self.p["physics"]["motor"]
        requested = np.asarray(requested, dtype=float)
        caps = np.r_[
            np.full(4, motor["normal_hip_command_cap_Nm"]),
            np.full(2, motor["normal_wheel_command_cap_Nm"]),
        ]
        reflected = np.clip(requested, -caps, caps)
        command = reflected / (self.ratio * motor["efficiency_screen"])
        speed = d.qvel[self.rv]
        maximum = np.where(
            command * speed > 0,
            motor["peak_torque_motor_Nm"]
            * np.maximum(0, 1 - abs(speed) / motor["no_load_speed_motor_rad_s"]),
            motor["peak_torque_motor_Nm"],
        )
        command = np.clip(command, -maximum, maximum)
        desired_current = np.sign(command) * np.where(
            abs(command) <= 0.2,
            abs(command) * 2.85 / 0.2,
            2.85 + (abs(command) - 0.2) * (11.6 - 2.85) / 0.4,
        )
        lag = np.r_[
            np.full(4, self.p["articulation"]["torque_time_constant"]),
            np.full(2, self.p["motor"]["torque_time_constant"]),
        ]
        change = -np.expm1(-dt / lag) * (desired_current - self.phase_current)
        slew = motor["FOC_current_slew_limit_A_s"] * dt
        self.phase_current += np.clip(change, -slew, slew)
        self.motor_torque = np.sign(self.phase_current) * np.where(
            abs(self.phase_current) <= 2.85,
            abs(self.phase_current) * 0.2 / 2.85,
            0.2 + (abs(self.phase_current) - 2.85) * 0.4 / (11.6 - 2.85),
        )
        self.motor_torque = np.clip(self.motor_torque, -maximum, maximum)
        d.ctrl[:] = self.motor_torque
        # Remove this object's previous elastic force before refreshing the
        # external acceleration predictor. User pushes remain untouched.
        d.qfrc_applied[:] += self.J.T @ self.last_forces
        self.forward(m, d, contact_callback)
        mujoco.mj_solveM(m, d, self.inverse_rows, self.J)
        mobility = self.J @ self.inverse_rows.T
        relative = self.J @ d.qvel
        acceleration = self.J @ d.qacc
        gap = self._coordinates(d)
        self._last_clutch_coordinates = d.qpos[self.gq] - d.qpos[self.oq[:4]]
        predicted_gap = gap + dt * relative + dt * dt * acceleration
        side = np.sign(predicted_gap)
        active = abs(predicted_gap) > self.half
        active[6:] = True
        k = self.stiffness * active
        c = self.damping * active
        deformation = gap - side * self.half
        rhs = k * (deformation + dt * relative + dt * dt * acceleration) + c * (
            relative + dt * acceleration
        )
        a = np.eye(10) + (k * dt * dt + c * dt)[:, None] * mobility
        fixed = np.zeros(10, dtype=bool)
        fixed_value = np.zeros(10)
        # Re-solve the coupled belt force system after a unilateral face
        # releases. Clutch sticking/slip is resolved by native MuJoCo friction.
        for _ in range(11):
            matrix, target = a.copy(), rhs.copy()
            matrix[fixed] = np.eye(10)[fixed]
            target[fixed] = fixed_value[fixed]
            force = np.linalg.solve(matrix, target)
            release = (~fixed[:6]) & ((~active[:6]) | (force[:6] * side[:6] <= 0))
            if not release.any():
                break
            ids = np.flatnonzero(release)
            fixed[ids] = True
        d.qfrc_applied[:] -= self.J.T @ force
        self.last_forces = force
        self.transmitted[4:] = force[4:6]
        self.peak_belt_torque = np.maximum(self.peak_belt_torque, abs(force[:6]))
        self.peak_motor_current = np.maximum(self.peak_motor_current, abs(self.phase_current))
        self.step_count += 1
        # Physical journal friction is finite Coulomb friction resolved by
        # MuJoCo constraints. Radial reaction is taken from the multibody solve.
        mujoco.mj_rnePostConstraint(m, d)
        friction = self.p["physics"]["friction"]
        h = self.p["joint_drive"]
        multiplier = 1 + 2 * h["shaft_overhang_m"] / h["bearing_span_m"]
        for i, body in enumerate([f"{s}_crank{j}" for s in ["left", "right"] for j in range(2)]):
            load = d.cfrc_int[m.body(body).id, 3:]
            axis = d.xaxis[m.joint(JOINTS[i]).id]
            radial = math.sqrt(max(0, float(load @ load - (load @ axis) ** 2)))
            m.dof_frictionloss[self.ov[i]] = (
                _journal_pretension_friction(self.p)
                + friction["journal_mu"] * friction["journal_radius_m"] * multiplier * radial
            )
        # Coulomb belt loss is a dissipative coupling loss in both power
        # directions. It also supplies the .90 nominal efficiency at load.
        loss = (1 - motor["efficiency_screen"]) * abs(force[:6])
        m.dof_frictionloss[self.gv] = np.maximum(loss[:4], 1e-8)
        m.dof_frictionloss[self.ov[4:]] = friction["wheel_coulomb_Nm"] + loss[4:]
        applied = self.motor_torque * self.ratio * motor["efficiency_screen"]
        saturated = (
            abs(requested)
            > np.minimum(caps, maximum * self.ratio * motor["efficiency_screen"]) + 1e-8
        )
        return applied.copy(), saturated

    def observe_after_step(self, m, d):
        """Actual finite clutch force and signed work after MuJoCo integration."""
        forces = np.zeros(4)
        for i in np.flatnonzero(
            d.efc_type[: d.nefc] == mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON
        ):
            ids = np.flatnonzero(self.clutch_ids == d.efc_id[i])
            if len(ids):
                forces[ids[0]] = d.efc_force[i]
        current = d.qpos[self.gq] - d.qpos[self.oq[:4]]
        if self._last_clutch_coordinates is not None:
            displacement = current - self._last_clutch_coordinates
            self.slip_angle += abs(displacement)
            self.plastic_energy += float(np.maximum(-forces * displacement, 0).sum())
        self._last_clutch_coordinates = current.copy()
        self.transmitted[:4] = -forces
        self.peak_clutch_torque = np.maximum(self.peak_clutch_torque, abs(forces))

    def integrate(self, m, d, contact_callback=None):
        mujoco.mj_step1(m, d)
        (contact_callback or self.contacts)(m, d)
        mujoco.mj_step2(m, d)
        self.observe_after_step(m, d)

    def step(self, m, d, requested, dt=None, contact_callback=None):
        result = self.apply(m, d, requested, dt, contact_callback)
        self.integrate(m, d, contact_callback)
        return result

    def metrics(self):
        return {
            "physics_substeps": self.step_count,
            "belt_peak_transmitted_Nm": self.peak_belt_torque.tolist(),
            "clutch_peak_transmitted_Nm": self.peak_clutch_torque.tolist(),
            "phase_current_peak_A": self.peak_motor_current.tolist(),
            "clutch_slip_angle_rad": self.slip_angle.tolist(),
            "clutch_plastic_dissipated_J": self.plastic_energy,
            "clutch_dissipated_J": self.plastic_energy,
            "last_output_transmitted_Nm": self.transmitted.tolist(),
            "state_projection_count": 0,
            "rotor_inertia_basis": "Six actual-angle CAD rotor tensors already include manufacturer coreIyy; zero extra output armature",
        }
