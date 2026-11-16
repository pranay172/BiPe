"""M13 training plant: the M10 robot in primitive geometry, with no Python physics callbacks.

Same kinematic tree, frames, masses and inertias as ``bipe.m10.plant`` (both are
generated from config/robot.json). Differences, all stated in
config/training-plant-m13.json:
- hip belts (``drives.hip_belt``): "rigid" folds the four RO40 rotors into the
  chassis inertia and into armature on the leg hinges (the gyroscopic cross term
  between a rotor and the body is dropped); "spring" (M14) keeps the rotor hinges
  and couples each to its leg hinge with a 4:1 fixed tendon spring whose dead band
  is the belt lash, as the full plant's belt rows;
- the 1:1 knee belts are fixed tendon springs between the knee-drive hinge and the
  actual shin angle, with the M10 stiffness and damping; ``drives.knee_lash``
  "static" offsets the rest length by half the lash on the loaded side, "deadband"
  (M14) gives the spring a dead band of the full lash;
- tendon damping acts inside the lash too (the full plant's belt damping acts only
  while a face is engaged);
- the exterior is boxes and cylinders enclosing the B3 hulls, colliding with the
  ground only; the tires are MuJoCo soft contacts with fitted direct stiffness;
- friction is constant per hinge (no load-dependent bearing or belt-loss term).

The MJCF runs unchanged in CPU MuJoCo and MuJoCo Warp. ``Drives`` gives it the
``MechanicalDrives`` interface, so ``bipe.moving_experiment.simulate`` runs the
classical controller on it.
"""

import copy
import json
import math
import xml.etree.ElementTree as ET
from functools import lru_cache

import mujoco
import numpy as np

from ..m10 import plant as full
from ..m10.kinematics import SIDES, equilibrium, leg_fk, posture, support_feedforward
from ..mass_properties import add_inertial, combine
from ..model import values
from ..parameters import ROOT
from . import drives

CONFIG = ROOT / "config/training-plant-m13.json"
JOINTS = full.JOINTS
WHEELS = full.WHEELS
# Shell groups that are boxes unless the config says otherwise.
SHAPES = ("box", "cylinder")


def parameters(shared, training=None):
    """The M10 plant profile with the training-plant section (config/training-plant-m13.json)."""
    p = full.parameters(shared)
    p["profile"], p["name"] = "m13", "bipe_m13_training"
    p["training"] = copy.deepcopy(
        training if training is not None else json.loads(CONFIG.read_text())
    )
    p["simulation"]["physics_dt"] = p["training"]["simulation"]["physics_dt"]
    return p


def _obb(vertices):
    """Smaller-volume enclosing box of the link-frame and principal-axis boxes: (centre, R, half)."""
    best = None
    pca = np.linalg.svd(vertices - vertices.mean(0), full_matrices=False)[2].T
    if np.linalg.det(pca) < 0:
        pca[:, 2] *= -1
    for rot in (np.eye(3), pca):
        local = vertices @ rot
        low, high = local.min(0), local.max(0)
        volume = float(np.prod(high - low))
        if best is None or volume < best[3] - 1e-12:
            best = (rot @ ((low + high) / 2), rot, (high - low) / 2, volume)
    return best[:3]


def _quat(rot):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(rot).ravel())
    return q


@lru_cache(maxsize=4)
def _shell_cached(groups_json):
    groups = json.loads(groups_json)
    hulls = {name: (body, v) for name, body, v, _, _ in full._exterior_meshes()}
    rows = []
    for name, group in groups.items():
        parts = [hulls[part] for part in group["parts"]]
        if {body for body, _ in parts} != {group["body"]}:
            raise ValueError(f"shell group {name} spans several links")
        vertices = np.vstack([v for _, v in parts])
        hull_volume = sum(float(_hull_volume(v)) for _, v in parts)
        shape = group.get("shape", "box")
        if shape not in SHAPES:
            raise ValueError(f"unknown shell shape {shape}")
        if shape == "cylinder":
            # Axis along link y, centred on the hull's x-z extent.
            low, high = vertices.min(0), vertices.max(0)
            centre = (low + high) / 2
            radius = float(np.max(np.hypot(*(vertices[:, [0, 2]] - centre[[0, 2]]).T)))
            half = (high[1] - low[1]) / 2
            geom = {
                "type": "cylinder",
                "pos": centre,
                "quat": np.array([math.sqrt(0.5), math.sqrt(0.5), 0, 0]),
                "size": [radius, half],
            }
            volume = math.pi * radius**2 * 2 * half
        else:
            centre, rot, half = _obb(vertices)
            geom = {"type": "box", "pos": centre, "quat": _quat(rot), "size": half}
            volume = float(np.prod(2 * half))
        rows.append(
            {
                "name": name,
                "body": group["body"],
                "geom": geom,
                "hull_volume_m3": hull_volume,
                "primitive_volume_m3": volume,
            }
        )
    return rows


def _hull_volume(vertices):
    from scipy.spatial import ConvexHull

    return ConvexHull(vertices).volume


def shell(p):
    """Primitive exterior geoms with the hull and primitive volumes of each group."""
    return _shell_cached(json.dumps(p["training"]["shell"]["groups"], sort_keys=True))


def _pads(p):
    """Bumper pads: the shaped faces' convex hulls reduced to at most pad_hull_vertices
    vertices, or (when that is null) the skid as a box and the rings as cylinders at their
    unshaped sizes. Returns (body, geom, mesh vertices or None)."""
    limit = p["training"]["shell"].get("pad_hull_vertices")
    rows = []
    for pad in full._bumper_pads(p):
        geom = dict(pad["geom"])
        if geom.get("type") == "mesh" and limit:
            rows.append((pad["body"], geom, reduced_hull(pad["vertices"], limit)))
            continue
        if geom.get("type") == "mesh":
            geom.pop("mesh")
            if pad["shape"] == "box":
                geom.update(type="box", size=values(np.asarray(pad["size_m"]) / 2))
            else:
                geom.update(
                    type="cylinder", size=values([pad["outer_radius_m"], pad["thickness_m"] / 2])
                )
        rows.append((pad["body"], geom, None))
    return rows


def reduced_hull(vertices, count):
    """At most count hull vertices by farthest-point sampling (an inner approximation)."""
    from scipy.spatial import ConvexHull

    points = np.asarray(vertices)[ConvexHull(vertices).vertices]
    if len(points) <= count:
        return points
    centre = points.mean(0)
    chosen = [int(np.argmax(np.linalg.norm(points - centre, axis=1)))]
    distance = np.linalg.norm(points - points[chosen[0]], axis=1)
    while len(chosen) < count:
        chosen.append(int(np.argmax(distance)))
        distance = np.minimum(distance, np.linalg.norm(points - points[chosen[-1]], axis=1))
    return points[chosen]


def hip_spring(p):
    return p["training"]["drives"].get("hip_belt") == "spring"


def knee_deadband(p):
    return p["training"]["drives"].get("knee_lash") == "deadband"


def knee_lash_offset(p):
    """The knee belt's rest length: half its lash on the side the standing load takes up.

    The knee drives carry the body's weight in one direction at every posture, so the
    full plant's belt sits against one face of its lash; the lash-free spring is offset
    by that half lash. A reversal of the knee torque (rare outside falls) is not modeled.
    """
    if p["training"]["drives"].get("knee_lash") != "static":
        return 0.0
    q = posture(40, p)
    preload = support_feedforward(p, q, equilibrium(p, q))[1]
    return math.copysign(math.radians(p["physics"]["belts"]["knee_lash_deg"]) / 2, preload)


def rotor_armature_floor(p, hip_k):
    """Extra thigh-rotor armature that keeps the rotor-on-belt mode resolvable at dt.

    The belt spring is explicit, so its mode needs omega*dt well below 2. The hip belts
    put the RO40 rotor mode at 200-300 Hz; the rotor gets just enough armature for
    omega*dt = drives.rotor_omega_dt (default 1.0) when it falls short (the mode sits
    far above the controller's bandwidth either way). 1.6 held the classical
    controller, but violent RL actions at 1.35-1.6 spun the rotors up to 5000 rad/s
    (31 October); at 1.0 and 1.2 none did. At 1.0 the nominal belt's rotor carries
    about 0.8x its own spin inertia as extra armature.
    """
    ratio, dt = p["joint_drive"]["ratio"], p["simulation"]["physics_dt"]
    wdt = p["training"]["drives"].get("rotor_omega_dt", 1.0)
    return max(0.0, hip_k / ratio**2 * (dt / wdt) ** 2 - rotor_spin(p))


def rotor_spin(p):
    """The thigh rotor's own spin inertia, kg m^2 (motor side)."""
    return float(np.asarray(p["rigid_bodies"]["left_thighrotor"]["inertia_kg_m2"])[1][1])


def hip_half(p):
    return math.radians(p["physics"]["belts"]["hip_lash_deg"]) / 2


def knee_chain(p, hip_k, knee_k):
    """Stiffness of the knee chain's spring: with spring hip belts the knee rotor is
    folded onto the knee-drive hinge, so its 4:1 hip belt and the 1:1 knee belt act as
    one spring in series (output-side stiffness, both referred to the knee drive)."""
    if not hip_spring(p):
        return knee_k, 1.0
    series = 1 / (1 / hip_k + 1 / knee_k)
    return series, series / knee_k


def knee_dead(p, hip_lash_deg=None, knee_lash_deg=None):
    """Half the knee chain's dead band: the knee belt's lash, plus the knee rotor's hip
    belt lash when that belt is folded into the chain."""
    b = p["physics"]["belts"]
    half = math.radians(b["knee_lash_deg"] if knee_lash_deg is None else knee_lash_deg) / 2
    if hip_spring(p):
        half += math.radians(b["hip_lash_deg"] if hip_lash_deg is None else hip_lash_deg) / 2
    return half


def rotor_folding(p):
    """Chassis rigid body with the rotors' mass, and each leg hinge's rotor armature.

    With spring hip belts only the knee rotors are folded (onto the knee-drive hinge,
    whose own pulley inertia is too small to sit between two belt springs at a 1 ms
    step); the thigh rotors are bodies of their own.
    """
    rb, g = p["rigid_bodies"], p["geometry"]
    ratio = p["joint_drive"]["ratio"]
    rotor_pos = np.array([g["motor_row_x"], 0.0, g["motor_row_z"]])
    rows, armature = [rb["chassis"]], {}
    roles = (("knee", "knee"),) if hip_spring(p) else (("thigh", "hip"), ("knee", "knee"))
    for side in SIDES:
        if hip_spring(p):
            armature[f"{side}_hip"] = 0.0
        for role, short in roles:
            joint = f"{side}_{short}"
            row = rb[f"{side}_{role}rotor"]
            rows.append(
                {
                    "mass_kg": row["mass_kg"],
                    "com_m": (rotor_pos + row["com_m"]).tolist(),
                    "inertia_kg_m2": row["inertia_kg_m2"],
                }
            )
            # Hinge axes are -y in the body frame; the rotor's spin inertia is about y.
            armature[joint] = ratio**2 * float(np.asarray(row["inertia_kg_m2"])[1][1])
    return combine(rows), armature


def friction(p):
    """Constant frictionloss and damping per hinge kind (config training.friction or defaults)."""
    phys, fr = p["physics"], p["physics"]["friction"]
    hm, ratio = phys["hip_motor"], p["joint_drive"]["ratio"]
    cfg = p["training"]["friction"]
    base_leg = full.hip_pretension_friction(p) + ratio * hm["coulomb_motor_Nm"]
    leg_damping = fr["hip_viscous_Nm_s_rad"] + ratio**2 * hm["viscous_motor_Nm_s_rad"]

    def pick(key, default):
        return default if cfg.get(key) is None else cfg[key]

    hip = (leg_damping, pick("hip_frictionloss_Nm", base_leg))
    if hip_spring(p):
        # The thigh rotor's Coulomb and viscous terms move to its own hinge.
        loss = hip[1] - ratio * hm["coulomb_motor_Nm"]
        hip = (fr["hip_viscous_Nm_s_rad"], max(full.hip_pretension_friction(p), loss))
    return {
        "hip": hip,
        "knee": (leg_damping, pick("knee_drive_frictionloss_Nm", base_leg)),
        "shin": (
            fr["knee_viscous_Nm_s_rad"],
            pick("shin_frictionloss_Nm", fr["knee_bearing_coulomb_Nm"]),
        ),
        "wheel": (
            fr["wheel_viscous_Nm_s_rad"],
            pick("wheel_frictionloss_Nm", fr["wheel_coulomb_Nm"]),
        ),
    }


def generate_xml(p):
    phys, g, t = p["physics"], p["geometry"], p["training"]
    sim, contact = t["simulation"], t["contact"]
    root = ET.Element("mujoco", model="bipe_m13_training")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false")
    ET.SubElement(
        root,
        "option",
        timestep=str(p["simulation"]["physics_dt"]),
        gravity=values([0, 0, -p["sizing"]["gravity"]]),
        integrator=sim["integrator"],
        cone=sim["cone"],
        iterations=str(sim["iterations"]),
        impratio=str(sim["impratio"]),
        noslip_iterations=str(sim.get("noslip_iterations", 0)),
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    default = ET.SubElement(root, "default")
    # Robot geoms collide with the ground only (contype 1 against the ground's affinity).
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
        condim="3",
        solref=values(contact["shell_solref"]),
        solimp=values(contact["shell_solimp"]),
        contype="1",
        conaffinity="0",
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
        contype="0",
        conaffinity="1",
    )
    base = ET.SubElement(
        world, "body", name="chassis", pos=values([0, 0, g["nominal_height"] + g["wheel_radius"]])
    )
    ET.SubElement(base, "freejoint", name="base")
    ET.SubElement(base, "site", name="imu", pos=values(phys["imu_position_m"]), size=".003")
    chassis, armature = rotor_folding(p)
    add_inertial(base, chassis)
    nodes = {"chassis": base}
    lim = phys["end_stops_deg"]
    passive = friction(p)
    tendons = ET.SubElement(root, "tendon")
    belts = phys["belts"]
    for side, sign in zip(SIDES, [1, -1], strict=True):
        thigh = ET.SubElement(base, "body", name=f"{side}_thigh")
        damping, loss = passive["hip"]
        full._hinge(
            thigh,
            f"{side}_hip",
            damping=damping,
            frictionloss=loss,
            armature=armature[f"{side}_hip"],
            range=values(np.radians(lim["thigh_a1"])),
        )
        drive = ET.SubElement(base, "body", name=f"{side}_kneedrive")
        damping, loss = passive["knee"]
        full._hinge(
            drive,
            f"{side}_knee",
            damping=damping,
            frictionloss=loss,
            armature=armature[f"{side}_knee"],
            range=values(np.radians(lim["knee_drive_a2"])),
        )
        shin = ET.SubElement(
            thigh, "body", name=f"{side}_shin", pos=values([g["thigh_length"], 0, 0])
        )
        damping, loss = passive["shin"]
        full._hinge(
            shin,
            f"{side}_shin_joint",
            damping=damping,
            frictionloss=loss,
            range=values(np.radians(lim["knee_angle"])),
        )
        wheel = ET.SubElement(
            shin, "body", name=f"{side}_wheel", pos=values([g["shin_length"], 0, 0])
        )
        damping, loss = passive["wheel"]
        full._hinge(wheel, f"{side}_wheel_joint", axis="0 1 0", damping=damping, frictionloss=loss)
        ET.SubElement(
            wheel,
            "geom",
            name=f"{side}_tire",
            type="cylinder",
            pos=values([0, sign * g["wheel_track"] / 2, 0]),
            quat=values([math.sqrt(0.5), math.sqrt(0.5), 0, 0]),
            size=values([g["wheel_radius"], g["wheel_width"] / 2]),
            rgba=".42 .42 .42 1",
            contype="0",
            conaffinity="0",
        )
        ET.SubElement(
            wheel,
            "site",
            name=f"{side}_axle",
            pos=values([0, sign * g["wheel_track"] / 2, 0]),
            size=".001",
        )
        nodes.update(
            {
                f"{side}_thigh": thigh,
                f"{side}_kneedrive": drive,
                f"{side}_shin": shin,
                f"{side}_wheel": wheel,
            }
        )
        # Knee belt: gap = drive - (thigh + shin), positive when the shin lags its driver.
        if t["drives"]["knee_belt"] == "spring":
            belt = ET.SubElement(
                tendons,
                "fixed",
                name=f"{side}_knee_belt",
                stiffness=str(
                    knee_chain(p, belts["hip_stiffness_Nm_rad"], belts["knee_stiffness_Nm_rad"])[0]
                ),
                damping=str(belts["knee_damping_Nm_s_rad"]),
                springlength=(
                    values([-knee_dead(p), knee_dead(p)])
                    if knee_deadband(p)
                    else str(knee_lash_offset(p))
                ),
            )
            for joint, coef in (
                (f"{side}_knee", 1),
                (f"{side}_hip", -1),
                (f"{side}_shin_joint", -1),
            ):
                ET.SubElement(belt, "joint", joint=joint, coef=str(coef))
        else:
            raise ValueError("only the knee belt spring is implemented")
        if hip_spring(p):
            ratio = p["joint_drive"]["ratio"]
            hm = phys["hip_motor"]
            rotor_pos = values([g["motor_row_x"], 0, g["motor_row_z"]])
            for role, joint in (("thigh", f"{side}_hip"),):
                rotor = ET.SubElement(base, "body", name=f"{side}_{role}rotor", pos=rotor_pos)
                full._hinge(
                    rotor,
                    f"{side}_{role}_rotor_joint",
                    damping=hm["viscous_motor_Nm_s_rad"],
                    frictionloss=hm["coulomb_motor_Nm"],
                )
                nodes[f"{side}_{role}rotor"] = rotor
                # Belt gap = rotor / ratio - output, as the full plant's belt rows 0..3.
                belt = ET.SubElement(
                    tendons,
                    "fixed",
                    name=f"{side}_{role}_hip_belt",
                    stiffness=str(belts["hip_stiffness_Nm_rad"]),
                    damping=str(belts["hip_damping_Nm_s_rad"]),
                    springlength=values([-hip_half(p), hip_half(p)]),
                )
                ET.SubElement(
                    belt, "joint", joint=f"{side}_{role}_rotor_joint", coef=str(1 / ratio)
                )
                ET.SubElement(belt, "joint", joint=joint, coef="-1")
                # The rotor encoder's output angle includes the belt deflection.
                encoder = ET.SubElement(tendons, "fixed", name=f"{side}_{role}_rotor_output")
                ET.SubElement(
                    encoder, "joint", joint=f"{side}_{role}_rotor_joint", coef=str(1 / ratio)
                )
    for name, node in nodes.items():
        if name != "chassis":
            add_inertial(node, p["rigid_bodies"][name])
    asset = ET.SubElement(root, "asset")
    for body, geom, vertices in _pads(p):
        if vertices is not None:
            ET.SubElement(
                asset,
                "mesh",
                name=geom["mesh"],
                vertex=values(np.asarray(vertices).ravel()),
            )
        ET.SubElement(nodes[body], "geom", **geom)
    for row in shell(p):
        geom = row["geom"]
        ET.SubElement(
            nodes[row["body"]],
            "geom",
            name=row["name"],
            type=geom["type"],
            pos=values(geom["pos"]),
            quat=values(geom["quat"]),
            size=values(geom["size"]),
            rgba=".85 .85 .82 .6",
        )
    pairs = ET.SubElement(root, "contact")
    friction_row = [
        p["contact"][k] for k in ["sliding_friction", "torsional_friction", "rolling_friction"]
    ]
    for side in SIDES:
        ET.SubElement(
            pairs,
            "pair",
            geom1="ground",
            geom2=f"{side}_tire",
            condim=str(contact["tire_condim"]),
            friction=values(
                [
                    friction_row[0],
                    friction_row[0],
                    friction_row[1],
                    friction_row[2],
                    friction_row[2],
                ]
            ),
            solref=values(contact["tire_solref"]),
            solimp=values(contact["tire_solimp"]),
            solreffriction=values(contact.get("tire_solreffriction", [0, 0])),
        )
    actuators = ET.SubElement(root, "actuator")
    hm, gl = phys["hip_motor"], phys["hub_motor"]
    leg_peak = hm["peak_torque_motor_Nm"] * p["joint_drive"]["ratio"] * hm["efficiency_screen"]
    # Controls are output-side torques. With spring hip belts the hip motors drive the
    # rotor hinges through gear 1 / ratio, so the rotor sees the motor torque.
    rotors = {f"{s}_hip": f"{s}_thigh_rotor_joint" for s in SIDES} if hip_spring(p) else {}
    for name, peak in [(n, leg_peak) for n in JOINTS] + [(n, gl["peak_torque_Nm"]) for n in WHEELS]:
        ET.SubElement(
            actuators,
            "motor",
            name=name + "_drive",
            joint=rotors.get(name, name),
            gear=str(1 / p["joint_drive"]["ratio"]) if name in rotors else "1",
            ctrllimited="true",
            ctrlrange=values([-peak, peak]),
        )
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="accelerometer", site="imu")
    # Rigid hip belts: the rotor encoder's output angle is the hinge angle.
    for name in JOINTS + WHEELS:
        if name in rotors:
            side = name.split("_")[0]
            ET.SubElement(
                sensors, "tendonpos", name=name + "_encoder", tendon=f"{side}_thigh_rotor_output"
            )
        else:
            ET.SubElement(sensors, "jointpos", name=name + "_encoder", joint=name)
    return ET.tostring(root, encoding="unicode")


def build(p):
    model = mujoco.MjModel.from_xml_string(generate_xml(p))
    return model, mujoco.MjData(model)


def _qadr(m, name):
    return m.joint(name).qposadr[0]


def reset(m, d, p, angle_deg=40, offset_deg=0, slope_deg=0):
    """Rest pose at posture angle_deg on the (tilted) ground, as bipe.m10.plant.reset."""
    mujoco.mj_resetData(m, d)
    q = posture(angle_deg, p)
    pitch = equilibrium(p, q) + math.radians(offset_deg)
    preload = support_feedforward(p, q, pitch)
    b = p["physics"]["belts"]
    knee_k, hip_k = b["knee_stiffness_Nm_rad"], b["hip_stiffness_Nm_rad"]
    ratio = p["joint_drive"]["ratio"]
    for i, side in enumerate(SIDES):
        a1, a2 = q[2 * i : 2 * i + 2]
        t1, t2 = preload[2 * i : 2 * i + 2]
        lash = math.copysign(knee_dead(p), t2) if knee_deadband(p) else knee_lash_offset(p)
        drive = a2 + lash + t2 / knee_chain(p, hip_k, knee_k)[0]
        d.qpos[_qadr(m, f"{side}_hip")] = a1
        d.qpos[_qadr(m, f"{side}_knee")] = drive
        d.qpos[_qadr(m, f"{side}_shin_joint")] = a2 - a1
        if hip_spring(p):
            # Static belt preload as an initial condition, as bipe.m10.plant.reset.
            d.qpos[_qadr(m, f"{side}_thigh_rotor_joint")] = ratio * (
                a1 + math.copysign(hip_half(p), t1) + t1 / hip_k
            )

    d.qpos[3:7] = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    axle = np.mean([leg_fk(q[:2], p), leg_fk(q[2:], p)], axis=0)
    rotated = (
        np.array([[math.cos(pitch), math.sin(pitch)], [-math.sin(pitch), math.cos(pitch)]]) @ axle
    )
    d.qpos[0] = -rotated[0]
    d.qpos[2] = p["geometry"]["wheel_radius"] - 0.0001 - rotated[1]
    slope = math.radians(slope_deg)
    ground = m.geom("ground").id
    m.geom_sameframe[ground] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
    m.geom_quat[ground] = [math.cos(slope / 2), 0, -math.sin(slope / 2), 0]
    full._refit_world_bvh(m, ground)
    mujoco.mj_forward(m, d)
    if slope_deg:
        axle3 = np.mean([d.site(s + "_axle").xpos for s in SIDES], axis=0)
        n = np.array([-math.sin(slope), 0, math.cos(slope)])
        d.qpos[2] += (p["geometry"]["wheel_radius"] - 0.0001 - n @ axle3) / n[2]
    mujoco.mj_forward(m, d)


truth = full.truth
geometry_check = full.geometry_check


class Drives:
    """The ``MechanicalDrives`` interface on the training plant.

    The motors are the M10 equations (bipe.m13.drives); the belts are in the MJCF.
    As in the full plant, a rigid hip belt transmits ratio x the motor torque, and the
    belt loss (1 - efficiency) x |belt torque| is Coulomb friction on each belt's
    driven member (the leg hinge, or the shin for a knee belt), set every step.
    ``motor_torque`` is the hip motors' torque referred through the belt ratio
    (before the belt efficiency) and the hubs' torque, so that ``motor_torque *
    qvel[rv]`` is motor power, as in the full plant.
    """

    def __init__(self, m, p, disturbance=None):
        self.p, self.disturbance, self.model = p, disturbance or {}, m
        supported = set()
        if hip_spring(p):
            supported |= {"hip_backlash_deg", "hip_belt_stiffness_Nm_rad"}
        if knee_deadband(p):
            supported |= {"knee_backlash_deg", "backlash_deg", "knee_belt_stiffness_Nm_rad"}
        unsupported = {
            k
            for k, v in self.disturbance.items()
            if ("belt" in k or "backlash" in k) and v and k not in supported
        }
        if unsupported:
            raise ValueError(f"training plant has no such belt variants: {sorted(unsupported)}")
        self.ratio = p["joint_drive"]["ratio"]
        self.ov = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        self.wv = np.array([m.joint(n).dofadr[0] for n in WHEELS])
        self.spring = hip_spring(p)
        # Motor speed: the thigh rotor hinges when they exist (motor side, scale 1), else
        # the leg hinges (output side, scale ratio). Knee rotors are always folded.
        self.rotor_v, self.rotor_scale = self.ov.copy(), np.full(4, float(self.ratio))
        if self.spring:
            self.rotor_v[0::2] = [m.joint(f"{s}_thigh_rotor_joint").dofadr[0] for s in SIDES]
            self.rotor_scale[0::2] = 1.0
        self.rv = np.r_[self.rotor_v, self.wv]
        self._belt_variants(m, p)
        self.sv = np.array([m.joint(n).dofadr[0] for n in full.SHINS])
        self.belts = np.array([m.tendon(f"{s}_knee_belt").id for s in SIDES])
        self.base_friction = m.dof_frictionloss.copy()
        self.loss = 1 - p["physics"]["hip_motor"]["efficiency_screen"]
        self.motor_torque = np.zeros(6)
        self.phase_current = np.zeros(6)
        self.peak_motor_current = np.zeros(6)
        self.peak_bus_limited = np.zeros(2)
        self.step_count = 0

    def _belt_variants(self, m, p):
        """Lash and stiffness variants into the model's belt tendons (as the full plant)."""
        b, dist = p["physics"]["belts"], self.disturbance
        hip_k = dist.get("hip_belt_stiffness_Nm_rad", b["hip_stiffness_Nm_rad"])
        knee_k = dist.get("knee_belt_stiffness_Nm_rad", b["knee_stiffness_Nm_rad"])
        hip_lash = dist.get("hip_backlash_deg", b["hip_lash_deg"])
        knee_lash = dist.get("knee_backlash_deg", dist.get("backlash_deg", b["knee_lash_deg"]))
        groups = []
        if self.spring:
            groups.append(
                (
                    [f"{s}_thigh_hip_belt" for s in SIDES],
                    hip_k,
                    b["hip_damping_Nm_s_rad"] / b["hip_stiffness_Nm_rad"],
                    math.radians(hip_lash) / 2,
                )
            )
        if knee_deadband(p):
            groups.append(
                (
                    [f"{s}_knee_belt" for s in SIDES],
                    knee_chain(p, hip_k, knee_k)[0],
                    b["knee_damping_Nm_s_rad"] / b["knee_stiffness_Nm_rad"],
                    knee_dead(p, hip_lash, knee_lash),
                )
            )
        if self.spring:
            floor = rotor_armature_floor(p, hip_k)
            for s in SIDES:
                m.dof_armature[m.joint(f"{s}_thigh_rotor_joint").dofadr[0]] = floor
        for names, k, tau, half in groups:
            for name in names:
                t = m.tendon(name).id
                m.tendon_stiffness[t] = k
                # Damping keeps each belt's nominal time constant, as in the full plant.
                m.tendon_damping[t] = k * tau
                m.tendon_lengthspring[t] = [-half, half]

    def forward(self, m, d, contact_callback=None):
        mujoco.mj_forward(m, d)

    def motor_speed(self, d):
        rotor = d.qvel[self.rotor_v] * self.rotor_scale / self.ratio
        return np.r_[rotor, d.qvel[self.wv]]

    def apply(self, m, d, requested, dt=None, contact_callback=None):
        dt = float(m.opt.timestep if dt is None else dt)
        requested = np.asarray(requested, dtype=float)
        phys = self.p["physics"]
        hip_t, hip_out, hip_sat, self.phase_current[:4] = drives.hip_motor(
            np,
            phys["hip_motor"],
            self.ratio,
            requested[:4],
            d.qvel[self.rotor_v] * self.rotor_scale,
            self.phase_current[:4],
            dt,
        )
        hub_t, hub_sat, self.phase_current[4:], limited = drives.hub_motor(
            np, phys["hub_motor"], requested[4:], d.qvel[self.wv], self.phase_current[4:], dt
        )
        belt = hip_t * self.ratio
        # motor_torque x qvel[rv] is motor power: motor-side torque where rv is a rotor.
        self.motor_torque = np.r_[np.where(self.rotor_scale == 1.0, hip_t, belt), hub_t]
        d.ctrl[:] = np.r_[belt, hub_t]
        length = d.ten_length[self.belts]
        if knee_deadband(self.p):
            # Tension only past the dead band (the static M13 belt keeps its old form).
            low, high = m.tendon_lengthspring[self.belts].T
            length = length - np.clip(length, low, high)
        knee = m.tendon_stiffness[self.belts] * length + (
            m.tendon_damping[self.belts] * d.ten_velocity[self.belts]
        )
        m.dof_frictionloss[self.ov] = self.base_friction[self.ov] + self.loss * abs(belt)
        m.dof_frictionloss[self.sv] = self.base_friction[self.sv] + self.loss * abs(knee)
        self.peak_motor_current = np.maximum(self.peak_motor_current, abs(self.phase_current))
        self.peak_bus_limited = np.maximum(self.peak_bus_limited, limited.astype(float))
        self.step_count += 1
        return np.r_[hip_out, hub_t], np.r_[hip_sat, hub_sat]

    def integrate(self, m, d, contact_callback=None):
        mujoco.mj_step(m, d)

    def step(self, m, d, requested, dt=None, contact_callback=None):
        result = self.apply(m, d, requested, dt)
        self.integrate(m, d)
        return result

    def metrics(self):
        return {
            "physics_substeps": self.step_count,
            "phase_current_peak_A": self.peak_motor_current.tolist(),
            "hub_voltage_limited": self.peak_bus_limited.tolist(),
            "state_projection_count": 0,
            "rotor_inertia_basis": (
                "M14 training plant: RO40 rotor hinges on spring hip belts"
                if self.spring
                else "M13 training plant: RO40 rotors as hinge armature (rigid hip belts)"
            ),
        }


MechanicalDrives = Drives
