"""Free-base M10 serial-leg plant with finite elastic belts and GL40 hub motors.

Every inertial body uses the B3 CAD frames and tensors (config m10.cad) plus
the mass allowances. Per side: thigh and knee-drive hinges on the hip axis,
the shin hinge at the knee on the thigh, the wheel hinge at the axle on the
shin, and two RO40 rotor hinges in the rear motor row. Four 4:1 hip belts and
two 1:1 knee belts are finite stiffness, damping and unilateral lash elements
solved by ``MechanicalDrives``; the GL40s drive the wheel hinges directly,
reacting on the shins. No pose or velocity is prescribed after reset. Contact
laws and end stops are stated digital assumptions (config m10.physics).
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

from ..m9_impact import MaterialContacts
from ..mass_properties import add_inertial, combine
from ..model import values
from ..parameters import ROOT
from .bumpers import FacePads, ear_cantilever, hull_vertices, shaped
from .kinematics import (
    SIDES,
    balance_x,
    body_placements,
    equilibrium,
    leg_fk,
    posture,
    posture_target,
    rotation,
    support_feedforward,
)

JOINTS = ["left_hip", "left_knee", "right_hip", "right_knee"]
# Hinge suffix -> config m10.physics end_stops_deg / end_stop_pads key.
END_STOPS = {"hip": "thigh_a1", "knee": "knee_drive_a2", "shin_joint": "knee_angle"}
WHEELS = ["left_wheel_joint", "right_wheel_joint"]
SHINS = ["left_shin_joint", "right_shin_joint"]
ROTORS = [f"{s}_{r}_rotor_joint" for s in SIDES for r in ("thigh", "knee")]
MESH = ROOT / "artifacts/m10/assembly/render-mesh.json.gz"
# Printed exterior parts that carry collision hulls, by link.
EXTERIOR = {
    "body": [
        "BodyCover",
        "BodyTub",
        "BatteryHatch",
        "HeadFront",
        "HeadRear",
        "Neck",
        "Ear",
        "HipHousing",
        "FrontCameraBezel",
    ],
    "thigh": ["Thigh", "ThighLid", "HipCap"],
    "shin": ["Shin"],
}


def _cad_hip_com(row):
    """A CAD rigid body's centre of mass in the hip frame at the CAD pose."""
    origin, angle = row["frame"]["origin_hip_m"], math.radians(row["frame"]["angle_deg"])
    x, y, z = row["com_link_m"]
    c, s = math.cos(angle), math.sin(angle)
    return np.array([origin[0] + c * x - s * z, y, origin[1] + s * x + c * z])


def parameters(shared):
    """Flatten config m10 (geometry, CAD rigid bodies, physics) into a plant profile."""
    c = shared["m10"]
    cad, phys = c["cad"], c["physics"]
    p = {"profile": "m10", "name": "bipe_m10_serial", "units": "SI"}
    rb = {}
    for name, row in cad["rigid_bodies"].items():
        rb["chassis" if name == "body" else name] = {
            "mass_kg": row["mass_kg"],
            "com_m": list(row["com_link_m"]),
            "inertia_kg_m2": [list(r) for r in row["inertia_about_com_link_kg_m2"]],
        }
    # Mass allowances (harness, fasteners, head pan reserve) are point masses on the body.
    # Harness and fasteners sit at the centre of mass of all body-mounted CAD: the body
    # plus the knee drive trains and rotors, whose centres lie on their body-fixed axes.
    b, h = c["body"], c["head"]
    allow = c["mass_properties"]["allowances_kg"]
    head_z = -b["hip_z_above_bottom"] + b["height"] + h["neck_height"] + h["pod_height"] / 2
    zero = [[0.0] * 3 for _ in range(3)]
    mounted = [
        (row["mass_kg"], _cad_hip_com(row))
        for name, row in cad["rigid_bodies"].items()
        if name == "body" or name.endswith(("_kneedrive", "rotor"))
    ]
    mounted_com = sum(m * x for m, x in mounted) / sum(m for m, _ in mounted)
    rb["chassis"] = combine(
        [
            rb["chassis"],
            {
                "mass_kg": allow["harness"] + allow["fasteners_and_inserts"],
                "com_m": [mounted_com[0], 0.0, mounted_com[2]],
                "inertia_kg_m2": zero,
            },
            {
                "mass_kg": allow["head_pan_reserve"],
                "com_m": [-b["hip_x"] + h["x"], 0.0, head_z],
                "inertia_kg_m2": zero,
            },
        ]
    )
    p["rigid_bodies"] = rb
    lg, w, hb = c["leg"], c["wheel"], c["internals"]["hip_belt"]
    p["geometry"] = {
        "thigh_length": lg["thigh_length"],
        "shin_length": lg["shin_length"],
        "wheel_radius": w["diameter"] / 2,
        "wheel_width": w["width"],
        "wheel_track": 2 * w["y_center"],
        "tire_section": c["exterior"]["wheel"]["tire_section"],
        "motor_row_x": -hb["centre_distance"],
        "motor_row_z": 0.0,
        "nominal_height": c["poses"]["nominal_hip_height"] - w["diameter"] / 2,
        "body_length": b["length"],
        "body_width": b["width"],
        "body_height": b["height"],
        "body_center": [-b["hip_x"], 0.0, -b["hip_z_above_bottom"] + b["height"] / 2],
        "hip_z_above_bottom": b["hip_z_above_bottom"],
        "shell_wall": c["internals"]["wall"],
    }
    p["joint_drive"] = {"ratio": phys["hip_motor"]["ratio"], "knee_belt_ratio": 1.0}
    p["physics"] = copy.deepcopy(phys)
    p["ear"] = copy.deepcopy(c["head"]["ear"])
    p["simulation"] = copy.deepcopy(phys["simulation"])
    p["sizing"] = copy.deepcopy(shared["sizing"])
    p["sizing"]["mass_ceiling"] = (
        c["targets"]["allocated_mass_ceiling_kg"] + c["targets"]["minimum_reserve_kg"]
    )
    # The M12 drive envelope sets the firmware command gate (BIPE_DRIVE_SPEED).
    envelope = c["targets"].get("drive_envelope")
    if envelope:
        p["sizing"]["drive_speed"] = envelope["speed_m_s"]
    p["contact"] = copy.deepcopy(shared["contact"])
    p["contact_materials"] = json.loads((ROOT / phys["contact"]["material_source"]).read_text())
    p["geometry_source_sha256"] = hashlib.sha256(
        json.dumps(c, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    p["collision_mesh_sha256"] = hashlib.sha256(MESH.read_bytes()).hexdigest()
    mapping = copy.deepcopy(phys["posture_mapping"])
    p["posture_mapping"] = mapping
    # Hip heights are above the ground; the leg places the axle one wheel radius higher.
    radius = p["geometry"]["wheel_radius"]
    mapping["axle_depth_m"] = [h - radius for h in mapping["hip_height_m"]]
    mapping["axle_x_m"] = [balance_x(depth, p) for depth in mapping["axle_depth_m"]]
    hm, gl, fr = phys["hip_motor"], phys["hub_motor"], phys["friction"]
    hip_speed = hm["no_load_speed_motor_rad_s"] / hm["ratio"]
    lim = phys["end_stops_deg"]
    p["articulation"] = {
        "peak_torque_per_joint": hm["normal_command_cap_Nm"],
        "continuous_torque_reference": hm["continuous_torque_motor_Nm"]
        * hm["ratio"]
        * hm["efficiency_screen"],
        "no_load_speed": hip_speed,
        "torque_time_constant": hm["FOC_time_constant_s"],
        "rotor_inertia_at_output": hm["rotor_core_inertia_kg_m2"] * hm["ratio"] ** 2,
        "viscous_friction": fr["hip_viscous_Nm_s_rad"],
        "coulomb_friction": hip_pretension_friction(p),
        "hip_limits_deg": lim["thigh_a1"],
        "knee_limits_deg": lim["knee_drive_a2"],
        "knee_fold_direction": -1,
    }
    p["motor"] = {
        "peak_torque_per_wheel": gl["peak_torque_Nm"],
        "continuous_torque_per_wheel": gl["rated_torque_Nm"],
        "no_load_speed": gl["drive_voltage_limit_V"] / gl["back_emf_V_s_rad"],
        "torque_time_constant": gl["FOC_time_constant_s"],
        "rotor_inertia_at_output": gl["rotor_inertia_kg_m2"],
        "viscous_friction": fr["wheel_viscous_Nm_s_rad"],
        "coulomb_friction": fr["wheel_coulomb_Nm"],
        "peak_basis": "GL40 KV70 direct drive at the driver current and voltage limits",
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
    g = p["geometry"]
    g.update(
        link_length=g["thigh_length"],
        body_center_above_hip=g["body_center"][2],
        fixed_leg_angle_deg=40.0,
    )
    return p


def hip_pretension_friction(p):
    """Ball-bearing friction moment on the hip outputs from hip-belt pretension (SKF simplified)."""
    f = p["physics"]["friction"]
    pull = 2 * p["physics"]["belts"]["hip_pretension_per_span_N"]
    return 0.5 * f["ball_bearing_mu"] * pull * f["hip_bearing_bore_m"]


def _hinge(body, name, axis="0 -1 0", **attrs):
    return ET.SubElement(
        body, "joint", name=name, type="hinge", axis=axis, **{k: str(v) for k, v in attrs.items()}
    )


@lru_cache(maxsize=1)
def _exterior_meshes():
    """Convex hulls of the B3 printed exterior parts, in their CAD link frames.

    Reads the nominal-pose tessellation that cad/m10/assembly.py exports and
    the link frames of that pose from artifacts/m10/cad/mass.json. Concave
    pockets are filled by the hulls; the tire is a cylinder geom instead.
    """
    data = json.loads(gzip.decompress(MESH.read_bytes()))
    pose = json.loads((ROOT / "artifacts/m10/cad/mass.json").read_text())["pose"]
    a1, a2 = math.radians(pose["a1_deg"]), math.radians(pose["a2_deg"])
    knee, axle = pose["knee_m"], pose["axle_m"]
    # The tessellation is in the CAD world frame (cad/m10/exterior.py to_world: ground at
    # z = 0, origin over the wheel contacts); the link frames below are hip-relative.
    hip = np.array([-axle[0], 0.0, pose["hip_height_m"]])
    frames = {
        "body": (np.zeros(3), np.eye(3)),
        "thigh": (np.zeros(3), rotation(a1)),
        "shin": (np.array([knee[0], 0, knee[1]]), rotation(a2)),
        "wheel": (np.array([axle[0], 0, axle[1]]), np.eye(3)),
    }
    rows, seen = [], {}
    for comp in data["poses"]["nominal"]:
        link = comp["link"]
        role = link.split("_", 1)[1] if link.startswith(("left_", "right_")) else link
        part = (
            comp["name"].split("_", 1)[1]
            if comp["name"].startswith(("left_", "right_"))
            else comp["name"]
        )
        if part not in EXTERIOR.get(role, []):
            continue
        origin, rot = frames[role]
        vertices = (np.asarray(comp["vertices"]) - hip - origin) @ rot
        hull = ConvexHull(vertices)
        ids = hull.vertices
        lookup = {v: i for i, v in enumerate(ids)}
        faces = np.array([[lookup[i] for i in face] for face in hull.simplices])
        count = seen.get(comp["name"], 0)
        seen[comp["name"]] = count + 1
        name = comp["name"] + (f"_{count}" if count else "")
        color = data["colors"][comp["color_key"]]
        rows.append((name, "chassis" if link == "body" else link, vertices[ids], faces, color))
    return rows


def _bumper_pads(p):
    """Phase D bumper geoms (config m10.physics.bumpers): the frame-mounted skid pad under
    the body tub and a foam ring on each wheel's outer face, outboard of the tire."""
    bumpers = p["physics"].get("bumpers", {})
    if not bumpers.get("enabled"):
        return []
    g, rows = p["geometry"], []
    skid = bumpers["skid"]
    lx, ly, lz = skid["size_m"]
    rows.append(
        {
            "name": "SkidPad",
            "body": "chassis",
            "shape": "box",
            "size_m": skid["size_m"],
            "thickness_m": lz,
            **{k: skid[k] for k in ("end_bevel_deg", "flat_length_m") if k in skid},
            "geom": {
                "name": "SkidPad",
                "type": "box",
                "pos": values([skid["center_x_m"], 0.0, -g["hip_z_above_bottom"] - lz / 2]),
                "size": values([lx / 2, ly / 2, lz / 2]),
                "rgba": ".16 .17 .19 1",
            },
        }
    )
    ring = bumpers["wheel_ring"]
    for side in SIDES:
        sign = 1 if side == "left" else -1
        t = ring["thickness_m"]
        rows.append(
            {
                "name": f"{side}_WheelRing",
                "body": f"{side}_wheel",
                "shape": "ring",
                "outer_radius_m": ring["outer_radius_m"],
                "inner_radius_m": ring["inner_radius_m"],
                "thickness_m": t,
                **({"dome_deg": ring["dome_deg"]} if "dome_deg" in ring else {}),
                "geom": {
                    "name": f"{side}_WheelRing",
                    "type": "cylinder",
                    "pos": values([0, sign * (g["wheel_track"] / 2 + g["wheel_width"] / 2 + t / 2), 0]),
                    # Local +z (the ring's outer face) points outboard.
                    "quat": values([math.sqrt(0.5), -sign * math.sqrt(0.5), 0, 0]),
                    "size": values([ring["outer_radius_m"], t / 2]),
                    "rgba": ".16 .17 .19 1",
                },
            }
        )
    for row in rows:
        geom = row["geom"]
        row["frame"] = (
            [float(v) for v in geom["pos"].split()],
            [float(v) for v in geom.get("quat", "1 0 0 0").split()],
        )
        if shaped(row):
            # Shaped faces collide as the convex hull of the foam face and its backing.
            del row["geom"]["size"]
            row["geom"].update(type="mesh", mesh=row["name"] + "_mesh")
            row["vertices"] = hull_vertices(row)
    return rows


def generate_xml(p):
    phys, g = p["physics"], p["geometry"]
    root = ET.Element("mujoco", model="bipe_m10_free_serial")
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
    ET.SubElement(base, "site", name="imu", pos=values(phys["imu_position_m"]), size=".003")
    nodes = {"chassis": base}
    tendons = ET.SubElement(root, "tendon")
    fr, lim = phys["friction"], phys["end_stops_deg"]
    ratio = p["joint_drive"]["ratio"]
    rotor_pos = values([g["motor_row_x"], 0, g["motor_row_z"]])
    for side, sign in zip(SIDES, [1, -1], strict=True):
        thigh = ET.SubElement(base, "body", name=f"{side}_thigh")
        _hinge(
            thigh,
            f"{side}_hip",
            damping=fr["hip_viscous_Nm_s_rad"],
            frictionloss=hip_pretension_friction(p),
            range=values(np.radians(lim["thigh_a1"])),
        )
        drive = ET.SubElement(base, "body", name=f"{side}_kneedrive")
        _hinge(
            drive,
            f"{side}_knee",
            damping=fr["hip_viscous_Nm_s_rad"],
            frictionloss=hip_pretension_friction(p),
            range=values(np.radians(lim["knee_drive_a2"])),
        )
        shin = ET.SubElement(
            thigh, "body", name=f"{side}_shin", pos=values([g["thigh_length"], 0, 0])
        )
        _hinge(
            shin,
            f"{side}_shin_joint",
            damping=fr["knee_viscous_Nm_s_rad"],
            frictionloss=fr["knee_bearing_coulomb_Nm"],
            range=values(np.radians(lim["knee_angle"])),
        )
        wheel = ET.SubElement(
            shin, "body", name=f"{side}_wheel", pos=values([g["shin_length"], 0, 0])
        )
        _hinge(
            wheel,
            f"{side}_wheel_joint",
            axis="0 1 0",
            damping=fr["wheel_viscous_Nm_s_rad"],
            frictionloss=fr["wheel_coulomb_Nm"],
        )
        ET.SubElement(
            wheel,
            "geom",
            name=f"{side}_tire",
            type="cylinder",
            pos=values([0, sign * g["wheel_track"] / 2, 0]),
            quat=values([math.sqrt(0.5), math.sqrt(0.5), 0, 0]),
            size=values([g["wheel_radius"], g["wheel_width"] / 2]),
            rgba=".42 .42 .42 1",
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
        for role in ("thigh", "knee"):
            rotor = ET.SubElement(base, "body", name=f"{side}_{role}rotor", pos=rotor_pos)
            _hinge(
                rotor,
                f"{side}_{role}_rotor_joint",
                damping=phys["hip_motor"]["viscous_motor_Nm_s_rad"],
                frictionloss=phys["hip_motor"]["coulomb_motor_Nm"],
            )
            nodes[f"{side}_{role}rotor"] = rotor
            # AS5047P on the rotor: the output angle it reports includes belt deflection.
            encoder = ET.SubElement(tendons, "fixed", name=f"{side}_{role}_rotor_output")
            ET.SubElement(encoder, "joint", joint=f"{side}_{role}_rotor_joint", coef=str(1 / ratio))
    for name, node in nodes.items():
        add_inertial(node, p["rigid_bodies"][name])
    for pad in _bumper_pads(p):
        if "vertices" in pad:
            ET.SubElement(
                asset, "mesh", name=pad["name"] + "_mesh", vertex=values(pad["vertices"].ravel())
            )
        ET.SubElement(nodes[pad["body"]], "geom", **pad["geom"])
    for name, body, vertices, faces, color in _exterior_meshes():
        ET.SubElement(
            asset,
            "mesh",
            name=name + "_mesh",
            vertex=values(vertices.ravel()),
            face=" ".join(str(v) for v in faces.ravel()),
        )
        ET.SubElement(
            nodes[body],
            "geom",
            name=name,
            type="mesh",
            mesh=name + "_mesh",
            rgba=values(list(color) + [1]),
        )
    actuators = ET.SubElement(root, "actuator")
    hm, gl = phys["hip_motor"], phys["hub_motor"]
    for name, peak in [(n, hm["peak_torque_motor_Nm"]) for n in ROTORS] + [
        (n, gl["peak_torque_Nm"]) for n in WHEELS
    ]:
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
    for side in SIDES:
        for joint, role in ((f"{side}_hip", "thigh"), (f"{side}_knee", "knee")):
            ET.SubElement(
                sensors, "tendonpos", name=joint + "_encoder", tendon=f"{side}_{role}_rotor_output"
            )
    for name in WHEELS:
        ET.SubElement(sensors, "jointpos", name=name + "_encoder", joint=name)
    return ET.tostring(root, encoding="unicode")


def tire_force(p, depth, materials, axial_fraction=0, stiffness_scale=1.0):
    """Solid SLS TPU tire on a plane as a Winkler layer of the tire section thickness.

    Radial contact integrates the cylindrical patch; lateral contact uses the
    annular face. Knob relief is neglected (a stated assumption).
    """
    if depth <= 0:
        return 0.0
    g = p["geometry"]
    radius, width, section = g["wheel_radius"], g["wheel_width"], g["tire_section"]
    modulus = materials["materials"]["tpu"]["modulus_Pa"] * stiffness_scale
    d = min(depth, section)
    half_patch = math.sqrt(max(0, 2 * radius * d - d * d))
    x = np.linspace(-half_patch, half_patch, 33)
    local = np.maximum(d - (radius - np.sqrt(np.maximum(radius**2 - x * x, 0))), 0)
    radial = width * modulus / section * float(np.trapezoid(local, x))
    lateral = modulus / width * _tilted_face_integral(radius, radius - section, depth, axial_fraction)
    return (1 - axial_fraction) * radial + axial_fraction * lateral


def _tilted_face_integral(outer, inner, depth, axial_fraction):
    """Integral of the penetration over the annular side face of a tilted tire, m^3.

    axial_fraction is |contact normal . axle|, the cosine of the face's tilt from the
    ground. The face's lowest rim point is `depth` into the ground and the penetration
    falls by sin(tilt) per metre across the face, so a tilted tire bears on an edge
    sliver; a flat face (tilt 0) gives the full annulus times the depth.
    """
    slope = math.sqrt(max(0.0, 1.0 - axial_fraction**2))
    if slope * 2 * outer <= 1e-3 * depth:
        return math.pi * (outer**2 - inner**2) * depth
    span = 2 * outer if slope * 2 * outer <= depth else depth / slope
    u = np.linspace(-outer, -outer + span, 65)
    chord = 2 * (
        np.sqrt(np.maximum(outer**2 - u * u, 0)) - np.sqrt(np.maximum(inner**2 - u * u, 0))
    )
    return float(np.trapezoid(np.maximum(depth - (u + outer) * slope, 0) * chord, u))


class Contacts(MaterialContacts):
    """M9 material-contact machinery with the M10 solid TPU tire and PA12 printed exterior.

    The law is mapped through the solver's row diagonal (solver_mobility), so the
    solved normal force equals the material law in every configuration.
    """

    solver_mobility = True

    def configure(self, m):
        super().configure(m)
        bumpers = self.p["physics"].get("bumpers", {})
        materials = self.materials["materials"]
        self.face, self.ears = None, None
        if bumpers.get("enabled"):
            self.face = FacePads(
                m, _bumper_pads(self.p), materials[bumpers["foam"]], bumpers["stiffness_scale"]
            )
            ear = bumpers["ear"]
            if ear["material"] == "tpu":
                stiffness, strain_per_m = ear_cantilever(self.p["ear"], materials["tpu"])
                self.ears = {
                    "names": ("Ear", "Ear_1"),
                    "stiffness_N_m": stiffness,
                    "strain_per_m": strain_per_m,
                    "allowable_strain": ear["elongation_at_break_Z"] * ear["strain_knockdown"],
                }
        pads = self.p["physics"].get("end_stop_pads")
        self.stops, self.peak_stop_strain = {}, {}
        if pads is None:
            return
        materials = self.materials["materials"]
        pad_e = materials[pads["material"]]["modulus_Pa"]
        backing_e = materials[pads["backing_material"]]["modulus_Pa"]
        impedance = self.materials["contacts"]["impedance"]
        for side in SIDES:
            for joint, key in END_STOPS.items():
                j = m.joint(f"{side}_{joint}").id
                pad = pads["joints"][key]
                arm = pad["area_m2"] * pad["radius_m"] ** 2
                self.stops[j] = {
                    "name": f"{side}_{joint}",
                    "stiffness": pad_e * arm / pad["thickness_m"],
                    "backing": backing_e * arm / pads["backing_thickness_m"],
                    "travel": pads["max_strain"] * pad["thickness_m"] / pad["radius_m"],
                    "strain_per_rad": pad["radius_m"] / pad["thickness_m"],
                    "dof": int(m.jnt_dofadr[j]),
                }
                m.jnt_solimp[j] = [impedance, impedance, 0.001, 0.5, 2]

    def stop_torque(self, joint, overtravel):
        """Pad torque at an overtravel (rad): TPU pad, then PA12 backing past max strain."""
        s = self.stops[joint]
        return s["stiffness"] * overtravel + s["backing"] * max(0.0, overtravel - s["travel"])

    def _end_stops(self, m, d):
        rows = [
            i
            for i in range(d.nefc)
            if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and d.efc_id[i] in self.stops
        ]
        for i in rows:
            j = int(d.efc_id[i])
            s = self.stops[j]
            overtravel = max(0.0, -float(d.efc_pos[i]))
            self.peak_stop_strain[s["name"]] = max(
                self.peak_stop_strain.get(s["name"], 0.0), overtravel * s["strain_per_rad"]
            )
            sample = max(overtravel, 1e-6)
            stiffness = self.stop_torque(j, sample) / sample
            unit = np.zeros((1, m.nv))
            unit[0, s["dof"]] = 1.0
            mobility = np.zeros((1, m.nv))
            mujoco.mj_solveM(m, d, mobility, unit)
            damping = 2 * self.zeta * math.sqrt(stiffness / mobility[0, s["dof"]])
            row = float(d.efc_diagA[i])
            m.jnt_solref[j] = [-stiffness * row, -damping * row]

    def __call__(self, m, d):
        # Stop-pad parameters first: the material-contact call rebuilds every constraint row.
        if self.stops:
            self._end_stops(m, d)
        if self.face is not None:
            self.face.update(m, d)
        super().__call__(m, d)

    def category(self, name):
        if self.face is not None and name in self.face.rows:
            return "foam_pad"
        if self.ears and name in self.ears["names"]:
            return "ear"
        return "tire" if "tire" in name else "shell"

    def force(self, name, depth, axial_fraction=0):
        kind = self.category(name)
        if kind == "tire":
            return tire_force(self.p, depth, self.materials, axial_fraction, self.scale)
        if kind == "foam_pad":
            # Face integral of the foam law at the pad's current pose; a contact on the
            # pad geom away from the foam face (e.g. the ring's hub hole) carries ~0.
            return max(self.face.force[name], 1e-9)
        if kind == "ear":
            return self.ears["stiffness_N_m"] * depth
        backing = self.p["physics"]["contact"].get("shell_backing", {}).get("parts", {})
        if name in backing:
            # M9 strip law with the backing at the gap measured behind the strike spot
            # (m10.physics.contact.shell_backing) instead of the 4 mm travel assumption.
            c = self.materials["contacts"]
            return self.shell["stiffness_N_m"] * self.scale * depth + (
                max(0, depth - backing[name]["travel_m"])
                * c["metal_contact_stiffness_N_m_assumption"]
                * self.backing_scale
            )
        return super().force("Shell", depth, axial_fraction)


def build(p):
    model = mujoco.MjModel.from_xml_string(generate_xml(p))
    Contacts(p).configure(model)
    return model, mujoco.MjData(model)


def _qadr(m, name):
    return m.joint(name).qposadr[0]


def _refit_world_bvh(m, ground):
    """Refit the world body's bounding volume to the ground plane's current orientation.

    The compiler bounds the world body (the ground plane alone) by the flat half-space
    z <= 0. A plane tilted after compilation rises above z = 0 uphill, so the stale bound
    culls valid contacts in the midphase for any body with several geoms (the wheels once
    the bumper rings are fitted).
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
    ratio = p["joint_drive"]["ratio"]
    pitch = equilibrium(p, q) + math.radians(offset_deg)
    # Static belt pretension is an initial condition, never reapplied as a pose clamp.
    preload = support_feedforward(p, q, pitch)
    b = p["physics"]["belts"]
    hip_half, knee_half = (math.radians(b[k]) / 2 for k in ("hip_lash_deg", "knee_lash_deg"))
    for i, side in enumerate(SIDES):
        a1, a2 = q[2 * i : 2 * i + 2]
        t1, t2 = preload[2 * i : 2 * i + 2]
        drive = a2 + np.sign(t2) * knee_half + t2 / b["knee_stiffness_Nm_rad"]
        d.qpos[_qadr(m, f"{side}_hip")] = a1
        d.qpos[_qadr(m, f"{side}_knee")] = drive
        d.qpos[_qadr(m, f"{side}_shin_joint")] = a2 - a1
        d.qpos[_qadr(m, f"{side}_thigh_rotor_joint")] = ratio * (
            a1 + np.sign(t1) * hip_half + t1 / b["hip_stiffness_Nm_rad"]
        )
        d.qpos[_qadr(m, f"{side}_knee_rotor_joint")] = ratio * (
            drive + np.sign(t2) * hip_half + t2 / b["hip_stiffness_Nm_rad"]
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
    # Clear the compiler's body-aligned flag before tilting the plane (see M9 plant).
    m.geom_sameframe[ground] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
    m.geom_quat[ground] = [math.cos(slope / 2), 0, -math.sin(slope / 2), 0]
    _refit_world_bvh(m, ground)
    mujoco.mj_forward(m, d)
    if slope_deg:
        axle3 = np.mean([d.site(s + "_axle").xpos for s in SIDES], axis=0)
        n = np.array([-math.sin(slope), 0, math.cos(slope)])
        d.qpos[2] += (p["geometry"]["wheel_radius"] - 0.0001 - n @ axle3) / n[2]
    contacts = Contacts(p)
    contacts.configure(m)
    _material_forward(m, d, contacts)


def _material_forward(m, d, contacts):
    mujoco.mj_forward(m, d)
    contacts(m, d)
    mujoco.mj_fwdConstraint(m, d)
    # Refresh the IMU at the rebuilt constraint solution (see M9 plant).
    mujoco.mj_rnePostConstraint(m, d)
    mujoco.mj_sensorAcc(m, d)


@lru_cache(maxsize=16)
def topology(m):
    thigh_q = np.array([_qadr(m, f"{s}_hip") for s in SIDES])
    shin_q = np.array([_qadr(m, f"{s}_shin_joint") for s in SIDES])
    thigh_v = np.array([m.joint(f"{s}_hip").dofadr[0] for s in SIDES])
    shin_v = np.array([m.joint(f"{s}_shin_joint").dofadr[0] for s in SIDES])
    wv = np.array([m.joint(n).dofadr[0] for n in WHEELS])
    return (
        thigh_q,
        shin_q,
        thigh_v,
        shin_v,
        wv,
        [m.site(s + "_axle").id for s in SIDES],
        m.body("chassis").id,
        m.sensor("gyro").adr[0],
    )


def truth(m, d, p):
    """True physical state. q is [a1, actual shin angle] per side (after both belts)."""
    thigh_q, shin_q, thigh_v, shin_v, wv, sites, chassis, ga = topology(m)
    rot = d.xmat[chassis].reshape(3, 3)
    pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
    roll, yaw = math.atan2(rot[2, 1], rot[2, 2]), math.atan2(rot[1, 0], rot[0, 0])
    a1, a1d = d.qpos[thigh_q], d.qvel[thigh_v]
    a2, a2d = a1 + d.qpos[shin_q], a1d + d.qvel[shin_v]
    q = np.ravel(np.column_stack([a1, a2]))
    qd = np.ravel(np.column_stack([a1d, a2d]))
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
    # Wheel joints are relative to the shin, whose body-frame angle a2 changes in a crouch.
    world_wheel_speed = d.qvel[wv] + gyro[1] - a2d
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


def assembled_world(p, q, height):
    """Whole-robot mass, COM and inertia from the rigid bodies placed at posture q."""
    place = body_placements(p, q)
    rows = []
    for name, row in p["rigid_bodies"].items():
        origin, rot = place[name]
        rows.append(
            {
                "mass_kg": row["mass_kg"],
                "com_m": origin + rot @ row["com_m"] + [0, 0, height],
                "inertia_kg_m2": rot @ np.asarray(row["inertia_kg_m2"]) @ rot.T,
            }
        )
    return combine(rows)


def geometry_check(p, config=None):
    """Plant mass against the CAD allocation, IK/FK closure and postures inside the end stops."""
    config = config or json.loads((ROOT / "config/robot.json").read_text())["m10"]
    mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())
    lim = p["physics"]["end_stops_deg"]
    closure, inside = 0.0, True
    for angle in np.linspace(30, 65, 71):
        q = posture(float(angle), p)
        height, x = posture_target(float(angle), p)
        closure = max(closure, float(np.linalg.norm(leg_fk(q[:2], p) - [x, -height])))
        a1, a2 = np.degrees(q[:2])
        inside &= bool(
            lim["thigh_a1"][0] <= a1 <= lim["thigh_a1"][1]
            and lim["knee_drive_a2"][0] <= a2 <= lim["knee_drive_a2"][1]
            and lim["knee_angle"][0] <= a2 - a1 <= lim["knee_angle"][1]
        )
    mass_error = abs(mass - config["cad"]["allocated_kg"])
    return {
        "profile": "m10",
        "rigid_body_count": len(p["rigid_bodies"]),
        "mass_kg": mass,
        "allocated_mass_error_kg": mass_error,
        "closure_error_m": closure,
        "postures_inside_end_stops": inside,
        "passed": mass_error < 1e-3 and closure < 1e-10 and inside,
    }


def hub_drive_resistance(gl):
    """GL40 line-to-line resistance for the voltage limit: hot winding plus the harness leads."""
    rise = gl["voltage_limit_winding_temperature_C"] - 20
    hot = 1 + gl["copper_resistance_tempco_per_C"] * rise
    return gl["phase_to_phase_resistance_ohm"] * hot + gl["lead_resistance_ohm"]


class MechanicalDrives:
    """Four RO40 rotors on 4:1 hip belts, two 1:1 knee belts and two direct GL40 hubs.

    Belt rows (gap, positive = output lags its driver):
      0..3  hip belts: rotor / ratio - output (thigh a1 or knee drive a2)
      4..5  knee belts: knee drive - (thigh + shin relative), the actual shin angle
    A backward-Euler belt-force predictor uses the full mass matrix, as in the
    M9 plant; MuJoCo integrates every robot coordinate.
    """

    def __init__(self, m, p, disturbance=None):
        self.p, self.disturbance = p, disturbance or {}
        phys = p["physics"]
        self.ratio = p["joint_drive"]["ratio"]
        self.rq = np.array([_qadr(m, n) for n in ROTORS])
        self.wv = np.array([m.joint(n).dofadr[0] for n in WHEELS])
        self.rv = np.r_[np.array([m.joint(n).dofadr[0] for n in ROTORS]), self.wv]
        self.oq = np.array([_qadr(m, n) for n in JOINTS])
        self.ov = np.array([m.joint(n).dofadr[0] for n in JOINTS])
        self.sq = np.array([_qadr(m, n) for n in SHINS])
        self.sv = np.array([m.joint(n).dofadr[0] for n in SHINS])
        thigh_v, knee_v = self.ov[0::2], self.ov[1::2]
        self.J = np.zeros((6, m.nv))
        self.J[np.arange(4), self.rv[:4]] = 1 / self.ratio
        self.J[np.arange(4), self.ov] = -1
        self.J[4 + np.arange(2), knee_v] = 1
        self.J[4 + np.arange(2), thigh_v] = -1
        self.J[4 + np.arange(2), self.sv] = -1
        self.inverse_rows = np.zeros_like(self.J)
        b, dist = phys["belts"], self.disturbance
        hip_k = dist.get("hip_belt_stiffness_Nm_rad", b["hip_stiffness_Nm_rad"])
        knee_k = dist.get("knee_belt_stiffness_Nm_rad", b["knee_stiffness_Nm_rad"])
        self.stiffness = np.r_[np.full(4, hip_k), np.full(2, knee_k)]
        # Damping keeps each belt's nominal time constant when its stiffness is varied.
        self.damping = np.r_[
            np.full(4, b["hip_damping_Nm_s_rad"] * hip_k / b["hip_stiffness_Nm_rad"]),
            np.full(2, b["knee_damping_Nm_s_rad"] * knee_k / b["knee_stiffness_Nm_rad"]),
        ]
        self.half = np.r_[
            np.full(4, math.radians(dist.get("hip_backlash_deg", b["hip_lash_deg"])) / 2),
            # The inherited backlash family (backlash_deg) varies the knee belt, the M10
            # drive closest to the M8 wheel gearbox it was written for.
            np.full(
                2,
                math.radians(
                    dist.get("knee_backlash_deg", dist.get("backlash_deg", b["knee_lash_deg"]))
                )
                / 2,
            ),
        ]
        self.motor_torque = np.zeros(6)
        self.phase_current = np.zeros(6)
        self.last_forces = np.zeros(6)
        self.peak_belt_torque = np.zeros(6)
        self.peak_motor_current = np.zeros(6)
        self.peak_bus_limited = np.zeros(2)
        self.step_count = 0
        self.contacts = Contacts(p)
        self.contacts.configure(m)
        self._thigh_ids = [m.body(f"{s}_thigh").id for s in SIDES]
        self._drive_ids = [m.body(f"{s}_kneedrive").id for s in SIDES]
        self._shin_ids = [m.body(f"{s}_shin").id for s in SIDES]

    def forward(self, m, d, contact_callback=None):
        _material_forward(m, d, contact_callback or self.contacts)

    def motor_speed(self, d):
        return np.r_[d.qvel[self.rv[:4]] / self.ratio, d.qvel[self.wv]]

    def _coordinates(self, d):
        thigh, drive = d.qpos[self.oq[0::2]], d.qpos[self.oq[1::2]]
        return np.r_[
            d.qpos[self.rq] / self.ratio - d.qpos[self.oq],
            drive - thigh - d.qpos[self.sq],
        ]

    def _hip_motor(self, requested, speed, dt):
        """RO40 through the 4:1 belt: command caps, torque-speed limit, FOC current lag."""
        hm = self.p["physics"]["hip_motor"]
        caps = np.full(4, hm["normal_command_cap_Nm"])
        command = np.clip(requested, -caps, caps) / (self.ratio * hm["efficiency_screen"])
        maximum = np.where(
            command * speed > 0,
            hm["peak_torque_motor_Nm"]
            * np.maximum(0, 1 - abs(speed) / hm["no_load_speed_motor_rad_s"]),
            hm["peak_torque_motor_Nm"],
        )
        command = np.clip(command, -maximum, maximum)
        rated, peak = hm["continuous_torque_motor_Nm"], hm["peak_torque_motor_Nm"]
        ir, ip = hm["rated_current_A"], hm["peak_current_A"]
        desired = np.sign(command) * np.where(
            abs(command) <= rated,
            abs(command) * ir / rated,
            ir + (abs(command) - rated) * (ip - ir) / (peak - rated),
        )
        change = -np.expm1(-dt / hm["FOC_time_constant_s"]) * (desired - self.phase_current[:4])
        slew = hm["FOC_current_slew_limit_A_s"] * dt
        self.phase_current[:4] += np.clip(change, -slew, slew)
        current = self.phase_current[:4]
        torque = np.sign(current) * np.where(
            abs(current) <= ir,
            abs(current) * rated / ir,
            rated + (abs(current) - ir) * (peak - rated) / (ip - ir),
        )
        torque = np.clip(torque, -maximum, maximum)
        saturated = (
            abs(requested) > np.minimum(caps, maximum * self.ratio * hm["efficiency_screen"]) + 1e-8
        )
        return torque, torque * self.ratio * hm["efficiency_screen"], saturated

    def _hub_motor(self, requested, speed, dt):
        """GL40 direct drive: linear Kt, driver current limit and the drive-voltage limit.

        Motoring current is limited by (V - Ke w) / R_hot with the bus voltage
        limit V; braking current is limited by the peak current only.
        """
        gl = self.p["physics"]["hub_motor"]
        kt, ip = gl["torque_constant_Nm_A"], gl["peak_current_A"]
        volt = (gl["drive_voltage_limit_V"] - gl["back_emf_V_s_rad"] * abs(speed)) / (
            hub_drive_resistance(gl)
        )
        limit = np.where(requested * speed > 0, np.clip(volt, 0, ip), ip)
        desired = np.clip(requested / kt, -limit, limit)
        change = -np.expm1(-dt / gl["FOC_time_constant_s"]) * (desired - self.phase_current[4:])
        slew = gl["FOC_current_slew_limit_A_s"] * dt
        self.phase_current[4:] += np.clip(change, -slew, slew)
        self.phase_current[4:] = np.clip(self.phase_current[4:], -limit, limit)
        self.peak_bus_limited = np.maximum(self.peak_bus_limited, (limit < ip).astype(float))
        torque = kt * self.phase_current[4:]
        return torque, torque.copy(), abs(requested) > kt * limit + 1e-8

    def apply(self, m, d, requested, dt=None, contact_callback=None):
        dt = float(m.opt.timestep if dt is None else dt)
        requested = np.asarray(requested, dtype=float)
        hip_t, hip_out, hip_sat = self._hip_motor(requested[:4], d.qvel[self.rv[:4]], dt)
        hub_t, hub_out, hub_sat = self._hub_motor(requested[4:], d.qvel[self.wv], dt)
        self.motor_torque = np.r_[hip_t, hub_t]
        d.ctrl[:] = self.motor_torque
        # Remove this object's previous belt force before refreshing the predictor.
        d.qfrc_applied[:] += self.J.T @ self.last_forces
        self.forward(m, d, contact_callback)
        mujoco.mj_solveM(m, d, self.inverse_rows, self.J)
        mobility = self.J @ self.inverse_rows.T
        relative = self.J @ d.qvel
        acceleration = self.J @ d.qacc
        gap = self._coordinates(d)
        predicted = gap + dt * relative + dt * dt * acceleration
        side = np.sign(predicted)
        active = abs(predicted) > self.half
        k, c = self.stiffness * active, self.damping * active
        deformation = gap - side * self.half
        rhs = k * (deformation + dt * relative + dt * dt * acceleration) + c * (
            relative + dt * acceleration
        )
        a = np.eye(6) + (k * dt * dt + c * dt)[:, None] * mobility
        fixed = np.zeros(6, dtype=bool)
        # Re-solve the coupled belt system after a unilateral face releases.
        for _ in range(7):
            matrix, target = a.copy(), rhs.copy()
            matrix[fixed] = np.eye(6)[fixed]
            target[fixed] = 0.0
            force = np.linalg.solve(matrix, target)
            release = (~fixed) & ((~active) | (force * side <= 0))
            if not release.any():
                break
            fixed[np.flatnonzero(release)] = True
        d.qfrc_applied[:] -= self.J.T @ force
        self.last_forces = force
        self.peak_belt_torque = np.maximum(self.peak_belt_torque, abs(force))
        self.peak_motor_current = np.maximum(self.peak_motor_current, abs(self.phase_current))
        self.step_count += 1
        self._bearing_friction(m, d, force)
        return np.r_[hip_out, hub_out], np.r_[hip_sat, hub_sat]

    def _bearing_friction(self, m, d, force):
        """Load-dependent ball-bearing friction plus Coulomb belt loss on the moving members.

        Bearing moment 0.5 mu P d (SKF simplified) from the multibody radial
        reaction; belt loss (1 - efficiency) |belt torque| acts on each belt's
        driven member relative to its carrier (body for hip belts, thigh for knee belts).
        """
        mujoco.mj_rnePostConstraint(m, d)
        fr = self.p["physics"]["friction"]
        loss = (1 - self.p["physics"]["hip_motor"]["efficiency_screen"]) * abs(force)
        base = hip_pretension_friction(self.p)

        def radial(body, joint):
            load = d.cfrc_int[body, 3:]
            axis = d.xaxis[m.joint(joint).id]
            return math.sqrt(max(0.0, float(load @ load - (load @ axis) ** 2)))

        for i, side in enumerate(SIDES):
            for j, (body, joint) in enumerate(
                [(self._thigh_ids[i], f"{side}_hip"), (self._drive_ids[i], f"{side}_knee")]
            ):
                bearing = (
                    0.5 * fr["ball_bearing_mu"] * fr["hip_bearing_bore_m"] * radial(body, joint)
                )
                m.dof_frictionloss[self.ov[2 * i + j]] = base + bearing + loss[2 * i + j]
            knee = (
                0.5
                * fr["ball_bearing_mu"]
                * fr["knee_bearing_bore_m"]
                * radial(self._shin_ids[i], f"{side}_shin_joint")
            )
            m.dof_frictionloss[self.sv[i]] = fr["knee_bearing_coulomb_Nm"] + knee + loss[4 + i]

    def integrate(self, m, d, contact_callback=None):
        mujoco.mj_step1(m, d)
        (contact_callback or self.contacts)(m, d)
        mujoco.mj_step2(m, d)

    def step(self, m, d, requested, dt=None, contact_callback=None):
        result = self.apply(m, d, requested, dt, contact_callback)
        self.integrate(m, d, contact_callback)
        return result

    def metrics(self):
        return {
            "physics_substeps": self.step_count,
            "belt_peak_transmitted_Nm": self.peak_belt_torque.tolist(),
            "phase_current_peak_A": self.peak_motor_current.tolist(),
            "hub_voltage_limited": self.peak_bus_limited.tolist(),
            "last_belt_transmitted_Nm": self.last_forces.tolist(),
            "state_projection_count": 0,
            "rotor_inertia_basis": "RO40 rotor bodies carry the manufacturer core inertia; GL40 rotor inertia is in the wheel body",
        }
