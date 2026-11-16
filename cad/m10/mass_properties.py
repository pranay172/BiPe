"""M10 phase B3 CAD mass and inertia of the detailed assembly.

/snap/bin/freecad.cmd -c "p='cad/m10/mass_properties.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds cad/m10/assembly.py at the nominal pose and gives every part a mass:
volume x density for designed parts, catalog mass spread uniformly over the
CAD envelope for purchased parts, and a catalog-derived fill factor for
bearings (config m10.mass_properties). Parts are grouped into the phase C
rigid bodies: body, and per side thigh, knee drive (inner shaft and its
pulleys), shin, wheel and the two RO40 rotors (catalog rotor split from the
module envelope, plus motor shaft and pulley). Each rigid body's centre of
mass and inertia about it are expressed in its link frame: origin at the
joint, x along the link, y the joint axis.
Writes artifacts/m10/cad/mass.json.
"""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location(
    "m10_assembly", Path(__file__).with_name("assembly.py")
)
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)

C = A.C
MP = C["mass_properties"]
OUT = ROOT / "artifacts/m10/cad"
MM3 = 1e-9  # m3 per mm3
MM2 = 1e-6  # m2 per mm2


def base_name(name):
    """Part key without the side prefix."""
    for side in ("left_", "right_"):
        if name.startswith(side):
            return name[len(side) :]
    return name


def solids_props(shape):
    """Volume (mm3), centroid (mm) and unit-density inertia about it (mm5) summed over solids."""
    solids = shape.Solids or [shape]
    vols = np.array([s.Volume for s in solids])
    cents = np.array([[s.CenterOfMass.x, s.CenterOfMass.y, s.CenterOfMass.z] for s in solids])
    volume = vols.sum()
    centroid = (vols[:, None] * cents).sum(axis=0) / volume
    inertia = np.zeros((3, 3))
    for v, c, s in zip(vols, cents, solids):
        m = s.MatrixOfInertia
        local = np.array([[m.A[r * 4 + k] for k in range(3)] for r in range(3)])
        d = c - centroid
        inertia += local + v * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return volume, centroid, inertia


def part_mass(name, volume_mm3):
    key = base_name(name)
    if key in MP["catalog_kg"]:
        return MP["catalog_kg"][key], "catalog"
    if key in MP["bearing_parts"]:
        return volume_mm3 * MM3 * MP["density_kg_m3"]["steel"] * MP[
            "bearing_fill_factor"
        ], "bearing"
    material = MP["part_material"].get(key)
    if material is None:
        raise KeyError(f"No material or catalog mass for part {name}")
    return volume_mm3 * MM3 * MP["density_kg_m3"][material], material


def link_frame(link, state):
    """Origin (mm) and link angle (rad) of a rigid body's frame in the hip frame."""
    knee = np.array(state["knee_m"]) * 1000
    axle = np.array(state["axle_m"]) * 1000
    a1, a2 = math.radians(state["a1_deg"]), math.radians(state["a2_deg"])
    if link.endswith("_thigh"):
        return np.array([0.0, 0.0]), a1
    if link.endswith("_kneedrive"):
        return np.array([0.0, 0.0]), a2
    if link.endswith("rotor"):
        return np.array([-C["internals"]["hip_belt"]["centre_distance"] * 1000, 0.0]), 0.0
    if link.endswith("_shin"):
        return knee, a2
    if link.endswith("_wheel"):
        return axle, 0.0
    return np.array([0.0, 0.0]), 0.0


def to_link(com_mm, inertia_mm, origin_xz, angle):
    """Express a hip-frame centroid and inertia in a link frame rotated by angle about y."""
    e1 = np.array([math.cos(angle), 0.0, math.sin(angle)])
    e2 = np.array([0.0, 1.0, 0.0])
    e3 = np.array([-math.sin(angle), 0.0, math.cos(angle)])
    R = np.vstack([e1, e2, e3])  # rows: link axes in hip coordinates
    d = com_mm - np.array([origin_xz[0], 0.0, origin_xz[1]])
    return R @ d, R @ inertia_mm @ R.T


def split_rotating(rows):
    """Move the RO40 rotors (catalog split) and their shafts and pulleys into rotor bodies.

    The rotor is a ring on the module axis with the manufacturer core inertia
    about it; the remaining module mass keeps its envelope centroid and its
    inertia scales with its mass. The GL40 rotor keeps its envelope but takes
    the catalog inertia about its axis (config m10.mass_properties.rotating).
    """
    rot = MP["rotating"]
    rotor_of = {"KneeMotor": "kneerotor", "ThighMotor": "thighrotor"}
    moving_with = {
        "KneeMotorShaft": "kneerotor",
        "KneeMotorPulley": "kneerotor",
        "ThighMotorShaft": "thighrotor",
        "ThighMotorPulley": "thighrotor",
    }
    out = []
    for r in rows:
        key, side = base_name(r["name"]), r["name"].split("_")[0]
        if key in rotor_of:
            mr, iyy = rot["ro40_rotor_mass_kg"], rot["ro40_rotor_inertia_axis_kg_m2"]
            fraction = (r["mass_kg"] - mr) / r["mass_kg"]
            out.append(
                {
                    **r,
                    "link": f"{side}_{rotor_of[key]}",
                    "name": f"{r['name']}Rotor",
                    "basis": "catalog rotor",
                    "mass_kg": mr,
                    "inertia_kg_m2": np.diag([iyy / 2, iyy, iyy / 2]),
                }
            )
            r = {**r, "mass_kg": r["mass_kg"] - mr, "inertia_kg_m2": r["inertia_kg_m2"] * fraction}
        elif key in moving_with:
            r = {**r, "link": f"{side}_{moving_with[key]}"}
        elif key in rot["catalog_inertia_axis_kg_m2"]:
            target = rot["catalog_inertia_axis_kg_m2"][key]
            r = {**r, "inertia_kg_m2": r["inertia_kg_m2"] * target / r["inertia_kg_m2"][1, 1]}
        out.append(r)
    return out


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    state = pose(C["poses"]["nominal_hip_height"], C)
    parts = A.build(state)
    rows = []
    for link, name, shape, _ in parts:
        volume, centroid, inertia_unit = solids_props(shape)
        mass, basis = part_mass(name, volume)
        density = mass / (volume * MM3)  # kg/m3, uniform over the CAD solid
        inertia = inertia_unit * MM3 * density * MM2  # kg m2
        rows.append(
            {
                "link": link,
                "name": name,
                "basis": basis,
                "mass_kg": mass,
                "volume_mm3": volume,
                "com_mm": centroid,
                "inertia_kg_m2": inertia,
            }
        )

    rows = split_rotating(rows)

    def combine(selected):
        m = sum(r["mass_kg"] for r in selected)
        com = sum(r["mass_kg"] * r["com_mm"] for r in selected) / m
        inertia = np.zeros((3, 3))
        for r in selected:
            d = (r["com_mm"] - com) / 1000
            inertia += r["inertia_kg_m2"] + r["mass_kg"] * (
                np.dot(d, d) * np.eye(3) - np.outer(d, d)
            )
        return m, com, inertia

    bodies = {}
    rigid = {}
    for r in rows:
        rigid.setdefault(r["link"], []).append(r)
    for key, selected in sorted(rigid.items()):
        m, com, inertia = combine(selected)
        origin, angle = link_frame(key, state)
        com_l, inertia_l = to_link(com, inertia, origin, angle)
        bodies[key] = {
            "mass_kg": round(m, 5),
            "com_link_m": [round(v / 1000, 6) for v in com_l],
            "inertia_about_com_link_kg_m2": [[float(f"{v:.4e}") for v in row] for row in inertia_l],
            "frame": {
                "origin_hip_m": [round(v / 1000, 6) for v in origin],
                "angle_deg": round(math.degrees(angle), 3),
            },
        }

    # Knee drive train inertia about the hip axis (y through the origin).
    kd = [r for r in rows if r["link"].endswith("_kneedrive")]
    knee_drive = {}
    for side in ("left", "right"):
        sel = [r for r in kd if r["link"].startswith(side)]
        m, com, inertia = combine(sel)
        d = com / 1000
        iyy = inertia[1, 1] + m * (d[0] ** 2 + d[2] ** 2)
        knee_drive[side] = {"mass_kg": round(m, 5), "inertia_hip_axis_kg_m2": float(f"{iyy:.4e}")}

    total = sum(r["mass_kg"] for r in rows)
    allow = MP["allowances_kg"]
    allocated = total + sum(allow.values())
    targets = C["targets"]
    by_basis = {}
    for r in rows:
        by_basis[r["basis"]] = by_basis.get(r["basis"], 0.0) + r["mass_kg"]
    by_part = {}
    for r in rows:
        k = base_name(r["name"])
        by_part[k] = by_part.get(k, 0.0) + r["mass_kg"]
    record = {
        "basis": "cad/m10/assembly.py at the nominal pose; " + MP["status"],
        "pose": {k: state[k] for k in ("hip_height_m", "a1_deg", "a2_deg", "knee_m", "axle_m")},
        "cad_mass_kg": round(float(total), 4),
        "allowances_kg": allow,
        "allocated_kg": round(float(allocated), 4),
        "ceiling_kg": targets["allocated_mass_ceiling_kg"],
        "margin_kg": round(float(targets["allocated_mass_ceiling_kg"] - allocated), 4),
        "gate_pass": bool(allocated <= targets["allocated_mass_ceiling_kg"]),
        "robot_com_hip_m": [
            round(float(v) / 1000, 5) for v in sum(r["mass_kg"] * r["com_mm"] for r in rows) / total
        ],
        "rigid_bodies": bodies,
        "knee_drive_train": knee_drive,
        "mass_by_basis_kg": {k: round(float(v), 4) for k, v in sorted(by_basis.items())},
        "mass_by_part_kg": {
            k: round(v, 4) for k, v in sorted(by_part.items(), key=lambda kv: -kv[1])
        },
    }
    (OUT / "mass.json").write_text(json.dumps(record, indent=2) + "\n")
    print(
        "M10 MASS",
        json.dumps(
            {k: record[k] for k in ("cad_mass_kg", "allocated_kg", "margin_kg", "gate_pass")}
        ),
        flush=True,
    )
    return record


if __name__ in ("__main__", "bridge_main"):
    run()
