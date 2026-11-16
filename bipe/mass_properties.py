"""Declared CAD component masses and rigid-body inertia, always in SI."""

import argparse
import hashlib
import json

import numpy as np

from .parameters import ROOT


def combine(rows):
    mass = sum(r["mass_kg"] for r in rows)
    center = sum(r["mass_kg"] * np.asarray(r["com_m"]) for r in rows) / mass
    tensor = np.zeros((3, 3))
    for row in rows:
        offset = np.asarray(row["com_m"]) - center
        tensor += np.asarray(row["inertia_kg_m2"]) + row["mass_kg"] * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )
    return {"mass_kg": mass, "com_m": center.tolist(), "inertia_kg_m2": tensor.tolist()}


def add_inertial(body, row):
    import xml.etree.ElementTree as ET

    from .model import values

    tensor = np.asarray(row["inertia_kg_m2"])
    if row["mass_kg"] <= 0 or np.linalg.eigvalsh(tensor).min() <= 0:
        raise ValueError("Rigid body requires positive mass and inertia")
    ET.SubElement(
        body,
        "inertial",
        pos=values(row["com_m"]),
        mass=str(row["mass_kg"]),
        fullinertia=values(
            [tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]]
        ),
    )


def body_transform(p, name, q):
    """Rigid frames in the chassis frame; wheel spin has a centered COM."""
    g = p["geometry"]
    length = g["link_length"]
    if name == "chassis":
        return np.zeros(3), np.eye(3)
    side, role = name.split("_")
    sign = 1 if side == "left" else -1
    pair = q[:2] if side == "left" else q[2:]
    a, b = pair

    def rotation(angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    hip = np.array([0, sign * g["hip_half_spacing"], 0])
    knee = hip + rotation(a) @ np.array([0, 0, -length])
    if role == "upper":
        return hip, rotation(a)
    if role == "lower":
        return knee, rotation(a + b)
    axle = knee + rotation(a + b) @ np.array([0, 0, -length])
    axle[1] = sign * g["wheel_track"] / 2
    return axle, np.eye(3)


def assembled_rows(p, q, include_wheels=True):
    result = []
    for name, row in p["rigid_bodies"].items():
        if not include_wheels and name.endswith("_wheel"):
            continue
        offset, rot = body_transform(p, name, q)
        result.append(
            {
                "mass_kg": row["mass_kg"],
                "com_m": (offset + rot @ row["com_m"]).tolist(),
                "inertia_kg_m2": (rot @ row["inertia_kg_m2"] @ rot.T).tolist(),
            }
        )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", default="artifacts/milestone-3/cad/component-mass-properties.json"
    )
    args = parser.parse_args()
    source = ROOT / args.source
    records = json.loads(source.read_text())["components"]
    path = ROOT / "config/robot.json"
    p = json.loads(path.read_text())
    p["rigid_bodies"] = {name: combine(rows) for name, rows in records.items()}
    budget = {}
    for rows in records.values():
        for row in rows:
            budget[row["location"]] = budget.get(row["location"], 0) + row["mass_kg"]
    p["mass_budget"] = [
        {"name": "cad_" + loc, "location": loc, "count": 1, "mass_each": mass}
        for loc, mass in budget.items()
    ]
    p["mass_properties_source"] = {
        "path": args.source,
        "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "basis": "CAD solids with declared uniform density or component mass. Unknown actuator internal mass distribution is approximated; rotor inertia is a separate armature.",
    }
    path.write_text(json.dumps(p, indent=2) + "\n")
    print(
        json.dumps(
            {
                "total_mass_kg": sum(r["mass_kg"] for r in p["rigid_bodies"].values()),
                "bodies": len(records),
            }
        )
    )


if __name__ == "__main__":
    main()
