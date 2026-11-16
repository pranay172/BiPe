"""Shared SI parameters and geometry; FreeCAD performs its own mm conversion."""

import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_parameters(path=None, profile="m4"):
    p = json.loads(
        Path(path or os.environ.get("BIPE_ROBOT_CONFIG") or ROOT / "config/robot.json").read_text()
    )
    if p["units"] != "SI":
        raise ValueError("Robot parameters must use SI units")
    from .profiles import PROFILES, plant

    if profile in PROFILES:
        return plant({"profile": profile}).parameters(p)
    if profile != "m4":
        raise ValueError("Unknown plant profile")
    g = p["geometry"]
    if any(g[k] <= 0 for k in ["link_length", "wheel_radius", "wheel_track", "body_height"]):
        raise ValueError("Dimensions must be positive")
    if g["wheel_track"] - g["wheel_width"] <= g["body_width"]:
        raise ValueError("Wheels overlap the body in the lateral envelope")
    if any(row["mass_each"] <= 0 or row["count"] < 1 for row in p["mass_budget"]):
        raise ValueError("Component masses and counts must be positive")
    validate_actuator_limits(p)
    return p


def validate_actuator_limits(p):
    for block, peak, continuous in [
        ("motor", "peak_torque_per_wheel", "continuous_torque_per_wheel"),
        ("articulation", "peak_torque_per_joint", "continuous_torque_reference"),
    ]:
        if block not in p:
            continue
        a, b = p[block][peak], p[block][continuous]
        if not (math.isfinite(a) and math.isfinite(b) and 0 < b <= a):
            raise ValueError(
                f"{block}: peak torque must be finite and >= positive continuous torque"
            )


def leg_geometry(p, angle_deg=None):
    """Coordinates relative to the wheel axle plane, with hip and axle aligned in X."""
    g = p["geometry"]
    angle = math.radians(g["fixed_leg_angle_deg"] if angle_deg is None else angle_deg)
    l = g["link_length"]
    return {
        "hip_z": 2 * l * math.cos(angle),
        "knee_x": (-1 if "rigid_bodies" in p else 1) * l * math.sin(angle),
        "knee_z": l * math.cos(angle),
    }


def mass_at(p, location):
    return sum(
        row["count"] * row["mass_each"] for row in p["mass_budget"] if row["location"] == location
    )


def total_mass(p):
    return sum(row["count"] * row["mass_each"] for row in p["mass_budget"])
