"""Plant-profile registry: the native CAD plants and the files each one uses.

The M4 articulated plant is the default and is not registered. A registered
profile names its plant and kinematics modules, its controller, sensor and
electrical-screen configs and its output folders; shared code dispatches on
``native(p)`` instead of comparing profile names.
"""

import importlib

PROFILES = {
    "m9": {
        "plant": "bipe.m9_plant",
        "kinematics": "bipe.m9_kinematics",
        "controller": "config/controller_m9.json",
        "sensors": "config/sensors-m9.json",
        "electrical": "config/m9-screen-assumptions.json",
        "experiments": "artifacts/milestone-9/experiments",
        "acceptance": "artifacts/milestone-9/acceptance",
        "moving": "artifacts/milestone-9/moving",
        "firmware_build": ".venv/m9-build/current",
        "runtime_sources": ["m9_plant", "m9_kinematics", "m9_impact", "protective_fall"],
    },
    "m10": {
        "plant": "bipe.m10.plant",
        "kinematics": "bipe.m10.kinematics",
        "controller": "config/controller_m10.json",
        "sensors": "config/sensors-m10.json",
        "electrical": "config/m10-screen-assumptions.json",
        "experiments": "artifacts/m10/experiments",
        "acceptance": "artifacts/m10/acceptance",
        "moving": "artifacts/m10/moving",
        "firmware_build": ".venv/m10-build/current",
        "runtime_sources": ["m10/plant", "m10/kinematics", "m9_impact", "protective_fall"],
        # User decisions (13 October 2026): 60 (72 mm hip height) is the lowest operating
        # posture, and the unpowered case passes when the robot does not stay standing,
        # by falling past 30 deg or coming to rest on its body.
        "low_posture_deg": 60,
        "unpowered_rest_on_body": True,
    },
    # M13 training plant: the M10 robot in primitive geometry for GPU training. It is run
    # with the M10 controller, sensors and gates; its results are training-plant results.
    "m13": {
        "plant": "bipe.m13.plant",
        "kinematics": "bipe.m10.kinematics",
        "controller": "config/controller_m10.json",
        "sensors": "config/sensors-m10.json",
        "electrical": "config/m10-screen-assumptions.json",
        "experiments": "artifacts/m13/experiments",
        "acceptance": "artifacts/m13/acceptance",
        "moving": "artifacts/m13/moving",
        "firmware_build": ".venv/m10-build/current",
        "runtime_sources": ["m13/plant", "m13/drives", "m10/kinematics"],
        "low_posture_deg": 60,
        "unpowered_rest_on_body": True,
    },
}
CHOICES = ["m4", *PROFILES]


def native(p):
    """Registry entry of a native CAD plant profile, or None for the M4 plant."""
    return PROFILES.get(p.get("profile")) if p is not None else None


def plant(p):
    entry = native(p)
    return importlib.import_module(entry["plant"]) if entry else None


def kinematics(p):
    entry = native(p)
    return importlib.import_module(entry["kinematics"]) if entry else None
