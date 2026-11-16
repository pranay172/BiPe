"""Adopt the M10 CAD results into config/robot.json m10.cad.

python -m bipe.m10.adopt_cad

Reads artifacts/m10/cad/mass.json (cad/m10/mass_properties.py) and
artifacts/m10/cad/sweep.json (cad/m10/sweep.py) and writes the rigid bodies,
knee drive train inertia, CAD mass and joint ranges to m10.cad. With m10.cad
present, bipe.m10.concept balances with CAD masses instead of the concept
allocation. Only the m10.cad block is rewritten, in json indent=2 form.
"""

import json

from bipe.parameters import ROOT

CONFIG = ROOT / "config/robot.json"
CAD = ROOT / "artifacts/m10/cad"


def block():
    mass = json.loads((CAD / "mass.json").read_text())
    sweep = json.loads((CAD / "sweep.json").read_text())
    return {
        "status": "Phase D4 CAD adoption (B3 assembly with the protection set): layout-study fidelity, not hardware-validated",
        "source": {
            "mass": "artifacts/m10/cad/mass.json (cad/m10/mass_properties.py)",
            "sweep": "artifacts/m10/cad/sweep.json (cad/m10/sweep.py)",
        },
        "frames": "Link frames: origin at the joint, x along the link (thigh hip->knee, shin knee->axle), y the joint axis; body and wheel frames keep hip-frame axes. Inertia about the centre of mass in kg m2.",
        "cad_mass_kg": mass["cad_mass_kg"],
        "allocated_kg": mass["allocated_kg"],
        "margin_to_ceiling_kg": mass["margin_kg"],
        "rigid_bodies": mass["rigid_bodies"],
        "knee_drive_train": {
            "treatment": "separate rigid bodies (rigid_bodies {side}_kneedrive, and {side}_thighrotor / {side}_kneerotor for the RO40 rotors); this entry repeats the knee drive train for reference",
            "per_side": mass["knee_drive_train"]["left"],
        },
        "joint_ranges_covered_deg": sweep["joint_ranges_covered_deg"],
        "sit_pose_hip_height_m": sweep["sit_pose_hip_height_m"],
        "sweep_grid": {
            "heights_m": sweep["heights_m"],
            "axle_offsets_m": sweep["axle_offsets_m"],
            "clean": sweep["grid_clean"],
        },
    }


def nested(value, indent):
    """value as json indent=2, re-indented to sit at the given depth of the config."""
    return json.dumps(value, indent=2).replace("\n", "\n" + " " * indent)


def main():
    text = CONFIG.read_text()
    config = json.loads(text)
    # Replace only the m10.cad block, so hand-formatted rows elsewhere keep their form.
    old = '"cad": ' + nested(config["m10"]["cad"], 4)
    if text.count(old) != 1:
        raise ValueError("m10.cad is not in its canonical form in config/robot.json")
    config["m10"]["cad"] = block()
    text = text.replace(old, '"cad": ' + nested(config["m10"]["cad"], 4))
    assert json.loads(text) == config
    CONFIG.write_text(text)
    print(
        json.dumps(
            {
                k: config["m10"]["cad"][k]
                for k in ("cad_mass_kg", "allocated_kg", "sit_pose_hip_height_m")
            }
        )
    )


if __name__ == "__main__":
    main()
