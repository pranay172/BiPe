"""Generate the learned controller's firmware constants from the M13 export manifest.

.venv/bin/python tools/generate_locomotion_config.py

Writes firmware/include/locomotion_config.h: the action law, policy timing, command
limits and posture table from the deployed policy's export manifest (bipe.m13.headers
DEPLOYED; artifacts/m14/export/locomotion since M14), and the output
guards' limits (joint limits and the hub's no-load speed) from the M10 parameters. A test
checks that the committed header matches a fresh generation.
"""

import itertools
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bipe.m10.kinematics import posture
from bipe.parameters import load_parameters

HEADER = ROOT / "firmware/include/locomotion_config.h"
from bipe.m13.headers import DEPLOYED, POLICIES  # noqa: E402

MANIFEST = ROOT / POLICIES[DEPLOYED]["export"] / "manifest.json"


def number(x):
    return repr(float(x))


def header(manifest, p):
    legs, wheels = manifest["action"]["legs"], manifest["action"]["wheels"]
    timing, command = manifest["timing"], manifest["command"]
    table = manifest["posture_table"]
    labels = table["labels"]
    steps = {round(b - a, 9) for a, b in itertools.pairwise(labels)}
    assert len(steps) == 1, "the firmware interpolates an evenly spaced posture table"
    lo, hi = command["trained_range"]["posture_label"]
    assert (lo, hi) == (labels[0], labels[-1])
    joints = p["articulation"]
    # Full precision from the M10 kinematics the manifest rounds to 9 digits, so the
    # firmware's targets are the simulation's to the last bit.
    nominal = [posture(float(x), p) for x in labels]
    assert all(
        abs(a - b) < 1e-8
        for row, saved in zip(nominal, table["nominal_rad"])
        for a, b in zip(row, saved)
    ), "the manifest's posture table is not the M10 kinematics'"
    rows = ",\n".join("    {" + ", ".join(number(q) for q in row) + "}" for row in nominal)
    yaw_rate = command["rate_limit_per_s"]["yaw_rate_rad_s"]
    identity = manifest["identity"]
    lines = [
        "/* Generated: tools/generate_locomotion_config.py; do not edit. */",
        "#ifndef BIPE_LOCOMOTION_CONFIG_H",
        "#define BIPE_LOCOMOTION_CONFIG_H",
        f"/* {identity['variant']} policy, {identity['checkpoint']}: the forward pass must",
        "   report this checkpoint (bipe_locomotion_checkpoint) to be selectable. */",
        f'#define BIPE_LEARNED_CHECKPOINT_SHA256 "{identity["checkpoint_sha256"]}"',
        "/* Action law (manifest action). */",
        f"#define BIPE_LEARNED_LEG_SCALE {number(legs['leg_scale_rad'])}",
        f"#define BIPE_LEARNED_JOINT_KP {number(legs['servo']['kp_Nm_per_rad'])}",
        f"#define BIPE_LEARNED_JOINT_KD {number(legs['servo']['kd_Nm_s_per_rad'])}",
        "/* Leg target filter (0: off); overridable so make test also checks it on. */",
        "#ifndef BIPE_LEARNED_LEG_FILTER_TAU_S",
        f"#define BIPE_LEARNED_LEG_FILTER_TAU_S {number(legs['target_filter']['tau_s'])}",
        "#endif",
        "#ifndef BIPE_LEARNED_LEG_FILTER_BAND_RAD",
        f"#define BIPE_LEARNED_LEG_FILTER_BAND_RAD {number(legs['target_filter']['band_rad'])}",
        "#endif",
        f"#define BIPE_LEARNED_WHEEL_SCALE {number(wheels['wheel_scale_rad_s'])}",
        f"#define BIPE_LEARNED_WHEEL_RADIUS {number(wheels['wheel_radius_m'])}",
        f"#define BIPE_LEARNED_WHEEL_TRACK {number(wheels['wheel_track_m'])}",
        f"#define BIPE_LEARNED_WHEEL_KV {number(wheels['speed_loop']['kv_Nm_s_per_rad'])}",
        f"#define BIPE_LEARNED_WHEEL_KI {number(wheels['speed_loop']['ki_Nm_per_rad'])}",
        f"#define BIPE_LEARNED_WHEEL_PEAK {number(wheels['speed_loop']['peak_torque_Nm'])}",
        "/* Timing: the action is held for POLICY_TICKS control ticks. */",
        f"#define BIPE_LEARNED_TICK_S {number(timing['control_tick_s'])}",
        f"#define BIPE_LEARNED_POLICY_TICKS {timing['policy_every_ticks']}",
        "/* Command rate limits per second (0: unlimited) and trained ranges. */",
        f"#define BIPE_LEARNED_SPEED_RATE {number(command['rate_limit_per_s']['speed_m_s'])}",
        f"#define BIPE_LEARNED_YAW_RATE_RATE {number(yaw_rate or 0)}",
        f"#define BIPE_LEARNED_LABEL_RATE {number(command['rate_limit_per_s']['posture_label'])}",
        "#define BIPE_LEARNED_SPEED_RANGE {"
        + ", ".join(number(x) for x in command["trained_range"]["speed_m_s"])
        + "}",
        "#define BIPE_LEARNED_YAW_RATE_RANGE {"
        + ", ".join(number(x) for x in command["trained_range"]["yaw_rate_rad_s"])
        + "}",
        "/* Posture table: nominal [hip, knee] x [left, right] at evenly spaced labels. */",
        f"#define BIPE_LEARNED_POSTURES {len(labels)}",
        f"#define BIPE_LEARNED_LABEL_FIRST {number(labels[0])}",
        f"#define BIPE_LEARNED_LABEL_STEP {number(steps.pop())}",
        f"#define BIPE_LEARNED_LABEL_CENTRE {number((lo + hi) / 2)}",
        f"#define BIPE_LEARNED_LABEL_HALF_SPAN {number((hi - lo) / 2)}",
        "#define BIPE_LEARNED_NOMINAL { \\",
        rows.replace("\n", " \\\n") + " \\",
        "}",
        "/* Output guards (M10 parameters): joint limits and hub speed. */",
        f"#define BIPE_LEARNED_HIP_MIN {number(math.radians(joints['hip_limits_deg'][0]))}",
        f"#define BIPE_LEARNED_HIP_MAX {number(math.radians(joints['hip_limits_deg'][1]))}",
        f"#define BIPE_LEARNED_KNEE_MIN {number(math.radians(joints['knee_limits_deg'][0]))}",
        f"#define BIPE_LEARNED_KNEE_MAX {number(math.radians(joints['knee_limits_deg'][1]))}",
        f"#define BIPE_LEARNED_HUB_SPEED_MAX {number(p['motor']['no_load_speed'])}",
        f"/* Inputs: {MANIFEST.relative_to(ROOT)}, config/robot.json (m10).",
        "   tests/test_m13_handover.py regenerates. */",
        "#endif",
    ]
    return "\n".join(lines) + "\n"


def main():
    manifest = json.loads(MANIFEST.read_text())
    HEADER.write_text(header(manifest, load_parameters(profile="m10")))
    print("LOCOMOTION CONFIG", HEADER.relative_to(ROOT))


if __name__ == "__main__":
    main()
