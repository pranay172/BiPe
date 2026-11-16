"""Balance core response to synthetic speed commands: variant kind amp seconds."""

import sys

import numpy as np

from bipe.m10.kinematics import posture
from bipe.m12 import speed as SP
from bipe.moving_experiment import simulate
from bipe.parameters import ROOT
from bipe.profiles import native
from bipe.robustness_control import build_candidate

variant, kind, amp, dur = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
p, gains, sensors, _ = SP.load()
lib = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
plant, config, seed = SP.variant_inputs(p, sensors, None if variant == "none" else variant)
q = posture(40, p)


def cmd(t, sc):
    v = 0.0
    if t >= 1.0 and kind.startswith("sine"):
        v = amp * np.sin(2 * np.pi * float(kind[4:]) * t)
    return v, 0.0, q, np.zeros(4)


run = simulate(
    "lowcmd",
    dur,
    sensing="sensors",
    seed=seed,
    nominal=p,
    plant=plant,
    gains=gains,
    sensor_config=config,
    implementation="c",
    firmware_library=lib,
    physics_dt=plant["simulation"]["physics_dt"],
    disturbance={},
    command_function=cmd,
)
print(variant, kind, amp, run["failure"], round(run["stats"]["peak_pitch_deg"], 1))
