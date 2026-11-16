"""Attribute the higher peak pitch of the M10 acceptance slope and combined cases after
phase B, on the Python controller: the phase B controller, without its lean feedforward,
without the estimator acceleration compensation, and the phase A controller (77e764c)
without the compensation (the pre-phase-B stack). Records each peak and when it occurs.

Run from a source tree root: PYTHONPATH=. .venv/bin/python <this file> <output.json>
"""

import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import numpy as np

import bipe.sensing
from bipe.moving_experiment import COLUMNS, simulate
from bipe.parameters import ROOT, load_parameters
from bipe.robustness import command, matrix, physical_metrics, plant_variant

P = load_parameters(profile="m10")
SENSORS = json.loads((ROOT / "config/sensors-m10.json").read_text())
GAINS = json.loads((ROOT / "config/controller_m10.json").read_text())
PHASE_A = json.loads(
    subprocess.check_output(["git", "show", "77e764c:config/controller_m10.json"], cwd=ROOT)
)
CASES = {c["name"]: c for c in matrix(P)}
NAMES = ["slope_up_4", "slope_up_10", "slope_hold_10", "mismatch_body_mass_scale_1.15_combined"]
VARIANTS = {
    "phase_b": (GAINS, True),
    "no_feedforward": ({**GAINS, "acceleration_feedforward": 0.0}, True),
    "no_compensation": (GAINS, False),
    "phase_a_controller_no_compensation": (PHASE_A, False),
}


def job(arg):
    name, variant = arg
    gains, compensation = VARIANTS[variant]
    if not compensation:
        update = bipe.sensing.SensorEstimator.update

        def uncompensated(self, d):
            self.imu_acceleration = 0.0
            return update(self, d)

        bipe.sensing.SensorEstimator.update = uncompensated
    case = CASES[name]
    run = simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=P,
        plant=plant_variant(P, case.get("variation", {})),
        gains=gains,
        sensor_config=SENSORS,
        implementation="python",
        disturbance=case.get("disturbance", {}),
        command_function=partial(command, case=case, nominal=P),
    )
    m = physical_metrics(run)
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    pitch = np.abs(f["pitch_deg"]) if "pitch_deg" in f else None
    peak_time = float(f["time"][int(np.argmax(pitch))]) if pitch is not None else None
    keys = ["physical_pass", "recovered", "failure", "peak_pitch_deg", "rollback_m"]
    return {"case": name, "variant": variant, "peak_time_s": peak_time,
            **{k: m.get(k) for k in keys}}


if __name__ == "__main__":
    args = [(n, v) for n in NAMES for v in VARIANTS]
    # One job per process: the uncompensated patch must not leak into later jobs.
    with ProcessPoolExecutor(8, max_tasks_per_child=1) as pool:
        rows = list(pool.map(job, args))
    for r in rows:
        print(json.dumps(r), flush=True)
    Path(sys.argv[1]).write_text(
        json.dumps({"variants": {k: v[0] for k, v in VARIANTS.items()}, "rows": rows}, indent=2)
    )
