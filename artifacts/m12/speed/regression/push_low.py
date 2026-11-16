"""The M10 acceptance low-posture pushes on the Python controller, with the phase B
estimator acceleration compensation on and off (off: the compensation term is zeroed
each update), to attribute a change in the acceptance push_low cases.

Records: push_low.json is the run with the first phase B compensation (odometric
acceleration at all times); push_low_agreed.json limited it to the acceleration both
commanded and observed (restores the pushes, but costs the drive overshoot, so it was not
kept); push_low_driving.json applies it only while driving, with the pushes during a
drive added; push_low_bounded.json is the adopted rule, applied only while driving and
clipped to the drive acceleration limit (the compensation_limit the controller sets).

Run from a source tree root: PYTHONPATH=. .venv/bin/python <this file> <output.json>
"""

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import bipe.sensing
from bipe.moving_experiment import simulate
from bipe.parameters import ROOT, load_parameters
from bipe.robustness import command, matrix, physical_metrics, plant_variant

P = load_parameters(profile="m10")
SENSORS = json.loads((ROOT / "config/sensors-m10.json").read_text())
GAINS = json.loads((ROOT / "config/controller_m10.json").read_text())
CASES = {c["name"]: c for c in matrix(P)}
NAMES = [
    "push_low_+2.5",
    "push_low_+2",
    "push_low_-2.5",
    "push_hold_+2.5",
    "push_drive_+2.5",
    "push_drive_-2.5",
    "push_drive_+2",
]
KEYS = ["physical_pass", "recovered", "failure", "peak_pitch_deg", "late_speed_peak_m_s"]


def job(arg):
    name, compensation = arg
    if not compensation:
        update = bipe.sensing.SensorEstimator.update

        def uncompensated(self, d):
            self.imu_acceleration = 0.0
            return update(self, d)

        bipe.sensing.SensorEstimator.update = uncompensated
    m = physical_metrics(python_case(CASES[name]))
    return {"case": name, "compensation": compensation, **{k: m.get(k) for k in KEYS}}


def python_case(case):
    """As bipe.robustness.run_case, on the Python controller."""
    return simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=P,
        plant=plant_variant(P, case.get("variation", {})),
        gains=GAINS,
        sensor_config=SENSORS,
        implementation="python",
        disturbance=case.get("disturbance", {}),
        command_function=partial(command, case=case, nominal=P),
    )


if __name__ == "__main__":
    args = [(n, co) for n in NAMES for co in (True, False)]
    # One job per process: the uncompensated patch must not leak into later jobs.
    with ProcessPoolExecutor(7, max_tasks_per_child=1) as pool:
        rows = list(pool.map(job, args))
    for r in rows:
        print(json.dumps(r), flush=True)
    Path(sys.argv[1]).write_text(json.dumps({"controller": GAINS, "rows": rows}, indent=2))
