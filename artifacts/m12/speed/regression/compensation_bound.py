"""Bounding the estimator's drive-acceleration compensation: the M10 acceptance pushes
during a drive and the fast M12 speed cases, on the Python controller, with the
compensated acceleration unbounded or clipped to +/-bound (the low-passed odometric
acceleration itself is unchanged; only the part removed from the accelerometer is clipped).
compensation_bound.json was run on the driving-only estimator before the bound became the
estimator's compensation_limit input; with that input in place, "None" is no longer
unbounded.

Run from a source tree root: PYTHONPATH=. .venv/bin/python <this file> <output.json>
"""

import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import numpy as np

import bipe.m12.speed as S
import bipe.sensing
from bipe.moving_experiment import simulate
from bipe.robustness import command, matrix, physical_metrics, plant_variant

P, GAINS, SENSORS, CFG = S.load()
PUSHES = {c["name"]: c for c in matrix(P)}
SPEED = S.definitions(CFG)
NAMES = [
    "push_drive_+2",
    "push_drive_+2.5",
    "push_drive_-2.5",
    "drive_step",
    "replay_corner_90",
    "replay_start_stop",
    "replay_curve_r1",
]
BOUNDS = [None, 1.25, 0.75]


def bounded(bound):
    update = bipe.sensing.SensorEstimator.update

    def step(self, d):
        if not self.fusion_tau:
            return update(self, d)
        a = self.imu_acceleration
        clipped = float(np.clip(a, -bound, bound))
        self.imu_acceleration = clipped
        out = update(self, d)
        # Restore the unclipped filter state: a + k(x - a) = c + k(x - c) + (1 - k)(a - c).
        k = -math.expm1(-self.dt / self.fusion_tau)
        self.imu_acceleration += (1 - k) * (a - clipped)
        return out

    bipe.sensing.SensorEstimator.update = step


def python_simulate(*args, **kw):
    kw.update(implementation="python", firmware_library=None)
    return simulate(*args, **kw)


def job(arg):
    name, bound = arg
    if bound is not None:
        bounded(bound)
    if name in PUSHES:
        case = PUSHES[name]
        run = simulate(case["mode"], case["duration"], sensing="sensors", nominal=P,
                       plant=plant_variant(P, case.get("variation", {})), gains=GAINS,
                       sensor_config=SENSORS, implementation="python",
                       disturbance=case.get("disturbance", {}),
                       command_function=partial(command, case=case, nominal=P))
        m = physical_metrics(run)
        keys = ["physical_pass", "recovered", "peak_pitch_deg", "late_speed_peak_m_s"]
        return {"case": name, "bound": bound, **{k: m[k] for k in keys}}
    S.simulate = python_simulate
    run = S.run_case(P, GAINS, SENSORS, None, SPEED[name])
    m = S.speed_metrics(run, SPEED[name])
    keys = ["peak_pitch_deg", "peak_speed_m_s", "overshoot_m_s", "path_error_m"]
    return {"case": name, "bound": bound, "failure": run["failure"],
            **{k: m.get(k) for k in keys}}


if __name__ == "__main__":
    args = [(n, b) for n in NAMES for b in BOUNDS]
    # One job per process: the patch must not leak into later jobs.
    with ProcessPoolExecutor(8, max_tasks_per_child=1) as pool:
        rows = list(pool.map(job, args))
    for r in rows:
        print(json.dumps(r), flush=True)
    Path(sys.argv[1]).write_text(json.dumps({"controller": GAINS, "rows": rows}, indent=2))
