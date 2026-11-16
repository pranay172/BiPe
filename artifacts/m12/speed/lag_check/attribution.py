"""Which phase B change costs margin under a 25 ms hub torque lag: the Python controller
in the M10 acceptance cases, with the phase B shaping on or off and the estimator's
acceleration compensation on or off (off: the compensation term is zeroed each update).

attribution.json is the record of this check on the phase B controller before the hold
reference kept its plain rate limit (the gains are saved with the rows); the fix followed
from it. Run from a source tree root: PYTHONPATH=. .venv/bin/python <this file> <output.json>
"""

import copy
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import bipe.sensing
from bipe.moving_experiment import metrics, simulate
from bipe.parameters import ROOT, load_parameters

P = load_parameters(profile="m10")
SENSORS = json.loads((ROOT / "config/sensors-m10.json").read_text())
GAINS = json.loads((ROOT / "config/controller_m10.json").read_text())
DURATION = {"drive": 20, "combined": 36, "push": 16}
KEYS = ["passed", "failure", "max_pitch_error_deg", "peak_speed_m_s"]
LAG = 0.025


def job(arg):
    scenario, shaping, compensation = arg
    if not compensation:
        update = bipe.sensing.SensorEstimator.update

        def uncompensated(self, d):
            self.imu_acceleration = 0.0
            return update(self, d)

        bipe.sensing.SensorEstimator.update = uncompensated
    gains = dict(GAINS)
    if not shaping:
        gains.update(command_jerk_m_s3=0, acceleration_feedforward=0, max_command_acceleration=3)
    plant = copy.deepcopy(P)
    plant["physics"]["hub_motor"]["FOC_time_constant_s"] = LAG
    run = simulate(scenario, DURATION[scenario], sensing="sensors", nominal=P, plant=plant,
                   sensor_config=SENSORS, implementation="python", gains=gains)
    m = metrics(run)
    return {"scenario": scenario, "hub_lag_s": LAG, "shaping": shaping,
            "compensation": compensation, **{k: m.get(k) for k in KEYS}}


if __name__ == "__main__":
    args = [(s, sh, co) for s in DURATION for sh, co in ((True, True), (True, False), (False, False))]
    # One job per process: the uncompensated patch must not leak into later jobs.
    with ProcessPoolExecutor(5, max_tasks_per_child=1) as pool:
        rows = list(pool.map(job, args))
    for r in rows:
        print(json.dumps(r), flush=True)
    Path(sys.argv[1]).write_text(json.dumps({"controller": GAINS, "rows": rows}, indent=2))
