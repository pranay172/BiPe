"""The M10 acceptance drive, combined and push cases with the hub torque lag applied
through physics.hub_motor.FOC_time_constant_s (the constant the M10 drives read).

Run from a source tree root: PYTHONPATH=. .venv/bin/python <this file> <output.json>
"""

import copy
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from bipe.moving_experiment import metrics, simulate
from bipe.parameters import ROOT, load_parameters
from bipe.robustness_control import build_candidate

P = load_parameters(profile="m10")
SENSORS = json.loads((ROOT / "config/sensors-m10.json").read_text())
DURATION = {"drive": 20, "combined": 36, "push": 16}
KEYS = ["passed", "failure", "max_pitch_error_deg", "plateau_speed_mae_m_s",
        "max_stopping_travel_m", "peak_speed_m_s"]


def job(arg):
    scenario, lag, library = arg
    plant = copy.deepcopy(P)
    plant["physics"]["hub_motor"]["FOC_time_constant_s"] = lag
    run = simulate(scenario, DURATION[scenario], sensing="sensors", nominal=P, plant=plant,
                   sensor_config=SENSORS, implementation="c", firmware_library=library)
    m = metrics(run)
    return {"scenario": scenario, "hub_lag_s": lag, **{k: m.get(k) for k in KEYS}}


if __name__ == "__main__":
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    library = str(build_candidate(P, gains, ROOT / ".venv/m10-build/current"))
    args = [(s, lag, library) for s in DURATION for lag in (0.025, 0.01)]
    with ProcessPoolExecutor(6) as pool:
        rows = list(pool.map(job, args))
    for r in rows:
        print(json.dumps(r), flush=True)
    Path(sys.argv[1]).write_text(json.dumps({"controller": gains, "rows": rows}, indent=2))
