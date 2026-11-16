import copy
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from functools import partial

import numpy as np

from bipe.m10.kinematics import posture
from bipe.moving_experiment import COLUMNS, metrics, simulate
from bipe.parameters import ROOT, load_parameters

P = load_parameters(profile="m10")
P["sizing"]["drive_speed"] = 1.5
BASE = json.loads((ROOT / "config/controller_m10.json").read_text())

def command(t, scenario, case):
    v = w = 0.0
    if case == "step":
        v = 1.2 if 2 <= t < 8 else 0.0
    elif case == "reverse":
        v = -0.3 if 2 <= t < 6 else 0.0
    return v, w, posture(40, P), np.zeros(4)

def job(arg):
    tag, over, case = arg
    g = copy.deepcopy(BASE); over = dict(over); sensing = over.pop("_sensing", "sensors"); g.update(over)
    t0 = time.time()
    if case == "drive":
        run = simulate("drive", 20, sensing="sensors", nominal=P, plant=copy.deepcopy(P), gains=g)
        m = metrics(run)
        return tag, case, {k: m.get(k) for k in ["passed", "plateau_speed_mae_m_s", "max_stopping_travel_m", "max_pitch_error_deg", "peak_speed_m_s", "final_hold_pitch_error_deg"]}
    run = simulate(case, 12, sensing=sensing, nominal=P, plant=copy.deepcopy(P), gains=g,
                   command_function=partial(command, case=case))
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    t = f["time"]; xy = run["trace"][:, 1:3]
    i8 = np.searchsorted(t, 8 if case == "step" else 6)
    travel = np.linalg.norm(xy[i8:] - xy[i8], axis=1)
    plateau = (t > 5) & (t < 8)
    out = {"failure": run["failure"], "peak_v": float(np.max(np.abs(f["velocity"]))),
           "peak_pitch": float(np.max(np.abs(f["pitch_error_deg"]))),
           "plateau_mae": float(np.mean(np.abs(f["velocity"][plateau] - f["user_velocity"][plateau]))) if case == "step" else None,
           "stop_travel": float(travel.max()), "final_back": float(travel.max() - travel[-1]),
           "final_v": float(f["velocity"][-1]), "wheel_sat": float(np.mean(f["wheel_saturated"])),
           "peak_wheel": float(np.max(np.abs(f["left_applied_Nm"]))), "wall": round(time.time() - t0)}
    np.save(f"{sys.argv[1]}/{tag}_{case}.npy", run["trace"])
    return tag, case, out

if __name__ == "__main__":
    variants = json.loads(sys.argv[2])
    cases = sys.argv[3].split(",")
    jobs = [(tag, over, c) for tag, over in variants.items() for c in cases]
    with ProcessPoolExecutor(min(8, len(jobs))) as pool:
        for tag, case, r in pool.map(job, jobs):
            print(tag, case, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}), flush=True)
