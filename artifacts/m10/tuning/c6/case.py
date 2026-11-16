"""python case.py case_name duration impl [gains-json] -> runs one robustness case, saves trace."""
import sys, json, numpy as np
from functools import partial
from bipe.parameters import ROOT, load_parameters
from bipe.robustness import matrix, command, plant_variant, physical_metrics
from bipe.moving_experiment import simulate, COLUMNS
p = load_parameters(profile="m10")
case = {c["name"]: c for c in matrix(p)}[sys.argv[1]]
dur = float(sys.argv[2]); impl = sys.argv[3]
gains = json.loads((ROOT / "config/controller_m10.json").read_text()) | (json.loads(sys.argv[4]) if len(sys.argv) > 4 else {})
sensors = json.loads((ROOT / "config/sensors-m10.json").read_text()) | json.loads(__import__("os").environ.get("SENS","{}"))
run = simulate(case["mode"], dur, sensing=__import__("os").environ.get("SENSING","sensors"), nominal=p, plant=plant_variant(p, case.get("variation", {})), gains=gains,
               sensor_config=sensors, implementation=impl, disturbance=case.get("disturbance", {}) | json.loads(__import__("os").environ.get("DIST","{}")),
               command_function=partial(command, case=case, nominal=p))
m = physical_metrics(run)
print(json.dumps({k: m[k] for k in ("physical_pass", "recovered", "failure", "peak_pitch_deg", "late_pitch_peak_deg", "late_speed_peak_m_s", "rollback_m")}, default=float))
np.save(f"case_{sys.argv[1]}.npy", run["trace"])
