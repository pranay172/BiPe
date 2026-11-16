import sys, json, copy
from bipe.parameters import ROOT, load_parameters
from bipe.moving_validate import jobs
from bipe.moving_experiment import simulate, metrics
p = load_parameters(profile="m10")
sensors = json.loads((ROOT / "config/sensors-m10.json").read_text()) | json.loads(sys.argv[2])
gains = json.loads((ROOT / "config/controller_m10.json").read_text()) | json.loads(sys.argv[3])
job = {j["name"]: j for j in jobs(sensors, "python", p)}[sys.argv[1]]
kw = copy.deepcopy(job); kw.pop("name")
kw.update(nominal=p, gains=gains)
if "sensor_config" in kw: kw["sensor_config"] = kw["sensor_config"] | json.loads(sys.argv[2])
run = simulate(**kw); r = metrics(run)
import os, numpy as np
if os.environ.get('TRACE'): np.save(os.environ['TRACE'], run['trace'])
print(sys.argv[1], json.dumps({k: r.get(k) for k in ("settling_time_s", "max_travel_m", "warnings", "passed", "failure", "max_pitch_error_deg", "plateau_speed_mae_m_s", "final_velocity_m_s", "max_joint_error_deg", "longest_saturation_s", "final_heading_error_deg")}, default=float))
