"""python exp.py scenario duration sensing '{"sensors": {...}, "gains": {...}}' -> metrics JSON line."""
import sys, json, time, copy
from bipe.parameters import ROOT
from bipe.m10 import plant as P
from bipe.moving_experiment import simulate, metrics
shared = json.loads((ROOT / "config/robot.json").read_text())
p = P.parameters(shared)
scen, dur, sens = sys.argv[1], float(sys.argv[2]), sys.argv[3]
over = json.loads(sys.argv[4]) if len(sys.argv) > 4 else {}
sensors = json.loads((ROOT / "config/sensors-m10.json").read_text()) | over.get("sensors", {})
gains = json.loads((ROOT / "config/controller_m10.json").read_text()) | over.get("gains", {})
t = time.time()
run = simulate(scen, dur, over.get("offset", {"tilt_positive": 5, "tilt_negative": -5}.get(scen, 0)), sensing=sens, nominal=p, implementation="python", sensor_config=sensors, gains=gains, disturbance=over.get("disturbance"))
r = metrics(run)
keys = ("failure", "passed", "max_pitch_error_deg", "max_travel_m", "max_joint_error_deg", "settling_time_s",
        "final_velocity_m_s", "plateau_speed_mae_m_s", "height_range_m", "longest_saturation_s")
out = {k: r.get(k) for k in keys} | {"peak": r.get("peak_torque_Nm"), "wall_s": round(time.time() - t, 1), "label": over.get("label", "")}
print(json.dumps(out, default=float))
if len(sys.argv) > 5:
    import numpy as np
    np.save(sys.argv[5], run["trace"])
