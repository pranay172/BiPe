"""Log the slope observer against MuJoCo's true horizontal momentum in one robustness case."""
import sys, json, math, numpy as np, mujoco
from functools import partial
import bipe.sensing as sensing, bipe.locomotion as loco
from bipe.parameters import ROOT, load_parameters
from bipe.robustness import matrix, command, plant_variant
from bipe.moving_experiment import simulate
p = load_parameters(profile="m10")
case = {c["name"]: c for c in matrix(p)}[sys.argv[1]]
box = {}
orig_update = sensing.SensorEstimator.update
def update(self, data):
    box["data"], box["model"] = data, self.model if hasattr(self, "model") else box.get("model")
    return orig_update(self, data)
sensing.SensorEstimator.update = update
orig_init = sensing.SensorEstimator.__init__
def init(self, model, data, *a, **k):
    box["model"] = model; orig_init(self, model, data, *a, **k)
sensing.SensorEstimator.__init__ = init
rows = []
orig = loco.LocomotionController.slope_lean
def lean(self, s, center, axle, dt):
    out = orig(self, s, center, axle, dt)
    m, d = box["model"], box["data"]
    mujoco.mj_subtreeVel(m, d)
    total = m.body_subtreemass[1]
    pt = total * d.subtree_linvel[1][0]
    mass = sum(r["mass_kg"] for r in self.p["rigid_bodies"].values())
    rows.append([d.time, out, self.slope_force, self.slope_momentum, pt, total, mass, s["velocity"], self.slope_torque, center[1]-axle[1]])
    return out
loco.LocomotionController.slope_lean = lean
gains = json.loads((ROOT / "config/controller_m10.json").read_text())
sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
run = simulate(case["mode"], float(sys.argv[2]), sensing="sensors", nominal=p, plant=plant_variant(p, case.get("variation", {})),
               gains=gains, sensor_config=sensors, implementation="python", disturbance=case.get("disturbance", {}),
               command_function=partial(command, case=case, nominal=p))
r = np.array(rows)
np.save(sys.argv[3], r)
print("subtree mass", r[0,5], "config mass", r[0,6], "h", r[0,9])
for T in np.arange(0, r[-1,0], 0.25):
    k = np.argmin(abs(r[:,0]-T)); x = r[k]
    print(f"{x[0]:5.2f} lean {math.degrees(x[1]):6.2f} F {x[2]:7.3f} p_obs {x[3]:7.4f} p_true {x[4]:7.4f} Mv {x[6]*x[7]:7.4f} torque/r {x[8]/0.036:7.3f}")
