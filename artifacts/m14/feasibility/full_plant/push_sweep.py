"""Full-plant push sweep: hold, push +-size, at nominal plant and two stress corners."""
import copy, json, multiprocessing, sys
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path
import bipe.m14.stress as S
from bipe.moving_experiment import simulate as _sim
from bipe.parameters import ROOT, load_parameters
from bipe.profiles import native
from bipe.robustness import command, physical_metrics
from bipe.robustness_control import build_candidate

def run(payload):
    p, gains, sensors, library, case = payload
    if case["corner"] is None:
        plant, config = copy.deepcopy(p), copy.deepcopy(sensors)
    else:
        plant, config = S.stressed(p, sensors, case["corner"])
    result = S.simulate(case["mode"], case["duration"], sensing="sensors", nominal=p, plant=plant,
        gains=gains, sensor_config=config, implementation="c", firmware_library=library,
        disturbance=case["disturbance"], command_function=partial(command, case=case, nominal=p))
    m = physical_metrics(result)
    return case["name"], {k: m[k] for k in ("physical_pass", "recovered", "peak_pitch_deg", "failure") if k in m}

def main():
    ckpt = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "classical" else None
    out = Path(sys.argv[2])
    p = load_parameters(profile="m10"); prof = native(p)
    gains = json.loads((ROOT / prof["controller"]).read_text())
    sensors = json.loads((ROOT / prof["sensors"]).read_text())
    library = str(build_candidate(p, gains, ROOT / prof["firmware_build"]))
    corners = S.corners()
    cases = []
    for cname, c in (("nom", None), ("c0", corners[0]), ("c1", corners[1])):
        for size in (1.5, 2.5, 3.0, 3.75):
            for sign in (1, -1):
                d = {"push_Ns": sign * size, "initial_posture_deg": 40}
                if c: d.update({k: c[k] for k in S.RANGES if "belt" in k or "backlash" in k})
                cases.append({"name": f"{cname}_{'fwd' if sign > 0 else 'back'}_{size}", "family": "push",
                              "mode": "hold", "duration": 16, "corner": c, "disturbance": d})
    payloads = [(p, gains, sensors, library, c) for c in cases]
    multiprocessing.set_start_method("fork", force=True)
    def ex():
        with ProcessPoolExecutor(4) as pool:
            return dict(pool.map(run, payloads))
    if ckpt:
        from bipe.m13.policy_control import policy_in_loop
        with policy_in_loop(ckpt, runners=("bipe.m14.stress",)):
            res = ex()
    else:
        res = ex()
    out.write_text(json.dumps(res, indent=1, default=float))
    for k, v in res.items(): print(k, v.get("physical_pass"), v.get("recovered"))

if __name__ == "__main__":
    main()
