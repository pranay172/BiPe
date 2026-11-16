"""Run one acceptance case with the handover runtime and save the full 1 kHz trace."""
import argparse, json, sys, copy
from functools import partial
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(Path(__file__).parent))
from bipe import robustness as R, firmware_control as fc
from bipe.parameters import load_parameters
from bipe.robustness_control import build_candidate
from bipe.moving_validate import jobs
from bipe.m13 import handover as H
from bipe.m13 import policy_control as pc

def run(case_name, variant, fault_s, **extra):
    p = load_parameters(profile="m10")
    prof = R.native(p)
    sensors = json.loads((ROOT / prof["sensors"]).read_text())
    gains = json.loads((ROOT / prof["controller"]).read_text())
    library = build_candidate(p, gains, ROOT / ".venv/m8-build/current")
    job = next(j for j in jobs(sensors, "c", p) if j["name"] == case_name)
    import os
    if os.environ.get("LASH0"):
        job["plant"]["physics"]["belts"]["hip_lash_deg"]=0; job["plant"]["physics"]["belts"]["knee_lash_deg"]=0
    if os.environ.get("NODELAY"):
        job["sensor_config"]["transport_delay_s"]=0.0
    case = {"name": case_name, "family": "acceptance", "job": job}
    actor = pc.CActor() if variant != 'classical' else None
    lib = H.build()
    opts = {} if variant=='classical' else dict(H.VARIANTS[variant], fault_s=fault_s if variant.startswith("fault") else None)
    opts.update(extra)
    import variants as V
    if variant == 'classical':
        r = R.run_case(p, gains, sensors, str(library), case); r['handover_s']=None; return r, R.physical_metrics(r)
    fc.FirmwareController = partial(V.Smooth, actor=actor, runtime=lib, **opts)
    fc.FirmwareEstimator = pc.PolicyEstimator
    r = R.run_case(p, gains, sensors, str(library), case)
    r["handover_s"] = getattr(pc.PolicyController.latest, "handover_s", None)
    return r, R.physical_metrics(r)

import variants as V
if __name__ == "__main__":
    a = argparse.ArgumentParser()
    a.add_argument("case"); a.add_argument("--variant", default="fault")
    a.add_argument("--fault-s", type=float, default=5.0)
    a.add_argument("--out"); a.add_argument("--seed", action="store_true"); a.add_argument("--zero", action="store_true"); a.add_argument("--bumpless", action="store_true"); a.add_argument("--stop", action="store_true"); a.add_argument("--ramp", type=float, default=0)
    args = a.parse_args()
    r, m = run(args.case, args.variant, args.fault_s, seed=args.seed, ramp_s=args.ramp, zero=args.zero, bumpless=args.bumpless, stop=args.stop)
    print(json.dumps({k: m.get(k) for k in ("physical_pass", "recovered", "peak_pitch_deg", "failure")}), "handover", r["handover_s"])
    if args.out:
        np.savez_compressed(args.out, log=np.array(V.Smooth.log) if V.Smooth.log else np.zeros(0), trace=r["trace"], columns=np.array(R.COLUMNS if hasattr(R,'COLUMNS') else []))
