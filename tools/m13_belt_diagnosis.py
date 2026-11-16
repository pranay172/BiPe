"""M13 belt range follow-up: the settle policy on the full plant, one runtime change a run.

    python tools/m13_belt_diagnosis.py artifacts/m13/belt/diagnosis_jobs/jobs3.json

Each job names a 20 s hold with its belt disturbance (or a matrix case by name) and a
runtime change: ``kd`` (the leg servo's rate gain per joint) and ``leg_tau`` (an
unbanded 1 kHz leg target filter) replace PolicyController.leg_torques in Python;
``band`` (with ``tau``, default 0.03) sets the 5 ms tick filter of policy_control with a
band. Full traces and the observations go to local/belt/diag/<name>.pkl (not committed;
artifacts/m13/belt/diagnosis.json summarises them). Four worker processes.
"""

import json
import multiprocessing
import pickle
import sys
from pathlib import Path

import numpy as np

from bipe import robustness as rb
from bipe.m13 import policy_control as pc
from bipe.m13.full_plant import CHECKPOINT
from bipe.parameters import ROOT, load_parameters
from bipe.profiles import native
from bipe.robustness_control import build_candidate

OUT = ROOT / "local/belt/diag"


def setup():
    p = load_parameters(profile="m10")
    prof = native(p)
    sensors = json.loads((ROOT / prof["sensors"]).read_text())
    gains = json.loads((ROOT / prof["controller"]).read_text())
    lib = build_candidate(p, gains, ROOT / ".venv/m8-build/current")
    return p, gains, sensors, str(lib)


def patch(mods):
    C = pc.PolicyController
    if "band" in mods or "tau" in mods:
        pc.LEG_FILTER_TAU_S = mods.get("tau", 0.03)
        band = mods.get("band")

        def filter_legs(self, dt):
            a = dt / (self.leg_filter_tau + dt)
            out = self.leg_output + a * (self.leg_target - self.leg_output)
            if band is not None:
                out = np.clip(out, self.leg_target - band, self.leg_target + band)
            self.leg_output = out

        C.filter_legs = filter_legs
        return
    tau = mods.get("leg_tau", 0.0)
    kp = np.array(mods.get("kp", [8.0] * 4))
    kd = np.array(mods.get("kd", [0.05] * 4))

    def leg_torques(self, s, qref, qdref, feedforward=None):
        if self.classical.protection.latched:
            return self.classical.leg_torques(s, qref, qdref, feedforward)
        if not hasattr(self, "filt"):
            self.filt = np.array(self.leg_target, float)
        if tau > 0:
            self.filt += (self.leg_target - self.filt) * (0.001 / (tau + 0.001))
        else:
            self.filt = np.array(self.leg_target, float)
        return kp * (self.filt - s["q"]) - kd * s["qd"]

    C.leg_torques = leg_torques


def one(args):
    name, dist, controller, mods = args
    p, gains, sensors, lib = setup()
    if isinstance(dist, str):
        case = next(c for c in rb.matrix(p) if c["name"] == dist)
    else:
        case = {"name": name, "family": "belt", "mode": "hold", "duration": 20, "disturbance": dist}
    if controller == "policy":
        patch(mods)
        with pc.policy_in_loop(CHECKPOINT, runners=("bipe.robustness",)):
            run = rb.run_case(p, gains, sensors, lib, case)
        ctl = pc.PolicyController.latest
        obs = np.array(ctl.observations)
    else:
        run = rb.run_case(p, gains, sensors, lib, case)
        obs = None
    m = rb.physical_metrics(run)
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / f"{name}.pkl", "wb") as f:
        pickle.dump({"trace": run["trace"][:, :], "obs": obs, "metrics": m, "dist": dist}, f)
    return name, m["recovered"], m["late_pitch_pp_deg"], m["late_speed_peak_m_s"]


if __name__ == "__main__":
    multiprocessing.set_start_method("fork", force=True)
    arg = sys.argv[1]
    jobs = json.loads(Path(arg).read_text() if Path(arg).exists() else arg)
    with multiprocessing.Pool(min(4, len(jobs))) as pool:
        for r in pool.imap_unordered(
            one,
            [
                (j["name"], j["dist"], j.get("controller", "policy"), j.get("mods", {}))
                for j in jobs
            ],
        ):
            print(r, flush=True)
