"""M16 phase G: the sit_stand policy's sit and stand on the full M10 plant.

    python -m bipe.m16.sit_eval --policy CKPT [--locomotion CKPT] [--trials 10]
        [--workers 4] [--output DIR]

Each trial starts balancing in the nominal 40 posture and runs three phases with the
sit_stand policy (the command's posture slot carries the request, bipe.intents):

- stand for STAND_S, held to the full plant's limits and safety gate as any balancing run;
- sit for SIT_S: the shell may touch the floor, so the physical limits, the safety gate
  and the protective fall are off (as before a get-up's handover). The robot must end
  the phase seated: shell on the floor, both tyres down, body within 20 degrees of
  level, the chassis nearly still. The peak downward speed of the chassis on the way
  down is recorded (a soft landing), and no leg joint may pass its range at any time;
- stand again: the handover back to balancing uses the get-up evaluation's rule on the
  true state and, as in M16, on the estimate (bipe.m15.getup_eval, quiet handover). It
  must come within RISE_BY_S, and the robot must then balance to the end with the
  locomotion policy (``--locomotion``, default the sit_stand policy itself), ending
  with each axle within POSTURE_MM of the 40 posture's depth (mean of the last second).

Trials cycle through the get-up gate's plants (nominal and four robustness variants);
each cycle shifts the sensor seed, so no two trials repeat. The C fallen mode runs the estimator throughout.

Evidence: a validated simulation under the M10 plant's assumptions, not hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial
from pathlib import Path

import numpy as np

from .. import firmware_control as fc
from ..getup_study import HANDOVER_DEG, HANDOVER_RATE, _contacts
from ..intents import SIT_LABEL
from ..m10.kinematics import leg_fk, posture, posture_target
from ..m15 import getup_eval as GE
from ..parameters import ROOT

STAND_S, SIT_S, RISE_BY_S, BALANCE_S = 3.0, 5.0, 5.0, 5.0
POSTURE_MM = 5.0
"""Gate 6c: the run ends with each axle within this of the commanded posture's depth
(mean over the last second)."""
SEATED_DEG, SEATED_SPEED = 20.0, 0.05
POLICY = LOCO = None  # actors, set in the parent before the fork


def _controller_class():
    from ..m13.policy_control import PolicyController

    class SitController(PolicyController):
        """The sit_stand policy with a sit request; ``armed`` tells simulate() when the
        balancing limits apply."""

        def __init__(self, p, gains, library=None, actor=None):
            super().__init__(p, gains, library, actor=POLICY)
            self.sitting, self.armed, self.rise_s, self.handover_s = False, True, None, None
            SitController.latest = self

        def label(self, qref):
            return SIT_LABEL if self.sitting else super().label(qref)

        def sit(self):
            self.sitting, self.armed = True, False
            self.step_limit[2] = math.inf  # the request is set at once, as in training

        def stand(self, t):
            self.sitting, self.rise_s = False, t
            self.value[2] = 40.0
            self.step_limit[2] = type(self)._posture_rate

        def command(self, s, velocity, yaw_rate, qref, qdref, dt):
            if self.armed:
                return super().command(s, velocity, yaw_rate, qref, qdref, dt)
            saved, self.classical.fall_settings = self.classical.fall_settings, None
            try:
                return super().command(s, velocity, yaw_rate, qref, qdref, dt)
            finally:
                self.classical.fall_settings = saved

        def arm(self, t):
            if not self.armed:
                self.armed, self.handover_s, self.actor = True, t, LOCO
                from ..firmware import activated

                cl = self.classical
                cl.supervisor, cl.safety_settings, _ = activated(
                    cl.lib, cl.start_ms + round(t * 1000) - cl.safety_settings.calibration_ms - 2
                )
                cl.protection = fc.FallState()

    return SitController


def trial(index):
    from ..m12 import speed as SP
    from ..moving_experiment import simulate
    from ..profiles import native
    from ..robustness_control import build_candidate

    plant_name = GE.PLANTS[index % len(GE.PLANTS)]
    p, gains, sensors, _ = SP.load()
    plant, config, seed = SP.variant_inputs(p, sensors, plant_name)
    # Each cycle through the plants draws new sensor noise: the variant's own seed, or the
    # sensor configuration's, shifted by the cycle (otherwise later cycles repeat the first
    # to the last digit).
    seed = (config["seed"] if seed is None else seed) + index // len(GE.PLANTS)
    library = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
    cls = _controller_class()
    from ..m13.rl.mdp import DriveCommandCfg

    cls._posture_rate = DriveCommandCfg(resampling_time_range=(1.0, 1.0)).posture_rate
    qref = posture(40, p)
    seen = {"peak_down_m_s": 0.0, "joint_limit": False, "quiet": 0, "depth_err": []}
    depth_ref = -posture_target(40, p)[0]
    est_cls = GE._estimator_class()
    GE.FALLEN = "c"

    def command(t, scenario):
        return 0.0, 0.0, qref, np.zeros(4)

    def hook(t, est, s):
        ctrl, d, m = cls.latest, seen.get("d"), seen["m"]
        if d is None:
            return
        if not ctrl.sitting and ctrl.rise_s is None and t >= STAND_S:
            ctrl.sit()
        elif ctrl.sitting and t >= STAND_S + SIT_S:
            tyres, shell = _contacts(m, d, m.geom("ground").id)
            speed = float(np.linalg.norm(d.qvel[:3]))
            seen["seated"] = bool(
                shell and tyres == 2 and abs(math.degrees(s["pitch"])) <= SEATED_DEG
                and speed <= SEATED_SPEED
            )  # fmt: skip
            seen["seated_state"] = {"pitch_deg": math.degrees(s["pitch"]), "speed_m_s": speed,
                                    "tyres": tyres, "shell": bool(shell)}  # fmt: skip
            ctrl.stand(t)
        if ctrl.sitting:
            seen["peak_down_m_s"] = max(seen["peak_down_m_s"], -float(d.qvel[2]))
        lo, hi = seen["range"][:, 0], seen["range"][:, 1]
        q = np.asarray(s["q"][:4])
        seen["joint_limit"] |= bool(np.any(q < lo - 0.001) or np.any(q > hi + 0.001))
        if t >= STAND_S + SIT_S + RISE_BY_S + BALANCE_S - 1.0:
            seen["depth_err"].append([abs(leg_fk(q[i : i + 2], p)[1] - depth_ref) for i in (0, 2)])
        g = p["sizing"]["gravity"]
        quiet = (abs(est["pitch"]) <= math.radians(HANDOVER_DEG)
                 and abs(est["pitch_rate"]) <= HANDOVER_RATE
                 and abs(est["accel_norm"] - g) <= GE.QUIET_G_TOLERANCE * g)  # fmt: skip
        seen["quiet"] = seen["quiet"] + 1 if quiet else 0
        if ctrl.rise_s is not None and not ctrl.armed and t <= ctrl.rise_s + RISE_BY_S:
            settled = (
                est_cls.accepted >= GE.ESTIMATE_SETTLE_TICKS
                and seen["quiet"] >= GE.ESTIMATE_SETTLE_TICKS
            )
            if settled and GE.handover_truth(m, d, p, s):
                ctrl.arm(t)

    def observer(t, dt, drives, d):
        seen.setdefault("d", d)

    saved = fc.FirmwareController, fc.FirmwareEstimator
    fc.FirmwareController, fc.FirmwareEstimator = partial(cls), est_cls
    t0 = time.time()
    try:
        from ..m10 import plant as m10
        from ..m10.plant import JOINTS

        model = m10.build(plant)[0]
        seen["m"] = model
        seen["range"] = np.array([model.joint(j).range for j in JOINTS[:4]])
        duration = STAND_S + SIT_S + RISE_BY_S + BALANCE_S
        try:
            run = simulate(
                "m16_sit", duration, sensing="sensors", seed=seed, nominal=p, plant=plant,
                gains=gains, sensor_config=config, implementation="c", firmware_library=library,
                physics_dt=plant["simulation"]["physics_dt"], command_function=command,
                command_hook=hook, observer=observer,
            )  # fmt: skip
        except ValueError as exc:
            run = {"failure": f"estimator rejected: {exc}"}
    finally:
        fc.FirmwareController, fc.FirmwareEstimator = saved
    ctrl = cls.latest
    rise = None if ctrl.handover_s is None or ctrl.rise_s is None else ctrl.handover_s - ctrl.rise_s
    depth_mm = (float(np.max(np.mean(seen["depth_err"], axis=0))) * 1000
                if seen["depth_err"] else None)  # fmt: skip
    passed = (seen.get("seated", False) and rise is not None and rise <= RISE_BY_S
              and not run["failure"] and not seen["joint_limit"]
              and depth_mm is not None and depth_mm <= POSTURE_MM)  # fmt: skip
    return {
        "trial": index, "plant": plant_name or "nominal", "seed": seed,
        "seated": seen.get("seated", False), "seated_state": seen.get("seated_state"),
        "peak_down_m_s": seen["peak_down_m_s"], "joint_limit": seen["joint_limit"],
        "rise_s": rise, "final_depth_error_mm": depth_mm,
        "failure": run["failure"], "passed": bool(passed),
        "wall_s": time.time() - t0,
    }  # fmt: skip


def main():
    global POLICY, LOCO
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--locomotion", type=Path)
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", type=Path)
    a = parser.parse_args()
    from ..m13.policy_control import Actor

    out = a.output or ROOT / f"local/m16/sit_eval_{a.policy.resolve().parent.name}"
    out.mkdir(parents=True, exist_ok=True)
    POLICY = Actor(str(a.policy))
    LOCO = Actor(str(a.locomotion)) if a.locomotion else POLICY
    record = out / "trials.json"
    done = {r["trial"]: r for r in json.loads(record.read_text())} if record.exists() else {}
    todo = [i for i in range(a.trials) if i not in done]
    ctx = multiprocessing.get_context("fork")
    with ProcessPoolExecutor(a.workers, mp_context=ctx) as pool:
        for fut in as_completed([pool.submit(trial, i) for i in todo]):
            r = fut.result()
            done[r["trial"]] = r
            record.write_text(json.dumps(sorted(done.values(), key=lambda r: r["trial"]),
                                         indent=1, default=float) + "\n")  # fmt: skip
            print("M16 SIT", r["trial"], r["plant"], "passed" if r["passed"] else "FAILED",
                  "seated" if r["seated"] else "not seated", r["rise_s"], r["failure"] or "",
                  f"{r['wall_s']:.0f} s", flush=True)  # fmt: skip
    rows = list(done.values())
    summary = {
        "trials": len(rows), "passed": sum(r["passed"] for r in rows),
        "seated": sum(r["seated"] for r in rows),
        "risen": sum(r["rise_s"] is not None for r in rows),
        "joint_limit": sum(r["joint_limit"] for r in rows),
        "peak_down_m_s_max": max((r["peak_down_m_s"] for r in rows), default=None),
        "final_depth_error_mm_max": max((r.get("final_depth_error_mm") or 0.0 for r in rows), default=None),
        "gate_6c_95pct": sum(r["passed"] for r in rows) >= 0.95 * len(rows) if rows else False,
    }  # fmt: skip
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
