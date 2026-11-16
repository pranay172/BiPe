"""Get-up feasibility study on the full M10 plant (roadmap v2, before M15).

Question: after a fall, can the six actuators, inside their torque, speed and
current limits, bring the robot back onto its wheels? No policy is trained. The
robot is tipped as in the M10 falls (bipe.m10.falls.release) with its legs held
at the nominal posture, and either keeps holding them or has its torque cut
after 1 s (the firmware's protective fall crouches, then cuts torque). Once at
rest it is driven open loop through joint-target keyframes with a local PD per
joint and wheel torque commands, all through MechanicalDrives. A cross-entropy
search over the keyframes looks for any motion that reaches a handover state.

Handover: both tyres on the ground, no shell contact, body pitch error (pitch
less the nominal posture's M10 equilibrium) and roll inside 10 deg, pitch rate
inside 1 rad/s. From there balancing is the locomotion controller's job, which
open-loop keyframes cannot do. A found motion is an existence proof under the
plant's assumptions; finding none is evidence, not proof, of infeasibility.

    .venv/bin/python -m bipe.getup_study rest
    .venv/bin/python -m bipe.getup_study search --case backward
"""

import argparse
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from bipe.m10.falls import release
from bipe.m10.kinematics import equilibrium, posture
from bipe.m10.plant import JOINTS, MechanicalDrives, build, parameters, reset

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts" / "getup_study"
# Fallen states: (tip direction, time the leg-holding torque is cut).
CASES = {
    "sit": ("forward", 1.0),  # torque cut: forward and backward tips both collapse to a level sit
    "front_lean": ("forward", None),  # legs held: rests on the tyres and the front shell
    "back_lean": ("backward", None),  # legs held: rests on the tyres and the rear shell
    "side_roll": ("sideways", 1.0),  # torque cut after a sideways tip: rolled over, tyres off the ground
}
SETTLE_S = 2.5
KEYS = 4  # keyframes after the settled pose
KEY_S = 0.6  # time between keyframes
HOLD_S = 1.0  # final keyframe held; the last 0.5 s is judged
KP, KD = 20.0, 0.3  # joint PD (Nm/rad, Nm s/rad); the 1.65 Nm cap binds first
HANDOVER_DEG, HANDOVER_RATE = 10.0, 1.0


def _plant():
    p = parameters(json.loads((ROOT / "config/robot.json").read_text()))
    m, d = build(p)
    return p, m, d


def _pitch_roll(d):
    w, x, y, z = d.qpos[3:7]
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return pitch, roll


def _contacts(m, d, ground):
    tyres, shell = set(), False
    for i in range(d.ncon):
        c = d.contact[i]
        if ground not in c.geom or c.dist > 0:
            continue
        name = m.geom(c.geom[1] if c.geom[0] == ground else c.geom[0]).name
        if name.endswith("_tire"):
            tyres.add(name)
        else:
            shell = True
    return len(tyres), shell


def settle(case):
    """Tip the nominal robot as in the M10 falls, legs held, and let it come to rest.

    The plant's own contact rows (as in the acceptance runs) at its 0.25 ms step,
    not the falls' explicit Coulomb law at 0.125 ms: the get-up motions are slow
    and the falls' impact accuracy is not what is judged here.
    """
    direction, cut = CASES[case]
    p, m, d = _plant()
    reset(m, d, p)
    release(m, d, p, direction)
    jq = np.array([m.joint(j).qposadr[0] for j in JOINTS])
    jv = np.array([m.joint(j).dofadr[0] for j in JOINTS])
    hold = posture(40, p)
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    for _ in range(round(SETTLE_S / m.opt.timestep)):
        held = cut is None or d.time < cut
        tau = KP * (hold - d.qpos[jq]) - KD * d.qvel[jv] if held else np.zeros(4)
        drives.step(m, d, np.r_[tau, 0.0, 0.0])
    # MechanicalDrives writes the load-dependent bearing friction into the model.
    return p, m, d, drives, m.dof_frictionloss.copy()


_SETTLED = {}


def settled(case):
    """A fresh copy of the settled state (each worker settles each case once)."""
    if case not in _SETTLED:
        _SETTLED[case] = settle(case)
    p, m, d0, drives0, friction = _SETTLED[case]
    m.dof_frictionloss[:] = friction
    d = mujoco.MjData(m)
    mujoco.mj_copyData(d, m, d0)
    drives = MechanicalDrives(m, p)
    for k in ("phase_current", "last_forces", "motor_torque"):
        setattr(drives, k, getattr(drives0, k).copy())
    return p, m, d, drives


def rest_report():
    rows = {}
    for case in CASES:
        _, m, d, _ = settled(case)
        pitch, roll = _pitch_roll(d)
        tyres, shell = _contacts(m, d, m.geom("ground").id)
        q = [float(d.qpos[m.joint(j).qposadr[0]]) for j in JOINTS]
        rows[case] = {
            "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll),
            "com_height_m": float(d.subtree_com[m.body("chassis").id][2]),
            "tyres_on_ground": tyres,
            "tip": CASES[case][0],
            "torque_cut_s": CASES[case][1],
            "shell_on_ground": shell,
            "joints_deg": [math.degrees(v) for v in q],
            "speed_m_s": float(np.linalg.norm(d.qvel[:3])),
        }
    return rows


def _bounds(p):
    lim = p["articulation"]
    lo1, hi1 = lim["hip_limits_deg"]
    lo2, hi2 = lim["knee_limits_deg"]
    # Two degrees inside the modelled stops so the PD does not sit on the pads.
    return np.radians([lo1 + 2, lo2 + 2]), np.radians([hi1 - 2, hi2 - 2])


def decode(theta, p, symmetric):
    """theta per keyframe: a1, a2 (unit box) per side (or shared) and two wheel torques."""
    lo, hi = _bounds(p)
    per = 3 if symmetric else 6
    keys = []
    for k in range(KEYS):
        t = np.clip(theta[k * per : (k + 1) * per], 0, 1)
        if symmetric:
            left = right = lo + t[:2] * (hi - lo)
            wheel = np.array([t[2], t[2]]) * 2 - 1
        else:
            left, right = lo + t[:2] * (hi - lo), lo + t[2:4] * (hi - lo)
            wheel = t[4:6] * 2 - 1
        keys.append((np.r_[left, right], wheel * 0.5))  # GL40 peak 0.5 Nm
    return keys


def rollout(case, theta, symmetric, record=False):
    p, m, d, drives = settled(case)
    ground = m.geom("ground").id
    jq = np.array([m.joint(j).qposadr[0] for j in JOINTS])
    jv = np.array([m.joint(j).dofadr[0] for j in JOINTS])
    keys = decode(theta, p, symmetric)
    start = d.qpos[jq].copy()
    q_end = posture(40, p)
    target_pitch = equilibrium(p, q_end)
    dt = m.opt.timestep
    total = KEYS * KEY_S + HOLD_S
    best, reached, when = math.inf, False, None
    trace = []
    peak_current = np.zeros(6)
    t0 = d.time
    pitch_dof = 4  # free-joint angular velocity about the body y axis
    for step in range(round(total / dt)):
        t = d.time - t0
        k = min(int(t // KEY_S), KEYS - 1)
        if t < KEYS * KEY_S:
            prev = start if k == 0 else keys[k - 1][0]
            s = (t - k * KEY_S) / KEY_S
            s = s * s * (3 - 2 * s)
            target = prev + s * (keys[k][0] - prev)
            wheel = keys[k][1]
        else:
            target, wheel = keys[-1][0], np.zeros(2)
        tau = KP * (target - d.qpos[jq]) - KD * d.qvel[jv]
        drives.step(m, d, np.r_[tau, wheel])
        peak_current = np.maximum(peak_current, abs(drives.phase_current))
        if not np.isfinite(d.qpos).all():
            return {"cost": 1e6, "handover": False}
        if step % 4 == 0:
            pitch, roll = _pitch_roll(d)
            err = abs(math.degrees(pitch - target_pitch))
            rate = abs(float(d.qvel[pitch_dof]))
            tyres, shell = _contacts(m, d, ground)
            score = (
                err / HANDOVER_DEG + abs(math.degrees(roll)) / HANDOVER_DEG
                + rate / HANDOVER_RATE + 2 * (2 - tyres) + 2 * shell
            )
            best = min(best, score)
            ok = (
                err <= HANDOVER_DEG and abs(math.degrees(roll)) <= HANDOVER_DEG
                and rate <= HANDOVER_RATE and tyres == 2 and not shell
            )
            if ok and not reached:
                reached, when = True, t
        if record and step % round(0.01 / dt) == 0:
            pitch, roll = _pitch_roll(d)
            trace.append([t, math.degrees(pitch), math.degrees(roll), *np.degrees(d.qpos[jq])])
    out = {
        "cost": best - 100 * reached,
        "handover": reached,
        "handover_time_s": when,
        "best_handover_score": best,
        "phase_current_peak_A": peak_current.tolist(),
        "belt_peak_Nm": drives.peak_belt_torque.tolist(),
    }
    if record:
        out["trace_columns"] = ["t", "pitch_deg", "roll_deg", *[f"{j}_deg" for j in JOINTS]]
        out["trace"] = trace
    return out


def _job(args):
    return rollout(*args)


def search(case, symmetric, population, iterations, workers, seed, stop=True):
    rng = np.random.default_rng(seed)
    dim = KEYS * (3 if symmetric else 6)
    mean, std = np.full(dim, 0.5), np.full(dim, 0.35)
    elite_n = max(2, population // 5)
    history, best = [], None
    with ProcessPoolExecutor(workers) as pool:
        for it in range(iterations):
            thetas = np.clip(rng.normal(mean, std, size=(population, dim)), 0, 1)
            results = list(pool.map(_job, [(case, th, symmetric) for th in thetas]))
            costs = np.array([r["cost"] for r in results])
            order = np.argsort(costs)
            elite = thetas[order[:elite_n]]
            mean, std = elite.mean(0), np.maximum(elite.std(0), 0.03)
            i = order[0]
            if best is None or costs[i] < best["cost"]:
                best = {**results[i], "theta": thetas[i].tolist(), "iteration": it}
            history.append(
                {
                    "iteration": it,
                    "best_cost": float(costs[i]),
                    "best_handover": results[i]["handover"],
                    "best_handover_score": results[i]["best_handover_score"],
                    "handovers": int(sum(r["handover"] for r in results)),
                    "population": len(results),
                    "mean_cost": float(costs.mean()),
                }
            )
            print(case, json.dumps(history[-1]), flush=True)
            if best["handover"] and stop:
                break
    return best, history


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("rest")
    s = sub.add_parser("search")
    s.add_argument("--case", choices=list(CASES), required=True)
    s.add_argument("--asymmetric", action="store_true")
    s.add_argument("--population", type=int, default=24)
    s.add_argument("--iterations", type=int, default=8)
    s.add_argument("--workers", type=int, default=4)
    s.add_argument("--seed", type=int, default=0)
    sub.add_parser("time")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.cmd == "rest":
        rows = rest_report()
        (OUT / "rest.json").write_text(json.dumps(rows, indent=2) + "\n")
        print(json.dumps(rows, indent=2))
    elif a.cmd == "time":
        for _ in range(2):
            t = time.time()
            r = rollout("sit", np.full(KEYS * 3, 0.5), True)
            print(time.time() - t, r["cost"])
    else:
        sym = not a.asymmetric
        t = time.time()
        best, history = search(a.case, sym, a.population, a.iterations, a.workers, a.seed)
        replay = rollout(a.case, np.array(best["theta"]), sym, record=True)
        name = f"{a.case}_{'sym' if sym else 'asym'}"
        (OUT / f"{name}.json").write_text(
            json.dumps(
                {
                    "case": a.case,
                    "symmetric": sym,
                    "settings": {
                        "settle_s": SETTLE_S, "keys": KEYS, "key_s": KEY_S, "hold_s": HOLD_S,
                        "kp": KP, "kd": KD, "population": a.population,
                        "iterations": a.iterations, "seed": a.seed,
                    },
                    "best": {k: v for k, v in best.items()},
                    "replay": replay,
                    "history": history,
                    "wall_s": time.time() - t,
                },
                indent=1,
            )
            + "\n"
        )


if __name__ == "__main__":
    main()
