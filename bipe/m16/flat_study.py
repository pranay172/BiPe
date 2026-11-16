"""M16 phase F: can the robot get up from lying flat or on its side? (feasibility, no policy)

    python -m bipe.m16.flat_study states [--workers 4]
    python -m bipe.m16.flat_study random [--per-start 16] [--workers 4]
    python -m bipe.m16.flat_study search --start NAME [--population 24] [--iterations 8]

Finding that shapes this study (7 November 2026): of the 17 sideways tips that M15 phase D
left "on the side" under the configured protective fall, 15 went over the shell and came
to rest flat on the face or back (pitch 90 degrees, tyres off the floor), one rests on
its side and one in a back lean. The seven M15 get-up trials that never hand over are
the same kind of state: flat on the face or back. So the target here is the flat rests,
plus the one true side rest.

``states`` makes the starts on the full plant (the get-up study's plant, contact rows and
0.25 ms step): the 17 phase D side rests rerun with the configured protective fall
(bipe.m15.roll_cut), and the seven flat M15 get-up starts (bipe.m15.getup_eval.fallen_state).
``random`` drives each start through random open-loop keyframe motions (both legs
independent: hip and knee per side, plus wheel torques) and records which reach the
get-up study's handover state. ``search`` runs the study's cross-entropy search from one
start. Decision 3: train a flat/side get-up only if motions are found from at least 4 of
16 starts.

Evidence: a found motion is an existence proof under the plant's assumptions; finding
none is evidence, not proof, of infeasibility.
"""

from __future__ import annotations

import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from ..getup_study import HANDOVER_DEG, HANDOVER_RATE, KD, KP, _bounds, _contacts, _pitch_roll, _plant
from ..m10.kinematics import equilibrium, posture
from ..m10.plant import JOINTS, MechanicalDrives

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "local/m16/flat_study"
GETUP_FLAT_SEEDS = (12, 14, 20, 43, 52, 55, 66)  # never handed over in M15 (getup_eval_s1_v2)
KEYS, KEY_S, HOLD_S = 6, 0.6, 1.0
DIM = KEYS * 6


def _side_state(case):
    from ..m15 import roll_cut

    r = roll_cut.rollout(case, "firmware")
    return {"name": "side_t{tilt_deg:g}_s{side}_r{roll_rate:g}_v{speed:g}_g{slope_deg:g}".format(**case),
            "source": "roll_cut", "rest": r["rest"], "pitch_deg": r["pitch_deg"],
            "roll_deg": r["roll_deg"], **roll_cut.rollout.last}  # fmt: skip


def _flat_state(seed):
    from ..m15.getup_eval import fallen_state

    s = fallen_state(seed)
    return {"name": f"getup_{seed}", "source": "getup_eval", "rest": s["class"],
            "pitch_deg": s["pitch_deg"], "qpos": s["qpos"], "qvel": s["qvel"],
            "gravity": [0.0, 0.0, -9.81]}  # fmt: skip


def make_states(workers):
    from ..m15 import roll_cut

    old = json.loads((ROOT / "artifacts/m15/roll_cut/runs.json").read_text())
    keys = ("tilt_deg", "side", "roll_rate", "speed", "slope_deg")
    side = [{k: r[k] for k in keys} for r in old if r["variant"] == "firmware" and r["rest"] == "side"]
    with ProcessPoolExecutor(workers) as pool:
        states = list(pool.map(_side_state, side)) + list(pool.map(_flat_state, GETUP_FLAT_SEEDS))
    del roll_cut
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "states.json").write_text(json.dumps(states) + "\n")
    for s in states:
        print(s["name"], s["rest"], round(s["pitch_deg"]), round(s.get("roll_deg", 0.0)))
    return states


def load_states():
    return {s["name"]: s for s in json.loads((OUT / "states.json").read_text())}


def decode(theta, p):
    lo, hi = _bounds(p)
    keys = []
    for k in range(KEYS):
        t = np.clip(theta[k * 6 : (k + 1) * 6], 0, 1)
        keys.append((np.r_[lo + t[:2] * (hi - lo), lo + t[2:4] * (hi - lo)], (t[4:6] * 2 - 1) * 0.5))
    return keys


def rollout(state, theta, record=False):
    p, m, d = _plant()
    m.opt.gravity[:] = state["gravity"]
    d.qpos[:], d.qvel[:] = state["qpos"], state["qvel"]
    mujoco.mj_forward(m, d)
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    ground = m.geom("ground").id
    jq = np.array([m.joint(j).qposadr[0] for j in JOINTS])
    jv = np.array([m.joint(j).dofadr[0] for j in JOINTS])
    keys = decode(theta, p)
    start = d.qpos[jq].copy()
    target_pitch = equilibrium(p, posture(40, p))
    dt = m.opt.timestep
    best, reached, when, trace = math.inf, False, None, []
    peak_current = np.zeros(6)
    for step in range(round((KEYS * KEY_S + HOLD_S) / dt)):
        t = step * dt
        k = min(int(t // KEY_S), KEYS - 1)
        if t < KEYS * KEY_S:
            prev = start if k == 0 else keys[k - 1][0]
            s = (t - k * KEY_S) / KEY_S
            s = s * s * (3 - 2 * s)
            target, wheel = prev + s * (keys[k][0] - prev), keys[k][1]
        else:
            target, wheel = keys[-1][0], np.zeros(2)
        tau = KP * (target - d.qpos[jq]) - KD * d.qvel[jv]
        drives.step(m, d, np.r_[tau, wheel])
        peak_current = np.maximum(peak_current, abs(drives.phase_current))
        if not np.isfinite(d.qpos).all():
            return {"cost": 1e6, "handover": False, "best_handover_score": math.inf}
        if step % 4 == 0:
            pitch, _ = _pitch_roll(d)
            # Lateral tilt from the body y axis (the Euler roll is degenerate when the
            # robot lies flat on its face or back, pitch near 90 degrees).
            roll = math.asin(max(-1.0, min(1.0, float(d.xmat[m.body("chassis").id][7]))))
            err = abs(math.degrees(pitch - target_pitch))
            rate = abs(float(d.qvel[4]))
            tyres, shell = _contacts(m, d, ground)
            score = (err / HANDOVER_DEG + abs(math.degrees(roll)) / HANDOVER_DEG
                     + rate / HANDOVER_RATE + 2 * (2 - tyres) + 2 * shell)  # fmt: skip
            best = min(best, score)
            ok = (err <= HANDOVER_DEG and abs(math.degrees(roll)) <= HANDOVER_DEG
                  and rate <= HANDOVER_RATE and tyres == 2 and not shell)  # fmt: skip
            if ok and not reached:
                reached, when = True, t
            if record and step % 40 == 0:
                trace.append([t, math.degrees(pitch), math.degrees(roll), *np.degrees(d.qpos[jq])])
    out = {"cost": best - 100 * reached, "handover": reached, "handover_time_s": when,
           "best_handover_score": best, "phase_current_peak_A": peak_current.tolist(),
           "belt_peak_Nm": drives.peak_belt_torque.tolist()}  # fmt: skip
    if record:
        out["trace"] = trace
    return out


def _job(args):
    name, theta = args
    return name, theta.tolist(), rollout(load_states()[name], theta)


def random_motions(per_start, workers, seed=0):
    states = load_states()
    rng = np.random.default_rng(seed)
    jobs = [(n, rng.random(DIM)) for n in states for _ in range(per_start)]
    rows = {}
    with ProcessPoolExecutor(workers) as pool:
        for name, theta, r in pool.map(_job, jobs):
            rows.setdefault(name, []).append({"theta": theta, **r})
            print(name, r["handover"], round(r["best_handover_score"], 2), flush=True)
    summary = {
        n: {"rest": states[n]["rest"], "pitch_deg": states[n]["pitch_deg"],
            "handovers": sum(r["handover"] for r in v), "trials": len(v),
            "best_score": min(r["best_handover_score"] for r in v)}
        for n, v in rows.items()
    }  # fmt: skip
    found = sum(s["handovers"] > 0 for s in summary.values())
    result = {"per_start": per_start, "starts": len(summary), "starts_with_a_way_up": found,
              "summary": summary}  # fmt: skip
    (OUT / "random.json").write_text(json.dumps({**result, "rows": rows}) + "\n")
    print(json.dumps(result, indent=1))


def search(name, population, iterations, workers, seed=0):
    rng = np.random.default_rng(seed)
    mean, std = np.full(DIM, 0.5), np.full(DIM, 0.35)
    elite_n, history, best = max(2, population // 5), [], None
    with ProcessPoolExecutor(workers) as pool:
        for it in range(iterations):
            thetas = np.clip(rng.normal(mean, std, size=(population, DIM)), 0, 1)
            res = [r for _, _, r in pool.map(_job, [(name, th) for th in thetas])]
            costs = np.array([r["cost"] for r in res])
            order = np.argsort(costs)
            elite = thetas[order[:elite_n]]
            mean, std = elite.mean(0), np.maximum(elite.std(0), 0.03)
            i = order[0]
            if best is None or costs[i] < best["cost"]:
                best = {**res[i], "theta": thetas[i].tolist(), "iteration": it}
            history.append({"iteration": it, "best_cost": float(costs[i]),
                            "handovers": int(sum(r["handover"] for r in res))})  # fmt: skip
            print(name, json.dumps(history[-1]), flush=True)
            if best["handover"]:
                break
    replay = rollout(load_states()[name], np.array(best["theta"]), record=True)
    (OUT / f"search_{name}.json").write_text(
        json.dumps({"start": name, "best": best, "replay": replay, "history": history}) + "\n"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("states")
    s.add_argument("--workers", type=int, default=4)
    r = sub.add_parser("random")
    r.add_argument("--per-start", type=int, default=16)
    r.add_argument("--workers", type=int, default=4)
    c = sub.add_parser("search")
    c.add_argument("--start", required=True)
    c.add_argument("--population", type=int, default=24)
    c.add_argument("--iterations", type=int, default=8)
    c.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    if a.cmd == "states":
        make_states(a.workers)
    elif a.cmd == "random":
        random_motions(a.per_start, a.workers)
    else:
        search(a.start, a.population, a.iterations, a.workers)


if __name__ == "__main__":
    main()
