"""M15 phase D: does cutting leg torque at once on a roll fault turn roll-overs into sits?

    python -m bipe.m15.roll_cut [--workers 4] [--output DIR]

The get-up study (docs/studies/getup-feasibility.md) found a sideways tip with the legs
held rolls the robot over, which no sagittal leg motion undoes, while the same tip with
limp legs rolled back to a sit. This compares, on the full M10 plant (the get-up study's
plant, contact rows and 0.25 ms step):
- ``firmware``: the protective fall as configured (robot.json m9_physics.protective_fall, which the M10 plant's
  simulations do not load: they stop at the first failure,
  bipe.protective_fall.ProtectiveFall): on the 12 degree roll fault, crouch towards the
  60 degree posture for 0.25 s, ramp out over 0.08 s, then torque off;
- ``cut``: the same supervisor with the crouch, ramp and wheel fade at 1 ms, so the legs
  go limp as soon as the fault latches;
- ``held``: the get-up study's case for reference: the posture held for 1 s after the
  fault, then torque off.

Each tip starts at the nominal 40 degree posture, tilted sideways about the lower tyre's
outer edge to the tipping angle (the get-up study's release) plus an offset, with an
outward roll rate, a forward speed and a cross slope (gravity tilted about the heading;
both sides, so the tip goes downhill or uphill). Upright starts with a roll rate do not
tip: the wide track needs far more than the 4 rad/s tried. Until the fault
latches, the legs are held at the posture by a joint PD (the get-up study's kp 20, kd
0.3 through the command cap) and the wheels carry no torque: the balance controller's
pitch loop is left out over the tip's few hundred milliseconds. The supervisor reads the
plant's true state at the 5 ms control tick (no sensor model). After 4 s the rest state
is classed by the chassis's up axis: ``rolled`` (upside down, more than 100 degrees from
upright), ``sit`` (within 45 degrees, a tyre on the ground), ``side`` (between), or
``upright`` (no fault latched). ``rolled_through`` marks a run whose unwrapped roll
passed 180 degrees: it went over the shell, even if it came back onto its tyres.

Evidence: a validated simulation under these stated assumptions, not hardware.
"""

import argparse
import copy
import itertools
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from ..getup_study import KD, KP, _contacts, _pitch_roll, _plant
from ..m10.kinematics import posture
from ..m10.plant import JOINTS, WHEELS, MechanicalDrives, reset
from ..parameters import ROOT
from ..protective_fall import ProtectiveFall

OUT = ROOT / "local/m15/roll_cut"
TILTS = (-5.0, 0.0, 5.0, 10.0)  # deg past the tyre-edge tipping angle, both sides
ROLL_RATES = (0.0, 1.5)  # rad/s, outward
SPEEDS = (0.0, 0.5)  # m/s
SLOPES = (0.0, 4.0)  # cross slope, deg (downhill to +y)
VARIANTS = ("firmware", "cut", "held")
CONTROL_DT = 0.005
DURATION_S = 4.0


def cases():
    out = []
    for tilt, rate, sign, speed, slope in itertools.product(
        TILTS, ROLL_RATES, (1, -1), SPEEDS, SLOPES
    ):
        out.append(
            {"tilt_deg": tilt, "side": sign, "roll_rate": rate, "speed": speed, "slope_deg": slope}
        )
    return out


def rollout(case, variant):
    p, m, d = _plant()
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    q0 = posture(40, p)
    reset(m, d, p)
    s = math.radians(case["slope_deg"])
    g = abs(m.opt.gravity[2])
    m.opt.gravity[:] = [0.0, g * math.sin(s), -g * math.cos(s)]
    radius = p["geometry"]["wheel_radius"]
    sign = case["side"]
    mujoco.mj_forward(m, d)
    com = d.subtree_com[m.body("chassis").id]
    geo = p["geometry"]
    edge = geo["wheel_track"] / 2 + geo["wheel_width"] / 2
    angle = sign * (math.atan2(edge, com[2]) + math.radians(case["tilt_deg"]))
    tilt = np.r_[math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0]
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, tilt, d.qpos[3:7].copy())
    d.qpos[3:7] = quat
    mujoco.mj_forward(m, d)
    ground, distance = m.geom("ground").id, np.zeros(6)
    supports = [f"{x}_tire" for x in ("left", "right")] + [
        m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.endswith(("WheelRing", "SkidPad"))
    ]
    d.qpos[2] -= min(
        mujoco.mj_geomDistance(m, d, ground, m.geom(n).id, 1.0, distance) for n in supports
    )
    d.qvel[0] = case["speed"]
    d.qvel[3] = sign * case["roll_rate"]
    for w in WHEELS:
        d.qvel[m.joint(w).dofadr[0]] = case["speed"] / radius
    mujoco.mj_forward(m, d)
    jq = np.array([m.joint(j).qposadr[0] for j in JOINTS])
    jv = np.array([m.joint(j).dofadr[0] for j in JOINTS])
    pp = copy.deepcopy(p)
    shared = json.loads((ROOT / "config/robot.json").read_text())
    pp["physics"]["protective_fall"] = shared["m9_physics"]["protective_fall"]
    if variant == "cut":
        pp["physics"]["protective_fall"].update(crouch_s=0.001, ramp_s=0.001, wheel_ramp_s=0.001)
    fall = ProtectiveFall(pp, gains)
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    ticks = round(CONTROL_DT / m.opt.timestep)
    tau, latch_t, peak_roll, prev_pitch = np.zeros(6), None, 0.0, None
    unwrapped, last_roll = 0.0, None
    trace = []
    for step in range(round(DURATION_S / m.opt.timestep)):
        if step % ticks == 0:
            pitch, roll = _pitch_roll(d)
            q, qd = d.qpos[jq].copy(), d.qvel[jv].copy()
            rate = 0.0 if prev_pitch is None else (pitch - prev_pitch) / CONTROL_DT
            prev_pitch = pitch
            est = {"pitch": pitch, "pitch_rate": rate, "roll": roll, "velocity": float(d.qvel[0]),
                   "accel_norm": 9.81, "q": q, "qd": qd}  # fmt: skip
            out = fall.step(est, tau, CONTROL_DT)
            if fall.latched and latch_t is None:
                latch_t = float(d.time)
            if fall.latched and variant == "held" and d.time - latch_t < 1.0:
                tau = np.r_[np.clip(KP * (q0 - q) - KD * qd, -1.65, 1.65), 0.0, 0.0]
            elif fall.latched and variant == "held":
                tau = np.zeros(6)
            elif fall.latched:
                tau = out
            else:
                hold = np.clip(KP * (q0 - q) - KD * qd, -1.65, 1.65)
                tau = np.r_[hold, 0.0, 0.0]
            trace.append(
                [d.time, math.degrees(pitch), math.degrees(roll), float(np.abs(tau[:4]).max())]
            )
            if last_roll is not None:
                unwrapped += math.remainder(roll - last_roll, 2 * math.pi)
            else:
                unwrapped = roll
            last_roll = roll
            peak_roll = max(peak_roll, abs(math.degrees(unwrapped)))
        drives.step(m, d, tau)
    rollout.last = {"qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(),
                    "gravity": m.opt.gravity.tolist()}  # the rest state (M16 flat study)
    pitch, roll = _pitch_roll(d)
    tyres, shell = _contacts(m, d, m.geom("ground").id)
    up = float(d.xmat[m.body("chassis").id].reshape(3, 3)[2, 2])  # body z against world z
    if latch_t is None:
        rest = "upright"
    elif up < -0.17:
        rest = "rolled"
    elif up > 0.7 and tyres:
        rest = "sit"
    else:
        rest = "side"
    return {
        **case, "variant": variant, "latched_s": latch_t, "reason": int(fall.reason),
        "rest": rest, "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
        "peak_roll_deg": peak_roll, "rolled_through": peak_roll > 180, "up_z": up, "tyres_on_ground": tyres, "shell_on_ground": shell,
        "trace_columns": ["t", "pitch_deg", "roll_deg", "max_leg_torque_Nm"], "trace": trace[::4],
    }  # fmt: skip


def _job(args):
    return rollout(*args)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", default=str(OUT))
    parser.add_argument("--limit", type=int)
    a = parser.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    jobs = [(c, v) for c in cases() for v in VARIANTS][: a.limit]
    # Resumable: one JSON line per finished rollout.
    record = out / "runs.jsonl"
    results = [json.loads(x) for x in record.read_text().splitlines()] if record.exists() else []
    key = lambda c, v: json.dumps([c, v], sort_keys=True)
    done = {key({k: r[k] for k in c}, r["variant"]) for r in results for c in [cases()[0]]}
    todo = [(c, v) for c, v in jobs if key(c, v) not in done]
    with ProcessPoolExecutor(a.workers) as pool, record.open("a") as f:
        for r in pool.map(_job, todo):
            f.write(json.dumps(r, default=float) + "\n")
            f.flush()
            results.append(r)
    (out / "runs.json").write_text(json.dumps(results, default=float) + "\n")
    table = {}
    for r in results:
        t = table.setdefault(r["variant"], {})
        t[r["rest"]] = t.get(r["rest"], 0) + 1
    paired = {}
    for c in cases():
        rests = tuple(
            next(r["rest"] for r in results if r["variant"] == v and all(r[k] == c[k] for k in c))
            for v in ("firmware", "cut")
            if any(r["variant"] == v and all(r[k] == c[k] for k in c) for r in results)
        )
        if len(rests) == 2:
            paired[f"{rests[0]} -> {rests[1]}"] = paired.get(f"{rests[0]} -> {rests[1]}", 0) + 1
    summary = {"rest_by_variant": table, "firmware_to_cut": paired, "cases": len(cases())}
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
