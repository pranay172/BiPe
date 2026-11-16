"""M15 phase E: a library of fallen states in the training plant, for the get-up task.

    python -m bipe.m15.fallen [--count 400] [--workers 1]

Each state is a tip of the M14 training plant (belts as springs with lash, rotor
bodies; bipe.m13.plant with ``hip_belt="spring"``, ``knee_lash="deadband"``) on flat
ground, as in the get-up study (docs/studies/getup-feasibility.md) but randomised:
- start posture label 35 to 55 degrees, tipped forward or backward about the tyre
  contact by 15 to 35 degrees, forward speed -0.3 to 1.0 m/s;
- the legs held at the start posture by a joint PD (kp 20 Nm/rad, kd 0.3, through the
  1.65 Nm command cap) until a cut time drawn from 0.05 to 1.5 s, or held throughout
  (a third of the tips), with no wheel torque;
- 2.5 s to settle.

The rest state is kept if the robot lies on its tyres and shell, not rolled (|roll|
under 30 degrees), and classed by pitch: ``sit`` (within 20 degrees of level),
``front_lean`` and ``back_lean``. The library (full qpos, the class and the draw)
goes to artifacts/m15/fallen_states.npz; the get-up task's reset samples from it.

Evidence: training-plant states, a starting distribution for training, not a result.
"""

import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from ..parameters import ROOT, load_parameters
from ..m13 import plant as tp

OUT = ROOT / "artifacts/m15/fallen_states.npz"
KP, KD, CAP = 20.0, 0.3, 1.65
SETTLE_S = 2.5
CLASSES = ("sit", "front_lean", "back_lean")


def plant():
    p = load_parameters(profile="m13")
    p["training"]["drives"].update(hip_belt="spring", knee_lash="deadband")
    m, d = tp.build(p)
    return p, m, d


def _pitch_roll(q):
    """Pitch (nose down positive, the body x axis below the horizon) and the sideways
    tilt (the body y axis out of the horizontal), rad: well defined at large pitch."""
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, np.asarray(q, float))
    r = r.reshape(3, 3)
    return math.asin(np.clip(-r[2, 0], -1, 1)), math.asin(np.clip(r[2, 1], -1, 1))


def tip(seed):
    rng = np.random.default_rng(seed)
    draw = {
        "posture_deg": float(rng.uniform(35, 55)),
        "tilt_deg": float(rng.choice([-1, 1]) * rng.uniform(15, 35)),
        "speed_m_s": float(rng.uniform(-0.3, 1.0)),
        "cut_s": None if rng.random() < 1 / 3 else float(rng.uniform(0.05, 1.5)),
    }
    p, m, d = plant()
    tp.reset(m, d, p, angle_deg=draw["posture_deg"])
    legs = [m.joint(j).qposadr[0] for j in tp.JOINTS]
    vel = [m.joint(j).dofadr[0] for j in tp.JOINTS]
    hold = d.qpos[legs].copy()
    # Tip about the tyre contact: rotate the base about y through the contact point.
    a = math.radians(draw["tilt_deg"])
    contact = np.array([d.qpos[0], 0.0, 0.0])
    rot = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    d.qpos[:3] = contact + rot @ (d.qpos[:3] - contact)
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, np.r_[math.cos(a / 2), 0.0, math.sin(a / 2), 0.0], d.qpos[3:7].copy())
    d.qpos[3:7] = quat
    mujoco.mj_forward(m, d)
    lowest = min(
        mujoco.mj_geomDistance(m, d, m.geom("ground").id, g, 1.0, np.zeros(6))
        for g in range(m.ngeom)
        if g != m.geom("ground").id and m.geom(g).bodyid != 0
    )
    d.qpos[2] -= lowest - 1e-4
    d.qvel[0] = draw["speed_m_s"]
    radius = p["geometry"]["wheel_radius"]
    for w in tp.WHEELS:
        d.qvel[m.joint(w).dofadr[0]] = draw["speed_m_s"] / radius
    mujoco.mj_forward(m, d)
    knee_drive = [m.actuator(f"{s}_knee_drive").id for s in tp.SIDES]
    hip_drive = [m.actuator(f"{s}_hip_drive").id for s in tp.SIDES]
    order = [hip_drive[0], knee_drive[0], hip_drive[1], knee_drive[1]]
    for _ in range(round(SETTLE_S / m.opt.timestep)):
        held = draw["cut_s"] is None or d.time < draw["cut_s"]
        tau = np.clip(KP * (hold - d.qpos[legs]) - KD * d.qvel[vel], -CAP, CAP) if held else np.zeros(4)
        d.ctrl[:] = 0.0
        d.ctrl[order] = tau
        mujoco.mj_step(m, d)
    pitch, roll = _pitch_roll(d.qpos[3:7])
    ok = abs(math.degrees(roll)) < 30 and np.isfinite(d.qpos).all()
    pd = math.degrees(pitch)
    cls = "sit" if abs(pd) < 20 else ("front_lean" if pd > 0 else "back_lean")
    return {
        "seed": int(seed), "draw": draw, "kept": bool(ok), "class": cls,
        "pitch_deg": pd, "roll_deg": math.degrees(roll), "qpos": d.qpos.tolist(),
        "speed_m_s": float(np.linalg.norm(d.qvel[:3])),
    }  # fmt: skip


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--count", type=int, default=400)
    parser.add_argument("--workers", type=int, default=1)
    a = parser.parse_args()
    with ProcessPoolExecutor(a.workers) as pool:
        rows = list(pool.map(tip, range(a.count), chunksize=8))
    kept = [r for r in rows if r["kept"]]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUT,
        qpos=np.array([r["qpos"] for r in kept], dtype=np.float32),
        cls=np.array([CLASSES.index(r["class"]) for r in kept], dtype=np.int8),
        classes=np.array(CLASSES),
        pitch_deg=np.array([r["pitch_deg"] for r in kept], dtype=np.float32),
        draws=np.array([json.dumps(r["draw"]) for r in kept]),
    )
    counts = {c: sum(r["class"] == c for r in kept) for c in CLASSES}
    summary = {"tips": len(rows), "kept": len(kept), "rejected_rolled": len(rows) - len(kept), **counts}
    (OUT.with_suffix(".json")).write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
