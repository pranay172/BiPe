"""M13 phase A fit: full-plant excitation traces, open-loop window replay and the parameter fit.

``record`` runs the full M10 plant with the classical C controller and the sensor model
on excitation commands (smooth random speed, yaw rate and posture, with pushes) and saves
the robot state, the requested motor torques and the phase currents at 1 kHz.

``replay`` starts the training plant at a recorded state and applies the recorded
requests open loop through the training plant's drives over short windows; ``errors``
summarises the state error against the full plant. ``fit`` adjusts the training-plant
parameters within their bounds to minimise that error on the fit split. Outputs go to
artifacts/m13/fit/.
"""

import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from ..m10 import plant as full
from ..m10.kinematics import SIDES, posture
from ..moving_experiment import simulate
from ..parameters import ROOT
from . import plant as tp
from .validate import load

OUTPUT = ROOT / "artifacts/m13/fit"
LOG_DT = 0.001
# Joints carried from the full plant to the training plant (same names in both).
JOINT_NAMES = ["base"] + [
    f"{s}_{j}" for s in SIDES for j in ("hip", "knee", "shin_joint", "wheel_joint")
]
EXCITATION = {
    "fit_a": {"seed": 1, "duration": 16, "speed": 0.6, "yaw": 0.8, "posture": True},
    "fit_b": {"seed": 2, "duration": 16, "speed": 1.0, "yaw": 0.4, "posture": False},
    "fit_hold": {"seed": 5, "duration": 8, "speed": 0.0, "yaw": 0.0, "posture": False},
    "fit_c": {"seed": 3, "duration": 16, "speed": 0.3, "yaw": 1.0, "posture": True, "push": 1.5},
    "val_a": {"seed": 4, "duration": 16, "speed": 0.8, "yaw": 0.6, "posture": True, "push": -1.0},
}


def excitation(spec):
    """Smooth random commands: sums of five sines per channel, faded in over 1 s."""
    rng = np.random.default_rng(spec["seed"])
    waves = {
        k: (rng.uniform(0.05, 0.6, 5), rng.uniform(0, 2 * math.pi, 5), rng.normal(size=5))
        for k in ("v", "w", "a")
    }

    def channel(key, t):
        freq, phase, amp = waves[key]
        value = float(np.sum(amp * np.sin(2 * math.pi * freq * t + phase)))
        return math.tanh(value / 2) * min(1.0, max(0.0, t - 1.0))

    def command(t, p):
        v = spec["speed"] * channel("v", t)
        w = spec["yaw"] * channel("w", t)
        angle = 47.5 + 12.5 * channel("a", t) if spec.get("posture") else 40.0
        return v, w, posture(angle, p), np.zeros(4)

    return command


class Recorder:
    """Reads the full plant every physics step; keeps the 1 kHz samples at servo updates."""

    def __init__(self, m):
        self.m = m
        self.qadr = [m.joint(n).qposadr[0] for n in JOINT_NAMES]
        self.vadr = [m.joint(n).dofadr[0] for n in JOINT_NAMES]
        self.rows = []
        self.next_t = 0.0
        self.wrapped = False

    def wrap(self, drives):
        apply = drives.apply

        def recorded(m, d, requested, dt=None, contact_callback=None):
            self.requested = np.asarray(requested, dtype=float).copy()
            self.current = drives.phase_current.copy()
            return apply(m, d, requested, dt, contact_callback)

        drives.apply = recorded
        self.wrapped = True

    def __call__(self, t, dt, drives, d):
        if not self.wrapped:
            # First call: apply() already ran for this step; take its request from ctrl.
            self.wrap(drives)
            return
        if t + 1e-9 < self.next_t:
            return
        self.next_t = t + LOG_DT
        qpos = np.concatenate(
            [d.qpos[a : a + (7 if i == 0 else 1)] for i, a in enumerate(self.qadr)]
        )
        qvel = np.concatenate(
            [d.qvel[a : a + (6 if i == 0 else 1)] for i, a in enumerate(self.vadr)]
        )
        self.rows.append((t, qpos, qvel, self.requested, self.current))


def library():
    """The M10 C core, built once in the parent (parallel builds race on its folder)."""
    from ..profiles import native
    from ..robustness_control import build_candidate

    p10, _, gains, _ = load()
    return str(build_candidate(p10, gains, ROOT / native(p10)["firmware_build"]))


def run_excitation(name, plant=None, core=None):
    """One excitation run on the full plant (plant None) or the given training plant.
    Returns the simulate result and the 1 kHz recorder rows."""
    p10, _, gains, sensors = load()
    core = core or library()
    spec = EXCITATION[name]
    command = excitation(spec)
    # The joint addresses come from a model of the same plant (simulate builds its own).
    model = full.build(p10)[0] if plant is None else tp.build(plant)[0]
    recorder = Recorder(model)
    disturbance = {"push_Ns": spec["push"]} if spec.get("push") else {}
    run = simulate(
        name,
        spec["duration"],
        sensing="sensors",
        nominal=p10,
        plant=plant,
        gains=gains,
        sensor_config=sensors,
        implementation="c",
        firmware_library=core,
        disturbance=disturbance,
        command_function=lambda t, s: command(t, p10),
        observer=recorder,
    )
    return run, recorder.rows


def record(name, output=OUTPUT, core=None):
    run, rows = run_excitation(name, core=core)
    spec = EXCITATION[name]
    out = Path(output) / "data"
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out / f"{name}.npz",
        time=np.array([r[0] for r in rows]),
        qpos=np.array([r[1] for r in rows]),
        qvel=np.array([r[2] for r in rows]),
        requested=np.array([r[3] for r in rows]),
        current=np.array([r[4] for r in rows]),
        joints=np.array(JOINT_NAMES),
    )
    meta = {"spec": spec, "failure": run["failure"], "warnings": run["warnings"]}
    (out / f"{name}.json").write_text(json.dumps(meta, indent=2) + "\n")
    return name, run["failure"], len(rows)


def replay(p13, data, starts, window_s):
    """Training-plant state errors over open-loop windows from recorded full-plant states.

    Returns an array (window, sample, channel) of training minus full for the channels
    pitch, pitch rate, forward speed, yaw rate, hip, knee drive, shin and wheel speeds.
    """
    m, d = tp.build(p13)
    tp.reset(m, d, p13)
    qadr = [m.joint(n).qposadr[0] for n in JOINT_NAMES]
    vadr = [m.joint(n).dofadr[0] for n in JOINT_NAMES]
    base_friction = m.dof_frictionloss.copy()
    dt = m.opt.timestep
    steps = round(window_s / LOG_DT)
    sub = round(LOG_DT / dt)
    if not math.isclose(sub * dt, LOG_DT):
        raise ValueError("training timestep must divide 1 ms")
    out = []
    for k in starts:
        m.dof_frictionloss[:] = base_friction
        mujoco.mj_resetData(m, d)
        _set_state(d, qadr, vadr, data["qpos"][k], data["qvel"][k])
        if p13["training"]["drives"].get("knee_lash") != "static":
            _close_knee_lash(m, d, p13)
        mujoco.mj_forward(m, d)
        drives = tp.Drives(m, p13)
        drives.phase_current[:] = data["current"][k]
        rows = []
        for j in range(steps):
            for _ in range(sub):
                drives.forward(m, d)
                drives.apply(m, d, data["requested"][k + j], dt)
                drives.integrate(m, d)
            rows.append(channels(_state(d, qadr, vadr)) - channels(_ref(data, k + j + 1)))
        out.append(rows)
    return np.array(out)


def _close_knee_lash(m, d, p):
    """Move each knee drive by the full plant's lash, so the training plant's lash-free
    belt starts at the recorded belt deflection rather than deflection plus lash."""
    half = math.radians(p["physics"]["belts"]["knee_lash_deg"]) / 2
    for side in SIDES:
        knee = m.joint(f"{side}_knee").qposadr[0]
        gap = d.qpos[knee] - d.qpos[m.joint(f"{side}_hip").qposadr[0]]
        gap -= d.qpos[m.joint(f"{side}_shin_joint").qposadr[0]]
        d.qpos[knee] -= math.copysign(min(abs(gap), half), gap)


def _set_state(d, qadr, vadr, qpos, qvel):
    i = v = 0
    for n, (a, b) in enumerate(zip(qadr, vadr, strict=True)):
        size_q, size_v = (7, 6) if n == 0 else (1, 1)
        d.qpos[a : a + size_q] = qpos[i : i + size_q]
        d.qvel[b : b + size_v] = qvel[v : v + size_v]
        i, v = i + size_q, v + size_v


def _state(d, qadr, vadr):
    qpos = np.concatenate([d.qpos[a : a + (7 if i == 0 else 1)] for i, a in enumerate(qadr)])
    qvel = np.concatenate([d.qvel[a : a + (6 if i == 0 else 1)] for i, a in enumerate(vadr)])
    return qpos, qvel


def _ref(data, k):
    k = min(k, len(data["time"]) - 1)
    return data["qpos"][k], data["qvel"][k]


CHANNELS = [
    "pitch_deg",
    "pitch_rate_deg_s",
    "forward_speed_m_s",
    "yaw_rate_rad_s",
    "left_hip_deg",
    "left_knee_deg",
    "left_shin_relative_deg",
    "left_wheel_rad_s",
    "right_wheel_rad_s",
]
SCALES = np.array([0.5, 10.0, 0.02, 0.05, 0.5, 0.5, 0.5, 1.0, 1.0])


def channels(state):
    """Compared channels of a carried state (qpos, qvel in JOINT_NAMES order)."""
    qpos, qvel = state
    quat = qpos[3:7]
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    rot = rot.reshape(3, 3)
    pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    # The free joint's angular velocity is in the body frame.
    omega = qvel[3:6]
    world = rot @ omega
    forward = float(qvel[0] * math.cos(yaw) + qvel[1] * math.sin(yaw))
    j = qpos[7:]
    jv = qvel[6:]
    return np.array(
        [
            math.degrees(pitch),
            math.degrees(omega[1]),
            forward,
            float(world[2]),
            math.degrees(j[0]),
            # The actual knee angle (thigh + shin), after the knee belt and its lash.
            math.degrees(j[0] + j[2]),
            math.degrees(j[2]),
            jv[3],
            jv[7],
        ]
    )


def load_data(name, output=OUTPUT):
    z = np.load(Path(output) / "data" / f"{name}.npz")
    return {k: z[k] for k in z.files}


def starts(data, window_s, count, rng):
    """Window start indices after the first 1.5 s, evenly spread with a random jitter."""
    first = int(1.5 / LOG_DT)
    last = len(data["time"]) - int(window_s / LOG_DT) - 2
    base = np.linspace(first, last, count).astype(int)
    jitter = rng.integers(0, max(1, (last - first) // count), count)
    return np.minimum(base + jitter, last)


def errors(p13, names, window_s=0.2, count=40, seed=0, output=OUTPUT):
    """RMS error per channel over the windows of the named traces, and a scalar score."""
    rng = np.random.default_rng(seed)
    blocks = []
    for name in names:
        data = load_data(name, output)
        blocks.append(replay(p13, data, starts(data, window_s, count, rng), window_s))
    e = np.concatenate(blocks)
    rms = np.sqrt(np.mean(e**2, axis=(0, 1)))
    final = np.sqrt(np.mean(e[:, -1, :] ** 2, axis=0))
    return {
        "window_s": window_s,
        "windows": int(e.shape[0]),
        "rms": dict(zip(CHANNELS, rms.tolist(), strict=True)),
        "end_rms": dict(zip(CHANNELS, final.tolist(), strict=True)),
        "score": float(np.mean((rms / SCALES) ** 2)),
    }


def closed_loop(payload):
    """Closed-loop RMS channel errors of one excitation run on a training plant."""
    training, name, output, core = payload
    shared = json.loads((ROOT / "config/robot.json").read_text())
    run, rows = run_excitation(name, tp.parameters(shared, training), core)
    data = load_data(name, output)
    n = min(len(rows), len(data["time"]))
    ours = np.array([channels((r[1], r[2])) for r in rows[:n]])
    theirs = np.array([channels((data["qpos"][k], data["qvel"][k])) for k in range(n)])
    rms = np.sqrt(np.mean((ours - theirs) ** 2, axis=0))
    # A failed (fallen) run scores as if every channel were ten scales off.
    penalty = 100.0 if run["failure"] else 0.0
    return {
        "name": name,
        "failure": run["failure"],
        "rms": dict(zip(CHANNELS, rms.tolist(), strict=True)),
        "score": float(np.mean((rms / CLOSED_SCALES) ** 2)) + penalty,
    }


# Closed-loop channel scales: quiet-hold and tracking differences that matter.
CLOSED_SCALES = np.array([0.3, 5.0, 0.03, 0.05, 0.5, 0.5, 0.5, 1.0, 1.0])
# Fitted parameters: (path in the training config, lower, upper, log scale).
PARAMETERS = {
    "tire_k": (("contact", "tire_solref", 0), -3e6, -2e5, True),
    "tire_b": (("contact", "tire_solref", 1), -3000.0, -10.0, True),
    "friction_tc": (("contact", "tire_solreffriction", 0), 0.002, 0.02, True),
    "hip_loss": (("friction", "hip_frictionloss_Nm"), 0.0, 0.03, False),
    "knee_loss": (("friction", "knee_drive_frictionloss_Nm"), 0.0, 0.03, False),
    "wheel_loss": (("friction", "wheel_frictionloss_Nm"), 0.0, 0.01, False),
}


def apply(training, values):
    t = copy.deepcopy(training)
    for key, value in values.items():
        path = PARAMETERS[key][0]
        node = t
        for part in path[:-1]:
            node = node[part]
        node[path[-1]] = float(value)
    return t


def current(training):
    """Current values of the fitted parameters (load-free defaults where null)."""
    p = tp.parameters(json.loads((ROOT / "config/robot.json").read_text()), training)
    fr = tp.friction(p)
    out = {}
    for key, (path, *_rest) in PARAMETERS.items():
        node = training
        for part in path:
            node = node[part]
        out[key] = node
    out["hip_loss"] = out["hip_loss"] if out["hip_loss"] is not None else fr["hip"][1]
    out["knee_loss"] = out["knee_loss"] if out["knee_loss"] is not None else fr["knee"][1]
    out["wheel_loss"] = out["wheel_loss"] if out["wheel_loss"] is not None else fr["wheel"][1]
    return out


def _evaluate(payload):
    training, names, window_s, count, seed, output = payload
    shared = json.loads((ROOT / "config/robot.json").read_text())
    return errors(tp.parameters(shared, training), names, window_s, count, seed, output)


def fit(names, iterations=60, workers=4, output=OUTPUT, start=None):
    """Coordinate search in normalised parameter space (log for stiffness-like terms),
    minimising the mean closed-loop score of the named excitation runs."""
    from ..parameters import load_parameters

    training = load_parameters(profile="m13")["training"]
    x = {**current(training), **(start or {})}
    keys = list(PARAMETERS)

    def to_unit(key, value):
        _, lo, hi, log = PARAMETERS[key]
        if log:
            return (math.log(abs(value)) - math.log(abs(lo))) / (
                math.log(abs(hi)) - math.log(abs(lo))
            )
        return (value - lo) / (hi - lo)

    def from_unit(key, u):
        _, lo, hi, log = PARAMETERS[key]
        u = min(1.0, max(0.0, u))
        if log:
            sign = -1 if lo < 0 else 1
            return sign * math.exp(math.log(abs(lo)) + u * (math.log(abs(hi)) - math.log(abs(lo))))
        return lo + u * (hi - lo)

    def values(u):
        return {k: from_unit(k, u[k]) for k in keys}

    unit = {k: to_unit(k, x[k]) for k in keys}
    history = []
    core = library()

    def score_many(candidates):
        payloads = [
            (apply(training, values(c)), n, output, core) for c in candidates for n in names
        ]
        with ProcessPoolExecutor(max_workers=workers) as pool:
            runs = list(pool.map(closed_loop, payloads))
        out = []
        for i in range(len(candidates)):
            block = runs[i * len(names) : (i + 1) * len(names)]
            out.append({"score": float(np.mean([r["score"] for r in block])), "runs": block})
        return out

    best = score_many([unit])[0]
    history.append({"values": values(unit), **best})
    print("start", round(best["score"], 4), flush=True)
    step = 0.15
    for _ in range(iterations):
        improved = False
        for key in keys:
            trial = [dict(unit, **{key: unit[key] + s}) for s in (-step, step)]
            results = score_many(trial)
            i = int(np.argmin([r["score"] for r in results]))
            if results[i]["score"] < best["score"] - 1e-4:
                unit, best, improved = trial[i], results[i], True
                history.append({"values": values(unit), **best})
                print(key, round(values(unit)[key], 6), round(best["score"], 4), flush=True)
        if not improved:
            step /= 2
            if step < 0.02:
                break
    return {
        "objective": "closed-loop RMS channel error against the full plant, scaled by CLOSED_SCALES",
        "scales": dict(zip(CHANNELS, CLOSED_SCALES.tolist(), strict=True)),
        "bounds": {k: PARAMETERS[k][1:3] for k in keys},
        "values": values(unit),
        "best": best,
        "history": history,
        "training": apply(training, values(unit)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=["record", "errors", "fit", "evaluate"])
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--window", type=float, default=0.2)
    args = parser.parse_args()
    fit_names = [n for n in EXCITATION if n.startswith("fit")]
    val_names = [n for n in EXCITATION if n.startswith("val")]
    if args.command == "record":
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            core = [library()] * len(EXCITATION)
            for result in pool.map(record, EXCITATION, [args.output] * len(EXCITATION), core):
                print(*result, flush=True)
        return
    if args.command == "evaluate":
        from ..parameters import load_parameters

        training, core = load_parameters(profile="m13")["training"], library()
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            runs = list(
                pool.map(closed_loop, [(training, n, args.output, core) for n in EXCITATION])
            )
        report = {
            "objective": "closed-loop RMS channel error against the full plant (1 kHz)",
            "scales": dict(zip(CHANNELS, CLOSED_SCALES.tolist(), strict=True)),
            "training": training,
            "runs": {r["name"]: r for r in runs},
            "fit_score": float(np.mean([r["score"] for r in runs if r["name"] in fit_names])),
            "validation_score": float(
                np.mean([r["score"] for r in runs if r["name"] in val_names])
            ),
        }
        (args.output / "closed_loop.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({k: report[k] for k in ("fit_score", "validation_score")}))
        return
    if args.command == "errors":
        from ..parameters import load_parameters

        p13 = load_parameters(profile="m13")
        report = {
            w: {
                "fit": errors(p13, fit_names, w, output=args.output),
                "validation": errors(p13, val_names, w, output=args.output),
            }
            for w in (0.05, 0.2, 0.5)
        }
        (args.output / "errors.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return
    result = fit(
        fit_names,
        workers=args.workers,
        output=args.output,
        start={"tire_k": -2e6, "tire_b": -300.0},
    )
    result["validation"] = [
        closed_loop((result["training"], n, args.output, library())) for n in val_names
    ]
    (args.output / "fit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["values"], indent=2))


if __name__ == "__main__":
    main()
