"""M15 phase E: the get-up gate on the full M10 plant.

    python -m bipe.m15.getup_eval --getup CKPT [--locomotion CKPT] [--per-class 20]
        [--workers 4] [--output DIR] [--first-seed N] [--fallen python|c]

Trials: fallen states made on the full plant as bipe.m15.fallen makes them on the
training plant (tips forward and backward with the legs held, or cut at a random time,
2.5 s to settle; the get-up study's plant, contact rows and 0.25 ms step), drawn until
each class (sit, front lean, back lean) has ``--per-class`` trials. Each trial also
draws one plant from nominal and the speed suite's robustness variants.

Each trial runs the full closed loop (moving_experiment.simulate with the sensor model
and the policy where the C core runs, bipe.m13.policy_control), starting from the fallen
state with the sensors commissioned standing. The get-up policy acts until the get-up
study's handover test holds on the plant's true state (both tyres on the ground, no
shell contact, pitch error and roll inside 10 degrees, pitch rate inside 1 rad/s);
then the locomotion policy takes over and the run is judged as a balancing run (the
physical limits and the safety gate apply from the handover on). A trial passes if the
handover comes within 5 s and the robot then balances for 5 s with no physical failure.
Gate 6: at least 80 % of the trials, per class and overall.

Assumption, recorded: the switch uses the plant's true state. The firmware's own
handover detector (from the IMU and encoders) is a follow-up for the bench.

Evidence: a validated simulation under stated assumptions, not hardware.
"""

import argparse
import json
import math
import multiprocessing
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial
from pathlib import Path

import mujoco
import numpy as np

from .. import firmware_control as fc
from ..getup_study import HANDOVER_DEG, HANDOVER_RATE, KD, KP, _contacts, _plant
from ..m10.kinematics import equilibrium, posture
from ..m10.plant import JOINTS, MechanicalDrives, reset
from ..parameters import ROOT

LOCOMOTION = ROOT / "artifacts/m14/checkpoints/finetune_s1/model_9998.pt"
CLASSES = ("sit", "front_lean", "back_lean")
PLANTS = (None, "mass_plus_15pct", "friction_0p5", "wheel_lag_10ms", "delay_15ms_noise_x2")
HANDOVER_BY_S, BALANCE_S, SETTLE_S = 5.0, 5.0, 2.5
# Handover on the estimate: the C estimator accepted this many control ticks in a row
# and its pitch is inside this share of the supervisor's attitude limit.
ESTIMATE_SETTLE_TICKS, ATTITUDE_MARGIN = 50, 0.8
# M16 (held-out gate, 8 November 2026): five getup3 trials and one getup_s1 trial tripped
# the supervisor's attitude check 60-80 ms after a handover with the true pitch inside
# 1 degree: the estimate, re-initialised from the accelerometer during a fast get-up,
# had not converged. With QUIET_HANDOVER the estimate must also pass the study's own
# test (pitch inside 10 degrees, pitch rate inside 1 rad/s) with the accelerometer
# reading 1 g within 10 % (quasi-static, so its attitude can be trusted), for
# ESTIMATE_SETTLE_TICKS ticks in a row. Set by --quiet-handover.
QUIET_HANDOVER, QUIET_G_TOLERANCE = False, 0.1
# M18: each trial also draws a robot from bipe.m18.fidelity's ranges (--fidelity).
FIDELITY = False
MEASURED = None
"""M18 phase F: the measured robot's values (bipe.m18.measured, --measured) under every
trial, with the --fidelity draw on top."""
GETUP = None  # the get-up actor, set in the parent before the fork
GETUP_PATH, RECORD, RECORD_DIR = None, set(), None
# M16: the estimator's fallen mode in C (firmware/src/fallen.c, bipe_fallen_step) instead
# of this module's Python model of it ("python", as M15 ran). Set by --fallen.
FALLEN = "python"
LOCO = None
# M17: flat pre-pose (set by --flat-prepose). Leg targets [hip, knee drive] x 2 (rad) per
# flat side; a side left out keeps the plain get-up. 0.5 s is the M17 setting (0.25 s
# misses more flat-on-the-back starts: docs/milestones/17-flat-getup.md).
PREPOSE, PREPOSE_S, FLAT_DEG = {}, 0.5, 70.0


def stops_pose():
    """The flat pre-pose: hip and knee drive at their upper stops less the get-up study's
    2 degree margin, both legs. Found on the M16 held-out record: every face-down start that
    passed had its legs limp at these stops, every one that failed had them held at the
    riding pose (docs/milestones/17-flat-getup.md)."""
    from ..getup_study import _bounds

    hi = _bounds(_plant()[0])[1]
    return np.r_[hi, hi].tolist()


def fallen_state(seed):
    """One tip on the full plant, as bipe.m15.fallen.tip on the training plant."""
    rng = np.random.default_rng(10_000 + seed)
    draw = {
        "posture_deg": float(rng.uniform(35, 55)),
        "tilt_deg": float(rng.choice([-1, 1]) * rng.uniform(15, 35)),
        "speed_m_s": float(rng.uniform(-0.3, 1.0)),
        "cut_s": None if rng.random() < 1 / 3 else float(rng.uniform(0.05, 1.5)),
        "plant": PLANTS[int(rng.integers(len(PLANTS)))],
    }
    p, m, d = _plant()
    reset(m, d, p, angle_deg=draw["posture_deg"])
    jq = np.array([m.joint(j).qposadr[0] for j in JOINTS])
    jv = np.array([m.joint(j).dofadr[0] for j in JOINTS])
    hold = d.qpos[jq].copy()
    a = math.radians(draw["tilt_deg"])
    contact = np.array([d.qpos[0], 0.0, 0.0])
    rot = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    d.qpos[:3] = contact + rot @ (d.qpos[:3] - contact)
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, np.r_[math.cos(a / 2), 0.0, math.sin(a / 2), 0.0], d.qpos[3:7].copy())
    d.qpos[3:7] = quat
    mujoco.mj_forward(m, d)
    ground = m.geom("ground").id
    lowest = min(
        mujoco.mj_geomDistance(m, d, ground, g, 1.0, np.zeros(6))
        for g in range(m.ngeom)
        if g != ground and m.geom(g).bodyid != 0
    )
    d.qpos[2] -= lowest - 1e-4
    d.qvel[0] = draw["speed_m_s"]
    for w in ("left_wheel_joint", "right_wheel_joint"):
        d.qvel[m.joint(w).dofadr[0]] = draw["speed_m_s"] / p["geometry"]["wheel_radius"]
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    for _ in range(round(SETTLE_S / m.opt.timestep)):
        held = draw["cut_s"] is None or d.time < draw["cut_s"]
        tau = KP * (hold - d.qpos[jq]) - KD * d.qvel[jv] if held else np.zeros(4)
        drives.step(m, d, np.r_[tau, 0.0, 0.0])
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, d.qpos[3:7])
    pitch = math.degrees(math.asin(np.clip(-r[6], -1, 1)))
    side = math.degrees(math.asin(np.clip(r[7], -1, 1)))
    cls = "sit" if abs(pitch) < 20 else ("front_lean" if pitch > 0 else "back_lean")
    return {"seed": seed, "draw": draw, "class": cls, "pitch_deg": pitch, "side_deg": side,
            "kept": abs(side) < 30, "qpos": d.qpos.tolist(), "qvel": d.qvel.tolist()}  # fmt: skip


def _pose(s):
    """True pitch, roll (deg) and leg angles (rad) for the trial record."""
    return {
        "pitch_deg": math.degrees(s["pitch"]), "roll_deg": math.degrees(s["roll"]),
        "q": np.asarray(s["q"][:4], float).tolist(),
    }  # fmt: skip


def handover_truth(m, d, p, s):
    """The get-up study's handover test on the true state. The pitch error is from the
    nominal 40 posture's equilibrium, as in the study, not from the equilibrium at the
    current legs (which any pose with its COM over the axle meets, folded legs too)."""
    tyres, shell = _contacts(m, d, m.geom("ground").id)
    err = math.degrees(s["pitch"] - equilibrium(p, posture(40, p)))
    return (
        tyres == 2
        and not shell
        and abs(err) <= HANDOVER_DEG
        and abs(math.degrees(s["roll"])) <= HANDOVER_DEG
        and abs(s["pitch_rate"]) <= HANDOVER_RATE
    )


def _controller_class():
    from ..m13.policy_control import PolicyController

    class GetupController(PolicyController):
        """The get-up actor until the handover, then the locomotion actor (same
        observation and action law, so the switch keeps the action history)."""

        def __init__(self, p, gains, library=None, actor=None):
            super().__init__(p, gains, library, actor=GETUP)
            self.armed, self.handover_s = False, None
            self.prepose, self.prepose_ticks = None, 0
            GetupController.latest = self

        def _prepose(self, s, dt):
            """M17 flat pre-pose: on a flat start (estimated pitch past FLAT_DEG on the
            first tick) the legs go to that side's pose for PREPOSE_S before the get-up
            actor starts, wheels held at zero speed. Returns False once it is over."""
            if self.prepose is None:
                pitch = math.degrees(s["pitch"])
                pose = PREPOSE.get("front" if pitch > 0 else "back") if abs(pitch) > FLAT_DEG else None
                self.prepose = (np.asarray(pose, float), round(PREPOSE_S / dt)) if pose is not None else False
            if not self.prepose or self.prepose_ticks >= self.prepose[1]:
                return False
            self.prepose_ticks += 1
            self.leg_target, self.wheel_target = self.prepose[0].copy(), np.zeros(2)
            self.filter_legs(dt)
            return True

        def command(self, s, velocity, yaw_rate, qref, qdref, dt):
            if self.armed:
                return super().command(s, velocity, yaw_rate, qref, qdref, dt)
            if PREPOSE and self._prepose(s, dt):
                return np.r_[np.zeros(4), self.wheel_request], s["pitch"], self.value[0], s["yaw"]
            # Before the handover the robot is down on purpose: the protective fall
            # (which latches at once on a fallen pose and cuts torque) is not run.
            saved, self.classical.fall_settings = self.classical.fall_settings, None
            try:
                return super().command(s, velocity, yaw_rate, qref, qdref, dt)
            finally:
                self.classical.fall_settings = saved

        def arm(self, t):
            if not self.armed:
                self.armed, self.handover_s, self.actor = True, t, LOCO
                # The supervisor is not ticked while the robot is down (fallen mode), so
                # it is commissioned again at the handover, ending 1 ms before the first
                # gate tick; otherwise that tick trips the control deadline.
                from ..firmware import activated

                cl = self.classical
                cl.supervisor, cl.safety_settings, _ = activated(
                    cl.lib, cl.start_ms + round(t * 1000) - cl.safety_settings.calibration_ms - 2
                )
                cl.protection = fc.FallState()

    return GetupController


def _estimator_class():
    from ..m13.policy_control import PolicyEstimator

    class GetupEstimator(PolicyEstimator):
        """The C estimator, and while it refuses the state (legs outside the range its
        kinematics accept, as in a fallen pose; a pitch near vertical) the raw channels
        with filtered encoder rates, the attitude from the accelerometer, and the C
        estimator re-initialised from that packet, so it takes over as soon as the state
        is valid again. This is M16's rule; the C fallen mode (--fallen c) also waits for
        BIPE_FALLEN_TAKEOVER_TICKS quiet ticks since M17, which this model does not."""

        refused = 0
        accepted = 0  # consecutive ticks the C estimator has accepted

        def update(self, data):
            if FALLEN == "c":
                return self._update_c(data)
            gyro, accel, enc = self.next_packet(data)
            enc = np.asarray(enc, float)
            prev = getattr(self, "_prev", enc)
            rate = (enc - prev) / self.dt
            a = self.dt / (self.settings.joint_tau + self.dt)
            self._rate = getattr(self, "_rate", np.zeros(6)) + a * (
                rate - getattr(self, "_rate", np.zeros(6))
            )
            self._prev = enc
            result = fc.Estimate()
            ok = self.lib.bipe_estimator_step(
                fc.c.byref(self.state), fc.c.byref(self.settings), fc.D3(*gyro), fc.D3(*accel),
                fc.D6(*enc), fc.c.byref(result),
            )  # fmt: skip
            if ok:
                s = fc.estimate_dict(result)
                GetupEstimator.accepted += 1
            else:
                GetupEstimator.refused += 1
                GetupEstimator.accepted = 0
                g = np.asarray(accel, float)
                s = {
                    "pitch": math.atan2(-g[0], math.hypot(g[1], g[2])), "pitch_rate": float(gyro[1]),
                    "roll": math.atan2(g[1], g[2]), "yaw": 0.0, "yaw_rate": float(gyro[2]),
                    "position": np.zeros(2), "velocity": 0.0, "q": enc[:4].copy(),
                    "qd": self._rate[:4].copy(), "wheel_speed": self._rate[4:].copy(),
                    "accel_norm": float(np.linalg.norm(g)),
                }  # fmt: skip
                self.lib.bipe_estimator_init(fc.c.byref(self.state), fc.D3(*accel), fc.D6(*enc))
            s.update(gyro_raw=np.array(gyro), accel_raw=np.array(accel), encoders_raw=enc.copy())
            return s

        def _update_c(self, data):
            """The firmware's fallen mode (bipe_fallen_step), from the commissioned state."""
            gyro, accel, enc = self.next_packet(data)
            if not hasattr(self, "fallen"):
                self.fallen = fc.FallenState()
                self.fallen.estimator = self.state
            result, valid = fc.Estimate(), fc.c.c_bool()
            fc.checked(self.lib.bipe_fallen_step(
                fc.c.byref(self.fallen), fc.c.byref(self.settings), fc.D3(*gyro), fc.D3(*accel),
                fc.D6(*enc), fc.c.byref(result), fc.c.byref(valid)))  # fmt: skip
            GetupEstimator.accepted = int(self.fallen.accepted)
            GetupEstimator.refused = int(self.fallen.refused)
            s = fc.estimate_dict(result)
            if not valid.value:
                s["yaw"] = 0.0
            s.update(gyro_raw=np.array(gyro), accel_raw=np.array(accel),
                     encoders_raw=np.asarray(enc, float).copy())  # fmt: skip
            return s

    return GetupEstimator


def trial(state, record_to=None):
    from ..m12 import speed as SP
    from ..moving_experiment import simulate
    from ..profiles import native
    from ..robustness_control import build_candidate

    p, gains, sensors, _ = SP.load()
    plant, config, seed = SP.variant_inputs(p, sensors, state["draw"]["plant"])
    fid = None
    if FIDELITY or MEASURED is not None:
        # M18: the measured robot (--measured) and/or a random robot from the fidelity
        # ranges (--fidelity) on top of the trial's plant; measured values stay fixed.
        from ..m13 import policy_control
        from ..m18 import fidelity, measured

        fid = fidelity.sample(np.random.default_rng(1_000_003 + state["seed"])) if FIDELITY else {}
        if MEASURED is not None:
            fid.update(measured.draw(MEASURED))
            plant, config = measured.apply_plant(plant, config, MEASURED)
        plant, config, policy = fidelity.apply(plant, config, fid, seed=state["seed"])
        policy_control.FIDELITY = policy
    library = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
    cls = _controller_class()
    qref = posture(40, p)
    seen = {}

    def command(t, scenario):
        return 0.0, 0.0, qref, np.zeros(4)

    def hook(t, est, s):
        ctrl = cls.latest
        seen.setdefault("start", _pose(s))
        if QUIET_HANDOVER:
            g = p["sizing"]["gravity"]
            quiet = (
                abs(est["pitch"]) <= math.radians(HANDOVER_DEG)
                and abs(est["pitch_rate"]) <= HANDOVER_RATE
                and abs(est["accel_norm"] - g) <= QUIET_G_TOLERANCE * g
            )
            seen["quiet"] = seen.get("quiet", 0) + 1 if quiet else 0
        if "d" in seen and not ctrl.armed and t <= HANDOVER_BY_S:
            # The firmware hands over on its own estimate: the C estimator must have
            # accepted the state for a while and see the robot inside the supervisor's
            # attitude limit, as well as the true state passing the study's test.
            settled = est_cls.accepted >= ESTIMATE_SETTLE_TICKS and abs(est["pitch"]) <= (
                ATTITUDE_MARGIN * ctrl.classical.safety_settings.attitude_max
            )
            if QUIET_HANDOVER:
                settled = settled and seen.get("quiet", 0) >= ESTIMATE_SETTLE_TICKS
            if settled and handover_truth(seen["m"], seen["d"], p, s):
                ctrl.arm(t)
                seen["at_handover"] = _pose(s)
        seen["end"] = _pose(s)

    recorder = None
    if record_to is not None:
        from ..demo.record import Recorder

        recorder = Recorder(1, [])  # the chassis body follows the world body

    def observer(t, dt, drives, d):
        seen.setdefault("d", d)
        if recorder is not None:
            recorder(t, dt, drives, d)

    saved = fc.FirmwareController, fc.FirmwareEstimator
    est = est_cls = _estimator_class()
    fc.FirmwareController, fc.FirmwareEstimator = partial(cls), est
    try:
        from ..m10 import plant as m10

        seen["m"] = m10.build(plant)[0]  # for contact names only; the run's own model is used
        try:
            run = simulate(
            "m15_getup", HANDOVER_BY_S + BALANCE_S, sensing="sensors", seed=seed, nominal=p,
            plant=plant, gains=gains, sensor_config=config, implementation="c",
            firmware_library=library, physics_dt=plant["simulation"]["physics_dt"],
            disturbance={"initial_state": {"qpos": state["qpos"], "qvel": state["qvel"]}},
            command_function=command, command_hook=hook, observer=observer,
            )  # fmt: skip
        except ValueError as exc:
            # The C estimator refuses a state it cannot represent (pitch within 6 degrees
            # of vertical, the robot flat on its face or back): a firmware fault.
            run = {"failure": f"estimator rejected: {exc}", "stats": {"peak_torque": np.zeros(6)}}
    finally:
        fc.FirmwareController, fc.FirmwareEstimator = saved
    ctrl = cls.latest
    handover = ctrl.handover_s
    if recorder is not None and recorder.times:
        _save_clip(record_to, recorder, state, handover, run["failure"])
    passed = handover is not None and handover <= HANDOVER_BY_S and not run["failure"]
    return {
        "seed": state["seed"], "class": state["class"], "draw": state["draw"],
        "start_pitch_deg": state["pitch_deg"], "handover_s": handover,
        "balanced_s": None if handover is None else HANDOVER_BY_S + BALANCE_S - handover,
        "failure": run["failure"], "passed": bool(passed), "estimator_refused_ticks": est.refused,
        "peak_motor_Nm": np.asarray(run["stats"]["peak_torque"]).tolist(),
        "at_start": seen.get("start"), "at_handover": seen.get("at_handover"),
        "at_end": seen.get("end"),
    }  # fmt: skip


def _save_clip(path, recorder, state, handover, failure):
    """The trial's poses in the bipe.demo format (``python -m bipe.demo render``)."""
    from ..demo.record import FPS

    title = f"Get-up · {state['class'].replace('_', ' ')} · seed {state['seed']}"
    meta = {
        "scene": f"getup_{state['seed']}", "controller": "getup", "policy": str(GETUP_PATH),
        "failure": failure, "handover_s": handover, "fps": FPS,
        "scene_def": {"duration": recorder.times[-1], "title": title, "title_hold": True,
                      "camera": {"move": "fixed", "distance": 0.9, "elevation": -10,
                                 "azimuth": 90}},
    }  # fmt: skip
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path, t=np.array(recorder.times), qpos=np.array(recorder.qpos),
        force=np.array(recorder.forces), meta=json.dumps(meta),
    )  # fmt: skip


def _trial_worker(state):
    t0 = time.time()
    clip = RECORD_DIR / f"getup_{state['seed']}.npz" if state["seed"] in RECORD else None
    r = trial(state, clip)
    r["wall_s"] = time.time() - t0
    return r


def _rel(path):
    path = Path(path).resolve()
    return str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)


def _commit():
    from ..provenance import commit

    return commit()


def main():
    global GETUP, LOCO, FALLEN, QUIET_HANDOVER, PREPOSE, PREPOSE_S
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--getup", type=Path, required=True)
    parser.add_argument("--locomotion", type=Path, default=LOCOMOTION)
    parser.add_argument("--per-class", type=int, default=20)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--record", type=int, action="append", default=[], metavar="SEED",
        help="save these trials' poses as demo clips (OUTPUT/clips/getup_SEED.npz) and "
        "run only them (rerunning them if already done)",
    )  # fmt: skip
    parser.add_argument("--first-seed", type=int, default=0,
                        help="draw the fallen states from this seed on (M16: 1000 is the "
                        "held-out set; M15 picked on seeds from 0)")
    parser.add_argument("--fallen", choices=["python", "c"], default="python",
                        help="the estimator's fallen mode: the C firmware, or this module's model of M16's "
                        "rule (python, without M17's quiet takeover)")
    parser.add_argument("--classes", default=",".join(CLASSES),
                        help="only these start classes, comma separated (M16: sit alone "
                        "judges the sit_stand policy's stand)")
    parser.add_argument("--fidelity", action="store_true",
                        help="M18: each trial on a robot drawn from the fidelity ranges")
    parser.add_argument("--measured", nargs="?", const="config/hardware-measured.json",
                        help="M18: each trial on the measured robot (bipe.m18.measured)")
    parser.add_argument("--quiet-handover", action="store_true",
                        help="hand over only once the estimate itself is level and still "
                        "and the accelerometer reads 1 g (M16)")
    parser.add_argument("--flat-prepose", default="",
                        help="M17: flat sides (front, back, comma separated) whose start "
                        "first moves the legs to their upper stops for --prepose-s")
    parser.add_argument("--prepose-s", type=float, default=PREPOSE_S)
    a = parser.parse_args()
    FALLEN, QUIET_HANDOVER, PREPOSE_S = a.fallen, a.quiet_handover, a.prepose_s
    global FIDELITY, MEASURED
    FIDELITY = a.fidelity
    if a.measured:
        from ..m18 import measured

        MEASURED = measured.load(ROOT / a.measured)
    sides = [x for x in a.flat_prepose.split(",") if x]
    assert set(sides) <= {"front", "back"}, sides
    PREPOSE = dict.fromkeys(sides, stops_pose())
    from ..m13.policy_control import Actor

    out = a.output or ROOT / f"local/m15/getup_eval_{a.getup.resolve().parent.name}"
    out.mkdir(parents=True, exist_ok=True)
    GETUP, LOCO = Actor(str(a.getup)), Actor(str(a.locomotion))
    global GETUP_PATH, RECORD, RECORD_DIR
    GETUP_PATH, RECORD, RECORD_DIR = a.getup, set(a.record), out / "clips"
    ctx = multiprocessing.get_context("fork")
    states_path = out / "states.json"
    if states_path.exists():
        states = json.loads(states_path.read_text())
    else:
        wanted = [c for c in a.classes.split(",") if c]
        assert set(wanted) <= set(CLASSES), wanted
        states, counts, seed = [], dict.fromkeys(wanted, 0), a.first_seed
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as pool:
            while min(counts.values()) < a.per_class and seed < a.first_seed + 50 * a.per_class:
                batch = list(pool.map(fallen_state, range(seed, seed + 4 * a.workers)))
                seed += 4 * a.workers
                for s in batch:
                    if s["kept"] and s["class"] in counts and counts[s["class"]] < a.per_class:
                        counts[s["class"]] += 1
                        states.append(s)
        states_path.write_text(json.dumps(states) + "\n")
    record = out / "trials.json"
    done = {r["seed"]: r for r in json.loads(record.read_text())} if record.exists() else {}
    todo = [s for s in states if s["seed"] not in done]
    if RECORD:
        todo = [s for s in states if s["seed"] in RECORD]
        record = out / "clips" / "trials.json"
        record.parent.mkdir(parents=True, exist_ok=True)
        done = {}
    with ProcessPoolExecutor(a.workers, mp_context=ctx) as pool:
        for fut in as_completed([pool.submit(_trial_worker, s) for s in todo]):
            r = fut.result()
            done[r["seed"]] = r
            record.write_text(json.dumps(list(done.values()), indent=1, default=float) + "\n")
            print("M15 GETUP", r["seed"], r["class"], "passed" if r["passed"] else "FAILED",
                  r["handover_s"], r["failure"] or "", f"{r['wall_s']:.0f} s", flush=True)  # fmt: skip
    rows = list(done.values())
    summary = {
        c: {"trials": sum(r["class"] == c for r in rows), "passed": sum(r["passed"] for r in rows if r["class"] == c)}
        for c in CLASSES
    }  # fmt: skip
    summary["all"] = {"trials": len(rows), "passed": sum(r["passed"] for r in rows)}
    flat = [r for r in rows if abs(r.get("start_pitch_deg") or 0.0) > 70.0]
    summary["flat_over_70deg"] = {"trials": len(flat), "passed": sum(r["passed"] for r in flat)}
    summary["gate_80pct"] = all(
        summary[k]["passed"] >= 0.8 * summary[k]["trials"]
        for k in (*CLASSES, "all") if summary[k]["trials"]
    )  # the flat subset is reported, not gated
    summary["settings"] = {
        "getup": _rel(a.getup), "locomotion": _rel(a.locomotion), "fallen": FALLEN,
        "quiet_handover": QUIET_HANDOVER, "flat_prepose": sorted(PREPOSE),
        "prepose_s": PREPOSE_S if PREPOSE else None, "flat_deg": FLAT_DEG,
        "commit": _commit(), "mujoco": mujoco.__version__, "fidelity": FIDELITY,
        "measured": MEASURED,
    }  # fmt: skip
    if not RECORD:
        (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
