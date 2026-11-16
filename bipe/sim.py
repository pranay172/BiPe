"""M16 phase I: ``bipe sim``, BiPe in an interactive MuJoCo viewer on a CPU.

    python -m bipe sim                      # keyboard (and a gamepad if one is plugged in)
    python -m bipe sim --start sit          # start sitting; also front_lean, back_lean
    python -m bipe sim --headless [--video out.mp4]   # the scripted tour, no window

Runs the catalogue's ONNX policies (config/behaviours.json: drive, sit_stand, getup) on
the M14 training plant (bipe.m13.plant with the spring hip belts and knee dead band the
policies were trained on) in plain CPU MuJoCo, with ONNX Runtime and NumPy only: no
torch, no mjlab. Every key press is an intent (bipe.intents); the intent layer picks the
behaviour and its command, and the policy contract (docs/policy-contract.md) does the
rest: a 20 ms policy step, the 1 kHz leg PD and wheel PI loops, and the drive models
(bipe.m13.drives) under them.

What it is not: the full M10 plant. The gates are judged on that plant, with its sensor
model and C firmware, which runs about 18 times slower than real time. This sim reads
exact sensors (5 ms transport delay, as training's default) and runs a simple stand-in
for the supervisor: a fall starts the get-up policy, and a get-up or a stand hands back
to drive once the body has been level and still for half a second.

Keys (also printed at start):
    Up / Down        speed +/- 0.1 m/s          Left / Right   turn left / right
    Page Up / Down   stand taller / crouch      Home           stop
    End              sit                        Insert         stand (or get up)
    Enter            push from behind           F9             push from in front
    F8               a shove from in front that knocks it over (it then gets up)
    F10 / F11 / F12  moves: crouch bob, spin and stop, wiggle
    Delete           reset to standing
Gamepad: left stick speed, right stick turn, A sit, B stand or get up, X push, Y a move.
Mouse: the usual MuJoCo viewer (drag to orbit, right-drag to pan, scroll to zoom).
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

from .intents import NOMINAL_POSTURE, SIT_LABEL, Intents
from .m10.kinematics import posture
from .m13 import plant as tp
from .parameters import ROOT, load_parameters

POLICY_DT, PHYSICS_STEPS = 0.02, 20
TRANSPORT_DELAY = 5  # physics steps (ms), training's default
KP, KD, WHEEL_KV, WHEEL_KI = 8.0, 0.05, 0.05, 0.5  # docs/policy-contract.md
LEG_SCALE, WHEEL_SCALE = 0.5, 40.0
ACCELERATION, POSTURE_RATE = 1.25, 20.0
LABELS = np.linspace(30, 65, 36)
HANDOVER_DEG, HANDOVER_RATE, HANDOVER_HOLD_S = 10.0, 1.0, 0.5
FALL_DEG, FALL_HOLD_S = 50.0, 0.5
PUSH_S = 0.1
STARTS = ("stand", "sit", "front_lean", "back_lean")
FALLEN_STATES = ROOT / "artifacts/m15/fallen_states.npz"


def parameters():
    """The M14 training plant: spring hip belts and knee dead band (env_cfg belts=True)."""
    p = load_parameters(profile="m13")
    p["training"]["drives"].update(hip_belt="spring", knee_lash="deadband")
    return p


DRESSING = (
    '<texture name="sim_grid" type="2d" builtin="checker" rgb1=".30 .34 .40" rgb2=".22 .25 .30" '
    'width="512" height="512" mark="edge" markrgb=".45 .50 .56"/>'
    '<material name="sim_floor" texture="sim_grid" texrepeat="5 5" texuniform="true" reflectance=".03"/>'
    '<texture name="sim_sky" type="skybox" builtin="gradient" rgb1=".55 .65 .78" rgb2=".12 .14 .18" '
    'width="512" height="512"/>'
)


def dressed(xml):
    """The training plant's MJCF with a checker floor (0.1 m squares), a sky and a
    brighter headlight. Visual only: no physical field changes."""
    ground = '<geom name="ground" type="plane" size="0 0 .1" rgba=".26 .31 .37 1"'
    if ground not in xml or "<asset>" not in xml:
        return xml  # the plant changed shape: run undressed rather than fail
    xml = xml.replace(
        ground, '<geom name="ground" type="plane" size="0 0 .05" material="sim_floor"'
    )
    xml = xml.replace("<asset>", "<asset>" + DRESSING, 1)
    return xml.replace(
        '<visual><global offwidth="1280" offheight="720" />',
        '<visual><global offwidth="1280" offheight="720" />'
        '<headlight ambient=".35 .35 .35" diffuse=".55 .55 .55"/>',
    )


class OnnxPolicy:
    def __init__(self, path):
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), opts, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name

    def __call__(self, obs):
        x = np.asarray(obs, np.float32)[None]
        return self.session.run(None, {self.input: x})[0][0].astype(float)


def catalogue_policies():
    """ONNX graphs of the catalogue's policy behaviours, {name: path}."""
    cat = json.loads((ROOT / "config/behaviours.json").read_text())["behaviours"]
    return {
        k: ROOT / v["export"] / "policy.onnx" for k, v in cat.items() if v.get("kind") == "policy"
    }


class Sim:
    """The training plant, the policies, the intent layer and a stand-in supervisor."""

    def __init__(self, start="stand", seed=0, policies=None):
        self.p = p = parameters()
        self.m = mujoco.MjModel.from_xml_string(dressed(tp.generate_xml(p)))
        self.d = mujoco.MjData(self.m)
        self.rng = np.random.default_rng(seed)
        self.policies = {k: OnnxPolicy(v) for k, v in (policies or catalogue_policies()).items()}
        missing = {"drive", "sit_stand", "getup"} - set(self.policies)
        if missing:
            raise ValueError(f"catalogue has no policy for {sorted(missing)}")
        m = self.m
        self.ratio = p["joint_drive"]["ratio"]
        self.table = np.array([posture(float(x), p) for x in LABELS])
        g = p["geometry"]
        self.wheel_radius, self.wheel_track = g["wheel_radius"], g["wheel_track"]
        hub = p["physics"]["hub_motor"]
        self.wheel_limit = hub["torque_constant_Nm_A"] * hub["peak_current_A"] / WHEEL_KI
        qadr = lambda n: m.joint(n).qposadr[0]
        vadr = lambda n: m.joint(n).dofadr[0]
        sides = tp.SIDES
        self.hip_q = np.array([qadr(f"{s}_hip") for s in sides])
        self.shin_q = np.array([qadr(f"{s}_shin_joint") for s in sides])
        self.rotor_q = np.array([qadr(f"{s}_thigh_rotor_joint") for s in sides])
        self.hip_v = np.array([vadr(f"{s}_hip") for s in sides])
        self.shin_v = np.array([vadr(f"{s}_shin_joint") for s in sides])
        self.rotor_v = np.array([vadr(f"{s}_thigh_rotor_joint") for s in sides])
        self.wheel_v = np.array([vadr(w) for w in tp.WHEELS])
        self.gyro = m.sensor("gyro").adr[0]
        self.accel = m.sensor("accelerometer").adr[0]
        self.chassis = m.body("chassis").id
        self.ground = m.geom("ground").id
        self.tyres = {m.geom(f"{s}_tire").id for s in sides}
        self.reset(start)

    # State.

    def reset(self, start="stand"):
        if start not in STARTS:
            raise ValueError(f"unknown start {start!r}; known: {STARTS}")
        m, d, p = self.m, self.d, self.p
        self.drives = tp.Drives(m, p)
        tp.reset(m, d, p, NOMINAL_POSTURE)
        self.intents = Intents()
        if start != "stand":
            lib = np.load(FALLEN_STATES)
            names = [str(n) for n in lib["classes"]]
            rows = np.flatnonzero(lib["cls"] == names.index(start))
            q = lib["qpos"][self.rng.choice(rows)].astype(float)
            d.qpos[:] = q
            d.qpos[:2] = 0.0
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            if start == "sit":
                self.intents.sit()
            else:
                self.intents.get_up()
        self.value = np.array([0.0, 0.0, NOMINAL_POSTURE])
        self.action = np.zeros(6)
        self.leg_target = self.nominal(NOMINAL_POSTURE)
        self.wheel_target = np.zeros(2)
        self.wheel_integral = np.zeros(2)
        self.history = [self.measure()] * (TRANSPORT_DELAY + 1)
        self.push_left, self.push_force = 0, np.zeros(3)
        self.level_s = self.fallen_s = 0.0
        self.behaviour = None
        self.events = []
        self.t = 0.0

    def nominal(self, label):
        x = min(max(label, 30.0), 65.0) - 30.0
        i = min(math.floor(x), len(LABELS) - 2)
        w = x - i
        return self.table[i] * (1 - w) + self.table[i + 1] * w

    def measure(self):
        """Gyro, accelerometer, leg encoders [hip, shin angle] x 2, their speeds, wheel
        speeds: the training plant's exact readings (bipe.m13.rl.firmware, sensors off)."""
        d = self.d
        hip = d.qpos[self.rotor_q] / self.ratio  # the hip encoder is on the rotor
        hipd = d.qvel[self.rotor_v] / self.ratio
        a2 = d.qpos[self.hip_q] + d.qpos[self.shin_q]
        a2d = d.qvel[self.hip_v] + d.qvel[self.shin_v]
        return np.r_[
            d.sensordata[self.gyro : self.gyro + 3], d.sensordata[self.accel : self.accel + 3],
            hip[0], a2[0], hip[1], a2[1], hipd[0], a2d[0], hipd[1], a2d[1], d.qvel[self.wheel_v],
        ]  # fmt: skip

    def attitude(self):
        rot = self.d.xmat[self.chassis].reshape(3, 3)
        pitch = math.asin(float(np.clip(-rot[2, 0], -1, 1)))
        roll = math.atan2(rot[2, 1], rot[2, 2])
        gyro = self.d.sensordata[self.gyro : self.gyro + 3]
        return pitch, roll, float(gyro[1] * math.cos(roll) - gyro[2] * math.sin(roll))

    def speed(self):
        """Forward speed of the chassis, m/s."""
        rot = self.d.xmat[self.chassis].reshape(3, 3)
        heading = rot[:2, 0] / max(1e-6, np.linalg.norm(rot[:2, 0]))
        return float(self.d.qvel[:2] @ heading)

    # Intents that need the plant.

    def push(self, impulse_Ns, forward=True):
        """A push on the chassis along its heading, spread over PUSH_S."""
        rot = self.d.xmat[self.chassis].reshape(3, 3)
        heading = np.r_[rot[:2, 0], 0.0]
        heading /= max(1e-6, np.linalg.norm(heading))
        self.push_force = (1 if forward else -1) * impulse_Ns / PUSH_S * heading
        self.push_left = round(PUSH_S / self.m.opt.timestep)
        self.events.append((round(self.t, 2), f"push {impulse_Ns:+.1f} N s" if forward
                            else f"push {-impulse_Ns:+.1f} N s"))  # fmt: skip

    def rise(self):
        """Stand from a sit, or get up from a fall."""
        if self.intents.mode == "sit":
            self.intents.stand()
        elif self.fallen():
            self.intents.get_up()

    def seated(self):
        """Both tyres and some part of the body on the floor."""
        d, ground = self.d, self.ground
        others = {c.geom2 if c.geom1 == ground else c.geom1 for c in d.contact[: d.ncon]
                  if ground in (c.geom1, c.geom2)}  # fmt: skip
        tyres = sum(o in self.tyres for o in others)
        return tyres == 2 and len(others - self.tyres) > 0

    def fallen(self):
        pitch, roll, _ = self.attitude()
        return max(abs(pitch), abs(roll)) > math.radians(FALL_DEG)

    # Stepping.

    def supervise(self, dt):
        """The stand-in supervisor: falls start a get-up; level and still hands over."""
        pitch, roll, rate = self.attitude()
        mode = self.intents.mode
        if mode in ("drive",):
            self.fallen_s = self.fallen_s + dt if self.fallen() else 0.0
            if self.fallen_s >= FALL_HOLD_S:
                self.events.append((round(self.t, 2), "fell: get up"))
                self.intents.get_up()
                self.fallen_s = 0.0
        level = (abs(pitch) <= math.radians(HANDOVER_DEG) and abs(roll) <= math.radians(HANDOVER_DEG)
                 and abs(rate) <= HANDOVER_RATE)  # fmt: skip
        self.level_s = self.level_s + dt if level else 0.0
        if mode in ("getup", "stand") and self.level_s >= HANDOVER_HOLD_S:
            self.events.append((round(self.t, 2), f"{mode}: handed over to drive"))
            self.intents.handed_over()

    def policy_step(self):
        self.supervise(POLICY_DT)
        c = self.intents.step(POLICY_DT)
        if c.behaviour != self.behaviour:
            self.events.append((round(self.t, 2), f"behaviour {c.behaviour}"))
            self.behaviour = c.behaviour
        target = np.array([c.speed, c.yaw_rate, c.posture])
        if c.posture >= SIT_LABEL:
            self.value = target  # the sit request is set at once, as in training
        else:
            if self.value[2] >= SIT_LABEL:
                self.value[2] = NOMINAL_POSTURE
            step = np.array([ACCELERATION, math.inf, POSTURE_RATE]) * POLICY_DT
            self.value = self.value + np.clip(target - self.value, -step, step)
        row = self.history[0]
        q = row[6:10] - self.nominal(self.value[2])
        command = [self.value[0], self.value[1], (self.value[2] - 47.5) / 17.5]
        obs = np.r_[row[:6], q, row[10:16], self.action, command]
        self.action = self.policies[c.behaviour](obs)
        self.leg_target = self.nominal(self.value[2]) + LEG_SCALE * self.action[:4]
        forward = self.value[0] / self.wheel_radius
        turn = self.value[1] * self.wheel_track / (2 * self.wheel_radius)
        self.wheel_target = WHEEL_SCALE * self.action[4:] + np.array(
            [forward - turn, forward + turn]
        )

    def physics_step(self):
        m, d = self.m, self.d
        row = self.measure()
        self.history = self.history[1:] + [row]
        q, qd = row[6:10], row[10:14]
        legs = KP * (self.leg_target - q) - KD * qd
        error = self.wheel_target - d.qvel[self.wheel_v]
        dt = m.opt.timestep
        self.wheel_integral = np.clip(
            self.wheel_integral + error * dt, -self.wheel_limit, self.wheel_limit
        )
        wheels = WHEEL_KV * error + WHEEL_KI * self.wheel_integral
        self.drives.apply(m, d, np.r_[legs, wheels], dt)
        d.xfrc_applied[self.chassis, :3] = self.push_force if self.push_left > 0 else 0.0
        self.push_left = max(0, self.push_left - 1)
        mujoco.mj_step(m, d)
        self.t += dt

    def step(self):
        """One policy period: a policy step, then 20 physics steps."""
        self.policy_step()
        for _ in range(PHYSICS_STEPS):
            self.physics_step()


# The scripted tour (--headless), also the front-door check.

TOUR = [
    # (time s, what, argument)
    (0.0, "drive", (0.0, 0.0)),
    (2.0, "drive", (0.6, 0.0)),
    (5.0, "drive", (0.6, 0.6)),
    (8.0, "stop", None),
    (10.0, "push", 2.0),
    (13.0, "move", "crouch_bob"),
    (21.0, "sit", None),
    (27.0, "stand", None),
    (33.0, "push", -7.0),
    (42.0, "end", None),
]


def tour(sim, record=None):
    """Run TOUR; returns a summary with a verdict per segment."""
    script = list(TOUR)
    checks, samples = {}, []
    sat = False
    while script:
        t, what, arg = script[0]
        if sim.t + 1e-9 >= t:
            script.pop(0)
            if what == "end":
                break
            if what == "drive":
                sim.intents.drive(*arg)
            elif what == "stop":
                sim.intents.stop()
            elif what == "push":
                sim.push(abs(arg), forward=arg > 0)
            elif what == "move":
                sim.intents.move(arg)
            elif what == "sit":
                sim.intents.sit()
            elif what == "stand":
                sim.rise()
            continue
        sim.step()
        pitch, roll, _ = sim.attitude()
        samples.append((sim.t, sim.speed(), math.degrees(pitch), math.degrees(roll), sim.behaviour))
        if record is not None:
            record(sim)
        if 25.5 <= sim.t <= 27.0 and sim.seated():
            sat = True
    a = np.array([s[:4] for s in samples])
    window = lambda t0, t1: a[(a[:, 0] >= t0) & (a[:, 0] < t1)]
    drive = window(4.0, 5.0)
    checks["drive_0.6"] = bool(abs(drive[:, 1].mean() - 0.6) < 0.1)
    checks["push_recovered"] = bool(np.abs(window(12.0, 13.0)[:, 2]).max() < 10.0)
    checks["no_fall_before_sit"] = not any("fell" in e for t, e in sim.events if t < 21.0)
    checks["sat"] = sat
    checks["stood_and_handed_over"] = any("stand: handed over" in e for _, e in sim.events)
    checks["upright_after_stand"] = bool(np.abs(window(32.0, 33.0)[:, 2]).max() < 10.0)
    checks["shoved_over_and_got_up"] = any("fell" in e for t, e in sim.events if t >= 33.0) and any(
        "getup: handed over" in e for t, e in sim.events if t >= 33.0
    )
    checks["upright_at_end"] = bool(np.abs(window(41.0, 42.0)[:, 2]).max() < 10.0)
    return {"passed": all(checks.values()), "checks": checks, "events": sim.events,
            "sim_s": round(sim.t, 2)}  # fmt: skip


def headless(args):
    sim = Sim(start="stand", seed=args.seed)
    frames, renderer = [], None
    if args.video:
        renderer = mujoco.Renderer(sim.m, 480, 640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = sim.chassis
        camera.distance, camera.elevation, camera.azimuth = 0.9, -15.0, 120.0
        every = max(1, round(1 / (30 * POLICY_DT)))
        count = [0]

        def record(s):
            count[0] += 1
            if count[0] % every == 0:
                renderer.update_scene(s.d, camera)
                frames.append(renderer.render().copy())
    else:
        record = None
    wall = time.time()
    summary = tour(sim, record)
    summary["wall_s"] = round(time.time() - wall, 1)
    summary["real_time_factor"] = round(summary["sim_s"] / summary["wall_s"], 2)
    if frames:
        import imageio.v2 as imageio

        imageio.mimsave(args.video, frames, fps=round(1 / (every * POLICY_DT)))
        summary["video"] = str(args.video)
    print(json.dumps(summary, indent=1))
    return 0 if summary["passed"] else 1


# The viewer.

HELP = __doc__.split("Keys (also printed at start):\n")[1]
KEYS = {
    "up": 265, "down": 264, "left": 263, "right": 262, "page_up": 266, "page_down": 267,
    "home": 268, "end": 269, "insert": 260, "delete": 261, "enter": 257,
    "f8": 297, "f9": 298, "f10": 299, "f11": 300, "f12": 301,
}  # fmt: skip
MOVES = {"f10": "crouch_bob", "f11": "spin_stop", "f12": "wiggle"}


def on_key(sim, key):
    i = sim.intents
    name = next((k for k, v in KEYS.items() if v == key), None)
    if name in ("up", "down"):
        i.drive(i.speed + (0.1 if name == "up" else -0.1), i.yaw_rate)
    elif name in ("left", "right"):
        i.drive(i.speed, i.yaw_rate + (0.2 if name == "left" else -0.2))
    elif name in ("page_up", "page_down"):
        i.set_posture(i.posture + (-5 if name == "page_up" else 5))
    elif name == "home":
        i.stop()
    elif name == "end":
        i.sit()
    elif name == "insert":
        sim.rise()
    elif name in ("enter", "f9"):
        sim.push(2.0, forward=name == "enter")
    elif name == "f8":
        sim.push(7.0, forward=False)
    elif name in MOVES:
        i.move(MOVES[name])
    elif name == "delete":
        sim.reset("stand")


class Gamepad:
    """Polls the first joystick through GLFW (the viewer's own window library)."""

    def __init__(self):
        import glfw

        self.glfw, self.held, self.active = glfw, set(), False
        self.moves = list(MOVES.values())
        self.next_move = 0

    def poll(self, sim):
        glfw = self.glfw
        if not glfw.joystick_present(glfw.JOYSTICK_1):
            return
        axes, _ = glfw.get_joystick_axes(glfw.JOYSTICK_1)
        buttons, n = glfw.get_joystick_buttons(glfw.JOYSTICK_1)
        dead = lambda x: 0.0 if abs(x) < 0.15 else x
        if len(axes) >= 4:
            speed, yaw = -dead(axes[1]) * 1.2, -dead(axes[3]) * 1.0
            active = bool(speed or yaw)
            # Sticks command only while deflected, and once more on release (to stop),
            # so the keys keep control while the sticks rest.
            if (active or self.active) and sim.intents.move_name is None:
                sim.intents.drive(max(speed, -0.5), yaw)
            self.active = active
        pressed = {b for b in range(n) if buttons[b]}
        for b in pressed - self.held:
            if b == 0:
                sim.intents.sit()
            elif b == 1:
                sim.rise()
            elif b == 2:
                sim.push(2.0)
            elif b == 3:
                sim.intents.move(self.moves[self.next_move % len(self.moves)])
                self.next_move += 1
        self.held = pressed


def interactive(args):
    import mujoco.viewer

    sim = Sim(start=args.start, seed=args.seed)
    print(HELP)
    pending = []
    pad = None
    try:
        pad = Gamepad()
    except Exception:  # noqa: BLE001 - a missing joystick library is not an error here
        pad = None
    with mujoco.viewer.launch_passive(
        sim.m, sim.d, key_callback=pending.append, show_left_ui=False, show_right_ui=False
    ) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = sim.chassis
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.9, -15.0, 120.0
        shown = 0.0
        start = time.time() - sim.t
        while viewer.is_running():
            while pending:
                key = pending.pop(0)
                with viewer.lock():
                    on_key(sim, key)
                start = time.time() - sim.t if key == KEYS["delete"] else start
            if pad is not None:
                try:
                    pad.poll(sim)
                except Exception:  # noqa: BLE001
                    pad = None
            with viewer.lock():
                sim.step()
            viewer.sync()
            if sim.t - shown >= 0.5:
                shown = sim.t
                i = sim.intents
                status = (f"{sim.behaviour:9s} speed {sim.value[0]:+.2f} m/s  turn "
                          f"{sim.value[1]:+.2f} rad/s  posture {i.posture:.0f}  "
                          f"move {i.move_name or '-'}")  # fmt: skip
                if hasattr(viewer, "set_texts"):  # older viewers have no overlay
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, status, ""))
            lag = sim.t - (time.time() - start)
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.5:
                start = time.time() - sim.t  # slower than real time: do not try to catch up
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="bipe sim", description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog=HELP)  # fmt: skip
    parser.add_argument("--start", choices=STARTS, default="stand")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true", help="run the scripted tour, no window")
    parser.add_argument("--video", type=Path, help="with --headless: write an mp4 of the tour")
    args = parser.parse_args(argv)
    return headless(args) if args.headless else interactive(args)


if __name__ == "__main__":
    raise SystemExit(main())
