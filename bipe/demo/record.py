"""Record a demo scene: one full-plant simulation, poses sampled at the video frame rate.

The run is the acceptance runners' simulate() on the M10 full plant with the sensor model
and the C firmware (classical controller), or a policy in its place
(bipe.m13.policy_control). Pushes are applied as chassis force pulses from the physics
step observer, the same xfrc_applied pulse the acceptance push cases use, so the
controller sees them only through its sensors.

    python -m bipe.demo record push_recovery --controller classical
    python -m bipe.demo record push_recovery --checkpoint artifacts/m13/training/settle/model_10493.pt
    python -m bipe.demo record push_recovery --onnx artifacts/m13/export/settle/policy.onnx
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from .. import moving_experiment
from ..parameters import ROOT
from .scenes import SCENES

OUTPUT = ROOT / "local/demo"
FPS = 50
PUSH_S = 0.1
_simulate = moving_experiment.simulate


class OnnxActor:
    """An exported policy graph (normaliser included), as bipe.m13.policy_control.Actor."""

    def __init__(self, path):
        import onnxruntime

        self.session = onnxruntime.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.name = self.session.get_inputs()[0].name

    def __call__(self, obs):
        x = np.asarray(obs, np.float32)[None]
        return self.session.run(None, {self.name: x})[0][0].astype(float)


def knot_command(knots, t):
    """Piecewise-linear [velocity, yaw_rate, posture_deg]; a repeated time is a step."""
    k = np.asarray(knots, dtype=float)
    i = int(np.searchsorted(k[:, 0], t, side="right")) - 1
    if i < 0:
        return k[0, 1:]
    if i >= len(k) - 1:
        return k[-1, 1:]
    s = (t - k[i, 0]) / (k[i + 1, 0] - k[i, 0])
    return k[i, 1:] + s * (k[i + 1, 1:] - k[i, 1:])


class Recorder:
    """Physics step observer: samples poses at the frame rate and applies the pushes."""

    def __init__(self, m_body_chassis, pushes, fps=FPS):
        self.chassis, self.pushes, self.dt_frame = m_body_chassis, pushes, 1.0 / fps
        self.times, self.qpos, self.forces = [], [], []

    def __call__(self, t, dt, drives, d):
        force = np.zeros(3)
        for start, impulse, heading in self.pushes:
            if start - 1e-9 <= t < start + PUSH_S - 1e-9:
                yaw = _yaw(d.qpos[3:7]) + math.radians(heading)
                force += impulse / PUSH_S * np.array([math.cos(yaw), math.sin(yaw), 0.0])
        if force.any():
            # simulate() clears xfrc_applied every step before its own pushes; this adds
            # the pulse after the drive torques are set and before the step integrates.
            d.xfrc_applied[self.chassis, :3] += force
        if not self.times or t >= self.times[-1] + self.dt_frame - 1e-9:
            self.times.append(t)
            self.qpos.append(d.qpos.copy())
            self.forces.append(force)


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def record(name, controller="classical", checkpoint=None, onnx=None, output=None, scene=None):
    from ..m10.kinematics import posture
    from ..m12 import speed as SP

    scene = scene or SCENES[name]
    p, gains, sensors, _ = SP.load()
    # An acceptance variant (mass_plus_15pct, friction_0p5, wheel_lag_10ms, ...).
    plant, sensors, seed = SP.variant_inputs(p, sensors, scene.get("variant"))
    chassis = 1  # the chassis body follows the world body in the M10 plant
    recorder = Recorder(chassis, scene.get("pushes", []))

    def command(t, scenario):
        v, w, deg = knot_command(scene["knots"], t)
        return float(v), float(w), posture(float(deg), p), np.zeros(4)

    disturbance = {}
    if scene.get("slope_deg"):
        disturbance["slope_deg"] = scene["slope_deg"]
    kwargs = {
        "sensing": "sensors",
        "nominal": p,
        "plant": plant,
        "seed": seed,
        "gains": gains,
        "sensor_config": sensors,
        "implementation": "c",
        "physics_dt": p["simulation"]["physics_dt"],
        "disturbance": disturbance,
        "command_function": command,
        "observer": recorder,
    }
    started = time.time()
    label = "classical"
    if checkpoint or onnx:
        from ..m13.policy_control import Actor, Counted, classes

        actor = Counted(OnnxActor(onnx) if onnx else Actor(checkpoint))
        label = str(onnx or checkpoint)
        with classes(actor):
            run = _simulate("demo", scene["duration"], **kwargs)
        if actor.calls == 0:
            raise RuntimeError("the policy never ran")
    else:
        run = _simulate("demo", scene["duration"], **kwargs)
    out = Path(output or OUTPUT / f"{name}_{controller}.npz")
    out.parent.mkdir(parents=True, exist_ok=True)
    meta = {
        "scene": name,
        "controller": controller,
        "policy": label,
        "failure": run["failure"],
        "peak_pitch_deg": float(run["stats"]["peak_pitch_deg"]),
        "peak_roll_deg": float(run["stats"]["peak_roll_deg"]),
        "wall_s": round(time.time() - started, 1),
        "fps": FPS,
        "scene_def": scene,
    }
    np.savez_compressed(
        out,
        t=np.array(recorder.times),
        qpos=np.array(recorder.qpos),
        force=np.array(recorder.forces),
        meta=json.dumps(meta),
    )
    return out, meta
