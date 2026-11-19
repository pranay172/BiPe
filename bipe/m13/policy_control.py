"""M13 phase C: the trained policy as a controller in the full plant's simulate().

The policy replaces the classical body loop and joint references; everything around
it is the robot's: the full plant's sensor model and transport delay, its motor and
belt models, and the C supervisor's gate and protective fall, which run on the
classical estimator's state exactly as they do around the classical controller.

    with policy_in_loop("artifacts/m13/training/hard/model_2998.pt"):
        run = simulate(..., implementation="c", ...)

Inside the context, ``simulate`` builds :class:`PolicyEstimator` and
:class:`PolicyController` where it would build the C estimator and controller, so the
M10 acceptance and M12 speed runners take the policy unchanged.

The robot side of the policy interface is bipe.m13.rl.firmware's, in numpy:
- every 20 ms (every fourth 5 ms control tick) the observation is built from the
  latest delayed sensor packet (gyro, accelerometer, joint encoders) and the
  estimator's encoder-derived joint and wheel speeds, the last action and the
  rate-limited command, and the policy's mean action is held for 20 ms;
- every 5 ms tick the leg references step through the leg target filter
  (LEG_FILTER_TAU_S, kept within LEG_FILTER_BAND_RAD of them) towards the policy's
  leg targets;
- every 1 ms the leg PD runs on the local encoder readings and their filtered rates
  (no added lag: the full plant has the belt lash that training randomised as lag),
  and the wheel PI speed loop runs on the hub speed with the rolling feedforward.
The knee encoder reads the motor side of the knee belt, where training read the
shin angle (about 0.5-0.6 degrees apart at rest, inside the trained encoder offset).
The hub speed is the wheel joint's speed, as the hub drive measures it on its own
encoder (training read the same).
"""

from __future__ import annotations

import contextlib
import math
from functools import partial

import numpy as np

from .. import firmware_control as fc
from ..parameters import ROOT
from .rl.firmware import DriveActionCfg, posture_table
from .rl.mdp import DriveCommandCfg

WHEELS = ("left_wheel_joint", "right_wheel_joint")
LABELS = np.linspace(30, 65, 36)
# The leg target filter's time constant, s (0: off, the targets apply directly, as in
# training). Every 5 ms control tick the leg references the servo follows take one
# first-order step towards the policy's leg targets. At the soft end of the knee belt
# range (23.5 N m/rad) a 4.6 Hz knee and pitch mode is underdamped and the policy
# sustains it as an 8-11 degree pitch limit cycle at rest. 0.03 s with a 0.06 rad band
# recovers 7 of the 8 belt corners (4 without), but 30 of the 44 speed cases (36 without),
# so it is off (docs/milestones/13-learned-locomotion-plan.md, "Belt range follow-up").
LEG_FILTER_TAU_S = 0.0
# With the filter on, the filtered reference stays within this band of the target, rad,
# so large leg moves (a push reaction) pass with at most this lag: unbanded, forward
# pushes of 1.5 N s and more fall.
LEG_FILTER_BAND_RAD = 0.06
# M16 command shaper (None: off, as trained and as M13 to M15 ran). A dict with
# ``acceleration`` (m/s^2), ``jerk`` (m/s^3) and ``yaw_acceleration`` (rad/s^2): speed and
# yaw-rate commands pass through a jerk-limited second-order limiter at the policy step,
# before the training rate limit (bipe.m16.shaper). Set by ``full_plant --shaper``.
SHAPER = None
# M18 fidelity settings for the policy's own firmware path (bipe.m18.fidelity.apply):
# ``command_loss`` (probability that a policy step's new targets reach the moteus one
# 5 ms tick late), ``wheel_encoder_scale`` (the hub speed loop reads its encoder with
# the wheel radius scale) and ``seed``. None: off.
FIDELITY = None
# The C classes, kept here because policy_in_loop() swaps the module's names.
Classical, ClassicalEstimator = fc.FirmwareController, fc.FirmwareEstimator


class Actor:
    """The checkpoint's actor: observation normaliser and MLP, deterministic mean."""

    def __init__(self, checkpoint):
        import torch

        state = torch.load(checkpoint, map_location="cpu", weights_only=True)["actor_state_dict"]
        get = lambda k: state[k].double().numpy()
        self.mean, self.std = get("obs_normalizer._mean")[0], get("obs_normalizer._std")[0]
        self.layers = [(get(f"mlp.{i}.weight"), get(f"mlp.{i}.bias")) for i in (0, 2, 4, 6)]

    def __call__(self, obs):
        x = (np.asarray(obs, float) - self.mean) / (self.std + 1e-2)
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < len(self.layers) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
        return x


class CActor:
    """The firmware forward pass (firmware/src/locomotion, float32) through ctypes.

    Build it with ``make -C firmware locomotion`` after ``python -m bipe.m13.export``."""

    def __init__(self, library=None):
        import ctypes

        self.lib = ctypes.CDLL(str(library or ROOT / "firmware/build/liblocomotion.so"))
        self.lib.bipe_locomotion_checkpoint.restype = ctypes.c_char_p
        self.checkpoint_sha256 = self.lib.bipe_locomotion_checkpoint().decode()
        self.obs, self.act = (ctypes.c_float * 25)(), (ctypes.c_float * 6)()

    def __call__(self, obs):
        self.obs[:] = np.asarray(obs, np.float32).tolist()
        self.lib.bipe_locomotion_forward(self.obs, self.act)
        return np.array(self.act[:], float)


class Perturbed:
    """An actor whose observations get seeded Gaussian noise of ``scale`` (M16 margin
    reruns: a recovery verdict within a hair of its band is close to chaotic, PR #3
    review). The generator is made here, so every forked case starts from the same
    state and a (seed, case) pair is reproducible."""

    def __init__(self, actor, seed, scale=1e-5):
        self.actor, self.scale = actor, scale
        self.rng = np.random.default_rng(seed)

    def __call__(self, obs):
        obs = np.asarray(obs, float)
        return self.actor(obs + self.scale * self.rng.standard_normal(obs.shape))


class PolicyEstimator(ClassicalEstimator):
    """The C estimator, also returning the raw delayed packet it was given."""

    def update(self, data):
        gyro, accel, enc = self.next_packet(data)
        result = fc.Estimate()
        fc.checked(
            self.lib.bipe_estimator_step(
                fc.c.byref(self.state), fc.c.byref(self.settings), fc.D3(*gyro), fc.D3(*accel),
                fc.D6(*enc), fc.c.byref(result),
            )
        )  # fmt: skip
        s = fc.estimate_dict(result)
        s.update(gyro_raw=np.array(gyro), accel_raw=np.array(accel), encoders_raw=np.array(enc))
        return s


class PolicyController:
    """The simulate() controller interface around the policy and its firmware."""

    def __init__(self, p, gains, library=None, actor=None):
        self.classical = Classical(p, gains, library)
        self.actor, self.gains = actor, gains
        cfg, command = DriveActionCfg(), DriveCommandCfg(resampling_time_range=(1.0, 1.0))
        self.cfg = cfg
        self.table = posture_table(p, LABELS)
        self.wheel_radius = p["geometry"]["wheel_radius"]
        self.wheel_track = p["geometry"]["wheel_track"]
        hub = p["physics"]["hub_motor"]
        self.wheel_limit = hub["torque_constant_Nm_A"] * hub["peak_current_A"] / cfg.wheel_ki
        self.step_limit = np.array([command.acceleration, math.inf, command.posture_rate])
        self.policy_ticks = 4  # 20 ms policy step over the 5 ms control tick
        self.feedforward = np.zeros(4)
        self.action = np.zeros(6)
        self.value = np.array([0.0, 0.0, 40.0])
        self.leg_target = self.nominal(40.0)
        self.leg_output = self.leg_target.copy()
        self.leg_filter_tau, self.leg_filter_band = LEG_FILTER_TAU_S, LEG_FILTER_BAND_RAD
        self.wheel_target = np.zeros(2)
        self.wheel_integral = np.zeros(2)
        self.wheel_request = np.zeros(2)
        self.ticks = 0
        self.observations = []
        self.shaper = None
        fid = FIDELITY or {}
        self.command_loss = fid.get("command_loss", 0.0)
        self.wheel_scale = np.asarray(fid.get("wheel_encoder_scale", [1.0, 1.0]), float)
        self.loss_rng = np.random.default_rng(fid.get("seed", 0) + 7919)
        self.pending = None
        if SHAPER:
            from ..m16.shaper import Shaper

            self.shaper = Shaper(**SHAPER)
        PolicyController.latest = self  # for inspection after a run

    # The supervisor's state, read by simulate().
    @property
    def protection(self):
        return self.classical.protection

    @property
    def compensation_limit(self):
        moving = self.value[0] != 0
        return self.gains["max_command_acceleration"] if moving else 0.0

    def gate(self, s, elapsed_s):
        self.classical.gate(s, elapsed_s)

    def nominal(self, label):
        x = min(max(label, 30.0), 65.0) - 30.0
        i = min(math.floor(x), len(LABELS) - 2)
        w = x - i
        return self.table[i] * (1 - w) + self.table[i + 1] * w

    def label(self, qref):
        """The posture label whose references are nearest qref (interpolated)."""
        d = np.linalg.norm(self.table - np.asarray(qref), axis=1)
        i = int(np.argmin(d))
        j = i + 1 if i == 0 or (i < len(LABELS) - 1 and d[i + 1] < d[i - 1]) else i - 1
        a, b = sorted((i, j))
        seg = self.table[b] - self.table[a]
        t = float(np.clip(np.dot(np.asarray(qref) - self.table[a], seg) / np.dot(seg, seg), 0, 1))
        return float(LABELS[a] + t * (LABELS[b] - LABELS[a]))

    def observe(self, s):
        if "encoders_raw" not in s:
            # Ideal sensing hands the controller the physics state: no IMU to observe.
            raise ValueError("the policy observes the sensor model; ideal sensing has none")
        q = s["encoders_raw"][:4] - self.nominal(self.value[2])
        command = [self.value[0], self.value[1], (self.value[2] - 47.5) / 17.5]
        return np.r_[
            s["gyro_raw"], s["accel_raw"], q, s["qd"], s["wheel_speed"], self.action, command
        ]

    def command(self, s, velocity, yaw_rate, qref, qdref, dt):
        if self.classical.fall_settings is not None:
            self.classical.previous_request = np.r_[np.zeros(4), self.wheel_request]
            protective = self.classical.protect(s, dt)
            if self.classical.protection.latched:
                return protective, s["pitch"], self.value[0], s["yaw"]
        if self.ticks % self.policy_ticks == 0:
            previous = (self.leg_target, self.wheel_target)
            step_dt = self.policy_ticks * dt
            # The training command: rate limited at the policy step, then observed.
            if self.shaper is not None:
                velocity, yaw_rate = self.shaper.step(velocity, yaw_rate, step_dt)
            target = np.array([velocity, yaw_rate, self.label(qref)])
            self.value += np.clip(
                target - self.value, -self.step_limit * step_dt, self.step_limit * step_dt
            )
            obs = self.observe(s)
            self.action = self.actor(obs)
            self.observations.append(obs)
            cfg = self.cfg
            self.leg_target = self.nominal(self.value[2]) + cfg.leg_scale * self.action[:4]
            forward = self.value[0] / self.wheel_radius
            turn = self.value[1] * self.wheel_track / (2 * self.wheel_radius)
            rolling = np.array([forward - turn, forward + turn]) if cfg.wheel_feedforward else 0.0
            self.wheel_target = cfg.wheel_scale * self.action[4:] + rolling
            if self.command_loss and self.loss_rng.random() < self.command_loss:
                # The command frame is lost: the moteus keeps the old targets a tick.
                self.pending = (self.leg_target, self.wheel_target)
                self.leg_target, self.wheel_target = previous
        elif self.pending is not None:
            self.leg_target, self.wheel_target = self.pending
            self.pending = None
        self.filter_legs(dt)
        self.ticks += 1
        return np.r_[np.zeros(4), self.wheel_request], s["pitch"], self.value[0], s["yaw"]

    def filter_legs(self, dt):
        """One control tick of the leg target filter (firmware bipe_learned_legs)."""
        if self.leg_filter_tau <= 0:  # off: the targets exactly, as trained
            self.leg_output = np.array(self.leg_target, float)
            return
        a, band = dt / (self.leg_filter_tau + dt), self.leg_filter_band
        out = self.leg_output + a * (self.leg_target - self.leg_output)
        self.leg_output = np.clip(out, self.leg_target - band, self.leg_target + band)

    def leg_torques(self, s, qref, qdref, feedforward=None):
        if self.classical.protection.latched:
            return self.classical.leg_torques(s, qref, qdref, feedforward)
        return self.cfg.joint_kp * (self.leg_output - s["q"]) - self.cfg.joint_kd * s["qd"]

    def wheel_torques(self, d, dt):
        if self.classical.protection.latched:
            return self.classical.previous_request[4:]
        speed = np.array([d.joint(w).qvel[0] for w in WHEELS]) * self.wheel_scale
        error = self.wheel_target - speed
        cfg = self.cfg
        self.wheel_integral = np.clip(
            self.wheel_integral + error * dt, -self.wheel_limit, self.wheel_limit
        )
        self.wheel_request = cfg.wheel_kv * error + cfg.wheel_ki * self.wheel_integral
        return self.wheel_request


@contextlib.contextmanager
def classes(actor):
    """The firmware module's controller and estimator names point at the policy's."""
    fc.FirmwareController = partial(PolicyController, actor=actor)
    fc.FirmwareEstimator = PolicyEstimator
    try:
        yield
    finally:
        fc.FirmwareController, fc.FirmwareEstimator = Classical, ClassicalEstimator


class Counted:
    """An actor that counts its calls (the per-run record that the policy ran)."""

    def __init__(self, actor):
        self.actor, self.calls = actor, 0

    def __call__(self, obs):
        self.calls += 1
        return self.actor(obs)


@contextlib.contextmanager
def policy_in_loop(checkpoint, runners=("bipe.robustness", "bipe.m12.speed"), actor=None):
    """simulate(implementation="c") runs the policy instead of the classical controller.

    The runners' simulate() is wrapped so the swap holds only inside a simulation (the
    C/Python equivalence check before a batch keeps the classical classes). Process
    pool workers inherit the swap only when they are forked: under spawn or forkserver
    they re-import the runners and would silently run the classical controller, so the
    context refuses any other start method. Each run gets ``policy_steps``, the number
    of policy calls it made. ``actor`` replaces the checkpoint's numpy actor (e.g.
    :class:`CActor`)."""
    import importlib
    import multiprocessing

    from .. import moving_experiment

    if multiprocessing.get_start_method() != "fork":
        raise RuntimeError("policy_in_loop needs the fork start method for the runners' pools")
    counted = Counted(actor or Actor(checkpoint))

    def simulate(*args, **kwargs):
        start = counted.calls
        with classes(counted):
            run = moving_experiment.simulate(*args, **kwargs)
        run["policy_steps"] = counted.calls - start
        return run

    modules = [importlib.import_module(name) for name in runners]
    saved = [module.simulate for module in modules]
    for module in modules:
        module.simulate = simulate
    try:
        yield counted.actor
    finally:
        for module, original in zip(modules, saved):
            module.simulate = original
