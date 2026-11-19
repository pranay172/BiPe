"""Commands, observations, rewards, terminations and events of the M13 task."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import torch
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.utils.lab_api.math import quat_apply, quat_from_angle_axis, quat_mul

from ...m10.kinematics import posture_target

ROBOT = "robot"
ACTION = "drive"


def _drive(env):
    return env.action_manager.get_term(ACTION)


def _heading(env):
    return env.scene[ROBOT].data.heading_w


def forward_speed(env) -> torch.Tensor:
    v = env.scene[ROBOT].data.root_link_lin_vel_w
    h = _heading(env)
    return v[:, 0] * torch.cos(h) + v[:, 1] * torch.sin(h)


def yaw_rate(env) -> torch.Tensor:
    return env.scene[ROBOT].data.root_link_ang_vel_w[:, 2]


def pitch_roll(env) -> tuple[torch.Tensor, torch.Tensor]:
    g = env.scene[ROBOT].data.projected_gravity_b
    pitch = torch.asin(g[:, 0].clamp(-1, 1))
    roll = torch.atan2(-g[:, 1], -g[:, 2])
    return pitch, roll


def axle_depth(env) -> torch.Tensor:
    """Hip-to-axle height: chassis origin above the mean wheel centre, m."""
    robot = env.scene[ROBOT]
    ids = robot.find_bodies(("left_wheel", "right_wheel"))[0]
    wheels = robot.data.body_link_pos_w[:, ids, 2].mean(dim=1)
    return robot.data.root_link_pos_w[:, 2] - wheels


# Commands.


@dataclass(kw_only=True)
class DriveCommandCfg(CommandTermCfg):
    """Forward speed, yaw rate and posture label, with the speed rate limited."""

    speed_range: tuple[float, float] = (-0.5, 1.2)
    yaw_rate_range: tuple[float, float] = (-1.0, 1.0)
    posture_range: tuple[float, float] = (30.0, 65.0)
    standing_fraction: float = 0.15
    acceleration: float = 1.25
    posture_rate: float = 20.0
    """Posture label rate limit, per second."""
    corner_fraction: float = 0.0
    """Share of resamples at combined high speed and yaw rate (M15: the follower's
    curves, 70-100 % of both range ends with random signs)."""
    sit_fraction: float = 0.0
    """M16 sit/stand: share of resamples that ask to sit. Sitting is the posture label
    SIT_LABEL (outside the 30-65 drive range, so the contract's command slot carries it
    as a bit); it is set at once, not rate limited, with zero speed and yaw rate. Leaving
    it puts the label back at 40 and the usual rate limit takes over."""
    parameters: dict = field(default_factory=dict)

    def build(self, env):
        return DriveCommand(self, env)


SIT_LABEL = 100.0
"""M16 sit/stand: the posture label that asks to sit (normalised command 3.0)."""


class DriveCommand(CommandTerm):
    cfg: DriveCommandCfg

    def __init__(self, cfg: DriveCommandCfg, env):
        super().__init__(cfg, env)
        n, dev = self.num_envs, self.device
        self.target = torch.zeros(n, 3, device=dev)
        self.value = torch.zeros(n, 3, device=dev)
        self.value[:, 2] = 40.0
        self.target[:, 2] = 40.0
        labels = np.linspace(30, 65, 36)
        depths = [posture_target(float(x), cfg.parameters)[0] for x in labels]
        self.depth_table = torch.tensor(depths, dtype=torch.float, device=dev)
        self.metrics["speed_error"] = torch.zeros(n, device=dev)
        self.metrics["yaw_rate_error"] = torch.zeros(n, device=dev)
        self.since_switch = torch.zeros(n, device=dev)
        """Seconds since the sit/stand request last changed."""

    @property
    def command(self) -> torch.Tensor:
        return self.value

    def reset(self, env_ids):
        # Episodes start at rest in the 40 posture; the command ramps from there.
        self.value[env_ids] = torch.tensor([0.0, 0.0, 40.0], device=self.device)
        self.since_switch[env_ids] = 0.0
        return super().reset(env_ids)

    def sitting(self) -> torch.Tensor:
        """True where the command asks to sit."""
        return self.value[:, 2] >= SIT_LABEL

    def depth(self) -> torch.Tensor:
        """Commanded axle depth from the posture label, m."""
        x = self.value[:, 2].clamp(30, 65) - 30
        i = x.floor().long().clamp(0, 34)
        w = x - i
        return self.depth_table[i] * (1 - w) + self.depth_table[i + 1] * w

    def _resample_command(self, env_ids):
        k = len(env_ids)
        r = lambda lo_hi: torch.empty(k, device=self.device).uniform_(*lo_hi)
        self.target[env_ids, 0] = r(self.cfg.speed_range)
        self.target[env_ids, 1] = r(self.cfg.yaw_rate_range)
        self.target[env_ids, 2] = r(self.cfg.posture_range)
        if self.cfg.corner_fraction > 0:
            corner = env_ids[torch.rand(k, device=self.device) < self.cfg.corner_fraction]
            m = len(corner)
            u = lambda: torch.empty(m, device=self.device).uniform_(0.7, 1.0)
            forward = torch.rand(m, device=self.device) < 0.8
            ends = torch.where(forward, self.cfg.speed_range[1], self.cfg.speed_range[0])
            sign = torch.where(torch.rand(m, device=self.device) < 0.5, -1.0, 1.0)
            self.target[corner, 0] = ends * u()
            self.target[corner, 1] = sign * max(map(abs, self.cfg.yaw_rate_range)) * u()
        still = torch.rand(k, device=self.device) < self.cfg.standing_fraction
        self.target[env_ids[still], :2] = 0.0
        if self.cfg.sit_fraction > 0:
            sit = env_ids[torch.rand(k, device=self.device) < self.cfg.sit_fraction]
            self.target[sit] = torch.tensor([0.0, 0.0, SIT_LABEL], device=self.device)

    def _update_command(self, env_ids):
        dt = self._env.step_dt
        step = torch.tensor(
            [self.cfg.acceleration * dt, 1e9, self.cfg.posture_rate * dt], device=self.device
        )
        ids = slice(None) if env_ids is None else env_ids
        if self.cfg.sit_fraction > 0:
            want, was = self.target[ids, 2] >= SIT_LABEL, self.value[ids, 2] >= SIT_LABEL
            switched = want != was
            value = self.value[ids]
            value[want] = torch.tensor([0.0, 0.0, SIT_LABEL], device=self.device)
            value[~want & was, 2] = 40.0
            self.value[ids] = value
            self.since_switch[ids] = torch.where(switched, 0.0, self.since_switch[ids] + dt)
            if want.all():
                return
            change = (self.target[ids] - self.value[ids]).clamp(-step, step)
            self.value[ids] += torch.where(want.unsqueeze(-1), 0.0, change)
            return
        change = (self.target[ids] - self.value[ids]).clamp(-step, step)
        self.value[ids] += change

    def _update_metrics(self):
        dt = self._env.step_dt
        horizon = self._env.max_episode_length_s
        self.metrics["speed_error"] += (forward_speed(self._env) - self.value[:, 0]).abs() * dt / horizon
        self.metrics["yaw_rate_error"] += (yaw_rate(self._env) - self.value[:, 1]).abs() * dt / horizon


# Observations.


def sensors_delayed(env) -> torch.Tensor:
    """Gyro, accelerometer, leg encoders (relative to the posture), leg speeds and
    wheel speeds, at the per-world transport delay."""
    drive = _drive(env)
    drive.refill()
    delay = drive.transport_delay
    if drive.cfg.fidelity:
        # M18: per-observation jitter, and a lost newest packet leaves the one before.
        n = delay.shape[0]
        jitter = (torch.rand(n, device=env.device) * (drive.jitter_steps + 1)).long()
        lost = torch.rand(n, device=env.device) < drive.packet_loss
        step = getattr(drive, "packet_steps", 5)
        delay = (delay + jitter + lost.long() * step).clamp(max=39)  # firmware.HISTORY - 1
    row = drive.sample(delay)
    q = row[:, 6:10] - drive.nominal_q()
    return torch.cat([row[:, :6], q, row[:, 10:]], dim=-1)


def last_action(env) -> torch.Tensor:
    return env.action_manager.action


def drive_command(env) -> torch.Tensor:
    c = env.command_manager.get_command("drive")
    return torch.stack([c[:, 0], c[:, 1], (c[:, 2] - 47.5) / 17.5], dim=-1)


def privileged(env) -> torch.Tensor:
    """True state and the randomised parameters, for the critic."""
    robot, drive = env.scene[ROBOT], _drive(env)
    pitch, roll = pitch_roll(env)
    return torch.cat(
        [
            robot.data.root_link_lin_vel_b,
            robot.data.root_link_ang_vel_b,
            torch.stack([pitch, roll, axle_depth(env), forward_speed(env)], dim=-1),
            robot.data.joint_pos,
            robot.data.joint_vel * 0.05,
            drive.motor_torque,
            drive.wheel_integral * drive.cfg.wheel_ki,
            drive.transport_delay.float().unsqueeze(-1) / 20.0,
            drive.encoder_lag.float().unsqueeze(-1) / 5.0,
            drive.noise_scale,
            drive.hub_tau * 100.0,
            drive.torque_scale,
            *([drive.belt_params / drive.belt_scale] if drive.spring else []),
            *(
                [drive.hip_tau * 1000.0, drive.pack_V / 22.2, drive.sag_ohm * 10.0,
                 drive.imu_bias * torch.tensor([300.0] * 3 + [10.0] * 3, device=drive.device)]
                if drive.cfg.actuator_dr
                else []
            ),  # fmt: skip
        ],
        dim=-1,
    )


# Rewards.


def stand_still(env, std: float, threshold: float = 0.05) -> torch.Tensor:
    """With a stand-still command (speed and yaw rate below ``threshold``), a narrow
    kernel on the forward speed: the speed reward's width does not see a few cm/s of
    sway, which the full plant's 0.03 m/s recovery band fails."""
    c = env.command_manager.get_command("drive")
    still = (c[:, 0].abs() < threshold) & (c[:, 1].abs() < threshold)
    return still.float() * torch.exp(-(forward_speed(env) ** 2) / std**2)


def slope_deg(env) -> torch.Tensor:
    """The per-world ground slope (the tilt of gravity, mdp.slope), degrees."""
    down = env.scene[ROBOT].data.gravity_vec_w
    return torch.rad2deg(torch.acos((-down[:, 2]).clamp(-1, 1)))


def stand_still_slope(env, std: float, threshold: float = 0.05) -> torch.Tensor:
    """M15: the stand-still kernel scaled by the slope over 10 degrees, so holding on a
    steep slope counts more (fine-tune 1 crept 0.09 and 0.21 m on the speed suite's 8 and
    10 degree downhill holds, which settle held)."""
    return stand_still(env, std, threshold) * slope_deg(env) / 10.0


def pitch_excess(env, limit_deg: float) -> torch.Tensor:
    """M15: squared body pitch past ``limit_deg``, rad^2 (the speed suite's 15 degree
    peak pitch; fine-tune 1 reached 15.1 at 1.2 m/s)."""
    pitch, _ = pitch_roll(env)
    return (pitch.abs() - math.radians(limit_deg)).clamp(min=0) ** 2


def com_lean(env, contact: bool = False) -> torch.Tensor:
    """Lean of the whole-body COM from the axle along the heading, rad: the full
    plant's pitch error (pitch less the M10 equilibrium, which puts the COM over the
    axle). With ``contact`` it is taken from the tyre contact instead, the wheel radius
    below the axle along the plane's normal, which reads about 1.5 times smaller for
    the same COM offset. The angle is from the true vertical, so it holds on slopes."""
    robot = env.scene[ROBOT]
    com = robot.data.data.subtree_com[:, robot.indexing.root_body_id]
    ids = robot.find_bodies(("left_wheel", "right_wheel"))[0]
    base = robot.data.body_link_pos_w[:, ids].mean(dim=1)
    if contact:
        base[:, 2] -= _drive(env).wheel_radius
    up = -robot.data.gravity_vec_w
    h = _heading(env)
    forward = torch.stack([torch.cos(h), torch.sin(h), torch.zeros_like(h)], dim=-1)
    forward = forward - (forward * up).sum(-1, keepdim=True) * up
    forward = forward / forward.norm(dim=-1, keepdim=True)
    d = com - base
    return torch.atan2((d * forward).sum(-1), (d * up).sum(-1))


def rest_lean(env, std: float, threshold: float = 0.05) -> torch.Tensor:
    """With a stand-still command, a kernel on the COM lean: stopping the wheels by
    holding a lean fails the full plant's 2 degree recovery band (25 October)."""
    c = env.command_manager.get_command("drive")
    still = (c[:, 0].abs() < threshold) & (c[:, 1].abs() < threshold)
    return still.float() * torch.exp(-(com_lean(env, contact=True) ** 2) / std**2)


def track_speed(env, std: float) -> torch.Tensor:
    c = env.command_manager.get_command("drive")
    return torch.exp(-((forward_speed(env) - c[:, 0]) ** 2) / std**2)


def track_yaw_rate(env, std: float) -> torch.Tensor:
    c = env.command_manager.get_command("drive")
    return torch.exp(-((yaw_rate(env) - c[:, 1]) ** 2) / std**2)


def yaw_rate_error(env) -> torch.Tensor:
    """Squared yaw-rate error: keeps a gradient where the tracking kernel is flat."""
    c = env.command_manager.get_command("drive")
    return (yaw_rate(env) - c[:, 1]) ** 2


def track_posture(env, std: float) -> torch.Tensor:
    depth = env.command_manager.get_term("drive").depth()
    return torch.exp(-((axle_depth(env) - depth) ** 2) / std**2)


def posture_error(env, scale: float) -> torch.Tensor:
    """Squared axle-depth error in units of ``scale``: a gradient across the posture range."""
    depth = env.command_manager.get_term("drive").depth()
    return ((axle_depth(env) - depth) / scale) ** 2


def level_body(env, std: float) -> torch.Tensor:
    pitch, roll = pitch_roll(env)
    return torch.exp(-(pitch**2 + roll**2) / std**2)


def action_rate(env) -> torch.Tensor:
    am = env.action_manager
    return ((am.action - am.prev_action) ** 2).sum(dim=-1)


def leg_torque(env) -> torch.Tensor:
    return (_drive(env).output_torque[:, :4] ** 2).sum(dim=-1)


def wheel_torque(env) -> torch.Tensor:
    return (_drive(env).output_torque[:, 4:] ** 2).sum(dim=-1)


def motor_power(env) -> torch.Tensor:
    drive, robot = _drive(env), env.scene[ROBOT]
    speed = robot.data.joint_vel[:, drive.driven]
    return (drive.output_torque * speed).clamp(min=0).sum(dim=-1)


def leg_limits(env, margin: float) -> torch.Tensor:
    robot, drive = env.scene[ROBOT], _drive(env)
    q = robot.data.joint_pos[:, drive.leg]
    lim = robot.data.soft_joint_pos_limits[:, drive.leg]
    low = (lim[..., 0] + margin - q).clamp(min=0)
    high = (q - lim[..., 1] + margin).clamp(min=0)
    return (low + high).sum(dim=-1)


# Terminations.


def tipped(env, limit_deg: float, roll_limit_deg: float | None = None) -> torch.Tensor:
    pitch, roll = pitch_roll(env)
    limit = math.radians(limit_deg)
    roll_limit = limit if roll_limit_deg is None else math.radians(roll_limit_deg)
    return (pitch.abs() > limit) | (roll.abs() > roll_limit)


def rolled(env, limit_deg: float) -> torch.Tensor:
    """Roll past ``limit_deg`` from the true vertical (the full plant's 10 degree fall)."""
    _, roll = pitch_roll(env)
    return roll.abs() > math.radians(limit_deg)


def leaned(env, limit_deg: float) -> torch.Tensor:
    """COM lean past ``limit_deg`` from the axle: the full plant's 20 degree pitch fall
    is on the pitch error (pitch less the M10 equilibrium), which a policy that brakes
    by swinging its legs reaches with the chassis still level."""
    return com_lean(env).abs() > math.radians(limit_deg)


def joint_limit(env, tolerance: float = 0.001) -> torch.Tensor:
    """A leg joint (hip or knee drive) past its range, as the full plant fails it."""
    robot, drive = env.scene[ROBOT], _drive(env)
    q = robot.data.joint_pos[:, drive.leg]
    lim = robot.data.joint_pos_limits[:, drive.leg]
    return ((q < lim[..., 0] - tolerance) | (q > lim[..., 1] + tolerance)).any(dim=-1)


def side_tilt(env) -> torch.Tensor:
    """Sideways tilt, rad: the body y axis out of the horizontal. Unlike the Euler roll
    of pitch_roll, it stays defined with the robot on its face or back (M15 get-up)."""
    g = env.scene[ROBOT].data.projected_gravity_b
    return torch.asin((-g[:, 1]).clamp(-1, 1))


def side_rolled(env, limit_deg: float) -> torch.Tensor:
    """M15 get-up: tilted sideways past ``limit_deg`` (towards a roll-over, which no
    sagittal leg motion undoes: docs/studies/getup-feasibility.md)."""
    return side_tilt(env).abs() > math.radians(limit_deg)


def handover(env, sensor_name: str, lean_deg: float = 10.0, roll_deg: float = 10.0,
             rate: float = 1.0) -> torch.Tensor:  # fmt: skip
    """M15 get-up: the get-up study's handover test, 1 or 0: no shell contact, the body
    pitch error and the sideways tilt inside 10 degrees, pitch rate inside 1 rad/s. On
    flat ground with the shell clear, the tyres carry the robot."""
    clear = ~shell_contact(env, sensor_name)
    # Body pitch from the nominal 40 posture's equilibrium (0 to 1e-15 deg), as in the
    # study: the COM lean alone is met by any pose with the COM over the axle.
    lean = pitch_roll(env)[0].abs() < math.radians(lean_deg)
    side = side_tilt(env).abs() < math.radians(roll_deg)
    still = env.scene[ROBOT].data.root_link_ang_vel_b[:, 1].abs() < rate
    return (clear & lean & side & still).float()


def handover_held(env, sensor_name: str, hold_s: float = 0.5, **kw) -> torch.Tensor:
    """M16 get-up: the handover test counted only once it has held for ``hold_s``. getup3
    learned to touch the test for an instant (handovers at 0.6 s, then the drive policy
    tipped forward); a held test is what the firmware will hand over on."""
    now = handover(env, sensor_name, **kw) > 0
    held = env.__dict__.setdefault("_handover_held", torch.zeros(env.num_envs, device=env.device))
    held[:] = torch.where(now, held + env.step_dt, torch.zeros_like(held))
    held[env.episode_length_buf <= 1] = 0.0
    return (held >= hold_s).float()


def upright(env, std: float) -> torch.Tensor:
    """M15 get-up: a wide kernel on the body pitch and the COM lean (both from the
    nominal posture's balance), a gradient out of the fallen states."""
    lean = torch.maximum(pitch_roll(env)[0].abs(), com_lean(env).abs())
    return torch.exp(-(lean**2) / std**2)


def shell_contact(env, sensor_name: str) -> torch.Tensor:
    found = env.scene[sensor_name].data.found
    return found.reshape(found.shape[0], -1).any(dim=-1)


# M16 sit/stand.


def _sit_command(env):
    return env.command_manager.get_term("drive")


def standing_only(env, term, **params) -> torch.Tensor:
    """A drive reward term counted only while the command asks to stand."""
    return term(env, **params) * (~_sit_command(env).sitting()).float()


def standing_failure(env, term, grace_s: float = 3.0, **params) -> torch.Tensor:
    """A drive termination that applies only once a stand request is ``grace_s`` old, so
    sitting and rising from a sit are not failures."""
    c = _sit_command(env)
    return term(env, **params) & ~c.sitting() & (c.since_switch > grace_s)


def seated(env, sensor_name: str, level_deg: float = 20.0, std: float = 0.2) -> torch.Tensor:
    """While asked to sit: resting on the shell and tyres near level (the fallen-state
    library's ``sit``), with a kernel on the body's speed so it settles."""
    robot = env.scene[ROBOT]
    rest = shell_contact(env, sensor_name) & (pitch_roll(env)[0].abs() < math.radians(level_deg))
    speed = robot.data.root_link_lin_vel_b.norm(dim=-1) + 0.1 * robot.data.root_link_ang_vel_b.norm(dim=-1)
    return _sit_command(env).sitting().float() * rest.float() * torch.exp(-(speed**2) / std**2)


def sit_landing(env) -> torch.Tensor:
    """While asked to sit: the vertical speed squared, so the robot lowers itself onto the
    shell instead of dropping."""
    vz = env.scene[ROBOT].data.root_link_lin_vel_w[:, 2]
    return _sit_command(env).sitting().float() * vz**2


def sit_torque(env) -> torch.Tensor:
    """While asked to sit: the leg torque squared (motors to low torque once down)."""
    return _sit_command(env).sitting().float() * leg_torque(env)


# Events.


def reset_fallen(
    env, env_ids, path: str, joint_names: tuple[str, ...], yaw: bool = True,
    class_weights: dict[str, float] | None = None, flat_deg: float | None = None,
    fraction: float = 1.0,
):  # fmt: skip
    """M15 get-up: start each episode in a fallen state drawn from the library made by
    bipe.m15.fallen (full qpos in the training plant's joint order, ``joint_names``),
    at rest, at a random heading. Runs after the base and joint resets. With
    ``class_weights`` ({class name: weight}) each class is drawn with that share
    whatever its count in the library (unnamed classes weigh 1). ``flat_deg`` splits the
    leans past that pitch into two more classes, ``flat_front`` and ``flat_back`` (M16:
    lying flat on the face or back, which the M15 get-up policy never lifted).
    ``fraction`` < 1 starts only that share of the episodes fallen (M16 sit/stand: the
    rest start standing)."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    if fraction < 1.0:
        env_ids = env_ids[torch.rand(len(env_ids), device=env.device) < fraction]
        if len(env_ids) == 0:
            return
    cache = env.__dict__.setdefault("_fallen_states", {})
    if path not in cache:
        lib = np.load(path)
        cls, names = lib["cls"].astype(int), [str(n) for n in lib["classes"]]
        if flat_deg is not None:
            names += ["flat_front", "flat_back"]
            pitch = lib["pitch_deg"]
            cls = np.where(pitch > flat_deg, len(names) - 2, np.where(pitch < -flat_deg, len(names) - 1, cls))
        w = np.array([(class_weights or {}).get(names[c], 1.0) for c in cls], float)
        w /= np.bincount(cls)[cls] if class_weights else 1.0  # each class its share
        cache[path] = (
            torch.tensor(lib["qpos"], dtype=torch.float, device=env.device),
            torch.tensor(w / w.sum(), dtype=torch.float, device=env.device),
        )
    states, prob = cache[path]
    k, dev = len(env_ids), env.device
    q = states[torch.multinomial(prob, k, replacement=True)]
    robot = env.scene[ROBOT]
    pos = q[:, :3].clone()
    pos[:, :2] = 0.0
    pos += env.scene.env_origins[env_ids]
    quat = q[:, 3:7]
    if yaw:
        a = torch.empty(k, device=dev).uniform_(-math.pi, math.pi)
        z = torch.stack([torch.cos(a / 2), torch.zeros_like(a), torch.zeros_like(a), torch.sin(a / 2)], -1)
        w1, x1, y1, z1 = z.unbind(-1)
        w2, x2, y2, z2 = quat.unbind(-1)
        quat = torch.stack([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ], -1)  # fmt: skip
    robot.write_root_state_to_sim(torch.cat([pos, quat, torch.zeros(k, 6, device=dev)], -1), env_ids=env_ids)
    order = [joint_names.index(n) for n in robot.joint_names]
    joints = q[:, 7:][:, order]
    robot.write_joint_state_to_sim(joints, torch.zeros_like(joints), env_ids=env_ids)


def randomise_drive(
    env,
    env_ids,
    transport_delay_ms: tuple[int, int] = (5, 20),
    encoder_lag_ms: tuple[int, int] = (0, 5),
    encoder_offset_rad: float = math.radians(0.5),
    noise_scale: tuple[float, float] = (0.5, 2.0),
    hub_lag_s: tuple[float, float] = (0.0008, 0.025),
    torque_scale: tuple[float, float] = (0.9, 1.1),
    hip_belt_Nm_rad: tuple[float, float] = (118.9, 713.3),
    knee_belt_Nm_rad: tuple[float, float] = (23.5, 140.9),
    hip_lash_deg: tuple[float, float] = (0.1125, 0.45),
    knee_lash_deg: tuple[float, float] = (0.225, 0.9),
    actuator_torque_scale: tuple[float, float] = (0.8, 1.2),
    hip_lag_scale: tuple[float, float] = (1.0, 2.0),
    friction_scale: tuple[float, float] = (0.5, 1.5),
    pack_V: tuple[float, float] = (19.8, 25.2),
    sag_scale: tuple[float, float] = (1.0, 2.0),
    gyro_bias_rad_s: float = 0.001,
    accel_bias_m_s2: float = 0.1,
    ramp_steps: int = 0,
    fidelity_ranges: dict | None = None,
):
    """Per-world firmware and sensor parameters, drawn at each reset (1 ms steps).

    With M14's belts also each belt's stiffness (log-uniform) and lash (uniform), over
    the M10 belt acceptance ranges.

    With the actuator randomisation (M14 phase B) the torque scale (torque constant)
    takes ``actuator_torque_scale``, and each world also draws the RO40 FOC lag, every
    joint's Coulomb friction and viscous damping, the pack's open-circuit voltage and
    sag resistance, and a gyro and accelerometer bias.

    ``ramp_steps`` (env steps) ramps the belt and actuator draws in from the nominal
    plant: each draw is nominal + t (draw - nominal), t rising linearly to 1 (the M14
    recipe)."""
    drive = _drive(env)
    t = 1.0 if ramp_steps <= 0 else min(1.0, env.common_step_counter / ramp_steps)
    toward = lambda nominal, x: nominal + t * (x - nominal)
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    k, dev = len(env_ids), env.device
    steps = lambda r: torch.randint(r[0], r[1] + 1, (k,), device=dev)
    uniform = lambda r, *shape: torch.empty(k, *shape, device=dev).uniform_(*r)
    drive.transport_delay[env_ids] = steps(transport_delay_ms)
    drive.encoder_lag[env_ids] = steps(encoder_lag_ms)
    drive.encoder_offset[env_ids] = uniform((-encoder_offset_rad, encoder_offset_rad), 4)
    drive.noise_scale[env_ids] = uniform(noise_scale, 1)
    lag = torch.exp(uniform((math.log(hub_lag_s[0]), math.log(hub_lag_s[1]))))
    drive.hub_tau[env_ids] = lag.unsqueeze(-1).expand(k, 2)
    drive.torque_scale[env_ids] = uniform(torque_scale, 6)
    if drive.cfg.actuator_dr:
        base = drive.torque_scale[env_ids]
        drive.torque_scale[env_ids] = toward(base, uniform(actuator_torque_scale, 6))
        lag = drive.phys["hip_motor"]["FOC_time_constant_s"]
        drive.hip_tau[env_ids] = lag * toward(1.0, uniform(hip_lag_scale, 4))
        nv = drive.friction_scale.shape[-1]
        drive.set_friction(
            env_ids, toward(1.0, uniform(friction_scale, nv)), toward(1.0, uniform(friction_scale, nv))
        )
        nominal_V = drive.phys["hub_motor"]["drive_voltage_limit_V"] / 0.95
        drive.pack_V[env_ids] = toward(nominal_V, uniform(pack_V, 1))
        drive.bus_V[env_ids] = drive.pack_V[env_ids]
        drive.sag_ohm[env_ids] = 0.5 / 6.63 * toward(1.0, uniform(sag_scale, 1))
        bias = torch.tensor([gyro_bias_rad_s] * 3 + [accel_bias_m_s2] * 3, device=dev)
        drive.imu_bias[env_ids] = bias * toward(0.0, uniform((-1.0, 1.0), 6))
    if drive.cfg.fidelity:
        _randomise_fidelity(drive, env_ids, uniform, fidelity_ranges)
    if drive.spring:
        log = lambda r: uniform((math.log(r[0]), math.log(r[1])))
        nominal = drive.belt_scale
        drive.set_belts(
            env_ids,
            torch.stack(
                [torch.exp(toward(nominal[0].log(), log(hip_belt_Nm_rad))),
                 torch.exp(toward(nominal[1].log(), log(knee_belt_Nm_rad))),
                 toward(nominal[2], uniform(hip_lash_deg)),
                 toward(nominal[3], uniform(knee_lash_deg))], dim=-1,
            ),  # fmt: skip
        )


def _randomise_fidelity(drive, env_ids, uniform, ranges=None):
    """M18 per-world draws (bipe.m18.fidelity.RANGES, overridden by ``ranges``)."""
    from ...m18 import fidelity

    r = {**fidelity.RANGES, **(ranges or {})}
    k, dev = len(env_ids), drive.device
    sym = lambda x, *shape: uniform((-x, x), *shape)
    drive.cog_amp[env_ids] = drive.cog_rated * uniform(r["cogging_fraction"], 1)
    drive.cog_phase[env_ids] = uniform((0.0, 2 * math.pi), 6)
    drive.stiction_ratio[env_ids] = uniform(r["stiction_ratio"], 1)
    winding = uniform(r["winding_C"], 1)
    drive.winding_C[env_ids] = winding
    drive.torque_scale[env_ids] = drive.torque_scale[env_ids] * fidelity.kt_scale(winding)
    drive.wheel_enc_scale[env_ids] = 1.0 / uniform(r["wheel_radius_ratio"], 2)
    roll, pitch = torch.deg2rad(sym(r["imu_mount_deg"], 2)).unbind(-1)
    cr, sr, cp, sp = roll.cos(), roll.sin(), pitch.cos(), pitch.sin()
    zero, one = torch.zeros_like(roll), torch.ones_like(roll)
    rx = torch.stack([one, zero, zero, zero, cr, sr, zero, -sr, cr], -1).view(k, 3, 3)
    ry = torch.stack([cp, zero, -sp, zero, one, zero, sp, zero, cp], -1).view(k, 3, 3)
    drive.imu_mount[env_ids] = ry @ rx
    e = torch.diag_embed(sym(r["accel_scale"], 3))
    off = sym(r["accel_cross"], 3, 3) * (1 - torch.eye(3, device=dev))
    drive.imu_accel[env_ids] = torch.eye(3, device=dev) + e + off
    drive.gyro_walk_std[env_ids] = uniform(r["gyro_walk"], 1)
    drive.gyro_walk[env_ids] = 0.0
    drive.encoder_offset[env_ids] = torch.deg2rad(sym(r["encoder_offset_deg"], 4))
    lo, hi = r["jitter_ms"]
    drive.jitter_steps[env_ids] = torch.randint(int(lo), int(hi) + 1, (k,), device=dev)
    drive.packet_loss[env_ids] = uniform(r["packet_loss"])
    drive.command_loss[env_ids] = uniform(r["command_loss"])


def floor_softness(env, env_ids, stiffness: tuple[float, float] = (0.3, 1.0), base_tc: float = 0.002):
    """M18 soft floor: the tire contact pairs' solref time constant per world, for a
    contact stiffness scale s (time constant base / sqrt(s), mass-scaled reference)."""
    asset = env.scene[ROBOT]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    pairs = asset.indexing.pair_ids
    s = torch.empty(len(env_ids), device=env.device).uniform_(*stiffness)
    tc = base_tc / s.sqrt()
    solref = env.sim.model.pair_solref
    solref[env_ids[:, None], pairs[None, :], 0] = tc[:, None]


def slope(
    env, env_ids, max_deg: float = 10.0, flat_fraction: float = 0.3, upright: bool = True,
    steep: float = 1.0, along_heading: bool = False,
):  # fmt: skip
    """A ground slope of up to ``max_deg`` in a random direction, as a tilt of gravity.

    The robot touches nothing but the ground plane, so tilting gravity is the same
    physics as tilting the plane. The accelerometer and the bias forces read the
    per-world gravity; ``gravity_vec_w`` follows it, so pitch, roll and the tipping
    limit are taken from the true vertical, as on a real slope. Speeds and heading
    stay in the plane's frame.

    With ``upright`` the robot starts balanced on the slope, as a supervised start
    would leave it: the reset pose is turned about the axle by the slope along its
    heading. The slope across the heading stays as roll. Runs after the base reset.

    ``steep`` below 1 draws steep slopes more often: the tilt is max_deg u^steep.
    ``along_heading`` points the slope up or down the reset heading, as the full plant's
    slope cases do (a cross slope near the roll fault's 10 degrees trips it at rest)."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    k, dev = len(env_ids), env.device
    tilt = math.radians(max_deg) * torch.rand(k, device=dev) ** steep
    tilt[torch.rand(k, device=dev) < flat_fraction] = 0.0
    azimuth = torch.empty(k, device=dev).uniform_(-math.pi, math.pi)
    if along_heading:
        env.sim.forward()
        flip = math.pi * (torch.rand(k, device=dev) < 0.5).float()
        azimuth = _heading(env)[env_ids] + flip
    down = torch.stack(
        [torch.sin(tilt) * torch.cos(azimuth), torch.sin(tilt) * torch.sin(azimuth), -torch.cos(tilt)],
        dim=-1,
    )
    gravity = env.sim.model.opt.gravity
    g = float(env.sim.mj_model.opt.gravity[2])
    gravity[env_ids] = -g * down
    robot = env.scene[ROBOT]
    robot.data.gravity_vec_w[env_ids] = down
    if upright:
        env.sim.forward()
        h = _heading(env)[env_ids]
        forward = torch.stack([torch.cos(h), torch.sin(h), torch.zeros_like(h)], dim=-1)
        lateral = torch.stack([-torch.sin(h), torch.cos(h), torch.zeros_like(h)], dim=-1)
        lean = torch.asin(-(down * forward).sum(dim=-1))
        turn = quat_from_angle_axis(lean, lateral)
        ids = robot.find_bodies(("left_wheel", "right_wheel"))[0]
        axle = robot.data.body_link_pos_w[env_ids][:, ids].mean(dim=1)
        pose = robot.data.root_link_pose_w[env_ids].clone()
        pose[:, :3] = axle + quat_apply(turn, pose[:, :3] - axle)
        pose[:, 3:7] = quat_mul(turn, pose[:, 3:7])
        robot.write_root_link_pose_to_sim(pose, env_ids=env_ids)


def push(env, env_ids, impulse_Ns: tuple[float, float] = (0.0, 2.5), forward: float = 0.5):
    """A horizontal push as a change of base velocity (impulse over the robot's mass),
    along the heading with probability ``forward``, else against it."""
    robot = env.scene[ROBOT]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    k = len(env_ids)
    mass = float(env.sim.mj_model.body_subtreemass[robot.indexing.root_body_id])
    size = torch.empty(k, device=env.device).uniform_(*impulse_Ns) / mass
    sign = torch.where(torch.rand(k, device=env.device) < forward, 1.0, -1.0)
    h = _heading(env)[env_ids]
    vel = robot.data.root_link_vel_w[env_ids].clone()
    vel[:, 0] += sign * size * torch.cos(h)
    vel[:, 1] += sign * size * torch.sin(h)
    robot.write_root_link_velocity_to_sim(vel, env_ids=env_ids)


def push_force(
    env, env_ids, impulse_Ns: tuple[float, float] = (0.0, 2.5), forward: float = 0.5,
    duration_s: float = 0.1,
):  # fmt: skip
    """A horizontal push as the full plant applies it (bipe.moving_experiment): a
    force on the chassis for ``duration_s``, along the heading with probability
    ``forward``, else against it. ``push`` steps the base velocity instead, which
    moves the wheels with the chassis and barely pitches the robot."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    k = len(env_ids)
    size = torch.empty(k, device=env.device).uniform_(*impulse_Ns) / duration_s
    sign = torch.where(torch.rand(k, device=env.device) < forward, 1.0, -1.0)
    h = _heading(env)[env_ids]
    drive = _drive(env)
    drive.push_force[env_ids, 0, 0] = sign * size * torch.cos(h)
    drive.push_force[env_ids, 0, 1] = sign * size * torch.sin(h)
    drive.push_force[env_ids, 0, 2] = 0.0
    drive.push_steps[env_ids] = round(duration_s / env.physics_dt)

