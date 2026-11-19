"""The robot's side of the policy interface, run every physics step.

``DriveAction`` is an mjlab action term. The policy's six actions (four leg joint
targets as offsets from the commanded posture, two wheel speed targets) are held
for the 20 ms policy step. Every 1 ms physics step the term:

- samples the sensors (gyro, accelerometer, joint encoders, wheel speeds) into a
  ring buffer, with the per-world encoder offset and noise;
- adds the kinematic wheel speeds for the commanded speed and yaw rate to the
  policy's wheel actions, so the policy learns a residual around rolling;
- runs the firmware joint law: the leg PD of the classical controller (joint_kp,
  joint_kd, on the actual shin angle a2 = hip + shin) on the joint encoder
  readings, delayed by the per-world encoder lag, and a PI wheel speed loop (the
  integral clamped at the GL40 peak torque) on the current wheel speed, as the
  hub drive closes it on its own encoder;
- runs the RO40 and GL40 models (bipe.m13.drives, in torch) and writes the motor
  torques to the MJCF motors and the belt loss to the leg hinges' frictionloss.

With M14's belts (training.drives hip_belt "spring", knee_lash "deadband") the hip
motors drive the thigh rotor hinges, whose encoders (rotor angle / ratio) are the
hip readings and whose speeds are the motor speeds; the belts' stiffness and lash
are per world (mdp.randomise_drive), as tendon fields.

Observations read the ring buffer at the per-world transport delay, so the delay is
resolved at 1 ms, not at the policy step.

With ``sensors`` set (the robot's sensor model, config/sensors-m10.json) the readings
are the robot's channels instead of exact instantaneous states, as bipe.sensing
builds them for the full plant:
- the gyro and accelerometer pass their anti-alias filters every physics step and
  are sampled with bias and noise into a packet every control period (5 ms), held
  in between;
- the encoders are quantised (joint and wheel counts per revolution) with their
  periodic error, and the knee encoder reads the knee-drive hinge (the motor side
  of the knee belt) instead of the shin;
- the packet's leg and wheel speeds are the estimator's: finite differences of
  consecutive packets' encoders through the joint and wheel velocity filters;
- the leg PD reads the local encoder feedback: quantised encoders every physics
  step with their filtered finite-difference rates, with no encoder lag (the
  randomised lag stood in for belt lash on the shin reading; on the knee-drive
  hinge it makes the servo chatter).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import torch

from mjlab.managers.action_manager import ActionTerm, ActionTermCfg
from mjlab.sim.randomization import expand_model_fields

from .. import drives
from .. import plant as tp
from ..plant import JOINTS, WHEELS
from ...m10.kinematics import posture

SHINS = ("left_shin_joint", "right_shin_joint")
BELTS = ("left_knee_belt", "right_knee_belt")
HISTORY = 40  # physics steps kept for the delays (covers 0-39 ms)


def posture_table(p, labels):
    """Leg joint references [a1, a2] x 2 at posture labels (M10 kinematics)."""
    return np.array([posture(float(x), p) for x in labels])


@dataclass(kw_only=True)
class DriveActionCfg(ActionTermCfg):
    entity_name: str = "robot"
    parameters: dict = field(default_factory=dict)
    """The m13 plant parameters (bipe.parameters.load_parameters(profile='m13'))."""
    joint_kp: float = 8.0
    joint_kd: float = 0.05
    wheel_kv: float = 0.05
    """Wheel speed loop gain, N m per rad/s."""
    wheel_ki: float = 0.5
    """Wheel speed loop integral gain, N m per rad (zero: P only). Without it the loop
    settles short of its target under rolling and scrub losses."""
    leg_scale: float = 0.5
    """Leg action scale, rad per unit action."""
    wheel_scale: float = 40.0
    """Wheel action scale, rad/s per unit action."""
    wheel_feedforward: bool = True
    """Add the commanded speed and yaw rate's wheel speeds to the wheel targets."""
    command_name: str = "drive"
    sensors: dict | None = None
    """The robot's sensor model (config/sensors-m10.json); None reads exact states."""
    actuator_dr: bool = False
    """M14 phase B: per-world RO40 FOC lag, joint friction and damping, IMU bias and
    pack voltage with sag (the GL40 voltage limit and RO40 speed follow the bus),
    drawn by mdp.randomise_drive."""
    fidelity: bool = False
    """M18 (bipe.m18.fidelity): per-world cogging, stiction, winding temperature,
    wheel radius (as a hub encoder scale), IMU mounting tilt and accelerometer
    scale and cross-axis errors, gyro bias walk, packet jitter and loss and lost
    commands, drawn by mdp.randomise_drive (``fidelity_ranges``)."""

    def build(self, env):
        return DriveAction(self, env)


class DriveAction(ActionTerm):
    cfg: DriveActionCfg

    def __init__(self, cfg: DriveActionCfg, env):
        super().__init__(cfg, env)
        p = cfg.parameters
        self.p, self.phys = p, p["physics"]
        self.ratio = p["joint_drive"]["ratio"]
        self.loss = 1 - self.phys["hip_motor"]["efficiency_screen"]
        robot, n, dev = self._entity, self.num_envs, self.device
        names = list(robot.joint_names)
        self.leg = torch.tensor([names.index(j) for j in JOINTS], device=dev)
        self.wheel = torch.tensor([names.index(j) for j in WHEELS], device=dev)
        self.shin = torch.tensor([names.index(j) for j in SHINS], device=dev)
        mj = env.sim.mj_model
        prefix = cfg.entity_name + "/"
        dof = lambda j: int(mj.joint(prefix + j).dofadr[0])
        self.leg_dof = torch.tensor([dof(j) for j in JOINTS], device=dev)
        self.shin_dof = torch.tensor([dof(j) for j in SHINS], device=dev)
        self.belts = torch.tensor([mj.tendon(prefix + b).id for b in BELTS], device=dev)
        self.knee_k = torch.tensor(mj.tendon_stiffness[self.belts.cpu()], dtype=torch.float, device=dev)
        self.knee_c = torch.tensor(mj.tendon_damping[self.belts.cpu()], dtype=torch.float, device=dev)
        # Motor-side joints: the thigh rotors for the hips with spring belts (motor
        # speed = rotor speed, hip encoder = rotor / ratio), else the leg hinges.
        self.spring = tp.hip_spring(p)
        self.motor = self.leg.clone()
        self.motor_scale = torch.full((4,), float(self.ratio), device=dev)
        if self.spring:
            rotors = [f"{s}_thigh_rotor_joint" for s in tp.SIDES]
            self.motor[[0, 2]] = torch.tensor([names.index(j) for j in rotors], device=dev)
            self.motor_scale[[0, 2]] = 1.0
            self.hip_belts = torch.tensor(
                [mj.tendon(prefix + f"{s}_thigh_hip_belt").id for s in tp.SIDES], device=dev
            )
            self.rotor_dof = torch.tensor([dof(j) for j in rotors], device=dev)
            env.sim.expand_model_fields(
                ("tendon_stiffness", "tendon_damping", "tendon_lengthspring", "dof_armature")
            )
            b = self.phys["belts"]
            # Per-world belt parameters (hip k, knee k, hip lash, knee lash), nominal here.
            self.belt_params = torch.tensor(
                [[b["hip_stiffness_Nm_rad"], b["knee_stiffness_Nm_rad"], b["hip_lash_deg"],
                  b["knee_lash_deg"]]] * n, device=dev,
            )  # fmt: skip
            self.belt_scale = self.belt_params[0].clone()  # privileged obs: x nominal
        gyro, accel = mj.sensor(prefix + "gyro"), mj.sensor(prefix + "accelerometer")
        self.gyro_adr, self.accel_adr = int(gyro.adr[0]), int(accel.adr[0])
        self.driven = torch.cat([self.motor, self.wheel])
        g = p["geometry"]
        self.wheel_radius, self.wheel_track = g["wheel_radius"], g["wheel_track"]

        # Per-world gravity, so that slopes can tilt it (mdp.slope). The sim's own
        # expansion below clears the model cache and recaptures the graph.
        expand_model_fields(env.sim.wp_model.opt, n, ["gravity"])
        env.sim.expand_model_fields(("dof_frictionloss",))
        self.base_friction = env.sim.get_default_field("dof_frictionloss").clone()

        labels = np.linspace(30, 65, 36)
        self.labels = torch.tensor(labels, dtype=torch.float, device=dev)
        self.table = torch.tensor(posture_table(p, labels), dtype=torch.float, device=dev)

        self._raw = torch.zeros(n, 6, device=dev)
        self.leg_target = torch.zeros(n, 4, device=dev)
        self.wheel_target = torch.zeros(n, 2, device=dev)
        self.current = torch.zeros(n, 6, device=dev)
        self.wheel_integral = torch.zeros(n, 2, device=dev)
        hub = self.phys["hub_motor"]
        self.hub_peak = hub["torque_constant_Nm_A"] * hub["peak_current_A"]
        self.motor_torque = torch.zeros(n, 6, device=dev)
        self.output_torque = torch.zeros(n, 6, device=dev)
        # Sensor ring buffer: gyro 3, accel 3, q 4, qd 4, wheel speed 2.
        self.history = torch.zeros(HISTORY, n, 16, device=dev)
        self._worlds = torch.arange(n, device=dev)
        self.head = 0
        # Worlds reset since their last sample: their history is refilled from the
        # first reading after the reset (the robot was standing before the start).
        self.fresh = torch.ones(n, dtype=torch.bool, device=dev)
        # Per-world randomised parameters (set by the reset events).
        self.transport_delay = torch.full((n,), 5, dtype=torch.long, device=dev)
        self.encoder_lag = torch.zeros(n, dtype=torch.long, device=dev)
        self.encoder_offset = torch.zeros(n, 4, device=dev)
        self.noise_scale = torch.ones(n, 1, device=dev)
        self.torque_scale = torch.ones(n, 6, device=dev)
        self.hub_tau = torch.full(
            (n, 2), self.phys["hub_motor"]["FOC_time_constant_s"], device=dev
        )
        if cfg.actuator_dr:
            self._actuator_dr(env, n)
        if cfg.fidelity:
            if not cfg.actuator_dr:
                raise ValueError("fidelity builds on actuator_dr (per-world friction)")
            self._fidelity(env, n)
        # A push as the full plant applies it: a force on the chassis for a while
        # (mdp.push_force), not a step in the base velocity.
        self.chassis = robot.find_bodies(("chassis",))[0]
        self.push_force = torch.zeros(n, 1, 3, device=dev)
        self.push_steps = torch.zeros(n, dtype=torch.long, device=dev)
        sensors = cfg.sensors or p.get("sensor_noise", {})
        self.gyro_std = sensors.get("gyro_noise_std_rad_s", 0.003)
        self.accel_std = sensors.get("accel_noise_std_m_s2", 0.06)
        if cfg.sensors is not None:
            self._robot_sensing(cfg.sensors, n, env.physics_dt)

    def _actuator_dr(self, env, n):
        dev, hm, gl = self.device, self.phys["hip_motor"], self.phys["hub_motor"]
        self.hip_tau = torch.full((n, 4), hm["FOC_time_constant_s"], device=dev)
        self.imu_bias = torch.zeros(n, 6, device=dev)
        env.sim.expand_model_fields(("dof_damping",))
        self.base_damping = env.sim.get_default_field("dof_damping").clone()
        nv = self.base_friction.shape[-1]
        self.friction_scale = torch.ones(n, nv, device=dev)
        # The bus: the pack's open-circuit voltage less its sag (pack and harness
        # resistance x bus current). Nominal is the M11 low pack, where the GL40
        # voltage limit (0.95 x bus) was set; the RO40 no-load speed is taken at the
        # full pack (its catalog speed is KV140 at about 24 V).
        self.pack_V = torch.full((n, 1), gl["drive_voltage_limit_V"] / 0.95, device=dev)
        # 6 x 8 mOhm cells plus the 20/22 AWG harness: 0.50 V at 6.63 A (M11 phase D).
        self.sag_ohm = torch.full((n, 1), 0.5 / 6.63, device=dev)
        self.bus_V = self.pack_V.clone()
        self.hub_R = drives.hub_resistance(gl)
        self.full_pack_V = 25.2

    def _fidelity(self, env, n):
        """M18 per-world fidelity state, nominal (off) until randomise_drive draws it."""
        dev, hm, gl = self.device, self.phys["hip_motor"], self.phys["hub_motor"]
        # Rated torque per driven column, in the units the columns are applied in
        # (hip motor torque x ratio, hub torque).
        self.cog_rated = torch.tensor(
            [hm["continuous_torque_motor_Nm"] * self.ratio] * 4 + [gl["rated_torque_Nm"]] * 2,
            device=dev,
        )
        self.cog_amp = torch.zeros(n, 6, device=dev)
        self.cog_phase = torch.zeros(n, 6, device=dev)
        self.stiction_ratio = torch.ones(n, 1, device=dev)
        self.winding_C = torch.full((n, 1), gl["voltage_limit_winding_temperature_C"], device=dev)
        self.wheel_enc_scale = torch.ones(n, 2, device=dev)
        eye = torch.eye(3, device=dev).expand(n, 3, 3)
        self.imu_mount = eye.clone()
        self.imu_accel = eye.clone()
        self.gyro_walk_std = torch.zeros(n, 1, device=dev)
        self.gyro_walk = torch.zeros(n, 3, device=dev)
        self.jitter_steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.packet_loss = torch.zeros(n, device=dev)
        self.command_loss = torch.zeros(n, device=dev)
        self.hold_steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.leg_applied = torch.zeros(n, 4, device=dev)
        self.wheel_applied = torch.zeros(n, 2, device=dev)
        env.sim.expand_model_fields(("pair_solref",))  # mdp.floor_softness

    def _board(self, gyro, accel):
        """Body-frame gyro and accelerometer as the tilted, mis-scaled board reads them."""
        if not self.cfg.fidelity:
            return gyro, accel
        gyro = (self.imu_mount @ gyro.unsqueeze(-1)).squeeze(-1)
        accel = (self.imu_accel @ (self.imu_mount @ accel.unsqueeze(-1))).squeeze(-1)
        return gyro, accel

    def _robot_sensing(self, c, n, dt):
        dev = self.device
        alpha = lambda tau, step: -float(np.expm1(-step / tau))
        self.packet_steps = round(self.p["simulation"]["control_dt"] / dt)
        packet_dt = self.packet_steps * dt
        gyro, accel = c["gyro_anti_alias_tau_s"], c["accel_anti_alias_tau_s"]
        self.imu_alpha = torch.tensor([alpha(gyro, dt)] * 3 + [alpha(accel, dt)] * 3, device=dev)
        rates = [c["joint_velocity_filter_tau_s"]] * 4 + [c["velocity_filter_tau_s"]] * 2
        self.rate_alpha = torch.tensor([alpha(t, packet_dt) for t in rates], device=dev)
        self.local_alpha = alpha(c["local_joint_velocity_filter_tau_s"], dt)
        self.gyro_bias = torch.tensor(c["gyro_residual_bias_rad_s"], device=dev)
        self.accel_bias = torch.tensor(c["accel_bias_m_s2"], device=dev)
        counts = [c["joint_encoder_counts_per_rev"]] * 4 + [c["wheel_encoder_counts_per_rev"]] * 2
        self.increment = torch.tensor([2 * np.pi / k for k in counts], device=dev)
        self.local_increment = 2 * np.pi / c.get(
            "local_joint_encoder_counts_per_rev", c["joint_encoder_counts_per_rev"]
        )
        self.periodic = torch.tensor(
            [c.get("joint_encoder_periodic_error_rad", 0.0)] * 4
            + [c.get("wheel_encoder_periodic_error_rad", 0.0)] * 2,
            device=dev,
        )
        self.packet_dt = packet_dt
        self.imu = torch.zeros(n, 6, device=dev)  # anti-alias filtered gyro, accel
        self.packet = torch.zeros(n, 16, device=dev)
        self.encoder_previous = torch.zeros(n, 6, device=dev)
        self.rates = torch.zeros(n, 6, device=dev)
        self.local_previous = torch.zeros(n, 4, device=dev)
        self.local_rates = torch.zeros(n, 4, device=dev)

    def set_belts(self, env_ids, params) -> None:
        """Belt stiffness and lash (hip k, knee k, hip lash, knee lash; N m/rad and
        degrees) into the worlds' tendons, as plant.Drives does for one model."""
        self.belt_params[env_ids] = params
        p, b = self.p, self.phys["belts"]
        hip_k, knee_k, hip_lash, knee_lash = params.unbind(-1)
        series = 1 / (1 / hip_k + 1 / knee_k)
        hip_half = torch.deg2rad(hip_lash) / 2
        knee_half = torch.deg2rad(knee_lash) / 2 + hip_half
        model = self._env.sim.model
        rows = env_ids[:, None]
        for belts, k, tau, half in (
            (self.hip_belts, hip_k, b["hip_damping_Nm_s_rad"] / b["hip_stiffness_Nm_rad"], hip_half),
            (self.belts, series, b["knee_damping_Nm_s_rad"] / b["knee_stiffness_Nm_rad"], knee_half),
        ):
            model.tendon_stiffness[rows, belts] = k[:, None]
            model.tendon_damping[rows, belts] = (k * tau)[:, None]
            model.tendon_lengthspring[rows, belts] = torch.stack([-half, half], dim=-1)[:, None]
        dt, wdt = p["simulation"]["physics_dt"], p["training"]["drives"].get("rotor_omega_dt", 1.0)
        floor = (hip_k / self.ratio**2 * (dt / wdt) ** 2 - tp.rotor_spin(p)).clamp(min=0.0)
        model.dof_armature[rows, self.rotor_dof] = floor[:, None]

    def set_friction(self, env_ids, scale, damping_scale) -> None:
        """Per-world joint friction and damping scales (n_ids, nv); the leg and shin
        friction is rewritten every step with the belt loss, the rest is set here."""
        self.friction_scale[env_ids] = scale
        model = self._env.sim.model
        model.dof_frictionloss[env_ids] = self.base_friction * scale
        model.dof_damping[env_ids] = self.base_damping * damping_scale

    # ActionTerm interface.

    @property
    def action_dim(self) -> int:
        return 6

    @property
    def raw_action(self) -> torch.Tensor:
        return self._raw

    def nominal_q(self) -> torch.Tensor:
        """Posture references for the commanded posture labels, (n, 4)."""
        label = self._env.command_manager.get_command(self.cfg.command_name)[:, 2]
        x = (label.clamp(30, 65) - 30) / 1.0
        i = x.floor().long().clamp(0, len(self.labels) - 2)
        w = (x - i).unsqueeze(-1)
        return self.table[i] * (1 - w) + self.table[i + 1] * w

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw[:] = actions
        self.leg_target[:] = self.nominal_q() + self.cfg.leg_scale * actions[:, :4]
        self.wheel_target[:] = self.cfg.wheel_scale * actions[:, 4:]
        if self.cfg.wheel_feedforward:
            self.wheel_target += self.rolling_speeds()
        if self.cfg.fidelity:
            # A lost command frame leaves the moteus on its old targets for one more
            # 5 ms control tick (5 physics steps).
            lost = torch.rand(self.num_envs, device=self.device) < self.command_loss
            self.hold_steps[:] = torch.where(lost, getattr(self, "packet_steps", 5), 0)

    def rolling_speeds(self) -> torch.Tensor:
        """Wheel speeds (left, right) that roll at the commanded speed and yaw rate.

        Both wheel axes are +y, so forward is positive on both; yaw left needs the
        right wheel faster."""
        c = self._env.command_manager.get_command(self.cfg.command_name)
        forward = c[:, 0] / self.wheel_radius
        turn = c[:, 1] * self.wheel_track / (2 * self.wheel_radius)
        return torch.stack([forward - turn, forward + turn], dim=-1)

    def reset(self, env_ids=None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        self._raw[env_ids] = 0.0
        self.current[env_ids] = 0.0
        self.wheel_integral[env_ids] = 0.0
        self.motor_torque[env_ids] = 0.0
        self.history[:, env_ids] = 0.0
        self.fresh[env_ids] = True
        self.push_steps[env_ids] = 0
        if self.cfg.actuator_dr:
            self.bus_V[env_ids] = self.pack_V[env_ids]
        if self.cfg.fidelity:
            self.gyro_walk[env_ids] = 0.0
            self.hold_steps[env_ids] = 0
            self.leg_applied[env_ids] = self.leg_target[env_ids]
            self.wheel_applied[env_ids] = self.wheel_target[env_ids]

    # Sensors.

    def _measure(self) -> torch.Tensor:
        if self.cfg.sensors is not None:
            return self.packet
        data = self._entity.data
        q, qd = data.joint_pos, data.joint_vel
        sensordata = self._env.sim.data.sensordata
        gyro = sensordata[:, self.gyro_adr : self.gyro_adr + 3]
        accel = sensordata[:, self.accel_adr : self.accel_adr + 3]
        hip = q[:, self.leg[[0, 2]]]
        a2 = hip + q[:, self.shin]
        hipd = qd[:, self.leg[[0, 2]]]
        a2d = hipd + qd[:, self.shin]
        if self.spring:
            # The hip encoder is on the rotor: it reads through the belt.
            hip = q[:, self.motor[[0, 2]]] / self.ratio
            hipd = qd[:, self.motor[[0, 2]]] / self.ratio
        legs = torch.stack([hip[:, 0], a2[:, 0], hip[:, 1], a2[:, 1]], dim=-1)
        legsd = torch.stack([hipd[:, 0], a2d[:, 0], hipd[:, 1], a2d[:, 1]], dim=-1)
        n = legs.shape[0]
        s = self.noise_scale
        gyro, accel = self._board(gyro, accel)
        gyro = gyro + s * self.gyro_std * torch.randn(n, 3, device=self.device)
        accel = accel + s * self.accel_std * torch.randn(n, 3, device=self.device)
        if self.cfg.actuator_dr:
            gyro, accel = gyro + self.imu_bias[:, :3], accel + self.imu_bias[:, 3:]
        return torch.cat(
            [gyro, accel, legs + self.encoder_offset, legsd, qd[:, self.wheel]], dim=-1
        )

    def _imu(self) -> torch.Tensor:
        sensordata = self._env.sim.data.sensordata
        gyro = sensordata[:, self.gyro_adr : self.gyro_adr + 3]
        accel = sensordata[:, self.accel_adr : self.accel_adr + 3]
        return torch.cat(self._board(gyro, accel), dim=-1)

    def _encoders(self) -> torch.Tensor:
        """Joint encoders (hip, knee drive) x 2 with the calibration offset and wheel
        encoders (relative to the shin), quantised with their periodic error."""
        q = self._entity.data.joint_pos
        leg = q[:, self.motor] * (self.motor_scale / self.ratio) if self.spring else q[:, self.leg]
        wheel = q[:, self.wheel] * self.wheel_enc_scale if self.cfg.fidelity else q[:, self.wheel]
        raw = torch.cat([leg + self.encoder_offset, wheel], dim=-1)
        harmonic = torch.cat([2 * raw[:, :4], raw[:, 4:]], dim=-1)
        raw = raw + self.periodic * torch.sin(harmonic)
        return torch.round(raw / self.increment) * self.increment

    def _packet(self, encoders) -> torch.Tensor:
        n, s, dev = self.num_envs, self.noise_scale, self.device
        gyro = self.imu[:, :3] + self.gyro_bias + s * self.gyro_std * torch.randn(n, 3, device=dev)
        accel = self.imu[:, 3:] + self.accel_bias + s * self.accel_std * torch.randn(n, 3, device=dev)
        if self.cfg.actuator_dr:
            gyro, accel = gyro + self.imu_bias[:, :3], accel + self.imu_bias[:, 3:]
        if self.cfg.fidelity:
            step = math.sqrt(self.packet_dt)
            self.gyro_walk += self.gyro_walk_std * step * torch.randn(n, 3, device=dev)
            gyro = gyro + self.gyro_walk
        return torch.cat([gyro, accel, encoders[:, :4], self.rates], dim=-1)

    def _start(self, fresh) -> None:
        """Sensor and filter states of freshly reset worlds, from the robot at rest."""
        m = fresh[:, None]
        imu, enc = self._imu(), self._encoders()
        local = enc[:, :4]
        self.imu.copy_(torch.where(m, imu, self.imu))
        self.encoder_previous.copy_(torch.where(m, enc, self.encoder_previous))
        self.rates.copy_(torch.where(m, 0.0, self.rates))
        self.local_previous.copy_(torch.where(m, local, self.local_previous))
        self.local_rates.copy_(torch.where(m, 0.0, self.local_rates))
        self.packet.copy_(torch.where(m, self._packet(enc), self.packet))

    def _sense(self) -> None:
        """One physics step of the robot's sensors: filters, packet, local feedback."""
        self.imu += self.imu_alpha * (self._imu() - self.imu)
        enc = self._encoders()
        if self.head % self.packet_steps == 0:
            raw = (enc - self.encoder_previous) / self.packet_dt
            self.rates += self.rate_alpha * (raw - self.rates)
            self.encoder_previous.copy_(enc)
            self.packet.copy_(self._packet(enc))
        q = enc[:, :4]
        dt = self._env.physics_dt
        self.local_rates += self.local_alpha * ((q - self.local_previous) / dt - self.local_rates)
        self.local_previous.copy_(q)

    def sample(self, lag: torch.Tensor) -> torch.Tensor:
        """Sensor row recorded ``lag`` physics steps ago, per world, (n, 16)."""
        index = (self.head - lag) % HISTORY
        return self.history[index, self._worlds]

    def refill(self, row: torch.Tensor | None = None) -> None:
        """Fill the whole history of freshly reset worlds with a current reading.

        Run by the sensor observation, which follows the post-reset forward and
        sense, so before the first physics step; without a host sync."""
        if self.cfg.sensors is not None:
            self._start(self.fresh)
        row = self._measure() if row is None else row
        self.history.copy_(torch.where(self.fresh[None, :, None], row, self.history))
        self.fresh.zero_()

    # Physics step.

    def apply_actions(self) -> None:
        self.head = (self.head + 1) % HISTORY
        if self.cfg.sensors is not None:
            self._sense()
        self.history[self.head] = self._measure()
        if self.cfg.sensors is not None:
            # The servo reads its own encoder at 1 kHz with no added lag: a lagged
            # collocated loop on the light knee-drive hinge chatters.
            q, qd = self.local_previous, self.local_rates
        else:
            enc = self.sample(self.encoder_lag)
            q, qd = enc[:, 6:10], enc[:, 10:14]
        # The hub drive closes its speed loop on its own encoder: no leg encoder lag.
        wheel_speed = self._entity.data.joint_vel[:, self.wheel]
        cfg = self.cfg
        leg_target, wheel_target = self.leg_target, self.wheel_target
        if cfg.fidelity:
            wheel_speed = wheel_speed * self.wheel_enc_scale
            held = (self.hold_steps > 0)[:, None]
            self.leg_applied[:] = torch.where(held, self.leg_applied, self.leg_target)
            self.wheel_applied[:] = torch.where(held, self.wheel_applied, self.wheel_target)
            self.hold_steps.sub_(1).clamp_(min=0)
            leg_target, wheel_target = self.leg_applied, self.wheel_applied
        legs = cfg.joint_kp * (leg_target - q) - cfg.joint_kd * qd
        error = wheel_target - wheel_speed
        dt = self._env.physics_dt
        if cfg.wheel_ki > 0:
            limit = self.hub_peak / cfg.wheel_ki
            self.wheel_integral[:] = (self.wheel_integral + error * dt).clamp(-limit, limit)
        wheels = cfg.wheel_kv * error + cfg.wheel_ki * self.wheel_integral

        data = self._entity.data
        joint_qd = data.joint_vel
        hm, gl = self.phys["hip_motor"], dict(self.phys["hub_motor"])
        gl["FOC_time_constant_s"] = self.hub_tau
        if cfg.actuator_dr:
            hm = dict(hm)
            hm["FOC_time_constant_s"] = self.hip_tau
            hm["no_load_speed_motor_rad_s"] = hm["no_load_speed_motor_rad_s"] * (
                self.bus_V / self.full_pack_V
            )
            gl["drive_voltage_limit_V"] = 0.95 * self.bus_V
        if cfg.fidelity:
            gl["voltage_limit_winding_temperature_C"] = self.winding_C
        hip_t, hip_out, _, self.current[:, :4] = drives.hip_motor(
            torch, hm, self.ratio, legs, self.motor_scale * joint_qd[:, self.motor], self.current[:, :4], dt
        )
        hub_t, _, self.current[:, 4:], _ = drives.hub_motor(
            torch, gl, wheels, joint_qd[:, self.wheel], self.current[:, 4:], dt
        )
        belt = hip_t * self.ratio
        torque = torch.cat([belt, hub_t], dim=-1) * self.torque_scale
        if cfg.fidelity:
            q_all = self._entity.data.joint_pos
            angle = torch.cat([self.motor_scale * q_all[:, self.motor], q_all[:, self.wheel]], dim=-1)
            torque = torque + self.cog_amp * torch.sin(84 * angle + self.cog_phase)
        if cfg.actuator_dr:
            # Bus current from the motors' mechanical power and the hubs' copper loss
            # (motoring only; regeneration is not credited), for the next step's sag.
            speed = torch.cat(
                [self.motor_scale * joint_qd[:, self.motor], joint_qd[:, self.wheel]], dim=-1
            )
            motor = torch.cat([hip_t, hub_t], dim=-1) * self.torque_scale
            power = (motor * speed).clamp(min=0).sum(-1, keepdim=True)
            power = power + 1.5 * self.hub_R * (self.current[:, 4:] ** 2).sum(-1, keepdim=True)
            self.bus_V = self.pack_V - self.sag_ohm * power / self.bus_V
        self.motor_torque[:] = torque
        self.output_torque[:] = torch.cat([hip_out, hub_t], dim=-1) * self.torque_scale
        self._entity.set_joint_effort_target(torque, joint_ids=self.driven)
        on = (self.push_steps > 0).float()[:, None, None]
        self._entity.write_external_wrench_to_sim(
            self.push_force * on, torch.zeros_like(self.push_force), body_ids=self.chassis
        )
        self.push_steps.sub_(1).clamp_(min=0)

        friction = self._env.sim.model.dof_frictionloss
        ten = self._env.sim.data
        if self.spring:
            model = self._env.sim.model
            spring = model.tendon_lengthspring[:, self.belts]
            length = ten.ten_length[:, self.belts]
            # Tension only past the dead band.
            length = length - torch.clamp(length, spring[..., 0], spring[..., 1])
            knee = model.tendon_stiffness[:, self.belts] * length + (
                model.tendon_damping[:, self.belts] * ten.ten_velocity[:, self.belts]
            )
        else:
            knee = self.knee_k * ten.ten_length[:, self.belts] + self.knee_c * ten.ten_velocity[:, self.belts]
        base = self.base_friction
        if cfg.actuator_dr:
            base = base * self.friction_scale
        if cfg.fidelity:
            base = base.clone()
            v = self._entity.data.joint_vel
            for dofs, joints in ((self.leg_dof, self.leg), (self.shin_dof, self.shin)):
                stick = 1 + (self.stiction_ratio - 1) * torch.exp(-((v[:, joints] / 0.02) ** 2))
                base = base.expand_as(friction).clone() if base.dim() == 1 else base
                base[:, dofs] = base[:, dofs] * stick
        # base is (nv,) without actuator DR and (worlds, nv) with it.
        friction[:, self.leg_dof] = base[..., self.leg_dof] + self.loss * belt.abs()
        friction[:, self.shin_dof] = base[..., self.shin_dof] + self.loss * knee.abs()
