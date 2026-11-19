"""M18 fidelity models: what the plants still left out about the real motors, sensors,
timing and floor, as one parameter set shared by the training environment
(bipe.m13.rl.firmware, ``DriveActionCfg.fidelity``) and the full plant (``apply``).

Every model is off at its nominal value, so every earlier result reproduces exactly.
A *draw* is a dict with any of the keys of ``NOMINAL``; ``apply`` turns one into the
full plant's parameters, sensor configuration and policy settings, and
``bipe.m13.rl.mdp.randomise_drive`` draws them per world from ``RANGES``.

Models and their bases (each one a stated assumption until the hardware build
measures it; docs/hardware/build-plan.md lists the bench tests):

- Cogging (``cogging_fraction``): a torque ripple sin(84 theta + phase) on each RO40
  rotor and GL40 hub, theta the rotor angle, amplitude a fraction of the motor's rated
  torque. 84 cycles per turn is the least common multiple of 12 slots and 14 poles, the
  usual layout of 40 mm gimbal motors (an assumption; the M10 friction basis lists GL40
  cogging as unmeasured).
- Stiction (``stiction_ratio``): every leg and shin hinge's Coulomb friction rises to
  this multiple near rest, 1 + (r - 1) exp(-(v / 0.02 rad/s)^2): breakaway above
  sliding friction, as in belt drives.
- Winding temperature (``winding_C``): hot magnets lower the torque constant
  (NdFeB -0.12 %/degree C from 25 degrees C) and hot copper raises the GL40 resistance
  (the voltage limit's R_hot, 0.393 %/degree C). The nominal plant keeps its 80
  degree C voltage-limit resistance and 25 degree C torque constant; a draw sets one
  temperature for both. Held for an episode: a 20 s episode heats the windings by a
  few degrees at most (M11 copper-loss screen), so no heating dynamics.
- Wheel radius (``wheel_radius_ratio``, true over nominal, per wheel): tyre wear and a
  left/right mismatch. Modelled as a scale s = r0 / r on the hub encoder reading
  (estimator, observation and the hub speed loop), which gives the same ground speed
  (r / r0 x the command) and the same odometry error as a real radius change; the 3 %
  change of force per torque is left out.
- IMU mounting (``imu_mount_deg``, roll and pitch): the board sits off the chassis
  axes. The pre-release gravity calibration sees the tilted board too, so the robot's
  "level" is off by the tilt.
- IMU scale and cross-axis (``accel_scale``, ``accel_cross``): the accelerometer reads
  (I + E) f, E diagonal the scale errors, off-diagonal +-accel_cross.
- Gyro bias random walk (``gyro_walk``, rad/s per sqrt(s)): bias drift within an
  episode on top of the fixed residual bias (LSM6DSO zero-rate change with
  temperature, 0.01 deg/s per degree C).
- Encoder zero offsets (``encoder_offset_deg``, four joints): assembly calibration
  error against the end stops.
- Timing (``jitter_ms``, ``packet_loss``): each sensor packet's transport delay is the
  nominal plus 0..jitter_ms, drawn per packet; a lost packet never arrives and the
  firmware keeps the last one it had. ``command_loss``: a lost CAN command leaves the
  moteus on its previous leg and wheel targets for one more 5 ms tick. The control loop
  is synchronous (the Teensy polls every moteus each 5 ms tick over CAN-FD, M11), so
  a reply's age varies by well under a tick and a reply that misses its tick is
  superseded by the next one: a late reply is a lost one. The training range keeps
  jitter to 1 ms. The full plant (bipe.sensing) counts delay and jitter in whole 5 ms
  ticks, so jitter under 2.5 ms rounds to none there: its timing cases test packet
  and command loss only; the training environment applies jitter in 1 ms steps.
  Jitter of whole ticks (an asynchronous link) breaks the policy on
  the full plant, because the C estimator differentiates whatever encoder packet it is
  handed (16 November: fine-tune 1 falls within 1 s at 10 ms jitter; the classical
  controller holds), so the suite does not include it.
- Floor (``floor_friction``, ``floor_stiffness``): sliding friction, and a stiffness
  scale on every contact (below 1: a soft floor, carpet or a mat).
- Payload (``payload_kg``): a point mass at the head (HEAD_POINT_M), as a camera or a
  heavier battery would add.
"""

from __future__ import annotations

import copy
import math

import numpy as np

COG_CYCLES = 84
"""Cogging cycles per rotor turn (12 slots, 14 poles; assumption)."""
KT_TEMPCO = -0.0012
"""Torque constant change per degree C (NdFeB remanence), from 25 degrees C."""
KT_REFERENCE_C = 25.0
STICTION_SPEED = 0.02
"""Joint speed (rad/s) below which the breakaway friction applies."""
HEAD_POINT_M = (0.035, 0.0, 0.07)
"""The head pod's centre in the chassis frame (m10 body and head layout:
x = -hip_x + head x, z = body top + neck + half the pod)."""

NOMINAL = {
    "cogging_fraction": 0.0,
    "stiction_ratio": 1.0,
    "winding_C": None,
    "wheel_radius_ratio": (1.0, 1.0),
    "imu_mount_deg": (0.0, 0.0),
    "accel_scale": (0.0, 0.0, 0.0),
    "accel_cross": 0.0,
    "gyro_walk": 0.0,
    "encoder_offset_deg": None,
    "jitter_ms": 0,
    "packet_loss": 0.0,
    "command_loss": 0.0,
    "floor_friction": None,
    "floor_stiffness": 1.0,
    "payload_kg": 0.0,
}

RANGES = {
    "cogging_fraction": (0.0, 0.03),
    "stiction_ratio": (1.0, 1.5),
    "winding_C": (20.0, 80.0),
    "wheel_radius_ratio": (0.97, 1.03),
    "imu_mount_deg": 2.0,
    "accel_scale": 0.02,
    "accel_cross": 0.01,
    "gyro_walk": (0.0, 3e-4),
    "encoder_offset_deg": 1.5,
    "jitter_ms": (0, 1),
    "packet_loss": (0.0, 0.02),
    "command_loss": (0.0, 0.02),
    "floor_friction": (0.3, 1.2),
    "floor_stiffness": (0.3, 1.0),
    "payload_kg": (0.0, 0.1),
    "com_m": 0.008,
}
"""Training ranges (uniform; +-x for a single number). ``com_m`` widens the chassis
COM offset (M14: +-5 mm) to cover the payload's shift."""


def kt_scale(winding_C):
    """Torque constant at the winding temperature, over its 25 degree C value."""
    return 1.0 + KT_TEMPCO * (winding_C - KT_REFERENCE_C)


def cogging(xp, angle, amplitude, phase):
    """Cogging torque at rotor ``angle`` (rad), any array module."""
    return amplitude * xp.sin(COG_CYCLES * angle + phase)


def stiction(xp, speed, ratio):
    """Multiplier on Coulomb friction: ``ratio`` at rest, 1 when moving."""
    return 1.0 + (ratio - 1.0) * xp.exp(-((speed / STICTION_SPEED) ** 2))


def mount_matrix(roll_deg, pitch_deg):
    """Rotation from the chassis frame to the tilted IMU board's frame (body vector b
    reads as R @ b on the board)."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), math.sin(r)], [0, -math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, -math.sin(p)], [0, 1, 0], [math.sin(p), 0, math.cos(p)]])
    return ry @ rx


def accel_matrix(scale, cross, rng=None):
    """I + E: diagonal scale errors, off-diagonal +-cross (signs from rng, else +)."""
    e = np.diag(np.asarray(scale, float))
    if cross:
        signs = np.ones((3, 3)) if rng is None else rng.choice([-1.0, 1.0], (3, 3))
        e = e + cross * signs * (1 - np.eye(3))
    return np.eye(3) + e


def resolved(draw):
    out = dict(NOMINAL)
    unknown = set(draw) - set(NOMINAL)
    if unknown:
        raise KeyError(f"unknown fidelity keys: {sorted(unknown)}")
    out.update(draw)
    return out


def apply(p, sensors, draw, seed=0):
    """The full plant's parameters and sensor config under ``draw``, and the policy
    settings (``bipe.m13.policy_control.FIDELITY``). Copies; the inputs are unchanged."""
    f = resolved(draw)
    p, sensors = copy.deepcopy(p), copy.deepcopy(sensors)
    rng = np.random.default_rng(seed)
    phys = p["physics"]
    hm, gl = phys["hip_motor"], phys["hub_motor"]
    fid = phys.setdefault("fidelity", {})
    if f["cogging_fraction"]:
        rated = [hm["continuous_torque_motor_Nm"]] * 4 + [gl["rated_torque_Nm"]] * 2
        fid["cogging_Nm"] = (np.asarray(rated) * f["cogging_fraction"]).tolist()
        fid["cogging_phase"] = rng.uniform(0, 2 * math.pi, 6).tolist()
    if f["stiction_ratio"] != 1.0:
        fid["stiction_ratio"] = f["stiction_ratio"]
    if f["floor_stiffness"] != 1.0:
        fid["floor_stiffness"] = f["floor_stiffness"]
    if f["winding_C"] is not None:
        k = kt_scale(f["winding_C"])
        for key in ("peak_torque_motor_Nm", "continuous_torque_motor_Nm", "torque_constant_Nm_A"):
            hm[key] *= k
        for key in ("peak_torque_Nm", "rated_torque_Nm", "torque_constant_Nm_A"):
            gl[key] *= k
        for key in ("peak_torque_per_wheel", "continuous_torque_per_wheel"):
            p["motor"][key] *= k
        gl["voltage_limit_winding_temperature_C"] = f["winding_C"]
    if f["floor_friction"] is not None:
        p["contact"]["sliding_friction"] = f["floor_friction"]
    if f["payload_kg"]:
        body = p["rigid_bodies"]["chassis"]
        m0, mp = body["mass_kg"], f["payload_kg"]
        c0, point = np.asarray(body["com_m"]), np.asarray(HEAD_POINT_M)
        com = (m0 * c0 + mp * point) / (m0 + mp)
        inertia = np.asarray(body["inertia_kg_m2"], float)
        # Parallel axis about the new COM: the body's own shift plus the point mass.
        for mass, at in ((m0, c0), (mp, point)):
            r = at - com
            inertia = inertia + mass * (r @ r * np.eye(3) - np.outer(r, r))
        body.update(mass_kg=m0 + mp, com_m=com.tolist(), inertia_kg_m2=inertia.tolist())
    # Sensors.
    if any(f["imu_mount_deg"]) or any(f["accel_scale"]) or f["accel_cross"]:
        sensors["imu_mount_deg"] = list(f["imu_mount_deg"])
        sensors["accel_matrix"] = accel_matrix(f["accel_scale"], f["accel_cross"], rng).tolist()
    if f["gyro_walk"]:
        sensors["gyro_bias_walk_rad_s_sqrt_s"] = f["gyro_walk"]
    if f["encoder_offset_deg"] is not None:
        sensors["joint_encoder_bias_rad"] = np.radians(f["encoder_offset_deg"]).tolist()
    ratio = np.asarray(f["wheel_radius_ratio"], float)
    if np.any(ratio != 1.0):
        sensors["wheel_encoder_scale"] = (1.0 / ratio).tolist()
    if f["jitter_ms"]:
        sensors["transport_jitter_s"] = f["jitter_ms"] / 1000
    if f["packet_loss"]:
        sensors["packet_loss"] = f["packet_loss"]
    policy = {}
    if f["command_loss"]:
        policy["command_loss"] = f["command_loss"]
    if "wheel_encoder_scale" in sensors:
        policy["wheel_encoder_scale"] = sensors["wheel_encoder_scale"]
    policy["seed"] = seed
    return p, sensors, policy


def sample(rng):
    """One draw with every model inside its training range (uniform), as a dict for
    ``apply``; used where a run wants a random robot rather than the suite's draws."""
    r = RANGES
    u = lambda lo, hi, *n: rng.uniform(lo, hi, *n)
    sym = lambda x, n: tuple(float(v) for v in rng.uniform(-x, x, n))
    return {
        "cogging_fraction": float(u(*r["cogging_fraction"])),
        "stiction_ratio": float(u(*r["stiction_ratio"])),
        "winding_C": float(u(*r["winding_C"])),
        "wheel_radius_ratio": tuple(float(v) for v in u(*r["wheel_radius_ratio"], 2)),
        "imu_mount_deg": sym(r["imu_mount_deg"], 2),
        "accel_scale": sym(r["accel_scale"], 3),
        "accel_cross": float(u(0, r["accel_cross"])),
        "gyro_walk": float(u(*r["gyro_walk"])),
        "encoder_offset_deg": sym(r["encoder_offset_deg"], 4),
        "jitter_ms": int(rng.integers(r["jitter_ms"][0], r["jitter_ms"][1] + 1)),
        "packet_loss": float(u(*r["packet_loss"])),
        "command_loss": float(u(*r["command_loss"])),
        "floor_friction": float(u(*r["floor_friction"])),
        "floor_stiffness": float(u(*r["floor_stiffness"])),
        "payload_kg": float(u(*r["payload_kg"])),
    }
