"""M18 phase F: the built robot's measured values in place of the nominal ones.

config/hardware-measured.json holds one entry per measurable value, each null until a
bench test (docs/hardware/bench-tests.md, fitters in bipe.m18.bench) fills it. ``apply``
turns the filled ones into the full plant's parameters and sensor configuration:

- plant values (mass, COM, torque constants, friction) replace the parameter they name;
- sensor values (bias, noise, delay) replace the sensor configuration's entry;
- the rest are bipe.m18.fidelity draw keys (stiction, cogging, wheel radius, IMU mount,
  accelerometer scale, gyro walk, encoder offsets, timing, floor, payload) and go
  through ``fidelity.apply``, so the measured robot is modelled exactly as the fidelity
  suite models a drawn one.

``draw(values)`` gives the fidelity part alone, which the fidelity suite and the
get-up check add under every case (``--measured``): the measured robot, then the case's
own model on top.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

from ..parameters import ROOT
from . import fidelity

PATH = ROOT / "config/hardware-measured.json"

PLANT = {
    "chassis_mass_kg": ("rigid_bodies", "chassis", "mass_kg"),
    "chassis_com_m": ("rigid_bodies", "chassis", "com_m"),
    "hip_coulomb_motor_Nm": ("physics", "hip_motor", "coulomb_motor_Nm"),
    "hip_viscous_motor_Nm_s_rad": ("physics", "hip_motor", "viscous_motor_Nm_s_rad"),
    "wheel_coulomb_Nm": ("physics", "friction", "wheel_coulomb_Nm"),
    "wheel_viscous_Nm_s_rad": ("physics", "friction", "wheel_viscous_Nm_s_rad"),
}
"""Measured key: the parameter path it replaces."""

TORQUE_CONSTANT = {
    "hip_torque_constant_Nm_A": (
        "hip_motor", ("torque_constant_Nm_A", "peak_torque_motor_Nm", "continuous_torque_motor_Nm")
    ),
    "hub_torque_constant_Nm_A": (
        "hub_motor", ("torque_constant_Nm_A", "peak_torque_Nm", "rated_torque_Nm")
    ),
}
"""A measured torque constant scales the motor's torque limits with it (same currents)."""

SENSORS = {
    "gyro_residual_bias_rad_s": "gyro_residual_bias_rad_s",
    "gyro_noise_std_rad_s": "gyro_noise_std_rad_s",
    "accel_noise_std_m_s2": "accel_noise_std_m_s2",
    "transport_delay_ms": "transport_delay_s",
}

FIDELITY = (
    "stiction_ratio", "cogging_fraction", "wheel_radius_ratio", "imu_mount_deg",
    "accel_scale", "gyro_walk", "encoder_offset_deg", "jitter_ms", "packet_loss",
    "command_loss", "floor_friction", "payload_kg",
)  # fmt: skip

KNOWN = set(PLANT) | set(TORQUE_CONSTANT) | set(SENSORS) | set(FIDELITY)


def load(path: Path | str = PATH) -> dict:
    """The measured values that are filled in, as {key: value}."""
    entries = json.loads(Path(path).read_text())["values"]
    unknown = set(entries) - KNOWN
    if unknown:
        raise KeyError(f"unknown measured keys: {sorted(unknown)}")
    return {k: e["value"] for k, e in entries.items() if e["value"] is not None}


def draw(values: dict) -> dict:
    """The fidelity part of the measured values, as a ``fidelity.apply`` draw."""
    return {k: (tuple(v) if isinstance(v, list) else v) for k, v in values.items() if k in FIDELITY}


def apply_plant(p: dict, sensors: dict, values: dict) -> tuple[dict, dict]:
    """The plant and sensor entries replaced by the measured ones (copies)."""
    p, sensors = copy.deepcopy(p), copy.deepcopy(sensors)
    for key, path in PLANT.items():
        if key in values:
            node = p
            for name in path[:-1]:
                node = node[name]
            node[path[-1]] = values[key]
    for key, (motor, fields) in TORQUE_CONSTANT.items():
        if key in values:
            m = p["physics"][motor]
            scale = values[key] / m["torque_constant_Nm_A"]
            for field in fields:
                m[field] *= scale
            if motor == "hub_motor":
                for field in ("peak_torque_per_wheel", "continuous_torque_per_wheel"):
                    p["motor"][field] *= scale
    for key, field in SENSORS.items():
        if key in values:
            v = values[key]
            sensors[field] = v / 1000 if key == "transport_delay_ms" else v
    return p, sensors


def apply(p: dict, sensors: dict, values: dict | None = None, seed: int = 0, extra=None):
    """The full plant's parameters, sensor configuration and policy settings for the
    measured robot (``values``, default the file), with ``extra`` fidelity draw keys on
    top. Same return as ``fidelity.apply``."""
    values = load() if values is None else values
    p, sensors = apply_plant(p, sensors, values)
    return fidelity.apply(p, sensors, {**draw(values), **(extra or {})}, seed=seed)
