"""M18 phase F: fitters that turn bench recordings into config/hardware-measured.json
values (docs/hardware/bench-tests.md says how to record each one).

Each fitter takes plain arrays (as logged by moteus_tool, the Teensy's bench image or
a scale) and returns {measured key: value}; ``record`` writes them into the file with
their source.

    python -m bipe.m18.bench record KEY VALUE --source "B3, 2027-01-10, left hip"
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .fidelity import COG_CYCLES
from .measured import KNOWN, PATH


def torque_constant(current_A, torque_Nm, key):
    """B2: the slope of output torque (load cell x arm, divided by the reduction) on
    q-axis current, through the origin, below saturation."""
    i, t = np.asarray(current_A, float), np.asarray(torque_Nm, float)
    return {key: float(i @ t / (i @ i))}


def friction(speed_rad_s, torque_Nm, breakaway_Nm=None, prefix="hip"):
    """B3: Coulomb and viscous terms from a constant-speed sweep in both directions
    (torque = c sign(w) + b w, least squares), and the stiction ratio from the breakaway
    torque of a slow current ramp from rest. ``prefix`` hip gives the RO40 motor-side
    keys, wheel the hub's."""
    w, t = np.asarray(speed_rad_s, float), np.asarray(torque_Nm, float)
    a = np.column_stack([np.sign(w), w])
    (c, b), *_ = np.linalg.lstsq(a, t, rcond=None)
    names = {"hip": ("hip_coulomb_motor_Nm", "hip_viscous_motor_Nm_s_rad"),
             "wheel": ("wheel_coulomb_Nm", "wheel_viscous_Nm_s_rad")}[prefix]  # fmt: skip
    out = {names[0]: float(c), names[1]: float(b)}
    if breakaway_Nm is not None:
        out["stiction_ratio"] = float(max(1.0, breakaway_Nm / c))
    return out


def cogging(angle_rad, torque_Nm, rated_Nm):
    """B4: the ripple at COG_CYCLES per turn in the torque needed to turn the rotor
    slowly at constant speed (any mean friction drops out), over the rated torque."""
    th, t = np.asarray(angle_rad, float), np.asarray(torque_Nm, float)
    a = np.column_stack([np.sin(COG_CYCLES * th), np.cos(COG_CYCLES * th), np.ones_like(th)])
    (s, c, _), *_ = np.linalg.lstsq(a, t, rcond=None)
    return {"cogging_fraction": float(np.hypot(s, c) / rated_Nm)}


def wheel_radius(distance_m, turns, nominal_m):
    """B5: per wheel, the floor distance rolled over the hub turns counted by its
    encoder (tape measure over a few metres), as true over nominal radius."""
    d, n = np.atleast_1d(distance_m).astype(float), np.atleast_1d(turns).astype(float)
    return {"wheel_radius_ratio": (d / (2 * np.pi * n) / nominal_m).tolist()}


def allan(rate, dt, tau_s):
    """Overlapping Allan deviation of a rate signal at averaging time ``tau_s``."""
    m = max(1, int(round(tau_s / dt)))
    theta = np.cumsum(rate) * dt
    d = theta[2 * m :] - 2 * theta[m:-m] + theta[: -2 * m]
    return float(np.sqrt(np.mean(d**2) / (2 * (m * dt) ** 2)))


def imu_still(gyro_rad_s, accel_m_s2, dt):
    """B6: the robot standing on its stand, level by a machinist's level, IMU logging
    at its rate for at least 30 min. Gives the gyro bias (mean), white noise (std of
    the per-sample rate), the bias walk (Allan deviation slope at long averaging times,
    K = sigma(tau) x sqrt(3 / tau)), the accelerometer noise, and the board's mounting
    tilt from the mean specific force (roll, pitch, degrees)."""
    g, a = np.asarray(gyro_rad_s, float), np.asarray(accel_m_s2, float)
    bias = g.mean(axis=0)
    t = len(g) * dt
    tau = t / 10
    walk = float(np.mean([allan(g[:, k] - bias[k], dt, tau) for k in range(3)]) * np.sqrt(3 / tau))
    f = a.mean(axis=0)
    roll = np.degrees(np.arctan2(f[1], f[2]))
    pitch = np.degrees(np.arctan2(-f[0], np.hypot(f[1], f[2])))
    return {
        "gyro_residual_bias_rad_s": bias.tolist(),
        "gyro_noise_std_rad_s": float((g - bias).std(axis=0).mean()),
        "gyro_walk": walk,
        "accel_noise_std_m_s2": float((a - f).std(axis=0).mean()),
        "imu_mount_deg": [float(roll), float(pitch)],
    }


def accel_six_face(up_means, down_means, g=9.80665):
    """B7: the robot's IMU board (or the chassis on a cube jig) resting on each face in
    turn; per axis the mean reading with that axis up and down. Scale error =
    (up - down) / 2g - 1 (the offset (up + down) / 2 is the bias the estimator already
    calibrates)."""
    up, down = np.asarray(up_means, float), np.asarray(down_means, float)
    return {"accel_scale": ((up - down) / (2 * g) - 1).tolist()}


def encoder_offsets(read_deg, stop_deg):
    """B8: each joint pressed onto its calibrated end stop; offset = reading - stop."""
    return {"encoder_offset_deg": (np.asarray(read_deg, float) - np.asarray(stop_deg, float)).tolist()}


def loop_timing(sampled_s, used_s, tick_s=0.005, commands_sent=None, commands_acked=None):
    """B9: the age of the sensor data when the control law uses it. The bench image
    logs, per sensor reading (IMU sample, each moteus reply), the time the reading was
    taken (``sampled_s``: the IMU's data-ready time, a moteus reply's own timestamp or,
    failing that, its request time) and the control tick that used it (``used_s``: the
    tick's start; NaN when no tick used it). On the synchronous loop a reading taken in
    one tick is used at the next, so the age is about one tick (5 ms), whatever the
    CAN round trip is; this is the quantity the simulation's ``transport_delay_s`` and
    the C estimator's prediction delay mean.

    - ``transport_delay_ms``: the median age;
    - ``jitter_ms``: the 99th percentile minus the median, in whole ms (0 under
      0.5 ms). The full plant applies delay and jitter in whole 5 ms ticks
      (bipe.sensing), so a jitter under 2.5 ms changes nothing there; the training
      environment applies it in 1 ms steps;
    - ``packet_loss``: readings no tick used, or used more than a tick late (the
      synchronous loop has moved on: a late reading is a lost one, bipe.m18.fidelity);
    - ``command_loss``: commands without an acknowledgement, when both counts are given.
    """
    s, u = np.asarray(sampled_s, float), np.asarray(used_s, float)
    age = u - s
    ok = np.isfinite(age) & (age <= 2 * tick_s)
    median = float(np.median(age[ok]))
    spread = float(np.percentile(age[ok], 99)) - median
    out = {
        "transport_delay_ms": median * 1000,
        "jitter_ms": 0 if spread < 0.0005 else int(np.ceil(spread * 1000)),
        "packet_loss": float(1 - ok.mean()),
    }
    if commands_sent and commands_acked is not None:
        out["command_loss"] = float(1 - commands_acked / commands_sent)
    return out


def incline_slip(angle_deg):
    """B10: the robot, wheels locked, on a board tilted until it slides: mu = tan."""
    return {"floor_friction": float(np.tan(np.radians(angle_deg)))}


def record(values: dict, source: str, path: Path = PATH):
    """Write measured values into the file with their source."""
    data = json.loads(path.read_text())
    for key, value in values.items():
        if key not in KNOWN:
            raise KeyError(key)
        data["values"][key]["value"] = value
        data["values"][key]["source"] = source
    path.write_text(json.dumps(data, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("record", help="write one measured value (JSON) into the file")
    r.add_argument("key", choices=sorted(KNOWN))
    r.add_argument("value", type=json.loads)
    r.add_argument("--source", required=True)
    sub.add_parser("show", help="list the filled values")
    args = parser.parse_args()
    if args.command == "record":
        record({args.key: args.value}, args.source)
    from .measured import load

    print(json.dumps(load(), indent=2))


if __name__ == "__main__":
    main()
