"""Noisy delayed IMU/encoder pipeline. Controller receives only estimated state."""

import math
from collections import deque

import numpy as np

from .articulated import JOINTS, WHEELS
from .profiles import kinematics


class JointEncoderFeedback:
    """Local actuator encoder readout at the servo rate, before network delay."""

    def __init__(self, data, config):
        self.increment = (
            2
            * math.pi
            / config.get(
                "local_joint_encoder_counts_per_rev", config["joint_encoder_counts_per_rev"]
            )
        )
        self.tau = config["local_joint_velocity_filter_tau_s"]
        self.bias = np.asarray(config.get("joint_encoder_bias_rad", [0, 0, 0, 0]))
        self.periodic = config.get("joint_encoder_periodic_error_rad", 0.0)
        self.previous = self.read(data)
        self.rates = np.zeros(4)

    def read(self, data):
        raw = np.array([data.sensor(n + "_encoder").data[0] for n in JOINTS])
        return (
            np.round((raw + self.bias + self.periodic * np.sin(2 * raw)) / self.increment)
            * self.increment
        )

    def update(self, data, dt):
        q = self.read(data)
        self.rates += -math.expm1(-dt / self.tau) * ((q - self.previous) / dt - self.rates)
        self.previous = q
        return q, self.rates.copy()


class SensorMeasurements:
    """Synthetic device bandwidth, quantization, noise and packet transport only."""

    def __init__(self, model, data, p, config, seed):
        self.p, self.c = p, config
        self.rng = np.random.default_rng(seed)
        self.dt = p["simulation"]["control_dt"]
        self.delay_steps = round(config["transport_delay_s"] / self.dt)
        self.queue = deque()
        self.encoder_names = [n + "_encoder" for n in JOINTS + WHEELS]
        self.gyro_bias = np.array(config["gyro_residual_bias_rad_s"])
        # Simulate a stationary pre-release gravity calibration. This synthesizes
        # a measurement, with the same bias/noise as the operating accelerometer.
        R = data.xmat[model.body("chassis").id].reshape(3, 3)
        calibration = R.T @ np.array([0, 0, p["sizing"]["gravity"]])
        calibration += self.accel_error()
        self.calibration = calibration.copy()
        self.gyro_filtered = data.sensor("gyro").data.copy()
        self.accel_filtered = calibration.copy()
        self.initial_packet = self.packet(data)

    def preprocess(self, data, dt):
        # Device bandwidth before 200 Hz downsampling reduces mechanical
        # vibration that could alias into the sampled attitude channels.
        for attribute, channel, key in [
            ("gyro_filtered", "gyro", "gyro_anti_alias_tau_s"),
            ("accel_filtered", "accelerometer", "accel_anti_alias_tau_s"),
        ]:
            filtered = getattr(self, attribute)
            filtered += -math.expm1(-dt / self.c[key]) * (data.sensor(channel).data - filtered)

    def accel_error(self):
        return np.array(self.c["accel_bias_m_s2"]) + self.rng.normal(
            0, self.c["accel_noise_std_m_s2"], 3
        )

    def encoders(self, data):
        raw = np.array([data.sensor(name).data[0] for name in self.encoder_names])
        raw[:4] += self.c.get("joint_encoder_bias_rad", [0, 0, 0, 0])
        raw[:4] += self.c.get("joint_encoder_periodic_error_rad", 0.0) * np.sin(2 * raw[:4])
        raw[4:] += self.c.get("wheel_encoder_periodic_error_rad", 0.0) * np.sin(raw[4:])
        increments = np.r_[
            np.full(4, 2 * math.pi / self.c["joint_encoder_counts_per_rev"]),
            np.full(2, 2 * math.pi / self.c["wheel_encoder_counts_per_rev"]),
        ]
        return np.round(raw / increments) * increments

    def packet(self, data):
        gyro = (
            self.gyro_filtered.copy()
            + self.gyro_bias
            + self.rng.normal(0, self.c["gyro_noise_std_rad_s"], 3)
        )
        accel = self.accel_filtered.copy() + self.accel_error()
        return gyro, accel, self.encoders(data)

    def next_packet(self, data):
        self.queue.append(self.packet(data))
        return self.queue.popleft() if len(self.queue) > self.delay_steps else self.initial_packet


class SensorEstimator(SensorMeasurements):
    def __init__(self, model, data, p, config, seed):
        super().__init__(model, data, p, config, seed)
        calibration = self.calibration
        self.pitch = math.atan2(-calibration[0], math.hypot(calibration[1], calibration[2]))
        self.roll = math.atan2(calibration[1], calibration[2])
        self.yaw = 0.0
        self.position = np.zeros(2)
        self.velocity = 0.0
        self.encoder_previous = self.encoders(data)
        self.posture_filtered = self.encoder_previous[:4].copy()
        self.rates = np.zeros(6)
        # Optional IMU-fused odometry (M10): the state is the IMU point's forward velocity.
        self.fusion_tau = config.get("imu_velocity_fusion_tau_s")
        if self.fusion_tau:
            self.imu_velocity = 0.0
            self.accel_bias = 0.0
            self.imu_acceleration = 0.0
            self.odometry = 0.0
            self.imu_offset = self.imu_horizontal_offset(self.encoder_previous[:4])
        # Set by the caller after each control step: the drive acceleration limit while
        # driving (a drive command or a brake to rest), 0 while holding.
        self.compensation_limit = 0.0

    def imu_horizontal_offset(self, q):
        """Forward horizontal offset of the IMU from the mean axle, at the estimated pitch."""
        _, axle = kinematics(self.p).com_and_axle(self.p, q)
        imu = self.p["physics"]["imu_position_m"]
        dx, dz = imu[0] - axle[0], imu[2] - axle[1]
        return math.cos(self.pitch) * dx + math.sin(self.pitch) * dz

    def update(self, data):
        gyro, accel, encoder = self.next_packet(data)
        dt = self.dt
        roll_rate = gyro[0] + math.tan(self.pitch) * (
            gyro[1] * math.sin(self.roll) + gyro[2] * math.cos(self.roll)
        )
        pitch_rate = gyro[1] * math.cos(self.roll) - gyro[2] * math.sin(self.roll)
        yaw_rate = (gyro[1] * math.sin(self.roll) + gyro[2] * math.cos(self.roll)) / math.cos(
            self.pitch
        )
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        self.pitch += pitch_rate * dt
        self.roll += roll_rate * dt
        self.yaw += yaw_rate * dt
        f = np.array(accel, dtype=float)
        if self.fusion_tau:
            # Gravity alone: remove the forward horizontal acceleration, whose body-frame
            # direction is (cos p, sin p sin r, sin p cos r), so a sustained acceleration
            # does not pull the attitude towards atan(a / g). Only a drive's acceleration:
            # in a push the low-passed wheel acceleration is not the IMU's.
            limit = max(self.compensation_limit, 0.0)
            a = min(max(self.imu_acceleration, -limit), limit)
            sp, cp = math.sin(self.pitch), math.cos(self.pitch)
            f -= a * np.array([cp, sp * sr, sp * cr])
        if abs(np.linalg.norm(f) - self.p["sizing"]["gravity"]) < self.c["accel_norm_gate_m_s2"]:
            alpha = -math.expm1(-dt / self.c["attitude_correction_tau_s"])
            self.pitch += alpha * (math.atan2(-f[0], math.hypot(f[1], f[2])) - self.pitch)
            self.roll += alpha * (math.atan2(f[1], f[2]) - self.roll)
        alpha = -math.expm1(-dt / self.c["velocity_filter_tau_s"])
        raw_rates = (encoder - self.encoder_previous) / dt
        joint_alpha = -math.expm1(-dt / self.c["joint_velocity_filter_tau_s"])
        self.rates[:4] += joint_alpha * (raw_rates[:4] - self.rates[:4])
        self.rates[4:] += alpha * (raw_rates[4:] - self.rates[4:])
        self.encoder_previous = encoder
        posture_tau = self.c.get("posture_filter_tau_s", 0)
        if posture_tau > 0:
            self.posture_filtered += -math.expm1(-dt / posture_tau) * (
                encoder[:4] - self.posture_filtered
            )
        else:
            self.posture_filtered = encoder[:4].copy()
        # Filter the complete rolling rate together so gyro and encoder terms
        # cannot acquire different filter phases and create false translation.
        native = kinematics(self.p)
        if native is not None:
            carrier = np.array(
                [
                    native.wheel_carrier_angle_rate(
                        encoder[i : i + 2], raw_rates[i : i + 2], self.p
                    )[1]
                    for i in [0, 2]
                ]
            )
            wheel_world_rate = raw_rates[4:] + gyro[1] - carrier
        else:
            wheel_world_rate = raw_rates[4:] + gyro[1] + raw_rates[:4].reshape(2, 2).sum(axis=1)
        raw_velocity = self.p["geometry"]["wheel_radius"] * np.mean(wheel_world_rate)
        if self.fusion_tau:
            # Predict with the forward horizontal specific force; correct towards wheel
            # odometry moved to the IMU point. The clipped innovation keeps hops and
            # spinning wheels from dragging the estimate.
            offset = self.imu_horizontal_offset(encoder[:4])
            offset_rate = (offset - self.imu_offset) / dt
            self.imu_offset = offset
            vertical = accel[1] * math.sin(self.roll) + accel[2] * math.cos(self.roll)
            self.imu_velocity += dt * (
                accel[0] * math.cos(self.pitch) + vertical * math.sin(self.pitch) - self.accel_bias
            )
            odometry = raw_velocity + offset_rate
            self.imu_acceleration += -math.expm1(-dt / self.fusion_tau) * (
                (odometry - self.odometry) / dt - self.imu_acceleration
            )
            self.odometry = odometry
            limit = self.c["imu_velocity_innovation_limit_m_s"]
            innovation = odometry - self.imu_velocity
            innovation = min(max(innovation, -limit), limit)
            self.imu_velocity += -math.expm1(-dt / self.fusion_tau) * innovation
            # Second state: forward accelerometer bias (sensor bias and pitch error x g),
            # so a constant bias leaves no steady velocity error.
            self.accel_bias -= dt * innovation / (self.fusion_tau * self.c["imu_accel_bias_tau_s"])
            raw_velocity = self.imu_velocity - offset_rate
        self.velocity += alpha * (raw_velocity - self.velocity)
        self.position += self.velocity * dt * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        delay = self.c["transport_delay_s"] + self.c["gyro_anti_alias_tau_s"]
        return {
            "pitch": self.pitch + pitch_rate * delay,
            "pitch_rate": float(pitch_rate),
            "roll": self.roll + roll_rate * delay,
            "yaw": self.yaw + yaw_rate * delay,
            "yaw_rate": float(yaw_rate),
            "position": self.position.copy(),
            "velocity": float(self.velocity),
            "q": self.posture_filtered.copy(),
            "qd": self.rates[:4].copy(),
            "wheel_speed": self.rates[4:].copy(),
            "accel_norm": float(np.linalg.norm(accel)),
        }
