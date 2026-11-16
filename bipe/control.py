"""Cascade feedback and finite motor response; no pose editing or support forces."""

import math

import numpy as np


def friction_compensation(torque, speed, gains):
    """Measured rotation sets sliding-friction direction; torque aids breakaway."""
    compensation = gains.get("wheel_friction_compensation", 0)
    stationary = np.tanh(torque / gains.get("friction_compensation_scale", 0.01))
    threshold = gains.get("friction_velocity_threshold_rad_s")
    if threshold is None:
        return compensation * stationary
    blend = np.minimum(np.abs(speed) / threshold, 1.0)
    moving = np.tanh(speed / (threshold / 4))
    return compensation * ((1 - blend) * stationary + blend * moving)


class CascadeController:
    def __init__(self, p, gains, equilibrium):
        self.p = p
        self.g = gains
        self.equilibrium = equilibrium
        self.integral = 0.0
        self.velocity_reference = 0.0
        self.hold_x = 0.0

    def command(self, s, user_velocity, dt):
        g = self.g
        # Capture stop location on the command transition, not every step.
        if (
            user_velocity == 0
            and self.velocity_reference != 0
            and not getattr(self, "stopping", False)
        ):
            self.hold_x = s["x"]
            self.stopping = True
        if user_velocity != 0:
            self.stopping = False
        target = user_velocity
        if user_velocity == 0:
            target = float(
                np.clip(
                    g["hold_kp"] * (self.hold_x - s["x"]), -g["max_hold_speed"], g["max_hold_speed"]
                )
            )
        step = g["max_command_acceleration"] * dt
        self.velocity_reference += float(np.clip(target - self.velocity_reference, -step, step))
        error = self.velocity_reference - s["velocity"]
        candidate = float(
            np.clip(
                self.integral + error * dt, -g["speed_integral_limit"], g["speed_integral_limit"]
            )
        )
        offset = g["speed_kp"] * error + g["speed_ki"] * candidate
        if abs(offset) < g["max_pitch_offset"] or error * offset < 0:
            self.integral = candidate
        offset = float(
            np.clip(
                g["speed_kp"] * error + g["speed_ki"] * self.integral,
                -g["max_pitch_offset"],
                g["max_pitch_offset"],
            )
        )
        reference = self.equilibrium + offset
        torque = g["pitch_kp"] * (s["pitch"] - reference) + g["pitch_kd"] * s["pitch_rate"]
        wheels = np.full(2, torque)
        wheels += friction_compensation(wheels, s["wheel_speed"], g)
        return wheels, reference, self.velocity_reference


class WheelMotors:
    def __init__(self, p):
        self.p = p["motor"]
        self.applied = np.zeros(2)

    def step(self, requested, speed, dt):
        p = self.p
        peak = p["peak_torque_per_wheel"]
        limited = np.clip(requested, -peak, peak)
        self.applied += (1 - math.exp(-dt / p["torque_time_constant"])) * (limited - self.applied)
        # Motoring torque tapers with speed; braking retains the peak limit.
        envelope = np.where(
            self.applied * speed > 0,
            peak * np.maximum(0, 1 - np.abs(speed) / p["no_load_speed"]),
            peak,
        )
        self.applied = np.clip(self.applied, -envelope, envelope)
        saturated = np.abs(requested) > envelope + 1e-8
        return self.applied.copy(), saturated
