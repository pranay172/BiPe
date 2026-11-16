"""Posture-aware balance, planar hold, heading and physical leg torque commands."""

import math

import numpy as np

from .control import friction_compensation
from .kinematics import com_and_axle, equilibrium, posture, support_feedforward
from .profiles import native


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def shape_reference(vref, acceleration, target, limit, jerk, dt):
    """Jerk-limited velocity reference (M12 option): the acceleration slews at the jerk
    limit toward the largest value that still ramps it to zero at the target, so the
    reference reaches the target without overshoot and its lean feedforward is continuous."""
    error = target - vref
    step = jerk * dt
    if abs(error) <= step * dt and abs(acceleration) <= step:
        return target, 0.0
    remaining = error - acceleration * abs(acceleration) / (2 * jerk)
    want = math.copysign(math.sqrt(2 * jerk * abs(remaining)), remaining)
    acceleration += float(np.clip(np.clip(want, -limit, limit) - acceleration, -step, step))
    return vref + acceleration * dt, acceleration


class TorqueMotors:
    def __init__(self, parameters, count, peak_key):
        self.p = parameters
        self.peak = parameters[peak_key]
        self.applied = np.zeros(count)

    def step(self, requested, speed, dt):
        limited = np.clip(requested, -self.peak, self.peak)
        self.applied += -math.expm1(-dt / self.p["torque_time_constant"]) * (limited - self.applied)
        envelope = np.where(
            self.applied * speed > 0,
            self.peak * np.maximum(0, 1 - np.abs(speed) / self.p["no_load_speed"]),
            self.peak,
        )
        self.applied = np.clip(self.applied, -envelope, envelope)
        return self.applied.copy(), np.abs(requested) > envelope + 1e-8


class LocomotionController:
    def __init__(self, p, gains):
        self.p, self.g = p, gains
        self.integral = 0.0
        self.vref = 0.0
        self.hold_position = None
        self.previous_velocity_command = 0.0
        self.heading = 0.0
        self.common_lag = 0.0
        self.command_acceleration = 0.0
        self.acceleration_lean = 0.0
        self.braking = False
        self.moving = False
        from .protective_fall import ProtectiveFall

        self.protection = (
            ProtectiveFall(p, gains) if p.get("physics", {}).get("protective_fall") else None
        )
        self.previous_request = np.zeros(6)
        com, axle = com_and_axle(p, posture(40, p))
        self.nominal_height = com[1] - axle[1]
        self.slope_momentum = None
        self.slope_force = 0.0
        self.slope_offset = 0.0
        self.slope_torque = 0.0
        self.slope_state = 0.0

    def slope_lean(self, s, center, axle, dt):
        """Static slope lean from a whole-robot momentum observer (M10 option).

        F = (M (v + dd/dt) - p) / tau with dp/dt = sum(previous wheel torque) / r + F,
        using the wheel torque before friction compensation (the hub friction absorbs it),
        where d is the CoM's horizontal offset from the axle, so pitching and leg
        motion are not mistaken for an external force. The observed force gives
        sin(slope) = -F / (M g); the lean puts the CoM over the tilted contact point,
        less the lean the wheel-torque term itself needs. The lean is slew limited so
        that a push (an impulsive external force) cannot step it.
        """
        g, p = self.g, self.p
        tau = g.get("slope_observer_tau_s")
        if not tau:
            return 0.0
        mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())
        radius, gravity = p["geometry"]["wheel_radius"], p["sizing"]["gravity"]
        offset = math.cos(s["pitch"]) * (center[0] - axle[0]) + math.sin(s["pitch"]) * (
            center[1] - axle[1]
        )
        if self.slope_momentum is None:
            self.slope_momentum = mass * s["velocity"]
            self.slope_offset = offset
        offset_rate = (offset - self.slope_offset) / dt
        self.slope_offset = offset
        self.slope_momentum += dt * (self.slope_torque / radius + self.slope_force)
        self.slope_force = (mass * (s["velocity"] + offset_rate) - self.slope_momentum) / tau
        height = center[1] - axle[1]
        limit = math.sin(g["slope_feedforward_limit_rad"])
        sine = float(np.clip(-self.slope_force / (mass * gravity), -limit, limit))
        target = (
            math.asin(radius * sine / height) - mass * gravity * radius * sine / 2 / g["pitch_kp"]
        )
        step = g["slope_lean_rate_limit_rad_s"] * dt
        self.slope_state += float(np.clip(target - self.slope_state, -step, step))
        return self.slope_state

    @property
    def compensation_limit(self):
        """The estimator's acceleration compensation limit: the drive limit while driving."""
        return self.g["max_command_acceleration"] if self.moving else 0.0

    def command(self, s, velocity, yaw_rate, qref, qdref, dt):
        if self.protection is not None:
            protective = self.protection.step(s, self.previous_request, dt)
            if self.protection.latched:
                self.moving = False
                self.previous_request = protective.copy()
                return protective, s["pitch"], self.vref, self.heading
        g = self.g
        # With a jerk limit, a stop command first brakes the reference to rest at the
        # acceleration limit and then holds where it is, instead of holding the position
        # at the command (which, at speed, lies far behind the stopping point).
        jerk = g.get("command_jerk_m_s3", 0.0)
        stopping = velocity == 0 and self.previous_velocity_command != 0
        settled = self.braking and self.vref == 0 and self.command_acceleration == 0
        if jerk > 0:
            self.braking = (self.braking or stopping) and velocity == 0 and not settled
        moving = velocity != 0 or self.braking
        self.moving = moving
        pitch_kd = g["pitch_kd"]
        if native(self.p) and moving:
            pitch_kd = g.get("drive_pitch_kd", pitch_kd)
        speed_kp, speed_ki = g["speed_kp"], g["speed_ki"]
        if native(self.p) and not moving:
            speed_kp = g.get("hold_speed_kp", speed_kp)
            speed_ki = g.get("hold_speed_ki", speed_ki)
        if self.hold_position is None or (stopping and not self.braking) or settled:
            self.hold_position = s["position"].copy()
        self.previous_velocity_command = velocity
        self.heading += yaw_rate * dt
        forward = np.array([math.cos(s["yaw"]), math.sin(s["yaw"])])
        target = velocity
        if not moving:
            target = float(
                np.clip(
                    g["hold_kp"] * ((self.hold_position - s["position"]) @ forward),
                    -g["max_hold_speed"],
                    g["max_hold_speed"],
                )
            )
        if jerk > 0 and moving:
            self.vref, self.command_acceleration = shape_reference(
                self.vref,
                self.command_acceleration,
                target,
                g["max_command_acceleration"],
                jerk,
                dt,
            )
            # The lean that carries the reference acceleration, tan(lean) = a / g.
            self.acceleration_lean = g.get("acceleration_feedforward", 0.0) * math.atan(
                self.command_acceleration / self.p["sizing"]["gravity"]
            )
        else:
            # The hold reference is the position loop's output, not a command: it keeps
            # the plain rate limit, without the jerk limit and the feedforward.
            limit = g["max_command_acceleration"]
            if not moving:
                limit = g.get("hold_command_acceleration", limit)
            self.command_acceleration = self.acceleration_lean = 0.0
            step = limit * dt
            self.vref += float(np.clip(target - self.vref, -step, step))
        error = self.vref - s["velocity"]
        candidate = float(
            np.clip(
                self.integral + error * dt, -g["speed_integral_limit"], g["speed_integral_limit"]
            )
        )
        lean = speed_kp * error + speed_ki * candidate
        lqr = g.get("lqr_schedule")
        wheel_gate = True
        slope = 0.0
        if native(self.p):
            center, axle_now = com_and_axle(self.p, s["q"])
            slope = self.slope_lean(s, center, axle_now, dt)
            height_scale = (
                (center[1] - axle_now[1]) / self.nominal_height
                if g["posture_gain_schedule"]
                else 1.0
            )
            candidate_common = (
                g["pitch_kp"]
                * (
                    s["pitch"]
                    - equilibrium(self.p, s["q"])
                    - slope
                    - self.acceleration_lean
                    - np.clip(lean, -g["max_pitch_offset"], g["max_pitch_offset"])
                )
                + height_scale * pitch_kd * s["pitch_rate"]
            )
            wheel_gate = (
                abs(candidate_common) < self.p["motor"]["peak_torque_per_wheel"]
                or (-g["pitch_kp"] * speed_ki * error) * candidate_common < 0
            )
        if not lqr and wheel_gate and (abs(lean) < g["max_pitch_offset"] or error * lean < 0):
            self.integral = candidate
        lean = float(
            np.clip(
                speed_kp * error + speed_ki * self.integral,
                -g["max_pitch_offset"],
                g["max_pitch_offset"],
            )
        )
        reference = equilibrium(self.p, s["q"]) + slope + self.acceleration_lean + lean
        com, axle = com_and_axle(self.p, s["q"])
        scale = (com[1] - axle[1]) / self.nominal_height if g["posture_gain_schedule"] else 1.0
        # Proportional torque retains its gravity margin at low postures.
        # Rate damping follows the shorter pendulum's smaller lever arm.
        common = g["pitch_kp"] * (s["pitch"] - reference) + scale * pitch_kd * s["pitch_rate"]
        if lqr:
            reference = equilibrium(self.p, s["q"])
            K = np.array(
                [
                    np.interp(scale, [r["height_scale"] for r in lqr], [r["K"][i] for r in lqr])
                    for i in range(9)
                ]
            )
            common = -float(
                K
                @ [
                    s["pitch"] - reference,
                    s["pitch_rate"],
                    -error,
                    self.common_lag,
                    candidate,
                    float(np.mean((s["q"] - qref)[::2])),
                    float(np.mean((s["q"] - qref)[1::2])),
                    float(np.mean((s["qd"] - qdref)[::2])),
                    float(np.mean((s["qd"] - qdref)[1::2])),
                ]
            )
            peak = self.p["motor"]["peak_torque_per_wheel"]
            if abs(common) < peak or (-K[4] * error) * common < 0:
                self.integral = candidate
            common += K[4] * (candidate - self.integral)
        differential = g["heading_kp"] * wrap(self.heading - s["yaw"]) + g["yaw_rate_kd"] * (
            yaw_rate - s["yaw_rate"]
        )
        available = max(0.0, self.p["motor"]["peak_torque_per_wheel"] - abs(common))
        differential = float(np.clip(differential, -available, available))
        wheels = np.array([common - differential, common + differential])
        peak = self.p["motor"]["peak_torque_per_wheel"]
        self.slope_torque = float(np.clip(wheels, -peak, peak).sum())
        # Smooth breakaway compensation acts only through finite motor torque.
        # Its coefficient must later be measured on the selected gearbox.
        wheels += friction_compensation(wheels, s["wheel_speed"], g)
        peak = self.p["motor"]["peak_torque_per_wheel"]
        wheels = np.clip(wheels, -peak, peak)
        self.common_lag += -math.expm1(-dt / self.p["motor"]["torque_time_constant"]) * (
            float(wheels.mean()) - self.common_lag
        )
        feedforward = None
        reaction = self.g.get("knee_wheel_reaction", 0.0)
        if reaction and native(self.p) and self.p.get("profile") == "m10":
            # The hub's stator sits in the shin, held by the 1:1 knee belt from the knee
            # drive (axis -y against the wheel's +y): cancel the wheel torque's reaction.
            feedforward = support_feedforward(self.p, s["q"], s["pitch"], s["roll"])
            feedforward[[1, 3]] -= reaction * wheels
        legs = self.leg_torques(s, qref, qdref, feedforward)
        self.previous_request = np.r_[legs, wheels]
        return self.previous_request.copy(), reference, self.vref, self.heading

    def leg_torques(self, s, qref, qdref, feedforward=None):
        if self.protection is not None and self.protection.latched:
            return self.protection.leg_torques(s["q"], s["qd"])
        if feedforward is None:
            feedforward = support_feedforward(self.p, s["q"], s["pitch"], s["roll"])
        return (
            feedforward
            + self.g["joint_kp"] * (qref - s["q"])
            + self.g["joint_kd"] * (qdref - s["qd"])
        )
