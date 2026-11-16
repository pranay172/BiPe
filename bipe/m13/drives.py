"""RO40 and GL40 drive models as array code, for the training plant and the GPU environment.

The equations are those of ``bipe.m10.plant.MechanicalDrives._hip_motor`` and
``_hub_motor``: command caps, the torque-speed and drive-voltage limits, the FOC
current lag and slew and the catalog torque-current curve. Every function takes
its array module (``numpy`` or ``torch``) and works on any leading batch shape,
with the actuators on the last axis. Phase currents are returned, never stored.
"""

import math


def _lag(xp, dt, tau):
    """First-order lag fraction over dt; tau may be a float or a per-world array."""
    if isinstance(tau, (int, float)):
        return -math.expm1(-dt / tau)
    return -xp.expm1(-dt / tau)


def hip_motor(xp, hm, ratio, requested, rotor_speed, current, dt):
    """RO40 through the 4:1 belt. Returns (motor torque, output torque, saturated, current).

    requested is the output torque request, rotor_speed the rotor's speed, current the
    phase current state.
    """
    cap = hm["normal_command_cap_Nm"]
    peak, rated = hm["peak_torque_motor_Nm"], hm["continuous_torque_motor_Nm"]
    ir, ip = hm["rated_current_A"], hm["peak_current_A"]
    efficiency = hm["efficiency_screen"]
    command = xp.clip(requested, -cap, cap) / (ratio * efficiency)
    derated = peak * xp.clip(1 - xp.abs(rotor_speed) / hm["no_load_speed_motor_rad_s"], 0, None)
    maximum = xp.where(command * rotor_speed > 0, derated, xp.full_like(derated, peak))
    command = xp.clip(command, -maximum, maximum)
    size = xp.abs(command)
    desired = xp.sign(command) * xp.where(
        size <= rated, size * ir / rated, ir + (size - rated) * (ip - ir) / (peak - rated)
    )
    slew = hm["FOC_current_slew_limit_A_s"] * dt
    change = _lag(xp, dt, hm["FOC_time_constant_s"]) * (desired - current)
    current = current + xp.clip(change, -slew, slew)
    level = xp.abs(current)
    torque = xp.sign(current) * xp.where(
        level <= ir, level * rated / ir, rated + (level - ir) * (peak - rated) / (ip - ir)
    )
    torque = xp.clip(torque, -maximum, maximum)
    saturated = xp.abs(requested) > xp.clip(maximum * ratio * efficiency, None, cap) + 1e-8
    return torque, torque * ratio * efficiency, saturated, current


def hub_resistance(gl):
    """GL40 line-to-line resistance for the voltage limit (as bipe.m10.plant.hub_drive_resistance)."""
    rise = gl["voltage_limit_winding_temperature_C"] - 20
    hot = 1 + gl["copper_resistance_tempco_per_C"] * rise
    return gl["phase_to_phase_resistance_ohm"] * hot + gl["lead_resistance_ohm"]


def hub_motor(xp, gl, requested, speed, current, dt):
    """GL40 direct drive. Returns (torque, saturated, current, voltage limited)."""
    kt, ip = gl["torque_constant_Nm_A"], gl["peak_current_A"]
    volt = (gl["drive_voltage_limit_V"] - gl["back_emf_V_s_rad"] * xp.abs(speed)) / hub_resistance(
        gl
    )
    limit = xp.where(requested * speed > 0, xp.clip(volt, 0, ip), xp.full_like(volt, ip))
    desired = xp.clip(requested / kt, -limit, limit)
    slew = gl["FOC_current_slew_limit_A_s"] * dt
    change = _lag(xp, dt, gl["FOC_time_constant_s"]) * (desired - current)
    current = xp.clip(current + xp.clip(change, -slew, slew), -limit, limit)
    return kt * current, xp.abs(requested) > kt * limit + 1e-8, current, limit < ip
