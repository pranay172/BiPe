"""Generate MCU constants from the shared SI model and saved controller configuration."""

import hashlib
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def generate(robot=None, controller=None, output=None):
    provided_robot, provided_controller = robot, controller
    robot = robot or json.loads(
        Path(os.environ.get("BIPE_ROBOT_CONFIG") or ROOT / "config/robot.json").read_text()
    )
    fivebar = robot.get("profile") == "m9"
    serial = robot.get("profile") == "m10"
    controller_name = (
        "controller_m9" if fivebar else "controller_m10" if serial else "controller_moving"
    )
    controller = controller or json.loads((ROOT / f"config/{controller_name}.json").read_text())
    profile = json.loads((ROOT / "config/actuator-profile.json").read_text())
    electronics = json.loads((ROOT / "config/electronics.json").read_text())
    constants = {
        key.upper(): value
        for key, value in controller.items()
        if isinstance(value, (int, float)) and not isinstance(value, bool)
    }
    constants.setdefault("HOLD_SPEED_KP", controller["speed_kp"])
    constants.setdefault("HOLD_SPEED_KI", controller["speed_ki"])
    constants.setdefault("DRIVE_PITCH_KD", controller["pitch_kd"])
    # Optional M10 slope lean; zero disables it in the portable core.
    constants.setdefault("SLOPE_OBSERVER_TAU_S", 0.0)
    constants.setdefault("SLOPE_FEEDFORWARD_LIMIT_RAD", 0.0)
    constants.setdefault("SLOPE_LEAN_RATE_LIMIT_RAD_S", 0.0)
    # Optional M10 knee feedforward against the wheel torque's reaction; zero disables it.
    constants.setdefault("KNEE_WHEEL_REACTION", 0.0)
    constants.update(
        WHEEL_PEAK=robot["motor"]["peak_torque_per_wheel"],
        WHEEL_TAU=robot["motor"]["torque_time_constant"],
        DRIVE_SPEED=robot["sizing"]["drive_speed"],
        GRAVITY=robot["sizing"]["gravity"],
        DISCOVERY_MS=electronics["timing"]["discovery_timeout_ms"],
        STATIONARY_WHEEL_SPEED=electronics["limits"]["maximum_stationary_wheel_speed_rad_s"],
        STATIONARY_ACCEL_TOLERANCE=electronics["limits"]["stationary_acceleration_tolerance_m_s2"],
    )
    settings = {
        "FEEDBACK_MS": electronics["timing"]["feedback_timeout_ms"],
        "IMU_MS": electronics["timing"]["imu_timeout_ms"],
        "HOST_MS": electronics["timing"]["host_timeout_ms"],
        "STOP_MS": electronics["timing"]["stop_duration_ms"],
        "DEADLINE_MS": electronics["timing"]["deadline_timeout_ms"],
        "CALIBRATION_MS": electronics["timing"]["calibration_ms"],
        "BUS_ARM_MIN": electronics["limits"]["bus_minimum_arm_V"],
        "BUS_ARM_MAX": electronics["limits"]["bus_maximum_arm_V"],
        "BUS_FAULT_MIN": electronics["limits"]["bus_fault_low_V"],
        "BUS_FAULT_MAX": electronics["limits"]["bus_fault_high_V"],
        "TEMPERATURE_MAX": electronics["limits"]["driver_fault_temperature_C"],
        "STATIONARY_GYRO": electronics["limits"]["maximum_stationary_gyro_rad_s"],
        "STATIONARY_JOINT_SPEED": electronics["limits"]["maximum_stationary_joint_speed_rad_s"],
        "ATTITUDE_MAX": electronics["limits"]["maximum_attitude_error_rad"],
    }
    constants.update(settings)
    constants["BUS_ADC_SCALE"] = (
        electronics["adc"]["upper_ohm"] + electronics["adc"]["lower_ohm"]
    ) / electronics["adc"]["lower_ohm"]
    constants["ADC_REFERENCE_V"] = electronics["adc"]["reference_V_assumption"]
    constants["ADC_MAX_COUNTS"] = 2 ** electronics["adc"]["resolution_bits"] - 1
    constants.update(
        MAX_ENCODER_SPEED=max(
            robot["motor"]["no_load_speed"], robot["articulation"]["no_load_speed"]
        ),
        LINK_LENGTH=robot["geometry"].get("link_length", robot["geometry"].get("crank")),
        WHEEL_RADIUS=robot["geometry"]["wheel_radius"],
        TOTAL_MASS=sum(r["mass_kg"] for r in robot["rigid_bodies"].values()),
        JOINT_PEAK=robot["articulation"]["peak_torque_per_joint"],
        HIP_MIN=math.radians(robot["articulation"]["hip_limits_deg"][0]),
        HIP_MAX=math.radians(robot["articulation"]["hip_limits_deg"][1]),
        KNEE_MIN=math.radians(robot["articulation"]["knee_limits_deg"][0]),
        KNEE_MAX=math.radians(robot["articulation"]["knee_limits_deg"][1]),
        POSTURE_GAIN_SCHEDULE=int(controller["posture_gain_schedule"]),
    )
    sensor_name = "sensors-m9" if fivebar else "sensors-m10" if serial else "sensors-m4"
    sensors = json.loads((ROOT / f"config/{sensor_name}.json").read_text())
    if fivebar:
        g, joint, wheel = robot["geometry"], robot["joint_drive"], robot["drive"]
        h, half = g["nominal_height"], g["base"] / 2
        distance = math.hypot(half, h)
        bend = math.acos(
            (distance**2 + g["crank"] ** 2 - g["rod"] ** 2) / (2 * distance * g["crank"])
        )
        constants.update(
            FIVEBAR_BASE=g["base"],
            FIVEBAR_CRANK=g["crank"],
            FIVEBAR_ROD=g["rod"],
            FIVEBAR_NOMINAL_HEIGHT=g["nominal_height"],
            FIVEBAR_MIN_HEIGHT=g["height_range"][0],
            FIVEBAR_MAX_HEIGHT=g["height_range"][1],
            FIVEBAR_CROUCH_HEIGHT=g["crouched_height"],
            FIVEBAR_HIP_RATIO=joint["ratio"],
            FIVEBAR_HIP_MOTOR_Z=joint["motor_z"],
            FIVEBAR_WHEEL_RATIO=wheel["large_teeth"] / wheel["small_teeth"],
            FIVEBAR_WHEEL_MOTOR_X=g["rod"] + wheel["motor_along_rod_from_foot"],
            FIVEBAR_WHEEL_MOTOR_Z=wheel["motor_perpendicular"],
            FIVEBAR_NOMINAL_Q0=math.atan2(-h, half) - bend,
            FIVEBAR_NOMINAL_Q1=math.atan2(-h, -half) + bend,
        )
    if serial:
        g, mapping = robot["geometry"], robot["posture_mapping"]
        constants.update(
            SERIAL_THIGH=g["thigh_length"],
            SERIAL_SHIN=g["shin_length"],
            SERIAL_HIP_RATIO=robot["joint_drive"]["ratio"],
            SERIAL_MOTOR_X=g["motor_row_x"],
            SERIAL_MOTOR_Z=g["motor_row_z"],
            **{f"SERIAL_HEIGHT{i}": h for i, h in enumerate(mapping["axle_depth_m"])},
            **{f"SERIAL_AXLE_X{i}": x for i, x in enumerate(mapping["axle_x_m"])},
            IMU_X=robot["physics"]["imu_position_m"][0],
            IMU_Z=robot["physics"]["imu_position_m"][2],
        )
    for key in [
        "velocity_filter_tau_s",
        "joint_velocity_filter_tau_s",
        "posture_filter_tau_s",
        "attitude_correction_tau_s",
        "accel_norm_gate_m_s2",
        "local_joint_velocity_filter_tau_s",
    ]:
        constants[key.upper()] = sensors[key]
    # Optional IMU-fused odometry (serial profile); zero disables it.
    for key in [
        "imu_velocity_fusion_tau_s",
        "imu_velocity_innovation_limit_m_s",
        "imu_accel_bias_tau_s",
    ]:
        constants[key.upper()] = sensors.get(key, 0.0)
    constants["PREDICTION_DELAY"] = sensors["transport_delay_s"] + sensors["gyro_anti_alias_tau_s"]
    constants["CONTROL_DT"] = robot["simulation"]["control_dt"]
    for name, row in robot["rigid_bodies"].items():
        constants[name.upper() + "_MASS"] = row["mass_kg"]
        constants[name.upper() + "_X"] = row["com_m"][0]
        constants[name.upper() + "_Z"] = row["com_m"][2]
    # A boolean alone cannot qualify an incomplete exact-variant profile.
    qualified = (
        not fivebar
        and profile["hardware_qualified"]
        and all(
            profile.get(key)
            for key in [
                "command_ranges",
                "minimum_bus_voltage_V",
                "maximum_bus_voltage_V",
                "output_torque_per_iq_Nm_A",
                "calibration_and_startup_verified",
                "local_pd_mapping_verified",
                "command_timeout_verified",
                "feedback_position_convention_verified",
                "joint_mapping_verified",
            ]
        )
    )
    qualified = qualified and not serial
    lines = [
        "/* Generated: tools/generate_firmware_config.py; do not edit. */",
        "#ifndef BIPE_GENERATED_CONFIG_H",
        "#define BIPE_GENERATED_CONFIG_H",
        f"#define BIPE_FIVEBAR {int(fivebar)}",
        f"#define BIPE_SERIAL {int(serial)}",
        f"#define BIPE_HARDWARE_QUALIFIED {int(bool(qualified))}",
    ]
    for key, value in constants.items():
        literal = f"{float(value):.17g}"
        if "." not in literal and "e" not in literal:
            literal += ".0"
        lines.append(f"#define BIPE_{key} {literal}")
    rows = controller.get("lqr_schedule", [])
    lines.append(f"#define BIPE_BALANCE_LQR {int(bool(rows))}")
    if rows:
        lines.append(f"#define BIPE_LQR_ROWS {len(rows)}")
        lines.append("static const double bipe_lqr_schedule[][10] = {")
        for row in rows:
            lines.append(
                "    {" + ",".join(f"{x:.17g}" for x in [row["height_scale"], *row["K"]]) + "},"
            )
        lines.append("};")
    for name in ["robot", controller_name, "electronics", "actuator-profile", sensor_name]:
        inline = (
            provided_robot
            if name == "robot"
            else provided_controller
            if name == controller_name
            else None
        )
        if inline is not None:
            digest = hashlib.sha256(json.dumps(inline, sort_keys=True).encode()).hexdigest()
            lines.append(f"/* candidate {name} canonical JSON sha256: {digest} */")
            continue
        source = (
            Path(os.environ["BIPE_ROBOT_CONFIG"])
            if name == "robot" and os.environ.get("BIPE_ROBOT_CONFIG")
            else ROOT / f"config/{name}.json"
        )
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        lines.append(f"/* config/{name}.json sha256: {digest} */")
    lines += ["#endif", ""]
    path = Path(output or ROOT / "firmware/include/generated_config.h")
    path.write_text("\n".join(lines))
    return path


if __name__ == "__main__":
    print(generate())
