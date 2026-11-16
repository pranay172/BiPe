"""Generate the M11 firmware hardware header and the moteus driver configurations.

.venv/bin/python tools/generate_m11_hardware.py

Writes firmware/include/m11_hardware.h (pins from the routed carrier, IMU registers and
axis rotation, bus monitor, driver table) and artifacts/m11/firmware/moteus/<name>.cfg
(moteus_tool "conf set" lines per driver). Sources: config/robot.json (SI),
config/controller_m10.json and the carrier KiCad files. A test checks that the committed
outputs match a fresh generation. With --m10-config PATH it writes only the M10
generated_config.h that the Teensy M11 image compiles against.
"""

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bipe.m11.firmware import (
    CARRIER_PCB,
    axes,
    imu_rotation,
    teensy_pins,
    write_m10_config,
)

HEADER = ROOT / "firmware/include/m11_hardware.h"
MOTEUS = ROOT / "artifacts/m11/firmware/moteus"
SOURCES = ["config/robot.json", "config/controller_m10.json", str(CARRIER_PCB.relative_to(ROOT))]


def number(x):
    return repr(float(x)) if isinstance(x, float) else str(x)


def header(robot, gains):
    m10 = robot["m10"]
    e, fw = m10["electronics"], m10["electronics"]["firmware"]
    imu, mon = fw["imu"], e["battery_monitor"]
    lines = [
        "/* Generated: tools/generate_m11_hardware.py; do not edit. */",
        "#ifndef BIPE_M11_HARDWARE_H",
        "#define BIPE_M11_HARDWARE_H",
        "/* Teensy 4.0 pins from the routed carrier (electronics/m11/carrier). */",
    ]
    lines += [f"#define BIPE_M11_PIN_{net} {pin}" for net, pin in teensy_pins().items()]
    can = e["drivers"]["can"]
    lines += [
        f"#define BIPE_M11_CAN_NOMINAL_BIT_S {can['arbitration_bit_s']}",
        f"#define BIPE_M11_CAN_DATA_BIT_S {can['data_bit_s']}",
        f"#define BIPE_M11_CAN_SOURCE {fw['host_can_source']}",
        "/* LSM6DSO (ST DS12140 Rev 3). */",
        f"#define BIPE_M11_IMU_SPI_HZ {imu['spi_rate_hz']}",
        f"#define BIPE_M11_IMU_WHO_AM_I 0x{imu['who_am_i']:02x}",
        f"#define BIPE_M11_IMU_CTRL3_C 0x{imu['ctrl3_c']:02x}",
        f"#define BIPE_M11_IMU_CTRL1_XL 0x{imu['ctrl1_xl']:02x}",
        f"#define BIPE_M11_IMU_CTRL2_G 0x{imu['ctrl2_g']:02x}",
        f"#define BIPE_M11_IMU_INT1_CTRL 0x{imu['int1_ctrl']:02x}",
        f"#define BIPE_M11_IMU_FIRST_OUTPUT 0x{imu['first_output_register']:02x}",
        (
            f"#define BIPE_M11_IMU_ACCEL_M_S2_PER_LSB "
            f"{number(imu['accel_mg_per_lsb'] * 1e-3 * imu['standard_gravity_m_s2'])}"
        ),
        (
            f"#define BIPE_M11_IMU_GYRO_RAD_S_PER_LSB "
            f"{number(imu['gyro_mdps_per_lsb'] * 1e-3 * math.pi / 180)}"
        ),
        f"#define BIPE_M11_IMU_GRAVITY_CHECK_RAD {number(math.radians(imu['gravity_check_deg']))}",
        "/* Body = R chip, row-major. */",
        "#define BIPE_M11_IMU_TO_BODY {"
        + ", ".join(number(float(v)) for v in imu_rotation(m10).ravel())
        + "}",
        "/* Bus monitor: divider into the Teensy ADC. */",
        f"#define BIPE_M11_BUS_ADC_SCALE {number((mon['upper_ohm'] + mon['lower_ohm']) / mon['lower_ohm'])}",
        f"#define BIPE_M11_ADC_REFERENCE_V {number(mon['adc_reference_V'])}",
        f"#define BIPE_M11_ADC_MAX_COUNTS {number(float(2 ** mon['adc_bits'] - 1))}",
        "/* Drivers in JOINTS + WHEELS order: {id, wheel, sign, centre_rad}. */",
        "#define BIPE_M11_AXES { \\",
    ]
    rows = axes(robot, gains)
    lines += [
        f"    {{{r['id']}, {int(r['wheel'])}, {number(r['sign'])}, {number(r['centre_rad'])}}}, \\"
        for r in rows
    ]
    lines += ["}"]
    lines.append(f"/* Inputs: {', '.join(SOURCES)}. tests/test_m11_firmware.py regenerates. */")
    lines += ["#endif", ""]
    return "\n".join(lines)


def moteus_config(robot, gains, row):
    """moteus_tool conf set lines (moteus docs/reference/configuration.md)."""
    m10 = robot["m10"]
    fw, drivers = m10["electronics"]["firmware"], m10["electronics"]["drivers"]
    sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
    encoder = m10["electronics"]["harness"]["encoder"]
    wheel = row["wheel"]
    settings = {
        "id.id": row["id"],
        "aux2.spi.mode": 2,
        "aux2.spi.rate_hz": encoder["spi_rate_hz"],
        "motor_position.sources.0.aux_number": 2,
        "motor_position.sources.0.type": 1,
        "motor_position.sources.0.cpr": 16384,
        "motor_position.sources.0.reference": 0,
        "motor_position.commutation_source": 0,
        "motor_position.output.source": 0,
        "motor_position.output.sign": 1,
        "motor_position.output.offset": 0.0,
        "motor_position.rotor_to_output_ratio": row["rotor_to_output_ratio"],
        "servo.pid_position.kp": row["kp_Nm_per_rev"],
        "servo.pid_position.ki": 0.0,
        "servo.pid_position.kd": row["kd_Nm_per_rev_s"],
        "servo.max_current_A": row["max_current_A"],
        "servo.default_timeout_s": fw["watchdog_timeout_s"],
        "servo.timeout_mode": fw["timeout_mode_wheels" if wheel else "timeout_mode_joints"],
        "servo.timeout_max_torque_Nm": 0.0 if wheel else fw["timeout_max_torque_Nm"],
        **drivers["settings"],
    }
    if not wheel:
        tau = sensors["local_joint_velocity_filter_tau_s"]
        settings["motor_position.sources.0.pll_filter_hz"] = 1 / (2 * math.pi * tau)
        half = row["half_range_rad"] / (2 * math.pi)
        settings["servopos.position_min"] = -half
        settings["servopos.position_max"] = half
    notes = [
        f"# {row['name']}: moteus-c1, generated by tools/generate_m11_hardware.py.",
        "# Commissioning: motor_position.output.sign and .offset are set on the bench so",
        "# that the joint's end-stop centre reads 0 (joints) and the sign matches the plant;",
        "# then rerun moteus_tool --calibrate. Gains are N*m per output revolution.",
    ]
    body = [f"conf set {k} {number(v) if isinstance(v, float) else v}" for k, v in settings.items()]
    return "\n".join(notes + body + ["conf write", ""])


def main():
    if sys.argv[1:2] == ["--m10-config"]:
        # The Teensy M11 image's generated_config.h (firmware/Makefile teensy-m11).
        write_m10_config(sys.argv[2])
        return
    robot = json.loads((ROOT / "config/robot.json").read_text())
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    HEADER.write_text(header(robot, gains))
    MOTEUS.mkdir(parents=True, exist_ok=True)
    for row in axes(robot, gains):
        (MOTEUS / f"{row['id']}-{row['name']}.cfg").write_text(moteus_config(robot, gains, row))
    print("M11 HARDWARE", HEADER.relative_to(ROOT), MOTEUS.relative_to(ROOT))


if __name__ == "__main__":
    main()
