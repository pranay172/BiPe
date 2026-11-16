"""M11 firmware hardware layer: generated outputs, IMU axes, joint law parity and the
supervisor driven by M11 hardware samples."""

import ctypes as c
import functools
import importlib.util
import io
import json
import math

import numpy as np
import pytest

from bipe.articulated import JOINTS, WHEELS
from bipe.locomotion import LocomotionController
from bipe.m11.firmware import (
    Axis,
    FdFrame,
    MotorSample,
    axes,
    build,
    imu_rotation,
    teensy_pins,
)
from bipe.parameters import ROOT, load_parameters

moteus = pytest.importorskip("moteus")
mp = pytest.importorskip("moteus.multiplex")
protocol = pytest.importorskip("moteus.protocol")

ROBOT = json.loads((ROOT / "config/robot.json").read_text())
GAINS = json.loads((ROOT / "config/controller_m10.json").read_text())
FW = ROBOT["m10"]["electronics"]["firmware"]
TWO_PI = 2 * math.pi


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build(tmp_path_factory.mktemp("m11"))


def generator():
    spec = importlib.util.spec_from_file_location("gen", ROOT / "tools/generate_m11_hardware.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def frame_from(data, can_id):
    f = FdFrame(id=can_id, size=len(data))
    for i, b in enumerate(data):
        f.data[i] = b
    return f


def test_committed_header_and_driver_configs_are_current():
    gen = generator()
    assert (ROOT / "firmware/include/m11_hardware.h").read_text() == gen.header(ROBOT, GAINS)
    for row in axes(ROBOT, GAINS):
        path = ROOT / f"artifacts/m11/firmware/moteus/{row['id']}-{row['name']}.cfg"
        assert path.read_text() == gen.moteus_config(ROBOT, GAINS, row)


def test_pins_use_the_teensy_peripherals_the_firmware_needs():
    pins = teensy_pins()
    # Teensy 4.0: CAN3 on the bottom pads 30 (CRX3) and 31 (CTX3); SPI0 on 11/12/13;
    # A0 is pin 14.
    assert (pins["CAN_RX"], pins["CAN_TX"]) == (30, 31)
    assert (pins["SPI_MOSI"], pins["SPI_MISO"], pins["SPI_SCK"]) == (11, 12, 13)
    assert pins["BUS_ADC"] == 14
    assert len(set(pins.values())) == len(pins)


def test_imu_axes_follow_the_datasheet_and_the_carrier_frame():
    r = imu_rotation(ROBOT["m10"])
    board = ROBOT["m10"]["electronics"]["boards"]["carrier"]
    # Chip Z is the copper side's normal; chip X runs along KiCad +x, chip Y along -y.
    assert r[:, 2] == pytest.approx(board["front_normal_body"])
    assert r[:, 0] == pytest.approx(board["board_x_axis_body"])
    assert r[:, 1] == pytest.approx(-np.array(board["board_y_axis_body"]))
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_lsm6dso_samples_map_to_the_body_frame(lib):
    imu = FW["imu"]
    r = imu_rotation(ROBOT["m10"])
    accel_lsb = imu["accel_mg_per_lsb"] * 1e-3 * imu["standard_gravity_m_s2"]
    gyro_lsb = math.radians(imu["gyro_mdps_per_lsb"] * 1e-3)
    rng = np.random.default_rng(3)
    for _ in range(500):
        counts = rng.integers(-32768, 32768, 6).astype(np.int16)
        raw = (c.c_uint8 * 12).from_buffer_copy(counts.astype("<i2").tobytes())
        gyro, accel = (c.c_double * 3)(), (c.c_double * 3)()
        lib.bipe_lsm6dso_body(raw, gyro, accel)
        assert np.array(gyro) == pytest.approx(r @ (counts[:3] * gyro_lsb), abs=1e-12)
        assert np.array(accel) == pytest.approx(r @ (counts[3:] * accel_lsb), abs=1e-12)
    regs, values = (c.c_uint8 * 4)(), (c.c_uint8 * 4)()
    assert lib.bipe_lsm6dso_setup(regs, values) == 4
    assert list(regs) == [0x12, 0x10, 0x11, 0x0D]
    assert list(values) == [0x44, 0x88, 0x84, 0x02]


def test_gravity_check_accepts_upright_and_rejects_large_tilt(lib):
    limit = math.radians(FW["imu"]["gravity_check_deg"])
    for angle, ok in [(0, True), (limit - 0.01, True), (limit + 0.01, False), (math.pi, False)]:
        a = (c.c_double * 3)(9.81 * math.sin(angle), 0, 9.81 * math.cos(angle))
        assert lib.bipe_m11_gravity_ok(a) == ok


def c_axis(lib, index, sign=None):
    table = (Axis * 6).in_dll(lib, "bipe_m11_axes")
    axis = Axis(table[index].id, table[index].wheel, table[index].sign, table[index].centre_rad)
    if sign is not None:
        axis.sign = sign
    return axis


def test_axis_table_matches_the_config(lib):
    table = (Axis * 6).in_dll(lib, "bipe_m11_axes")
    for row, axis in zip(axes(ROBOT, GAINS), table):
        assert (axis.id, axis.wheel, axis.sign) == (row["id"], row["wheel"], row["sign"])
        assert axis.centre_rad == pytest.approx(row["centre_rad"], abs=1e-15)
    assert [r["name"] for r in axes(ROBOT, GAINS)] == JOINTS + WHEELS


def test_joint_ranges_fit_the_rotor_encoder_window():
    """A 4:1 belt with a rotor encoder reads the output unambiguously over +-45 deg."""
    for row in axes(ROBOT, GAINS):
        if row["wheel"]:
            continue
        window = math.pi * row["rotor_to_output_ratio"]  # half a rotor turn at the output
        assert row["half_range_rad"] < window - math.radians(5)


def test_joint_frames_reproduce_the_plant_joint_law(lib):
    """moteus position mode with the generated kp, kd (N*m/rev) gives the plant's
    ff + Kp (qref - q) + Kd (qdref - qd) for the commanded frame (f32 rounding only)."""
    p = load_parameters(profile="m10")
    controller = LocomotionController(p, GAINS)
    rows = axes(ROBOT, GAINS)
    rng = np.random.default_rng(11)
    for trial in range(400):
        sign = 1.0 if trial % 2 else -1.0
        q, qd = np.empty(4), rng.normal(0, 1, 4)
        qref, qdref, ff = np.empty(4), rng.normal(0, 1, 4), rng.normal(0, 0.5, 4)
        actual = np.empty(4)
        for j in range(4):
            row = rows[j]
            q[j] = row["centre_rad"] + rng.uniform(-1, 1) * row["half_range_rad"]
            qref[j] = q[j] + rng.normal(0, 0.05)
            axis = c_axis(lib, j, sign)
            frame = FdFrame()
            assert lib.bipe_m11_joint_command(
                c.byref(axis), qref[j], qdref[j], ff[j], c.byref(frame)
            )
            assert frame.id == axis.id and frame.size <= 20  # no reply requested
            command = protocol.parse_registers(frame.payload()).command
            assert command[0] == moteus.Mode.POSITION
            # The driver's own measurement in output revolutions.
            position = sign * (q[j] - row["centre_rad"]) / TWO_PI
            velocity = sign * qd[j] / TWO_PI
            torque = (
                row["kp_Nm_per_rev"] * (command[0x20] - position)
                + row["kd_Nm_per_rev_s"] * (command[0x21] - velocity)
                + command[0x22]
            )
            actual[j] = sign * torque
        expected = controller.leg_torques({"q": q, "qd": qd}, qref, qdref, ff)
        assert actual == pytest.approx(expected, abs=2e-5)


def test_wheel_frames_carry_feedforward_torque_only(lib):
    for j in (4, 5):
        axis = c_axis(lib, j)
        frame = FdFrame()
        assert lib.bipe_m11_wheel_command(c.byref(axis), 0.2345, c.byref(frame))
        command = protocol.parse_registers(frame.payload()).command
        assert math.isnan(command[0x20])
        assert command[0x21] == 0 and command[0x23] == 0 and command[0x24] == 0
        assert command[0x22] == pytest.approx(0.2345 * axis.sign, rel=1e-7)
        assert not lib.bipe_m11_joint_command(c.byref(axis), 0.0, 0.0, 0.0, c.byref(frame))


def test_wheel_speed_frames_reproduce_the_policy_speed_loop(lib):
    """The learned controller's wheel frame: with the generated wheel kd the drive gives
    kv (target - speed) + ff, the M13 speed loop's proportional term at the drive's rate."""
    rows = axes(ROBOT, GAINS)
    from bipe.m13.headers import DEPLOYED, POLICIES

    kv = json.loads((ROOT / POLICIES[DEPLOYED]["export"] / "manifest.json").read_text())["action"][
        "wheels"
    ]["speed_loop"]["kv_Nm_s_per_rad"]
    rng = np.random.default_rng(5)
    for j in (4, 5):
        for sign in (1.0, -1.0):
            axis = c_axis(lib, j, sign)
            target, speed, ff = rng.normal(0, 30), rng.normal(0, 30), rng.normal(0, 0.3)
            frame = FdFrame()
            assert lib.bipe_m11_wheel_speed_command(c.byref(axis), target, ff, c.byref(frame))
            command = protocol.parse_registers(frame.payload()).command
            assert math.isnan(command[0x20]) and command[0x23] == 0 and command[0x24] == 1
            torque = rows[j]["kd_Nm_per_rev_s"] * (command[0x21] - sign * speed / TWO_PI)
            torque += command[0x22]
            assert sign * torque == pytest.approx(kv * (target - speed) + ff, rel=1e-5, abs=1e-6)
            assert not lib.bipe_m11_wheel_speed_command(
                c.byref(axis), math.nan, 0.0, c.byref(frame)
            )
    assert not lib.bipe_m11_wheel_speed_command(c.byref(c_axis(lib, 0)), 1.0, 0.0, c.byref(frame))


@functools.cache
def reply_bytes(mode, position, velocity, temperature, fault):
    """A driver reply in the default query's layout, written with the package's writers."""
    buf = io.BytesIO()
    w = protocol.Writer(buf)
    for start, fields in [
        (
            0x000,
            [
                (w.write, mode, mp.INT8),
                (w.write_position, position, mp.F32),
                (w.write_velocity, velocity, mp.F32),
                (w.write_torque, 0.1, mp.F32),
            ],
        ),
        (
            0x00D,
            [
                (w.write_voltage, 22.0, mp.INT8),
                (w.write_temperature, temperature, mp.INT8),
                (w.write, fault, mp.INT8),
            ],
        ),
    ]:
        combiner = mp.WriteCombiner(w, 0x20, start, [res for _, _, res in fields])
        for write, value, res in fields:
            if combiner.maybe_write():
                write(value, res)
    return buf.getvalue()


def reply(mode, position, velocity, temperature, fault, source):
    return frame_from(reply_bytes(mode, position, velocity, temperature, fault), source << 8)


def test_driver_replies_become_supervisor_samples(lib):
    for j, row in enumerate(axes(ROBOT, GAINS)):
        axis = c_axis(lib, j, -1.0 if j % 2 else 1.0)
        sample = MotorSample()
        f = reply(10, 0.0625, -0.5, 41, 0, axis.id)
        assert lib.bipe_m11_sample(c.byref(axis), c.byref(f), 1234, 7, c.byref(sample))
        base = 0.0 if row["wheel"] else row["centre_rad"]
        assert sample.position == pytest.approx(base + axis.sign * TWO_PI * 0.0625)
        assert sample.speed == pytest.approx(-axis.sign * TWO_PI * 0.5)
        assert (sample.id, sample.error, sample.temperature) == (axis.id, 0, 41)
        assert sample.present and sample.valid and sample.time_ms == 1234
        f = reply(1, 0.0, 0.0, 41, 33, axis.id)
        assert lib.bipe_m11_sample(c.byref(axis), c.byref(f), 1, 1, c.byref(sample))
        assert sample.error == 33
        f = reply(10, 0.0, 0.0, 41, 0, axis.id % 6 + 1)  # another driver's reply
        assert not lib.bipe_m11_sample(c.byref(axis), c.byref(f), 1, 1, c.byref(sample))


def test_bus_monitor_scale(lib):
    mon = ROBOT["m10"]["electronics"]["battery_monitor"]
    scale = (mon["upper_ohm"] + mon["lower_ohm"]) / mon["lower_ohm"]
    assert lib.bipe_m11_bus_volts(4095) == pytest.approx(3.3 * scale)
    assert lib.bipe_m11_bus_volts(1240) == pytest.approx(1240 * 3.3 / 4095 * scale)


# --- Supervisor driven by M11 samples ---------------------------------------------------


class Inputs(c.Structure):
    _fields_ = [
        ("imu_valid", c.c_bool),
        ("imu_present", c.c_bool),
        ("bus_off", c.c_bool),
        ("estop_closed", c.c_bool),
        ("power_good", c.c_bool),
        ("imu_time_ms", c.c_uint32),
        ("imu_sequence", c.c_uint32),
        ("host_time_ms", c.c_uint32),
        ("gyro_norm", c.c_double),
        ("acceleration_norm", c.c_double),
        ("attitude_error", c.c_double),
        ("bus_V", c.c_double),
        ("motor", MotorSample * 6),
    ]


class Settings(c.Structure):
    _fields_ = [
        (k, c.c_uint)
        for k in ("feedback_ms", "imu_ms", "host_ms", "stop_ms", "deadline_ms", "calibration_ms")
    ] + [
        (k, c.c_double)
        for k in (
            "bus_arm_min",
            "bus_arm_max",
            "bus_fault_min",
            "bus_fault_max",
            "temperature_max",
            "stationary_gyro",
            "stationary_joint_speed",
            "attitude_max",
        )
    ]


class Supervisor(c.Structure):
    _fields_ = [
        ("state", c.c_int),
        ("fault", c.c_int),
        ("last_tick", c.c_uint32),
        ("calibration_since", c.c_uint32),
        ("stop_since", c.c_uint32),
        ("last_imu_sequence", c.c_uint32),
        ("calibration_samples", c.c_uint),
        ("tick_seen", c.c_bool),
        ("calibrating_stationary", c.c_bool),
        ("start_requested", c.c_bool),
        ("arm_requested", c.c_bool),
        ("power_request", c.c_bool),
        ("torque_enabled", c.c_bool),
        ("fall_stop_ms", c.c_uint),
    ]


class Outputs(c.Structure):
    _fields_ = [("transceiver_on", c.c_bool), ("commands", c.c_bool), ("stop_frames", c.c_bool)]


STATES = [
    "OFF",
    "DISCOVERING",
    "CALIBRATING",
    "READY",
    "ACTIVE",
    "STOPPING",
    "FAULT",
    "FALLING",
    "FALLEN",
]
FAULTS = [
    "OK",
    "PROFILE",
    "IMU",
    "FEEDBACK",
    "BUS_OFF",
    "VOLTAGE",
    "TEMPERATURE",
    "ESTOP",
    "DEADLINE",
    "ATTITUDE",
    "ENCODER",
    "DRIVER",
    "HOST",
]


class Bench:
    """The supervisor fed through the M11 adapter: PGD, the bus ADC, the stop input,
    LSM6DSO samples and moteus replies."""

    def __init__(self, lib):
        self.lib, self.now, self.seq = lib, 0, 0
        lib.bipe_default_settings.restype = Settings
        lib.bipe_m11_outputs_for.restype = Outputs
        self.settings = lib.bipe_default_settings()
        self.s, self.inputs = Supervisor(), Inputs()
        lib.bipe_init(c.byref(self.s))
        self.mode, self.fault_code, self.reply = [0] * 6, [0] * 6, [True] * 6
        # 1530 counts is 23.4 V through the x19 divider.
        self.adc, self.power_good, self.stop_low = 1530, True, False

    def tick(self, ms=1, qualified=True):
        lib = self.lib
        for _ in range(ms):
            self.now += 1
            self.seq += 1
            i = self.inputs
            i.power_good, i.estop_closed = self.power_good, not self.stop_low
            i.bus_V = lib.bipe_m11_bus_volts(self.adc)
            raw = (c.c_uint8 * 12).from_buffer_copy(
                np.array([0, 0, 0, 0, -8198, 0], "<i2").tobytes()  # 1 g on chip -Y
            )
            gyro, accel = (c.c_double * 3)(), (c.c_double * 3)()
            lib.bipe_lsm6dso_body(raw, gyro, accel)
            i.imu_present = i.imu_valid = bool(lib.bipe_m11_gravity_ok(accel))
            i.imu_time_ms, i.imu_sequence, i.host_time_ms = self.now, self.seq, self.now
            i.gyro_norm, i.acceleration_norm = 0.0, float(np.linalg.norm(accel))
            i.attitude_error = 0.0
            for j in range(6):
                axis = c_axis(lib, j)
                if self.reply[j]:
                    f = reply(self.mode[j], 0.0, 0.0, 35, self.fault_code[j], axis.id)
                    assert lib.bipe_m11_sample(
                        c.byref(axis), c.byref(f), self.now, self.seq, c.byref(i.motor[j])
                    )
            lib.bipe_tick(c.byref(self.s), c.byref(self.settings), c.byref(i), self.now, qualified)

    @property
    def state(self):
        return STATES[self.s.state]

    def outputs(self, qualified=True):
        return self.lib.bipe_m11_outputs_for(c.byref(self.s), qualified, self.power_good)


def test_imu_bench_sample_is_upright(lib):
    # Chip -Y is body +z: at rest the specific force (+1 g, upward) reads -8198 counts
    # on chip Y and +9.81 m/s^2 along body z.
    raw = (c.c_uint8 * 12).from_buffer_copy(np.array([0, 0, 0, 0, -8198, 0], "<i2").tobytes())
    gyro, accel = (c.c_double * 3)(), (c.c_double * 3)()
    lib.bipe_lsm6dso_body(raw, gyro, accel)
    assert list(accel) == pytest.approx([0, 0, 9.808], abs=1e-3)
    assert lib.bipe_m11_gravity_ok(accel)


def run_to_active(bench):
    assert bench.state == "OFF" and not bench.outputs().transceiver_on
    bench.lib.bipe_start(c.byref(bench.s))
    bench.tick()
    assert bench.state in ("DISCOVERING", "CALIBRATING")
    assert bench.outputs().transceiver_on and bench.outputs().stop_frames
    bench.tick(bench.settings.calibration_ms + 50)
    assert bench.state == "READY"
    bench.lib.bipe_arm(c.byref(bench.s))
    bench.tick()
    assert bench.state == "ACTIVE"


def test_power_states_from_discovery_to_active(lib):
    bench = Bench(lib)
    run_to_active(bench)
    assert bench.outputs().commands and not bench.outputs().stop_frames
    # The hardware lock: an unqualified profile never sends torque frames.
    assert not bench.outputs(qualified=False).commands


@pytest.mark.parametrize(
    "fault,setup",
    [
        ("VOLTAGE", lambda b: setattr(b, "adc", 1100)),  # 16.8 V, under the 18 V cutoff
        ("VOLTAGE", lambda b: setattr(b, "power_good", False)),  # LM5060 power good lost
        ("ESTOP", lambda b: setattr(b, "stop_low", True)),
        ("DRIVER", lambda b: (b.mode.__setitem__(2, 1), b.fault_code.__setitem__(2, 33))),
        ("DRIVER", lambda b: b.mode.__setitem__(4, 11)),  # driver watchdog timeout
        ("FEEDBACK", lambda b: b.reply.__setitem__(5, False)),
    ],
)
def test_m11_faults_stop_torque(lib, fault, setup):
    bench = Bench(lib)
    run_to_active(bench)
    for j in range(6):
        bench.mode[j] = 10
    bench.tick()
    setup(bench)
    bench.tick(bench.settings.feedback_ms + 2)
    assert bench.state == "FAULT" and FAULTS[bench.s.fault] == fault
    out = bench.outputs()
    assert not out.commands
    # Without power good the drivers are unpowered and the transceiver stays in standby.
    assert out.transceiver_on == out.stop_frames == bench.power_good


def test_arming_needs_the_arm_voltage(lib):
    bench = Bench(lib)
    bench.adc = 1220  # 18.5 V: above the cutoff, below the 19.2 V arm minimum
    bench.lib.bipe_start(c.byref(bench.s))
    bench.tick(bench.settings.calibration_ms + 50)
    assert bench.state == "CALIBRATING"  # calibration also waits for the arm range
    bench.adc = 1350
    bench.tick(bench.settings.calibration_ms + 50)
    assert bench.state == "READY"
