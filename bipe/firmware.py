"""Host binding to the same portable C core compiled for the MCU.

The qualification override exists only in this host experiment API. The diagnostic
target has a compile-time inhibit and cannot use it to enable hardware motors.
"""

import ctypes as c

from .parameters import ROOT

STATES = ["OFF", "DISCOVERING", "CALIBRATING", "READY", "ACTIVE", "STOPPING", "FAULT", "FALLING", "FALLEN"]
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


class Motor(c.Structure):
    _fields_ = [
        ("present", c.c_bool),
        ("valid", c.c_bool),
        ("time_ms", c.c_uint32),
        ("sequence", c.c_uint32),
        ("position", c.c_double),
        ("speed", c.c_double),
        ("temperature", c.c_double),
        ("id", c.c_uint8),
        ("error", c.c_uint8),
    ]


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
        ("motor", Motor * 6),
    ]


class Settings(c.Structure):
    _fields_ = [
        (x, c.c_uint)
        for x in [
            "feedback_ms",
            "imu_ms",
            "host_ms",
            "stop_ms",
            "deadline_ms",
            "calibration_ms",
        ]
    ] + [
        (x, c.c_double)
        for x in [
            "bus_arm_min",
            "bus_arm_max",
            "bus_fault_min",
            "bus_fault_max",
            "temperature_max",
            "stationary_gyro",
            "stationary_joint_speed",
            "attitude_max",
        ]
    ]


class Supervisor(c.Structure):
    _fields_ = (
        [("state", c.c_int), ("fault", c.c_int)]
        + [
            (x, c.c_uint32)
            for x in ["last_tick", "calibration_since", "stop_since", "last_imu_sequence"]
        ]
        + [("calibration_samples", c.c_uint)]
        + [
            (x, c.c_bool)
            for x in [
                "tick_seen",
                "calibrating_stationary",
                "start_requested",
                "arm_requested",
                "power_request",
                "torque_enabled",
            ]
        ]
        + [("fall_stop_ms", c.c_uint)]
    )


class Observation(c.Structure):
    _fields_ = (
        [
            (x, c.c_double)
            for x in [
                "pitch",
                "pitch_rate",
                "yaw",
                "yaw_rate",
                "x",
                "y",
                "velocity",
                "equilibrium",
                "height_scale",
            ]
        ]
        + [("wheel_speed", c.c_double * 2)]
        + [(x, c.c_double * 4) for x in ["q", "qd", "qref", "qdref"]]
    )


class Balance(c.Structure):
    _fields_ = [
        (x, c.c_double)
        for x in [
            "integral",
            "vref",
            "hold_x",
            "hold_y",
            "previous_velocity",
            "heading",
        ]
    ] + [("hold_initialized", c.c_bool), ("common_lag", c.c_double)] + [
        (x, c.c_double) for x in [
            "slope_momentum",
            "slope_force",
            "wheel_sum",
            "slope_lean",
            "slope_offset",
        ]
    ] + [("slope_initialized", c.c_bool)] + [
        (x, c.c_double) for x in ["command_acceleration", "acceleration_lean"]
    ] + [("braking", c.c_bool)]


def core(path=None):
    lib = c.CDLL(str(path or ROOT / "firmware/build/libbipe.so"))
    for name in ["init", "start", "arm", "reset"]:
        function = getattr(lib, "bipe_" + name)
        function.argtypes = [c.POINTER(Supervisor)]
        function.restype = None
    lib.bipe_default_settings.argtypes = []
    lib.bipe_default_settings.restype = Settings
    lib.bipe_tick.argtypes = [
        c.POINTER(Supervisor),
        c.POINTER(Settings),
        c.POINTER(Inputs),
        c.c_uint32,
        c.c_bool,
    ]
    lib.bipe_tick.restype = None
    lib.bipe_balance_step.argtypes = [
        c.POINTER(Balance),
        c.POINTER(Observation),
        c.c_double,
        c.c_double,
        c.c_double,
        c.POINTER(c.c_double),
    ]
    lib.bipe_balance_step.restype = c.c_bool
    return lib


def healthy(now):
    now %= 2**32
    data = Inputs(
        imu_valid=True,
        imu_present=True,
        estop_closed=True,
        power_good=True,
        imu_time_ms=now,
        imu_sequence=now,
        host_time_ms=now,
        acceleration_norm=9.81,
        bus_V=24,
    )
    for i, motor in enumerate(data.motor):
        motor.present = motor.valid = True
        motor.time_ms = motor.sequence = now
        motor.id, motor.temperature = i + 1, 25
    return data


def activated(lib, start_ms=0):
    """Virtual stationary sensor stream, separate from any physical balance test."""
    state, settings = Supervisor(), lib.bipe_default_settings()
    lib.bipe_init(c.byref(state))
    lib.bipe_start(c.byref(state))
    for t in range(start_ms, start_ms + settings.calibration_ms + 2):
        data = healthy(t)
        lib.bipe_tick(c.byref(state), c.byref(settings), c.byref(data), t % 2**32, True)
    assert state.state == STATES.index("READY") and not state.torque_enabled
    lib.bipe_arm(c.byref(state))
    now = start_ms + settings.calibration_ms + 2
    data = healthy(now)
    lib.bipe_tick(c.byref(state), c.byref(settings), c.byref(data), now % 2**32, True)
    assert state.state == STATES.index("ACTIVE") and state.torque_enabled
    return state, settings, now
