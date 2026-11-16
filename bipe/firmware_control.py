"""Sensor-only host adapter to the complete portable C estimator/control core.

Python supplies device measurements, transport and physics; all attitude, odometry,
kinematics, mass feedforward, joint rate estimation and torque commands run in C.
The local 1 kHz leg PD and transport remain modeled behavior pending hardware.
"""

import ctypes as c
import hashlib
import json

import numpy as np

from .firmware import Balance, activated, core, healthy
from .parameters import ROOT, load_parameters
from .profiles import native
from .sensing import JointEncoderFeedback, SensorMeasurements
from .protective_fall import SETTING_KEYS

D4, D6, D3 = c.c_double * 4, c.c_double * 6, c.c_double * 3
P = c.POINTER(c.c_double)


class SensorConfig(c.Structure):
    _fields_ = [
        (x, c.c_double)
        for x in [
            "dt",
            "velocity_tau",
            "joint_tau",
            "posture_tau",
            "correction_tau",
            "accel_gate",
            "prediction_delay",
            "local_tau",
            "gravity",
            "wheel_radius",
            "fusion_tau",
            "innovation_limit",
            "bias_tau",
        ]
    ]


class Estimate(c.Structure):
    _fields_ = [
        (x, c.c_double)
        for x in [
            "pitch",
            "pitch_rate",
            "roll",
            "yaw",
            "yaw_rate",
            "x",
            "y",
            "velocity",
        ]
    ] + [("q", D4), ("qd", D4), ("wheel_speed", c.c_double * 2), ("accel_norm", c.c_double)]


class FallSettings(c.Structure):
    _fields_ = [(key, c.c_double) for key in SETTING_KEYS]


class FallState(c.Structure):
    _fields_ = [(key, c.c_double) for key in ["exposure_s", "freefall_s", "age_s"]] + [
        ("qref", D4), ("qdref", D4), ("wheel", c.c_double * 2), ("reason", c.c_uint),
        ("latched", c.c_bool), ("torque_off", c.c_bool),
    ]


class EstimatorState(c.Structure):
    _fields_ = [(x, c.c_double) for x in ["pitch", "roll", "yaw", "x", "y", "velocity"]] + [
        ("previous", D6),
        ("posture", D4),
        ("rates", D6),
        ("initialized", c.c_bool),
        ("imu_velocity", c.c_double),
        ("accel_bias", c.c_double),
        ("imu_offset", c.c_double),
        ("fusion_ready", c.c_bool),
        ("imu_acceleration", c.c_double),
        ("odometry", c.c_double),
        ("compensation_limit", c.c_double),
    ]


class FallenState(c.Structure):
    """bipe_fallen_estimator (firmware/include/fallen.h, M16)."""

    _fields_ = [("estimator", EstimatorState), ("previous", D6), ("rates", D6),
                ("accepted", c.c_uint32), ("refused", c.c_uint32), ("quiet", c.c_uint32),
                ("started", c.c_bool)]  # fmt: skip


class LocalState(c.Structure):
    _fields_ = [("previous", D4), ("rates", D4), ("initialized", c.c_bool)]


def control_core(path=None):
    lib = core(path)
    signatures = {
        "estimator_init": [c.POINTER(EstimatorState), P, P],
        "estimator_step": [
            c.POINTER(EstimatorState),
            c.POINTER(SensorConfig),
            P,
            P,
            P,
            c.POINTER(Estimate),
        ],
        "local_step": [c.POINTER(LocalState), P, c.c_double, c.c_double, P],
        "map_joints": [P, P, P, P],
        "joint_limits": [P, c.c_double],
        "kinematics": [P, P, P, P, P],
        "carrier_rates": [P, P, P],
        "feedforward": [P, c.c_double, c.c_double, P],
        "legs": [P, P, P, P, P, P],
        "fall_step": [c.POINTER(FallState), c.POINTER(FallSettings), c.POINTER(Estimate),
                      c.c_bool, c.c_double, P, P],
        "fall_legs": [c.POINTER(FallState), c.POINTER(FallSettings), P, P, P],
        "control_step": [
            c.POINTER(Balance),
            c.POINTER(Estimate),
            c.c_double,
            c.c_double,
            P,
            P,
            c.c_double,
            P,
            P,
            P,
        ],
    }
    for name, signature in signatures.items():
        func = getattr(lib, "bipe_" + name)
        func.argtypes, func.restype = signature, c.c_bool
    from .firmware import Supervisor
    lib.bipe_fallen_step.argtypes = [c.POINTER(FallenState), c.POINTER(SensorConfig), P, P, P,
                                     c.POINTER(Estimate), c.POINTER(c.c_bool)]  # fmt: skip
    lib.bipe_fallen_step.restype = c.c_bool
    lib.bipe_begin_fall.argtypes = [c.POINTER(Supervisor), c.c_uint32, c.c_uint]
    lib.bipe_begin_fall.restype = c.c_bool
    lib.bipe_end_fall.argtypes = [c.POINTER(Supervisor)]
    lib.bipe_end_fall.restype = None
    return lib


def checked(result):
    if not result:
        raise ValueError("C sensing/control rejected invalid inputs or joint limits")


def observation(s):
    return Estimate(
        s["pitch"],
        s["pitch_rate"],
        s["roll"],
        s["yaw"],
        s["yaw_rate"],
        *s["position"],
        s["velocity"],
        D4(*s["q"]),
        D4(*s["qd"]),
        (c.c_double * 2)(*s["wheel_speed"]),
        s.get("accel_norm", 9.81),
    )


def estimate_dict(e):
    return {
        "pitch": e.pitch,
        "pitch_rate": e.pitch_rate,
        "roll": e.roll,
        "yaw": e.yaw,
        "yaw_rate": e.yaw_rate,
        "position": np.array([e.x, e.y]),
        "velocity": e.velocity,
        "q": np.array(e.q),
        "qd": np.array(e.qd),
        "wheel_speed": np.array(e.wheel_speed),
        "accel_norm": e.accel_norm,
    }


class FirmwareEstimator(SensorMeasurements):
    def __init__(self, model, data, p, config, seed, library=None):
        super().__init__(model, data, p, config, seed)
        self.lib = control_core(library)
        self.state = EstimatorState()
        self.settings = SensorConfig(
            self.dt,
            config["velocity_filter_tau_s"],
            config["joint_velocity_filter_tau_s"],
            config.get("posture_filter_tau_s", 0),
            config["attitude_correction_tau_s"],
            config["accel_norm_gate_m_s2"],
            config["transport_delay_s"] + config["gyro_anti_alias_tau_s"],
            config["local_joint_velocity_filter_tau_s"],
            p["sizing"]["gravity"],
            p["geometry"]["wheel_radius"],
            config.get("imu_velocity_fusion_tau_s", 0),
            config.get("imu_velocity_innovation_limit_m_s", 0),
            config.get("imu_accel_bias_tau_s", 0),
        )
        checked(
            self.lib.bipe_estimator_init(
                c.byref(self.state), D3(*self.calibration), D6(*self.encoders(data))
            )
        )

    @property
    def compensation_limit(self):
        return self.state.compensation_limit

    @compensation_limit.setter
    def compensation_limit(self, value):
        self.state.compensation_limit = value

    def update(self, data):
        gyro, accel, enc = self.next_packet(data)
        result = Estimate()
        checked(
            self.lib.bipe_estimator_step(
                c.byref(self.state),
                c.byref(self.settings),
                D3(*gyro),
                D3(*accel),
                D6(*enc),
                c.byref(result),
            )
        )
        return estimate_dict(result)


class FirmwareLocalFeedback(JointEncoderFeedback):
    def __init__(self, data, config, library=None):
        super().__init__(data, config)
        self.lib, self.state = control_core(library), LocalState()
        rates = D4()
        checked(
            self.lib.bipe_local_step(
                c.byref(self.state), D4(*self.previous), 0.001, self.tau, rates
            )
        )

    def update(self, data, dt):
        q, rates = self.read(data), D4()
        checked(self.lib.bipe_local_step(c.byref(self.state), D4(*q), dt, self.tau, rates))
        return q, np.array(rates)


class FirmwareController:
    def __init__(self, p, gains, library=None):
        # Constants are generated from these files, never silently use different host gains.
        if library is None and native(p):
            from .robustness_control import build_candidate

            library = build_candidate(p, gains, ROOT / f".local-tools/{p['profile']}-firmware")
        if library is None:
            configuration = {
                "nominal": load_parameters(),
                "gains": json.loads((ROOT / "config/controller_moving.json").read_text()),
            }
        else:
            from pathlib import Path

            configuration = json.loads(Path(library).with_suffix(".json").read_text())
            if (
                configuration["library_sha256"]
                != hashlib.sha256(Path(library).read_bytes()).hexdigest()
            ):
                raise ValueError("Candidate C library does not match its configuration")
        if p != configuration["nominal"] or gains != configuration["gains"]:
            raise ValueError("Rebuild the C core from the requested nominal configuration")
        self.lib, self.state = control_core(library), Balance()
        self.gains = gains
        self.feedforward = np.zeros(4)
        self.supervisor, self.safety_settings, self.start_ms = activated(self.lib)
        self.last_ms = self.start_ms
        settings = p.get("physics", {}).get("protective_fall")
        self.fall_settings = FallSettings(*(settings[key] for key in SETTING_KEYS)) if settings else None
        self.protection = FallState()
        self.previous_request = np.zeros(6)
        self.fall_announced = False


    @property
    def compensation_limit(self):
        """The estimator's acceleration compensation limit: the drive limit while driving."""
        moving = self.state.previous_velocity != 0 or self.state.braking
        return self.gains["max_command_acceleration"] if moving else 0.0
    def gate(self, s, elapsed_s):
        # Test-only stationary commissioning prelude and healthy electrical branch.
        # Dynamic IMU/encoder fields come from the estimator, never the physics pose.
        now = self.start_ms + 1 + round(elapsed_s * 1000)
        if self.protection.latched and not self.fall_announced:
            maximum = round((self.fall_settings.crouch_s + self.fall_settings.ramp_s) * 1000)
            checked(self.lib.bipe_begin_fall(c.byref(self.supervisor), now, maximum))
            self.fall_announced = True
        if self.protection.torque_off:
            self.lib.bipe_end_fall(c.byref(self.supervisor))
        data = healthy(now)
        body_time = self.start_ms + 1 + round(elapsed_s / 0.005) * 5
        # A cached 200 Hz sample retains its acquisition timestamp between body ticks.
        body_time = min(body_time, now)
        data.imu_time_ms = data.imu_sequence = body_time
        data.attitude_error = s["pitch"]
        data.gyro_norm = abs(s["pitch_rate"])
        for j, motor in enumerate(data.motor):
            motor.time_ms = motor.sequence = body_time
            motor.position = s["q"][j] if j < 4 else 0
            motor.speed = s["qd"][j] if j < 4 else s["wheel_speed"][j - 4]
        self.lib.bipe_tick(
            c.byref(self.supervisor), c.byref(self.safety_settings), c.byref(data), now, True
        )
        if not self.supervisor.torque_enabled and self.supervisor.state != 8:
            raise ValueError(f"C supervisor inhibited: fault {self.supervisor.fault}")
        self.last_ms = now

    def command(self, s, velocity, yaw_rate, qref, qdref, dt):
        if self.fall_settings is not None:
            protective = self.protect(s, dt)
            if self.protection.latched:
                self.feedforward[:] = 0
                return protective, s["pitch"], self.state.vref, self.state.heading
        torque, ff, reference = D6(), D4(), c.c_double()
        e = observation(s)
        checked(
            self.lib.bipe_control_step(
                c.byref(self.state),
                c.byref(e),
                velocity,
                yaw_rate,
                D4(*qref),
                D4(*qdref),
                dt,
                torque,
                ff,
                c.byref(reference),
            )
        )
        self.feedforward = np.array(ff)
        self.previous_request = np.array(torque)
        return np.array(torque), reference.value, self.state.vref, self.state.heading

    def protect(self, s, dt, valid=True):
        if self.fall_settings is None:
            raise ValueError("Protective fall settings are not configured")
        torque = D6()
        checked(self.lib.bipe_fall_step(c.byref(self.protection), c.byref(self.fall_settings),
                                      c.byref(observation(s)), valid, dt,
                                      D6(*self.previous_request), torque))
        if self.protection.latched:
            self.previous_request = np.array(torque)
        return np.array(torque)

    def leg_torques(self, s, qref, qdref, feedforward=None):
        ff, torque = D4(), D4()
        if self.protection.latched:
            checked(self.lib.bipe_fall_legs(c.byref(self.protection), c.byref(self.fall_settings),
                                          D4(*s["q"]), D4(*s["qd"]), torque))
            return np.array(torque)
        if feedforward is None:
            checked(self.lib.bipe_feedforward(D4(*s["q"]), s["pitch"], s["roll"], ff))
        else:
            ff = D4(*feedforward)
        checked(self.lib.bipe_legs(D4(*s["q"]), D4(*s["qd"]), D4(*qref), D4(*qdref), ff, torque))
        return np.array(torque)


def build_evidence(path=None):
    """Bind evidence to the actual compiled candidate and generated header."""
    from pathlib import Path

    library = Path(path).resolve() if path is not None else ROOT / "firmware/build/libbipe.so"
    header = (
        library.parent / "generated_config.h"
        if path is not None
        else ROOT / "firmware/include/generated_config.h"
    )
    paths = [library, header]
    paths += sorted((ROOT / "firmware/src").glob("*.c"))
    paths += [ROOT / "firmware/include/bipe.h"]
    return {
        str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p): hashlib.sha256(
            p.read_bytes()
        ).hexdigest()
        for p in paths
    }
