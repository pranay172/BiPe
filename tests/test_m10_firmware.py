"""Portable M10 core parity through valid asymmetric serial-leg poses."""

import ctypes as c
import hashlib
import json
import math

import numpy as np
import pytest

from bipe.firmware_control import (
    D3,
    D4,
    D6,
    Estimate,
    FirmwareController,
    FirmwareEstimator,
    build_evidence,
    control_core,
    estimate_dict,
)
from bipe.locomotion import LocomotionController
from bipe.m10.kinematics import (
    com_and_axle,
    leg_ik,
    posture,
    posture_target,
    support_feedforward,
    wheel_carrier_angle_rate,
)
from bipe.m10.plant import build, reset
from bipe.parameters import ROOT, load_parameters
from bipe.robustness_control import build_candidate
from bipe.sensing import SensorEstimator


@pytest.fixture(scope="module")
def m10_core(tmp_path_factory):
    p = load_parameters(profile="m10")
    p["physics"].pop("protective_fall", None)
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    library = build_candidate(p, gains, tmp_path_factory.mktemp("m10-build"))
    return p, gains, library


def random_pose(rng, p):
    """Asymmetric poses near the posture range, inside the joint end stops."""
    sides = []
    for _ in range(2):
        depth, x = posture_target(rng.uniform(30, 65), p)
        sides.append(leg_ik(x + rng.uniform(-0.004, 0.004), -depth + rng.uniform(-0.004, 0.004), p))
    return np.concatenate(sides)


def test_serial_C_mass_feedforward_carrier_and_saturated_control(m10_core):
    p, gains, library = m10_core
    assert gains["slope_observer_tau_s"] > 0, "parity must cover the slope observer"
    lib = control_core(library)
    py, fw = LocomotionController(p, gains), FirmwareController(p, gains, library)
    rng = np.random.default_rng(20261012)
    caps = np.r_[
        np.full(4, p["articulation"]["peak_torque_per_joint"]),
        np.full(2, p["motor"]["peak_torque_per_wheel"]),
    ]
    for j in range(500):
        q = random_pose(rng, p)
        qd = rng.normal(0, 0.2, 4)
        com, axle = (c.c_double * 2)(), (c.c_double * 2)()
        eq, scale = c.c_double(), c.c_double()
        assert lib.bipe_kinematics(D4(*q), com, axle, c.byref(eq), c.byref(scale))
        expected_com, expected_axle = com_and_axle(p, q)
        assert np.array(com) == pytest.approx(expected_com, abs=1e-12)
        assert np.array(axle) == pytest.approx(expected_axle, abs=1e-12)
        carrier = (c.c_double * 2)()
        assert lib.bipe_carrier_rates(D4(*q), D4(*qd), carrier)
        expected_rates = [
            wheel_carrier_angle_rate(q[2 * i : 2 * i + 2], qd[2 * i : 2 * i + 2], p)[1]
            for i in range(2)
        ]
        assert np.array(carrier) == pytest.approx(expected_rates, abs=1e-12)
        pitch, roll = rng.uniform(-0.3, 0.3, 2)
        ff = D4()
        assert lib.bipe_feedforward(D4(*q), pitch, roll, ff)
        assert np.array(ff) == pytest.approx(support_feedforward(p, q, pitch, roll), abs=1e-12)
        state = {
            "q": q,
            "qd": qd,
            "position": rng.normal(0, 0.1, 2),
            "pitch": pitch,
            "roll": roll,
            "pitch_rate": rng.uniform(-3, 3),
            "yaw": rng.uniform(-0.3, 0.3),
            "yaw_rate": rng.uniform(-1, 1),
            "velocity": rng.uniform(-0.4, 0.4),
            "wheel_speed": rng.uniform(-10, 10, 2),
        }
        qr, qdr = posture(rng.uniform(30, 65), p), rng.normal(0, 0.2, 4)
        velocity, yaw = [0, 0.3, -0.3][j // 30 % 3], [0, 0.4, -0.4][j // 45 % 3]
        expected, ref, _, _ = py.command(state, velocity, yaw, qr, qdr, 0.005)
        actual, actual_ref, _, _ = fw.command(state, velocity, yaw, qr, qdr, 0.005)
        assert actual == pytest.approx(np.clip(expected, -caps, caps), abs=1e-10)
        assert actual_ref == pytest.approx(ref, abs=1e-10)
        assert fw.state.integral == pytest.approx(py.integral, abs=1e-12)
        assert fw.state.slope_force == pytest.approx(py.slope_force, abs=1e-9)


def test_serial_C_estimator_uses_identical_delayed_output_packets(m10_core):
    p, _, library = m10_core
    sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
    assert sensors["imu_velocity_fusion_tau_s"] > 0, "parity must cover the fused velocity"
    model, data = build(p)
    reset(model, data, p)
    py = SensorEstimator(model, data, p, sensors, 12)
    fw = FirmwareEstimator(model, data, p, sensors, 12, library)
    rng = np.random.default_rng(14)
    for j in range(1000):
        t = j * 0.005
        q = np.r_[
            leg_ik(-0.012 + 0.003 * math.sin(t), -0.067 - 0.02 * math.sin(t * 0.8), p),
            leg_ik(-0.012 - 0.003 * math.sin(t), -0.067 - 0.015 * math.sin(t * 0.9), p),
        ]
        gyro = rng.normal(0, 0.05, 3)
        accel = np.array([-0.15 * math.sin(t), 0.1 * math.cos(t), 9.81])
        accel += rng.normal(0, 0.06, 3)
        encoder = np.r_[q, 4 * math.sin(t * 0.2), 5 * math.sin(t * 0.2)]
        packet = gyro, accel, encoder
        py.next_packet = lambda _, packet=packet: packet
        expected = py.update(data)
        out = Estimate()
        assert fw.lib.bipe_estimator_step(
            c.byref(fw.state),
            c.byref(fw.settings),
            D3(*gyro),
            D3(*accel),
            D6(*encoder),
            c.byref(out),
        )
        actual = estimate_dict(out)
        for key in expected:
            assert actual[key] == pytest.approx(expected[key], abs=1e-10)


def test_serial_build_evidence_hashes_actual_candidate(m10_core):
    _, _, library = m10_core
    evidence = build_evidence(library)
    assert evidence[str(library)] == hashlib.sha256(library.read_bytes()).hexdigest()


def test_serial_portable_posture_mapping_and_workspace(m10_core):
    p, _, library = m10_core
    lib = control_core(library)
    lib.bipe_posture.argtypes = [c.c_double, c.POINTER(c.c_double)]
    lib.bipe_posture.restype = c.c_bool
    q = D4()
    for angle in np.r_[np.linspace(30, 65, 201), [30, 40, 60, 65]]:
        assert lib.bipe_posture(float(angle), q)
        assert np.array(q) == pytest.approx(posture(float(angle), p), abs=1e-12)
    for angle in [29, 66, math.nan, math.inf]:
        assert not lib.bipe_posture(angle, q)
