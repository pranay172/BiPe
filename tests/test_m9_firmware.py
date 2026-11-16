"""Portable M9 core parity through valid asymmetric five-bar output poses."""

import ctypes as c
import hashlib
import json
import math

import numpy as np
import pytest

from bipe.articulated import build, reset
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
from bipe.kinematics import equilibrium as profile_equilibrium
from bipe.kinematics import posture as profile_posture
from bipe.locomotion import LocomotionController
from bipe.m9_kinematics import (
    com_and_axle,
    leg_ik,
    posture,
    support_feedforward,
    wheel_carrier_angle_rate,
)
from bipe.parameters import ROOT, load_parameters
from bipe.robustness_control import build_candidate
from bipe.sensing import SensorEstimator


@pytest.fixture
def m9_core(tmp_path):
    p = load_parameters(profile="m9")
    # These component tests deliberately exercise saturated/unsafe attitudes;
    # the integrated fall policy has its own equivalence and shutdown tests.
    p["physics"].pop("protective_fall", None)
    gains = json.loads((ROOT / "config/controller_m9.json").read_text())
    library = build_candidate(p, gains, tmp_path)
    return p, gains, library


def test_fivebar_C_mass_feedforward_carrier_and_saturated_control(m9_core):
    p, gains, library = m9_core
    lib = control_core(library)
    py, fw = LocomotionController(p, gains), FirmwareController(p, gains, library)
    rng = np.random.default_rng(20261004)
    caps = np.r_[
        np.full(4, p["articulation"]["peak_torque_per_joint"]),
        np.full(2, p["motor"]["peak_torque_per_wheel"]),
    ]
    for j in range(500):
        q = np.r_[
            leg_ik(rng.uniform(-0.01, 0.01), -rng.uniform(0.045, 0.118), p),
            leg_ik(rng.uniform(-0.01, 0.01), -rng.uniform(0.045, 0.118), p),
        ]
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


def test_fivebar_C_estimator_uses_identical_delayed_output_packets(m9_core):
    p, _, library = m9_core
    sensors = json.loads((ROOT / "config/sensors-m9.json").read_text())
    model, data = build(p)
    reset(model, data, p)
    py = SensorEstimator(model, data, p, sensors, 12)
    fw = FirmwareEstimator(model, data, p, sensors, 12, library)
    rng = np.random.default_rng(13)
    for j in range(1000):
        t = j * 0.005
        q = np.r_[
            leg_ik(0.003 * math.sin(t), -0.1 - 0.012 * math.sin(t * 0.8), p),
            leg_ik(-0.003 * math.sin(t), -0.1 - 0.009 * math.sin(t * 0.9), p),
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


def test_fivebar_build_evidence_hashes_actual_candidate(m9_core):
    _, _, library = m9_core
    evidence = build_evidence(library)
    assert evidence[str(library)] == hashlib.sha256(library.read_bytes()).hexdigest()
    header = library.parent / "generated_config.h"
    assert evidence[str(header)] == hashlib.sha256(header.read_bytes()).hexdigest()
    assert "firmware/build/libbipe.so" not in evidence
    assert "firmware/include/generated_config.h" not in evidence


def test_fivebar_portable_posture_mapping_and_workspace(m9_core):
    p, _, library = m9_core
    lib = control_core(library)
    lib.bipe_posture.argtypes = [c.c_double, c.POINTER(c.c_double)]
    lib.bipe_posture.restype = c.c_bool
    q = D4()
    for angle in np.r_[np.linspace(30, 65, 201), [30, 40, 60, 65]]:
        assert lib.bipe_posture(float(angle), q)
        assert np.array(q) == pytest.approx(posture(float(angle), p), abs=1e-12)
    for angle in [29, 66, math.nan, math.inf]:
        assert not lib.bipe_posture(angle, q)


@pytest.mark.parametrize("profile", ["m4", "m9"])
@pytest.mark.parametrize("distinct_hold", [False, True])
def test_command_mode_transition_reference_integral_and_saturation(
    tmp_path, profile, distinct_hold
):
    p = load_parameters(profile=profile)
    p.get("physics", {}).pop("protective_fall", None)
    name = "controller_m9.json" if profile == "m9" else "controller_moving.json"
    gains = json.loads((ROOT / "config" / name).read_text())
    gains.pop("hold_speed_kp", None)
    gains.pop("hold_speed_ki", None)
    if profile == "m9":
        gains.update(pitch_kp=1.3, pitch_kd=0.1, speed_kp=0.3, speed_ki=0.2)
    if distinct_hold:
        gains.update(hold_speed_kp=0.4, hold_speed_ki=0.1)
    library = build_candidate(p, gains, tmp_path)
    py, fw = LocomotionController(p, gains), FirmwareController(p, gains, library)
    py.integral = fw.state.integral = 0.15
    q = profile_posture(40, p)
    caps = np.r_[
        np.full(4, p["articulation"]["peak_torque_per_joint"]),
        np.full(2, p["motor"]["peak_torque_per_wheel"]),
    ]
    saturated = 0
    for j in range(400):
        velocity = [0, 0.3, 0, -0.3, 0][j // 80]
        state = {
            "q": q,
            "qd": np.zeros(4),
            "position": np.array([0.02 * math.sin(j * 0.03), 0]),
            "pitch": 1.9 if j % 80 >= 70 else 0.03 * math.sin(j * 0.07),
            "roll": 0.0,
            "pitch_rate": 0.2 * math.cos(j * 0.07),
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "velocity": 0.2 * math.sin(j * 0.05),
            "wheel_speed": np.full(2, 4 * math.sin(j * 0.05)),
        }
        previous_integral = py.integral
        expected, reference, _, _ = py.command(state, velocity, 0, q, np.zeros(4), 0.005)
        actual, actual_reference, _, _ = fw.command(
            state, velocity, 0, q, np.zeros(4), 0.005
        )
        assert actual == pytest.approx(np.clip(expected, -caps, caps), abs=1e-10)
        assert actual_reference == pytest.approx(reference, abs=1e-12)
        assert fw.state.integral == pytest.approx(py.integral, abs=1e-12)
        assert abs(py.integral) <= gains["speed_integral_limit"]
        # Switching gains retains the bounded learned integral; no reset.
        assert abs(py.integral - previous_integral) <= (
            abs(py.vref - state["velocity"]) * 0.005 + 1e-12
        )
        kp, ki = gains["speed_kp"], gains["speed_ki"]
        if profile == "m9" and velocity == 0:
            kp, ki = gains.get("hold_speed_kp", kp), gains.get("hold_speed_ki", ki)
        assert reference == pytest.approx(
            profile_equilibrium(p, q)
            + np.clip(
                kp * (py.vref - state["velocity"]) + ki * py.integral,
                -gains["max_pitch_offset"],
                gains["max_pitch_offset"],
            ),
            abs=1e-12,
        )
        saturated += bool(np.isclose(abs(actual[4:]), caps[4:]).any())
    assert saturated > 0


def test_mode_gains_full_configured_sensor_control_stream(tmp_path):
    p = load_parameters(profile="m9")
    gains = json.loads((ROOT / "config/controller_m9.json").read_text())
    gains.update(
        pitch_kp=1.3,
        pitch_kd=0.1,
        speed_kp=0.3,
        speed_ki=0.2,
        hold_speed_kp=0.4,
        hold_speed_ki=0.1,
        max_command_acceleration=3,
    )
    library = build_candidate(p, gains, tmp_path)
    sensors = json.loads((ROOT / "config/sensors-m9.json").read_text())
    model, data = build(p)
    reset(model, data, p)
    py_est = SensorEstimator(model, data, p, sensors, 12)
    fw_est = FirmwareEstimator(model, data, p, sensors, 12, library)
    py, fw = LocomotionController(p, gains), FirmwareController(p, gains, library)
    caps = np.r_[
        np.full(4, p["articulation"]["peak_torque_per_joint"]),
        np.full(2, p["motor"]["peak_torque_per_wheel"]),
    ]
    # Synthetic device channels pass through the actual bandwidth, noise,
    # quantization, delay and estimator states. This is a core parity test.
    for j in range(1000):
        t = j * 0.005
        q = np.r_[
            leg_ik(0.003 * math.sin(t), -0.1 - 0.008 * math.sin(t), p),
            leg_ik(-0.002 * math.sin(t), -0.1 - 0.006 * math.sin(t), p),
        ]
        data.sensor("gyro").data[:] = [0.01 * math.sin(t), 0.03 * math.cos(t), 0.01]
        data.sensor("accelerometer").data[:] = [-0.1 * math.sin(t), 0.02, 9.81]
        for name, value in zip(
            py_est.encoder_names, np.r_[q, 3 * math.sin(t), 3 * math.sin(t)], strict=True
        ):
            data.sensor(name).data[0] = value
        for _ in range(5):
            py_est.preprocess(data, 0.001)
            fw_est.preprocess(data, 0.001)
        expected_state, actual_state = py_est.update(data), fw_est.update(data)
        for key in expected_state:
            assert actual_state[key] == pytest.approx(expected_state[key], abs=1e-10)
        velocity = [0, 0.3, 0, -0.3, 0][j // 200]
        qr = posture(40 + 5 * math.sin(t), p)
        expected, reference, _, _ = py.command(
            expected_state, velocity, 0, qr, np.zeros(4), 0.005
        )
        actual, actual_reference, _, _ = fw.command(
            actual_state, velocity, 0, qr, np.zeros(4), 0.005
        )
        assert actual == pytest.approx(np.clip(expected, -caps, caps), abs=1e-10)
        assert actual_reference == pytest.approx(reference, abs=1e-10)
        assert fw.state.integral == pytest.approx(py.integral, abs=1e-10)

@pytest.mark.parametrize('trigger', ['pitch', 'roll', 'freefall', 'invalid'])
def test_protective_crouch_C_python_parity_and_latched_finite_shutdown(tmp_path, trigger):
    from bipe.protective_fall import ProtectiveFall

    p = load_parameters(profile='m9')
    gains = json.loads((ROOT / 'config/controller_m9.json').read_text())
    library = build_candidate(p, gains, tmp_path)
    fw = FirmwareController(p, gains, library)
    py = ProtectiveFall(p, gains)
    settings = p['physics']['protective_fall']
    q = posture(40, p)
    e = dict(q=q.copy(), qd=np.zeros(4), pitch=profile_equilibrium(p, q),
             pitch_rate=0., roll=0., yaw=0., yaw_rate=0., position=np.zeros(2),
             velocity=0., wheel_speed=np.zeros(2), accel_norm=9.81)
    previous = np.array([0., 0., 0., 0., .12, -.12])
    fw.previous_request = previous.copy()
    detected = None
    off = None
    last_ref = q.copy()
    for j in range(100):
        valid = True
        if j >= 4:
            if trigger == 'pitch':
                e['pitch'] = profile_equilibrium(p, q) + math.radians(19.5)
                e['pitch_rate'] = math.radians(40)
            elif trigger == 'roll':
                e['roll'] = math.radians(14)
            elif trigger == 'freefall':
                e['accel_norm'] = .5
            else:
                valid = False
        expected = py.step(e, previous, .005, valid)
        fw.previous_request = previous.copy()
        actual = fw.protect(e, .005, valid)
        assert actual == pytest.approx(expected, abs=1e-12)
        assert fw.protection.latched == py.latched
        assert fw.protection.torque_off == py.torque_off
        assert fw.protection.reason == py.reason
        assert np.array(fw.protection.qref) == pytest.approx(py.qref, abs=1e-12)
        if py.latched:
            if detected is None:
                detected = j
                last_ref = q.copy()
            if not py.torque_off:
                assert np.max(abs(py.qref - last_ref)) <= settings['reference_rate_rad_s'] * .005 + 1e-12
                last_ref = py.qref.copy()
            assert np.max(abs(actual[:4])) <= settings['joint_cap_Nm']
            assert np.max(abs(actual[4:])) <= .12
        # The local supervisor runs at 1 kHz, even when the body policy is 200 Hz.
        for local_tick in range(5):
            fw.gate(e, j * .005 + local_tick * .001)
        if py.torque_off:
            off = off if off is not None else j
            assert np.array_equal(actual, np.zeros(6))
            assert fw.supervisor.state == 8
            e['pitch'], e['pitch_rate'], e['roll'], e['accel_norm'] = profile_equilibrium(p,q), 0., 0., 9.81
        previous = expected.copy()
    assert detected is not None and off is not None
    assert (off - detected) * .005 <= settings['crouch_s'] + settings['ramp_s'] + 1e-12
    assert not fw.supervisor.torque_enabled
    assert not fw.supervisor.power_request


def test_protective_supervisor_keeps_electrical_fault_and_deadline_checks(tmp_path):
    from bipe.firmware import healthy

    p = load_parameters(profile='m9')
    gains = json.loads((ROOT / 'config/controller_m9.json').read_text())
    library = build_candidate(p, gains, tmp_path)
    fw = FirmwareController(p, gains, library)
    now = fw.start_ms + 1
    assert fw.lib.bipe_begin_fall(c.byref(fw.supervisor), now, 280)
    sample = healthy(now)
    sample.attitude_error = math.radians(70)
    fw.lib.bipe_tick(c.byref(fw.supervisor), c.byref(fw.safety_settings), c.byref(sample), now, True)
    assert fw.supervisor.state == 7 and fw.supervisor.torque_enabled
    sample.bus_off = True
    fw.lib.bipe_tick(c.byref(fw.supervisor), c.byref(fw.safety_settings), c.byref(sample), now+1, True)
    assert fw.supervisor.state == 6 and not fw.supervisor.torque_enabled
    # Missing a local deadline remains a fault in the protective interval.
    fw = FirmwareController(p, gains, library)
    now = fw.start_ms + 1
    assert fw.lib.bipe_begin_fall(c.byref(fw.supervisor), now, 280)
    sample = healthy(now)
    fw.lib.bipe_tick(c.byref(fw.supervisor), c.byref(fw.safety_settings), c.byref(sample), now, True)
    late = now + fw.safety_settings.deadline_ms + 1
    sample = healthy(late)
    fw.lib.bipe_tick(c.byref(fw.supervisor), c.byref(fw.safety_settings), c.byref(sample),
                    late, True)
    assert fw.supervisor.state == 6 and not fw.supervisor.torque_enabled
