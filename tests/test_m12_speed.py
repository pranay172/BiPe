import copy
import json
import math

import numpy as np
import pytest

from bipe.firmware_control import FirmwareController, FirmwareEstimator
from bipe.locomotion import LocomotionController, shape_reference
from bipe.m10.kinematics import posture
from bipe.m10.plant import build, reset
from bipe.m12 import speed as S
from bipe.parameters import ROOT, load_parameters
from bipe.robustness_control import build_candidate, equivalence
from bipe.sensing import SensorEstimator

P, GAINS, SENSORS, CFG = S.load()
ENVELOPE = json.loads((ROOT / "config/robot.json").read_text())["m10"]["targets"]["drive_envelope"]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    p = copy.deepcopy(P)
    p["physics"].pop("protective_fall", None)
    return p, build_candidate(p, GAINS, tmp_path_factory.mktemp("m12-build"))


def test_shaped_reference_reaches_targets_without_overshoot_at_bounded_jerk():
    limit, jerk, dt = GAINS["max_command_acceleration"], GAINS["command_jerk_m_s3"], 0.005
    rng = np.random.default_rng(3)
    for target in [1.2, -0.3, 0.3, 0.01, *rng.uniform(-1.2, 1.2, 20)]:
        v = a = 0.0
        start = 0.0
        history = []
        for _ in range(2000):
            previous = a
            v, a = shape_reference(v, a, target, limit, jerk, dt)
            assert abs(a) <= limit + 1e-12
            assert abs(a - previous) <= jerk * dt + 1e-12
            history.append(v)
        history = np.array(history)
        assert history[-1] == target
        # Monotonic approach from rest, no overshoot past the target: both within the
        # discretisation, jerk dt^2, before the final snap to the target.
        sign, tolerance = math.copysign(1, target - start), jerk * dt * dt
        assert np.all(np.diff(history) * sign >= -tolerance)
        assert np.all((history - target) * sign <= tolerance)


def test_shaped_reference_follows_a_moving_target_continuously():
    limit, jerk, dt = GAINS["max_command_acceleration"], GAINS["command_jerk_m_s3"], 0.005
    v = a = 0.0
    for i in range(4000):
        target = 1.0 * math.sin(i * dt * 2.0) + (0.5 if i % 700 < 350 else -0.5)
        previous = a
        v, a = shape_reference(v, a, target, limit, jerk, dt)
        assert abs(a - previous) <= jerk * dt + 1e-12 and abs(a) <= limit + 1e-12


def test_stop_brakes_the_reference_to_rest_and_holds_there():
    controller = LocomotionController(P, GAINS)
    controller.protection = None
    q = posture(40, P)
    s = {
        "q": q,
        "qd": np.zeros(4),
        "position": np.zeros(2),
        "pitch": 0.0,
        "roll": 0.0,
        "pitch_rate": 0.0,
        "yaw": 0.0,
        "yaw_rate": 0.0,
        "velocity": 0.0,
        "wheel_speed": np.zeros(2),
    }
    dt, latched, commanded = 0.005, None, None
    for i in range(1200):
        if i == 400:
            commanded = s["position"].copy()
        velocity = 1.2 if i < 400 else 0.0
        controller.command(s, velocity, 0.0, q, np.zeros(4), dt)
        # An ideal plant that follows the reference.
        s["velocity"] = controller.vref
        s["position"] = s["position"] + np.array([controller.vref * dt, 0.0])
        if i >= 400 and latched is None and not controller.braking:
            latched = s["position"].copy()
        if 400 <= i < 400 + 20:
            assert controller.braking
    assert latched is not None and not controller.braking
    # Held where the reference came to rest, not where the stop was commanded.
    np.testing.assert_allclose(controller.hold_position, latched, atol=1.2 * dt)
    # About the 0.58 m braking distance at 1.25 m/s^2 past the command point.
    assert controller.hold_position[0] - commanded[0] > 0.5
    assert controller.vref == 0 and controller.command_acceleration == 0


def test_hold_reference_keeps_the_plain_rate_limit_without_feedforward():
    controller = LocomotionController(P, GAINS)
    controller.protection = None
    q = posture(40, P)
    s = {
        "q": q,
        "qd": np.zeros(4),
        "position": np.zeros(2),
        "pitch": 0.0,
        "roll": 0.0,
        "pitch_rate": 0.0,
        "yaw": 0.0,
        "yaw_rate": 0.0,
        "velocity": 0.0,
        "wheel_speed": np.zeros(2),
    }
    dt = 0.005
    controller.command(s, 0.0, 0.0, q, np.zeros(4), dt)
    # Pushed 0.5 m off the hold position: the position loop asks for the hold speed limit.
    s["position"] = np.array([-0.5, 0.0])
    controller.command(s, 0.0, 0.0, q, np.zeros(4), dt)
    assert not controller.braking
    assert controller.vref == pytest.approx(GAINS["hold_command_acceleration"] * dt)
    assert controller.command_acceleration == 0 and controller.acceleration_lean == 0


def test_drive_envelope_sets_the_firmware_command_gate(library):
    p, lib = library
    assert P["sizing"]["drive_speed"] == ENVELOPE["speed_m_s"] == 1.2
    shared = json.loads((ROOT / "config/robot.json").read_text())
    assert shared["sizing"]["drive_speed"] == 0.3, "other profiles keep the shared value"
    assert load_parameters()["sizing"]["drive_speed"] == 0.3
    header = (lib.parent / "generated_config.h").read_text()
    assert "#define BIPE_DRIVE_SPEED 1.2\n" in header
    assert f"#define BIPE_COMMAND_JERK_M_S3 {GAINS['command_jerk_m_s3']:.1f}\n" in header
    fw = FirmwareController(p, GAINS, lib)
    fw.fall_settings = None
    q = posture(40, p)
    s = {
        "q": q,
        "qd": np.zeros(4),
        "position": np.zeros(2),
        "pitch": 0.05,
        "roll": 0.0,
        "pitch_rate": 0.0,
        "yaw": 0.0,
        "yaw_rate": 0.0,
        "velocity": 1.0,
        "wheel_speed": np.zeros(2),
    }
    fw.command(s, 1.2, 1.0, q, np.zeros(4), 0.005)
    with pytest.raises(ValueError, match="rejected"):
        fw.command(s, 1.21, 0.0, q, np.zeros(4), 0.005)
    with pytest.raises(ValueError, match="rejected"):
        fw.command(s, 0.0, 1.01, q, np.zeros(4), 0.005)


def test_C_and_python_controllers_agree_at_the_envelope_speed(library):
    p, lib = library
    check = equivalence(p, GAINS, lib, count=1500)
    assert check["passed"], check


def _accelerating_estimate(sensors, acceleration, duration, limit=1.25, wheel=None, library=None):
    """Estimator on synthetic packets: constant lean atan(a/g), constant forward acceleration.

    limit is the compensation limit the controller gives the estimator (its drive
    acceleration limit while driving, 0 while holding); wheel is the acceleration the wheel
    odometry shows (default: the true one).
    """
    p = copy.deepcopy(P)
    m, d = build(p)
    reset(m, d, p)
    est = (
        FirmwareEstimator(m, d, p, sensors, 3, library)
        if library
        else SensorEstimator(m, d, p, sensors, 3)
    )
    g = p["sizing"]["gravity"]
    lean = math.atan(acceleration / g)
    radius = p["geometry"]["wheel_radius"]
    start = np.array(est.state.previous) if library else est.encoder_previous
    q, wheels = start[:4].copy(), start[4:].copy()
    est.compensation_limit = limit
    wheel = acceleration if wheel is None else wheel
    if library:
        est.state.pitch = lean
    else:
        est.pitch = lean
    for j in range(round(duration / est.dt)):
        t = (j + 1) * est.dt
        # Specific force of the lean and the forward acceleration, in the body frame.
        accel = np.array(
            [
                acceleration * math.cos(lean) - g * math.sin(lean),
                0.0,
                acceleration * math.sin(lean) + g * math.cos(lean),
            ]
        )
        encoder = np.r_[q, wheels + 0.5 * wheel * t**2 / radius]
        packet = np.zeros(3), accel, encoder
        est.next_packet = lambda _, packet=packet: packet
        out = est.update(d)
    return lean, out


def test_attitude_holds_the_lean_through_a_sustained_acceleration():
    lean, out = _accelerating_estimate(SENSORS, 1.25, 3.0)
    assert SENSORS["imu_velocity_fusion_tau_s"] > 0
    assert math.degrees(abs(out["pitch"] - lean)) < 0.5
    # Without the fusion (and so without the compensation) the attitude drifts toward level.
    plain = {**SENSORS, "imu_velocity_fusion_tau_s": 0}
    lean, out = _accelerating_estimate(plain, 1.25, 3.0)
    assert math.degrees(lean - out["pitch"]) > 3.0


def test_compensation_applies_only_to_a_drive_acceleration():
    # The controller gives its drive acceleration limit while driving, 0 while holding.
    controller = LocomotionController(P, GAINS)
    assert controller.compensation_limit == 0
    controller.moving = True
    assert controller.compensation_limit == GAINS["max_command_acceleration"] == 1.25
    # Holding (a push at rest): nothing is compensated, as without the fusion.
    lean, held = _accelerating_estimate(SENSORS, 1.25, 3.0, limit=0.0)
    assert math.degrees(lean - held["pitch"]) > 3.0
    # Driving against a blocked wheel: the odometry shows no acceleration to remove.
    level, blocked = _accelerating_estimate(SENSORS, 0.0, 3.0, wheel=0.0)
    assert level == 0 and math.degrees(abs(blocked["pitch"])) < 0.2
    # Beyond the drive limit (as in a push) only the limit is removed.
    lean, full = _accelerating_estimate(SENSORS, 3.0, 3.0, limit=1e9)
    _, clipped = _accelerating_estimate(SENSORS, 3.0, 3.0)
    _, none = _accelerating_estimate(SENSORS, 3.0, 3.0, limit=0.0)
    assert abs(full["pitch"] - lean) < abs(clipped["pitch"] - lean) < abs(none["pitch"] - lean)


def test_C_and_python_estimators_agree_with_the_compensation_limit(library):
    _, lib = library
    for limit, wheel in [(1.25, None), (0.0, None), (1.25, 0.0), (0.5, None), (1.25, -2.0)]:
        _, py = _accelerating_estimate(SENSORS, 1.25, 1.0, limit, wheel)
        _, fw = _accelerating_estimate(SENSORS, 1.25, 1.0, limit, wheel, lib)
        for key in ("pitch", "roll", "velocity"):
            assert fw[key] == pytest.approx(py[key], abs=1e-9), (limit, wheel, key)


def test_speed_cases_follow_the_config():
    cases = S.definitions(CFG)
    assert {"drive_step", "drive_ramp", "spin", "weave", "replay_corner_90"} <= set(cases)
    for case in cases.values():
        if "knots" in case:
            k = np.array(case["knots"])
            assert np.all(np.diff(k[:, 0]) >= 0)
            assert np.all(np.abs(k[:, 1]) <= ENVELOPE["speed_m_s"] + 1e-12)
            assert np.all(np.abs(k[:, 2]) <= ENVELOPE["yaw_rate_rad_s"] + 1e-12)
            assert np.all(k[:, 1] >= -ENVELOPE["reverse_speed_m_s"] - 1e-12)
    assert np.allclose(S.knot_command(cases["drive_step"]["knots"], 9.0), 0.0)
    assert np.allclose(S.knot_command(cases["drive_ramp"]["knots"], 1.48), [0.6, 0.0])


def test_speed_record_is_current_and_passes():
    summary = json.loads((S.OUTPUT / "summary.json").read_text())
    assert summary["controller_sha256"] == S._sha(GAINS)
    assert summary["speed_config_sha256"] == S._sha(CFG)
    assert set(summary["cases"]) == set(S.definitions(CFG))
    # Every failure is a recorded shortfall, and every recorded shortfall still fails.
    assert summary["passed_except_recorded_shortfalls"], summary["unrecorded_failures"]
    assert summary["recorded_shortfalls"] == sorted(CFG["recorded_shortfalls"]["cases"])
    assert summary["convergence"]["passed"]
