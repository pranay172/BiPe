import json
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from bipe.articulated import JOINTS, build, reset, truth
from bipe.kinematics import com_and_axle, leg_fk, leg_ik, leg_jacobian, posture
from bipe.locomotion import TorqueMotors
from bipe.parameters import ROOT, load_parameters, total_mass
from bipe.sensing import JointEncoderFeedback, SensorEstimator


@pytest.mark.parametrize("angle", [25, 40, 60, 65])
def test_articulated_mass_and_independent_kinematics(angle):
    p = load_parameters()
    m, d = build(p)
    reset(m, d, p, angle)
    assert m.neq == 0 and m.nv == 12 and m.nu == 6
    assert m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert sum(m.body_mass) == pytest.approx(total_mass(p))
    q = posture(angle)
    com, axle = com_and_axle(p, q)
    origin = d.xpos[m.body("chassis").id]
    R = d.xmat[m.body("chassis").id].reshape(3, 3)
    physical_com = R.T @ (d.subtree_com[m.body("chassis").id] - origin)
    physical_axle = R.T @ (d.site_xpos[m.site("left_axle").id] - origin)
    assert physical_com[[0, 2]] == pytest.approx(com, abs=1e-10)
    assert physical_axle[[0, 2]] == pytest.approx(axle, abs=1e-10)
    knee = R.T @ (d.geom_xpos[m.geom("left_knee_motor").id] - origin)
    assert knee[0] < 0 and m.geom_pos[m.geom("left_eye").id, 0] > 0
    assert all(m.jnt_limited[m.joint(name).id] for name in JOINTS)


def test_leg_workspace_inverse_and_jacobian():
    length = 0.09
    for angle in [25, 40, 60, 65]:
        q = posture(angle)[:2]
        point = leg_fk(q, length)
        assert leg_ik(*point, length) == pytest.approx(q)
        eps = 1e-6
        numeric = np.column_stack(
            [
                (leg_fk(q + eps * np.eye(2)[i], length) - leg_fk(q - eps * np.eye(2)[i], length))
                / (2 * eps)
                for i in range(2)
            ]
        )
        assert numeric == pytest.approx(leg_jacobian(q, length), abs=1e-9)
    with pytest.raises(ValueError):
        leg_ik(0, -0.19, length)


def test_local_encoder_and_estimator_updates_use_sensor_channels_only():
    p = load_parameters()
    m, d = build(p)
    reset(m, d, p)
    c = json.loads((ROOT / "config/sensors.json").read_text())
    c.update(
        gyro_noise_std_rad_s=0,
        gyro_residual_bias_rad_s=[0, 0, 0],
        accel_noise_std_m_s2=0,
        accel_bias_m_s2=[0, 0, 0],
        transport_delay_s=0,
        joint_encoder_counts_per_rev=10**12,
        wheel_encoder_counts_per_rev=10**12,
    )
    estimator = SensorEstimator(m, d, p, c, 1)
    local = JointEncoderFeedback(d, c)
    assert estimator.pitch == pytest.approx(truth(m, d, p)["pitch"], abs=1e-10)
    measurements = {name: d.sensor(name).data.copy() for name in estimator.encoder_names}
    measurements["gyro"] = np.zeros(3)
    R = d.xmat[m.body("chassis").id].reshape(3, 3)
    measurements["accelerometer"] = R.T @ np.array([0, 0, 9.81])
    # A leg bends while its wheel counter-rotates, producing zero world rolling
    # angle. Wheel-only odometry would falsely report forward translation.
    for name, delta in zip(JOINTS, [0.01, -0.02, 0.01, -0.02], strict=True):
        measurements[name + "_encoder"] += delta
    for name in ["left_wheel_joint_encoder", "right_wheel_joint_encoder"]:
        measurements[name] += 0.01
    packet_only = SimpleNamespace(sensor=lambda name: SimpleNamespace(data=measurements[name]))
    estimator.preprocess(packet_only, 0.001)
    state = estimator.update(packet_only)
    q, qd = local.update(packet_only, 0.001)
    assert abs(state["velocity"]) < 1e-8
    assert np.isfinite(qd).all() and q.shape == (4,)
    assert not hasattr(packet_only, "qpos") and not hasattr(packet_only, "qvel")


def test_joint_motor_limits_lag_and_braking():
    p = load_parameters()["articulation"]
    motors = TorqueMotors(p, 4, "peak_torque_per_joint")
    speed = p["no_load_speed"]
    velocities = np.array([11, -13, -13, 11]) * speed / 12
    applied, saturated = motors.step(np.array([100, -100, 100, -100]), velocities, 0.001)
    assert saturated.all()
    assert applied[0] > 0 and applied[1] == 0
    assert applied[2] > 0 and applied[3] < 0
    assert max(abs(applied)) < p["peak_torque_per_joint"]
    for _ in range(100):
        applied, _ = motors.step(np.array([100, -100, 100, -100]), velocities, 0.001)
        assert abs(applied[0]) <= p["peak_torque_per_joint"] / 12 + 1e-10
        assert max(abs(applied)) <= p["peak_torque_per_joint"]
