"""M9 physical topology, CAD mass closure, virtual work and passive drives."""

import copy
import json

import mujoco
import numpy as np
import pytest

from bipe.m9_kinematics import (
    body_placements,
    com_and_axle,
    leg_fk,
    leg_ik,
    leg_jacobian,
    posture,
    support_feedforward,
)
from bipe.m9_plant import JOINTS, MechanicalDrives, build, geometry_check, parameters, reset
from bipe.parameters import ROOT


@pytest.fixture
def p():
    return parameters(json.loads((ROOT / "config/robot.json").read_text()))


@pytest.mark.parametrize("angle,height", [(30, 0.118), (40, 0.100), (60, 0.070), (65, 0.045)])
def test_free_five_bar_CAD_mass_and_output_kinematics(p, angle, height):
    m, d = build(p)
    reset(m, d, p, angle)
    assert m.nbody == 22 and m.nv == 26 and m.nu == 6 and m.neq == 2
    assert m.ntendon == 4 and m.tendon_frictionloss == pytest.approx(np.full(4, 1.85))
    assert m.jnt_type[m.joint("base").id] == mujoco.mjtJoint.mjJNT_FREE
    assert sum(m.body_mass) == pytest.approx(1.7940856482928784, abs=1e-12)
    assert np.all(m.dof_armature == 0)  # Actual motor-angle CAD rotor tensors.
    q = posture(angle, p)
    assert leg_fk(q[:2], p) == pytest.approx([0, -height], abs=1e-12)
    origin = d.xpos[m.body("chassis").id]
    rot = d.xmat[m.body("chassis").id].reshape(3, 3)
    com = rot.T @ (d.subtree_com[m.body("chassis").id] - origin)
    axle = rot.T @ (d.site("left_axle").xpos - origin)
    expected_com, expected_axle = com_and_axle(p, q)
    assert com[[0, 2]] == pytest.approx(expected_com, abs=1e-10)
    assert axle[[0, 2]] == pytest.approx(expected_axle, abs=1e-10)
    assert np.max(abs(d.efc_pos[:6])) < 1e-10
    assert all(m.jnt_limited[m.joint(n).id] for n in JOINTS)
    ground = m.geom("ground").id
    tires = {m.geom(s + "_tire").id for s in ["left", "right"]}
    assert all(c.geom1 == ground and c.geom2 in tires for c in d.contact)
    assert geometry_check(p)["passed"]


def test_five_bar_inverse_jacobian_and_feedforward_virtual_work(p):
    eps = 1e-6
    for h in [0.045, 0.07, 0.1, 0.118]:
        for x in [-0.01, 0, 0.01]:
            pair = leg_ik(x, -h, p)
            assert leg_fk(pair, p) == pytest.approx([x, -h], abs=1e-12)
            numeric = np.column_stack(
                [
                    (leg_fk(pair + eps * np.eye(2)[i], p) - leg_fk(pair - eps * np.eye(2)[i], p))
                    / (2 * eps)
                    for i in range(2)
                ]
            )
            assert leg_jacobian(pair, p) == pytest.approx(numeric, abs=2e-9)
            q = np.tile(pair, 2)
            pitch, roll = -0.14, 0.04
            up = np.array([-np.sin(pitch), np.cos(pitch) * np.cos(roll)])
            mass = sum(r["mass_kg"] for r in p["rigid_bodies"].values())

            def virtual_potential(angles, up=up, mass=mass):
                placements = body_placements(p, angles)
                potential = sum(
                    r["mass_kg"] * (placements[n][0] + placements[n][1] @ r["com_m"])[[0, 2]] @ up
                    for n, r in p["rigid_bodies"].items()
                )
                support_work = mass / 2 * sum(leg_fk(a, p) @ up for a in [angles[:2], angles[2:]])
                return p["sizing"]["gravity"] * (potential - support_work)

            expected = np.array(
                [
                    (
                        virtual_potential(q + eps * np.eye(4)[i])
                        - virtual_potential(q - eps * np.eye(4)[i])
                    )
                    / (2 * eps)
                    for i in range(4)
                ]
            )
            assert support_feedforward(p, q, pitch, roll) == pytest.approx(expected, abs=2e-8)


def test_material_contact_refresh_exposes_current_IMU_acceleration(p):
    """A second contact solve must invalidate the first solve's body cache."""
    m, d = build(p)
    reset(m, d, p)
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    adr = m.sensor("accelerometer").adr[0]
    measured = d.sensordata[adr : adr + 3].copy()
    acceleration = d.qacc.copy()
    mujoco.mj_rnePostConstraint(m, d)
    mujoco.mj_sensorAcc(m, d)
    assert d.qacc == pytest.approx(acceleration, abs=0)
    assert measured == pytest.approx(d.sensordata[adr : adr + 3], abs=1e-12)


@pytest.mark.parametrize("slope_deg", [-10, 0, 10])
def test_slope_reset_rotates_the_physical_plane_and_preserves_tire_contact(p, slope_deg):
    m, d = build(p)
    reset(m, d, p, slope_deg=slope_deg)
    slope = np.radians(slope_deg)
    normal = d.geom_xmat[m.geom("ground").id].reshape(3, 3)[:, 2]
    assert normal == pytest.approx([-np.sin(slope), 0, np.cos(slope)], abs=1e-12)
    for side in ["left", "right"]:
        axle = d.site(side + "_axle").xpos
        assert normal @ axle == pytest.approx(p["geometry"]["wheel_radius"] - 0.0001)
    assert d.ncon >= 2


def test_lost_motion_clutch_release_and_passive_energy_dissipation(p):
    from bipe.m9_impact import mechanical_energy

    m, d = build(p)
    reset(m, d, p)
    # An initial free-flight pose and finite belt preload provide a physical
    # overload; gravity, friction, CAD inertia and the ground stay unchanged.
    d.qpos[2] += 1
    drives = MechanicalDrives(m, p)
    d.qpos[drives.rq[0]] += drives.ratio * (drives.half[0] + 0.06)
    initial_relative = d.qpos[drives.gq] - d.qpos[drives.oq[:4]]

    def energy():
        drives.forward(m, d)
        return mechanical_energy(m, d, drives)["total_J"]

    initial = energy()
    measured = [initial]
    for _ in range(400):
        drives.step(m, d, np.zeros(6))
        measured.append(energy())
        assert d.ncon == 0  # The short free fall has not reached the ground.
    assert m.opt.noslip_iterations == 5
    assert max(measured) <= initial + 5e-6
    assert measured[-1] < initial - 0.1
    relative = d.qpos[drives.gq] - d.qpos[drives.oq[:4]]
    assert relative[0] - initial_relative[0] > 0.01
    assert drives.peak_clutch_torque[0] == pytest.approx(drives.clutch_release, abs=1e-8)
    assert np.max(drives.peak_clutch_torque) <= drives.clutch_release + 1e-10
    assert drives.metrics()["state_projection_count"] == 0
    assert np.max(abs(drives.motor_torque)) == 0


def test_subthreshold_clutches_hold_free_robot_below_output_encoder_lsb(p, monkeypatch, tmp_path):
    from bipe.moving_experiment import simulate
    from bipe.robustness_control import build_candidate

    gains = json.loads((ROOT / "config/controller_m9.json").read_text())
    sensors = json.loads((ROOT / "config/sensors-m9.json").read_text())
    library = build_candidate(p, gains, tmp_path)
    integrate = MechanicalDrives.integrate
    relative, forces, models, projections = [], [], [], []

    def measure(self, m, d, *args, **kwargs):
        if not relative:
            relative.append(d.qpos[self.gq] - d.qpos[self.oq[:4]])
        integrate(self, m, d, *args, **kwargs)
        relative.append(d.qpos[self.gq] - d.qpos[self.oq[:4]])
        forces.append(self.transmitted[:4].copy())
        models.append(m)
        projections.append(self.metrics()["state_projection_count"])

    monkeypatch.setattr(MechanicalDrives, "integrate", measure)
    run = simulate(
        "hold",
        duration=1,
        nominal=copy.deepcopy(p),
        gains=gains,
        sensing="sensors",
        sensor_config=sensors,
        implementation="c",
        firmware_library=library,
        log_dt=0.005,
    )
    displacement = np.array(relative) - relative[0]
    encoder_lsb = 2 * np.pi / sensors["joint_encoder_counts_per_rev"]
    assert np.max(abs(displacement)) < encoder_lsb
    assert len(forces) == round(1 / p["simulation"]["physics_dt"])
    peak_force = np.max(np.abs(forces))
    # Loaded support, not an unloaded coupon. Coulomb tire rows peak near
    # 0.08; the archived viscous rows pre-loaded the hip0 drives to 0.18.
    assert peak_force > 0.05
    assert peak_force < 1.85 - 0.1
    assert not run["failure"]
    assert not any(projections)
    m = models[-1]
    assert m.opt.noslip_iterations == 5
    assert m.jnt_type[m.joint("base").id] == mujoco.mjtJoint.mjJNT_FREE
    assert m.neq == 2  # Only the two physical five-bar foot closures.


def test_motor_lag_slew_caps_output_encoders_and_free_passive_fall(p):
    m, d = build(p)
    reset(m, d, p, offset_deg=10)
    drives = MechanicalDrives(m, p)
    before = d.qpos.copy()
    requested = np.full(6, 100.0)
    applied, saturated = drives.apply(m, d, requested)
    # Forces-only application must not project any robot coordinate.
    assert d.qpos == pytest.approx(before, abs=0)
    assert saturated.all()
    assert max(abs(d.ctrl)) <= 0.6
    assert max(abs(applied[:4])) < 1.65
    assert (
        max(abs(drives.phase_current))
        <= p["physics"]["motor"]["FOC_current_slew_limit_A_s"] * m.opt.timestep
    )
    for n in JOINTS:
        assert d.sensor(n + "_encoder").data[0] == d.qpos[m.joint(n).qposadr[0]]
    # A completely separate unpowered run must actually move its free base.
    reset(m, d, p, offset_deg=10)
    passive = MechanicalDrives(m, p)
    before = d.qpos[:7].copy()
    for _ in range(800):
        passive.step(m, d, np.zeros(6))
    assert np.linalg.norm(d.qpos[:7] - before) > 0.01
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert max(abs(d.ctrl)) == 0


def test_inherited_contact_and_wheel_lag_variants_reach_the_physical_model(p):
    variant = copy.deepcopy(p)
    variant["contact"]["sliding_friction"] = 0.5
    variant["motor"]["torque_time_constant"] = 0.025
    m, d = build(variant)
    reset(m, d, variant)
    assert m.geom_friction[m.geom("left_tire").id, 0] == 0.5
    drive = MechanicalDrives(m, variant)
    measured, _ = drive.apply(m, d, np.full(6, 0.1))
    dt = m.opt.timestep
    assert measured[:4] == pytest.approx(
        0.1 * -np.expm1(-dt / p["articulation"]["torque_time_constant"])
    )
    assert measured[4:] == pytest.approx(0.1 * -np.expm1(-dt / 0.025))


@pytest.mark.parametrize("speed", [0.0, 0.3])
def test_unpowered_contact_fall_clutch_energy_balance(p, speed):
    from bipe.m9_impact import mechanical_energy

    m, d = build(p)
    reset(m, d, p, offset_deg=10)
    drive = MechanicalDrives(m, p)
    # Rolling initial conditions give the free base, both wheels and the
    # native motor rotors consistent finite translational/rotational energy.
    d.qvel[0] = speed
    d.qvel[drive.ov[4:]] = speed / p["geometry"]["wheel_radius"]
    d.qvel[drive.rv[4:]] = drive.ratio * speed / p["geometry"]["wheel_radius"]
    drive.forward(m, d)
    initial = mechanical_energy(m, d, drive)["total_J"]
    excess = 0.0
    for step in range(round(1 / m.opt.timestep)):
        drive.step(m, d, np.zeros(6))
        if step % 10 == 0:
            total = mechanical_energy(m, d, drive)["total_J"]
            excess = max(excess, total + drive.plastic_energy - initial)
    # Includes gravity, actual-angle rotor kinetic energy, finite belt energy
    # and measured native-clutch work. Tiny initial tire preload/solver error
    # is tolerated; spurious multi-joule slip energy is rejected.
    assert excess < 5e-4
    assert drive.contacts.peak_depth
    assert drive.peak_clutch_torque.max() <= 1.85 + 1e-8


def test_contact_cache_preserves_physics_forces_and_sensor_pipeline(
    p, monkeypatch, duration=0.05, gains=None, sensors=None
):
    from bipe.m9_impact import MaterialContacts
    from bipe.moving_experiment import simulate

    gains = gains or json.loads((ROOT / "config/controller_m9.json").read_text())
    sensors = sensors or json.loads((ROOT / "config/sensors-m9.json").read_text())
    original_init, original_integrate = MaterialContacts.__init__, MechanicalDrives.integrate
    enabled, measured, contacts = False, [], []

    def initialize(self, *args, **kwargs):
        kwargs["cache_enabled"] = enabled
        original_init(self, *args, **kwargs)
        contacts.append(self)

    def integrate(self, m, d, *args, **kwargs):
        original_integrate(self, m, d, *args, **kwargs)
        measured.append(
            np.r_[
                d.qpos,
                d.qvel,
                d.qacc,
                d.sensordata,
                d.ctrl,
                d.efc_force,
                self.last_forces,
                self.transmitted,
                m.dof_frictionloss,
            ]
        )

    monkeypatch.setattr(MaterialContacts, "__init__", initialize)
    monkeypatch.setattr(MechanicalDrives, "integrate", integrate)
    previous = None
    for enabled in [False, True]:
        measured.clear()
        contacts.clear()
        run = simulate(
            "hold",
            duration=duration,
            nominal=copy.deepcopy(p),
            gains=copy.deepcopy(gains),
            sensing="sensors",
            sensor_config=copy.deepcopy(sensors),
            log_dt=0.005,
        )
        result = [np.array(measured), run["trace"], run["qpos"]]
        if previous is not None:
            for actual, expected in zip(result, previous, strict=True):
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
            assert sum(c.cache_hits for c in contacts) > 0
        previous = result
