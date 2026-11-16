import math

import mujoco
import numpy as np
import pytest

from bipe.m10 import plant as full
from bipe.m13 import drives
from bipe.m13 import plant as tp
from bipe.parameters import load_parameters


@pytest.fixture(scope="module")
def plants():
    p10, p13 = load_parameters(profile="m10"), load_parameters(profile="m13")
    return p10, p13, full.build(p10), tp.build(p13)


def _posed(m, d, reset, p):
    reset(m, d, p)
    mujoco.mj_forward(m, d)
    chassis = m.body("chassis").id
    return m.body_subtreemass[chassis], d.subtree_com[chassis].copy()


def test_mass_and_centre_of_mass_match_full_plant(plants):
    p10, p13, (m10, d10), (m13, d13) = plants
    mass10, com10 = _posed(m10, d10, full.reset, p10)
    mass13, com13 = _posed(m13, d13, tp.reset, p13)
    assert mass13 == pytest.approx(mass10, abs=1e-9)
    assert np.allclose(com13, com10, atol=2e-5)


def test_pitch_inertia_matches_full_plant_with_rotor_armature(plants):
    p10, p13, (m10, d10), (m13, d13) = plants
    full.reset(m10, d10, p10)
    tp.reset(m13, d13, p13)

    def pitch_inertia(m, d):
        # Mass-matrix entry of the free base's body-frame pitch rate.
        dense = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, dense)
        return dense[4, 4]

    assert pitch_inertia(m13, d13) == pytest.approx(pitch_inertia(m10, d10), rel=1e-3)


def test_rotor_armature_is_reflected_spin_inertia(plants):
    _, p13, _, (m13, _) = plants
    rotor = np.asarray(p13["rigid_bodies"]["left_thighrotor"]["inertia_kg_m2"])[1][1]
    hip = m13.joint("left_hip").dofadr[0]
    assert m13.dof_armature[hip] == pytest.approx(16 * rotor)


def test_rest_pose_touches_with_tires_only(plants):
    _, p13, _, (m13, d13) = plants
    tp.reset(m13, d13, p13)
    names = {m13.geom(g).name for c in d13.contact for g in (c.geom1, c.geom2)}
    assert names == {"ground", "left_tire", "right_tire"}


def test_training_plant_has_no_callbacks_and_steps(plants):
    _, p13, _, _ = plants
    m, d = tp.build(p13)
    tp.reset(m, d, p13)
    assert m.nv == 14 and m.nu == 6 and m.ntendon == 2
    assert m.nmeshvert <= 3 * p13["training"]["shell"]["pad_hull_vertices"]
    for _ in range(200):
        mujoco.mj_step(m, d)
    assert np.isfinite(d.qpos).all()


def _full_drives(p):
    """MechanicalDrives' motor functions without building the model."""
    obj = full.MechanicalDrives.__new__(full.MechanicalDrives)
    obj.p, obj.ratio = p, p["joint_drive"]["ratio"]
    obj.phase_current = np.zeros(6)
    obj.peak_bus_limited = np.zeros(2)
    return obj


def test_motor_models_equal_mechanical_drives():
    p = load_parameters(profile="m10")
    ref = _full_drives(p)
    rng = np.random.default_rng(3)
    current = np.zeros(6)
    hm, gl = p["physics"]["hip_motor"], p["physics"]["hub_motor"]
    for _ in range(300):
        request = rng.normal(0, 1.0, 6) * np.r_[np.ones(4), 0.3 * np.ones(2)]
        speed = rng.normal(0, 1, 6) * np.r_[np.full(4, 200.0), np.full(2, 80.0)]
        dt = 0.00025
        want_hip = ref._hip_motor(request[:4], speed[:4], dt)
        want_hub = ref._hub_motor(request[4:], speed[4:], dt)
        hip = drives.hip_motor(np, hm, ref.ratio, request[:4], speed[:4], current[:4], dt)
        hub = drives.hub_motor(np, gl, request[4:], speed[4:], current[4:], dt)
        current = np.r_[hip[3], hub[2]]
        assert np.allclose(hip[0], want_hip[0]) and np.allclose(hip[1], want_hip[1])
        assert (hip[2] == want_hip[2]).all()
        assert np.allclose(hub[0], want_hub[0]) and (hub[1] == want_hub[2]).all()
        assert np.allclose(current, ref.phase_current)


def test_knee_belt_spring_has_m10_stiffness(plants):
    _, p13, _, (m13, _) = plants
    belt = m13.tendon("left_knee_belt").id
    assert m13.tendon_stiffness[belt] == pytest.approx(
        p13["physics"]["belts"]["knee_stiffness_Nm_rad"]
    )
    coef = {
        m13.joint(m13.wrap_objid[m13.tendon_adr[belt] + i]).name: m13.wrap_prm[
            m13.tendon_adr[belt] + i
        ]
        for i in range(m13.tendon_num[belt])
    }
    assert coef == {"left_knee": 1, "left_hip": -1, "left_shin_joint": -1}


def test_shell_primitives_enclose_hulls(plants):
    _, p13, _, _ = plants
    for row in tp.shell(p13):
        assert row["primitive_volume_m3"] >= row["hull_volume_m3"] * (1 - 1e-9)
        assert not math.isnan(row["primitive_volume_m3"])


def test_m14_belts_hold_at_their_stiffest_corner():
    """Spring hip belts and knee dead band: rest is inside the dead bands, and the
    stiffest hip belt (with its rotor armature floor) holds a PD stand at 1 ms."""
    import copy

    p = copy.deepcopy(load_parameters(profile="m13"))
    p["training"]["drives"].update(hip_belt="spring", knee_lash="deadband")
    assert tp.rotor_armature_floor(p, 100.0) == 0
    assert tp.rotor_armature_floor(p, 713.3) > 0
    m, d = tp.build(p)
    tp.reset(m, d, p)
    tp.Drives(m, p, {"hip_belt_stiffness_Nm_rad": 713.3, "hip_backlash_deg": 0.1125})
    assert m.dof_armature[m.joint("left_thigh_rotor_joint").dofadr[0]] > 0
    for _ in range(1000):  # unpowered: the robot sits down on its belts
        mujoco.mj_step(m, d)
    assert np.isfinite(d.qpos).all()
    belt = d.ten_length[m.tendon("left_thigh_hip_belt").id]
    assert abs(belt) < math.radians(2)
