import json

import mujoco
import numpy as np
import pytest

from bipe.m9_impact import (
    MaterialContacts,
    fastener_capacities,
    foam_stress,
    hardware_ledger,
    hollow_section,
    load_materials,
    mechanical_energy,
    paired_reactions,
    shell_strip,
    tire_force,
)
from bipe.parameters import ROOT


@pytest.fixture
def inputs():
    return json.loads((ROOT / "config/robot.json").read_text())["m9_cad"][
        "parameters"
    ], load_materials()


def test_edge_reactions_reproduce_statics_and_reverse_load():
    force = np.array([0, 0, 1220.3512707158216])
    point = [0, 0.0935, 0]
    moment = np.cross(point, force)
    row = paired_reactions(force, moment, [0.0693, 0.0885])
    reactions = np.array(row["signed_reactions_N"])
    np.testing.assert_allclose(reactions.sum(axis=0), force)
    np.testing.assert_allclose(
        sum(np.cross([0, y, 0], f) for y, f in zip([0.0693, 0.0885], reactions)), moment
    )
    assert reactions[0, 2] < 0
    assert 1350 / max(row["equivalent_static_N"]) == pytest.approx(0.877677113)
    axial = paired_reactions(force + [0, 2000, 0], moment, [0.0693, 0.0885])
    assert max(axial["equivalent_static_N"]) > max(row["equivalent_static_N"])


def test_material_curves_remain_finite_and_do_not_clip_overload(inputs):
    p, materials = inputs
    stress = foam_stress(np.array([0, 0.25, 0.5, 0.75]), materials)
    np.testing.assert_allclose(stress[:3], [0, 163000, 251000])
    assert stress[3] > stress[2]
    depths = np.linspace(0, 0.004, 100)
    forces = [tire_force(p, x, materials) for x in depths]
    assert np.isfinite(forces).all()
    assert np.diff(forces).min() >= 0
    assert forces[-1] > forces[-20]
    work = np.trapezoid(forces, depths)
    assert work > 0
    contact = MaterialContacts(p, materials)
    assert contact.force("ShellRear", 0.008) > 2 * contact.force("ShellRear", 0.004)


def test_hollow_axle_bending_is_not_solid_shaft_credit():
    _, inertia, section, polar = hollow_section(0.008, 0.006)
    assert section == pytest.approx(np.pi * (0.008**4 - 0.006**4) / (32 * 0.008))
    assert polar == 2 * inertia
    assert section < hollow_section(0.008, 0)[2]


def test_hardware_fits_existing_allocation_but_requires_cad_adoption(inputs):
    _, materials = inputs
    ledger = hardware_ledger(materials)
    assert 0.025 < ledger["total_mass_kg"] < 0.055
    for group in materials["fasteners"]["schedule"]:
        capacities = fastener_capacities(group, materials)
        assert min(capacities.values()) > 0
    group = next(
        row
        for row in materials["fasteners"]["schedule"]
        if row["name"] == "wheel_fork_and_bearing_cap_to_rod"
    )
    # Parent edge tearout, rather than high bolt strength, controls this narrow
    # existing3mm link. This distinguishes actual loadpath from screw catalogs.
    assert fastener_capacities(group, materials)["parent_tearout_N"] < 400


def test_contact_conversion_matches_physical_force_depth_for_two_masses(inputs):
    p, materials = inputs
    stiffness = shell_strip(p, materials)["stiffness_N_m"]
    depths = []
    for mass in [0.2, 1.8]:
        m = mujoco.MjModel.from_xml_string(f"""
          <mujoco><option timestep='.0001' gravity='0 0 0' integrator='implicitfast'
          cone='elliptic'/><worldbody>
          <geom name='ground' type='plane' size='1 1 .1'/>
          <body name='coupon' pos='0 0 .01'><joint type='slide' axis='0 0 1'/>
          <inertial mass='{mass}' pos='0 0 0' diaginertia='.0001 .0001 .0001'/>
          <geom name='ShellCoupon' type='box' size='.01 .01 .01' mass='0'/>
          </body></worldbody></mujoco>""")
        d = mujoco.MjData(m)
        d.qpos[0] = 0.00001
        contact = MaterialContacts(p, materials)
        contact.configure(m)
        for _ in range(8000):
            d.qfrc_applied[0] = -5
            mujoco.mj_step1(m, d)
            contact(m, d)
            mujoco.mj_step2(m, d)
        depth = -d.qpos[0]
        depths.append(depth)
        assert depth == pytest.approx(5 / stiffness, rel=0.01)
    # Force stiffness must not secretly be proportional to modeled bodymass.
    assert depths[0] == pytest.approx(depths[1], rel=0.01)


def _box_on_incline(inputs, slope_deg, duration=1.0):
    """1.8 kg shell box under gravity tilted by slope_deg; returns x(t)."""
    p, materials = inputs
    m = mujoco.MjModel.from_xml_string("""
      <mujoco><option timestep='.00025' cone='elliptic'/><worldbody>
      <geom name='ground' type='plane' size='5 5 .1' friction='.8 .005 .0001' condim='6'/>
      <body pos='0 0 .0099'><freejoint/>
      <geom name='ShellCoupon' type='box' size='.05 .05 .01' mass='1.8'
       friction='.8 .005 .0001' condim='6'/></body></worldbody></mujoco>""")
    slope = np.radians(slope_deg)
    m.opt.gravity[:] = [9.81 * np.sin(slope), 0, -9.81 * np.cos(slope)]
    d = mujoco.MjData(m)
    contact = MaterialContacts(p, materials)
    contact.configure(m)
    x = []
    for _ in range(round(duration / m.opt.timestep)):
        mujoco.mj_step1(m, d)
        contact(m, d)
        mujoco.mj_step2(m, d)
        x.append(d.qpos[0])
    return np.arange(1, len(x) + 1) * m.opt.timestep, np.array(x)


def test_friction_rows_stick_below_mu_and_slide_at_coulomb_rate(inputs):
    # 10 deg uses 0.176 of mu = 0.8: no creep once settled.
    t, x = _box_on_incline(inputs, 10)
    settled = t > 0.5
    creep = (x[-1] - x[settled][0]) / (t[-1] - t[settled][0])
    assert abs(creep) < 1e-4
    # 45 deg exceeds mu: sliding acceleration is the Coulomb value.
    t, x = _box_on_incline(inputs, 45)
    sliding = t > 0.2
    acceleration = 2 * np.polyfit(t[sliding], x[sliding], 2)[0]
    expected = 9.81 * (np.sin(np.radians(45)) - 0.8 * np.cos(np.radians(45)))
    assert acceleration == pytest.approx(expected, rel=0.02)


def test_rotational_energy_decomposition_matches_native_kinetic_energy():
    from bipe.m9_plant import MechanicalDrives, build, parameters, reset

    p = parameters(json.loads((ROOT / "config/robot.json").read_text()))
    m, d = build(p)
    reset(m, d, p, 40)
    drives = MechanicalDrives(m, p)
    d.qvel[:] = np.linspace(-0.2, 0.2, m.nv)
    drives.forward(m, d)
    row = mechanical_energy(m, d, drives)
    assert row["body_rotation_J"] > 0
    assert row["body_translation_J"] > 0
    assert row["body_rotation_J"] + row["body_translation_J"] == pytest.approx(
        row["kinetic_J"], abs=1e-12
    )


def test_rebuilt_contacts_keep_native_noslip_finite_and_base_free():
    from bipe.m9_plant import MechanicalDrives, build, parameters, reset

    p = parameters(json.loads((ROOT / "config/robot.json").read_text()))
    m, _ = build(p)
    m.opt.noslip_iterations = 5
    d = mujoco.MjData(m)
    reset(m, d, p, offset_deg=10)
    drives = MechanicalDrives(m, p)
    drives.contacts.cache_enabled = False
    initial_base = d.qpos[:7].copy()
    assert m.neq == 2 and m.jnt_type[m.joint("base").id] == mujoco.mjtJoint.mjJNT_FREE
    for _ in range(200):
        before = d.qpos.copy()
        drives.forward(m, d)
        np.testing.assert_array_equal(d.qpos, before)
        drives.step(m, d, np.zeros(6))
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
        rows = d.efc_type[: d.nefc] == mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON
        assert rows.sum() == 4
        assert np.max(abs(d.efc_force[: d.nefc][rows])) <= 1.85 + 1e-10
    assert np.linalg.norm(d.qpos[:7] - initial_base) > 1e-3
    assert drives.contacts.peak_depth
    assert not np.any(d.ctrl)
    assert drives.metrics()["state_projection_count"] == 0
