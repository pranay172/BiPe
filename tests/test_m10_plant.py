"""M10 serial-leg plant: CAD mass closure, kinematics, virtual work and passive drives."""

import json
import math

import mujoco
import numpy as np
import pytest

from bipe.m10.kinematics import (
    body_placements,
    com_and_axle,
    leg_fk,
    leg_ik,
    leg_jacobian,
    posture,
    posture_target,
    support_feedforward,
    wheel_carrier_angle_rate,
)
from bipe.m10.plant import (
    JOINTS,
    ROTORS,
    MechanicalDrives,
    assembled_world,
    build,
    geometry_check,
    hub_drive_resistance,
    parameters,
    reset,
    truth,
)
from bipe.parameters import ROOT, load_parameters

SHARED = json.loads((ROOT / "config/robot.json").read_text())


@pytest.fixture(scope="module")
def p():
    return parameters(SHARED)


@pytest.mark.parametrize("angle", [30, 40, 60, 65])
def test_free_serial_plant_CAD_mass_and_kinematics(p, angle):
    m, d = build(p)
    reset(m, d, p, angle)
    # World, chassis and six bodies per side; free base plus 6 hinges per side.
    assert m.nbody == 14 and m.nv == 18 and m.nu == 6 and m.neq == 0
    assert m.jnt_type[m.joint("base").id] == mujoco.mjtJoint.mjJNT_FREE
    assert np.all(m.dof_armature == 0)
    assert sum(m.body_mass) == pytest.approx(SHARED["m10"]["cad"]["allocated_kg"], abs=1e-3)
    q = posture(angle, p)
    height, x = posture_target(angle, p)
    assert leg_fk(q[:2], p) == pytest.approx([x, -height], abs=1e-12)
    origin = d.xpos[m.body("chassis").id]
    rot = d.xmat[m.body("chassis").id].reshape(3, 3)
    axle = rot.T @ (d.site("left_axle").xpos - origin)
    _, expected_axle = com_and_axle(p, q)
    assert axle[[0, 2]] == pytest.approx(expected_axle, abs=1e-9)
    # The plant COM includes belt preload deflection of the rotors only.
    com = rot.T @ (d.subtree_com[m.body("chassis").id] - origin)
    expected_com, _ = com_and_axle(p, q)
    assert com[[0, 2]] == pytest.approx(expected_com, abs=2e-6)
    s = truth(m, d, p)
    assert s["q"] == pytest.approx(q, abs=1e-12)
    assert all(m.jnt_limited[m.joint(n).id] for n in JOINTS)
    ground = m.geom("ground").id
    tires = {m.geom(f"{side}_tire").id for side in ["left", "right"]}
    assert d.ncon >= 2
    assert all(c.geom1 == ground and c.geom2 in tires for c in d.contact)


def test_geometry_check_and_registry(p):
    check = geometry_check(p)
    assert check["passed"], check
    loaded = load_parameters(profile="m10")
    assert loaded["profile"] == "m10" and loaded["rigid_bodies"].keys() == p["rigid_bodies"].keys()
    world = assembled_world(p, posture(40, p), 0.0)
    assert world["mass_kg"] == pytest.approx(check["mass_kg"], abs=1e-12)
    # Level-body postures: the whole-robot COM sits over the axle at every knot.
    for angle in [30, 40, 60, 65]:
        com, axle = com_and_axle(p, posture(angle, p))
        assert com[0] == pytest.approx(axle[0], abs=1e-6)


def test_low_posture_and_unpowered_criterion_are_m10_only():
    from bipe.robustness import low_posture, matrix

    m10, m9 = load_parameters(profile="m10"), load_parameters(profile="m9")
    assert low_posture(m10) == 60 and low_posture(m9) == 65 and low_posture(None) == 65
    lows = [c for c in matrix(m10) if c["family"] == "push" and c["mode"] == "low"]
    assert lows and all(c["disturbance"]["initial_posture_deg"] == 60 for c in lows)
    # The unpowered case may come to rest on the body (m10), not only fall past 30 deg.
    from bipe.profiles import native

    assert native(m10)["unpowered_rest_on_body"] and not native(m9).get("unpowered_rest_on_body")


def test_serial_jacobian_and_feedforward_virtual_work(p):
    eps = 1e-6
    for h in [0.024, 0.036, 0.067, 0.099]:
        for x in [-0.02, -0.01, 0.0, 0.01]:
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

            def potential(angles, up=up, mass=mass):
                place = body_placements(p, angles)
                weight = sum(
                    r["mass_kg"] * (place[n][0] + place[n][1] @ r["com_m"])[[0, 2]] @ up
                    for n, r in p["rigid_bodies"].items()
                )
                support = mass / 2 * sum(leg_fk(a, p) @ up for a in [angles[:2], angles[2:]])
                return p["sizing"]["gravity"] * (weight - support)

            expected = np.array(
                [
                    (potential(q + eps * np.eye(4)[i]) - potential(q - eps * np.eye(4)[i]))
                    / (2 * eps)
                    for i in range(4)
                ]
            )
            assert support_feedforward(p, q, pitch, roll) == pytest.approx(expected, abs=2e-8)


def test_wheel_odometry_carrier_is_the_shin_angle(p):
    q = posture(40, p)
    assert wheel_carrier_angle_rate(q[:2], np.array([0.3, -0.7]), p) == (q[1], -0.7)


def test_rotor_encoders_read_output_angles_including_belt_deflection(p):
    m, d = build(p)
    reset(m, d, p)
    ratio = p["joint_drive"]["ratio"]
    for name, rotor in zip(JOINTS, ROTORS, strict=True):
        assert d.sensor(name + "_encoder").data[0] == pytest.approx(
            d.qpos[m.joint(rotor).qposadr[0]] / ratio, abs=1e-12
        )
    # Static preload: the reading differs from the true output by lash plus deflection.
    s = truth(m, d, p)
    reading = np.array([d.sensor(n + "_encoder").data[0] for n in JOINTS])
    assert np.all(abs(reading - s["q"]) > 0)
    assert np.max(abs(reading - s["q"])) < np.radians(2)


@pytest.mark.parametrize("slope_deg", [-10, 0, 10])
def test_slope_reset_preserves_tire_contact(p, slope_deg):
    m, d = build(p)
    reset(m, d, p, slope_deg=slope_deg)
    slope = np.radians(slope_deg)
    normal = d.geom_xmat[m.geom("ground").id].reshape(3, 3)[:, 2]
    assert normal == pytest.approx([-np.sin(slope), 0, np.cos(slope)], abs=1e-12)
    for side in ["left", "right"]:
        assert normal @ d.site(side + "_axle").xpos == pytest.approx(
            p["geometry"]["wheel_radius"] - 0.0001
        )
    assert d.ncon >= 2


@pytest.mark.parametrize("slope_deg", [-10, 4, 10])
def test_tilted_ground_keeps_tire_contacts_away_from_the_origin(p, slope_deg):
    """The world bound is refitted to the tilted plane: the midphase keeps every contact
    the brute-force pair test finds, uphill and downhill (wheels carry their bumper rings)."""
    p = {**p, "physics": {**p["physics"], "bumpers": {**p["physics"]["bumpers"], "enabled": True}}}
    m, d = build(p)
    reset(m, d, p, slope_deg=slope_deg)
    start = d.qpos.copy()
    midphase = int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE)
    for dx in (-0.3, -0.03, 0.03, 0.3):
        counts = []
        for flags in (0, midphase):
            m.opt.disableflags = flags
            d.qpos[:] = start
            d.qpos[0] += dx
            d.qpos[2] += dx * math.tan(math.radians(slope_deg))
            mujoco.mj_forward(m, d)
            counts.append(d.ncon)
        assert counts[0] == counts[1] >= 2, (dx, counts)
    m.opt.disableflags = 0


def test_motor_limits_lag_and_no_state_projection(p):
    m, d = build(p)
    reset(m, d, p)
    drives = MechanicalDrives(m, p)
    before = d.qpos.copy()
    _, saturated = drives.apply(m, d, np.full(6, 100.0))
    assert d.qpos == pytest.approx(before, abs=0)
    assert saturated.all()
    hm, gl = p["physics"]["hip_motor"], p["physics"]["hub_motor"]
    assert max(abs(d.ctrl[:4])) <= hm["peak_torque_motor_Nm"]
    assert max(abs(d.ctrl[4:])) <= gl["peak_torque_Nm"]
    assert max(abs(drives.phase_current[:4])) <= hm["FOC_current_slew_limit_A_s"] * m.opt.timestep
    assert drives.metrics()["state_projection_count"] == 0


def test_GL40_voltage_limit_reduces_motoring_torque_with_speed(p):
    m, d = build(p)
    reset(m, d, p)
    drives = MechanicalDrives(m, p)
    gl = p["physics"]["hub_motor"]
    out = []
    for speed in [0.0, 10.0, 27.8, 130.0]:  # no-load speed is V / Ke = 121 rad/s
        drives.phase_current[:] = 0
        for _ in range(200):
            torque, _, _ = drives._hub_motor(np.full(2, 1.0), np.full(2, speed), 2.5e-4)
        out.append(torque[0])
    volt_limited = (
        gl["torque_constant_Nm_A"]
        * (gl["drive_voltage_limit_V"] - gl["back_emf_V_s_rad"] * 27.8)
        / hub_drive_resistance(gl)
    )
    assert out[0] == pytest.approx(gl["peak_torque_Nm"], rel=0.02)
    assert out[2] == pytest.approx(volt_limited, rel=1e-6)
    assert out[0] >= out[1] >= out[2] > out[3] == 0
    # Braking is limited only by the peak current.
    drives.phase_current[:] = 0
    for _ in range(200):
        torque, _, _ = drives._hub_motor(np.full(2, -1.0), np.full(2, 100.0), 2.5e-4)
    assert torque[0] == pytest.approx(-gl["peak_torque_Nm"], rel=0.02)


def test_unpowered_free_fall_belt_energy_is_passive(p):
    from bipe.m9_impact import mechanical_energy

    m, d = build(p)
    reset(m, d, p)
    d.qpos[2] += 1
    drives = MechanicalDrives(m, p)
    # Pre-wind one hip belt beyond its lash: a finite overload, released in flight.
    d.qpos[drives.rq[0]] += drives.ratio * (drives.half[0] + 0.02)

    def energy():
        drives.forward(m, d)
        return mechanical_energy(m, d, drives)["total_J"]

    initial = energy()
    measured = [initial]
    for _ in range(400):
        drives.step(m, d, np.zeros(6))
        measured.append(energy())
        assert d.ncon == 0
    assert max(measured) <= initial + 5e-6
    assert measured[-1] < initial
    assert np.max(abs(drives.motor_torque)) == 0


def test_unpowered_robot_collapses_onto_its_body(p):
    """No hidden stabilization: with zero motor commands the legs fold and the body comes
    to rest on its belly: on the skid pad under the tub (phase D4; the CAD sit pose has
    the skid on the floor), or on the tub itself when the bumpers are disabled."""
    m, d = build(p)
    reset(m, d, p, offset_deg=5)
    drives = MechanicalDrives(m, p)
    hip = m.joint("left_hip").id
    mujoco.mj_forward(m, d)
    standing = d.xanchor[hip][2]
    for _ in range(round(1 / m.opt.timestep)):
        drives.step(m, d, np.zeros(6))
    ground = m.geom("ground").id
    resting = {
        m.geom(g).name
        for i in range(d.ncon)
        for g in d.contact[i].geom
        if ground in d.contact[i].geom and g != ground
    }
    assert d.xanchor[hip][2] < standing - 0.04
    assert resting & {"BodyTub", "BatteryHatch", "SkidPad"}
    assert np.isfinite(d.qpos).all()


def test_material_contacts_apply_the_tire_law_on_the_multibody_robot(p):
    """The solved tire force equals the material law, not law x J M^-1 J.T / diagA."""
    from bipe.m10.plant import tire_force

    m, d = build(p)
    reset(m, d, p)
    drives = MechanicalDrives(m, p)
    drives.forward(m, d)
    tires = {m.geom(f"{s}_tire").id for s in ("left", "right")}
    rows = [i for i in range(d.ncon) if tires & set(d.contact[i].geom)]
    assert rows
    for i in rows:
        c = d.contact[i]
        force = np.zeros(6)
        mujoco.mj_contactForce(m, d, i, force)
        count = sum(1 for j in rows if set(d.contact[j].geom) == set(c.geom))
        law = tire_force(p, -c.dist, drives.contacts.materials) / count
        # The reset is at rest (zero normal speed), so only the elastic term acts.
        assert force[0] == pytest.approx(law, rel=0.01)


def test_shell_backing_sits_at_the_measured_gap_only_on_listed_parts(p):
    """A listed part's strip law reaches the metal backing at its measured gap; other
    shells keep the M9 travel assumption."""
    from bipe.m10.plant import Contacts

    m, _ = build(p)
    law = Contacts(p)
    law.configure(m)
    travel = law.shell["maximum_travel_m"]
    k = law.shell["stiffness_N_m"] * law.scale
    gap = p["physics"]["contact"]["shell_backing"]["parts"]["BodyTub"]["travel_m"]
    assert travel < gap
    depth = (travel + gap) / 2
    assert law.force("BodyTub", depth) == pytest.approx(k * depth)
    assert law.force("HeadFront", depth) > 10 * k * depth
    assert law.force("BodyTub", gap + 1e-4) > law.force("BodyTub", gap) + 1.0


def test_end_stop_pad_law_and_passive_fall_energy(p):
    """Stop pads follow M = E A r^2 theta / t; a passive collapse into them never gains energy."""
    from bipe.m10.falls import CoulombContacts, energy, release

    m, d = build(p)
    contacts = CoulombContacts(m, p)
    pads = p["physics"]["end_stop_pads"]
    hip = m.joint("left_hip").id
    pad = pads["joints"]["thigh_a1"]
    e = contacts.normal.materials["materials"][pads["material"]]["modulus_Pa"]
    k = e * pad["area_m2"] * pad["radius_m"] ** 2 / pad["thickness_m"]
    assert contacts.normal.stop_torque(hip, 0.01) == pytest.approx(0.01 * k)
    travel = pads["max_strain"] * pad["thickness_m"] / pad["radius_m"]
    assert contacts.normal.stop_torque(hip, travel + 0.01) > k * (travel + 0.01)
    reset(m, d, p)
    release(m, d, p, "forward")
    drives = MechanicalDrives(m, p)
    drives.forward(m, d, contacts)
    previous = energy(m, d, drives, contacts.normal)
    initial, resampled = previous["total_J"], 0.0
    for _ in range(round(0.6 / m.opt.timestep)):
        drives.step(m, d, np.zeros(6), contact_callback=contacts)
        now = energy(m, d, drives, contacts.normal)
        rise = now["total_J"] - previous["total_J"]
        stored = now["contact_elastic_J"] - previous["contact_elastic_J"]
        # Motion energy never rises; steps in stored contact energy alone are mesh-plane
        # contact resampling, bounded in total.
        assert rise <= 1e-3 or rise - stored <= 1e-3
        resampled += rise if rise > 1e-3 else 0.0
        previous = now
    assert resampled < 0.02 * initial
    assert now["total_J"] < initial
    assert contacts.normal.peak_stop_strain


def test_stop_pad_share_and_remedy_candidates():
    from bipe.m10.falls import candidate_config, with_pad

    react = {"radial_N": [10.0, 20.0], "axial_N": 4.0, "equivalent_static_N": [10.0, 20.0]}
    # Overhung pad beyond the outer bearing: lever shares 0.5 F and 1.5 F.
    out = with_pad(react, 100.0, 0.03, 0.0, 0.02)
    assert out["radial_N"] == pytest.approx([60.0, 170.0])
    assert out["equivalent_static_N"] == pytest.approx([60.0, 170.0])
    shared = json.loads((ROOT / "config/robot.json").read_text())
    for name, row in shared["m10"]["structure"]["candidates"].items():
        if name == "basis":
            continue
        merged = candidate_config(shared, name)
        assert merged["m10"]["structure"]["required_factor"] == 2.0
        assert "mass_delta_kg_estimate" in row
    soft = candidate_config(shared, "soft_pads")["m10"]["physics"]["end_stop_pads"]["joints"]
    assert soft["knee_angle"] == {
        **shared["m10"]["physics"]["end_stop_pads"]["joints"]["knee_angle"],
        "thickness_m": 0.008,
    }
    assert candidate_config(shared, None) is shared


def test_exterior_hulls_coincide_with_the_cad_at_the_nominal_pose(p):
    """Collision hulls sit where the CAD puts the parts (the tessellation is in the CAD
    world frame, ground at z = 0; the plant link frames are hip-relative)."""
    import gzip

    from bipe.m10.plant import MESH

    m, d = build(p)
    reset(m, d, p)
    mujoco.mj_forward(m, d)
    pose = json.loads((ROOT / "artifacts/m10/cad/mass.json").read_text())["pose"]
    cad_hip = np.array([-pose["axle_m"][0], 0.0, pose["hip_height_m"]])
    plant_hip = d.xanchor[m.joint("left_hip").id]
    assert abs(math.degrees(d.qpos[m.joint("left_hip").qposadr[0]]) - pose["a1_deg"]) < 0.1
    cad, seen = {}, {}
    for c in json.loads(gzip.decompress(MESH.read_bytes()))["poses"]["nominal"]:
        count = seen.get(c["name"], 0)
        seen[c["name"]] = count + 1
        cad[c["name"] + (f"_{count}" if count else "")] = np.asarray(c["vertices"])
    checked = 0
    for g in range(m.ngeom):
        name = m.geom(g).name
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or name not in cad:
            continue
        mesh = m.geom_dataid[g]
        start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
        world = m.mesh_vert[start : start + count] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        expected = cad[name] - cad_hip + plant_hip
        assert np.allclose(world.min(0), expected.min(0), atol=1.5e-3), name
        assert np.allclose(world.max(0), expected.max(0), atol=1.5e-3), name
        checked += 1
    assert checked >= 10


def test_tilted_tire_face_bears_on_an_edge_sliver(p):
    """The lateral tire law is the full annulus only for a flat face; tilted, it is the
    penetrated sliver, and upright tires keep the radial law."""
    from bipe.m10.plant import _tilted_face_integral, tire_force

    g = p["geometry"]
    outer, inner = g["wheel_radius"], g["wheel_radius"] - g["tire_section"]
    annulus = math.pi * (outer**2 - inner**2)
    assert _tilted_face_integral(outer, inner, 1e-4, 1.0) == pytest.approx(annulus * 1e-4)
    tilted = [_tilted_face_integral(outer, inner, 1e-4, a) for a in (0.999, 0.98, 0.7)]
    assert tilted == sorted(tilted, reverse=True)
    assert tilted[1] < 0.01 * annulus * 1e-4
    materials = json.loads((ROOT / "config/m9-impact-materials.json").read_text())
    assert tire_force(p, 1e-4, materials, 0.0) > 0
    assert tire_force(p, 1e-4, materials, 0.03) == pytest.approx(
        0.97 * tire_force(p, 1e-4, materials, 0.0), rel=0.01
    )


def test_inherited_wheel_lag_variant_reaches_the_hub_drives(p):
    from bipe.moving_validate import jobs

    job = next(j for j in jobs(nominal=p) if j["name"] == "wheel_lag_25ms_drive")
    torques = {}
    for name, plant in [("nominal", p), ("lagged", job["plant"])]:
        m, d = build(plant)
        reset(m, d, plant)
        drives = MechanicalDrives(m, plant)
        for _ in range(200):
            torque, _, _ = drives._hub_motor(np.full(2, 0.1), np.zeros(2), 2.5e-4)
        torques[name] = torque[0]
    assert job["plant"]["physics"]["hub_motor"]["FOC_time_constant_s"] == 0.025
    assert torques["nominal"] == pytest.approx(0.1, rel=1e-6)
    assert torques["lagged"] == pytest.approx(0.1 * -math.expm1(-0.05 / 0.025), rel=1e-6)
