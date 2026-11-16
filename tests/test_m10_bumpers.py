"""M10 phase D bumper laws: foam face pads and flexible TPU ears."""

import json
import math

import mujoco
import numpy as np
import pytest

from bipe.m10.bumpers import (
    FacePads,
    ear_cantilever,
    face_samples,
    foam_energy_density,
    foam_stress,
    hull_vertices,
)
from bipe.parameters import ROOT

MATERIALS = json.loads((ROOT / "config/m9-impact-materials.json").read_text())["materials"]
FOAM = MATERIALS["ld45_upper"]


def _pad_model(shape_xml):
    xml = f"""<mujoco><worldbody><geom name="ground" type="plane" size="1 1 .1"/>
    <body name="b" pos="0 0 0.1"><freejoint/>{shape_xml}</body></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def test_face_samples_cover_the_face():
    _, areas, _ = face_samples({"shape": "box", "size_m": [0.12, 0.03, 0.01]})
    assert areas.sum() == pytest.approx(0.12 * 0.03)
    ring = {"shape": "ring", "outer_radius_m": 0.031, "inner_radius_m": 0.021, "thickness_m": 0.01}
    _, areas, _ = face_samples(ring)
    assert areas.sum() == pytest.approx(math.pi * (0.031**2 - 0.021**2))


def test_flat_pad_follows_the_supplier_curve_and_stores_its_work():
    pad = {"name": "pad", "shape": "box", "size_m": [0.12, 0.03, 0.01], "thickness_m": 0.01}
    m, d = _pad_model('<geom name="pad" type="box" size=".06 .015 .005"/>')
    face = FacePads(m, [pad], FOAM)
    forces, energies, depths = [], [], np.linspace(0, 0.004, 41)
    for depth in depths:
        d.qpos[2] = 0.005 - depth
        mujoco.mj_forward(m, d)
        face.update(m, d)
        forces.append(face.force["pad"])
        energies.append(face.energy["pad"])
    # 25 % strain on the whole face: the supplier 90 kPa point.
    assert forces[25] == pytest.approx(90e3 * 0.12 * 0.03, rel=1e-6)
    # Stored energy is the integral of the force over the compression.
    assert energies[-1] == pytest.approx(np.trapezoid(forces, depths), rel=0.01)
    assert face.peak_strain["pad"] == pytest.approx(0.4)


def test_tilted_pad_bears_on_an_edge_sliver():
    pad = {"name": "pad", "shape": "box", "size_m": [0.12, 0.03, 0.01], "thickness_m": 0.01}
    m, d = _pad_model('<geom name="pad" type="box" size=".06 .015 .005"/>')
    face = FacePads(m, [pad], FOAM)
    tilt = math.radians(12)
    d.qpos[3:7] = [math.cos(tilt / 2), 0, math.sin(tilt / 2), 0]
    # Lowest edge 1 mm into the ground.
    d.qpos[2] = 0.06 * math.sin(tilt) + 0.005 * math.cos(tilt) - 0.001
    mujoco.mj_forward(m, d)
    face.update(m, d)
    flat = float(foam_stress(FOAM, 0.1)) * 0.12 * 0.03
    assert 0 < face.force["pad"] < 0.1 * flat


def test_bevelled_pad_lands_on_its_facet():
    """Pitched to its bevel angle, the end facet bears on its whole area at once."""
    bevel = 12.5
    pad = {
        "name": "pad",
        "shape": "box",
        "size_m": [0.12, 0.03, 0.01],
        "thickness_m": 0.01,
        "end_bevel_deg": bevel,
        "flat_length_m": 0.06,
        "frame": ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
    }
    points, areas, thickness = face_samples(pad)
    end = points[:, 0] > 0.03 + 1e-9
    assert thickness[end].min() == pytest.approx(0.01 - 0.03 * math.tan(math.radians(bevel)))
    vertices = hull_vertices(pad)
    xml = f"""<mujoco><asset><mesh name="pad" vertex="{" ".join(map(str, vertices.ravel()))}"/>
    </asset><worldbody><geom name="ground" type="plane" size="1 1 .1"/>
    <body name="b" pos="0 0 0.1"><freejoint/><geom name="pad" type="mesh" mesh="pad"/>
    </body></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    face = FacePads(m, [pad], FOAM)
    tilt = math.radians(bevel)
    # Nose down by the bevel angle: the +x facet is parallel to the ground.
    d.qpos[3:7] = [math.cos(tilt / 2), 0, math.sin(tilt / 2), 0]
    rot = np.array(
        [[math.cos(tilt), 0, math.sin(tilt)], [0, 1, 0], [-math.sin(tilt), 0, math.cos(tilt)]]
    )
    lowest = (points @ rot.T)[:, 2].min()
    d.qpos[2] = -lowest - 0.001
    mujoco.mj_forward(m, d)
    face.update(m, d)
    facet = areas[end].sum()
    # The facet carries about 1 mm over its local thickness, not an edge sliver.
    assert face.force["pad"] > 0.6 * float(foam_stress(FOAM, 0.001 / thickness[end].mean())) * facet
    # The convex-hull geom touches the ground where the foam does.
    mujoco.mj_collision(m, d)
    assert d.ncon > 0 and all(d.contact[i].pos[0] > 0.02 for i in range(d.ncon))


def test_foam_energy_density_matches_the_stress_integral():
    strain = np.linspace(0, 0.6, 601)
    assert foam_energy_density(FOAM, 0.6) == pytest.approx(
        np.trapezoid(foam_stress(FOAM, strain), strain), rel=1e-3
    )


def test_tpu_ear_cantilever():
    ear = {"length": 0.04, "width": 0.03, "thickness": 0.007}
    stiffness, strain_per_m = ear_cantilever(ear, MATERIALS["tpu"])
    inertia = 0.03 * 0.007**3 / 12
    assert stiffness == pytest.approx(3 * 70e6 * inertia / 0.04**3)
    # 10 mm tip deflection bends the root to 3 t d / (2 L^2).
    assert strain_per_m * 0.01 == pytest.approx(3 * 0.007 * 0.01 / (2 * 0.04**2))


def test_shaped_plant_pads_sample_their_mesh_hulls():
    """The plant's shaped pads: the foam samples coincide with the compiled mesh hulls."""
    from bipe.m10.falls import candidate_config
    from bipe.m10.plant import build, parameters, reset

    shared = candidate_config(
        json.loads((ROOT / "config/robot.json").read_text()), "protection_ring10_flange"
    )
    b = shared["m10"]["physics"]["bumpers"]
    b["skid"].update(end_bevel_deg=12.5, flat_length_m=0.06)
    b["wheel_ring"]["dome_deg"] = 15.0
    p = parameters(shared)
    m, d = build(p)
    reset(m, d, p, 40)
    mujoco.mj_forward(m, d)
    from bipe.m10.plant import _bumper_pads

    face = FacePads(m, _bumper_pads(p), FOAM)
    for name, row in face.rows.items():
        g = row["geom"]
        mesh = m.geom_dataid[g]
        start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
        hull = d.geom_xpos[g] + m.mesh_vert[start : start + count] @ d.geom_xmat[g].reshape(3, 3).T
        samples = d.xpos[row["body"]] + row["points"] @ d.xmat[row["body"]].reshape(3, 3).T
        # Every foam sample is a vertex of the compiled mesh, at the same world point.
        gap = np.linalg.norm(samples[:, None, :] - hull[None, :, :], axis=2).min(axis=1)
        assert gap.max() < 1e-7, name
