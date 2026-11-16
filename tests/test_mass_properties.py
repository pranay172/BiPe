import hashlib
import json

import mujoco
import numpy as np
import pytest

from bipe.articulated import JOINTS, build, reset
from bipe.kinematics import posture, support_feedforward
from bipe.mass_properties import combine
from bipe.parameters import ROOT, load_parameters, total_mass


def test_cad_mass_source_and_exact_mirror():
    p = load_parameters()
    source = ROOT / p["mass_properties_source"]["path"]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == p["mass_properties_source"]["sha256"]
    report = json.loads(source.read_text())
    shape_inputs = json.dumps(
        {"geometry": p["geometry"], "mechanics": p["mechanics"]}, sort_keys=True
    ).encode()
    assert hashlib.sha256(shape_inputs).hexdigest() == report["geometry_sha256"]
    reflection = np.diag([1, -1, 1])
    for role in ["upper", "lower", "wheel"]:
        left, right = [p["rigid_bodies"][side + "_" + role] for side in ["left", "right"]]
        assert left["mass_kg"] == pytest.approx(right["mass_kg"], abs=1e-12)
        assert reflection @ np.asarray(left["com_m"]) == pytest.approx(right["com_m"], abs=1e-12)
        assert reflection @ np.asarray(left["inertia_kg_m2"]) @ reflection == pytest.approx(
            np.asarray(right["inertia_kg_m2"]), abs=1e-12
        )


@pytest.mark.parametrize("angle", [25, 40, 60, 65])
def test_support_torque_matches_independent_physics_jacobians(angle):
    p = load_parameters()
    m, d = build(p)
    reset(m, d, p, angle)
    indices = [m.jnt_dofadr[m.joint(n).id] for n in JOINTS]
    expected = d.qfrc_bias[indices].copy()
    for side in ["left", "right"]:
        jac, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jac, jacr, m.site(side + "_axle").id)
        expected -= jac[:, indices].T @ np.array([0, 0, total_mass(p) * 9.81 / 2])
    rotation = d.xmat[m.body("chassis").id].reshape(3, 3)
    pitch = np.arcsin(-rotation[2, 0])
    assert support_feedforward(p, posture(angle), pitch) == pytest.approx(expected, abs=1e-9)


def test_parallel_axis_combination():
    rows = [
        {"mass_kg": 2, "com_m": [sign, 0, 0], "inertia_kg_m2": np.eye(3).tolist()}
        for sign in [-1, 1]
    ]
    combined = combine(rows)
    assert combined["mass_kg"] == 4
    assert combined["com_m"] == [0, 0, 0]
    assert np.asarray(combined["inertia_kg_m2"]) == pytest.approx(np.diag([2, 6, 6]))
