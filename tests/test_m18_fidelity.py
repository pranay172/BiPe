import math

import numpy as np
import pytest

from bipe.m18 import fidelity
from bipe.parameters import load_parameters


@pytest.fixture(scope="module")
def p():
    return load_parameters(profile="m10")


def sensors():
    import json

    from bipe.parameters import ROOT

    return json.loads((ROOT / "config/sensors-m10.json").read_text())


def test_nominal_draw_changes_nothing(p):
    plant, config, policy = fidelity.apply(p, sensors(), {})
    q = dict(plant)
    q["physics"] = {k: v for k, v in plant["physics"].items() if k != "fidelity"}
    assert q == p and plant["physics"]["fidelity"] == {}
    assert config == sensors()
    assert policy == {"seed": 0}


def test_unknown_key_is_refused(p):
    with pytest.raises(KeyError):
        fidelity.apply(p, sensors(), {"cogging": 0.1})


def test_mount_matrix_tilts_gravity():
    g = np.array([0, 0, 9.81])
    pitched = fidelity.mount_matrix(0.0, 2.0) @ g
    assert math.degrees(math.atan2(-pitched[0], pitched[2])) == pytest.approx(2.0)
    rolled = fidelity.mount_matrix(2.0, 0.0) @ g
    assert math.degrees(math.atan2(rolled[1], rolled[2])) == pytest.approx(2.0)


def test_payload_moves_com_towards_head(p):
    plant, _, _ = fidelity.apply(p, sensors(), {"payload_kg": 0.1})
    before, after = p["rigid_bodies"]["chassis"], plant["rigid_bodies"]["chassis"]
    assert after["mass_kg"] == pytest.approx(before["mass_kg"] + 0.1)
    shift = np.subtract(after["com_m"], before["com_m"])
    head = np.subtract(fidelity.HEAD_POINT_M, before["com_m"])
    assert np.allclose(shift, 0.1 * head / after["mass_kg"])
    assert np.all(np.linalg.eigvalsh(after["inertia_kg_m2"]) > 0)


def test_winding_temperature_lowers_torque_constant(p):
    plant, _, _ = fidelity.apply(p, sensors(), {"winding_C": 80.0})
    k = plant["physics"]["hub_motor"]["torque_constant_Nm_A"] / p["physics"]["hub_motor"]["torque_constant_Nm_A"]
    assert k == pytest.approx(1 - 0.0012 * 55)


def test_radius_is_an_encoder_scale(p):
    _, config, policy = fidelity.apply(p, sensors(), {"wheel_radius_ratio": (0.97, 1.03)})
    assert config["wheel_encoder_scale"] == pytest.approx([1 / 0.97, 1 / 1.03])
    assert policy["wheel_encoder_scale"] == config["wheel_encoder_scale"]


def test_stiction_and_cogging_shapes():
    assert fidelity.stiction(np, np.array([0.0, 1.0]), 1.5) == pytest.approx([1.5, 1.0])
    assert fidelity.cogging(np, np.array([0.0]), 0.1, math.pi / 2) == pytest.approx([0.1])


def _plant(config):
    import mujoco

    from bipe.m10 import plant as m10
    from bipe.sensing import SensorMeasurements

    p = load_parameters(profile="m10")
    m, d = m10.build(p)
    m10.reset(m, d, p)
    mujoco.mj_forward(m, d)
    return SensorMeasurements(m, d, p, config, seed=1), d


def test_packet_loss_holds_the_last_packet():
    config = sensors()
    config["packet_loss"] = 1.0
    s, d = _plant(config)
    for _ in range(10):
        assert s.next_packet(d) is s.initial_packet


def test_jitter_delivers_in_order():
    config = sensors()
    config["transport_jitter_s"] = 0.01
    s, d = _plant(config)
    seen = []
    for _ in range(40):
        s.next_packet(d)
        if s.delivered is not None:
            seen.append(s.delivered[0])
    assert seen == sorted(seen) and seen[-1] >= 40 - 1 - 3
