import json
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT / "artifacts/m13/training/hard/model_2998.pt"


def test_actor_matches_the_trained_policy():
    pytest.importorskip("mjlab")
    from tensordict import TensorDict

    from bipe.m13.policy_control import Actor
    from bipe.m13.rl.env_cfg import env_cfg
    from bipe.m13.rl.replay import load_policy

    env, _, policy = load_policy(CHECKPOINT, env_cfg(play=True, num_envs=1), "cpu")
    actor = Actor(CHECKPOINT)
    rng = np.random.default_rng(0)
    for _ in range(5):
        obs = rng.normal(0, 1, 25)
        x = TensorDict({"actor": torch.tensor(obs, dtype=torch.float32)[None]}, batch_size=[1])
        with torch.no_grad():
            expected = policy(x)[0].numpy()
        assert np.allclose(actor(obs), expected, atol=1e-4)
    env.close()


def test_posture_label_inverts_the_references():
    from bipe import firmware_control as fc
    from bipe.m13.policy_control import PolicyController
    from bipe.parameters import load_parameters

    p = load_parameters(profile="m10")
    controller = PolicyController.__new__(PolicyController)
    from bipe.m13.policy_control import LABELS
    from bipe.m13.rl.firmware import posture_table

    controller.table = posture_table(p, LABELS)
    for label in (30.0, 37.5, 40.0, 52.25, 65.0):
        assert controller.label(controller.nominal(label)) == pytest.approx(label, abs=1e-6)
    assert fc.FirmwareController.__name__ == "FirmwareController"


def test_policy_runs_in_the_full_plant_and_restores_the_classical_classes():
    pytest.importorskip("mujoco")
    from bipe import firmware_control as fc
    from bipe import robustness
    from bipe.m13.policy_control import PolicyController, policy_in_loop
    from bipe.parameters import load_parameters
    from bipe.profiles import native
    from bipe.robustness_control import build_candidate

    p = load_parameters(profile="m10")
    profile = native(p)
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    gains = json.loads((ROOT / profile["controller"]).read_text())
    library = build_candidate(p, gains, ROOT / ".venv/m8-build/current")
    job = next(j for j in robustness.jobs(sensors, "c", p) if j["name"] == "sensors_hold")
    job = {**job, "duration": 0.2}
    case = {"name": "sensors_hold", "family": "acceptance", "job": job}
    classical = fc.FirmwareController, fc.FirmwareEstimator, robustness.simulate
    with policy_in_loop(CHECKPOINT):
        # Outside a simulation the classical classes stay in place.
        assert fc.FirmwareController is classical[0]
        run = robustness.run_case(p, gains, sensors, str(library), case)
    assert (fc.FirmwareController, fc.FirmwareEstimator, robustness.simulate) == classical
    assert run["failure"] is None
    obs = np.array(PolicyController.latest.observations)
    assert obs.shape == (11, 25)  # 0 to 0.2 s at 20 ms
    # The standing robot's accelerometer reads gravity in the IMU frame.
    assert 9.0 < np.linalg.norm(obs[0, 3:6]) < 10.6
