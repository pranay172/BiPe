"""M13 hardware handover: the firmware's learned-controller runtime (learned.c) against
the simulation's (bipe.m13.policy_control), its bench parity check, and the handover
to the classical controller in the full plant."""

import ctypes as c
import importlib.util
import json
import math
import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = ROOT / "firmware/include/generated_locomotion_weights.h"
PARITY = ROOT / "firmware/include/generated_locomotion_parity.h"
CHECKPOINT = ROOT / "artifacts/m13/training/settle/model_10493.pt"
LOW = np.radians([-195.0, -65.0, -195.0, -65.0])  # M10 hip and knee limits
HIGH = np.radians([-120.0, 2.0, -120.0, 2.0])


def generator():
    spec = importlib.util.spec_from_file_location(
        "gen", ROOT / "tools/generate_locomotion_config.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_committed_config_is_current():
    from bipe.parameters import load_parameters

    gen = generator()
    manifest = json.loads(gen.MANIFEST.read_text())
    expected = gen.header(manifest, load_parameters(profile="m10"))
    assert gen.HEADER.read_text() == expected
    assert manifest["identity"]["checkpoint_sha256"] in expected


def test_host_unit_tests_pass(tmp_path):
    out = subprocess.run(
        ["make", "-C", str(ROOT / "firmware"), "test"], check=False, capture_output=True, text=True
    )
    assert out.returncode == 0, out.stdout + out.stderr
    assert "learned: selection switch" in out.stdout


@pytest.mark.skipif(not PARITY.exists() or not WEIGHTS.exists(), reason="run bipe.m13.export")
def test_bench_parity_check_passes_on_the_host_and_fails_a_wrong_forward(tmp_path):
    lib = tmp_path / "libparity.so"
    subprocess.run(
        ["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-fPIC",
         "-shared", f"-I{ROOT / 'firmware/include'}",
         str(ROOT / "firmware/src/locomotion/parity.c"),
         str(ROOT / "firmware/src/locomotion/locomotion.c"), "-lm", "-o", str(lib)],
        check=True,
    )  # fmt: skip

    class Result(c.Structure):
        _fields_ = [("vectors", c.c_uint), ("max_error", c.c_double), ("min_cycles", c.c_uint32),
                    ("max_cycles", c.c_uint32), ("passed", c.c_bool)]  # fmt: skip

    dll = c.CDLL(str(lib))
    dll.bipe_locomotion_parity.restype = Result
    dll.bipe_locomotion_parity.argtypes = [c.c_void_p, c.c_void_p]
    forward = c.cast(dll.bipe_locomotion_forward, c.c_void_p)
    r = dll.bipe_locomotion_parity(forward, None)
    assert r.passed and r.vectors == 16 and r.max_error < 1e-5
    Forward = c.CFUNCTYPE(None, c.POINTER(c.c_float), c.POINTER(c.c_float))

    def off_by(observation, action):
        dll.bipe_locomotion_forward(observation, action)
        action[3] += 2e-4

    wrong = Forward(off_by)
    r = dll.bipe_locomotion_parity(c.cast(wrong, c.c_void_p), None)
    assert not r.passed and r.max_error > 1e-4


class Hub:
    """simulate()'s MjData, as far as PolicyController.wheel_torques reads it."""

    def __init__(self):
        self.speed = {"left_wheel_joint": 0.0, "right_wheel_joint": 0.0}

    def joint(self, name):
        return SimpleNamespace(qvel=[self.speed[name]])


def test_firmware_runtime_matches_the_simulation_runtime():
    """Same sensor packets, commands and actor: the same observations, leg targets,
    wheel targets and 1 kHz wheel torques as bipe.m13.policy_control."""
    pytest.importorskip("mjlab")
    from bipe.m13 import handover
    from bipe.m13.policy_control import PolicyController
    from bipe.parameters import load_parameters

    p = load_parameters(profile="m10")
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    rng = np.random.default_rng(3)
    weights = rng.normal(0, 0.3, (6, 25))
    actor = lambda obs: np.tanh(weights @ np.asarray(obs, float)) * 1.5
    sim = PolicyController(p, gains, actor=actor)
    lib = handover.build()
    fw = handover.RuntimeController(p, gains, actor=actor, runtime=lib)
    hub_sim, hub_fw = Hub(), Hub()
    qref = sim.nominal(40.0)
    clamped = 0
    # The leg target filter, one step a tick, from rest (both runtimes).
    alpha = 0.005 / (sim.leg_filter_tau + 0.005)
    out_sim, out_fw = sim.nominal(40.0), sim.nominal(40.0)
    for tick in range(240):
        if tick % 60 == 0:  # new command: speed (some beyond the trained range), yaw, posture
            velocity, yaw = rng.uniform(-0.8, 1.6), rng.uniform(-1.2, 1.2)
            qref = sim.nominal(rng.uniform(30, 65))
            velocity_fw = min(max(velocity, -0.5), 1.2)  # the firmware's trained-range clamp
            yaw_fw = velocity_fw, min(max(yaw, -1.0), 1.0)
        s = {
            "gyro_raw": rng.normal(0, 0.3, 3),
            "accel_raw": rng.normal([0, 0, 9.8], 0.5),
            "encoders_raw": np.r_[sim.nominal(sim.value[2]) + rng.normal(0, 0.05, 4), 0, 0],
            "qd": rng.normal(0, 1, 4),
            "wheel_speed": rng.normal(0, 5, 2),
            "pitch": 0.0,
            "yaw": 0.0,
        }
        a = sim.command(s, yaw_fw[0], yaw_fw[1], qref, np.zeros(4), 0.005)
        b = fw.command(s, velocity, yaw, qref, np.zeros(4), 0.005)
        assert a[2] == pytest.approx(b[2], abs=1e-12)
        assert np.allclose(sim.value, fw.value, atol=1e-9)
        # The firmware's guards: leg targets clamped to the joint limits.
        assert np.allclose(np.clip(sim.leg_target, LOW, HIGH), fw.leg_target, atol=1e-6)
        clamped += not np.allclose(sim.leg_target, fw.leg_target, atol=1e-6)
        assert np.allclose(sim.wheel_target, fw.wheel_target, atol=1e-6)
        band = sim.leg_filter_band
        out_sim = np.clip(
            out_sim + alpha * (sim.leg_target - out_sim), *(sim.leg_target + [[-band], [band]])
        )
        out_fw = np.clip(
            out_fw + alpha * (fw.leg_target - out_fw), *(fw.leg_target + [[-band], [band]])
        )
        assert np.allclose(sim.leg_output, out_sim, atol=1e-12)
        assert np.allclose(fw.leg_output, out_fw, atol=1e-9)
        for _ in range(5):
            speeds = rng.normal(fw.wheel_target, 1.0)
            hub_sim.speed = hub_fw.speed = dict(zip(hub_sim.speed, speeds))
            expected = np.clip(sim.wheel_torques(hub_sim, 0.001), -0.495, 0.495)  # hub torque
            assert np.allclose(expected, fw.wheel_torques(hub_fw, 0.001), atol=1e-6)
    assert len(sim.observations) == 60
    # The firmware's observation is the float32 of the simulation's.
    assert np.allclose(np.array(fw.learned.observation[:]), sim.observations[-1], atol=1e-6)
    assert fw.learned.leg_clamps > 0 and clamped > 0 and fw.handover_s is None


def test_handover_to_the_classical_controller_in_the_full_plant():
    """A NaN action mid-run: the firmware hands over to a fresh classical controller and
    the robot keeps balancing."""
    pytest.importorskip("mjlab")
    if not WEIGHTS.exists():
        pytest.skip("run bipe.m13.export")
    from bipe import robustness
    from bipe.m13 import handover
    from bipe.m13.policy_control import CActor, PolicyController
    from bipe.parameters import load_parameters
    from bipe.profiles import native
    from bipe.robustness_control import build_candidate

    subprocess.run(["make", "-C", str(ROOT / "firmware"), "locomotion"], check=True)
    p = load_parameters(profile="m10")
    profile = native(p)
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    gains = json.loads((ROOT / profile["controller"]).read_text())
    library = build_candidate(p, gains, ROOT / ".venv/m8-build/current")
    job = next(j for j in robustness.jobs(sensors, "c", p) if j["name"] == "sensors_hold")
    case = {"name": "sensors_hold", "family": "acceptance", "job": {**job, "duration": 4.0}}
    original = handover.runtime_in_loop(
        robustness, CActor(), **handover.VARIANTS["fault"], fault_s=1.0
    )
    try:
        run = robustness.run_case(p, gains, sensors, str(library), case)
    finally:
        robustness.simulate = original
    assert run["failure"] is None
    assert run["handover_s"] == pytest.approx(1.0, abs=0.025)
    assert 45 <= run["policy_steps"] <= 55  # policy steps stop at the handover
    latest = PolicyController.latest
    assert latest.classical.state.hold_initialized  # the classical controller ran
    # The handover transient (about 4 degrees) has settled two seconds later.
    assert math.isfinite(run["stats"]["peak_pitch_deg"])
    assert run["stats"]["last_2s_peak_pitch_error_deg"] < 1.0
