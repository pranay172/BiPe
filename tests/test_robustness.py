import copy
import json
import math

import mujoco
import numpy as np
import pytest

from bipe.articulated import truth
from bipe.disturbances import BacklashGear, build_disturbed, reset_on_plane
from bipe.kinematics import support_feedforward
from bipe.moving_experiment import simulate
from bipe.parameters import ROOT, load_parameters
from bipe.robustness import margins, matrix, plant_variant
from bipe.robustness_control import build_candidate, equivalence


@pytest.mark.parametrize(
    "block,field", [("motor", "peak_torque_per_wheel"), ("articulation", "peak_torque_per_joint")]
)
def test_parameter_load_rejects_inverted_ratings(tmp_path, block, field):
    p = load_parameters()
    p[block][field] = 0.1
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(p))
    with pytest.raises(ValueError, match="peak torque"):
        load_parameters(path)


def test_zero_and_near_zero_backlash_reproduce_free_contact_baseline():
    sensors = json.loads((ROOT / "config/sensors-m4.json").read_text())
    kw = {"duration": 0.2, "sensing": "sensors", "sensor_config": sensors, "implementation": "c"}
    baseline = simulate(**kw)
    for gap in [0, 1e-8]:
        run = simulate(**kw, disturbance={"backlash_deg": gap})
        assert run["qpos"] == pytest.approx(baseline["qpos"], abs=1e-12)
        assert run["trace"] == pytest.approx(baseline["trace"], abs=1e-12)


def test_backlash_free_flight_and_inelastic_coupled_engagement():
    p = load_parameters()
    m, d, active = build_disturbed(p, 0, 0.6)
    assert active and m.neq == 0 and m.nv == 14 and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    reset_on_plane(m, d, p, 0)
    gear = BacklashGear(m, 0.6)
    d.qvel[gear.rv] = [3, -2]
    before = d.qvel.copy()
    gear.engage(m, d)
    assert np.array_equal(before, d.qvel)  # inside the gap: rotor crosses freely
    d.qpos[gear.rq] = d.qpos[gear.wq] + [gear.half, -gear.half]
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    energy = 0.5 * before @ M @ before
    gear.engage(m, d)
    assert gear.J @ d.qvel == pytest.approx([0, 0], abs=1e-12)
    assert 0.5 * d.qvel @ M @ d.qvel < energy
    assert energy - 0.5 * d.qvel @ M @ d.qvel == pytest.approx(gear.dissipated_J, abs=1e-12)
    momentum = M @ (d.qvel - before)
    # Only opposing rotor/wheel face impulses: base/leg generalized momentum unchanged.
    other = np.ones(m.nv, bool)
    other[gear.wv] = False
    other[gear.rv] = False
    assert momentum[other] == pytest.approx(np.zeros(other.sum()), abs=1e-12)
    assert momentum[gear.wv] + momentum[gear.rv] == pytest.approx([0, 0], abs=1e-12)
    # A reversal separates from the face without a tensile impulse.
    gear.side[:] = 0
    d.qvel[:] = 0
    d.qvel[gear.rv] = [-1, 1]
    before = d.qvel.copy()
    gear.engage(m, d)
    assert np.array_equal(before, d.qvel)


def test_active_near_zero_gap_reproduces_rigid_gear_dynamics():
    p = load_parameters()
    m, d, _ = build_disturbed(p, 0, 0)
    bm, bd, _ = build_disturbed(p, 0, 0.00001)
    reset_on_plane(m, d, p, 0)
    reset_on_plane(bm, bd, p, 0)
    gear = BacklashGear(bm, 0.00001)
    for _ in range(200):
        for model, data in [(m, d), (bm, bd)]:
            mujoco.mj_forward(model, data)
            s = truth(model, data, p)
            data.ctrl[:4] = support_feedforward(p, s["q"], s["pitch"])
            data.ctrl[4:] = 0.1
        mujoco.mj_step(m, d)
        gear.step(bm, bd, np.array([0.1, 0.1]))
    mujoco.mj_forward(m, d)
    mujoco.mj_forward(bm, bd)
    a, b = truth(m, d, p), truth(bm, bd, p)
    assert abs(a["pitch"] - b["pitch"]) < math.radians(0.01)
    assert np.max(abs(a["position"] - b["position"])) < 2e-5
    assert abs(a["velocity"] - b["velocity"]) < 1e-4
    assert gear.engagements > 0 and gear.dissipated_J >= 0


def test_real_plane_and_body_only_mismatch():
    p = load_parameters()
    m, d, _ = build_disturbed(p, 10, 0)
    reset_on_plane(m, d, p, 10)
    n = np.array([-math.sin(math.radians(10)), 0, math.cos(math.radians(10))])
    assert d.geom("ground").xmat.reshape(3, 3)[:, 2] == pytest.approx(n)
    assert m.opt.gravity == pytest.approx([0, 0, -9.81])
    assert n @ d.site("left_axle").xpos == pytest.approx(p["geometry"]["wheel_radius"] - 0.0001)
    changed = plant_variant(p, {"body_mass_scale": 1.15, "body_com_x_m": 0.005})
    assert changed["rigid_bodies"]["chassis"]["mass_kg"] == pytest.approx(
        p["rigid_bodies"]["chassis"]["mass_kg"] * 1.15
    )
    assert changed["rigid_bodies"]["left_upper"] == p["rigid_bodies"]["left_upper"]
    assert changed["geometry"] == p["geometry"]
    assert p == load_parameters()


def test_fixed_matrix_and_candidate_C_python_equivalence(tmp_path):
    cases = matrix()
    assert len(cases) == 70 and len({c["name"] for c in cases}) == 70
    assert sum(c["family"] == "push" for c in cases) == 30
    p = load_parameters()
    for name in ["baseline", "cascade-retuned", "lqr"]:
        g = json.loads((ROOT / f"config/m8/{name}.json").read_text())
        nominal = copy.deepcopy(p)
        if name == "baseline":
            nominal["motor"]["peak_torque_per_wheel"] = 0.6
        library = build_candidate(nominal, g, tmp_path / name)
        assert equivalence(nominal, g, library, 300)["passed"]


def test_margin_does_not_hide_a_recovery_hole_or_call_survival_recovery():
    cases = {}
    for impulse, recovered in [(-0.5, True), (-1, False), (-1.5, True)]:
        name = f"push_hold_{impulse:+g}"
        cases[name] = {
            "case": {
                "name": name,
                "family": "push",
                "mode": "hold",
                "disturbance": {"push_Ns": impulse},
            },
            "metrics": {"physical_pass": True, "recovered": recovered},
        }
    result = margins(cases)
    assert result["push_hold_-1_Ns"] == 0.5
    assert result["largest_observed_passing_push_hold_-1_Ns"] == 1.5
    assert result["largest_survived_push_hold_-1_Ns"] == 1.5
    assert result["robustness_physical_passes"] == 3
    assert not result["targets"]["recover_2_Ns_both_directions"]


@pytest.mark.parametrize(
    "pitch_peak,speed_peak,recovered", [(2.01, 0.02, False), (1, 0.031, False), (2, 0.03, True)]
)
def test_m9_strict_recovery_and_cycle_use_unlogged_physics_extrema(
    pitch_peak, speed_peak, recovered
):
    from bipe.moving_experiment import COLUMNS
    from bipe.robustness import physical_metrics

    trace = np.zeros((4, len(COLUMNS)))
    trace[:, COLUMNS.index("time")] = [0, 18, 19, 20]
    trace[:, COLUMNS.index("velocity")] = 0.01
    trace[:, COLUMNS.index("pitch_error_deg")] = 0.4
    stats = {
        "peak_pitch_deg": pitch_peak,
        "max_joint_error_deg": 0,
        "peak_torque": np.zeros(6),
        "torque_squared": np.zeros(6),
        "push_impulse_Ns": 2,
        "positive_motor_work_J": 0,
        "negative_motor_work_J": 0,
        "peak_conditional_dc_input_A": 0,
        "peak_regen_mechanical_W": 0,
        "last_2s_peak_pitch_error_deg": pitch_peak,
        "last_2s_peak_speed_m_s": speed_peak,
        "last_4s_pitch_min_deg": -pitch_peak,
        "last_4s_pitch_max_deg": pitch_peak,
    }
    run = {
        "trace": trace,
        "duration_requested": 20,
        "failure": None,
        "warnings": {},
        "stats": stats,
        "parameters": {"profile": "m9"},
        "backlash": None,
    }
    result = physical_metrics(run)
    assert result["recovered"] == recovered
    assert result["late_pitch_peak_deg"] == pitch_peak
    assert result["late_speed_peak_m_s"] == speed_peak
    assert result["late_pitch_pp_deg"] == 2 * pitch_peak
    # M4 remains the inherited logged-sample yardstick.
    run["parameters"]["profile"] = "m4"
    historical = physical_metrics(run)
    assert historical["recovered"]
    assert historical["late_pitch_pp_deg"] == 0
    assert historical["late_speed_peak_m_s"] == 0.01


def test_m9_electrical_work_matches_actual_mujoco_motor_shaft_work(monkeypatch):
    from bipe.m9_plant import MechanicalDrives

    p = load_parameters(profile="m9")
    assumptions = json.loads((ROOT / "config/m9-screen-assumptions.json").read_text())
    observed_work = np.zeros(2)
    original = MechanicalDrives.integrate

    def measured(self, m, d, *args, **kwargs):
        motor_dofs = m.jnt_dofadr[m.actuator_trnid[:, 0]]
        shaft_power = d.actuator_force * m.actuator_gear[:, 0] * d.qvel[motor_dofs]
        observed_work[:] += m.opt.timestep * np.array(
            [np.maximum(shaft_power, 0).sum(), -np.minimum(shaft_power, 0).sum()]
        )
        original(self, m, d, *args, **kwargs)

    monkeypatch.setattr(MechanicalDrives, "integrate", measured)
    run = simulate("hold", duration=0.05, nominal=p, sensing="sensors")
    stats = run["stats"]
    assert observed_work[0] > 0
    assert stats["positive_motor_work_J"] == pytest.approx(observed_work[0], abs=1e-12)
    assert stats["negative_motor_work_J"] == pytest.approx(observed_work[1], abs=1e-12)
    copper_work = (
        0.75
        * assumptions["phase_to_phase_resistance_ohm"]
        * np.sum(stats["phase_current_squared_A2s"])
    )
    expected_dc_work = (
        observed_work[0] / assumptions["inverter_efficiency_assumption"]
        + copper_work
        + assumptions["auxiliary_power_W_assumption"] * 0.05
    )
    assert stats["estimated_dc_input_work_J"] == pytest.approx(expected_dc_work, abs=1e-12)


@pytest.mark.parametrize("path", [20, 40, 80])
def test_temperature_dependent_copper_mean_node_matches_independent_ODE(path):
    from scipy.integrate import solve_ivp

    from bipe.robustness import copper_mean_node_screen

    copper, regen = np.array([0.8, 1.6]), np.array([0.2, 0.05])
    air, capacity, duration = 40, 13.09, 180
    report = copper_mean_node_screen(copper, regen, duration, path, air, capacity)

    def heat_balance(_time, temperature):
        return (
            copper * (1 + 0.00393 * (temperature - 20)) + regen - (temperature - air) / path
        ) / capacity

    numerical = solve_ivp(heat_balance, [0, duration], [air, air], rtol=1e-11, atol=1e-11)
    assert report["cold_start_mean_winding_C_each"] == pytest.approx(numerical.y[:, -1], abs=1e-8)
    steady = np.asarray(report["repeat_duty_mean_winding_C_each"])
    assert heat_balance(0, steady) == pytest.approx([0, 0], abs=1e-12)
    assert report["thermal_runaway_each"] == [False, False]


def test_thermal_runaway_is_not_reported_as_a_finite_steady_temperature():
    from bipe.robustness import copper_mean_node_screen

    report = copper_mean_node_screen([10], [0], 20, 40, 40, 13.09)
    assert report["thermal_runaway_each"] == [True]
    assert report["repeat_duty_mean_winding_C_each"] == [None]
    from scipy.integrate import solve_ivp

    def heat_balance(_time, temperature):
        return (10 * (1 + 0.00393 * (temperature - 20)) - (temperature - 40) / 40) / 13.09

    numerical = solve_ivp(heat_balance, [0, 20], [40], rtol=1e-11, atol=1e-11)
    assert report["cold_start_mean_winding_C_each"] == pytest.approx(numerical.y[:, -1], abs=1e-8)
    json.dumps(report, allow_nan=False)


def test_zero_conductance_copper_node_has_finite_linear_transient():
    from scipy.integrate import solve_ivp

    from bipe.robustness import copper_mean_node_screen

    # Exact zero feedback conductance: 1/40 - 0.005*5 = 0.
    report = copper_mean_node_screen([5], [0.3], 20, 40, 40, 13.09, alpha_per_K=0.005)

    def heat_balance(_time, temperature):
        return (5 * (1 + 0.005 * (temperature - 20)) + 0.3 - (temperature - 40) / 40) / 13.09

    numerical = solve_ivp(heat_balance, [0, 20], [40], rtol=1e-11, atol=1e-11)
    assert report["cold_start_mean_winding_C_each"] == pytest.approx(numerical.y[:, -1], abs=1e-8)
    assert report["repeat_duty_mean_winding_C_each"] == [None]
    assert report["thermal_runaway_each"] == [True]
    json.dumps(report, allow_nan=False)


def test_rescreen_preserves_measurements_and_runtime_provenance(tmp_path):
    from bipe.robustness import rescreen_saved

    measurements = {
        "duration_s": 10,
        "phase_current_rms_A": [1] * 6,
        "phase_current_peak_A": [2] * 6,
        "positive_motor_work_J": 20,
        "negative_motor_work_each_J": [1] * 6,
        "peak_conditional_dc_input_A": 2,
        "physical_pass": True,
    }
    original = {
        "status": "complete",
        "nominal": {"profile": "m9"},
        "runtime_source_sha256": {"bipe/robustness.py": "original-runtime-hash"},
        "outcome": {"hardware_qualified": False},
        "controllers": {
            "current": {
                "nominal": {"joint_drive": {"stator_mass_kg_assumption": 0.034}},
                "cases": {"sensors_hold": {"metrics": measurements, "trace": [[0, 0], [10, 0]]}},
                "margins": {"preserved": True},
                "conditional_electrical": {"runtime": {"legacy": 42}},
            }
        },
    }
    source = tmp_path / "suite" / "summary.json"
    source.parent.mkdir()
    source.write_text(json.dumps(original))
    before = source.read_bytes()
    screened = rescreen_saved(source, tmp_path / "report")
    assert source.read_bytes() == before
    assert screened["runtime_source_sha256"] == original["runtime_source_sha256"]
    assert screened["outcome"] == original["outcome"]
    controller = screened["controllers"]["current"]
    assert controller["cases"] == original["controllers"]["current"]["cases"]
    assert controller["margins"] == {"preserved": True}
    assert controller["conditional_electrical"]["runtime"] == {"legacy": 42}
    corrected = controller["conditional_electrical"]["corrected"]
    assert corrected["hot_copper_resistance_at_design_limit_ohm"] == pytest.approx(2.401505)
    scenarios = corrected["runtime"]["sensors_hold"]["scenarios"]
    for row in scenarios:
        factor = 2 if row["current_convention"] == "RMS_current" else 1
        resistance_factor = 1 + 0.00393 * (row["copper_temperature_C"] - 20)
        power = (2 + factor * resistance_factor * 6 * 0.75 * 1.7) / row["inverter_efficiency"] + 6
        assert row["average_bus_W"] == pytest.approx(power)
        assert row["runtime_minutes"] == pytest.approx(60 * 15.096 / power)
    assert corrected["cold_peak_bus_current_A_upper_bound"]["peak_phase_A"] == pytest.approx(
        2 + 6 * 0.75 * 1.7 * 4 * (1 / 0.95 - 1) / 22.2
    )
    with pytest.raises(ValueError, match="differ"):
        rescreen_saved(source, source.parent)
    original["status"] = "running"
    source.write_text(json.dumps(original))
    with pytest.raises(ValueError, match="completed"):
        rescreen_saved(source, tmp_path / "unfinished-report")
