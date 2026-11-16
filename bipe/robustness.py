"""Lean permanent robustness yardstick, C control with sensors-m4 and free contact.

Run: .venv/bin/python -m bipe.robustness --output artifacts/milestone-8 --workers 6
The summary retains configs, all failures and decimated measured traces. No seals.
"""

import argparse
import copy
import csv
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from .kinematics import posture
from .moving_experiment import COLUMNS, commands, metrics, simulate
from .moving_validate import jobs
from .parameters import ROOT, load_parameters
from .profiles import CHOICES, native
from .robustness_control import build_candidate, equivalence

# Native CAD plants are held to the stricter M9 slope rollback limit.
NATIVE_SLOPE_ROLLBACK_LIMIT_M = 0.1


def low_posture(nominal=None):
    """Lowest operating posture command; a profile may set it (M10: 72 mm hip height,
    user decision 13 October 2026), otherwise the inherited 65."""
    entry = native(nominal)
    return entry.get("low_posture_deg", 65) if entry else 65


def matrix(nominal=None):
    cases = []
    for mode in ["hold", "drive", "low"]:
        for impulse in [0.5, 1, 1.5, 2, 2.5]:
            for sign in [-1, 1]:
                cases.append(
                    {
                        "name": f"push_{mode}_{sign * impulse:+g}",
                        "family": "push",
                        "mode": mode,
                        "duration": 16,
                        "disturbance": {
                            "push_Ns": sign * impulse,
                            "initial_posture_deg": low_posture(nominal) if mode == "low" else 40,
                        },
                    }
                )
    for slope in [0, 4, 8, 10]:
        for mode in ["hold", "up", "down"]:
            cases.append(
                {
                    "name": f"slope_{mode}_{slope}",
                    "family": "slope",
                    "mode": mode,
                    "duration": 20,
                    "disturbance": {"slope_deg": slope},
                }
            )
    for gap in [0, 0.3, 0.6]:
        for mode in ["hold", "drive", "turn", "crouch"]:
            cases.append(
                {
                    "name": f"backlash_{mode}_{gap:g}",
                    "family": "backlash",
                    "mode": mode,
                    "duration": 36 if mode == "crouch" else 20,
                    "disturbance": {"backlash_deg": gap},
                }
            )
    for kind, values in [
        ("body_mass_scale", [0.85, 1.15]),
        ("body_com_x_m", [-0.005, 0.005]),
        ("body_com_z_m", [-0.003, 0.003]),
        ("friction", [0.5, 1.0]),
    ]:
        for value in values:
            for mode in ["hold", "combined"]:
                cases.append(
                    {
                        "name": f"mismatch_{kind}_{value:g}_{mode}",
                        "family": "mismatch",
                        "mode": mode,
                        "duration": 36 if mode == "combined" else 20,
                        "variation": {kind: value},
                    }
                )
    if nominal is not None and nominal.get("profile") == "m10":
        b = nominal["physics"]["belts"]
        # Hip and knee belt corners: lash range by Gates tensile-stiffness bounds.
        for hip_gap in b["hip_lash_range_deg"]:
            for knee_gap in b["knee_lash_range_deg"]:
                for index in [0, 1]:
                    cases.append(
                        {
                            "name": f"belt_hold_h{hip_gap:g}_n{knee_gap:g}_k{index}",
                            "family": "belt",
                            "mode": "hold",
                            "duration": 20,
                            "disturbance": {
                                "hip_backlash_deg": hip_gap,
                                "knee_backlash_deg": knee_gap,
                                "hip_belt_stiffness_Nm_rad": b["hip_stiffness_range_Nm_rad"][index],
                                "knee_belt_stiffness_Nm_rad": b["knee_stiffness_range_Nm_rad"][
                                    index
                                ],
                            },
                        }
                    )
    if nominal is not None and nominal.get("profile") == "m9":
        b = nominal["physics"]["belts"]
        # Corners include both catalog lash and Gates tensile-stiffness bounds.
        # Keep the inherited seventy cases and report these separately.
        for hip_gap in b["hip_lash_range_deg"]:
            for wheel_gap in b["wheel_lash_range_deg"]:
                for index in [0, 1]:
                    cases.append(
                        {
                            "name": f"belt_hold_h{hip_gap:g}_w{wheel_gap:g}_k{index}",
                            "family": "belt",
                            "mode": "hold",
                            "duration": 20,
                            "disturbance": {
                                "hip_backlash_deg": hip_gap,
                                "backlash_deg": wheel_gap,
                                "hip_belt_stiffness_Nm_rad": b["hip_stiffness_range_Nm_rad"][index],
                                "wheel_belt_stiffness_Nm_rad": b["wheel_stiffness_range_Nm_rad"][
                                    index
                                ],
                            },
                        }
                    )
    return cases


def plant_variant(nominal, variation):
    p = copy.deepcopy(nominal)
    body = p["rigid_bodies"]["chassis"]
    if "body_mass_scale" in variation:
        scale = variation["body_mass_scale"]
        body["mass_kg"] *= scale
        body["inertia_kg_m2"] = (np.array(body["inertia_kg_m2"]) * scale).tolist()
        for row in p["mass_budget"]:
            if row["location"] == "body":
                row["mass_each"] *= scale
    for key, axis in [("body_com_x_m", 0), ("body_com_z_m", 2)]:
        if key in variation:
            body["com_m"][axis] += variation[key]
    if "friction" in variation:
        p["contact"]["sliding_friction"] = variation["friction"]
    return p


def command(t, scenario, case, nominal=None):
    mode = case["mode"]
    if case["family"] == "push":
        v = 0.3 if mode == "drive" and 2 <= t < 10 else 0
        return v, 0, posture(low_posture(nominal) if mode == "low" else 40, nominal), np.zeros(4)
    if case["family"] == "slope":
        v = (0.3 if mode == "up" else -0.3) if mode != "hold" and 2 <= t < 12 else 0
        return v, 0, posture(40, nominal), np.zeros(4)
    return commands(t, "hold" if mode == "low" else mode, nominal)


def physical_metrics(run):
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    t, pitch = f["time"], f["pitch_error_deg"]
    complete = t[-1] >= run["duration_requested"] - 0.02
    physical = complete and not run["failure"] and not run["warnings"]
    last = t >= run["duration_requested"] - 2
    late = t >= run["duration_requested"] - 4
    recovered = (
        physical
        and last.any()
        and float(abs(pitch[last]).max()) <= 2
        and float(abs(f["velocity"][last]).max()) <= 0.03
    )
    late_pitch_pp = float(np.ptp(pitch[late])) if late.any() else None
    late_speed_peak = float(abs(f["velocity"][last]).max()) if last.any() else None
    if native(run["parameters"]):
        strict_pitch = run["stats"]["last_2s_peak_pitch_error_deg"]
        late_speed_peak = run["stats"]["last_2s_peak_speed_m_s"]
        low, high = (
            run["stats"]["last_4s_pitch_min_deg"],
            run["stats"]["last_4s_pitch_max_deg"],
        )
        late_pitch_pp = high - low if low is not None and high is not None else None
        recovered = (
            physical
            and strict_pitch is not None
            and strict_pitch <= 2
            and late_speed_peak is not None
            and late_speed_peak <= 0.03
        )
    result = {
        "physical_pass": bool(physical),
        "recovered": bool(recovered),
        "duration_s": float(t[-1]),
        "failure": run["failure"],
        "warnings": run["warnings"],
        "peak_pitch_deg": run["stats"]["peak_pitch_deg"],
        "late_pitch_pp_deg": late_pitch_pp,
        "max_travel_m": float(
            np.linalg.norm(run["trace"][:, 1:3] - run["trace"][0, 1:3], axis=1).max()
        ),
        "rollback_m": float(max(0, -(f["x"] - f["x"][0]).min())),
        "late_speed_peak_m_s": late_speed_peak,
        "max_joint_error_deg": run["stats"]["max_joint_error_deg"],
        "peak_motor_Nm": float(run["stats"]["peak_torque"].max()),
        "peak_wheel_Nm": float(run["stats"]["peak_torque"][4:].max()),
        "rms_wheel_Nm": float(
            np.sqrt(run["stats"]["torque_squared"][4:] / max(t[-1], 0.001)).max()
        ),
        **{
            k: run["stats"][k]
            for k in [
                "push_impulse_Ns",
                "positive_motor_work_J",
                "negative_motor_work_J",
                "peak_conditional_dc_input_A",
                "peak_regen_mechanical_W",
            ]
        },
        "backlash": run["backlash"],
    }
    if native(run["parameters"]):
        duration = max(float(t[-1]), 0.001)
        result["late_pitch_peak_deg"] = run["stats"]["last_2s_peak_pitch_error_deg"]
        result["phase_current_peak_A"] = (
            run["stats"].get("phase_current_peak_A", np.zeros(6)).tolist()
        )
        result["phase_current_rms_A"] = np.sqrt(
            run["stats"].get("phase_current_squared_A2s", np.zeros(6)) / duration
        ).tolist()
        result["estimated_dc_input_work_J"] = run["stats"].get("estimated_dc_input_work_J", 0.0)
        result["negative_motor_work_each_J"] = (
            run["stats"].get("negative_motor_work_each_J", np.zeros(6)).tolist()
        )
        result["peak_net_regen_estimate_W"] = run["stats"].get("peak_net_regen_estimate_W", 0.0)
        result["net_regen_estimate_J"] = run["stats"].get("net_regen_estimate_J", 0.0)
    return result


def run_case(nominal, gains, sensors, library, case, observer=None):
    """Simulate one matrix or acceptance case, as the acceptance runs it."""
    if case["family"] == "acceptance":
        kw = copy.deepcopy(case["job"])
        kw.pop("name", None)
        # Old acceptance variations are relative to the *nominal candidate*.
        if "plant" in kw:
            kw["plant"]["motor"]["peak_torque_per_wheel"] = nominal["motor"][
                "peak_torque_per_wheel"
            ]
        kw.update(nominal=nominal, gains=gains, firmware_library=library, observer=observer)
        return simulate(**kw)
    from functools import partial

    return simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=nominal,
        plant=plant_variant(nominal, case.get("variation", {})),
        gains=gains,
        sensor_config=sensors,
        implementation="c",
        firmware_library=library,
        physics_dt=case.get("physics_dt"),
        disturbance=case.get("disturbance", {}),
        command_function=partial(command, case=case, nominal=nominal),
        observer=observer,
    )


def execute(payload):
    name, nominal, gains, sensors, library, case = payload
    run = run_case(nominal, gains, sensors, library, case)
    if case["family"] == "acceptance":
        result = metrics(run)
        result["physical_metrics"] = physical_metrics(run)
    else:
        result = physical_metrics(run)
    # 10 Hz traces, with terminal sample retained; physical checks run at 1 kHz.
    selected = [
        "time",
        "x",
        "y",
        "velocity",
        "pitch_error_deg",
        "user_velocity",
        "yaw_rate",
        "left_applied_Nm",
        "right_applied_Nm",
        "forbidden_contact",
    ]
    indices = [COLUMNS.index(s) for s in selected]
    trace = run["trace"][::10, indices]
    if trace[-1, 0] != run["trace"][-1, 0]:
        trace = np.vstack([trace, run["trace"][-1, indices]])
    return (
        name,
        case["name"],
        {"case": case, "metrics": result, "trace": np.round(trace, 7).tolist()},
    )


def convergence(base, fine):
    from .articulated import JOINTS
    from .locomotion import wrap

    r = {
        "half_timestep_passed": fine["passed"],
        "pitch_deg": abs(fine["max_pitch_error_deg"] - base["max_pitch_error_deg"]),
        "heading_deg": abs(
            math.degrees(wrap(math.radians(fine["final_yaw_deg"] - base["final_yaw_deg"])))
        ),
        "height_m": abs(fine["height_range_m"] - base["height_range_m"]),
        "joint_peak_Nm": max(
            abs(fine["peak_torque_Nm"][n] - base["peak_torque_Nm"][n]) for n in JOINTS
        ),
        "joint_rms_Nm": max(
            abs(fine["rms_torque_Nm"][n] - base["rms_torque_Nm"][n]) for n in JOINTS
        ),
        "joint_speed_rad_s": max(
            abs(fine["peak_joint_speed_rad_s"][n] - base["peak_joint_speed_rad_s"][n])
            for n in JOINTS
        ),
    }
    r["passed"] = bool(
        fine["passed"]
        and r["pitch_deg"] < 1
        and r["heading_deg"] < 2
        and r["height_m"] < 0.003
        and r["joint_peak_Nm"] < 0.18
        and r["joint_rms_Nm"] < 0.05
        and r["joint_speed_rad_s"] < 0.3
    )
    return r


def margins(cases, slope_rollback_limit=0.3):
    result = {}
    for mode in ["hold", "drive", "low"]:
        # A margin is contiguous from the smallest pulse; holes are retained.
        for sign in [-1, 1]:
            values = sorted(
                (abs(c["case"]["disturbance"]["push_Ns"]), c["metrics"]["recovered"])
                for c in cases.values()
                if c["case"]["family"] == "push"
                and c["case"]["mode"] == mode
                and np.sign(c["case"]["disturbance"]["push_Ns"]) == sign
            )
            margin = 0.0
            for v, passed in values:
                if not passed:
                    break
                margin = v
            result[f"push_{mode}_{sign:+d}_Ns"] = margin
            result[f"largest_observed_passing_push_{mode}_{sign:+d}_Ns"] = max(
                (v for v, passed in values if passed), default=0.0
            )
            result[f"largest_survived_push_{mode}_{sign:+d}_Ns"] = max(
                (
                    abs(c["case"]["disturbance"]["push_Ns"])
                    for c in cases.values()
                    if c["case"]["family"] == "push"
                    and c["case"]["mode"] == mode
                    and c["case"]["disturbance"]["push_Ns"] * sign > 0
                    and c["metrics"]["physical_pass"]
                ),
                default=0.0,
            )
    for mode in ["hold", "up", "down"]:
        values = sorted(
            (c["case"]["disturbance"]["slope_deg"], c["metrics"]["physical_pass"])
            for c in cases.values()
            if c["case"]["family"] == "slope" and c["case"]["mode"] == mode
        )
        margin = None
        for v, passed in values:
            if not passed:
                break
            margin = v
        result[f"stable_slope_{mode}_deg"] = margin
    for gap in [0, 0.3, 0.6]:
        c = cases.get(f"backlash_hold_{gap:g}")
        result[f"backlash_hold_{gap:g}_pp_deg"] = c["metrics"]["late_pitch_pp_deg"] if c else None
    c = cases.get("slope_hold_10")
    result["slope_hold_10_rollback_m"] = c["metrics"]["rollback_m"] if c else None
    result["robustness_physical_passes"] = sum(
        c["metrics"]["physical_pass"]
        for c in cases.values()
        if c["case"]["family"] not in ["acceptance", "numerical", "belt"]
    )
    result["robustness_cases"] = sum(
        c["case"]["family"] not in ["acceptance", "numerical", "belt"] for c in cases.values()
    )
    slope = cases.get("slope_hold_10")
    gap = cases.get("backlash_hold_0.6")
    result["targets"] = {
        "survive_2_Ns_both_directions": all(
            cases.get(f"push_hold_{v:+g}", {}).get("metrics", {}).get("physical_pass", False)
            for v in [-2, 2]
        ),
        "recover_2_Ns_both_directions": all(
            cases.get(f"push_hold_{v:+g}", {}).get("metrics", {}).get("recovered", False)
            for v in [-2, 2]
        ),
        "slope_10_rollback_at_most_0p3_m": bool(
            slope and slope["metrics"]["physical_pass"] and slope["metrics"]["rollback_m"] <= 0.3
        ),
        "slope_10_rollback_within_limit": bool(
            slope
            and slope["metrics"]["physical_pass"]
            and slope["metrics"]["rollback_m"] <= slope_rollback_limit
        ),
        "backlash_0p6_pp_at_most_1p5_deg": bool(
            gap and gap["metrics"]["physical_pass"] and gap["metrics"]["late_pitch_pp_deg"] <= 1.5
        ),
    }
    result["targets"]["all_three_backlash_cycles_at_most_1p5_deg"] = all(
        cases.get(f"backlash_hold_{gap:g}", {}).get("metrics", {}).get("physical_pass", False)
        and cases[f"backlash_hold_{gap:g}"]["metrics"]["late_pitch_pp_deg"] <= 1.5
        for gap in [0, 0.3, 0.6]
    )
    slope_rows = [c["metrics"] for c in cases.values() if c["case"]["family"] == "slope"]
    if slope_rows:
        result["targets"]["all_slopes_through_10_survive"] = bool(
            len(slope_rows) == 12 and all(c["physical_pass"] for c in slope_rows)
        )
        holds = [
            c["metrics"]["rollback_m"]
            for c in cases.values()
            if c["case"]["family"] == "slope" and c["case"]["mode"] == "hold"
        ]
        result["slope_hold_max_rollback_m"] = max(holds, default=0)
        result["targets"]["all_slope_hold_rollbacks_within_limit"] = bool(
            len(holds) == 4 and max(holds) <= slope_rollback_limit
        )
    belts = [c["metrics"] for c in cases.values() if c["case"]["family"] == "belt"]
    if belts:
        result["belt_range_max_pitch_pp_deg"] = max(
            (c["late_pitch_pp_deg"] for c in belts if c["late_pitch_pp_deg"] is not None),
            default=None,
        )
        result["targets"]["belt_range_cycles_at_most_1p5_deg"] = bool(
            len(belts) == 8
            and all(c["physical_pass"] and c["late_pitch_pp_deg"] <= 1.5 for c in belts)
        )
    accept = [
        c
        for c in cases.values()
        if c["case"]["family"] == "acceptance" and c["case"]["name"] != "combined_half_timestep"
    ]
    result["inherited_passes"] = sum(c["metrics"]["passed"] for c in accept)
    result["inherited_cases"] = len(accept)
    result["inherited_failures"] = [c["case"]["name"] for c in accept if not c["metrics"]["passed"]]
    if "sensors_combined" in cases and "combined_half_timestep" in cases:
        result["convergence"] = convergence(
            cases["sensors_combined"]["metrics"], cases["combined_half_timestep"]["metrics"]
        )
        result["targets"]["all_27_and_convergence"] = bool(
            len(accept) == 27
            and result["inherited_passes"] == 27
            and result["convergence"]["passed"]
        )
    if "backlash_hold_0.6_half_timestep" in cases and "backlash_hold_0.6" in cases:
        a = cases["backlash_hold_0.6"]["metrics"]
        b = cases["backlash_hold_0.6_half_timestep"]["metrics"]
        result["backlash_half_timestep"] = {
            "physical_pass": b["physical_pass"],
            "pitch_pp_difference_deg": abs(a["late_pitch_pp_deg"] - b["late_pitch_pp_deg"])
            if a["late_pitch_pp_deg"] is not None and b["late_pitch_pp_deg"] is not None
            else None,
        }
    return result


def comparison_outcome(record):
    for controller in record["controllers"].values():
        m9 = native(controller["nominal"]) is not None
        controller["margins"] = margins(
            controller["cases"], NATIVE_SLOPE_ROLLBACK_LIMIT_M if m9 else 0.3
        )
        rows = {
            name: row["metrics"].get("physical_metrics", row["metrics"])
            for name, row in controller["cases"].items()
        }
        controller["conditional_electrical"] = {
            "peak_dc_input_A": max(r["peak_conditional_dc_input_A"] for r in rows.values()),
            "max_wheel_rms_Nm": max(r["rms_wheel_Nm"] for r in rows.values()),
            "peak_negative_mechanical_W": max(r["peak_regen_mechanical_W"] for r in rows.values()),
            "max_gross_negative_motor_work_J": max(
                r["negative_motor_work_J"] for r in rows.values()
            ),
            "cases_over_6p5_A": [
                name for name, r in rows.items() if r["peak_conditional_dc_input_A"] > 6.5
            ],
            "basis": "M4 conditional efficiency 0.6, output Kt=0.056*10*0.6, 1.5*0.978*Iq^2 copper, 6 W auxiliaries, 19.2 V bus; gross negative work is an upper bound, not measured dump energy",
        }
        if m9:
            controller["conditional_electrical"]["basis"] = (
                "RO40 catalog rated/peak torque-current piecewise interpolation; actual motor-shaft torque times rotor speed, peak-phase balanced-winding copper 0.75*Rline-line*Iq², separate 95% inverter efficiency engineering assumption, 6W auxiliaries, 22.2V. Reduction loss is already physical in the plant and is not applied again to motor electrical power. RMS interpretation doubles copper. No M4 bus limit transfers."
            )
            if controller["nominal"].get("profile") == "m10":
                hub = controller["nominal"]["physics"]["hub_motor"]
                controller["conditional_electrical"]["basis"] += (
                    " M10: the wheels are GL40 KV70 direct-drive hub motors (4.5 ohm line-line) whose"
                    f" motoring current is limited by the {hub['drive_voltage_limit_V']:g} V"
                    " drive-voltage setting in the plant."
                )
            controller["conditional_electrical"].pop("cases_over_6p5_A")
            controller["conditional_electrical"].update(
                m9_electrical_screen(rows, controller["nominal"])
            )
    if native(record["nominal"]):
        required = [
            "all_27_and_convergence",
            "survive_2_Ns_both_directions",
            "recover_2_Ns_both_directions",
            "all_slopes_through_10_survive",
            "all_slope_hold_rollbacks_within_limit",
            "all_three_backlash_cycles_at_most_1p5_deg",
            "belt_range_cycles_at_most_1p5_deg",
        ]
        eligible = [
            name
            for name, row in record["controllers"].items()
            if all(row["margins"]["targets"].get(key, False) for key in required)
        ]
        return {
            "eligible_configurations": eligible,
            "recommended_default": eligible[0] if eligible else None,
            "criterion": "All absolute M9 locomotion targets (also for M10); historical M8 is information only",
            "required_targets": required,
            "recovery_note": "Largest observed recovery can have non-monotonic holes; contiguous margins and all case-level failures remain recorded",
            "hardware_qualified": False,
        }
    baseline = record["controllers"].get("baseline", {}).get("margins", {})
    eligible = []
    for name in ["cascade-retuned", "lqr", "cascade-corrected"]:
        if name not in record["controllers"]:
            continue
        row = record["controllers"][name]
        m = row["margins"]
        if (
            m["targets"].get("all_27_and_convergence", False)
            and baseline
            and m["robustness_physical_passes"] > baseline["robustness_physical_passes"]
            and all(
                m[f"largest_survived_push_hold_{s:+d}_Ns"]
                >= baseline[f"largest_survived_push_hold_{s:+d}_Ns"]
                for s in [-1, 1]
            )
            and not row["conditional_electrical"]["cases_over_6p5_A"]
        ):
            eligible.append(name)
    return {
        "eligible_configurations": eligible,
        "recommended_default": eligible[0] if eligible else None,
        "criterion": "All 27 plus unchanged convergence, more physical robustness cases than historical baseline, no lost nominal hold survival margin, conditional bus-current domain respected",
        "recovery_note": "Largest observed recovery can have non-monotonic holes; contiguous margins and all case-level failures remain recorded",
        "hardware_qualified": False,
    }


def copper_mean_node_screen(
    copper_ref_W,
    other_heat_W,
    duration_s,
    path_K_W,
    air_C,
    capacity_J_K,
    alpha_per_K=0.00393,
    reference_C=20,
):
    """Mean-node RC screen with R(T), not a qualified winding hot-spot model."""
    copper = np.asarray(copper_ref_W, dtype=float)
    heat = np.asarray(other_heat_W, dtype=float)
    conductance = 1 / path_K_W - alpha_per_K * copper
    stable = conductance > 0
    heating_at_air = copper * (1 + alpha_per_K * (air_C - reference_C)) + heat
    rise = np.divide(heating_at_air, conductance, out=np.full_like(copper, np.nan), where=stable)
    transient_factor = np.divide(
        -np.expm1(-duration_s * conductance / capacity_J_K),
        conductance,
        out=np.full_like(copper, duration_s / capacity_J_K),
        where=conductance != 0,
    )
    cold = air_C + heating_at_air * transient_factor
    nullable = lambda values: [float(v) if np.isfinite(v) else None for v in values]
    return {
        "path_K_W": path_K_W,
        "cold_start_mean_winding_C_each": nullable(cold),
        "repeat_duty_mean_winding_C_each": nullable(air_C + rise),
        "thermal_runaway_each": (~stable).tolist(),
    }


def m9_corrected_electrical_screen(rows, assumptions, capacity):
    """Supplemental engineering scenarios from saved case integrals/extrema."""
    a = copy.deepcopy(assumptions)
    a.update(
        {
            "resistance_reference_C_assumption": 20,
            "copper_alpha20_per_K": 0.00393,
            "inverter_efficiency_scenarios": [0.85, 0.95],
            "copper_temperature_scenarios_C": [20, 125],
            "catalog_ambient_limit_C": 50,
            "interpretation": "Fixed-R baseline fields remain legacy cold diagnostics. The reference temperature is unpublished; 20C is an engineering assumption. All-stator RC capacity and case-average heat describe a mean node, not a short-burst hot spot. Local housing air and cooling paths require measurement; no hardware thermal pass.",
        }
    )
    a["sources"].update(
        {
            "copper_temperature": "https://nvlpubs.nist.gov/nistpubs/Legacy/hb/nbshandbook100.pdf",
            "moteus_efficiency": "https://blog.mjbots.com/2024/07/25/electrical-power-reporting-with-moteus/",
        }
    )
    resistance, air, limit = (
        a[k] for k in ["phase_to_phase_resistance_ohm", "housing_air_C", "winding_design_limit_C"]
    )
    resistance = np.asarray(resistance, dtype=float)
    hot_factor = 1 + a["copper_alpha20_per_K"] * (limit - 20)
    usable_Wh = a["pack_capacity_Ah"] * a["pack_nominal_V"] * a["usable_capacity_fraction"]
    thermal, runtime, heating, bounds = [], {}, {"peak_phase": [], "RMS_current": []}, []
    for name, row in rows.items():
        duration = max(row["duration_s"], 0.001)
        copper = 0.75 * resistance * np.asarray(row["phase_current_rms_A"]) ** 2
        regen = np.asarray(row["negative_motor_work_each_J"]) / duration
        entry = {"case": name, "duration_s": duration}
        for convention, factor in [("peak_phase", 1), ("RMS_current", 2)]:
            entry[convention] = [
                copper_mean_node_screen(factor * copper, regen, duration, path, air, capacity)
                for path in a["winding_to_housing_air_K_W_sensitivity"]
            ]
            if name != "unpowered":
                heating[convention].append(factor * hot_factor * copper + regen)
        thermal.append(entry)
        peak_copper = float(
            (0.75 * resistance * np.asarray(row["phase_current_peak_A"]) ** 2).sum()
        )
        legacy = row["peak_conditional_dc_input_A"]
        bounds.append(
            {
                "case": name,
                "peak_phase_A": legacy + peak_copper * (1 / 0.95 - 1) / a["pack_nominal_V"],
                "RMS_current_A": legacy + peak_copper * (2 / 0.95 - 1) / a["pack_nominal_V"],
            }
        )
        if name in ["sensors_hold", "sensors_drive", "sensors_combined"]:
            shaft = row["positive_motor_work_J"] / duration
            scenarios = []
            for temperature in a["copper_temperature_scenarios_C"]:
                temperature_factor = 1 + a["copper_alpha20_per_K"] * (temperature - 20)
                for convention, factor in [("peak_phase", 1), ("RMS_current", 2)]:
                    for efficiency in a["inverter_efficiency_scenarios"]:
                        power = (
                            shaft + factor * temperature_factor * float(copper.sum())
                        ) / efficiency + a["auxiliary_power_W_assumption"]
                        scenarios.append(
                            {
                                "copper_temperature_C": temperature,
                                "current_convention": convention,
                                "inverter_efficiency": efficiency,
                                "average_bus_W": power,
                                "runtime_minutes": 60 * usable_Wh / power,
                            }
                        )
            runtime[name] = {
                "observed_duration_s": duration,
                "physical_pass": row["physical_pass"],
                "positive_mechanical_average_W": shaft,
                "scenarios": scenarios,
            }
    required = {}
    for convention, heat in heating.items():
        worst = np.max(heat, axis=0) if heat else np.zeros(6)
        path = np.divide(limit - air, worst, out=np.full(6, np.nan), where=worst > 0)
        required[convention] = [float(v) if np.isfinite(v) else None for v in path]
    return {
        "assumptions": a,
        "hot_copper_resistance_at_design_limit_ohm": (resistance * hot_factor).tolist()
        if resistance.ndim
        else float(resistance * hot_factor),
        "required_winding_to_air_K_W_each_at_125C": required,
        "thermal_cases": thermal,
        "cold_peak_bus_current_A_upper_bound": {
            "inverter_efficiency": 0.95,
            "peak_phase_A": max(r["peak_phase_A"] for r in bounds),
            "RMS_current_A": max(r["RMS_current_A"] for r in bounds),
            "basis": "Legacy measured-simulation peak plus all six individual current-peak copper corrections; these peaks need not coincide. Conservative algebraic upper bounds, not measured maxima. Fixed cold resistance and 22.2V engineering bus.",
        },
        "usable_energy_Wh_assumption": usable_Wh,
        "runtime": runtime,
        "runtime_basis": "Positive motor-shaft work plus temperature-corrected copper, both divided by inverter efficiency, plus auxiliaries. No regeneration credit. Nominal pack energy and 80% usable fraction are assumptions, not guaranteed runtime; hot125C is a fixed-temperature sensitivity, not coupled duty-cycle thermal prediction.",
        "thermal_hardware_qualified": False,
    }


def rescreen_saved(source, output):
    """Add corrected electrical scenarios without rerunning/changing measurements."""
    source = source / "summary.json" if source.is_dir() else source
    if output.resolve() == source.parent.resolve():
        raise ValueError("Rescreen output must differ from the source suite directory")
    content = source.read_bytes()
    record = json.loads(content)
    if record.get("status") != "complete" or record["nominal"].get("profile") != "m9":
        raise ValueError("Rescreen requires a completed M9 summary")
    assumptions = json.loads((ROOT / "config/m9-screen-assumptions.json").read_text())
    for controller in record["controllers"].values():
        rows = {
            name: case["metrics"].get("physical_metrics", case["metrics"])
            for name, case in controller["cases"].items()
        }
        capacity = (
            controller["nominal"]["joint_drive"]["stator_mass_kg_assumption"]
            * assumptions["stator_cp_J_kg_K"]
        )
        controller["conditional_electrical"]["corrected"] = m9_corrected_electrical_screen(
            rows, assumptions, capacity
        )
    record["electrical_rescreen"] = {
        "source_summary": str(source.resolve()),
        "source_summary_sha256": hashlib.sha256(content).hexdigest(),
        "report_source_sha256": {
            "bipe/robustness.py": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "config/m9-screen-assumptions.json": hashlib.sha256(
                (ROOT / "config/m9-screen-assumptions.json").read_bytes()
            ).hexdigest(),
        },
        "policy": "Original configurations, case measurements/traces, margins, outcome and runtime_source_sha256 preserved; only corrected electrical subtree and this report provenance added. No simulation rerun or hardware qualification.",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(
        json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
    )
    return record


def stator_capacity(nominal, assumptions):
    """Credited stator heat capacity (J/K), one value or one per motor."""
    if "stator_mass_kg_each" in assumptions:
        return np.asarray(assumptions["stator_mass_kg_each"]) * assumptions["stator_cp_J_kg_K"]
    return nominal["joint_drive"]["stator_mass_kg_assumption"] * assumptions["stator_cp_J_kg_K"]


def m9_electrical_screen(rows, nominal):
    """Duty-cycle engineering screen with explicit unqualified housing bounds.

    Catalog phase-current convention is ambiguous; the RMS interpretation
    doubles the winding loss. No assumed thermal path becomes a hardware pass.
    """
    assumptions = json.loads((ROOT / native(nominal)["electrical"]).read_text())
    resistance = np.asarray(assumptions["phase_to_phase_resistance_ohm"], dtype=float)
    ambient = assumptions["housing_air_C"]
    capacity = stator_capacity(nominal, assumptions)
    winding_limit = assumptions["winding_design_limit_C"]
    motors = []
    thermal = []
    for name, row in rows.items():
        duration = max(row["duration_s"], 0.001)
        rms = np.asarray(row["phase_current_rms_A"])
        copper = 0.75 * resistance * rms**2
        # Full-charge no-pack-sink upper bound: all gross negative work goes
        # into windings. Actual bus return is also reported independently.
        regen_heat = np.asarray(row["negative_motor_work_each_J"]) / duration
        cold_temperatures = []
        steady_temperatures = []
        for factor in [1, 2]:
            power = factor * copper + regen_heat
            cold_temperatures.append(
                [
                    (ambient + power * path * (-np.expm1(-duration / (path * capacity)))).tolist()
                    for path in assumptions["winding_to_housing_air_K_W_sensitivity"]
                ]
            )
            steady_temperatures.append(
                [
                    (ambient + power * path).tolist()
                    for path in assumptions["winding_to_housing_air_K_W_sensitivity"]
                ]
            )
        thermal.append(
            {
                "case": name,
                "duration_s": duration,
                "copper_W_each_peak_phase": copper.tolist(),
                "gross_regen_heat_W_each": regen_heat.tolist(),
                "cold_start_winding_C": cold_temperatures,
                "repeat_duty_steady_winding_C": steady_temperatures,
            }
        )
        if name != "unpowered":
            motors.append((name, duration, row, copper))
    peak = np.max([row["phase_current_peak_A"] for row in rows.values()], axis=0)
    rms_peak = np.max([row["phase_current_rms_A"] for row in rows.values()], axis=0)
    maximum_repeat_heat = (
        np.max(
            [
                2 * c + np.asarray(row["negative_motor_work_each_J"]) / duration
                for _, duration, row, c in motors
            ],
            axis=0,
        )
        if motors
        else np.zeros(6)
    )
    required_path = np.divide(
        winding_limit - ambient,
        maximum_repeat_heat,
        out=np.full(6, np.nan),
        where=maximum_repeat_heat > 0,
    )
    usable_Wh = (
        assumptions["pack_capacity_Ah"]
        * assumptions["pack_nominal_V"]
        * assumptions["usable_capacity_fraction"]
    )
    runtime = {}
    for name, duration, row, copper in motors:
        if name in ["sensors_hold", "sensors_drive", "sensors_combined"]:
            power = row["estimated_dc_input_work_J"] / duration
            runtime[name] = {
                "observed_duration_s": duration,
                "physical_pass": row["physical_pass"],
                "average_bus_W_peak_phase": power,
                "runtime_minutes_peak_phase": 60 * usable_Wh / power,
                "runtime_minutes_RMS_current": 60 * usable_Wh / (power + float(copper.sum())),
            }
    capacitor = assumptions["retained_external_bus_capacitance_F"]
    full = assumptions["pack_full_V"]
    ceiling = assumptions["example_bus_ceiling_V"]
    return {
        "assumptions": assumptions,
        "motor_order": [
            "left_hip0",
            "left_hip1",
            "right_hip0",
            "right_hip1",
            "left_wheel",
            "right_wheel",
        ],
        "phase_current_peak_A_each": peak.tolist(),
        "maximum_case_phase_current_rms_A_each": rms_peak.tolist(),
        "thermal_cases": thermal,
        "required_winding_to_air_K_W_each_at_repeat_duty_RMS": [
            float(v) if np.isfinite(v) else None for v in required_path
        ],
        "thermal_hardware_qualified": False,
        "corrected": m9_corrected_electrical_screen(rows, assumptions, capacity),
        "runtime": runtime,
        "full_charge": {
            "battery_charge_acceptance_A_assumption": 0,
            "gross_negative_motor_work_max_J": max(
                r["negative_motor_work_J"] for r in rows.values()
            ),
            "gross_regen_peak_W": max(r["peak_regen_mechanical_W"] for r in rows.values()),
            "estimated_net_regen_peak_W": max(
                r["peak_net_regen_estimate_W"] for r in rows.values()
            ),
            "estimated_net_regen_max_J": max(r["net_regen_estimate_J"] for r in rows.values()),
            "external_cap_energy_to_example_ceiling_J": 0.5 * capacitor * (ceiling**2 - full**2),
            "required_action": "Qualify retained dump path or moteus max_regen_power_W=0 with current/voltage and winding-heat limits; no CAN-FD adapter implemented",
        },
    }


def write_results(output, record, plot=True):
    output.mkdir(parents=True, exist_ok=True)
    record["outcome"] = comparison_outcome(record)
    temporary = output / "summary.json.tmp"
    temporary.write_text(
        json.dumps(
            record,
            separators=(",", ":"),
            allow_nan=False,
            default=lambda o: o.item() if isinstance(o, np.generic) else str(o),
        )
        + "\n"
    )
    temporary.replace(output / "summary.json")
    with (output / "metrics.csv").open("w") as stream:
        keys = [
            "controller",
            "case",
            "physical_pass",
            "recovered",
            "passed",
            "peak_pitch_deg",
            "late_pitch_pp_deg",
            "rollback_m",
            "peak_wheel_Nm",
            "peak_conditional_dc_input_A",
            "failure",
        ]
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        for name, controller in record["controllers"].items():
            for cn, case in controller["cases"].items():
                m = case["metrics"]
                physics = m.get("physical_metrics", m)
                row = {**physics, **m, "controller": name, "case": cn}
                writer.writerow({k: row.get(k) for k in keys})
    if plot and all(
        all(
            n in c["cases"]
            for n in [
                "push_hold_-0.5",
                "push_hold_+2.5",
                "slope_hold_0",
                "slope_hold_10",
                "backlash_hold_0",
                "backlash_hold_0.6",
            ]
        )
        for c in record["controllers"].values()
    ):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
        for name, c in record["controllers"].items():
            ms = c["margins"]
            impulses = [-0.5, -1, -1.5, -2, -2.5, 0.5, 1, 1.5, 2, 2.5]
            peaks = [c["cases"][f"push_hold_{v:+g}"]["metrics"]["peak_pitch_deg"] for v in impulses]
            color = (
                axs[0]
                .plot(impulses, peaks, ls="none", marker="o", fillstyle="none", label=name)[0]
                .get_color()
            )
            passed = [
                i
                for i, v in enumerate(impulses)
                if c["cases"][f"push_hold_{v:+g}"]["metrics"]["recovered"]
            ]
            axs[0].scatter(
                [impulses[i] for i in passed], [peaks[i] for i in passed], color=color, s=20
            )
            axs[1].plot(
                [0, 4, 8, 10],
                [c["cases"][f"slope_hold_{v}"]["metrics"]["rollback_m"] for v in [0, 4, 8, 10]],
                "o-",
                label=name,
            )
            axs[2].plot(
                [0, 0.3, 0.6],
                [ms[f"backlash_hold_{v:g}_pp_deg"] for v in [0, 0.3, 0.6]],
                "o-",
                label=name,
            )
        for ax in axs:
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        axs[0].set(
            xlabel="Signed chassis impulse (N·s)",
            ylabel="Peak pitch error (°)",
            title="Nominal hold: filled = recovered",
        )
        axs[0].axhline(20, color="gray", ls="--")
        axs[1].set(xlabel="Ground slope (°)", ylabel="Maximum rollback (m)")
        axs[1].axhline(record["slope_rollback_limit_m"], color="gray", ls="--")
        axs[2].set(xlabel="Wheel backlash (°)", ylabel="Last 4 s pitch peak-to-peak (°)")
        axs[2].axhline(1.5, color="gray", ls="--")
        fig.savefig(output / "robustness.png", dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=CHOICES, default="m4")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument(
        "--controllers",
        nargs="+",
        default=["baseline", "cascade-retuned", "lqr", "cascade-corrected"],
    )
    parser.add_argument("--phase", choices=["all", "robustness", "acceptance"], default="all")
    parser.add_argument("--case", action="append", help="Exact case names, for compact tuning only")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Continue saved cases only when configuration and runtime sources match",
    )
    parser.add_argument(
        "--rescreen",
        type=Path,
        help="Postprocess a completed M9 summary into a separate --output; no simulation",
    )
    args = parser.parse_args()
    if args.rescreen is not None:
        if args.output is None:
            parser.error("--rescreen requires an explicit separate --output")
        try:
            record = rescreen_saved(args.rescreen, args.output)
        except (ValueError, OSError) as error:
            parser.error(str(error))
        print(
            json.dumps(
                {
                    "rescreened": str(args.output),
                    "source": record["electrical_rescreen"]["source_summary"],
                }
            )
        )
        return
    p = load_parameters(profile=args.profile)
    profile = native(p)
    if args.output is None:
        args.output = ROOT / (profile["acceptance"] if profile else "artifacts/milestone-8")
    sensors = json.loads(
        (ROOT / (profile["sensors"] if profile else "config/sensors-m4.json")).read_text()
    )
    slope_limit = NATIVE_SLOPE_ROLLBACK_LIMIT_M if profile else 0.3
    tasks = [] if args.phase == "acceptance" else matrix(p)
    if args.phase != "acceptance":
        tasks.append(
            {
                "name": "backlash_hold_0.6_half_timestep",
                "family": "numerical",
                "mode": "hold",
                "duration": 20,
                "disturbance": {"backlash_deg": 0.6},
                "physics_dt": p["simulation"]["physics_dt"] / 2,
            }
        )
    if args.phase != "robustness":
        for job in jobs(sensors, "c", p):
            tasks.append({"name": job["name"], "family": "acceptance", "job": job})
        tasks.append(
            {
                "name": "combined_half_timestep",
                "family": "acceptance",
                "job": {
                    "scenario": "combined",
                    "duration": 36,
                    "sensing": "sensors",
                    "sensor_config": sensors,
                    "implementation": "c",
                    "physics_dt": p["simulation"]["physics_dt"] / 2,
                },
            }
        )
    if args.case:
        tasks = [c for c in tasks if c["name"] in args.case]
    if not tasks:
        parser.error("No matching cases")
    record = {
        "scope": f"{args.profile.upper()} C/sensor simulation with M8 permanent yardstick, no hardware qualification",
        "nominal": p,
        "sensors": sensors,
        "physical_limits": {"pitch_deg": 20, "roll_deg": 10, "joint_tolerance_rad": 0.001},
        "recovery_last_s": 2,
        "recovery_pitch_deg": 2,
        "recovery_speed_m_s": 0.03,
        "slope_rollback_limit_m": slope_limit,
        "trace_columns": [
            "time",
            "x",
            "y",
            "velocity",
            "pitch_error_deg",
            "user_velocity",
            "yaw_rate",
            "left_applied_Nm",
            "right_applied_Nm",
            "forbidden_contact",
        ],
        "trace_dt_s": 0.1,
        "controllers": {},
        "status": "running",
        "requested_cases": sorted(c["name"] for c in tasks),
    }
    runtime_sources = [
        ROOT / "bipe" / (n + ".py")
        for n in [
            "articulated",
            "kinematics",
            "locomotion",
            "moving_experiment",
            "parameters",
            "sensing",
            "firmware_control",
            "control",
            "disturbances",
            "robustness_control",
            "robustness",
            "moving_validate",
        ]
    ]
    runtime_sources += sorted((ROOT / "firmware/src").glob("*.c"))
    runtime_sources += [
        ROOT / "firmware/include/bipe.h",
        ROOT / "tools/generate_firmware_config.py",
    ]
    if profile:
        runtime_sources += [ROOT / "bipe" / (n + ".py") for n in profile["runtime_sources"]]
        runtime_sources += [ROOT / "bipe/profiles.py", ROOT / profile["electrical"]]
    record["runtime_source_sha256"] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in runtime_sources
    }
    previous = None
    if args.resume and (args.output / "summary.json").exists():
        previous = json.loads((args.output / "summary.json").read_text())
        for key in ["nominal", "sensors", "requested_cases", "runtime_source_sha256"]:
            if previous.get(key) != record[key]:
                parser.error(
                    f"Cannot resume changed {key}; use a separate output for a complete rerun"
                )
        if set(previous["controllers"]) != set(args.controllers):
            parser.error("Cannot resume changed controller selection")
    work = []
    for name in args.controllers:
        nominal = copy.deepcopy(p)
        if name == "baseline":
            nominal["motor"]["peak_torque_per_wheel"] = 0.6
            nominal["motor"]["peak_basis"] = (
                "Historical inconsistent 0.6 Nm placeholder, comparison only; not a permitted current parameter file"
            )
        gain_path = (
            ROOT / (profile["controller"] if profile else "config/controller_moving.json")
            if name == "current"
            else ROOT / f"config/m8/{name}.json"
        )
        gains = json.loads(gain_path.read_text())
        library = build_candidate(nominal, gains, ROOT / f".venv/m8-build/{name}")
        check = equivalence(nominal, gains, library)
        if not check["passed"]:
            raise ValueError(f"{name} C/Python mismatch: {check}")
        record["controllers"][name] = {
            "nominal": nominal,
            "gains": gains,
            "equivalence": check,
            "cases": {},
        }
        if previous is not None:
            saved = previous["controllers"][name]
            if saved["nominal"] != nominal or saved["gains"] != gains:
                parser.error(f"Cannot resume changed {name} controller or plant")
            record["controllers"][name]["cases"] = saved["cases"]
        work.extend(
            (name, nominal, gains, sensors, str(library), case)
            for case in tasks
            if case["name"] not in record["controllers"][name]["cases"]
        )
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(execute, payload) for payload in work]):
            name, cn, result = future.result()
            record["controllers"][name]["cases"][cn] = result
            m = result["metrics"]
            print(name, cn, m.get("passed", m.get("recovered")), m.get("failure"), flush=True)
            write_results(args.output, record, plot=False)
    for row in record["controllers"].values():
        row["cases"] = dict(sorted(row["cases"].items()))
        row["margins"] = margins(row["cases"], slope_limit)
    record["status"] = "complete"
    write_results(args.output, record)
    print(json.dumps({k: v["margins"] for k, v in record["controllers"].items()}, indent=2))


if __name__ == "__main__":
    main()
