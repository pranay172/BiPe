"""M12 phase B: speed-envelope gates for the M10 balance controller.

Every case runs as the acceptance runs its cases: the C core with the sensor model, a free
base, finite motor limits and physical contact. The M11 electrical recorder rides along as
a read-only observer. Commands are piecewise-linear knots from config/speed-m12.json, or
replays of the phase A ideal-follower commands. The outputs go to artifacts/m12/speed/.
"""

import argparse
import copy
import hashlib
import json
import math
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from ..m10.kinematics import height_command, posture
from ..m11.requirements import ElectricalRecorder, model_config
from ..moving_experiment import COLUMNS, metrics, simulate
from ..parameters import ROOT, load_parameters
from ..profiles import native
from ..robustness import physical_metrics
from ..robustness_control import build_candidate, equivalence

CONFIG = ROOT / "config/speed-m12.json"
OUTPUT = ROOT / "artifacts/m12/speed"
FOLLOWER = ROOT / "artifacts/m12/requirements/follower_traces.npz"
TRACE_COLUMNS = [
    "time",
    "x",
    "y",
    "velocity",
    "user_velocity",
    "velocity_reference",
    "estimated_velocity",
    "yaw_deg",
    "yaw_rate",
    "user_yaw_rate",
    "heading_reference_deg",
    "pitch_error_deg",
    "estimated_pitch_deg",
    "roll_deg",
    "left_applied_Nm",
    "right_applied_Nm",
]
TRACE_DT_S = 0.02


def load():
    p = load_parameters(profile="m10")
    profile = native(p)
    gains = json.loads((ROOT / profile["controller"]).read_text())
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    return p, gains, sensors, json.loads(CONFIG.read_text())


def knot_command(knots, t):
    """Piecewise-linear command; a repeated knot time is a step to the later value."""
    k = np.asarray(knots, dtype=float)
    i = int(np.searchsorted(k[:, 0], t, side="right")) - 1
    if i < 0:
        return k[0, 1:]
    if i >= len(k) - 1:
        return k[-1, 1:]
    s = (t - k[i, 0]) / (k[i + 1, 0] - k[i, 0])
    return k[i, 1:] + s * (k[i + 1, 1:] - k[i, 1:])


def follower_trace(layout, scenario):
    with np.load(FOLLOWER) as z:
        columns = [str(c) for c in z["columns"]]
        trace = z[f"{layout}/{scenario}"]
    return {c: trace[:, i] for i, c in enumerate(columns)}


def replay_command(f, start, t):
    """The ideal follower's speed and yaw rate, zero while it waits or holds."""
    from .follower import MODES

    dt = f["t"][1] - f["t"][0]
    i = math.floor((t - start) / dt + 1e-9)
    if i < 0 or i >= len(f["t"]):
        return np.zeros(2)
    if MODES[int(f["mode"][i])] in ("waiting", "holding"):
        return np.zeros(2)
    return np.array([f["speed"][i], f["yaw_rate"][i]])


def definitions(cfg):
    """All cases by name: the gated cases, slopes, replays, variants, convergence, information."""
    cases = {}
    for name, c in cfg["cases"].items():
        cases[name] = {
            "name": name,
            "gate": c.get("gate", "full"),
            "duration": c["knots"][-1][0],
            **{k: v for k, v in c.items() if k != "gate"},
        }
    s = cfg["slopes"]
    for slope in s["degrees"]:
        for direction, sign in [("up", 1), ("down", -1)]:
            name = f"slope_{direction}_{slope:g}"
            cases[name] = {
                "name": name,
                "purpose": f"Drive {direction} a {slope:g} deg slope at {s['speed_m_s']:g} m/s",
                "knots": [[t, v * s["speed_m_s"], w] for t, v, w in s["knots_unit_speed"]],
                "duration": s["knots_unit_speed"][-1][0],
                "plateaus": s["plateaus"],
                "stops": s["stops"],
                "disturbance": {"slope_deg": sign * slope},
                "gate": s["gate"],
            }
    r = cfg["replays"]
    for scenario in r["scenarios"]:
        f = follower_trace(r["layout"], scenario)
        name = f"replay_{scenario}"
        cases[name] = {
            "name": name,
            "purpose": f"Ideal-follower commands, {r['layout']} layout, {scenario}",
            "replay": {"layout": r["layout"], "scenario": scenario, "start_s": r["start_s"]},
            "duration": round(r["start_s"] + float(f["t"][-1]) + r["settle_s"], 2),
            "gate": "full",
        }
    v = cfg["variants"]
    for variant in v["names"]:
        for base in v["cases"]:
            name = f"{variant}_{base}"
            cases[name] = {
                **copy.deepcopy(cases[base]),
                "name": name,
                "purpose": f"{base} with {variant}",
                "variant": variant,
                "gate": v["gate"],
            }
    c = cfg["convergence"]
    name = f"{c['case']}_half_timestep"
    cases[name] = {
        **copy.deepcopy(cases[c["case"]]),
        "name": name,
        "purpose": f"{c['case']} at {c['physics_dt_scale']:g} x the physics timestep",
        "physics_dt_scale": c["physics_dt_scale"],
        "gate": "convergence",
    }
    for name, info in cfg["information"].items():
        cases[name] = {
            **copy.deepcopy(cases[info["base"]]),
            "name": name,
            "purpose": f"{info['base']} at posture {info['posture_deg']} (information only)",
            "posture_deg": info["posture_deg"],
            "disturbance": {"initial_posture_deg": info["posture_deg"]},
            "gate": "information",
        }
    return cases


def variant_inputs(p, sensors, variant):
    """Plant and sensor configuration of an acceptance variant (moving_validate.jobs)."""
    plant, config, seed = copy.deepcopy(p), copy.deepcopy(sensors), None
    if variant == "mass_plus_15pct":
        for row in plant.get("mass_budget", []):
            row["mass_each"] *= 1.15
        for row in plant["rigid_bodies"].values():
            row["mass_kg"] *= 1.15
            row["inertia_kg_m2"] = (np.asarray(row["inertia_kg_m2"]) * 1.15).tolist()
    elif variant == "friction_0p5":
        plant["contact"]["sliding_friction"] = 0.5
    elif variant is not None and variant.startswith("wheel_lag_"):
        lag_ms = float(variant.removeprefix("wheel_lag_").removesuffix("ms"))
        plant["physics"]["hub_motor"]["FOC_time_constant_s"] = lag_ms / 1000
    elif variant == "delay_15ms_noise_x2":
        config["transport_delay_s"] = 0.015
        config["gyro_noise_std_rad_s"] *= 2
        config["accel_noise_std_m_s2"] *= 2
    elif variant in ("seed_plus_1", "seed_plus_2"):
        seed = sensors["seed"] + int(variant[-1])
    elif variant is not None:
        raise ValueError(f"unknown variant {variant}")
    return plant, config, seed


def run_case(p, gains, sensors, library, case, observer=None):
    plant, config, seed = variant_inputs(p, sensors, case.get("variant"))
    if case.get("physics_dt_scale"):
        plant["simulation"]["physics_dt"] = p["simulation"]["physics_dt"] * case["physics_dt_scale"]
    qref = posture(case.get("posture_deg", 40), p)
    if "replay" in case:
        r = case["replay"]
        f = follower_trace(r["layout"], r["scenario"])

        def command(t, scenario):
            v, w = replay_command(f, r["start_s"], t)
            return float(v), float(w), qref, np.zeros(4)
    elif "posture_knots" in case:
        # M15 moves: a posture label over time ([t, label] knots), with the references
        # interpolated from a 0.5 degree table.
        labels = np.arange(30.0, 65.01, 0.5)
        table = np.array([posture(float(x), p) for x in labels])
        k = np.asarray(case["posture_knots"], dtype=float)

        def command(t, scenario):
            v, w = knot_command(case["knots"], t)
            label = float(np.interp(t, k[:, 0], k[:, 1]))
            q = np.array([np.interp(label, labels, table[:, j]) for j in range(4)])
            return float(v), float(w), q, np.zeros(4)
    else:

        def command(t, scenario):
            v, w = knot_command(case["knots"], t)
            return float(v), float(w), qref, np.zeros(4)

    return simulate(
        "m12_speed",
        case["duration"],
        sensing="sensors",
        seed=seed,
        nominal=p,
        plant=plant,
        gains=gains,
        sensor_config=config,
        implementation="c",
        firmware_library=library,
        physics_dt=plant["simulation"]["physics_dt"],
        disturbance=case.get("disturbance", {}),
        command_function=command,
        observer=observer,
    )


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def speed_metrics(run, case):
    """Tracking metrics of one case, from the 1 kHz trace."""
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    t, v, cmd = f["time"], f["velocity"], f["user_velocity"]
    xy = run["trace"][:, 1:3]
    out = {
        "peak_pitch_deg": float(np.abs(f["pitch_error_deg"]).max()),
        "peak_roll_deg": float(np.abs(f["roll_deg"]).max()),
        "peak_speed_m_s": float(np.abs(v).max()),
        "peak_command_m_s": float(np.abs(cmd).max()),
        "peak_yaw_rate_rad_s": float(np.abs(f["yaw_rate"]).max()),
        "final_heading_error_deg": abs(
            float(wrap_deg(f["heading_reference_deg"][-1] - f["yaw_deg"][-1]))
        ),
        "peak_wheel_Nm": float(
            max(np.abs(f["left_applied_Nm"]).max(), np.abs(f["right_applied_Nm"]).max())
        ),
    }
    if case.get("plateaus"):
        out["overshoot_m_s"] = out["peak_speed_m_s"] - out["peak_command_m_s"]
        # A run that ends before its command starts (an early fall) has no peak command.
        out["overshoot_fraction"] = (
            out["peak_speed_m_s"] / out["peak_command_m_s"] - 1
            if out["peak_command_m_s"] > 0
            else float("inf")
        )
    out["plateau_speed_error_m_s"] = [
        float(np.mean(np.abs(v - cmd)[(t >= a) & (t < b)])) for a, b in case.get("plateaus", [])
    ]
    out["yaw_rate_error_rad_s"] = [
        float(np.mean(np.abs(f["yaw_rate"] - f["user_yaw_rate"])[(t >= a) & (t < b)]))
        for a, b in case.get("yaw_windows", [])
    ]
    stops = []
    for ts in case.get("stops", []):
        i = int(np.searchsorted(t, ts - 1e-9))
        if i >= len(t):
            continue
        travel = float(np.linalg.norm(xy[i:] - xy[i], axis=1).max())
        # The shaped reference's own remaining distance, until it settles at rest.
        vref = f["velocity_reference"][i:]
        rest = np.flatnonzero(vref == 0)
        end = rest[0] + 1 if len(rest) else len(vref)
        reference = float(abs(np.trapezoid(vref[:end], t[i : i + end])))
        stops.append(
            {
                "t_s": ts,
                "speed_m_s": float(abs(v[i])),
                "travel_m": travel,
                "reference_m": reference,
                "excess_m": travel - reference,
            }
        )
    out["stops"] = stops
    if "replay" in case:
        r = case["replay"]
        g = follower_trace(r["layout"], r["scenario"])
        shifted = g["t"] + r["start_s"]
        inside = (t >= shifted[0]) & (t <= shifted[-1])
        ideal = np.c_[
            np.interp(t[inside], shifted, g["x"] - g["x"][0]),
            np.interp(t[inside], shifted, g["y"] - g["y"][0]),
        ]
        out["path_error_m"] = float(np.linalg.norm(xy[inside] - xy[0] - ideal, axis=1).max())
    return out


STEADY_HOLD_S = 1.0  # bipe.m13.rl.evaluate --settle: the command held this long
STEADY_TOL = (0.01, 0.02)  # speed reference (m/s) and yaw-rate command (rad/s) spread


def steady_mask(f, hold=STEADY_HOLD_S, tol=STEADY_TOL):
    """Samples whose shaped speed reference and yaw-rate command have stayed within
    ``tol`` over the last ``hold`` seconds (rest included): the steady segments the RL
    training's posture metric measures (command reached and held)."""
    t, v, w = f["time"], f["velocity_reference"], f["user_yaw_rate"]
    start = np.searchsorted(t, t - hold - 1e-9)
    out = np.zeros(len(t), dtype=bool)
    for i, j in enumerate(start):
        if t[i] - t[0] < hold - 1e-9:
            continue
        out[i] = np.ptp(v[j : i + 1]) <= tol[0] and np.ptp(w[j : i + 1]) <= tol[1]
    return out


def axle_depth_error(run, case, p):
    """The axle height (chassis origin above the mean wheel centre, the depth the RL
    training's posture metric uses) against the commanded posture's depth, mm: p95 and
    peak over the case, and p95 over its steady segments (steady_mask), which M14
    gate 3 judges (+-5 mm, user decision 1 November 2026)."""
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    error = np.abs(f["height"] - height_command(case.get("posture_deg", 40), p)) * 1000
    steady = steady_mask(f)
    return {
        "p95_mm": float(np.percentile(error, 95)),
        "peak_mm": float(error.max()),
        "steady_p95_mm": float(np.percentile(error[steady], 95)) if steady.any() else None,
        "steady_fraction": float(steady.mean()),
    }


def electrical_summary(summary):
    hubs = [summary["motors"][m] for m in ("left_hub", "right_hub")]
    hips = [summary["motors"][m] for m in ("left_hip", "left_knee", "right_hip", "right_knee")]
    draw = summary["draw"]
    return {
        "hub_phase_current_peak_A": max(h["phase_current_peak_A"] for h in hubs),
        "hub_time_at_current_limit_fraction": max(
            h["time_at_current_limit_fraction"] for h in hubs
        ),
        "hub_time_at_voltage_limit_fraction": max(
            h["time_at_voltage_limit_fraction"] for h in hubs
        ),
        "hub_phase_voltage_demand_peak_V": max(
            h["draw"]["phase_voltage_demand_peak_V"] for h in hubs
        ),
        "hub_rotor_speed_peak_rad_s": max(h["rotor_speed_peak_rad_s"] for h in hubs),
        "leg_phase_current_peak_A": max(h["phase_current_peak_A"] for h in hips),
        "bus_current_peak_A_pack_low": draw["pack_low"]["bus_current_peak_A"],
        "bus_current_rms_A_pack_low": draw["pack_low"]["bus_current_rms_A"],
        "bus_power_mean_W": draw["bus_power_mean_W"],
        "runtime_at_this_load_min": draw["runtime_at_this_load_min"],
    }


def assess(result, case, criteria):
    """Pass or fail of one case against its gate."""
    phys, s = result["physical"], result["speed"]
    if case["gate"] == "slope":
        late = phys["late_speed_peak_m_s"]
        held = late is not None and late <= criteria["hold_speed_max_m_s"]
        checks = {"survives_and_holds": phys["physical_pass"] and result["general_passed"] and held}
        return {"checks": checks, "passed": all(checks.values())}
    survive = phys["physical_pass"] and phys["recovered"] and result["general_passed"]
    checks = {"survives_and_recovers": survive}
    if case["gate"] == "full":
        checks["peak_pitch"] = s["peak_pitch_deg"] <= criteria["peak_pitch_max_deg"]
        if "overshoot_m_s" in s:
            checks["overshoot"] = s["overshoot_m_s"] <= criteria["overshoot_max_m_s"]
        if s["plateau_speed_error_m_s"]:
            checks["plateau"] = (
                max(s["plateau_speed_error_m_s"]) <= criteria["plateau_speed_error_max_m_s"]
            )
        if s["yaw_rate_error_rad_s"]:
            checks["yaw_rate"] = (
                max(s["yaw_rate_error_rad_s"]) <= criteria["yaw_rate_error_max_rad_s"]
            )
        checks["heading"] = s["final_heading_error_deg"] <= criteria["heading_error_max_deg"]
        if s["stops"]:
            checks["stop"] = max(x["excess_m"] for x in s["stops"]) <= criteria["stop_excess_max_m"]
        if "path_error_m" in s:
            checks["path"] = s["path_error_m"] <= criteria["replay_path_error_max_m"]
    if case["gate"] == "information":
        return {"checks": checks, "passed": None}
    return {"checks": checks, "passed": all(checks.values())}


def task(payload):
    p, gains, sensors, library, case, electrical, output = payload
    recorder = ElectricalRecorder(electrical)
    run = run_case(p, gains, sensors, library, case, observer=recorder)
    general = metrics(run)
    result = {
        "case": case,
        "failure": run["failure"],
        "warnings": run["warnings"],
        "general_passed": bool(general["passed"]),
        "longest_saturation_s": general["longest_saturation_s"],
        "max_joint_error_deg": general["max_joint_error_deg"],
        # Set when the runner's metrics() are wrapped to judge without the joint-error
        # criterion (bipe.m13.full_plant.with_joint_exemption).
        "general_passed_without_joint_error": general.get("passed_without_joint_error"),
        "axle_depth_error": axle_depth_error(run, case, p),
        "physical": physical_metrics(run),
        "speed": speed_metrics(run, case),
        "electrical": electrical_summary(recorder.summary()),
    }
    step = max(1, round(TRACE_DT_S / float(np.diff(run["trace"][:2, 0])[0])))
    index = [COLUMNS.index(c) for c in TRACE_COLUMNS]
    trace = run["trace"][::step, index]
    if trace[-1, 0] != run["trace"][-1, 0]:
        trace = np.vstack([trace, run["trace"][-1, index]])
    (Path(output) / "results" / f"{case['name']}.json").write_text(
        json.dumps(result, indent=2, default=str) + "\n"
    )
    np.savez_compressed(
        Path(output) / "traces" / f"{case['name']}.npz",
        columns=np.array(TRACE_COLUMNS),
        trace=trace.astype(np.float32),
    )
    return case["name"], result


def convergence(results, cfg):
    c = cfg["convergence"]
    base, fine = results[c["case"]]["speed"], results[f"{c['case']}_half_timestep"]["speed"]
    pitch = abs(base["peak_pitch_deg"] - fine["peak_pitch_deg"])
    speed = abs(base["peak_speed_m_s"] - fine["peak_speed_m_s"])
    return {
        "pitch_peak_difference_deg": pitch,
        "speed_peak_difference_m_s": speed,
        "passed": pitch <= c["pitch_peak_difference_max_deg"]
        and speed <= c["speed_peak_difference_max_m_s"],
    }


def plot(results, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [n for n in results if not results[n]["case"].get("variant")]
    fig, axes = plt.subplots(len(names), 3, figsize=(15, 2.2 * len(names)), squeeze=False)
    for row, name in zip(axes, names, strict=True):
        with np.load(output / "traces" / f"{name}.npz") as z:
            f = {str(c): z["trace"][:, i] for i, c in enumerate(z["columns"])}
        t = f["time"]
        row[0].plot(t, f["user_velocity"], "k--", lw=0.8, label="command")
        row[0].plot(t, f["velocity_reference"], lw=0.8, label="reference")
        row[0].plot(t, f["velocity"], lw=1.0, label="true")
        row[0].plot(t, f["estimated_velocity"], lw=0.8, label="estimate")
        row[0].set_ylabel(name, fontsize=7)
        row[1].plot(t, f["user_yaw_rate"], "k--", lw=0.8)
        row[1].plot(t, f["yaw_rate"], lw=1.0)
        row[2].plot(t, f["pitch_error_deg"], lw=1.0, label="true")
        row[2].plot(t, f["estimated_pitch_deg"] - f["pitch_error_deg"], lw=0.8, label="est. - true")
        for ax in row:
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=7)
    axes[0, 0].set_title("speed (m/s)")
    axes[0, 1].set_title("yaw rate (rad/s)")
    axes[0, 2].set_title("pitch from equilibrium (deg)")
    axes[0, 0].legend(fontsize=6)
    axes[0, 2].legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(output / "speed_cases.png", dpi=90)
    plt.close(fig)


def _sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--case", action="append", help="run only these case names")
    args = parser.parse_args()
    p, gains, sensors, cfg = load()
    cases = definitions(cfg)
    if args.case:
        unknown = sorted(set(args.case) - set(cases))
        if unknown:
            parser.error(f"unknown cases {unknown}")
        cases = {n: c for n, c in cases.items() if n in args.case}
    library = build_candidate(p, gains, ROOT / native(p)["firmware_build"])
    check = equivalence(p, gains, library)
    if not check["passed"]:
        raise ValueError(f"C/Python mismatch: {check}")
    electrical = model_config(p)
    output = args.output
    (output / "traces").mkdir(parents=True, exist_ok=True)
    (output / "results").mkdir(parents=True, exist_ok=True)
    record = {
        "scope": "M10 C/sensor simulation under stated assumptions; not hardware-validated",
        "envelope": json.loads((ROOT / "config/robot.json").read_text())["m10"]["targets"][
            "drive_envelope"
        ],
        "drive_speed_gate_m_s": p["sizing"]["drive_speed"],
        "speed_config": cfg,
        "speed_config_sha256": _sha(cfg),
        "controller": gains,
        "controller_sha256": _sha(gains),
        "sensors": sensors,
        "electrical_model": electrical,
        "equivalence": check,
        "library_sha256": hashlib.sha256(Path(library).read_bytes()).hexdigest(),
    }
    (output / "config.json").write_text(json.dumps(record, indent=2, default=str) + "\n")
    payloads = [
        (p, gains, sensors, str(library), c, electrical, str(output)) for c in cases.values()
    ]
    results = {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(task, x) for x in payloads]):
            name, result = future.result()
            result["assessment"] = assess(result, result["case"], cfg["criteria"])
            results[name] = result
            s = result["speed"]
            print(
                name,
                result["failure"],
                f"pitch {s['peak_pitch_deg']:.1f}",
                f"v {s['peak_speed_m_s']:.2f}/{s['peak_command_m_s']:.2f}",
                result["assessment"]["passed"],
                flush=True,
            )
    results = dict(sorted(results.items()))
    (output / "cases.json").write_text(json.dumps(results, indent=2, default=str) + "\n")
    summary = {
        "scope": record["scope"],
        "controller_sha256": record["controller_sha256"],
        "speed_config_sha256": record["speed_config_sha256"],
        "cases": {n: r["assessment"] for n, r in results.items()},
        "failed": sorted(n for n, r in results.items() if r["assessment"]["passed"] is False),
        "worst": {
            "peak_pitch_deg": max(
                (r["speed"]["peak_pitch_deg"], n)
                for n, r in results.items()
                if r["case"]["gate"] != "information"
            ),
            "hub_phase_current_peak_A": max(
                (r["electrical"]["hub_phase_current_peak_A"], n) for n, r in results.items()
            ),
            "hub_phase_voltage_demand_peak_V": max(
                (r["electrical"]["hub_phase_voltage_demand_peak_V"], n) for n, r in results.items()
            ),
            "bus_current_peak_A_pack_low": max(
                (r["electrical"]["bus_current_peak_A_pack_low"], n) for n, r in results.items()
            ),
        },
    }
    if all(
        n in results
        for n in (cfg["convergence"]["case"], f"{cfg['convergence']['case']}_half_timestep")
    ):
        summary["convergence"] = convergence(results, cfg)
    converged = summary.get("convergence", {}).get("passed", True)
    recorded = set(cfg.get("recorded_shortfalls", {}).get("cases", []))
    summary["recorded_shortfalls"] = sorted(set(summary["failed"]) & recorded)
    summary["unrecorded_failures"] = sorted(set(summary["failed"]) - recorded)
    summary["passed"] = not summary["failed"] and converged
    summary["passed_except_recorded_shortfalls"] = not summary["unrecorded_failures"] and converged
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    plot(results, output)
    keys = ("failed", "unrecorded_failures", "worst", "passed", "passed_except_recorded_shortfalls")
    print(json.dumps({k: summary[k] for k in keys}, indent=2))


if __name__ == "__main__":
    main()
