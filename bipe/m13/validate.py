"""M13 phase A closed-loop validation: the classical controller on the training plant.

Runs the M10 acceptance (robustness matrix and inherited cases) and the M12 speed cases
on the training plant with the C core and the sensor model, exactly as on the full plant,
and compares each case with the full plant's saved result
(artifacts/m10/acceptance/summary.json, artifacts/m12/speed/results/). Cases that vary
the belts (lash or stiffness) are skipped: the training plant has rigid hip belts and no
lash. Outputs go to artifacts/m13/validation/.
"""

import argparse
import copy
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial
from pathlib import Path

import numpy as np

from ..m12 import speed
from ..moving_experiment import COLUMNS, commands, metrics, simulate
from ..moving_validate import jobs
from ..parameters import ROOT, load_parameters
from ..profiles import native
from ..robustness import command, matrix, physical_metrics, plant_variant
from ..robustness_control import build_candidate

OUTPUT = ROOT / "artifacts/m13/validation"
ACCEPTANCE = ROOT / "artifacts/m10/acceptance/summary.json"
SPEED = ROOT / "artifacts/m12/speed"
# Compared metrics: (key, absolute floor, relative band). A case passes a metric when
# |training - full| <= max(floor, relative x |full|).
BANDS = {
    "peak_pitch_deg": (0.5, 0.25),
    "late_pitch_pp_deg": (0.5, 0.25),
    "rollback_m": (0.02, 0.25),
    "max_travel_m": (0.02, 0.25),
    "peak_wheel_Nm": (0.02, 0.25),
    "peak_motor_Nm": (0.05, 0.25),
}
SPEED_BANDS = {
    "peak_pitch_deg": (0.5, 0.25),
    "peak_speed_m_s": (0.02, 0.1),
    "peak_wheel_Nm": (0.02, 0.25),
    "plateau_speed_error_m_s": (0.02, 0.25),
    "stop_travel_m": (0.02, 0.25),
}


def load():
    p10, p13 = load_parameters(profile="m10"), load_parameters(profile="m13")
    profile = native(p10)
    gains = json.loads((ROOT / profile["controller"]).read_text())
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    return p10, p13, gains, sensors


def belt_keys(p13):
    """Belt variant keys the training plant represents (M14 spring hip belts, knee dead band)."""
    drives = p13["training"]["drives"]
    keys = set()
    if drives.get("hip_belt") == "spring":
        keys |= {"hip_backlash_deg", "hip_belt_stiffness_Nm_rad"}
    if drives.get("knee_lash") == "deadband":
        keys |= {"knee_backlash_deg", "backlash_deg", "knee_belt_stiffness_Nm_rad"}
    return keys


def modeled(disturbance, p13=None):
    """False for belt variants the training plant cannot represent."""
    keys = belt_keys(p13) if p13 is not None else set()
    return not any(
        v
        for k, v in (disturbance or {}).items()
        if ("belt" in k or "backlash" in k) and v and k not in keys
    )


def acceptance_cases(p10, p13, sensors):
    """M10 acceptance cases by name, with their training-plant job."""
    cases = {}
    for case in matrix(p10):
        if case["family"] == "belt" and not modeled(case.get("disturbance"), p13):
            continue
        cases[case["name"]] = {**case, "suite": "m10"}
    cases["hold_half_timestep"] = {
        "name": "hold_half_timestep",
        "family": "numerical",
        "mode": "hold",
        "duration": 20,
        "physics_dt": p13["simulation"]["physics_dt"] / 2,
        "suite": "m13",
        "reference": "backlash_hold_0",
    }
    # Variant plants come from the training plant; the controller sees the M10 nominal.
    for job in jobs(sensors, "c", p13):
        cases[job["name"]] = {
            "name": job["name"],
            "family": "acceptance",
            "job": job,
            "suite": "m10",
        }
    job = copy.deepcopy(cases["sensors_combined"]["job"])
    job.update(name="combined_half_timestep", physics_dt=p13["simulation"]["physics_dt"] / 2)
    cases["combined_half_timestep"] = {
        "name": "combined_half_timestep",
        "family": "acceptance",
        "job": job,
        "suite": "m10",
    }
    return cases


def run_acceptance(p10, p13, gains, sensors, library, case):
    if case["family"] == "acceptance":
        kw = copy.deepcopy(case["job"])
        kw.pop("name", None)
        kw.setdefault("plant", p13)
        if "motor" in kw["plant"]:
            kw["plant"]["motor"]["peak_torque_per_wheel"] = p10["motor"]["peak_torque_per_wheel"]
        kw.update(nominal=p10, gains=gains, firmware_library=library)
        kw.setdefault("command_function", partial(_rounded, commands, p=p10))
        return simulate(**kw)
    return simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=p10,
        plant=plant_variant(p13, case.get("variation", {})),
        gains=gains,
        sensor_config=sensors,
        implementation="c",
        firmware_library=library,
        physics_dt=case.get("physics_dt"),
        disturbance=case.get("disturbance", {}),
        command_function=partial(_rounded, command, case=case, nominal=p10),
    )


def _rounded(function, t, scenario, **kw):
    """The scenario command at d.time rounded to 1 ns. At the 1 ms step, accumulated
    time can land a crouch segment boundary a few ulp early, and the M10 crouch
    reference then asks for a posture just outside 30..65 degrees."""
    return function(round(t, 9), scenario, **kw)


def run_speed(p10, p13, gains, sensors, library, case):
    plant, config, seed = speed.variant_inputs(p13, sensors, case.get("variant"))
    if case.get("physics_dt_scale"):
        plant["simulation"]["physics_dt"] = (
            p13["simulation"]["physics_dt"] * case["physics_dt_scale"]
        )
    qref = speed.posture(case.get("posture_deg", 40), p10)
    if "replay" in case:
        r = case["replay"]
        f = speed.follower_trace(r["layout"], r["scenario"])

        def cmd(t, scenario):
            v, w = speed.replay_command(f, r["start_s"], t)
            return float(v), float(w), qref, np.zeros(4)
    else:

        def cmd(t, scenario):
            v, w = speed.knot_command(case["knots"], t)
            return float(v), float(w), qref, np.zeros(4)

    return simulate(
        "m12_speed",
        case["duration"],
        sensing="sensors",
        seed=seed,
        nominal=p10,
        plant=plant,
        gains=gains,
        sensor_config=config,
        implementation="c",
        firmware_library=library,
        physics_dt=plant["simulation"]["physics_dt"],
        disturbance=case.get("disturbance", {}),
        command_function=cmd,
    )


def _trace(run, columns, every_s):
    index = [COLUMNS.index(c) for c in columns]
    if len(run["trace"]) < 2:
        # A run rejected at its first step (the policy cannot run on ideal sensing).
        return np.asarray(run["trace"]).reshape(-1, len(COLUMNS))[:, index]
    step = max(1, round(every_s / float(np.diff(run["trace"][:2, 0])[0])))
    trace = run["trace"][::step, index]
    if trace[-1, 0] != run["trace"][-1, 0]:
        trace = np.vstack([trace, run["trace"][-1, index]])
    return trace


def task(payload):
    suite, case, p10, p13, gains, sensors, library = payload
    if suite == "speed":
        run = run_speed(p10, p13, gains, sensors, library, case)
        general = metrics(run)
        result = {
            "failure": run["failure"],
            "warnings": run["warnings"],
            "general_passed": bool(general["passed"]),
            "physical": physical_metrics(run),
            "speed": speed.speed_metrics(run, case),
        }
        trace = _trace(run, speed.TRACE_COLUMNS, speed.TRACE_DT_S)
        return suite, case["name"], result, speed.TRACE_COLUMNS, trace
    run = run_acceptance(p10, p13, gains, sensors, library, case)
    result = metrics(run) if case["family"] == "acceptance" else physical_metrics(run)
    if case["family"] == "acceptance":
        result["physical_metrics"] = physical_metrics(run)
    columns = ["time", "x", "y", "velocity", "pitch_error_deg", "user_velocity", "yaw_rate"]
    columns += ["left_applied_Nm", "right_applied_Nm", "forbidden_contact"]
    return suite, case["name"], result, columns, _trace(run, columns, 0.01)


def within(train, ref, floor, relative):
    if train is None or ref is None:
        return None
    return bool(abs(train - ref) <= max(floor, relative * abs(ref)))


def compare_acceptance(name, case, result, reference):
    phys_t = result.get("physical_metrics", result)
    phys_f = reference["metrics"].get("physical_metrics", reference["metrics"])
    row = {"suite": "m10", "family": case["family"], "reference": reference["case"]["name"]}
    row["full"] = {k: phys_f.get(k) for k in ["physical_pass", "recovered", *BANDS]}
    row["training"] = {k: phys_t.get(k) for k in ["physical_pass", "recovered", *BANDS]}
    checks = {
        "physical_pass": phys_t["physical_pass"] == phys_f["physical_pass"]
        or phys_t["physical_pass"] is True,
        "recovered": phys_t["recovered"] == phys_f["recovered"] or phys_t["recovered"] is True,
    }
    if case["family"] == "acceptance" and "passed" in result:
        row["full"]["passed"] = reference["metrics"].get("passed")
        row["training"]["passed"] = result.get("passed")
        checks["passed"] = (result.get("passed") == reference["metrics"].get("passed")) or bool(
            result.get("passed")
        )
    for key, (floor, rel) in BANDS.items():
        ok = within(phys_t.get(key), phys_f.get(key), floor, rel)
        if ok is not None:
            checks[key] = ok
    row["checks"] = checks
    row["passed"] = all(checks.values())
    return row


def _speed_values(result):
    s = result["speed"]
    return {
        "peak_pitch_deg": s["peak_pitch_deg"],
        "peak_speed_m_s": s["peak_speed_m_s"],
        "peak_wheel_Nm": s["peak_wheel_Nm"],
        "plateau_speed_error_m_s": max(s["plateau_speed_error_m_s"], default=None),
        "stop_travel_m": max((x["travel_m"] for x in s["stops"]), default=None),
    }


def compare_speed(name, case, result, reference, criteria):
    gate_t = speed.assess({**result, "case": case}, case, criteria)
    gate_f = reference["assessment"]
    row = {"suite": "m12", "family": case["gate"], "reference": name}
    row["full"] = {"gate_passed": gate_f["passed"], **_speed_values(reference)}
    row["training"] = {"gate_passed": gate_t["passed"], **_speed_values(result)}
    checks = {"gate": gate_t["passed"] == gate_f["passed"] or gate_t["passed"] is True}
    for key, (floor, rel) in SPEED_BANDS.items():
        ok = within(row["training"][key], row["full"][key], floor, rel)
        if ok is not None:
            checks[key] = ok
    row["checks"] = checks
    row["passed"] = all(checks.values())
    return row


def trace_error(columns, trace, ref_columns, ref_trace):
    """RMS differences of the shared traces, sampled at the reference times."""
    out = {}
    t, rt = trace[:, 0], np.asarray(ref_trace)[:, 0]
    end = min(t[-1], rt[-1])
    keep = rt <= end
    for key in ("x", "velocity", "pitch_error_deg", "left_applied_Nm"):
        if key in columns and key in ref_columns:
            ours = np.interp(rt[keep], t, trace[:, columns.index(key)])
            theirs = np.asarray(ref_trace)[keep, ref_columns.index(key)]
            out[key] = float(np.sqrt(np.mean((ours - theirs) ** 2)))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--suite", choices=["all", "m10", "speed"], default="all")
    parser.add_argument("--case", action="append", help="run only these case names")
    parser.add_argument("--family", action="append", help="run only these M10 families")
    parser.add_argument(
        "--hip-belt", choices=["rigid", "spring"], help="override training drives.hip_belt (M14)"
    )
    parser.add_argument(
        "--knee-lash", choices=["static", "deadband"], help="override training drives.knee_lash"
    )
    args = parser.parse_args()
    p10, p13, gains, sensors = load()
    if args.hip_belt:
        p13["training"]["drives"]["hip_belt"] = args.hip_belt
    if args.knee_lash:
        p13["training"]["drives"]["knee_lash"] = args.knee_lash
    library = build_candidate(p10, gains, ROOT / native(p10)["firmware_build"])
    accepted = json.loads(ACCEPTANCE.read_text())["controllers"]["current"]
    full_cases = accepted["cases"]
    full_columns = [
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
    cfg = json.loads(speed.CONFIG.read_text())
    tasks = []
    if args.suite in ("all", "m10"):
        for name, case in acceptance_cases(p10, p13, sensors).items():
            ref = case.get("reference", name)
            if ref in full_cases and modeled(case.get("disturbance"), p13):
                tasks.append(("m10", case))
    if args.suite in ("all", "speed"):
        for name, case in speed.definitions(cfg).items():
            if (SPEED / "results" / f"{name}.json").exists():
                tasks.append(("speed", case))
    if args.case:
        tasks = [t for t in tasks if t[1]["name"] in args.case]
    if args.family:
        tasks = [t for t in tasks if t[0] == "m10" and t[1]["family"] in args.family]
    if not tasks:
        parser.error("no matching cases")
    output = args.output
    (output / "traces").mkdir(parents=True, exist_ok=True)
    rows = {}
    payloads = [(s, c, p10, p13, gains, sensors, str(library)) for s, c in tasks]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(task, x) for x in payloads]
        for future in as_completed(futures):
            suite, name, result, columns, trace = future.result()
            case = next(c for s, c in tasks if c["name"] == name and s == suite)
            if suite == "speed":
                reference = json.loads((SPEED / "results" / f"{name}.json").read_text())
                reference["assessment"] = speed.assess(
                    reference, reference["case"], cfg["criteria"]
                )
                row = compare_speed(name, case, result, reference, cfg["criteria"])
                ref = np.load(SPEED / "traces" / f"{name}.npz")
                row["trace_rms"] = trace_error(
                    columns, trace, [str(c) for c in ref["columns"]], ref["trace"]
                )
            else:
                reference = full_cases[case.get("reference", name)]
                row = compare_acceptance(name, case, result, reference)
                row["trace_rms"] = trace_error(columns, trace, full_columns, reference["trace"])
            row["failure"] = result.get("failure")
            rows[f"{suite}/{name}"] = row
            np.savez_compressed(
                output / "traces" / f"{suite}_{name}.npz",
                columns=np.array(columns),
                trace=trace.astype(np.float32),
            )
            print(name, "pass" if row["passed"] else "FAIL", row["failure"] or "", flush=True)
    rows = dict(sorted(rows.items()))
    failed = sorted(n for n, r in rows.items() if not r["passed"])
    summary = {
        "scope": "M13 training plant with the M10 C core and sensor model, against the full M10 plant's saved results; training-plant results, not acceptance",
        "training_plant": p13["training"],
        "training_plant_sha256": hashlib.sha256(
            json.dumps(p13["training"], sort_keys=True).encode()
        ).hexdigest(),
        "bands": {"m10": BANDS, "m12": SPEED_BANDS},
        "skipped": "belt variants the training plant does not represent: "
        + (
            "none"
            if {"hip_backlash_deg", "knee_backlash_deg"} <= belt_keys(p13)
            else "lash and stiffness (rigid hip belts, no lash dead band)"
        ),
        "cases": len(rows),
        "failed": failed,
        "failed_checks": {
            n: sorted(k for k, v in rows[n]["checks"].items() if not v) for n in failed
        },
        "passed": not failed,
    }
    (output / "cases.json").write_text(json.dumps(rows, indent=2, default=str) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, default=str) + "\n")
    print(json.dumps({k: summary[k] for k in ("cases", "failed_checks", "passed")}, indent=2))


if __name__ == "__main__":
    main()
