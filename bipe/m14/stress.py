"""M14 phase B: the full-plant stress set.

    python -m bipe.m14.stress [--checkpoint PATH] [--case NAME ...] [--workers 4]

26 cases on the full M10 plant: three scenarios (a 2.5 N s push forward and back
while holding, a drive up a 10 degree slope) under eight draws from inside the
training stress ranges (stratified: each parameter's eight values cover its range in
eighths, in a seeded random order), plus the two pushes on the nominal plant. The
parameters:
- RO40 and GL40 torque constant (with their torque ratings) x 0.8 or 1.2;
- joint, motor and wheel Coulomb and viscous friction x 0.5 or 1.5;
- RO40 FOC lag x 1 or 2, GL40 FOC lag 0.8 or 37.5 ms;
- pack 19.8 or 25.2 V: the GL40 voltage limit is 0.95 x pack and the RO40 no-load
  speed scales with pack / 25.2 V (no sag: the full plant has no bus model);
- chassis COM x and z +-5 mm;
- gyro and accelerometer bias +-1 mrad/s and +-0.1 m/s^2;
- transport delay 22.5 ms (1.5 x the acceptance's 15 ms) with noise x 2;
- hip and knee belt lash and stiffness at the belt-range corners.

Until 2 November 2026 the eight draws were corners (every parameter at a range end at
once); ``--corners`` keeps them. The feasibility check (docs/milestones/14, "Seed
results and stress feasibility") found corners where every controller falls without
a push, so the gate moved to interior draws; with the 22.5 ms delay a 3.75 N s push
reaches the 20 degree pitch limit about 70 ms after it lands, so the pushes moved to
the acceptance suite's 2.5 N s (Pranay, 2 November 2026).

Without ``--checkpoint`` the classical controller runs, for reference; with it the
policy runs in its place (bipe.m13.policy_control.policy_in_loop). Results go to
artifacts/m14/stress_classical or artifacts/m14/stress_<run>.
"""

import argparse
import copy
import json
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import numpy as np

from ..moving_experiment import simulate
from ..parameters import ROOT, load_parameters
from ..profiles import native
from ..robustness import command, physical_metrics
from ..robustness_control import build_candidate

SCENARIOS = {
    "push_fwd": {"family": "push", "mode": "hold", "duration": 16, "push_Ns": 2.5},
    "push_back": {"family": "push", "mode": "hold", "duration": 16, "push_Ns": -2.5},
    "slope_up": {"family": "slope", "mode": "up", "duration": 20, "slope_deg": 10},
}
RANGES = {
    "torque_scale": (0.8, 1.2),
    "friction_scale": (0.5, 1.5),
    "hip_lag_scale": (1.0, 2.0),
    "hub_lag_s": (0.0008, 0.0375),
    "pack_V": (19.8, 25.2),
    "com_x_m": (-0.005, 0.005),
    "com_z_m": (-0.005, 0.005),
    "gyro_bias_rad_s": (-0.001, 0.001),
    "accel_bias_m_s2": (-0.1, 0.1),
    "hip_backlash_deg": (0.1125, 0.45),
    "knee_backlash_deg": (0.225, 0.9),
    "hip_belt_stiffness_Nm_rad": (118.9, 713.3),
    "knee_belt_stiffness_Nm_rad": (23.5, 140.9),
}
DRAWS, SEED = 8, 20261030


def corners():
    """Eight draws, each parameter at its low or high end, balanced per parameter."""
    rng = np.random.default_rng(SEED)
    ends = {k: rng.permutation([0, 1] * (DRAWS // 2)) for k in RANGES}
    return [{k: RANGES[k][int(ends[k][i])] for k in RANGES} for i in range(DRAWS)]


def interior():
    """Eight draws inside the ranges, stratified per parameter."""
    rng = np.random.default_rng(SEED)
    cells = {k: (rng.permutation(DRAWS) + rng.random(DRAWS)) / DRAWS for k in RANGES}
    return [{k: lo + (hi - lo) * float(cells[k][i]) for k, (lo, hi) in RANGES.items()}
            for i in range(DRAWS)]  # fmt: skip


def stressed(p, sensors, c):
    """The full plant's parameters and sensor config at corner ``c``."""
    p, sensors = copy.deepcopy(p), copy.deepcopy(sensors)
    hm, gl, fr, motor = (
        p["physics"]["hip_motor"],
        p["physics"]["hub_motor"],
        p["physics"]["friction"],
        p["motor"],
    )
    for key in ("peak_torque_motor_Nm", "continuous_torque_motor_Nm", "torque_constant_Nm_A"):
        hm[key] *= c["torque_scale"]
    for key in ("peak_torque_Nm", "rated_torque_Nm", "torque_constant_Nm_A"):
        gl[key] *= c["torque_scale"]
    for key in ("peak_torque_per_wheel", "continuous_torque_per_wheel"):
        motor[key] *= c["torque_scale"]
    for block, keys in (
        (hm, ("viscous_motor_Nm_s_rad", "coulomb_motor_Nm")),
        (fr, ("hip_viscous_Nm_s_rad", "knee_viscous_Nm_s_rad", "knee_bearing_coulomb_Nm",
              "wheel_viscous_Nm_s_rad", "wheel_coulomb_Nm")),
        (motor, ("viscous_friction", "coulomb_friction")),
    ):  # fmt: skip
        for key in keys:
            block[key] *= c["friction_scale"]
    hm["FOC_time_constant_s"] *= c["hip_lag_scale"]
    gl["FOC_time_constant_s"] = motor["torque_time_constant"] = c["hub_lag_s"]
    gl["drive_voltage_limit_V"] = 0.95 * c["pack_V"]
    hm["no_load_speed_motor_rad_s"] *= c["pack_V"] / 25.2
    com = p["rigid_bodies"]["chassis"]["com_m"]
    com[0] += c["com_x_m"]
    com[2] += c["com_z_m"]
    sensors["transport_delay_s"] = 0.0225
    sensors["gyro_noise_std_rad_s"] *= 2
    sensors["accel_noise_std_m_s2"] *= 2
    sensors["gyro_residual_bias_rad_s"] = (
        np.asarray(sensors["gyro_residual_bias_rad_s"]) + c["gyro_bias_rad_s"]
    ).tolist()
    sensors["accel_bias_m_s2"] = (
        np.asarray(sensors["accel_bias_m_s2"]) + c["accel_bias_m_s2"]
    ).tolist()
    return p, sensors


def cases(draws="interior"):
    out = {}
    tag = "c" if draws == "corners" else "d"
    for name in ("push_fwd", "push_back"):
        s = SCENARIOS[name]
        out[f"{name}_nominal"] = {
            "name": f"{name}_nominal", "family": s["family"], "mode": s["mode"],
            "duration": s["duration"], "corner": None,
            "disturbance": {"push_Ns": s["push_Ns"], "initial_posture_deg": 40},
        }  # fmt: skip
    for i, c in enumerate(corners() if draws == "corners" else interior()):
        for name, s in SCENARIOS.items():
            disturbance = {k: c[k] for k in RANGES if "belt" in k or "backlash" in k}
            if "push_Ns" in s:
                disturbance.update(push_Ns=s["push_Ns"], initial_posture_deg=40)
            else:
                disturbance["slope_deg"] = s["slope_deg"]
            out[f"{name}_{tag}{i}"] = {
                "name": f"{name}_{tag}{i}",
                "family": s["family"],
                "mode": s["mode"],
                "duration": s["duration"],
                "corner": c,
                "disturbance": disturbance,
            }
    return out


def run(payload):
    p, gains, sensors, library, case = payload
    if case["corner"] is None:
        plant, config = copy.deepcopy(p), copy.deepcopy(sensors)
    else:
        plant, config = stressed(p, sensors, case["corner"])
    result = simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=p,
        plant=plant,
        gains=gains,
        sensor_config=config,
        implementation="c",
        firmware_library=library,
        disturbance=case["disturbance"],
        command_function=partial(command, case=case, nominal=p),
    )
    metrics = physical_metrics(result)
    metrics["policy_steps"] = result.get("policy_steps")
    return case["name"], metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--corners", action="store_true", help="the pre-2 November corner set")
    args = parser.parse_args()
    p = load_parameters(profile="m10")
    profile = native(p)
    gains = json.loads((ROOT / profile["controller"]).read_text())
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    library = str(build_candidate(p, gains, ROOT / profile["firmware_build"]))
    run_name = args.checkpoint.resolve().parent.name if args.checkpoint else "classical"
    output = args.output or ROOT / f"artifacts/m14/stress_{run_name}"
    output.mkdir(parents=True, exist_ok=True)
    draws = "corners" if args.corners else "interior"
    todo = {k: v for k, v in cases(draws).items() if not args.case or k in args.case}
    payloads = [(p, gains, sensors, library, c) for c in todo.values()]

    def execute():
        with ProcessPoolExecutor(args.workers) as pool:
            return dict(pool.map(run, payloads))

    multiprocessing.set_start_method("fork", force=True)
    if args.checkpoint:
        from ..m13.policy_control import policy_in_loop

        with policy_in_loop(args.checkpoint, runners=(__name__,)):
            results = execute()
    else:
        results = execute()
    rows = {k: {"corner": todo[k]["corner"], **results[k]} for k in todo}
    falls = [k for k, r in rows.items() if not r["physical_pass"]]
    summary = {
        "controller": str(args.checkpoint) if args.checkpoint else "classical",
        "draws": draws,
        "cases": len(rows),
        "falls": len(falls),
        "fall_rate": len(falls) / len(rows),
        "recovered": sum(bool(r["recovered"]) for r in rows.values()),
        "fallen": sorted(falls),
    }
    (output / "cases.json").write_text(json.dumps(rows, indent=2, default=float) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
