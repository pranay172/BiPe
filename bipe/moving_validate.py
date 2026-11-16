"""Nominal, sensing, disturbance, variation, and convergence acceptance gate."""

import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

from .articulated import JOINTS, build, reset
from .kinematics import com_and_axle, posture
from .locomotion import wrap
from .moving_experiment import save_run, simulate
from .parameters import ROOT, load_parameters, total_mass
from .profiles import CHOICES, native, plant


def geometry_check(p=None):
    p = load_parameters() if p is None else p
    if native(p):
        return plant(p).geometry_check(p)
    m, d = build(p)
    errors = []
    folded_backward = True
    for angle in [25, 40, 60, 65]:
        reset(m, d, p, angle)
        R = d.xmat[m.body("chassis").id].reshape(3, 3)
        origin = d.xpos[m.body("chassis").id]
        com, axle = com_and_axle(p, posture(angle))
        physical_com = (R.T @ (d.subtree_com[m.body("chassis").id] - origin))[[0, 2]]
        physical_axle = (R.T @ (d.site_xpos[m.site("left_axle").id] - origin))[[0, 2]]
        errors.append(float(max(abs(com - physical_com).max(), abs(axle - physical_axle).max())))
        knee = R.T @ (d.geom_xpos[m.geom("left_knee_motor").id] - origin)
        folded_backward &= bool(knee[0] < 0 and m.geom_pos[m.geom("left_eye").id, 0] > 0)
    result = {
        "total_mass_kg": float(sum(m.body_mass)),
        "generalized_velocities": m.nv,
        "driven_hinges": m.nu,
        "equality_constraints": m.neq,
        "max_kinematic_error_m": max(errors),
        "knees_fold_backward": folded_backward,
    }
    result["passed"] = bool(
        m.nv == 12
        and m.nu == 6
        and m.neq == 0
        and abs(sum(m.body_mass) - total_mass(p)) < 1e-10
        and max(errors) < 1e-8
        and folded_backward
    )
    return result


def jobs(sensors=None, implementation="python", nominal=None):
    nominal = load_parameters() if nominal is None else nominal
    sensors = copy.deepcopy(sensors or json.loads((ROOT / "config/sensors.json").read_text()))
    cases = [
        ("hold", 60, 0),
        ("tilt_positive", 60, 5),
        ("tilt_negative", 60, -5),
        ("drive", 20, 0),
        ("turn", 15, 0),
        ("crouch", 36, 0),
        ("combined", 36, 0),
        ("push", 16, 0),
    ]
    result = []
    for mode in ["ideal", "sensors"]:
        for case, duration, offset in cases:
            result.append(
                {
                    "name": mode + "_" + case,
                    "scenario": case,
                    "duration": duration,
                    "offset": offset,
                    "sensing": mode,
                }
            )
    result.append(
        {"name": "unpowered", "scenario": "unpowered", "duration": 5, "offset": 5, "enabled": False}
    )
    for seed in [sensors["seed"] + 1, sensors["seed"] + 2]:
        result.append(
            {
                "name": f"combined_seed_{seed}",
                "scenario": "combined",
                "duration": 36,
                "sensing": "sensors",
                "seed": seed,
            }
        )
    for variant in ["mass_plus_15pct", "friction_0p5", "wheel_lag_25ms", "delay_15ms_noise_x2"]:
        plant = copy.deepcopy(nominal)
        sensor_config = copy.deepcopy(sensors)
        if variant == "mass_plus_15pct":
            for row in plant["mass_budget"]:
                row["mass_each"] *= 1.15
            for row in plant.get("rigid_bodies", {}).values():
                row["mass_kg"] *= 1.15
                row["inertia_kg_m2"] = (np.asarray(row["inertia_kg_m2"]) * 1.15).tolist()
        elif variant == "friction_0p5":
            plant["contact"]["sliding_friction"] = 0.5
        elif variant == "wheel_lag_25ms":
            plant["motor"]["torque_time_constant"] = 0.025
            # The M10 hub drives read the FOC constant from the physics block.
            if "hub_motor" in plant.get("physics", {}):
                plant["physics"]["hub_motor"]["FOC_time_constant_s"] = 0.025
        else:
            sensor_config["transport_delay_s"] = 0.015
            sensor_config["gyro_noise_std_rad_s"] *= 2
            sensor_config["accel_noise_std_m_s2"] *= 2
        for case, duration in [("drive", 20), ("combined", 36)]:
            result.append(
                {
                    "name": variant + "_" + case,
                    "scenario": case,
                    "duration": duration,
                    "sensing": "sensors",
                    "plant": plant,
                    "sensor_config": sensor_config,
                }
            )
    for job in result:
        job.setdefault("sensor_config", copy.deepcopy(sensors))
        job["implementation"] = implementation
        job["nominal"] = nominal
    return result


def execute_job(job, output):
    job = job.copy()
    name = job.pop("name")
    run = simulate(**job)
    return name, save_run(run, output / name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=CHOICES, default="m4")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--output")
    parser.add_argument("--sensor-config")
    parser.add_argument("--implementation", choices=["python", "c"], default="python")
    args = parser.parse_args()
    nominal = load_parameters(profile=args.profile)
    if args.output is None:
        args.output = (
            native(nominal)["moving"] if native(nominal) else "artifacts/milestone-3/acceptance"
        )
    if args.sensor_config is None:
        args.sensor_config = (
            native(nominal)["sensors"] if native(nominal) else "config/sensors.json"
        )
    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    (output / "acceptance.json").write_text(
        json.dumps({"passed": False, "status": "validation running"}) + "\n"
    )
    reports = {}
    sensors = json.loads((ROOT / args.sensor_config).read_text())
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for task in as_completed(
            [
                pool.submit(execute_job, job, output)
                for job in jobs(sensors, args.implementation, nominal)
            ]
        ):
            name, report = task.result()
            reports[name] = report
            print(f"{name}: {'PASS' if report['passed'] else 'FAIL'}", flush=True)
    fine = simulate(
        "combined",
        36,
        sensing="sensors",
        physics_dt=nominal["simulation"]["physics_dt"] / 2,
        sensor_config=sensors,
        implementation=args.implementation,
        nominal=nominal,
    )
    fine_report = save_run(fine, output / "combined_half_timestep")
    base = reports["sensors_combined"]
    convergence = {
        "half_timestep_passed": fine_report["passed"],
        "peak_pitch_difference_deg": abs(
            fine_report["max_pitch_error_deg"] - base["max_pitch_error_deg"]
        ),
        "final_heading_difference_deg": abs(
            math.degrees(wrap(math.radians(fine_report["final_yaw_deg"] - base["final_yaw_deg"])))
        ),
        "height_range_difference_m": abs(fine_report["height_range_m"] - base["height_range_m"]),
        "max_joint_peak_torque_difference_Nm": max(
            abs(fine_report["peak_torque_Nm"][n] - base["peak_torque_Nm"][n]) for n in JOINTS
        ),
        "max_joint_rms_torque_difference_Nm": max(
            abs(fine_report["rms_torque_Nm"][n] - base["rms_torque_Nm"][n]) for n in JOINTS
        ),
        "max_joint_peak_speed_difference_rad_s": max(
            abs(fine_report["peak_joint_speed_rad_s"][n] - base["peak_joint_speed_rad_s"][n])
            for n in JOINTS
        ),
    }
    convergence["passed"] = bool(
        fine_report["passed"]
        and convergence["peak_pitch_difference_deg"] < 1
        and convergence["final_heading_difference_deg"] < 2
        and convergence["height_range_difference_m"] < 0.003
        and convergence["max_joint_peak_torque_difference_Nm"] < 0.18
        and convergence["max_joint_rms_torque_difference_Nm"] < 0.05
        and convergence["max_joint_peak_speed_difference_rad_s"] < 0.3
    )
    geometry = geometry_check(nominal)
    expected_controller = json.loads(
        (
            ROOT
            / (
                native(nominal)["controller"]
                if native(nominal)
                else "config/controller_moving.json"
            )
        ).read_text()
    )
    consistent_controller = all(
        json.loads((output / name / "configuration.json").read_text())["controller"]
        == expected_controller
        for name in reports
    )
    consistent_controller &= (
        json.loads((output / "combined_half_timestep/configuration.json").read_text())["controller"]
        == expected_controller
    )
    result = {
        "passed": bool(
            geometry["passed"]
            and consistent_controller
            and convergence["passed"]
            and all(r["passed"] for r in reports.values())
        ),
        "implementation": args.implementation,
        "profile": args.profile,
        "geometry": geometry,
        "same_controller_across_suite": bool(consistent_controller),
        "controller": expected_controller,
        "sensor_configuration": sensors,
        "scenarios": dict(sorted(reports.items())),
        "convergence": convergence,
        "conditions": "Level floor, assumed hardware, 200 Hz body supervisor and 1 kHz local joint servos. Variations are tested separately, not all combinations.",
    }
    (output / "acceptance.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"passed": result["passed"], "report": str(output / "acceptance.json")}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
