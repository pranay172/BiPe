"""M11 phase A: electrical requirements from the M10 simulation.

Runs a representative set of M10 acceptance and robustness cases exactly as
the acceptance runs them (C controller, sensor models), with an observer that
records per-motor electrical quantities every physics step. The observer only
reads the plant, so the dynamics are unchanged.

Electrical model, per motor i (the M10 screen conventions, config/m10-screen-assumptions.json):
- shaft power P = motor torque x rotor speed;
- copper loss 0.75 R_ll I^2 (peak sinusoidal phase current), R_ll scaled to the
  scenario's copper temperature;
- driver DC power (P + Cu) / eta when motoring, (P + Cu) x eta when regenerating;
- phase voltage demand |Ke w + R_ll I| with signed current (the DC-equivalent the
  plant's GL40 voltage limit uses), compared with each drive's voltage limit
  (16 V for the GL40 hubs, the low pack voltage for the RO40s).
Bus power is the sum over drivers plus the auxiliary load; bus current is that
power at the stated pack voltage.
"""

import argparse
import copy
import json
import math
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from ..moving_validate import jobs
from ..parameters import ROOT, load_parameters
from ..profiles import native
from ..robustness import matrix, physical_metrics, run_case
from ..robustness_control import build_candidate, equivalence

MOTORS = ["left_hip", "left_knee", "right_hip", "right_knee", "left_hub", "right_hub"]

CASES = [
    ("sensors_hold", "quiet hold, 60 s (runtime baseline)"),
    ("sensors_drive", "drive +-0.3 m/s with starts and stops"),
    ("sensors_turn", "turn +-0.4 rad/s"),
    ("sensors_crouch", "crouch cycle"),
    ("sensors_combined", "drive, turn and crouch together"),
    ("sensors_push", "acceptance push"),
    ("mass_plus_15pct_combined", "combined with 15% more mass"),
    ("slope_up_10", "drive up a 10 deg slope"),
    ("slope_down_10", "drive down a 10 deg slope"),
    ("slope_hold_10", "hold on a 10 deg slope"),
    ("push_hold_+2.5", "largest matrix push, holding"),
    ("push_hold_-2.5", "largest matrix push, holding"),
    ("push_drive_+2.5", "largest matrix push, driving"),
    ("push_drive_-2.5", "largest matrix push, driving"),
    ("push_low_+2.5", "largest matrix push, low posture"),
    ("push_low_-2.5", "largest matrix push, low posture"),
    ("backlash_crouch_0.6", "crouch with 0.6 deg lash (belt bounce)"),
]

WINDOWS_S = [0.01, 0.1, 1.0]
COPPER_ALPHA_PER_K = 0.00393


def model_config(p):
    """Electrical model inputs, with their sources, saved with every run."""
    screen = json.loads((ROOT / native(p)["electrical"]).read_text())
    hm, gl = p["physics"]["hip_motor"], p["physics"]["hub_motor"]
    ke_hip = 60 / (2 * math.pi * 140)
    return {
        "screen": native(p)["electrical"],
        "phase_to_phase_resistance_ohm_20C": screen["phase_to_phase_resistance_ohm"],
        "back_emf_V_s_rad": [ke_hip] * 4 + [gl["back_emf_V_s_rad"]] * 2,
        "back_emf_basis": "RO40 KV140: 1/KV = 60/(2 pi 140) V s/rad (model name in physics.hip_motor); "
        "GL40: physics.hub_motor.back_emf_V_s_rad (16 V at the catalog no-load speed)",
        "peak_phase_current_A": [hm["peak_current_A"]] * 4 + [gl["peak_current_A"]] * 2,
        "drive_voltage_limit_V": [19.2] * 4 + [gl["drive_voltage_limit_V"]] * 2,
        "drive_voltage_limit_basis": "RO40: the low pack voltage (DC-equivalent; no driver "
        "voltage cap is set); GL40: physics.hub_motor.drive_voltage_limit_V",
        "auxiliary_power_W": screen["auxiliary_power_W_assumption"],
        "auxiliary_basis": "M10 screen assumption (host, IMU, LEDs); phase C replaces it with catalog figures",
        "pack_capacity_Ah": screen["pack_capacity_Ah"],
        "pack_nominal_V": screen["pack_nominal_V"],
        "pack_full_V": screen["pack_full_V"],
        "pack_low_V": 19.2,
        "pack_low_basis": "3.2 V per cell, the M4 arm minimum (config/electronics.json limits.bus_minimum_arm_V)",
        "usable_depth_of_discharge": 0.8,
        "scenarios": {
            "draw": {
                "inverter_efficiency": 0.85,
                "copper_C": 125,
                "basis": "Conservative for bus draw: low inverter efficiency, hot copper",
            },
            "regen": {
                "inverter_efficiency": 0.95,
                "copper_C": 20,
                "basis": "Conservative for regeneration into the bus: high efficiency, cold copper",
            },
        },
        "windows_s": WINDOWS_S,
        "copper_alpha_per_K": COPPER_ALPHA_PER_K,
    }


class ElectricalRecorder:
    """Observer for bipe.moving_experiment.simulate; records every physics step."""

    def __init__(self, config):
        self.c = config
        self.rows = []

    def __call__(self, t, dt, drives, d):
        self.dt = dt
        speed = d.qvel[drives.rv]
        self.rows.append(
            np.r_[t, drives.phase_current, drives.motor_torque * speed, speed, drives.motor_torque]
        )

    def arrays(self):
        a = np.asarray(self.rows)
        return {
            "time": a[:, 0],
            "current": a[:, 1:7],
            "shaft_power": a[:, 7:13],
            "speed": a[:, 13:19],
            "torque": a[:, 19:25],
        }

    def bus(self, a, scenario):
        s = self.c["scenarios"][scenario]
        r = np.asarray(self.c["phase_to_phase_resistance_ohm_20C"]) * (
            1 + COPPER_ALPHA_PER_K * (s["copper_C"] - 20)
        )
        copper = 0.75 * r * a["current"] ** 2
        terminal = a["shaft_power"] + copper
        eta = s["inverter_efficiency"]
        dc = np.where(terminal > 0, terminal / eta, terminal * eta)
        voltage = abs(np.asarray(self.c["back_emf_V_s_rad"]) * a["speed"] + r * a["current"])
        return dc, dc.sum(axis=1) + self.c["auxiliary_power_W"], copper, voltage

    def summary(self):
        a = self.arrays()
        dt, duration = self.dt, a["time"][-1] - a["time"][0] + self.dt
        c = self.c
        out = {"duration_s": duration, "physics_dt_s": dt}
        i = abs(a["current"])
        peak = np.asarray(c["peak_phase_current_A"])
        limit = np.asarray(c["drive_voltage_limit_V"])
        draw_voltage = self.bus(a, "draw")[3]
        out["motors"] = {
            name: {
                "phase_current_peak_A": float(i[:, k].max()),
                "phase_current_rms_A": float(np.sqrt((i[:, k] ** 2).mean())),
                "time_at_current_limit_fraction": float((i[:, k] >= 0.99 * peak[k]).mean()),
                "time_at_voltage_limit_fraction": float(
                    (draw_voltage[:, k] >= 0.98 * limit[k]).mean()
                ),
                "rotor_speed_peak_rad_s": float(abs(a["speed"][:, k]).max()),
            }
            for k, name in enumerate(MOTORS)
        }
        for scenario in c["scenarios"]:
            dc, bus, copper, voltage = self.bus(a, scenario)
            for k, name in enumerate(MOTORS):
                out["motors"][name].setdefault(scenario, {}).update(
                    driver_dc_power_peak_W=float(dc[:, k].max()),
                    driver_dc_power_min_W=float(dc[:, k].min()),
                    phase_voltage_demand_peak_V=float(voltage[:, k].max()),
                    copper_loss_mean_W=float(copper[:, k].mean()),
                )
            row = {
                "bus_power_peak_W": float(bus.max()),
                "bus_power_min_W": float(bus.min()),
                "bus_power_mean_W": float(bus.mean()),
                "bus_energy_J": float(bus.sum() * dt),
                "regenerated_energy_J": float(-np.minimum(bus, 0).sum() * dt),
                "time_regenerating_fraction": float((bus < 0).mean()),
                "phase_voltage_demand_peak_V": float(voltage.max()),
            }
            for v_name in ["pack_nominal_V", "pack_low_V"]:
                current = bus / c[v_name]
                windows = {}
                for w in WINDOWS_S:
                    n = max(1, round(w / dt))
                    if n > len(current):
                        continue
                    kernel = np.ones(n) / n
                    mean = np.convolve(current, kernel, mode="valid")
                    windows[f"{w:g}s"] = {"max_A": float(mean.max()), "min_A": float(mean.min())}
                row[v_name.replace("_V", "")] = {
                    "bus_current_peak_A": float(current.max()),
                    "bus_current_min_A": float(current.min()),
                    "bus_current_rms_A": float(np.sqrt((current**2).mean())),
                    "bus_current_mean_A": float(current.mean()),
                    "windows": windows,
                    "pack_C_rate_peak": float(current.max() / c["pack_capacity_Ah"]),
                }
            usable_J = (
                c["pack_capacity_Ah"] * c["pack_nominal_V"] * 3600 * c["usable_depth_of_discharge"]
            )
            row["runtime_at_this_load_min"] = (
                float(usable_J / row["bus_power_mean_W"] / 60)
                if row["bus_power_mean_W"] > 0
                else None
            )
            out[scenario] = row
        return out

    def trace(self, motor_step_s=0.005, bus_step_s=0.001):
        """Decimated float32 traces: per-motor channels at the control rate, bus power
        at 1 ms. Peaks and current windows are computed at the physics step in summary()."""
        a = self.arrays()
        return decimate(
            {
                "time": a["time"],
                "phase_current_A": a["current"],
                "shaft_power_W": a["shaft_power"],
                "rotor_speed_rad_s": a["speed"],
                "phase_voltage_demand_draw_V": self.bus(a, "draw")[3],
                "bus_power_draw_W": self.bus(a, "draw")[1],
                "bus_power_regen_W": self.bus(a, "regen")[1],
            },
            self.dt,
            motor_step_s,
            bus_step_s,
        )


def decimate(full, dt, motor_step_s=0.005, bus_step_s=0.001):
    m, b = max(1, round(motor_step_s / dt)), max(1, round(bus_step_s / dt))
    out = {k: np.asarray(v)[::m].astype(np.float32) for k, v in full.items() if "bus" not in k}
    out["bus_time"] = np.asarray(full["time"])[::b].astype(np.float32)
    out.update({k: np.asarray(v)[::b].astype(np.float32) for k, v in full.items() if "bus" in k})
    out["motor_dt_s"], out["bus_dt_s"] = np.float32(m * dt), np.float32(b * dt)
    return out


def task(payload):
    nominal, gains, sensors, library, case, config, output = payload
    recorder = ElectricalRecorder(config)
    run = run_case(nominal, gains, sensors, library, case, observer=recorder)
    result = {
        "case": case["name"],
        "physical": physical_metrics(run),
        "failure": run["failure"],
        "electrical": recorder.summary(),
    }
    np.savez_compressed(Path(output) / "traces" / f"{case['name']}.npz", **recorder.trace())
    return case["name"], result


def all_cases(p, sensors):
    named = {c["name"]: c for c in matrix(p)}
    for job in jobs(sensors, "c", p):
        named[job["name"]] = {"name": job["name"], "family": "acceptance", "job": job}
    return named


def aggregate(results, config):
    """Requirements: the worst value over cases, with the case that set it."""

    def worst(values, pick=max):
        name, value = pick(values.items(), key=lambda kv: kv[1])
        return {"value": value, "case": name}

    req = {"motors": {}, "bus": {}}
    for m in MOTORS:
        rows = {n: r["electrical"]["motors"][m] for n, r in results.items()}
        req["motors"][m] = {
            "phase_current_peak_A": worst({n: r["phase_current_peak_A"] for n, r in rows.items()}),
            "phase_current_rms_A": worst({n: r["phase_current_rms_A"] for n, r in rows.items()}),
            "time_at_current_limit_fraction": worst(
                {n: r["time_at_current_limit_fraction"] for n, r in rows.items()}
            ),
            "phase_voltage_demand_peak_V": worst(
                {n: r["draw"]["phase_voltage_demand_peak_V"] for n, r in rows.items()}
            ),
            "driver_dc_power_peak_W": worst(
                {n: r["draw"]["driver_dc_power_peak_W"] for n, r in rows.items()}
            ),
            "time_at_voltage_limit_fraction": worst(
                {n: r["time_at_voltage_limit_fraction"] for n, r in rows.items()}
            ),
            "driver_regen_power_peak_W": worst(
                {n: max(0.0, -r["regen"]["driver_dc_power_min_W"]) for n, r in rows.items()}
            ),
        }
    e = {n: r["electrical"] for n, r in results.items()}
    for v in ["pack_nominal", "pack_low"]:
        req["bus"][v] = {
            "current_peak_A": worst({n: r["draw"][v]["bus_current_peak_A"] for n, r in e.items()}),
            "current_rms_A": worst({n: r["draw"][v]["bus_current_rms_A"] for n, r in e.items()}),
            "regen_current_peak_A": worst(
                {n: max(0.0, -r["regen"][v]["bus_current_min_A"]) for n, r in e.items()}
            ),
            "windows_max_A": {
                w: worst(
                    {
                        n: r["draw"][v]["windows"][w]["max_A"]
                        for n, r in e.items()
                        if w in r["draw"][v]["windows"]
                    }
                )
                for w in [f"{w:g}s" for w in WINDOWS_S]
            },
            "pack_C_rate_peak": worst({n: r["draw"][v]["pack_C_rate_peak"] for n, r in e.items()}),
        }
    req["bus"]["power_peak_W"] = worst({n: r["draw"]["bus_power_peak_W"] for n, r in e.items()})
    req["bus"]["power_min_W"] = worst(
        {n: r["regen"]["bus_power_min_W"] for n, r in e.items()}, pick=min
    )
    req["bus"]["regen_power_peak_W"] = worst(
        {n: max(0.0, -r["regen"]["bus_power_min_W"]) for n, r in e.items()}
    )
    req["bus"]["regen_energy_J"] = worst(
        {n: r["regen"]["regenerated_energy_J"] for n, r in e.items()}
    )
    req["runtime_min"] = {
        n: {
            "draw": r["draw"]["runtime_at_this_load_min"],
            "regen": r["regen"]["runtime_at_this_load_min"],
        }
        for n, r in e.items()
    }
    req["mean_power_W"] = {
        n: {"draw": r["draw"]["bus_power_mean_W"], "regen": r["regen"]["bus_power_mean_W"]}
        for n, r in e.items()
    }
    return req


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/m11/requirements")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--case", action="append", help="Run only these case names")
    args = parser.parse_args()
    p = load_parameters(profile="m10")
    profile = native(p)
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    gains = json.loads((ROOT / profile["controller"]).read_text())
    named = all_cases(p, sensors)
    selected = [(n, why) for n, why in CASES if not args.case or n in args.case]
    missing = [n for n, _ in selected if n not in named]
    if missing:
        parser.error(f"unknown cases {missing}")
    nominal = copy.deepcopy(p)
    library = build_candidate(nominal, gains, ROOT / ".venv/m8-build/current")
    check = equivalence(nominal, gains, library)
    if not check["passed"]:
        raise ValueError(f"C/Python mismatch: {check}")
    config = model_config(p)
    (args.output / "traces").mkdir(parents=True, exist_ok=True)
    (args.output / "config.json").write_text(
        json.dumps(
            {
                "scope": "M10 simulation under the stated electrical model; not a hardware measurement",
                "model": config,
                "cases": [
                    {"name": n, "purpose": why, "definition": named[n]} for n, why in selected
                ],
                "controller": profile["controller"],
                "sensors": profile["sensors"],
                "equivalence": check,
            },
            indent=2,
            default=str,
        )
        + "\n"
    )
    results = {}
    payloads = [
        (nominal, gains, sensors, str(library), named[n], config, str(args.output))
        for n, _ in selected
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(task, x) for x in payloads]):
            name, result = future.result()
            results[name] = result
            draw = result["electrical"]["draw"]
            print(
                name,
                result["failure"],
                f"peak {draw['pack_low']['bus_current_peak_A']:.2f} A",
                f"mean {draw['bus_power_mean_W']:.1f} W",
                flush=True,
            )
    results = dict(sorted(results.items()))
    (args.output / "cases.json").write_text(json.dumps(results, indent=2) + "\n")
    requirements = aggregate(results, config)
    (args.output / "requirements.json").write_text(json.dumps(requirements, indent=2) + "\n")
    print(json.dumps(requirements["bus"], indent=2))


if __name__ == "__main__":
    main()
