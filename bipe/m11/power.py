"""M11 phase C: power path and protection calculations.

Reads m10.electronics in config/robot.json and the phase A requirements in
artifacts/m11/requirements/, and writes artifacts/m11/power/report.json.
Every check states its inputs; assumptions are labelled in the config.
"""

import argparse
import json
import math
from pathlib import Path

from ..parameters import ROOT

REQUIREMENTS = ROOT / "artifacts/m11/requirements"


def logic_budget(e):
    """5 V rail load (typical and peak) and the auxiliary bus power."""
    leds = e["eye_leds"]
    full_white = leds["count"] * leds["leds_each"] * leds["channels"] * leds["channel_current_A"]
    three_v3 = e["imu"]["supply_A_3V3"]
    v = e["vision_reservation"]
    rows = {
        "teensy_A": e["host"]["input_A_at_5V"],
        "three_v3_loads_A": three_v3,
        "can_transceiver_A": e["can_transceiver"]["supply_A_5V_dominant_max"],
        "eye_leds_capped_A": full_white * leds["firmware_brightness_cap"],
        "eye_leds_full_white_A": full_white,
        "vision_typical_A": v["typical_A_at_5V"] + v["camera_A_at_5V_assumption"],
        "vision_supply_A": v["supply_A_at_5V"],
    }
    base = rows["teensy_A"] + rows["three_v3_loads_A"] + rows["can_transceiver_A"]
    reg = e["regulator_5V"]
    typical = base + rows["eye_leds_capped_A"] + rows["vision_typical_A"]
    peak = base + rows["eye_leds_full_white_A"] + rows["vision_supply_A"]
    without_vision = base + rows["eye_leds_capped_A"]
    d = e["drivers"]
    logic = d["count"] * d["logic_power_W_each_assumption"]

    def bus_W(amps):
        return amps * reg["output_V"] / reg["efficiency_assumption"] + logic

    return {
        "loads": rows,
        "five_volt_typical_A": typical,
        "five_volt_peak_A": peak,
        "five_volt_without_vision_A": without_vision,
        "regulator": reg["model"],
        "regulator_peak_utilisation": peak / reg["continuous_A"],
        "regulator_passes": peak <= reg["continuous_A"],
        "driver_logic_W": logic,
        "auxiliary_bus_W": {
            "typical": bus_W(typical),
            "without_vision": bus_W(without_vision),
            "peak": bus_W(peak),
        },
    }


def runtime(e, budget, cases, config):
    """Phase A runtimes with the 6 W auxiliary assumption replaced by the budget."""
    old = config["model"]["auxiliary_power_W"]
    model = config["model"]
    usable = (
        model["pack_capacity_Ah"]
        * model["pack_nominal_V"]
        * 3600
        * model["usable_depth_of_discharge"]
    )
    out = {}
    for name, row in cases.items():
        mean = row["electrical"]["draw"]["bus_power_mean_W"]
        out[name] = {
            label: usable / (mean - old + aux) / 60
            for label, aux in budget["auxiliary_bus_W"].items()
            if label != "peak"
        }
    return {
        "basis": "draw scenario (0.85 inverter efficiency, 125 C copper), 80% of 0.85 Ah x 22.2 V",
        "replaced_auxiliary_W": old,
        "minutes": out,
        "range_typical_min": [
            min(r["typical"] for r in out.values()),
            max(r["typical"] for r in out.values()),
        ],
    }


def bus_capacitance(e):
    d = e["drivers"]
    nominal = d["count"] * d["bulk_capacitance_F_each"]
    return nominal, nominal * d["mlcc_dc_bias_retained_fraction_assumption"]


def soft_start(e, budget):
    """LM5060 output rise: dV/dt = I_gate / C_gate. Inrush into the nominal bus
    capacitance (the conservative case for inrush) and MOSFET energy (capacitor
    charge plus the logic loads that start during the ramp)."""
    s, pack = e["soft_start"], e["pack"]
    c_bus, _ = bus_capacitance(e)
    v = pack["full_V"]
    rows = {}
    for label, current in zip(["slowest", "typical", "fastest"], s["gate_charge_current_A"]):
        slew = current / s["gate_capacitor_F"]
        rise = v / slew
        inrush = c_bus * slew
        # Loads start near 9-10 V (regulator and driver logic) and draw constant power.
        load_W = budget["auxiliary_bus_W"]["typical"]
        start = 9.0
        load_J = load_W / slew * (v * math.log(v / start) - (v - start))
        cap_J = 0.5 * c_bus * v**2
        rows[label] = {
            "rise_time_s": rise,
            "slew_V_s": slew,
            "inrush_A": inrush,
            "mosfet_energy_J": cap_J + load_J,
            "mosfet_mean_power_W": (cap_J + load_J) / rise,
        }
    turn_off = (s["gate_capacitor_F"] + 3e-9) * (v + 12) / s["gate_discharge_current_A_off"]
    return {
        "bus_capacitance_nominal_F": c_bus,
        "ramps": rows,
        "turn_off_s_estimate": turn_off,
        "turn_off_basis": "GATE-to-GND capacitor plus about 3 nF MOSFET input capacitance, "
        "discharged from VIN + 12 V at IGATE-OFF 2.2 mA",
        "inrush_passes": max(r["inrush_A"] for r in rows.values()) < 1.0,
        "inrush_limit_basis": "Inrush below 1 A: under 15% of the 6.6 A phase A peak and far below the fuse rating",
    }


def lm5060_design(e, budget, req):
    """Timer, VDS threshold, gate drive and MOSFET linear-mode checks for the chosen parts.

    Start-up: the gate (C_gate plus the MOSFET capacitances) charges at the minimum gate
    current to the plateau, the output then follows over the full pack, and VGS finally
    rises to the highest VGS-status threshold. The timer must not reach its fault
    threshold first at the highest start-up charge current."""
    s = e["soft_start"]
    m, t, sense = s["mosfet"], s["timer"], s["sense"]
    c_t, c_g, v = s["timer_capacitor_F"], s["gate_capacitor_F"], e["pack"]["full_V"]
    i_g = min(s["gate_charge_current_A"])
    plateau = m["plateau_V_assumption"]
    to_plateau = plateau * (c_g + m["ciss_F_max"]) / i_g
    ramp = v * (c_g + m["crss_F_max"]) / i_g
    to_status = (max(t["vgs_status_threshold_V"]) - plateau) * (c_g + m["ciss_F_max"]) / i_g
    needed = to_plateau + ramp + to_status
    v_lo = t["fault_threshold_V"] * (1 - t["fault_threshold_tolerance_assumption"])
    v_hi = t["fault_threshold_V"] * (1 + t["fault_threshold_tolerance_assumption"])
    start_timeout = v_lo * c_t / max(t["start_up_charge_A"])
    overcurrent = [v_lo * c_t / max(t["fault_charge_A"]), v_hi * c_t / min(t["fault_charge_A"])]
    vds = [
        sense["resistor_ohm"] * min(sense["current_A"]) - sense["offset_V"],
        sense["resistor_ohm"] * max(sense["current_A"]) + sense["offset_V"],
    ]
    trip_min = vds[0] / m["rds_on_ohm_max_125C"]
    bus_peak = req["bus"]["pack_low"]["current_peak_A"]["value"]
    # Linear-mode power over the fastest ramp: capacitor charge plus constant-power loads above 9 V.
    c_bus, _ = bus_capacitance(e)
    inrush = c_bus * max(s["gate_charge_current_A"]) / c_g
    load_W = budget["auxiliary_bus_W"]["typical"]
    peak_W = max(
        (v - vo) * (inrush + (load_W / vo if vo >= 9.0 else 0.0))
        for vo in [v * k / 1000 for k in range(1001)]
    )
    vgs_on = min(s["gate_clamp_V"], 14.0)
    return {
        "start_up": {
            "to_plateau_s": to_plateau,
            "ramp_s": ramp,
            "to_vgs_status_s": to_status,
            "needed_s": needed,
            "timer_timeout_min_s": start_timeout,
            "margin": start_timeout / needed,
            "passes": start_timeout >= 1.5 * needed,
        },
        "overcurrent_delay_s": overcurrent,
        "vds_threshold_V": vds,
        "trip_current_min_A": trip_min,
        "trip_above_peak": trip_min >= 1.5 * bus_peak,
        "bus_peak_A": bus_peak,
        "vgs_on_V": vgs_on,
        "vgs_within_rating": vgs_on <= m["vgs_max_V"],
        "linear_mode_peak_W": peak_W,
        "linear_mode_junction_rise_K": peak_W * m["rth_jc_K_W"],
        "linear_mode_passes": peak_W <= 0.5 * m["single_pulse_power_W_10ms"],
        "basis": "Gate path: C_gate plus Ciss (to the plateau and after the ramp) or Crss (during "
        "the ramp) at IGATE 17 uA. The MOSFET single-pulse rating is for a 25 C case, so the "
        "check keeps half of it; the junction rise uses RthJC because the ramps last 18-33 ms",
    }


def uvlo_divider(e, rising_V=20.0, lower_ohm=4700.0):
    """LM5060 UVLO divider, including the 5.5 uA UVLO pull-down current.

    V_pin = (V_in / R_h - I_b) / (1 / R_h + 1 / R_l) at the rising threshold."""
    threshold, hysteresis, bias = 1.6, 0.18, 5.5e-6
    # Solve for R_h at the rising threshold.
    upper = (rising_V - threshold) / (threshold / lower_ohm + bias)

    def input_at(pin):
        return upper * (pin * (1 / upper + 1 / lower_ohm) + bias)

    falling = input_at(threshold - hysteresis)
    return {
        "upper_ohm": upper,
        "lower_ohm": lower_ohm,
        "rising_V": input_at(threshold),
        "falling_V": falling,
        "falling_V_per_cell": falling / e["pack"]["cells"],
        "divider_current_A_at_full": e["pack"]["full_V"] / (upper + lower_ohm),
        "placement": "The shell switch connects the top of the UVLO divider (and EN) to the pack, "
        "so the off-state drain is the LM5060 disabled current only (<15 uA)",
        "basis": "LM5060 UVLO 1.6 V rising, 180 mV hysteresis, 5.5 uA pull-down (typical)",
    }


def fuse_and_wiring(e, req):
    f, w, pack = e["fuse"], e["wiring"], e["pack"]
    bus = req["bus"]["pack_low"]
    r_pack = pack["cells"] * pack["internal_resistance_ohm_per_cell_assumption"]
    r_trunk = w["trunk"]["ohm_per_m"] * w["trunk"]["length_m"] * 2
    r_branch = w["branch"]["ohm_per_m"] * w["branch"]["length_m"] * 2
    r_conn = 4 * w["connector_ohm_each_assumption"]
    driver_peak = max(m["driver_dc_power_peak_W"]["value"] for m in req["motors"].values())
    branch_peak = driver_peak / pack["arm_minimum_V"]
    peak = bus["current_peak_A"]["value"]
    sag = peak * (r_pack + r_trunk + r_conn) + branch_peak * r_branch
    short = pack["full_V"] / (r_pack + r_trunk + r_conn)
    return {
        "fuse": f["model"],
        "carries_peak": peak < f["rating_A"],
        "carries_rms_with_75pct_derating": bus["current_rms_A"]["value"] < 0.75 * f["rating_A"],
        "peak_A": peak,
        "rms_A": bus["current_rms_A"]["value"],
        "worst_1s_mean_A": bus["windows_max_A"]["1s"]["value"],
        "short_circuit_A_estimate": short,
        "short_circuit_basis": "Full pack across the pack resistance (assumed per-cell value), the "
        "trunk and the connectors; this ignores the MOSFET, so it is an upper estimate",
        "interrupting_passes": short < f["interrupting_A"],
        "drop": {
            "pack_and_wiring_ohm": r_pack + r_trunk + r_conn,
            "branch_ohm": r_branch,
            "branch_peak_A": branch_peak,
            "sag_at_peak_V": sag,
            "driver_minimum_V_at_low_pack": pack["arm_minimum_V"] - sag,
            "driver_minimum_passes": pack["arm_minimum_V"] - sag > e["drivers"]["input_V"][0],
        },
        "trunk_power_loss_W_at_rms": bus["current_rms_A"]["value"] ** 2 * r_trunk,
    }


def overvoltage(e, p):
    """Regenerated energy into the bus capacitance alone (pack disconnected)."""
    _, c_derated = bus_capacitance(e)
    mass = p["cad"]["allocated_kg"]
    v0 = e["pack"]["full_V"]
    rows = {}
    for speed in [0.3, 1.0]:
        energy = 0.5 * mass * speed**2
        rows[f"{speed:g}_m_s"] = {
            "kinetic_energy_J": energy,
            "bus_V_without_flux_braking": math.sqrt(v0**2 + 2 * energy / c_derated),
        }
    settings = e["drivers"]["settings"]
    hub = p["physics"]["hub_motor"]
    radius = p["wheel"]["diameter"] / 2
    back_emf_speed = e["pack"]["arm_minimum_V"] / hub["back_emf_V_s_rad"] * radius
    return {
        "bus_capacitance_derated_F": c_derated,
        "pack_disconnected": rows,
        "flux_brake_point_V": settings["servo.max_voltage"]
        - settings["servo.flux_brake_margin_voltage"],
        "max_voltage_V": settings["servo.max_voltage"],
        "finding": "While the pack is plugged in it absorbs regenerated energy, even with the "
        "switch off: the high-side N-MOSFET's body diode conducts from the bus back to the pack "
        "(phase A: the bus never regenerates in operation). Only an unplugged pack or a blown fuse "
        "leaves the bus capacitance alone; braking energy would then lift it far above max_voltage, "
        "so flux braking (above the brake point) must absorb it.",
        "hub_back_emf_reaches_low_pack_at_m_s": back_emf_speed,
        "back_emf_basis": "Hub back-EMF Ke w equals the 19.2 V low pack voltage at this ground "
        "speed; below it a back-driven hub cannot charge the bus through the driver body diodes",
    }


def battery_monitor(e):
    b, pack = e["battery_monitor"], e["pack"]
    ratio = b["lower_ohm"] / (b["upper_ohm"] + b["lower_ohm"])
    lsb = b["adc_reference_V"] / 2 ** b["adc_bits"] / ratio
    return {
        "ratio": ratio,
        "pin_V_at_full": pack["full_V"] * ratio,
        "pin_within_reference": pack["full_V"] * ratio < b["adc_reference_V"],
        "bus_V_per_lsb": lsb,
        "source_impedance_ohm": b["upper_ohm"] * b["lower_ohm"] / (b["upper_ohm"] + b["lower_ohm"]),
        "thresholds_V": {
            "arm_minimum": pack["arm_minimum_V"],
            "firmware_cutoff": pack["cutoff_V"],
        },
        "placement": "On the switched bus, so it draws nothing when the robot is off",
    }


def fault_table():
    return [
        {
            "fault": "Short circuit on the bus",
            "detection": "LM5060 VDS comparator; fuse",
            "response": "LM5060 pulls GATE low at 80 mA (5 us); the 10 A MINI fuse interrupts if the MOSFET fails short",
        },
        {
            "fault": "Inrush at switch-on",
            "detection": "-",
            "response": "LM5060 rise time 15-33 ms holds inrush below 0.5 A; XT30s are never hot-plugged",
        },
        {
            "fault": "Pack over-discharge",
            "detection": "Teensy bus ADC and moteus bus voltage; LM5060 UVLO",
            "response": "Firmware stops and disarms at 18.0 V (3.0 V/cell); the UVLO opens the switch near 2.9 V/cell as a hardware backstop",
        },
        {
            "fault": "Bus overvoltage from regeneration",
            "detection": "moteus bus sensing",
            "response": "Flux braking above 27 V; moteus faults at 30 V; regen power limited to 10 W per driver",
        },
        {
            "fault": "Reverse pack polarity",
            "detection": "-",
            "response": "Prevented by the keyed XT30; the moteus has no reverse protection",
        },
        {
            "fault": "CAN-FD bus loss",
            "detection": "moteus watchdog; Teensy feedback timeout",
            "response": "moteus stops on command timeout; the supervisor latches a stop",
        },
        {
            "fault": "IMU loss",
            "detection": "Teensy IMU timeout",
            "response": "Supervisor latches a stop",
        },
        {
            "fault": "Driver or motor overtemperature",
            "detection": "moteus temperature sensing",
            "response": "moteus derates, then faults",
        },
        {
            "fault": "5 V regulator overload",
            "detection": "-",
            "response": "LED brightness capped in firmware; the vision host is reserved within the 5 A rating",
        },
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/m11/power")
    args = parser.parse_args()
    p = json.loads((ROOT / "config/robot.json").read_text())["m10"]
    e = p["electronics"]
    req = json.loads((REQUIREMENTS / "requirements.json").read_text())
    cases = json.loads((REQUIREMENTS / "cases.json").read_text())
    config = json.loads((REQUIREMENTS / "config.json").read_text())
    budget = logic_budget(e)
    report = {
        "scope": "M11 phase C calculations from config/robot.json m10.electronics and the phase A "
        "requirements; a digital design check, not a hardware measurement",
        "electronics_config": e,
        "logic_budget": budget,
        "runtime": runtime(e, budget, cases, config),
        "soft_start": soft_start(e, budget),
        "uvlo": uvlo_divider(e),
        "fuse_and_wiring": fuse_and_wiring(e, req),
        "lm5060": lm5060_design(e, budget, req),
        "overvoltage": overvoltage(e, p),
        "battery_monitor": battery_monitor(e),
        "fault_table": fault_table(),
    }
    checks = {
        "regulator": budget["regulator_passes"],
        "inrush": report["soft_start"]["inrush_passes"],
        "lm5060_start_up_timer": report["lm5060"]["start_up"]["passes"],
        "lm5060_trip_above_peak": report["lm5060"]["trip_above_peak"],
        "mosfet_gate_voltage": report["lm5060"]["vgs_within_rating"],
        "mosfet_linear_mode": report["lm5060"]["linear_mode_passes"],
        "fuse_peak": report["fuse_and_wiring"]["carries_peak"],
        "fuse_rms": report["fuse_and_wiring"]["carries_rms_with_75pct_derating"],
        "fuse_interrupting": report["fuse_and_wiring"]["interrupting_passes"],
        "driver_minimum_voltage": report["fuse_and_wiring"]["drop"]["driver_minimum_passes"],
        "battery_monitor_range": report["battery_monitor"]["pin_within_reference"],
    }
    report["checks"] = checks
    report["passed"] = all(checks.values())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"checks": checks, "passed": report["passed"]}, indent=2))


if __name__ == "__main__":
    main()
