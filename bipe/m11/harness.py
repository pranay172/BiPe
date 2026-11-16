"""M11 phase D: hub harness checks along the M10 leg route.

Reads m10.electronics.harness in config/robot.json, the joint end stops and the
phase A requirements, and writes artifacts/m11/harness/report.json. The checks
cover the bundle against the shaft ports, gallery and channels, the lead
resistance and current density, the encoder supply and SPI edges, and the strand
bending strain at the rotating crossings. Strain is reported, not passed: wire
fatigue life is a bench cycle test.
"""

import argparse
import json
import math
from pathlib import Path

from ..parameters import ROOT

REQUIREMENTS = ROOT / "artifacts/m11/requirements"
# Bundled-wire current density screen (assumption, conservative for PTFE at 200 C rating).
CURRENT_DENSITY_LIMIT_A_mm2 = 5.0


def wire_resistance(h, wire):
    """Stranded conductor resistance per metre at 20 C."""
    area = wire["strands"] * math.pi * wire["strand_d_m"] ** 2 / 4
    return h["copper_resistivity_ohm_m_20C"] * h["lay_factor"] / area, area


def bundle(h, phase_od, encoder_od):
    """Equivalent bundle diameter at each packing fraction and its fill of each restriction."""
    n = h["hub_conductors"]
    sum_d2 = n["phase"] * phase_od**2 + n["encoder"] * encoder_od**2
    rows = {}
    for pf in h["packing_fraction"]:
        d = math.sqrt(sum_d2 / pf)
        rows[f"packing_{pf}"] = {
            "bundle_diameter_m": d,
            "fits": {k: d <= v for k, v in h["restrictions_m"].items()},
        }
    fill = {k: sum_d2 / v**2 for k, v in h["restrictions_m"].items()}
    worst = rows[f"packing_{min(h['packing_fraction'])}"]
    return {
        "wire_area_fill": fill,
        "packings": rows,
        "fits_everywhere": all(worst["fits"].values()),
    }


def route_length(h):
    return sum(h["route_m"].values()) * (1 + h["slack_fraction"])


def phase_leads(h, gl, hub):
    """Phase pair resistance against the plant's lead allowance; drop and density at the hub currents."""
    per_m, area = wire_resistance(h, h["wires"]["phase"])
    length = route_length(h)
    line = 2 * per_m * length
    return {
        "route_length_m": length,
        "ohm_per_m": per_m,
        "line_to_line_ohm": line,
        "plant_allowance_ohm": gl["lead_resistance_ohm"],
        "within_plant_allowance": line <= gl["lead_resistance_ohm"],
        "drop_at_peak_V": line * hub["peak_A"],
        "lead_copper_loss_rms_W": 0.75 * line * hub["rms_A"] ** 2,
        "current_density_rms_A_mm2": hub["rms_A"] / (area * 1e6),
        "current_density_limit_A_mm2": CURRENT_DENSITY_LIMIT_A_mm2,
        "current_density_passes": hub["rms_A"] / (area * 1e6) <= CURRENT_DENSITY_LIMIT_A_mm2,
    }


def encoder(h):
    """AS5047P supply drop over the 3V3/GND pair and SPI edge time against the clock."""
    e = h["encoder"]
    per_m, _ = wire_resistance(h, h["wires"]["encoder"])
    length = route_length(h)
    drop = 2 * per_m * length * e["supply_A"]
    capacitance = e["line_capacitance_F_per_m_assumption"] * length
    rise = 2.2 * (e["driver_resistance_ohm_assumption"] + e["series_resistor_ohm"]) * capacitance
    half_period = 0.5 / e["spi_rate_hz"]
    round_trip = 2 * length / (0.7 * 299792458.0)
    return {
        "supply_drop_V": drop,
        "supply_drop_passes": drop <= 0.1,
        "line_capacitance_F": capacitance,
        "rise_time_s": rise,
        "half_period_s": half_period,
        "rise_within_20pct_of_half_period": rise <= 0.2 * half_period,
        "round_trip_delay_s": round_trip,
        "spi_rate_hz": e["spi_rate_hz"],
        "note": "Single-ended over the hub run beside the PWM phase leads: capacitive pickup is "
        "the open risk. Twist the phase leads, group the encoder leads with GND, fit the series "
        "resistors at the drivers and check on the bench with the real harness.",
    }


def crossings(h, stops):
    """Wrap length and strand bending strain at each rotating crossing."""
    loop = h["service_loop_radius_m"]
    out = {}
    for name, c in h["crossings"].items():
        lo, hi = stops[c["range"]]
        theta = math.radians(hi - lo)
        strain = {}
        for kind in ("phase", "encoder"):
            w = h["wires"][kind]
            direct = c["shaft_radius_m"] + w["od_m"] / 2
            strain[kind] = {
                "direct_wrap": w["strand_d_m"] / (2 * direct),
                "service_loop": w["strand_d_m"] / (2 * (loop - w["od_m"] / 2)),
            }
        out[name] = {
            "range_deg": hi - lo,
            "loop_arc_change_m": loop * theta,
            "strand_strain": strain,
        }
    return out


def hub_currents(req):
    hubs = [req["motors"][n] for n in ("left_hub", "right_hub")]
    return {
        "peak_A": max(m["phase_current_peak_A"]["value"] for m in hubs),
        "rms_A": max(m["phase_current_rms_A"]["value"] for m in hubs),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/m11/harness")
    args = parser.parse_args()
    p = json.loads((ROOT / "config/robot.json").read_text())["m10"]
    h = p["electronics"]["harness"]
    req = json.loads((REQUIREMENTS / "requirements.json").read_text())
    hub = hub_currents(req)
    w = h["wires"]
    report = {
        "scope": "M11 phase D harness calculations from config/robot.json m10.electronics.harness, "
        "the end stops and the phase A requirements; a digital design check, not a hardware "
        "measurement",
        "harness_config": h,
        "hub_currents": hub,
        "bundle": bundle(h, w["phase"]["od_m"], w["encoder"]["od_m"]),
        "rejected_bundle": bundle(h, w["rejected"]["phase_od_m"], w["rejected"]["encoder_od_m"]),
        "phase_leads": phase_leads(h, p["physics"]["hub_motor"], hub),
        "encoder": encoder(h),
        "crossings": crossings(h, p["physics"]["end_stops_deg"]),
    }
    checks = {
        "bundle_fits": report["bundle"]["fits_everywhere"],
        "lead_resistance_within_plant": report["phase_leads"]["within_plant_allowance"],
        "lead_current_density": report["phase_leads"]["current_density_passes"],
        "encoder_supply": report["encoder"]["supply_drop_passes"],
        "spi_edges": report["encoder"]["rise_within_20pct_of_half_period"],
    }
    report["checks"] = checks
    report["passed"] = all(checks.values())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"checks": checks, "passed": report["passed"]}, indent=2))


if __name__ == "__main__":
    main()
