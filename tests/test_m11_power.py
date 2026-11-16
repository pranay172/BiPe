import json

import pytest

from bipe.m11 import power
from bipe.parameters import ROOT


@pytest.fixture(scope="module")
def m10():
    return json.loads((ROOT / "config/robot.json").read_text())["m10"]


def test_soft_start_rise_time_is_full_pack_over_gate_current_slew(m10):
    e = m10["electronics"]
    ramp = power.soft_start(e, power.logic_budget(e))["ramps"]["typical"]
    s = e["soft_start"]
    assert ramp["rise_time_s"] == pytest.approx(
        e["pack"]["full_V"] * s["gate_capacitor_F"] / s["gate_charge_current_A"][1]
    )
    c_bus = e["drivers"]["count"] * e["drivers"]["bulk_capacitance_F_each"]
    assert ramp["inrush_A"] == pytest.approx(c_bus * ramp["slew_V_s"])


def test_uvlo_divider_reproduces_its_rising_threshold_with_the_bias_current(m10):
    u = power.uvlo_divider(m10["electronics"], rising_V=20.0)
    assert u["rising_V"] == pytest.approx(20.0)
    assert u["falling_V"] < u["rising_V"]


def test_phase_c_checks_pass_for_the_committed_configuration(m10):
    e = m10["electronics"]
    req = json.loads((power.REQUIREMENTS / "requirements.json").read_text())
    assert power.logic_budget(e)["regulator_passes"]
    assert power.soft_start(e, power.logic_budget(e))["inrush_passes"]
    wiring = power.fuse_and_wiring(e, req)
    assert wiring["carries_peak"] and wiring["interrupting_passes"]
    assert wiring["drop"]["driver_minimum_passes"]


def test_lm5060_timer_and_trip_follow_the_datasheet_currents(m10):
    e = m10["electronics"]
    req = json.loads((power.REQUIREMENTS / "requirements.json").read_text())
    d = power.lm5060_design(e, power.logic_budget(e), req)
    s = e["soft_start"]
    t, sense, m = s["timer"], s["sense"], s["mosfet"]
    # Start-up timeout: the low fault threshold over the highest start-up charge current.
    v_lo = t["fault_threshold_V"] * (1 - t["fault_threshold_tolerance_assumption"])
    assert d["start_up"]["timer_timeout_min_s"] == pytest.approx(
        v_lo * s["timer_capacitor_F"] / max(t["start_up_charge_A"])
    )
    # The lowest VDS threshold over the hot on-resistance sets the lowest trip current.
    vds_min = sense["resistor_ohm"] * min(sense["current_A"]) - sense["offset_V"]
    assert d["trip_current_min_A"] == pytest.approx(vds_min / m["rds_on_ohm_max_125C"])
    assert d["start_up"]["needed_s"] == pytest.approx(
        d["start_up"]["to_plateau_s"] + d["start_up"]["ramp_s"] + d["start_up"]["to_vgs_status_s"]
    )
    assert d["start_up"]["passes"] and d["trip_above_peak"] and d["vgs_within_rating"]
    assert d["linear_mode_passes"]


def test_a_smaller_timer_capacitor_fails_the_start_up_check(m10):
    e = json.loads(json.dumps(m10["electronics"]))
    e["soft_start"]["timer_capacitor_F"] = 1.0e-07
    req = json.loads((power.REQUIREMENTS / "requirements.json").read_text())
    assert not power.lm5060_design(e, power.logic_budget(e), req)["start_up"]["passes"]
