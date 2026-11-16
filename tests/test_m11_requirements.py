from types import SimpleNamespace

import numpy as np
import pytest

from bipe.m11.requirements import ElectricalRecorder, model_config
from bipe.parameters import load_parameters


def recorder_with(current, torque, speed, dt=0.001):
    """Feed constant per-motor current, torque and rotor speed for 1 s."""
    config = model_config(load_parameters(profile="m10"))
    recorder = ElectricalRecorder(config)
    drives = SimpleNamespace(
        rv=np.arange(6), phase_current=np.asarray(current), motor_torque=np.asarray(torque)
    )
    d = SimpleNamespace(qvel=np.asarray(speed, dtype=float))
    for k in range(1000):
        recorder(k * dt, dt, drives, d)
    return config, recorder


def test_bus_power_divides_motoring_and_multiplies_regenerating_power_by_efficiency():
    current = [2.0, 0, 0, 0, 0, 0]
    config, recorder = recorder_with(current, [0.1, 0, 0, 0, -0.2, 0], [50, 0, 0, 0, 10, 0])
    s = recorder.summary()
    for scenario, row in config["scenarios"].items():
        eta = row["inverter_efficiency"]
        r = config["phase_to_phase_resistance_ohm_20C"][0] * (1 + 0.00393 * (row["copper_C"] - 20))
        hip = (0.1 * 50 + 0.75 * r * 2.0**2) / eta
        hub = -0.2 * 10 * eta
        expected = hip + hub + config["auxiliary_power_W"]
        assert s[scenario]["bus_power_mean_W"] == pytest.approx(expected)
        assert s[scenario]["pack_low"]["bus_current_peak_A"] == pytest.approx(
            expected / config["pack_low_V"]
        )
        assert s["motors"]["left_hub"][scenario]["driver_dc_power_min_W"] == pytest.approx(hub)


def test_phase_voltage_demand_is_back_emf_plus_resistive_drop():
    config, recorder = recorder_with([0, 0, 0, 0, 3.0, 0], [0] * 6, [0, 0, 0, 0, 20, 0])
    hub = recorder.summary()["motors"]["left_hub"]["regen"]
    r = config["phase_to_phase_resistance_ohm_20C"][4]
    assert hub["phase_voltage_demand_peak_V"] == pytest.approx(
        config["back_emf_V_s_rad"][4] * 20 + r * 3.0
    )


def test_regeneration_is_reported_when_the_bus_power_goes_negative():
    _, recorder = recorder_with([0] * 6, [0, 0, 0, 0, -0.5, -0.5], [0, 0, 0, 0, 20, 20])
    regen = recorder.summary()["regen"]
    assert regen["bus_power_min_W"] < 0
    assert regen["time_regenerating_fraction"] == 1.0
    assert regen["regenerated_energy_J"] == pytest.approx(-regen["bus_energy_J"])


def test_braking_current_lowers_the_phase_voltage_demand():
    config, recorder = recorder_with([0, 0, 0, 0, -3.0, 0], [0] * 6, [0, 0, 0, 0, 20, 0])
    hub = recorder.summary()["motors"]["left_hub"]["regen"]
    r = config["phase_to_phase_resistance_ohm_20C"][4]
    assert hub["phase_voltage_demand_peak_V"] == pytest.approx(
        abs(config["back_emf_V_s_rad"][4] * 20 - r * 3.0)
    )


def test_saved_traces_keep_bus_power_at_1_ms_and_motor_channels_at_the_control_rate():
    _, recorder = recorder_with([1.0] * 6, [0.1] * 6, [5.0] * 6, dt=0.00025)
    trace = recorder.trace()
    assert float(trace["motor_dt_s"]) == pytest.approx(0.005)
    assert float(trace["bus_dt_s"]) == pytest.approx(0.001)
    assert len(trace["bus_time"]) == len(trace["bus_power_draw_W"]) == 250
    assert trace["phase_current_A"].shape == (50, 6)
