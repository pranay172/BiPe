"""M18 phase F: the measured-values file and the bench fitters."""

import json

import numpy as np
import pytest

from bipe.m18 import bench, fidelity, measured
from bipe.parameters import ROOT, load_parameters
from bipe.profiles import native


def _inputs():
    p = load_parameters(profile="m10")
    sensors = json.loads((ROOT / native(p)["sensors"]).read_text())
    return p, sensors


def test_committed_file_is_empty_and_changes_nothing():
    values = measured.load()
    assert values == {}
    p, sensors = _inputs()
    q, s, policy = measured.apply(p, sensors, values)
    r, t, _ = fidelity.apply(p, sensors, {})
    assert q == r and s == t and policy == {"seed": 0}


def test_every_key_is_known():
    entries = json.loads(measured.PATH.read_text())["values"]
    assert set(entries) == measured.KNOWN


def test_plant_sensor_and_fidelity_values_land():
    p, sensors = _inputs()
    values = {
        "chassis_mass_kg": 1.5,
        "hub_torque_constant_Nm_A": 0.12,
        "wheel_coulomb_Nm": 0.006,
        "transport_delay_ms": 7.0,
        "gyro_residual_bias_rad_s": [0.001, 0.0, -0.001],
        "imu_mount_deg": [0.5, -1.0],
        "stiction_ratio": 1.3,
        "payload_kg": 0.05,
    }
    q, s, _ = measured.apply(p, sensors, values)
    hub = p["physics"]["hub_motor"]
    assert q["physics"]["hub_motor"]["torque_constant_Nm_A"] == pytest.approx(0.12)
    assert q["physics"]["hub_motor"]["peak_torque_Nm"] == pytest.approx(hub["peak_torque_Nm"] * 0.8)
    assert q["physics"]["friction"]["wheel_coulomb_Nm"] == 0.006
    assert q["rigid_bodies"]["chassis"]["mass_kg"] == pytest.approx(1.55)  # payload on top
    assert s["transport_delay_s"] == pytest.approx(0.007)
    assert s["gyro_residual_bias_rad_s"] == [0.001, 0.0, -0.001]
    assert s["imu_mount_deg"] == [0.5, -1.0]
    assert q["physics"]["fidelity"]["stiction_ratio"] == 1.3
    assert p["rigid_bodies"]["chassis"]["mass_kg"] != 1.5  # inputs untouched


def test_unknown_key_refused(tmp_path):
    path = tmp_path / "m.json"
    path.write_text(json.dumps({"values": {"warp_drive": {"value": 1}}}))
    with pytest.raises(KeyError):
        measured.load(path)


def test_record_round_trip(tmp_path):
    path = tmp_path / "m.json"
    path.write_text(measured.PATH.read_text())
    bench.record({"floor_friction": 0.7}, "B10 test", path)
    assert measured.load(path) == {"floor_friction": 0.7}
    assert json.loads(path.read_text())["values"]["floor_friction"]["source"] == "B10 test"


def test_fitters_recover_known_values():
    rng = np.random.default_rng(0)
    i = np.linspace(0.2, 2.0, 20)
    assert bench.torque_constant(i, 0.068 * i, "hip_torque_constant_Nm_A")[
        "hip_torque_constant_Nm_A"
    ] == pytest.approx(0.068)
    w = np.concatenate([np.linspace(-20, -1, 20), np.linspace(1, 20, 20)])
    f = bench.friction(w, 0.004 * np.sign(w) + 3e-4 * w, breakaway_Nm=0.006, prefix="wheel")
    assert f["wheel_coulomb_Nm"] == pytest.approx(0.004)
    assert f["wheel_viscous_Nm_s_rad"] == pytest.approx(3e-4)
    assert f["stiction_ratio"] == pytest.approx(1.5)
    th = np.linspace(0, 2 * np.pi, 5000)
    c = bench.cogging(th, 0.01 + 0.006 * np.sin(84 * th + 1.0), rated_Nm=0.2)
    assert c["cogging_fraction"] == pytest.approx(0.03, rel=1e-3)
    assert bench.wheel_radius([3.0], [10.0], 3.0 / (20 * np.pi) / 1.02)["wheel_radius_ratio"][0] == (
        pytest.approx(1.02)
    )
    g = 9.80665
    r, pch = np.radians(1.0), np.radians(-2.0)
    f_true = g * np.array([-np.sin(pch), np.sin(r) * np.cos(pch), np.cos(r) * np.cos(pch)])
    n, dt = 20000, 0.005
    gyro = 0.001 + 0.003 * rng.standard_normal((n, 3))
    accel = f_true + 0.05 * rng.standard_normal((n, 3))
    imu = bench.imu_still(gyro, accel, dt)
    assert imu["gyro_residual_bias_rad_s"] == pytest.approx([0.001] * 3, abs=1e-4)
    assert imu["gyro_noise_std_rad_s"] == pytest.approx(0.003, rel=0.05)
    assert imu["imu_mount_deg"] == pytest.approx([1.0, -2.0], abs=0.02)
    assert bench.accel_six_face([g * 1.02] * 3, [-g * 1.02] * 3)["accel_scale"] == pytest.approx([0.02] * 3)
    assert bench.encoder_offsets([10.5, -3.0], [10.0, -2.0])["encoder_offset_deg"] == [0.5, -1.0]
    # A synchronous loop on a 0.4 ms bus: each reply (requested at a tick's start,
    # back 0.4 ms later) is used at the next tick, so its data is one tick old.
    ticks = np.arange(1000) * 0.005
    sampled = ticks + 0.0004
    used = ticks + 0.005
    used[::50] = np.nan  # never used: lost
    t = bench.loop_timing(sampled, used, commands_sent=1000, commands_acked=990)
    assert t["transport_delay_ms"] == pytest.approx(4.6)
    assert t["jitter_ms"] == 0 and t["packet_loss"] == pytest.approx(0.02)
    assert t["command_loss"] == pytest.approx(0.01)
    assert "command_loss" not in bench.loop_timing(sampled, used, commands_sent=1000)


def test_b9_value_keeps_one_tick_of_delay_in_the_full_plant():
    # Through measured.apply, a synchronous bench loop gives the nominal one-tick delay.
    p, sensors = _inputs()
    ticks = np.arange(200) * 0.005
    t = bench.loop_timing(ticks + 0.0004, ticks + 0.005)
    _, s, _ = measured.apply(p, sensors, {"transport_delay_ms": t["transport_delay_ms"]})
    assert round(s["transport_delay_s"] / 0.005) == round(sensors["transport_delay_s"] / 0.005) == 1
    assert bench.incline_slip(45)["floor_friction"] == pytest.approx(1.0)


def test_fitted_values_are_valid_measured_values():
    keys = set()
    for out in (
        bench.friction([1, -1, 2, -2], [1, -1, 1.1, -1.1], 1.5),
        bench.cogging(np.linspace(0, 6, 100), np.zeros(100), 1),
        bench.imu_still(np.zeros((1000, 3)) + 1e-3, np.tile([0, 0, 9.8], (1000, 1)), 0.01),
        bench.loop_timing(np.zeros(3), np.full(3, 0.005)),
    ):
        keys |= set(out)
    assert keys <= measured.KNOWN
