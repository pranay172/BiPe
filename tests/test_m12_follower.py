import math

import numpy as np
import pytest

from bipe.m12 import follow as FW
from bipe.m12 import following as FL
from bipe.m12 import scenarios as S


def _stream(py, cf, seed):
    """Drive both followers with one random report stream; return the largest difference."""
    rng = np.random.default_rng(seed)
    seq, worst, modes = 0, 0.0, set()
    x = y = yaw = 0.0
    tx, ty = 1.07, 0.0
    for i in range(4000):
        t = 0.005 * (i + 1)
        if i == 100:
            py.request(True)
            cf.request(True)
        if i == 3000:
            py.request(False)
            cf.request(False)
        if i == 3100:
            py.request(True)
            cf.request(True)
        tx += 0.003
        x += 0.0025
        yaw += 0.0005 * math.sin(i / 200)
        if i % 7 == 3:
            seq += 1 + int(rng.random() < 0.05)
            detected = bool(rng.random() < (0.0 if 1500 < i < 1700 else 0.9))
            cam = (x + 0.074 * math.cos(yaw), y + 0.074 * math.sin(yaw))
            r = math.hypot(tx - cam[0], ty - cam[1]) + rng.normal(0, 0.01)
            b = math.atan2(ty - cam[1], tx - cam[0]) - yaw + rng.normal(0, 0.004)
            tag = 0 if rng.random() > 0.02 else 3
            age = 0.13 + 0.02 * rng.random() if rng.random() > 0.03 else 0.6
            s = seq if rng.random() > 0.02 else seq - 1
            a = py.report(s, tag, detected, r, b, age, t - 0.001)
            assert a == cf.report(s, tag, detected, r, b, age, t - 0.001)
        allowed = not 3500 < i < 3510
        rv = 0.0 if py.mode in (1, 5) or 2000 < i < 2200 else 0.5
        lean = 0.1 * math.sin(i / 40)
        a = py.step(t, x, y, yaw, lean, rv, allowed)
        b = cf.step(t, x, y, yaw, lean, rv, allowed)
        assert (py.mode, py.event) == (cf.mode, cf.event)
        worst = max(
            worst,
            abs(a[0] - b[0]),
            abs(a[1] - b[1]),
            max(abs(p - q) for p, q in zip(py.s, cf.s, strict=True)),
            max(abs(p - q) for p, q in zip(py.P, cf.P, strict=True)),
        )
        modes.add(py.mode)
    assert py.reports == cf.reports
    return worst, modes


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_python_reference_matches_the_c_follower(seed):
    worst, modes = _stream(FW.Follower(), FW.CFollower(), seed)
    assert worst <= 1e-9
    assert {1, 2, 3, 4} <= modes


def test_generated_header_is_current():
    assert FW.HEADER.read_text() == FW.header_text()


def test_c_default_config_is_valid_and_matches_the_json():
    lib = FW.library()
    default = lib.bipe_follow_default_config()
    c_ref = FW.c_config()
    assert lib.bipe_follow_config_valid(c_ref)
    for name, value in FW.config_values().items():
        got = getattr(default, name)
        want = getattr(c_ref, name)
        if isinstance(value, list):
            assert list(got) == list(want)
        else:
            assert got == want


def test_off_until_requested_and_cancel_stops():
    f = FW.Follower()
    assert f.report(1, 0, True, 1.0, 0.0, 0.13, 0.2) == "ignored"
    v, w = f.step(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, True)
    assert (v, w) == (0.0, 0.0) and FW.MODES[f.mode] == "off"
    f.request(True)
    assert FW.MODES[f.mode] == "waiting"
    f.request(False)
    assert FW.MODES[f.mode] == "off"


def test_a_pi_restart_and_a_new_request_reset_the_sequence_check():
    for f in (FW.Follower(), FW.CFollower()):
        f.request(True)
        assert f.report(100, 0, False, 0.0, 0.0, 0.13, 1.0) == "empty"
        # A late report within the restart gap is still a repeat.
        assert f.report(100 - FW.SEQUENCE_RESTART_GAP, 0, False, 0.0, 0.0, 0.13, 1.1) == "repeated"
        # Further back is the Pi restarting its counter: heard again.
        assert f.report(1, 0, False, 0.0, 0.0, 0.13, 1.2) == "empty"
        # A new request accepts the counter afresh.
        f.request(False)
        f.request(True)
        assert f.report(1, 0, False, 0.0, 0.0, 0.13, 1.3) == "empty"


def test_pi_link_rates_and_latency():
    env = FL.setup()
    tracks = S.tracks(env["defs"]["start_stop"], env["nominal"] + env["ahead"], env["nominal"])
    link = FL._make_link(env, tracks, 7, 0.0)
    robot = {"x": 0.0, "y": 0.0, "yaw": 0.0, "acceleration": 0.0, "origin": np.zeros(2)}
    arrivals = []
    for k in range(1, 401):
        arrivals += link.poll(k * 0.005, robot)
    frames = link.frames
    lo, hi = env["link"]["tracking_latency_s"]
    tracked = [f for f in frames if f["tracking"]]
    # The first frame is a full-frame search; the frames ready before it ends are skipped.
    assert frames[1]["sequence"] > 5 and len(frames) >= 45
    assert all(f["possible"] for f in frames)
    assert sum(f["detected"] for f in frames) / len(frames) > 0.9
    assert all(lo <= f["latency_s"] <= hi + 1e-12 for f in tracked)
    for arrival, r in arrivals:
        assert r["age_s"] == pytest.approx(arrival - _frame_time(frames, r["sequence"]) - 0.002)
    # The same seed gives the same reports.
    again = FL._make_link(env, tracks, 7, 0.0)
    for k in range(1, 401):
        again.poll(k * 0.005, robot)
    assert again.frames == frames


def _frame_time(frames, sequence):
    return next(f["t"] for f in frames if f["sequence"] == sequence)


def test_parity_on_a_scenario_report_stream():
    r = FL.parity_task(("corner_90", 0, 0.15))
    assert r["reports"] > 300 and r["mismatches"] == 0 and r["max_difference"] <= 1e-9
    assert {"waiting", "following"} <= set(r["modes"])


@pytest.mark.parametrize("speed", [0.5, 1.0, 1.2])
def test_coast_is_a_flat_0_45_s_at_every_speed(speed):
    """The loss coast is 0.45 s whatever the robot speed (phase D sign-off); C and Python
    agree."""
    coast = FW.load()["modes"]["coast_s"]
    assert coast == 0.45
    for f in (FW.Follower(), FW.CFollower()):
        f.request(True)
        t, seq = 0.0, 0
        while FW.MODES[f.mode] != "following":
            t += 0.005
            if round(t / 0.005) % 7 == 0:
                seq += 1
                f.report(seq, 0, True, 1.0, 0.0, 0.13, t)
            f.step(t, 0.0, 0.0, 0.0, 0.0, 0.0, True)
        last = t
        for _ in range(200):
            t += 0.005
            if round(t / 0.005) % 7 == 0:
                seq += 1
                f.report(seq, 0, False, 0.0, 0.0, 0.13, t)
            f.step(t, 0.0, 0.0, 0.0, 0.0, speed, True)
            if FW.MODES[f.mode] == "braking":
                break
        assert t - last == pytest.approx(coast, abs=0.006)
