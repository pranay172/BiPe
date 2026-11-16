import math

import mujoco
import numpy as np
import pytest

from bipe.m12 import following as FL
from bipe.m12 import rendered_link as RL
from bipe.m12 import scenarios as S

ENV = FL.setup()
RCFG = RL.load()
EXPOSURE = ENV["perc"]["camera"]["exposure"]
LINK = ENV["link"]
DELAY = ENV["cfg"]["timing"]["link_delay_s"]
TRACKS = S.tracks(ENV["defs"]["straight_1.0"], ENV["nominal"] + ENV["ahead"], ENV["nominal"])


class _State:
    def __init__(self, qpos):
        self.qpos = qpos


def _link(seed=1, check_repeat=False, lighting="nominal", **kw):
    return RL.RenderedLink(
        LINK,
        EXPOSURE["exposure_s"],
        EXPOSURE["subframes"],
        DELAY,
        TRACKS,
        seed,
        lighting=RCFG["lighting"][lighting],
        check_repeat=check_repeat,
        **kw,
    )


def _standing(link, seconds):
    """Record a robot standing at the origin for `seconds` and return the track origin."""
    sc = link.sc
    S.place_robot(sc.m, sc.d, sc.p, 0.0, 0.0, 0.0, 40, 0.0)
    mujoco.mj_forward(sc.m, sc.d)
    axle = np.mean([sc.d.site(s + "_axle").xpos for s in ("left", "right")], axis=0)
    state = _State(sc.d.qpos.copy())
    for k in range(round(seconds / 0.00025) + 1):
        link.record(k * 0.00025, 0.00025, None, state)
    return {"origin": axle[:2].copy()}


def _run(link, robot, seconds):
    reports = []
    for k in range(round(seconds / ENV["cfg"]["timing"]["control_dt_s"]) + 1):
        t = k * ENV["cfg"]["timing"]["control_dt_s"]
        reports += [(t, a, r) for a, r in link.poll(t, robot)]
    return reports


@pytest.fixture(scope="module")
def standing_run():
    link = _link(check_repeat=True)
    robot = _standing(link, 0.6)
    reports = _run(link, robot, 0.5)
    yield link, reports
    link.close()


def test_the_rendered_tag_is_tracked_with_the_phase_c_accuracy(standing_run):
    link, _ = standing_run
    frames = link.frames
    assert frames and all(f["detected"] for f in frames)
    # The first frame is a full-frame search; after a detection the region of interest.
    assert not frames[0]["tracking"] and not frames[0]["roi_used"]
    assert all(f["tracking"] and f["roi_used"] for f in frames[1:])
    for f in frames:
        # Phase C: range +1-2 % (the tone curve narrows the black square), bearing small.
        assert 0 < f["measured_range_m"] - f["range_m"] < 0.03 * f["range_m"]
        assert abs(f["measured_bearing_rad"] - f["bearing_rad"]) < math.radians(0.3)
    assert link.repeat_mismatches == 0


def test_latency_follows_the_tracker_mode_and_the_search_skips_frames(standing_run):
    link, reports = standing_run
    frames = link.frames
    lo, hi = LINK["acquisition_latency_s"]
    assert lo <= frames[0]["latency_s"] <= hi
    lo, hi = LINK["tracking_latency_s"]
    assert all(lo <= f["latency_s"] <= hi for f in frames[1:])
    # Frames ready before the full-frame search ends are skipped.
    mid = frames[0]["t"] - EXPOSURE["exposure_s"] / 2
    skipped_until = mid + frames[0]["latency_s"] - LINK["pipeline_s"]
    assert frames[1]["t"] >= skipped_until
    assert frames[1]["sequence"] > frames[0]["sequence"] + 1
    by_sequence = {f["sequence"]: f for f in frames}
    for t, arrival, r in reports:
        f = by_sequence[r["sequence"]]
        # The latency counts from the exposure's centre.
        assert arrival == pytest.approx(f["t"] - EXPOSURE["exposure_s"] / 2 + f["latency_s"])
        assert t >= arrival - 1e-12
        assert r["age_s"] == pytest.approx(f["latency_s"] - DELAY)
        assert r["range_m"] == f["measured_range_m"]


def test_a_run_depends_on_its_seed_alone():
    a, b, c = _link(seed=3), _link(seed=3), _link(seed=4)
    try:
        for link in (a, b, c):
            _run(link, _standing(link, 0.25), 0.2)

        # The detector's own run time (a measurement on this machine) is the exception.
        def strip(link):
            return [{k: v for k, v in f.items() if k != "detector_s"} for f in link.frames]

        assert strip(a) == strip(b)
        assert [f["latency_s"] for f in a.frames] != [f["latency_s"] for f in c.frames]
    finally:
        for link in (a, b, c):
            link.close()


def test_lighting_variants_set_the_lights_and_the_noise_budget():
    nominal = RCFG["lighting"]["nominal"]
    link = _link(lighting="side_light")
    try:
        side = RCFG["lighting"]["side_light"]
        m = link.sc.m
        assert np.allclose(m.vis.headlight.ambient, side["headlight"]["ambient"])
        sun = np.array(side["lights"][1]["dir"], float)
        assert np.allclose(m.light_dir[1], sun / np.linalg.norm(sun))
        assert link.sc.camera.noise["illuminance_lux"] == side["illuminance_lux"]
        RL.apply_lighting(link.sc, RCFG["lighting"]["dim"])
        assert (
            link.sc.camera.noise["illuminance_lux"]
            == ENV["perc"]["camera"]["noise"]["low_light_lux"]
        )
        assert np.allclose(m.light_diffuse[0], nominal["lights"][0]["diffuse"])
    finally:
        link.close()


def test_the_nominal_lighting_is_the_phase_c_scene():
    link = _link(lighting="nominal")
    try:
        # A fresh phase C scene has these lights; applying nominal changes nothing.
        from bipe.m12 import perception as PC

        fresh = PC.Scene(RCFG["layout"])
        try:
            m, f = link.sc.m, fresh.m
            assert np.allclose(m.light_dir, f.light_dir)
            assert np.allclose(m.light_diffuse, f.light_diffuse)
            assert np.allclose(m.vis.headlight.ambient, f.vis.headlight.ambient)
            assert np.allclose(m.vis.headlight.diffuse, f.vis.headlight.diffuse)
            assert link.sc.camera.noise == fresh.camera.noise
        finally:
            fresh.close()
    finally:
        link.close()


def test_the_phase_f_matrix_is_the_approved_plan():
    from bipe.m12 import following_rendered as FR

    items = FR.payloads("lib.so", "matrix")
    groups = {}
    for group, payload in items:
        groups[group] = groups.get(group, 0) + 1
        assert payload[6]["rendered"]["lighting"] in RCFG["lighting"]
    n = len(ENV["defs"])
    assert groups == {
        "nominal": n * RCFG["nominal_seeds"],
        "latency_0.10": n,
        "latency_0.20": n,
        "variants": len(FL.VARIANTS) * len(FL.VARIANT_SCENARIOS),
        "slopes": len(FL.SLOPE_CASES),
        "light_dim": len(RCFG["lighting_scenarios"]),
        "light_side_light": len(RCFG["lighting_scenarios"]),
    }
    keys = [FR.run_key(p) for _, p in items]
    assert len(set(keys)) == len(keys)
    # The check is seed 0 of each scenario, which the matrix reuses.
    check = {FR.run_key(p) for _, p in FR.payloads("lib.so", "check")}
    assert len(check) == n and check <= set(keys)


def test_frame_metrics_count_misses_and_errors():
    from bipe.m12 import following_rendered as FR

    def frame(detected, in_frame=True, possible=True, r=1.0, mr=1.01):
        return {
            "detected": detected,
            "in_frame": in_frame,
            "possible": possible,
            "range_m": r,
            "bearing_rad": 0.0,
            "measured_range_m": mr if detected else None,
            "measured_bearing_rad": 0.001 if detected else None,
            "detector_s": 0.01,
        }

    frames = [frame(True), frame(False), frame(False), frame(True), frame(False, False, False)]
    m = FR.frame_metrics(frames)
    assert m["frames"] == 5 and m["detected"] == 2 and m["possible"] == 4
    assert m["miss_runs"] == 1 and m["miss_run_max"] == 2
    assert m["range_error_mean_m"] == pytest.approx(0.01)
    assert m["bearing_error_mean_deg"] == pytest.approx(math.degrees(0.001))
    # A detection the view gate dropped is a miss, and its range is not in the errors.
    frames[0] = dict(frame(True, mr=0.97), gated=True)
    m = FR.frame_metrics(frames)
    assert m["detected"] == 1 and m["gated"] == 1 and m["miss_run_max"] == 3
    assert m["range_error_mean_m"] == pytest.approx(0.01)


def test_the_fixed_mount_conversion_agrees_on_a_robot_standing_level():
    plant, fixed = _link(seed=5), _link(seed=5, pose_conversion="fixed_mount")
    try:
        for link in (plant, fixed):
            _run(link, _standing(link, 0.25), 0.2)
        for a, b in zip(plant.frames, fixed.frames, strict=True):
            assert a["detected"] and b["detected"]
            assert b["measured_range_m"] == pytest.approx(a["measured_range_m"], abs=1e-6)
            assert b["measured_bearing_rad"] == pytest.approx(a["measured_bearing_rad"], abs=1e-6)
    finally:
        plant.close()
        fixed.close()


def test_a_region_miss_is_reported_at_once_and_the_next_frame_searches_the_full_frame():
    link = _link(seed=7)
    try:
        robot = _standing(link, 0.8)
        _run(link, robot, 0.4)
        assert link.frames[-1]["roi_used"] and link.frames[-1]["detected"]
        n = len(link.frames)
        # Move the region of interest off the tag: the next frame misses there.
        link.tracker.last["corners"] = link.tracker.last["corners"] + [400.0, 200.0]
        dt = ENV["cfg"]["timing"]["control_dt_s"]
        k = round(0.4 / dt) + 1
        while len(link.frames) < n + 2:
            link.poll(k * dt, robot)
            k += 1
        miss, search = link.frames[n], link.frames[n + 1]
        assert miss["tracking"] and miss["roi_used"] and not miss["detected"]
        lo, hi = LINK["tracking_latency_s"]
        assert lo <= miss["latency_s"] <= hi
        # The miss holds no cores: the next frame is the very next one, a full search.
        assert search["sequence"] == miss["sequence"] + 1
        assert not search["tracking"] and not search["roi_used"] and search["detected"]
        lo, hi = LINK["acquisition_latency_s"]
        assert lo <= search["latency_s"] <= hi
    finally:
        link.close()


def test_the_retry_policy_retries_the_region_once_and_grows_it_with_age():
    policy = RCFG["tracker_policies"]["retry_and_grow"]
    link = _link(seed=7, tracker_policy=policy)
    try:
        robot = _standing(link, 0.9)
        _run(link, robot, 0.4)
        assert link.frames[-1]["roi_used"] and link.frames[-1]["detected"]
        shape = (ENV["perc"]["camera"]["output"]["height_px"], 2000)
        x0, _, x1, _ = link._window(shape, 0.0)
        x0a, _, x1a, _ = link._window(shape, 0.1)
        grow = policy["growth_rad_s"] * 0.1 * link.sc.model["f_px"]
        assert (x1a - x0a) - (x1 - x0) == pytest.approx(2 * grow, abs=2)
        n = len(link.frames)
        link.tracker.last["corners"] = link.tracker.last["corners"] + [400.0, 200.0]
        dt = ENV["cfg"]["timing"]["control_dt_s"]
        k = round(0.4 / dt) + 1
        while len(link.frames) < n + 3:
            link.poll(k * dt, robot)
            k += 1
        first, retry, search = link.frames[n : n + 3]
        lo, hi = LINK["tracking_latency_s"]
        for f in (first, retry):
            assert f["tracking"] and f["roi_used"] and not f["detected"]
            assert lo <= f["latency_s"] <= hi
        assert retry["sequence"] == first["sequence"] + 1
        assert search["sequence"] == retry["sequence"] + 1
        assert not search["tracking"] and not search["roi_used"] and search["detected"]
    finally:
        link.close()


def test_the_view_gate_drops_edge_on_poses_and_keeps_tracking():
    plain, gated = _link(seed=5), _link(seed=5, view_gate_deg=0.0)
    try:
        reports = {}
        for name, link in (("plain", plain), ("gated", gated)):
            reports[name] = _run(link, _standing(link, 0.8), 0.6)
        # Facing the tag the yaw view angle is 0; the fitted angle is the 3-D one, with the
        # head camera looking down on the waist tag, so well above a 0 degree gate.
        assert len(plain.frames) > 3 and all(f["view_deg"] < 1 for f in plain.frames)
        assert all(20 < f["measured_view_deg"] < 60 for f in plain.frames)
        assert not any(f["gated"] for f in plain.frames)
        assert all(f["gated"] and f["detected"] for f in gated.frames)
        assert all(not r["detected"] for _, _, r in reports["gated"])
        # The tracker still has the tag: after the first search, region-of-interest frames.
        assert all(f["tracking"] and f["roi_used"] for f in gated.frames[1:])
    finally:
        plain.close()
        gated.close()
