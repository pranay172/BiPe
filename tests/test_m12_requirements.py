import json
import math

import mujoco
import numpy as np
import pytest

from bipe.m10 import plant
from bipe.m12 import follower as F
from bipe.m12 import geometry as G
from bipe.m12 import requirements as R
from bipe.m12 import scenarios as S

SHARED, FOLLOWING, P = R.load()
MOUNT = G.camera_mount(SHARED)
WIDE = SHARED["m12"]["camera"]["lenses"]["wide"]
LOW = FOLLOWING["tag"]["layouts"]["low_tag"]
NOMINAL = FOLLOWING["range"]["nominal_m"]
AHEAD = G.camera_pose(P, MOUNT, 40)["ahead_of_axle_m"]


def world(occluder=False, layout=LOW):
    m = mujoco.MjModel.from_xml_string(
        S.world_xml(P, MOUNT, WIDE, FOLLOWING, layout, occluder=occluder)
    )
    return m, mujoco.MjData(m)


def test_camera_mount_follows_the_cad_head_parameters():
    # Hand arithmetic from m10.body, m10.head and m10.exterior.head (metres): the web face
    # at the pod front less the recess, the lens face 1.8 mm behind it on the 15 degree
    # axis, the camera centre 3.5 mm below the barrel axis.
    front = -0.010 + 0.045 + 0.060 / 2
    axis_z = -0.042 + 0.080 + 0.010 + 0.044 / 2 - 0.0015
    pitch = math.radians(15)
    x = front - 0.0015 - 0.0018 * math.cos(pitch)
    z = axis_z - 0.0035 - 0.0018 * math.sin(pitch)
    assert MOUNT["position_m"] == pytest.approx([x, 0.0, z], abs=1e-12)
    assert MOUNT["pitch_deg"] == 15


@pytest.mark.parametrize("posture, lean_deg", [(30, 0.0), (40, 0.0), (60, 0.0), (40, 6.0)])
def test_camera_pose_matches_the_head_camera_in_the_simulated_world(posture, lean_deg):
    m, d = world()
    S.place_robot(m, d, P, 0.3, -0.2, 0.7, posture, lean_deg)
    pose = G.camera_pose(P, MOUNT, posture, math.radians(lean_deg))
    camera = d.cam("head")
    # The reset leaves the tires 0.1 mm into the ground.
    assert camera.xpos[2] == pytest.approx(pose["height_m"], abs=2e-4)
    axle = np.mean([d.site(s + "_axle").xpos for s in plant.SIDES], axis=0)
    ahead = (camera.xpos - axle)[:2] @ [math.cos(0.7), math.sin(0.7)]
    assert ahead == pytest.approx(pose["ahead_of_axle_m"], abs=1e-9)
    forward = -camera.xmat.reshape(3, 3)[:, 2]
    assert math.degrees(math.asin(forward[2])) == pytest.approx(pose["elevation_deg"], abs=1e-9)


def test_lens_fields_of_view_are_the_datasheet_values():
    lenses = SHARED["m12"]["camera"]["lenses"]
    assert (lenses["standard"]["horizontal_fov_deg"], lenses["standard"]["vertical_fov_deg"]) == (
        66,
        41,
    )
    assert (lenses["wide"]["horizontal_fov_deg"], lenses["wide"]["vertical_fov_deg"]) == (102, 67)
    assert "raspberrypi.com/documentation/accessories/camera" in SHARED["m12"]["camera"]["source"]
    # The pinhole render covers both datasheet angles at 16:9.
    for lens in lenses.values():
        fovy = S.render_fovy_deg(lens)
        hfov = 2 * math.degrees(math.atan(math.tan(math.radians(fovy / 2)) * S.ASPECT))
        assert fovy >= lens["vertical_fov_deg"] and hfov >= lens["horizontal_fov_deg"] - 1e-9


def test_max_bearing_rate_closes_the_bearing_error_at_the_allowance():
    allowance, lag, accel = 0.7, 0.3, 3.0
    w = G.max_bearing_rate(allowance, lag, accel)
    assert w * lag + w**2 / (2 * accel) == pytest.approx(allowance)


def test_vertical_band_keeps_the_tag_inside_the_view_at_the_band_edges():
    poses = [G.camera_pose(P, MOUNT, 40)]
    size, margin = FOLLOWING["tag"]["size_m"], FOLLOWING["camera"]["edge_margin_fraction"]
    band = G.vertical_band(poses, WIDE["vertical_fov_deg"], [0.7, 1.5], size, margin)
    for r in (0.7, 1.5):
        low, high = G.visible_heights(poses[0], WIDE["vertical_fov_deg"], r, margin)
        assert band["low_m"] - size / 2 >= low - 1e-12
        assert band["high_m"] + size / 2 <= high + 1e-12


def test_scenario_paths_are_deterministic_and_respect_their_limits():
    defs = S.definitions(FOLLOWING, NOMINAL + AHEAD)
    for name, sc in defs.items():
        a = S.tracks(sc, NOMINAL + AHEAD, NOMINAL)
        b = S.tracks(sc, NOMINAL + AHEAD, NOMINAL)
        for key in a:
            for column in a[key]:
                np.testing.assert_array_equal(a[key][column], b[key][column])
        target = a["target"]
        assert np.max(np.abs(target["acceleration"])) <= sc["acceleration"] + 1e-9, name
        legs = [leg for leg in sc["legs"] if leg[0] in ("straight", "arc")]
        assert np.max(target["speed"]) <= max(
            leg[-1] if leg[0] == "arc" else leg[2] for leg in legs
        )
        # Every target starts at rest at the nominal range in front of the camera.
        assert target["speed"][0] == 0 and target["x"][0] == pytest.approx(NOMINAL + AHEAD)
    assert set(S.EXPECTATIONS) >= {sc["expect"] for sc in defs.values()}


def test_target_and_occluder_never_touch_or_constrain_the_robot():
    m, d = world(occluder=True)
    reference, ref_d = plant.build(P)
    assert (m.nq, m.nv, m.nu) == (reference.nq, reference.nv, reference.nu)
    for name in ("target", "occluder"):
        body = m.body(name)
        assert body.mocapid[0] >= 0
        geoms = range(body.geomadr[0], body.geomadr[0] + body.geomnum[0])
        assert all(m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0 for g in geoms)
    # The same drive torques give the same motion with the people standing by the robot.
    plant.Contacts(P).configure(m)
    plant.reset(m, d, P)
    plant.reset(reference, ref_d, P)
    S.place_person(m, d, "target", 0.05, 0.0, 0.0)
    S.place_person(m, d, "occluder", -0.05, 0.05, 1.0)
    drives = plant.MechanicalDrives(m, P)
    ref_drives = plant.MechanicalDrives(reference, P)
    torque = np.array([0.05, -0.05, 0.05, -0.05, 0.1, 0.1])
    for _ in range(200):
        drives.step(m, d, torque)
        ref_drives.step(reference, ref_d, torque)
    np.testing.assert_array_equal(d.qpos, ref_d.qpos)
    np.testing.assert_array_equal(d.qvel, ref_d.qvel)


def test_world_body_holds_the_ground_alone_so_slopes_still_work():
    m, d = world(occluder=True)
    plant.reset(m, d, P, slope_deg=8)
    assert m.body_geomnum[0] == 1


def test_loss_rule_stops_inside_the_stopping_distance_and_rates_are_consistent():
    loss = FOLLOWING["loss"]
    v = FOLLOWING["target"]["speed_max_m_s"]
    assert v * loss["coast_s"] + v**2 / (2 * loss["brake_deceleration_m_s2"]) <= (
        loss["stopping_distance_max_m"] + 1e-9
    )
    assert loss["stopping_distance_max_m"] < NOMINAL
    band = FOLLOWING["range"]["band_m"]
    assert band[0] < NOMINAL < band[1]
    engage = FOLLOWING["engage"]
    frames = engage["consecutive_detections"] - 1
    assert frames / FOLLOWING["camera"]["detection_rate_hz_assumption"] <= engage["minimum_s"]


def test_follower_engages_coasts_and_brakes_to_hold_when_the_tag_is_occluded():
    defs = S.definitions(FOLLOWING, NOMINAL + AHEAD)
    sight = F.Sight(P, MOUNT, WIDE, FOLLOWING, LOW)
    limits = {"speed": 1.2, "acceleration": 1.25, "yaw_rate": 1.0, "yaw_acceleration": 1.25}
    tracks = S.tracks(defs["occlusion_long"], NOMINAL + AHEAD, NOMINAL)
    a = F.run(tracks, sight, FOLLOWING, limits=limits, latency_s=0.1)
    b = F.run(tracks, sight, FOLLOWING, limits=limits, latency_s=0.1)
    np.testing.assert_array_equal(a["trace"], b["trace"])
    kinds = [e["event"] for e in a["events"]]
    assert kinds[:4] == ["engaged", "lost", "holding", "engaged"]
    f = dict(zip(a["columns"], a["trace"].T, strict=True))
    holding = f["mode"] == F.MODES.index("holding")
    assert np.all(np.abs(f["speed"][holding]) < 1e-3)
    lost = next(e for e in a["events"] if e["event"] == "lost")
    seen = f["detected"] > 0
    assert lost["t"] - f["t"][seen & (f["t"] < lost["t"])][-1] == pytest.approx(
        FOLLOWING["ideal_follower"]["coast_s"], abs=0.02
    )


def test_requirements_record_is_current_and_its_checks_pass():
    record = json.loads((R.OUTPUT / "requirements.json").read_text())
    assert record["following_sha256"] == R._sha(FOLLOWING)
    assert record["camera"] == SHARED["m12"]["camera"]
    assert record["check_run"]["all_expectations_met"]
    assert record["pitch_allowance_covers_lean"]
    peaks, targets = record["envelope_run"]["peaks"], record["phase_b_targets"]
    for key in ("speed", "acceleration", "yaw_rate", "yaw_acceleration"):
        assert targets[key] >= peaks[key]
    render = record["render"]
    assert all(render["repeat_identical"])
    agreement = render["geometry_agrees_with_render"]
    assert agreement["checked"] > 0 and agreement["agree"] == agreement["checked"]
    frames = json.loads((R.OUTPUT / "frames/frames.json").read_text())["frames"]
    assert len(frames) == record["frames"]
    assert all((R.OUTPUT / "frames" / row["file"]).exists() for row in frames)


def test_rendered_frame_depends_on_the_state_alone():
    m, d = world(occluder=True)
    try:
        r = S.renderer(m, (90, 160))
    except (RuntimeError, ImportError, mujoco.FatalError) as error:  # no offscreen GL
        pytest.skip(f"offscreen rendering unavailable: {error}")

    def put(i):
        S.place_robot(m, d, P, 0.3 * i, 0.1, 0.05 * i, pitch_offset_deg=1.0)
        S.place_person(m, d, "target", 0.3 * i + 1.1, 0.2, math.pi)
        S.place_person(m, d, "occluder", 0.3 * i + 0.6, 0.5 - 0.1 * i, 0.0)
        mujoco.mj_forward(m, d)

    images = {}
    for order in (range(6), reversed(range(6))):
        for i in order:
            put(i)
            image = S.frame(r, d)
            if i in images:
                np.testing.assert_array_equal(image, images[i])
            images[i] = image
    r.close()
