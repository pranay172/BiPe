"""The Pi's frame pipeline (bipe.pi) on a host build: the 74 deg view gate, the tracker
policy around it, the pose and the mount conversion. Synthetic frames are drawn through a
pinhole camera and read by the real pupil-apriltags detector. Real footage is phase E."""

import json
import math
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bipe.m12 import detector as D
from bipe.pi import lens
from bipe.pi.apriltag import Detector
from bipe.pi.gate import CONFIG, ViewGate, view_angle
from bipe.pi.pipeline import FixedMount, FramePipeline
from bipe.pi.tracker import RegionTracker

PERCEPTION = json.loads((CONFIG.parent / "perception-m12.json").read_text())
DET = PERCEPTION["detector"]
SIZE = 0.16
W, H = 1152, 648
# A pinhole model with a long focal length, so edge-on tags still have enough pixels to
# decode and the gate (not the detector) decides.
PINHOLE = {"f_px": 1400.0, "cx_px": W / 2, "cy_px": H / 2, "k1": 0.0, "k2": 0.0}
PHASE_C = {"region_retries": 0, "growth_rad_s": 0.0}


def _tag_pose(range_m, view_deg, offset_px=(0.0, 0.0)):
    """A tag at range_m along the line of sight through pixel offset_px from the centre,
    turned about the vertical by view_deg from facing the camera."""
    ray = np.array([offset_px[0] / PINHOLE["f_px"], offset_px[1] / PINHOLE["f_px"], 1.0])
    ray /= np.linalg.norm(ray)
    # Facing the camera: the tag's z axis along the line of sight (into the tag); then
    # turned about the tag's own y axis, across the line of sight.
    facing = Rotation.align_vectors([ray], [[0, 0, 1]])[0]
    rot = (facing * Rotation.from_euler("y", view_deg, degrees=True)).as_matrix()
    return rot, ray * range_m


def _frame(rot, t, model=PINHOLE, size=SIZE):
    """The tag drawn into a grey frame through the camera model (2 x 2 supersampled): each
    sample's pixel is undistorted through the lens (bipe.pi.lens) and traced to the tag's
    plane. Noise-free and blur-free, so these frames are the best case for the fit."""
    tag = D.tag_image("tag36h11", 0)
    n = tag.shape[0]
    cell = size / (n - 2)  # the black square spans n - 2 cells
    # Homography from tag-plane metres to undistorted normalised coordinates, inverted.
    inv = np.linalg.inv(np.c_[rot[:, 0], rot[:, 1], t])
    ss = 2
    v, u = np.mgrid[0 : H * ss, 0 : W * ss]
    xu, yu = lens.normalised(model, (u + 0.5) / ss - 0.5, (v + 0.5) / ss - 0.5)
    pts = np.stack([xu, yu, np.ones_like(xu)])
    plane = np.tensordot(inv, pts, axes=1)
    x, y = plane[0] / plane[2], plane[1] / plane[2]
    # Row 0 of the tag image at the tag's -y edge (image up when facing the camera), so the
    # detector decodes it unmirrored.
    col = np.floor(x / cell + n / 2).astype(int)
    row = np.floor(y / cell + n / 2).astype(int)
    inside = (col >= 0) & (col < n) & (row >= 0) & (row < n) & (plane[2] > 0)
    img = np.full(x.shape, 128.0)
    img[inside] = tag[row[inside], col[inside]]
    img = img.reshape(H, ss, W, ss).mean(axis=(1, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


def _pipeline(gate_deg=74, policy=PHASE_C, model=PINHOLE):
    full = Detector(DET, 0, 2.0, threads=1)
    local = Detector(DET, 0, 1.0)
    tracker = RegionTracker(full, local, DET["roi"], policy, model["f_px"])
    return FramePipeline(tracker, model, SIZE, ViewGate(gate_deg), FixedMount(0.0))


def test_the_pi_package_does_not_need_the_simulator():
    code = (
        "import sys, bipe.pi.pipeline, bipe.pi.gate, bipe.pi.tracker, bipe.pi.apriltag;"
        "print(any(m == 'mujoco' or m.startswith('bipe.m') for m in sys.modules))"
    )
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"


def test_the_gate_is_the_signed_off_74_degrees():
    assert ViewGate.from_config().deg == 74
    with pytest.raises(ValueError):
        ViewGate(95)
    with pytest.raises(ValueError):
        ViewGate(float("nan"))


@pytest.mark.parametrize("view_deg", [0.0, 30.0, 60.0, 73.9, 74.1, 85.0])
def test_the_view_angle_is_the_tag_normal_against_the_line_of_sight(view_deg):
    rot, t = _tag_pose(1.2, view_deg, offset_px=(300.0, -150.0))
    pose = {"rotation": rot, "translation": t}
    assert math.degrees(view_angle(pose)) == pytest.approx(view_deg, abs=1e-6)
    # The normal's sign does not matter (a flipped fit reads the same angle).
    flipped = {"rotation": rot @ np.diag([1.0, -1.0, -1.0]), "translation": t}
    assert view_angle(flipped) == pytest.approx(view_angle(pose))
    gated, _ = ViewGate(74).check(pose)
    assert gated == (view_deg > 74)
    assert ViewGate(None).check(pose)[0] is False


def test_a_pose_that_is_not_finite_is_gated():
    pose = {"rotation": np.full((3, 3), np.nan), "translation": np.array([0.0, 0.0, 1.0])}
    assert ViewGate(74).check(pose)[0]
    assert not ViewGate(None).check(pose)[0]


@pytest.mark.parametrize("view_deg, gated", [(40.0, False), (65.0, False), (82.0, True)])
def test_detected_frames_pass_or_are_dropped_by_their_fitted_angle(view_deg, gated):
    rot, t = _tag_pose(1.0, view_deg)
    pipe = _pipeline()
    report, record = pipe.process(_frame(rot, t), 0.0, 1, 0.0)
    assert record["detected"]
    assert record["measured_view_deg"] == pytest.approx(view_deg, abs=3.0)
    assert record["gated"] is gated
    assert report["detected"] is (not gated)
    if gated:
        assert report["range_m"] == 0.0 and report["bearing_rad"] == 0.0
    else:
        assert report["range_m"] == pytest.approx(1.0, abs=0.03)
    assert set(report) == {"sequence", "tag_id", "detected", "range_m", "bearing_rad", "age_s"}


def test_a_gated_frame_keeps_the_tag_for_the_region_of_interest():
    pipe = _pipeline()
    facing, edge_on, turning = _tag_pose(1.0, 40.0), _tag_pose(1.0, 82.0), _tag_pose(1.0, 70.0)
    # The window after the gated frames is the edge-on detection's (narrow) box, so the tag
    # coming back is at 70 deg, which still fits it.
    sequence = [facing, edge_on, edge_on, turning]
    out = [pipe.process(_frame(*pose), k / 30, k + 1, 0.0) for k, pose in enumerate(sequence)]
    reports = [r for r, _ in out]
    records = [f for _, f in out]
    assert [r["detected"] for r in reports] == [True, False, False, True]
    assert [f["gated"] for f in records] == [False, True, True, False]
    # The first frame is a full search; every later one stays a region-of-interest search.
    assert not records[0]["roi_used"]
    assert all(f["tracking"] and f["roi_used"] for f in records[1:])
    # Without the gate the same edge-on frames are sent.
    plain = _pipeline(gate_deg=None)
    sent = [plain.process(_frame(*pose), k / 30, k + 1, 0.0)[0] for k, pose in enumerate(sequence)]
    assert all(r["detected"] for r in sent)


@pytest.mark.parametrize("offset_px", [(0.0, 0.0), (300.0, -150.0)])
def test_the_fitted_angle_sweep(offset_px):
    """The fitted view angle against the drawn one, 0-85 deg, centred and off-centre: under
    0.1 deg from 60 deg up, around the gate, and under 1 deg near facing the camera, where
    the angle is ill-conditioned and far from the gate (clean frames)."""
    errors = {}
    for view_deg in range(0, 86, 5):
        rot, t = _tag_pose(1.0, float(view_deg), offset_px)
        _, record = _pipeline().process(_frame(rot, t), 0.0, 1, 0.0)
        assert record["detected"], view_deg
        errors[view_deg] = abs(record["measured_view_deg"] - view_deg)
    assert max(errors.values()) < 1.0, errors
    assert max(e for v, e in errors.items() if v >= 60) < 0.1, errors


def test_the_gate_through_a_distorted_lens():
    """The Wide lens's barrel distortion (k1, k2 of the order bipe.m12.camera fits), with the
    tag off-centre where it bends most: the pose fit undistorts through bipe.pi.lens."""
    model = dict(PINHOLE, k1=-0.12, k2=0.02)
    assert lens.normalised(model, 0.0, 0.0)[0] < lens.normalised(model, 1.0, 0.0)[0]
    for view_deg, gated in ((65.0, False), (82.0, True)):
        rot, t = _tag_pose(1.0, view_deg, offset_px=(350.0, -150.0))
        report, record = _pipeline(model=model).process(_frame(rot, t, model), 0.0, 1, 0.0)
        assert record["detected"]
        assert record["measured_view_deg"] == pytest.approx(view_deg, abs=0.5)
        assert record["gated"] is gated and report["detected"] is (not gated)


@pytest.mark.parametrize("lean", [None, float("nan"), float("inf"), "0"])
def test_the_lean_is_required_and_finite(lean):
    rot, t = _tag_pose(1.0, 40.0)
    with pytest.raises(ValueError):
        _pipeline().process(_frame(rot, t), 0.0, 1, lean)
    with pytest.raises(ValueError):
        FixedMount(0.3).range_bearing(t, lean)
    with pytest.raises(TypeError):
        _pipeline().process(_frame(rot, t), 0.0, 1)  # no lean given


def test_the_lean_moves_the_range():
    """A lean of 2 deg on a 42 deg camera at 1 m changes the range by centimetres: the
    fixed-mount error phase C to D measured, which is why the lean is required."""
    mount = FixedMount(math.radians(42.0))
    t = np.array([0.0, 0.2, 1.0])
    level = mount.range_bearing(t, 0.0)[0]
    leaning = mount.range_bearing(t, math.radians(2.0))[0]
    assert 0.01 < abs(level - leaning) < 0.1


class _Fake:
    """A detector that returns the scripted hits in turn and logs the image shapes."""

    def __init__(self, hits):
        self.hits, self.shapes = list(hits), []

    def detect(self, image, offset=(0, 0)):
        self.shapes.append(image.shape)
        hit = self.hits.pop(0)
        return hit, 0.001, int(hit is not None)


def _hit(x=500.0, y=300.0, side=40.0):
    c = np.array([[x, y + side], [x + side, y + side], [x + side, y], [x, y]])
    return {"corners": c, "center": c.mean(axis=0), "decision_margin": 50.0, "hamming": 0}


def test_the_tracker_follows_the_phase_c_policy():
    image = np.zeros((H, W), np.uint8)
    full, local = _Fake([_hit(), _hit()]), _Fake([None])
    tr = RegionTracker(full, local, DET["roi"], PHASE_C, PINHOLE["f_px"])
    assert tr.detect(image, 0.0)[2] is False and tr.tracking
    # A region miss is reported at once and the next frame is a full search.
    hit, _, roi = tr.detect(image, 1 / 30)
    assert hit is None and roi and not tr.tracking
    assert tr.detect(image, 2 / 30)[2] is False and tr.tracking
    assert len(full.shapes) == 2 and full.shapes[0] == (H, W)
    # The window: the 40 px box doubled plus 24 px each side.
    assert local.shapes == [(128, 128)]


def test_region_retries_and_growth():
    policy = {"region_retries": 1, "growth_rad_s": 1.5}
    image = np.zeros((H, W), np.uint8)
    tr = RegionTracker(_Fake([_hit()]), _Fake([None, None]), DET["roi"], policy, 1000.0)
    tr.detect(image, 0.0)
    assert tr.detect(image, 0.1)[0] is None and tr.tracking  # one retry
    x0, _, x1, _ = tr.window(image.shape, 0.2)
    assert x1 - x0 == pytest.approx(128 + 2 * 1.5 * 0.2 * 1000.0, abs=2)
    tr.detect(image, 0.2)
    assert not tr.tracking


@pytest.mark.parametrize("pitch_deg", [0.0, 15.0, 42.0])
def test_the_mount_matches_the_simulation_conversion(pitch_deg):
    """FixedMount against bipe.m12.detector.world_point and range_bearing, for a MuJoCo
    camera pitched down at the mount angle on a robot heading along x."""
    p = math.radians(pitch_deg)
    right = [0.0, -1.0, 0.0]
    up = [math.sin(p), 0.0, math.cos(p)]
    back = [-math.cos(p), 0.0, math.sin(p)]
    xmat = np.array([right, up, back]).T
    cam = np.array([0.3, -0.2, 0.62])
    mount = FixedMount(math.radians(pitch_deg - 5.0))
    rng = np.random.default_rng(0)
    for _ in range(20):
        t = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3), rng.uniform(0.6, 2.5)])
        want = D.range_bearing(cam, 0.0, D.world_point(cam, xmat, t))
        got = mount.range_bearing(t, extra_pitch_rad=math.radians(5.0))
        assert got == pytest.approx(want, abs=1e-12)


def test_the_simulation_uses_the_pi_code():
    from bipe.m12 import rendered_link as RL

    assert RL._view_angle is view_angle
    assert RL.RegionTracker is RegionTracker
    assert RL.FramePipeline is FramePipeline
    assert D.fit_pose.__module__ == "bipe.pi.pose" and D.Detector is Detector
