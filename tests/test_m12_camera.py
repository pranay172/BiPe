import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bipe.m12 import camera as C
from bipe.m12 import detector as D
from bipe.m12 import requirements as R
from bipe.parameters import ROOT

SHARED, FOLLOWING, P = R.load()
PERC = json.loads((ROOT / "config/perception-m12.json").read_text())
MODEL = C.intrinsics(SHARED, PERC)
WIDE = SHARED["m12"]["camera"]["lenses"]["wide"]
SIZE = FOLLOWING["tag"]["size_m"]


def test_intrinsics_follow_the_datasheet_and_the_output_mode():
    # 2.75 mm over 4 x 1.4 um output pixels.
    assert MODEL["f_px"] == pytest.approx(0.00275 / 5.6e-6)
    assert (MODEL["width_px"], MODEL["height_px"]) == (1152, 648)
    # The lens model puts the documented half fields at the image edges.
    for half_px, fov in ((576, WIDE["horizontal_fov_deg"]), (324, WIDE["vertical_fov_deg"])):
        ru = math.tan(math.radians(fov / 2))
        assert C.distort_radius(MODEL, ru) * MODEL["f_px"] == pytest.approx(half_px)
    assert C.monotonic_over_image(MODEL)
    # Two documented angles fix k1 and k2; the diagonal they imply is recorded against the
    # product page's 120 degrees.
    assert 110 < MODEL["diagonal_fov_deg"] < 120


def test_distortion_round_trips_over_the_image():
    u, v = np.meshgrid(np.linspace(0, 1151, 9), np.linspace(0, 647, 7))
    xu, yu = C.normalised(MODEL, u, v)
    u2, v2 = C.project(MODEL, xu, yu)
    assert np.max(np.abs(u2 - u)) < 1e-9 and np.max(np.abs(v2 - v)) < 1e-9


def test_render_covers_the_whole_distorted_output():
    geo = C.render_geometry(MODEL, 2)
    rows, cols = geo["map"]
    assert rows.min() >= 0 and cols.min() >= 0
    assert rows.max() <= geo["height"] - 1 and cols.max() <= geo["width"] - 1
    # Two render pixels per output pixel at the image centre.
    assert geo["f_px"] == pytest.approx(2 * MODEL["f_px"])


def test_drawn_tag_is_the_library_code():
    image = D.tag_image("tag36h11", 0)
    assert image.shape == (10, 10)
    assert np.all(image[0] == 255) and np.all(image[1, 1:9] == 0)
    big = np.pad(np.kron(image, np.ones((20, 20), np.uint8)), 60, constant_values=255)
    hit, _, _ = D.Detector(PERC["detector"], 0, 1.0).detect(big)
    assert hit is not None and hit["hamming"] == 0


def test_black_runs_cover_the_black_cells_once():
    image = D.tag_image("tag36h11", 0)
    covered = np.zeros_like(image, dtype=int)
    for r, c, n in C.black_runs(image):
        covered[r, c : c + n] += 1
    assert np.array_equal(covered, (image < 128).astype(int))


def test_pose_fit_recovers_a_known_pose_through_the_lens():
    rot = Rotation.from_euler("xyz", [10, -35, 4], degrees=True).as_matrix()
    t = np.array([0.25, -0.1, 1.2])
    pts = D.object_points(SIZE) @ rot.T + t
    u, v = C.project(MODEL, pts[:, 0] / pts[:, 2], pts[:, 1] / pts[:, 2])
    pose = D.fit_pose(MODEL, np.c_[u, v], SIZE)
    assert pose["translation"] == pytest.approx(t, abs=1e-7)
    assert pose["reprojection_rms_px"] < 1e-6


def test_electron_budget_matches_the_hand_arithmetic():
    n = PERC["camera"]["noise"]
    e = C.electrons_at_white(n, 0.01, 2.2, 16, (1.4e-6) ** 2)
    # 200 lx x 0.9 / (4 x 2.2^2) = 9.30 lx at the sensor; x 4.1e15 x 1.96e-12 m^2 x 0.3 QE
    # x 10 ms x 16 native pixels.
    assert e == pytest.approx(9.2975 * 4.1e15 * 1.96e-12 * 0.3 * 0.01 * 16, rel=1e-3)


def test_lens_fit_record_keeps_the_wide_module_clear_at_the_current_pitch():
    record = json.loads((ROOT / "artifacts/m12/perception/lens_fit.json").read_text())
    cases = {(c["lens"], c["camera_pitch_deg"]): c for c in record["cases"]}
    assert cases[("wide", 15)]["fits"]
    assert cases[("wide", 15)]["module_depth_mm"] == 12.4
    # The 42 degree waist-tag pitch needs the phase E head change.
    assert not cases[("wide", 42)]["fits"]
