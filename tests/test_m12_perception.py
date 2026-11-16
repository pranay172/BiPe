import copy
import json
import math

import numpy as np
import pytest

from bipe.m12 import camera as C
from bipe.m12 import detector as D
from bipe.m12 import perception as PC
from bipe.parameters import ROOT

SHARED, FOLLOWING, P, PERC = PC.load()
MODEL = C.intrinsics(SHARED, PERC)
WIDE = SHARED["m12"]["camera"]["lenses"]["wide"]
SIZE = FOLLOWING["tag"]["size_m"]


@pytest.fixture(scope="module")
def scene():
    sc = PC.Scene("waist_tag")
    yield sc
    sc.close()


def test_rendered_tag_is_detected_with_a_small_error(scene):
    sc = scene
    sc.place((0.0, 0.0, 0.0, 0.0, 40), (10.0, 10.0), 0.0)
    cam = sc.d.cam_xpos[sc.cid].copy()
    tag = cam[:2] + [1.0, 0.1]
    sc.place((0.0, 0.0, 0.0, 0.0, 40), tag)
    truth = sc.truth()
    # The truth is the tag's face, which stands sc.tag_front proud of the person's origin.
    assert truth["range_m"] == pytest.approx(math.hypot(1.0, 0.1) - sc.tag_front, abs=1e-5)
    assert truth["in_frame"] and truth["view_deg"] < 0.1
    image = sc.camera.develop(sc.camera.linear(sc.d), 0.01, np.random.default_rng(0))
    result = sc.detect_all(image, truth)
    for method in PC.METHODS:
        r = result[method]
        assert r is not None
        # The sRGB tone curve narrows the black square (about 1.5 % of range at 1 m).
        assert 0 < r["range_error_m"] < 0.03
        assert abs(r["bearing_error_deg"]) < 0.3


def test_exposure_depends_on_the_state_and_seed_alone(scene):
    sc = scene
    sc.place((0.1, 0.0, 0.2, 2.0, 40), (1.3, 0.3))

    def frame():
        return sc.camera.develop(sc.camera.linear(sc.d), 0.01, np.random.default_rng(5))

    first = frame()
    sc.place((0.0, 0.0, 0.0, 0.0, 40), (2.0, -0.4))
    frame()
    sc.place((0.1, 0.0, 0.2, 2.0, 40), (1.3, 0.3))
    assert np.array_equal(frame(), first)


def test_region_of_interest_falls_back_to_the_full_frame(scene):
    sc = scene
    sc.place((0.0, 0.0, 0.0, 0.0, 40), (1.1, 0.0))
    image = sc.camera.develop(sc.camera.linear(sc.d), 0.01, None)
    tracker = D.Tracker(PERC["detector"], 0, 2.0, True)
    hit, _, used = tracker.detect(image)
    assert hit is not None and not used
    hit, _, used = tracker.detect(image)
    assert hit is not None and used
    # Move the tag out of the region: the region misses and the full frame finds it.
    sc.place((0.0, 0.0, 0.0, 0.0, 40), (1.1, 0.5))
    moved = sc.camera.develop(sc.camera.linear(sc.d), 0.01, None)
    hit, _, used = tracker.detect(moved)
    assert hit is not None and not used


def test_model_lookup_returns_the_table_at_its_nodes():
    model = {
        "range_m": [0.5, 1.0],
        "view_deg": [0, 30],
        "table": [
            {
                "range_m": r,
                "view_deg": v,
                "p_detect_tracking": 1 - v / 100,
                "p_detect_acquisition": 1.0,
                "range_bias_m": r / 100,
                "range_std_m": 0.001,
                "bearing_bias_deg": 0.0,
                "bearing_std_deg": 0.01 * (1 + v / 30),
            }
            for r in (0.5, 1.0)
            for v in (0, 30)
        ],
    }
    m = PC.model_lookup(model, 1.0, 30)
    assert m["p_detect_tracking"] == pytest.approx(0.7)
    assert m["bearing_std_deg"] == pytest.approx(0.02)
    mid = PC.model_lookup(model, 0.75, 15)
    assert mid["range_bias_m"] == pytest.approx(0.0075)
    assert mid["bearing_std_deg"] == pytest.approx(0.015)


def test_perception_record_is_current_and_carries_the_measurement_model():
    summary = json.loads((ROOT / "artifacts/m12/perception/summary.json").read_text())
    inputs = copy.deepcopy(PERC)
    model = inputs.pop("measurement_model")
    assert summary["perception_inputs_sha256"] == PC._sha(inputs)
    assert model is not None and model["layout"] == PERC["replays"]["layout"]
    assert {e["range_m"] for e in model["table"]} == set(PERC["grid"]["ranges_m"])
    assert "rendered frames" in model["evidence"]
    config = json.loads((ROOT / "artifacts/m12/perception/config.json").read_text())
    assert config["intrinsics"]["f_px"] == pytest.approx(MODEL["f_px"])
    timing = json.loads((ROOT / "artifacts/m12/perception/timing.json").read_text())
    assert "estimate" in timing["evidence"]


def test_motion_bias_grows_with_the_square_of_the_blur_and_the_range():
    model = PERC["measurement_model"]
    blur = model["motion_range_bias"]
    assert blur["coefficient_m"] > 0 and blur["fit"]["n"] >= 8
    one = PC.motion_bias(model, 1.0, 1.0, 0.01)
    assert PC.motion_bias(model, 1.0, -2.0, 0.01) == pytest.approx(4 * one)
    assert PC.motion_bias(model, 2.0, 1.0, 0.01) == pytest.approx(4 * one)
    assert PC.motion_bias({}, 1.0, 1.0, 0.01) == 0.0
