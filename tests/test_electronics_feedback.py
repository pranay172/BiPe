import json
import math

import numpy as np
import pytest

from bipe.articulated import build, reset
from bipe.parameters import ROOT, load_parameters
from bipe.sensing import JointEncoderFeedback, SensorEstimator


def test_can_feedback_resolution_does_not_degrade_the_local_servo_encoder():
    p = load_parameters()
    m, d = build(p)
    reset(m, d, p)
    config = json.loads((ROOT / "config/sensors-m4.json").read_text())
    estimator = SensorEstimator(m, d, p, config, 1)
    local = JointEncoderFeedback(d, config)
    raw = float(d.sensor("left_hip_encoder").data[0])
    host_increment = math.radians(0.1)
    local_increment = 2 * math.pi / 65536
    assert estimator.encoders(d)[0] == pytest.approx(
        np.round(raw / host_increment) * host_increment
    )
    q, _ = local.update(d, 0.001)
    assert q[0] == pytest.approx(np.round(raw / local_increment) * local_increment)
    assert abs(q[0] - raw) <= local_increment / 2
    assert local.increment < host_increment / 10


def test_m3_encoder_default_is_preserved():
    p = load_parameters()
    m, d = build(p)
    reset(m, d, p)
    config = json.loads((ROOT / "config/sensors.json").read_text())
    assert JointEncoderFeedback(d, config).increment == 2 * math.pi / 65536
