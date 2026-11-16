import json
import math

import pytest

from bipe.m11 import harness
from bipe.parameters import ROOT


@pytest.fixture(scope="module")
def m10():
    return json.loads((ROOT / "config/robot.json").read_text())["m10"]


def test_stranded_resistance_is_resistivity_over_the_strand_area(m10):
    h = m10["electronics"]["harness"]
    wire = {"strands": 7, "strand_d_m": 0.0001}
    per_m, area = harness.wire_resistance(h, wire)
    assert area == pytest.approx(7 * math.pi * 0.0001**2 / 4)
    assert per_m == pytest.approx(h["copper_resistivity_ohm_m_20C"] * h["lay_factor"] / area)


def test_bundle_diameter_follows_the_packing_fraction_and_gates_each_restriction(m10):
    h = dict(m10["electronics"]["harness"], packing_fraction=[0.5])
    h["restrictions_m"] = {"tight": 0.002, "loose": 0.004}
    b = harness.bundle(h, 0.001, 0.0005)
    expected = math.sqrt((3 * 0.001**2 + 6 * 0.0005**2) / 0.5)
    row = b["packings"]["packing_0.5"]
    assert row["bundle_diameter_m"] == pytest.approx(expected)
    assert row["fits"] == {"tight": expected <= 0.002, "loose": True}
    assert b["fits_everywhere"] is (expected <= 0.002)


def test_crossing_strain_uses_the_end_stop_range_and_the_inner_wire_radius(m10):
    h = m10["electronics"]["harness"]
    stops = m10["physics"]["end_stops_deg"]
    out = harness.crossings(h, stops)["hip_gallery_port"]
    lo, hi = stops["knee_angle"]
    assert out["range_deg"] == pytest.approx(hi - lo)
    assert out["loop_arc_change_m"] == pytest.approx(
        h["service_loop_radius_m"] * math.radians(hi - lo)
    )
    w = h["wires"]["phase"]
    assert out["strand_strain"]["phase"]["direct_wrap"] == pytest.approx(
        w["strand_d_m"] / (2 * (0.005 + w["od_m"] / 2))
    )
    # A service loop always lowers the strain against wrapping on the shaft.
    for kind in ("phase", "encoder"):
        s = out["strand_strain"][kind]
        assert s["service_loop"] < s["direct_wrap"]
