import csv
import json

import pytest

from bipe.m11 import boards
from bipe.parameters import ROOT

BOARDS = ROOT / "artifacts/m11/boards"


@pytest.fixture(scope="module")
def cfg():
    return json.loads((ROOT / "config/robot.json").read_text())["m10"]["electronics"]["boards"]


@pytest.mark.parametrize("board", ["carrier", "power"])
def test_committed_board_passes_erc_drc_and_parity(board):
    report = json.loads((BOARDS / board / "report.json").read_text())
    assert report["erc_violations"] == 0
    assert report["drc_violations"] == 0
    assert report["drc_unconnected"] == 0
    assert report["schematic_parity_issues"] == 0
    assert report["passed"]


@pytest.mark.parametrize("board", ["carrier", "power"])
def test_board_outline_matches_the_config(cfg, board):
    audit = json.loads((BOARDS / board / "pin-audit.json").read_text())
    assert audit["size_mm"] == pytest.approx([1000 * v for v in cfg[board]["size_m"]])
    # The board axes are orthonormal and the front normal is their cross product.
    x, y, n = (
        cfg[board][k] for k in ("board_x_axis_body", "board_y_axis_body", "front_normal_body")
    )
    assert sum(a * b for a, b in zip(x, y)) == 0
    cross = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]]
    # KiCad draws y downwards, so the front (viewer) side is -(x cross y).
    assert cross == [-v for v in n]


@pytest.mark.parametrize("board", ["carrier", "power"])
def test_jlc_placement_covers_every_bom_part(board):
    with (BOARDS / board / "bom-jlc.csv").open() as f:
        bom = {r for row in csv.DictReader(f) for r in row["Designator"].split(",")}
    report = json.loads((BOARDS / board / "report.json").read_text())
    assert bom - set(report["hand_fit"]) == set(report["jlc_placed"])


def test_plant_imu_position_is_the_carrier_imu_centre():
    m10 = json.loads((ROOT / "config/robot.json").read_text())["m10"]
    assert m10["physics"]["imu_position_m"] == pytest.approx(
        list(boards.imu_position(m10)), abs=1e-4
    )


def test_board_points_map_through_the_back_face_origin(cfg):
    b = cfg["power"]
    # KiCad (0, 0) on the back face; the front face is one thickness along the normal.
    assert list(boards.to_body(b, 0, 0, 0)) == pytest.approx(b["origin_body_m"])
    front = boards.to_body(b, 0, 0, b["thickness_m"]) - boards.to_body(b, 0, 0, 0)
    assert list(front) == pytest.approx([b["thickness_m"] * v for v in b["front_normal_body"]])
