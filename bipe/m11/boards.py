"""M11 board placement helpers: board points in the body frame.

The KiCad layouts (artifacts/m11/boards/<board>/envelopes.json, board millimetres) are
placed with m10.electronics.boards: origin_body_m is the KiCad (0, 0) corner on the
board's back face, and a point maps as origin + x board_x_axis + y board_y_axis +
height front_normal, the height measured from the back face.
"""

import json

import numpy as np

from ..parameters import ROOT

BOARDS = ROOT / "artifacts/m11/boards"


def to_body(board, x_mm, y_mm, height_m):
    """Body-frame point (m) of a board point (KiCad mm) at a height above the back face."""
    o = np.array(board["origin_body_m"])
    ax, ay, n = (
        np.array(board[k], float)
        for k in ("board_x_axis_body", "board_y_axis_body", "front_normal_body")
    )
    return o + ax * x_mm / 1000 + ay * y_mm / 1000 + n * height_m


def part_centre(m10, board_key, ref):
    """Body-frame centre (m) of a part's body box on a board."""
    boards = m10["electronics"]["boards"]
    board = boards[board_key]
    env = json.loads((BOARDS / board_key / "envelopes.json").read_text())
    part = next(p for p in env["parts"] if p["ref"] == ref)
    x0, x1, y0, y1 = part["box_mm"]
    h = boards["component_heights_m"][part["footprint"]]
    w0, w1 = (h[0], h[1]) if isinstance(h, list) else (0.0, h)
    t = board["thickness_m"]
    return to_body(board, (x0 + x1) / 2, (y0 + y1) / 2, t + (w0 + w1) / 2)


def imu_position(m10):
    """The LSM6DSO (carrier U3) package centre in the body frame, m."""
    return part_centre(m10, "carrier", "U3")
