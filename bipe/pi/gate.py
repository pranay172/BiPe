"""The view-angle gate (M12 phase F, signed off 24 October 2026).

The Pi drops a tag pose whose fitted 3-D view angle, between the tag's normal and the line
of sight to the camera, is past the gate: edge-on poses read short in range (1.4-2.0 cm
past 70 deg of yaw in the phase F renders). The frame is then reported as an empty frame
(detected false), and the tracker keeps the tag, so the next frame is still a
region-of-interest search. The head camera looks down on the waist tag, so facing the
person reads about 38 deg; the 74 deg gate is about 69-70 deg of yaw view angle.

The threshold is config/following-rendered-m12.json view_gate_deg, the signed-off value.
"""

import json
import math
from pathlib import Path

import numpy as np

CONFIG = Path(__file__).resolve().parents[2] / "config/following-rendered-m12.json"


def view_angle(pose):
    """The angle, in radians, between the fitted tag's normal (the rotation's third column)
    and the line of sight from the tag to the camera. pose: bipe.pi.pose.fit_pose's dict,
    in the camera frame. The sign of the normal does not matter."""
    normal = pose["rotation"][:, 2]
    sight = -np.asarray(pose["translation"], float)
    c = float(abs(normal @ sight) / np.linalg.norm(sight))
    if not math.isfinite(c):
        return math.nan  # min() below would turn a NaN into 0 deg
    return math.acos(min(1.0, c))


class ViewGate:
    """gate_deg None: no gate (every pose passes)."""

    def __init__(self, gate_deg):
        if gate_deg is not None and not (math.isfinite(gate_deg) and 0 <= gate_deg <= 90):
            raise ValueError(f"view gate {gate_deg!r} deg is outside 0-90")
        self.deg = gate_deg
        self.rad = None if gate_deg is None else math.radians(gate_deg)

    @classmethod
    def from_config(cls, path=CONFIG):
        return cls(json.loads(Path(path).read_text())["view_gate_deg"])

    def check(self, pose):
        """(gated, view angle in radians). With a gate, a pose whose angle is not finite is
        gated too."""
        view = view_angle(pose)
        if self.rad is None:
            return False, view
        return not math.isfinite(view) or view > self.rad, view
