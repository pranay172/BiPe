"""M12 phase C AprilTag detection: the tag image, the detector and the tag pose.

The detector is pupil-apriltags, Python bindings to the AprilRobotics apriltag C library
(the same C code a Raspberry Pi would run). The tag image is drawn from that library's
family definition (code words and bit layout), so the rendered tag is the code the
detector decodes.

The detector wrapper and the pose fit are the Pi's code (bipe.pi.apriltag, bipe.pi.pose),
re-exported here for the simulation.
"""

import math

import numpy as np

from ..pi.apriltag import Detector, _AprilTag
from ..pi.pose import fit_pose, object_points  # noqa: F401


def tag_image(family, tag_id):
    """The tag's image from the C library's family definition: 0 black, 255 white, the
    white border included. Drawn as apriltag_to_image draws it: a white outer ring, the
    black square, and each code bit that is set as a white cell at (bit_x, bit_y)."""
    det = _AprilTag(families=family)
    fam = det.tag_families[family].contents
    n = fam.total_width
    start = (n - fam.width_at_border) // 2
    if fam.reversed_border:
        raise ValueError("families with a reversed border are not drawn")
    out = np.full((n, n), 255, dtype=np.uint8)
    out[start : n - start, start : n - start] = 0
    code = fam.codes[tag_id]
    for i in range(fam.nbits):
        if code >> (fam.nbits - i - 1) & 1:
            out[fam.bit_y[i] + start, fam.bit_x[i] + start] = 255
    return out


class Tracker:
    """Full-frame search, or a region of interest around the last detection at full
    resolution, falling back to the full frame when the region misses."""

    def __init__(self, cfg, tag_id, quad_decimate, roi):
        self.full = Detector(cfg, tag_id, quad_decimate)
        self.local = Detector(cfg, tag_id, 1.0) if roi else None
        self.roi_cfg = cfg["roi"]
        self.last = None

    def reset(self):
        self.last = None

    def window(self, shape):
        c = self.last["corners"]
        lo, hi = c.min(axis=0), c.max(axis=0)
        centre, half = (lo + hi) / 2, (hi - lo) / 2 * self.roi_cfg["scale"]
        half = half + self.roi_cfg["margin_px"]
        x0, y0 = np.maximum(0, np.floor(centre - half)).astype(int)
        x1 = int(min(shape[1], math.ceil(centre[0] + half[0])))
        y1 = int(min(shape[0], math.ceil(centre[1] + half[1])))
        return x0, y0, x1, y1

    def detect(self, image):
        """Detection, total detector time and whether the region was used."""
        spent = 0.0
        if self.local is not None and self.last is not None:
            x0, y0, x1, y1 = self.window(image.shape)
            hit, t, _ = self.local.detect(image[y0:y1, x0:x1], (x0, y0))
            spent += t
            if hit is not None:
                self.last = hit
                return hit, spent, True
        hit, t, _ = self.full.detect(image)
        spent += t
        self.last = hit
        return hit, spent, False


# MuJoCo cameras look along -z with y up; the image frame looks along +z with y down.
MUJOCO_TO_IMAGE = np.diag([1.0, -1.0, -1.0])


def world_point(cam_pos, cam_xmat, t_image):
    """A point in the image camera frame to world coordinates."""
    return np.asarray(cam_pos) + cam_xmat.reshape(3, 3) @ (MUJOCO_TO_IMAGE @ t_image)


def range_bearing(cam_pos, body_yaw, point):
    """Horizontal range from the camera and bearing in the robot's heading frame."""
    rel = np.asarray(point)[:2] - np.asarray(cam_pos)[:2]
    bearing = math.atan2(rel[1], rel[0]) - body_yaw
    return float(np.hypot(*rel)), float((bearing + math.pi) % (2 * math.pi) - math.pi)
