"""The Pi's tag tracker: the phase C pipeline (signed-off timing, 22 October 2026).

While the last frame had the tag, only a region of interest around it is searched, at full
resolution, and a miss there is reported at once; after more than the policy's
region_retries misses in a row (0: the phase C pipeline) the next frame is a full-frame
search at the acquisition decimation. A frame the view gate drops still counts as having
the tag here: the gate acts on the pose, after the tracker.

The window is the last detection's bounding box grown by the scale, plus the margin, with
the margin grown by how far the tag can move across the image in the detection's age at
the policy's bearing rate (growth_rad_s 0 in the phase C policy).
"""

import math

import numpy as np


class RegionTracker:
    """full, local: bipe.pi.apriltag.Detector-like objects (detect(image, offset) returns
    (hit or None, seconds, count)); roi: {"scale", "margin_px"}; policy: {"region_retries",
    "growth_rad_s"}; f_px: the lens focal length in output pixels."""

    def __init__(self, full, local, roi, policy, f_px):
        self.full, self.local = full, local
        self.roi = roi
        self.policy = policy
        self.f_px = f_px
        self.reset()

    def reset(self):
        self.last, self.last_t, self.misses = None, -math.inf, 0

    @property
    def tracking(self):
        return self.last is not None

    def window(self, shape, age_s):
        """(x0, y0, x1, y1) of the region of interest in an image of the given shape."""
        grow = self.policy["growth_rad_s"] * age_s * self.f_px
        c = self.last["corners"]
        lo, hi = c.min(axis=0), c.max(axis=0)
        centre, half = (lo + hi) / 2, (hi - lo) / 2 * self.roi["scale"]
        half = half + self.roi["margin_px"] + grow
        x0, y0 = np.maximum(0, np.floor(centre - half)).astype(int)
        x1 = int(min(shape[1], math.ceil(centre[0] + half[0])))
        y1 = int(min(shape[0], math.ceil(centre[1] + half[1])))
        return x0, y0, x1, y1

    def detect(self, image, t):
        """(hit or None, detector seconds, whether the region of interest was used) for the
        frame at time t (seconds, any fixed origin)."""
        if self.last is None:
            hit, spent, _ = self.full.detect(image)
            self.last = hit
            self.last_t, self.misses = t, 0
            return hit, spent, False
        x0, y0, x1, y1 = self.window(image.shape, t - self.last_t)
        hit, spent, _ = self.local.detect(image[y0:y1, x0:x1], (x0, y0))
        if hit is not None:
            self.last, self.last_t, self.misses = hit, t, 0
        else:
            self.misses += 1
            if self.misses > self.policy["region_retries"]:
                self.last = None
        return hit, spent, True
