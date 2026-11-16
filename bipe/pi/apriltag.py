"""The AprilTag detector on the Pi: pupil-apriltags, Python bindings to the AprilRobotics
apriltag C library. One configuration per Detector; detect() keeps the configured tag's
best detection (exact code match up to max_hamming, highest decision margin)."""

import time

import numpy as np
import pupil_apriltags


class _AprilTag(pupil_apriltags.Detector):
    """pupil_apriltags.Detector with a safe destructor. Its __del__ frees the tag families
    before apriltag_detector_destroy, which then reads each family's quick-decode table:
    a use-after-free that crashes intermittently. Detaching the families first leaves the
    destroy with nothing to read."""

    def __del__(self):
        if getattr(self, "tag_detector_ptr", None) is not None:
            self.libc.apriltag_detector_clear_families.restype = None
            self.libc.apriltag_detector_clear_families(self.tag_detector_ptr)
        super().__del__()


class Detector:
    """One detector configuration; detect() returns the tag's detection or None and the
    time the C library took."""

    def __init__(self, cfg, tag_id, quad_decimate, threads=1, family="tag36h11"):
        self.cfg = cfg
        self.tag_id = tag_id
        self.det = _AprilTag(
            families=family,
            nthreads=threads,
            quad_decimate=quad_decimate,
            quad_sigma=cfg["quad_sigma"],
            refine_edges=int(cfg["refine_edges"]),
            decode_sharpening=cfg["decode_sharpening"],
        )

    def detect(self, image, offset=(0, 0)):
        t0 = time.perf_counter()
        found = self.det.detect(np.ascontiguousarray(image))
        elapsed = time.perf_counter() - t0
        best = None
        for d in found:
            if d.tag_id != self.tag_id or d.hamming > self.cfg["max_hamming"]:
                continue
            if best is None or d.decision_margin > best.decision_margin:
                best = d
        if best is None:
            return None, elapsed, len(found)
        return (
            {
                "corners": best.corners + np.asarray(offset, float),
                "center": best.center + np.asarray(offset, float),
                "decision_margin": float(best.decision_margin),
                "hamming": int(best.hamming),
            },
            elapsed,
            len(found),
        )
