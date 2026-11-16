"""One frame through the Pi: tracker, pose, view gate, report.

Each processed frame gives one report for the Teensy (firmware/include/follower.h
bipe_target_report): sequence, tag_id, detected, range_m, bearing_rad, age_s. A frame the
view gate drops is an empty frame (detected false, range and bearing 0), as in the
phase F runs; the tracker keeps the tag. age_s, from the exposure's middle to sending,
is filled in by the sender (the link is a phase E item).

The range and bearing come from the tag's position in the camera frame through a mount:
any object with range_bearing(t_image, extra_pitch_rad). On the Pi that is FixedMount, the
head camera pitched down at camera_pitch_rad, on the robot's centre line, with no roll;
the simulator passes its plant-camera mount (bipe.m12.rendered_link), so the phase F runs
went through this report code. Range is horizontal from the camera, bearing is positive
to the left of the heading.

The body's lean is a required input on every frame, and it matters: phase F converted with
the plant's own camera pose (lean included), and the M12 doc's fixed_mount diagnostic, a
lean of 0 on a balancing robot, put +2.7 to +5.9 cm of mean range error (peaks of 20 cm)
into the follower and ended every run in hold. A lean of 0 is that failing diagnostic, not
a safe default. The lean comes from the Teensy (a phase E link item).
"""

import math

import numpy as np

from .pose import fit_pose


def _check_lean(extra_pitch_rad):
    try:
        finite = math.isfinite(extra_pitch_rad)
    except TypeError:
        finite = False
    if not finite:
        raise ValueError(f"the lean must be a finite number of radians, not {extra_pitch_rad!r}")


class FixedMount:
    def __init__(self, camera_pitch_rad):
        if not math.isfinite(camera_pitch_rad):
            raise ValueError("camera pitch must be finite")
        self.pitch = camera_pitch_rad

    def range_bearing(self, t_image, extra_pitch_rad):
        """t_image: a point in the camera frame (x right, y down, z forward), metres.
        extra_pitch_rad: pitch below the nominal (positive down), from the body's lean."""
        _check_lean(extra_pitch_rad)
        x, y, z = np.asarray(t_image, float)
        p = self.pitch + extra_pitch_rad
        forward = z * math.cos(p) - y * math.sin(p)
        left = -x
        return float(math.hypot(forward, left)), float(math.atan2(left, forward))


class FramePipeline:
    """tracker: bipe.pi.tracker.RegionTracker; model: the lens model dict; tag_size_m: the
    tag's black square; gate: bipe.pi.gate.ViewGate; mount: FixedMount, or any object with
    range_bearing(t_image, extra_pitch_rad). The gate's threshold comes from the caller
    (ViewGate.from_config takes the config path), so a packaged install can pass its own."""

    def __init__(self, tracker, model, tag_size_m, gate, mount, tag_id=0):
        self.tracker, self.model, self.size = tracker, model, tag_size_m
        self.gate, self.mount, self.tag_id = gate, mount, tag_id

    def process(self, image, t, sequence, extra_pitch_rad, mount=None):
        """(report, record) for the frame exposed around time t (seconds). extra_pitch_rad:
        the camera's pitch below the mount's nominal from the body's lean, at the exposure
        (required; see the module notes). mount: this frame's mount, in place of the
        pipeline's (the simulator's plant camera moves with the plant). The record keeps
        what the report leaves out: whether the tracker had the tag, the region of interest,
        the raw detection, the fitted view angle and whether it was gated."""
        _check_lean(extra_pitch_rad)
        mount = self.mount if mount is None else mount
        tracking = self.tracker.tracking
        hit, spent, roi_used = self.tracker.detect(image, t)
        report = {
            "sequence": int(sequence),
            "tag_id": self.tag_id,
            "detected": False,
            "range_m": 0.0,
            "bearing_rad": 0.0,
            "age_s": None,
        }
        record = {
            "t": t,
            "sequence": int(sequence),
            "tracking": tracking,
            "roi_used": bool(roi_used),
            "detected": hit is not None,
            "detector_s": float(spent),
            "gated": False,
            "measured_view_deg": None,
            "measured_range_m": None,
            "measured_bearing_rad": None,
            "decision_margin": None,
            "reprojection_px": None,
        }
        if hit is None:
            return report, record
        pose = fit_pose(self.model, hit["corners"], self.size)
        gated, view = self.gate.check(pose)
        rng, bearing = mount.range_bearing(pose["translation"], extra_pitch_rad)
        record.update(
            gated=bool(gated),
            measured_view_deg=math.degrees(view),
            measured_range_m=rng,
            measured_bearing_rad=bearing,
            decision_margin=hit["decision_margin"],
            reprojection_px=pose["reprojection_rms_px"],
        )
        if not gated:
            report.update(detected=True, range_m=rng, bearing_rad=bearing)
        return report, record
