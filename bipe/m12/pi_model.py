"""M12 phase D: the measurement link model, standing in for the Pi and the phase E link.

At 30 Hz the camera takes a frame. The Pi tracks the tag in a region of interest while
the last frame was a detection, and otherwise searches the full frame (quad_decimate 2 on
4 threads), skipping the frames that arrive while the search runs. Each processed frame
gives one report: a detection or an empty frame. Whether a frame can show the tag is the
phase A geometry (bipe.m12.follower.Sight: field of view less the edge margin, pixels per
cell, occlusion); whether it is detected, and with what bias and noise, is the phase C
measurement model at the true range and view angle, with the yaw-blur range term at the
true bearing rate. The latency of each report is drawn from the phase C estimate. All
draws come from one seeded generator, so a run is deterministic.
"""

import math

import numpy as np

from . import perception as PC
from .scenarios import at


class PiLink:
    def __init__(
        self, link, model, exposure_s, link_delay_s, sight, tracks, seed, latency_shift_s=0.0
    ):
        self.link, self.model, self.exposure = link, model, exposure_s
        self.link_delay = link_delay_s
        self.sight, self.tracks = sight, tracks
        self.rng = np.random.default_rng(seed)
        self.shift = latency_shift_s
        self.frame_dt = 1.0 / link["frame_rate_hz"]
        self.next_frame = 0.0
        self.sequence = 0
        self.busy_until = -math.inf
        self.tracking = False
        self.missed = False
        self.queue = []
        self.previous = None  # (time, true bearing) of the last frame
        self.frames = []

    def _truth(self, t, robot):
        tau = t - self.link["start_s"]
        s = at(self.tracks["target"], tau)
        pos = np.array([s["x"], s["y"]]) + robot["origin"]
        tag_yaw = s["tag_yaw"]
        camera, _ = self.sight.camera(robot["x"], robot["y"], robot["yaw"], robot["acceleration"])
        if math.isnan(tag_yaw):
            tag_yaw = math.atan2(camera[1] - pos[1], camera[0] - pos[0])
        occluder = None
        if "occluder" in self.tracks:
            o = at(self.tracks["occluder"], tau)
            occluder = np.array([o["x"], o["y"]]) + robot["origin"]
        seen = self.sight.look(
            (robot["x"], robot["y"], robot["yaw"]), robot["acceleration"], pos, tag_yaw, occluder
        )
        return seen, pos

    def poll(self, t, robot):
        """Process the frame due at time t, if any, and return the reports that have
        arrived by t as (arrival time, report dict), oldest first. robot: the true plan
        pose of the axle (x, y, yaw), the equivalent acceleration of the body lean, and the
        origin of the target's track in the same frame."""
        if t + 1e-9 >= self.next_frame:
            self.next_frame += self.frame_dt
            self.sequence += 1
            if t >= self.busy_until:
                self._frame(t, robot)
        out = []
        while self.queue and self.queue[0][0] <= t + 1e-12:
            out.append(self.queue.pop(0))
        return out

    def _frame(self, t, robot):
        link, model = self.link, self.model
        seen, _ = self._truth(t, robot)
        rate = 0.0
        if self.previous is not None and t > self.previous[0]:
            rate = _wrap(seen["bearing"] - self.previous[1]) / (t - self.previous[0])
        self.previous = (t, seen["bearing"])
        view_deg = math.degrees(seen["view_angle"])
        possible = seen["in_view"] and not seen["occluded"] and seen["pixels"] >= self.sight.min_px
        m = PC.model_lookup(model, seen["range"], view_deg)
        tracking = self.tracking
        p = m["p_detect_tracking"] if tracking else m["p_detect_acquisition"]
        if self.missed:
            p = min(p, 1.0 - link["p_miss_after_miss"])
        u_detect, u_latency = self.rng.random(), self.rng.random()
        noise = self.rng.standard_normal(2)
        detected = bool(possible and u_detect < p)
        lo, hi = link["tracking_latency_s" if tracking else "acquisition_latency_s"]
        latency = lo + u_latency * (hi - lo) + self.shift
        if not tracking:
            # The full-frame search holds the cores; frames ready before it ends are skipped.
            self.busy_until = t + latency - link["pipeline_s"]
        report = {
            "sequence": self.sequence,
            "tag_id": 0,
            "detected": detected,
            "range_m": 0.0,
            "bearing_rad": 0.0,
            "age_s": latency - self.link_delay,
        }
        if detected:
            rng_err = (
                m["range_bias_m"]
                + PC.motion_bias(model, seen["range"], rate, self.exposure)
                + m["range_std_m"] * noise[0]
            )
            brg_err = math.radians(m["bearing_bias_deg"] + m["bearing_std_deg"] * noise[1])
            report["range_m"] = seen["range"] + rng_err
            report["bearing_rad"] = seen["bearing"] + brg_err
        self.frames.append(
            {
                "t": t,
                "sequence": self.sequence,
                "tracking": tracking,
                "possible": bool(possible),
                "detected": detected,
                "range_m": seen["range"],
                "bearing_rad": seen["bearing"],
                "view_deg": view_deg,
                "latency_s": latency,
            }
        )
        self.tracking = detected
        self.missed = not detected and possible
        self.queue.append((t + latency, report))
        self.queue.sort(key=lambda q: q[0])


def _wrap(a):
    return math.remainder(a, 2 * math.pi)
