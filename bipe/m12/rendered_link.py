"""M12 phase F: the rendered perception link, standing in for the Pi and the phase E link.

A drop-in for bipe.m12.pi_model.PiLink in the closed loop. At 30 Hz the head camera takes
a frame: the mean of the sub-frame renders over the exposure, each from the free-base
plant's own state at that time (recorded by the simulation's read-only observer), with
the target and occluder placed from the scenario tracks; then the phase C lens, noise and
tone model. The phase C tracker (region of interest after a detection, otherwise a
full-frame search at quad_decimate 2) reads the frame, and a detection's range and bearing
come from the tag pose fitted through the lens model, as in phase C. The frame scheduling
is the phase D link model's: frames that arrive while a full-frame search holds the cores
are skipped, and the latency of each report is drawn from the phase C Pi Zero 2 W estimate
for the tracker's mode (signed-off timing, user decision 22 October 2026): while tracking,
only the region of interest is searched and a miss is reported with the tracking latency;
the next frame is a full-frame search, with its latency, which holds the cores. The latency counts from the exposure's centre, as the phase C
budget's. Seeds vary only that draw and the sensor noise; the renders are a function of
the state alone, and each frame is rendered twice to check that.

The tag pose becomes a range and bearing through a camera pose: "plant_camera" (default,
as phase C) uses the plant's own camera pose, lean and pitch included, as if the Pi knew
it; "fixed_mount" uses the camera of the robot standing level at the nominal posture with
the same axle position and heading, as a Pi that is not told the body's lean. The range is
to the tag's face.

Evidence: perception on rendered frames in the loop, under the phase C camera, light and
noise assumptions; the Pi timing is the phase C estimate. Not real footage or hardware.
"""

import json
import math
from bisect import bisect_left

import numpy as np

from ..parameters import ROOT
from ..pi.gate import ViewGate, view_angle
from ..pi.pipeline import FramePipeline
from ..pi.tracker import RegionTracker
from . import perception as PC
from . import scenarios as S

CONFIG = ROOT / "config/following-rendered-m12.json"


def load():
    return json.loads(CONFIG.read_text())


def _seed(*parts):
    return PC._seed(*parts)


def apply_lighting(sc, lighting):
    """Set a lighting variant on a phase C scene (model lights and the noise budget)."""
    m = sc.m
    head = lighting["headlight"]
    m.vis.headlight.ambient[:] = head["ambient"]
    m.vis.headlight.diffuse[:] = head["diffuse"]
    lights = lighting["lights"]
    if len(lights) != m.nlight:
        raise ValueError(f"lighting has {len(lights)} lights; the scene has {m.nlight}")
    for i, light in enumerate(lights):
        direction = np.asarray(light["dir"], float)
        m.light_dir[i] = direction / np.linalg.norm(direction)
        m.light_diffuse[i] = light["diffuse"]
    sc.camera.noise = dict(sc.camera.noise, illuminance_lux=lighting["illuminance_lux"])


def tilt_ground(sc, slope_deg):
    """Tilt the scene's floor as the plant's (bipe.m10.plant.reset)."""
    import mujoco

    m = sc.m
    slope = math.radians(slope_deg)
    ground = m.geom("ground").id
    m.geom_sameframe[ground] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
    m.geom_quat[ground] = [math.cos(slope / 2), 0, -math.sin(slope / 2), 0]


_view_angle = view_angle  # the Pi's gate (bipe.pi.gate)


class _PlantMount:
    """The simulator's mount for one frame: the tag's camera-frame position to range and
    bearing through the plant's own camera pose at the exposure ("plant_camera"), or the
    camera of the robot standing level ("fixed_mount"). The lean is in the pose already."""

    def __init__(self, link, truth):
        self.link, self.truth = link, truth

    def range_bearing(self, t_image, extra_pitch_rad):
        D, truth = self.link.sc.D, self.truth
        cam_pos, cam_mat = truth["cam_pos"], truth["cam_mat"]
        if self.link.conversion == "fixed_mount":
            cam_pos, cam_mat = self.link._standing_camera(truth["yaw"])
        point = D.world_point(cam_pos, cam_mat, t_image)
        return D.range_bearing(cam_pos, truth["yaw"], point)


class RenderedLink:
    def __init__(
        self,
        link,
        exposure_s,
        subframes,
        link_delay_s,
        tracks,
        seed,
        latency_shift_s=0.0,
        layout="waist_tag",
        lighting=None,
        slope_deg=0.0,
        check_repeat=True,
        keep_frames=(),
        scene=None,
        sight=None,
        pose_conversion="plant_camera",
        tracker_policy=None,
        view_gate_deg=None,
    ):
        self.gate = ViewGate(view_gate_deg)
        self.policy = tracker_policy or {"region_retries": 0, "growth_rad_s": 0.0}
        if pose_conversion not in ("plant_camera", "fixed_mount"):
            raise ValueError(f"unknown pose conversion {pose_conversion!r}")
        self.conversion = pose_conversion
        self.link, self.exposure, self.subframes = link, exposure_s, subframes
        self.link_delay = link_delay_s
        self.tracks = tracks
        self.sight = sight
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.shift = latency_shift_s
        self.frame_dt = 1.0 / link["frame_rate_hz"]
        self.next_frame = 0.0
        self.sequence = 0
        self.busy_until = -math.inf
        self.queue = []
        self.frames = []
        self.kept = []
        self.keep = set(keep_frames)
        self.check_repeat = check_repeat
        self.repeat_mismatches = 0
        self.slope = math.radians(slope_deg)
        self.times, self.states = [], []
        self.sc = scene or PC.Scene(layout, "occluder" in tracks)
        if lighting is not None:
            apply_lighting(self.sc, lighting)
        if slope_deg:
            tilt_ground(self.sc, slope_deg)
        # The Pi's frame pipeline (bipe.pi) on the scene's detectors; only the detectors of
        # the scene's phase C Tracker are used, not its state.
        tr = self.sc.tracker
        self.tracker = RegionTracker(
            tr.full, tr.local, tr.roi_cfg, self.policy, self.sc.model["f_px"]
        )
        self.pipeline = FramePipeline(self.tracker, self.sc.model, self.sc.size, self.gate, None)

    def close(self):
        self.sc.close()

    # The plant's state ----------------------------------------------------------------

    def record(self, t, dt, drives, d):
        """The simulation's observer: keep the plant state of every physics step (only
        what the exposures can still reach)."""
        self.times.append(float(t))
        self.states.append(d.qpos.copy())
        horizon = t - self.exposure - 0.05
        if len(self.times) > 4096 and self.times[2048] < horizon:
            del self.times[:2048], self.states[:2048]

    def _state(self, t):
        i = bisect_left(self.times, t)
        if i == len(self.times) or (i > 0 and t - self.times[i - 1] <= self.times[i] - t):
            i -= 1
        return self.states[max(i, 0)]

    def _place(self, t, origin):
        tau = t - self.link["start_s"]
        s = S.at(self.tracks["target"], tau)
        tag_yaw = None if math.isnan(s["tag_yaw"]) else s["tag_yaw"]
        occ = None
        if "occluder" in self.tracks:
            o = S.at(self.tracks["occluder"], tau)
            occ = (o["x"] + origin[0], o["y"] + origin[1], o["heading"])
        sc = self.sc
        sc.place(self._state(t), (s["x"] + origin[0], s["y"] + origin[1]), tag_yaw, occ)
        if self.slope:
            # The people stand on the tilted floor (z = x tan(slope) in the world frame).
            for name in ("target", "occluder") if occ is not None else ("target",):
                i = sc.m.body(name).mocapid[0]
                sc.d.mocap_pos[i][2] = sc.d.mocap_pos[i][0] * math.tan(self.slope)
            sc.mujoco.mj_forward(sc.m, sc.d)

    def _expose(self, t_end, origin, rng):
        sc = self.sc
        return sc.camera.expose(
            lambda t: self._place(t, origin), sc.d, t_end, self.exposure, self.subframes, rng
        )

    # The link ---------------------------------------------------------------------------

    def poll(self, t, robot):
        """As PiLink.poll: process the frame due at time t, if any, and return the reports
        that have arrived by t as (arrival time, report dict), oldest first. Only
        robot["origin"] is used for the render; the camera pose comes from the plant's
        state. With a sight, the phase A geometry's view of the same frame is recorded
        alongside ("possible", as the phase D link model), for the comparison."""
        if t + 1e-9 >= self.next_frame:
            self.next_frame += self.frame_dt
            self.sequence += 1
            if t >= self.busy_until and self.times:
                self._frame(t, robot)
        out = []
        while self.queue and self.queue[0][0] <= t + 1e-12:
            out.append(self.queue.pop(0))
        return out

    def _possible(self, t, robot):
        tau = t - self.link["start_s"]
        s = S.at(self.tracks["target"], tau)
        pos = np.array([s["x"], s["y"]]) + robot["origin"]
        camera, _ = self.sight.camera(robot["x"], robot["y"], robot["yaw"], robot["acceleration"])
        tag_yaw = s["tag_yaw"]
        if math.isnan(tag_yaw):
            tag_yaw = math.atan2(camera[1] - pos[1], camera[0] - pos[0])
        occluder = None
        if "occluder" in self.tracks:
            o = S.at(self.tracks["occluder"], tau)
            occluder = np.array([o["x"], o["y"]]) + robot["origin"]
        seen = self.sight.look(
            (robot["x"], robot["y"], robot["yaw"]), robot["acceleration"], pos, tag_yaw, occluder
        )
        return bool(
            seen["in_view"] and not seen["occluded"] and seen["pixels"] >= self.sight.min_px
        )

    def _detect(self, image, t):
        """The phase C pipeline, as the Pi runs it (bipe.pi.tracker.RegionTracker): while
        tracking, the region of interest alone, and a miss there is reported at once; after
        more than the policy's region_retries misses in a row the next frame is a full-frame
        search. (bipe.m12.detector.Tracker falls back to the full frame within the same
        frame, which the Pi's timing budget does not allow.)"""
        return self.tracker.detect(image, t)

    def _window(self, shape, age_s):
        return self.tracker.window(shape, age_s)

    def _standing_camera(self, yaw):
        """The head camera's pose with the robot standing level at the nominal posture,
        at the current axle position and heading (the scene is at the frame's state)."""
        sc = self.sc
        axle = np.mean([sc.d.site(s + "_axle").xpos for s in ("left", "right")], axis=0)
        S.place_robot(sc.m, sc.d, sc.p, float(axle[0]), float(axle[1]), yaw, 40, 0.0)
        sc.mujoco.mj_forward(sc.m, sc.d)
        return sc.d.cam_xpos[sc.cid].copy(), sc.d.cam_xmat[sc.cid].reshape(3, 3).copy()

    def _frame(self, t, robot):
        sc, link = self.sc, self.link
        origin = robot["origin"]
        noise_seed = _seed("frame", self.seed, self.sequence)
        image = self._expose(t, origin, np.random.default_rng(noise_seed))
        if self.check_repeat:
            again = self._expose(t, origin, np.random.default_rng(noise_seed))
            self.repeat_mismatches += int(not np.array_equal(image, again))
        mid = t - self.exposure / 2
        self._place(mid, origin)
        truth = sc.truth()
        tracking = self.tracker.tracking
        # The Pi's pipeline: tracker, pose, gate and report. The plant camera's pose already
        # holds the lean, so the lean passed is 0.
        report, pi = self.pipeline.process(
            image, t, self.sequence, 0.0, mount=_PlantMount(self, truth)
        )
        lo, hi = link["tracking_latency_s" if tracking else "acquisition_latency_s"]
        latency = lo + self.rng.random() * (hi - lo) + self.shift
        if not tracking:
            # The full-frame search holds the cores; frames ready before it ends are skipped.
            self.busy_until = mid + latency - link["pipeline_s"]
        report["age_s"] = latency - self.link_delay
        possible = None
        if self.sight is not None:
            possible = self._possible(t, robot)
        self.frames.append(
            {
                "t": t,
                "sequence": self.sequence,
                "tracking": tracking,
                "possible": possible,
                "roi_used": pi["roi_used"],
                "detected": pi["detected"],
                "in_frame": bool(truth["in_frame"]),
                "pixels": float(truth["pixels"]),
                "range_m": float(truth["range_m"]),
                "bearing_rad": math.radians(truth["bearing_deg"]),
                "view_deg": float(truth["view_deg"]),
                "measured_range_m": pi["measured_range_m"],
                "measured_bearing_rad": pi["measured_bearing_rad"],
                "decision_margin": pi["decision_margin"],
                "reprojection_px": pi["reprojection_px"],
                "detector_s": pi["detector_s"],
                "latency_s": latency,
                "gated": pi["gated"],
                "measured_view_deg": pi["measured_view_deg"],
            }
        )
        if self.sequence in self.keep:
            self.kept.append((t, image))
        self.queue.append((mid + latency, report))
        self.queue.sort(key=lambda q: q[0])
