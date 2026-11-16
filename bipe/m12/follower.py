"""M12 phase A ideal follower and the geometric tag visibility it runs on.

The follower is a planar unicycle with no balance dynamics: it commands speed from the
range error and yaw rate from the bearing error, each with the target's own motion fed
forward, and applies them through first-order lags. Run without limits and with perfect
knowledge, it gives the speeds, accelerations, yaw rates and yaw accelerations following
needs (the phase B targets). Run with those limits, the perception latency and the
visibility rules, it checks the engage, loss and reacquisition rules on the scenarios.

Visibility is geometric: the whole tag inside the field of view less the edge margin
(vertical: tag edge elevations against the lens axis, which leans with the quasi-static
body pitch atan(a/g)), the view angle off the tag normal, the pixels per tag cell and
occlusion of the camera-to-tag triangle by an occluding person's plan circle. The
rendered frames check the same geometry against the renderer.
"""

import math

import numpy as np

from . import geometry
from .scenarios import DT, at

TORSO_PLAN_RADIUS_M = 0.18
MODES = ["waiting", "following", "coasting", "braking", "holding"]
COLUMNS = [
    "t",
    "x",
    "y",
    "yaw",
    "speed",
    "yaw_rate",
    "acceleration",
    "yaw_acceleration",
    "mode",
    "visible",
    "occluded",
    "range",
    "bearing",
    "view_angle",
    "pixels",
    "detected",
]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _segment_distance(p, a, b):
    ab, ap = b - a, p - a
    s = float(np.clip(ap @ ab / max(ab @ ab, 1e-12), 0, 1))
    return float(np.linalg.norm(ap - s * ab))


def occluded(camera, tag, tag_yaw, half_width, occluder):
    if occluder is None:
        return False
    across = np.array([-math.sin(tag_yaw), math.cos(tag_yaw)]) * half_width
    edges = [tag + across, tag - across]
    if any(_segment_distance(occluder, camera, e) < TORSO_PLAN_RADIUS_M for e in edges + [tag]):
        return True
    # Inside the camera-to-tag triangle.
    a, b, c = camera, *edges
    signs = [
        (v - u)[0] * (occluder - u)[1] - (v - u)[1] * (occluder - u)[0]
        for u, v in ((a, b), (b, c), (c, a))
    ]
    return all(s > 0 for s in signs) or all(s < 0 for s in signs)


class Sight:
    """Tag visibility from the head camera under the stated geometry."""

    def __init__(self, p, mount, lens, following, layout):
        self.p, self.mount, self.lens = p, dict(mount), lens
        self.mount["pitch_deg"] = layout["camera_pitch_deg"]
        self.tag_height = layout["centre_height_m"]
        cam, tag = following["camera"], following["tag"]
        self.margin = cam["edge_margin_fraction"]
        self.width = cam["image_width_px"]
        self.size = tag["size_m"]
        self.max_view = math.radians(tag["max_view_angle_deg"])
        self.min_px = tag["minimum_pixels_per_cell"] * tag["cells_across_black_square"]
        self.g = p["sizing"]["gravity"]
        self._poses = {}

    def pose(self, acceleration):
        """Camera pose at the nominal posture with the lean of a steady acceleration."""
        key = round(acceleration, 2)
        if key not in self._poses:
            self._poses[key] = geometry.camera_pose(self.p, self.mount, 40, math.atan2(key, self.g))
        return self._poses[key]

    def camera(self, x, y, yaw, acceleration):
        pose = self.pose(acceleration)
        ahead = pose["ahead_of_axle_m"]
        return np.array([x + ahead * math.cos(yaw), y + ahead * math.sin(yaw)]), pose

    def look(self, robot, acceleration, tag, tag_yaw, occluder=None):
        """Range, bearing, view angle, pixels and whether a detection is possible."""
        x, y, yaw = robot
        camera, pose = self.camera(x, y, yaw, acceleration)
        rel = tag - camera
        r = float(np.linalg.norm(rel))
        bearing = wrap(math.atan2(rel[1], rel[0]) - yaw)
        normal = np.array([math.cos(tag_yaw), math.sin(tag_yaw)])
        view = math.acos(float(np.clip(-rel @ normal / max(r, 1e-9), -1, 1)))
        hfov, vfov = self.lens["horizontal_fov_deg"], self.lens["vertical_fov_deg"]
        in_h = abs(bearing) <= geometry.bearing_allowance_rad(hfov, self.margin, self.size, r)
        half_v = math.radians(vfov / 2 - geometry.edge_margin_deg(vfov, self.margin))
        axis = math.radians(pose["elevation_deg"])
        top = math.atan2(self.tag_height + self.size / 2 - pose["height_m"], r)
        bottom = math.atan2(self.tag_height - self.size / 2 - pose["height_m"], r)
        in_v = bottom >= axis - half_v and top <= axis + half_v
        px = geometry.pixels_on_tag(hfov, self.width, self.size * math.cos(view), r)
        blocked = occluded(camera, tag, tag_yaw, self.size / 2, occluder)
        return {
            "range": r,
            "bearing": bearing,
            "view_angle": view,
            "pixels": px,
            "in_view": in_h and in_v,
            "occluded": blocked,
            "visible": in_h
            and in_v
            and view <= self.max_view
            and px >= self.min_px
            and not blocked,
        }


def run(scenario_tracks, sight, following, limits=None, latency_s=0.0, perfect=False):
    """Simulate the follower on one scenario.

    limits: None (no limits) or a dict of speed, reverse, acceleration, yaw_rate and
    yaw_acceleration. perfect: the target is always known (no visibility, engage or
    loss logic); otherwise detections arrive at the detection rate when visible, each
    reporting the target as it was latency_s earlier, predicted forward over the latency.
    """
    target = scenario_tracks["target"]
    occ = scenario_tracks.get("occluder")
    f, cam, req = following["ideal_follower"], following["camera"], following
    nominal = req["range"]["nominal_m"]
    kr, kb = f["range_gain_1_s"], f["bearing_gain_1_s"]
    tv, tw = f["speed_lag_s"], f["yaw_lag_s"]
    reverse = req["close_range"]["reverse_speed_max_m_s"]
    stop_range = req["close_range"]["stop_range_m"]
    # The ideal follower keeps the phase A coast (its traces are the phase B and C inputs).
    coast = f.get("coast_s", req["loss"]["coast_s"])
    brake = req["loss"]["brake_deceleration_m_s2"]
    engage = req["engage"]
    gate = req["reacquisition"]["gate_m"]
    frame_dt = 1 / cam["detection_rate_hz_assumption"]
    steps = len(target["t"])
    x = y = yaw = v = w = a = alpha = 0.0
    mode = "following" if perfect else "waiting"
    estimate = None  # (time of validity, position, velocity)
    last_seen, streak, streak_start = -math.inf, 0, None
    next_frame = 0.0
    lost_at = None
    rows, events = [], []

    def truth(t):
        s = at(target, t)
        pos = np.array([s["x"], s["y"]])
        vel = s["speed"] * np.array([math.cos(s["heading"]), math.sin(s["heading"])])
        return pos, vel, s["tag_yaw"]

    for i in range(steps):
        t = i * DT
        pos, vel, tag_yaw = truth(t)
        if math.isnan(tag_yaw):
            camera, _ = sight.camera(x, y, yaw, a)
            tag_yaw = math.atan2(*(camera - pos)[::-1])
        occluder = None
        if occ is not None:
            o = at(occ, t)
            occluder = np.array([o["x"], o["y"]])
        seen = sight.look((x, y, yaw), a, pos, tag_yaw, occluder)
        detected = False
        if perfect:
            estimate = (t, pos, vel)
        elif t + 1e-9 >= next_frame:
            next_frame += frame_dt
            past = max(0.0, t - latency_s)
            p_pos, p_vel, _ = truth(past)
            # Visibility at the exposure, judged from the present geometry (the lag is
            # under a frame of robot motion at the detection rate).
            if seen["visible"]:
                detected = True
                measured = (past, p_pos, p_vel)
                predicted = None
                if estimate is not None:
                    predicted = estimate[1] + estimate[2] * (past - estimate[0])
                consistent = predicted is not None and np.linalg.norm(p_pos - predicted) <= gate
                if mode in ("coasting", "braking") and consistent:
                    events.append({"t": t, "event": "reacquired", "from": mode})
                    mode = "following"
                streak = streak + 1 if streak else 1
                streak_start = t if streak == 1 else streak_start
                estimate = measured
                last_seen = t
            else:
                streak, streak_start = 0, None
        if not perfect:
            if mode in ("waiting", "holding") and estimate is not None and detected:
                r_now = seen["range"]
                if (
                    streak >= engage["consecutive_detections"]
                    and t - streak_start >= engage["minimum_s"] - 1e-9
                    and engage["range_m"][0] <= r_now <= engage["range_m"][1]
                    and abs(v) < 0.05
                ):
                    events.append({"t": t, "event": "engaged", "from": mode})
                    mode = "following"
            # A missed frame starts the coast on the prediction; the coast ends in braking.
            if mode == "following" and t - last_seen > 1.5 * frame_dt:
                mode = "coasting"
            if mode == "coasting" and t - last_seen > coast + 1e-9:
                events.append({"t": t, "event": "lost", "speed": v, "x": x, "y": y})
                lost_at = (x, y)
                mode = "braking"
            if mode == "braking" and abs(v) < 1e-3 and abs(w) < 1e-3:
                events.append(
                    {
                        "t": t,
                        "event": "holding",
                        "stopping_distance_m": float(np.hypot(x - lost_at[0], y - lost_at[1])),
                    }
                )
                mode = "holding"
        # Commands.
        if mode in ("following", "coasting") and estimate is not None:
            e_pos = estimate[1] + estimate[2] * (t - estimate[0])
            e_vel = estimate[2]
            camera, _ = sight.camera(x, y, yaw, a)
            rel = e_pos - camera
            r = float(np.linalg.norm(rel))
            u = rel / max(r, 1e-9)
            n = np.array([-u[1], u[0]])
            bearing = wrap(math.atan2(rel[1], rel[0]) - yaw)
            v_cmd = float(e_vel @ u) + kr * (r - nominal)
            v_cmd = max(v_cmd, -reverse)
            if r < stop_range:
                v_cmd = 0.0
            w_cmd = kb * bearing + float(e_vel @ n) / max(r, 0.2)
        else:
            v_cmd, w_cmd = 0.0, 0.0
        if limits is not None:
            v_cmd = float(np.clip(v_cmd, -reverse, limits["speed"]))
            w_cmd = float(np.clip(w_cmd, -limits["yaw_rate"], limits["yaw_rate"]))
        a_new = (v_cmd - v) / tv
        alpha_new = (w_cmd - w) / tw
        if mode == "braking":
            a_new = float(np.clip(-v / DT, -brake, brake))
        if limits is not None:
            a_new = float(np.clip(a_new, -limits["acceleration"], limits["acceleration"]))
            alpha_new = float(
                np.clip(alpha_new, -limits["yaw_acceleration"], limits["yaw_acceleration"])
            )
        a, alpha = a_new, alpha_new
        rows.append(
            [
                t,
                x,
                y,
                yaw,
                v,
                w,
                a,
                alpha,
                MODES.index(mode),
                float(seen["visible"]),
                float(seen["occluded"]),
                seen["range"],
                seen["bearing"],
                seen["view_angle"],
                seen["pixels"],
                float(detected),
            ]
        )
        v += a * DT
        w += alpha * DT
        yaw = wrap(yaw + w * DT)
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
    return {"columns": COLUMNS, "trace": np.array(rows), "events": events}


def summary(result, following, straight=False):
    """Peaks and requirement checks for one follower run."""
    f = dict(zip(result["columns"], result["trace"].T, strict=True))
    following_mask = f["mode"] == MODES.index("following")
    band = following["range"]["band_m"]
    moving = np.flatnonzero(np.abs(f["speed"]) > 0.02)
    after = f["t"] >= (f["t"][moving[0]] if len(moving) else 0)
    tracked = following_mask & after
    out = {
        "peak_speed_m_s": float(np.max(f["speed"])),
        "peak_reverse_m_s": float(max(0.0, -np.min(f["speed"]))),
        "peak_acceleration_m_s2": float(np.max(np.abs(f["acceleration"]))),
        "peak_yaw_rate_rad_s": float(np.max(np.abs(f["yaw_rate"]))),
        "peak_yaw_acceleration_rad_s2": float(np.max(np.abs(f["yaw_acceleration"]))),
        "range_min_m": float(np.min(f["range"][tracked])) if tracked.any() else None,
        "range_max_m": float(np.max(f["range"][tracked])) if tracked.any() else None,
        "peak_bearing_deg": float(np.degrees(np.max(np.abs(f["bearing"][tracked]))))
        if tracked.any()
        else None,
        "peak_view_angle_deg": float(np.degrees(np.max(f["view_angle"][tracked])))
        if tracked.any()
        else None,
        "min_pixels": float(np.min(f["pixels"][tracked])) if tracked.any() else None,
        "invisible_while_following_s": float(np.sum(tracked & (f["visible"] < 0.5)) * DT),
        "events": result["events"],
    }
    out["range_in_band"] = bool(
        tracked.any() and band[0] <= out["range_min_m"] and out["range_max_m"] <= band[1]
    )
    if straight:
        out["steady_bearing_ok"] = bool(
            out["peak_bearing_deg"] is not None
            and out["peak_bearing_deg"] <= following["bearing"]["steady_tolerance_deg"]
        )
    return out
