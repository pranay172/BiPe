"""M12 phase C: AprilTag perception measured on rendered head-camera frames.

.venv/bin/python -m bipe.m12.perception [--workers N] [--skip grid,blur,replays]

Writes artifacts/m12/perception/:
- config.json: the inputs (config/perception-m12.json, the camera intrinsics and render
  geometry, the following requirements' digest, the detector library);
- grid/: the static accuracy grid (range, bearing, view angle, body pitch, both tag
  layouts): samples.csv and summary.json;
- blur/: the motion-blur set (yaw and pitch rates, exposures): samples.csv, summary.json;
- replays/: the phase A scenarios with the phase B balance controller in the loop and
  motion blur, rendered at 30 Hz: <layout>/<scenario>.csv per frame, summary.json, and a
  few example frames;
- timing.json: detector times on this machine and the Pi Zero 2 W estimate with its
  basis, and the latency budget;
- model_check.json: the measurement model against the held-out rendered runs;
- summary.json and plots.
It also fills measurement_model in config/perception-m12.json.

Evidence kind: perception measured on rendered frames under the stated camera, light and
noise assumptions (config/perception-m12.json). Nothing here is measured on real footage
or on a Raspberry Pi; the Pi timing is an estimate scaled from this machine.
"""

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import statistics
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

from ..parameters import ROOT
from . import follower as F
from . import geometry as G
from . import requirements as R
from . import scenarios as S

OUTPUT = ROOT / "artifacts/m12/perception"
CONFIG = ROOT / "config/perception-m12.json"
METHODS = ["dec1", "dec1.5", "dec2"]
DECIMATION = {"dec1": 1.0, "dec1.5": 1.5, "dec2": 2.0}
PROPOSED = {"tracking": "roi", "acquisition": "dec2"}
EXPECT_VIEW_DEG = 60.0


def load():
    shared, following, p = R.load()
    return shared, following, p, json.loads(CONFIG.read_text())


def _sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def dumps(obj, indent=2):
    """JSON with lists of scalars kept on one line (the style of the hand-written configs)."""

    def scalar_list(x):
        return isinstance(x, list) and all(not isinstance(v, (list, dict)) for v in x)

    def walk(x, level):
        pad = " " * (indent * level)
        if isinstance(x, dict):
            if not x:
                return "{}"
            items = [
                f"{pad}{' ' * indent}{json.dumps(k)}: {walk(v, level + 1)}" for k, v in x.items()
            ]
            return "{\n" + ",\n".join(items) + f"\n{pad}}}"
        if isinstance(x, list) and not scalar_list(x):
            items = [f"{pad}{' ' * indent}{walk(v, level + 1)}" for v in x]
            return "[\n" + ",\n".join(items) + f"\n{pad}]"
        return json.dumps(x, allow_nan=False)

    return walk(obj, 0) + "\n"


# Scene: one world per tag layout (and occluder) in a worker process ----------------------


class Scene:
    def __init__(self, layout_name, occluder=False):
        import mujoco

        from . import camera as C
        from . import detector as D

        self.mujoco, self.C, self.D = mujoco, C, D
        shared, following, p, perc = load()
        self.shared, self.following, self.p, self.perc = shared, following, p, perc
        self.layout = following["tag"]["layouts"][layout_name]
        self.layout_name = layout_name
        cam = perc["camera"]
        self.model = C.intrinsics(shared, perc)
        self.geometry = C.render_geometry(self.model, cam["render"]["supersample"])
        self.lens = shared["m12"]["camera"]["lenses"][cam["lens"]]
        self.mount = G.camera_mount(shared)
        tag = D.tag_image(perc["tag"]["family"], perc["tag"]["id"])
        xml = C.world(
            p, self.mount, self.lens, following, self.layout, self.geometry, tag, occluder
        )
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        self.camera = C.Camera(
            self.m,
            self.model,
            self.geometry,
            perc,
            self.lens["f_number"],
            shared["m12"]["camera_sensor"]["pixel_size_m"],
        )
        self.cid = self.m.camera("head").id
        self.tag_geom = self.m.geom("target_tag").id
        self.tag_front = float(self.m.geom_size[self.tag_geom][0]) + C.TAG_INK_M
        self.size = following["tag"]["size_m"]
        det = perc["detector"]
        self.detectors = {k: D.Detector(det, perc["tag"]["id"], DECIMATION[k]) for k in METHODS}
        self.tracker = D.Tracker(det, perc["tag"]["id"], DECIMATION[PROPOSED["acquisition"]], True)
        self.chassis = self.m.body("chassis").id

    def close(self):
        self.camera.close()

    # Truth -------------------------------------------------------------------------

    def truth(self):
        d, D = self.d, self.D
        xmat = d.geom_xmat[self.tag_geom].reshape(3, 3)
        normal = xmat[:, 0]
        centre = d.geom_xpos[self.tag_geom] + normal * self.tag_front
        cam_pos = d.cam_xpos[self.cid].copy()
        cam_mat = d.cam_xmat[self.cid].reshape(3, 3).copy()
        body = d.xmat[self.chassis].reshape(3, 3)
        yaw = math.atan2(body[1, 0], body[0, 0])
        rng, bearing = D.range_bearing(cam_pos, yaw, centre)
        to_cam = cam_pos - centre
        plan = to_cam[:2] / max(np.linalg.norm(to_cam[:2]), 1e-12)
        n_plan = normal[:2] / max(np.linalg.norm(normal[:2]), 1e-12)
        hview = math.degrees(math.acos(float(np.clip(plan @ n_plan, -1, 1))))
        view3 = math.degrees(
            math.acos(float(np.clip(to_cam @ normal / np.linalg.norm(to_cam), -1, 1)))
        )
        # The black square's corners through the true pose and the lens model.
        h = self.size / 2
        across = np.cross([0.0, 0.0, 1.0], normal)
        across /= np.linalg.norm(across)
        up = np.cross(normal, across)
        corners = [centre + a * across + b * up for a, b in ((h, h), (-h, h), (-h, -h), (h, -h))]
        uv = []
        for c in corners:
            v = D.MUJOCO_TO_IMAGE @ (cam_mat.T @ (c - cam_pos))
            if v[2] <= 1e-6:
                uv.append((math.nan, math.nan))
                continue
            u_, v_ = self.C.project(self.model, v[0] / v[2], v[1] / v[2])
            uv.append((float(u_), float(v_)))
        uv = np.array(uv)
        w, hh = self.model["width_px"], self.model["height_px"]
        finite = bool(np.all(np.isfinite(uv)))
        in_frame = finite and bool(
            np.all((uv[:, 0] >= 0) & (uv[:, 0] <= w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= hh - 1))
        )
        if finite:
            x, y = uv[:, 0], uv[:, 1]
            area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
            pixels = math.sqrt(area)
        else:
            pixels = 0.0
        return {
            "range_m": rng,
            "bearing_deg": math.degrees(bearing),
            "view_deg": hview,
            "view3d_deg": view3,
            "in_frame": in_frame,
            "pixels": pixels,
            "cam_pos": cam_pos,
            "cam_mat": cam_mat,
            "yaw": yaw,
            "centre": centre,
        }

    def measure(self, hit, truth):
        """Range and bearing from a detection, against the truth."""
        if hit is None:
            return None
        pose = self.D.fit_pose(self.model, hit["corners"], self.size)
        point = self.D.world_point(truth["cam_pos"], truth["cam_mat"], pose["translation"])
        rng, bearing = self.D.range_bearing(truth["cam_pos"], truth["yaw"], point)
        db = (math.degrees(bearing) - truth["bearing_deg"] + 180) % 360 - 180
        return {
            "range_error_m": rng - truth["range_m"],
            "bearing_error_deg": db,
            "decision_margin": hit["decision_margin"],
            "reprojection_px": pose["reprojection_rms_px"],
        }

    def detect_all(self, image, truth, roi=False):
        out = {}
        for k, det in self.detectors.items():
            hit, _, _ = det.detect(image)
            out[k] = self.measure(hit, truth)
        if roi:
            hit, spent, used = self.tracker.detect(image)
            out["roi"] = self.measure(hit, truth)
            out["roi_used"] = used
            out["roi_time_s"] = spent
        return out

    # Placement ---------------------------------------------------------------------

    def place(self, robot, tag_xy, tag_yaw=None, occluder=None):
        """robot: (x, y, yaw, pitch offset deg, posture deg), or a qpos vector."""
        mj, d = self.mujoco, self.d
        if isinstance(robot, np.ndarray):
            d.qpos[: len(robot)] = robot
            mj.mj_forward(self.m, d)
        else:
            x, y, yaw, pitch, posture = robot
            S.place_robot(self.m, d, self.p, x, y, yaw, posture, pitch)
        if tag_yaw is None:
            cam = d.cam_xpos[self.cid]
            tag_yaw = math.atan2(cam[1] - tag_xy[1], cam[0] - tag_xy[0])
        S.place_person(self.m, d, "target", tag_xy[0], tag_xy[1], tag_yaw)
        if occluder is not None:
            S.place_person(self.m, d, "occluder", *occluder)
        mj.mj_forward(self.m, d)
        return tag_yaw


_SCENES = {}


def scene(layout, occluder=False):
    key = (layout, occluder)
    if key not in _SCENES:
        _SCENES[key] = Scene(layout, occluder)
    return _SCENES[key]


def flatten(truth, results):
    row = {k: v for k, v in truth.items() if not isinstance(v, np.ndarray) and k != "yaw"}
    for method, r in results.items():
        if method in ("roi_used", "roi_time_s"):
            row[method] = r
            continue
        row[f"{method}_detected"] = r is not None
        for key in ("range_error_m", "bearing_error_deg", "decision_margin", "reprojection_px"):
            row[f"{method}_{key}"] = None if r is None else r[key]
    return row


# Static grid --------------------------------------------------------------------------


def grid_points(perc, following, lens):
    g = perc["grid"]
    cam = following["camera"]
    points = []
    for layout in g["layouts"]:
        for r in g["ranges_m"]:
            allowance = math.degrees(
                G.bearing_allowance_rad(
                    lens["horizontal_fov_deg"],
                    cam["edge_margin_fraction"],
                    following["tag"]["size_m"],
                    r,
                )
            )
            for frac in g["bearing_fractions"]:
                for view in g["view_angles_deg"]:
                    for pitch in g["body_pitch_offsets_deg"]:
                        for k in range(g["samples"] + g["held_out_samples"]):
                            points.append(
                                {
                                    "layout": layout,
                                    "range_set_m": r,
                                    "bearing_fraction": frac,
                                    "bearing_set_deg": frac * allowance,
                                    "view_set_deg": view,
                                    "pitch_offset_deg": pitch,
                                    "sample": k,
                                    "held_out": k >= g["samples"],
                                }
                            )
    return points


def _seed(*parts):
    return int.from_bytes(hashlib.sha256(repr(parts).encode()).digest()[:8], "little")


def grid_task(points):
    rows = []
    for pt in points:
        sc = scene(pt["layout"])
        perc = sc.perc
        rng = np.random.default_rng(
            _seed("grid", pt["layout"], pt["range_set_m"], pt["bearing_fraction"])
            + _seed(pt["view_set_deg"], pt["pitch_offset_deg"], pt["sample"])
        )
        posture = perc["grid"]["posture_deg"]
        sc.place((0.0, 0.0, 0.0, pt["pitch_offset_deg"], posture), (10.0, 10.0), 0.0)
        cam = sc.d.cam_xpos[sc.cid].copy()
        b = math.radians(pt["bearing_set_deg"])
        jitter = rng.uniform(-1, 1, 2) * perc["grid"]["jitter_m"]
        tag = cam[:2] + pt["range_set_m"] * np.array([math.cos(b), math.sin(b)]) + jitter
        facing = math.atan2(cam[1] - tag[1], cam[0] - tag[0])
        sc.place(
            (0.0, 0.0, 0.0, pt["pitch_offset_deg"], posture),
            tag,
            facing + math.radians(pt["view_set_deg"]),
        )
        truth = sc.truth()
        image = sc.camera.develop(
            sc.camera.linear(sc.d), perc["camera"]["exposure"]["exposure_s"], rng
        )
        rows.append({**pt, **flatten(truth, sc.detect_all(image, truth))})
    return rows


# Motion blur set ----------------------------------------------------------------------


def blur_points(perc):
    b = perc["blur_set"]
    out = []
    for r in b["ranges_m"]:
        for w in b["yaw_rates_rad_s"]:
            for q in b["pitch_rates_rad_s"]:
                for exposure in perc["camera"]["exposure"]["comparison_exposures_s"]:
                    for k in range(b["samples"]):
                        out.append(
                            {
                                "layout": b["layout"],
                                "range_set_m": r,
                                "yaw_rate_rad_s": w,
                                "pitch_rate_rad_s": q,
                                "exposure_s": exposure,
                                "sample": k,
                            }
                        )
    return out


def blur_task(points):
    rows = []
    for pt in points:
        sc = scene(pt["layout"])
        perc = sc.perc
        rng = np.random.default_rng(
            _seed("blur", pt["range_set_m"], pt["yaw_rate_rad_s"], pt["pitch_rate_rad_s"])
            + _seed(pt["exposure_s"], pt["sample"])
        )
        posture = perc["grid"]["posture_deg"]
        sc.place((0.0, 0.0, 0.0, 0.0, posture), (10.0, 10.0), 0.0)
        cam = sc.d.cam_xpos[sc.cid].copy()
        jitter = rng.uniform(-1, 1, 2) * perc["grid"]["jitter_m"]
        tag = cam[:2] + np.array([pt["range_set_m"], 0.0]) + jitter
        facing = math.atan2(cam[1] - tag[1], cam[0] - tag[0])
        exposure = pt["exposure_s"]
        n = max(3, round(exposure / 0.002))

        def state(t, pt=pt, tag=tag, facing=facing, posture=posture, sc=sc, exposure=exposure):
            # t runs over the exposure ending at 0; the truth is taken at mid-exposure.
            s = t + exposure / 2
            yaw = pt["yaw_rate_rad_s"] * s
            pitch = math.degrees(pt["pitch_rate_rad_s"] * s)
            sc.place((0.0, 0.0, yaw, pitch, posture), tag, facing)

        state(-exposure / 2)
        truth = sc.truth()
        image = sc.camera.expose(state, sc.d, 0.0, exposure, n, rng)
        rows.append({**pt, **flatten(truth, sc.detect_all(image, truth))})
    return rows


# Scenario replays ---------------------------------------------------------------------


def _interp_track(track, t):
    """Target position and heading at time t, linearly interpolated (held at the ends)."""
    tt = track["t"]
    x = float(np.interp(t, tt, track["x"]))
    y = float(np.interp(t, tt, track["y"]))
    i = int(np.clip(np.searchsorted(tt, t), 0, len(tt) - 1))
    tag_yaw = float(track["tag_yaw"][i]) if "tag_yaw" in track else math.nan
    return x, y, float(track["heading"][i]), tag_yaw


def _distance_to(t):
    return lambda s: abs(s - t)


def replay_task(payload):
    """Run one scenario through the C core and sensor model, recording the plant state
    at every sub-frame time, then render and detect each 30 Hz frame for each layout."""
    from ..moving_experiment import COLUMNS
    from . import speed as SP

    name, library, layouts, sample_frames = payload
    shared, following, p, perc = load()
    _, gains, sensors, _ = SP.load()
    rp = perc["replays"]
    cam = perc["camera"]
    mount = G.camera_mount(shared)
    nominal = following["range"]["nominal_m"]
    ahead = G.camera_pose(p, mount, 40)["ahead_of_axle_m"]
    defs = S.definitions(following, nominal + ahead)
    tracks = S.tracks(defs[name], nominal + ahead, nominal)
    f = SP.follower_trace(rp["layout"], name)
    duration = round(rp["start_s"] + float(f["t"][-1]) + rp["settle_s"], 2)
    case = {
        "name": f"perception_{name}",
        "replay": {"layout": rp["layout"], "scenario": name, "start_s": rp["start_s"]},
        "duration": duration,
    }
    rate, exposure, n_sub = (
        cam["output"]["frame_rate_hz"],
        cam["exposure"]["exposure_s"],
        cam["exposure"]["subframes"],
    )
    frame_ends = np.arange(1, math.floor(duration * rate) + 1) / rate
    wanted = sorted(
        {
            round(te - exposure * (1 - (k + 0.5) / n_sub), 6)
            for te in frame_ends
            for k in range(n_sub)
        }
        | {0.0}
    )
    states, cursor = {}, [0]

    def observer(t, dt, drives, d):
        while cursor[0] < len(wanted) and wanted[cursor[0]] <= t + dt / 2:
            states[wanted[cursor[0]]] = d.qpos.copy()
            cursor[0] += 1

    run = SP.run_case(p, gains, sensors, library, case, observer=observer)
    trace = {c: run["trace"][:, i] for i, c in enumerate(COLUMNS)}
    out = {"scenario": name, "duration_s": duration, "failure": run["failure"], "layouts": {}}
    # Phase B consistency: the same replay as the speed gate's record (where it exists).
    record = ROOT / f"artifacts/m12/speed/traces/replay_{name}.npz"
    if record.exists():
        with np.load(record) as z:
            cols = [str(c) for c in z["columns"]]
            ref = z["trace"]
        ref = ref[ref[:, 0] <= trace["time"][-1]]
        tx = np.interp(ref[:, 0], trace["time"], trace["x"])
        ty = np.interp(ref[:, 0], trace["time"], trace["y"])
        out["phase_b_path_difference_m"] = float(
            np.max(np.hypot(tx - ref[:, cols.index("x")], ty - ref[:, cols.index("y")]))
        )
    samples = {}
    for layout in layouts:
        sc = scene(layout, "occluder" in tracks)
        sc.tracker.reset()
        # The follower frame's origin is the robot axle at the start.
        sc.place(states[0.0], (10.0, 10.0), 0.0)
        axle = np.mean([sc.d.site(s + "_axle").xpos for s in ("left", "right")], axis=0)
        origin = axle[:2]

        def state(t, sc=sc, origin=origin):
            q = states[t] if t in states else states[min(states, key=_distance_to(t))]
            tau = t - rp["start_s"]
            x, y, _, tag_yaw = _interp_track(tracks["target"], tau)
            occ = None
            if "occluder" in tracks:
                ox, oy, oh, _ = _interp_track(tracks["occluder"], tau)
                occ = (ox + origin[0], oy + origin[1], oh)
            sc.place(
                q,
                (x + origin[0], y + origin[1]),
                None if math.isnan(tag_yaw) else tag_yaw,
                occ,
            )

        rng = np.random.default_rng(_seed("replay", name, layout))
        rows = []
        keep = []
        for k, te in enumerate(frame_ends):
            mid = round(te - exposure / 2, 6)
            state(mid)
            truth = sc.truth()
            occluded = False
            if "occluder" in tracks:
                ox, oy, _, _ = _interp_track(tracks["occluder"], mid - rp["start_s"])
                normal = sc.d.geom_xmat[sc.tag_geom].reshape(3, 3)[:, 0]
                tag_yaw = math.atan2(normal[1], normal[0])
                occluded = F.occluded(
                    truth["cam_pos"][:2],
                    np.array(truth["centre"][:2]),
                    tag_yaw,
                    sc.size / 2,
                    np.array([ox + origin[0], oy + origin[1]]),
                )
            image = sc.camera.expose(
                lambda t, state=state: state(round(t, 6)), sc.d, te, exposure, n_sub, rng
            )
            state(mid)
            truth = sc.truth()
            detections = sc.detect_all(image, truth, roi=True)
            body = sc.d.xmat[sc.chassis].reshape(3, 3)
            i = int(np.searchsorted(trace["time"], mid))
            i = min(i, len(trace["time"]) - 1)
            row = {
                "t": float(te),
                "t_mid": mid,
                "occluded": bool(occluded),
                "speed_m_s": float(trace["velocity"][i]),
                "yaw_rate_rad_s": float(trace["yaw_rate"][i]),
                "body_pitch_deg": math.degrees(math.atan2(-body[2, 0], body[0, 0])),
                **flatten(truth, detections),
            }
            row["expect_detection"] = bool(
                row["in_frame"]
                and not row["occluded"]
                and row["view_deg"] <= EXPECT_VIEW_DEG
                and row["range_m"] <= 2.0
            )
            rows.append(row)
            if k % sample_frames == 0 or k in (len(frame_ends) // 2,):
                keep.append((float(te), image))
        out["layouts"][layout] = rows
        samples[layout] = keep
    return out, samples


# Summaries ----------------------------------------------------------------------------


def _stats(values):
    v = np.array([x for x in values if x is not None], dtype=float)
    if len(v) == 0:
        return {"n": 0}
    return {
        "n": len(v),
        "mean": float(v.mean()),
        "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
        "p95_abs": float(np.percentile(np.abs(v), 95)),
        "max_abs": float(np.abs(v).max()),
    }


def summarise(rows, keys, method):
    """Detection rate and errors grouped by keys, over the samples whose tag is in frame."""
    groups = {}
    for r in rows:
        if not r["in_frame"]:
            continue
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    out = []
    for key, rs in sorted(groups.items()):
        det = [r for r in rs if r[f"{method}_detected"]]
        out.append(
            {
                **dict(zip(keys, key, strict=True)),
                "n": len(rs),
                "detection_rate": len(det) / len(rs),
                "pixels_mean": float(np.mean([r["pixels"] for r in rs])),
                "view3d_mean_deg": float(np.mean([r["view3d_deg"] for r in rs])),
                "range_error_m": _stats(r[f"{method}_range_error_m"] for r in det),
                "bearing_error_deg": _stats(r[f"{method}_bearing_error_deg"] for r in det),
            }
        )
    return out


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0])
    for r in rows:
        keys += [k for k in r if k not in keys]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


def read_csv(path):
    def parse(v):
        if v in ("True", "False"):
            return v == "True"
        if v == "":
            return None
        try:
            return float(v)
        except ValueError:
            return v

    with path.open() as f:
        return [{k: parse(v) for k, v in r.items()} for r in csv.DictReader(f)]


def view_limit(summary, layout, threshold=0.9):
    """Largest view angle at which every in-band range keeps the detection rate."""
    out = {}
    views = sorted({r["view_set_deg"] for r in summary if r["layout"] == layout})
    ranges = sorted({r["range_set_m"] for r in summary if r["layout"] == layout})
    for r in ranges:
        ok = [
            v
            for v in views
            if all(
                s["detection_rate"] >= threshold
                for s in summary
                if s["layout"] == layout and s["range_set_m"] == r and s["view_set_deg"] <= v
            )
        ]
        out[f"{r:g}"] = max(ok) if ok else None
    return out


def dropouts(rows, method):
    """Detection over the frames where the tag should be seen, and the miss runs."""
    expected = [r for r in rows if r["expect_detection"]]
    if not expected:
        return {"frames": 0}
    hits = [r[f"{method}_detected"] for r in expected]
    runs, current = [], 0
    for r in rows:
        if r["expect_detection"] and not r[f"{method}_detected"]:
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)
    det = [r for r in expected if r[f"{method}_detected"]]
    return {
        "frames": len(expected),
        "detection_rate": float(np.mean(hits)),
        "miss_runs": len(runs),
        "longest_miss_frames": max(runs) if runs else 0,
        "range_error_m": _stats(r[f"{method}_range_error_m"] for r in det),
        "bearing_error_deg": _stats(r[f"{method}_bearing_error_deg"] for r in det),
    }


# Timing -------------------------------------------------------------------------------


def timing(frames, sequences, perc):
    """Detector time on this machine (no other load) and the Pi Zero 2 W estimate."""
    from . import detector as D

    det_cfg = perc["detector"]
    t = perc["timing"]
    tag_id = perc["tag"]["id"]
    repeats = t["repeats"]
    full = {}
    for dec in det_cfg["quad_decimate"]:
        for threads in det_cfg["threads"]:
            det = D.Detector(det_cfg, tag_id, dec, threads)
            times = []
            for image in frames:
                det.detect(image)
                per = []
                for _ in range(max(1, repeats // 4)):
                    per.append(det.detect(image)[1])
                times.append(statistics.median(per))
            full[f"dec{dec:g}_t{threads}"] = {
                "median_ms": 1e3 * statistics.median(times),
                "p95_ms": 1e3 * float(np.percentile(times, 95)),
                "frames": len(times),
            }
    roi_times, roi_used = [], 0
    for seq in sequences:
        tracker = D.Tracker(det_cfg, tag_id, DECIMATION[PROPOSED["acquisition"]], True)
        for image in seq:
            _, spent, used = tracker.detect(image)
            if used:
                roi_used += 1
                roi_times.append(spent)
    roi = {
        "median_ms": 1e3 * statistics.median(roi_times) if roi_times else None,
        "p95_ms": 1e3 * float(np.percentile(roi_times, 95)) if roi_times else None,
        "frames": len(roi_times),
        "basis": "Frames from consecutive replay sequences where the region of interest held "
        "the tag (one thread)",
    }
    # Pose time: on the Pi the pose comes from the C library (apriltag estimate_tag_pose,
    # homography and orthogonal iteration); the Python fit used here for accuracy is not
    # what the Pi would run. Timed as the extra detector time with pose estimation on, on
    # the region-of-interest crops.
    from . import camera as C

    shared, following = load()[:2]
    model = C.intrinsics(shared, perc)
    params = (model["f_px"], model["f_px"], model["cx_px"], model["cy_px"])
    det = D.Detector(det_cfg, tag_id, 1.0).det
    pose_times = []
    for seq in sequences:
        tracker = D.Tracker(det_cfg, tag_id, DECIMATION[PROPOSED["acquisition"]], True)
        for image in seq:
            if tracker.last is not None:
                x0, y0, x1, y1 = tracker.window(image.shape)
                crop = np.ascontiguousarray(image[y0:y1, x0:x1])
                plain, posed = [], []
                for _ in range(5):
                    t0 = time.perf_counter()
                    det.detect(crop)
                    plain.append(time.perf_counter() - t0)
                    t0 = time.perf_counter()
                    det.detect(crop, True, params, following["tag"]["size_m"])
                    posed.append(time.perf_counter() - t0)
                pose_times.append(max(0.0, statistics.median(posed) - statistics.median(plain)))
            tracker.detect(image)
    lo, hi = t["pi_zero_2w_geekbench6_single"]
    ratio = [t["host_geekbench6_single"] / hi, t["host_geekbench6_single"] / lo]

    def pi(ms):
        return None if ms is None else [ms * ratio[0], ms * ratio[1]]

    estimate = {
        k: {"median_ms": pi(v["median_ms"]), "p95_ms": pi(v["p95_ms"])} for k, v in full.items()
    }
    estimate["roi_t1"] = {"median_ms": pi(roi["median_ms"]), "p95_ms": pi(roi["p95_ms"])}
    pose_ms = 1e3 * statistics.median(pose_times) if pose_times else None
    estimate["pose_fit"] = {"median_ms": pi(pose_ms)}
    lat = t["latency_budget"]
    exposure = perc["camera"]["exposure"]["exposure_s"]
    budget = {}
    for setting, det_key in (("roi tracking", "roi_t1"), ("full frame dec2, 4 threads", "dec2_t4")):
        e = estimate[det_key]["p95_ms"]
        rows = {
            "exposure_half_s": exposure / 2,
            "readout_s": lat["readout_s"],
            "isp_s": lat["isp_s"],
            "detection_s": [x / 1e3 for x in e],
            "pose_s": [x / 1e3 for x in estimate["pose_fit"]["median_ms"]] if pose_ms else None,
            "link_s": lat["link_s"],
        }
        fixed = rows["exposure_half_s"] + rows["readout_s"] + rows["isp_s"] + rows["link_s"]
        pose_range = rows["pose_s"] or [0.0, 0.0]
        rows["total_s"] = [fixed + rows["detection_s"][i] + pose_range[i] for i in (0, 1)]
        budget[setting] = rows
    rate = {}
    for key in ("roi_t1", "dec2_t1", "dec2_t4", "dec1.5_t4", "dec1_t4"):
        ms = estimate[key]["median_ms"]
        if ms is None:
            continue
        rate[key] = {
            "one_core_hz": [1e3 / ms[1], 1e3 / ms[0]],
            "frames_on_parallel_cores_hz": [
                min(1e3 / ms[1] * t["pi_zero_2w_cores"], 56.0) if key.endswith("_t1") else None,
                min(1e3 / ms[0] * t["pi_zero_2w_cores"], 56.0) if key.endswith("_t1") else None,
            ],
        }
    return {
        "evidence": "Detector times measured on this machine on rendered frames; the Pi Zero 2 W "
        "figures are an estimate scaled by published single-core benchmarks, not a measurement",
        "host": {
            "cpu": t["host_cpu"],
            "full_frame": full,
            "roi": roi,
            "pose_c_library_ms": pose_ms,
        },
        "host_to_pi_ratio": ratio,
        "ratio_basis": t["basis"],
        "pi_zero_2w_estimate_ms": estimate,
        "pi_zero_2w_rate_hz": rate,
        "rate_basis": "One core: 1 / median detection time. Parallel cores: frames handed to the "
        "four cores in turn (throughput up to four times one core, capped at the 56 fps sensor "
        "mode); latency stays that of one frame on one core. Thread scaling inside one frame "
        "is taken from this machine (an assumption; the Zero 2 W's memory bandwidth may scale "
        "worse)",
        "latency_budget_s": budget,
        "latency_basis": t["latency_budget"]["basis"],
    }


# Measurement model --------------------------------------------------------------------


def fit_model(grid_rows, layout, perc, timing_record, replay_dropouts):
    """Range and bearing bias and noise, and detection probability, by range and view
    angle, from the grid's training samples (tracking: full resolution; acquisition:
    the decimated full-frame search)."""
    train = [r for r in grid_rows if r["layout"] == layout and not r["held_out"] and r["in_frame"]]
    ranges = sorted({r["range_set_m"] for r in train})
    views = sorted({r["view_set_deg"] for r in train})
    table = []
    for rg in ranges:
        for v in views:
            rs = [r for r in train if r["range_set_m"] == rg and r["view_set_deg"] == v]
            if not rs:
                continue
            det = [r for r in rs if r["dec1_detected"]]
            re = _stats(r["dec1_range_error_m"] for r in det)
            be = _stats(r["dec1_bearing_error_deg"] for r in det)
            table.append(
                {
                    "range_m": rg,
                    "view_deg": v,
                    "n": len(rs),
                    "p_detect_tracking": len(det) / len(rs),
                    "p_detect_acquisition": float(np.mean([r["dec2_detected"] for r in rs])),
                    "range_bias_m": re.get("mean"),
                    "range_std_m": re.get("std"),
                    "bearing_bias_deg": be.get("mean"),
                    "bearing_std_deg": be.get("std"),
                }
            )
    lat = timing_record["latency_budget_s"]["roi tracking"]["total_s"]
    return {
        "layout": layout,
        "evidence": "fitted on rendered frames (the static grid's training samples) under the "
        "stated camera, light and noise assumptions; not real footage",
        "measurements": "range: horizontal distance from the camera to the tag centre; bearing: "
        "angle of the tag centre in the robot's levelled heading frame, positive left",
        "detector": {
            "tracking": "region of interest around the last detection, full resolution",
            "acquisition": "full frame, quad_decimate 2",
        },
        "not_included": "lens calibration error (the pose uses the same lens model that "
        "rendered the frames), the attitude error of the levelling (the simulator's attitude "
        "is used), rolling shutter, ISP denoising and sharpening, real lighting, glare and "
        "print flatness; the spreads here are the rendered-frame floor, and phase D designs "
        "with margins above them",
        "interpolation": "bilinear in range and view angle; outside the table the nearest "
        "entry; p_detect is 0 when the tag is outside the frame or occluded",
        "range_m": ranges,
        "view_deg": views,
        "table": table,
        "latency_s": lat,
        "latency_basis": "middle of the exposure to the measurement at the Teensy, the region-"
        "of-interest path (timing.json latency_budget_s); a range from the Pi estimate",
        "rate_hz": perc["camera"]["output"]["frame_rate_hz"],
        "rate_basis": "30 Hz is held only with the region-of-interest search on the Zero 2 W "
        "(timing.json); a full-frame search after a loss takes several frames",
        "dropouts": replay_dropouts,
    }


def fit_motion_bias(blur_rows, model, f_px):
    """The extra range bias of yaw-rate blur: the blur set's error above the static table,
    fitted as c (blur px)^2 (range m)^2 through the origin, the blur being yaw rate x
    exposure x focal length. Pitch-rate blur adds under 0.15 cm and is left out."""
    pts = []
    for r in blur_rows:
        if not r["dec1_detected"] or r["pitch_rate_rad_s"] != 0 or r["yaw_rate_rad_s"] == 0:
            continue
        blur_px = r["yaw_rate_rad_s"] * r["exposure_s"] * f_px
        static = model_lookup(model, r["range_m"], r["view_deg"])["range_bias_m"]
        pts.append((blur_px, r["range_m"], r["dec1_range_error_m"] - static))
    b, rg, y = (np.array(v) for v in zip(*pts))
    x = b**2 * rg**2
    c = float(x @ y / (x @ x))
    res = y - c * x
    return {
        "coefficient_m": c,
        "formula": "extra range bias (m) = coefficient_m x (|bearing rate| x exposure x f_px)^2 "
        "x (range m)^2, added to range_bias_m; the bearing rate is the tag's angular rate in "
        "the levelled heading frame (equal to the yaw rate for a still target)",
        "f_px": f_px,
        "fit": {
            "n": len(y),
            "blur_px_max": float(b.max()),
            "rms_m": float(np.sqrt(np.mean(res**2))),
            "max_abs_m": float(np.abs(res).max()),
        },
        "basis": "blur set (blur/samples.csv): waist tag at 0.7, 1.0 and 1.5 m, yaw rates 0.5 to "
        "1.4 rad/s, exposures 5 to 20 ms, detections only; the blur smears the black square into "
        "the white border and the sRGB tone curve then shrinks it, so the tag reads smaller",
        "domain": "blur up to fit.blur_px_max; beyond it the detection is lost (blur/summary.json)",
    }


def motion_bias(model, rng, yaw_rate, exposure_s):
    m = model.get("motion_range_bias")
    if not m:
        return 0.0
    return m["coefficient_m"] * (abs(yaw_rate) * exposure_s * m["f_px"]) ** 2 * rng**2


def model_lookup(model, rng, view):
    """Bilinear lookup of the model's table at (range, view angle)."""
    ranges, views = model["range_m"], model["view_deg"]
    cells = {(e["range_m"], e["view_deg"]): e for e in model["table"]}

    def bracket(axis, x):
        x = min(max(x, axis[0]), axis[-1])
        i = max(0, min(len(axis) - 2, int(np.searchsorted(axis, x)) - 1))
        a, b = axis[i], axis[i + 1]
        return i, (x - a) / (b - a) if b > a else 0.0

    i, fr = bracket(ranges, rng)
    j, fv = bracket(views, view)
    out = {}
    for key in (
        "p_detect_tracking",
        "p_detect_acquisition",
        "range_bias_m",
        "range_std_m",
        "bearing_bias_deg",
        "bearing_std_deg",
    ):
        acc, wsum = 0.0, 0.0
        for di, wi in ((0, 1 - fr), (1, fr)):
            for dj, wj in ((0, 1 - fv), (1, fv)):
                e = cells.get((ranges[i + di], views[j + dj]))
                if e is None or e[key] is None or wi * wj == 0:
                    continue
                acc += wi * wj * e[key]
                wsum += wi * wj
        out[key] = acc / wsum if wsum else None
    return out


def check_model(model, grid_rows, replay_rows, perc):
    """The model against held-out rendered runs: per bin, the detection rate, the error
    spread and the bias within the stated bounds."""
    c = perc["model_check"]
    tol_p = c["detection_probability_tolerance"]
    lo, hi = c["noise_ratio_bounds"]
    exposure = perc["camera"]["exposure"]["exposure_s"]
    held = [
        r for r in grid_rows if r["layout"] == model["layout"] and r["held_out"] and r["in_frame"]
    ]
    sets = {"grid_held_out": (held, "dec1")}
    sets["replays_roi"] = (replay_rows, "roi")
    out = {}
    for name, (rows, method) in sets.items():
        bins = {}
        for r in rows:
            if name == "replays_roi" and not r["expect_detection"]:
                continue
            rg = min(model["range_m"], key=lambda x, r=r: abs(x - r["range_m"]))
            v = min(model["view_deg"], key=lambda x, r=r: abs(x - r["view_deg"]))
            bins.setdefault((rg, v), []).append(r)
        results = []
        for (rg, v), rs in sorted(bins.items()):
            if len(rs) < c["minimum_samples"]:
                continue
            m = model_lookup(model, rg, v)
            det = [r for r in rs if r[f"{method}_detected"]]
            # Replays move: take the blur term off each range error first, at the tag's
            # bearing rate (the robot's yaw rate less the target's own turn about the camera).
            extra = {
                id(r): motion_bias(model, r["range_m"], r.get("bearing_rate_rad_s", 0.0), exposure)
                for r in det
            }
            rate = len(det) / len(rs)
            row = {"range_m": rg, "view_deg": v, "n": len(rs), "detection_rate": rate}
            row["p_detect_model"] = m["p_detect_tracking"]
            row["detection_ok"] = abs(rate - m["p_detect_tracking"]) <= tol_p
            for q, unit, tol in (
                ("range", "m", c["bias_tolerance"]["range_m"]),
                ("bearing", "deg", c["bias_tolerance"]["bearing_deg"]),
            ):
                errs = [
                    r[f"{method}_{q}_error_{unit}"] - (extra[id(r)] if q == "range" else 0.0)
                    for r in det
                ]
                if len(errs) < c["minimum_samples"] or m[f"{q}_std_{unit}"] is None:
                    continue
                std = float(np.std(errs, ddof=1))
                mean = float(np.mean(errs))
                floor = {"m": 1e-4, "deg": 0.01}[unit]
                ratio = max(std, floor) / max(m[f"{q}_std_{unit}"], floor)
                row[f"{q}_std"] = std
                row[f"{q}_p95_abs_residual"] = float(
                    np.percentile(np.abs(np.array(errs) - m[f"{q}_bias_{unit}"]), 95)
                )
                row[f"{q}_std_model"] = m[f"{q}_std_{unit}"]
                row[f"{q}_std_ok"] = lo <= ratio <= hi
                row[f"{q}_bias"] = mean
                row[f"{q}_bias_model"] = m[f"{q}_bias_{unit}"]
                row[f"{q}_bias_ok"] = abs(
                    mean - m[f"{q}_bias_{unit}"]
                ) <= tol + 2 * std / math.sqrt(len(errs))
            results.append(row)
        oks = [v for r in results for k, v in r.items() if k.endswith("_ok")]
        out[name] = {
            "bins": results,
            "checks": len(oks),
            "passed": int(sum(oks)),
            "all_passed": bool(all(oks)),
        }
    return out


# Plots --------------------------------------------------------------------------------


def plot_grid(summary, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    layouts = sorted({r["layout"] for r in summary})
    fig, axs = plt.subplots(len(layouts), 3, figsize=(15, 4.2 * len(layouts)), squeeze=False)
    for row, layout in zip(axs, layouts, strict=True):
        views = sorted({r["view_set_deg"] for r in summary if r["layout"] == layout})
        for v in views:
            rs = sorted(
                (r for r in summary if r["layout"] == layout and r["view_set_deg"] == v),
                key=lambda r: r["range_set_m"],
            )
            x = [r["range_set_m"] for r in rs]
            row[0].plot(x, [r["detection_rate"] for r in rs], "o-", label=f"{v:g} deg")
            row[1].plot(x, [r["range_error_m"].get("mean", np.nan) * 100 for r in rs], "o-")
            row[2].plot(x, [r["bearing_error_deg"].get("std", np.nan) for r in rs], "o-")
        row[0].set_ylabel(f"{layout}\ndetection rate")
        row[1].set_ylabel("mean range error (cm)")
        row[2].set_ylabel("bearing error std (deg)")
        for ax in row:
            ax.set_xlabel("range (m)")
            ax.grid(alpha=0.3)
        row[0].legend(title="view angle", fontsize=7)
    fig.suptitle("M12 phase C: AprilTag on rendered frames, full resolution (dec 1)")
    fig.tight_layout()
    fig.savefig(output / "grid.png", dpi=80)
    plt.close(fig)


def plot_replays(replays, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(replays)
    fig, axs = plt.subplots(len(names), 1, figsize=(12, 1.9 * len(names)), squeeze=False)
    for ax, name in zip(axs[:, 0], names, strict=True):
        rows = replays[name]
        t = np.array([r["t"] for r in rows])
        ax.plot(t, [r["range_m"] for r in rows], "k-", lw=1, label="range (m)")
        exp = np.array([r["expect_detection"] for r in rows])
        roi = np.array([r["roi_detected"] for r in rows])
        ax.plot(t[exp & ~roi], np.full((exp & ~roi).sum(), 0.2), "r|", label="missed")
        ax.plot(t[~exp], np.full((~exp).sum(), 0.1), "|", color="0.6", label="not expected")
        ax.set_ylabel(name, fontsize=8)
        ax.set_ylim(0, 2.2)
        ax.grid(alpha=0.3)
    axs[0, 0].legend(fontsize=7, ncol=3)
    axs[-1, 0].set_xlabel("time (s)")
    fig.suptitle("M12 phase C replays (waist tag): region-of-interest tracking")
    fig.tight_layout()
    fig.savefig(output / "replays.png", dpi=80)
    plt.close(fig)


# Main ---------------------------------------------------------------------------------


def _chunks(items, n):
    size = max(1, math.ceil(len(items) / n))
    return [items[i : i + size] for i in range(0, len(items), size)]


def build(workers=8, skip=(), resume=False):
    from PIL import Image

    from ..profiles import native
    from ..robustness_control import build_candidate, equivalence
    from . import camera as C
    from . import speed as SP

    shared, following, p, perc = load()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    model = C.intrinsics(shared, perc)
    geometry = C.render_geometry(model, perc["camera"]["render"]["supersample"])
    lens = shared["m12"]["camera"]["lenses"][perc["camera"]["lens"]]
    import pupil_apriltags

    inputs = copy.deepcopy(perc)
    inputs.pop("measurement_model", None)
    record = {
        "evidence": "rendered frames under stated assumptions; not real footage, not a Pi",
        "perception_inputs": inputs,
        "perception_inputs_sha256": _sha(inputs),
        "following_sha256": _sha(following),
        "camera_sensor": shared["m12"]["camera_sensor"],
        "intrinsics": model,
        "distortion_monotonic": C.monotonic_over_image(model),
        "render": {k: v for k, v in geometry.items() if k != "map"},
        "electrons_at_white": C.electrons_at_white(
            perc["camera"]["noise"],
            perc["camera"]["exposure"]["exposure_s"],
            lens["f_number"],
            perc["camera"]["output"]["native_pixels_per_output_pixel"] ** 2,
            shared["m12"]["camera_sensor"]["pixel_size_m"] ** 2,
        ),
        "detector_library": f"pupil-apriltags {pupil_apriltags.__version__ if hasattr(pupil_apriltags, '__version__') else ''}".strip(),
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
    }
    (OUTPUT / "config.json").write_text(json.dumps(record, indent=1, default=float) + "\n")
    timings = {}

    with ProcessPoolExecutor(max_workers=workers) as pool:
        if "grid" not in skip:
            t0 = time.time()
            points = grid_points(perc, following, lens)
            rows = []
            futures = [pool.submit(grid_task, c) for c in _chunks(points, workers * 8)]
            for fut in as_completed(futures):
                rows.extend(fut.result())
            rows.sort(
                key=lambda r: (
                    r["layout"],
                    r["range_set_m"],
                    r["bearing_fraction"],
                    r["view_set_deg"],
                    r["pitch_offset_deg"],
                    r["sample"],
                )
            )
            write_csv(OUTPUT / "grid/samples.csv", rows)
            timings["grid_s"] = time.time() - t0
            print(
                f"M12 PERCEPTION grid {len(rows)} frames in {timings['grid_s']:.0f} s", flush=True
            )
        if "blur" not in skip:
            t0 = time.time()
            points = blur_points(perc)
            rows = []
            futures = [pool.submit(blur_task, c) for c in _chunks(points, workers * 4)]
            for fut in as_completed(futures):
                rows.extend(fut.result())
            rows.sort(
                key=lambda r: (
                    r["range_set_m"],
                    r["yaw_rate_rad_s"],
                    r["pitch_rate_rad_s"],
                    r["exposure_s"],
                    r["sample"],
                )
            )
            write_csv(OUTPUT / "blur/samples.csv", rows)
            timings["blur_s"] = time.time() - t0
            print(
                f"M12 PERCEPTION blur {len(rows)} frames in {timings['blur_s']:.0f} s", flush=True
            )
        if "replays" not in skip:
            t0 = time.time()
            library = build_candidate(p, SP.load()[1], ROOT / native(p)["firmware_build"])
            check = equivalence(p, SP.load()[1], library)
            if not check["passed"]:
                raise ValueError(f"C/Python mismatch: {check}")
            rp = perc["replays"]
            layouts = [rp["layout"], rp["comparison_layout"]]
            folder = OUTPUT / "replays"

            def todo(name):
                # With resume, a scenario already rendered reruns its physics only (its
                # run record), not the rendering.
                done = all((folder / lay / f"{name}.csv").exists() for lay in layouts)
                return [] if resume and done else layouts

            futures = {
                pool.submit(replay_task, (name, str(library), todo(name), 45)): name
                for name in rp["scenarios"]
            }
            info = {}
            for fut in as_completed(futures):
                out, samples = fut.result()
                name = out["scenario"]
                for layout, rows in out["layouts"].items():
                    write_csv(folder / layout / f"{name}.csv", rows)
                info[name] = {k: v for k, v in out.items() if k != "layouts"}
                for layout, keep in samples.items():
                    if layout == rp["layout"] and name in ("straight_1.0", "corner_90"):
                        for t, image in keep[:3]:
                            png = folder / "frames" / layout / f"{name}_t{t:05.2f}.png"
                            png.parent.mkdir(parents=True, exist_ok=True)
                            Image.fromarray(image).save(png)
                print(f"M12 PERCEPTION replay {name} done", flush=True)
            (folder / "runs.json").write_text(json.dumps(info, indent=1, default=float) + "\n")
            timings["replays_s"] = time.time() - t0
    # Timing on a fixed rendered set: consecutive frames for the region of interest, and
    # every fourth of them for the full-frame search.
    if "replays" not in skip or not (OUTPUT / "timing.json").exists():
        sequences = timing_sequences(perc)
        frames = [img for seq in sequences for img in seq[::4]]
        t_record = timing(frames, sequences, perc)
        (OUTPUT / "timing.json").write_text(json.dumps(t_record, indent=1, default=float) + "\n")
    return finish(perc, timings)


def timing_sequences(perc, seconds=2.0):
    """Consecutive 30 Hz frames of a moving tag (straight walk at 1.0 m/s, robot placed
    kinematically) for the region-of-interest timing."""
    sc = scene(perc["replays"]["layout"])
    rate = perc["camera"]["output"]["frame_rate_hz"]
    rng = np.random.default_rng(_seed("timing"))
    out = []
    for speed, view in ((1.0, 0.0), (0.6, 30.0)):
        seq = []
        for k in range(int(seconds * rate)):
            t = k / rate
            sc.place((0.0, 0.0, 0.0, 0.0, 40), (0.0, 0.0), 0.0)
            cam = sc.d.cam_xpos[sc.cid].copy()
            tag = cam[:2] + np.array([0.8 + speed * t / 2, 0.15 * math.sin(2 * t)])
            facing = math.atan2(cam[1] - tag[1], cam[0] - tag[0]) + math.radians(view)
            sc.place((0.0, 0.0, 0.0, 0.0, 40), tag, facing)
            seq.append(
                sc.camera.develop(
                    sc.camera.linear(sc.d), perc["camera"]["exposure"]["exposure_s"], rng
                )
            )
        out.append(seq)
    return out


def finish(perc, timings=None):
    """Summaries, the measurement model, its check and the plots from the saved samples."""
    grid_rows = read_csv(OUTPUT / "grid/samples.csv")
    blur_rows = read_csv(OUTPUT / "blur/samples.csv")
    t_record = json.loads((OUTPUT / "timing.json").read_text())
    rp = perc["replays"]
    grid = {}
    for method in METHODS:
        grid[method] = summarise(grid_rows, ["layout", "range_set_m", "view_set_deg"], method)
    by_bearing = summarise(grid_rows, ["layout", "range_set_m", "bearing_fraction"], "dec1")
    by_pitch = summarise(grid_rows, ["layout", "range_set_m", "pitch_offset_deg"], "dec1")
    limits = {
        layout: {m: view_limit(grid[m], layout) for m in METHODS}
        for layout in perc["grid"]["layouts"]
    }
    out_of_frame = {
        layout: sum(1 for r in grid_rows if r["layout"] == layout and not r["in_frame"])
        for layout in perc["grid"]["layouts"]
    }
    grid_record = {
        "evidence": "static robot and tag, rendered frames, the stated assumptions",
        "by_range_view": grid,
        "by_range_bearing_dec1": by_bearing,
        "by_range_pitch_dec1": by_pitch,
        "view_angle_limit_deg_at_90pct": limits,
        "samples_out_of_frame": out_of_frame,
    }
    (OUTPUT / "grid/summary.json").write_text(json.dumps(grid_record, indent=1) + "\n")
    blur = {
        m: summarise(
            blur_rows, ["range_set_m", "yaw_rate_rad_s", "pitch_rate_rad_s", "exposure_s"], m
        )
        for m in ("dec1", "dec2")
    }
    (OUTPUT / "blur/summary.json").write_text(json.dumps(blur, indent=1) + "\n")
    replays, replay_summary = {}, {}
    for layout in (rp["layout"], rp["comparison_layout"]):
        replay_summary[layout] = {}
        for name in rp["scenarios"]:
            path = OUTPUT / "replays" / layout / f"{name}.csv"
            if not path.exists():
                continue
            rows = read_csv(path)
            # The tag's angular rate across the image (the blur rate), from the bearing track.
            bearing = np.unwrap(np.radians([r["bearing_deg"] for r in rows]))
            rate = np.gradient(bearing, [r["t"] for r in rows]) if len(rows) > 1 else [0.0]
            for r, w in zip(rows, rate):
                r["bearing_rate_rad_s"] = float(w)
            if layout == rp["layout"]:
                replays[name] = rows
            replay_summary[layout][name] = {
                m: dropouts(rows, m) for m in ("roi", "dec2", "dec1.5", "dec1")
            }
            replay_summary[layout][name]["frames"] = len(rows)
            replay_summary[layout][name]["frames_expected"] = sum(
                r["expect_detection"] for r in rows
            )
    runs = json.loads((OUTPUT / "replays/runs.json").read_text())
    (OUTPUT / "replays/summary.json").write_text(
        json.dumps({"runs": runs, "dropouts": replay_summary}, indent=1) + "\n"
    )
    all_replay_rows = [r for rows in replays.values() for r in rows]
    pooled = {m: dropouts(all_replay_rows, m) for m in ("roi", "dec2")}
    # Burstiness: probability that a missed expected frame follows a miss.
    after_miss = []
    for rows in replays.values():
        prev = None
        for r in rows:
            if r["expect_detection"]:
                if prev is not None and prev is False:
                    after_miss.append(not r["roi_detected"])
                prev = r["roi_detected"]
            else:
                prev = None
    replay_drop = {
        "pooled_roi": pooled["roi"],
        "pooled_dec2": pooled["dec2"],
        "p_miss_after_miss_roi": float(np.mean(after_miss)) if after_miss else None,
        "basis": "Frames of the waist-tag replays where the tag is in frame, unoccluded, within "
        "the 60 degree view angle and 2 m",
    }
    model = fit_model(grid_rows, rp["layout"], perc, t_record, replay_drop)
    from . import camera as C

    f_px = C.intrinsics(load()[0], perc)["f_px"]
    model["motion_range_bias"] = fit_motion_bias(
        [r for r in blur_rows if r["layout"] == rp["layout"]], model, f_px
    )
    check = check_model(model, grid_rows, all_replay_rows, perc)
    (OUTPUT / "model_check.json").write_text(json.dumps(check, indent=1) + "\n")
    cfg = json.loads(CONFIG.read_text())
    model["check"] = {
        k: {"checks": v["checks"], "passed": v["passed"], "all_passed": v["all_passed"]}
        for k, v in check.items()
    }
    cfg["measurement_model"] = model
    CONFIG.write_text(dumps(cfg))
    plot_grid(grid["dec1"], OUTPUT)
    if replays:
        plot_replays(replays, OUTPUT)
    inputs = copy.deepcopy(cfg)
    inputs.pop("measurement_model", None)
    summary = {
        "evidence": "perception measured on rendered frames under the stated assumptions "
        "(config/perception-m12.json); no real footage; the Pi timing is an estimate",
        "perception_inputs_sha256": _sha(inputs),
        "view_angle_limit_deg_at_90pct": limits,
        "replay_dropouts_waist_tag": {
            n: {
                "roi_detection_rate": v["roi"].get("detection_rate"),
                "roi_longest_miss_frames": v["roi"].get("longest_miss_frames"),
                "dec2_detection_rate": v["dec2"].get("detection_rate"),
            }
            for n, v in replay_summary.get(rp["layout"], {}).items()
        },
        "pooled_replays": replay_drop,
        "timing": {
            "host_roi_ms": t_record["host"]["roi"]["median_ms"],
            "host_full_frame_ms": {
                k: v["median_ms"] for k, v in t_record["host"]["full_frame"].items()
            },
            "pi_estimate_ms": t_record["pi_zero_2w_estimate_ms"],
            "latency_budget_s": t_record["latency_budget_s"],
        },
        "model_check": model["check"],
        "build_times_s": timings or {},
    }
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--skip", default="", help="comma-separated parts to reuse: grid, blur, replays"
    )
    parser.add_argument("--finish", action="store_true", help="only rebuild the summaries")
    parser.add_argument(
        "--resume", action="store_true", help="keep replays already rendered (physics rerun)"
    )
    args = parser.parse_args()
    if args.finish:
        summary = finish(load()[3])
    else:
        summary = build(args.workers, tuple(s for s in args.skip.split(",") if s), args.resume)
    print(json.dumps(summary["view_angle_limit_deg_at_90pct"]))
    print(json.dumps(summary["model_check"]))


if __name__ == "__main__":
    main()
