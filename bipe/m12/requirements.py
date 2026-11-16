"""M12 phase A: following requirements, scenarios, phase B targets and rendered frames.

.venv/bin/python -m bipe.m12.requirements [--rerun-yaw] [--no-frames]

Writes artifacts/m12/requirements/:
- requirements.json: the proposed requirements (config/following-m12.json) checked against
  the head-camera geometry, the measured yaw response, the ideal follower and the
  rendered frames; the phase B targets; the open decisions for sign-off;
- scenarios.json and follower_traces.npz: the scenario definitions and follower traces;
- yaw_response/: the M10 turn case the bearing-rate limit is taken from (Python
  controller with the sensor model, 12 s; reused unless --rerun-yaw);
- frames/: head-camera frames per tag layout and scenario, with frames.json;
- paths_<layout>.png: target and robot paths.

Evidence kind: a layout study plus an ideal-follower simulation under stated
assumptions. The frames are rendered with the robot placed kinematically at the
follower's poses; they are not a balance simulation.
"""

import argparse
import copy
import csv
import ctypes
import hashlib
import json
import math
import os

import numpy as np

from ..parameters import ROOT, load_parameters
from . import follower as F
from . import geometry as G
from . import scenarios as S

OUTPUT = ROOT / "artifacts/m12/requirements"
FOLLOWING = ROOT / "config/following-m12.json"
POSTURES_DEG = (30, 40, 60)
FRAME_SIZE = (360, 640)


def load():
    shared = json.loads((ROOT / "config/robot.json").read_text())
    following = json.loads(FOLLOWING.read_text())
    p = load_parameters(profile="m10")
    return shared, following, p


def _sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


# Yaw response -------------------------------------------------------------------------


def yaw_response(p, output, rerun=False):
    """Dead time, rise, peak yaw acceleration and steady heading lag of the M10 body
    turn (the turn acceptance command: 0.4 rad/s for 3 s each way)."""
    summary_path = output / "summary.json"
    if summary_path.exists() and not rerun:
        return json.loads(summary_path.read_text())
    from ..moving_experiment import COLUMNS, simulate

    run = simulate("turn", 12, sensing="sensors", nominal=p, plant=p)
    if run.get("failure"):
        raise RuntimeError(f"turn case failed: {run['failure']}")
    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    t, w = f["time"], f["yaw_rate"]
    dt = float(t[1] - t[0])
    k = max(1, round(0.05 / dt))
    smooth = np.convolve(w, np.ones(k) / k, "same")
    accel = np.gradient(smooth, dt)
    steps = []
    for t0, rate in ((2.0, 0.4), (7.0, -0.4)):
        i0 = int(np.searchsorted(t, t0 - 1e-9))
        seg = slice(i0, i0 + round(1.5 / dt))
        rel = w[seg] / rate
        ts = t[seg] - t0
        hold = (t >= t0 + 2.0) & (t < t0 + 3.0)
        lag = float(
            np.mean(np.radians(f["heading_reference_deg"][hold] - f["yaw_deg"][hold])) / rate
        )
        steps.append(
            {
                "command_rad_s": rate,
                "dead_time_s": float(ts[np.argmax(rel >= 0.1)]),
                "rise_10_90_s": float(ts[np.argmax(rel >= 0.9)] - ts[np.argmax(rel >= 0.1)]),
                "peak_yaw_acceleration_rad_s2": float(np.max(np.abs(accel[seg]))),
                "steady_heading_lag_s": lag,
            }
        )
    result = {
        "case": "moving_experiment turn: 0.4 rad/s 2-5 s, -0.4 rad/s 7-10 s, 12 s, "
        "nominal posture, Python controller, sensor model",
        "steps": steps,
        "heading_lag_s": max(s["steady_heading_lag_s"] for s in steps),
        "yaw_acceleration_rad_s2": min(s["peak_yaw_acceleration_rad_s2"] for s in steps),
        "dead_time_s": max(s["dead_time_s"] for s in steps),
        "peak_pitch_error_deg": float(np.max(np.abs(f["pitch_error_deg"]))),
        "inputs": {
            "controller_sha256": hashlib.sha256(
                (ROOT / "config/controller_m10.json").read_bytes()
            ).hexdigest(),
            "sensors_sha256": hashlib.sha256(
                (ROOT / "config/sensors-m10.json").read_bytes()
            ).hexdigest(),
            "plant_geometry_source_sha256": p["geometry_source_sha256"],
            "seed": run["seed"],
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    keep = ["time", "user_yaw_rate", "yaw_rate", "heading_reference_deg", "yaw_deg"]
    keep += ["pitch_error_deg", "velocity"]
    with (output / "trace.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(keep)
        for row in run["trace"]:
            writer.writerow([f"{row[COLUMNS.index(c)]:.6g}" for c in keep])
    summary_path.write_text(json.dumps(result, indent=2) + "\n")
    return result


# Geometry -----------------------------------------------------------------------------


def geometry_tables(shared, following, p, yaw):
    mount = G.camera_mount(shared)
    cam, tag = following["camera"], following["tag"]
    allowance = math.radians(cam["body_pitch_allowance_deg"])
    poses = [
        G.camera_pose(p, mount, a, o) for a in POSTURES_DEG for o in (-allowance, 0.0, allowance)
    ]
    band = following["range"]["band_m"]
    engage = following["engage"]["range_m"]
    lag = yaw["heading_lag_s"] + cam["perception_latency_s_assumption"]
    lenses = {}
    for name, lens in shared["m12"]["camera"]["lenses"].items():
        v, h = lens["vertical_fov_deg"], lens["horizontal_fov_deg"]
        margin = cam["edge_margin_fraction"]
        rows = {
            "vertical_band_range_band": G.vertical_band(
                poses, v, np.linspace(*band, 9), tag["size_m"], margin
            ),
            "vertical_band_engage_window": G.vertical_band(
                poses, v, np.linspace(*engage, 11), tag["size_m"], margin
            ),
            "camera_pitch_windows": {},
            "by_range": [],
        }
        for height in (0.3, 0.4, 0.5, 0.6, 0.8, 0.95, 1.1):
            window = G.elevation_window(
                poses, v, np.linspace(*engage, 11), height, tag["size_m"], margin
            )
            rows["camera_pitch_windows"][f"{height:.2f}"] = window
        for r in (0.5, 0.7, 1.0, 1.5, 2.0):
            allowance_h = G.bearing_allowance_rad(h, margin, tag["size_m"], r)
            low, high = G.visible_heights(G.camera_pose(p, mount, 40), v, r, margin)
            rows["by_range"].append(
                {
                    "range_m": r,
                    "visible_heights_nominal_m": [low, high],
                    "pixels_on_tag": G.pixels_on_tag(h, cam["image_width_px"], tag["size_m"], r),
                    "pixels_on_tag_at_max_view_angle": G.pixels_on_tag(
                        h,
                        cam["image_width_px"],
                        tag["size_m"] * math.cos(math.radians(tag["max_view_angle_deg"])),
                        r,
                    ),
                    "bearing_allowance_deg": math.degrees(allowance_h),
                    "max_bearing_rate_rad_s_m10_yaw": G.max_bearing_rate(
                        allowance_h, lag, yaw["yaw_acceleration_rad_s2"]
                    ),
                }
            )
        lenses[name] = rows
    return {
        "mount": mount,
        "poses": poses,
        "pitch_allowance_deg": cam["body_pitch_allowance_deg"],
        "bearing_rate_lag_s": lag,
        "bearing_rate_lag_basis": "M10 steady heading lag in the turn case plus the "
        "perception latency assumption",
        "lenses": lenses,
        "source": shared["m12"]["camera"]["source"],
    }


# Follower -----------------------------------------------------------------------------


def _ceil(value, step):
    return math.ceil(value / step - 1e-9) * step


def envelope(defs, all_tracks, sight, following):
    """Run 1: perfect knowledge, no limits; the peaks over the in-requirement scenarios."""
    rows = {}
    for name, sc in defs.items():
        if sc["expect"] == "stop_at_hold":
            continue
        result = F.run(all_tracks[name], sight, following, perfect=True)
        rows[name] = F.summary(result, following, straight=name.startswith("straight"))
        rows[name].pop("events")
    keys = {
        "speed": ("peak_speed_m_s", 0.1),
        "acceleration": ("peak_acceleration_m_s2", 0.25),
        "yaw_rate": ("peak_yaw_rate_rad_s", 0.1),
        "yaw_acceleration": ("peak_yaw_acceleration_rad_s2", 0.25),
    }
    peaks = {k: max(r[key] for r in rows.values()) for k, (key, _) in keys.items()}
    targets = {k: _ceil(peaks[k], step) for k, (_, step) in keys.items()}
    targets["acceleration"] = max(
        targets["acceleration"], following["loss"]["brake_deceleration_m_s2"]
    )
    targets["reverse"] = following["close_range"]["reverse_speed_max_m_s"]
    g = 9.81
    targets["lean_deg"] = math.degrees(math.atan2(targets["acceleration"], g))
    return {"by_scenario": rows, "peaks": peaks, "targets": targets}


def check(name, sc, s, following):
    """Requirement checks for one limited follower run (run 2)."""
    events = s["events"]
    kinds = [e["event"] for e in events]
    stops = [e["stopping_distance_m"] for e in events if e["event"] == "holding"]
    engaged = [e["t"] for e in events if e["event"] == "engaged"]
    limit = following["loss"]["stopping_distance_max_m"]
    out = {
        "engaged_at_s": engaged[0] if engaged else None,
        "engage_ok": bool(engaged) and engaged[0] >= following["engage"]["minimum_s"] - 1e-9,
    }
    expect = sc["expect"]
    if expect == "follow":
        ok = "lost" not in kinds and s["range_in_band"]
    elif expect == "follow_reacquire":
        ok = "holding" not in kinds and kinds.count("lost") == kinds.count("reacquired")
        ok = ok and s["range_in_band"]
    elif expect == "close_range":
        ok = "lost" not in kinds and s["range_min_m"] >= following["close_range"]["stop_range_m"]
    elif expect == "stop_at_hold":
        ok = "holding" in kinds and all(d <= limit for d in stops)
        ok = ok and "engaged" not in kinds[kinds.index("holding") :]
    else:
        ok = "holding" in kinds and "engaged" in kinds[kinds.index("holding") :]
        ok = ok and all(d <= limit for d in stops)
    out["expectation_met"] = bool(ok and out["engage_ok"])
    out["stopping_distances_m"] = stops
    if name.startswith("straight"):
        out["steady_bearing_ok"] = s["steady_bearing_ok"]
        out["expectation_met"] = out["expectation_met"] and s["steady_bearing_ok"]
    return out


# Rendering ----------------------------------------------------------------------------


def gl_strings(renderer):
    try:
        renderer._gl_context.make_current()
        gl = ctypes.CDLL("libOpenGL.so.0")
        gl.glGetString.restype = ctypes.c_char_p
        return {
            k: gl.glGetString(code).decode()
            for k, code in (("vendor", 0x1F00), ("renderer", 0x1F01), ("version", 0x1F02))
        }
    except (AttributeError, OSError):
        return None


def frame_times(sc, trace, columns):
    f = dict(zip(columns, trace.T, strict=True))
    end = float(f["t"][-1])
    times = [1.0]
    moving = np.abs(f["bearing"]) * (f["mode"] == F.MODES.index("following"))
    times.append(float(f["t"][int(np.argmax(moving))]) if moving.max() > 0.02 else end / 2)
    lost = np.flatnonzero(f["visible"] < 0.5)
    lost = lost[f["t"][lost] > 1.0]
    if len(lost):
        times.append(float(f["t"][lost[0]]) + 0.1)
    times.append(end - 0.5)
    return sorted({round(t, 2) for t in times})


def render(shared, following, p, defs, all_tracks, runs, output):
    """Head-camera frames at the follower's poses, with a segmentation cross-check of the
    geometric visibility."""
    import mujoco
    from PIL import Image

    mount = G.camera_mount(shared)
    lens_name = following["camera"]["lens_proposal"]
    lenses = shared["m12"]["camera"]["lenses"]
    g = p["sizing"]["gravity"]
    index, info, compare = [], None, []
    jobs = [(layout, lens_name) for layout in following["tag"]["layouts"]]
    jobs.append(("low_tag", "standard"))
    for layout_name, lens_key in jobs:
        layout = following["tag"]["layouts"][layout_name]
        lens = lenses[lens_key]
        sight = F.Sight(p, mount, lens, following, layout)
        for occluder in (False, True):
            xml = S.world_xml(p, mount, lens, following, layout, occluder=occluder)
            m = mujoco.MjModel.from_xml_string(xml)
            d = mujoco.MjData(m)
            renderer = S.renderer(m, FRAME_SIZE)
            segment = S.renderer(m, FRAME_SIZE, segmentation=True)
            if info is None:
                info = {
                    "backend": os.environ.get("MUJOCO_GL", "default"),
                    "mujoco": mujoco.__version__,
                    "gl": gl_strings(renderer),
                    "size": list(FRAME_SIZE),
                    "render_fovy_deg": {k: S.render_fovy_deg(v) for k, v in lenses.items()},
                    "repeat_identical": [],
                }
            tag_id = m.geom("target_tag").id
            first = None
            for name, sc in defs.items():
                if ("occluder" in sc) != occluder:
                    continue
                if lens_key == "standard" and name not in ("straight_1.0", "lateral_crossing"):
                    continue
                result = runs[layout_name][name]
                f = dict(zip(result["columns"], result["trace"].T, strict=True))
                folder = (
                    output / layout_name / (name if lens_key == lens_name else f"{name}_{lens_key}")
                )
                folder.mkdir(parents=True, exist_ok=True)
                for t in frame_times(sc, result["trace"], result["columns"]):
                    i = min(len(f["t"]) - 1, round(t / S.DT))
                    x, y, yaw, a = (float(f[k][i]) for k in ("x", "y", "yaw", "acceleration"))
                    lean = math.degrees(math.atan2(a, g))
                    S.place_robot(m, d, p, x, y, yaw, pitch_offset_deg=lean)
                    tgt = S.at(all_tracks[name]["target"], t)
                    tag = np.array([tgt["x"], tgt["y"]])
                    camera, _ = sight.camera(x, y, yaw, a)
                    tag_yaw = tgt["tag_yaw"]
                    if math.isnan(tag_yaw):
                        tag_yaw = math.atan2(*(camera - tag)[::-1])
                    S.place_person(m, d, "target", *tag, tag_yaw)
                    occ_xy = None
                    if occluder:
                        o = S.at(all_tracks[name]["occluder"], t)
                        occ_xy = np.array([o["x"], o["y"]])
                        S.place_person(m, d, "occluder", *occ_xy, o["heading"])
                    mujoco.mj_forward(m, d)
                    image = S.frame(renderer, d)
                    if first is None:
                        first = (image, d.qpos.copy(), d.mocap_pos.copy(), d.mocap_quat.copy())
                    seg = S.frame(segment, d)
                    mask = (seg[..., 0] == tag_id) & (seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
                    rows_, cols_ = np.nonzero(mask)
                    border = bool(
                        len(rows_)
                        and (
                            rows_.min() == 0
                            or cols_.min() == 0
                            or rows_.max() == FRAME_SIZE[0] - 1
                            or cols_.max() == FRAME_SIZE[1] - 1
                        )
                    )
                    seen = sight.look((x, y, yaw), a, tag, tag_yaw, occ_xy)
                    png = folder / f"t{t:05.2f}.png"
                    Image.fromarray(image).save(png)
                    row = {
                        "layout": layout_name,
                        "lens": lens_key,
                        "scenario": name,
                        "t": t,
                        "file": str(png.relative_to(output)),
                        "sha256": hashlib.sha256(image.tobytes()).hexdigest(),
                        "robot": {"x": x, "y": y, "yaw": yaw, "lean_deg": lean},
                        "target": {"x": tgt["x"], "y": tgt["y"], "tag_yaw": tag_yaw},
                        "mode": F.MODES[int(f["mode"][i])],
                        "geometric": {
                            "range_m": seen["range"],
                            "bearing_deg": math.degrees(seen["bearing"]),
                            "view_angle_deg": math.degrees(seen["view_angle"]),
                            "in_view": seen["in_view"],
                            "occluded": seen["occluded"],
                            "visible": seen["visible"],
                        },
                        "rendered": {
                            "tag_pixels": int(mask.sum()),
                            "tag_box_px": [
                                int(cols_.min()),
                                int(rows_.min()),
                                int(cols_.max()),
                                int(rows_.max()),
                            ]
                            if len(rows_)
                            else None,
                            "touches_border": border,
                        },
                    }
                    # In view and unoccluded geometrically: the whole tag is in the image.
                    if seen["in_view"] and not seen["occluded"]:
                        compare.append(row["rendered"]["tag_pixels"] > 0 and not border)
                    index.append(row)
            # Determinism: the first frame again, after all the others.
            if first is not None:
                d.qpos[:], d.mocap_pos[:], d.mocap_quat[:] = first[1:]
                mujoco.mj_forward(m, d)
                info["repeat_identical"].append(
                    bool(np.array_equal(S.frame(renderer, d), first[0]))
                )
            renderer.close()
            segment.close()
    info["geometry_agrees_with_render"] = {"checked": len(compare), "agree": int(sum(compare))}
    (output / "frames.json").write_text(json.dumps({"render": info, "frames": index}, indent=1))
    return info, index


def plot_paths(defs, all_tracks, runs, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for layout, results in runs.items():
        n = len(defs)
        cols = 4
        fig, axs = plt.subplots(math.ceil(n / cols), cols, figsize=(16, 4 * math.ceil(n / cols)))
        for ax, (name, sc) in zip(axs.ravel(), defs.items(), strict=False):
            tr = all_tracks[name]
            f = dict(zip(results[name]["columns"], results[name]["trace"].T, strict=True))
            ax.plot(tr["target"]["x"], tr["target"]["y"], label="target (tag)")
            ax.plot(f["x"], f["y"], label="robot axle")
            if "occluder" in tr:
                ax.plot(tr["occluder"]["x"], tr["occluder"]["y"], ":", label="occluder")
            lost = f["visible"] < 0.5
            ax.plot(f["x"][lost], f["y"][lost], "r.", ms=2, label="tag not detectable")
            for e in results[name]["events"]:
                if e["event"] in ("lost", "holding", "reacquired", "engaged"):
                    i = min(len(f["t"]) - 1, round(e["t"] / S.DT))
                    ax.annotate(e["event"], (f["x"][i], f["y"][i]), fontsize=7)
            ax.set_title(f"{name} ({sc['expect']})", fontsize=9)
            ys = np.concatenate([tr["target"]["y"], f["y"]])
            middle = (ys.max() + ys.min()) / 2
            half = max(1.0, (ys.max() - ys.min()) / 2 + 0.2)
            ax.set_ylim(middle - half, middle + half)
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(alpha=0.3)
        for ax in axs.ravel()[n:]:
            ax.axis("off")
        axs.ravel()[0].legend(fontsize=7)
        fig.suptitle(f"M12 phase A ideal follower with the phase B limits, {layout}")
        fig.tight_layout()
        fig.savefig(output / f"paths_{layout}.png", dpi=80)
        plt.close(fig)


# Main ---------------------------------------------------------------------------------


def build(rerun_yaw=False, frames=True):
    shared, following, p = load()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    yaw = yaw_response(p, OUTPUT / "yaw_response", rerun_yaw)
    geom = geometry_tables(shared, following, p, yaw)
    mount = geom["mount"]
    lens = shared["m12"]["camera"]["lenses"][following["camera"]["lens_proposal"]]
    nominal = following["range"]["nominal_m"]
    ahead = G.camera_pose(p, mount, 40)["ahead_of_axle_m"]
    defs = S.definitions(following, nominal + ahead)
    all_tracks = {n: S.tracks(sc, nominal + ahead, nominal) for n, sc in defs.items()}
    first_layout = next(iter(following["tag"]["layouts"].values()))
    env = envelope(defs, all_tracks, F.Sight(p, mount, lens, following, first_layout), following)
    limits = env["targets"]
    latency = following["camera"]["perception_latency_s_assumption"]
    runs, checks, traces = {}, {}, {}
    for layout_name, layout in following["tag"]["layouts"].items():
        sight = F.Sight(p, mount, lens, following, layout)
        runs[layout_name], checks[layout_name] = {}, {}
        for name, sc in defs.items():
            result = F.run(all_tracks[name], sight, following, limits=limits, latency_s=latency)
            runs[layout_name][name] = result
            s = F.summary(result, following, straight=name.startswith("straight"))
            checks[layout_name][name] = {"summary": s, "checks": check(name, sc, s, following)}
            traces[f"{layout_name}/{name}"] = result["trace"].astype(np.float32)
    np.savez_compressed(OUTPUT / "follower_traces.npz", columns=np.array(F.COLUMNS), **traces)
    scenario_record = {
        name: {**copy.deepcopy(sc), "duration_s": float(all_tracks[name]["target"]["t"][-1])}
        for name, sc in defs.items()
    }
    (OUTPUT / "scenarios.json").write_text(
        json.dumps({"expectations": S.EXPECTATIONS, "scenarios": scenario_record}, indent=1) + "\n"
    )
    plot_paths(defs, all_tracks, runs, OUTPUT)
    render_info, frame_rows = (None, [])
    if frames:
        render_info, frame_rows = render(
            shared, following, p, defs, all_tracks, runs, OUTPUT / "frames"
        )
    allowance_needed = env["targets"]["lean_deg"]
    record = {
        "status": "proposed requirements for sign-off; evidence: layout study and ideal "
        "follower under stated assumptions (no balance dynamics, no perception)",
        "following": following,
        "following_sha256": _sha(following),
        "camera": shared["m12"]["camera"],
        "yaw_response": yaw,
        "geometry": geom,
        "pitch_allowance_covers_lean": following["camera"]["body_pitch_allowance_deg"]
        >= allowance_needed,
        "envelope_run": {
            "assumptions": "perfect target knowledge, no limits, no latency, first-order "
            "speed and yaw lags (config ideal_follower)",
            **env,
        },
        "phase_b_targets": env["targets"],
        "check_run": {
            "assumptions": "phase B targets as limits, perception latency and detection "
            "rate assumptions, geometric visibility, engage, loss and reacquisition rules",
            "lens": following["camera"]["lens_proposal"],
            "layouts": checks,
            "all_expectations_met": all(
                c["checks"]["expectation_met"] for rows in checks.values() for c in rows.values()
            ),
        },
        "render": render_info,
        "frames": len(frame_rows),
    }
    (OUTPUT / "requirements.json").write_text(
        json.dumps(record, indent=1, default=float, allow_nan=False) + "\n"
    )
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rerun-yaw", action="store_true")
    parser.add_argument("--no-frames", action="store_true")
    args = parser.parse_args()
    record = build(args.rerun_yaw, not args.no_frames)
    t = record["phase_b_targets"]
    print(
        f"M12 PHASE A targets: speed {t['speed']:.1f} m/s, acceleration "
        f"{t['acceleration']:.2f} m/s^2, yaw rate {t['yaw_rate']:.1f} rad/s, yaw "
        f"acceleration {t['yaw_acceleration']:.2f} rad/s^2; expectations met: "
        f"{record['check_run']['all_expectations_met']}; frames {record['frames']}"
    )


if __name__ == "__main__":
    main()
