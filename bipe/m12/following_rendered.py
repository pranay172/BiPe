"""M12 phase F: closed-loop following with the head camera rendered in the loop.

.venv/bin/python -m bipe.m12.following_rendered [--workers N] [--stage check|matrix|all]

The phase D closed loop (bipe.m12.following.closed_task: the free-base M10 plant, the C
balance core with the sensor model, the C follower) with the phase D link model replaced
by bipe.m12.rendered_link: each 30 Hz frame is rendered from the plant's state and read
by the phase C tracker. Stages (outputs in artifacts/m12/following_rendered/):
- check: seed 0 of the twelve scenarios against the phase D runs on the measurement
  model (artifacts/m12/following/closed_loop/runs.json): detection rate, misses and
  losses, measured range and bearing error, the follower's modes and events, and the run
  time; check.json;
- matrix: the phase F runs (nominal over seeds, the latency sweep, the robustness
  variants, the slopes and the lighting variants); runs.json, traces/, frames/ (each
  processed frame's truth and detection) and a few example images per scenario.

Evidence: perception on rendered frames in the loop under the phase C camera, light and
noise assumptions, with the phase C Pi Zero 2 W timing estimate; not real footage or
hardware.
"""

import argparse
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

from ..parameters import ROOT
from . import following as FL
from . import rendered_link as RL
from . import speed as SP

OUTPUT = ROOT / RL.load()["output_dir"]
PHASE_D_RUNS = ROOT / "artifacts/m12/following/closed_loop/runs.json"
LATENCIES_S = (0.10, 0.20)
FRAME_COLUMNS = [
    "t",
    "sequence",
    "tracking",
    "roi_used",
    "possible",
    "detected",
    "in_frame",
    "pixels",
    "range_m",
    "bearing_rad",
    "view_deg",
    "measured_range_m",
    "measured_bearing_rad",
    "decision_margin",
    "reprojection_px",
    "detector_s",
    "latency_s",
    "gated",
    "measured_view_deg",
]


def payloads(library, stage):
    """(group, closed_task payload) for the check or the matrix."""
    env = FL.setup()
    rcfg = RL.load()
    names = list(env["defs"])

    def item(group, name, seed=0, centre=0.15, variant=None, slope=0.0, lighting="nominal"):
        rendered = {"lighting": lighting, "keep": rcfg["kept_frames_per_run"] if seed == 0 else 0}
        return group, (name, seed, centre, variant, slope, library, {"rendered": rendered})

    if stage == "check":
        return [item("nominal", name) for name in names]
    out = [item("nominal", n, s) for n in names for s in range(rcfg["nominal_seeds"])]
    out += [item(f"latency_{c:.2f}", n, centre=c) for c in LATENCIES_S for n in names]
    out += [item("variants", n, variant=v) for v in FL.VARIANTS for n in FL.VARIANT_SCENARIOS]
    out += [item("slopes", n, slope=s) for n, s in FL.SLOPE_CASES]
    out += [
        item(f"light_{light}", n, lighting=light)
        for light in rcfg["lighting"]
        if light != "nominal"
        for n in rcfg["lighting_scenarios"]
    ]
    return out


def run_key(payload):
    key = FL.run_key(payload)
    lighting = payload[6]["rendered"]["lighting"]
    return key if lighting == "nominal" else f"{key}_{lighting}"


def _keep_sequences(payload, env, n):
    """n frame sequence numbers spread over the target's track (the first and last
    seconds are the robot alone)."""
    if n <= 0:
        return []
    name = payload[0]
    from . import scenarios as S

    tracks = S.tracks(env["defs"][name], env["nominal"] + env["ahead"], env["nominal"])
    start = env["link"]["start_s"]
    end = start + float(tracks["target"]["t"][-1])
    rate = env["link"]["frame_rate_hz"]
    return [round(rate * (start + (end - start) * (k + 0.5) / n)) for k in range(n)]


def _worker(item):
    group, payload = item
    env = FL.setup()
    rendered = dict(payload[6]["rendered"])
    rendered["keep_frames"] = _keep_sequences(payload, env, rendered.pop("keep"))
    payload = (*payload[:6], {**payload[6], "rendered": rendered})
    t0 = time.time()
    run = FL.closed_task(payload)
    run["wall_s"] = time.time() - t0
    return group, run_key(payload), run


def frame_table(frames):
    out = np.full((len(frames), len(FRAME_COLUMNS)), np.nan)
    for i, f in enumerate(frames):
        for j, c in enumerate(FRAME_COLUMNS):
            v = f[c]
            out[i, j] = np.nan if v is None else float(v)
    return out


def frame_metrics(source):
    """Detection rate, misses, and the measured range and bearing errors of the rendered
    frames (detections against the simulator's truth). A detection the view gate dropped
    counts as a miss, since the follower never receives it."""
    frames = [dict(f, detected=f["detected"] and not f.get("gated", False)) for f in source]
    n = len(frames)
    detected = [f["detected"] for f in frames]
    runs, run = [], 0
    for f in frames:
        if not f["detected"] and f["in_frame"]:
            run += 1
        elif run:
            runs.append(run)
            run = 0
    if run:
        runs.append(run)
    hits = [f for f in frames if f["detected"]]
    re = np.array([f["measured_range_m"] - f["range_m"] for f in hits])
    be = np.degrees(
        [math.remainder(f["measured_bearing_rad"] - f["bearing_rad"], 2 * math.pi) for f in hits]
    )
    possible = [f for f in frames if f["possible"]]
    return {
        "frames": n,
        "detected": int(sum(detected)),
        "in_frame": int(sum(f["in_frame"] for f in frames)),
        "possible": len(possible),
        "detected_when_possible": int(sum(f["detected"] for f in possible)),
        "detected_not_possible": int(sum(f["detected"] and not f["possible"] for f in frames)),
        "miss_runs": len(runs),
        "miss_run_max": max(runs, default=0),
        "range_error_mean_m": float(re.mean()) if len(re) else None,
        "range_error_std_m": float(re.std()) if len(re) else None,
        "range_error_max_abs_m": float(np.abs(re).max()) if len(re) else None,
        "bearing_error_mean_deg": float(be.mean()) if len(be) else None,
        "bearing_error_std_deg": float(be.std()) if len(be) else None,
        "bearing_error_max_abs_deg": float(np.abs(be).max()) if len(be) else None,
        "detector_s_mean": float(np.mean([f["detector_s"] for f in frames])) if n else None,
        "gated": int(sum(f.get("gated", False) for f in source)),
    }


def _save(out, key, run, metrics):
    from PIL import Image

    np.savez_compressed(
        out / "traces" / f"{key}.npz",
        columns=np.array(run["columns"]),
        trace=FL._thin(run).astype(np.float32),
    )
    np.savez_compressed(
        out / "frames" / f"{key}.npz",
        columns=np.array(FRAME_COLUMNS),
        frames=frame_table(run["frames"]),
    )
    for t, image in run.get("kept_frames", []):
        d = out / "images" / key
        d.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image).save(d / f"t{t:06.3f}.png", optimize=True)


def build(stage, workers, groups=None):
    p, gains, _, speed_cfg = SP.load()
    from ..profiles import native
    from ..robustness_control import build_candidate

    library = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
    env = FL.setup()
    out = OUTPUT
    for sub in ("traces", "frames", "images"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    (out / "config.json").write_text(
        json.dumps(
            {
                "rendered": RL.load(),
                "follower": env["cfg"],
                "perception_camera": env["perc"]["camera"],
                "following_requirements_sha256": FL._sha(env["following"]),
            },
            indent=1,
        )
        + "\n"
    )
    record = out / "runs.json"
    done = json.loads(record.read_text()) if record.exists() else {}
    todo = [
        it
        for it in payloads(library, stage)
        if run_key(it[1]) not in done and (groups is None or it[0] in groups)
    ]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers, max_tasks_per_child=1) as pool:
        futures = [pool.submit(_worker, it) for it in todo]
        for fut in as_completed(futures):
            group, key, run = fut.result()
            metrics = FL.closed_metrics(run, env, speed_cfg)
            metrics["group"] = group
            metrics["lighting"] = run["rendered"]["lighting"]
            metrics["perception"] = frame_metrics(run["frames"])
            metrics["repeat_mismatches"] = run["repeat_mismatches"]
            metrics["wall_s"] = run["wall_s"]
            _save(out, key, run, metrics)
            done[key] = metrics
            record.write_text(json.dumps(done, indent=1, default=float) + "\n")
            print(
                "M12 RENDERED",
                key,
                "passed" if metrics["passed"] else "FAILED",
                metrics["failure"] or "",
                f"{run['wall_s']:.0f} s run, {time.time() - t0:.0f} s total",
                flush=True,
            )
    return done


def check(runs):
    """Seed 0 of each scenario against the phase D run on the measurement model."""
    phase_d = json.loads(PHASE_D_RUNS.read_text())
    rows = {}
    for key, r in runs.items():
        if r["group"] != "nominal" or r["seed"] != 0:
            continue
        d = phase_d.get(key)
        p = r["perception"]
        row = {
            "rendered": {
                "passed": r["passed"],
                "fell": r["fell"],
                "detection_rate": p["detected"] / max(p["frames"], 1),
                "detected_when_possible": p["detected_when_possible"] / max(p["possible"], 1),
                "detected_not_possible": p["detected_not_possible"],
                "miss_runs": p["miss_runs"],
                "miss_run_max": p["miss_run_max"],
                "losses_to_hold": r["losses_to_hold"],
                "events": [e["event"] for e in r["events"]],
                "coasts": len(r["coasts"]),
                "estimate": r["estimate"],
                "range_error_mean_m": p["range_error_mean_m"],
                "range_error_std_m": p["range_error_std_m"],
                "bearing_error_mean_deg": p["bearing_error_mean_deg"],
                "bearing_error_std_deg": p["bearing_error_std_deg"],
                "repeat_mismatches": r["repeat_mismatches"],
                "wall_s": r["wall_s"],
            }
        }
        if d is not None:
            row["model"] = {
                "passed": d["passed"],
                "fell": d["fell"],
                "detection_rate": d["frames_detected"] / max(d["frames"], 1),
                "detected_when_possible": d["frames_detected"] / max(d["frames_possible"], 1),
                "losses_to_hold": d["losses_to_hold"],
                "events": [e["event"] for e in d["events"]],
                "coasts": len(d["coasts"]),
                "estimate": d["estimate"],
            }
            row["same_events"] = row["model"]["events"] == row["rendered"]["events"]
            row["same_pass"] = row["model"]["passed"] == row["rendered"]["passed"]
        rows[key] = row
    walls = [r["rendered"]["wall_s"] for r in rows.values()]
    result = {
        "runs": rows,
        "same_pass": sum(r.get("same_pass", False) for r in rows.values()),
        "same_events": sum(r.get("same_events", False) for r in rows.values()),
        "compared": sum("model" in r for r in rows.values()),
        "repeat_mismatches": sum(r["rendered"]["repeat_mismatches"] for r in rows.values()),
        "wall_s_mean": float(np.mean(walls)) if walls else None,
        "wall_s_max": float(np.max(walls)) if walls else None,
    }
    (OUTPUT / "check.json").write_text(json.dumps(result, indent=1, default=float) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--stage", default="check", choices=["check", "matrix", "all"])
    parser.add_argument(
        "--groups", help="comma-separated matrix groups to run (default: all), e.g. nominal"
    )
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    runs = build("check", args.workers)
    r = check(runs)
    print(
        "M12 RENDERED check: same pass",
        f"{r['same_pass']}/{r['compared']}, same events {r['same_events']}/{r['compared']},",
        f"repeat mismatches {r['repeat_mismatches']}, mean run {r['wall_s_mean']:.0f} s",
        flush=True,
    )
    if args.stage in ("matrix", "all"):
        build("matrix", args.workers, args.groups.split(",") if args.groups else None)
        print("M12 RENDERED matrix done", flush=True)


if __name__ == "__main__":
    main()
