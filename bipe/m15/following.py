"""M15 phase B: camera following with the locomotion policy in place of the classical core.

    python -m bipe.m15.following --checkpoint PATH [--link model|rendered] [--workers 4]
        [--group nominal ...] [--limit NAME=VALUE] [--output DIR]
    python -m bipe.m15.following --classical [...]

``--classical`` runs the same loop with the classical C core (no policy swap): the
classical record under follower limits other than the signed-off ones (M16 decision 5).

The M12 closed loop (bipe.m12.following.closed_task: the free-base M10 plant, the sensor
model, the C follower) with the policy running where the C balance core ran
(bipe.m13.policy_control.classes): the follower's speed and yaw-rate commands drive the
policy exactly as the speed suite's commands do. ``--link model`` is the phase D
measurement model (artifacts/m12/following/closed_loop for the classical record);
``--link rendered`` renders every frame at 5 ms with the 74 degree view gate, as the
signed-off phase F matrix (artifacts/m12/following_rendered_5ms). Rendering needs
MUJOCO_GL=osmesa in the cloud.

Records go to local/m15/following_<link>_<checkpoint parent> unless --output is given:
runs.json (bipe.m12.following.closed_metrics per run, plus policy_steps) and
summary.json (pass and fall counts per group, against the classical record).

Evidence: a validated simulation under the M12 assumptions, not hardware.
"""

import argparse
import json
import multiprocessing
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from ..m12 import following as FL
from ..m12 import speed as SP
from ..parameters import ROOT

CLASSICAL = {
    "model": ROOT / "artifacts/m12/following/closed_loop/runs.json",
    "rendered": ROOT / "artifacts/m12/following_rendered_5ms/runs.json",
}
ACTOR = None  # set in the parent before the fork


def _items(link, library, limits=None):
    """(group, payload, key); ``limits`` overrides the follower's limits section."""

    def patched(p):
        if not limits:
            return p
        extra = dict(p[6]) if len(p) > 6 else {}
        extra["limits"] = {**extra.get("limits", {}), **limits}
        return (*p[:6], extra)

    if link == "model":
        return [(g, patched(p), FL.run_key(p)) for g, p in FL.closed_payloads(library)]
    from ..m12 import following_rendered as FR

    return [(g, patched(p), FR.run_key(p)) for g, p in FR.payloads(library, "matrix")]


def _worker(item):
    from contextlib import nullcontext

    from ..m13.policy_control import classes

    link, group, payload, key = item
    t0 = time.time()
    start = ACTOR.calls if ACTOR else 0
    with classes(ACTOR) if ACTOR else nullcontext():
        if link == "model":
            run = FL.closed_task(payload)
        else:
            from ..m12 import following_rendered as FR

            _, _, run = FR._worker((group, payload))
    run["policy_steps"] = ACTOR.calls - start if ACTOR else 0
    run["wall_s"] = time.time() - t0
    return group, key, run


def summary(runs, classical):
    groups = defaultdict(lambda: {"runs": 0, "passed": 0, "falls": 0, "classical_passed": 0})
    for key, m in runs.items():
        g = groups[m["group"]]
        g["runs"] += 1
        g["passed"] += bool(m["passed"])
        g["falls"] += bool(m["failure"])
        g["classical_passed"] += bool(classical.get(key, {}).get("passed"))
    total = {
        k: sum(g[k] for g in groups.values())
        for k in ("runs", "passed", "falls", "classical_passed")
    }
    flips = {
        "lost": sorted(
            k for k, m in runs.items() if not m["passed"] and classical.get(k, {}).get("passed")
        ),
        "gained": sorted(
            k
            for k, m in runs.items()
            if m["passed"] and k in classical and not classical[k]["passed"]
        ),
    }
    return {"groups": dict(groups), "all": total, "against_classical": flips}


def main():
    global ACTOR
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--checkpoint")
    parser.add_argument("--classical", action="store_true", help="the classical core, no policy")
    parser.add_argument("--link", choices=["model", "rendered"], default="model")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--group", action="append")
    parser.add_argument("--key", action="append", help="only these run keys")
    parser.add_argument(
        "--limit", action="append", default=[], metavar="NAME=VALUE",
        help="override a follower limit (config/follower-m12.json limits), e.g. "
        "acceleration_m_s2=1.0",
    )  # fmt: skip
    parser.add_argument("--output")
    args = parser.parse_args()
    if not (args.classical or args.checkpoint):
        parser.error("--checkpoint is required unless --classical")
    limits = {k: float(v) for k, v in (a.split("=", 1) for a in args.limit)}
    multiprocessing.set_start_method("fork", force=True)

    from ..m13.policy_control import Actor, Counted
    from ..profiles import native
    from ..robustness_control import build_candidate

    ckpt = Path(args.checkpoint) if args.checkpoint else None
    out = (
        Path(args.output)
        if args.output
        else ROOT / f"local/m15/following_{args.link}_{ckpt.parent.name if ckpt else 'classical'}"
    )
    out.mkdir(parents=True, exist_ok=True)
    ACTOR = Counted(Actor(str(ckpt))) if ckpt else None
    p, gains, _, speed_cfg = SP.load()
    library = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
    env = FL.setup()
    record = out / "runs.json"
    done = json.loads(record.read_text()) if record.exists() else {}
    todo = [
        (args.link, g, pl, k)
        for g, pl, k in _items(args.link, library, limits)
        if k not in done and (not args.group or g in args.group) and (not args.key or k in args.key)
    ]
    # Seed 0 of every scenario first, then the next seed: a partial matrix covers them all.
    todo.sort(key=lambda it: (it[2][1], it[0] != "nominal"))
    (out / "config.json").write_text(
        json.dumps(
            {"checkpoint": str(ckpt) if ckpt else "classical", "link": args.link, "limits": limits},
            indent=1,
        )
        + "\n"
    )
    print(f"M15 FOLLOWING {len(todo)} runs to do, {len(done)} done", flush=True)
    t0 = time.time()
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("fork")
    ) as pool:
        futures = [pool.submit(_worker, it) for it in todo]
        for fut in as_completed(futures):
            group, key, run = fut.result()
            m = FL.closed_metrics(run, env, speed_cfg)
            m["group"] = group
            m["policy_steps"] = run["policy_steps"]
            m["wall_s"] = run["wall_s"]
            if args.link == "rendered":
                from ..m12 import following_rendered as FR

                m["lighting"] = run["rendered"]["lighting"]
                m["perception"] = FR.frame_metrics(run["frames"])
            np.savez_compressed(
                out / f"{key}.npz",
                columns=np.array(run["columns"]),
                trace=FL._thin(run).astype(np.float32),
            )
            done[key] = m
            record.write_text(json.dumps(done, indent=1, default=float) + "\n")
            print(
                "M15 FOLLOWING", key, "passed" if m["passed"] else "FAILED", m["failure"] or "",
                f"{run['wall_s']:.0f} s, {run['policy_steps']} policy steps, {time.time() - t0:.0f} s total",
                flush=True,
            )  # fmt: skip
    classical = json.loads(CLASSICAL[args.link].read_text())
    s = summary(done, classical)
    (out / "summary.json").write_text(json.dumps(s, indent=1) + "\n")
    print(json.dumps(s["all"]), json.dumps(s["against_classical"]), flush=True)


if __name__ == "__main__":
    main()
