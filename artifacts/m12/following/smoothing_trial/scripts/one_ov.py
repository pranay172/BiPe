"""Run closed-loop cases with config overrides; print one JSON line per case."""

import json
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from bipe.m12 import following as FL
from bipe.profiles import native
from bipe.robustness_control import build_candidate


def work(item):
    tag, name, variant, ov, lib = item
    env = FL.setup()
    _, _, _, speed_cfg = FL.SP.load()
    run = FL.closed_task((name, 0, 0.15, variant, 0.0, lib, ov))
    m = FL.closed_metrics(run, env, speed_cfg)
    s = m["summary"]
    return {
        "tag": tag,
        "name": name,
        "variant": variant,
        "passed": m["passed"],
        "fell": m["fell"],
        "failure": m["failure"],
        "pitch": round(m["peak_pitch_deg"], 2),
        "rmin": s.get("range_min_m"),
        "rmax": s.get("range_max_m"),
        "bearing": s.get("peak_bearing_deg"),
        "exp": m["checks"]["expectation_met"],
    }


if __name__ == "__main__":
    items = json.loads(Path(sys.argv[1]).read_text())
    workers = int(sys.argv[2])
    p, gains, _, _ = FL.SP.load()
    LIB = str(build_candidate(p, gains, FL.ROOT / native(p)["firmware_build"]))
    with ProcessPoolExecutor(workers, initializer=None) as pool:
        for f in as_completed([pool.submit(work, [*it, LIB]) for it in items]):
            print(json.dumps(f.result(), default=float), flush=True)
