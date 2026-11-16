"""M15 phase F: three expressive moves as command sequences for the locomotion policy.

    python -m bipe.m15.moves [--checkpoint PATH] [--output DIR] [--workers 4] [--case NAME ...]

No training: each move is a speed, yaw-rate and posture-label sequence that the policy
already takes (its command is the rate-limited speed, yaw rate and posture label, as in
training). The moves run as M12 speed cases on the full M10 plant with the policy in
place of the classical core (bipe.m13.full_plant speed, with the moves in place of the
speed suite's cases):
- ``crouch_bob``: at rest, three bobs between the 35 and 60 degree postures at the
  training's 20 degree/s posture rate limit, then back to 40;
- ``spin_stop``: one turn at the 1.0 rad/s envelope yaw rate, a stop, one turn back;
- ``wiggle``: at rest, the yaw rate alternating +-0.8 rad/s every 0.5 s for 4 s.
Each runs nominal and under the speed suite's seven robustness variants.

Judged by the speed suite's policy rule (bipe.m14.evaluate.speed_rule: the checks
without the joint-error criterion) for the nominal cases, survival and recovery for the
variants. The axle height check is reported, not judged: the bob moves the posture on
purpose, and the steady segments are judged by speed and yaw only. Results go to
local/m15/moves_<checkpoint parent> (cases.json, summary.json from the speed runner;
moves.json with the verdicts).

Evidence: a validated simulation under the M12 assumptions, not hardware.
"""

import argparse
import copy
import json
import sys
from pathlib import Path

from ..parameters import ROOT

CHECKPOINT = ROOT / "artifacts/m14/checkpoints/finetune_s1/model_9998.pt"


def _bob():
    t, knots = 1.0, [[0, 40], [1.0, 40]]
    for _ in range(3):
        t += 0.25
        knots.append([t, 35])  # 40 -> 35 at 20 deg/s
        t += 1.25
        knots.append([t, 60])
        t += 1.25
        knots.append([t, 35])
        t += 0.5
        knots.append([t, 35])
    t += 0.25
    knots.append([t, 40])
    end = t + 5.0
    knots.append([end, 40])
    return {
        "purpose": "Three crouch bobs at rest, 35 to 60 degree posture at 20 deg/s",
        "knots": [[0, 0, 0], [end, 0, 0]],
        "posture_knots": knots,
        "duration": end,
    }


def _spin():
    turn = 2 * 3.141592653589793  # one turn at 1.0 rad/s
    a, b = 1.0, 1.0 + turn
    c, d = b + 2.0, b + 2.0 + turn
    return {
        "purpose": "One turn at 1.0 rad/s, a stop, one turn back",
        "knots": [
            [0, 0, 0],
            [a, 0, 0],
            [a, 0, 1.0],
            [b, 0, 1.0],
            [b, 0, 0],
            [c, 0, 0],
            [c, 0, -1.0],
            [d, 0, -1.0],
            [d, 0, 0],
            [d + 5.0, 0, 0],
        ],
        "yaw_windows": [[a + 1.0, b], [c + 1.0, d]],
        "duration": d + 5.0,
    }


def _wiggle():
    knots, t, sign = [[0, 0, 0], [1.0, 0, 0]], 1.0, 1.0
    for _ in range(8):
        knots += [[t, 0, sign * 0.8], [t + 0.5, 0, sign * 0.8]]
        t += 0.5
        sign = -sign
    knots += [[t, 0, 0], [t + 5.0, 0, 0]]
    return {
        "purpose": "Yaw wiggle at rest: +-0.8 rad/s every 0.5 s for 4 s",
        "knots": knots,
        "duration": t + 5.0,
    }


MOVES = {"crouch_bob": _bob, "spin_stop": _spin, "wiggle": _wiggle}


def definitions(cfg):
    """The moves, nominal and under the speed suite's robustness variants."""
    cases = {}
    for name, make in MOVES.items():
        cases[name] = {"name": name, "gate": "full", **make()}
        for variant in cfg["variants"]["names"]:
            v = f"{variant}_{name}"
            cases[v] = {**copy.deepcopy(cases[name]), "name": v, "variant": variant,
                        "purpose": f"{name} with {variant}", "gate": "survive"}  # fmt: skip
    return cases


def judge(output):
    from ..m14.evaluate import speed_rule

    criteria = json.loads((ROOT / "config/speed-m12.json").read_text())["criteria"]
    rows = json.loads((output / "cases.json").read_text())
    verdicts = {}
    for name, r in rows.items():
        v = speed_rule(r, criteria, ("steady", float("inf")))
        verdicts[name] = {
            "passed": v and v["passed"],
            "checks": v and {k: x for k, x in v["checks"].items() if k != "axle_depth_p95"},
            "axle_depth_error": r["axle_depth_error"],
            "peak_pitch_deg": r["speed"]["peak_pitch_deg"],
            "failure": r["failure"],
        }
    nominal = [n for n in MOVES if n in verdicts]
    out = {
        "nominal_passed": sum(bool(verdicts[n]["passed"]) for n in nominal),
        "nominal": len(nominal),
        "variants_passed": sum(bool(v["passed"]) for n, v in verdicts.items() if n not in MOVES),
        "variants": len(verdicts) - len(nominal),
        "falls": sum(bool(v["failure"]) for v in verdicts.values()),
        "cases": verdicts,
    }
    (output / "moves.json").write_text(json.dumps(out, indent=1, default=float) + "\n")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--checkpoint", type=Path, default=CHECKPOINT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--case", action="append", default=[])
    a = parser.parse_args()
    output = a.output or ROOT / f"local/m15/moves_{a.checkpoint.resolve().parent.name}"
    from ..m12 import speed
    from ..m13 import full_plant

    speed.definitions = definitions
    argv = ["speed", "--checkpoint", str(a.checkpoint), "--output", str(output)]
    argv += ["--workers", str(a.workers), *[x for n in a.case for x in ("--case", n)]]
    sys.argv = ["bipe.m13.full_plant", *argv]
    full_plant.main()
    print(json.dumps({k: v for k, v in judge(output).items() if k != "cases"}))


if __name__ == "__main__":
    main()
