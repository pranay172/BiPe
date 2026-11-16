"""M16 phase A: rerun a policy's at-margin acceptance verdicts with perturbations.

    python -m bipe.m16.margin run SCORES CHECKPOINT [--seeds 1,2,3] [--workers 4] [--output DIR]
    python -m bipe.m16.margin score DIR

``SCORES`` is a ``bipe.m14.evaluate`` scores.json; its ``at_margin`` cases (recovery
verdicts within MARGIN of the band edge) are rerun on the full plant once per seed with
seeded observation noise of 1e-5 (``bipe.m13.full_plant --perturb``), each seed into
DIR/perturb_<seed>. ``score`` writes DIR/margin.json and prints the gate 2 counts with
each at-margin case decided by the majority of its perturbed reruns (the single
nominal verdict stands for every other case).

Evidence: a validated simulation under the M10 plant's assumptions, not hardware.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from ..m14.evaluate import GATES, acceptance_cases, case_verdict

ROOT = Path(__file__).resolve().parents[2]


def run(args):
    scores = json.loads(Path(args.scores).read_text())["scores"]
    cases = scores["acceptance"]["at_margin"]
    out = Path(args.output or ROOT / f"local/m16/margin_{scores['name']}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "source.json").write_text(json.dumps({
        "scores": str(Path(args.scores)), "checkpoint": args.checkpoint, "cases": cases,
        "nominal": {n: scores["acceptance"]["verdicts"][n] for n in cases},
        "base_verdicts": scores["acceptance"]["verdicts"],
        "push_cases": [n for n in scores["acceptance"]["verdicts"] if n.startswith("push_")],
        "belt_cases": [n for n in scores["acceptance"]["verdicts"] if n.startswith("belt_")],
    }, indent=2) + "\n")  # fmt: skip
    for seed in (int(s) for s in args.seeds.split(",")):
        cmd = [sys.executable, "-m", "bipe.m13.full_plant", "acceptance",
               "--checkpoint", args.checkpoint, "--perturb", str(seed),
               "--workers", str(args.workers), "--output", str(out / f"perturb_{seed}")]
        cmd += [x for c in cases for x in ("--case", c)]  # fmt: skip
        if (out / f"perturb_{seed}" / "summary.json").exists():
            cmd.append("--resume")  # the acceptance runner keeps the cases it saved
        print("M16 MARGIN", " ".join(cmd), flush=True)
        subprocess.run(cmd, check=True, cwd=ROOT)
    score(out)


def score(out):
    out = Path(out)
    src = json.loads((out / "source.json").read_text())
    reruns, falls = {}, []
    for d in sorted(out.glob("perturb_*")):
        for name, row in acceptance_cases([d]).items():
            survived, recovered, _ = case_verdict(row)
            reruns.setdefault(name, []).append(recovered)
            if not survived:
                falls.append(f"{name} ({d.name})")
    verdicts = dict(src["base_verdicts"])
    decided = {}
    for name in src["cases"]:
        runs = reruns.get(name, [])
        if runs:
            verdicts[name] = sum(runs) * 2 > len(runs)
        decided[name] = {"nominal": src["nominal"][name], "perturbed": runs,
                         "verdict": verdicts[name]}  # fmt: skip
    counts = {
        "recovered_98": sum(verdicts.values()),
        "pushes_recovered": sum(verdicts[n] for n in src["push_cases"]),
        "belt_recovered": sum(verdicts[n] for n in src["belt_cases"]),
    }
    flips = sorted(n for n, d in decided.items() if d["verdict"] != d["nominal"])
    counts["perturbed_falls"] = falls
    gate2 = (counts["recovered_98"] >= GATES["recovered_98"]
             and counts["pushes_recovered"] >= GATES["pushes_recovered"]
             and counts["belt_recovered"] >= GATES["belt_recovered"] and not falls)  # fmt: skip
    result = {"counts": counts, "gate_2_recovery": bool(gate2), "flipped": flips,
              "cases": decided}  # fmt: skip
    (out / "margin.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("counts", "gate_2_recovery", "flipped")}, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run")
    r.add_argument("scores")
    r.add_argument("checkpoint")
    r.add_argument("--seeds", default="1,2,3")
    r.add_argument("--workers", type=int, default=4)
    r.add_argument("--output")
    s = sub.add_parser("score")
    s.add_argument("dir")
    args = parser.parse_args()
    run(args) if args.command == "run" else score(args.dir)


if __name__ == "__main__":
    main()
