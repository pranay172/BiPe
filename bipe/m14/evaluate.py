"""M14: score a trained policy against the exit gates and the M13 `settle` baseline.

    python -m bipe.m14.evaluate run CHECKPOINT [--output DIR] [--stages S,S,...] [--workers 4]
    python -m bipe.m14.evaluate score DIR [DIR ...] [--baseline artifacts/m14/eval/settle]

``run`` takes a checkpoint inside the repository (copy a laptop run's model_N.pt under
local/ first) and runs these stages, each a subprocess of an existing runner, writing
into DIR (default local/m14/eval/<run>_<checkpoint stem>):

- ``hardware``: the firmware runtime with the M11 image's timing (``bipe.m13.handover
  --variant hardware``: action one 5 ms tick late, wheel loop split) on the 8 belt-range
  cases and the 10 PR #5 handover cases (17 cases, about 25 min on 4 CPUs). The quick
  triage: settle recovers 8/8 belt and 8/10 handover cases here.
- ``stress``: the 26-case full-plant stress set (``bipe.m14.stress``), gate 5's spot check.
- ``acceptance``: the 107 M10 acceptance cases on the full plant
  (``bipe.m13.full_plant acceptance``), gate 2. Resumable.
- ``speed``: the 44 M12 speed cases (``bipe.m13.full_plant speed``), gate 3, recording the
  joint-exempt verdict and the axle depth error the proposed policy rule needs.
- ``training``: the training-plant stress suite (``bipe.m13.rl.evaluate --stress``), gate
  5's 5 % fall limit; ``--device`` (default cpu, 64 episodes as settle's reference ran).

A stage whose result file exists is skipped (``--force`` reruns it). Stages run in the
order given; the default order puts the quick, decisive ones first. After the run (and
on ``score``) DIR gets scores.json and report.md: each gate against its pass mark and
against settle, the cases gained and lost against settle, and the verdicts that sit
within a hair of the recovery band (single runs there are close to chaotic, PR #3
review). ``score`` with several DIRs (two seeds) also prints them side by side.

The baseline, artifacts/m14/eval/settle/eval.json, points at settle's committed records
(phase C acceptance, the M14 stress set, the PR #5 and belt hardware runs, the CPU
training stress run) and a speed rerun with the policy rule's metrics.

Counting (the PR #3 review's "98 the policy can run"): the 107 acceptance cases less the
8 ideal-sensing cases the policy rejects and ``unpowered``. The 79 matrix cases (push,
backlash, belt, mismatch, slope, numerical) are judged by the recovery rule; the 19
scenario cases by their own pass rule without the joint-error criterion (classical:
with it, as its records hold). The M13 doc's 80/98 for settle is not reproduced by this
rule from the committed records (76 recovered, 81 survived); 83/98 for the classical
controller is.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / "artifacts/m14/eval/settle"
STAGES = ("hardware", "stress", "acceptance", "speed", "training")
BELT = [
    f"belt_hold_h{h}_n{n}_k{k}" for h in ("0.1125", "0.45") for n in ("0.225", "0.9")
    for k in (0, 1)
]  # fmt: skip
HANDOVER = [
    "push_hold_+2", "push_hold_-2", "push_drive_+2.5", "push_low_-2.5", "sensors_combined",
    "backlash_turn_0.6", "belt_hold_h0.45_n0.9_k1", "slope_up_8", "delay_15ms_noise_x2_drive",
    "wheel_lag_25ms_drive",
]  # fmt: skip
HARDWARE_CASES = list(dict.fromkeys(BELT + HANDOVER))
# Full coverage of each stage: a gate is judged only on complete stages.
EXPECTED = {"acceptance": 98, "belt": 8, "push": 30, "speed": 44, "stress": 26,
            "hardware_belt": 8, "hardware_handover": 10}  # fmt: skip
# Gate pass marks (docs/milestones/14-release-grade-locomotion-plan.md).
GATES = {
    "recovered_98": 83,
    "belt_recovered": 6,
    "pushes_recovered": 30,
    "physical_failures": 0,
    "speed_policy_rule": 40,
    "training_fall_rate": 0.05,
    "stress_physical_failures": 0,
}
# A recovery verdict within this of its band edge (last 2 s: peak speed 0.03 m/s, peak
# pitch error 2 deg) is "at the margin".
MARGIN = {"speed_m_s": 0.005, "pitch_deg": 0.3}
# Gate 3's axle height check, (window, p95 limit mm): "steady" is the case's speed
# plateaus (cases without plateaus get no axle check), as the training metric measures
# steady segments; "case" is the whole run. User decision (1 November 2026): steady,
# 5 mm (the drafted whole-case 5 mm rule fails settle's start and stop dips, 4 of 43).
AXLE_RULE = ("steady", 5.0)


# --------------------------------------------------------------------------- run


def result_file(stage, out):
    return {
        "hardware": out / "hardware/summary.json",
        "stress": out / "stress/summary.json",
        "acceptance": out / "acceptance/summary.json",
        "speed": out / "speed/cases.json",
        "training": out / "training_stress.json",
    }[stage]


def stage_command(stage, checkpoint, out, args):
    py = [sys.executable, "-m"]
    workers = ["--workers", str(args.workers)]
    if stage == "hardware":
        cases = [x for c in HARDWARE_CASES for x in ("--case", c)]
        return py + ["bipe.m13.handover", "acceptance", "--variant", "hardware",
                     "--checkpoint", str(checkpoint), "--output", str(out / "hardware"),
                     *workers, *cases]  # fmt: skip
    if stage == "stress":
        return py + ["bipe.m14.stress", "--checkpoint", str(checkpoint),
                     "--output", str(out / "stress"), *workers]  # fmt: skip
    if stage in ("acceptance", "speed"):
        resume = ["--resume"] if stage == "acceptance" and (out / stage).exists() else []
        return py + ["bipe.m13.full_plant", stage, "--checkpoint", str(checkpoint),
                     "--output", str(out / stage), *workers, *resume]  # fmt: skip
    return py + ["bipe.m13.rl.evaluate", str(checkpoint), "--stress", "--device", args.device,
                 "--worlds", str(args.worlds), "--episodes", str(args.episodes),
                 "--episode-s", str(args.episode_s),
                 *(["--seed", str(args.seed)] if args.seed is not None else [])]  # fmt: skip


def git_state():
    """(HEAD commit, whether the code and configuration differ from it)."""
    run = lambda *a: subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True,
                                    check=False).stdout.strip()  # fmt: skip
    return run("rev-parse", "HEAD"), bool(
        run(
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--",
            "bipe",
            "config",
            "firmware",
            "tools",
        )
    )


def environment():
    """The versions that decide a closed-loop trajectory: single runs at the recovery band
    diverge between environments (PR #7 and #8 reviews), so a seed is compared with a
    baseline from the same environment."""
    import platform
    from importlib.metadata import PackageNotFoundError, version

    packages = ("numpy", "scipy", "mujoco", "torch", "mjlab", "rsl-rl-lib")
    out = {"python": platform.python_version(), "machine": platform.machine(),
           "cpus": os.cpu_count(), "node": platform.node()}  # fmt: skip
    for name in packages:
        try:
            out[name] = version(name)
        except PackageNotFoundError:
            out[name] = None
    return out


def run(args):
    checkpoint = args.checkpoint.resolve()
    if not checkpoint.is_relative_to(ROOT):
        sys.exit(f"{checkpoint} is outside the repository; copy it under local/ first")
    rel = checkpoint.relative_to(ROOT)
    out = args.output or ROOT / "local/m14/eval" / f"{checkpoint.parent.name}_{checkpoint.stem}"
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    sha = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    record_path = out / "eval.json"
    record = json.loads(record_path.read_text()) if record_path.exists() else {}
    if record and record.get("checkpoint_sha256") != sha:
        sys.exit(f"{out} holds another checkpoint's evaluation; use another --output")
    record.update(checkpoint=str(rel), checkpoint_sha256=sha)
    record.pop("commit", None)  # per stage, below
    record.setdefault("stages", {})
    stages = args.stages.split(",") if args.stages else list(STAGES)
    for stage in stages:
        if stage not in STAGES:
            sys.exit(f"unknown stage {stage}; choose from {', '.join(STAGES)}")
    for stage in stages:
        done = result_file(stage, out).exists()
        if done and not args.force and stage != "acceptance":
            print(f"[{stage}] done, skipped", flush=True)
            continue
        if stage == "acceptance" and done and not args.force and complete_acceptance(out):
            print(f"[{stage}] done, skipped", flush=True)
            continue
        if args.force and stage != "training":
            shutil.rmtree(out / stage, ignore_errors=True)
        cmd = stage_command(stage, checkpoint, out, args)
        print(f"[{stage}] {' '.join(cmd[1:])}", flush=True)
        start = datetime.now(UTC)
        log = out / f"{stage}.log"
        with log.open("a") as f:
            code = subprocess.call(cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT,
                                   env=child_env())  # fmt: skip
        minutes = (datetime.now(UTC) - start).total_seconds() / 60
        written = checkpoint.with_name("evaluation_stress.json")
        if stage == "training" and code == 0:
            shutil.move(written, result_file(stage, out))  # not left beside the checkpoint
        commit, dirty = git_state()
        training = {"device": args.device, "worlds": args.worlds, "episodes": args.episodes,
                    "episode_s": args.episode_s, "seed": args.seed}  # fmt: skip
        record["stages"][stage] = {
            "exit": code, "minutes": round(minutes, 1), "log": log.name,
            "finished": datetime.now(UTC).isoformat(timespec="seconds"),
            "commit": commit, "dirty": dirty, "environment": environment(),
            **(training if stage == "training" else {}),
        }  # fmt: skip
        record_path.write_text(json.dumps(record, indent=2) + "\n")
        print(f"[{stage}] exit {code} after {minutes:.0f} min (log {log})", flush=True)
        if code != 0:
            print(f"[{stage}] failed; see {log}", flush=True)
    record_path.write_text(json.dumps(record, indent=2) + "\n")
    report = score_dir(out, args.baseline)
    print(report)


def child_env():
    env = dict(os.environ)
    env.setdefault("MUJOCO_GL", "disable")
    env["PYTHONPATH"] = str(ROOT) + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    return env


def complete_acceptance(out):
    summary = json.loads(result_file("acceptance", out).read_text())
    return len(summary["controllers"]["current"]["cases"]) >= 107


# ------------------------------------------------------------------------- score


def sources(directory):
    """Each stage's records for an eval directory: its own subdirectories, or (the
    settle baseline) the committed records its eval.json names."""
    directory = Path(directory)
    record = json.loads((directory / "eval.json").read_text())
    named = record.get("sources", {})
    out = {}
    for stage in STAGES:
        if stage in named:
            paths = named[stage] if isinstance(named[stage], list) else [named[stage]]
            out[stage] = [ROOT / x for x in paths]
        else:
            path = result_file(stage, directory)
            out[stage] = [path.parent if stage != "training" else path]
    return record, out


def acceptance_cases(dirs):
    cases = {}
    for d in dirs:
        path = d / "summary.json"
        if path.exists():
            cases.update(json.loads(path.read_text())["controllers"]["current"]["cases"])
    return cases


def case_verdict(row):
    """(survived, recovered, at margin) of one acceptance case."""
    m = row["metrics"]
    survived = m.get("physical_pass", m.get("failure") is None) and m.get("failure") is None
    if "recovered" in m:
        recovered = bool(m["recovered"])
        speed, pitch = m.get("late_speed_peak_m_s"), m.get("late_pitch_peak_deg")
        margin = survived and (
            (speed is not None and abs(speed - 0.03) <= MARGIN["speed_m_s"])
            or (pitch is not None and abs(pitch - 2.0) <= MARGIN["pitch_deg"])
        )
    else:
        recovered = bool(m.get("passed_without_joint_error", m.get("passed")))
        margin = False
    return bool(survived), recovered and bool(survived), bool(margin)


def score_acceptance(cases):
    policy = {n: r for n, r in cases.items() if not n.startswith("ideal_") and n != "unpowered"}
    v = {n: case_verdict(r) for n, r in policy.items()}
    family = lambda f: [n for n, r in policy.items() if r["case"].get("family") == f]
    matrix = [n for n, r in policy.items() if r["case"].get("family") != "acceptance"]
    return {
        "cases": len(policy),
        "complete": len(policy) >= EXPECTED["acceptance"]
        and len(family("belt")) >= EXPECTED["belt"]
        and len(family("push")) >= EXPECTED["push"],
        "recovered_98": sum(x[1] for x in v.values()),
        "survived_98": sum(x[0] for x in v.values()),
        "recovered_79": sum(v[n][1] for n in matrix),
        "matrix_cases": len(matrix),
        "belt_recovered": sum(v[n][1] for n in family("belt")),
        "belt_cases": len(family("belt")),
        "pushes_survived": sum(v[n][0] for n in family("push")),
        "pushes_recovered": sum(v[n][1] for n in family("push")),
        "push_cases": len(family("push")),
        "physical_failures": sorted(n for n, x in v.items() if not x[0]),
        "not_recovered": sorted(n for n, x in v.items() if not x[1]),
        "at_margin": sorted(n for n, x in v.items() if x[2]),
        "verdicts": {n: x[1] for n, x in v.items()},
    }


def speed_rule(result, criteria, axle_rule=None):
    """The proposed policy rule (phase D, signed off 1 November 2026): the speed suite's checks
    with the joint-error criterion dropped and the axle height check (AXLE_RULE)."""
    from ..m12.speed import assess

    r = copy.deepcopy(result)
    exempt = r.get("general_passed_without_joint_error")
    if exempt is None:
        return None
    r["general_passed"] = bool(exempt)
    verdict = assess(r, r["case"], criteria)
    if verdict["passed"] is None:
        return verdict
    window, limit = axle_rule or AXLE_RULE
    e = r["axle_depth_error"]
    value = e.get("steady_p95_mm") if window == "steady" else e["p95_mm"]
    axle = value is None or value <= limit
    verdict["checks"]["axle_depth_p95"] = axle
    verdict["passed"] = verdict["passed"] and axle
    return verdict


def score_speed(dirs):
    path = dirs[0] / "cases.json"
    if not path.exists():
        return None
    rows = json.loads(path.read_text())
    criteria = json.loads((ROOT / "config/speed-m12.json").read_text())["criteria"]
    judged = {n: r for n, r in rows.items() if r["assessment"]["passed"] is not None}
    policy = {n: speed_rule(r, criteria) for n, r in judged.items()}
    has_rule = all(p is not None for p in policy.values())
    failing = {}
    if has_rule:
        for n, p in policy.items():
            if not p["passed"]:
                failing[n] = sorted(k for k, ok in p["checks"].items() if not ok)
    return {
        "cases": len(rows),
        "judged": len(judged),
        "complete": len(rows) >= EXPECTED["speed"],
        "survived_recovered": sum(
            r["physical"]["physical_pass"] and bool(r["physical"]["recovered"])
            for r in rows.values()
        ),
        "classical_rule": sum(bool(r["assessment"]["passed"]) for r in judged.values()),
        "policy_rule": sum(p["passed"] for p in policy.values()) if has_rule else None,
        "policy_rule_variants": {
            f"{w} {m:g} mm": sum(speed_rule(r, criteria, (w, m))["passed"] for r in judged.values())
            for w, m in (("case", 10.0), ("case", 5.0))
        }
        if has_rule
        else None,
        "policy_rule_failing": failing if has_rule else None,
        "axle_steady_p95_mm_worst": max(
            (
                (r["axle_depth_error"]["steady_p95_mm"], n)
                for n, r in judged.items()
                if (r.get("axle_depth_error") or {}).get("steady_p95_mm") is not None
            ),
            default=None,
        ),
        "axle_case_p95_mm_worst": max(
            (
                (r["axle_depth_error"]["p95_mm"], n)
                for n, r in judged.items()
                if "axle_depth_error" in r
            ),
            default=None,
        ),
        "axle_unchecked": sorted(
            n
            for n, r in judged.items()
            if (r.get("axle_depth_error") or {}).get("steady_p95_mm") is None
        ),
        "physical_failures": sorted(
            n for n, r in rows.items() if not r["physical"]["physical_pass"]
        ),
        "verdicts": {n: (p["passed"] if p else None) for n, p in policy.items()},
    }


def score_stress(dirs):
    path = dirs[0] / "summary.json"
    if not path.exists():
        return None
    s = json.loads(path.read_text())
    return {
        "cases": s["cases"],
        "complete": s["cases"] >= EXPECTED["stress"],
        "survived": s["cases"] - s["falls"],
        "recovered": s.get("recovered"),
        "physical_failures": s["fallen"],
    }


def score_hardware(dirs):
    cases = acceptance_cases(dirs)
    if not cases:
        return None
    v = {n: case_verdict(r) for n, r in cases.items()}
    belt = [n for n in BELT if n in v]
    handover = [n for n in HANDOVER if n in v]
    return {
        "complete": len(belt) >= EXPECTED["hardware_belt"]
        and len(handover) >= EXPECTED["hardware_handover"],
        "belt_recovered": sum(v[n][1] for n in belt),
        "belt_cases": len(belt),
        "handover_recovered": sum(v[n][1] for n in handover),
        "handover_cases": len(handover),
        "physical_failures": sorted(n for n in v if not v[n][0]),
        "at_margin": sorted(n for n in v if v[n][2]),
        "verdicts": {n: v[n][1] for n in v},
    }


def wilson(k, n, z=1.96):
    """95 % interval of a fall rate from k falls in n episodes."""
    if not n:
        return None
    p, d = k / n, 1 + z * z / n
    centre, half = (p + z * z / (2 * n)) / d, z * (p * (1 - p) / n + z * z / (4 * n * n)) ** 0.5 / d
    return [max(0.0, centre - half), min(1.0, centre + half)]


def score_training(dirs, setting=None):
    path = dirs[0]
    if not path.exists():
        return None
    s = json.loads(path.read_text())
    return {
        "episodes": s["episodes"],
        "falls": s["falls"],
        "fall_rate": s["fall_rate"],
        "interval_95": wilson(s["falls"], s["episodes"]),
        "episode_s": (setting or {}).get("episode_s"),
        "complete": True,
        "causes": s.get("causes"),
    }


def score(directory):
    record, src = sources(directory)
    cases = acceptance_cases(src["acceptance"])
    stages = record.get("stages", {})
    setting = stages.get("training") or record.get("training_setting")
    out = {
        "name": Path(directory).name,
        "checkpoint": record.get("checkpoint"),
        "checkpoint_sha256": record.get("checkpoint_sha256"),
        "acceptance": score_acceptance(cases) if cases else None,
        "speed": score_speed(src["speed"]),
        "stress": score_stress(src["stress"]),
        "hardware": score_hardware(src["hardware"]),
        "training": score_training(src["training"], setting),
    }
    # A stage that exited non-zero is incomplete, whatever its records hold.
    for stage, info in stages.items():
        if out.get(stage) and info.get("exit") not in (0, None):
            out[stage]["complete"] = False
    out["stages"] = {
        k: {"exit": v.get("exit"), "commit": v.get("commit"), "dirty": v.get("dirty")}
        for k, v in stages.items()
    }
    out["gates"] = gates(out)
    return out


def gates(s):
    """Each gate: value, pass mark, and passed (None: incomplete, its stages have not
    covered their full case set or a stage failed)."""
    a, sp, st, tr = s["acceptance"], s["speed"], s["stress"], s["training"]
    done = lambda x: bool(x and x.get("complete"))
    g = {}
    if a:
        g["2_recovered_98"] = (a["recovered_98"], GATES["recovered_98"], done(a))
        g["2_belt"] = (a["belt_recovered"], GATES["belt_recovered"], done(a))
        g["2_pushes"] = (a["pushes_recovered"], GATES["pushes_recovered"], done(a))
        phys = len(a["physical_failures"]) + (len(sp["physical_failures"]) if sp else 0)
        g["2_physical_failures"] = (phys, GATES["physical_failures"], done(a) and done(sp))
    if sp and sp["policy_rule"] is not None:
        g["3_speed_policy_rule"] = (sp["policy_rule"], GATES["speed_policy_rule"], done(sp))
    if tr:
        g["5_training_fall_rate"] = (tr["fall_rate"], GATES["training_fall_rate"], done(tr))
    if st:
        g["5_stress_physical_failures"] = (len(st["physical_failures"]), 0, done(st))
    lower = {"2_physical_failures", "5_training_fall_rate", "5_stress_physical_failures"}
    return {
        k: {
            "value": v,
            "mark": m,
            "complete": c,
            "passed": (v <= m if k in lower else v >= m) if c else None,
        }
        for k, (v, m, c) in g.items()
    }


# ------------------------------------------------------------------------ report


def fmt(value, of=None):
    if value is None:
        return "not run"
    if isinstance(value, float):
        return f"{value:.1%}" if value < 1 else f"{value:.2f}"
    return f"{value}/{of}" if of is not None else str(value)


def rows(s):
    a, sp, st, hw, tr = (s[k] for k in ("acceptance", "speed", "stress", "hardware", "training"))
    return {
        "Gate 2: recovered, 98 policy cases (>= 83)": fmt(a and a["recovered_98"], a and a["cases"]),
        "Gate 2: belt range recovered (>= 6/8)": fmt(a and a["belt_recovered"], a and a["belt_cases"]),
        "Gate 2: pushes recovered (30/30)": fmt(a and a["pushes_recovered"], a and a["push_cases"]),
        "Gate 2: physical failures, acceptance + speed (0)": fmt(
            None if not a else len(a["physical_failures"]) + (len(sp["physical_failures"]) if sp else 0)
        ) + ("" if sp else " (speed not run)"),
        "Gate 3: speed policy rule (>= 40 of 43 judged)": fmt(sp and sp["policy_rule"], sp and sp["judged"]),
        "Gate 3: axle worst steady p95, mm": fmt(sp and sp["axle_steady_p95_mm_worst"] and round(sp["axle_steady_p95_mm_worst"][0], 1)),
        "Gate 3: judged cases without a steady segment (no axle check)": fmt(sp and sp["axle_unchecked"] is not None and len(sp["axle_unchecked"])),
        **{
            f"Speed rule with axle check {k}": fmt(n, sp["judged"])
            for k, n in ((sp or {}).get("policy_rule_variants") or {}).items()
        },
        "Gate 5: training stress falls (<= 5 %)": "not run" if not tr else (
            f"{tr['fall_rate']:.1%} ({tr['falls']}/{tr['episodes']} x {tr['episode_s'] or '?'} s; "
            f"95 % {tr['interval_95'][0]:.1%}-{tr['interval_95'][1]:.1%})"
        ),
        "Gate 5: full-plant stress survived (26/26)": fmt(st and st["survived"], st and st["cases"]),
        "Survived, 98 policy cases": fmt(a and a["survived_98"], a and a["cases"]),
        "Recovered, 79 matrix cases (M13 headline)": fmt(a and a["recovered_79"], a and a["matrix_cases"]),
        "Speed: survived and recovered": fmt(sp and sp["survived_recovered"], sp and sp["cases"]),
        "Speed: classical joint-error rule": fmt(sp and sp["classical_rule"], sp and sp["judged"]),
        "Hardware timing: belt recovered": fmt(hw and hw["belt_recovered"], hw and hw["belt_cases"]),
        "Hardware timing: PR #5 handover cases recovered": fmt(
            hw and hw["handover_recovered"], hw and hw["handover_cases"]
        ),
        "Hardware timing: physical failures": fmt(hw and len(hw["physical_failures"])),
        "Acceptance verdicts at the band margin": fmt(a and len(a["at_margin"])),
        **{
            f"Stage {k}": ("complete" if (s.get(k) or {}).get("complete") else "INCOMPLETE")
            + f" (exit {v['exit']}, {str(v['commit'])[:8]}{' dirty' if v.get('dirty') else ''})"
            for k, v in (s.get("stages") or {}).items()
        },
    }  # fmt: skip


def changes(new, old, key):
    if not (new.get(key) and old.get(key)):
        return None
    a, b = new[key]["verdicts"], old[key]["verdicts"]
    common = set(a) & set(b)
    return {
        "gained": sorted(n for n in common if a[n] and not b[n]),
        "lost": sorted(n for n in common if b[n] and not a[n]),
    }


def report(scores, baseline):
    names = [s["name"] for s in scores] + ([f"{baseline['name']} (baseline)"] if baseline else [])
    table = [rows(s) for s in scores] + ([rows(baseline)] if baseline else [])
    lines = [
        "# M14 policy evaluation",
        "",
        *[
            f"- `{s['name']}`: `{s['checkpoint']}` (sha256 {str(s['checkpoint_sha256'])[:12]})"
            for s in scores
        ],
        "",
        "| | " + " | ".join(names) + " |",
        "|---" * (len(names) + 1) + "|",
    ]
    labels = list(dict.fromkeys(label for t in table for label in t))
    for label in labels:
        lines.append(f"| {label} | " + " | ".join(t.get(label, "not run") for t in table) + " |")
    lines += [
        "",
        (
            "Gate 2 is judged on the acceptance suite at simulation timing, as the plan "
            "defines it; the hardware-timing rows (the M11 image's one-tick-late action) are "
            "reported beside it, not gated. Gate 3: the speed suite's checks without the "
            "joint-error criterion, plus axle height p95 within 5 mm over steady segments "
            "(command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two "
            "seeds, at most 6 h each) is read from the training runs. Verdicts at the band "
            "margin are close to chaotic in single runs; rerun them perturbed before calling "
            "gate 2 on a seed that lands within a few cases of 83."
        ),
    ]
    for s in scores:
        lines += ["", f"## `{s['name']}`", ""]
        passed = [k for k, g in s["gates"].items() if g["passed"]]
        failed = [k for k, g in s["gates"].items() if g["passed"] is False]
        partial = [k for k, g in s["gates"].items() if g["passed"] is None]
        lines.append(f"- Gates passed: {', '.join(passed) or 'none'}")
        lines.append(f"- Gates missed: {', '.join(failed) or 'none'}")
        lines.append(
            f"- Gates incomplete (stage not finished or failed): {', '.join(partial) or 'none'}"
        )
        for stage in ("acceptance", "speed", "hardware"):
            c = changes(s, baseline, stage) if baseline else None
            if c and (c["gained"] or c["lost"]):
                lines.append(f"- {stage} against settle: gained {', '.join(c['gained']) or 'none'}; "
                             f"lost {', '.join(c['lost']) or 'none'}")  # fmt: skip
        a = s["acceptance"]
        if a:
            lines.append(f"- Physical failures: {', '.join(a['physical_failures']) or 'none'}")
            lines.append(f"- Not recovered: {', '.join(a['not_recovered']) or 'none'}")
            lines.append(f"- At the band margin (rerun with perturbations before deciding on them): "
                         f"{', '.join(a['at_margin']) or 'none'}")  # fmt: skip
        sp = s["speed"]
        if sp and sp["policy_rule_failing"]:
            fail = "; ".join(f"{n} ({', '.join(c)})" for n, c in sp["policy_rule_failing"].items())
            lines.append(f"- Speed cases failing the policy rule: {fail}")
        if sp and sp["physical_failures"]:
            lines.append(f"- Speed physical failures: {', '.join(sp['physical_failures'])}")
        st = s["stress"]
        if st and st["physical_failures"]:
            lines.append(f"- Stress set falls: {', '.join(st['physical_failures'])}")
        hw = s["hardware"]
        if hw and hw["physical_failures"]:
            lines.append(f"- Hardware timing falls: {', '.join(hw['physical_failures'])}")
    return "\n".join(lines) + "\n"


def score_dir(directory, baseline_dir=BASELINE, others=()):
    scores = [score(directory), *(score(d) for d in others)]
    base = (
        score(baseline_dir)
        if baseline_dir and Path(baseline_dir).resolve() != Path(directory).resolve()
        else None
    )
    text = report(scores, base)
    strip = lambda s: {k: ({kk: vv for kk, vv in v.items() if kk != "verdicts"}
                           if isinstance(v, dict) and "verdicts" in v else v)
                       for k, v in s.items()}  # fmt: skip
    payload = {"scores": scores[0], "baseline": base and strip(base)}
    (Path(directory) / "scores.json").write_text(json.dumps(payload, indent=2) + "\n")
    (Path(directory) / "report.md").write_text(text)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="run the stages on a checkpoint, then score")
    r.add_argument("checkpoint", type=Path)
    r.add_argument("--output", type=Path)
    r.add_argument("--stages", help=f"comma-separated, from {','.join(STAGES)} (default all)")
    r.add_argument("--workers", type=int, default=4)
    r.add_argument("--device", default="cpu", help="training stage")
    r.add_argument("--worlds", type=int, default=64, help="training stage")
    r.add_argument("--episodes", type=int, default=64, help="training stage")
    r.add_argument("--seed", type=int, help="training stage: environment seed (default none: "
                   "torch's fixed default state, so repeats are not independent)")
    r.add_argument("--episode-s", type=float, default=10.0,
                   help="training stage (settle's 20 %% reference: 64 x 10 s)")  # fmt: skip
    r.add_argument("--force", action="store_true", help="rerun stages that have results")
    r.add_argument("--baseline", type=Path, default=BASELINE)
    r.add_argument("--no-baseline", action="store_true")
    s = sub.add_parser("score", help="score eval directories (the first gets the report)")
    s.add_argument("dirs", type=Path, nargs="+")
    s.add_argument("--baseline", type=Path, default=BASELINE)
    s.add_argument("--no-baseline", action="store_true")
    args = parser.parse_args()
    if args.no_baseline:
        args.baseline = None
    if args.command == "run":
        run(args)
    else:
        print(score_dir(args.dirs[0], args.baseline, args.dirs[1:]))


if __name__ == "__main__":
    main()
