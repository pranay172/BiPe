"""M13 phase C: the policy on the full plant's M10 acceptance and M12 speed cases.

    python -m bipe.m13.full_plant acceptance [--case NAME ...] [--workers 4]
    python -m bipe.m13.full_plant speed [--case NAME ...] [--workers 4]
    python -m bipe.m13.full_plant gap [--case NAME ...] [--workers 4]
    ... [--forward c]  (phase D: the firmware forward pass in place of the numpy actor)
    ... [--leg-filter 0.03]  (leg target filter time constant, s; default 0, off)
    ... [--perturb SEED [--perturb-scale 1e-5]]  (M16: seeded noise on every observation)
    ... [--shaper acceleration=A,jerk=J,yaw_acceleration=Y]  (M16 command shaper)

Runs the classical runners (bipe.robustness for the M10 profile, bipe.m12.speed)
with the policy in the loop (bipe.m13.policy_control), so cases, timing, sensors,
supervisor and pass criteria are theirs. One criterion is built around the classical
controller's structure: a joint may not stray 5 degrees from the commanded posture,
where the policy places its legs itself to balance. Each case also records whether
it passes with that criterion left out (``passed_without_joint_error``).
Results go to artifacts/m13/phase_c/<run>_<suite> (with ``--forward c``,
artifacts/m13/phase_d/<run>_c_<suite>), <run> being the checkpoint's training run,
beside a policy.json naming the checkpoint, its hash and the forward pass. A
directory holding another policy's results is refused. The classical results to
compare with are rerun beside them (artifacts/m13/phase_c/*_classical).
Every case records ``policy_steps``, and a case without policy steps (other than
the ideal-sensing cases the policy rejects) stops the run with an error.

``--checkpoint`` defaults to M13's settle, whose phase C records these are; the
deployed policy (bipe.m13.headers.CHECKPOINT, M14's fine-tune 1 since 3 November 2026)
needs ``--checkpoint``.

``gap`` is the sim-to-sim gap: the same cases with the policy on the training plant
(bipe.m13.validate, which phase A ran with the classical controller), compared with
the policy's full-plant results in the bands phase A used.
"""

import argparse
import copy
import hashlib
import json
import multiprocessing
import sys
from pathlib import Path

from . import policy_control
from .policy_control import CActor, policy_in_loop

ROOT = Path(__file__).resolve().parents[2]
CHECKPOINT = ROOT / "artifacts/m13/training/settle/model_10493.pt"


def with_joint_exemption(metrics):
    """metrics(run), adding the verdict without the joint error criterion."""

    def wrapped(run):
        report = metrics(run)
        report["policy_steps"] = run.get("policy_steps")
        if "passed" in report and run["stats"]["max_joint_error_deg"] >= 5:
            exempt = copy.copy(run)
            exempt["stats"] = {**run["stats"], "max_joint_error_deg": 0.0}
            report["passed_without_joint_error"] = metrics(exempt)["passed"]
        elif "passed" in report:
            report["passed_without_joint_error"] = report["passed"]
        return report

    return wrapped


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("suite", choices=["acceptance", "speed", "gap"])
    parser.add_argument("--checkpoint", type=Path, default=CHECKPOINT)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--forward", choices=["numpy", "c"], default="numpy")
    parser.add_argument("--resume", action="store_true", help="acceptance: continue saved cases")
    parser.add_argument(
        "--reference", type=Path, help="gap: the policy's full-plant results, <dir>/<run>_ prefix"
    )
    parser.add_argument("--hip-belt", choices=["rigid", "spring"], help="gap: training plant (M14)")
    parser.add_argument("--knee-lash", choices=["static", "deadband"], help="gap: training plant")
    parser.add_argument(
        "--leg-filter",
        type=float,
        default=policy_control.LEG_FILTER_TAU_S,
        help="leg target filter time constant, s (0: off, as phases C and D ran)",
    )
    parser.add_argument(
        "--perturb", type=int, help="seed of a tiny observation perturbation (margin reruns)"
    )
    parser.add_argument("--perturb-scale", type=float, default=1e-5)
    parser.add_argument(
        "--shaper", help="M16 command shaper, e.g. acceleration=1.0,jerk=4,yaw_acceleration=3"
    )
    args = parser.parse_args()
    policy_control.LEG_FILTER_TAU_S = args.leg_filter
    shaper = None
    if args.shaper:
        shaper = {k: float(v) for k, v in (x.split("=", 1) for x in args.shaper.split(","))}
        policy_control.SHAPER = shaper
    run = args.checkpoint.resolve().parent.name
    default = (
        ("phase_c", f"{run}_{args.suite}")
        if args.forward == "numpy"
        else ("phase_d", f"{run}_c_{args.suite}")
    )
    output = args.output or ROOT / "artifacts/m13" / default[0] / default[1]
    output.mkdir(parents=True, exist_ok=True)
    cases = [x for name in args.case for x in ("--case", name)]
    policy = {
        **({"hip_belt": args.hip_belt} if args.hip_belt else {}),
        **({"knee_lash": args.knee_lash} if args.knee_lash else {}),
        "checkpoint": str(args.checkpoint.resolve().relative_to(ROOT)),
        "checkpoint_sha256": hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        "suite": args.suite,
        "forward": args.forward,
        "leg_filter_tau_s": args.leg_filter,
        "leg_filter_band_rad": policy_control.LEG_FILTER_BAND_RAD,
        **({"perturb": args.perturb, "perturb_scale": args.perturb_scale}
           if args.perturb is not None else {}),
        **({"shaper": shaper} if shaper else {}),
    }  # fmt: skip
    if (output / "policy.json").exists():
        saved = {
            "leg_filter_tau_s": 0.0,
            "leg_filter_band_rad": policy_control.LEG_FILTER_BAND_RAD,
            **json.loads((output / "policy.json").read_text()),
        }
        if {k: saved.get(k) for k in policy} != policy:
            parser.error(
                f"{output} holds results of another policy or forward pass; use another --output"
            )
    (output / "policy.json").write_text(
        json.dumps({**policy, "cases": args.case or "all"}, indent=2) + "\n"
    )
    if args.suite == "acceptance":
        from .. import robustness as runner

        argv = ["--profile", "m10", "--controllers", "current", "--phase", "all"]
        argv += ["--resume"] if args.resume else []
    elif args.suite == "speed":
        from ..m12 import speed as runner

        argv = []
    else:
        from . import validate as runner

        # The policy's full-plant results beside the output: <prefix>gap ->
        # <prefix>acceptance and <prefix>speed.
        prefix = output.name.removesuffix("gap")
        full = output.parent
        if args.reference:
            full, prefix = args.reference.parent, args.reference.name
        runner.ACCEPTANCE = full / f"{prefix}acceptance/summary.json"
        runner.SPEED = full / f"{prefix}speed"
        argv = []
        for flag, value in (("--hip-belt", args.hip_belt), ("--knee-lash", args.knee_lash)):
            argv += [flag, value] if value else []
    sys.argv = [runner.__name__, *argv, "--output", str(output), "--workers", str(args.workers)]
    sys.argv += cases
    runner.metrics = with_joint_exemption(runner.metrics)
    runner.physical_metrics = with_policy_steps(runner.physical_metrics)
    actor = None
    if args.forward == "c":
        actor = CActor()
        sha = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
        assert actor.checkpoint_sha256 == sha, "liblocomotion.so was built from another checkpoint"
    if args.perturb is not None:
        from .policy_control import Actor, Perturbed

        actor = Perturbed(actor or Actor(args.checkpoint), args.perturb, args.perturb_scale)
    multiprocessing.set_start_method("fork", force=True)  # see policy_in_loop
    with policy_in_loop(args.checkpoint, runners=(runner.__name__,), actor=actor):
        runner.main()
    check_policy_ran(args.suite, output)


def with_policy_steps(physical_metrics):
    def wrapped(run):
        return {**physical_metrics(run), "policy_steps": run.get("policy_steps")}

    return wrapped


def check_policy_ran(suite, output):
    """Every case the policy did not reject made policy steps."""
    if suite == "acceptance":
        rows = json.loads((output / "summary.json").read_text())["controllers"]["current"]["cases"]
        # The unpowered case runs with the controller disabled, so no policy step.
        steps = {
            name: case["metrics"]
            for name, case in rows.items()
            if case["case"].get("job", {}).get("enabled", True)
        }
    elif suite == "speed":
        rows = json.loads((output / "cases.json").read_text())
        steps = {
            name: {**case["physical"], "failure": case["failure"]} for name, case in rows.items()
        }
    else:
        return
    missing = [
        name
        for name, m in steps.items()
        if not m.get("policy_steps")
        and not str(m.get("failure") or "").startswith("controller rejected")
    ]
    if missing:
        raise RuntimeError(f"cases ran without the policy: {missing}")


if __name__ == "__main__":
    main()
