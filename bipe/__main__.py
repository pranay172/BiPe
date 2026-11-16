"""BiPe command line.

    python -m bipe sim [--start sit|front_lean|back_lean] [--headless [--video out.mp4]]

``sim`` opens BiPe in an interactive MuJoCo viewer on a CPU, driven by the catalogue's
ONNX policies through the intent layer (bipe.sim; keys with ``sim --help``).

    python -m bipe train --task locomotion --seed N [--iterations 8000] [--num-envs 4096]
        [--log-root local/m14/runs] [extra mjlab train arguments]

``train --task locomotion`` is the M14 recipe: the mjlab task variant ``locomotion``
(bipe.m13.rl.env_cfg.VARIANTS) trained from scratch, one run per seed, named
``locomotion_s<seed>`` under ``<log-root>/bipe_m13``.

``train --task finetune --resume-run RUN [--resume-checkpoint model_7999.pt]`` continues a
run under ``<log-root>/bipe_m13`` on the fine-tune task (env_cfg ``locomotion_ft``) for
``--iterations`` more iterations. ``finetune2`` and ``finetune3`` (M15) are the later
fine-tune tasks (``locomotion_ft2``, ``locomotion_ft3``).

M16 names tasks by behaviour (config/behaviours.json): ``drive`` is ``locomotion`` and
``drive-finetune`` is ``finetune``; run names follow the name given.
"""

import argparse
import sys

TASKS = {"locomotion": "locomotion", "finetune": "locomotion_ft", "finetune2": "locomotion_ft2",
         "finetune3": "locomotion_ft3", "getup": "getup", "getup2": "getup2", "getup3": "getup3", "getup4": "getup4",
         # M16 behaviour names (config/behaviours.json); the names above stay as aliases.
         "drive": "locomotion", "drive-finetune": "locomotion_ft",
         "finetune4": "locomotion_ft4", "drive-finetune4": "locomotion_ft4",
         "sit_stand": "sit_stand", "sit-stand": "sit_stand"}


def train(argv):
    parser = argparse.ArgumentParser(prog="bipe train")
    parser.add_argument("--task", choices=sorted(TASKS), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--iterations", type=int, default=8000)
    parser.add_argument("--num-envs", type=int, default=4096)
    parser.add_argument("--log-root", default="local/m14/runs")
    parser.add_argument("--resume-run")
    parser.add_argument("--resume-checkpoint", default="model_.*.pt")
    args, extra = parser.parse_known_args(argv)
    if args.resume_run:
        extra = ["--agent.resume", "True", "--agent.load-run", args.resume_run,
                 "--agent.load-checkpoint", args.resume_checkpoint, *extra]  # fmt: skip
    from .m13.rl import train as m13_train

    sys.argv = [
        "bipe.m13.rl.train",
        "--variant", TASKS[args.task],
        "--env.scene.num-envs", str(args.num_envs),
        "--agent.seed", str(args.seed),
        "--agent.max-iterations", str(args.iterations),
        "--agent.run-name", f"{args.task}_s{args.seed}",
        "--log-root", args.log_root,
        *extra,
    ]  # fmt: skip
    m13_train.main()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(prog="bipe", description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=["train", "sim"])
    # Parse only the command, so that "bipe sim --help" reaches the sim's own parser.
    args = parser.parse_args(argv[:1])
    rest = argv[1:]
    if args.command == "sim":
        from .sim import main as sim

        sys.exit(sim(rest))
    train(rest)


if __name__ == "__main__":
    main()
