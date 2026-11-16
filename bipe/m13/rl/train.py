"""Train the M13 policy: registers the task, then runs mjlab's trainer.

    .venv/rl/bin/python -m bipe.m13.rl.train --env.scene.num-envs 4096
    .venv/rl/bin/python -m bipe.m13.rl.train --robot ...   # robot sensor channels
    .venv/rl/bin/python -m bipe.m13.rl.train --variant narrow ...   # env_cfg.VARIANTS
"""

import sys

from . import register


def main():
    args = sys.argv[1:]
    robot = "--robot" in args
    variant = None
    if "--variant" in args:
        i = args.index("--variant")
        variant = args[i + 1]
        del args[i : i + 2]
    task = register(robot=robot, variant=variant)
    from mjlab.scripts.train import main as mjlab_main

    sys.argv = [sys.argv[0], task, *(a for a in args if a != "--robot")]
    mjlab_main()


if __name__ == "__main__":
    main()
