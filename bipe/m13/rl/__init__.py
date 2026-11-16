"""M13 phase B: the mjlab task for the first learned policy (run in .venv/rl)."""

TASK = "Bipe-M13-Flat"
# The same task on the robot's sensor channels and full-plant falls (phase C findings).
ROBOT_TASK = "Bipe-M13-Robot"


def register(robot=False, variant=None):
    """Register the task (``variant``: a key of env_cfg.VARIANTS) and return its id."""
    from mjlab.tasks.registry import list_tasks, register_mjlab_task

    from .env_cfg import VARIANTS, env_cfg, runner_cfg

    if variant is not None:
        task, kwargs = f"{ROBOT_TASK}-{variant}", VARIANTS[variant]()
    else:
        task, kwargs = (ROBOT_TASK if robot else TASK), {"robot": robot}
    if task not in list_tasks():
        register_mjlab_task(
            task_id=task,
            env_cfg=env_cfg(hard=True, **kwargs),
            play_env_cfg=env_cfg(play=True, **kwargs),
            rl_cfg=runner_cfg(),
        )
    return task
