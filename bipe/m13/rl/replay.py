"""Replay a trained M13 policy under fixed commands and summarise what it does.

    .venv/rl/bin/python -m bipe.m13.rl.replay <checkpoint.pt> [--device cuda:0]

Each case holds one (speed, yaw rate, posture) command for the whole episode, on the
play configuration (no pushes). Prints per case the late means of forward speed,
yaw rate, pitch, axle depth (and its command, mm), wheel torque, wheel speeds and
actions, and the share of worlds that fell; saves
the traces next to the checkpoint as replay.npz.
"""

import argparse
from dataclasses import asdict
from pathlib import Path

import numpy as np
import torch

CASES = [
    ("hold", 0.0, 0.0, 40.0),
    ("drive_0.5", 0.5, 0.0, 40.0),
    ("drive_1.0", 1.0, 0.0, 40.0),
    ("turn_+0.5", 0.0, 0.5, 40.0),
    ("turn_-0.5", 0.0, -0.5, 40.0),
    ("turn_+1.0", 0.0, 1.0, 40.0),
    ("arc", 0.5, 0.5, 40.0),
    ("crouch", 0.0, 0.0, 60.0),
    ("tall", 0.0, 0.0, 30.0),
]


def load_policy(checkpoint, cfg, device):
    """The environment for ``cfg``, its rsl_rl wrapper and the checkpoint's policy."""
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper

    from .env_cfg import runner_cfg

    env = ManagerBasedRlEnv(cfg, device=device)
    agent = runner_cfg()
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
    runner = MjlabOnPolicyRunner(wrapped, asdict(agent), device=device)
    runner.load(str(checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
    return env, wrapped, runner.get_inference_policy(device=device)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--worlds", type=int, default=64)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--robot", action="store_true", help="robot sensor channels and falls")
    args = parser.parse_args()

    from . import mdp
    from .env_cfg import env_cfg

    cfg = env_cfg(play=True, num_envs=args.worlds, robot=args.robot)
    env, wrapped, policy = load_policy(args.checkpoint, cfg, args.device)

    command = env.command_manager.get_term("drive")
    drive = env.action_manager.get_term("drive")
    robot = env.scene[mdp.ROBOT]
    steps = round(args.seconds / env.step_dt)
    traces = {}
    print(f"{'case':10s} {'v_cmd':>6s} {'v':>6s} {'w_cmd':>6s} {'w':>6s} {'|w|':>5s} {'pitch':>6s} "
          f"{'fell':>5s} {'depth':>6s} {'d_cmd':>6s} {'|tau_w|':>7s}  "
          f"wheel speeds (L, R)   actions (mean)")  # fmt: skip
    for name, v, w, posture in CASES:
        obs, _ = wrapped.reset()
        fell = torch.zeros(args.worlds, dtype=torch.bool, device=env.device)
        rows = []
        for _ in range(steps):
            command.target[:] = torch.tensor([v, w, posture], device=env.device)
            command.time_left[:] = 1e9
            with torch.no_grad():
                action = policy(obs)
            obs, _, dones, _ = wrapped.step(action)
            fell |= dones.bool()
            pitch, _ = mdp.pitch_roll(env)
            rows.append(
                torch.cat(
                    [
                        torch.stack([mdp.forward_speed(env), mdp.yaw_rate(env), pitch], dim=-1),
                        robot.data.joint_vel[:, drive.wheel],
                        action,
                        torch.stack(
                            [mdp.axle_depth(env), command.depth(),
                             drive.output_torque[:, 4:].abs().mean(dim=-1)], dim=-1
                        ),
                    ],
                    dim=-1,
                ).cpu().numpy()
            )
        trace = np.stack(rows)  # steps, worlds, 14
        traces[name] = trace
        late = trace[steps // 2 :][:, ~fell.cpu().numpy()]
        if late.shape[1] == 0:
            late = trace[steps // 2 :]
        m = late.reshape(-1, late.shape[-1]).mean(axis=0)
        print(
            f"{name:10s} {v:6.2f} {m[0]:6.2f} {w:6.2f} {m[1]:6.2f} {np.abs(late[..., 1]).mean():5.2f} "
            f"{np.degrees(m[2]):6.1f} "
            f"{fell.float().mean().item():5.2f} {m[11] * 1e3:6.1f} {m[12] * 1e3:6.1f} {m[13]:7.3f}  "
            f"{m[3]:7.1f} {m[4]:7.1f}   "
            + " ".join(f"{x:+.2f}" for x in m[5:11])
        )
    out = args.checkpoint.with_name("replay.npz")
    np.savez_compressed(out, **traces)
    print("saved", out)


if __name__ == "__main__":
    main()
