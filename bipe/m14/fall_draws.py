"""M14: which stress draws make a policy fall (training-plant stress suite, one episode per world).

    python -m bipe.m14.fall_draws CKPT [CKPT ...] --seed 1 [--device cuda:0] [--worlds 1000]
        [--output local/m14/fall_draws]

Runs the gate's stress suite (bipe.m13.rl.evaluate.stress settings) with a fixed
environment seed and records, per world, the drive delays and lags, actuator draws,
the slope along the heading, every push (signed along the heading) and the fall
cause and time. With the same seed, checkpoints see the same draws until their first
fall, so the falls can be compared world by world.
"""

import argparse
import json
from pathlib import Path


def run(checkpoint, seed, device, worlds, episode_s=10.0):
    import torch

    from ..m13.rl.env_cfg import env_cfg
    from ..m13.rl.evaluate import load_policy
    from ..m13.rl.mdp import ROBOT, _heading

    cfg = env_cfg(num_envs=worlds, robot=True, full_push=True, belts=True, actuators=True,
                  stress=True)  # fmt: skip
    cfg.episode_length_s = episode_s
    cfg.seed = seed
    env, wrapped, policy = load_policy(Path(checkpoint), cfg, device)
    drive = env.action_manager.get_term("drive")
    robot = env.scene[ROBOT]
    terms = [t for t in env.termination_manager.active_terms if t != "time_out"]
    push_s = cfg.events["push"].params.get("duration_s", 0.1)  # push_force's default
    obs, _ = wrapped.reset()
    h = _heading(env)
    fwd = torch.stack([torch.cos(h), torch.sin(h), torch.zeros_like(h)], dim=-1)
    slope = torch.rad2deg(torch.asin((robot.data.gravity_vec_w * fwd).sum(-1)))
    rows = [{
        "world": i, "delay_ms": int(drive.transport_delay[i]), "encoder_lag": int(drive.encoder_lag[i]),
        "hub_tau_ms": float(drive.hub_tau[i].mean() * 1000), "torque_scale": float(drive.torque_scale[i].mean()),
        "slope_deg": float(slope[i]), "pushes": [], "fell": False, "cause": [], "t": None,
    } for i in range(worlds)]  # fmt: skip
    for name in ("pack_V", "friction_scale", "hip_tau", "belt_params"):
        if hasattr(drive, name):
            v = getattr(drive, name)
            for i in range(worlds):
                rows[i][name] = v[i].flatten().tolist()
    live = torch.ones(worlds, dtype=torch.bool, device=device)
    pushing = torch.zeros(worlds, dtype=torch.bool, device=device)
    for step in range(env.max_episode_length):
        with torch.no_grad():
            obs, _, dones, _ = wrapped.step(policy(obs))
        now = drive.push_steps > 0
        new = now & ~pushing & live
        if new.any():
            h = _heading(env)
            f = drive.push_force[:, 0, :2]
            along = (f[:, 0] * torch.cos(h) + f[:, 1] * torch.sin(h)) * push_s
            for i in new.nonzero().flatten().tolist():
                rows[i]["pushes"].append([round(step * env.step_dt, 2), round(float(along[i]), 3)])
        pushing = now
        ended = env.termination_manager.terminated.bool() & live
        for i in ended.nonzero().flatten().tolist():
            rows[i]["fell"] = True
            rows[i]["t"] = round((step + 1) * env.step_dt, 2)
            rows[i]["cause"] = [t for t in terms if bool(env.termination_manager.get_term(t)[i])]
        live &= ~dones.bool()
    env.close()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checkpoints", nargs="+", type=Path)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--worlds", type=int, default=1000)
    parser.add_argument("--output", type=Path, default=Path("local/m14/fall_draws"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for ckpt in args.checkpoints:
        rows = run(ckpt, args.seed, args.device, args.worlds)
        name = f"{ckpt.resolve().parent.name}_seed{args.seed}"
        (args.output / f"{name}.json").write_text(json.dumps(rows) + "\n")
        print(name, sum(r["fell"] for r in rows), "falls of", len(rows), flush=True)


if __name__ == "__main__":
    main()
