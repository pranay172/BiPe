"""M14 stress feasibility: training-plant fall rate as each stress factor is switched on.

    python -m bipe.m14.feasibility CKPT [CKPT ...] [--device cuda:0] [--worlds 1000]
        [--episodes 1000] [--output local/m14/feasibility]

Each setting runs in its own process (env_cfg.STRESS is patched per setting) and
appends a line to <output>/<checkpoint run>.jsonl. Settings: training ranges with and
without pushes, the stress delays alone, the stress mix, fixed push sizes forward
and backward under the stress delays, and the stress mix without slopes or without
actuator randomisation.
"""

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

TRAIN_DRIVE = {
    "transport_delay_ms": [5, 20],
    "encoder_lag_ms": [0, 5],
    "hub_lag_s": [0.0008, 0.025],
}
STRESS_DRIVE = {
    "transport_delay_ms": [5, 23],
    "encoder_lag_ms": [0, 8],
    "hub_lag_s": [0.0008, 0.0375],
}


def settings():
    out = {
        "train_nopush": {"drive": TRAIN_DRIVE, "impulse": [0, 0]},
        "train": {"drive": TRAIN_DRIVE, "impulse": [0, 2.5]},
        "stressdrive_nopush": {"drive": STRESS_DRIVE, "impulse": [0, 0]},
        "stressdrive_push2.5": {"drive": STRESS_DRIVE, "impulse": [0, 2.5]},
        "stress": {"drive": STRESS_DRIVE, "impulse": [0, 3.75]},
        "stress_flat": {"drive": STRESS_DRIVE, "impulse": [0, 3.75], "slope": 0.0},
        "stress_noactdr": {"drive": STRESS_DRIVE, "impulse": [0, 3.75], "actuators": False},
        "flat_nopush": {"drive": TRAIN_DRIVE, "impulse": [0, 0], "slope": 0.0},
    }
    for size in (1.0, 1.5, 2.0, 2.5, 3.0, 3.75):
        for name, fwd in (("fwd", 1.0), ("back", 0.0)):
            out[f"push_{name}_{size}"] = {
                "drive": STRESS_DRIVE,
                "impulse": [size, size],
                "forward": fwd,
                "slope": 0.0,
            }
    return out


def run(ckpt, drive, impulse, worlds, episodes, device, forward=None, slope=10.0,
        actuators=True, episode_s=10.0):  # fmt: skip
    import torch

    from ..m13.rl import env_cfg as E
    from ..m13.rl.evaluate import load_policy

    E.STRESS["drive"] = {k: tuple(v) for k, v in drive.items()}
    E.STRESS["impulse_Ns"] = tuple(impulse)
    E.STRESS["slope_deg"] = slope
    cfg = E.env_cfg(num_envs=worlds, robot=True, full_push=True, belts=True,
                    actuators=actuators, stress=True)  # fmt: skip
    cfg.episode_length_s = episode_s
    if forward is not None:
        cfg.events["push"].params["forward"] = forward
    env, wrapped, policy = load_policy(Path(ckpt), cfg, device)
    terms = [t for t in env.termination_manager.active_terms if t != "time_out"]
    falls, causes, total = 0, dict.fromkeys(terms, 0), 0
    for _ in range(math.ceil(episodes / worlds)):
        obs, _ = wrapped.reset()
        live = torch.ones(worlds, dtype=torch.bool, device=device)
        for _ in range(env.max_episode_length):
            with torch.no_grad():
                obs, _, dones, _ = wrapped.step(policy(obs))
            ended = env.termination_manager.terminated.bool() & live
            falls += int(ended.sum())
            for t in terms:
                causes[t] += int((ended & env.termination_manager.get_term(t).bool()).sum())
            live &= ~dones.bool()
        total += worlds
    env.close()
    return {"episodes": total, "falls": falls, "fall_rate": falls / total, "causes": causes}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checkpoints", nargs="+", type=Path)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--worlds", type=int, default=1000)
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--output", type=Path, default=Path("local/m14/feasibility"))
    parser.add_argument("--setting", action="append", default=[])
    parser.add_argument("--one", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.one:
        spec = json.loads(args.one)
        t = time.time()
        out = run(**spec)
        print("RESULT " + json.dumps({**out, "minutes": (time.time() - t) / 60}), flush=True)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    todo = {k: v for k, v in settings().items() if not args.setting or k in args.setting}
    for ckpt in args.checkpoints:
        name = ckpt.resolve().parent.name
        path = args.output / f"{name}.jsonl"
        done = set()
        if path.exists():
            done = {json.loads(line)["setting"] for line in path.read_text().splitlines() if line}
        for key, s in todo.items():
            if key in done:
                continue
            spec = {"ckpt": str(ckpt), "worlds": args.worlds, "episodes": args.episodes,
                    "device": args.device, **s}  # fmt: skip
            proc = subprocess.run([sys.executable, "-m", "bipe.m14.feasibility", str(ckpt),
                                   "--one", json.dumps(spec)], capture_output=True, text=True)  # fmt: skip
            lines = [x for x in proc.stdout.splitlines() if x.startswith("RESULT ")]
            if proc.returncode or not lines:
                print(f"{name} {key}: FAILED\n{proc.stderr[-2000:]}", flush=True)
                continue
            row = {"setting": key, **s, **json.loads(lines[-1][7:])}
            with path.open("a") as f:
                f.write(json.dumps(row) + "\n")
            print(
                f"{name} {key}: {row['falls']}/{row['episodes']} falls {row['causes']}", flush=True
            )


if __name__ == "__main__":
    main()
