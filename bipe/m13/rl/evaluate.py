"""Check a trained M13 policy against the phase B exit thresholds (training plant).

    .venv/rl/bin/python -m bipe.m13.rl.evaluate <checkpoint.pt> [--device cuda:0] [--robot]

``--robot`` runs the robot sensor channels and full-plant falls (env_cfg(robot=True))
and saves evaluation_robot.json.

Tracking: randomised episodes on the training configuration without pushes (all
other randomisation on), commands drawn by the task's own command term. Every world
runs one episode per round; terminations count, and the errors are taken in steady
segments only (the command has reached its target and held it for ``--settle``
seconds) of the episodes that ran to the time limit. The same errors including the
terminated episodes, the yaw-rate error with no turn commanded, and breakdowns by
the randomised parameters are reported beside them.

Pushes: hold and drive at 0.5 m/s, a push of exactly ``--push`` N s forwards and
backwards after 4 s. A push is recovered when the world does not fall and, within
3 s, pitch is back under 3 degrees and speed within 0.1 m/s of the command for the
rest of the window.

Writes the numbers to evaluation.json next to the checkpoint.
"""

import argparse
import json
import math
from pathlib import Path

import torch

from .replay import load_policy

# Phase B exit as relaxed by the user on 23 October 2026 (was 0 terminations and
# every push recovered).
EXIT = {
    "termination_rate": 0.03,
    "speed_rms_m_s": 0.1,
    "yaw_rate_rms_rad_s": 0.15,
    "posture_p95_mm": 5.0,
    "push_recovered": 0.9,
}


def tracking(
    checkpoint, device, worlds, episodes, settle_s, episode_s=20.0, robot=False, full_push=False
):
    from . import mdp
    from .env_cfg import env_cfg

    cfg = env_cfg(num_envs=worlds, robot=robot, full_push=full_push)
    cfg.events.pop("push")
    cfg.episode_length_s = episode_s
    env, wrapped, policy = load_policy(checkpoint, cfg, device)
    command = env.command_manager.get_term("drive")
    drive = env.action_manager.get_term("drive")
    steps = env.max_episode_length
    settle = round(settle_s / env.step_dt)
    rounds = math.ceil(episodes / worlds)
    # Per world and round: squared errors and sample counts in steady segments
    # (speed, yaw rate, yaw rate with no turn commanded), and depth errors.
    sums, depths, params_all, fell_all, causes_all = [], [], [], [], []
    terms = [t for t in env.termination_manager.active_terms if t != "time_out"]
    for _ in range(rounds):
        obs, _ = wrapped.reset()
        g = env.sim.model.opt.gravity
        params_all.append(
            torch.stack(
                [
                    drive.transport_delay.float(),
                    torch.rad2deg(torch.acos((-g[:, 2] / g.norm(dim=-1)).clamp(-1, 1))),
                    drive.noise_scale[:, 0],
                    drive.hub_tau[:, 0] * 1e3,
                ],
                dim=-1,
            )
        )
        acc = torch.zeros(worlds, 5, device=device)
        depth = []
        fell = torch.zeros(worlds, dtype=torch.bool, device=device)
        live = torch.ones(worlds, dtype=torch.bool, device=device)
        held = torch.zeros(worlds, dtype=torch.long, device=device)
        # Per world, the termination terms that ended its episode.
        causes = torch.zeros(worlds, len(terms), dtype=torch.bool, device=device)
        for _ in range(steps):
            with torch.no_grad():
                obs, _, dones, _ = wrapped.step(policy(obs))
            ended = env.termination_manager.terminated.bool() & live
            for i, t in enumerate(terms):
                causes[:, i] |= ended & env.termination_manager.get_term(t).bool()
            fell |= ended
            live &= ~dones.bool()
            reached = (command.value - command.target).abs().amax(dim=-1) < 1e-6
            held = torch.where(reached, held + 1, torch.zeros_like(held))
            steady = (live & (held > settle)).float()
            c = command.value
            yaw_sq = (mdp.yaw_rate(env) - c[:, 1]) ** 2
            still = steady * (c[:, 1].abs() < 0.05).float()
            acc += torch.stack(
                [(mdp.forward_speed(env) - c[:, 0]) ** 2 * steady, yaw_sq * steady, steady,
                 yaw_sq * still, still], dim=-1,
            )  # fmt: skip
            depth.append(torch.where(steady.bool(), (mdp.axle_depth(env) - command.depth()).abs(), torch.nan))
        sums.append(acc)
        depths.append(torch.stack(depth, dim=-1))
        fell_all.append(fell)
        causes_all.append(causes)
    env.close()
    acc, depth = torch.cat(sums), torch.cat(depths)
    params, fell, causes = torch.cat(params_all), torch.cat(fell_all), torch.cat(causes_all)

    def rms(rows, i, n):
        count = float(acc[rows, n].sum())
        return math.sqrt(float(acc[rows, i].sum()) / count) if count else math.nan

    def p95(rows):
        d = depth[rows].flatten()
        d = d[~torch.isnan(d)]
        return float(torch.quantile(d.float(), 0.95)) * 1e3 if len(d) else math.nan

    done = ~fell
    every = torch.ones_like(done)
    # The exit is judged on the episodes that ran to the time limit: a falling robot's
    # last steps are not tracking, and the falls are counted on their own.
    return {
        "episodes": len(fell),
        "terminations": int(fell.sum()),
        "steady_samples": int(acc[done, 2].sum()),
        "speed_rms_m_s": rms(done, 0, 2),
        "yaw_rate_rms_rad_s": rms(done, 1, 2),
        "yaw_rate_rms_no_turn_rad_s": rms(done, 3, 4),
        "posture_p95_mm": p95(done),
        "including_terminated": {
            "speed_rms_m_s": rms(every, 0, 2),
            "yaw_rate_rms_rad_s": rms(every, 1, 2),
            "posture_p95_mm": p95(every),
        },
        "termination_rate_by": breakdown(params, fell.float()),
        # Episodes ended by each term (a step can end on several at once).
        "terminations_by_term": {t: int(causes[:, i].sum()) for i, t in enumerate(terms)},
        "termination_term_rate_by_slope": {
            t: breakdown(params, causes[:, i].float())["slope_deg"] for i, t in enumerate(terms)
        },
        "yaw_rate_rms_by": {
            name: {b: round(rms(done & m, 1, 2), 3) for b, m in masks(params[:, col], edges)}
            for name, (col, edges) in BINS.items()
        },
    }


BINS = {
    "transport_delay_ms": (0, [5, 10, 15, 21]),
    "slope_deg": (1, [0, 0.01, 5, 10.01]),
    "noise_scale": (2, [0.5, 1.0, 1.5, 2.01]),
    "hub_lag_ms": (3, [0.8, 3, 10, 25.1]),
}


def masks(x, edges):
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (x >= lo) & (x < hi)
        if mask.any():
            yield f"{lo:g}-{hi:g}", mask


def breakdown(params, failed):
    """Share of episodes that ended early, per bin of each randomised parameter."""
    return {
        name: {
            b: {"episodes": int(m.sum()), "terminated": round(float(failed[m].mean()), 3)}
            for b, m in masks(params[:, col], edges)
        }
        for name, (col, edges) in BINS.items()
    }


def pushes(checkpoint, device, worlds, impulse, at_s=4.0, window_s=6.0, robot=False, force=False):
    from . import mdp
    from .env_cfg import env_cfg

    cfg = env_cfg(play=True, num_envs=worlds, robot=robot, full_push=force)
    env, wrapped, policy = load_policy(checkpoint, cfg, device)
    command = env.command_manager.get_term("drive")
    before, window = round(at_s / env.step_dt), round(window_s / env.step_dt)
    limit = round(3.0 / env.step_dt)
    results = {}
    for name, v in (("hold", 0.0), ("drive_0.5", 0.5)):
        for sign in (1.0, -1.0):
            obs, _ = wrapped.reset()
            fell = torch.zeros(worlds, dtype=torch.bool, device=device)
            ok = torch.zeros(window, worlds, dtype=torch.bool, device=device)
            for k in range(before + window):
                command.target[:] = torch.tensor([v, 0.0, 40.0], device=device)
                command.time_left[:] = 1e9
                if k == before and force:
                    # As mdp.push_force (the full plant's push), size and sign fixed.
                    mdp.push_force(env, None, (impulse, impulse), 1.0 if sign > 0 else 0.0)
                elif k == before:
                    # As mdp.push, with the size and sign fixed.
                    robot = env.scene[mdp.ROBOT]
                    mass = float(env.sim.mj_model.body_subtreemass[robot.indexing.root_body_id])
                    h = mdp._heading(env)
                    vel = robot.data.root_link_vel_w.clone()
                    vel[:, 0] += sign * impulse / mass * torch.cos(h)
                    vel[:, 1] += sign * impulse / mass * torch.sin(h)
                    robot.write_root_link_velocity_to_sim(vel)
                with torch.no_grad():
                    obs, _, dones, _ = wrapped.step(policy(obs))
                fell |= dones.bool()
                if k >= before:
                    pitch, _ = mdp.pitch_roll(env)
                    good = (pitch.abs() < math.radians(3.0)) & (
                        (mdp.forward_speed(env) - v).abs() < 0.1
                    )
                    ok[k - before] = good
            # Recovered: never fell, and good from some step within 3 s to the end.
            tail = torch.flip(torch.cumprod(torch.flip(ok, [0]).long(), dim=0), [0]).bool()
            recovered = ~fell & tail[:limit].any(dim=0)
            first = torch.where(tail, torch.arange(window, device=device)[:, None], window)
            settle = first.amin(dim=0).float()[recovered] * env.step_dt
            key = f"{name}_{'+' if sign > 0 else '-'}{impulse:g}"
            results[key] = {
                "fell": float(fell.float().mean()),
                "recovered": float(recovered.float().mean()),
                "recovery_s_mean": float(settle.mean()) if len(settle) else math.nan,
            }
    env.close()
    return results


def stress(checkpoint, device, worlds, episodes, episode_s=20.0, seed=None):
    """M14 stress suite: falls with belts, actuator and power randomisation and the
    stress mix (env_cfg STRESS), robot sensing and the full plant's pushes and falls.
    The gate is at most 5 % of episodes ending in a fall."""
    from .env_cfg import env_cfg

    cfg = env_cfg(
        num_envs=worlds, robot=True, full_push=True, belts=True, actuators=True, stress=True
    )
    cfg.episode_length_s = episode_s
    cfg.seed = seed
    env, wrapped, policy = load_policy(checkpoint, cfg, device)
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
    return {"episodes": total, "falls": falls, "fall_rate": falls / total, "causes": causes,
            "passed": falls / total <= 0.05}  # fmt: skip


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--worlds", type=int, default=1000)
    parser.add_argument("--episodes", type=int, default=1000)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--push", type=float, default=2.0)
    parser.add_argument("--push-worlds", type=int, default=256)
    parser.add_argument("--episode-s", type=float, default=20.0)
    parser.add_argument("--push-at", type=float, default=4.0, help="s before the push")
    parser.add_argument("--window", type=float, default=6.0, help="s watched after the push")
    parser.add_argument("--robot", action="store_true", help="robot sensor channels and falls")
    parser.add_argument(
        "--push-force", action="store_true",
        help="push and fall as the full plant does (env_cfg full_push)",
    )
    parser.add_argument("--stress", action="store_true", help="the M14 stress suite only")
    parser.add_argument("--seed", type=int, help="environment seed (stress suite)")
    args = parser.parse_args()
    if args.stress:
        out = stress(args.checkpoint, args.device, args.worlds, args.episodes, args.episode_s,
                     args.seed)  # fmt: skip
        out["seed"] = args.seed
        print(json.dumps(out, indent=2))
        path = args.checkpoint.with_name("evaluation_stress.json")
        path.write_text(json.dumps(out, indent=2))
        print("saved", path)
        return

    track = tracking(
        args.checkpoint, args.device, args.worlds, args.episodes, args.settle, args.episode_s,
        args.robot, args.push_force,
    )
    push = pushes(
        args.checkpoint, args.device, args.push_worlds, args.push, args.push_at, args.window,
        args.robot, args.push_force,
    )
    worst_push = min(r["recovered"] for r in push.values())
    passed = {
        "termination_rate": track["terminations"] / track["episodes"] <= EXIT["termination_rate"],
        "speed_rms_m_s": track["speed_rms_m_s"] < EXIT["speed_rms_m_s"],
        "yaw_rate_rms_rad_s": track["yaw_rate_rms_rad_s"] < EXIT["yaw_rate_rms_rad_s"],
        "posture_p95_mm": track["posture_p95_mm"] < EXIT["posture_p95_mm"],
        "push_recovered": worst_push >= EXIT["push_recovered"],
    }
    out = {"tracking": track, "pushes": push, "exit": EXIT, "passed": passed}
    print(f"episodes {track['episodes']}, terminations {track['terminations']}, "
          f"steady samples {track['steady_samples']}")  # fmt: skip
    for k in ("speed_rms_m_s", "yaw_rate_rms_rad_s", "posture_p95_mm"):
        print(f"{k:20s} {track[k]:8.3f}  (exit < {EXIT[k]})  {'pass' if passed[k] else 'FAIL'}")
    print(f"yaw_rate_rms with no turn commanded {track['yaw_rate_rms_no_turn_rad_s']:.3f}; "
          f"including terminated episodes: speed {track['including_terminated']['speed_rms_m_s']:.3f}, "
          f"yaw rate {track['including_terminated']['yaw_rate_rms_rad_s']:.3f}")  # fmt: skip
    for name, rows in track["yaw_rate_rms_by"].items():
        print(f"yaw_rate_rms by {name:18s} " + "  ".join(f"{b}: {r:.3f}" for b, r in rows.items()))
    print("terminations by term", track["terminations_by_term"])
    for name, rows in track["termination_term_rate_by_slope"].items():
        print(f"{name:14s} by slope " + "  ".join(f"{b}: {r['terminated']:.2f}" for b, r in rows.items()))
    for name, rows in track["termination_rate_by"].items():
        print(f"terminated by {name:18s} "
              + "  ".join(f"{b}: {r['terminated']:.2f} (n {r['episodes']})" for b, r in rows.items()))
    for k, r in push.items():
        print(f"push {k:14s} fell {r['fell']:.2f}  recovered {r['recovered']:.2f}  "
              f"in {r['recovery_s_mean']:.2f} s")  # fmt: skip
    print("terminations", "pass" if passed["termination_rate"] else "FAIL",
          "| pushes", "pass" if passed["push_recovered"] else "FAIL")  # fmt: skip
    name = "evaluation_robot" if args.robot else "evaluation"
    path = args.checkpoint.with_name(name + ("_full_push" if args.push_force else "") + ".json")
    path.write_text(json.dumps(out, indent=2))
    print("saved", path)


if __name__ == "__main__":
    main()
