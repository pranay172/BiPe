"""M18 phase B: the fidelity suite on the full plant.

    python -m bipe.m18.suite [--checkpoint PATH] [--case NAME ...] [--workers 4]

Each M18 model (bipe.m18.fidelity) alone at the wide end of its training range, plus
eight combined draws from inside every range at once (stratified as in
bipe.m14.stress), each under three scenarios: a 20 s hold (rest lean and drift), a
2 N s push forward and back while holding, and the M10 drive (0.3 m/s from 2 to 10 s).

Without ``--checkpoint`` the classical controller runs; with it the policy runs in its
place (bipe.m13.policy_control.policy_in_loop). Results go to
artifacts/m18/fidelity_classical or artifacts/m18/fidelity_<run>. ``--measured`` runs
every case on the measured robot (bipe.m18.measured, config/hardware-measured.json),
with the case's model on top: the suite then says what the measured robot loses.

The full plant applies timing jitter in whole 5 ms ticks, so at the training range
(0 to 1 ms) the timing and combined cases test packet and command loss only.
``command_loss`` reaches the policy path only; the classical controller runs those
cases without it. Each case's seed is the CRC-32 of its name (``seed_of``).
"""

from __future__ import annotations

import argparse
import json
import multiprocessing
import zlib
from concurrent.futures import ProcessPoolExecutor
from functools import partial
from pathlib import Path

import mujoco
import numpy as np

from ..moving_experiment import simulate
from ..parameters import ROOT, load_parameters
from ..profiles import native
from ..robustness import command, physical_metrics
from ..robustness_control import build_candidate
from ..provenance import commit, sha256
from . import fidelity, measured

SCENARIOS = {
    "hold": {"family": "hold", "mode": "hold", "duration": 20, "disturbance": {}},
    "push_fwd": {"family": "push", "mode": "hold", "duration": 16,
                 "disturbance": {"push_Ns": 2.0, "initial_posture_deg": 40}},
    "push_back": {"family": "push", "mode": "hold", "duration": 16,
                  "disturbance": {"push_Ns": -2.0, "initial_posture_deg": 40}},
    "drive": {"family": "push", "mode": "drive", "duration": 16,
              "disturbance": {"initial_posture_deg": 40}},
}  # fmt: skip
"""``drive`` reuses the push family's command (0.3 m/s from 2 to 10 s) with no push."""

MODELS = {
    "nominal": {},
    "cogging": {"cogging_fraction": 0.03},
    "stiction": {"stiction_ratio": 1.5},
    "hot": {"winding_C": 80.0},
    "radius": {"wheel_radius_ratio": (0.97, 1.03)},
    "imu_pitch_fwd": {"imu_mount_deg": (0.0, 2.0)},
    "imu_pitch_back": {"imu_mount_deg": (0.0, -2.0)},
    "imu_roll": {"imu_mount_deg": (2.0, 0.0)},
    "accel": {"accel_scale": (0.02, -0.02, 0.02), "accel_cross": 0.01},
    "gyro_walk": {"gyro_walk": 3e-4},
    "encoders": {"encoder_offset_deg": (1.5, -1.5, -1.5, 1.5)},
    "timing": {"jitter_ms": 1, "packet_loss": 0.02, "command_loss": 0.02},
    "slippery": {"floor_friction": 0.3},
    "soft": {"floor_stiffness": 0.3},
    "payload": {"payload_kg": 0.1},
}
DRAWS, SEED = 8, 20261116


def combined():
    """Eight draws with every model inside its range, stratified per parameter."""
    rng = np.random.default_rng(SEED)
    r = fidelity.RANGES
    cell = lambda: (rng.permutation(DRAWS) + rng.random(DRAWS)) / DRAWS
    span = lambda lo, hi, c: lo + (hi - lo) * c
    sym = lambda x, c: x * (2 * c - 1)
    cells = {k: [cell() for _ in range(6)] for k in r}
    draws = []
    for i in range(DRAWS):
        c = lambda k, j=0: float(cells[k][j][i])
        draws.append({
            "cogging_fraction": span(*r["cogging_fraction"], c("cogging_fraction")),
            "stiction_ratio": span(*r["stiction_ratio"], c("stiction_ratio")),
            "winding_C": span(*r["winding_C"], c("winding_C")),
            "wheel_radius_ratio": tuple(span(*r["wheel_radius_ratio"], c("wheel_radius_ratio", j)) for j in range(2)),
            "imu_mount_deg": tuple(sym(r["imu_mount_deg"], c("imu_mount_deg", j)) for j in range(2)),
            "accel_scale": tuple(sym(r["accel_scale"], c("accel_scale", j)) for j in range(3)),
            "accel_cross": r["accel_cross"] * c("accel_cross"),
            "gyro_walk": span(*r["gyro_walk"], c("gyro_walk")),
            "encoder_offset_deg": tuple(sym(r["encoder_offset_deg"], c("encoder_offset_deg", j)) for j in range(4)),
            "jitter_ms": int(round(span(*r["jitter_ms"], c("jitter_ms")))),
            "packet_loss": span(*r["packet_loss"], c("packet_loss")),
            "command_loss": span(*r["command_loss"], c("command_loss")),
            "floor_friction": span(*r["floor_friction"], c("floor_friction")),
            "floor_stiffness": span(*r["floor_stiffness"], c("floor_stiffness")),
            "payload_kg": span(*r["payload_kg"], c("payload_kg")),
        })  # fmt: skip
    return draws


def cases():
    out = {}
    models = dict(MODELS)
    models.update({f"combined{i}": d for i, d in enumerate(combined())})
    for model, draw in models.items():
        for name, s in SCENARIOS.items():
            key = f"{model}__{name}"
            out[key] = {"name": key, "model": model, "draw": draw, **s}
    return out


def seed_of(name):
    """The case's fixed seed (CRC-32 of its name)."""
    return zlib.crc32(name.encode())


def run(payload):
    p, gains, sensors, library, case, seed, base = payload
    from ..m13 import policy_control

    # The measured robot (--measured) under the case's own model.
    plant, config, policy = fidelity.apply(p, sensors, {**base, **case["draw"]}, seed=seed)
    policy_control.FIDELITY = policy
    try:
        result = _simulate(p, gains, plant, config, library, case)
    except ValueError as error:
        # The C core refuses a non-finite input or a leg past its limit: a failure.
        return case["name"], {"physical_pass": False, "recovered": False, "error": str(error)}
    metrics = physical_metrics(result)
    metrics["policy_steps"] = result.get("policy_steps")
    return case["name"], metrics


def _simulate(p, gains, plant, config, library, case):
    return simulate(
        case["mode"],
        case["duration"],
        sensing="sensors",
        nominal=p,
        plant=plant,
        gains=gains,
        sensor_config=config,
        implementation="c",
        firmware_library=library,
        disturbance=case["disturbance"],
        command_function=partial(command, case=case, nominal=p),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--model", action="append", default=[])
    parser.add_argument("--measured", nargs="?", const=measured.PATH, type=Path,
                        help="run on the measured robot (config/hardware-measured.json or PATH)")
    args = parser.parse_args()
    if (args.case or args.model) and not args.output:
        # A partial run would otherwise overwrite the full 92-case record.
        parser.error("--case/--model need --output (the default is the full record)")
    p = load_parameters(profile="m10")
    profile = native(p)
    gains = json.loads((ROOT / profile["controller"]).read_text())
    sensors = json.loads((ROOT / profile["sensors"]).read_text())
    base = {}
    if args.measured:
        values = measured.load(args.measured)
        p, sensors = measured.apply_plant(p, sensors, values)
        base = measured.draw(values)
    library = str(build_candidate(p, gains, ROOT / profile["firmware_build"]))
    run_name = args.checkpoint.resolve().parent.name if args.checkpoint else "classical"
    run_name += "_measured" if args.measured else ""
    output = args.output or ROOT / f"artifacts/m18/fidelity_{run_name}"
    output.mkdir(parents=True, exist_ok=True)
    todo = {
        k: v for k, v in cases().items()
        if (not args.case or k in args.case) and (not args.model or v["model"] in args.model)
    }  # fmt: skip
    # The seed (cogging phases, accelerometer cross-axis signs, loss streams) comes from
    # the case name, so a partial rerun (--case, --model) gets the same robot as a full run.
    payloads = [(p, gains, sensors, library, c, seed_of(c["name"]), base) for c in todo.values()]

    def execute():
        with ProcessPoolExecutor(args.workers) as pool:
            return dict(pool.map(run, payloads))

    multiprocessing.set_start_method("fork", force=True)
    if args.checkpoint:
        from ..m13.policy_control import policy_in_loop

        with policy_in_loop(args.checkpoint, runners=(__name__,)):
            results = execute()
    else:
        results = execute()
    rows = {
        k: {"model": todo[k]["model"], "draw": todo[k]["draw"], "seed": seed_of(k), **results[k]}
        for k in todo
    }
    falls = sorted(k for k, r in rows.items() if not r["physical_pass"])
    by_model = {}
    for k, r in rows.items():
        m = by_model.setdefault(r["model"], {"cases": 0, "survived": 0, "recovered": 0})
        m["cases"] += 1
        m["survived"] += bool(r["physical_pass"])
        m["recovered"] += bool(r.get("recovered"))
    summary = {
        "controller": str(args.checkpoint) if args.checkpoint else "classical",
        "commit": commit(),
        "checkpoint_sha256": sha256(args.checkpoint) if args.checkpoint else None,
        "mujoco": mujoco.__version__,
        "measured": {"file": str(args.measured), "draw": base} if args.measured else None,
        "cases": len(rows),
        "survived": len(rows) - len(falls),
        "recovered": sum(bool(r.get("recovered")) for r in rows.values()),
        "fallen": falls,
        "by_model": by_model,
    }
    (output / "cases.json").write_text(json.dumps(rows, indent=2, default=float) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
