"""Hardware handover: the firmware's learned-controller runtime in the full plant.

    python -m bipe.m13.handover acceptance --variant inline [--case NAME ...]
    python -m bipe.m13.handover acceptance --variant hardware [--case NAME ...]
    python -m bipe.m13.handover acceptance --variant fault --fault-s 5 [--case NAME ...]
    ... [--checkpoint PATH]  (another policy with the M13 interface, e.g. an M14 seed)

``--checkpoint`` defaults to M13's settle, whose records artifacts/m13/handover holds;
the deployed policy (bipe.m13.headers.CHECKPOINT, M14's fine-tune 1) needs
``--checkpoint`` and ``--output``.

The robot side of the policy is firmware/src/locomotion/learned.c (the selection
switch, command limits, observation, action law, output guards and the classical
handover), built here for the host with the M10 configuration and run through ctypes
in place of bipe.m13.policy_control's numpy runtime. Variants:

- ``inline``: forward pass inside the tick and the wheel speed loop at 1 kHz, as the
  policy was trained and phase C/D ran it; it should reproduce the phase D records;
- ``hardware``: the M11 image's placement, the forward pass after the tick's commands
  are sent (its action applies one 5 ms tick later), the wheel speed loop's
  proportional term on the drives at 1 kHz (moteus velocity mode, kd = kv) and its
  integral on the Teensy as the 5 ms feedforward;
- ``deferred``: the action one tick later only; ``wheel_tick``: the whole wheel speed
  loop on the Teensy at the 5 ms tick only; ``deferred_tick``: both (the first wiring);
- ``fault`` (``fault_inline``, ``fault_deferred_tick``): ``hardware`` (``inline``,
  ``deferred_tick``) with a non-finite policy output from ``--fault-s`` on, so the
  firmware hands over to the classical controller (fresh balance state) mid-case.

Results go to artifacts/m13/handover/<variant>_<suite>, laid out as the runners write
them, with handover.json naming the variant.
"""

import argparse
import ctypes as c
import hashlib
import json
import multiprocessing
import os
import re
import subprocess
import sys
from functools import partial
from pathlib import Path

import numpy as np

from .. import firmware_control as fc
from .policy_control import CActor, Counted, PolicyController, PolicyEstimator

ROOT = Path(__file__).resolve().parents[2]
CHECKPOINT = ROOT / "artifacts/m13/training/settle/model_10493.pt"
SOURCES = [
    ROOT / "firmware/src/locomotion/learned.c",
    *(ROOT / f"firmware/src/{name}.c" for name in ("balance", "estimator", "kinematics")),
]
OK, BAD_INPUT, BAD_OUTPUT = 0, 1, 2
D2, D4 = c.c_double * 2, c.c_double * 4
Forward = c.CFUNCTYPE(None, c.POINTER(c.c_float), c.POINTER(c.c_float))


class Sensors(c.Structure):
    _fields_ = [(n, c.c_double * k) for n, k in
                (("gyro", 3), ("accel", 3), ("q", 4), ("qd", 4), ("wheel_speed", 2))]  # fmt: skip


class Learned(c.Structure):
    _fields_ = [
        ("command", c.c_double * 3),
        ("observation", c.c_float * 25),
        ("action", c.c_float * 6),
        ("leg_target", c.c_double * 4),
        ("wheel_target", c.c_double * 2),
        ("wheel_integral", c.c_double * 2),
        ("leg_output", c.c_double * 4),
        ("ticks", c.c_uint),
        ("leg_clamps", c.c_uint),
        ("wheel_clamps", c.c_uint),
        ("due", c.c_bool),
    ]


def compiled_leg_filter(header=ROOT / "firmware/include/locomotion_config.h"):
    """The leg target filter's time constant (s) and band (rad) the runtime compiles."""
    text = header.read_text()
    get = lambda name: float(re.search(rf"#define {name} (\S+)", text).group(1))
    return {
        "tau_s": get("BIPE_LEARNED_LEG_FILTER_TAU_S"),
        "band_rad": get("BIPE_LEARNED_LEG_FILTER_BAND_RAD"),
    }


def checkpoint_build(checkpoint, directory=ROOT / "firmware/build/checkpoints"):
    """The forward pass and runtime headers for a checkpoint other than the one the
    committed locomotion_config.h names: its weights header, and that config with only
    the checkpoint changed (the action law, timing, command limits and posture table
    are the M13 interface's, which the M14 recipe keeps), in a private include directory
    under firmware/build. Returns the include directory and its liblocomotion.so."""
    from .headers import sha256, weights_header

    sha = sha256(checkpoint)
    config = ROOT / "firmware/include/locomotion_config.h"
    source = ROOT / "firmware/src/locomotion/locomotion.c"
    key = hashlib.sha256(sha.encode())
    for path in (config, source, ROOT / "firmware/include/locomotion.h",
                 ROOT / "bipe/m13/headers.py"):  # fmt: skip
        key.update(path.read_bytes())
    include = Path(directory) / f"{sha[:16]}-{key.hexdigest()[:12]}"
    lib = include / "liblocomotion.so"
    if not lib.exists():
        include.mkdir(parents=True, exist_ok=True)
        weights_header(checkpoint, include / "generated_locomotion_weights.h")
        text = config.read_text()
        name = checkpoint.relative_to(ROOT) if checkpoint.is_relative_to(ROOT) else checkpoint
        text, n = re.subn(
            r'/\* .* policy, .*: the forward pass must\n(.*\n)#define BIPE_LEARNED_CHECKPOINT_SHA256 "\w+"',
            lambda m: (
                f"/* evaluation build of {name}: the forward pass must\n{m.group(1)}"
                f'#define BIPE_LEARNED_CHECKPOINT_SHA256 "{sha}"'
            ),
            text,
        )
        assert n == 1, "locomotion_config.h's checkpoint block has changed layout"
        (include / "locomotion_config.h").write_text(text)
        flags = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-Wshadow"]
        tmp = include / f".liblocomotion.{os.getpid()}.so"
        subprocess.run(
            ["gcc", *flags, f"-I{include}", f"-I{ROOT / 'firmware/include'}", "-fPIC", "-shared",
             str(ROOT / "firmware/src/locomotion/locomotion.c"), "-lm", "-o", str(tmp)],
            check=True,
        )  # fmt: skip
        tmp.replace(lib)  # atomic for parallel workers
    return include, lib


def build(directory=ROOT / "firmware/build/handover", include=None):
    """learned.c and its classical dependencies with the M10 generated_config.h
    (``include``: a checkpoint_build directory searched before firmware/include)."""
    from ..m11.firmware import write_m10_config

    extra = [Path(include)] if include else []
    identity = hashlib.sha256()
    headers = [h for d in extra for h in sorted(d.glob("*.h"))]
    for path in [*SOURCES, *headers, *sorted((ROOT / "firmware/include").glob("*.h"))]:
        identity.update(path.read_bytes())
    identity.update((ROOT / "tools/generate_firmware_config.py").read_bytes())
    out = Path(directory) / identity.hexdigest()[:16]
    lib = out / "liblearned.so"
    if not lib.exists():
        out.mkdir(parents=True, exist_ok=True)
        write_m10_config(out / "generated_config.h")
        flags = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-Wshadow"]
        tmp = out / f".liblearned.{os.getpid()}.so"
        subprocess.run(
            ["gcc", *flags, "-fPIC", "-shared", f"-I{out}", *(f"-I{d}" for d in extra),
             f"-I{ROOT / 'firmware/include'}",
             *map(str, SOURCES), "-lm", "-o", str(tmp)],
            check=True,
        )  # fmt: skip
        tmp.replace(lib)  # atomic for parallel workers
    dll = c.CDLL(str(lib))
    dll.bipe_learned_label.restype = c.c_double
    dll.bipe_learned_tick.argtypes = [c.c_void_p, c.c_void_p, c.c_double, c.c_double, D4]
    dll.bipe_learned_forward.argtypes = [c.c_void_p, Forward]
    dll.bipe_learned_wheels.argtypes = [c.c_void_p, D2, c.c_double, D2]
    dll.bipe_learned_legs.argtypes = [c.c_void_p, D4]
    dll.bipe_learned_wheels.restype = c.c_bool
    return dll


class RuntimeController(PolicyController):
    """PolicyController with the firmware runtime (learned.c) in place of its numpy one."""

    def __init__(self, p, gains, library=None, actor=None, runtime=None, deferred=False,
                 wheel_tick=False, fault_s=None):  # fmt: skip
        super().__init__(p, gains, library, actor)
        compiled = compiled_leg_filter()
        assert (self.leg_filter_tau, self.leg_filter_band) == (
            compiled["tau_s"],
            compiled["band_rad"],
        ), "policy_control's leg filter is not the compiled one (regenerate locomotion_config.h)"
        self.rt, self.deferred, self.wheel_tick, self.fault_s = (
            runtime,
            deferred,
            wheel_tick,
            fault_s,
        )
        self.learned = Learned()
        self.rt.bipe_learned_init(c.byref(self.learned), D4(*self.nominal(40.0)))
        self.forward = Forward(self._forward)
        self.handover_s = None
        self.wheel_calls = 0
        self.time = 0.0

    @property
    def feedforward(self):
        return self.classical.feedforward if self.handover_s is not None else np.zeros(4)

    @feedforward.setter
    def feedforward(self, value):
        pass

    @property
    def compensation_limit(self):
        if self.handover_s is not None:
            return self.classical.compensation_limit
        return self.gains["max_command_acceleration"] if self.learned.command[0] != 0 else 0.0

    def _forward(self, observation, action):
        if self.fault_s is not None and self.time >= self.fault_s:
            out = np.full(6, np.nan)
        else:
            out = self.actor(np.ctypeslib.as_array(observation, (25,)).astype(float))
        for i in range(6):
            action[i] = float(out[i])

    def _run_forward(self):
        if self.rt.bipe_learned_forward(c.byref(self.learned), self.forward) == BAD_OUTPUT:
            self.handover_s = self.time

    def command(self, s, velocity, yaw_rate, qref, qdref, dt):
        if self.handover_s is None and self.classical.fall_settings is not None:
            self.classical.previous_request = np.r_[np.zeros(4), self.wheel_request]
            protective = self.classical.protect(s, dt)
            if self.classical.protection.latched:
                return protective, s["pitch"], self.learned.command[0], s["yaw"]
        if self.handover_s is None and self.deferred and self.learned.due:
            self._run_forward()  # the previous tick's pass, after its commands went out
        if self.handover_s is None:
            if "encoders_raw" not in s:
                raise ValueError("the policy observes the sensor model; ideal sensing has none")
            sensors = Sensors(
                (c.c_double * 3)(*s["gyro_raw"]), (c.c_double * 3)(*s["accel_raw"]),
                D4(*s["encoders_raw"][:4]), D4(*s["qd"]), D2(*s["wheel_speed"]),
            )  # fmt: skip
            status = self.rt.bipe_learned_tick(
                c.byref(self.learned), c.byref(sensors), velocity, yaw_rate, D4(*qref)
            )
            if status != OK:
                raise ValueError("learned controller input not finite")
            if not self.deferred and self.learned.due:
                self._run_forward()
        self.time += dt
        self.ticks += 1
        if self.handover_s is not None:
            return self.classical.command(s, velocity, yaw_rate, qref, qdref, dt)
        self.value = np.array(self.learned.command[:])
        self.action = np.array(self.learned.action[:], float)
        self.leg_target = np.array(self.learned.leg_target[:])
        self.rt.bipe_learned_legs(c.byref(self.learned), D4())
        self.leg_output = np.array(self.learned.leg_output[:])
        self.wheel_target = np.array(self.learned.wheel_target[:])
        return np.r_[np.zeros(4), self.wheel_request], s["pitch"], self.value[0], s["yaw"]

    def leg_torques(self, s, qref, qdref, feedforward=None):
        if self.handover_s is not None:
            return self.classical.leg_torques(s, qref, qdref, feedforward)
        return super().leg_torques(s, qref, qdref, feedforward)

    def wheel_torques(self, d, dt):
        if self.handover_s is not None or self.classical.protection.latched:
            return self.classical.previous_request[4:]
        calls, self.wheel_calls = self.wheel_calls, self.wheel_calls + 1
        speed = D2(*(d.joint(w).qvel[0] for w in ("left_wheel_joint", "right_wheel_joint")))
        if self.wheel_tick and calls % self.policy_ticks_per_control:
            if self.wheel_tick != "driver":
                return self.wheel_request  # held between 5 ms ticks
            # The drive's own 1 kHz velocity term (kd = kv) around the Teensy's held
            # integral feedforward.
            error = np.array(self.learned.wheel_target[:]) - np.array(speed[:])
            torque = self.cfg.wheel_kv * error + self.cfg.wheel_ki * np.array(
                self.learned.wheel_integral[:]
            )
            self.wheel_request = np.clip(torque, -self.wheel_peak, self.wheel_peak)
            return self.wheel_request
        torque = D2()
        step = dt * self.policy_ticks_per_control if self.wheel_tick else dt
        if not self.rt.bipe_learned_wheels(c.byref(self.learned), speed, step, torque):
            raise ValueError("wheel speed loop input not finite")
        self.wheel_request = np.array(torque[:])
        return self.wheel_request

    wheel_peak = 0.495  # locomotion_config.h BIPE_LEARNED_WHEEL_PEAK
    policy_ticks_per_control = 5  # 1 ms servo steps in the 5 ms control tick


VARIANTS = {
    "inline": {"deferred": False, "wheel_tick": False},
    "hardware": {"deferred": True, "wheel_tick": "driver"},
    "deferred": {"deferred": True, "wheel_tick": False},
    "wheel_tick": {"deferred": False, "wheel_tick": True},
    "deferred_tick": {"deferred": True, "wheel_tick": True},
    "fault_inline": {"deferred": False, "wheel_tick": False},
    "fault": {"deferred": True, "wheel_tick": "driver"},
    "fault_deferred_tick": {"deferred": True, "wheel_tick": True},
}


def runtime_in_loop(runner, actor, include=None, **options):
    """The runner's simulate() runs RuntimeController (as policy_control.policy_in_loop)."""
    from .. import moving_experiment

    if multiprocessing.get_start_method() != "fork":
        raise RuntimeError("runtime_in_loop needs the fork start method for the runners' pools")
    counted, lib = Counted(actor), build(include=include)
    original = runner.simulate

    def simulate(*args, **kwargs):
        start = counted.calls
        fc.FirmwareController = partial(RuntimeController, actor=counted, runtime=lib, **options)
        fc.FirmwareEstimator = PolicyEstimator
        try:
            run = moving_experiment.simulate(*args, **kwargs)
        finally:
            from . import policy_control

            fc.FirmwareController = policy_control.Classical
            fc.FirmwareEstimator = policy_control.ClassicalEstimator
        run["policy_steps"] = counted.calls - start
        latest = PolicyController.latest
        run["handover_s"] = getattr(latest, "handover_s", None)
        if hasattr(latest, "learned"):
            run["leg_clamps"] = latest.learned.leg_clamps
            run["wheel_clamps"] = latest.learned.wheel_clamps
        return run

    runner.simulate = simulate
    return original


def with_runtime_record(physical_metrics):
    def wrapped(run):
        report = physical_metrics(run)
        for key in ("policy_steps", "handover_s", "leg_clamps", "wheel_clamps"):
            report[key] = run.get(key)
        return report

    return wrapped


def main():
    from .full_plant import with_joint_exemption

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("suite", choices=["acceptance", "speed"])
    parser.add_argument("--variant", choices=sorted(VARIANTS), required=True)
    parser.add_argument("--fault-s", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--resume", action="store_true", help="acceptance: continue saved cases")
    parser.add_argument("--checkpoint", type=lambda s: Path(s).resolve(), default=CHECKPOINT)
    args = parser.parse_args()
    checkpoint = args.checkpoint
    if checkpoint != CHECKPOINT and not args.output:
        parser.error("--checkpoint needs --output (artifacts/m13/handover holds settle's records)")
    output = args.output or ROOT / f"artifacts/m13/handover/{args.variant}_{args.suite}"
    output.mkdir(parents=True, exist_ok=True)
    options = dict(
        VARIANTS[args.variant], fault_s=args.fault_s if args.variant.startswith("fault") else None
    )
    from .headers import CHECKPOINT as DEPLOYED  # the default build's headers carry this one

    include, library = checkpoint_build(checkpoint) if checkpoint != DEPLOYED else (None, None)
    record = {
        "checkpoint": str(
            checkpoint.relative_to(ROOT) if checkpoint.is_relative_to(ROOT) else checkpoint
        ),
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "runtime": "firmware/src/locomotion/learned.c",
        "variant": args.variant,
        **options,
        "leg_filter": compiled_leg_filter(),
        "suite": args.suite,
        "cases": args.case or "all",
    }
    (output / "handover.json").write_text(json.dumps(record, indent=2) + "\n")
    if args.suite == "acceptance":
        from .. import robustness as runner

        argv = ["--profile", "m10", "--controllers", "current", "--phase", "all"]
        argv += ["--resume"] if args.resume else []
    else:
        from ..m12 import speed as runner

        argv = []
    sys.argv = [runner.__name__, *argv, "--output", str(output), "--workers", str(args.workers)]
    sys.argv += [x for name in args.case for x in ("--case", name)]
    runner.metrics = with_joint_exemption(runner.metrics)
    runner.physical_metrics = with_runtime_record(runner.physical_metrics)
    actor = CActor(library)
    assert actor.checkpoint_sha256 == record["checkpoint_sha256"], "rebuild liblocomotion.so"
    multiprocessing.set_start_method("fork", force=True)
    original = runtime_in_loop(runner, actor, include=include, **options)
    try:
        runner.main()
    finally:
        runner.simulate = original


if __name__ == "__main__":
    main()
