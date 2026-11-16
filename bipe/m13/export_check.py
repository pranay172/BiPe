"""M13 phase D: parity of the exported policy, and its Teensy 4.0 inference budget.

    python -m bipe.m13.export_check record  [--policy NAME] [--case NAME ...]
    python -m bipe.m13.export_check parity  [--policy NAME]
    python -m bipe.m13.export_check budget  [--policy NAME]
    python -m bipe.m13.export_check closed  [--policy NAME]

``--policy`` names a headers.POLICIES entry (default the deployed one); its files go to
that policy's export directory. The headers and the ARM image must be built from the
same policy (``python -m bipe.m13.export --policy NAME``, then make).

Needs ``python -m bipe.m13.export``, ``make -C firmware locomotion locomotion-arm``
(the ARM compiler extracted into .local-tools/root, as ``make arm`` uses) and
onnxruntime and unicorn in the venv.

- record: the observations the numpy actor saw on full-plant M10 acceptance cases
  (bipe.m13.full_plant's runner), to the policy's export directory (observations.npz).
- parity: the numpy actor that phase C ran (float64), torch, ONNX Runtime, the host C
  forward pass through ctypes and the Cortex-M7 image run instruction by instruction
  under unicorn, on the recorded observations plus random ones drawn uniformly over
  each channel's recorded range; parity.json.
- budget: the Cortex-M7 image's instruction count per call (from the emulation) and
  memory, turned into a time range at 600 MHz with the cycle model stated in
  budget.json, against the M11 cycle budget and the 20 ms policy period.
- closed: the M10 acceptance cases rerun with the C forward pass in the loop
  (``bipe.m13.full_plant acceptance --forward c``, the policy's ``c_acceptance``)
  against its numpy-actor run (``numpy_acceptance``), verdict by verdict and in phase
  A's metric bands (bipe.m13.validate.BANDS), with its numpy reruns (``reruns``)
  showing whether the reference reproduces; closed_loop.json. Paths: headers.POLICIES.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import shutil
import subprocess
import sys

import numpy as np

from ..parameters import ROOT
from .export import DEPLOYED, POLICIES, sha256
from .policy_control import Actor, CActor, policy_in_loop

POLICY = DEPLOYED
CHECKPOINT = ROOT / POLICIES[POLICY]["checkpoint"]
OUT = ROOT / POLICIES[POLICY]["export"]
RECORD = OUT / "observations.npz"


def select(name):
    global POLICY, CHECKPOINT, OUT, RECORD
    POLICY, CHECKPOINT = name, ROOT / POLICIES[name]["checkpoint"]
    OUT = ROOT / POLICIES[name]["export"]
    RECORD = OUT / "observations.npz"
ARM = ROOT / ".local-tools/root/usr/bin"
IMAGE = ROOT / "firmware/build/arm/locomotion"
BASE = 0x10000
TOLERANCE = 1e-4
CASES = ["push_drive_+2.5", "sensors_combined", "belt_hold_h0.45_n0.9_k1", "backlash_turn_0.6"]
CLOCK_HZ = 600e6
# M11 timing budget (docs/milestones/11-*.md, artifacts/m11/.../timing.json)
COMPUTE_BOUND_S, MARGIN_S, POLICY_PERIOD_S = 0.5e-3, 1.56e-3, 0.02


class Recorder:
    """The numpy actor, appending what it sees to a file per process (pool workers are forked)."""

    def __init__(self, actor, directory):
        self.actor, self.directory = actor, directory

    def __call__(self, obs):
        with open(self.directory / f"{os.getpid()}.f64", "ab") as f:
            f.write(np.asarray(obs, float).tobytes())
        return self.actor(obs)


def record(cases):
    from .. import robustness as runner

    parts = OUT / "observations"
    parts.mkdir(parents=True, exist_ok=True)
    for f in parts.glob("*.f64"):
        f.unlink()
    sys.argv = [runner.__name__, "--profile", "m10", "--controllers", "current", "--phase", "all"]
    sys.argv += ["--output", str(parts / "run"), "--workers", str(len(cases))]
    sys.argv += [x for name in cases for x in ("--case", name)]
    with policy_in_loop(
        CHECKPOINT, runners=(runner.__name__,), actor=Recorder(Actor(CHECKPOINT), parts)
    ):
        runner.main()
    obs = np.concatenate([np.fromfile(f).reshape(-1, 25) for f in sorted(parts.glob("*.f64"))])
    np.savez_compressed(RECORD, observations=obs, cases=np.array(cases))
    shutil.rmtree(parts)
    print(json.dumps({"observations": len(obs), "cases": cases}))


class Cortex:
    """The Cortex-M7 image of the forward pass under unicorn (instruction accurate, untimed)."""

    OBS, ACT, STACK, STOP = 0x80000, 0x80100, 0xF0000, 0x90000

    def __init__(self, count=False):
        from unicorn import UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB, Uc

        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0, 0x100000)
        self.image = (IMAGE.with_suffix(".bin")).read_bytes()
        self.uc.mem_write(self.STOP, b"\x00\xbf\x00\xbf")  # nop; emulation stops here
        self.symbols = symbols()
        self.counts = None
        if count:
            self.counts = {}
            self.uc.hook_add(
                UC_HOOK_CODE,
                lambda uc, a, s, u: self.counts.__setitem__(a, self.counts.get(a, 0) + 1),
            )

    def __call__(self, obs):
        from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP

        self.uc.mem_write(BASE, self.image)  # fresh .data and .bss each call
        self.uc.mem_write(self.OBS, np.asarray(obs, np.float32).tobytes())
        for reg, value in ((UC_ARM_REG_SP, self.STACK), (UC_ARM_REG_R0, self.OBS)):
            self.uc.reg_write(reg, value)
        self.uc.reg_write(UC_ARM_REG_R1, self.ACT)
        self.uc.reg_write(UC_ARM_REG_LR, self.STOP | 1)
        self.uc.emu_start(self.symbols["bipe_locomotion_forward"][0] | 1, self.STOP)
        return np.frombuffer(self.uc.mem_read(self.ACT, 24), np.float32).astype(float)


def symbols():
    """Function symbols of the ARM image: name -> (start, end)."""
    out = subprocess.run([ARM / "arm-none-eabi-nm", "-S", "--defined-only", IMAGE.with_suffix(".elf")],
                         capture_output=True, text=True, check=True).stdout  # fmt: skip
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in "tT":
            table[parts[3]] = (int(parts[0], 16), int(parts[0], 16) + int(parts[1], 16))
    return table


def observations():
    recorded = np.load(RECORD)["observations"]
    rng = np.random.default_rng(20261029)
    lo, hi = recorded.min(0), recorded.max(0)
    return recorded, rng.uniform(lo, hi, size=(2000, recorded.shape[1]))


def parity():
    import onnxruntime as ort
    import torch

    from .export import torch_actor

    recorded, random = observations()
    numpy_actor, c_actor, cortex = Actor(CHECKPOINT), CActor(), Cortex()
    module = torch_actor(CHECKPOINT)
    session = ort.InferenceSession(str(OUT / "policy.onnx"), providers=["CPUExecutionProvider"])
    assert c_actor.checkpoint_sha256 == sha256(CHECKPOINT)
    arm_rows = np.r_[
        np.arange(0, len(recorded), max(1, len(recorded) // 500)), len(recorded) + np.arange(500)
    ]
    report = {
        "tolerance": TOLERANCE,
        "reference": "numpy actor (float64), as phase C ran",
        "sets": {},
    }
    for name, obs in (("recorded", recorded), ("random", random)):
        ref = np.array([numpy_actor(o) for o in obs])
        x = obs.astype(np.float32)
        with torch.no_grad():
            runs = {"torch": module(torch.from_numpy(x)).numpy()}
        runs["onnxruntime"] = np.concatenate(
            [session.run(None, {"observation": o[None]})[0] for o in x]
        )
        runs["c_host"] = np.array([c_actor(o) for o in x])
        rows = (
            arm_rows[arm_rows < len(obs)]
            if name == "recorded"
            else arm_rows[arm_rows >= len(recorded)] - len(recorded)
        )
        runs["c_cortex_m7"] = np.array([cortex(o) for o in x[rows]])
        entry = {
            "observations": len(obs),
            "action_abs_range": float(np.abs(ref).max()),
            "max_abs_error": {},
        }
        for k, v in runs.items():
            r = ref[rows] if k == "c_cortex_m7" else ref
            entry["max_abs_error"][k] = float(np.abs(v - r).max())
        entry["c_cortex_m7_observations"] = len(rows)
        entry["c_cortex_m7_vs_c_host_max_abs"] = float(
            np.abs(runs["c_cortex_m7"] - runs["c_host"][rows]).max()
        )
        entry["passed"] = all(e <= TOLERANCE for e in entry["max_abs_error"].values())
        report["sets"][name] = entry
    report["passed"] = all(s["passed"] for s in report["sets"].values())
    report["recorded_from"] = {
        "cases": np.load(RECORD)["cases"].tolist(),
        "runner": "bipe.robustness --profile m10",
    }
    report["identity"] = {"checkpoint_sha256": sha256(CHECKPOINT), "onnx_sha256": sha256(OUT / "policy.onnx"),
                          "arm_image_sha256": sha256(IMAGE.with_suffix(".elf"))}  # fmt: skip
    (OUT / "parity.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


def budget():
    from .export import load

    recorded, random = observations()
    cortex = Cortex(count=True)
    table = cortex.symbols
    forward = table["bipe_locomotion_forward"]
    inner = mac_loop(forward)
    picks = np.r_[recorded[:: max(1, len(recorded) // 100)], random[:20]]
    calls = []
    for o in picks:
        cortex.counts = {}
        cortex(o)
        total = sum(cortex.counts.values())
        loop = sum(n for a, n in cortex.counts.items() if a in inner)
        elu = sum(n for a, n in cortex.counts.items() if not forward[0] <= a < forward[1])
        calls.append((total, loop, elu))
    calls = np.array(calls)
    worst = calls[calls[:, 0].argmax()]
    _, _, layers = load(CHECKPOINT)
    macs = int(sum(w.size for w, _ in layers))
    neurons = int(sum(w.shape[0] for w, _ in layers[:-1]))
    # Cycle model for the Cortex-M7 (dual issue, 3-cycle FP add latency, weights in DTCM
    # with no wait states): the MAC loop is 6 instructions per MAC carrying one
    # accumulator, so it is bound by the add chain at best (3 cycles a MAC) and by
    # single issue with a load-use stall at worst (8 cycles a MAC); everything else,
    # expm1f included, at 1 to 2 cycles an instruction.
    best = 3 * worst[1] / 6 + 1 * (worst[0] - worst[1])
    slow = 8 * worst[1] / 6 + 2 * (worst[0] - worst[1])
    size = subprocess.run([ARM / "arm-none-eabi-size", "-A", IMAGE.with_suffix(".elf")],
                          capture_output=True, text=True, check=True).stdout  # fmt: skip
    sections = {
        p[0]: int(p[1])
        for p in (l.split() for l in size.splitlines())
        if len(p) == 3
        and p[0].startswith(".")
        and not p[0].startswith((".debug", ".comment", ".ARM"))
    }
    report = {
        "scope": "estimate from the ARM build run under an instruction-accurate emulator; the Teensy's DWT cycle counter replaces it at the bench check",
        "network": {"multiply_accumulates": macs, "elu_neurons": neurons, "parameters_bytes": 4 * (macs + neurons + 6 + 2 * 25)},
        "build": {
            "compiler": subprocess.run([ARM / "arm-none-eabi-gcc", "--version"], capture_output=True, text=True, check=True).stdout.splitlines()[0],
            "flags": "-std=c11 -O2 -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard; newlib libm expm1f",
            "note": "Teensy's PlatformIO toolchain (gcc 11.3) was not reachable from the build host; the Debian arm-none-eabi gcc 13.2 stands in",
            "sections_bytes": sections,
            "stack_bytes": 4 * (25 + 256 + 128 + 64) + 64,
            "memory_note": "the weights are const: Teensy 4.0's linker copies them to DTCM (RAM1, 512 KB shared with ITCM) unless marked PROGMEM, leaving them in QSPI flash behind the 32 KB cache",
        },
        "instructions_per_call": {
            "calls_measured": len(calls),
            "max": int(calls[:, 0].max()), "mean": float(calls[:, 0].mean()), "min": int(calls[:, 0].min()),
            "mac_loop_at_max": int(worst[1]), "expm1f_at_max": int(worst[2]),
        },
        "cycle_model": {
            "mac_loop_cycles_per_mac": [3, 8],
            "other_cycles_per_instruction": [1, 2],
            "basis": "Cortex-M7 dual issue and FP add/multiply latency (ARM Cortex-M7 TRM); one accumulator chain per output; DTCM zero wait state",
        },
        "time_s": {"best": best / CLOCK_HZ, "worst": slow / CLOCK_HZ, "clock_Hz": CLOCK_HZ},
        "against": {
            "m11_compute_bound_s": COMPUTE_BOUND_S, "m11_margin_s": MARGIN_S, "policy_period_s": POLICY_PERIOD_S,
            "worst_over_compute_bound": slow / CLOCK_HZ / COMPUTE_BOUND_S,
            "worst_fits_in_tick_margin": bool(slow / CLOCK_HZ <= MARGIN_S),
            "worst_fraction_of_policy_period": slow / CLOCK_HZ / POLICY_PERIOD_S,
        },
        "identity": {"checkpoint_sha256": sha256(CHECKPOINT), "arm_image_sha256": sha256(IMAGE.with_suffix(".elf"))},
    }  # fmt: skip
    (OUT / "budget.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


def mac_loop(forward):
    """Addresses of the innermost loops (backward branch over at most 8 instructions)."""
    out = subprocess.run([ARM / "arm-none-eabi-objdump", "-d", f"--start-address={forward[0]}",
                          f"--stop-address={forward[1]}", IMAGE.with_suffix(".elf")],
                         capture_output=True, text=True, check=True).stdout  # fmt: skip
    rows = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) >= 3 and parts[0].strip().endswith(":"):
            rows.append(
                (
                    int(parts[0].strip()[:-1], 16),
                    parts[2].strip(),
                    parts[3].strip() if len(parts) > 3 else "",
                )
            )
    loops = set()
    for i, (addr, op, arg) in enumerate(rows):
        if op.startswith("bne"):
            target = int(arg.split()[0], 16)
            body = [a for a, _, _ in rows if target <= a <= addr]
            if (
                target < addr
                and len(body) <= 8
                and any(o.startswith("vmul") for a, o, _ in rows if target <= a <= addr)
            ):
                loops.update(body)
    return loops


def closed():
    from .validate import BANDS, within

    entry = POLICIES[POLICY]
    phase_c = ROOT / entry["numpy_acceptance"] / "summary.json"
    phase_d = ROOT / entry["c_acceptance"] / "summary.json"
    reruns = [ROOT / d / "summary.json" for d in entry["reruns"]]
    cases = lambda f: json.loads(f.read_text())["controllers"]["current"]["cases"]
    ref, c = cases(phase_c), cases(phase_d)
    verdicts = ["passed", "passed_without_joint_error", "physical_pass", "recovered"]
    rows, flipped, out_of_band = {}, [], []
    for name, case in sorted(c.items()):
        a, b = ref[name]["metrics"], case["metrics"]
        if a.get("failure", "").startswith("controller rejected") if a.get("failure") else False:
            continue  # ideal sensing: the policy needs the sensor model
        row = {"flips": {k: [a.get(k), b.get(k)] for k in verdicts if a.get(k) != b.get(k)}}
        row["bands"] = {k: within(b.get(k), a.get(k), *band) for k, band in BANDS.items()}
        row["late"] = {
            k: [a.get(k), b.get(k)] for k in ("late_pitch_peak_deg", "late_speed_peak_m_s")
        }
        rows[name] = row
        flipped += [name] if row["flips"] else []
        out_of_band += [name] if not all(v for v in row["bands"].values() if v is not None) else []
    deterministic = {}
    for rerun in reruns:
        for name, case in cases(rerun).items():
            drop = lambda m: {k: v for k, v in m.items() if k != "policy_steps"}
            deterministic[name] = drop(case["metrics"]) == drop(ref[name]["metrics"])
    report = {
        "reference": str(phase_c.relative_to(ROOT)),
        "c_forward_pass": str(phase_d.relative_to(ROOT)),
        "complete": json.loads(phase_d.read_text()).get("status") == "complete",
        "cases": len(rows),
        "same_verdicts": len(rows) - len(flipped),
        "flipped": {n: rows[n]["flips"] for n in flipped},
        "flipped_late_rest": {n: rows[n]["late"] for n in flipped},
        "within_bands": len(rows) - len(out_of_band),
        "out_of_band": {n: rows[n]["bands"] for n in out_of_band},
        "bands": BANDS,
        "numpy_rerun_reproduces_phase_c": deterministic,
        "max_abs_difference": {
            k: max(abs(c[n]["metrics"][k] - ref[n]["metrics"][k]) for n in rows if ref[n]["metrics"].get(k) is not None)
            for k in [*BANDS, "late_pitch_peak_deg", "late_speed_peak_m_s"]
        },
    }  # fmt: skip
    (OUT / "closed_loop.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=["record", "parity", "budget", "closed"])
    parser.add_argument("--case", action="append")
    parser.add_argument("--policy", choices=sorted(POLICIES), default=DEPLOYED)
    args = parser.parse_args()
    select(args.policy)
    multiprocessing.set_start_method("fork", force=True)  # see policy_in_loop
    {
        "record": lambda: record(args.case or CASES),
        "parity": parity,
        "budget": budget,
        "closed": closed,
    }[args.command]()


if __name__ == "__main__":
    main()
