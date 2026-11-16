"""The C forward pass's generated headers from the checkpoint, without torch.

    .venv/bin/python -m bipe.m13.headers [--checkpoint PATH]

Writes firmware/include/generated_locomotion_weights.h (the weights for
firmware/src/locomotion/locomotion.c) and firmware/include/generated_locomotion_parity.h
(the M11 image's bench parity vectors), generated and not committed, like
generated_config.h. bipe.m13.export writes them too; this module needs only numpy, so the
Teensy image builds without the training stack. The checkpoint is read as the zip of
pickled tensors torch saves (tensors rebuilt from their storages; no code is run).
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import pickle
import zipfile

import numpy as np

from ..parameters import ROOT

# The exported policies: checkpoint, export directory (bipe.m13.export, export_check)
# and the full-plant records the closed-loop check compares against. DEPLOYED is the
# one the firmware's generated headers carry by default (M14, 3 November 2026:
# M14's fine-tuned seed 1 replaces M13's settle).
POLICIES = {
    "settle": {
        "checkpoint": "artifacts/m13/training/settle/model_10493.pt",
        "export": "artifacts/m13/export/settle",
        "variant": "settle",
        "records": ["artifacts/m13/phase_c/settle_acceptance", "artifacts/m13/phase_c/settle_speed",
                    "artifacts/m13/phase_c/settle_gap"],
        "numpy_acceptance": "artifacts/m13/phase_c/settle_acceptance",
        "c_acceptance": "artifacts/m13/phase_d/settle_c_acceptance",
        "reruns": ["artifacts/m13/phase_d/settle_numpy_rerun",
                   "artifacts/m13/phase_d/settle_numpy_rerun_belt"],
        "training_plant": "config/training-plant-m13.json",
    },
    "locomotion": {
        "checkpoint": "artifacts/m14/checkpoints/finetune_s1/model_9998.pt",
        "export": "artifacts/m14/export/locomotion",
        "variant": "locomotion (seed 1, 8000 iterations) + locomotion_ft (2000)",
        "records": ["artifacts/m14/eval/finetune_ft1"],
        "numpy_acceptance": "artifacts/m14/eval/finetune_ft1/acceptance",
        "c_acceptance": "artifacts/m14/export/locomotion/c_acceptance",
        "reruns": ["artifacts/m14/export/locomotion/numpy_rerun"],
        # The fine-tune's own params/ are found beside the checkpoint; the base run's
        # config completes the recipe.
        "base_config": ["artifacts/m14/checkpoints/locomotion_s1/env.yaml",
                        "artifacts/m14/checkpoints/locomotion_s1/agent.yaml"],
        "training_plant": "config/training-plant-m13.json",
    },
    # M15 get-up policy: runs from a fallen state until the handover to "locomotion"
    # (same observation and action law). Not in the firmware image yet.
    "getup": {
        "checkpoint": "artifacts/m15/checkpoints/getup_s1/model_12997.pt",
        "export": "artifacts/m15/export/getup",
        "variant": "getup (3000 iterations from locomotion fine-tune 1)",
        "records": ["artifacts/m15/results/getup_eval_s1_v2"],
        "base_config": ["artifacts/m14/checkpoints/locomotion_s1/env.yaml",
                        "artifacts/m14/checkpoints/locomotion_s1/agent.yaml"],
        "training_plant": "config/training-plant-m13.json",
    },
    # M16 sit/stand: sits on the shell and stands again on the posture slot's request
    # (bipe.intents), then hands over to "locomotion". Not in the firmware image yet.
    "sit_stand": {
        "checkpoint": "artifacts/m16/checkpoints/sit_stand_s1/model_11997.pt",
        "export": "artifacts/m16/export/sit_stand",
        "variant": "sit_stand (2000 iterations from locomotion fine-tune 1)",
        "records": ["artifacts/m16/results/sit_eval_sit_stand_40",
                    "artifacts/m16/results/sit_eval_sit_stand_20",
                    "artifacts/m16/results/stand_eval_sit_stand"],
        "scope": "learned sit and stand policy for the M10 robot; validated in simulation under stated assumptions, not on hardware",
        "supervision": "sits with the C supervisor's limit checks, gate and protective fall off (the shell lands on the floor); stands under the get-up handover rule (quiet estimate, level and still), then hands over to drive (bipe.m16.sit_eval)",
        "requests": {"sit": {"posture_label": 100.0, "normalised": 3.0,
                             "law": "set at once (no rate limit) with speed and yaw rate 0; leg nominal(posture) uses the label clamped to 65; leaving it sets the label back to 40, then the usual rate limit applies",
                             "accepted_by": "policies whose manifest lists it (sit_stand); other policies were never trained on it"}},
        "base_config": ["artifacts/m14/checkpoints/locomotion_s1/env.yaml",
                        "artifacts/m14/checkpoints/locomotion_s1/agent.yaml"],
        "training_plant": "config/training-plant-m13.json",
    },
}  # fmt: skip
DEPLOYED = "locomotion"
CHECKPOINT = ROOT / POLICIES[DEPLOYED]["checkpoint"]
WEIGHTS_HEADER = ROOT / "firmware/include/generated_locomotion_weights.h"
PARITY_HEADER = ROOT / "firmware/include/generated_locomotion_parity.h"
OBSERVATIONS = ROOT / POLICIES[DEPLOYED]["export"] / "observations.npz"


PARITY_VECTORS = 16
LAYERS = ("mlp.0", "mlp.2", "mlp.4", "mlp.6")
EPS = 1e-2  # rsl_rl EmpiricalNormalization: (x - mean) / (std + eps)
STORAGES = {"FloatStorage": np.float32, "DoubleStorage": np.float64,
            "LongStorage": np.int64, "IntStorage": np.int32, "BoolStorage": np.bool_}  # fmt: skip


def policy_of(checkpoint):
    """The POLICIES entry whose checkpoint this is, or None."""
    for name, entry in POLICIES.items():
        if (ROOT / entry["checkpoint"]).resolve() == checkpoint.resolve():
            return name
    return None


def read_checkpoint(path):
    """A torch checkpoint's pickled dict with its tensors as numpy arrays. Only tensor
    rebuilds, storages and OrderedDict are allowed; any other class is refused."""
    archive = zipfile.ZipFile(path)
    prefix = archive.namelist()[0].split("/")[0]

    def tensor(storage, offset, size, stride, *_):
        if not size:
            return storage[offset].copy()
        view = np.lib.stride_tricks.as_strided(
            storage[offset:], shape=size, strides=[s * storage.itemsize for s in stride]
        )
        return np.array(view)

    class Reader(pickle.Unpickler):
        def find_class(self, module, name):
            if (module, name) == ("torch._utils", "_rebuild_tensor_v2"):
                return tensor
            if (module, name) == ("collections", "OrderedDict"):
                return collections.OrderedDict
            if module == "torch" and name in STORAGES:
                return name
            raise pickle.UnpicklingError(f"refused {module}.{name}")

        def persistent_load(self, pid):
            _, kind, key, _, _ = pid
            return np.frombuffer(archive.read(f"{prefix}/data/{key}"), dtype=STORAGES[kind])

    with archive.open(f"{prefix}/data.pkl") as f:
        return Reader(f).load()


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(checkpoint):
    """Normaliser and layers of the checkpoint's actor, float64 numpy."""
    state = read_checkpoint(checkpoint)["actor_state_dict"]
    get = lambda k: np.asarray(state[k], float)
    mean, std = get("obs_normalizer._mean")[0], get("obs_normalizer._std")[0]
    layers = [(get(f"{n}.weight"), get(f"{n}.bias")) for n in LAYERS]
    return mean, std, layers


def weights_header(checkpoint, path):
    """The C forward pass's weights as float32 arrays; w{i} is flat row-major [out][in], a row a line."""
    mean, std, layers = load(checkpoint)
    name = checkpoint.relative_to(ROOT) if checkpoint.is_relative_to(ROOT) else checkpoint
    f = lambda a: ",".join(f"{float(x):.9g}f" for x in np.ravel(a).astype(np.float32))
    sizes = [layers[0][0].shape[1]] + [w.shape[0] for w, _ in layers]
    lines = [
        "/* Generated by bipe/m13/export.py from",
        f"   {name} (sha256 {sha256(checkpoint)}).",
        "   Do not edit. */",
        "#ifndef BIPE_GENERATED_LOCOMOTION_WEIGHTS_H",
        "#define BIPE_GENERATED_LOCOMOTION_WEIGHTS_H",
        f'#define BIPE_LOCOMOTION_CHECKPOINT_SHA256 "{sha256(checkpoint)}"',
        f"#define BIPE_LOCOMOTION_OBS {sizes[0]}",
        f"#define BIPE_LOCOMOTION_ACT {sizes[-1]}",
    ]
    lines += [f"#define BIPE_LOCOMOTION_H{i} {n}" for i, n in enumerate(sizes[1:-1], 1)]
    lines.append(f"static const float bipe_locomotion_mean[{sizes[0]}] = {{{f(mean)}}};")
    lines.append(
        f"static const float bipe_locomotion_scale[{sizes[0]}] = {{{f(1 / (std + EPS))}}};"
    )
    for i, (w, b) in enumerate(layers):
        rows = ",\n".join(f"    {f(row)}" for row in w)
        lines.append(
            f"static const float bipe_locomotion_w{i}[{w.shape[0]} * {w.shape[1]}] = {{\n{rows}}};"
        )
        lines.append(f"static const float bipe_locomotion_b{i}[{b.shape[0]}] = {{{f(b)}}};")
    lines.append("#endif")
    path.write_text("\n".join(lines) + "\n")


def parity_header(checkpoint, path, observations=None):
    """Bench parity vectors: recorded full-plant observations, evenly spaced, with the
    float64 actor's actions, for the M11 image's on-board check against parity.json."""
    if observations is None:
        name = policy_of(checkpoint)
        observations = ROOT / POLICIES[name]["export"] / "observations.npz" if name else OBSERVATIONS
    mean, std, layers = load(checkpoint)
    rows = np.load(observations)["observations"]
    rows = rows[np.linspace(0, len(rows) - 1, PARITY_VECTORS).round().astype(int)]
    rows = rows.astype(np.float32).astype(float)  # the reference sees the stored inputs
    actions = []
    for obs in rows:
        x = (obs - mean) / (std + EPS)
        for i, (w, b) in enumerate(layers):
            x = w @ x + b
            if i < len(layers) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        actions.append(x)

    def literal(x, digits):
        text = f"{float(x):.{digits}g}"
        return text if any(ch in text for ch in ".en") else text + ".0"

    f = lambda a: ",".join(literal(x, 9) + "f" for x in np.ravel(a).astype(np.float32))
    g = lambda a: ",".join(literal(x, 17) for x in np.ravel(a))
    lines = [
        "/* Generated by bipe/m13/export.py from",
        f"   {checkpoint.relative_to(ROOT)} (sha256 {sha256(checkpoint)})",
        f"   and {observations.relative_to(ROOT)}. Do not edit. */",
        "#ifndef BIPE_GENERATED_LOCOMOTION_PARITY_H",
        "#define BIPE_GENERATED_LOCOMOTION_PARITY_H",
        f'#define BIPE_LOCOMOTION_PARITY_CHECKPOINT "{sha256(checkpoint)}"',
        f"#define BIPE_LOCOMOTION_PARITY_VECTORS {len(rows)}",
        "#define BIPE_LOCOMOTION_PARITY_TOLERANCE 1e-4 /* parity.json */",
        f"static const float bipe_locomotion_parity_observation[{len(rows)}][{rows.shape[1]}] = {{",
        ",\n".join(f"    {{{f(r)}}}" for r in rows),
        "};",
        "/* The float64 actor (bipe.m13.policy_control.Actor), the parity reference. */",
        f"static const double bipe_locomotion_parity_action[{len(rows)}][{len(actions[0])}] = {{",
        ",\n".join(f"    {{{g(a)}}}" for a in actions),
        "};",
        "#endif",
    ]
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkpoint", type=lambda s: (ROOT / s).resolve(), default=CHECKPOINT)
    args = parser.parse_args()
    weights_header(args.checkpoint, WEIGHTS_HEADER)
    parity_header(args.checkpoint, PARITY_HEADER)
    print(WEIGHTS_HEADER.relative_to(ROOT), PARITY_HEADER.relative_to(ROOT))


if __name__ == "__main__":
    main()
