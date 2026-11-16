"""M13 phase D: the policy as ONNX with a manifest, and the C forward pass's weights.

    .venv/rl/bin/python -m bipe.m13.export [--policy NAME] [--checkpoint PATH] [--output DIR]

Writes to the policy's export directory (headers.POLICIES; the deployed one by default,
M14's ``locomotion`` since 3 November 2026; settle's is artifacts/m13/export/settle):
- policy.onnx: the actor as trained, observation normaliser included, so the graph
  takes the raw 25-value observation and returns the 6 mean actions (float32,
  opset 17, batch of one);
- manifest.json: the contract around the graph, from the code that ran in phase C
  (bipe.m13.policy_control): observation layout, action law, timing, posture table,
  identity;
- SHA256SUMS.
and firmware/include/generated_locomotion_weights.h for firmware/src/locomotion, with
firmware/include/generated_locomotion_parity.h, the M11 image's bench parity vectors
(generated, not committed, like generated_config.h; bipe.m13.headers writes them alone
without torch).
"""

from __future__ import annotations

import argparse
import json
import subprocess

import numpy as np

from ..parameters import ROOT, load_parameters
from .headers import (  # noqa: F401 (the headers' names, as before)
    CHECKPOINT,
    DEPLOYED,
    EPS,
    LAYERS,
    PARITY_HEADER,
    POLICIES,
    WEIGHTS_HEADER,
    load,
    parity_header,
    policy_of,
    sha256,
    weights_header,
)
from .policy_control import LABELS, LEG_FILTER_BAND_RAD, LEG_FILTER_TAU_S
from .rl.firmware import DriveActionCfg, posture_table
from .rl.mdp import DriveCommandCfg

OBSERVATION = [
    ("gyro", 3, "rad/s", "body rates from the latest delayed IMU packet"),
    ("accelerometer", 3, "m/s^2", "specific force from the latest delayed IMU packet"),
    ("leg_encoders", 4, "rad", "[hip, knee] x [left, right] encoder angles less nominal(posture)"),
    ("leg_speeds", 4, "rad/s", "estimator's encoder-derived leg joint speeds"),
    ("wheel_speeds", 2, "rad/s", "estimator's wheel speeds [left, right]"),
    ("last_action", 6, "1", "the previous policy output"),
    ("command", 3, "m/s, rad/s, 1", "rate-limited forward speed, yaw rate, (label - 47.5) / 17.5"),
]


def torch_actor(checkpoint):
    """The actor as a float32 torch module: normaliser, then Linear/ELU x 3, Linear."""
    import torch
    from torch import nn

    mean, std, layers = load(checkpoint)

    class Actor(nn.Module):
        def __init__(self):
            super().__init__()
            self.register_buffer("mean", torch.tensor(mean, dtype=torch.float32))
            self.register_buffer("scale", torch.tensor(1 / (std + EPS), dtype=torch.float32))
            modules = []
            for i, (w, b) in enumerate(layers):
                linear = nn.Linear(w.shape[1], w.shape[0])
                linear.weight.data = torch.tensor(w, dtype=torch.float32)
                linear.bias.data = torch.tensor(b, dtype=torch.float32)
                modules.append(linear)
                if i < len(layers) - 1:
                    modules.append(nn.ELU())
            self.mlp = nn.Sequential(*modules)

        def forward(self, obs):
            return self.mlp((obs - self.mean) * self.scale)

    return Actor().eval()


def export_onnx(checkpoint, path):
    import torch

    actor = torch_actor(checkpoint)
    dummy = torch.zeros(1, actor.mean.numel())
    torch.onnx.export(
        actor, (dummy,), str(path), input_names=["observation"], output_names=["action"],
        opset_version=17, dynamo=False,
    )  # fmt: skip


def manifest(checkpoint, onnx_path, policy=None):
    cfg, command = DriveActionCfg(), DriveCommandCfg(resampling_time_range=(3.0, 8.0))
    p10 = load_parameters(profile="m10")
    hub = p10["physics"]["hub_motor"]
    run = checkpoint.parent
    entry = POLICIES[policy or policy_of(checkpoint) or "settle"]
    config = [run / n for n in ("env.yaml", "agent.yaml")]
    if not config[0].exists():
        config = [run / "params" / n for n in ("env.yaml", "agent.yaml")]
    mean, _, layers = load(checkpoint)
    commit = subprocess.run(
        ["git", "log", "-1", "--format=%H", "--", str(checkpoint)], cwd=ROOT,
        capture_output=True, text=True, check=True,
    ).stdout.strip()  # fmt: skip
    offset = 0
    observation = []
    for name, size, unit, meaning in OBSERVATION:
        observation.append(
            {"name": name, "index": [offset, offset + size], "unit": unit, "meaning": meaning}
        )
        offset += size
    assert offset == len(mean), (offset, len(mean))
    return {
        "scope": entry.get("scope", "learned locomotion policy for the M10 robot; validated in simulation under stated assumptions, not on hardware"),
        "graph": {
            "file": onnx_path.name,
            "sha256": sha256(onnx_path),
            "input": {"name": "observation", "shape": [1, len(mean)], "dtype": "float32"},
            "output": {"name": "action", "shape": [1, layers[-1][0].shape[0]], "dtype": "float32"},
            "contains": "observation normaliser (x - mean) / (std + 0.01), then Linear-ELU 256, 128, 64, Linear 6; deterministic mean action",
            "multiply_accumulates": int(sum(w.size for w, _ in layers)),
            "parameters": int(sum(w.size + b.size for w, b in layers) + 2 * len(mean)),
        },
        "observation": observation,
        "action": {
            "legs": {
                "index": [0, 4],
                "law": "leg target [hip, knee] x [left, right] = nominal(posture) + leg_scale * action[0:4], rad",
                "leg_scale_rad": cfg.leg_scale,
                "target_filter": {
                    "rate_Hz": 200,
                    "law": "every control tick, output += dt / (tau + dt) * (target - output), then output clamped to target +- band; the servo follows output",
                    "tau_s": LEG_FILTER_TAU_S,
                    "band_rad": LEG_FILTER_BAND_RAD,
                    "basis": "M13 belt range follow-up: tau 0 is off (as trained); 0.03 s recovers the soft knee belt corners but loses speed cases, so it is off",
                },
                "servo": {
                    "rate_Hz": 1000,
                    "law": "torque = kp * (target - q) - kd * qd on the local encoder and its filtered rate",
                    "kp_Nm_per_rad": cfg.joint_kp,
                    "kd_Nm_s_per_rad": cfg.joint_kd,
                },
            },
            "wheels": {
                "index": [4, 6],
                "law": "wheel speed target = wheel_scale * action[4:6] + rolling feedforward [v/r - w*t/(2r), v/r + w*t/(2r)], rad/s",
                "wheel_scale_rad_s": cfg.wheel_scale,
                "wheel_radius_m": p10["geometry"]["wheel_radius"],
                "wheel_track_m": p10["geometry"]["wheel_track"],
                "speed_loop": {
                    "rate_Hz": 1000,
                    "law": "torque = kv * e + ki * integral(e), integral clamped to +-peak_torque / ki; e on the hub's own speed",
                    "kv_Nm_s_per_rad": cfg.wheel_kv,
                    "ki_Nm_per_rad": cfg.wheel_ki,
                    "peak_torque_Nm": hub["torque_constant_Nm_A"] * hub["peak_current_A"],
                },
            },
        },
        "timing": {
            "policy_period_s": 0.02,
            "control_tick_s": 0.005,
            "policy_every_ticks": 4,
            "hold": "the action is held for the policy period",
        },
        "command": {
            "rate_limit_per_s": {
                "speed_m_s": command.acceleration,
                "yaw_rate_rad_s": None,
                "posture_label": command.posture_rate,
            },
            "trained_range": {
                "speed_m_s": list(command.speed_range),
                "yaw_rate_rad_s": list(command.yaw_rate_range),
                "posture_label": [float(LABELS[0]), float(LABELS[-1])],
            },
        },
        "posture_table": {
            "labels": [float(x) for x in LABELS],
            "nominal_rad": np.round(posture_table(p10, LABELS), 9).tolist(),
            "basis": "M10 kinematics (bipe.m10.kinematics.posture) on the m10 parameters, as phase C ran; interpolated linearly between labels",
        },
        "supervision": entry.get("supervision", "runs inside the C supervisor's gate and protective fall, which act on the classical estimator's state (phase C)"),
        **({"requests": entry["requests"]} if "requests" in entry else {}),
        "identity": {
            "checkpoint": str(checkpoint.relative_to(ROOT)),
            "checkpoint_sha256": sha256(checkpoint),
            "checkpoint_commit": commit,
            "variant": entry["variant"],
            "training_config": [str(c.relative_to(ROOT)) for c in config],
            **({"base_training_config": entry["base_config"]} if "base_config" in entry else {}),
            "training_plant": entry["training_plant"],
            "training_plant_sha256": sha256(ROOT / entry["training_plant"]),
            "sensors": "config/sensors-m10.json",
            "sensors_sha256": sha256(ROOT / "config/sensors-m10.json"),
            "full_plant_records": entry["records"],
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--policy", choices=sorted(POLICIES), default=DEPLOYED)
    parser.add_argument("--checkpoint", type=lambda s: (ROOT / s).resolve())
    parser.add_argument("--output", type=lambda s: (ROOT / s).resolve())
    args = parser.parse_args()
    args.checkpoint = args.checkpoint or ROOT / POLICIES[args.policy]["checkpoint"]
    out = args.output or ROOT / POLICIES[args.policy]["export"]
    out.mkdir(parents=True, exist_ok=True)
    onnx_path = out / "policy.onnx"
    export_onnx(args.checkpoint, onnx_path)
    weights_header(args.checkpoint, WEIGHTS_HEADER)
    # The recorded observations live in the policy's export directory, whatever --output.
    known = policy_of(args.checkpoint)
    observations = [out / "observations.npz"]
    if known:
        observations.append(ROOT / POLICIES[known]["export"] / "observations.npz")
    found = next((o for o in observations if o.exists()), None)
    if found:
        parity_header(args.checkpoint, PARITY_HEADER, found)
    else:
        # Another policy's vectors beside these weights would fail the bench parity check.
        PARITY_HEADER.unlink(missing_ok=True)
        print("no observations.npz yet: run export_check record, then export again for the parity header")
    (out / "manifest.json").write_text(
        json.dumps(manifest(args.checkpoint, onnx_path, args.policy), indent=2) + "\n"
    )
    names = ["manifest.json", "policy.onnx"]
    (out / "SHA256SUMS").write_text("".join(f"{sha256(out / n)}  {n}\n" for n in names))
    print(json.dumps({"output": str(out.relative_to(ROOT)), "files": names}))


if __name__ == "__main__":
    main()
