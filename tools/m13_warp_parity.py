"""Replay a recorded training-plant run under MuJoCo Warp and CPU MuJoCo, window by window.

Run in the RL environment (MuJoCo and MuJoCo Warp, no bipe imports):
    python tools/m13_warp_parity.py artifacts/m13/parity/push_drive_+2.npz --device cpu

Each window starts both simulators at a recorded state and applies the recorded controls
and hinge frictionloss for every physics step. The output is the largest state difference
between Warp and CPU MuJoCo, and between CPU MuJoCo of this version and the recording.
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recording")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--window", type=float, default=0.1)
    parser.add_argument("--windows", type=int, default=20)
    parser.add_argument("--output")
    args = parser.parse_args()
    import mujoco_warp as mjw
    import warp as wp

    wp.config.quiet = True
    z = np.load(args.recording)
    m = mujoco.MjModel.from_xml_string(str(z["xml"]))
    d = mujoco.MjData(m)
    steps = round(args.window / m.opt.timestep)
    total = len(z["time"])
    starts = np.linspace(0, total - steps - 1, args.windows).astype(int)
    out = {
        "mujoco": mujoco.__version__,
        "device": args.device,
        "window_s": args.window,
        "windows": [],
    }
    with wp.ScopedDevice(args.device):
        mw = mjw.put_model(m)
        dw = mjw.put_data(m, d)
        for k in starts:
            d.qpos[:] = z["qpos"][k]
            d.qvel[:] = z["qvel"][k]
            mujoco.mj_forward(m, d)
            dw = mjw.put_data(m, d)
            warp_err = cpu_err = 0.0
            for j in range(steps):
                i = k + j
                m.dof_frictionloss[:] = z["frictionloss"][i]
                mw.dof_frictionloss.assign(
                    z["frictionloss"][i].reshape(mw.dof_frictionloss.shape).astype(np.float32)
                )
                d.ctrl[:] = z["ctrl"][i]
                dw.ctrl.assign(z["ctrl"][i][None].astype(np.float32))
                mujoco.mj_step(m, d)
                mjw.step(mw, dw)
                q = dw.qpos.numpy()[0]
                warp_err = max(warp_err, float(np.abs(q - d.qpos).max()))
                cpu_err = max(cpu_err, float(np.abs(d.qpos - z["qpos"][i + 1]).max()))
            out["windows"].append(
                {
                    "start_s": float(z["time"][k]),
                    "warp_vs_cpu": warp_err,
                    "cpu_vs_recording": cpu_err,
                }
            )
        # Throughput: steps per second for a batch of worlds at the rest state.
        for n in (1, 64) if args.device == "cpu" else (1024, 4096, 8192):
            d.qpos[:] = z["qpos"][0]
            d.qvel[:] = z["qvel"][0]
            mujoco.mj_forward(m, d)
            dn = mjw.put_data(m, d, nworld=n)
            mjw.step(mw, dn)
            wp.synchronize()
            count = 50
            t0 = time.perf_counter()
            for _ in range(count):
                mjw.step(mw, dn)
            wp.synchronize()
            out.setdefault("throughput_steps_per_s", {})[n] = n * count / (time.perf_counter() - t0)
    out["max_warp_vs_cpu"] = max(w["warp_vs_cpu"] for w in out["windows"])
    out["max_cpu_vs_recording"] = max(w["cpu_vs_recording"] for w in out["windows"])
    text = json.dumps(out, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
