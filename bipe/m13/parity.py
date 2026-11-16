"""M13 phase A: record the training plant for the MuJoCo Warp parity check.

Runs one closed-loop case on the training plant in CPU MuJoCo (the classical C controller
with the sensor model) and saves the MJCF, every physics step's state, controls and
hinge frictionloss to artifacts/m13/parity/<case>.npz. ``tools/m13_warp_parity.py`` then
replays windows of it under MuJoCo Warp and CPU MuJoCo in the RL environment.
"""

import argparse
from pathlib import Path

import numpy as np

from ..parameters import ROOT
from . import plant as tp
from .validate import acceptance_cases, load, run_acceptance

OUTPUT = ROOT / "artifacts/m13/parity"


class Recorder:
    def __init__(self):
        self.rows = []

    def __call__(self, t, dt, drives, d):
        m = drives.model
        self.rows.append(
            (t, d.qpos.copy(), d.qvel.copy(), d.ctrl.copy(), m.dof_frictionloss.copy())
        )


def record(name, output=OUTPUT):
    from ..profiles import native
    from ..robustness_control import build_candidate

    p10, p13, gains, sensors = load()
    library = build_candidate(p10, gains, ROOT / native(p10)["firmware_build"])
    case = acceptance_cases(p10, p13, sensors)[name]
    recorder = Recorder()
    if case["family"] == "acceptance":
        case = {**case, "job": {**case["job"], "observer": recorder}}
        run = run_acceptance(p10, p13, gains, sensors, library, case)
    else:
        from functools import partial

        from ..moving_experiment import simulate
        from ..robustness import command, plant_variant

        run = simulate(
            case["mode"],
            case["duration"],
            sensing="sensors",
            nominal=p10,
            plant=plant_variant(p13, case.get("variation", {})),
            gains=gains,
            sensor_config=sensors,
            implementation="c",
            firmware_library=library,
            disturbance=case.get("disturbance", {}),
            command_function=partial(command, case=case, nominal=p10),
            observer=recorder,
        )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows = recorder.rows
    np.savez_compressed(
        output / f"{name}.npz",
        xml=np.array(tp.generate_xml(p13)),
        time=np.array([r[0] for r in rows]),
        qpos=np.array([r[1] for r in rows]),
        qvel=np.array([r[2] for r in rows]),
        ctrl=np.array([r[3] for r in rows]),
        frictionloss=np.array([r[4] for r in rows]),
        failure=np.array(str(run["failure"])),
    )
    return output / f"{name}.npz"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--case", default="push_drive_+2")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    print(record(args.case, args.output))


if __name__ == "__main__":
    main()
