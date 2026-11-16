"""Compile explicit C candidates and derive a lag-aware scheduled LQR.

No controller uses simulator state at run time. Linearization is an offline
fixed-posture reduction of MuJoCo's contact dynamics, with the local PD loop
closed, sliding-friction breakaway regularized away and motor lag restored.
"""

import hashlib
import json
import math
import os
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are

from tools.generate_firmware_config import generate

from .articulated import JOINTS, WHEELS, build, truth
from .kinematics import com_and_axle, posture, support_feedforward
from .parameters import ROOT
from .profiles import native


def build_candidate(nominal, gains, directory):
    sources = [
        ROOT / f"firmware/src/{n}.c"
        for n in ["protocol", "supervisor", "balance", "estimator", "kinematics", "fallen"]
    ]
    identity = hashlib.sha256(json.dumps([nominal, gains], sort_keys=True).encode())
    for source in [
        *sources,
        ROOT / "firmware/include/bipe.h",
        ROOT / "firmware/include/fallen.h",
        ROOT / "tools/generate_firmware_config.py",
    ]:
        identity.update(source.read_bytes())
    directory = Path(directory).resolve() / identity.hexdigest()[:16]
    directory.mkdir(parents=True, exist_ok=True)
    header = directory / "generated_config.h"
    lib = directory / "libbipe.so"
    record_path = lib.with_suffix(".json")
    # Parallel workers each call this: reuse a finished build, and otherwise write every
    # file under a private name and rename it into place, so no worker ever reads a
    # half-written library, header or record (M16: a get-up trial failed the library
    # check while another worker was rebuilding it in place).
    if lib.exists() and record_path.exists():
        try:
            record = json.loads(record_path.read_text())
            if (record["nominal"] == nominal and record["gains"] == gains
                    and record["library_sha256"] == hashlib.sha256(lib.read_bytes()).hexdigest()):
                return lib
        except (ValueError, KeyError):
            pass
    tag = f".{os.getpid()}"
    tmp_header = directory / f"generated_config.h{tag}"
    generate(nominal, gains, tmp_header)
    os.replace(tmp_header, header)
    tmp_lib = directory / f".libbipe{tag}.so"
    subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wconversion",
            "-Wshadow",
            "-fPIC",
            "-shared",
            "-I" + str(directory),
            "-I" + str(ROOT / "firmware/include"),
            *map(str, sources),
            "-lm",
            "-o",
            str(tmp_lib),
        ],
        check=True,
        capture_output=True,
    )
    # Metadata binds the requested host nominal/gains to this exact compiled core.
    record = {
        "nominal": nominal,
        "gains": gains,
        "library_sha256": hashlib.sha256(tmp_lib.read_bytes()).hexdigest(),
        "header_sha256": hashlib.sha256(header.read_bytes()).hexdigest(),
    }
    tmp_record = directory / f"libbipe.json{tag}"
    tmp_record.write_text(json.dumps(record, indent=2) + "\n")
    os.replace(tmp_lib, lib)
    os.replace(tmp_record, record_path)
    return lib


def linearize(p, gains, angle):
    from .kinematics import equilibrium
    from .moving_experiment import simulate

    m, d = build(p)
    # Start from an actual settled contact state, without pose/equality constraints.
    run = simulate(
        "hold",
        3,
        gains=gains,
        nominal=p,
        plant=p,
        sensing="ideal",
        disturbance={"initial_posture_deg": angle},
        command_function=lambda t, s: (0, 0, posture(angle), np.zeros(4)),
    )
    if run["failure"]:
        raise ValueError("Cannot linearize failed posture: " + run["failure"])
    d.qpos[:] = run["qpos"][-1]
    d.qvel[:] = 0
    # Coulomb stiction at exactly zero velocity obscures the rolling mode.
    # The tested physical plant retains its full friction and compensation.
    m.dof_frictionloss[:] = 0
    m.opt.tolerance = 0
    mujoco.mj_forward(m, d)
    s = truth(m, d, p)
    d.ctrl[:4] = support_feedforward(p, s["q"], s["pitch"]) + gains["joint_kp"] * (
        posture(angle) - s["q"]
    )
    mujoco.mj_forward(m, d)
    n = 2 * m.nv
    A, B = np.zeros((n, n)), np.zeros((n, m.nu))
    mujoco.mjd_transitionFD(m, d, 1e-6, True, A, B, None, None)
    F = np.zeros((4, n))
    for i, name in enumerate(JOINTS):
        v = m.joint(name).dofadr[0]
        F[i, v], F[i, m.nv + v] = -gains["joint_kp"], -gains["joint_kd"]
    # The support feedforward changes with posture and body attitude. Omitting
    # this feedback would linearize a different leg loop from the tested core.
    original_q = d.qpos.copy()
    for j in range(m.nv):
        delta = np.zeros(m.nv)
        delta[j] = 1
        samples = []
        for sign in [-1, 1]:
            d.qpos[:] = original_q
            mujoco.mj_integratePos(m, d.qpos, delta, sign * 1e-6)
            mujoco.mj_forward(m, d)
            state = truth(m, d, p)
            samples.append(support_feedforward(p, state["q"], state["pitch"], state["roll"]))
        F[:, j] += (samples[1] - samples[0]) / 2e-6
    d.qpos[:] = original_q
    mujoco.mj_forward(m, d)
    closed = A + B[:, :4] @ F
    # Retain symmetric leg position/rate modes. Eliminating them after each
    # 1 ms transition would discard the compliance that makes the real leg
    # feedforward and balance dynamics interact.
    P = np.zeros((7, n))
    P[0, 4], P[1, m.nv + 4] = 1, 1
    leg_dofs = [m.joint(name).dofadr[0] for name in JOINTS]
    eq_grad = np.zeros(4)
    for i in range(4):
        dq = np.eye(4)[i] * 1e-6
        eq_grad[i] = (equilibrium(p, s["q"] + dq) - equilibrium(p, s["q"] - dq)) / 2e-6
        P[0, leg_dofs[i]] = -eq_grad[i]
        P[3 + i % 2, leg_dofs[i]] = 0.5
        P[5 + i % 2, m.nv + leg_dofs[i]] = 0.5
    jac = np.zeros((3, m.nv))
    for side in ["left", "right"]:
        one = np.zeros_like(jac)
        mujoco.mj_jacSite(m, d, one, None, m.site(side + "_axle").id)
        jac += one / 2
    P[2, m.nv :] = jac[0]
    L = np.zeros((n, 7))
    for col in [0, 3, 4]:
        if col == 0:
            L[4, col] = 1
        else:
            for i in range(4):
                if i % 2 == col - 3:
                    L[leg_dofs[i], col] = 1
            L[4, col] = sum(eq_grad[i] * L[leg_dofs[i], col] for i in range(4))
        L[0, col], L[2, col] = -jac[0] @ L[: m.nv, col], -jac[2] @ L[: m.nv, col]
        for name in WHEELS:
            L[m.joint(name).dofadr[0], col] = -L[4, col] - (1 if col in [3, 4] else 0)
    L[m.nv :, 1] = L[: m.nv, 0]
    for col in [5, 6]:
        for i in range(4):
            if i % 2 == col - 5:
                L[m.nv + leg_dofs[i], col] = 1
        L[m.nv, col], L[m.nv + 2, col] = -jac[0] @ L[m.nv :, col], -jac[2] @ L[m.nv :, col]
        for name in WHEELS:
            L[m.nv + m.joint(name).dofadr[0], col] = -1
    L[m.nv, 2] = 1
    for name in WHEELS:
        L[m.nv + m.joint(name).dofadr[0], 2] = 1 / p["geometry"]["wheel_radius"]
    if np.max(abs(P @ L - np.eye(7))) > 1e-8:
        raise ValueError("Rolling-coordinate projection is inconsistent")
    small, b = P @ closed @ L, P @ (B[:, 4] + B[:, 5])
    alpha = -math.expm1(-m.opt.timestep / p["motor"]["torque_time_constant"])
    aa = np.zeros((8, 8))
    bb = np.zeros((8, 1))
    aa[:7, :7], aa[:7, 7], aa[7, 7] = small, b * (1 - alpha), 1 - alpha
    bb[:7, 0], bb[7, 0] = b * alpha, alpha
    Ac, Bc = np.eye(8), np.zeros((8, 1))
    for _ in range(round(p["simulation"]["control_dt"] / m.opt.timestep)):
        Bc = aa @ Bc + bb
        Ac = aa @ Ac
    Aug, Bug = np.eye(9), np.zeros((9, 1))
    Aug[:8, :8], Bug[:8] = Ac, Bc
    Aug[8, 2] = -p["simulation"]["control_dt"]
    order = [0, 1, 2, 7, 8, 3, 4, 5, 6]
    Aug, Bug = Aug[np.ix_(order, order)], Bug[order]
    com, ax = com_and_axle(p, s["q"])
    nc, na = com_and_axle(p, posture(40))
    return {
        "posture_deg": angle,
        "height_scale": (com[1] - ax[1]) / (nc[1] - na[1]),
        "A": Aug.tolist(),
        "B": Bug.tolist(),
        "equilibrium_rad": equilibrium(p, s["q"]),
        "settled_joint_error_deg": float(abs(np.degrees(s["q"] - posture(angle))).max()),
        "projection_identity_error": float(abs(P @ L - np.eye(7)).max()),
    }


def design_lqr(p, gains, qdiag=(180, 2, 15, 0.1, 5, 2, 2, 0.05, 0.05), r=3):
    rows = []
    for angle in [25, 40, 50, 60, 65]:
        row = linearize(p, gains, angle)
        A, B = np.array(row["A"]), np.array(row["B"])
        S = solve_discrete_are(A, B, np.diag(qdiag), [[r]])
        K = np.linalg.solve(np.array([[r]]) + B.T @ S @ B, B.T @ S @ A)
        row["K"] = K[0].tolist()
        row["closed_loop_spectral_radius"] = float(abs(np.linalg.eigvals(A - B @ K)).max())
        rows.append(row)
    return {
        "state": [
            "pitch-equilibrium",
            "pitch_rate",
            "velocity-vref",
            "estimated_applied_common_torque",
            "integral(vref-velocity)",
            "mean hip tracking error",
            "mean knee tracking error",
            "mean hip rate error",
            "mean knee rate error",
        ],
        "Q_diagonal": list(qdiag),
        "R_per_wheel_common_Nm": r,
        "method": "mjd_transitionFD centered 1e-6, closed local joint PD/support feedforward, symmetric leg modes, rolling projection, 20 ms lag, 5 ms ZOH, integral augmentation; Coulomb stiction removed only for design",
        "schedule": sorted(rows, key=lambda row: row["height_scale"]),
    }


def equivalence(p, gains, library, count=1000):
    from .firmware_control import FirmwareController
    from .locomotion import LocomotionController

    py, fw = LocomotionController(p, gains), FirmwareController(p, gains, library)
    # Isolate normal cascade arithmetic over deliberately unsafe random inputs.
    # Physical runs keep protection enabled; fall parity is separately tested.
    py.protection = None
    fw.fall_settings = None
    rng = np.random.default_rng(8)
    maximum = 0.0
    for i in range(count):
        q = posture(rng.uniform(30 if native(p) else 25, 65), p)
        s = {
            "q": q,
            "qd": rng.normal(0, 0.2, 4),
            "position": rng.normal(0, 0.1, 2),
            "pitch": rng.uniform(-0.25, 0.25),
            "roll": rng.uniform(-0.05, 0.05),
            "pitch_rate": rng.uniform(-1, 1),
            "yaw": rng.uniform(-0.3, 0.3),
            "yaw_rate": rng.uniform(-0.3, 0.3),
            "velocity": rng.uniform(-0.3, 0.3),
            "wheel_speed": rng.uniform(-10, 10, 2),
        }
        # Commands up to the firmware gates, so the reference shaping is covered at speed.
        top = p["sizing"]["drive_speed"]
        v = [0, 0.3, -0.3, top, -top][i // 100 % 5]
        y = [0, 0.4, -0.4, 1.0, -1.0][i // 150 % 5]
        qref, qdref = (
            posture(rng.uniform(30 if native(p) else 25, 65), p),
            rng.normal(0, 0.2, 4),
        )
        a, ref, _, _ = py.command(s, v, y, qref, qdref, 0.005)
        b, rf, _, _ = fw.command(s, v, y, qref, qdref, 0.005)
        caps = np.array(
            [p["articulation"]["peak_torque_per_joint"]] * 4
            + [p["motor"]["peak_torque_per_wheel"]] * 2
        )
        maximum = max(maximum, float(abs(np.clip(a, -caps, caps) - b).max()), abs(ref - rf))
    return {"samples": count, "max_error": maximum, "passed": maximum < 1e-10}


if __name__ == "__main__":
    import argparse

    from .parameters import load_parameters

    parser = argparse.ArgumentParser(description="Regenerate the nominal M8 LQR schedule")
    parser.add_argument("--output", type=Path, default=ROOT / "config/m8")
    parser.add_argument("--r", type=float, default=3)
    parser.add_argument("--friction-compensation", type=float, default=0.03)
    args = parser.parse_args()
    gains = json.loads((ROOT / "config/m8/baseline.json").read_text())
    design = design_lqr(load_parameters(), gains, r=args.r)
    gains.update(
        description="M8 scheduled LQR retaining symmetric leg modes and 20 ms motor lag",
        lqr_schedule=[{"height_scale": r["height_scale"], "K": r["K"]} for r in design["schedule"]],
        speed_integral_limit=1.0,
        wheel_friction_compensation=args.friction_compensation,
    )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "lqr-design.json").write_text(json.dumps(design, indent=2) + "\n")
    (args.output / "lqr.json").write_text(json.dumps(gains, indent=2) + "\n")
