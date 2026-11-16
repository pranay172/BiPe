"""Reproducible articulated experiments, evaluated against true physical motion."""

import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from .articulated import JOINTS, build, generate_xml, reset, truth
from .kinematics import crouch_reference, posture, support_feedforward
from .locomotion import LocomotionController, TorqueMotors, wrap
from .parameters import ROOT, load_parameters
from .profiles import CHOICES, kinematics, native
from .profiles import plant as native_plant_module
from .sensing import JointEncoderFeedback, SensorEstimator

EFFORT_NAMES = ["left_hip", "left_knee", "right_hip", "right_knee", "left", "right"]
COLUMNS = [
    "time",
    "x",
    "y",
    "velocity",
    "user_velocity",
    "yaw_deg",
    "yaw_rate",
    "user_yaw_rate",
    "heading_reference_deg",
    "pitch_deg",
    "pitch_error_deg",
    "estimated_pitch_deg",
    "pitch_reference_deg",
    "roll_deg",
    "height",
    "velocity_reference",
    "estimated_velocity",
]
COLUMNS += [n + "_deg" for n in JOINTS] + [n + "_reference_deg" for n in JOINTS]
COLUMNS += [n + "_requested_Nm" for n in EFFORT_NAMES] + [n + "_applied_Nm" for n in EFFORT_NAMES]
COLUMNS += ["joint_saturated", "wheel_saturated", "forbidden_contact"]


def commands(t, scenario, p=None):
    velocity = yaw_rate = 0.0
    if scenario == "drive":
        velocity = 0.3 if 2 - 1e-9 <= t < 7 - 1e-9 else (-0.3 if 9 - 1e-9 <= t < 14 - 1e-9 else 0)
    if scenario == "turn":
        yaw_rate = 0.4 if 2 - 1e-9 <= t < 5 - 1e-9 else (-0.4 if 7 - 1e-9 <= t < 10 - 1e-9 else 0)
    if scenario == "combined":
        velocity = (
            0.15 if 3 - 1e-9 <= t < 15 - 1e-9 else (-0.15 if 18 - 1e-9 <= t < 29 - 1e-9 else 0)
        )
        yaw_rate = (
            0.25 if 6 - 1e-9 <= t < 10 - 1e-9 else (-0.25 if 20 - 1e-9 <= t < 24 - 1e-9 else 0)
        )
    q, qd = (
        crouch_reference(t) if scenario in ["crouch", "combined"] else (posture(40), np.zeros(4))
    )
    native_kinematics = kinematics(p)
    if native_kinematics is not None:
        q, qd = (
            native_kinematics.crouch_reference(t, p)
            if scenario in ["crouch", "combined"]
            else (posture(40, p), np.zeros(4))
        )
    return velocity, yaw_rate, q, qd


def simulate(
    scenario="hold",
    duration=60,
    offset=0,
    sensing="ideal",
    seed=None,
    plant=None,
    gains=None,
    sensor_config=None,
    physics_dt=None,
    enabled=True,
    implementation="python",
    nominal=None,
    firmware_library=None,
    disturbance=None,
    command_function=None,
    log_dt=None,
    observer=None,
    command_hook=None,
):
    """observer(t, dt, drives, d), if given, is called once per physics step after the
    drive torques are applied and before integration (CAD plants only); it must only read.
    command_hook(t, estimate, truth), if given, is called at each control step after the
    estimator update; a (velocity, yaw_rate) it returns replaces the command function's
    until the next control step (the M12 follower in the loop)."""
    nominal = copy.deepcopy(load_parameters() if nominal is None else nominal)
    p = copy.deepcopy(nominal if plant is None else plant)
    if physics_dt is not None:
        p["simulation"]["physics_dt"] = physics_dt
    profile = native(p)
    cad_plant = profile is not None
    electrical = json.loads((ROOT / profile["electrical"]).read_text()) if cad_plant else None
    gain_path = ROOT / (profile["controller"] if cad_plant else "config/controller_moving.json")
    sensor_path = ROOT / (profile["sensors"] if cad_plant else "config/sensors.json")
    gains = copy.deepcopy(gains or json.loads(gain_path.read_text()))
    sensors = copy.deepcopy(sensor_config or json.loads(sensor_path.read_text()))
    seed = sensors["seed"] if seed is None else seed
    disturbance = copy.deepcopy(disturbance or {})
    gear = None
    mechanical = None
    if cad_plant:
        native_plant = native_plant_module(p)
        m, d = native_plant.build(p)
        native_plant.reset(
            m,
            d,
            p,
            disturbance.get("initial_posture_deg", 40),
            offset,
            disturbance.get("slope_deg", 0),
        )
        mechanical = native_plant.MechanicalDrives(m, p, disturbance)
    elif disturbance:
        from .disturbances import BacklashGear, build_disturbed, reset_on_plane

        m, d, active = build_disturbed(
            p, disturbance.get("slope_deg", 0), disturbance.get("backlash_deg", 0)
        )
        reset_on_plane(
            m,
            d,
            p,
            disturbance.get("slope_deg", 0),
            disturbance.get("initial_posture_deg", 40),
            offset,
        )
        gear = BacklashGear(m, disturbance["backlash_deg"]) if active else None
    else:
        m, d = build(p)
        reset(m, d, p, offset_deg=offset)
    if implementation not in ["python", "c"]:
        raise ValueError("Unknown controller implementation")
    estimator_type, local_type, controller_type = (
        SensorEstimator,
        JointEncoderFeedback,
        LocomotionController,
    )
    firmware_evidence = None
    if implementation == "c":
        from functools import partial

        from .firmware_control import (
            FirmwareController,
            FirmwareEstimator,
            FirmwareLocalFeedback,
            build_evidence,
        )

        if cad_plant and firmware_library is None:
            from .robustness_control import build_candidate

            firmware_library = build_candidate(nominal, gains, ROOT / profile["firmware_build"])

        if firmware_library is None:
            estimator_type, local_type, controller_type = (
                FirmwareEstimator,
                FirmwareLocalFeedback,
                FirmwareController,
            )
        else:
            estimator_type, local_type, controller_type = (
                partial(FirmwareEstimator, library=firmware_library),
                partial(FirmwareLocalFeedback, library=firmware_library),
                partial(FirmwareController, library=firmware_library),
            )
        firmware_evidence = build_evidence(firmware_library)
    controller = controller_type(nominal, gains)
    jm = TorqueMotors(p["articulation"], 4, "peak_torque_per_joint")
    wm = TorqueMotors(p["motor"], 2, "peak_torque_per_wheel")
    if disturbance.get("initial_state") is not None:
        # M15 get-up: start from this state (a fallen pose), qpos and qvel. The sensors
        # below calibrate in it, at rest, as a supervised start would.
        state = disturbance["initial_state"]
        d.qpos[:] = state["qpos"]
        d.qvel[:] = state.get("qvel", np.zeros(m.nv))
        if mechanical is not None:
            mechanical.forward(m, d)
        else:
            mujoco.mj_forward(m, d)
    estimator = estimator_type(m, d, nominal, sensors, seed) if sensing == "sensors" else None
    local_encoders = local_type(d, sensors) if sensing == "sensors" else None
    dt, control_dt = m.opt.timestep, nominal["simulation"]["control_dt"]
    control_steps = round(control_dt / dt)
    log_steps = max(1, round((log_dt or nominal["simulation"]["log_dt"]) / dt))
    servo_dt = nominal["simulation"]["joint_servo_dt"]
    servo_steps = round(servo_dt / dt)
    if not math.isclose(control_steps * dt, control_dt, abs_tol=1e-10):
        raise ValueError("Control timing must divide into the physics steps")
    ground = m.geom("ground").id
    tires = {m.geom(side + "_tire").id for side in ["left", "right"]}
    joint_ids = [m.joint(n).id for n in JOINTS]
    requested = np.zeros(6)
    applied = np.zeros(6)
    ref, vref, heading = 0.0, 0.0, 0.0
    saturated = np.zeros(6, bool)
    sat_duration = np.zeros(6)
    stats = {
        "peak_torque": np.zeros(6),
        "torque_squared": np.zeros(6),
        "above_reference": np.zeros(6),
        "peak_speed": np.zeros(6),
        "longest_saturation": np.zeros(6),
        "max_joint_error_deg": 0.0,
        "peak_pitch_deg": 0.0,
        "peak_roll_deg": 0.0,
        "forbidden_contact": False,
        "final_forbidden_contact": False,
        "push_impulse_Ns": 0.0,
        "positive_motor_work_J": 0.0,
        "negative_motor_work_J": 0.0,
        "peak_conditional_dc_input_A": 0.0,
        "peak_regen_mechanical_W": 0.0,
    }
    if cad_plant:
        stats.update(
            last_2s_peak_pitch_error_deg=None,
            last_2s_peak_speed_m_s=None,
            last_4s_pitch_min_deg=None,
            last_4s_pitch_max_deg=None,
        )
    rows, poses = [], []
    failure = None
    estimated = truth(m, d, p)
    local_reference = posture(disturbance.get("initial_posture_deg", 40), p)
    local_velocity_reference = np.zeros(4)
    joint_feedforward = np.zeros(4)
    hooked = None
    for step in range(round(duration / dt) + 1):
        # mj_step integrates qpos after computing derived sensor channels.
        # Refresh the channels at this sample time before local feedback.
        if mechanical is not None:
            mechanical.forward(m, d)
        else:
            mujoco.mj_forward(m, d)
        s = truth(m, d, p)
        t = float(d.time)
        velocity, yaw_rate, qref, qdref = (
            command_function(t, scenario) if command_function else commands(t, scenario, nominal)
        )
        if hooked is not None:
            velocity, yaw_rate = hooked
        if estimator is not None:
            estimator.preprocess(d, dt)
        if step % control_steps == 0:
            estimated = estimator.update(d) if estimator is not None else s
            if command_hook is not None:
                hooked = command_hook(t, estimated, s)
                if hooked is not None:
                    velocity, yaw_rate = hooked
            local_reference, local_velocity_reference = qref.copy(), qdref.copy()
            if implementation == "python":
                joint_feedforward = support_feedforward(
                    nominal, estimated["q"], estimated["pitch"], estimated["roll"]
                )
            if enabled:
                try:
                    supervisor_request, ref, vref, heading = controller.command(
                        estimated, velocity, yaw_rate, qref, qdref, control_dt
                    )
                    requested[4:] = supervisor_request[4:]
                    if estimator is not None:
                        # The next estimator step's drive acceleration compensation limit.
                        estimator.compensation_limit = getattr(
                            controller, "compensation_limit", 0.0
                        )
                    protective = getattr(controller, "protection", None)
                    if protective is not None and protective.latched:
                        local_reference = np.array(protective.qref)
                        local_velocity_reference = np.array(protective.qdref)
                except ValueError as exc:
                    failure = f"controller rejected at {t:.3f}s: {exc}"
                if implementation == "c":
                    joint_feedforward = controller.feedforward.copy()
                elif gains.get("knee_wheel_reaction") and nominal.get("profile") == "m10":
                    # As LocomotionController.command: the knee holds the wheel's reaction.
                    joint_feedforward[[1, 3]] -= gains["knee_wheel_reaction"] * requested[4:]
        # A controller that is not yet armed (M15 get-up: before its handover) is not
        # held to the physical limits or the safety gate, which judge a balancing robot.
        armed = getattr(controller, "armed", True)
        if enabled and step % servo_steps == 0:
            if implementation == "c" and armed:
                try:
                    controller.gate(estimated, t)
                except ValueError as exc:
                    failure = f"safety inhibited at {t:.3f}s: {exc}"
            if local_encoders is not None:
                local_q, local_qd = local_encoders.update(d, servo_dt)
                local_feedback = {
                    "q": local_q,
                    "qd": local_qd,
                    "pitch": estimated["pitch"],
                    "roll": estimated["roll"],
                }
            else:
                local_feedback = s
            try:
                requested[:4] = controller.leg_torques(
                    local_feedback, local_reference, local_velocity_reference, joint_feedforward
                )
                # A controller whose wheel speed loop runs at the servo rate (M13 policy).
                wheel_torques = getattr(controller, "wheel_torques", None)
                if wheel_torques is not None:
                    requested[4:] = wheel_torques(d, servo_dt)
            except ValueError as exc:
                failure = f"local control rejected at {t:.3f}s: {exc}"
        forbidden = any(
            (c.geom1 == ground and c.geom2 not in tires)
            or (c.geom2 == ground and c.geom1 not in tires)
            or (c.geom1 != ground and c.geom2 != ground)
            for c in d.contact
        )
        error = s["pitch"] - s["equilibrium"]
        if cad_plant:
            pitch_error_deg = math.degrees(error)
            if t >= duration - 2 - 1e-9:
                stats["last_2s_peak_pitch_error_deg"] = max(
                    stats["last_2s_peak_pitch_error_deg"] or 0.0, abs(pitch_error_deg)
                )
                stats["last_2s_peak_speed_m_s"] = max(
                    stats["last_2s_peak_speed_m_s"] or 0.0, abs(s["velocity"])
                )
            if t >= duration - 4 - 1e-9:
                low, high = stats["last_4s_pitch_min_deg"], stats["last_4s_pitch_max_deg"]
                stats["last_4s_pitch_min_deg"] = (
                    pitch_error_deg if low is None else min(low, pitch_error_deg)
                )
                stats["last_4s_pitch_max_deg"] = (
                    pitch_error_deg if high is None else max(high, pitch_error_deg)
                )
        stats["peak_pitch_deg"] = max(stats["peak_pitch_deg"], abs(math.degrees(error)))
        stats["peak_roll_deg"] = max(stats["peak_roll_deg"], abs(math.degrees(s["roll"])))
        stats["forbidden_contact"] |= forbidden
        stats["final_forbidden_contact"] = forbidden
        limits = m.jnt_range[joint_ids]
        violated = np.any(s["q"] < limits[:, 0] - 0.001) or np.any(s["q"] > limits[:, 1] + 0.001)
        if enabled and armed and (
            abs(error) > math.radians(20)
            or abs(s["roll"]) > math.radians(10)
            or forbidden
            or violated
        ):
            failure = f"physical limit at {t:.3f}s: pitch={math.degrees(error):.2f}, roll={math.degrees(s['roll']):.2f}, contact={forbidden}, joint_limit={bool(violated)}"
        terminal = bool(failure) or (not enabled and abs(error) > math.radians(30))
        if step % log_steps == 0 or terminal:
            rows.append(
                [
                    t,
                    *s["position"],
                    s["velocity"],
                    velocity,
                    math.degrees(s["yaw"]),
                    s["yaw_rate"],
                    yaw_rate,
                    math.degrees(heading),
                    math.degrees(s["pitch"]),
                    math.degrees(error),
                    math.degrees(estimated["pitch"]),
                    math.degrees(ref),
                    math.degrees(s["roll"]),
                    s["height"],
                    vref,
                    estimated["velocity"],
                    *np.degrees(s["q"]),
                    *np.degrees(qref),
                    *requested,
                    *applied,
                    float(saturated[:4].any()),
                    float(saturated[4:].any()),
                    float(forbidden),
                ]
            )
            poses.append(d.qpos.copy())
        if terminal or step == round(duration / dt):
            break
        if t >= 1:
            stats["max_joint_error_deg"] = max(
                stats["max_joint_error_deg"], float(np.max(np.abs(np.degrees(qref - s["q"]))))
            )
        d.xfrc_applied[:] = 0
        if scenario == "push" and (4 <= t < 4.1 or 10 <= t < 10.1):
            d.xfrc_applied[m.body("chassis").id, 0] = 1 if t < 5 else -1
        pulse = disturbance.get("push_Ns", 0)
        if pulse and 4 - 1e-9 <= t < 4.1 - 1e-9:
            d.xfrc_applied[m.body("chassis").id, 0] = pulse / 0.1
            stats["push_impulse_Ns"] += pulse / 0.1 * dt
        if mechanical is not None:
            speed = mechanical.motor_speed(d)
            applied, saturated = mechanical.apply(m, d, requested if enabled else np.zeros(6), dt)
        else:
            jt, js = jm.step(requested[:4], s["qd"], dt)
            rotor_speed = d.qvel[gear.rv] if gear is not None else s["wheel_speed"]
            wt, ws = wm.step(requested[4:], rotor_speed, dt)
            d.ctrl[:] = np.r_[jt, wt]
            applied = d.ctrl.copy()
            saturated = np.r_[js, ws]
            speed = np.r_[s["qd"], rotor_speed]
        sat_duration = np.where(saturated, sat_duration + dt, 0)
        stats["longest_saturation"] = np.maximum(stats["longest_saturation"], sat_duration)
        stats["peak_torque"] = np.maximum(stats["peak_torque"], abs(applied))
        stats["torque_squared"] += applied**2 * dt
        continuous = np.r_[
            np.full(4, p["articulation"]["continuous_torque_reference"]),
            np.full(2, p["motor"]["continuous_torque_per_wheel"]),
        ]
        stats["above_reference"] += (abs(applied) > continuous) * dt
        stats["peak_speed"] = np.maximum(stats["peak_speed"], abs(np.r_[s["qd"], s["wheel_speed"]]))
        power = mechanical.motor_torque * d.qvel[mechanical.rv] if cad_plant else applied * speed
        positive, negative = float(np.maximum(power, 0).sum()), float(-np.minimum(power, 0).sum())
        stats["positive_motor_work_J"] += positive * dt
        stats["negative_motor_work_J"] += negative * dt
        # Same deliberately conditional phase/current conventions as M4.
        if cad_plant:
            # Catalog rated/peak torque-current endpoints; conservative interpolant
            # avoids extending rated Kt through the saturated peak domain.
            current = abs(mechanical.phase_current)
            copper = float(
                (0.75 * np.asarray(electrical["phase_to_phase_resistance_ohm"]) * current**2).sum()
            )
            efficiency, bus, auxiliaries = (
                electrical["inverter_efficiency_assumption"],
                electrical["pack_nominal_V"],
                electrical["auxiliary_power_W_assumption"],
            )
            stats.setdefault("phase_current_peak_A", np.zeros(6))
            stats.setdefault("phase_current_squared_A2s", np.zeros(6))
            stats.setdefault("estimated_dc_input_work_J", 0.0)
            stats.setdefault("negative_motor_work_each_J", np.zeros(6))
            stats.setdefault("peak_net_regen_estimate_W", 0.0)
            stats.setdefault("net_regen_estimate_J", 0.0)
            stats["phase_current_peak_A"] = np.maximum(stats["phase_current_peak_A"], current)
            stats["phase_current_squared_A2s"] += current**2 * dt
            stats["estimated_dc_input_work_J"] += (
                positive / efficiency + copper + auxiliaries
            ) * dt
            stats["negative_motor_work_each_J"] += np.maximum(-power, 0) * dt
            net_regen = max(0, negative * efficiency - positive / efficiency - copper - auxiliaries)
            stats["peak_net_regen_estimate_W"] = max(stats["peak_net_regen_estimate_W"], net_regen)
            stats["net_regen_estimate_J"] += net_regen * dt
            if observer is not None:
                observer(t, dt, mechanical, d)
        else:
            copper = float((1.5 * 0.978 * (applied / (0.056 * 10 * 0.6)) ** 2).sum())
            efficiency, bus, auxiliaries = 0.6, 19.2, 6.0
        stats["peak_conditional_dc_input_A"] = max(
            stats["peak_conditional_dc_input_A"],
            (positive / efficiency + copper + auxiliaries) / bus,
        )
        stats["peak_regen_mechanical_W"] = max(stats["peak_regen_mechanical_W"], negative)
        if mechanical is not None:
            mechanical.integrate(m, d)
        elif gear is not None:
            gear.step(m, d, wt)
        else:
            mujoco.mj_step(m, d)
        if not (
            np.isfinite(d.qpos).all()
            and np.isfinite(d.qvel).all()
            and np.isfinite(d.sensordata).all()
        ):
            failure = f"nonfinite physical state after {t:.3f}s"
            break
    warnings = {
        mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(d.warning) if w.number
    }
    return {
        "implementation": implementation,
        "firmware_build": firmware_evidence,
        "scenario": scenario,
        "sensing": sensing,
        "seed": seed,
        "duration_requested": duration,
        "physics_dt": dt,
        "failure": failure,
        "warnings": warnings,
        "parameters": p,
        "nominal": nominal,
        "gains": gains,
        "sensors": sensors,
        "trace": np.array(rows),
        "qpos": np.array(poses),
        "stats": stats,
        "enabled": enabled,
        "disturbance": disturbance,
        "backlash": mechanical.metrics()
        if mechanical is not None
        else gear.metrics()
        if gear is not None
        else None,
    }


def metrics(run):
    a = run["trace"]
    fields = {n: a[:, i] for i, n in enumerate(COLUMNS)}
    f, stats = fields, run["stats"]
    t, err = f["time"], f["pitch_error_deg"]
    travel = np.linalg.norm(a[:, 1:3] - a[0, 1:3], axis=1)
    report = {
        "scenario": run["scenario"],
        "sensing": run["sensing"],
        "seed": run["seed"],
        "duration_simulated_s": float(t[-1]),
        "failure": run["failure"],
        "warnings": run["warnings"],
        "max_pitch_error_deg": stats["peak_pitch_deg"],
        "max_roll_deg": stats["peak_roll_deg"],
        "max_travel_m": float(travel.max()),
        "max_joint_error_deg": stats["max_joint_error_deg"],
        "peak_speed_m_s": float(np.abs(f["velocity"]).max()),
        "height_range_m": float(np.ptp(f["height"][t >= 2])) if (t >= 2).any() else 0,
        "final_velocity_m_s": float(f["velocity"][-1]),
        "final_yaw_deg": float(f["yaw_deg"][-1]),
        "final_heading_reference_deg": float(f["heading_reference_deg"][-1]),
        "final_heading_error_deg": abs(
            math.degrees(wrap(math.radians(f["heading_reference_deg"][-1] - f["yaw_deg"][-1])))
        ),
        "forbidden_contact": stats["forbidden_contact"],
        "physics_dt_s": run["physics_dt"],
        "peak_torque_Nm": dict(zip(EFFORT_NAMES, stats["peak_torque"].tolist(), strict=True)),
        "rms_torque_Nm": dict(
            zip(
                EFFORT_NAMES,
                np.sqrt(stats["torque_squared"] / max(float(t[-1]), 0.001)).tolist(),
                strict=True,
            )
        ),
        "time_above_continuous_reference_s": dict(
            zip(EFFORT_NAMES, stats["above_reference"].tolist(), strict=True)
        ),
        "peak_joint_speed_rad_s": dict(
            zip(EFFORT_NAMES, stats["peak_speed"].tolist(), strict=True)
        ),
        "longest_saturation_s": float(stats["longest_saturation"].max()),
    }
    if not run["enabled"]:
        fell = abs(err[-1]) >= 30
        entry = native(run["nominal"])
        if entry and entry.get("unpowered_rest_on_body"):
            report["final_body_contact"] = bool(stats["final_forbidden_contact"])
            fell = fell or report["final_body_contact"]
        report["passed"] = bool(fell and t[-1] <= 5 + 1e-6 and not run["warnings"])
        return report
    passed = (
        not run["failure"] and not run["warnings"] and t[-1] >= run["duration_requested"] - 0.02
    )
    passed = passed and report["longest_saturation_s"] <= 0.5 and report["max_joint_error_deg"] < 5
    case = run["scenario"]
    if case in ["hold", "tilt_positive", "tilt_negative"]:
        violations = np.flatnonzero(abs(err) > 2)
        settled = (
            float(t[violations[-1] + 1])
            if len(violations) and violations[-1] < len(t) - 1
            else (0.0 if not len(violations) else None)
        )
        report["settling_time_s"] = settled
        passed = passed and settled is not None and settled <= 3 and report["max_travel_m"] <= 0.1
    if case == "drive":
        plateau = ((t >= 4) & (t < 7)) | ((t >= 11) & (t < 14))
        mae = (
            float(np.mean(abs(f["velocity"][plateau] - f["user_velocity"][plateau])))
            if plateau.any()
            else None
        )
        stops = []
        for start, end in [(7, 9), (14, 20)]:
            ix = np.searchsorted(t, start - 1e-9)
            mask = (t >= start - 1e-9) & (t <= end + 1e-9)
            if ix < len(t) and mask.any():
                stops.append(float(np.linalg.norm(a[mask, 1:3] - a[ix, 1:3], axis=1).max()))
        report.update(
            plateau_speed_mae_m_s=mae, max_stopping_travel_m=max(stops) if stops else None
        )
        passed = passed and mae is not None and mae < 0.03 and bool(stops) and max(stops) < 0.1
    if case == "turn":
        plateau = ((t >= 3.5) & (t < 5)) | ((t >= 8.5) & (t < 10))
        mae = (
            float(np.mean(abs(f["yaw_rate"][plateau] - f["user_yaw_rate"][plateau])))
            if plateau.any()
            else None
        )
        report["plateau_yaw_rate_mae_rad_s"] = mae
        passed = passed and mae is not None and mae < 0.08 and report["final_heading_error_deg"] < 8
    if case in ["crouch", "combined"]:
        report["commanded_crouch_cycles"] = 5 if t[-1] >= 32 else 0
        passed = (
            passed and report["commanded_crouch_cycles"] >= 5 and report["height_range_m"] >= 0.04
        )
    if case == "push":
        recovered = []
        for release in [4.1, 10.1]:
            mask = (t >= release + 2) & (t < release + 3)
            recovered.append(bool(mask.any() and np.max(abs(err[mask])) < 2))
        report["push_recovered"] = recovered
        passed = passed and all(recovered)
    if case in ["drive", "turn", "crouch", "combined", "push"]:
        final = t >= run["duration_requested"] - 2
        final_pitch = float(np.max(abs(err[final]))) if final.any() else None
        report["final_hold_pitch_error_deg"] = final_pitch
        passed = (
            passed
            and final_pitch is not None
            and final_pitch < 2
            and abs(f["velocity"][-1]) < 0.03
            and report["final_heading_error_deg"] < 8
        )
    report["passed"] = bool(passed)
    return report


def save_run(run, output, plot=True):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    with (out / "trace.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(COLUMNS)
        writer.writerows(run["trace"])
    np.savez_compressed(out / "trajectory.npz", time=run["trace"][:, 0], qpos=run["qpos"])
    (out / "robot.xml").write_text(generate_xml(run["parameters"]))
    config = {
        "parameters": run["parameters"],
        "nominal_controller_parameters": run["nominal"],
        "controller": run["gains"],
        "implementation": run["implementation"],
        "firmware_build": run["firmware_build"],
        "sensors": run["sensors"],
        "sensing": run["sensing"],
        "seed": run["seed"],
    }
    (out / "configuration.json").write_text(json.dumps(config, indent=2) + "\n")
    report = metrics(run)
    report["configuration_sha256"] = hashlib.sha256(
        json.dumps(config, sort_keys=True).encode()
    ).hexdigest()
    (out / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if plot:
        plot_run(run, out)
    return report


def plot_run(run, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    f = {n: run["trace"][:, i] for i, n in enumerate(COLUMNS)}
    t = f["time"]
    fig, axs = plt.subplots(5, 1, figsize=(12, 11), sharex=True, layout="constrained")
    axs[0].plot(t, f["pitch_error_deg"], label="True pitch relative to posture equilibrium")
    axs[0].plot(
        t, f["estimated_pitch_deg"] - f["pitch_deg"], alpha=0.7, label="Pitch estimate error"
    )
    axs[0].axhspan(-2, 2, color="#328f70", alpha=0.12)
    axs[0].set_ylabel("Pitch (deg)")
    axs[1].plot(t, f["velocity"], label="True axle forward velocity")
    axs[1].plot(t, f["user_velocity"], "--", label="User velocity")
    axs[1].set_ylabel("Speed (m/s)")
    axs[2].plot(t, f["yaw_deg"], label="True heading")
    axs[2].plot(t, f["heading_reference_deg"], "--", label="Heading target")
    axs[2].set_ylabel("Yaw (deg)")
    axs[3].plot(t, f["height"] * 1000, label="Measured hip-to-axle height")
    axs[3].set_ylabel("Height (mm)")
    for name in ["left_hip", "left_knee", "left"]:
        axs[4].plot(t, f[name + "_applied_Nm"], label=name.replace("_", " ") + " applied")
    axs[4].set_ylabel("Torque (N m)")
    axs[4].set_xlabel("Time (s)")
    for ax in axs:
        ax.grid(alpha=0.2)
        ax.legend(loc="upper right")
    fig.suptitle(f"{run['scenario']} · articulated torque control · {run['sensing']} feedback")
    fig.savefig(Path(output) / "trace.png", dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=CHOICES, default="m4")
    parser.add_argument(
        "--scenario",
        choices=[
            "hold",
            "tilt_positive",
            "tilt_negative",
            "drive",
            "turn",
            "crouch",
            "combined",
            "push",
            "unpowered",
        ],
        default="combined",
    )
    parser.add_argument("--sensing", choices=["ideal", "sensors"], default="sensors")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    duration = args.duration or {
        "drive": 20,
        "turn": 15,
        "crouch": 36,
        "combined": 36,
        "push": 16,
        "unpowered": 5,
    }.get(args.scenario, 60)
    offset = {"tilt_positive": 5, "tilt_negative": -5, "unpowered": 5}.get(args.scenario, 0)
    nominal = load_parameters(profile=args.profile)
    run = simulate(
        args.scenario,
        duration,
        offset,
        args.sensing,
        enabled=args.scenario != "unpowered",
        nominal=nominal,
    )
    output = args.output or ROOT / (
        native(nominal)["experiments"] if native(nominal) else "artifacts/milestone-3/experiments"
    ) / (args.sensing + "_" + args.scenario)
    report = save_run(run, output)
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
