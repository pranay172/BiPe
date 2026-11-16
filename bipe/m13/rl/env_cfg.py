"""mjlab task configuration for the M13 first policy (phase B plan)."""

from __future__ import annotations

import json
import math

import mujoco

from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as base
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg

from ...parameters import ROOT, load_parameters
from .. import plant as tp
from . import mdp
from .firmware import DriveActionCfg

SHELL = (
    "SkidPad", "Head", "Neck", "Shell", "left_Ear", "right_Ear", "left_HipHousing",
    "right_HipHousing", "left_Thigh", "right_Thigh", "left_Shin", "right_Shin",
    "left_WheelRing", "right_WheelRing",
)  # fmt: skip


def _joint_names(p) -> list[str]:
    """The training plant's joints after the free base, in qpos order."""
    m, _ = tp.build(p)
    return [m.joint(i).name for i in range(1, m.njnt)]


def robot_cfg(p) -> EntityCfg:
    """The training-plant MJCF (with its ground plane and tire pairs) as one entity."""
    xml = tp.generate_xml(p)
    m, d = tp.build(p)
    tp.reset(m, d, p)
    joints = [m.joint(i).name for i in range(1, m.njnt)]
    legs = tp.JOINTS
    if tp.hip_spring(p):
        legs = [f"{j[: -len('hip')]}thigh_rotor_joint" if j.endswith("_hip") else j for j in legs]
    return EntityCfg(
        spec_fn=lambda: mujoco.MjSpec.from_string(xml),
        articulation=EntityArticulationInfoCfg(
            actuators=(XmlActuatorCfg(target_names_expr=(*legs, *tp.WHEELS)),),
        ),
        init_state=EntityCfg.InitialStateCfg(
            pos=tuple(float(x) for x in d.qpos[:3]),
            rot=tuple(float(x) for x in d.qpos[3:7]),
            joint_pos={j: float(d.qpos[m.joint(j).qposadr[0]]) for j in joints},
            joint_vel={".*": 0.0},
        ),
    )


STRESS = {
    "drive": {"transport_delay_ms": (5, 23), "encoder_lag_ms": (0, 8), "hub_lag_s": (0.0008, 0.0375)},
    "impulse_Ns": (0.0, 2.5),
    "slope_deg": 10.0,
    "slope_along_heading": True,
}
"""M14 stress mix: 1.5x the acceptance ranges (15 ms delay, 25 ms hub lag; encoder lag
1.5x the training range), pushes to the acceptance's 2.5 N s (2 November decision: with
the 1.5x delay a 3.75 N s push reaches the 20 degree pitch limit in about 70 ms, before
feedback can act), slopes to 10 degrees up or down the
heading (2 November feasibility: cross slopes trip the 10 degree roll fault at rest). With
``actuators`` it is the gate's stress suite (at most 5 % falls)."""

STEEP = {"flat_fraction": 0.15, "steep": 0.5}
"""Training slopes: fewer flat episodes, tilt max u^0.5 (three quarters of tilts past half the maximum)."""


def exact_sensors() -> dict:
    """The robot's sensor pipeline (packets, encoder differences, knee-drive encoder)
    with its filters, quantisation and periodic errors taken out, for diagnosis."""
    c = json.loads((ROOT / "config/sensors-m10.json").read_text())
    for key in ("gyro_anti_alias_tau_s", "accel_anti_alias_tau_s", "joint_velocity_filter_tau_s",
                "velocity_filter_tau_s", "local_joint_velocity_filter_tau_s"):  # fmt: skip
        c[key] = 1e-6
    for key in ("joint_encoder_counts_per_rev", "wheel_encoder_counts_per_rev",
                "local_joint_encoder_counts_per_rev"):  # fmt: skip
        c[key] = 2**30
    c["joint_encoder_periodic_error_rad"] = c["wheel_encoder_periodic_error_rad"] = 0.0
    return c


# Diagnostic variants of the robot task (bipe.m13.rl.train --variant NAME), each
# changing one thing: the robot's falls on exact readings; the robot's pipeline with
# near-exact readings; the robot's channels with the delay and hub lag ranges cut to
# the robot's own (5-10 ms, up to 3 ms).
VARIANTS = {
    "falls": lambda: {"falls": True},
    "pipeline": lambda: {"sensors": exact_sensors(), "falls": True},
    "narrow": lambda: {
        "robot": True,
        "drive_ranges": {"transport_delay_ms": (5, 10), "hub_lag_s": (0.0008, 0.003)},
    },
    # Not a diagnosis: the robot task with the rest sway trained out.
    "still": lambda: {"robot": True, "still": True},
    # And the rest lean that the stand-still reward taught (25 October).
    "lean": lambda: {"robot": True, "still": True, "lean": True},
    # And the full plant's push and pitch fall (26 October).
    "push": lambda: {"robot": True, "still": True, "lean": True, "full_push": True},
    # And twice the stand-still weight, for the rest sway left after pushaxle (27 October).
    "settle": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
    },
    # M14: the settle chain's lessons in one task from scratch, with the belts and the
    # actuator and power randomisation ramped in over the first RAMP_ITERATIONS.
    "locomotion": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True,
        "drive_ranges": {"ramp_steps": RAMP_ITERATIONS * 24},
    },
    # M14 fine-tune (2 November feasibility): no DR ramp, pushes to the stress set's
    # 3.75 N s, yaw rate tracking at twice the weight.
    "locomotion_ft": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True, "push_Ns": (0.5, 3.75), "yaw_weight": 2.0,
    },
    # Second fine-tune (2 November, stress pushes at 2.5 N s): the stress mix's drive
    # delays and lags during training, pushes to 3.0 N s.
    "locomotion_ft2": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True, "push_Ns": (0.5, 3.0), "yaw_weight": 2.0,
        "drive_ranges": dict(STRESS["drive"]),
    },
    # M15 fine-tune from fine-tune 1 (4 November plan, phase C): the speed suite's
    # misses (downhill hold and axle depth, curve paths, top-speed pitch, the step stop)
    # and the training-stress falls (steep slopes, pushes on slopes).
    "locomotion_ft3": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True, "push_Ns": (0.5, 3.0), "yaw_weight": 2.0,
        "posture_fine": 1.0, "still_slope": 2.0, "pitch_limit_deg": 13.0,
        "corner_fraction": 0.15,
    },
    # M16 fine-tune 4 from fine-tune 1 (phase C): fine-tune 1's task (pushes to 3.75 N s,
    # yaw at twice the weight) with only the terms aimed at what the command shaper
    # leaves (downhill hold and axle depth, push_fast_-2's axle depth, the r1 curve):
    # the 5 mm axle-depth kernel at half fine-tune 3's weight, the slope-scaled
    # stand-still kernel, and 10 % of commands at combined high speed and yaw rate, so
    # about 90 % of commands stay fine-tune 1's. No body pitch penalty: fine-tune 3's
    # 13 degree penalty is the likely cause of its ten low-push falls (a low push needs
    # a large pitch excursion to recover).
    "locomotion_ft4": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True, "push_Ns": (0.5, 3.75), "yaw_weight": 2.0,
        "posture_fine": 0.5, "still_slope": 2.0, "corner_fraction": 0.10,
    },
    # M15 get-up (phase E): episodes start in the fallen-state library
    # (artifacts/m15/fallen_states.npz), stand-still commands at the 40 posture, rewards
    # for the get-up study's handover test. Trained from the locomotion policy's weights.
    "getup": lambda: {
        "robot": True, "belts": True, "actuators": True, "getup": True,
    },
    # Second get-up round: classes drawn equally, back leans (9/20 on the full plant in
    # the first round) half as often again.
    "getup2": lambda: {
        "robot": True, "belts": True, "actuators": True, "getup": True,
        "getup_weights": {"sit": 1.0, "front_lean": 1.0, "back_lean": 1.5},
    },
    # M16 getup3 (phase E/F): the flat rests split out of the leans (past 70 degrees),
    # back and front leans weighted alike (getup2's back weight cost a front lean),
    # flat on the back weighted up (the flat study found motions from 6 of 24 starts, all
    # on the back), flat on the face kept but not weighted (no motion found), and 8 s
    # episodes (the found motions take up to 4.6 s before the handover).
    "getup3": lambda: {
        "robot": True, "belts": True, "actuators": True, "getup": True,
        "getup_weights": {"sit": 1.0, "front_lean": 1.5, "back_lean": 1.5,
                          "flat_back": 1.5, "flat_front": 0.5},
        "getup_flat_deg": 70.0, "getup_episode_s": 8.0,
    },
    # M16 getup4: getup3 rejected on the held-out gate (early handovers, face-down
    # losses). Back to getup's class mix with the flat rests split out and flat on the
    # face weighted as much as the leans, the handover rewarded only once it has held
    # 0.5 s, and 8 s episodes.
    "getup4": lambda: {
        "robot": True, "belts": True, "actuators": True, "getup": True,
        "getup_weights": {"sit": 1.0, "front_lean": 1.0, "back_lean": 1.0,
                          "flat_back": 1.0, "flat_front": 1.0},
        "getup_flat_deg": 70.0, "getup_episode_s": 8.0, "getup_hold_s": 0.5,
    },
    # M16 sit/stand (phase G): the drive task with a sit request in the command's
    # posture slot (mdp.SIT_LABEL); half the episodes start sitting (the fallen-state
    # library's sit class), half standing. Trained from fine-tune 1's weights.
    "sit_stand": lambda: {
        "robot": True, "still": True, "lean": True, "full_push": True, "still_weight": 2.0,
        "belts": True, "actuators": True, "push_Ns": (0.5, 3.75), "yaw_weight": 2.0,
        "sit_stand": True,
    },
}
FALLEN_STATES = ROOT / "artifacts/m15/fallen_states.npz"
GETUP_EPISODE_S = 6.0
RAMP_ITERATIONS = 2000
"""M14 recipe: iterations (24 env steps each) over which the belt and actuator DR ramps in."""


def env_cfg(
    play: bool = False,
    num_envs: int = 4096,
    slope_deg: float = 10.0,
    hard: bool = False,
    robot: bool = False,
    sensors: dict | None = None,
    falls: bool | None = None,
    drive_ranges: dict | None = None,
    still: bool = False,
    lean: bool = False,
    full_push: bool = False,
    still_weight: float = 1.0,
    belts: bool = False,
    actuators: bool = False,
    stress: bool = False,
    push_Ns: tuple[float, float] | None = None,
    yaw_weight: float = 1.0,
    posture_fine: float = 0.0,
    still_slope: float = 0.0,
    pitch_limit_deg: float | None = None,
    corner_fraction: float = 0.0,
    getup: bool = False,
    getup_weights: dict[str, float] | None = None,
    getup_flat_deg: float | None = None,
    getup_episode_s: float = GETUP_EPISODE_S,
    sit_stand: bool = False,
    getup_hold_s: float = 0.0,
) -> ManagerBasedRlEnvCfg:
    """The task. ``hard`` is the training mix: steep slopes more often and larger
    pushes than the evaluation draws (bipe.m13.rl.evaluate uses the default).
    ``robot`` observes the robot's sensor channels (config/sensors-m10.json, see
    bipe.m13.rl.firmware) and fails as the full plant does: roll past 10 degrees or
    a leg joint past its range. ``sensors`` (a sensor config in place of
    config/sensors-m10.json) and ``falls`` set the two halves on their own;
    ``drive_ranges`` overrides the randomised firmware ranges (mdp.randomise_drive).
    ``still`` trains the rest sway out (24 October full-plant results): a stand-still
    reward, more stand-still commands and more pushes along the heading. ``lean`` adds
    a kernel on the COM lean at rest (the full plant's pitch error). ``full_push``
    pushes as the full plant does (a force on the chassis for 0.1 s, mdp.push_force)
    and fails at a 20 degree COM lean, as the full plant's pitch limit does.
    ``still_weight`` scales the stand-still reward. ``belts`` is M14's plant: hip
    belts as springs with lash on their own rotors and the knee belt's lash as a dead
    band, stiffness and lash randomised per world (bipe.m13.rl.firmware).
    ``actuators`` adds M14's actuator and power randomisation (firmware
    ``actuator_dr``) and widens the COM offset to 5 mm; ``stress`` draws the delays,
    lags, pushes and slopes from ``STRESS``. ``push_Ns`` overrides the push impulse
    range and ``yaw_weight`` scales the yaw rate tracking terms (M14 fine-tune).
    M15 (fine-tune 3): ``posture_fine`` weights a 5 mm axle-depth kernel (the speed
    rule's band; the posture kernel is 15 mm wide); ``still_slope`` weights the
    stand-still kernel scaled by the slope; ``pitch_limit_deg`` penalises body pitch
    past it (weight -50 per rad^2); ``corner_fraction`` samples combined high speed and
    yaw rate commands. ``getup`` is the M15 get-up task: fallen starts from
    ``FALLEN_STATES``, stand-still commands, no pushes or slopes, a handover reward,
    and only the roll-over (sideways past 60 degrees), joint-limit and non-finite
    terminations, over ``getup_episode_s`` episodes; ``getup_weights`` and
    ``getup_flat_deg`` set the start classes' shares (mdp.reset_fallen). ``sit_stand``
    (M16) adds sit requests to the drive commands (40 % of resamples), starts half the
    episodes sitting, counts the drive rewards and failures only while standing (the
    failures from 3 s after a stand request), and adds the sit rewards: seated, a soft
    landing and low leg torque once down."""
    p = load_parameters(profile="m13")
    if belts:
        p["training"]["drives"].update(hip_belt="spring", knee_lash="deadband")
    sim = p["training"]["simulation"]
    physics_dt = p["simulation"]["physics_dt"]

    actor = {
        "sensors": ObservationTermCfg(func=mdp.sensors_delayed),
        "action": ObservationTermCfg(func=mdp.last_action),
        "command": ObservationTermCfg(func=mdp.drive_command),
    }
    critic = {
        **actor,
        "privileged": ObservationTermCfg(func=mdp.privileged),
    }
    observations = {
        "actor": ObservationGroupCfg(actor, concatenate_terms=True, enable_corruption=False),
        "critic": ObservationGroupCfg(critic, concatenate_terms=True, enable_corruption=False),
    }

    commands = {
        "drive": mdp.DriveCommandCfg(
            resampling_time_range=(3.0, 8.0), parameters=p,
            **({} if getup else {"standing_fraction": 0.3 if still else 0.15}),
            corner_fraction=corner_fraction,
            sit_fraction=0.4 if sit_stand else 0.0,
            **({"standing_fraction": 1.0, "posture_range": (40.0, 40.0)} if getup else {}),
        ),
    }
    if sensors is None and robot:
        sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
    falls = robot if falls is None else falls
    actions = {
        mdp.ACTION: DriveActionCfg(
            entity_name=mdp.ROBOT, parameters=p, sensors=sensors, actuator_dr=actuators
        )
    }
    if stress:
        drive_ranges = {**STRESS["drive"], **(drive_ranges or {})}
        slope_deg = STRESS["slope_deg"]
    along = stress and STRESS.get("slope_along_heading", False)

    entity = SceneEntityCfg(mdp.ROBOT)
    events = {
        "reset_base": EventTermCfg(
            func=base.reset_root_state_uniform,
            mode="reset",
            params={"pose_range": {"yaw": (-math.pi, math.pi)}, "velocity_range": {}},
        ),
        "reset_joints": EventTermCfg(
            func=base.reset_joints_by_offset,
            mode="reset",
            params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0)},
        ),
        "drive": EventTermCfg(func=mdp.randomise_drive, mode="reset", params=drive_ranges or {}),
        "slope": EventTermCfg(
            func=mdp.slope,
            mode="reset",
            params={"max_deg": slope_deg, "along_heading": along, **(STEEP if hard else {})},
        ),
        "mass": EventTermCfg(
            # Mass and inertia together, per body: e^(2 alpha) in 0.85-1.15.
            func=dr.pseudo_inertia,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg(mdp.ROBOT, body_names=(".*",)),
                "alpha_range": (math.log(0.85) / 2, math.log(1.15) / 2),
            },
        ),
        "com": EventTermCfg(
            func=dr.body_com_offset,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg(mdp.ROBOT, body_names=("chassis",)),
                "ranges": {0: (-0.005, 0.005), 2: (-0.005, 0.005) if actuators else (-0.003, 0.003)},
            },
        ),
        "tire_friction": EventTermCfg(
            func=dr.pair_friction,
            mode="startup",
            params={"asset_cfg": entity, "ranges": (0.5, 1.0), "isotropic": True,
                    "shared_random": True},
        ),  # fmt: skip
        "push": EventTermCfg(
            func=mdp.push_force if full_push else mdp.push, mode="interval",
            interval_range_s=(3.0, 6.0),
            params={"impulse_Ns": push_Ns or (STRESS["impulse_Ns"] if stress else (0.5, 3.0) if hard else (0.0, 2.5)),
                    "forward": 0.7 if still else 0.5},
        ),  # fmt: skip
    }

    if getup:
        events.pop("push")
        events.pop("slope")
        events["fallen"] = EventTermCfg(
            func=mdp.reset_fallen,
            mode="reset",
            params={
                "path": str(FALLEN_STATES), "joint_names": tuple(_joint_names(p)),
                "class_weights": getup_weights, "flat_deg": getup_flat_deg,
            },  # fmt: skip
        )

    if sit_stand:
        events["fallen"] = EventTermCfg(
            func=mdp.reset_fallen,
            mode="reset",
            params={
                "path": str(FALLEN_STATES), "joint_names": tuple(_joint_names(p)),
                "class_weights": {"sit": 1.0, "front_lean": 0.0, "back_lean": 0.0},
                "fraction": 0.5,
            },  # fmt: skip
        )

    rewards = {
        "speed": RewardTermCfg(func=mdp.track_speed, weight=2.0, params={"std": 0.25}),
        "yaw_rate": RewardTermCfg(func=mdp.track_yaw_rate, weight=yaw_weight, params={"std": 0.25}),
        "yaw_rate_error": RewardTermCfg(func=mdp.yaw_rate_error, weight=-0.2 * yaw_weight),
        "posture": RewardTermCfg(func=mdp.track_posture, weight=1.0, params={"std": 0.015}),
        "posture_error": RewardTermCfg(
            func=mdp.posture_error, weight=-0.05, params={"scale": 0.01}
        ),
        "level": RewardTermCfg(func=mdp.level_body, weight=0.5, params={"std": 0.1}),
        "action_rate": RewardTermCfg(func=mdp.action_rate, weight=-0.02),
        "leg_torque": RewardTermCfg(func=mdp.leg_torque, weight=-0.01),
        "wheel_torque": RewardTermCfg(func=mdp.wheel_torque, weight=-0.5),
        "power": RewardTermCfg(func=mdp.motor_power, weight=-0.01),
        "leg_limits": RewardTermCfg(func=mdp.leg_limits, weight=-10.0, params={"margin": 0.05}),
        "terminated": RewardTermCfg(func=base.is_terminated, weight=-50.0),
    }
    if still:
        rewards["stand_still"] = RewardTermCfg(
            func=mdp.stand_still, weight=still_weight, params={"std": 0.02}
        )
    if lean:
        rewards["rest_lean"] = RewardTermCfg(
            func=mdp.rest_lean, weight=1.0, params={"std": math.radians(1.0)}
        )
    if posture_fine:
        rewards["posture_fine"] = RewardTermCfg(
            func=mdp.track_posture, weight=posture_fine, params={"std": 0.005}
        )
    if still_slope:
        rewards["stand_still_slope"] = RewardTermCfg(
            func=mdp.stand_still_slope, weight=still_slope, params={"std": 0.02}
        )
    if pitch_limit_deg is not None:
        rewards["pitch_excess"] = RewardTermCfg(
            func=mdp.pitch_excess, weight=-50.0, params={"limit_deg": pitch_limit_deg}
        )
    terminations = {
        "time_out": TerminationTermCfg(func=base.time_out, time_out=True),
        "tipped": TerminationTermCfg(func=mdp.tipped, params={"limit_deg": 20.0}),
        "shell_contact": TerminationTermCfg(
            func=mdp.shell_contact, params={"sensor_name": "shell_contact"}
        ),
    }
    if sit_stand:
        keep = {"action_rate", "wheel_torque", "power", "leg_limits", "terminated"}
        for name, term in rewards.items():
            if name not in keep:
                rewards[name] = RewardTermCfg(
                    func=mdp.standing_only, weight=term.weight,
                    params={"term": term.func, **(term.params or {})},
                )  # fmt: skip
        rewards["upright"] = RewardTermCfg(
            func=mdp.standing_only, weight=1.0, params={"term": mdp.upright, "std": 0.6}
        )
        rewards["seated"] = RewardTermCfg(
            func=mdp.seated, weight=3.0, params={"sensor_name": "shell_contact"}
        )
        rewards["sit_landing"] = RewardTermCfg(func=mdp.sit_landing, weight=-2.0)
        rewards["sit_torque"] = RewardTermCfg(func=mdp.sit_torque, weight=-0.05)
    if getup:
        rewards["upright"] = RewardTermCfg(func=mdp.upright, weight=1.0, params={"std": 0.6})
        rewards["handover"] = RewardTermCfg(
            func=mdp.handover_held if getup_hold_s else mdp.handover, weight=3.0,
            params={"sensor_name": "shell_contact",
                    **({"hold_s": getup_hold_s} if getup_hold_s else {})},
        )  # fmt: skip
        terminations = {
            "time_out": terminations["time_out"],
            "rolled": TerminationTermCfg(func=mdp.side_rolled, params={"limit_deg": 60.0}),
            "joint_limit": TerminationTermCfg(func=mdp.joint_limit),
        }
        falls = False
    if falls:
        terminations["rolled"] = TerminationTermCfg(func=mdp.rolled, params={"limit_deg": 10.0})
        terminations["joint_limit"] = TerminationTermCfg(func=mdp.joint_limit)
    if belts:
        # A world whose physics goes non-finite (a stiff belt corner under a violent
        # action) ends as a fall and is reset, instead of stopping the run.
        terminations["nan"] = TerminationTermCfg(func=base.nan_detection)
    if full_push:
        terminations["leaned"] = TerminationTermCfg(func=mdp.leaned, params={"limit_deg": 20.0})
    if sit_stand:
        # Sitting is a rest on the shell, and rising takes a few seconds: the drive
        # failures count only from 3 s after a stand request.
        for name in ("tipped", "shell_contact", "leaned", "rolled"):
            if name in terminations:
                t = terminations[name]
                terminations[name] = TerminationTermCfg(
                    func=mdp.standing_failure, params={"term": t.func, **(t.params or {})}
                )
    shell = ContactSensorCfg(
        name="shell_contact",
        primary=ContactMatch(mode="geom", pattern=SHELL, entity=mdp.ROBOT),
        secondary=ContactMatch(mode="geom", pattern="ground", entity=mdp.ROBOT),
        fields=("found",),
        reduce="none",
        num_slots=1,
    )

    cfg = ManagerBasedRlEnvCfg(
        scene=SceneCfg(
            entities={mdp.ROBOT: robot_cfg(p)},
            sensors=(shell,),
            num_envs=num_envs,
            env_spacing=1.0,
        ),
        observations=observations,
        actions=actions,
        commands=commands,
        events=events,
        rewards=rewards,
        terminations=terminations,
        sim=SimulationCfg(
            nconmax=16,
            njmax=120,
            mujoco=MujocoCfg(
                timestep=physics_dt,
                integrator=sim["integrator"],
                cone=sim["cone"],
                iterations=sim["iterations"],
                impratio=sim["impratio"],
                gravity=(0.0, 0.0, -p["sizing"]["gravity"]),
            ),
        ),
        decimation=round(0.02 / physics_dt),
        episode_length_s=getup_episode_s if getup else 20.0,
    )
    if play:
        cfg.episode_length_s = 1e9
        cfg.events.pop("push", None)
        cfg.events.pop("slope", None)
    return cfg


def runner_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(256, 128, 64),
            activation="elu",
            obs_normalization=True,
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 0.5,
                "std_type": "scalar",
            },
        ),
        critic=RslRlModelCfg(hidden_dims=(256, 128, 64), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.005,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=1.0e-3,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
        experiment_name="bipe_m13",
        logger="tensorboard",
        save_interval=100,
        num_steps_per_env=24,
        max_iterations=3000,
    )
