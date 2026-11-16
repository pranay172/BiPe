import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("mjlab")


def test_task_builds_and_steps():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4), device="cpu")
    obs, _ = env.reset()
    assert obs["actor"].shape == (4, 25)
    # The training plant's own falls, unless the robot's are asked for.
    assert not {"joint_limit", "rolled"} & set(env.termination_manager.active_terms)
    for _ in range(3):
        obs, reward, *_ = env.step(torch.zeros(4, 6))
    assert torch.isfinite(obs["critic"]).all() and torch.isfinite(reward).all()
    drive = env.action_manager.get_term("drive")
    assert (drive.motor_torque[:, :4].abs() > 0).any()


def test_slope_tilts_gravity_per_world():
    import math

    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.utils.lab_api.math import quat_apply_inverse

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4), device="cpu")
    env.reset()
    torch.manual_seed(0)
    mdp.slope(env, None, max_deg=10.0, flat_fraction=0.0)
    gravity = env.sim.model.opt.gravity
    g = -float(env.sim.mj_model.opt.gravity[2])
    assert torch.allclose(gravity.norm(dim=-1), torch.full((4,), g))
    tilt = torch.acos(-gravity[:, 2] / g)
    assert (tilt > 0).all() and (tilt <= math.radians(10.0) + 1e-6).all()
    assert len({tuple(x) for x in gravity.tolist()}) == 4

    # Lifted clear of the ground with the drives off, each world falls along its
    # own gravity, and pitch and roll are taken from the true vertical.
    robot = env.scene[mdp.ROBOT]
    pose = robot.data.root_link_pose_w.clone()
    pose[:, 2] += 0.5
    robot.write_root_link_pose_to_sim(pose)
    robot.write_root_link_velocity_to_sim(torch.zeros(4, 6))
    env.sim.data.qvel[:] = 0.0
    env.sim.data.ctrl[:] = 0.0
    env.sim.data.qfrc_applied[:] = 0.0
    env.sim.forward()
    root = robot.indexing.root_body_id
    start = env.sim.data.subtree_com[:, root].clone()
    steps = 50
    for _ in range(steps):
        env.sim.step()
    # Internal forces do not move the centre of mass: it falls at g per world.
    # (subtree_com is computed before each step's integration, hence n(n - 1).)
    expected = gravity * env.physics_dt**2 * steps * (steps - 1) / 2
    drop = env.sim.data.subtree_com[:, root] - start
    assert torch.allclose(drop, expected, atol=2e-5), (drop, expected)
    pitch, roll = mdp.pitch_roll(env)
    upright = torch.zeros(4)
    assert not torch.allclose(torch.sqrt(pitch**2 + roll**2), upright, atol=0.5 * float(tilt.min()))


def test_slope_start_is_upright():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import env_cfg

    cfg = env_cfg(num_envs=4)
    cfg.events.pop("slope")
    env = ManagerBasedRlEnv(cfg, device="cpu")
    env.reset()
    robot = env.scene[mdp.ROBOT]
    wheels = robot.find_bodies(("left_wheel", "right_wheel"))[0]
    mdp.slope(env, None, max_deg=0.0, flat_fraction=0.0)
    env.sim.forward()
    flat_pitch, _ = mdp.pitch_roll(env)
    flat_wheels = robot.data.body_link_pos_w[:, wheels].clone()
    heading = mdp._heading(env).clone()

    torch.manual_seed(0)
    mdp.slope(env, None, max_deg=10.0, flat_fraction=0.0)
    env.sim.forward()
    pitch, _ = mdp.pitch_roll(env)
    # Balanced against the true vertical as on flat ground (to the coupling with the
    # roll that a cross slope leaves), wheels where they were.
    assert torch.allclose(pitch, flat_pitch, atol=1e-3)
    assert torch.allclose(robot.data.body_link_pos_w[:, wheels], flat_wheels, atol=1e-5)
    assert torch.allclose(mdp._heading(env), heading, atol=1e-5)

    mdp.slope(env, None, max_deg=10.0, flat_fraction=0.0, upright=False)
    env.sim.forward()
    pitch, _ = mdp.pitch_roll(env)
    assert (pitch - flat_pitch).abs().max() > 0.01


def test_reset_fills_sensor_history():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4), device="cpu")
    for _ in range(2):
        obs, _ = env.reset()
        # Delayed readings at the start are the standing robot, not zeros (the
        # accelerometer reads less than g while the tires take up the load).
        sensors = obs["actor"][:, :16]
        assert (sensors[:, 3:6].norm(dim=-1) > 5.0).all()
        assert (sensors[:, 6:10].abs() < 0.05).all()
        drive = env.action_manager.get_term("drive")
        enc = drive.sample(drive.encoder_lag)
        assert (enc[:, 6:10].abs() > 0.1).any(dim=-1).all()
        env.step(torch.zeros(4, 6))


def test_robot_sensing_channels_and_falls():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4, robot=True), device="cpu")
    obs, _ = env.reset()
    assert obs["actor"].shape == (4, 25)
    assert (obs["actor"][:, 3:6].norm(dim=-1) > 5.0).all()
    for _ in range(3):
        obs, *_ = env.step(torch.zeros(4, 6))
    assert torch.isfinite(obs["actor"]).all()
    drive = env.action_manager.get_term("drive")
    # Packets every 5 ms, held in between: a reading changes only at a packet step.
    rows = torch.stack([drive.sample(torch.full((4,), k)) for k in range(20)])
    changed = (rows[:-1] != rows[1:]).any(dim=-1)
    packet = torch.tensor([(drive.head - k) % 5 == 0 for k in range(19)])
    assert (changed == packet[:, None]).all()
    # Quantised encoders on the knee-drive hinges (motor side of the knee belts).
    assert torch.equal(drive.packet[:, 6:10], drive.encoder_previous[:, :4])
    enc = drive._encoders()
    counts = enc / drive.increment
    assert torch.allclose(counts, counts.round(), atol=1e-3)
    robot = env.scene[mdp.ROBOT]
    hinge = robot.data.joint_pos[:, drive.leg] + drive.encoder_offset
    # Within the periodic error (0.3 degrees) of the hinge, not the shin angle.
    assert (enc[:, :4] - hinge).abs().max() < 0.006

    terms = env.termination_manager.active_terms
    assert "joint_limit" in terms
    assert env.termination_manager.get_term_cfg("rolled").params["limit_deg"] == 10.0
    assert not mdp.joint_limit(env).any()
    q = robot.data.joint_pos.clone()
    lim = robot.data.joint_pos_limits[0, drive.leg[1]]
    q[0, drive.leg[1]] = lim[1] + 0.01
    robot.write_joint_state_to_sim(q, robot.data.joint_vel)
    env.sim.forward()
    assert mdp.joint_limit(env).tolist() == [True, False, False, False]


def test_diagnostic_variants_build():
    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    for name, kwargs in VARIANTS.items():
        cfg = env_cfg(num_envs=2, **kwargs())
        sensors = cfg.actions["drive"].sensors
        assert ("rolled" in cfg.terminations) is True, name
        assert (sensors is None) == (name == "falls"), name
    narrow = env_cfg(num_envs=2, **VARIANTS["narrow"]())
    assert narrow.events["drive"].params["transport_delay_ms"] == (5, 10)
    assert VARIANTS["pipeline"]()["sensors"]["wheel_encoder_counts_per_rev"] == 2**30


def test_stand_still_reward():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    cfg = env_cfg(num_envs=4, **VARIANTS["still"]())
    assert cfg.events["push"].params["forward"] == 0.7
    assert cfg.commands["drive"].standing_fraction == 0.3
    env = ManagerBasedRlEnv(cfg, device="cpu")
    env.reset()
    command = env.command_manager.get_term("drive")
    command.value[:] = torch.tensor([0.0, 0.0, 40.0])
    r = mdp.stand_still(env, std=0.02)
    assert ((r > 0.5) & (r <= 1.0)).all()  # at rest after the reset
    command.value[:, 0] = 0.5
    assert (mdp.stand_still(env, std=0.02) == 0).all()


def test_rest_lean_reward():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    cfg = env_cfg(num_envs=4, **VARIANTS["lean"]())
    assert "rest_lean" in cfg.rewards and "stand_still" in cfg.rewards
    assert "rest_lean" not in env_cfg(num_envs=4, **VARIANTS["still"]()).rewards
    env = ManagerBasedRlEnv(cfg, device="cpu")
    env.reset()
    lean = mdp.com_lean(env)
    assert torch.isfinite(lean).all() and (lean.abs() < 0.1).all()  # upright start
    command = env.command_manager.get_term("drive")
    command.value[:] = torch.tensor([0.0, 0.0, 40.0])
    r = mdp.rest_lean(env, std=0.0175)
    assert ((r > 0) & (r <= 1.0)).all()
    command.value[:, 1] = 0.5
    assert (mdp.rest_lean(env, std=0.0175) == 0).all()


def test_full_push():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl import mdp
    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    cfg = env_cfg(num_envs=2, **VARIANTS["push"]())
    assert cfg.events["push"].func is mdp.push_force and "leaned" in cfg.terminations
    assert "leaned" not in env_cfg(num_envs=2, **VARIANTS["lean"]()).terminations
    # Level ground, no random pushes: the zero action holds a standing robot.
    env = ManagerBasedRlEnv(env_cfg(play=True, num_envs=2, **VARIANTS["push"]()), device="cpu")
    env.reset()
    zero = torch.zeros(2, 6)
    v0 = mdp.forward_speed(env).clone()
    mdp.push_force(env, None, (1.0, 1.0), 1.0)
    for _ in range(5):  # 0.1 s
        env.step(zero)
    drive = env.action_manager.get_term("drive")
    assert (drive.push_steps == 0).all()
    assert (mdp.forward_speed(env) - v0 > 0.2).all()  # pushed forward
    env.step(zero)
    assert env.sim.data.xfrc_applied.abs().sum() == 0  # and released


def test_m14_belts_per_world():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4, belts=True, robot=True), device="cpu")
    env.reset()
    drive = env.action_manager.get_term("drive")
    assert drive.spring
    names = env.scene["robot"].joint_names
    assert {names[i] for i in drive.motor[[0, 2]].tolist()} == {
        "left_thigh_rotor_joint", "right_thigh_rotor_joint"
    }  # fmt: skip
    model = env.sim.model
    k = model.tendon_stiffness[:, drive.hip_belts[0]]
    assert torch.allclose(k, drive.belt_params[:, 0])
    assert len(set(k.tolist())) == 4
    # The stiffest corner, which needs the rotor armature floor, stays finite.
    ids = torch.arange(4)
    drive.set_belts(ids, torch.tensor([[713.3, 140.9, 0.1125, 0.225]] * 4))
    assert (model.dof_armature[:, drive.rotor_dof] > 0).all()
    for _ in range(25):
        obs, reward, *_ = env.step(torch.zeros(4, 6))
    assert torch.isfinite(obs["critic"]).all() and torch.isfinite(reward).all()
    assert (drive.motor_torque[:, :4].abs() > 0).any()


def test_m14_actuator_and_power_randomisation():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import env_cfg

    env = ManagerBasedRlEnv(
        env_cfg(num_envs=4, belts=True, actuators=True, stress=True, robot=True), device="cpu"
    )
    obs, _ = env.reset()
    drive = env.action_manager.get_term("drive")
    model = env.sim.model
    assert ((drive.pack_V >= 19.8) & (drive.pack_V <= 25.2)).all()
    assert len(set(model.dof_damping[:, drive.leg_dof[0]].tolist())) == 4
    assert (drive.torque_scale.min() >= 0.8) and (drive.torque_scale.max() <= 1.2)
    assert drive.transport_delay.max() <= 23
    for _ in range(10):
        obs, reward, *_ = env.step(torch.zeros(4, 6))
    assert torch.isfinite(obs["critic"]).all() and torch.isfinite(reward).all()
    assert (drive.bus_V <= drive.pack_V + 1e-6).all() and (drive.bus_V > 15).all()


def test_m14_dr_ramp_starts_at_nominal():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    kwargs = VARIANTS["locomotion"]()
    env = ManagerBasedRlEnv(env_cfg(num_envs=4, hard=True, **kwargs), device="cpu")
    env.reset()
    drive = env.action_manager.get_term("drive")
    assert env.common_step_counter == 0
    assert torch.allclose(drive.belt_params, drive.belt_scale.expand(4, 4))
    assert torch.allclose(drive.friction_scale, torch.ones_like(drive.friction_scale))
    assert torch.allclose(drive.imu_bias, torch.zeros_like(drive.imu_bias))


def test_m14_non_finite_world_is_reset():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    env = ManagerBasedRlEnv(env_cfg(num_envs=4, hard=True, **VARIANTS["locomotion"]()), device="cpu")
    env.reset()
    env.sim.data.qvel[1, 6] = float("nan")
    obs, reward, terminated, *_ = env.step(torch.zeros(4, 6))
    assert bool(env.termination_manager.get_term("nan")[1])
    assert torch.isfinite(obs["critic"]).all() and torch.isfinite(reward).all()
    for _ in range(3):
        obs, reward, *_ = env.step(torch.zeros(4, 6))
    assert torch.isfinite(obs["critic"]).all()


def test_m14_stress_slopes_follow_the_heading():
    import math

    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import env_cfg
    from bipe.m13.rl.mdp import ROBOT, _heading

    cfg = env_cfg(num_envs=16, robot=True, full_push=True, belts=True, actuators=True, stress=True)
    env = ManagerBasedRlEnv(cfg, device="cpu")
    env.reset()
    down = env.scene[ROBOT].data.gravity_vec_w
    h = _heading(env)
    lateral = torch.stack([-torch.sin(h), torch.cos(h), torch.zeros_like(h)], dim=-1)
    assert (down * lateral).sum(dim=-1).abs().max() < math.sin(math.radians(0.5))


def test_m14_finetune_variant():
    from mjlab.envs import ManagerBasedRlEnv

    from bipe.m13.rl.env_cfg import VARIANTS, env_cfg

    cfg = env_cfg(num_envs=4, hard=True, **VARIANTS["locomotion_ft"]())
    assert cfg.events["push"].params["impulse_Ns"] == (0.5, 3.75)
    assert cfg.rewards["yaw_rate"].weight == 2.0
    env = ManagerBasedRlEnv(cfg, device="cpu")
    env.reset()
    drive = env.action_manager.get_term("drive")
    assert not torch.allclose(drive.belt_params, drive.belt_scale.expand(4, 4))
