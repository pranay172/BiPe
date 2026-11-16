# M13: learned locomotion

Plan started 21 October 2026, after M12 phase C. The roadmap puts a
learned-locomotion milestone before the public simulation release, in the
style of Pollen Robotics' Microduck: a policy trained end to end with
reinforcement learning on a GPU simulator, exported as ONNX with a
manifest.

M13 trains a policy that balances, drives, turns and crouches the M10
robot on command, and evaluates it on the full M10 plant. The deliverables
are:
- a training plant that a GPU simulator can run, fitted to the full M10
  plant and validated against it;
- a vectorised training environment and a trained policy;
- the policy's evaluation on the full CPU plant, inside the supervised
  firmware gates, against the classical controller;
- an ONNX export with a manifest, and an inference budget for the Teensy.

As before, nothing is bought, built or powered. The results are a
validated simulation under stated assumptions, not a hardware-validated
design. The full M10 plant stays the evaluator: a result measured only on
the training plant is a training result, not an acceptance result.

## Starting point

The full plant (`bipe/m10/plant.py`):
- **Model.** Free base, 18 DoF (4 leg hinges, 2 shins, 2 wheels, 4 RO40
  rotors on the hip belts), 6 actuators. 26 geoms: convex hulls of the
  printed exterior parts with 101,533 mesh vertices, two cylinder tires,
  bumper pads.
- **Physics outside MuJoCo.** Between `mj_step1` and `mj_step2`, Python
  callbacks set the material contact laws (Winkler TPU tire, PA12 shell
  strip law, foam pads, ear cantilever, end-stop pads) through the
  solver rows. `MechanicalDrives` solves four 4:1 hip belts and two 1:1
  knee belts as finite-stiffness lash elements with a backward-Euler
  predictor on the full mass matrix, the RO40 and GL40 motor models
  (caps, torque-speed and voltage limits, FOC current lag and slew), and
  load-dependent bearing friction from the multibody reactions.
- **Timing.** Physics 0.25 ms, local leg loop 1 ms, body control 5 ms.
- **Speed.** A 2 s hold with sensors takes 41 s on one core here: about
  0.05× real time.

GPU simulators:
- **MJX** rejects the M10 MJCF (cylinder and mesh collisions, the mesh
  size), and the callbacks cannot run inside it.
- **MuJoCo Warp** 3.16 installs here and runs on the CPU device in this
  container (no GPU), including plane-cylinder contact, with the same
  result as CPU MuJoCo on a small test.
- **mjlab** 1.6.0 and **rsl_rl** 5.4.2 install. mjlab pins MuJoCo and
  MuJoCo Warp 3.11; the repository pins MuJoCo 3.14.
- **GPU.** Pranay's workstation has an RTX 5050 Laptop GPU (8 GB, M12
  phase A). The cloud sessions have none.

Earlier learning work (docs/history): M6 and M7 trained a residual PPO
policy (bounded torque corrections on top of the classical controller)
in native CPU rollouts, about 0.6 M transitions in total. M7's finding
was negative: a 3.6% median gain against a 10% gate. M13 differs in
three ways: the policy controls the robot alone, the simulator is
vectorised on the GPU, and the budget is two to three orders of
magnitude larger.

## Simulator choice: MuJoCo Warp with mjlab

MuJoCo Warp with mjlab and rsl_rl PPO, over MJX, because:
- **The recipe is proven on a robot like ours.** Microduck trains its
  gait with mjlab and rsl_rl PPO in 1–2 h at 4096 environments, exports
  ONNX, and is Apache-2.0. Following it keeps M13 comparable and gives a
  known-good starting configuration.
- **Collisions.** MuJoCo Warp supports more collision pairs than MJX
  (cylinders and convex meshes among them), so the training plant can
  keep cylinder tires, and the primitive shell is a fidelity choice,
  not a hard limit.
- **Testing without a GPU.** Warp kernels run on the CPU device, so the
  training plant's Warp parity tests run in the cloud sessions and in CI.
  GPU timing and training still need the workstation.
- **One framework.** mjlab and rsl_rl are PyTorch, and the motor model
  becomes vectorised torch code beside the simulator (as Microduck's
  XL330 model is). MJX needs JAX plus a separate PPO (Brax or our own),
  and MJX's own Warp backend points the same way.

Costs, accepted:
- mjlab is young and its API moves. The RL stack is pinned in its own
  environment (`.venv/rl`, `requirements-rl.lock`), as M6 did.
- mjlab's MuJoCo 3.11 differs from the repository's 3.14. The training
  plant's MJCF must load in both; the full plant and every acceptance
  run stay on 3.14 in `.venv`.

## Rules

These carry over from the earlier milestones:
- **Balance acceptance.** Free base, finite motor limits, physical
  contact. Nothing in the training plant or the policy's evaluation
  holds the robot up through pose constraints or clamps.
- **Kinds of evidence.** Training-plant results, full-plant results and
  hardware results stay separate. Every number says which plant it was
  measured on.
- **The full plant decides.** Acceptance, the policy's comparison with
  the classical controller and any adoption decision use the full M10
  plant on CPU MuJoCo with the C supervisor.
- **Held-out evaluation.** Final evaluation cases take no part in
  training, tuning or checkpoint selection (as in M6 and M7).
- **Records.** Save configurations, seeds and traces with each output.
  Training runs save their effective configuration, plant identity and
  checkpoints.
- **Safety.** The supervisor's gates stay outside the policy, as they do
  around the classical controller.

## Phases

| Phase | Content | Exit |
|---|---|---|
| A. Training plant and fit | A primitive-geometry MJCF with the M10 inertias and kinematics; the motor models as vectorised code; contact, friction and drive parameters fitted to traces from the full plant; validation open-loop and in closed loop with the classical controller; MuJoCo Warp parity; GPU throughput | The training plant passes the validation bands against the full plant, matches CPU MuJoCo under Warp, and reaches the throughput target on the workstation |
| B. Environment and first policy | mjlab task: observations from the sensor channels the robot has, actions through the firmware's joint law, commands (speed, yaw rate, height), rewards, terminations, domain randomisation from the M10 variant families. Train balance, drive, turn and crouch | A policy that tracks commands in the training plant, with its training curves and configuration |
| C. Full-plant evaluation | The policy runs in the full plant through the sensor model and the C supervisor. M10 acceptance families, M12 speed cases, pushes, slopes and the variants, against the classical controller on identical jobs | Acceptance results for the policy beside the classical controller, with the sim-to-sim gap measured |
| D. Export and inference budget | ONNX and a manifest (observation and action contract, scaling, plant identity); parity between the ONNX, torch and a portable C forward pass; a Teensy 4.0 inference time estimate inside the control loop budget | Parity within a stated tolerance, and a timing estimate with its basis |
| E. Exit | Results, the adoption decision (classical, policy, or both selectable) and README status. Get-up and expressive moves are later work unless decided otherwise | Results and records saved, decisions signed off |

Detailed plans for phases B to E are written when each phase starts.

## Decisions for the user

Before phase A:
- **Simulator.** MuJoCo Warp with mjlab and rsl_rl (recommended, above),
  or MJX with JAX and Brax PPO.
- **Scope of the first policy.** Either:
  - **the M10 scope** (balance, drive, turn and crouch on command),
    which the classical controller already does, so phase C has a
    direct comparison (recommended); or
  - that plus **get-up and expressive moves** in M13, which need the
    shell contacts to be faithful during falls and a longer phase B.
- **What the policy commands.** Either:
  - **leg joint targets plus wheel velocity targets** at 50 Hz, through
    the 1 kHz firmware joint law (equal to moteus) for the legs and a
    wheel velocity loop. This is how Microduck and most wheel-legged RL
    work it; the torque limits stay in the motor model (recommended); or
  - **all six torques** at 200 Hz, as the classical body loop does.
    That is closer to the current firmware path, but harder to train
    and to transfer.
- **Where it runs later.** On the Teensy, inside the supervised core
  (the M12 follower's choice), or on the Pi Zero 2 W sending commands.
  Phase D measures the Teensy budget either way; the choice is needed
  before hardware, not before phase A.

User decisions (21 October 2026): the plan as recommended. MuJoCo Warp
with mjlab and rsl_rl; the first policy covers the M10 scope (balance,
drive, turn, crouch); it commands leg joint targets and wheel velocity
targets at 50 Hz. Where it runs is decided before hardware.

Phase A decision (21 October 2026): the belt-lash pitch limit cycle is
covered by domain randomisation, not modelled. The training plant stays
lash-free. Phase B randomises the joint encoder offset and lag, and phase
C's full-plant evaluation shows whether that is enough.

## Phase A plan: training plant and fit (21 October 2026)

Goal: a plant that MuJoCo Warp can run in thousands of parallel worlds
and that behaves like the full plant where locomotion cares: wheel
contact, balance dynamics, leg drives and motor limits.

1. **Training plant** (`bipe/m13/plant.py`, profile `m13`).
   - Same kinematic tree, frames, masses and inertias as the full plant,
     generated from `config/robot.json` through `bipe.m10.plant
     .parameters`, so a CAD change reaches both.
   - Drives: the RO40 rotors become armature on the leg hinges
     (rotor inertia × 4²). The hip and knee belts start rigid; a belt
     spring (a fixed tendon with stiffness between a rotor hinge and its
     output) is kept as a fit option if rigid belts miss the validation
     bands.
   - Collisions: cylinder tires; the shell as a handful of boxes and
     capsules fitted to the B3 convex hulls (enclosing them, with the
     largest gap recorded); bumper pads as boxes. Only tires touch the
     ground in normal driving; the shell matters for terminations and
     falls.
   - Contact: MuJoCo soft contact with solref and solimp fitted to the
     tire's Winkler law over the load range (static load to the push and
     landing peaks), and the M10 friction coefficients.
   - Friction: joint damping and frictionloss, with the load-dependent
     bearing term replaced by its value at the operating load (fitted).
   - Timestep: the largest of 1, 0.5 and 0.25 ms whose results stay
     inside the bands; decimation to the policy rate.
   - The MJCF loads in MuJoCo 3.11 and 3.14, with no callbacks.
2. **Motor and joint-law model** (`bipe/m13/drives.py`).
   - The RO40 and GL40 equations from `MechanicalDrives`: command caps,
     torque-speed and voltage limits, FOC current lag and slew,
     efficiency. One NumPy implementation for CPU MuJoCo and one torch
     implementation for training, with a parity test between them and
     against `MechanicalDrives`' motor functions on the same inputs.
   - A CPU drive class with the `MechanicalDrives` interface (`forward`,
     `apply`, `step`, `metrics`), so `bipe.moving_experiment.simulate`
     runs the classical controller on the training plant unchanged.
3. **Fit data from the full plant** (`artifacts/m13/fit/`).
   - Closed-loop traces with the classical C controller: hold, drive
     and turn to the M12 envelope, crouch, pushes, a slope, with the
     torques, states and contact forces saved at 1 kHz.
   - Excitation: the same cases with a chirp and random steps added to
     the command and to the leg references, to cover more of the state
     space than the controller's normal use.
   - A fit split and a held-out validation split.
4. **Fit** (`bipe.m13.fit`, `artifacts/m13/fit/`).
- Five excitation runs on the full plant with the C core and sensors:
  smooth random speed (to 1.0 m/s), yaw rate and posture, two with
  pushes, and a quiet hold. 1 kHz state, requests and phase currents.
- Open-loop window replay was tried first. It does not constrain the
  parameters that matter: the leg torques come from a stiff 1 kHz PD,
  so the joints diverge in open loop within 50 ms, and the search
  absorbed that into friction at its bounds. It is kept as a diagnostic.
- The fit uses closed loop instead: the same C controller, sensors and
  commands on both plants, scored on the 1 kHz RMS difference of pitch,
  pitch rate, speed, yaw rate, joint angles and wheel speeds.
  - The friction rows need their own reference (about 2 ms): with the
    normal's direct damping the tires crept at 0.03 m/s with the wheels
    still.
  - A coordinate search moved the friction rows to 2.18 ms and the hip
    frictionloss to 4.25 mN m.
  - **Tire contact chatter.** The first fitted tire (direct reference
    −2e6, −371.5) chattered at 330–400 Hz: normal force 19.4 ± 7.8 N, with
    moments of no contact. That made a 3–4 mm/s hold drift and cost the
    delay margin. A scan of stiffness and damping in a sensor-model hold
    found chatter-free settings only at high damping (−5e5/−700 and
    −1000; −1e6/−1000 to −1200 at 1 ms; −1e6/−850 and −900 chatter; −2e6
    chatters at every damping tried). **−1e6/−1100 is adopted**, the
    middle of the clean band. With it the hold has no drift, and the
    normal force is 19.37 ± 0.01 N. The clean band is narrow, so the Warp
    parity check has to confirm it in float32.
  - **Post-push wheel hop.** With −1e6/−1100 the wheels hopped at about
    20 Hz for 2 s after the 2 N·s push in `push_drive_+2` (288 ms
    unloaded between 4 and 6 s). The full plant's tires stay loaded
    (16.5–22.7 N). The full plant's material contact is about
    −2.9e5·diagA at rest, with damping ratio 0.36, so the direct
    reference was 3.5 times stiffer than the tire. Direct references
    near the physical stiffness chattered or hopped (−2.9e5/−270 to
    −5e5/−500). **The mass-scaled reference [0.002, 1] is adopted**
    (MuJoCo's default form, critically damped). It keeps the tires loaded
    after the push and has a 0.24 N spread in drive. It also scales with
    the contact's effective mass, so mass randomisation does not move it
    out of a narrow band.
  - **Delay margin and the rolling row.** At 1.2 m/s with 15 ms delay
    the training plant fell into an 8.8 Hz limit cycle (wheel torque
    spread 0.21 N m against 0.002 on the full plant). Delay sweeps on the
    drive step: the full plant holds at 16.25 ms and falls at 17.5 ms,
    the training plant limit-cycled from 12.5–13.1 ms. Running the
    training plant with noslip (as the full plant does) restored the
    margin, and so did dropping the condim-6 rolling row. Without noslip
    that row is a soft torsion spring between wheel and ground. Stiffer
    or less damped friction rows, impratio 10–30, a 0.25–0.5 ms step and
    the friction values did not help. MuJoCo Warp has no noslip, so
    **the tire contact is condim 4** (sliding and torsional). The rolling
    resistance (about 1 mN m per wheel) is left out: as wheel
    frictionloss it also sticks at rest, and it nearly doubled the late
    pitch wobble after hold pushes.
  - The knee lash offset and the tire changes took the score from 1.13 to
    0.54 on the fit runs and 0.55 on the held-out run (0.69 and 0.98 with
    the −1e6/−1100 condim-6 tire).
  - Closed-loop RMS against the full plant: pitch 0.18–0.26°, speed
    0.002–0.016 m/s, yaw rate 0–0.05 rad/s, hip 0.02–0.48°, knee
    0.26–0.48°. The joint errors are largest while the posture moves.
  - The half-timestep cases had been running at 1 ms: the MJCF took its
    step from the training config, ignoring the case's override. Fixed;
    they now run at 0.5 ms.

**Closed-loop validation** (`bipe.m13.validate`, `artifacts/m13/validation/`).
135 cases: 91 from the M10 acceptance (the robustness matrix without the
belt lash and stiffness variants, and the inherited cases) and the 44 M12
speed cases, all with the C core and the sensor model.
- **Pass and fail agree in 132 of 135**, and in all 91 M10 cases.
- The traces agree closely. Median RMS difference of pitch error is
  0.19° (90th percentile 0.35°) on the M10 cases and 0.15° (0.24°) on the
  speed cases. Speed is 0.010 m/s (0.019) and 0.005 m/s (0.010).
- **The three disagreements** (all speed cases):
  - `delay_15ms_noise_x2_replay_corner_90` falls at 8.9 s on the training
    plant, in the braking turn at the corner. The full plant passes (peak
    pitch 12.3°). The training plant's yaw rate overshoots there (0.72
    against 0.56 rad/s). It passes at 12.5 ms. With noslip it survives at
    15 ms on condim 4 and passes on condim 6, so the soft sliding and
    torsional rows cost this last margin; MuJoCo Warp cannot remove them.
    The drive step and weave under the same variant now match (peak pitch
    13.7° against 13.5°).
  - `slope_up_10` fails the gate on the training plant by a hair (its
    speed metrics are within 1% of the full plant's).
  - `slope_up_4` passes on the training plant where the full plant just
    fails.
- **Band failures:** 27 of 135 cases miss at least one 25% band (32 with
  the condim-6 tire, 79 before the chatter fix). Most are peak wheel
  torque in the combined and mismatch cases (15).
- **Lash limit cycle.** The full plant keeps a 0.5–1° pitch limit cycle
  from belt lash. The lash-free training plant mostly lacks it.
- The `wheel_lag_25ms` cases fall on both plants, as recorded in M12
  phase B.

**Warp parity** (`bipe.m13.parity`, `tools/m13_warp_parity.py`,
`artifacts/m13/parity/`). A 16 s `push_drive_+2` run on the training plant
in CPU MuJoCo 3.14 was replayed in 12 windows of 0.1 s, with the recorded
controls and frictionloss, under MuJoCo Warp on the CPU device and under
CPU MuJoCo of the same version. Both MuJoCo Warp 3.11 (mjlab's pin) and
3.16 were run.
- CPU MuJoCo 3.11 and 3.16 reproduce the 3.14 recording to 1.4e-7: the
  MJCF behaves the same across versions.
- **Warp against CPU MuJoCo** (−1e6/−1100 tire): at most 5e-5 in steady
  contact; 1.5e-2 rad of wheel angle in the window after the push, where
  the wheels hopped on both simulators and the hop timing diverged.
- **With the adopted [0.002, 1] tire, condim 6:** at most 7e-5 in 19
  windows and 9.3e-4 in one (10.04 s) on the CPU device; on the GPU 1.2e-3
  in that window and under 1e-4 elsewhere.
- **With the adopted condim-4 tire** (recording redone): at most 5.8e-5
  in all 20 windows, on Warp 3.11 and 3.16 (CPU device).
- **GPU** (workstation, RTX 5050 Laptop 8 GB, Warp 3.11, CUDA 13), on
  the condim-4 tire: at most 5.8e-5 in all 20 windows. Throughput 323k
  physics steps per second at 1,024 worlds, 1.11M at 4,096 and 1.73M at
  8,192 (about 29 simulated hours per minute at 8,192 worlds; the
  condim-6 tire ran 828k at 8,192).

**Phase A exit** (user sign-off, 21 October 2026): accepted with the
gaps as recorded above. The braking turn under 15 ms delay and the
`slope_up_10` hairline are accepted; phase B randomises the transport
delay past 15 ms and the joint encoder offset and lag (for the belt-lash
limit cycle). The 25% band misses (27 cases, mostly peak wheel torque in
the combined and mismatch cases) are accepted as training-plant gaps;
phase C measures the policy on the full plant.

## Phase B plan: environment and first policy (21 October 2026)

Goal: a policy that balances, drives, turns and crouches on command in
the training plant, trained on the workstation GPU, with its training
curves and full configuration saved. Training-plant results only; the
full plant judges it in phase C.

**Environment** (`bipe/m13/rl/`, mjlab task `Bipe-M13-Flat`, in `.venv/rl`).
- The training-plant MJCF (profile m13) at a 1 ms step; the policy acts
  every 20 ms (50 Hz).
- The motor and drive models from `bipe/m13/drives.py` in torch, run every
  physics step: the hip and knee drives through the firmware joint law
  (the 1 kHz PD of the classical controller, equal to moteus position
  mode) with the RO40 torque, current and speed limits; the wheels through
  a velocity loop into the GL40 model with its FOC lag and limits. Belt
  loss as per-step frictionloss, as in phase A.
- A parity test runs the torch drives against `bipe.m13.plant.Drives` on
  recorded inputs (tolerance 1e-5) before any training.

**Actions** (as decided): four leg joint targets (hip and knee per side)
as offsets from the commanded posture, and two wheel velocity targets.
Scaled to the joint ranges and the wheel speed limit; clipped.

**Observations.**
- Actor (what the robot measures): gyro, accelerometer (or the gravity
  vector from the attitude filter), joint encoder angles and speeds,
  wheel speeds, the last action, and the commands. All through the
  sensor model's noise and transport delay. No body velocity or height.
- Critic (privileged, training only): adds true body velocity, height,
  tire contact forces, and the randomised parameters.

**Commands.** Forward speed −0.5 to 1.2 m/s with the 1.25 m/s² envelope
acceleration, yaw rate within the M12 replay range (about ±1 rad/s),
posture label 30–65 (the M10 crouch range). Sampled per episode with
random step changes, plus zero-command episodes for holds.

**Rewards** (weights tuned in the first runs and recorded): speed and
yaw-rate tracking (exponential kernels), posture (axle height)
tracking, body pitch near the posture's balance value, penalties on
action rate, joint torque and wheel torque, electrical power, joint
limit proximity, and a termination penalty.

**Terminations.** The physical limits used in acceptance (|pitch| or
|roll| past 20°, shell contact with the ground, joint limits), and a 20 s
time limit.

**Domain randomisation**, from the M10 variant families and the phase A
decisions:
- body mass ±15 %, COM x ±5 mm and z ±3 mm;
- tire friction 0.5–1.0;
- transport delay 5–20 ms (past the 15 ms variant) and sensor noise
  0.5–2× the sensor model;
- wheel FOC lag 2–25 ms; motor torque constant ±10 %;
- joint encoder offset ±0.5° and lag 0–5 ms (the belt-lash limit cycle);
- pushes up to 2.5 N·s; slopes to ±10° (the M10 range).

**Training.** rsl_rl PPO, 4,096 worlds, on the workstation (remote session,
M13 worktree only). Curriculum: holds and posture first, then speed and
yaw rate, then the full randomisation. Three seeds of the final
configuration. Checkpoints, TensorBoard logs and the exact config go to
`artifacts/m13/training/<run>/`; checkpoints larger than a few MB stay
on the workstation, with their hashes recorded in the repository.

**Exit.** In the training plant, over 1,000 randomised episodes per seed:
no terminations on commands inside the envelope; speed tracking RMS
under 0.1 m/s and yaw-rate RMS under 0.15 rad/s in steady segments;
posture tracked within 5 mm of axle height; pushes up to 2 N·s
recovered. The numbers are a first proposal and can move with your
sign-off.

**Decisions for the user (phase B).**
1. **Install mjlab and torch in the workstation's `.venv/rl`** (about 130
   packages, PyTorch 2.14 with CUDA). Needed to train; the M12 checkout is
   not touched.
2. **Velocity observation.** Either no velocity in the actor's
   observations, so the policy infers it from the IMU and wheel speeds,
   as Microduck does (recommended), or feed it the classical estimator's
   velocity, which ties the policy to that estimator.
3. **Exit thresholds** above, as proposed or changed.

User decisions (21 October 2026): the phase B plan as recommended.
mjlab and torch go into the workstation's `.venv/rl`; the actor sees no
velocity (it infers it from the IMU and wheel speeds); the exit
thresholds as proposed.

**First run** (21 October 2026, workstation, 4,096 worlds, 200 iterations,
7 minutes; run `first`, kept on the workstation). Training-plant result.
Mean reward rose from −0.8 to 44 and episode length from 10 to 828 of
1,000 steps, still rising at the end. Most episodes now reach the time
limit. Speed tracking error 0.10 m/s. Yaw-rate tracking failed (error
about 5 rad/s, yaw reward near zero), which suggests the policy spins;
to be checked in a replay before the next run. Throughput 46k policy
steps per second.

**Replay of the first policy** (22 October 2026, `bipe.m13.rl.replay`,
64 worlds, 10 s per fixed command, no pushes). Training-plant result.
Speed is tracked (0.55 and 1.00 m/s for 0.5 and 1.0 commanded), pitch
stays within 1°, and at most 2 % of worlds fall. Yaw rate is not tracked
at all: the policy spins at about +9 rad/s at standstill whatever the yaw
command (hold, ±0.5, crouch), with the wheels at about −24 and +30 rad/s,
and the spin falls to 7 and 5 rad/s at 0.5 and 1.0 m/s. It is a stable
spinning gait, not a sign error. Once spinning, the yaw kernel (std
0.4 rad/s) is flat, so nothing pulls the policy back. Fix for the next
run: a squared yaw-rate error penalty (weight −0.2) alongside the kernel,
widened to std 0.5. The crouch command also moved the legs only slightly;
this is to be checked again once yaw is fixed.

**Second run, `yawfix`** (22 October 2026, 1,500 iterations, 66 minutes).
Training-plant result. Final mean reward 53, episode length 780 of 1,000
steps, speed error 0.088 m/s, yaw-rate error 0.35 rad/s (5.4 before). In
the replay the spin is gone: hold gives −0.03 rad/s, and speed is tracked
(0.51 and 1.00 m/s). Turns go the right way but fall short (0.30 and −0.36
rad/s for ±0.5), with a 0.13–0.18 rad/s yaw wobble at zero command. Crouch
to posture 60 barely moves the legs, and 11 % of worlds fall. Both
shortfalls come from kernels that are flat at the errors involved: near
the yaw target the std-0.5 kernel gives little slope, and the posture
kernel (std 10 mm) gives nothing at the 31 mm depth change from 40 to 60.
Changes for the third run: yaw kernel std 0.25 (the squared penalty still
covers large errors), posture kernel std 15 mm, plus a squared depth-error
penalty (−0.05 per cm²).

**Third run, `posture`** (22 October 2026). It stopped at iteration 1358 of
1,500 because the session on the workstation that owned it closed; the
replay below uses checkpoint 1300. Training-plant result. With axle depth
added to the replay, both this policy and `yawfix` track the commanded
posture within 1.5 mm (crouch: 36.5 mm against 36; tall: 98–100 mm against
99), so the crouch was never failing: the leg targets already follow the
posture, and the policy's leg actions only need to correct around it. Arcs
improved (0.41 rad/s against 0.5, from 0.33). Turning in place did not:
0.33 for ±0.5, and 0.87 for 1.0, in both policies. The wheels themselves
are short of the kinematic speeds (about ±1.2 rad/s where 0.5 rad/s needs
±1.4 on a 0.2 m track with 36 mm wheels). The commanded differential is a
few hundredths of an action unit at a scale of 40 rad/s, under the P-only
wheel speed loop. This policy also leans 2–4° nose-down when standing; the
level kernel (std 0.2 rad) is nearly flat at that angle.
Changes for the fourth run: the firmware adds the commanded speed and yaw
rate's rolling wheel speeds (v/r ∓ ωW/2r) to the wheel targets, so the
policy learns a residual around rolling; the level kernel goes to std
0.1 rad.

**Slopes** (added 22 October 2026, after the fourth run was launched).
A slope is a per-world tilt of gravity, up to 10° in a random direction
(30 % of episodes flat), drawn at each reset (`mdp.slope`). The robot
touches nothing but the ground plane, so this is the same physics as a
tilted plane. The accelerometer and bias forces read the per-world
gravity, and pitch, roll and the tipping limit follow the true vertical.
A test drops the robot clear of the ground and checks that each world's
centre of mass falls along its own gravity. The replay runs on the flat
ground; the exit evaluator (`bipe.m13.rl.evaluate`) includes slopes.

**Fourth run, `feedforward`** (22 October 2026, 1,500 iterations, 68 minutes,
with slopes). Training-plant result. Final reward 47, speed error 0.091 m/s,
yaw-rate error 0.28 rad/s (the lowest so far). In the replay the policy's
wheel actions are near zero while driving, so the feedforward does the rolling.
Turning at 1.0 rad/s now reaches 0.95, but ±0.5 still reaches only +0.37 and
−0.31. Depth sits 3–4 mm low in every case, and the crouch case falls in 14 %
of worlds.

First exit evaluation (`bipe.m13.rl.evaluate`, 1,000 randomised episodes
with slopes, no pushes; separate 2 N·s push test):

| Check | Result | Exit | |
|---|---|---|---|
| Terminations | 178 of 1,000 | 0 | fail |
| Speed RMS (steady) | 0.083 m/s | < 0.1 | pass |
| Yaw-rate RMS (steady) | 0.433 rad/s | < 0.15 | fail |
| Posture p95 | 6.7 mm | < 5 | fail |
| 2 N·s pushes recovered | 66–70 % (14–18 % fall) | all | fail |

In the turn-in-place replay the wheels are well short of their targets:
the feedforward plus the policy asks for a 3.6 rad/s difference and the
wheels give 1.9. The cause is the P-only wheel speed loop, which settles
short under scrub and rolling losses. Changes for the fifth run: a PI wheel
speed loop (integral gain 0.5 N·m/rad, clamped at the GL40 peak torque,
its state given to the critic), and the evaluator now breaks terminations
down by delay, slope, noise and hub lag.

**Fifth run, `pi`** (22 October 2026, 3,000 iterations, 3 h 07 min under CPU
contention; checkpoint, configuration and evaluation in
`artifacts/m13/training/pi/`). Training-plant result. Replay: depth within
2 mm in every case, at most 3 % of worlds fall in any case, turns of ±0.5
reach ±0.40, and 1.0 reaches 0.90. Evaluation, judged on the 884 episodes that
ran to the time limit:

| Check | Result | Exit | |
|---|---|---|---|
| Terminations | 116 of 1,000 (18 % on 5–10° slopes, 8–9 % below) | 0 | fail |
| Speed RMS (steady) | 0.064 m/s | < 0.1 | pass |
| Yaw-rate RMS (steady) | 0.412 rad/s (0.386 with no turn commanded) | < 0.15 | fail |
| Posture p95 | 4.1 mm | < 5 | pass |
| 2 N·s pushes recovered | 64–70 % (5–13 % fall) | all | fail |

The yaw failure is heading drift while holding, not turn tracking. A hold
run with full randomisation (32 worlds, CPU) shows it is bimodal: most
worlds hold within 0.01–0.03 rad/s, and a few (6 of 32) run a 25 Hz
fore-aft oscillation of legs and wheels (hip ±7°, wheel speeds ±30 rad/s
relative to the shin), with yaw from its asymmetry. The policy's actions do
not alternate; the oscillation is in the 1 kHz firmware loops. The
oscillating worlds had 4–5 ms of encoder lag, and the wheel speed loop was
reading the wheel speed through that leg-encoder lag. The GL40 drive closes
its speed loop on its own encoder, so that lag does not belong there. With
the wheel loop on the current wheel speed, the same policy and seed
oscillate in 1 world of 32 (encoder lag 5 ms with 16.7 ms hub lag and a 6°
slope), and a flat sweep of encoder lag 0–5 ms shows no leg oscillation
(hip amplitude under 0.1°). The sixth run trains with this fix.

**Sixth run, `hubenc`** (22 October 2026, 2,000 iterations, wheel loop on the
hub encoder from 7761b79; checkpoint, configuration and evaluation in
`artifacts/m13/training/hubenc/`). Training-plant result. Replay: the heading
wobble is gone (|w| 0.01 rad/s holding) and a turn of 1.0 reaches 1.05.
Evaluation, judged on the 927 episodes that ran to the time limit:

| Check | Result | Exit | |
|---|---|---|---|
| Terminations | 73 of 1,000 | 0 | fail |
| Speed RMS (steady) | 0.045 m/s | < 0.1 | pass |
| Yaw-rate RMS (steady) | 0.082 rad/s (0.049 with no turn commanded) | < 0.15 | pass |
| Posture p95 | 4.6 mm | < 5 | pass |
| 2 N·s pushes recovered | 87–94 % (6–13 % fall) | all | fail |

Tracking now meets the exit. The terminations concentrate in the hardest
randomisation: 13.5 % of episodes on 5–10° slopes (3.7–3.8 % below 5°),
10.3 % at 15–21 ms transport delay (5–6 % below) and 12.1 % at 10–25 ms hub
lag (5–6 % below); noise scale makes no difference. Recovered pushes settle in
0.83–0.90 s. The next run targets steep slopes, long delays and push recovery.

**Fall diagnosis and start fixes** (23 October 2026, cloud, CPU). The sixth
policy on 256 training-plant episodes without pushes fell 16 times (6.3 %).
A third of the falls came in the first second, from two faults in the start
rather than in the policy: on a slope the robot started in the flat-ground
pose, so it leaned off balance by the slope angle, and the firmware's sensor
history started at zeros, so for the first 5–20 ms the policy saw no gravity
and the leg loops saw zero joint angles. Half were shell contacts in deep
crouch (posture 55–65) on 5–10° slopes: at posture 65 the skid pad clears the
ground by 8 mm when balanced on flat ground and by 1–3 mm on a 9–12° slope.
Now the slope event turns the reset pose about the axle so the robot starts
balanced against the true vertical (the cross slope stays as roll), and a
reset world's history is filled from its first reading. With these, the same
policy and seed fall 5 times (2.0 %): three skid-pad contacts at posture
60–65, and two tips moving off on a slope at 15–19 ms of delay. Both fixes
apply to training and evaluation alike.

**Seventh run plan.** Fine-tune from the sixth policy (`hubenc`
model_1999) with a harder training mix, `env_cfg(hard=True)`: 15 % flat
episodes instead of 30 %, slope tilts drawn as 10° u^0.5 (three quarters
past 5°), and training pushes 0.5–3 N·s instead of 0–2.5. The evaluation
keeps the plan's distribution and the 2 N·s push test. The sixth policy is
evaluated again with the start fixes first, as the baseline.

**Sixth policy again, with the start fixes** (23 October 2026, same
evaluator at 17b8c8f; `artifacts/m13/training/hard/baseline_hubenc_evaluation.json`):
38 of 1,000 episodes end early (73 before the fixes), 10 % on 5–10° slopes;
speed 0.051 m/s, yaw rate 0.081 rad/s, posture p95 4.4 mm; 2 N·s pushes
86–96 % recovered (3–14 % fall).

**Seventh run, `hard`** (23 October 2026, fine-tuned from the sixth policy
for 1,000 iterations on the harder mix, 41 min; checkpoint, configuration
and evaluation in `artifacts/m13/training/hard/`). Training-plant result.
Replay: no falls in any case but drive at 0.5 m/s (2 % of worlds), depth
within 1.6 mm, turns of +0.5 and 1.0 reach 0.50 and 1.00 (−0.5 reaches −0.45).

| Check | Result | Exit | |
|---|---|---|---|
| Terminations | 27 of 1,000 (6 % on 5–10° slopes, 5 % at 10–25 ms hub lag) | 0 | fail |
| Speed RMS (steady) | 0.030 m/s | < 0.1 | pass |
| Yaw-rate RMS (steady) | 0.087 rad/s (0.049 with no turn commanded) | < 0.15 | pass |
| Posture p95 | 4.2 mm | < 5 | pass |
| 2 N·s pushes recovered | 92–95 % (5–7 % fall), settling in 0.83–1.15 s | all | fail |

**Phase B exit decision (23 October 2026).** The user relaxed the exit to at
most 3 % terminations over the 1,000 randomised episodes and at least 90 %
of 2 N·s pushes recovered, with the tracking thresholds unchanged, and moved
to phase C. The remaining failure corners (steep slopes at the deepest
crouch, the longest hub lags) are judged in phase C against the classical
controller instead. The seventh policy (`hard`, model_2998) passes: 2.7 %
terminations, speed 0.030 m/s, yaw rate 0.087 rad/s, posture p95 4.2 mm,
pushes 92–95 % recovered. It is the policy phase C evaluates.
`bipe.m13.rl.evaluate` checks the relaxed exit.

## Phase C plan: full-plant evaluation (23 October 2026)

**Goal.** Run the seventh policy in the full M10 plant through the same
sensor model, timing and supervisor as the classical controller, on the
M10 acceptance families and the M12 speed cases, and report both
controllers side by side. Measure the sim-to-sim gap between the training
plant and the full plant.

**Policy controller** (`bipe/m13/policy_control.py`). A controller with the
interface `simulate()` already drives (`command`, `gate`, `leg_torques`,
`feedforward`), so every runner takes it unchanged:
- **Observations** as in training, built from the full plant's sensor
  model: the delayed gyro and accelerometer packet (anti-alias filter,
  bias, noise, quantised encoders, the configured transport delay), leg
  angles relative to the posture reference, leg and wheel speeds from the
  encoders as the classical local loop derives them, the last action and
  the command. Training read true joint speeds with noise; the
  encoder-derived speeds are one of the gaps this phase measures.
- **Inference** at 50 Hz with the torch checkpoint, held between updates.
- **Firmware** at 1 kHz, the same law as training: leg PD on the lagged
  encoders, the wheel PI speed loop on the hub encoder with the rolling
  feedforward. Torque requests go to the full plant's motor and belt
  models, so the motor limits are the full plant's.
- **Supervisor.** The C library's gate keeps running on the classical
  estimator's state, which runs alongside for that alone. A latched
  protection stops the policy exactly as it stops the classical
  controller.

**Cases.**
- The M10 acceptance families as `artifacts/m10/acceptance/` runs them,
  through `bipe.robustness.run_case`: holds, drives, crouches, 2 N·s pushes
  both ways, slopes through 10°, slope-hold rollback, backlash and belt
  variants.
- The 44 M12 speed cases (`artifacts/m12/speed/cases.json`), through
  `bipe.m12.speed.run_case`: arcs, steps, weaves, follower replays, and
  the 15 ms delay with doubled noise variants.
- The same pass criteria as the classical controller: the physical limits
  (pitch error 20°, roll 10°, joint limits, shell contact), recovery,
  rollback, and peak wheel torque and DC input current.
- The classical results come from the committed records when their
  runtime hash matches; otherwise both controllers are rerun on identical
  jobs.

**Sim-to-sim gap.** The policy on matched cases (hold, drive 0.5 and
1.0 m/s, turn, crouch, a 2 N·s push, a 6° slope) in both plants: tracking
errors, pitch and roll traces, wheel and leg torque RMS. This gives the
gap in the units the exit uses. The full plant has the belt lash that the
training plant left to randomisation, and this phase answers the phase A
question of whether that is enough. Two known differences are reported,
not changed: the full plant fails a run at 10° of roll (training: 20°),
and it measures pitch from the equilibrium.

**Compute.** The full plant runs at 0.25 ms on the CPU and needs no GPU,
so phase C runs in the cloud container (4 cores) and leaves the laptop to
M12. A smoke run of one hold case gives the runtime estimate before the
full set.

**Exit.** As in the phases table: acceptance results for the policy beside
the classical controller, with the sim-to-sim gap measured. There is no
pass threshold for the policy itself. Its results feed the adoption
decision in phase E.

**Decisions for the user (phase C).**
1. **Inference for phase C.** The torch policy in Python inside the loop,
   with the C forward pass left to phase D (recommended); or the C forward
   pass now.
2. **Supervisor.** The C gate on the classical estimator's state, running
   alongside (recommended); or no supervisor, which tests the policy
   alone but is not what the robot would run.
3. **Where it runs.** The cloud CPU (recommended, frees the laptop); or the
   laptop CPU.
4. **A policy that fails only in the corners phase B left open** (deepest
   crouch on the steepest slopes, longest hub lag) is reported as it
   stands, with no retraining inside phase C (recommended); or one
   fine-tune round on those corners before the comparison.

User decisions (23 October 2026): the phase C plan as recommended.

## Phase C results: the seventh policy on the full plant (24 October 2026)

The policy (`artifacts/m13/training/hard/model_2998.pt`) ran through
`python -m bipe.m13.full_plant acceptance|speed`, and the classical
controller was rerun on the same jobs and runtime
(`artifacts/m13/phase_c/*_classical`), since the committed M10 records
had drifted 0.03 % with later changes to `moving_experiment.py` and
`profiles.py`. These classical records predate the knee wheel-reaction
feedforward (1 November 2026, "Fallback fall while driving" below) and
were not rerun with it.

**M10 acceptance, 107 cases.** Survived the physical limits, and survived
and recovered (last 2 s pitch peak-to-peak within 2°, speed within
0.03 m/s):

| Family | Cases | Policy survived | Policy recovered | Classical survived | Classical recovered |
|---|---|---|---|---|---|
| Pushes (0.5 to 3 N·s, both ways) | 30 | 8 | 4 | 30 | 28 |
| Backlash | 12 | 12 | 7 | 12 | 12 |
| Belt range | 8 | 8 | 5 | 8 | 8 |
| Parameter mismatch | 16 | 16 | 13 | 16 | 15 |
| Slopes through 10° | 12 | 12 | 3 | 12 | 3 |
| Numerical | 1 | 1 | 1 | 1 | 1 |

The 28 acceptance-family cases (holds, drives, crouches, sensor, delay
and friction variants) are judged with the joint-error rule as well: the
policy passes 2 of 28, and 12 of 28 with that rule left out
(`passed_without_joint_error`); the classical controller passes 25 of 28.
The rule fails a joint 5° from the commanded posture, which the classical
controller holds by construction and the policy does not aim for, since
it places its legs to balance.

**M12 speed cases, 44.** The classical controller passes 38 of 44 on the
rerun, as in the committed M12 records. The policy passes none as judged,
and 24 of 43 with the joint-error rule left out (one case has no joint
error to exempt). It passes all 6 slope speed cases, where the classical
controller fails 2. Its drive steps peak at about 16° of pitch against
the cases' 15° limit, with 0.23 m/s of speed overshoot.

**Why the policy fails.**
- *Pushes.* After a push of 1 N·s or more the legs swing 25 to 33° and
  touch their stops, which the full plant counts as a fall. The training
  plant shows the same swing, but it only penalised the limits. Backward
  pushes reach the 20° pitch limit, and `push_low` reaches the full
  plant's 10° roll limit (training allowed 20°).
- *Recovery.* At rest the policy keeps a 3 to 5° pitch wobble in the full
  plant (23° with belt lash), against 0.1 to 0.25° in training, so many
  cases survive but do not meet the 2° recovery band.

**Interface diagnosis.** The training plant was run through the M10
harness with the policy, and the robot's sensor channels were put back
one at a time over exact readings with a 5 ms delay (late pitch
peak-to-peak after `sensors_push`):

| Observation | Late pitch pp |
|---|---|
| mjlab training env, nominal | 0.1–0.25° |
| Exact readings, 5 ms delay | 2.3° |
| …with the robot's IMU channel | 2.69° |
| …with the knee encoder on the motor side | 2.98° |
| …with the estimator's leg speeds | 4.45° |
| …with the estimator's wheel speeds | 4.33° |
| The full M10 harness | 7.45° |

The policy was trained on exact instantaneous joint and wheel speeds. The
robot reads 200 Hz finite differences of quantised encoders through the
velocity filters, and those channels account for most of the gap. Belt
lash then adds to it.

**Fixes for the next policy (commits 178c96e and f515811, not trained).**
`env_cfg(robot=True)` (task `Bipe-M13-Robot`, `train.py --robot`,
`evaluate.py --robot`) observes the robot's channels as `bipe.sensing`
builds them: IMU anti-alias filters every physics step, packets every
5 ms with bias and noise, quantised encoders with their periodic error,
the knee encoder on the knee-drive hinge, and leg and wheel speeds from
consecutive packets through the estimator's filters. The leg PD reads the
local encoder feedback (quantised, 1 kHz filtered differences). The
episode ends where the full plant fails a run: roll past 10° or a leg
joint past its range.

Check on the training plant, 2 N·s pushes, 16 worlds per case:

| Push | Exact channels, recovered | Robot channels and falls, recovered |
|---|---|---|
| Hold, forward | 100 % | 0 % |
| Hold, backward | 100 % | 12 % |
| 0.5 m/s, forward | 94 % | 0 % |
| 0.5 m/s, backward | 81 % | 25 % |

So the training plant now shows the full-plant failure, and a policy
trained on it has to learn to balance on the robot's channels and keep
its legs off the stops.

**Sim-to-sim gap** (`python -m bipe.m13.full_plant gap`,
`artifacts/m13/phase_c/gap/`). The policy on the training plant through the
same harness, sensor model and C supervisor as the full plant, on the 135
cases phase A compared (belt lash and stiffness variants skipped), in
phase A's bands:
- 69 of 135 cases agree on every compared metric, where the classical
  controller agreed on 108 in phase A. 39 of the 44 speed cases agree.
- Survival agrees: on the 91 M10 cases the policy survives 63 on the
  training plant and 60 on the full plant.
- Recovery does not: it recovers 2 of 91 on the training plant and 34 on
  the full plant, with a median late pitch peak-to-peak of 6.1° against
  1.8°. Of the 66 cases out of band, 54 are out on that metric.

So with the robot's channels in the loop the two plants agree on which
runs fall, as they did for the classical controller, and the full-plant
falls are an interface problem the training plant can show. On the
residual wobble the training plant run this way is the more pessimistic
of the two. That is not explained yet, and the retrained policy's gap run
will show whether it persists.

**Recommendation.** Retrain on `Bipe-M13-Robot`: fine-tune from the
seventh policy for about 1,500 iterations on the laptop GPU (the
observation layout is unchanged), and judge it with
`evaluate.py --robot` against the phase B exit, then on the full plant
with `bipe.m13.full_plant`. If the fine-tune cannot unlearn the leg swing,
train from scratch with the same task (3,000 iterations, about 2 hours).

## The robot-signal policy (24 October 2026)

**Runs.** The first two runs on `Bipe-M13-Robot` (a 1,500-iteration
fine-tune from the seventh policy, and 3,000 iterations from scratch)
both stalled with about half of the evaluation episodes falling. Three
500-iteration diagnostic runs (`train.py --variant falls|pipeline|narrow`)
showed that the robot's falls on exact readings were easy to learn, and
that the sensor pipeline with near-exact readings was the hardest of all.
That pointed to a bug: the robot servo loop read its encoder through the
randomised 0–5 ms encoder lag (a stand-in for belt lash in the exact-reading
task). On the light knee-drive hinge, a lagged collocated loop chatters:
knee-drive speed about 10 rad/s RMS and accelerometer ±20 m/s², against
0.3 rad/s and 0.7 m/s² without the lag. The robot's servo reads its own
encoder at 1 kHz with no added lag, and c176135 does the same. All the
earlier robot-signal runs had the bug.

**Policy `robotfix`** (`artifacts/m13/training/robotfix/model_4497.pt`):
a 1,500-iteration fine-tune from the seventh policy on the fixed task.

| Training plant | Robot channels and falls | Exact channels |
|---|---|---|
| Terminations, 1,000 episodes | 16 (8 shell, 4 pitch, 4 roll, all on 5–10° slopes) | 564 |
| Speed RMS | 0.041 m/s | 0.061 |
| Yaw-rate RMS | 0.112 rad/s | 0.213 |
| Posture p95 | 4.32 mm | 5.02 |
| 2 N·s pushes recovered | 60–83 % | 36–49 % |

On the robot channels it passes the phase B exit except for pushes (90 %).
It no longer does well on exact channels, which the robot never sees.

**Full plant** (`artifacts/m13/phase_c/robotfix_*`). Survived the physical
limits, and survived and recovered:

| Family | Cases | Seventh policy | `robotfix` | Classical |
|---|---|---|---|---|
| Pushes | 30 | 8 / 4 | 26 / 6 | 30 / 28 |
| Backlash | 12 | 12 / 7 | 12 / 0 | 12 / 12 |
| Belt range | 8 | 8 / 5 | 8 / 0 | 8 / 8 |
| Parameter mismatch | 16 | 16 / 13 | 16 / 0 | 16 / 15 |
| Slopes | 12 | 12 / 3 | 12 / 0 | 12 / 3 |

- On the 44 M12 speed cases `robotfix` has no physical failure (the
  seventh policy had 2, the classical controller 3). None pass as judged,
  as before: 27 fail only the survive-and-recover check, which includes
  the joint-error rule.
- Falls are nearly gone. What fails recovery now is speed: at rest the
  estimated speed sways with a peak of 0.038–0.047 m/s over the last 2 s,
  against the 0.03 m/s band (4 of 48 cases inside it; the seventh policy
  had 37 of 48). Pitch is inside its 2° band in 32 of those 48 cases.
- Sim-to-sim gap: survival agrees (78 of 91 on the full plant, 80 on the
  training plant), and the training plant's late pitch peak-to-peak is
  now 2.5° median against 1.6° (it was 6.1° against 1.8°).

**Next.** Train the rest drift out: a stand-still term in the reward (the
speed reward's 0.25 m/s width does not see a 0.04 m/s sway) and pushes
weighted toward the forward direction, which still fails most often,
then a fine-tune from `robotfix`.

## The stand-still policy (25 October 2026)

Variant `still` (045572e): the robot channels, a stand-still reward (an
exp kernel on forward speed, std 0.02 m/s, weight 1.0, when the command is
at rest), 30 % of commands at rest, and 70 % of pushes forward. Fine-tuned
1,500 iterations from `robotfix`; the policy is
`artifacts/m13/training/still/model_5996.pt` (836632f).

**Training plant, robot channels.** 30 of 1,000 terminations (shell 18,
rolled 11, tipped 1), inside the 3 % gate. Speed error 0.026 m/s, yaw
0.098 rad/s, posture 4.7 mm. Pushes recovered 83–92 % (target 90 %), up
from 60–83 %.

**Full plant** (`artifacts/m13/phase_c/still_*`). Survived, and survived
and recovered:

| Family | Cases | Seventh | `robotfix` | `still` | Classical |
|---|---|---|---|---|---|
| Pushes | 30 | 8 / 4 | 26 / 6 | 21 / 0 | 30 / 28 |
| Backlash | 12 | 12 / 7 | 12 / 0 | 12 / 3 | 12 / 12 |
| Belt range | 8 | 8 / 5 | 8 / 0 | 8 / 0 | 8 / 8 |
| Parameter mismatch | 16 | 16 / 13 | 16 / 0 | 16 / 4 | 16 / 15 |
| Slopes | 12 | 12 / 3 | 12 / 0 | 12 / 1 | 12 / 3 |

- The sway is gone. The rest speed peak over the last 2 s is inside
  0.03 m/s in 55 of 70 surviving cases (median 0.022 m/s; `robotfix` had
  10 of 75, median 0.039).
- Recovery now fails on pitch instead. The policy comes to rest leaning
  back: the mean pitch error over the last 2 s is −1.9° on the full plant
  (speed-case traces; `robotfix` −0.8°, classical −0.1°) and −1.0° on the
  training plant, so the peak passes the 2° band in 42 of 44 speed cases.
  The stand-still reward taught it to stop the wheels by holding a lean,
  and the full plant's bias doubles it.
- Pushes got worse: it falls on every forward push of 1.5 N s or more in
  all three modes (9 cases), where `robotfix` fell only on the four
  largest low-posture pushes. The backward lean leaves less margin
  forward.
- Speed cases: one physical failure (`push_fast_+2`), against none for
  `robotfix`, 2 for the seventh policy and 3 for the classical controller.
- Sim-to-sim gap: 82 of 135 cases pass (`robotfix` 67, seventh 69).
  Survival agrees in 89 of 91 (73 on each plant).

**Next.** Add the pitch error to the rest term (a kernel on pitch error at
rest, or a joint kernel on speed and pitch), keep the pushes forward, and
fine-tune again from `still` or `robotfix`.

## The rest-lean policy (26 October 2026)

Variant `lean` (89ebcca): `still` plus a kernel on the whole-body COM lean
from the tyre contact at rest (std 1°, weight 1.0), the full plant's pitch
error measured physically. Fine-tuned 1,500 iterations from `still`; the
policy is `artifacts/m13/training/lean/model_7495.pt` (4a793f5).

**Training plant, robot channels.** 10 of 1,000 terminations (`still` 30;
7 rolled on 5–10° slopes). Speed 0.029 m/s, yaw 0.085 rad/s (0.033 with no
turn), posture 4.17 mm. Pushes recovered 88–95 % (hold +2 0.89, hold −2
0.93, drive +2 0.88, drive −2 0.95): two of four still under 90 %.

**Full plant** (`artifacts/m13/phase_c/lean_*`). Survived, and survived
and recovered:

| Family | Cases | Seventh | `robotfix` | `still` | `lean` | Classical |
|---|---|---|---|---|---|---|
| Pushes | 30 | 8 / 4 | 26 / 6 | 21 / 0 | 16 / 16 | 30 / 28 |
| Backlash | 12 | 12 / 7 | 12 / 0 | 12 / 3 | 12 / 12 | 12 / 12 |
| Belt range | 8 | 8 / 5 | 8 / 0 | 8 / 0 | 8 / 5 | 8 / 8 |
| Parameter mismatch | 16 | 16 / 13 | 16 / 0 | 16 / 4 | 16 / 14 | 16 / 15 |
| Slopes | 12 | 12 / 3 | 12 / 0 | 12 / 1 | 12 / 3 | 12 / 3 |
| All | 79 | 57 / 33 | 75 / 6 | 70 / 8 | 65 / 51 | 79 / 67 |

- The lean is gone: the mean pitch error over the last 2 s of the speed
  cases is +0.3° (`still` −1.9°), and 37 of 44 are inside the 2° peak
  (`still` 2). The rest speed stays inside 0.03 m/s (38 of 44).
- Every case that survives now recovers, except three belt, two mismatch and nine slope
  cases. Backlash and mismatch are level with the classical controller.
- Pushes are the one family left, and they are worse: 16 of 30 survive.
  It falls on every forward push of 1.5 N s and more in all three modes
  (1 N s in the low posture), and on the largest backward ones (−2.5 in
  hold and drive, −2 and −2.5 low), all by pitching past 20°.
- Speed cases: 33 of 44 survive and recover (`still` 1, classical 34);
  one physical failure (`push_fast_+2`). None pass as judged, because of
  the joint-error rule, as for every policy.
- Sim-to-sim gap: 65 of 135 pass. Survival agrees in 90 of 91 (68 full,
  69 training) and recovery now agrees too, in 79 of 91 (56 full, 46
  training; it was 50 for `still`).

**Next.** Pushes. The training plant's 2 N s push fails 5–12 % of the time
while the full plant fails most pushes of 1.5 N s and more, so the training
push mix (0.5–3 N s, 70 % forward) is too gentle or too rare where it
counts. A fine-tune from `lean` with more and larger forward pushes is the
direct step; the leg swing into the joint stops seen in Phase C is worth
checking on the failing traces first.

## The push policy (26 October 2026)

**Diagnosis of the lean policy's push falls.** The full plant fails at a
20° pitch error, which is pitch less the M10 equilibrium for the measured
leg angles, so in effect a 20° lean of the COM from the axle. The training
plant tipped only at 20° of body pitch. In the mjlab environment the lean
policy brakes a forward push by swinging its legs, holding the body within
9° while the COM lean reaches 13° and more. The training push also differed:
a step in the base velocity, where the full plant applies a force on the
chassis for 0.1 s (`bipe.moving_experiment`).

Variant `push` (e92ded5) trains with both as the full plant has them:
`mdp.push_force` and a `leaned` fall at 20° of COM lean
(`evaluate --push-force` judges the same way). Fine-tuned 1,500 iterations
from `lean`; the policy is `artifacts/m13/training/push/model_8994.pt`
(a677200).

**Training plant.** Full-plant rules: 14 of 1,000 terminations, 2 N s
pushes 99–100 % recovered, passing every gate. Old rules: 22 terminations,
pushes 87–92 %.

**Full plant** (`artifacts/m13/phase_c/push_*`). It did not carry over:

| | `lean` | `push` |
|---|---|---|
| Pushes survived / recovered (of 30) | 16 / 16 | 17 / 1 |
| All physical cases (of 79) | 65 / 51 | 66 / 16 |
| Speed cases survive and recover (of 44) | 33 | 4 |
| Rest speed over 0.03 m/s (speed cases) | 6 | 31 |
| Gap: recovery agrees (of 91) | 79 | 53 |

Forward pushes now fall from 1 N s in all three modes (backward ones hold
except −2.5 N s low), and the rest sway is back.

**What this shows.** The lean policy under the full-plant push and fall
rules in mjlab does not fall at 1, 1.5 or 2 N s (32 worlds each, hold and
drive, both signs), yet on the same training plant run through the M10
harness (the gap suite) it falls on 14 of the 30 push cases, matching the
full plant (16 of 30 survive there). So the difference is between the mjlab
environment and the harness running the same plant and policy, not between
the plants. In the first 0.1 s of a 1.5 N s forward push the harness shows a
pitch error of −11° where mjlab shows a COM lean near −1°.

**Next.** Find that difference before more training: the same push from the
same state through mjlab and through the harness on the training plant,
compared channel by channel (the method of the Phase C interface study).
`lean` stays the best policy.

## The push gap found (27 October 2026)

The lean policy through the same 1.5 N s forward push in mjlab and in the
harness on the training plant moves nearly the same way: the legs swing
from about −162° to −130° (right hip) within 0.2 s in both, body pitch
stays within a few degrees, and the wheel torques match in size. The
difference was the fall measure. The M10 pitch error puts the COM over the
axle; `com_lean` measured from the tyre contact, 36 mm lower, which reads
about 1.5 times smaller (−10° against −16° at 0.2 s). Judged by the M10
equilibrium for the logged leg angles, the mjlab run reaches −21°, a fall.

5ef9713 measures `leaned` from the axle (the rest-lean reward keeps the
contact form it was trained with). The axle lean now matches the M10 pitch
error within 0.5° along the push. Under it the lean policy falls in mjlab as
on the full plant, most on forward pushes (16 worlds each):

| Push | hold + | hold − | drive + | drive − |
|---|---|---|---|---|
| 1 N s | 44 % | 6 % | 31 % | 13 % |
| 1.5 N s | 63 % | 19 % | 63 % | 25 % |

The `push` policy was trained against the contact measure, so it learned to
stay inside a limit the full plant does not use. A fine-tune from `lean`
with the axle measure is the next step.

## The pushaxle policy (27 October 2026)

Variant `push` with the axle-based `leaned` fall (5ef9713), fine-tuned
1,500 iterations from `lean`; the policy is
`artifacts/m13/training/pushaxle/model_8994.pt` (3fd852d). It trained on the
laptop at a677200 with 5ef9713's `mdp.py` change applied by hand (the SSH key
was locked); the code matches 5ef9713.

**Training plant.** Full-plant rules: 30 of 1,000 terminations, no falls on
2 N s pushes, recovered 79–100 % (hold 0.80 and 0.79, drive 1.00 and 0.99).
Old rules: 17 terminations, pushes fell 11–28 %.

**Full plant** (`artifacts/m13/phase_c/pushaxle_*`). Survived, and survived
and recovered:

| Family | Cases | `lean` | `pushaxle` | Classical |
|---|---|---|---|---|
| Pushes | 30 | 16 / 16 | 30 / 2 | 30 / 28 |
| Backlash | 12 | 12 / 12 | 12 / 10 | 12 / 12 |
| Belt range | 8 | 8 / 5 | 8 / 5 | 8 / 8 |
| Parameter mismatch | 16 | 16 / 14 | 16 / 12 | 16 / 15 |
| Slopes | 12 | 12 / 3 | 12 / 2 | 12 / 3 |
| All | 79 | 65 / 51 | 79 / 31 | 79 / 67 |

- No physical failure anywhere: all 79 acceptance cases and all 44 speed
  cases survive (`lean` failed 14 and 1, classical 0 and 3). It is the first
  policy to survive every push.
- Recovery is held back by the rest sway again: the speed peak over the last
  2 s is just above the 0.03 m/s band (median 0.031 m/s on acceptance, 0.030
  on the speed cases; 17 of 44 speed cases above it, `lean` 6). Pitch is
  fine: mean −0.2° at rest, 38 of 44 inside the 2° peak.
- Speed cases: 19 of 44 survive and recover (`lean` 33, classical 34).
- Sim-to-sim gap: survival agrees in 90 of 91 (82 full, 83 training);
  recovery agrees in 58 (33 full, 66 training), so the training plant is
  optimistic about the sway.

**Next.** A short fine-tune from `pushaxle` aimed at the sway alone: a
stronger stand-still term (the speed peak needs to drop by about 0.005 m/s),
keeping the full-plant push and fall.

## The settle policy (27 October 2026)

Variant `settle` (b638f9e): `push` with twice the stand-still weight.
Fine-tuned 1,500 iterations from `pushaxle`; the policy is
`artifacts/m13/training/settle/model_10493.pt` (2a172d3).

**Training plant.** It looks worse than `pushaxle` there: posture p95
6.1 mm (gate 5 mm), full-plant-rule pushes 72–82 % recovered with no falls,
25 of 1,000 terminations. The training plant underrates the rest sway
(recovery agreed in only 58 of 91 gap cases for `pushaxle`), so it was run
on the full plant anyway.

**Full plant** (`artifacts/m13/phase_c/settle_*`). Survived, and survived
and recovered:

| Family | Cases | `lean` | `pushaxle` | `settle` | Classical |
|---|---|---|---|---|---|
| Pushes | 30 | 16 / 16 | 30 / 2 | 30 / 30 | 30 / 28 |
| Backlash | 12 | 12 / 12 | 12 / 10 | 12 / 11 | 12 / 12 |
| Belt range | 8 | 8 / 5 | 8 / 5 | 8 / 4 | 8 / 8 |
| Parameter mismatch | 16 | 16 / 14 | 16 / 12 | 16 / 14 | 16 / 15 |
| Slopes | 12 | 12 / 3 | 12 / 2 | 12 / 3 | 12 / 3 |
| All | 79 | 65 / 51 | 79 / 31 | 79 / 62 | 79 / 67 |

- No physical failure in any case: all 79 acceptance and all 44 speed cases
  (classical: 3 speed-case falls, all at 25 ms wheel lag).
- Every push survives and recovers (classical 28 of 30).
- The sway is down: rest speed peak median 0.023 m/s on acceptance (14 of 79
  over 0.03; `pushaxle` 0.031 and 48) and 0.022 on the speed cases (6 of 44
  over; classical 6). Rest pitch −0.3° mean, 38 of 44 inside 2°.
- Speed cases: 36 of 44 survive and recover (classical 34). None pass as
  judged, because of the joint-error rule.
- Sim-to-sim gap: survival agrees in 90 of 91, recovery in 88 of 91 (69 full,
  72 training), the best agreement of any policy.

Against the classical controller it now trails only on belt range (4 of 8
recovered against 8) and by one case each on backlash and mismatch. It is
the M13 policy candidate.

## Phase C summary (29 October 2026)

User decision (29 October 2026): `settle` is the M13 policy, and the
belt-range shortfall is a follow-up item. Phase C is closed. Phases D
(export and inference budget) and E (exit) remain.

**The policy.** `artifacts/m13/training/settle/model_10493.pt` (2a172d3),
variant `settle`. It is the end of a chain of fine-tunes, each fixing one
full-plant shortfall:

| Policy | Commit | What changed | Full-plant result that drove the next step |
|---|---|---|---|
| `hard` (seventh) | b769fa7 | Training mix with steep slopes and big pushes | Pushes 8 of 30 survive: trained on ideal sensor channels |
| `robotfix` | bd2afe1 | The robot's sensor channels; full-plant roll and joint-limit falls | Pushes 26 of 30, but rest sway 0.04 m/s |
| `still` | 836632f | Stand-still reward | Sway fixed, but rests leaning back 2° |
| `lean` | 4a793f5 | Rest-lean reward | Recovers 51 of 79, but pushes 16 of 30 |
| `push` | a677200 | Chassis-force push, COM-lean fall (measured from the contact) | No gain: the fall measure was wrong |
| `pushaxle` | 3fd852d | COM-lean fall measured from the axle, as the M10 pitch error | Survives every case, but rest sway 0.031 m/s |
| `settle` | 2a172d3 | Twice the stand-still weight | Below |

**Full plant, `settle` against the classical controller** (records in
`artifacts/m13/phase_c/settle_*` and `*_classical`; the classical column
is from before the knee wheel-reaction feedforward of 1 November 2026):

| | `settle` | Classical |
|---|---|---|
| Physical failures, 79 acceptance + 44 speed cases | 0 | 3 |
| Pushes survived / recovered (of 30) | 30 / 30 | 30 / 28 |
| Acceptance cases recovered (of 79) | 62 | 67 |
| Belt range recovered (of 8) | 4 | 8 |
| Speed cases survived and recovered (of 44) | 36 | 34 |
| Speed cases passed as judged (of 44) | 1 | 39 |

The speed suite's joint-error rule (a joint within 5° of the commanded
posture) is built around the classical controller, which holds the legs at
the posture; the policy moves them to balance, so it fails that rule by
design (Phase C plan). Sim-to-sim: the training plant agrees with the full
plant on survival in 90 of 91 cases and on recovery in 88 of 91.

**What was learned.**
- The training plant is only as good as its interface to the policy. The two
  largest gaps were both in the interface, not the physics: the robot's
  sensor channels (5 ms packets, quantised encoders, finite-difference
  speeds) and the fall rules (the full plant's 20° limit is on the pitch
  error, the COM lean from the axle, not the body pitch).
- A bug in the robot-mode servo loop (encoder lag on a collocated loop,
  fixed in c176135) invalidated three training runs before it was found;
  channel-by-channel replays found it and the fall-rule gap.
- The training plant underrates the rest sway: it called `settle` worse than
  `pushaxle` while the full plant showed it much better. The full plant has
  to judge every candidate.

**Follow-ups.**
- Belt range: 4 of 8 cases recover (the training plant has no lash and rigid
  hip belts, so these cases are outside what it trains).
- The speed suite's joint-error rule needs a policy form before the speed
  cases can be passed as judged.
- Phase D: ONNX export with a manifest, ONNX/torch/C parity and the Teensy
  inference budget. Phase E: the adoption decision and README status.

## Phase D plan: export and inference budget (29 October 2026)

User decision (29 October 2026): proceed with phase D for `settle`
(`artifacts/m13/training/settle/model_10493.pt`).

**Export** (`bipe/m13/export.py` → `artifacts/m13/export/settle/`):
- `policy.onnx`: the actor as trained, observation normaliser included, so
  the graph takes the raw 25-value observation and returns the 6 mean actions
  (float32, opset 17, batch of one).
- `manifest.json`: the contract the firmware needs around the graph, taken
  from the code that ran in phase C (`bipe.m13.policy_control`):
  - the observation layout: gyro (3, rad/s), accelerometer (3, m/s²), leg
    encoders less the posture's nominal angles (4, rad), leg speeds (4,
    rad/s), wheel speeds (2, rad/s), last action (6), command (forward speed
    m/s, yaw rate rad/s, posture label scaled as (label − 47.5)/17.5);
  - the action law: leg targets = nominal(posture) + 0.5 × action[0:4],
    wheel speed targets = 40 × action[4:6] plus the rolling feedforward; the
    1 kHz leg PD (kp 8, kd 0.05) and wheel PI (kv 0.05, ki 0.5) with its
    clamp; the 20 ms policy period on the 5 ms tick; the command rate limits;
    the posture table;
  - identity: checkpoint, ONNX and plant hashes, the training variant, the
    commit, and the phase C records it was judged by.
- `SHA256SUMS`.

**Portable C forward pass** (`firmware/src/locomotion/locomotion.c`,
`firmware/include/locomotion.h`): C99, float32, no allocation, weights in a
header generated from the checkpoint by the export (not committed, like
`generated_config.h`). It is the forward pass only; wiring it into the
supervised image is phase E's adoption decision.

**Parity** (`artifacts/m13/export/settle/parity.json`): the double-precision
numpy actor that phase C ran, torch, ONNX Runtime and the C forward pass
(host gcc, through ctypes), on the observations recorded from full-plant
runs plus random observations over each channel's recorded range. Tolerance:
1e-4 on each action (float32 rounding through four layers is about 1e-6;
1e-4 of an action is 0.003° of leg target). Then closed loop: M10
acceptance cases rerun on the full plant with the C forward pass in place of
the numpy actor give the same pass/fail and metrics within the M10 bands.

**Inference budget** (`artifacts/m13/export/settle/budget.json`): the
multiply-accumulate count and memory, and a Teensy 4.0 (Cortex-M7, 600 MHz)
time estimate from an ARM build of the C forward pass if the toolchain can be
installed in the project, else from an operation-count model with its
basis stated. It is set against the M11 cycle budget (0.5 ms compute bound
in the 5 ms tick, 1.56 ms margin) and the 20 ms policy period. The board's
DWT timer replaces the estimate at the bench check.

Exit: parity within the tolerance, closed-loop agreement, and the timing
estimate with its basis.

## Phase D results: export and inference budget (29 October 2026)

Achieved results for the plan above. The records are in
`artifacts/m13/export/settle/` and `artifacts/m13/phase_d/`. The commands
are `python -m bipe.m13.export`, `make -C firmware locomotion
locomotion-arm` and `python -m bipe.m13.export_check
record|parity|budget|closed`.

**Export.**
- `policy.onnx` is the `settle` actor: 25 float32 inputs to 6 outputs,
  opset 17, normaliser included, sha256 `acedce42…`.
- `manifest.json` carries:
  - the observation layout and the action law;
  - the 1 kHz leg PD and wheel PI with its clamp (peak torque 0.495 N·m);
  - the 20 ms policy period on the 5 ms tick;
  - the command rate limits and trained ranges;
  - the 36-label posture table;
  - identity hashes: checkpoint `bfccb812…` (commit 2a172d3), the
    training plant, the sensors and the phase C records.
- `SHA256SUMS` covers both files.
- `tests/test_m13_export.py` rebuilds the ONNX graph and the C library
  from the checkpoint and checks both against the actor.

**C forward pass.**
- `firmware/src/locomotion/locomotion.c` and `firmware/include/locomotion.h`
  hold the forward pass: float32, stateless, no allocation, with a 1.9 KB
  stack of activations. The code is C99 and builds with the project's
  `-std=c11 -Werror -Wconversion` flags.
- The export generates the weights header
  `firmware/include/generated_locomotion_weights.h`, about 0.7 MB of text,
  which is gitignored. It carries the checkpoint hash, which
  `bipe_locomotion_checkpoint()` returns, and the runner refuses a library
  built from another checkpoint.
- `make locomotion` builds the host library for ctypes.
- `make locomotion-arm` builds a bare Cortex-M7 image (`-mcpu=cortex-m7
  -mfpu=fpv5-d16 -mfloat-abi=hard`, newlib `expm1f`).
- The ARM compiler is Debian's arm-none-eabi gcc 13.2, extracted into
  `.local-tools/root` as `make arm` expects. The build host could not reach
  PlatformIO's registry, so the Teensy toolchain (gcc 11.3) was not used.
- With `-std=c11`, gcc does not fuse the multiply and add (no
  `-ffp-contract=fast`), so the ARM and host builds do the same arithmetic
  in the same order.

**Parity** (`parity.json`): passed. The reference is the float64 numpy
actor that phase C ran.

| Implementation | Recorded (4600) | Random (2000) |
|---|---|---|
| torch (float32) | 6.5e-7 | 1.0e-6 |
| ONNX Runtime | 6.5e-7 | 8.8e-7 |
| C, host gcc through ctypes | 5.3e-7 | 1.0e-6 |
| C, Cortex-M7 image under unicorn | 3.3e-7 | 8.2e-7 |

- Each figure is the largest absolute action error. The tolerance is 1e-4,
  and the actions range up to 2.0.
- The recorded observations are every policy step of four full-plant M10
  cases: `push_drive_+2.5`, `sensors_combined`,
  `belt_hold_h0.45_n0.9_k1` and `backlash_turn_0.6`.
- The random observations are uniform over each channel's recorded range.
- The Cortex-M7 image was run instruction by instruction under unicorn on
  512 of the recorded and 500 of the random observations. It matches the
  host C within one float32 ulp (1.2e-7), the difference coming from newlib
  against glibc `expm1f`.

**Inference budget** (`budget.json`): this is an estimate. The Teensy's
DWT cycle counter replaces it at the bench check.

| | |
|---|---|
| Multiply-accumulates | 47,744 (448 ELU) |
| Weights and normaliser | 193 KB `.rodata`; code 5.1 KB with libm |
| Stack | 1.9 KB |
| Instructions per call (emulated, 120 calls) | 304,714 to 307,684 |
| … of which the MAC loops | 286,664 (6 instructions a MAC) |
| … of which `expm1f` | up to 15,950 |
| Time at 600 MHz (model) | 0.27 to 0.71 ms |

- The cycle model is the inner loop at 3 cycles a MAC at best (the
  single-accumulator add chain, 3-cycle FP latency) and 8 at worst (single
  issue with a load-use stall), with everything else at 1 to 2 cycles an
  instruction. It assumes the weights sit in DTCM with no wait states.
  Teensy 4.0's linker copies `const` data to RAM1 unless it is marked
  `PROGMEM`; the 193 KB fits in the 512 KB.
- Against the M11 budget:
  - the worst case is 1.4× the 0.5 ms compute bound assumed for the
    classical tick, but it fits in the 1.56 ms margin;
  - in the tick that runs the policy, sample to actuation is about 3.7 ms
    of the 5 ms if the policy step takes the place of the 0.5 ms bound, or
    4.2 ms if it is added on top;
  - it is 3.5 % of the 20 ms policy period.
- The action is held for 20 ms. If the bench measurement is tight, the
  forward pass can run after the tick's commands are sent, leaving the
  sample-to-actuation path alone. Four accumulators per row would also
  break the add chain, at the cost of a different summation order.

**Closed loop** (`closed_loop.json`; `artifacts/m13/phase_d/settle_c_acceptance`):
the M10 acceptance suite was rerun on the full plant with the C forward pass
(host build) in place of the numpy actor. It was compared with phase C's
`settle_acceptance`, leaving out the 8 ideal-sensing cases, which the policy
rejects in both runs.
- Cases compared: all 99 judged cases.
- Survival: all 99 agree. No case falls in either run.
- Verdicts: 96 of 99 agree. `combined_half_timestep` fails as judged in
  both runs, with a peak pitch error of 11.78° against 11.74°. Three cases changed `recovered`, all on the
  rest-sway band (late speed ≤ 0.03 m/s):
  | Case | Late speed, numpy → C (m/s) | Recovered |
  |---|---|---|
  | `backlash_drive_0` | 0.0301 → 0.0237 | no → yes |
  | `mismatch_body_mass_scale_0.85_combined` | 0.0302 → 0.0236 | no → yes |
  | `belt_hold_h0.1125_n0.225_k1` | 0.0163 → 0.0387 | yes → no |
- Metric bands: 95 of 99 are within phase A's bands
  (`bipe.m13.validate.BANDS`) on all six metrics. The four outside differ
  on the late pitch peak-to-peak or the travel:
  - two belt cases with no knee belt (`_k0`), which sway 8 to 10° in both
    runs and recover in neither;
  - `slope_down_4`, which recovers in neither;
  - `mismatch_body_com_x_m_-0.005_hold`, which recovers in both (0.84
    against 1.55° peak to peak).
- Peak transients agree closely: the largest differences are 0.63° of peak
  pitch, 0.03 N·m of wheel torque, 0.05 N·m of motor torque and 15 mm of
  rollback.
- The run is deterministic. A rerun of the first two flipped cases with the
  numpy actor (`artifacts/m13/phase_d/settle_numpy_rerun`) reproduces phase
  C exactly, metric for metric. So the flips come from the float32 forward
  pass's 1e-7-level difference, which the closed loop amplifies over the
  16 s, and not from a C error. The late rest sway sits near the band in
  these cases, and a perturbation that small moves it either way.

**Exit.**
- Parity is within the tolerance, by more than two orders of magnitude.
- The timing estimate is stated with its basis. It fits the tick margin and
  the policy period but not the 0.5 ms compute bound assumed for the
  classical controller.
- Closed-loop agreement holds for survival and peak transients. The late
  rest sway is sensitive enough that float32 rounding moves it. That
  sensitivity belongs to the policy, which is the known rest-sway weak spot,
  and not to the export. Bench sensor noise is orders of magnitude larger
  than 1e-7.
- Phase E (the adoption decision and README status) remains.

## Phase E: exit (30 October 2026)

User decision (30 October 2026): both controllers are selectable, and the
classical controller is the default until the bench check.

**Adoption.**
- The policy is `settle`
  (`artifacts/m13/training/settle/model_10493.pt`, 2a172d3), exported in
  `artifacts/m13/export/settle/`.
- Selection in simulation:
  - the M10 and M12 runners (`bipe.robustness`, `bipe.m12.speed`) keep the
    classical controller;
  - `bipe.m13.full_plant` runs the same cases with the policy, using the
    numpy actor or, with `--forward c`, the firmware forward pass;
  - `bipe.m13.policy_control.policy_in_loop` swaps the policy into any
    `simulate()` caller.
- Selection on the robot: the C forward pass
  (`firmware/src/locomotion/`) is not yet linked into the Teensy image.
  The hardware build wires it behind a switch, classical by default, and
  the switch should be a build or arming-time choice, not a runtime
  toggle. At the same step the DWT cycle counter replaces the phase D
  timing estimate. Where the policy runs, on the Teensy or on the Pi, was
  left open until hardware (21 October 2026); the phase D budget shows it
  fits on the Teensy.

**Results.**
- The achieved results are in the phase C summary, the phase D results
  and the README.
- The records are in:
  - `artifacts/m13/fit/`, `validation/`, `parity/`;
  - `artifacts/m13/training/`;
  - `artifacts/m13/phase_c/`;
  - `artifacts/m13/export/`;
  - `artifacts/m13/phase_d/`.

**Open items, carried to M14.**
- Belt range: 4 of 8 cases recover (classical 8).
- Rest recovery: 62 of 79 (classical 67). The late rest sway sits near the
  0.03 m/s band, which phase D shows is sensitive to perturbations as
  small as float32 rounding.
- The speed suite's joint-error rule needs a policy form.
- The policy is a seven-step fine-tune chain; M14 needs a single,
  reproducible from-scratch recipe.
- Get-up and expressive moves are later work (M15).

M13 is closed in simulation. Nothing is hardware-validated.

## PR review corrections (30 October 2026)

An independent review of PR #3 (`/mnt/project-files/reviews/pr3-review.md`)
found four tooling faults and two reporting gaps.

**Tooling, fixed.**
- `bipe.m13.full_plant` defaulted to the `hard` checkpoint and to
  `artifacts/m13/phase_c/<suite>`, which holds the `hard` records. A
  `settle` run without `--output` would have overwritten them.
  - It now defaults to `settle`, writing to `phase_c/<run>_<suite>`, or to
    `phase_d/<run>_c_<suite>` with `--forward c`.
  - It refuses a directory whose `policy.json` names another checkpoint or
    forward pass, which also covers `--resume`.
- `policy_in_loop` relied on forked pool workers. Under spawn (macOS) or
  forkserver (the Linux default from Python 3.14), the workers would have
  run the classical controller and recorded it as the policy.
  - The context now refuses any start method but fork, and the tools set
    fork.
  - Each run records `policy_steps`. `full_plant` stops with an error if
    any case other than the rejected ideal-sensing ones made none.
  - The committed records are unaffected: they ran under fork, and the
    ideal-sensing rejections show the policy was in the loop. A smoke rerun
    of `push_hold_+1` under the new checks (801 policy steps) reproduces
    phase C exactly.
- The observation recorder saved every 50 calls, so each worker's last
  steps were dropped (the committed 4,600 rows are 4 × 1,150). It now
  appends every call. Parity is unaffected: it holds on the observations
  recorded.
- Checkpoints load with `weights_only=True`.

**Reporting, corrected.**
- *The whole suite.* The 79-case figures leave out the 19 scenario cases
  (sensor, delay, friction, mass, wheel lag and combined). Of the 107 M10
  acceptance cases, the policy runs 98: it rejects the 8 ideal-sensing
  cases, and `unpowered` has no controller to compare. Over those 98:

  | | `settle` | Classical |
  |---|---|---|
  | Physical failures, 98 acceptance cases | 0 | 3 (`delay_15ms_noise_x2_combined`, `wheel_lag_25ms_combined`, `wheel_lag_25ms_drive`) |
  | Physical failures, 98 acceptance and 44 speed cases | 0 | 6 |
  | Recovered (of 98) | 80 | 83 |

  The C forward pass run recovers 82 of the same 98.
- *Single-run sensitivity.* The float32 and float64 actions differ by at
  most about 1e-6. Yet across the 99 judged cases, the late pitch
  peak-to-peak differs by up to 2.1°, and the late speed by up to
  0.031 m/s. That is the size of the recovery bands themselves. So a
  single run's rest verdict near the band is close to chaotic, and 80
  against 83 (or 62 against 67 on the matrix cases) is likely within that
  noise. Before M14 decisions, compare over several sensor-noise seeds or
  small perturbations per case.
- The numpy rerun first covered two of the three flipped cases. The third,
  `belt_hold_h0.1125_n0.225_k1`, is now rerun too
  (`artifacts/m13/phase_d/settle_numpy_rerun_belt`). It also reproduces
  phase C exactly, so all three flips come from the float32 forward pass.
- *Budget overrun.* The 0.71 ms worst case is 1.41× the 0.5 ms compute
  bound that M11 assumed for the tick, not only inside the 1.56 ms margin.
  The plan for the hardware build:
  - run the forward pass outside the sample-to-actuation path, after the
    tick's commands are sent, since its action is held for 20 ms; or split
    it over ticks;
  - keep the 193 KB of weights in DTCM, because from QSPI flash behind the
    32 KB cache the cycle model no longer holds, and check RAM1 against
    the rest of the image;
  - measure with the DWT cycle counter.

**Hardware wiring checklist** (the policy's adoption on the Teensy):
1. A selection switch, classical by default, chosen at build or arming
   time.
2. Output guards. The forward pass is unbounded: 1e6 in gives about 1e6
   out, and NaN in gives NaN out. In simulation, the plant limits hide
   this. Required:
   - clamp the leg targets to the joint limits;
   - clamp the wheel speed targets to the hub's speed limit;
   - a non-finite input or output hands over to the supervisor's
     protective path (or the classical controller), not to the motors.
3. Forward pass placement and memory, as above, and the DWT measurement.
4. A bench parity check of the Teensy build against `parity.json`, using
   the toolchain the image is built with (Teensy gcc 11.3).

**Follow-ups (low).**
- Move the interface constants (`DriveActionCfg` defaults, `posture_table`)
  to a torch-free module, so the deploy tools don't need the training
  stack.
- Add a test that `PolicyController` equals `DriveAction` on the same
  sensor rows.
- Move the `policy_control` tests from `hard` to `settle`.
- 14 of the 22 M13 tests skip without torch, and the repository has no CI.
- Before the public release, consider Git LFS or pruning the traces of
  variants that were not adopted (about 85 MB of `.npz`).

## Hardware handover firmware (31 October 2026)

The wiring checklist above, built and tested against simulation and host
builds. No hardware was bought, so nothing here is bench-measured.

**Firmware** (`firmware/include/learned.h`, `firmware/src/locomotion/`):
- *Selection switch* (`bipe_selector`). Classical by default; the build flag
  `BIPE_CONTROLLER_DEFAULT` changes the default. A choice is taken only while
  torque is off (OFF to READY) and holds for the whole armed run. The learned
  controller is selectable only when the linked forward pass reports the
  checkpoint `locomotion_config.h` was generated for. In the M11 image the
  choice is a serial command, `c` or `l`.
- *Runtime* (`bipe_learned`): `PolicyController` in C. The command is rate
  limited at the 20 ms policy step and clamped to the trained ranges, the
  observation is built in the manifest layout, and the action law gives leg
  targets and wheel speed targets. The wheel speed loop is split, because
  the 5 ms tick alone is too slow for the trained 1 kHz loop (results
  below): its proportional term runs on the drives (moteus velocity
  commands with kd scale 1; the wheel drivers' generated kd is now the
  policy's kv, 0.05 N·m·s/rad, and the classical controller still sends
  torque with kp and kd scale 0), and its integral runs on the Teensy as
  the 5 ms feedforward (`bipe_m11_wheel_speed_command`). `locomotion_config.h` is generated from the manifest and the
  M10 parameters by `tools/generate_locomotion_config.py` (the posture table
  at full precision from the M10 kinematics, checked against the manifest's
  9-digit table); a test checks it is current.
- *Output guards.* Leg targets are clamped to the M10 joint limits (hip
  −195° to −120°, knee −65° to 2°), wheel speed targets to the hub's
  no-load speed (121 rad/s), and the wheel torque to the hub peak
  (0.495 N·m). Clamps are counted and reported.
- *Non-finite handover.* A non-finite action (NaN or infinity) never reaches
  the targets: the drive step hands over to the classical controller with a
  fresh balance state for the rest of the run, and the learned controller is
  locked out until reset. A non-finite input (or one beyond the float range)
  is a sensing fault: the step fails and the supervisor faults, as for a
  failed classical step.
- *Placement* (`bipe_drive_step`, `bipe_drive_forward`). The forward pass
  can run inside the tick (its action applies at once, as in simulation) or
  after the tick's commands are sent (the action applies one 5 ms tick
  later). The M11 image uses the second, times it with the DWT cycle counter
  and reports the worst case each second (`policy=…us`).
- *Arming.* The M11 loop computes a tick's frames before the supervisor
  ticks, so the frames sent on the arming tick were computed in READY.
  While torque is off the learned output is therefore held at rest
  (nominal legs, still wheels, no feedforward) while the policy still runs
  for timing, and both controllers start afresh on the first tick with
  torque (PR #5 review).
- *Headers without torch.* `python -m bipe.m13.headers` writes the weights
  and parity headers with numpy only (a restricted reader of the
  checkpoint's zip of pickled tensors), so `make teensy-m11` does not need
  the training stack; the export calls the same code.
- *Bench parity check* (`firmware/src/locomotion/parity.c`). The export now
  also writes `generated_locomotion_parity.h`: 16 recorded full-plant
  observations with the float64 actor's actions. Serial `p` in the M11
  image runs them through the forward pass and reports pass or fail against
  the parity.json tolerance (1e-4), checks the vectors' checkpoint against
  the linked weights', and reports the per-call time. On the host the
  largest error is 1.7e-7.
- The M11 image (`teensy_m11.cpp`) uses the drive step in place of the
  classical step and links the forward pass. Its torque path stays locked.

**Tests.**
- `firmware/tests/test_learned.c` (`make -C firmware test`): the switch,
  posture table, command limits, observation layout, action law, leg and
  wheel clamps, non-finite inputs and outputs (NaN, infinity, 1e300), and
  the handover with the forward pass inline and deferred, and the rest
  hold before arming; built against the M10 `generated_config.h`, as the
  image is.
- `tests/test_m13_handover.py`: the generated header is current; the C
  runtime gives the same commands, observations, targets and 1 kHz wheel
  torques as `PolicyController` over 240 ticks of random packets (to 1e-6,
  with the guards applied to the simulation's values); the bench parity
  check passes and fails a forward pass that is off by 2e-4; and a NaN
  action at 1 s in the full plant hands over to the classical controller,
  which holds the robot (the transient is about 4° and has settled 2 s later).
- Builds: the C parts compile for the Cortex-M7 with the Debian
  arm-none-eabi gcc 13.2 (`make arm`, `-Werror`). `teensy_m11.cpp` was
  syntax-checked against stub Arduino headers only: the PlatformIO registry
  is blocked from the cloud, so the Teensy image itself (gcc 11.3) is not
  built here.

**Full plant** (`python -m bipe.m13.handover acceptance --variant …`,
records in `artifacts/m13/handover/`). Ten acceptance cases (four pushes,
the combined sensor case, backlash turn, a belt case, slope up 8°, the 15 ms
delay drive and the 25 ms wheel-lag drive), with the C runtime and the C
forward pass in the loop. R is recovered, F a physical failure; the number
is the peak pitch in degrees.

| Variant | Recovered | Physical failures |
|---|---|---|
| Phase D (reference) | 9/10 | 0 |
| `inline`: forward pass in the tick, wheel loop 1 kHz | 9/10, same verdicts | 0 |
| `deferred`: action one tick later | 8/10 | 0 |
| `wheel_tick`: wheel loop at the 5 ms tick | 8/10 | 0 |
| `deferred_tick`: both, the first wiring | 2/10 | 0 |
| `hardware`: action one tick later, wheel loop split (the M11 image) | 8/10 | 0 |
| `fault_inline`: inline, NaN action from 5 s | 7/10 | 1 (`wheel_lag_25ms_drive`, which the classical controller also fails) |
| `fault_deferred_tick`: deferred_tick, NaN action from 5 s | 7/10 | 2 (the same, and `delay_15ms_noise_x2_drive`) |
| `fault`: hardware, NaN action from 5 s | 7/10 | 2 (`wheel_lag_25ms_drive` and `delay_15ms_noise_x2_drive`) |

- The C runtime reproduces phase D: the same verdicts, peaks within 0.15°.
- Each timing change alone costs one recovery (`wheel_lag_25ms_drive`);
  together the policy loses most rest recoveries (late sway over the band,
  peaks up to 8.6° on the backlash turn), though nothing falls. The policy
  was trained with the action applied in the tick and the wheel loop at
  1 kHz. With the wheel loop's proportional term back at 1 kHz on the
  drives, the deferred placement keeps 8 of 9 recoveries, the same as
  deferring alone. Running the forward pass inside the tick would recover
  the ninth, if the DWT measurement leaves room for it in the tick.
- The handover holds in 9 of 10 cases inline and 8 of 10 with the deferred
  wirings. It is a last resort that can itself fall while driving:
  - `wheel_lag_25ms_drive` falls with every wiring; the classical
    controller also fails that case on its own;
  - `delay_15ms_noise_x2_drive` falls with the deferred wirings, though the
    classical controller passes it from the start (`artifacts/m10/acceptance`,
    peak 10.7°), so the handover itself causes that fall. Seeding the fresh
    balance state's speed and heading references at handover did not help
    (PR #5 review); a ramp from the policy's last leg targets to the
    posture is the next thing to try. Fixed on 1 November 2026 in the
    classical controller, not the handover (see "Fallback fall while
    driving" below).

**Open for the bench.**
- Build the image with the Teensy toolchain; check RAM1 in the linker map
  (the 193 KB of weights are `const`, which Teensy 4 places in RAM1).
- Run `p` (parity) and read `policy=…us` with the DWT counter; add the
  policy phase to `artifacts/m11/firmware/timing.json`.
- In speed mode the total wheel torque is limited by the drive
  (`servo.max_current_A` 3.3 A through its calibrated torque constant),
  not by the Teensy: check the calibrated Kt against 0.15 N·m/A after
  `moteus_tool --calibrate`. Wheel position commands from `moteus_tool` or
  tview now get the configured kd (0.05 N·m·s/rad) by default.
- The wheel speed guard (121 rad/s, the hub's no-load speed) is loose
  against the recorded envelope (60 rad/s), and the leg clamps sit on the
  end stops; tighten both, or treat a far out-of-range action as a bad
  output, if the bench shows a need.
- The label from the commanded posture follows `PolicyController.label`,
  including its neighbour pick, which between table labels can snap to the
  nearer label; fix both together if it matters.

## Belt range follow-up (31 October 2026)

The phase C follow-up: `settle` recovered 4 of the 8 belt range cases
against the classical controller's 8. Records in `artifacts/m13/belt/`.

**Cause.** The four failures are the soft corners (`_k0`: belts at
0.2 × the Gates cord stiffness), whatever the lash. The stiff corners
(`_k1`) all recover. At rest the robot settles into a knee and pitch
oscillation that the policy keeps going: 8 to 11° of pitch peak-to-peak
over the last 4 s and a rest speed peak of 0.22 to 0.26 m/s. The knee,
pitch and speed traces peak at 4.6 Hz (`diagnosis.json`, `spectra`). One
belt at a time (`diagnosis.json`, 20 s hold; script
`tools/m13_belt_diagnosis.py`, job lists in `diagnosis_jobs/`):

| Hip belt (N m/rad) | Knee belt (N m/rad) | Recovered | Late pitch p-p (°) |
|---|---|---|---|
| 118.9 (soft end) | 69.1 (nominal) | yes | 1.0 |
| 350.0 (nominal) | 23.5 (soft end) | no | 6.6 |
| 350.0 | 40 | yes | 1.0 |
| 118.9 | 23.5 | no | 9.0 |

The soft knee belt alone causes it, and the threshold lies between 23.5
and 40 N m/rad. The knee encoder reads the motor side of the knee belt,
so the shin's deflection through the belt is invisible to the servo and
the policy. Training had rigid belts (M13 training plant), so this corner
is outside what the policy saw. M14 phase A (on the M14 branch, not yet
merged) found the same trigger for the classical controller's
`delay_15ms_noise_x2_drive` limit cycle on the belt training plant (knee
chain compliance).

**Runtime changes tried** (no retraining). One change at a time, on the
hardest cases first (`diagnosis.json`, one run per case), then partial or
full suites where noted:

| Change | Soft corners recovered | Stiff corners recovered | Pushes recovered (of 30) | Why it was dropped |
|---|---|---|---|---|
| Leg kp 8 → 12 on the knees | 0 of 1 run (10.5° p-p) | | | stiffer is worse |
| Knee kd 0.05 → 0.15 | 4 of 4 | | | one soft corner at 1.55° p-p, over the 1.5° belt target |
| Knee kd 0.05 → 0.10 | 4 of 4 | 0 of 4 | 29 | step-to-step chatter on the stiff corners: rest speed 0.035 to 0.039 m/s, over 0.03 |
| Knee kd 0.05 → 0.075 | 4 of 4 | 3 of 4 | | the same chatter at the stiff, low-lash corner (0.039 m/s) |
| Unbanded leg target filter, 30 ms, at 1 kHz | 2 of 2 run | 1 of 1 run (15 ms: 0 of 1, 0.038 m/s) | | |
| Unbanded leg target filter, 30 ms, at the 5 ms tick | not run | | 24 | forward pushes of 1.5 N s and more fall: the filter slows the push reaction |
| Banded filter: 30 ms at the 5 ms tick, within 0.06 rad of the target | 4 of 4 | 3 of 4 | 30 | loses speed cases (below) |

- Knee kd 0.10 stiff corners and pushes, and the unbanded 5 ms tick
  filter's pushes, come from partial acceptance runs
  (`partial_knee_kd0.10_acceptance`, 90 of 107 cases, with
  `knee_kd.patch`; `partial_filter_unbanded_acceptance`, 43 cases, from
  commit 9b3e108). The other cells are single diagnosis runs.
- The chatter: with knee kd 0.10 on the stiff corner the speed trace
  peaks at 25 Hz, the 50 Hz policy rate's period two, and the wheel
  actions' step-to-step change is four to five times the trained gain's
  (`spectra`, `k1_kd0.1` against `k1_kd0.075`).

The kept filter is a first-order filter from the policy's leg targets to
the references the servo follows, one step per 5 ms control tick, with
the output kept within a band of the target so a large move (a push
reaction) passes with at most that lag. A 0.03 rad band was too narrow
for the soft corner (9.1° p-p).

**Full plant with the banded filter** (`settle_filter_acceptance`,
`settle_filter_speed`), against `settle` as phase C ran it:

| | `settle` | With the banded filter |
|---|---|---|
| Physical failures, 79 acceptance + 44 speed cases | 0 | 0 |
| Pushes survived / recovered (of 30) | 30 / 30 | 30 / 30 |
| Belt range recovered (of 8) | 4 | 7 |
| Acceptance cases recovered (of 79) | 62 | 64 |
| Speed cases survived and recovered (of 44) | 36 | 30 |

- Gained, of the 79: the four soft belt corners (rest speed 0.024 to
  0.028 m/s) and `backlash_drive_0` (62 + 5 − 3 = 64). On the speed suite:
  `drive_step_half_timestep`. Outside the 79, among the acceptance jobs:
  `mass_plus_15pct_drive` (0.032 → 0.030 m/s).
- Lost, of the 79: `belt_hold_h0.45_n0.9_k1` (0.035 m/s), `slope_up_0`
  (0.032) and `mismatch_body_mass_scale_0.85_hold` (0.046). On the speed
  suite: four weaves (0.030 to 0.042), `wheel_lag_25ms_drive_step`
  (0.030), and `wheel_lag_25ms_weave` and `wheel_lag_25ms_replay_corner_90`
  with a real rest sway (0.17 and 0.19 m/s). Outside the 79:
  `wheel_lag_25ms_drive` (0.023 → 0.172 m/s).
- The 0.03 m/s verdicts are chaotic at the margin: an independent rerun
  of main without the filter, in another environment, fails
  `belt_hold_h0.45_n0.9_k1` at 0.037 m/s too (`reviews/pr7-review.md` in
  the project files), so that loss is within noise. The 25 ms wheel-lag
  sways are not.

Every change moves the shortfall rather than removing it: damping the
soft knee mode in the runtime changes the closed loop the policy was
trained on everywhere else. The rest-speed limit (0.03 m/s) decides most
of the lost cases, where `settle` already sat at 0.020 to 0.029.

**Hardware timing.** The firmware runtime (`python -m bipe.m13.handover`,
C runtime and C forward pass) on the belt cases
(`artifacts/m13/belt/*_acceptance`):

| Variant | Filter | Soft corners recovered | Stiff corners recovered | Late pitch p-p (°) | Rest speed peak (m/s) |
|---|---|---|---|---|---|
| `inline` (as trained and as phase C ran) | off | 0 of 4 (2 here, 2 in the review) | | 7.5 to 11.3 | 0.21 to 0.27 |
| `deferred` (action one 5 ms tick later) | off | 2 of 2 run | | 0.7 to 1.0 | 0.020 to 0.022 |
| `hardware` (the M11 image) | off | 4 of 4 | 4 of 4 | 0.74 to 1.40 | 0.016 to 0.027 |
| `hardware` | on | 4 of 4 | 4 of 4 | 0.94 to 1.43 | 0.019 to 0.028 |

- `inline` reproduces phase C's soft-corner failures. Applying the action
  one tick later, as the M11 image does, removes the limit cycle on its
  own: the extra 5 ms shifts the phase of the policy's loop around the
  4.6 Hz knee mode enough to stop sustaining it.
- So on the robot's wiring the limit cycle is gone without the filter:
  late pitch p-p at most 1.4° in all 8 cases, against 7.5 to 11.3°
  inline. The rest-speed verdict sits at the limit, 7 to 8 of 8: the
  review reran `hardware` with a 1e-5 rad perturbation (firmware time
  constant 1e-6 s) and `belt_hold_h0.45_n0.225_k0` failed at 0.031 m/s,
  pitch p-p 1.08°.
- This cuts the other way from the handover findings above, where the
  deferred placement costs `wheel_lag_25ms_drive`: moving the forward
  pass into the tick to recover that case (if the DWT budget allows) would
  bring back the soft knee belt limit cycle. Both effects are timing
  margins of a policy trained with neither the delay nor the compliance;
  M14 should train with the image's timing and the belts.

**Decision.** The filter stays in the simulation runtime
(`policy_control.LEG_FILTER_TAU_S`, `full_plant --leg-filter`) and the
firmware (`bipe_learned_legs`, `BIPE_LEARNED_LEG_FILTER_TAU_S` and
`_BAND_RAD` from the manifest's `action.legs.target_filter`), off by
default (time constant 0: the targets apply directly, as in training and
in every earlier record; user decision, 31 October 2026). The belt range
shortfall goes to M14, whose training plant has the compliant hip and
knee belts (M14 phase A, on the M14 branch). On the M11 image's timing
the limit cycle does not appear. For the bench: keep the forward pass
deferred, as the M11 image places it. The filter is not a fallback for
moving it inline: its full-suite runs are inline, and they lose
`wheel_lag_25ms_drive` and all three `wheel_lag_25ms` speed cases, the
case moving inline would be meant to win. If the knee belt measures soft
and inline timing is still wanted, that is an open question for M14,
not for this filter.

## Fallback fall while driving (1 November 2026)

PR #5 left one handover fall open: in `delay_15ms_noise_x2_drive` (sensor
transport delay 15 ms, three times nominal, IMU noise ×2, driving at
0.3 m/s from 2 to 7 s) a NaN action at 5 s hands over to the classical
controller, which then falls at 5.79 s, though it passes the case on its
own.

**Cause.** The handover is not what fails. With the 15 ms delay the
classical controller has a 7 to 10 Hz pitch, knee and wheel oscillation
with a very small basin, and anything that moves the knee sets it off.
The hub's stator sits in the shin, and the 1:1 knee belt holds the shin
from the knee drive, so every wheel torque reacts on the knee drive.
Runs (classical controller unless named):

| Run | Before |
|---|---|
| Classical alone | recovers, peak 10.7° |
| Classical alone, push 0.3, 0.6 or −0.6 N·s at 4 s, mid-drive | recovers |
| Classical alone, push 1.0 N·s at 4 s | falls at 5.98 s |
| Handover at 0 s (no policy step) | identical to classical alone |
| Handover after 25 ms of policy, before the drive (standing) | falls at 1.76 s |
| The same with a policy that outputs zero actions | recovers |
| The same with the sensor delay at 5 ms | recovers |
| The same with no belt lash | falls |

Changes at the handover alone do not help. These all still fall or only
survive: seeding the speed and heading references (the PR #5 review's
try), a 0.3 or 1.0 s leg ramp from the policy's targets to the posture,
a bumpless switch (the speed integral set so that the pitch reference
starts at the measured pitch), stopping on the fault (survives at 19.3°),
leg feedforward variants, and the pitch-rate, speed or leg gains changed
after the handover. Higher leg damping or stiffness gets through the
1.0 N·s push but leaves the robot running away in a 9 Hz limit cycle
after the handover. The diagnosis table, the scripts behind it and the
lag probe below are in `artifacts/m13/handover/fallback_fix/`
(`diagnosis.md`, `tools/`).

**Fix.** The classical knee feedforward now cancels the wheel torque's
reaction: `ff_knee -= knee_wheel_reaction × wheel torque` on each side,
with `knee_wheel_reaction` 1.0 in `config/controller_m10.json`. The knee
drive's axis is −y against the wheel's +y, hence the sign; the opposite
sign makes the fall come sooner. The change is in `bipe_control_step`
(serial profile only, so the M9 and earlier profiles are unchanged), in
`LocomotionController` and in the Python sensor path of
`moving_experiment.simulate`; C and Python agree to 8e-16
(`equivalence`). The M11 image sends this feedforward in the joint frames
as before. The learned controller is unchanged: its leg targets carry no
feedforward, as trained. `test_learned` checks the feedforward against
`bipe_feedforward` and the wheel torques.

**Results.**

| Run | Before | After |
|---|---|---|
| Classical alone, push 1.0 N·s at 4 s | falls | recovers, peak 11.9° |
| Classical alone, push 1.5 N·s at 4 s | — | survives, peak 18.5°, not recovered |
| Handover after 25 ms (standing) | falls | recovers, peak 10.6° |
| `fault` (hardware wiring), NaN from 5 s | falls at 5.79 s | survives, peak 11.3°; rest speed 0.030 m/s against the 0.03 limit, so not counted as recovered |
| `fault_inline` | survives, peak 19.6° | survives, peak 12.1° |
| `fault_deferred_tick` | falls | recovers, peak 10.7° |

- Handover suites (`artifacts/m13/handover/fallback_fix/`, the same ten
  cases as above): the only physical failure left under each fault
  variant is `wheel_lag_25ms_drive`, which the classical controller also
  fails on its own (`fault` 2 → 1, `fault_inline` 1 → 1,
  `fault_deferred_tick` 2 → 1). The other nine keep their verdicts.
- M10 acceptance (`artifacts/m10/acceptance`, 107 cases, rerun): every
  target is unchanged. Physical failures go from 4 to 3:
  `delay_15ms_noise_x2_combined`, which fell, now passes (peak 7.97°), and
  the rest are `unpowered` and the two `wheel_lag_25ms` cases, as before.
  Two recovery verdicts at the margin swap: `push_low_+0.5` now recovers
  and `push_low_-2.5` no longer does (peak 17.7° against 17.6°, late
  pitch p-p 1.19° against 1.13°), so the contiguous low-push recovery
  margin is 2 N·s in both directions (was 2.5 and 0).
- M12 speed suite (`artifacts/m12/speed`, 44 cases, rerun): the same five
  failures as before, all recorded shortfalls (`slope_up_4`, `slope_up_8`
  and the three `wheel_lag_25ms` cases); convergence passes; among the
  cases that stay up no peak pitch moves by more than 1°.
- M12 following closed loop (`artifacts/m12/following`, 214 runs, rerun):
  see the M12 numbers in the README.

**The 25 ms hub-lag cases now fall sooner, and forward.** These cases
lag the hub torque by 25 ms (the model's hub lag is 0.8 ms). They failed
before the fix and still fail, but the feedforward now acts on the
commanded wheel torque 25 ms before the hub delivers it, so the knee is
pushed the wrong way and the robot falls earlier and pitches forward
instead of backward:

| Case | Main | With the fix |
|---|---|---|
| M10 `wheel_lag_25ms_drive` | falls 7.21 s, backward | falls 3.43 s, forward |
| M10 `wheel_lag_25ms_combined` | falls 4.02 s | falls 3.79 s |
| M12 speed `wheel_lag_25ms` `drive_step` | falls 9.15 s | falls 2.43 s |
| M12 speed `wheel_lag_25ms` `weave` | falls 10.13 s | falls 2.22 s |
| M12 speed `wheel_lag_25ms` `corner_90` | falls 11.83 s | falls 9.07 s |
| Handover `fault`, `wheel_lag_25ms_drive` | falls 7.10 s | falls 5.37 s |

A probe that passes the feedforward through a first-order lag matched to
the hub (tau 25 ms) brings `wheel_lag_25ms_drive` back to the main
behaviour (falls 7.17 s, backward) while the 1.0 N·s push in the delay
case still recovers (peak 12.2°); tau 10 ms falls at 3.78 s
(`diagnosis.md`). With the modelled 0.8 ms hub lag a filter has no
effect, so it is not in the firmware: the hub lag is a bench measurement.

- Not rerun, so made with the classical controller from before this fix:
  the M12 rendered-perception records (`artifacts/m12/following_rendered*`),
  M13 phase C `acceptance_classical` and `speed_classical`, and the
  M13/M14 classical baselines (gap, stress).

**For the bench.** The knee drive now also carries the wheel torque
(up to 0.495 N·m on top of the gravity feedforward, inside the 1.65 N·m
joint peak). Check the sign on the first tethered drive: the knee
current should rise with the hub torque, and no 7 to 10 Hz knee and
pitch buzz should appear. `knee_wheel_reaction` is a compile-time
constant, so build two images, k = 0 and k = 1, and compare them on the
same tethered drive. Measure the hub's torque lag (step the hub current
and log the knee current); if it is well above 1 ms, add the matched lag
filter from the probe before trusting k = 1 on the robot.
