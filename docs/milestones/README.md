# Milestone log

The status of each milestone as it closed, with links to its plan, records and
code. This was the repository's front page until the public README replaced
it (November 2026); the text is unchanged apart from link paths. Each
milestone's full plan, decisions and results are in its own document here.

| Milestone | Document |
|---|---|
| M10 mechanical design, plant and classical control | [10-redesign-plan.md](10-redesign-plan.md) |
| M11 electronics and firmware | [11-electronics-plan.md](11-electronics-plan.md) |
| M12 camera following | [12-camera-following-plan.md](12-camera-following-plan.md) |
| M13 learned locomotion | [13-learned-locomotion-plan.md](13-learned-locomotion-plan.md) |
| M14 release-grade locomotion | [14-release-grade-locomotion-plan.md](14-release-grade-locomotion-plan.md) |
| M15 behaviours: get-up, moves, following with the policy | [15-behaviours-plan.md](15-behaviours-plan.md) |
| M16 a complete behaviour set in simulation | [16-behaviour-set-plan.md](16-behaviour-set-plan.md) |
| M0–M9 | [../history/](../history/README.md) |

## M10 (18 October 2026)

A compact wheeled biped, designed digitally so that it can later be
manufactured. It balances, drives, turns and crouches. Camera following
and jumping are later work.

M10 is the current design: a compact serial leg. Each hip housing
holds two coaxial RO40 drives, the knee is belt-driven inside the
thigh, and each outboard wheel has a GL40 hub motor. There is a styled
binocular head pod. The full plan, decisions and results are in
[docs/milestones/10-redesign-plan.md](10-redesign-plan.md).

**Status (18 October 2026).** Phases A to E are complete. Phase E
pruned the repository to what M10 uses.

The results below are a layout study plus simulation under the stated
assumptions. Nothing is hardware-validated.

| Phase | Result |
|---|---|
| A. Concept and sizing | Serial-leg layout and look signed off by the user. |
| B. Detailed CAD | Shells, legs, hub wheels, head and internals from manufacturer data. The sweep is clean. |
| C. Plant and control | Free-base MuJoCo plant with a knee belt and hub motors; Python and C posture kinematics. Acceptance: robustness 70/70; inherited cases 26/27 (`delay_15ms_noise_x2_combined` accepted at C6), 24/27 since the `wheel_lag_25ms` variant was made to reach the hub drives in M12 phase B (both lag cases also fall with the M10 controller). 25/27 since the knee feedforward cancels the wheel torque's reaction on the knee drive (1 November 2026): `delay_15ms_noise_x2_combined` passes again. |
| D. Falls | Forward and backward tips reach factor ≥2 on every structural path (minimum 2.06). Sideways tips are contained, and their structural shortfalls (hip 6704 bearings 0.48 and others) are a recorded risk the user signed off. CAD mass 1.815 kg with the M11 parts (was 1.800 kg); 1.975 kg allocated against a 2.15 kg ceiling. |

Key outputs:
- CAD and checks:
  - [assembly CAD](../../artifacts/m10/assembly/d4/assembly.FCStd);
  - [renders](../../artifacts/m10/assembly/renders/);
  - [mass](../../artifacts/m10/cad/mass.json);
  - [sweep](../../artifacts/m10/cad/sweep.json).
- Simulation and tuning:
  - [falls summary](../../artifacts/m10/falls/summary.json);
  - [acceptance](../../artifacts/m10/acceptance/);
  - [tuning](../../artifacts/m10/tuning/).
- Configuration:
  - [shared SI configuration](../../config/robot.json) (`m10` section);
  - [controller](../../config/controller_m10.json);
  - [sensors](../../config/sensors-m10.json).

**M11 electronics (19 October 2026).** The electrical design is
complete as a digital design. Nothing is bought, built, flashed or
powered. The plan, decisions and results are in
[docs/milestones/11-electronics-plan.md](11-electronics-plan.md).

| Phase | Result |
|---|---|
| A. Requirements | Loads from 17 acceptance cases: worst driver 3.41 A peak; bus 6.63 A peak, 127 W, never regenerates; runtime 28–39 min. |
| B. Parts | Six moteus-c1 on CAN-FD; a Teensy 4.0 on a custom carrier with an MCP2562FD transceiver and an LSM6DSO IMU; a Pi Zero 2 W slot reserved. |
| C. Power | LM5060 high-side switch, 10 A fuse and a 5 V module; the regeneration and fault checks pass. |
| D. Packaging | Carrier and power boards routed in KiCad (ERC/DRC pass) and placed in the CAD; the sweep is clean; CAD mass 1.815 kg, 1.975 kg allocated. The falls and acceptance were rerun with the M11 plant. |
| E. Firmware | moteus codec and joint law equal to mjbots' `moteus` package; IMU axes from the routed board; latency 3.44 ms of the 5 ms budget; the Teensy image builds with the torque path locked. |
| F. Exit | Acceptance unchanged (checked); the open items for hardware are listed in the plan. |

M11 outputs:
- [requirements](../../artifacts/m11/requirements/), [power](../../artifacts/m11/power/),
  [boards](../../electronics/m11/) and [packaging](../../artifacts/m11/packaging/);
- [driver configurations](../../artifacts/m11/firmware/moteus/) and the
  [timing budget](../../artifacts/m11/firmware/timing.json);
- firmware: [`firmware/src/m11/`](../../firmware/src/m11/) and
  [`firmware/src/teensy_m11.cpp`](../../firmware/src/teensy_m11.cpp).

**M12 camera following (started 19 October 2026).** An AprilTag worn or
carried by the person; following to about 1.0 m/s with a fixed head; the
follower in the Teensy's supervised C core.
- **Phase A** (requirements and scenarios) is signed off: the
  requirements in [config/following-m12.json](../../config/following-m12.json);
  twelve scripted scenarios, the phase B targets and rendered
  head-camera frames in
  [artifacts/m12/requirements/](../../artifacts/m12/requirements/)
  (`.venv/bin/python -m bipe.m12.requirements`).
- **Phase B** (speed envelope: 1.2 m/s, 1.25 m/s², 1.0 rad/s,
  1.25 rad/s², 0.3 m/s reverse) is signed off (21 October 2026). A
  jerk-limited drive reference with a lean feedforward, a stop that
  brakes to rest, and an estimator compensated for drive acceleration.
  44 speed cases on the C core
  ([config/speed-m12.json](../../config/speed-m12.json),
  [artifacts/m12/speed/](../../artifacts/m12/speed/),
  `.venv/bin/python -m bipe.m12.speed`) pass except recorded shortfalls.
  The M10 acceptance has no new failures. Falls at speed are in
  [artifacts/m12/falls_speed_sweep/](../../artifacts/m12/falls_speed_sweep/).
  This is a validated simulation under stated assumptions, not
  hardware.
- **Phase C** (perception) is signed off (21 October 2026). It covers:
  - the Wide-lens camera model;
  - tag36h11 detection with pupil-apriltags on rendered frames;
  - a Pi Zero 2 W timing estimate;
  - the measurement model in `config/perception-m12.json`.

  Results:
  - Up to a 45° view angle, the waist tag is detected in every frame
    from 0.5 to 2 m. Range reads 1–2 % long and bearing is within 0.11°.
  - In the replays, region-of-interest tracking detects 99.9 % of
    expected frames.
  - The Pi Zero 2 W figures are an estimate scaled from this machine,
    not a measurement: region-of-interest tracking at 18–24 Hz per
    core, and 0.12–0.15 s of latency against the 0.1 s assumed in
    phase A.

  Outputs are in
  [artifacts/m12/perception/](../../artifacts/m12/perception/); rebuild them
  with `.venv/bin/python -m bipe.m12.perception`. These are rendered
  frames under stated assumptions, not real footage. Decisions:
  - pupil-apriltags;
  - 1152 px with region-of-interest tracking over 2 cores, with phase D
    designing against 0.12–0.15 s of latency;
  - keep the Zero 2 W and benchmark it on a real board before phase E;
  - keep the 42° camera pitch, which needs a head change in phase E.
- **Phase D** (following controller) is signed off (22 October 2026).
  It covers:
  - a latency-compensated target estimator, the follow modes and the
    follower law in the supervised C core (`firmware/src/follower.c`,
    [config/follower-m12.json](../../config/follower-m12.json));
  - a Python reference in exact parity;
  - a Pi link model.

  The sign-off changes the band floor to 0.6 m, and the loss coast to
  0.45 s with a 0.95 m stopping distance.

  Results:
  - Following starts only on an explicit request.
  - With following off, the phase B speed cases and all 107 M10
    acceptance cases are bit-identical to their records.
  - In closed loop on the free-base plant, 114/120 nominal runs meet
    every requirement check, with no falls and peak pitch at most 13.7°.
  - Accepted shortfalls:
    - losses in the 90° corner and the 1 m curve can still reach hold;
    - the `delay_15ms_noise_x2` sensor variant fell, recorded as a
      balance-core limit; since the knee wheel-reaction feedforward
      (1 November 2026) it no longer falls and the variants pass 19/20;
    - slope runs reach 15.2–15.3° peak pitch.

  Records are in
  [artifacts/m12/following/](../../artifacts/m12/following/)
  (`.venv/bin/python -m bipe.m12.following`). This is a validated
  simulation on the phase C measurement model, not rendered perception
  or hardware.
- **Phase F** (closed loop with rendered perception in the loop) is
  signed off (24 October 2026). Each 30 Hz frame is rendered from the
  plant over a 5 ms exposure and read by the phase C tracker, with the
  phase C Pi Zero 2 W latency. The Pi drops near edge-on tag poses
  (fitted view angle past 74°), whose range reads short.

  Results:
  - The tag is detected in 99.6–100 % of the frames where it can be
    seen.
  - 60/60 nominal runs and 101/112 overall pass, with no falls in the
    nominal plant.
  - The following-off regression is bit-identical.
  - Accepted shortfalls:
    - the `delay_15ms_noise_x2` falls (records from before the knee
      wheel-reaction feedforward, not rerun);
    - the slope pitch;
    - two losses under a strong side light (left to phase E);
    - two runs at 0.20 s latency.

  Records are in
  [artifacts/m12/following_rendered_5ms/](../../artifacts/m12/following_rendered_5ms/)
  (`.venv/bin/python -m bipe.m12.following_rendered`). This is
  rendered perception under stated assumptions, not real footage or
  hardware. Phase E (hardware) waits for the purchases.
Rendering uses MuJoCo on EGL (`MUJOCO_GL=egl`, set by `bipe.m12`). The
plan and results are in
[docs/milestones/12-camera-following-plan.md](12-camera-following-plan.md).

**M13 learned locomotion (30 October 2026).** An RL policy for the M10
scope: balance, drive, turn and crouch. It was trained with mjlab on
MuJoCo Warp, on a primitive-geometry training plant fitted to the full
plant. The policy reads the robot's own sensor channels. Every 20 ms it
commands leg joint targets and wheel speed targets, through the 1 kHz
firmware joint law. The plan and results are in
[docs/milestones/13-learned-locomotion-plan.md](13-learned-locomotion-plan.md).

| Phase | Result |
|---|---|
| A. Training plant | Fitted to full-plant traces. Agrees with the full plant on pass/fail in 132/135 cases with the classical controller. Warp parity 5.8e-5. |
| B. Environment | mjlab task, observations from the sensor channels the robot has, domain randomisation from the M10 variant families ([`bipe/m13/rl/`](../../bipe/m13/rl/)). |
| C. Full plant | The `settle` policy, against the classical controller on the same jobs. The policy runs 98 of the 107 M10 acceptance cases (it rejects the 8 ideal-sensing cases, and `unpowered` has no controller). Physical failures over those 98 and the 44 speed cases: 0 (classical 6). Recovered: 80/98 (83/98); pushes 30/30 (28/30); belt range 4/8 (8/8). Rest recovery is sensitive enough that one run per case cannot separate the two (phase D). |
| D. Export | [ONNX and manifest](../../artifacts/m13/export/settle/), and a float32 C forward pass ([`firmware/src/locomotion/`](../../firmware/src/locomotion/)). Torch, ONNX, host C and the Cortex-M7 build agree within 1.1e-6. Closed loop with the C forward pass: 99 cases, no falls. Teensy 4.0 estimate 0.27–0.71 ms per step (a model, not measured). The worst case is 1.41× the 0.5 ms compute bound M11 assumed, so the forward pass runs outside the 5 ms tick path, and the bench measures it. |
| E. Exit | Both controllers selectable, with the classical one the default until the bench check (user decision, 30 October 2026). |

- In simulation, the M10 and M12 runners use the classical controller.
  `python -m bipe.m13.full_plant acceptance|speed --checkpoint
  artifacts/m13/training/settle/model_10493.pt [--forward c]` runs the
  same cases with the policy.
- The M11 Teensy image links the C forward pass
  (`firmware/src/locomotion/locomotion.c`) and its runtime
  ([`learned.c`](../../firmware/src/locomotion/learned.c), 31 October 2026), with
  the torque path still locked. The image itself is not yet built with the
  Teensy toolchain (`.venv/bin/python -m bipe.m13.headers` writes its
  generated headers):
  - a selection switch, classical by default, chosen over serial while
    torque is off;
  - leg target, wheel speed and wheel torque clamps; a non-finite action
    hands over to the classical controller, a non-finite input faults the
    supervisor;
  - the forward pass after the tick's commands, timed with the DWT counter,
    and an on-board parity check (serial `p`).

  `python -m bipe.m13.handover` runs this firmware runtime in the full
  plant; the milestone doc has the results and the bench steps left.
- This is a validated simulation under stated assumptions, not
  hardware.
- Follow-ups for M14 (done, below): belt-range recovery, a policy form of
  the speed suite's joint-error rule, a reproducible from-scratch training
  recipe.

**M14 release-grade locomotion (3 November 2026).** A from-scratch training
recipe (`python -m bipe train --task locomotion --seed N`, 5.9 h per seed on an
8 GB laptop GPU, reproduced on two seeds) on a training plant with belt lash
and stiffness, actuator, pack-voltage and sensor randomisation. The deployed
policy adds a 2000-iteration fine-tune of seed 1 with larger pushes and
doubled turn tracking (about 7.4 h in all, run on one seed so far). The resulting policy replaces
settle in the firmware's generated headers (classical stays the power-up
default until the bench check). Against M14's gates, with settle in brackets:
- recovered 89/98 (74/98; 88/98 with the C forward pass), belt range 8/8
  (2/8), pushes 30/30, 0 physical failures over the acceptance and speed
  suites;
- speed suite 35/43 under the policy rule (31/43); the gate is 40, missed. It
  no longer holds on the 8 and 10 degree downhill cases, which settle did;
- stress at 1.5x delays and lags with 2.5 N s pushes: training plant 5.9 %
  falls (10.3 %, one unseeded run), a near miss on the 5 % gate from steep
  slopes and from pushes on gentler ground; full plant 25/26 draws, surviving
  only (10 recover);
- export: [ONNX and manifest](../../artifacts/m14/export/locomotion/), parity with
  the C forward pass and its Cortex-M7 build within 6.3e-7, closed loop 98/99
  same verdicts and no falls.

The plan, feasibility study and results are in
[docs/milestones/14-release-grade-locomotion-plan.md](14-release-grade-locomotion-plan.md).

**Hardware build plan (21 October 2026, not started).** Bill of
materials with prices and lead times, a staged build (bench timing,
boards, single-leg rig, tethered robot) and the single-leg rig design:
[docs/hardware/build-plan.md](../hardware/build-plan.md) and
[docs/hardware/bom.csv](../hardware/bom.csv). Nothing is bought.

Milestones 0–9 (M4 articulated prototype and electronics, M5 digital
release, M6/M7 learning studies, M8 robustness yardstick, M9 five-bar
redesign) are documented in [docs/history/](../history/README.md).
Their code and electronics are in the git tags
`archive/m10-pre-cleanup` and `archive/m9-final`; their generated
outputs are in the full-history bundle kept outside the repository (see
[docs/history/README.md](../history/README.md)).
