# Results

What the BiPe simulations show, milestone by milestone, and what is still
open. Every number here comes from a committed record under `artifacts/`;
the milestone documents in [milestones/](milestones/README.md) hold the full
tables, the decisions and the methods.

**Nothing here is hardware-validated.** Each result is one of:
- a **layout study**: geometry, masses and clearances from CAD built on
  manufacturer data;
- a **validated simulation under stated assumptions**: a physics model with
  its assumptions written down, judged by fixed pass rules;
- an **estimate**: a model of something to measure later (CPU timing, for
  example).

## The plant that judges everything

The full plant is a free-base MuJoCo model of the CAD assembly at a 0.25 ms
step. It has finite motor torque and speed limits, physical wheel and shell
contact with friction from the shell and tyre materials, belt stiffness and lash in the hips
and knees, bearing friction, a sensor model (IMU and encoder delay, noise,
quantisation and bias) and the robot's own C firmware in the loop at 1 kHz.
Nothing is stabilised by hidden constraints or state clamps.

The learned policy is trained on a faster, primitive-geometry plant fitted to
the full plant. A result measured only on that training plant is labelled a
training result; the full plant is the judge.

## M10: mechanics, plant and classical control

A serial leg: two coaxial RO40 drives in each hip housing, a belt-driven knee
inside the thigh, and a GL40 hub motor in each outboard wheel.
Plan: [10-redesign-plan.md](milestones/10-redesign-plan.md).

| Check | Result |
|---|---|
| Mass (layout study) | 1.815 kg from CAD; 1.975 kg allocated against a 2.15 kg ceiling |
| Robustness matrix, classical | 70/70 cases (pushes, backlash, belt range, parameter mismatch, slopes, numerical) |
| Scenario cases, classical | 25/27 |
| Forward and backward falls | Structural factor at least 2 on every load path (minimum 2.06) |
| Sideways falls | Contained; some parts fall short of factor 2 (hip 6704 bearings 0.48), a recorded and accepted risk |

## M11: electronics and firmware

Six moteus-c1 drivers on CAN-FD, a Teensy 4.0 on a custom carrier with an
IMU, a power board with a high-side switch and fuse, and a Raspberry Pi Zero
2 W slot for vision. Boards are routed in KiCad (ERC and DRC pass) and placed
in the CAD. Plan: [11-electronics-plan.md](milestones/11-electronics-plan.md).

| Check | Result |
|---|---|
| Loads (from 17 acceptance cases) | Worst driver 3.41 A peak; bus 6.63 A peak, 127 W; never regenerates |
| Runtime | 28–39 min per pack |
| Control loop latency | 3.44 ms of the 5 ms budget |
| Driver protocol | Codec and joint law equal to mjbots' `moteus` package |
| Firmware image | Builds for the Teensy with the torque path locked until hardware qualification |

## M12: camera following

A person wears an AprilTag at the waist; the robot follows to about 1.0 m/s
with a fixed head. Plan:
[12-camera-following-plan.md](milestones/12-camera-following-plan.md).

| Check | Result |
|---|---|
| Speed envelope, classical | 1.2 m/s, 1.25 m/s², 1.0 rad/s, 1.25 rad/s², 0.3 m/s reverse; the 44 speed cases pass apart from recorded shortfalls |
| Tag detection on rendered frames | Every frame from 0.5 to 2 m up to a 45° view angle; range 1–2 % long, bearing within 0.11° |
| Vision timing on the Pi Zero 2 W | 18–24 Hz per core and 0.12–0.15 s latency (an estimate scaled from a desktop, not measured) |
| Following, measurement model | 114/120 nominal runs meet every check; no falls |
| Following, rendered camera in the loop | 60/60 nominal runs and 101/112 overall; no falls on the nominal plant; tag found in 99.6–100 % of the frames where it is visible |

## M13 and M14: learned locomotion

A policy trained with [mjlab](https://github.com/mujocolab/mjlab) on MuJoCo
Warp with rsl_rl PPO. Every 20 ms it reads the robot's own sensor channels
(gyro, accelerometer, leg encoders and speeds, wheel speeds, its last action and the command) and outputs leg joint targets and
wheel speed targets, which go through the firmware's 1 kHz joint law. It sees
no velocity estimate; it infers it from the IMU and wheel speeds. The
domain randomisation covers mass, friction, lags, delays, noise, belt lash and
stiffness, actuator torque constants and friction, and pack voltage. Plans:
[13-learned-locomotion-plan.md](milestones/13-learned-locomotion-plan.md),
[14-release-grade-locomotion-plan.md](milestones/14-release-grade-locomotion-plan.md).

The deployed policy (M14 "fine-tune 1") is the from-scratch recipe on seed 1
(8000 iterations, about 5.9 h on an 8 GB laptop GPU) plus a 2000-iteration
fine-tune with larger pushes and stronger turn tracking. Full-plant results
against the M14 gates, with the classical controller and the M13 policy
for comparison:

| Gate | Pass mark | M14 policy | Classical | M13 policy |
|---|---|---|---|---|
| Acceptance cases recovered (98 the policy can run) | at least 83 | **89** | 83 | 74 |
| Belt-range cases recovered | at least 6/8 | **8/8** | 8/8 | 2/8 |
| Pushes recovered | 30/30 | **30/30** | 28/30 | 30/30 |
| Physical failures, acceptance and speed suites | 0 | **0** | 6 | 0 |
| Speed cases under the policy rule | at least 40/43 | 35/43 (missed) | | 31/43 |
| Training-plant stress falls (1.5x delays and lags, 2.5 N·s pushes) | at most 5 % | 5.9 % (missed) | | 10.3 % |
| Full-plant stress cases survived | 26/26 | 25/26 (missed) | | 12/24 (earlier 24-case set) |

Notes:
- The 98 cases are the 107 acceptance cases less the 8 ideal-sensing cases
  (the policy reads real sensor channels only) and `unpowered`.
- The speed rule for the policy drops the classical "joint within 5° of the
  commanded posture" check, which a policy that places its own legs cannot
  meet, and adds an axle-height tracking check. The policy no longer holds on
  the 8° and 10° downhill cases, which the M13 policy did.
- The recipe was run from scratch on two seeds. The fine-tune has so far
  been run on one seed only.

Export ([artifacts/m14/export/locomotion/](../artifacts/m14/export/locomotion/)):

| Check | Result |
|---|---|
| Graph | ONNX with a manifest naming every input and output channel and unit; 48,248 parameters, an MLP of 256, 128 and 64 units |
| Parity | PyTorch, ONNX, host C and the Cortex-M7 build agree within 6.3e-7 |
| Closed loop with the C forward pass | 98/99 cases with the same verdict as the reference, no falls |
| Teensy 4.0 compute | 0.27–0.71 ms per step (a model, measured on the bench later) |

The firmware links both controllers. The classical one is the power-up
default; the policy is selected over serial while torque is off, with clamps
on every output and a handover to the classical controller on a non-finite
action. The policy becomes the default only after it is checked on the bench.

## M15: behaviours on the learned policy

Getting up after a fall, three expressive moves, and camera following with the
policy in place of the classical controller. Plan and results:
[15-behaviours-plan.md](milestones/15-behaviours-plan.md).

**Get-up.** A separate get-up policy (`getup_s1`, 3000 iterations from the
locomotion policy, trained on a library of fallen states) runs from a fallen
state and hands over to the locomotion policy once both tyres are down, no
shell touches the floor and pitch and roll are within 10° of standing. A trial
passes if the handover comes within 5 s and the robot then balances for 5 s.
On 60 randomised full-plant falls, each with a plant drawn from the robustness
variants:

| Fallen state | Got up |
|---|---|
| Sit | 20/20 |
| Front lean | 16/20 |
| Back lean | 16/20 |
| All | 52/60 (pass mark 80 % per class) |

A quick leg push brings 40° to 70° leans over the wheels in 0.2 to 0.4 s.
Seven starts (flat on the face or back) never hand over. M15 put this down to
the C estimator refusing those poses; M16 found the policy cannot lift them
at all (see below).
The 60 starts were also used to choose this policy, and the M10 falls screen
for the get-up motion has not been run. The get-up policy is exported like
the locomotion one ([artifacts/m15/export/getup/](../artifacts/m15/export/getup/)):
parity within 7.0e-7, 0.28–0.72 ms per call on the Teensy model.

**Roll-overs.** A sideways tip that rolls the robot over cannot be undone with
these legs (the [get-up study](studies/getup-feasibility.md)). On 64 sideways
tips, the configured protective fall (crouch, then torque off) ends 47 as sits,
17 on the side and none rolled over; cutting torque at once rolls 9 over, so it
was not adopted. A robot resting on its side is not covered by the get-up, and
the Teensy application does not call the protective fall yet.

**Moves.** A crouch bob, a spin with a stop and a side-to-side wiggle, as
command sequences for the locomotion policy (no extra training): 3/3 nominal
and 21/21 plant variants, no falls.

**Following with the policy.** On the rendered camera link with the 74° view
gate, 60/60 nominal runs (classical 60/60), with the follower's acceleration
limit at 1.1 m/s² for the policy (1.25 m/s² for the classical controller). At
1.25 m/s² the policy brakes hard enough to trip the 15° pitch check (34/60).
The 1.1 limit was chosen on these same 60 runs, and the 52 robustness runs of
the following matrix have not been run with the policy.

**Speed and stress.** A fine-tune aimed at the failing speed and stress cases
(fine-tune 3) did worse (speed 31/43, acceptance 75/98) and was not adopted.
The two-stage recipe on seed 2 reaches 83/98 with 7/8 belt cases and 29/30
pushes, one margin verdict short of gate 2. The deployed locomotion policy is
still M14's fine-tune 1.

## M16: a complete behaviour set

One observation and action contract for every policy
([policy-contract.md](policy-contract.md)), a catalogue of behaviours with
intents to command them ([behaviours.md](behaviours.md)), a sit-and-stand
policy, held-out evaluation of the get-up, and `bipe sim`. Plan and results:
[16-behaviour-set-plan.md](milestones/16-behaviour-set-plan.md).

| Check | Result |
|---|---|
| Reproducible recipe | Seed 2 of the two-stage recipe: 85/98 acceptance, 30/30 pushes, 7/8 belt, no falls, margin verdicts rerun three times |
| Get-up, held out | M16: 53/60 on 60 new randomised starts with a quiet handover (sit 20/20, front 15/20, back 18/20); every miss started almost level on the face or back (pitch 87 to 90°). M17: 59/60 (sit 20/20, front 20/20, back 19/20) with a 0.5 s leg pre-pose on flat starts and a quiet fallen-mode takeover; 40/40 on a fresh set of 40 lean starts ([M17](milestones/17-flat-getup.md)) |
| Flat on the face or back | A search over 24 flat starts found a way up from 6, all lying on the back, and none face down. Two get-up retrains on flat starts did worse and were rejected. M17 gets up from all 23 flat starts of the fresh set without retraining; one held-out flat-on-the-back start (low grip) still misses |
| Sit and stand | 40/40 randomised cycles, final depth error at most 1.34 mm; stands from 20/20 held-out sits |
| Following with the policy | 48/52 robustness runs at 1.1 m/s² (classical 41/52 at 1.25 m/s²); the one group behind has a named cause (the tag's range floor in dim light, which the classical controller also misses at 1.1) |
| Deployment parity | Drive, get-up and sit-and-stand exported with parity under 1e-6 and 0.27 to 0.71 ms per call on the Teensy model |
| `bipe sim` | From a clean clone, install plus the headless tour in under a minute on a 4-core CPU; the tour passes all eight checks |

Speed stays at 35/43: fine-tune 3, fine-tune 4 and a command shaper each failed
to beat fine-tune 1 without losing pushes or recovery, and the gap is accepted as
a known limit.

## M18: a simulation closer to the real robot

What the plants still left out about the real motors, sensors, timing and floor,
modelled in both the training environment and the full plant (off by default):
cogging, stiction, winding temperature, tyre radius, IMU mounting tilt and scale,
gyro bias drift, encoder offsets, timing jitter and lost CAN frames, floor grip
and softness, and a head payload. A new fidelity suite runs each model alone at
the wide end of its range, plus eight combined draws, under a hold, two pushes and
a drive. Plan and results:
[18-sim-to-real-plan.md](milestones/18-sim-to-real-plan.md).

| Check | Result |
|---|---|
| Fidelity suite, 92 cases | Locomotion fine-tune 1: 92/92 survive, 91 recover. Classical: 88/92, 74 |
| Drive retrained under the models | Four fine-tunes (two plain, two anchored to fine-tune 1) all lose to fine-tune 1 on the full plant (acceptance 83 to 86 against 89, speed 20 to 33 against 35); the plain ones drift more while holding still. Fine-tune 1 stays |
| Get-up retrained under the models | 60/60 held out (59), 40/40 fresh (40), 39/40 on robots drawn from the models (35), and 38/40 against 39/40 on a second, unseen fidelity set: no worse in total, a small margin; the new default |
| Timing | Jitter of whole 5 ms ticks makes the policy fall within a second; the firmware loop must stay synchronous, so a late reply counts as a lost one |
| Measured values | `config/hardware-measured.json` and ten bench tests ([bench-tests.md](hardware/bench-tests.md)) replace the assumed ranges once the robot is built; the suites run on the measured robot with `--measured` |

## Open gaps

- **Speed and stress.** 35/43 speed cases against a mark of 40 (accepted as a
  known limit), and 5.9 % stress falls against 5 %.
- **Get-up.** The flattest starts on the face or back (all three misses of M18's
  get-up, on robots drawn from the fidelity models; it passes all 60 held-out
  starts), a robot on its side, physical-limit checks before the handover, and a falls
  screen. The sit lands the shell at up to 0.51 m/s with no structural screen
  on that load yet.
- **Firmware.** The supervisor's switch between the three networks in C, and a
  memory plan for them: three networks do not all fit the Teensy's fast memory.
- **Everything physical.** Timing on the real Pi and Teensy, motor and belt
  parameters, friction and the camera's auto exposure all come from data
  sheets and models; M18 widened the models and wrote the bench tests that will
  replace them ([bench-tests.md](hardware/bench-tests.md)). The
  [hardware build plan](hardware/build-plan.md) lists what each build stage
  measures.
