# M6 measured RL environment and first policy results

Completed 2 October 2026 against the [M6 plan](06-rl-environment-and-policy.md).
M6 delivers a reproducible environment, a genuinely trained residual PPO actor,
portable weights and complete evaluation. The **hybrid** passes the original
27-case locomotion/diagnostic suite and half-timestep convergence. It **does not
improve on the classical controller** for the measured overall objective and
tracking metrics. The adopted controller remains unchanged. Independent learned
balance, MCU inference and hardware deployment are not established.

## Preserved baseline and architecture

The six-actuator serial-leg plant remains the exploratory baseline. Its native
assembly, seven rigid-body mass/inertia tensors, finite torque/speed envelopes,
20 ms nominal motor response and physical contacts are unchanged. The base is
free: 12 generalized velocities, six driven hinges and zero equality constraints.
No physical pose constraints or state clamps stabilize training or acceptance.

The [baseline identity snapshot](../../../artifacts/milestone-6/baseline-identity.json)
binds the SI configuration, controller, M4 sensors, C library, adopted native CAD
and user's modified M3 CAD save. All six hashes still match. The 2,391-file M4
seal and M5 release archive remain intact. No historical result is overwritten.

The estimated mass remains 2.499519 kg, width 284 mm and estimated mass margin
0.481 g. Actuator/pack variants, production interfaces and mass/width tradeoffs
remain future hardware-design decisions; choosing this learning plant does not
qualify them.

## Environment and action boundary

- Actor: 42 IMU/encoder-estimate, command, controller-reference/state and
  previous-action features. It receives no simulator root pose or oracle state
  in the realistic sensing path. Ideal sensing remains a labeled diagnostic
  branch of the original acceptance protocol.
- Network: 42 → 32 → 32 → 6, tanh hidden/output activations, 2,630 FP32 parameters
  and 10,520 raw weight bytes. A separate critic uses the same observations.
- Actions: 50 Hz dimensionless residuals in [-1,1], capped at 0.20 Nm per leg
  actuator and 0.06 Nm per wheel. These are corrections around the required
  classical C controller, not an independent balance policy.
- Classical estimator/body loop: 200 Hz; local leg feedback: 1 kHz; native
  physics: 1 kHz. Combined requests retain the existing supervisory gate,
  torque/speed limiting and motor lag outside the network.
- Episodes: 12 s, initial pitch offset ±3°, staged hold → drive/turn →
  crouch/combined commands. Settings, reward terms and randomization ranges are
  saved in [learning settings](../../../config/learning.json).
- Training samples copied mass ±5%, chassis COM x ±1 mm, friction 0.65–1.0,
  response scaling 0.85–1.15, noise scaling 0.75–1.5 and transport delay 5–10 ms.
  These ranges are experiment assumptions, not measured hardware bounds.

Six meaningful environment tests check zero-residual reference trajectory
agreement, seeded resets, absence of an actor truth input, free-base/contact/
finite-limit behavior, physical unpowered falling, and failure/timeout GAE
semantics. The complete Python suite passes **28 tests** in 3.31 s; source checks
pass. The disabled model falls physically rather than being held upright.

## Tooling and GPU outcome

The isolated pinned learning environment lives at `.venv/rl`. MuJoCo 3.14.0,
JAX 0.11.2, Optax 0.2.8 and Gymnasium 1.3.0 are recorded in
[requirements-rl.lock](../../../requirements-rl.lock). The base environment remains
available for the earlier tools.

CUDA works on the inspected RTX 5050 Laptop GPU. Native complete-environment
throughput is **132.42 policy steps/s** for the single-environment benchmark.
The separate MJX-Warp physics microbenchmark removed 70 noncolliding visual
geometries without changing native physical trajectories. Its 10-step GPU/CPU
differences were 6.94e-8 in qpos and 2.03e-5 in qvel; cold compilation/first
execution took 140.77 s and warm single-world physics measured 2,311 steps/s.

This short physics check does **not** qualify a GPU training environment with
the C estimator/controller and sensor transport. The GPU contact warning is
retained. Training therefore uses native CPU rollouts and CUDA neural
sampling/updates. See [native benchmark](../../../artifacts/milestone-6/tooling/native.json)
and [GPU benchmark](../../../artifacts/milestone-6/tooling/gpu.json).

## Training and independent selection

The run completed **192 PPO updates**, **196,608 transitions**, all three
curriculum stages and **320 completed episodes**, with no terminated training
episodes. Aggregate simulated rollout time is 3,932.16 s. The training loop,
including checkpoint validation, took 948.76 s; this excludes setup, benchmarks
and the initial control validations.

Training seed is 20261002. Every 16 updates retained actor weights, critic,
optimizer leaves, RNG state and validation results. Each of the 192 rollout
files contains measured observations/actions/rewards, pre-tanh likelihoods,
values, terminal/timeout flags, bootstrap observations and GAE/returns. All
196,608 saved transitions have finite arrays and bounded observations/actions.
Reset settings and episode outcome/load traces are retained alongside them.

Only full-curriculum checkpoints were eligible for delivery. **Update 176**
was selected using nine separate nominal validation trials, three curriculum
phases × seeds 20261101/20261102/20261103. Final acceptance cases did not select
the checkpoint. Selected actor parameters changed by up to 0.127507 from
initialization; this is an optimized policy, not a renamed initial network.

| Control | Mean validation return | Failed short trials | Final scenario cases | Convergence |
| --- | ---: | ---: | ---: | --- |
| Classical only | 52.083770 | 0/9 | 27/27, preserved matched M4 evidence | Pass |
| Untrained deterministic residual | 52.086135 | 0/9 | 27/27, newly evaluated | Pass |
| Trained deterministic hybrid | 49.984958 | 0/9 | 27/27, newly evaluated | Pass |

The selected return is **4.03% below** classical. The tiny untrained difference
does not establish a meaningful improvement. Training reward is not an
acceptance result. Twelve-second validation episodes also do not substitute
for the long hold cases in the final suite.

See [training report](../../../artifacts/milestone-6/training/training-report.json),
[selected weights](../../../artifacts/milestone-6/training/policy.npz),
[actor contract](../../../artifacts/milestone-6/training/policy.json), and
[learning/control plot](../../../artifacts/milestone-6/learning-and-control.png).
The training report's acceptance flag records its pre-final-evaluation status;
final hybrid acceptance is reported separately below and in the verifier.

## Physical comparison and acceptance

The unchanged protocol includes 26 powered scenarios and an intentional
unpowered fall diagnostic, ideal and sensor sensing, ±5° initial offsets,
pushes, hold/drive/turn/crouch/combined motion, extra seeds and individual mass,
friction, delay/noise and wheel-lag variations. Both residual policies have all
27 case configurations, physical traces/poses, XML and actor traces saved.
The original limits and acceptance thresholds are unchanged.
All saved actor traces are finite, obey their torque/feature bounds, update at
50 Hz and agree with re-inference of the exported network within 0.00001.

All case physical/nominal-controller parameters, sensors, seeds and firmware
build identities match the preserved M4 cases. Zero-residual trajectory parity
was tested separately. The classical comparison reuses that unchanged accepted
evidence; it is not a new classical run.

| Sensor-feedback metric | Classical | Untrained | Trained hybrid |
| --- | ---: | ---: | ---: |
| Hold maximum pitch error (°) | 0.684 | 0.686 | 1.581 |
| Hold maximum travel (mm) | 22.407 | 22.369 | 20.600 |
| Drive plateau speed MAE (m/s) | 0.01327 | 0.01365 | 0.02381 |
| Drive stopping travel (mm) | 88.036 | 87.769 | 92.056 |
| Turn plateau yaw-rate MAE (rad/s) | 0.00452 | 0.00424 | 0.03798 |
| Combined maximum joint error (°) | 1.626 | 1.622 | 2.791 |
| Combined final heading error (°) | 0.587 | 0.569 | 0.173 |

Some isolated metrics improve, while tracking and several pitch/joint metrics
get worse. No overall advantage or controller adoption is claimed. Passing
hybrid cases includes substantial work done by the classical controller.

The trained half-timestep combined run passes. Relative to 1 ms physics,
differences are 0.08966° peak pitch, 0.001378° final heading, 0.04442 mm height
range, 0.002150 Nm joint peak torque, 0.0001211 Nm joint RMS torque and
0.02759 rad/s joint peak speed. These meet the original convergence bounds.

Evidence: [trained acceptance](../../../artifacts/milestone-6/acceptance/acceptance.json),
[untrained acceptance](../../../artifacts/milestone-6/untrained-acceptance/acceptance.json)
and [matched comparison](../../../artifacts/milestone-6/comparison.json).
The additional historical 12-case M4 mechanical uncertainty study and electrical
stop/thermal analyses are not newly qualified for this learned controller.

## Export precision, inference and inspection

The first strict GPU-default/NumPy export check **failed**: the maximum
normalized action difference was 0.000583962, exceeding the 0.00001 check.
Batch-size/precision diagnostics show that CUDA's default batched matrix
arithmetic causes this difference. The failure and diagnostic are preserved.

The same JAX MLP with `highest` matrix precision as the FP32 reference matches
the portable NumPy actor within **2.38e-7** across 10,000 recorded inputs.
Portable FP32 math parity passes; **actual default CUDA training execution
does not pass the strict action parity check**. Its observed action gap implies
at most 0.0001168 Nm leg / 0.00003504 Nm wheel difference for the same input,
not a bound on closed-loop trajectory differences. Both validation selection
and complete final suites use the delivered NumPy actor. No weights or physical
limits were changed to resolve the export check.

Host-only NumPy inference measured 3.938 µs median and 7.244 µs p99. This excludes
sensing, classical control and transport, and says nothing about MCU execution.
See [policy profile](../../../artifacts/milestone-6/policy-profile.json),
[initial failed check](../../../artifacts/milestone-6/policy-profile-initial-failure.json)
and [precision diagnostic](../../../artifacts/milestone-6/export-precision-diagnostic.json).

The comparison plot, initial pose and 22 s replay frame were visually inspected:
labels are readable, eyes face forward, knees fold backward and the recorded
robot remains on wheel contact. The 1280 × 720 H.264 replay has 1,081 frames at
30 fps. It renders saved physical states, rather than rerunning physics.
See the [replay](../../../artifacts/milestone-6/replay/simulation.mp4) and
[inspection record](../../../artifacts/milestone-6/replay/review.json).

## Completion, reproduction and next work

The [M6 verifier](../../../tools/verify_rl_milestone.py) checks the evidence seal,
source/configuration bindings, preserved M4/M5 baseline, full training and
saved data, independent checkpoint selection, FP32 export parity with its
CUDA limitation, both complete policy suites and inspected replay. A copied
weight-file corruption test fails as expected without modifying source weights.
M6 completion concerns an **experimental hybrid learning deliverable**;
candidate locomotion acceptance, controller adoption and hardware qualification
are separate fields in [verification](../../../artifacts/milestone-6/verification.json).

```bash
.venv/bin/python tools/verify_rl_milestone.py
```

Use [the workflow](../rl-workflow.md) to recreate the environment and run new
training/evaluation into fresh folders. Keep the sealed run unchanged. Focused
local checkpoints record the plan, environment diagnostics, completed training
and final evaluation/verification.

The next recommended learning study is a controlled improvement experiment:
align reward/episode durations with hold, speed/yaw tracking and stopping;
record matrix precision explicitly; compare multiple training seeds using a
separate validation set. These measured regressions do not justify simply
extending this run or replacing the classical controller. Geometry refinement,
terrain/recovery, end-to-end learned balance, MCU integration and hardware
transfer remain later work. Create the next detailed milestone plan immediately
before that study starts.
