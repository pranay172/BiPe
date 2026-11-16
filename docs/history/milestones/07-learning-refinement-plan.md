# M7 controlled residual-learning refinement plan

Started 2 October 2026, immediately before implementation. Proposed requirements
and gates below are not achieved results; measured outcomes belong in `07-results.md`.

## Scope and hypothesis

M6's first residual PPO actor passes the locomotion suite but worsens tracking
and its measured validation return. Retain the accepted physical plant, classical
C controller, sensor model, 42-feature actor interface, 50 Hz cadence and six
residual torque caps. M7 tests one declared refinement bundle; it does not
attribute any improvement causally to individual settings.

The bundle uses a persistent mix of longer hold, drive/reverse/stop, turn/stop
and crouch/combined episodes; tracking-sensitive reward costs; a longer discount
horizon; gentler exploration around an exactly zero-residual initialization;
and explicitly recorded FP32 matrix precision. Three independent PPO seeds
receive equal, predeclared transition budgets. Keep classical and frozen M6
controls as matched physical comparisons. No fabrication, flashing or hardware
actuation is part of this milestone.

## Preservation and physical boundary

- Keep `config/robot.json`, adopted CAD, C library/firmware, original sensors,
  M6 source/configuration/evidence and the user's modified M3 save unchanged.
- Preserve the original 698-file M6 seal. Archive only its sealed README and
  project overview before updating live documentation, with an authenticated
  map tied to checkpoint `046782e`. The M7 verifier may substitute only these
  two archived documentation paths into the unchanged legacy verifier; no
  physical/core/evidence remap is permitted. Never overwrite M6 verification.
- Add M7 modules/settings/tests beside frozen M6 modules. Reuse their physical
  C/device/contact boundary. Free base, finite motor limits, physical contacts,
  failure termination and lack of pose constraints/state clamps remain required.
- Dependencies remain in `.venv/rl`; record exact existing lock and packages.
  CPU rollouts preserve native sensing/contact/C behavior. CUDA performs PPO
  optimization. Host sampling may use an explicitly verified FP32 equivalent
  to remove per-step GPU transfers; save its RNG and density/parity diagnostics.

## Declared training experiment

- Seeds: **20261020, 20261021, 20261022**. Each gets **192 updates × 128 steps ×
  8 environments = 196,608 transitions**. Total budget 589,824 transitions.
  Retain failures and do not extend budgets selectively to rescue a seed.
- Episode families remain mixed throughout, each with probability 0.25.
  Hold durations sample 30/45/60 s. Drive, turn and combined episodes include
  opposing commands and explicit post-stop holds, lasting roughly 28–40 s.
  Sample commands, timing and crouch strokes independently from final test tables.
- Preserve M6's declared copied-plant randomization ranges; these are simulation
  assumptions, not hardware bounds or qualification of arbitrary terrain.
- PPO actor/critic stay small, with the same 42 actor inputs and six outputs.
  Initialize the final actor layer to zero; initial deterministic actor equals
  classical-only. Initial Gaussian standard deviation 0.15, learning rate
  0.0001, gamma 0.999 (~20 s discount time constant), GAE lambda 0.99 (~1.8 s
  trace time constant), zero entropy bonus, existing clipping/gradient safeguards.
- Use negative tracking/posture/hold/residual/effort costs rather than a broad
  positive survival return. Normalize speed error at 0.03 m/s and yaw error at
  0.04 rad/s. Track the classical rate-limited speed reference and pitch reference
  during transients so reward does not punish their necessary dynamics; physical
  test metrics use the actual requested commands. Use continuous Huber costs
  for tracking/posture/drift, finite failure penalties and no physical clamps.
  All coefficients/scales and exact command tables must be saved before training.
- Explicitly set JAX matrix precision to `highest` before sampling/optimization.
  Verify FP32 host/GPU actor, critic and log-density agreement on recorded inputs.
- Save configurations/source identities, all sampled resets and complete command
  schedules, observations/actions/likelihoods/rewards and physical reward traces,
  bootstrap observations/GAE, optimizer/RNG checkpoints and episode outcomes.
  Log critic explained variance, reward components, action means/near-limit
  fractions and tracking/hold errors in addition to aggregate PPO losses.

## Validation, selection and independent testing

Create explicit fresh task tables at implementation start and seal their hashes
before training. Selection and final test offsets, sensor seeds, command sizes,
timing and durations must be distinct; neither final test nor inherited acceptance
may select weights. Both use realistic sensor feedback and identical plant/job
settings for every compared control.

- Selection: four task families × two fresh seeds = eight trials, including a
  30 s hold and roughly 28–36 s drive/turn/combined cases. Compare classical and
  frozen M6 once. Evaluate eligible M7 checkpoints only at updates 64/128/192
  with bounded independent native workers. Select each seed's candidate from
  this physical validation score with a failure penalty; deliver negative
  results too, but never describe best-available selection as improvement.
- Lock all three candidates and record weight/metadata hashes before final
  testing. Test: four families × three different seeds = 12 paired trials per
  control, including a 60 s hold, new speed/yaw commands and longer final holds.
  Compare classical, frozen M6 and all three selected M7 actors on the same jobs.
- Report speed/yaw plateau MAE/IAE, sustained stopping time and displacement,
  hold RMS pitch/drift, heading, joint error, saturation and authoritative
  1 kHz torque/speed loads. Compute one predeclared normalized physical score
  with equal family weight, not raw returns across different reward definitions.
- Run the **complete inherited 27-case suite plus half-timestep convergence for
  each selected seed policy**. Keep thresholds unchanged. Save exceptions as
  failed jobs with configuration/traceback and continue remaining cases; report
  convergence as unavailable if required traces cannot be obtained.

## Proposed advancement gates

This is a study completion gate and a separate experimental-improvement gate.
M7 may complete with a negative finding if all required runs/evidence are retained.
Keep the classical default and hardware qualification false regardless of a
short study's result; operational adoption needs stop/thermal/inference work.

An improvement claim requires all three inherited suites/convergence to pass,
zero failures on independent paired tests, a **median training-seed physical
score gain of at least 10%** over classical and gain in the same direction for
all three seeds. No task-family median may exceed classical's score by >5%.
For individual tracking errors permit at most 5% + 0.0005 m/s speed or 0.001 rad/s
yaw numerical allowances; hold/stopping displacement permits 5% + 1 mm.
Pitch/joint peaks may increase by at most max(10%, 0.25°), motor RMS by at most
10% + 0.005 Nm and saturation by at most 0.05 s while retaining the 0.5 s hard
limit. Explicitly report failed gates, all seeds and paired deltas/median/range.
Three training seeds and small paired tests do not establish broad superiority.

## Implementation and completion evidence

1. Commit this plan before implementation; authenticate/archive the M6 baseline.
2. Add fresh tasks, reward/environment, recorded FP32 sampling, multi-seed PPO,
   complete evaluation and preservation verification. Test schedule continuity,
   reward semantics, zero-action physical parity, timeout and export/density math.
3. Record fixed settings/task hashes, baseline validations and benchmark the
   chosen rollout path; commit verified implementation before full training.
4. Train exactly the three declared budgets, select/lock independently, and run
   all paired tests and inherited acceptance suites. Retain failures and seeds.
5. Produce measured comparisons, uncertainty/limitations, inspected plots/replay,
   reproduction commands, results and evidence seal. Verify M4/M5/M6 preservation
   and user CAD identity, and create focused local verification commits.

Future terrain/recovery, standalone learned balance, geometry changes and MCU/
hardware deployment stay in the overview until their own implementation starts.
