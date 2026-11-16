# M14: release-grade locomotion

Plan written 29 October 2026. User decision (30 October 2026): approved as
recommended, after M13 (PR #3) and the release prep (PR #4) merged:
1. lash model (a): MuJoCo dead-band tendon springs and RO40 rotor joints,
   with the torque-side dead-band (b) as the fallback if the Warp step rate
   drops by more than 30 %;
2. the stress ranges as listed in phase B;
3. at most 6 h per seed on the laptop GPU, two seeds;
4. the speed-suite policy rule proposed in phase D, for sign-off before the
   rerun.

As before, nothing is bought, built or powered.

M14 closes roadmap v2 gates 2 to 5 for the locomotion policy:
- recovery at least as good as the classical controller;
- a policy form of the speed suite's judging rule;
- a training recipe anyone can rerun from scratch;
- a robustness margin that makes the later sim-to-real step credible.

The full M10 plant stays the judge. A result measured only on the training
plant is a training result.

## Exit gates (from roadmap v2)

| Gate | Pass mark | `settle` today |
|---|---|---|
| 2. Recovery | at least the classical 83/98 acceptance cases recovered, belt range at least 6/8, pushes 30/30, 0 physical failures in 98 + 44 | 80/98, belt 4/8 (the 98 the policy can run; M13 PR review corrections) |
| 3. Speed suite | policy joint-error rule signed off; at least 40 of the 43 judged cases pass (`low_posture_drive_ramp` is information only) | 1/44 under the classical rule |
| 4. Reproducible | `bipe train --task locomotion` from scratch; 2 seeds both meet gate 2; at most 6 h each on the RTX 5050 | 7-run fine-tune chain |
| 5. Stress margin | at most 5 % terminations on the stress mix in training; full-plant spot check on 24 stress cases, 0 physical failures | not built |

Revised 2 November 2026 (Pranay, after the feasibility study below): gate 5's stress
pushes are 2.5 N s (delays and lags stay at 1.5x), training-plant slopes run up or down
the heading, and the full-plant spot check is 26 cases (the three scenarios under eight
stratified draws from inside the ranges, plus the two pushes on the nominal plant),
26/26 surviving. Phase E judges against the revised gate.

## Phases

**A. Belt lash in the training plant** (cloud CPU, then one GPU check).
- The belt-range family varies hip and knee lash with belt stiffness at the
  range corners. The training plant has rigid hip belts and an elastic knee
  belt with no lash.
- Options:
  - (a) MuJoCo tendon dead-band springs (two-valued `springlength`) on the knee
    belt, plus RO40 rotor DoFs with dead-band hip belts. This is physical and
    adds 4 DoF.
  - (b) A dead-band and spring between the commanded and applied joint torque,
    inside the torch action term. This is cheaper but not a physical lash.
- Recommended: (a), if the Warp step rate drops by less than 30 %; otherwise
  (b).
- Exit: the training plant agrees with the full plant on the 8 belt cases
  (pass/fail and late pitch peak-to-peak within 0.5°). The 91-case gap suite
  does no worse than for `settle`.

**B. Actuator and power randomisation, and the stress suite.**
- Per-world DR:
  - RO40 and GL40 torque constant ±20 %;
  - Coulomb and viscous friction ±50 %;
  - FOC lag up to 2×;
  - 6S pack 19.8 to 25.2 V with sag under load (the GL40 voltage limit
    follows it);
  - COM offset ±5 mm;
  - IMU bias;
  - lash and belt stiffness over their ranges.
- Stress mix: these ranges plus 1.5× the acceptance ranges (delay, encoder
  and hub lag, pushes, slopes to 10°).
- Full-plant stress set: 24 cases sampled from the corners, run with the
  classical controller too, for reference.

**C. One recipe from scratch.**
- Fold the seven fine-tune lessons into one curriculum:
  - robot sensor channels;
  - the axle COM-lean fall;
  - the stand-still and rest-lean terms;
  - chassis-force pushes;
  - lash and actuator DR, ramped in over training.
- Entry point: `bipe train --task locomotion --seed N`, wrapping
  `bipe.m13.rl.train`.
- Two seeds on the laptop GPU, each at most 6 h, then the full plant
  (cloud CPU, about 4 to 6 h per policy).
- If a seed misses gate 2, iterate on the curriculum, not on fine-tunes.

**D. Speed-suite rule for a policy.**
- The classical rule (each joint within 5° of the posture) judges posture
  holding, which a policy does not do by design.
- Proposed policy form:
  - axle height within the posture band (±5 mm p95; signed off 1 November 2026 as
    the p95 over steady driving, see "Evaluation tooling");
  - the existing speed, yaw and pitch criteria unchanged.
- For your sign-off before the rerun.

**E. Exit.**
- Choose the M14 policy.
- Export it through the M13 phase D pipeline (ONNX, manifest, C parity).
- Update the README status and record the results.

## Phase A results (30 October 2026)

Option (a) is in: `training.drives` `hip_belt: "spring"` and `knee_lash: "deadband"`
(the M13 defaults, rigid and static, are unchanged and reproduce M13).
- Thigh rotors are bodies; each hip belt is a tendon dead-band spring between rotor
  and hinge, and the hip encoder reads the rotor, as on the robot. The knee rotor
  stays folded on the knee-drive hinge (too light to sit between two springs at
  1 ms), so the knee chain is one spring: hip and knee belts in series, with both
  lashes as its dead band. +2 DoF (14 to 16).
- The stiffest hip belt in the range (713 Nm/rad) puts the rotor mode near 300 Hz,
  unstable for an explicit spring at 1 ms. Only there, the rotor gets just enough
  extra armature for omega*dt = 1.6 (`plant.rotor_armature_floor`); nominal belts
  get none.
- RL env: `env_cfg(belts=True)`; `mdp.randomise_drive` draws each world's belt
  stiffness (log-uniform) and lash over the belt-range corners; the critic sees them.
- Step rate (CPU Warp proxy, 256 worlds): about 18 % slower, inside the 30 % limit.
  The laptop GPU check is still to do.

Classical controller, training plant vs full plant (`artifacts/m14/validation`):
- 8/8 belt-range cases pass on both, late pitch peak-to-peak within 0.25 deg.
- Physical pass/fail agrees in 149 of 151 cases (M13 plant: 134 of 135).
- One regression: `delay_15ms_noise_x2_drive` recovers on the full plant but holds a
  9 deg limit cycle on the training plant. The trigger is the knee chain's
  compliance (it recovers at knee k 69, the M13 value, and not at the 57.7 series
  value), with the classical controller at its margin under 15 ms delay. Seed, time
  step, hip stiffness and lash do not change it. Accepted for now: it makes the
  training plant harsher, not easier, at the delay corner.
- Splitting the hip Coulomb friction to keep the M13 fitted total (tried and
  reverted) made the delay corner worse, so the rotor keeps the motor's own friction
  as in the full plant.

Policy gap suite, `settle` on the training plant vs its full-plant results
(`artifacts/m14/settle_gap`, `python -m bipe.m13.full_plant gap --reference
artifacts/m13/phase_c/settle_ --hip-belt spring --knee-lash deadband`):

| | M13 plant | M14 plant |
| --- | --- | --- |
| Survival agrees, M10 cases | 90 of 91 | 106 of 107 (16 new belt and lash cases all agree) |
| Recovery agrees, M10 cases | 88 of 91 | 102 of 107 (86 of the old 91) |
| Pass/fail agrees, all cases | 134 of 135 | 150 of 151 |

Survival is unchanged. Recovery disagrees in 2 more of the old cases
(`backlash_drive_0`, `mismatch_body_mass_scale_0.85_combined` recover on the
training plant only; `mass_plus_15pct_combined`, `mismatch_body_mass_scale_1.15_hold`,
`slope_down_0` only on the full plant), all borderline settling calls with no fall.
`settle` was trained on the rigid plant, so this is a check of the plant, not a
policy result.

GPU step rate (RTX 5050 laptop, 4096 worlds, 30 iterations, 30 October): M13 `settle`
2.14 s per iteration, M14 `locomotion` 2.42 s, 13 % slower, inside the 30 % limit.
The compiled model has the intended 1 ms step and solver; the mjlab attach warning
about the robot's own options is cosmetic.

## Phase B results (30 October 2026)

Per-world randomisation in the RL env, behind `env_cfg(actuators=True)` (M13
defaults unchanged), drawn by `mdp.randomise_drive` at each reset:
- torque constant (the applied torque scale) 0.8 to 1.2 per actuator;
- RO40 FOC lag x 1 to 2 (the GL40 lag already spans 0.8 to 25 ms);
- every joint's Coulomb friction and viscous damping x 0.5 to 1.5;
- pack open-circuit voltage 19.8 to 25.2 V, with sag through the pack and harness
  (0.075 ohm, from the M11 0.50 V at 6.63 A; x 1 to 2) from the motors' mechanical
  power and the hubs' copper loss. The GL40 voltage limit is 0.95 x bus and the RO40
  no-load speed scales with bus / 25.2 V;
- gyro bias +-1 mrad/s and accelerometer bias +-0.1 m/s^2 (assumed ranges);
- chassis COM x and z +-5 mm.
The critic sees the new parameters. `env_cfg(stress=True)` draws the stress mix:
transport delay to 23 ms, encoder lag to 8 ms, hub lag to 37.5 ms, pushes to
3.75 N s, slopes to 10 deg.

Stress suite, the gate (`python -m bipe.m13.rl.evaluate CKPT --stress`): falls
over episodes with belts, actuator DR, the stress mix, robot sensing and the full
plant's pushes and falls. `settle` (trained without any of it), 64 episodes of
10 s on CPU: 20 % falls against the 5 % gate (`artifacts/m14/settle_stress_cpu64.json`).

Full-plant stress set (`python -m bipe.m14.stress [--checkpoint CKPT]`): 24 cases,
a 3.75 N s push forward and back while holding and a drive up 10 deg, each under 8
balanced corner draws of all the ranges above at once, with 22.5 ms delay and
noise x 2 (no bus sag: the full plant has no bus model).

| Controller | Survived (of 24) | Pushes survived (of 16) | Slopes survived (of 8) |
| --- | --- | --- | --- |
| Classical | 0 | 0 | 0 |
| `settle` | 12 | 4 | 8 |

All corners at once are harsher than any acceptance case: the classical
controller falls in every one (its 15 ms delay case is already at its margin).
These are references for the phase C recipe, not gates.

Phase B open item: none in the cloud. The training recipe (phase C) needs the
laptop GPU.

## Phase C recipe

`python -m bipe train --task locomotion --seed N` (variant `locomotion`, from scratch):
robot sensor channels, the axle COM-lean fall, the stand-still (x 2) and rest-lean
terms, chassis-force pushes, the training mix, belts and actuator DR ramped in
linearly over the first 2000 iterations (`env_cfg.RAMP_ITERATIONS`). 8000 iterations
are about 5.4 h on the laptop GPU. Runs go to `local/m14/runs`.

## Training-run blow-ups and the rotor margin (31 October 2026)

The first two seed 1 runs failed on the laptop: one stopped on a NaN at iteration
1104; the other learned well to about iteration 1000 (reward 48) and then collapsed
as some worlds blew up to huge finite values (rewards to -1e17, value loss to inf).
Random violent actions on CPU reproduce it: the thigh rotors spin up to 5000 rad/s
with the hip belt springs; with the belts off, actuator DR alone is clean. The
rotor-belt mode at omega*dt 1.35-1.6 holds the classical controller but not RL
actions. `drives.rotor_omega_dt` now defaults to 1.0 (5345c7d): the floor applies at
every belt stiffness (at nominal about 0.8x the rotor's own spin inertia); 1.0 and
1.2 show no blow-ups. A world whose physics goes non-finite also ends and resets.
- Belt cases: 8/8 pass, late pitch within 0.31 deg of the full plant.
- Full validation rerun (`artifacts/m14/validation`, replacing the 1.6 run): physical
  pass/fail agrees in 149 of 151 (unchanged); band misses 22, down from 31.
- The laptop runs must be launched so they outlive the remote session (the second
  run also stopped at iteration 1490 when the session's process exited).

## Evaluation tooling (1 November 2026)

`python -m bipe.m14.evaluate run CHECKPOINT` scores a trained policy against the exit
gates and against `settle`, so each seed can be judged as soon as it finishes. The
checkpoint must sit inside the repository (copy a laptop run's `model_N.pt` under
`local/`). Results go to `local/m14/eval/<run>_<checkpoint>`; `--stages` picks stages,
finished stages are skipped, and acceptance resumes.

| Stage | What runs | Gate | 4 CPUs |
|---|---|---|---|
| `hardware` | firmware runtime with the M11 image's timing (`bipe.m13.handover --variant hardware --checkpoint`) on the 8 belt cases and the 10 PR #5 handover cases | timing check | about 25 min |
| `stress` | the 24-case full-plant stress set (`bipe.m14.stress`) | 5 | about 30 min |
| `acceptance` | the 107 M10 acceptance cases (`bipe.m13.full_plant acceptance`) | 2 | a few hours |
| `speed` | the 44 M12 speed cases, with the policy rule's metrics | 3 | about 1 h |
| `training` | the training-plant stress suite (`bipe.m13.rl.evaluate --stress`, CPU, 64 episodes by default) | 5 | |

Each run writes `report.md` and `scores.json`: every gate against its pass mark and
against `settle` (`artifacts/m14/eval/settle`, which points at settle's committed
records), the cases gained and lost against `settle`, and the verdicts within
0.005 m/s or 0.3 deg of the recovery band (single runs there flip on tiny
perturbations, PR #3 review). `--no-baseline` leaves the comparison out. `python -m bipe.m14.evaluate score DIR1 DIR2` puts two
seeds side by side.

- Counting: the 98 cases the policy can run (107 less the 8 ideal-sensing cases and
  `unpowered`); the 79 matrix cases by the recovery rule, the 19 scenario cases by their
  own pass rule without the joint-error criterion. This reproduces the classical 83/98
  and settle's 62/79, but gives settle 76/98 recovered (81 survived), not the M13
  review's 80, which the committed records do not reproduce under any one rule.
- Speed suite: each case now records the joint-exempt verdict and the axle depth error
  (the measured axle height against the commanded posture's depth,
  as the chassis origin above the mean wheel centre, the RL training's posture metric).
  The phase D rule, signed off 1 November 2026 (user decision, "steady driving, within
  5 mm"): the suite's checks without the joint-error criterion, plus the axle height
  p95 within 5 mm over the case's steady segments. A steady segment is as the training
  metric takes it: the shaped speed reference and the yaw-rate command held (within
  0.01 m/s and 0.02 rad/s) for the last 1 s, rest included (`speed.steady_mask`). A
  first version used only the cases' speed plateaus, which left 16 of the 43 judged
  cases (the replays, `push_fast_±2`, `spin`) unchecked (PR #8 review); the report
  lists any judged case still without a steady segment. The drafted whole-case p95
  passed `settle` in only 4 of 43, because the robot dips 7 to 10 mm for about a
  second when it starts and stops, and holds about 3 mm low while driving.
- The baseline's acceptance and speed suites were rerun on this branch's code
  (`artifacts/m14/eval/settle`). Against phase C's records: acceptance 74/98
  recovered (60/79), belt 2/8, pushes 30/30, 0 physical failures; it gains
  `backlash_drive_0` and `mismatch_body_mass_scale_0.85_combined` and loses
  `belt_hold_h0.45_n0.225_k1` and `belt_hold_h0.45_n0.9_k1` (rest speed 0.031 and
  0.037 m/s against the 0.03 limit) and two mismatch cases: borderline rest-speed
  calls, no falls (3 of the 6 within the evaluator's 0.005 m/s margin). Speed: 37/44
  survive and recover.
- The baseline is specific to its environment: only 28 of 107 acceptance cases keep
  phase C's late rest speed in the cloud rerun, though reruns in the same environment
  are bit-exact (PR #8 review). Score seeds in the same cloud environment (a `.venv`
  from `requirements.lock`, torch 2.15 from PyPI, mjlab 1.6.0, rsl-rl-lib 5.4.2,
  mujoco held at 3.14). Each stage records its commit, a dirty-tree flag and the
  package versions in `eval.json`.
- Gate 2 is judged on the acceptance suite, at the simulation timing the plan defined
  it on (`settle`: belt 2/8). The hardware stage (the M11 image's timing, one tick
  late, which the robot will run; `settle` 8/8, 7/8 under a 1e-5 rad perturbation) is
  reported beside it and not gated.
- Gates are judged only on complete stages: a stage that covers less than its full
  case set (98 policy cases with 8 belt and 30 pushes, 44 speed, 24 stress, 8 + 10
  hardware) or exited non-zero leaves its gates "incomplete"; gate 2's physical
  failures need both acceptance and speed.
- Training stress: `--episode-s` defaults to 10 s, as `settle`'s 20 % reference ran
  (64 x 10 s), and the report gives a 95 % interval. 64 episodes cannot separate 3 %
  from 8 %; for the gate call, run more (for example `--device cuda:0 --worlds 1000
  --episodes 1000` on the laptop) and rerun `settle` at the same setting.
- 16 of `settle`'s 98 verdicts sit within the band margin. A seed that lands within a
  few cases of 83 should have its at-margin cases rerun with a small perturbation
  before gate 2 is called.
- The hardware stage builds a forward pass and a `locomotion_config.h` naming the
  checkpoint under `firmware/build/checkpoints/` (the M13 interface is unchanged by the
  recipe). Run on a copy of `settle` it reproduces the committed belt and handover
  records exactly (8/8 belt, 8/10 handover cases, identical peaks and rest speeds).

## Seed results and stress feasibility (2 November 2026)

Seeds 1 and 2 (8000 iterations, 5.9 h each, no NaNs; `artifacts/m14/eval/seeds_report.md`)
both pass gate 2 (84 and 89 of 98 recovered, belt 8/8, pushes 30/30, 0 physical
failures; settle 74, 2/8) and miss gate 3 (26 and 29 of 43; settle 31) and both
stress gates: full-plant stress 9/24 each (settle 12/24), training-plant stress on the
laptop GPU (1000 x 10 s) 18.6 % and 26.7 % falls (settle 23.2 %;
`artifacts/m14/eval/training_gpu`).

Feasibility (`artifacts/m14/feasibility`, `bipe.m14.feasibility`):
- Training plant, falls per 1000 episodes (settle / s1 / s2): flat ground without pushes
  2 / 4 / 2; 10 degree slopes without pushes 33 / 16 / 32, mostly `rolled`: a cross slope
  near 10 degrees puts the robot at the 10 degree roll fault while it stands still. The
  full plant's slope cases climb along the heading only. Pushes dominate the rest: fixed
  pushes under the stress delays (flat) give 5-8 % falls at 1.5 N s, 11-17 % at 2.5,
  22-35 % at 3.0 and 49-72 % at 3.75. Training draws pushes from 0.5-3.0 N s (the
  `hard` mix), so the policies never saw the top fifth of the stress range. Actuator randomisation barely
  matters (stress without it: 224 / 232 / 240 against 222 / 213 / 260).
- Full plant, hold then push (`full_plant/`): on the nominal plant settle and seed 1
  recover 3.75 N s pushes both ways (classical survives 3.0 and 3.75 without
  recovering; seed 2 falls at 3.0 forward). At stress corner c0 (torque 0.8, friction
  1.5, 37.5 ms hub lag, 19.8 V, softest belts, 22.5 ms delay, noise x 2) every
  controller falls even at 1.5 N s; at c1 settle and both seeds survive up to 3.75 N s
  but none recovers. Without any push, classical falls at 5 of 8 corners and settle
  holds still at only 3 of 8.

So the 24/24 full-plant gate is infeasible as drawn (every parameter at a range end at
once, on top of 1.5x delay and doubled noise), the training-plant gate is dominated by
a roll-fault artefact on cross slopes and by pushes beyond the training range, and
3.75 N s pushes are physically recoverable on the nominal plant.

Decision (Pranay, 2 November 2026): revise the stress gates and fine-tune seed 1.
- Training-plant stress: slopes up or down the heading (`STRESS["slope_along_heading"]`),
  gate still at most 5 % falls.
- Full-plant stress: 26 cases, the three scenarios under eight stratified draws from
  inside the ranges plus the two pushes on the nominal plant; gate 26/26 survive.
- Push size (Pranay, 2 November, after the interior draws): stress pushes at the
  acceptance's 2.5 N s in both stress suites, everything else stays 1.5x. With the
  22.5 ms delay and doubled noise a 3.75 N s push reaches the 20 degree pitch limit
  about 70 ms after it lands (settle and seed 1 survive 1 of 16 interior push cases;
  3.0 N s about 8 of 16; 2.5 N s 14 of 16; `artifacts/m14/feasibility/full_plant_interior`). The corner set stays available (`bipe.m14.stress --corners`).
- Fine-tune: `python -m bipe train --task finetune --seed 1 --iterations 2000
  --resume-run <seed 1 run> --resume-checkpoint model_7999.pt` (env_cfg
  `locomotion_ft`: full DR from the start, pushes 0.5-3.75 N s, yaw rate tracking
  weights doubled).

## Fine-tunes and the stress gate near miss (2 November 2026)

- Fine-tune 1 (`finetune_s1/model_9998.pt`, 2000 iterations from seed 1, task
  `locomotion_ft`): reward 59 to 65. Fine-tune 2 (`finetune2_s1/model_10997.pt`, 1000
  more on the stress delays, task `locomotion_ft2`): reward 71. No NaNs in either.
- Training-plant stress (2.5 N s pushes, slopes along the heading, 1000 x 10 s, seeds
  11-15, `artifacts/m14/eval/training_gpu5`): fine-tune 1 5.88 %, fine-tune 2 5.94 %
  (95 % about 5.3-6.6 % each). Settle was not run on seeds 11-15: one unseeded
  1000-episode run gave 10.3 % (`training_gpu3/settle`), and the seeded per-world
  draws below 10.8 %. Earlier unseeded repeats reused
  torch's default random state, so they were not independent; the training stage now
  takes `--seed`.
- Per-world draws for fine-tune 1 (`bipe.m14.fall_draws`,
  `artifacts/m14/fall_draws/finetune_s1_seed1.json`, 54 falls in 1000 worlds): slopes
  up to 6 degrees 14/722 (1.9 %), 6-8 degrees 14/139 (10 %), 8-9 degrees 13/73 (17.8 %),
  9-10 degrees 13/66 (19.7 %). Steep slopes push it over 5 %, but only 11 of the 26
  falls at 8 degrees or more are `rolled` (turning brings the slope across the robot
  and trips the 10 degree roll fault, which the real supervisor would do too); the
  rest lean, tip or touch the shell. About half the falls (28) are on slopes under 8
  degrees, and 26 of those follow a push. Settle on the same draws: 108 falls, 49 of
  them on slopes up to 6 degrees.
- Full-plant stress, fine-tune 1: 25/26; the miss (`push_back_d6`: hub lag 33 ms,
  20.2 V pack, hip lag x1.9, with the 22.5 ms delay) also fells settle and seed 1.
  The gate counts survival only, and the margin is thin: fine-tune 1 recovers 10 of
  the 26 cases, and four peak above 18 degrees (`push_back_d2` at 20.0, at the pitch
  limit, plus `push_fwd_d6`, `push_back_d6` and `push_back_d7`). Expect stress-level
  pushes to be marginal on the bench.

Decision (Pranay, 2 November 2026): keep the gate at 10 degree slopes and at most 5 %
falls, and record the near miss with its cause; the M14 policy is chosen on the other
gates.

## Phase E: the M14 policy (3 November 2026)

Decision (Pranay, 3 November 2026): fine-tune 1 (`artifacts/m14/checkpoints/finetune_s1/model_9998.pt`,
seed 1's 8000 iterations of `locomotion` plus 2000 of `locomotion_ft`) is the M14
policy. It replaces settle as the policy the firmware's generated headers carry
(`bipe.m13.headers.DEPLOYED`; `firmware/include/locomotion_config.h` regenerated, the
action law, timing and posture table unchanged). The classical controller stays the
default at power-up until the bench check (M13 phase E).

| Gate | Result | Settle |
|---|---|---|
| 2. Recovery (>= 83/98), belt (>= 6/8), pushes (30/30), 0 physical failures | 89/98, 8/8, 30/30, 0: passed | 74/98, 2/8, 30/30, 0 |
| 3. Speed policy rule (>= 40/43) | 35/43: missed (path replays, `drive_step` stop, downhill: `slope_down_8` and `slope_down_10` fail to hold as well as the axle check, both lost against settle) | 31/43 |
| 4. Base recipe (`locomotion`): two seeds, at most 6 h each | 5.9 h and 5.9 h on the laptop RTX 5050, both meet gate 2: passed | - |
| 4. Deployed recipe (`locomotion` + `locomotion_ft`) | one seed, about 7.4 h (over the 6 h cap); not reproduced on a second seed | - |
| 5. Training-plant stress (<= 5 %) | 5.9 % over 5 x 1000 seeded episodes: near miss, recorded (steep slopes, and pushes on gentler ground) | 10.3 % (one unseeded run) |
| 5. Full-plant stress (26/26) | 25/26 (`push_back_d6`, which also fells settle and seed 1) | - |

Export (`artifacts/m14/export/locomotion`, `python -m bipe.m13.export`, `export_check`
with `--policy locomotion`):
- `policy.onnx`, `manifest.json`, `SHA256SUMS`;
- parity (`parity.json`): torch, ONNX Runtime, host C and the Cortex-M7 image under
  unicorn within 6.3e-7 of the float64 actor (tolerance 1e-4), on 4604 recorded and
  2000 random observations;
- budget (`budget.json`): 0.27-0.71 ms per call at 600 MHz (a model; the bench's DWT
  counter replaces it), 3.5 % of the 20 ms policy period;
- closed loop (`closed_loop.json`, `c_acceptance/`): the 107 acceptance cases with
  the C forward pass in the loop against the numpy actor's run: 99 compared, 98 same
  verdicts (`wheel_lag_25ms_drive` flips `passed_without_joint_error`), 99 within the
  metric bands, 0 physical failures. The flip costs a gained case, so gate 2 with the
  C forward pass is 88/98 (still a pass). Its final speed is 0.0060 m/s against 0.0035
  with the numpy actor; a numpy rerun of it and two other cases
  (`numpy_rerun/`) reproduces the eval exactly, so the flip is float32, not the
  environment.

Open after M14: the speed rule (35/43), including holding on 8 and 10 degree downhill
slopes, which settle did; the training-plant stress near miss; reproducing the
two-stage recipe on a second seed (a fine-tune from seed 2's `model_7999.pt`, about
1.5 h on the laptop GPU); the
bench checks of M13 phase E (forward-pass timing, on-board parity, the selection
switch) now carry this policy.

## Resources and timing

- GPU: the RTX 5050 laptop for phases A (check), B and C. Roughly 2 to 3
  training runs of up to 6 h, scheduled around your laptop hours.
- Cloud CPU: the fit, the gap suite and the full-plant runs.
- Estimate: about 2 weeks.
