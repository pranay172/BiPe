# M15: behaviours on the learned policy (plan 4 November, results 6 November 2026)

Approved by Pranay on 4 November 2026 with the recommended option for each of the
five decisions below. As before, nothing is bought, built or powered, and
every run file, checkpoint and log stays inside the repository (`local/m15/`,
gitignored, or `artifacts/m15/` for committed records).

M15 closes roadmap v2 gates 6 and 7 and takes another pass at the two gates M14
missed:
- getting up after a fall (gate 6);
- camera following with the policy instead of the classical controller (gate 7);
- the speed suite under the policy rule (gate 3, 35/43 after M14);
- the training-plant stress margin (gate 5, 5.9 % after M14);
- the learned-policy demo shots that the classical cut left out.

The full M10 plant stays the judge. A result measured only on the training plant
is a training result.

## Decisions for you

1. **How the get-up runs.**
   - (a) A separate get-up policy. The supervisor runs it only from a fallen
     state and hands over to the locomotion policy at the get-up study's handover
     test. The M14 policy and its gates are untouched.
   - (b) Fold get-up into the locomotion policy (fallen-state resets in its
     training). One network, but every M14 gate has to be passed again.
   - Recommended: (a). It keeps the risk inside the new behaviour, and the
     firmware already switches controllers at a handover (M13 phase E).
2. **Expressive moves.** Gate 6 reads "get-up at least 80 %, or a documented
   negative, plus 3 moves".
   - (a) Three moves as command sequences for the locomotion policy (it already
     takes speed, yaw rate and the posture label): a crouch bob, a spin with a
     stop, and a side-to-side wiggle. No training; judged on the full plant like
     any other speed case.
   - (b) Train dedicated move policies.
   - Recommended: (a). If a move needs something the command space cannot give,
     it is recorded, not trained.
3. **The speed and stress gaps.**
   - (a) One fine-tune round from fine-tune 1 aimed at the failing cases
     (below), at most 2 runs of about 2 h on the laptop GPU.
   - (b) Accept 35/43 and 5.9 % as recorded and move on.
   - Recommended: (a), with (b) as the fallback if two runs do not move both
     numbers without losing gate 2.
4. **Reproducing the deployed recipe (gate 4).** The deployed two-stage recipe
   (8000 iterations of `locomotion`, 2000 of `locomotion_ft`) took about 7.4 h on
   one seed.
   - (a) Run the fine-tune from seed 2's `model_7999.pt` (about 1.5 h) and, if
     it meets gate 2, restate gate 4 for the two-stage recipe as "two seeds, at
     most 8 h each".
   - (b) Keep the 6 h cap and fold the fine-tune into one from-scratch run.
   - Recommended: (a). It is the cheapest way to show the recipe is not one
     lucky seed. (b) is a full retrain with no gain in the policy.
5. **Roll-overs.** The get-up study found a roll-over cannot be undone with
   sagittal legs, and one case where limp legs rolled back to a sit.
   - (a) Test an immediate torque cut on a roll fault (supervisor change, in
     simulation) on a set of sideways tips. Adopt it in the firmware if it turns
     roll-overs into sits without making other falls worse.
   - (b) Leave roll-overs to the release notes ("needs a hand").
   - Recommended: (a). It is a small firmware change and, if it works, makes
     the get-up cover most falls.

## Exit gates

| Gate | Pass mark | Result (6 November 2026) |
|---|---|---|
| 2. Recovery, kept | at least 83/98 recovered, belt at least 6/8, pushes 30/30, 0 physical failures, for any policy that replaces fine-tune 1 | kept: fine-tune 1 stays (89/98, 8/8, 30/30, 0); fine-tune 3 (75/98, 20/30) not adopted |
| 3. Speed suite | at least 40/43 under the signed-off policy rule | missed: 35/43 (fine-tune 3 31/43) |
| 4. Reproducible | the deployed recipe meets gate 2 on two seeds (decision 4) | undecided: seed 2's fine-tune is 83/98, 7/8, 29/30 (push_low_-2, a margin verdict); speed 25/43, stress 24/26 |
| 5. Stress margin | training-plant stress at most 5 % (5 x 1000 seeded episodes); full-plant stress at least 25/26 | as M14: 5.9 %, 25/26 (no new policy adopted) |
| 6. Get-up | at least 80 % of randomised full-plant trials reach the handover and balance for 5 s, from sit, front lean and back lean; 0 structural failures on the M10 falls screen; plus 3 moves passing as speed cases | partly met: get-up 52/60 (sit 20/20, front 16/20, back 16/20) with the physical-limit checks off before the handover; moves met (3/3 nominal, 21/21 variants); falls screen not run |
| 7. Following | at least 58/60 nominal runs with the policy, rendered link at 5 ms with the 74° gate; no group worse than the classical record (101/112) without a named cause nominal met with the policy follower profile (1.1 m/s^2): 60/60, but 1.1 was picked on these same 60 runs; the other 52 runs (robustness groups) not run |

## Phases

**A. Seed 2 fine-tune** (laptop GPU, about 1.5 h; then cloud CPU).
- `python -m bipe train --task finetune --resume-run <seed 2 run>` on
  `model_7999.pt`, under the `systemd-run --user` unit as in M14.
- Score it with `bipe.m14.evaluate run` (all stages). Gate 4 is met if it meets
  gate 2.
- It can run as soon as you have the laptop on; nothing else depends on it.

**B. The policy under the M12 follower** (cloud CPU, no training).
- Add the following runners (`bipe.m12.following`, then
  `bipe.m12.following_rendered`) to `bipe.m13.policy_control.policy_in_loop`, so
  the C follower's speed and yaw-rate commands drive the policy, through the same
  handover code as the firmware.
- Run fine-tune 1 on the 112-run matrix: the phase D measurement model first (fast),
  then the rendered link at 5 ms with the 74° gate.
- This sets a baseline for gate 7 before any new training, and shows whether the
  path-replay misses in the speed suite also break following.

**C. Speed and stress fine-tune** (laptop GPU, at most 2 runs of about 2 h).
Fine-tune 1 fails eight speed cases:

| Case | Fails on | Likely fix in training |
|---|---|---|
| `slope_down_4`, `slope_down_8`, `slope_down_10` | axle depth p95; 8 and 10 also fail to hold (settle held them) | slope-along-heading commands with stand-still on downhill, axle-depth term kept on slopes |
| `replay_corner_90`, `replay_curve_r1` | path | yaw-rate tracking at speed: replay-shaped command sequences in the resampler |
| `replay_straight_1.0` | peak pitch | pitch-rate penalty at the top speed |
| `drive_step` | stop | step commands to zero in the resampler |
| `push_fast_-2` | axle depth p95 | pushes while driving fast |

- New variant `locomotion_ft3` from fine-tune 1, with these command families and
  the training-stress fall causes from M14 (steep slopes, and pushes on slopes
  under 8°; `artifacts/m14/fall_draws`).
- Judge with the M14 evaluator plus phase B's following run. A run is kept only
  if it holds gate 2 and full-plant stress at least 25/26.
- Expected gain: five of the eight cases are needed for 40/43. Downhill holding
  is the most likely to resist, since fine-tune 1 lost it against settle.

**D. Roll-fault torque cut** (cloud CPU, firmware).
- Sideways tips on the full plant: 3 speeds, level and 4° cross-slope, both sides,
  legs held (today) against an immediate torque cut on the roll fault.
- Also rerun the forward and backward falls and the M10 falls screen, so the
  change does not make them worse.
- If adopted: change `bipe_fall_step` in the firmware with Python parity, rerun
  the acceptance and the handover cases.

**E. Get-up policy** (laptop GPU, one run of at most 6 h; then cloud CPU).
- Task `getup` in `bipe.m13.rl`: episodes start in fallen states drawn from the
  study's three (sit, front lean, back lean), plus falls collected from the
  M14 stress draws and phase D (falls while driving and on slopes).
- Same plant as M14 (belts, actuator and pack randomisation). Reward reaching the
  handover test, with penalties on hub and hip current over 80 % of their limits
  and on shell impacts.
- Supervisor: a fallen state switches to the get-up policy; at the handover it
  switches to the locomotion policy; a timeout of 5 s cuts torque. Python model
  first, firmware after.
- Full-plant evaluation: 60 trials (20 per state, randomised plant and start),
  the 80 % mark, then the M10 falls screen on every trial.
- Export, parity and the forward-pass budget as in M14 (two networks now; the
  budget has to fit both, only one runs at a time).
- If it misses 80 %: one curriculum iteration, then the documented negative
  that gate 6 allows.

**F. Three moves** (cloud CPU).
- Command sequences in `bipe.m15.moves`: crouch bob (posture labels, 1 Hz),
  spin with a stop (yaw rate to the envelope limit), wiggle (alternating yaw
  rate at standstill).
- Judged as speed cases on the full plant under the policy rule, with the
  robustness variants.

**G. Learned-policy demo shots** (cloud CPU, `bipe/demo`).
- The demo pipeline (`bipe/demo`, PR #11) is on `main`.
- Shots with the M15 policy: learned against classical split screen (pushes),
  following with the head-camera inset, a get-up from each state, the three moves.
- A learned cut alongside the classical one (about 90 s). Private, as before.

**H. Exit.**
- Choose the M15 policies (locomotion and get-up), export, regenerate the
  firmware headers, write up the gates.
- The classical controller stays the default at power-up until the bench check.

## Order and timing

- A, B and D run in parallel in the first days (A on the laptop, B and D in the
  cloud).
- C after B, so the fine-tune also targets any following failure.
- E after D, so the get-up trains on the fall states the supervisor actually
  leaves.
- F and G at the end, on the chosen policy.
- Laptop GPU: about 1.5 h (A), up to 4 h (C), up to 6 h (E), scheduled around
  your laptop hours.
- Estimate: about 2 to 3 weeks.

## Progress (4 November 2026)

Built (cloud):
- B: `python -m bipe.m15.following --checkpoint CKPT [--link model|rendered]` runs the
  M12 closed loop with the policy in place of the C core (the follower's commands drive
  it). Fine-tune 1 passes `straight_0.3` seed 0 on the measurement model (503 s of CPU
  for 31 s). The rendered nominal matrix is running, seed 0 of each scenario first; a
  rendered run takes 30 to 50 min of CPU here (classical record: 31 min mean), so the 60
  nominal runs take most of a day on the cloud's 4 cores.
- C: task `locomotion_ft3` (`python -m bipe train --task finetune3 --resume-run RUN`),
  aimed at fine-tune 1's eight speed misses. Three are near misses: `drive_step` stop
  excess 0.1502 m (limit 0.15), `replay_corner_90` path 0.201 m (0.2),
  `replay_straight_1.0` peak pitch 15.1 degrees (15). The downhill cases sit 5.8 to
  6.7 mm off the commanded axle depth (5 mm rule) and creep 0.09 and 0.21 m on the 8 and
  10 degree holds; `replay_curve_r1` is off by 0.60 m. New terms: a 5 mm axle-depth
  kernel (the posture kernel is 15 mm wide), the stand-still kernel scaled by the
  slope, a body pitch penalty past 13 degrees, and 15 % of commands at combined high
  speed and yaw rate. Smoke-tested on CPU (8 worlds); training needs the laptop.
- D: `python -m bipe.m15.roll_cut` compares the configured protective fall (crouch
  0.25 s, ramp 0.08 s, then torque off) with an immediate cut and with the get-up
  study's legs held for 1 s, over 64 sideways tips (offsets past the tyre-edge tipping
  angle, roll rates, speeds, cross slopes). Running. A check run found an upright start
  with a roll rate does not tip even at 4 rad/s (the track is wide), so the tips start
  at the tipping angle as in the study; one held tip rolled through 360 degrees and
  landed on its tyres, so rest states are classed by the chassis's up axis and runs
  that went over the shell are flagged. Note: the M10 plant's simulations do not load
  the protective fall (it is in `m9_physics`), and the Teensy application does not yet
  call `bipe_fall_step`; both are hardware-build wiring items.
- E: `python -m bipe.m15.fallen` made 400 fallen states in the training plant
  (`artifacts/m15/fallen_states.npz`: 202 sits, 102 front and 96 back leans, none
  rolled; many leans end near 90 degrees, flat on the face or back, harder than the
  study's 48 degree leans). Task `getup` (`python -m bipe train --task getup
  --resume-run <fine-tune 1 run>`): fallen starts, stand-still commands, an upright
  kernel and the handover reward, roll-over and joint-limit terminations, 6 s episodes.
  `python -m bipe.m15.getup_eval --getup CKPT` is the gate-6 harness on the full plant
  (checked on one trial with the locomotion policy standing in: it does not get up).
  Finding: the C estimator refuses fallen states (legs outside the range its kinematics
  accept, and a pitch within 6 degrees of vertical), so the firmware needs a fallen mode
  (raw channels, the estimator re-initialised until the state is valid). The harness
  models that mode and switches policies on the true handover state; both are
  recorded assumptions for the bench.
- F: `python -m bipe.m15.moves` (crouch bob, spin and stop, wiggle; nominal and seven
  variants), queued for when B and D free the CPU.

Laptop (4 November 2026, evening):
- A: the seed 2 fine-tune finished (10 000 iterations, reward 66.95, no NaN
  terminations). Its evaluation was stopped at Pranay's request with the hardware and
  stress stages done; acceptance resumes with the same command, speed not yet run.
- C: fine-tune 3 finished (`2026-11-04_20-15-14_finetune3_s1`, last checkpoint
  `model_11997.pt`, reward 88.8 at the end, 94.6 at its peak, no NaN terminations).
  Not yet evaluated.
- F: all three moves pass on fine-tune 1: 3/3 nominal, 21/21 variants, no falls, peak
  pitch 1.4 to 2.8 degrees (the crouch bob's axle-depth check is reported, not judged,
  since the posture changes on purpose).
- B: the rendered matrix with the policy in the loop takes over 3 h per run on the
  shared laptop CPU; stopped with no run finished. To be rerun as the 60 nominal runs
  (`--group nominal`) on a free CPU.
- Get-up training, fine-tune 3 evaluation and nominal following ran next (below).

Results (5 November 2026, laptop):

| Gate | Needs | Fine-tune 1 | Seed 2 fine-tune | Fine-tune 3 |
|---|---|---|---|---|
| 2 recovered | >= 83/98 | 89/98 | 83/98 | 75/98 |
| 2 belt | >= 6/8 | 8/8 | 7/8 | 8/8 |
| 2 pushes | 30/30 | 30/30 | 29/30 | 20/30 (all ten low pushes fall) |
| 3 speed rule | >= 40/43 | 35/43 | 25/43 | 31/43 |
| 5 full-plant stress | >= 25/26 | 25/26 | 24/26 | 25/26 |

- C: fine-tune 3 fixes two of fine-tune 1's speed misses (slope_down_4 and
  push_fast_-2) but loses the low pushes (11 physical failures: the ten low pushes and
  low_posture_drive_ramp), so fine-tune 1 stays the policy. Its training-plant stress
  (gate 5's 5 % share) was not run. Its remaining speed misses are overshoot (arc_r2,
  drive_ramp, drive_step, reverse, weave), the three replays, the 8 and 10 degree
  downhill holds and two wheel-lag cases.
- A / gate 4: seed 2's fine-tune misses speed (25/43) and full-plant stress (24/26);
  its only gate-2 miss is push_low_-2, a verdict at the band margin (35 margin flags).
  Gate 4 is undecided until the margin verdicts are rerun with perturbations.
- E: get-up (`2026-11-05_16-29-13_getup_s1`, `model_12997.pt`, 3000 iterations from
  fine-tune 1) gets up from 43/60 starts on the full plant (72 %, gate 6 needs 80 %):
  sit 18/20, front lean 9/20, back lean 16/20 (front and back corrected, see round
  2). Failures: 8 front leans trip the
  supervisor's attitude fault just after the handover, 7 leans never reach the
  handover (the estimator refuses 534 to 1757 ticks in each), 2 sits tip past 20
  degrees under the locomotion policy after the handover. The median handover comes at
  0.22 s, so the handover test passes almost at once from many starts and needs a
  check against the study. A first run scored 0/60 on two harness bugs (the protective
  fall ran before the handover and cut torque; the supervisor was not ticked while
  down and tripped its deadline), fixed in 878b6cc.
- B / gate 7: fine-tune 1 under the M12 follower, rendered link, nominal: 34/60
  (classical 60/60). No falls, and every miss meets the follower's expectations: all
  26 fail only the 15 degree peak pitch gate (15.0 to 15.7 degrees, when braking from
  about 1.2 m/s to a stop). 5/5 on approach, loss of view,
  both occlusions, straight 0.3 and 0.6; straight 1.0 2/5; curve r1 and start-stop
  1/5; corner 90, curve r2 and lateral crossing 0/5.

### Phase D result: keep the protective crouch, do not cut at once (4 November 2026)

64 sideways tips on the full plant, three leg strategies after the 12 degree roll fault
(`python -m bipe.m15.roll_cut`, records `artifacts/m15/roll_cut/`). Rest state after 4 s,
by the chassis's up axis:

| Strategy | Sit (on its tyres) | On its side | Rolled over | Went over the shell (incl. landing back on the tyres) |
|---|---|---|---|---|
| Configured protective fall (crouch 0.25 s, ramp 0.08 s, torque off) | 47 | 17 | 0 | 15 |
| Immediate cut | 35 | 20 | 9 | 22 |
| Legs held 1 s, then cut (the get-up study's case) | 17 | 31 | 16 | 50 |

- The immediate cut is worse than the configured crouch: paired tip by tip, it turns 2
  side rests into sits but 8 sits into side rests and 9 tips into roll-overs (6 of them
  from a sit). Most of its roll-overs are at rest (7 of 9) and on the 4 degree cross
  slope with the tip to one side (6 of 9).
- The configured crouch-then-cut leaves no roll-overs in this set; 17 tips end on the
  side (8 of them at the largest offset, 15 of 17 at 0.5 m/s), which the get-up policy
  does not cover.
- The get-up study's roll-over came from holding the legs, which the firmware does not
  do.

Decision 5 is answered by the result: no firmware change to the roll response. Two
wiring items follow for the hardware build: the Teensy application does not call
`bipe_fall_step` yet, and the M10 plant's simulations do not load the protective fall
(it sits under `m9_physics`), so no full-plant run so far includes it.

A, C and E ran on the laptop GPU on 5 November (results above).

## Out of scope

- Hardware: no purchases, no bench work (M13 phase E bench checks stay open).
- Release work (M16): packaging, browser replay, the public video.

## Round 2 (6 November 2026, Pranay's choice: get-up and following only)

- Get-up handover fixed: the eval and the training reward measured the pitch error from
  the equilibrium at the current legs, which any pose with its COM over the axle meets;
  they now use the nominal 40 posture's equilibrium (about 0 degrees) as the study does.
  The handover also needs the C estimator to have accepted 50 ticks in a row and its
  pitch inside 80 % of the supervisor's attitude limit. Trials record the pose at the
  start (not in the committed records, which predate it), at the handover and at the end. The round 1 get-up result (43/60) used the lax test.
- Get-up task `getup2`: fallen starts drawn equally by class with back leans 1.5 times
  as often (by the fallen-state library's labels, which are correct); 3000 iterations from the first get-up checkpoint (laptop).
- Following: the follower's acceleration limit at 1.0 m/s^2 instead of 1.25 passes the
  three all-fail scenarios on the measurement model (peak pitch 11.7 to 13.1 degrees).
  This is a policy-only follower profile (`python -m bipe.m15.following --limit
  acceleration_m_s2=1.0`); config/follower-m12.json and the classical record stay at
  1.25 m/s^2. Adopting it for both needs the classical 60 rerun.
- Get-up results (handover on the nominal posture, 20 starts per class, gate 6 needs
  80 % in every class): the first get-up policy (`getup_s1`, `model_12997.pt`) gets up
  from 52/60 (sit 20, front lean 16, back lean 16), which meets it; `getup2`
  (`2026-11-06_10-24-16_getup2_s1`, `model_15996.pt`) gets up from 54/60 (20, 15, 19)
  and misses on front leans by one: the extra back-lean weight helped back leans
  (16 to 19) and cost front leans (16 to 15). Seven starts (three front, four back
  leans) never
  hand over in any run: the C estimator refuses 534 to 1757 ticks there (the fallen
  mode the firmware needs). Sits hand over at the earliest tick the estimate rule
  allows (0.245 s); leans of 40 to 50 degrees come inside 10 degrees in about 0.25 s,
  which looks fast. Rendered clips of nine `getup_s1` trials
  (`getup_eval --record SEED`; kept out of git in `local/m15/getup_clips/` and the project files) show it is real: a quick
  leg push brings 40 to 70 degree leans up over the wheels in 0.2 to 0.4 s, with the
  wheels on the floor and no shell bounce in the stills. Failures seen: seed 29 hands
  over from flat on its front and tips back past 20 degrees; seed 12 never gets up from
  flat on its back.
- Front and back were swapped in the get-up eval (`getup_eval` took the pitch from the
  wrong matrix entry, so its sign was reversed; the fallen-state library and training
  were right). Fixed in the eval, and the committed records, export case names and the
  numbers above are relabelled; totals and per-trial verdicts do not change.
- Following with the follower limited to 1.0 m/s^2 (rendered link, nominal): 57/60
  (34/60 at 1.25). Peak pitch is 11.8 to 13.9 degrees in every run; the three misses
  (corner_90, curve_r2, straight_1.0, one seed each) dip to 0.594 m, under the 0.6 m
  range band, as the gentler braking stops closer to the target.
- At 1.1 m/s^2: 60/60, every scenario 5/5 (gate 7 met; classical 60/60 at 1.25).
  Peak pitch 12.8 to 14.7 degrees; the lowest range in a band-checked scenario is
  0.629 m (curve_r2). Output on the laptop: local/m15/following_rendered_ft1_a11.
  The 1.1 m/s^2 follower is a policy profile; the classical record stays at 1.25.
- Not done yet: gate 6's falls screen; before the handover the physical-limit checks
  are off (`armed`), which the get-up count does not see.

## Exit (6 November 2026)

Policies:
- Locomotion: fine-tune 1 (`artifacts/m14/checkpoints/finetune_s1/model_9998.pt`), as
  M14 exported it (`artifacts/m14/export/locomotion`); the firmware headers do not
  change.
- Get-up: `getup_s1` (`artifacts/m15/checkpoints/getup_s1/model_12997.pt`, 3000
  iterations of `getup` from fine-tune 1). `getup2` scores higher overall (54/60) but
  misses on front leans (15/20). A later get-up round should weight front leans too,
  and check on a held-out seed set (the 60 starts here were also used to pick it).
- Follower: the policy runs with the follower's acceleration limit at 1.1 m/s^2; the
  classical controller keeps 1.25 m/s^2 (config/follower-m12.json). The classical
  controller stays the default at power-up until the bench check.

Only the `getup_s1` checkpoint is committed; the `getup2`, fine-tune 3 and seed 2
fine-tune checkpoints are in `local/m15/checkpoints/` on the laptop and the project files.

Get-up export (`artifacts/m15/export/getup`; `python -m bipe.m13.export --policy getup`,
`export_check parity|budget --policy getup`):
- `policy.onnx`, `manifest.json`, `SHA256SUMS`; `observations.npz` holds the get-up
  actor's 94 policy steps before the handover on four full-plant trials (sit 3, back
  front leans 15 and 29, back lean 36);
- parity (`parity.json`): torch, ONNX Runtime, host C and the Cortex-M7 image under
  unicorn within 7.0e-7 of the float64 actor on those and 2000 random observations
  (tolerance 1e-4);
- budget (`budget.json`): the same network size as locomotion, 0.28-0.72 ms per call
  at 600 MHz, 3.6 % of the policy period. Its weights are another 193 KB of const
  data: both networks in DTCM take about 386 KB of the 512 KB shared with ITCM, so one
  of them may have to stay in flash (PROGMEM) on the Teensy.
- The firmware image still carries only the locomotion weights
  (`bipe.m13.headers` regenerates them).

Demo shots (kept out of git, in `local/m15/demo/` and the project files): learned against classical under the 15 ms delay
and noise x2 variant with two 1.5 N s pushes (the classical controller falls at 2.3 s,
the policy holds), the three moves, pushes standing and while driving, and get-ups
from a back and a front lean. The following shot with the head-camera inset was not
made.

Carried to the hardware build (firmware, not simulation):
- a fallen mode: the C estimator refuses fallen poses (legs outside its kinematics,
  pitch near vertical), so the firmware has to run the get-up on raw channels and
  re-initialise the estimator, as `bipe.m15.getup_eval` models it; the seven starts
  that never hand over (flat on the face or back) are those refusals;
- the second network and the supervisor switch (fallen state to get-up, handover to
  locomotion, 5 s timeout), with the supervisor re-commissioned at the handover;
- the protective fall: not called by the Teensy application and not loaded in the
  M10 simulations; phase D keeps its configured crouch-then-cut;
- the follower's acceleration limit switches with the controller (1.1 m/s^2 with the
  policy, 1.25 m/s^2 with the classical controller).

Open in simulation: gate 3 (speed), gate 5 (training stress), gate 4 (seed 2's
margin verdicts, rerun with perturbations), gate 6's falls screen and limit checks
before the handover, and the 52 robustness runs of the following matrix with the
policy at 1.1 m/s^2.
