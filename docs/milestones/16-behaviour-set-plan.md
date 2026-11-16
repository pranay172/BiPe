# M16: a complete behaviour set in simulation (plan 7 November 2026)

Approved by Pranay on 7 November 2026 with the recommended option for each of the
seven decisions below. As before, nothing is bought,
built or powered, and every run file, checkpoint and log stays inside the repository
(`local/m16/`, gitignored, or `artifacts/m16/` for committed records).

M16 has two jobs:
1. **Close the gaps M15 left open**, so every roadmap v2 gate is either met or a
   documented negative:
   - the speed suite under the policy rule (gate 3, 35/43, needs 40/43);
   - the training-plant stress share (gate 5, 5.9 %, needs at most 5 %);
   - two-seed reproducibility (gate 4, seed 2's margin verdicts not rerun);
   - the get-up falls screen and the physical-limit checks before the handover
     (gate 6), never run;
   - the 52 robustness runs of the following matrix with the policy (gate 7);
   - getting up from lying on the side, which no policy covers yet (17 of 64
     sideways tips end there, M15 phase D).
2. **Organise the work the way Microduck does**, so that when the sim results are
   strong enough the release is a packaging step and not a reorganisation. Public
   release itself comes after M16.

The full M10 plant stays the judge. A result measured only on the training plant is a
training result.

## What we take from Microduck

Pollen's Microduck ([site](https://pollen-robotics.com/microduck/),
[robot repo](https://github.com/pollen-robotics/microduck), training in
`microduck_rl`) is a small legged robot whose public face is a set of trained moves
("7 policies in the box": walk, sit and stand, kick, grab, roller skating, get back
up, spin). What it does that BiPe does not, and what M16 borrows (structure only, none
of its code, meshes or media):

| Microduck | BiPe today | M16 |
|---|---|---|
| One task per behaviour, named `Mjlab-<Behaviour>-<Terrain>-MicroDuck`, with backlash variants | Tasks named by history: `locomotion`, `locomotion_ft`, `finetune3`, `getup2` | Phase H: a behaviour catalogue. Each behaviour is a named task, one recipe, one card |
| Policies share one 61-dimension observation contract and are hot-swapped at runtime | Locomotion and get-up already share the 25-dimension observation and 6 actions, but the contract is only implied by two manifests | Phase H: the observation and action contract written down and versioned; the supervisor switches between any catalogue policies |
| `policy.onnx` plus `manifest.json` (a schema), published per policy | The same pair per export (M13 to M15), plus parity and budget records | Phase H: one manifest schema for every policy, with its gate results and clip. Publishing (Hugging Face Hub) waits for the release |
| One command API: gamepad, phone and `robotctl robot do polite-bow` send the same intents | Speed, yaw rate and posture label, set by scripts per milestone | Phase I: an intent layer (drive, posture, do move, get up, stop) used by the keyboard/gamepad sim, the follower and the firmware command packet |
| A simulator anyone can open (`duck-sim`, a Hugging Face Space) | `python -m bipe.demo record/render` (offline video) | Phase I: `bipe sim`, an interactive CPU sim driven by those intents. The browser version waits for the release |
| Actuator models from Rhoban's BAM, modelled gear backlash, randomised delay and battery sag | Fitted RO40/GL40 drive models, belt lash, actuator and pack randomisation (M14) | Kept. Phase J writes the identification procedure for the bench (H2) in the same shape, so measured parameters drop into the same fields |
| A committed training playbook (`AGENTS.md`) | Lessons spread over six milestone documents | Phase J: `docs/training-playbook.md`, the reward, curriculum and evaluation lessons from M13 to M16 |
| A moves gallery with short clips | Demo cuts, private | Phase I: a behaviour gallery page, each clip next to its gate numbers |

Where BiPe goes further than Microduck, and should keep doing so: every behaviour is
judged on a separate full plant with published pass marks. Microduck's page shows
specs and clips but no success rates.

## Decisions for you

1. **How to attack the speed gap** (35/43, needs 40/43). Fine-tune 3 fixed two cases
   but lost all ten low pushes, because the new command families crowded out the old
   ones. Five of the eight misses are overshoot and stopping (`drive_step`,
   `drive_ramp`, `arc_r2`, `reverse`, `weave` on fine-tune 3, `drive_step` and the
   replays on fine-tune 1), which is the same failure the follower's 1.1 m/s²
   limit fixed in following.
   - (a) First a command shaper (rate and jerk limit on speed and yaw rate in the
     supervisor, the same place the follower limit lives), measured on the speed
     suite with no training. Then one fine-tune ("fine-tune 4") from fine-tune 1 for
     whatever is left, keeping fine-tune 1's command mix at 70 % so the pushes stay.
   - (b) Fine-tune only, as in M15.
   - (c) Accept 35/43 as a documented negative.
   - Recommended: (a). The shaper is cheap, deployable, and already proven on
     following. It needs decision 2.
2. **How the speed suite judges a shaped command.** If the shaper is part of the
   robot, the suite can judge tracking against the shaped command (what the robot
   was asked to do) or the raw one (what the operator asked for).
   - (a) Judge against the shaped command, and report the lag between raw and
     shaped (time to reach 90 % of a step) next to it, with a ceiling of 0.5 s.
   - (b) Keep judging against the raw command.
   - Recommended: (a), stated in the results as a change to the rule. Microduck
     and every commercial robot filter operator input; (b) would push the policy to
     track steps no motor can follow.
3. **Side-lying get-up.** The get-up study found a full roll-over (upside down)
   cannot be undone with legs that move fore and aft. Lying on one side is a
   different state: the two legs can move differently (hip and knee on each side),
   which may lever the body back onto its tyres.
   - (a) A feasibility search first (as the M14 get-up study did: random and CEM leg
     motions on the full plant from the side rests M15 phase D recorded). Train a
     side get-up only if the search finds a way up in at least 4 of 16 starts.
     Otherwise a documented negative, plus a protective-fall change that lands
     fewer tips on the side.
   - (b) Train a side get-up policy straight away.
   - (c) Leave side rests to the release notes ("needs a hand").
   - Recommended: (a). M15's get-up only worked because the study had shown the way
     first.
4. **New behaviours for the catalogue.** Today's set: balance and drive
   (locomotion), get up (sit, front, back), three command moves. Microduck's
   page is built around a handful of distinct trained moves.
   - (a) Add one trained behaviour: **sit down and stand up on command** (park on
     the shell, legs folded, and rise again). It reuses the get-up's sit start,
     gives the robot a resting state with the motors off, and is a clear clip.
   - (b) (a) plus a **hop** (both legs extend to lift the wheels briefly off the
     floor). Spectacular, but a new physical regime (impacts at landing, belt and
     shell loads) that needs its own study.
   - (c) No new behaviours; M16 is gaps only.
   - Recommended: (a). The hop is better as a study at the end of M16 (phase K,
     optional) than as a gate.
5. **The follower profile.** The policy follows at 1.1 m/s², the classical
   controller at 1.25 m/s².
   - (a) Rerun the classical 60 nominal runs at 1.1 m/s²; if it still passes 60/60,
     use 1.1 for both and drop the per-controller switch the hardware build would
     otherwise need.
   - (b) Keep two profiles.
   - Recommended: (a). One rendered matrix on the cloud CPU, and one fewer firmware
     item.
6. **Fallen mode now or at the hardware build.** The C estimator refuses fallen
   poses, so seven get-up starts (flat on the face or back) never hand over. M15
   left the fix to the hardware build.
   - (a) Write the firmware fallen mode now, in C, tested in simulation through the
     host C path (raw channels while down, estimator re-initialised until it accepts
     the state), with Python parity.
   - (b) Leave it to the hardware build, as M15 planned.
   - Recommended: (a). It is software, it decides seven of the sixty get-up trials,
     and the sim is the only place to test it safely. The second network and the
     supervisor switch come with it.
7. **How much of the release packaging goes in M16.**
   - (a) Build the catalogue, manifest schema, intent layer, `bipe sim` and the
     gallery page now, privately (phases H to J). Publishing (Hugging Face Hub,
     a browser sim, community channel, public video) is the release milestone.
   - (b) Gaps only in M16; all packaging at release.
   - Recommended: (a). It is what "like Microduck" asks for, it gives the get-up and
     sit/stand policies one switching path to test, and it shortens the release.

## Exit gates

Gate numbers are roadmap v2's. Every policy that replaces one in use must keep
gate 2 (at least 83/98 recovered, belt at least 6/8, pushes 30/30, 0 physical
failures) and full-plant stress at least 25/26.

| Gate | Pass mark |
|---|---|
| 3. Speed suite | at least 40/43 under the policy rule (decision 2 for shaped commands) |
| 4. Reproducible | the deployed recipe meets gate 2 on two seeds, margin verdicts rerun with perturbations |
| 5. Stress margin | training-plant stress at most 5 % (5 x 1000 seeded episodes); full-plant stress at least 25/26 |
| 6. Get-up | at least 80 % in each of sit, front lean and back lean on a held-out set of 60 full-plant starts, with the physical-limit checks on throughout; 0 structural failures on the M10 falls screen |
| 6b. Side get-up | at least 80 % of side rests (held-out set), or the documented negative from decision 3 with fewer side landings than M15's 17/64 |
| 6c. Sit and stand (if decision 4 a) | at least 95 % of 40 randomised sit-then-stand cycles, 0 structural failures, ends within 5 mm of the commanded posture |
| 7. Following | 60/60 nominal and the 52 robustness runs with no group worse than the classical record (101/112) without a named cause, on the profile from decision 5 |
| 8. Deployment parity | every catalogue policy exported with parity under 1e-4 and its forward pass in budget; all networks fit the Teensy memory plan |
| 9. Front door (if decision 7 a) | from a clean clone on a CPU laptop, `bipe sim` runs a behaviour in under 10 minutes by following the README |

## Phases

**A. Reproducibility and margin reruns** (cloud CPU, no training).
- Rerun the margin verdicts of seed 2's fine-tune with perturbations
  (`bipe.m14.evaluate`), so gate 4 is met or missed on evidence.
- Score fine-tune 3's training-plant stress share, which M15 skipped, to see whether
  the speed families helped gate 5.

**B. Command shaper** (cloud CPU, Python and C).
- Rate and jerk limits on speed and yaw rate in the supervisor, before the policy and
  the classical controller; parameters in `config/`.
- Run fine-tune 1 through the speed suite with it, judged per decision 2, plus gate
  2 and following, to check nothing regresses.
- Sweep two or three settings; pick the gentlest that clears the overshoot cases.

**C. Fine-tune 4** (laptop GPU, at most 2 runs of about 2 h; then cloud CPU).
- From fine-tune 1, aimed at what phase B leaves (expected: the 8 and 10° downhill
  holds and `replay_curve_r1`), plus the training-stress fall causes
  (`artifacts/m14/fall_draws`).
- 70 % of commands and disturbances from fine-tune 1's mix, so the low pushes stay
  in the training distribution (the cause of fine-tune 3's failure).
- Kept only if it holds gate 2 and full-plant stress. Fallback: fine-tune 1 with the
  shaper, gate 3 and 5 recorded as they land.

**D. Following, robustness** (cloud CPU).
- The 52 robustness runs with the policy at 1.1 m/s², rendered link at 5 ms with the
  74° gate.
- Decision 5: the classical 60 nominal at 1.1 m/s².
- The following demo shot with the head-camera inset that M15 did not make.

**E. Fallen mode and get-up 3** (cloud CPU first, then laptop GPU about 3 h).
- Decision 6: firmware fallen mode in C with Python parity; the supervisor switches
  get-up and locomotion networks (fallen state to get-up, handover to locomotion,
  5 s timeout, supervisor re-commissioned at the handover). Rerun `getup_s1` through
  it.
- Turn the physical-limit checks on before the handover (with get-up limits where
  the locomotion ones do not apply) and run the M10 falls screen on every trial.
- Task `getup3` from `getup_s1`: front and back leans both weighted, starts drawn
  from a training seed set, judged on a held-out 60.
- Wire the protective fall into the M10 plant's simulations (it is only in
  `m9_physics` today) so the falls that feed get-up come from the real fall
  response.

**F. Side-lying get-up** (cloud CPU study, then laptop GPU at most 6 h).
- Decision 3: feasibility search from the 17 side rests in
  `artifacts/m15/roll_cut/` plus new ones: random asymmetric leg motions, then CEM,
  with hip current limits as in the get-up study. Record what works and the current
  it needs.
- If at least 4 of 16 starts can be righted: task `getup_side` (or side starts added
  to `getup3`, whichever the study suggests), same evaluation shape as gate 6.
- If not: try a protective-fall variant (asymmetric crouch during the tip) that
  lands more tips on the tyres, on the 64-tip set, and document the negative.

**G. Sit and stand** (laptop GPU at most 3 h; then cloud CPU), if decision 4 a.
- Task `sit_stand`: a command bit asks to sit (fold onto the shell, motors to low
  torque) or stand (rise to the commanded posture and hand over to locomotion).
- Shares the observation and action contract; the supervisor switches to it on the
  intent.

**H. Behaviour catalogue** (cloud CPU), if decision 7 a.
- `docs/policy-contract.md`: the 25 observations and 6 actions, units, rates,
  normaliser, as version 1 (what both exported manifests already state).
- A manifest schema (`config/policy-manifest.schema.json`) and a check that every
  export conforms: contract version, recipe (task, seed, iterations, parent
  checkpoint), gate results, parity, budget, clip.
- `python -m bipe train --task <behaviour>` names the behaviour, not the history:
  `drive` (locomotion recipe plus fine-tune), `getup`, `getup_side`, `sit_stand`.
  The old task names stay as aliases so earlier records reproduce.
- `artifacts/policies/<behaviour>/` holds the chosen export of each behaviour;
  the firmware headers are generated from that folder.

**I. Intents, interactive sim and gallery** (cloud CPU), if decision 7 a.
- An intent layer (`bipe/intents.py`, and its C twin in the firmware command path):
  drive (speed, yaw rate), posture, do move (crouch bob, spin, wiggle), sit, stand,
  get up, stop. The follower, the moves and the sim all send intents.
- `python -m bipe sim`: interactive MuJoCo viewer on CPU, keyboard and gamepad,
  ONNX policies only (no torch or mjlab needed), pushes on a key.
- `docs/behaviours.md`: one card per behaviour with a short clip, its gate numbers
  and its limits. Clips rendered with `bipe/demo`; they stay out of git and private
  until the release.

**J. Playbook and bench procedure** (docs).
- `docs/training-playbook.md`: reward terms that worked and failed, curricula,
  domain randomisation ranges, evaluation traps found in M13 to M16 (lax handover
  test, swapped pitch sign, picking on the scored set).
- `docs/hardware/system-id.md`: how the H2 leg rig will identify the RO40 and GL40
  drives and the belts (current steps, friction sweeps, lash), with the parameter
  names the plant and the training randomisation use.

**K. Hop study** (optional, cloud CPU), if decision 4 b or time allows.
- Feasibility only: open-loop leg extension profiles on the full plant, landing
  loads on belts and shell against their limits. No training in M16.

**L. Exit.**
- Choose the M16 policies, export, regenerate the firmware headers, write up the
  gates, refresh the README and results pages.
- The classical controller stays the default at power-up until the bench check.

## Order and timing

- A, B, D and E's fallen mode run first, in parallel on the cloud CPU. F's
  feasibility search starts as soon as the CPU frees up.
- C after B, so the fine-tune only targets what the shaper leaves.
- `getup3` after the fallen mode and the protective-fall wiring, so it trains on the
  falls the firmware actually produces.
- G and the side get-up training after F's study.
- H and I alongside, on the policies as they land; J at the end.
- Laptop GPU: up to 4 h (C), about 3 h (E), up to 6 h (F), up to 3 h (G), scheduled
  around your laptop hours.
- Estimate: about 3 weeks, 4 if the side get-up trains.

## Out of scope

- Hardware: no purchases, no bench work.
- Publishing: Hugging Face Hub upload, a browser sim, the public video, a community
  channel and making the repository public are the release milestone, after your go.
- Naming: the separate renaming work decides the robot's name and the M10 file
  names; this plan does not use the old reference name.

## Exit status (9 November 2026)

| Gate | Result |
|---|---|
| 2. Kept by any replacement | drive stays fine-tune 1 (89/98, belt 8/8, pushes 30/30, no failures); no replacement adopted |
| 3. Speed suite | **missed, accepted** by Pranay as a known limit at 35/43 (fine-tune 3, fine-tune 4 and the shaper did not beat it) |
| 4. Reproducible | **met**: seed 2's recipe 85/98, pushes 30/30, belt 7/8, no falls with margin verdicts rerun three times |
| 5. Stress margin | **not moved**: training-plant stress 5.9 % (not rescored, no policy change); full-plant 25/26 |
| 6. Get-up | **missed on three counts**: (1) front leans at 75 %: getup_s1 with the quiet handover 53/60 held out (sit 20, front 15, back 18); every miss starts flat, and the front-lean misses are flat face down, where the phase F search found no motion either (the named cause covers this count only); (2) the physical-limit checks were still off before the handover (`simulate` skips the safety gate and the pitch, roll, contact and joint-limit checks while unarmed), not turned on as phase E planned; (3) the falls screen was not run. getup3 and getup4 were worse and rejected |
| 6b. Side get-up | **missed**: a feasibility negative only. 15 of 17 "side" rests are flat on the face or back. Flat-back motions exist in 6 of 24 starts (25 %, exactly decision 3's 4-of-16 mark). getup_s1, never trained on flat rests, gets up from 12 of 19 held-out flat starts; the two training rounds made that worse (getup3 4/19, getup4 5/19). The pass mark's second half, a protective-fall change landing fewer of the 64 tips on the side than M15's 17, was not tried |
| 6c. Sit and stand | **met for posture and repeatability; structural screen not run**: 40/40 cycles, final depth error at most 1.34 mm, stand from 20/20 held-out sits. During the sit the limit checks, safety gate and protective fall are off, and the shell lands at up to 0.514 m/s with no structural or falls screen on that load |
| 7. Following | **met**: 60/60 nominal and 48/52 robustness at 1.1 m/s², the one group behind classical with a named cause; classical 60/60 at 1.1. Decision 5 followed through: `config/follower-m12.json` now has 1.1 m/s² for both controllers. The classical robustness 52 (41/52) was run at 1.25 and not rerun at 1.1 |
| 8. Deployment parity | **partly met**: parity and budget met (drive, getup, sit_stand under 1e-6, 0.27 to 0.71 ms, assuming zero-wait DTCM); the memory plan is not: three networks do not all fit DTCM, and a forward pass with weights in flash (behind the 32 KB cache) was not timed. Both go to the hardware build |
| 9. Front door | **met in the cloud check**: from a clean clone of the branch, following the README, install plus `bipe sim --headless` took 56 s on a 4-core cloud CPU (clone 25 s, install 19 s, tour 12 s), and the tour passed all eight checks. The window and keys were checked under a virtual display; a run on a real laptop screen is still to do. A 563 MB clone on a slower link adds minutes but stays well inside 10 |

`bipe sim` (phase I, `bipe/sim.py`): the catalogue's three ONNX policies on the M14
training plant (spring hip belts, knee dead band), with exact sensors delayed 5 ms, the
contract's 20 ms policy step, the 1 kHz leg PD and wheel PI loops, and the RO40 and GL40
drive models, in plain CPU MuJoCo with ONNX Runtime (now in `requirements.lock`); no
torch or mjlab. Keys and a gamepad send intents; a stand-in supervisor starts the get-up
after a fall (50° for 0.5 s) and hands a get-up or a stand back to drive after 0.5 s
level and still. It runs about 2.8 times faster than real time on one core. Checks on
it:
- the headless tour (drive 0.6 m/s, turn, stop, a 2 N s push, the crouch bob, sit,
  stand, a 7 N s shove over, get up) passes all eight checks;
- get-up from 5 each of the library's sit, front-lean and back-lean starts: 15/15
  handed over within 8 s; shoves of 7 to 12 N s from either side: 5/5 got up;
- `tests/test_sim.py` (drive, sit, stand, get up from a back lean) in about 8 s.
The full plant stays the judge of every gate; this is the training plant, so its
results are not gate evidence.

Records behind the table, in `artifacts/m16/results/` (small summaries only; traces stay
on the laptop under `local/m16/`, mapped in `SOURCES.tsv`):
- gate 4: `gate4_margin_s2/` (margin.json, three perturbation summaries);
- gate 6: `getup_eval_getup_s1_heldout_quiet/` (53/60, with seed 1021 rerun after the
  library race), `getup_s1_heldout/` (52/60 without the quiet handover),
  `getup3_heldout/` (45/60), `getup4_heldout_quiet/` (46/60);
- gate 6b: `flat_cem_search/` (the five CEM searches), with `artifacts/m16/flat_study/`;
- gate 6c: `sit_eval_sit_stand_40/`, `sit_eval_sit_stand_20/`, `stand_eval_sit_stand/`;
- gate 7: `gate7_following_ft1_a11_robust/` (48/52, classical 41/52 alongside),
  `gate7_following_classical_a11/` (60/60), `gate7_following_classical_a11_dim/` (2/3);
- fine-tune 4's rejection: `ft4/` (scores, speed summary, acceptance metrics, stress
  26/26); its acceptance summary embeds traces (3.9 MB) and stays on the laptop.

Not done in M16 (planned, not on the branch), and where each goes:
- E: the supervisor switching between the get-up, sit_stand and drive networks in C
  (decision 6's "second network and the supervisor switch"); only `bipe_fallen_step`
  exists. Hardware build.
- E: the protective fall wired into the M10 plant's simulations, so the falls that feed
  get-up come from the real fall response. M17.
- E and 6c: limit checks on before the get-up handover, and a falls or structural
  screen for get-up and sit landings. M17.
- 6b: the protective-fall landing change on the 64-tip set. M17.
- H: the manifest schema (`config/policy-manifest.schema.json`) and the
  `artifacts/policies/<behaviour>/` folder the firmware headers come from; today the
  catalogue check compares manifests field by field. Release.
- J: `docs/hardware/system-id.md`, the H2 leg-rig identification procedure. Hardware
  build.
- L: regenerating the firmware headers for the chosen policies (the headers still hold
  the M14 drive network only); and the memory plan for three networks (gate 8). Hardware
  build.
- I: the behaviour gallery page with clips (clips stay private until the release).

Built in M16 besides the policies: the policy contract and catalogue check, the intent
layer, the behaviours page, the training playbook, the C fallen mode, the command
shaper (off by default), `bipe sim`, held-out and quiet-handover get-up evaluation, the sit
evaluation, a classical mode for the following runner, and atomic C library builds for
parallel workers.

## Progress (7 November 2026)

Running:
- D: the 52 following robustness runs with fine-tune 1 at 1.1 m/s² (rendered link,
  5 ms, 74° gate), cloud CPU, `local/m16/following_rendered_ft1_a11_robust`.
- A: `python -m bipe.m16.margin run` reruns seed 2's 35 at-margin acceptance verdicts
  with three seeds of 1e-5 observation noise (`bipe.m13.full_plant --perturb`); each
  margin case is decided by the majority of its reruns. A first check: `push_low_-2`
  (seed 2's only push miss) recovers under perturbation seed 1 (late speed peak
  0.024 m/s against 0.03).
- B: `bipe.m16.shaper` (jerk-limited speed, rate-limited yaw rate, `full_plant
  --shaper`). First setting, a cloud try before the sweep (1.0 m/s², 4 m/s³, 3 rad/s²;
  the sweep's 1.0 setting, `artifacts/m16/shaper/a10_j4_y2`, used 2 rad/s²) on `drive_step`: the stop
  excess drops from 0.150 to 0.079 m and peak pitch from 14.7 to 12.1°, but the
  overshoot rises from 0.08 to 0.17 m/s (limit 0.15), so the setting needs a sweep
  (laptop, four settings on the full speed suite).
- F: `bipe.m16.flat_study` (states, random motions, CEM), laptop.

Built:
- E: the estimator's fallen mode in C (`firmware/include/fallen.h`,
  `firmware/src/fallen.c`, `bipe_fallen_step`), compiled into the simulation core, and
  `bipe.m15.getup_eval --fallen c` to run it in place of the Python model. On get-up
  trial 14 (flat on the face, 1387 refused ticks) the C and Python paths give the same
  run to the last digit.
- B: `bipe.m16.shaped_rule` scores a shaped run against the shaped command (decision
  2). The shaper rerun offline reproduces the in-loop speed reference within 5e-7 m/s.
  With 1.0 m/s², 4 m/s³, 3 rad/s² (the cloud try; added lag 0.30 s): `replay_corner_90` and
  `replay_straight_1.0` pass under the shaped rule (both fail under the raw rule on path,
  0.47 and 0.26 m, from the lag alone); `drive_step` still fails on overshoot.
- H: `config/behaviours.json` (drive, getup, moves), `docs/policy-contract.md`
  (contract v1, hash be4d79c683b4) and `python -m bipe.m16.catalogue check|list`: both
  exports match the contract. `bipe train --task drive|drive-finetune` are the behaviour
  names for the M14 recipe.

Results (7 November 2026, laptop):
- B, shaper sweep (fine-tune 1, full speed suite; `artifacts/m16/shaper/*/shaped_rule.json`;
  fine-tune 1 without a shaper: 35/43):

  | Setting (m/s², m/s³, rad/s²) | Added lag | Raw rule | Shaped rule |
  |---|---|---|---|
  | 1.25, 8, 3 | 0.08 s | 35/43 | 35/43 |
  | 1.25, 4, 3 | 0.16 s | 33/43 | 33/43 |
  | 1.1, 6, 3 | 0.18 s | 32/43 | 32/43 |
  | 1.0, 4, 2 | 0.30 s | 27/43 | 30/43 |

  The mildest setting fixes the three near misses (`drive_step`, `replay_corner_90`,
  `replay_straight_1.0`) but the yaw-rate limit costs `arc_r2` and `weave` (yaw-rate
  error) and `slope_up_10` fails to hold. Slower speed limits make overshoot worse, not
  better: the policy overshoots at the end of a slow ramp. With the yaw limit off
  (1.25 m/s², 8 or 12 m/s³): 35/43 and 34/43; `arc_r2` and `weave` still fail on
  yaw-rate error, so the yaw limit was not their cause. Every setting trades the three
  near misses it fixes for others that sit near their own limits. **Conclusion: the
  shaper does not move gate 3; it stays in the code, off by default.** The downhill
  cases, `push_fast_-2` and `replay_curve_r1` fail in every setting; fine-tune 4 takes
  all eight of fine-tune 1's misses.
- A, gate 4: seed 2's fine-tune with its 35 at-margin acceptance verdicts rerun under
  three perturbation seeds (majority per case): 85/98 recovered, pushes 30/30, belt 7/8,
  no falls in any rerun. Two verdicts flip to recovered (`push_low_-2`,
  `backlash_turn_0`), none flip the other way. With 98/98 survived, the two-stage
  recipe meets gate 2 on a second seed: **gate 4 met** (two seeds, about 7.4 h each).
- F, random motions (16 per start, asymmetric legs, 6 keyframes;
  `artifacts/m16/flat_study/random_summary.json`): 3 of 24 starts reach the handover,
  all on the back (the phase D back lean and two of the flat-on-the-back get-up
  misses); no flat-on-the-face start does.
- F, cross-entropy search (8 iterations on the five closest starts,
  `local/m16/flat_study/search_<start>.json` on the laptop): three more starts reach the
  handover. `getup_12` (2 handovers, iteration 2), `getup_20` (4, iteration 2) and
  `side_t10_s1_r1.5_v0.5_g0` (2, iteration 4) do; `side_t0_s1_r1.5_v0.5_g0` and
  `side_t10_s-1_r1.5_v0_g4` do not (the second's cost was still falling). That is
  **6 of 24 starts with a way up, exactly on decision 3's mark (4 of 16, 25 %)**, so a flat get-up is
  trained. Every start that works lies on its back or rolls onto it; no face-down start
  has a way up, so getup3 adds the flat-on-the-back states and face-down stays a known
  gap unless training finds one.
- E/F, task `getup3` built (`python -m bipe train --task getup3 --resume-run <getup_s1>`):
  the M15 library already holds 107 flat rests (past 70 degrees), so no new library is
  needed. They become two classes of their own; the draw is sit 17 %, front lean 25 %,
  back lean 25 %, flat on the back 25 % and flat on the face 8 %. Episodes run 8 s
  instead of 6 s, since the found motions need up to 4.6 s before the handover. It
  trains on the laptop GPU after fine-tune 4, and is judged on held-out seeds.
- D, following robustness (fine-tune 1, follower acceleration 1.1 m/s², rendered link;
  laptop, `local/m16/following_rendered_ft1_a11_robust`): **48/52, no falls**, against
  the classical controller's 41/52 on the same runs. It gains 8 runs (five at 15 ms delay
  with doubled noise, `start_stop` at 0.20 m lateral, both 4 % slopes) and loses one
  (`corner_90_seed0_lat0.15_dim`). By group (policy / classical): variants 20/15,
  latency 0.10 12/12, latency 0.20 11/10, dim light 2/3, side light 1/1, slopes 2/0. The
  four misses are three `corner_90` runs (0.20 m lateral, dim, side light) and
  `curve_r1` in side light. Gate 7 asks for no group worse than classical without a
  named cause: only dim light is (2/3 against 3/3, one run). Its cause, from the trace
  against the classical record: perception is the same (490 of 493 frames, no loss, the
  same 0.375 s coast after the corner), but the range estimate read short in the coast
  (0.85 m against 0.92 m), the follower asked for a small reverse, the policy overdid it
  (-0.25 m/s against classical's +0.1 m/s), and it then restarted on the 1.1 m/s² ramp
  (classical's record used 1.25) so the range peaked at 1.507 m against the 1.5 m band
  (classical 1.29 m). A classical dim-light run at 1.1 m/s² separates the profile from
  the policy; it runs after the classical nominal 60 (`--classical`, added to
  `bipe.m15.following`).

- G, task `sit_stand` built (`python -m bipe train --task sit_stand --resume-run <FT1 run>`):
  the contract's posture slot carries the request, with the label 100 (command 3.0,
  outside the 30 to 65 drive range) meaning sit, set at once rather than rate limited. 40 %
  of command resamples ask to sit and half the episodes start sitting (the fallen
  library's sit class). The drive rewards count only while standing, and the drive
  failures only from 3 s after a stand request. Sitting earns a seated reward (on the
  shell, within 20 degrees of level, settled), a soft-landing penalty on vertical speed
  and a leg-torque penalty. A 150-step CPU smoke run steps cleanly. It trains on the
  laptop GPU after getup3.

- C, fine-tune 4 (2000 iterations from fine-tune 1, laptop GPU; evaluation
  `local/m16/eval_ft4`, hardware, stress, acceptance and speed stages): **rejected,
  fine-tune 1 stays.** Against fine-tune 1: recovered 85/98 (89), belt 8/8 (8/8), pushes
  30/30 (30/30), no physical failures, full-plant stress 26/26 (25/26), worst steady
  axle p95 5.3 mm (8.0), speed policy rule **29/43** (35/43). It fixes the stress
  near-miss and the 8 and 10 % uphill holds but overshoots more (arc_r1, arc_r2,
  drive_ramp, drive_step, reverse, weave), still misses the four replay paths,
  push_fast_-2's axle depth and the 10 % downhill hold, and loses both 25 ms wheel-lag
  cases. Training reward (74 against 65) did not predict the full-plant result.
- D, classical following at 1.1 m/s² (decision 5): **60/60 nominal, no falls**, the
  same as its 1.25 record, so one profile at 1.1 serves both controllers. The classical
  dim-light run at 1.1 also misses `corner_90_seed0_lat0.15_dim`, on the other side of
  the band (minimum range 0.578 m against the 0.6 m floor; the policy's maximum was
  1.507 m against 1.5 m). The named cause is the short range estimate in the coast after
  the corner plus the gentler 1.1 restart, not the policy: **gate 7 met** (48/52, every
  group at or above classical or with this cause).
- E, held-out get-up gate (states drawn from seed 1000 on until each class has 20, seeds 1000 to 1078, C fallen mode, fine-tune 1 after the
  handover; `local/m16/getup_eval_*_heldout`): **getup3 rejected, getup_s1 stays.**

  | Class | getup3 | getup_s1 |
  |---|---|---|
  | sit | 20/20 | 20/20 |
  | front lean | 8/20 | 15/20 |
  | back lean | 17/20 | 17/20 |
  | all | 45/60 | 52/60 |
  | flat (past 70 degrees) | 4/19 | 11/19 |

  Every failure of either policy starts flat; no lean under 70 degrees fails. The same
  seven flat states never hand over for either policy. getup3 adds seven face-down
  failures: three early handovers (0.6 to 0.8 s) after which the drive policy tips
  forward, and attitude faults. Five getup3 trials and one getup_s1 trial trip the
  supervisor's attitude check 60 to 80 ms after a handover with the true pitch inside
  1 degree: the estimate had not converged after a fast get-up. getup_s1 on the held-out
  set meets 80 % overall (87 %) but not per class (front lean 75 %), so gate 6 is not met
  yet. Next: hand over only when the estimate itself is level and still and the
  accelerometer reads 1 g (`getup_eval --quiet-handover`, the firmware's handover rule
  to be), and in training, reward the handover only once it has held (task `getup4`:
  getup's class mix with flat face-down and flat back starts weighted as the leans, the
  handover reward after 0.5 s held, 8 s episodes). A cloud check of trial 1054 with
  getup_s1: fault 9 at 1.77 s without `--quiet-handover`, passed with it (handover at
  4.6 s). Full held-out rerun with it (getup_s1): sit 20/20, front lean 15/20, back
  lean 18/20, all 53/60, flat 12/19. Trial 1054 now passes; the seven remaining
  failures are the same flat states that never hand over (five face down, two on the
  back). One sit trial first failed the C library's identity check because another
  worker was rebuilding the library in place. Rerun into the same folder after the fix
  (seed 1021 only), it passed; the committed record
  (`artifacts/m16/results/getup_eval_getup_s1_heldout_quiet`) holds the rerun. The C core
  build now reuses a finished library and renames new files into place. Gate 6 still misses on
  front leans (75 %). getup4 trains next.
- E, getup4 (3000 iterations from getup_s1, final reward 54), held-out with the quiet
  handover: sit 20/20, front lean 9/20, back lean 17/20, all 46/60, flat 5/19.
  **Rejected.** Every failure is a flat start that never reaches the handover (no faults,
  no early handovers: the held reward removed those), and it loses seven flat starts
  getup_s1 gets up from. **getup_s1 with the quiet handover stays the get-up policy
  (53/60).** Its only gate 6 shortfall is front leans at 75 %, and all five front misses
  are flat face down, the one state the phase F search found no way up from either:
  the shortfall has a named cause (no known motion from flat face down), not a policy
  regression.
- G, sit_stand trained (2000 iterations from fine-tune 1, final reward 58). Its stand,
  judged as a get-up from 20 held-out sit starts (`getup_eval --classes sit
  --first-seed 1000 --fallen c --quiet-handover`): **20/20**, handover in 0.65 s median
  (0.72 s at most), balancing to the end with either itself or fine-tune 1 after the
  handover. Its sit (`python -m bipe.m16.sit_eval`: stand 3 s, sit 5 s with the
  balancing limits off and leg ranges checked, stand again with the quiet handover):
  5 of 5 distinct trials passed across the five plants. Each sat at about -17 degrees,
  still, with both tyres and the shell down, at a peak downward speed of 0.43 to
  0.51 m/s, no leg past its range, and rose again in 0.49 to 0.58 s. (A first run of 10
  repeated the 5 plants with identical seeds; the runner now shifts the sensor seed each
  cycle.) With distinct seeds (`local/m16/sit_eval_sit_stand_20`): **20/20**, all
  seated, no leg past its range, peak downward speed at most 0.514 m/s, risen again in
  0.49 to 0.58 s. Gate 6c run (40 cycles, each ending with both axles within 5 mm of the
  40 posture's depth; `artifacts/m16/results/sit_eval_sit_stand_40`): **40/40, gate 6c
  met**, final depth error at most 1.34 mm, no leg past its range.
- G, sit_stand exported (`artifacts/m16/export/sit_stand`): parity against the float64
  actor 3.0e-7 to 8.5e-7 across torch, ONNX Runtime, host C and the Cortex-M7 (unicorn),
  tolerance 1e-4; 47 744 MACs, 0.27 to 0.71 ms at 600 MHz (3.5 % of the policy period);
  `bipe.m16.catalogue check` passes for drive, getup and sit_stand (contract
  be4d79c683b4). The recorded parity set covers balancing cases, not the sit motion; the
  random set covers the input space. Three 193 KB networks do not all fit the Teensy's
  512 KB DTCM, so the hardware build keeps one or two there and the rest in flash.
- G, sit_stand training started on the laptop GPU (2000 iterations from fine-tune 1).

- Gate 3 (speed): **accepted by Pranay on 8 November 2026 as a known limit at 35/43**
  with fine-tune 1. Fine-tune 3, fine-tune 4 and the command shaper each failed to beat
  it and each fix cost pushes or recovery. The eight misses (drive_step's stop,
  push_fast_-2's axle depth, the corner, curve and straight replays' paths and the 4,
  8 and 10 % downhill holds) stay listed in the catalogue as the drive policy's limits.

Infrastructure note: the cloud container restarts between sessions of work, and a
rendered following run takes over 1.5 h on its CPU, so the 52 robustness runs never
finished there (0 of 52 after two starts). On the laptop they ran with
`MUJOCO_GL=egl PYOPENGL_PLATFORM=egl` in 2 h 12 min over 14 workers. The first laptop try
crashed because the workers compiled the C follower library at the same time; the C
library builds are now atomic (compile to a temporary file, then rename).

Open question for phase I: the full plant runs about 18 times slower than real time on
a CPU core, so an interactive `bipe sim` cannot run on it. Options are the training
plant (lighter, to be measured) for the interactive sim, with the full plant kept for
recorded clips and every gate.

Corrections to the plan above, found while starting:
- Decision 6's claim that the fallen mode "decides seven of the sixty get-up
  trials" is wrong. The seven trials that never hand over end where they started,
  flat on the face or back (pitch 87 to 90°): the get-up policy cannot lift them,
  and the Python fallen-mode model already feeds it the raw channels. The C fallen
  mode is still worth writing for the deployment, but it will not move gate 6.
- Of the 17 "side" rests in M15 phase D, 15 went over the shell and lie flat on the
  face or back (tyres off the floor), one lies on its side, one is a back lean. So
  phase F's target is mostly the same flat states as the seven get-up misses, and
  one get-up for flat rests would cover both.
- Decision 2's lag ceiling: the training rate limit alone (1.25 m/s²) takes 0.72 s
  to reach 90 % of a 1 m/s step, so "0.5 s to 90 %" cannot hold for any shaper.
  Restated: the shaper may add at most 0.5 s to the time the training rate limit
  takes to reach 90 % of a step.
