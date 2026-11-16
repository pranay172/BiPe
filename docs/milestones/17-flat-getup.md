# M17: get-up from flat on the face or back

Status: result recorded 14 November 2026, revised after the PR #17 review the same day.
Evidence: a validated simulation under stated assumptions (full M10 plant, sensor model,
C firmware core), not hardware.

## Goal

M16 closed with getup_s1 plus the quiet handover at 53/60 on the held-out get-up gate
(sit 20, front lean 15, back lean 18). All seven misses started flat (pitch 87 to 90
degrees): five face down, two on the back. getup3 (45/60) and getup4 (46/60) were
retrained for these states and did worse. M17 asks for the flat starts without losing any
of the 53.

## How the held-out set was used

Both causes below were found by reading the M16 held-out record and rerunning its
failures (1041, 1061). So the held-out 60 is no longer a clean judge of M17: the 59/60
below is a number on the set the fix was diagnosed from, and the same 60 also chose the
quiet handover and getup_s1 over getup3 and getup4 in M16. The rule was then checked on a
tuning set (seeds 2000 to 2129), and the clean check is a **fresh set** of 40 lean starts
(seeds 3000 to 3088) that nothing in M17 was tuned on. The fresh set has since been used to
compare two pre-pose durations, so it is evidence for 0.5 s, not a held-out gate for
whatever comes next.

## Causes found

1. **Leg pose at the start (face down).** Of the 13 held-out face-down starts, the 8 that
   passed all had their legs cut during the fall, so the legs lay limp at their upper
   stops (hip -2.09 rad, knee drive 0.0). The 5 that failed all had their legs still held
   at the riding pose (hip about -2.7, knee drive about -0.6). getup_s1 gets up from the
   first pose and not the second.
2. **Fallen-mode takeover mid-flip (both sides).** The firmware fallen mode
   (`firmware/src/fallen.c`) let the balance estimator take over on the first tick it
   accepted. In a fast get-up that tick can come mid-flip, with the accelerometer reading
   up to 1.6 g, so the estimator started from a wrong attitude and needed about 5 s to
   agree with the true pitch. The robot was up within about 1 s, but the handover (which
   needs the estimate level) never came inside the 5 s deadline. Seen on held-out 1061 and
   on 8 tuning starts.

## Changes

- **Flat pre-pose** (`bipe.m15.getup_eval --flat-prepose front,back`, `--prepose-s`,
  default **0.5 s**): when the estimate reads past 70 degrees on the get-up's first tick,
  both legs go to their upper stops less 2 degrees (`stops_pose`, from
  `getup_study._bounds`) with the wheels held at zero speed, then getup_s1 starts. The same
  pose serves both sides. No retraining; getup_s1 is unchanged. Lean and sit starts never
  see it.
- **Quiet takeover** (`firmware/src/fallen.c`, `firmware/include/fallen.h`): the estimator
  takes over only after `BIPE_FALLEN_TAKEOVER_TICKS` (20, 100 ms) ticks in a row with
  the accelerometer inside the estimator's 1.5 m/s^2 gate of 1 g and every gyro axis under
  `BIPE_FALLEN_TAKEOVER_RATE` (1 rad/s). Until then it is re-initialised from each packet
  as before. One-tick quiet was tried first; a flip passes through quiet instants (tuning
  seed 2012), so the 100 ms run was needed. `bipe.firmware_control.FallenState` gains the
  matching `quiet` counter; `firmware/tests/test_fallen.c` tests the rule. The Python
  fallen model (`--fallen python`) stays M16's rule.
- Every `getup_eval` summary now records its settings (policies, fallen mode, handover
  rule, pre-pose sides and duration, commit).

## Pre-pose duration: 0.25 s against 0.5 s

Tuning started at 0.25 s. The committed held-out gate then ran at the code default of
0.5 s, which the first write-up wrongly reported as 0.25 s (caught in the PR #17 review).
Both durations, all with the 100 ms takeover:

| Set | 0.25 s | 0.5 s |
|---|---|---|
| Held-out 60 | 58/60 (flat 17/19; 1026 and 1051 fail) | **59/60** (flat 18/19; 1051 fails) |
| Fresh 40 (23 flat: 17 face down, 6 back) | 37/40 (3006, 3023, 3029 fail, all flat on the back) | **40/40** |
| Tuning 36 flat | 8/8 of round 1's failures rerun | 35/36 (2067 fails) |

0.5 s is the M17 setting. Face down is solved at either duration; flat on the back is the
side that needs the longer pre-pose. Records: `artifacts/m17/results/` (held-out flat starts at 0.25 s in
`heldout_flat_prepose_025`; the non-flat 41 never see the pre-pose. The 0.25 s fresh-set
figures are from the review's own run, `reviews/pr17-review.md` in the project files; the
rest are committed).

## Tuning set

36 flat starts from seeds 2000 to 2129 (24 face down, 12 on the back;
`artifacts/m17/results/tuning_flat/states.json`), each round labelled in its `variant`
field:
- round 1, one-tick takeover: getup_s1 alone 18/36, with a 0.25 s pre-pose 30/36;
- round 2, the 8 round 1 failures with the 100 ms takeover, 0.25 s: 8/8;
- round 3, all 36 with the final settings (0.5 s, 100 ms): **35/36**. Tuning seed 2067
  (face down, legs held) hands over at 1.50 s and the locomotion policy then tips back past
  the 20 degree limit at 1.66 s: a post-handover fault, not a missed get-up.

## Result: held-out get-up gate (0.5 s)

Same 60 states as M16 (seeds 1000 to 1078, `--fallen c --quiet-handover --flat-prepose
front,back`), record `artifacts/m17/results/getup_eval_s1_prepose_heldout`:

| Class | M16 (getup_s1, quiet handover) | M17 (+ 0.5 s pre-pose, quiet takeover) |
|---|---|---|
| sit | 20/20 | 20/20 |
| front lean | 15/20 | **20/20** |
| back lean | 18/20 | 19/20 |
| all | 53/60 | **59/60** |
| flat (past 70 degrees) | 12/19 | 18/19 |

No trial that passed in M16 fails. Gained: face down 1014, 1041, 1044, 1046, 1048, on the
back 1061. Median handover: sit 0.47 to 0.51 s, front lean 0.91 to 1.52 s (the 0.5 s
pre-pose and the 100 ms quiet run), back lean 0.71 to 0.76 s; all inside the 5 s deadline.

Remaining held-out miss: **1051**, flat on the back with half friction (`friction_0p5`)
and legs cut at the knee-drive lower stop. Without the pre-pose it stays flat (pitch -87
degrees); with it the robot lifts to a -43 degree back lean and stops there without a
fault.

## Fresh set (0.5 s)

`getup_eval --classes front_lean,back_lean --per-class 20 --first-seed 3000 --fallen c
--quiet-handover --flat-prepose front,back --prepose-s 0.5`: **40/40** (front lean 20/20,
back lean 20/20, flat 23/23). Record `artifacts/m17/results/fresh_seed3000_prepose_05`.

Clips (project files `m17/`, out of git like earlier videos, 0.5 s): `getup_1046.mp4` (face
down), `getup_1061.mp4` (back), `flat_getup_face_back.mp4` (both, 10 s).

## Not done / hardware-build todo

- The pre-pose lives in the evaluator's get-up controller, as the quiet handover did in
  M16. On the robot it belongs in the supervisor's get-up mode: on entering get-up with
  the accelerometer past 70 degrees, hold the stops pose for 0.5 s, then start the net.
- The flat test reads one tick's accelerometer pitch; near 70 degrees noise decides it. It
  does not matter while both sides get the same pose.
- The falls screen and the physical-limit checks before the handover are still as M16
  left them (gate 6 counts 2 and 3).
- `bipe sim` (training plant, stand-in supervisor) does not run the pre-pose yet.
