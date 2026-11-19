# Behaviours

BiPe's behaviours, one card each. Every policy shares one observation and action
contract ([policy-contract.md](policy-contract.md)), so the firmware will be able to
switch between them without a reset (the supervisor's switch is a hardware-build item), and every one is commanded through the same small set of intents
(`bipe/intents.py`). The catalogue behind this page is
[config/behaviours.json](../config/behaviours.json); `python -m bipe.m16.catalogue check`
checks every export against the contract.

To try them yourself, `python -m bipe sim` opens all of them in an interactive viewer
on a CPU; the keys are in [getting-started.md](getting-started.md#drive-it-yourself).
The sim runs on the lighter training plant, so its behaviour is a preview, not gate
evidence.

All numbers are from validated simulation on the full M10 plant, with its sensor model,
C firmware, supervisor and contact rows, under the stated assumptions. None of them is
from hardware. Clips are rendered with `bipe/demo`, and stay out of git until the
release.

## Drive

Balances, drives, turns and crouches on speed, yaw-rate and posture commands.

- Intents: `drive(speed, yaw_rate)`, `set_posture(label)`, `stop()`.
- Policy: M14 fine-tune 1 (`artifacts/m14/export/locomotion`), trained with
  `python -m bipe train --task drive`, then `--task drive-finetune`.
- Results:
  - recovers from 89 of 98 acceptance disturbances: 8 of 8 belt cases and 30 of 30
    pushes, with no falls;
  - 25 of 26 full-plant stress cases;
  - follows a person in 60 of 60 nominal runs and 48 of 52 robustness runs at
    1.1 m/s² (the classical controller manages 41 of 52).
- Limits: the speed rule passes 35 of 43 cases, accepted as a known limit in M16. It
  misses drive_step's stop, push_fast_-2's axle depth, the corner, curve and straight
  path replays, and the 4, 8 and 10 % downhill holds. Training stress is 5.9 %.

## Moves

Crouch bob, spin and stop, and a side-to-side wiggle, played as command sequences for
the drive policy.

- Intent: `move(name)`.
- Results: 3 of 3 nominal and 21 of 21 under the robustness variants.

## Get up

Gets back onto the wheels from a sit, a front lean or a back lean, then hands over to
drive.

- Intent: `get_up()`; the supervisor also starts it after a fall.
- Policy: M18 getup_fid_s1 (`artifacts/m18/export/getup`): M15's getup_s1 fine-tuned
  for 2000 iterations under the M18 fidelity models (motor, sensor, timing and floor
  effects drawn per robot).
- Flat starts (M17): when the get-up begins with the robot past 70 degrees (flat on the
  face or back), both legs first go to their upper stops for 0.5 s, then the policy
  starts. The firmware fallen mode hands back to the balance estimator only after 100 ms
  of quiet readings.
- Results on 60 held-out fallen states (C fallen mode, quiet handover, flat pre-pose): sit
  20 of 20, front lean 20 of 20, back lean 19 of 20 (M16 without the pre-pose: 53 of 60).
  On a fresh set of 40 lean starts: 40 of 40, all 23 flat starts included.
  M18's policy: held out 60 of 60, fresh 40 of 40, and 39 of 40 fresh starts on robots
  drawn from the fidelity ranges (getup_s1: 59, 40 and 35), and 38 of 40 on a second,
  unseen fidelity set (getup_s1: 39).
- Handover rule: control passes back to drive only once the onboard estimate itself is
  level and still, and the accelerometer reads 1 g.
- Limits: the supervisor's physical-limit checks are off until the handover, and no
  falls screen has been run. M18's policy misses no held-out start; its three misses on
  fidelity robots all start almost flat (pitch 88 to 90 degrees), two on the back and one
  on the face. The pre-pose runs in the evaluator's get-up controller;
  on the robot it still has to be added to the supervisor's get-up mode, and `bipe sim`
  does not run it yet.

## Sit and stand

Sits down on the shell on request, settles there, and stands again, handing over to
drive. (Training rewards low leg torque while seated; the evaluation does not measure it.)

- Intents: `sit()`, `stand()`.
- Policy: M16 sit_stand_s1. The request rides in the contract's posture slot: label
  100, outside the drive range, means sit (the contract's "Requests" table).
- Results:
  - sits and stands again in 40 of 40 cycles across five plants, settling at about
    −17° with no leg past its range and a peak downward speed of 0.51 m/s, and ends
    within 1.34 mm of the 40 posture's axle depth (the gate allows 5 mm);
  - stands from 20 of 20 held-out sitting starts.
- Limits: no structural or falls screen was run on the shell landing; during the sit
  the supervisor's limit checks and protective fall are off.
