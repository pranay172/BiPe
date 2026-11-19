# M18: a simulation closer to the real robot, then speed tracking

Status: closed 18 November 2026 (plan of 16 November; results, decision and review
fixes below).
Evidence when done: a validated simulation under stated assumptions (full M10 plant,
sensor model, C firmware core), not hardware.

## Goal

Two parts, in this order (Pranay, 16 November):

1. **Fidelity.** Model what the simulation still leaves out about the real motors,
   sensors and timing, widen the randomisation to cover it, retrain the policies under
   it, and show on the existing suites that nothing gets worse. The point is that the
   policies need less fixing once the hardware exists.
2. **Speed.** A targeted retrain for the speed suite (35/43 with fine-tune 1, mark 40),
   on top of part 1's policy.

## What the simulation already models

M10 to M14 already cover a lot, so M18 adds only what is missing. Already in the
training plant and the full plant:

- RO40 and GL40 drives: torque-speed and drive-voltage limits, FOC current lag and slew,
  the catalog torque-current curve, torque constant per motor (0.8 to 1.2), FOC lag
  (1 to 2x), Coulomb friction and viscous damping per joint (0.5 to 1.5x), and
  load-dependent belt friction.
- Belts: stiffness (log-uniform over the M10 acceptance range) and lash (hip 0.11 to
  0.45°, knee 0.23 to 0.9°).
- Power: pack voltage 19.8 to 25.2 V with sag (1 to 2x the harness resistance), which
  lowers the GL40 voltage limit and the RO40 speed.
- Sensors: IMU white noise (0.5 to 2x), a fixed gyro and accelerometer bias per
  episode, anti-alias filters, encoder quantisation, the AS5047P periodic error,
  encoder offsets up to ±0.5°.
- Timing: transport delay 5 to 20 ms and encoder lag 0 to 5 ms, each fixed for an
  episode; hub speed-loop lag; the action applied one 5 ms tick late.
- Body: mass and inertia 0.85 to 1.15x, chassis COM ±5 mm, tyre friction 0.5 to 1.0.

## What M18 adds

Each item gets one shared parameter set (`config/fidelity-m18.json`), used by both the
training environment and the full plant, so the judge sees the same robot the policy
trained on. Every value names its source (data sheet, moteus documentation, or a
stated assumption the hardware build will measure).

**Actuators**

- A1. Cogging torque on the RO40 and GL40: a ripple periodic in rotor angle at the
  motors' slot/pole frequency, amplitude 0 to 3 % of rated torque (assumption until
  the bench measures it). This matters most at the slow wheel speeds of standing still.
- A2. Stiction: breakaway friction 1 to 1.5x the Coulomb level on every joint, so the
  hip and knee stick and then slip, as real belt drives do.
- A3. Winding temperature: an initial temperature per episode (20 to 80 °C) and a
  first-order heating model from copper loss. Hot copper raises resistance (lower GL40
  voltage limit) and hot magnets lower the torque constant. The moteus-c1 thermal
  current derating is included.
- A4. Wheels: tyre radius ±3 % per wheel (wear and a left/right mismatch, which makes
  the robot drift in yaw), and rolling resistance (left out since M13).

**Sensors**

- S1. IMU mounting error: the board sits up to ±2° off level in pitch and roll. On a
  balancing robot this shifts the felt "upright" directly, so it is likely the largest
  single gap.
- S2. IMU drift and scale: gyro bias random walk within an episode, accelerometer
  scale error ±2 %, small cross-axis coupling.
- S3. Encoder zero offsets widened from ±0.5° to ±1.5° (calibration after assembly
  against the end stops).

**Timing**

- T1. Latency that varies tick to tick within an episode instead of a fixed delay,
  plus occasional late or lost CAN replies (0 to 2 % of ticks), where the firmware
  reuses the last reading.
- T2. Lost commands: the moteus keeps the previous command for that tick.

**Environment**

- E1. Floor: friction widened to 0.3 to 1.2 (smooth tiles to rubber mats), and a soft
  floor setting (carpet-like contact).
- E2. Payload: 0 to 100 g on the head, battery position ±10 mm.

Rough terrain (slopes past 10°, steps, bumps) stays a separate milestone.

## Phases

**A. Models (cloud).** Add A1 to E2 to the training environment and the full plant,
off by default, so every earlier result still reproduces exactly. Each model gets a
unit test, and the full plant's existing suites must give the same verdicts with
everything off.

**B. Sensitivity study (cloud, no training).** Run fine-tune 1, the get-up, sit/stand
and the classical controller on the full plant with each new model alone at its
widest setting, then all together. This is a new **fidelity suite** of about 30 cases.
It shows which models actually hurt today's policies and decides how much retraining
is needed. If a model costs nothing, it stays in the randomisation but needs no
special attention.

**C. Fidelity fine-tune (laptop GPU).** A new variant `locomotion_fid`: fine-tune 1's
task and command mix unchanged, with all the new randomisation on, fine-tuned from
fine-tune 1 (about 2,000 iterations, roughly 1.5 to 2 h per seed). Two seeds, as the
playbook asks. The get-up and sit/stand policies are fine-tuned the same way only if
phase B shows them losing cases under the new models.

**D. Evaluation (cloud).** The full M14 evaluator (acceptance, belt, pushes, stress,
speed), the get-up gate on the fresh set, sit/stand, moves, margin reruns for
band-edge verdicts, export parity and the Teensy timing budget, and the fidelity
suite.

**E. Speed retrain (laptop GPU).** M15 and M16 showed that fine-tunes aimed at speed
forget pushes, because the new commands crowd out the old ones. The new idea here is
an **anchored fine-tune**: the PPO loss gets an extra term that keeps the policy's
actions close to the phase C policy on the old command mix (pushes, rest, recovery),
while new command families target the eight misses (the stop after `drive_step`, the
4, 8 and 10 % downhill holds, `push_fast_-2`'s axle depth, and the curve, corner and
straight replay paths). Two seeds, about 2 h each. Then phase D's evaluation again.

**F. Hardware hooks (cloud, small).** One override file for measured values
(`config/hardware-measured.json`) that the simulation reads in place of the data-sheet
values, and the bench tests in the build plan that fill it (step response, friction
sweep, latency, IMU still capture). This is what closes the gap once parts arrive.

## Gates

1. **Fidelity suite:** the phase C policy survives at least 95 % of it and does better
   than fine-tune 1.
2. **No regression** against today's numbers: acceptance recovery 89/98 or better
   (band-edge verdicts by majority of three reruns), belt 8/8, pushes 30/30, speed
   35/43 or better, stress falls 5.9 % or lower, get-up 59/60 held out and 40/40 fresh,
   sit/stand 40/40, moves 21/21, export parity under 1e-4, inside the Teensy budget.
3. **Speed:** 40/43 or better with gate 2 still met.

If gate 3 fails, the phase C policy still ships as the new default and the speed limit
stays documented, with what the anchored fine-tune changed.

## Compute and time

- Phases A, B, D and F run in the cloud (CPU). Phase A is about a day of work; one full
  evaluation is a few hours.
- Phases C and E need Pranay's laptop GPU, about 3 to 4 h each for two seeds, launched
  with `systemd-run --user` so they survive a disconnected session. All run files stay
  under `local/` in the repository.

## Decisions for Pranay

1. **Approve the plan as written.** Recommended. Phase A and B start now either way,
   since they change nothing by default.
2. **Retraining scope.** Recommended: drive policy always; get-up and sit/stand only if
   phase B shows them losing cases.
3. **Speed approach.** Recommended: the anchored fine-tune. Alternatives: a full
   retrain from scratch with the speed families in the mix from the start (about 6 h
   per seed, 2 seeds), or accepting 35/43 again.

## Results (16 and 17 November)

**A. Models.** In both plants, off by default (`bipe/m18/fidelity.py`, the
`drive-fidelity` and `getup-fidelity` tasks). The latency-jitter range was cut from
0 to 10 ms to 0 to 1 ms: the Teensy polls every moteus each 5 ms tick, so a late reply
is a lost one, and the C estimator differentiates whatever encoder packet it holds.
With whole-tick jitter fine-tune 1 fell within a second on the full plant. The
hardware build must keep the loop synchronous.

**B. Fidelity suite** (15 models alone at their wide ends plus 8 combined draws, each
under hold, push forward, push back and drive; `artifacts/m18/fidelity_*`):

| Controller | Survived | Recovered |
|---|---|---|
| Classical | 88/92 | 74 |
| Fine-tune 1 | 92/92 | 91 |
| Fidelity fine-tune, seed 1 | 92/92 | 90 |
| Fidelity fine-tune, seed 2 | 92/92 | 92 |

(All four with the case-name seeds of the review fixes below, MuJoCo 3.14. The first
runs, with seeds from each case's position, gave 79, 90, 90 and 91 recovered.)

Get-up (getup_s1) on the fresh set with a fidelity robot per trial: 35/40, four of the
five misses flat on the back.

**C/D. Drive fine-tunes from fine-tune 1** (2000 iterations each, laptop GPU; M14
evaluator, laptop CPU; training stress 1000 x 10 s, seed 11):

| | FT1 | fid s1 | fid s2 | anchored, weight 1 | anchored, weight 5 |
|---|---|---|---|---|---|
| Acceptance recovered (>= 89) | 89/98 | 86 | 85 | 83 | 83 |
| Belt | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| Pushes | 30/30 | 30/30 | 30/30 | 25/30 | 30/30 |
| Speed policy rule | 35/43 | 33 | 30 | 31 | 20 |
| Full-plant stress | 25/26 | 26/26 | 24/26 | 25/26 | 25/26 |
| Training stress falls | 6.2 % | - | - | 5.0 % | 5.5 % |

The plain fidelity fine-tunes hold position less tightly: on the 60 s sensor-bias
holds they travel 0.14 to 0.27 m against fine-tune 1's 0.04 m (limit 0.1 m), which
costs the sensor holds and most of the speed slips (slope holds, replay paths). The
anchored fine-tune (`bipe/m18/anchored.py`) kept the mean action within an MSE of
about 5e-4 of fine-tune 1 on the sampled observations, yet its closed-loop behaviour
moved further than the plain fine-tune's: weight 1 lost the five low forward pushes,
weight 5 overshoots most speed commands. A per-action anchor does not hold the
closed-loop behaviour, so phase E's plan (speed via the anchored fine-tune) lost its
basis. Since fine-tune 1 already passes the fidelity suite, it stays the drive policy.

**Get-up fidelity fine-tune** (getup_s1 + 2000 iterations under the fidelity models;
`artifacts/m18/checkpoints/getup_fid_s1`, results `artifacts/m18/getup_fid_*`):
held-out 60/60 (getup_s1 59/60), fresh 40/40 (40/40), fresh with a fidelity robot
39/40 (35/40). It is the candidate get-up policy.

**Bug found:** the M18 change to `DriveAction.apply_actions` indexed the per-world
friction as a vector when actuator randomisation was on without fidelity, which
crashed the training-stress stage; fixed in a868166.

## Decision and phase F (17 November)

Pranay's decision after phase D: fine-tune 1 stays the drive policy, the getup-fidelity
fine-tune becomes the default get-up (`artifacts/m18/export/getup`: parity within 1e-4
on every path, 0.28 to 0.71 ms per call on the Teensy model, the same as getup_s1;
`config/behaviours.json`, `bipe sim`), phase E (the anchored speed fine-tune) is
dropped and speed stays at 35/43, and phase F is built:

- `config/hardware-measured.json`: one entry per measurable value, all null; nothing
  reads it unless a run passes `--measured`.
- `bipe/m18/measured.py`: plant values (mass, COM, torque constants, friction) replace
  their parameters, sensor values (bias, noise, delay) their configuration entries,
  and the rest go through `fidelity.apply` as a fixed draw. `bipe.m18.suite --measured`
  and `bipe.m15.getup_eval --measured` run on the measured robot (with `--fidelity`,
  the measured values stay fixed and the draw varies the rest).
- `bipe/m18/bench.py`: fitters for the ten bench tests in
  [docs/hardware/bench-tests.md](../hardware/bench-tests.md) and `record`/`show`.
  `tests/test_m18_measured.py` checks each fitter on synthetic data with known values.

Gate summary: gate 1 met on survival (92/92); on recovery seed 1 is one case worse (90)
and seed 2 one case better (92) than fine-tune 1's 91, so not met for seed 1, and moot
since fine-tune 1 stays. The policies' 90 to 92 spread is within seed noise: reseeding
the suite (review fixes below) flipped 9 of the classical controller's 92 verdicts.
Gate 2 not met by any drive fine-tune, met by the get-up; gate 3 not met
(phase E dropped). Training stress (1000 x 10 s, seed 11) is 6.2 % for fine-tune 1,
consistent with M14's 5.9 % over five seeds.

## Review fixes (18 November, PR #20 review)

**What the models leave out.** Against the plan's list:

- A3 winding temperature: one temperature per episode, with no heating dynamics and no
  moteus thermal derating (a 20 s episode heats the windings by a few degrees, M11).
- A4 tyre radius: an encoder scale only (same ground speed and odometry error as a
  real radius change, force per torque unchanged); rolling resistance is not modelled.
- Timing: the full plant counts delay and jitter in whole 5 ms ticks
  (`bipe.sensing`), so the training range's 0 to 1 ms jitter rounds to none there;
  the suite's timing and combined cases test packet and command loss only. The
  training environment applies jitter in 1 ms steps.
- `command_loss` reaches the policy path only; the classical controller's suite runs
  without it, so its 88/92 is not quite the same robot as the policies'.

**B9 now measures data age.** `bench.loop_timing` took the CAN round trip (about
0.4 ms), which `measured.apply` would have written into `transport_delay_s`, the age
of the data when the control law runs (nominal one 5 ms tick, also the C estimator's
prediction delay): a real B9 run would have set the simulated delay to zero ticks. It
now takes each reading's sample time and the tick that used it; a test checks that a
synchronous 0.4 ms bus keeps the one-tick delay.

**Suite seeds.** Each case's seed (cogging phases, accelerometer cross-axis signs,
loss streams) was its position in the run, so fine-tune 1's partial rerun after the
jitter fix ran some cases on other robots than the other controllers' full runs. Seeds
now come from the case name; fine-tune 1's and the classical suites are rerun with
them, and the two fidelity fine-tunes' suites too (19 November), so all four rows of
phase B's table are on the same 92 robots: classical 88/92 survive, 74 recovered
(was 79); fine-tune 1 92/92, 91 (was 90); seed 1 92/92, 90 (unchanged; misses
`encoders__hold` and `encoders__push_fwd`); seed 2 92/92, 92 (was 91). Fine-tune 1's
one miss is `combined3__push_fwd`. Summaries now record the commit and the MuJoCo
version.

**Synchronous loop.** The requirement is now item 5 of the M13 hardware wiring
checklist: today's `teensy_main.cpp` paces on `millis()` and accepts motor data up to
20 ms old in its stationary check.

**Get-up records rerun** (`artifacts/m18/getup_fid_eval.sh`, full-plant environment
`requirements.lock`, MuJoCo 3.14). getup_s1 on the seed-3000 fidelity set, clean:
35/40, the same five misses. A new fidelity set nothing in M15 to M18 used (seeds
5000 on, 40 lean starts, `fidelity_5000_*`):

| | getup_fid_s1 | getup_s1 |
|---|---|---|
| All | 38/40 | 39/40 |
| Front lean | 19/20 (5047) | 20/20 |
| Back lean | 19/20 (5056) | 19/20 (5056) |

**MuJoCo version.** Full-plant results are judged in `requirements.lock`'s
environment (MuJoCo 3.14, with torch and mjlab added under the 3.14 override,
docs/getting-started.md); `requirements-rl.lock` (MuJoCo 3.11) is for training only.
Every committed M18 record is a 3.14 run: the get-up records rerun from a clean tree
match the committed trials exactly, as do spot reruns of `getup_fid_fresh_fidelity`
(3006, 3023), `getup_fid_heldout` (1026, 1051) and fine-tune 1's `combined3` cases.
`getup_eval` and the fidelity suite now write the version into their summaries (the
suite also the commit and the checkpoint's sha256). The evaluation environment is
locked in `requirements-eval.lock` (docs/getting-started.md). The two fidelity
fine-tunes' checkpoints are committed (`artifacts/m18/checkpoints/fid_s{1,2}`, with
`params/`), and a partial suite run (`--case`, `--model`) now needs `--output`, so it
cannot overwrite a full record.

The PR review ran the same seeds under MuJoCo 3.11 (`requirements-rl.lock`) and
found 39/40 against 34/40; the fallen states and outcomes differ between the two
versions (14 flat starts here, 13 there). So the clean set does not separate the two
policies here. Over both fidelity sets getup_fid_s1 gets 77/80 and getup_s1 74/80,
and 60/60 against 59/60 held out on the nominal robot: no worse anywhere in total,
better where the seed-3000 set motivated it, but the margin is small. It stays the
default.
