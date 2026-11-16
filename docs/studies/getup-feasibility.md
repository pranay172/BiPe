# Get-up feasibility (29 October 2026)

A study done before M15 (roadmap v2, gate 6). The question: after a fall, can
the six actuators, inside their limits, bring BiPe back onto its wheels? It
decides whether M15 trains a get-up policy.

This is a validated simulation under stated assumptions, on the full M10 plant,
not hardware. Code: `bipe/getup_study.py`. Records: `artifacts/getup_study/`.

## Result

Getting up is feasible from every fallen state where the tyres still touch the
ground. It is not feasible from a roll-over:

| Fallen state | How it arises | Handover reached | Random motions that reach it (first 16) | Peak hip current (A, limit 11.6) |
|---|---|---|---|---|
| Sit | forward or backward tip, torque cut after 1 s | yes, at 0.77 s | 4 of 16 | 7.2 |
| Front lean (48.6°) | forward tip, legs held | yes, at 2.0 s | 3 of 16 | 7.1 |
| Back lean (−47.6°) | backward tip, legs held | yes, at 0.79 s | 1 of 16 | 8.3 |
| Rolled over (roll −179°) | sideways tip past the tyre edge, torque cut after 1 s | no | 0 of 160 (10 search rounds, asymmetric legs) | n/a |

In the three successful cases, the first random population already contained
a handover. The hub currents stayed at or below 2.9 A (limit 3.3 A). From the
rolled-over state, the search score did not improve in 10 rounds: the legs
swing in the sagittal plane and cannot produce the roll torque needed to turn
the body back over.

Further finding: the same sideways tip with no leg torque at all (legs limp
from the release) rolls back to a level sit. Legs held at the posture make the
robot roll over. The firmware's protective fall crouches before it cuts torque
(`bipe_fall_step`). For a roll fault, an immediate torque cut may turn a
roll-over into a sit. This is one case, a hypothesis to test, not a result.

## Method

1. **Fallen states.** The nominal robot is tipped as in the M10 falls
   (`bipe.m10.falls.release`), with the legs held at the 40° posture by a
   joint PD (kp 20 Nm/rad, kd 0.3 Nm s/rad, through the 1.65 Nm command cap).
   The torque is either kept on or cut at 1 s, then the robot settles for
   2.5 s in total. The plant's own contact rows and 0.25 ms step are used, as
   in the acceptance runs, not the falls' explicit Coulomb law.
2. **Motions.** There are four joint-target keyframes 0.6 s apart, blended by
   smoothstep and held for 1 s, plus wheel torque per keyframe (±0.5 Nm). All
   go through `MechanicalDrives`: RO40 and GL40 limits, FOC lag, belts with
   lash, and bearing friction. Targets stay 2° inside the modelled end stops.
   Legs move symmetrically, except in the rolled-over case.
3. **Search.** Cross-entropy, population 16, up to 8 rounds (10 for the
   roll-over), stopping at the first handover. The best motion is replayed in
   a fresh process and saved with its trace.
4. **Handover.** Both tyres on the ground, no shell contact, pitch error (to
   the nominal posture's equilibrium) and roll inside 10°, and pitch rate
   inside 1 rad/s. Open-loop keyframes cannot balance, so success is judged
   at the handover to a balancing controller, not at a held stance.

## Limits

- Open-loop motions are an existence proof, not a get-up controller. A trained
  policy has to reach the handover from varied states and then balance.
- The fallen states come from one tip per direction, at zero speed. Falls
  while driving, and on slopes, may land elsewhere.
- Structural factors during the get-up were not computed: the M10 falls
  screen was not run on these motions. Peak belt torques are in the records.
- "Not feasible" for the roll-over covers this motion family (sagittal legs
  and wheel torque) and 160 attempts. Dynamic leg swings faster than the
  0.6 s keyframes were not tried.

## Implications for M15

- Train get-up from the sit and from the front and back leans as part of the
  locomotion policy (a fallen-state reset distribution in the training plant).
  The gate-6 criterion (at least 80 % of randomised trials) is plausible for
  these states.
- Roll-overs need a design or supervisor answer, not a policy:
  - test an immediate torque cut on a roll fault;
  - or a rounded shell or ear profile that rolls the robot back.
  Until then, the release notes state that a roll-over needs a hand.
