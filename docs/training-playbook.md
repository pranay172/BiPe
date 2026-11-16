# Training playbook

These are lessons from training and judging BiPe's policies in M13 to M16. Each one
cost at least one run to learn. Microduck commits a similar playbook next to its
training code. Read this before you change a reward, a curriculum or an evaluation.

Every claim here comes from a milestone document, which has the numbers:
[M13](milestones/13-learned-locomotion-plan.md),
[M14](milestones/14-release-grade-locomotion-plan.md),
[M15](milestones/15-behaviours-plan.md),
[M16](milestones/16-behaviour-set-plan.md).

## Judge on a plant you did not train on

- The training plant (mjlab, MuJoCo Warp) is fast and simplified. The full M10 plant,
  with its sensor model, C firmware, supervisor and contact rows, is the judge. A result
  measured only on the training plant counts as a training result.
- Keep a sim-to-sim gap check (`bipe.m13.full_plant gap`). A policy that is good on the
  training plant and bad on the full plant is usually exploiting something the
  training plant leaves out. In M13 that was knee compliance, and in M14 it was belt
  lash.
- Train with the timing the robot will run. The M11 image applies the action one 5 ms
  tick late. That 5 ms is enough to start or stop a 4.6 Hz knee limit cycle (M13 belt
  follow-up).

## Rewards and curricula

- A fine-tune forgets whatever its new command mix crowds out. Fine-tune 3 added
  high-speed and slope command families, fixed two speed cases, and lost all ten low
  pushes. Keep most of the parent's command and disturbance mix (M16 uses 70 %) and
  add the new families on top.
- A narrow kernel judges what a wide one hides. The posture kernel was 15 mm wide and
  the speed rule judges 5 mm. Fine-tune 3 added a 5 mm axle-depth kernel for exactly
  that reason.
- Weight the classes you judge. Weighting back leans 1.5× in `getup2` raised back leans
  from 16 to 19 and cost a front lean (16 to 15), so the class that was unweighted
  failed the per-class gate.
- Fallen-state libraries need the hard states as well as the convenient ones. Many tips
  end flat on the face or back, near 90° and well past the study's 48° leans. Those are
  exactly the states the get-up policy cannot lift.

## Evaluation traps we fell into

- **Lax success tests.** The first get-up handover measured the pitch error from the
  equilibrium at the current legs, which any pose with its COM over the axle meets,
  folded legs included. Measure from the nominal posture's equilibrium, and require
  the onboard estimator to accept the state too.
- **Sign conventions.** `getup_eval` read the pitch from the wrong rotation-matrix entry,
  so front and back were swapped in its records. Check a labelled state by eye (render
  a still) before trusting a per-class table.
- **Picking on the scored set.** The 1.1 m/s² follower profile and `getup_s1` were both
  chosen on the same runs that scored them. Hold out a seed set for the final count.
- **Single-run verdicts at a band edge are close to chaotic.** A 1e-6 difference in the
  actions can move a rest-recovery verdict across its 2° / 0.03 m/s band. Rerun margin
  cases with small perturbations (`bipe.m16.margin`, `full_plant --perturb`) and
  decide each one by majority.
- **Harness bugs look like policy failures.** The first get-up evaluation scored 0/60
  because the protective fall ran before the handover and cut torque, and because the
  supervisor was not ticked while the robot was down. Watch one trial before reading
  sixty.
- **Count what the gate counts.** "79 acceptance cases" left out 28, including all
  three of the classical controller's falls. Report the whole suite and say why
  anything is excluded.

## Running the training

- `python -m bipe train --task <behaviour> --seed N` (see
  [config/behaviours.json](../config/behaviours.json)). One run per seed. Keep the
  recipe two-stage only if it is reproduced on a second seed.
- mjlab exits with return code 0 when it crashes, so read the log, not the exit code.
- With belts on, physics NaNs happen. The `nan` termination ends those episodes. If its
  share grows, trace the cause.
- A stiff belt corner (713 N m/rad) is unstable with an explicit 1 ms spring. The
  rotor armature floor (`plant.rotor_armature_floor`) keeps ω·dt ≤ 1.6 and is zero at
  nominal.
- Policy-in-loop simulations need the fork start method. Under spawn, the workers
  quietly run the classical controller, and `policy_in_loop` refuses spawn for this
  reason.
- On the laptop, wrap long runs in `systemd-run --user` with `systemd-inhibit`. Keep every
  run file inside the repository (`local/`).

## Deployment checks before calling a policy done

- Export with `python -m bipe.m13.export`. Then check parity across torch, ONNX Runtime,
  host C and the Cortex-M7 image (tolerance 1e-4; ours land near 1e-6), and check the
  forward-pass budget at 600 MHz. Run `python -m bipe.m16.catalogue check` against the
  policy contract.
- Run the closed loop with the C forward pass (`full_plant --forward c`). Float32 flips
  happen only at band-edge verdicts. That is expected, but count them.
