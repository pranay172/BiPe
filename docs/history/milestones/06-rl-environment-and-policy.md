# M6 RL environment and first locomotion policy plan

Started 2 October 2026, immediately before implementation. Requirements below
are proposed checks; measured outcomes belong in `06-results.md`.

## Architecture review and scope

Retain M5's free-base six-actuator serial-leg model for exploratory learning.
It already has finite motor torque/speed/response limits, seven rigid-body
tensors, contact, simulated IMU/encoders, a portable classical controller and
27 locomotion cases plus convergence. The 284 mm width, 2.499519 kg estimated
mass and exact actuator/pack qualification remain engineering holds. Changing
the mechanism now would introduce a new plant before learning is established.
This choice is a training baseline, not approval of the hardware architecture.

Implement a **hybrid residual policy** first: PPO learns bounded six-motor
torque corrections around the existing sensor-based classical controller. The
network does not independently establish locomotion; compare classical-only,
untrained and trained policies explicitly. Retain the classical controller and
its inner leg loop. Hard torque/speed limits and motor lag remain outside the
network. No base constraints or physical state clamps may stabilize training.

Keep `config/robot.json`, accepted controllers, all M0–M5 evidence and the user's
modified M3 assembly unchanged. New environment, training settings and outputs
are separate. No fabrication, flashing, manufacturer messages or hardware
actuation is part of this milestone.

## Dependencies and assumptions

- Project-local learning environment under `.venv/rl`; preserve `.venv` and
  previous exact dependency locks. Pin installed learning distributions.
- Benchmark native MuJoCo CPU execution and an available GPU simulation
  backend, checking contact/actuator/model compatibility before selecting it.
  GPU availability is not proof of training speed or physics equivalence.
- Policy observations derive from IMU/encoder estimates, commands, controller
  references and previous actions. Simulator truth may compute rewards and
  evaluation metrics but must not enter the deployed actor observations.
- Actions are dimensionless bounded residual requests; physical scales are
  derived from the accepted per-motor limits. Only actuator commands may be
  limited. Reset may initialize a free robot; failure ends the episode.
- Sensor noise/delay and the existing motor response are modeled assumptions,
  not measurements. Randomization ranges are declared experiment settings;
  they do not qualify exact hardware or guarantee simulation-to-real transfer.
- Network inference on the actual MCU is later work; export a portable small
  network and measure host inference without claiming a Teensy timing result.

## Implementation order

1. Save source/configuration identities and this plan; commit the planning
   checkpoint before implementation. Inspect GPU memory and install isolated,
   pinned learning dependencies from official documented packages.
2. Implement a reproducible episodic RL environment using the accepted plant,
   sensing, controller and motor dynamics. Define actor observations, six
   bounded residual actions, command curriculum, reward terms and termination.
   Save reset seeds and sampled settings. Provide zero-action and disabled
   controls as meaningful diagnostics.
3. Test physical freedom/contact/finite limits, actor observation independence
   from simulator truth, zero-residual agreement with the reference experiment,
   reproducible reset, termination and timeout semantics. Benchmark native and
   GPU execution; choose the measured, compatible training path.
4. Train PPO on a staged command mix: hold/balance, driving/turning, then crouch
   and combined motion. Save initialization, optimizer settings, random seeds,
   checkpoints, training metrics and exact configuration/source identities.
   Evaluate deterministic actor outputs on held-out scenarios/seeds; do not
   choose the delivered checkpoint from the final acceptance set.
5. Run the full applicable 27-case articulated acceptance plus half-timestep
   convergence with the delivered policy. Compare matched classical-only and
   hybrid metrics, retain failures, test zero versus untrained residuals, and
   check the exported network's inference against its training implementation.
   If results do not establish improvement, report that and keep the existing
   controller as the accepted default.
6. Save measured results, an inspected replay/plots, regeneration commands and
   verification evidence; commit implementation and verification checkpoints.

## Deliverables and measurable checks

| Deliverable | Completion evidence |
| --- | --- |
| Baseline identities | Adopted SI configuration, controller, accepted native assembly and user M3 save unchanged; historical seals preserved |
| Learning environment | Explicit observation/action/reward contract, free base, physical contact, six finite motor limits and response; seeded reset and failure/timeout semantics |
| Reference agreement | Zero residual matches reference controller trajectories/metrics within stated numerical tolerance, under the same sensing and commands |
| Tooling/benchmark | Isolated dependency lock, actual CUDA/device check, native simulation throughput, GPU model compatibility/performance outcome with retained failures |
| PPO training | A genuinely optimized network, recorded seeds/configuration/reward trace/checkpoints; staged task mix and a separate validation set |
| Policy export | Small portable weights and observation/scaling metadata; deterministic training/export output parity and measured host inference |
| Evaluation | Complete articulated acceptance and half-timestep convergence saved for the candidate; classical/untrained/trained comparisons and held-out trials |
| Reporting | Distinguish functioning training infrastructure, hybrid policy behavior, any measured improvement and remaining qualification holds |

M6 can deliver a trained, evaluated **experimental** policy even if it does not
outperform or pass every baseline criterion. Such a result must keep learned
locomotion acceptance false and name the failing cases. Training reward alone
does not establish balance or hardware readiness. Broad terrain, recovery,
jumping, end-to-end learned control and physical deployment are later scope.
