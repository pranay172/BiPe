# Milestone 1 Results

Completed on 30 September 2026. All planned fixed-height acceptance scenarios and the selected half-timestep checks pass for the assumed model. The [plan](01-fixed-height-balance.md) records scope and criteria; the machine-readable result is [acceptance.json](../../../artifacts/milestone-1/acceptance.json).

## Model and controller

The robot has an unconstrained six-degree-of-freedom root and two wheel hinges. The torso, leg links, joint motor envelopes, and wheel motor housings form one fixed assembly at the 40° leg posture. Total mass is 2.02 kg. There are no equality constraints or support fixtures, and the controller never edits position or orientation during physics steps.

Dimensions and mass assumptions come from [robot.json](../../../config/robot.json). The model includes wheel and reflected rotor inertia, tire friction, viscous losses, a 15 ms first-order torque response, and a motor torque-speed envelope. Each wheel has an assumed 0.6 N·m peak torque, 0.25 N·m continuous reference, and 30 rad/s no-load speed. The continuous reference is an effort reporting threshold; thermal behaviour is not modeled and these values do not select a real motor.

The centre of mass gives a −4.151° equilibrium body pitch. Pitch errors and initial tilt disturbances are relative to that equilibrium. Feedback uses ideal simulator pitch, pitch rate, forward velocity, and position. IMU sites exist in the model, but a noisy sensor estimator is not yet used.

The cascade uses speed PI gains 0.70 and 0.10, pitch PD gains 1.50 and 0.20, and a bounded position-hold layer. [controller.json](../../../config/controller.json) contains all settings. The pitch reference may move ±0.22 rad from equilibrium, and the velocity reference changes at up to 2.0 m/s². That rate is a command limit, not a bound on measured acceleration. Settings are identical across scenarios.

Physics uses 1 ms steps, control updates every 5 ms, and traces are recorded every 10 ms. Motor response is applied every physics step. Short tuning trials found excessive stopping travel with tighter pitch and speed reference bounds. Widening those bounds met the stopping criterion without increasing motor torque limits.

## Measured acceptance results

| Check | Result |
| --- | --- |
| Unpowered 5° release | Exceeded 30° deviation after 0.207 s, demonstrating physical instability without control. |
| Stationary hold | Completed 60 s within the pitch and travel bounds. It starts exactly at equilibrium with ideal sensing; tiny numerical errors do not predict hardware precision. |
| +5° and −5° recovery | Both completed 60 s; entered and stayed within ±2° after 0.210 s. Maximum travel was 28.52 mm. |
| Tilt error after 3 s | At most 0.00741° in either recovery run. |
| Forward and reverse | Completed 20 s; commanded +0.3 m/s from 2–7 s, −0.3 m/s from 9–14 s, and hold otherwise. |
| Steady-speed mean absolute error | 0.00338 m/s over the 4–7 s and 11–14 s plateaus. |
| Maximum stopping travel | 96.73 mm from the stopping position through the hold intervals. Below the implemented 100 mm target, with little margin. |
| Final hold after driving | Pitch error after 17 s at most 0.0319°; final speed 0.000101 m/s. |
| Peak driving pitch excursion | 11.93° relative to equilibrium during acceleration or braking. The stationary ±2° criterion does not apply during driving transients. |
| Peak applied wheel torque | 0.10885 N·m in tilt recovery; 0.12636 N·m while driving. |
| Saturation / effort reference | No recorded saturation or time above the 0.25 N·m continuous reference in accepted runs. |
| Contacts and solver | No body-ground contact or MuJoCo numerical warnings in successful controlled runs. |

The drive trace shows speed overshoot as the wheels move to catch the body. The plateau error is a steady-speed measure; measured speed does not stay within ±0.3 m/s throughout acceleration and braking. The [plot](../../../artifacts/milestone-1/drive/trace.png) exposes those transients.

Repeating positive tilt and drive with 0.5 ms physics and unchanged control timing also passed. Drive peak pitch changed by 0.0169° and RMS velocity by 0.000469 m/s. Tilt peak pitch was unchanged because the initial condition sets that maximum; RMS velocity changed by 0.000233 m/s. These support timestep consistency for the selected scenarios, not robustness to parameter changes.

## CAD, appearance, rendering, and checks

The model uses [Milestone 0 geometry](00-results.md), including the editable [FreeCAD blockout](../../../artifacts/milestone-0/blockout.FCStd). Enclosure styling remains future work.

The eyes are at body X = +56 mm; the knees extend toward +X as well. A recorded +0.3 m/s command at 5 s produces +0.3036 m/s travel along world X, confirming that the eyes face the defined forward direction. [orientation-check.json](../../../artifacts/milestone-1/drive/orientation-check.json) records the check. The eyes are massless and non-colliding.

Compared with the [reference robot imagery](https://www.mondorobotics.com/), which appears to fold the knees away from its face, our leg silhouette is reversed. This is an exterior interpretation, not a claim about the reference robot's internal mechanism. Record backward folding as a design constraint for the moving-leg layout; the validated baseline is retained during this orientation check.

The drive recording rendered on NVIDIA as a 1280 × 720 MP4 at 30 fps. Its 601 decoded frames cover 20 seconds of recorded physics; including both endpoints produces approximately 20.03 seconds of encoded duration. First, peak-pitch, and final frames decoded at the expected dimensions. The video replays measured poses rather than scripted motion.

Six regression tests passed: shared mass and equilibrium consistency, free-base topology, wheel effort direction, motor limits, expected unpowered instability, and both tilt directions. Ruff checks passed. The live viewer opened, ran four seconds of physics, and closed with exit code zero after asynchronous rendering thread shutdown was handled.

## Reproduction and artifacts

From the repository root, using its existing `.venv`:

```bash
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.validate
.venv/bin/python -m pytest -q
MUJOCO_GL=egl .venv/bin/python -m bipe.render artifacts/milestone-1/drive
```

Each scenario directory contains `robot.xml`, `trace.csv`, `trajectory.npz`, `configuration.json`, and `metrics.json`. Normal-timestep scenarios also contain `trace.png`. Configuration snapshots and SHA-256 fingerprints identify the parameters and controller settings used, including the changed timestep for convergence checks. Dependencies are pinned in [requirements.lock](../../../requirements.lock).

- [Drive video](../../../artifacts/milestone-1/drive/simulation.mp4), [initial render](../../../artifacts/milestone-1/drive/robot.png), and [peak-pitch frame](../../../artifacts/milestone-1/drive/braking.png)
- [Positive tilt plot](../../../artifacts/milestone-1/tilt_positive/trace.png) and [negative tilt plot](../../../artifacts/milestone-1/tilt_negative/trace.png)
- [Acceptance report](../../../artifacts/milestone-1/acceptance.json) and [video decode check](../../../artifacts/milestone-1/drive/video-check.json)
- [Model generator](../../../bipe/model.py), [controller](../../../bipe/control.py), and [scenario runner](../../../bipe/experiment.py)

## Handoff

Milestone 2 begins with its own detailed plan immediately before implementation. It must test sensitivity to mass, friction, delay, and sensing errors; improve braking margin and assess transient motion; introduce independent leg joints, posture compensation, crouch trajectories, and differential-wheel heading control. It must also align the leg folding direction with the intended face and silhouette. The current acceptance suite remains a regression baseline.

Success establishes fixed-height balance on a level surface for one provisional geometry. Crouching, turning, real motor feasibility, noisy sensing, manufacturing clearances, strength, and electrical behaviour remain unverified. Physical measurements are necessary before the digital design can be considered build-ready.
