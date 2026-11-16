# Milestone 2 Moving legs and turning

Created on 30 September 2026 immediately before implementation. Inputs are the [Milestone 0 sizing](00-results.md) and [Milestone 1 balance results](01-results.md). This plan sets proposed checks; achieved results will be recorded separately in `02-results.md`.

## Purpose and scope

Develop a freely moving six-actuator wheeled biped that balances, drives, brakes, turns, and repeatedly crouches on a level floor. Replace welded legs with independent hip and knee hinges. Preserve the existing fixed-height experiment as a regression baseline. Correct the external silhouette so the face points forward (+X) and the knees fold backward (−X).

The milestone also introduces simulated IMU and encoder feedback, sensing delay, push disturbances, and parameter variation. Initial debugging may use exact simulator state; final acceptance must also run from measured sensor channels and an estimator. No world support, motion clamps, prescribed joint positions during simulation, or ideal position actuators may establish success.

Real parts, manufacturing CAD, electronics, firmware, jumping, and uneven terrain remain later work. Motor capabilities, dimensions, friction, and sensor quality are provisional assumptions. Results must identify those assumptions and the conditions actually tested.

## Dependencies and organization

Use the existing Python 3.12 environment at `.venv/` in the repository, with pinned MuJoCo, NumPy, SciPy, Matplotlib, imageio, pytest, and Ruff dependencies. NVIDIA rendering has been verified. No new system installation is initially needed.

Keep `config/robot.json` as the source for dimensions, masses, contact, timing, and physical limits. Add articulated joint assumptions without changing the Milestone 1 mechanism. Keep moving-leg controller settings and sensor assumptions in separate configuration files. Save each run's complete configurations, generated XML, seed, parameter fingerprints, traces, and measured outcome in `artifacts/milestone-2/`.

Implement an articulated model, planar leg kinematics, finite joint motor response, posture-aware balance and heading control, a sensor/estimator pipeline, reproducible scenarios, acceptance aggregation, plots, a recorded video, and a live viewer. Existing Milestone 1 outputs remain available.

## Coordinates, geometry, and mass

X is forward, Y left, and Z up. The body frame origin is at the midpoint between hip pivots. Joint axes are +Y. A straight downward leg is hip = 0 and knee = 0. For a backward-folded equal-link posture with angle a, hip = +a and knee = −2a. Positive wheel rotation rolls forward. Positive yaw is a left turn, requiring the right wheel to receive greater forward effort than the left.

Begin at a = 40°. Exercise the existing 25°–65° geometry range gradually; the initial acceptance crouch trajectory uses 30°–60° to retain room before stops. Provide physical joint limits wider than that trajectory and document their values. At each posture compute axle placement and centre of mass from the nominal component layout, then compute equilibrium pitch. Confirm analytic kinematics against the generated model at several postures.

Retain the assumed 2.02 kg mass budget. Distribute hip housings on the chassis, knee housings on the upper links, wheel motor housings on the lower links, and tire masses on wheel bodies. Keep rotor inertia, damping, finite effort, and contact physics explicit. Parent-child mount geometry may overlap at its intended pivot; nonadjacent parts must remain free of unintended contacts in accepted motion. Layout validity does not establish manufacturing clearances.

## Actuators and control

Retain the wheel motor assumptions: 0.6 N·m peak, 0.25 N·m continuous reference, 30 rad/s no-load speed, and 15 ms torque response. Start joint sizing from the previously computed 1.616 N·m knee demand with allowance. Use explicit provisional joint peak and continuous-reference values, a torque-speed envelope, response lag, damping, and reflected output inertia. Report required torque, speed, and time above reference rather than silently increasing capability during tuning.

Control hips and knees with torque PD feedback and analytically computed gravity/support feedforward. Reference joint trajectories must have smooth position, velocity, and acceleration. The controller must not read contact forces or simulator inverse dynamics as an unavailable hardware measurement.

Adapt balance to posture using nominal geometry and measured joint angles. Use either the existing cascade with scheduling or a documented replacement if the articulated dynamics justify it. Keep balance effort common to both wheels and heading effort differential. Allocate effort within per-wheel limits and report saturation. Use a planar odometry hold target rather than assuming world X remains forward while turning.

Prioritize improved braking margin and smaller acceleration transients. The previous 96.7 mm stopping travel was close to its 100 mm target. Controller changes must be tested as a complete suite, using one configuration across accepted cases.

## Sensors and estimator

Simulate body-mounted gyro and accelerometer measurements, wheel encoders, and joint encoders. Record noise, residual bias, quantization, sampling, transport delay, and deterministic random seeds. Start with a 200 Hz controller, 1 kHz physics, 100 Hz traces, encoder resolution assumptions, and modest IMU noise; test additional delay and noise explicitly.

Estimate pitch using gyro integration and accelerometer correction with acceleration gating. Estimate yaw and planar velocity/position from gyro and wheel odometry, accounting for wheel rotation relative to moving lower links. Use joint encoder feedback for leg servos and posture compensation. Initial attitude may use a simulated stationary gravity calibration before release; identify this startup assumption. The controller may not consume true pose, true chassis speed, or true attitude in sensor-feedback mode. Ground truth is allowed for evaluating errors and plotting.

## Implementation sequence

1. Build and check backward-folded articulated geometry, mass accounting, joint axes, inverse kinematics, finite motor limits, and the unpowered falling comparison.
2. Hold the nominal posture with torque-controlled joints. Establish balance using ideal state before adding crouch or yaw commands. Improve drive and stop response.
3. Add smooth posture trajectories and balance compensation across heights. Verify crouch tracking, joint effort, body clearance, and numerical stability.
4. Add yaw control and planar hold; exercise left/right turns and combined driving, turning, and crouching.
5. Introduce sensor-derived feedback. Re-run stationary, tilt, drive, turn, crouch, and combined tests; add repeatable push inputs and parameter variation.
6. Repeat representative motion at half timestep with unchanged controller timing. Run regression tests, save figures/video, verify the live viewer, and write measured results and remaining limitations.

## Proposed acceptance checks

| Case | Conditions and target |
| --- | --- |
| Geometry and dynamics | Free root, six independent driven hinges, no equality constraints, mass accounting matches the budget; analytic axle/COM checks agree with model; eyes +X and knees −X. |
| Unpowered comparison | Released 5° away from equilibrium, exceeds 30° pitch deviation within 5 s without actuator effort. |
| Hold and tilt recovery | Hold 60 s; ±5° initial offsets remain upright for 60 s and settle within ±2° of posture equilibrium by 3 s. Stationary travel stays within 0.1 m. |
| Drive and brake | Track ±0.3 m/s with plateau mean absolute error <0.03 m/s; stop and hold without falling. Stopping travel <0.1 m, with a nominal design goal ≤0.08 m. Report peak speed and transient pitch. |
| Crouch | At least five 30°–60°–30° cycles while balancing. Joint tracking error <5° after startup, achieved height range ≥40 mm, no forbidden contact or joint-stop violation. |
| Turning | Left and right commands up to 0.4 rad/s. Established yaw-rate mean absolute error <0.08 rad/s; final heading error <8° relative to the commanded heading. |
| Combined manoeuvre | At least 30 s combining drive, turn, and crouch; remain upright, track joint posture, respect effort and stop at a stable hold. |
| Sensor feedback | Hold, ±5° recovery, drive, turn, crouch, and combined cases also pass with documented IMU/encoder noise and transport delay. One seed per full nominal suite, with additional seeds for representative combined tests. |
| Disturbance | Apply separate ±1 N forward/backward body pushes for 0.1 s after settling; return within ±2° of posture equilibrium within 3 s of release and remain upright. |
| Parameter variation | Repeat sensor-feedback drive and representative crouch/turn motion with +15% mass, sliding friction 0.5, wheel response 25 ms, and increased sensor delay/noise, initially one variation at a time. Report failures rather than infer all combinations are covered. |
| Effort and physics | No solver warnings, nonfinite states, body-ground contact, or unintended self-contact. Applied effort respects torque-speed envelopes. No continuous saturation >0.5 s; report peak/RMS torque, speed, and time above continuous references. |
| Timestep check | Representative combined case at 0.5 ms still passes; peak pitch difference <1°, final heading difference <2°, and achieved height-range difference <3 mm. |

During motion, pitch may depart from stationary equilibrium. Limit transient pitch deviation to 20° and roll to 10° in accepted controlled runs; exceeding those values fails the case rather than being corrected through a state clamp. Reassess failed targets with a documented engineering explanation before changing scope or thresholds. Keep this original plan intact and record amendments separately.

## Deliverables and completion gate

Deliver runnable source, shared physical parameters, controller and sensor configurations, a joint convention/kinematics record, full acceptance and variation reports, traces, plots, a rendered motion video, viewer commands, and `02-results.md`. Required regressions include Milestone 1 plus the new articulated and sensor/actuator tests.

Milestone 2 is complete only when the required scenarios and checks pass under their stated assumptions, or an explicit user-approved scope revision changes the gate. A successful nominal animation alone is insufficient. Use measured actuator demands and geometric limitations to prepare Milestone 3, whose detailed plan is written only when that work starts.

## Implementation clarification: servo timing

Initial tests found that putting joint torque PD behind the same delayed 200 Hz measurement path as body balance made the leg servos unstable. Use a local joint encoder/torque servo at 1 kHz with quantization and filtered encoder derivatives, while the body balance, posture compensation, and heading supervisor remain at 200 Hz with delayed IMU/encoder packets. Local actuator feedback does not pass through the supervisory transport delay. This establishes a hardware architecture requirement, not evidence that any chosen actuator meets it. Preserve the original effort limits and exercise sensor-derived feedback in both loops.

The initial 10 ms joint torque-response assumption also produced oscillations at crouched postures with encoder feedback. A 2 ms first-order response supported those trials. Use 2 ms as the provisional joint response requirement for the completed suite and record the dependency explicitly in results. No selected actuator has demonstrated it. The 2.5 N·m joint peak and 1.8 N·m continuous reference remain unchanged; wheel response remains 15 ms nominal, with its planned 25 ms variation test.

## Implementation clarification: supervisory bandwidth

Delayed sensing exposed a fast oscillatory mode in the initial articulated controller. Increasing hold gain, predicting velocity from encoder acceleration, and adding wheel torque lead did not give one robust configuration across the delay tests. Those experimental features are excluded from the delivered controller. Separate the speed and pitch loop bandwidths instead, preserving finite motor response, the original acceptance targets, and one configuration across the suite.

The sensor front end applies first-order gyro and accelerometer bandwidth limits before 200 Hz sampling, at 3 ms and 5 ms time constants respectively. Posture used for balance compensation has a 20 ms filter; local joint positions remain directly quantized encoder measurements. Pitch, roll, and yaw estimates use a short constant-rate extrapolation for known transport and gyro-filter delay. Velocity remains filtered wheel/leg/gyro odometry without acceleration prediction. Record the exact final settings and their measured performance in results.

The first full acceptance run also exposed inadequate proportional balance margin when crouched with 15% extra mass. Keep proportional pitch torque gain constant across posture; scale only the rate-damping term with estimated COM-to-axle height. Reduce the position-hold bandwidth to suit the articulated response. This change retains posture-dependent equilibrium compensation, the original mass variation, and all original gate thresholds. Final acceptance must be re-run after these changes.

## Verification addition: sampled channels and actuator loads

Reviewing the first passing body-motion run found hip torque and speed that differed substantially at half timestep, despite converged body motion. MuJoCo derived sensor channels had been computed before the preceding position integration. Refresh plant-derived measurements at the current sample time before local and supervisory feedback, keeping all sensor noise, quantization, transport delay, and motor response intact. This corrects the simulated sampling schedule; the controller still reads measured channels only.

Strengthen the convergence gate with joint-load checks: maximum per-joint peak torque difference <0.18 N·m, RMS torque difference <0.05 N·m, and peak joint-speed difference <0.3 rad/s between the nominal and half-timestep representative manoeuvres. These supplement the original body pitch, heading, and height checks. Re-run the full suite and regenerate the video after the correction.
