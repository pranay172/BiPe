# Milestone 1 Balance at fixed height

Created on 30 September 2026 immediately before implementation, following [Milestone 0 results](00-results.md). This milestone establishes a reproducible balance experiment for the preliminary 2.02 kg robot. Its leg posture is fixed; moving legs and turning belong to the next milestone.

## Physical model

Generate MJCF from the shared SI parameter file. Represent the central body, joint motor envelopes, wheel motors, and links as a welded assembly with explicit component masses. Give the assembly a free joint in six degrees of freedom and each wheel a hinge about Y. No rail, pitch constraint, contact fixture, pose clamp, or world support may stabilize the robot.

Use contact with a level ground plane, explicit tire friction, wheel inertia, motor rotor inertia, viscous losses, and a finite torque-speed envelope. Default wheel limits are assumed values: 0.6 N·m peak, 0.25 N·m continuous reference, 30 rad/s no-load speed, and 15 ms torque response. These must remain clearly identified as assumptions rather than chosen actuator specifications.

Compute the equilibrium pitch from the generated model's fixed-assembly centre of mass. Initial ±5° disturbances are relative to that equilibrium, not relative to the CAD body's vertical orientation. Keep the render geometry and mass geometry tied to the same component layout.

## Controller and state

Use an outer speed PI loop that commands a bounded pitch reference and an inner pitch PD loop that commands wheel torque. A first-order motor torque response applies the requested effort subject to the motor torque-speed envelope. This uses a torque input rather than the velocity-mode actuator used in parts of the reference series. The abstraction remains compatible with a future actuator driver that exposes torque control.

Add a bounded position-to-speed command for stationary hold and a rate-limited user speed command for driving. Use integral clamping or conditional integration to prevent windup. Log requested and applied effort separately. Use simulator state for this first debugging baseline; do not claim sensor realism yet.

The controller runs every 5 ms, physics every 1 ms, and logs every 10 ms. Determine gains by controlled experiments and save the final values. The default controller must not depend on scenario-specific gain changes or direct edits to the robot pose during a run.

## Implementation order

1. Generate and load the free-base MJCF; verify joint axes, total mass, centre of mass, and output inertia. Show that an unpowered tilted robot falls.
2. Check that positive wheel effort moves the wheel contact in the expected forward direction. Establish controller signs on short runs before tuning.
3. Implement balance, speed, hold, and actuator response. Tune on short tilt and command experiments, then run the longer acceptance suite.
4. Repeat selected checks at half the physics timestep, with the same controller interval, to test numerical convergence.
5. Save the model, configurations, CSV traces, metrics, plots, a rendered image and video, and a viewer command.
6. Run meaningful regression checks for the unconstrained model and acceptance behaviour. Write the results document with exact conditions and remaining limitations.

## Acceptance scenarios

| Scenario | Proposed pass conditions |
| --- | --- |
| Unpowered comparison | A robot released 5° away from equilibrium exceeds 30° deviation within 5 seconds without control, demonstrating instability. |
| Positive and negative tilt | Starting ±5° from equilibrium, remain upright for 60 seconds, enter and remain within ±2° after at most 3 seconds, and keep total travel within 0.1 m during stationary hold. |
| Stationary hold | Over 60 seconds, remain within 0.1 m of the hold position and ±2° of equilibrium after settling. |
| Forward and reverse | Rate-limited commands reach +0.3 and −0.3 m/s; mean absolute tracking error during established command plateaus is below 0.03 m/s. Return to a stable hold after braking. |
| Effort | Applied torque respects the configured envelope; report peak, RMS, time above the continuous reference, and saturation fraction. No sustained saturation interval over 0.5 seconds. |
| Numerical convergence | Repeated tilt and drive scenarios at 0.5 ms physics still pass; peak pitch differs by less than 0.5° and RMS velocity by less than 0.01 m/s. |

Document stopping drift after the command reaches zero, measured from the hold position captured at the start of stopping. A transient pitch excursion during a driving ramp is permitted; the free base must remain upright and return to equilibrium at hold. The model must produce no numerical instability warnings or body-ground collision in successful runs.

## Deliverables and limits

Deliver an executable experiment, generated model, controller configuration, acceptance report, plots, video, and a results note. Record dependency versions and parameter file fingerprints so results can be regenerated.

Success establishes balance for the assumed model at one fixed posture. It does not validate actual motors, sensor noise, communication latency, crouching, rough terrain, or hardware safety. Those remain explicitly outside the reported achievement.

## Results

Implementation and acceptance checks completed on 30 September 2026. See [Milestone 1 results](01-results.md) for measured outcomes, reproduction commands, and limitations. Proposed criteria above are retained separately from those results.
