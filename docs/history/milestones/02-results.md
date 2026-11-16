# Milestone 2 results

Completed on 30 September 2026 under the simulation assumptions below. The [detailed plan](02-moving-legs-and-turning.md) was created before implementation. All 27 scenarios pass, the representative half-timestep run passes the strengthened body-motion and joint-load convergence checks, and one controller configuration is used throughout. Milestone 1 regression acceptance, 13 Python tests, and Ruff checks also pass.

The robot now balances, drives, brakes, turns, and repeatedly crouches with four independently torque-controlled leg joints and two driven wheels. Sensor-feedback cases use simulated IMU and encoder measurements. These results establish a digital baseline; actuator capabilities and sensor specifications have not been verified against selected hardware.

## Delivered model and control

The root has six free degrees of freedom, with six driven hinges, 12 generalized velocities, and zero equality constraints. Total mass is 2.02 kg. Analytic axle and centre-of-mass calculations agree with the generated model to 5.55e-17 m at 25°, 40°, 60°, and 65° leg postures. The face is on +X, and knees fold toward −X. See [joint conventions](../joint-conventions.md).

The nominal posture is hip +40° and knee −80° on each leg. Hip limits are 20°–70°; knee limits are −135° to −40°. Crouching uses smooth quintic 30°–60°–30° trajectories, five cycles over 30 seconds, with smooth transitions into and out of the sequence. Joint motion is produced through applied torque, finite response, rotor inertia, and physical contact. No prescribed pose, support constraint, or state clamp holds the robot upright. With actuators disabled, a 5° release reaches 30° pitch deviation in 0.344 s.

Balance uses posture-dependent equilibrium pitch, constant proportional pitch gain, and height-scaled rate damping. A speed PI loop supplies pitch offset; odometry supplies planar position hold. Heading control adds differential wheel torque. Local joint servos use quantized encoder positions, filtered derivatives, and nominal gravity/support feedforward. Controller code does not read contact forces or simulator inverse dynamics.

## Conditions and provisional hardware requirements

| Item | Final assumption |
| --- | --- |
| Floor | Level; sliding friction 0.8 nominal, 0.5 in a separate variation |
| Timing | 1 ms physics, 5 ms body supervisor, 1 ms local joint servo, 10 ms recorded traces |
| Wheels | 0.6 N·m peak, 0.25 N·m continuous reference, 30 rad/s no-load speed, 15 ms torque response |
| Leg joints | 2.5 N·m peak, 1.8 N·m continuous reference, 12 rad/s no-load speed, 2 ms torque response |
| Encoders | 4096 wheel counts/revolution; 16384 joint counts/revolution |
| IMU noise | Gyro σ = 0.003 rad/s; accelerometer σ = 0.06 m/s² |
| Residual bias | Gyro [0.0003, −0.0003, 0.0003] rad/s; accelerometer [0.01, −0.01, 0.01] m/s² |
| Supervisory transport | 5 ms nominal; 15 ms with doubled IMU noise in a separate variation |
| Device filters | Gyro 3 ms; accelerometer 5 ms before 200 Hz sampling |
| Estimator filters | Velocity 5 ms; posture 20 ms; local joint-rate derivative 2 ms |
| Startup | Simulated stationary gravity calibration before free release; residual gyro bias is assumed after calibration |

Transport delay applies to the body supervisor. The local encoder/torque servo runs before that transport path. Suitable local control, output-side sensing, and a 2 ms joint torque response are requirements to verify when selecting real parts. No part has demonstrated them yet. Noise, quantization, bias, lag, friction, masses, damping, and rotor inertia are stored in each experiment's configuration.

The estimator uses gyro integration, gated accelerometer correction, short constant-rate attitude extrapolation for known delay, and wheel odometry that includes hip, knee, and body rotation. Runtime estimation reads named sensor channels only. Plant-derived channels are refreshed at their sampling time before feedback. Ground truth is used to evaluate motion, contacts, effort, and errors.

## Measured nominal sensor-feedback results

| Check | Result |
| --- | --- |
| 60 s stationary hold | Maximum travel 9.5 mm; maximum pitch error 0.635° |
| +5° / −5° recovery | Settled within ±2° by 0.29 / 0.35 s; held for 60 s |
| ±0.3 m/s driving | Plateau mean absolute speed error 0.0125 m/s |
| Braking | Maximum travel after stop command 91.0 mm, below the 100 mm gate |
| Left/right turning at 0.4 rad/s | Established yaw-rate error 0.0266 rad/s; final heading error 0.232° |
| Five crouch cycles | Hip-to-axle height span 66.6 mm; maximum joint tracking error after startup 0.110° |
| 36 s combined motion | Height span 70.2 mm; maximum pitch error 8.489°; final heading error 0.509° |
| Separate ±1 N, 0.1 s pushes | Both recover within the specified 3 s window; maximum pitch error 2.864° |

Pitch errors refer to the actual posture equilibrium. Driving and combined motion include deliberate lean and brief velocity transients when commands change. The nominal drive peaks at 0.597 m/s and 12.19° pitch deviation; the 0.3 m/s target describes established driving. The aspirational 80 mm nominal stopping goal remains unmet; the required 100 mm limit passes.

## Variation and repeatability

The suite contains eight nominal cases with ideal feedback, the same eight with sensor feedback, the unpowered comparison, two additional combined-motion seeds, and drive/combined pairs for four separate variations. Default seed is 20260930; extra seeds are 20260931 and 20260932. Variations retain the nominal controller's mass model and gains.

| Separate variation | Drive speed MAE | Maximum stop travel | Combined case |
| --- | ---: | ---: | --- |
| Uniform mass +15%, to 2.323 kg | 0.0111 m/s | 96.5 mm | Pass |
| Sliding friction 0.5 | 0.0129 m/s | 91.0 mm | Pass |
| Wheel torque response 25 ms | 0.0110 m/s | 93.7 mm | Pass |
| Sensor transport 15 ms, IMU noise ×2 | 0.0109 m/s | 82.8 mm | Pass |

These variations were tested individually. Combined extremes, tire compliance, backlash, structural flexibility, electrical faults, thermal behavior, uneven floors, and real hardware timing remain unverified.

## Effort and numerical checks

Across controlled cases, peak hip torque is 0.242 N·m, knee torque 0.772 N·m, and wheel torque 0.155 N·m. The largest per-actuator RMS values are 0.123, 0.513, and 0.026 N·m respectively. No controlled case exceeds a continuous reference or saturates. Maximum controlled joint speeds are 0.507 rad/s at a hip and 0.828 rad/s at a knee; wheel speed reaches 18.630 rad/s. Unpowered falling speeds are excluded from hardware operating demands.

Maximum controlled pitch deviation is 13.911°, roll 0.068°, and joint tracking error after startup 2.137°. All accepted controlled cases have finite states, zero solver warnings, no forbidden contact, and no joint-stop violation.

The half-timestep combined run keeps the 200 Hz supervisor and 1 kHz local servo unchanged:

| Difference between 1 ms and 0.5 ms physics | Measured | Limit |
| --- | ---: | ---: |
| Peak pitch error | 0.0668° | 1° |
| Final actual heading | 0.0226° | 2° |
| Height span | 0.0452 mm | 3 mm |
| Maximum per-joint peak torque difference | 0.00748 N·m | 0.18 N·m |
| Maximum per-joint RMS torque difference | 0.000122 N·m | 0.05 N·m |
| Maximum per-joint peak speed difference | 0.01825 rad/s | 0.3 rad/s |

The load checks were added after noticing timestep-sensitive joint oscillation in an earlier body-motion pass. Correcting channel sampling removed it. Earlier tuning and validation attempts are development evidence; the final [acceptance report](../../../artifacts/milestone-2/acceptance.json) and its named scenario folders define the accepted configuration.

## CAD and visual verification

The updated [FreeCAD blockout](../../../artifacts/milestone-2/cad/blockout.FCStd) has 17 valid objects and reopens successfully. [STEP export](../../../artifacts/milestone-2/cad/blockout.step) and a 20 × 30 × 40 mm solid round-trip pass. The nominal upright layout envelope is approximately 143.85 × 189.00 × 252.89 mm, including the eyes. This is a placeholder layout at 40° leg posture; manufacturing clearances, bearings, shell design, and the complete motion envelope require Milestone 3.

The [recorded video](../../../artifacts/milestone-2/sensors_combined/simulation.mp4) contains 1081 frames at 30 fps, 1280 × 720, covering the accepted 36 s combined trajectory. Start, crouched, and final frames were decoded and visually inspected, along with [measured traces](../../../artifacts/milestone-2/sensors_combined/trace.png). The renderer reports NVIDIA RTX 5050 and OpenGL 4.6. The final live viewer executed the complete sequence for 37 s and closed with exit code zero. Manual keyboard inputs were not separately automated.

## Reproduce and inspect

Run from the repository root; keep the existing uv environment at `.venv/`:

```bash
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.moving_validate
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.validate
.venv/bin/python -m pytest -q
.venv/bin/ruff check bipe cad tests
MUJOCO_GL=egl .venv/bin/python -m bipe.render artifacts/milestone-2/sensors_combined
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia .venv/bin/python -m bipe.moving_viewer --scripted --seconds 37
BIPE_ARTICULATED=1 freecad.cmd cad/export_blockout.py
```

Without `--scripted`, the articulated viewer accepts Up/Down for driving, Left/Right for turning, X for hold, and C for five crouch cycles. The experiment runner supports individual scenarios through `python -m bipe.moving_experiment`.

Every accepted case stores CSV traces, poses, generated XML, complete configurations, a SHA-256 configuration fingerprint, metrics, and a plot. See [shared parameters](../../../config/robot.json), [controller](../../../config/controller_moving.json), [sensors](../../../config/sensors.json), [CAD check](../../../artifacts/milestone-2/cad/cad-check.json), [GPU check](../../../artifacts/milestone-2/gpu-check.json), [viewer check](../../../artifacts/milestone-2/viewer-check.json), and [video check](../../../artifacts/milestone-2/sensors_combined/video-check.json).

Milestone 3 will select actual actuators and develop the compact mechanical assembly, then replace provisional masses, inertias, and response limits with documented component values and repeat simulation. Its detailed plan will be written immediately before that implementation.
