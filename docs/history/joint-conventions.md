# Articulated joint and feedback conventions

The Milestone 1 model retains its original welded legs. The Milestone 2 generator uses a new free body at the midpoint of the hip pivots, with X forward, Y left, and Z up. SI units are used throughout; the FreeCAD export boundary converts metres to millimetres.

| Joint | Axis and zero | Normal folded pose |
| --- | --- | --- |
| Left/right hip | +Y; upper link points straight down at zero | +40° |
| Left/right knee | +Y relative to upper link; both links align at zero | −80° |
| Left/right wheel | +Y relative to lower link; continuous angle | Positive rotation rolls forward |
| Root | Unconstrained six-degree-of-freedom body | Pitch chosen from posture centre of mass |

For equal links of length L and angles h and k, the axle relative to its hip is:

```text
x = −L sin(h) − L sin(h + k)
z = −L cos(h) − L cos(h + k)
```

Backward-folded symmetric posture uses h = a, k = −2a. It has axle X = 0, hip-to-axle height = 2L cos(a), and knee X = −L sin(a). The face stays on +X. Physical hip limits are 20°–70°; knee limits are −135° to −40°. Acceptance trajectories use a = 30°–60°.

Mass locations are assigned to physical bodies: central payload and hip housings on chassis, upper-link mass and knee housing on upper body, lower-link mass and wheel housing on lower body, tires on driven wheels. Nominal component masses determine posture equilibrium and support feedforward; controllers do not read simulator contact loads or inverse dynamics.

The 200 Hz supervisor estimates attitude from gyro and accelerometer channels and uses quantized, delayed wheel/joint encoder reports for odometry. Wheel encoders measure rotation relative to the lower links. Forward rolling rate therefore includes wheel-relative rate, body pitch-axis gyro rate, hip rate, and knee rate. Filtering those terms together avoids artificial velocity caused by different filter delays. Yaw is gyro-integrated with the stated residual bias; this is not an absolute heading sensor.

Each leg joint has a 1 kHz local encoder/torque servo. Its quantized encoder and filtered derivative are available locally, before the supervisory transport delay. Reference posture and estimated body attitude come from the supervisor. The motor applies finite torque through its modeled response and torque-speed envelope. This architecture requires suitable local actuator control and output-side sensing when hardware is selected.

Positive yaw is a left turn. Common wheel torque balances and drives; differential torque subtracts from the left and adds to the right. Stationary hold uses a planar odometry position projected onto the current forward direction.

Sensor feedback starts with a simulated stationary gravity calibration, including accelerometer noise and bias. Runtime estimation reads named IMU and encoder channels. True pose and velocity are reserved for evaluating motion, contacts, limits, and error plots.

The device front end filters gyro and accelerometer channels at the physics/device rate before sampling supervisory packets. A FIFO then applies the configured transport delay. Attitude is gyro-integrated with gated accelerometer correction and short constant-rate delay extrapolation. Supervisory posture is filtered for equilibrium compensation; the local leg servo still uses current quantized encoders. The delivered speed estimate has no acceleration extrapolation. All timing, bandwidth, quantization, bias, and noise assumptions are saved with each run.
