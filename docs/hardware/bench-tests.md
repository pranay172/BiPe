# Bench tests that measure the simulation's assumptions

M18 added motor, sensor, timing and floor effects to the simulation, each at a range
the data sheets and screening assumptions suggest (`bipe/m18/fidelity.py`). Once the
robot is built, these tests replace the ranges with measured values. Each test fills
entries of [`config/hardware-measured.json`](../../config/hardware-measured.json) through
a fitter in `bipe/m18/bench.py`:

```
.venv/bin/python -m bipe.m18.bench record KEY VALUE --source "B3, 2027-01-10, left hip"
.venv/bin/python -m bipe.m18.bench show
```

The simulation reads the file only when a run asks for it:

```
.venv/bin/python -m bipe.m18.suite --checkpoint CKPT --measured     # fidelity suite on the measured robot
.venv/bin/python -m bipe.m15.getup_eval --getup CKPT ... --measured  # get-up on the measured robot
```

A value that differs a lot from the simulation's nominal is worth a fidelity-suite run
before the robot first balances: the suite then shows whether the policy copes with it.
Values that fall outside the M18 training ranges (`fidelity.RANGES`) are the ones that
call for retraining with the range widened.

| Test | Measures (keys) | Setup | Fitter |
|---|---|---|---|
| B1 Mass and COM | `chassis_mass_kg`, `chassis_com_m`, `payload_kg` | Kitchen scale for the chassis without wheels and legs; COM by balancing the chassis on a knife edge along x and y, and hanging it from two points for z, in the chassis body frame of `config/robot.json` (origin and axes as the `chassis` body there) | none (enter directly) |
| B2 Torque constant | `hip_torque_constant_Nm_A`, `hub_torque_constant_Nm_A` | Drive pinned through an arm onto a load cell or kitchen scale; step the q-axis current 0.2 to 2 A in moteus_tool; torque = force x arm, divided by the belt ratio for the hips | `bench.torque_constant` |
| B3 Friction sweep and breakaway | `hip_coulomb_motor_Nm`, `hip_viscous_motor_Nm_s_rad`, `wheel_coulomb_Nm`, `wheel_viscous_Nm_s_rad`, `stiction_ratio` | Unloaded drive (belt off for the RO40 motor terms, wheel off the floor for the hub) in velocity mode at 10 speeds each way, logging the q-axis current x Kt; then a slow current ramp from rest and the torque at which it first moves. The breakaway is measured on the motor side; the simulation applies `stiction_ratio` to the joint-side bearing and pretension friction, so it is a first estimate there | `bench.friction` |
| B4 Cogging | `cogging_fraction` | Same drive, constant 0.5 rad/s for one turn, torque logged against rotor angle | `bench.cogging` |
| B5 Wheel roll-out | `wheel_radius_ratio` | Robot pushed straight along a taped 3 m line on the test floor in its standing posture; hub encoder turns per wheel | `bench.wheel_radius` |
| B6 IMU still capture | `imu_mount_deg`, `gyro_residual_bias_rad_s`, `gyro_noise_std_rad_s`, `gyro_walk`, `accel_noise_std_m_s2` | Robot on its stand, chassis levelled with a machinist's level, motors off; IMU logged at its rate for 30 min after a 10 min warm-up | `bench.imu_still` |
| B7 Six-face accelerometer | `accel_scale` | IMU board (or the chassis on a square jig) resting on each of its six faces for 10 s | `bench.accel_six_face` |
| B8 End-stop check | `encoder_offset_deg` | After calibration, each joint pressed gently onto the end stop it was calibrated on, reading compared with the stop angle | `bench.encoder_offsets` |
| B9 Loop timing | `transport_delay_ms`, `jitter_ms`, `packet_loss`, `command_loss` | Teensy bench image over 10 min at 200 Hz with all six moteus powered, logging for every IMU sample and moteus reply the time it was taken and the start of the control tick that used it (the age of the data when the control law runs, about one 5 ms tick on a synchronous loop, not the CAN round trip); also counts the command acknowledgements | `bench.loop_timing` |
| B10 Incline slip | `floor_friction` | Robot with wheels locked (position hold) on a board covered with the test floor, tilted slowly until it slides; the angle | `bench.incline_slip` |

Notes:

- B6's bias walk comes from the Allan deviation at a tenth of the capture length, so a
  30 min capture resolves walks down to about 1e-5 rad/s per root second; shorter
  captures overstate it.
- B6's mounting tilt should also be rotated out in the firmware (the IMU-to-chassis
  rotation at start-up), not only modelled; the simulation then runs with the
  residual tilt after that correction.
- B9 measures data age, which is what the simulation's `transport_delay_s` and the C
  estimator's prediction delay mean; the CAN round trip alone (about 0.4 ms) would set
  the simulated delay to zero ticks. The full plant applies delay and jitter in whole
  5 ms ticks, so a jitter under 2.5 ms changes nothing there (the training environment
  applies it in 1 ms steps). A reading used more than one tick late counts as lost, as
  the firmware's synchronous loop treats it. If B9 shows replies regularly slipping by whole ticks, the
  loop is not synchronous as M11 designed it, and the policy will not cope (M18,
  16 November: fine-tune 1 falls within a second with whole-tick jitter); fix the loop
  before anything else.
- The torque-speed limits, FOC lag, belt stiffness and lash, pack sag and IMU noise
  were modelled before M18; B2, B3 and B6 measure the inputs the simulation already
  randomises, the rest measure what M18 added. The hardware build plan
  ([build-plan.md](build-plan.md)) has the stage at which each test fits.
