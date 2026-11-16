# M12: camera following

Plan started 19 October 2026, after M11 completed the agreed initial
scope (a hardware-oriented digital prototype that balances, drives,
turns and crouches). Camera following was the first item of later work.
Jumping stays later work.

M12 adds camera following to the digital prototype: the robot keeps a
target in view and at a set distance while balancing. The deliverables
are:
- requirements;
- perception tested on the images the simulation renders from the head
  camera;
- a following controller in the supervised firmware;
- the Pi-to-Teensy link and any hardware changes;
- closed-loop acceptance in the free-base plant.

As before, nothing is bought, built or powered. The results are a
validated simulation under stated assumptions, not a hardware-validated
design.

## Starting point

Hardware reserved in M10 and M11:
- **Camera.** One Raspberry Pi Camera Module 3 envelope sits in the head
  pod, behind a 14 × 12 mm window, pitched 15°
  (`m10.cad.head.camera_pitch_deg`). The lens block and offset are
  assumptions. The two front body "cameras" are styling lenses with no
  board.
- **Vision host.** A Raspberry Pi Zero 2 W slot is reserved against the
  rear wall, with a keep-out for the camera FPC (M11 phase B decision).
  It is not wired to the controller. The M11 power budget already
  carries it (about 3.7 W with the camera); runtime with the vision host
  running is 26–36 min.
- **Head.** The head is fixed. Its neck interface was sized for a later
  pan joint, and the allocation carries a 40 g head-pan reserve.
- **Mass.** CAD 1.815 kg, allocated 1.975 kg, against the 2.15 kg cap.
  That leaves a 0.175 kg margin above the 0.15 kg reserve.
- **Carrier.** The Teensy 4.0 carrier has no link to the Pi. The routed
  carrier uses pins 2–5, 7, 9–14, 30 and 31.

Controller and acceptance:
- **Command interface.** The balance controller takes velocity,
  yaw-rate and posture (qref, qdref) commands
  (`bipe/moving_experiment.py`, `firmware/src/balance.c`). Command
  acceleration is limited to 3 m/s², and the hold speed to 0.1 m/s.
- **Validated envelope.** Acceptance covers driving at 0.15–0.3 m/s,
  turning at 0.25–0.4 rad/s, crouching, 2 N·s pushes, slopes to 10°,
  and the sensor, delay, mass, friction and backlash variants. It does
  not cover faster driving.
- **Wheel headroom.** At the 18.24 V hub drive limit, the GL40's
  no-load speed is about 1160 rpm, or 4.4 m/s at the 36 mm wheel. A
  normal walk is 1.2–1.4 m/s.

Tooling:
- MuJoCo offscreen rendering (EGL or OSMesa) currently fails in this
  session. The NVIDIA and Mesa EGL libraries are installed, so phase A
  starts by fixing or explaining it. Vision in the loop needs rendered
  frames.

## Rules

These carry over from the earlier milestones:
- **Balance acceptance.** It keeps the free base, finite motor limits
  and physical contact. Following must not lean on hidden pose
  constraints or clamps. The target is a body in the simulated world,
  never a constraint on the robot.
- **Kinds of evidence.** Keep layout studies, validated simulation and
  hardware validation separate. Perception results say which images
  they were measured on: rendered frames, recorded footage or none.
- **Parts.** Choose popular, budget-friendly and easy-to-obtain parts,
  checked against computed loads and manufacturer documentation. Record
  price and stock for each.
- **Records.** Save configurations and traces with each output. Rerun
  the acceptance suite after any physical model or controller change.
- **Safety.** Following only issues commands inside the supervised
  envelope. A lost or stale target brings the robot to a stop.

## Phases

| Phase | Content | Exit |
|---|---|---|
| A. Requirements and scenarios | What is followed and how: range band, bearing tolerance, target speeds and paths, start, loss and reacquisition behaviour. Camera geometry (FOV, pitch, mounting height) against the paths. Target models and scenario set in the simulated world. Rendering toolchain working | Requirements table with each value's basis; scripted target scenarios; rendered head-camera frames saved |
| B. Speed envelope (to about 1.0 m/s) | Controller retune and gates for the faster driving and turning that following needs: acceleration, braking, turning at speed, the push and slope cases at speed, falls at the new speed | Acceptance and falls pass at the new envelope, or the shortfalls are recorded and signed off |
| C. Perception | Camera model: intrinsics, resolution, frame rate, exposure and motion blur, latency, rendered from the head camera pose. Detector chosen against a Pi Zero 2 W budget. Measurement model (bearing, range, dropouts) checked against ground truth on rendered frames | Detector accuracy and rate on rendered frames; a Pi Zero 2 W timing estimate with its basis; the measurement model |
| D. Following controller | Target estimator (bearing and range filter, prediction over the latency, loss timeout). Follower producing velocity and yaw-rate commands inside the envelope, with stop on loss. C implementation in the supervised firmware with Python parity, unless decided otherwise | Parity tests; open-loop and closed-loop tests on the measurement model |
| E. Integration | Pi-to-Teensy link: protocol, rate, latency and watchdog. Carrier revision for the link if needed. Any head-pan hardware. CAD, mass, power and harness updates, with the gates rerun if the plant changes | Clean fit and sweep, mass within the cap, and a latency budget for the following loop |
| F. Closed-loop acceptance and exit | Following scenarios in the free-base plant with rendered perception in the loop, under the robustness variants. Existing acceptance unchanged or explained. Results and README status | Following targets met, the balance acceptance unchanged or explained, records saved |

Detailed plans for each phase are written when the phase starts.

## Decisions for the user

Before phase A:
- **What is followed.** Either:
  - **a fiducial marker** (an AprilTag worn by the person or carried on
    an object). Its detector is deterministic, cheap enough for a Pi
    Zero 2 W, and gives range and bearing directly. The same detector
    code can run on frames rendered in MuJoCo, so the simulation tests
    the real perception path; or
  - **a person, with a learned detector** (for example a MobileNet-SSD
    or YOLO-nano class model). That is the eventual goal. But a
    COCO-trained detector cannot be credibly tested on MuJoCo's
    simplified human shapes, so in simulation it would only be a
    measurement model. Its accuracy would be argued from published or
    recorded data, and range from box height is noisier.
- **Following speed.**
  - Stay inside the validated 0.3 m/s, which only covers a slow walk.
  - Or extend the envelope (phase B) to about 1.0 m/s, which covers a
    slow-to-normal walk. That needs a retune and the gates at speed.
- **Head pan.**
  - Keep the head fixed and turn the whole robot to face the target.
    The camera's field of view sets how fast the target can move
    sideways. The Camera Module 3 Wide (about 102° horizontal FOV,
    against 66° for the standard lens) would widen the margin.
  - Or add a pan joint using the 40 g reserve. That means another
    actuator and driver, and changes to the CAD, plant and electronics,
    so the gates need rerunning.
- **Where the follower runs.**
  - On the Teensy, inside the supervised C core: the Pi sends target
    measurements, and the command limits and stale-target stop stay
    beside the balance controller.
  - Or on the Pi: it sends velocity and yaw-rate commands, and the
    Teensy only applies the existing envelope.

Later, when their phases start:
- **Phase C:** the camera variant and resolution, and the detector
  model, with price and stock.
- **Phase E:** the link hardware (a UART on free Teensy pins with a
  carrier revision, or another bus), and any head-pan part choices.

User decisions (19 October 2026):
- **Target:** an AprilTag marker, worn or carried by the person.
  Person detection stays a later extension.
- **Speed:** extend the envelope to about 1.0 m/s, so phase B is
  required.
- **Head:** stays fixed, and the robot turns its body to face the
  target. The 40 g pan reserve stays unused. Phase C chooses the lens
  (standard or Wide).
- **Follower:** runs on the Teensy, inside the supervised C core. The Pi
  sends tag measurements.

## Phase A plan: requirements and scenarios (19 October 2026)

Goal: fix what "following" must achieve, and build the simulated world
that tests it, before any controller or perception work.

1. **Rendering toolchain.**
   - Get MuJoCo offscreen rendering working: EGL on the NVIDIA GPU
     first, then OSMesa. Record the backend and versions.
   - Check that the same state renders the same frame twice, bit for
     bit, so rendered-perception runs are reproducible.
   - If a package is needed (for example OSMesa), it is a download
     decision for the user.
2. **Head-camera geometry** (`bipe/m12/geometry.py`).
   - Take the camera pose in the body frame from the M10 CAD parameters:
     the head position and the 15° pitch. Lens offsets are flagged as
     assumptions.
   - Compute the camera height and line of sight over the posture
     range (nominal to full crouch) and over the balancing pitch.
   - For both Camera Module 3 lenses (standard 66° × 41°, Wide
     102° × 67°), find:
     - the band of heights where a tag stays inside the vertical field
       of view across the range band;
     - the largest bearing rate that keeps the tag inside the
       horizontal field of view with a margin, given the robot's yaw
       response.
   - The datasheet FOV values are taken from Raspberry Pi's
     documentation and recorded with their source.
3. **Following requirements** (`artifacts/m12/requirements/requirements.json`).
   These are proposed values, each with its basis, for the user's
   sign-off at the phase exit:
   - nominal range and band (starting proposal 1.0 m, band 0.7–1.5 m);
   - bearing tolerance;
   - target speed (0–1.0 m/s) and acceleration;
   - path curvature;
   - engage rule: the tag is seen for N frames before following starts;
   - loss rule: after a timeout the robot brakes to hold, with a bounded
     stopping distance, and does not search;
   - reacquisition;
   - tag size against the range band.
4. **Target model and scenarios** (`bipe/m12/scenarios.py`).
   - The target is a person-sized kinematic (mocap) body in the MuJoCo
     world, carrying a flat marker plane. It moves on scripted paths and
     never touches or constrains the robot.
   - Phase A uses a placeholder marker texture. The AprilTag family and
     its images come with the detector in phase C.
   - Scenarios:
     - straight walks at 0.3, 0.6 and 1.0 m/s;
     - start-stop;
     - curves of 1 m and 2 m radius;
     - a 90° corner;
     - a lateral crossing;
     - an approach (the target walks toward the robot);
     - loss by leaving the field of view, and by occlusion.
   - Each scenario is deterministic and saves its configuration.
5. **Envelope needed for phase B.**
   - Run an ideal follower (perfect target knowledge, no balance
     dynamics) on the scenarios with the proposed range band.
   - From it, derive the robot speed, acceleration, yaw rate and yaw
     acceleration that following needs. These become the phase B
     targets.
6. **Records and tests.**
   - Rendered head-camera frames for each scenario at a few times
     (`artifacts/m12/requirements/frames/`).
   - The scenario and geometry configurations.
   - Tests for scenario determinism, the geometry against the CAD
     parameters and the requirements file.
   - The plant and controller don't change in phase A, so no acceptance
     rerun is needed.

Exit:
- the requirements table, with each value's basis, signed off by the
  user;
- the scenario set;
- the phase B envelope targets;
- rendered frames saved with their configurations.

## Phase A results (20 October 2026)

Evidence: a layout study plus an ideal-follower simulation under the
stated assumptions. There is no balance dynamics and no perception in the
loop. The frames are rendered with the robot placed kinematically at the
follower's poses. Everything is regenerated by
`.venv/bin/python -m bipe.m12.requirements` into
`artifacts/m12/requirements/`. Two runs gave bit-identical frames and
records.

**Rendering toolchain.**
- **Backend.** MuJoCo 3.14.0 offscreen on EGL: NVIDIA GeForce RTX 5050
  Laptop GPU, driver 595.99.02, OpenGL 4.6. It renders at about 0.5 ms
  per 640 × 360 frame. `bipe.m12` selects EGL by default. The EGL errors
  seen at interpreter exit came from renderers left to the garbage
  collector; renderers are now closed explicitly.
- **Determinism.** Two effects showed up, both at single pixels and one
  grey level:
  - The first render of a state can differ from the frame rendered
    before it. `scenarios.frame()` therefore renders twice and keeps the
    second image.
  - With shadow maps and multisampling both on, some states differ
    between consecutive renders. The M12 world therefore casts no
    shadows; multisampling stays on, since its edges are closer to a
    camera's blur.
- **Checks.** With both measures, an image depends on the state alone:
  - a test renders states in forward and reverse order;
  - the build re-renders its first frame after all the others (5/5
    identical).
- **No package download was needed.**

**Camera geometry** (`bipe/m12/geometry.py`, the camera block in
`config/robot.json` `m12`).
- **Source.** FOV and lens data are from the Raspberry Pi camera
  documentation:
  - standard lens: 66° × 41°;
  - Wide lens: 102° × 67°.
- **Camera pose.** The pinhole is on the 15° lens axis, 61.8 mm ahead of
  the hip axis and 64.5 mm above it. In the nominal posture that puts it
  0.168 m above the ground and 74 mm ahead of the axle. The CAD formulas
  are reproduced in metres, and a test checks them against the camera in
  the simulated world.
- **Tag height.** These are the tag-centre heights that keep a 0.16 m tag
  inside the vertical view, less a 10 % edge margin. The band holds at
  every operating posture (30–60°) and at ±8° of body pitch:

  | Lens | Range band 0.7–1.5 m | Engage window 0.5–1.5 m |
  |---|---|---|
  | Standard | none | none |
  | Wide | 0.20–0.58 m | 0.23–0.42 m |

  - A waist tag at 0.95 m needs the camera pitched to 39–46° with the
    Wide lens. 42° is proposed, and covers a tag at about 0.85–1.1 m.
  - With the standard lens no camera pitch works for a waist tag.
- **Bearing rate.** This is the largest bearing rate the body can catch.
  It uses the M10 yaw response (heading lag 0.205 s, 3.2 rad/s²) plus a
  0.1 s perception latency. At 1 m:
  - standard lens: 0.96 rad/s;
  - Wide lens: 1.38 rad/s.

  The requirement asks for 1.0 rad/s (a carried tag crossing at 1.0 m/s).
- **Pixels.** At 1152 px image width, the Wide lens gives 50 px across the
  tag at 1.5 m. At a 60° view angle that falls to 25 px, which is
  3.1 px per cell of tag36h11.
- **Recommendation.** The Wide lens, on both counts. Phase C makes the
  choice and checks the Wide module's 12.4 mm depth against the 11.5 mm
  envelope.

**Yaw response.** Measured from the M10 turn case (12 s, Python
controller, sensor model; `yaw_response/`):

| Measure | Value |
|---|---|
| Dead time | 0.16 s from moving; 0.30 s from rest after a stop (wheel friction) |
| 10–90 % rise | 0.11 s |
| Peak yaw acceleration | 3.2–3.5 rad/s² |
| Steady heading lag | 0.20 s |

**Requirements** (`config/following-m12.json`). These are proposals,
each with its basis:

| Item | Value |
|---|---|
| Range | nominal 1.0 m; band 0.7–1.5 m |
| Bearing, straight following | ±10° |
| Target speed | 0–1.0 m/s |
| Target acceleration | 1.0 m/s² |
| Lateral acceleration | 0.5 m/s² |
| Curves | 1 m radius at 0.7 m/s; 2 m radius at 1.0 m/s |
| Corners | 0.3 m radius at 0.5 m/s |
| Carried tag, sideways | up to 1.0 m/s |
| Engage | 5 consecutive detections, at least 0.15 s, range 0.5–1.5 m, from hold |
| Loss | coast 0.3 s on the prediction, then brake at 1.0 m/s² to hold; stopping distance ≤ 0.8 m; no search |
| Reacquisition | during the coast or braking, a detection within 0.3 m of the prediction resumes at once; from hold, the engage rule |
| Close range | reverse at ≤ 0.3 m/s; hold below 0.4 m |
| Tag | tag36h11, 0.16 m black square (0.20 m with border, an A4 sheet); view angle ≤ 60° |
| Detection resolution | ≥ 3 px per cell, 1152 px wide |

Perception latency (0.1 s) and detection rate (30 Hz) are assumptions
for phase C to replace.

**Scenarios** (`bipe/m12/scenarios.py`, `scenarios.json`).
- **Set.** Twelve deterministic scripted paths:
  - straight walks at 0.3, 0.6 and 1.0 m/s;
  - start-stop;
  - curves of 1 m and 2 m radius;
  - a 90° corner;
  - a lateral crossing with a carried tag;
  - an approach;
  - loss by leaving the field of view (a 2.5 m/s run past the robot,
    outside the requirements);
  - a short occlusion and a long occlusion (a second person).
- **Target and occluder.** Each is a person-sized mocap body with
  contype and conaffinity 0. A test steps the plant with the people
  standing in it and gets bit-identical motion to the bare M10 plant.
  The world body still holds only the ground, so slopes keep working.
- **Tag.** A worn tag faces back along the walk. A carried tag is held
  toward the robot. Its texture is a 4 × 4 checker placeholder.

**Ideal follower and phase B targets** (`bipe/m12/follower.py`).
- **Envelope run.** Perfect target knowledge, no limits and 0.2 s lags,
  over the in-requirement scenarios. Peaks and the targets rounded up
  from them:

  | Quantity | Peak | Phase B target |
  |---|---|---|
  | Speed | 1.10 m/s | 1.2 m/s |
  | Acceleration | 1.13 m/s² | 1.25 m/s² |
  | Yaw rate | 0.91 rad/s | 1.0 rad/s |
  | Yaw acceleration | 1.13 rad/s² | 1.25 rad/s² |

  The yaw peaks come from the lateral crossing. Phase B also keeps:
  - a 0.3 m/s reverse speed;
  - the 1.0 m/s² brake;
  - a quasi-static lean of up to 7.3°, which the 8° camera pitch
    allowance covers.

  Against today's validated envelope (0.3 m/s, 0.4 rad/s), phase B
  raises speed by 4× and yaw rate by 2.5×.
- **Check run.** The same scenarios with the targets as limits, the
  0.1 s latency, the 30 Hz detections, geometric visibility, and the
  engage, loss and reacquisition rules. Wide lens. The two tag layouts
  give identical results.

  | Scenario | Range (m) | Peak bearing (°) | Peak view angle (°) | Events after engage | Met |
  |---|---|---|---|---|---|
  | straight 0.3 / 0.6 / 1.0 | 0.93–1.07 / 0.87–1.13 / 0.80–1.20 | 0 | 0 | none | yes |
  | start_stop | 0.81–1.20 | 0 | 0 | none | yes |
  | curve_r1 | 0.86–1.15 | 13.6 | 60 | lost 8.38 s, reacquired 8.80 s | yes |
  | curve_r2 | 0.81–1.20 | 3.4 | 30 | none | yes |
  | corner_90 | 0.81–1.20 | 19.2 | 63 | lost 7.31 s, reacquired 7.57 s | yes |
  | lateral_crossing | 0.85–1.10 | 6.1 | 0 | none | yes |
  | approach | 0.51–1.13 | 0 | 0 | none (range above the 0.4 m stop) | yes |
  | loss_fov | 0.63–0.97 | 40.5 | 0 | lost 2.05 s, hold 3.68 s, no search | yes |
  | occlusion_short | 0.87–1.13 | 0 | 0 | lost 4.11 s, reacquired 4.17 s | yes |
  | occlusion_long | 0.87–1.36 | 0 | 0 | lost 4.05 s, hold 4.67 s (0.19 m), engaged 7.40 s | yes |

**Findings for the sign-off.**
1. **Tag height and camera pitch.** The current 15° head camera sees a
   tag only low on the person, about 0.2–0.6 m, and only with the Wide
   lens. There are two layouts:
   - **low tag:** keep the head, and wear the tag on the back of the
     leg or carry it low;
   - **waist tag:** re-pitch the camera to about 42° and wear the tag
     on a belt at about 0.95 m. That is a head CAD change in phase E.

   Both pass the scenarios. The waist tag is steadier on a walking
   person.
2. **View angle in turns.** A single tag worn on the back turns past
   60° from the robot in a 1 m curve or a 90° corner, because the
   follower cuts inside the person's path. The robot brakes briefly and
   reacquires (curve_r1, corner_90). In a steady pursuit at 1 m the view
   angle is about 46° on a 1.5 m radius and 32° on a 2 m radius (that
   is analytic; the simulated 2 m curve gives 30°). Tags on the back and
   both hips would ease the limit; that has not been simulated.
3. **Lens.** The Wide lens is needed for the tag band and for the
   1 rad/s bearing rate; the standard lens has no margin. Phase C makes
   the choice and the fit check.
4. **Phase B targets.** 1.2 m/s, 1.25 m/s², 1.0 rad/s and 1.25 rad/s².

Validation:
- `tests/test_m12_requirements.py`, 15 tests:
  - CAD camera pose against the simulated camera;
  - datasheet FOVs, and a render camera that covers them;
  - geometry identities;
  - scenario determinism and limits;
  - the target and occluder never acting on the plant;
  - the follower's engage, coast and brake to hold;
  - the record being current;
  - render repeatability.
- Full suite: 173 passed.
- The plant and controller are unchanged, so no acceptance rerun was
  needed.

Exit status: the scenarios, the phase B targets and the frames are done.
The requirements table and the tag-layout choice wait for the user's
sign-off.

User decisions at the phase A exit (20 October 2026). The user took
Claude's recommendations:
- **Requirements:** signed off as proposed (`config/following-m12.json`).
- **Tag layout:** a waist tag at about 0.95 m. The head camera is
  re-pitched from 15° to about 42° in phase E, a head CAD change. The
  low-tag layout stays in the phase A checks for comparison.
- **Turns:** in 1 m curves and 90° corners, a single tag worn on the
  back can briefly pass the view-angle limit. The robot then brakes for
  a moment and reacquires the tag. That behaviour is accepted.
- **Lens:** the Wide lens. Phase C checks that its 12.4 mm depth fits
  the 11.5 mm head envelope, and makes room if it doesn't.
- **Phase B targets:** 1.2 m/s, 1.25 m/s², 1.0 rad/s and 1.25 rad/s².
  Phase B also keeps the 0.3 m/s reverse and the 1.0 m/s² brake.

## Phase B plan: speed envelope (20 October 2026)

Goal: the M10 balance controller follows velocity and yaw-rate commands
up to the signed-off envelope under the acceptance conditions: 1.2 m/s,
1.25 m/s², 1.0 rad/s and 1.25 rad/s², with a 0.3 m/s reverse and a
1.0 m/s² brake. The 0.3 m/s acceptance must keep passing.

Starting point. A probe ran the current controller with its 0.3 m/s
command gate lifted in a scratch copy (C core, sensor model, 14 s
cases):

| Case | Peak speed (m/s) | Peak pitch (°) | Note |
|---|---|---|---|
| Step to 1.2 m/s, then stop | 1.58 | 16.7 | 32 % overshoot; plateau 1.13–1.15 m/s; the stop runs back 0.11 m |
| Ramp at 1.25 m/s², brake at 1.0 m/s² | 1.67 | 15.3 | |
| Spin ±1.0 rad/s at rest | — | 0.9 | clean |
| Arc, 1.0 m/s at 0.5 rad/s | 1.37 | 14.2 | |
| Weave, 0.9 m/s at ±0.9 rad/s | — | 13.3 | |

No case fell, saturated a wheel or exceeded about 0.08 N·m of wheel
torque, out of the 0.35 N·m left at 1.2 m/s. So the envelope is
reachable. The problem is tracking:
- **Speed loop.** It leans only from the velocity error, so the speed
  lags the reference and the integral winds up during the ramp.
- **Stop.** A stop holds the position where the command was given.
  At speed that point is far behind the stopping point, so the robot
  drives back to it.
- **Estimator.** During a sustained acceleration the accelerometer
  sees the lean plus the acceleration. The attitude correction reads
  that as level, so the pitch estimate drifts about 2.5° low. The
  IMU-fused velocity then reads about 0.1 m/s low, and the controller
  keeps accelerating. With ideal sensing, the same controller with the
  step 2 changes peaks at 1.29 m/s instead of 1.50 m/s.

Margins already thin at 0.3 m/s (the recorded M10 acceptance):
- the drive case overshoots to 0.43 m/s;
- plateau error 0.027 m/s against the 0.03 m/s limit;
- stopping travel 0.084 m against the 0.1 m limit;
- `delay_15ms_noise_x2_combined` fails (pitch 20° at 3.4 s), a recorded
  M10 shortfall.

A pre-existing gap found on the way: the acceptance variant
`wheel_lag_25ms` sets `motor.torque_time_constant`, but the M10 hub
drives read `physics.hub_motor.FOC_time_constant_s`. On M10 the variant
therefore does nothing; its results equal `sensors_drive` to the last
digit. Phase B applies the lag through the hub FOC constant in its own
cases. Fixing the inherited acceptance job is a user decision, because
it changes a recorded M10 result.

Steps:
1. **Command envelope.**
   - Add `m10.targets.drive_envelope` to `config/robot.json`: the
     signed-off speeds, accelerations and yaw values, with their basis.
   - `m10.plant.parameters` sets `sizing.drive_speed` from it, so the
     firmware command gate (`BIPE_DRIVE_SPEED` in `balance.c` and
     `application.c`) becomes 1.2 m/s. `tools/generate_firmware_config.py`
     stays unchanged.
   - The 1.0 rad/s yaw gate already equals the target and stays as it is.
2. **Reference shaping and lean feedforward** (Python and C, parity).
   - A jerk-limited velocity reference (`command_jerk_m_s3`). Its
     acceleration slews toward the largest value that still stops at
     the target, so the reference never overshoots and its lean stays
     continuous. `max_command_acceleration` drops from 3 to 1.25 m/s².
   - A lean feedforward of atan(a/g) on the reference acceleration
     (`acceleration_feedforward`, a gain on that lean).
   - A stop brakes the reference to rest at the acceleration limit, and
     the hold position is latched there. Drive gains apply until rest.
   - Both keys default to off, so the M4 and M9 profiles are unchanged.
     The Python/C equivalence samples extend to the envelope speed.
3. **Estimator under acceleration** (serial profile with IMU fusion;
   Python and C, parity).
   - Take the IMU point's forward acceleration from the wheel odometry
     already used by the fusion, low-passed with the fusion time
     constant.
   - Subtract it from the accelerometer before the attitude correction
     and its norm gate. That is the kinematic model the fusion's
     prediction step already assumes.
   - No new configuration key is needed.
4. **Tuning** (`artifacts/m12/speed/tuning/`).
   - Sweep the jerk limit, the feedforward gain and, if needed, the
     speed-loop gains.
   - Score them on the fast step and stop, and on the 0.3 m/s
     acceptance drive.
   - Save each candidate with its metrics.
5. **Speed gates** (`bipe/m12/speed.py`, `config/speed-m12.json`).
   These run on the C core and sensor model. The cases are:
   - drive: a step and a ramp to 1.2 m/s, and a stop at 1.0 m/s²;
     reverse at 0.3 m/s;
   - spin ±1.0 rad/s at rest;
   - arcs: 2 m radius at 1.0 m/s, 1 m radius at 0.7 m/s; a weave at
     speed;
   - replays of the phase A ideal-follower commands for the waist-tag
     layout (straight walk at 1.0 m/s, start-stop, 1 m curve, 90°
     corner, lateral crossing, loss);
   - ±2 N·s pushes at 1.2 m/s, which must survive and recover;
   - 4°, 8° and 10° slopes, up and down at 1.0 m/s;
   - variants on the fast cases: mass +15 %, friction 0.5, a 25 ms
     wheel lag through the hub FOC constant, 15 ms delay with ×2 noise,
     two more sensor seeds;
   - a half-timestep convergence check on a fast case;
   - for information only, the low posture at speed.

   Proposed criteria, in the config and for the user's sign-off at the
   exit:
   - no physical failure (the acceptance limits: 20° pitch, 10° roll,
     contact, joint limits);
   - speed overshoot and plateau error;
   - stopping distance within v²/2a plus an allowance;
   - yaw-rate error and heading error at the acceptance limits;
   - recovery after pushes and stops.
6. **Electrical check.**
   - Run the M11 `ElectricalRecorder` on the fast cases.
   - Check hub and driver current against the GL40 and moteus limits,
     and the voltage demand against the 18.24 V drive limit.
   - Report bus current and runtime against the M11 pack and fuse.
7. **Falls at speed.**
   - `bipe/m10/falls.py` gains a `--speeds` option, and the M12 output
     goes to `artifacts/m12/falls`.
   - Run unpowered tips forward, backward and sideways at 1.2 m/s. The
     M10 requirement holds: forward and backward factor ≥ 2, sideways as
     containment.
8. **Regression.**
   - Rerun the full M10 acceptance and robustness suite with the new
     controller and estimator, under `nice`, one suite at a time.
   - Compare with the recorded results: no new failures, with the
     0.3 m/s drive margins reported.
   - Rerun the M11 electrical requirements, then check the M11 part
     ratings against them.
9. **Records and tests.**
   - Outputs: `artifacts/m12/speed/` (configuration, metrics, traces,
     plots).
   - Tests: the shaper (no overshoot, continuity, rest latch), C/Python
     parity at speed, the drive envelope reaching the firmware gate,
     and the record being current.
   - Docs: the plan results and the README.

Exit:
- the speed gates pass;
- falls at 1.2 m/s meet the M10 requirement;
- the electrical demand stays within the ratings;
- the M10 acceptance is unchanged or the changes are explained.

Any shortfall is recorded for the user's sign-off.

## Phase B results (20 October 2026)

Evidence: a validated simulation under the stated assumptions. Every
case runs the C core with the sensor model, a free base, finite motor
limits and physical contact. Nothing is hardware-validated. The records
are in `artifacts/m12/speed/`, `artifacts/m12/falls/` and
`artifacts/m12/falls_speed_sweep/`; the speed gates are regenerated by
`.venv/bin/python -m bipe.m12.speed`.

**Controller** (`bipe/locomotion.py`, `firmware/src/balance.c`,
`config/controller_m10.json`; Python and C agree).
- **Envelope.** `config/robot.json` `m10.targets.drive_envelope` holds
  the signed-off values. `m10.plant.parameters` sets `sizing.drive_speed`
  from it, so the M10 firmware command gate is 1.2 m/s. Other profiles
  keep 0.3 m/s. The yaw gate stays 1.0 rad/s.
  `tools/generate_firmware_config.py` is unchanged.
- **Reference shaping.** The drive velocity reference is jerk limited
  (10 m/s³) at the 1.25 m/s² envelope acceleration. It never overshoots
  its target, and its acceleration is continuous.
- **Lean feedforward.** The lean reference adds atan(a/g) of the
  reference acceleration (gain 1.0).
- **Stop.** A stop brakes the reference to rest at the acceleration
  limit, then latches the hold position there. Drive gains stay on
  until rest. Before, the hold was latched where the stop was
  commanded. At speed that point lies behind the stopping point, so the
  robot drove back to it: 0.11 m in the probe, and 0.44 m once the
  reference was shaped.
- **Hold.** While holding, the velocity reference is the output of the
  position loop. It keeps the pre-phase-B 3 m/s² rate limit, with no
  jerk limit and no feedforward (`hold_command_acceleration`). See
  "Wheel lag" below for why.
- **Defaults.** The new C keys default to off, so the M4 and M9
  profiles are unchanged.

**Estimator under acceleration** (`bipe/sensing.py`,
`firmware/src/estimator.c`; serial profile with IMU fusion).
- **Change.** The odometric forward acceleration, low-passed with the
  fusion time constant, is removed from the accelerometer before the
  attitude correction and its norm gate. No new configuration key.
- **Only for a drive acceleration.** The removed part is clipped to
  `compensation_limit`, an estimator input the caller sets after each
  control step. It is the drive acceleration limit (1.25 m/s²) while
  driving (a drive command or a brake to rest) and 0 while holding.
  Python and the C application set it the same way.
- **Effect.** A synthetic 1.25 m/s² acceleration held for 3 s keeps the
  pitch estimate within 0.5° of the true lean; without the change it
  drifts more than 3° toward level. This was the main cause of the
  probe's overshoot: with the old estimator the shaped controller still
  reached 1.50 m/s; with the compensation, 1.30 m/s.
- **Why it is limited.** Applied at all times, the compensation lost
  the M10 acceptance push `push_low_+2.5` (it fell at 4.5 s; before
  phase B it survived at 18.51°). In a push the wheels accelerate hard
  while the body tips, so the axle's acceleration is not the IMU's.
  The records are in `artifacts/m12/speed/regression/` (Python
  controller):

  | Rule | Pushes at rest | Pushes in a drive (peak) | Drive step overshoot, replay path |
  |---|---|---|---|
  | None (pre-phase-B estimator) | push_low_+2.5 survives, 18.51° | 17.20–18.80° | 1.50 m/s peak (probe) |
  | At all times (`push_low.json`) | push_low_+2.5 falls; others +0.6–1.5° | 18.78–19.47° | 0.105, 0.171–0.198 m |
  | Both commanded and observed (`push_low_agreed.json`) | as before phase B | — | 0.146, a replay over 0.2 m |
  | While driving (`push_low_driving.json`) | as before phase B | 18.78–19.47° | as at all times |
  | While driving, clipped to 1.25 m/s² (`push_low_bounded.json`, adopted) | as before phase B | 18.32–18.57° | as at all times |
  | While driving, clipped to 0.75 m/s² (`compensation_bound.json`) | — | 18.08–18.49° | 0.149, a replay 0.222 m |

- **Cost.** Holding uses no compensation, so after a stop on a slope
  the robot creeps back to the hold point slightly faster. slope_up_8
  now peaks at 0.032 m/s in the last 2 s (0.024 before), and joins
  slope_up_4 as a shortfall (below).

**Tuning** (`artifacts/m12/speed/tuning/`, 40 runs in five sweeps, on
the Python controller).
- **Selected.** Jerk 10 m/s³, feedforward 1.0 (`r4 j10f1`): the 1.2 m/s
  step peaks at 1.31 m/s (8.8 %) and plateaus within 0.010 m/s. The
  0.3 m/s acceptance drive passes with a 0.005 m/s plateau error and
  0.093 m stopping travel (limit 0.1 m).
- **Rejected.** Jerk 7.5 or less fails the 0.1 m stop at 0.3 m/s. Jerk
  20, or feedforward 1.2, overshoot about 14 % at 1.2 m/s.
- **Note.** The sweep ran before the hold change. The hold change does
  not touch a drive or its stop, and the acceptance drive gives the
  same 0.093 m with it.

**Speed gates** (`config/speed-m12.json`, 44 cases). The criteria are
proposed, for sign-off:
- 5° pitch margin to the 20° failure limit;
- overshoot at most 0.15 m/s;
- plateau error at most 0.05 m/s;
- yaw-rate error at most 0.08 rad/s and heading error at most 8°;
- a stop at most 0.15 m past the shaped reference's stopping distance;
- a replay path within 0.2 m of the ideal follower's;
- recovery as in the acceptance (last 2 s within 2° and 0.03 m/s);
- slopes must survive and hold (last 2 s within 0.03 m/s).

Each value's basis is in the config.

| Case | Peak pitch (°) | Peak speed (m/s) | Result |
|---|---|---|---|
| Step to 1.2 m/s and stop | 13.6 | 1.31 | overshoot 0.107, plateau 0.007, stop +0.082 m |
| Ramp 1.25 m/s², brake 1.0 m/s² | 13.3 | 1.31 | overshoot 0.108, stop +0.053 m |
| Reverse 0.3 m/s | 10.8 | 0.41 | overshoot 0.109 |
| Spin ±1.0 rad/s | 0.9 | 0.03 | yaw error 0.001 rad/s |
| Arc 2 m at 1.0 m/s | 13.2 | 1.11 | plateau 0.038, stop +0.099 m |
| Arc 1 m at 0.7 m/s | 12.3 | 0.79 | plateau 0.032 |
| Weave ±0.8 rad/s at 1.0 m/s | 13.2 | 1.10 | yaw error 0.064 rad/s |
| Push +2 N·s at 1.2 m/s | 17.4 | 2.64 | survives and recovers |
| Push −2 N·s at 1.2 m/s | 18.7 | 1.36 | survives and recovers |
| Slopes up 4°/8°/10° at 1.0 m/s | 16.0/18.4/19.6 | 1.13 | rollback 0.029/0.042/0.058 m; 4° and 8° miss the hold speed (below) |
| Slopes down 4°/8°/10° | 13.2/15.7/16.9 | 1.09 | hold |
| Follower replays (waist tag) | 8.6–12.8 | 1.26 | path error 0.032–0.198 m |
| Low posture ramp (information) | 16.6 | 1.37 | overshoot 0.165 |

- **Variants.** Every variant passes on the step, weave and 90° corner
  replay except the 25 ms wheel lag: mass +15 %, friction 0.5, 10 ms
  wheel lag, 15 ms delay with ×2 noise, two more sensor seeds. With the
  other seeds the corner replay's path error is 0.236 and 0.265 m;
  variants are judged on survival only.
- **Convergence.** At half the timestep the step differs by 0.013° in
  peak pitch and 0.0004 m/s in peak speed.
- **Margins.**
  - The slopes keep less than the 5° pitch margin. They are judged by
    survival, as in M10, and slope_up_10 peaks 0.4° short of the
    failure limit.
  - The fast pushes keep 1–2° of margin.

**Electrical** (the M11 recorder on every case).
- **Nominal cases.**
  - Hub phase current at most 0.94 A (slope_up_10), against the GL40's
    3.3 A peak.
  - Hub voltage demand at most 10 V, against the 18.24 V drive limit.
  - Leg phase current at most 1.91 A.
  - Bus current at most 2.4 A against the 10 A fuse; mean bus power
    23–33 W, so 28–40 min of runtime on the M11 pack.
- **Fast push +2 N·s.** The hub rotor reaches 76 rad/s. Its voltage
  demand of 19.5 V is above the 18.24 V limit for 0.15 % of the run.
  The drive limits it and the case recovers. Bus current peaks at
  5.1 A.

**Falls at speed** (unpowered tips, M10 requirement: forward and
backward factor at least 2 on every path, sideways as containment).

| Initial speed (m/s) | Forward factor | Backward factor | Sideways containment |
|---|---|---|---|
| 0.3 (M10 record) | 2.36 | 2.50 | holds |
| 0.6 | 1.86 | 1.81 | holds |
| 0.9 | 1.50 | 1.48 | open: head-front shell travel |
| 1.2 | 1.46 | 1.50 | open: head-front shell travel |

- **Limiting path.** Hip-belt slack. From 0.9 m/s the hip belts also
  pass their rating, and forward the skid-pad bumper bottoms. At
  1.2 m/s backward the head-front shell exceeds its travel.
- **Crossing.** Factor 2 is crossed near 0.45 m/s.
- **Energy.** No case injects more than 0.0015 J.
- **Controller.** The falls are unpowered, so the controller does not
  enter them.

**Wheel lag.** The inherited M10 acceptance variant `wheel_lag_25ms`
never applied on M10 (phase B plan); it is fixed at the phase B exit
(below). Applied through the hub FOC constant, it gives
(`artifacts/m12/speed/lag_check/`):

| 0.3 m/s acceptance case | Before phase B, 25 ms | Phase B, 25 ms | Both, 10 ms |
|---|---|---|---|
| Drive | falls at 7.2 s | falls at 7.2 s | pass |
| Combined | falls at 15.9 s | falls at 4.0 s | pass |
| Push at rest | recovers (3.7°) | recovers (3.7°) | pass |

- **Push at rest.** The first phase B controller also shaped the hold
  reference. It lost the 25 ms push at rest: the robot did not recover
  and reached 2.4 m/s. `attribution.json` puts the loss on the
  reference shaping, not on the estimator change. `hold_variants.json`
  narrows it to the lean feedforward on the hold reference: the jerk
  limit alone keeps the push. The hold therefore keeps its pre-phase-B
  rate limit.
- **Cost of the hold choice.** slope_up_4 returned to its hold point
  at 0.036 m/s; with the hold shaped it was 0.029 m/s. (With the
  estimator compensation now off while holding, 0.041 m/s.)
- **At speed.** With a 25 ms lag the robot falls in all three fast
  cases. With 10 ms it passes them all. The modelled moteus FOC lag is
  0.8 ms.

**M10 acceptance** (rerun in full with the phase B stack, 21 October
2026; `artifacts/m10/acceptance/`, compared with the phase A record by
`artifacts/m12/speed/regression/compare.py 77e764c` into
`regression/acceptance.json`).
- **No new failures.** The robustness set passes 70 of 70. The
  inherited set passes 26 of 27: `delay_15ms_noise_x2_combined` falls,
  as it did before phase B. Every acceptance target keeps its value,
  and convergence passes.
- **Wheel lag variant fixed** (user decision, 21 October 2026). The
  inherited `wheel_lag_25ms` variant set `motor.torque_time_constant`,
  which the M10 hub drives don't read; `bipe/moving_validate.py` now
  also sets `physics.hub_motor.FOC_time_constant_s`. Rerun in full and
  compared with the record above
  (`regression/wheel_lag_fix.json`): only the two lag cases change.
  `wheel_lag_25ms_drive` falls at 7.2 s and `wheel_lag_25ms_combined`
  at 4.0 s, as in the lag check, so the inherited set now passes 24 of
  27. The controller before phase B also falls in both (7.2 and
  15.9 s, `lag_check/before_phase_b.json`), so these are inherited M10
  gaps, not phase B regressions.
- **Improved.** `push_drive_+2.5` and `delay_15ms_noise_x2_drive` now
  recover; before, they survived without recovering. The 0.3 m/s drive
  plateau error falls from 0.027 to 0.005 m/s, and its stopping travel
  is 0.081 m (0.084 before; limit 0.1 m).
- **Pushes at rest** are unchanged to the last digit (push_low_+2.5
  18.51°, push_hold_+2.5 18.31°): holding uses no compensation and the
  plain rate limit, as before phase B.
- **Higher peak pitch on drive starts.** Drive and combined cases peak
  2–3° higher; slope starts up to 4.6° higher (slope_up_10 17.6°
  against 13.1°, a 2.4° margin to the 20° limit). The cause is the lean
  feedforward: at the start of a drive the shaped reference reaches the
  full 1.25 m/s², so the robot leans an extra atan(1.25/g) = 7.3°.
  `regression/acceptance_pitch.json` (Python controller) attributes it:
  without the feedforward slope_up_10 peaks at 11.9°, below phase A;
  without the compensation it is unchanged (17.7°).

**M11 electrical requirements** (rerun on the M10 acceptance cases;
`artifacts/m11/requirements/`, then `bipe.m11.power` into
`artifacts/m11/power/report.json`). The committed records predated the
M11 plant update (body mass, geometry, 18.24 V drive limit), so the
changes below mix that update with phase B.
- **Ratings.** All 11 power checks pass: regulator, inrush, LM5060
  timer and trip, MOSFET gate and linear mode, fuse peak, RMS and
  interrupting rating, driver minimum voltage, battery monitor range.
- **Changes.** Hip phase current peak 3.41 → 3.56 A and hip driver
  power 24 → 26 W (push_drive_-2.5). Hub voltage demand peak
  20.7 → 20.0 V (push_hold_-2.5), at the limit 0.3 % of the time
  instead of 1 %. The 10 ms bus peak falls from 6.4 to 6.3 A.
- **Runtime.** The 0.3 m/s drive gives 31.8 min (33.6 before); the
  lowest case, push_low_-2.5, gives 26.0 min (26.3 before).

**Phase B exit (21 October 2026): for the user's sign-off.** Every
speed gate passes except the recorded shortfalls. The M10 acceptance
has no new failures, and its changes are explained above. Decisions:
1. **Estimator compensation only while driving, clipped to the drive
   acceleration limit.** It restores every push at rest. Pushes during
   a drive peak up to 1.2° higher than with no compensation (18.57°
   at most), all recovering.
2. **Speed criteria** as proposed in `config/speed-m12.json`.
3. **Shortfalls** (`config/speed-m12.json` `recorded_shortfalls`):
   - The 25 ms wheel lag: the three speed cases and, with the variant
     fixed, the M10 acceptance `wheel_lag_25ms_drive` and
     `wheel_lag_25ms_combined`. This is an inherited M10 gap: the
     controller before phase B also falls at 0.3 m/s. The modelled
     moteus FOC lag is 0.8 ms, and 10 ms passes everywhere.
   - slope_up_4 and slope_up_8 creep back to the hold point at 0.041
     and 0.032 m/s against the proposed 0.03 m/s. They meet M10's 0.1 m
     rollback limit.
4. **Margins kept below 5°.**
   - slope_up_10 at 1.0 m/s peaks at 19.6° (0.4° margin); the slopes
     are judged on survival, as in M10.
   - The acceptance slope_up_10 at 0.3 m/s peaks at 17.6° (2.4°), from
     the lean feedforward.
5. **Unpowered falls** below factor 2 above about 0.45 m/s (1.46 at
   1.2 m/s, limited by hip-belt slack). The M10 requirement holds only up
   to about 0.45 m/s.
6. **Wheel lag variant.** Fixed at the user's request; it adds the two
   acceptance failures above (inherited 24 of 27).

User sign-off (21 October 2026): phase B is signed off as recorded
above, with the six decisions, the speed criteria and the recorded
shortfalls accepted. Phase C starts.

## Phase C plan: perception (21 October 2026)

Goal: an AprilTag measurement of the waist tag (range, bearing and
dropouts) taken from frames the simulation renders from the head
camera, with a Pi Zero 2 W timing estimate and its basis, and a measurement
model for phase D to design against.

Starting point (signed off in phase A):
- tag36h11 proposed, 0.16 m black square (0.20 m printed, A4), worn as a
  waist tag at 0.95 m;
- Camera Module 3 Wide (about 102° × 67°), 1152 px wide (half of the
  2304 × 1296 mode), camera re-pitched to about 42° (a CAD change in
  phase E; phase C renders at 42° and keeps the 15° low-tag layout for
  comparison);
- assumptions to replace: 0.1 s perception latency and 30 Hz detection.

Steps:
1. **Camera model** (`bipe/m12/camera.py`).
   - Intrinsics from the Wide lens datasheet: focal length, sensor
     size, the 1152 × 648 output after binning. Sources recorded.
   - MuJoCo renders a pinhole image. The Wide lens is strongly
     distorted, so a fisheye or Brown-Conrady distortion model is
     applied by remapping the rendered frame (rendered over a wider
     pinhole field so that no edge is lost). Its coefficients are
     an assumption until a real calibration exists.
   - Exposure and motion blur: average sub-frame renders over the
     exposure time (rolling shutter as a later option), at the yaw rates
     and body pitch rates from the phase B speed records.
   - Noise: shot and read noise at an indoor light level. Every value is
     an assumption with its basis.
2. **Tag in the world.** Replace the phase A placeholder with real
   tag36h11 images on the target's waist plane, at the signed-off size,
   with a matte material.
3. **Detector** (choice for the user, with price, licence and
   maintenance):
   - `pupil-apriltags` (Python bindings to the AprilRobotics C library,
     BSD-2). It is the same C code the Pi would run, so the sim tests
     the real detector. Proposed.
   - OpenCV ArUco's AprilTag mode. It is an alternative, and it is
     slower and less accurate on small tags.
   - The detector settings to sweep: `quad_decimate` (1, 1.5, 2),
     threads, refinement, region of interest from the last detection.
   - The tag pose comes from the homography with the camera
     intrinsics; range and bearing are taken from it in the robot
     frame.
4. **Accuracy on rendered frames** (`artifacts/m12/perception/`).
   - A static grid over range (0.5–1.5 m), bearing (to the field-of-view
     edge less the 10 % margin), view angle (0–60°) and body pitch.
     Each point gives the detection rate, and the range and bearing
     errors against simulator truth.
   - The phase A scenarios replayed with the phase B controller in the
     loop and the motion blur on, giving dropout statistics along real
     paths.
   - Measure the detector's view-angle limit, which phase A left to
     phase C.
5. **Pi Zero 2 W timing estimate.**
   - Time the detector on this machine at each setting, scaled to the
     Zero 2 W's Cortex-A53 at 1 GHz using published benchmarks of the
     same library. This is an estimate with its basis, not a
     measurement, and the README says so.
   - The latency budget: exposure, ISP and transfer (from Raspberry Pi
     documentation), detection, and the link to the Teensy (an
     assumption until phase E).
   - If 30 Hz at 1152 px does not fit, the fallbacks are a smaller
     image with an ROI search, a lower rate, or a faster vision host
     (for example a Raspberry Pi 4 class board, which changes the mass,
     power and packaging; a decision for the user, with price and
     stock).
6. **Measurement model** (`config/perception-m12.json`).
   - Bias and noise for range and bearing as functions of range and
     view angle, the detection probability, the latency, and the rate.
   - Checked against held-out rendered runs: the model's statistics
     agree with the rendered results within stated bounds.

Exit: detector accuracy and rate on rendered frames, the Pi Zero 2 W
timing estimate with its basis, and the measurement model. The lens
fit in the head (its 12.4 mm depth) is confirmed against the CAD.

Decisions for the user at the phase C exit: the detector, the image
size and rate, and any vision-host change.

Evidence: perception results measured on rendered frames only. No real
footage exists yet, and the distortion and noise are assumptions.

## Phase C results (21 October 2026)

Evidence: everything below is measured on frames the simulation
renders, under the camera, light and noise assumptions in
[config/perception-m12.json](../../config/perception-m12.json). No real
footage exists. The Pi Zero 2 W timing is an estimate scaled from this
machine, not a measurement. Outputs, configs and traces are in
[artifacts/m12/perception/](../../artifacts/m12/perception/). Rebuild with
`.venv/bin/python -m bipe.m12.perception` (about 25 minutes on 8 workers);
`--finish` rewrites the summaries, the model and the plots from the
saved samples.

### Camera model (`bipe/m12/camera.py`)

- **Intrinsics.** Camera Module 3 Wide: 2.75 mm focal length and
  1.4 µm pixels (Raspberry Pi camera documentation). The 1152 × 648
  output is the 2304 × 1296 binned mode scaled by half, so one output
  pixel is 5.6 µm and f = 491.1 px. The sources are recorded in
  `config/robot.json` (`m12.camera_sensor`) and the artifact config.
- **Distortion.** Two-term radial (Brown-Conrady) k1 = 0.0031,
  k2 = −0.0236, fitted exactly to the documented 102° × 67° fields.
  That gives a 113.9° diagonal against the 120° on the product page.
  A three-point fit that included the diagonal was not monotonic over
  the image, so it was rejected. The model is an assumption until a
  real calibration exists.
- **Rendering.** MuJoCo renders a wider pinhole image (2630 × 1480,
  73.99° vertical, two render pixels per output pixel at the centre),
  which is remapped through the distortion to the output.
- **Optics and sensor.**
  - A Gaussian of 0.29 px (pixel aperture) and 0.5 px (lens and
    scaler, assumed).
  - Shot and read noise from an electron budget at 200 lx (50 lx as
    the dim case), f/2.2, QE 0.3 and 2 e− read noise.
  - The sRGB tone curve, then 8 bits.
- **Motion blur.** The mean of 5 sub-frames over a 10 ms global-shutter
  exposure. Rolling shutter is not modelled.
- Every value carries its basis in the config.

**Tag.** Real tag36h11 (id 0), 0.16 m black square, on the target's
waist plane. The tag is built from geometry: matte white board and
black cell boxes 0.2 mm proud. A textured mesh rendered the cells 1–4
render pixels out of place, so it was replaced. Edges in the render now
match the projection to 0.02 px. The camera is pitched to 42° for the
waist tag, and the 15° low-tag layout is kept for comparison.

### Detector

pupil-apriltags 1.0.4 (MIT-licensed bindings to the AprilRobotics C
library, BSD-2, free; the same C code the Pi would run). Settings:
refine_edges on, decode_sharpening 0.25, max_hamming 0 (no corrected
bits accepted).

The pose is computed from undistorted corners: a homography, then a
Levenberg-Marquardt reprojection fit. Range and bearing are taken in
the levelled heading frame using the simulator's attitude.

Two modes are proposed:
- **Tracking:** a region of interest (twice the last tag box plus
  24 px) at full resolution.
- **Acquisition:** the full frame at quad_decimate 2, used when
  tracking misses.

OpenCV ArUco was not measured, because it would need a download that
was not approved.

### Accuracy grid (static)

Waist and low layouts. Range 0.5–2 m, bearing at the centre, half way
and the frame edge less 10 % (3 positions), view angle 0–80°, body
pitch ±8°. 4 samples per point plus 2 held out; 5292 frames in total.

**Detection rate, waist tag, full resolution (dec 1):**

| range (m) \ view | 0–45° | 60° | 70° | 80° |
|---|---|---|---|---|
| 0.5 | 1.00 | 0.98 | 0.48 | 0 |
| 0.7 | 1.00 | 0.80 | 0.39 | 0 |
| 1.0 | 1.00 | 0.74 | 0.50 | 0 |
| 1.25 | 1.00 | 0.85 | 0.54 | 0 |
| 1.5 | 1.00 | 0.85 | 0.63 | 0 |
| 1.75 | 1.00 | 0.87 | 0.65 | 0 |
| 2.0 | 1.00 | 0.91 | 0.57 | 0 |

- At quad_decimate 2 the rates are the same up to 45°, and lower at
  60–70°.
- The low tag holds 1.0 at 60° out to 1 m, and 0.67 beyond.
- **View-angle limit** (90 % detection): 45° for the waist tag at most
  ranges, 60° at 0.5 and 2 m. Phase A assumed 60°.
- **The misses at 60–70° are at the frame edge.** At the edge bearing
  the waist tag is detected 0.60–0.70 of the time, pooled over all
  views. The centre and half-way positions stay at 100 % at 60°. The
  Wide lens's distortion bends the tag's edges near the frame edge.
- **Out of frame:** 42 samples per layout, all at 0.5 m with +8° body
  pitch at the edge bearing. Phase A's rectangular field-of-view
  assumption overstates the corner coverage of this lens.

**Errors, waist tag, dec 1 (mean / standard deviation):**

| range (m) | range error, 0° view (cm) | range error, 45° (cm) | bearing error, 0° (deg) |
|---|---|---|---|
| 0.5 | +0.44 / 0.07 | +0.56 / 0.09 | −0.108 / 0.008 |
| 1.0 | +1.11 / 0.23 | +1.40 / 0.31 | −0.075 / 0.006 |
| 1.5 | +2.04 / 0.46 | +2.31 / 0.58 | −0.063 / 0.005 |
| 2.0 | +3.26 / 0.74 | +3.34 / 0.88 | −0.057 / 0.005 |

- Range reads about 1–2 % long. Blur spreads the black square's edge
  in linear light, and the sRGB tone curve then shrinks the square. The
  measurement model carries this as a bias rather than hiding it.
- At 70° the range bias falls towards zero and its spread rises to
  1.4–2.2 cm.
- Bearing errors are under 0.11° throughout.

### Motion blur

Blur set: waist tag at 0.7, 1.0 and 1.5 m; yaw rates 0–1.4 rad/s;
pitch rates 0 and 1 rad/s; exposures 5, 10 and 20 ms.

- At **20 ms** the tag is lost with any pitch rate, and at
  ≥ 1.0–1.4 rad/s yaw.
- At **10 ms** every case is detected up to 1.4 rad/s at 0.7–1.0 m.
  At 1.5 m it is lost at 1.4 rad/s, and partly lost with pitch motion.
- At **5 ms** every case is detected.
- **Yaw blur lengthens the range reading.** The extra bias is fitted as
  1.18 × 10⁻³ m × (blur px)² × (range m)², where blur px = bearing rate
  × exposure × f (rms residual 0.4 cm, 84 detections, up to 9.8 px).
  At 1 m and 1 rad/s with 10 ms this is about +2.8 cm on top of the
  static +1.1 cm. Pitch blur adds under 0.15 cm.
- The 10 ms cap stands. Phase D should either correct this bias using
  the gyro or allow for it.

### Scenario replays

The 12 phase A scenarios, replayed with the C core and sensor model
under the phase A follower commands. The robot path matches the
phase B record to 2.5 × 10⁻⁷ m. Frames are rendered at 30 Hz with
motion blur, and every frame is run through every detector mode.

**Waist tag, region-of-interest tracking with full-frame fallback**
(expected frames are those in view, unoccluded, within 60° and 2 m):

| scenario | detection rate | longest miss (frames) |
|---|---|---|
| straight 0.3 / 0.6 / 1.0, start_stop, curve_r2, corner_90, lateral_crossing, approach, occlusion_short | 1.000 | 0 |
| curve_r1 | 0.998 | 1 |
| loss_fov (86 of 283 frames expected) | 0.988 | 1 |
| occlusion_long | 0.996 | 2 |

- Pooled over 5945 expected frames: 0.9993. Range error +1.40 cm mean,
  0.32 cm SD, 5.5 cm worst. Bearing −0.074° mean, 0.006° SD.
- After a miss, the next frame was also missed 1 time in 3.
- The low tag gave 0.998–1.000.
- The worst range errors (4–5.5 cm) occur in loss_fov, as the target
  leaves the frame edge at 35–55° bearing while the robot turns.

### Timing (host measured, Pi Zero 2 W estimated)

Host: Ryzen 7 260, median of 30 replay frames.

| mode | host (ms) | Pi Zero 2 W estimate (ms) | Pi rate, one core |
|---|---|---|---|
| full frame dec 1, 1 thread | 75.1 | 2191–3017 | 0.3–0.5 Hz |
| full frame dec 2, 1 thread | 18.9 | 553–761 | 1.3–1.8 Hz |
| full frame dec 2, 4 threads | 6.4 | 188–259 | 3.9–5.3 Hz |
| region of interest, 1 thread | 1.41 (p95 2.01) | 41–57 | 18–24 Hz |
| pose (C library) | 0.24 | 7–10 | |

**Basis for the scaling.** Geekbench 6 single-core:
- Host: 2451.
- Zero 2 W: no entry was found. Its RP3A0 is the Pi 3's Cortex-A53, so
  the Pi 3 and 3 B+ scores, scaled to 1.0 GHz, give 61–84.
- The ratio is therefore 29–40.

Cross-check: PhotonVision users report 13–15 fps at 640 × 480 on a
Pi 3B+. **No published Zero 2 W benchmark of this library was found**,
which departs from the plan's wording. The scaling is a bracket, not a
point.

**Latency** (middle of the exposure to the Teensy):

| component | value |
|---|---|
| half exposure | 5 ms |
| readout | 17.9 ms |
| ISP | 33.3 ms (one frame, assumed) |
| detection (95th percentile) and pose | ROI: 66–91 ms; full frame dec 2, 4 threads: 204–281 ms |
| link | 2 ms (assumed) |
| **total** | **ROI: 0.12–0.15 s; full-frame acquisition: 0.26–0.34 s** |

Phase A assumed 0.1 s.

The timing figures here come from the record regenerated at the
sign-off, after the config status text changed its hash. Host times
vary by a few per cent between runs. The first run gave 0.25–0.33 s for
the full-frame acquisition and 18–25 Hz for one core; the
region-of-interest latency is unchanged.

**Rate.** One core cannot hold 30 Hz even with the region of interest
(18–24 Hz). Handing successive frames to 2 of the 4 cores reaches
30 Hz throughput, with each frame still taking the latency above.
Re-acquiring with a full frame takes 6–8 frame times.

### Measurement model (`config/perception-m12.json`, `measurement_model`)

The model is a table over range (0.5–2 m) × view angle (0–80°),
interpolated bilinearly. Each entry holds:
- the detection probability for tracking (dec 1) and for acquisition
  (dec 2);
- the range bias and standard deviation;
- the bearing bias and standard deviation.

It also carries:
- the yaw-blur range term above;
- the latency (0.12–0.15 s) and the rate (30 Hz with frames over
  2 cores);
- the replay dropout statistics.

It does not include lens calibration error, levelling error, rolling
shutter, ISP sharpening, real light, glare or print flatness. The
spreads are therefore a floor for rendered frames.

**Check against held-out runs**
([model_check.json](../../artifacts/m12/perception/model_check.json)):

- **Grid held-out samples: 208 of 209 checks pass.** At 1.0 m and 60°,
  12 of 18 held-out samples were detected (0.67) against a modelled
  0.78; the tolerance is ±0.1.
- **Replays (region of interest), after the blur term: 48 of 55 checks
  pass.** Every bias check passes. The failures:
  - **Detection at 1.0 m, 60°:** 0.98 in the replays against 0.78 in
    the model. The model is conservative here, because the replays
    rarely reach the frame edge.
  - **Spread too small (model narrower than replays), 2 bins:**
    - 0.7 m, 0°: 0.38 cm in the replays against 0.13 cm modelled;
      95th percentile residual 0.39 cm.
    - 1.0 m, 45°: 0.69 cm against 0.32 cm; 95th percentile 1.2 cm.
    - These are heavy tails from frames at the frame edge during turns
      (loss_fov, corner_90). The bulk of the distribution fits.
  - **Spread too large (model wider than replays), 4 bins** (1.0 m at
    0° and 30°, 1.25 m at 15° and 30°): the replays are 0.43–0.47 of
    the model. Each grid bin pools bearings, body pitch and positional
    jitter, while a replay bin covers a narrower set of positions.
- These are recorded shortfalls; the bounds (ratio 0.5–2) were not
  changed. For phase D: use the table's standard deviation as the
  nominal value, and design against the 95th-percentile residual
  (≤ 1.5 cm at ≤ 1.25 m) for robustness.

### Lens fit in the head

From [lens_fit.json](../../artifacts/m12/perception/lens_fit.json), FreeCAD
on the M10 head:

| lens | depth | 15° pitch | 42° pitch |
|---|---|---|---|
| Wide | 12.4 mm | fits (0.5 mm gap to the shell) | **collides**: 33.9 mm³; the board's top rear corner hits the pod roof |
| Standard | 11.5 mm | fits | collides: 7.9 mm³ |

Re-pitching to 42° needs the pod roof or the module seat changed. This
is an input for phase E.

### Phase C exit

Done on rendered frames:
- detector accuracy and rate;
- the Pi Zero 2 W timing estimate with its basis;
- the measurement model and its check (with the shortfalls above);
- the lens fit (Wide fits at 15°; the 42° re-pitch needs a head change).

**Decisions for the user:**

1. **Detector.** pupil-apriltags is proposed: MIT bindings to the BSD-2
   C library, free, and already in the lock. The alternative, OpenCV
   ArUco, was not measured.
2. **Image size and rate.** Options:
   - **(a) Proposed:** keep 1152 × 648 with region-of-interest tracking,
     frames spread over 2 cores for 30 Hz, and accept 0.12–0.15 s of
     latency. Phase D would design against that latency instead of
     0.1 s.
   - **(b)** A lower rate, about 20 Hz on one core.
   - **(c)** A smaller image, which would shorten the detection range
     and needs a re-run.
   - With any option, a full-frame re-acquisition takes 0.26–0.34 s.
3. **Vision host.**
   - The Zero 2 W fits (a) only on the estimate; a measurement on the
     real board would settle it.
   - A Pi 4 or Pi 5 would cut detection several-fold. It would also
     add mass and power and need a different mount.
   - Prices (mid-October 2026 reports): Zero 2 W $15 list, but widely
     reported sold out; Pi 4 2 GB $67.50; Pi 5 2 GB $77.50 (raised
     1 October 2026).
   - Proposed: keep the Zero 2 W and benchmark it on the real board
     before phase E.
4. **Camera pitch, for phase E.** The waist tag needs about 42°, which
   does not fit the current pod. The low tag at 15° fits but is
   detected less reliably beyond 60° view and 1 m.

User sign-off (21 October 2026): phase C is signed off as recommended,
with the recorded shortfalls of the measurement model check accepted.
The decisions:
1. **Detector:** pupil-apriltags.
2. **Image size and rate:** option (a). Keep 1152 × 648 with
   region-of-interest tracking and a full-frame (quad_decimate 2)
   re-acquisition, sharing frames across 2 cores for 30 Hz. Phase D
   designs against 0.12–0.15 s of latency instead of 0.1 s, and
   against 0.25–0.33 s for re-acquisition.
3. **Vision host:** keep the Pi Zero 2 W, and benchmark the detector
   on a real board before phase E. The timing stays an estimate until
   then.
4. **Camera pitch:** keep the waist tag with the camera re-pitched to
   about 42°. The head pod roof or the module seat changes in phase E.

## Phase D plan: following controller (21 October 2026)

Goal: a target estimator and a follower in the supervised Teensy core.
They turn the Pi's tag measurements into velocity and yaw-rate commands
inside the signed-off envelope, and stop the robot when the target is
lost or stale. Phase D proves this against the phase C measurement
model, open loop and closed loop in the free-base plant. Rendering in
the loop is phase F.

Starting point:
- requirements in `config/following-m12.json` (signed off in phase A);
- the phase B envelope and stop behaviour: 1.2 m/s, 1.25 m/s²,
  1.0 rad/s, 1.25 rad/s², 0.3 m/s reverse, 1.0 m/s² brake, a
  jerk-limited drive reference that brakes to rest and latches hold;
- the phase C measurement model in `config/perception-m12.json`:
  - bias and spread by range and view angle, plus the yaw-blur range
    term;
  - detection probability (tracking and acquisition), and dropouts
    including bursts;
  - latency 0.12–0.15 s while tracking, 0.26–0.34 s for
    re-acquisition, at 30 Hz.

Phase C also changes some of the phase A inputs:
- The view-angle limit for 90 % detection is 45°, not 60°. A steady
  1.5 m-radius pursuit already reaches about 46°, so the brief
  brake-and-reacquire in 1 m curves and corners may happen more often.
  Phase D measures this.
- The latency is 0.12–0.15 s, not 0.1 s.

Steps:
1. **Measurement link model (Python, `bipe/m12/pi_model.py`).** This
   stands in for the Pi and for the phase E link.
   - Frames at 30 Hz, shared over 2 cores.
   - The tracking and acquisition modes, as the phase C tracker uses
     them.
   - Each frame is a detection or a miss. The probability comes from
     the model table at the true range and view angle; the burstiness
     from the replay statistics; the field-of-view and occlusion
     geometry from phase A.
   - A detection reports range and bearing with the model's bias,
     noise and blur term, the latter at the true bearing rate.
   - Each message carries a sequence number, a tag id and the
     measurement's age (exposure to send). It arrives after the
     sampled latency plus a 2 ms link delay.
   - Seeded and deterministic. Its statistics are checked against the
     model table it draws from.
2. **Target estimator (C and Python).**
   - **State:** the target's plan position and velocity in the
     robot's odometry frame (the balance estimator's x, y and yaw),
     propagated as a constant-velocity Kalman filter every control
     tick (200 Hz). The process noise is set by the target's 1.0 m/s²
     acceleration requirement.
   - **Latency compensation:** a ring buffer of the robot's pose
     (0.5 s at 200 Hz). Each measurement is placed at its exposure
     time (now less its age and the link delay), converted through
     the robot pose at that time into the odometry frame, and fused
     there. The estimate is then predicted forward to now. Messages
     out of order, repeated, older than 0.5 s or with the wrong id are
     rejected and counted.
   - **Measurement noise:** from the phase C table, looked up at the
     predicted range and view angle (a compact table compiled into
     the firmware). The range bias is not corrected by default; see
     decision 1.
   - **Gate:** a measurement is accepted only within 0.3 m of the
     prediction while coasting or braking (the reacquisition rule).
     While following, the gate is the filter's own innovation bound.
3. **Mode logic**, as in `following-m12.json` and the phase A ideal
   follower:
   - **Waiting/holding to following:** the engage rule (5 consecutive
     detections over at least 0.15 s, range 0.5–1.5 m, robot at rest)
     and an explicit enable (decision 2).
   - **Following to coasting:** no accepted measurement for 1.5 frame
     periods. The robot keeps following the prediction.
   - **Coasting to braking:** after 0.3 s without a measurement,
     command zero and brake at 1.0 m/s² through the phase B stop,
     then hold.
   - **Reacquisition:** during coasting or braking, a gated
     measurement resumes following at once. From hold, the engage
     rule applies again. There is no search.
   - **Stale link:** no message for 0.3 s (including empty frame
     reports) is treated as a loss.
   - **Faults:** a supervisor fault, a disable, or a manual command
     overrides the follower, and following then needs a fresh engage.
4. **Follower law (C and Python).** The phase A law, run on the
   estimate:
   - v = target velocity along the line of sight + k_r (r − 1.0 m);
   - ω = k_b × bearing + lateral target velocity / r.
   - Limits:
     - commands clamped to the signed-off envelope;
     - reverse at no more than 0.3 m/s;
     - v = 0 below 0.4 m (yaw still tracks);
     - the drive reference still shapes acceleration and jerk.
   - Gains are retuned for the 0.12–0.15 s latency and the filter's
     lag, starting from k_r 1.0 /s and k_b 2.5 /s. The phase margin is
     checked on the linearised bearing loop, with the delay and the
     drive reference's lag.
   - The view angle cannot be controlled with a fixed head. It is
     only logged, together with the brake-and-reacquire events.
5. **C implementation** (`firmware/src/follower.c`, declared in
   `bipe.h`; tests in `firmware/tests/test_follower.c`).
   - Fixed memory, no allocation, double precision as in the core.
   - `bipe_follower_measurement()` takes a link message.
   - `bipe_follower_step()` takes the balance estimate and the time,
     and returns velocity, yaw rate and mode. Its output goes to
     `bipe_app_command`, so the supervisor and envelope keep the final
     say.
   - With the follower disabled, the command path is unchanged.
6. **Python parity** (`bipe/m12/follow.py` as the reference; tests in
   `tests/test_m12_follower.py`). The C core and Python must agree to
   1e-9:
   - on randomised message streams (losses, out-of-order messages,
     stale messages, wrong ids);
   - on every scenario's measurement stream;
   - on the mode sequences, which must be identical.
7. **Open-loop tests** (on the measurement model, no plant).
   - The robot moves along the phase A ideal-follower paths. The
     estimator runs on the modelled messages; the follower's output
     is logged but not applied.
   - Measured against truth over 20 seeds per scenario:
     - estimate error (position, range, bearing, velocity);
     - error from latency compensation, against a no-compensation
       baseline;
     - engage, coast, brake and reacquire timing;
     - false gating.
   - Latency sweep: 0.10, 0.15 and 0.20 s.
8. **Closed-loop tests** (free-base M10 plant).
   - The C core with the sensor model, finite motor limits and
     physical contact. The follower is in C, and the measurement link
     model is fed by simulator truth. The target and occluder are the
     phase A non-contact bodies.
   - All twelve scenarios over 10 seeds; the latency sweep; a subset
     of the robustness variants (mass, wheel lag, sensor noise, slope
     start). The full variant set with rendering is phase F.
   - Records go to `artifacts/m12/following/`, with the configs and
     traces.
9. **Regression.** With the follower disabled:
   - the phase B speed cases and the M10 acceptance are bit-identical
     to their records;
   - the full pytest and the firmware host tests pass.

Exit criteria (validated simulation on the measurement model; not
rendered perception, not hardware):
- Parity to 1e-9 with identical modes. The firmware host tests and the
  regression above pass.
- In closed loop, on every seed, each scenario within the requirements
  meets the phase A table:
  - range in 0.7–1.5 m after engage;
  - bearing within ±10° on straight segments;
  - engage within the rule;
  - loss as coast 0.3 s, then brake to hold within 0.8 m;
  - reacquisition within the 0.3 m gate;
  - hold below 0.4 m.
  - Brief brake-and-reacquire events in curve_r1 and corner_90 are
    accepted, as signed off in phase A. Their number and length are
    reported.
- No falls, commands always inside the envelope, peak pitch within the
  phase B gates.
- Any shortfall is recorded with its cause, for the user's sign-off.

Decisions for the user (proposals; answer before implementation):
1. **Range bias:** the proposal is to leave it uncorrected. The model
   bias is 1–3 cm at 0.7–1.5 m, well inside the band, and a
   rendered-frame bias would not carry over to the real ISP. The
   alternative is to correct with the phase C table and the
   gyro-based blur term, at the risk of building a rendering artefact
   into the firmware.
2. **What starts following:** the proposal is an explicit "follow"
   request in the Pi link, sent by the Pi when it is told to follow;
   any manual command, fault or disable cancels it. The alternative is
   to engage whenever the tag is seen, which is simpler but starts the
   robot moving unasked.
3. **If curve and corner losses become stops** (a loss that reaches
   hold), the options are: accept it as a recorded shortfall; slow the
   curve and corner requirements; or add tags on both hips (a
   wearable change; no hardware cost). The proposal is to measure
   first and bring the result to the phase D exit.

User approval (21 October 2026): the phase D plan is approved as
recommended:
1. the range bias is left uncorrected;
2. following starts only on an explicit follow request over the Pi
   link, and any manual command, fault or disable cancels it;
3. the curve and corner losses are measured, and the options are
   brought to the phase D exit.

Phase D starts.

## Phase D checkpoint (21 October 2026)

Phase D is not finished; this records the state at the end of the day.
Evidence: validated simulation on the phase C measurement model (not
rendered perception, not hardware). Records are in
`artifacts/m12/following/`.

Done:
- **Firmware follower** (`firmware/src/follower.c`, `follower.h`, the
  generated `follower_config.h` from `config/follower-m12.json`) and the
  application integration. Following starts only on an explicit request
  while ACTIVE; a manual command, a fault or leaving ACTIVE cancels it.
  `make test` passes, including the new follower and application cases.
- **Python reference** (`bipe/m12/follow.py`). Parity with the C is 0.0
  difference and no mode mismatches: 3 random streams and all 12
  scenarios × 2 seeds × 3 latencies (32,727 reports; `parity.json`).
- **Link model** (`bipe/m12/pi_model.py`), the simulator command hook,
  and the runner (`bipe/m12/following.py`, `following_report.py`).
- **Two changes from the plan:**
  - Each history pose carries the head camera position at the measured
    body lean (0.1311 m/rad). Without it, the 2.3 mm/degree camera
    motion entered the target velocity and held a 2 Hz, ±8° pitch
    oscillation.
  - The coast starts after 2.25 frame periods, not 1.5. The tracking
    latency spread spaces report arrivals by up to 58 ms.
- **Open loop** (12 scenarios × 20 seeds × 3 latencies): the target
  estimate is within 2.9 cm RMS at 0.15 s latency, against 12 cm
  without the latency compensation. The process-noise tune kept
  1.0 m²/s³. The bearing loop has a 66° phase margin with the
  compensation, against 46° without it at 0.15 s.
- **Gain study** (`gain_study.json`): k_r 1.0–2.0 and k_b 2.5–3.5 do not
  change the stop dip, and k_b 3.5 makes the bearing oscillate in the
  1 m curve. The phase A gains are kept.
- **Closed-loop matrix**: all 214 runs (`closed_loop/runs.json`; traces
  at 0.02 s).
  - Nominal: 12 scenarios × 10 seeds, 72/120 pass.
  - Latency 0.10 s: 32/36 pass. Latency 0.20 s: 20/36 pass.
  - No falls in any of these. Commands always inside the envelope. Peak
    pitch at most 13.7°.
- **Regression, following off** (`regression/summary.json`):
  - the 44 phase B speed cases are bit-identical to their records
    (cases and traces);
  - the M10 acceptance phase (28 cases) is identical to the record,
    case by case, in the summary and in `metrics.csv`;
  - the record's 79 robustness-phase cases were not rerun today.

Shortfalls so far (for the user at the phase D exit):
1. **Range dip below the 0.7 m band** when the target stops or crosses:
   - straight_1.0 0.66–0.70 m, start_stop 0.64–0.68 m;
   - curve_r2 0.63–0.70 m, lateral_crossing 0.68–0.70 m;
   - 0.58–0.64 m at 0.20 s latency;
   - never near the 0.4 m stop range.
   The limit is the plant's stopping response with the latency; the gain
   study does not move it.
2. **Corner and curve losses** (decision 3), in the nominal 10 seeds:
   - corner_90 fails all 10 seeds; 5 losses reach hold (stopping 0.11 m,
     re-engaged by the engage rule about 0.3 s later). The waist tag
     turns away with the person (view angle up to 72°), and the
     0.26–0.34 s full-frame search after a miss outlasts the 0.3 s coast.
   - curve_r1 fails 3/10; 4 losses reach hold in all.
   - Other brake-and-reacquire events recover.
3. **Falls with the delay_15ms_noise_x2 variant**: 5 of 5 scenarios
   fell, at 1.6–2.9 s, while the target was still standing. The other
   variants (mass +15 %, wheel lag 10 ms, friction 0.5) had no falls;
   their failures are the range dip and the corner.
   - **Cause: the M10/phase B balance core.** The core falls on its own
     under this variant, with no follower, from a 0.05 m/s speed command
     at 4 Hz or 8 Hz (1 Hz and the nominal plant are fine; constant,
     white-noise and zero commands are fine).
   - The follower's speed command carries such content from the target
     velocity estimate. Replaying the recorded commands open loop
     reproduces the fall, even those from a nominal run that passed. The
     phase A ideal command (smooth, exactly zero while the target
     stands) survives on the same scenarios.
   - Phase B tested this variant only with smooth commands.
   - Follower-side trials, not adopted:
     - a 0.3 s command low-pass kept straight_1.0 up (19.6° peak), but
       the range fell to 0.07 m;
     - zeroing commands below 0.1 m/s did not prevent the curve_r1 fall.
     - Lower process noise (0.1, 0.25) did not help.
4. **Slope starts** (straight_0.6 at ±4°): peak pitch 15.2–15.3°, over
   the 15° gate. The phase B slope cases at 1.0 m/s already exceed it
   (16–19.6°, recorded).

Next (morning):
- Rerun the M10 robustness phase (79 cases) with following off and
  compare it with the record.
- Decide the delay-variant falls with the user. The options are:
  - band-limit or reshape the follower command so it carries no
    4–8 Hz content (a follower change), checked against the range band;
  - harden the balance core for command content under the delay variant
    (a controller change; it changes behaviour with following off and
    reopens the phase B and M10 records);
  - record it as an inherited limit of the core.
- Bring the range-dip and corner/curve options to the exit, with the
  final summary, plots and the results section.

## Phase D results and exit (22 October 2026)

Evidence: validated simulation under stated assumptions, not hardware.
The setup is:
- the free-base M10 plant with the C balance core, the sensor model,
  finite motor limits and physical contact;
- the C follower;
- the Pi stood in for by the phase C measurement model, not rendered
  perception in the loop (phase F).

Records are in `artifacts/m12/following/` (`summary.json`,
`closed_loop/runs.json` and `paths.png`, `regression/summary.json`,
`smoothing_trial/`). Rebuild them with `.venv/bin/python -m
bipe.m12.following --stage <stage>`. The follower and the C core are as
in the checkpoint above; nothing in either changed after it.

### Met

- **Parity:** 0.0 difference and identical modes on 72 scenario streams
  and 3 random streams. `make test` and the full pytest pass.
- **Regression, following off** (`regression/summary.json`): every
  record is bit-identical:
  - the 44 phase B speed cases, cases and traces;
  - the M10 record's 28 acceptance cases;
  - its 79 robustness-phase cases, rerun on 22 October 2026, so all 107
    cases are covered.
- **Open loop** at 0.15 s:
  - target position within 2.9 cm RMS, against 12 cm without the
    latency compensation;
  - bearing loop phase margin 66°, against 46° (39° at 0.20 s) without
    the compensation.
- **Closed loop, nominal plant:**
  - no falls, commands always inside the envelope, peak pitch at most
    13.7° (gate 15°);
  - engage, coast, brake and hold behave as specified, and every brake
    stops within 0.8 m (largest 0.36 m);
  - the range never goes below the 0.4 m stop range (lowest 0.45 m, in
    the approach scenario, where the band allows it).
- **Passing scenarios** (all seeds): approach, straight_0.3,
  straight_0.6, occlusion_short, occlusion_long and loss_fov, at every
  latency.
- **Brake-and-reacquire** (accepted in phase A), nominal 10 seeds:
  - curve_r1: 16 brakes; 12 reacquired within 0.51 s, 4 reached hold;
  - corner_90: 11 brakes; 6 reacquired within 0.32 s, 5 reached hold.

| Group | Runs | Pass | Falls | Peak pitch |
|---|---|---|---|---|
| Nominal, 0.15 s | 120 | 72 | 0 | 13.7° |
| Latency 0.10 s | 36 | 32 | 0 | 13.6° |
| Latency 0.20 s | 36 | 20 | 0 | 13.7° |
| Variants | 20 | 7 | 5 | 20.0° (falls) |
| Slopes ±4° | 2 | 0 | 0 | 15.3° |

### Shortfalls (for the user's sign-off)

1. **Range dip below the 0.7 m band floor.** This is the main
   shortfall: 40 of the 48 nominal failures fail on it alone.
   - Lowest range by scenario (nominal, 10 seeds each):

     | Scenario | Lowest range | Fail |
     |---|---|---|
     | straight_1.0 | 0.66 m | 10/10 |
     | start_stop | 0.64 m | 10/10 |
     | lateral_crossing | 0.68 m | 8/10 |
     | curve_r2 | 0.63 m | 7/10 |
     | corner_90 | 0.63 m | 5/10 |

   - At 0.20 s latency the lowest is 0.58 m; it never comes near the
     0.4 m stop range.
   - Cause: when the target stops, the robot needs the latency plus its
     stopping response (a jerk-limited 1.0 m/s² brake from 1.0 m/s)
     before it slows. The gain study does not move the dip.
   - Pass counts with a lower floor, everything else unchanged:

     | Floor | Nominal | 0.10 s | 0.20 s | Variants |
     |---|---|---|---|---|
     | 0.7 m (signed off) | 72/120 | 32/36 | 20/36 | 7/20 |
     | 0.65 m | 105/120 | 34/36 | 23/36 | 10/20 |
     | 0.6 m | 110/120 | 34/36 | 32/36 | 14/20 |

2. **Corner and curve losses that reach hold** (decision 3), nominal:
   - corner_90 has 5 such seeds and curve_r1 has 3.
   - After a loss, corner_90 also opens the range above 1.5 m (1.58–
     2.02 m): the robot is still turning while the target is already
     back at 1.0 m/s.
   - Cause: the waist tag turns away with the person (view angle up to
     72°). The 0.26–0.34 s full-frame search after a miss outlasts the
     0.3 s coast.
3. **delay_15ms_noise_x2 falls: an inherited balance-core limit** (user
   decision, 22 October 2026). All 5 scenarios with this variant fall;
   the other three variants (mass +15 %, wheel lag 10 ms, friction 0.5)
   have no falls, and their failures are the range dip and the corner.
   - **Without smoothing:** the falls come at 1.6–2.9 s while the target
     stands. Under this variant the core falls on its own from a small
     4–8 Hz speed command, which the target velocity estimate carries
     (checkpoint, shortfall 3).
   - **The smoothing trial** (user choice, 21 October; reverted;
     `smoothing_trial/`):
     - a two-stage 1.5 Hz low-pass on the follower commands, in C and
       Python, with exact parity;
     - it removes the standing falls;
     - straight_1.0, curve_r1 and loss_fov still fall, later (5.8–6.6
       s), during the 1 m/s cruise;
     - corner_90 and lateral_crossing no longer fall but fail on range
       (0.51–1.63 m and 0.59–1.21 m);
     - its lag deepens the nominal stop dip from 0.65 m to 0.55 m.
   - **The saved straight_1.0 trace:** the speed command stays at
     0.99–1.02 m/s, but from about 4 s the core carries a
     self-oscillation near 10 Hz in speed and pitch:
     - speed 0.017 m/s RMS about its mean at 4.0–5.6 s, then 0.19 m/s;
     - pitch 0.25° RMS, then 2.4°;
     - it grows to the fall at 6.55 s.

     The filter cannot remove this, because it is not in the command.
     Phase B's drive-step, weave and corner-replay cases under this
     variant passed, so the core is marginal at cruise here, not
     unstable everywhere.
   - **Decision:** smoothing was dropped, and the falls are recorded
     as a limit of the M10/phase B core under its harshest sensor
     variant (15 ms extra delay, twice the sensor noise). Hardening
     the core would change behaviour with following off and reopen the
     phase B and M10 records; it is not part of M12 unless the user
     opens it.
4. **Slope peak pitch**, straight_0.6 on ±4°:
   - +4°: 15.2° at the walk-off (2.9 s);
   - −4°: 15.3° when the target stops and the robot reverses briefly
     (17.3 s).

   Both are just over the 15° gate, with no fall and range 0.76–1.22 m.
   The phase B slope cases already exceed the gate at 1.0 m/s
   (16–19.6°, recorded shortfalls).

### Options for the exit decision

1. **Range dip:**
   - (a) accept it as a recorded shortfall, with the stop range (0.4 m)
     as the safety bound;
   - (b) lower the band floor to 0.6 m: 110/120 nominal pass, 32/36 at
     0.20 s;
   - (c) raise the nominal range from 1.0 m to about 1.1 m, which keeps
     the 0.7 m floor. It costs tag pixels at the far edge (1.5 m band
     top unchanged), and needs a matrix rerun;
   - (d) predict the target's deceleration (a target-acceleration
     state or a stop detector). This is a follower change that raises
     false-stop risk, and needs a rerun.

   Recommendation: (b), or (a) if the 0.7 m floor matters to the user.
   It is a requirement margin, not a safety issue, and (c) and (d) add
   risk for a few centimetres.
2. **Corner and curve losses** (decision 3 options):
   - (a) accept them as a recorded shortfall: the robot stops safely
     within 0.36 m and re-engages when the person faces it;
   - (b) slow the corner requirement (0.5 m/s on a 0.3 m radius), for
     example to a 0.6 m radius;
   - (c) tags on both hips: a wearable change with no hardware cost.
     It needs a phase C detector check for two tags and a
     tag-selection rule;
   - (d) lengthen the coast from 0.3 s to about 0.45 s so that it
     covers the re-acquisition search. That lengthens the worst-case
     stop by about 0.15 m at 1.0 m/s, still inside 0.8 m.

   Recommendation: (d), followed by a rerun, with (a) for what is
   left.
3. **Slope peak pitch:**
   - (a) accept it as recorded, consistent with phase B's slope
     shortfalls;
   - (b) a slope-dependent gate (for example 16° on ±4°);
   - (c) limit the follower's acceleration and reverse on a slope. That
     needs a slope estimate in the core.

   Recommendation: (a).
4. **delay_15ms_noise_x2:** recorded as above (decided). Hardening the
   core is a separate task if wanted.

Phase D exit: the follower is complete and in parity, and the
following-off regression is bit-identical across all 107 M10 cases and
the 44 speed cases. The closed-loop requirement checks pass in 72/120
nominal runs, with the shortfalls above. Sign-off and the choices
above go to the user before phase E.

### Phase D sign-off (22 October 2026)

Pranay signed off phase D as recommended:
1. **Range dip:** the band floor is lowered from 0.7 m to 0.6 m
   (`config/following-m12.json` range.band_m, with its basis).
2. **Corner and curve losses:** the loss coast is lengthened to cover
   the 0.26–0.34 s full-frame search, and the matrix is rerun. Whatever
   still fails is accepted as recorded.
   - Correction to option (d) above: a 0.45 s coast stops in 0.95 m
     from 1.0 m/s (0.45 m coasting plus 0.5 m braking), not within
     0.8 m.
   - A speed-limited coast (0.3–0.45 s, keeping 0.8 m) was implemented
     first (938698f). Pranay then chose a flat 0.45 s at every speed
     and accepted the longer stop (22 October 2026, 07:01 UTC). The
     speed-limited version's partial matrix was discarded.
   - The changes:
     - `loss.coast_s` is 0.45 s and `loss.stopping_distance_max_m` is
       0.95 m in the requirements;
     - `modes.coast_s` and `stale_link_s` are 0.45 s in the follower
       config, because the search sends no report;
     - C and Python change only in config, with parity.
   - The phase A ideal follower keeps the phase A 0.3 s coast
     (`ideal_follower.coast_s`). Its traces are the phase B and phase C
     replay inputs, so those records stay valid. The regenerated phase A
     record checks the new requirements, with traces and frames
     identical. With the 0.45 s coast in the ideal follower,
     occlusion_short would ride through the occlusion without braking,
     and the corner_90 replay input would change.
   - **The 0.95 m stop against the 1.0 m nominal range and the
     close-range rules.** The range is measured from the camera, so a
     person last seen at 1.0 m stands 1.0 m ahead of the camera.
     - A person who decelerates within the 1.0 m/s² target limit
       travels a further 0.5 m while stopping. Counting from the last
       exposure (about 0.15 s older than the coast start), the robot
       travels 0.15 + 0.45 + 0.5 = 1.1 m, which leaves about 0.4 m: the
       stop range. So there is no conflict within the target limits.
     - The margin is gone if the tag is lost at 1.0 m/s while the range
       is already at the 0.6 m floor, or if the person stops faster
       than 1.0 m/s².
     - The 0.4 m close-range stop does not act during a coast: the
       coast follows the prediction, which keeps the target moving.
     - This is recorded as the accepted cost of the longer coast.
3. **Slope peak pitch** (15.2–15.3°) is accepted as recorded, as in
   phase B.
4. **delay_15ms_noise_x2 falls** stay an inherited balance-core limit.

### Phase D results after the sign-off (22 October 2026)

The matrix is rerun with the signed-off follower and requirements: the
flat 0.45 s coast and stale-link time, the 0.95 m stopping distance and
the 0.6 m band floor. It is the same 214 runs, in
`artifacts/m12/following/closed_loop/` and `summary.json`. Evidence as
above.

| Group | Before the sign-off | After | Falls |
|---|---|---|---|
| Nominal, 0.15 s | 72/120 | 114/120 | 0 |
| Latency 0.10 s | 32/36 | 35/36 | 0 |
| Latency 0.20 s | 20/36 | 32/36 | 0 |
| Variants | 7/20 | 15/20 | 5 (all delay_15ms_noise_x2) |
| Slopes ±4° | 0/2 | 0/2 | 0 |

- **Now passing every nominal seed:** straight_1.0, start_stop,
  curve_r2 and lateral_crossing. Their dips (lowest 0.63–0.66 m) are
  inside the 0.6 m floor.
- **Corner and curve (accepted as recorded):**
  - corner_90 passes 6/10 (was 0/10). Losses that reach hold fall from
    5 to 3, and 8 of 11 brakes are reacquired within 0.29 s.
  - curve_r1 passes 8/10 (was 7/10). Holds fall from 4 to 2, and 11 of
    13 brakes are reacquired within 0.60 s.
  - The 6 nominal failures are these holds, plus corner_90's range
    opening past 1.5 m (1 seed without a hold).
- **Variants:**
  - mass +15 %, wheel lag 10 ms and friction 0.5 pass 15/15;
  - delay_15ms_noise_x2 falls in all 5 scenarios, the recorded core
    limit.
- **Slopes:** 15.2–15.3° peak pitch, accepted as recorded.
- **Loss stops:** loss_fov and occlusion_long hold as specified. The
  longest stop over the whole matrix is 0.36 m, well inside 0.95 m.
- **Limits:** no falls in the nominal plant; peak pitch at most 13.7°;
  commands always inside the envelope.
- **No new failure classes.**
- **Parity** is 0.0 with no mode mismatches, and the open loop is
  rerun.
- **Following-off regression (rerun 22 October 2026):** the 44 phase B
  speed cases and all 107 M10 cases are bit-identical to their records
  (`regression/summary.json`).

Phase D is complete: signed off, with the shortfalls above accepted as
recorded.

## Phase F plan: closed loop with rendered perception (approved 22 October 2026)

Approved as recommended on 22 October 2026 (see the approval at the end of this section). The proposals below are the approved choices.

Goal: run the following scenarios in the free-base plant with
perception in the loop. At each frame, MuJoCo renders the head camera
from the live plant state, the phase C detector and tracker read the
frame, and the result reaches the C follower through the link. This
replaces the phase D measurement model with the real perception path on
rendered frames. Phase F also covers the robustness variants, shows the
balance acceptance unchanged or explains it, and closes M12 with the
results and the README status.

Evidence: validated simulation on rendered frames under stated
assumptions, not real footage and not hardware. The Pi's timing stays
the phase C estimate until the real board is benchmarked (phase C
decision 3).

Starting point:
- the phase D follower as signed off (`config/follower-m12.json`: the
  coast is a flat 0.45 s, the stopping distance 0.95 m, and the band floor
  0.6 m);
- phase C perception (`config/perception-m12.json`):
  - camera: Wide lens, 1152 × 648 output, 10 ms global exposure from
    5 sub-frames, the waist tag at 42° camera pitch;
  - pupil-apriltags with region-of-interest tracking, and a
    full-frame (quad_decimate 2) re-acquisition;
- the phase C replays, which render the same scenes, but after the run
  and on the phase A ideal follower's path, so perception does not
  feed back;
- the phase D runner (`bipe/m12/following.py`), its metrics and its
  requirement checks.

Steps:
1. **Perception in the loop** (`bipe/m12/rendered_link.py`, a drop-in
   for the phase D link model):
   - The simulator observer records the plant state at each exposure
     sub-frame time.
   - When a frame's exposure ends, the frame is rendered:
     - the 5 sub-frames are averaged;
     - the target body and the occluder are placed from the scenario
       track;
     - shadows are off, and each frame is rendered twice to check that
       both renders are identical.
   - The phase C tracker detects the tag. Its mode (region of interest
     or full-frame) sets the latency, sampled from the phase C Pi Zero 2
     W estimate: 0.12–0.15 s tracking, 0.26–0.34 s re-acquisition. The
     frames are shared over 2 cores as in phase D. The report (sequence,
     tag id, range, bearing, age) arrives after the latency plus the
     2 ms link delay.
   - Seeds set only the latency samples. The images are deterministic.
2. **Check against the measurement model.** On nominal seed 0 of all
   12 scenarios, compare the rendered loop with the phase D link model:
   - detection rate;
   - miss runs and losses;
   - range and bearing error against truth;
   - the follower's modes and events.

   Differences beyond the model's stated spread are explained before
   the matrix runs. This is also the run-time measurement.
3. **Acceptance matrix:**
   - nominal, 12 scenarios × 5 seeds at 0.15 s latency (decision 3);
   - latency 0.10 and 0.20 s, 12 scenarios × 1 seed;
   - variants: mass +15 %, wheel lag 10 ms, friction 0.5 and
     delay_15ms_noise_x2, on the phase D five scenarios. The delay
     variant is expected to fall: that is the recorded core limit;
   - slopes, straight_0.6 at ±4°;
   - perception variants (decision 4): dim light and a strong side
     light, on straight_1.0, curve_r1 and corner_90.

   Records go to `artifacts/m12/following_rendered/`, with configs,
   traces at 0.02 s, the per-frame detections, and one saved frame
   sequence per scenario.
4. **Regression, following off:**
   - the 44 phase B speed cases and the 107 M10 cases are bit-identical
     to their records;
   - the full pytest and `make test` pass.
5. **M12 exit:** results, shortfalls, the README status, and the
   phase E items that stay open.

Exit criteria (as signed off at phase D):
- On every run within the requirements:
  - range in 0.6–1.5 m after engage;
  - bearing within ±10° on straight segments;
  - engage, loss (coast 0.45 s, then brake to hold within 0.95 m),
    reacquisition within the 0.3 m gate, and hold below 0.4 m.
- No falls apart from the recorded delay_15ms_noise_x2 limit.
  Commands stay inside the envelope. Peak pitch is within the 15° gate,
  apart from the accepted slope cases.
- Corner and curve losses that reach hold are reported with their
  number and length. They are accepted as recorded at the phase D
  sign-off.
- Rendered-loop results are no worse than phase D in pass rate or
  failure classes, or each new failure is explained by the comparison
  in step 2.
- Any shortfall is recorded with its cause, for the user's sign-off.

Run time (an estimate; step 2 measures it):
- rendering is the cost: 5 sub-frames per frame at 30 Hz, plus a 2 ms
  region-of-interest detection or an 11 ms full-frame one on this
  machine;
- at about 0.1–0.2 s per frame, a 20 s scenario adds 1–2 min to the
  8 min plant run at 12 workers;
- the proposed matrix is about 110 runs, so about 2–2.5 h at 12
  workers, with the M13 trainer allowed for. Rendering uses EGL; the
  GPU is shared with M13.

Decisions for the user:
1. **Order with phase E.** The phase table puts integration (E) before
   F. E's simulation-relevant parts are:
   - the 42° head re-pitch, which changes the head CAD and mass a
     little;
   - the link latency budget.

   E's hardware parts (the real-board benchmark, the link hardware, a
   carrier revision) cannot be done without buying parts.
   - **Proposed:** run F now on the phase C camera pose and timing
     estimate. Then do E, and rerun a reduced F subset if E changes
     the plant or the latency budget.
   - The alternative is E first, as tabled.
2. **Latency source.**
   - **Proposed:** sample the phase C Pi Zero 2 W estimate by tracker
     mode.
   - The alternative is to scale this machine's measured detection
     time per frame by the phase C host-to-Pi ratio (29–40×). That ties
     latency to the image content but adds the ratio's uncertainty.
3. **Seeds.** 5 seeds nominal is proposed, against phase D's 10. The
   images are deterministic, so seeds only vary latency. With 10 seeds
   the run time is about 3.5 h.
4. **Perception variants.** Dim light and a strong side light are
   proposed on three scenarios. They test detection margins that the
   phase D model could not, because phase C measured one lighting.
   The alternative is to leave lighting to hardware.

User approval (22 October 2026, 08:14 UTC): the phase F plan is
approved as recommended:
1. phase F runs now, on the phase C camera pose and timing estimate;
   phase E follows, with a reduced phase F rerun if E changes the plant
   or the latency budget;
2. the latency is sampled from the phase C Pi Zero 2 W estimate by
   tracker mode;
3. 5 nominal seeds;
4. the dim-light and side-light variants on straight_1.0, curve_r1 and
   corner_90.

Phase F starts after the phase D rerun results are recorded.

## Phase F checkpoint (22 October 2026)

Evidence: validated simulation on rendered frames under the phase C
camera, light and noise assumptions, with the phase C Pi Zero 2 W
timing estimate. Seed 0 of the 12 scenarios only; not the acceptance
matrix.

Rendered link (`bipe/m12/rendered_link.py`, `bipe/m12/following_rendered.py`):
- a578367 and 4c05b75: the link, with the latency drawn by tracker mode
  from the exposure centre, and the `fixed_mount` pose-conversion option;
- ae43038: while tracking, only the region of interest is searched, and
  a miss is reported at the tracking latency. The first check used
  `bipe.m12.detector.Tracker`, which falls back to a full-frame search
  within the same frame. Each single miss then cost two full-frame
  searches, about 0.6 s with no report, and 8 of the first 9 runs lost
  the target. That check was stopped and not kept;
- 0696509: tracker policies. `phase_c` is the phase C pipeline.
  `retry_and_grow` retries one region miss on the next frame before a
  full search, and grows the window margin with the detection's age at
  1.5 rad/s. `config/following-rendered-m12.json` selects
  `retry_and_grow`, pending the user's decision.

Check results, seed 0, nominal, against the phase D runs on the
measurement model (all 12 pass in phase D):

| | `phase_c` | `retry_and_grow` | Phase D |
|---|---|---|---|
| same pass as phase D | 6/12 | 8/12 | — |
| falls | 0 | 0 | 0 |
| losses (all runs) | 24 | 7 | 5 |
| holds (all runs) | 3 | 2 | 2 |
| frames processed against phase D | 14–37 % fewer | 0–10 % fewer | — |
| mean run time (10 workers) | 19 min | 25 min | — |

- `phase_c` passes: loss_fov, lateral_crossing, approach, curve_r1,
  occlusion_short, occlusion_long.
- `retry_and_grow` passes: loss_fov, start_stop, curve_r2, straight_1.0,
  curve_r1, occlusion_short, straight_0.3, occlusion_long.
- `retry_and_grow` failures are expectation checks only, with no falls,
  pitch within 13.3° and commands inside the envelope:
  - lateral_crossing and approach: range dips to 0.46 m and 0.50 m, below
    the 0.6 m floor. The estimate tracks the truth; a coast or retry
    overlaps the target's deceleration, which deepens the phase D range
    dip (0.58–0.70 m);
  - corner_90: out of the band, with 1 brief loss;
  - straight_0.6: 1 brief loss.
- Measured error against truth, all runs: range +1.1 to +1.8 cm (the
  range is to the tag's face); bearing about −0.07°. Repeat-render
  mismatches: 0.

Miss diagnosis (patched reruns of corner_90, curve_r2 and straight_1.0
on `phase_c`; behaviour identical to the check):
- of 58 region misses, 42 (72 %) were after an ordinary tracking frame,
  with the window 0.035 s old. The tag lay inside the window in all 42,
  and a full-frame search found it in only 6 at quad_decimate 2 and in 1
  at quad_decimate 1. These are undecodable frames, not window misses;
- 16 were after a full-frame search, with the window about 0.27 s old.
  The tag lay outside the window in 12 of them, and a full-frame search
  would have found it in 10. Region frames after a search missed 20 %
  of the time (27–41 % on the curves), against 4.1 % after ordinary
  tracking;
- the rendered per-frame miss rate is about 4–10 % of possible frames,
  against the phase D measurement model's 0–1 % (detection when possible
  0.89–0.97 rendered, 0.99–1.00 model). The phase C replays' dropout
  record (about 3 miss runs per replay, 1 in 3 repeated) is closer to
  the rendered rate.

Pose conversion (`fixed_mount` against `plant_camera`, seed 0 of
straight_1.0, corner_90 and start_stop, on `phase_c`): without the
body's lean, the measured range error is +2.7 to +5.9 cm mean with
6–9 cm spread and ±20 cm peaks, against 0.4 cm spread. The follower's
range estimate error rises from 0.03 m to 0.17–0.35 m rms, and every
run ends in hold. The Pi needs the lean, or the Teensy does the
conversion: a phase E link item.

Window size (`retry_and_grow`, curve_r2 seed 0, 506 region frames):
mean 235 × 217 px (7 % of the 1152 × 648 frame). After a search or a
retry the mean is 110 k px² and the maximum 663 × 641 px (57 % of the
frame). The phase C region timing was measured on smaller windows, so
the Pi time of these windows is a gap in the estimate.

Records:
- `artifacts/m12/following_rendered/`: the `retry_and_grow` check
  (check.json, runs.json, traces, frames, config.json). The example
  frame images (21 MB) are not committed;
- `artifacts/m12/following_rendered_phase_c/`: the `phase_c` check and
  `diagnostic_fixed_mount.json`. The example frame images (16 MB) are
  not committed.

Open:
- the tracker policy (`phase_c` or `retry_and_grow`) awaits the user's
  decision;
- whether the measurement model takes the rendered per-frame miss rate;
- the acceptance matrix (about 100 runs, about 4 h at 10 workers,
  resumable from runs.json) is deferred while the M13 training run
  loads the machine.

### Pitch blur and the exposure decision (22 October 2026)

The rendered misses follow the body's pitch rate, not the tracker: the
camera pitches with the balance core's oscillation (about 10 Hz while
driving), and over the 10 ms exposure that smears the tag vertically.
Across the 24 nominal runs at that point (16,287 possible frames):

| Pitch rate | Missed |
|---|---|
| under 20°/s | 1 of 6513 (0.0 %) |
| 20–40°/s | 25 of 4254 (0.6 %) |
| 40–60°/s | 406 of 3926 (10 %) |
| 60–100°/s | 809 of 1594 (51 %) |

The phase C blur set and the phase D measurement model have only the
yaw term, which is why the model's misses are 0–1 %. straight_0.6 fails
in every seed on one loss where the robot slows and the body pitch
swings 12.6° to 1.2° in 0.23 s.

Nominal at 10 ms (retry_and_grow, 5 seeds): 47 of 60 pass, no falls,
peak pitch 13.9° (artifacts/m12/following_rendered/runs.json).

At 5 ms (seed 0, cloud, software rendering): straight_0.6, approach and
corner_90 pass with no losses or coasts, 1589 of 1590 possible frames
detected, range error unchanged (+1.2 to +1.3 cm).

User decision (22 October 2026, 18:45 UTC): a 5 ms exposure with the
phase C tracker (config/following-rendered-m12.json exposure_s and
tracker). The phase F matrix reruns at 5 ms into
artifacts/m12/following_rendered_5ms/; the 10 ms runs stay as
evidence. The camera's 10 ms in config/perception-m12.json (the phase C
records) is unchanged; the phase E camera setup takes 5 ms. The dim
(50 lx) variant tests the higher noise.

## Phase F results (23 October 2026)

Evidence: validated simulation on rendered frames under stated
assumptions, not real footage or hardware. The setup is the phase D
closed loop (free-base M10 plant, C balance core with the sensor model,
C follower) with the phase D measurement model replaced by
`bipe.m12.rendered_link`:
- every 30 Hz frame rendered from the plant's state over a 5 ms
  exposure (user decision, 22 October 2026);
- read by the phase C tracker (region of interest while tracking, a
  full-frame search after a miss);
- latency drawn from the phase C Pi Zero 2 W estimate by tracker mode.

Records are in `artifacts/m12/following_rendered_5ms/` (`check.json`,
`runs.json`, `config.json`, `traces/`, `frames/`; the example images are
not committed). Rebuild them with `.venv/bin/python -m
bipe.m12.following_rendered --stage all --workers 12`. The 10 ms runs
stay in `artifacts/m12/following_rendered/` and
`artifacts/m12/following_rendered_phase_c/`.

### Met

- **Check against phase D** (seed 0 of the twelve scenarios): the same
  pass or fail in 11 of 12, with no repeat mismatches (every frame was
  rendered twice).
- **Perception in the loop**, nominal plant:
  - detected in 99.6–100 % of the frames where the tag can be seen
    (mean 99.97 %), against 0–1 % misses in the phase D model; the
    pitch-blur misses at 10 ms are gone;
  - range reads +1.2 to +1.5 cm long (to the tag's face) and the bearing
    error is under 0.08°, as in phase C.
- **Limits:** no falls apart from delay_15ms_noise_x2; peak pitch at
  most 13.9° on level ground; commands always inside the envelope; the
  longest brake stops in 0.31 m (limit 0.95 m).
- **Losses:** in the nominal runs, only loss_fov and occlusion_long
  reach hold, as specified. No corner or curve loss reaches hold
  (phase D: 3 corner_90 and 2 curve_r1 holds over 10 seeds).
- **Following-off regression:** the 44 phase B speed cases and all 107
  M10 cases (28 acceptance, 79 robustness) are bit-identical to their
  records. The full pytest passes (220).

| Group | Runs | Pass | Phase D, same runs | Falls |
|---|---|---|---|---|
| Nominal, 5 seeds | 60 | 55 | 57 | 0 |
| Latency 0.10 s | 12 | 11 | 11 | 0 |
| Latency 0.20 s | 12 | 10 | 11 | 0 |
| Variants | 20 | 14 | 15 | 5 (all delay_15ms_noise_x2) |
| Slopes ±4° | 2 | 0 | 0 | 0 |
| Dim light (50 lx) | 3 | 2 | – | 0 |
| Side light | 3 | 1 | – | 0 |

Eleven of the twelve scenarios pass every nominal seed.

### Shortfalls (for the user's sign-off)

1. **corner_90 opens the range past the 1.5 m band top**, in every
   seed: 1.54–1.65 m (phase D, seeds 0–4: 1.29–1.46 m), with no loss
   reaching hold. It also fails at both latencies, with wheel lag 10 ms
   and in dim light; each run fails only on this.
   - Cause: the waist tag turns edge-on as the person takes the corner.
     Over every nominal frame, the range reads +1.3 cm long up to 30°
     of view angle, +0.8 cm at 65–70°, and 1.4–2.0 cm short past 70°.
   - The last detection before the miss (at about 70°) is the short
     one: 3.4 cm closer than the frame before it, 35 ms earlier. The
     follower's estimator reads that as the person coming back towards
     the robot. The 0.45 s coast then predicts on it: the estimate falls
     22–30 cm short of the truth and the robot reverses at up to
     0.25 m/s while the person walks away round the corner. On
     reacquisition it is 1.5 m and more behind.
   - The phase D measurement model has no view-angle bias, so its
     coasts predict forward motion; that is the difference from
     phase D.
2. **Side light: 2 losses.** With a strong low light from one side, the
   tag cannot be decoded for about 2 s once the person turns away from
   the light (curve_r1: 10 of 172 possible frames missed; corner_90: 7
   of 224). The follower brakes and holds, which fails these runs on
   the expectation of no loss. straight_1.0 passes with every frame
   detected. The lighting is an assumption (phase C measured one
   condition); real light and auto exposure are for phase E.
3. **start_stop at 0.20 s latency** goes to 0.596 m, just under the
   0.6 m floor. Phase D passed this seed at 0.63 m, but its seed 2 at
   0.20 s went to 0.576 m, so this is within the spread at that
   latency.
4. **Recorded and accepted already:** delay_15ms_noise_x2 falls in all
   5 scenarios (the inherited balance-core limit), and the slope runs
   reach 15.1–15.3° peak pitch.

### The view-angle gate trial (23 October 2026)

A candidate fix for shortfall 1: the Pi drops a pose whose fitted
view angle is past a gate, and does not send it. The tracker keeps the
tag, so the next frame is still a region-of-interest search
(`view_gate_deg` in `bipe.m12.rendered_link`, off by default; bffa4b5).
- The gate is on the fitted 3-D angle between the tag's normal and the
  line of sight. The head camera looks down on the waist tag, so
  facing the person reads about 38°, not the 0° of the phase A
  yaw-only view angle. 74° fitted is about 69–70° of yaw.
- These trial runs are in the cloud on software rendering (OSMesa),
  whose frames differ slightly from the laptop's GPU (EGL) frames.
  corner_90 seed 0 passes in the cloud without the gate (it failed on
  the laptop at 1.555 m), so the comparison is made in the cloud, gate
  against no gate, on the same seeds.

| corner_90, 0.15 s | No gate (cloud) | Gate 65° | Gate 74° |
|---|---|---|---|
| Seed 0 | pass (record) | hold, 1.29 m | pass, 1.41 m |
| Seed 1 | fail, 1.64 m | hold, 1.28 m | pass, 1.49 m |
| Seed 2 | fail, 1.52 m | hold, 1.28 m | pass, 1.28 m |
| Seed 3 | fail, 1.51 m | hold, 1.28 m | pass, 1.42 m |
| Seed 4 | fail, 1.67 m | hold, 1.30 m | pass, 1.30 m |

- At 65° (about 57° of yaw) the gate drops 7–9 good frames before the
  miss, so the gap outlasts the 0.45 s coast and every seed reaches
  hold.
- At 74° it drops 0–3 frames, the edge-on ones, and the coast carries
  through. Seed 1's 1.494 m sits just inside the band. In seed 0 it
  dropped none, so that row is the cloud render, not the gate.

**The steep-angle runs with the 74° gate.** Only the 12 runs with a
detection past 60° of yaw view angle can change, since the gate never
acts below about 69°. All 12 were rerun with the gate, in the cloud
(`artifacts/m12/following_rendered_5ms_gate74_cloud/`, with the
no-gate baseline and the 65° runs). 11 of 12 pass, with no holds and
no falls. Detection where possible is 98.9–100 %.

| Run | Laptop, no gate | Cloud, gate 74° | Frames dropped |
|---|---|---|---|
| corner_90, nominal seeds 0–4 | 0/5 (1.54–1.65 m) | 5/5 (1.28–1.49 m) | 0–3 |
| corner_90, 0.10 s | fail, 1.58 m | pass, 1.23 m | 3 |
| corner_90, 0.20 s | fail, 1.66 m | fail, 1.555 m | 1 |
| corner_90, mass +15 % | pass, 1.49 m | pass, 1.46 m | 1 |
| corner_90, wheel lag 10 ms | fail, 1.60 m | pass, 1.42 m | 3 |
| corner_90, friction 0.5 | pass, 1.28 m | pass, 1.40 m | 1 |
| corner_90, dim light | fail, 1.51 m | pass, 1.43 m | 0 |
| curve_r1, 0.10 s | pass, 1.17 m | pass, 1.17 m | 0 |

- corner_90 at 0.20 s still opens to 1.555 m. Phase D failed the same
  run too (1.561 m): at that latency the corner is outside the band on
  either model.
- The dim-light row dropped no frames, so its pass is the cloud render.
  The laptop's 1.509 m was 9 mm over the band.
- The cloud comparison is clean only for nominal seeds 1–4, where the
  same seeds ran with and without the gate. The variant and latency
  rows compare against the laptop record, across the two renderers.
  A laptop rerun of these 12 runs with the gate (about 1 h at 12
  workers) would settle them on one renderer.

### Options for the exit decision

1. **corner_90 above the band top:**
   - (a) adopt the 74° gate (`view_gate_deg` 74 in
     `config/following-rendered-m12.json`) as part of the Pi
     pipeline, and confirm it on the laptop by rerunning the 12
     steep-angle runs into the phase F record. The cost is one more
     check on the Pi per detection (the pose's rotation is already
     fitted). It needs no follower or core change, so the following-off
     regression is not affected;
   - (b) accept it as recorded: no loss reaches hold, and the overshoot
     is 4–15 cm past the band top;
   - (c) clamp the target's range rate in the follower's coast. That
     is a C follower change, with parity and a matrix rerun.

   Recommendation: (a). It removes the cause, the measurement, rather
   than tuning the follower around it, and it fits a real Pi: edge-on
   AprilTag poses are known to be unreliable.
2. **Side-light losses:**
   - (a) accept them as recorded, and test real lighting with the
     camera's auto exposure in phase E. The sim lighting is an
     assumption, and the follower stops and holds safely;
   - (b) model auto exposure and gain in the camera model now, and
     rerun the lighting variants.

   Recommendation: (a).
3. **start_stop at 0.20 s** (0.596 m) and the **0.20 s corner** (1.555
   m): accept as recorded, as the latency-sweep spread phase D also
   showed.

### Phase F exit

Phase F met its plan: perception on rendered frames in the loop for the
whole matrix, the check against phase D, the robustness variants, the
lighting variants and a bit-identical following-off regression. With
the 74° gate (option 1a, on the cloud evidence so far), all 60 nominal
runs pass: the 55 that already passed have no frame the gate can act
on, and the 5 corner_90 seeds pass with it. loss_fov and
occlusion_long hold as specified. Remaining for M12 is phase E, the hardware, which
waits for the purchases (user decision, 21 October 2026: no hardware
purchases before the public release).

### Phase F sign-off (24 October 2026)

Pranay signed off phase F as recommended:
1. **corner_90:** the 74° view gate is adopted
   (`config/following-rendered-m12.json` view_gate_deg, with its
   basis). The 12 steep-angle runs are rerun on the laptop into the
   phase F record; the other 100 runs have no frame the gate can act
   on.
2. **Side-light losses:** accepted as recorded; real lighting and the
   camera's auto exposure are tested on hardware in phase E.
3. **start_stop and corner_90 at 0.20 s:** accepted as recorded, with
   the latency-sweep spread.

### Phase F results with the view gate (24 October 2026)

The 12 steep-angle runs were rerun on the laptop with the 74° gate,
into the phase F record (`artifacts/m12/following_rendered_5ms/`,
9afcc78). The other 100 runs are unchanged. The no-gate record is
kept locally (not committed). 11 of the 12 pass, with no falls and no
holds; each corner_90 run now has one brief loss that is reacquired.
The cloud trial agreed on all 12.

| Group | Runs | Without the gate | With the gate | Falls |
|---|---|---|---|---|
| Nominal, 5 seeds | 60 | 55 | **60** | 0 |
| Latency 0.10 s | 12 | 11 | **12** | 0 |
| Latency 0.20 s | 12 | 10 | 10 | 0 |
| Variants | 20 | 14 | **15** | 5 (all delay_15ms_noise_x2) |
| Slopes ±4° | 2 | 0 | 0 | 0 |
| Dim light (50 lx) | 3 | 2 | **3** | 0 |
| Side light | 3 | 1 | 1 | 0 |
| All | 112 | 93 | **101** | 5 |

- corner_90, nominal seeds 0–4: range 0.64–1.49 m (was up to 1.65 m).
- The check now matches phase D's pass or fail in 12 of 12 (was 11),
  with the same events in 11 and no repeat mismatches.
- corner_90 at 0.20 s still opens to 1.53 m (phase D: 1.56 m), and
  start_stop at 0.20 s reaches 0.596 m. Both are accepted with the
  latency-sweep spread.
- Peak pitch on level ground is at most 14.0° (mass +15 %), and
  commands stay inside the envelope.
- The gate is in the rendered link only, so the following-off
  regression is unaffected. The rendered-link tests pass.

Phase F is complete and signed off. Remaining failures, all accepted:
- the delay_15ms_noise_x2 falls (the core limit);
- the slope pitch;
- the two side-light losses (phase E);
- the two runs at 0.20 s latency.

M12's simulated phases are done. Phase E, the hardware, waits for the
purchases.

## Review follow-ups (29 October 2026)

A review of PR #2 found the following. Each is fixed here or recorded.
1. **Pi restart and keep-alive (firmware, fixed).**
   - Before: the follower rejected any report whose sequence was not
     above the last one, and nothing reset that check. After a Pi
     restart, every report was "repeated" until the Teensy rebooted.
   - Now a sequence more than 30 behind the last (one second of frames)
     is taken as the Pi restarting its counter and is accepted. A new
     follow request from off also accepts the counter afresh.
   - A repeated report no longer counts as host keep-alive in the
     application.
   - Changed in `firmware/src/follower.c` and `bipe/m12/follow.py`
     together, with parity. Sequences only increase and the request is
     made once in every recorded run, so the phase D and F records are
     unchanged.
2. **Follower config and step failures (firmware, fixed).**
   - The application refuses a follow request when
     `bipe_follow_config_valid` fails.
   - A failed `bipe_follow_step` now cancels following instead of
     leaving the mode on with zero output.
   - The config check now rejects a history shorter than two samples.
3. **Gated frames in the phase F records.**
   - The frame records now keep `gated` and `measured_view_deg`.
   - `frame_metrics` counts a gated detection as a miss and leaves its
     range out of the error statistics.
   - The 12 gated runs in `artifacts/m12/following_rendered_5ms/`
     predate this change. Their perception statistics count the gated
     frames as detections (0–3 per run in the cloud trial), and their
     frame files cannot show which frames were dropped. Pass or fail
     is unaffected, since it comes from the follower's reports.
   - The claim that the other 100 runs are unaffected was checked from
     the true view angle: they peak at 59° of yaw, against a gate that
     acts from about 69°.
4. **The gate is in-sample and in the simulation only.**
   - The 74° gate was chosen on the same corner_90 runs it is scored
     on, so 101/112 is an in-sample figure.
   - It exists only in `bipe/m12/rendered_link.py`. The real Pi
     pipeline in phase E must implement it, and phase E should check
     it on real footage of a turning person before relying on the
     number.

## The view gate on the Pi code (31 October 2026)

Follow-up to review item 4: the 74° gate now lives in the code the Pi
will run, and the simulation runs that same code.
- `bipe/pi/` is the Pi Zero 2 W side of the link. It imports only
  numpy, scipy and pupil-apriltags (a test checks that MuJoCo and the
  simulator are not loaded), so it installs on Raspberry Pi OS as it is.
  - `lens.py` and `pose.py`: the lens model and the tag pose fit, moved
    from `bipe.m12.camera` and `bipe.m12.detector`, which re-export them.
  - `apriltag.py`: the pupil-apriltags wrapper (with the safe
    destructor), moved from `bipe.m12.detector`.
  - `tracker.py`: the phase C tracker policy (region of interest after a
    detection, a full-frame search after a region miss), moved from
    `bipe.m12.rendered_link`.
  - `gate.py`: the view angle and the gate. The threshold is read from
    `config/following-rendered-m12.json` `view_gate_deg`, the signed-off
    value, so there is one copy of the number.
  - `pipeline.py`: one frame through tracker, pose, gate and report. The
    range and bearing go through a mount object: `FixedMount` on the Pi
    (camera pitch plus the body's lean), the plant's camera pose in the
    simulator.
- `bipe.m12.rendered_link` now runs each frame through the Pi's
  `FramePipeline` (tracker, pose fit, gate and report assembly); only the
  mount differs, and the simulator fills in the latency. The code moved
  without change, apart from one fix: a pose whose view angle is
  not finite read as 0° (`min(1.0, nan)` is 1.0) and passed the gate; it
  is now gated. The fit has not produced such a pose in any recorded run.
- The lean is a required input on every frame, and a value that is not
  finite is refused. A lean of 0 on a balancing robot is the
  `fixed_mount` diagnostic of the phase C to D pose conversion check
  (+2.7 to +5.9 cm mean range error, every run ended in hold), so it is
  no default.
- Parity: the gated corner_90 seed 1 run (cloud, OSMesa) was run on
  main and on this change, with the simulator running `FramePipeline`. The trace, the follower's report counts and
  every frame record are identical apart from the detector's wall-clock
  time; the run has one gated frame (75.4°) and passes on both.
- What the gate does on the Pi: a detection whose fitted view angle is
  past 74° is sent as an empty frame (detected false, range and bearing
  0), and the tracker keeps the tag, so the next frame is a region of
  interest search. This is what the phase F runs did. The config's basis
  said the Pi "sends no report"; the runs sent an empty-frame report,
  which the follower counts as a miss (`BIPE_FOLLOW_EMPTY`). The Pi code
  follows the runs, and the config's basis now says so.
- One thing the tests showed: after gated frames the region of interest
  is still the edge-on detection's narrow box, so a tag that turns back
  quickly can fall outside it and cost one full-frame search. The
  phase F runs had the same behaviour.

Host tests (`tests/test_pi_pipeline.py`) draw tag36h11 frames through a
pinhole camera and read them with the real pupil-apriltags detector. In a
sweep from 0° to 85° in 5° steps, centred and 300/−150 px off-centre, the
fitted angle is within 0.08° of the drawn one from 60° to 85°, around the
gate, and within 0.55° near facing the camera, where the angle is
ill-conditioned. These frames are noise-free and blur-free, so this is the
best case; the margin against fit noise comes from the rendered phase F
runs. Poses at 40° and 65° are sent, one at 82° is dropped (also through
a distorted lens, k1 −0.12, k2 0.02, off-centre), and the tracker stays
on the region of interest through the dropped frames. The tracker policy, the
mount conversion (against the simulator's world point and range-bearing)
and the config value are tested too.

Still open for phase E, on hardware:
- the camera capture (picamera2, exposure timestamps for `age_s`);
- the config path on the Pi: `ViewGate.from_config` reads the repository
  checkout's `config/` by default, and takes a path for a packaged
  install;
- the link to the Teensy (the bus is a phase E choice) and its encoding
  of the report;
- the lens calibration of the real Camera Module 3 Wide, and the camera
  pitch of the re-pitched head;
- the body lean the Pi is given: phase F converted with the plant's own
  camera pose, so the Pi needs the lean from the Teensy, or the
  fixed-mount error has to be accepted;
- the Pi timing of the pipeline, and the gate checked on real footage
  of a person walking and turning (build plan stage 0).

### Closed loop rerun with the knee wheel-reaction feedforward (1 November 2026)

The classical controller's knee feedforward now cancels the wheel
torque's reaction on the knee drive (M13 doc, "Fallback fall while
driving"). The 214-run matrix was rerun from scratch with it
(`artifacts/m12/following/closed_loop/`, `summary.json`):

| Group | Before | After | Falls |
|---|---|---|---|
| Nominal, 0.15 s | 114/120 | 114/120 | 0 |
| Latency 0.10 s | 35/36 | 35/36 | 0 |
| Latency 0.20 s | 32/36 | 32/36 | 0 |
| Variants | 15/20 | 19/20 | 0 (was 5, all delay_15ms_noise_x2) |
| Slopes ±4° | 0/2 | 0/2 | 0 |

- delay_15ms_noise_x2 no longer falls in any scenario. Four of its five
  runs now pass; corner_90 stays up (peak 13.9°) and misses only its
  expectation, the same corner shortfall as the nominal seeds.
- Every other run keeps its verdict, and no peak pitch moves by more
  than 0.31°; the nominal peak is 13.7° as before.
- Not rerun: the phase F rendered-perception records
  (`artifacts/m12/following_rendered*`) still show the earlier
  controller, including their delay_15ms_noise_x2 falls.
