# M10 — Compact serial-leg redesign

11 October 2026. The M9 two-hip five-bar works as a layout study but does not
match the reference robot: the linkage is exposed, the face is flat on the body and the
proportions are long and low. M10 redesigns the robot around the reference
robot's architecture and appearance. M9 stays the reference for methods and
acceptance; its final state is tagged `archive/m9-final` (`62ec65b`), and
the unaccepted inboard-wheel step is kept on `m9-step2-inboard-wip`.

## Reference

The reference is a small commercial wheeled camera robot by Mondo Robotics: 1.75 kg, 21.5 × 18 × 18 cm (L × W × H), as listed on
[mondorobotics.com](https://www.mondorobotics.com/). Product photos
show:

- a faceted, rounded body box with two small cameras low on the front;
- a binocular head pod (two large lit eye cylinders, camera between) on a
  short neck, with two flap ears;
- per side, a large round hip housing on the body flank, a thick capsule thigh
  running back and down to a knee behind the wheel, and a short shin running
  forward and down to an outboard wheel with a knobby tire and three-spoke
  hub cover.

Internal mechanism, actuators and dimensions beyond the listed envelope are
not published; everything below is our engineering interpretation.

## User decisions (11 October 2026)

| Topic | Decision |
|---|---|
| Legs | Serial thigh–knee–shin. Two coaxial motors in the round hip housing; the knee is belt-driven inside the thigh. |
| Wheels | Hub motor inside each outboard wheel. |
| Head | Styled, fixed binocular eye pod with ears. Space and mass reserved for a later pan joint; camera following stays later work. |
| Size and mass | About 215 × 180 × 180 mm at the nominal pose; allocated mass ≤2.15 kg with ≥150 g reserve. |

## Requirements carried over

- `config/robot.json` remains the SI source; millimetres only at the FreeCAD
  boundary.
- Balance acceptance needs a free base, finite motor limits and physical
  contact; no hidden pose constraints or state clamps.
- The 107-case locomotion acceptance and its thresholds carry over unchanged.
  The posture parameterization changes from five-bar crank angles to a serial
  leg.
- The six passive tips need structural factor ≥2. Cosmetic parts must stay
  attached and contained.
- Real parts are selected from manufacturer documentation against computed
  loads; unknowns are stated as assumptions.

## Phases

| Phase | Content | Exit |
|---|---|---|
| A. Concept and sizing | Leg geometry, actuator and hub-motor selection, mass budget, scripted blockout CAD, reference comparison renders | User signs off on look and layout |
| B. Detailed CAD | Shells, serial legs with knee belt, hub wheels, head pod, battery and electronics packaging; fit, all-pose sweep, mass adoption | Mass checkpoint, clean fit and sweep |
| C. Plant and control | Serial-leg free-base MuJoCo plant with knee belt and hub motors; posture kinematics in Python and C firmware; 107-case acceptance | Acceptance targets |
| D. Falls | Six passive tips, structural screen, bumpers where needed | Structural factor ≥2 |
| E. Repository cleanup | Prune code and artifacts M10 no longer uses (kept in the archive tag); reorganize docs; single README status | Tests pass on the slim tree |

Each phase gets a detailed plan immediately before it starts and is reported
before the next.

## Phase A plan — concept and sizing

1. **Proportions from the reference.** Derive target body, head, wheel and leg
   proportions from the photos and the listed envelope: body box, head pod
   and eye diameter, wheel diameter and width, hip location on the flank,
   thigh and shin lengths.
2. **Leg kinematics.** Choose thigh and shin lengths so that the nominal,
   crouched and extended heights cover the current ride-height range, the
   wheel stays under the centre of mass, and folded legs fit the envelope.
   Record the knee-belt ratio and the joint ranges.
3. **Actuators.**
   - **Hip and knee.** Re-check the M9 RO40 Lite frameless plus 4:1 belt
     against the serial-leg torques computed from the new geometry, mass and
     acceptance demands.
   - **Hub motor.** Compare direct-drive candidates against measured wheel
     demand: peak 0.45 N·m in the 2.5 N·s pushes, RMS ≤0.08 N·m.
     - [CubeMars GL40 KV70](https://www.cubemars.com/product/gl40-kv70-gimbal-motor.html):
       Ø46.5 mm, 107 g, 0.25 N·m rated, 0.5 N·m peak.
     - [CubeMars GL60 KV25](https://www.cubemars.com/product/gl60-kv25-gimbal-motor.html):
       Ø69 mm, 230 g, 0.6 N·m rated, 1.75 N·m peak.
     - Any geared alternative found in the same search.
   - Check speed, the 6S bus, encoder and driver compatibility, and the
     effect of foot mass on balance.
4. **Mass budget.** Build an itemized allocation against 2.15 kg with
   ≥150 g reserve, reusing M9 catalog masses where parts carry over.
5. **Blockout CAD.** Write a new scripted FreeCAD model (`cad/m10/`) at
   blockout fidelity:
   - the body shell form;
   - the head pod;
   - legs at the nominal, crouched and folded poses;
   - wheels with hub motors;
   - battery and electronics volumes.

   Check the envelope and gross interferences.
6. **Renders.** Render operating, crouched and folded poses in Blender, plus a
   side-by-side with the reference robot, then report for sign-off.

## Phase A results

Concept: `config/robot.json` `m10`, `bipe/m10/concept.py` (kinematics, mass
layout, static drive torques) and `cad/m10/blockout.py` (built through the
FreeCAD GUI bridge, `tools/freecad_bridge.py`). Outputs are in
`artifacts/m10/concept/`.

**Revision 1** (`99fadea`, knee forward) was reviewed by the user on
11 October 2026. Findings: the reference robot's knees point backward; its body is a
longer cuboid with rounded edges, not a faceted cube; the ears faced forward
instead of to the sides.

**Revision 2** (knee back):

- Body 130 × 92 × 80 mm with 12 mm edge fillets. The hip sits 10 mm ahead of
  the body centre and 42 mm above the bottom. The battery is placed behind
  the hip so that the axle sits behind it, as in the photos.
- Thigh 70 mm and shin 62 mm. A knee-back leg needs the shin close to the
  thigh length; with the first 78/48 mm trial the crouched knee fell below
  the axle.
- Ears turned to the sides, 25° forward, swept back 30° and splayed 20°.

| Pose | Hip height | Knee angle | Thigh / knee drive (N·m) | Body clearance | Bounds L × W × H (mm) |
|---|---|---|---|---|---|
| Crouched | 72 mm | 147.5° | 0.43 / 0.38 | 30 mm | 146 × 194 × 195 |
| Nominal | 103 mm | 118.5° | 0.39 / 0.34 | 61 mm | 139 × 194 × 226 |
| Extended | 135 mm | 82.3° | 0.31 / 0.26 | 93 mm | 132 × 194 × 258 |

Drive torques are static, per leg, against 0.72 N·m continuous and
2.16 N·m peak. The height includes the ears; the head top is 195 mm at
nominal. The allocated mass is 1,657 g against the 2,150 g cap. No moving
leg part overlaps the body, the head or a non-adjacent leg part in any pose.

Open against the targets: width 194 mm (target about 180) and length
139 mm (the reference lists 215).

User review of revision 2 (11 October 2026): accepted for phase B. Small
changes in measurements and mass are acceptable; appearance has priority.

## Phase B plan — detailed CAD

Work outside in. The exterior is styled and approved first, and the
internals are then fitted behind it. When an internal part does not fit,
the interior changes, not the look. Small drifts in size and mass are
allowed and recorded.

Tooling: FreeCAD scripts in `cad/m10/`, run through the GUI bridge
(`tools/freecad_bridge.py`). Blender renders come from
`tools/blender_bridge.py`, which talks to the Blender Lab MCP add-on on port
9877. Both apps run as the user's snap GUI instances. Geometry stays SI in
`config/robot.json` `m10`; millimetres appear only inside the FreeCAD
scripts.

### B1. Exterior styling — user review gate

1. **Body shell.**
   - Rounded cuboid with the reference robot's details: soft front facets, panel split lines,
     front camera bezels, and a battery hatch underneath.
   - Printable wall (2 mm PA12 assumption) and the split into printable parts.
2. **Head.**
   - Wider pod set further back.
   - Eye cylinders with LED ring and diffuser, a camera window, a rear housing,
     and swappable ear mounts.
   - Neck interface sized for the later pan joint.
3. **Legs.**
   - Hip cap.
   - Thick rounded thigh cover with the reference robot's flat inset panel and knee bolt.
   - Slim shin.
4. **Wheels.**
   - Knobby tire tread.
   - Three-spoke hub cover hiding the hub motor.
5. **Blender look renders** (`cad/m10/render_blender.py`).
   - Scene: per-part meshes exported from FreeCAD, matte plastic and rubber
     materials, emissive eyes, soft studio lighting.
   - Views: front three-quarter, side, rear three-quarter, crouched and
     extended.
   - A sheet against the reference photos.

**Exit:** the user approves the look.

### B2. Internal parts from manufacturer data

1. **Hip module packaging study.** The phase A sizing assumed RO40 KV140
   Lite frameless motors with 4:1 belts (0.72 N·m continuous). Compare:
   - (a) motors offset inside the body, belted to concentric hip shafts (an
     outer thigh shaft and an inner knee-pulley shaft);
   - (b) motors coaxial in the hip housing with an in-line reduction.

   Choose on fit behind the approved exterior, mass and parts risk. Report
   the choice before detailing.
2. **Knee drive.**
   - 1:1 belt inside the thigh with pulleys and tensioner.
   - Size the pulley and belt width so peak tension (226 N with a Ø19 mm
     pulley at 2.16 N·m) has a stated margin; otherwise apply a current cap.
3. **Hub wheels.**
   - GL40 KV70 outer rotor carrying the rim.
   - Stator shaft fixed to the shin; bearings; AS5047P encoder on a rotor
     magnet.
4. **Electronics and power.**
   - 6S 850 mAh pack behind the hip, as the concept balance assumes.
   - Six moteus-c1 drivers (four joints, two hubs), host board, IMU and power
     board.
   - Head LEDs and camera; harness route through each hip.
5. **Fasteners and inserts** at the shell splits and part mounts.

### B3. Checks and adoption

1. **Pose sweep.**
   - Joint grid across hip heights 0.072–0.135 m, fore-aft ±0.01 m and the
     fold pose.
   - No unintended overlaps between moving parts, covers, belts or harness.
2. **Mass and inertia.**
   - Per-part masses from CAD volumes with material densities, plus
     catalog masses.
   - Rigid bodies for phase C: body, thighs, shins, wheel stators, wheel
     rotors.
   - The allocated mass stays within 2.15 kg with ≥150 g reserve.
3. **Manufacturability screen.**
   - Wall thickness, print orientation, insert bosses, assembly order.
4. **Adoption.**
   - Write the CAD-derived values to `config/robot.json` `m10`.
   - Save the FreeCAD document, sweep, mass report and renders under
     `artifacts/m10/cad/`.

**Phase exit:** the approved look; a clean sweep; mass within the cap; the
rigid-body set saved for phase C.

#### B3 detailed plan (12 October 2026)

The order puts mass before the buildability details, because a mass
overrun is the result most likely to need a user decision.

1. **Pose sweep** (`cad/m10/sweep.py`, headless FreeCAD).
   - **Grid.**
     - Eight hip heights from 0.072 to 0.135 m.
     - At each height, axle offsets of −10, −5, 0, +5 and +10 mm fore–aft
       from the balance point, as when accelerating or on a slope.
     - `bipe.m10.concept` gains `pose_at(hip_height, dx)`, which uses the
       same IK and does not search for balance.
   - **Sit pose.** M10 has not yet defined a fold pose. Define it as the
     lowest hip height at which the legs fold with the wheels still on the
     ground and nothing clashes, down to the body resting on the floor
     (hip height 0.042 m). This is the power-off rest pose.
   - **Checks at every pose.**
     - No cross-link overlap above 1 mm³ between moving parts, belts or
       covers.
     - Nothing except the tread below the ground.
     - The knee stays behind the hip–axle line.
   - **Output.** The joint ranges covered (thigh angle a1, shin angle a2
     and knee angle) become the phase C joint limits, saved in
     `artifacts/m10/cad/sweep.json`.
2. **Mass and inertia** (`cad/m10/mass_properties.py`).
   - **Sources.**
     - Each part gets a material or a catalog mass from a table in
       `config/robot.json` `m10.mass_properties`.
     - Densities and catalog masses are reused from the sourced M9 catalog:
       PA12 930, TPU 1100, 6061 2700, 7075 2810, FR4 1850 and belt
       1300 kg/m³; RO40 60 g, moteus-c1 8.9 g, the pack 165 g; GL40 107 g
       from CubeMars.
     - Unsourced items (module housing, camera, host board, bearings) are
       explicit assumptions.
     - A catalog part's mass is spread uniformly over its CAD envelope.
   - **Rigid bodies for phase C.**
     - body;
     - `{side}_thigh`;
     - `{side}_shin`, including the GL40 stator, axle tube and encoder;
     - `{side}_wheel`, the GL40 rotor with hub and tire.
   - **Knee drive train.** The concentric inner shaft and its pulleys turn
     relative to the body. Their mass is lumped into the body, and their
     inertia about the hip axis is reported separately for the transmission
     model.
   - **Frames.** Centre of mass and inertia tensor about the centre of
     mass, in each link's frame at the nominal pose. Each frame has its
     origin at the joint and its x axis along the link.
   - **Gate.** CAD mass plus the harness (60 g) and fastener (60 g)
     allowances must be at most 2.15 kg, with at least 150 g reserve. If
     the gate fails, stop and report options before cutting mass.
3. **Buildability CAD** (added to `cad/m10/assembly.py`).
   - **Body shell.**
     - Split into an upper cover and a lower tub along a horizontal line,
       with M2.5 screw bosses for heat-set inserts.
     - A battery hatch underneath, which completes the B1 item.
   - **Head.** Split into front and rear halves, and fix the 2 mm head-pod
     fillet that the kernel skips.
   - **Motor mounting.**
     - Each motor pair is held between the bulkhead and a side carrier on
       standoffs.
     - Slotted holes along x give each 4:1 belt its tension travel.
     - The hip bracket and bulkhead tie to the shell bosses.
   - **Knee belt.** Eccentric bearing seats in the thigh give the tension
     adjustment.
   - **Harness.** A Ø6 mm pass from the body through each hip pod into the
     thigh, plus a neck pass for the head wiring.
   - **Belt check.** The knee belt (2GT 9 mm, 40T) is checked against the
     Gates rating, as M9 did for the 6 mm hip belt.
   - Re-run the overlap checks in three poses and the sweep after these
     changes.
4. **Manufacturability screen** (`artifacts/m10/cad/manufacturability.md`).
   - Minimum wall per printed part, measured from the CAD.
   - Print orientation and support for each printed part.
   - Insert bosses.
   - Assembly order.
   - Parts that need machining: shafts, bulkhead and pulleys.
5. **Adoption.**
   - Write the CAD rigid bodies and joint ranges to `config/robot.json`
     `m10.cad`.
   - Switch `bipe.m10.concept` to CAD masses, re-solve the balance poses
     and refresh the renders.
   - Save the documents, sweep, mass report and renders under
     `artifacts/m10/cad/`, then report.

## Phase B results

**B1 exterior** (`7f99cc6`; camera pitch and shading on 12 October). The user
approved the look. The head camera is pitched up 15° like the reference.
Renders are in `artifacts/m10/exterior/renders/`.

**B2.1 hip drive packaging study** (`bipe/m10/hip_packaging.py`; outputs in
`artifacts/m10/hip-packaging/`). The RO40 Lite module (Ø49.3 × 15.5 mm with
housing and encoder) and the 80T output pulley (Ø54) are too large for two
coaxial drives per side inside a 92 mm body.

| | A: rear motor row, 4:1 belts to concentric hip pulleys | B: motor plus 4:1 planetary per joint on the hip axis |
|---|---|---|
| Fit | All parts inside the body; no clashes | Hip stack protrudes 16.5 mm beyond the body wall into deep pods |
| Body | 145 × 102 × 80 mm | 130 × 92 × 80 mm |
| Overall width | ≈205 mm | ≈231 mm |
| Layout | Battery in front of the hip; four motors on one axis 40 mm behind it; six drivers above and below the motors and beside the battery | Battery at the rear; motors on the hip axis |
| Nominal thigh slope | 26.0° | 27.5° |
| Static drive (thigh / knee) | 0.40 / 0.33 N·m | 0.39 / 0.34 N·m |
| Parts risk | M9 belt drive with ratings, tension and compliance already screened | Four custom planetary stages (backlash, noise, gear supply) |

The AK40-10 integrated actuator (Ø53 × 40.2 mm, 190 g) cannot fit two per
side on the hip axis, and as an offset unit it needs a body of about 165 mm.

Proposed: A. Awaiting the user's choice before detailing.

**User decision (12 October 2026): option A.** Adopted in `config/robot.json`
`m10`:

- body 145 × 102 × 80 mm;
- hip pod, thigh, shin and wheel moved 5 mm outboard (overall width 205 mm);
- head flush with the new front;
- the `layout` block places the motor row 40 mm behind the hip, the battery
  44 mm ahead of it and the drivers' centroid 13 mm behind it.

The concept balance uses these positions. Nominal pose: thigh 26° below
horizontal; static drive 0.40 N·m (thigh) and 0.33 N·m (knee). The exterior
was rebuilt with no cross-link overlaps; bounds at nominal are
147 × 205 × 211 mm.

**B2 internal parts** (`cad/m10/assembly.py`; outputs in
`artifacts/m10/assembly/`). This is a layout study at envelope fidelity, not a
hardware-validated design. The internal sizes come from catalog and drawing
envelopes in `config/robot.json` `m10.internals`. Where no manufacturer figure
was used, the value is marked as an assumption.

The assembly reuses the B1 exterior. It hollows the body to a 2 mm wall, the
thighs to a 2 mm wall, the head to 1.6 mm and the wheel hubs around the motor.
Designed seats are cut from the part that holds them.

Packaging set these dimensions:

- **Body: 150 × 107 × 80 mm.**
  - Width: an 8 mm centre bulkhead carries the inner-shaft bearings.
  - Length: the battery, stood on end, has to clear the 10 mm inner fillet
    and the hip pulley flanges.
- **Wheel width: 28 mm.** The GL40 (21.5 mm) plus the encoder stack and a
  1.5 mm hub wall.
- **Overall width: 219 mm.**

The y stack, outward per side (mm):

| Part | y range |
|---|---|
| Bulkhead | 0–4 |
| Knee 80T pulley | 4.5–11.5 |
| Knee motor | 12–27.5 |
| Thigh motor | 28–43.5 |
| Thigh 80T pulley | 44–51 |
| Side wall | 51.5–53.5 |
| Hip pod | 53.5–56.5 |
| Thigh | 56.5–74.5 |
| Shin | 74.5–80.5 |
| Wheel | 81–109 |

**Hip.**
- Two RO40 Lite modules per side sit on one axis 40.5 mm behind the hip.
  Each drives a 2GT 6 mm belt (20T to 80T, 95T belt) to a concentric hip
  pulley.
- The thigh pulley is on a Ø20/12 mm outer shaft. It runs in two 6704
  bearings: one in an L-shaped hip bracket, one in the pod.
- The knee pulley is on a Ø10 mm inner shaft. It runs in MR128 bearings at
  the bulkhead and at the thigh's outer wall, with a 10×12 mm plain sleeve
  inside the outer shaft.

**Knee.**
- A 2GT 9 mm belt (40T to 40T, 110T belt) runs inside the thigh to the knee
  pulley.
- The knee pulley is on a Ø10 mm shaft fixed to the shin, in two 6700
  bearings.
- The 40T pulley gives about 170 N peak belt tension at 2.16 N·m. Checking
  the margin against the belt maker's rating is part of B3.

**Wheel.**
- The GL40 stator base bolts to the shin face, and the rotor can carries
  the hub.
- An AS5047P on the end of a fixed Ø8/5 mm axle tube reads a 6 × 1 mm
  magnet in the hub wall, with a 0.6 mm air gap.

**Electronics.**
- The 6S 850 mAh pack stands on end at the front.
- Six moteus envelopes: four flat boards above and below the motor row, and
  two vertical boards beside the battery.
- Host, IMU and power board: 40 × 10 × 30 mm beside the knee hip pulley.

**Head.**
- 24 mm LED boards sit behind the eye lenses.
- A Camera Module 3 envelope (board plus lens block, pitched 15°) sits
  behind the bezel, with an aperture cut through the pod.
- The lens offset on the board is an assumption, to be checked against the
  official mechanical drawing.

**Exterior changes needed for the head fit:**
- the camera bezel is 2.5 mm higher (`camera_drop_z` 3.5 mm);
- the web between the eye barrels is 2.2 mm taller (`web_height_ratio`
  1.5).

**Checks.**
- In the crouched, nominal and extended poses there are no overlaps above
  1 mm³:
  - between parts on different links;
  - between an internal part and any part on the same link.
- Bounds at nominal are 152 × 219 × 211 mm.
- The concept balance is unchanged: thigh 26° below horizontal; static drive
  0.39 N·m (thigh) and 0.33 N·m (knee).
- The 2 mm head-pod fillet has been skipped by the kernel since B1. Fixing it
  is part of the B3 manufacturability screen.
- FreeCAD views, including see-through cutaways, are in
  `artifacts/m10/assembly/views/`.
- The Blender renders of the assembly are pending: Blender can't open the
  NVIDIA GPU until the machine reboots after the driver update.

**Open for B3 and later:**
- motor standoffs and belt tension slots;
- cable routing through the hip pods;
- fasteners and inserts;
- the GL40's 16 V rating on the 6S bus, which needs current and voltage
  limits.

**B3 checks and adoption** (12 October 2026). This is layout-study
fidelity: purchased parts are catalog envelopes, and nothing is
hardware-validated. Outputs are in `artifacts/m10/cad/` and
`artifacts/m10/assembly/`.

**Pose sweep** (`cad/m10/sweep.py`, `sweep.json`).
- The grid is 8 hip heights from 0.072 to 0.135 m, each at axle offsets of
  ±10, ±5 and 0 mm. All 40 poses are clean:
  - no overlap between parts on different links;
  - nothing but tread below the ground;
  - the knee is behind the hip–axle line.
- The sit pose (power-off rest) is at a hip height of 0.045 m.
- Joint ranges covered, which are the limits for phase C:

  | Joint | Range |
  |---|---|
  | Thigh a1 | −188.2° to −127.9° |
  | Shin a2 | −57.7° to −3.4° |
  | Knee | 80.6° to 170.4° |

**Mass and inertia** (`cad/m10/mass_properties.py`, `mass.json`).
- CAD mass is 1.668 kg. Adding the harness (60 g), fasteners and inserts
  (60 g) and the head-pan reserve (40 g) gives 1.828 kg allocated. That is
  0.32 kg under the 2.15 kg ceiling, so the gate passes with more than the
  150 g reserve.
- Rigid bodies:
  - body 1.091 kg, including both knee drive trains;
  - thigh 79 g, shin 88 g and wheel 121 g per side.
- Each knee drive train (inner shaft and its pulleys, 65 g) has an inertia
  of 1.41 × 10⁻⁵ kg·m² about the hip axis, for the phase C transmission
  model.

**Buildability** (`cad/m10/assembly.py`).
- **Body.** Cover and tub split at hip height, four M2.5 screw columns
  with heat-set inserts, and a battery hatch in the tub floor.
- **Head.** Front and rear halves, with a Ø12 mm neck bore for the head
  harness. The 2 mm head-pod fillet, skipped since B1, is now applied:
  only edges sharper than 5° are filleted, and the camera recess is cut
  after the fillet.
- **Thigh.** Split into a body and an outer lid that carries the hip cap
  and the panel inlay, so the knee belt and the outer bearings can be
  fitted.
- **Motor mounting.**
  - Three Ø5 mm standoffs per side run from the bulkhead to the knee
    module, and a saddle on the hip bracket holds the thigh module.
  - Bulkhead slots give ±0.7 mm of x travel for hip-belt tension (as in
    M9).
  - Eccentric 6700 seats tension the knee belt.
- **Knee belt.** The 2GT 9 mm, 40T belt passes the Gates rating check:
  4.64 / 3.43 N·m at 10 / 100 rpm against a 2.16 N·m peak, a service
  factor of at least 1.59.
- **Checks.** No overlaps above 1 mm³ in the crouched, nominal and extended
  poses. Bounds at nominal are 152 × 219 × 213 mm.

**Manufacturability** (`artifacts/m10/cad/manufacturability.md`).
- Every printed part measures at least 1.0 mm of wall, with two explained
  exceptions:
  - two grazing ray samples at the camera-seat edge;
  - the 0.6 mm tread relief.
- The report also covers process, orientation, machined and purchased
  parts, fastening and assembly order.

**Exterior changes from the approved B1 look**, made for printable walls
and fits:

| Feature | Approved (B1) | Now | Why |
|---|---|---|---|
| Camera bezel | 16 mm wide, Ø8 mm lens | 11 mm wide, Ø7 mm lens | Clears the eye lenses |
| Eyes | Ø36 mm | Ø35 mm | Printable wall at the base |
| Head pod base flat | 4 mm | 1.5 mm | Pod stands 2.5 mm taller on the same neck |
| Hip cap | Proud disc | Flush inset, r 19 mm | |
| Side panel | At 40% along the thigh, 0.8 mm deep | At 50%, 1.4 mm deep | |

Also changed:
- the head web starts 3 mm further forward;
- the 2 mm head fillet now shows;
- new seams: the body split at hip height, the hatch, the head split 5 mm
  ahead of the pod centre, and the thigh lid split 2.2 mm in from the
  outer face.

Renders: `artifacts/m10/exterior/renders/`, plus cutaways in
`artifacts/m10/assembly/renders/`.

**Adoption.**
- `python -m bipe.m10.adopt_cad` writes the rigid bodies, knee drive
  train, joint ranges and sit pose to `config/robot.json` `m10.cad`.
- `bipe.m10.concept` now balances with these CAD masses. The poses were
  re-solved; the rigid bodies are in link frames and don't depend on pose,
  so one re-adoption converged.
- Assembly, sweep and mass were re-run on the new poses. Static drive
  torques:

  | Pose | Thigh slope | Axle behind hip | Thigh | Knee |
  |---|---|---|---|---|
  | Nominal | 28.2° | 9.8 mm | 0.42 N·m | 0.37 N·m |
  | Crouched | — | — | 0.46 N·m | 0.42 N·m |
  | Extended | — | — | 0.33 N·m | 0.28 N·m |

  The concept masses gave 26.0°, 12.7 mm and 0.40 / 0.33 N·m.

**Open:**
- **Harness route to the legs.** The planned Ø6 mm pass through each hip
  pod does not fit. The thigh's 80T pulley fills the side wall to
  r ≈ 27 mm, the pod ends at r 32 mm, and the thigh cavity at the hip ends
  at r 23.4 mm. Resolved below.
- **GL40 on 6S.** Its 16 V rating needs current and voltage limits in
  phase C.
- **Unchecked assumptions.**
  - The Camera Module 3 lens offset.
  - Bearing-seat fits and insert pull-out, which need test prints.
- **Mass savings, if needed later.** The pulleys and tires are modelled
  solid, so lightened pulley webs, deeper bulkhead pocketing and hollow
  TPU tires are all available.

**Harness route** (user decision, 12 October 2026: hollow shafts).

The wires run inside the knee drive, in this order:

1. A port in the inner shaft at the hip axis inside the body, 24 mm from
   the centre plane, opening downward towards the drivers.
2. A Ø5 mm bore in the inner shaft.
3. A port in a new thigh *wire gallery*.
4. A Ø2.6 mm bundle along the gallery.
5. A port in the knee shaft, then its Ø5 mm bore.
6. A Ø3.2 mm channel inside the shin.
7. The axle-tube bore, which carries the GL40 leads and the encoder board
   wiring.

The bundle (3 × AWG26 phase leads and 6 × AWG30 encoder leads) is modelled
as parts, so the overlap checks cover it. Its mass is in the 60 g harness
allowance.

Over the swept ranges, the wire twist at the ports stays within the shin
angle range (54°, against the body) and the knee range (90°, against the
thigh).

One placement is an assumption: the GL40 leads enter the axle tube through
a slot inside the motor's 8 mm through-bore.

The ports need exposed shaft between each knee-belt pulley and its outer
bearing. That is the only reason for the gallery, and it makes the thigh
3 mm thicker:

| | Before | After |
|---|---|---|
| Thigh thickness | 18 mm | 21 mm |
| Thigh outer face | 74.5 mm | 77.5 mm |
| Shin and wheel position | — | 3 mm further out |
| Overall width | 219 mm | 225 mm |

The narrower 6 mm knee belt would free the space but fails the Gates
rating, and dropping a knee bearing leaves the shin cantilevered.

The axle tube now starts 1.2 mm inside the shin, so its bore stays closed on
the inner face; the shin channel enters it through the tube wall, 0.3 mm
above the seat floor. Its seat and the knee-shaft seat are solid
cylinders, so the bores stay clear. The thinnest shin wall is now the
1.2 mm seat floor.

## Phase C plan — plant and control (detailed, 12 October 2026)

**Fidelity.** The target is a validated simulation under stated
assumptions: B3 CAD masses at layout-study fidelity, manufacturer motor
data, and assumed belt, friction and contact laws. Nothing in phase C is
hardware-validated.

**Carried-over rules.**
- The base is free, with finite motor and drive limits and physical
  contact.
- No pose constraint, state clamp or projection after reset.
- The 107-case acceptance and its M9 thresholds are unchanged.

### C1. Physical data (`config/robot.json` `m10.physics`)

1. **Separate rotating bodies.** `cad/m10/mass_properties.py` currently
   lumps the knee drive train into the body. It will export each side's
   knee drive train as its own body on the hip axis, with the body tensor
   reduced to match. RO40 rotors get the same treatment, using the
   M9-sourced core inertia (1.22 × 10⁻⁵ kg·m²; the rotor mass is an
   assumption). Re-adopt into `m10.cad`.
2. **Drives.**
   - RO40 Lite with 4:1 hip belts: the M9 manufacturer data — torque and
     current curve, FOC lag, rotor inertia — and the M9 friction screens.
   - Hip belts (2GT 6 mm, 20T→80T, 95T): stiffness, lash and damping
     recomputed from the Gates basis for the M10 span length.
   - Knee belts (2GT 9 mm, 40T:40T, 110T): the same Gates basis for width
     and span. Lash is an assumption.
   - GL40 KV70 hub motors, direct drive: torque constant, phase
     resistance, rotor inertia, rated and peak torque (0.25 / 0.5 N·m)
     from CubeMars documentation.
   - The 6S bus is limited to the GL40's 16 V rating, which sets its
     no-load speed and current limit. These are recorded as driver
     settings.
3. **Contact and collision.** The M9 contact law and materials (TPU tire,
   PA12 shell). Collision shapes are convex hulls of each link's B3
   exterior parts. No M10 clutches exist, so none are modelled.

### C2. Plant and kinematics (`bipe/m10/plant.py`, `bipe/m10/kinematics.py`)

1. **Bodies and joints.**
   - Free chassis.
   - Thigh hinge and knee-drive hinge, both on the hip axis.
   - Shin hinge at the knee, on the thigh.
   - Wheel hinge at the axle, on the shin.
   - Four RO40 rotor hinges at the motor row.
   - GL40 motors act on the wheel joints, with the reaction on the shin.
   - Joint end stops at the B3 sweep ranges.
2. **Drive solver.** Modelled on M9 `MechanicalDrives`:
   - four hip belts (rotor/4 against thigh or knee drive) and two knee
     belts (knee drive against absolute shin angle);
   - finite stiffness, damping and unilateral lash, with Coulomb belt
     loss;
   - motor current lag and torque–speed limits;
   - load-dependent journal friction.
3. **Kinematics.**
   - Serial IK and FK.
   - Posture labels 30, 40, 60 and 65 map to hip heights of 135, 103, 72
     and 60 mm, using the M9 smoothstep. The crouch excursion is 63 mm,
     against the ≥40 mm required. (Label 65 was 45 mm in the first draft
     of this plan; the 45 mm sit pose leaves 3 mm body clearance, so the
     held sit is 60 mm.) Each label sets an axle depth below the hip (hip
     height minus wheel radius) and an axle x at which the level body
     balances.
   - CoM and axle, equilibrium pitch, and gravity-support feedforward by
     virtual work.
   - Crouch reference, and wheel-encoder odometry: the encoder reads wheel
     relative to shin, so the absolute shin rate is added.
4. **Tests** (`tests/test_m10_plant.py`):
   - mass, CoM and inertia match `m10.cad`;
   - IK/FK round trip;
   - support feedforward matches a finite-difference potential;
   - unpowered drive coupling is passive (no energy gain);
   - an unpowered robot falls over, which proves there is no hidden
     stabilization;
   - a static hold at each posture with feedforward and finite torques.

### C3. Integration and control

1. **Profile registry.** Replace the scattered `profile == "m9"` checks in
   `parameters`, `articulated`, `kinematics`, `sensing`,
   `moving_experiment`, `firmware_control` and `robustness` with one
   plant-profile registry. Add `m10` to it. M4 and M9 behaviour must stay
   identical: their regression tests must pass unchanged.
2. **Sensors.** `config/sensors-m10.json`:
   - LSM6DSO IMU in the body;
   - AS5047P on each RO40 rotor, so joint angles include belt deflection;
   - AS5047P on each axle, reading the hub magnet.
3. **Controller.** `config/controller_m10.json` starts from the M9 cascade.
   Settle it at the sit, crouched, nominal and extended heights, then
   retune for the GL40's 0.5 N·m peak per wheel (M9 had 2.16 N·m).
   Gains change; physical limits and thresholds do not.

### C4. Firmware (`firmware/src/kinematics.c`)

- Add serial-leg posture, CoM, support feedforward and odometry behind a
  generated `BIPE_SERIAL` flag.
- C/Python equivalence tests, plus native and ARM builds.
- `tools/generate_firmware_config.py` has uncommitted user formatting.
  Only the serial-leg hunks are committed, on top of HEAD; the formatting
  stays in the working tree for the user.

### C5. Acceptance

- Freeze the sources, then run `bipe.robustness --profile m10`. The 107
  runs are:
  - the 70 inherited M8 cases;
  - 8 stiffness and lash corners covering the hip and knee belts;
  - the 27 locomotion cases;
  - 2 half-timestep checks.
- Write each case and its trace as it completes.
- Run the electrical screen at the limited GL40 voltage.
- If a target fails, correct the plant or controller where justified and
  repeat all 107 runs.

**Risks.**
- **GL40 torque headroom.** The measured push demand is 0.45 N·m against a
  0.5 N·m peak.
- **Knee-belt compliance** in the shin-angle loop.
- **Sensor-delay cases.** These needed retuning in M9.

**Exit.** All acceptance targets are met, or the remaining failures are
reported with their causes before phase D.

## Phase C results (12 October 2026)

**Fidelity.** A validated simulation under stated assumptions: B3 CAD
masses, manufacturer motor data, and assumed belt, friction, contact and
sensor laws. Nothing here is hardware-validated.

**Delivered.**
- C1 (c07799e): knee drive trains and RO40 rotors are separate rigid
  bodies; `m10.physics` holds the drives, belts, friction, contact, end
  stops and posture mapping. The held sit label is 60 mm hip height.
- C2 and C3.1 (673c303): `bipe/m10/plant.py` and `kinematics.py`, the
  `bipe/profiles.py` registry (M4 and M9 unchanged), and the M10 sensor,
  controller and electrical-screen configs. Plant tests: 15 pass.
- C4 (4eb0531): the `BIPE_SERIAL` portable core. C/Python parity to
  1e-10 or better; clean native and Cortex-M7 builds.
- C3 tuning (f0879fb): both pitch-rate gains lowered to 0.07 (record in
  `artifacts/m10/tuning/`).
- C5 (f33d989 runner): `python -m bipe.robustness --profile m10
  --controllers current`, outputs in `artifacts/m10/acceptance/`.

**Acceptance: 88 of 107 cases pass their criteria.** The table shows
cases that pass their criterion, out of the cases run.

| Family | Cases | Physically survive | Pass criterion |
|---|---|---|---|
| Push (hold, drive, low) | 30 | 30 | 22 |
| Slope 0–10° | 12 | 12 | 3 |
| Backlash 0–0.6° | 12 | 12 | 12 |
| Mass/CoM/friction mismatch | 16 | 16 | 16 |
| Hip and knee belt corners | 8 | 8 | 8 |
| Locomotion (27) + half-timestep | 28 | 26 | 26 |
| Half-timestep backlash | 1 | 1 | 1 |

All 70 inherited robustness cases survive. Against the M9 targets:

| Target | Result |
|---|---|
| Survive ±2 N·s pushes | Pass |
| Recover from ±2 N·s pushes | **Fail**: −2 N·s and −1.5 N·s hold pushes run away backwards (12–13 m) |
| Slopes through 10° survive | Pass |
| Slope-hold rollback ≤ 0.1 m | **Fail**: 0.369 m at 10°, 0.12 m at 8° |
| Backlash cycles ≤ 1.5° | Pass (0.05° at 0.6°) |
| Belt-corner cycles ≤ 1.5° | Pass (0.48°) |
| All 27 locomotion cases and convergence | **Fail**: 25/27; the 15 ms delay with 2× noise drive and combined cases fail. Convergence passes |

**Causes.**
1. *Knee-belt bounce with sensor delay.* The body rides on the 1:1 knee
   belts; at the nominal 69 N·m/rad this is a vertical mode near 19 Hz
   with ζ ≈ 0.004 (assumed belt damping). Hub torque reacts on the shin
   and excites it. With 5 ms transport plus 3 ms anti-alias lag, the M9
   rate gains drive it unstable; the tires hop and the airborne wheels
   spin to about 60 rad/s, so wheel odometry reads 2–3.5 m/s. Removing
   only the delay cures it; filtering, noise, INL and the posture filter
   do not. The lower rate gains avoid it in all 27 locomotion cases, but
   large backward pushes still trigger it. Ideal sensing recovers the
   same −1.5 N·s push (0.39 m rollback).
2. *Slope rollback.* The direct-drive GL40 hubs have almost no rolling
   resistance, and the low hold gains build the slope lean slowly. With
   ideal sensing the 10° hold still rolls back 0.16 m. Stiffer hold
   gains cut this to 0.06 m but become oscillatory, because the reduced
   rate damping leaves no margin.
3. *15 ms delay with 2× noise.* The same bandwidth limit, made worse by
   tripling the delay.

**Probes** (python, M9 rate gains, not adopted):
- knee belt at its upper Gates bound (140.9 N·m/rad): the crouch with
  0.6° knee lash still fails, and the −2 N·s push still runs away;
- 2 ms sensor latency: the −2 N·s push recovers, and the crouch with lash
  survives without settling;
- a 5–20 ms wheel-torque low-pass does not restore the M9 rate gains.

**Electrical screen.** The GL40s reach their 3.3 A current limit in
pushes. The RO40 peak phase current is 5.5 A. To keep the windings at
125 °C under repeated duty, the GL40s need a winding-to-air path of at
most about 6.5 K/W, and the RO40s at most about 12–15 K/W (peak-phase
convention). The sensors_hold runtime is about 51 min (85% inverter,
cold copper). These are screens, not thermal passes.

**Open decisions (user).** Options that address the causes:
- Leg damping or stiffness: a damped knee transmission or a stiffer or
  shorter knee belt. This is a design trade-off.
- Lower sensor latency, as a firmware/hardware requirement. On-target
  IMU reads would replace the 5 ms host assumption.
- A velocity estimate that does not rely on wheel odometry during hops
  (IMU fusion), or a slope feedforward. These are controller-structure
  work.
- Accept the M10 limits as reported.

## Phase C6 plan — IMU-fused velocity and slope feedforward (detailed, 12 October 2026)

User decision (12 October 2026): add an IMU-fused velocity estimate and
a slope feedforward. If the acceptance still fails, the fallback is an
on-target sensor latency requirement (option 2). Both features are
enabled only by M10 config keys, so M4 and M9 behaviour and outputs stay
unchanged.

1. **Fused velocity** (`bipe/sensing.py`, `firmware/src/estimator.c`;
   enabled by `imu_velocity_fusion_tau_s` in `config/sensors-m10.json`).
   - The state is the forward velocity of the IMU point.
   - Predict: integrate the forward horizontal specific force from the
     body-frame accelerometer, rotated by the estimated pitch and roll.
   - Correct: wheel odometry plus the rate of change of the IMU's
     horizontal offset from the axle. The offset uses the configured IMU
     position, the encoder posture and the estimated pitch.
   - The correction is first order with time constant `tau`, and its
     innovation is clipped (`imu_velocity_innovation_limit_m_s`), so a
     hop or a spinning wheel cannot drag the estimate.
   - Output: IMU-point velocity minus the offset rate, filtered with the
     existing velocity filter.
2. **Slope feedforward** (`bipe/locomotion.py`, `firmware/src/balance.c`;
   enabled by `slope_observer_tau_s` in `config/controller_m10.json`).
   - A momentum observer on the whole robot: the force is
     `F = L (M v - p)` with `dp/dt = sum(wheel torque) / r + F`, where M
     is the total CAD mass and L = 1/tau. It uses the previous wheel
     command, not simulator state.
   - The observed force gives the slope, `sin(a) = -F / (M g)`, limited
     to `slope_feedforward_limit_rad`.
   - The lean reference adds `asin(r sin(a) / h) - (M g r sin(a) / 2) / pitch_kp`.
     This is the static lean that puts the CoM over the tilted contact
     point, less the lean the wheel-torque term needs. h is the CoM height
     above the axle from the encoder posture.
3. **Parity and tests.**
   - Python and C stay identical: extended estimator and controller
     parity tests in `tests/test_m10_firmware.py`.
   - The generator emits the new constants only for m10.
   - M4 and M9 tests must pass unchanged.
4. **Tuning and acceptance.**
   - Probe the failing cases in Python: the −2 and −1.5 N·s hold pushes,
     the slope holds, and the 15 ms-delay drive and combined cases.
   - Re-tune gains only where needed, record them in
     `artifacts/m10/tuning/`, and re-run all 107 cases.

## Phase C6 results (12 October 2026)

**Fidelity.** A validated simulation under stated assumptions, as in
phase C. Nothing here is hardware-validated.

**Delivered.**
- 0f11e6b: IMU-fused velocity (`bipe/sensing.py`, `estimator.c`) and the
  slope observer and lean (`bipe/locomotion.py`, `balance.c`,
  `bipe_control_step`). Both are enabled only by M10 config keys; M4 and
  M9 are unchanged.
- 6065291: observer corrections found by the first acceptance run.
- Tuning record: `artifacts/m10/tuning/c6/` (probes, scripts, one trace).

Changes from the plan, found while tuning:
- **Bias state.** The fusion gained an accelerometer-bias state
  (`imu_accel_bias_tau_s` = 3 s). Without it, accelerometer and pitch
  bias became a velocity bias and the 60 s hold drifted 0.15 m.
- **CoM momentum.** The observer compares the wheel torque with the CoM
  momentum, M(v + ḋ), where d is the CoM's horizontal offset from the
  axle. Using M·v alone, pitching and leg motion read as a slope.
- **Torque before friction compensation.** The observer uses the wheel
  torque before friction compensation, because the hub friction absorbs
  that term. It caused about 0.9° of phantom slope.
- **Lean rate limit.** The slope lean is rate-limited to 0.1 rad/s
  (`slope_lean_rate_limit_rad_s`). A push is an external force: the
  unlimited observer stepped the lean by up to 12° at the 65° crouch,
  where the CoM sits only 24.5 mm above the axle. Before these three
  fixes, all ten `push_low` cases fell (60/70 survive).
- The hold integral limit is 1.0 (was 0.3). Rate gains are unchanged
  at 0.07.

Values: fusion τ 0.2 s, innovation limit 0.05 m/s, bias τ 3 s; observer
τ 0.2 s, slope limit 0.2 rad.

Validation: pytest passes (130 passed, 4 skipped). M10 C/Python parity
holds to 1e-10 with both features enabled. Native and Cortex-M7 builds
are clean.

**Acceptance: 91 of 107 cases pass their criteria** (was 88).

| Family | Cases | Physically survive | Pass criterion |
|---|---|---|---|
| Push (hold, drive, low) | 30 | 30 | 26 |
| Slope 0–10° | 12 | 12 | 3 |
| Backlash 0–0.6° | 12 | 12 | 12 |
| Mass/CoM/friction mismatch | 16 | 16 | 15 |
| Hip and knee belt corners | 8 | 8 | 8 |
| Locomotion (27) + half-timestep | 28 | 26 | 26 |
| Half-timestep backlash | 1 | 1 | 1 |

Notes on the counts:
- Slopes: the case-level "recovered" criterion compares pitch with
  level ground, so a held slope lean never meets it. The slope targets
  below are what apply.
- Low pushes: `push_low_-0.5`, `-2`, `-2.5` and `+2.5` survive but keep
  a late pitch oscillation of 0.8–1.8° peak-to-peak.
- `mismatch_body_com_x_m_-0.005_combined` survives with 1.1°.

| Target | Result |
|---|---|
| Survive ±2 N·s pushes | Pass |
| Recover from ±2 N·s pushes | Pass. The −2 N·s hold push recovers with 0.68 m rollback (was a 12–13 m runaway) |
| Slopes through 10° survive | Pass |
| Slope-hold rollback ≤ 0.1 m | Pass: 0.056 m at 10° (was 0.369 m) |
| Backlash cycles ≤ 1.5° | Pass, but 0.54–0.58° peak-to-peak (was 0.05°) |
| Belt-corner cycles ≤ 1.5° | Pass (0.98°, was 0.48°) |
| All 27 locomotion cases and convergence | **Fail**: 25/27. Both 15 ms-delay cases with 2× noise fall: drive at 3.1 s, combined at 1.4 s. Convergence passes |

Peak wheel torque is 0.495 N·m against the 0.5 N·m peak, in the 15 ms
combined case.

**Remaining causes.**
1. *15 ms delay with 2× noise.* This case sits on a knife edge. The
   drive case passes with the first observer version (MAE 0.0255 m/s)
   and with observer τ 0.5 s. It falls with the adopted observer, with
   fusion alone (observer off) and with a slower lean limit. τ 0.5 s
   makes `push_low_+2.5` fall instead.

   The combined case diverges at about 11 Hz while the legs extend from
   the crouch. Lower rate gains move its fall time non-monotonically
   (0.06 lasts to 11–17 s; 0.05 falls sooner). This is the knee-belt
   bounce under 18 ms of total lag; no rate gain gives a robust margin.

   Option 2 (an on-target latency requirement) cannot change this,
   because the case injects an absolute 15 ms. Passing it needs leg
   damping or stiffness, or a change to the case definition. Both are
   user decisions.
2. *Residual lean oscillation.* At standstill the tire's rolling friction
   holds a small static wheel torque, which the observer reads as about
   0.15 N. Combined with the rate limit, this is a slow lean oscillation:
   about 0.5° peak-to-peak at hold, and up to 1.8° late after
   low-posture pushes. It is within the 1.5° backlash target.

**Provenance.** The record's `runtime_source_sha256` hashes the working
copy of `tools/generate_firmware_config.py`. That copy carries the
user's uncommitted formatting on top of the committed content, and the
two are identical after `ruff format`. All other source hashes match
the commits.

User decision (12 October 2026): accept the C6 limits as reported (the
two 15 ms-delay cases and the residual hold lean oscillation) and
proceed to phase D.

## Phase D plan — falls and structure (detailed, 12 October 2026)

**Scope.** The requirement carried over from M9: the six passive tips need
structural factor ≥2 on every structural load path, and cosmetic parts
must stay attached and contained. Powered over-limit pushes and the 40°
cross-slope tumble are containment-only (M9 user decision, 5 October),
so they are reported, not gated. Bumpers are added only where a
structural path fails. Any visible change (bumpers, thicker sections)
comes back to the user before adoption, because appearance has
priority.

**Fidelity.** A conditional, undeformed-body screen: CAD masses and
sections, manufacturer bearing and belt ratings, supplier material
strengths with the M9 knockdowns
(`config/m9-impact-materials.json`), and the M10 material contact
laws. The model does not fracture. It is not a drop test.

### D1. Load-path inventory (`config/robot.json` `m10.structure`)

One entry per structural member, with its section, material, allowable
and source:
- Hip: inner shaft (7075, Ø10/Ø8 ends, Ø5 harness bore) on MR128s, outer
  shaft (7075, Ø20/Ø12) on two 6704s, and the PA12-CF hip bracket that
  carries the bearings to the 6061 bulkhead.
- Knee: Ø10 hollow 7075 knee shaft on 6700s in eccentric seats.
- Belts: 2GT 6 mm hip belts (20T→80T, 60 N pretension per span) and 2GT
  9 mm knee belts. Checks: Gates rated torque at the small pulley, the
  tooth-jump tension and positive slack-side tension.
- Printed PA12 thigh and shin: sections at the hip root, mid-span, knee
  and axle stations.
- Wheel: the 7075 axle tube (Ø8/Ø5) in the shin, and the GL40 stator and
  rotor bearings. The bearing ratings come from the CubeMars drawing, or
  are a stated assumption if CubeMars does not publish them.
- Shell and head retention: the M2.5 screws into heat-set inserts (body
  columns, head screws), the neck spigot, ears, hatch and thigh lid.
  Cosmetic parts are checked for attachment only.

Sources: static ratings (C0) from manufacturer catalogs for 6704, MR128
and 6700; Gates ratings; insert pull-out data. Sections come from
FreeCAD slices of the B3 CAD (`cad/m10/sections.py` →
`artifacts/m10/cad/sections.json`), not from hand estimates.

### D2. Fall runner (`bipe/m10/falls.py`)

- Six passive tips on the M10 plant:
  - forward and backward from ±10° pitch;
  - sideways from a roll that puts the CoM beyond the tire's outer
    edge, plus 5°;
  - each at rest and at 0.3 m/s.
- Unpowered throughout: zero commanded current, with the RO40 rotor
  inertia, belt stiffness and lash, and bearing and belt friction kept.
- Material contacts on the exterior hulls and the tire. No state clamps
  or pose projections.
- Records per case:
  - first non-tire contact (time, part, kinetic energy);
  - peak contact depth per part;
  - per-sample internal wrenches at every joint and belt.
- The worst case is repeated at half the timestep for convergence.
- The containment cases (±6 N·s and ±10 N·s powered pushes, 40°
  cross-slope with the C core and sensors) are run and reported only.

### D3. Structural screen

- Factors per load path, from the solved wrenches each sample:
  - shafts: von Mises from bending, torsion and axial load;
  - bearings: P0 = max(Fr, 0.6 Fr + 0.5 Fa) against C0, with either
    bearing assumed to locate;
  - belts: transmitted torque against the rated torque, and slack-side
    tension ≥ 0;
  - printed sections: bending plus axial against the PA12 weakest-Z
    allowable;
  - fasteners and inserts: the M9 capacity functions.
- Contact forces on the exterior parts load their retention.
- Output `artifacts/m10/falls/`: configuration, per-case traces and
  factors, and an acceptance summary with the minimum factor per path.

### D4. Remedies where needed

For each failing path, the lightest change that reaches factor ≥2:
section or material, a bearing size, belt width, or a frame-mounted
foam bumper (LD45) that contacts first with 15–20 mm stroke. The
budget is the 2.15 kg allocation with ≥150 g reserve. Visible changes
are rendered and returned to the user before CAD adoption. Re-run D2
and D3 after adoption.

#### D4 detailed plan (13 October 2026)

User decisions (13 October 2026):

- Sideways: a foam ring on each wheel's outer face, plus 6900 knee
  bearings and a 7075 shin.
- Belly: a skid pad under the tub.
- Ears: printed in flexible TPU, with a checked screw retention.
- The lowest operating posture becomes command 60 (72 mm hip height).
  60 mm becomes a resting pose.
- The inherited `unpowered` acceptance case passes when the unpowered
  robot does not stay standing: pitch ≥30°, or the body or skid on the
  floor.
- The thigh's hip end gets a 7075 flange.

Steps:

1. Hip flange screen. Before any CAD change, a `thigh_hip_flange`
   screen assumption credits the open thigh_closed rows near the hip
   (the inner-wall bore cuts the slice) with the closed-cell area of the
   nearest closed row, for Bredt torsion. Re-run the protection
   candidate to confirm the thigh reaches ≥2 and to list what remains.
2. Internal remedies for the remaining paths, lightest first:
   - ear-retention screws (count or size);
   - hip and knee belt pretension or width;
   - the knee outer bearing.
   Bumper sizing (skid and rings bottom at strains >0.5): sizes and
   shapes are visible, so they are rendered for the user before
   adoption.
3. CAD:
   - the flange is integral with the outer shaft's thigh end: 7075,
     1.5 mm thick, Ø36 mm;
   - six M2 countersunk screws from the inner wall's outer face into
     tapped holes in the flange;
   - the knee-belt pulley stack moves 1.5 mm outboard, so the thigh is
     1.5 mm thicker and the overall width becomes about 228 mm;
   - re-slice `sections.py` with the flange closing the hip-end section
     (Bredt area only; bending properties stay PA12-only);
   - then the sweep, mass adoption and plant.
4. Low posture and acceptance criteria:
   - in m10 robustness, the low mode starts at command 60;
   - the m10 `unpowered` criterion follows the decision above. The M4
     and M9 profiles are unchanged.
5. Re-run the six falls at both timesteps (0.125 and 0.0625 ms), and
   the acceptance suite.

Requirement change (user decision 13 October 2026, after the v2
screens): every sideways tip ends in a secondary strike of roughly
250–670 N on the opposite wheel. The robot either rocks back upright or
tumbles. That strike overloads the hip drive (6704 bearings 0.5–1.3, hip
belts 0.74–1.3) whatever the padding, and a tenfold softer tire only
reaches about 1.2. The sideways tips are therefore containment cases
with inspection after the fall:

- cosmetic parts must stay attached and contained;
- no factor-≥2 requirement on the hip drive. The overload is a recorded
  risk: possible bearing marking, and a belt tooth-jump that needs the
  joint re-zeroed.

The forward and backward tips keep factor ≥2 on every structural path.
Visible remedies are rendered in Blender for the user before CAD
adoption.

#### D4 CAD adoption plan (13 October 2026)

The user approved the protection_v2 look renders and its adoption into
the CAD.

Bearings: the peak static loads P0 over the v2 runs, at both timesteps,
set the bearing choice.

| Bearing | fwd/back P0 (N) | needed C0 at ×2 (N) | sideways P0 (N) | adopted (C0, N) |
|---|---|---|---|---|
| BulkheadBearing | 239 | 478 | 276 | 688A 8×16×4 (715) |
| InnerShaftEndBearing | 221 | 441 | 220 | 688A 8×16×4 (715) |
| KneeInnerBearing | 327 | 654 | 727 | 6800 10×19×5 (925) |
| KneeOuterBearing | 196 | 391 | 873 | 6800 10×19×5 (925) |

- The 6900 from the screen is not needed. The sideways tips are
  containment cases, and the 6800 already carries the sideways peak at
  ×1.06. The 6900 (22×6) also does not fit the knee boss.
- The 688A end bearing is 0.5 mm wider than the MR128. With the 1.5 mm
  flange in the stack, the harness gallery between the knee-belt hip
  pulley and the end bearing would shrink to 2.6 mm, under the Ø3 port.
- So the thigh grows 2.0 mm, not 1.5 mm. The gallery keeps its present
  3.1 mm, and the tire-to-tire width becomes 228 mm, the width the user
  approved (B3: 224 mm).

Steps:

1. Config:
   - `thigh_y` outer, `shin_y` and the wheel `y_center` move 2.0 mm
     outboard, and so does the knee_angle stop station.
   - Knees become 6800; the bulkhead and end bearings become 688A.
   - The protection_v2 values become the defaults, with the bumpers
     enabled.
   - The new parts get densities: LD45 foam (47 kg/m³) and the TPU
     skins and ears.
2. CAD (`cad/m10/assembly.py`):
   - `HipFlange` is 7075, Ø36 × 1.5 mm, on the cavity side of the inner
     wall. It is machined integral with the outer shaft, but kept as its
     own solid so the section can include it. Six M2 countersunk holes
     on a Ø30 circle.
   - The knee-belt pulleys move 1.5 mm outboard.
   - New bearing seats; the knee boss grows to Ø23 for the Ø19 6800.
   - The bulkhead bores become Ø16, leaving a 0.5 mm 6061 web between
     the left and right seats.
   - `SkidPad` and `WheelRing` are the plant shapes: LD45 foam behind a
     1 mm TPU skin. The skid sits under the tub, over the bulkhead
     feet; each ring sits on its hub disc.
   - The ears are TPU, with a 3 mm local boss inside the head wall at
     each ear root for the three inserts.
   - The knee-angle stop is a TPU pad at 16 mm radius with its
     striker.
   - The FCStd output goes to a separate path, so the user's modified
     `artifacts/m10/assembly/assembly.FCStd` stays as it is.
3. Headless pipeline:
   - exterior, assembly (overlap check), sweep, sections, mass
     (adopted into `m10.cad` if within the ledger) and
     manufacturability;
   - the protection mesh and the Blender renders.
   - `sections.py` adds a torsion-only thigh row set that includes the
     flange. A closed ring now requires a face with an inner wire.
   - The falls screen uses those rows in place of the
     `thigh_hip_flange` assumption.
4. Re-run the six falls at both timesteps, then the acceptance suite.
5. D5: record the results, the mass ledger and the remaining sideways
   risks.

#### D4 knee stop and skid colour plan (13 October 2026)

User decisions, from the look study
(`cad/m10/knee_stop_options.py`, renders in
`artifacts/m10/knee_stop/` and `artifacts/m10/review/`):

- Knee stop: option B, the arc slot. Option A (knuckles) was rejected
  for its visible heel spur.
- The skid stays the battery-hatch retention: its screws hold the hatch,
  and the skid comes off for a battery swap.
- The skid's display colour becomes a grey close to the tire's. The wheel
  rings stay black.

Arc-slot geometry:

- The thigh's knee end grows from 19 to 23 mm radius
  (`m10.exterior.thigh.knee_radius`).
- A groove in the thigh's outer face (in the lid) runs on a 14.5 mm pin
  circle, 6 mm wide radially (r 11.5–17.5) and 4 mm deep.
  - The floor is at y 75.5 mm, 1.2 mm above the lid split.
  - The lid is solid behind the groove from the split to the outer wall
    (a boss in the thigh cavity out to the groove's outer radius plus
    2 mm).
- A TPU 88A pad at each groove end: 12 mm along the arc, a 6 × 4 mm
  face (24 mm², as in the falls).
- The pin is an ISO 8734 Ø5 × 8 m6 hardened steel dowel.
  - It is pressed 4.4 mm into a blind hole in the shin's inner face
    (1.6 mm skin left on the 6 mm shin) and reaches 3.6 mm into the
    groove, 0.4 mm clear of the floor.
  - It sits 22° off the shin axis, so it clears the Ø3.2 harness channel
    that runs along the axis (about 1.3 mm web).
- The groove ends sit where the pin meets a pad exactly at the plant
  stops: knee angle 75° and 176° (unchanged).

Physics (`m10.physics.end_stop_pads.joints.knee_angle`):

- radius 0.016 → 0.0145 m;
- station y 0.0795 → 0.0775 m (groove mid-depth);
- area and thickness are unchanged.

Loads, from the D4 falls:

- The peak knee-angle stop torque is 3.95 Nm (sideways_rest). In the
  forward and backward tips it is 0.
- At 14.5 mm that is 272 N. The Ø5 pin sees about 14 MPa shear and
  about 51 MPa bending over a 2.3 mm lever, and the PA12 groove end
  about 11 MPa bearing.
- The lid carries the load into the thigh through its rim screws and
  the outer knee-bearing seat.

Steps:

1. Config:
   - the knee radius;
   - a new `m10.internals.knee_stop` block;
   - the pad radius and station;
   - materials for `KneeStopPad` (TPU) and `KneeStopPin` (steel).
2. CAD:
   - `exterior.py` cuts the groove and adds the pads, the pin and the
     pin's blind hole in the shin.
   - `assembly.py` keeps the solid boss behind the groove in the thigh
     cavity, and reports the gap from pin to harness channel.
   - The skid's display colour becomes grey.
3. Headless pipeline, as in the D4 CAD adoption: exterior, assembly,
   sweep, mass, adopt (twice), sections, manufacturability, renders.
   - Checks: no overlaps, sweep clean, a contact check at both stops
     (pad gap 0, no overlap), pin web ≥1 mm, groove floor ≥1.2 mm.
4. Re-run the six falls at both timesteps. The 10% higher stop force
   lands on the knee bearings, which are already a recorded sideways
   risk.
5. The acceptance suite, after the diagnosis of the current regression
   (`slope_up_4`, `push_drive_+2.5` and a C encoder-rate rejection that
   aborted the run).

Addenda (13 October 2026):

- Slope regression diagnosis. The D4 acceptance failed `slope_up_4`, and
  `slope_up_8` and `slope_up_10` aborted the run on the C encoder-rate
  check. The cause is a plant bug, not the design:
  - `reset` tilts the ground plane after compilation, so the world body's
    bounding volume stays the flat half-space z ≤ 0.
  - Once the bumper rings gave each wheel body a second geom, MuJoCo's
    midphase began culling the tire–ground pair uphill (axle x > about
    3 cm). The tires lost contact (0.6 mm penetration, no contact) and
    the robot dropped.
  - Fix: `_refit_world_bvh` refits the world bound to the tilted plane.
  - `test_tilted_ground_keeps_tire_contacts_away_from_the_origin` checks
    that the midphase keeps every contact the brute-force pair test
    finds, uphill and downhill. It fails without the fix.
- Hip-belt pretension (user decision): the first knee stop B falls left
  `forward_0p3` at dt/2 with the thigh hip-belt slack at factor 1.98
  (dt: 2.23; before B, 2.01).
  - The pretension rises from 60 to 65 N per span, in the screen and in
    the plant's hip bearing friction.
  - Gates basis (Light Power & Precision manual, p65, Tables 6–8):
    - Formula 1 gives 57 N.
    - The general 2MGT 6 mm static tension is 10 lb (44 N) per span;
      registration drives start at 1.5–2.0× that (67–89 N).
    - The rated working tension at 20 grooves is 39 lb (173 N).
  - The hip bearings' pretension load rises 8%.
  - The falls and the acceptance are rerun at 65 N.
  - Superseded: at 65 N the forward_0p3 dt/2 hip pod bearing fell to
    1.68. The user chose 62.5 N, which is in the config
    (`m10.physics.belts.hip_pretension_basis`).
- Hip belt length (user decision, 14 October 2026). The Grok parts check
  found that the 95T (190 mm) hip belt is not a stock Gates length, and
  that it had carried the M9 202 mm belt's 0.95 length factor. Checked
  against the Gates manual (printed p.15 stock list, p.18 factors):
  - The belt becomes the stock 2MR-192 (96T). The centre distance moves
    from 40.5 to 41.53 mm (open-belt length), the motor row moves with
    it, and the span is 36.9 mm.
  - Cord stiffness 594.4 N·m/rad. Nominal hip k is 350 N·m/rad (range
    119–713), damping 0.0292 N·m·s/rad.
  - The length factor is 0.90 (170–196 mm). The 20-groove 6 mm torque
    rows at 800–2000 rpm in `config/m9-impact-materials.json` are
    corrected to the manual (6.76…5.39 lb·in). M9's own 202 mm belt
    keeps 0.95.
- Corner sleeve (studied and removed, 14–17 October 2026). With knee
  stop B, sideways_0p3 struck the tub's front-lower vertical edge at
  95 N and 4.25 mm, past the shell law's 4.0 mm travel. The user first
  chose a TPU corner bumper:
  - an LD45 arc sleeve round each front vertical edge, on a PA12
    carrier. At 15, 12 and 10 mm it passed all six falls at both dt.
    Bottom-thick tapers left HeadFront at 4.1–4.2 mm.
  - The user rejected every sleeve look (renders in
    `artifacts/m10/review/corner_sleeve*`), since the reference robot has
    none. The sleeves were removed from the plant, CAD and config.
  - Without the sleeves, sideways containment failed on travel at two
    spots, both just past the 4.0 mm M9 shell-law assumption:
    - the tub corner in sideways_0p3 (4.23–4.26 mm, 87–100 N);
    - the left thigh's knee end in sideways_rest (4.19–4.28 mm).
    With the backing removed, the walls deflect 4.96 mm (tub) and
    4.40 mm (thigh).
- Measured shell backing (user decision, 17 October 2026). At those two
  spots the strip law's backing sits at the clearance measured in the
  CAD, not at the 4 mm assumption (`m10.physics.contact.shell_backing`).
  - Probe: `cad/m10/shell_clearance.py` →
    `artifacts/m10/cad/shell_clearance.json`. It ray-casts from the
    struck outer surface along the inward contact normal, on a 5 × 5
    grid at ±6 mm, against every other part at the nominal pose. Rays
    through a solid section (thicker than 5 mm: the knee-stop boss) are
    reported but not counted.
  - Gaps: tub 7.6 mm (its own screw column; battery about 32 mm). Each
    thigh 6.65 mm (its far wall at the boss edge; knee belt 7.2 mm).
    Config travel: 7.6 and 6.6 mm.
  - The measured gap holds only within 6 mm of a probed spot. The falls
    record each listed part's deepest strike on and off the spots, and
    fail an off-spot strike past the default 4 mm.
  - Wall stress stays a recorded sideways risk. M9's strip law (2 mm
    PA12, 50 mm strip, 78 mm span, notch 1.5) gives about 21–24 MPa at
    the free deflections, about 1% bending strain. Against 60% of the
    42 MPa supplier strength that is a factor of 1.0–1.2, and against
    the supplier value 1.7–2.0. M10 does not apply M9's force-based
    printed-strength check (capacity about 29 N). Inspect the tub
    corners and the thighs' knee ends for dents or whitening after a
    side fall.

### D5. Exit

The six passive tips reach factor ≥2 on every structural path, at both
timesteps. Cosmetic parts stay attached. The mass ledger is within the
allocation. Otherwise the remaining failures and the candidate remedies
are reported with their mass cost.

## Phase D results (achieved)

### Acceptance re-run after the contact correction (12 October 2026)

Commit aa66305 fixed the contact mapping, so material contacts now apply
their force law on the multibody robot, and added physical end-stop pads.
Because that changes the plant, the full 107-case acceptance was re-run
(`artifacts/m10/acceptance/`; every runtime source hash matches the
committed files).

- Outcome unchanged: only `all_27_and_convergence` fails. The 79 cases
  that passed physically before still pass.
- What moved:
  - `push_hold_+2.5` no longer recovers. The +1 direction recover margin
    falls from 2.5 to 2 N·s and still meets the 2 N·s target.
  - Backlash peak-to-peak at 0.6 crouch rises from 0.58° to 0.94°
    (limit 1.5°).
  - Belt range rises from 0.98° to 1.05° (limit 1.5°).
  - Slope rollback rises from 0.056 to 0.057 m.

### D4 CAD adoption (13 October 2026)

The protection_v2 set is now in the CAD and is the default design:
`m10.physics.bumpers` enabled, the structure screen defaults set to the
adopted parts, and `m10.cad` re-adopted. Fidelity is still a layout
study plus simulation under the stated assumptions. Nothing here is
hardware-validated.

What changed in the CAD (`cad/m10/assembly.py`):

- Thigh 2.0 mm thicker; shin and wheel 2.0 mm outboard. Tire-to-tire
  width is 228 mm (B3: 224 mm); with the wheel rings, 248 mm.
- 7075 `HipFlange`, Ø36 × 1.5 mm, on the cavity side of the inner wall.
  Six M2 countersunk screws sit on a Ø26 circle with the first 30° from
  the thigh axis. That keeps the holes between the 8.4 mm and 14 mm
  section stations and leaves a 1.9 mm ligament to the Ø20 bore. The
  knee-belt pulleys move 1.5 mm outboard; the harness gallery stays
  3.1 mm.
- Bearings:
  - 6800 knees in a Ø23 knee boss, with 2 mm inner bosses inside the
    thigh cavity so each seat holds the full 5 mm width;
  - 688A bulkhead and end bearings; the bulkhead keeps a 0.5 mm 6061
    web between the left and right seats.
- Pads (LD45 foam inside a uniform 1.2 mm TPU 88A cup):
  - `SkidPad`: 130 × 80 × 10 mm with 12.5° end bevels. It is screwed
    through the tub floor into the bulkhead feet.
  - `WheelRing`: 12° cone. It sits on a PA12 `WheelRingCarrier` that
    fills the hub recess around the blades.
  - The skin is 1.2 mm rather than 1 mm, so it passes the 1.0 mm
    printed-wall screen. The plant still neglects the skin.
- TPU ears:
  - Each root is trimmed square where the outer face stands 1.5 mm off
    the pod, then clipped to the pod. A plain clip left 0.15 mm feather
    edges. The ear base is about 5 mm shorter than in B3.
  - Under each root is a 3 mm sector boss inside the barrel wall for the
    three M2 inserts. The screws and inserts are in the fastener
    allowance; the TPU root tab is not modelled.
- 7075 shins and hip brackets (material change only).
- The FCStd is written to `artifacts/m10/assembly/d4/` (`M10_FCSTD_DIR`),
  so the working copy of `artifacts/m10/assembly/assembly.FCStd` is
  untouched.

Checks (headless, `artifacts/m10/cad/`, `artifacts/m10/assembly/`):

- Overlaps: none, on different links or on internal parts, in the
  crouched, nominal and extended poses.
- Sweep:
  - the 40-pose grid is clean;
  - the sit pose is 0.054 m, the lowest pose with the skid on the floor
    (B3: 0.045 m with no skid);
  - covered ranges: a1 −192.3…−128.3°, a2 −58.1…−5.0°, knee
    80.4…169.6°. All lie inside the plant's end stops.
- Mass: CAD 1.788 kg; allocated 1.948 kg; 0.202 kg under the 2.15 kg
  ceiling, so the ≥0.15 kg reserve holds. The increase over B3 is 123 g:
  - shins +45 g and hip brackets +12 g (7075);
  - bearings +21 g;
  - pads, skins and ring carriers +34 g;
  - hip flanges +7 g;
  - head bosses and trimmed TPU ears +2 g.
- Manufacturability: no new part below the 1.0 mm screen. The only
  entries are the two HeadFront samples and the 0.8 mm tread knobs,
  both from B3.
- Sections: with the flange, every `thigh_torsion` station is a closed
  cell (`closed_ring` now needs a face with an inner wire). The last
  station moves from 0.88 to 0.85, 1 mm outside the 19 mm 6800 bore.
  This matches B3's 0.88 relative to the 15 mm 6700 bore.

Falls (`artifacts/m10/falls/`, dt 0.125 ms, and `artifacts/m10/falls/dt_half/`, dt 0.0625 ms):

| Case | worst path (dt) | factor | factor (dt/2) |
|---|---|---|---|
| forward_rest | right thigh hip-belt slack | 2.51 | 2.32 |
| forward_0p3 | right thigh hip-belt slack | 2.21 | 2.01 |
| backward_rest | left bulkhead bearing | 2.80 | 2.89 |
| backward_0p3 | thigh hip-belt / knee-belt slack | 2.58 | 2.40 |
| sideways_rest | left hip pod bearing (containment) | 0.47 | 0.49 |
| sideways_0p3 | left hip pod bearing / hip-belt slack (containment) | 0.85 | 0.96 |

- Forward and backward reach factor ≥2 on every path at both timesteps.
  The tightest is the thigh hip-belt slack in forward_0p3 at dt/2, at
  2.01. The skid takes the first strike in every forward and backward
  case, at a peak strain of 0.25–0.49. forward_0p3 at 0.49 is close to
  the 0.5 end of the supplier curve.
- Sideways containment holds at both timesteps: ears unloaded, no
  shell-travel failure, retention factors far above 1. The summary's
  `requirement` block applies the 13 October rule and passes.
- Recorded sideways risk (minimum over both timesteps):
  - hip 6704 bearings 0.47;
  - thigh hip belt 0.86 (slack);
  - left knee outer bearing 0.98 and inner 1.19. These are 6800s; the
    screen's 6900 gave 1.45 but does not fit.
  - hip bracket arm 1.02;
  - knee belt 1.55 (slack);
  - left thigh section 1.66;
  - the skid bottoms at 0.73 when the robot rolls onto it.
  Inspect after a side fall: hip and knee bearing marking, and belt
  tooth-jump (re-zero the joint).

Open items:

- The knee-angle stop pad that the falls use (TPU, 12 mm long, 24 mm²,
  at 16 mm radius) does not fit the knee. The thigh's knee end is a
  19 mm radius with a 5 mm edge fillet, and an internal stop has room
  for about a third of the pad volume. None of the three end stops are
  in the CAD; they are modelled in the plant only. This needs a visible
  design decision.
- The skid's front bevel covers the rear 28 mm of the battery hatch, so
  the skid comes off (and retains the hatch) for a battery swap.

### D4 knee stop B, stock hip belt and measured shell backing (17 October 2026)

Design state: knee stop B (arc slot) and the grey skid; hip-belt
pretension 62.5 N; Gates 2MR-192 hip belt at 41.53 mm centres; no corner
sleeves; and the measured shell backing at the tub corners and the
thighs' knee ends (plan addenda above).

CAD (headless pipeline, two adoption passes; FCStd in
`artifacts/m10/assembly/d4/`):

- No overlaps. The sweep grid is clean, and the sit pose is 0.054 m.
  - Covered ranges: a1 −194.95…−128.62°, a2 −58.55…−6.25°, knee
    80.25…169.07°.
- Mass: CAD 1.8005 kg, allocated 1.9605 kg. That is 0.1895 kg under the
  2.15 kg ceiling, so the ≥0.15 kg reserve holds.
- No new part falls below the 1.0 mm wall screen.
- Every `thigh_torsion` station is closed.
- Clearance: `artifacts/m10/cad/shell_clearance.json`. Tub 7.6 mm,
  thighs 6.65 mm.
- Renders: `artifacts/m10/assembly/renders/` and
  `artifacts/m10/exterior/renders/`.

Falls (`artifacts/m10/falls/`, dt 0.125 ms; `artifacts/m10/falls/dt_half/`, dt 0.0625 ms):

| Case | worst path (dt) | factor | worst path (dt/2) | factor (dt/2) |
|---|---|---|---|---|
| forward_rest | right thigh hip-belt slack | 2.63 | right thigh hip-belt slack | 2.42 |
| forward_0p3 | right thigh hip-belt slack | 2.35 | right thigh hip-belt slack | 2.11 |
| backward_rest | right bulkhead bearing | 2.79 | left bulkhead bearing | 2.85 |
| backward_0p3 | right knee-belt slack | 2.48 | right knee-belt slack | 2.41 |
| sideways_rest | left hip inner bearing (containment) | 0.49 | left hip inner bearing (containment) | 0.49 |
| sideways_0p3 | left hip pod bearing (containment) | 1.28 | left hip pod bearing (containment) | 1.33 |

- Forward and backward reach factor ≥2 on every path at both
  timesteps. No energy is injected.
  - The lowest hip-belt rating factor (with the 0.90 length factor) is
    2.43, forward_0p3 at dt/2.
  - The skid strikes first in every case, at a peak strain of 0.25–0.48.
- Sideways containment holds at both timesteps, and the summary's
  `requirement` block passes.
  - The ears are unloaded, and retention is far above 1.
  - Tub corner: 4.71–4.96 mm against the 7.6 mm gap. Thigh knee end:
    4.40–4.97 mm against 6.6 mm. Every strike past 4 mm lands within
    6 mm of a probed spot. Off-spot depths stay ≤1.9 mm.
  - The skid no longer bottoms in a side fall: peak strain 0.40–0.47,
    against 0.73 before.
- Recorded sideways risk (minimum over both timesteps):
  - hip 6704 bearings 0.49;
  - left knee outer bearing 1.03 and inner bearing 1.27;
  - hip bracket arm 1.05;
  - knee belt 1.43 (slack);
  - thigh hip belt 1.43 (slack);
  - left thigh section 1.71.
  - The printed walls at the tub corners and the thighs' knee ends sit
    at a factor of about 1.0–1.2 against the knocked-down PA12 strength.

  Inspect after a side fall:
  - hip and knee bearing marking;
  - belt tooth-jump (re-zero the joint);
  - dents or whitening at the tub's front-lower corners and the thighs'
    knee ends.
- sideways_0p3 improved over the sleeve-free run with the 4 mm backing
  (hip pod bearing 0.58–0.69 → 1.28–1.33): the tub no longer bottoms
  on a fictitious backing.

Acceptance (`artifacts/m10/acceptance/`, 107 cases, `python -m
bipe.robustness --profile m10 --controllers current`; resumed after a
pause with `--resume`, which checks the config and runtime-source hashes):

- Robustness physical passes: 70/70. Targets: all met except
  `all_27_and_convergence`. The half-timestep convergence and backlash
  checks pass.
- Inherited cases: 26/27. The only failure is
  `delay_15ms_noise_x2_combined`, which was accepted at C6.
  `delay_15ms_noise_x2_drive` now passes (it failed in the 13 October
  run).
- Compared with the 13 October run:
  - slope-hold 10° rollback 0.059 m (was 0.057);
  - belt-range pitch 1.01° (was 1.05);
  - backlash at 0.6° 0.94° (was 0.95);
  - the largest passing push_hold +1 and push_low −1 rise from 2.0 to
    2.5 N s.
- The slope cases log `recovered: False` from 4° up, as before: the
  late pitch error on a slope exceeds the 2° settle band. Every slope
  case passes physically.

### Parts-check corrections (17 October 2026)

I re-checked the leftover findings of the Grok parts check (tag
`grok-parts-check-2026-10-14`) against the manufacturers' documents:

| Item | Config before | Manufacturer | Change |
|---|---|---|---|
| 2MGT belt thickness (hip and knee belts) | 1.1 mm | 0.060 in = 1.52 mm overall (ref), Gates manual Table 15; 1.10 mm is the minimum flange height (Table 22) | 1.52 mm in the CAD (+4.8 g CAD mass with the belts' volume) |
| Shell screw inserts | M2.5 screw in a 3.6 × 6 mm hole (the M2 insert's overknurl) | SPIROL INS 19/M2.5 long, item 150989: overknurl 4.75 mm, length 5.74 mm, pilot 3.91 mm, recommended hole 3.99 +0.08/−0 mm | 3.99 × 6 mm hole and the part number; screw column wall 2.0 mm |
| Battery mass | 0.165 kg, labelled "catalog" | GensTattu TA-RL5-150C-850-6S1P: net 145 g ±20 g | Value kept, as the conservative upper tolerance; label corrected |
| Battery envelope | 32 × 47 × 65 mm, "M9 catalog envelope" | 60 (±5) × 30 (±2) × 45 (±2) mm, L × W × H | Value kept, as the tolerance maximum; label corrected |
| Inner-shaft bearings | `internals.inner_shaft` names MR128 8 × 12 × 3.5 | — | Label only: the CAD already takes 688A (8 × 16 × 4) from `structure.bearings` |
| Bearing friction basis | "hip 6704 and MR128 at a 20 mm bore" | — | Text: the knee-drive joint's 688A bearings (8 mm bore) are lumped at 20 mm, which overstates their small moment (about 2 vs 0.8 mN m) |
| GL40 insulation class | F for both motors | CubeMars GL40 KV70: class H (180 °C) | Text: RO40 F, GL40 H. The 125 °C screen limit stays, and is conservative for the hubs |

The belt and insert changes alter the CAD, so the pipeline, falls and
acceptance were rerun.

Results after the corrections (commit f9998f2):

- CAD: no overlaps; sweep grid clean. CAD 1.8053 kg (+4.8 g), allocated
  1.9653 kg, 0.1847 kg under the ceiling. Manufacturability and the
  shell clearance (7.6 / 6.65 mm) are unchanged.
- Falls at both timesteps:
  - Forward and backward reach factor ≥2. The minimum is 2.25, up from
    2.11 (forward_0p3 thigh hip-belt slack, dt/2). The lowest hip-belt
    rating factor is 2.57. No energy injection.
  - Sideways containment passes: tub 4.72–4.96 mm, thigh 4.64–5.01 mm.
  - Recorded sideways risk: hip 6704 bearings 0.48, left knee outer
    bearing 1.01, hip bracket arm 1.03, knee belt slack 1.37.
- Acceptance (107 cases, 8 workers): robustness 70/70; all targets met
  except `all_27_and_convergence`. Inherited cases 26/27; the only
  failure is `delay_15ms_noise_x2_combined`. Convergence passes.
  - Slope-hold 10° rollback 0.059 m.
  - Belt-range pitch 0.99°.
  - Backlash at 0.6° 0.94°.
- Renders refreshed (`artifacts/m10/assembly/renders/`,
  `artifacts/m10/exterior/renders/`). The 0.4 mm thicker belts show only
  in the cutaway views.

#### D4 hip and knee-drive end stop look study (plan, 18 October 2026)

The `thigh_a1` and `knee_drive_a2` stops are modelled in the plant only.
In the falls their pads peak at about 2.7 N m (strain ≤0.13). Two options
are built for review in `cad/m10/end_stop_options.py`, with renders in
`artifacts/m10/end_stop/<option>/`:

- A, hidden: both stops are internal. A steel ISO 8734 Ø5 dowel fixed to
  the body runs in an arc groove (r 17–23 mm, 4 mm deep) in the face of
  the joint's 80T pulley. A TPU pad sits at each groove end: 8 mm long
  along the arc, 6 × 3.8 mm face. The pin circle is 20 mm.
  - Hip: Ø5 × 8 dowel at 200° (below the z = 0 tub/cover split). It is
    pressed through the tub side wall into the hip housing, leaving 1 mm
    of skin. The groove is in the outboard face of the thigh 80T pulley.
  - Knee drive: one Ø5 × 16 dowel through the 8 mm bulkhead at 180°. Its
    ends run in grooves in the inboard faces of both knee 80T pulleys.
  - Peak fall stop torques 2.66 N m (hip) and 3.66 N m (knee drive) give
    133 N and 183 N on the pin. Pad strain is about 0.11 at 183 N.
    Plant pad as r 0.020, A 22.8e-6, t 0.008: 86 N m/rad. Today's plant
    pad is 90 N m/rad.
- B, visible hip: TPU-faced PA12 lugs on the hip housing's outer face at
  30 mm radius, 12.5 mm tall. Each is struck by the thigh's flank 16.6 mm
  from the flank's tangent point, which is also the moment arm: 160 N at
  2.66 N m. At the nominal pose the lugs clear the thigh's hip end by
  3.0 mm. The knee drive is as in A.

Checks: at both stop poses (a1 −195°/−120°, a2 −65°/+2°) the pin meets
the pad with zero gap and no overlap; at the nominal pose the new parts
overlap nothing. After the user's choice, the adopted option goes into
`assembly.py` and the config, the plant pads take the new radius, area
and length, and the pipeline, falls and acceptance are rerun.

Results (`checks.json` and `renders/` in each option folder):

- The pad-to-pin and pad-to-thigh gaps are 0.00 mm at both stop poses,
  with 0.00 mm³ overlap.
- The new parts overlap nothing, except a 0.06 mm³ boolean sliver
  between the hip pad and its pulley in A.
- Dowel seats: hip in BodyTub and HipHousing; knee drive in Bulkhead.
- The design choice is the user's.

#### D4 end stop adoption: option A (plan, 18 October 2026)

The user chose option A (hidden slots).

Design change from the study: the hip dowel moves to the hip bracket. In
the study it sat in the PA12 tub wall. A rigid pin in an elastic seat
peaks at (F/dL)(4 + 6a/L). For the tub-wall seat (F 133 N, d 5, L 4.8,
a 2) that gives about 36 MPa, above the 25.2 MPa PA12 allowable.

- Hip:
  - An ISO 8734 Ø5 × 10 dowel is pressed through the machined 7075 hip
    bracket arm at 0° (x = 20 mm, arm z ±6 mm).
  - It runs in a groove in the inboard face of the thigh 80T pulley.
  - Seat pressure is about 43 MPa in 7075, against the 402 MPa allowable.
  - Pin bending is about 36 MPa.
- Knee drive: as in the study. One Ø5 × 16 dowel through the 6061
  bulkhead at 180°, with a groove in each knee 80T pulley's inboard face.
- Both stops share the groove geometry: pin circle 20 mm, groove 6 mm
  wide and 4 mm deep, 8 mm TPU pads, 0.2 mm pad recess, 0.5 mm floor
  clearance.

Steps:

1. Config:
   - `m10.internals.end_stops` holds the geometry and the load basis.
   - `m10.physics.end_stop_pads.joints`: thigh_a1 and knee_drive_a2 take
     r 0.020, A 22.8e-6, t 0.008, with y_m at groove mid-depth (0.046
     and 0.0065).
   - Pad and dowel materials go into the mass table.
2. `cad/m10/assembly.py` builds the grooves, pads and dowels with their
   seats.
3. Run the CAD pipeline twice for adopt_cad, then sections,
   shell_clearance and manufacturability.
4. Run the falls at both dt, then the acceptance, one at a time, with
   ≤8 workers under nice.
5. Refresh the assembly renders.

Results after adoption (commits cc9ea38 feat, 1d8a6b5 CAD data):

- CAD:
  - Pad-to-dowel gap 0.000 mm with 0.000 mm³ overlap at both stop
    poses, both sides. The new parts overlap nothing.
  - The sweep grid is clean.
  - CAD mass 1.8000 kg (was 1.8053): the grooves remove more aluminium
    than the dowels and pads add. Margin 0.19 kg.
  - Shell clearance is unchanged. The TPU pads are at least 3.8 mm
    thick.
- Plant tests: 25/25.
- Falls, dt / dt/2:
  - Forward and backward pass. Minimum factor 2.28 / 2.06 (was
    2.32 / 2.25).
  - Sideways: the same structural failures as before (worst 0.48,
    left_HipInnerBearing). These are the open sideways risks for the D5
    sign-off. Failure counts are 15→15 and 6→4 at dt, 16→14 and 7→6 at
    dt/2.
  - New in sideways_rest at dt only: the SkidPad peak strain is 0.53,
    against the 0.50 bottoming limit (was 0.48; 0.48 at dt/2).
  - Hip and knee-drive stop pad strains are ≤0.07.
- Acceptance: robustness 70/70 physical passes; inherited 26/27. The
  only miss is `delay_15ms_noise_x2_combined`, accepted at C6. The
  convergence check passes.

### D5 exit report (18 October 2026)

Phase D is closed, under the 13 October rule:
- Forward and backward tips must reach a structural factor ≥2 on every
  path.
- Sideways tips are containment cases. Their structural shortfalls are
  a recorded risk.

The user signed off the sideways-fall risk below on 18 October 2026.

Fidelity: this is a layout study plus a simulation under stated
assumptions. It uses an undeformed-body screen with CAD masses and
sections, manufacturer bearing and belt ratings, knocked-down supplier
strengths, and the M10 contact laws. Nothing here is hardware-validated.

Design state:
- D4 protection set: skid, wheel rings, TPU ears, 7075 shins, hip
  brackets and hip flanges, 6800 knees, 688A inner shaft.
- Knee stop B.
- Stock 2MR-192 hip belt at 62.5 N.
- Measured shell backing.
- Parts-check corrections.
- Option A hidden hip and knee-drive stops.

Commits cc9ea38 to 7927891.

| Exit criterion | Result |
|---|---|
| Forward/backward tips: factor ≥2 on every structural path, both timesteps | Met. Minimum 2.06: right thigh hip-belt slack, forward_0p3, dt/2. |
| Sideways tips: cosmetic parts attached and contained | Met at both timesteps (`summary.json` `requirement.passed`). |
| Sideways tips: factor ≥2 | Not met. Recorded risk, signed off by the user. |
| No energy injection | Forward/backward: none. Sideways_rest: 0.0074 J at dt and 0.0014 J at dt/2, one event each, against a fall energy of order 1 J. |
| Mass ledger within the allocation, ≥0.15 kg reserve | Met. 0.19 kg under the ceiling. |
| Locomotion acceptance after the last physical change | Robustness 70/70; inherited 26/27. The only miss is `delay_15ms_noise_x2_combined`, accepted at C6. Convergence passes. |

The top-level `passed: false` in `artifacts/m10/falls/summary.json` is
the strict factor-2-everywhere flag. Under the 13 October rule, the
`requirement` block is the gate.

Forward and backward: the lowest paths, as minima over the four cases
at both timesteps.

| Path | Factor |
|---|---|
| right thigh hip-belt slack | 2.06 |
| left thigh hip-belt slack | 2.27 |
| right knee-belt slack | 2.33 |
| left knee-belt slack | 2.36 |
| right thigh hip-belt rating (0.90 length factor) | 2.38 |
| left hip 6704 bearings (inner / pod) | 2.92 / 2.96 |

- The skid strikes first in every forward and backward case, at a peak
  strain of 0.21–0.49 against the 0.5 bottoming limit.
- Thigh contact depths are ≤0.45 mm.
- The hip and knee-drive stop pads stay at strain ≤0.07, and the knee
  stop pad at ≤0.13.

Sideways containment, at both timesteps:

- The ears are unloaded (root strain 0).
- The neck section factor is ≥20, and the ear retention is effectively
  unloaded.
- The right wheel ring strikes first, at strain 0.45–0.49.
- Shell strikes:

  | Struck part | Depth on a probed spot (mm) | Measured backing gap (mm) | Off-spot depth (mm) |
  |---|---|---|---|
  | Tub | 4.75–4.97 | 7.6 | ≤1.83 |
  | Thigh | 4.61–5.28 | 6.6 | — |

  The cover and the head front touch at 2.2 mm and 3.1 mm.

Recorded sideways risk (minimum over both timesteps), accepted:

| Path | Factor |
|---|---|
| left hip 6704 bearings (inner / pod) | 0.48 / 0.48 |
| left knee outer 6800 bearing | 1.02 |
| left hip bracket arm (7075) | 1.05 |
| left knee inner 6800 bearing | 1.26 |
| knee belt slack | 1.43 |
| thigh hip belt slack | 1.46 |
| knee-drive hip belt slack | 1.50 |
| hip belt ratings | 1.53–1.77 |
| right knee outer 6800 bearing | 1.66 |
| left thigh section | 1.69 |
| left hip housing seat (PA12) | 1.81 |

- SkidPad: peak strain 0.53 in sideways_rest at dt, so it bottoms
  (limit 0.50). It is 0.48 at dt/2.
- The printed walls at the tub corners and the thighs' knee ends sit at
  a factor of about 1.0–1.2 against the knocked-down PA12 strength.

Inspect after a side fall:
- hip and knee bearing marking;
- belt tooth-jump (re-zero the joints);
- dents or whitening at the tub's front-lower corners and the thighs'
  knee ends;
- the skid foam for permanent set.

Candidate remedies, not adopted. Mass deltas are the config estimates
(`m10.structure.candidates`), not CAD results.

| Candidate | Addresses | Mass | Status |
|---|---|---|---|
| `hip_pod_6804`: hip pod 6704 → 6804 (20 × 32 × 7) | HipPodBearing | +0.030 kg | Visible: the pod grows from a 27 to a 32 mm bearing OD. The 7 mm width needs 3 mm more in the lateral stack. |
| 6804 at the hip bracket | HipInnerBearing | not screened | Needs a wider bracket hub and lateral stack (motor row and side wall move). |
| `knee_6900`: knees 6800 → 6900 (10 × 22 × 6) | knee bearings | +0.030 kg | Does not fit the Ø23 knee boss (B3/D4 finding). |
| A thicker or wider skid | skid bottoming | not screened | Visible. The skid already covers part of the battery hatch. |

None of these lifts the hip bearings from 0.48 to 2. That would take a
larger hip bearing pair, a bracket change and a wider stack, which is a
visible layout change. The user chose to accept the risk.

Mass ledger (`artifacts/m10/cad/mass.json`, nominal pose):

| Item | kg |
|---|---|
| CAD mass | 1.8000 |
| Harness allowance | 0.060 |
| Fasteners and inserts allowance | 0.060 |
| Head pan reserve | 0.040 |
| Allocated | 1.960 |
| Ceiling | 2.150 |
| Margin (requirement ≥0.150) | 0.190 |

The mass by basis is:
- catalog 0.707 kg, plus catalog rotors 0.104 kg;
- printed PA12 0.303 kg;
- aluminium 0.272 kg, of which 7075 is 0.161 kg;
- TPU 0.145 kg;
- bearings 0.056 kg;
- steel 0.021 kg;
- the rest under 0.02 kg.

Carried forward (not part of the Phase D gate):
- Hardware validation of every factor above (the screen is conditional).
- The skid comes off for a battery swap (its front bevel covers the
  hatch's rear 28 mm).
- A forbidden-contact screen check for airborne contact loss.
- Phase E: repository cleanup.

## Phase E plan — repository cleanup (detailed, 18 October 2026)

User decisions (18 October 2026):

- Prune only:
  - Delete what M10 does not reach.
  - Keep the shared modules M10 imports (for example `robustness`,
    `m9_impact`, `m9_plant`, `articulated`) with their M4/M9 branches.
  - An M10-only refactor is not in scope.
- Remove the M4 electronics: the KiCad projects and their build, verify
  and SPICE tools. Firmware and `config/electronics.json` stay, because
  the firmware config generator reads it.
- Commit the user's pending edits to `tools/analyze_learning_study.py`
  and `tools/build_protection.py` first, then remove the RL/learning
  study and the M4 protection tools.
- Move the milestone docs 00–09 to `docs/history/`. Remove
  `artifacts/milestone-0…9`. The README becomes a single M10 status.

Everything removed stays reachable through git history and the tags:
- `archive/m9-final` (62ec65b): M9 state.
- `archive/m10-pre-cleanup`: a new annotated tag at the commit just
  before the first removal.

The tree change does not shrink `.git` (2.1 GB). That would need a
history rewrite, which is out of scope.

### What stays

The keep set is the Python import closure of the M10 entry points:
`bipe/m10/*`, `cad/m10/*`, `tests/test_m10_*`, `bipe/robustness.py`
and the MCP/firmware tools. That is 53 files.

| Area | Keep | Remove |
|---|---|---|
| `bipe/` | 29 modules in the closure | 45 modules: RL/learning, M4 electrical/thermal/packaging/closure/verify, M9 sizing, control and bumper |
| `cad/` | `cad/m10/*`, including the two historical look-study scripts | 16 legacy M3–M9 scripts |
| `tools/` | 9 in the closure, plus `blender-mcp.sh` and `freecad-mcp.sh` | 61 M4–M7 build, verify, audit, closure, power, protection and learning tools, plus `electronics-env.sh` and `rl-env.sh` |
| `tests/` | 11 whose imports are all kept: m10 ×3, m9 firmware/impact/plant/tilted ground, articulated, electronics feedback, mass properties, robustness | 10 that test removed modules: learning ×5, rl, physics, m9 bumper/control/sizing |
| `config/` | Every file a kept file reads (19) | `actuator-candidates`, `learning`, `learning-study`, `power-stage` |
| `electronics/` | — | All 150 files |
| `firmware/` | All | — |
| `artifacts/` | `m10/`, `tooling/`, and any old artifact a kept test or M10 entry point reads as an input (found by running them on the slim tree; kept at its path) | `milestone-0…9`. The exception is `milestone-3/cad/assembly.FCStd`: the user has an uncommitted edit there, so it stays tracked and untouched until the user decides. |
| `docs/` | `milestones/10-…`, `blender-mcp.md`, `freecad-mcp.md`, `workstation.md`, `references.md` | Milestones 00–09 and the other topic docs move to `docs/history/`, with a README there saying their artifact links resolve in `archive/m10-pre-cleanup` |
| Root | `requirements{,-mcp}.{in,lock}`, `requirements-blender-mcp.lock`, `requirements-platformio.lock`, `pyproject.toml`, `AGENTS.md` | `requirements-rl.*`, `requirements-electronics.*`, `requirements-delivery.lock`; the stale `.gitattributes` and `.gitignore` entries for milestone-4 and milestone-5 |

The user's uncommitted edit to `tools/generate_firmware_config.py`
(kept) and to `artifacts/m10/concept/blockout.FCStd` stays uncommitted
and untouched.

### Steps (focused commits on main)

1. Commit the user's edits to the two learning/protection tools. Tag
   `archive/m10-pre-cleanup`.
2. Baseline: the 11 kept tests on the current tree.
3. Remove the code: modules, legacy CAD scripts, tools and their tests.
   Then run the kept tests and a stale-reference scan: no kept file may
   import or open a removed path.
4. Remove `electronics/` and the electronics/RL/delivery requirement
   files.
5. Move the docs to `docs/history/`.
6. Remove the old artifacts, keeping the inputs found in step 3. Trim
   `.gitattributes` and `.gitignore`.
7. Rewrite the README as a single M10 status: what the robot is, the
   phase results with links, how to run the tests, CAD pipeline, falls
   and acceptance, and the MCP tooling.
8. Exit validation on the slim tree:
   - all kept tests;
   - the firmware tests;
   - the headless CAD pipeline, with `mass.json`, `sweep.json` and
     `check.json` unchanged;
   - one fall case (forward_rest at dt) identical to the committed
     result;
   - the 107-case M10 acceptance, run one job at a time with ≤8 workers
     under nice, with the same outcome as d54b29b.

Exit: tests pass on the slim tree, and the M10 pipeline, falls and
acceptance reproduce the committed results.

## Phase E results (18 October 2026)

Commits e9b1ee5 (the user's tool edits; tag `archive/m10-pre-cleanup`),
72d7027, fa1f1e7, 21b1399, 6060705 and f734d15.

- Tracked files: about 8,300 → 548.
- Code: 131 files removed (44 `bipe` modules, 16 CAD scripts, 61 tools,
  10 tests). `tools/call_freecad.py` now logs to
  `artifacts/tooling/freecad-mcp/`.
- Electronics and configs: `electronics/` (150 files), the RL,
  electronics and delivery requirement files, and four unused configs
  are removed.
- Docs: 52 docs moved to `docs/history/` with rewritten relative links,
  plus `docs/history/README.md`.
- Artifacts: 7,472 milestone files removed. The 6060705 message says
  7,467; 7,472 is the count, plus `.gitattributes`. These are kept as
  inputs of the kept code:
  - the M4 component mass file and its 65 STL meshes (M4 plant tests);
  - the M3 component mass file;
  - the M9 components and render mesh.
  `milestone-3/cad/assembly.FCStd` stays for the user's uncommitted
  edit.
- Not changed:
  - about 560 MB of git-ignored local leftovers in the old milestone
    folders (their `.gitignore` entries stay);
  - `.git` (2.1 GB; shrinking it needs a history rewrite);
  - the user's uncommitted edits to `tools/generate_firmware_config.py`
    and `artifacts/m10/concept/blockout.FCStd`.

Exit validation on the slim tree:

- Tests: 109 passed. This includes the M10 and M9 firmware parity
  tests, which load the existing `firmware/build/libbipe.so`. The
  library was not rebuilt, because `make -C firmware` regenerates
  `generated_config.h` from the user's uncommitted generator edit.
- Headless CAD pipeline (two adoption passes, then sections, shell
  clearance and manufacturability):
  - the sweep is clean, the sit pose is 0.054 m, and the covered ranges
    are unchanged;
  - mass 1.8000 kg, allocated 1.96 kg;
  - clearance 7.6 / 6.65 mm.
  - The only differences are fixed-point drift in the adoption loop:
    pose angles about 1.7e-6°, and inertias in the last stored digit
    (1e-11 kg m²). The committed outputs were restored.
- Fall forward_rest at dt: factor 2.6192452, the same as 5e5ccd2 to
  about 1e-9.
- Acceptance (107 cases, 8 workers): the same outcome as d54b29b.
  - Robustness 70/70; inherited 26/27, the only miss being
    `delay_15ms_noise_x2_combined`.
  - All targets are identical, and convergence passes.
  - Metrics agree to the noise level: slope rollback 0.058761 →
    0.058764 m, belt-range pitch 1.029 → 1.033°, backlash at 0.6
    0.911 → 0.909°.

Phase E exit met: tests pass on the slim tree, and the M10 pipeline,
falls and acceptance reproduce the committed results.
