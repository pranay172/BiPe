# Digital assembly and service sequence

M5 reference, 1 October 2026. Open the accepted M4 native assembly and M5 reference
drawings together. The Blender exploded view shows component relationships;
the M4 saved removal sweeps supply the conditional clearance evidence. Neither
releases unmodeled fasteners, cable bends or hardware construction. Fabrication
and operational flashing remain held in [hardware qualification](hardware-qualification.md).

## Coordinates and preparation

X is forward, Y left, Z up. Hip datum Z=0 in CAD; floor-relative height is set by
the selected posture. At the 40-degree drawing posture, knees point -X and eyes
face +X. The drawing envelope is 144.851 x 284.000 x 260.888 mm. Per-part STEP/STL
exports are in millimetres and remove the native object's assembly placement.
The Blender scene is in metres; do not rescale it by 1000. Left and right files
are distinct mirrored references, not automatically interchangeable parts.

Confirm configuration/native-file hashes through the delivery verifier first.
Review custom-part datums, every part-specific HOLD, material/temper and process
with the outsourced supplier before any fabrication quote is promoted to an
order. Supplier STEP actuator surfaces are reference only; the solid cylinders
are envelopes. The chosen pack and driver firmware have not been qualified.

## Proposed assembly order

1. Inspect machined links, adapters, hubs, forks, tray and roof against native
   coordinates and reference dimensions. Finish and measure bearing seats after
   rough profiling. Check countersunk heads sit flush; retain full lands and
   deburr cable-facing edges. Calibrate PETG insert coupons before using shells.
2. Assemble each hub/tire/bearing/fork group only after the axial retention and
   tire interface are designed. Fit the loaded ring with suitable tooling;
   pressing through the opposite ring is not the proposed load path. Confirm
   free rotation and the complete actuator's axial/overhung load permission.
3. Attach wheel stators to lower plates with the documented flush countersunk
   screws. Attach the fork ears and lower output adapters. Check full 360-degree
   hub/head clearance before connecting a motor or enabling power.
4. Build upper/knee/hip groups using separate housing and output bolt patterns.
   Inspect screw projection and blind-hole engagement. Use a supplier-approved
   torque/locking method after thread/material qualification; there is no
   numerical assembly torque in this digital release.
5. Attach the hip brackets, rear shell and tray. The exact shell load path,
   printed insert strength and mounting stack are still held. Install the
   professional protected pack, pad and straps only after anchor/cable/fault
   qualification. Do not assemble a bare-cell pack from the digital envelope.
6. Fit the two native boards to qualified insulated supports at their saved
   orientations. The carrier is horizontal; native power X maps to robot Y,
   native Y maps to robot Z, components face +X. Inspect H1-H4 coordinates in
   the boards and preserve capacitor/FET clearances and low-profile terminations.
7. Fit resistor bank, precharge resistor, thermistors and insulated roof/seal
   using qualified clamping/thermal interfaces. Verify calibrated sensor error
   including lag, pulse limits and heat paths before using the modeled 40 C /
   600 s duty domain. Do not infer temperature from a render or nominal contact.
8. Route and label the [13 inter-board connections](../../artifacts/milestone-5/inventory/harness.json),
   two independent arm contacts, NC stop, CAN and motor power. Verify polarity,
   net continuity and isolation against native pads. Do not infer motor connector
   pins. Perform full swept flex/tool/strain-relief checks before closing the face.
9. Fit the front shell and eyes after verified service access. Record as-built
   mass, COM changes, actual fasteners/wire lengths and clearances. Update the
   shared config/CAD and rerun acceptance if physical assumptions change.

All nine steps are a future assembly sequence. No physical construction has
been performed or validated by M5.

## Power and firmware gates

The supplied ELF/HEX is a locked diagnostic target with hardware RUN inhibited.
Portable C runtime/supervisor/startup behavior and free-base sensor/fault traces
are verified digitally; exact AK40 driver codec and ATtiny ADC/IO/WDT adapter
are not commissioned. Confirm exact actuator/pack voltage, motor zeroing and
encoder behavior, measured current/transient/storage bounds and independent
cutoff before developing an operational image.

Qualified startup would begin with both contact loops and sensors checked,
all six motors disabled, a cold manual attempt, independently metered precharge,
BUS_READY and explicit driver permission. START_OK alone is not bus readiness.
Normal stopping, host loss and independent fault cutoff differ; the robot can
fall after power removal. Reset owes the full ten-minute cooldown. Physical
commissioning needs a separate supported test fixture and measurement plan.

## Service order

1. Support the robot mechanically. Isolate and remove the protected pack supply;
   measure residual bus voltage using a qualified method. The software's zero
   reading is not a physical isolation check.
2. Disconnect roof sensor/control/resistor looms and capacitor strain relief
   before lifting the roof. Preserve thermal interface/seal condition for
   inspection; qualify reassembly contact after each service disturbance.
3. Remove front-shell screws and open the face. Disconnect the carrier looms
   by their keyed/labelled interfaces; remove carrier mounts before the carrier.
4. Release straps and remove the pack through the modeled front path without
   pulling on power terminals. Remove the power assembly after clearing its
   side/rear looms. Native M4 sweeps exclude real cable bends and driver tools.
5. For a wheel, disconnect that motor's loom and support its link before
   releasing fork/retainer hardware. Keep bearing rings/seat surfaces clean and
   identify mirrored parts. Recheck zero/calibration, retention and manual motion
   after reassembly, then repeat the relevant hardware commissioning checks.

## Acceptance provenance

The digital operating evidence remains M4: 27 full C locomotion cases plus
half-timestep convergence, fixed-height regression, 12 mechanical uncertainty
cases and five unconstrained shutdown scenarios. The model has a free base,
six finite actuators, physical contact and no equality constraints. M5 changes
delivery/presentation only; it rechecks baseline hashes rather than representing
presentation animation as a new physics test. The entire configuration and
measured traces are included in the release package.
