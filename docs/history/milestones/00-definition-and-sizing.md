# Milestone 0 Definition and sizing

Created on 30 September 2026 before implementation. This milestone establishes the working dimensions, mass assumptions, and software workflow for the first robot experiment. Its outputs describe a preliminary concept, not a selected hardware bill of materials.

## Scope and dependencies

Use the agreed hardware-oriented scope in [the project plan](../project-plan.md): balance, drive, turn, and crouch, with compact proportions inspired by small wheeled camera robots. This milestone precedes balance controller development. A detailed plan for each later milestone will be written immediately before its implementation begins.

## Implementation sequence

1. Create an isolated Python environment with MuJoCo, numerical tools, plotting, and video export. Record working versions. Run independent physics and rendering smoke checks.
2. Recheck NVIDIA hardware outside the restricted process where necessary. Inspect the renderer actually used. Avoid changing drivers or interrupting other GPU workloads.
3. Verify FreeCAD can execute an export macro and create a dimensioned solid. Use that path to generate an editable rough assembly, STEP export, and shared geometry for inspection.
4. Define one parameter file in SI units containing the body and wheel dimensions, link lengths, candidate actuator envelopes, component masses, joint conventions, contact assumptions, and preliminary motor limits.
5. Calculate a mass budget, centre of mass at fixed posture, initial wheel speed and acceleration torque, and a sampled leg load envelope. Include assumptions and distinguish nominal loads from design allowances.
6. Compare a serial two-joint leg with a four-bar linkage concept using dimensioned side views and a packaging comparison. Select the concept for the first experiment while retaining the alternative for later evaluation.
7. Save the generated artifacts and a results note. Proceed to Milestone 1 only after writing its detailed plan.

## Initial modeling choices

Use X forward, Y left, Z up. Positive pitch rotates about Y. Represent a two-link leg with equal links bending forward at the knee and returning toward the wheel axle. Both wheel axes run along Y; document the positive wheel motion explicitly before tuning control.

The initial actuator dimensions and masses are placeholders for feasibility calculations. Use finite wheel torque, speed, and motor response limits in the next milestone, but label those limits as assumptions until parts are selected. Model reserves for electronics, cables, fasteners, and a potential camera so the apparent robot mass is not just the shell mass.

For a serial leg, calculate static torque from the leg Jacobian and a vertical load share. Sample the proposed crouch range. Report nominal values and a dynamic allowance separately. These calculations cannot establish peak impact torque or sustained thermal capacity.

The four-bar alternative is a schematic parallelogram with one driven crank and one passive parallel link per side, plus a wheel drive. It is an alternative topology study, not a validated substitute: its circular axle path, bearing loads, closure clearances, and control authority need further analysis if selected.

## Deliverables

- Shared robot parameter JSON and a script that checks units, geometry, and mass accounting.
- Reproducible package environment and renderer diagnostics.
- Dimensioned concept comparison and mass and load plots.
- FreeCAD macro, editable FCStd assembly, STEP export, and export check results.
- A results document listing the selected preliminary topology, computed dimensions and loads, and unresolved hardware assumptions.

## Completion checks

The parameter masses sum consistently. Geometry produces the intended wheel position, reachable height range, and overall envelope without obvious blockout interference. The concept comparison shows both mechanisms and states the reason for the preliminary choice. The sizing report records candidate joint and wheel demands without claiming purchased actuators can meet them.

MuJoCo must load and step a simple model; rendering must produce an actual inspected image. FreeCAD must export a solid with the expected dimensions and volume, and the rough assembly must reopen successfully. Installed application metadata alone is insufficient evidence.

## Risks and unresolved inputs

The compact six-actuator arrangement may require different actuator envelopes or larger dimensions after real part selection. Bearing supports, fasteners, cable bend radii, tolerances, gearbox inertia, friction, battery performance, and thermal loads remain preliminary. Hardware budget, operating time, and manufacturing access are still open inputs before component selection.

## Results

Completed as a preliminary study. See [the results and completion evidence](00-results.md).
