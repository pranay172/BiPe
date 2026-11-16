# Milestone 3 — compact mechanical assembly

Plan created on 30 September 2026, before implementation. Milestones 0–2 and their saved acceptance artifacts form the baseline. This document defines proposed work and completion checks; measured results belong in `03-results.md`.

## Objective and scope

Develop a compact, component-based digital assembly that can guide a future physical build. Retain the six-actuator serial-leg architecture, forward-facing eyes, backward-folding knees, and balance/drive/turn/crouch scope unless the feasibility review demonstrates a reason to revise them. Fabrication is assumed to be outsourced. Prefer readily documented components and common fabrication processes; no purchasing region or fixed procurement budget is required for this digital phase.

The result must include real actuator and battery candidates, wheel and joint interfaces, structural members, bearings/support where needed, fasteners, a serviceable shell, and reserved electronics/cable space. Check geometry over the intended range and update the physical model before repeating acceptance. A component envelope is not a production CAD model, and simulation is not hardware validation.

Electronics schematics, firmware implementation, person following, jumping, and a release-ready drawing package belong to later milestones. M3 must identify the electrical/firmware interfaces and mechanical provisions those stages need.

## Inputs and dependencies

- `config/robot.json` is the shared SI source for dimensions, masses, and physical limits. CAD converts units at its boundary.
- The M2 baseline is 2.02 kg, with 90 mm links, 80 mm wheels, 165 mm wheel track, and a nominal 143.85 × 189 × 252.89 mm CAD envelope at 40° posture.
- The commanded crouch range is 30–60°. Mechanical limits are hip 20–70° and knee −135 to −40°. Check symmetric crouch and independent leg perturbations as well as the nominal assembly pose.
- Controlled M2 peak demands are approximately 0.242 N·m at a hip, 0.772 N·m at a knee, and 0.155 N·m at a wheel. Largest RMS demands are 0.123, 0.513, and 0.026 N·m. These belong to the old mass model and must be recalculated for the assembly.
- M2 assumes 1 kHz local joint control, 200 Hz body control, 2 ms joint torque response, and 15 ms wheel response. A 2 ms simulation assumption is not an established hardware requirement.
- FreeCAD 1.1.1 with the project Robust MCP launcher, MuJoCo, plotting, and rendering are available. The bridge is connected directly in this assistant session. Keep all Python dependencies in `.venv`.

## Implementation order and decision gates

### A. Actuator feasibility before detailed mounting CAD

1. Preserve M2 snapshots and derive a load summary from its accepted controlled traces.
2. Sweep joint response at 2, 5, 10, and 20 ms, with the corrected M2 sampling scheme and unchanged controller first. Include hold, drive, crouch, and combined motion; save configuration and measured results. Retune only if justified by observed behavior, and identify which assumptions change.
3. Compare at least three documented actuator families. Record exact variant, drive electronics inclusion, continuous/rated versus stall/peak torque, speed, mass, envelope, supply, reduction, rotor inertia, position sensing, backlash, local control, startup/calibration, and protocol constraints. Unknown fields remain unknown.
4. Choose a digital candidate using rated-load margin and compact packaging. Reject a candidate when the available evidence cannot support an essential requirement. Record alternatives and indicative manufacturer prices separately from a procurement quote.
5. Evaluate bus utilization and control placement. Separate module-local motor/position loops from host CAN/serial commands; do not assume that six command/reply pairs at 1 kHz fit a 1 Mbit/s bus.

Gate A: a reasoned component candidate, computed torque/speed margins, a viable communication budget, and an explicit list of response, thermal, backlash, and startup assumptions requiring later physical verification. A datasheet absence must not silently become a claimed performance value.

### B. Packaging and mechanical interfaces

1. Size the battery and electronics allocation against actuator voltage/current needs. Estimate energy and runtime from stated load and loss assumptions, with a sensitivity range. Do not present an estimated duty cycle as measured battery endurance.
2. Create parametric FreeCAD assembly parts and distinct manufacturer-envelope objects. Use source drawings for mount interfaces; label simplified envelopes and incomplete supplier dimensions.
3. Design structural members, joint adapters/supports, wheel hubs, shell panels, battery retention, fastening access, and cable corridors. Prefer outsourced sheet/plate machining and printable nonstructural covers where suitable.
4. Evaluate joint-bearing and shaft loads, fastener loads, nominal stress/deflection, fits, edge distances, thread engagement, and service access with documented engineering assumptions. Hand calculations do not replace supplier bearing limits or structural testing.
5. Sample the motion envelope, calculate solid intersections/distances for nonmating pairs, and check tire-ground/body clearance. Record allowed mating contacts explicitly so they cannot hide unrelated interference. Use at least 2 mm nominal clearance between moving nonmating hard parts, with exceptions justified individually.

Gate B: valid, reopenable native CAD and STEP exports; measurable component fit; a repeatable motion-clearance report; a mass budget including cables, fasteners, and reserve. Define folded and operating envelopes separately. Aim for the original 2.5 kg working ceiling; explain and record any necessary revision instead of omitting components.

### C. CAD and simulation consistency

1. Derive rigid-body masses, centers of mass, and inertia tensors from material-based CAD and documented component masses. Where internal component mass distribution is unavailable, use a stated uniform-envelope approximation and test uncertainty.
2. Preserve the original model artifacts. Save the assembly parameter set, generated physics model, controller, sensors, sources, and configuration fingerprints with new M3 runs.
3. Update motor torque/speed limits, reflected inertia, encoder resolution, lag, and modeled backlash as the evidence permits. Keep uncertain terms labeled and test sensitivity.
4. Repeat the articulated acceptance scenarios after physical-model/controller changes, including nominal sensing, disturbance, variation, repeatability, and timestep convergence. Repeat the fixed-height checks as a regression without overwriting accepted M1/M2 evidence.
5. Inspect CAD views, rendered physics motion, measured traces, and joint loads together. Iterate packaging and control when an updated model fails; do not pass by constraining the free base or disabling a necessary collision.

Gate C: updated nominal digital configuration passes the motion tests, and load demand remains within the chosen derated continuous and peak limits. The accepted controller must be identical across its suite. If a required check cannot pass, document the unresolved gate and keep M3 open.

## Proposed completion checks

- Exact component variants and source links are recorded; no stall rating is relabeled as continuous torque.
- Actuation has at least 1.5× measured RMS-load margin to a documented rated/continuous limit where one is available, plus adequate peak and loaded-speed margin. Supplier test conditions and thermal uncertainty remain visible.
- Assembly parts have valid solids; native reopen and STEP round-trip preserve dimensions and volumes.
- The intended motion range has no unexplained nonmating intersections. Clearance sampling resolution, pair exclusions, and minimum gaps are saved.
- Nominal mass meets the justified design ceiling, with battery, structure, six actuators, electronics allocation, bearings/fasteners, wiring, tires, and reserve included.
- CAD-to-physics mass, COM, and inertia comparisons pass documented numerical tolerances; inertias are positive definite and use SI units.
- The updated free-base model passes the established 60 s hold/recovery, drive/brake, turn, push, five-crouch, combined-motion, variation, repeatability, and convergence gates. The required stopping limit remains 100 mm; 80 mm remains an aspirational target.
- Actual actuator timing, thermal performance, startup/calibration, joint backlash, fabrication tolerances, and endurance requiring physical testing are listed for the eventual build.

## Deliverables and checkpoints

- This detailed plan, committed before implementation.
- Manufacturer-backed component comparison and selection notes, machine-readable physical data, and preserved source/drawing references.
- Actuator sensitivity and sizing reports, including power/runtime estimates with assumptions.
- Parametric CAD sources, assembly FCStd/STEP files, representative views, motion-clearance report, BOM, and preliminary fabrication/assembly notes.
- Updated simulation configuration, acceptance traces/plots, mass/inertia consistency checks, and a recorded representative trajectory.
- `03-results.md` separating completed checks from open hardware questions.

Make focused local commits after the plan, actuator/architecture decision, validated mechanical assembly, and final acceptance. Preserve failed experiments as development evidence where they inform decisions; identify the final accepted configuration unambiguously.
