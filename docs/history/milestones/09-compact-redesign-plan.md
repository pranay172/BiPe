# M9 — compact redesign: amended implementation plan

**Approved revision:** retain the two-hip five-bar, 2.0 kg ceiling and **200 g
unallocated reserve**; replace all four AK40-10 joint drives with a lighter
load-sized class. Retain RO40 Lite wheel cores/4:1 belts, moteus-c1 and the
850 mAh 6S pack. Add rounded leg covers and **25–30 mm tires**.
This step ends at CAD, clearance and mass adoption; no MuJoCo/controller work.
The [previous failed checkpoint](09-cad-checkpoint-results.md) is historical:
2297 g allocated, six motion interferences and four static shell/fork fits.
The [reference sheet](../../../artifacts/milestone-9/concepts/concept-sheet.png)
and a reference comparison render
remain appearance references, not engineering drawings.

## Selection and implementation order

1. **Recompute loads and select real joint drives.** Reuse five-bar Jacobians;
   include actual moving CAD masses/tensors, gravity, the 20 N/0.1 s chassis
   pulse, commanded crouch velocity/acceleration and rotor/reduction inertia.
   Size maneuvers at the full 2 kg ceiling as well as the measured candidate.
   Keep inertial terms distinct from chassis Jacobian-transpose loads.
   Compare ≥3 manufacturer-documented options, including reduced gimbal/
   outrunner BLDC+moteus-c1 and integrated quasi-direct drives. Record rated/
   pulse torque and thermal duty, reflected inertia, backlash, complete drive
   mass, voltage, torque/current command interface, startup output-position
   knowledge and price. Aim ≤100 g/joint including driver, encoder, reduction
   and housing; loads take precedence. Position-only hobby servos require a
   demonstrated torque-control case. Do not invent missing vendor values:
   label calculated/estimated quantities and retain qualification gaps.
2. **Size the lean fall load path before CAD.** Screen the full 2 kg ceiling,
   nominal-stance fall at rest and 0.3 m/s, initially 150 mm COM drop. Energy
   is ~3.03 J; triangular stopping force is ~1.23 kN at 5 mm effective leg
   stopping travel (~2.04 kN at 3 mm shell travel). Replace height/contact
   leverage with measured CAD COM/geometry; include rotational energy.
   Use Jacobians/link equilibrium to obtain worst hip reaction and fall-related
   joint torque. Select thin-section bearings from catalog static ratings,
   actual support spacing/overhang and combined-load rules, with **factor ≥2**.
   Consider one external support plus an adequately documented motor bearing,
   pocketed/bent-sheet forks, hollow aluminium axles and short overhangs.
   Direct load paths and a mechanical torque-isolation/impact-stop mechanism
   must bypass gearbox/belt shock. A radial catalog rating alone does not prove
   moment capacity. Material allowables, contact travel and isolation remain
   explicit assumptions pending the mandatory fall gate; unknown capacity
   cannot be called a passing gate.
3. **Native CAD with covered legs and wide lightweight tires.** Keep the curved
   pebble shell/expressive eye face. Package both joint drives inside/flush
   with the body; re-stack bearing/link planes to eliminate all previous fits.
   Fit rounded covers around both crank/rod mechanisms, with moving-clearance
   and service access. Select tire width within 25–30 mm from packaging/contact
   needs; model actual spoked/hollow hubs and foam/hollow-tire material volumes.
   Track/body width may change, but complete operating width stays **≤200 mm**.
   Include drive housings, purchased motor/pack/board envelopes, bearings,
   encoders, flanged pulleys, belt clearance/tension access, passive forks,
   shell/cover mounts and impact bypasses. Resolve disconnected custom-part
   details. SI parameters remain the input; mm conversion is only at FreeCAD.
4. **Clearance and mass checkpoint.** Sweep the full operating↔crouched↔folded
   height range and fore-aft workspace with covers installed; check forbidden
   finite intersections, minimum clearances, closure and static/service fits.
   Report sampling and intentional mates/exclusions. Derive custom mass/COM/
   tensors from actual CAD solids; use catalog masses for purchased parts and
   honest general allowances only for wires/connectors and fasteners. Include
   all six drivers, sensors, covers, tires, battery and power/MCU packaging.
   If allocated **≤1800 g** and dimensional/clearance gates pass, adopt the
   measured SI CAD design and body tensors into `config/robot.json`, clearly
   separating it from the still-active M4 physics profile, **then stop**.
   If overweight, reasonably lighten spokes, pockets/frame/covers and tires;
   shell thinning requires the fall-strength basis. If still over, **stop and
   report shortfall/options**. Never reduce reserve or raise the ceiling.

Deliver one short actuator/load comparison, one CAD/catalog mass ledger with
rigid-body tensors, the native assembly, motion/service results and a small
render set: operating, crouched, folded, exploded and reference comparison. Reuse the
CAD builder/sweep and sizing tooling; no new seal/verifier scripts. Make local
commits at plan, selection and validated-CAD checkpoints; do not push. Never
modify/stage the user's `artifacts/milestone-3/cad/assembly.FCStd`.

## Electronics and subsequent gates

Prefer six moteus-c1 CAN-FD drives if the load-sized BLDC option wins; account
for four added board footprints and all rotor/output sensing. Document output
angle ambiguity after belt reduction and use an output absolute encoder or
manufacturer-documented homing. Scope new current/torque/fault adapters, bus
load, pack/regen/dump and thermal/current budgets; the old 6.5 A bus limit does
not carry over. No circuitry/firmware implementation in this step.

After this checkpoint is approved, implement physical closed-loop/free-base
MuJoCo joints, finite drives and **nonzero** wheel/hip belt compliance/lash.
Use M8 lost-motion infrastructure and Gates stiffness information, with
justified uncertainty rather than zero lash. Controller design must include
5 ms transport delay, estimator/filter states, actuator lag and sample/hold
in the posture-dependent design model; compare delay-aware LQR/cascade and
port the adopted core to C with equivalence.

Later M9 acceptance remains: all inherited 27 cases plus convergence; hold
survives ±2 N·s, strict recovery target final 2 s ≤2°/0.03 m/s; slopes through
10° with rollback ≤0.10 m; hold pitch cycle ≤1.5° peak-to-peak at 0/0.3/0.6°
backlash. Compare M8 for information. The fall/impact gate is mandatory:
links, supports, shell/covers and actuator bearings need factor ≥2 against
stated allowables, no tooth jump or alignment loss and qualified shock bypass.
Digital layout, simulated acceptance and hardware validation remain distinct.
Camera following, jumping and RL remain out of scope.

The authorized steps 3–6 session uses the separate
[physics execution plan](09-physics-execution-plan.md). It includes Claude’s
clarification: complete independent steps 3–5 before reporting a fall failure,
and engineer missing allowables/compliance. The 5.914 g allocation headroom
and absolute acceptance thresholds remain unchanged.

The frozen steps 3–6 session is recorded in
[measured physics results](09-physics-results.md): implementation is retained,
but locomotion, prediction and fall gates fail. No reserve trade was applied.

**Closure authorization, 4 October 2026:** the user allows allocated mass
≤1,850 g with ≥150 g reserve (up to 50 g reserve draw), and requires factor ≥2
for structural paths and bumper mounts. Retained cosmetic prints may be
replaceable, isolated from electronics and protected by frame-mounted foam and
controlled falls. The unchanged locomotion thresholds and this amendment are
tracked under the [closure plan](09-closure-plan.md); later fairings remain
out of scope. Prior limits/results above remain historical records.

## Completion pass — authorized 5 October 2026

This amendment supersedes the closure allocation and fall scope: total ceiling
2.300 kg, allocated CAD ≤2.150 kg and unallocated reserve ≥0.150 kg, including
fairings. Design falls are the six passive tips and powered falls with measured
first-contact translation-plus-rotation energy ≤6 J; every structural path and
bumper mount requires factor ≥2. Search 4–10 N·s for the smallest unrecoverable
hold push. Higher-energy falls require retention, battery containment and no
exposed live conductors; report their factors without claiming structural pass.
Cosmetic prints may be replaceable but must remain attached in design falls.

Implement in order: (1) justify slope hold authority with finite torque and
correct Python/C control, probe 10° rollback against 0.100 m; (2) try bounded
drive damping schedules for both delayed-drive misses; (3) measure protective
crouch/contact order and detection/shutdown timing on actual design falls;
(4) size complete shaft/journal/fork/hub/belt/clutch/fastener/bumper load paths
using conservative measured loads and supplier data, then adopt heat paths;
(5) fit rounded leg fairings and close side windows within 200 mm; (6) run full
clearance/service sweep, export/adopt CAD masses and tensors, stopping before
adoption if the complete allocation would exceed 2.150 kg; (7) freeze sources
and execute all 107 cases, convergence, design/extreme falls and electrical/
thermal screening. Failed locomotion targets remain failures while independent
work continues. Do not relax thresholds or begin indefinite fidelity work.

Keep achieved results separate in the results record (about one added page),
retain final configurations/traces and necessary sensitivities under ~60 MB,
replace superseded artifacts, render operating/crouched/folded/exploded and
reference comparison. Commit plan, control/protection, validated CAD and final
verification checkpoints locally with validation notes; never push or touch
the user-modified M3 assembly. Prediction/LQR is omitted unless explicitly rerun.

### Completion-pass achieved checkpoint

The authorized pass ended before CAD adoption at the evaluated reinforcement mass stop. Drive-delay targets and 27-case/convergence acceptance now pass; 70-case survival is preserved. Slope hold, structure and containment remain unaccepted. Fairings, bumpers and heat geometry were not adopted. Measured outcomes, the runner-specific hold-fall bracket and reproduction commands are recorded in [the frozen completion results](09-physics-results.md); these are results, not new proposed requirements.

## Bumper-first finishing pass — authorized 5 October 2026

This amendment supersedes the preceding fall classification. Only the six
passive tips (forward/backward/sideways, at rest and 0.3 m/s) require structural
factor ≥2. The 40° cross-slope fall including its downhill tumble and contacting
hold-push falls near 6 N·s and above require containment, including retained
bumpers/mounts; report structural factors for information. Cosmetic prints must
remain attached and contained in design tips. Allocation remains ≤2.150 kg,
reserve ≥0.150 kg, width ≤0.200 m. No thresholds are relaxed.

1. Compute support/convex-hull coverage of the fitted robot in nominal and
   protective-crouch postures through each passive-tip orientation. Size named
   foam from manufacturer stress–strain data and tip energy, targeting 15–20 mm
   useful stroke. Design short frame load paths that bypass the leg drivetrain;
   include mounts, retention, cosmetic attachment and mass in the candidate.
2. Put the bumper geometry and compliant contact law in the free-base plant,
   rerun all six tips and the conservative belt-load screen, then reinforce
   only remaining failures. Check a complete candidate mass before adoption;
   stop with quantified options if it exceeds 2.150 kg.
3. Fix slope hold with justified finite hold authority or estimated incline
   feedforward, retaining the adopted delayed-drive damping and Python/C parity.
   Probe rollback before the frozen suite.
4. Adopt the 22.5 g, 15.9 K/W stationary heat paths; close shell side windows and
   fit rounded leg fairings, integrating bumper forms where useful.
   Recheck bumper coverage, all-pose clearance, service, width and full CAD mass;
   adopt exported rigid-body masses/inertias only after the checkpoint passes.
5. Freeze inputs and rerun 107 cases plus convergence, six design tips, extreme
   falls and thermal/electrical screening. Render operating, crouched, folded,
   exploded and reference comparison views. Report residual failures honestly.

Commit this plan before implementation and focused implementation/verification
checkpoints thereafter, recording checks. Replace superseded outputs to keep
M9 near 60 MB; add approximately one page of measured results. Keep model-fidelity
limitations as open items. Never push or modify/stage the M3 assembly.

### Bumper-first achieved checkpoint

The 2,888-direction nominal/crouch hull preflight exposes a lateral stroke
shortfall: 0.250 mm clearance beyond hard hardware at 90°, versus 20 mm useful
foam stroke. A revised frame clears the two preliminary convex pose checks.
Its evaluated frame/foam/heat plus optimistic width-only belt option is
2,163.68 g, 13.7 g over allocation after native union/notches, included bolts and optimistic drilling credit;
fairings, detailed retention and residual reinforcement are excluded. The
authorized stop applies before dynamic tipping/CAD adoption and subsequent
steps. This is one evaluated option, not a global impossibility result. No
adopted physical/control profile changed; 42 targeted checks and baseline XML/
input invariance pass. See [measured results](09-physics-results.md) and the
[new reproduction record](../../../artifacts/milestone-9/bumper-first/reproduction.json).

### Independent completion step 1: slope hold

Implement and report slope hold before starting another completion step. Keep
the adopted physical model and acceptance thresholds fixed; this step changes
only sensor-based hold control and its C equivalent. Probe faster position
capture and bounded integral lean authority, justified against the existing
wheel torque limits. Save each candidate configuration and measured traces;
retain the adopted drive damping. Check the 0/4/8/10° hold/up/down cases, strict
push recovery and delayed drives, then freeze the selected controller and rerun
all 107 cases, convergence and Python/C equivalence. Record rollback, suite
counts and torque authority in a short results paragraph and one table row.
Commit this plan before implementation and commit the verified result locally.
CAD, mass, falls, bumpers, fairings and the user's M3 assembly are outside this
step; the remaining completion steps await their separate instructions.

Step 1 achieved checkpoint: all six gain candidates are rejected; the 10° target
remains unmet. The unchanged controller's fresh 107-case acceptance preserves
27/27 plus convergence and 70/70 survival. See [measured results](09-physics-results.md).

### Independent completion step 1b: contact friction fidelity (Claude)

Step 1's diagnosis measured a constant 13.4 mm/s gap between the wheel rolling
speed and the axle speed during the 10° hold. Contact-point slip velocity
confirms that the tires creep downhill at about 13 mm/s while using only
0.176 of μ = 0.8. A rubber or foam tire sticks below μ; its tread-shear
compliance at hold loads is about 0.04 mm and does not grow.

**Cause.** `MaterialContacts` uses a shared 1e-4 impedance so the normal rows
reproduce the foam force–depth law. The friction rows inherit the same
impedance, so friction acts as a viscous damper. A minimal box on a 10° slope
creeps at 2.3 mm/s with the same formulation.

**Plan.**
1. Give the friction rows a Coulomb formulation. A large `impratio` removes
   the friction regularization, and a finite direct damping on each friction
   row (1000/s, four 0.25 ms physics steps) keeps the stick explicit and
   stable. Apply it to every contact. The normal law is unchanged.
2. Verify on a box: zero creep below μ, and exact Coulomb sliding acceleration
   above it.
3. Leave the controller and thresholds unchanged. Rerun the 107 cases plus
   convergence, robustness, delayed drives and Python/C parity. Retune the
   hold only if the target still fails.
4. Fall and tip results depend on contact friction and are rerun in step 2.
   Archived configurations without the new keys reproduce the earlier
   viscous rows.

Step 1b achieved checkpoint:
- Coulomb rows remove the creep, and the 10° hold rolls back 0.092 m (pass).
- The unchanged controller then shows a backlash hold limit cycle, and the
  inherited suite drops to 25/27.
- All six hold-gain candidates are rejected, and the controller is unchanged.
- The hold fix awaits a controller decision.

See [measured results](09-physics-results.md).

### Independent completion step 2: inboard wheel support and bumpers (Claude)

**Goal.** In sideways falls the tire, not hardware, takes the first lateral
contact. Front and rear frame-mounted foam takes forward and backward falls.
The six passive tips must reach structural factor ≥2.

**Limits (unchanged).** Allocated mass ≤2.150 kg with ≥150 g reserve; width
≤200 mm; Coulomb friction from step 1b; no state clamps or pose constraints.

**Hull check before this step.**
- Beyond about 70° of sideways tilt, the outboard readhead and its mount, the
  outer fork arm, the axle end and the outboard support of the wheel-motor
  housing all sit at 99.5–99.8 mm, which is 6 mm outside the tire face.
- Forward falls land on the face cap and eyes, and backward falls on the rear
  shell and the crank covers.

**Plan.**
1. **Single-sided wheel support.**
   - Remove the outer fork arm and the outboard readhead mount.
   - Shorten the fork bridge so it carries the wheel-motor housing from the
     inboard arm.
   - Cantilever the fixed 17-4PH axle from the inboard arm.
   - Recess the hub web and move the outer 608 bearing inboard.
   - Mount the wheel encoder ring on the hub and the readhead on an axle-end
     bracket inside the hub recess. The cable runs through the hollow axle.
   - Keep all hard hardware at least 8 mm inside the tire face.
   - Add an LD45 side pad on the motor housing's outer face, flush with the
     tire. It doubles as a leg fairing.
2. **Front and rear foam bumpers.**
   - Front: an LD45 face pad with recessed eye windows, leading the eyes and
     face cap.
   - Rear: an LD45 rear pad leading the rear shell and the crank covers.
   - Both are backed by aluminium plates strutted to the chassis spine, and
     sized from tip energy and the manufacturer's stress–strain data.
3. **Rebuild the CAD** (fit check, mass export).
   - Model the new pads as foam contacts in the free-base plant.
   - Update the structural screen for the single fork arm, the cantilever
     axle, the hub web and the bumper mounts.
4. **Rerun the six passive tips**, then reinforce only what still fails.
   - Use the conservative belt-load bound; report the belt honestly if it
     still fails.
5. **Mass checkpoint**, then adopt the masses and inertias.
6. **Rerun** the all-pose sweep and the full 107-case acceptance plus
   convergence.
7. **Record** the results and renders.

Heat paths, closed shell windows, fairing styling and repo cleanup are
step 3 and step 4.
