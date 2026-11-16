# M9 steps 3–6 — execution plan (2026-10-04)

Proposed work below; the CAD checkpoint is the achieved result. This session
preserves the M4 profile/evidence, the 1,800 g allocation/200 g reserve and the
user's unstaged M3 assembly. No push, cosmetic closure, CAN-FD adapter or PCB work.

1. **Check the mandatory fall-gate prerequisites first.** Independently audit
   the adopted CAD load paths, catalog capacities, shell/link allowables,
   clutch isolation, tire/belt retention and stopping compliance. Recompute
   the existing energy/load screen without overwriting its historical output.
   Check directional/lateral-contact sensitivities at nominal stance, at rest
   and 0.3 m/s, including rotation. Manufacturer typical properties and assumed
   stopping travel cannot silently become certified bounds. If missing
   allowables/isolation or a factor below 2 makes the gate fail, investigate
   reasonable options within the authorized geometry/mass limits, record the
   evidence and **stop**, as required by the task; do not claim later acceptance.
2. **Step 3, if prerequisites permit:** create a selectable M9 free-base
   MuJoCo five-bar tree/physical loop closures from all 21 SI CAD tensors.
   Add finite RO40/4:1 torque-speed limits, rotor inertia, current-loop lag,
   nonzero Gates-based belt compliance/lost motion, dry journal friction and
   1.7–2.0 Nm hip slip couplings. Sense actual outputs after the clutches;
   retain M4 selection. Map inherited posture commands to CAD heights:
   nominal 100 mm, crouched 70 mm and folded 45 mm hip-to-axle (overall
   232.3/202.3/177.3 mm). Update sensor resolution/latency from vendor data.
   Verify closure, mass/COM/tensors, finite energy and passive coupling before
   closed-loop experiments; commit this implementation checkpoint.
3. **Step 4:** plan the controller implementation in detail when it starts.
   Identify at multiple heights using actual estimator/filter states, 5 ms
   packet transport, local/host sample-and-hold, FOC lag and belt states.
   Verify linear predictions against nonlinear small-perturbation runs before
   any Riccati solve. Compare scheduled LQR and retuned cascade with torque
   saturation/anti-windup. Port the chosen five-bar kinematics, odometry and
   support feedforward/controller to portable C; require C/Python equivalence.
4. **Step 5:** run 27 inherited cases plus convergence and the M8 yardstick
   with `--controllers current`, using the geometric lowest posture. Require
   ±2 N·s survival; target strict final-2-s recovery ≤2°/0.03 m/s; require
   slopes through 10° with rollback ≤0.10 m and hold cycles ≤1.5° p-p at
   0/0.3/0.6° backlash and the documented belt range. Save configurations,
   decimated traces and current/regen/thermal/runtime screens. Repeat the full
   acceptance after changes affecting the physical model or controller.
5. **Step 6:** perform unpowered forward/back/side contact tips at rest and
   0.3 m/s, with stated passive drive behavior and CAD rotational/contact
   energy. Require justified compliance, every stated factor ≥2, qualified
   torque isolation, no tooth jump/alignment loss and stated allowables.
   Unknowns fail. Stop on a failed gate after reasonable effort.

Validation/results are recorded separately in a short results document with
key numbers, commands and commits. Reuse existing sizing/tests/suites; no new
seal/verifier scripts. New saved artifacts target <50 MB and ≤1 replay video.

## Clarification and resumed implementation (Claude, 2026-10-04)

The prior stop was premature. Steps 3, 4 and 5 are independent of fall
acceptance and will be implemented in order. Missing material/retention/
compliance information is engineering work, not an early stop condition.
Finish independent work before reporting any unresolved gate. Stop only
when reasonable locomotion tuning still fails an absolute target, or when a
necessary geometry/mass remedy exceeds the 5.914 g allocation headroom.
Historical preflight results remain an audit, not achieved physics acceptance.

**Step 3 detailed implementation:** add an explicit `m9` profile generated
from the adopted 21 CAD tensors and SI geometry; leave default `m4` intact.
Use two independent hip cranks/rods per side with physical foot loop joints,
a free chassis and ground/tire/shell/link contact. Represent independent
rotors, belt-input gears and output cranks; apply dissipative finite belt
spring/damper lost-motion forces and torque-limited hip slip interfaces,
without pose/state projection. Include manufacturer rotor inertia, finite
RO40 torque/speed/current lag and load-dependent dry journal friction.
Map legacy 40/60/65 posture labels to 100/70/45 mm height through five-bar IK;
retain the inherited crouch excursion ≥40 mm by mapping its upper 30 label
to 118 mm. Sensors use LSM6DSO assumptions, AS5047P outputs after clutches and
an explicitly selected RLS interpolation; account for five-bar wheel-carrier
rotation in odometry. Test mass/COM/tensors, loop closure, gravity/support,
physical coupling passivity and unpowered free response. Run the available
suite at checkpoints, including expected failed controller cases until tuned.

Steps 4/5 detailed plans are added immediately before those stages. In
parallel, develop the separate fall gate: source 6061/7075, named shell
material, fasteners/inserts and knocked-down allowables; derive tire/pad/
shell compliance bounds and actual impact loads from six passive contact
simulations. Check belt jump/retention and bearings with combined loads.
Evaluate the lightest necessary remedy; rerun CAD clearance/mass for geometry
changes, preserve the 200 g reserve, and report any headroom shortfall.

**Step 4 detailed implementation (starting after the step-3 plant checks):**
settle a finite-drive cascade operating point at nominal/crouched/folded and
intermediate heights. Identify the full deterministic nonlinear sample map,
including MuJoCo tangent state, motor/clutch/coupling history, 1 kHz local PD
sample/hold, 200 Hz host hold, IMU anti-alias states, packet queue (5 ms),
estimator complementary/posture/rate/velocity/odometry states and local
encoder-rate filter. Account explicitly for Coulomb/lash nonsmooth operating
conditions. Compare centered finite-difference predictions with nonlinear
small disturbances/input changes over a stated interval; do not solve Riccati
until this agrees. Remove only uncontrollable/unobservable gauge modes using
a measured controllability/observability reduction; verify the reduction.
Compare a posture-scheduled observer/LQR (runtime sensor/output/input history,
never simulator truth) against retuned cascade with motor-saturation
anti-windup. If local nonsmooth response prevents the prediction gate, retain
and report that model/design failure, and complete independent cascade effort.
Port the adopted law, five-bar COM/support and carrier-rate odometry to C;
retain M4 exact branches and C/Python tests. Save operating configurations,
linear matrices and prediction traces, then run complete M9 acceptance after
final model/controller changes. Reuse the M8 yardstick with an explicit M9
profile and absolute 0.10 m rollback threshold; M4 thresholds/evidence remain.

**Step 5 detailed execution (starting after the controller prediction study):**
freeze the physical law, sensor configuration and portable cascade source,
then run `bipe.robustness --profile m9 --controllers current` against its
saved configuration. The complete set is 107 runs: the inherited 70 M8 cases,
eight low/high Gates stiffness/lash corners, 27 inherited locomotion cases,
and two half-timestep checks. Require all 27 and convergence, nominal hold
survival and strict recovery at both signed 2 N·s pulses, all slope cases
through 10° with rollback ≤0.10 m, and ≤1.5° hold cycles at 0/0.3/0.6° wheel
backlash and every belt corner. The folded command is geometry-derived
45 mm hip-to-axle, not the M4 joint limit. Preserve the inherited case checks;
report M8 numbers only as historical comparison.

Write each completed case and its decimated trace immediately. Resume only
when plant, sensors, gains, requested cases and runtime source hashes match;
changed physical/controller sources require a complete new run. Aggregate
actual six phase-current histories, conditional bus current, negative work
and estimated net regeneration at 25.2 V. Screen cold and repeated duty with
explicit current-convention and winding-to-air resistance sensitivities;
report thermal qualification gaps and 850 mAh runtime by observed scenario.
Review the saved plots, run Python/native/ARM regressions and C equivalence,
then commit measured results. If an absolute locomotion target fails, make
reasonable plant/controller corrections and repeat this entire acceptance.
Complete the independent six-direction/speed fall engineering before any
final stop caused by an over-headroom structural remedy.

**Step 5 delay-robustness tuning checkpoint:** complete the frozen-source
107-case run and preserve its failures. Compare lower pitch/speed bandwidth
and gentler commanded acceleration in isolated candidate builds against the
25 ms wheel-lag drive/combined and 15 ms transport/double-noise cases. Use
full inherited tracking/stopping checks, not survival alone; also check signed
2 N·s hold recovery before selecting gains. Keep physical limits, sensor
errors and acceptance thresholds unchanged. Adopt only a justified cascade
candidate, repeat all 107 cases, and record any remaining absolute failure.
Two exactly equivalent placement/telemetry optimizations may be adopted before
that mandatory repeat after numerical parity checks. Correct the electrical
postprocessing for temperature-dependent copper resistance and inverter losses;
keep the measured histories and explicit unqualified thermal assumptions.

**Final cascade comparison before the full repeat:** isolated full runs show
that pitch P=1.3/D=0.10 with drive speed P=0.30/I=0.20 and acceleration 3
passes both delayed drive variants, while speed P=0.40/I=0.10 passes both
strict signed push recoveries. Implement explicit hold-versus-drive outer
gains selected only by the user's velocity command, with unchanged sensor
feedback/finite limits. Keep the integral state bounded and apply the selected
gain to anti-windup and reported reference consistently in Python/C. Optional
gains default to existing values, preserving M4. Verify equivalence across
mode transitions; test both full delayed drives/combined and signed pushes,
then freeze and repeat all 107 acceptance cases if the combination is viable.

**Clutch numerical fidelity check before adoption:** reported gear/output slip
below the 1.85 N·m release bound may be native friction regularization rather
than physical release. Measure relative drift and force in a nominal hold.
If significant, tighten only the four real mechanical clutch constraints using
MuJoCo's direct velocity-damping reference and high finite impedance; keep the
torque bound, free base, belt compliance and all masses/geometry unchanged.
Require negligible subthreshold drift relative to encoder resolution, actual
release under overload, and passive energy behavior. Reidentify prediction
models and rerun all 107 locomotion cases plus six impacts after any correction;
do not inherit acceptance from the softer clutch model. Record numerical
regularization separately from hardware stiffness/calibration assumptions.

The audit rejects very high direct damping: it destabilizes finite-step native
friction. The selected correction is five native NoSlip iterations, which remove
subthreshold friction drift while preserving the finite Coulomb capacities.
Rebuild native island bookkeeping before constraint projection/reference after
material contacts are updated; otherwise the installed engine's postsolver can
crash. Coupon release/passivity and one-second free-robot drift checks precede
adoption. NoSlip also changes contact/journal friction, so this is explicitly a
new physical simulation: use separate full-suite, impact and prediction outputs.

**Belt constitutive fidelity before final acceptance:** the corrected clutch
reveals a large hip-belt force residual: the unconstrained inverse-mass
backward-Euler predictor can disagree with the actual spring/damper law at
the solved next state when native loop/clutch constraints react. Keep the
current run/source frozen as discrete-model evidence. Prototype a constraint-
aware implicit coupling in an isolated project-local temporary checkout;
retain all SI masses, stiffness/lash, finite capacities and physical joints.
Require the actual endpoint force to agree with its spring/damper law, finite
overload release, passive energy and timestep sensitivity before adopting.
No base constraints or state projection. If corrected, repeat prediction,
full 107-case acceptance and six passive impacts from a fresh frozen source.

**Final wrap-up freeze (Claude, 2026-10-04):** time-box the existing belt
correction; do not open further model-fidelity work. The isolated native
candidate still fails branch consistency, so do not adopt it. Freeze the
10360ae physical/controller sources, resume the identical saved 107-case run,
repeat the six impacts and prediction check, and retain one actual C/sensor
replay. Use the conservative timestep/load-law envelope in the failed fall
gate, explicitly report the residual as open fidelity work, and do not tune
thresholds or spend the reserve. Delete superseded working artifacts (Git
retains committed evidence), keep the final results to about two pages,
commit locally and stop with the measured shortfalls/options.
