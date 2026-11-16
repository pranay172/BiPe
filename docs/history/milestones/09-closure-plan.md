# M9 closure plan — authorized 4 October 2026

Requirements amend the earlier M9 gates, not the measured results. Structure,
shafts/journals, forks/hubs/axles, belt/clutch protection, retention and bumper
mounts require factor ≥2 in every passive and powered fall. Cosmetic printed
parts may be replaceable; they must remain retained, avoid sharp/live exposure
and bypass electronics loads. Report their factors separately. Allocation may
reach 1.850 kg with ≥0.150 kg reserve; at most 50 g of the former reserve is
authorized here. Later fairings/closure remain outside this phase.

1. **Diagnose, then correct control.** Compare recorded physical chassis/axle
motion with measured five-bar carrier/encoder odometry, attitude correction,
hold latching and integral history. Correct an identified cause in Python and
C; preserve M4 behavior and finite torque/speed/current limits. Check combined,
crouch, delay and drive failures before freezing gains. Run all 107 inherited,
robustness/belt and convergence cases after the change; retain 70/70 physical
robustness, unchanged stopping/recovery/cycle thresholds and C equivalence.
2. **Protective fall state.** Implement a latched, estimator-only unrecoverable
state with explicit pitch/rate/roll thresholds justified against recovered
pushes. Rate-limit a fold command, bound wheel/leg torques and ramp to zero;
never constrain the free base. Record detection, posture and shutdown timing.
Extend the existing impact suite with 3–4 N·s pushes and an over-limit slope or
step; retain the six open-phase passive tips and all physical contacts.
3. **Structure and bumpers.** Use named foam with supplier data, mounted on
frame load paths at face/front/rear/corners/sides. Size supports, shafts,
forks, hubs and actual fastener features using the impact loads and conservative
125 µs/constitutive belt envelope. Resolve retention, tooth jump, alignment
and clutch isolation without claiming cosmetic skins as structure. Estimate
the complete incremental mass before adoption; stop/report options if it
exceeds the authorized 50 g reserve draw.
4. **Thermal path.** Size aluminium motor-to-housing/frame conduction and
exposed convection for each RO40 to ≤about 16 K/W winding-to-air, including
interfaces, existing housing area and added mass. State winding-contact and
airflow assumptions; rerun both catalog-current interpretations over saved
duty cycles. Do not qualify unmeasured hardware temperatures.
5. **Adopt and freeze.** Generate native CAD from SI robot.json, convert only
at FreeCAD, run the complete bumper-fitted clearance/service sweep, export
mass/COM/inertias and enforce ≤1.850 kg with ≥0.150 kg reserve. Re-adopt actual
tensors, freeze physical/controller sources, then run all 107 plus convergence,
passive/powered falls and the thermal screen again. Structural loads require
factor ≥2; cosmetic information and retention are separate.

Use existing CAD, tests and suites; no new seals/verifiers. Keep only final
configs/traces, necessary sensitivity evidence, CAD and one replay under about
50 MB. Add about one page of closure results to the existing M9 docs. Make local
plan/control/protection/CAD/final verification commits, never push and never
touch or stage the user's M3 assembly. If control remains below 27/27 after
reasonable diagnosis/fixing, finish feasible independent engineering and report
the failure. Keep unresolved fidelity explicit rather than opening indefinite
solver work.
