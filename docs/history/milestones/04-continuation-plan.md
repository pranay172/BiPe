# Milestone 4 continuation — qualification, full control core, power and fit

Planned 1 October 2026, immediately before the remaining implementation authorized by the user. This extends the existing M4 plan; M5 remains in the overview. Measured results go in `04-continuation-results.md`. Preserve the initial M4 evidence in Git and use separate continuation output folders.

## Order and scope

1. Audit exact AK40-10 V3 / AK48-2405-2D-A2 manufacturer product links, current manual, official repository and software resources. Save dated URLs/hashes and an evidence matrix. Do not substitute older AK40 or other driver parameters. If public evidence cannot close a field, keep it null and prepare a precise manufacturer-question document without sending it.
2. Implement the missing portable C estimator, stationary gyro calibration, leg kinematics/COM/equilibrium, feedforward/local leg servo, coordinate mapping and joint limits. Integrate the C core into a sensor-only host adapter and drive the full unconstrained simulation suite with it. Build a complete control application boundary and a motor-inhibited target; physical CAN transmit/current/local-PD mapping remains qualified-profile gated. Keep synthetic/test-only profiles separate.
3. Select real candidate parts for a conditional 6S power assembly: protected logic branch, pack monitoring/BMS interface, independent stop/watchdog disconnect, precharge, capacitance and regenerative dump. Derive bounds from finite simulated loads and conservative declared electrical assumptions. Create native component-level electrical CAD and BOM; test relevant device/passive circuit models, startup, watchdog/stop loss and battery-open regeneration. Exact actuator voltage/current limits, BMS qualification and real device behavior remain gates where evidence is missing; no fabrication or flashing.
4. Integrate carrier/power candidate envelopes into a separate FreeCAD electronics-fit study within the M3 allocation, including module heights, board mounts, connector plugs and wire paths. Reconcile explicit component mass estimates against existing allocations; do not silently reduce validated masses/inertias or raise the 2.5 kg ceiling. Record any failed fit or mass target. Rerun acceptance after relevant physical/controller changes.

## Acceptance and checkpoints

- Qualification matrix cites exact-variant primary evidence or clearly identifies unknowns; real hardware enable cannot become true from an incomplete profile. Manufacturer questions are saved for the user, not sent.
- C/Python estimator, kinematic, feedforward and controller comparisons cover posture extremes, noise/delay, finite inputs, startup and sign/zero faults. Meaningful C tests, host sanitizer and target builds pass.
- All original 27 articulated acceptance cases and half-timestep convergence run with the C sensing/control implementation, free base, finite actuator limits and physical contact. Traces/configurations record implementation and sensor settings. Changed physical/controller assumptions also require fixed-height and appropriate uncertainty checks.
- Power sources load as native KiCad, pin/footprint maps are checked, ERC/DRC have no unexplained violations. Real MPN ratings and conditional load/regen bounds are documented. Behavioral assumptions are not presented as device or hardware qualification.
- Fit study saves native CAD and machine-readable clearances/dimensions/mass arithmetic. Electronics geometry derives from SI robot configuration; fabrication dimensions convert only at the CAD/electrical drawing boundary.
- Make focused local Git checkpoints after plan, firmware verification and power/fit verification. Save results and updated delivery verification. Full M4 closure still requires its exact-variant and implementation gates; future physical validation is recorded separately from digital completion.

Public motor evidence may prevent final electrical release, but it does not prevent porting the complete controller, creating a conditional circuit candidate, or inspecting fit. Complete that independent work before reporting any remaining external dependency.
