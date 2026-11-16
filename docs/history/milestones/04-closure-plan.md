# M4 digital closure plan

Prepared 1 October 2026, immediately before implementation, from `fefea17`.
M5 remains in the overview. Results will be recorded separately.

The original M4 acceptance is an electronics/firmware digital design: native
schematics/PCB/BOM, relevant circuit analysis, and simulated sensing/communication
faults. Hardware commissioning, supplier-confirmed missing actuator parameters,
manufactured conductor/thermal/timing qualification and fabrication release are
separate gates. Closing M4 must not set the hardware gate true or invent a real
AK48 mapping. The preceding candidates exposed genuine digital integration gaps;
those must be resolved before a digital-complete decision.

## Implementation order

1. Archive the 1449-file checkpoint, accepted SI configuration and user-saved M3
   CAD hash. Recheck current manufacturer download evidence; preserve missing or
   conflicting data instead of inferring exact-driver limits.
2. Consolidate reverse blocking, TVS and revised SAFE5 into the existing 66 × 60 mm
   power-board allocation; revise the carrier buck/ADC. Include an independent
   startup monitor with default-low permission and finite energy/time supervision,
   plus explicit interfaces for thermal/energy measurements. Verify native pins,
   ERC/DRC/parity, heavy current paths and component envelopes. Keep prior boards
   and rejected separate-module mass evidence intact.
3. Replace the 0.1 m future-fall shortcut with a mechanical-energy inventory:
   kinetic energy plus gravitational potential relative to the ground, capacitive
   reserve and bounded positive driver work. Screen all saved nominal, variation,
   convergence and unpowered-fall trajectories. Derive a complete stop reservation
   and temperature-admitted event budget; adjust cooldown/duty rather than crediting
   imaginary cooling. Retain overheating, excess work and external-energy failures.
4. Integrate the operating policy with the portable application through a tested
   permission/stop/disable boundary. Implement and independently test the startup
   monitor. Exercise reset debt, stale sensing, deadline/transport loss, overload,
   unavailable real profiles and shutdown. Build host/sanitizer and locked Teensy
   targets. An operational adapter cannot use unknown driver fields.
5. Reconcile board/components/roof/pack/harness mass and seven rigid-body tensors
   from the consolidated native layout and CAD. Check native fit/removal/reopen.
   Run the physical acceptance suite against any revised physical candidate and
   sensor inputs before adopting it; retain a complete snapshot and traces.
6. Run a closure verifier against explicit acceptance criteria, commit implementation
   and validation, and seal the final evidence. Set digital M4 complete only if all
   digital gates pass. Hardware qualification and fabrication release remain false.

## Acceptance and limits

- One consolidated power board fits the existing allocation; the whole robot's
  estimate remains ≤2.5 kg without consuming camera/pack/harness uncertainty silently.
- Native electrical rules/pin parity/connectivity pass; high-current geometry and
  voltage domains are analyzed independently of DRC. ADC inputs tolerate the declared
  transient envelope. Actual parasitics and ceramic bias remain stated assumptions.
- Startup short/load cases stop within their thermal reservation. Policy/application
  integration preserves queue/PD disable and cooldown across reset. Production cannot
  authorize synthetic or incomplete real evidence.
- Stop/fall reserve uses energy conservation and available potential to the ground;
  sampled unpowered motion cannot escape through an arbitrary velocity/height cutoff.
  External pushing/downhill travel and unknown actuator storage remain hardware/domain
  limits. Full motor work cannot persist beyond the independently specified cutoff.
- Finite-contact, free-base locomotion acceptance passes with the final physical
  snapshot if changed. No pose constraints or state clamps are introduced.
- Manufacturer and physical build gates are delivered as explicit, actionable
  requirements; digital completeness does not describe a hardware-validated robot.
