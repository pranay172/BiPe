# M4 continuation — protection and operating policy

Planned 1 October 2026 immediately before implementation, extending `160aed1`. This is remaining M4 work; M5 stays in the overview. Achieved results will be recorded separately.

The accepted robot and C controller remain unchanged. Preserve the user's M3 native CAD and the 1357-file delivery seal in a new `artifacts/milestone-4/protection/baseline/` checkpoint. Keep new source evidence, complete experiment configurations, measured traces and build identities in the protection tree. No real profile, power permission, procurement, flashing or external messages are enabled.

## Implementation

1. Establish a manufacturer-supported protection interface: reverse-input blocking before both branches, raw-input bidirectional TVS, local positive clamps, higher-voltage switch candidates, and supplies compatible with the actual clamp bound. Analyze hot-reversal with a charged bus, inrush, isolated regeneration, harness turn-off energy and temperature/layout uncertainty. A TVS pulse rating does not establish an actuator-safe bus voltage or sustained braking sink. Retain failed historical ratings and unqualified corners.
2. Save an editable native KiCad protection module study and its pin/net/BOM audit. Route its declared power paths, run ERC/DRC/parity and verify native geometry. Record integration requirements for the existing power/carrier boards and new dimensions/mass; do not imply a standalone module has passed whole-robot fit or spend existing mass margin before a complete reconciliation.
3. Calculate precharge energy and readiness over resistor/capacitance/standby-load corners, including repeated failed starts. Derive a conservative stopping/fall-energy allowance from the shared robot snapshot and explicitly bounded velocities/heights. Resolve peak power and energy separately; unbounded falls and slopes remain outside the operating domain.
4. Implement a portable, independently tested C operating-policy candidate: timestamped local-temperature and bus/dump feedback, fixed event reservations, finite startup attempts, accumulated heat without invented cooling credit, stop reserve, cooldown, stale/invalid feedback and latched faults. Reset must not erase heat history. The policy cannot switch off the independent dump to conceal an overload; the real adapter remains inhibited. Test boundary, interruption, wrap, reset and overload behavior with synthetic measurements, and compile for Cortex-M7.
5. Verify relevant existing firmware/application checks, preserve the accepted controller/physical configuration identity, and seal the new evidence. Update results and remaining gates. Make focused plan, implementation and verification commits. Run physical acceptance only if a physical model/controller affecting it is adopted; new candidates need their own saved checks.

## Checks and boundaries

- Primary-source ordering codes and pin mappings are linked and locally fingerprinted where retrievable. Typical curves and assumed parasitics remain labeled.
- Protection calculations include reverse plus retained output voltage, all downstream supply ratings, local overshoot and TVS pulse/repetition limits. Any unresolved hot/device/actuator limit keeps hardware qualification false.
- Native rule reports are inspected; current paths are audited independently of connectivity. Whole-robot fit and mass are required before integrating/adopting a module.
- Thermal policy refuses incomplete real evidence, preserves heat/reservations across ordinary reset, checks peak voltage independently of integrated energy, and requests stop early enough to retain its declared stop allowance. No thermal sensor or software model alone guarantees arbitrary external energy.
- Save traces/configurations for startup success/failure, braking, repeated events, stale sensors, invalid values and restart. Host warnings/sanitizer and MCU compilation pass. Existing locked Teensy target stays locked.
- M4 remains open where exact AK48, controlled protected-pack specification, real thermal interfaces, native integration or hardware adapter evidence is missing; completed independent digital work is recorded separately.
