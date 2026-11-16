# Milestone 4 — electronics and firmware

Planned on 30 September 2026, immediately before implementation. This document states proposed work and acceptance; measured results belong in `04-results.md`.

## Scope and dependencies

Build the digital electrical and firmware design for the accepted M3 six-actuator robot: power distribution/protection, regenerative-energy handling, MCU, IMU, CAN, startup, command limits and fault response. Produce editable KiCad sources, a component BOM, circuit-analysis inputs, portable compiled firmware, and reproducible fault experiments. No procurement, fabrication, flashing a physical robot, camera following or jumping is included. M5 stays in the overview.

M3 remains the saved mechanical/physics baseline. `config/robot.json` remains the source of SI physical parameters. The provisional 6S P28A pack has 25.2 V maximum and occupies 75 × 64 × 44 mm with 326 g allocated. Electronics occupy 70 × 64 × 10 mm with 90 g allocated. Only 7.8 g remains below the 2.5 kg ceiling: track real component envelopes and estimated assembled mass rather than silently adding parts.

## Evidence gates and assumptions

1. Confirm exact AK40-10 V3.0 / AK48-2405-2D-A2 voltage, MIT current/torque scaling, feedback/angle convention, calibration, timeout and local PD behavior from manufacturer material. The generic V3 manual omits this variant from its scaling table. Other AK variants are not substitutes. If evidence remains unavailable, retain a disabled hardware profile, build the documented family codec against explicitly synthetic test ranges, and record the unresolved gate. Passing synthetic tests does not approve motor commands.
2. Treat direct 6S connection, BMS charge acceptance, bus cutoff and clamp thresholds as conditional until the exact motor limits are known. Design logic electronics with adequate documented input ratings; simulate regeneration with the battery disconnected. A TVS alone is not the sustained braking sink. Cell phase current is not DC battery current.
3. Preserve free-base, finite-effort contact physics. Faults that remove torque may cause a fall; verify detection/inhibition, not an invented ability to remain upright without power. Healthy-feedback controlled stopping and immediate invalid-feedback shutdown are distinct policies.
4. Software timing thresholds, pack cutoff, thermal thresholds and power-loss estimates are engineering assumptions. Save them in versioned configuration and distinguish them from manufacturer ratings. Startup must require fresh sensors, all six unique actuator identities, stationary calibration and explicit arm; reset never automatically re-arms.

## Implementation order and checkpoints

1. Commit this plan. Install or make project-local KiCad, ngspice and a firmware toolchain available, preferring CLI/API access. Record reproducible commands and versions.
2. Save manufacturer source URLs/hashes. Resolve the actuator evidence gate as far as public material allows. Select practical MCU/IMU/CAN and regulated logic supply; define pins, axes, connectors, current paths, terminating resistor placement and the conditional power interface.
3. Implement profile-driven CAN encoding/decoding, coordinate conversion, rollover handling, scheduling/bandwidth accounting and a portable C/C++ supervisor behind a hardware abstraction. Compile with warnings enabled. Test malformed frames, limits, stale/duplicate data, rollover and startup. Checkpoint commit with validation.
4. Create editable KiCad schematic and a PCB candidate within the reserved board envelope. Use documented pinouts/footprints; export netlist and run ERC/DRC. State clearly if power/BMS/regen hardware remains a separate unresolved assembly. Do not manufacture an unresolved design.
5. Analyze divider/filter behavior, logic loading, startup and bus regenerative energy. Use ngspice where appropriate and save input decks and measured output, including battery-disconnect cases. Inspect schematic and layout visually.
6. Exercise startup and fault behavior with saved time traces and configuration snapshots. Cover communication loss, bus-off, invalid/stale IMU, encoder discontinuity, brownout, overtemperature, operator stop, missed deadlines and restart. Couple representative faults to free-base physics; report falls honestly. Rerun M3 acceptance if physical assumptions or the locomotion controller change.
7. Write results, evidence gaps, component mass/packaging estimate, hardware verification tasks, and reproducible validation commands. Commit the verified checkpoint. Update the overview to achieved work with remaining gates; do not declare complete if required evidence or checks remain unresolved.

## Deliverables

- `config/electronics.json` and an explicit unqualified actuator profile, with source references and test-only ranges kept separate.
- `firmware/`: buildable portable supervisor/protocol/HAL, tests and target integration notes; target binaries only if a real target toolchain is available and integration is implemented.
- `electronics/`: KiCad schematic/PCB/project, BOM, pin map, power interface and SPICE decks.
- `bipe/` verification/physics-fault utilities and `artifacts/milestone-4/` snapshots, tool reports, electrical-analysis data and fault traces.
- `docs/electronics-and-firmware.md` engineering decisions and `docs/milestones/04-results.md` achieved evidence and outstanding gates.

## Measurable acceptance

| Check | Proposed pass condition |
| --- | --- |
| Actuator profile | No hardware arm possible while voltage, scale, startup/zero, local gains and timeout are unqualified; exact-variant evidence required for hardware-ready integration. |
| Protocol | Byte-level golden tests for the documented V3 extended-ID layout; correct units/signs; reject bad DLC/ID/nonfinite values, invalid ranges and stale/discontinuous measurements. No silent transfer of V2 codec. |
| Timing | Six commands + six feedback at 200 Hz fit within 1 Mbps with conservative frame stuffing and margin; deadline faults tested; 1 kHz actuator-local loop remains a documented evidence gate. |
| Startup/faults | Zero effort before explicit successful arm; fresh six-actuator and IMU data required; latched faults inhibit within configured detection interval; reset requires fresh calibration and explicit arm. |
| Electrical CAD | Native schematic and PCB load in KiCad; ERC and DRC reports have no unexplained violations; footprint/pin mapping audit; reserve envelope met. Digital rule checks are not hardware validation. |
| Circuit analysis | Logic supply/input voltage ratings documented; ADC divider below MCU range across modeled bus envelope; filters match intended bandwidth; conditional regen model stays below stated design bus ceiling or produces an explicit failed case. |
| Physics | Representative shutdown experiments use free base, physical contact and existing finite limits; no pose clamps or false balance claims. Changed mechanics/controller rerun the full milestone acceptance. |
| Delivery | Commands reproduce tests and outputs; sources/configurations/traces saved; estimated mass and unresolved power/thermal/hardware requirements stated; focused local commits record validation. |
