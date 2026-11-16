# M5 digital delivery results

Implemented 1 October 2026 against the [M5 plan](05-digital-delivery-plan.md).
M5 delivers a **conditional digital prototype release**, with dimensioned
reference drawings and inhibited firmware. Hardware qualification and
fabrication release remain false. See [release instructions](../digital-release.md)
and the repository-level final report at `artifacts/milestone-5/verification.json`.
Archive digests and fresh-extraction reports are adjacent outputs outside the
archive; the delivered verifier reproduces the checks without those reports.

## Measured deliverables

| Deliverable | Evidence |
| --- | --- |
| Baseline | 2391 M4 sealed identities preserved from checkpoint `c1bca53`; changed delivery documentation archived separately; adopted SI config and accepted physical sources unchanged |
| Native CAD | 178 valid single solids; 26 local custom-part STEP/STL exports; per-part STEP bounds error below 1e-8 m, maximum relative volume error 1.284e-8 |
| Reference drawings | 27 dimensioned A3 sheets plus native HLR SVG projections; local/assembly frames, hole patterns, material/process, nominal dimensions and explicit fabrication holds; all rendered sheets inspected |
| BOM | 202 unique reference rows; native power 92 and carrier 31 electrical references; four mechanical H1-H4 holes per board excluded from electrical quantity parity; nonoverlapping accepted mass ledger retained |
| Harness | 13 native inter-board pin mappings; actuator/CAN/roof/stop/arm topology, routing/flex proposals, retention stacks and service sequence recorded |
| PCB outputs | Four-layer power and two-layer carrier Gerbers, metric drills/placement; original boards pass ERC/DRC with zero unconnected items; enlarged schematic reference copies preserve net/pin identity and pass ERC with zero warnings |
| Blender | Official Lab addon/server configured; real stdio MCP exposes 26 tools, measured cuboid edit/save/cleanup and fresh native reopen pass; one Lab repository, localhost 9877 |
| Presentation | Two native metric scenes with 178 CAD objects each; native reopen and renders inspected; maximum CAD/mesh bounds discrepancy 0.012433 mm; eyes face +X, knees -X |
| Firmware/physics | Exact accepted C library and inhibited target build evidence delivered, alongside configurations, 27 free-base C cases, convergence, 12 uncertainty cases and shutdown/thermal traces; no physical/controller changes |
| Release | SHA-256 manifest and deterministic archive; fresh extraction verification, relocated model/C-library checks and corruption/extra-file rejection recorded under `artifacts/milestone-5/release/` |

The archive is generated rather than duplicated in Git. Native source files,
exporters, BOM/detail documents, manifests and measured verification are committed
at focused checkpoints. Its file count/digest/size are given in the adjacent
archive metadata. Every retained physical assumption remains an assumption;
presentation and reference drawings do not establish actual manufacturing fits.

## Details established during delivery

The harness review identifies opposite arm signal polarities: power `ARM_RAW`
is active high, carrier `ARM_N` active low. Specify two isolated normally-open
contacts operated together; do not connect both to a shared contact. Exact switch
selection, mass and independent startup/application commissioning are still held.

The earlier 3 mm bundle example is insufficient for two 1.9 mm OD power wires.
A 4 mm bundle in the 7.8 mm gap leaves only 1.9 mm per side, below the 2 mm target.
The [harness details](../harness-and-retention.md) require individual/staggered
routing or a revised layout and a complete moving cable sweep. Existing proxy
volume clearance is not cable-flex qualification. Proposed strap anchor stacks,
bearing/tire/roof/PCB retainers and resistor clamps are reviewable definitions;
their missing geometry and mechanical qualification are explicitly held.

The native carrier's original symbol grid extended beyond its paper. M5 creates
separate translated A1 carrier/A0 power reference copies, with library/rule
dependencies and exact net/pin equivalence. It does not change the accepted boards
or their electrical behavior. Raw placement outputs still need assembly exceptions
for wired capacitor/module/remote sensor interfaces and exact generic-part MPNs.

## Blender setup and observed limitation

Both empty Lab caches described the same official MCP addon. Preferences had not
saved a Lab repository; setup saved one, enabled the verified 1.0.3 addon and
backed up original preferences locally. Server 1.0.2 is pinned to official revision
`dbbf836ad4b1025f14a2b3b504c43903f39e0b04` in the project-local environment.
Port 9876 was occupied but did not answer the official protocol, so 9877 is used.

GUI MCP round-trip worked. The native library-write call for the larger generated
scenes then crashed Blender 5.2.2. Regular save-copy in a fresh supported background
bridge session, native reopen and rendering succeeded. This is the current stable
path. [Setup instructions](../../blender-mcp.md) explain GUI autostart, the single
listener requirement, Codex reload and the retained diagnostic evidence. No
connection blocker remains for the verified workflow.

## Remaining hardware gates

Mass remains **2.499519 kg estimated**, only **0.481 g** below the 2.5 kg ceiling;
there is no reliable manufacturing contingency. The 284 mm width and six-actuator
cost also deserve review before hardware commitment. Exact actuator voltage,
driver/encoder/timing and moment limits, professional pack/BMS interfaces,
actual fasteners/fits/retention/flex and loaded power/thermal/SOA still need
qualification. The modeled 40 C / 600 s domain depends on unmeasured thermal,
sensing/reaction, storage and driver-work requirements. Diagnostic firmware is
not an operational image.

These holds are carried into the delivery rather than declared solved by renders,
nominal matching surfaces or metadata. See [hardware qualification](../hardware-qualification.md).
M5 adds no new locomotion claim and preserves the original user-saved M3 assembly;
the package's historical M3 file comes from the committed checkpoint instead.
