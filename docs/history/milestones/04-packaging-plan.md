# Milestone 4 — electronics packaging and mass reconciliation

Planned 1 October 2026, immediately before this implementation. This is the next M4 checkpoint, not the start of M5. Results will be recorded separately in `04-packaging-results.md`.

## Baseline and scope

The validated robot weighs 2.492233 kg against a 2.5 kg project ceiling. The saved stacked carrier/power arrangement fails: 39.32 mm required above a 10 mm allocation. Its estimated 96.116 g also exceeds the 90 g electronics provision. Keep those failed-study outputs and all previous acceptance traces intact.

Revise the packaging, account for pack protection, cables, connectors, fasteners, mounts and heat rejection, and determine whether a compact arrangement can be adopted responsibly. Exact motor/driver qualification remains open; hardware enable stays inhibited. No procurement, fabrication, flashing or manufacturer messages are part of this work.

The existing M3 native file has a user-saved view/metadata change. Compare its engineering content against Git, record both file fingerprints and preserve that file. Create new CAD documents and artifacts under `artifacts/milestone-4/packaging/`.

## Implementation order

1. Verify the local MCP/FreeCAD connection and record the baseline geometry identity. Archive the current robot, electronics, power and delivery snapshots for historical checks.
2. Build an explicit, non-overlapping mass ledger. Retain the six motors, battery cells, future camera provision and mechanical solids. Split the 90 g electronics and 80 g wiring/fastener allocations into identifiable items plus explicit remaining allowances. Account separately for the 50 g battery-pack overhead. State manufacturer masses versus estimates, density calculations and unresolved purchased parts.
3. Evaluate alternatives in CAD: horizontal carrier with vertical/rear power electronics, relocated capacitor and an upper resistor bank; or an enlarged enclosure if the compact options cannot meet clearance, retention, wiring or thermal needs. Prefer retaining existing real electrical parts. Select alternatives only from manufacturer documentation and repeat energy analysis if capacitance or dump components change.
4. Model component courtyards, documented heights, connector mating space, insulated wiring routes, mounts and the resistor heat spreader. Check actual solid intersections, declared 2 mm clearances, access/removal and thermal feasibility under explicitly stated assumptions. Do not describe a package envelope as production STEP geometry or electrical release.
5. Adopt only a candidate with an auditable mass/inertia source and a clear account of unresolved details. Use `config/robot.json` for shared SI dimensions, masses and physical limits; CAD alone converts to millimetres. Preserve historical CAD and traces. If a candidate fails a requirement, save the failure and revise it or report the remaining concrete constraint.
6. For adopted physical changes, regenerate the C constants, rebuild host/target firmware and rerun the full 27-case C acceptance with convergence, fixed-height regression, relevant mechanical uncertainty and controller checks. Save configurations and measured traces in new folders. Keep free base, finite motors and physical contacts.
7. Verify native CAD reopen/STEP identity and any changed electrical source/pin/ERC/DRC checks. Update stage-aware delivery verification, documentation and focused local commits, recording validation at each checkpoint.

## Completion checks

- Historical evidence is preserved; user-saved M3 CAD is not overwritten.
- Every modeled gram belongs to one ledger line, including explicit uncertainty allowances and unselected pack protection. Component estimates are not presented as measured assembly mass.
- A saved CAD candidate has no unintended intersections and reports clearance/access exceptions explicitly; fit, heat rejection and hardware qualification are separate results.
- Any adopted geometry/mass change has seven consistent rigid-body COM/inertia tensors and passes the milestone acceptance suite with the compiled C controller.
- Updated evidence fingerprints distinguish previous configurations from current ones. M4 stays open for exact actuator documentation, current-rated power layout/device qualification and operational hardware integration.

Thermal case temperature, capacitor pulse current, connector/copper capability, pack/BMS behavior and outsourced tolerances still require justified design or later hardware measurement. Unchanged behavioral power tests remain valid only within their saved assumptions.
