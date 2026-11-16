# M5 digital delivery plan

Started 1 October 2026, immediately before implementation. This document states
proposed work; measured outcomes belong in `05-results.md`.

## Scope and baseline

Deliver a reproducible, reviewable **digital prototype release**, using the M4
checkpoint `c1bca53` and the adopted SI parameters in `config/robot.json`. Include
native CAD, engineering reference drawings, component exports, electrical output,
a consolidated BOM, harness/retention details, assembly/service instructions,
diagnostic firmware, simulation evidence and a verified manifest. Add a Blender
presentation scene derived from CAD and verify the requested MCP installation.

Do not promote unqualified parts, nominal fits, thermal assumptions or inhibited
firmware into a hardware/fabrication release. M4's measured mass is an estimate
of 2.499519 kg: the 0.481 g margin is not a manufacturing allowance. Preserve the
user-modified M3 assembly. Engineering geometry and physical limits remain in
`config/robot.json`; presentation geometry does not change the physical model.

## Dependencies and assumptions

- M4 native integrated assembly, seven rigid-body tensors, both native KiCad
  projects and the sealed 27-case acceptance plus convergence are the baseline.
- Use FreeCAD's existing MCP bridge for native CAD export/inspection and Blender
  5.2's official Lab MCP addon for presentation. Verify SI conversion explicitly.
- Purchased actuator/pack interfaces, bearing fits, thread strength, tire
  retention, mounting, cable flex and thermal contact remain qualification holds.
- Manufacturing drawings are dimensioned digital reference sheets with clear
  part-specific holds, not supplier-approved fabrication instructions.
- Detail proposed retention and harness provisions separately; do not add
  unweighed hardware to the adopted robot or silently consume a reserve twice.
- Reuse M4's measured physics if its configuration/controller/geometry are
  unchanged. Any adopted physical change requires CAD mass reconciliation and
  the full relevant acceptance suite before delivery.

## Implementation order

1. Save baseline identities, resolve the duplicate empty Blender repository,
   install a pinned official addon/server in the project-local environment,
   configure Codex and prove real MCP initialize/list/query/edit/save/cleanup.
2. Extract native per-part geometry, SI bounds and topology. Export STEP and
   presentation meshes; make dimensioned assembly and custom-part drawings with
   datums, hole patterns, material/process and explicit unresolved interfaces.
3. Consolidate mechanical, purchased, native board, external electrical and
   allocated items into a traceable BOM without double-counting mass. Document
   fastener engagement, bearing/pack/roof/PCB retention and electrical harness
   net/pin mapping, routing, slack, strain relief and service sequence.
4. Export KiCad Gerbers, drill, placement and schematic PDFs, retaining exact
   native stackup and unresolved manufacturing/commissioning holds. Include
   source files, library dependencies and inhibited target firmware outputs.
5. Create a CAD-derived assembled and exploded Blender scene and rendered
   views. Check imported bounds, front eyes (+X), mirrored legs and metric scale.
   These are presentation outputs, not balance or clearance evidence.
6. Build a deterministic release archive, source/evidence manifests and
   regeneration/verification commands. Verify it in a fresh extraction, record
   measured results and commit implementation and verification checkpoints.

## Deliverables

- `artifacts/milestone-5/`: CAD exports/inventory, reference drawings, BOM,
  electrical outputs, presentation scene/images, baseline and verification.
- `docs/assembly-and-service.md`, `docs/harness-and-retention.md` and release
  instructions identifying relevant hardware holds and accepted operating domain.
- Project-local Blender MCP launcher/test, setup evidence and reproducible M5
  exporter/bundler/verifier. No personal credentials enter the bundle.
- Native/config/source/evidence package with SHA-256 identities and a separate
  archive digest; preserve the historical M4 seal without rewriting it to cover
  new M5 files.

## Measurable completion checks

| Check | Required evidence |
| --- | --- |
| Baseline | All 2391 M4 sealed identities available from current files or checkpoint; accepted robot/controller/firmware/CAD unchanged; original M3 user save preserved |
| MCP | One Lab repository, addon enabled, localhost bridge; real stdio MCP calls and owned-object cleanup; native saved test reopened |
| CAD exports | Every selected custom/purchased reference part has native object identity, valid topology, STEP/mesh hashes and measured bounds; reference sheets agree with config/native dimensions |
| Drawings/details | Assembly orthographic views, custom-part sheets, dimensions/datums and material/process; each unreleased fit, fastener, retention or tooling interface explicitly identified |
| BOM/harness | Board-reference counts reconcile with native BOMs; mass ledger sums to accepted configuration with reserves distinguished; inter-board pins/nets traced; assembly/service steps and retention holds recorded |
| Electronics | Both boards produce native Gerber/drill/placement and schematic outputs; layer/output inventory matches declared stack; fabrication status remains false |
| Presentation | Import metric bounds match CAD within mesh tolerance, front eyes correct, native `.blend` reopens and render is inspected |
| Release | Deterministic archive rebuilt identically; fresh extraction verifies every manifest entry, no missing dependency or prohibited personal file; change/corruption is rejected |
| Completion | Results document links measured evidence and remaining hardware gates; focused local Git commits record validation |

Digital M5 completion does not mean procurement, physical assembly, operational
flashing or fabrication is approved. Those gates remain in
`docs/hardware-qualification.md`.
