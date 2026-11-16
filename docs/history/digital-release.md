# Digital release r1

M5 delivers the conditional digital robot design. It does not release physical
fabrication, procurement or operational motor firmware. Start with the
[results](milestones/05-results.md), [assembly sequence](assembly-and-service.md),
[harness/retention detail](harness-and-retention.md) and
[hardware qualification](hardware-qualification.md).

## Contents and units

| Content | Location |
| --- | --- |
| Adopted SI configuration / controllers | `config/` |
| Accepted native assembly / full assembly STEP | `artifacts/milestone-4/closure/cad/integrated-robot.FCStd`, `.step` |
| 26 local custom-part STEP/STL exports / native HLR SVGs | `artifacts/milestone-5/cad/parts`, `cad/projections` |
| 27 dimensioned reference sheets | `artifacts/milestone-5/drawings/reference-drawings.pdf` |
| Reference BOM, mass ledger and 13 native harness links | `artifacts/milestone-5/inventory/` |
| Native KiCad sources and local libraries | `electronics/power-integrated`, `electronics/carrier-integrated` |
| Gerbers, drill, placement, translated schematic reference sources/PDFs, net identity/ERC/DRC | `artifacts/milestone-5/electronics/` |
| Metric assembled/exploded Blender file and verified renders | `artifacts/milestone-5/presentation/` |
| Firmware source and inhibited target build evidence | `firmware/`, `artifacts/milestone-4/closure/firmware/` |
| Configurations, traces and measured acceptance/fault/thermal evidence | `artifacts/milestone-4/closure/` and preserved baseline artifacts |
| Tools, dependency pins and Blender MCP wheel/addon | `tools/`, `requirements*.lock`, `artifacts/tooling/blender-mcp/` |

Native CAD, STEP, STL and HLR SVG drawing coordinates use millimetres. Robot
parameters, CAD inventory and Blender geometry use metres. Custom-part STEP/STL
files remove the source object's assembly placement; left/right exports retain
their actual handed geometry. The complete assembly STEP retains the 40-degree
nominal pose. Blender is a presentation scene, not a manufacturing or balance
model. Its exploded offsets do not establish a collision-free removal path.

Native schematic reference copies translate drawing objects onto A0 (power) and
A1 (carrier) sheets to
avoid the original carrier's clipped symbols. Net/pin identities are compared
with the original netlists and ERC is repeated. The accepted electrical sources,
board layout, configuration and physical behavior are unchanged. Generic BOM
specifications and raw placement exports still require assembly exceptions and
exact part selection before any manufacturing handoff.

## Build and check the package

Build from the original Git checkout, which retains checkpoint `c1bca53` for
historical identity checks. Verification of an extracted package needs no Git.

```bash
.venv/bin/python tools/build_digital_release.py
mkdir -p /tmp/bipe-release-check
tar -xzf artifacts/milestone-5/release/bipe-digital-r1.tar.gz -C /tmp/bipe-release-check
python3 /tmp/bipe-release-check/bipe-digital-r1/tools/verify_digital_release.py \
  /tmp/bipe-release-check/bipe-digital-r1
```

The standard-library verifier runs without FreeCAD, Blender, a GPU or installed
robot Python dependencies. It verifies all included hashes, accepted configuration,
2391 historical M4 identities, CAD/export/electrical/BOM/MCP evidence and explicit
hardware holds. Compare the archive digest in its adjacent `.json` before
extraction. The package is deterministic for fixed input bytes: sorted paths,
normalized permissions/owners and zero archive timestamps. Release archives are
generated outputs and are not duplicated inside Git.

The final archive digest, repeat-build comparison, extraction/portability reports
and repository-level `artifacts/milestone-5/verification.json` stay outside the
archive. This avoids making a package contain its own digest or final extraction
report. Included component evidence and verifier tools are sufficient to repeat
the checks; save new reports outside the extracted tree so they do not become
unexpected package files.

M4's seal is not expanded or overwritten. Changed delivery documentation is
archived from checkpoint `c1bca53` and mapped in M5 baseline identity. The user
modified M3 assembly remains untouched and is excluded from this archive; the
package's historical M3 assembly is the committed `c1bca53` version, with explicit
provenance. The accepted robot is the **M4 integrated assembly**.

Use the M5 verifier for this delivered tree. M4's original verifier deliberately
identifies new files as outside its historical seal; that is not a new locomotion
failure. Preserve the original configuration, traces and measured results rather
than relabeling historical evidence after a physical edit.

## Regenerate presentation and drawing outputs

With the existing project-local MuJoCo/MCP environment, KiCad tooling and running
FreeCAD bridge:

```bash
.venv/bin/python tools/call_freecad.py cad/export_delivery.py --timeout 45
uv venv --python .venv/bin/python .venv/delivery
uv pip install --python .venv/delivery/bin/python -r requirements-delivery.lock
.venv/delivery/bin/python tools/build_delivery_drawings.py
bash tools/electronics-env.sh python3 tools/export_delivery_electronics.py
PYTHONPATH=. .venv/bin/python tools/build_delivery_inventory.py
```

In a fresh owned Blender bridge session, generate the native presentation with
`tools/call_blender.py tools/build_blender_delivery.py`. The generator refuses an
existing loaded project or existing owned scenes. Rendering uses
`tools/render_blender_delivery.py` in a background Blender process. Native
save/export formats may embed application metadata; archive determinism is
tested from fixed saved deliverables, not asserted across software versions.

Restore the simulation environment using its existing pinned requirements and
the [M4 closure instructions](milestones/04-closure-results.md). Saved acceptance
is conditioned on the exact model and controller assumptions. Re-run the full
suite after a physical model or controller change. No hardware actuator enable,
calibration, fabrication or construction is performed by these commands.

Recorded MJCF files retain their original absolute asset paths as sealed
evidence. The relocation helper resolves those assets strictly inside an
extracted release, without changing the saved files:

```bash
.venv/bin/python tools/replay_digital_release.py /tmp/bipe-release-check/bipe-digital-r1 \
  --verify-all --report /tmp/bipe-portability.json
MUJOCO_GL=egl .venv/bin/python tools/replay_digital_release.py \
  /tmp/bipe-release-check/bipe-digital-r1 --scenario sensors_combined \
  --output /tmp/bipe-replay --render
```

The first command compiles all 28 accepted/convergence models, forwards three
saved poses per model and loads the exact accepted C library. The second makes
an external replay copy and renders measured states. Neither reruns or replaces
locomotion acceptance. New experiments should write separate output folders.
