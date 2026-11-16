# M9 revised CAD checkpoint

**Current bumper-first status, 5 October 2026:** this remains the adopted
historical 1,794 g/199.5 mm layout. The authorized allocation is now ≤2,150 g
with ≥150 g reserve. The unadopted bumper/frame/heat and optimistic belt option
reaches 2,164 g before fairings/retention and fails side-stroke coverage; the
mass stop prevented fitted robot CAD, all-pose clearance, tensors and pose renders.
See [current results](09-physics-results.md). Limits and results below describe
the earlier CAD checkpoint.

**1794 g allocated ≤1800 g**, plus the unchanged **200 g unallocated reserve**:
**1994 g**, 6 g below the 2 kg ceiling. This saves **503 g** against rejected
commit `6e31457`, adds rounded leg covers and widens tires **14→28 mm**.
Shell walls remain **2 mm**; tolerances remain unqualified.

| Ledger (grams rounded independently) | Rejected | Revised | Delta |
|---|---:|---:|---:|
| Four hip motor cores | 760 g | 240 g | −520 g |
| External bearings, shafts, forks | 494 g | 387 g | −107 g |
| New hip housing/sensing/reduction/clutches | — | 181 g | +181 g |
| FOC boards | 18 g / two | 53 g / six | +36 g |
| Shell/frame/face | 187 g | 163 g | −24 g |
| Links/guards and covers | 85 g | 104 g | +19 g |
| Wheels/tires | 94 g | 70 g | −23 g |
| Wheel housings/sensing/reduction | 182 g | 148 g | −34 g |
| Battery; wiring/connectors/fasteners | 165; 130 g | 165; 130 g | unchanged |
| Battery retention; MCU/power packaging | 62 g | 32 g | −30 g |
| **Total** | **2297 g** | **1794 g** | **−503 g** |

[21 body tensors](../../../artifacts/milestone-9/cad/mass-properties.json) use native
volumes/catalog masses. Allowances: **75 g wires/connectors +55 g fasteners**.
Motor internal distributions/electronics remain declared envelope estimates.
PA12 hubs, upper-density LD70 foam, 1 mm TPU tread and bond films provide
28 mm tires on **159 mm track**.

**Dimensions:** **199.5 mm width**; operating/crouched/folded heights
**232.3/202.3/177.3 mm**, lengths **204.2/206.2/210.7 mm**. The 0.5 mm width
margin needs tolerance qualification. The **222-pose sweep passes** at 1 mm height steps over 45–118 mm and
foot X=−10/0/+10 mm, with covers fitted: zero collisions/ground violations,
closure **2.8×10⁻¹⁷ m**. Unrelated belt/pulley pairs also pass all 222 poses;
only matching tooth engagement is excluded. Zero gap is at intended hinge
mates. The SI geometry and 21 body tensors are adopted under `m9_cad` in
`config/robot.json`; existing M4 physics/control fields are unchanged.
[Static fit](../../../artifacts/milestone-9/cad/packaging-check.json): **zero overlaps**,
all 274 native features valid. Battery withdrawal clears 200 mm with the face
removed; separate mating pieces retain their mass/attachment notes.

[Selection/load screen](09-joint-drive-selection.md): RO40/4:1/moteus-c1,
~**128 g joint hardware**, **0.51/1.49/0.53 Nm static/pulse/crouch** at 2 kg.
Opposed 12/20 mm plain journals, hollow 7075 shafts and connected cassettes
replace 6804 stacks. Hollow 17-4PH wheel axles add 8 g. Preliminary 3.00 J/5 mm
fall factors: **2.98 journals, 2.20 shaft, 2.03 fork web, 2.80 tube**;
wheel axle/bearings **2.39/2.08** for central radial contact. Full shell/cover/
fastener strength and calibrated slip-release impact acceptance remain pending.
Six CAN-FD drives need AUX2 rotor SPI, host output SPI and new adapters/carrier;
thermal/bus/regen budgets remain pending.
Nonzero belt stiffness/lash and journal friction/clearance are saved; exact
tooth/adjuster machining and retention tolerances remain open.
**No MuJoCo/controller work began. The user's M3 file is untouched.**

[Native CAD](../../../artifacts/milestone-9/cad/five-bar-assembly.FCStd),
[sweep](../../../artifacts/milestone-9/cad/clearance-sweep.json),
[four poses](../../../artifacts/milestone-9/renders/pose-sheet.png),
reference comparison render (not kept in the repository).
FreeCADCmd 1.1.1/Blender CLI worked. Validation: 56 pytest passed, four optional
RL cases skipped; Ruff passes; 21 finite symmetric positive physical tensors
and mass closure pass.

```bash
M9_ACTION=build freecad.cmd cad/m9_assembly.py
M9_ACTION=fit freecad.cmd cad/m9_assembly.py
M9_ACTION=sweep M9_SWEEP_STEPS=74 freecad.cmd cad/m9_assembly.py
.venv/bin/python -m bipe.m9_sizing
blender --background --factory-startup --python cad/m9_render.py
.venv/bin/python cad/m9_render.py --sheet
```
