# Milestone 0 Results

Completed on 30 September 2026 as a preliminary definition and sizing study. The next stage may begin with the serial leg concept and the recorded assumptions; real actuator selection and detailed mechanical clearance remain open.

## Environment and CAD checks

The isolated environment is at `.venv/` in the repository; installed package versions are pinned in [requirements.lock](../../../requirements.lock). MuJoCo 3.14.0 stepped a falling cube, which settled within 0.11 mm of the expected floor height. Rendering produced an inspected 960 × 640 image using NVIDIA OpenGL on the RTX 5050 Laptop GPU. The GPU has 8151 MiB of reported VRAM. Device access was hidden inside the restricted process, explaining the earlier failed check.

FreeCAD 1.1.1 executed the export script. The 15-object assembly saved to FCStd, reopened, and contained valid shapes. Its STEP export completed. A separate 20 × 30 × 40 mm test solid round-tripped through STEP with a volume of 24000 mm³. GUI interaction has not been tested.

## Preliminary dimensions and sizing

| Quantity | Computed value |
| --- | --- |
| Assumed total robot mass | 2.02 kg |
| Margin to the working 2.5 kg ceiling | 0.48 kg |
| Fixed-posture blockout envelope | 133.85 × 189.00 × 252.89 mm |
| Proposed overall height range | 191.07–278.14 mm |
| Link lengths and wheel diameter | 90 mm links; 80 mm wheels |
| Wheel angular speed at 0.3 m/s | 7.5 rad/s |
| Translational acceleration torque at 0.5 m/s² | 0.0202 N·m per wheel, excluding balancing, drivetrain losses, and wheel acceleration inertia |
| Maximum static knee endpoint torque in the sampled range | 0.808 N·m |
| Knee torque with the proposed 2× allowance | 1.616 N·m |
| Approximate hip moment from link and knee self-weight | Up to 0.116 N·m; horizontal endpoint loads and dynamic loads are excluded |
| Fixed assembly centre of mass relative to axle | 9.02 mm forward; 124.27 mm upward |
| Static equilibrium body pitch at the fixed posture | −4.15° using the stated coordinate convention |

The mass and motor envelope inputs are assumptions. The knee calculation uses the total ground reaction as an endpoint load, making it a conservative preliminary estimate for this static case; it is not a full inverse-dynamics calculation. The 2× allowance does not establish jump or impact capacity.

## Mechanism decision

Use two serial joint actuators plus a wheel drive per side for the first experiment. It provides independent fore-aft and vertical axle placement, which is useful for posture and balance studies. The schematic parallelogram alternative has fewer actuators but couples its axle motion to a circular path and requires separate linkage closure and bearing analysis. Neither option has a confirmed parts list.

The blockout reserves space for motor envelopes and shows a compact folded-leg arrangement. Mount interfaces overlap intentionally; bearings, fasteners, mechanical stops, wiring clearance, and manufacturing tolerances are not yet designed. It is an editable layout model, not a manufacturing assembly.

## Artifacts and reproduction

- [Shared parameters](../../../config/robot.json)
- [Concept comparison](../../../artifacts/milestone-0/concepts.png)
- [Mass and load study](../../../artifacts/milestone-0/mass-and-loads.png)
- [Editable FreeCAD blockout](../../../artifacts/milestone-0/blockout.FCStd)
- [STEP blockout](../../../artifacts/milestone-0/blockout.step)
- [Sizing data](../../../artifacts/milestone-0/sizing.json), [CAD checks](../../../artifacts/milestone-0/cad-check.json), and [environment checks](../../../artifacts/milestone-0/environment-check.json)

Run `.venv/bin/python -m bipe.sizing` from the project directory to regenerate the calculations and figures. Run `freecad.cmd cad/export_blockout.py` for CAD exports, and `MUJOCO_GL=egl .venv/bin/python -m bipe.smoke` for physics and graphics checks. This desktop environment requires permission for GPU access and the FreeCAD snap launcher when commands run under its sandbox.
