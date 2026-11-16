# Repository layout

| Path | Contents |
|---|---|
| [`bipe/`](../bipe/) | The Python package. Shared pieces sit at the top: the plant model, kinematics, sensing, the firmware bindings and the robustness runner. Each milestone has its own subpackage. |
| `bipe/m10/` | The M10 plant, kinematics, falls and the CAD-to-config step |
| `bipe/m11/` | Electrical requirements, power, boards, harness and firmware timing |
| `bipe/m12/` | Speed envelope, camera model, tag detection, follower and the following runners |
| `bipe/pi/` | The vision pipeline for the Pi Zero 2 W: lens, tag detection, tracking and the view-angle gate |
| `bipe/m13/` | The learned policy on the full plant, export, parity and the firmware handover |
| `bipe/m13/rl/` | The mjlab training environment: observations, actions, rewards, domain randomisation, training and evaluation |
| `bipe/m14/` | The policy evaluator, the stress suite and the feasibility studies |
| `bipe/m15/` | Fallen states, the get-up evaluation, moves, following with the policy, the roll-fault study |
| `bipe/demo/` | Demo scenes, recording, rendering and video editing |
| [`firmware/`](../firmware/) | The C core that runs on the Teensy 4.0 and in every simulation: estimator, balance, supervisor, follower, the policy forward pass and runtime (`src/locomotion/`), the M11 hardware layer, and the Teensy images. Tests in `firmware/tests/`. |
| [`config/`](../config/) | The robot in SI units (`robot.json`: dimensions, masses, limits), controller gains, sensor models, speed, following and training-plant settings |
| [`cad/`](../cad/) | Headless FreeCAD scripts that build the M10 assembly and its checks (sweep, sections, mass, manufacturability), and the Blender render script |
| [`electronics/`](../electronics/) | KiCad projects for the M11 carrier and power boards |
| [`artifacts/`](../artifacts/) | Generated outputs with the configuration that made them: CAD and renders, falls, acceptance and speed records, board outputs, training checkpoints, exported policies, demo card stills |
| [`docs/`](README.md) | This documentation, the milestone plans and results, the hardware plan and studies |
| [`tests/`](../tests/) | The pytest suite |
| [`tools/`](../tools/) | Generators for firmware headers and board files, FreeCAD and Blender bridges |
| `local/` | Gitignored scratch: demo recordings, training runs, logs |

## Configuration

`config/robot.json` is the single source for dimensions, masses and physical
limits. CAD converts to millimetres only at the FreeCAD boundary, and
`bipe.m10.adopt_cad` copies the CAD masses back. Generated firmware headers
embed the hashes of the configuration they came from, and a test checks that
the committed M11 hardware outputs are current.

## Records

Each record directory under `artifacts/` holds its results beside the
configuration it ran with (and, for most, the hashes of the controller and
configuration files), so a result can be traced to its inputs. The policy
runners refuse to write into a directory that holds another policy's results.
