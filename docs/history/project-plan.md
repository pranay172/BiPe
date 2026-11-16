# Bipedal wheeled robot project plan

This project will produce a compact robot design that can eventually be built in hardware. The first implementation will balance, drive, turn, and crouch in simulation. We will use the supplied build series for engineering lessons and a small commercial wheeled camera robot (Mondo Robotics) for the desired proportions and character.

Begin with a specification, a rough component layout, and a simple balance model. Mechanical dimensions and controller behaviour must develop together because changing the body, battery location, or leg posture changes the balance problem.

## Agreed scope

- Produce a design intended for eventual hardware construction.
- Implement balance, forward and reverse driving, turning, and crouching first.
- Work on the inspected Ubuntu computer, using the user's existing experience with robotics, CAD, and AI.
- Develop the robot's own compact mechanism and enclosure using the references as guidance.
- Keep the current build entirely digital, while preparing for possible physical construction later.
- Prefer appropriate, widely used, well-documented, budget-friendly real components. No fixed procurement budget or purchasing region is required for the current digital phase.
- Assume fabrication will be outsourced. Decide the battery runtime target after actuator and battery sizing.

Camera tracking, person following, jumping, automatic recovery from a fall, and advanced terrain traversal are later extensions. Reserve a possible camera mounting location and payload allowance during packaging, without implementing those features now.

## Initial design targets

These are proposed starting targets. They become requirements only after the first packaging and actuator feasibility review.

| Item | Starting point | Evidence or next check |
| --- | --- | --- |
| Size | Aim for the same compact class as the reference robot | Mondo lists 215 × 180 × 180 mm, but does not identify the measurement pose on the reviewed page. Define our folded and operating envelopes separately. |
| Mass | Initial working ceiling of 2.5 kg including battery | A project assumption. The reference robot's advertised 1.75 kg is a reference; our ceiling must be checked against candidate actuators and the CAD mass budget. |
| Driving | Start at 0.3 m/s on level ground | A deliberately modest simulation target; higher speeds follow successful braking and turning tests. |
| Leg motion | Controlled symmetric crouch and extension | Determine the height range from geometry, collision checks, and available torque. |
| Power | Battery and voltage remain undecided | Select from actuator requirements, electrical margins, packaging, and desired operating time. |
| Appearance | Rounded central body, compact folded legs, prominent wheels, expressive front | Interpretation of the reference robot's visible exterior, rather than a claim about its internal mechanism. |

The mass model must include motors, gearbox inertia, wheels, battery, electronics, shell, fasteners, and cables. Maintain explicit margins for unmodeled parts.

## Proposed mechanism and control architecture

Use a six-actuator baseline: one hip actuator, one knee actuator, and one wheel actuator per leg. This follows the architecture described in the Tazer build series. Check this arrangement against the compact envelope before committing to it; a linkage arrangement is a candidate if actuator packaging makes the serial leg unsuitable. The reference-robot sources reviewed here do not establish its internal actuator count or linkage dimensions.

Start with both legs held at a fixed posture so that the robot behaves approximately as a wheeled inverted pendulum. Build the balance and speed controller on that reduced model, then release the leg motion. Keep the robot free to fall in the physics simulation; visual animation or a constrained support fixture cannot establish balance.

Use a cascade controller as the first baseline: the speed loop commands a desired body pitch, the balance loop commands wheel motion, and a modeled motor loop applies limited torque. Include saturation, command rate limits, and anti-windup. Add heading control through differential wheel commands. Use a position hold layer for stationary tests if needed to prevent drift. Evaluate LQR later against this working baseline, with a model and estimator that use the same units and actuator input conventions.

As the legs move, calculate the centre of mass and equilibrium body pitch for each posture. Check whether gain scheduling is required across the height range. Use ideal simulator state initially for debugging; later acceptance tests must use simulated IMU and encoder measurements, with noise and delay.

Define coordinates consistently: X forward, Y left, Z up; body pitch is rotation about Y. Record positive joint directions, zero positions, units, wheel radius, and gear ratios in one shared parameter source. Generate CAD and simulation inputs from those parameters wherever practical. Update simulation masses and inertias from CAD as its fidelity improves.

## Tools

| Work | Proposed tool | Reason and status |
| --- | --- | --- |
| Mechanical CAD | FreeCAD | Already installed; parametric dimensions, solid models, and manufacturing exports. |
| Physics and controllers | MuJoCo with Python, NumPy, SciPy, and Matplotlib | Installed in project-local `.venv`; fixed-height physics, controller checks, plotting, NVIDIA rendering, and viewer operation verified. |
| Exterior studies and presentation | Blender | Already installed; useful for shape exploration and renders. Engineering dimensions remain in CAD. |
| Electronics | KiCad with suitable SPICE models | KiCad 9.0.9 and ngspice 42 work project-locally; a native routed controller carrier and conditional power studies are saved. Full actuator power assembly remains open. |
| Firmware | C or C++ with a hardware abstraction layer | Teensy 4.0 selected; portable C protocol/supervisor/wheel core and locked diagnostic image build. Operational motor integration awaits exact driver qualification. |
| Learning | Native MuJoCo rollouts and JAX/Optax PPO | M6 uses project-local `.venv/rl`, CPU contact/C sensing/control and CUDA neural updates. GPU physics remains a separate microbenchmark. |
| Integration | ROS 2 if later features justify it | Revisit for sensors, navigation, or person following; the first balance experiment can run directly in Python. |

These tool choices are engineering recommendations based on the current scope. See the linked official documentation in [reference notes](../references.md). Validate one working CAD-to-simulation export path early, including units and inertias.

## Milestones and completion evidence

| Milestone | Work and deliverables | Completion evidence |
| --- | --- | --- |
| 0 Define and size | Requirements, two rough mechanism layouts, mass budget, joint map, initial wheel and actuator load estimates | A feasible candidate layout with stated assumptions and a comparison of size, mass, motion range, and torque needs. |
| 1 Balance at fixed height | Primitive MuJoCo model, controller, parameter file, plots, reproducible run command | Proposed tests below pass with a free base and finite actuator limits. |
| 2 Add legs and turning | Leg kinematics, joint limits, crouch trajectories, posture compensation, heading control | The robot drives, stops, turns, and completes repeated crouches without losing balance or violating modeled limits. |
| 3 Develop the compact CAD assembly | Selected actuator envelopes, bearings, battery, shell, wiring space, manufacturing tolerances | Interference-free assembly through the intended motion range; updated masses and inertias still pass simulation tests. Iterate with milestones 1 and 2. |
| 4 Design electronics and firmware | Power architecture, schematic, PCB, BOM, protocol layer, startup and fault states | Electrical rule checks and relevant circuit analysis pass; simulated sensor and communication faults exercise the intended firmware behaviour. |
| 5 Deliver the digital design | CAD sources, STEP and mesh exports, drawings, PCB outputs, BOM, firmware, simulation scenarios, assembly notes | A reproducible release package, with assumptions and hardware verification still required stated explicitly. |
| 6 Establish RL environment and first policy | Sensor-derived residual interface, seeded curriculum, PPO weights/checkpoints, classical/untrained/trained comparison | Trained experimental hybrid policy, portable export parity and full physical acceptance evaluation; improvement and candidate acceptance reported separately. |
| 7 Controlled learning refinement | Persistent hold/drive/turn/combined tasks, aligned costs, explicit FP32 math and three equal-budget PPO seeds | Independent locked comparisons, complete inherited suites for every seed and measured advancement gates; a negative finding is a completed study. |
| 8 Robustness yardstick and control baseline | Current M4 plant: real slopes, pushes, backlash and model mismatch; corrected wheel limits; cascade/LQR comparison | Permanent C/sensors-m4 suite with measured margins, all 27 inherited cases and convergence; report candidate rejections and missed targets. |
| 9 Compact redesign (CAD checkpoint adopted) | Covered five-bar, inboard RO40 hips, geared RO40 wheels, 28 mm tires and rounded shell | [Amended plan](milestones/09-compact-redesign-plan.md); [checkpoint](milestones/09-cad-checkpoint-results.md): 1794 g allocated +200 g reserve, 199.5 mm width, 222 clear poses. Separate SI CAD profile adopted; plant/control/electrical/fall acceptance pending. |

Use evidence to advance between milestones. Set calendar estimates after the initial setup and mechanism experiment reveal the main uncertainties.

Create a detailed document in `docs/milestones/` immediately before implementing each milestone. Keep future milestones in this overview until their implementation starts. Each detailed plan must state scope, dependencies, modeling assumptions, implementation order, deliverables, and measurable completion checks; record results separately so proposed targets remain distinguishable from achieved results.

## First simulation acceptance targets

The following thresholds are proposed engineering targets, not achieved results. Record the model parameters, controller settings, and test inputs with every run.

- Balance on a level surface for 60 seconds at the chosen fixed leg posture, without an external support or hidden constraint.
- Recover from initial pitch offsets of +5 and −5 degrees; settle within 2 degrees of the modeled equilibrium within 3 seconds.
- During a separate stationary hold test, keep travel within 0.1 m over 60 seconds after settling.
- Track a ramped forward and reverse command up to 0.3 m/s, then brake to a stationary hold while respecting modeled torque and speed limits.
- Log pitch, pitch rate, position, velocity, requested and applied actuator effort, and saturation duration. Avoid passing a test through sustained saturation or an artificial clamp on robot motion.

First use ideal sensing to establish the controller and correct signs. Before milestone 2 is considered complete, repeat the suite with simulated sensors, plausible actuator lag, and documented parameter variation. Add defined push disturbances, turning manoeuvres, and repeated crouch cycles at that stage. Test convergence under a smaller physics timestep so an apparent success is not a numerical artifact.

## Actuator and electrical selection

Select real parts from calculated loads and manufacturer documentation. For each candidate, record continuous and peak torque, speed under load, dimensions, mass, voltage, current, reduction ratio, reflected inertia, thermal limitations, available control modes, protocol timing, and encoder behaviour after power cycling. Joint actuators and wheel actuators may require different choices.

For leg joints, verify how output position becomes known at startup: an output-side absolute measurement or a suitable homing arrangement must support the chosen mechanism. An encoder count or product variant name alone does not prove that requirement. Include documented calibration and zeroing behaviour in the firmware design.

For the electronics, analyze startup transients, regenerative braking, current paths, voltage margins, and actuator enable behaviour. Model communication latency and command timeouts. Verify actual component footprints and pinouts before routing. These checks address specific issues observed in the supplied build series; its component names and voltage descriptions are not a substitute for datasheets.

## Digital verification limits

Simulation can establish internal consistency and predict behaviour under stated assumptions. It cannot certify the completed physical robot. Motor calibration, tire friction, printed-part strength, backlash, thermal performance, electrical noise, and real timing still require physical measurement later. Maintain a hardware verification list alongside the digital results.

## Next implementation work

Milestone 4's digital electronics/firmware criteria are complete. The final integrated boards, conditional stop/startup/thermal analyses and portable runtime pass verification; the 2.499519 kg candidate was tested before adoption. Its full C locomotion suite passes 27 cases plus convergence, fixed-height regression and 12 uncertainty cases. See [final M4 results](milestones/04-closure-results.md) and [hardware qualification requirements](hardware-qualification.md).

M5 now delivers dimensioned reference sheets, native custom-part/PCB exports, a consolidated BOM, pin-level harness and retention proposals, assembly/service notes and a reproducible digital package. See [M5 results](milestones/05-results.md) and [release instructions](digital-release.md). Its deliverable is a conditional digital prototype: manufacturing tolerances, real cable sweeps/retainers, exact actuator/pack commissioning and loaded electrical/thermal qualification remain held.

M6 completes the first RL experiment on this unchanged plant. Its [detailed plan](milestones/06-rl-environment-and-policy.md) uses six bounded torque corrections around the required classical C controller. The first trained policy passes all 27 cases and convergence, but has a 4.0% lower validation return and poorer speed/turn tracking than the classical baseline; it remains experimental. See [measured results](milestones/06-results.md) and the [workflow](rl-workflow.md). Training success, hybrid locomotion acceptance and hardware qualification are distinct.

M7 completes that controlled learning study under its [committed detailed plan](milestones/07-learning-refinement-plan.md). The three seeds each received 196,608 transitions, followed by locked independent tests and full inherited suites. Median independent score gain is 3.6%, with one regressing seed and two candidate threshold misses across the new and inherited tasks. The advancement gates fail; see [measured results](milestones/07-results.md) and the [reproduction workflow](learning-study.md). The classical controller remains the default. Further training on these level-ground tasks has no demonstrated consistent advantage.

M8 completes a robustness yardstick on unchanged M4 geometry and mass. Its [plan](milestones/08-robustness-and-control.md) and [results](milestones/08-results.md) record 396 C-controlled runs. The original cascade gains remain the default with a corrected 1.5 N·m wheel cap: all 27 inherited cases plus convergence pass, physical robustness improves from 51/70 to 59/70, and strict ±2 N·s recovery remains missed. The retuned cascade and scheduled LQR fail inherited acceptance. The 10° rollback is 78 mm with residual oscillation; 0.6° backlash gives 1.00° late pitch peak-to-peak. These measured limits form the redesign yardstick, not hardware validation.

M9 now has a revised CAD checkpoint under its committed amended plan: 1794 g allocation, 200 g reserve, 199.5 mm width and 222 sampled clearance poses pass. Four lighter RO40 belt joint drives, covered legs, foam-core 28 mm tires and lean load-sized supports replace the rejected 2297 g layout. The separate `m9_cad` SI profile includes 21 rigid-body COMs/tensors; the active M4 plant remains unchanged. Subsequent implementation needs a new free-base five-bar plant, delay/estimator-aware controller design, inherited/robustness acceptance and the complete fall/impact and power/thermal gates. Future terrain/recovery learning should use a frozen revised physical baseline and distinct measurable tasks. Standalone learned balance and hardware policy deployment remain later work.

Before fabrication, complete M9 physical, fall/impact, tolerance and electrical/thermal qualification. Its remaining 6 g above the fixed reserve leaves little room for additional modeled hardware; any additions must retain the 1800 g allocation ceiling. Update CAD, mass/inertia and physical acceptance together when geometry or limits change. Camera following and jumping remain later scope.

## Current status

As of 30 September 2026, [Milestone 0](milestones/00-results.md), [Milestone 1](milestones/01-results.md), [Milestone 2](milestones/02-results.md), and [Milestone 3](milestones/03-results.md) are complete under their documented digital assumptions. The earlier 2.02 kg M2 blockout and acceptance remain saved as the baseline. The historical M3 component candidate is **2.49223 kg** and approximately **144.85 × 284 × 260.89 mm** in its nominal unpitched 40° pose, with forward eyes and backward knees.

The 33-solid assembly passes 220 CAD clearance poses, structural mating-contact checks, and fresh native/STEP round-trip. Seven rigid bodies carry the declared CAD COMs and full inertias. The free-base six-actuator robot passes all 27 articulated scenarios, extra seeds, separate parameter variations, half-timestep body/load convergence, fixed-height regression, and 12 added inertia/COM/angle-uncertainty scenarios with one moving controller. Twenty Python tests, source checks, NVIDIA replay and the 37 s live viewer pass. The native FreeCAD MCP bridge is operating.

The knee's rated-to-worst-RMS margin is approximately 2.39 under the saved simulations. Battery scenarios support a provisional one-hour planning target, with actual voltage compatibility and losses unresolved. Native CAD is a component-based digital candidate, not a production release or hardware-validated design. Exact actuator timing/firmware, thermal/backlash behavior, final electronics, pack/BMS/regen interfaces, fits, harness/retention details and physical performance still need confirmation. See [mechanical notes](mechanical-design.md) and the [M3 detailed plan](milestones/03-compact-mechanical-assembly.md).

At the initial M4 checkpoint on 1 October 2026, M4 added a 66 × 60 mm controller carrier passing ERC/DRC/parity, 19 C startup/fault checks, 10,000-sample wheel-port equivalence, ngspice ADC/precharge/regen studies, and five unconstrained physics fault experiments. All 27 locomotion cases plus convergence pass with coarser host CAN feedback, fixed-height regression passes, and 22 Python tests/Ruff pass. The Teensy diagnostic ELF/HEX build succeeds with hardware RUN inhibited. Full M4 completion remains gated by exact driver evidence, selected power hardware, operational firmware and packaging qualification; see its results.

The M4 continuation plan was committed before the full C implementation. The estimator, kinematics, feedforward and six-motor control now pass packet equivalence and all 27 unconstrained cases plus convergence. A conditional native power schematic/PCB and device-bound behavioral analyses are saved. The electronics fit study fails: 39.32 mm stacked height versus 10 mm allocated, and 96.116 g estimated electronics versus 90 g allocated. Next revise packaging and reconcile the complete power/pack/harness mass, then revalidate any adopted CAD/mass changes. Exact driver evidence and hardware application/power qualification remain gates; see [continuation results](milestones/04-continuation-results.md). M3 physics inputs remain unchanged.

The M4 packaging revision retains the outer body; its reconciled estimate is 2.494762 kg, including the future camera reserve. Seven COM/inertia tensors now include explicit electronics/cell/protection estimates and cable/fastener mass distributed onto the legs. Native envelope clearance, conservative removal sweeps and a 143-solid native/STEP round-trip pass. The internal heat spreader does not establish the assumed 60°C case limit under repeated braking; this is the next concrete power-design gate. See the packaging results for final current-configuration simulation and compiler evidence. All earlier results above remain historical, with their snapshots preserved.

The thermal/application checkpoint adds an unadopted 2.495538 kg exposed-roof/pack candidate, conditional periodic thermal bounds with retained negative cases, native 70 µm power-copper and shunt pick-offs, exact-part source auditing and a tested reusable application boundary. See its results for the historical snapshot.

The latest M4 protection checkpoint adds a native reverse-input/TVS/supply module, conditional hot-transient bounds, startup and three-node thermal calculations, independent SPICE and a portable operating policy with positive stop-work metering. Rule/pin checks, host/UBSan/production inhibition and Cortex-M7 compilation pass. Standalone mass fails at 2.502935 kg; integrate into the existing boards before adoption. A proposed 6.5 J stopping reserve covers its declared sampled motion bounds, but the saved unpowered fall exceeds the height/speed domain. Actual cutoff, sensor/meter and thermal-interface bounds remain unqualified. Exact driver/pack data and the real adapter remain gates. See [protection results](milestones/04-protection-results.md); evidence checks keep M4 and hardware qualification false.

The final M4 closure integrates the rejected standalone protection functions into the existing boards, revises sensing and the carrier buck, adopts the exposed thermal roof and reconciles seven rigid-body tensors. Both native boards and CAD checks pass. A full mechanical-energy inventory covers 33 trajectories including falls; conditional thermal duty is 40°C/600 s with hotter/frequent failures retained. The final verifier sets digital M4 complete and keeps hardware/fabrication gates false. Earlier status paragraphs above describe preserved historical checkpoints.

M5 digital delivery is recorded separately in its results and release verification. No physical model/controller changes were made; M4 acceptance identities are preserved rather than replaced by presentation renders.

On 2 October 2026, M7 records a completed experimental learning study with a negative advancement finding. The three candidates select updates 192/64/192; 313 training episodes complete without termination. Seeds 20261020 and 20261022 pass all 27 inherited cases and convergence. Seed 20261021 passes 26/27 and convergence, missing the sensor-turn final-speed limit. On the 12 independent tasks per control, classical passes 12, frozen M6 passes 10 and the M7 candidates pass 11/12/12. These endpoint threshold misses are retained with physical traces. Strict FP32 export/data checks pass, and the recorded plots/replay are inspected. M4/M5 physical inputs and M6's sealed evidence remain preserved; archived copies of its two overview documents authenticate later live documentation changes. Hardware and fabrication remain held.
