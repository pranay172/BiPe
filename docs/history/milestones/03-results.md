# Milestone 3 — implementation record

Started on 30 September 2026. **Complete as a digital mechanical candidate under the saved assumptions.** The [detailed plan](03-compact-mechanical-assembly.md) was committed before implementation. Hardware/procurement and production-drawing qualification remain open. The earlier checkpoints below preserve the development history; final acceptance is recorded at the end.

## Completed actuator response study

All 16 combinations of 2, 5, 10 and 20 ms joint torque response with stationary hold, driving, crouching and combined motion passed using simulated sensors and the unchanged M2 controller. Every case retains its configuration, model, trace and metrics in `artifacts/milestone-3/actuator-response/`; `study.json` aggregates the results and the accepted M2 load demands.

This supports a response tolerance within the tested model. It does not measure the bandwidth of a real actuator or establish that its internal controller matches ours. The M2 2 ms assumption is not a demonstrated hardware requirement.

## Manufacturer shortlist

Numerical facts, exact variants, conversions and unresolved items are saved in `config/actuator-candidates.json`. Prices are indicative USD list prices checked on 30 September 2026, excluding shipping and tax.

| Candidate | Published torque basis | Mass | Main implication |
| --- | --- | --- | --- |
| [CubeMars AK40-10 V3.0 KV170](https://www.cubemars.com/product/ak40-10-v3-0-kv170-robotic-actuator.html) | 1.3 N·m rated, 4.1 N·m peak | 190 g | Integrated CAN torque/position control is close to the simulated interface. $135.90 each; six would be $815.40. |
| [FEETECH STS3250 / ST-3250-C001](https://www.feetechrc.com/en/562636.html) | 1.569 N·m rated, 4.903 N·m stall at 12 V | 74.5 g | A credible lightweight leg candidate. Position-servo gains, response and direct torque behavior are not established; manufacturer price is not listed. |
| [ROBOTIS XM430-W350-T](https://robotis.us/products/dynamixel-xm430-w350-t) | 4.1 N·m stall; store **estimates** continuous torque as 0.82 N·m | 82 g | Documented current control, but the continuous estimate is marginal against the old knee RMS demand with a 1.5× margin. $310.39 each. |

At the M2 knee RMS demand of 0.513 N·m, the nominal rated-to-demand ratios are approximately 2.53 for CubeMars and 3.06 for FEETECH. These ratios must be recalculated after incorporating actual component masses, inertias, friction and geometry. Stall torque is never used as a continuous rating.

The CubeMars rotor figure converts to 9.735×10⁻⁶ kg·m² at the motor and 9.735×10⁻⁴ kg·m² reflected through the 10:1 reduction. This is materially larger than M2's wheel assumption. An exploratory plant using these heavier modules and reflected inertias **failed** with the unchanged M2 controller; its three traces are saved under `candidate-plant/`. That trial retains the old nominal controller mass model and is a feasibility warning, not an acceptance test. Component integration needs model/controller work before a selection can be accepted.

## Source and tool checks

The native FreeCAD MCP connection works in this restarted session. The exact manufacturer drawing and STEP reference were downloaded with URL and SHA-256 records in `component-sources/sources.json`.

FreeCAD reports the supplier STEP as a valid compound of 254 faces in 46 open shells, **zero solids**, with a 53 × 40.2 × 53 mm envelope. It cannot provide reliable component mass properties or establish a closed manufacturing solid. Use the dimensioned mounting drawing and a declared uniform-mass surrogate; retain the STEP as a visual reference.

The [V3.2.0 generic manual](https://www.cubemars.com/data/cms/202602/ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator.pdf) linked by the product page does not list the exact AK40 V3 model in its protocol parameter table. Do not copy older AK40 protocol ranges, or larger-module voltage specifications, into firmware for this variant. Exact driver limits, command scaling, timeout behavior and local timing remain confirmation items.

## Work queued after the actuator-study checkpoint

Compare the assembled mass and reflected-inertia costs, choose the mechanical candidate, establish voltage/battery provisions, build parametric interfaces and service spaces through MCP, and check swept clearances. Transfer declared CAD mass properties to simulation, repeat the articulated acceptance suite and fixed-height regression in separate M3 output folders, and document outstanding physical verification. No purchasing or fabrication has been authorized or performed.

## Component assembly and model checkpoint

The candidate uses six AK40-10 V3.0 modules, 3 mm aluminium leg plates, stepped output adapters, supported 80 mm wheels, SKF 608-2Z outboard bearing envelopes, a removable printed front/rear shell, and a reserved six-cell battery. It is a digital mechanical candidate; exact drive firmware and electrical limits remain unresolved.

The native assembly has 33 valid solids and an estimated mass of **2.492233 kg**, including electronics, future camera, and 80 g wiring/fastener reserves. Only 7.8 g remains against the working ceiling: this is a tight estimate, not a manufacturing mass guarantee. The nominal unpitched 40° envelope is **144.851 × 284 × 260.888 mm**. Real motor and bearing packaging increases width substantially over the M2 layout.

A first fork design cleared nearby parts but had no load-bearing contact at its proposed lower-plate mount. It was corrected with a connected bolted ear above the plate window. The checker now verifies structural mating contact as well as overlap. The shell has screw passages, insert pockets, and tray tabs. All **220 sampled hip/knee poses** pass exact solid-interference and 2 mm nonmating-clearance checks. Analytical radial/lateral bounds supplement sampling. All required mating pairs touch; the battery's reserved 1 mm pads are the named exception. Fresh native and STEP reopening preserves 33 solids, with relative volume error approximately 1.39×10⁻⁹ and maximum bounding-coordinate difference 1.61×10⁻¹¹ mm.

Seven explicit rigid-body masses, COMs and full inertia tensors are transferred from declared CAD materials/component masses. The manufacturer surface STEP supplies no mass properties. The physical robot still has a free base, six driven hinges, zero equality constraints, finite actuator response/torque/speed, and wheel contact. CAD meshes are visual only; collision envelopes include housings, wheels, links and conservative fork boxes. Separate CAD checks establish self-clearance.

The software incorporates 16-bit encoders, 20 ms nominal response, 9.735×10⁻⁴ kg·m² reflected inertia per actuator, and a conservative 0.06 N·m Coulomb-friction proxy from published backdrive torque. That proxy is an assumption. Joint references and support feedforward are latched at 200 Hz; local encoder PD runs at 1 kHz. Module firmware timing and gain mapping have not been verified.

The heavier plant required controller tuning. Initial oscillatory runs and rejected gains are preserved under `controller-development/`. The first complete updated articulated run passes nominal cases, seeds, mass and floor-friction variations, and half-timestep convergence, but fails the 25 ms wheel-lag driving and 15 ms delay/doubled-noise checks. Its complete evidence is preserved in `controller-development/acceptance-v2/`. M3 remained open at that checkpoint; the final controller below resolves these checks.

At this checkpoint, 19 Python tests and Ruff pass, including independent MuJoCo gravity/Jacobian checks against the support calculation, exact mirrored CAD mass/inertia checks, source fingerprints, and free-base mass/kinematics checks. Fixed-height acceptance and further controller robustness work are ongoing.


## Final acceptance

The final physical configuration is the seven-body CAD mass model, six finite-torque/speed motors, 20 ms nominal torque response, 16-bit encoders, physical wheel contact and a free base. The moving controller uses pitch gains 2.0/0.24, speed gains 0.4/0.1 and bounded friction compensation. During rotation its friction direction follows measured wheel speed; near rest it blends into smooth torque-based breakaway compensation. The same controller is used across every articulated and uncertainty run. The live viewer now uses the same latched 200 Hz references/feedforward and 1 kHz local PD timing.

All **27 articulated scenarios** and the additional combined half-timestep run pass. This includes ideal and noisy/delayed sensor feedback, 60 s stationary holds and ±5° recovery, ±0.3 m/s driving/braking, turning, five crouch cycles, combined motion, two signed pushes, unpowered falling, two extra seeds, and separate +15% mass, friction 0.5, 25 ms wheel response, and 15 ms/doubled-noise variations. Acceptance thresholds were retained. No constraints or physical-state clamps were added.

Nominal sensor-feedback hold travels 21.8 mm over 60 s, with peak pitch error 0.667°. Driving has 11.9 mm/s plateau speed error and **87.9 mm stopping travel**, below the required 100 mm; the 80 mm aspirational target remains unmet. The worst driving stop across the acceptance suite is 91.9 mm. Five sensor-feedback crouch cycles span **67.4 mm** in hip-to-axle height. No controlled case has forbidden ground contact or a simulator warning.

Half-timestep differences are 0.0554° peak pitch, 0.00960° final heading, 0.0473 mm height span, 0.00323 N·m joint peak torque, 0.0000246 N·m RMS torque and 0.0136 rad/s joint speed. All convergence gates pass. Fixed-height regression also passes its five cases and two half-timestep checks.

All **12 additional uncertainty cases** pass: drive and combined motion with 0.5×/1.5× body inertias, ±5 mm motor internal-COM shifts, and ±0.15° static output-angle biases. These are separate perturbations. Static bias tests are not a dynamic backlash model: the published 0.3° gear free play, reversal impacts and elastic transmission still need physical characterization.

CAD components and MuJoCo's reconstructed full tensors agree to approximately 4.5×10⁻¹⁴ m in COM and 2.4×10⁻¹⁵ kg·m² in inertia. All 220 CAD pose checks and fresh native/STEP reopening pass. Display triangulation can slightly refine a reported bounding box; the nominal envelope should be read as approximately 144.85 × 284 × 260.89 mm. The 60° crouched modeled-solid envelope is approximately 164.79 × 284 × 213 mm, and the 25° extended envelope is 124.94 × 284 × 286.14 mm. Opposite-leg moving solids remain separated laterally; the exact per-leg scan covers independent hip/knee limits.

The [mechanical design notes](../mechanical-design.md) record the preliminary BOM, manufacturing processes, screw patterns/lengths, flush wheel stator screws, thread depths, fit/retention work, service openings and cable provisions. The source is parametrically regenerated from JSON; these are Part features rather than an Assembly Workbench constraint tree or production drawing package.

## Load, structure and energy results

The [engineering report](../../../artifacts/milestone-3/engineering/review.json) aggregates controlled acceptance and uncertainty loads.

| Actuator role | Worst peak N·m | Worst RMS N·m | Rated / RMS | Peak speed rad/s |
| --- | ---: | ---: | ---: | ---: |
| Hip | 0.329 | 0.119 | 10.92 | 0.647 |
| Knee | 0.806 | 0.544 | 2.39 | 1.180 |
| Wheel | 0.406 | 0.116 | 11.22 | 15.685 |

 Rated-to-worst-RMS margins exceed the 1.5× target for every role; the knee is the limiting role. Peak/load-speed screens also pass. Manufacturer thermal test conditions remain unknown; these are digital load comparisons, not endurance certification.

The structural screen uses twice nominal per-wheel weight with another 15% mass margin, conservative assumed aluminium properties, chassis weight on the hip braces, and twice observed peak hip torque. It finds about 32.8 MPa fork crossbar stress and 0.121 mm deflection. The combined hip-brace screen stays below the assumed 80 MPa allowance. The bearing's published static rating greatly exceeds this vertical design case, but its rating does not qualify motor overhung moments, misalignment, impact/fatigue or all possible fault torques. Threads, inserts and axial retention remain production/hardware checks.

A provisional six-cell P28A pack provides 60.48 Wh by nominal voltage × capacity. Using 70% usable energy, assumed 18–30 W holding/driver/electronics consumption and 60–80% conversion efficiency gives roughly **1.36–2.35 hours** across the three recorded power scenarios. Those losses are unmeasured; adopt **one hour as a provisional planning target**, rather than claiming this endurance. Direct 6S/actuator compatibility, BMS, voltage limits and regeneration handling remain unresolved for M4.

The baseline six actuators alone cost an indicative $815.40 before shipping/tax. This choice prioritizes documented mechanical data and the torque-control interface. It is not a final budget approval; lighter/cheaper leg modules and separate wheel drives remain alternatives before procurement.

## Reproduction and delivery

- Articulated suite: `MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.moving_validate`.
- Fixed-height regression: `MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.validate`.
- Additional uncertainty: `MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.mechanical_study`.
- Engineering review and saved evidence checks: `.venv/bin/python -m bipe.mechanical_review` then `.venv/bin/python -m bipe.verify_m3`.
- CAD scan from the project directory: `/snap/bin/freecad.cmd cad/check_assembly.py`; fresh native check: `/snap/bin/freecad.cmd cad/check_saved_assembly.py`. The scan preserves the GUI-saved native assembly; use the bridge builder after geometry edits to regenerate that document.
- Tests: `.venv/bin/pytest -q` and `.venv/bin/ruff check bipe cad tools tests`.

The default experiment/acceptance paths now target M3 and preserve the historical M1/M2 outputs. Each measured run saves parameters, controller, sensors, model, poses, CSV and metrics; the final delivery checker verifies **40 configuration snapshots**, controller identity, nominal model equality, CAD mass hashes and output hashes. The 36 s NVIDIA-rendered video was inspected at the initial pose and mid-motion frames. The live NVIDIA viewer completed 37 s of scripted physics and closed cleanly. Twenty Python tests and Ruff pass.

The FreeCAD bridge works after the user's restart. Keep interactive bridge jobs short and run complete CAD sweeps headlessly. KiCad installation is the main software setup remaining when M4 starts; there is no current CAD/simulation software blocker. Detailed M4/M5 plans remain unwritten until each milestone starts.
