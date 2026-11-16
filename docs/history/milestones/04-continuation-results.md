# Milestone 4 continuation results

Recorded 1 October 2026. The [continuation plan](04-continuation-plan.md) was committed as `36fed37` before implementation. The full-control checkpoint is `c05224d`; subsequent power/fit verification checkpoints are recorded in Git history. M4 remains open. No parts were bought, fabricated, connected or flashed.

## Achieved work

1. **Exact-variant audit:** reviewed the AK40-10 V3 product, official repository and current software resources. A static extraction located AK40 V3 resource names and an unnamed gains record; neither establishes MIT ranges or current-to-output-torque conversion. The exact AK48 guide remains unavailable. [Qualification matrix](../actuator-qualification.md) and [manufacturer questions](../actuator-manufacturer-questions.md) are saved; questions were not sent and unknown fields remain null.
2. **Full portable C control:** estimator, stationary gyro calibration, encoder plausibility, COM/kinematics, height schedule, feedforward, local leg feedback, coordinate mapping, finite torque requests and supervisor are implemented. Host adapters supply sensor packets and plant dynamics; C performs the complete sensing/control computation. Real Teensy diagnostic ELF/HEX build with hardware commands inhibited.
3. **Conditional power candidate:** native KiCad schematic, connectivity PCB, self-contained libraries, BOM, manufacturer evidence, source/pad audit, ngspice input decks and fault/energy traces. Stop/watchdog, manual arm, supply supervision, protected logic branch, precharge, current protection and an independently bus-powered dump circuit are concrete. Current-rated copper, pack/BMS, motor ratings, loaded device behavior and thermal/transient qualification remain open. See [power-stage notes](../power-stage.md).
4. **Separate FreeCAD fit study:** component, mount, connector, insulation, capacitor, external resistor and heat-spreader envelopes are saved as native CAD and STEP. The tested stacked layout **fails** the present envelope and mass allocation. M3 CAD and `config/robot.json` mass/inertia remain unchanged.

## Measured evidence

| Check | Achieved result | Scope |
| --- | --- | --- |
| Full C/Python oracle | 10,000 packets; maximum estimator error 2.776e−17, COM 1.388e−17 m, complete control 2.297e−15 Nm; local rate error 0 | Packet-identical asymmetric/noisy dynamic cases; no hardware timing proof |
| C-controlled physical acceptance | All 27 cases and half-timestep convergence pass; free base, nv=12, six finite hinges, physical contact, zero equality constraints | Existing M3 masses/contact/actuator dynamics and M4 sensor assumptions |
| Host/target verification | Meaningful C tests, UBSan, generic Cortex-M7 and pinned Teensy build pass; 22 Python tests and Ruff pass | Compiled diagnostic image is not flashed; driver/local-loop/axes/zero transport qualification pending |
| Free-base C fault experiments | Healthy run passes; four faults inhibit then fall. CAN inhibit 3.016 s; IMU/stop 3.000 s; host loss 3.750 s for injection at 3 s | No claim of balance after power removal |
| Conditional regeneration | 4 A × 100 ms battery-open worst corner peaks at 26.821 V; MCU-power-loss case matches; analytic assumed-delay upper bound 26.984 V | 28 V project target, not actuator rating; behavioral switching and declared temperature/delay bounds |
| Energy / negative controls | Worst-corner dump disposes about 10.416 J; 6 A overload breaches 28 V; absent clamp breaches it | Test-domain limit is explicit; no universal fall/braking qualification |
| Precharge | At 0 / .05 / .10 A standby, readiness occurs at 63 / 78 / 122 ms; .15 A fails readiness and must remain off | Conditional six-actuator idle load; readiness ratio 89.72%, no forced time bypass |
| External watchdog | Capacitor-derived timeout 62.74 ms nominal, 56.43–69.12 ms bound; six logic streams pass | Logic abstraction; real disconnect latency and complete startup remain unmeasured |
| Electrical CAD | ERC 0; DRC 0; unconnected 0; parity 0; 193 declared pin mappings match native pads/netlist | 647 tracks/vias, two ground fills; declared 0.15 mm connectivity clearance; no power copper/current qualification |
| Physical fit | Boards + cap require 39.32 mm, allocation 10 mm; obstacles and native shapes recorded | This arrangement fails; not adopted as the robot assembly |
| Candidate electronics mass | Carrier 42 g + power 54.116 g = 96.116 g, allocation 90 g | Estimated, not weighed; proposed mass change not adopted |

Outputs: [full C comparison](../../../artifacts/milestone-4/continuation/firmware/comparison.json), [acceptance](../../../artifacts/milestone-4/continuation/acceptance/acceptance.json), [C fault report](../../../artifacts/milestone-4/continuation/physics-faults/report.json), [power report](../../../artifacts/milestone-4/continuation/power/analysis/report.json), [native electrical check](../../../artifacts/milestone-4/continuation/power/native-check.json), [fit report](../../../artifacts/milestone-4/continuation/fit/report.json), [fit CAD](../../../artifacts/milestone-4/continuation/fit/electronics-fit.FCStd), [fit image](../../../artifacts/milestone-4/continuation/fit/electronics-fit.png). Configurations and measured traces accompany experiments.

## Remaining decisions and gates

The next independent M4 work is **packaging revision and a complete mass budget**, followed by a physically appropriate power PCB/thermal arrangement. Evaluate a vertical power board and relocated/split capacitance while retaining the compact form factor; do not adopt the failed stack. Current estimate would leave only 1.65 g below the robot's 2.5 kg ceiling if adopted, and the unfinished pack/BMS/fuse/harness can consume that margin.

Exact AK40/AK48 manufacturer evidence is still needed before any live command mapping, local PD timing, bus limits or torque conversion can be qualified. Then complete the MCU's physical axes/zero calibration, CAN transmit/bus-off handling and external heartbeat integration. The tested full portable core is reusable; the hardware application remains diagnostic-only. The power and profile gates must remain explicit even if software checks pass.

All current FreeCAD MCP, CAD, simulation, compiler, KiCad, router and ngspice tools work. Public manufacturer endpoints have some missing/blocked links, which are recorded as evidence gaps. No additional MCP setup or user software installation is required for the next digital packaging work. M5 remains in the overview without a detailed plan.
