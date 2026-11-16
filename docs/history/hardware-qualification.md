# Hardware qualification requirements

M4's digital completion does not authorize fabrication, an operational flash or
hardware actuation. These are concrete unverified assumptions carried by the
current prototype; this is not a detailed plan for a future milestone.

| Requirement | Evidence needed before the corresponding hardware gate opens |
| --- | --- |
| Exact six-actuator choice | Resolve the AK40 drawing/model versus AK48 guide/profile evidence. Obtain the exact variant's continuous/peak output torque, allowable supply/transients, capacitor/storage limits, torque/current scale, encoder/zero conventions, local PD mapping and fail-silent timeout. Replace the candidate if its limits cannot meet the circuit/loads. See [actuator questions](actuator-manufacturer-questions.md). |
| Pack/protection | Confirm the selected 6S assembly's dimensions, mass, fuse, BMS disconnect/recovery and charge acceptance. The pack is not relied upon as a regeneration sink. See [pack questions](pack-supplier-questions.md). |
| Startup | Total actuator/external capacitance ≤4 mF; disabled bus load ≤75 mA; monitor readings calibrated to ±0.25 V/channel. Validate ATtiny402 ADC/IO/open-drain fault, watchdog, brownout and default-low output. Stale/fault detection and physical cutoff must meet the ≤10 ms requirement, with a 500 ms startup timeout. Host tests are not an AVR hardware image or timing measurement. |
| Energy/stop | Establish the 1.5 J total auxiliary-storage bound, including phase magnetic energy: 4 mF at the analyzed peak already uses 1.456 J. Independently bound positive work after stop to 3.5 J and worst reaction power to 130 W, including motor lag and actual driver behavior. Prove meter errors ≤5% plus 0.05 J and stop/disable queue/PD semantics. External pushes/downhill driving are outside the current energy domain. |
| Voltage/current/SOA | Validate actual harness/TVS/commutation loops, capacitor bias and loaded gate behavior. Modeled turn-off peaks are above the nominal 28 V braking ceiling; compatibility must be established against the exact actuator transient rating. Confirm switch/trace/shunt temperatures, peak-latch/current limits, diode/FET SOA and fuse coordination. Native DRC and an IPC trace screen do not establish those bounds. |
| Roof/thermal sensing | Verify ≤0.1 K/W interfaces, minimum modeled convection/emissivity, ≤0.2 W other heat entering the roof and calibrated conservative case-temperature error including lag ≤1°C. The selected NTC's documented response does not itself prove that dynamic error: qualify an observer/sensor/mount or revise thresholds. Full modeled duty is ≤40°C ambient, ≥600 s between full events. Do not apply the retained failed 45°C/frequent-duty cases as an operating envelope. |
| PCB/build details | Confirm the 1 mm four-layer/2 oz outer stack with the outsourced fabricator. Complete return/decoupling/EMC review, insulation, connector mating, specified low-profile wire soldering, strain relief, tooling and retention drawings. The Pololu module uses a wired adapter; no vendor pin/drill pattern has been inferred. Verify actual bypass-loop impedance and ADC acquisition/reference/filtering. |
| Mass/manufacture | Weigh components and the assembled robot; obtain controlled pack/module heights and manufacturing tolerances. The estimate is 2.499519 kg with only 0.481 g margin. Retained allowances and typical masses do not form a hardware upper bound. Redesign or explicitly revise requirements if measured mass/fit fails. |
| Operational adapter | Implement the manufacturer-backed codec and assembled IMU/joint transform/zeros; verify discovery, calibration, explicit arm, stale feedback, transport/deadline faults, brownout, overtemperature, operator stop and manual reset. Keep `hardware_qualified=false` until the complete evidence is available. The delivered Teensy image remains diagnostic/listen-only and unflashed. |

Manufacturer download evidence was rechecked on 1 October 2026. CubeMars' public
page listed an AK48 guide whose download link was empty in the saved observation;
the available older manual is not treated as new exact-variant qualification.
Manufacturer questions are prepared locally and have not been sent.
