# Milestone 4 packaging and mass reconciliation

Implementation started 1 October 2026 after plan commit `769ef4f`. The historical stacked layout, M3 CAD and previous simulation traces remain preserved. This checkpoint separates an envelope layout, conditional simulation and hardware qualification.

## Layout and mass decisions

The carrier stays horizontal above the battery. The existing 66 × 60 mm power connectivity board moves into the rear YZ plane, facing forward. Its 1000 µF capacitor lies along Y on short insulated leads; capacitance, voltage rating and regenerative test domain remain unchanged. Five external LTO50 resistors sit on a 70 × 22 × 2 mm internal spreader above the carrier. Neither that mounting proposal nor the 0.15 mm connectivity PCB is released for power operation.

The custom pack provision changes from 75 × 64 × 44 mm to 70 × 64 × 48 mm and moves 6.5 mm forward. Six real P28A maximum cell envelopes fit in two rows of three, with separate 45 × 36 × 5 mm BMS and 14 × 10 × 5 mm fuse provisions. These two parts are unselected; a purchased protected pack has not been shown to meet those provisions. The external body and leg dimensions are retained.

Manufacturer drawings set the capacitor maximum diameter to 16.5 mm and length to 27 mm; LTO50 body envelopes include +0.3 mm tolerance. P28A 46 g and capacitor 6.9 g are **typical**, not guaranteed maximum masses. The saved ledger retains uncertainty allowances and makes those distinctions explicit.

The hardware audit uses documented counts: 48 mechanical screws, 44 washers, eight nuts and four inserts, plus five resistor mounting sets and eight board mounting screws/washers. Full solid head/shank/nut envelopes and declared stainless/brass density give **56.416 g** for all hardware. Threads and recesses are not subtracted; purchased masses remain unverified. CAN is a 1.4 m individual-conductor daisy-chain provision; motor power remains a 3 m individual-conductor star harness.

Electronics/support allocation is **79.5 g**, covering 74.116 g board/component/bank/spreader estimates and 5.346 g support volume, with 0.038 g remaining. Hardware/harness allocation is **111 g**, with about 0.873 g remaining after the explicit inventory, copper/insulation, mating plugs, straps, sleeve and logic-wire estimates. Mount hardware belongs to that ledger once; fitted bank/board hardware masses have their own representative spatial proxies. The 326 g pack retains an explicit 50 g protection/strip/insulation/termination budget, including about 5.262 g unresolved allowance. The full 80 g future camera provision stays.

Four 32 × 26 mm through-windows remove **17.971 g** from the 2 mm aluminum tray, leaving its 14 mm central rail, outer rim and original mounting lands. Native mating-contact and zero-volume interference checks pass. A stated-load beam screen evaluates the rail; fatigue, corner stress, pack-case support and production detail remain unqualified.

Total is **2.494762 kg**, with **5.238 g** below the 2.5 kg ceiling. The ledger reconciles estimates; it does not establish a hardware upper mass bound. The generic earlier fastener allowance was optimistic and was superseded before final acceptance. Seven COM/inertia tensors derive from declared CAD/component groups, with cable/fastener mass distributed onto the legs. Detailed fit envelopes and simplified mass proxies are identified separately in the source.


## Thermal finding

A 10.5 J assumed event raises the 8.316 g spreader bulk temperature by approximately 1.40 K before cooling. A periodic lumped model with both broad faces exposed to declared internal air rejects frequent events: at 45°C internal air, h=5 W/m²K and 12 s between events, the predicted periodic bulk peak is approximately 103°C. Some slower-event rows stay below 60°C, but neither internal-air temperature nor a path to ambient is established. Contact/junction lag is excluded; a passing row is not qualification.

The current bank therefore does **not** close the thermal gate behind the prior conditional 60°C resistor-case calculation. It needs a justified path to ambient and bounded braking duty, followed by updated fit, mass and electrical checks. No bus-voltage guarantee is inferred from this layout.

## Verification outputs

The first rear layout had five clearance failures in mount/loom space; its report is saved as `failed-rear-layout-report.json`. The revised arrangement has no reported unintended intersections or 2 mm assembly clearance failures. Conservative forward removal sweeps pass with the front/future camera removed, leads unplugged and the bank, carrier, pack and rear board removed in order. Final fastener attachment, finger access, tolerances and wire strain relief remain release details.

Native and STEP contain 143 valid single solids. Exact surface bounds agree; ordinary display bounds differed because of triangulation. The current user-saved M3 file differs from Git only in view/metadata/thumbnail records; its engineering data and saved bytes are preserved.

Final corrected-configuration verification passes:

| Check | Measured result | Scope |
| --- | --- | --- |
| C-controlled acceptance | 27/27 and half-timestep convergence pass | Free base, nv=12, six finite hinges, physical contact, zero equality constraints; unchanged gains |
| Convergence | Peak pitch difference 0.047°, heading 0.020°, joint peak torque 0.0250 Nm, joint RMS 0.000080 Nm | Nominal combined case at 0.5 ms versus 1 ms physics |
| Fixed-height regression | Five cases and two finer-timestep cases pass | Original ideal-feedback reduced test, new declared masses |
| Mechanical uncertainty | 12/12 pass with full C sensing/control | Separate inertia, motor COM and static angle bias surrogates; no dynamic backlash qualification |
| Sensor/kinematic/control oracle | 10,000 packets; maximum complete-control difference 2.307e−15 Nm | Packet-identical C/Python computation, not hardware timing |
| Shutdown experiments | Healthy case and four fault/inhibit cases pass | Fault cases may fall; no powered-off balance claim |
| Compiler/source checks | Host C tests, UBSan, Cortex-M7, Teensy diagnostic build, 22 Python tests and Ruff pass | No flashing; real motor commands remain inhibited |
| Tray beam screen | 39.824 MPa; 0.390 mm deflection | Entire non-motor chassis load, 2g and 15% margin; cantilever surrogate, not FEA/fatigue/pack-case validation |
| Actuator load screen | Knee rated-to-worst-RMS ratio 2.44 | Declared actuator assumptions and separate scenarios, not measured motor thermal performance |

The current behavioral power analysis is regenerated from the corrected traces. It retains the conditional 4 A / 100 ms battery-open test and assumed 60°C case limit; packaging does not prove that temperature. The digital delivery check verifies current snapshots/build fingerprints separately from the historical seal at `5fd998b`. It reports `milestone_complete=false` and `hardware_qualified=false` even when evidence checks pass.

Artifacts are under `artifacts/milestone-4/packaging/`. The planning checkpoint is `769ef4f`, first envelope implementation `d363641`, and corrected hardware/tray checkpoint `ab2a5e5`; final verification is recorded in Git history. An EGL replay of the recorded accepted combined motion accompanies the CAD view and measured traces.

## Remaining M4 work

Develop the external thermal path and current-rated power layout; qualify the exact AK40/AK48 interface; select the protected pack/BMS/fuse; finish operational CAN/startup/calibration/heartbeat integration. Hardware RUN remains inhibited. M5's detailed plan remains unwritten.
