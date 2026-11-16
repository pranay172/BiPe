# M4 digital closure results

Completed 1 October 2026 against the [closure plan](04-closure-plan.md).
M4 is complete as a **conditional digital electronics and firmware prototype**.
The [delivery verifier](../../../artifacts/milestone-4/closure/verification.json)
keeps hardware qualification and fabrication release false. The original M4
criteria concern native electrical CAD, relevant circuit analysis, and simulated
sensing/communication/startup faults; physical commissioning is a separate gate.
Earlier failed candidates and sealed experiments remain historical evidence.

## Integrated design

The separate protection module has been consolidated into the existing
66 × 60 × 1 mm power-board allocation. The native power board uses four copper
layers to escape the TPS48111 pins: 70 µm outer copper, 35 µm inner copper,
0.77 mm declared FR4 and 0.02 mm total solder mask. Wide outer current paths
remain explicit; there are no filled power-board planes. The conservative bare
board mass screen is **10.121 g**, within the retained 11.5 g allocation. The
two-layer carrier's corresponding screen is **10.341 g**. Stack availability,
fabricated copper, solder, loop impedance and loaded thermal/SOA still need
supplier/physical qualification.

The integrated circuit adds LM74700 reverse blocking, AOD2910 switches, three
TVS locations and bus-powered TPS7A1650 SAFE5. The carrier uses the 50 V input
Pololu D36V28F5 #3782 and an independent 5 V supervisor. Its module footprint
represents a **wired adapter**, not inferred vendor pin or mounting coordinates.
The 180 kΩ/10 kΩ divider reaches at most **2.334 V** under the declared 43.512 V
transient and resistor tolerance envelope. Its resistor-only uncalibrated error
at 25.2 V can reach **0.482 V**; the ±0.25 V monitor requirement therefore needs
calibration and ADC-reference/gain verification. INA240A1, dedicated shunt
pick-offs, dump switching capture, roof thermistors and a matched six-wire
measurement interface are explicit in both native boards.

The ATtiny402 algorithm is independent of the main application. `START_OK` is
energy/time-supervised **precharge permission**, asserted only after a cold
manual attempt; it is not bus readiness. The TPS48111 main input additionally
requires the independent analog `BUS_READY`. Stale data, energy, timeout and
fault drop permission; the real open-drain fault, ADC, IO and watchdog adapter
remain uncommissioned. Local 100 nF bypass capacitors for the new logic gates
are routed explicitly, at 6.39 mm and 2.25 mm centre distances; these distances
do not establish EMC or switching performance.

The exterior aluminium roof replaces the former internal spreader and the
corresponding polymer roof volume. The capacitor moves upward to clear the
taller switch packages, adding 60 mm of individual 18 AWG lead length and
**0.581 g** of modeled wire. Power J4–J6/J9/J10 use specified low-profile
soldered 24 AWG terminations, limited to 2 mm height. Removal disconnects the
looms before lifting the roof, opening the front and removing the carrier,
pack and power assembly. Tool access, lead bend radii, insulation, strain relief
and detailed mounting remain release-drawing work.

## Measured verification

| Check | Final result |
| --- | --- |
| Native electronics | Both boards: zero ERC, DRC, unconnected and parity violations; full native pin/interface audit passes |
| Native conductor screen | Declared heavy paths present and finite-current screens pass; no fabricated temperature claim |
| CAD | Envelope/pack clearances, conservative removal sweeps, native reopen and STEP volume/bounds round-trip pass |
| Mass/inertia | Seven positive rigid-body tensors reconcile with the nonoverlapping ledger; **2.499519 kg**, **0.481 g** below the 2.5 kg estimate ceiling |
| Full C locomotion | **27/27** cases and half-timestep convergence pass; free base, nv=12, six finite actuators, physical contact, zero equality constraints |
| Regression/uncertainty | Fixed-height suite and both convergence checks pass; **12/12** separate mechanical uncertainty cases pass |
| C equivalence | 10,000 identical sensor/control packet cases pass; maximum torque difference from Python is 2.70 × 10⁻¹⁵ N m |
| Physics shutdown | Five scenarios pass their stated criteria; power-removal and host-loss cases can fall after inhibition, as recorded |
| Portable runtime | Startup/calibration, 200 Hz command batches, bounded stop, queue/PD disable, reset debt and eight injected faults pass; production rejects synthetic/incomplete profiles |
| Independent startup | Short, readiness, residual heating after readiness, stale/invalid sensing, timeout, retry and reboot cooldown checks pass |
| Compilation | Host/UBSan, 22 Python tests, Ruff, Cortex-M7 policy object and locked Teensy ELF/HEX pass; no hardware flash or actuator commands |
| Historical identity | Prior 1449-file seal remains recoverable and verified; user-saved M3 assembly is preserved |

The mass figure retains camera, protected-pack and harness provisions. It is an
estimate built from native volumes, typical part masses and declared allowances,
**not a guaranteed hardware maximum**. Its remaining margin is too small to
treat as manufacturing contingency. The candidate was tested before adoption;
`config/robot.json` now contains its derived SI parameters. A preliminary pass
before the final bypass/source update is retained separately.

## Energy and thermal operating domain

The inventory covers **33 saved trajectories**, including the unpowered fall,
all nominal/variation/convergence cases and five physics shutdown cases. It uses
MuJoCo kinetic energy including armature plus all positive gravitational
potential relative to ground, with no 0.1 m fall shortcut or speed exclusion.
Maximum sampled mechanical energy is **4.21784 J**. Adding the declared 0.75 J
inter-sample/model allowance, 1.5 J auxiliary storage and 3.5 J future positive
driver work gives **9.96784 J**, within the **10.5 J** stop reservation.
Friction/contact dissipation is not credited. External pushing, downhill motion,
unknown driver storage and unbounded power remain outside this qualified domain.

The policy reserves 15 J per dump cycle and 4 J per startup, requests stopping at
2 J normal dump expenditure or 0.5 J positive stopping work, and faults at a
550 ms policy timeout around the application's 500 ms stop. Its 130 W positive
power and ≤10 ms reaction bounds are requirements: the modeled static torque-speed
screen is 127.55 W, while real driver/transient work still needs measurement.
Total capacitance must stay ≤4 mF and disabled startup load ≤75 mA. The 20
analytic startup corners preserve out-of-domain failures; all in-domain corners
pass readiness/energy. A short plus the reaction allowance reaches 3.524 J.

Full modeled event duty is **≤40°C ambient and ≥600 s between cycles**. The
three-node thermal model gives dump/precharge cases **55.50/55.18°C** and elements
**142.60/131.14°C**. The 45°C and frequent-duty failures remain saved. Contact
resistance ≤0.1 K/W, calibrated case-temperature error including lag ≤1°C,
minimum convection/emissivity and **other heat reaching the roof ≤0.2 W** are
unqualified requirements, not measured properties of the enclosed electronics.
Firmware uses temperature admission and full event reservations without crediting
imaginary cooling; reset imposes the full ten-minute cooldown.

Independent ngspice thermal endpoints agree within 4.35 × 10⁻⁹ K and heat balance
within 1.08 × 10⁻⁵ J. Finite 0.5/2 µH harness turn-off models peak at
40.71/43.15 V. These use explicitly assumed local parasitics and a behavioral TVS
envelope; the actual PCB loops and exact actuator transient rating are unqualified.

## Deliverables and next milestone

Sources and measured outputs are in
[`artifacts/milestone-4/closure`](../../../artifacts/milestone-4/closure), with the
[native power project](../../../electronics/power-integrated/bipe-power.kicad_pro),
[carrier project](../../../electronics/carrier-integrated/bipe-controller.kicad_pro),
[assembly](../../../artifacts/milestone-4/closure/cad/integrated-robot.FCStd),
[STEP](../../../artifacts/milestone-4/closure/cad/integrated-robot.step),
[acceptance](../../../artifacts/milestone-4/closure/acceptance/acceptance.json),
[energy inventory](../../../artifacts/milestone-4/closure/energy/report.json), and
[recorded simulation](../../../artifacts/milestone-4/closure/acceptance/sensors_combined/simulation.mp4).
The portable application/runtime/startup code, source documentation, circuit decks,
traces, hashes and diagnostic target image accompany them.

M5 is the digital design delivery milestone: manufacturing drawings, detailed
retention/wiring and assembly notes, consolidated BOM and a reproducible release
bundle. Its detailed plan has not been created. The
[hardware qualification requirements](../hardware-qualification.md) remain the
entry conditions for procurement, flashing an operational image or fabrication
release. No Blender work is required to close M4.
