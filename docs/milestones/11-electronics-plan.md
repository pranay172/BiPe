# M11: M10 electronics

Plan written 18 October 2026, after the user chose the M10 electronics as
the next milestone. It finishes the agreed initial scope, a
hardware-oriented digital prototype that balances, drives, turns and
crouches. Camera following and jumping stay later work.

M11 delivers a digital electrical design: parts selected against
computed loads from manufacturer documentation, packaged in the M10 CAD,
with the firmware hardware layer tested in simulation. It is not a
hardware-validated design. Nothing is bought, built or powered.

## Starting point

M10 has only an electrical concept and envelopes:
- six moteus-c1 drivers (four RO40 hip/knee drives, two GL40 hubs) on
  CAN-FD, as 38 × 38 × 9 mm envelopes;
- a Tattu R-Line 850 mAh 6S pack (165 g upper tolerance);
- a "host MCU, IMU and power board" envelope, 40 × 10 × 30 mm, with an
  assumed LSM6DSO IMU and 15 g;
- an AS5047P hub encoder on the fixed axle tube, one Camera Module 3
  envelope in the head (the two front body "cameras" are styling lenses
  with no board) and head LED boards;
- the hollow-shaft harness route through the hips and knees, 60 g of
  harness allowance;
- an electrical screen in `bipe/robustness.py` (peak and RMS phase
  currents, copper loss, thermal path bounds, runtime about 51 min).

The firmware (`firmware/`, Teensy 4.0 target, `BIPE_SERIAL`) runs the
balance controller and is parity-tested against Python. Its hardware
layer, `config/electronics.json`, is the M4 candidate (Teensy 4.0,
LSM6DSO, a classic-CAN TCAN330 at 1 Mbit/s, a 5 V regulator, precharge
and dump circuits). The M4 KiCad boards are in the tag
`archive/m10-pre-cleanup`; they were designed for other actuators.

Budget: the CAD allocation is 1.96 kg against the 2.15 kg cap (190 g
headroom). Electronics carry 82 g and the harness 60 g.

## Selection rules

These apply to every part choice in M11:
- Meet the computed loads (phase A) with margin, using manufacturer
  documentation for every rating.
- Prefer popular, budget-friendly and easy-to-obtain parts (user,
  18 October 2026): widely stocked at common distributors (DigiKey,
  Mouser, LCSC; Amazon or AliExpress for modules), with a large user
  community and good documentation.
- Prefer off-the-shelf modules and breakouts over custom boards where
  they fit the envelope. A custom board only carries what no module can.
- Each selection records unit price, sources and stock as checked on
  the day, alongside the engineering checks.
- Parts that change the mechanical design or the plant (motors, pack
  size, driver envelopes) are trade-offs the user signs off.

## Phases

| Phase | Content | Exit |
|---|---|---|
| A. Electrical requirements | Time-resolved bus and phase loads from the M10 simulation | Requirements table, traces and configs saved |
| B. Architecture and parts | Block diagram; drivers, host, IMU, power path, protection, connectors, wire gauges, from the rules above; cost and availability review of the existing M10 parts | User signs off the part list and any trade-offs |
| C. Power and protection | Power path, switch/precharge, fuse, regulators, voltage monitoring, regen handling, a fault table | Calculations pass against phase A |
| D. Harness and packaging | Real part envelopes and connectors in the CAD; wire counts and bundle sizes through the hollow shafts; bend and twist at the joints; mass update; sweep | Clean fit and sweep, mass within the cap |
| E. Firmware hardware layer | Driver protocol over CAN-FD, IMU driver, bus timing and latency budget, supervisor and power states; a Teensy build | Parity tests and a timing budget that meets the C6 latency assumption |
| F. Exit | Acceptance re-run if the plant or controller changed; results; README status | Acceptance unchanged or explained |

Detailed plans for phases B–F are written when each starts.

## Decisions for the user

These are raised when the phase that needs them starts:
- Phase B: keep moteus-c1, or move to a cheaper, more common driver if
  one meets the loads and fits. A different driver changes the
  envelopes, the CAN protocol and the plant's FOC assumptions.
- Phase B: whether the existing niche parts (RO40, GL40, Tattu pack)
  should be swapped for more common equivalents. Motor swaps change the
  plant and need the fall and acceptance gates again.
- Phase B: vision compute. The head's Camera Module 3 needs a host with
  a camera input. Camera following is later work, but the space, power
  and mass for its computer could be reserved now.
- Phase D: board depth. Off-the-shelf modules wired on a carrier, or a
  custom routed board with fabrication outputs.

## Phase A plan: electrical requirements

Goal: the loads that phase B selects parts against, from the M10 plant
in its accepted configuration.

1. Instrument the M10 plant (`bipe/m10/plant.py`) to record, every
   physics step:
   - per-motor shaft power (motor torque × rotor speed) and copper loss
     (0.75 × R line-line × I², the screen's peak-phase convention);
   - per-motor phase voltage demand, |Ke ω| + R I, to compare against
     the drive limit;
   - bus power: motoring power ÷ inverter efficiency, regenerating power
     × efficiency, plus auxiliaries; bus current at the pack voltage;
   - downsampled traces at `log_dt`, plus peaks and integrals at the
     physics step.
   Recording must not change the dynamics. Check that a case's metrics
   are bit-identical with and without it.
2. Run a representative case set with the m10 profile, at most 8
   workers, under `nice`:
   - quiet hold and the crouch cycle (30 to 60 command);
   - straight drive at maximum speed, acceleration and braking;
   - turning at maximum yaw rate;
   - slope up, down and hold at 10°;
   - the push recoveries at their limits;
   - the powered fall cases from `bipe.m10.falls`.
3. Derive the requirements:
   - per driver: peak and RMS phase current, peak phase voltage, and
     the share of time spent current- or voltage-limited;
   - bus: peak, RMS and mean current; peak regenerated power and
     energy; the worst 10 ms, 100 ms and 1 s current windows (for
     fuse and wire sizing);
   - pack: discharge C-rate peak and mean, runtime from usable energy
     (80% depth of discharge), sag at an assumed internal resistance
     (labelled as an assumption until a datasheet gives it);
   - logic rails: a 5 V and 3.3 V load estimate from catalog figures
     (host, IMU, CAN transceiver, LEDs), with the cameras and vision
     compute listed separately as a reservation.
4. Save `artifacts/m11/requirements/`: the run config, the case list,
   traces (`.npz`), `requirements.json` and plots. Add the results to
   this document.

Exit: a requirements table that names each number's source case and
basis.

## Phase A results (18 October 2026)

`bipe/m11/requirements.py` runs 17 cases exactly as the acceptance does (C
controller, sensor models) with an observer that only reads the plant.
With and without it, `sensors_turn` gives bit-identical metrics and traces.
All 17 cases are physical passes, and none failed. The slope cases are
"not recovered" by design, as in the committed acceptance.

Outputs are in `artifacts/m11/requirements/`:
- `config.json`: the electrical model, its sources and the case
  definitions;
- `cases.json`: per-case results;
- `requirements.json`: the worst value per quantity, with the case that
  set it;
- `traces/`: per-motor channels at 5 ms and bus power at 1 ms. Peaks and
  current windows are computed at the 0.25 ms physics step.

Scenarios:
- **draw**: inverter efficiency 0.85, copper at 125 °C. Bus current and
  runtime come from this one.
- **regen**: inverter efficiency 0.95, copper at 20 °C. This is the
  conservative case for regeneration.

Auxiliaries are the 6 W screen assumption.

**Per driver** (worst over cases):

| | RO40 hip | RO40 knee | GL40 hub |
|---|---|---|---|
| Phase current peak | 3.41 A (`push_drive_-2.5`) | 2.54 A | 2.81 A (`push_hold_-2.5`) |
| Phase current RMS | 1.88 A (`push_low_+2.5`) | 1.80 A | 0.44 A (`slope_up_10`) |
| Driver DC power peak | 24 W | 14 W | 51 W (`push_drive_-2.5`) |
| Driver regeneration peak | 0 | 0 | 5.7 W (`push_drive_+2.5`) |
| Phase voltage demand peak | 8.1 V | 6.3 V | 16.0 V cold; 20.7 V hot |
| Time at the current limit | 0 | 0 | 0 |

**Bus** (at the 19.2 V low pack voltage; nominal 22.2 V in brackets):

| Quantity | Value | Case |
|---|---|---|
| Peak current | 6.63 A (5.74 A) | `push_hold_+2.5` |
| Worst 10 ms / 100 ms / 1 s mean | 6.43 / 3.72 / 2.82 A | pushes |
| RMS current | 1.69 A (1.46 A) | `push_low_-2.5` |
| Peak power | 127 W | `push_hold_+2.5` |
| Minimum power | +3.9 W: the bus never regenerates | `push_hold_-2.5` |
| Pack C-rate peak | 7.8 C | `push_hold_+2.5` |

**Runtime** from 80% of the pack energy:
- draw scenario: 28–39 min (`push_low_-2.5` to `sensors_hold`);
- regen scenario: 40–54 min.

The quiet hold draws 23 W in the draw scenario. Most of it is copper loss
in the four leg motors holding the body up (about 15 W hot); the hubs use
under 1 W.

Findings for the later phases:
- The loads are small for the planned parts. Against the moteus-c1
  ratings (20 A peak, 5 A continuous without cooling, 250 W at 28 V), the
  worst driver uses 17% of the peak current, 38% of the continuous
  current and 20% of the power. The pack's peak draw is 7.8 C, so any
  common 6S 850 mAh pack rated 45 C or more covers it. Phase B can
  therefore weigh cheaper, more common parts.
- The bus never regenerates, because the drivers' regeneration is
  absorbed by the other drivers and the auxiliaries. The supply is still
  not proven safe against regeneration: a back-driven robot with its
  drives disabled, or a switch opening during braking, is a phase C case.
- The plant's GL40 voltage limit uses 20 °C winding resistance. With
  125 °C copper the same currents need up to 20.7 V, against the 16 V
  drive limit. Hot hubs would therefore be voltage-limited for up to 1%
  of the hardest pushes, so the push margins are slightly optimistic
  when the windings are hot. This is recorded as an open item: changing
  the plant is a physical model change and needs the acceptance gates.
- The plan's "powered falls" are not in the set, because the M10 falls
  are unpowered (`bipe/m10/falls.py`).
- The logic-rail estimate moves to phase C, where the parts are known;
  6 W stands until then.

Exit met: the requirements table above, with each number's case and
basis, the configs and the traces.

## Phase B plan: architecture and parts

Goal: a part list that meets the phase A loads with margin, ranked by
the selection rules (popular, budget-friendly, easy to obtain), for the
user to sign off.

1. Fix the hard requirements from phase A and the M10 design:
   - drivers: a full 6S bus (25.2 V, with transients) plus margin;
     at least 3.5 A peak and 2 A continuous phase current; an SPI
     encoder input for the AS5047P; a host bus; the 38 × 38 × 9 mm
     envelope or a CAD change;
   - host: the 200 Hz control loop with the current firmware (Cortex-M
     with an FPU), the driver bus, SPI for the IMU, and enough pins for
     the supervisor I/O;
   - IMU: the LSM6DSO assumed by the plant's sensor model, or an
     equivalent with the same noise and rate;
   - the pack: 6S 850 mAh at 45 C or more, in the 65 × 32 × 47 mm
     envelope;
   - logic power: 5 V and 3.3 V for the host, IMU, LEDs and a camera
     reservation.
2. For each slot, compare two to four candidates on the engineering
   checks (manufacturer documents), unit price, sources and stock on the
   day, community use and documentation. Record every source.
3. Review the existing M10 parts the same way: RO40, GL40, Tattu pack,
   AS5047P boards, Camera Module 3. Report only; any swap is the user's
   call.
4. Draw the block diagram: power path, driver bus, host I/O and sensors.
5. Present the recommended list, the alternatives and the decisions to
   the user. Phase C starts after sign-off.

Exit: a costed part list and block diagram, with the user's decisions
recorded.

## Phase B findings (draft for the user's sign-off, 18 October 2026)

Prices and stock are as found on 18 October 2026, for single units in
USD. Retail pages change, so recheck them before buying.

**Block diagram.**

```
6S pack (XT30) -> main switch -> fuse -> bus distribution --+-- 6 x moteus-c1 -- 4 RO40 + 2 GL40
                                                           |        (each: AS5047P on aux SPI)
                                                           +-- 5 V regulator -- Teensy 4.0, LEDs
Teensy 4.0 (carrier) -- CAN3 (FD) -- transceiver -- CAN-FD bus (JST PH-3 daisy chain) -- 6 x moteus-c1
                     -- SPI -- LSM6DSO IMU
                     -- ADC -- bus voltage divider
                     -- GPIO -- arm button, stop, LED data
```

**Recommended parts.**

| Slot | Part | Why it fits | Price, stock | Alternatives checked |
|---|---|---|---|---|
| Joint and hub drivers (6) | mjbots moteus-c1 | 10–51 V; 20 A peak, 5 A continuous (no cooling); 250 W; CAN-FD 5 Mbps; AS5047P on the aux port; 38 × 38 × 9 mm, 8.9 g. Phase A peaks: 3.4 A, 1.9 A RMS, 51 W | $69, 1114 in stock (mjbots) | ODrive Micro: $89, sold out, CAN 2.0 only, 30 V. MKS XDrive Mini: about $35 but 28 V absolute maximum (too close to 25.2 V), dual axis, too large. B-G431B-ESC1: 11.1–22.2 V. SimpleFOC Mini: 8–24 V, 1.5 A continuous |
| Bench CAN-FD adapter (1) | mjbots mjcanfd-usb-1x | Needed once to calibrate each moteus; also a gs_usb socketcan device | $39, in stock | fdcanusb: deprecated, sold out |
| Host | PJRC Teensy 4.0 | The current firmware target; Cortex-M7 600 MHz; CAN3 is FD-capable; about 100 mA at 600 MHz; 35.6 × 17.8 mm fits the 40 × 30 face | $23.80, in stock (SparkFun) | Teensy 4.1: 61 mm long, does not fit. STM32G431 Nucleo-32: 50 mm long, needs a firmware port |
| CAN-FD transceiver | TI TCAN330G (on the carrier) or Adafruit CAN Pal (NXP TJA1051T/3) | Both are rated for 5 Mbps CAN-FD at 3.3 V | CAN Pal $3.95, in stock | The M4 TCAN330 without G is limited to 1 Mbps, so it is unusable with moteus |
| IMU | ST LSM6DSO (chip on the carrier) or Adafruit LSM6DSOX breakout | The plant's sensor model assumes LSM6DSO | Breakout $11.95, in stock; 25.6 × 17.8 × 4.6 mm | — |
| 5 V regulator | Pololu D36V28F5 | 5.3–50 V in, 3.2 A, 17.8 × 20.3 × 8.8 mm | $23.95, "rationed" | Cheaper modules are compared in phase C, once the 5 V load is known |
| Pack | Tattu R-Line v5 850 mAh 6S (current) or GNB 850 mAh 6S 80C | Peak draw 7.8 C; both fit 65 × 32 × 47 mm (GNB 65 × 30 × 45, 150 g) | Tattu $30.99, GNB $32.49 | — |
| Head eye LEDs (2) | Adafruit NeoPixel Jewel (7 × 5050) | 23 mm fits the 24 mm LED board slot; 5 V, about 18 mA per LED | $5.95, in stock | — |
| Motor encoders (6) | ams AS5047P (current design) | moteus aux SPI mode 2 | About $3.5 (LCSC) to $11.7 (DigiKey), about 9.7k stocked; marked "not recommended for new designs" | MPS MA732 (moteus mode 4, active part, $4.32, low stock at 40–100) is the fallback |

**Existing M10 parts, cost and availability review** (report only):

| Part | Price, stock | Note |
|---|---|---|
| CubeMars RO40 Lite KV140 (4) | $83.38 at RobotShop, out of stock; €52.95 at OpenELAB, sold out | The main availability risk, at about $330 for four |
| CubeMars GL40 KV70 (2) | $84.85 without encoder (on demand) to $128.99 with encoder (in stock) at RobotShop | The design uses its own AS5047P, so the no-encoder version is enough |
| Raspberry Pi Camera Module 3 (1, head) | From $25 | Needs a Raspberry Pi host with camera inputs (the vision decision) |

Estimated cost of one robot's electronics:
- drivers, host, transceiver, IMU, regulator, pack, LEDs and encoders:
  about $560;
- motors at the prices above: about $500 more;
- camera: $25.
This leaves out the bench adapter, connectors, wire, PCBs and printed
parts.

Sources:
- [moteus-c1](https://mjbots.com/products/moteus-c1),
  [mjcanfd-usb-1x](https://mjbots.com/products/mjcanfd-usb-1x),
  [fdcanusb](https://mjbots.com/products/fdcanusb),
  [moteus aux encoder modes](https://github.com/mjbots/moteus/blob/main/docs/reference/configuration.md);
- [ODrive Micro](https://shop.odriverobotics.com/products/odrive-micro),
  [MKS XDrive Mini guide](https://www.hackster.io/justlovescience/mks-xdrive-mini-guide-set-up-tuning-arduino-3e23eb),
  [B-G431B-ESC1 (DigiKey)](https://www.digikey.it/reference-designs/en/motor-control/2840),
  [SimpleFOC Mini](https://GitHub.com/simplefoc/SimpleFOCMini);
- [Teensy 4.0 (PJRC)](https://www.pjrc.com/store/teensy40.html),
  [Teensy 4.0 (SparkFun)](https://www.sparkfun.com/teensy-4-0.html),
  [Teensy 4 pin map (Zephyr)](https://docs.zephyrproject.org/latest/boards/pjrc/teensy4/doc/index.html);
- [TCAN330 (TI)](https://ti.com/product/TCAN330),
  [Adafruit CAN Pal](https://www.adafruit.com/product/5708),
  [Adafruit LSM6DSOX](https://www.adafruit.com/product/4438),
  [Pololu D36V28F5](https://www.pololu.com/product/3782);
- [GNB 6S 850](https://www.racedayquads.com/products/gaoneng-gnb-22-2v-6s-850mah-80c-lipo-micro-battery-square-type-xt30),
  [Tattu R-Line 850 6S (Pyrodrone list)](https://pyrodrone.com/collections/6s-batteries/capacity_500-1000-mah);
- [NeoPixel Jewel](https://www.adafruit.com/product/2226),
  [AS5047P (LCSC)](https://lcsc.com/product-detail/image/AS5047P-ATSM_C962063.html),
  [MA732 (MPS)](https://www.monolithicpower.com/en/products/ma732.html);
- [RO40 Lite (RobotShop)](https://www.robotshop.com/products/cubemars-ro40-kv140-lite-motor),
  [RO40 Lite (OpenELAB)](https://openelab.io/a/s/products/cubemars-ro40-lite-frameless-motor),
  [GL40 (RobotShop)](https://www.robotshop.com/products/cubemars-gl40-kv70-motor-w-o-encoder),
  [Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/).

**User decisions (18 October 2026).**
- Drivers: keep six moteus-c1 on the 6S pack.
- RO40 supply: keep the RO40 Lite and record the stock risk; check
  CubeMars direct at build time.
- Vision compute: reserve a Raspberry Pi Zero 2 W slot (one CSI input
  for the head camera) in space, mass and 5 V power. Nothing is wired to
  the controller.
- Host board: a small custom 2-layer carrier PCB that the Teensy 4.0
  solders onto. It carries the CAN-FD transceiver, the LSM6DSO, the
  bus-voltage divider and the connectors. The 5 V regulator stays a
  module. KiCad schematic, layout and fabrication files.

Phase B exit met: the part list above, the block diagram and these
decisions. Encoders stay AS5047P, with the MA732 as the fallback.

## Phase C plan: power and protection

Goal: a power path and protection scheme, at schematic-block level, that
meets the phase A loads and the manufacturer's rules. Calculations go in
`bipe/m11/power.py`, which reads a new `m10.electronics` block in
`config/robot.json`. Results go in `artifacts/m11/power/report.json`.
The PCB layout and CAD fit are phase D.

Manufacturer rules that bind the design (mjbots electrical setup guide):
- The moteus has no reverse-polarity protection.
- XT30s must never be hot-plugged. The bulk capacitance is twelve 4.7 µF
  100 V ceramics per c1 (`hw/c1/bulk_capacitance.kicad_sch`), about
  340 µF nominal for six, so the bus needs a precharge or soft start.
- More than three controllers use T-spliced power, not daisy-chained
  power.
- CAN-FD at 5 Mbps needs twisted pair and termination at both ends
  (split 2 × 60 Ω with 4.7 nF suggested).
- Set `servo.max_voltage` at least 5 V above the maximum supply
  voltage. Use `servo.max_regen_power_W` and flux braking where the bus
  cannot absorb regenerated energy.

Steps:
1. Logic-rail budget from catalog figures:
   - the Teensy 4.0 (about 100 mA at 600 MHz);
   - the LSM6DSO, the CAN-FD transceiver and the divider;
   - two NeoPixel Jewels (14 LEDs at about 18 mA per colour channel),
     with a firmware brightness cap;
   - the Pi Zero 2 W and camera reservation;
   - the encoders, which run from each c1's 3.3 V aux (50 mA).
   The moteus logic draw is not published. It is an assumption, to be
   measured on the bench. The result replaces the 6 W auxiliary
   assumption and reruns the runtime figure.
2. 5 V regulator: compare the Pololu D36V28F5 with cheaper common modules
   on input rating (bus plus flux-braking margin), current, efficiency
   and size.
3. Power path:
   - XT30 pack lead;
   - a soft-start high-side MOSFET with gate-slew control, switched by
     a small shell switch that carries only gate current;
   - a fuse;
   - a T-splice to the six c1s and the regulator.
   Size the inrush current, turn-on time, MOSFET dissipation and
   safe-operating area, the fuse (phase A windows: 10 ms 6.4 A, 1 s
   2.8 A, RMS 1.7 A), and the wire gauges and voltage drop.
4. Regeneration and overvoltage:
   - the bus rise if the pack disconnects while moving (robot kinetic
     energy into the bus capacitance);
   - back-driven motors with the drives off (back-EMF against the
     bus);
   - the moteus settings: `max_voltage`, flux braking and the regen
     power limit.
5. Battery monitoring: divider ratio, ADC resolution and filtering, and
   arm and cutoff thresholds per cell. The moteus bus-voltage readings
   serve as a cross-check.
6. A fault table: cause, detection, response, and what each part does.

Exit: every calculation passes against phase A and the rules above, with
the assumptions labelled.

## Phase C results (18 October 2026)

`bipe/m11/power.py` reads `m10.electronics` (new in `config/robot.json`)
and the phase A outputs, and writes `artifacts/m11/power/report.json`.
All seven checks pass: regulator, inrush, fuse at peak, fuse at RMS,
fuse interrupting rating, driver minimum voltage and battery-monitor
range.

**Power path.**

```
6S pack -- XT30 -- 10 A MINI fuse -- N-MOSFET (LM5060 high-side controller) -- bus T-splice
              |                                 |                               +-- 6 x moteus-c1 (XT30)
              +-- shell switch -> UVLO/EN divider                               +-- Matek BEC12S-PRO 5.2 V
                                                                                +-- divider -> Teensy ADC
```

**5 V rail.**
- Typical load is 0.95 A: Teensy 0.10 A, 3.3 V loads 0.06 A, eye LEDs
  at the 25% firmware cap 0.19 A, and the Pi Zero 2 W with camera
  0.60 A.
- Worst-case peak is 2.92 A, with the LEDs at full white and the Pi at
  its recommended 2 A supply.
- The BEC12S-PRO (5 A, 9–55 V in, 5 g) runs at 58% of its rating at that
  peak. It replaces the phase B Pololu D36V28F5, which would sit at 91%.
- Auxiliary bus power is 8.2 W typical, or 4.5 W without the vision
  host. This includes an assumed 0.4 W of logic per moteus.

**Runtime.** Draw scenario, with the 6 W screen assumption replaced by
this budget:
- 26–36 min with the vision host running;
- 42 min for a quiet hold without it.

**Soft start.**
- The LM5060 gate current (17/24/31 µA) charges a 22 nF gate-to-ground
  capacitor. The bus rises in 18–33 ms, so inrush into the 338 µF bus is
  0.26–0.48 A.
- The MOSFET absorbs 0.16–0.21 J (capacitor charge plus the logic loads
  that start during the ramp).
- Turn-off takes about 0.4 ms. A VDS fault turns the MOSFET off in 5 µs
  at 80 mA.
- The MOSFET itself is chosen in phase D from stocked 40 V N-channel
  parts, against its single-pulse safe operating area.

**Switching and undervoltage.**
- The shell switch connects the UVLO/EN divider to the pack: 53.6 kΩ and
  4.7 kΩ, including the 5.5 µA pull-down.
- The bus switches on at 20.0 V and off at 17.8 V (2.96 V/cell). That is
  the hardware backstop below the 18.0 V firmware cutoff.
- When off, the drain is the LM5060's disabled current (under 15 µA).
  The battery monitor sits on the switched bus.

**Fuse and wiring.**
- The fuse is a Littelfuse MINI 297 10 A: 32 V DC, 1000 A interrupting.
  It carries the 6.63 A peak and the 1.69 A RMS (under 75% derating).
- The estimated short-circuit current is 367 A, with an assumed
  8 mΩ/cell pack. That is within the MINI fuse's rating but close to the
  400 A of the Nano2 0451 that was considered first.
- At peak current the pack, a 20 AWG trunk and 22 AWG branches sag
  0.50 V. A driver therefore still sees 18.7 V at the low pack voltage,
  against its 10 V minimum.

**Regeneration and overvoltage.**
- The pack absorbs regenerated energy whenever it is plugged in. That
  holds even with the switch off, because the N-MOSFET's body diode
  conducts from the bus back to the pack.
- If the pack is unplugged, or the fuse blows, while the robot moves at
  0.3 m/s, braking energy alone would lift the derated 169 µF bus to
  41 V. The moteus settings (`max_voltage` 30 V, flux braking above
  27 V, regeneration limited to 10 W per driver) must absorb that.
- A back-driven hub cannot charge the bus below 4.6 m/s of ground speed.

**Battery monitor.** The 180 k/10 k divider gives 1.33 V at a full pack
and 15 mV per ADC step at the bus.

**Faults.** The report has the fault table: short circuit, inrush, over-
discharge, regenerative overvoltage, reverse polarity, CAN loss, IMU
loss, overtemperature and 5 V overload.

**Assumptions still to measure on hardware:**
- moteus logic power (0.4 W each);
- transceiver supply current (60 mA allowance);
- Camera Module 3 current (0.25 A);
- pack internal resistance (8 mΩ/cell);
- MLCC DC-bias retention (0.5);
- regulator efficiency (0.85).

Exit met.

## Phase D plan: packaging, harness and boards

Goal: the real electronics in the M10 CAD, with a clean fit and sweep
and the mass within the cap. Also the harness through the joints, and
the two custom boards in KiCad.

Starting map (`cad/m11/interior_map.py`, nominal pose, body frame mm;
the interior is x −83 to 63, y ±51.5, z −40 to 36):
- Free rear strip behind the motor row: x −83 to −68, full width and
  height. This is the candidate for the Pi Zero 2 W, standing in the
  y–z plane.
- Free front corners outboard of the side moteus boards: x 35 to 63,
  |y| 33 to 51.5. These are the candidates for the power board.
- The host envelope (x −14 to 26, y 14 to 24, z 6 to 36) becomes the
  Teensy carrier. Its IMU would sit about 30 mm from the plant's
  assumed IMU position (−10, 0, −2 mm).

Steps:
1. **Envelopes from manufacturer data.**
   - moteus-c1: the r1.2 2D CAD/STEP, plus the mated XT30PW and XT30U
     plugs, phase-wire exits, the JST PH-3 CAN chain and the GH7 aux.
   - Matek BEC12S-PRO, the Pi Zero 2 W and its camera cable.
   - The carrier (Teensy 4.0 on a 2-layer board).
   - The power board: XT30 in, MINI fuse holder, LM5060, MOSFET,
     T-splice outputs.
   - The shell power switch.
2. **Packaging probe** (`cad/m11/packaging_probe.py`). Test each
   candidate placement against every part (tub and cover screw columns
   included) at the sit, low, nominal and extended poses with exact
   booleans. Pick placements with assembly access: the battery hatch,
   and USB access to the Teensy for firmware.
3. **Harness.**
   - Count the conductors on each route. Each hub carries 3 phases plus
     6 encoder wires down its leg; the RO40s stay inside the body.
   - Check the bundles against the hollow-shaft bores and channels,
     the bend radius, and the twist over each joint's range.
   - Check AS5047P SPI over the hub run length.
   - Route the CAN chain and power drops inside the body.
4. **Boards** (KiCad 9 via `.local-tools`).
   - The carrier: Teensy 4.0 pads (including CAN3 on 30/31), TCAN330G,
     LSM6DSO, the bus divider, connectors and the split termination.
   - The power board: phase C's LM5060 path with a stocked 40 V
     N-MOSFET chosen against its SOA, the fuse holder and the outputs.
   - ERC, DRC, Gerbers, drill files, BOM and placement files, using
     common LCSC parts.
5. **CAD adoption.** Add the parts to `cad/m10/assembly.py` and the
   config, then run the pipeline, adopt the CAD masses and sweep.
6. **Gates.** The mass and IMU position change the plant and sensor
   model, so the falls and the acceptance run again.

Decisions for the user, raised when known: the shell switch location
(visible), and any visible change to the shell.

## Phase D results: packaging (18 October 2026, layout study)

`cad/m11/packaging_probe.py` intersects each envelope, keep-outs
included, with every M10 part at hip heights 0.060, 0.072, 0.103 and
0.135 m. The parts include the tub, the cover, the battery hatch and the
Ø8 screw columns, and the booleans are exact. Each envelope is also
checked against the others. All of the placements below clear every
pose, with no mutual overlaps (`artifacts/m11/packaging/probe.json`).
They are envelopes from manufacturer data plus stated margins; the
board layouts in step 4 replace the two custom-board boxes.

| Item | Body-frame box, mm (x; y; z) | Notes |
|---|---|---|
| Pi Zero 2 W (reserved) | −81 to −75; ±32.5; −12 to 18 | Stands in the y–z plane against the rear wall. The camera FPC keep-out (y 32.5 to 41, z −3 to 9) passes behind the rear screw column. |
| Power board | −82 to −73; ±30; −33.5 to −13.5 | 60 × 20 mm, stands in the y–z plane below the Pi. Carries the XT30 input, LM5060, DPAK MOSFET and output pads. The MINI fuse is inline in the pack lead. |
| Matek BEC12S-PRO | −12 to 24; −23.5 to −16.5; 6 to 32 | Stands in the x–z plane, the mirror of the carrier slot. The 5 V lead crosses the centre to the carrier. |
| Teensy carrier | −14 to 26; 13.9 to 23.9; 6 to 36 | Replaces the HostPowerIMU envelope: a 40 × 30 mm board in the x–z plane. |
| Power switch (proposed) | −83 to −76.5; 31 to 41.5; −27.5 to −19.5 | Miniature SPDT slide switch on the flat rear wall, beside the power board and behind the rear screw column. It switches only the LM5060 EN/UVLO divider (microamps). Its location is the user's call (visible). |
| Teensy USB access | 26 to 46; 15.5 to 23.5; 20 to 33, and a cable column to the hatch | With the pack removed, a down-angle micro-USB plug reaches the Teensy's +x port through the battery bay and out of the floor hatch. No shell opening is needed. |
| moteus connector keep-outs | top and bottom rows: 13 to 14.5 mm off the +x edge; side boards: 14 mm above | Cover the XT30 and PH-3 plugs and the wire bends. |

Rejected placements:
- The front outer corners (x 35 to 61, |y| > 44.5) hit the inner edge
  fillets of the tub and cover past x 56.
- A BEC lying flat under the cover overlaps the side boards' connector
  keep-out (x 21.5 to 24), or leaves no room for its leads.
- The power board's first box (z down to −37) cut the tub floor
  fillet below z −34.4.

Consequences carried forward:
- The carrier IMU sits at about (6, 18.9, 21) mm. The plant's assumed
  IMU position is (−10, 0, −2) mm, so the sensor model changes in step 5
  and the gates in step 6 cover it.
- The Teensy USB route only works with the pack removed. The carrier
  cuts the Teensy's VUSB–VIN pad, which is PJRC's guidance when VIN has
  an external supply, so the BEC never back-feeds the USB host. For
  flashing, a bench supply on the pack's XT30 lead in the open bay
  powers the logic.

## Phase D harness findings (draft, 18 October 2026)

Geometry comes from `cad/m11/harness_geometry.py`
(`artifacts/m11/harness/geometry_0.103.json`), which gives a 59 mm
thigh gallery run and a 52 mm shin channel. Each hub carries 3 phase
leads and 6 encoder leads. The c1 boards beside the battery drive the
hubs, and the motor-row boards drive the RO40s. The estimated run is
about 0.32 m per hub: body 60 mm, inner-shaft bore 48.5, gallery 59,
knee shaft 10, shin 52 and axle tube 26.5 mm, plus slack.

- **Wire fit** (9 wires, packing fraction 0.6 to 0.75, against the
  3.0 mm ports, the 3.1 mm gallery and the 3.2 mm shin channel):
  - Popular silicone wire does not fit. Adafruit lists 26 AWG at
    1.4 mm ±0.1, and vendors list 30 AWG at 0.8 to 1.0 mm. The bundle
    would be about 3.7 to 4.0 mm.
  - Thin-wall PTFE fits: Alpha Wire 2843/7 (26 AWG, 7/34, 0.79 mm) and
    2841/7 (30 AWG, 7/38, 0.61 mm) give a 2.3 to 2.6 mm bundle. The M10
    basis of 1.0 and 0.6 mm sat between the two families.
- **Ampacity:** the hub phase current is 0.43 A RMS and 2.8 A peak
  (phase A), so 26 or 28 AWG is enough.
- **Lead resistance:** about 0.09 Ω line to line over 0.32 m. This adds
  roughly 0.25 V to the 20.7 V hub voltage demand peak, which makes
  the open GL40 voltage-limit item slightly worse.
- **Flex at the rotating crossings:** the body port sees the a2 range;
  the gallery and knee-shaft ports see the knee range, up to 101°
  between the end stops. If the bundle wraps directly on the 10 mm
  shafts, the strand bending strain is about 1.5% (7/34) or 1.0%
  (7/38). That suggests only about 10³ to 10⁴ cycles for copper. An
  in-plane service loop of radius 10 mm or more in the gallery annulus
  roughly halves the strain. This needs a bench cycle test before it
  counts as hardware validated.
- **AS5047P SPI:** single-ended over about 0.32 m, bundled with the PWM
  phase leads. Timing and capacitance are fine at a few MHz. The risk
  is capacitive pickup from the phase edges, so plan on twisting the
  phase leads, grouping the encoder wires with ground, adding series
  resistors and a conservative SPI clock, then a bench check.

Decisions (user, 18 October 2026):
- The power switch goes on the flat rear wall beside the power board,
  as probed.
- Harness wire is thin-wall PTFE: Alpha Wire 2843/7 for the phases and
  2841/7 for the encoder, or equivalent 7-strand thin-wall PTFE.
- Service loops of radius 10 mm or more at the rotating crossings.

Computed check (`python -m bipe.m11.harness`, writes
`artifacts/m11/harness/report.json`; inputs in
`m10.electronics.harness`). All checks pass:
- **Bundle:** 2.34 to 2.62 mm (packing 0.75 to 0.6). Wire-area fill is
  46% of the 3.0 mm ports, 43% of the gallery and 40% of the shin
  channel. The rejected silicone bundle is 3.6 to 4.0 mm and fits none
  of them.
- **Phase leads:** the route is 0.29 m with 15% slack. Resistance is
  0.125 Ω/m (7/34), so 0.074 Ω line to line, inside the plant's 0.09 Ω
  allowance. The drop at the 2.8 A peak is 0.21 V. Current density is
  3.1 A/mm² RMS against a 5 A/mm² bundled screen (an assumption).
- **Encoder:**
  - The supply drop is 3 mV at 15 mA.
  - SPI runs at 2 MHz. moteus defaults `aux2.spi.rate_hz` to 12 MHz,
    above the AS5047P's 10 MHz, so phase E sets it explicitly.
  - Edge time is about 5 ns, with 33 Ω series resistors and assumed
    line values of 100 pF/m and 50 Ω.
- **Crossings:**
  - Ranges: body port 67° (a2); hip-gallery and knee-shaft ports 101°
    (knee).
  - The 10 mm service loops take up 11.7 to 17.6 mm of length change.
  - Strand strain is 0.83% (phase, 7/34) and 0.52% (encoder, 7/38),
    against 1.48% and 0.96% wrapped directly on the shaft.
  - Thin-wall PTFE with 19/38 stranding (M16878/6 class) would bring
    the phase strands to about 0.5%. This is the preferred phase lead
    if a stocked part is found at the board BOM stage.
  - Fatigue life stays a bench cycle test, and the loop annuli still
    have to be checked in the CAD in step 5.

## Hub voltage limit review (18 October 2026, proposal)

The plant caps the GL40 drive voltage at a fixed 16 V
(`physics.hub_motor.drive_voltage_limit_V`), described as "a driver
setting". Two of its premises do not hold:
- moteus has no output-voltage cap. `servo.max_voltage` is an input
  over-voltage fault (moteus `docs/reference/configuration.md`).
- 16 V is the GL40's rated (nominal) voltage. CubeMars lists no
  maximum operating voltage, only a 500 V insulation test.

The real limit is the bus voltage times the modulation limit. The
phase A traces (5 ms samples, plus the 0.09 Ω PTFE leads) give the
following share of time where the hub's voltage demand exceeds the
limit, in the worst case:

| Limit | 20 °C | 60 °C | 80 °C | 125 °C |
|---|---|---|---|---|
| 16 V (current plant) | 0.06% | 0.31% | 0.47% | 0.95% |
| 0.95 × 19.2 V = 18.2 V (low pack) | 0 | 0 | 0.06% | 0.34% |
| 0.95 × 22.2 V = 21.1 V (nominal pack) | 0 | 0 | 0 | 0 |

The 125 °C phase A scenario is far from the hubs' duty. Mean copper
loss is at most 0.64 W (slope_up_10), so reaching 125 °C would take
about 160 K/W from a 25 °C ambient. Even 50 K/W at 40 °C gives about
72 °C. The 0.95 modulation limit is an assumption to check against the
moteus firmware.

Adopted (user sign-off, 18 October 2026):
- Replace the fixed 16 V with the low-pack bus limit, 0.95 × 19.2 V.
- Evaluate the winding resistance at 80 °C.
- Rerun the gates once, together with phase D step 6.

This also raises the plant's hub no-load speed from 106 to 121 rad/s.

Implemented in `physics.hub_motor`: `drive_voltage_limit_V` 18.24,
`voltage_limit_winding_temperature_C` 80, `lead_resistance_ohm` 0.09.
`bipe/m10/plant.py` applies them through `hub_drive_resistance`. Unit
tests pass (117). The falls and acceptance gates have not been rerun
yet; they run in phase D step 6. Until then the M10 acceptance records
still describe the 16 V plant. The phase A requirement traces also come
from the 16 V plant; their time-at-voltage-limit fractions are
superseded by the table above.

## Phase D results: boards (18 October 2026, digital candidates)

Both boards are routed in KiCad 9 and pass ERC, DRC and the schematic
parity check with 0 violations and 0 unconnected items. They are
digital candidates. None has been ordered, assembled or tested.

| Board | Size | Tracks / vias | Pours | Status |
|---|---|---|---|---|
| Teensy carrier | 40 × 30 mm, 2 layers | 280 / 10 | GND, both layers | passes |
| Power board | 60 × 20 mm, 2 layers | 160 / 3 | GND both layers; VBAT, BUS over the 2 mm tracks | passes |

How the boards are built:
- `tools/m11_boards.py` generates the schematic, placed PCB, pin
  audit, JLC BOM, hand-fit BOM and Specctra DSN from
  `m10.electronics.boards`.
- Freerouting 1.9.0 routes the DSN into `routed.ses`. The session
  files are committed, so the routing is reproducible without
  rerunning Freerouting.
- `tools/m11_boards_finish.py` then:
  - imports the session and pours and fills the zones;
  - runs `kicad-cli` ERC and DRC with parity;
  - exports Gerbers (zipped), the Excellon drill files, KiCad and JLC
    placement files, SVG plots and the schematic PDF;
  - writes `artifacts/m11/boards/<board>/report.json` with file hashes.
- Repeated DSN exports list pins and components in a different order,
  but the placements and nets are identical.
- `tests/test_m11_boards.py` checks:
  - the committed reports;
  - the outline against the config;
  - the board-axis handedness;
  - that the JLC placement file covers the BOM.

Design changes made while laying out the boards:
- **CAN transceiver.** The MCP2562FD-E/SN (LCSC C124016) replaces the
  TCAN330G, which LCSC doesn't stock. It runs from 5 V with VIO at
  3.3 V and handles CAN FD up to 8 Mbit/s. The logic budget now counts
  its 70 mA dominant-state maximum on the 5 V rail.
- **TVS on the bus.** An SMBJ30A (C135065) sits on BUS after the
  MOSFET: 30 V standoff and 48.4 V clamp, below the moteus 54 V rating
  and the MOSFET's 40 V rating.
- **LM5060 parts and checks** (`bipe/m11/power.py`, `lm5060_design`):
  - Parts: C_TIMER 330 nF, R_SENSE 10 kΩ, C_GATE 22 nF, a 10 Ω gate
    resistor and the AOD4184A (C99124).
  - Start-up takes 42 ms; the shortest timeout is 88.6 ms, a 2.1×
    margin.
  - The lowest trip current is 10.75 A, against the 6.6 A bus peak.
  - The linear-mode peak is 22.5 W, against a 50 W single-pulse rating
    at a 25 °C case. The roughly 68 K junction rise during the ramp is
    a bench check.
  - New tests: the timer and trip arithmetic, and a smaller timer
    capacitor failing the start-up check.
- **Teensy CAN3.** CAN3 is on the Teensy's bottom pads 30 (CRX3) and
  31 (CTX3). Two 30 AWG wire links run from those pads to carrier
  pads U1-29 and U1-30, just above the Teensy's upper header row.
  - The Teensy sits on 2.54 mm headers, and the SMD parts under it
    clear the 2.5 mm header spacer.
  - The Teensy's VUSB–VIN pad is cut so the 5 V comes from SYS. Both
    points are in `bom-hand-fit.csv`.
- **Connector direction and carrier mounting.** The packaging probe
  showed that side-entry PH plugs have no room on either board:
  - Carrier: the knee shaft is below it and the cover above it.
  - Power board: the tub floor sits 2.5 mm below its edge.

  Both boards now use top-entry PH headers (B2B C131337, B3B C131339,
  B7B C157981) with the plugs exiting along the board normal:
  - The carrier faces +y, into clear space at y 23.9–36 mm. Its −y
    side holds the knee pulley and belt.
  - The power board faces +x, into the gap at x −73 to −66 mm.

  The probe now carries both plug keep-outs, and every candidate is
  clear at all four hip heights. This also fixed a mirror error in the
  carrier config, which now has board y → body +z and its origin at
  z 6 mm.
- **Mounting.**
  - Carrier: three M2 holes, two at the −x end and one at the +x
    connector corner.
  - Power board: three M2 holes along its lower edge, clear of the
    MOSFET and the connectors.

Values for phase D step 5:
- IMU (U3) centre in the body frame: about (5, 15.9, 26) mm.
- Carrier stack: 1.6 mm PCB, 2.5 mm header spacer, Teensy.

Open items (bench, before hardware validation):
- the MOSFET's linear-mode temperature during start-up;
- part rotations in the JLC assembly preview;
- the CAN3 wire links;
- the harness fatigue test from the harness findings.

## Phase D step 5 plan: CAD adoption (19 October 2026)

Goal: the real electronics in `cad/m10/assembly.py` in place of the
HostPowerIMU envelope. The pipeline then gives the new mass, IMU
position and sweep, and step 6 reruns the gates.

1. **Board geometry from the KiCad layouts.**
   - The finisher writes `envelopes.json` per board: the board outline
     plus each footprint's body box in board millimetres, taken from
     its F.Fab outline (courtyard if it has none), and its side.
   - Body heights are new config values,
     `m10.electronics.boards.component_heights_m`. They are package
     maxima from the datasheets.
   - Mated PH plugs use JST's 8.0 mm mated height (ePH catalog) at the
     PHR widths.
   - The CAD maps board millimetres to the body frame with the
     committed board axes and origins.
2. **Other parts from config** (`m10.internals.electronics`, SI boxes,
   taken from the probed placements):
   - Pi Zero 2 W: 65 × 30 mm, 6 mm envelope.
   - Matek BEC12S-PRO: 35 × 24 × 5.5 mm.
   - Rear-wall slide switch (SS12D00G3 class): body inside the wall,
     and a slot for the actuator cut through the 2 mm rear wall. The
     user approved this visible change with the switch location.
3. **Service loops** (`harness` colour, mass in the harness
   allowance):
   - **Body port:** a half annulus below the inner shaft at the body
     port, r 5.5 to 11.3 mm and 2.6 mm thick. The port turns with a2,
     −30° to +37° from its nominal downward direction between the end
     stops, so it stays in the lower half with at least 53° to spare.
   - **Gallery:** full annuli of the same section around the inner
     shaft and the knee shaft in the gallery plane.
   - The loops may overlap the bundle parts they join. Any other
     overlap fails the internal checks (same link) or the sweep
     (cross link).
4. **Masses** (`m10.mass_properties`):
   - The two PCBs: volume × FR-4 density (1850 kg/m³).
   - Catalog or assumed masses, with their sources:
     - Teensy 4.0 with two 1×14 headers: 5.1 g (assumption);
     - the SMD parts and PH headers on each board;
     - BEC: 5 g (retailers);
     - Pi Zero 2 W: 10 g (reseller);
     - switch: 1 g.
   - The plugs and leads stay in the 60 g harness allowance.
   - HostPowerIMU (15 g) and the `internals.host` envelope are removed.
   - Expected change: about +17 g, which leaves a margin of about
     0.17 kg against the 0.15 kg minimum reserve. The ceiling and the
     reserve stay as they are.
5. **IMU position.** `physics.imu_position_m` becomes the U3 centre
   through the carrier transform. A test derives it again from the pin
   audit and the config.
6. **Pipeline and checks.**
   - Run two exterior → assembly → sweep → mass → adopt passes, then
     the sections, the shell clearance and manufacturability.
   - Rerun the packaging probe against the new parts.
   - Acceptance:
     - no same-link overlap involving a new part;
     - the sweep grid clean, with the sit pose found;
     - the mass gate passes with at least the 0.15 kg reserve;
     - the M11 and M10 unit tests pass.

## Phase D step 5 results: CAD adoption (19 October 2026, layout study)

The M10 assembly now carries the M11 electronics in place of the
HostPowerIMU envelope:
- the carrier PCB, Teensy stack and carrier parts, and the power PCB
  and its parts. These come from the KiCad layouts
  (`artifacts/m11/boards/<board>/envelopes.json`, written by the
  finisher);
- the BEC, the Pi Zero 2 W and the rear-wall switch, with a slot in the
  wall for its flush actuator;
- six service-loop parts.

The headless pipeline ran two adoption passes, then the sections, shell
clearance and manufacturability. All results are at layout-study
fidelity.

| Check | Result |
|---|---|
| Same- and cross-link overlaps (crouched, nominal and extended) | none |
| Sweep grid (8 heights × 5 axle offsets) | clean; sit pose at 0.054 m |
| Joint ranges covered | a1 −192.8 to −129.1°, a2 −59.1 to −7.6°, knee 80.0 to 166.5°, inside the end stops |
| CAD mass / allocated | 1.815 / 1.975 kg (was 1.800 / 1.960), margin 0.175 kg, above the 0.15 kg reserve |
| Nominal pose | a1 −153.7° (was −153.1°), a2 −35.5° (was −34.8°) |
| Manufacturability | nothing below threshold; the thinnest thigh sample went from 1.31 to 1.27 mm with the pose |
| Shell clearance behind the strike spots | unchanged at 7.6 / 6.6 mm |
| Packaging probe (all candidates, four hip heights) | clear, no mutual overlaps |
| `cad/m11/adoption_check.py` (each new part, four hip heights) | no overlaps |

Mass changes: the new parts total 30.1 g and replace HostPowerIMU's
15 g, a net gain of 15.1 g.
- Carrier PCB 3.6 g, Teensy stack 5.1 g, carrier parts 0.8 g.
- Power PCB 3.6 g, power parts 1.0 g.
- BEC 5 g, Pi 10 g, switch 1 g.

The body link goes from 0.883 to 0.898 kg, and the robot's centre of
mass moves 0.8 mm rearward.

Placement changes:
- **Carrier height.** The adoption check found the carrier touching
  the cover with no gap, so the carrier now spans z 5.5 to 35.5 mm.
  That leaves 0.5 mm to the cover and 0.5 mm to the inner shaft.
- **Power board origin.** It is now the KiCad (0, 0) corner on the
  board's back face, at x −82 mm. The carrier already used this
  convention; it is stated in `m10.electronics.boards.frame`.
- **IMU position.** `physics.imu_position_m` is now (5.0, 15.9,
  25.5) mm, the U3 package centre (was (−10, 0, −2) mm, an assumption).
  - `bipe.m11.boards.imu_position` computes it from the layout and the
    placement, and a test checks that the config matches.
  - The chip faces body +y, so the firmware maps its axes in phase E.

Service loops:
- Each loop is a 10 mm loop plus half the bundle: r up to 11.3 mm, one
  bundle (2.6 mm) thick.
- The body-port half annulus is 0.5 mm from the inner shaft.
- The gallery annuli sit 0.25 mm from the knee-belt pulley faces on
  both sides. This is the gallery's built width (3.1 mm against the
  2.6 mm bundle). The knee pulley turns relative to the thigh, so the
  bench harness test should also check rubbing on the pulley flange.

Board files: the generator and finisher now seed KiCad's UUIDs, so a
rerun reproduces the committed PCB and schematic files byte for byte.
Only the export dates in the fab files change.

Not modelled; open items:
- Mounting brackets or bosses for the carrier, BEC, Pi and power
  board. The old host envelope had none either. The carrier's top edge
  sits 0.5 mm under the cover, which is a candidate place for bosses.
- The wire runs and the camera FPC; their mass is in the 60 g harness
  allowance.
- The switch part number. It is an SS12D00G2-class part with
  datasheet-typical dimensions, to be chosen at order.

Next: step 6 reruns the falls and the acceptance gates once. That run
covers the hub voltage limit, the new masses and the IMU position.

## Phase D step 6 plan: gates (19 October 2026)

One rerun of the M10 gates. It covers three plant changes together:
- the hub drive-voltage limit of 18.24 V, with windings at 80 °C and
  the leads;
- the CAD masses from step 5;
- the IMU position, (5.0, 15.9, 25.5) mm.

1. Run `tests/test_m10_plant.py` and the unit tests first.
2. Run the falls at dt, then at dt/2 (6 workers).
3. Run `bipe.robustness --profile m10 --controllers current` (8 workers).
4. Run the jobs one at a time, under `nice -n 10`, and leave the config
   untouched while they run.
5. Compare against the committed records (falls `5e5ccd2`, acceptance
   `d54b29b`). Record the results here and commit the outputs with the
   configurations they saved.

Pass criteria are the same as in the earlier runs:
- forward and backward falls pass;
- sideways keeps only its known structural failures;
- robustness passes all physical cases;
- inherited cases miss only the case accepted at C6;
- the convergence and backlash checks pass.

## Phase D step 6 results: gates (19 October 2026, validated simulation)

This run covers the 18.24 V hub limit (windings at 80 °C, leads), the
step 5 CAD masses and the IMU at (5.0, 15.9, 25.5) mm. Unit tests passed
first (130).

Falls (dt and dt/2 agree; committed record `5e5ccd2` in brackets):

| Case | Worst factor, dt | Worst factor, dt/2 | Requirement |
|---|---|---|---|
| forward_rest | 2.64 (2.62) | 2.45 (2.40) | pass |
| forward_0p3 | 2.36 (2.28) | 2.14 (2.06) | pass |
| backward_rest | 3.26 (3.26) | 2.98 (2.96) | pass |
| backward_0p3 | 2.50 (2.48) | 2.41 (2.33) | pass |
| sideways_rest | 0.49 (0.48) | 0.52 (0.49) | **fails containment** |
| sideways_0p3 | 1.05 (1.59) | 0.96 (1.46) | pass |

- Forward and backward keep factor ≥ 2 on every path, with slightly more
  margin than before.
- **New open item: `left_Thigh_shell_travel_exceeded` in sideways_rest,**
  at both dt and dt/2:
  - The thigh's knee end now strikes 4.27 mm deep (4.33 mm at dt/2) at
    (89.2, 61.3, 8.1) mm in the thigh frame, inward normal
    (−0.91, 0.11, −0.40).
  - That is 8.3 mm from the probed backing spot, outside its 6 mm
    radius, so the default 4 mm travel applies.
  - The step 5 masses moved the strike; the depth is about the same as
    before (4.6 mm, then at the spot).
  - A read-only ray-cast with the `cad/m10/shell_clearance.py` method
    measures a 6.34 mm thin-wall gap at the new spot; the first obstacle
    is the knee belt. The existing spot still measures 6.65 mm, so the
    service loops don't change it.
  - Adding this as a second probed spot (6.3 mm travel) on each thigh
    would clear the requirement. That is a config change and needs the
    user's decision.
- Sideways recorded risks (not requirements):
  - The structural shortfalls are the known set, worst left_HipInnerBearing
    at 0.49–0.52.
  - sideways_0p3 adds the left hip-belt rating and slack at dt; dt/2 had
    them before. At dt/2 it adds left_HipInnerBearing.
  - SkidPad strain is 1.09–1.10 at both timesteps (was 0.53 / 0.48). The
    tub strikes at the skid's rear corner, (−74, −39) mm in the body
    frame, where the end bevel leaves thin foam; the 3.7 mm depth there
    reads as bottoming.

Acceptance (`bipe.robustness --profile m10`, 8 workers; committed record
`d54b29b` in brackets):
- Robustness: 70/70 physical passes (70/70).
- Inherited cases: 26/27; the only miss is delay_15ms_noise_x2_combined,
  accepted at C6 (same).
- Convergence and the backlash half-timestep check pass. Every required
  target holds:
  - survive and recover 2 N·s in both directions;
  - slopes through 10°;
  - rollback 0.058 m (0.059);
  - backlash p-p 0.51 / 0.61 / 0.93° (0.50 / 0.54 / 0.91);
  - belt range 0.92° (1.03).
- Recovery misses (survived, not recovered): push_low_+0.5,
  push_low_+2.5 and push_drive_+2.5.
  - The late wheel speed peak is 0.0306, 0.0326 and 0.0304 m/s, against
    the 0.03 m/s recovery limit (was 0.028–0.029).
  - The late pitch stays at about 1° p-p, so this is a residual speed
    oscillation sitting at the limit. It is not a loss of balance.
  - Margins: contiguous push_low_+1 falls from 2.5 to 0 N·s (the
    0.5 N·s hole); push_drive_+1 falls from 2.5 to 2 N·s. The largest
    observed recovery is 2 N·s in both.
- Conditional electrical screen:
  - peak DC input 8.19 A (9.47);
  - wheel RMS 0.233 N·m (0.240);
  - peak regen 38.4 W (38.1).
- The screen's basis text now reads the configured drive-voltage limit
  instead of a fixed "16 V" (`bipe/robustness.py`).

Containment close-out (user decision 19 October 2026):
- The user approved re-probing the gap at the new strike spot. Each
  thigh's `shell_backing` entry now lists the second spot,
  (89.24, ±61.25, 8.05) mm with inward normal (−0.91, ±0.112, −0.399).
- `cad/m10/shell_clearance.py` measures:
  - 6.34 mm at the new spot and 6.65 mm at the first, on both thighs,
    with the knee belt as the first obstacle;
  - the BodyTub unchanged at 7.6 mm.
- Thigh travel is therefore 6.3 mm (was 6.6).
- The backing only acts past its travel and the strikes stay below
  4.6 mm, so the dynamics are unchanged. The fall factors match the run
  above to every printed digit.
- Falls rerun at dt and dt/2: every requirement passes. The sideways_rest
  strike now reads at the spot (4.27 / 4.33 mm against 6.3 mm); the
  deepest off-spot depth is 0.32 mm.
- Acceptance doesn't need a rerun: the change only affects thigh strikes
  deeper than 6.3 mm, which a balancing case can't produce.
- `tests/test_m10_plant.py`: 25 passed.

Status:
- The M10 records now describe the M11 plant. Phase D is complete.
- Recorded and not failing a required target:
  - the three recovery misses at the late-speed limit;
  - the SkidPad rear-corner bottoming in the sideways falls.

## Phase E plan: firmware hardware layer (19 October 2026)

Goal: the Teensy-side hardware layer for the M11 electronics. It covers:
- the moteus CAN-FD protocol on CAN3;
- the LSM6DSO driver and its axis map;
- the 200 Hz bus schedule and its latency budget;
- the supervisor and power states for the M11 power path;
- a Teensy 4.0 build.

The parity tests run on the host. Nothing is flashed or powered, and the
motor-enable lock (`BIPE_HARDWARE_QUALIFIED 0`) stays in place: the image
can query drivers but cannot command torque until hardware commissioning
supplies the evidence bits.

Starting point:
- `firmware/` holds the portable controller core (parity-tested against
  Python for the serial profile) and an M4-era hardware layer:
  - a CubeMars MIT codec for the AK48;
  - classic CAN1 at 1 Mbit/s, listen-only;
  - a dump/precharge power policy.
- None of these matches the M11 parts, so the M11 layer is new code next
  to them. The M4 files stay for their own tests.

Sources: the mjbots moteus reference at commit 8f747f4 (27 September
2026):
- `docs/protocol/can.md`: CAN-FD at 1 Mbit/s nominal and 5 Mbit/s data;
  16-bit ID, source high byte (bit 15 requests a reply), destination low
  byte; multiplex subframes; NOP padding 0x50.
- `docs/protocol/registers.md`: register numbers, the int8/16/32 and
  float mappings, modes and fault codes.
- `docs/reference/configuration.md`:
  - `servo.pid_position` kp/kd are in N·m per output revolution;
  - `motor_position.rotor_to_output_ratio`;
  - `aux2.spi.rate_hz` (default 12 MHz) and `aux2.spi.mode` 2 for the
    AS5047P;
  - `servo.default_timeout_s` / `timeout_mode`;
  - `servo.max_voltage`, `flux_brake_margin_voltage` and
    `max_regen_power_W`.

Steps:
1. **moteus codec (portable C, `firmware/src/m11/moteus.c`).**
   - Encode position-mode commands, queries and stop frames; decode
     replies.
   - Joints: mode 10; position, velocity and feedforward torque as
     float; kp/kd scale as int16; one reply request.
   - Wheels: the same frame with position NaN and kp/kd scale 0, so it
     carries pure feedforward torque.
   - Query: position, velocity and torque (float); voltage, temperature
     and fault (int8); mode (int8).
   - The decoder rejects unknown subframes, short frames and stale
     sequence numbers, and checks the ID routing.
   - Parity: byte-for-byte against mjbots' own `moteus` Python package
     (`make_position`, `make_query`, `make_stop` and its reply parser),
     across random commands.
2. **Units and driver configuration (`bipe/m11/moteus_config.py`).**
   From `config/robot.json`, write one `conf set` file per driver
   (ids 1–6) to `artifacts/m11/firmware/moteus/`:
   - kp, kd: the controller's joint gains × 2π (N·m/rev); ki 0;
   - `rotor_to_output_ratio` 0.25 for the joints and 1 for the hubs;
   - aux2 AS5047P at 2 MHz;
   - `pll_filter_hz` from the plant's 2 ms local velocity filter;
   - position limits from the end stops;
   - `max_current_A` from the motor peak;
   - `max_voltage` 30 V, flux braking from 27 V, `max_regen_power_W`
     10 (phase C);
   - the watchdog timeout and timeout mode (decision below).
   A test checks two things:
   - the moteus position-mode law with this configuration reproduces the
     plant's joint law, ff + Kp (qref − q) + Kd (qdref − qd), to
     floating-point accuracy;
   - each joint's travel fits the rotor window (a 4:1 belt and a rotor
     encoder give an unambiguous output angle over 90°; the joints use
     60° and 54°).
   Output sign and offset are commissioning values, left as identity
   and flagged.
3. **IMU driver and axis map (`firmware/src/m11/imu.c`).**
   - LSM6DSO over SPI (mode 3, 8 MHz, under the 10 MHz limit):
     WHO_AM_I check; 1.66 kHz ODR, ±4 g / ±500 dps; burst read of
     gyro and accel; data-ready on INT1 (D9).
   - The chip-to-body rotation comes from the LSM6DSO datasheet axis
     figure, the U3 footprint rotation and the carrier frame
     (`m10.electronics.boards.frame`). It is written into a generated
     header.
   - A startup check rejects a gravity sample more than 30° from
     body +z.
   - Parity: C mapping against Python, for random samples.
4. **Bus schedule and latency budget (`bipe/m11/timing.py`).**
   - Each 5 ms cycle: read the IMU at data-ready; query all six drivers;
     compute; send the six commands.
   - The budget adds worst-case CAN-FD frame times (extended ID, BRS,
     worst-case stuff bits, CRC-17/21), the driver reply turnaround
     (an assumption to measure), SPI time and a compute bound. It
     compares sample-to-actuation latency with the plant's 5 ms
     `transport_delay_s`. The FOC lag and the IMU filter are modelled
     separately, so the budget doesn't count them.
   - It writes `artifacts/m11/firmware/timing.json`.
   - The Teensy image times each phase with the DWT cycle counter, for
     the bench check.
5. **Supervisor and power states (`firmware/src/m11/power_states.c`).**
   - States: OFF, DISCOVER, CALIBRATE, READY, ACTIVE, STOPPING, FAULT.
   - DISCOVER queries ids 1–6 (mode, fault, voltage). The transceiver's
     STBY pin (D2) stays high until power good.
   - Inputs:
     - LM5060 PGD (active low, D7);
     - the bus ADC on A0 (180 k / 10 k, ×19);
     - arm and stop inputs (D4, D5, active low).
   - Thresholds: arm at ≥ 19.2 V, fault below the 18.0 V firmware
     cutoff; the hardware backstop is 17.8 V.
   - Faults: driver fault or missing reply, IMU loss and over-
     temperature. STOPPING sends stop frames, mode 0, to every id.
   - Host tests cover the transitions.
6. **Teensy build.**
   - A new PlatformIO environment `teensy40_m11` builds the M11 image:
     FlexCAN_T4FD on CAN3 at 1/5 Mbit/s, SPI, and the pins above,
     generated from the carrier netlist so the board and the firmware
     can't disagree.
   - The torque command path stays compiled out while
     `BIPE_HARDWARE_QUALIFIED` is 0.
   - Built with the local PlatformIO.
7. **Checks and records.**
   - The firmware make tests and pytest.
   - The plant and controller don't change, so acceptance doesn't need
     a rerun. Phase F confirms that.

Exit:
- parity tests pass for the codec, units and IMU map;
- the timing budget meets the 5 ms assumption with margin;
- the M11 Teensy image builds.

Out of scope until hardware: commissioning evidence (signs, offsets,
calibrations, measured timings).

Decisions for the user:
- **Downloads.**
  - mjbots' `moteus` Python package from PyPI, as the reference for the
    codec parity tests. It is a test dependency only and goes in the
    lock file.
  - The ST LSM6DSO datasheet PDF from st.com, for the axis-orientation
    figure.
- **CAN library.** FlexCAN_T4FD, which is bundled with Teensyduino and
  needs nothing new, or ACAN_T4, an extra download.
- **CAN-loss behaviour.** The driver watchdog (`servo.default_timeout_s`,
  4 control periods = 20 ms) and what each driver does on timeout:
  - stopped (limp), mode 0;
  - "zero velocity" damping, mode 12, with a capped torque.

User decisions (19 October 2026):
- Download both references: the `moteus` 1.1.1 package (test only) and
  the LSM6DSO datasheet.
- CAN library: FlexCAN_T4FD.
- CAN loss: the joints damp (mode 12, capped torque); the wheels go limp
  (mode 0).

## Phase E results: firmware hardware layer (19 October 2026, host parity and a build)

Status: host parity tests and an estimated timing budget. Nothing has been
flashed, powered or measured on hardware. The motor-enable lock stays in
place.

Delivered:
- **moteus codec** (`firmware/src/m11/moteus.c`, `firmware/include/m11_moteus.h`).
  - Position, query and stop frames; reply parsing.
  - Byte-for-byte equal to `moteus` 1.1.1 (`make_position`, `make_query`,
    `make_stop`) over 3000 random commands per frame type, including
    resolution grouping, saturation and NaN.
  - The reply parser equals `protocol.parse_reply` over 3000 random
    replies, and reproduces the worked example in `can.md`.
  - The parser rejects foreign destinations, error/write/read subframes,
    unknown registers and truncated frames. moteus replies carry no
    sequence number, so staleness is checked by the supervisor's
    feedback age instead (20 ms).
- **Hardware description, generated** (`tools/generate_m11_hardware.py` →
  `firmware/include/m11_hardware.h`, `artifacts/m11/firmware/moteus/*.cfg`).
  - Teensy pins come from the routed carrier board and its symbol.
  - The LSM6DSO rotation comes from the datasheet axis figure (DS12140
    Rev 3, Fig. 3), the U3 footprint and the carrier frame:
    body = [[1,0,0],[0,0,1],[0,−1,0]] · chip. That is, body x = chip X,
    body y = chip Z and body z = −chip Y.
  - The driver table and `conf set` files come from `config/robot.json`
    and `controller_m10.json`. Joint kp 50.27 and kd 0.314 N·m per
    output revolution (Kp 8, Kd 0.05 × 2π). The rotor encoder is AS5047P
    on aux2 at 2 MHz, with `pll_filter_hz` 79.6 (τ 2 ms). The
    `servopos` limits are the end stops about their centre.
  - Watchdog `default_timeout_s` 0.02. On timeout the joints switch to
    mode 12 capped at 0.5 N·m, and the wheels to mode 0.
  - Output sign and offset are flagged as commissioning values.
  - A test regenerates the outputs and compares them with the committed
    files.
- **Hardware layer** (`firmware/src/m11/layer.c`).
  - LSM6DSO setup and the burst conversion to body rad/s and m/s².
  - The 30° gravity check.
  - Joint and wheel command frames, and replies converted to supervisor
    samples.
  - The bus monitor scale.
  - Outputs per supervisor state.
- **Timing budget** (`bipe/m11/timing.py` → `artifacts/m11/firmware/timing.json`).
- **Teensy image** (`firmware/src/teensy_m11.cpp`, env `teensy40_m11`,
  `make -C firmware teensy-m11`).

Parity and state tests (`tests/test_m11_moteus.py`, `tests/test_m11_firmware.py`):
- IMU conversion against the datasheet scales and the rotation, over
  500 random samples.
- Joint law. The C joint frames are decoded with moteus'
  `parse_registers`, then the drive's position-mode law is applied with
  the generated kp and kd. The result equals the plant's
  ff + Kp (qref − q) + Kd (qdref − qd) to 2e-5 N·m (float32 rounding),
  for both signs.
- Wheel frames carry feedforward torque only.
- Reply → sample conversion, including fault and timeout modes.
- Joint end-stop travel (75° hip, 67° knee) fits the 90° output window
  of one rotor turn.
- The supervisor (`bipe_tick`) fed through the M11 adapter:
  - OFF → CALIBRATING → READY → ACTIVE;
  - faults from low bus voltage, lost power good, stop input, driver
    fault mode, driver watchdog timeout and a missing reply;
  - no arming below the 19.2 V arm minimum;
  - no torque frames from an unqualified profile.

Timing budget (estimate; `timing.json`):

| Phase | Time |
|---|---|
| IMU sample age (1/1660 s) | 0.602 ms |
| IMU SPI burst (13 B at 8 MHz + 20 µs) | 0.033 ms |
| Six queries (6 B) and replies (24 B), plus one driver turnaround (0.25 ms, assumed) | 1.382 ms |
| Compute (assumed bound) | 0.500 ms |
| Four joint (20 B) and two wheel (32 B) commands | 0.677 ms |
| Driver applies the command (0.25 ms, assumed) | 0.250 ms |
| **Sample to actuation** | **3.44 ms of 5 ms (1.56 ms margin)** |

- Bus load is 36 % of each 5 ms cycle.
- The 20 ms watchdog is 4 cycles.
- The host-measured cycle of the same firmware work is 21 µs at p99,
  through ctypes. It covers the IMU conversion, six reply decodes, the M10
  estimator and controller steps and six command encodes. The 0.5 ms
  compute bound is 23× that.
- Frame times use ISO 11898-1:2015 bit counts with worst-case dynamic
  stuffing.
- The turnaround and compute figures are assumptions. The image's DWT
  phase timers replace them at the bench check.

Teensy image:
- The image builds with the local PlatformIO against the M10
  `generated_config.h`. It is generated into `firmware/build/m11-config`
  and selected with `-iquote`. A `static_assert(BIPE_SERIAL == 1)`
  proves which header was used.
- Flash: 67.6 kB code. RAM1: 19.1 kB variables.
- There are no warnings in the project sources. The 23 warnings come
  from the Teensy core and FlexCAN_T4.
- `static_assert(BIPE_HARDWARE_QUALIFIED == 0)`.
- The torque path is locked:
  - The supervisor faults with PROFILE once started.
  - The image queries the drivers and sends stop frames.
  - It encodes the controller's frames only so the compute can be timed;
    it never sends them.
  - Each second it reports the worst phase times over serial.

Checks:
- The full pytest suite passed (158).
- The M11 tests passed after the power-good gate was added (28).
- `make -C firmware test` passed.
- `make -C firmware teensy-m11` built.
- The plant and the controller are unchanged.

Differences from the plan, recorded:
- Commands don't request replies, and the kp/kd scales are left at the
  drive's default of 1. Feedback comes from the query phase, so the bus
  carries each reply once.
- The supervisor is the existing `bipe_tick`, not a new
  `power_states.c`. It has the planned states plus FALLING and FALLEN.
  - STOPPING keeps torque enabled. It is the supervisor's controlled
    stop, as in the plant.
  - FAULT sends stop frames.
  - The transceiver leaves standby only while the supervisor runs and
    the LM5060 reports power good.
- The moteus configuration writer is part of the hardware generator,
  not a separate `bipe/m11/moteus_config.py`.
- **Setpoint advance (for sign-off).** In position mode, moteus advances
  its position setpoint at the commanded velocity between frames. The
  plant holds qref for each 5 ms tick. The largest difference is
  Kp · max|qdref| · 5 ms = 8 × 0.68 rad/s (the fastest crouch reference)
  × 5 ms = 0.027 N·m. That is small next to the joint commands (the
  1.4 N·m protective cap). The plant is unchanged, so acceptance does
  not need a rerun.

Out of scope until hardware: output signs and offsets, `moteus_tool
--calibrate`, measured turnaround and compute, CAN bus captures, and the
evidence bits for `BIPE_HARDWARE_QUALIFIED`.

User decision (19 October 2026): the setpoint advance is accepted as a
recorded difference. The plant keeps its zero-order hold.

## Phase F plan: exit (19 October 2026)

1. **Acceptance status.** The committed acceptance (`70da4c6`) predates one
   plant change: the second thigh backing spot (`ed939cb`), which cut the
   thigh travel from 6.6 to 6.3 mm. Phase E changed no plant or
   controller file; the 25 runtime-source hashes saved with the record
   still match. The explanation to check:
   - the plant's thigh shell force differs only at depths past 6.3 mm;
   - every powered case ends at its first non-tire contact (a forbidden
     contact), long before such a depth;
   - the one unpowered case runs until 30° pitch error and does not end
     on contact.
   To confirm, rerun `unpowered` alone into a scratch output and compare
   it with the committed row. Identical → acceptance unchanged, no full
   rerun. Any difference → full acceptance rerun.
2. **Milestone exit record.** A summary of phases A–F in this document,
   keeping the layout study, validated simulation and hardware-validated
   claims separate (nothing is hardware-validated), plus the open items
   for hardware.
3. **README.** An M11 status section and the commands:
   - the hardware generator;
   - the timing budget;
   - the Teensy M11 build;
   - the test-only moteus environment (`requirements-moteus.lock`).
4. **Checks.** ruff on the touched Python, the full pytest suite,
   `make -C firmware test` (restoring the tracked `generated_config.h`
   afterwards) and the Teensy build.
5. Commit the records and the README.

## Phase F results: exit (19 October 2026)

**Acceptance: unchanged (checked, not rerun).**
- The committed record `70da4c6` predates one plant change: the second
  thigh backing spot (`ed939cb`, thigh travel 6.6 → 6.3 mm). The 25
  runtime-source hashes saved with the record still match. Phase E
  changed no plant or controller file.
- The plant's thigh shell force is the shell stiffness times depth. The
  backing adds a term only past its travel (`bipe/m10/plant.py`), so the
  change affects strikes deeper than 6.3 mm and nothing else.
- Every powered case fails at its first non-tire contact
  (`forbidden_contact` in `bipe/moving_experiment.py`), so none can reach
  that depth.
- The one unpowered case runs to 30° pitch error regardless of contact.
  I reran it alone with the current plant
  (`artifacts/m11/exit/unpowered_check/`). Every measured value and
  trace equals the committed record; only the saved configuration
  differs (the new spot and travel).
- The acceptance therefore stands:
  - robustness 70/70;
  - inherited 26/27 (`delay_15ms_noise_x2_combined`, accepted at C6);
  - every required target met;
  - recorded, not failing: three recovery misses at the 0.03 m/s
    late-speed limit.

**Milestone summary.** M11 is a digital electrical design for the M10
robot. Nothing is bought, built, flashed or powered.

| Phase | Result | Kind of evidence |
|---|---|---|
| A. Requirements | Loads from 17 acceptance cases: worst driver 3.41 A peak / 1.88 A RMS; bus 6.63 A peak at 19.2 V, 127 W, never regenerates; runtime 28–39 min (draw scenario) | Validated simulation (M10 plant) |
| B. Architecture and parts | Six moteus-c1, Teensy 4.0 on a custom carrier with the MCP2562FD and LSM6DSO, Pi Zero 2 W slot reserved; prices and stock recorded | Part selection from manufacturer documentation |
| C. Power and protection | LM5060 high-side switch, 10 A MINI fuse, 5 V module, regen and fault table; seven checks pass | Calculation against phase A and datasheets |
| D. Packaging, harness and boards | Both boards routed (ERC/DRC pass); parts and harness in the CAD, clean sweep; CAD mass 1.815 / 1.975 kg allocated (0.175 kg margin above the 0.15 kg reserve); falls and acceptance rerun with the M11 plant | Layout study; validated simulation for the gates |
| E. Firmware hardware layer | moteus codec and joint law equal to the moteus package; IMU map from the routed board; 3.44 ms of 5 ms latency; Teensy image builds with torque locked | Host parity tests and an estimate |
| F. Exit | Acceptance unchanged, checked above; README updated | Records |

**Open items before hardware validation.** They are collected from the
phases above:
- **Bench measurements:**
  - the MOSFET's linear-mode start-up temperature;
  - moteus logic power;
  - transceiver, camera and 5 V currents;
  - pack resistance;
  - driver turnaround and Teensy compute time, with the image's DWT
    timers;
  - CAN bus captures.
- **Commissioning:**
  - moteus output signs and offsets;
  - `moteus_tool --calibrate`;
  - the watchdog damping cap (0.5 N·m);
  - the evidence bits that `BIPE_HARDWARE_QUALIFIED` needs.
- **Build:**
  - part rotations in the JLC assembly preview;
  - the CAN3 wire links;
  - mounting bosses;
  - the switch part number;
  - wire runs and the camera FPC (inside the 60 g allowance);
  - the harness fatigue and pulley-rub test.
- **Risks carried from M10:**
  - the RO40 Lite stock;
  - the sideways-fall structural shortfalls (signed off);
  - the SkidPad rear-corner bottoming;
  - the AS5047P "not recommended for new designs" status, with the
    MA732 as the fallback.

Checks:
- The full pytest suite passed (158).
- `make -C firmware test` passed; the tracked `generated_config.h` was
  restored afterwards.
- `make -C firmware teensy-m11` built.
- ruff passed on the M11 Python.

Exit met: acceptance unchanged and explained, results recorded, and the
README updated. M11 completes the agreed initial scope as a
hardware-oriented digital prototype. Camera following and jumping stay
later work.

Follow-up (19 October 2026, user request): `firmware/include/generated_config.h`
is no longer tracked.
- The generator writes each config file's SHA-256 into the header, so
  the committed copy went stale with every `config/robot.json` commit.
  Its last commit was `940095a`.
- `make -C firmware test` rewrote it each time.
- The committed generator and the user's formatting-only edit produce
  byte-identical headers.
- `make -C firmware config` generates the header, and the new
  `make -C firmware teensy` builds the M4 diagnostic image with it. The
  host and test builds already generated their own.

Environment after the Ubuntu 24.04 → 26.04 upgrade (19 October 2026):
- **Python.** The upgrade removed the system Python 3.12 that `.venv`
  was built on (`/usr/bin/python3` is now 3.14.4). With the user's
  approval, `.venv` now points at a uv-managed CPython 3.12.13. Its
  packages are unchanged; a dry-run sync of the three lock files would
  install nothing. The full pytest suite passed (158).
- **Reproduction with the new toolchain.** Python 3.12.13 and GCC
  15.2.0 now build the C controller. Five powered cases were rerun:
  - `sensors_combined`, `ideal_combined`, `push_low_+2.5`,
    `slope_up_10` and `backlash_hold_0.6`;
  - every metric and record equals the committed acceptance, bit for
    bit;
  - outputs: `artifacts/m11/exit/os_upgrade_check/`.
- **Board tools: repaired (user request).** The first check was wrong
  about Java: the project-local OpenJDK 21 was intact but not on PATH.
  The KiCad 9.0.9 (24.04 build) in `.local-tools/root` had three
  problems:
  - It needs the 3.12 `libpython` that 26.04 removed.
  - Its 24.04 `libssh2` shadowed the newer system copy that 26.04's
    `libcurl` needs.
  - Three libraries are no longer installed.

  The fixes:
  - `tools/electronics-env.sh` now puts the system libraries before the
    extracted ones, adds uv's Python 3.12 library and puts the local Java
    on PATH.
  - The pcbnew scripts run with `python3.12` (uv-managed) instead of
    `/usr/bin/python3`.
  - With the user's approval, three 26.04 packages were downloaded with
    `apt-get download` and extracted into `.local-tools/root`:
    `libfreeimage3` 3.18.0+ds2-11build1, `libodbc2` 2.3.14-1 and
    `libhwloc15` 2.13.0-2 (652 kB).

  Checks:
  - `pcbnew` 9.0.9 imports with wxPython 4.2.1.
  - Rerunning `tools/m11_boards.py` and `tools/m11_boards_finish.py`
    gives both boards ERC 0, DRC 0, 0 unconnected and schematic parity.
    The KiCad design files come out byte-identical; the exports differ
    only in their dates (restored to the committed files).
  - Freerouting 1.9.0 runs under the local `xvfb-run`. It routes both
    DSNs (carrier 11 vias against the committed 10), so the committed
    `routed.ses` files stay the routing of record, as before.
- **Still working:**
  - FreeCAD 1.1.1 (snap) and Blender 5.2.2 (snap);
  - the project-local arm-none-eabi-gcc 13.2.1;
  - the PlatformIO and Blender MCP environments (uv Python 3.10).
