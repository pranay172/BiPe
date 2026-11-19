# Hardware build plan

Plan written 21 October 2026, for roadmap phase 4 (hardware build). It
turns the M10 CAD and the M11 electronics into a parts list with prices
and lead times, a staged build, and a single-leg test rig, so the build is
ready when the simulation is.

Refreshed 29 October 2026 against main after the M12 merge (phases C, D
and F, e50fdf7) and the M13 `settle` policy (PR #3, phase D in
progress). The parts list barely moves: no new compute is needed on the
robot. The changes are in what stage 0, the rig and the tethered robot
must test.

Nothing here is bought, built or powered. Buying starts after the public
release and a few improvements (user, 21 October 2026), so this is a
planning list. Prices marked "checked" were read
from the seller's page on 21 October 2026; the others are M11 records or
estimates, as the BOM says per line. Retail pages change, so every price is
rechecked at the procurement freeze. The design it builds is a layout study
plus a validated simulation under stated assumptions; it is not
hardware-validated.

## Decisions this plan respects

- M11 parts (user, 18–19 October 2026): six moteus-c1 on the 6S pack; RO40
  Lite kept with its stock risk recorded; custom Teensy carrier; thin-wall
  PTFE harness with 10 mm service loops; power switch on the rear wall.
- M12 phase C (user, 21 October 2026):
  - the Raspberry Pi Zero 2 W stays the vision host, and a real board is
    bench-timed before M12 phase E;
  - Camera Module 3 Wide;
  - waist tag with the camera re-pitched to about 42°. The head pod is
    redesigned in phase E, so the head is the last part ordered.
- M12 phases D and F (user, 22–24 October 2026): the follower runs in the
  Teensy's supervised C core and the Pi sends tag measurements. Phase F
  adopted a 74° view gate (`config/following-rendered-m12.json`). It
  exists only in the simulated link, so the real Pi pipeline must
  implement it, and it was chosen in-sample, so it is checked on real
  footage of a turning person (review follow-up, 29 October 2026). The
  side-light losses and the camera's auto exposure are left to hardware.
  Phase E (Pi link, carrier revision, head re-pitch, real-board timing)
  is the hardware phase and waits for the purchases.
- M13 (user, 29 October 2026): `settle` is the learned policy. It is
  exported as ONNX with a portable C forward pass, and phase D estimates
  its Teensy 4.0 inference time. It is planned to run on the Teensy
  (where it runs is decided before hardware), so it adds no board. The
  classical controller stays the safety net.
- Roadmap: printed and machined parts are outsourced; a fit-check set is
  printed first; single-leg rig before the tethered robot.

## Bill of materials

The full list is [bom.csv](bom.csv): one row per item, with quantity per
robot, spares, unit price, the price basis, the source and the lead time.
Quantities come from `config/robot.json` (`m10.internals`,
`m10.structure`, `m10.mass_properties`) and the M11 board files.

Summary (USD, single units, before shipping and import duty):

| Group | One robot | Spares, minimum-order extras and tools | Main lines |
|---|---|---|---|
| Actuators | 463 | 157 | 4 RO40 Lite, 2 GL40 KV70 (+1 of each) |
| Drivers | 453 | 69 | 6 moteus-c1 (+1), mjcanfd-usb-1x |
| Compute and camera | 86 | 36 | Teensy 4.0 (+1), Pi Zero 2 W, Camera Module 3 Wide, microSD |
| Boards | 78 | 172 | carrier, carrier revision for the Pi link and power PCBA (5-off minimum), 6 encoder boards |
| Power | 70 | 181 | pack (+1), BEC, fuse, switch; charger and bench supply as tools |
| Mechanical purchased | 259 | 93 | bearings, belts, pulleys, dowels, inserts |
| Machined | 658 | 106 | 7075 shafts, shins, hip brackets; 6061 bulkhead; motor housings |
| Printed | 285 | 0 | PA12 shells and links, TPU tires and pads, fit-check set |
| Harness | 175 | 0 | Alpha Wire 2843/7 and 2841/7 spools, connectors |
| **Total** | **≈2,530** | **≈810** | |
| Single-leg rig | | ≈300 | listed in the rig section |

So the whole programme is about $3,600 with the rig and the spares. Half of
the value is estimates (machining, printing, PCBA, commodity parts), so
plan on ±30% until the quotes are in.

Supply notes:
- **RO40 Lite** is the long pole. RobotShop lists it at $74.47 with no stock
  shown; OpenELAB lists €55.60, shipping from China in 5–10 working days;
  M11 found it out of stock at both on 18 October. Check CubeMars direct
  as well. Buying all five early is the main supply hedge.
- **GL40 KV70** without encoder is $82.48 at RobotShop. The CubeMars store
  shows only the KV210, sold out.
- **moteus-c1** and the CAN-FD adapter: about 1,100 in stock each, shipped
  in one business day.
- **Pi Zero 2 W** is often sold out at resellers. It is needed first (stage
  0), so order it with the actuators.
- **Camera Module 3 Wide:** about $35 list (£33.60 at the Pi Hut); some
  resellers are out of stock.
- **PCBA** (JLC): 5 is the minimum order, so the extra boards are spares.

### What the CAD does not yet define

Three items in the BOM are not designed yet, so they cannot be ordered
from the repository as it is:
1. **RO40 module.** The CAD carries each RO40 as a Ø49.3 × 15.5 mm envelope
   with a 90 g catalog-plus-assumption mass (`m10.mass_properties`). The
   housing, rotor hub, motor shaft, bearings and encoder mount need a
   detail design. The M9 joint-drive notes (`m9_cad.parameters.joint_drive`)
   are the starting point.
2. **Encoder boards.** Six small AS5047P boards (four on the RO40 rotors,
   two on the hub axle tubes, 14 mm round in the CAD). M11 laid out only
   the carrier and the power board. They are a small KiCad job with the
   same generator pattern; MA732 stays the fallback chip.
3. **Mounting details** already listed as M11 open items: bosses for the
   carrier, BEC, Pi and power board; thigh-lid screws; harness strain
   relief at the shaft ports.

The `manufacturability.md` report predates phase D4 in places (MR128 and
6700 bearings, PA12 shin and hip bracket). The BOM follows the current
config: 688A, 6800, and 7075 shins and hip brackets.

## Staged build

Each stage tests one group of model assumptions, so a sim-to-real gap is
found on a bench rather than on a falling robot. A stage starts when the
previous one meets its exit.

The M18 bench tests ([bench-tests.md](bench-tests.md), B1 to B10) fit into these
stages as their parts arrive: B2 to B4 with the first calibrated drive, B6 to B9 once
the Teensy and IMU are on the chassis, B1, B5 and B10 on the assembled robot. Their
results go into `config/hardware-measured.json`, and the fidelity suite rerun with
`--measured` before the first balance.

| Stage | Content | Exit |
|---|---|---|
| 0. Long lead and bench timing | Order the actuators, drivers, adapter, Pi Zero 2 W and Camera Module 3 Wide. On the Zero 2 W: pupil-apriltags at 1152 × 648 with region-of-interest tracking over 2 cores, the full-frame re-acquisition, the 74° view gate, and the board temperature after 30 min. Record real footage of a person walking and turning with the waist tag, in normal, dim and side light, with auto exposure on. Spin one moteus-c1 with one GL40 from the bench supply: `moteus_tool --calibrate`, no load | Zero 2 W rate and latency against the 30 Hz, 0.12–0.15 s tracking and 0.25–0.33 s re-acquisition design values; the gate checked on the footage; one drive calibrated |
| 1. Detail design and fit checks | RO40 module, encoder boards, mounting bosses, rig parts. Print the fit-check set: bearing-seat coupons (printed 0.1 mm under and reamed), insert bosses, one thigh with its lid, one hub | Coupons fit; drawings and quotes for the machined parts; the recomputed mass stays under the 2.15 kg ceiling with the 0.15 kg reserve |
| 2. Board bring-up | Carrier and power boards from JLC. Current-limited supply, no motors: LM5060 start-up (time and MOSFET temperature), the 5 V rail, the Teensy VUSB–VIN cut and CAN3 links, IMU WHO_AM_I and axis map, CAN-FD to one moteus | Start-up within the 42 ms design and the 88.6 ms timeout; IMU gravity on body +z; driver discovery on all ids present |
| 3. Single-leg rig | One side's drive module and one leg on the rig below | The rig tests below pass, and the plant values they measure are recorded |
| 4. Tethered robot | Full assembly with a dummy head of the same mass. Overhead tether, then balance in place, then free driving at 0.3 m/s | Balance and 0.3 m/s driving on the tether; sim versus measured traces on the same commands |
| 5. Envelope and phase E | After the plant is updated from stages 3–4: the M12 speed envelope on hardware; M12 phase E (head redesign at 42°, the Pi link and carrier revision, then a reduced phase F rerun if the latency budget or plant changes); following on the tether first | Separate plans when they start |

Stage 4 detail:
- **Torque lock.** The M11 image keeps `BIPE_HARDWARE_QUALIFIED` at 0. The
  rig produces the evidence the lock asks for (signs, offsets,
  calibrations, measured timings). Lifting it is the user's decision at the
  start of stage 4.
- **Tether.** A cord from an overhead frame to a strap round the body, with slack set so the cord catches the robot at about 15° of
  pitch, before any shell contact. The fall analysis is a simulation result
  only, so no untethered falls until it is checked.
- **Order.** Sit pose (hip 0.054 m) → stand → hold → crouch cycle → turn on
  the spot → 0.3 m/s straight → turns. The supervisor's stop and the
  hardware switch are tested at each step before moving on.
- **Controllers.** The classical controller first, as the baseline and
  safety net. The M13 `settle` policy follows behind the same supervisor
  and envelope gates, starting with balance in place, once phase E of M13
  has decided its adoption and the bench has timed its forward pass with
  the DWT counter.
- **Then** the measured values go into `config/robot.json`, the
  acceptance runs again in simulation, and the robot is retested ("update
  the plant, not the gains", roadmap phase 5).

Rough spend by stage: stages 0–3 need all the actuators, drivers, compute,
boards, tools and one side's mechanical parts, about $2,600 with the rig.
Stage 4 adds the second side, the shells and the harness, about $1,000.
The head waits for M12 phase E.

## Single-leg test rig

Purpose: measure, on one hip–knee–wheel chain, everything the plant
assumes about the drive train, before the robot can fall: signs and zeros,
the torque path, friction and lash, lag, heat, belt tension and the
harness.

### Layout

```
        2020 extrusion tower, 500 mm
        │ MGN12 rail, vertical, 400 mm
        │   ┌──────────────── carriage plate (6 mm aluminium)
        │   │  bulkhead + one side's RO40 pair, shafts, belts, hip bracket
        │   │  rig hip pod (printed, holds the outer 6704)
        │   │  ballast on M6 rods
        │   └── linear potentiometer → hip height
        │        │ thigh
        │        └─ shin
        │             ◯ GL40 wheel
   ═════╧═════════════════════════  ground: one of three modules
```

- **Frame.** 2020 extrusion: a 500 mm tower on a 300 × 300 mm base.
- **Carriage.** An MGN12H block on a 400 mm vertical rail. A 6 mm aluminium
  plate carries the real 6061 bulkhead, so the drive module is the
  production part. One side is populated (thigh and knee RO40, both 4:1
  belts, concentric shafts, the 7075 hip bracket); the other side carries
  ballast. A printed rig hip pod holds the outer 6704, which the body's
  hip housing holds on the robot.
- **Pitch is fixed** by the rail. At a given posture, the leg's static
  torques therefore match the robot's (nominal 0.42 N·m thigh, 0.37 N·m
  knee) as long as the wheel load matches.
- **Wheel load.** Ballast is trimmed until the load cell reads the robot's
  static wheel load, 1.815 kg / 2 × g = 8.9 N. A 1.5× setting (13.4 N)
  gives the overload case.
- **Travel limits.** A collar on the rail stops the carriage at the sit
  height (hip 0.054 m) with a TPU bumper; a top bumper limits the extended
  pose.
- **Ground modules** (swapped on the base):
  1. **Load-cell plate.** A 3 mm plate on a 5 kg single-point load cell
     (HX711), with a removable wheel chock: static loads and the crouch
     cycle.
  2. **Roller dyno.** The wheel drives a Ø72 mm roller on two 608 bearings
     with a steel flywheel, Ø100 × 15 mm (1.16 × 10⁻³ kg·m²). That matches
     half the robot's translational inertia at the wheel, 0.908 kg ×
     (0.036 m)² = 1.18 × 10⁻³ kg·m², at 1:1, so wheel acceleration tests see
     the right load.
  3. **Torque arm.** A clamp on the wheel, or on a locked joint, with an arm
     onto a 1 kg load cell: stall torque against current.
- **Instruments.** A spare Teensy 4.0 logs the potentiometer, both load
  cells and two thermocouples (MAX31856), with a sync line to the logging
  laptop. moteus reports position, velocity, torque, current, voltage and
  temperature. A digital inclinometer sets the joint zeros.
- **Power and safety.** A 30 V 5 A bench supply set to 24 V with a current
  limit, through a DC-rated mushroom stop that cuts the drive supply. The
  pack is only used in the stage 2 power-path tests. Safety glasses for
  the belt and fall tests.
- **Control.** First from a laptop through the mjcanfd-usb-1x
  (`moteus_tool` and the `moteus` Python package already in the test
  lock); then through the real carrier and power board, so the rig also
  times the M11 bus schedule.

Rig parts (estimates, about $300):

| Item | USD |
|---|---|
| 2020 extrusion, brackets, base | 45 |
| MGN12H rail 400 mm and block | 25 |
| Carriage plate (cut 6 mm aluminium) | 30 |
| Printed rig hip pod and adapters | 20 |
| Ballast plates and M6 rods | 15 |
| 150 mm linear potentiometer | 20 |
| 5 kg and 1 kg load cells, two HX711 | 25 |
| Ground plate and chock | 15 |
| Roller dyno: roller, 608 bearings, flywheel | 40 |
| Two MAX31856 boards and K thermocouples | 40 |
| DC-rated mushroom stop and box | 20 |
| Bumpers and collar | 5 |

The rig's actuators, drivers, bearings, belts and leg parts are the robot's
own, so they move into the robot in stage 4. The spare RO40 and GL40 let
the rig keep running endurance tests after that.

### Tests

Each test names the plant value it checks or replaces (`m10.physics`
unless noted). Results go in a rig record with the configs and traces, in
the same way as the simulation outputs.

| # | Test | Mode | Measures | Plant value | Pass or record |
|---|---|---|---|---|---|
| 1 | Signs and zeros | leg in air | output sign and offset per joint against the inclinometer | commissioning values in the moteus configs | written into `artifacts/m11/firmware/moteus/` |
| 2 | Encoder integrity | leg in air, phases driven | AS5047P errors and noise at 2 MHz with the phase leads energised | `aux2.spi.rate_hz` 2 MHz | no CRC or parity errors over 10 min |
| 3 | Friction and lash | leg in air, slow sweeps and reversals | Coulomb and viscous torque; lash at each joint | `hip_motor` coulomb/viscous, `belts` lash 0.25° hip / 0.5° knee (ranges 0.11–0.45° and 0.23–0.9°) | inside the ranges, or new values |
| 4 | Torque constant | torque arm | stall torque against q-axis current | RO40 0.068 N·m/A with 4:1 and 0.9; GL40 0.15 N·m/A | within ±10%, or new values |
| 5 | Lag | leg in air and dyno | current and position step responses | FOC 0.8 ms; the 25 ms wheel-lag variant that fails two acceptance cases | which lag is real |
| 6 | Belt stiffness and tension | torque arm, phone frequency meter | span tension; twist under torque | hip 62.5 N per span, stiffness 119–713 N·m/rad; knee 120 N (assumed) | inside the ranges |
| 7 | Static hold and heat | load-cell plate, nominal pose, 20 min | winding temperature rise, K/W | the 23 W quiet hold; thermal path bounds | temperature levels off well below 125 °C |
| 8 | Crouch cycle | load-cell plate | tracking error, peak current, bus current | phase A currents (3.41 A peak hip) | currents within 20% of the simulated case |
| 9 | Wheel drive | roller dyno | acceleration against torque, speed at the voltage limit | GL40 limit 18.24 V, no-load 121 rad/s | inside 10% |
| 10 | Harness flex | leg cycling over the knee range | continuity through 10⁴ cycles; rub on the knee pulley flange | M11 harness check (strain 0.83% / 0.52%) | no break or wear marks |
| 11 | Bus timing | carrier and power board | driver turnaround and compute with the DWT timers | timing budget 3.44 ms of 5 ms (0.25 ms turnaround assumed) | under 5 ms |
| 12 | Power path | power board, bench pack | LM5060 start-up MOSFET temperature, inrush | 22.5 W linear peak, 18–33 ms ramp | inside the design |
| 13 | Sensor channels | carrier, all modes | encoder quantisation, packet timing and jitter, finite-difference speed noise, IMU noise and bias | the robot channels M13 trains on (5 ms packets, quantised encoders) and `config/sensors-m10.json` | inside the modelled values, or new values for both plants |
| 14 | Policy forward pass | carrier, leg in air | `settle` forward pass time on the Teensy, inside the 5 ms tick | M13 phase D inference estimate | under its share of the 0.5 ms compute bound |

Tests 3 and 6 matter more since M13: `settle` recovers only 4 of 8 belt
cases, and its training plant has no lash and rigid hip belts, so the
measured lash and belt stiffness decide whether the policy needs
retraining with them. Test 13 matters for the same reason: M13 found its
largest sim gaps in the sensor interface, not the physics.

Tests 1–4 need no load and come first. Test 10 runs unattended overnight
with the supply current-limited.

## Decisions for the user

These carry real trade-offs. They are open until buying starts, after the
public release:
1. **Order scope.**
   - (a) Everything for one robot plus spares and the rig at once, about
     $3,600.
   - (b) Only stage 0 and one leg's actuators and drivers now (about
     $550), the rest after the rig. This keeps the RO40 stock risk.
   - (c) **Recommended:** all actuators, drivers, the Pi and camera now,
     plus one side's mechanical parts, the boards and the rig (about
     $2,600). The second side, shells, harness and head wait for the
     rig results, which may change them.
2. **Print process.** The strength screen uses SLS PA2200 (EOS), weakest
   Z 42 MPa × 0.6. JLC's MJF PA12 is cheaper ($0.275/g) and fast. Its
   properties must be checked from HP's datasheet against the screen
   before it replaces the basis, and the fit-check coupons test it.
   Recommended: MJF PA12 for the fit-check set, and decide for the
   structural parts once the coupons are in.
3. **Tools.** The plan counts a 6S charger and a 30 V 5 A bench supply.
   Leave them out if you already have them.

The detail design in stage 1 (RO40 module, encoder boards, bosses) is
design work in the repository and gets its own plan when it starts.

## Open risks

- RO40 supply, as above.
- GL40 bearings are unpublished; the M10 screen lists the rating each would
  need, to confirm with CubeMars.
- The sideways-fall structural shortfalls (hip 6704 at 0.48–0.52), signed
  off in M10, are why the tether stays on until the falls are checked on
  hardware.
- The AS5047P is "not recommended for new designs"; the MA732 is the
  fallback.
- The Pi Zero 2 W timing, the view gate and the lighting behaviour are
  simulation estimates until stage 0.
- MLCC derating, pack resistance and moteus logic power are M11
  assumptions that stage 2 measures.
