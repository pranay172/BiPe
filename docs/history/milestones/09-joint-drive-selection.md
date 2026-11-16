# M9 joint drives — revised selection

Choose **RO40 Lite + 4:1 202-2MGT-06 belt + moteus-c1**, with separate
rotor and **actual output-shaft AS5047P absolute SPI encoders**. Output sensing
is after the slip interface, so startup angle remains known after a fall.
Supplier values and calculated/estimated quantities are distinguished below.
Prices are USD, observed 2026-10-04; fabrication, tax and shipping are extra.

| Option | Output continuous / peak, Nm | Motor / drive mass | Motor reflected inertia, kg m² | Lash | Supply / torque interface / startup | Price basis |
|---|---:|---:|---:|---|---|---|
| [RO40 Lite](https://www.cubemars.com/product/ro40-kv140-lite-without-hall-frameless-torque-motor.html), 4:1 belt | **0.72 / 2.16**, η=0.90 calculation | 60 g / **~128 g CAD hardware** with driver, two encoders, housing, reduction/clutch and motor bearings; harness/fasteners and passive bypass counted separately in robot ledger | **0.000195**, 122.11 g cm² ×16 | Belt lash unpublished; **0.11–0.45°** assumed qualification range; journal clearance also nonzero | 24 V, 6S; moteus FOC current/torque over CAN-FD; independent output absolute SPI | Motor **$51.90** + driver **$69**, custom mechanical/sensing parts extra |
| [GL30 KV290](https://www.cubemars.com/product/gl30-kv290-gimbal-motor.html), 7.5:1, two belt stages | 0.54 / 1.89, η=0.90 total calculation | 41 g / **~120 g estimated**, no detailed CAD | 0.000136, 24.2 g cm² ×56.25 | Unpublished belt lash; two stages need measured nonzero range | Rated **12 V**; moteus FOC/CAN-FD + output absolute SPI; regulated bus or different pack needed | Motor **$50.99** + driver **$69**, second stage/regulator extra |
| [AK40-10 V3 QDD](https://www.cubemars.com/product/ak40-10-v3-0-kv170-robotic-actuator.html) | 1.3 / 4.1, supplier | 190 g integrated, external bypass extra | 0.000974, 97.35 g cm² ×100 | **0.3°**, supplier | 24 V; CAN MIT torque/current control; dual 16-bit magnetic encoders with absolute output | **$135.90**, [manufacturer store](https://store.cubemars.com/products/ak40-10-v3-0-kv170) |

RO40 retains useful continuous torque margin, one reduction stage and the
approved wheel-drive motor/driver/6S family. GL30 saves core mass but adds a
stage and 12 V domain; AK40 is too heavy. The selected hardware exceeds the
100 g/joint aspiration. This is a custom drive candidate, not an assembled
catalog actuator.

## Computed demand and belt assumptions

`python -m bipe.m9_sizing` uses actual CAD body COMs/tensors, finite-difference
kinetic/potential energy derivatives, a **2 kg** design mass including reserve,
loop virtual work, rotor/gear inertia and worst dry journal friction. It covers
45–118 mm height and ±10 mm fore-aft workspace. Measured demand is approximately
**0.51 Nm static, 1.49 Nm pulse, 0.53 Nm crouch and 0.55 Nm maneuver**. Crouch is
80 mm/s at 0.5 m/s², reaching 2.89 rad/s at a joint; maneuver acceleration is
2 m/s². The pulse combines 20 N chassis force and 2g additional vertical
acceleration. Exact measured values are in
[load-screen.json](../../../artifacts/milestone-9/cad/load-screen.json).

[Gates data](https://www.gates.com/content/dam/documents-library/catalogs/light-power-and-precision-manual.pdf)
Formula 1 gives **60 N per joint belt strand** at 0.6 Nm motor peak; this causes
110 N radial pull and 0.239 Nm worst dry journal friction before leg loads.
The 20T pulley has 7.34 teeth engaged, and its low-speed 6 mm rating supports
0.6 Nm transient and 0.2 Nm continuous at service factor 2. Both pulleys have
flanges and 7.25 mm faces. Table 6 fiberglass elongation gives stiffness proxies:
2MGT6 EA≈16.9 kN and 3MGT6 EA≈26.7 kN. Scale 0.2–1.2 for tooth/contact and
assembly uncertainty: hip **102–611 Nm/rad**, wheel **183–1097 Nm/rad**. Assumed
50–200 µm tangential lash gives hip **0.11–0.45°**, wheel **0.075–0.30°**.
These ranges must enter the later physical model; neither is measured lash.
Installed center distances include 60 N hip / 40 N wheel pretension elongation.
Exact tooth grooves, tension-adjustment machining and retention tolerances
remain manufacturing details; the CAD uses conservative tooth-stock envelopes.

## Lean fall load path and its limits

Opposed **GFM-1214-12 front / GFM-1214-20 rear** plain journals have centers at
22 / 39.5 mm. The outer rod hinge loads them at 54.5 mm: **17.5 mm span and
15 mm overhang**, including the covers. Hardened running sleeves surround
hollow 7075 aluminium shafts (minimum 11 mm aluminium OD / 6 mm bore).
The connected cassette uses an 18/14 mm tube and a 28×2.5 mm fork web.
[Igus data](https://www.igus.com/ContentData/Products/Downloads/iglide_G300_FM_USen.pdf)
gives 80 MPa pressure at 20°C; use conservative **35 MPa** hot allowable,
μ=0.08–0.15 and nonzero press-fit/shaft clearances. Steel sleeves receive no
composite bending credit. The [7075 source](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1028/Kaiser_Aluminum_7075_Rod_and_Bar.pdf)
is typical property evidence; **435 MPa is a required certified stock allowable**,
not an assumed guaranteed minimum from that table.

The fall screen uses CAD COM-to-ground drop, 0.3 m/s translation, 1 rad/s pitch
and a triangular **5 mm assumed stopping travel**, on one leg at worst linkage
leverage: approximately **3.00 J, 1.22 kN impact, 1.48 kN hip reaction**.
Gross-section factors are about **2.98 journal pressure, 2.20 shaft bending,
2.03 fork web and 2.80 cassette tube**. These satisfy the preliminary ≥2 sizing
screen. Fastener/notch stresses, shell/covers, stopping travel and fatigue are
not proven. [HK0810](https://koyo.jtekt.co.jp/en/products/detail/?pno=HK0810)
needle hinges (4.07 kN static, 4 g) replace the under-capacity 688 option.
The aluminium wheel axle was under-capacity in a central tire-contact fall
screen. **8/6 mm hollow 17-4PH H900 axles** add 8 g for the pair and give **2.39
bending factor**, with a required certified 1000 MPa allowable below
[Carpenter's typical H900 data](https://www.carpentertechnology.com/hubfs/7407324/Material%20Saftey%20Data%20Sheets/Custom%20630%20%2817-4%20PH%29.pdf).
The 608 wheel-bearing static factor is **2.08** for central radial contact;
lateral edge contact and combined axial/radial loads remain the full fall gate.

A two-face steel/iglidur slip interface and
[SCHNORR 008900 spring](https://www.schnorr-group.com/fileadmin/4_Downloads/Brochures/SCHNORR_Produktbroschuere_EN_2024-02.pdf)
route radial impact through the cassette and limit transmitted torque.
**Calibrate release to 1.7–2.0 Nm** and cap normal commanded torque below
measured release; 2.16 Nm is the calculated motor/reduction capability. Catalog
friction/preload alone does not establish release. Unisolated fall joint torque can reach ~67 Nm. The input
pulley and crank are separate rigid bodies for later slip modeling. The
mandatory complete M9 fall/impact gate remains pending.

## Electronics scope

All six **8.9 g, 38×38×9 mm moteus-c1 boards** are inside the chassis.
The [c1 AUX2 pin table](https://mjbots.github.io/moteus/reference/encoders/#c1n1x1-pins-aux2abs)
supports external rotor SPI sensing; its onboard AUX1 encoder is unused.
Four output encoders use host MCU SPI after the clutches; wheel output uses
RLS quadrature. Later firmware needs CAN-FD/current scaling, AUX2 rotor
calibration, output zeroing, watchdog/fault handling and power arbitration.
At Kt=0.068 and η=0.90, the ~1.49 Nm joint pulse needs ~6.1 A phase current,
versus ~2.1 A for static hold. [c1](https://mjbots.com/products/moteus-c1) is
rated 5 A continuous without heatsink / 20 A peak; motor rated current is
2.85 A. Pulse duty, thermal paths, simultaneous bus/regen limits and 6S voltage
qualification require the next phase; M4 budgets do not automatically transfer.
