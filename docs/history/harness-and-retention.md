# Harness and retention reference

M5 digital detail, 1 October 2026. The adopted CAD/configuration is unchanged.
These definitions support review and future detailing; they do not establish
cable flexibility, tooling access, insulation strength or retention loads.
See the [native pin inventory](../../artifacts/milestone-5/inventory/harness.json),
[reference drawings](../../artifacts/milestone-5/drawings/reference-drawings.pdf)
and [hardware qualification gates](hardware-qualification.md).

## Electrical harness

Pin numbers below come from the native board pads, not connector appearance.
The mating connector's view/numbering still requires its manufacturer's drawing.
`power` means the integrated power PCB; `carrier` means the controller carrier.

| Loom | Pin mapping | Construction and service |
| --- | --- | --- |
| Control | power J3.1-4 to carrier J4.1-4: GND, RUN_HEARTBEAT, STOP_N, +3V3 | Keyed JST PH 4-way, exact mating housing/crimp selection held; disconnect before roof removal |
| Measurement | power J9.1-6 to carrier J9.1-6: GND, PACK_MON, BUS_MON, CURRENT_MON, DUMP_ACTIVE, START_OK | Short 24 AWG loom; power-board joints must stay within 2 mm height; individually label and insulate |
| Logic supply | power J4.1 LOGIC_BUS to carrier J3.2; power J4.2 GND to carrier J3.1 | Remains upstream of actuator disconnect; pin 1 is not positive on carrier J3 |
| Legacy bus sense | power J2.1 ACT_BUS to carrier J3.3 ACTUATOR_BUS_SENSE | Separate protected sense conductor; not a logic supply or a substitute for J9 measurements |
| Pack | Protected pack P+ to power J1.1 RAW, P- to J1.2 GND | Exact BMS/fuse, conductor, polarity and fault ratings held |
| Motor power | power J2.1 ACT_BUS / J2.2 GND to six separately routed power pairs | 18 AWG modeled allocation; real connector pinout and DC distribution/fault qualification held |
| Precharge | power J7.1 PACK / J7.2 PRE_D to external R574 | LTO050F22R00JTE3; insulation/lead forming/thermal mount held |
| Dump | power J8.1 ACT_BUS / J8.2 DUMP_D to four parallel external RD1-RD4 | Same 22 ohm part per branch; keep bank wire clear of ceramic interfaces and seal |
| Roof sensing | carrier J10.1 DUMP_NTC, J11.1 PRE_NTC; respective pin 2 GND | Two external NTCS0603E3103FLT sensors; mount at the specified dump/precharge cases; error including lag <=1 C requires calibration |
| Stop | power J5.1 +3V3 through a normally-closed contact to J5.2 STOP_CLOSED | Independent hardware path; exact switch, response and actual driver disable are unqualified |
| Arm | Two isolated NO contacts operated together: power J6.1 +3V3 to J6.2 ARM_RAW; carrier J5.1 GND to J5.2 ARM_N | Opposite signal polarities. Do not join ARM_RAW to ARM_N or use one shared contact. Carrier J5.3 is +3V3, unused by this dry-contact pair |
| Startup programming | power J10.1 START_UPDI, .2 GND, .3 +3V3 | Inhibited service interface; verify external programmer voltage/back-power before connection |

The arm contact arrangement is an explicit wiring requirement found during
native pad reconciliation. No exact switch or new mass has been adopted. The
startup monitor and main application still require separate commissioning.

CAN uses a single linear electrical bus: left wheel - left knee - left hip -
carrier J1/J2 junction - right hip - right knee - right wheel. Both carrier
connectors have pin 1 GND, 2 CAN_L, 3 CAN_H. With the carrier inside the chain,
its JP1 termination is open; the two wheel-end terminations must be selected and
verified. Exactly two qualified end terminations are required. The real motor's
integrated termination and XT30PW(2+2) pin assignment remain unknown. Do not
infer those pins from the CAD cylinder or a generic AK-series protocol drawing.

## Routing and length reference

Use the existing rear bus/side logic loom reserves, then route motor power and
twisted CAN along the inboard links. Keep connectors stationary on their parent
link. Flexible loops cross hip and knee pivots; clamp both sides of each moving
span, away from rotating output annuli, flush screw seats and tires. Separate
roof resistor leads from analog signal loops and insulate every formed capacitor
lead. Disconnect looms before lifting their assemblies; do not use wires as
lifting handles.

The accepted mass ledger allocates individual conductor length, not finished
cable cut length: 3.0 m of 18 AWG at 1.9 mm assumed OD, 0.2 m of 14 AWG at 3 mm
OD, and 1.4 m of 24 AWG at 1.1 mm OD, plus the explicitly added 60 mm capacitor
wire. These quantities include slack estimates. Two conductors in a cable do
not each get the complete pair allowance again. Connector halves on boards and
mating plugs in the harness have separate allocations.

For a first flex trial, reserve a **10 mm bend radius** for each 1.9 mm conductor;
this is a proposal, not the wire manufacturer's minimum. A 135-degree sweep
at that radius alone needs 23.56 mm of arc; add measured attachment travel and
assembly slack to obtain cut lengths. A four-wire bundle cannot be justified
by the earlier 3 mm bundle example: two 1.9 mm power wires already span 3.8 mm
side by side. A 4 mm bundle in a 7.8 mm gap leaves 1.9 mm per side, below the
2 mm target. Route individual/staggered conductors or revise the layout, then
perform a swept cable/connector check across all 220 CAD poses. Current proxy
volumes do not certify that result. All cut lengths remain **HOLD**.

## Retention details and holds

| Interface | Existing nominal provision | Detailed next design and verification |
| --- | --- | --- |
| Motor outputs | 18 M2.5 x 10 plus 0.5 mm washers; 4 mm adapter + 3 mm plate gives 2.5 mm engagement | Drawing limit 3 mm thread depth; verify actual blind-hole depth, tolerance stack, locking method, strip strength and driver tool access |
| Hip stators | 6 M2.5 x 8 plus washers; 3 mm face gives 4.5 mm engagement | Drawing limit 5 mm; no released tightening torque |
| Knee/wheel stators | 12 ISO 7046-2 Phillips countersunk M2.5 x 6 | DIA 5.1 x 1.2 mm 90-degree seats; selected 4.7 mm head candidate, 1.8 mm remaining plate. Other head standards are not interchangeable |
| Fork | Four M3 x 12 screws, eight washers, four nuts | 6 mm ear/plate stack; lock nuts and verify tools, screw projection, load and fatigue |
| Bearing | Two nominal 8 x 22 x 7 mm 608-2Z seats | Shaft/housing fits and axial stack unqualified. Develop an end washer/shaft retainer and a removable fork outer-ring retainer; support the loaded ring and avoid locking both rings into unintended preload. Retainer solids, threads and new mass must be reconciled before adoption |
| Tire | 80/60 x 24 mm annulus on hub | Select positive flange/retention or a qualified interference/adhesive interface; check slip at finite braking torque, creep and balance. Current matching diameter is not a retention method |
| Pack | 70 x 64 x 50 mm protected-pack allocation, tray and strap/pad budget | Two 6 mm wide, <=0.5 mm strap proposals over the pack at X +/-25 mm; anchor eyelets at existing tray mount positions Y +/-39 mm. This changes the screw stack: current M2.5 x 8 engagement must be reviewed. Keep buckles outside the carrier sweep, insulate anchors, verify compression/abrasion and the pack supplier's allowed load. Slots/eyelets are not modeled or released |
| PCB supports | Two support bars for each board; four M2 x 6 screws/washers each in the ledger | Final threaded inserts/standoffs/insulators are not modeled. Match native H1-H4 hole centres; do not assume proxy bars establish torque, clearance or strain relief |
| Roof | Four DIA 2.2 holes; 3 g combined mount/seal allowance | Select four insulating shoulder/insert mount sets and a removable perimeter seal. Verify clamp force, thermal growth, dielectric clearance and screw access. Mount sets are allowances, not exact fasteners |
| Roof resistors | Five M3 x 10/washer/nut sets in mass ledger | Current ceramic blocks have no detailed bolt feature. Match actual resistor mount drawing, orient ceramic interface toward roof, control contact <=0.1 K/W, insulate leads and fit both thermistors. Validate pulse/SOA and <=0.2 W other roof heat |
| Front/eyes | Four M3 x 30, washers, rear insert pockets; two eye discs | Calibrate print/insert retention; choose eye attachment and verify camera reserve/face removal. No strength follows from PETG density |

These are reviewable interfaces and proposals, not installed hardware. Additional
retainers, strain relief and switch selection compete with a **0.481 g** global
estimate margin and **0.873 g** unresolved harness/mount allowance. Confirm each
item's budget membership and measured mass rather than adding it twice or
claiming it fits. A practical hardware phase should revisit the 2.5 kg ceiling,
284 mm width and six-actuator cost before detailing these physical interfaces.

The complete native BOM includes generic capacitor/resistor specifications and
external-module adapter footprints. Exact MPNs, effective capacitance, adapter
pin coordinates, mounting holes and board fabrication stack availability remain
holds. Placement CSVs are raw board reference outputs: wired capacitor C527,
module adapters, terminals and remote sensors need assembly exceptions; they are
not an unattended pick-and-place program.
