# M3 mechanical candidate and fabrication notes

This is a component-based digital prototype for level-floor balancing, driving, turning and crouching. Native CAD, STEP and simulation agree under declared mass and actuator assumptions. These notes are preliminary manufacturing interfaces, not a fabrication release or hardware qualification. Dimensions originate in `config/robot.json`; the CAD boundary converts SI values to millimetres.

## Architecture and packaging

Six provisional CubeMars AK40-10 V3.0 KV170 actuators drive two serial 90 mm legs and two 80 mm wheels. Eyes face +X; knees fold toward −X. The actuator has a 53 mm body diameter, 40.2 mm overall axial length and a published 190 g mass. [Manufacturer product](https://www.cubemars.com/product/ak40-10-v3-0-kv170-robotic-actuator.html) and [exact mounting drawing](https://www.cubemars.com/data/cms/202607/ak40-10-v3-0-kv170-2d-drawing.pdf).

The body is 114 × 94 × 96 mm, the hip spacing is 190 mm and wheel track is 242 mm. Real motor bodies extend inboard of their output faces; the wheel shaft receives outboard bearing support. This increases width over the M2 layout. The modeled-solid envelope at nominal unpitched 40° posture is approximately 144.85 × 284 × 260.89 mm. Unmodeled screw heads, connectors, cable bends and compliant tires can increase the hardware envelope.

Historical M3 estimated mass is 2.49223 kg, including six actuators, enclosure, aluminium structure, tires/hubs, two bearings, 326 g battery allowance, 90 g electronics, 80 g future camera and 80 g wiring/fasteners. The remaining 7.8 g against 2.5 kg is smaller than likely fabrication and procurement variation. The mass target therefore needs renewed review when actual electronics, connector and pack hardware are selected; no reserve has been silently removed.

All motor internal mass distributions use a uniform drawing-based surrogate. Their reflected rotor inertia is added separately as an ideal joint armature. This approximates the gearbox dynamics and omits detailed rotor/body coupling, gear compliance and free-play impacts. The supplier STEP contains open surfaces rather than solids; it is a reference, not a source of computed mass.

## Component and fabrication list

| Item | Quantity | Digital basis and proposed process |
| --- | ---: | --- |
| AK40-10 V3.0 KV170 actuator | 6 | Exact drawing-based envelope, published mass; integrated driver and dual 16-bit encoder claims. Provisional until exact firmware/electrical limits are confirmed. |
| Hip bracket | 2 | CNC aluminium, one connected face mount/brace/post solid; mounts the chassis load to each stator. |
| Upper/lower link plate | 4 | 3 mm 6061-T6 aluminium plate. Laser/waterjet rough profiles followed by finished holes, countersinks and edge deburring, or CNC throughout. |
| Hip/knee output adapter | 4 | Turned/CNC aluminium annulus, 4 mm axial length, 3 output bolts. |
| Wheel hub and integral shaft | 2 | Turned/CNC aluminium candidate, output adapter and rim; bearing seat finish and axial retention require production detailing. |
| Wheel support fork | 2 | CNC aluminium with a 3 mm crossbar, connected mounting ear, 7 mm outboard bearing land; finished bearing bore. Internal tool radii require drawing review. |
| SKF 608-2Z bearing | 2 | Nominal 8 × 22 × 7 mm purchased component, 12 g allocation. Bearing internal geometry is approximated as an annulus. |
| Tire | 2 | Custom TPU annulus, 80 mm outside / 60 mm inside diameter, 24 mm wide. Density 1200 kg/m³ and floor friction are assumptions; tread, retention and durability need trials. |
| Shell front/rear and eye discs | 1 set | Outsourced PETG printing; 2.5 mm shell wall, rounded corners, removable front, corner screw passages and rear insert pockets. |
| Battery tray | 1 | 2 mm aluminium, 84 × 84 mm, shell-tab mounting holes; pack pads and straps allocated. |
| Battery | 1 provisional pack | Six P28A cells, 6S1P, professionally assembled if hardware is pursued; allocated space/hardware are not a selected or qualified pack. |
| Electronics, camera, wiring/fasteners | allocations | Bounding boxes and declared masses, not selected boards or a wiring harness. |

For the 6061-T6 structure, [Hydro's alloy sheet](https://www.hydro.com/globalassets/01-products--services/extruded-profiles/americas/ena-resources/alloy-data-sheets/hydro_2019_data_sheet_6061.pdf) lists 240 MPa minimum yield for its documented extrusion products. The preliminary screen uses an assumed 80 MPa normal-stress allowance and 60 GPa modulus. These are engineering assumptions; the outsourced plate/billet supplier must certify the actual alloy, temper and relevant properties. Printed shell strength and insert pullout are not inferred from the aluminium calculation.

## Mounting and fastener interfaces

The exact drawing shows separate output and stationary housing bolt circles. Their angular patterns differ: output holes are 3 × M2.5 on PCD 27 mm at 30°, 150°, 270°; stationary front holes are 3 × M2.5 on PCD 47.5 mm at 0°, 120°, 240°. Output tapped depth is 3 mm; housing depth is 5 mm. The custom parts use 2.7 mm clearance holes. The output adapter's 15.2 mm bore is a clearance feature; it does not establish a precision pilot fit with an undocumented motor surface.

| Interface | Nominal hardware provision | Engagement/access |
| --- | --- | --- |
| Six motor outputs | 18 × M2.5 × 10 screws, nominal 0.5 mm washers | 4 mm adapter + 3 mm plate/disc + 0.5 mm washer leaves 2.5 mm thread engagement within the 3 mm drawing depth. |
| Hip stators to brackets | 6 × M2.5 × 8 screws, nominal 0.5 mm washers | 3 mm bracket + washer leaves 4.5 mm engagement within 5 mm. |
| Knee/wheel stators to link plates | 12 × M2.5 × 6 **Phillips countersunk** screws | 3 mm plate leaves 3 mm engagement. A 90° countersink, 5.1 mm mouth diameter and 1.2 mm depth reserves a flush seat for a 4.7 mm head. Ordinary raised heads would interfere with the rotating hub disc. |
| Fork to lower plate | 4 × M3 × 12 screws, washers and nuts | Two holes per side, 8 mm pitch, 6 mm plate/ear stack. Located above the central window and wheel envelope. |
| Front shell to rear shell | 4 × M3 × 30 screws and rear inserts | 25 mm front stack plus 0.5 mm washer leaves 4.5 mm engagement. Nominal insert pocket is Ø4.6 × 5 mm; adapt it to the purchased insert and print calibration. |
| Shell tabs to tray | 4 × M2.5 × 8 screws, washers and nuts | 4 mm tab/tray stack; nominal holes at X ±25 mm and Y ±39 mm. |

The [Accu DIN 965H / ISO 7046-2 M2.5 × 6 drawing data](https://accu-components.com/us/phillips-countersunk-screws/65910-SIK-M2-5-6-A2) specifies a 4.7 mm head. An ISO 10642 socket countersunk screw can have a substantially larger head and is **not interchangeable with this seat**. Check the ordered drawing, head recess and tightening torque. The 1.8 mm plate thickness below the countersink, small screw threads and motor thread material still need joint/strip-strength review before fabrication. The mass allowance includes hardware; individual thread and screw solids are not in the 33-part CAD clearance report.

The fork's mounting ear touches the lower plate; it does not pass unsupported through the lightening window. Its two M3 holes are 6 mm above the window edge. The plate and fork faces meet without positive-volume overlap. Inspect tool clearance with the selected washers, nut and driver before releasing drawings.

## Fits, retention and service

The digital bearing interfaces are nominal Ø8 and Ø22 contact surfaces. SKF distinguishes fits for rotating inner-ring load from stationary outer-ring load; a generic loose shaft fit is not automatically suitable. Consult the [SKF rolling-bearing catalogue and seat recommendations](https://www.skf.com/binaries/pub12/Images/0901d196802809de-Rolling-bearings---17000_1-EN_tcm_12-121486.pdf). A transition/interference shaft seat such as j5 and an H7 housing are candidates for the light-load arrangement, subject to material, temperature, clearance and actual catalogue conditions. No micrometre tolerance has been asserted in the nominal CAD. Finish/inspect both seats after profile machining. Provide documented shoulder/clip or retainer geometry, preserve bearing internal clearance and avoid unintended axial preload between the motor output and outboard bearing. Current CAD does not qualify axial retention or the complete motor's overhung-moment limit.

Remove the front shell's four screws for battery/electronics service; shell tabs connect the rear to the tray. The pack reserve fits a 3-wide, 2-high arrangement of cells oriented along X: 65.2 mm cell length inside 75 mm, 55.8 mm combined width inside 64 mm, and 37.2 mm height inside 44 mm. Remaining space is only a packaging provision for insulation, busbars and protection. Use two straps and a compliant bottom pad; their allocated mass and 1 mm tray gap do not constitute a released retention drawing. Select strap slots and certified pack/BMS before production.

Route small bundles through the side brace/shell slots and along inboard motor faces, with flexible loops at the hip and knee, strain relief at connectors and protection from moving annuli and tires. The nearest body/hip-housing gap is 7.8 mm; a 3 mm bundle could leave approximately 2.4 mm nominal clearance each side. Connector and bend-radius dimensions are not known, so this is a corridor reservation, not verified harness fit. The 70 × 64 × 10 mm electronics space and 80 g wiring allocation must be replaced by exact boards, connectors and harness geometry in M4/M5. The front camera space remains a future payload reserve.

## Preliminary load and power review

`python -m bipe.mechanical_review` derives load margins from the saved final acceptance traces and writes configuration and calculations to `artifacts/milestone-3/engineering/`. It compares each role's worst RMS/peak/speed with the published 1.3 N·m rated and 4.1 N·m peak actuator values. A deliberately conservative linear torque/speed screen is not a measured motor operating map. Rated thermal conditions, current/torque conversion and supply sensitivity remain unknown.

The structure screen uses twice nominal half-robot weight per wheel with another 15% mass margin, a simple cantilever fork crossbar, chassis load on the hip brace, twice observed peak hip torque, and projected bolt loads. It checks CAD-to-MuJoCo mass, COM and full inertia tensors independently. It is a sizing screen; it does not certify fatigue, impacts, fall loads, bearings as a complete module, printed inserts or all possible software/fault torques. Jumping is outside scope.

The provisional P28A pack has 21.6 V nominal, 25.2 V fully charged and 2.8 Ah: 60.48 Wh by nominal voltage × capacity. [Molicel cell datasheet](https://www.molicel.com/wp-content/uploads/INR18650P28A_1.3_Product-Data-Sheet-of-INR-18650-P28A-80093.pdf). The runtime scenarios use 70% usable energy, 60–80% conversion efficiency and an assumed 18–30 W holding/driver/electronics allowance, plus sampled positive mechanical work. They take no credit for regeneration. Adopt a **one-hour provisional runtime planning target**, pending measured DC loss and voltage confirmation. These calculations are not a battery endurance prediction. Published motor phase currents cannot be added to infer pack DC current, and cell current capability does not establish BMS or wiring capability.

The actuator is rated at 24 V. The exact AK48-2405-2D-A2 driver's allowable voltage range and regenerative-braking behavior are not established by the linked generic manual. Direct 6S connection remains unapproved. M4 must resolve these limits and choose pack protection, regulation if required, regeneration handling, current sensing, fusing, enable interlocks and startup/calibration before any hardware power-up.

## Cost and remaining hardware checks

The manufacturer's indicative actuator price is $135.90 each, or **$815.40 for six**, before all other parts, fabrication, shipping and tax. [CubeMars store](https://store.cubemars.com/products/ak40-10-v3-0-kv170). This is a documented digital baseline, not the cheapest solution or a purchasing recommendation. FEETECH STS3250 is a much lighter alternative for leg joints, but its local servo transfer function, inertia and calibrated torque interface remain unresolved. Its no-load speed leaves little margin for 0.3 m/s wheel drive with 40 mm radius wheels; a cheaper build may use different wheel and leg actuators. Reopen this tradeoff before procurement.

Before hardware construction: confirm exact protocol scaling, dual-encoder startup and zeroing, real control bandwidth/gain mapping, temperature-dependent torque, gear friction and 0.3° backlash/reversal impacts; select and test tire properties; finish connector/fastener/harness/retention drawings; qualify material, bearing fits and shaft alignment; and measure actual power/regen behavior. Simulation uses continuous encoder coordinates; real packet angle wrapping, dropped samples and turn accumulation must be resolved with the exact protocol in M4. Digital sensitivity tests cover separate inertia, COM and static angle-bias variations. They do not model dynamic free play or validate combinations of all uncertainties, terrain, jumping, falls or long-term wear.

## M4 packaging update

The [packaging checkpoint](milestones/04-packaging-results.md) preserves this M3 mechanical assembly as historical evidence. Current `config/robot.json` reshapes the custom pack provision to 70 × 64 × 48 mm, shifts it 6.5 mm forward and places the carrier, rear power board, horizontal capacitor and upper resistor bank in separate zones. The lightened tray retains its mounting lands; electronics are allocated 79.5 g and hardware/harness 111 g after the explicit screw inventory. Component/support and remaining allowances reconcile to 2.494762 kg. Cable and fastener estimates are distributed to their rigid bodies. The full 80 g future camera allocation remains.

Current envelopes pass assembly clearance and conservative service sweeps, while pack/BMS/fuse selection, exact mount attachment, connector bending and heat rejection remain unqualified. Current simulations and CAD mass consistency are in `artifacts/milestone-4/packaging/`; the M3 load/runtime figures above remain historical results for their saved configuration.
