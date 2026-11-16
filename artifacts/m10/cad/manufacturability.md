# M10 manufacturability screen (phase B3)

Layout-study fidelity. Purchased parts are catalog envelopes, and fasteners
other than the shell screws and the motor studs are not modelled. Nothing
here is hardware-validated.

Inputs: `cad/m10/assembly.py` at the nominal pose. Measured walls are in
`manufacturability.json`, produced by `cad/m10/manufacturability.py`.

## Process

- **Printed shells, covers and brackets: SLS PA12.** The mass model uses
  the density of EOS PA 2200. SLS needs no support structures, so print
  orientation is chosen for accuracy and finish:
  - Body cover and tub: split face down, so the parting faces print flat.
  - Battery hatch: outer face up.
  - Head halves: split face down.
  - Thigh body and lid, shin, hub, hip housing: bearing-seat axes along
    the build z axis, so the seats print round.
  - Bearing seats are printed 0.1 mm under size and reamed. This is an
    assumption until test prints confirm it.
- **Hip bracket: PA12-CF** (an assumption carried from B2; the mass model
  uses the PA12 density).
- **Tires: SLS TPU** (BASF Ultrasint TPU 88A, from the M9 sources).
- **Machined:**
  - 7075 inner shaft (Ø10/5 mm, Ø8/5 mm journals) and knee shaft
    (Ø10/5 mm), both with harness ports, plus the outer shaft and the axle
    tube;
  - 6061 bulkhead, a pocketed plate with tension slots;
  - aluminium motor standoffs;
  - 2GT pulleys, as stock 20T, 40T and 80T bodies re-bored to the shafts.
- **Purchased:**
  - RO40 Lite modules and GL40 hub motors;
  - MR128, 6704 and 6700 bearings;
  - 2GT belts: 95T 6 mm (hip) and 110T 9 mm (knee);
  - moteus-c1 boards, the 6S 850 mAh pack, Camera Module 3, AS5047P boards;
  - M2.5 heat-set inserts, screws and studs.

## Measured walls

Screen: inward normal rays from a 5 × 5 grid on every face of each printed
part (left side; the right side is a mirror). The threshold is 1.0 mm,
from SLS PA12/TPU minimum-wall guidance of about 0.8–1.0 mm (an
assumption).

| Part | Material | Samples | Min (mm) | 5th pct (mm) | Median (mm) | Below 1.0 mm |
|---|---|---|---|---|---|---|
| BodyCover | PA12 | 1269 | 2.0 | 2.0 | 2.0 | 0 |
| BodyTub | PA12 | 1376 | 1.5 | 2.0 | 2.0 | 0 |
| BatteryHatch | PA12 | 170 | 1.9 | 2.0 | 2.0 | 0 |
| FrontCameraBezel | PA12 | 117 | 3.0 | 3.0 | 3.27 | 0 |
| HeadRear | PA12 | 800 | 1.43 | 1.59 | 1.62 | 0 |
| HeadFront | PA12 | 1457 | 0.34 | 1.6 | 3.37 | 2 |
| CameraBezel | PA12 | 250 | 3.0 | 3.0 | 12.16 | 0 |
| Neck | PA12 | 140 | 12.0 | 12.0 | 14.0 | 0 |
| Ear | PA12 | 640 | 7.0 | 7.0 | 12.41 | 0 |
| HipHousing | PA12 | 140 | 3.0 | 3.0 | 3.02 | 0 |
| Thigh | PA12 | 477 | 2.0 | 2.0 | 2.0 | 0 |
| ThighLid | PA12 | 741 | 1.6 | 1.6 | 2.42 | 0 |
| ThighPanel | PA12 | 250 | 1.2 | 1.2 | 22.81 | 0 |
| HipCap | PA12 | 117 | 1.2 | 1.2 | 1.32 | 0 |
| Shin | PA12 | 415 | 1.2 | 1.31 | 7.66 | 0 |
| Tire | TPU | 100 | 8.0 | 8.0 | 9.41 | 0 |
| HubDisc | PA12 | 182 | 1.1 | 1.1 | 4.45 | 0 |
| HubBlade | PA12 | 150 | 1.5 | 1.5 | 2.0 | 0 |
| HubCap | PA12 | 117 | 2.1 | 2.1 | 2.28 | 0 |
| Tread | TPU | 21600 | 0.8 | 0.8 | 2.5 | 7200 |
| HipBracket | PA12 | 310 | 2.0 | 2.0 | 4.53 | 0 |
| KneeBoss | PA12 | 74 | 2.0 | 2.0 | 2.0 | 0 |

Two entries in the table fall under the threshold, and neither is a thin
wall:

- **HeadFront: 2 samples at 0.34 mm.** There is one sample per side, at the
  lower corner of the camera-bezel seat, where the pitched seat round meets
  the flat front face at about 85°. The sample point is 0.03 mm from that
  edge, so its ray is nearly parallel to the front face and leaves through
  it. The wall behind the bezel seat is a 1.6 mm boss, and the median wall
  for the part is 3.4 mm.
- **Tread: 7200 samples at 0.8 mm.** The tread knobs are 2.5 × 2.5 mm and
  stand 0.6 mm proud of the TPU tire, which they are printed with. The rays
  measure knob height, which is surface relief, not a wall. SLS may round
  the knobs; that is a cosmetic risk only.

Changes the screen drove in B3 (the visual ones are listed in the milestone
doc):

- radial battery-hatch cut;
- screw columns moved clear of the corner fillets;
- base flat and eye size for the eye rim at the base;
- eye lens seated on a ledge in place of a 0.5 mm printed eye skin;
- a straight camera window with a boss-backed, front-opening bezel seat;
- the thigh split below the cavity fillet;
- offset cavity fillets;
- inlay plates in 1.4 mm pockets with a 3 mm outer thigh wall;
- shafts through the shin;
- hub wall 2.1 mm;
- the shin harness channel kept 0.3 mm above the axle-tube seat floor
  (it first cut a 0.1 mm sliver into the floor).

## Fastening

- **Body shell.**
  - The split is horizontal at hip height. The cover and the tub join on
    four M2.5 screws: screw columns Ø8 mm at x −70/+49 mm and y ±40 mm,
    with heat-set inserts (Ø3.6 × 6 mm holes) in the tub and counterbored
    clearance holes in the cover.
  - The hip pods are bolt-on bearing housings spanning the split.
- **Battery hatch.** In the tub floor, with a 0.3 mm gap. Its front edge
  lies on a radial plane through the front-bottom fillet centre, so both
  edges are square.
- **Head.** Split front and rear at 5 mm ahead of the pod centre. The neck
  bore is Ø12 mm, which also carries the head harness.
- **Thigh.** A body and an outer lid. The split is 0.5 mm below the cavity
  ceiling, giving the lid a shallow locating rim. The lid carries the hip
  cap and the panel inlay. Lid screws are not modelled yet.
- **Motors.**
  - The knee module stands on three Ø5 mm standoffs from the bulkhead.
    One M2.5 stud per position clamps the left and right standoffs
    through a slot.
  - The slots allow ±0.7 mm along x, which is the hip-belt tension travel
    (as in M9). The thigh module's saddle screws to the hip bracket
    through matching slots.
- **Knee belt.** Tensioned by eccentric 6700 seats in the thigh, with
  0.5 mm eccentricity (an assumption).

## Assembly order

1. **Drive module.** Bulkhead, four RO40 modules on their standoffs and
   saddles, concentric inner and outer shafts, sleeves, 80T pulleys and
   hip belts, then the hip brackets. Tension the hip belts on the slots.
2. **Electronics.** Fit the moteus boards and the host/IMU board to the
   module, then lower the module into the tub. The bulkhead feet screw to
   the tub floor.
3. **Hip pods and cover.** Fit the hip pods over the 6704 outer bearings,
   then close the cover on the four screw columns.
4. **Thigh and shin.** Fit the thigh body over the outer shaft. Fit the
   knee shaft with its 6700 bearings and the shin. Fit the 40T pulleys and
   the 110T knee belt, set the eccentric seats, thread the harness through
   the shaft ports and along the gallery, then close the thigh lid,
   the hip cap and the panel inlay.
5. **Wheel.** Fit the GL40 stator to the shin face, the axle tube and
   encoder, then the rotor with the hub and the tire.
6. **Head.** Fit the LED boards and the camera, pass the harness through
   the neck bore, then join the head halves.
7. **Battery.** Fit last, through the hatch.

## Open items

- **Harness route to the legs (decided: hollow shafts).** The harness
  runs from a port in the inner shaft inside the body, along its Ø5 mm
  bore to a port in the thigh's 3 mm wire gallery, and along the gallery
  as a Ø2.6 mm bundle to a port in the hollow knee shaft. From there it
  passes through a Ø3.2 mm channel in the shin to the axle-tube bore.
  - The thigh is 3 mm thicker to make room for the gallery.
  - How the bundle is fixed to the lid, and its strain relief at the
    ports, are not modelled yet.
  - The GL40 leads entering the axle tube inside the motor's through-bore
    is an assumption.
- **GL40 on 6S.** The GL40's 16 V rating on the 6S bus needs current and
  voltage limits in phase C.
- **Camera Module 3.** The lens block and lens offset are assumptions until
  checked against the official mechanical drawing.
- **Fits.** Bearing-seat fits, insert-boss pull-out and the thigh-lid
  screws are left for test prints.
