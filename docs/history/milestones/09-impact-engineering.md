# M9 impact engineering — bumper-first checkpoint

5 October 2026. Only the six passive tips require structural factor ≥2.
Cross-slope including the tumble and all contacting powered pushes require
containment; cosmetic factor2 is exempt but attachment/containment is still
required. The evaluated 40 mm LD45 pad / frame option provides 20 mm nominal
compression stroke, but the 200 mm width leaves only 0.250 mm beyond existing
lateral hard hardware. Its incomplete allocation is 2,164 g against 2,150 g,
so no dynamic bumper tips, mount qualification or CAD adoption occurred. See
[the current measured record](09-physics-results.md) and
[explicit mass ledger](../../../artifacts/milestone-9/bumper-first/mass-gate.json).
The following studies are historical; they do not establish an adopted remedy.

## Historical closure — impact and engineering bounds

Completion-pass note: the earlier 50 g reserve/1.85 kg allocation below is historical. The authorized ceiling is now 2.15 kg allocated plus ≥150 g reserve; [current results](09-physics-results.md) record the evaluated H900/foam/heat option and mass stop.

5 October 2026. **Structural gate fails; candidates are not adopted.** The amended gate requires factor ≥2 for structural load paths, tooth-jump/clutch protection and retention. Cosmetic strength is informational, but containment, safe edges and electronics isolation remain mandatory. The CAD remains 1,794 g; the authorized allocation is 1,850 g with ≥150 g reserve. No reserve was spent.

## Frozen falls

[Measured falls](../../../artifacts/milestone-9/closure-final/impacts/fall-screen.json) combine six passive tips and three powered falls. The free base uses all 21 CAD tensors, physical contact, finite motor/belt dynamics and mechanical five-bar joints. Passive phases are ideally open, retaining rotor inertia, journal friction, belt losses and 1.85 N·m Coulomb slip couplings. Actual inverter/body-diode isolation is unqualified. Nominal passive releases use ±10° pitch or 32° sideways tilt, at rest/0.3 m/s; angular release velocity is zero. No state is clamped.

| Case | Kinetic energy at first body contact, J | Detection / command off / current <0.01 A |
|---|---:|---|
| Six passive tips | Peak over fall: 0.644–0.940, including rotation | Unpowered throughout |
| +10 N·s push | 33.2, including 3.53 rotation | 55 / 130 / 135 ms after push |
| −10 N·s push | 33.3, including 2.53 rotation | Local fault/off 50 ms; current 54 ms |
| Physical 40° cross-slope | 5.36, including 0.0701 rotation | 140 / 420 / 422 ms from release |

Both supplementary ±6 N·s pushes remain upright. The +10 body strike occurs at 111 ms, before shutdown; joint-feedback rejection terminates its crouch. The −10 local attitude interlock precedes host invalid-state logging by 5 ms and prevents crouching. Cross-slope first body contact is 546 ms; continued downhill rolling over six seconds reaches 222 J kinetic energy and governs many later loads. This is not 222 J at first contact. The model does not fracture; loads after predicted yielding remain conditional undeformed-body screens.

The portable C/Python policy latches at posture-relative pitch 19° moving outward >15°/s for 10 ms, immediate 25°, roll 12° for 10 ms, or <0.2 g for 20 ms. Recovered ±2.5 N·s hold peaks remain below 17.4°. It slews the 70 mm crouch reference ≤4 rad/s, caps hip requests at 0.7 N·m, fades wheel requests in 50 ms and ramps all requests off by 280 ms. Electrical, health and deadline checks retain immediate inhibit and manual reset. Torque shutdown is demonstrated; **legs/wheels-first protection is not**.

| Structural minimum over nine actual falls | Factor; all require ≥2 |
|---|---:|
| Hip shaft / journals / links / cassette tube | 0.0219 / 0.0407 / 0.114 / 0.0393 |
| Fork / printed hub spokes / hollow axle / output 608 | 0.00889 / 0.00745 / 0.206 / 0.293 |
| Wheel motor bearing / hip motor bearing | 0.343 / 2.85 |
| Fork-cap / saddle retention / saddle contact and preload | 0.0726 / 0.0302 / 0.0114 |
| Wheel readhead mount (sensor retention, not cosmetic) | 0.0902 |
| Hip housing / shell retention / clutch thrust face | 0.797 / 0.0346 / 0.811 |
| Clutch release protection | 1.19 |
| Belt uncertainty envelope, passive | 0.344 |
| Powered belt catalog domain / bumper mounts | Unqualified / unadopted |
| **Cosmetic only:** face / eye / rear / crank / rod covers | **0.0111 / 0.0117 / 0.0190 / 0.0921 / 0.208** |

All nine structural gates fail. Passive-only output-bearing factor is 8.83, replacing the historical 0.878 assumed-energy bound; powered loads remove that margin. Cosmetic retention/containment remains failed or unqualified despite the amended strength requirement.

## Materials, compliance and retention

[Material inputs](../../../config/m9-impact-materials.json) use supplier typical strengths with explicit knockdowns, not finished-part guarantees: [Kaiser 6061](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1015/Kaiser_Aluminum_6061_Sheet_Coil_and_Plate.pdf) 276×0.8=221 MPa; [7075](https://online.kaiseraluminum.com/depot/PublicProductInformation/Document/1028/Kaiser_Aluminum_7075_Rod_and_Bar.pdf) 503×0.8=402 MPa; [Carpenter H900](https://www.carpentertechnology.com/hubfs/7407324/Material%20Saftey%20Data%20Sheets/Custom%20630%20%2817-4%20PH%29.pdf) 1,262×0.79=997 MPa; [EOS PA2200](https://www.eos.info/polymer-solutions/polymer-materials/data-sheets/mds-pa-2200) weakest-Z 42×0.6=25.2 MPa; [BASF TPU88A](https://forward-am.com/wp-content/uploads/2021/04/BASF_3DPS_TDS_Ultrasint-TPU-88A.pdf) 7×0.6=4.2 MPa; [Igus G](https://www.igus.com/ContentData/Products/Downloads/iglide_G300_FM_USen.pdf) hot journal pressure 35 MPa.

Foam-core tires integrate cylindrical/annular contact patches, [LD70 typical 163/251 kPa at 25/50% compression](https://www.zotefoams.com/wp-content/uploads/2025/03/PIS-A017-Plastazote-LD70.pdf) and TPU tread in series, with 0.5–4 stiffness uncertainty and rising densification. PA12 shell strip bending has an assumed 4 mm backing stroke; that backing and cosmetic containment are unqualified. Six proposed frame-backed LD45 pads cover face, front/rear corners and sides: 18.9 g at [47 kg/m³ maximum sheet density](https://zotefoams.com/wp-content/uploads/2025/03/TIS-01-Product-Specification-1.pdf). Frozen compliance uses December-2017 typical 90/163 kPa at 25/50%; [current manufacturer data](https://www.zotefoams.com/wp-content/uploads/2025/03/PIS-A013-Plastazote-LD45.pdf) are 91/164 kPa, inside the stated bracket. The strong-side candidate probe still fails hub/fork/belt loads. Mounts and loaded clearance are unadopted.

Internal wrenches include tire-edge moments, actual bearing spacing and added belt preload. [608 static rating](https://koyo.jtekt.co.jp/en/products/detail/?pno=608) is checked with max(Fr,0.6Fr+0.5Fa), assigning full axial load to either locating bearing. Proposed retention uses [Bossard class-8.8](https://assets.eu.ctfassets.net/0vp0u5uh75zd/1VPwGxSRcAQdDRArMtTqeY/8fe01cf6ef692e134b45f895fb69fe97/012_016_Screws_property_class46_Fastening_EN_01_2025.pdf) proof 580 MPa and [SPIROL19 inserts](https://www.spirol.com/assets/files/SPIROL-Inserts-for-Plastics-us.pdf), with Nylon66 pullout ×0.75 plus PA12 parent-shear/punching bounds. Bolt stress, thread stripping, tearout and pull-through remain separate. Actual shoulders, drilling/access and retention are not CAD-adopted.

[Gates ratings](https://www.gates.com/content/dam/documents-library/catalogs/light-power-and-precision-manual.pdf) include actual smaller-pulley speed, width/length factors, pretension, positive slack tension and engagement. At 250→125 µs, applied hip peak is 1.31→1.69 N·m (+29.1%); endpoint law demand is 4.53–8.37 N·m. The conservative envelope retains 8.37 N·m: factor 0.344, required tension 199 N versus installed 60 N, negative slack. Powered rotor speeds exceed the 2,000 rpm table: no extrapolated pass. [Alignment limit 0.25°](https://www.gates.com/content/dam/gates/home/knowledge-center/resource-library/operating-manuals/preventive-maintenance-manual-en.pdf), journal clearances and structural deflection remain unqualified. No invented tooth-shear strength or shock-isolation credit is used.

## Heat path and mass stop

[SI sizing](../../../artifacts/milestone-9/closure-final/engineering/sizing.json) proposes stationary end-turn contact through 0.2 mm [STYCAST2850FT/CAT11](https://datasheets.tdx.henkel.com/LOCTITE-STYCAST-2850FT-CAT-11-en_GL.pdf), k=1.28 W/m/K; it avoids the rotating outer bell. Six exposed black 60×80×0.25 mm aluminium plates, four conduction ribs each, potting and mounts total **22.5 g**. Series Rθ: 0.592 enamel +0.370 potting +0.223 cap +2.99 ribs +0.500 spreading +11.2 air = **15.9 K/W**. Air assumes natural convection 5 W/m²/K, radiation at 65/40°C, emissivity/view 0.8/0.8 and fin efficiency 0.804 ([NASA guidance](https://ntrs.nasa.gov/api/citations/20230013900/downloads/NASA%20Thermal%20Control%20Engineering%20Guidebook%20v4.pdf?attachment=true)). Room-air exposure, end-turn contact, hip-plate overlap and hot spots require CAD/bench validation. All 107 duties give worst cold mean ≤50.0°C and repeated mean 74.9/120°C under peak/RMS interpretation; required worst RMS path ≤16.7 K/W at 40°C air. This is a conditional mean-node screen, not a hardware thermal pass. Final duty screening is in [thermal.json](../../../artifacts/milestone-9/closure-final/engineering/thermal.json).

Stock 12 mm hip belts only raise the conservative factor to 0.689. An optimistic six-lane 36 mm equivalent gives 2.07 before load-sharing penalties: extra belts 48.7 g and four large tooth-stock rings alone 89.1 g. With foam and heat paths, this **incomplete width-only option** adds ≥179 g. Even crediting an unadopted 19.9 g hardware-ledger reduction gives ≥1,953 g allocated: ≥153 g reserve draw, **≥103 g beyond the authorized 50 g**, before other reinforcements. It is not a global minimum or complete passing remedy. The earlier lightest quantified passive fork/foam change was 8.24 g, also incomplete and not sized for these powered loads. A lighter isolated-input clutch/guard needs a separate engineered design. Stop without applying reserve trade-offs; no bumper-fitted CAD sweep or tensor readoption is claimed.
