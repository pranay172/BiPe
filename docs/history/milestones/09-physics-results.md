# M9 independent step 1b — Coulomb tire friction

7 October 2026. **The slope target is met; the inherited suite drops to 25/27.**
Contact friction rows are now Coulomb: impratio 1e6 plus direct damping of
1000/s on every friction row. The force–depth normal law, controller and
thresholds are unchanged. A box on a 10° incline now creeps **4.3e−6 m/s**
(the archived viscous rows: **8.2 mm/s**). At 45° it slides at **1.387 m/s²**,
the Coulomb value.

| Check | Archive (step 1) | Step 1b |
|---|---|---|
| Hold rollback, 4 / 8 / 10° (limit 0.100 m) | 0.101 / 0.216 / 0.301 m | **0.045 / 0.077 / 0.092 m** |
| Inherited cases / convergence | 27/27 / pass | **25/27** / pass |
| Robustness survival; ±2 N·s recovery; slopes; backlash and belt cycles ≤1.5° | pass | pass (belt max 1.26°) |

The two inherited failures are `sensors_combined` and
`mass_plus_15pct_combined`. They end at **−0.0315** and **−0.0358 m/s**
against the 0.03 m/s limit.

**Cause: a hold limit cycle.** The unchanged hold loop cycles with a period
of about 4 s: ±15 mm of travel, 0.8° pitch peak-to-peak and 0.03 m/s peak.
Wheel torque reverses abruptly at each velocity reversal, which points to
backlash crossing in the wheel drive. The cycle is physical, not numerical:
- Friction-row damping of 300 or 3000/s and impratio 1e3 leave it unchanged.
- Removing rolling friction enlarges it to 1.2°.
- In the archive, viscous tire creep damped it.

Whether a combined case passes depends on where in the cycle it ends.

**Rejected gain candidates.** Six bounded hold-gain candidates were tried;
all are rejected and the controller is unchanged. Each trades one target for
another without removing the cycle:
- `hold_speed_ki` 0.05: 27/27 plus convergence, but the 10° rollback is
  0.111 m and the drive-push recovery margins fall from 2.5 to 0 N·s.
- `hold_speed_ki` 0.03, and 0.05 with `hold_kp` 0.6: rollback 0.204 m and
  0.147 m.
- Lower or zero wheel friction compensation enlarges the cycle.

**Hip clutch hold load.** The 1 s hold now loads the hip clutches to
0.082 N·m peak; the viscous rows' creep had pre-loaded them to 0.185. The
loaded-support floor in the clutch test drops from 0.1 to 0.05. Its
encoder-LSB and slip-margin checks are unchanged.

**Open.** The hold loop needs a backlash-aware change: a controller
decision, not a gain retune. Fall and tip results are rerun on the new rows
in step 2.

Regression: **111 passed, 4 skipped**. Evidence:
- [report](../../../artifacts/milestone-9/friction-rows/report.json)
- [acceptance](../../../artifacts/milestone-9/friction-rows/acceptance/summary.json)
- [probes and reproduction](../../../artifacts/milestone-9/friction-rows/reproduction.json)

## Independent step 1 — slope hold remains unresolved

5 October 2026. Six bounded sensor/C capture-and-integral trials achieve no better
than **0.276 m** rollback at 10°; all are rejected. Wheel torque stays ≤0.117 N·m
against the existing 0.720 N·m continuous rating. Independent geometry measures
a **0.0134 m/s** rolling/axle speed gap over 4–6 s; carrier-differencing bias is
only 1.42e−10 m/s. The adopted controller, plant and thresholds remain unchanged.
Frozen `2f70dcf` runs all 107 cases; 32 firmware/robustness regression tests pass.
Justified rolling-creep/ground-speed handling remains open. See
[fresh evidence and reproduction](../../../artifacts/milestone-9/slope-hold/reproduction.json).

| Independent step | Frozen result |
|---|---|
| Slope hold only | **4/8/10° rollback: 0.101/0.216/0.301 m**, versus ≤0.100 m; 12/12 slopes survive. **27/27 + convergence PASS; 70/70 survive, 60/70 recover**, required ±2 N·s recover in all three modes. Delayed-drive MAE **0.0290/0.0228 m/s** (limit 0.0300). Python/C: 1,000 samples, max error **4.44e−16**. |

## Historical bumper-first preflight — mass stop

5 October 2026. **M9 remains incomplete.** The revised gate requires structural
factor ≥2 for the six passive tips only; cross-slope/tumble and contacting push
falls are containment cases. The evaluated bumper/frame option exceeds the
unchanged allocation, so implementation stopped before dynamic tips or CAD
adoption. No new slope control, heat adoption, fairings, clearance/tensor
adoption, 107-case acceptance, extreme containment rollout or CAD renders were
completed in this pass.

| Candidate mass checkpoint | Mass, g |
|---|---:|
| Adopted CAD | 1,794 |
| Native net LD45 foam / joined 6061 tubes and backings | 29.8 / 182 |
| Conditional stationary heat paths | 22.5 |
| Optimistic additional hip belts / large pulley tooth stock | 48.7 / 89.1 |
| Hardware overrun (includes 16 M3 bolts) / optimistic drilling credit | +1.66 / −4.17 |
| **Incomplete candidate / allocation / excess** | **2,164 / 2,150 / 13.7** |

This bounds **one option**, not every bumper-first architecture.
It excludes small pulleys/flanges, reinforcement, extra attachment brackets/pad retention, closed
windows, fairings and belt packaging. Native solid unions remove overlap; foam
is notched around frame envelopes. All bolt-shank drilling is optimistically
credited. The six-lane width-only belt estimate is
unqualified; it preserves the **8.37 N·m** measured passive-tip endpoint-law
uncertainty and **0.344** conservative factor. No shaft thickening was adopted.

The support-function study evaluates **2,888** orientations at 100/70 mm
postures, including both lateral directions, at 0.5° intervals. Enclosing foam
boxes optimistically cover rounded forms; **660** samples lack the intended
stroke and **144** have hard-first contact. At 90° the ±99.8 mm readheads leave
only **0.250 mm** beyond hardware within 200 mm width, versus **20.0 mm** useful
foam stroke. At 85° the readhead mount leads foam by **3.91 mm**. The frame
clears the preliminary nominal/crouch convex check;
this is not the native all-pose sweep or a verified mount/load path.

[LD45 manufacturer data](https://www.zotefoams.com/wp-content/uploads/2025/03/PIS-A013-Plastazote-LD45.pdf)
give typical 91/164 kPa at 25/50% compression. The candidate uses 40 mm foam,
20 mm useful stroke, upper-density 47 kg/m³, and calculated **1.66–2.21 J**
unnotched storage per pad at 50% strain; merged/notched pads are not qualified
from this estimate. Rate/patch uncertainty and actual load sharing
remain unvalidated; no candidate tip force is claimed.

At the same limits, options are to repackage outer hard hardware **14.8–19.8 mm
inboard per side**, use the existing spine as backing with lighter supported
pads, and qualify a lighter belt/input isolation arrangement. Savings and
containment need measurement; reducing frame mass alone cannot fix side stroke.

The adopted XML, masses/tensors, controller, sensors and firmware are identical
to `8a541af`; the following frozen results remain **historical**, not a fresh
bumper acceptance. **42 targeted tests pass**, including independent envelope,
candidate mass/free-base topology and prior M9 regression checks. See
[new evidence and reproduction](../../../artifacts/milestone-9/bumper-first/reproduction.json)
and [coverage/mass plot](../../../artifacts/milestone-9/bumper-first/coverage-and-mass.png).
Local commits only; M3 remains untouched/unstaged.

## Historical completion pass — prior fall scope and mass stop

The following record predates the passive-only design-case amendment above.

5 October 2026. **M9 is not accepted.** Runtime `8ecbb7f`; all 107 cases executed with unchanged thresholds. The evaluated reinforcement exceeds the authorized allocation, so CAD adoption stopped. Existing CAD/tensors remain 1,794 g and 199.5 mm; 506 g is unallocated against the 2.3 kg total ceiling.

| Check | Measured result |
|---|---|
| Inherited / convergence | **27/27 / PASS**; differences: pitch 0.0159°, height 0.0219 mm |
| Robustness / strict recovery / belt corners | **70/70 survive / 60/70 recover / 8/8 survive**; required ±2 N·s recover |
| Hold rollback, 4 / 8 / 10° | **0.101 / 0.216 / 0.301 m**, versus ≤0.100 m; all 12 slopes survive |
| Drive MAE, 25 ms lag / 15 ms delay + noise×2 | **0.0290 / 0.0228 m/s**, versus ≤0.0300 |
| Design / extreme falls | **0/7 qualified / containment unqualified for all 12 contacting push trials** |
| Shaft/journal/fork/hub/axle/bearing: passive minima | 1.41 / 2.88 / 0.571 / 0.334 / 5.20 / 4.56; required ≥2 |
| Same: cross-slope, initial 2 s only | 1.08 / 2.15 / 0.0338 / 0.0204 / 0.621 / 0.873; informational |
| Same: extreme minima | 0.811 / 1.56 / 0.0409 / 0.0525 / 0.738 / 1.29; informational |
| Belt / clutch protection / fasteners: passive; cross 2 s; extreme | 0.344 / 1.44 / 0.0981; 0 / 1.19 / 0.125; 0 / 1.19 / 0.0229 |
| **Cosmetic only**, face/eye/rear/crank/rod: passive | **0.0415 / 0.233 / 0.238 / 0.0921 / 0.208** |
| Same: cross 2 s | **no face contact / 0.0369 / 0.0350 / 0.117 / 0.122** |
| Same: extreme | **0.0152 / 0.0117 / 0.00744 / 0.0461 / 0.100** |
| Candidate heat path / mass; cold / repeated RMS winding mean | 15.9 K/W / 22.5 g; 50.0 / 120°C; conditional, **unadopted** |
| Cold bus upper bound, peak/RMS convention; gross regen energy/power | 1.89 / 2.45 A; 7.22 J / 9.70 W; hardware unqualified |
| Current CAD / incomplete reinforcement option | 1,794 / **2,247 g** allocated; limit 2,150 g + ≥150 g reserve |

Drive damping is now 0.13, with hold damping 0.15. Bounded hold-integral probes (0.2–1.0) achieved no better than 0.275 m rollback; hold gains remain unchanged. No pose constraints or state clamps were added. Incline claims preceding the ground-transform fix remain invalid.

The 40° design fall first contacts a leg cover at **5.03 J**: detection **140 ms**, physical crouch **188 ms**, current shutdown **472 ms**, contact **552 ms**. Its six-second rollout becomes numerically unstable at **3.58 s**; the short initial-landing factors above do not qualify that full run. Passive first contacts span 0.359–0.908 J. The actual-contact hold search brackets both directions at **(6.00, 6.125] N·s**, 16 s horizon, pulse at 2 s; ±6.125 contact at **17.9/19.2 J**, hence extreme. Both crouch before contact; the negative case lands on leg covers. ±10 contact at 33.3–33.4 J. All twelve distinct contacting push trials, including the bounded search, are included in the extreme minima; five no-body-contact outcomes are also saved. The separate 4 s-pulse yardstick crosses its 20° limit at negative (4.5625, 4.625] N·s; this alone is not a demonstrated fall.

The fixed-load H900 shaft option plus [Plastazote LD45 foam](https://www.zotefoams.com/wp-content/uploads/2025/03/PIS-A013-Plastazote-LD45.pdf) and heat paths totals **2,247 g**, **96.6 g over** allocation. It excludes enlarged interfaces/bearings, forks, hubs, axles, belts, mounts, retention and fairings. This is an optimistic bound for that option, not a global minimum; loads after predicted yield are conditional. Options: change the impact load path and arrest downhill rolling; qualify an isolated belt input/impact bypass; or increase allocation for this incomplete option.

Bumper mounts, retention, battery/conductor containment, heat plates, closed windows and new fairings remain **unadopted**. No new fitted clearance sweep, tensor adoption or renders; the original 222-pose layout remains. Belt factor 0 denotes speed outside the catalog domain. Cosmetic factor 2 is exempted by the amendment; attachment/containment is still unqualified. Open fidelity: impact convergence, tooth jump/alignment, foam rate/backing, post-yield behavior and winding hot spots. Prediction/LQR was not rerun.

[Acceptance](../../../artifacts/milestone-9/completion/acceptance/summary.json), [factor/time audits](../../../artifacts/milestone-9/completion/validation/report.json), [thermal screen](../../../artifacts/milestone-9/completion/engineering/thermal.json), and [reproduction commands](../../../artifacts/milestone-9/completion/validation/reproduction.json) save inputs, hashes and traces. Regression: **107 passed, 4 skipped**; native/default/M9 ARM builds pass; C/Python 1,000-sample discrepancy **4.44e−16 N·m**. Artifacts are about 54.5 MB; large CSV/probe records are losslessly gzipped. Superseded closure/prediction artifacts remain in Git history. [Existing renders](../../../artifacts/milestone-9/renders) are unchanged. Local commits only; M3 assembly untouched/unstaged.
