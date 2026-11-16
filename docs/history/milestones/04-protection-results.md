# M4 protection and operating-policy results

Implementation follows plan checkpoint `a7f3c6f`, extending `160aed1`. New evidence is under `artifacts/milestone-4/protection/`. **M4 remains open; the accepted physical model is unchanged and real motor operation remains inhibited.** The separate interface module is a circuit/routing study, not an integrated robot board.

## Protection interface

The native module places a reverse-input controller before both actuator and logic branches. [LM74700-Q1](https://www.ti.com/lit/ds/symlink/lm74700-q1.pdf) drives an [AOD2910](https://www.aosmd.com/sites/default/files/res/datasheets/AOD2910.pdf) with source at raw input and drain at protected output. A bidirectional raw-input TVS avoids forward conduction with a reversed 6S pack. Positive clamps protect the output and actuator interfaces. Regeneration must be dissipated independently; this topology prevents relying on charging current into the pack.

The proposed downstream substitutions are four 100 V AOD2910 switches, bus-powered [TPS7A1650DGNR](https://www.ti.com/lit/ds/symlink/tps7a16.pdf) for SAFE5, and the 50 V-input [Pololu D36V28F5](https://www.pololu.com/product/3782/specs) for logic. The original MCP1799 must be removed before connecting the alternative SAFE5 source. These changes have **not** been applied to the historical carrier/power boards or adopted robot. Native module pins, explicit DNC pads, body-diode polarity and route coverage are audited; final **ERC/DRC/unconnected/parity counts are all zero**, with **37 mapped pins** and declared 70 µm copper per face.

[ST SMBJ26A/CA datasheet DS1283 Rev14](https://www.st.com/resource/en/datasheet/smbj26a.pdf), Table 2 and footnotes 3/4, provides clamp formulas for current and temperature. The selected ordering codes are SMBJ26CA-TR / SMBJ26A-TR. Both pulse families, 25/65/125°C junction, 6.5/10 A and 50/200 ns removal are retained. With junction ≤125°C, current ≤10 A, local loop ≤20 nH and removal ≥200 ns, the maximum calculated rail is **43.512 V** and charged-output plus reversed-pack differential is **68.712 V**. These are conditional envelope calculations, not extracted parasitics, measured switching or an approved actuator voltage. The 50 ns/hot/reversed-input negative corner exceeds the controller's 70 V recommended differential; a higher FET rating alone does not solve it.

Finite-inductance SPICE experiments use 0.5/2 µH harnesses, a 20 nH clamp loop and explicitly assumed 10 nF local / 100 pF TVS capacitance. Measured maxima are **40.705/43.148 V**, with TVS energies **46.9/250.4 µJ** including source work. Full traces are compressed losslessly. The circuits use behavioral maximum-envelope devices. Actual capacitance, loaded gate timing, pulse shape/repetition, junction temperature, inrush, reverse transition and short-circuit peaks remain qualification items. The TVS is not a 10.5 J braking sink and does not prove motor survival during a surge.

The revised 4 A / 100 ms regenerative analytical screen is **26.965 V**, with minimum modeled dump capacity above 4 A. AOD2910's higher on-resistance matters: the main pair's screen is **3.549 W at 6.5 A**, using the published 125°C / 10 V-gate resistance; the reverse device adds **1.775 W** at that gate condition. LM74700 regulation does not guarantee that gate voltage or dissipation. Real board/device thermal resistance, SOA and gate behavior remain open. The original carrier's 100k/10k ADC divider also needs a clamp or divider revision before integrating this transient domain.

The **38 × 36 × 1 mm** separate module includes 3 mm continuous paths and 1.5 mm pulse paths with three ground vias per TVS. Native geometry is audited independently of rule reports. A full-copper/gross-FR4 upper screen, declared part/mount/wire allowances and the buck's 0.8 g mass increase yield a standalone robot estimate of **2.502935 kg**, **2.935 g over the ceiling**. Reject adding this module as another robot board. Consolidating its functions into the existing power-board allocation, revising the carrier interfaces and auditing the full layout/fit/mass are still required. No existing PCB mass saving, camera reserve or BMS uncertainty has been spent to conceal this failure.

## Startup heat and stop reserve

Sixteen analytic precharge corners use 22 Ω ±5%, 800/1200 µF and 0/0.05/0.1/0.15 A disabled load. **12 corners** reach a measured 90% bus ratio within 150 ms and the energy gate; all 0.15 A corners fail readiness. The exact combined motor capacitance and disabled current remain unknown. A short would dissipate **15.192 J** in the old 500 ms timeout. The proposed 0.75 J upper-meter trigger plus a qualified 10 ms response fits a **1.2 J** reservation. An independent cutoff must enforce that bound; the current hardware circuit does not yet implement it.

A three-node model adds the fifth resistor's mounting footprint without adding panel heat capacity. Its precharge-to-panel spreading requirement is 8 K/W; the previous dump-to-panel requirement remains 2 K/W. At 45°C, h≥3, ε≥0.8, contact≤0.1 K/W and other roof heat≤0.2 W, full **1.2 J startup + 10.5 J dump** reservations every 180 s give **58.832°C dump case / 145.932°C element** and **55.448°C precharge case / 131.410°C element**. Independent RC SPICE agrees at the cycle endpoint within **2.7e−9 K** and conserves energy within **1.3e−5 J**. Boundary/interface/material limits remain assumptions and requirements, not measured hardware properties.

The stop calculation now includes translation, root/joint/wheel rotation, output-referred rotor inertia, one ≤0.1 m COM drop, **1.5 J capacitive/auxiliary energy** and **1.3 J subsequent positive driver work**. Proposed bounds are mass≤2.5 kg, every body COM speed≤0.6 m/s, root rate≤2.5 rad/s, relative leg rate≤1.5 and wheel rate≤16 rad/s. The baseline-tensor screen totals **6.135 J**, with a **6.5 J reservation**. Exact motor capacitor/magnetic energy and driver effort mapping must fit the auxiliary allowance. Unlimited external forcing, slopes, arbitrary falls and powered oscillation are outside the domain.

Allowing full finite motor effort throughout a 500 ms stop permits **17.1 J** of positive work under these speed bounds, which fails the 1.3 J allowance. The policy therefore adds a positive-work meter/cutoff; it does not assume that sending a stop request ends energy input. Saved-pose screening covers 28 nominal/variation/convergence/fault traces using native forward kinematics and joint names. **25** fit the sampled motion/mass screen; the two +15% mass cases and the unpowered fall are outside it. Finite differences miss between-sample peaks, and this read-only check does not establish future-stop or hardware compliance. The accepted physical acceptance suite was not rerun or changed.

## Portable policy and verification

The separate C policy reserves 6.5 J for stopping and begins stopping at **2.5 J** of upper-bounded normal dump energy. The remaining 1.5 J covers the specified response margin. It separately checks interval peak voltage, two local temperatures, precharge energy, dump energy, positive stop work, readiness and stopping-domain evidence. Feedback must be current within 5 ms; calibrated temperature error/lag is bounded by 1°C and meter errors by 5% + 0.05 J in the declared policy. The proposed positive-work cutoff is 0.5 J; bounded power and both stop-entry/cutoff delays fit the 1.3 J future-work reservation. A real adapter must cancel queued commands, disable retained local PD and remove power within those timing limits.

Startup and repeated failed starts require a full 180 s lockout and 2 s stable qualified temperatures. Ordinary reset retains counters/debts and restarts cooldown; a new boot also locks out. Heat or motoring work during cooldown postpones rearm. Reset during operation faults; stale, duplicate, backward/reset counters, invalid sensing, excessive voltage/energy, missed startup/stop timing, revoked evidence and excessive positive stop work are exercised. Real and synthetic profiles are rechecked against compilation/evidence gates, so changing a cached permission flag cannot enable the production build. Faults do not disable the independent hardware dump.

Warning-clean host tests, UBSan, production inhibition and Cortex-M7 compilation pass. Existing application/core tests, 22 Python tests and Ruff also pass. The policy is **not integrated into the real Teensy HAL**; the previously built diagnostic target remains locked. The accepted robot, C control core/gains and user's M3 CAD remain byte-identical to the prior checkpoint. The 1357-file seal is archived and checked against its original Git commit; the current seal includes the new evidence separately.

## Remaining M4 work

Integrate protection, ADC/sensing and independent startup cutoff into the existing board, then rerun native current/transient checks and CAD/whole-mass reconciliation. A supported actuator/pack specification and defensible gate/device/board thermal/SOA, peak-latch, meter, cutoff and thermal-interface limits are required for an operational adapter. The earlier AK48 and protected-pack evidence gaps remain open; supplier questions are prepared and unsent. Recompute candidate inertias and rerun physical acceptance before adopting any physical revision. Do not infer M4 completion, fabrication release or hardware qualification from the passing separate-module checks.

FreeCAD MCP connectivity works. Native KiCad, routing, ngspice and compilers work; manufacturer PDF endpoints for the selected TVS timed out locally, while its primary PDF was readable through the web tool. That source is marked web-read, without a fabricated local PDF hash. Blender is not needed.

```bash
.venv/bin/python tools/prepare_power_policy.py
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.protection_analysis
bash tools/electronics-env.sh .venv/bin/python tools/verify_protection_spice.py
.venv/bin/python tools/audit_stopping_domain.py
bash tools/verify_power_policy.sh
.venv/bin/python -m bipe.verify_protection
```

See `electronics/protection/README.md` for native regeneration and route import. All changes remain candidates.
