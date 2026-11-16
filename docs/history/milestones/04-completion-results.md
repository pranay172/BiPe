# M4 thermal, current-path and application results

Implementation started 1 October 2026 after detailed-plan commit `f6bc216`, extending packaging checkpoint `7db425b`. M4 remains open. The new CAD/power/firmware candidates and measured results are saved under `artifacts/milestone-4/completion/`; no hardware design or fabrication release is claimed.

## Thermal and packaging candidate

A **78 × 74 × 1.5 mm aluminum roof panel** replaces the internal spreader and a cut/recessed portion of the polymer roof. Its outer face is exposed to ambient; four dump resistors and one precharge resistor mount beneath it. Four 2.2 mm roof holes, a 3 g seal/mount/grease allowance, maximum resistor body envelopes and intentional seating contacts are modeled. The CAD body-to-panel contact area is an envelope intersection, not proof of the real ceramic thermal contact.

The thermal calculation has two finite-capacity metal nodes and conservatively linearized exterior radiation. Event heat enters only the four mounting footprints before spreading to the rest of the panel. It uses full manufacturer DC junction-to-case resistance at peak resistor power; no beneficial transient junction response is credited. The panel capacity excludes mounting holes. Periodic-state and independent energy-balance checks pass; 96 parameter corners and a saved cycle trace are provided.

The proposed **10.5 J / 100 ms event every 180 s** passes only with ambient ≤45°C, convection ≥3 W/m²K, exterior emissivity ≥0.8, each case contact ≤0.1 K/W, bank-to-panel spreading ≤2 K/W and other heat entering the roof ≤0.2 W. At the conditional electrical bound of 26.984 V, the model gives **58.73°C case** and **145.83°C element**, below its 60/150°C limits. These are predictions under requirements, not measured temperatures or an enforced operating duty.

Negative checks are retained: the previous internal spreader reaches **102.52°C** in its 12 s duty corner; applying the full **28 V** ceiling raises the new element bound to **152.77°C**. Routing all of the screened BMS's 0.845 W loss into the roof also fails. The calculated pre-event local-metal limit is approximately **51.06°C**, with a full 10.5 J reserve; calibrated sensing, actual event energy, precharge/restart heat and stopping/fall reserves still need implementation and qualification. A passive cooling path cannot by itself guarantee bounded energy during an external fall.

The **64 × 36 × 8 mm BMS envelope** uses the largest of conflicting manufacturer-page dimensions; it is not a selected module's guaranteed maximum. The pack provision grows to **70 × 64 × 50 mm**, and the carrier moves up 2 mm. Six maximum cell envelopes, BMS and fuse provisions remain contained with saved separation checks. The CAD also uses the revised power PCB placement rather than the historical shunt/IC coordinates.

All **143 native solids** are valid. Intentional contacts have zero volume overlap; no unintended intersections or assembly clearance failures are reported. STEP and fresh native round-trips agree within numerical tolerances. Conservative removal sweeps pass: unplug leads/remove front and camera, lift roof/bank upward, then remove carrier, pack and rear power board forward. A deliberately conservative 50 g, 2g roof-strip cantilever screen predicts **14.22 MPa / 0.557 mm**. Screw/insert pullout, polymer strength, seal preload, alloy, fatigue and tool access remain unqualified.

The combined candidate estimate is **2.495538 kg**, leaving **4.462 g** below the unchanged 2.5 kg ceiling before real BMS/protection/suppression uncertainties. The full 80 g camera reserve and existing PCB/harness allocations are retained. A panel bridge that also retains the old spreader already exceeds the ceiling before adding the bridge. The selected panel replacement fits the declared budget, but it is **not adopted**. Its snapshot retains the accepted rigid-body tensors for reference; they are not a valid dynamic mass model for the modified roof/pack. Recompute all COM/inertia and rerun physical acceptance before any future adoption.

## Native power-board work

`electronics/power-current/` is a separate **66 × 60 × 1 mm** native KiCad candidate. It specifies 70 µm copper per face, fixed 3 mm main bus paths, a 2 mm capacitor return and six 0.3 mm-drill vias at each relevant power land. The shunt moves near the input and the protection IC rotates so its sense escapes can route. Dedicated pick-offs now reach the two-terminal shunt lands rather than the input connector and main drain rail. Pad voltage gradient, IC bias, noise and mutual coupling remain qualification items; this does not turn the resistor into a four-terminal part.

Final native **ERC, DRC, connectivity and schematic parity are zero**; **193 pin/net mappings** are checked. The current auditor verifies the saved native width coverage and via groups against the actual board hash. Its design domain is **6.5 A continuous** and an assumed **10 A for 1.1 ms** before protection, not a measured six-motor limit. The external-trace empirical screen gives 3 mm copper approximately **11.89 A at 20 K rise** and the 1.2 mm source-pad neck approximately **7.32 A at 30 K rise**. Via sharing/plating, assembled temperatures and the fine logic/sense branches remain conditional. Native gate-net lengths/vias are recorded; real gate/source switching loops still need validation.

A geometric copper upper sum, including overlapping tracks/pads repeatedly, gross FR4 and a coating allowance, estimates **9.176 g** for the bare revised board under its stated material assumptions. This fits the preserved **11.5 g** board allocation. No mass saving is claimed or spent. Actual manufacturing thickness, coating, solder, via fill/capping, component and wire masses remain requirements.

Inductive turn-off screening exposes corners above the **40 V AOD4184A** rating. The current schematic therefore still needs an input/actuator suppression and reverse-input revision coordinated with exact motor limits and pack impedance. Back-to-back main FETs do not establish reverse tolerance for the upstream logic/charger ports. Device SOA, protection timing/errors, capacitor low-frequency pulse/ripple and short-circuit clearing also remain open. A clean PCB rule report does not resolve those limits.

## Exact-part evidence

The freshly retrieved CubeMars manual is byte-identical to the prior manual. The AK48 installation PDF entries still have empty links. The newly listed AK48 archive contains only **AK48-2405-2D-A2-V1.11.step**. Exact permissible voltage, MIT units/ranges, torque/current/gain mapping, local finite limits, timeout, encoder convention and startup/disable fields remain unresolved. See the [manufacturer download page](https://www.cubemars.com/technical-support-and-software-download.html) and the prepared [actuator questions](../actuator-manufacturer-questions.md).

The [EYBMS PCM-L06S10-554 page](https://www.eybms.com/product/6s-10a-bms-battery-protection-board-pcm-554/) conflicts on thickness, continuous discharge and protection/balancing values; NTC is optional and mass unlisted. It stays an unqualified fit candidate. The [AYAA PCM-L06S10-J15](https://www.ayaatech.com/product/pcm-l06s10-j15-6s-10a-bms-pcm-with-communications/) has a 109 mm edge longer than the entire candidate pack's diagonal, so it cannot fit in any orientation.

The **Littelfuse 0453010.MR** is a conditional fuse candidate. Its maximum body dimensions fit the fuse-module provision. Published 400 A interruption at 32 VDC requires a prospective short-loop resistance ≥63 mΩ at 25.2 V if used as the bounding rating. Nominal I²t is not maximum clearing energy; temperature derating and BMS/FET/wire coordination are unresolved. Its [official datasheet](https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1) was read through the web tool after direct downloads returned HTTP403; no local PDF hash is claimed.

AMASS XT30U battery mates are screened separately from the AK48's combined power/CAN connector. [Manufacturer catalog](https://www.china-amass.com/public/upload/20250319/41b7a7787d7c61edc3371469ba98f193.pdf) family ratings do not establish exact motor pinout or acceptable assembled temperature rise. [Pack supplier questions](../pack-supplier-questions.md) are prepared. No requests have been sent and no parts ordered.

## Application and verification

The [application boundary](../firmware-application.md) adds timestamped sensing, explicit sign/zero/axes, stationary calibration, 200 Hz six-frame scheduling, driver-limit acknowledgments, profile gating, stale/invalid feedback and CAN/deadline faults, controlled stopping, heartbeat and explicit reset/rearm. Stop/reset cancels queued commands and disables retained local PD through the HAL contract. All real motor transport remains inhibited.

Host synthetic and production-gate tests, warning-clean C, UBSan, Cortex-M7 and Teensy builds pass. The new target ELF/HEX is saved without flashing. The **10,000-packet** accepted-core comparison retains a maximum torque difference of **2.307e−15 Nm**; **22 Python tests** and Ruff pass. The new HAL boundary is tested separately and has not been validated against a commissioned driver or full physics with a real driver adapter.

The accepted `config/robot.json`, generated C core, gains and physics inputs remain identical to checkpoint `7db425b`. Their saved **27/27 contact-physics cases, half-timestep convergence and 12 uncertainty cases** remain applicable; no claim is made that those cases were rerun for this unadopted roof candidate. The user's modified M3 CAD bytes remain preserved. The previous 1241-file delivery seal is archived and checked against its Git checkpoint; current delivery includes the new candidates separately.

FreeCAD MCP, native KiCad API/CLI, router, ngspice and compiler tools work. The router uses a virtual display, and KiCad's relocated CLI emits a nonfatal schema-path message; native report contents are inspected. **Blender is not required for M4**. Its MCP can be considered when exterior presentation or mesh workflows justify it. M5's detailed plan remains unwritten.

## Remaining gates and reproducibility

The next decision is the exact actuator/protection specification, which determines the suppression revision, real protected pack and hardware adapter. The roof also needs defensible contact/cooling/other-heat limits and an enforceable energy/temperature policy. Adopt no physical candidate before its full mass/inertia and acceptance work. Digital checks passing does not set `milestone_complete` or `hardware_qualified` true.

```bash
# Project-local environment; FreeCAD bridge running for CAD commands.
bash tools/electronics-env.sh /usr/bin/python3 tools/audit_power_current.py
bash tools/electronics-env.sh /usr/bin/python3 tools/export_electronics_envelopes.py --power-board electronics/power-current/bipe-power.kicad_pcb --output artifacts/milestone-4/completion/power
.venv/bin/python tools/prepare_m4_completion.py
.venv/bin/python tools/call_freecad.py cad/thermal_roof_study.py --timeout 30
.venv/bin/python tools/call_freecad.py cad/thermal_roof_service.py --timeout 30
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.thermal_design
.venv/bin/python tools/audit_completion_sources.py
tools/verify_completion_firmware.sh
.venv/bin/python -m bipe.verify_completion
.venv/bin/python -m bipe.verify_m4
```

The native board and saved DSN/SES are retained. Board regeneration/import/Kelvin steps must be completed before re-exporting its envelopes; see `electronics/power-current/README.md`. Reconciliation and hash sealing occur after artifact changes. Planning, application and candidate checkpoints are `f6bc216`, `aae4674` and `544a16b`. The final current delivery check passes all 1357 sealed files while retaining `milestone_complete=false` and `hardware_qualified=false`; its verification checkpoint is recorded in Git history.
