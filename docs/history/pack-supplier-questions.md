# Pack/protection specification request — prepared, not sent

The candidate is an outsourced 6S1P pack of Molicel P28A cells. Its screened enclosure is 70 × 64 × 50 mm, including insulation, cell interconnects, protection, fuse and terminations. The preserved whole-pack estimate is 326 g; it is not a supplier maximum. We need a complete dimensioned, revision-controlled specification before adopting this layout.

1. Identify exact cell, BMS, fuse and mating connector variants. Give guaranteed assembled maximum dimensions, lead exit/bend space, mass, insulation/interconnect construction, manufacturing tolerances and supported mounting/retention.
2. Resolve the EYBMS PCM-L06S10-554 page's conflicting 6/8 mm thickness, 8/10 A continuous discharge, undervoltage, overvoltage and balancing entries. State the exact ordered variant, maximum mass, temperature derating and equipped NTC functions. Our **64 × 36 × 8 mm / 20 g** model is only a screening assumption.
3. Coordinate cell charge/discharge/temperature limits with a 25.2 V CC/CV charger, per-cell cutoff thresholds and tolerances, balancing, charge/discharge paths, recovery and locked-off behavior. Give permitted regeneration and behavior if charge or discharge protection opens; the BMS must not be assumed a guaranteed brake-energy sink.
4. Give continuous and peak load limits, resistance at temperature/state of charge/age, overcurrent and short-circuit thresholds, tolerances and maximum response times. The electronics study uses a **6.5 A bus design domain**, not a confirmed six-actuator requirement; exact motor consumption is pending.
5. Provide minimum prospective short-loop impedance/current and maximum protection/fuse clearing energy. The screened Littelfuse **0453010.MR** has a 400 A interrupting specification at 32 VDC; 25.2 V / 400 A requires at least **63 mΩ** total loop resistance if that specification is to bound a direct short. Its nominal melting I²t is not a maximum clearing-energy guarantee.
6. Confirm reverse-connection policy, keyed connector/cable specification, insulation temperature, strain relief and protection of the upstream logic/charger ports. Supply drawings and electrical/thermal evidence, including the actual protection PCB and fuse carrier.

This request is saved for review. No vendor has been contacted and no pack has been purchased.
