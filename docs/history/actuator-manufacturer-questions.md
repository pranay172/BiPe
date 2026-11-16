# Manufacturer questions — prepared, not sent

For six AK40-10 V3.0 KV170 modules with driver AK48-2405-2D-A2, please supply a revision-controlled driver installation guide and firmware/protocol specification. Our planned application is a small wheeled biped; these are pre-build questions.

1. What continuous minimum/maximum input voltages and regenerative overvoltage thresholds/durations are allowed? Is a 6S lithium-ion pack, 25.2 V fully charged, permitted? How does the driver handle a BMS disconnect during regeneration?
2. For the exact shipping firmware, what are MIT Kp/Kd/position/velocity/effort bit widths, ranges and SI units? Is the effort field output torque, rotor torque or Iq? What is output Nm per reported Iq, including the current convention and gearbox efficiency?
3. Confirm the 0x29 feedback units, encoder reference frame, multi-turn behavior, wrapping, persistent zero and response after power loss. Does output position represent the load-side encoder directly?
4. Give the enable/disable/calibration/zeroing/startup sequence and CAN IDs. Does the driver autonomously disable on stale commands, after how long, and what happens to torque?
5. Specify the local position/velocity/torque loop rates, latency and how to implement 200 Hz host feedforward/references with a local 1 kHz PD loop. Can torque limits and joint limits be enforced inside the driver?
6. Confirm XT30PW(2+2)-M power/CAN pin numbering and polarity, mating connector, current rating, wire requirements and built-in CAN termination. What CAN bus wiring and total termination are recommended for six modules at 1 Mbit/s?
7. Provide continuous torque/temperature derating, thermal time constants and current consumption at idle/disabled. This is needed to qualify precharge and the pack/BMS.
8. State maximum input capacitance, its tolerance, effective disabled/startup current versus bus voltage, and stored magnetic energy that can return to the DC bus. We need a six-driver precharge/auxiliary-energy bound rather than a typical single-driver value.
9. State maximum disable response and continued positive mechanical work after a stop/disable request, including retained local PD, queued CAN commands, communication loss and bus-power removal. Describe regeneration behavior during disable. Our proposed stopping reservation requires an enforceable work/timing bound; a nominal command timeout is insufficient.

A response would close manufacturer-documentation fields. Actual mapping, startup, stop and thermal behavior still need commissioning measurements before hardware qualification.
