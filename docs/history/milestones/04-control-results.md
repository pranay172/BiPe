# M4 continuation — full portable control core results

Verified 1 October 2026. This is digital firmware/control evidence under the saved M3 physical assumptions. It is not actuator or hardware qualification.

The portable C core now computes IMU attitude and rolling odometry, filtered posture and joint rates, seven-body COM and equilibrium, posture scheduling, gravity/support feedforward, local leg PD and finite wheel/joint commands. It rejects nonfinite inputs, singular attitude, impossible encoder increments and invalid joint references; it applies no physical pose or velocity clamps. Joint zero/sign mapping and interrupted/frozen stationary gyro calibration have explicit tests.

Python supplies the simulated sensor device bandwidth, noise, quantization, transport delay and the physical motor/contact plant. The 200 Hz estimator/body controller and modeled 1 kHz local leg feedback/control run in C. The C supervisor gates the running controller at the servo rate. Electrical health and the pre-release commissioning interval are explicitly synthetic. The local PD behavior inside the actual AK48 remains unverified.

| Check | Achieved result |
| --- | --- |
| Identical-packet C/Python comparison | 10,000 samples; estimator error below 3e-17, COM/axle error below 6e-17 m, feedforward below 4e-16 Nm, complete control torque below 3e-15 Nm; local rate agreement exact in this comparison |
| Articulated acceptance | All 27 original cases passed using C, including noisy/delayed sensing, drive, turn, crouch, push, combined motion and independent variations |
| Physics convergence | Half-timestep combined case passed; peak-pitch difference <0.05°, heading difference <0.01°, height-range difference <0.05 mm |
| Physical validity | Free base (12 velocities), six finite-effort hinges, zero equality constraints, physical contact; unpowered negative control falls |
| C tests / UBSan | Codec, startup/fault tests plus complete control/calibration/mapping/rejection tests pass; no reported undefined behavior |
| Regression | 22 Python tests pass; Ruff passes |
| MCU target | Real Teensy 4.0 ELF/HEX builds; board-frame diagnostic calibration/estimation included; CAN listen-only, silent HIGH, run output LOW, no transmit implementation |
| Free-base fault traces | Healthy run remains balanced; CAN loss inhibits at 3.016 s for a 3 s injection, IMU invalid/operator stop at 3 s, host loss begins stopping at 3.25 s and inhibits at 3.75 s |

All four shutdown cases fall into forbidden contact after torque removal. The shutdown evidence establishes inhibition under an assumed 20 ms decay; it does not establish fall prevention, safe disconnect transients, stored-current behavior or a hardware emergency-stop performance claim.

Outputs, configurations, source/build fingerprints and measured traces are in `artifacts/milestone-4/continuation/acceptance`, `firmware`, and `physics-faults`. The earlier M3 and initial M4 outputs remain preserved. The delivered actuator profile is still false with unresolved fields null. Exact CAN/current/local-PD mapping, assembled IMU-axis transform, electrical power qualification and commissioning remain gates.
