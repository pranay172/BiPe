# M8 results — current-plant robustness and control

4 October 2026. **Keep original cascade gains; correct the wheel cap to
1.5 N·m.** M4 geometry, mass and inertia remain unchanged. Plan: `0506ccd`;
implementation: `b9968c8`. M9 has not started; hardware qualification remains held.

396 C/sensors-m4 runs: 70 robustness cases, 27 inherited cases and two
half-timestep checks per configuration.

| Controller / wheel cap | Inherited / convergence | Physical robustness | Largest observed recovery, forward / backward (N·s) | 10° rollback (m) | 0.6° backlash pitch p-p (°) |
| --- | --- | --- | --- | --- | --- |
| Historical cascade / 0.6 N·m | 27/27 / pass | 51/70 | 1.0 / none | 0.078 | 1.00 |
| Retuned cascade / 1.5 N·m | 25/27 / pass | 58/70 | 2.0 / 2.0 | 0.087 | 1.14 |
| Scheduled LQR / 1.5 N·m | 9/27 / fail | 63/70 | none / none | 0.243 | 3.46 |
| **Default: original cascade / 1.5 N·m** | **27/27 / pass** | **59/70** | **1.5 / 1.5** | **0.078** | **1.00** |

Recovery requires final-two-second samples within 2° pitch error and 0.03 m/s.
Observed passing pulses have holes: contiguous backward recovery margins are
respectively 0, 0.5, 0 and 0 N·s. Physical hold survival margins are 1/1 N·s
historically and 2/2 for the others.
All four physically complete hold, uphill and downhill driving through 10°.

**Default targets:** strict ±2 N·s recovery is **missed**, with 1.06–1.13 m
catch travel. The 10° rollback threshold is met, but motion remains (3.82°
pitch p-p, 0.124 m/s peak); stationary hold is not established. Backlash amplitude
and all-27/convergence targets are met. No tested lowest-crouch backward push
survives with the default. Retuning fails delayed drive/combined; LQR oscillates
and fails the finer combined task, despite small numerical differences. Both
candidates also exceed the conditional 6.5 A current domain.

[CubeMars AK40-10 V3.0](https://www.cubemars.com/product/ak40-10-v3-0-kv170-robotic-actuator.html)
specifies 1.3 N·m rated / 4.1 N·m peak; derating to 1.5 N·m respects M4's
conditional electrical domain. Default maxima: estimated DC current 5.184 A,
wheel RMS 0.152 N·m, negative mechanical power 27.42 W and gross negative work
8.77 J; pack, driver and dump performance remain unmeasured.

Real slopes retain vertical gravity; mass/COM/friction variations retain nominal
controller parameters. Backlash includes free rotors and coupled inelastic
engagement; near-zero parity and momentum/energy tests pass. LQR uses MuJoCo
transition derivatives at five postures, nine states and wheel lag. Joint lag,
sensor delay and Coulomb friction are tested nonlinearly, outside its Riccati
model. Regenerated gains are identical. Halving dt changes default 0.6° backlash
pitch p-p by 0.015°.

Validation: pytest **52 passed / 4 RL skips**, RL environment **64 passed**;
Ruff, native/ARM builds pass. 10,000-sample C/Python error: ≤1.18e−15 N·m.

Reproduce from the project directory:

```bash
.venv/bin/python -m bipe.robustness --controllers baseline cascade-retuned lqr cascade-corrected --workers 8 --output artifacts/milestone-8
.venv/bin/python -m pytest -q
make -C firmware test arm
.venv/bin/python -m bipe.control_validate --output /tmp/bipe-m8-equivalence
.venv/bin/python -m bipe.robustness_control --output /tmp/bipe-m8-lqr-design
```

Future plants: use `--controllers current`. Evidence: [summary/configurations/10 Hz traces](../../../artifacts/milestone-8/summary.json),
[metrics CSV](../../../artifacts/milestone-8/metrics.csv) and [one plot](../../../artifacts/milestone-8/robustness.png).
