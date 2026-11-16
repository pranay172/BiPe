# M7 measured residual-learning refinement results

Measured 2 October 2026 against the [M7 plan](07-learning-refinement-plan.md).
The declared three-seed study completed its training budgets, independent tests,
inherited acceptance runs and precision checks. It produces a **negative
improvement finding**: the median physical-score gain is **3.576%**, below the
declared 10% gate, and one seed regresses. Several stopping and paired tracking
gates also fail. The classical controller remains the default and remains
required by every residual actor. Hardware qualification is false.

## Preserved plant and experiment boundary

The six-actuator serial-leg plant, SI configuration, classical C controller,
sensor model, motor envelopes/lag and adopted CAD are unchanged. The robot has
a free base, 12 generalized velocities, six driven hinges and zero equality
constraints. Physical contacts and finite motor limits remain active; no pose
constraints or state clamps hold it upright.

Estimated mass remains **2.499519 kg**, width **284 mm**, and estimated mass
margin **0.481 g**. These are layout and simulation assumptions. Production
interfaces, actuator variants and fabrication qualification remain open.

The [authenticated baseline identity](../../../artifacts/milestone-7/baseline/identity.json)
preserves checkpoint `046782e` and the original **698-file M6 seal**. Only the
sealed README and project overview have archived documentation substitutes;
the unchanged legacy verifier checks that archived view. No physical source or
evidence substitution is allowed. The [M6 preservation check](../../../artifacts/milestone-7/m6-preservation-verification.json)
passes with all 698 identities intact. The user's modified M3 CAD save remains
bound to its original SHA-256. Historical M4/M5/M6 evidence is retained.

The actor remains 42 → 32 → 32 → 6, with **2,630 FP32 parameters** and **10,520
raw weight bytes**. It uses IMU/encoder estimates, commands and classical
controller state, with the same 50 Hz residual interface and caps: **0.20 Nm per
leg actuator** and **0.06 Nm per wheel**. Classical body control runs at 200 Hz;
local control and authoritative motor-load accounting run at 1 kHz. This is a
hybrid controller, not independent learned balance.

## Training, split locking and restart evidence

Each seed received exactly **192 × 128 × 8 = 196,608 transitions**. Together
they produced **589,824 transitions**, **313 completed episodes**, and **zero
terminated training episodes**. Aggregate simulated rollout time is 11,796.48 s.
Recorded training wall time including checkpoint validation totals 1,457.34 s;
this excludes setup, baseline comparisons and final evaluation.

The persistent hold/drive/turn/combined mixture chooses each family with
probability 0.25 per episode reset. Episode lengths vary, so transition shares
need not be equal. Holds last 30/45/60 s; moving episodes include command
reversals, stops and final holds. The declared refinement bundle uses negative
tracking/hold costs, gamma 0.999, GAE lambda 0.99, initial exploration standard
deviation 0.15, learning rate 0.0001 and zero entropy bonus. Final actor layers
start at zero, giving exact zero deterministic residuals initially. This study
does not identify the separate causal effects of these changes.

Native CPU MuJoCo/C/sensor rollouts use host NumPy FP32 actor/critic/density
math and PCG sampling; CUDA performs PPO optimization. JAX matrix precision is
explicitly `highest` throughout. Configurations, randomized resets, complete
commands, observations/actions/likelihoods/rewards, physical reward traces,
bootstrap/GAE data, optimizer/RNG checkpoints and episode outcomes are saved.
Reward-component mean rates exclude terminal penalties; total rewards and
episode flags retain them.

| Training seed | Completed episodes | Selected update | Selection score | Failed selection trials | Selected actor maximum parameter change |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20261020 | 101 | 192 | 0.299998 | 0/8 | 0.040698 |
| 20261021 | 106 | 64 | 0.324967 | 0/8 | 0.024760 |
| 20261022 | 106 | 192 | 0.288347 | 0/8 | 0.035258 |

Only updates 64/128/192 were eligible. Eight fixed selection jobs use four
families and two sensor seeds; twelve final jobs use different sensor seeds,
offsets, command sizes, timings and durations. The [study definition](../../../artifacts/milestone-7/study-definition.json)
records settings/task/source hashes before production training. All three
weight/metadata pairs were [locked](../../../artifacts/milestone-7/candidate-lock.json)
before final tests or inherited acceptance. There is no test-driven checkpoint
or seed replacement.

An interrupted preflight completed 32 updates/32,768 transitions before full
baseline metrics exposed NumPy Boolean JSON serialization failures. Its data
is retained in [diagnostics](../../../artifacts/milestone-7/diagnostics/serialization-attempt-1/attempt-report.json)
and excluded from the production comparison. The correction changes metrics
serialization, not thresholds, physics or learning calculations. All three
production runs restarted with equal budgets. The first 32 saved seed-20261020
rollouts are **bit-identical** to the preflight, as recorded in
[restart reproduction](../../../artifacts/milestone-7/restart-reproduction.json).
This establishes that restart comparison; exact mid-run resume is not supported.

Training reports: [20261020](../../../artifacts/milestone-7/training/seed-20261020/training-report.json),
[20261021](../../../artifacts/milestone-7/training/seed-20261021/training-report.json),
[20261022](../../../artifacts/milestone-7/training/seed-20261022/training-report.json).

## Independent physical comparison

Classical, frozen M6 and all three locked M7 actors were newly evaluated on the
same twelve final tasks: **60 paired trials** in total. The score uses physical
measurements rather than comparing M6/M7 returns with different reward units.
Lower is better. Each family's mean has equal weight; relevant components are
normalized speed MAE/0.03 m/s, yaw MAE/0.08 rad/s, stopping travel/0.1 m, and
hold RMS pitch/2° plus whole-hold maximum drift/0.1 m. A score improvement does
not override a failed physical gate.

| Control | Physical score | Gain over classical | Passed final tasks | Hold score | Drive score | Turn score | Combined score |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Classical | 0.300451 | — | 12/12 | 0.145043 | 0.466130 | 0.329411 | 0.261219 |
| Frozen M6 | 0.382288 | −27.238% | 10/12 | 0.172808 | 0.547921 | 0.481735 | 0.326690 |
| M7 seed 20261020 | 0.264468 | +11.976% | 11/12 | 0.137046 | 0.409767 | 0.271657 | 0.239404 |
| M7 seed 20261021 | 0.318356 | −5.959% | 12/12 | 0.140368 | 0.512848 | 0.340351 | 0.279856 |
| M7 seed 20261022 | 0.289706 | +3.576% | 12/12 | 0.141980 | 0.402747 | 0.379218 | 0.234878 |

The median training-seed gain is **3.576%**, with range **−5.959% to +11.976%**.
Two actors improve the score, while seed 20261021 worsens drive, turn and
combined scores. Family median scores meet the ≤5% regression gate, but that
does not guarantee consistent per-seed or per-case behavior. Three training
seeds and three sensor/offset settings per family describe this sample; they
do not establish statistical significance or hardware reliability.

| Mean physical metric over applicable final tasks | Classical | Frozen M6 | M7 seed 20 | M7 seed 21 | M7 seed 22 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Speed plateau MAE (m/s) | 0.006777 | 0.011447 | 0.003725 | 0.008748 | 0.003368 |
| Yaw plateau MAE (rad/s) | 0.021351 | 0.032382 | 0.018184 | 0.022168 | 0.026020 |
| Maximum stopping travel (mm) | 46.306 | 48.889 | 45.937 | 46.953 | 46.451 |
| Hold RMS pitch error (°) | 0.201926 | 0.266747 | 0.207801 | 0.210297 | 0.209204 |
| Whole-hold maximum drift (mm) | 18.912 | 21.224 | 17.019 | 17.559 | 17.936 |

These are means of per-trial metrics, not worst-case limits. Improved drive
tracking accounts for much of seeds 20/22's gain. Their stopping travel changes
little. All three reduce whole-hold drift but slightly increase hold RMS pitch;
some final-hold drift and combined yaw metrics regress even when overall scores
improve. Full per-case metrics, paired allowances and all violations are in the
[measured comparison](../../../artifacts/milestone-7/paired-comparison.json).

### Threshold misses and advancement gates

The three failed independent tasks complete their full commanded durations;
none reports falling, forbidden contact, excessive saturation or a simulation
exception. Their failed final-hold gate is terminal speed, with **|v| < 0.03 m/s**:

| Control and final task | Terminal speed (m/s) | Absolute excess over threshold (m/s) |
| --- | ---: | ---: |
| M7 seed 20261020, drive / sensor seed 20261211 | +0.030151742 | 0.000151742 |
| Frozen M6, turn / sensor seed 20261212 | −0.033165896 | 0.003165896 |
| Frozen M6, turn / sensor seed 20261213 | −0.038633548 | 0.008633548 |

These are threshold misses, not falls, and remain failures without relaxing the
gate. Final-hold speed/yaw gates inspect the endpoint; pitch is checked over
the final five seconds. Passing does not mean every speed sample in that window
is below 0.03 m/s. Sustained stopping time and travel are separate saved metrics.

Five declared improvement gates fail:

- **All inherited suites/convergence pass:** seed 20261021 fails one inherited
  terminal-speed check; every convergence check passes.
- **Zero independent test failures:** seed 20261020 fails one final task, and
  the frozen M6 comparison fails two. M7 alone already misses this gate.
- **Median score gain ≥10%:** observed gain is 3.576%.
- **Every seed improves:** seed 20261021 regresses by 5.959%.
- **Individual tracking/displacement allowances:** seed 20 has seven violating
  cases, seed 21 eight, and seed 22 four. They comprise combined yaw tracking,
  seed-21 drive/combined speed tracking, seed-21/22 turn yaw tracking and
  particular final-hold drift regressions. These paired regression checks are
  stricter than the absolute task-pass thresholds.

The family-median regression, pitch/joint peak, authoritative motor RMS,
saturation, matched-task and strict FP32 recording/export gates pass. All final
task saturation durations are zero. Passing load comparisons does not qualify
thermal behavior or a hardware stop path.

## Inherited acceptance and convergence

Each selected actor completed the unchanged **27-case protocol**, including
26 powered scenarios and the intentional unpowered fall diagnostic, followed
by half-timestep combined convergence. Initial offsets, pushes, realistic and
ideal sensing, crouching, and individual mass/friction/delay/noise/wheel-lag
variations retain their original thresholds.

| Selected seed | Inherited cases passed | Half-timestep convergence | Failed inherited case |
| --- | ---: | --- | --- |
| 20261020 | 27/27 | Pass | None |
| 20261021 | 26/27 | Pass | `sensors_turn` |
| 20261022 | 27/27 | Pass | None |

Seed 20261021's failed turn has terminal speed **−0.032798964 m/s**, peak pitch
error 0.647672°, final heading error 0.204425°, zero saturation and no forbidden
contact or physical failure. Its speed exceeds the absolute threshold by
0.002798964 m/s. Convergence does not override this failure.

Across the three convergence comparisons, maximum differences are 0.05004°
peak pitch, 0.007561° final heading, 0.05021 mm height range, 0.03445 Nm joint
peak torque, 0.00008208 Nm joint RMS torque and 0.04101 rad/s joint peak speed.
Each actor meets the unchanged convergence bounds. The earlier twelve-case M4
mechanical uncertainty study and electrical stop/thermal analyses have not
been newly qualified for these actors.

Acceptance reports: [20261020](../../../artifacts/milestone-7/acceptance/seed-20261020/acceptance.json),
[20261021](../../../artifacts/milestone-7/acceptance/seed-20261021/acceptance.json),
[20261022](../../../artifacts/milestone-7/acceptance/seed-20261022/acceptance.json).

## Precision, tooling and inspected replay

The complete suite passes **56 tests** in 3.09 s, including task continuity and
split separation, zero-residual physical parity, reward/timeout/GAE semantics,
host sampling/density and realistic full-metric serialization. See the
[test log](../../../artifacts/milestone-7/tooling/tests.log).

The [precision profiles](../../../artifacts/milestone-7/policy-profiles.json) pass for
the actual declared `highest` CUDA math and portable actor. All **589,824 saved
transitions** are checked; saved host log densities recompute with maximum error
zero. The maximum selected-policy JAX/NumPy normalized-action error over 10,000
recorded inputs per actor is **7.45e-8**. Initialization and all eligible
checkpoint actor/critic/density parity checks pass. The largest checkpoint
critic/value and log-density errors are 1.91e-6 and 3.58e-6 respectively.

All **120 final actor-trace files** are checked: twelve independent tasks,
27 inherited cases and convergence per actor. The intentional disabled cases
contain no actor calls. The maximum portable re-inference difference is
**8.94e-8**, residual-request recomputation error is zero, and the saved cadence
and bounds pass. These checks establish the delivered interface and recording
precision, not controller improvement.

Host NumPy actor inference measures 3.797–4.048 µs median and 4.639–9.097 µs p99
across the three profiles. This excludes sensing, classical control and
transport and does not qualify MCU execution. The short native benchmark records
562.80 transitions/s over 512 transitions; it is not a full production throughput
claim or a GPU physics result. See [math diagnostics](../../../artifacts/milestone-7/math-diagnostic.json).

The [training diagnostic plot](../../../artifacts/milestone-7/plots/training-diagnostics.png),
[paired score/stopping plot](../../../artifacts/milestone-7/plots/paired-test-score-and-stopping.png)
and [nine-frame contact sheet](../../../artifacts/milestone-7/replay/contact-sheet.png)
were visually inspected. Labels are readable; near-limit residual fractions
are zero in all three recorded training runs. The replay shows front eyes,
backward-folding knees and wheel contact through drive/turn/crouch in the saved
combined task. It uses the median-test-score actor, seed 20261022, solely for
presentation after candidate locking and testing; it is not a new policy
selection or adoption decision.

The [replay](../../../artifacts/milestone-7/replay/simulation.mp4) renders recorded
native physics states from `test-combined-20261211`: **1,201 frames at 30 fps,
1280 × 720**, approximately 40.03 s. Actual source/output hashes, frame times
and inspection findings are recorded in the [review](../../../artifacts/milestone-7/replay/review.json).
This simplified visual model illustrates one completed case; other cases are
established by their own saved evidence.

## Reproduction, completion and next work

The [workflow](../learning-study.md) gives training, evaluation, profiling,
analysis and replay commands into fresh output folders. Environment and
dependencies remain inside `.venv/rl`. Reproduction restarts from declared seeds;
optimizer/RNG snapshots alone do not restore live physical worker states.

The [M7 verifier](../../../tools/verify_learning_study.py) checks the final evidence
seal, fixed budgets/splits and candidate lock, saved learning/actor data,
complete acceptance evidence including retained failures, precision, replay
inspection and baseline preservation. The [final verification record](../../../artifacts/milestone-7/verification.json)
separates study completion from improvement, inherited acceptance, operational
adoption and hardware qualification. The study can complete with this negative
finding; no failed physical or improvement gate is waived by completion.

```bash
.venv/bin/python tools/verify_learning_study.py
```

The recommended next work is compact design refinement: resolve mass/width
margin and packaging/manufacturing interfaces, then establish a new classical
physical baseline before returning to learned terrain/recovery behavior.
Geometry changes invalidate direct reuse of this plant's controller evidence.
M8 remains an overview proposal until its detailed plan is written immediately
before implementation. Additional training alone is not supported as a remedy
for these inconsistent gains and stopping/hold tradeoffs.
