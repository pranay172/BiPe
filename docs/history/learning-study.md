# Controlled residual-learning workflow

M7 tests the refinement bundle declared in
[the milestone plan](milestones/07-learning-refinement-plan.md). It retains the
physical model, sensors, classical C controller, 42 actor observations and six
bounded residual actions from M6. The policy remains a hybrid component that
requires classical control. Classical remains the default; hardware and MCU
inference remain unqualified regardless of the study outcome.

## Fixed experiment and numerical execution

`config/learning-study.json` declares three independent PPO seeds: 20261020,
20261021 and 20261022. Each receives 192 updates × 128 steps × 8 native
environments: **196,608 transitions per seed**, 589,824 in the production study.
All seeds have the same budget and eligible checkpoints at updates 64, 128
and 192. An interrupted preflight with a metrics-serialization failure is
retained separately and excluded from production comparisons.

Each episode samples hold, drive, turn or combined/crouch with probability
0.25. Hold episodes last 30/45/60 seconds; moving episodes include reversal,
stopping and final holds. The mixture is per episode reset. Transition shares
differ because durations differ. Every reset stores the complete task table,
actual initial offset, sensor seed and sampled physical/sensor assumptions.
These copied-plant assumptions do not establish hardware uncertainty bounds.

The actor is a tanh MLP with two 32-unit hidden layers and a zero-initialized
final layer. Its initial deterministic output is exactly zero. Gaussian
exploration begins at standard deviation 0.15. Host NumPy FP32 actor, critic
and pre-tanh likelihood math use a recorded PCG64 action RNG. CUDA performs
PPO optimization with explicitly configured JAX `highest` matrix precision.
Host/CUDA mean, critic and log-density checks are recorded initially and at
every eligible checkpoint; portable actor export is profiled independently.

Stored pre-tanh Gaussian densities produce the correct PPO likelihood ratio:
the unchanged tanh transform's Jacobian cancels. GAE bootstraps timeouts from
their final observations and stops across both failures and automatic resets.
The discount gamma is 0.999 at 50 Hz, with GAE lambda 0.99. Reward uses negative
tracking, posture, hold, effort and residual costs; its units differ from M6.
Compare physical test scores rather than reward returns between milestones.

Native MuJoCo, the free base, contacts, finite torque/speed envelopes, response
lag and C supervisory gate remain outside the learned policy. Policy requests
arrive at 50 Hz; classical sensing/body control remains 200 Hz and local leg
control remains 1 kHz. Residual caps remain 0.20 N m per leg actuator and
0.06 N m per wheel. Simulator truth measures reward and evaluation; realistic
actor observations derive from sensor estimates and controller state.

## Selection and independent comparisons

The pretraining `study-definition.json` saves source/configuration identities,
budgets and full validation/test task tables. Selection uses eight trials:
four families × two fixed sensor seeds. Each seed's candidate is selected only
from updates 64/128/192 by lower normalized physical validation score plus a
penalty for failed trials. Final testing does not participate in selection.

After all three selections, `candidate-lock.json` binds each weight and
metadata digest. The testing tool authenticates this complete lock before
running 12 independent trials per control: four families × three new sensor
seeds, with different offsets, commands, timing and durations. Classical,
frozen M6 and all three M7 candidates use identical jobs. Each candidate also
attempts the complete inherited 27-case suite and half-timestep convergence.
Exceptions remain failed jobs with configurations and tracebacks; remaining
jobs continue.

Physical score averages relevant normalized components within each family,
then gives the four families equal weight. Components are speed MAE/0.03 m/s,
yaw MAE/0.08 rad/s, stopping displacement/0.1 m, and hold RMS pitch/2 degrees
plus drift/0.1 m. Independent hard gates also check physical contacts, pitch,
roll, joints, motor limits, tracking, stopping and final holds. Authoritative
motor peak/RMS statistics integrate at 1 kHz. Recorded 100 Hz physical traces
and 50 Hz actor traces remain inspectable separately.

`paired-comparison.json` reports every seed, paired metric deltas and the
predeclared advancement gates. A possible improvement requires zero final
test failures, all three inherited suites/convergence, at least 10% median
score improvement and the declared tracking/load regression limits. A
completed negative study is a valid result. Three training seeds and small
paired test sets do not establish broad superiority or hardware readiness.

## Reproduction and inspection commands

Run commands from the project directory. The pinned learning environment
lives at `.venv/rl`; caches remain under `.venv`.

```bash
UV_CACHE_DIR=.venv/uv-cache uv venv --python .venv/bin/python .venv/rl
UV_CACHE_DIR=.venv/uv-cache uv pip sync --python .venv/rl/bin/python requirements-rl.lock
bash tools/rl-env.sh -m pytest -q
```

The production study uses `artifacts/milestone-7`. Preserve completed and
partial outputs. To reproduce training, use a fresh study folder and pass that
same folder to the subsequent tools:

```bash
bash tools/rl-env.sh -m bipe.learning_study_train --require-gpu \
  --output artifacts/experiments/m7-reproduction
bash tools/rl-env.sh tools/evaluate_learning_study.py \
  --output artifacts/experiments/m7-reproduction --workers 6
bash tools/rl-env.sh tools/profile_learning_study.py \
  --output artifacts/experiments/m7-reproduction
bash tools/rl-env.sh tools/analyze_learning_study.py \
  --output artifacts/experiments/m7-reproduction
MUJOCO_GL=egl bash tools/rl-env.sh tools/render_learning_study.py \
  --output artifacts/experiments/m7-reproduction
```

The evaluator supports `--part testing` and `--part acceptance` for separate
fresh output stages. The profile measures portable FP32 math and host actor
latency; it does not measure an MCU or the complete control loop. Training
retains optimizer and action/minibatch RNG checkpoints, but exact mid-run
resume is not supported because native worker state is not checkpointed.
Reproduction restarts from the recorded seeds and complete configuration.

The renderer requires completed locked tests. It plots per-seed rewards,
critic explained variance, mean requested residuals and near-limit fractions,
plus matched family scores and stopping distances. It chooses the median
test-score seed for presentation unless `--seed` is supplied. This choice
does not select new weights or change controller adoption. The replay copies
an actual combined test's saved native states and renders them without
integrating new physics. `replay/contact-sheet.png` exposes nine sampled
frames; `review.json` records input/output hashes and encoded frame count.

The tool initially records `inspected:false`. After actually viewing both
plots and the contact sheet, record concrete findings with:

```bash
bash tools/rl-env.sh tools/render_learning_study.py \
  --output artifacts/experiments/m7-reproduction --mark-inspected \
  --finding 'Describe the observed plots, pose/contact frames and any limitations.'
```

Do not substitute that example sentence for actual inspection findings.
Replay rendering is presentation evidence rather than physical acceptance.

## Evidence preservation and completion

The unchanged M6 verifier remains bound to its original 698-file seal. M7
authenticates checkpoint `046782e` and archives only its sealed README/project
overview before updating those live documents. A strict two-document mapped
view lets the unchanged legacy verifier check historical identities. Core
sources, physical inputs, sealed evidence and the user's modified M3 CAD save
remain at their original paths and bytes. M6 verification is never overwritten.

For the repository's designated M7 study, check preservation, then verify or
seal completed evidence:

```bash
.venv/bin/python tools/verify_learning_study.py --preservation-only
.venv/bin/python tools/verify_learning_study.py --seal
.venv/bin/python tools/verify_learning_study.py
```

Sealing requires complete measured outputs and actual visual inspection.
The verifier addresses the designated M7 artifact tree; a separate reproduction
folder is an additional experiment rather than an automatic replacement for
sealed milestone evidence. Study completion, experimental improvement,
candidate locomotion acceptance, classical default and hardware qualification
remain distinct fields. Operational adoption still requires stop/thermal,
inference/application timing and hardware qualification work.
