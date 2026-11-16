# Residual RL workflow

M6 trains a small PPO network that requests bounded torque corrections around
the accepted classical C controller. This is a **hybrid controller**. The
network is not an independent balancing controller, and its weights cannot
replace the existing controller. The adopted digital release and diagnostic
firmware remain unchanged. Hardware remains unqualified and unflashed.

## Model and control boundary

`config/robot.json` remains the shared SI physical source. Episode variants
sample copied masses/COM, contact friction, motor response and sensor noise/
delay, without editing that source. A free base, six driven hinges, finite
motor torque/speed envelopes and first-order response remain physical. Only
actuator requests and normalized observation features are limited; simulator
positions and velocities are never stabilized by clamps or constraints.

The classical body/estimator loop remains 200 Hz and the local leg loop 1 kHz.
The policy requests corrections at 50 Hz. Its 42 features are IMU/encoder
estimates, commands, classical controller references/state and previous policy
actions. Positions are rolling odometry, not simulator root coordinates.
Simulator truth computes rewards and independent metrics, and appears in the
original ideal-sensing diagnostic cases. The realistic actor path uses sensor
estimates. Observation names and scaling are in `bipe/rl_contract.py` and the
exported policy metadata.

Six actions lie in [-1,1]. The configured residual caps are 8% of leg peak and
10% of wheel peak: 0.20 Nm per leg actuator and 0.06 Nm per wheel for the adopted
configuration. Classical plus residual requests pass through the existing motor
limit/lag model. C supervisory gating and invalid-signal rejection remain
outside the actor. The modeled healthy electrical/commissioning branch is
synthetic, as in M4, and does not establish real stop or startup performance.

## Training and checks

PPO uses a tanh-transformed Gaussian actor, two 32-unit hidden layers and a
separate critic with the same observable inputs. Stored pre-tanh likelihoods
give the exact new/old probability ratio because the unchanged transform's
Jacobian cancels. Entropy regularization uses the Gaussian entropy proxy; it is
not reported as the transformed action entropy. GAE bootstraps timeouts using
the final observation and stops recursion at both failures and episode resets.

The curriculum progresses from hold/balance to drive/turn, then crouch/combined
motion. Settings and experiment randomization ranges are in
`config/learning.json`. Selected weights must come from a trained full-curriculum
checkpoint using separate validation seeds. Final acceptance cases do not
participate in model selection. Training reward is not locomotion acceptance.

Restore and run from the project directory:

```bash
UV_CACHE_DIR=.venv/uv-cache uv venv --python .venv/bin/python .venv/rl
UV_CACHE_DIR=.venv/uv-cache uv pip sync --python .venv/rl/bin/python requirements-rl.lock
bash tools/rl-env.sh -m pytest -q
bash tools/rl-env.sh -m bipe.rl_train --require-gpu --output artifacts/experiments/rl-new-run
bash tools/rl-env.sh -m bipe.rl_evaluate \
  --policy artifacts/experiments/rl-new-run/policy.npz \
  --output artifacts/experiments/rl-new-acceptance
```

Training and acceptance require fresh output folders. Do not rerun a command
into sealed M0–M6 results. The exported deterministic NumPy actor does not need
JAX/CUDA; the existing MuJoCo environment can evaluate it when the exact C
library, configuration, controller and sensor contract match its metadata.

Native rollouts preserve the established C/device/contact pipeline. CUDA is
used for neural sampling/optimization. The GPU physics microbenchmark measures
a separate, visually stripped model; a short agreement result does not qualify
the entire rollout environment for GPU training. Its mixed-contact backend
warning and compilation/performance results are retained in M6 tooling logs.

Each training run saves the effective configuration, source/physical identities,
all sampled reset settings, measured rollout observations/actions/rewards,
episode outcomes, optimizer/RNG checkpoints, validation and training curves.
Acceptance outputs retain the original physical traces/poses and add actor
observation/action/residual traces with weight hashes. Export parity and host
inference timing are measured separately; neither establishes MCU deadlines.

The M6 export check exposed a CUDA arithmetic difference: default batched JAX
matrix products differ from the NumPy actor by up to 0.000584 in normalized
action. The strict 0.00001 check fails for that backend setting and is retained.
The same JAX network with `highest` matrix precision agrees with the portable
FP32 export within 0.000000239. This qualifies the exported network's FP32
math, while explicitly recording the actual training backend's numerical gap.
Validation selection and both final policy suites use NumPy. Do not describe
the default CUDA execution as passing the strict FP32 comparison. A future
training study should explicitly record its matrix precision setting.

Inspect or verify the saved M6 experiment without retraining:

```bash
.venv/bin/python tools/verify_rl_milestone.py
```

The verifier checks the evidence seal, completed training, export, preserved
M4/M5 baseline, and full evaluated suites. Its `milestone_complete` concerns
the experiment delivery; `candidate_locomotion_acceptance` reports the hybrid
suite separately. Hardware qualification and controller adoption remain false.

## Interpretation and later gates

Compare classical-only, untrained deterministic and trained policies. Keep the
accepted classical controller as the default unless a candidate earns a clear
measured benefit without losing required behavior. Policy-induced changes to
motor work, stopping/thermal behavior and real inference/application timing
require further qualification before any deployment. Passing flat-floor
simulation does not qualify terrain, recovery, jumping or hardware transfer.

Architecture/action-interface changes can require a new environment and policy.
Mass and packaging refinements require updated CAD inertias and re-evaluation;
randomized ranges do not make arbitrary geometry changes covered by training.

Technical basis: [PPO paper](https://arxiv.org/abs/1707.06347),
[MuJoCo MJX documentation](https://mujoco.readthedocs.io/en/stable/mjx.html),
[JAX CUDA installation](https://docs.jax.dev/en/latest/installation.html).
