# Getting started

Everything runs on a Linux machine from the repository root. The simulations
use CPU MuJoCo; only training a policy needs a GPU.

## 1. Install

Python 3.12 and [uv](https://docs.astral.sh/uv/). If your distribution ships
another Python, `uv python install 3.12` fetches one.

```bash
git clone https://github.com/pranay172/BiPe.git bipe && cd bipe
uv venv .venv --python 3.12
uv pip sync requirements.lock --python .venv/bin/python
export PYTHONPATH=.
```

`requirements.lock` holds MuJoCo 3.14, NumPy, SciPy, Matplotlib, imageio,
pupil-apriltags, ONNX Runtime and pytest. That is enough for the classical
controller, `bipe sim`, the test suites and the renders.

To run the learned policy in the full plant, add PyTorch and mjlab to the
same environment (ONNX Runtime is already in `requirements.lock`). The policy's action path is shared with the
training environment, so it imports them even when it runs from the ONNX
graph. mjlab pins an older MuJoCo; the override keeps 3.14:

```bash
echo "mujoco==3.14.0" > /tmp/mujoco-override.txt
uv pip install torch==2.14.1 mjlab==1.6.0 --override /tmp/mujoco-override.txt --python .venv/bin/python
```

Training uses its own environment instead (section 6).

### Drive it yourself

```bash
.venv/bin/python -m bipe sim
```

opens BiPe in an interactive MuJoCo viewer (`bipe/sim.py`). It needs only
`requirements.lock`: the catalogue's ONNX policies
([config/behaviours.json](../config/behaviours.json): drive, sit and stand,
get up) run through ONNX Runtime, with no PyTorch or mjlab. Every key is an
intent ([bipe/intents.py](../bipe/intents.py)); the intent layer picks the
behaviour and its command.

| Key | Intent |
|---|---|
| Up / Down | speed ± 0.1 m/s |
| Left / Right | turn left / right (± 0.2 rad/s) |
| Page Up / Page Down | stand taller / crouch |
| Home | stop |
| End | sit |
| Insert | stand, or get up after a fall |
| Enter / F9 | a 2 N s push from behind / in front |
| F8 | a 7 N s shove from in front, which knocks it over |
| F10 / F11 / F12 | moves: crouch bob, spin and stop, wiggle |
| Delete | reset to standing |

A gamepad also works: left stick for speed, right stick to turn, A to sit, B to
stand or get up, X to push, Y for the next move. `--start sit`, `front_lean` or
`back_lean` starts in a fallen state from the get-up library.

What it runs: the M14 training plant (`bipe.m13.plant`, primitive shapes, the
spring hip belts and knee dead band the policies were trained on) with exact
sensor readings delayed 5 ms, the contract's 20 ms policy step and 1 kHz leg and
wheel loops, and the RO40 and GL40 drive models. A simple stand-in for the
supervisor starts the get-up policy after a fall and hands a get-up or a stand
back to drive once the body is level and still for half a second. It runs
about three times faster than real time on one core, so the viewer paces it to
real time. It is for driving the robot around; the gates are judged on the
full plant (section 5).

Without a display:

```bash
.venv/bin/python -m bipe sim --headless                        # scripted tour, about 15 s
MUJOCO_GL=egl .venv/bin/python -m bipe sim --headless --video local/sim/tour.mp4
```

The tour drives at 0.6 m/s, turns, stops, takes a 2 N s push, plays the crouch
bob, sits, stands, is shoved over and gets up, and prints a verdict for each
step (exit status 0 when all pass).

## 2. Run a shot

`bipe.demo record` runs one full-plant scene and saves the robot's poses at
50 fps to `local/demo/` (gitignored). The plant, sensor model and C firmware
are the ones the acceptance suites use.

```bash
.venv/bin/python -m bipe.demo record push_recovery
.venv/bin/python -m bipe.demo record push_recovery \
    --onnx artifacts/m14/export/locomotion/policy.onnx --controller policy
```

The first line uses the classical controller and the second the M14 policy.
Each 12 s scene takes about 2 minutes on 4 CPU cores. The command prints
whether the robot fell and its peak pitch and roll.

Scenes are defined in [`bipe/demo/scenes.py`](../bipe/demo/scenes.py):

| Scene | What happens |
|---|---|
| `intro_orbit` | Standing and balancing |
| `drive_turn_crouch` | Drive to 0.8 m/s, turn, stop, crouch, stand |
| `push_recovery` | Pushes from behind and in front |
| `push_while_driving` | A push while driving |
| `slope` | Driving on a 10° slope |
| `moves` | Crouch bob, spin and stop, wiggle (record with the policy) |
| `compare_delay_push` | Pushes under 15 ms sensor delay and doubled noise: the classical controller falls, the policy holds |
| `variant_*` | Heavier body, low friction, wheel lag, sensor delay and noise |

## 3. Render to video

```bash
MUJOCO_GL=egl .venv/bin/python -m bipe.demo render local/demo/push_recovery_classical.npz
```

This replays the poses with the CAD render meshes and writes an mp4 beside
the `.npz`. It needs an OpenGL context: EGL on a machine with a GPU driver,
or `MUJOCO_GL=osmesa` with the OSMesa library (`libosmesa6` on Debian and
Ubuntu) on a headless machine. Compositing (title cards, grids, cuts) is in
[demo/README.md](demo/README.md).

## 4. Tests

```bash
.venv/bin/python -m pytest -q
make -C firmware test          # the C core's own tests (needs gcc)
```

The camera and perception tests render frames, so they need the same OpenGL
context as section 3. The M11 driver-codec parity tests are skipped unless
mjbots' `moteus` package is installed ([development.md](development.md)), and
the policy tests need the packages from section 1.

## 5. Run the test suites

The suites behind [results.md](results.md). They are long: run one at a time,
with at most as many workers as you have cores.

```bash
# M10 robustness and acceptance, classical controller (107 cases)
nice -n 10 .venv/bin/python -m bipe.robustness --profile m10 --controllers current --workers 8

# M12 speed cases, classical controller
.venv/bin/python -m bipe.m12.speed

# The deployed policy on the acceptance and speed cases
.venv/bin/python -m bipe.m13.full_plant acceptance \
    --checkpoint artifacts/m14/checkpoints/finetune_s1/model_9998.pt --workers 4
.venv/bin/python -m bipe.m13.full_plant speed \
    --checkpoint artifacts/m14/checkpoints/finetune_s1/model_9998.pt --workers 4

# Score a checkpoint against every M14 gate (hardware timing, stress, acceptance, speed)
.venv/bin/python -m bipe.m14.evaluate run artifacts/m14/checkpoints/finetune_s1/model_9998.pt

# M15: get-up from 60 randomised falls, the three moves, following with the policy
.venv/bin/python -m bipe.m15.getup_eval --getup artifacts/m15/checkpoints/getup_s1/model_12997.pt
.venv/bin/python -m bipe.m15.moves --checkpoint artifacts/m14/checkpoints/finetune_s1/model_9998.pt
.venv/bin/python -m bipe.m15.following --checkpoint artifacts/m14/checkpoints/finetune_s1/model_9998.pt \
    --link rendered --limit acceleration_m_s2=1.1
```

Following with the rendered link is slow (tens of minutes per run on 4 cores).

Each runner writes its records under `artifacts/` or `local/`, with the
configuration it ran with, and compares against the committed records where
there are any. The module docstrings describe each runner's options.

## 6. Train a policy

Training uses mjlab on MuJoCo Warp, with rsl_rl PPO, on a simplified training
plant fitted to the full plant. It runs in a separate environment with its own
lock file, because mjlab pins MuJoCo 3.11:

```bash
uv venv .venv/rl --python 3.12
uv pip sync requirements-rl.lock --python .venv/rl/bin/python
```

The M14 recipe, from scratch, then the fine-tune that made the deployed policy:

```bash
.venv/rl/bin/python -m bipe train --task locomotion --seed 1
.venv/rl/bin/python -m bipe train --task finetune --seed 1 --iterations 2000 \
    --resume-run <the seed 1 run> --resume-checkpoint model_7999.pt
```

The first is 8000 iterations at 4096 parallel worlds, about 5.4 to 5.9 hours
on an 8 GB laptop GPU (RTX 5050). The fine-tune adds about 1.5 hours. Runs go
to `local/m14/runs/bipe_m13/`.

The get-up policy continues from the locomotion policy on a library of fallen
states (`artifacts/m15/fallen_states.npz`, made by `python -m bipe.m15.fallen`):

```bash
.venv/rl/bin/python -m bipe train --task getup --seed 1 --iterations 3000 \
    --resume-run <the fine-tune run> --resume-checkpoint model_9998.pt
```

Score a trained checkpoint on the full plant with `bipe.m14.evaluate`
(section 5), after copying it inside the repository. Export it to ONNX, a
manifest and the firmware's C headers with
`.venv/rl/bin/python -m bipe.m13.export --checkpoint PATH` (`--policy getup` for
the get-up policy) and check parity with
`python -m bipe.m13.export_check parity`; the M13 to M15 milestone
documents describe both.

## Where to go next

- [results.md](results.md): what the suites measure and what they show.
- [repository-layout.md](repository-layout.md): where each part lives.
- [development.md](development.md): CAD, boards, firmware images and renders.
