# Fallback fall while driving: diagnosis (1 November 2026)

Case: `delay_15ms_noise_x2_drive` (15 ms sensor transport delay, 3x the nominal 5 ms, gyro and accelerometer noise x2; drive 0.3 m/s from 2 to 7 s). PR #5's `fault` variant: NaN policy output from 5 s, so the firmware hands over to the classical controller. Branch main a33839f, cloud full-plant runs with the C runtime (`bipe.m13.handover`) and the C forward pass.

## Reproduced
- `fault` at 5 s: handover at 5.005 s, falls at 5.786 s (pitch 20.03°), the same as the committed record.
- The classical controller alone passes the case (peak 10.69°), as the M10 acceptance says.

## What the fall is
After the handover a ~8-10 Hz pitch oscillation grows in a few cycles until the wheels saturate. The knee joins it (±1.5° at the same frequency); in the classical-only run the knee sits still (stiction holds it 1.3° off its reference).

## The handover is not special: the fallback's basin is tiny with 15 ms delay
| Run (classical after the handover unless named) | Result |
|---|---|
| Classical alone | recovers, peak 10.7° |
| Classical alone, push 0.3 / 0.6 / -0.6 N·s at 4 s (mid-drive) | recovers |
| Classical alone, push 1.0 N·s at 4 s | **falls** at 5.98 s |
| Handover at 0 s (no policy step) | recovers, peak 10.67° (identical to classical alone) |
| Handover after 25 ms of policy (standing still, before the drive) | **falls** at 1.76 s |
| Handover after 100 ms of a policy that outputs zero actions | recovers |
| Handover after 25 ms, sensor delay 5 ms instead of 15 ms | recovers |
| Handover after 25 ms, belt lash 0 | falls |

So 25 ms of real policy control (a 2° pitch, 2° knee blip) is enough to tip the classical controller into the growing oscillation when the sensors are 15 ms late. The delay is decisive; lash is not.

## Handover-side fixes tried (all at 5 s unless noted; none recovers)
| Change at handover | Result |
|---|---|
| Seed vref (measured speed), heading, previous command (PR #5 review) | falls 5.61 s |
| Leg ramp from last policy targets to posture 40, 0.3 s / 1.0 s (cosine, with qdref) | falls 5.25 s / with seeding 5.40, 5.87 s |
| Bumpless: vref = command, acceleration 0, speed integral set so the pitch reference starts at the measured pitch | falls 5.76 s; with ramps 5.97, 5.99 s; at 25 ms falls 2.19 s |
| Stop on handover (command 0) + bumpless | survives, peak 19.3°, not recovered; at 25 ms falls 2.70 s; + 0.5 s ramp falls |
| Leg feedforward from the reference / local encoders / constant / zero | falls (zero: 25 ms case recovers, 5 s case falls at 14.1 s) |
| Classical pitch-rate gain 0.035 / 0.05 / 0.1 (from 0.07) after handover | falls / survives 17.0° not recovered / falls |
| Speed loop gains halved after handover | falls 5.25 s |

## Conclusion
The fall is a robustness limit of the classical controller under the 15 ms delay stress, which the handover exposes; it is not something the handover adds that a smoother switch can remove. A fix has to make the classical controller tolerate that delay with margin (it should then also survive a 1.0 N·s push in this case), and that changes the default controller, so the M10 acceptance would need a rerun.


## The fix and the lag probe (added for the PR #9 review)
The fall turned out to be the hub's wheel torque reacting on the knee drive through the 1:1 knee belt (see the M13 doc, "Fallback fall while driving"). The fix subtracts `knee_wheel_reaction` (1.0) times the wheel torque from the knee feedforward.

Lag probe on `wheel_lag_25ms_drive` (25 ms hub torque lag, an M10 stress case that fell before the fix too): knee feedforward put through a first-order lag of time constant tau (`KNEEFF=-1 KNEETAU=tau`, `push.py` / `diag.py`):

| tau | `wheel_lag_25ms_drive` | 1.0 N·s push in `delay_15ms_noise_x2_drive` |
|---|---|---|
| 0 (the committed fix) | falls 3.43 s, forward | recovers, peak 11.92° |
| 0.01 s | falls 3.78 s | recovers, 11.88° |
| 0.025 s (matched to the case's lag) | falls 7.17 s, backward (main: 7.21 s, backward) | recovers, 12.23° |
| main (no term) | falls 7.21 s, backward | falls 5.98 s |

So with the feedforward lagged to match the hub, the 25 ms case falls the same way as on main and the delay fix still holds. The plant model's real hub lag is 0.8 ms, so a matched filter has no effect in the nominal model; tau is left out of the firmware until the bench measures the hub lag.

## Files
- `tools/diag.py`: one acceptance case with the handover runtime and variant switches (`--variant --fault-s --seed --ramp --zero --bumpless --stop --out`; env LASH0, NODELAY, GAINS, LEGPD, FF0, FFREF, FFLOCAL, FFCONST, KNEEFF, KNEETAU).
- `tools/variants.py`: the handover variants (Smooth runtime controller) and the KneeFF classical controller with the lag filter.
- `tools/push.py`: classical-only push probe (`push.py <case> <impulse N·s>`, GAINS / KNEEFF / KNEETAU).
- `tools/sweep.sh`: the gain sweep. `tools/show.py`: prints a saved trace.
- `tools/queue.sh`: the reruns behind the committed records (handover subsets, M12 speed).
- `tools/speed_merge.py`: rebuilds `artifacts/m12/speed/summary.json` from per-case results; used after a container restart split the speed suite into two runs (27 + 17 cases).
Run from the repo root with the `.venv` and `MUJOCO_GL=disable`.
