# Policy contract, version 1

Every BiPe policy reads the same observation and writes the same action at the same
rates. The supervisor can therefore switch between behaviours (get up, then drive)
without changing anything in sensing or actuation. Microduck does the same: its policies
share one observation layout and are swapped at runtime.

`python -m bipe.m16.catalogue check` compares each policy in the catalogue
([config/behaviours.json](../config/behaviours.json)) with this contract and fails on
any difference. The contract's hash is `be4d79c683b4`.

## Timing

| | |
|---|---|
| Policy period | 20 ms (every 4th 5 ms control tick); the action is held for the period |
| Leg servo | 1 kHz PD on the local encoder: torque = 8.0 (target − q) − 0.05 q̇ |
| Wheel speed loop | 1 kHz PI on the hub speed, integral clamped to the hub's peak torque |

## Observation (25 values, float32)

| Index | Name | Unit | Meaning |
|---|---|---|---|
| 0–2 | gyro | rad/s | body rates from the latest delayed IMU packet |
| 3–5 | accelerometer | m/s² | specific force from the latest delayed IMU packet |
| 6–9 | leg_encoders | rad | [hip, knee] × [left, right] encoder angles less nominal(posture) |
| 10–13 | leg_speeds | rad/s | encoder-derived leg joint speeds (estimator, or fallen mode's filtered differences) |
| 14–15 | wheel_speeds | rad/s | wheel speeds [left, right] |
| 16–21 | last_action | 1 | the previous policy output |
| 22–24 | command | m/s, rad/s, 1 | rate-limited forward speed, yaw rate, (posture label − 47.5) / 17.5 |

The network carries its own normaliser, (x − mean) / (std + 0.01).

## Action (6 values)

| Index | Law |
|---|---|
| 0–3 | leg targets [hip, knee] × [left, right] = nominal(posture) + 0.5 rad × action |
| 4–5 | wheel speed targets = 40 rad/s × action + rolling feedforward from the speed and yaw-rate command |

## Command

Rate limits at the policy step: speed 1.25 m/s², posture label 20 per second, yaw rate
none. Trained ranges: speed −0.5 to 1.2 m/s, yaw rate ±1 rad/s, posture label 30 to 65.
The M16 command shaper (`bipe.m16.shaper`) sits in front of these limits and does not
change the contract.

## Requests

A request is a value in the command's posture slot outside the trained 30 to 65 range.
Only a policy whose manifest lists the request under `requests` accepts it; the others
were never trained on it, and the intent layer never sends it to them. Requests sit
outside the contract hash: adding one to a new policy does not change contract v1 for the
others.

| Request | Posture slot | Law | Accepted by |
|---|---|---|---|
| sit | label 100 (normalised 3.0) | set at once, with no rate limit, with speed and yaw rate 0. The leg nominal uses the label clamped to 65. Leaving it sets the label back to 40, and the usual 20 per second limit applies from there | sit_stand |

`python -m bipe.m16.catalogue check` checks every listed request against this table.

## Each export

An export directory holds `policy.onnx`, `manifest.json` (the full statement of the
above plus the policy's identity: checkpoint, its hash and commit, training task and
plant), `SHA256SUMS`, `parity.json` (torch, ONNX Runtime, host C and the Cortex-M7 image
against the float64 actor) and `budget.json` (forward-pass time and memory on the
Teensy). The catalogue entry adds the behaviour's training task and gate results.

A change to any field above is a new contract version: every policy is re-exported and
the firmware's observation builder changes with it.
