# M14 policy evaluation

- `ft1_seed11`: `local/m18/runs/bipe_m13/ft1/model_9998.pt` (sha256 8cfe26ae6f50)

| | ft1_seed11 |
|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | not run |
| Gate 2: belt range recovered (>= 6/8) | not run |
| Gate 2: pushes recovered (30/30) | not run |
| Gate 2: physical failures, acceptance + speed (0) | not run (speed not run) |
| Gate 3: speed policy rule (>= 40 of 43 judged) | not run |
| Gate 3: axle worst steady p95, mm | not run |
| Gate 3: judged cases without a steady segment (no axle check) | not run |
| Gate 5: training stress falls (<= 5 %) | 6.2% (62/1000 x 10.0 s; 95 % 4.9%-7.9%) |
| Gate 5: full-plant stress survived (26/26) | not run |
| Survived, 98 policy cases | not run |
| Recovered, 79 matrix cases (M13 headline) | not run |
| Speed: survived and recovered | not run |
| Speed: classical joint-error rule | not run |
| Hardware timing: belt recovered | not run |
| Hardware timing: PR #5 handover cases recovered | not run |
| Hardware timing: physical failures | not run |
| Acceptance verdicts at the band margin | not run |
| Stage training | complete (exit 0, a868166b) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `ft1_seed11`

- Gates passed: none
- Gates missed: 5_training_fall_rate
- Gates incomplete (stage not finished or failed): none
