# M14 policy evaluation

- `anchor_c5_s1_model_11997`: `local/m18/ckpt/anchor_c5_s1/model_11997.pt` (sha256 6813a2ce3197)

| | anchor_c5_s1_model_11997 |
|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | 83/98 |
| Gate 2: belt range recovered (>= 6/8) | 8/8 |
| Gate 2: pushes recovered (30/30) | 30/30 |
| Gate 2: physical failures, acceptance + speed (0) | 0 |
| Gate 3: speed policy rule (>= 40 of 43 judged) | 20/43 |
| Gate 3: axle worst steady p95, mm | 8.00 |
| Gate 3: judged cases without a steady segment (no axle check) | 0 |
| Speed rule with axle check case 10 mm | 31/43 |
| Speed rule with axle check case 5 mm | 4/43 |
| Gate 5: training stress falls (<= 5 %) | not run |
| Gate 5: full-plant stress survived (26/26) | 25/26 |
| Survived, 98 policy cases | 98/98 |
| Recovered, 79 matrix cases (M13 headline) | 69/79 |
| Speed: survived and recovered | 38/44 |
| Speed: classical joint-error rule | 0/43 |
| Hardware timing: belt recovered | 8/8 |
| Hardware timing: PR #5 handover cases recovered | 8/10 |
| Hardware timing: physical failures | 0 |
| Acceptance verdicts at the band margin | 3 |
| Stage hardware | complete (exit 0, a868166b) |
| Stage stress | complete (exit 0, a868166b) |
| Stage acceptance | complete (exit 0, a868166b) |
| Stage speed | complete (exit 0, a868166b) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `anchor_c5_s1_model_11997`

- Gates passed: 2_recovered_98, 2_belt, 2_pushes, 2_physical_failures
- Gates missed: 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- Physical failures: none
- Not recovered: mismatch_body_com_x_m_0.005_hold, sensors_hold, sensors_tilt_negative, sensors_tilt_positive, sensors_turn, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8, wheel_lag_25ms_drive
- At the band margin (rerun with perturbations before deciding on them): backlash_hold_0.6_half_timestep, slope_hold_8, slope_up_8
- Speed cases failing the policy rule: arc_r1 (overshoot); arc_r2 (axle_depth_p95, overshoot, plateau, yaw_rate); drive_ramp (axle_depth_p95, overshoot, plateau); drive_step (axle_depth_p95, overshoot, peak_pitch, plateau, stop); drive_step_half_timestep (axle_depth_p95); friction_0p5_drive_step (axle_depth_p95); mass_plus_15pct_drive_step (axle_depth_p95); mass_plus_15pct_weave (axle_depth_p95); push_fast_+2 (axle_depth_p95); push_fast_-2 (axle_depth_p95); replay_corner_90 (path, peak_pitch); replay_curve_r1 (path); replay_start_stop (path, peak_pitch); replay_straight_1.0 (axle_depth_p95, path, peak_pitch); reverse (overshoot); seed_plus_1_drive_step (axle_depth_p95); seed_plus_2_drive_step (axle_depth_p95); slope_down_10 (axle_depth_p95, survives_and_holds); slope_down_4 (axle_depth_p95); slope_down_8 (axle_depth_p95, survives_and_holds); weave (overshoot, yaw_rate); wheel_lag_10ms_drive_step (axle_depth_p95); wheel_lag_25ms_drive_step (axle_depth_p95)
- Stress set falls: push_back_d6
