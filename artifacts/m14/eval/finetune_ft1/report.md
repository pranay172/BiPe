# M14 policy evaluation

- `ft1`: `artifacts/m14/checkpoints/finetune_s1/model_9998.pt` (sha256 8cfe26ae6f50)
- `ft2`: `artifacts/m14/checkpoints/finetune2_s1/model_10997.pt` (sha256 2c2a0c97e856)

| | ft1 | ft2 | settle (baseline) |
|---|---|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | 89/98 | 86/98 | 74/98 |
| Gate 2: belt range recovered (>= 6/8) | 8/8 | 8/8 | 2/8 |
| Gate 2: pushes recovered (30/30) | 30/30 | 30/30 | 30/30 |
| Gate 2: physical failures, acceptance + speed (0) | 0 | 0 | 0 |
| Gate 3: speed policy rule (>= 40 of 43 judged) | 35/43 | 33/43 | 31/43 |
| Gate 3: axle worst steady p95, mm | 8.00 | 6.80 | 9.80 |
| Gate 3: judged cases without a steady segment (no axle check) | 0 | 0 | 0 |
| Speed rule with axle check case 10 mm | 37/43 | 35/43 | 31/43 |
| Speed rule with axle check case 5 mm | 3/43 | 6/43 | 4/43 |
| Gate 5: training stress falls (<= 5 %) | not run | not run | 20.3% (13/64 x 10.0 s; 95 % 12.3%-31.7%) |
| Gate 5: full-plant stress survived (26/26) | 25/26 | 26/26 | 12/24 |
| Survived, 98 policy cases | 98/98 | 98/98 | 98/98 |
| Recovered, 79 matrix cases (M13 headline) | 70/79 | 70/79 | 60/79 |
| Speed: survived and recovered | 38/44 | 38/44 | 37/44 |
| Speed: classical joint-error rule | 0/43 | 0/43 | 0/43 |
| Hardware timing: belt recovered | 8/8 | 8/8 | 8/8 |
| Hardware timing: PR #5 handover cases recovered | 8/10 | 8/10 | 8/10 |
| Hardware timing: physical failures | 0 | 0 | 0 |
| Acceptance verdicts at the band margin | 5 | 3 | 16 |
| Stage hardware | complete (exit 0, 5f13ade4) | complete (exit 0, 3d17bcd7) | not run |
| Stage stress | complete (exit 0, 8d0ca371) | complete (exit 0, 3d17bcd7) | not run |
| Stage acceptance | complete (exit 0, 3d17bcd7) | complete (exit 0, 3d17bcd7) | complete (exit 0, None) |
| Stage speed | complete (exit 0, 3d17bcd7) | complete (exit 0, 3d17bcd7) | complete (exit 0, a6b6cbae) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `ft1`

- Gates passed: 2_recovered_98, 2_belt, 2_pushes, 2_physical_failures
- Gates missed: 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_hold, sensors_hold, sensors_tilt_negative, sensors_tilt_positive, wheel_lag_25ms_drive; lost none
- speed against settle: gained drive_ramp, drive_step_half_timestep, replay_start_stop, slope_up_10, slope_up_8, spin; lost slope_down_10, slope_down_8
- Physical failures: none
- Not recovered: slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8
- At the band margin (rerun with perturbations before deciding on them): slope_down_10, slope_down_8, slope_hold_10, slope_hold_8, slope_up_10
- Speed cases failing the policy rule: drive_step (stop); push_fast_-2 (axle_depth_p95); replay_corner_90 (path); replay_curve_r1 (path); replay_straight_1.0 (peak_pitch); slope_down_10 (axle_depth_p95, survives_and_holds); slope_down_4 (axle_depth_p95); slope_down_8 (axle_depth_p95, survives_and_holds)
- Stress set falls: push_back_d6

## `ft2`

- Gates passed: 2_recovered_98, 2_belt, 2_pushes, 2_physical_failures, 5_stress_physical_failures
- Gates missed: 3_speed_policy_rule
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_hold, wheel_lag_25ms_drive; lost none
- speed against settle: gained drive_ramp, drive_step, drive_step_half_timestep, slope_down_4, slope_up_10, slope_up_8; lost reverse, slope_down_10, slope_down_8, weave
- Physical failures: none
- Not recovered: sensors_hold, sensors_tilt_negative, sensors_tilt_positive, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8
- At the band margin (rerun with perturbations before deciding on them): slope_down_10, slope_hold_10, slope_up_10
- Speed cases failing the policy rule: push_fast_-2 (axle_depth_p95); replay_corner_90 (path); replay_curve_r1 (path); replay_start_stop (path); replay_straight_1.0 (path); reverse (overshoot); slope_down_10 (axle_depth_p95, survives_and_holds); slope_down_8 (axle_depth_p95); spin (yaw_rate); weave (yaw_rate)
