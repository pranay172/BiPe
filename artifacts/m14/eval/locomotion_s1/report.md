# M14 policy evaluation

- `s1_model_7999`: `local/m14/seeds/s1/model_7999.pt` (sha256 c6be8876cdfc)
- `s2_model_7999`: `local/m14/seeds/s2/model_7999.pt` (sha256 58410065b2ee)

| | s1_model_7999 | s2_model_7999 | settle (baseline) |
|---|---|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | 84/98 | 89/98 | 74/98 |
| Gate 2: belt range recovered (>= 6/8) | 8/8 | 8/8 | 2/8 |
| Gate 2: pushes recovered (30/30) | 30/30 | 30/30 | 30/30 |
| Gate 2: physical failures, acceptance + speed (0) | 0 | 0 | 0 |
| Gate 3: speed policy rule (>= 40 of 43 judged) | 26/43 | 29/43 | 31/43 |
| Gate 3: axle worst steady p95, mm | 6.90 | 5.10 | 9.80 |
| Gate 3: judged cases without a steady segment (no axle check) | 0 | 0 | 0 |
| Speed rule with axle check case 10 mm | 27/43 | 30/43 | 31/43 |
| Speed rule with axle check case 5 mm | 26/43 | 30/43 | 4/43 |
| Gate 5: training stress falls (<= 5 %) | not run | not run | 20.3% (13/64 x 10.0 s; 95 % 12.3%-31.7%) |
| Gate 5: full-plant stress survived (24/24) | 9/24 | 9/24 | 12/24 |
| Survived, 98 policy cases | 98/98 | 98/98 | 98/98 |
| Recovered, 79 matrix cases (M13 headline) | 70/79 | 71/79 | 60/79 |
| Speed: survived and recovered | 35/44 | 39/44 | 37/44 |
| Speed: classical joint-error rule | 0/43 | 0/43 | 0/43 |
| Hardware timing: belt recovered | 8/8 | 8/8 | 8/8 |
| Hardware timing: PR #5 handover cases recovered | 8/10 | 8/10 | 8/10 |
| Hardware timing: physical failures | 0 | 0 | 0 |
| Acceptance verdicts at the band margin | 10 | 25 | 16 |
| Stage hardware | complete (exit 0, None) | complete (exit 0, 3dca5299) | not run |
| Stage stress | complete (exit 0, None) | complete (exit 0, 3dca5299) | not run |
| Stage speed | complete (exit 0, 3dca5299) | complete (exit 0, 3dca5299) | complete (exit 0, a6b6cbae) |
| Stage acceptance | not run | complete (exit 0, 3dca5299) | complete (exit 0, None) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `s1_model_7999`

- Gates passed: 2_recovered_98, 2_belt, 2_pushes, 2_physical_failures
- Gates missed: 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_hold; lost wheel_lag_25ms_combined
- speed against settle: gained drive_ramp, drive_step, drive_step_half_timestep, replay_start_stop; lost arc_r1, arc_r2, reverse, slope_down_10, slope_up_4, weave, wheel_lag_25ms_drive_step, wheel_lag_25ms_replay_corner_90, wheel_lag_25ms_weave
- Physical failures: none
- Not recovered: sensors_hold, sensors_tilt_negative, sensors_tilt_positive, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8, wheel_lag_25ms_combined, wheel_lag_25ms_drive
- At the band margin (rerun with perturbations before deciding on them): mismatch_body_com_x_m_-0.005_hold, mismatch_body_com_z_m_0.003_hold, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_8, slope_up_10, slope_up_4, slope_up_8
- Speed cases failing the policy rule: arc_r1 (yaw_rate); arc_r2 (overshoot, yaw_rate); push_fast_-2 (axle_depth_p95); replay_corner_90 (path); replay_curve_r1 (path); replay_straight_1.0 (path); reverse (overshoot); slope_down_10 (survives_and_holds); slope_down_4 (survives_and_holds); slope_up_10 (survives_and_holds); slope_up_4 (survives_and_holds); slope_up_8 (survives_and_holds); spin (yaw_rate); weave (overshoot, yaw_rate); wheel_lag_25ms_drive_step (survives_and_recovers); wheel_lag_25ms_replay_corner_90 (survives_and_recovers); wheel_lag_25ms_weave (survives_and_recovers)
- Stress set falls: push_back_c0, push_back_c1, push_back_c2, push_back_c3, push_back_c4, push_back_c5, push_back_c6, push_back_c7, push_fwd_c0, push_fwd_c2, push_fwd_c3, push_fwd_c4, push_fwd_c5, push_fwd_c6, push_fwd_c7

## `s2_model_7999`

- Gates passed: 2_recovered_98, 2_belt, 2_pushes, 2_physical_failures
- Gates missed: 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_hold, sensors_hold, sensors_tilt_negative, sensors_tilt_positive, slope_down_4, slope_up_4; lost mismatch_body_com_x_m_-0.005_hold
- speed against settle: gained drive_step_half_timestep, push_fast_-2, slope_down_4, slope_up_8; lost arc_r1, arc_r2, push_fast_+2, reverse, slope_down_10, weave
- Physical failures: none
- Not recovered: mismatch_body_com_x_m_-0.005_hold, slope_down_10, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_8, wheel_lag_25ms_drive
- At the band margin (rerun with perturbations before deciding on them): backlash_drive_0.3, backlash_drive_0.6, backlash_hold_0, belt_hold_h0.1125_n0.225_k0, mismatch_body_com_x_m_-0.005_hold, mismatch_body_com_z_m_-0.003_hold, mismatch_body_mass_scale_0.85_combined, mismatch_body_mass_scale_0.85_hold, push_drive_+2, push_drive_-1.5, push_hold_+1, push_hold_+1.5, push_hold_+2.5, push_hold_-2, push_low_+1.5, push_low_-1, push_low_-1.5, push_low_-2.5, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_8
- Speed cases failing the policy rule: arc_r1 (overshoot, yaw_rate); arc_r2 (overshoot, yaw_rate); drive_ramp (overshoot); drive_step (overshoot); push_fast_+2 (axle_depth_p95); replay_corner_90 (path, peak_pitch); replay_curve_r1 (path); replay_start_stop (path); replay_straight_1.0 (path); reverse (overshoot, peak_pitch); slope_down_10 (survives_and_holds); slope_up_10 (survives_and_holds); spin (yaw_rate); weave (overshoot)
- Stress set falls: push_back_c0, push_back_c1, push_back_c2, push_back_c3, push_back_c4, push_back_c5, push_back_c6, push_back_c7, push_fwd_c0, push_fwd_c2, push_fwd_c3, push_fwd_c4, push_fwd_c5, push_fwd_c6, push_fwd_c7
