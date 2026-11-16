# M14 policy evaluation

- `finetune_s2_model_9998`: `local/m15/checkpoints/finetune_s2/model_9998.pt` (sha256 44840c50f4e6)

| | finetune_s2_model_9998 | settle (baseline) |
|---|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | 83/98 | 74/98 |
| Gate 2: belt range recovered (>= 6/8) | 7/8 | 2/8 |
| Gate 2: pushes recovered (30/30) | 29/30 | 30/30 |
| Gate 2: physical failures, acceptance + speed (0) | 0 | 0 |
| Gate 3: speed policy rule (>= 40 of 43 judged) | 25/43 | 31/43 |
| Gate 3: axle worst steady p95, mm | 5.00 | 9.80 |
| Gate 3: judged cases without a steady segment (no axle check) | 0 | 0 |
| Speed rule with axle check case 10 mm | 27/43 | 31/43 |
| Speed rule with axle check case 5 mm | 22/43 | 4/43 |
| Gate 5: training stress falls (<= 5 %) | not run | 20.3% (13/64 x 10.0 s; 95 % 12.3%-31.7%) |
| Gate 5: full-plant stress survived (26/26) | 24/26 | 12/24 |
| Survived, 98 policy cases | 98/98 | 98/98 |
| Recovered, 79 matrix cases (M13 headline) | 64/79 | 60/79 |
| Speed: survived and recovered | 33/44 | 37/44 |
| Speed: classical joint-error rule | 0/43 | 0/43 |
| Hardware timing: belt recovered | 4/8 | 8/8 |
| Hardware timing: PR #5 handover cases recovered | 5/10 | 8/10 |
| Hardware timing: physical failures | 0 | 0 |
| Acceptance verdicts at the band margin | 35 | 16 |
| Stage hardware | complete (exit 0, 567e8d89) | not run |
| Stage stress | complete (exit 0, 567e8d89) | not run |
| Stage acceptance | complete (exit 0, 567e8d89) | complete (exit 0, None) |
| Stage speed | complete (exit 0, 567e8d89) | complete (exit 0, a6b6cbae) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `finetune_s2_model_9998`

- Gates passed: 2_recovered_98, 2_belt, 2_physical_failures
- Gates missed: 2_pushes, 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, sensors_hold, sensors_tilt_negative, sensors_tilt_positive, wheel_lag_25ms_drive; lost backlash_turn_0, mismatch_body_mass_scale_0.85_combined, push_low_-2
- speed against settle: gained drive_step_half_timestep, push_fast_-2, replay_curve_r1; lost arc_r1, arc_r2, delay_15ms_noise_x2_drive_step, friction_0p5_replay_corner_90, mass_plus_15pct_drive_step, slope_down_10, slope_down_8, weave, wheel_lag_25ms_replay_corner_90
- hardware against settle: gained none; lost belt_hold_h0.1125_n0.225_k1, belt_hold_h0.1125_n0.9_k1, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k1, delay_15ms_noise_x2_drive, push_hold_+2
- Physical failures: none
- Not recovered: backlash_turn_0, belt_hold_h0.1125_n0.225_k0, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_combined, mismatch_body_mass_scale_0.85_hold, push_low_-2, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8
- At the band margin (rerun with perturbations before deciding on them): backlash_crouch_0.3, backlash_drive_0.3, backlash_turn_0, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.1125_n0.9_k1, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k1, mismatch_body_com_x_m_-0.005_combined, mismatch_body_com_x_m_-0.005_hold, mismatch_body_com_z_m_0.003_combined, mismatch_body_mass_scale_0.85_combined, mismatch_body_mass_scale_1.15_hold, mismatch_friction_0.5_combined, push_drive_+2.5, push_drive_-0.5, push_drive_-1, push_drive_-1.5, push_hold_+0.5, push_hold_+2, push_hold_-0.5, push_hold_-1.5, push_hold_-2, push_low_+0.5, push_low_+1, push_low_+2, push_low_+2.5, push_low_-0.5, push_low_-1, push_low_-1.5, push_low_-2, push_low_-2.5, slope_down_4, slope_down_8, slope_up_4
- Speed cases failing the policy rule: arc_r1 (overshoot); arc_r2 (overshoot); delay_15ms_noise_x2_drive_step (survives_and_recovers); drive_ramp (overshoot); drive_step (overshoot); friction_0p5_replay_corner_90 (survives_and_recovers); mass_plus_15pct_drive_step (survives_and_recovers); replay_corner_90 (path, survives_and_recovers); replay_start_stop (path); replay_straight_1.0 (path); slope_down_10 (axle_depth_p95); slope_down_4 (survives_and_holds); slope_down_8 (axle_depth_p95); slope_up_10 (survives_and_holds); slope_up_8 (survives_and_holds); spin (yaw_rate); weave (overshoot); wheel_lag_25ms_replay_corner_90 (survives_and_recovers)
- Stress set falls: push_back_d6, push_fwd_d2
