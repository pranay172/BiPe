# M14 policy evaluation

- `finetune3_s1_model_11997`: `local/m15/checkpoints/finetune3_s1/model_11997.pt` (sha256 dbda4e4c28a6)

| | finetune3_s1_model_11997 | settle (baseline) |
|---|---|---|
| Gate 2: recovered, 98 policy cases (>= 83) | 75/98 | 74/98 |
| Gate 2: belt range recovered (>= 6/8) | 8/8 | 2/8 |
| Gate 2: pushes recovered (30/30) | 20/30 | 30/30 |
| Gate 2: physical failures, acceptance + speed (0) | 11 | 0 |
| Gate 3: speed policy rule (>= 40 of 43 judged) | 31/43 | 31/43 |
| Gate 3: axle worst steady p95, mm | 3.30 | 9.80 |
| Gate 3: judged cases without a steady segment (no axle check) | 0 | 0 |
| Speed rule with axle check case 10 mm | 31/43 | 31/43 |
| Speed rule with axle check case 5 mm | 31/43 | 4/43 |
| Gate 5: training stress falls (<= 5 %) | not run | 20.3% (13/64 x 10.0 s; 95 % 12.3%-31.7%) |
| Gate 5: full-plant stress survived (26/26) | 25/26 | 12/24 |
| Survived, 98 policy cases | 88/98 | 98/98 |
| Recovered, 79 matrix cases (M13 headline) | 60/79 | 60/79 |
| Speed: survived and recovered | 35/44 | 37/44 |
| Speed: classical joint-error rule | 0/43 | 0/43 |
| Hardware timing: belt recovered | 8/8 | 8/8 |
| Hardware timing: PR #5 handover cases recovered | 7/10 | 8/10 |
| Hardware timing: physical failures | 1 | 0 |
| Acceptance verdicts at the band margin | 4 | 16 |
| Stage hardware | complete (exit 0, 567e8d89) | not run |
| Stage stress | complete (exit 0, 567e8d89) | not run |
| Stage acceptance | complete (exit 0, 567e8d89) | complete (exit 0, None) |
| Stage speed | complete (exit 0, 878b6cc3) | complete (exit 0, a6b6cbae) |

Gate 2 is judged on the acceptance suite at simulation timing, as the plan defines it; the hardware-timing rows (the M11 image's one-tick-late action) are reported beside it, not gated. Gate 3: the speed suite's checks without the joint-error criterion, plus axle height p95 within 5 mm over steady segments (command held 1 s, rest included), signed off 1 November 2026. Gate 4 (two seeds, at most 6 h each) is read from the training runs. Verdicts at the band margin are close to chaotic in single runs; rerun them perturbed before calling gate 2 on a seed that lands within a few cases of 83.

## `finetune3_s1_model_11997`

- Gates passed: 2_belt
- Gates missed: 2_recovered_98, 2_pushes, 2_physical_failures, 3_speed_policy_rule, 5_stress_physical_failures
- Gates incomplete (stage not finished or failed): none
- acceptance against settle: gained backlash_hold_0.6_half_timestep, belt_hold_h0.1125_n0.225_k0, belt_hold_h0.1125_n0.9_k0, belt_hold_h0.45_n0.225_k0, belt_hold_h0.45_n0.225_k1, belt_hold_h0.45_n0.9_k0, belt_hold_h0.45_n0.9_k1, mass_plus_15pct_drive, mismatch_body_com_x_m_0.005_combined, mismatch_body_com_x_m_0.005_hold, mismatch_body_mass_scale_0.85_hold; lost push_low_+0.5, push_low_+1, push_low_+1.5, push_low_+2, push_low_+2.5, push_low_-0.5, push_low_-1, push_low_-1.5, push_low_-2, push_low_-2.5
- speed against settle: gained drive_step_half_timestep, push_fast_-2, replay_start_stop, slope_down_4, slope_up_10, slope_up_8, spin; lost arc_r2, reverse, slope_down_10, slope_down_8, weave, wheel_lag_25ms_drive_step, wheel_lag_25ms_weave
- hardware against settle: gained none; lost push_low_-2.5
- Physical failures: push_low_+0.5, push_low_+1, push_low_+1.5, push_low_+2, push_low_+2.5, push_low_-0.5, push_low_-1, push_low_-1.5, push_low_-2, push_low_-2.5
- Not recovered: push_low_+0.5, push_low_+1, push_low_+1.5, push_low_+2, push_low_+2.5, push_low_-0.5, push_low_-1, push_low_-1.5, push_low_-2, push_low_-2.5, sensors_hold, sensors_tilt_negative, sensors_tilt_positive, slope_down_10, slope_down_4, slope_down_8, slope_hold_10, slope_hold_4, slope_hold_8, slope_up_10, slope_up_4, slope_up_8, wheel_lag_25ms_drive
- At the band margin (rerun with perturbations before deciding on them): mismatch_body_com_z_m_-0.003_combined, mismatch_body_mass_scale_0.85_combined, slope_hold_10, slope_up_10
- Speed cases failing the policy rule: arc_r2 (overshoot); drive_ramp (overshoot); drive_step (overshoot); replay_corner_90 (path); replay_curve_r1 (path); replay_straight_1.0 (path); reverse (overshoot); slope_down_10 (survives_and_holds); slope_down_8 (survives_and_holds); weave (overshoot, yaw_rate); wheel_lag_25ms_drive_step (survives_and_recovers); wheel_lag_25ms_weave (survives_and_recovers)
- Speed physical failures: low_posture_drive_ramp
- Stress set falls: push_fwd_d6
- Hardware timing falls: push_low_-2.5
