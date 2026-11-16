# M8 — robustness yardstick and control baseline

Started 4 October 2026. Keep M4 geometry, rigid-body masses and tensors unchanged;
M9 compact redesign is later. These are simulation measurements, not hardware
qualification. Preserve the user's modified M3 CAD. No new evidence seal system.

1. Integrate configurable chassis pulses, an inclined ground plane and wheel
   lost motion into the normal free-base simulator. Backlash uses freely driven
   rotor inertia and dissipative, inelastic engagement; test rigid/near-zero
   parity, momentum/energy behavior and gap reversals. Keep nominal controller
   mass/COM parameters when varying the plant.
2. Correct the wheel torque placeholder using AK40-10 manufacturer ratings and
   the conditional M4 electrical/thermal domain. Require actuator peak torque
   ≥ continuous torque at parameter load. Regenerate MCU constants and test
   C/Python equivalence. Preserve the original 0.6 Nm baseline as an explicit
   comparison configuration, never silently as the current physical limit.
3. Design a posture-scheduled balance/speed LQR from MuJoCo finite-difference
   transitions at several crouch heights, including motor lag. Retune the
   existing cascade. Use the same C estimator, local leg loop and finite motor
   model for each candidate; implement candidate balance laws in portable C.
4. Run the same permanent matrix for baseline, retuned cascade and LQR:
   ±0.5/1/1.5/2/2.5 N·s chassis pushes (0.1 s at t=4 s) in hold, sustained
   drive and lowest posture; 0/4/8/10° real planes in hold and up/down drive;
   0/0.3/0.6° backlash in hold/drive/turn/crouch; body mass ±15%, body COM
   X ±5 mm / Z ±3 mm, friction 0.5/1.0, individually in hold and combined
   locomotion. Use sensors-m4, fixed seeds and full requested durations.
5. Fail physically at >20° pitch error, >10° roll, forbidden contact, joint
   limits or non-finite state (and retain C safety inhibition). Recovery means
   final two seconds within 2° pitch error and 0.03 m/s hold speed; slope
   stability is reported separately from position drift. Report both push
   directions separately, largest passing slope, rollback and late pitch
   peak-to-peak. Do not change thresholds during tuning. Proposed targets:
   recover ≥2 N·s each way at nominal hold; 10° hold rollback ≤0.3 m;
   0.6° backlash hold oscillation ≤1.5° peak-to-peak.
6. Compare all 27 inherited cases and unchanged half-timestep convergence.
   Adopt a candidate only with complete inherited acceptance and improved
   robustness; otherwise retain the cascade and explain the limitation. Make
   base-environment pytest skip optional RL dependencies cleanly.

Save one summary JSON (configurations, per-case metrics and decimated measured
traces), one metrics CSV and one plot; no videos or per-case artifact trees.
Keep tuning diagnostics compact and distinguish them from final runs. Commit
the plan first, implementation/tests next, and measured results/adoption last.
Results document: about one page, numbers table, missed targets and exact
reproduction commands. Geometry/mass changes and hardware deployment are out
of scope.
