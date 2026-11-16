#!/bin/bash
cd /home/claude/BiPe; export MUJOCO_GL=disable
for v in fault fault_inline fault_deferred_tick; do
  .venv/bin/python -m bipe.m13.handover acceptance --variant $v --case push_hold_+2 --case push_hold_-2 --case push_drive_+2.5 --case push_low_-2.5 --case sensors_combined --case backlash_turn_0.6 --case belt_hold_h0.45_n0.9_k1 --case slope_up_8 --case delay_15ms_noise_x2_drive --case wheel_lag_25ms_drive --output artifacts/m13/handover/fallback_fix/${v}_acceptance --workers 4 > local/handover_$v.log 2>&1
  echo "handover $v exit $?"
done
.venv/bin/python -m bipe.m12.speed --workers 4 > local/speed.log 2>&1
echo "speed exit $?"
