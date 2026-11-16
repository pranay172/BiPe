#!/bin/bash
cd ./.worktrees/m14
export MUJOCO_GL=egl
log=local/m14/feasibility.log
echo "START $(date -Is) $(git rev-parse --short HEAD)" >> $log
../m13/.venv/rl/bin/python -m bipe.m14.feasibility artifacts/m13/training/settle/model_10493.pt artifacts/m14/checkpoints/locomotion_s1/model_7999.pt artifacts/m14/checkpoints/locomotion_s2/model_7999.pt --device cuda:0 --worlds 1000 --episodes 1000 --output local/m14/feasibility >> $log 2>&1
echo "END $(date -Is) rc=$?" >> $log
touch local/m14/feasibility.done
