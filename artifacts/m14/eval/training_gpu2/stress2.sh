#!/bin/bash
cd ./.worktrees/m14
export MUJOCO_GL=egl
PY=../m13/.venv/rl/bin/python
declare -A CK=([ft1]=artifacts/m14/checkpoints/finetune_s1/model_9998.pt [s1]=artifacts/m14/checkpoints/locomotion_s1/model_7999.pt [s2]=artifacts/m14/checkpoints/locomotion_s2/model_7999.pt [settle]=artifacts/m13/training/settle/model_10493.pt)
for n in ft1 s1 s2 settle; do
  log=local/m14/trainstress2_$n.log
  echo "START $(date -Is) $(git rev-parse --short HEAD)" > $log
  $PY -m bipe.m14.evaluate run ${CK[$n]} --stages training --device cuda:0 --worlds 1000 --episodes 1000 --episode-s 10 --no-baseline --output local/m14/eval2/$n >> $log 2>&1
  echo "END $(date -Is) rc=$?" >> $log
done
log=local/m14/feasibility2.log
echo "START $(date -Is) $(git rev-parse --short HEAD)" > $log
$PY -m bipe.m14.feasibility artifacts/m14/checkpoints/finetune_s1/model_9998.pt --device cuda:0 --output local/m14/feasibility2 >> $log 2>&1
echo "END $(date -Is) rc=$?" >> $log
touch local/m14/stress2.done
