#!/bin/bash
cd ./.worktrees/m14
export MUJOCO_GL=egl
PY=../m13/.venv/rl/bin/python
declare -A CK=([s1]=artifacts/m14/checkpoints/locomotion_s1/model_7999.pt [s2]=artifacts/m14/checkpoints/locomotion_s2/model_7999.pt [settle]=artifacts/m13/training/settle/model_10493.pt)
for n in s1 s2 settle; do
  log=local/m14/trainstress_$n.log
  echo "START $(date -Is)" > $log
  t0=$(date +%s)
  $PY -m bipe.m14.evaluate run ${CK[$n]} --stages training --device cuda:0 --worlds 1000 --episodes 1000 --episode-s 10 --no-baseline --output local/m14/eval/$n >> $log 2>&1
  rc=$?
  echo "END $(date -Is) rc=$rc elapsed_s=$(( $(date +%s)-t0 ))" >> $log
done
echo ALLDONE > local/m14/trainstress.done
