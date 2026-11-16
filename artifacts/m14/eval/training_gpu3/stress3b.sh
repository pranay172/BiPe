#!/bin/bash
cd ./.worktrees/m14
while systemctl --user is-active -q bipe-m14-stress3; do sleep 10; done
export MUJOCO_GL=egl
PY=../m13/.venv/rl/bin/python
for n in ft1_r2 ft1_r3; do
  log=local/m14/trainstress3_$n.log
  echo "START $(date -Is) $(git rev-parse --short HEAD)" > $log
  $PY -m bipe.m14.evaluate run artifacts/m14/checkpoints/finetune_s1/model_9998.pt --stages training --device cuda:0 --worlds 1000 --episodes 1000 --episode-s 10 --no-baseline --output local/m14/eval3/$n >> $log 2>&1
  echo "END $(date -Is) rc=$?" >> $log
done
touch local/m14/stress3.done
