#!/bin/bash
cd ./.worktrees/m14
export MUJOCO_GL=egl
PY=../m13/.venv/rl/bin/python
log=local/m14/finetune2_s1.log
echo "START $(date -Is) $(git rev-parse --short HEAD)" >> $log
$PY -m bipe train --task finetune2 --seed 1 --iterations 1000 --resume-run 2026-11-02_16-34-44_finetune_s1 --resume-checkpoint model_9998.pt >> $log 2>&1
echo "END $(date -Is) rc=$?" >> $log
R=$(ls -d local/m14/runs/bipe_m13/*_finetune2_s1 | sort | tail -1)
CK=$(ls $R/model_*.pt | sort -V | tail -1)
echo "LAST $CK" >> $log
for i in 1 2 3; do
  elog=local/m14/trainstress4_ft2_r$i.log
  echo "START $(date -Is) $(git rev-parse --short HEAD) $CK" > $elog
  $PY -m bipe.m14.evaluate run $CK --stages training --device cuda:0 --worlds 1000 --episodes 1000 --episode-s 10 --no-baseline --output local/m14/eval4/ft2_r$i >> $elog 2>&1
  echo "END $(date -Is) rc=$?" >> $elog
done
touch local/m14/finetune2_s1.done
