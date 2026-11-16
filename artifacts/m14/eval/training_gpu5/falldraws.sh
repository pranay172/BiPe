#!/bin/bash
cd ./.worktrees/m14
export MUJOCO_GL=egl
PY=../m13/.venv/rl/bin/python
FT2=artifacts/m14/checkpoints/finetune2_s1/model_10997.pt
FT1=artifacts/m14/checkpoints/finetune_s1/model_9998.pt
L=local/m14/fall_draws.log
echo "START $(date -Is) $(git rev-parse --short HEAD)" > $L
$PY -m bipe.m14.fall_draws $FT2 $FT1 artifacts/m13/training/settle/model_10493.pt --seed 1 --device cuda:0 --output local/m14/fall_draws >> $L 2>&1; echo "rc=$? seed1" >> $L
for s in 2 3; do $PY -m bipe.m14.fall_draws $FT2 --seed $s --device cuda:0 --output local/m14/fall_draws >> $L 2>&1; echo "rc=$? seed$s" >> $L; done
echo "END $(date -Is)" >> $L
for n in ft2 ft1; do
  [ $n = ft2 ] && CK=$FT2 || CK=$FT1
  for s in 11 12 13 14 15; do
    o=${n}_seed$s; elog=local/m14/trainstress5_$o.log
    echo "START $(date -Is) $(git rev-parse --short HEAD) $CK" > $elog
    $PY -m bipe.m14.evaluate run $CK --stages training --device cuda:0 --worlds 1000 --episodes 1000 --episode-s 10 --seed $s --no-baseline --output local/m14/eval5/$o >> $elog 2>&1
    echo "END $(date -Is) rc=$?" >> $elog
  done
done
touch local/m14/falldraws.done
