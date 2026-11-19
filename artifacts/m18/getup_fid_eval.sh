#!/bin/sh
# M18: the get-up checks of getup_fid_s1 (and the getup_s1 baseline under fidelity).
#   fresh_fidelity: the M17 fresh set (seeds 3000+), each trial on a fidelity robot
#   fresh:          the M17 fresh set, nominal robot
#   heldout:        the M16 held-out set (seeds 1000+), nominal robot
#   fidelity_5000:  a set nothing in M15 to M18 saw (seeds 5000+), fidelity robots
# The M17 settings throughout: C fallen mode, quiet handover, 0.5 s flat pre-pose.
cd "$(dirname "$0")/../.." || exit 1
export MUJOCO_GL=disable PYTHONPATH=.
PY=.venv/bin/python
NEW=artifacts/m18/checkpoints/getup_fid_s1/model_14996.pt
OLD=artifacts/m15/checkpoints/getup_s1/model_12997.pt
M17="--fallen c --quiet-handover --flat-prepose front,back --prepose-s 0.5 --workers 4"
LEANS="--classes front_lean,back_lean"
$PY -m bipe.m15.getup_eval --getup $NEW $M17 --first-seed 3000 $LEANS --fidelity --output artifacts/m18/getup_fid_fresh_fidelity
$PY -m bipe.m15.getup_eval --getup $NEW $M17 --first-seed 3000 $LEANS --output artifacts/m18/getup_fid_fresh
$PY -m bipe.m15.getup_eval --getup $NEW $M17 --first-seed 1000 --classes sit,front_lean,back_lean --output artifacts/m18/getup_fid_heldout
$PY -m bipe.m15.getup_eval --getup $OLD $M17 --first-seed 3000 $LEANS --fidelity --output artifacts/m18/getup_fidelity_fresh
for name in getup_fid_s1:$NEW getup_s1:$OLD; do
  $PY -m bipe.m15.getup_eval --getup ${name#*:} $M17 --first-seed 5000 $LEANS --fidelity --output artifacts/m18/fidelity_5000_${name%%:*}
done
