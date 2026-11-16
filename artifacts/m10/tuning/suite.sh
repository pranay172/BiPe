#!/bin/bash
# suite.sh '<override json>' tag [sensing]
# S holds the tuning session's exp.py and its outputs (a scratch directory, not committed).
S=${S:-local/m10-tuning}
cd "$(dirname "$0")/../../.."
sens=${3:-sensors}
for sc in hold:60 tilt_positive:60 tilt_negative:60 drive:20 turn:15 crouch:36 combined:36 push:16; do
  n=${sc%%:*}; d=${sc##*:}
  ( PYTHONPATH=. .venv/bin/python $S/exp.py $n $d $sens "$1" > $S/res_$2_$n.json 2>$S/err_$2_$n.txt; echo "$2 $n $(cat $S/res_$2_$n.json | head -c 400)" ) &
done
wait
