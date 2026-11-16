#!/bin/bash
# usage: sweep.sh '<gains json>'  -> push 1.0 classical + handover at 5 s
cd /home/claude/BiPe; export MUJOCO_GL=disable
g="$1"
a=$(GAINS="$g" .venv/bin/python artifacts/m13/handover/fallback_fix/tools/push.py delay_15ms_noise_x2_drive 1.0 2>&1 | tail -1 | grep -o '"recovered": [a-z]*, "peak_pitch_deg": [0-9.]*\|"failure": "[^"]*"' | tr '\n' ' ')
b=$(GAINS="$g" .venv/bin/python artifacts/m13/handover/fallback_fix/tools/diag.py delay_15ms_noise_x2_drive 2>&1 | grep physical | grep -o '"recovered": [a-z]*, "peak_pitch_deg": [0-9.]*\|"failure": "[^"]*"' | tr '\n' ' ')
echo "$g | push1.0: $a | handover5: $b"
