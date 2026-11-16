import sys
import bipe.m14.stress as S
size = float(sys.argv.pop(1))
S.SCENARIOS["push_fwd"]["push_Ns"] = size
S.SCENARIOS["push_back"]["push_Ns"] = -size
S.main()
