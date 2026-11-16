import sys, numpy as np
from bipe.moving_experiment import COLUMNS as C
t=np.load(sys.argv[1])["trace"]; i=C.index
a,b,s=float(sys.argv[2]),float(sys.argv[3]),int(sys.argv[4]) if len(sys.argv)>4 else 2
for r in t[(t[:,0]>=a)&(t[:,0]<=b)][::s]:
    print(f"{r[0]:.2f} v={r[i('velocity')]:.3f} vu={r[i('user_velocity')]:.2f} p={r[i('pitch_deg')]:5.2f} pe={r[i('pitch_error_deg')]:5.2f} "
      "q="+" ".join(f"{r[i(c)]:6.1f}" for c in ['left_hip_deg','left_knee_deg'])+" qr="+" ".join(f"{r[i(c)]:6.1f}" for c in ['left_hip_reference_deg','left_knee_reference_deg'])
      +" W="+" ".join(f"{r[i(c)]:.3f}" for c in ['left_applied_Nm','right_applied_Nm']))
