import sys

import numpy as np

from bipe.m12 import following as FL
from bipe.profiles import native
from bipe.robustness_control import build_candidate

name, variant, out = sys.argv[1], (sys.argv[2] if sys.argv[2] != "none" else None), sys.argv[3]
p, gains, _, _ = FL.SP.load()
lib = str(build_candidate(p, gains, FL.ROOT / native(p)["firmware_build"]))
run = FL.closed_task((name, 0, 0.15, variant, 0.0, lib, {}))
np.savez_compressed(out, columns=np.array(run["columns"]), trace=run["trace"])
print("saved", run["trace"].shape)
