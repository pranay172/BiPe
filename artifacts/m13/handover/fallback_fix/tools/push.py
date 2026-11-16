import sys, json, copy
from pathlib import Path
ROOT = Path(__file__).resolve().parents[5]; sys.path.insert(0, str(ROOT))
from bipe import robustness as R
from bipe.parameters import load_parameters
from bipe.robustness_control import build_candidate
from bipe.moving_validate import jobs
from bipe.moving_experiment import simulate
p = load_parameters(profile="m10"); prof = R.native(p)
sensors = json.loads((ROOT / prof["sensors"]).read_text())
gains = json.loads((ROOT / prof["controller"]).read_text())
import os
if os.environ.get("GAINS"):
    gains.update(json.loads(os.environ["GAINS"]))
tag = "".join(c for c in os.environ.get("GAINS","current") if c.isalnum() or c in "._")
library = build_candidate(p, gains, ROOT / (".venv/m8-build/hand_" + tag))
job = copy.deepcopy(next(j for j in jobs(sensors, "c", p) if j["name"] == sys.argv[1])); job.pop("name")
job.update(nominal=p, gains=gains, firmware_library=str(library), disturbance={"push_Ns": float(sys.argv[2])}, duration=float(sys.argv[3]) if len(sys.argv)>3 else job["duration"])
sys.path.insert(0, str(Path(__file__).parent))
import variants as V
from bipe import firmware_control as fc
fc.FirmwareController = V.KneeFF
r = simulate(**job); m = R.physical_metrics(r)
print(sys.argv[1:], json.dumps({k: m.get(k) for k in ("physical_pass", "recovered", "peak_pitch_deg", "failure")}))
