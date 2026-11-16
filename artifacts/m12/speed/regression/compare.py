"""Compare the M10 acceptance record at a git revision with the current one.

PYTHONPATH=. .venv/bin/python artifacts/m12/speed/regression/compare.py <revision> <output.json>
"""

import json
import subprocess
import sys
from pathlib import Path

from bipe.parameters import ROOT

PATH = "artifacts/m10/acceptance/summary.json"
KEYS = [
    "passed", "physical_pass", "recovered", "max_pitch_error_deg", "peak_pitch_deg",
    "plateau_speed_mae_m_s", "max_stopping_travel_m", "peak_speed_m_s", "rollback_m",
    "late_pitch_pp_deg", "final_heading_error_deg", "failure",
]


def rows(summary):
    cases = summary["controllers"]["current"]["cases"]
    out = {}
    for name, c in cases.items():
        m = c["metrics"]
        flat = {**m.get("physical_metrics", {}), **m}
        out[name] = {k: flat.get(k) for k in KEYS if k in flat}
    return out


old = json.loads(subprocess.run(["git", "show", f"{sys.argv[1]}:{PATH}"], cwd=ROOT,
                                capture_output=True, check=True, text=True).stdout)
new = json.loads((ROOT / PATH).read_text())
a, b = rows(old), rows(new)
changed = {}
for name in sorted(set(a) | set(b)):
    x, y = a.get(name, {}), b.get(name, {})
    flags = [k for k in ("passed", "physical_pass", "recovered") if k in x and x.get(k) != y.get(k)]
    if flags:
        changed[name] = {"changed": flags, "before": x, "after": y}
record = {
    "before_revision": sys.argv[1],
    "margins_before": old["controllers"]["current"]["margins"],
    "margins_after": new["controllers"]["current"]["margins"],
    "pass_flag_changes": changed,
    "cases": {n: {"before": a.get(n), "after": b.get(n)} for n in sorted(set(a) | set(b))},
}
Path(sys.argv[2]).write_text(json.dumps(record, indent=2, default=str))
print(json.dumps({"targets_before": record["margins_before"]["targets"],
                  "targets_after": record["margins_after"]["targets"],
                  "pass_flag_changes": {n: v["changed"] for n, v in changed.items()}}, indent=1))
