"""M16 phase H: the behaviour catalogue and the shared policy contract.

    python -m bipe.m16.catalogue check     every catalogue export against contract v1
    python -m bipe.m16.catalogue list      behaviours, exports and gate results

The catalogue is config/behaviours.json. Every policy reads the same 25 observations and
writes the same 6 actions at the same rates (docs/policy-contract.md), so the supervisor
can switch between them without touching the firmware's sensing or actuation, as it does
between get-up and drive. ``check`` compares each export's manifest.json with the
contract (observation layout and units, action layout and laws' scales, timing, command
rate limits and ranges), checks the ONNX graph's hash and shapes against the manifest, and
fails on any difference. It also prints the contract hash each manifest carries in
effect, so a later contract version is a visible change.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CATALOGUE = ROOT / "config/behaviours.json"

# Contract v1, as the M14 and M15 exports state it.
OBSERVATION = [
    ("gyro", [0, 3], "rad/s"),
    ("accelerometer", [3, 6], "m/s^2"),
    ("leg_encoders", [6, 10], "rad"),
    ("leg_speeds", [10, 14], "rad/s"),
    ("wheel_speeds", [14, 16], "rad/s"),
    ("last_action", [16, 22], "1"),
    ("command", [22, 25], "m/s, rad/s, 1"),
]
ACTION = {"legs": [0, 4], "wheels": [4, 6], "leg_scale_rad": 0.5, "wheel_scale_rad_s": 40.0}
TIMING = {"policy_period_s": 0.02, "control_tick_s": 0.005, "policy_every_ticks": 4}
COMMAND = {
    "rate_limit_per_s": {"speed_m_s": 1.25, "yaw_rate_rad_s": None, "posture_label": 20.0},
    "trained_range": {"speed_m_s": [-0.5, 1.2], "yaw_rate_rad_s": [-1.0, 1.0],
                      "posture_label": [30.0, 65.0]},
}  # fmt: skip
# Optional requests in the command's posture slot (docs/policy-contract.md, "Requests"):
# outside the hash, each accepted only by the policies whose manifest lists it.
REQUESTS = {"sit": {"posture_label": 100.0, "normalised": 3.0}}
CONTRACT = {"version": 1, "observation": OBSERVATION, "action": ACTION, "timing": TIMING,
            "command": COMMAND, "input": [1, 25], "output": [1, 6]}  # fmt: skip


def contract_of(manifest):
    """The contract fields a manifest states, in CONTRACT's shape."""
    a = manifest["action"]
    return {
        "version": 1,
        "observation": [(o["name"], o["index"], o["unit"]) for o in manifest["observation"]],
        "action": {"legs": a["legs"]["index"], "wheels": a["wheels"]["index"],
                   "leg_scale_rad": a["legs"]["leg_scale_rad"],
                   "wheel_scale_rad_s": a["wheels"]["wheel_scale_rad_s"]},
        "timing": {k: manifest["timing"][k] for k in TIMING},
        "command": manifest["command"],
        "input": manifest["graph"]["input"]["shape"],
        "output": manifest["graph"]["output"]["shape"],
    }  # fmt: skip


def contract_hash(c):
    return hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()[:12]


def check():
    cat = json.loads(CATALOGUE.read_text())
    want = json.loads(json.dumps(CONTRACT))  # tuples to lists, as JSON reads them
    bad = []
    for name, b in cat["behaviours"].items():
        if b.get("kind") != "policy":
            print(f"{name}: {b['kind']} (no network)")
            continue
        d = ROOT / b["export"]
        m = json.loads((d / "manifest.json").read_text())
        got = json.loads(json.dumps(contract_of(m)))
        diff = [k for k in want if got.get(k) != want[k]]
        graph = d / m["graph"]["file"]
        sha_ok = hashlib.sha256(graph.read_bytes()).hexdigest() == m["graph"]["sha256"]
        for req, spec in m.get("requests", {}).items():
            known = REQUESTS.get(req)
            if known is None or any(spec.get(k) != v for k, v in known.items()):
                diff.append(f"request {req}")
        ok = not diff and sha_ok
        req = ",".join(m.get("requests", {})) or "none"
        print(f"{name}: {b['export']} contract {contract_hash(got)} requests {req} "
              f"{'ok' if ok else 'MISMATCH ' + ', '.join(diff) + ('' if sha_ok else ' graph sha')}")
        bad += [] if ok else [name]
    print(f"contract v1 {contract_hash(want)}")
    return not bad


def listing():
    cat = json.loads(CATALOGUE.read_text())
    for name, b in cat["behaviours"].items():
        gates = ", ".join(f"{k} {v}" for k, v in b.get("gates", {}).items())
        print(f"{name:8s} {b['what']}\n         {b.get('export', b['kind'])}\n         {gates}")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    sys.exit(0 if (check() if cmd == "check" else listing() or True) else 1)
