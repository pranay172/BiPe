"""M16 decision 2: score a speed-suite run made with the command shaper against the shaped
command (what the robot was asked to do) instead of the raw one.

    python -m bipe.m16.shaped_rule DIR [DIR ...]

DIR is a ``bipe.m13.full_plant speed --shaper ...`` output (policy.json names the shaper).
Each case is rescored from its saved 50 Hz trace with the M14 policy rule
(bipe.m14.evaluate.speed_rule) after these substitutions:

- plateau speed error against the shaped speed (the trace's ``velocity_reference``);
- yaw-rate error against the shaped yaw rate (the shaper rerun on ``user_yaw_rate`` at the
  20 ms policy step, as in the loop);
- overshoot above the peak shaped command;
- in a replay, the ideal follower's path moved by the difference between the paths the
  raw and the shaped commands trace (unicycle integration from the replay start), so the
  rule is unchanged when the shaper is off.

The stop check already uses the shaped reference. Survival, recovery, peak pitch, heading
and the axle depth check are unchanged. Writes DIR/shaped_rule.json with each case's
verdict under both rules, and the shaper's added lag: the extra time it takes to reach
90 % of a 1 m/s step beyond the training rate limit's 0.72 s (decision 2: at most 0.5 s).
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

from .shaper import Shaper

ROOT = Path(__file__).resolve().parents[2]
POLICY_DT = 0.02
TRAINING_ACCELERATION = 1.25
LAG_CEILING_S = 0.5


def added_lag(cfg, step=1.0):
    s, t, v = Shaper(**cfg), 0.0, 0.0
    while v < 0.9 * step:
        v = min(s.step(step, 0.0, POLICY_DT)[0], v + TRAINING_ACCELERATION * POLICY_DT)
        t += POLICY_DT
    return t - 0.9 * step / TRAINING_ACCELERATION


def _unicycle(t, v, w):
    dt = np.diff(t, prepend=t[0])
    heading = np.cumsum(w * dt)
    return np.c_[np.cumsum(v * np.cos(heading) * dt), np.cumsum(v * np.sin(heading) * dt)]


def rescore(row, trace, cfg, criteria):
    from ..m12.speed import follower_trace
    from ..m14.evaluate import speed_rule

    f = {str(c): trace["trace"][:, i] for i, c in enumerate(trace["columns"])}
    t = f["time"]
    shaper, shaped_w = Shaper(**cfg), np.zeros_like(t)
    for i in range(len(t)):
        shaped_w[i] = shaper.step(f["user_velocity"][i], f["user_yaw_rate"][i], POLICY_DT)[1]
    v_ref, case = f["velocity_reference"], row["case"]
    r = copy.deepcopy(row)
    s = r["speed"]
    if case.get("plateaus"):
        s["overshoot_m_s"] = s["peak_speed_m_s"] - float(np.abs(v_ref).max())
    s["plateau_speed_error_m_s"] = [
        float(np.mean(np.abs(f["velocity"] - v_ref)[(t >= a) & (t < b)]))
        for a, b in case.get("plateaus", [])
    ]
    s["yaw_rate_error_rad_s"] = [
        float(np.mean(np.abs(f["yaw_rate"] - shaped_w)[(t >= a) & (t < b)]))
        for a, b in case.get("yaw_windows", [])
    ]
    if "replay" in case:
        rp = case["replay"]
        g = follower_trace(rp["layout"], rp["scenario"])
        shifted = g["t"] + rp["start_s"]
        inside = (t >= shifted[0]) & (t <= shifted[-1])
        ti = t[inside]
        ideal = np.c_[np.interp(ti, shifted, g["x"] - g["x"][0]),
                      np.interp(ti, shifted, g["y"] - g["y"][0])]  # fmt: skip
        raw = _unicycle(ti, f["user_velocity"][inside], f["user_yaw_rate"][inside])
        shaped = _unicycle(ti, v_ref[inside], shaped_w[inside])
        xy = np.c_[f["x"], f["y"]][inside]
        s["path_error_m"] = float(
            np.linalg.norm(xy - xy[0] - (ideal + shaped - raw), axis=1).max()
        )
    return speed_rule(row, criteria), speed_rule(r, criteria), s


def score(directory):
    directory = Path(directory)
    policy = json.loads((directory / "policy.json").read_text())
    cfg = policy.get("shaper") or {}
    criteria = json.loads((ROOT / "config/speed-m12.json").read_text())["criteria"]
    rows = {}
    if (directory / "cases.json").exists():
        rows = json.loads((directory / "cases.json").read_text())
    for p in sorted((directory / "results").glob("*.json")):
        rows.setdefault(p.stem, json.loads(p.read_text()))
    out, counts = {}, {"raw": [0, 0], "shaped": [0, 0]}
    for name, row in sorted(rows.items()):
        with np.load(directory / "traces" / f"{name}.npz") as z:
            trace = {"columns": z["columns"], "trace": z["trace"]}
        raw, shaped, _ = rescore(row, trace, cfg, criteria) if cfg else (None, None, None)
        if raw is None or raw["passed"] is None:
            continue
        for k, v in (("raw", raw), ("shaped", shaped)):
            counts[k][0] += bool(v["passed"])
            counts[k][1] += 1
        out[name] = {
            "raw": raw["passed"], "shaped": shaped["passed"],
            "failing_shaped": [k for k, x in shaped["checks"].items() if not x],
        }  # fmt: skip
    lag = added_lag(cfg) if cfg else 0.0
    result = {"shaper": cfg, "added_lag_s": lag, "lag_ok": lag <= LAG_CEILING_S,
              "passed_raw": counts["raw"], "passed_shaped": counts["shaped"], "cases": out}  # fmt: skip
    (directory / "shaped_rule.json").write_text(json.dumps(result, indent=1) + "\n")
    fails = {n: c["failing_shaped"] for n, c in out.items() if not c["shaped"]}
    print(directory.name, "raw", counts["raw"], "shaped", counts["shaped"],
          f"added lag {lag:.2f} s", fails)  # fmt: skip
    return result


if __name__ == "__main__":
    for d in sys.argv[1:]:
        score(d)
