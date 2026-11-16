"""M12 phase D: the summary, the requirement checks and the plots from the saved runs."""

import hashlib
import json
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from . import follow as FW
from . import following as FL

EVIDENCE = (
    "Validated simulation under stated assumptions, not hardware: the free-base M10 plant "
    "with the C balance core, the sensor model, finite motor limits and physical contact; "
    "the C follower; the Pi stood in for by the phase C measurement model (rendered-frame "
    "statistics, detection probability, bias, noise, yaw blur and latency), not rendered "
    "perception in the loop (phase F). Planar target geometry; slopes tilt the plant only."
)


def _rms(values):
    v = [x for x in values if x is not None]
    return float(np.sqrt(np.mean(np.square(v)))) if v else None


def _max(values):
    v = [x for x in values if x is not None]
    return float(max(v)) if v else None


def open_summary(rows):
    out = {}
    for centre in sorted({r["latency_centre_s"] for r in rows}):
        rs = [r for r in rows if r["latency_centre_s"] == centre]
        entry = {}
        for key in ("compensated", "uncompensated"):
            entry[key] = {
                "position_rms_m": _rms([r[key]["position_rms_m"] for r in rs if r[key]]),
                "position_p95_max_m": _max([r[key]["position_p95_m"] for r in rs if r[key]]),
                "range_rms_m": _rms([r[key]["range_rms_m"] for r in rs if r[key]]),
                "range_mean_m": float(np.mean([r[key]["range_mean_m"] for r in rs if r[key]])),
                "bearing_rms_deg": _rms([r[key]["bearing_rms_deg"] for r in rs if r[key]]),
                "bearing_max_abs_deg": _max([r[key]["bearing_max_abs_deg"] for r in rs if r[key]]),
            }
        by = defaultdict(list)
        for r in rs:
            by[r["name"]].append(r)
        entry["by_scenario"] = {
            name: {
                "position_rms_m": _rms(
                    [r["compensated"]["position_rms_m"] for r in g if r["compensated"]]
                ),
                "uncompensated_position_rms_m": _rms(
                    [r["uncompensated"]["position_rms_m"] for r in g if r["uncompensated"]]
                ),
                "gated": int(sum(r["reports"]["gated"] for r in g)),
                "accepted": int(sum(r["reports"]["accepted"] for r in g)),
                "lost": int(sum(1 for r in g for _, e in r["events"] if e == "lost")),
            }
            for name, g in sorted(by.items())
        }
        entry["runs"] = len(rs)
        out[f"{centre:.2f}"] = entry
    return out


def closed_summary(runs):
    groups = defaultdict(list)
    for key, m in runs.items():
        groups[m["group"]].append((key, m))
    out = {}
    for group, items in sorted(groups.items()):
        by = defaultdict(list)
        for _, m in items:
            by[m["name"]].append(m)
        scen = {}
        for name, ms in sorted(by.items()):
            scen[name] = {
                "runs": len(ms),
                "passed": sum(m["passed"] for m in ms),
                "expectation_met": sum(m["checks"]["expectation_met"] for m in ms),
                "fell": sum(m["fell"] for m in ms),
                "peak_pitch_max_deg": max(m["peak_pitch_deg"] for m in ms),
                "envelope_inside": all(m["envelope"]["inside"] for m in ms),
                "losses_to_brake": sum(len(m["brakes"]) for m in ms),
                "brakes_reacquired": sum(
                    1 for m in ms for b in m["brakes"] if b["then"] == "following"
                ),
                "holds": sum(m["losses_to_hold"] for m in ms),
                "coasts": sum(len(m["coasts"]) for m in ms),
                "range_min_m": _min_of(m["summary"].get("range_min_m") for m in ms),
                "range_max_m": _max([m["summary"].get("range_max_m") for m in ms]),
                "peak_bearing_deg": _max([m["summary"].get("peak_bearing_deg") for m in ms]),
                "stopping_distance_max_m": _max(
                    [d for m in ms for d in m["checks"]["stopping_distances_m"]]
                ),
                "estimate_range_rms_m": _rms([m["estimate"].get("range_rms_m") for m in ms]),
                "estimate_bearing_rms_deg": _rms(
                    [m["estimate"].get("bearing_rms_deg") for m in ms]
                ),
                "failed": [_why(m) for m in ms if not m["passed"]],
            }
        out[group] = {
            "runs": len(items),
            "passed": sum(m["passed"] for _, m in items),
            "fell": sum(m["fell"] for _, m in items),
            "peak_pitch_max_deg": max(m["peak_pitch_deg"] for _, m in items),
            "envelope_inside": all(m["envelope"]["inside"] for _, m in items),
            "by_scenario": scen,
        }
    return out


def _min_of(values):
    v = [x for x in values if x is not None]
    return float(min(v)) if v else None


def _why(m):
    reasons = []
    if m["fell"]:
        reasons.append(f"fell: {m['failure']}")
    if not m["pitch_within_gate"]:
        reasons.append(f"peak pitch {m['peak_pitch_deg']:.1f} deg")
    if not m["envelope"]["inside"]:
        reasons.append("envelope")
    c = m["checks"]
    if not c["expectation_met"]:
        kinds = [e["event"] for e in m["events"] if e["event"] != "coasting"]
        detail = ", ".join(kinds)
        if c.get("steady_bearing_ok") is False:
            detail += f"; steady bearing {m['summary'].get('peak_bearing_deg'):.1f} deg"
        if m["summary"].get("range_in_band") is False:
            detail += f"; range {m['summary'].get('range_min_m'):.2f}-{m['summary'].get('range_max_m'):.2f} m"
        reasons.append(f"expectation ({detail})")
    return {"seed": m["seed"], "variant": m["variant"], "reasons": reasons}


def plot(runs, output):
    keys = [k for k, m in runs.items() if m["group"] == "nominal" and m["seed"] == 0]
    if not keys:
        return
    fig, axes = plt.subplots(3, 4, figsize=(16, 11))
    for ax, key in zip(axes.flat, sorted(keys), strict=False):
        with np.load(output / "closed_loop/traces" / f"{key}.npz") as z:
            f = dict(zip([str(c) for c in z["columns"]], z["trace"].T.astype(float), strict=True))
        ax.plot(f["target_x"], f["target_y"], color="0.6", label="target")
        ax.plot(f["x"], f["y"], color="C0", label="robot")
        braking = f["mode"] == FW.MODES.index("braking")
        ax.plot(f["x"][braking], f["y"][braking], ".", color="C3", ms=2, label="braking")
        ax.set_title(runs[key]["name"] + (" pass" if runs[key]["passed"] else " FAIL"))
        ax.set_aspect("equal", "datalim")
    axes.flat[0].legend(fontsize=7)
    fig.suptitle("M12 phase D closed loop, seed 0, 0.15 s latency (simulation)")
    fig.tight_layout()
    fig.savefig(output / "closed_loop/paths.png", dpi=110)
    plt.close(fig)


def finish():
    out = FL.OUTPUT
    cfg = FW.load()
    summary = {
        "evidence": EVIDENCE,
        "follower_config_sha256": hashlib.sha256(FW.CONFIG.read_bytes()).hexdigest(),
        "follower_source_sha256": hashlib.sha256(
            FW.SOURCE.read_bytes() + FW.API.read_bytes() + FW.HEADER.read_bytes()
        ).hexdigest(),
        "margin": [FL.margin(cfg, latency) for latency in FL.SWEEP_LATENCY_S],
    }
    tune = out / "tune/summary.json"
    if tune.exists():
        summary["tune"] = json.loads(tune.read_text())
    rows = out / "open_loop/runs.json"
    if rows.exists():
        summary["open_loop"] = open_summary(json.loads(rows.read_text()))
    runs = out / "closed_loop/runs.json"
    if runs.exists():
        r = json.loads(runs.read_text())
        summary["closed_loop"] = closed_summary(r)
        plot(r, out)
    (out / "config.json").write_bytes(FW.CONFIG.read_bytes())
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=float) + "\n")
    return summary
