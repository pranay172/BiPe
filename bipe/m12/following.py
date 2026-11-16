"""M12 phase D: the following controller against the phase C measurement model.

.venv/bin/python -m bipe.m12.following [--workers N] [--stage all|tune|open|closed|finish]

Stages (outputs in artifacts/m12/following/, each with its configuration):
- tune: the estimator's process noise density, open loop on a subset of scenarios;
- open: open loop. The robot moves along the phase A ideal-follower path; the Pi link
  model reports from the true geometry; the C follower estimates the target and its
  commands are logged, not applied. Estimate error against truth, with and without the
  latency compensation, over seeds and a latency sweep;
- closed: closed loop in the free-base M10 plant. The C balance core with the sensor
  model, finite motor limits and physical contact; the C follower turns the Pi link
  model's reports (from the simulator's true geometry) into the commands. All twelve
  scenarios over seeds, a latency sweep and a subset of the robustness variants;
- finish: the summaries, the requirement checks and the plots from the saved runs.

Evidence: validated simulation on the measurement model (rendered-frame statistics, not
rendered perception in the loop, which is phase F) under stated assumptions; not hardware.
"""

import argparse
import copy
import ctypes
import hashlib
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np

from ..parameters import ROOT
from . import follow as FW
from . import follower as F
from . import geometry as G
from . import perception as PC
from . import pi_model as PM
from . import requirements as RQ
from . import scenarios as S
from . import speed as SP

OUTPUT = ROOT / "artifacts/m12/following"
FOLLOW_MODES = FW.MODES
# The phase A trace's modes (bipe.m12.follower.MODES) for the requirement checks.
TO_PHASE_A = {
    "off": "waiting",
    "waiting": "waiting",
    "following": "following",
    "coasting": "coasting",
    "braking": "braking",
    "holding": "holding",
}
ROW = [
    "t",
    "x",
    "y",
    "yaw",
    "speed",
    "yaw_rate",
    "pitch_error_deg",
    "lean_estimate_deg",
    "est_x",
    "est_y",
    "est_yaw",
    "est_speed",
    "command_v",
    "command_w",
    "mode",
    "event",
    "target_x",
    "target_y",
    "range",
    "bearing",
    "view_angle",
    "pixels",
    "in_view",
    "occluded",
    "visible",
    "est_range",
    "est_bearing",
    "reports_arrived",
    "detections_arrived",
]
CLOSED_SEEDS = 10
OPEN_SEEDS = 20
SWEEP_LATENCY_S = (0.10, 0.15, 0.20)
SWEEP_SEEDS = 3
VARIANT_SCENARIOS = ("straight_1.0", "curve_r1", "corner_90", "lateral_crossing", "loss_fov")
VARIANTS = ("mass_plus_15pct", "wheel_lag_10ms", "delay_15ms_noise_x2", "friction_0p5")
SLOPE_CASES = (("straight_0.6", 4.0), ("straight_0.6", -4.0))
TUNE_DENSITIES = (0.25, 0.5, 1.0, 2.0, 4.0)
TUNE_SCENARIOS = ("straight_1.0", "curve_r1", "corner_90", "lateral_crossing", "start_stop")
SETTLE_S = 3.0


def _sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


def _seed(*parts):
    return int.from_bytes(hashlib.sha256(repr(parts).encode()).digest()[:8], "little")


def setup():
    """Shared inputs: requirements, perception model, follower config, sight, scenarios."""
    shared, following, p, perc = PC.load()
    cfg = FW.load()
    link = cfg["link_model"]
    layout = following["tag"]["layouts"][link["layout"]]
    mount = G.camera_mount(shared)
    lens = shared["m12"]["camera"]["lenses"][perc["camera"]["lens"]]
    sight = F.Sight(p, mount, lens, following, layout)
    nominal = following["range"]["nominal_m"]
    m = dict(mount)
    m["pitch_deg"] = layout["camera_pitch_deg"]
    ahead = G.camera_pose(p, m, 40)["ahead_of_axle_m"]
    defs = S.definitions(following, nominal + ahead)
    return {
        "shared": shared,
        "following": following,
        "p": p,
        "perc": perc,
        "cfg": cfg,
        "link": link,
        "sight": sight,
        "defs": defs,
        "ahead": ahead,
        "nominal": nominal,
    }


def latency_shift(link, centre_s):
    lo, hi = link["tracking_latency_s"]
    return centre_s - 0.5 * (lo + hi)


def _make_link(env, tracks, seed, shift):
    perc = env["perc"]
    return PM.PiLink(
        env["link"],
        perc["measurement_model"],
        perc["camera"]["exposure"]["exposure_s"],
        env["cfg"]["timing"]["link_delay_s"],
        env["sight"],
        tracks,
        seed,
        shift,
    )


def _make_rendered_link(env, tracks, seed, shift, rendered, slope):
    """The phase F link: the head camera rendered from the plant in the loop.
    rendered: {"lighting": name, "keep_frames": [sequence numbers]}."""
    from . import rendered_link as RL

    rcfg = RL.load()
    exposure = env["perc"]["camera"]["exposure"]
    return RL.RenderedLink(
        env["link"],
        rendered.get("exposure_s", rcfg.get("exposure_s", exposure["exposure_s"])),
        exposure["subframes"],
        env["cfg"]["timing"]["link_delay_s"],
        tracks,
        seed,
        shift,
        layout=rcfg["layout"],
        lighting=rcfg["lighting"][rendered.get("lighting", "nominal")],
        slope_deg=slope or 0.0,
        check_repeat=rendered.get("check_repeat", rcfg["check_repeat"]),
        keep_frames=rendered.get("keep_frames", ()),
        pose_conversion=rendered.get("pose_conversion", rcfg["pose_conversion"]),
        tracker_policy=rcfg["tracker_policies"][rendered.get("tracker", rcfg["tracker"])],
        sight=env["sight"],
        view_gate_deg=rendered.get("view_gate_deg", rcfg.get("view_gate_deg")),
    )


def _deliver(follower, reports):
    n = d = 0
    for arrival, r in reports:
        follower.report(
            r["sequence"],
            r["tag_id"],
            r["detected"],
            r["range_m"],
            r["bearing_rad"],
            r["age_s"],
            arrival,
        )
        n += 1
        d += int(r["detected"])
    return n, d


def _truth_row(env, tracks, t, robot):
    tau = t - env["link"]["start_s"]
    s = S.at(tracks["target"], tau)
    pos = np.array([s["x"], s["y"]]) + robot["origin"]
    camera, _ = env["sight"].camera(robot["x"], robot["y"], robot["yaw"], robot["acceleration"])
    tag_yaw = s["tag_yaw"]
    if math.isnan(tag_yaw):
        tag_yaw = math.atan2(camera[1] - pos[1], camera[0] - pos[0])
    occluder = None
    if "occluder" in tracks:
        o = S.at(tracks["occluder"], tau)
        occluder = np.array([o["x"], o["y"]]) + robot["origin"]
    seen = env["sight"].look(
        (robot["x"], robot["y"], robot["yaw"]), robot["acceleration"], pos, tag_yaw, occluder
    )
    return pos, seen


# Open loop ----------------------------------------------------------------------------


def open_task(payload):
    """One open-loop run: the ideal follower's path, the link model and two C followers
    (with the latency compensation, and a baseline that takes each report as current)."""
    name, seed, centre, density = payload
    env = setup()
    cfg = copy.deepcopy(env["cfg"])
    if density is not None:
        cfg["estimator"]["process_acceleration_density_m2_s3"] = density
    tracks = S.tracks(env["defs"][name], env["nominal"] + env["ahead"], env["nominal"])
    f = SP.follower_trace(env["link"]["layout"], name)
    start = env["link"]["start_s"]
    dt = cfg["timing"]["control_dt_s"]
    duration = start + float(f["t"][-1]) + 1.0
    link = _make_link(env, tracks, _seed("open", name, seed), latency_shift(env["link"], centre))
    comp, base = FW.CFollower(cfg), FW.CFollower(cfg)
    g = env["p"]["sizing"]["gravity"]
    rows = []
    for k in range(1, round(duration / dt) + 1):
        t = k * dt
        tau = t - start
        i = min(max(0, math.floor(tau / 0.01)), len(f["t"]) - 1)
        x, y, yaw = float(f["x"][i]), float(f["y"][i]), float(f["yaw"][i])
        speed = float(f["speed"][i]) if tau >= 0 else 0.0
        accel = float(f["acceleration"][i]) if tau >= 0 else 0.0
        robot = {"x": x, "y": y, "yaw": yaw, "acceleration": accel, "origin": np.zeros(2)}
        if t >= env["link"]["follow_request_s"] - 1e-9:
            comp.request(True)
            base.request(True)
        reports = link.poll(t, robot)
        n, d = _deliver(comp, reports)
        _deliver(base, [(a, {**r, "age_s": 0.0}) for a, r in reports])
        lean = math.atan2(accel, g)
        v, w = comp.step(t, x, y, yaw, lean, speed, True)
        base.step(t, x, y, yaw, lean, speed, True)
        pos, seen = _truth_row(env, tracks, t, robot)
        e_c, e_b = comp.estimate(), base.estimate()
        sc, sb = comp.s, base.s
        rows.append(
            [
                t,
                comp.mode,
                comp.event,
                pos[0],
                pos[1],
                seen["range"],
                seen["bearing"],
                float(seen["visible"]),
                sc[0],
                sc[1],
                e_c[0] if e_c else math.nan,
                e_c[1] if e_c else math.nan,
                sb[0],
                sb[1],
                e_b[0] if e_b else math.nan,
                e_b[1] if e_b else math.nan,
                v,
                w,
                n,
                d,
            ]
        )
    cols = [
        "t",
        "mode",
        "event",
        "target_x",
        "target_y",
        "range",
        "bearing",
        "visible",
        "est_x",
        "est_y",
        "est_range",
        "est_bearing",
        "base_x",
        "base_y",
        "base_range",
        "base_bearing",
        "command_v",
        "command_w",
        "reports",
        "detections",
    ]
    tr = np.array(rows)
    return {
        "name": name,
        "seed": seed,
        "latency_centre_s": centre,
        "density": density,
        "columns": cols,
        "trace": tr,
        "frames": link.frames,
        "reports": dict(zip(FW.REPORTS, comp.reports, strict=True)),
    }


def parity_task(payload):
    """The Python reference and the C follower on one scenario's open-loop report stream:
    the largest difference in commands, state and covariance, and any mode mismatch."""
    name, seed, centre = payload
    env = setup()
    tracks = S.tracks(env["defs"][name], env["nominal"] + env["ahead"], env["nominal"])
    f = SP.follower_trace(env["link"]["layout"], name)
    start = env["link"]["start_s"]
    dt = env["cfg"]["timing"]["control_dt_s"]
    duration = start + float(f["t"][-1]) + 1.0
    link = _make_link(env, tracks, _seed("open", name, seed), latency_shift(env["link"], centre))
    cf, py = FW.CFollower(env["cfg"]), FW.Follower(env["cfg"])
    g = env["p"]["sizing"]["gravity"]
    worst, mismatches, modes, reports = 0.0, 0, set(), 0
    for k in range(1, round(duration / dt) + 1):
        t = k * dt
        tau = t - start
        i = min(max(0, math.floor(tau / 0.01)), len(f["t"]) - 1)
        x, y, yaw = float(f["x"][i]), float(f["y"][i]), float(f["yaw"][i])
        speed = float(f["speed"][i]) if tau >= 0 else 0.0
        accel = float(f["acceleration"][i]) if tau >= 0 else 0.0
        robot = {"x": x, "y": y, "yaw": yaw, "acceleration": accel, "origin": np.zeros(2)}
        if t >= env["link"]["follow_request_s"] - 1e-9:
            cf.request(True)
            py.request(True)
        arrived = link.poll(t, robot)
        for arrival, r in arrived:
            a = cf.report(
                *(r[k] for k in ("sequence", "tag_id", "detected", "range_m")),
                r["bearing_rad"],
                r["age_s"],
                arrival,
            )
            b = py.report(
                *(r[k] for k in ("sequence", "tag_id", "detected", "range_m")),
                r["bearing_rad"],
                r["age_s"],
                arrival,
            )
            mismatches += a != b
            reports += 1
        lean = math.atan2(accel, g)
        a = cf.step(t, x, y, yaw, lean, speed, True)
        b = py.step(t, x, y, yaw, lean, speed, True)
        mismatches += (cf.mode, cf.event) != (py.mode, py.event)
        modes.add(FW.MODES[cf.mode])
        worst = max(
            worst,
            abs(a[0] - b[0]),
            abs(a[1] - b[1]),
            max(abs(p - q) for p, q in zip(cf.s, py.s, strict=True)),
            max(abs(p - q) for p, q in zip(cf.P, py.P, strict=True)),
        )
    return {
        "name": name,
        "seed": seed,
        "latency_centre_s": centre,
        "reports": reports,
        "max_difference": worst,
        "mismatches": mismatches,
        "modes": sorted(modes),
    }


def parity(workers):
    env = setup()
    payloads = [(n, s, c) for n in env["defs"] for s in range(2) for c in SWEEP_LATENCY_S]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(parity_task, payloads))
    out = {
        "runs": rows,
        "max_difference": max(r["max_difference"] for r in rows),
        "mismatches": sum(r["mismatches"] for r in rows),
    }
    out["passed"] = out["max_difference"] <= 1e-9 and out["mismatches"] == 0
    (OUTPUT / "parity.json").write_text(json.dumps(out, indent=1) + "\n")
    return out


def open_metrics(run):
    f = dict(zip(run["columns"], run["trace"].T, strict=True))
    tracking = np.isin(f["mode"], [FOLLOW_MODES.index("following"), FOLLOW_MODES.index("coasting")])
    out = {"ticks_tracking": int(tracking.sum())}
    for key, px, py, pr, pb in (
        ("compensated", "est_x", "est_y", "est_range", "est_bearing"),
        ("uncompensated", "base_x", "base_y", "base_range", "base_bearing"),
    ):
        pos = np.hypot(f[px] - f["target_x"], f[py] - f["target_y"])[tracking]
        rng = (f[pr] - f["range"])[tracking]
        brg = np.degrees(np.vectorize(PM._wrap)(f[pb] - f["bearing"]))[tracking]
        if len(pos) == 0:
            out[key] = None
            continue
        out[key] = {
            "position_rms_m": float(np.sqrt(np.mean(pos**2))),
            "position_p95_m": float(np.percentile(pos, 95)),
            "position_max_m": float(np.max(pos)),
            "range_mean_m": float(np.mean(rng)),
            "range_rms_m": float(np.sqrt(np.mean(rng**2))),
            "bearing_mean_deg": float(np.mean(brg)),
            "bearing_rms_deg": float(np.sqrt(np.mean(brg**2))),
            "bearing_max_abs_deg": float(np.max(np.abs(brg))),
        }
    ev = [
        (float(t), FW.EVENTS[int(e)])
        for t, e in zip(f["t"], f["event"], strict=True)
        if int(e) != 0
    ]
    out["events"] = ev
    out["reports"] = run["reports"]
    frames = run["frames"]
    out["frames"] = len(frames)
    out["frames_possible"] = sum(fr["possible"] for fr in frames)
    out["frames_detected"] = sum(fr["detected"] for fr in frames)
    return out


# Closed loop --------------------------------------------------------------------------


def closed_task(payload):
    """One closed-loop run in the free-base M10 plant with the C core and the C follower."""
    from ..firmware_control import control_core
    from ..m10.kinematics import posture
    from ..moving_experiment import simulate

    name, seed, centre, variant, slope, library, *rest = payload
    overrides = rest[0] if rest else None
    env = setup()
    cfg = copy.deepcopy(env["cfg"])
    overrides = copy.deepcopy(overrides or {})
    diagnostic = overrides.pop("diagnostic", {})
    rendered = overrides.pop("rendered", None)
    for section, values in overrides.items():
        cfg[section].update(values)
    p, gains, sensors, _ = SP.load()
    plant, config, sim_seed = SP.variant_inputs(p, sensors, variant)
    tracks = S.tracks(env["defs"][name], env["nominal"] + env["ahead"], env["nominal"])
    start = env["link"]["start_s"]
    duration = round(start + float(tracks["target"]["t"][-1]) + SETTLE_S, 2)
    link_seed = _seed("closed", name, seed, variant, slope)
    shift = latency_shift(env["link"], centre)
    if rendered is not None:
        link = _make_rendered_link(env, tracks, link_seed, shift, rendered, slope)
    else:
        link = _make_link(env, tracks, link_seed, shift)
    follower = FW.CFollower(cfg)
    core = control_core(library)
    qref = posture(40, p)
    g = env["p"]["sizing"]["gravity"]
    rows = []
    state = {"origin": None, "requested": False}

    def command(t, scenario):
        return 0.0, 0.0, qref, np.zeros(4)

    def hook(t, est, s):
        if state["origin"] is None:
            state["origin"] = np.array(s["position"][:2], dtype=float)
        if not state["requested"] and t >= env["link"]["follow_request_s"] - 1e-9:
            follower.request(True)
            state["requested"] = True
        lean = s["pitch"] - s["equilibrium"]
        robot = {
            "x": float(s["position"][0]),
            "y": float(s["position"][1]),
            "yaw": float(s["yaw"]),
            "acceleration": 0.0 if diagnostic.get("upright_camera") else g * math.tan(lean),
            "origin": state["origin"],
        }
        n, d = _deliver(follower, link.poll(t, robot))
        ex, ey = float(est["position"][0]), float(est["position"][1])
        # The firmware's lean: estimated pitch less the equilibrium of the estimated joints.
        lean_est = float(est["pitch"]) - _equilibrium(core, est["q"])
        pose = (ex, ey, float(est["yaw"]))
        if diagnostic.get("truth_pose"):
            pose = (robot["x"], robot["y"], robot["yaw"])
        out = follower.step(t, *pose, lean_est, float(est["velocity"]), True)
        v, w = out if out is not None else (0.0, 0.0)
        if diagnostic.get("lowpass_s"):
            k = 0.005 / (diagnostic["lowpass_s"] + 0.005)
            state["v"] = state.get("v", 0.0) + k * (v - state.get("v", 0.0))
            state["w"] = state.get("w", 0.0) + k * (w - state.get("w", 0.0))
            v, w = state["v"], state["w"]
        if diagnostic.get("standstill_m_s") and abs(v) < diagnostic["standstill_m_s"]:
            v = 0.0
        v *= diagnostic.get("scale_v", 1.0)
        w *= diagnostic.get("scale_w", 1.0)
        pos, seen = _truth_row(env, tracks, t, robot)
        e = follower.estimate()
        rows.append(
            [
                t,
                robot["x"],
                robot["y"],
                robot["yaw"],
                float(s["velocity"]),
                float(s["yaw_rate"]),
                math.degrees(lean),
                math.degrees(lean_est),
                ex,
                ey,
                float(est["yaw"]),
                float(est["velocity"]),
                v,
                w,
                follower.mode,
                follower.event,
                pos[0],
                pos[1],
                seen["range"],
                seen["bearing"],
                seen["view_angle"],
                seen["pixels"],
                float(seen["in_view"]),
                float(seen["occluded"]),
                float(seen["visible"]),
                e[0] if e else math.nan,
                e[1] if e else math.nan,
                n,
                d,
            ]
        )
        return v, w

    disturbance = {"slope_deg": slope} if slope else {}
    run = simulate(
        "m12_following",
        duration,
        sensing="sensors",
        seed=sim_seed,
        nominal=p,
        plant=plant,
        gains=gains,
        sensor_config=config,
        implementation="c",
        firmware_library=library,
        physics_dt=plant["simulation"]["physics_dt"],
        disturbance=disturbance,
        command_function=command,
        command_hook=hook,
        observer=link.record if rendered is not None else None,
    )
    out = {
        "name": name,
        "seed": seed,
        "latency_centre_s": centre,
        "variant": variant,
        "slope_deg": slope,
        "overrides": overrides,
        "duration_s": duration,
        "failure": run["failure"],
        "peak_pitch_deg": float(run["stats"]["peak_pitch_deg"]),
        "peak_roll_deg": float(run["stats"]["peak_roll_deg"]),
        "columns": ROW,
        "trace": np.array(rows),
        "frames": link.frames,
        "reports": dict(zip(FW.REPORTS, follower.reports, strict=True)),
    }
    if rendered is not None:
        out["rendered"] = rendered
        out["repeat_mismatches"] = link.repeat_mismatches
        out["kept_frames"] = link.kept
        link.close()
    return out


def _equilibrium(core, q):
    d = ctypes.c_double
    qa, com, axle = (d * 4)(*q), (d * 2)(), (d * 2)()
    eq, scale = d(), d()
    if not core.bipe_kinematics(qa, com, axle, ctypes.byref(eq), ctypes.byref(scale)):
        raise RuntimeError("estimated joints outside the kinematics")
    return eq.value


def phase_a_result(run):
    """The closed-loop trace in the phase A follower's format, for its requirement checks."""
    full = dict(zip(run["columns"], run["trace"].T, strict=True))
    # The phase A summary counts time in its 0.01 s steps: take every second control tick.
    f = {k: v[1::2] for k, v in full.items()}
    t = f["t"]
    accel = np.gradient(f["speed"], t)
    yaw_accel = np.gradient(f["yaw_rate"], t)
    modes = [F.MODES.index(TO_PHASE_A[FOLLOW_MODES[int(m)]]) for m in f["mode"]]
    trace = np.column_stack(
        [
            t,
            f["x"],
            f["y"],
            f["yaw"],
            f["speed"],
            f["yaw_rate"],
            accel,
            yaw_accel,
            modes,
            f["visible"],
            f["occluded"],
            f["range"],
            f["bearing"],
            f["view_angle"],
            f["pixels"],
            f["detections_arrived"] > 0,
        ]
    )
    f = full
    t = f["t"]
    events, lost_at = [], None
    for i in np.flatnonzero(f["event"]):
        ev = FW.EVENTS[int(f["event"][i])]
        x, y = f["x"][i], f["y"][i]
        if ev == "engaged":
            events.append({"t": float(t[i]), "event": "engaged"})
        elif ev == "lost":
            lost_at = (x, y)
            events.append({"t": float(t[i]), "event": "lost", "speed": float(f["speed"][i])})
        elif ev == "reacquired":
            events.append({"t": float(t[i]), "event": "reacquired"})
        elif ev == "holding":
            d = float(np.hypot(x - lost_at[0], y - lost_at[1])) if lost_at else 0.0
            events.append({"t": float(t[i]), "event": "holding", "stopping_distance_m": d})
        elif ev == "coasting":
            events.append({"t": float(t[i]), "event": "coasting"})
    return {"columns": F.COLUMNS, "trace": trace, "events": events}


def _spans(t, mode, name):
    """Each stretch of one follower mode: start, length and the mode that ended it."""
    code = FOLLOW_MODES.index(name)
    out, i = [], 0
    while i < len(mode):
        if mode[i] != code:
            i += 1
            continue
        j = i
        while j < len(mode) and mode[j] == code:
            j += 1
        then = FOLLOW_MODES[int(mode[j])] if j < len(mode) else "end"
        out.append(
            {"start_s": float(t[i]), "length_s": float(t[min(j, len(t) - 1)] - t[i]), "then": then}
        )
        i = j
    return out


def closed_metrics(run, env, speed_cfg):
    following = env["following"]
    name = run["name"]
    res = phase_a_result(run)
    # The phase A checks count a brief loss as "lost" then "reacquired"; a coast that
    # recovers before braking is reported separately.
    checks_events, after_loss = [], False
    for e in res["events"]:
        if e["event"] == "coasting" or (e["event"] == "reacquired" and not after_loss):
            continue
        after_loss = e["event"] == "lost"
        checks_events.append(e)
    summary = F.summary(
        {**res, "events": checks_events}, following, straight=name.startswith("straight")
    )
    checks = RQ.check(name, env["defs"][name], summary, following)
    f = dict(zip(run["columns"], run["trace"].T, strict=True))
    t = f["t"]
    dt = np.diff(t)
    lim = env["cfg"]["limits"]
    dv = np.abs(np.diff(f["command_v"])) / dt
    dw = np.abs(np.diff(f["command_w"])) / dt
    envelope = {
        "speed_max_m_s": float(f["command_v"].max()),
        "reverse_max_m_s": float(max(0.0, -f["command_v"].min())),
        "yaw_rate_max_rad_s": float(np.abs(f["command_w"]).max()),
        "acceleration_max_m_s2": float(dv.max()),
        "yaw_acceleration_max_rad_s2": float(dw.max()),
    }
    envelope["inside"] = bool(
        envelope["speed_max_m_s"] <= lim["speed_m_s"] + 1e-9
        and envelope["reverse_max_m_s"] <= lim["reverse_speed_m_s"] + 1e-9
        and envelope["yaw_rate_max_rad_s"] <= lim["yaw_rate_rad_s"] + 1e-9
        and envelope["acceleration_max_m_s2"] <= lim["acceleration_m_s2"] + 1e-6
        and envelope["yaw_acceleration_max_rad_s2"] <= lim["yaw_acceleration_rad_s2"] + 1e-6
    )
    tracking = np.isin(f["mode"], [FOLLOW_MODES.index("following"), FOLLOW_MODES.index("coasting")])
    est = {}
    if tracking.any():
        re = (f["est_range"] - f["range"])[tracking]
        be = np.degrees(np.vectorize(PM._wrap)(f["est_bearing"] - f["bearing"]))[tracking]
        est = {
            "range_mean_m": float(np.mean(re)),
            "range_rms_m": float(np.sqrt(np.mean(re**2))),
            "range_max_abs_m": float(np.max(np.abs(re))),
            "bearing_rms_deg": float(np.sqrt(np.mean(be**2))),
            "bearing_max_abs_deg": float(np.max(np.abs(be))),
        }
    kinds = [e["event"] for e in res["events"]]
    brief = _spans(t, f["mode"], "coasting")
    braking = _spans(t, f["mode"], "braking")
    gate = speed_cfg["criteria"]["peak_pitch_max_deg"]
    out = {
        "name": name,
        "seed": run["seed"],
        "latency_centre_s": run["latency_centre_s"],
        "variant": run["variant"],
        "slope_deg": run["slope_deg"],
        "failure": run["failure"],
        "fell": run["failure"] is not None,
        "peak_pitch_deg": run["peak_pitch_deg"],
        "pitch_within_gate": run["peak_pitch_deg"] <= gate,
        "summary": {k: v for k, v in summary.items() if k != "events"},
        "checks": checks,
        "events": res["events"],
        "coasts": brief,
        "brakes": braking,
        "losses_to_hold": kinds.count("holding"),
        "envelope": envelope,
        "estimate": est,
        "reports": run["reports"],
        "frames": len(run["frames"]),
        "frames_possible": sum(fr["possible"] for fr in run["frames"]),
        "frames_detected": sum(fr["detected"] for fr in run["frames"]),
    }
    out["passed"] = bool(
        checks["expectation_met"]
        and not out["fell"]
        and out["pitch_within_gate"]
        and envelope["inside"]
    )
    return out


# Build --------------------------------------------------------------------------------


def closed_payloads(library):
    env = setup()
    names = list(env["defs"])
    out = []
    for name in names:
        for seed in range(CLOSED_SEEDS):
            out.append(("nominal", (name, seed, 0.15, None, 0.0, library)))
    for centre in SWEEP_LATENCY_S:
        if centre == 0.15:
            continue
        for name in names:
            for seed in range(SWEEP_SEEDS):
                out.append((f"latency_{centre:.2f}", (name, seed, centre, None, 0.0, library)))
    for variant in VARIANTS:
        for name in VARIANT_SCENARIOS:
            out.append(("variants", (name, 0, 0.15, variant, 0.0, library)))
    for name, slope in SLOPE_CASES:
        out.append(("slopes", (name, 0, 0.15, None, slope, library)))
    return out


def run_key(payload):
    name, seed, centre, variant, slope, *_ = payload
    parts = [name, f"seed{seed}", f"lat{centre:.2f}"]
    if variant:
        parts.append(variant)
    if slope:
        parts.append(f"slope{slope:+g}")
    return "_".join(parts)


TRACE_DT_S = 0.02


def _thin(run):
    """The saved trace at 0.02 s (as the speed records), keeping every row with a follower
    event and the last row; the metrics are computed from the full 200 Hz trace."""
    tr = run["trace"]
    event = tr[:, run["columns"].index("event")]
    keep = np.zeros(len(tr), bool)
    keep[:: round(TRACE_DT_S / 0.005)] = True
    keep[event != 0] = True
    keep[-1] = True
    return tr[keep]


def _closed_worker(item):
    group, payload = item
    run = closed_task(payload)
    return group, run_key(payload), run


def build_closed(workers):
    p, gains, _, speed_cfg = SP.load()
    from ..profiles import native
    from ..robustness_control import build_candidate

    library = str(build_candidate(p, gains, ROOT / native(p)["firmware_build"]))
    env = setup()
    out = OUTPUT / "closed_loop"
    (out / "traces").mkdir(parents=True, exist_ok=True)
    items = closed_payloads(library)
    results = {}
    done = {}
    record = out / "runs.json"
    if record.exists():
        done = json.loads(record.read_text())
    todo = [it for it in items if run_key(it[1]) not in done]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers, max_tasks_per_child=1) as pool:
        futures = [pool.submit(_closed_worker, it) for it in todo]
        for fut in as_completed(futures):
            group, key, run = fut.result()
            np.savez_compressed(
                out / "traces" / f"{key}.npz",
                columns=np.array(run["columns"]),
                trace=_thin(run).astype(np.float32),
            )
            metrics = closed_metrics(run, env, speed_cfg)
            metrics["group"] = group
            done[key] = metrics
            record.write_text(json.dumps(done, indent=1, default=float) + "\n")
            print(
                "M12 FOLLOWING closed",
                key,
                "passed" if metrics["passed"] else "FAILED",
                metrics["failure"] or "",
                f"{time.time() - t0:.0f} s",
                flush=True,
            )
    results = done
    return results


def _open_worker(payload):
    run = open_task(payload)
    return payload, open_metrics(run)


def build_open(workers, densities=None, names=None, seeds=OPEN_SEEDS, centres=SWEEP_LATENCY_S):
    env = setup()
    names = names or list(env["defs"])
    payloads = [
        (name, seed, centre, density)
        for density in (densities or [None])
        for centre in centres
        for name in names
        for seed in range(seeds)
    ]
    out = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for fut in as_completed([pool.submit(_open_worker, p) for p in payloads]):
            (name, seed, centre, density), m = fut.result()
            out.append(
                {"name": name, "seed": seed, "latency_centre_s": centre, "density": density, **m}
            )
    out.sort(key=lambda r: (r["density"] or 0, r["latency_centre_s"], r["name"], r["seed"]))
    return out


def _pool(rows, key, field):
    vals = [r[key][field] for r in rows if r.get(key)]
    return float(np.sqrt(np.mean(np.square(vals)))) if vals else None


def tune(workers):
    rows = build_open(
        workers, densities=TUNE_DENSITIES, names=TUNE_SCENARIOS, seeds=4, centres=(0.15,)
    )
    table = []
    for q in TUNE_DENSITIES:
        rs = [r for r in rows if r["density"] == q]
        table.append(
            {
                "density_m2_s3": q,
                "position_rms_m": _pool(rs, "compensated", "position_rms_m"),
                "range_rms_m": _pool(rs, "compensated", "range_rms_m"),
                "bearing_rms_deg": _pool(rs, "compensated", "bearing_rms_deg"),
            }
        )
    best = min(table, key=lambda r: r["position_rms_m"])
    out = {
        "evidence": "open loop on the measurement model (no plant): the ideal follower's path, "
        "the link model's reports and the C follower",
        "scenarios": list(TUNE_SCENARIOS),
        "seeds": 4,
        "table": table,
        "best_density_m2_s3": best["density_m2_s3"],
        "configured_density_m2_s3": FW.load()["estimator"]["process_acceleration_density_m2_s3"],
    }
    (OUTPUT / "tune").mkdir(parents=True, exist_ok=True)
    (OUTPUT / "tune/summary.json").write_text(json.dumps(out, indent=1) + "\n")
    return out


def open_loop(workers):
    rows = build_open(workers)
    (OUTPUT / "open_loop").mkdir(parents=True, exist_ok=True)
    (OUTPUT / "open_loop/runs.json").write_text(json.dumps(rows, indent=1, default=float) + "\n")
    return rows


def margin(cfg, latency_s, lag_s=0.2):
    """Phase margin of the linearised bearing loop k e^(-s tau) / (s (1 + lag s)), with the
    delay either left whole (no compensation) or removed by the prediction."""
    k = cfg["law"]["bearing_gain_1_s"]
    # Crossover of k / (w sqrt(1 + (lag w)^2)) = 1.
    lo, hi = 1e-3, 100.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if k / (mid * math.sqrt(1 + (lag_s * mid) ** 2)) > 1:
            lo = mid
        else:
            hi = mid
    wc = 0.5 * (lo + hi)
    base = 180 - 90 - math.degrees(math.atan(lag_s * wc))
    return {
        "crossover_rad_s": wc,
        "phase_margin_deg_compensated": base,
        "phase_margin_deg_uncompensated": base - math.degrees(wc * latency_s),
        "lag_s": lag_s,
        "latency_s": latency_s,
    }


def _differences(a, b, path=""):
    """Paths at which two JSON values differ (exact comparison)."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                out.append(f"{path}/{k}")
            else:
                out += _differences(a[k], b[k], f"{path}/{k}")
        return out
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [
            d
            for i, (x, y) in enumerate(zip(a, b, strict=True))
            for d in _differences(x, y, f"{path}/{i}")
        ]
    return [] if a == b else [path or "/"]


def compare_m10(rerun):
    """The rerun's M10 cases against the record, case by case (an acceptance-phase rerun
    covers the record's acceptance cases, not its robustness-phase cases)."""
    import csv

    m10 = ROOT / "artifacts/m10/acceptance"
    a = json.loads((m10 / "summary.json").read_text())["controllers"]["current"]["cases"]
    b = json.loads((rerun / "summary.json").read_text())["controllers"]["current"]["cases"]
    common = sorted(set(a) & set(b))

    def rows(path):
        with open(path) as f:
            return {(r["controller"], r["case"]): r for r in csv.DictReader(f)}

    ra, rb = rows(m10 / "metrics.csv"), rows(rerun / "metrics.csv")
    out = {
        "record_cases": len(a),
        "rerun_cases": len(b),
        "compared": len(common),
        "only_in_rerun": sorted(set(b) - set(a)),
        "not_rerun": len(set(a) - set(b)),
        "summary_differences": [d for n in common for d in _differences(a[n], b[n], n)],
        "metrics_rows_differing": sorted(k[1] for k in rb if ra.get(k) != rb[k]),
    }
    out["identical"] = bool(
        common
        and not out["only_in_rerun"]
        and not out["summary_differences"]
        and not out["metrics_rows_differing"]
    )
    return out


def regression(workers):
    """With following off, the M12 speed cases and the M10 acceptance and robustness phases
    must reproduce their records bit for bit (the simulator hook and the follower are
    inactive)."""
    import subprocess
    import sys

    out = OUTPUT / "regression"
    out.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    subprocess.run(
        [py, "-m", "bipe.m12.speed", "--output", str(out / "speed"), "--workers", str(workers)],
        check=True,
        cwd=ROOT,
    )
    for phase in ("acceptance", "robustness"):
        subprocess.run(
            [
                py,
                "-m",
                "bipe.robustness",
                "--profile",
                "m10",
                "--controllers",
                "current",
                "--phase",
                phase,
                "--output",
                str(out / f"m10_{phase}"),
                "--workers",
                str(workers),
            ],
            check=True,
            cwd=ROOT,
        )
    return regression_summary()


def regression_summary():
    """Compare the reruns in regression/ with the records and write regression/summary.json."""
    import shutil

    out = OUTPUT / "regression"
    speed_out = out / "speed"
    record = ROOT / "artifacts/m12/speed"
    result = {
        "evidence": "following off: the simulator's command hook is not given and the "
        "balance core library is built from the unchanged sources",
    }
    if (speed_out / "traces").exists():
        speed_cases = _differences(
            json.loads((record / "cases.json").read_text()),
            json.loads((speed_out / "cases.json").read_text()),
        )
        traces = []
        for path in sorted((record / "traces").glob("*.npz")):
            with np.load(path) as a, np.load(speed_out / "traces" / path.name) as b:
                if not (
                    np.array_equal(a["trace"], b["trace"])
                    and np.array_equal(a["columns"], b["columns"])
                ):
                    traces.append(path.name)
        result["speed"] = {
            "cases": len(json.loads((record / "cases.json").read_text())),
            "cases_json_differences": speed_cases,
            "traces_compared": len(list((record / "traces").glob("*.npz"))),
            "traces_differing": traces,
        }
    else:
        # The identical speed traces were dropped after an earlier comparison: keep its result.
        result["speed"] = json.loads((out / "summary.json").read_text())["speed"]
    speed_cases = result["speed"]["cases_json_differences"]
    traces = result["speed"]["traces_differing"]
    for phase in ("acceptance", "robustness"):
        result[f"m10_{phase}"] = compare_m10(out / f"m10_{phase}")
    covered = (
        result["m10_acceptance"]["compared"] + result["m10_robustness"]["compared"]
        == result["m10_acceptance"]["record_cases"]
    )
    result["m10_record_covered"] = covered
    result["identical"] = (
        not speed_cases
        and not traces
        and covered
        and result["m10_acceptance"]["identical"]
        and result["m10_robustness"]["identical"]
    )
    (out / "summary.json").write_text(json.dumps(result, indent=1) + "\n")
    if result["identical"]:
        # Identical copies of the record traces are dropped; the rerun summaries, cases and
        # plots stay for this comparison and are not committed (.gitignore).
        for d in (speed_out / "traces", speed_out / "results"):
            shutil.rmtree(d, ignore_errors=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--stage",
        default="all",
        choices=["all", "parity", "tune", "open", "closed", "regression", "compare", "finish"],
    )
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if args.stage in ("all", "parity"):
        r = parity(args.workers)
        print("M12 FOLLOWING parity", r["max_difference"], r["mismatches"], r["passed"], flush=True)
    if args.stage in ("all", "tune"):
        print("M12 FOLLOWING tune", json.dumps(tune(args.workers)), flush=True)
    if args.stage in ("all", "open"):
        open_loop(args.workers)
        print("M12 FOLLOWING open loop done", flush=True)
    if args.stage in ("all", "closed"):
        build_closed(args.workers)
        print("M12 FOLLOWING closed loop done", flush=True)
    if args.stage in ("all", "regression"):
        r = regression(args.workers)
        print("M12 FOLLOWING regression identical", r["identical"], flush=True)
    if args.stage == "compare":
        r = regression_summary()
        print("M12 FOLLOWING regression identical", r["identical"], flush=True)
    if args.stage in ("all", "finish"):
        from .following_report import finish

        print(json.dumps(finish(), indent=1, default=float)[:4000], flush=True)


if __name__ == "__main__":
    main()
