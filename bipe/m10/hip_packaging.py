"""M10 phase B2 step 1: hip drive packaging study.

python -m bipe.m10.hip_packaging

Lays out the four leg joint drives, battery and drivers inside the body
for two architectures, checks primitive overlaps and interior fit, and
reports the effect on width, length and the balanced leg pose.

Frame: hip axis origin, x forward, y left, z up (SI). Parts are boxes or
cylinders with their axis along y. Envelopes are catalog values from M9
(config m9_cad) plus stated assumptions; the study is a screen, not CAD.
"""

import copy
import json
import math

import numpy as np

from bipe.m10 import concept
from bipe.parameters import ROOT

OUT = ROOT / "artifacts/m10/hip-packaging"

# Envelopes (m). RO40 Lite module: M9 housing radius and axial length incl. encoder.
MOTOR_R, MOTOR_W = 0.02465, 0.0155
SMALL_PULLEY_R, LARGE_PULLEY_R, PULLEY_W = 0.0075, 0.027, 0.007  # 2GT 20T/80T with flanges
PLANETARY_W = 0.012  # assumption: single-stage 4:1 planetary in the motor diameter
BEARING_W = 0.004
BULKHEAD = 0.003
CLEAR = 0.0005
BATTERY = (0.032, 0.047, 0.065)  # 6S 850 pack stood on end: x, y, z
MOTEUS = (0.038, 0.009, 0.038)  # moteus-c1 stood vertically: x, y, z
MOTEUS_FLAT = (0.038, 0.038, 0.009)
HOST = (0.040, 0.010, 0.030)  # host, IMU and power board stack (assumption)


def cyl(name, x, z, r, y0, y1, mass=0.0):
    return {
        "name": name,
        "kind": "cyl",
        "x": x,
        "z": z,
        "r": r,
        "y": sorted([y0, y1]),
        "mass": mass,
    }


def box(name, centre, size, mass=0.0):
    return {"name": name, "kind": "box", "c": list(centre), "s": list(size), "mass": mass}


def extent(p):
    if p["kind"] == "cyl":
        return [p["x"] - p["r"], p["x"] + p["r"]], p["y"], [p["z"] - p["r"], p["z"] + p["r"]]
    return [[c - s / 2, c + s / 2] for c, s in zip(p["c"], p["s"], strict=True)]


def overlap(a, b):
    ea, eb = extent(a), extent(b)
    if any(min(ea[i][1], eb[i][1]) - max(ea[i][0], eb[i][0]) <= 1e-6 for i in range(3)):
        return False
    if a["kind"] == "cyl" and b["kind"] == "cyl":
        return math.hypot(a["x"] - b["x"], a["z"] - b["z"]) < a["r"] + b["r"] - 1e-6
    if "cyl" in (a["kind"], b["kind"]):
        c, bx = (a, b) if a["kind"] == "cyl" else (b, a)
        (x0, x1), _, (z0, z1) = extent(bx)
        dx = max(x0 - c["x"], 0, c["x"] - x1)
        dz = max(z0 - c["z"], 0, c["z"] - z1)
        return math.hypot(dx, dz) < c["r"] - 1e-6
    return True


def interior(body):
    w = body["wall"]
    cx = -body["hip_x"]
    bottom = -body["hip_z_above_bottom"]
    return [
        [cx - body["length"] / 2 + w, cx + body["length"] / 2 - w],
        [-body["width"] / 2 + w, body["width"] / 2 - w],
        [bottom + w, bottom + body["height"] - w],
    ]


def outside(p, inner, allow_y=0.0):
    """Distance (m) a part pokes out of the interior; allow_y lets hip parts enter the pod."""
    e = extent(p)
    worst = 0.0
    for i, (lo, hi) in enumerate(e):
        a, b = inner[i]
        if i == 1:
            b, a = b + allow_y, a - allow_y
        worst = max(worst, a - lo, hi - b)
    return worst


def option_rear_row(body, c=0.040):
    """A: four motors on one transverse axis behind the hip, 4:1 belts to concentric hip pulleys."""
    parts = []
    per_joint = body["hip_module_kg"] / 4
    for s in (1, -1):
        y = BULKHEAD / 2 + CLEAR
        for joint in ("knee", "thigh"):
            if joint == "knee":
                # Knee pulley plane inboard; the inner shaft runs out through the thigh drive.
                parts.append(
                    cyl(
                        f"{joint}_small_pulley_{s}",
                        -c,
                        0,
                        SMALL_PULLEY_R,
                        s * y,
                        s * (y + PULLEY_W),
                    )
                )
                parts.append(
                    cyl(
                        f"{joint}_large_pulley_{s}",
                        0,
                        0,
                        LARGE_PULLEY_R,
                        s * y,
                        s * (y + PULLEY_W),
                        0.012,
                    )
                )
                y += PULLEY_W + CLEAR
                parts.append(
                    cyl(
                        f"{joint}_motor_{s}",
                        -c,
                        0,
                        MOTOR_R,
                        s * y,
                        s * (y + MOTOR_W),
                        per_joint - 0.012,
                    )
                )
                y += MOTOR_W + CLEAR
            else:
                parts.append(
                    cyl(
                        f"{joint}_motor_{s}",
                        -c,
                        0,
                        MOTOR_R,
                        s * y,
                        s * (y + MOTOR_W),
                        per_joint - 0.012,
                    )
                )
                y += MOTOR_W + CLEAR
                parts.append(
                    cyl(
                        f"{joint}_small_pulley_{s}",
                        -c,
                        0,
                        SMALL_PULLEY_R,
                        s * y,
                        s * (y + PULLEY_W),
                    )
                )
                parts.append(
                    cyl(
                        f"{joint}_large_pulley_{s}",
                        0,
                        0,
                        LARGE_PULLEY_R,
                        s * y,
                        s * (y + PULLEY_W),
                        0.012,
                    )
                )
                y += PULLEY_W + CLEAR
    inner = interior(body)
    top, bottom = inner[2][1], inner[2][0]
    front = inner[0][1]
    # Battery stood on end in front of the hip pulleys.
    bx = front - BATTERY[0] / 2 - CLEAR
    parts.append(box("battery", (bx, 0, 0), BATTERY, body["battery_kg"]))
    each = body["electronics_kg"] / 7
    # Drivers: two above and two below the motor row, one each side of the battery.
    for s in (1, -1):
        for zsign, z_face in ((1, top), (-1, bottom)):
            zc = z_face - zsign * (MOTEUS_FLAT[2] / 2 + CLEAR)
            parts.append(
                box(f"moteus_{zsign:+d}_{s}", (-c - 0.005, s * 0.020, zc), MOTEUS_FLAT, each)
            )
        y_side = BATTERY[1] / 2 + CLEAR + MOTEUS[1] / 2
        parts.append(box(f"moteus_front_{s}", (bx - 0.003, s * y_side, 0), MOTEUS, each))
    host = (0.030, 0.010, 0.040)
    host_y = BULKHEAD / 2 + PULLEY_W + MOTOR_W + 3 * CLEAR + host[1] / 2
    parts.append(box("host_power_imu", (0.004, host_y, 0.0), host, each))
    return parts


def option_coaxial(body):
    """B: each joint a motor plus 4:1 planetary on the hip axis, two per side."""
    parts = []
    per_joint = body["hip_module_kg"] / 4
    for s in (1, -1):
        y = BULKHEAD / 2 + CLEAR
        for joint in ("knee", "thigh"):
            w = MOTOR_W + PLANETARY_W + BEARING_W
            parts.append(
                cyl(f"{joint}_motor_gear_{s}", 0, 0, MOTOR_R, s * y, s * (y + w), per_joint)
            )
            y += w + CLEAR
    inner = interior(body)
    rear = inner[0][0]
    bx = rear + BATTERY[0] / 2 + CLEAR
    parts.append(box("battery", (bx, 0, 0), BATTERY, body["battery_kg"]))
    each = body["electronics_kg"] / 7
    top, bottom = inner[2][1], inner[2][0]
    for s in (1, -1):
        y_side = BATTERY[1] / 2 + CLEAR + MOTEUS[1] / 2
        parts.append(box(f"moteus_rear_{s}", (bx + 0.003, s * y_side, 0), MOTEUS, each))
        # Two flat drivers above and two below the hip drive stack.
        for zsign, z_face in ((1, top), (-1, bottom)):
            zc = z_face - zsign * (MOTEUS_FLAT[2] / 2 + CLEAR)
            parts.append(box(f"moteus_{zsign:+d}_{s}", (0.0, s * 0.020, zc), MOTEUS_FLAT, each))
    host = (HOST[1], HOST[0], HOST[2])
    parts.append(box("host_power_imu", (inner[0][1] - host[0] / 2 - CLEAR, 0, 0), host, each))
    return parts


def check(parts, body, pod_depth):
    inner = interior(body)
    clashes = [
        [a["name"], b["name"]] for i, a in enumerate(parts) for b in parts[i + 1 :] if overlap(a, b)
    ]
    poke = {}
    for p in parts:
        hip_part = p["kind"] == "cyl" and abs(p["x"]) < 1e-9
        d = outside(p, inner, allow_y=pod_depth if hip_part else 0.0)
        if d > 1e-6:
            poke[p["name"]] = round(d * 1000, 1)
    hip_outer = max(
        abs(y) for p in parts if p["kind"] == "cyl" and abs(p["x"]) < 1e-9 for y in p["y"]
    )
    return clashes, poke, hip_outer


def balanced_pose(body_parts, c):
    """Concept pose with the study's body mass positions replacing the concept layout."""
    original = concept.masses

    def masses(cfg):
        body, legs = original(cfg)
        replaced = ("battery", "electronics", "hip_modules", "motor_row", "hip_outputs")
        keep = [m for m in body if m[0] not in replaced]
        extra = [(p["name"], p["mass"], *centre(p)) for p in body_parts if p["mass"] > 0]
        return keep + extra, legs

    concept.masses = masses
    try:
        state = concept.pose(c["poses"]["nominal_hip_height"], c)
        torques = concept.static_torques(state, c)
    finally:
        concept.masses = original
    return state, torques


def centre(p):
    if p["kind"] == "cyl":
        return p["x"], p["z"]
    return p["c"][0], p["c"][2]


def width_with(c, hip_outer, pod):
    """Overall width when the thigh starts outboard of both the hip pod and the drive stack."""
    leg, wheel = c["leg"], c["wheel"]
    thigh_y0 = max(hip_outer + 0.0015, c["body"]["width"] / 2 + pod)
    shift = thigh_y0 - leg["thigh_y"][0]
    return 2 * (wheel["y_center"] + shift + wheel["width"] / 2), shift


def study():
    c = concept.load()
    budget = c["mass_budget_kg"]
    # The study started from the B1 body (130 x 92 x 80); option A's growth is its result.
    base = dict(c["body"], length=0.130, width=0.092, wall=0.002)
    base.update(
        hip_module_kg=budget["hip_modules_four_RO40_with_reductions_bearings"],
        battery_kg=budget["battery_6S_850"],
        electronics_kg=budget["electronics_six_moteus_c1_carrier_power"],
    )
    options = {
        "A_rear_motor_row": (option_rear_row, dict(base, length=0.145, width=0.102)),
        "B_coaxial_planetary": (option_coaxial, dict(base)),
    }
    report = {
        "assumptions": {
            "motor_module_m": [2 * MOTOR_R, MOTOR_W],
            "pulleys_m": {"small_r": SMALL_PULLEY_R, "large_r": LARGE_PULLEY_R, "width": PULLEY_W},
            "planetary_width_m": PLANETARY_W,
            "battery_m": BATTERY,
            "moteus_m": MOTEUS_FLAT,
            "host_m": HOST,
            "hip_pod_proud_of_body_m": c["leg"]["hip_housing_y"][1] - c["body"]["width"] / 2,
        }
    }
    figures = {}
    for key, (build, body) in options.items():
        parts = build(body)
        # Hip parts may enter the pod: its protrusion beyond the body plus the wall.
        pod = c["leg"]["hip_housing_y"][1] - c["body"]["width"] / 2 + 0.002
        clashes, poke, hip_outer = check(parts, body, pod)
        cfg = copy.deepcopy(c)
        cfg["body"].update({k: body[k] for k in ("length", "width")})
        pod_proud = c["leg"]["hip_housing_y"][1] - c["body"]["width"] / 2
        width, shift = width_with(cfg, hip_outer, pod_proud)
        cfg["leg"]["thigh_y"] = [y + shift for y in cfg["leg"]["thigh_y"]]
        state, torques = balanced_pose(parts, cfg)
        report[key] = {
            "body_lwh_m": [body["length"], body["width"], body["height"]],
            "hip_drive_outer_y_m": hip_outer,
            "thigh_shift_outboard_m": shift,
            "overall_width_m": width,
            "clashes": clashes,
            "outside_interior_mm": poke,
            "nominal_thigh_deg_below_horizontal_back": 180 + state["a1_deg"],
            "nominal_axle_x_m": state["axle_m"][0],
            "static_drive_Nm": torques,
        }
        figures[key] = (parts, body)
    return report, figures


def draw(figures, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

    colours = {
        "motor": "#6c8ebf",
        "pulley": "#d79b00",
        "battery": "#82b366",
        "moteus": "#b85450",
        "host": "#9673a6",
    }

    def colour(name):
        return next((v for k, v in colours.items() if k in name), "#999999")

    fig, axes = plt.subplots(2, len(figures), figsize=(6.5 * len(figures), 9))
    for col, (key, (parts, body)) in enumerate(figures.items()):
        inner = interior(body)
        for row, (i, j, label) in enumerate(
            ((0, 2, "side section (x-z)"), (0, 1, "top section (x-y)"))
        ):
            ax = axes[row][col]
            (a0, a1), (b0, b1) = inner[i], inner[j]
            ax.add_patch(
                FancyBboxPatch(
                    (a0, b0),
                    a1 - a0,
                    b1 - b0,
                    boxstyle="round,pad=0,rounding_size=0.01",
                    fill=False,
                    lw=1.5,
                    ec="k",
                )
            )
            for p in sorted(parts, key=lambda q: "pulley" in q["name"]):
                e = extent(p)
                if p["kind"] == "cyl" and j == 2:
                    ax.add_patch(
                        Circle(
                            (p["x"], p["z"]),
                            p["r"],
                            fc=colour(p["name"]),
                            ec="k",
                            alpha=0.45,
                            lw=0.6,
                        )
                    )
                else:
                    ax.add_patch(
                        Rectangle(
                            (e[i][0], e[j][0]),
                            e[i][1] - e[i][0],
                            e[j][1] - e[j][0],
                            fc=colour(p["name"]),
                            ec="k",
                            alpha=0.45,
                            lw=0.6,
                        )
                    )
            ax.plot(0, 0, "k+", ms=12)
            ax.set_aspect("equal")
            ax.set_xlim(a0 - 0.012, a1 + 0.012)
            ax.set_ylim(b0 - 0.03, b1 + 0.03)
            ax.set_title(f"{key.replace('_', ' ')}: {label}", fontsize=10)
            ax.tick_params(labelsize=7)
    handles = [Rectangle((0, 0), 1, 1, fc=v, alpha=0.45, ec="k") for v in colours.values()]
    fig.legend(handles, list(colours), loc="lower center", ncol=5)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, dpi=110)


def main():
    report, figures = study()
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "study.json").write_text(
        json.dumps(report, indent=2, default=lambda v: np.asarray(v).tolist()) + "\n"
    )
    draw(figures, OUT / "sections.png")
    print(json.dumps({k: v for k, v in report.items() if k != "assumptions"}, indent=2))


if __name__ == "__main__":
    main()
