"""M10 phase D4 protection look: the exterior render mesh plus the candidate bumpers.

.venv/bin/python cad/m10/protection_mesh.py [--candidate protection_v2]

Adds the frame-mounted skid pad under the tub and the foam ring on each wheel's outer
face, built from the same shapes the fall plant uses (bipe.m10.bumpers.hull_vertices),
to the B1 exterior render mesh in every pose. Writes
artifacts/m10/protection/render-mesh.json.gz for cad/m10/render_blender.py
(--arg stage=protection). The pads are placed from the exterior meshes themselves:
the skid is centred under the body tub, and each ring is coaxial with its tire and
backed on the tire's outer face.
"""

import argparse
import gzip
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from bipe.m10.bumpers import RING_ANGULAR, hull_vertices
from bipe.m10.falls import candidate_config
from bipe.m10.plant import _bumper_pads, parameters

SOURCE = ROOT / "artifacts/m10/exterior/render-mesh.json.gz"
OUT = ROOT / "artifacts/m10/protection/render-mesh.json.gz"
FOAM_RGB = [0.16, 0.17, 0.19]


def hull_mesh(vertices):
    """Outward-wound triangles of the convex hull, in its own vertex numbering."""
    hull = ConvexHull(vertices)
    used = np.unique(hull.simplices)
    index = {v: i for i, v in enumerate(used)}
    triangles = []
    for tri, eq in zip(hull.simplices, hull.equations, strict=True):
        a, b, c = (vertices[i] for i in tri)
        if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
            tri = tri[[0, 2, 1]]
        triangles.append([index[i] for i in tri])
    return vertices[used], triangles


def ring_mesh(pad, segments=RING_ANGULAR):
    """The ring as the part looks: the foam annulus revolved from its profile (the plant
    collides with its convex hull, which also fills the hub)."""
    inner, outer, t = pad["inner_radius_m"], pad["outer_radius_m"], pad["thickness_m"]
    rim = t - (outer - inner) * math.tan(math.radians(pad.get("dome_deg", 0.0)))
    # Closed profile (r, z), base on the wheel face at z = -t/2.
    profile = [(inner, -t / 2), (outer, -t / 2), (outer, -t / 2 + rim), (inner, t / 2)]
    angles = np.arange(segments) * 2 * math.pi / segments
    vertices = np.array([[r * math.cos(a), r * math.sin(a), z] for a in angles for r, z in profile])
    n = len(profile)
    triangles = []
    for i in range(segments):
        j = (i + 1) % segments
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            c, d = j * n + (k + 1) % n, j * n + k
            triangles += [[a, b, c], [a, c, d]]
    return vertices, triangles


def pad_mesh(pad):
    if pad["shape"] == "ring":
        return ring_mesh(pad)
    return hull_mesh(hull_vertices(pad))


def placement(pad, comps):
    """Rotation and centre of a pad's frame in the CAD frame of one pose."""
    by_name = {}
    for c in comps:
        by_name.setdefault(c["name"], []).append(np.array(c["vertices"]))
    if pad["shape"] == "box":
        body = by_name["BodyShell"][0]
        lo, hi = body.min(axis=0), body.max(axis=0)
        lz = pad["size_m"][2]
        return np.eye(3), np.array([(lo[0] + hi[0]) / 2, 0.0, lo[2] - lz / 2])
    side = pad["name"].split("_")[0]
    tire = by_name[f"{side}_Tire"][0]
    lo, hi = tire.min(axis=0), tire.max(axis=0)
    sign = 1 if side == "left" else -1
    face_y = hi[1] if sign > 0 else lo[1]
    centre = np.array(
        [(lo[0] + hi[0]) / 2, face_y + sign * pad["thickness_m"] / 2, (lo[2] + hi[2]) / 2]
    )
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, np.array(pad["frame"][1]))
    return rot.reshape(3, 3), centre


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", default="protection_v2")
    args = parser.parse_args()
    shared = candidate_config(json.loads((ROOT / "config/robot.json").read_text()), args.candidate)
    pads = _bumper_pads(parameters(shared))
    with gzip.open(SOURCE) as f:
        data = json.load(f)
    data["colors"]["foam"] = FOAM_RGB
    for comps in data["poses"].values():
        for pad in pads:
            local, triangles = pad_mesh(pad)
            rot, centre = placement(pad, comps)
            points = local @ rot.T + centre
            comps.append(
                {
                    "name": pad["name"],
                    "link": pad["body"],
                    "color": FOAM_RGB,
                    "color_key": "foam",
                    "vertices": points.tolist(),
                    "triangles": triangles,
                }
            )
    data["candidate"] = args.candidate
    data["status"] = (
        "Phase D4 look study: B1 exterior plus candidate bumpers; not adopted into the CAD"
    )
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(OUT, "wt") as f:
        json.dump(data, f)
    print("M10 PROTECTION MESH", OUT.relative_to(ROOT), [p["name"] for p in pads], flush=True)


if __name__ == "__main__":
    main()
