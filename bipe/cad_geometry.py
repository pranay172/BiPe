"""CAD visual meshes and conservative structural ground-contact envelopes."""

import json
import math
import xml.etree.ElementTree as ET

from .mass_properties import body_transform
from .model import values
from .parameters import ROOT, leg_geometry


def add_cad_geometry(root, p, fixed=False):
    from .kinematics import posture

    source = ROOT / p["mass_properties_source"]["path"]
    report = json.loads(source.read_text())
    base = root.find("worldbody/body")
    bodies = {b.get("name"): b for b in base.iter("body")}
    for geom in base.iter("geom"):
        geom.set("rgba", "0 0 0 0")
    asset = root.find("asset")
    q = posture(p["geometry"]["fixed_leg_angle_deg"])
    hip_z = leg_geometry(p)["hip_z"] if fixed else 0
    for parent, rows in report["components"].items():
        for row in rows:
            name = row["name"]
            path = source.parent / "meshes" / (name + ".stl")
            ET.SubElement(asset, "mesh", name="cad_" + name, file=str(path), scale=".001 .001 .001")
            body = bodies[parent] if not fixed or parent.endswith("_wheel") else base
            attrs = {
                "name": "cad_" + name,
                "type": "mesh",
                "mesh": "cad_" + name,
                "mass": "0",
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
            }
            if fixed and not parent.endswith("_wheel"):
                offset, rot = body_transform(p, parent, q)
                offset[2] += hip_z
                angle = math.atan2(rot[0, 2], rot[0, 0])
                attrs.update(pos=values(offset), euler=values([0, angle, 0]))
            color = ".65 .7 .75 1"
            if "Shell" in name:
                color = ".84 .89 .92 1"
            elif "Eye" in name or "Tire" in name or "module" in name:
                color = ".08 .12 .16 1"
            elif "plate" in name or "hub" in name:
                color = ".12 .37 .57 1"
            elif name == "Battery_pack":
                color = ".22 .36 .24 1"
            attrs["rgba"] = color
            ET.SubElement(body, "geom", **attrs)
    # Ground-only boxes bound the fork; exact self-interference is checked in CAD.
    c, g = p["mechanics"], p["geometry"]
    for sign, side in [(1, "left"), (-1, "right")]:
        body = base if fixed else bodies[side + "_lower"]
        start = c["lower_plate_y"] + c["plate_thickness"]
        outer_y = c["bearing_y"]
        top, thick, width = c["fork_top_above_axle"], c["fork_top_thickness"], c["fork_width"]
        boxes = [
            (
                [width, outer_y + 0.007 - start, thick],
                [0, sign * (start + outer_y + 0.007) / 2, -g["link_length"] + top + thick / 2],
            ),
            (
                [width, 0.007, top + c["fork_outer_radius"]],
                [
                    0,
                    sign * (outer_y + 0.0035),
                    -g["link_length"] + (top - c["fork_outer_radius"]) / 2,
                ],
            ),
            (
                [width, c["fork_mount_thickness"], c["fork_mount_height"]],
                [0, sign * (start + c["fork_mount_thickness"] / 2), -g["link_length"] + top],
            ),
        ]
        for i, (size, center) in enumerate(boxes):
            angle = 0
            if fixed:
                import numpy as np

                offset, rot = body_transform(p, side + "_lower", q)
                center = offset + rot @ np.asarray(center)
                center[2] += hip_z
                angle = math.atan2(rot[0, 2], rot[0, 0])
            ET.SubElement(
                body,
                "geom",
                name=f"{side}_fork_contact_{i}",
                type="box",
                pos=values(center),
                euler=values([0, angle, 0]),
                size=values([s / 2 for s in size]),
                mass="0",
                contype="2",
                conaffinity="1",
                rgba="0 0 0 0",
                group="3",
            )
