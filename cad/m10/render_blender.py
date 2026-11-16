"""M10 look renders in Blender from the FreeCAD per-part meshes.

python3 tools/blender_bridge.py cad/m10/render_blender.py [--arg stage=exterior]
    [--arg views=<json file>] [--arg out=<folder>] [--arg resolution=1920x1080]

A mesh file that carries a "views" list (look studies such as knee_stop/<option>), or a
views JSON file given with --arg views, replaces the stage's fixed camera set. Each view:
name, pose, direction, target ([x, y, z] m) or target_z, lens (mm), optional size (framed
subject, m; default 0.3), floor (default true; false adds a light from below) and hide
(part names left out of the render).

Runs inside the user's Blender (Blender Lab MCP add-on). Builds a separate
scene "M10Render" so the user's open file is not touched, then renders the
views below to artifacts/m10/<stage>/renders/. Meshes are in metres.
"""

import gzip
import json
import math
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2]
ARGS = globals().get("BRIDGE_ARGS", {})
STAGE = ARGS.get("stage", "concept")
MESH = ROOT / f"artifacts/m10/{STAGE}/render-mesh.json.gz"
OUT = ROOT / ARGS["out"] if "out" in ARGS else ROOT / f"artifacts/m10/{STAGE}/renders"
SCENE = "M10Render"

# Material look per colour key: (roughness, emission strength).
FINISH = {
    "shell": (0.45, 0.0),
    "shell_dark": (0.5, 0.0),
    "tire": (0.85, 0.0),
    "hub": (0.4, 0.0),
    "eye": (0.3, 3.0),
    "lens": (0.15, 0.0),
    "ear": (0.5, 0.0),
    "panel": (0.5, 0.0),
    "bolt": (0.35, 0.0),
    "camera": (0.6, 0.0),
    "motor": (0.4, 0.0),
    "metal": (0.3, 0.0),
    "belt": (0.8, 0.0),
    "bearing": (0.3, 0.0),
    "frame": (0.5, 0.0),
    "battery": (0.6, 0.0),
    "pcb": (0.5, 0.0),
    "harness": (0.5, 0.0),
    "foam": (0.9, 0.0),
    "foam_skin": (0.75, 0.0),
    "stop_pad": (0.7, 0.0),
    "skid_foam": (0.9, 0.0),
    "skid_skin": (0.75, 0.0),
}

# (name, pose, camera direction from the target, target height m, lens mm)
VIEWS = [
    ("three_quarter", "nominal", (1.0, 0.85, 0.32), 0.10, 70),
    ("side", "nominal", (0.0, 1.0, 0.08), 0.11, 70),
    ("rear_three_quarter", "nominal", (-0.9, 0.9, 0.35), 0.10, 70),
    ("crouched", "crouched", (1.0, 0.85, 0.32), 0.09, 70),
    ("extended", "extended", (1.0, 0.85, 0.32), 0.12, 70),
]
# Assembly stage only: the same camera set with the outer shells hidden on the
# camera (+y) side so the drive, power and head internals show.
CUTAWAY_VIEWS = [
    ("cutaway_three_quarter", "nominal", (1.0, 0.85, 0.32), 0.10, 70),
    ("cutaway_side", "nominal", (0.0, 1.0, 0.08), 0.11, 70),
    ("cutaway_rear_three_quarter", "nominal", (-0.9, 0.9, 0.35), 0.10, 70),
]
# Protection and assembly stages (phase D4 bumpers): the skid from near the floor and the
# wheel ring face-on.
PROTECTION_VIEWS = [
    ("low_side", "nominal", (0.3, 1.0, 0.04), 0.06, 70),
    ("wheel_face", "nominal", (0.15, 1.0, 0.25), 0.05, 100),
]
# Assembly stage only: straight-on front views, where the thigh, shin and wheel planes
# show side by side (in side views the wheel overlaps the thigh in projection only).
FRONT_VIEWS = [
    ("front_nominal", "nominal", (1.0, 0.0, 0.05), 0.10, 70),
    ("front_crouched", "crouched", (1.0, 0.0, 0.05), 0.08, 70),
]
CUTAWAY_HIDE = {
    "BodyShell",
    "BodyCover",
    "BodyTub",
    "BatteryHatch",
    "HeadPod",
    "HeadFront",
    "HeadRear",
    "Neck",
    "left_Thigh",
    "left_ThighLid",
    "left_ThighPanel",
    "left_HipCap",
    "left_HipHousing",
}


def linear(rgb):
    """Config colours are display (sRGB) values; Blender shader inputs are linear."""
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb)


def material(key, rgb):
    name = f"M10_{key}"
    rgb = linear(rgb)
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    rough, glow = FINISH[key]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    if glow:
        bsdf.inputs["Emission Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Emission Strength"].default_value = glow
    return mat


def build_scene():
    old = bpy.data.scenes.get(SCENE)
    if old:
        for obj in list(old.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.scenes.remove(old)
    for block in (bpy.data.meshes, bpy.data.cameras, bpy.data.lights):
        for item in [b for b in block if b.name.startswith("M10_") and b.users == 0]:
            block.remove(item)
    scene = bpy.data.scenes.new(SCENE)
    scene.render.engine = "BLENDER_EEVEE"
    size = ARGS.get("resolution", "1600x1200")  # e.g. 1920x1080 for the demo title cards
    scene.render.resolution_x, scene.render.resolution_y = (int(v) for v in size.split("x"))
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"
    world = bpy.data.worlds.get("M10_World") or bpy.data.worlds.new("M10_World")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.92, 0.92, 0.93, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.35
    scene.world = world

    floor = bpy.data.meshes.new("M10_floor")
    s = 3.0
    floor.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
    obj = bpy.data.objects.new("M10_floor", floor)
    fm = bpy.data.materials.get("M10_floor") or bpy.data.materials.new("M10_floor")
    fm.use_nodes = True
    fm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.8, 0.8, 0.8, 1)
    fm.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.9
    floor.materials.append(fm)
    scene.collection.objects.link(obj)

    for name, energy, size, loc in [
        ("key", 14.0, 0.6, (0.6, 0.5, 0.9)),
        ("fill", 4.0, 0.8, (0.3, -0.8, 0.5)),
        ("rim", 8.0, 0.5, (-0.8, 0.2, 0.7)),
    ]:
        light = bpy.data.lights.new(f"M10_{name}", "AREA")
        light.energy, light.size = energy, size
        lo = bpy.data.objects.new(f"M10_{name}", light)
        lo.location = loc
        lo.rotation_euler = (Vector((0, 0, 0.1)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        scene.collection.objects.link(lo)

    # Light from below for views without the floor (undersides).
    light = bpy.data.lights.new("M10_under", "AREA")
    light.energy, light.size = 10.0, 0.8
    lo = bpy.data.objects.new("M10_under", light)
    lo.location = (0.3, 0.4, -0.7)
    lo.rotation_euler = (
        (Vector((0, 0, 0.05)) - Vector(lo.location)).to_track_quat("-Z", "Y").to_euler()
    )
    lo.hide_render = True
    scene.collection.objects.link(lo)

    cam = bpy.data.objects.new("M10_camera", bpy.data.cameras.new("M10_camera"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    return scene


def load_pose(scene, comps, colors):
    for obj in [o for o in scene.objects if o.get("m10_part")]:
        bpy.data.objects.remove(obj, do_unlink=True)
    for i, comp in enumerate(comps):
        key = comp.get("color_key") or next(
            k for k, v in colors.items() if tuple(v) == tuple(comp["color"])
        )
        mesh = bpy.data.meshes.new(f"M10_{comp['name']}_{i}")
        mesh.from_pydata(comp["vertices"], [], comp["triangles"])
        mesh.validate()
        mesh.shade_smooth()
        # Keep B-rep edges between faces meeting at a sharp angle crisp (Blender 4.1+).
        if hasattr(mesh, "set_sharp_from_angle"):
            mesh.set_sharp_from_angle(angle=math.radians(35))
        mesh.materials.append(material(key, comp["color"]))
        obj = bpy.data.objects.new(f"M10_{comp['name']}_{i}", mesh)
        obj["m10_part"] = True
        obj["m10_name"] = comp["name"]
        scene.collection.objects.link(obj)


def aim(scene, direction, target, lens, size=0.3):
    cam = scene.camera
    cam.data.lens = lens
    target = Vector(target) if isinstance(target, (list, tuple)) else Vector((0.0, 0.0, target))
    d = Vector(direction).normalized()
    # Distance that frames a subject of this size (m) with this lens on a 36 mm sensor.
    distance = size * lens / 36 * 1.5
    cam.location = target + d * distance
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()


def render():
    with gzip.open(MESH) as f:
        data = json.load(f)
    colors = data["colors"]
    scene = build_scene()
    OUT.mkdir(parents=True, exist_ok=True)
    window = bpy.context.window_manager.windows[0]
    written = []
    loaded = None
    views = [
        {"name": n, "pose": p, "direction": d, "target_z": z, "lens": lens}
        for n, p, d, z, lens in (
            VIEWS
            + (CUTAWAY_VIEWS + PROTECTION_VIEWS + FRONT_VIEWS if STAGE == "assembly" else [])
            + (PROTECTION_VIEWS if STAGE == "protection" else [])
        )
    ]
    views = data.get("views", views)
    if "views" in ARGS:
        views = json.loads((ROOT / ARGS["views"]).read_text())
    for view in views:
        name, pose = view["name"], view["pose"]
        if pose != loaded:
            load_pose(scene, data["poses"][pose], colors)
            loaded = pose
        hide = set(view.get("hide", [])) | (CUTAWAY_HIDE if name.startswith("cutaway") else set())
        for obj in [o for o in scene.objects if o.get("m10_part")]:
            obj.hide_render = obj["m10_name"] in hide
        floor = view.get("floor", True)
        scene.objects["M10_floor"].hide_render = not floor
        scene.objects["M10_under"].hide_render = floor
        aim(
            scene,
            view["direction"],
            view.get("target", view.get("target_z")),
            view["lens"],
            view.get("size", 0.3),
        )
        scene.render.filepath = str(OUT / f"{name}.png")
        with bpy.context.temp_override(window=window, scene=scene):
            bpy.ops.render.render(write_still=True, scene=scene.name)
        written.append(scene.render.filepath)
    return written


result = {"written": render(), "fov_deg": math.degrees(2 * math.atan(18 / 70))}
