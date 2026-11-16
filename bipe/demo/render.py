"""Render a recorded demo scene to video with MuJoCo's OpenGL renderer.

The render model is a visual copy of the M10 plant: the collision hulls are hidden and
the CAD render meshes (artifacts/m10/assembly/render-mesh.json.gz, the same tessellation
the Blender stills use) ride on the plant's bodies. The recorded qpos is replayed with
forward kinematics only; nothing is simulated here.

    MUJOCO_GL=egl python -m bipe.demo render local/demo/push_recovery_classical.npz
"""

from __future__ import annotations

import gzip
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from ..m10 import plant
from ..m10.plant import MESH, rotation
from ..parameters import ROOT, load_parameters

# Parts inside the shells: skipped, they are never seen and cost render time.
HIDDEN = (
    "Battery", "BEC", "Bulkhead", "CameraBoard", "CarrierPCB", "CarrierParts", "Moteus",
    "PiZero", "PowerPCB", "PowerParts", "Teensy", "EyeLEDBoard", "Bearing", "Harness",
    "MotorStandoff", "Shaft", "Pulley", "Belt", "Encoder", "Stator", "Rotor", "Sleeve",
    "Flange", "StopPin", "KneeMotor", "ThighMotor", "Sensor",
)  # fmt: skip
ROLES = ("thigh", "shin", "wheel")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def _visible(name):
    return not any(h in name for h in HIDDEN)


def cad_geoms(root):
    """Add the CAD render meshes as visual-only geoms on the plant's bodies."""
    data = json.loads(gzip.decompress(MESH.read_bytes()))
    pose = json.loads((ROOT / "artifacts/m10/cad/mass.json").read_text())["pose"]
    a1, a2 = math.radians(pose["a1_deg"]), math.radians(pose["a2_deg"])
    knee, axle = pose["knee_m"], pose["axle_m"]
    # As plant._exterior_meshes: CAD world frame to the hip-relative link frames.
    hip = np.array([-axle[0], 0.0, pose["hip_height_m"]])
    frames = {
        "body": (np.zeros(3), np.eye(3)),
        "thigh": (np.zeros(3), rotation(a1)),
        "shin": (np.array([knee[0], 0, knee[1]]), rotation(a2)),
        "wheel": (np.array([axle[0], 0, axle[1]]), np.eye(3)),
    }
    asset = root.find("asset")
    bodies = {b.get("name"): b for b in root.iter("body")}
    for color_key, rgb in data["colors"].items():
        ET.SubElement(
            asset, "material", name=f"cad_{color_key}", rgba=plant.values(list(rgb) + [1]),
            specular="0.25", shininess="0.35",
        )  # fmt: skip
    count = 0
    for k, comp in enumerate(data["poses"]["nominal"]):
        link = comp["link"]
        role = link.split("_", 1)[1] if link.startswith(("left_", "right_")) else link
        if role not in frames or not _visible(comp["name"]):
            continue
        origin, rot = frames[role]
        vertices = (np.asarray(comp["vertices"]) - hip - origin) @ rot
        faces = np.asarray(comp["triangles"]).reshape(-1, 3)
        mesh = f"cad{k}_{comp['name']}"
        ET.SubElement(
            asset, "mesh", name=mesh, vertex=plant.values(vertices.ravel()),
            face=" ".join(str(int(v)) for v in faces.ravel()),
        )  # fmt: skip
        body = bodies["chassis" if link == "body" else link]
        ET.SubElement(
            body, "geom", type="mesh", mesh=mesh, material=f"cad_{comp['color_key']}",
            contype="0", conaffinity="0", group="2", mass="0",
        )  # fmt: skip
        count += 1
    return count


def render_xml(p, size=(1280, 720), base_xml=None):
    """The plant XML dressed for video: CAD meshes, studio floor, sky and shadows."""
    root = ET.fromstring(base_xml or plant.generate_xml(p))
    vis = root.find("visual")
    vis.find("global").set("offwidth", str(size[0]))
    vis.find("global").set("offheight", str(size[1]))
    ET.SubElement(vis, "quality", shadowsize="4096", offsamples="8")
    ET.SubElement(vis, "headlight", ambient=".25 .25 .27", diffuse=".35 .35 .35", specular="0 0 0")
    ET.SubElement(vis, "map", znear="0.01", zfar="60", shadowclip="2.5", fogstart="1.5", fogend="5")
    ET.SubElement(vis, "rgba", fog=".80 .82 .86 1", haze=".85 .87 .90 1")
    # Map distances are in units of the extent: 1 m, so znear 1 cm and fog from 1.5 m.
    ET.SubElement(root, "statistic", extent="1", center="0 0 0.1")
    asset = root.find("asset")
    if root.find("asset/texture[@type='skybox']") is None:
        ET.SubElement(
            asset, "texture", type="skybox", builtin="gradient", rgb1=".93 .94 .96",
            rgb2=".62 .66 .72", width="512", height="512",
        )  # fmt: skip
    ET.SubElement(
        asset, "texture", name="demo_floor", type="2d", builtin="checker", rgb1=".70 .71 .73",
        rgb2=".63 .64 .67", width="512", height="512", mark="edge", markrgb=".56 .57 .60",
    )  # fmt: skip
    ET.SubElement(
        asset, "material", name="demo_floor", texture="demo_floor", texrepeat="8 8",
        texuniform="true", reflectance="0.0",
    )  # fmt: skip
    ground = root.find(".//geom[@name='ground']")
    ground.set("material", "demo_floor")
    ground.set("size", "40 40 .1")
    ground.attrib.pop("rgba", None)
    # The plant's own geoms (collision hulls, tire cylinders) go to group 3, not drawn.
    for geom in root.find("worldbody").iter("geom"):
        if geom.get("name") != "ground" and geom.get("group") is None:
            geom.set("group", "3")
    world = root.find("worldbody")
    for light in list(world.findall("light")):
        world.remove(light)
    ET.SubElement(
        world, "light", name="key", mode="trackcom", target="chassis", pos="1.2 -1.0 2.2",
        dir="-0.45 0.4 -1", diffuse=".62 .61 .59", specular=".15 .15 .15", castshadow="true",
    )  # fmt: skip
    ET.SubElement(
        world, "light", name="fill", directional="true", dir="0.3 0.6 -0.6",
        diffuse=".25 .26 .3", specular="0 0 0", castshadow="false",
    )  # fmt: skip
    cad_geoms(root)
    return ET.tostring(root, encoding="unicode")


class Camera:
    """Smoothed camera moves around the chassis."""

    def __init__(self, spec, fps):
        self.spec, self.fps = spec, fps
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.lookat = None
        self.yaw = None

    def update(self, k, d, chassis):
        s = self.spec
        target = d.xpos[chassis].copy()
        target[2] = max(target[2] + s.get("lift", 0.0), 0.05)
        heading = _yaw(d.xquat[chassis])
        a = 1 - math.exp(-1 / (s.get("lag_s", 0.12) * self.fps))
        if self.lookat is None:
            self.lookat, self.yaw = target, heading
        else:
            self.lookat = self.lookat + a * (target - self.lookat)
            self.yaw = self.yaw + a * _wrap(heading - self.yaw)
        move = s.get("move", "fixed")
        azimuth = s.get("azimuth", 90.0)
        if move == "orbit":
            azimuth += s.get("degrees", 90) * k / max(1, self.frames - 1)
        elif move == "chase":
            azimuth += math.degrees(self.yaw)
        if move == "fixed" and s.get("anchor", True):
            look = self.lookat if self.fixed is None else self.fixed
            self.fixed = look
        self.cam.lookat[:] = self.lookat
        self.cam.distance = s.get("distance", 1.0)
        self.cam.elevation = s.get("elevation", -10.0)
        self.cam.azimuth = azimuth
        return self.cam

    frames = 1
    fixed = None


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _arrow(scene, start, end, rgba=(0.95, 0.35, 0.1, 1.0), width=0.012):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.asarray(rgba, np.float32))  # fmt: skip
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(start, float),
                         np.asarray(end, float))  # fmt: skip
    scene.ngeom += 1


class Overlay:
    """Captions burnt into the frame with Pillow."""

    def __init__(self, size):
        from PIL import ImageFont

        self.size = size
        scale = size[1] / 720
        self.big = ImageFont.truetype(FONT, int(40 * scale))
        self.small = ImageFont.truetype(FONT, int(22 * scale))
        self.pad = int(28 * scale)

    def draw(self, image, title=None, title_alpha=0.0, footer=None, badge=None):
        from PIL import Image, ImageDraw

        img = Image.fromarray(image).convert("RGBA")
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
        g = ImageDraw.Draw(layer)
        w, h = img.size
        if title and title_alpha > 0:
            a = int(255 * title_alpha)
            g.text((self.pad, self.pad), title, font=self.big, fill=(25, 28, 34, a))
        if footer:
            g.text((self.pad, h - self.pad - self.small.size), footer, font=self.small,
                   fill=(40, 44, 52, 230))  # fmt: skip
        if badge:
            box = g.textbbox((0, 0), badge, font=self.small)
            bw = box[2] - box[0]
            x, y = w - self.pad - bw, self.pad
            g.rounded_rectangle((x - 12, y - 6, x + bw + 12, y + self.small.size + 12),
                                radius=8, fill=(232, 90, 30, 220))  # fmt: skip
            g.text((x, y), badge, font=self.small, fill=(255, 255, 255, 255))
        return np.asarray(Image.alpha_composite(img, layer).convert("RGB"))


def render(npz, output=None, size=(1280, 720), footer=None, camera=None, preview_every=0):
    """Render a recorded scene to an mp4; returns the output path."""
    import imageio.v2 as imageio

    rec = np.load(npz)
    meta = json.loads(str(rec["meta"]))
    scene = meta["scene_def"]
    fps = int(meta["fps"])
    p = load_parameters(profile="m10")
    m = mujoco.MjModel.from_xml_string(render_xml(p, size))
    d = mujoco.MjData(m)
    if scene.get("slope_deg"):
        slope = math.radians(scene["slope_deg"])
        ground = m.geom("ground").id
        # As plant.reset: clear the compiler's body-aligned flag before tilting the plane.
        m.geom_sameframe[ground] = mujoco.mjtSameFrame.mjSAMEFRAME_NONE
        m.geom_quat[ground] = [math.cos(slope / 2), 0, -math.sin(slope / 2), 0]
    renderer = mujoco.Renderer(m, size[1], size[0])
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    chassis = m.body("chassis").id
    from .scenes import SCENES

    # The scene's current camera (so camera moves can be retuned without re-simulating).
    live = SCENES.get(meta["scene"], scene)
    spec = camera or live.get("camera", {})
    cam = Camera(spec, fps)
    cam.frames = len(rec["t"])
    overlay = Overlay(size)
    controller = (
        "classical controller"
        if meta["controller"] == "classical"
        else f"learned policy ({meta['controller']})"
    )
    footer = footer or f"Simulation (MuJoCo, full plant, sensor model) · {controller}"
    out = Path(output or Path(npz).with_suffix(".mp4"))
    writer = imageio.get_writer(out, fps=fps, codec="libx264", quality=8, macro_block_size=8,
                                ffmpeg_params=["-pix_fmt", "yuv420p"])  # fmt: skip
    push_until, push_label = -1.0, None
    for k, (t, q, f) in enumerate(zip(rec["t"], rec["qpos"], rec["force"], strict=True)):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        renderer.update_scene(d, cam.update(k, d, chassis), scene_option=opt)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = 1
        badge = None
        if np.any(f):
            impulse = np.linalg.norm(f) * 0.1
            push_until, push_label = t + 0.6, f"push {impulse:.1f} N·s"
            direction = f / np.linalg.norm(f)
            tip = d.xpos[chassis] + np.array([0, 0, 0.03])
            _arrow(renderer.scene, tip - 0.22 * direction, tip - 0.03 * direction)
        if t < push_until:
            badge = push_label
        image = renderer.render()
        title_alpha = float(np.clip(min(t / 0.5, (3.0 - t) / 0.5), 0, 1))
        if live.get("title_hold"):
            title_alpha = 1.0
        image = overlay.draw(image, live.get("title"), title_alpha, footer, badge)
        writer.append_data(image)
        if preview_every and k % preview_every == 0:
            imageio.imwrite(out.with_name(f"{out.stem}_f{k:04d}.png"), image)
    writer.close()
    renderer.close()
    return out
