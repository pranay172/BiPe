"""M12 phase C camera model: the Camera Module 3 Wide as the head camera sees the world.

MuJoCo renders pinhole images. The Wide lens is distorted, so a frame is rendered over a
wider pinhole field at a finer pitch than the output and remapped through the lens model
onto the 1152 x 648 output grid. Exposure is the mean of sub-frame renders spread over the
exposure time (motion blur, global shutter; rolling shutter is a later option). Shot and
read noise come from an electron budget at the stated indoor light level, and the ISP's
tone curve and 8-bit quantisation follow. The detector sees the luminance (Y) plane.

Sources and assumptions (config/robot.json m12.camera and config/perception-m12.json):
- focal length, pixel pitch, sensor size and fields of view from the Raspberry Pi camera
  documentation;
- the output pitch: the 2304 x 1296 mode bins 2 x 2 and the ISP scales it by half, so an
  output pixel spans 4 x 4 native pixels (5.6 um);
- the distortion: Brown-Conrady radial k1, k2 fitted so the lens reaches the documented
  horizontal and vertical fields at the documented focal length; an assumption until a
  real calibration exists;
- light, quantum efficiency, read noise, lens blur and the tone curve: assumptions with
  their basis in the config.
"""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy import ndimage

from ..model import values
from ..pi.lens import distort_radius, normalised, project, undistort_radius  # noqa: F401
from . import scenarios as S

LUMA = np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)  # Rec. 709 luminance of linear RGB
TAG_INK_M = 0.0002  # black cells stand this proud of the white board
# Matte paper and ink (no specular highlight); rendered base colours.
TAG_WHITE_RGBA = [0.9, 0.9, 0.9, 1.0]
TAG_BLACK_RGBA = [0.03, 0.03, 0.03, 1.0]


# Intrinsics and distortion ------------------------------------------------------------


def fit_radial(f_px, half_width_px, half_height_px, hfov_deg, vfov_deg):
    """Brown-Conrady k1, k2 that put the documented horizontal and vertical half fields
    at the image edges for the documented focal length: two conditions, two unknowns."""
    rows, rhs = [], []
    for half_px, fov in ((half_width_px, hfov_deg), (half_height_px, vfov_deg)):
        ru = math.tan(math.radians(fov / 2))
        rows.append([ru**3, ru**5])
        rhs.append(half_px / f_px - ru)
    k1, k2 = np.linalg.solve(np.array(rows), np.array(rhs))
    return float(k1), float(k2)


def intrinsics(shared, perception):
    """Output-image intrinsics of the chosen lens, with their sources."""
    cam = shared["m12"]["camera"]
    lens = cam["lenses"][perception["camera"]["lens"]]
    out = perception["camera"]["output"]
    width, height = out["width_px"], out["height_px"]
    sensor = shared["m12"]["camera_sensor"]
    pitch = sensor["pixel_size_m"] * out["native_pixels_per_output_pixel"]
    f_px = lens["focal_length_m"] / pitch
    k1, k2 = fit_radial(
        f_px, width / 2, height / 2, lens["horizontal_fov_deg"], lens["vertical_fov_deg"]
    )
    model = {
        "width_px": width,
        "height_px": height,
        "pixel_pitch_m": pitch,
        "f_px": f_px,
        "cx_px": (width - 1) / 2,
        "cy_px": (height - 1) / 2,
        "k1": k1,
        "k2": k2,
    }
    corner = math.hypot(width / 2, height / 2) / f_px
    model["diagonal_fov_deg"] = 2 * math.degrees(math.atan(undistort_radius(model, corner)))
    model["basis"] = (
        f"focal length {lens['focal_length_m'] * 1e3:g} mm ({cam['source']}) and "
        f"{sensor['pixel_size_m'] * 1e6:g} um pixels ({sensor['source']}); {out['basis']}; "
        "principal point at the image centre and "
        "k1, k2 fitted to the documented horizontal and vertical fields (an assumption until "
        "a calibration exists)"
    )
    return model


def monotonic_over_image(model):
    corner = math.hypot(model["width_px"] / 2, model["height_px"] / 2) / model["f_px"]
    ru = undistort_radius(model, corner)
    r2 = np.linspace(0, ru, 200) ** 2
    return bool(np.all(1 + 3 * model["k1"] * r2 + 5 * model["k2"] * r2 * r2 > 0))


# Render geometry ----------------------------------------------------------------------


def render_geometry(model, supersample):
    """The pinhole render that covers the whole distorted output, at supersample render
    pixels per output pixel at the image centre, and the remap from output to render."""
    w, h = model["width_px"], model["height_px"]
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    xu, yu = normalised(model, u, v)
    f_r = supersample * model["f_px"]
    # Two render pixels of margin past the furthest sample, for the bilinear stencil.
    half_w = math.ceil(np.abs(xu).max() * f_r) + 2
    half_h = math.ceil(np.abs(yu).max() * f_r) + 2
    width, height = 2 * half_w, 2 * half_h
    fovy = 2 * math.degrees(math.atan(height / 2 / f_r))
    # MuJoCo's image rows run top to bottom, image x to the right, as the output's.
    cols = (width - 1) / 2 + xu * f_r
    rows = (height - 1) / 2 + yu * f_r
    return {
        "width": width,
        "height": height,
        "fovy_deg": fovy,
        "f_px": f_r,
        "supersample": supersample,
        "map": np.stack([rows, cols]).astype(np.float32),
    }


# World --------------------------------------------------------------------------------


def black_runs(code_image):
    """Horizontal runs of black cells, as (row, first column, length)."""
    runs = []
    for r, row in enumerate(np.asarray(code_image) < 128):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c < len(row) and row[c]:
                    c += 1
                runs.append((r, start, c - start))
            else:
                c += 1
    return runs


def world(p, mount, lens, following, layout, geometry, tag_image, occluder=False, worn=True):
    """The phase A world with the AprilTag on the target and the head camera set up as the
    wide pinhole render (model XML)."""
    root = ET.fromstring(S.world_xml(p, mount, lens, following, layout, occluder, worn))
    root.set("model", "bipe_m12_perception")
    glob = root.find("visual/global")
    glob.set("offwidth", str(max(geometry["width"], 1920)))
    glob.set("offheight", str(max(geometry["height"], 1080)))
    # The tag is built from geometry: a matte white board and a thin black box for each
    # run of black cells, standing TAG_INK_M proud of the board. Exact edges, without
    # the texture sampling of a textured face (which placed the cells up to 2 mm off).
    asset = root.find("asset")
    for name, rgba in (("tag_white", TAG_WHITE_RGBA), ("tag_black", TAG_BLACK_RGBA)):
        ET.SubElement(
            asset,
            "material",
            name=name,
            rgba=values(rgba),
            specular="0",
            shininess="0",
            reflectance="0",
        )
    tag = root.find(".//geom[@name='target_tag']")
    tag.set("material", "tag_white")
    board_half = float(tag.get("size").split()[1])
    front = [float(v) for v in tag.get("pos").split()]
    front[0] += float(tag.get("size").split()[0])
    cells = len(tag_image)
    cell = 2 * board_half / cells
    body = root.find(".//body[@name='target']")
    quiet = {"contype": "0", "conaffinity": "0", "group": "1"}
    for k, (r, c, n) in enumerate(black_runs(tag_image)):
        # Image column c runs to the viewer's right, which is +y for a tag facing +x.
        y = (c + n / 2) * cell - board_half
        z = front[2] + board_half - (r + 0.5) * cell
        ET.SubElement(
            body,
            "geom",
            name=f"target_tag_ink{k}",
            type="box",
            size=values([TAG_INK_M / 2, n * cell / 2, cell / 2]),
            pos=values([front[0] + TAG_INK_M / 2, front[1] + y, z]),
            material="tag_black",
            **quiet,
        )
    root.find(".//camera[@name='head']").set("fovy", values([geometry["fovy_deg"]]))
    return ET.tostring(root, encoding="unicode")


# Image pipeline -----------------------------------------------------------------------


def srgb_to_linear(c):
    c = np.asarray(c, dtype=np.float32) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


SRGB_TO_LINEAR_LUT = srgb_to_linear(np.arange(256)).astype(np.float32)


def linear_to_srgb(c):
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.0031308, 12.92 * c, 1.055 * np.power(c, 1 / 2.4) - 0.055)


def electrons_at_white(noise, exposure_s, f_number, binned, pixel_area_m2):
    """Photo-electrons per output pixel from a perfect white diffuser at the stated
    illuminance (camera equation for a Lambertian scene: E_sensor = E T / (4 N^2))."""
    lux_at_sensor = noise["illuminance_lux"] * noise["lens_transmission"] / (4 * f_number**2)
    photons = lux_at_sensor * noise["photons_per_lux_s_m2"] * pixel_area_m2
    return photons * noise["quantum_efficiency"] * exposure_s * binned


class Camera:
    """Renders head-camera exposures through the lens, noise and tone model."""

    def __init__(self, m, model, geometry, perception, f_number, pixel_size_m):
        self.model, self.geometry = model, geometry
        self.pixel_area = pixel_size_m**2
        self.cfg = perception["camera"]
        self.noise = self.cfg["noise"]
        self.renderer = mujoco.Renderer(m, geometry["height"], geometry["width"])
        self.binned = self.cfg["output"]["native_pixels_per_output_pixel"] ** 2
        self.f_number = f_number
        blur = self.cfg["blur"]
        sigma = math.hypot(blur["pixel_aperture_sigma_px"], blur["lens_sigma_px"])
        self.sigma_render = sigma * geometry["supersample"]

    def close(self):
        self.renderer.close()

    def linear(self, d):
        """Linear luminance of the pinhole render of the current state."""
        rgb = S.frame(self.renderer, d)
        lut = SRGB_TO_LINEAR_LUT
        return lut[rgb[..., 0]] * LUMA[0] + lut[rgb[..., 1]] * LUMA[1] + lut[rgb[..., 2]] * LUMA[2]

    def expose(self, set_state, d, t_end, exposure_s, subframes, rng):
        """One frame whose exposure ends at t_end: the mean of `subframes` renders at the
        sub-frame centres, set by set_state(t); then lens, noise and tone."""
        acc = None
        for k in range(subframes):
            t = t_end - exposure_s * (1 - (k + 0.5) / subframes)
            set_state(t)
            img = self.linear(d)
            acc = img if acc is None else acc + img
        return self.develop(acc / subframes, exposure_s, rng)

    def develop(self, linear, exposure_s, rng):
        """Lens and pixel blur, remap through the distortion, noise, tone curve, 8 bits."""
        smooth = ndimage.gaussian_filter(linear, self.sigma_render, mode="nearest")
        out = ndimage.map_coordinates(smooth, self.geometry["map"], order=1, mode="nearest")
        n = self.noise
        white = electrons_at_white(n, exposure_s, self.f_number, self.binned, self.pixel_area)
        if rng is not None:
            e = out * white
            read = n["read_noise_e_native"] * math.sqrt(self.binned)
            e = e + rng.standard_normal(e.shape) * np.sqrt(np.maximum(e, 0) + read**2)
            out = e / white
        # Auto exposure and gain bring a perfect white to the stated level, then the tone
        # curve (sRGB) and 8-bit quantisation of the ISP's Y output.
        y = linear_to_srgb(out * n["white_level"])
        return np.clip(np.rint(y * 255), 0, 255).astype(np.uint8)
