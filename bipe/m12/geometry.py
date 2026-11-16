"""M12 head-camera geometry: the camera pose from the M10 CAD parameters, its height and
line of sight over the operating postures and the balancing pitch, and the tag
visibility limits for the two Camera Module 3 lenses.

Frames follow the M10 plant: the chassis frame sits on the hip axis in the midplane,
x forward and z up; a positive body pitch turns the nose down (rotation about +y).
Ranges are horizontal distances from the camera to the tag plane, and elevations are
measured from the horizontal, positive up.
"""

import math

import numpy as np

from ..m10.kinematics import equilibrium, leg_fk, posture

# assembly.head_internals: the lens block's front face is 1.5 mm (the bezel plate)
# and a 0.3 mm gap behind the recessed web face.
LENS_FACE_BEHIND_WEB_M = 0.0015 + 0.0003


def camera_mount(shared):
    """Camera pinhole position (chassis frame) and lens-axis elevation above body x.

    Mirrors cad/m10/exterior.camera_bezel and assembly.head_internals in metres: the
    lens axis is pitched up camera_pitch_deg about a y axis through the recessed web
    face at the camera height, and the pinhole is taken at the lens block's front face.
    """
    c = shared["m10"]
    b, h, xh = c["body"], c["head"], c["exterior"]["head"]
    cx = -b["hip_x"] + h["x"]
    front = cx + h["pod_depth_x"] / 2
    top = -b["hip_z_above_bottom"] + b["height"]
    zc = top + h["neck_height"] + h["pod_height"] / 2 - xh["base_flat_from_bottom"]
    cam_z = zc - xh["camera_drop_z"]
    pivot_x = front - xh["front_recess"]
    pitch = math.radians(xh["camera_pitch_deg"])
    back = LENS_FACE_BEHIND_WEB_M
    position = [pivot_x - back * math.cos(pitch), 0.0, cam_z - back * math.sin(pitch)]
    return {
        "position_m": position,
        "pitch_deg": xh["camera_pitch_deg"],
        "basis": "m10.head, m10.body and m10.exterior.head as cad/m10 builds the bezel and "
        "camera; " + shared["m12"]["camera"]["optical_centre_assumption"],
    }


def camera_pose(p, mount, angle_deg, pitch_offset_rad=0.0):
    """Camera height above flat ground, forward offset from the axle and lens elevation,
    with the body at the posture's balance pitch plus pitch_offset_rad (nose down)."""
    q = posture(angle_deg, p)
    pitch = equilibrium(p, q) + pitch_offset_rad
    c, s = math.cos(pitch), math.sin(pitch)
    to_world = np.array([[c, s], [-s, c]])
    axle = to_world @ np.mean([leg_fk(q[:2], p), leg_fk(q[2:], p)], axis=0)
    x, _, z = mount["position_m"]
    camera = to_world @ [x, z] - axle
    return {
        "posture_deg": angle_deg,
        "body_pitch_deg": math.degrees(pitch),
        "hip_height_m": p["geometry"]["wheel_radius"] - axle[1],
        "height_m": p["geometry"]["wheel_radius"] + camera[1],
        "ahead_of_axle_m": camera[0],
        "elevation_deg": mount["pitch_deg"] - math.degrees(pitch),
    }


def edge_margin_deg(fov_deg, fraction):
    return fraction * fov_deg / 2


def vertical_band(poses, fov_deg, ranges_m, tag_size_m, margin_fraction):
    """Tag-centre heights that keep the whole tag inside the vertical field of view, less
    the edge margin, at every range and camera pose given (bearing zero, tag upright)."""
    half = math.radians(fov_deg / 2 - edge_margin_deg(fov_deg, margin_fraction))
    low, high = -math.inf, math.inf
    for pose in poses:
        e = math.radians(pose["elevation_deg"])
        for r in ranges_m:
            low = max(low, pose["height_m"] + r * math.tan(e - half) + tag_size_m / 2)
            high = min(high, pose["height_m"] + r * math.tan(e + half) - tag_size_m / 2)
    return {"low_m": low, "high_m": high, "width_m": high - low}


def elevation_window(poses, fov_deg, ranges_m, tag_height_m, tag_size_m, margin_fraction):
    """Camera pitch above body x (degrees) that keeps a tag centred at tag_height_m in the
    vertical field of view at every range and pose; the poses' camera heights and body
    pitches are kept (the pinhole moves by under 1 mm with the pitch)."""
    half = math.radians(fov_deg / 2 - edge_margin_deg(fov_deg, margin_fraction))
    low, high = -math.inf, math.inf
    for pose in poses:
        body = pose["body_pitch_deg"]
        for r in ranges_m:
            top = math.atan2(tag_height_m + tag_size_m / 2 - pose["height_m"], r)
            bottom = math.atan2(tag_height_m - tag_size_m / 2 - pose["height_m"], r)
            low = max(low, math.degrees(top - half) + body)
            high = min(high, math.degrees(bottom + half) + body)
    return {"low_deg": low, "high_deg": high, "feasible": low <= high}


def visible_heights(pose, fov_deg, range_m, margin_fraction):
    half = math.radians(fov_deg / 2 - edge_margin_deg(fov_deg, margin_fraction))
    e = math.radians(pose["elevation_deg"])
    return [pose["height_m"] + range_m * math.tan(e - s * half) for s in (1, -1)]


def bearing_allowance_rad(fov_deg, margin_fraction, tag_size_m, range_m):
    """Bearing at which the near tag edge reaches the horizontal edge margin."""
    half = math.radians(fov_deg / 2 - edge_margin_deg(fov_deg, margin_fraction))
    return half - math.atan(tag_size_m / 2 / range_m)


def max_bearing_rate(allowance_rad, lag_s, yaw_acceleration):
    """Largest step in target bearing rate the body can catch before the tag leaves the
    view: the bearing error grows at the full rate for the lag, then closes as the yaw
    rate ramps up at the yaw acceleration, peaking at w*lag + w^2/(2a)."""
    a = yaw_acceleration
    return a * (-lag_s + math.sqrt(lag_s**2 + 2 * allowance_rad / a))


def pixels_on_tag(fov_deg, image_width_px, tag_size_m, range_m):
    """Tag width in pixels for a pinhole camera of that horizontal field of view."""
    focal = image_width_px / 2 / math.tan(math.radians(fov_deg / 2))
    return focal * tag_size_m / range_m
