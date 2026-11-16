"""M12 following scenarios: scripted target and occluder paths and the simulated world.

The target is a person-sized kinematic (mocap) body carrying a flat tag; an occluder is a
second person without a tag. Neither collides with anything (contype and conaffinity 0),
so they never touch or constrain the robot. Paths are scripted in time from legs:

    ("stand", seconds)
    ("straight", metres, speed[, heading_deg])   heading_deg sets the walking direction
    ("arc", radius, turn_deg, speed)             positive turns left
    ("stop",)                                    brake to rest along the current heading

Speed follows each leg's speed at the path's acceleration limit. The tag faces back
along the walking direction ("back", worn) or toward the robot's camera ("robot",
carried and held toward it; the follower or the renderer sets that yaw from the robot's
pose, so tag_yaw is NaN in the track). Phase A's tag texture is a placeholder; phase C brings the AprilTag
images. Positions are the tag centre's ground point; the robot starts at the origin
facing +x in the nominal posture, with the tag at the nominal range from the camera.
"""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..m10 import plant
from ..model import values

DT = 0.01
# A worn tag sits 1 cm proud of the person's back: legs 0.065 m and torso 0.17 m radius
# (the torso capsule reaches down to 0.83 m).
LEG_RADIUS_M, TORSO_RADIUS_M, TORSO_BOTTOM_M = 0.065, 0.17, 0.83
BODY_BEHIND_CARRIED_TAG_M = 0.35
ASPECT = 16 / 9  # the 2304 x 1296 and 1536 x 864 sensor modes


def stand(seconds):
    return ("stand", seconds)


def straight(metres, speed, heading_deg=None):
    return ("straight", metres, speed, heading_deg)


def arc(radius, turn_deg, speed):
    return ("arc", radius, turn_deg, speed)


def stop():
    return ("stop",)


def path(legs, start, heading_deg, acceleration, dt=DT):
    """Sample a scripted path: time, x, y, walking heading, speed and acceleration."""
    x, y = start
    heading, v = math.radians(heading_deg), 0.0
    rows = [(0.0, x, y, heading, v, 0.0)]

    def advance(target, curvature=0.0):
        nonlocal x, y, heading, v
        step = float(np.clip(target - v, -acceleration * dt, acceleration * dt))
        v_new = v + step
        ds = (v + v_new) / 2 * dt
        heading += curvature * ds
        x += ds * math.cos(heading)
        y += ds * math.sin(heading)
        v = v_new
        rows.append((rows[-1][0] + dt, x, y, heading, v, step / dt))
        return ds

    for leg in legs:
        kind = leg[0]
        if kind == "stand":
            for _ in range(round(leg[1] / dt)):
                advance(0.0)
        elif kind == "straight":
            if leg[3] is not None:
                heading = math.radians(leg[3])
            done = 0.0
            while done < leg[1]:
                done += advance(leg[2])
        elif kind == "arc":
            radius, turn, speed = leg[1], math.radians(leg[2]), leg[3]
            done = 0.0
            while done < abs(turn) * radius:
                done += advance(speed, math.copysign(1 / radius, turn))
        elif kind == "stop":
            while v > 0:
                advance(0.0)
        else:
            raise ValueError(f"unknown leg {kind}")
    t, x, y, h, v, a = (np.array(c) for c in zip(*rows, strict=True))
    return {"t": t, "x": x, "y": y, "heading": h, "speed": v, "acceleration": a}


def at(track, t):
    """Track state at time t (held at its ends)."""
    i = min(len(track["t"]) - 1, max(0, round(t / DT)))
    return {k: float(v[i]) for k, v in track.items()}


EXPECTATIONS = {
    "follow": "following throughout: no loss, range inside the band",
    "follow_reacquire": "brief losses allowed, each reacquired without reaching hold",
    "close_range": "no loss; the range may leave the band but stays above the stop range",
    "stop_at_hold": "the loss rule brakes to hold within the stopping distance; no search",
    "hold_then_engage": "the loss rule brakes to hold, then the engage rule restarts following",
}


def definitions(following, ranges_ahead_m):
    """The phase A scenario set. ranges_ahead_m is the nominal range plus the camera's
    distance ahead of the axle, so each target starts at the nominal range."""
    req = following["target"]
    a = req["acceleration_max_m_s2"]
    x0 = ranges_ahead_m
    corner = req["corner"]
    (slow, r1), (fast, r2) = req["curve_radius_min_m"]
    lateral = req["lateral_speed_max_m_s"]
    walk = req["speed_max_m_s"]
    out = {}

    def add(name, legs, purpose, expect, tag="back", acceleration=a, **extra):
        if expect not in EXPECTATIONS:
            raise ValueError(expect)
        out[name] = {
            "legs": legs,
            "start": [x0, 0.0],
            "heading_deg": 0.0,
            "acceleration": acceleration,
            "tag": tag,
            "purpose": purpose,
            "expect": expect,
            **extra,
        }

    for speed in (0.3, 0.6, walk):
        add(
            f"straight_{speed:.1f}",
            [stand(1), straight(6 + 4 * speed, speed), stop(), stand(2)],
            f"straight walk away at {speed} m/s, then stop",
            "follow",
        )
    add(
        "start_stop",
        [stand(1), straight(3, walk), stop(), stand(2), straight(3, walk), stop(), stand(2)],
        "walk, stop for 2 s, walk again, stop",
        "follow",
    )
    add(
        "curve_r1",
        [stand(1), straight(2, slow), arc(r1, 180, slow), straight(2, slow), stop(), stand(2)],
        f"half turn of {r1} m radius at {slow} m/s; a tag worn on the back turns past the "
        "60 degree view angle for a moment",
        "follow_reacquire",
    )
    add(
        "curve_r2",
        [stand(1), straight(2, fast), arc(r2, 180, fast), straight(2, fast), stop(), stand(2)],
        f"half turn of {r2} m radius at {fast} m/s",
        "follow",
    )
    add(
        "corner_90",
        [
            stand(1),
            straight(3, walk),
            straight(1, corner["speed_m_s"]),
            arc(corner["radius_m"], 90, corner["speed_m_s"]),
            straight(3, walk),
            stop(),
            stand(2),
        ],
        f"90 degree corner of {corner['radius_m']} m radius at {corner['speed_m_s']} m/s; "
        "the worn tag turns past the 60 degree view angle for a moment",
        "follow_reacquire",
    )
    add(
        "lateral_crossing",
        [stand(1), straight(1.5, lateral, 90), stop(), stand(3)],
        f"carried tag held toward the robot, moved 1.5 m sideways at {lateral} m/s",
        "follow",
        tag="robot",
    )
    add(
        "approach",
        [stand(1), straight(0.8, 0.6, 180), stop(), stand(3), straight(0.8, 0.6, 0), stop()]
        + [stand(2)],
        "carried tag: the target walks 0.8 m toward the robot at 0.6 m/s, faster than the "
        "robot may reverse, waits, walks back",
        "close_range",
        tag="robot",
    )
    add(
        "loss_fov",
        [stand(1), straight(3.0, 2.5, 180 - math.degrees(math.asin(0.6 / x0))), stop()]
        + [stand(3)],
        "carried tag: the target runs past the robot's left side at 2.5 m/s (outside the "
        "requirements), 0.6 m from the camera at closest, and leaves the field of view "
        "faster than the robot can turn",
        "stop_at_hold",
        tag="robot",
        acceleration=3.0,
    )
    add(
        "occlusion_short",
        [stand(1), straight(6, 0.6), stop(), stand(2)],
        "a second person crosses between robot and target at 1.4 m/s",
        "follow_reacquire",
        occluder={"cross_time_s": 4.0, "speed_m_s": 1.4, "fraction": 0.5, "stand_s": 0.0},
    )
    add(
        "occlusion_long",
        [stand(1), straight(2.0, 0.6), stop(), stand(6), straight(3, 0.6), stop(), stand(2)],
        "a second person steps between robot and target and stands 2.5 s; the target "
        "waits, then walks on",
        "hold_then_engage",
        occluder={"cross_time_s": 4.0, "speed_m_s": 1.0, "fraction": 0.6, "stand_s": 2.5},
    )
    return out


def occluder_path(spec, target, ranges_ahead_m, nominal_range_m):
    """A person crossing the robot's line of sight from the right (-y), reaching it at
    cross_time_s at the given fraction of the nominal range in front of the camera, as
    the robot following straight behind the target would see it; it stands there for
    stand_s, then walks on."""
    t0, speed = spec["cross_time_s"], spec["speed_m_s"]
    ahead = target["x"][round(t0 / DT)] - nominal_range_m
    x = ahead + (ranges_ahead_m - nominal_range_m) + spec["fraction"] * nominal_range_m
    side = 2.0
    # Start early enough to reach full speed (at 2 m/s^2) before the line of sight.
    accel = 2.0
    run_up = speed / accel
    start_t = t0 - (side - speed**2 / (2 * accel)) / speed - run_up
    legs = [stand(max(0.0, start_t))]
    if spec["stand_s"] > 0:
        legs += [straight(side - speed**2 / (2 * accel), speed, 90), stop()]
        legs += [stand(spec["stand_s"]), straight(3.0, speed, 90), stop()]
    else:
        legs += [straight(2 * side, speed, 90), stop()]
    track = path(legs, (x, -side), 90.0, accel)
    return track


def tracks(scenario, ranges_ahead_m, nominal_range_m):
    target = path(
        scenario["legs"], scenario["start"], scenario["heading_deg"], scenario["acceleration"]
    )
    if scenario["tag"] not in ("back", "robot"):
        raise ValueError("tag faces 'back' or 'robot'")
    target["tag_yaw"] = (
        target["heading"] + math.pi
        if scenario["tag"] == "back"
        else np.full_like(target["t"], math.nan)
    )
    out = {"target": target}
    if "occluder" in scenario:
        out["occluder"] = occluder_path(
            scenario["occluder"], target, ranges_ahead_m, nominal_range_m
        )
    return out


def _person(world, name, rgba, with_tag, tag_size, tag_height, worn):
    """A mocap person, built facing +x; its origin is the tag's ground point for a person
    with a tag, else the body centre."""
    body = ET.SubElement(world, "body", name=name, mocap="true")
    if not with_tag:
        behind = 0.0
    elif not worn:
        behind = BODY_BEHIND_CARRIED_TAG_M
    else:
        tag_top = tag_height + tag_size * 10 / 8 / 2
        behind = 0.01 + (TORSO_RADIUS_M if tag_top > TORSO_BOTTOM_M else LEG_RADIUS_M)
    # The tag faces the body's +x (toward the robot); the person stands behind it.
    c = -behind
    quiet = {"contype": "0", "conaffinity": "0", "group": "1"}
    for side in (1, -1):
        ET.SubElement(
            body,
            "geom",
            type="capsule",
            fromto=values([c, side * 0.1, 0.07, c, side * 0.1, 0.88]),
            size=str(LEG_RADIUS_M),
            rgba=values(rgba),
            **quiet,
        )
    ET.SubElement(
        body,
        "geom",
        type="capsule",
        fromto=values([c, 0, TORSO_BOTTOM_M + TORSO_RADIUS_M, c, 0, 1.4]),
        size=str(TORSO_RADIUS_M),
        rgba=values(rgba),
        **quiet,
    )
    ET.SubElement(body, "geom", type="sphere", pos=values([c, 0, 1.62]), size="0.1", **quiet)
    if with_tag:
        border = tag_size * 10 / 8
        ET.SubElement(
            body,
            "geom",
            name=f"{name}_tag",
            type="box",
            size=values([0.002, border / 2, border / 2]),
            pos=values([0.0, 0.0, tag_height]),
            material="tag_placeholder",
            **quiet,
        )
    return body


def render_fovy_deg(lens, aspect=ASPECT):
    """Vertical field of view of the pinhole render camera: wide enough to cover both
    datasheet angles at the image aspect (the datasheet pair is not one pinhole; phase C
    fits the real intrinsics)."""
    implied = 2 * math.degrees(
        math.atan(math.tan(math.radians(lens["horizontal_fov_deg"] / 2)) / aspect)
    )
    return max(lens["vertical_fov_deg"], implied)


def world_xml(p, mount, lens, following, layout, occluder=False, worn=True):
    """The M10 plant XML with the head camera, the target and an optional occluder."""
    root = ET.fromstring(plant.generate_xml(p))
    root.set("model", "bipe_m12_following")
    root.find("visual/global").set("offwidth", "1920")
    root.find("visual/global").set("offheight", "1080")
    ET.SubElement(root.find("visual"), "headlight", ambient=".35 .35 .35", diffuse=".5 .5 .5")
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1=".75 .8 .85",
        rgb2=".35 .38 .42",
        width="256",
        height="256",
    )
    ET.SubElement(
        asset,
        "texture",
        name="tag_placeholder",
        type="2d",
        builtin="checker",
        rgb1="0 0 0",
        rgb2="1 1 1",
        width="64",
        height="64",
    )
    # A 4 x 4 checker: the texture's 2 x 2 checker repeated twice on the box face.
    ET.SubElement(
        asset, "material", name="tag_placeholder", texture="tag_placeholder", texrepeat="2 2"
    )
    chassis = root.find("worldbody/body[@name='chassis']")
    pitch = math.radians(layout["camera_pitch_deg"])
    s, c = math.sin(pitch), math.cos(pitch)
    x, _, z = mount["position_m"]
    ET.SubElement(
        chassis,
        "camera",
        name="head",
        pos=values([x, 0.0, z]),
        xyaxes=values([0, -1, 0, -s, 0, c]),
        fovy=str(render_fovy_deg(lens)),
    )
    world = root.find("worldbody")
    ET.SubElement(world, "light", directional="true", dir="0.2 0.1 -1", diffuse=".6 .6 .6")
    # With shadow maps and multisampling together, some states render differently from
    # one render to the next on the EGL/NVIDIA backend; shadows are off (see frame()).
    for light in world.iter("light"):
        light.set("castshadow", "false")
    _person(
        world,
        "target",
        [0.2, 0.35, 0.6, 1],
        True,
        following["tag"]["size_m"],
        layout["centre_height_m"],
        worn,
    )
    if occluder:
        _person(world, "occluder", [0.55, 0.3, 0.25, 1], False, 0, 0, True)
    return ET.tostring(root, encoding="unicode")


def renderer(m, size, segmentation=False):
    """An offscreen renderer of size (height, width)."""
    r = mujoco.Renderer(m, *size)
    if segmentation:
        r.enable_segmentation_rendering()
    return r


def frame(r, d, camera="head"):
    """Render the state twice and keep the second image.

    On the EGL/NVIDIA backend (MuJoCo 3.14) the first render of a state can differ from
    the frame before it in a few pixels by one level; the repeat makes the image a
    function of the state alone. With shadow maps and multisampling both on, some states
    also differ between consecutive renders, so the M12 world casts no shadows."""
    r.update_scene(d, camera)
    r.render()
    r.update_scene(d, camera)
    return r.render().copy()


def place_person(m, d, name, x, y, yaw):
    i = m.body(name).mocapid[0]
    d.mocap_pos[i] = [x, y, 0.0]
    d.mocap_quat[i] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def place_robot(m, d, p, x, y, yaw, posture_deg=40, pitch_offset_deg=0.0):
    """Kinematic placement for rendering: the plant's reset pose (balance pitch plus
    pitch_offset_deg nose down, tires on the ground) turned to yaw about the axle and
    moved so the axle midpoint is at (x, y). Not a balance simulation."""
    plant.reset(m, d, p, posture_deg, pitch_offset_deg)
    axle = np.mean([d.site(s + "_axle").xpos for s in plant.SIDES], axis=0)
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)], d.qpos[3:7].copy())
    base = d.qpos[:3] - axle
    c, s = math.cos(yaw), math.sin(yaw)
    d.qpos[0] = x + c * base[0] - s * base[1]
    d.qpos[1] = y + s * base[0] + c * base[1]
    d.qpos[3:7] = q
    mujoco.mj_forward(m, d)
