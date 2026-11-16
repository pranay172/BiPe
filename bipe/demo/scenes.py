"""Demo scenes: commands, pushes, terrain and camera for each shot.

A scene's ``knots`` are [t_s, velocity_m_s, yaw_rate_rad_s, posture_deg] rows, piecewise
linear (a repeated time is a step). ``pushes`` are (t_s, impulse_N_s, heading_deg) where
heading 0 pushes along the robot's forward axis, 180 from the front, +-90 sideways; each
is a 0.1 s force pulse on the chassis, as the acceptance push cases. ``camera`` names a
camera move in bipe.demo.render.
"""

SCENES = {
    # Opening: the robot standing, balancing, camera orbiting it.
    "intro_orbit": {
        "duration": 8.0,
        "knots": [[0, 0, 0, 40]],
        "camera": {"move": "orbit", "distance": 0.6, "elevation": -12, "degrees": 120},
        "title": "BiPe: a wheeled biped, in simulation",
    },
    # Drive, turn, crouch and stand: the M10 scope.
    "drive_turn_crouch": {
        "duration": 16.0,
        "knots": [
            [0, 0, 0, 40],
            [1, 0, 0, 40],
            [3, 0.8, 0, 40],
            [5, 0.8, 0, 40],
            [6, 0.6, 0.8, 40],
            [8.5, 0.6, 0.8, 40],
            [9.5, 0.3, 0, 40],
            [10.5, 0, 0, 40],
            [11, 0, 0, 40],
            [12.5, 0, 0, 65],
            [13.5, 0, 0, 65],
            [15, 0, 0, 30],
            [16, 0, 0, 40],
        ],
        "camera": {"move": "chase", "distance": 0.9, "elevation": -14, "azimuth": 120},
        "title": "Drive, turn, crouch",
    },
    # Pushes from behind and in front while standing.
    "push_recovery": {
        "duration": 12.0,
        "knots": [[0, 0, 0, 40]],
        "pushes": [(2.0, 1.5, 0), (5.0, 1.5, 180), (8.0, 2.5, 0)],
        "camera": {"move": "fixed", "distance": 0.7, "elevation": -8, "azimuth": 90},
        "title": "Push recovery",
    },
    # A push while driving.
    "push_while_driving": {
        "duration": 10.0,
        "knots": [
            [0, 0, 0, 40],
            [1, 0, 0, 40],
            [2.5, 0.6, 0, 40],
            [7, 0.6, 0, 40],
            [8.5, 0, 0, 40],
        ],
        "pushes": [(4.5, 2.0, 180)],
        "camera": {"move": "chase", "distance": 0.8, "elevation": -10, "azimuth": 90},
        "title": "Push while driving",
    },
    # Up and down a ramp (the plant's tilted ground plane).
    "slope": {
        "duration": 12.0,
        "slope_deg": 10.0,
        "knots": [
            [0, 0, 0, 40],
            [1, 0, 0, 40],
            [2.5, 0.5, 0, 40],
            [5.5, 0.5, 0, 40],
            [6.5, 0, 0, 40],
            [7.5, 0, 0, 40],
            [9, -0.3, 0, 40],
            [11, -0.3, 0, 40],
            [12, 0, 0, 40],
        ],
        "camera": {"move": "chase", "distance": 0.9, "elevation": -8, "azimuth": 100},
        "title": "10 degree slope",
    },
    # M15 moves on the learned policy (bipe.m15.moves): crouch bobs, a turn and back,
    # a yaw wiggle.
    "moves": {
        "duration": 22.0,
        "knots": [
            [0, 0, 0, 40], [1, 0, 0, 40],
            [2.25, 0, 0, 35], [3.5, 0, 0, 60], [4.75, 0, 0, 35], [6, 0, 0, 60],
            [7.25, 0, 0, 40], [8, 0, 0, 40],
            [8.5, 0, 1.0, 40], [14.28, 0, 1.0, 40], [14.78, 0, 0, 40], [15.5, 0, 0, 40],
            [15.5, 0, 0.8, 40], [16, 0, 0.8, 40], [16, 0, -0.8, 40], [16.5, 0, -0.8, 40],
            [16.5, 0, 0.8, 40], [17, 0, 0.8, 40], [17, 0, -0.8, 40], [17.5, 0, -0.8, 40],
            [17.5, 0, 0.8, 40], [18, 0, 0.8, 40], [18, 0, -0.8, 40], [18.5, 0, -0.8, 40],
            [18.5, 0, 0, 40], [22, 0, 0, 40],
        ],
        "camera": {"move": "fixed", "distance": 0.75, "elevation": -10, "azimuth": 110},
        "title": "Moves: crouch, turn, wiggle (learned policy)",
    },
    # Learned vs classical (record once per controller, then edit pair): pushes under
    # the 15 ms delay, noise x2 variant, where a 1.5 N s push takes the classical
    # controller past its pitch limit.
    "compare_delay_push": {
        "duration": 9.0,
        "variant": "delay_15ms_noise_x2",
        "knots": [[0, 0, 0, 40]],
        "pushes": [(2.0, 1.5, 0), (5.0, 1.5, 180)],
        "camera": {"move": "fixed", "distance": 0.7, "elevation": -8, "azimuth": 90},
        "title": "15 ms delay, noise x2, pushes",
    },
    # End card: a last slow orbit.
    "end_orbit": {
        "duration": 6.0,
        "knots": [[0, 0, 0, 40], [2, 0, 0, 40], [3.5, 0, 0, 55], [5, 0, 0, 40]],
        "camera": {
            "move": "orbit",
            "distance": 0.65,
            "elevation": -10,
            "azimuth": 200,
            "degrees": 70,
        },
        "title": None,
    },
}

# Same controller, different robots: one tile per acceptance variant, each pushed.
VARIANT_TILES = {
    "mass_plus_15pct": "mass +15 %",
    "friction_0p5": "floor friction 0.5",
    "wheel_lag_10ms": "hub motor lag 10 ms",
    # Under this delay a 1.5 N s push already takes the classical controller past the
    # 20 degree pitch limit (2 November 2026), so this tile holds without pushes.
    "delay_15ms_noise_x2": "15 ms delay, noise x2, no push",
}
for _variant, _label in VARIANT_TILES.items():
    SCENES[f"variant_{_variant}"] = {
        "duration": 8.0,
        "variant": _variant,
        "knots": [[0, 0, 0, 40]],
        "pushes": [] if _variant == "delay_15ms_noise_x2" else [(2.0, 1.5, 0), (5.0, 1.5, 180)],
        "camera": {"move": "fixed", "distance": 0.7, "elevation": -8, "azimuth": 90},
        "title": _label,
        "title_hold": True,
    }
