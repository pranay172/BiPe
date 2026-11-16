"""M9 ground plane tilted after compile: the world BVH must follow it.

The compiler bounds the world body by the flat half-space z <= 0. M9 writes
geom_quat after compile (bipe/m9_plant.py reset, and bipe/m9_impact.py
run_case for cross_slope_deg), so both refit the bound with _refit_world_bvh.
Without it MuJoCo's midphase drops contacts whose other body has several geoms
once the tilted plane rises above z = 0; a single-geom wheel is not culled.
The cross-slope case mirrors run_case: quaternion write, then the refit.
"""

import math

import mujoco
import pytest

from bipe.m9_plant import _refit_world_bvh, build, reset
from bipe.parameters import load_parameters

MIDPHASE = int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE)


@pytest.fixture(scope="module")
def planted():
    p = load_parameters(profile="m9")
    model, data = build(p)
    return p, model, data


def _move(planted, slope_deg, dx, drop, cross):
    p, m, d = planted
    reset(m, d, p, slope_deg=0 if cross else slope_deg)
    if cross:
        # Same quaternion m9_impact writes for cross_slope_deg, after reset.
        angle = math.radians(slope_deg)
        ground = m.geom("ground").id
        m.geom_quat[ground] = [math.cos(angle / 2), math.sin(angle / 2), 0, 0]
        _refit_world_bvh(m, ground)
        mujoco.mj_forward(m, d)
    pose = d.qpos.copy()
    pose[1 if cross else 0] += dx
    pose[2] += dx * math.tan(math.radians(slope_deg)) - drop
    return pose


def _counts(planted, pose):
    _, m, d = planted
    counts = []
    for flags in (0, MIDPHASE):
        m.opt.disableflags = flags
        d.qpos[:] = pose
        mujoco.mj_forward(m, d)
        counts.append(int(d.ncon))
    m.opt.disableflags = 0
    return counts


def test_m9_tires_and_downhill_shells_are_not_culled(planted):
    """Wheels are one geom, so tire contacts survive a stale bound. Downhill the
    plane stays inside z <= 0, so shell contacts survive too."""
    _, m, d = planted
    for slope, dx in ((-10, -0.6), (-10, 0.6), (4, -0.3), (4, 0.3), (10, -0.6), (10, 0.6)):
        pose = _move(planted, slope, dx, 0.0, cross=False)
        mid, brute = _counts(planted, pose)
        names = {m.geom(int(g)).name for i in range(d.ncon) for g in d.contact[i].geom}
        assert mid == brute >= 2, (slope, dx, mid, brute)
        assert names <= {"ground", "left_tire", "right_tire"}, names
    # Downhill shell strike: the body is lowered into the plane on the side
    # that is still inside the compiler half-space.
    for slope, dx, cross in ((8, -0.8, False), (-8, 0.8, False), (-8, 0.5, True)):
        pose = _move(planted, slope, dx, 0.05 if not cross else 0.04, cross=cross)
        mid, brute = _counts(planted, pose)
        assert mid == brute >= 20, (slope, dx, cross, mid, brute)


def test_m9_tilted_plane_keeps_uphill_shell_contacts(planted):
    """Uphill, and on a cross-slope, multi-geom bodies intersect the plane outside
    the flat half-space. Midphase must keep every contact the brute-force test finds
    (without the refit: 4 of 68 uphill, 4 of 40 cross-slope)."""
    missed = []
    cases = (
        (8, 0.8, 0.05, False),
        (15, 0.8, 0.05, False),
        (-8, -0.8, 0.05, False),
        (8, 0.5, 0.04, True),
        (15, 0.6, 0.05, True),
    )
    for slope, dx, drop, cross in cases:
        pose = _move(planted, slope, dx, drop, cross)
        mid, brute = _counts(planted, pose)
        if mid != brute:
            missed.append(
                f"slope {slope:+g} {'cross' if cross else 'long'} dx {dx:+.2f} "
                f"drop {drop:.2f}: midphase {mid} brute {brute}"
            )
    assert not missed, "stale world BVH culled uphill contacts:\n" + "\n".join(missed)
