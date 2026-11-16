"""M10 phase D fall protection: frame-mounted foam face pads and flexible TPU ears.

Face pads (a skid under the body tub and a ring on each wheel's outer face) are
LD45 closed-cell foam behind a thin TPU wear skin. They are carried by the
chassis frame or the wheel hub, so their load bypasses the printed shells. The
law integrates the supplier compression curve over the pad face as it actually
penetrates the ground, so a tilted pad bears on an edge sliver and a flat pad on
its whole face. The TPU skin is thin and much stiffer than the foam; it is
neglected in series.

TPU ears are flexible cantilever flaps: the contact law is the tip stiffness
3 E I / L^3 with the TPU flexural modulus, and the screen checks the root
bending strain against the supplier elongation at break.

All values are stated digital assumptions on top of supplier data
(config m10.physics.bumpers, config/m9-impact-materials.json).
"""

import math

import mujoco
import numpy as np

# Pad faces are sampled on fixed node grids (edges included, trapezoid weights) in the
# geom frame, so an edge landing engages from its first contact.
BOX_NODES = (49, 9)
RING_RADIAL, RING_ANGULAR = 7, 96


def _trapezoid_weights(n):
    w = np.ones(n)
    w[[0, -1]] = 0.5
    return w / (n - 1)


def foam_stress(foam, strain, scale=1.0):
    """LD45 supplier compression curve, then the densification term (M9 bumper law)."""
    strain = np.asarray(strain, dtype=float)
    stress = np.interp(strain, foam["compression_strain"], foam["compression_stress_Pa"])
    stress = stress + foam["densification_modulus_Pa_assumption"] * np.maximum(strain - 0.5, 0) ** 2
    return scale * stress


def foam_energy_density(foam, strain, scale=1.0):
    """Strain energy per unit volume to the given strains, J/m^3."""
    strain = np.asarray(strain, dtype=float)
    grid = np.linspace(0.0, max(float(strain.max(initial=0.0)), 1e-9), 257)
    cumulative = np.concatenate(
        [
            [0.0],
            np.cumsum(
                np.diff(grid)
                * 0.5
                * (foam_stress(foam, grid[1:], scale) + foam_stress(foam, grid[:-1], scale))
            ),
        ]
    )
    return np.interp(strain, grid, cumulative)


def face_samples(pad):
    """Sample points (geom frame), plan areas and foam thickness of a pad's contact face.

    The foam is a bed of columns along the geom z axis on a flat backing.
    box: the face is the geom's -z face, the backing its +z face. Optional end bevels
    (``end_bevel_deg`` outside the central ``flat_length_m``) raise the face towards
    the ends, so a pitched landing meets a facet instead of an edge.
    ring: the geom is a cylinder whose +z face is the outer face and whose -z face sits
    on the wheel. The foam is the annulus between the inner and outer radii; an optional
    ``dome_deg`` thins it linearly from the inner to the outer radius (a coned hubcap),
    so a tilted wheel meets a generatrix instead of the rim.
    """
    if pad["shape"] == "box":
        lx, ly, lz = pad["size_m"]
        nx, ny = BOX_NODES
        x, y = np.meshgrid(np.linspace(-lx / 2, lx / 2, nx), np.linspace(-ly / 2, ly / 2, ny))
        weights = np.outer(_trapezoid_weights(ny), _trapezoid_weights(nx)) * lx * ly
        flat = pad.get("flat_length_m", lx)
        rise = np.maximum(np.abs(x.ravel()) - flat / 2, 0) * math.tan(
            math.radians(pad.get("end_bevel_deg", 0.0))
        )
        thickness = lz - rise
        points = np.column_stack([x.ravel(), y.ravel(), -lz / 2 + rise])
        return points, weights.ravel(), thickness
    if pad["shape"] == "ring":
        outer, inner, t = pad["outer_radius_m"], pad["inner_radius_m"], pad["thickness_m"]
        radii = np.linspace(inner, outer, RING_RADIAL)
        radial = _trapezoid_weights(RING_RADIAL) * (outer - inner) * radii
        angles = np.arange(RING_ANGULAR) * 2 * math.pi / RING_ANGULAR
        r, a = np.meshgrid(radii, angles)
        local = t - (r.ravel() - inner) * math.tan(math.radians(pad.get("dome_deg", 0.0)))
        points = np.column_stack([(r * np.cos(a)).ravel(), (r * np.sin(a)).ravel(), -t / 2 + local])
        weights = np.tile(radial, RING_ANGULAR) * 2 * math.pi / RING_ANGULAR
        # Trapezoid in r of r dr is exact for the annulus area up to O(dr^2); normalise.
        weights *= math.pi * (outer**2 - inner**2) / weights.sum()
        return points, weights, local
    raise ValueError(f"Unknown pad shape {pad['shape']}")


def shaped(pad):
    return bool(pad.get("end_bevel_deg") or pad.get("dome_deg"))


def hull_vertices(pad):
    """Vertices (geom frame) whose convex hull is a shaped pad's collision geom: the face
    samples plus the backing outline, so contacts lie on the actual foam face."""
    points, _, _ = face_samples(pad)
    if pad["shape"] == "box":
        lx, ly, lz = pad["size_m"]
        base = [[sx * lx / 2, sy * ly / 2, lz / 2] for sx in (-1, 1) for sy in (-1, 1)]
    else:
        r, t = pad["outer_radius_m"], pad["thickness_m"]
        a = np.arange(RING_ANGULAR) * 2 * math.pi / RING_ANGULAR
        base = np.column_stack([r * np.cos(a), r * np.sin(a), np.full(a.size, -t / 2)])
        # The hub inside the inner radius is backed at the foam's inner height.
        inner = pad["inner_radius_m"]
        base = np.vstack(
            [base, np.column_stack([inner * np.cos(a), inner * np.sin(a), np.full(a.size, t / 2)])]
        )
    return np.vstack([points, base])


class FacePads:
    """Per-step face-pad forces, stored energy and peak strain, from the pad geoms' poses.

    The face samples sit on the rigid geom's contact face; a sample below the ground
    plane compresses the foam by its depth (the penalty penetration is the foam
    compression). Strain beyond the supplier curve (0.5) is reported as bottoming.
    """

    def __init__(self, m, pads, foam, scale=1.0):
        self.foam, self.scale = foam, scale
        self.rows = {}
        for pad in pads:
            points, areas, thickness = face_samples(pad)
            geom = m.geom(pad["name"]).id
            # Samples live in the pad's own frame in its body (``frame`` = pos, quat). The
            # compiler moves a mesh geom to the mesh's inertial frame, so only a primitive
            # geom's compiled pose may stand in for it.
            pos, quat = pad.get("frame", (m.geom_pos[geom], m.geom_quat[geom]))
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, np.asarray(quat, dtype=float))
            self.rows[pad["name"]] = {
                "geom": geom,
                "body": int(m.geom_bodyid[geom]),
                "points": np.asarray(pos) + points @ rot.reshape(3, 3).T,
                "areas": areas,
                "thickness": thickness,
            }
        self.ground = m.geom("ground").id
        self.force, self.energy, self.strain = {}, {}, {}
        self.peak_strain = {name: 0.0 for name in self.rows}

    def update(self, m, d):
        normal = d.geom_xmat[self.ground].reshape(3, 3)[:, 2]
        origin = d.geom_xpos[self.ground]
        for name, row in self.rows.items():
            b = row["body"]
            world = d.xpos[b] + row["points"] @ d.xmat[b].reshape(3, 3).T
            depth = np.maximum(-(world - origin) @ normal, 0.0)
            strain = depth / row["thickness"]
            self.force[name] = float(foam_stress(self.foam, strain, self.scale) @ row["areas"])
            self.energy[name] = float(
                foam_energy_density(self.foam, strain, self.scale)
                @ (row["areas"] * row["thickness"])
            )
            self.strain[name] = float(strain.max(initial=0.0))
            self.peak_strain[name] = max(self.peak_strain[name], self.strain[name])


def ear_cantilever(ear, tpu):
    """Tip stiffness (N/m) and root bending strain per metre of tip deflection of a TPU ear
    flap of the CAD length, width and thickness (rectangular section, root fixed)."""
    length, width, thickness = ear["length"], ear["width"], ear["thickness"]
    inertia = width * thickness**3 / 12
    stiffness = 3 * tpu["flexural_modulus_Pa"] * inertia / length**3
    strain_per_m = 3 * thickness / (2 * length**2)
    return stiffness, strain_per_m
