"""The tag pose from its four detected corners, through the lens model.

The corners are undistorted, a homography gives the start and Levenberg-Marquardt refines
the reprojection error in pixels. Corners follow the apriltag library's order,
counter-clockwise around the tag, matched to the object points (-s/2, s/2), (s/2, s/2),
(s/2, -s/2), (-s/2, -s/2) in the tag plane. The pose is in the camera frame (x right,
y down, z forward)."""

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import lens as C


def object_points(size_m):
    h = size_m / 2
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def fit_pose(model, corners, size_m):
    """Tag pose in the camera frame (x right, y down, z forward) from pixel corners."""
    xu, yu = C.normalised(model, corners[:, 0], corners[:, 1])
    obj = object_points(size_m)
    # Homography from the tag plane to normalised image points (DLT).
    a = []
    for (X, Y, _), x, y in zip(obj, xu, yu, strict=True):
        a.append([X, Y, 1, 0, 0, 0, -x * X, -x * Y, -x])
        a.append([0, 0, 0, X, Y, 1, -y * X, -y * Y, -y])
    h = np.linalg.svd(np.array(a))[2][-1].reshape(3, 3)
    h /= np.linalg.norm(h[:, 0])
    if h[2, 2] < 0:
        h = -h
    r1, r2 = h[:, 0], h[:, 1] / np.linalg.norm(h[:, 1]) * 1.0
    r3 = np.cross(r1, r2)
    u, _, vt = np.linalg.svd(np.c_[r1, r2, r3])
    r0 = u @ vt
    scale = 2 / (np.linalg.norm(h[:, 0]) + np.linalg.norm(h[:, 1]))
    t0 = h[:, 2] * scale

    def residual(x):
        rot = Rotation.from_rotvec(x[:3]).as_matrix()
        pts = obj @ rot.T + x[3:]
        u_, v_ = C.project(model, pts[:, 0] / pts[:, 2], pts[:, 1] / pts[:, 2])
        return np.r_[u_ - corners[:, 0], v_ - corners[:, 1]]

    x0 = np.r_[Rotation.from_matrix(r0).as_rotvec(), t0]
    sol = least_squares(residual, x0, method="lm")
    return {
        "rotation": Rotation.from_rotvec(sol.x[:3]).as_matrix(),
        "translation": sol.x[3:],
        "reprojection_rms_px": float(np.sqrt(np.mean(sol.fun**2))),
    }
