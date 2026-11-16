"""The Camera Module 3 Wide lens model: Brown-Conrady radial k1, k2 about the principal
point. model is a dict with f_px, cx_px, cy_px, k1 and k2 (bipe.m12.camera.intrinsics in
the simulation; a calibration on the real camera in phase E)."""

import numpy as np


def distort_radius(model, ru):
    r2 = ru * ru
    return ru * (1 + model["k1"] * r2 + model["k2"] * r2 * r2)


def undistort_radius(model, rd, iterations=30):
    """Invert the radial polynomial by Newton's method (monotonic over the image)."""
    ru = np.array(rd, dtype=float)
    for _ in range(iterations):
        r2 = ru * ru
        f = ru * (1 + model["k1"] * r2 + model["k2"] * r2 * r2) - rd
        df = 1 + 3 * model["k1"] * r2 + 5 * model["k2"] * r2 * r2
        ru = ru - f / df
    return ru if np.ndim(rd) else float(ru)


def normalised(model, u, v):
    """Pixel (distorted) to undistorted normalised coordinates."""
    xd = (np.asarray(u, float) - model["cx_px"]) / model["f_px"]
    yd = (np.asarray(v, float) - model["cy_px"]) / model["f_px"]
    rd = np.hypot(xd, yd)
    ru = undistort_radius(model, rd)
    scale = np.where(rd > 1e-12, ru / np.maximum(rd, 1e-12), 1.0)
    return xd * scale, yd * scale


def project(model, xu, yu):
    """Undistorted normalised coordinates to pixels."""
    ru = np.hypot(xu, yu)
    rd = distort_radius(model, ru)
    scale = np.where(ru > 1e-12, rd / np.maximum(ru, 1e-12), 1.0)
    return model["cx_px"] + model["f_px"] * xu * scale, model["cy_px"] + model["f_px"] * yu * scale
