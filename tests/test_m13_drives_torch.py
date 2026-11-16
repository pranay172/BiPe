import numpy as np
import pytest

from bipe.m13 import drives
from bipe.parameters import load_parameters

torch = pytest.importorskip("torch")


def test_torch_drives_equal_numpy_drives():
    p = load_parameters(profile="m13")
    hm, gl, ratio = p["physics"]["hip_motor"], p["physics"]["hub_motor"], p["joint_drive"]["ratio"]
    rng = np.random.default_rng(5)
    n, dt = 64, 0.001
    current = np.zeros((n, 6))
    tcurrent = torch.zeros((n, 6), dtype=torch.float64)
    for _ in range(200):
        request = rng.normal(0, 1.0, (n, 6)) * np.r_[np.ones(4), 0.3 * np.ones(2)]
        speed = rng.normal(0, 1, (n, 6)) * np.r_[np.full(4, 200.0), np.full(2, 80.0)]
        hip = drives.hip_motor(np, hm, ratio, request[:, :4], speed[:, :4], current[:, :4], dt)
        hub = drives.hub_motor(np, gl, request[:, 4:], speed[:, 4:], current[:, 4:], dt)
        tr, ts = torch.from_numpy(request), torch.from_numpy(speed)
        thip = drives.hip_motor(torch, hm, ratio, tr[:, :4], ts[:, :4], tcurrent[:, :4], dt)
        thub = drives.hub_motor(torch, gl, tr[:, 4:], ts[:, 4:], tcurrent[:, 4:], dt)
        for a, b in zip(hip + hub, thip + thub):
            assert np.allclose(a, b.numpy(), atol=1e-12)
        current = np.concatenate([hip[3], hub[2]], axis=1)
        tcurrent = torch.cat([thip[3], thub[2]], dim=1)
