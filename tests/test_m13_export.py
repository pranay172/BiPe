import pytest

torch = pytest.importorskip("torch")


def test_export_matches_actor(tmp_path):
    """ONNX graph and C forward pass from a fresh export agree with the numpy actor."""
    import subprocess

    import numpy as np

    from bipe.m13 import export
    from bipe.m13.policy_control import Actor, CActor

    ort = pytest.importorskip("onnxruntime")
    actor = Actor(export.CHECKPOINT)
    obs = np.random.default_rng(1).normal(size=(64, 25)) * actor.std + actor.mean
    ref = np.array([actor(o) for o in obs])

    export.export_onnx(export.CHECKPOINT, tmp_path / "policy.onnx")
    session = ort.InferenceSession(
        str(tmp_path / "policy.onnx"), providers=["CPUExecutionProvider"]
    )
    onnx = np.concatenate(
        [session.run(None, {"observation": o[None]})[0] for o in obs.astype(np.float32)]
    )
    assert np.abs(onnx - ref).max() < 1e-4

    (tmp_path / "include").mkdir()
    export.weights_header(export.CHECKPOINT, tmp_path / "include/generated_locomotion_weights.h")
    library = tmp_path / "liblocomotion.so"
    firmware = export.ROOT / "firmware"
    subprocess.run(
        ["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-Wshadow",
         f"-I{tmp_path / 'include'}", f"-I{firmware / 'include'}", "-fPIC", "-shared",
         str(firmware / "src/locomotion/locomotion.c"), "-lm", "-o", str(library)],
        check=True,
    )  # fmt: skip
    c = CActor(library)
    assert c.checkpoint_sha256 == export.sha256(export.CHECKPOINT)
    assert np.abs(np.array([c(o) for o in obs]) - ref).max() < 1e-4
