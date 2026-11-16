# Simulation demo video

Everything shown is simulation, not hardware footage.

Each shot is one full-plant run (M10 plant, sensor model, C firmware) recorded at 50 fps
and replayed with the CAD render meshes. Outputs go to `local/demo/` (gitignored).

```sh
export PYTHONPATH=. MUJOCO_GL=egl          # osmesa where EGL is missing (cloud)
python -m bipe.demo record slope           # classical; --onnx/--checkpoint for a policy
python -m bipe.demo render local/demo/slope_classical.npz
python -m bipe.demo.edit grid A.mp4 B.mp4 C.mp4 D.mp4 local/demo/variants_grid.mp4
python -m bipe.demo.edit card artifacts/demo/cards/title_three_quarter.png \
    local/demo/card_title.mp4 --title BiPe --window 0 0.78
python -m bipe.demo.edit cut local/demo/bipe_demo_classical.mp4 SHOT.mp4 ...
```

Scenes are in `bipe/demo/scenes.py`. Card stills come from Blender on the laptop:
`python3 tools/blender_bridge.py cad/m10/render_blender.py --arg stage=assembly
--arg views=docs/demo/card-views.json --arg out=artifacts/demo/cards --arg resolution=1920x1080`.

The 3 November 2026 cut (77 s) uses the classical controller. Under the
`delay_15ms_noise_x2` variant a 1.5 N s push takes the classical controller past the
20 degree pitch limit, so that tile holds without pushes. Learned-policy shots
(learned vs classical, moves, push recovery) were recorded in M15 with the same
pipeline; the release cut that combines them is still to be made.
