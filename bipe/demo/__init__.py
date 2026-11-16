"""Visual demo: record full-plant simulations and render them to video.

Recording runs the same full plant, sensor model and C firmware as the acceptance
runners (bipe.moving_experiment.simulate), with the classical controller or a policy
checkpoint. Rendering replays the recorded poses in a visual-only copy of the model with
the CAD render meshes. See docs/demo/plan.md.
"""
