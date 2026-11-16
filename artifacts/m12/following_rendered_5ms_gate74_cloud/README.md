# Phase F view-gate trial (cloud, 23–24 October 2026)

The phase F matrix runs with a detection past 60° of yaw view angle (12
of 112), rerun with `view_gate_deg` 74 (`bipe.m12.rendered_link`). Also
the no-gate baseline for corner_90 seeds 1–4 (`baseline_no_gate/`) and
the 65° gate on corner_90 seeds 0–4 and curve_r1 seed 0
(`gate65_runs.json`). Everything else is as in
`../following_rendered_5ms/` (5 ms exposure, phase C tracker).

The runs used MuJoCo on OSMesa, the cloud's software renderer, not the
laptop's EGL. The frames differ slightly, so compare gate against no
gate within this directory, not against `../following_rendered_5ms/`.
Made with a scratch driver over `bipe.m12.following.closed_task`. The
records are `closed_metrics` with `perception`, plus `gated_frames` as
(t, fitted 3-D view angle, yaw view angle).
