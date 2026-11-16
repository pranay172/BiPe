# Project working instructions

- Keep the uv virtual environment inside this project, at `.venv/`.
- Make local Git commits at meaningful implementation and verification checkpoints. Keep commits focused and record the validation performed.
- Create the detailed plan for a milestone immediately before implementing that milestone. Keep future milestones in the overview until they start. Record achieved results separately from proposed requirements.
- Use `config/robot.json` as the shared source for SI dimensions, mass assumptions, and physical limits. Convert to millimetres only at the FreeCAD boundary.
- The agreed initial scope is a hardware-oriented digital prototype that balances, drives, turns, and crouches. Camera following and jumping are later work.
- Preserve the distinction between a layout study, a validated simulation under stated assumptions, and a hardware-validated design. Select real parts using manufacturer documentation and computed loads.
- Balance acceptance requires a free base, finite motor limits, and physical contact. Do not stabilize a model through hidden pose constraints or state clamps.
- Save configurations and measured traces with experiment outputs. Run the milestone acceptance suite after physical model or controller changes that can affect its results.
