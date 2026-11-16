"""M12 camera following: requirements, scenarios, perception and the following controller."""

import os

# Headless rendering on the GPU (EGL); MuJoCo reads this when it is first imported.
os.environ.setdefault("MUJOCO_GL", "egl")
