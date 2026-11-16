# Robot design workstation assessment

Updated on 30 September 2026 after CAD export, MuJoCo physics, plotting, GPU rendering, and interactive viewer checks. This computer supports the current single-robot CAD and controller workflow.

## Observed system

| Item | Observation |
| --- | --- |
| OS | Ubuntu 24.04.5 LTS, x86-64 |
| CPU | AMD Ryzen 7 260 with Radeon 780M; 16 logical processors visible |
| RAM | 22.6 GiB visible to Linux; approximately 12.45 GiB available at inspection |
| Swap | 8.0 GiB |
| Project filesystem | Approximately 479.5 GiB total, 130.1 GiB free |
| Graphics devices | NVIDIA PCI device 10de:2d98 and AMD Phoenix3 display device 1002:1900 detected |
| NVIDIA GPU | GeForce RTX 5050 Laptop GPU; 8151 MiB VRAM; driver 595.91.07 verified by nvidia-smi. |

The first NVIDIA query failed inside a restricted process that did not expose GPU device nodes. Repeating it with device access succeeded. Another workload using the GPU was not established as the cause. No drivers or system settings were changed.

Standard MuJoCo physics runs on the CPU. The verified EGL renderer reports NVIDIA as its GL vendor, the RTX 5050 as its renderer, and OpenGL 4.6 with NVIDIA 595.91.07. GPU rendering succeeded for the physics smoke test and the 601-frame fixed-height video. The articulated Milestone 2 video also rendered successfully: 1081 frames at 1280 × 720 and 30 fps.

## Observed tools

| Tool | Status |
| --- | --- |
| FreeCAD | Runtime 1.1.1; initial and articulated FCStd/STEP checks passed. GUI parametric modeling, inspection, save/reopen, STEP round-trip, and screenshots verified through the project MCP launcher. |
| Blender | Runtime 5.2.2 LTS; factory startup in background mode, Python API access, and clean exit verified. GUI and MCP integration untested. |
| Python / uv | Python 3.12.3; uv environment at .venv/ in the repository. |
| Python dependencies | MuJoCo 3.14.0, NumPy 2.5.3, SciPy 1.18.1, Matplotlib 3.11.2, pytest 9.1.1, Ruff 0.16.9 installed locally. Exact distributions are pinned in requirements.lock. |
| Development tools | Git, CMake, and GCC available on PATH. |
| Electronics and integration | KiCad, ngspice, and ros2 not found on PATH; this does not exclude installations in other environments. |

Blender's runtime reports 5.2.2 LTS; background startup and Python API access were verified on 30 September 2026. The Blender check was saved on the workstation (`artifacts/tooling/`, not published). FreeCAD's version was read from the executed runtime. Python package versions were read from the installed project environment.

## Verified workflow

The project-local environment stepped a falling cube and passed the fixed-height robot acceptance suite. Matplotlib figures were generated and inspected. FreeCAD exported the layout model and verified a 20 × 30 × 40 mm test solid round-trip.

The live MuJoCo viewer opened through NVIDIA PRIME offload, stepped four seconds of robot physics, and exited with code zero. An initial shutdown fault was resolved by waiting for the passive viewer's rendering thread before Python terminates GLFW. Opening, stepping, and closing were tested; keyboard manoeuvres have not been separately exercised through UI automation.

The final articulated viewer also completed its full scripted manoeuvre for 37 seconds and closed with exit code zero. Its EGL renderer reports NVIDIA RTX 5050 and OpenGL 4.6; no driver or system configuration changes were needed. The Milestone 2 GPU, viewer and video check records were machine-specific and are not in the public tree.

See [requirements.lock](../requirements.lock) and [README commands](../README.md). The Milestone 0 environment and CAD check records were machine-specific and are not in the public tree.

## Remaining setup

Install KiCad and circuit simulation dependencies when electrical design starts. Reassess ROS 2 for later sensor and navigation integration. The current balance experiment runs directly in Python.

When Codex launches GPU, X11, or snap commands under its restricted process, device/display access may require a sandbox exception. The tested tools run with that access; this is separate from installing software or modifying system configuration.

## Milestone 3 tooling readiness

FreeCAD scripting, MuJoCo, Python, NVIDIA rendering, and the live viewer are available; Blender background scripting also starts successfully. No software blocker has been established for actuator research and mechanical CAD. Native-app click automation is disabled in the current assistant session. FreeCAD GUI operations have now been verified through MCP stdio against the installed addon; mouse/keyboard editing remains untested.

The user installed the [Robust MCP Bridge Workbench](https://wiki.freecad.org/Robust_MCP_Bridge_Workbench/en). Its local server is now installed in `.venv` and registered with Codex as `freecad`. The accepted MCP stdio run discovered 83 tools and verified a constrained sketch, thickness change, inspection, native save/reopen, STEP geometry, and an 800 × 600 screenshot. The restarted session exposes the native FreeCAD tools, used successfully for M3 assembly and inspection. See [setup and compatibility fixes](freecad-mcp.md) and verification results (saved on the workstation, not published). No Blender, KiCad, or MuJoCo MCP integration has been configured.

The bridge has two components: an addon inside the FreeCAD snap and a separate MCP server. XML-RPC at `127.0.0.1:9875` was verified with Python 3.12.3; this is the addon endpoint, not an MCP HTTP endpoint. The published server's SDK and FreeCAD API incompatibilities were corrected in the versioned project launcher. Its tested workflow works without changing the addon or importing FreeCAD into an external Python process. Blender's [Lab MCP integration](https://www.blender.org/lab/mcp-server/) remains an optional, untested candidate. Direct application APIs also remain available.

Codex supports local MCP servers and project configuration; see [official MCP documentation](https://learn.chatgpt.com/docs/extend/mcp). Evaluate a CAD bridge by checking document inspection, creation, recompute, geometric queries, saving/reopening, and STEP export before using it on project assemblies. Keep Python environments inside `bipe` and preserve the verified simulation dependencies.

The user initialized the Git repository and requested regular commits. The completed Milestones 0–2 are saved as the initial baseline, and subsequent implementation checkpoints will be committed after appropriate verification. The current phase is entirely digital, with budget-friendly, widely used components as design references and outsourced fabrication assumed. Runtime will be decided after actuator and battery sizing; an exact procurement budget or purchasing region is not a blocker for the digital work. M3 tested the old model at 2–20 ms response, and its new component model passes at 20 ms nominal with a separate 25 ms wheel-lag variation. Hardware bandwidth remains unmeasured. NVIDIA replay and the native bridge operate successfully; KiCad is needed when M4 electrical work starts.
