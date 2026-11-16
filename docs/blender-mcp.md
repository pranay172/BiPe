# Blender MCP setup and verification

Configured 1 October 2026 using [Blender's official Lab addon](https://www.blender.org/lab/mcp-server/)
and the [official server source](https://projects.blender.org/lab/blender_mcp).
Addon 1.0.3 archive SHA-256 is
`a7a9da816192502e5a0a202a396444e266b47d8fc4f74ad4698048bd43040707`.
Server 1.0.2 is built from revision
`dbbf836ad4b1025f14a2b3b504c43903f39e0b04`; its wheel includes upstream API/manual
resources. Exact dependencies are in `requirements-blender-mcp.lock`.

## Installed configuration

The two empty Lab repository caches contained the same MCP index. The preferences
had not saved a Lab entry. Setup saved **one** Lab repository, installed and
enabled `bl_ext.BlenderLab.mcp`, and removed the redundant empty `_001` cache.
The previous preferences are backed up beside `userpref.blend` as
`userpref.blend.bipe-mcp-backup`; no personal preference file is included in Git
or the release. Existing addon preferences were preserved.

The addon binds **127.0.0.1:9877**, with GUI autostart enabled. Port 9876 was already
occupied and did not answer the official null-delimited protocol. Its listener
was not stopped. Codex's enabled `blender` entry launches `tools/blender-mcp.sh`.
The server runs on demand over stdio and connects to the addon; a separate
always-running stdio terminal is unnecessary. Codex registration follows
[official MCP configuration](https://learn.chatgpt.com/docs/extend/mcp).

New tools may require a Codex MCP reload/restart to appear in an existing chat.
This chat has nevertheless exercised the real server using an MCP ClientSession,
including initialize, list-tools, execute, save and cleanup.

## Restore and operate

From the project directory:

```bash
uv venv .venv/blender-mcp
uv pip install --python .venv/blender-mcp/bin/python -r requirements-blender-mcp.lock
# the server itself (GPL-3.0, not redistributed here), built from the pinned revision:
git clone https://projects.blender.org/lab/blender_mcp.git local/blender_mcp
git -C local/blender_mcp checkout dbbf836ad4b1025f14a2b3b504c43903f39e0b04
uv pip install --python .venv/blender-mcp/bin/python --no-deps local/blender_mcp
codex mcp add blender -- /absolute/path/to/bipe/tools/blender-mcp.sh
```

Install the official addon (1.0.3, SHA-256 above) from Blender's Lab repository and
save a copy of its ZIP as `artifacts/tooling/blender-mcp/addon-1.0.3.zip`, which
`tools/verify_blender_mcp.py` checks; then
enable it; set its host/port to 127.0.0.1/9877. Normal GUI startup uses autostart.
For a fresh owned background session, which is the current stable delivery path:

```bash
blender --background --factory-startup --python tools/start_blender_bridge.py
```

Stop that background session with Ctrl+C before opening a GUI bridge on the same
port. Only one addon listener can own the port. The initialization script reads
preferences and explicitly enables the official addon; it does not load a user's
working scene. This server/addon is a Blender API bridge, separate from FreeCAD's
bridge on port 9875.

```bash
.venv/bin/python tools/verify_blender_mcp.py
.venv/bin/python tools/call_blender.py path/to/trusted_project_script.py
```

Returned data must be assigned to `result`. Calls execute within Blender and
should operate only on owned documents/objects. The verification creates an
owned scene with a measured 0.1 x 0.2 x 0.3 m cuboid, saves it, then restores the
initial object/scene/file state. Fresh native reopen confirms those dimensions
within 1.2e-8 m. There are **26 callable tools**; the verification writes its reports
to `artifacts/tooling/blender-mcp/` (gitignored, not published).

## Observed limitation and workaround

The GUI round-trip succeeded. A later `bpy.data.libraries.write` call saving two
large, newly generated presentation scenes crashed Blender 5.2.2 inside that
native save operation; the retained backtrace identifies the call. This does
not establish an upstream bug diagnosis. The supported background bridge and
`bpy.ops.wm.save_as_mainfile(copy=True)` succeeded. M5 uses a fresh owned session,
regular save-copy, native reopen and background rendering. It preserves the
original default scene and never overwrites a user's loaded project. GUI bulk
library saving remains an observed limitation; the verified MCP/delivery path
has no outstanding connection blocker.
