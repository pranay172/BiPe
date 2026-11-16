"""Initialize only the official MCP addon in a fresh Blender GUI session."""

import bpy

bpy.ops.wm.read_userpref()
repo = next(
    r
    for r in bpy.context.preferences.extensions.repos
    if r.remote_url.rstrip("/") == "https://lab.blender.org"
)
module = "bl_ext." + repo.module + ".mcp"
bpy.ops.preferences.addon_enable(module=module)
prefs = bpy.context.preferences.addons[module].preferences
prefs.host = "127.0.0.1"
prefs.port = 9877
bpy.ops.wm.save_userpref()
if bpy.app.background:
    import importlib

    importlib.import_module(module + ".cli").cli_execute(["--host", "127.0.0.1", "--port", "9877"])
else:
    bpy.ops.blmcp.server_start()
