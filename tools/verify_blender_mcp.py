"""Real MCP round-trip, owned-scene mutation, native save and cleanup."""

import asyncio
import hashlib
import json
from datetime import timedelta
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts/tooling/blender-mcp"


async def verify():
    report = {"passed": False, "transport": "stdio", "bridge": "127.0.0.1:9877"}
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT / "verification.log").open("w") as log:
        async with (
            stdio_client(
                StdioServerParameters(command=str(ROOT / "tools/blender-mcp.sh")), errlog=log
            ) as streams,
            ClientSession(*streams, read_timeout_seconds=timedelta(seconds=45)) as session,
        ):
            report["server"] = (await session.initialize()).serverInfo.model_dump()
            listing = await session.list_tools()
            report["tools"] = [t.name for t in listing.tools]
            (OUT / "tools.json").write_text(json.dumps(listing.model_dump(), indent=2) + "\n")
            code = """
import bpy,json
from pathlib import Path
before={'objects':sorted(bpy.data.objects.keys()),'scenes':sorted(bpy.data.scenes.keys()),
        'filepath':bpy.data.filepath,'active_scene':bpy.context.scene.name}
assert 'BipeMCPVerification' not in bpy.data.scenes
scene=bpy.data.scenes.new('BipeMCPVerification')
mesh=bpy.data.meshes.new('BipeMCPTestMesh')
verts=[(x,y,z) for x in [-.05,.05] for y in [-.05,.05] for z in [-.05,.05]]
faces=[(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)]
mesh.from_pydata(verts,[],faces)
mesh.update()
obj=bpy.data.objects.new('BipeMCPTestCube',mesh);scene.collection.objects.link(obj)
scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1.0
obj.scale=(1,2,3)
scene.view_layers[0].update()
dimensions=list(obj.dimensions)
assert max(abs(a-b) for a,b in zip(dimensions,[.1,.2,.3]))<1e-7
filepath=Path(PROJECT_ROOT)/'artifacts/tooling/blender-mcp/test-cube.blend'
bpy.data.libraries.write(str(filepath),{scene})
repos=[{'name':r.name,'module':r.module,'remote_url':r.remote_url} for r in bpy.context.preferences.extensions.repos]
prefs=next(a.preferences for a in bpy.context.preferences.addons if a.module.endswith('.mcp'))
bpy.data.objects.remove(obj,do_unlink=True);bpy.data.meshes.remove(mesh);bpy.data.scenes.remove(scene)
after={'objects':sorted(bpy.data.objects.keys()),'scenes':sorted(bpy.data.scenes.keys()),
       'filepath':bpy.data.filepath,'active_scene':bpy.context.scene.name}
result={'version':bpy.app.version_string,'background':bpy.app.background,'before':before,'after':after,
        'original_state_preserved':before==after,'repositories':repos,'host':prefs.host,'port':prefs.port,
        'autostart':prefs.use_autostart,'dimensions_m':dimensions,'native_file':str(filepath)}
"""
            r = await session.call_tool(
                "execute_blender_code", {"code": "PROJECT_ROOT=" + repr(str(ROOT)) + "\n" + code}
            )
            data = r.structuredContent or json.loads(r.content[0].text)
            assert not r.isError and data["status"] == "ok", data
            report["roundtrip"] = data["result"]
            report["addon_sha256"] = hashlib.sha256(
                (OUT / "addon-1.0.3.zip").read_bytes()
            ).hexdigest()
            assert (
                report["addon_sha256"]
                == "a7a9da816192502e5a0a202a396444e266b47d8fc4f74ad4698048bd43040707"
            )
            report["source_revision"] = "dbbf836ad4b1025f14a2b3b504c43903f39e0b04"
            report["native_sha256"] = hashlib.sha256(
                (OUT / "test-cube.blend").read_bytes()
            ).hexdigest()
            report["passed"] = (
                report["roundtrip"]["original_state_preserved"]
                and len(
                    [
                        r
                        for r in report["roundtrip"]["repositories"]
                        if r["remote_url"].rstrip("/") == "https://lab.blender.org"
                    ]
                )
                == 1
                and report["roundtrip"]["host"] == "127.0.0.1"
                and report["roundtrip"]["port"] == 9877
                and len(listing.tools) == 26
            )
    (OUT / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    asyncio.run(verify())
