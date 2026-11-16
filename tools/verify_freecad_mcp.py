"""Exercise the installed server through MCP stdio against a running GUI bridge."""

import asyncio
import base64
import hashlib
import json
from datetime import UTC, datetime, timedelta
from importlib.metadata import version
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts/tooling/freecad-mcp"


async def verify():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    report = {
        "timestamp_utc": datetime.now(UTC).isoformat(),
        "passed": False,
        "packages": {name: version(name) for name in ("freecad-robust-mcp", "mcp")},
        "transport": "stdio",
        "freecad_connection": "xmlrpc://127.0.0.1:9875",
        "calls": [],
    }
    parameters = StdioServerParameters(command=str(ROOT / "tools/freecad-mcp.sh"), cwd=ROOT)
    try:
        with (OUTPUT / "verification-stderr.log").open("w") as log:
            async with (
                stdio_client(parameters, errlog=log) as streams,
                ClientSession(*streams, read_timeout_seconds=timedelta(seconds=45)) as session,
            ):

                async def call(tool_name, **arguments):
                    result = await session.call_tool(tool_name, arguments)
                    if result.isError:
                        raise RuntimeError(f"{tool_name}: {result.content}")
                    data = result.structuredContent
                    if data is None:
                        texts = [c.text for c in result.content if c.type == "text"]
                        data = json.loads(texts[0])
                    if isinstance(data, dict) and data.get("success") is False:
                        raise RuntimeError(f"{tool_name}: {data}")
                    # Screenshots are saved separately; keep base64 out of the report.
                    summary = (
                        {k: v for k, v in data.items() if k != "data"}
                        if isinstance(data, dict)
                        else data
                    )
                    report["calls"].append(
                        {"tool": tool_name, "arguments": arguments, "result": summary}
                    )
                    return data

                init = await session.initialize()
                report["server"] = init.serverInfo.model_dump(mode="json")
                listed = await session.list_tools()
                report["tool_count"] = len(listed.tools)
                (OUTPUT / "tools.json").write_text(
                    json.dumps(listed.model_dump(mode="json"), indent=2) + "\n"
                )
                report["connection"] = await call("get_connection_status")
                report["freecad"] = await call("get_freecad_version")
                original = await call(
                    "execute_python",
                    code=(
                        "_result_ = {'documents': list(FreeCAD.listDocuments()), "
                        "'active': FreeCAD.ActiveDocument.Name if FreeCAD.ActiveDocument "
                        "else None}"
                    ),
                )
                original = original["result"]
                doc_name = "BipeMCPVerification"
                if doc_name in original["documents"]:
                    raise RuntimeError("Verification document already open; choose another name")
                owned = []
                try:
                    created = await call("create_document", name=doc_name)
                    doc_name = created["name"]
                    owned.append(doc_name)
                    await call("create_partdesign_body", name="TestBody", doc_name=doc_name)
                    await call(
                        "create_sketch", body_name="TestBody", name="Profile", doc_name=doc_name
                    )
                    await call(
                        "add_sketch_rectangle",
                        sketch_name="Profile",
                        x=0,
                        y=0,
                        width=20,
                        height=30,
                        doc_name=doc_name,
                    )
                    constrained = await call(
                        "execute_python",
                        code=f"""
import Sketcher
doc = FreeCAD.getDocument({doc_name!r})
s = doc.getObject('Profile')
for kind, edge in [('Horizontal', 0), ('Vertical', 1),
                   ('Horizontal', 2), ('Vertical', 3)]:
    s.addConstraint(Sketcher.Constraint(kind, edge))
s.addConstraint(Sketcher.Constraint('Distance', 0, 20.0))
s.addConstraint(Sketcher.Constraint('Distance', 1, 30.0))
s.addConstraint(Sketcher.Constraint('Coincident', 0, 1, -1, 1))
doc.recompute()
_result_ = {{'fully_constrained': s.FullyConstrained}}
""",
                    )
                    assert constrained["result"]["fully_constrained"]
                    await call(
                        "pad_sketch",
                        sketch_name="Profile",
                        length=4,
                        name="TestPad",
                        doc_name=doc_name,
                    )
                    await call("recompute_document", doc_name=doc_name)
                    await call("inspect_object", object_name="TestPad", doc_name=doc_name)
                    modified = await call(
                        "execute_python",
                        code=f"""
doc = FreeCAD.getDocument({doc_name!r})
pad = doc.getObject('TestPad')
before = pad.Shape.Volume
pad.Length = 6.0
doc.recompute()
_result_ = {{'volume_before_mm3': before, 'volume_after_mm3': pad.Shape.Volume}}
""",
                    )
                    assert abs(modified["result"]["volume_before_mm3"] - 2400) < 1e-6
                    assert abs(modified["result"]["volume_after_mm3"] - 3600) < 1e-6
                    fcstd = OUTPUT / "test-part.FCStd"
                    step = OUTPUT / "test-part.step"
                    await call("save_document", doc_name=doc_name, path=str(fcstd))
                    await call(
                        "export_step",
                        doc_name=doc_name,
                        object_names=["TestPad"],
                        file_path=str(step),
                    )
                    await call("fit_all", doc_name=doc_name)
                    shot = await call(
                        "get_screenshot",
                        doc_name=doc_name,
                        view_angle="Isometric",
                        width=800,
                        height=600,
                    )
                    png = OUTPUT / "test-part.png"
                    png.write_bytes(base64.b64decode(shot["data"], validate=True))
                    with Image.open(png) as image:
                        image.load()
                        report["screenshot_size"] = list(image.size)
                        assert image.size == (800, 600)
                    await call("close_document", doc_name=doc_name)
                    owned.remove(doc_name)
                    reopened = await call("open_document", path=str(fcstd))
                    doc_name = reopened["name"]
                    owned.append(doc_name)
                    geometry = await call(
                        "execute_python",
                        code=f"""
import Part
doc = FreeCAD.getDocument({doc_name!r})
native = doc.getObject('TestPad').Shape
exported = Part.read({str(step)!r})
def describe(shape):
    b = shape.BoundBox
    return {{'valid': shape.isValid(), 'solids': len(shape.Solids),
             'volume_mm3': shape.Volume,
             'dimensions_mm': [b.XLength, b.YLength, b.ZLength]}}
_result_ = {{'native': describe(native), 'step': describe(exported),
             'sketch_fully_constrained': doc.getObject('Profile').FullyConstrained,
             'pad_length_mm': doc.getObject('TestPad').Length.Value}}
""",
                    )
                    report["geometry"] = geometry["result"]
                    assert report["geometry"]["sketch_fully_constrained"]
                    assert report["geometry"]["pad_length_mm"] == 6
                    for shape in (report["geometry"]["native"], report["geometry"]["step"]):
                        assert shape["valid"] and shape["solids"] == 1
                        assert abs(shape["volume_mm3"] - 3600) < 1e-6
                        assert all(
                            abs(a - b) < 1e-6
                            for a, b in zip(shape["dimensions_mm"], [20, 30, 6], strict=True)
                        )
                finally:
                    for name in owned:
                        await call("close_document", doc_name=name)
                    await call(
                        "execute_python",
                        code=(f"FreeCAD.setActiveDocument({original['active'] or ''!r})"),
                    )
                after = await call(
                    "execute_python", code=("_result_ = list(FreeCAD.listDocuments())")
                )
                assert after["result"] == original["documents"]
                report["original_documents_preserved"] = True
                report["artifact_sha256"] = {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (fcstd, step, png)
                }
                report["passed"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        (OUTPUT / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in (
                        "passed",
                        "packages",
                        "tool_count",
                        "geometry",
                        "screenshot_size",
                        "error",
                    )
                    if k in report
                },
                indent=2,
            )
        )


if __name__ == "__main__":
    asyncio.run(verify())
