"""Launch Robust MCP 0.6.1 with sketch fixes for FreeCAD 1.x.

FreeCAD renamed Support to AttachmentSupport. Replace the affected tool through
the MCP SDK's public registration API, leaving the installed package unchanged.
The upstream rectangle tool also omits its required Sketcher import.
The pad tool uses the nonexistent Symmetric property instead of SideType/Midplane.
Nested FreeCAD objects in inspection results need conversion for XML-RPC.
Screenshot detection must use view capabilities, not its dynamic __class__ attribute.
"""

from importlib.metadata import version
from typing import Any

from freecad_mcp import server
from freecad_mcp.bridge import xmlrpc as bridge_xmlrpc
from freecad_mcp.bridge.base import ExecutionResult, ScreenshotResult, ViewAngle


class CompatibleXmlRpcBridge(bridge_xmlrpc.XmlRpcBridge):
    async def execute_python(self, code: str, timeout_ms: int = 30000) -> ExecutionResult:
        # Inspection returns link properties in lists/tuples. The upstream bridge
        # only converts top-level FreeCAD objects; XML-RPC cannot marshal nested
        # objects. Normalize the returned value while still inside FreeCAD.
        code += """
import json as _bipe_mcp_json
if '_result_' in globals():
    _result_ = _bipe_mcp_json.loads(_bipe_mcp_json.dumps(_result_, default=str))
"""
        return await super().execute_python(code, timeout_ms)

    async def get_screenshot(
        self,
        view_angle: ViewAngle | None = None,
        width: int = 800,
        height: int = 600,
        doc_name: str | None = None,
    ) -> ScreenshotResult:
        angle = view_angle or ViewAngle.ISOMETRIC
        methods = {
            ViewAngle.ISOMETRIC: "viewIsometric",
            ViewAngle.FRONT: "viewFront",
            ViewAngle.BACK: "viewRear",
            ViewAngle.TOP: "viewTop",
            ViewAngle.BOTTOM: "viewBottom",
            ViewAngle.LEFT: "viewLeft",
            ViewAngle.RIGHT: "viewRight",
            ViewAngle.FIT_ALL: "fitAll",
        }
        result = await self.execute_python(f"""
import base64, os, tempfile
doc = FreeCAD.ActiveDocument if {doc_name!r} is None else FreeCAD.getDocument({doc_name!r})
if doc is None or not FreeCAD.GuiUp:
    raise RuntimeError('A GUI document is required for screenshots')
view = FreeCADGui.getDocument(doc.Name).activeView()
if not hasattr(view, 'saveImage'):
    raise RuntimeError('The active document view cannot capture a 3D image')
getattr(view, {methods[angle]!r})()
view.fitAll()
FreeCADGui.updateGui()
with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as image:
    path = image.name
try:
    view.saveImage(path, {int(width)}, {int(height)}, 'Current')
    with open(path, 'rb') as image:
        data = base64.b64encode(image.read()).decode('ascii')
finally:
    os.unlink(path)
_result_ = {{'data': data}}
""")
        return ScreenshotResult(
            success=result.success,
            data=result.result["data"] if result.success else None,
            format="png",
            width=width,
            height=height,
            view_angle=angle,
            error=None if result.success else result.error_traceback or result.stderr,
        )


async def create_sketch(
    body_name: str | None = None,
    plane: str = "XY_Plane",
    name: str | None = None,
    doc_name: str | None = None,
) -> dict[str, Any]:
    """Create a sketch on XY/XZ/YZ or a body face, using the current attachment API."""
    bridge = await server.get_bridge()
    result = await bridge.execute_python(f"""
doc = FreeCAD.ActiveDocument if {doc_name!r} is None else FreeCAD.getDocument({doc_name!r})
if doc is None:
    doc = FreeCAD.newDocument('Unnamed')
body_name, plane = {body_name!r}, {plane!r}
sketch_name = {name!r} or 'Sketch'
if body_name:
    body = doc.getObject(body_name)
    if body is None:
        raise ValueError('Body not found: ' + body_name)
    sketch = body.newObject('Sketcher::SketchObject', sketch_name)
    support_property = ('AttachmentSupport' if 'AttachmentSupport' in sketch.PropertiesList
                        else 'Support')
    if plane in ['XY_Plane', 'XZ_Plane', 'YZ_Plane']:
        support = (body.Origin.getObject(plane), [''])
    elif plane.startswith('Face'):
        support = (body, [plane])
    else:
        raise ValueError('Unsupported sketch plane: ' + plane)
    setattr(sketch, support_property, support)
    sketch.MapMode = 'FlatFace'
else:
    sketch = doc.addObject('Sketcher::SketchObject', sketch_name)
    if plane == 'XZ_Plane':
        sketch.Placement = FreeCAD.Placement(
            FreeCAD.Vector(), FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 90))
    elif plane == 'YZ_Plane':
        sketch.Placement = FreeCAD.Placement(
            FreeCAD.Vector(), FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), 90))
    elif plane != 'XY_Plane':
        raise ValueError('Unsupported standalone sketch plane: ' + plane)
doc.recompute()
_result_ = {{'name': sketch.Name, 'label': sketch.Label, 'type_id': sketch.TypeId,
             'support': str(getattr(sketch, 'AttachmentSupport',
                                    getattr(sketch, 'Support', None)))}}
""")
    if not result.success:
        raise ValueError(result.error_traceback or "Create sketch failed")
    return result.result


async def add_sketch_rectangle(
    sketch_name: str,
    x: float,
    y: float,
    width: float,
    height: float,
    doc_name: str | None = None,
) -> dict[str, Any]:
    """Add four lines with coincident corners; coordinates and dimensions use mm."""
    bridge = await server.get_bridge()
    result = await bridge.execute_python(f"""
import Part, Sketcher
doc = FreeCAD.ActiveDocument if {doc_name!r} is None else FreeCAD.getDocument({doc_name!r})
sketch = doc.getObject({sketch_name!r})
if sketch is None:
    raise ValueError('Sketch not found: ' + {sketch_name!r})
x, y, width, height = {x!r}, {y!r}, {width!r}, {height!r}
points = [FreeCAD.Vector(x, y, 0), FreeCAD.Vector(x + width, y, 0),
          FreeCAD.Vector(x + width, y + height, 0), FreeCAD.Vector(x, y + height, 0)]
start = sketch.GeometryCount
for i in range(4):
    sketch.addGeometry(Part.LineSegment(points[i], points[(i + 1) % 4]), False)
for i in range(4):
    sketch.addConstraint(Sketcher.Constraint('Coincident', start + i, 2,
                                           start + (i + 1) % 4, 1))
doc.recompute()
_result_ = {{'constraint_count': sketch.ConstraintCount,
             'geometry_count': sketch.GeometryCount}}
""")
    if not result.success:
        raise ValueError(result.error_traceback or "Add rectangle failed")
    return result.result


async def pad_sketch(
    sketch_name: str,
    length: float,
    symmetric: bool = False,
    reversed: bool = False,
    name: str | None = None,
    doc_name: str | None = None,
) -> dict[str, Any]:
    """Extrude a body's sketch by a length in mm, optionally symmetric or reversed."""
    bridge = await server.get_bridge()
    result = await bridge.execute_python(f"""
doc = FreeCAD.ActiveDocument if {doc_name!r} is None else FreeCAD.getDocument({doc_name!r})
sketch = doc.getObject({sketch_name!r})
if sketch is None:
    raise ValueError('Sketch not found: ' + {sketch_name!r})
body = next((obj for obj in doc.Objects if obj.TypeId == 'PartDesign::Body'
             and sketch in obj.Group), None)
if body is None:
    raise ValueError('Sketch must belong to a PartDesign body')
pad = body.newObject('PartDesign::Pad', {name!r} or 'Pad')
pad.Profile = sketch
pad.Length = {length!r}
if 'SideType' in pad.PropertiesList:
    pad.SideType = 'Symmetric' if {symmetric!r} else 'One side'
else:
    pad.Midplane = {symmetric!r}
pad.Reversed = {reversed!r}
doc.recompute()
sketch.Visibility = False
_result_ = {{'name': pad.Name, 'label': pad.Label, 'type_id': pad.TypeId}}
""")
    if not result.success:
        raise ValueError(result.error_traceback or "Pad failed")
    return result.result


if __name__ == "__main__":
    if version("freecad-robust-mcp") != "0.6.1":
        raise RuntimeError("Review the sketch compatibility fix before changing server versions")
    bridge_xmlrpc.XmlRpcBridge = CompatibleXmlRpcBridge
    for tool in (create_sketch, add_sketch_rectangle, pad_sketch):
        server.mcp.remove_tool(tool.__name__)
        server.mcp.add_tool(tool)
    server.main()
