"""Print the bounding box (mm) of each solid in a STEP file. Usage: M11_STEP=path freecad.cmd -c ..."""

import os

import Part

shape = Part.Shape()
shape.read(os.environ["M11_STEP"])
b = shape.BoundBox
print(
    "M11 STEP total",
    [round(v, 2) for v in (b.XMin, b.XMax, b.YMin, b.YMax, b.ZMin, b.ZMax)],
    "solids",
    len(shape.Solids),
    flush=True,
)
for i, s in enumerate(shape.Solids):
    c = s.BoundBox
    if s.Volume > 2:
        print(
            "M11 STEP solid",
            i,
            round(s.Volume, 1),
            [round(v, 2) for v in (c.XMin, c.XMax, c.YMin, c.YMax, c.ZMin, c.ZMax)],
            flush=True,
        )
