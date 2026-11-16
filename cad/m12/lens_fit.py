"""M12 phase C: does the Camera Module 3 Wide (12.4 mm deep) fit the head?

/snap/bin/freecad.cmd -c "p='cad/m12/lens_fit.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds the M10 head pod and its internals (cad/m10/assembly.head_internals, hollowed as
the assembly hollows it) with the camera module envelope at the standard depth (11.5 mm)
and the Wide depth (12.4 mm, Raspberry Pi documentation), at the current 15 degree
camera pitch and at the 42 degree pitch of the waist-tag layout. The extra depth of the
Wide grows the module backwards from its fixed lens face (the board moves back). For each
case it reports the volume of the camera board and lens block inside the head shell or the
eye LED boards (and the bezel, neck and head screws), and the smallest gap to each. The
42 degree case is information for the phase E head change: the bezel and seat are
re-pitched with the module, as the CAD does today at 15 degrees.

Results in artifacts/m12/perception/lens_fit.json (mm, mm^3).
"""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

_spec = importlib.util.spec_from_file_location("m10_assembly", ROOT / "cad/m10/assembly.py")
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
E = A.E
OUT = ROOT / "artifacts/m12/perception/lens_fit.json"
SHARED = json.loads((ROOT / "config/robot.json").read_text())
LENSES = SHARED["m12"]["camera"]["lenses"]
PITCHES_DEG = (15, 42)


def head_case(depth_m, pitch_deg):
    # The CAD modules hold references into their config; change those values in place.
    module, xhead = A.INT["head"]["camera_module"], E.X["head"]
    original = (module[2], xhead["camera_pitch_deg"])
    try:
        module[2] = depth_m
        xhead["camera_pitch_deg"] = pitch_deg
        exterior = {}
        for _, name, shape, _ in E.head():
            exterior[name] = exterior[name].fuse(shape) if name in exterior else shape
        parts, cuts = A.head_internals()
        shell = exterior["HeadPod"]
        for target, tool in cuts:
            if target == "HeadPod":
                shell = shell.cut(tool)
        named = {name: shape for _, name, shape, _ in parts}
        camera = [named["CameraBoard"], named["CameraLensBlock"]]
        others = {"HeadShell": shell}
        for name in ("CameraBezel", "Neck", "HeadScrew"):
            others[name] = exterior[name]
        for _, name, shape, _ in parts:
            if name.startswith("EyeLEDBoard"):
                others.setdefault("EyeLEDBoards", shape)
                others["EyeLEDBoards"] = others["EyeLEDBoards"].fuse(shape)
        out = {"module_depth_mm": round(depth_m * 1e3, 2), "camera_pitch_deg": pitch_deg}
        for name, other in others.items():
            common = [c.common(other) for c in camera]
            overlap = sum(x.Volume for x in common)
            gap = min(c.distToShape(other)[0] for c in camera)
            out[name] = {"overlap_mm3": round(overlap, 3), "gap_mm": round(gap, 3)}
            if overlap > 1e-3:
                box = common[0].fuse(common[1]).BoundBox
                # Hip frame (mm): x forward, z up.
                out[name]["overlap_box_mm"] = [
                    round(v, 2)
                    for v in (box.XMin, box.XMax, box.YMin, box.YMax, box.ZMin, box.ZMax)
                ]
                hit = [
                    n for n, x in zip(("CameraBoard", "CameraLensBlock"), common) if x.Volume > 1e-3
                ]
                out[name]["parts"] = hit
        out["fits"] = all(v["overlap_mm3"] <= 1e-3 for k, v in out.items() if isinstance(v, dict))
        return out
    finally:
        module[2], xhead["camera_pitch_deg"] = original


def main():
    rows = []
    for name, lens in LENSES.items():
        for pitch in PITCHES_DEG:
            row = head_case(lens["module_depth_m"], pitch)
            row["lens"] = name
            rows.append(row)
            print("M12 LENS FIT", json.dumps(row), flush=True)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        json.dumps(
            {
                "status": "layout study: CAD envelopes from config/robot.json m10 (the lens block "
                "and its offset are assumptions until checked against the Camera Module 3 "
                "mechanical drawing); module depths from the Raspberry Pi documentation",
                "method": __doc__.split("\n\n")[2].replace("\n", " "),
                "cases": rows,
            },
            indent=1,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
