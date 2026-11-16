"""python -m bipe.demo record|render|scene NAME ... (see bipe.demo.record and .render)."""

import argparse

from .scenes import SCENES


def main():
    ap = argparse.ArgumentParser(prog="python -m bipe.demo")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for cmd in ("record", "scene"):
        s = sub.add_parser(cmd)
        s.add_argument("scene", choices=sorted(SCENES))
        s.add_argument("--controller", default=None, help="label for the output name")
        s.add_argument("--checkpoint", help="policy .pt (needs torch)")
        s.add_argument("--onnx", help="exported policy graph")
        s.add_argument("--output")
        if cmd == "scene":
            s.add_argument("--size", default="1280x720")
    r = sub.add_parser("render")
    r.add_argument("npz")
    r.add_argument("--output")
    r.add_argument("--size", default="1280x720")
    r.add_argument("--preview-every", type=int, default=0)
    a = ap.parse_args()
    if a.cmd in ("record", "scene"):
        from .record import record

        controller = a.controller or ("policy" if (a.checkpoint or a.onnx) else "classical")
        out, meta = record(a.scene, controller, a.checkpoint, a.onnx, a.output)
        print(out, {k: meta[k] for k in ("failure", "peak_pitch_deg", "wall_s")})
        if a.cmd == "scene":
            from .render import render

            print(render(out, size=tuple(int(v) for v in a.size.split("x"))))
    else:
        from .render import render

        size = tuple(int(v) for v in a.size.split("x"))
        print(render(a.npz, a.output, size, preview_every=a.preview_every))


if __name__ == "__main__":
    main()
