"""Run Python inside the user's Blender through the Blender Lab MCP add-on socket.

python3 tools/blender_bridge.py cad/m10/render_blender.py [--arg stage=exterior] [--port 9877]
python3 tools/blender_bridge.py --code "result = {'version': bpy.app.version_string}"

Used when snap Blender cannot start from the agent's shell. The add-on
executes each request on Blender's main thread and replies once with the
code's ``result`` dict. Port 9877 avoids FreeCAD's bridge on 9875/9876.
"""

import argparse
import json
import socket
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SCRIPT = """
import sys
sys.path.insert(0, {root!r})
for _m in [m for m in list(sys.modules) if m == "bipe" or m.startswith("bipe.")]:
    del sys.modules[_m]
_ns = {{"__file__": {path!r}, "__name__": "bridge_main", "BRIDGE_ARGS": {args!r}}}
exec(compile(open({path!r}).read(), {path!r}, "exec"), _ns)
result = _ns.get("result", {{}})
"""


def call(code, port=9877, host="127.0.0.1", timeout=900.0):
    request = {"type": "execute", "code": code, "strict_json": True}
    with socket.create_connection((host, port), timeout=timeout) as conn:
        conn.sendall((json.dumps(request) + "\0").encode())
        data = b""
        while not data.endswith(b"\0"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
    return json.loads(data.rstrip(b"\0"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("script", nargs="?")
    parser.add_argument("--code")
    parser.add_argument("--arg", action="append", default=[], help="KEY=VALUE for BRIDGE_ARGS")
    parser.add_argument("--port", type=int, default=9877)
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    if args.code:
        code = args.code
    elif args.script:
        path = (ROOT / args.script).resolve()
        bridge_args = dict(item.split("=", 1) for item in args.arg)
        code = SCRIPT.format(root=str(ROOT), path=str(path), args=bridge_args)
    else:
        parser.error("give a script path or --code")
    reply = call(code, port=args.port, timeout=args.timeout)
    print(json.dumps(reply, indent=2))
    raise SystemExit(0 if reply.get("status") == "ok" else 1)


if __name__ == "__main__":
    main()
