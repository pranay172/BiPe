"""Run a project FreeCAD script inside the user's FreeCAD GUI through its XML-RPC bridge.

python3 tools/freecad_bridge.py cad/m10/blockout.py [--timeout 900]

Used when snap FreeCAD cannot start from the agent's shell. The bridge
call itself times out after 30 s, so the script is scheduled on the GUI
event loop and this tool waits for a status file written by the wrapper.
"""

import argparse
import json
import time
import xmlrpc.client
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATUS = ROOT / "artifacts/.freecad-bridge-status.json"

WRAPPER = """
import FreeCAD, sys, json, time, traceback
from PySide import QtCore
def _bipe_run(path={path!r}, status={status!r}):
    t0 = time.time()
    for name in [n for n in FreeCAD.listDocuments() if n.startswith("M10")]:
        FreeCAD.closeDocument(name)
    for mod in [m for m in list(sys.modules) if m == "bipe" or m.startswith("bipe.")]:
        del sys.modules[mod]
    try:
        exec(compile(open(path).read(), path, "exec"), {{"__file__": path, "__name__": "bridge_main"}})
        result = {{"ok": True}}
    except Exception:
        result = {{"ok": False, "traceback": traceback.format_exc()}}
    result["seconds"] = round(time.time() - t0, 1)
    open(status, "w").write(json.dumps(result))
QtCore.QTimer.singleShot(0, _bipe_run)
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("script")
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--url", default="http://127.0.0.1:9875")
    args = parser.parse_args()
    path = (ROOT / args.script).resolve()
    STATUS.unlink(missing_ok=True)
    server = xmlrpc.client.ServerProxy(args.url, allow_none=True)
    reply = server.execute(WRAPPER.format(path=str(path), status=str(STATUS)))
    if not reply.get("success"):
        raise SystemExit(f"Bridge refused the job: {reply}")
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if STATUS.exists():
            result = json.loads(STATUS.read_text())
            STATUS.unlink()
            print(json.dumps(result, indent=2))
            raise SystemExit(0 if result["ok"] else 1)
        time.sleep(1)
    raise SystemExit("Timed out waiting for FreeCAD")


if __name__ == "__main__":
    main()
