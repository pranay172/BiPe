#!/usr/bin/env bash
# Codex starts this stdio server on demand; FreeCAD's addon must be running.
set -euo pipefail
bipe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$bipe_root"
exec "$bipe_root/.venv/bin/python" "$bipe_root/tools/freecad_mcp_server.py" \
    --mode xmlrpc --transport stdio --host 127.0.0.1 --port 9875 "$@"
