#!/usr/bin/env bash
set -euo pipefail
bipe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$bipe_root"
export BLENDER_MCP_HOST=127.0.0.1
export BLENDER_MCP_PORT=9877
exec "$bipe_root/.venv/blender-mcp/bin/python" -m blmcp "$@"
