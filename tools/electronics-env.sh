#!/bin/bash
set -euo pipefail
BIPE_TOOL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.local-tools/root"
# System libraries first: the extracted 24.04 packages only fill in what Ubuntu 26.04 lacks
# (wxWidgets 3.2 and others); an older copy of a library the system also has (libssh2)
# would break the newer system libraries that depend on it.
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$BIPE_TOOL_ROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$BIPE_TOOL_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export KICAD9_SYMBOL_DIR="$BIPE_TOOL_ROOT/usr/share/kicad/symbols"
export KICAD9_FOOTPRINT_DIR="$BIPE_TOOL_ROOT/usr/share/kicad/footprints"
export KICAD9_TEMPLATE_DIR="$BIPE_TOOL_ROOT/usr/share/kicad/template"
# Freerouting runs on the project-local OpenJDK 21 (the extracted Debian package has no
# /usr/bin/java alternatives link).
export JAVA_HOME="$BIPE_TOOL_ROOT/usr/lib/jvm/java-21-openjdk-amd64"
export PATH="$BIPE_TOOL_ROOT/usr/bin:$JAVA_HOME/bin:$PATH"
# KiCad 9 here is the Ubuntu 24.04 build: it embeds libpython3.12 and its pcbnew and wx
# modules are built for 3.12. Ubuntu 26.04 ships only Python 3.14, so use uv's managed 3.12
# (uv python install 3.12) for the library and as the interpreter for pcbnew scripts.
if [ -z "${BIPE_PYTHON312:-}" ] && command -v uv >/dev/null; then
    BIPE_PYTHON312="$(uv python find --system 3.12 2>/dev/null || true)"
fi
if [ -n "${BIPE_PYTHON312:-}" ]; then
    export BIPE_PYTHON312
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(dirname "$(dirname "$(readlink -f "$BIPE_PYTHON312")")")/lib"
fi
export XDG_CONFIG_HOME="$BIPE_TOOL_ROOT/../state/config"
export XDG_CACHE_HOME="$BIPE_TOOL_ROOT/../state/cache"
export XDG_DATA_HOME="$BIPE_TOOL_ROOT/../state/data"
export KICAD_CONFIG_HOME="$XDG_CONFIG_HOME/kicad"
mkdir -p "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME" "$XDG_DATA_HOME"
exec "$@"
