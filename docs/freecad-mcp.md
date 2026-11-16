# FreeCAD Robust MCP setup

Installed and verified on 30 September 2026. The native MCP connection has also been exercised during Milestone 3 assembly work.

## Connection and installation

Codex launches `tools/freecad-mcp.sh`, which uses this checkout's `.venv` and starts the Robust MCP server over stdio. The server connects through XML-RPC to the user's installed FreeCAD addon at `127.0.0.1:9875`. That port is the addon endpoint, not an MCP HTTP endpoint. No external listener or background MCP service is needed.

The installed addon is at `~/snap/freecad/common/Mod/FreecadRobustMCPBridge`. The connection was tested against the running GUI of FreeCAD 1.1.1, with Python 3.12.3 on both sides. Embedded FreeCAD imports are not used.

The server is `freecad-robust-mcp==0.6.1` with `mcp==1.30.0`. Direct tooling requirements are in `requirements-mcp.in`; exact transitive versions are in `requirements-mcp.lock`. The simulation pins in `requirements.lock` are unchanged. Restore the combined environment with:

```bash
uv pip sync --python .venv/bin/python requirements.lock requirements-mcp.lock
uv pip check --python .venv/bin/python
```

The enabled global Codex entry is named `freecad` and launches `tools/freecad-mcp.sh` in the repository. Registration was performed with the supported CLI and verified with `codex mcp get freecad --json`. Recreate it on this computer with:

```bash
codex mcp add freecad -- /absolute/path/to/bipe/tools/freecad-mcp.sh
```

If the checkout moves, register its new absolute launcher path. The launcher resolves its own project root. User-level Codex configuration is machine-local and is not committed into this repository.

For each session, open FreeCAD and start the Robust MCP Bridge from its workbench. Its optional auto-start preference can handle this on future FreeCAD launches. The addon was already running during verification; no addon files or preferences were changed by this setup. Reload the server in Codex Settings → MCP servers, or restart Codex, to load the newly registered tools. Native tools are now exposed in the restarted assistant session and have been used for the M3 assembly, saving and GUI views.

## Compatibility corrections

The published server release needed the following corrections to complete the actual CAD workflow:

| Observed issue | Project correction |
| --- | --- |
| The package accepts MCP 2.x but imports the removed `FastMCP` API. | Constrain the SDK to MCP 1.x and lock the verified version. |
| `create_sketch` assigns the obsolete `Support` property. | Select `AttachmentSupport` where available, with the older property as a fallback. |
| `add_sketch_rectangle` uses `Sketcher.Constraint` without importing Sketcher. | Import the module explicitly in the FreeCAD execution context. |
| `pad_sketch` assigns a nonexistent `Symmetric` property. | Use `SideType` or the older `Midplane` property. |
| Object inspection includes FreeCAD objects nested in link-property containers. XML-RPC cannot marshal them. | Normalize returned values to JSON-compatible containers, representing opaque objects as strings. |
| Screenshot code reads `view.__class__.__name__`, but this view's dynamic attribute returns a dictionary. | Check screenshot capabilities and capture the requested document's 3D view. |

`tools/freecad_mcp_server.py` applies these corrections when launching the pinned release. It uses public MCP tool registration and a local XML-RPC bridge subclass; the installed package and FreeCAD addon remain unchanged. The launcher refuses an unreviewed server version upgrade.

This verification covers the exercised workflow, not every advertised tool. XY origin-plane sketch creation, a constrained rectangle, ordinary pad extrusion, inspection, Python execution, saving/reopening, STEP export, and GUI capture passed. Other tools and options require verification when first used. `execute_python` remains available for application operations outside the tested high-level tools.

Sources: [upstream server](https://github.com/spkane/freecad-addon-robust-mcp-server), [published package](https://pypi.org/project/freecad-robust-mcp/), [FreeCAD attachment API change](https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Release_notes_1.0.md), and [official Codex MCP configuration](https://learn.chatgpt.com/docs/extend/mcp).

## Verification evidence

Run `tools/freecad-mcp.sh --check` for connection-only diagnostics. Run `.venv/bin/python tools/verify_freecad_mcp.py` for a separate-document integration check. It restores the original active document and closes only the test documents it creates.

The accepted run discovered 83 MCP tools and created a fully constrained 20 × 30 mm sketch. A 4 mm pad had volume 2400 mm³. Changing its thickness to 6 mm produced 3600 mm³. After native save/reopen and STEP round-trip, both shapes were valid single solids measuring 20 × 30 × 6 mm with volume 3600 mm³. The 800 × 600 PNG screenshot was decoded and visually inspected. The original open-document list was preserved.

`tools/verify_freecad_mcp.py` writes its evidence to `artifacts/tooling/freecad-mcp/` (gitignored, kept on the workstation and not published): `verification.json`, `tools.json`, the test FCStd and STEP files, the PNG, and server diagnostics. The report includes returned measurements and output hashes. The delivery checks verify all 57 locked package versions, CAD output hashes, Codex registration, and launcher source hashes. The combined environment passed dependency consistency checks, all 13 existing robot tests, Ruff checks, and shell syntax validation. Physical robot parameters and controllers were not changed, so the milestone acceptance traces remain the verified baseline.

Local Git history now includes the Milestones 0–2 baseline and this verified tooling checkpoint. Continue committing at meaningful implementation and validation checkpoints as requested.

## Long calculations

The addon executes on the FreeCAD GUI thread. Its XML-RPC client can time out after about 30 seconds even if a longer tool timeout is requested; a timed-out calculation can continue and occupy the bridge. Use short MCP operations and run full clearance scans with `FreeCADCmd` through `cad/check_assembly.py`. The scan preserves the GUI-saved native document. During M3 an overly expensive first scan required a user restart; the corrected exact-mirror geometry and broad-phase checks finish headlessly. The bridge is working again. No addon/server upgrade was required.
