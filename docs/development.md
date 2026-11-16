# Development pipelines

The pipelines behind the design: CAD, boards, firmware images, the heavy
simulation suites and renders. They need more tooling than the quick start
(FreeCAD, KiCad, PlatformIO, Blender) and were run on one Ubuntu workstation,
so treat the commands as a record of how the outputs were made. Run every
command from the repository root, with the environment from
[getting-started.md](getting-started.md).

## Extra environments

The workstation's `.venv` adds two lock files to `requirements.lock`:

```bash
uv pip sync requirements.lock requirements-mcp.lock requirements-moteus.lock --python .venv/bin/python
```

`requirements-mcp.lock` holds the FreeCAD and Blender MCP bridges' packages.
`requirements-moteus.lock` holds mjbots' `moteus` package (1.1.1),
which is used only as the reference for the M11 codec parity tests. It
is compiled against `requirements.lock`:

```bash
UV_CACHE_DIR=$PWD/.uv-cache uv pip compile requirements-moteus.in -c requirements.lock --python .venv/bin/python -o requirements-moteus.lock
```

Without it, the M11 firmware tests are skipped.

The Blender MCP server uses its own environment in `.venv/blender-mcp`,
built from `requirements-blender-mcp.lock`.

## Tests

The C core's own tests: `make -C firmware test`. Its `config` step
generates `firmware/include/generated_config.h` from the configs. The
header embeds their hashes, so it is a build output and is not tracked.
`make -C firmware teensy` builds the M4 diagnostic image after
generating it.

## M11 firmware

The hardware header and the moteus driver configurations are generated
from `config/robot.json`, `config/controller_m10.json` and the routed
carrier. A test checks that the committed outputs are current.

```bash
.venv/bin/python tools/generate_m11_hardware.py
.venv/bin/python -m bipe.m11.timing
make -C firmware teensy-m11
```

The M11 boards use the project-local KiCad 9, Java and Xvfb in
`.local-tools/root` through `tools/electronics-env.sh`. On Ubuntu 26.04,
pcbnew scripts run with uv's `python3.12`:

```bash
bash tools/electronics-env.sh python3.12 tools/m11_boards.py
bash tools/electronics-env.sh xvfb-run -a java -jar .local-tools/freerouting-1.9.0.jar -de artifacts/m11/boards/carrier/unrouted.dsn -do routed.ses -mp 20 -mt 1
bash tools/electronics-env.sh python3.12 tools/m11_boards_finish.py
```

The committed `routed.ses` files are the routing of record; a new
Freerouting run gives a different, valid routing.

`teensy-m11` builds with the local PlatformIO
(`.venv/platformio`, packages in `.local-tools/platformio`). The image
cannot command torque while `BIPE_HARDWARE_QUALIFIED` is 0. Load each
`artifacts/m11/firmware/moteus/<id>-<name>.cfg` into its driver with
`moteus_tool` at commissioning, after setting the output sign and offset
on the bench.

## CAD pipeline (headless FreeCAD)

The order matters. `adopt_cad` copies the CAD masses into
`config/robot.json`, so the exterior-to-adopt sequence runs twice.

`M10_FCSTD_DIR` sends the assembly FCStd to a separate folder. That
keeps a working copy of `artifacts/m10/assembly/assembly.FCStd`
untouched.

```bash
fc() { /snap/bin/freecad.cmd -c "p='cad/m10/$1.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"; }
export M10_FCSTD_DIR=artifacts/m10/assembly/d4
for pass in 1 2; do
  (unset M10_FCSTD_DIR; fc exterior); fc assembly; fc sweep; fc mass_properties
  .venv/bin/python -m bipe.m10.adopt_cad
done
fc sections; fc shell_clearance; fc manufacturability
```

## Falls and acceptance

These are heavy. Run one job at a time, with at most 8 workers.

```bash
nice -n 10 .venv/bin/python -m bipe.m10.falls --workers 6 --output artifacts/m10/falls
nice -n 10 .venv/bin/python -m bipe.m10.falls --workers 6 --dt 6.25e-5 --output artifacts/m10/falls/dt_half
nice -n 10 .venv/bin/python -m bipe.robustness --profile m10 --controllers current --workers 8
```

## Renders (Blender)

Keep Blender open with the Blender Lab MCP add-on (port 9877). Stages
include `exterior`, `assembly` and the look studies, for example
`end_stop/A_hidden_slots`.

```bash
python3 tools/blender_bridge.py cad/m10/render_blender.py --arg stage=assembly
```

## MCP tooling

- [FreeCAD MCP](freecad-mcp.md): `tools/freecad-mcp.sh --check`, then
  `.venv/bin/python tools/verify_freecad_mcp.py`.
- [Blender MCP](blender-mcp.md): `.venv/bin/python tools/verify_blender_mcp.py`.

Other reference documents:
- [workstation assessment](workstation.md);
- [build and design references](references.md).
