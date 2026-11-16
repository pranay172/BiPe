# Reproducing the M4 tools and checks

The verified environment is Ubuntu 24.04 amd64, system Python 3.12 for KiCad's PCB API and project `.venv` for simulation. KiCad 9.0.9, ngspice 42, ARM GCC 13.2.rel1, Java 21 and Freerouting 1.9.0 are unpacked into ignored `.local-tools/`. No administrator installation was needed. Native FreeCAD MCP remains usable; KiCad CLI/PCB Python API and compiler commands provide direct, reproducible access without a new MCP or desktop clicking.

The package [manifest](../../tools/electronics-tools.json) pins download URLs and SHA-256. All 38 used Ubuntu/PPA archives were verified against repository metadata; the KiCad 9 archive matches its PPA SHA-256. Standard footprints from KiCad 7.0.11 are copied into this self-contained KiCad 9 project. The schema warning emitted by the relocated CLI names a missing global `/usr/share/kicad/schemas/api.v1.schema.json`; actual ERC/DRC/native-load/export operations complete successfully and their reports are checked. This warning is preserved in the logs.

From the project directory, recreate tools and Python dependencies:

```bash
UV_CACHE_DIR=$PWD/.uv-cache uv pip sync requirements.lock requirements-mcp.lock requirements-electronics.lock --python .venv/bin/python
.venv/bin/python tools/setup_electronics.py
UV_CACHE_DIR=$PWD/.uv-cache uv venv .venv/platformio --python /usr/bin/python3
UV_CACHE_DIR=$PWD/.uv-cache uv pip sync requirements-platformio.lock --python .venv/platformio/bin/python
```

PlatformIO is isolated within `.venv/platformio` to avoid conflicting with FreeCAD MCP's Starlette/Click/Uvicorn versions. The first verified PlatformIO environment used Python 3.10.20; these pinned packages also allow system Python 3.12. `firmware/platformio.ini` pins Teensy platform 5.0.0; its resolved framework/tool versions are recorded in `firmware/platformio-packages.json` and the target build log. All virtual environments stay inside the project `.venv/` directory.

Build and exercise the portable and diagnostic target firmware:

```bash
make -C firmware test arm
PLATFORMIO_CORE_DIR=$PWD/.local-tools/platformio .venv/platformio/bin/pio run -d firmware
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.firmware_validate
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.firmware_fault_experiment
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.electrical_analysis
```

The target command builds a diagnostic ELF/HEX, with motor commands inhibited. No upload/flash step is included. Cortex-M7 core objects build with warnings treated as errors; the framework build has upstream library warnings. Hardware execution/deadline timing has not been measured. Host UBSan was also run on the C unit suite.

Check the delivered routed native design without regenerating it:

```bash
bash tools/electronics-env.sh kicad-cli sch erc --format json --severity-all --exit-code-violations -o artifacts/milestone-4/electrical/erc.json electronics/bipe-controller.kicad_sch
bash tools/electronics-env.sh kicad-cli pcb drc --format json --all-track-errors --schematic-parity --severity-all --exit-code-violations -o artifacts/milestone-4/electrical/drc.json electronics/bipe-controller.kicad_pcb
bash tools/electronics-env.sh kicad-cli sch export netlist --format kicadxml -o artifacts/milestone-4/electrical/carrier.xml electronics/bipe-controller.kicad_sch
bash tools/electronics-env.sh /usr/bin/python3 tools/verify_electronics.py
```

The generator **replaces the native schematic and PCB with a fresh unrouted candidate**. Preserve manual edits first. To reproduce layout from source, generate, route and fill in that order:

```bash
bash tools/electronics-env.sh /usr/bin/python3 tools/build_electronics.py
bash tools/electronics-env.sh xvfb-run -a -s '-screen 0 1280x1024x24' .local-tools/root/usr/lib/jvm/java-21-openjdk-amd64/bin/java -jar .local-tools/freerouting-1.9.0.jar -de artifacts/milestone-4/electrical/carrier-unrouted.dsn -do artifacts/milestone-4/electrical/carrier-routed.ses -mp 10 -mt 1 -da
bash tools/electronics-env.sh /usr/bin/python3 tools/finish_electronics.py
```

The virtual X display is only for the router's Java runtime. The sandbox may require escalation for its local X socket; it does not operate the user's desktop. Router analytics are disabled. Routing is not guaranteed byte-for-byte deterministic; repeat all native checks after changes. Supplied routing/session files and the final PCB are saved. Opening KiCad uses `bash tools/electronics-env.sh kicad electronics/bipe-controller.kicad_pro`.

The M4 sensor-resolution acceptance and unchanged fixed-height regression use separate output paths to preserve M3:

```bash
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.moving_validate --sensor-config config/sensors-m4.json --output artifacts/milestone-4/acceptance
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.validate --output artifacts/milestone-4/fixed-height-regression
.venv/bin/python -m pytest -q
.venv/bin/ruff check bipe cad tools tests
.venv/bin/python -m bipe.verify_m4
```

The delivery verifier checks saved evidence/configurations and sealed file hashes. After intentional source/output changes, rerun the relevant checks and use `python -m bipe.verify_m4 --seal` to record a new verified checkpoint. Sealing does not turn an unqualified actuator/power assembly into a complete milestone. Keep the sources, `.cir` decks, traces, reports and diagnostic images together.

## Final M4 integrated candidate

The earlier commands above describe historical stages. Current native projects
are `electronics/power-integrated` and `electronics/carrier-integrated`; the
stage-aware verifier checks final digital M4 completion and keeps physical gates
separate. Use explicit `closure` output paths to preserve earlier experiments.

```bash
bash tools/verify_closure_firmware.sh
.venv/bin/python -m bipe.verify_m4
```

To reconstruct the specified native routing, use the supplied `placed-heavy`
boards and `routed.ses` sessions in `closure/electrical`. The finisher preserves
wide current paths, provides the four-layer signal repairs and fills only the
carrier ground faces. Its repair endpoints refer to this saved session; a fresh
layout needs new endpoints and native checks.

```bash
bash tools/electronics-env.sh /usr/bin/python3 tools/finish_closure_electronics.py
bash tools/electronics-env.sh /usr/bin/python3 tools/add_closure_bypass.py
```

These commands replace the native candidate. Re-export KiCad XML netlists and
rerun each board's `sch erc` and `pcb drc --schematic-parity` into its own
`closure/electrical/*-integrated` folder, then run the independent audits:

```bash
bash tools/electronics-env.sh /usr/bin/python3 tools/audit_closure_copper.py
bash tools/electronics-env.sh /usr/bin/python3 tools/audit_closure_interfaces.py
bash tools/electronics-env.sh /usr/bin/python3 tools/export_closure_envelopes.py
.venv/bin/python tools/call_freecad.py cad/closure_study.py --timeout 45 --log artifacts/milestone-4/closure/cad/mcp.log
.venv/bin/python tools/adopt_closure.py
```

The CAD script uses the archived packaging mass source and refuses changed main
geometry; a new mechanical design needs a new corresponding CAD/component source.
The adoption tool derives the seven candidate tensors without touching the
accepted robot. Before adoption, set `BIPE_ROBOT_CONFIG` to the **absolute** path
of `closure/candidate-robot.json` for `make -C firmware all` and the physical
experiments. The header and C binding use this same selected source. Run:

```bash
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.moving_validate --implementation c --sensor-config config/sensors-m4.json --workers 6 --output artifacts/milestone-4/closure/acceptance
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.mechanical_study --implementation c --sensor-config config/sensors-m4.json --output artifacts/milestone-4/closure/mechanical-uncertainty
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.validate --output artifacts/milestone-4/closure/fixed-height-regression
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.control_validate --output artifacts/milestone-4/closure/firmware
MPLCONFIGDIR=/tmp/bipe-mpl .venv/bin/python -m bipe.firmware_fault_experiment --output artifacts/milestone-4/closure/physics-faults
.venv/bin/python -m bipe.mechanical_review --output artifacts/milestone-4/closure/engineering
.venv/bin/python -m bipe.closure_analysis
.venv/bin/python tools/verify_closure_spice.py
.venv/bin/python tools/audit_closure_energy.py
.venv/bin/python tools/adopt_closure.py --adopt
```

Adoption requires passing candidate acceptance, regression, uncertainty, packet
comparison, physical faults, fit/service/native checks and the whole-fall energy
inventory. Copy the generated `closure/power-candidate.json` to
`config/power-stage.json`, unset the candidate override, rebuild the inhibited
firmware and run the final verifier. The delivered accepted configuration already
contains those results. After intentional changes, complete the relevant checks
before making a fresh seal. An independent startup MCU algorithm is provided and
host-tested; native AVR ADC/IO/watchdog integration and hardware timing are still
qualification requirements.
