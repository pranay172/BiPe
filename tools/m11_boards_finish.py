"""Finish the routed M11 boards: import the Freerouting session, pour ground, check and export.

Run with: bash tools/electronics-env.sh python3.12 tools/m11_boards_finish.py

Takes the placed PCBs from tools/m11_boards.py and artifacts/m11/boards/<board>/routed.ses
(Freerouting 1.9.0 run on the generator's unrouted.dsn), imports the tracks, adds GND
pours on both layers (and VBAT/BUS pours over the power board's heavy tracks), fills
them, then runs ERC and DRC with schematic parity and writes Gerbers, drill, placement
(JLC CPL), SVG plots, a schematic PDF and report.json. The boards remain digital
candidates: not ordered, assembled or tested.
"""

import csv
import hashlib
import json
import subprocess
import sys
import zipfile
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "electronics/m11"
ART = ROOT / "artifacts/m11/boards"
KICAD_CLI = "kicad-cli"
GERBER_LAYERS = "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
# Parts JLC does not place: the Teensy module and its headers are hand-fitted, wire pads take the
# pack and bus leads, and mounting holes have no part.
HAND_FIT_PREFIXES = ("Teensy40", "SolderWire", "MountingHole")
# Pours over the power board's heavy tracks (net, inflate mm); they sit above the GND pour.
POURS = {"power": [("VBAT", 1.0), ("BUS", 1.0)]}


def mm(v):
    return pcbnew.FromMM(v)


def zone(pcb, net, layer, points, priority, solid):
    z = pcbnew.ZONE(pcb)
    z.SetLayer(layer)
    z.SetNetCode(pcb.FindNet("/" + net).GetNetCode())  # schematic label nets are rooted at /
    z.SetAssignedPriority(priority)
    z.SetMinThickness(mm(0.25))
    z.SetLocalClearance(mm(0.3))
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid else pcbnew.ZONE_CONNECTION_THERMAL)
    outline = z.Outline()
    outline.NewOutline()
    for x, y in points:
        outline.Append(mm(x), mm(y))
    pcb.Add(z)
    return z


def heavy_box(pcb, net, inflate):
    """Bounding box (mm) of the locked heavy tracks on a net, inflated."""
    boxes = [
        t.GetBoundingBox() for t in pcb.GetTracks() if t.GetNetname() == "/" + net and t.IsLocked()
    ]
    x0 = min(b.GetLeft() for b in boxes) / 1e6 - inflate
    x1 = max(b.GetRight() for b in boxes) / 1e6 + inflate
    y0 = min(b.GetTop() for b in boxes) / 1e6 - inflate
    y1 = max(b.GetBottom() for b in boxes) / 1e6 + inflate
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def run(*args, check=False):
    """kicad-cli; a check (--exit-code-violations) may exit nonzero when it wrote its report."""
    result = subprocess.run(
        [KICAD_CLI, *map(str, args)], capture_output=True, text=True, check=False
    )
    if result.returncode and not (check and Path(args[args.index("-o") + 1]).exists()):
        raise RuntimeError(f"{args[:3]} failed: {result.stderr[-2000:]}")
    return result.returncode


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def envelopes(pcb, short, size_mm):
    """Component body boxes in board millimetres for the CAD (cad/m10/assembly.py).

    Each footprint's F.Fab outline (its courtyard if it has none) is the body's footprint on the
    board; heights come from m10.electronics.boards.component_heights_m. The Teensy's two
    header rows are listed separately from the module above them."""
    rows = []
    for foot in pcb.GetFootprints():
        name = foot.GetFPID().GetLibItemName().wx_str()
        if name.startswith(("MountingHole", "SolderWire")):
            continue
        boxes = {}
        for item in foot.GraphicalItems():
            layer = item.GetLayer()
            if layer in (pcbnew.F_Fab, pcbnew.F_CrtYd) and item.GetClass() != "PCB_TEXT":
                boxes.setdefault(layer, []).append(item.GetBoundingBox())
        chosen = boxes.get(pcbnew.F_Fab) or boxes[pcbnew.F_CrtYd]
        x0 = min(b.GetLeft() for b in chosen) / 1e6
        x1 = max(b.GetRight() for b in chosen) / 1e6
        y0 = min(b.GetTop() for b in chosen) / 1e6
        y1 = max(b.GetBottom() for b in chosen) / 1e6
        rows.append({"ref": foot.GetReference(), "footprint": name, "box_mm": [x0, x1, y0, y1]})
        if name.startswith("Teensy40"):
            for row in ("top", "bottom"):
                pads = [p for p in foot.Pads() if (int(p.GetNumber()) <= 14) == (row == "top")]
                pads = [p.GetBoundingBox() for p in pads if int(p.GetNumber()) <= 28]
                yc = sum(b.GetCenter().y for b in pads) / len(pads) / 1e6
                rows.append(
                    {
                        "ref": f"{foot.GetReference()}-{row}-header",
                        "footprint": "PinHeader_1x14_P2.54mm",
                        "box_mm": [
                            min(b.GetLeft() for b in pads) / 1e6 - 0.42,
                            max(b.GetRight() for b in pads) / 1e6 + 0.42,
                            yc - 1.27,
                            yc + 1.27,
                        ],
                    }
                )
    for r in rows:
        r["box_mm"] = [round(v, 4) for v in r["box_mm"]]
    return {
        "board": short,
        "frame": "board millimetres; the generator draws the outline from (0, 0), KiCad x right "
        "and y down, front (component) side; map with m10.electronics.boards",
        "size_mm": size_mm,
        "parts": sorted(rows, key=lambda r: r["ref"]),
    }


def finish(short):
    name = f"bipe-m11-{short}"
    out, art = OUT / short, ART / short
    path = out / f"{name}.kicad_pcb"
    pcbnew.KIID.SeedGenerator(sum(name.encode()) + 1)  # reproducible UUIDs for the new items
    pcb = pcbnew.LoadBoard(str(path))
    heavy = [t for t in pcb.GetTracks() if t.IsLocked()]
    assert pcbnew.ImportSpecctraSES(pcb, str(art / "routed.ses")), "SES import failed"
    # The session replaces the routing; the locked heavy tracks must survive it.
    assert len([t for t in pcb.GetTracks() if t.IsLocked()]) >= len(heavy)

    edge = pcb.GetBoardEdgesBoundingBox()
    corners = [
        (edge.GetLeft() / 1e6, edge.GetTop() / 1e6),
        (edge.GetRight() / 1e6, edge.GetTop() / 1e6),
        (edge.GetRight() / 1e6, edge.GetBottom() / 1e6),
        (edge.GetLeft() / 1e6, edge.GetBottom() / 1e6),
    ]
    # Solid pad connections: the GND pours carry the pack return, and thermal spokes into the
    # narrow pour islands between connector pins starve (DRC starved_thermal).
    zones = [zone(pcb, "GND", layer, corners, 0, True) for layer in (pcbnew.F_Cu, pcbnew.B_Cu)]
    for priority, (net, inflate) in enumerate(POURS.get(short, []), start=1):
        zones.append(zone(pcb, net, pcbnew.F_Cu, heavy_box(pcb, net, inflate), priority, True))
    pcbnew.ZONE_FILLER(pcb).Fill(pcb.Zones())
    pcbnew.SaveBoard(str(path), pcb)

    tracks = [t for t in pcb.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [t for t in pcb.GetTracks() if t.GetClass() == "PCB_VIA"]
    widths = sorted({round(t.GetWidth() / 1e6, 3) for t in tracks})

    sch = out / f"{name}.kicad_sch"
    for f in (art / "erc.json", art / "drc.json"):
        f.unlink(missing_ok=True)
    run(
        "sch",
        "erc",
        "--format",
        "json",
        "--severity-all",
        "--exit-code-violations",
        "-o",
        art / "erc.json",
        sch,
        check=True,
    )
    run(
        "pcb",
        "drc",
        "--format",
        "json",
        "--severity-all",
        "--schematic-parity",
        "--exit-code-violations",
        "-o",
        art / "drc.json",
        path,
        check=True,
    )
    erc = json.loads((art / "erc.json").read_text())
    drc = json.loads((art / "drc.json").read_text())

    fab = art / "fab"
    gerbers = fab / "gerbers"
    gerbers.mkdir(parents=True, exist_ok=True)
    for f in gerbers.iterdir():
        f.unlink()
    run(
        "pcb",
        "export",
        "gerbers",
        "--layers",
        GERBER_LAYERS,
        "--no-protel-ext",
        "-o",
        gerbers,
        path,
    )
    run(
        "pcb",
        "export",
        "drill",
        "--format",
        "excellon",
        "--excellon-separate-th",
        "--generate-map",
        "--map-format",
        "gerberx2",
        "-o",
        str(gerbers) + "/",
        path,
    )
    with zipfile.ZipFile(fab / f"{name}-gerbers.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(gerbers.iterdir()):
            z.write(f, f.name)
    pos = fab / "pos-kicad.csv"
    run(
        "pcb",
        "export",
        "pos",
        "--format",
        "csv",
        "--units",
        "mm",
        "--side",
        "both",
        "-o",
        pos,
        path,
    )
    audit = json.loads((art / "pin-audit.json").read_text())
    (art / "envelopes.json").write_text(
        json.dumps(envelopes(pcb, short, audit["size_mm"]), indent=1) + "\n"
    )
    hand_fit = {
        c["ref"]
        for c in audit["components"]
        if not c["footprint"] or c["footprint"].split(":")[-1].startswith(HAND_FIT_PREFIXES)
    }
    placed = []
    with pos.open() as src, (fab / "cpl-jlc.csv").open("w") as dst:
        writer = csv.writer(dst, lineterminator="\n")
        writer.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in csv.DictReader(src):
            if row["Ref"] in hand_fit or row["Ref"].startswith("H"):
                continue
            placed.append(row["Ref"])
            side = "Top" if row["Side"] == "top" else "Bottom"
            writer.writerow([row["Ref"], f"{row['PosX']}mm", f"{row['PosY']}mm", side, row["Rot"]])
    for side, layers in [
        ("top", "F.Cu,F.Silkscreen,F.Mask,Edge.Cuts"),
        ("bottom", "B.Cu,B.Silkscreen,B.Mask,Edge.Cuts"),
    ]:
        run(
            "pcb",
            "export",
            "svg",
            "--layers",
            layers,
            "--page-size-mode",
            "2",
            "--exclude-drawing-sheet",
            *(["--mirror"] if side == "bottom" else []),
            "-o",
            art / f"{side}.svg",
            path,
        )
    run("sch", "export", "pdf", "-o", art / "schematic.pdf", sch)

    unconnected = len(drc["unconnected_items"])
    report = {
        "board": name,
        "status": "M11 phase D digital candidate: routed and checked in KiCad, not ordered, "
        "assembled or hardware validated",
        "router": "Freerouting 1.9.0 (-mp 20 -mt 1) on artifacts/m11/boards/"
        f"{short}/unrouted.dsn; heavy power tracks placed and locked by the generator",
        "erc_violations": sum(len(s["violations"]) for s in erc["sheets"]),
        "drc_violations": len(drc["violations"]),
        "drc_unconnected": unconnected,
        "schematic_parity_issues": len(drc.get("schematic_parity", [])),
        "tracks": len(tracks),
        "vias": len(vias),
        "track_widths_mm": widths,
        "track_length_mm": round(sum(t.GetLength() for t in tracks) / 1e6, 1),
        "zones": [
            {
                "net": z.GetNetname(),
                "layer": pcb.GetLayerName(z.GetLayer()),
                "priority": z.GetAssignedPriority(),
                "filled_area_mm2": round(z.GetFilledArea() / 1e12, 1),
            }
            for z in pcb.Zones()
        ],
        "jlc_placed": placed,
        "hand_fit": sorted(hand_fit),
        "files": {
            str(f.relative_to(ROOT)): digest(f)
            for f in sorted([path, sch, *fab.rglob("*")])
            if f.is_file()
        },
    }
    report["passed"] = (
        report["erc_violations"] == 0
        and report["drc_violations"] == 0
        and unconnected == 0
        and report["schematic_parity_issues"] == 0
    )
    (art / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return {
        k: report[k]
        for k in (
            "board",
            "erc_violations",
            "drc_violations",
            "drc_unconnected",
            "schematic_parity_issues",
            "vias",
            "passed",
        )
    }


def main():
    boards = sys.argv[1:] or ["carrier", "power"]
    print(json.dumps([finish(b) for b in boards], indent=1))


if __name__ == "__main__":
    main()
