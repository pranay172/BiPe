"""M11 phase D packaging probe: candidate electronics envelopes against the M10 assembly.

/snap/bin/freecad.cmd -c "p='cad/m11/packaging_probe.py'; exec(open(p).read(), {'__file__': p, '__name__': '__main__'})"

Builds cad/m10/assembly.py at each probe hip height (body frame, legs posed) and
intersects every candidate box (body frame, mm, keep-outs included) with every
part, the hollowed body shell and its screw columns included. A candidate may
replace existing parts (listed in "replaces"), which are then left out of its
check; since phase D step 5 the assembly holds the adopted parts, so each
candidate replaces the part it stands for. Overlaps above 1 mm^3 are reported per
pose; a candidate also fails if it leaves the body interior box. Candidates are
also checked against each other. Results go to
artifacts/m11/packaging/probe.json.
"""

import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import Part
from FreeCAD import Vector as V

from bipe.m10.concept import pose

_spec = importlib.util.spec_from_file_location("m10_assembly", ROOT / "cad/m10/assembly.py")
A = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(A)
C = A.C
OUT = ROOT / "artifacts/m11/packaging/probe.json"
HIP_HEIGHTS_M = [0.06, 0.072, 0.103, 0.135]
MOTEUS = ["Moteus"]

# name: (boxes [x0, x1, y0, y1, z0, z1] in mm, parts it replaces, note)
CANDIDATES = {
    "pi_zero_rear": (
        [[-81.0, -75.0, -32.5, 32.5, -12.0, 18.0], [-81.0, -75.0, 32.5, 41.0, -3.0, 9.0]],
        ["PiZero"],
        (
            "Pi Zero 2 W standing in the y-z plane against the rear wall; 6 mm board envelope "
            "(1 mm PCB, components both sides), camera FPC keep-out at the +y end, behind the rear "
            "screw column"
        ),
    ),
    "power_board_rear_low": (
        [[-82.0, -73.0, -30.0, 30.0, -33.5, -13.5]],
        ["PowerPCB", "PowerParts"],
        (
            "Power board (XT30 in, LM5060, DPAK MOSFET, output pads; the MINI fuse is inline in the "
            "pack lead), 60 x 20 mm standing in the y-z plane below the Pi; 9 mm component height"
        ),
    ),
    "bec_host_mirror": (
        [[-12.0, 24.0, -23.5, -16.5, 6.0, 32.0]],
        ["BEC"],
        (
            "Matek BEC12S-PRO (35 x 24 x 5.5 mm) standing in the x-z plane, mirrored from the host slot; "
            "1 mm for the end solder pads, the 5 V lead crosses to the carrier"
        ),
    ),
    "power_switch_rear": (
        [[-83.0, -76.5, 31.0, 41.5, -27.5, -19.5]],
        ["PowerSwitch"],
        (
            "Miniature SPDT slide switch (EN/UVLO divider, microamps) on the flat rear wall beside the "
            "power board, behind the rear screw column; actuator through a slot in the 2 mm wall"
        ),
    ),
    "teensy_usb_via_battery_bay": (
        [[26.0, 46.0, 15.5, 23.5, 20.0, 33.0], [38.0, 46.0, 15.5, 23.5, -40.0, 20.0]],
        ["Battery"],
        (
            "Down-angle micro-USB plug on the Teensy's +x port and its cable to the hatch, with the "
            "pack removed (layout-study envelope until the carrier layout fixes the port)"
        ),
    ),
    "carrier_host_slot": (
        [[-14.0, 26.0, 13.9, 23.9, 5.5, 35.5]],
        ["CarrierPCB", "CarrierParts", "Teensy"],
        "Teensy 4.0 carrier (40 x 30 mm board and its Teensy stack) in the old host envelope, 0.5 mm below the cover",
    ),
    "carrier_plug_keepout": (
        [[-13.7, 21.5, 23.9, 34.0, 5.5, 13.5]],
        ["CarrierParts"],
        (
            "Top-entry PH plugs (SYS, EYES, CAN) and wire bends off the carrier front (+y); the "
            "carrier -y side holds the knee pulley and belt and its edges the knee shaft and cover"
        ),
    ),
    "power_plug_keepout": (
        [[-73.0, -66.0, 4.5, 28.5, -30.0, -15.0], [-73.0, -66.0, -30.0, -24.0, -23.0, -15.0]],
        ["PowerParts"],
        (
            "Top-entry PH plugs (MON, CAN END, SWITCH) and the soldered pack, bus and BEC leads off "
            "the power board front (+x); the tub floor is 2.5 mm below the board edge"
        ),
    ),
    "moteus_top_connector_keepout": (
        [[-27.5, -14.5, 4.5, 42.5, 26.5, 35.5], [-27.5, -14.5, -42.5, -4.5, 26.5, 35.5]],
        [],
        (
            "XT30 and PH-3 plug plus wire-bend keep-out (13 mm) off the +x edge of the two upper "
            "motor-row moteus boards"
        ),
    ),
    "moteus_bottom_connector_keepout": (
        [[-27.5, -13.0, 4.5, 42.5, -39.5, -30.5], [-27.5, -13.0, -42.5, -4.5, -39.5, -30.5]],
        [],
        "The same keep-out (14.5 mm) off the two lower motor-row boards",
    ),
    "moteus_side_connector_keepout": (
        [[21.5, 59.5, 24.0, 33.0, 19.0, 33.0], [21.5, 59.5, -33.0, -24.0, 19.0, 33.0]],
        [],
        "XT30 and PH-3 keep-out (14 mm) above the two vertical moteus boards beside the battery",
    ),
}


def box(b):
    x0, x1, y0, y1, z0, z1 = b
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def main():
    interior = A.interior_box()
    results = {
        name: {"note": c[2], "replaces": c[1], "poses": {}} for name, c in CANDIDATES.items()
    }
    for height in HIP_HEIGHTS_M:
        state = pose(height, C)
        parts = A.build(state)
        for name, (boxes, replaces, _) in CANDIDATES.items():
            shapes = [box(b) for b in boxes]
            outside = sum(s.cut(interior).Volume for s in shapes)
            hits = []
            for link, part, shape, _ in parts:
                if part in replaces or part.endswith("Tread"):
                    continue
                for s in shapes:
                    if not s.BoundBox.intersect(shape.BoundBox):
                        continue
                    common = s.common(shape)
                    if common.Volume > 1.0:
                        bb = common.BoundBox
                        where = [
                            round(t, 1)
                            for t in (bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax)
                        ]
                        hits.append([link, part, round(common.Volume, 1), where])
            results[name]["poses"][f"{height:.3f}"] = {
                "outside_interior_mm3": round(outside, 1),
                "overlaps_mm3": hits,
            }
        print("M11 PROBE pose", height, flush=True)
    for r in results.values():
        r["clear"] = all(
            p["outside_interior_mm3"] < 1.0 and not p["overlaps_mm3"] for p in r["poses"].values()
        )
    # Candidates against each other (pose independent: all sit in the body frame).
    names = list(CANDIDATES)
    mutual = []
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            vol = sum(
                box(p).common(box(q)).Volume for p in CANDIDATES[a][0] for q in CANDIDATES[b][0]
            )
            if vol > 1.0:
                mutual.append([a, b, round(vol, 1)])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        json.dumps(
            {"hip_heights_m": HIP_HEIGHTS_M, "candidates": results, "mutual_overlaps_mm3": mutual},
            indent=1,
        )
    )
    print("M11 PROBE mutual", mutual, flush=True)
    for name, r in results.items():
        print("M11 PROBE", name, "clear" if r["clear"] else "BLOCKED", flush=True)


if __name__ == "__main__":
    main()
