"""Generate the M11 Teensy carrier and power boards: schematic, placed PCB, pin audit, BOM and DSN.

Run with: bash tools/electronics-env.sh python3.12 tools/m11_boards.py

Board outlines and their body-frame mounting come from config/robot.json
(m10.electronics.boards, SI). KiCad's file and API boundary uses millimetres; the part
positions below are drawing coordinates on each board, not a second robot dimension
source. The generator REPLACES the native schematics and PCBs with fresh unrouted
candidates: route with Freerouting and finish with tools/m11_boards_finish.py.
"""

import csv
import itertools
import json
import math
import shutil
import subprocess
import sys
import uuid
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parents[1]
FP = ROOT / ".local-tools/root/usr/share/kicad/footprints"
OUT = ROOT / "electronics/m11"
ART = ROOT / "artifacts/m11/boards"
NS = uuid.UUID("5d0f6c55-8f0b-4f6e-9a55-1a6e2f0b3c11")
LIBNAME = "BipeM11"

R0603, C0603 = "Resistor_SMD:R_0603_1608Metric", "Capacitor_SMD:C_0603_1608Metric"
C0805, C1206 = "Capacitor_SMD:C_0805_2012Metric", "Capacitor_SMD:C_1206_3216Metric"
JST = "https://www.jst-mfg.com/product/pdf/eng/ePH.pdf"
PASSIVE = "JLC basic/extended passive, matched by value and footprint at order time"


def uid(*names):
    return str(uuid.uuid5(NS, "/".join(names)))


def quoted(value):
    return json.dumps(str(value))


def symbol(name, pins):
    """Box symbol with all pins on the left (net labels outside, names inside); 2-pin R/C/D stubs."""
    if len(pins) == 2 and all(kind == "passive" for _, _, kind in pins):
        result = (
            f'(symbol "{name}" (pin_names hide) (pin_numbers hide) (in_bom yes) (on_board yes) '
            '(property "Reference" "X" (at 0 -3.81 0) (effects (font (size 1.27 1.27)))) '
            f'(property "Value" "{name}" (at 0 3.81 0) (effects (font (size 1.27 1.27)))) '
            f'(symbol "{name}_0_1" (rectangle (start -2.54 1.016) (end 2.54 -1.016) '
            "(stroke (width 0.254) (type default)) (fill (type none)))) "
            f'(symbol "{name}_1_1" '
        )
        for number, x, angle in [(pins[0][0], -5.08, 0), (pins[1][0], 5.08, 180)]:
            result += (
                f"(pin passive line (at {x} 0 {angle}) (length 2.54) "
                f'(name "~" (effects (font (size 1 1)))) (number "{number}" (effects (font (size 1 1)))))'
            )
        return result + "))", {str(pins[0][0]): (-5.08, 0), str(pins[1][0]): (5.08, 0)}, 7.62
    height = max(5.08, math.ceil((len(pins) + 1) / 2) * 2.54)
    lines = [
        f'(symbol "{name}" (pin_names (offset 0.8)) (in_bom yes) (on_board yes)',
        '(property "Reference" "U" (at 0 5.08 0) (effects (font (size 1.27 1.27))))',
        f'(property "Value" "{name}" (at 0 2.54 0) (effects (font (size 1.27 1.27))))',
        (
            f'(symbol "{name}_0_1" (rectangle (start -10.16 {height}) (end 10.16 {-height}) '
            "(stroke (width 0.254) (type default)) (fill (type background))))"
        ),
        f'(symbol "{name}_1_1"',
    ]
    positions = {}
    for i, (number, label, kind) in enumerate(pins):
        y = height - 2.54 - i * 2.54
        positions[str(number)] = (-15.24, y)
        lines.append(
            f"(pin {kind} line (at -15.24 {y} 0) (length 5.08) "
            f"(name {quoted(label)} (effects (font (size 1.0 1.0)))) "
            f"(number {quoted(number)} (effects (font (size 1.0 1.0)))))"
        )
    lines += [")", ")"]
    return "\n".join(lines), positions, 2 * height + 5.08


def teensy_footprint(lib):
    """Teensy 4.0 on 2.54 mm headers, USB end at +x. Pads 1-14: GND, D0-D12 (top row, from the
    USB end); pads 15-28: D13-D23, 3V3, GND, VIN (bottom row, from the far end). Pads 29/30 are
    wire-link pads for the bottom pads 30 (CRX3) and 31 (CTX3), outside the outline."""
    half_l, half_w, row = 35.56 / 2, 17.78 / 2, 7.62
    text = [
        '(footprint "Teensy40_Carrier" (version 20241229) (generator "bipe")',
        '(layer "F.Cu") (attr through_hole)',
        '(fp_text reference "REF**" (at 0 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
        '(fp_text value "Teensy 4.0" (at 0 1.5) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
        '(fp_text user "USB" (at 15.5 0 90) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
    ]
    for layer, grow, width in [("F.Fab", 0, 0.1), ("F.SilkS", 0.15, 0.12)]:
        text.append(
            f"(fp_rect (start {-half_l - grow} {-half_w - grow}) (end {half_l + grow} {half_w + grow}) "
            f'(stroke (width {width}) (type solid)) (fill none) (layer "{layer}"))'
        )
    # Courtyards cover the header rows only: low SMD parts sit under the Teensy on 2.5 mm headers.
    for y in (-row, row):
        text.append(
            f"(fp_rect (start {-half_l + 0.17} {y - 1.15}) (end {half_l - 0.17} {y + 1.15}) "
            '(stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))'
        )
    pads = [(k + 1, half_l - 1.27 - 2.54 * k, -row) for k in range(14)]
    pads += [(k + 15, -half_l + 1.27 + 2.54 * k, row) for k in range(14)]
    for number, x, y in pads:
        shape = "rect" if number == 1 else "circle"
        text.append(
            f'(pad "{number}" thru_hole {shape} (at {x:.3f} {y:.3f}) (size 1.7 1.7) (drill 1.0) '
            '(layers "*.Cu" "*.Mask"))'
        )
    for number, x in [(29, 1.78), (30, 3.78)]:
        text.append(
            f'(pad "{number}" thru_hole circle (at {x} -9.85) (size 1.4 1.4) (drill 0.8) '
            '(layers "*.Cu" "*.Mask"))'
        )
    text.append(
        '(fp_text user "30 31" (at 2.78 -11.2) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))'
    )
    text.append(")")
    (lib / "Teensy40_Carrier.kicad_mod").write_text("\n".join(text))


TEENSY_PINS = (
    [(1, "GND", "power_in")]
    + [(k + 2, f"D{k}", "bidirectional") for k in range(13)]
    + [(k + 15, f"D{13 + k}", "bidirectional") for k in range(11)]
    + [
        (26, "3V3", "power_out"),
        (27, "GND", "power_in"),
        (28, "VIN", "power_in"),
        (29, "CRX3_D30", "bidirectional"),
        (30, "CTX3_D31", "bidirectional"),
    ]
)
TYPES = {
    "Teensy40": TEENSY_PINS,
    "MCP2562FD": [
        (1, "TXD", "input"),
        (2, "VSS", "power_in"),
        (3, "VDD", "power_in"),
        (4, "RXD", "output"),
        (5, "VIO", "power_in"),
        (6, "CANL", "bidirectional"),
        (7, "CANH", "bidirectional"),
        (8, "STBY", "input"),
    ],
    "LSM6DSO": [
        (1, "SDO_SA0", "output"),
        (2, "SDx", "input"),
        (3, "SCx", "input"),
        (4, "INT1", "output"),
        (5, "VDDIO", "power_in"),
        (6, "GND", "power_in"),
        (7, "GND", "power_in"),
        (8, "VDD", "power_in"),
        (9, "INT2", "output"),
        (10, "OCS_Aux", "input"),
        (11, "SDO_Aux", "output"),
        (12, "CS", "input"),
        (13, "SCL_SPC", "input"),
        (14, "SDA_SDI", "input"),
    ],
    "74AHCT1G125": [
        (1, "~{OE}", "input"),
        (2, "A", "input"),
        (3, "GND", "power_in"),
        (4, "Y", "output"),
        (5, "VCC", "power_in"),
    ],
    "BAT54S": [(1, "A1", "passive"), (2, "K2", "passive"), (3, "K1_A2", "passive")],
    "LM5060": [
        (1, "SENSE", "input"),
        (2, "VIN", "power_in"),
        (3, "OVP", "input"),
        (4, "UVLO", "input"),
        (5, "EN", "input"),
        (6, "GND", "power_in"),
        (7, "TIMER", "passive"),
        (8, "~{PGD}", "open_collector"),
        (9, "OUT", "input"),
        (10, "GATE", "output"),
    ],
    "NMOS": [(1, "G", "input"), (2, "D", "passive"), (3, "S", "passive")],
    "R": [(1, "1", "passive"), (2, "2", "passive")],
    "C": [(1, "1", "passive"), (2, "2", "passive")],
    "D": [(1, "K", "passive"), (2, "A", "passive")],
    "PWR_FLAG": [(1, "POWER", "power_out")],
}
for n in (2, 3, 7):
    TYPES[f"Conn{n}"] = [(i, str(i), "passive") for i in range(1, n + 1)]


class Board:
    def __init__(self, name, title, size_m):
        self.name, self.title = name, title
        self.width, self.height = (1000 * v for v in size_m)
        self.parts, self.holes, self.heavy, self.texts, self.hand_fit = [], [], [], [], []
        self.out = OUT / name.split("-")[-1]
        self.lib = self.out / f"{LIBNAME}.pretty"

    def add(self, ref, kind, value, footprint, xy, rot, nets, mpn="", lcsc="", source=""):
        self.parts.append(
            {
                "ref": ref,
                "kind": kind,
                "value": value,
                "footprint": footprint,
                "xy": xy,
                "rot": rot,
                "nets": {str(k): v for k, v in nets.items()},
                "mpn": mpn or value,
                "lcsc": lcsc,
                "source": source,
            }
        )

    def r(self, ref, value, xy, rot, a, b, mpn=""):
        self.add(ref, "R", value, R0603, xy, rot, {1: a, 2: b}, mpn or f"0603 {value}", "", PASSIVE)

    def c(self, ref, value, xy, rot, a, b, footprint=C0603, mpn=""):
        self.add(ref, "C", value, footprint, xy, rot, {1: a, 2: b}, mpn or value, "", PASSIVE)

    def flag(self, net):
        self.parts.append(
            {
                "ref": f"#FLG{len([p for p in self.parts if p['kind'] == 'PWR_FLAG']) + 1:02d}",
                "kind": "PWR_FLAG",
                "value": "PWR_FLAG",
                "footprint": "",
                "xy": [0, 0],
                "rot": 0,
                "nets": {"1": net},
                "mpn": "",
                "lcsc": "",
                "source": "",
            }
        )


def carrier(cfg):
    b = Board("bipe-m11-carrier", "Bipe M11 Teensy 4.0 carrier", cfg["size_m"])
    ph7 = "Connector_JST:JST_PH_B7B-PH-K_1x07_P2.00mm_Vertical"
    ph3 = "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical"
    tnets = {
        1: "GND",
        4: "CAN_STBY",
        5: "LED_DATA",
        6: "ARM_N",
        7: "STOP_N",
        9: "PWR_GOOD_N",
        11: "IMU_INT1",
        12: "IMU_CS",
        13: "SPI_MOSI",
        14: "SPI_MISO",
        15: "SPI_SCK",
        16: "BUS_ADC",
        26: "+3V3",
        27: "GND",
        28: "+5V",
        29: "CAN_RX",
        30: "CAN_TX",
    }
    b.add(
        "U1",
        "Teensy40",
        "Teensy 4.0",
        "Teensy40_Carrier",
        [21.92, 20.0],
        0,
        tnets,
        "PJRC Teensy 4.0 (VUSB-VIN pad cut)",
        "",
        "https://www.pjrc.com/store/teensy40.html",
    )
    b.add(
        "U2",
        "MCP2562FD",
        "MCP2562FD-E/SN",
        "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        [31.5, 20.0],
        0,
        {
            1: "CAN_TX",
            2: "GND",
            3: "+5V",
            4: "CAN_RX",
            5: "+3V3",
            6: "CAN_L",
            7: "CAN_H",
            8: "CAN_STBY",
        },
        "MCP2562FD-E/SN",
        "C124016",
        "Microchip DS20005284",
    )
    b.r("R1", "10k", [27.0, 15.5], 0, "CAN_STBY", "+3V3")
    b.c("C1", "100nF 16V", [27.0, 18.5], 90, "+5V", "GND")
    b.c("C2", "100nF 16V", [27.0, 22.0], 90, "+3V3", "GND")
    b.r("R2", "60.4R 1%", [36.8, 16.0], 90, "CAN_H", "CAN_MID")
    b.r("R3", "60.4R 1%", [36.8, 20.0], 90, "CAN_MID", "CAN_L")
    b.c("C3", "4.7nF 50V", [36.8, 24.0], 90, "CAN_MID", "GND")
    b.add(
        "U3",
        "LSM6DSO",
        "LSM6DSOTR",
        "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y",
        [19.0, 20.0],
        0,
        {
            1: "SPI_MISO",
            2: "GND",
            3: "GND",
            4: "IMU_INT1",
            5: "+3V3",
            6: "GND",
            7: "GND",
            8: "+3V3",
            12: "IMU_CS",
            13: "SPI_SCK",
            14: "SPI_MOSI",
        },
        "LSM6DSOTR",
        "C2655100",
        "https://www.st.com/resource/en/datasheet/lsm6dso.pdf",
    )
    b.c("C4", "100nF 16V", [22.0, 16.5], 0, "+3V3", "GND")
    b.c("C5", "100nF 16V", [22.0, 23.5], 0, "+3V3", "GND")
    b.c("C6", "1uF 16V", [15.6, 16.5], 90, "+3V3", "GND")
    b.r("R4", "10k", [16.0, 23.5], 0, "IMU_CS", "+3V3")
    b.add(
        "U4",
        "74AHCT1G125",
        "SN74AHCT1G125DBVR",
        "Package_TO_SOT_SMD:SOT-23-5",
        [10.5, 16.5],
        0,
        {1: "GND", 2: "LED_DATA", 3: "GND", 4: "LED_Y", 5: "+5V"},
        "SN74AHCT1G125DBVR",
        "C7484",
        "https://www.ti.com/lit/ds/symlink/sn74ahct1g125.pdf",
    )
    b.c("C7", "100nF 16V", [13.8, 15.25], 90, "+5V", "GND")
    b.r("R5", "330R", [13.8, 18.5], 90, "LED_Y", "LED_DIN")
    b.add(
        "D1",
        "BAT54S",
        "BAT54S",
        "Package_TO_SOT_SMD:SOT-23",
        [9.5, 23.0],
        0,
        {1: "GND", 2: "+3V3", 3: "BUS_ADC"},
        "BAT54S",
        "C2828465",
        "BAT54S series Schottky pair (clamps the ADC input)",
    )
    b.r("R6", "10k 1%", [6.5, 20.5], 90, "BUS_ADC", "GND")
    b.c("C8", "10nF 50V", [6.5, 24.0], 90, "BUS_ADC", "GND")
    b.r("R7", "10k", [12.8, 24.5], 90, "PWR_GOOD_N", "+3V3")
    b.c("C9", "10uF 10V", [6.5, 15.45], 90, "+5V", "GND", C0805)
    b.add(
        "J1",
        "Conn7",
        "SYS: +5V GND BUS_SENSE PGD_N ARM_N STOP_N GND",
        ph7,
        [2.8, 2.4],
        0,
        {1: "+5V", 2: "GND", 3: "BUS_ADC", 4: "PWR_GOOD_N", 5: "ARM_N", 6: "STOP_N", 7: "GND"},
        "B7B-PH-K-S(LF)(SN)",
        "C157981",
        JST,
    )
    b.add(
        "J2",
        "Conn3",
        "EYES: +5V DIN GND",
        ph3,
        [19.9, 2.4],
        0,
        {1: "+5V", 2: "LED_DIN", 3: "GND"},
        "B3B-PH-K-S(LF)(SN)",
        "C131339",
        JST,
    )
    b.add(
        "J3",
        "Conn3",
        "CAN: H L GND (moteus-c1 order)",
        ph3,
        [29.0, 2.4],
        0,
        {1: "CAN_H", 2: "CAN_L", 3: "GND"},
        "B3B-PH-K-S(LF)(SN)",
        "C131339",
        JST,
    )
    b.flag("+5V")
    b.flag("GND")
    b.holes = [(1.8, 8.4), (1.8, 27.6), (38.0, 3.0)]
    b.texts = [("BIPE M11 CARRIER A", (20.0, 8.0), "B.SilkS")]
    # (item, quantity, part, source): fitted by hand after JLC assembly.
    b.hand_fit = [
        (
            "Teensy 4.0, VUSB-VIN pad cut (5 V from SYS)",
            1,
            "PJRC Teensy 4.0",
            "https://www.pjrc.com/store/teensy40.html",
        ),
        ("1x14 2.54 mm male header (Teensy rows; 2.5 mm spacer clears the parts below)", 2, "", ""),
        (
            "30 AWG wire links: Teensy bottom pads 30 (CRX3) and 31 (CTX3) to U1 pads 29 and 30",
            2,
            "",
            "",
        ),
        ("PH crimp housing, 7 way (SYS)", 1, "JST PHR-7", JST),
        ("PH crimp housing, 3 way (EYES, CAN)", 2, "JST PHR-3", JST),
        ("PH crimp contact, 24-32 AWG", 13, "JST SPH-002T-P0.5S", JST),
    ]
    return b


def power(cfg):
    b = Board("bipe-m11-power", "Bipe M11 power board (LM5060 soft start)", cfg["size_m"])
    ph3 = "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical"
    w05 = "Connector_Wire:SolderWire-0.5sqmm_1x02_P4.6mm_D0.9mm_OD2.1mm"
    w025 = "Connector_Wire:SolderWire-0.25sqmm_1x02_P4.2mm_D0.65mm_OD1.7mm"
    b.add(
        "J1",
        "Conn2",
        "PACK IN (fused XT30 pigtail, 20 AWG)",
        w05,
        [2.5, 3.5],
        270,
        {1: "VBAT", 2: "GND"},
    )
    b.c("C1", "4.7uF 50V", [7.5, 7.2], 0, "VBAT", "GND", C1206)
    b.c("C2", "4.7uF 50V", [7.5, 10.4], 0, "VBAT", "GND", C1206)
    b.add(
        "Q1",
        "NMOS",
        "AOD4184A",
        "Package_TO_SOT_SMD:TO-252-2",
        [16.0, 8.0],
        180,
        {1: "GATE_Q", 2: "VBAT", 3: "BUS"},
        "AOD4184A",
        "C99124",
        "https://www.aosmd.com/res/datasheets/AOD4184A.pdf",
    )
    b.add(
        "U1",
        "LM5060",
        "LM5060MM/NOPB",
        "Package_SO:VSSOP-10_3x3mm_P0.5mm",
        [28.5, 12.0],
        0,
        {
            1: "SENSE",
            2: "VBAT",
            3: "GND",
            4: "UVLO",
            5: "EN",
            6: "GND",
            7: "TIMER",
            8: "PWR_GOOD_N",
            9: "BUS",
            10: "GATE",
        },
        "LM5060MM/NOPB",
        "C74456",
        "https://www.ti.com/lit/ds/symlink/lm5060.pdf",
    )
    b.r("R1", "10k 1%", [24.0, 9.3], 0, "VBAT", "SENSE")
    b.c("C3", "100nF 50V", [24.0, 12.8], 90, "VBAT", "GND")
    b.c("C4", "22nF 50V C0G", [33.0, 9.3], 0, "GATE", "GND")
    b.r("R2", "10R", [21.5, 14.2], 90, "GATE", "GATE_Q")
    b.c("C5", "330nF 25V", [32.5, 14.2], 90, "TIMER", "GND")
    b.r("R3", "10k", [19.5, 18.2], 0, "SW_ON", "EN")
    b.r("R4", "53.6k 1%", [23.0, 18.2], 0, "SW_ON", "UVLO")
    b.r("R5", "4.7k 1%", [26.5, 18.2], 0, "UVLO", "GND")
    b.c("C6", "10nF 50V", [30.0, 18.2], 0, "UVLO", "GND")
    b.add(
        "D1",
        "D",
        "SMBJ30A",
        "Diode_SMD:D_SMB",
        [28.0, 6.0],
        0,
        {1: "BUS", 2: "GND"},
        "SMBJ30A-13-F",
        "C135065",
        "Diodes Inc. SMBJ30A (30 V standoff, 48.4 V clamp, 600 W)",
    )
    b.add(
        "J2",
        "Conn2",
        "BUS OUT (T-splice trunk, 20 AWG)",
        w05,
        [40.0, 2.5],
        270,
        {1: "BUS", 2: "GND"},
    )
    b.add("J3", "Conn2", "BEC12S-PRO IN", w025, [46.5, 2.5], 270, {1: "BUS", 2: "GND"})
    b.r("R6", "180k 1%", [49.6, 4.6], 0, "BUS", "BUS_SENSE")
    b.add(
        "J4",
        "Conn3",
        "MON: BUS_SENSE PGD_N GND",
        ph3,
        [37.0, 13.2],
        0,
        {1: "BUS_SENSE", 2: "PWR_GOOD_N", 3: "GND"},
        "B3B-PH-K-S(LF)(SN)",
        "C131339",
        JST,
    )
    b.add(
        "J6",
        "Conn3",
        "CAN END: H L GND",
        ph3,
        [46.0, 13.2],
        0,
        {1: "CAN_H", 2: "CAN_L", 3: "GND"},
        "B3B-PH-K-S(LF)(SN)",
        "C131339",
        JST,
    )
    b.r("R7", "60.4R 1%", [43.5, 9.5], 0, "CAN_H", "CAN_MID")
    b.r("R8", "60.4R 1%", [47.0, 9.5], 0, "CAN_MID", "CAN_L")
    b.c("C7", "4.7nF 50V", [50.3, 9.6], 0, "CAN_MID", "GND")
    b.add(
        "J5",
        "Conn2",
        "SWITCH: VBAT SW_ON (rear-wall slide switch)",
        "Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical",
        [54.0, 5.5],
        0,
        {1: "VBAT", 2: "SW_ON"},
        "B2B-PH-K-S(LF)(SN)",
        "C131337",
        JST,
    )
    b.flag("VBAT")
    b.flag("GND")
    b.holes = [(3.0, 16.5), (57.5, 17.0), (14.0, 16.5)]
    # Locked heavy copper (mm, F.Cu): pack to the MOSFET tab; MOSFET source to the outputs.
    b.heavy = [
        ("VBAT", 2.0, [(2.5, 3.5), (14.7, 3.5), (14.7, 6.0)]),
        ("BUS", 2.0, [(21.04, 5.72), (21.04, 2.5), (40.0, 2.5), (46.5, 2.5)]),
    ]
    b.texts = [("BIPE M11 POWER A", (30.0, 15.0), "B.SilkS")]
    b.hand_fit = [
        ("PH crimp housing, 3 way (MON, CAN END)", 2, "JST PHR-3", JST),
        ("PH crimp housing, 2 way (SWITCH)", 1, "JST PHR-2", JST),
        ("PH crimp contact, 24-32 AWG", 8, "JST SPH-002T-P0.5S", JST),
        ("Pack lead: fused XT30 pigtail, 20 AWG, soldered to J1", 1, "", ""),
        (
            "Bus trunk lead, 20 AWG, soldered to J2; BEC12S-PRO input leads soldered to J3",
            2,
            "",
            "",
        ),
        (
            "Rear-wall miniature SPDT slide switch on the J5 lead (microamp EN/UVLO signal)",
            1,
            "",
            "",
        ),
    ]
    return b


def copy_fp(board, identifier):
    if ":" not in identifier:
        return identifier
    library, name = identifier.split(":")
    shutil.copyfile(FP / f"{library}.pretty" / f"{name}.kicad_mod", board.lib / f"{name}.kicad_mod")
    return name


def schematic(board, symbols, rootid):
    sch = [
        '(kicad_sch (version 20250114) (generator "bipe")',
        f'(uuid "{rootid}") (paper "A3")',
        (
            f'(title_block (title "{board.title}") (date "2026-10-18") (rev "A") '
            '(comment 1 "M11 phase D digital candidate: not hardware validated, not released for fabrication"))'
        ),
        "(lib_symbols",
    ]
    for kind, (text, _, _) in symbols.items():
        sch.append(text.replace(f'(symbol "{kind}"', f'(symbol "{LIBNAME}:{kind}"', 1))
    sch.append(")")
    x, y, col_w = 40.64, 30.48, 81.28
    for c in board.parts:
        kind, ref = c["kind"], c["ref"]
        _, positions, height = symbols[kind]
        if y + height > 270:
            x, y = x + col_w, 30.48
        sx, sy = round(x / 1.27) * 1.27, round((y + height / 2) / 1.27) * 1.27
        y += height + 7.62
        two = kind in ("R", "C", "D")
        in_bom = "no" if ref.startswith("#") else "yes"
        sch.append(
            f'(symbol (lib_id "{LIBNAME}:{kind}") (at {sx:.2f} {sy:.2f} 0) (unit 1) (in_bom {in_bom}) '
            f'(on_board {"no" if ref.startswith("#") else "yes"}) (dnp no) (uuid "{uid(board.name, ref)}") '
            f'(property "Reference" {quoted(ref)} (at {sx + (0 if two else 11.43):.2f} {sy - 3.81:.2f} 0) '
            "(effects (font (size 1.27 1.27)) (justify left))) "
            f'(property "Value" {quoted(c["value"])} (at {sx + (0 if two else 11.43):.2f} {sy + (3.81 if two else 0):.2f} 0) '
            "(effects (font (size 1.0 1.0)) (justify left))) "
            f'(property "Footprint" "{(LIBNAME + ":" + c["fpname"]) if c["fpname"] else ""}" '
            f"(at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.0 1.0)) hide)) "
            f'(property "MPN" {quoted(c["mpn"])} (at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.0 1.0)) hide)) '
            f'(property "LCSC" {quoted(c["lcsc"])} (at {sx:.2f} {sy:.2f} 0) (effects (font (size 1.0 1.0)) hide))'
        )
        for number in positions:
            sch.append(f'(pin "{number}" (uuid "{uid(board.name, ref, "pin", number)}"))')
        sch.append(
            f'(instances (project "{board.name}" (path "/{rootid}" (reference "{ref}") (unit 1)))) )'
        )
        for number, (px, py) in positions.items():
            a, bb = sx + px, sy - py
            if number not in c["nets"]:
                sch.append(
                    f'(no_connect (at {a:.2f} {bb:.2f}) (uuid "{uid(board.name, ref, "nc", number)}"))'
                )
                continue
            net = c["nets"][number]
            end = a + 3.81 if px > 0 else a - 3.81
            sch.append(
                f"(wire (pts (xy {a:.2f} {bb:.2f}) (xy {end:.2f} {bb:.2f})) (stroke (width 0) (type default)) "
                f'(uuid "{uid(board.name, ref, "wire", number)}"))'
            )
            sch.append(
                f'(label "{net}" (at {end:.2f} {bb:.2f} 0) (effects (font (size 0.9 0.9)) '
                f'(justify {"left" if px > 0 else "right"} bottom)) (uuid "{uid(board.name, ref, "label", number)}"))'
            )
    sch += ['(sheet_instances (path "/" (page "1")))', ")"]
    return "\n".join(sch) + "\n"


def mm(v):
    return pcbnew.FromMM(v)


def place(board, rootid):
    # Seeded KiCad UUIDs, so a rerun reproduces the committed files.
    pcbnew.KIID.SeedGenerator(sum(board.name.encode()))
    pcb = pcbnew.BOARD()
    pcb.SetCopperLayerCount(2)
    pcb.GetDesignSettings().SetBoardThickness(mm(1.6))
    names = sorted({n for c in board.parts for n in c["nets"].values()})
    nets = {}
    for name in names:
        net = pcbnew.NETINFO_ITEM(pcb, "/" + name)
        pcb.Add(net)
        nets[name] = net
    for c in board.parts:
        if not c["fpname"]:
            continue
        foot = pcbnew.FootprintLoad(str(board.lib), c["fpname"])
        assert foot is not None, c["fpname"]
        foot.SetReference(c["ref"])
        foot.SetValue(c["value"])
        foot.SetFPID(pcbnew.LIB_ID(LIBNAME, c["fpname"]))
        foot.SetPath(pcbnew.KIID_PATH(f"/{rootid}/{uid(board.name, c['ref'])}"))
        pcb.Add(foot)
        foot.SetOrientationDegrees(c["rot"])
        x, y = c["xy"]
        foot.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        foot.Value().SetVisible(False)
        foot.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.7), mm(0.7)))
        foot.Reference().SetTextThickness(mm(0.1))
        foot.Reference().SetLayer(pcbnew.F_Fab)
        foot.SetAttributes(foot.GetAttributes() & ~pcbnew.FP_EXCLUDE_FROM_BOM)
        labels = {str(n): label for n, label, _ in TYPES[c["kind"]]}
        for pad in foot.Pads():
            number = pad.GetNumber()
            if number in c["nets"]:
                pad.SetNet(nets[c["nets"][number]])
            elif number in labels:
                # The schematic's no-connect pins carry KiCad's generated net names.
                name = f"unconnected-({c['ref']}-{labels[number]}-Pad{number})"
                net = pcbnew.NETINFO_ITEM(pcb, name)
                pcb.Add(net)
                pad.SetNet(net)
    corners = [(0, 0), (board.width, 0), (board.width, board.height), (0, board.height)]
    for a, b in zip(corners, corners[1:] + corners[:1]):
        edge = pcbnew.PCB_SHAPE(pcb)
        edge.SetShape(pcbnew.SHAPE_T_SEGMENT)
        edge.SetLayer(pcbnew.Edge_Cuts)
        edge.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        edge.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        edge.SetWidth(mm(0.05))
        pcb.Add(edge)
    for i, (x, y) in enumerate(board.holes):
        f = pcbnew.FootprintLoad(str(FP / "MountingHole.pretty"), "MountingHole_2.2mm_M2")
        f.SetReference(f"H{i + 1}")
        f.SetBoardOnly(True)
        pcb.Add(f)
        f.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        f.Reference().SetVisible(False)
        f.Value().SetVisible(False)
    for net, width, points in board.heavy:
        for a, b in itertools.pairwise(points):
            t = pcbnew.PCB_TRACK(pcb)
            t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            t.SetWidth(mm(width))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(nets[net])
            t.SetLocked(True)
            pcb.Add(t)
    for text, (x, y), layer in board.texts:
        t = pcbnew.PCB_TEXT(pcb)
        t.SetText(text)
        t.SetLayer({"F.SilkS": pcbnew.F_SilkS, "B.SilkS": pcbnew.B_SilkS}[layer])
        t.SetMirrored(layer.startswith("B."))
        t.SetTextSize(pcbnew.VECTOR2I(mm(1.0), mm(1.0)))
        t.SetTextThickness(mm(0.15))
        t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        pcb.Add(t)
    return pcb, names


def project_file(board, power_nets):
    return {
        "meta": {"filename": board.name + ".kicad_pro", "version": 1},
        "board": {
            "design_settings": {
                "rules": {
                    "min_clearance": 0.15,
                    "min_track_width": 0.15,
                    "min_via_diameter": 0.6,
                    "min_through_hole_diameter": 0.3,
                    "min_hole_clearance": 0.25,
                    "min_hole_to_hole": 0.25,
                    "min_copper_edge_clearance": 0.4,
                }
            }
        },
        "net_settings": {
            "classes": [
                {
                    "name": "Default",
                    "clearance": 0.15,
                    "track_width": 0.25,
                    "via_diameter": 0.6,
                    "via_drill": 0.3,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "diff_pair_width": 0.2,
                    "diff_pair_gap": 0.2,
                    "diff_pair_via_gap": 0.25,
                },
                {
                    "name": "Power",
                    "clearance": 0.15,
                    "track_width": 0.4,
                    "via_diameter": 0.8,
                    "via_drill": 0.4,
                    "microvia_diameter": 0.3,
                    "microvia_drill": 0.1,
                    "diff_pair_width": 0.2,
                    "diff_pair_gap": 0.2,
                    "diff_pair_via_gap": 0.25,
                },
            ],
            "netclass_patterns": [{"netclass": "Power", "pattern": "/" + n} for n in power_nets],
        },
        "schematic": {},
    }


def emit(board, power_nets):
    if board.out.exists():
        shutil.rmtree(board.out)
    board.lib.mkdir(parents=True)
    art = ART / board.name.split("-")[-1]
    art.mkdir(parents=True, exist_ok=True)
    teensy_footprint(board.lib)
    for c in board.parts:
        c["fpname"] = copy_fp(board, c["footprint"]) if c["footprint"] else ""
    kinds = sorted({c["kind"] for c in board.parts})
    symbols = {k: symbol(k, TYPES[k]) for k in kinds}
    (board.out / f"{LIBNAME}.kicad_sym").write_text(
        '(kicad_symbol_lib (version 20241209) (generator "bipe")\n'
        + "\n".join(s[0] for s in symbols.values())
        + ")\n"
    )
    (board.out / "sym-lib-table").write_text(
        f'(sym_lib_table (lib (name "{LIBNAME}") (type "KiCad") (uri "${{KIPRJMOD}}/{LIBNAME}.kicad_sym") (options "") (descr "")))\n'
    )
    (board.out / "fp-lib-table").write_text(
        f'(fp_lib_table (lib (name "{LIBNAME}") (type "KiCad") (uri "${{KIPRJMOD}}/{LIBNAME}.pretty") (options "") (descr "")))\n'
    )
    rootid = uid(board.name, "schematic")
    (board.out / f"{board.name}.kicad_sch").write_text(schematic(board, symbols, rootid))
    pcb, names = place(board, rootid)
    path = board.out / f"{board.name}.kicad_pcb"
    pcbnew.SaveBoard(str(path), pcb)
    # SaveBoard writes KiCad's default project; the rules and net classes are merged into it.
    pro = board.out / f"{board.name}.kicad_pro"
    data = json.loads(pro.read_text())
    ours = project_file(board, power_nets)
    data["board"]["design_settings"]["rules"].update(ours["board"]["design_settings"]["rules"])
    default = data["net_settings"]["classes"][0]
    data["net_settings"]["classes"] = [
        dict(default, **c, priority=default["priority"] if c["name"] == "Default" else 0)
        for c in ours["net_settings"]["classes"]
    ]
    data["net_settings"]["netclass_patterns"] = ours["net_settings"]["netclass_patterns"]
    pro.write_text(json.dumps(data, indent=2) + "\n")
    # pcbnew keeps the saved project cached in this process: export from a fresh one so the
    # DSN carries the merged net classes.
    export = "import pcbnew, sys; assert pcbnew.ExportSpecctraDSN(pcbnew.LoadBoard(sys.argv[1]), sys.argv[2])"
    subprocess.run([sys.executable, "-c", export, str(path), str(art / "unrouted.dsn")], check=True)
    parts = [c for c in board.parts if c["fpname"]]
    (art / "pin-audit.json").write_text(
        json.dumps(
            {
                "board": board.name,
                "size_mm": [board.width, board.height],
                "components": board.parts,
                "net_names": names,
                "power_nets": power_nets,
                "status": "M11 phase D digital candidate; not hardware validated or released",
            },
            indent=2,
        )
        + "\n"
    )
    with (art / "bom-jlc.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["Comment", "Designator", "Footprint", "LCSC Part #", "MPN", "Source"])
        groups = {}
        for c in parts:
            if c["fpname"].startswith("SolderWire"):
                continue  # wire pads: no part
            key = (c["value"], c["fpname"], c["lcsc"], c["mpn"], c["source"])
            groups.setdefault(key, []).append(c["ref"])
        for (value, fpname, lcsc, mpn, source), refs in sorted(
            groups.items(), key=lambda kv: kv[1][0]
        ):
            writer.writerow([value, ",".join(refs), fpname, lcsc, mpn, source])
    with (art / "bom-hand-fit.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["Item", "Quantity", "Part", "Source"])
        writer.writerows(board.hand_fit)
    return {"board": board.name, "parts": len(parts), "nets": len(names)}


def main():
    cfg = json.loads((ROOT / "config/robot.json").read_text())["m10"]["electronics"]["boards"]
    results = [
        emit(carrier(cfg["carrier"]), ["+5V"]),
        emit(power(cfg["power"]), ["VBAT", "BUS"]),
    ]
    print(json.dumps(results))


if __name__ == "__main__":
    main()
