"""M11 firmware hardware layer: host build and ctypes bindings for firmware/src/m11.

The portable C sources compile into one shared library for the parity tests
(tests/test_m11_firmware.py). The structures mirror firmware/include/m11_*.h.
"""

import ctypes as c
import hashlib
import itertools
import os
import subprocess
from pathlib import Path

import numpy as np

from ..parameters import ROOT

SOURCES = sorted((ROOT / "firmware/src/m11").glob("*.c"))
HEADERS = sorted((ROOT / "firmware/include").glob("m11_*.h"))
FD_MAX = 64
POSITION_REGISTERS = QUERY_REGISTERS = 14
REPLY_REGISTERS = 16


class FdFrame(c.Structure):
    _fields_ = [("id", c.c_uint32), ("size", c.c_uint8), ("data", c.c_uint8 * FD_MAX)]

    def payload(self):
        return bytes(self.data[: self.size])


class MxPosition(c.Structure):
    _fields_ = [
        ("value", c.c_double * POSITION_REGISTERS),
        ("resolution", c.c_uint8 * POSITION_REGISTERS),
    ]


class MxQuery(c.Structure):
    _fields_ = [("resolution", c.c_uint8 * QUERY_REGISTERS)]


class MxReply(c.Structure):
    _fields_ = [
        ("source", c.c_uint8),
        ("destination", c.c_uint8),
        ("present", c.c_uint16),
        ("value", c.c_double * REPLY_REGISTERS),
    ]


def m10_inputs():
    import json

    from ..parameters import load_parameters

    return load_parameters(profile="m10"), json.loads(
        (ROOT / "config/controller_m10.json").read_text()
    )


def write_m10_config(path):
    """generated_config.h for the M10 profile and controller (the M11 image's core)."""
    from tools.generate_firmware_config import generate

    generate(*m10_inputs(), Path(path))


def build(directory):
    """Compile firmware/src/m11 with the supervisor into a shared library; returns its CDLL.

    The supervisor reads generated_config.h, generated here for the M10 profile and the
    M10 controller, as the M10 parity tests do.
    """
    import json

    nominal, gains = m10_inputs()
    supervisor = ROOT / "firmware/src/supervisor.c"
    identity = hashlib.sha256(json.dumps([nominal, gains], sort_keys=True).encode())
    for path in [
        *SOURCES,
        *HEADERS,
        supervisor,
        ROOT / "firmware/include/bipe.h",
        ROOT / "tools/generate_firmware_config.py",
    ]:
        identity.update(path.read_bytes())
    out = Path(directory).resolve() / identity.hexdigest()[:16]
    lib = out / "libbipe_m11.so"
    if not lib.exists():
        out.mkdir(parents=True, exist_ok=True)
        write_m10_config(out / "generated_config.h")
        tmp = out / f".libbipe_m11.{os.getpid()}.so"
        subprocess.run(
            [
                "gcc",
                "-std=c11",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wconversion",
                "-Wshadow",
                "-fPIC",
                "-shared",
                "-I" + str(out),
                "-I" + str(ROOT / "firmware/include"),
                *map(str, SOURCES),
                str(supervisor),
                "-lm",
                "-o",
                str(tmp),
            ],
            check=True,
        )
        tmp.replace(lib)  # atomic for parallel workers
    dll = c.CDLL(str(lib))
    dll.bipe_mx_default_query.restype = MxQuery
    dll.bipe_mx_empty_position.restype = MxPosition
    for name in (
        "bipe_mx_position_frame",
        "bipe_mx_query_frame",
        "bipe_mx_stop_frame",
        "bipe_mx_parse",
        "bipe_fd_pad",
        "bipe_m11_gravity_ok",
        "bipe_m11_joint_command",
        "bipe_m11_wheel_command",
        "bipe_m11_wheel_speed_command",
        "bipe_m11_query",
        "bipe_m11_stop",
        "bipe_m11_sample",
    ):
        getattr(dll, name).restype = c.c_bool
    dll.bipe_m11_bus_volts.restype = c.c_double
    dll.bipe_m11_joint_command.argtypes = [
        c.c_void_p,
        c.c_double,
        c.c_double,
        c.c_double,
        c.c_void_p,
    ]
    dll.bipe_m11_wheel_command.argtypes = [c.c_void_p, c.c_double, c.c_void_p]
    dll.bipe_m11_wheel_speed_command.argtypes = [c.c_void_p, c.c_double, c.c_double, c.c_void_p]
    return dll


class Axis(c.Structure):
    _fields_ = [
        ("id", c.c_uint8),
        ("wheel", c.c_bool),
        ("sign", c.c_double),
        ("centre_rad", c.c_double),
    ]


class MotorSample(c.Structure):
    """bipe.h bipe_motor_sample."""

    _fields_ = [
        ("present", c.c_bool),
        ("valid", c.c_bool),
        ("time_ms", c.c_uint32),
        ("sequence", c.c_uint32),
        ("position", c.c_double),
        ("speed", c.c_double),
        ("temperature", c.c_double),
        ("id", c.c_uint8),
        ("error", c.c_uint8),
    ]


# --- Hardware description shared by the generator, the driver configuration and tests ---

CARRIER_PCB = ROOT / "electronics/m11/carrier/bipe-m11-carrier.kicad_pcb"
CARRIER_SYM = ROOT / "electronics/m11/carrier/BipeM11.kicad_sym"
PIN_NETS = (
    "CAN_STBY",
    "LED_DATA",
    "ARM_N",
    "STOP_N",
    "PWR_GOOD_N",
    "IMU_INT1",
    "IMU_CS",
    "SPI_MOSI",
    "SPI_MISO",
    "SPI_SCK",
    "BUS_ADC",
    "CAN_RX",
    "CAN_TX",
)


def _footprint(pcb_text, name):
    start = pcb_text.index(f'(footprint "{name}"')
    depth, i = 0, start
    while True:
        depth += {"(": 1, ")": -1}.get(pcb_text[i], 0)
        i += 1
        if depth == 0:
            return pcb_text[start:i]


def teensy_pins():
    """Teensy 4.0 pin number per carrier net, from the routed board and its symbol.

    The symbol names each U1 pad (D0..D23, CRX3_D30, CTX3_D31); the board gives each
    pad's net, so the firmware pin map cannot drift from the layout.
    """
    import re

    symbol = CARRIER_SYM.read_text()
    unit = symbol[symbol.index('(symbol "Teensy40_1_1"') :]
    names = {
        number: name for name, number in re.findall(r'\(name "([^"]+)".*?\(number "(\d+)"', unit)
    }
    footprint = _footprint(CARRIER_PCB.read_text(), "BipeM11:Teensy40_Carrier")
    pins = {}
    starts = [m.start() for m in re.finditer(r'\(pad "', footprint)] + [len(footprint)]
    for a, b in itertools.pairwise(starts):
        pad = re.match(r'\(pad "(\d+)"', footprint[a:b]).group(1)
        net = re.search(r'\(net \d+ "/?([^"]+)"\)', footprint[a:b]).group(1)
        if net in PIN_NETS:
            match = re.fullmatch(r"(?:\w+_)?D(\d+)", names[pad])
            pins[net] = int(match.group(1))
    missing = set(PIN_NETS) - set(pins)
    if missing:
        raise ValueError(f"carrier nets without a Teensy pin: {sorted(missing)}")
    return pins


def imu_rotation(m10):
    """Rotation (3 x 3, rows: body x, y, z) from LSM6DSO chip axes to the body frame.

    ST DS12140 Rev 3, Figure 3 (top view): pin 1 top left, X right, Y up, Z out of the
    top face. KiCad's LGA-14 footprint is drawn the same way (pad 1 at -x, -y with y
    down), so chip X is footprint +x, chip Y is footprint -y and chip Z is the copper
    side's normal. The footprint's rotation and side come from the routed board, the
    board axes from m10.electronics.boards.
    """
    import re

    footprint = _footprint(
        CARRIER_PCB.read_text(), "BipeM11:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
    )
    if '(layer "F.Cu")' not in footprint.splitlines()[1]:
        raise ValueError("U3 must sit on the front copper")
    at = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", footprint)
    pad1 = re.search(r'\(pad "1" \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+)', footprint)
    if not (float(pad1.group(1)) < 0 and float(pad1.group(2)) < 0):
        raise ValueError("footprint pad 1 is not at the datasheet's top-left corner")
    angle = np.radians(float(at.group(3) or 0.0))
    # KiCad rotates counter-clockwise on screen; with y down that maps local (x, y) to
    # board (x cos + y sin, -x sin + y cos).
    rot = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
    board = m10["electronics"]["boards"]["carrier"]
    ax, ay, n = (
        np.array(board[k], float)
        for k in ("board_x_axis_body", "board_y_axis_body", "front_normal_body")
    )
    chip_x = rot @ [1.0, 0.0]
    chip_y = rot @ [0.0, -1.0]
    columns = [chip_x[0] * ax + chip_x[1] * ay, chip_y[0] * ax + chip_y[1] * ay, n]
    r = np.column_stack(columns)
    if not np.allclose(r @ r.T, np.eye(3)) or np.linalg.det(r) < 0.999:
        raise ValueError("IMU axes are not a right-handed rotation")
    return np.round(r, 12) + 0.0


def axes(p, gains):
    """One row per moteus driver in JOINTS + WHEELS order: id, kind and unit mapping.

    Joints: q = centre + sign 2 pi position_rev, with the moteus output offset set at
    commissioning so the end-stop centre reads 0; a 4:1 belt and the rotor encoder
    give an unambiguous output angle over +-45 deg. Wheels: direct drive. Signs are
    commissioning values (identity here).
    """
    import json

    from ..articulated import JOINTS, WHEELS

    # Wheels: kd is the M13 policy's wheel speed loop gain, so the learned controller's
    # proportional term runs on the drive at its own rate (velocity commands with kd
    # scale 1); the classical controller sends torque with kp and kd scale 0.
    from ..m13.headers import DEPLOYED, POLICIES

    manifest = json.loads((ROOT / POLICIES[DEPLOYED]["export"] / "manifest.json").read_text())
    wheel_kv = manifest["action"]["wheels"]["speed_loop"]["kv_Nm_s_per_rad"]
    m10 = p.get("m10", p)
    fw = m10["electronics"]["firmware"]
    stops = m10["physics"]["end_stops_deg"]
    hip, hub = m10["physics"]["hip_motor"], m10["physics"]["hub_motor"]
    rows = []
    for name in JOINTS + WHEELS:
        wheel = name in WHEELS
        if wheel:
            lo = hi = 0.0
        else:
            lo, hi = stops["thigh_a1" if name.endswith("hip") else "knee_drive_a2"]
        rows.append(
            {
                "name": name,
                "id": fw["driver_ids"][name],
                "wheel": wheel,
                "sign": 1.0,
                "centre_rad": float(np.radians((lo + hi) / 2)),
                "half_range_rad": float(np.radians((hi - lo) / 2)),
                "rotor_to_output_ratio": 1.0 if wheel else 1.0 / hip["ratio"],
                "max_current_A": hub["peak_current_A"] if wheel else hip["peak_current_A"],
                "kp_Nm_per_rev": 0.0 if wheel else 2 * np.pi * gains["joint_kp"],
                "kd_Nm_per_rev_s": 2 * np.pi * (wheel_kv if wheel else gains["joint_kd"]),
            }
        )
    return rows
