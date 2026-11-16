"""M11 moteus CAN-FD codec: byte parity with mjbots' moteus Python package (1.1.1).

The package is a test-only reference (requirements-moteus.lock). The firmware frames must
match its make_position / make_query / make_stop bytes and its reply parser.
"""

import ctypes as c
import io
import math

import numpy as np
import pytest

moteus = pytest.importorskip("moteus")
mp = pytest.importorskip("moteus.multiplex")
protocol = pytest.importorskip("moteus.protocol")

from bipe.m11.firmware import FdFrame, MxPosition, MxQuery, MxReply, build

FIELDS = [
    "position",
    "velocity",
    "feedforward_torque",
    "kp_scale",
    "kd_scale",
    "maximum_torque",
    "stop_position",
    "watchdog_timeout",
    "velocity_limit",
    "accel_limit",
    "fixed_voltage_override",
    "ilimit_scale",
    "fixed_current_override",
    "ignore_position_bounds",
]
QUERY_FIELDS = [
    "mode",
    "position",
    "velocity",
    "torque",
    "q_current",
    "d_current",
    "abs_position",
    "power",
    "motor_temperature",
    "trajectory_complete",
    "home_state",
    "voltage",
    "temperature",
    "fault",
]
# Reply registers 0x000-0x00f the firmware decodes; 0x008/9 are never requested.
REPLY_REGISTERS = [r for r in range(16) if r not in (8, 9)]
INTEGER_REGISTERS = {0, 0xB, 0xC, 0xF}
FD_SIZES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build(tmp_path_factory.mktemp("m11"))


def resolution_class(base, fields, values):
    cls = type("R", (base,), {})
    for field, value in zip(fields, values):
        setattr(cls, field, value)
    return cls


def random_value(rng, field):
    if field == "ignore_position_bounds":
        return float(rng.integers(0, 2))
    roll = rng.random()
    if roll < 0.05:
        return math.nan
    if roll < 0.1:
        return float(rng.choice([-1, 1]) * 10 ** rng.uniform(3, 12))  # saturates
    return float(rng.normal(0, [0.3, 3, 30][rng.integers(0, 3)]))


def expected_padded(data):
    size = next(s for s in FD_SIZES if s >= len(data))
    return data + b"\x50" * (size - len(data))


def check(frame, command, destination, source):
    reply = command.reply_required
    assert frame.id == destination | (0x8000 if reply else 0) | source << 8
    assert frame.payload() == expected_padded(command.data)


def c_query(resolutions):
    q = MxQuery()
    for i, r in enumerate(resolutions):
        q.resolution[i] = r
    return q


def test_default_query_matches_the_package_default(lib):
    q = lib.bipe_mx_default_query()
    default = moteus.QueryResolution
    assert list(q.resolution) == [getattr(default, f) for f in QUERY_FIELDS]


def test_position_query_and_stop_frames_match_the_package_bytes(lib):
    rng = np.random.default_rng(20261019)
    resolutions = [mp.INT8, mp.INT16, mp.INT32, mp.F32]
    for trial in range(3000):
        destination, source = int(rng.integers(1, 128)), int(rng.integers(0, 128))
        pres = [int(rng.choice(resolutions)) for _ in FIELDS]
        qres = [int(rng.choice(resolutions + [mp.IGNORE] * 2)) for _ in QUERY_FIELDS]
        if trial % 7 == 0:
            qres = [getattr(moteus.QueryResolution, f) for f in QUERY_FIELDS]
        # The package merges rezero_state and home_state; keep rezero ignored.
        controller = moteus.Controller(
            id=destination,
            source_can_id=source,
            position_resolution=resolution_class(moteus.PositionResolution, FIELDS, pres),
            query_resolution=resolution_class(moteus.QueryResolution, QUERY_FIELDS, qres),
        )
        sent = [f for f in FIELDS if rng.random() < 0.5]
        values = {f: random_value(rng, f) for f in sent}
        query = bool(rng.random() < 0.6)
        command = controller.make_position(query=query, **values)
        position = MxPosition()
        for i, f in enumerate(FIELDS):
            position.resolution[i] = pres[i] if f in sent else mp.IGNORE
            position.value[i] = values.get(f, 0.0)
        q = c_query(qres)
        frame = FdFrame()
        built = lib.bipe_mx_position_frame(
            destination,
            source,
            c.byref(position),
            c.byref(q) if query else None,
            c.byref(frame),
        )
        # Over 64 bytes is not a CAN-FD frame; the firmware refuses it.
        assert built == (len(command.data) <= 64)
        if built:
            check(frame, command, destination, source)
        if any(r != mp.IGNORE for r in qres):
            assert lib.bipe_mx_query_frame(destination, source, c.byref(q), c.byref(frame))
            check(frame, controller.make_query(), destination, source)
        assert lib.bipe_mx_stop_frame(
            destination, source, c.byref(q) if query else None, c.byref(frame)
        )
        check(frame, controller.make_stop(query=query), destination, source)


def reply_bytes(rng, registers):
    """A reply frame written with the package's own combiner and scaled writers."""
    buf = io.BytesIO()
    writer = protocol.Writer(buf)
    resolutions = {r: int(rng.integers(0, 4)) for r in registers}
    values = {}
    for start, stop in ((0, 8), (10, 16)):
        span = [resolutions.get(r, mp.IGNORE) for r in range(start, stop)]
        combiner = mp.WriteCombiner(writer, 0x20, start, span)
        for r in range(start, stop):
            if not combiner.maybe_write():
                continue
            res = resolutions[r]
            if r in INTEGER_REGISTERS:
                value = int(rng.integers(0, 100))
                writer.write(value if res != mp.F32 else float(value), res)
            else:
                value = float(rng.normal(0, [0.4, 4, 40][rng.integers(0, 3)]))
                if rng.random() < 0.05:
                    value = math.nan
                write = {
                    1: writer.write_position,
                    2: writer.write_velocity,
                    3: writer.write_torque,
                    4: writer.write_current,
                    5: writer.write_current,
                    6: writer.write_position,
                    7: writer.write_power,
                    10: writer.write_temperature,
                    13: writer.write_voltage,
                    14: writer.write_temperature,
                }[r]
                write(value, res)
            values[r] = value
    return buf.getvalue()


def test_reply_parser_matches_the_package_parser(lib):
    rng = np.random.default_rng(7)
    host = 0
    for _ in range(3000):
        registers = [r for r in REPLY_REGISTERS if rng.random() < 0.5] or [1]
        data = reply_bytes(rng, registers)
        frame = FdFrame(id=(int(rng.integers(1, 128)) << 8) | host, size=len(data))
        for i, b in enumerate(data):
            frame.data[i] = b
        assert lib.bipe_fd_pad(c.byref(frame))
        reply = MxReply()
        assert lib.bipe_mx_parse(c.byref(frame), host, c.byref(reply))
        expected = protocol.parse_reply(data)
        assert reply.source == frame.id >> 8 & 0x7F
        assert {r for r in range(16) if reply.present >> r & 1} == {int(k) for k in expected}
        for register, value in expected.items():
            got = reply.value[int(register)]
            if isinstance(value, float) and math.isnan(value):
                assert math.isnan(got)
            else:
                assert got == pytest.approx(value, rel=1e-12, abs=1e-12)


def test_parser_rejects_foreign_error_and_truncated_frames(lib):
    good = bytes.fromhex("2404000a005000000170ff230d181400")  # moteus can.md example reply
    frame = FdFrame(id=0x100, size=len(good))
    for i, b in enumerate(good):
        frame.data[i] = b
    reply = MxReply()
    assert lib.bipe_mx_parse(c.byref(frame), 0, c.byref(reply))
    assert reply.value[0] == 10 and reply.value[1] == pytest.approx(0.008)
    assert reply.value[3] == pytest.approx(-1.44) and reply.value[13] == pytest.approx(12)
    assert not lib.bipe_mx_parse(c.byref(frame), 5, c.byref(reply)), "wrong destination"
    for bad in ("310102", "2404000a00", "1101", "24040020", "2101080a"):
        data = bytes.fromhex(bad)
        frame = FdFrame(id=0x100, size=len(data))
        for i, b in enumerate(data):
            frame.data[i] = b
        assert not lib.bipe_mx_parse(c.byref(frame), 0, c.byref(reply)), bad


def test_example_command_from_the_protocol_doc(lib):
    """moteus can.md: mode 10, int16 position/velocity/torque, query 0x000-0x003 and
    0x00d-0x00f, to id 1 with a reply: 01000a07206000200150ff140400130d."""
    position = MxPosition()
    for i in range(14):
        position.resolution[i] = mp.IGNORE
    for i, v in enumerate([0.00961, 0.07201, -1.7601]):
        position.resolution[i], position.value[i] = mp.INT16, v
    q = c_query([mp.INT16] * 4 + [mp.IGNORE] * 7 + [mp.INT8] * 3)
    frame = FdFrame()
    assert lib.bipe_mx_position_frame(1, 0, c.byref(position), c.byref(q), c.byref(frame))
    assert frame.id == 0x8001
    assert frame.payload()[:16].hex() == "01000a07206000200150ff140400130d"
