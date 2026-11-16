"""M11 bus schedule and sample-to-actuation latency budget against the plant's 5 ms.

.venv/bin/python -m bipe.m11.timing

One control cycle at the 200 Hz tick: read the LSM6DSO burst, query the six drivers,
compute, then send four joint and two wheel commands. The plant's transport_delay_s
covers sample to actuation (the gyro anti-alias and FOC lag are modelled separately),
so the budget is the age of the oldest sample (the IMU) when the last command takes
effect. CAN-FD frame times use ISO 11898-1:2015 bit counts for an extended identifier
with bit-rate switching and worst-case dynamic stuffing; frame sizes come from the
firmware codec itself. Writes artifacts/m11/firmware/timing.json.
"""

import ctypes as c
import json
import statistics
import time

import numpy as np

from ..parameters import ROOT, load_parameters
from .firmware import Axis, FdFrame, MotorSample, build

OUTPUT = ROOT / "artifacts/m11/firmware/timing.json"
FD_SIZES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def fd_padded(size):
    return next(s for s in FD_SIZES if s >= size)


def frame_time_s(payload_bytes, nominal_bit_s, data_bit_s):
    """Worst-case CAN-FD frame time, extended identifier, BRS set, plus intermission.

    Nominal rate: SOF, 11-bit base ID, SRR, IDE, 18-bit extension, RRS, FDF, res, BRS
    (36 bits) with up to one stuff bit per four, then CRC delimiter, ACK slot, ACK
    delimiter, EOF and intermission (13 bits). Data rate: ESI, DLC and the payload with
    dynamic stuffing, the stuff count (4 bits) and the CRC (17 bits up to 16 bytes,
    21 above) with a leading fixed stuff bit and one after every four further bits.
    """
    if payload_bytes not in FD_SIZES:
        raise ValueError("payload must be a CAN-FD frame size")
    arbitration = 36 + (36 - 1) // 4
    tail = 13
    dynamic = 5 + 8 * payload_bytes
    crc = 17 if payload_bytes <= 16 else 21
    data = dynamic + (dynamic - 1) // 4 + 4 + crc + 1 + (4 + crc - 1) // 4
    return (arbitration + tail) / nominal_bit_s + data / data_bit_s


def reply_bytes(query_resolutions):
    """Size of a moteus reply to the default query: per run of equal resolution, one
    0x2X header (count in the low bits up to 3, else one count byte), the varuint start
    register and the values (moteus docs/reference/can.md; the WriteCombiner groups)."""
    size_of = {0: 1, 1: 2, 2: 4, 3: 4}
    registers = list(range(0x008)) + list(range(0x00A, 0x010))
    total, i = 0, 0
    while i < len(registers):
        res = query_resolutions[i]
        j = i
        while (
            j + 1 < len(registers)
            and query_resolutions[j + 1] == res
            and registers[j + 1] == registers[j] + 1
        ):
            j += 1
        if res != 4:  # ignored registers are not reported
            count = j - i + 1
            total += (1 if count <= 3 else 2) + 1 + count * size_of[res]
        i = j + 1
    return fd_padded(total)


def frame_sizes(lib):
    rows = {}
    frame = FdFrame()
    table = (Axis * 6).in_dll(lib, "bipe_m11_axes")
    assert lib.bipe_m11_joint_command(
        c.byref(table[0]), table[0].centre_rad, 0.0, 0.0, c.byref(frame)
    )
    rows["joint_command"] = frame.size
    assert lib.bipe_m11_wheel_command(c.byref(table[4]), 0.0, c.byref(frame))
    rows["wheel_command"] = frame.size
    assert lib.bipe_m11_query(c.byref(table[0]), c.byref(frame))
    rows["query"] = frame.size
    rows["reply"] = reply_bytes(list(lib.bipe_mx_default_query().resolution))
    return rows


def host_cycle_s(m11, repeats=20000):
    """Host time of one cycle's firmware work through ctypes (an upper bound on the C
    itself): LSM6DSO conversion, six reply decodes, the M10 estimator and controller
    steps and six command encodes."""
    from ..firmware_control import Balance, Estimate, EstimatorState, SensorConfig, control_core
    from ..kinematics import posture
    from ..robustness_control import build_candidate

    p = load_parameters(profile="m10")
    gains = json.loads((ROOT / "config/controller_m10.json").read_text())
    sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
    core = control_core(build_candidate(p, gains, ROOT / ".local-tools/m10-firmware"))
    d3, d4, d6 = c.c_double * 3, c.c_double * 4, c.c_double * 6
    q = posture(40, p)
    encoders = d6(*q, 0.0, 0.0)
    gravity = d3(0.0, 0.0, p["sizing"]["gravity"])
    config = SensorConfig(
        p["simulation"]["control_dt"],
        sensors["velocity_filter_tau_s"],
        sensors["joint_velocity_filter_tau_s"],
        sensors["posture_filter_tau_s"],
        sensors["attitude_correction_tau_s"],
        sensors["accel_norm_gate_m_s2"],
        sensors["transport_delay_s"] + sensors["gyro_anti_alias_tau_s"],
        sensors["local_joint_velocity_filter_tau_s"],
        p["sizing"]["gravity"],
        p["geometry"]["wheel_radius"],
        sensors["imu_velocity_fusion_tau_s"],
        sensors["imu_velocity_innovation_limit_m_s"],
        sensors["imu_accel_bias_tau_s"],
    )
    state, estimate, balance = EstimatorState(), Estimate(), Balance()
    if not core.bipe_estimator_init(c.byref(state), gravity, encoders):
        raise RuntimeError("estimator init rejected the bench posture")
    table = (Axis * 6).in_dll(m11, "bipe_m11_axes")
    raw = (c.c_uint8 * 12).from_buffer_copy(np.array([0, 0, 0, 0, -8198, 0], "<i2").tobytes())
    gyro, accel, still = d3(), d3(), d3()
    # A reply to the default query: mode 10; position, velocity, torque 0 (f32); 22 V,
    # 25 degC, fault 0 (int8); NOP padding to 24 bytes.
    payload = bytes.fromhex("21000a2f01" + "00" * 12 + "230d2c19005050")
    replies = []
    for axis in table:
        replies.append(FdFrame(id=axis.id << 8, size=len(payload)))
        replies[-1].data[: len(payload)] = list(payload)
    sample, frame = MotorSample(), FdFrame()
    torque, ff, reference = d6(), d4(), c.c_double()
    qref, qdref = d4(*q), d4()
    times = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        m11.bipe_lsm6dso_body(raw, gyro, accel)
        for axis, reply in zip(table, replies, strict=True):
            if not m11.bipe_m11_sample(c.byref(axis), c.byref(reply), 1, 1, c.byref(sample)):
                raise RuntimeError("bench reply rejected")
        core.bipe_estimator_step(
            c.byref(state), c.byref(config), still, gravity, encoders, c.byref(estimate)
        )
        core.bipe_control_step(
            c.byref(balance),
            c.byref(estimate),
            0.0,
            0.0,
            qref,
            qdref,
            config.dt,
            torque,
            ff,
            c.byref(reference),
        )
        for axis in table[:4]:
            m11.bipe_m11_joint_command(c.byref(axis), axis.centre_rad, 0.0, 0.0, c.byref(frame))
        for axis in table[4:]:
            m11.bipe_m11_wheel_command(c.byref(axis), 0.0, c.byref(frame))
        times.append(time.perf_counter_ns() - start)
    times.sort()
    return {
        "median_s": statistics.median(times) * 1e-9,
        "p99_s": times[int(0.99 * len(times))] * 1e-9,
        "repeats": repeats,
        "note": "x86 host via ctypes, includes Python call overhead; the Teensy image "
        "measures the same work with the DWT cycle counter",
    }


def budget(m11, measure=True):
    robot = json.loads((ROOT / "config/robot.json").read_text())
    sensors = json.loads((ROOT / "config/sensors-m10.json").read_text())
    p = load_parameters(profile="m10")
    e = robot["m10"]["electronics"]
    fw, can, imu = e["firmware"], e["drivers"]["can"], e["firmware"]["imu"]
    t = fw["timing"]
    nominal, data = can["arbitration_bit_s"], can["data_bit_s"]
    sizes = frame_sizes(m11)
    frame = {k: frame_time_s(v, nominal, data) for k, v in sizes.items()}
    imu_age = 1 / imu["odr_hz"]
    spi = 13 * 8 / imu["spi_rate_hz"] + t["spi_overhead_s_assumption"]
    # Replies win arbitration over queries (lower identifiers), so the bus is idle at
    # most one driver turnaround while the six exchanges complete.
    feedback = 6 * frame["query"] + 6 * frame["reply"] + t["driver_turnaround_s_assumption"]
    commands = 4 * frame["joint_command"] + 2 * frame["wheel_command"]
    # The driver applies a command within its turnaround (it runs its PD at the PWM rate).
    phases = {
        "imu_sample_age_s": imu_age,
        "imu_spi_read_s": spi,
        "driver_feedback_s": feedback,
        "compute_s": t["compute_s_assumption"],
        "commands_s": commands,
        "driver_apply_s": t["driver_turnaround_s_assumption"],
    }
    total = sum(phases.values())
    limit = sensors["transport_delay_s"]
    control_dt = p["simulation"]["control_dt"]
    bus = 6 * (frame["query"] + frame["reply"]) + commands
    result = {
        "inputs": {
            "can": {k: can[k] for k in ("arbitration_bit_s", "data_bit_s")},
            "imu_odr_hz": imu["odr_hz"],
            "imu_spi_rate_hz": imu["spi_rate_hz"],
            "timing_assumptions": {k: v for k, v in t.items() if k != "basis"},
            "transport_delay_s": limit,
            "control_dt": control_dt,
            "watchdog_timeout_s": fw["watchdog_timeout_s"],
        },
        "frame_bytes": sizes,
        "frame_time_s": frame,
        "phases": phases,
        "sample_to_actuation_s": total,
        "margin_s": limit - total,
        "bus_utilization": bus / control_dt,
        "watchdog_cycles": fw["watchdog_timeout_s"] / control_dt,
        "status": "estimate from bit counts and stated assumptions; bench DWT and bus "
        "captures replace the assumptions at commissioning",
    }
    if measure:
        host = host_cycle_s(m11)
        result["host_cycle"] = host
        result["compute_bound_over_host_p99"] = t["compute_s_assumption"] / host["p99_s"]
    return result


def main():
    import tempfile

    with tempfile.TemporaryDirectory() as directory:
        result = budget(build(directory))
    OUTPUT.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"M11 TIMING sample-to-actuation {result['sample_to_actuation_s'] * 1e3:.3f} ms "
        f"of {result['inputs']['transport_delay_s'] * 1e3:.1f} ms, bus "
        f"{100 * result['bus_utilization']:.1f}%, compute bound "
        f"{result['compute_bound_over_host_p99']:.1f} x host p99"
    )


if __name__ == "__main__":
    main()
