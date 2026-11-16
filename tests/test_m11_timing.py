"""M11 bus schedule and latency budget (bipe/m11/timing.py)."""

import importlib.util
import json
from pathlib import Path

import pytest

from bipe.m11.firmware import build
from bipe.m11.timing import OUTPUT, budget, fd_padded, frame_time_s
from bipe.m11.timing import reply_bytes as size

pytest.importorskip("moteus")


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build(tmp_path_factory.mktemp("m11"))


def firmware_tests():
    path = Path(__file__).with_name("test_m11_firmware.py")
    spec = importlib.util.spec_from_file_location("m11_firmware_tests", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_reply_size_matches_the_package_reply(lib):

    resolutions = list(lib.bipe_mx_default_query().resolution)
    assert (
        size(resolutions) == fd_padded(len(firmware_tests().reply_bytes(10, 0.1, 0.2, 30, 0))) == 24
    )


def test_frame_times_count_the_fd_bits():

    # 8 bytes: 36 + 8 stuff + 13 tail nominal bits; ESI, DLC, payload (69) + 17 stuff,
    # stuff count 4, CRC 17 and 6 fixed stuff bits in the data phase.
    assert frame_time_s(8, 1e6, 5e6) == pytest.approx((44 + 13) * 1e-6 + 113 * 0.2e-6)
    times = [frame_time_s(n, 1e6, 5e6) for n in (0, 8, 16, 20, 64)]
    assert times == sorted(times)
    with pytest.raises(ValueError):
        frame_time_s(9, 1e6, 5e6)


def test_latency_budget_meets_the_plant_delay(lib):

    fresh = budget(lib, measure=False)
    committed = json.loads(OUTPUT.read_text())
    assert committed["inputs"] == fresh["inputs"]
    for key in ("frame_bytes", "frame_time_s", "phases", "sample_to_actuation_s"):
        assert committed[key] == pytest.approx(fresh[key]), key
    assert fresh["sample_to_actuation_s"] <= fresh["inputs"]["transport_delay_s"] - 1e-3
    assert fresh["bus_utilization"] < 0.5
    assert fresh["watchdog_cycles"] >= 3
    # The compute assumption keeps at least ten times the host-measured cycle.
    assert committed["compute_bound_over_host_p99"] >= 10
