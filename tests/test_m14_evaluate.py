"""The M14 evaluator scores settle's committed records as the M13 and M14 docs report them."""

import copy
import json
import os

os.environ.setdefault("MUJOCO_GL", "disable")  # headless: the speed module imports mujoco

from bipe.m14 import evaluate as ev
from bipe.parameters import ROOT


def test_settle_records_score(tmp_path):
    # settle's committed records (the baseline's sources before its rerun on this code)
    sources = json.loads((ev.BASELINE / "eval.json").read_text())["sources"]
    sources["acceptance"] = "artifacts/m13/phase_c/settle_acceptance"
    (tmp_path / "eval.json").write_text(json.dumps({"sources": sources}))
    s = ev.score(tmp_path)
    a = s["acceptance"]
    assert (a["cases"], a["matrix_cases"]) == (98, 79)
    assert a["recovered_79"] == 62  # M13 phase C headline
    assert (a["belt_recovered"], a["pushes_survived"], a["pushes_recovered"]) == (4, 30, 30)
    assert a["physical_failures"] == []
    hw = s["hardware"]  # PR #5 and belt follow-up hardware timing records
    assert (hw["belt_recovered"], hw["belt_cases"]) == (8, 8)
    assert (hw["handover_recovered"], hw["handover_cases"]) == (8, 10)
    assert s["stress"]["survived"] == 12  # M14 phase B
    assert s["training"]["fall_rate"] > 0.05
    assert not s["gates"]["2_belt"]["passed"] and s["gates"]["2_pushes"]["passed"]


def test_classical_counts_83_of_98():
    summary = ROOT / "artifacts/m13/phase_c/acceptance_classical/summary.json"
    cases = json.loads(summary.read_text())["controllers"]["current"]["cases"]
    a = ev.score_acceptance(cases)
    assert (a["recovered_98"], a["cases"]) == (83, 98)  # the gate 2 pass mark
    assert len(a["physical_failures"]) == 3


def test_speed_policy_rule():
    criteria = json.loads((ROOT / "config/speed-m12.json").read_text())["criteria"]
    rows = json.loads((ROOT / "artifacts/m13/phase_c/settle_speed/cases.json").read_text())
    row = rows["arc_r1"]
    assert ev.speed_rule(row, criteria) is None  # recorded before the policy rule's metrics
    row = copy.deepcopy(row)
    row["general_passed_without_joint_error"] = True
    row["axle_depth_error"] = {"p95_mm": 8.0, "peak_mm": 9.0, "steady_p95_mm": 4.0}
    assert row["assessment"]["passed"] is False  # classical rule: joint error 27 deg
    assert ev.speed_rule(row, criteria, ("steady", 5.0))["passed"] is True
    assert ev.speed_rule(row, criteria, ("case", 10.0))["passed"] is True
    verdict = ev.speed_rule(row, criteria, ("case", 5.0))
    assert verdict["passed"] is False and verdict["checks"]["axle_depth_p95"] is False


def test_report_renders(tmp_path):
    (tmp_path / "eval.json").write_text(
        json.dumps({"checkpoint": "x.pt", "checkpoint_sha256": "0" * 64,
                    "sources": {"acceptance": "artifacts/m13/belt/settle_filter_acceptance"}})
    )  # fmt: skip
    text = ev.score_dir(tmp_path)
    assert "Gate 2: belt range recovered" in text and "(baseline)" in text
    assert json.loads((tmp_path / "scores.json").read_text())["scores"]["acceptance"]


def test_partial_stages_leave_gates_incomplete(tmp_path):
    summary = ROOT / "artifacts/m14/eval/settle/acceptance/summary.json"
    data = json.loads(summary.read_text())
    cases = data["controllers"]["current"]["cases"]
    keep = [n for n in cases if n.startswith(("push_hold", "belt_hold_h0.1125"))]
    data["controllers"]["current"]["cases"] = {n: cases[n] for n in keep}
    (tmp_path / "acceptance").mkdir()
    (tmp_path / "acceptance/summary.json").write_text(json.dumps(data))
    (tmp_path / "eval.json").write_text(json.dumps({"stages": {"acceptance": {"exit": 0}}}))
    g = ev.score(tmp_path)["gates"]
    assert all(x["passed"] is None for x in g.values())  # no speed stage, partial acceptance
    assert g["2_physical_failures"]["value"] == 0 and not g["2_physical_failures"]["complete"]


def test_failed_stage_is_incomplete(tmp_path):
    shutil_copy = ROOT / "artifacts/m14/eval/settle/acceptance/summary.json"
    (tmp_path / "acceptance").mkdir()
    (tmp_path / "acceptance/summary.json").write_text(shutil_copy.read_text())
    (tmp_path / "eval.json").write_text(json.dumps({"stages": {"acceptance": {"exit": 1}}}))
    assert ev.score(tmp_path)["gates"]["2_belt"]["passed"] is None
