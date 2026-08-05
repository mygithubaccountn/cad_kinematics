"""Phase B — pivot/axis consensus + DecisionTrace completeness."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "serial_3dof.synthetic.json"


def _env():
    e = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    e["PYTHONPATH"] = f"{ROOT / 'src'}:{ROOT}"
    return e


def test_decision_trace_chosen_and_runner_up_fields():
    sys.path.insert(0, str(ROOT / "src"))
    from common.trace import DecisionTrace

    t = DecisionTrace(subject="axis:test")
    t.add("shaft_hole", 0.3, "ok")
    t.set_chosen(
        origin=[0.0, 0.0, 0.1],
        axis=[0.0, 0.0, 1.0],
        method="consensus:a+b",
        confidence=0.8,
    )
    t.set_runner_up(
        name="cluster_median",
        origin=[0.0, 0.0, 0.11],
        axis=[0.0, 0.0, 1.0],
        score=0.7,
    )
    d = t.to_dict()
    assert d["chosen"]["method"] == "consensus:a+b"
    assert d["runner_up"]["name"] == "cluster_median"
    assert "chosen=" in d["summary"]
    roundtrip = DecisionTrace.from_dict(d)
    assert roundtrip.chosen["origin"][2] == 0.1
    assert roundtrip.runner_up["name"] == "cluster_median"


def test_axis_consensus_prefers_agreeing_candidates():
    sys.path.insert(0, str(ROOT / "src"))
    from common.models import (
        ConcentricCluster,
        CylFeature,
        CylKind,
        FeatureGraph,
        JointHypothesis,
        JointType,
    )
    from common.tolerances import Tolerances
    from joints.axis_detection import resolve_joint_axis

    axis = [0.0, 0.0, 1.0]
    # True pivot near z=0.25; one outlier far away
    cluster = ConcentricCluster(
        id="c0",
        axis_point=[0.0, 0.0, 0.25],
        axis_dir=axis,
        cyl_ids=["cyl_a", "cyl_b"],
        part_ids=["pa", "pb"],
    )
    cyl_a = CylFeature(
        id="cyl_a",
        part_id="pa",
        kind=CylKind.OUTER,
        axis_point=[0.0, 0.0, 0.24],
        axis_dir=axis,
        radius=0.01,
        height=0.05,
    )
    cyl_b = CylFeature(
        id="cyl_b",
        part_id="pb",
        kind=CylKind.INNER,
        axis_point=[0.0, 0.0, 0.26],
        axis_dir=axis,
        radius=0.011,
        height=0.05,
    )
    hyp = JointHypothesis(
        id="jhyp_0000",
        part_a="pa",
        part_b="pb",
        joint_type=JointType.REVOLUTE,
        axis_point=[0.0, 0.0, 0.25],
        axis_dir=axis,
        pivot=[0.0, 0.0, 0.25],
        confidence=0.8,
        cluster_id="c0",
    )
    fg = FeatureGraph(cylinders=[cyl_a, cyl_b], clusters=[cluster])
    rj = resolve_joint_axis(hyp, fg, Tolerances())
    assert rj.trace is not None
    assert rj.trace.chosen is not None
    assert "method" in rj.trace.chosen
    # Pivot should stay near z≈0.25, not drift
    assert abs(rj.origin[2] - 0.25) < 0.03
    assert abs(rj.axis[2]) > 0.99
    # With multiple agreeing cands we expect runner_up or consensus method
    assert rj.trace.runner_up is not None or "consensus" in rj.trace.chosen["method"]


def test_pipeline_resolved_traces_have_chosen(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    resolved = json.loads((out / "resolved_axes.json").read_text())
    assert resolved.get("algorithm") == "axis_consensus-1"
    assert len(resolved["joints"]) >= 1
    for j in resolved["joints"]:
        tr = j["trace"]
        assert tr.get("chosen") is not None, tr
        assert len(tr["chosen"]["origin"]) == 3
        assert len(tr["chosen"]["axis"]) == 3
        assert "method" in tr["chosen"]

    traces = json.loads((out / "decision_trace.json").read_text())
    assert traces["traces"][0].get("chosen") is not None

    dt = json.loads((out / "decision_traces.json").read_text())
    assert "selected" in dt
    assert "rejected_notable" in dt
