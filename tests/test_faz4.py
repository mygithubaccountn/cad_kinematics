"""Phase C — confidence-based validation gate."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "serial_3dof.synthetic.json"


def _env():
    e = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    e["PYTHONPATH"] = f"{ROOT / 'src'}:{ROOT}"
    return e


def _eye():
    return np.eye(4).tolist()


def _part(pid: str, z: float = 0.0):
    from common.models import BBox, PartInstance, Provenance

    return PartInstance(
        id=pid,
        name=pid,
        placement=_eye(),
        volume=1e-3,
        bbox=BBox(min_xyz=[0.0, 0.0, z], max_xyz=[0.05, 0.05, z + 0.08]),
        provenance=Provenance(source="test"),
    )


def test_faz4_validation_metrics(tmp_path):
    sys.path.insert(0, str(ROOT / "src"))
    from phase import CURRENT_PHASE

    assert CURRENT_PHASE >= 4
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    report = json.loads((out / "validation_report.json").read_text())
    assert report["ok"] is True
    assert "overall_confidence" in report
    assert report["overall_confidence"] > 0.5
    assert "warnings" in report
    assert "suspicious_joints" in report
    assert "unresolved_parts" in report
    assert "n_movable_joints" in report["metrics"]
    assert report["metrics"]["n_movable_joints"] >= 1
    assert "rest_pose_mean_err_m" in report["metrics"]
    assert "fk_pivot_mean_err_m" in report["metrics"]
    assert "smoke_axis_distance_delta_mean" in report["metrics"]


def test_few_joints_is_warning_not_fail():
    """Under-resolved multi-part tree should warn, not hard-fail."""
    sys.path.insert(0, str(ROOT / "src"))
    from common.models import (
        AssemblyIR,
        JointType,
        KinematicJoint,
        KinematicLink,
        KinematicTree,
    )
    from common.tolerances import Tolerances
    from validation import run_validation

    parts = [_part(f"part_{i}", z=0.1 * i) for i in range(5)]
    ir = AssemblyIR(parts=parts, source_path="test")
    links = [
        KinematicLink(id="link_part_0", name="base", part_ids=["part_0"]),
        KinematicLink(id="link_part_1", name="l1", part_ids=["part_1"]),
        KinematicLink(id="link_part_2", name="l2", part_ids=["part_2", "part_3", "part_4"]),
    ]
    joints = [
        KinematicJoint(
            id="j0",
            name="j0",
            parent="link_part_0",
            child="link_part_1",
            joint_type=JointType.REVOLUTE,
            origin_local=[0, 0, 0.1],
            axis_local=[0, 0, 1],
            origin_world=[0, 0, 0.1],
            axis_world=[0, 0, 1],
            confidence=0.9,
        ),
        KinematicJoint(
            id="fixed_weak",
            name="fixed_weak",
            parent="link_part_0",
            child="link_part_2",
            joint_type=JointType.FIXED,
            origin_local=[0, 0, 0.2],
            axis_local=[0, 0, 1],
            origin_world=[0, 0, 0.2],
            axis_world=[0, 0, 1],
            confidence=0.05,
        ),
    ]
    link_world = {
        "link_part_0": _eye(),
        "link_part_1": [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1],
        ],
        "link_part_2": [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.2],
            [0, 0, 0, 1],
        ],
    }
    tree = KinematicTree(
        base_link="link_part_0",
        links=links,
        joints=joints,
        link_world=link_world,
        meta={
            "suspicious_orphans": [
                {"orphan": "link_part_2", "host": "link_part_0", "score": 0.05}
            ]
        },
    )
    with tempfile.TemporaryDirectory() as td:
        report = run_validation(ir, tree, None, td, Tolerances())
    assert report.ok is True
    codes = {i.code for i in report.issues}
    assert "few_movable_joints" in codes or "weak_fixed_joint" in codes or "suspicious_orphan" in codes
    assert len(report.warnings) >= 1
    assert report.overall_confidence < 0.95
    assert len(report.suspicious_joints) >= 1 or len(report.unresolved_parts) >= 1


def test_disconnected_child_is_hard_fail():
    sys.path.insert(0, str(ROOT / "src"))
    from common.models import AssemblyIR, KinematicLink, KinematicTree
    from common.tolerances import Tolerances
    from validation import run_validation

    ir = AssemblyIR(parts=[_part("a"), _part("b", z=2.0)], source_path="t")
    tree = KinematicTree(
        base_link="link_a",
        links=[
            KinematicLink(id="link_a", name="a", part_ids=["a"]),
            KinematicLink(id="link_b", name="b", part_ids=["b"]),
        ],
        joints=[],
        link_world={"link_a": _eye(), "link_b": _eye()},
        meta={},
    )
    with tempfile.TemporaryDirectory() as td:
        report = run_validation(ir, tree, None, td, Tolerances())
    assert report.ok is False
    assert any(i.code == "child_disconnected" for i in report.issues)
