"""SCARA / prismatic stage tests."""

from __future__ import annotations

from pathlib import Path

from pipeline.cli import main
from pipeline.common.io_util import read_json
from pipeline.common.models import JointType
from pipeline.common.tolerances import Tolerances
from pipeline.s01_import.scara_fixture import build_scara_fixture
from pipeline.s02_geometry import build_adjacency, cluster_concentric, estimate_contacts
from pipeline.s04_joint_detection import run_joint_detection

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "scara.synthetic.json"


def test_scara_prismatic_hypothesis(tmp_path):
    ir, fg = build_scara_fixture()
    tol = Tolerances()
    fg.clusters = cluster_concentric(fg.cylinders, tol)
    fg.contacts = estimate_contacts(ir, tol)
    fg.adjacency = build_adjacency(fg.contacts, fg.clusters, fg.cylinders)
    selected = run_joint_detection(ir, fg, tmp_path, include_prismatic=True)
    types = {j.joint_type for j in selected}
    assert JointType.REVOLUTE in types
    # arm2-zslide should prefer prismatic (similar outer guides, no shaft-hole)
    pair = [j for j in selected if set(j.ordered_parts()) == {"arm2", "zslide"}]
    assert pair, "expected joint between arm2 and zslide"
    assert pair[0].joint_type == JointType.PRISMATIC


def test_scara_pipeline(tmp_path):
    out = tmp_path / "scara_out"
    rc = main(["run", str(FIXTURE), "--out", str(out), "--name", "scara"])
    assert rc == 0
    robot = read_json(out / "robot.json")
    jtypes = [j["type"] for j in robot["joints"]]
    assert "revolute" in jtypes
    assert "prismatic" in jtypes
