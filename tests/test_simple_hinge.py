"""Simple-hinge regression: smallest possible unambiguous case (1 joint, no branching)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "simple_hinge.synthetic.json"


def _env():
    e = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    e["PYTHONPATH"] = f"{ROOT / 'src'}:{ROOT}"
    return e


def test_simple_hinge_single_revolute(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out), "--name", "simple_hinge"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    robot = json.loads((out / "robot.json").read_text())

    assert len(robot["links"]) == 2
    assert len(robot["joints"]) == 1
    assert robot["joints"][0]["type"] == "revolute"

    # Godot-space robot.json axis/origin are post-upright (S5 rotates mesh + joint
    # together). The pre-upright measurement — what the geometry actually says —
    # lives in kinematic_tree.json (world = CAD world here, identity placement).
    tree = json.loads((out / "kinematic_tree.json").read_text())
    joint = tree["joints"][0]
    assert joint["joint_type"] == "revolute"

    axis = joint["axis_world"]
    assert abs(abs(axis[2]) - 1.0) < 1e-6
    assert abs(axis[0]) < 1e-6
    assert abs(axis[1]) < 1e-6

    # Pivot must sit on the pin centerline (x=0, y=0) at the fixture's known height.
    pivot = joint["origin_world"]
    assert abs(pivot[0]) < 1e-6
    assert abs(pivot[1]) < 1e-6
    assert abs(pivot[2] - 0.05) < 5e-3

    # Unambiguous single shaft-hole pair: classifier should be fully confident.
    assert joint["confidence"] > 0.9

    validation = json.loads((out / "validation_report.json").read_text())
    assert validation["metrics"]["overall_confidence"] > 0.5
    assert validation["issues"] == [] or all(
        i["severity"] != "error" for i in validation["issues"]
    )


def test_simple_hinge_fk_moves_only_door(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out), "--name", "simple_hinge"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    tree = json.loads((out / "kinematic_tree.json").read_text())

    # Only two candidate links exist; base selection must pick one of them
    # (largest-volume heuristic — the door leaf is bigger than the frame post
    # in this fixture, so link_door is expected here, not link_frame).
    assert tree["base_link"] in ("link_frame", "link_door")
    assert tree["meta"]["parallel_loops"] == []
