"""Phase 3 hierarchy + robot.json joints for Godot."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "serial_3dof.synthetic.json"


def _env():
    e = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    e["PYTHONPATH"] = f"{ROOT / 'src'}:{ROOT}"
    return e


def test_faz3_robot_has_joints(tmp_path):
    sys.path.insert(0, str(ROOT / "src"))
    from phase import CURRENT_PHASE

    assert CURRENT_PHASE >= 3
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / "kinematic_tree.json").is_file()
    robot = json.loads((out / "robot.json").read_text())
    assert len(robot["joints"]) >= 1
    j = robot["joints"][0]
    for key in ("id", "parent", "child", "type", "origin", "axis"):
        assert key in j
    assert len(j["origin"]) == 3 and len(j["axis"]) == 3
    for link in robot["links"]:
        assert (out / link["mesh"]).is_file()
