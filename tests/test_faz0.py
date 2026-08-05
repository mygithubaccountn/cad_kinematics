"""Phase 0 infrastructure tests — no kinematics."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "serial_3dof.synthetic.json"


def test_phase_at_least_zero():
    sys.path.insert(0, str(ROOT / "src"))
    from phase import CURRENT_PHASE

    assert CURRENT_PHASE >= 0


def test_faz0_cli_run(tmp_path):
    out = tmp_path / "out"
    rc = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env={**dict(**{k: v for k, v in __import__("os").environ.items() if k != "PYTHONPATH"}), "PYTHONPATH": str(ROOT / "src")},
    )
    assert rc.returncode == 0, rc.stdout + rc.stderr
    assert (out / "assembly_ir.json").is_file()
    assert (out / "robot.json").is_file()
    assert (out / "geometry.json").is_file()
    assert (out / "validation_report.json").is_file()

    robot = json.loads((out / "robot.json").read_text())
    # Until phase 3, joints stay empty in robot.json
    assert isinstance(robot["joints"], list)
    assert len(robot["links"]) >= 1
    for link in robot["links"]:
        assert (out / link["mesh"]).is_file()
