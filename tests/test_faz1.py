"""Phase 1 geometry analysis tests."""

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


def test_phase_at_least_1():
    sys.path.insert(0, str(ROOT / "src"))
    from phase import CURRENT_PHASE

    assert CURRENT_PHASE >= 1


def test_faz1_analyze_produces_cylinders(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    geom = json.loads((out / "geometry.json").read_text())
    assert len(geom["cylinders"]) >= 1
    assert "clusters" in geom
    assert "contacts" in geom
    assert "adjacency" in geom
    cyl = geom["cylinders"][0]
    for key in ("id", "part_id", "axis_point", "axis_dir", "radius", "height"):
        assert key in cyl
    robot = json.loads((out / "robot.json").read_text())
    assert isinstance(robot["joints"], list)
    # Phase ≥3 fills joints; earlier phases left them empty — both OK for geometry check
