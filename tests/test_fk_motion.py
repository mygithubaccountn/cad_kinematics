"""Per-joint FK motion validation (Godot contract)."""

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


def test_fk_motion_on_serial_fixture(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / "fk_motion_report.json").is_file()
    fk = json.loads((out / "fk_motion_report.json").read_text())
    assert fk["summary"]["all_ok"] is True
    godot = [j for j in fk["godot_joints"] if not j.get("checks", {}).get("skipped")]
    assert len(godot) >= 1
    for j in godot:
        assert j["ok"] is True
        assert j.get("decision_trace") is not None or j["type"] != "revolute"
        if j["type"] == "revolute":
            assert j["checks"]["pivot_drift_m"] < 1e-6
            assert j["checks"]["child_point_err_m"] < 1e-6

    report = json.loads((out / "validation_report.json").read_text())
    assert "fk_motion" in report
    assert report["fk_motion"]["summary"]["all_ok"] is True
