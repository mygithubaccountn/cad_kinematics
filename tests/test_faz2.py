"""Phase 2 joint detection + DecisionTrace."""

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


def test_faz2_decision_traces(tmp_path):
    sys.path.insert(0, str(ROOT / "src"))
    from phase import CURRENT_PHASE

    assert CURRENT_PHASE >= 2
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    traces = json.loads((out / "decision_trace.json").read_text())
    assert len(traces["traces"]) >= 1
    t0 = traces["traces"][0]
    assert "evidence" in t0 and len(t0["evidence"]) >= 1
    # Phase B: resolved axis traces carry chosen pivot/axis
    if t0.get("chosen"):
        assert "origin" in t0["chosen"] and "axis" in t0["chosen"]
    sel = json.loads((out / "joints_selected.json").read_text())
    assert len(sel["joints"]) >= 1
    assert any(j["joint_type"] == "revolute" for j in sel["joints"])
    assert (out / "robot.json").is_file()
    dt = json.loads((out / "decision_traces.json").read_text())
    assert "selected" in dt
    assert isinstance(dt.get("rejected_notable"), list)
