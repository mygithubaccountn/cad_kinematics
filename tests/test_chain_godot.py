"""Chain sanity + Godot-contract runtime (no new joint heuristics)."""

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


def test_chain_and_godot_runtime_reports(tmp_path):
    out = tmp_path / "out"
    r = subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), "run", str(FIXTURE), "--out", str(out)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / "chain_sanity_report.json").is_file()
    assert (out / "debug_overlay.json").is_file()
    assert (out / "godot_runtime_report.json").is_file()

    chain = json.loads((out / "chain_sanity_report.json").read_text())
    assert chain["summary"]["n_error"] == 0
    assert len(chain["summary"]["bfs_order"]) >= 2

    overlay = json.loads((out / "debug_overlay.json").read_text())
    assert len(overlay["markers"]) >= 1
    assert len(overlay["axes"]) >= 1
    assert overlay["link_colors"]

    godot = json.loads((out / "godot_runtime_report.json").read_text())
    assert godot["ok"] is True
    assert godot["n_movable"] >= 1
