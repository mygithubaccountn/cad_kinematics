"""Integration: full pipeline on synthetic serial 3-DOF."""

from __future__ import annotations

from pathlib import Path

from pipeline.cli import main
from pipeline.common.io_util import read_json


FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "serial_3dof.synthetic.json"


def test_full_pipeline_serial_3dof(tmp_path):
    out = tmp_path / "out"
    rc = main(["run", str(FIXTURE), "--out", str(out), "--name", "serial3"])
    assert rc == 0
    assert (out / "assembly_ir.json").is_file()
    assert (out / "features.json").is_file()
    assert (out / "robot.json").is_file()
    assert (out / "validation_report.json").is_file()
    robot = read_json(out / "robot.json")
    assert robot["name"] == "serial3"
    assert robot["frame"] == "gltf_y_up"
    assert len(robot["joints"]) >= 3
    # Meshes exist
    for link in robot["links"]:
        mesh = out / link["mesh"]
        assert mesh.is_file(), mesh
    report = read_json(out / "validation_report.json")
    assert report["ok"] is True
    traces = read_json(out / "decision_traces.json")
    assert len(traces["traces"]) >= 1
