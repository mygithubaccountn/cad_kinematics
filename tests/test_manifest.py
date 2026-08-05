"""Manifest / incremental stage skip — Phase A infrastructure."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "fixtures" / "serial_3dof.synthetic.json"


def _env() -> dict:
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    env["PYTHONPATH"] = str(ROOT / "src")
    return env


def _run(args: list[str], out: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(ROOT / "pipeline.py"), *args],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=_env(),
    )


def test_second_run_skips_fresh_stages(tmp_path):
    out = tmp_path / "out"
    r1 = _run(["run", str(FIXTURE), "--out", str(out), "--name", "serial3"], out)
    assert r1.returncode == 0, r1.stdout + r1.stderr
    assert (out / "manifest.json").is_file()
    assert "run  ingest" in r1.stdout
    assert "run  features" in r1.stdout

    r2 = _run(["run", str(FIXTURE), "--out", str(out), "--name", "serial3"], out)
    assert r2.returncode == 0, r2.stdout + r2.stderr
    assert "skip ingest" in r2.stdout
    assert "skip features" in r2.stdout
    assert "skip joints" in r2.stdout
    assert "skip hierarchy" in r2.stdout
    # package/validate may skip too if hashes match
    assert "skip package" in r2.stdout or "run  package" in r2.stdout


def test_from_stage_invalidates_downstream(tmp_path):
    out = tmp_path / "out"
    r1 = _run(["run", str(FIXTURE), "--out", str(out)], out)
    assert r1.returncode == 0, r1.stdout + r1.stderr

    r2 = _run(
        ["run", str(FIXTURE), "--out", str(out), "--from-stage", "joints"],
        out,
    )
    assert r2.returncode == 0, r2.stdout + r2.stderr
    assert "skip ingest" in r2.stdout
    assert "skip features" in r2.stdout
    assert "run  joints" in r2.stdout
    assert "run  hierarchy" in r2.stdout


def test_force_reruns_all(tmp_path):
    out = tmp_path / "out"
    assert _run(["run", str(FIXTURE), "--out", str(out)], out).returncode == 0
    r = _run(["run", str(FIXTURE), "--out", str(out), "--force"], out)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "run  ingest" in r.stdout
    assert "run  features" in r.stdout
    assert "skip ingest" not in r.stdout


def test_status_lists_stages(tmp_path):
    out = tmp_path / "out"
    assert _run(["run", str(FIXTURE), "--out", str(out)], out).returncode == 0
    r = _run(["status", "--out", str(out)], out)
    assert r.returncode == 0, r.stdout + r.stderr
    for stage in ("ingest", "features", "joints", "hierarchy", "package", "meshes", "validate"):
        assert stage in r.stdout


def test_default_run_ensures_mesh_cache(tmp_path):
    out = tmp_path / "out"
    r = _run(["run", str(FIXTURE), "--out", str(out), "--name", "serial3"], out)
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / "robot.json").is_file()
    assert "run  meshes" in r.stdout or "skip meshes" in r.stdout
    assert (out / "meshes").is_dir()
    assert list((out / "meshes").glob("*.glb"))


def test_joint_iteration_reuses_mesh_cache(tmp_path):
    out = tmp_path / "out"
    r1 = _run(["run", str(FIXTURE), "--out", str(out), "--name", "serial3"], out)
    assert r1.returncode == 0, r1.stdout + r1.stderr
    assert "run  meshes" in r1.stdout
    glbs = sorted(p.name for p in (out / "meshes").glob("*.glb"))
    assert glbs

    r2 = _run(
        ["run", str(FIXTURE), "--out", str(out), "--name", "serial3", "--from-stage", "joints"],
        out,
    )
    assert r2.returncode == 0, r2.stdout + r2.stderr
    assert "run  joints" in r2.stdout
    assert "skip meshes" in r2.stdout
    assert "run  meshes" not in r2.stdout
    assert sorted(p.name for p in (out / "meshes").glob("*.glb")) == glbs


def test_no_meshes_skips_tessellate(tmp_path):
    out = tmp_path / "out"
    r = _run(
        ["run", str(FIXTURE), "--out", str(out), "--name", "serial3", "--no-meshes"],
        out,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert (out / "robot.json").is_file()
    assert "run  meshes" not in r.stdout
    meshes = list((out / "meshes").glob("*.glb")) if (out / "meshes").is_dir() else []
    assert meshes == []


def test_stage_joints_alone(tmp_path):
    out = tmp_path / "out"
    assert _run(["run", str(FIXTURE), "--out", str(out)], out).returncode == 0
    r = _run(["stage", "joints", "--out", str(out), "--force"], out)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "run  joints" in r.stdout
    assert (out / "joints_selected.json").is_file()
    assert (out / "resolved_axes.json").is_file()


def test_missing_source_recovers_from_out(tmp_path):
    out = tmp_path / "out"
    assert _run(["run", str(FIXTURE), "--out", str(out)], out).returncode == 0
    # Wipe manifest source bookkeeping but keep IR; wrong CLI path should recover
    import json

    man = json.loads((out / "manifest.json").read_text())
    man["source"] = str(FIXTURE)
    man["source_hash"] = "x"
    (out / "manifest.json").write_text(json.dumps(man))
    r = _run(
        ["run", "does_not_exist.step", "--out", str(out), "--from-stage", "joints"],
        out,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert "not found; using recorded source" in r.stdout
    assert "skip ingest" in r.stdout
    assert "run  joints" in r.stdout


def test_missing_source_errors_clearly(tmp_path):
    out = tmp_path / "fresh"
    r = _run(["run", "robot.step", "--out", str(out)], out)
    assert r.returncode != 0
    assert "source file not found" in (r.stdout + r.stderr)
