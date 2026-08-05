#!/usr/bin/env python3
"""
CAD → Godot Robot Pipeline CLI (incremental stages + manifest)

Usage:
  ./run_with_freecad.sh run robot.step --out out/run
  ./run_with_freecad.sh run robot.step --out out/run --force
  ./run_with_freecad.sh stage joints --out out/run
  ./run_with_freecad.sh status --out out/run
  ./run.sh run fixtures/serial_3dof.synthetic.json --out out/run

Stages: ingest → features → joints → hierarchy → package → meshes → validate

Godot CAD robot = robot.json (kinematics) + meshes/*.glb (CAD view).

Mesh files are a **cache** keyed by STEP geometry + link↔part topology — not by
joint pivots/axes. Joint/pivot iteration updates robot.json only; existing GLBs
are reused (``skip meshes``). Remesh when STEP/topology changes, or with
``--remesh`` / ``--final-meshes``.

  default run          kinematics + ensure mesh cache (skip if fresh)
  --remesh             force tessellate again
  --final-meshes       fine quality export (cache key includes quality)
  --no-meshes          skip mesh stage entirely (debug only)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _warn_numpy_env() -> None:
    import os

    py = sys.version_info
    pp = os.environ.get("PYTHONPATH", "")
    if "FreeCAD.app" in pp and (py.major, py.minor) != (3, 11):
        print(
            "ERROR: PYTHONPATH points at FreeCAD but this is Python "
            f"{py.major}.{py.minor} ({sys.executable}).\n"
            "  Fix: unset PYTHONPATH  or use ./run_with_freecad.sh for STEP\n",
            file=sys.stderr,
        )
        raise SystemExit(2)


_warn_numpy_env()

from common.io_util import ensure_dir, read_json, write_json  # noqa: E402
from common.manifest import (  # noqa: E402
    STAGE_ORDER,
    Manifest,
    inputs_hash_features,
    inputs_hash_hierarchy,
    inputs_hash_ingest,
    inputs_hash_joints,
    inputs_hash_meshes,
    inputs_hash_package,
    inputs_hash_validate,
)
from common.models import AssemblyIR, KinematicTree, RobotDesc  # noqa: E402
from common.timing import PerfTimer, mark_skip, set_timer, timed  # noqa: E402
from common.tolerances import Tolerances  # noqa: E402
from phase import CURRENT_PHASE, PHASE_NAMES, require_phase  # noqa: E402


# Stage → human-readable timing labels (for skip + summary order)
_STAGE_TIMING = {
    "ingest": ("STEP import",),
    "features": ("OCC analysis", "Feature extraction"),
    "joints": ("Joint inference",),
    "hierarchy": ("Hierarchy",),
    "package": ("Index/package",),
    "meshes": ("Mesh generation", "GLB export"),
    "validate": ("Validation",),
}


def _skip(stage: str, reason: str = "") -> None:
    extra = f" ({reason})" if reason else ""
    print(f"skip {stage}{extra}")
    for label in _STAGE_TIMING.get(stage, ()):
        mark_skip(label, note=reason or "skipped")


def _run_label(stage: str) -> None:
    print(f"run  {stage}")


def _close_freecad() -> None:
    try:
        from importer.freecad_session import close_step_document

        close_step_document()
    except Exception:
        pass


def _recover_source_from_out(out: Path) -> Path | None:
    """Prefer manifest / IR / robot.json recorded source paths."""
    candidates: list[Path] = []
    manifest = Manifest.load(out)
    if manifest.data.get("source"):
        candidates.append(Path(manifest.data["source"]))
    for rel, key in (
        ("assembly_ir.json", "source_path"),
        ("robot.json", "source"),
    ):
        p = out / rel
        if not p.is_file():
            continue
        try:
            raw = read_json(p)
        except Exception:
            continue
        val = raw.get(key)
        if val:
            candidates.append(Path(val))
    for c in candidates:
        if c.is_file():
            return c
    return None


def _resolve_source(source: Path, out: Path) -> Path:
    """Resolve STEP/synthetic path; recover from prior out/ if placeholder missing."""
    sp = Path(source).expanduser()
    if not sp.is_absolute():
        sp = (Path.cwd() / sp).resolve()
    else:
        sp = sp.resolve()
    if sp.is_file():
        return sp
    recovered = _recover_source_from_out(out)
    if recovered is not None:
        print(f"note: {source} not found; using recorded source:\n  {recovered}")
        return recovered
    hint = ""
    ir = out / "assembly_ir.json"
    if ir.is_file():
        try:
            recorded = read_json(ir).get("source_path")
            if recorded:
                hint = f"\n  Previously used: {recorded}"
        except Exception:
            pass
    raise SystemExit(
        f"ERROR: source file not found: {source}\n"
        f"  Resolved path: {sp}\n"
        f"  Use your real STEP path, e.g.\n"
        f"    ./run_with_freecad.sh run \"$HOME/Desktop/robot_assembly.stp\" --out out/step"
        f"{hint}"
    )


def _upstream_of(stage: str, from_stage: str | None) -> bool:
    if not from_stage or from_stage not in STAGE_ORDER or stage not in STAGE_ORDER:
        return False
    return STAGE_ORDER.index(stage) < STAGE_ORDER.index(from_stage)


def _maybe_skip(
    stage: str,
    *,
    force: bool,
    manifest: Manifest,
    inputs_hash: str,
    from_stage: str | None = None,
) -> bool:
    """Return True if stage should be skipped (and already logged)."""
    if force:
        return False
    if manifest.is_fresh(stage, inputs_hash):
        _skip(stage, "cache hit")
        manifest.mark_skipped(stage, inputs_hash)
        return True
    # --from-stage joints: keep existing upstream artifacts without re-import
    if _upstream_of(stage, from_stage) and manifest.outputs_exist(stage):
        _skip(stage, f"upstream of --from-stage {from_stage}")
        if not manifest.stage_record(stage).get("status"):
            manifest.mark_done(stage, inputs_hash, extra={"assumed": "from_stage_upstream"})
        else:
            manifest.mark_skipped(stage, inputs_hash)
        return True
    return False


# --- Stage implementations -------------------------------------------------


def stage_ingest(
    source: Path,
    out: Path,
    tol: Tolerances,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    ih = inputs_hash_ingest(source, tol)
    if _maybe_skip(
        "ingest", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        if not manifest.data.get("source"):
            manifest.set_source(source)
            manifest.save()
        return 0
    if not source.is_file():
        raise SystemExit(f"ERROR: cannot ingest; file not found: {source}")
    _run_label("ingest")
    from importer import run_import

    with timed("STEP import"):
        ir = run_import(source, out, tol)
    manifest.set_source(source)
    manifest.mark_done("ingest", ih, extra={"n_parts": len(ir.parts)})
    print(f"  parts={len(ir.parts)} → {out / 'assembly_ir.json'}")
    return 0


def stage_features(
    out: Path,
    tol: Tolerances,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    require_phase(1, "features")
    ih = inputs_hash_features(out, tol)
    if _maybe_skip(
        "features", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("features")
    ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))
    from geometry import run_geometry

    fg = run_geometry(ir, out, tol)
    write_json(out / "geometry.json", fg.to_dict())
    manifest.mark_done(
        "features",
        ih,
        extra={"n_cylinders": len(fg.cylinders), "n_clusters": len(fg.clusters)},
    )
    print(f"  cylinders={len(fg.cylinders)} clusters={len(fg.clusters)}")
    return 0


def stage_joints(
    out: Path,
    tol: Tolerances,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    require_phase(2, "joints")
    include_prismatic = CURRENT_PHASE >= 5
    ih = inputs_hash_joints(out, tol, include_prismatic)
    if _maybe_skip(
        "joints", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("joints")
    ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))
    from common.models import FeatureGraph
    from joints import run_joint_detection
    from joints.axis_detection import run_axis_detection

    with timed("Joint inference"):
        fg = FeatureGraph.from_dict(read_json(out / "features.json"))
        selected = run_joint_detection(ir, fg, out, tol, include_prismatic=include_prismatic)
        resolved = run_axis_detection(selected, fg, out, tol)
        write_json(
            out / "decision_trace.json",
            {
                "phase": CURRENT_PHASE,
                "algorithm": "axis_consensus-1",
                "traces": [j.trace.to_dict() for j in resolved if j.trace],
            },
        )
    manifest.mark_done("joints", ih, extra={"n_selected": len(selected)})
    print(f"  selected={len(selected)}")
    return 0


def stage_hierarchy(
    out: Path,
    tol: Tolerances,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    require_phase(3, "hierarchy")
    ih = inputs_hash_hierarchy(out, tol)
    if _maybe_skip(
        "hierarchy", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("hierarchy")
    ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))
    from common.models import FeatureGraph, ResolvedJoint
    from hierarchy import run_hierarchy

    with timed("Hierarchy"):
        fg = FeatureGraph.from_dict(read_json(out / "features.json"))
        resolved_path = out / "resolved_axes.json"
        if resolved_path.is_file():
            raw = read_json(resolved_path)
            joints_raw = raw.get("joints", raw if isinstance(raw, list) else [])
            resolved = [ResolvedJoint.from_dict(j) for j in joints_raw]
        else:
            from joints import run_joint_detection
            from joints.axis_detection import run_axis_detection

            selected = run_joint_detection(
                ir, fg, out, tol, include_prismatic=(CURRENT_PHASE >= 5)
            )
            resolved = run_axis_detection(selected, fg, out, tol)

        tree = run_hierarchy(ir, fg, resolved, out, tol)
    manifest.mark_done(
        "hierarchy",
        ih,
        extra={"base": tree.base_link, "n_joints": len(tree.joints)},
    )
    print(f"  base={tree.base_link} joints={len(tree.joints)}")
    return 0


def stage_package(
    out: Path,
    tol: Tolerances,
    name: str,
    *,
    force: bool,
    remesh: bool = False,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    """Kinematics package: robot.json (+ mesh path declarations). No FreeCAD tessellate."""
    _ = remesh  # legacy flag; meshes are a separate stage
    ih = inputs_hash_package(out, tol, name)
    if _maybe_skip(
        "package", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("package")
    ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))

    if CURRENT_PHASE >= 3 and (out / "kinematic_tree.json").is_file():
        from exporter.scene import run_scene_generation

        tree = KinematicTree.from_dict(read_json(out / "kinematic_tree.json"))
        desc = run_scene_generation(
            ir, tree, out, tol, name=name, export_meshes=False
        )
        print(f"  joints={len(desc.joints)} → {out / 'robot.json'} (meshes deferred)")
    else:
        from exporter.phase0_robot import export_phase0

        desc = export_phase0(ir, out, tol, name=name, export_meshes=False)
        if (out / "features.json").is_file():
            write_json(out / "geometry.json", read_json(out / "features.json"))
        print(f"  links={len(desc.links)} (phase0) → {out / 'robot.json'} (meshes deferred)")

    manifest.mark_done("package", ih, extra={"name": name, "meshes": False})
    return 0


def stage_meshes(
    out: Path,
    tol: Tolerances,
    name: str,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
    quality: str = "preview",
) -> int:
    """CAD mesh export: FreeCAD tessellate + GLB. Default quality=preview (fast)."""
    if quality not in ("preview", "final"):
        quality = "preview"
    if not (out / "robot.json").is_file():
        stage_package(out, tol, name, force=force, manifest=manifest, from_stage=from_stage)
    ih = inputs_hash_meshes(out, tol, quality=quality)
    if _maybe_skip(
        "meshes", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("meshes")
    print(f"  quality={quality}")
    ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))

    if CURRENT_PHASE >= 3 and (out / "kinematic_tree.json").is_file():
        from exporter.scene import run_scene_generation

        tree = KinematicTree.from_dict(read_json(out / "kinematic_tree.json"))
        desc = run_scene_generation(
            ir, tree, out, tol, name=name, export_meshes=True, mesh_quality=quality
        )
        print(f"  meshes={len(desc.links)} quality={quality} → {out / 'meshes'}")
    else:
        from exporter.phase0_robot import export_phase0

        desc = export_phase0(
            ir, out, tol, name=name, export_meshes=True, mesh_quality=quality
        )
        print(f"  meshes={len(desc.links)} (phase0) quality={quality} → {out / 'meshes'}")

    manifest.mark_done(
        "meshes", ih, extra={"n_links": len(desc.links), "quality": quality}
    )
    return 0


def stage_validate(
    out: Path,
    tol: Tolerances,
    *,
    force: bool,
    manifest: Manifest,
    from_stage: str | None = None,
) -> int:
    ih = inputs_hash_validate(out, tol)
    if _maybe_skip(
        "validate", force=force, manifest=manifest, inputs_hash=ih, from_stage=from_stage
    ):
        return 0
    _run_label("validate")
    with timed("Validation"):
        issues = []
        for required in ("assembly_ir.json", "robot.json"):
            if not (out / required).is_file():
                issues.append(f"missing {required}")
        if CURRENT_PHASE >= 1 and not (out / "geometry.json").is_file():
            issues.append("missing geometry.json")

        # Missing meshes are soft when meshes stage not run (kinematics-first workflow).
        robot_path = out / "robot.json"
        mesh_warnings: list[str] = []
        if robot_path.is_file():
            robot = read_json(robot_path)
            for link in robot.get("links", []):
                mesh = out / link.get("mesh", "")
                if link.get("mesh") and not mesh.is_file():
                    mesh_warnings.append(f"missing mesh {link.get('mesh')}")

        report = {
            "phase": CURRENT_PHASE,
            "ok": len(issues) == 0,
            "overall_confidence": 0.0 if issues else 1.0,
            "unresolved_parts": [],
            "suspicious_joints": [],
            "warnings": list(mesh_warnings),
            "issues": [{"severity": "error", "code": "missing", "message": m} for m in issues]
            + [
                {"severity": "warning", "code": "missing_mesh", "message": m}
                for m in mesh_warnings
            ],
            "metrics": {"missing_meshes": len(mesh_warnings)},
        }

        if CURRENT_PHASE >= 4 and (out / "kinematic_tree.json").is_file() and robot_path.is_file():
            from validation import run_validation

            ir = AssemblyIR.from_dict(read_json(out / "assembly_ir.json"))
            tree = KinematicTree.from_dict(read_json(out / "kinematic_tree.json"))
            desc = RobotDesc.from_dict(read_json(robot_path))
            full = run_validation(ir, tree, desc, out, tol)
            report = full.to_dict()
            report["phase"] = CURRENT_PHASE

        write_json(out / "validation_report.json", report)
        manifest.mark_done(
            "validate",
            ih,
            extra={
                "ok": report.get("ok"),
                "overall_confidence": report.get("overall_confidence"),
                "n_warnings": len(report.get("warnings") or []),
            },
        )
    conf = report.get("overall_confidence")
    n_warn = len(report.get("warnings") or [])
    conf_s = f" conf={conf:.2f}" if isinstance(conf, (int, float)) else ""
    print(f"  ok={report['ok']}{conf_s} warnings={n_warn} → {out / 'validation_report.json'}")
    fk = report.get("fk_motion") or {}
    summ = fk.get("summary") or {}
    if summ:
        g = summ.get("godot") or {}
        print(
            f"  fk_motion godot: ok={g.get('ok', 0)} warn={g.get('warning', 0)} "
            f"err={g.get('error', 0)} → {out / 'fk_motion_report.json'}"
        )
    ch_err = (report.get("metrics") or {}).get("chain_sanity_errors")
    ch_warn = (report.get("metrics") or {}).get("chain_sanity_warnings")
    if ch_err is not None:
        print(
            f"  chain_sanity errors={ch_err} warnings={ch_warn} "
            f"→ {out / 'chain_sanity_report.json'}"
        )
    if (report.get("metrics") or {}).get("godot_runtime_ok") is not None:
        print(
            f"  godot_runtime ok={report['metrics']['godot_runtime_ok']} "
            f"→ {out / 'godot_runtime_report.json'}"
        )
    n_m = (report.get("metrics") or {}).get("n_debug_markers")
    if n_m is not None:
        print(f"  debug_overlay markers={n_m} → {out / 'debug_overlay.json'}")
    return 0 if report.get("ok") else 2


def _emit_perf_summary(out: Path, timer: PerfTimer) -> None:
    write_json(out / "performance_summary.json", timer.to_dict())
    timer.print_summary()
    print(f"  → {out / 'performance_summary.json'}")


def cmd_run(
    source: Path,
    out: Path,
    name: str,
    tol: Tolerances,
    *,
    force: bool = False,
    remesh: bool = False,
    no_meshes: bool = False,
    final_meshes: bool = False,
    from_stage: str | None = None,
) -> int:
    ensure_dir(out)
    print(f"=== phase {CURRENT_PHASE}: {PHASE_NAMES[CURRENT_PHASE]} ===")
    # CAD meshes are part of Godot output, but cached: joint/pivot edits reuse GLBs.
    want_meshes = not no_meshes
    quality = "final" if final_meshes else "preview"
    force_meshes = bool(force or remesh)
    print("note: Godot CAD = robot.json + meshes/ (GLB cache)")
    if want_meshes:
        print(
            f"note: mesh quality={quality} — "
            f"{'force remesh' if force_meshes else 'reuse cache if topology unchanged'}"
        )
    else:
        print("note: --no-meshes (CAD view skipped)")
    source = _resolve_source(source, out)
    manifest = Manifest.load(out)
    if from_stage:
        print(f"invalidate from stage={from_stage}")
        manifest.invalidate_from(from_stage)

    timer = PerfTimer()
    set_timer(timer)
    code = 0
    try:
        code = (
            stage_ingest(
                source, out, tol, force=force, manifest=manifest, from_stage=from_stage
            )
            or code
        )
        if CURRENT_PHASE >= 1:
            code = (
                stage_features(
                    out, tol, force=force, manifest=manifest, from_stage=from_stage
                )
                or code
            )
        if CURRENT_PHASE >= 2:
            code = (
                stage_joints(
                    out, tol, force=force, manifest=manifest, from_stage=from_stage
                )
                or code
            )
        if CURRENT_PHASE >= 3:
            code = (
                stage_hierarchy(
                    out, tol, force=force, manifest=manifest, from_stage=from_stage
                )
                or code
            )
        code = (
            stage_package(
                out,
                tol,
                name,
                force=force,
                manifest=manifest,
                from_stage=from_stage,
            )
            or code
        )
        if want_meshes:
            code = (
                stage_meshes(
                    out,
                    tol,
                    name,
                    force=force_meshes,
                    manifest=manifest,
                    from_stage=from_stage,
                    quality=quality,
                )
                or code
            )
        else:
            for label in _STAGE_TIMING["meshes"]:
                mark_skip(label, note="--no-meshes")
        if CURRENT_PHASE >= 4:
            v = stage_validate(
                out, tol, force=force, manifest=manifest, from_stage=from_stage
            )
            if v:
                code = v
        print(f"manifest → {out / 'manifest.json'}")
        return code
    finally:
        with timed("FreeCAD cleanup"):
            _close_freecad()
        try:
            _emit_perf_summary(out, timer)
        finally:
            set_timer(None)


def cmd_stage(
    name: str,
    out: Path,
    tol: Tolerances,
    *,
    source: Path | None,
    robot_name: str,
    force: bool,
    remesh: bool,
    mesh_quality: str = "preview",
) -> int:
    ensure_dir(out)
    manifest = Manifest.load(out)
    try:
        if name == "ingest":
            if source is None:
                src = manifest.data.get("source")
                if not src:
                    print("ingest requires --source", file=sys.stderr)
                    return 2
                source = Path(src)
            return stage_ingest(source, out, tol, force=force, manifest=manifest)
        if name == "features":
            return stage_features(out, tol, force=force, manifest=manifest)
        if name == "joints":
            return stage_joints(out, tol, force=force, manifest=manifest)
        if name == "hierarchy":
            return stage_hierarchy(out, tol, force=force, manifest=manifest)
        if name == "package":
            return stage_package(
                out, tol, robot_name, force=force, manifest=manifest
            )
        if name == "meshes":
            q = mesh_quality if mesh_quality in ("preview", "final") else "preview"
            return stage_meshes(
                out,
                tol,
                robot_name,
                force=force or remesh,
                manifest=manifest,
                quality=q,
            )
        if name == "validate":
            return stage_validate(out, tol, force=force, manifest=manifest)
        return 2
    finally:
        _close_freecad()


def cmd_status(out: Path) -> int:
    manifest = Manifest.load(out)
    print(f"out: {out}")
    print(f"source: {manifest.data.get('source')}")
    sh = manifest.data.get("source_hash") or ""
    print(f"source_hash: {sh[:16]}{'…' if sh else ''}")
    print(f"{'stage':12} {'status':8} {'outputs':8} {'algo':16} {'hash':12} updated")
    for row in manifest.status_table():
        print(
            f"{row['stage']:12} {row['status'] or '-':8} "
            f"{'yes' if row['outputs_ok'] else 'no':8} "
            f"{(row['algorithm_version'] or '-'):16} "
            f"{(row['inputs_hash'] or '-'):12} "
            f"{row['updated_at'] or '-'}"
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    known = {
        "import",
        "export",
        "analyze",
        "validate",
        "run",
        "stage",
        "status",
        "-h",
        "--help",
        "--phase-info",
    }
    if argv and argv[0] not in known and not argv[0].startswith("-"):
        argv = ["run", *argv]

    p = argparse.ArgumentParser(
        prog="pipeline.py",
        description="CAD → Godot Robot Pipeline (manifest / stage skip)",
    )
    p.add_argument("--phase-info", action="store_true")
    sub = p.add_subparsers(dest="cmd")

    imp = sub.add_parser("import", help="S0 ingest")
    imp.add_argument("source", type=Path)
    imp.add_argument("--out", type=Path, required=True)
    imp.add_argument("--force", action="store_true")

    ana = sub.add_parser("analyze", help="features + joints + hierarchy")
    ana.add_argument("--from-dir", type=Path, required=True)
    ana.add_argument("--out", type=Path, required=True)
    ana.add_argument("--force", action="store_true")

    exp = sub.add_parser("export", help="Mesh export (default: CAD preview; --final-meshes for fine)")
    exp.add_argument("--from-dir", type=Path, required=True)
    exp.add_argument("--out", type=Path, required=True)
    exp.add_argument("--robot-name", default="robot")
    exp.add_argument("--force", action="store_true")
    exp.add_argument("--remesh", action="store_true", help="Force re-tessellate")
    exp.add_argument(
        "--final-meshes",
        action="store_true",
        help="Fine tessellation for shipping (slow)",
    )

    val = sub.add_parser("validate", help="S6 validate")
    val.add_argument("--from-dir", type=Path, required=True)
    val.add_argument("--out", type=Path, default=None)
    val.add_argument("--force", action="store_true")

    run_p = sub.add_parser(
        "run",
        help="CAD→Godot: kinematics + cached meshes (skip remesh if topology unchanged)",
    )
    run_p.add_argument("source", type=Path)
    run_p.add_argument("--out", type=Path, required=True)
    run_p.add_argument("--name", "--robot-name", dest="robot_name", default="robot")
    run_p.add_argument("--force", action="store_true", help="Ignore manifest; recompute all")
    run_p.add_argument(
        "--final-meshes",
        action="store_true",
        help="Fine tessellation quality (slow); still cached by topology+quality",
    )
    run_p.add_argument(
        "--remesh",
        action="store_true",
        help="Force FreeCAD tessellate even if GLB cache is fresh",
    )
    run_p.add_argument(
        "--no-meshes",
        action="store_true",
        help="Skip mesh stage (debug only; Godot CAD view may be empty)",
    )
    run_p.add_argument(
        "--from-stage",
        choices=list(STAGE_ORDER),
        default=None,
        help="Invalidate this stage and kinematic downstream (meshes preserved unless ingest/features)",
    )

    st = sub.add_parser("stage", help="Run a single stage")
    st.add_argument("stage_name", choices=list(STAGE_ORDER))
    st.add_argument("--out", type=Path, required=True)
    st.add_argument("--source", type=Path, default=None)
    st.add_argument("--robot-name", default="robot")
    st.add_argument("--force", action="store_true")
    st.add_argument("--remesh", action="store_true")
    st.add_argument(
        "--mesh-quality",
        choices=("preview", "final"),
        default="preview",
        help="For stage meshes: preview (default) or final",
    )

    status = sub.add_parser("status", help="Show manifest / cache status")
    status.add_argument("--out", type=Path, required=True)

    args = p.parse_args(argv)
    if args.phase_info:
        print(f"CURRENT_PHASE={CURRENT_PHASE} ({PHASE_NAMES[CURRENT_PHASE]})")
        return 0

    tol = Tolerances()
    if args.cmd is None:
        p.print_help()
        return 1
    if args.cmd == "status":
        return cmd_status(args.out)
    if args.cmd == "run":
        return cmd_run(
            args.source,
            args.out,
            args.robot_name,
            tol,
            force=args.force,
            remesh=args.remesh,
            no_meshes=getattr(args, "no_meshes", False),
            final_meshes=getattr(args, "final_meshes", False),
            from_stage=args.from_stage,
        )
    if args.cmd == "stage":
        return cmd_stage(
            args.stage_name,
            args.out,
            tol,
            source=args.source,
            robot_name=args.robot_name,
            force=args.force,
            remesh=args.remesh,
            mesh_quality=getattr(args, "mesh_quality", "preview"),
        )
    if args.cmd == "import":
        ensure_dir(args.out)
        m = Manifest.load(args.out)
        try:
            return stage_ingest(args.source, args.out, tol, force=args.force, manifest=m)
        finally:
            _close_freecad()
    if args.cmd == "analyze":
        ensure_dir(args.out)
        if args.from_dir.resolve() != args.out.resolve():
            src = args.from_dir / "assembly_ir.json"
            if src.is_file():
                write_json(args.out / "assembly_ir.json", read_json(src))
        m = Manifest.load(args.out)
        try:
            stage_features(args.out, tol, force=args.force, manifest=m)
            if CURRENT_PHASE >= 2:
                stage_joints(args.out, tol, force=args.force, manifest=m)
            if CURRENT_PHASE >= 3:
                stage_hierarchy(args.out, tol, force=args.force, manifest=m)
            return 0
        finally:
            _close_freecad()
    if args.cmd == "export":
        ensure_dir(args.out)
        if args.from_dir.resolve() != args.out.resolve():
            for name in (
                "assembly_ir.json",
                "kinematic_tree.json",
                "features.json",
                "geometry.json",
                "robot.json",
                "upright.json",
            ):
                src = args.from_dir / name
                if src.is_file():
                    write_json(args.out / name, read_json(src))
        m = Manifest.load(args.out)
        try:
            stage_package(
                args.out,
                tol,
                args.robot_name,
                force=args.force,
                manifest=m,
            )
            q = "final" if getattr(args, "final_meshes", False) else "preview"
            return stage_meshes(
                args.out,
                tol,
                args.robot_name,
                force=args.force or args.remesh,
                manifest=m,
                quality=q,
            )
        finally:
            _close_freecad()
    if args.cmd == "validate":
        out = args.out or args.from_dir
        ensure_dir(out)
        if Path(args.from_dir).resolve() != Path(out).resolve():
            for name in (
                "assembly_ir.json",
                "kinematic_tree.json",
                "robot.json",
                "geometry.json",
                "features.json",
            ):
                src = Path(args.from_dir) / name
                if src.is_file():
                    write_json(Path(out) / name, read_json(src))
        m = Manifest.load(out)
        return stage_validate(Path(out), tol, force=args.force, manifest=m)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
