"""CLI entrypoint: modular stage runners + full pipeline."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from pipeline.common.io_util import ensure_dir, read_json, write_json
from pipeline.common.models import AssemblyIR, FeatureGraph, JointHypothesis, KinematicTree, RobotDesc
from pipeline.common.serialize import resolved_joint_from_dict
from pipeline.common.tolerances import Tolerances
from pipeline.s01_import import run_import
from pipeline.s02_geometry import run_geometry
from pipeline.s03_mesh import run_mesh_export
from pipeline.s04_joint_detection import run_joint_detection
from pipeline.s05_axis_detection import run_axis_detection
from pipeline.s06_hierarchy import run_hierarchy
from pipeline.s07_scene_generation import run_scene_generation
from pipeline.s08_validation import run_validation


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="cad-robot",
        description="CAD → Godot Robot Pipeline (STEP assembly to kinematic Godot scene)",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_out(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--out", type=Path, required=True, help="Output directory")

    run_p = sub.add_parser("run", help="Run full pipeline")
    run_p.add_argument("source", type=Path, help="STEP file or synthetic fixture")
    add_out(run_p)
    run_p.add_argument("--name", default="robot")
    run_p.add_argument("--no-prismatic", action="store_true")
    run_p.add_argument("--cad-z-up", action="store_true", help="Keep CAD Z-up in robot.json")

    for name, help_ in [
        ("import", "01 Import STEP/synthetic → assembly_ir.json"),
        ("geometry", "02 Geometry features"),
        ("mesh", "03 Raw mesh export"),
        ("joints", "04 Joint detection"),
        ("axes", "05 Axis/pivot refinement"),
        ("hierarchy", "06 Kinematic tree"),
        ("scene", "07 Scene / robot.json"),
        ("validate", "08 Validation"),
    ]:
        sp = sub.add_parser(name, help=help_)
        if name == "import":
            sp.add_argument("source", type=Path)
        else:
            sp.add_argument("--from-dir", type=Path, required=True, help="Directory with prior stage outputs")
        add_out(sp)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    tol = Tolerances()

    if args.cmd == "run":
        return cmd_run(args, tol)
    if args.cmd == "import":
        run_import(args.source, args.out, tol)
        return 0
    if args.cmd == "geometry":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        run_geometry(ir, args.out, tol)
        return 0
    if args.cmd == "mesh":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        run_mesh_export(ir, args.out, tol)
        return 0
    if args.cmd == "joints":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        fg = FeatureGraph.from_dict(read_json(args.from_dir / "features.json"))
        run_joint_detection(ir, fg, args.out, tol)
        return 0
    if args.cmd == "axes":
        data = read_json(args.from_dir / "joints_selected.json")
        hyps = [JointHypothesis.from_dict(h) for h in data["joints"]]
        fg = FeatureGraph.from_dict(read_json(args.from_dir / "features.json"))
        run_axis_detection(hyps, fg, args.out, tol)
        return 0
    if args.cmd == "hierarchy":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        fg = FeatureGraph.from_dict(read_json(args.from_dir / "features.json"))
        joints = [
            resolved_joint_from_dict(j)
            for j in read_json(args.from_dir / "resolved_axes.json")["joints"]
        ]
        run_hierarchy(ir, fg, joints, args.out, tol)
        return 0
    if args.cmd == "scene":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        tree = KinematicTree.from_dict(read_json(args.from_dir / "kinematic_tree.json"))
        run_scene_generation(ir, tree, args.out, tol)
        return 0
    if args.cmd == "validate":
        ir = AssemblyIR.from_dict(read_json(args.from_dir / "assembly_ir.json"))
        tree = KinematicTree.from_dict(read_json(args.from_dir / "kinematic_tree.json"))
        desc = None
        rp = args.from_dir / "robot.json"
        if rp.is_file():
            desc = RobotDesc.from_dict(read_json(rp))
        run_validation(ir, tree, desc, args.out, tol)
        return 0
    return 1


def cmd_run(args: argparse.Namespace, tol: Tolerances) -> int:
    out = ensure_dir(args.out)
    print(f"[01] import {args.source}")
    ir = run_import(args.source, out, tol)
    print(f"     parts={len(ir.parts)}")
    if ir.meta.get("warning") == "single_solid_assembly":
        print("     WARNING: single solid — joints unlikely")

    print("[02] geometry")
    fg = run_geometry(ir, out, tol)
    print(f"     cylinders={len(fg.cylinders)} clusters={len(fg.clusters)}")

    print("[03] mesh (raw parts)")
    run_mesh_export(ir, out / "raw_meshes", tol)

    print("[04] joint detection")
    selected = run_joint_detection(
        ir, fg, out, tol, include_prismatic=not args.no_prismatic
    )
    print(f"     selected_joints={len(selected)}")

    print("[05] axis detection")
    resolved = run_axis_detection(selected, fg, out, tol)

    print("[06] hierarchy")
    tree = run_hierarchy(ir, fg, resolved, out, tol)
    print(f"     base={tree.base_link} links={len(tree.links)} joints={len(tree.joints)}")

    print("[07] scene generation")
    desc = run_scene_generation(
        ir,
        tree,
        out,
        tol,
        name=args.name,
        to_gltf_y_up=not args.cad_z_up,
    )

    print("[08] validation")
    report = run_validation(ir, tree, desc, out, tol)
    write_json(out / "pipeline_summary.json", {
        "ok": report.ok,
        "parts": len(ir.parts),
        "joints": len(tree.joints),
        "robot": str(out / "robot.json"),
        "validation": report.to_dict(),
    })
    print(f"DONE ok={report.ok} → {out / 'robot.json'}")
    return 0 if report.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
