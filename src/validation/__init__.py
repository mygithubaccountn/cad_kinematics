"""S6 validation — confidence-based gate (hard-fail only critical geometry)."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.io_util import write_json
from common.math3d import (
    as_vec3,
    bbox_contains,
    mat4_from_list,
    normalize,
    rotation_matrix_axis_angle,
    transform_point,
)
from common.models import (
    AssemblyIR,
    JointType,
    KinematicTree,
    RobotDesc,
    ValidationIssue,
    ValidationReport,
)
from common.tolerances import Tolerances

# Soft thresholds (warnings, not ship-blockers)
_LOW_JOINT_CONF = 0.45
_WEAK_FIXED_CONF = 0.15
# Pivot farther than this multiple of margin → critical fail
_PIVOT_FAIL_MARGIN_MULT = 8.0
# Child COM after spin flying away from axis → critical
_SMOKE_FAIL_M = 0.05
_SMOKE_WARN_M = 0.01


def run_validation(
    ir: AssemblyIR,
    tree: KinematicTree,
    desc: Optional[RobotDesc],
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> ValidationReport:
    """
    Critical errors → ok=False (physically broken kinematics).
    Low-quality STEP / weak evidence → warnings + overall_confidence < 1.
    Pipeline still produces robot.json; callers can ship with caveats.
    """
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    issues: list[ValidationIssue] = []
    metrics: dict[str, Any] = {}
    suspicious_joints: list[dict[str, Any]] = []
    unresolved_parts: list[dict[str, Any]] = []

    _check_structure(tree, issues)
    _check_pivots_axes(ir, tree, tolerances, issues, suspicious_joints)
    _check_fk_and_smoke(tree, tolerances, issues, metrics)
    _check_quality_signals(ir, tree, tolerances, issues, suspicious_joints, unresolved_parts, metrics)

    if desc is not None:
        _check_package(desc, out, issues, metrics)

    from validation.chain_sanity import run_chain_sanity
    from validation.debug_overlay import write_debug_overlay
    from validation.fk_motion import run_fk_motion_validation
    from validation.godot_runtime import run_godot_runtime_test

    chain = run_chain_sanity(ir, tree, desc, out, tolerances, issues)
    metrics["chain_sanity_errors"] = int(chain.get("summary", {}).get("n_error", 0))
    metrics["chain_sanity_warnings"] = int(chain.get("summary", {}).get("n_warning", 0))

    fk_motion = run_fk_motion_validation(tree, desc, out, tolerances, issues)
    metrics["fk_motion_all_ok"] = bool(fk_motion.get("summary", {}).get("all_ok"))
    metrics["fk_motion_godot_errors"] = int(
        fk_motion.get("summary", {}).get("godot", {}).get("error", 0)
    )
    metrics["fk_motion_tree_errors"] = int(
        fk_motion.get("summary", {}).get("tree", {}).get("error", 0)
    )

    overlay = write_debug_overlay(tree, desc, out)
    metrics["n_debug_markers"] = len(overlay.get("markers", []))

    godot_rt: dict = {}
    if desc is not None and (out / "robot.json").is_file():
        godot_rt = run_godot_runtime_test(out / "robot.json", out, tolerances.smoke_angle_rad)
        metrics["godot_runtime_ok"] = bool(godot_rt.get("ok"))
        if not godot_rt.get("ok"):
            issues.append(
                ValidationIssue(
                    "error" if godot_rt.get("orphan_links") else "warning",
                    "godot_runtime",
                    f"Godot-contract runtime issues: "
                    f"joints_fail={sum(1 for j in godot_rt.get('joints', []) if not j.get('ok'))} "
                    f"mesh_missing={len(godot_rt.get('mesh_missing') or [])} "
                    f"orphans={godot_rt.get('orphan_links')}",
                )
            )

    _sync_godot_test_robot(out)
    _sync_viewer_robot(out)

    warnings = [i.message for i in issues if i.severity == "warning"]
    overall = _overall_confidence(tree, issues, suspicious_joints, unresolved_parts, metrics)
    metrics["overall_confidence"] = overall
    metrics["n_errors"] = sum(1 for i in issues if i.severity == "error")
    metrics["n_warnings"] = len(warnings)

    ok = not any(i.severity == "error" for i in issues)
    report = ValidationReport(
        ok=ok,
        issues=issues,
        metrics=metrics,
        overall_confidence=overall,
        unresolved_parts=unresolved_parts,
        suspicious_joints=suspicious_joints,
        warnings=warnings,
        fk_motion={
            "angle_rad": fk_motion.get("angle_rad"),
            "summary": fk_motion.get("summary"),
            "report": "fk_motion_report.json",
            "joints": [
                {
                    "id": r.get("id"),
                    "source": r.get("source"),
                    "severity": r.get("severity"),
                    "ok": r.get("ok"),
                    "checks": r.get("checks"),
                    "decision_trace": r.get("decision_trace"),
                    "fail_details": r.get("fail_details"),
                    "warn_details": r.get("warn_details"),
                }
                for r in list(fk_motion.get("godot_joints") or [])
                + list(fk_motion.get("tree_joints") or [])
            ],
        },
    )
    # Attach extra reports into metrics paths for discoverability
    metrics["chain_sanity_report"] = "chain_sanity_report.json"
    metrics["debug_overlay"] = "debug_overlay.json"
    metrics["godot_runtime_report"] = "godot_runtime_report.json"
    metrics["chain_bfs_order"] = chain.get("summary", {}).get("bfs_order")
    report.metrics = metrics
    write_json(out / "validation_report.json", report.to_dict())
    return report


def _sync_viewer_robot(out: Path) -> None:
    """Keep browser viewer/ in sync with latest out (incl. debug_overlay)."""
    robot = out / "robot.json"
    if not robot.is_file():
        return
    root = Path(__file__).resolve().parents[2]
    dest = root / "viewer"
    if not dest.is_dir():
        return
    try:
        import shutil

        shutil.copy2(robot, dest / "robot.json")
        if (out / "debug_overlay.json").is_file():
            shutil.copy2(out / "debug_overlay.json", dest / "debug_overlay.json")
        src_meshes = out / "meshes"
        if src_meshes.is_dir():
            dmesh = dest / "meshes"
            dmesh.mkdir(parents=True, exist_ok=True)
            for glb in src_meshes.glob("*.glb"):
                shutil.copy2(glb, dmesh / glb.name)
    except Exception:
        pass


def _sync_godot_test_robot(out: Path) -> None:
    """Copy latest robot.json + meshes into godot_test/robot_data when present."""
    robot = out / "robot.json"
    if not robot.is_file():
        return
    # validation/ -> src/ -> project root
    root = Path(__file__).resolve().parents[2]
    dest = root / "godot_test" / "robot_data"
    if not (root / "godot_test").is_dir():
        return
    try:
        import shutil

        dest.mkdir(parents=True, exist_ok=True)
        shutil.copy2(robot, dest / "robot.json")
        if (out / "debug_overlay.json").is_file():
            shutil.copy2(out / "debug_overlay.json", dest / "debug_overlay.json")
        src_meshes = out / "meshes"
        if src_meshes.is_dir():
            dmesh = dest / "meshes"
            dmesh.mkdir(parents=True, exist_ok=True)
            for glb in src_meshes.glob("*.glb"):
                shutil.copy2(glb, dmesh / glb.name)
    except Exception:
        pass


def _check_structure(tree: KinematicTree, issues: list[ValidationIssue]) -> None:
    link_ids = {l.id for l in tree.links}
    if tree.base_link not in link_ids:
        issues.append(ValidationIssue("error", "base_missing", "base_link not in links"))

    children = {j.child for j in tree.joints}
    for lid in link_ids:
        if lid == tree.base_link:
            continue
        if lid not in children:
            # Child completely disconnected from tree — critical
            issues.append(
                ValidationIssue(
                    "error",
                    "child_disconnected",
                    f"{lid} has no parent joint (disconnected from kinematic tree)",
                )
            )

    if _has_cycle(tree):
        if tree.meta.get("parallel_loops"):
            issues.append(
                ValidationIssue(
                    "info",
                    "parallel_cycle",
                    "Joint graph contains cycles (parallel/Delta); closed-chain solver required",
                )
            )
        else:
            issues.append(
                ValidationIssue("error", "cycle", "Kinematic graph has unexpected cycle")
            )


def _check_pivots_axes(
    ir: AssemblyIR,
    tree: KinematicTree,
    tol: Tolerances,
    issues: list[ValidationIssue],
    suspicious_joints: list[dict[str, Any]],
) -> None:
    part_map = ir.part_map()
    for j in tree.joints:
        axis = as_vec3(j.axis_world)
        n = float(np.linalg.norm(axis))
        if j.joint_type != JointType.FIXED and abs(n - 1.0) > 1e-2:
            issues.append(
                ValidationIssue(
                    "error",
                    "axis_degenerate",
                    f"{j.id} axis not unit (norm={n:.4f}) — transform will break",
                )
            )
        elif abs(n - 1.0) > 1e-3:
            issues.append(ValidationIssue("warning", "axis_norm", f"{j.id} axis norm={n}"))

        if j.joint_type == JointType.FIXED:
            continue

        pivot = as_vec3(j.origin_world)
        child_link = next(l for l in tree.links if l.id == j.child)
        parent_link = next(l for l in tree.links if l.id == j.parent)
        pids = child_link.part_ids + parent_link.part_ids

        near = any(
            bbox_contains(part_map[pid].bbox.as_array(), pivot, tol.pivot_bbox_margin_m)
            for pid in pids
            if pid in part_map
        )
        far = not any(
            bbox_contains(
                part_map[pid].bbox.as_array(),
                pivot,
                tol.pivot_bbox_margin_m * _PIVOT_FAIL_MARGIN_MULT,
            )
            for pid in pids
            if pid in part_map
        )

        if far:
            issues.append(
                ValidationIssue(
                    "error",
                    "pivot_mesh_inconsistent",
                    f"{j.id} pivot far outside parent/child mesh bounds — physically inconsistent",
                )
            )
            suspicious_joints.append(
                {
                    "id": j.id,
                    "reason": "pivot_mesh_inconsistent",
                    "confidence": j.confidence,
                    "type": j.joint_type.value,
                }
            )
        elif not near:
            issues.append(
                ValidationIssue(
                    "warning",
                    "pivot_outside",
                    f"{j.id} pivot outside usual part bbox margin",
                )
            )
            suspicious_joints.append(
                {
                    "id": j.id,
                    "reason": "pivot_outside",
                    "confidence": j.confidence,
                    "type": j.joint_type.value,
                }
            )


def _check_fk_and_smoke(
    tree: KinematicTree,
    tol: Tolerances,
    issues: list[ValidationIssue],
    metrics: dict[str, Any],
) -> None:
    fk_err = _fk_pivot_error(tree)
    rest_err = _rest_pose_error(tree)
    metrics["fk_pivot_mean_err_m"] = fk_err
    metrics["rest_pose_mean_err_m"] = rest_err

    # Critical: rest pose / FK placement broken
    if fk_err > tol.rest_pose_hausdorff_m or rest_err > tol.rest_pose_hausdorff_m:
        issues.append(
            ValidationIssue(
                "error",
                "fk_placement",
                f"FK/rest-pose mismatch mean fk={fk_err:.4f} rest={rest_err:.4f} m — transform broken",
            )
        )
    elif max(fk_err, rest_err) > tol.rest_pose_hausdorff_m * 0.4:
        issues.append(
            ValidationIssue(
                "warning",
                "fk_soft",
                f"FK/rest-pose soft mismatch fk={fk_err:.4f} rest={rest_err:.4f} m",
            )
        )

    smoke = _smoke_revolute(tree, tol.smoke_angle_rad)
    metrics["smoke_axis_distance_delta_mean"] = smoke
    detach = _smoke_child_detach(tree, tol.smoke_angle_rad)
    metrics["smoke_child_detach_mean_m"] = detach

    if smoke > _SMOKE_FAIL_M or detach > _SMOKE_FAIL_M:
        issues.append(
            ValidationIssue(
                "error",
                "transform_break",
                f"Rotation smoke failed (axis_drift={smoke:.4f} detach={detach:.4f} m) — child flies apart",
            )
        )
    elif smoke > _SMOKE_WARN_M or detach > _SMOKE_WARN_M:
        issues.append(
            ValidationIssue(
                "warning",
                "smoke_spin",
                f"Revolute smoke drift axis={smoke:.4f} detach={detach:.4f} m",
            )
        )


def _check_quality_signals(
    ir: AssemblyIR,
    tree: KinematicTree,
    tol: Tolerances,
    issues: list[ValidationIssue],
    suspicious_joints: list[dict[str, Any]],
    unresolved_parts: list[dict[str, Any]],
    metrics: dict[str, Any],
) -> None:
    movable = [j for j in tree.joints if j.joint_type != JointType.FIXED]
    fixed = [j for j in tree.joints if j.joint_type == JointType.FIXED]
    metrics["n_movable_joints"] = len(movable)
    metrics["n_fixed_joints"] = len(fixed)
    metrics["n_links"] = len(tree.links)
    metrics["n_parts"] = len(ir.parts)

    # Soft: no / few movable joints on multi-part assemblies
    if len(movable) < 1 and len(ir.parts) > 1:
        issues.append(
            ValidationIssue(
                "warning",
                "no_movable_joints",
                "No movable joints detected — assembly may be under-resolved (low-quality STEP or welded)",
            )
        )
    elif len(ir.parts) >= 4 and len(movable) < 2:
        issues.append(
            ValidationIssue(
                "warning",
                "few_movable_joints",
                f"Only {len(movable)} movable joint(s) for {len(ir.parts)} parts — possible missing joints",
            )
        )

    for j in movable:
        if j.confidence < _LOW_JOINT_CONF:
            issues.append(
                ValidationIssue(
                    "warning",
                    "low_confidence_joint",
                    f"{j.id} confidence={j.confidence:.3f} ({j.parent}->{j.child})",
                )
            )
            suspicious_joints.append(
                {
                    "id": j.id,
                    "reason": "low_confidence",
                    "confidence": j.confidence,
                    "type": j.joint_type.value,
                    "parent": j.parent,
                    "child": j.child,
                }
            )

    for j in fixed:
        if j.confidence <= _WEAK_FIXED_CONF:
            issues.append(
                ValidationIssue(
                    "warning",
                    "weak_fixed_joint",
                    f"{j.id} weak fixed attach conf={j.confidence:.3f} ({j.parent}->{j.child})",
                )
            )
            suspicious_joints.append(
                {
                    "id": j.id,
                    "reason": "weak_fixed",
                    "confidence": j.confidence,
                    "type": "fixed",
                    "parent": j.parent,
                    "child": j.child,
                }
            )

    # Hierarchy meta: orphans
    for item in tree.meta.get("suspicious_orphans") or []:
        unresolved_parts.append({**item, "status": "suspicious_orphan"})
        issues.append(
            ValidationIssue(
                "warning",
                "suspicious_orphan",
                f"orphan {item.get('orphan')} weakly attached to {item.get('host')} "
                f"score={item.get('score')}",
            )
        )

    for item in tree.meta.get("merged_orphans") or []:
        # Merged is often OK (fasteners) — info unless score low
        score = float(item.get("score") or 0.0)
        if score < 0.5:
            unresolved_parts.append({**item, "status": "weak_merge"})
            issues.append(
                ValidationIssue(
                    "warning",
                    "weak_orphan_merge",
                    f"orphan {item.get('orphan')} merged into {item.get('host')} with weak score={score}",
                )
            )
        else:
            unresolved_parts.append({**item, "status": "merged"})

    # Parts not represented on any link
    linked_parts: set[str] = set()
    for link in tree.links:
        linked_parts.update(link.part_ids)
    for p in ir.parts:
        if p.id not in linked_parts:
            unresolved_parts.append({"part": p.id, "status": "missing_from_tree"})
            issues.append(
                ValidationIssue(
                    "warning",
                    "part_not_in_tree",
                    f"part {p.id} missing from kinematic links",
                )
            )


def _check_package(
    desc: RobotDesc,
    out: Path,
    issues: list[ValidationIssue],
    metrics: dict[str, Any],
) -> None:
    missing_meshes = 0
    for link in desc.links:
        mesh = link.get("mesh") if isinstance(link, dict) else getattr(link, "mesh", None)
        if not mesh:
            continue
        if not (out / mesh).is_file():
            missing_meshes += 1
            issues.append(
                ValidationIssue(
                    "warning",
                    "missing_mesh",
                    f"missing mesh file {mesh} (run stage meshes / --meshes for export)",
                )
            )
    metrics["n_robot_joints"] = len(desc.joints)
    metrics["n_robot_links"] = len(desc.links)
    metrics["missing_meshes"] = missing_meshes


def _overall_confidence(
    tree: KinematicTree,
    issues: list[ValidationIssue],
    suspicious_joints: list[dict[str, Any]],
    unresolved_parts: list[dict[str, Any]],
    metrics: dict[str, Any],
) -> float:
    movable = [j for j in tree.joints if j.joint_type != JointType.FIXED]
    if movable:
        base = float(np.mean([j.confidence for j in movable]))
    else:
        # No movable joints: low but not zero (may be a static assembly)
        base = 0.35

    n_err = sum(1 for i in issues if i.severity == "error")
    n_warn = sum(1 for i in issues if i.severity == "warning")
    penalty = 0.25 * n_err + 0.06 * n_warn
    penalty += 0.04 * len(suspicious_joints)
    # Only penalize unresolved that are not clean merges
    soft_unresolved = [
        u
        for u in unresolved_parts
        if u.get("status") in ("suspicious_orphan", "weak_merge", "missing_from_tree")
    ]
    penalty += 0.05 * len(soft_unresolved)

    # Density: many parts, few joints → soft confidence hit (already warned)
    n_parts = int(metrics.get("n_parts") or 0)
    n_mov = int(metrics.get("n_movable_joints") or 0)
    if n_parts >= 4 and n_mov < max(1, n_parts // 3):
        penalty += 0.08

    return float(np.clip(base - penalty, 0.0, 1.0))


def _has_cycle(tree: KinematicTree) -> bool:
    adj: dict[str, list[str]] = {l.id: [] for l in tree.links}
    for j in tree.joints:
        adj[j.parent].append(j.child)
        adj[j.child].append(j.parent)
    visited: set[str] = set()

    def dfs(u: str, parent: str | None) -> bool:
        visited.add(u)
        for v in adj[u]:
            if v == parent:
                continue
            if v in visited or dfs(v, u):
                return True
        return False

    for lid in adj:
        if lid not in visited:
            if dfs(lid, None):
                return True
    return False


def _fk_pivot_error(tree: KinematicTree) -> float:
    """Forward-kinematics of joint origins from base must match stored world pivots."""
    world_pos = {tree.base_link: np.zeros(3, dtype=np.float64)}
    by_parent: dict[str, list] = {}
    for j in tree.joints:
        by_parent.setdefault(j.parent, []).append(j)

    from collections import deque

    q = deque([tree.base_link])
    errs = []
    while q:
        pid = q.popleft()
        for j in by_parent.get(pid, []):
            pred = world_pos[pid] + as_vec3(j.origin_local)
            err = float(np.linalg.norm(pred - as_vec3(j.origin_world)))
            errs.append(err)
            world_pos[j.child] = pred
            q.append(j.child)
    return float(np.mean(errs)) if errs else 0.0


def _rest_pose_error(tree: KinematicTree) -> float:
    """origin_local must map through parent link_world to origin_world."""
    errs = []
    for j in tree.joints:
        Mp = mat4_from_list(tree.link_world[j.parent])
        pred = transform_point(Mp, as_vec3(j.origin_local))
        err = float(np.linalg.norm(pred - as_vec3(j.origin_world)))
        errs.append(err)
    return float(np.mean(errs)) if errs else 0.0


def _smoke_revolute(tree: KinematicTree, angle: float) -> float:
    """Axis-distance of a sample point should be invariant under revolute spin."""
    deltas = []
    for j in tree.joints:
        if j.joint_type != JointType.REVOLUTE:
            continue
        origin = as_vec3(j.origin_world)
        axis = normalize(as_vec3(j.axis_world))
        helper = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(helper, axis))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        radial = np.cross(axis, helper)
        radial = radial / (np.linalg.norm(radial) + 1e-12)
        p0 = origin + radial * 0.05
        d0 = float(np.linalg.norm(np.cross(p0 - origin, axis)))
        R = rotation_matrix_axis_angle(axis, angle)
        p1 = origin + R @ (p0 - origin)
        d1 = float(np.linalg.norm(np.cross(p1 - origin, axis)))
        deltas.append(abs(d1 - d0))
    return float(np.mean(deltas)) if deltas else 0.0


def _smoke_child_detach(tree: KinematicTree, angle: float) -> float:
    """
    Child link origin should stay at constant distance to joint axis after spin.
    Large jump ⇒ child 'flies apart' (broken joint frame).
    """
    deltas = []
    for j in tree.joints:
        if j.joint_type != JointType.REVOLUTE:
            continue
        if j.child not in tree.link_world:
            continue
        origin = as_vec3(j.origin_world)
        axis = normalize(as_vec3(j.axis_world))
        child_o = mat4_from_list(tree.link_world[j.child])[:3, 3]
        d0 = float(np.linalg.norm(np.cross(child_o - origin, axis)))
        R = rotation_matrix_axis_angle(axis, angle)
        child_1 = origin + R @ (child_o - origin)
        d1 = float(np.linalg.norm(np.cross(child_1 - origin, axis)))
        deltas.append(abs(d1 - d0))
    return float(np.mean(deltas)) if deltas else 0.0
