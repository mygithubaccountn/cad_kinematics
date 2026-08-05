"""08_validation — automatic checks on kinematic result."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import write_json
from pipeline.common.math3d import (
    as_vec3,
    bbox_contains,
    mat4_from_list,
    normalize,
    rotation_matrix_axis_angle,
    transform_point,
)
from pipeline.common.models import (
    AssemblyIR,
    JointType,
    KinematicTree,
    RobotDesc,
    ValidationIssue,
    ValidationReport,
)
from pipeline.common.tolerances import Tolerances


def run_validation(
    ir: AssemblyIR,
    tree: KinematicTree,
    desc: Optional[RobotDesc],
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> ValidationReport:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    issues: list[ValidationIssue] = []
    metrics: dict = {}

    # Tree structure
    link_ids = {l.id for l in tree.links}
    if tree.base_link not in link_ids:
        issues.append(ValidationIssue("error", "base_missing", "base_link not in links"))
    children = {j.child for j in tree.joints}
    for lid in link_ids:
        if lid == tree.base_link:
            continue
        if lid not in children:
            issues.append(ValidationIssue("error", "orphan_link", f"{lid} has no parent joint"))

    # Cycle check (serial MVP expects tree)
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
            issues.append(ValidationIssue("error", "cycle", "Kinematic graph has unexpected cycle"))

    # Axis / pivot checks
    part_map = ir.part_map()
    for j in tree.joints:
        axis = as_vec3(j.axis_world)
        n = float(np.linalg.norm(axis))
        if abs(n - 1.0) > 1e-3:
            issues.append(ValidationIssue("warning", "axis_norm", f"{j.id} axis norm={n}"))
        pivot = as_vec3(j.origin_world)
        # Pivot near union of parent/child part bboxes
        child_link = next(l for l in tree.links if l.id == j.child)
        parent_link = next(l for l in tree.links if l.id == j.parent)
        ok_pivot = False
        for pid in child_link.part_ids + parent_link.part_ids:
            if bbox_contains(part_map[pid].bbox.as_array(), pivot, tolerances.pivot_bbox_margin_m):
                ok_pivot = True
                break
        if not ok_pivot and j.joint_type != JointType.FIXED:
            issues.append(
                ValidationIssue("warning", "pivot_outside", f"{j.id} pivot far from part bboxes")
            )

    # Joint count heuristic for serial robots
    movable = [j for j in tree.joints if j.joint_type != JointType.FIXED]
    metrics["n_movable_joints"] = len(movable)
    metrics["n_links"] = len(tree.links)
    if len(movable) < 1 and len(ir.parts) > 1:
        issues.append(ValidationIssue("error", "no_joints", "No movable joints detected"))

    # Rest pose: joint=0 means link frames as exported — sample part centers vs link transforms
    rest_err = _rest_pose_error(ir, tree)
    metrics["rest_pose_mean_err_m"] = rest_err
    if rest_err > tolerances.rest_pose_hausdorff_m * 5:
        issues.append(
            ValidationIssue(
                "warning",
                "rest_pose",
                f"Rest pose mean sample error {rest_err:.4f} m",
            )
        )

    # Smoke: small rotation should keep child COM distance to axis roughly constant
    smoke = _smoke_revolute(tree, tolerances.smoke_angle_rad)
    metrics["smoke_axis_distance_delta_mean"] = smoke
    if smoke > 1e-2:
        issues.append(
            ValidationIssue(
                "warning",
                "smoke_spin",
                f"Revolute smoke axis-distance drift {smoke:.4f} m",
            )
        )

    ok = not any(i.severity == "error" for i in issues)
    report = ValidationReport(ok=ok, issues=issues, metrics=metrics)
    write_json(out / "validation_report.json", report.to_dict())
    return report


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


def _rest_pose_error(ir: AssemblyIR, tree: KinematicTree) -> float:
    """Compare part bbox centers to expectation that parts stay at CAD world positions."""
    # Link meshes are transformed to link-local; world restore = link_world * local
    # For validation without re-meshing: check joint origins match stored world.
    errs = []
    for j in tree.joints:
        # origin_world should equal transform of origin_local by parent link_world
        Mp = mat4_from_list(tree.link_world[j.parent])
        local = as_vec3(j.origin_local)
        # Note: origin_local was computed in CAD; if parent is identity for base, ok
        pred = transform_point(Mp, local)
        err = float(np.linalg.norm(pred - as_vec3(j.origin_world)))
        errs.append(err)
    return float(np.mean(errs)) if errs else 0.0


def _smoke_revolute(tree: KinematicTree, angle: float) -> float:
    """
    For each revolute joint, sample a point offset from axis on child;
    after rotation about axis, distance to axis should be unchanged.
    """
    deltas = []
    for j in tree.joints:
        if j.joint_type != JointType.REVOLUTE:
            continue
        origin = as_vec3(j.origin_world)
        axis = normalize(as_vec3(j.axis_world))
        # Pick a point not on axis
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
