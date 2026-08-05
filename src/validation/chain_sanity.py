"""Chain sanity — structural / kinematic consistency (no joint scoring).

Checks whether the selected joints form a coherent parent→child tree and
whether each child is the body that should move under that joint.
"""

from __future__ import annotations

from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.io_util import write_json
from common.math3d import as_vec3, normalize, rotation_matrix_axis_angle
from common.models import AssemblyIR, JointType, KinematicTree, RobotDesc, ValidationIssue
from common.tolerances import Tolerances

_CHILD_MOTION_WARN_M = 1.0e-4
_CHILD_MOTION_FAIL_M = 1.0e-5  # child should move at least this under small spin


def run_chain_sanity(
    ir: AssemblyIR,
    tree: KinematicTree,
    desc: Optional[RobotDesc],
    out_dir: Path,
    tol: Tolerances,
    issues: list[ValidationIssue],
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    link_ids = {l.id for l in tree.links}
    part_map = ir.part_map()
    link_by_id = {l.id: l for l in tree.links}

    # --- identity / references ---
    if tree.base_link not in link_ids:
        _add(
            checks,
            issues,
            "error",
            "chain_base_missing",
            f"base_link {tree.base_link} not in links",
        )
    else:
        _add(checks, issues, "ok", "chain_base_ok", f"base={tree.base_link}")

    for j in tree.joints:
        if j.parent not in link_ids or j.child not in link_ids:
            _add(
                checks,
                issues,
                "error",
                "chain_unknown_link",
                f"{j.id} references unknown link parent={j.parent} child={j.child}",
            )

    # --- unique parent (tree, not DAG with multi-parent) ---
    child_count = Counter(j.child for j in tree.joints)
    for child, n in child_count.items():
        if n > 1:
            _add(
                checks,
                issues,
                "error",
                "chain_multi_parent",
                f"{child} has {n} parent joints — not a tree",
            )

    # --- every non-base link has a parent ---
    children = set(child_count)
    for lid in link_ids:
        if lid == tree.base_link:
            continue
        if lid not in children:
            _add(
                checks,
                issues,
                "error",
                "chain_orphan_link",
                f"{lid} has no parent joint",
            )

    # --- no unexpected cycle (undirected) ---
    if _undirected_cycle(tree) and not tree.meta.get("parallel_loops"):
        _add(checks, issues, "error", "chain_cycle", "unexpected cycle in joint graph")
    elif tree.meta.get("parallel_loops"):
        _add(checks, issues, "info", "chain_parallel", "parallel loops declared in meta")

    # --- reachability from base ---
    reachable = _reachable_from_base(tree)
    unreachable = sorted(link_ids - reachable)
    if unreachable:
        _add(
            checks,
            issues,
            "error",
            "chain_unreachable",
            f"links not reachable from base: {unreachable}",
        )
    else:
        _add(checks, issues, "ok", "chain_connected", f"all {len(link_ids)} links reachable")

    # --- topological / depth order ---
    order = _bfs_order(tree)
    depth = {lid: i for i, lid in enumerate(order)}
    order_ok = True
    for j in tree.joints:
        if j.parent in depth and j.child in depth and depth[j.parent] > depth[j.child]:
            order_ok = False
            _add(
                checks,
                issues,
                "error",
                "chain_order",
                f"{j.id}: parent {j.parent} deeper than child {j.child} in BFS order",
            )
    if order_ok and tree.joints:
        _add(
            checks,
            issues,
            "ok",
            "chain_order_ok",
            f"BFS order from base: {' → '.join(order)}",
        )

    # --- serial vs branched topology (info / soft) ---
    deg = Counter()
    for j in tree.joints:
        if j.joint_type == JointType.FIXED:
            continue
        deg[j.parent] += 1
        deg[j.child] += 1
    branch_nodes = [n for n, d in deg.items() if d > 2]
    movable = [j for j in tree.joints if j.joint_type != JointType.FIXED]
    if branch_nodes:
        _add(
            checks,
            issues,
            "warning",
            "chain_branched",
            f"movable graph branches at {branch_nodes} (serial path expected for many arms)",
        )
    elif movable:
        _add(
            checks,
            issues,
            "ok",
            "chain_serial_or_star",
            f"movable joints={len(movable)} forms path/star (no node degree>2)",
        )

    # --- child payload / motion role ---
    for j in tree.joints:
        child_link = link_by_id.get(j.child)
        parent_link = link_by_id.get(j.parent)
        if not child_link or not parent_link:
            continue
        if not child_link.part_ids:
            _add(
                checks,
                issues,
                "error",
                "chain_empty_child",
                f"{j.id}: child link {j.child} has no part_ids",
            )
            continue

        # Child should own distinct parts from parent
        overlap = set(child_link.part_ids) & set(parent_link.part_ids)
        if overlap:
            _add(
                checks,
                issues,
                "error",
                "chain_shared_parts",
                f"{j.id}: parent/child share parts {sorted(overlap)}",
            )

        if j.joint_type == JointType.FIXED:
            continue

        # Motion role: a child geometry point off the axis must move under spin
        motion = _child_motion_role(tree, j, child_link, part_map, float(tol.smoke_angle_rad))
        sev = "ok"
        if motion["probe_delta_m"] < _CHILD_MOTION_FAIL_M:
            sev = "error"
        elif motion["probe_delta_m"] < _CHILD_MOTION_WARN_M:
            sev = "warning"
        if motion.get("radial_m", 0.0) < 1e-4 and sev != "error":
            # COM/probe nearly on axis (typical shaft) — soft note only
            sev = "ok"
            motion["note"] = "probe_near_axis_but_ok"
        _add(
            checks,
            issues,
            sev,
            "chain_child_motion",
            (
                f"{j.id}: probe_Δ={motion['probe_delta_m']:.5f}m "
                f"radial={motion.get('radial_m', 0):.5f}m parts={child_link.part_ids}"
                + (f" ({motion.get('note')})" if motion.get("note") else "")
            ),
            extra=motion,
        )

    # --- robot.json mirrors tree ---
    if desc is not None:
        tree_pairs = {(j.parent, j.child, j.joint_type.value) for j in tree.joints}
        desc_pairs = {
            (j["parent"], j["child"], j.get("type", "")) for j in desc.joints
        }
        if tree_pairs != desc_pairs:
            _add(
                checks,
                issues,
                "warning",
                "chain_export_mismatch",
                f"robot.json joints differ from kinematic_tree "
                f"(tree={len(tree_pairs)} export={len(desc_pairs)})",
            )
        else:
            _add(checks, issues, "ok", "chain_export_match", "robot.json joint set matches tree")

    summary = {
        "n_ok": sum(1 for c in checks if c["severity"] == "ok"),
        "n_warning": sum(1 for c in checks if c["severity"] == "warning"),
        "n_error": sum(1 for c in checks if c["severity"] == "error"),
        "n_info": sum(1 for c in checks if c["severity"] == "info"),
        "bfs_order": order,
        "n_movable": len(movable),
    }
    report = {"summary": summary, "checks": checks}
    write_json(Path(out_dir) / "chain_sanity_report.json", report)
    return report


def _add(
    checks: list[dict],
    issues: list[ValidationIssue],
    severity: str,
    code: str,
    message: str,
    extra: Optional[dict] = None,
) -> None:
    row = {"severity": severity, "code": code, "message": message}
    if extra:
        row["detail"] = extra
    checks.append(row)
    if severity in ("error", "warning"):
        issues.append(ValidationIssue(severity, code, message))


def _undirected_cycle(tree: KinematicTree) -> bool:
    adj: dict[str, list[str]] = defaultdict(list)
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

    for lid in list(adj.keys()):
        if lid not in visited and dfs(lid, None):
            return True
    return False


def _reachable_from_base(tree: KinematicTree) -> set[str]:
    adj: dict[str, list[str]] = defaultdict(list)
    for j in tree.joints:
        adj[j.parent].append(j.child)
        adj[j.child].append(j.parent)
    seen = {tree.base_link}
    q = deque([tree.base_link])
    while q:
        u = q.popleft()
        for v in adj[u]:
            if v not in seen:
                seen.add(v)
                q.append(v)
    return seen


def _bfs_order(tree: KinematicTree) -> list[str]:
    by_parent: dict[str, list] = defaultdict(list)
    for j in tree.joints:
        by_parent[j.parent].append(j)
    order = [tree.base_link]
    q = deque([tree.base_link])
    seen = {tree.base_link}
    while q:
        u = q.popleft()
        kids = sorted(by_parent.get(u, []), key=lambda j: (j.child, j.id))
        for j in kids:
            if j.child in seen:
                continue
            seen.add(j.child)
            order.append(j.child)
            q.append(j.child)
    # append any leftover links deterministically
    for lid in sorted(l.id for l in tree.links):
        if lid not in seen:
            order.append(lid)
    return order


def _farthest_corner_from_axis(
    part_ids: list[str],
    part_map: dict,
    origin: np.ndarray,
    axis: np.ndarray,
) -> tuple[np.ndarray, float]:
    best_pt = origin.copy()
    best_r = -1.0
    for pid in part_ids:
        p = part_map.get(pid)
        if p is None:
            continue
        bb = p.bbox.as_array()
        for x in (bb[0, 0], bb[1, 0]):
            for y in (bb[0, 1], bb[1, 1]):
                for z in (bb[0, 2], bb[1, 2]):
                    pt = np.array([x, y, z], dtype=np.float64)
                    r = float(np.linalg.norm(np.cross(pt - origin, axis)))
                    if r > best_r:
                        best_r = r
                        best_pt = pt
    return best_pt, max(best_r, 0.0)


def _child_motion_role(
    tree: KinematicTree,
    j,
    child_link,
    part_map: dict,
    angle: float,
) -> dict[str, Any]:
    origin = as_vec3(j.origin_world)
    axis = normalize(as_vec3(j.axis_world))
    probe, radial = _farthest_corner_from_axis(child_link.part_ids, part_map, origin, axis)

    if j.joint_type == JointType.REVOLUTE:
        R = rotation_matrix_axis_angle(axis, angle)
        probe1 = origin + R @ (probe - origin)
    elif j.joint_type == JointType.PRISMATIC:
        slide = 0.01
        probe1 = probe + axis * slide
    else:
        return {"probe_delta_m": 0.0, "radial_m": radial, "probe": probe.tolist()}

    return {
        "probe_delta_m": float(np.linalg.norm(probe1 - probe)),
        "radial_m": radial,
        "probe": probe.tolist(),
        "child_com_delta_m": float(np.linalg.norm(probe1 - probe)),  # alias for older readers
    }
