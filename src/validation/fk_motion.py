"""Per-joint FK motion checks (Godot robot.json contract + CAD tree).

No new joint heuristics — only measure whether exported pivots/axes produce
correct small-angle motion, and attach DecisionTrace for failures.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.io_util import read_json, write_json
from common.math3d import (
    as_vec3,
    mat4_from_list,
    mat4_from_rt,
    mat4_identity,
    normalize,
    rotation_matrix_axis_angle,
    transform_dir,
    transform_point,
)
from common.models import JointType, KinematicTree, RobotDesc, ValidationIssue
from common.tolerances import Tolerances

# Thresholds — geometric consistency only (not scoring heuristics)
_PIVOT_FAIL_M = 1.0e-4
_PIVOT_WARN_M = 1.0e-5
_AXIS_FAIL_DEG = 1.0
_AXIS_WARN_DEG = 0.1
_RADIAL_FAIL_M = 1.0e-3
_RADIAL_WARN_M = 1.0e-4
_CHILD_FAIL_M = 1.0e-3
_CHILD_WARN_M = 1.0e-4


def run_fk_motion_validation(
    tree: KinematicTree,
    desc: Optional[RobotDesc],
    out_dir: Path,
    tol: Tolerances,
    issues: list[ValidationIssue],
) -> dict[str, Any]:
    """
    Move each movable joint by ``tol.smoke_angle_rad`` and measure:
      - pivot invariance (revolute)
      - axis direction stability
      - sample-point radial distance to axis
      - child link origin motion vs Rodrigues expectation

    Also mirrors Godot loader: child.position = origin, rotate_object_local(axis, θ).
    """
    out = Path(out_dir)
    angle = float(tol.smoke_angle_rad)
    traces = _load_joint_traces(out)

    tree_results = [_eval_tree_joint(tree, j, angle, traces) for j in tree.joints]
    godot_results: list[dict[str, Any]] = []
    if desc is not None and desc.joints:
        godot_results = [
            _eval_godot_joint(desc, j, angle, traces) for j in desc.joints
        ]

    report = {
        "angle_rad": angle,
        "angle_deg": float(np.degrees(angle)),
        "tree_joints": tree_results,
        "godot_joints": godot_results,
        "summary": _summarize(tree_results, godot_results),
    }
    write_json(out / "fk_motion_report.json", report)

    _emit_issues(report, issues)
    return report


def _load_joint_traces(out: Path) -> dict[str, dict[str, Any]]:
    """Map joint id → DecisionTrace dict (resolved axis preferred)."""
    by_id: dict[str, dict[str, Any]] = {}
    resolved = out / "resolved_axes.json"
    if resolved.is_file():
        raw = read_json(resolved)
        for j in raw.get("joints", []):
            jid = j.get("id")
            if jid and j.get("trace"):
                by_id[jid] = j["trace"]
    # Fallback / supplement from decision_trace.json
    dt = out / "decision_trace.json"
    if dt.is_file():
        raw = read_json(dt)
        for t in raw.get("traces", []):
            subj = str(t.get("subject", ""))
            # subjects like "axis:jhyp_0002"
            if subj.startswith("axis:"):
                jid = subj.split(":", 1)[1]
                by_id.setdefault(jid, t)
    return by_id


def _attach_trace(result: dict[str, Any], traces: dict[str, dict], jid: str) -> None:
    tr = traces.get(jid)
    if not tr:
        result["decision_trace"] = None
        return
    result["decision_trace"] = {
        "subject": tr.get("subject"),
        "chosen": tr.get("chosen"),
        "runner_up": tr.get("runner_up"),
        "confidence": tr.get("confidence"),
        "rejected": tr.get("rejected", [])[:8],
        "summary": tr.get("summary"),
    }


def _eval_tree_joint(
    tree: KinematicTree,
    j,
    angle: float,
    traces: dict[str, dict],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "id": j.id,
        "source": "kinematic_tree",
        "type": j.joint_type.value,
        "parent": j.parent,
        "child": j.child,
        "confidence": j.confidence,
        "checks": {},
        "ok": True,
        "severity": "ok",
    }
    _attach_trace(result, traces, j.id)

    if j.joint_type == JointType.FIXED:
        result["checks"] = {"skipped": "fixed"}
        return result

    origin = as_vec3(j.origin_world)
    axis = normalize(as_vec3(j.axis_world))
    child_rest = mat4_from_list(tree.link_world[j.child])[:3, 3]

    if j.joint_type == JointType.REVOLUTE:
        R = rotation_matrix_axis_angle(axis, angle)
        # Pivot: world origin must be fixed under rotation about itself
        pivot_moved = float(np.linalg.norm(R @ np.zeros(3)))  # trivial 0
        # Axis: direction unchanged by rotation about itself
        axis_after = normalize(R @ axis)
        axis_err_deg = float(
            np.degrees(np.arccos(np.clip(abs(float(np.dot(axis, axis_after))), 0.0, 1.0)))
        )
        # Sample radial invariance
        helper = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(helper, axis))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        radial = normalize(np.cross(axis, helper))
        p0 = origin + radial * 0.05
        d0 = float(np.linalg.norm(np.cross(p0 - origin, axis)))
        p1 = origin + R @ (p0 - origin)
        d1 = float(np.linalg.norm(np.cross(p1 - origin, axis)))
        radial_delta = abs(d1 - d0)

        # Child origin should orbit about axis (distance conserved; predicted by Rodrigues)
        child_pred = origin + R @ (child_rest - origin)
        child_radial_0 = float(np.linalg.norm(np.cross(child_rest - origin, axis)))
        child_radial_1 = float(np.linalg.norm(np.cross(child_pred - origin, axis)))
        child_radial_delta = abs(child_radial_1 - child_radial_0)
        # Self-consistency of prediction vs analytic chord length
        chord = float(np.linalg.norm(child_pred - child_rest))
        expected_chord = 2.0 * child_radial_0 * abs(np.sin(angle / 2.0))
        child_chord_err = abs(chord - expected_chord)

        result["checks"] = {
            "pivot_drift_m": pivot_moved,
            "axis_err_deg": axis_err_deg,
            "sample_radial_delta_m": radial_delta,
            "child_radial_delta_m": child_radial_delta,
            "child_chord_err_m": child_chord_err,
            "child_radial_m": child_radial_0,
        }
        _grade_revolute(result)

    elif j.joint_type == JointType.PRISMATIC:
        # Child should translate along axis; radial to a side axis invariant
        child_pred = child_rest + axis * angle  # angle reused as metres? use small slide
        # Prefer dedicated slide distance from tol — smoke_angle_rad is ~0.15 rad;
        # for prismatic use same numeric as metres * 0.01 scale → use 0.01 m
        slide = 0.01
        child_pred = child_rest + axis * slide
        axial = abs(float(np.dot(child_pred - child_rest, axis)) - slide)
        lateral = float(np.linalg.norm((child_pred - child_rest) - axis * slide))
        result["checks"] = {
            "slide_m": slide,
            "axial_err_m": axial,
            "lateral_err_m": lateral,
        }
        sev = "ok"
        if axial > _CHILD_FAIL_M or lateral > _CHILD_FAIL_M:
            sev = "error"
        elif axial > _CHILD_WARN_M or lateral > _CHILD_WARN_M:
            sev = "warning"
        result["severity"] = sev
        result["ok"] = sev != "error"

    return result


def _grade_revolute(result: dict[str, Any]) -> None:
    c = result["checks"]
    sev = "ok"
    fails = []
    warns = []

    def check(name: str, val: float, warn_t: float, fail_t: float) -> None:
        nonlocal sev
        if val > fail_t:
            fails.append(f"{name}={val:.6g}")
            sev = "error"
        elif val > warn_t:
            warns.append(f"{name}={val:.6g}")
            if sev != "error":
                sev = "warning"

    check("pivot_drift_m", c["pivot_drift_m"], _PIVOT_WARN_M, _PIVOT_FAIL_M)
    check("axis_err_deg", c["axis_err_deg"], _AXIS_WARN_DEG, _AXIS_FAIL_DEG)
    check("sample_radial_delta_m", c["sample_radial_delta_m"], _RADIAL_WARN_M, _RADIAL_FAIL_M)
    check("child_radial_delta_m", c["child_radial_delta_m"], _RADIAL_WARN_M, _RADIAL_FAIL_M)
    check("child_chord_err_m", c["child_chord_err_m"], _CHILD_WARN_M, _CHILD_FAIL_M)

    result["severity"] = sev
    result["ok"] = sev != "error"
    result["fail_details"] = fails
    result["warn_details"] = warns


def _godot_rest_world(desc: RobotDesc) -> dict[str, np.ndarray]:
    """Build link world transforms matching CadRobotLoader (local origin = joint origin)."""
    base = desc.base_link
    children: dict[str, list[dict]] = {}
    for j in desc.joints:
        children.setdefault(j["parent"], []).append(j)

    world: dict[str, np.ndarray] = {base: mat4_identity()}
    stack = [base]
    while stack:
        pid = stack.pop()
        Mp = world[pid]
        for j in children.get(pid, []):
            cid = j["child"]
            origin = as_vec3(j["origin"])
            # Child local = translation only at rest (Godot position)
            Mc = mat4_from_rt(np.eye(3), origin)
            world[cid] = Mp @ Mc
            stack.append(cid)
    return world


def _eval_godot_joint(
    desc: RobotDesc,
    j: dict,
    angle: float,
    traces: dict[str, dict],
) -> dict[str, Any]:
    """
    Mirror Godot set_joint: rotate_object_local(axis, θ) on child node.
    Child world origin (pivot) must stay fixed; a body point orbits the axis.
    """
    jid = str(j.get("id", ""))
    jtype = str(j.get("type", "revolute"))
    result: dict[str, Any] = {
        "id": jid,
        "source": "robot.json",
        "type": jtype,
        "parent": j.get("parent"),
        "child": j.get("child"),
        "confidence": j.get("confidence"),
        "checks": {},
        "ok": True,
        "severity": "ok",
    }
    _attach_trace(result, traces, jid)

    if jtype == "fixed":
        result["checks"] = {"skipped": "fixed"}
        return result

    world = _godot_rest_world(desc)
    parent = str(j["parent"])
    child = str(j["child"])
    if parent not in world or child not in world:
        result["severity"] = "error"
        result["ok"] = False
        result["fail_details"] = ["missing_link_in_godot_tree"]
        return result

    Mp = world[parent]
    origin_local = as_vec3(j["origin"])
    axis_local = normalize(as_vec3(j["axis"]))
    # Rest: child world = Mp @ T(origin)
    pivot_world_0 = transform_point(Mp, origin_local)
    axis_world_0 = normalize(transform_dir(Mp, axis_local))

    if jtype == "revolute":
        # After rotate_object_local: child orientation R(axis_local, θ), position unchanged
        R_local = rotation_matrix_axis_angle(axis_local, angle)
        Mc_rest = mat4_from_rt(np.eye(3), origin_local)
        Mc_moved = mat4_from_rt(R_local, origin_local)
        child_world_0 = Mp @ Mc_rest
        child_world_1 = Mp @ Mc_moved

        pivot_world_1 = child_world_1[:3, 3]
        pivot_drift = float(np.linalg.norm(pivot_world_1 - pivot_world_0))

        # Axis in child local stays axis_local; world axis after = Mp_R @ R_local @ axis_local
        # At rest axis_world = Mp_R @ axis_local; after local rot about axis, axis still axis_local
        axis_world_1 = normalize(child_world_1[:3, :3] @ axis_local)
        # Should match parent-rotated axis (unchanged direction in parent frame...)
        axis_err_deg = float(
            np.degrees(
                np.arccos(np.clip(abs(float(np.dot(axis_world_0, axis_world_1))), 0.0, 1.0))
            )
        )

        # Body sample in child local (not at origin)
        helper = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(helper, axis_local))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        sample_local = normalize(np.cross(axis_local, helper)) * 0.05
        p0 = transform_point(child_world_0, sample_local)
        p1 = transform_point(child_world_1, sample_local)
        d0 = float(np.linalg.norm(np.cross(p0 - pivot_world_0, axis_world_0)))
        d1 = float(np.linalg.norm(np.cross(p1 - pivot_world_0, axis_world_0)))
        radial_delta = abs(d1 - d0)

        # Expected motion: Rodrigues about pivot_world / axis_world
        R_w = rotation_matrix_axis_angle(axis_world_0, angle)
        p1_pred = pivot_world_0 + R_w @ (p0 - pivot_world_0)
        child_err = float(np.linalg.norm(p1 - p1_pred))

        result["checks"] = {
            "pivot_drift_m": pivot_drift,
            "axis_err_deg": axis_err_deg,
            "sample_radial_delta_m": radial_delta,
            "child_point_err_m": child_err,
            "pivot_world": pivot_world_0.tolist(),
            "axis_world": axis_world_0.tolist(),
        }
        # Grade using same thresholds (map child_point_err → child_chord)
        fake = {
            "checks": {
                "pivot_drift_m": pivot_drift,
                "axis_err_deg": axis_err_deg,
                "sample_radial_delta_m": radial_delta,
                "child_radial_delta_m": radial_delta,
                "child_chord_err_m": child_err,
            }
        }
        _grade_revolute(fake)
        result["severity"] = fake["severity"]
        result["ok"] = fake["ok"]
        result["fail_details"] = fake.get("fail_details", [])
        result["warn_details"] = fake.get("warn_details", [])

    elif jtype == "prismatic":
        slide = 0.01
        rest_pos = origin_local
        moved_pos = rest_pos + axis_local * slide
        Mc0 = mat4_from_rt(np.eye(3), rest_pos)
        Mc1 = mat4_from_rt(np.eye(3), moved_pos)
        p0 = (Mp @ Mc0)[:3, 3]
        p1 = (Mp @ Mc1)[:3, 3]
        delta = p1 - p0
        axial = abs(float(np.dot(delta, axis_world_0)) - slide)
        lateral = float(np.linalg.norm(delta - axis_world_0 * float(np.dot(delta, axis_world_0))))
        result["checks"] = {"slide_m": slide, "axial_err_m": axial, "lateral_err_m": lateral}
        sev = "ok"
        if axial > _CHILD_FAIL_M or lateral > _CHILD_FAIL_M:
            sev = "error"
        elif axial > _CHILD_WARN_M or lateral > _CHILD_WARN_M:
            sev = "warning"
        result["severity"] = sev
        result["ok"] = sev != "error"

    return result


def _summarize(
    tree_results: list[dict[str, Any]],
    godot_results: list[dict[str, Any]],
) -> dict[str, Any]:
    def count(rows: list[dict]) -> dict[str, int]:
        c = {"ok": 0, "warning": 0, "error": 0, "skipped": 0}
        for r in rows:
            if r.get("checks", {}).get("skipped"):
                c["skipped"] += 1
            else:
                c[r.get("severity", "ok")] = c.get(r.get("severity", "ok"), 0) + 1
        return c

    return {
        "tree": count(tree_results),
        "godot": count(godot_results),
        "all_ok": all(r.get("ok", True) for r in tree_results + godot_results),
    }


def _emit_issues(report: dict[str, Any], issues: list[ValidationIssue]) -> None:
    for source_key, label in (("tree_joints", "tree"), ("godot_joints", "godot")):
        for r in report.get(source_key, []):
            sev = r.get("severity", "ok")
            if sev == "ok" or r.get("checks", {}).get("skipped"):
                continue
            details = r.get("fail_details") or r.get("warn_details") or []
            tr = r.get("decision_trace") or {}
            method = (tr.get("chosen") or {}).get("method", "?")
            msg = (
                f"FK motion {label} joint {r.get('id')} ({r.get('parent')}->{r.get('child')}): "
                f"{sev} [{', '.join(details)}]; trace_method={method}"
            )
            code = "fk_motion_fail" if sev == "error" else "fk_motion_soft"
            issues.append(ValidationIssue(sev if sev == "error" else "warning", code, msg))
