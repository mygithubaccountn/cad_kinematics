"""Headless Godot-contract runtime: build Node3D-equivalent tree and wiggle joints.

Mirrors ``godot_test/addons/cad_robot_importer/robot_loader.gd`` without requiring
a Godot binary. Optional ``./run_godot_test.sh`` launches real Godot when installed.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.io_util import write_json
from common.math3d import as_vec3, mat4_from_rt, mat4_identity, normalize, rotation_matrix_axis_angle


@dataclass
class SimNode:
    id: str
    parent: Optional["SimNode"] = None
    children: list["SimNode"] = field(default_factory=list)
    # local transform
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))  # 3x3
    rest_position: Optional[np.ndarray] = None
    joint_id: str = ""
    joint_type: str = ""
    joint_axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    mesh: str = ""

    def world_matrix(self) -> np.ndarray:
        local = mat4_from_rt(self.rotation, self.position)
        if self.parent is None:
            return local
        return self.parent.world_matrix() @ local


def build_sim_tree(robot: dict[str, Any]) -> tuple[SimNode, dict[str, SimNode]]:
    """Same attachment rules as CadRobotLoader.build_tree."""
    links = robot.get("links", [])
    joints = robot.get("joints", [])
    base_id = str(robot.get("base_link", ""))

    nodes: dict[str, SimNode] = {}
    for link in links:
        lid = str(link["id"])
        nodes[lid] = SimNode(id=lid, mesh=str(link.get("mesh", "")))

    if base_id not in nodes:
        raise RuntimeError(f"base_link missing: {base_id}")

    root = nodes[base_id]
    pending = list(joints)
    guard = 0
    while pending and guard < 10000:
        guard += 1
        j = pending.pop(0)
        parent_id = str(j["parent"])
        child_id = str(j["child"])
        if parent_id not in nodes or child_id not in nodes:
            continue
        parent = nodes[parent_id]
        child = nodes[child_id]
        if child.parent is not None:
            continue
        if parent.parent is None and parent_id != base_id:
            pending.append(j)
            continue
        parent.children.append(child)
        child.parent = parent
        origin = as_vec3(j.get("origin", [0, 0, 0]))
        child.position = origin.copy()
        child.rest_position = origin.copy()
        child.joint_id = str(j.get("id", ""))
        child.joint_type = str(j.get("type", "revolute"))
        child.joint_axis = normalize(as_vec3(j.get("axis", [0, 0, 1])))

    return root, nodes


def set_joint(node: SimNode, value: float) -> None:
    """Mirror CadRobotLoader.set_joint."""
    if node.joint_type == "revolute":
        node.rotation = rotation_matrix_axis_angle(node.joint_axis, value)
    elif node.joint_type == "prismatic":
        rest = node.rest_position if node.rest_position is not None else node.position
        node.position = rest + node.joint_axis * value


def run_godot_runtime_test(
    robot_path: Path,
    out_dir: Path,
    angle_rad: float = 0.15,
) -> dict[str, Any]:
    robot = json.loads(Path(robot_path).read_text())
    root, nodes = build_sim_tree(robot)

    # Mesh presence (export integrity)
    mesh_issues = []
    base_dir = Path(robot_path).parent
    for n in nodes.values():
        if n.mesh and not (base_dir / n.mesh).is_file():
            mesh_issues.append(n.mesh)

    movable = [n for n in nodes.values() if n.joint_type in ("revolute", "prismatic")]
    joint_results = []
    all_ok = True

    # Rest snapshot of world positions
    rest_world = {lid: n.world_matrix()[:3, 3].copy() for lid, n in nodes.items()}

    for n in movable:
        # Reset all
        for m in movable:
            if m.joint_type == "revolute":
                m.rotation = np.eye(3)
            elif m.rest_position is not None:
                m.position = m.rest_position.copy()

        set_joint(n, angle_rad)
        moved = n.world_matrix()[:3, 3]
        pivot_rest = rest_world[n.id]
        # For revolute: child origin (pivot) must stay fixed in parent frame → world
        # pivot = parent_world @ origin; child's own rotation shouldn't move its origin
        pivot_drift = float(np.linalg.norm(moved - pivot_rest))

        # A descendant or body offset should move
        sample_local = n.joint_axis * 0.0 + np.array([0.05, 0.0, 0.0])
        if abs(float(np.dot(normalize(sample_local), n.joint_axis))) > 0.9:
            sample_local = np.array([0.0, 0.05, 0.0])
        # world sample = world_matrix @ sample_local
        W0 = mat4_from_rt(np.eye(3), pivot_rest)  # approx at rest without parent — use full
        # Better: capture parent world and child local
        parent = n.parent
        assert parent is not None
        # reset to measure rest sample
        n.rotation = np.eye(3)
        if n.rest_position is not None:
            n.position = n.rest_position.copy()
        Wp = parent.world_matrix()
        Mc0 = mat4_from_rt(np.eye(3), n.position)
        p0 = (Wp @ Mc0 @ np.array([*sample_local, 1.0]))[:3]

        set_joint(n, angle_rad)
        Mc1 = mat4_from_rt(n.rotation, n.position)
        p1 = (Wp @ Mc1 @ np.array([*sample_local, 1.0]))[:3]
        sample_delta = float(np.linalg.norm(p1 - p0))

        ok = True
        details = []
        if n.joint_type == "revolute":
            if pivot_drift > 1e-6:
                ok = False
                details.append(f"pivot_drift={pivot_drift:.3e}")
            if sample_delta < 1e-6:
                ok = False
                details.append(f"sample_static delta={sample_delta:.3e}")
        else:
            if sample_delta < 1e-6 and pivot_drift < 1e-6:
                ok = False
                details.append("prismatic_no_motion")

        if not ok:
            all_ok = False

        joint_results.append(
            {
                "id": n.joint_id,
                "link": n.id,
                "type": n.joint_type,
                "ok": ok,
                "pivot_drift_m": pivot_drift,
                "sample_delta_m": sample_delta,
                "details": details,
                "mesh": n.mesh,
            }
        )

    # Hierarchy integrity
    orphans = [lid for lid, n in nodes.items() if n.parent is None and lid != root.id]
    report = {
        "ok": all_ok and not orphans,
        "kinematics_ok": all_ok and not orphans,
        "meshes_ok": not mesh_issues,
        "n_links": len(nodes),
        "n_movable": len(movable),
        "base_link": root.id,
        "frame": robot.get("frame"),
        "mesh_missing": mesh_issues,
        "orphan_links": orphans,
        "joints": joint_results,
        "contract": "CadRobotLoader.set_joint / build_tree",
    }
    write_json(Path(out_dir) / "godot_runtime_report.json", report)
    return report
