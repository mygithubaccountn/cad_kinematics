"""Unit tests for math, clustering, joint scoring."""

from __future__ import annotations

import numpy as np

from pipeline.common.math3d import axis_distance, nearly_parallel, normalize, project_point_to_axis
from pipeline.common.tolerances import Tolerances
from pipeline.s01_import.synthetic import build_serial_3dof_fixture
from pipeline.s02_geometry import cluster_concentric, estimate_contacts, build_adjacency
from pipeline.s04_joint_detection import run_joint_detection
from pipeline.s05_axis_detection import run_axis_detection
from pipeline.s06_hierarchy import run_hierarchy, choose_base
from pipeline.s06_hierarchy.parallel import detect_parallel_loops
from pipeline.common.models import ResolvedJoint, JointType


def test_nearly_parallel():
    tol = Tolerances()
    assert nearly_parallel(np.array([0, 0, 1.0]), np.array([0, 0, -1.0]), tol.angle_eps_rad)
    assert not nearly_parallel(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), tol.angle_eps_rad)


def test_axis_distance_parallel():
    d = axis_distance(
        np.array([0.0, 0, 0]),
        np.array([0.0, 0, 1]),
        np.array([0.01, 0, 0]),
        np.array([0.0, 0, 1]),
    )
    assert abs(d - 0.01) < 1e-9


def test_serial_fixture_clusters():
    ir, fg = build_serial_3dof_fixture()
    tol = Tolerances()
    clusters = cluster_concentric(fg.cylinders, tol)
    # At least 3 multi-part joint clusters (+ maybe deco alone)
    multi = [c for c in clusters if len(c.part_ids) >= 2]
    assert len(multi) >= 3


def test_joint_detection_finds_three_revolute(tmp_path):
    ir, fg = build_serial_3dof_fixture()
    fg.clusters = cluster_concentric(fg.cylinders, Tolerances())
    fg.contacts = estimate_contacts(ir, Tolerances())
    fg.adjacency = build_adjacency(fg.contacts, fg.clusters, fg.cylinders)
    selected = run_joint_detection(ir, fg, tmp_path, include_prismatic=False)
    revolute = [j for j in selected if j.joint_type == JointType.REVOLUTE]
    assert len(revolute) >= 3
    pairs = {j.ordered_parts() for j in revolute}
    assert ("base", "link1") in pairs
    assert ("link1", "link2") in pairs
    assert ("link2", "link3") in pairs


def test_hierarchy_base_and_chain(tmp_path):
    ir, fg = build_serial_3dof_fixture()
    tol = Tolerances()
    fg.clusters = cluster_concentric(fg.cylinders, tol)
    fg.contacts = estimate_contacts(ir, tol)
    fg.adjacency = build_adjacency(fg.contacts, fg.clusters, fg.cylinders)
    selected = run_joint_detection(ir, fg, tmp_path / "j", include_prismatic=False)
    resolved = run_axis_detection(selected, fg, tmp_path / "a", tol)
    tree = run_hierarchy(ir, fg, resolved, tmp_path / "h", tol)
    assert choose_base(ir, fg) == "base"
    assert tree.base_link.startswith("link_")
    movable = [j for j in tree.joints if j.joint_type == JointType.REVOLUTE]
    assert len(movable) >= 3


def test_parallel_loop_detection():
    joints = [
        ResolvedJoint("j0", "a", "b", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
        ResolvedJoint("j1", "b", "c", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
        ResolvedJoint("j2", "c", "a", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
    ]
    part_to_link = {"a": "a", "b": "b", "c": "c"}
    loops = detect_parallel_loops(joints, part_to_link)
    assert len(loops) >= 1
