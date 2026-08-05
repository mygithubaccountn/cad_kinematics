"""Phase 6 closed-chain loop detection scaffolding."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from common.models import JointType, ResolvedJoint
from hierarchy.parallel import detect_parallel_loops
from phase import CURRENT_PHASE


def test_phase_is_6():
    assert CURRENT_PHASE >= 6


def test_detect_triangle_loop():
    joints = [
        ResolvedJoint("j0", "a", "b", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
        ResolvedJoint("j1", "b", "c", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
        ResolvedJoint("j2", "c", "a", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
    ]
    loops = detect_parallel_loops(joints, {"a": "a", "b": "b", "c": "c"})
    assert len(loops) >= 1
    assert loops[0]["kind"] == "undirected_cycle"


def test_serial_has_no_loop():
    joints = [
        ResolvedJoint("j0", "a", "b", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
        ResolvedJoint("j1", "b", "c", JointType.REVOLUTE, [0, 0, 0], [0, 0, 1], 1.0),
    ]
    loops = detect_parallel_loops(joints, {"a": "a", "b": "b", "c": "c"})
    assert loops == []
