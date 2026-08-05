"""05_axis_detection — refine pivot and axis for selected joints."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import write_json
from pipeline.common.math3d import as_vec3, normalize, project_point_to_axis
from pipeline.common.models import (
    FeatureGraph,
    JointHypothesis,
    ResolvedJoint,
)
from pipeline.common.tolerances import Tolerances
from pipeline.common.trace import DecisionTrace


def run_axis_detection(
    hypotheses: list[JointHypothesis],
    features: FeatureGraph,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> list[ResolvedJoint]:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    cyl_map = features.cyl_map()
    cluster_map = {c.id: c for c in features.clusters}
    resolved: list[ResolvedJoint] = []

    for h in hypotheses:
        trace = DecisionTrace(subject=f"axis:{h.id}")
        if h.trace:
            for e in h.trace.evidence:
                trace.add(e.name, e.score, e.detail)

        axis = normalize(as_vec3(h.axis_dir))
        pivot_sources: list[tuple[np.ndarray, float]] = []

        cluster = cluster_map.get(h.cluster_id)
        if cluster:
            axis = normalize(as_vec3(cluster.axis_dir))
            pivot_sources.append((as_vec3(cluster.axis_point), 0.3))
            trace.note("axis_from_cluster_median")

        # Cylinder overlap midpoints for parts
        if cluster:
            cyls = [cyl_map[i] for i in cluster.cyl_ids if i in cyl_map]
            pts = []
            for c in cyls:
                if c.part_id in (h.part_a, h.part_b):
                    pts.append(as_vec3(c.axis_point))
            if pts:
                mid = np.mean(np.stack(pts), axis=0)
                mid = project_point_to_axis(mid, as_vec3(cluster.axis_point), axis)
                pivot_sources.append((mid, 0.5))
                trace.add("cyl_midpoint", 0.2, f"n={len(pts)}")

        # Hypothesis pivot
        pivot_sources.append((as_vec3(h.pivot), 0.4))

        # Weighted average
        weights = np.array([w for _, w in pivot_sources], dtype=np.float64)
        pts_a = np.stack([p for p, _ in pivot_sources])
        # Project all onto axis through first
        origin = pts_a[0]
        ts = np.array([float(np.dot(p - origin, axis)) for p in pts_a])
        t_star = float(np.average(ts, weights=weights))
        pivot = origin + axis * t_star

        variance = float(np.average((ts - t_star) ** 2, weights=weights))
        conf = float(h.confidence)
        if variance > (0.02**2):
            conf *= 0.85
            trace.note(f"high_pivot_variance={variance:.6f}")

        # Parent/child undecided here — hierarchy assigns direction; keep part_a/part_b order for now
        resolved.append(
            ResolvedJoint(
                id=h.id,
                parent=h.part_a,
                child=h.part_b,
                joint_type=h.joint_type,
                origin=pivot.tolist(),
                axis=axis.tolist(),
                confidence=conf,
                trace=trace,
            )
        )

    write_json(out / "resolved_axes.json", {"joints": [j.to_dict() for j in resolved]})
    return resolved
