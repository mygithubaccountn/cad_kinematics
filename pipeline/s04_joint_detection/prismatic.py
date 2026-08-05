"""Prismatic joint hypotheses (Faz 5)."""

from __future__ import annotations

from typing import Optional

import numpy as np

from pipeline.common.math3d import as_vec3, nearly_parallel, normalize
from pipeline.common.models import (
    AssemblyIR,
    FeatureGraph,
    JointHypothesis,
    JointType,
    PlaneFeature,
)
from pipeline.common.tolerances import Tolerances
from pipeline.common.trace import DecisionTrace


def score_prismatic_hypotheses(
    ir: AssemblyIR,
    features: FeatureGraph,
    tol: Tolerances,
) -> list[JointHypothesis]:
    """
    Prismatic candidates: shared axis cluster + sliding evidence.
    Signals: parallel outer cylinders with similar radius (guide rails),
    or contact centroid elongated along axis with weak rotational shaft-hole.
    """
    hyps: list[JointHypothesis] = []
    hid = 0
    cyl_map = features.cyl_map()
    contact_map = {
        tuple(sorted((c.part_a, c.part_b))): c for c in features.contacts
    }

    for cluster in features.clusters:
        parts = sorted(set(cluster.part_ids))
        if len(parts) < 2:
            continue
        for i, pa in enumerate(parts):
            for pb in parts[i + 1 :]:
                cyls = [cyl_map[cid] for cid in cluster.cyl_ids if cid in cyl_map]
                ca = [c for c in cyls if c.part_id == pa]
                cb = [c for c in cyls if c.part_id == pb]
                if not ca or not cb:
                    continue
                trace = DecisionTrace(subject=f"prismatic:{pa}|{pb}|{cluster.id}")
                # Similar-radius parallel guides (prefer both OUTER — rails/sliders)
                guide = 0.0
                both_outer = False
                for a in ca:
                    for b in cb:
                        rad_ok = abs(a.radius - b.radius) <= max(
                            tol.radius_abs_eps_m, tol.radius_rel_eps * max(a.radius, b.radius)
                        )
                        if not rad_ok:
                            continue
                        from pipeline.common.models import CylKind

                        if a.kind == CylKind.OUTER and b.kind == CylKind.OUTER:
                            guide = max(guide, 0.45)
                            both_outer = True
                        else:
                            guide = max(guide, 0.20)
                if guide > 0:
                    trace.add(
                        "parallel_guides",
                        guide,
                        "both_outer" if both_outer else "similar_radius",
                    )
                contact = contact_map.get(tuple(sorted((pa, pb))))
                if contact and contact.strength > 0.05:
                    trace.add("contact", 0.15 * min(1.0, contact.strength * 2), "")
                if guide <= 0:
                    continue
                # Skip prismatic if a clear shaft-in-hole exists on this pair
                has_inner = any(c.kind.value == "inner" for c in ca + cb)
                has_outer = any(c.kind.value == "outer" for c in ca + cb)
                if has_inner and has_outer and not both_outer:
                    continue
                conf = float(np.clip(trace.total_score, 0.0, 1.0))
                axis = normalize(as_vec3(cluster.axis_dir))
                pivot = as_vec3(cluster.axis_point)
                hyps.append(
                    JointHypothesis(
                        id=f"phyp_{hid:04d}",
                        part_a=pa,
                        part_b=pb,
                        joint_type=JointType.PRISMATIC,
                        axis_point=pivot.tolist(),
                        axis_dir=axis.tolist(),
                        pivot=pivot.tolist(),
                        confidence=conf,
                        evidence=[e.to_dict() for e in trace.evidence],
                        trace=trace,
                        cluster_id=cluster.id,
                    )
                )
                hid += 1
    return hyps
