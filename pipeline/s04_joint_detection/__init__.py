"""04_joint_detection — multi-hypothesis revolute / prismatic scoring."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import write_json
from pipeline.common.math3d import as_vec3, axis_distance, nearly_parallel, normalize, project_point_to_axis
from pipeline.common.models import (
    AssemblyIR,
    ConcentricCluster,
    ContactPair,
    CylFeature,
    CylKind,
    FeatureGraph,
    JointHypothesis,
    JointType,
    MateHint,
    MateKind,
)
from pipeline.common.tolerances import Tolerances
from pipeline.common.trace import DecisionTrace
from pipeline.s04_joint_detection.prismatic import score_prismatic_hypotheses


def run_joint_detection(
    ir: AssemblyIR,
    features: FeatureGraph,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    include_prismatic: bool = True,
) -> list[JointHypothesis]:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    hypotheses: list[JointHypothesis] = []
    hypotheses.extend(_generate_revolute(ir, features, tolerances))
    if include_prismatic:
        hypotheses.extend(score_prismatic_hypotheses(ir, features, tolerances))

    selected = select_joints(hypotheses, tolerances)
    write_json(out / "joint_hypotheses.json", {"all": [h.to_dict() for h in hypotheses]})
    write_json(out / "joints_selected.json", {"joints": [h.to_dict() for h in selected]})
    write_json(
        out / "decision_traces.json",
        {"traces": [h.trace.to_dict() for h in selected if h.trace]},
    )
    return selected


def _generate_revolute(
    ir: AssemblyIR,
    features: FeatureGraph,
    tol: Tolerances,
) -> list[JointHypothesis]:
    cyl_map = features.cyl_map()
    contact_map = {_pair_key(c.part_a, c.part_b): c for c in features.contacts}
    mate_map = _mate_index(ir.mate_hints)
    hyps: list[JointHypothesis] = []
    hid = 0

    for cluster in features.clusters:
        parts = sorted(set(cluster.part_ids))
        if len(parts) < 2:
            continue
        # All unordered pairs in cluster that share cylinders
        for i, pa in enumerate(parts):
            for pb in parts[i + 1 :]:
                hyp = _score_revolute_pair(
                    hid, pa, pb, cluster, cyl_map, contact_map, mate_map, ir, tol
                )
                hid += 1
                if hyp is not None:
                    hyps.append(hyp)

    # Also shaft-in-hole pairs even if clustering missed multi-part (shouldn't)
    # already covered via clusters.

    hyps.sort(key=lambda h: (-h.confidence, h.part_a, h.part_b, h.id))
    return hyps


def _score_revolute_pair(
    hid: int,
    pa: str,
    pb: str,
    cluster: ConcentricCluster,
    cyl_map: dict[str, CylFeature],
    contact_map: dict[tuple[str, str], ContactPair],
    mate_map: dict[tuple[str, str], list[MateHint]],
    ir: AssemblyIR,
    tol: Tolerances,
) -> Optional[JointHypothesis]:
    trace = DecisionTrace(subject=f"revolute:{pa}|{pb}|{cluster.id}")
    cyls = [cyl_map[cid] for cid in cluster.cyl_ids if cid in cyl_map]
    cyls_a = [c for c in cyls if c.part_id == pa]
    cyls_b = [c for c in cyls if c.part_id == pb]
    if not cyls_a or not cyls_b:
        return None

    # Shaft-in-hole (primary revolute signal)
    shaft_score, shaft_detail, pivot_hint = _shaft_in_hole(cyls_a, cyls_b, tol)
    if shaft_score > 0:
        # Full concentric bonus only when mechanical shaft-hole exists
        trace.add("concentric_shared", 0.42, f"cluster={cluster.id} n={len(cyls)}")
        trace.add("shaft_hole", shaft_score, shaft_detail)
    else:
        # Shared axis alone is weak for revolute (guides / rails may be prismatic)
        trace.add("concentric_shared", 0.12, f"cluster={cluster.id} weak_without_shaft_hole")
        trace.reject(f"no_shaft_hole ({shaft_detail})")

    # Contact ring
    contact = contact_map.get(_pair_key(pa, pb))
    if contact and contact.strength > 0:
        # Prefer contacts near axis
        cpt = as_vec3(contact.centroid)
        axis_p = as_vec3(cluster.axis_point)
        axis_d = as_vec3(cluster.axis_dir)
        radial = float(np.linalg.norm(cpt - project_point_to_axis(cpt, axis_p, axis_d)))
        ring = contact.strength * (1.0 / (1.0 + radial * 20.0))
        score = 0.18 * min(1.0, ring * 3.0)
        trace.add("contact_ring", score, f"strength={contact.strength:.3f} radial={radial:.4f}")
    else:
        trace.reject("no_contact")

    # Mate hints
    for m in mate_map.get(_pair_key(pa, pb), []):
        if m.kind in (MateKind.CONCENTRIC, MateKind.REVOLUTE):
            trace.add("mate_hint", 0.15 * m.confidence, m.kind.value)

    # Weak placement alignment (relative Z)
    part_map = ir.part_map()
    Ma = part_map[pa].placement_mat()
    Mb = part_map[pb].placement_mat()
    za = normalize(Ma[:3, 2])
    zb = normalize(Mb[:3, 2])
    axis = normalize(as_vec3(cluster.axis_dir))
    align = max(abs(float(np.dot(za, axis))), abs(float(np.dot(zb, axis))))
    if align > 0.95:
        trace.add("placement_align", 0.05, f"align={align:.3f}")

    # Name tokens disabled by default
    if tol.name_token_scoring:
        na, nb = part_map[pa].name.lower(), part_map[pb].name.lower()
        if any(t in na or t in nb for t in ("joint", "axis", "revolute")):
            trace.add("name_token", 0.02, "matched")

    confidence = float(np.clip(trace.total_score, 0.0, 1.0))
    if confidence < tol.min_joint_confidence * 0.5:
        # Keep weak hyps in "all" but mark
        pass

    pivot = pivot_hint if pivot_hint is not None else as_vec3(cluster.axis_point)
    if contact:
        # Blend with contact projection onto axis
        cproj = project_point_to_axis(as_vec3(contact.centroid), as_vec3(cluster.axis_point), axis)
        pivot = 0.6 * pivot + 0.4 * cproj

    hyp = JointHypothesis(
        id=f"jhyp_{hid:04d}",
        part_a=pa,
        part_b=pb,
        joint_type=JointType.REVOLUTE,
        axis_point=as_vec3(cluster.axis_point).tolist(),
        axis_dir=axis.tolist(),
        pivot=np.asarray(pivot, dtype=np.float64).tolist(),
        confidence=confidence,
        evidence=[e.to_dict() for e in trace.evidence],
        trace=trace,
        cluster_id=cluster.id,
    )
    return hyp


def _shaft_in_hole(
    cyls_a: list[CylFeature],
    cyls_b: list[CylFeature],
    tol: Tolerances,
) -> tuple[float, str, Optional[np.ndarray]]:
    best = 0.0
    detail = "none"
    pivot = None
    for ca in cyls_a:
        for cb in cyls_b:
            pairings = [
                (ca, cb),
            ]
            for outer, inner in ((ca, cb), (cb, ca)):
                if outer.kind == CylKind.INNER and inner.kind == CylKind.OUTER:
                    continue
                # Prefer OUTER inside INNER
                if outer.kind not in (CylKind.OUTER, CylKind.UNKNOWN):
                    continue
                if inner.kind not in (CylKind.INNER, CylKind.UNKNOWN):
                    continue
                if outer.kind == CylKind.UNKNOWN and inner.kind == CylKind.UNKNOWN:
                    # Radius: smaller = shaft candidate
                    if outer.radius >= inner.radius:
                        outer, inner = inner, outer
                gap = abs(inner.radius - outer.radius)
                if gap > tol.shaft_hole_radial_clearance_m:
                    continue
                if outer.radius > inner.radius + tol.radius_abs_eps_m:
                    continue
                # Axis already clustered
                overlap = min(outer.height, inner.height)
                if overlap < tol.shaft_hole_min_overlap_m:
                    continue
                score = 0.35 * (1.0 - gap / max(tol.shaft_hole_radial_clearance_m, 1e-9))
                if score > best:
                    best = score
                    detail = f"outer_r={outer.radius:.4f} inner_r={inner.radius:.4f} gap={gap:.4f}"
                    # Pivot: midpoint of axis segment overlap
                    p0 = as_vec3(outer.axis_point)
                    d = normalize(as_vec3(outer.axis_dir))
                    pivot = p0 + d * 0.0  # at feature origin; refined in axis stage
                    pivot = 0.5 * (as_vec3(outer.axis_point) + as_vec3(inner.axis_point))
                    pivot = project_point_to_axis(pivot, as_vec3(outer.axis_point), d)
    return best, detail, pivot


def select_joints(hypotheses: list[JointHypothesis], tol: Tolerances) -> list[JointHypothesis]:
    """Greedy max-weight matching: at most one moving joint per unordered part pair; prefer high confidence."""
    # Filter by type priority and confidence
    movable = [
        h
        for h in hypotheses
        if h.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC)
        and h.confidence >= tol.min_joint_confidence
    ]
    movable.sort(
        key=lambda h: (
            -h.confidence,
            0 if h.joint_type == JointType.REVOLUTE else 1,
            h.part_a,
            h.part_b,
            h.id,
        )
    )
    selected: list[JointHypothesis] = []
    used_pairs: set[tuple[str, str]] = set()
    # Soft: allow a part multiple joints (serial chain), but one joint per pair
    for h in movable:
        key = h.ordered_parts()
        if key in used_pairs:
            if h.trace:
                h.trace.reject("duplicate_pair_lower_score")
            continue
        used_pairs.add(key)
        selected.append(h)

    selected.sort(key=lambda h: (h.part_a, h.part_b, h.id))
    return selected


def _pair_key(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((a, b)))  # type: ignore[return-value]


def _mate_index(mates: list[MateHint]) -> dict[tuple[str, str], list[MateHint]]:
    idx: dict[tuple[str, str], list[MateHint]] = {}
    for m in mates:
        if not m.part_a or not m.part_b:
            continue
        idx.setdefault(_pair_key(m.part_a, m.part_b), []).append(m)
    return idx
