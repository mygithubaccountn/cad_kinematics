"""S3 axis/pivot consensus — multi-candidate resolve with DecisionTrace."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from common.io_util import write_json
from common.math3d import as_vec3, nearly_parallel, normalize, project_point_to_axis
from common.models import FeatureGraph, JointHypothesis, ResolvedJoint
from common.tolerances import Tolerances
from common.trace import DecisionTrace


@dataclass
class _Cand:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    prior: float
    detail: str = ""


def run_axis_detection(
    hypotheses: list[JointHypothesis],
    features: FeatureGraph,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> list[ResolvedJoint]:
    """
    For each selected joint hypothesis, build axis/pivot candidates from
    independent evidence, score agreement (consensus), pick winner + runner-up,
    and record a full DecisionTrace.
    """
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    resolved: list[ResolvedJoint] = []
    for h in hypotheses:
        rj = resolve_joint_axis(h, features, tolerances)
        resolved.append(rj)

    write_json(
        out / "resolved_axes.json",
        {
            "algorithm": "axis_consensus-1",
            "joints": [j.to_dict() for j in resolved],
        },
    )
    return resolved


def resolve_joint_axis(
    h: JointHypothesis,
    features: FeatureGraph,
    tol: Tolerances,
) -> ResolvedJoint:
    trace = DecisionTrace(subject=f"axis:{h.id}")
    if h.trace:
        for e in h.trace.evidence:
            trace.add(e.name, e.score, e.detail)
        for r in h.trace.rejected:
            # Keep selection-stage rejects as notes so pivot stage rejected[] stays clean
            trace.note(f"hyp_reject:{r}")

    cands = _build_candidates(h, features, tol, trace)
    if not cands:
        # Absolute fallback
        axis = normalize(as_vec3(h.axis_dir))
        origin = as_vec3(h.pivot)
        conf = float(h.confidence) * 0.5
        trace.reject("no_axis_candidates")
        trace.set_chosen(
            origin=origin.tolist(),
            axis=axis.tolist(),
            method="hypothesis_fallback",
            confidence=conf,
            detail="empty candidate set",
        )
        return ResolvedJoint(
            id=h.id,
            parent=h.part_a,
            child=h.part_b,
            joint_type=h.joint_type,
            origin=origin.tolist(),
            axis=axis.tolist(),
            confidence=conf,
            trace=trace,
        )

    ranked = _rank_candidates(cands, tol)
    best = ranked[0]
    method = best.name
    origin = best.origin.copy()
    axis = best.axis.copy()
    conf = float(h.confidence)

    # Consensus blend if runner-up agrees closely
    if len(ranked) >= 2:
        second = ranked[1]
        agree = _candidate_agreement(best, second, tol)
        trace.add(
            "runner_up_agreement",
            agree,
            f"{best.name} vs {second.name}",
        )
        trace.set_runner_up(
            name=second.name,
            origin=second.origin.tolist(),
            axis=second.axis.tolist(),
            score=float(second.prior + agree),
            detail=second.detail,
        )
        if agree >= 0.75:
            # Blend pivots along best axis; average axes if parallel
            origin = project_point_to_axis(
                0.5 * (best.origin + second.origin), best.origin, best.axis
            )
            if nearly_parallel(best.axis, second.axis, tol.angle_eps_rad):
                axis = normalize(best.axis + second.axis * np.sign(float(np.dot(best.axis, second.axis))))
            method = f"consensus:{best.name}+{second.name}"
            conf = min(1.0, conf * (0.92 + 0.08 * agree))
            trace.note(f"blended_with_runner_up agree={agree:.3f}")
        else:
            conf = min(1.0, conf * (0.85 + 0.1 * agree))
            trace.note(f"kept_top_candidate; runner_up_disagree agree={agree:.3f}")

        for loser in ranked[2:]:
            d = _pivot_axis_delta(best, loser)
            trace.reject(
                f"cand:{loser.name} score={loser.prior:.2f} "
                f"axial_delta={d['axial']:.4f}m angle_deg={d['angle_deg']:.2f}"
            )
    else:
        trace.note("single_candidate")

    # Spread among all candidates → confidence penalty
    if len(ranked) >= 2:
        spreads = [_pivot_axis_delta(best, c)["axial"] for c in ranked[1:]]
        spread = float(np.median(spreads))
        if spread > tol.axis_dist_eps_m * 8.0:
            conf *= 0.85
            trace.note(f"high_candidate_spread axial_med={spread:.5f}")
            trace.add("spread_penalty", -0.1, f"axial_med={spread:.5f}")

    for i, c in enumerate(ranked):
        trace.add(
            f"cand_{c.name}",
            float(c.prior),
            f"rank={i+1} {c.detail}",
        )

    trace.set_chosen(
        origin=origin.tolist(),
        axis=axis.tolist(),
        method=method,
        confidence=float(conf),
        detail=best.detail,
    )

    return ResolvedJoint(
        id=h.id,
        parent=h.part_a,
        child=h.part_b,
        joint_type=h.joint_type,
        origin=origin.tolist(),
        axis=axis.tolist(),
        confidence=float(conf),
        trace=trace,
    )


def _build_candidates(
    h: JointHypothesis,
    features: FeatureGraph,
    tol: Tolerances,
    trace: DecisionTrace,
) -> list[_Cand]:
    cyl_map = features.cyl_map()
    cluster_map = {c.id: c for c in features.clusters}
    contact_map = {_pair_key(c.part_a, c.part_b): c for c in features.contacts}
    cands: list[_Cand] = []

    hyp_axis = normalize(as_vec3(h.axis_dir))
    hyp_pivot = as_vec3(h.pivot)

    # 1) Hypothesis (scoring-stage blend)
    cands.append(
        _Cand(
            name="hypothesis",
            origin=hyp_pivot,
            axis=hyp_axis,
            prior=0.35,
            detail="from joint scoring pivot/axis",
        )
    )

    cluster = cluster_map.get(h.cluster_id) if h.cluster_id else None
    if cluster:
        c_axis = normalize(as_vec3(cluster.axis_dir))
        c_pt = as_vec3(cluster.axis_point)
        cands.append(
            _Cand(
                name="cluster_median",
                origin=c_pt,
                axis=c_axis,
                prior=0.45,
                detail=f"cluster={cluster.id}",
            )
        )

        cyls = [cyl_map[i] for i in cluster.cyl_ids if i in cyl_map]
        pts_ab = [
            as_vec3(c.axis_point)
            for c in cyls
            if c.part_id in (h.part_a, h.part_b)
        ]
        dirs_ab = [
            normalize(as_vec3(c.axis_dir))
            for c in cyls
            if c.part_id in (h.part_a, h.part_b)
        ]
        if pts_ab:
            mid = np.mean(np.stack(pts_ab), axis=0)
            mid = project_point_to_axis(mid, c_pt, c_axis)
            axis_mean = c_axis
            if dirs_ab:
                # Align signs to cluster axis before averaging
                signed = []
                for d in dirs_ab:
                    signed.append(d if float(np.dot(d, c_axis)) >= 0 else -d)
                axis_mean = normalize(np.mean(np.stack(signed), axis=0))
            cands.append(
                _Cand(
                    name="cyl_overlap_mid",
                    origin=mid,
                    axis=axis_mean,
                    prior=0.55,
                    detail=f"n_cyl={len(pts_ab)}",
                )
            )

        # Shaft/hole pair midpoints only
        shaft_pts = []
        for ca in cyls:
            if ca.part_id != h.part_a:
                continue
            for cb in cyls:
                if cb.part_id != h.part_b:
                    continue
                if _looks_shaft_hole(ca, cb, tol) or _looks_shaft_hole(cb, ca, tol):
                    shaft_pts.append(0.5 * (as_vec3(ca.axis_point) + as_vec3(cb.axis_point)))
        if shaft_pts:
            sp = np.mean(np.stack(shaft_pts), axis=0)
            sp = project_point_to_axis(sp, c_pt, c_axis)
            cands.append(
                _Cand(
                    name="shaft_hole_mid",
                    origin=sp,
                    axis=c_axis,
                    prior=0.65,
                    detail=f"pairs={len(shaft_pts)}",
                )
            )
    else:
        trace.note("no_cluster_on_hypothesis")

    contact = contact_map.get(_pair_key(h.part_a, h.part_b))
    if contact and contact.strength > 0:
        axis_ref = cands[0].axis
        origin_ref = cands[0].origin
        if cluster:
            axis_ref = normalize(as_vec3(cluster.axis_dir))
            origin_ref = as_vec3(cluster.axis_point)
        cpt = project_point_to_axis(as_vec3(contact.centroid), origin_ref, axis_ref)
        cands.append(
            _Cand(
                name="contact_on_axis",
                origin=cpt,
                axis=axis_ref,
                prior=0.40 * min(1.0, contact.strength * 2.0),
                detail=f"strength={contact.strength:.3f}",
            )
        )
    else:
        trace.note("no_contact_for_pivot")

    return cands


def _rank_candidates(cands: list[_Cand], tol: Tolerances) -> list[_Cand]:
    """Score each candidate by prior + agreement with peers; return best-first."""
    if len(cands) == 1:
        return list(cands)

    scored: list[tuple[float, _Cand]] = []
    for i, c in enumerate(cands):
        agrees = []
        for j, o in enumerate(cands):
            if i == j:
                continue
            agrees.append(_candidate_agreement(c, o, tol))
        agree = float(np.mean(agrees)) if agrees else 0.0
        # Final rank score
        score = float(c.prior) + 0.35 * agree
        # mutate prior to carry rank score for reject messages
        scored.append((score, _Cand(c.name, c.origin, c.axis, score, c.detail)))

    scored.sort(key=lambda t: (-t[0], t[1].name))
    return [c for _, c in scored]


def _candidate_agreement(a: _Cand, b: _Cand, tol: Tolerances) -> float:
    """1.0 = same axis direction and nearly same pivot along axis."""
    # Axis parallel score
    dot = abs(float(np.dot(normalize(a.axis), normalize(b.axis))))
    ang = float(np.degrees(np.arccos(np.clip(dot, 0.0, 1.0))))
    axis_score = float(np.clip(1.0 - ang / max(tol.angle_eps_deg * 4.0, 1e-6), 0.0, 1.0))

    # Axial separation of pivots measured along a's axis
    delta = _pivot_axis_delta(a, b)["axial"]
    pivot_score = float(
        np.clip(1.0 - delta / max(tol.axis_dist_eps_m * 20.0, 1e-6), 0.0, 1.0)
    )
    return 0.55 * axis_score + 0.45 * pivot_score


def _pivot_axis_delta(a: _Cand, b: _Cand) -> dict[str, float]:
    ax = normalize(a.axis)
    # Distance between lines approx: axial + radial of b.origin relative to a
    rel = b.origin - a.origin
    axial = abs(float(np.dot(rel, ax)))
    radial = float(np.linalg.norm(rel - ax * float(np.dot(rel, ax))))
    dot = abs(float(np.dot(ax, normalize(b.axis))))
    angle_deg = float(np.degrees(np.arccos(np.clip(dot, 0.0, 1.0))))
    return {"axial": axial, "radial": radial, "angle_deg": angle_deg}


def _looks_shaft_hole(outer, inner, tol: Tolerances) -> bool:
    from common.models import CylKind

    if outer.kind == CylKind.INNER and inner.kind == CylKind.OUTER:
        return False
    if outer.radius > inner.radius + tol.radius_abs_eps_m:
        return False
    gap = abs(inner.radius - outer.radius)
    if gap > tol.shaft_hole_radial_clearance_m:
        return False
    overlap = min(outer.height, inner.height)
    return overlap >= tol.shaft_hole_min_overlap_m


def _pair_key(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((a, b)))  # type: ignore[return-value]
