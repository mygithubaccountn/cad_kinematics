"""04_joint_detection — multi-hypothesis revolute / prismatic scoring."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from common.io_util import write_json
from common.math3d import as_vec3, axis_distance, bbox_aabb_gap, nearly_parallel, normalize, project_point_to_axis
from common.models import (
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
from common.tolerances import Tolerances
from common.trace import DecisionTrace
from joints.prismatic import score_prismatic_hypotheses


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

    early_rejects: list[DecisionTrace] = []
    hypotheses: list[JointHypothesis] = []
    hypotheses.extend(_generate_revolute(ir, features, tolerances, early_rejects))
    if include_prismatic:
        hypotheses.extend(score_prismatic_hypotheses(ir, features, tolerances))

    hypotheses = _demote_same_pair_losers(hypotheses)
    selected = select_joints(hypotheses, tolerances, ir=ir)
    selected_ids = {h.id for h in selected}

    # Notable rejects: early hard rejects + high-conf hyps that lost selection
    rejected_notable: list[dict] = [t.to_dict() for t in early_rejects]
    for h in hypotheses:
        if h.id in selected_ids:
            continue
        if h.confidence < tolerances.min_joint_confidence * 0.75:
            continue
        if h.trace is None:
            continue
        if not any("not_selected" in r for r in h.trace.rejected):
            reason = "not_selected"
            if any("would_create_cycle" in r for r in h.trace.rejected):
                reason = "would_create_cycle"
            elif h.confidence < tolerances.min_joint_confidence:
                reason = "below_min_confidence"
            else:
                h.trace.reject(reason)
        td = h.trace.to_dict()
        td["hypothesis_id"] = h.id
        td["confidence"] = h.confidence
        td["parts"] = [h.part_a, h.part_b]
        td["joint_type"] = h.joint_type.value
        rejected_notable.append(td)

    write_json(out / "joint_hypotheses.json", {"all": [h.to_dict() for h in hypotheses]})
    write_json(out / "joints_selected.json", {"joints": [h.to_dict() for h in selected]})
    write_json(
        out / "decision_traces.json",
        {
            "selected": [h.trace.to_dict() for h in selected if h.trace],
            "rejected_notable": rejected_notable,
            "traces": [h.trace.to_dict() for h in selected if h.trace] + rejected_notable,
        },
    )
    write_json(
        out / "joint_candidate_report.json",
        build_candidate_report(ir, features, hypotheses, selected, early_rejects, tolerances),
    )
    return selected


def build_candidate_report(
    ir: AssemblyIR,
    features: FeatureGraph,
    hypotheses: list[JointHypothesis],
    selected: list[JointHypothesis],
    early_rejects: list[DecisionTrace],
    tol: Tolerances,
) -> dict:
    """Explainable summary: who got candidates, why rejects, missing evidence."""
    selected_ids = {h.id for h in selected}
    cyl_count = {}
    for c in features.cylinders:
        cyl_count[c.part_id] = cyl_count.get(c.part_id, 0) + 1

    parts_in_hyps: set[str] = set()
    for h in hypotheses:
        parts_in_hyps.add(h.part_a)
        parts_in_hyps.add(h.part_b)

    parts_without: list[dict] = []
    for p in ir.parts:
        if p.id in parts_in_hyps:
            continue
        reasons = []
        n_cyl = cyl_count.get(p.id, 0)
        if n_cyl == 0:
            reasons.append("no_cylinders")
        if p.volume < tol.min_link_volume_m3:
            reasons.append(f"tiny_volume={p.volume:.3e}<min_link")
        in_multi = any(p.id in c.part_ids and len(set(c.part_ids)) >= 2 for c in features.clusters)
        if n_cyl > 0 and not in_multi:
            reasons.append("no_multi_part_cluster")
        if not reasons:
            reasons.append("no_hypothesis_generated")
        parts_without.append(
            {
                "part_id": p.id,
                "name": p.name,
                "volume": p.volume,
                "n_cylinders": n_cyl,
                "reasons": reasons,
            }
        )

    candidates = []
    for h in hypotheses:
        t = h.trace
        evidence = {e.name: {"score": e.score, "detail": e.detail} for e in (t.evidence if t else [])}
        missing = list(t.rejected) if t else []
        status = "selected" if h.id in selected_ids else "rejected"
        candidates.append(
            {
                "id": h.id,
                "status": status,
                "joint_type": h.joint_type.value,
                "parts": [h.part_a, h.part_b],
                "cluster_id": h.cluster_id,
                "confidence": h.confidence,
                "evidence": evidence,
                "missing_or_rejected": missing,
                "notes": list(t.notes) if t else [],
                "runner_up": t.runner_up if t else None,
            }
        )
    candidates.sort(key=lambda c: (-c["confidence"], c["id"]))

    return {
        "summary": {
            "n_parts": len(ir.parts),
            "n_hypotheses": len(hypotheses),
            "n_selected": len(selected),
            "n_early_rejects": len(early_rejects),
            "n_parts_without_candidates": len(parts_without),
        },
        "parts_without_candidates": parts_without,
        "candidates": candidates,
        "selected_ids": sorted(selected_ids),
        "early_rejects": [t.to_dict() for t in early_rejects],
    }

def _generate_revolute(
    ir: AssemblyIR,
    features: FeatureGraph,
    tol: Tolerances,
    early_rejects: list[DecisionTrace] | None = None,
) -> list[JointHypothesis]:
    cyl_map = features.cyl_map()
    contact_map = {_pair_key(c.part_a, c.part_b): c for c in features.contacts}
    mate_map = _mate_index(ir.mate_hints)
    adj_map = {_pair_key(e.part_a, e.part_b): e for e in features.adjacency}
    hyps: list[JointHypothesis] = []
    hid = 0
    sink = early_rejects if early_rejects is not None else []

    for cluster in features.clusters:
        parts = sorted(set(cluster.part_ids))
        if len(parts) < 2:
            continue
        for i, pa in enumerate(parts):
            for pb in parts[i + 1 :]:
                hyp = _score_revolute_pair(
                    hid,
                    pa,
                    pb,
                    cluster,
                    cyl_map,
                    contact_map,
                    mate_map,
                    adj_map,
                    ir,
                    tol,
                    sink,
                )
                hid += 1
                if hyp is not None:
                    hyps.append(hyp)

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
    adj_map: dict,
    ir: AssemblyIR,
    tol: Tolerances,
    early_rejects: list[DecisionTrace] | None = None,
) -> Optional[JointHypothesis]:
    trace = DecisionTrace(subject=f"revolute:{pa}|{pb}|{cluster.id}")
    cyls = [cyl_map[cid] for cid in cluster.cyl_ids if cid in cyl_map]
    cyls_a = [c for c in cyls if c.part_id == pa]
    cyls_b = [c for c in cyls if c.part_id == pb]
    if not cyls_a or not cyls_b:
        return None

    # Spatial proximity — reject long-range coaxial false positives
    part_map = ir.part_map()
    gap = bbox_aabb_gap(part_map[pa].bbox.as_array(), part_map[pb].bbox.as_array())
    if gap > tol.max_joint_aabb_gap_m:
        trace.reject(f"parts_too_far gap={gap:.4f}m")
        if early_rejects is not None:
            early_rejects.append(trace)
        return None
    if gap <= tol.contact_gap_m * 5.0:
        trace.add("bbox_proximate", tol.proximate_joint_bonus, f"aabb_gap={gap:.5f}")

    # Shaft-in-hole (primary) — then coaxial collar (outer/outer equal-r common in STEP)
    shaft_score, shaft_detail, pivot_hint = _shaft_in_hole(cyls_a, cyls_b, tol)
    coax_score, coax_detail, coax_pivot = _coaxial_mate(cyls_a, cyls_b, tol)
    if shaft_score > 0:
        trace.add("concentric_shared", 0.42, f"cluster={cluster.id} n={len(cyls)}")
        trace.add("shaft_hole", shaft_score, shaft_detail)
    elif coax_score > 0:
        # Weaker than true shaft-in-hole but stronger than bare shared axis
        trace.add("concentric_shared", 0.28, f"cluster={cluster.id} coaxial_collar")
        trace.add("coaxial_mate", coax_score, coax_detail)
        if pivot_hint is None:
            pivot_hint = coax_pivot
        trace.note("shaft_hole_missing_used_coaxial_mate")
    else:
        trace.add("concentric_shared", 0.12, f"cluster={cluster.id} weak_without_shaft_hole")
        trace.reject(f"no_shaft_hole ({shaft_detail})")
        if coax_detail and coax_detail != "none":
            trace.reject(f"no_coaxial_mate ({coax_detail})")
        else:
            trace.reject("no_coaxial_mate")

    # Contact ring
    contact = contact_map.get(_pair_key(pa, pb))
    if contact and contact.strength > 0:
        cpt = as_vec3(contact.centroid)
        axis_p = as_vec3(cluster.axis_point)
        axis_d = as_vec3(cluster.axis_dir)
        radial = float(np.linalg.norm(cpt - project_point_to_axis(cpt, axis_p, axis_d)))
        ring = contact.strength * (1.0 / (1.0 + radial * 20.0))
        score = 0.18 * min(1.0, ring * 3.0)
        trace.add("contact_ring", score, f"strength={contact.strength:.3f} radial={radial:.4f}")
    else:
        trace.reject("no_contact")

    # Shared-axis adjacency (often present when contact sampling is empty)
    adj = adj_map.get(_pair_key(pa, pb))
    if adj is not None:
        w = float(getattr(adj, "shared_axis_weight", 0.0) or 0.0)
        if w > 0.2:
            trace.add("adjacency_axis", min(0.12, 0.08 * w), f"shared_axis_weight={w:.2f}")
    else:
        trace.reject("no_adjacency")

    # Mate hints
    for m in mate_map.get(_pair_key(pa, pb), []):
        if m.kind in (MateKind.CONCENTRIC, MateKind.REVOLUTE):
            trace.add("mate_hint", 0.15 * m.confidence, m.kind.value)

    # Weak placement alignment (relative Z)
    Ma = part_map[pa].placement_mat()
    Mb = part_map[pb].placement_mat()
    za = normalize(Ma[:3, 2])
    zb = normalize(Mb[:3, 2])
    axis = normalize(as_vec3(cluster.axis_dir))
    align = max(abs(float(np.dot(za, axis))), abs(float(np.dot(zb, axis))))
    if align > 0.95:
        trace.add("placement_align", 0.05, f"align={align:.3f}")

    if tol.name_token_scoring:
        na, nb = part_map[pa].name.lower(), part_map[pb].name.lower()
        if any(t in na or t in nb for t in ("joint", "axis", "revolute")):
            trace.add("name_token", 0.02, "matched")

    confidence = float(np.clip(trace.total_score, 0.0, 1.0))

    pivot = pivot_hint if pivot_hint is not None else as_vec3(cluster.axis_point)
    if contact:
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
            for outer, inner in ((ca, cb), (cb, ca)):
                if outer.kind == CylKind.INNER and inner.kind == CylKind.OUTER:
                    continue
                # Prefer OUTER inside INNER
                if outer.kind not in (CylKind.OUTER, CylKind.UNKNOWN):
                    continue
                if inner.kind not in (CylKind.INNER, CylKind.UNKNOWN):
                    continue
                if outer.kind == CylKind.UNKNOWN and inner.kind == CylKind.UNKNOWN:
                    if outer.radius >= inner.radius:
                        outer, inner = inner, outer
                gap = abs(inner.radius - outer.radius)
                if gap > tol.shaft_hole_radial_clearance_m:
                    continue
                if outer.radius > inner.radius + tol.radius_abs_eps_m:
                    continue
                overlap = min(outer.height, inner.height)
                if overlap < tol.shaft_hole_min_overlap_m:
                    continue
                score = 0.35 * (1.0 - gap / max(tol.shaft_hole_radial_clearance_m, 1e-9))
                if score > best:
                    best = score
                    detail = f"outer_r={outer.radius:.4f} inner_r={inner.radius:.4f} gap={gap:.4f}"
                    d = normalize(as_vec3(outer.axis_dir))
                    pivot = 0.5 * (as_vec3(outer.axis_point) + as_vec3(inner.axis_point))
                    pivot = project_point_to_axis(pivot, as_vec3(outer.axis_point), d)
    return best, detail, pivot


def _coaxial_mate(
    cyls_a: list[CylFeature],
    cyls_b: list[CylFeature],
    tol: Tolerances,
) -> tuple[float, str, Optional[np.ndarray]]:
    """
    STEP often classifies mating bores/bosses as OUTER/OUTER with nearly equal radius.
    Treat as weaker revolute evidence than true shaft-in-hole.

    Skip long/long equal-radius tubes (typical prismatic guides / SCARA Z).
    Prefer a short collar against a longer mate.
    """
    best = 0.0
    detail = "none"
    pivot = None
    for ca in cyls_a:
        for cb in cyls_b:
            kinds = {ca.kind, cb.kind}
            if CylKind.INNER in kinds and CylKind.OUTER in kinds:
                continue
            if ca.kind not in (CylKind.OUTER, CylKind.UNKNOWN):
                continue
            if cb.kind not in (CylKind.OUTER, CylKind.UNKNOWN):
                continue
            gap = abs(ca.radius - cb.radius)
            if gap > tol.shaft_hole_radial_clearance_m:
                continue
            overlap = min(ca.height, cb.height)
            if overlap < tol.shaft_hole_min_overlap_m:
                continue
            h_lo, h_hi = min(ca.height, cb.height), max(ca.height, cb.height)
            # Long parallel tubes → prismatic territory, not revolute collar
            if h_hi >= 0.04 and (h_lo / max(h_hi, 1e-9)) >= 0.35:
                continue
            score = 0.28 * (1.0 - gap / max(tol.shaft_hole_radial_clearance_m, 1e-9))
            score *= float(np.clip(overlap / 0.02, 0.5, 1.0))
            if score > best:
                best = score
                detail = (
                    f"r_a={ca.radius:.4f} r_b={cb.radius:.4f} gap={gap:.4f} "
                    f"overlap={overlap:.4f} kinds={ca.kind.value}/{cb.kind.value}"
                )
                d = normalize(as_vec3(ca.axis_dir))
                pivot = 0.5 * (as_vec3(ca.axis_point) + as_vec3(cb.axis_point))
                pivot = project_point_to_axis(pivot, as_vec3(ca.axis_point), d)
    return best, detail, pivot


def _demote_same_pair_losers(hypotheses: list[JointHypothesis]) -> list[JointHypothesis]:
    """
    Same part-pair may get both revolute and prismatic hyps.
    Demote the one without type-specific evidence so selection keeps the right DOF
    (e.g. SCARA Z prismatic vs weak revolute from shared axis alone).
    """
    by_pair: dict[tuple[str, str], list[JointHypothesis]] = {}
    for h in hypotheses:
        by_pair.setdefault(h.ordered_parts(), []).append(h)

    for _key, group in by_pair.items():
        if len(group) < 2:
            continue

        def strength(h: JointHypothesis) -> tuple[int, float]:
            names = {e.name for e in (h.trace.evidence if h.trace else [])}
            if h.joint_type == JointType.PRISMATIC and "parallel_guides" in names:
                return (2, h.confidence)
            if h.joint_type == JointType.REVOLUTE and (
                "shaft_hole" in names or "coaxial_mate" in names
            ):
                return (2, h.confidence)
            if h.joint_type == JointType.REVOLUTE:
                return (1, h.confidence)
            return (0, h.confidence)

        group_sorted = sorted(group, key=lambda h: (-strength(h)[0], -h.confidence, h.id))
        winner = group_sorted[0]
        for loser in group_sorted[1:]:
            if strength(winner)[0] > strength(loser)[0]:
                if loser.trace:
                    loser.trace.reject(
                        f"same_pair_prefer_{winner.joint_type.value}:{winner.id}"
                    )
                    loser.trace.note(
                        f"demoted_for_selection conf_was={loser.confidence:.3f}"
                    )
                loser.confidence = 0.0

    return hypotheses


def select_joints(
    hypotheses: list[JointHypothesis],
    tol: Tolerances,
    ir: AssemblyIR | None = None,
    base_part: str | None = None,
) -> list[JointHypothesis]:
    """
    Prefer a serial kinematic chain from the base (industrial arms).

    Grow from either path endpoint (degree-1), not only the distal tip — so when
    the first pick is base→shaft, base→arm can still attach on the other end
    without mid-chain Kruskal forks.
    """
    from collections import Counter

    volumes = {p.id: p.volume for p in ir.parts} if ir else {}

    def ok_volume(pid: str, conf: float) -> bool:
        if not volumes:
            return True
        v = volumes.get(pid, 0.0)
        if v >= tol.min_link_volume_m3:
            return True
        return conf >= 0.65 and v >= 0.25 * tol.min_link_volume_m3

    movable = [
        h
        for h in hypotheses
        if h.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC)
        and h.confidence >= tol.min_joint_confidence
        and ok_volume(h.part_a, h.confidence)
        and ok_volume(h.part_b, h.confidence)
    ]

    if base_part is None and ir is not None:
        from hierarchy import choose_base
        from common.models import FeatureGraph

        base_part = choose_base(ir, FeatureGraph(), joints=None)

    selected: list[JointHypothesis] = []
    used_pairs: set[tuple[str, str]] = set()
    parent_uf: dict[str, str] = {}

    def find(a: str) -> str:
        parent_uf.setdefault(a, a)
        while parent_uf[a] != a:
            parent_uf[a] = parent_uf[parent_uf[a]]
            a = parent_uf[a]
        return a

    def union(a: str, b: str) -> bool:
        ra, rb = find(a), find(b)
        if ra == rb:
            return False
        parent_uf[max(ra, rb)] = min(ra, rb)
        return True

    def try_add(h: JointHypothesis) -> bool:
        key = h.ordered_parts()
        if key in used_pairs:
            return False
        if not union(h.part_a, h.part_b):
            if h.trace:
                h.trace.reject("would_create_cycle")
            return False
        used_pairs.add(key)
        selected.append(h)
        return True

    by_part: dict[str, list[JointHypothesis]] = {}
    for h in movable:
        by_part.setdefault(h.part_a, []).append(h)
        by_part.setdefault(h.part_b, []).append(h)

    def _record_runner_up(h: JointHypothesis, cands: list, endpoint: str, other: str) -> None:
        if len(cands) < 2 or h.trace is None:
            return
        _c2, other2, h2 = cands[1]
        h.trace.set_runner_up(
            name=f"joint:{h2.id}",
            origin=list(h2.pivot),
            axis=list(h2.axis_dir),
            score=float(h2.confidence),
            detail=f"alt_chain {endpoint}->{other2} conf={h2.confidence:.3f}",
        )
        h.trace.note(f"chain_pick endpoint={endpoint} -> {other} over {len(cands)-1} alts")

    # --- Phase 1: grow serial chain from base (one direction) ---
    if base_part:
        tip = base_part
        used_parts = {base_part}
        for _ in range(len(volumes) + 2):
            cands = []
            for h in by_part.get(tip, []):
                other = h.part_b if h.part_a == tip else h.part_a
                if other in used_parts:
                    continue
                if h.ordered_parts() in used_pairs:
                    continue
                cands.append((h.confidence, other, h))
            if not cands:
                break
            cands.sort(key=lambda t: (-t[0], t[1], t[2].id))
            _conf, other, h = cands[0]
            _record_runner_up(h, cands, tip, other)
            if try_add(h):
                tip = other
                used_parts.add(other)

    # --- Phase 2: extend from ANY degree-1 endpoint (keeps a path, allows back-growth) ---
    if base_part and selected:
        for _ in range(len(volumes) + 2):
            deg: Counter[str] = Counter()
            for h in selected:
                deg[h.part_a] += 1
                deg[h.part_b] += 1
            used_parts = set(deg.keys())
            endpoints = [p for p, d in deg.items() if d == 1]
            if not endpoints:
                break

            best_pick = None  # (conf, endpoint, other, hyp, cands)
            for ep in endpoints:
                cands = []
                for h in by_part.get(ep, []):
                    other = h.part_b if h.part_a == ep else h.part_a
                    if other in used_parts:
                        continue
                    cands.append((h.confidence, other, h))
                if not cands:
                    continue
                cands.sort(key=lambda t: (-t[0], t[1], t[2].id))
                conf, other, h = cands[0]
                cand = (conf, ep, other, h, cands)
                if best_pick is None or cand[0] > best_pick[0] + 1e-12 or (
                    abs(cand[0] - best_pick[0]) <= 1e-12 and (cand[2], cand[3].id) < (best_pick[2], best_pick[3].id)
                ):
                    best_pick = cand

            if best_pick is None:
                break
            conf, ep, other, h, cands = best_pick
            _record_runner_up(h, cands, ep, other)
            if h.trace:
                h.trace.note(f"endpoint_extend from={ep} (path endpoints={endpoints})")
            if not try_add(h):
                break

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
