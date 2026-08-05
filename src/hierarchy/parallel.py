"""Parallel / closed-chain loop detection (Faz 6 scaffolding)."""

from __future__ import annotations

from typing import Any

from common.models import ResolvedJoint


def detect_parallel_loops(
    joints: list[ResolvedJoint],
    part_to_link: dict[str, str],
) -> list[dict[str, Any]]:
    """
    Detect undirected cycles in the joint graph after mapping to links.
    Delta / parallel robots produce loops; serial trees do not.
    Returns loop descriptions for downstream closed-chain solvers (not solved here).
    """
    # Build undirected graph of links connected by joints
    adj: dict[str, list[str]] = {}
    for j in joints:
        a = part_to_link.get(j.parent, j.parent)
        b = part_to_link.get(j.child, j.child)
        if a == b:
            continue
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    for k in adj:
        adj[k] = sorted(set(adj[k]))

    loops: list[dict[str, Any]] = []
    visited: set[str] = set()

    def dfs(u: str, parent: str | None, path: list[str]) -> None:
        visited.add(u)
        path.append(u)
        for v in adj.get(u, []):
            if v == parent:
                continue
            if v in path:
                i = path.index(v)
                cycle = path[i:] + [v]
                if len(cycle) >= 4:  # at least 3 distinct nodes
                    loops.append({"nodes": cycle[:-1], "kind": "undirected_cycle"})
                continue
            if v not in visited:
                dfs(v, u, path)
        path.pop()

    for node in sorted(adj.keys()):
        if node not in visited:
            dfs(node, None, [])

    # Deduplicate cycles by frozenset
    uniq = []
    seen = set()
    for loop in loops:
        key = frozenset(loop["nodes"])
        if key not in seen:
            seen.add(key)
            uniq.append(loop)
    return uniq
