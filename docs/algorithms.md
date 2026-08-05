# Algorithms

## Concentric clustering

Union-find over cylindrical features. Merge when axes are parallel within `angle_eps` and line distance ≤ `axis_dist_eps`.

**Pros:** Direct mechanical signal. **Cons:** Filleted BRep may miss cylinders.

## Shaft-in-hole

Pair outer (shaft) with inner (hole) on a shared cluster; radius clearance and height overlap scored.

**Pros:** Strong revolute evidence + natural pivot. **Cons:** Welded assemblies without holes fail open.

## Contact sampling

Expanded AABB prune + seeded surface/bbox samples. Strength feeds adjacency and contact-ring score.

**Pros:** Works without STEP mates. **Cons:** O(n²); sampling noise.

## Mate hints

FreeCAD/STEP constraints boost confidence only; never required.

## Joint selection

Greedy max-confidence matching: one movable joint per unordered part pair. Revolute typically outranks prismatic when shaft-hole evidence exists.

## Pivot fusion (axis stage)

Weighted blend of cluster axis point, cylinder midpoints, and hypothesis pivot; all projected onto the robust axis. High variance lowers confidence.

**Forbidden as primary sources:** mesh origin, bbox center, Placement Base alone.

## Hierarchy

1. Choose base = max volume with lowest Z ground bonus  
2. Weld high-contact non-joint pairs into link components  
3. BFS spanning tree on joint edges from base  
4. Orphans fixed-welded to nearest adjacency  

## Parallel / Delta (Faz 6)

`detect_parallel_loops` flags undirected cycles. Closed-chain IK is not solved in MVP; validation reports `parallel_cycle`.

## Prismatic / SCARA (Faz 5)

Similar-radius parallel guide cylinders + contact → prismatic hypothesis (prior scaled below strong revolute).
