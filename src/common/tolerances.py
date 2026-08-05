"""Central geometric tolerances (metres unless noted). Deterministic defaults."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Tolerances:
    # Axis / cylinder clustering
    angle_eps_deg: float = 2.0
    axis_dist_eps_m: float = 1.0e-3
    radius_rel_eps: float = 0.05
    radius_abs_eps_m: float = 5.0e-4

    # Contact / proximity
    contact_gap_m: float = 2.0e-3
    contact_sample_count: int = 64
    bbox_expand_m: float = 1.0e-3
    # Reject coaxial false joints between distant solids (metres)
    max_joint_aabb_gap_m: float = 0.04
    # Bonus when AABBs nearly touch (helps real mates without perfect shaft-hole)
    proximate_joint_bonus: float = 0.28

    # Shaft-in-hole
    shaft_hole_radial_clearance_m: float = 3.0e-3
    shaft_hole_min_overlap_m: float = 2.0e-3

    # Scoring
    min_joint_confidence: float = 0.35
    name_token_scoring: bool = False

    # Mesh — CAD preview (default) vs final export
    # Preview: coarse enough for first-run Godot CAD checks (not manufacturing LOD).
    # Relative deflection applied *before* tessellate (post-caps alone do not speed OCC).
    mesh_preview_linear_deflection_m: float = 3.0e-2  # 30 mm absolute floor
    mesh_preview_angular_deflection_rad: float = 1.0
    mesh_preview_relative: bool = True  # deflection as fraction of bbox diagonal
    mesh_preview_relative_deflection: float = 0.04  # 4% of bbox — fast recognizable CAD
    mesh_preview_max_tris_per_part: int = 40_000  # hard LOD cap after tessellate
    mesh_preview_max_tris_per_link: int = 120_000  # link merge budget (preview GLB)
    mesh_preview_min_volume_m3: float = 5.0e-7  # smaller → bbox placeholder
    mesh_linear_deflection_m: float = 3.0e-3  # 3 mm — final export
    mesh_angular_deflection_rad: float = 0.2

    # Cylinder extract caps (skip fastener noise; keeps joint-relevant shafts/holes)
    min_cylinder_radius_m: float = 2.0e-3
    min_cylinder_height_m: float = 3.0e-3
    max_cylinders_per_part: int = 48

    # Validation
    rest_pose_hausdorff_m: float = 5.0e-3
    smoke_angle_rad: float = 0.15
    pivot_bbox_margin_m: float = 5.0e-2

    # Part filtering
    min_part_volume_m3: float = 1.0e-8
    max_part_volume_m3: float = 10.0  # reject datum/infinite planes
    max_bbox_extent_m: float = 50.0  # reject unbounded construction geometry
    # Parts below this are welded to nearest neighbor (screws, tiny jaws) — not jointed
    min_link_volume_m3: float = 2.0e-4
    # Refuse fixed welds between parts whose centers are farther than this (metres)
    max_weld_distance_m: float = 0.12
    # Orphan attach: require evidence; merge into host link if strong
    orphan_merge_min_score: float = 0.55
    orphan_attach_min_score: float = 0.28
    orphan_max_aabb_gap_m: float = 0.05

    # RNG
    seed: int = 42

    # Unit: internal SI metres
    unit: str = "metre"

    extra: dict = field(default_factory=dict)

    @property
    def angle_eps_rad(self) -> float:
        import math

        return math.radians(self.angle_eps_deg)
