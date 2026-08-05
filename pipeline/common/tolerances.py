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

    # Shaft-in-hole
    shaft_hole_radial_clearance_m: float = 3.0e-3
    shaft_hole_min_overlap_m: float = 2.0e-3

    # Scoring
    min_joint_confidence: float = 0.35
    name_token_scoring: bool = False

    # Mesh
    mesh_linear_deflection_m: float = 5.0e-4
    mesh_angular_deflection_rad: float = 0.1

    # Validation
    rest_pose_hausdorff_m: float = 5.0e-3
    smoke_angle_rad: float = 0.15
    pivot_bbox_margin_m: float = 5.0e-2

    # Part filtering
    min_part_volume_m3: float = 1.0e-8
    max_part_volume_m3: float = 10.0  # reject datum/infinite planes
    max_bbox_extent_m: float = 50.0  # reject unbounded construction geometry

    # RNG
    seed: int = 42

    # Unit: internal SI metres
    unit: str = "metre"

    extra: dict = field(default_factory=dict)

    @property
    def angle_eps_rad(self) -> float:
        import math

        return math.radians(self.angle_eps_deg)
