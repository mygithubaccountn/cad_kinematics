"""Pipeline phase gate. Bump only when the previous phase is complete and documented."""

# Roadmap phases: 0 infra → 1 geometry → 2 joints → 3 hierarchy/godot → 4 validation → 5 prismatic → 6 closed
CURRENT_PHASE = 6

PHASE_NAMES = {
    0: "infrastructure",
    1: "geometry",
    2: "joint_detection",
    3: "hierarchy_godot",
    4: "validation",
    5: "prismatic_scara",
    6: "closed_chain",
}


def require_phase(min_phase: int, feature: str) -> None:
    if CURRENT_PHASE < min_phase:
        raise RuntimeError(
            f"{feature} requires phase >= {min_phase} ({PHASE_NAMES.get(min_phase)}). "
            f"Current phase is {CURRENT_PHASE} ({PHASE_NAMES.get(CURRENT_PHASE)}). "
            "Complete and document the current phase before enabling the next."
        )
