"""Export visual debug overlay (pivots, axes, link colors) for viewer / Godot."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.io_util import write_json
from common.math3d import as_vec3, mat4_from_list, mat4_identity, mat4_from_rt, normalize, transform_point
from common.models import JointType, KinematicTree, RobotDesc

# Distinct, readable link colors (sRGB 0–1)
_LINK_PALETTE = [
    (0.91, 0.30, 0.24),  # red
    (0.20, 0.60, 0.86),  # blue
    (0.30, 0.78, 0.45),  # green
    (0.95, 0.61, 0.07),  # orange
    (0.61, 0.35, 0.71),  # purple
    (0.20, 0.76, 0.78),  # teal
    (0.90, 0.49, 0.13),  # dark orange
    (0.55, 0.71, 0.20),  # olive
]


def write_debug_overlay(
    tree: KinematicTree,
    desc: Optional[RobotDesc],
    out_dir: Path,
    axis_length_m: float = 0.08,
) -> dict[str, Any]:
    """
    Write debug_overlay.json with:
      - per-link colors
      - joint pivot markers (Godot/robot.json frame when desc present)
      - axis segments
    """
    out = Path(out_dir)
    links = sorted(tree.links, key=lambda l: l.id)
    link_colors = {
        l.id: {
            "rgb": list(_LINK_PALETTE[i % len(_LINK_PALETTE)]),
            "hex": _rgb_hex(_LINK_PALETTE[i % len(_LINK_PALETTE)]),
        }
        for i, l in enumerate(links)
    }

    markers: list[dict[str, Any]] = []
    axes: list[dict[str, Any]] = []

    if desc is not None and desc.joints:
        world = _godot_rest_world(desc)
        for j in desc.joints:
            if j.get("type") == "fixed":
                continue
            parent = j["parent"]
            child = j["child"]
            if parent not in world:
                continue
            Mp = world[parent]
            origin_local = as_vec3(j["origin"])
            axis_local = normalize(as_vec3(j.get("axis", [0, 0, 1])))
            pivot = transform_point(Mp, origin_local)
            axis_w = normalize(Mp[:3, :3] @ axis_local)
            half = axis_w * (axis_length_m * 0.5)
            color = link_colors.get(child, {}).get("rgb", [1, 1, 0])
            markers.append(
                {
                    "id": j["id"],
                    "kind": "pivot",
                    "position": pivot.tolist(),
                    "joint_type": j.get("type"),
                    "parent": parent,
                    "child": child,
                    "color": color,
                    "radius_m": 0.012,
                }
            )
            axes.append(
                {
                    "id": j["id"],
                    "kind": "axis",
                    "a": (pivot - half).tolist(),
                    "b": (pivot + half).tolist(),
                    "direction": axis_w.tolist(),
                    "parent": parent,
                    "child": child,
                    "color": color,
                }
            )
        frame = desc.frame
    else:
        # CAD tree world
        for j in tree.joints:
            if j.joint_type == JointType.FIXED:
                continue
            pivot = as_vec3(j.origin_world)
            axis_w = normalize(as_vec3(j.axis_world))
            half = axis_w * (axis_length_m * 0.5)
            color = link_colors.get(j.child, {}).get("rgb", [1, 1, 0])
            markers.append(
                {
                    "id": j.id,
                    "kind": "pivot",
                    "position": pivot.tolist(),
                    "joint_type": j.joint_type.value,
                    "parent": j.parent,
                    "child": j.child,
                    "color": color,
                    "radius_m": 0.012,
                }
            )
            axes.append(
                {
                    "id": j.id,
                    "kind": "axis",
                    "a": (pivot - half).tolist(),
                    "b": (pivot + half).tolist(),
                    "direction": axis_w.tolist(),
                    "parent": j.parent,
                    "child": j.child,
                    "color": color,
                }
            )
        frame = "cad_z_up"

    overlay = {
        "frame": frame,
        "base_link": tree.base_link if desc is None else desc.base_link,
        "link_colors": link_colors,
        "markers": markers,
        "axes": axes,
        "legend": [
            {"id": lid, "hex": link_colors[lid]["hex"]} for lid in sorted(link_colors)
        ],
    }
    write_json(out / "debug_overlay.json", overlay)
    if desc is not None:
        # Also next to robot for viewer sync
        pass
    return overlay


def _rgb_hex(rgb: tuple[float, float, float]) -> str:
    return "#{:02x}{:02x}{:02x}".format(
        int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255)
    )


def _godot_rest_world(desc: RobotDesc) -> dict[str, np.ndarray]:
    base = desc.base_link
    children: dict[str, list[dict]] = {}
    for j in desc.joints:
        children.setdefault(j["parent"], []).append(j)
    world: dict[str, np.ndarray] = {base: mat4_identity()}
    stack = [base]
    while stack:
        pid = stack.pop()
        Mp = world[pid]
        for j in children.get(pid, []):
            cid = j["child"]
            origin = as_vec3(j["origin"])
            Mc = mat4_from_rt(np.eye(3), origin)
            world[cid] = Mp @ Mc
            stack.append(cid)
    return world
