"""07_scene_generation — robot.json + meshes for Godot."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.frames import cad_z_up_to_gltf_y_up
from pipeline.common.io_util import write_json
from pipeline.common.math3d import (
    as_vec3,
    invert_mat4,
    mat4_from_list,
    mat4_to_list,
    normalize,
    transform_dir,
    transform_point,
)
from pipeline.common.models import AssemblyIR, KinematicTree, RobotDesc
from pipeline.common.tolerances import Tolerances
from pipeline.s03_mesh import run_mesh_export


def run_scene_generation(
    ir: AssemblyIR,
    tree: KinematicTree,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    name: str = "robot",
    to_gltf_y_up: bool = True,
) -> RobotDesc:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Export link meshes in CAD link-local frames first
    mapping = run_mesh_export(ir, out, tolerances, tree=tree)

    T = cad_z_up_to_gltf_y_up() if to_gltf_y_up else np.eye(4)

    links_out = []
    for link in sorted(tree.links, key=lambda l: l.id):
        mesh = mapping.get(link.id, link.mesh_path)
        links_out.append(
            {
                "id": link.id,
                "name": link.name,
                "mesh": mesh,
                "part_ids": list(link.part_ids),
            }
        )

    joints_out = []
    rest_pose = {"joints": {}}
    for j in sorted(tree.joints, key=lambda x: x.id):
        # Convert origin/axis from parent-local CAD to parent-local glTF if needed
        origin = as_vec3(j.origin_local)
        axis = normalize(as_vec3(j.axis_local))
        if to_gltf_y_up:
            # Parent local CAD → apply only rotation part of T (same for all locals)
            R = T[:3, :3]
            origin = R @ origin
            axis = normalize(R @ axis)
        joints_out.append(
            {
                "id": j.id,
                "name": j.name,
                "parent": j.parent,
                "child": j.child,
                "type": j.joint_type.value,
                "origin": origin.tolist(),
                "axis": axis.tolist(),
                "confidence": j.confidence,
            }
        )
        rest_pose["joints"][j.id] = 0.0

    # Child node transform relative to parent for rest pose (Godot):
    # For each joint, local transform places child frame at origin with identity rotation
    # relative to parent — meshes already in child link frame.
    # Godot importer: parent Node3D, child Node3D at `origin`, rotation about `axis`.

    desc = RobotDesc(
        name=name,
        frame="gltf_y_up" if to_gltf_y_up else "cad_z_up",
        base_link=tree.base_link,
        links=links_out,
        joints=joints_out,
        rest_pose=rest_pose,
        meta={
            "source": ir.source_path,
            "pipeline": "cad-godot-robot-pipeline",
            "kinematic_meta": tree.meta,
        },
    )
    write_json(out / "robot.json", desc.to_dict())
    return desc
