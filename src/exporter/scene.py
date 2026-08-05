"""07_scene_generation — robot.json + meshes for Godot."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from common.frames import cad_z_up_to_gltf_y_up
from common.io_util import write_json
from common.math3d import as_vec3, normalize
from common.models import AssemblyIR, KinematicTree, RobotDesc
from common.tolerances import Tolerances
from common.upright import upright_from_base, upright_mat4
from exporter import run_mesh_export


def run_scene_generation(
    ir: AssemblyIR,
    tree: KinematicTree,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    name: str = "robot",
    to_gltf_y_up: bool = False,
    *,
    export_meshes: bool = False,
    mesh_quality: str = "preview",
) -> RobotDesc:
    """
    Build robot.json (+ optional meshes).

    Default ``export_meshes=False``: kinematics-only package (paths declared, no tessellate/GLB).
    ``mesh_quality``: ``preview`` (fast CAD view) or ``final`` (fine export).
    """
    from common.timing import timed

    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    R, t = upright_from_base(ir, tree)
    M_up = upright_mat4(R, t)

    mapping = run_mesh_export(
        ir,
        out,
        tolerances,
        tree=tree,
        to_gltf_y_up=to_gltf_y_up,
        world_align=M_up,
        tessellate=export_meshes,
        write_glb=export_meshes,
        quality=mesh_quality if mesh_quality in ("preview", "final") else "preview",
    )

    with timed("Index/package"):
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

        T_yup = cad_z_up_to_gltf_y_up() if to_gltf_y_up else np.eye(4)
        joints_out = []
        rest_pose = {"joints": {}}
        for j in sorted(tree.joints, key=lambda x: x.id):
            # Parent-local translation-only frames: rotate vectors by R
            origin = R @ as_vec3(j.origin_local)
            axis = normalize(R @ as_vec3(j.axis_local))
            if to_gltf_y_up:
                Ry = T_yup[:3, :3]
                origin = Ry @ origin
                axis = normalize(Ry @ axis)
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
                "placement": "upright_z_up_base_on_ground",
                "upright_R": R.tolist(),
                "upright_t": t.tolist(),
                "mesh_quality": mesh_quality if export_meshes else "none",
            },
        )
        write_json(out / "robot.json", desc.to_dict())
        write_json(
            out / "upright.json",
            {"R": R.tolist(), "t": t.tolist(), "mat4": M_up.tolist()},
        )
    return desc
