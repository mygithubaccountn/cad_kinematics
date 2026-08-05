# Architecture

## Modules

| Stage | Package | Responsibility |
|-------|---------|----------------|
| 01 | `pipeline.s01_import` | STEP/synthetic → `AssemblyIR` |
| 02 | `pipeline.s02_geometry` | Cylinders, concentric clusters, contacts, adjacency |
| 03 | `pipeline.s03_mesh` | Part/link triangle meshes → GLB |
| 04 | `pipeline.s04_joint_detection` | Revolute/prismatic hypotheses + matching |
| 05 | `pipeline.s05_axis_detection` | Pivot/axis refinement |
| 06 | `pipeline.s06_hierarchy` | Base selection, welds, kinematic tree, loop flags |
| 07 | `pipeline.s07_scene_generation` | `robot.json` + link GLBs (glTF Y-up) |
| 08 | `pipeline.s08_validation` | Structure, pivot, rest-pose, smoke spin |

## Data flow

`AssemblyIR` → `FeatureGraph` → `JointHypothesis[]` → `ResolvedJoint[]` → `KinematicTree` → `RobotDesc`

Godot reads only `RobotDesc` (`robot.json`) and meshes.

## FreeCAD boundary

All FreeCAD/OCC calls live in:

- `s01_import/freecad_backend.py`
- `s02_geometry/freecad_faces.py`

Synthetic fixtures allow full CI without FreeCAD.

## Frames

Internal CAD math is **metres, Z-up**. `common/frames.py` converts to glTF/Godot **Y-up** at scene export.
