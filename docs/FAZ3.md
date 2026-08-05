# Faz 3 — Hierarchy + Godot

## Durum

**Tamamlandı.** `CURRENT_PHASE = 3`

## Ne üretir

| Dosya | İçerik |
|-------|--------|
| `kinematic_tree.json` | Base, links, joints (parent/child, local origin/axis) |
| `robot.json` | Godot sözleşmesi — **joints dolu** |
| `meshes/*.glb` | Link-local mesh’ler |

## Godot

Importer `robot.json` okur; child node origin = pivot; `rotate_object_local(axis, angle)`. Manuel pivot yok.

## Tasarım

Godot hesaplamaz — pipeline hazır frame üretir. Hierarchy: weld + joint spanning tree, base = hacim/ground/joint-connected.

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz3.py -q
```
