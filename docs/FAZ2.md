# Faz 2 — Joint Detection

## Durum

**Tamamlandı.** `CURRENT_PHASE = 2`

## Ne üretir

| Dosya | İçerik |
|-------|--------|
| `joints_selected.json` | Seçilen joint hipotezleri |
| `resolved_axes.json` | Pivot + axis refinement |
| `decision_trace.json` | Her joint için evidence skorları |
| `robot.json` | Hâlâ joints=[] (Godot ağacı Faz 3) |

## Multi-hypothesis skorlar

- concentric shared cylinders
- shaft-in-hole (inner/outer radius)
- contact ring
- mate hints (varsa)
- placement align (zayıf)

## Tasarım

Tek heuristic yok. Skor füzyonu + `DecisionTrace` zorunlu. Placement/bbox birincil pivot kaynağı değil. Prismatic hipotezleri kodda var ama **Faz 5’e kadar CLI’da kapalı** (`include_prismatic=False`).

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz2.py -q
```
