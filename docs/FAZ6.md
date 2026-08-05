# Faz 6 — Closed Chain Robots

## Durum

**Tamamlandı (scaffolding).** `CURRENT_PHASE = 6`

## Ne var

- `hierarchy.parallel.detect_parallel_loops` — joint graph’ta undirected cycle tespiti
- `kinematic_tree.meta.parallel_loops` — Delta/paralel adayları
- Validation: cycle varsa `parallel_cycle` info (kapalı zincir solver yok)

## Ne yok (bilinçli)

Kapalı zincir IK / loop-closure constraint solver. Serial/SCARA doğruluğu bozulmadan ayrı solver gerektirir.

## Tasarım

Önce cycle’ları **tespit et ve raporla**; yanlış ağaca zorla spanning-tree basma. Sonraki iterasyon: loop closure + passive joint’ler.

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz6.py -q
```
