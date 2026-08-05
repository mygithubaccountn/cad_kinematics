# Faz 1 — Geometri Analizi

## Durum

**Tamamlandı.** `CURRENT_PHASE = 1`

## Ne üretir (Faz 0 +)

| Dosya | İçerik |
|-------|--------|
| `geometry.json` / `features.json` | Silindirler, düzlemler, concentric cluster’lar, contact, adjacency |
| `robot.json` | Hâlâ joints=[] (kinematik Faz 3) |

## CLI

```bash
python pipeline.py run fixtures/serial_3dof.synthetic.json --out out/faz1
# veya adım adım:
python pipeline.py import … --out out/
python pipeline.py analyze --from-dir out/ --out out/
python pipeline.py export --from-dir out/ --out out/
python pipeline.py validate --from-dir out/
```

## Tasarım seçimleri

1. **OCC/FreeCAD face classification** — Silindir radius/axis/center doğrudan BRep’ten; mesh’ten silindir uydurmak daha gürültülü.
2. **Inner/outer cylinder** — Normal vs radial test; shaft-in-hole (Faz 2) için gerekli.
3. **Concentric clustering** — Aynı mekanik eksendeki mil+delik çiftlerini birleştirir; tek silindiri joint sanmayı azaltır.
4. **Contact/adjacency graph** — Mate olmayan STEP’lerde parent/joint adaylığı için part komşuluğu.
5. **Analyze ≠ export** — Geometri IR’si kinematikten ayrıldı; yanlış joint’ler mesh export’u kirletmesin.

## Test

```bash
PYTHONPATH=src:. python -m pytest tests/test_faz1.py -q
```
