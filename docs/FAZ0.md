# Faz 0 — Temel Altyapı

## Durum

**Tamamlandı.** `CURRENT_PHASE = 0`

## Ne üretir

| Dosya | İçerik |
|-------|--------|
| `assembly_ir.json` | Parçalar, placement, bbox, hacim |
| `meshes/*.glb` | Ham part mesh’leri |
| `robot.json` | Link listesi + mesh path; **joints = []** |
| `geometry.json` | Placeholder (Faz 1’de dolacak) |
| `decision_trace.json` | Boş (Faz 2’de dolacak) |
| `validation_report.json` | Dosya/mesh varlık kontrolü |

## CLI

```bash
# Sentetik (sistem Python)
python pipeline.py run fixtures/serial_3dof.synthetic.json --out out/faz0

# Gerçek STEP (FreeCAD Python)
./run_with_freecad.sh run /path/to/robot.step --out out/faz0

python pipeline.py import … --out out/
python pipeline.py export --from-dir out/ --out out/
python pipeline.py validate --from-dir out/
python pipeline.py --phase-info
```

`analyze` bu fazda **kapalı** (`require_phase(1)`).

## Tasarım seçimleri (neden)

1. **`src/` paketleri** — Roadmap modülleri (`importer`, `geometry`, …) tek sorumlulukla ayrıldı; fazlar birbirini bozmadan büyüsün.
2. **`CURRENT_PHASE` kapısı** — Bir sonraki fazın kodu repoda hazır olsa bile CLI onu çalıştırmaz; “önce doğruluk, sonra genelleme” kuralı.
3. **Phase 0’da joint yok** — Ham STEP→mesh doğrulanmadan pivot tahmin etmek hatalı kinematiği gizler.
4. **Ortak IR (`common.models`)** — Sonraki fazlar aynı `AssemblyIR` / `RobotDesc` sözleşmesini genişletir; schema’lar `schemas/` altında kilitli.
5. **FreeCAD ayrı interpreter** — `./run_with_freecad.sh` FreeCAD.app Python 3.11 kullanır; sistem Python 3.14 FreeCAD `.so` yükleyemez.

## Godot

`godot/addons/cad_robot_importer` `robot.json` okur; `joints` boşsa tüm link mesh’lerini kök altına düz yükler. Hareket yok.

## Test

```bash
unset PYTHONPATH
python -m pytest tests/test_faz0.py -q
```
